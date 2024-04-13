from migen import *
from litex.soc.interconnect.csr import AutoCSR, CSRStorage
from litex.soc.interconnect import stream

from modules.trigger import Trigger
from modules.subsampler import SubSampler
from modules.storage import Storage
from modules.hspi import HSPITransmitter, HSPIPacket


class LA(Module, AutoCSR):
    def __init__(self, ios, hspi_pads, depth, samplerate=1e12, clock_domain="sys", trigger_depth=16, register=False,
                 csr_csv="analyzer.csv", testing=False):
        self.ios = ios
        self.depth      = depth
        self.samplerate = int(samplerate)
        self.data_width = data_width = len(ios)
        self.fake_data = Signal(data_width)
        self.csr_csv = csr_csv
        assert len(ios) in [4, 8, 16]
        hspi_data_width = {4: 8,
         8: 16,
         16: 32}[len(ios)]

        # Frontend
        self.submodules.trigger = ClockDomainsRenamer({"scope": clock_domain})(Trigger(data_width, depth=trigger_depth))
        self.submodules.subsampler = ClockDomainsRenamer({"scope": clock_domain})(SubSampler(data_width))

        # Storage buffer
        self.submodules.storage = ClockDomainsRenamer({"scope": clock_domain})(Storage(data_width, depth))

        # Upconverter
        self.submodules.converter = ClockDomainsRenamer(clock_domain)(stream.Converter(len(ios), hspi_data_width))

        # SyncFIFO to buffer a HSPI packet
        self.submodules.hspi_packet_buffer = ClockDomainsRenamer(clock_domain)(stream.SyncFIFO([("data",
                                                                                                 hspi_data_width)],
                                                                                               depth=0x2000))
        # HSPI TX
        self.submodules.hspi_tx = ClockDomainsRenamer({"scope": clock_domain})(HSPITransmitter(hspi_pads, hspi_data_width))

        scope_sync = getattr(self.sync, clock_domain)
        scope_sync += [
            If(~self.hspi_tx.enable.storage,
                self.fake_data.eq(0)
            ).Else(
                self.fake_data.eq(self.fake_data + 1)
            )
        ]

        self.comb += self.trigger.sink.valid.eq(1)

        if testing:
            self.comb += self.trigger.sink.data.eq(self.fake_data)
        else:
            self.comb += self.trigger.sink.data.eq(ios)

        # Pipeline
        self.submodules.pipeline = stream.Pipeline(
            self.trigger.source,
            self.subsampler,
            self.storage,
            self.converter,
            self.hspi_packet_buffer,
        )

        self.packet_is_buffered = Signal()

        self.comb += [
            self.packet_is_buffered.eq(~self.hspi_tx.fsm.ongoing("WAIT_INPUT") |
                                       (self.hspi_packet_buffer.level >= self.hspi_tx.max_packet_size.storage)),
            If(self.packet_is_buffered,
               self.pipeline.source.connect(self.hspi_tx.sink)
            ).Else(
                self.hspi_tx.sink.valid.eq(0)
            )
        ]


class LATest(Module):
    def __init__(self, stop_via="max_packet_size", inject_not_ready=False):
        hd_layout = [("oe", 1), ("o", 8), ("i", 8)]
        self.pads = Record([("hd", hd_layout),
                            ("clk", 1),
                            ("ready", 1),
                            ("valid", 1),
                            ("req", 1),
                            ("hract", 1),
                            ("htack", 1)
                            ])
        self.ios = Signal(4)
        self.submodules.la = LA(self.ios, self.pads, depth=64, clock_domain="sys", samplerate=48e6, testing=True)

        def handle_data(data, buffer):
            assert data in range(256)
            buffer.extend(data.to_bytes(1, "little"))

        def wait_for_tx_req():
            while True:
                tx_req = yield self.pads.req
                if tx_req:
                    print("Detected TX_REQ")
                    break
                yield

        def simulate():
            yield
            yield from self.la.storage.length.write(64)
            yield
            yield from self.la.hspi_tx.enable.write(1)
            yield
            yield from self.la.subsampler.value.write(0)
            yield
            yield from self.la.hspi_tx.max_packet_size.write(50)
            yield
            yield
            yield
            yield
            yield from self.la.storage.enable.write(1)
            yield from wait_for_tx_req()

            yield self.pads.ready.eq(1)
            for i in range(100):
                tx_valid = yield self.pads.valid
                if tx_valid == 1:
                    print("Detected TX_VALID")
                    break
                yield

            data_buffer = bytearray()
            for i in range(200):
                if i == 40:
                    if stop_via == "hspi":
                        yield self.la.hspi_tx.enable.storage.eq(0)
                    elif stop_via == "storage":
                        yield self.la.storage.enable.storage.eq(0)
                tx_valid = yield self.pads.valid
                tx_req = yield self.pads.req
                hd_oe = yield self.pads.hd.oe
                if (i == 42) and inject_not_ready:
                    yield self.pads.ready.eq(0)
                if (i == 47) and inject_not_ready:
                    yield self.pads.ready.eq(1)
                ready = yield self.pads.ready
                if tx_valid == 0:
                    print("TX_VALID is gone (i={})!".format(i))
                if ready == 0:
                    print("RX_READY is gone (i={})!".format(i))
                if hd_oe == 0:
                    print("hd.oe is gone (i={})!".format(i))
                if tx_valid & ready & hd_oe:
                    data = yield self.pads.hd.o
                    handle_data(data, data_buffer)
                    print("{}: Transmitted 0x{:02x}".format(i, data))
                if tx_req == 0:
                    print("TX_REQ is gone (i={})!".format(i))
                    print("total buffer (len {}): {}".format(len(data_buffer), data_buffer))
                    break
                yield

            HSPIPacket(data_buffer)

        print("Running sim!")
        run_simulation(self, simulate(), vcd_name="sim.vcd")