from migen import *
from litex.soc.interconnect.csr import AutoCSR, CSRStorage
from litex.soc.interconnect import stream

from modules.trigger import Trigger
from modules.subsampler import SubSampler
from modules.storage import Storage
from modules.hspi import HSPITransmitter, HSPIPacket


class LA(Module, AutoCSR):
    def __init__(self, ios, hspi_pads, depth, samplerate=1e12, clock_domain="sys", trigger_depth=16, register=False,
                 csr_csv="analyzer.csv"):
        self.ios = ios
        self.depth      = depth
        self.samplerate = int(samplerate)
        self.data_width = data_width = len(ios)
        self.csr_csv = csr_csv

        self.hspi_enable = CSRStorage(1, reset=0)
        self.hspi_test_pattern = CSRStorage(data_width)

        # Frontend
        self.submodules.trigger = ClockDomainsRenamer({"scope": "sys"})(Trigger(data_width, depth=trigger_depth))
        self.submodules.subsampler = ClockDomainsRenamer({"scope": "sys"})(SubSampler(data_width))

        # Storage buffer
        self.submodules.storage = ClockDomainsRenamer({"scope": "sys"})(Storage(data_width, depth))

        # HSPI TX
        self.submodules.hspi_tx = HSPITransmitter(hspi_pads, data_width)

        self.comb += [
            self.trigger.sink.valid.eq(1),
            self.trigger.sink.data.eq(ios)
        ]

        # Pipeline
        self.submodules.pipeline = stream.Pipeline(
            self.trigger.source,
            self.subsampler,
            self.storage,
            self.hspi_tx.sink)


class LATest(Module):
    def __init__(self):
        hd_layout = [("oe", 1), ("o", 8), ("i", 8)]
        self.pads = Record([("hd", hd_layout),
                            ("clk", 1),
                            ("ready", 1),
                            ("valid", 1),
                            ("req", 1),
                            ])
        self.ios = Signal(4)
        self.submodules.la = LA(self.ios, self.pads, depth=64, clock_domain="sys", samplerate=48e6)

        def handle_data(data, buffer):
            assert data in range(256)
            buffer.extend(data.to_bytes(1, "little"))

        def wait_for_tx_req():
            while True:
                tx_req = yield self.pads.req
                if tx_req:
                    print("Detected TX_REQ")
                    break
                yield self.ios.eq(self.ios + 1)
                yield

        def simulate():
            yield
            yield
            yield self.ios.eq(0)
            yield self.la.hspi_test_pattern.storage.eq(0x3)
            yield self.la.hspi_enable.storage.eq(1)
            yield self.la.storage.length.storage.eq(64)
            yield
            yield
            yield
            yield
            yield self.la.storage.enable.storage.eq(1)
            yield
            yield from wait_for_tx_req()

            yield self.pads.ready.eq(1)
            for i in range(100):
                yield self.ios.eq(self.ios + 1)
                tx_valid = yield self.pads.valid
                if tx_valid == 1:
                    print("Detected TX_VALID")
                    break
                yield

            data_buffer = bytearray()
            for i in range(4096):
                if i == 40:
                    yield self.la.storage.enable.storage.eq(0)
                tx_valid = yield self.pads.valid
                if tx_valid == 0:
                    print("TX_VALID is gone (i={})!".format(i))
                    print("total buffer: {}".format(data_buffer))
                    break
                data = yield self.pads.hd.o
                handle_data(data, data_buffer)
                yield self.ios.eq(self.ios + 1)
                yield
                print("Transmitted {}".format(data))

            p = HSPIPacket(data_buffer)

            #yield self.la.storage.enable.storage.eq(0)
            for i in range(100):
                yield self.ios.eq(self.ios + 1)
                yield
            yield self.la.hspi_enable.storage.eq(0)
            yield self.la.hspi_tx.sink.last.eq(1)
            for i in range(20):
                yield self.ios.eq(self.ios + 1)
                yield

        print("Running sim!")
        run_simulation(self, simulate(), vcd_name="sim.vcd")
