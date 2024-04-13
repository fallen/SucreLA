from migen import *
from migen.genlib.cdc import MultiReg
from litex.soc.interconnect import stream
from litex.gen.genlib.misc import WaitTimer
from litex.soc.interconnect.csr import *

from modules.misc import core_layout

import sys


class CRC(Module):
    def __init__(self, polynomial, crc_size, datawidth, init=None, delay=False):
        if init is None:
            init = (1 << crc_size) - 1

        self.reset_in  = Signal()
        self.enable_in = Signal()
        self.data_in   = Signal(datawidth)
        self.crc_out   = Signal(crc_size)

        crcreg = [Signal(crc_size, reset=init) for i in range(datawidth + 1)]

        for i in range(datawidth):
            inv = self.data_in[i] ^ crcreg[i][crc_size - 1]
            tmp = []
            tmp.append(inv)
            for j in range(crc_size - 1):
                if((polynomial >> (j + 1)) & 1):
                    tmp.append(crcreg[i][j] ^ inv)
                else:
                    tmp.append(crcreg[i][j])
            self.comb += crcreg[i + 1].eq(Cat(*tmp))

        self.sync += [
            If(self.reset_in,
                crcreg[0].eq(init)
            ).Elif(self.enable_in,
                crcreg[0].eq(crcreg[datawidth])
            )
        ]

        out_expr = self.crc_out.eq(crcreg[datawidth][::-1] ^ init)

        if delay:
            self.sync += out_expr
        else:
            self.comb += out_expr


class HSPITransmitter(Module, AutoCSR):
    def __init__(self, pads, data_in_width, hd_width=None):
        self.sink = sink = stream.Endpoint(([("data", data_in_width)]))
        self.hd = hd = self.add_tristate(pads.hd) if not hasattr(pads.hd, "oe") else pads.hd
        self.hd_width = len(hd.o) if hd_width is None else hd_width
        assert self.hd_width <= len(hd.o)
        assert self.hd_width in [8, 16, 32]
        assert data_in_width in [4, 8, 16, 32]
        assert self.hd_width % data_in_width == 0
        assert self.hd_width >= data_in_width  # TODO: allow more diverse configurations
        crc_size = 32 if self.hd_width == 32 else 16
        self.cd_scope = ClockDomain()

        crc = CRC(polynomial=0x8005, crc_size=crc_size, datawidth=self.hd_width, delay=False)
        self.crc = crc = ClockDomainsRenamer("scope")(crc)
        self.submodules.crc = crc

        header =            Signal(32)
        header_reg =        Signal(32)
        tll_2b_in =         Signal(2, reset={8: 0, 16: 1, 32: 2}[self.hd_width])
        sequence_nr_in =    Signal(4, reset=0)
        user_id_in =        Signal(26, reset=0xA5B6C7D8)
        word_index = Signal(max=4096)
        self.crc_r = Signal(crc_size)
        self.state = Signal(max=6)
        self.state_r = CSRStatus(len(self.state))
        self.specials += MultiReg(self.state, self.state_r.status)
        self.enable = CSRStorage(write_from_dev=True)
        self.enable_r = Signal()
        self.enable_r_d = Signal()
        self.enable_dropped = Signal()
        self.specials += MultiReg(self.enable.storage, self.enable_r, "scope")
        self.max_packet_size_r = Signal(max=4097, reset=4096)
        self.max_packet_size = CSRStorage(len(self.max_packet_size_r), reset=4096)
        self.specials += MultiReg(self.max_packet_size.storage, self.max_packet_size_r, "scope")
        self.not_ready_happened = Signal()
        self.max_packet_num = Signal(16)
        self.max_packet_num_r = CSRStorage(16)
        self.remaining_packets = Signal(16)
        self.specials += MultiReg(self.max_packet_num_r.storage, self.max_packet_num, "scope")
        self.hspi_pads_debug = Signal(len(Cat(pads.req, pads.ready, pads.valid, hd.o, hd.oe, sink.valid, sink.ready, sink.last, sink.data)))
        self.hspi_pads_csr = CSRStatus(len(self.hspi_pads_debug))
        self.specials += MultiReg(self.hspi_pads_debug, self.hspi_pads_csr.status)

        self.comb += [
            pads.clk.eq(ClockSignal("scope")),
            header.eq(Cat(user_id_in, sequence_nr_in, tll_2b_in)),
            self.enable_dropped.eq(self.enable_r_d & ~self.enable_r),
            self.hspi_pads_debug.eq(Cat(pads.req, pads.ready, pads.valid, hd.o, hd.oe, sink.valid, sink.ready, sink.last, sink.data))
        ]

        self.sync.scope += [
            self.enable_r_d.eq(self.enable_r)
        ]

        self.header_timer = header_timer = ClockDomainsRenamer("scope")(WaitTimer((32 / self.hd_width) - 1))
        self.crc_timer = crc_timer = ClockDomainsRenamer("scope")(WaitTimer(1 if crc_size > self.hd_width else 0))
        self.submodules += header_timer, crc_timer
        self.header_reg = header_reg

        # FSM
        fsm = FSM(reset_state="WAIT_INPUT")
        self.fsm = fsm = ClockDomainsRenamer("scope")(fsm)
        self.submodules += fsm
        fsm.act("WAIT_INPUT",
            self.state.eq(0),
            If(~self.enable_r,
               NextValue(self.remaining_packets, self.max_packet_num)
            ),
            self.sink.ready.eq(0),
            pads.req.eq(0),
            NextValue(word_index, 0),
            If(pads.hract,
               pads.htack.eq(1)
            ),
            If(sink.valid & self.enable_r & ~pads.hract,
               pads.req.eq(1),
               NextValue(header_reg, header),
               NextState("WAIT_TX_READY")
            )
        )

        fsm.act("WAIT_TX_READY",
            self.state.eq(1),
            pads.req.eq(1),
            self.sink.ready.eq(0),
            If(pads.ready & sink.valid,
               NextState("TX_HEADER")
            ).Elif(~sink.valid,
                   NextState("WAIT_INPUT")
            )
        )

        fsm.act("TX_HEADER",
                self.state.eq(2),
                self.header_timer.wait.eq(1),
                pads.req.eq(1),
                hd.oe.eq(1),
                hd.o.eq(header_reg[:self.hd_width]),
                pads.valid.eq(1),
                crc.data_in.eq(header_reg[:self.hd_width]),
                crc.enable_in.eq(1),
                self.sink.ready.eq(0),
                NextValue(header_reg, header_reg >> self.hd_width),
                If(~sink.valid,
                   NextState("WAIT_INPUT")
                ).Elif(self.header_timer.done,
                    NextState("TX_DATA")
                )
        )

        fsm.act("TX_DATA",
                self.state.eq(3),
                self.sink.ready.eq(pads.ready),
                pads.req.eq(1),
                crc.data_in.eq(sink.data),
                crc.enable_in.eq(sink.valid & pads.ready),
                pads.valid.eq(sink.valid),
                hd.o.eq(sink.data),
                hd.oe.eq(1),
                If(~pads.ready,
                   NextValue(self.not_ready_happened, 1),
                ),
                If(sink.valid & pads.ready & self.enable_r,
                    NextValue(word_index, word_index + 1)
                ),
                If((self.enable_dropped | sink.valid) & pads.ready,
                   NextValue(self.crc_r, crc.crc_out),
                ),
                If(sink.last | (word_index == self.max_packet_size_r - 1) | ~sink.valid | ~self.enable_r,
                    NextState("TX_CRC"),
                )
        )

        fsm.act("TX_CRC",
                self.state.eq(4),
                self.crc_timer.wait.eq(pads.ready),
                pads.req.eq(1),
                pads.valid.eq(1),
                self.sink.ready.eq(0),
                hd.o.eq(self.crc_r[:self.hd_width]),
                If(pads.ready,
                    NextValue(self.crc_r, self.crc_r >> self.hd_width)
                ),
                hd.oe.eq(1),
                If(self.crc_timer.done,
                    NextState("WAIT_TX_READY_OVER"),
                    NextValue(self.remaining_packets, self.remaining_packets - 1)
                )
        )

        fsm.act("WAIT_TX_READY_OVER",
                self.state.eq(5),
                self.sink.ready.eq(0),
                pads.valid.eq(0),
                pads.req.eq(0),
                hd.oe.eq(0),
                crc.reset_in.eq(1),
                If(~pads.ready,
                    pads.req.eq(0),
                    NextValue(sequence_nr_in, sequence_nr_in + 1),
                    If(~self.remaining_packets & self.max_packet_num,  # max_packet_num of 0 means infinity
                       self.enable.we.eq(1),
                       self.enable.dat_w.eq(0),
                    ),
                    NextState("WAIT_INPUT")
                )
        )

    def add_tristate(self, pad):
        t = TSTriple(len(pad))
        self.specials += t.get_tristate(pad)
        return t


class HSPIPacket:
    def __init__(self, data, hd_width=8):
        # Complete data of the packet
        self.data = data
        self.hd_width = hd_width
        self.crc_len = 4 if hd_width == 32 else 2

        if len(data) < 7:
            print("Packet is buggy, it should be at least 7 bytes long!")
            return

        # Packet header
        self.header = data[:4]
        # Packet payload (without header nor CRC)
        self.payload = data[4:-self.crc_len]
        # Packet CRC
        self.crc = data[-self.crc_len:]
        # TX length low 2 bits
        self.tll2b = (self.header[0] & 0xc0) >> 6 # 2 MSB
        # TX sequence number
        self.tsqn = (self.header[0] & 0x3c) >> 2 # 4 following bits
        # User self defined field
        self.usdf = ((self.header[0] & 0x3) << 25) | int.from_bytes(self.header[1:], "big") # 26 following bits
        self.check_crc()
        self.check_payload()

    def check_crc(self):
        from crc import Configuration, Calculator
        if self.crc_len == 2:
            config = Configuration(
                width=16,
                polynomial=0x8005,
                init_value=0xffff,
                final_xor_value=0xffff,
                reverse_input=True,
                reverse_output=True,
            )
        else:
            config = Configuration(
                width=32,
                polynomial=0x4C11DB7,
                init_value=0xffff,
                final_xor_value=0xffff,
                reverse_input=False,
                reverse_output=True,
            )
        calculator = Calculator(config)
        correct = calculator.verify(self.data[:-self.crc_len], int.from_bytes(self.crc, "little"))
        real_crc = calculator.checksum(self.data[:-self.crc_len])
        if correct:
            print("VALID CRC {} !".format(self.crc))
        else:
            print("@@ INVALID CRC @@ {} instead of {}".format(self.crc, hex(real_crc)))
            sys.exit(1)

    def check_payload(self):
        prev = None
        for d in self.payload:
            assert (((d & 0xf) + 1) & 0xf) == ((d & 0xf0) >> 4)
            if not prev and d != 0:
                prev = ((d & 0xf) - 1) << 4
            if prev:
                assert (((prev & 0xf0) >> 4) + 1) & 0xf == d & 0xf
            prev = d
        print("VALID payload!")
