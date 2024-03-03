from migen import *
from migen.genlib.cdc import MultiReg
from litex.soc.interconnect import stream
from litex.gen.genlib.misc import WaitTimer
from litex.soc.interconnect.csr import *

from modules.misc import core_layout


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


class HSPITransmitter(Module):
    def __init__(self, pads, data_in_width):
        self.sink = sink = stream.Endpoint(core_layout(data_in_width))
        self.hd = hd = self.add_tristate(pads.hd) if not hasattr(pads.hd, "oe") else pads.hd
        hd_width = len(hd.o)
        assert hd_width in [8, 16, 32]
        assert data_in_width in [4, 8, 16, 32]
        assert hd_width % data_in_width == 0
        assert hd_width >= data_in_width  # TODO: allow more diverse configurations
        crc_size = 32 if hd_width == 32 else 16

        self.submodules.crc = crc = CRC(polynomial=0x8005, crc_size=crc_size, datawidth=hd_width, delay=False)

        header =            Signal(32)
        header_reg =        Signal(32)
        tll_2b_in =         Signal(2, reset={8: 0, 16: 1, 32: 2}[hd_width])
        sequence_nr_in =    Signal(4)
        user_id_in =        Signal(26, reset=0xA5B6C7D8)
        word_index = Signal(max=4095)
        self.crc_r = Signal(crc_size)

        self.comb += [
            pads.clk.eq(ClockSignal("sys")),
            header.eq(Cat(user_id_in, sequence_nr_in, tll_2b_in))
        ]

        self.header_timer = header_timer = WaitTimer((hd_width / data_in_width) + 1)
        self.crc_timer = crc_timer = WaitTimer(1 if crc_size > hd_width else 0)
        self.submodules += header_timer, crc_timer
        self.header_reg = header_reg

        # FSM
        self.fsm = fsm = FSM(reset_state="WAIT_INPUT")
        self.submodules += fsm
        self.state = Signal(max=5)
        fsm.act("WAIT_INPUT",
            self.state.eq(0),
            If(sink.valid,
               pads.req.eq(1),
               NextValue(header_reg, header),
               NextState("WAIT_TX_READY")
            )
        )

        fsm.act("WAIT_TX_READY",
            self.state.eq(1),
            pads.req.eq(1),
            If(pads.ready,
               NextState("TX_HEADER")
            )
        )

        fsm.act("TX_HEADER",
                self.state.eq(2),
                self.header_timer.wait.eq(1),
                hd.oe.eq(1),
                hd.o.eq(header_reg[:hd_width]),
                pads.valid.eq(1),
                crc.data_in.eq(header_reg[:hd_width]),
                crc.enable_in.eq(1),
                NextValue(header_reg, header_reg >> hd_width),
                If(self.header_timer.done,
                    NextState("TX_DATA")
                )
        )

        fsm.act("TX_DATA",
                self.state.eq(3),
                self.sink.ready.eq(1),
                crc.data_in.eq(sink.data),
                crc.enable_in.eq(1),
                pads.valid.eq(1),
                hd.o.eq(sink.data),
                hd.oe.eq(1),
                If(sink.valid,
                    NextValue(word_index, word_index + 1)
                ),
                If(sink.last | (word_index == 4095) | ~sink.valid,
                    NextState("TX_CRC"),
                    NextValue(self.crc_r, crc.crc_out),
                )
        )

        fsm.act("TX_CRC",
                self.state.eq(4),
                self.crc_timer.wait.eq(1),
                pads.valid.eq(1),
                hd.o.eq(self.crc_r[:hd_width]),
                NextValue(self.crc_r, self.crc_r >> hd_width),
                hd.oe.eq(1),
                If(self.crc_timer.done,
                    NextState("WAIT_TX_READY_OVER")
                )
        )

        fsm.act("WAIT_TX_READY_OVER",
                self.state.eq(5),
                pads.valid.eq(0),
                pads.req.eq(0),
                hd.oe.eq(0),
                If(~pads.ready,
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
