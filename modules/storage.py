from migen import *
from migen.genlib.cdc import MultiReg
from litex.soc.interconnect.csr import AutoCSR, CSRStorage, CSRStatus
from litex.soc.interconnect import stream
from litex.gen.genlib.misc import WaitTimer

from modules.misc import core_layout


class Storage(Module, AutoCSR):
    def __init__(self, data_width, depth):
        self.sink = sink = stream.Endpoint(core_layout(data_width))
        self.source = source = stream.Endpoint([("data", data_width)])

        self.enable    = CSRStorage()
        self.done      = CSRStatus()
        self.fsm_state_r = CSRStatus()

        self.length    = CSRStorage(bits_for(depth))
        self.offset    = CSRStorage(bits_for(depth))

        read_width = min(32, data_width)
        self.mem_level = CSRStatus(bits_for(depth))
        self.mem_data  = CSRStatus(read_width)

        # # #

        # Control re-synchronization
        enable   = Signal()
        enable_d = Signal()
        self.specials += MultiReg(self.enable.storage, enable, "scope")
        self.sync.scope += enable_d.eq(enable)

        length = Signal().like(self.length.storage)
        offset = Signal().like(self.offset.storage)
        self.specials += MultiReg(self.length.storage, length, "scope")
        self.specials += MultiReg(self.offset.storage, offset, "scope")

        # Status re-synchronization
        done  = Signal()
        level = Signal().like(self.mem_level.status)
        self.specials += MultiReg(done, self.done.status)
        self.specials += MultiReg(level, self.mem_level.status)

        # Memory
        mem = stream.SyncFIFO([("data", data_width)], depth, buffered=True)
        mem = ClockDomainsRenamer("scope")(mem)
        self.submodules += mem

        self.comb += level.eq(mem.level)

        # Flush
        mem_flush = WaitTimer(depth)
        self.mem_flush = mem_flush = ClockDomainsRenamer("scope")(mem_flush)
        self.submodules += mem_flush

        self.fsm_state = Signal(max=3)
        self.specials += MultiReg(self.fsm_state, self.fsm_state_r.status)

        # FSM
        fsm = FSM(reset_state="IDLE")
        self.fsm = fsm = ClockDomainsRenamer("scope")(fsm)
        self.submodules += fsm
        fsm.act("IDLE",
            self.fsm_state.eq(0),
            done.eq(1),
            If(enable & ~enable_d,
                NextState("FLUSH")
            ),
            sink.ready.eq(1),
        )
        fsm.act("FLUSH",
            self.fsm_state.eq(1),
            sink.ready.eq(1),
            mem_flush.wait.eq(1),
            mem.source.ready.eq(1),
            If(mem_flush.done,
                NextState("WAIT")
            )
        )
        fsm.act("WAIT",
            self.fsm_state.eq(2),
            sink.connect(mem.sink, omit={"hit"}),
            If(sink.valid & sink.hit,
                NextState("RUN")
            ),
            mem.source.ready.eq(mem.level >= offset)
        )
        fsm.act("RUN",
            self.fsm_state.eq(3),
            sink.connect(mem.sink, omit={"hit"}),
            If((mem.level >= length) | (~enable & enable_d),
                NextState("IDLE"),
            )
        )

        self.comb += [
            mem.source.connect(source)
        ]
