from migen import *
from migen.genlib.cdc import MultiReg
from litex.soc.interconnect.csr import AutoCSR, CSR, CSRStorage, CSRStatus
from litex.soc.interconnect import stream
from litex.gen.genlib.misc import WaitTimer

from modules.misc import core_layout


class Trigger(Module, AutoCSR):
    def __init__(self, data_width, depth=16):
        self.sink   = sink   = stream.Endpoint(core_layout(data_width))
        self.source = source = stream.Endpoint(core_layout(data_width))

        self.enable = CSRStorage()
        self.done   = CSRStatus()

        self.mem_write = CSR()
        self.mem_mask  = CSRStorage(data_width)
        self.mem_value = CSRStorage(data_width)
        self.mem_full  = CSRStatus()

        # # #

        # Control re-synchronization
        enable   = Signal()
        enable_d = Signal()
        self.specials += MultiReg(self.enable.storage, enable, "scope")
        self.sync.scope += enable_d.eq(enable)

        # Status re-synchronization
        done = Signal()
        self.specials += MultiReg(done, self.done.status)

        # Memory and configuration
        mem = stream.AsyncFIFO([("mask", data_width), ("value", data_width)], depth)
        mem = ClockDomainsRenamer({"write": "sys", "read": "scope"})(mem)
        self.submodules += mem
        self.comb += [
            mem.sink.valid.eq(self.mem_write.re),
            mem.sink.mask.eq(self.mem_mask.storage),
            mem.sink.value.eq(self.mem_value.storage),
            self.mem_full.status.eq(~mem.sink.ready)
        ]

        # Hit and memory read/flush
        hit   = Signal()
        flush = WaitTimer(2*depth)
        flush = ClockDomainsRenamer("scope")(flush)
        self.submodules += flush
        self.comb += [
            flush.wait.eq(~(~enable & enable_d)), # flush when disabling
            hit.eq((sink.data & mem.source.mask) == (mem.source.value & mem.source.mask)),
            mem.source.ready.eq((enable & hit) | ~flush.done),
        ]

        # Output
        self.comb += [
            sink.connect(source),
            # Done when all triggers have been consumed
            done.eq(~mem.source.valid),
            source.hit.eq(done)
        ]
