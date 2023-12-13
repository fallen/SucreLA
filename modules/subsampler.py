from migen import *
from migen.genlib.cdc import MultiReg
from litex.soc.interconnect.csr import AutoCSR, CSRStorage
from litex.soc.interconnect import stream

from modules.misc import core_layout


class SubSampler(Module, AutoCSR):
    def __init__(self, data_width):
        self.sink   = sink   = stream.Endpoint(core_layout(data_width))
        self.source = source = stream.Endpoint(core_layout(data_width))

        self.value = CSRStorage(16)

        # # #

        value = Signal(16)
        self.specials += MultiReg(self.value.storage, value, "scope")

        counter = Signal(16)
        done    = Signal()
        self.sync.scope += \
            If(source.ready,
                If(done,
                    counter.eq(0)
                ).Elif(sink.valid,
                    counter.eq(counter + 1)
                )
            )

        self.comb += [
            done.eq(counter == value),
            sink.connect(source, omit={"valid"}),
            source.valid.eq(sink.valid & done)
        ]
