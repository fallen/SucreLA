#!/usr/bin/env python3

from litex_boards.platforms.gsd_orangecrab import Platform, feather_serial
from litex_boards.targets.gsd_orangecrab import BaseSoC
from litex.build.generic_platform import *
from litex.soc.integration.builder import *

from modules.la import LA, LATest

feather_gpio = [
    ("gpios", 0, Pins("GPIO:2 GPIO:3 GPIO:5 GPIO:6"), IOStandard("LVCMOS33")
    )
]

feather_hspi = [
    ("hspi", 0,
     Subsignal("tx_clk", Pins("GPIO:9"), IOStandard("LVCMOS33")),
     Subsignal("tx_req", Pins("GPIO:10"), IOStandard("LVCMOS33")),
     Subsignal("tx_ready", Pins("GPIO:11"), IOStandard("LVCMOS33")),
     Subsignal("tx_valid", Pins("GPIO:12"), IOStandard("LVCMOS33")),
     Subsignal("hd", Pins("GPIO:13 GPIO:14 GPIO:15 GPIO:16 GPIO:18 GPIO:19 GPIO:20 GPIO:21"),
               IOStandard("LVCMOS33")),
    )
]


class LASoC(BaseSoC):
    def __init__(self, sys_clk_freq=48e6, **kwargs):
        BaseSoC.__init__(self, sys_clk_freq=sys_clk_freq,
                         with_led_chaser=False, **kwargs)

        self.platform.add_extension(feather_serial)
        self.platform.add_extension(feather_gpio)
        self.platform.add_extension(feather_hspi)
        self.add_uartbone()
        ios = self.platform.request("gpios")
        hspi_pads = self.platform.request("hspi")
        self.submodules.la = LA(ios, hspi_pads, depth=1024, clock_domain="sys", samplerate=sys_clk_freq)


def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=Platform, description="SucreLA on OrangeCrab.")
    parser.add_target_argument("--sys-clk-freq",    default=48e6, type=float, help="System clock frequency.")
    parser.add_target_argument("--revision",        default="0.2",            help="Board Revision (0.1 or 0.2).")
    parser.add_target_argument("--device",          default="25F",            help="ECP5 device (25F, 45F or 85F).")
    parser.add_target_argument("--sdram-device",    default="MT41K64M16",     help="SDRAM device (MT41K64M16, MT41K128M16, MT41K256M16 or MT41K512M16).")
    parser.add_target_argument("--sim",             action="store_true", help="run LA simulation testsuite")
    parser.set_defaults(no_uart=True, cpu_type="None")
    args = parser.parse_args()

    if args.sim:
        LATest()
        exit(0)

    soc = LASoC(sys_clk_freq=args.sys_clk_freq, **parser.soc_argdict)

    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))


if __name__ == "__main__":
    main()
