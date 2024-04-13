#!/usr/bin/env python3

from migen import *

from litex_boards.platforms.gsd_orangecrab import Platform, feather_serial
from litex_boards.targets.gsd_orangecrab import BaseSoC
from litex.build.generic_platform import *
from litex.soc.integration.builder import *

from litescope import LiteScopeAnalyzer

from modules.la import LA, LATest

feather_gpio = [
    ("gpios", 0, Pins("GPIO:2 GPIO:3 GPIO:5 GPIO:6"), IOStandard("LVCMOS33")  # C10 C9 B10 B9 aka SDA SCL 5 6
    )
]

feather_hspi = [
    ("hspi", 0,
     Subsignal("clk", Pins("GPIO:9"), IOStandard("LVCMOS33")),  # C8 aka 9
     Subsignal("req", Pins("GPIO:10"), IOStandard("LVCMOS33")),  # B8 aka 10
     Subsignal("ready", Pins("GPIO:11"), IOStandard("LVCMOS33")),  # A8 aka 11
     Subsignal("valid", Pins("GPIO:12"), IOStandard("LVCMOS33")),  # H2 aka 12
     Subsignal("hd", Pins("GPIO:13 GPIO:14 GPIO:15 GPIO:16 GPIO:18 GPIO:19 GPIO:20 GPIO:21"),
               IOStandard("LVCMOS33")),  # J2 N15 R17 N16 L4 N3 N4 H4 aka 13 MISO SCK MOSI A0 A1 A2 A3
     Subsignal("hract", Pins("GPIO:22"), IOStandard("LVCMOS33")),  # G4 aka A4
     Subsignal("htack", Pins("GPIO:23"), IOStandard("LVCMOS33")),  # T17 aka A5
    )
]


class LASoC(BaseSoC):
    def __init__(self, sys_clk_freq=48e6, **kwargs):
        BaseSoC.__init__(self, sys_clk_freq=sys_clk_freq,
                         with_led_chaser=True, **kwargs)

        self.cd_hspi = ClockDomain()
        self.platform.add_extension(feather_serial)
        self.platform.add_extension(feather_gpio)
        self.platform.add_extension(feather_hspi)
        self.add_uartbone()
        ios = self.platform.request("gpios")
        hspi_pads = self.platform.request("hspi")
        self.crg.pll.create_clkout(self.cd_hspi, 4e6)  # create 4 MHz clock for HSPI

        self.submodules.la = LA(ios, hspi_pads, depth=4096, clock_domain="hspi", samplerate=4e6,
                                testing=kwargs['testing'])

        if kwargs['testing']:
            analyzer_signals = [hspi_pads.req, hspi_pads.ready, hspi_pads.valid, hspi_pads.hract, hspi_pads.htack,
                                self.la.hspi_tx.hd.o, self.la.hspi_tx.hd.oe, self.cd_hspi.clk,
                                self.la.hspi_tx.remaining_packets]
            self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals,
                                                         depth=0x10000,
                                                         clock_domain="sys",
                                                         samplerate=sys_clk_freq,
                                                         csr_csv="analyzer.csv",
                                                         trigger_depth=64
                                                         )


def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=Platform, description="SucreLA on OrangeCrab.")
    parser.add_target_argument("--sys-clk-freq",    default=48e6, type=float, help="System clock frequency.")
    parser.add_target_argument("--revision",        default="0.2",            help="Board Revision (0.1 or 0.2).")
    parser.add_target_argument("--device",          default="85F",            help="ECP5 device (25F, 45F or 85F).")
    parser.add_target_argument("--sdram-device",    default="MT41K64M16",     help="SDRAM device (MT41K64M16, MT41K128M16, MT41K256M16 or MT41K512M16).")
    parser.add_target_argument("--sim",             action="store_true", help="run LA simulation testsuite")
    parser.add_target_argument("--testing",         action="store_true", help="run LA with testing sample data")
    parser.set_defaults(no_uart=True, cpu_type="None")
    args = parser.parse_args()

    if args.sim:
        LATest(stop_via="hspi")
        LATest(stop_via="storage")
        LATest()
        exit(0)

    soc = LASoC(sys_clk_freq=args.sys_clk_freq,
                revision=args.revision,
                device=args.device,
                sdram_device=args.sdram_device,
                testing=args.testing,
                **parser.soc_argdict)

    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))


if __name__ == "__main__":
    main()
