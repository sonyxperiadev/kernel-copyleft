# Copyright (c) 2013-2015, 2020-2021 The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from parser_util import register_parser, RamParser
from parsers.gpu.gpuinfo_54 import GpuParser_54
from parsers.gpu.gpuinfo_510 import GpuParser_510
from print_out import print_out_str


@register_parser('--print-gpuinfo',
                 'print gpu driver related info', optional=True)
class GpuParser(RamParser):
    def __init__(self, dump):
        super(GpuParser, self).__init__(dump)

    def parse(self):
        if not (self.ramdump.is_config_defined('CONFIG_QCOM_KGSL') or
                'msm_kgsl' in self.ramdump.ko_file_names):
            print_out_str(
                "No GPU support detected... Skipping GPU parser.")
            return

        if (self.ramdump.kernel_version == (0, 0, 0) or
           self.ramdump.kernel_version >= (5, 10, 0)):
            self.parser = GpuParser_510(self.ramdump)
        elif self.ramdump.kernel_version >= (4, 9, 0):
            self.parser = GpuParser_54(self.ramdump)
        else:
            print_out_str(
                "No GPU support detected for specified kernel version..."
                + " Skipping GPU parser.")
            return

        self.parser.parse()

    def write(self, string):
        self.out.write(string)

    def writeln(self, string=""):
        self.out.write(string + '\n')
