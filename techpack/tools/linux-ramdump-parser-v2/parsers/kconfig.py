# Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
from print_out import print_out_str


@register_parser('--print-kconfig', 'Print saved kernel configuration', shortopt='-c')
class Kconfig(RamParser):

    def parse(self):
        saved_config = self.ramdump.open_file('kconfig.txt')

        for l in self.ramdump.config:
            saved_config.write(l + '\n')

        saved_config.close()
        print_out_str('---wrote saved kernel config to kconfig.txt')
