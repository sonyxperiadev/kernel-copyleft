# Copyright (c) 2021 The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
import print_out
from parser_util import RamParser, cleanupString, register_parser

@register_parser('--rm-log', 'Dump resource manager log')
class RMLog(RamParser):

    def parse(self):
        if self.ramdump.rm_debug_addr is None:
            raise Exception('Cannot find rm_debug_addr!!!')
        rm_debug_addr = self.ramdump.read_u32(self.ramdump.rm_debug_addr, False)
        rm_log_addr = self.ramdump.read_u32(rm_debug_addr + 0x20, False)
        rm_log_size = self.ramdump.read_u32(rm_debug_addr + 0x28, False)
        rm_log = self.ramdump.read_physical(rm_log_addr, rm_log_size)
        if rm_log is None:
            raise Exception('!!!Could not read rm_log from address {0:x}'.format(rm_log_addr))
        rm_log_output = self.ramdump.open_file('rm_log.txt')
        rm_log_output.write(cleanupString(rm_log.decode('ascii', 'ignore')))
        rm_log_output.close()
        print_out.print_out_str('Resource manager log successfully extracted!')