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

@register_parser('--hyp-log', 'Dump hypervisor log')
class HypLog(RamParser):

    def parse(self):
        if self.ramdump.hyp_diag_addr is None:
            raise Exception('Cannot find hyp_diag_addr!!!')
        hyp_diag_addr = self.ramdump.read_u32(self.ramdump.hyp_diag_addr, False)
        hyp_log_addr = self.ramdump.read_u32(hyp_diag_addr + 0x78, False)
        hyp_log_size = self.ramdump.read_u32(hyp_diag_addr + 0x10, False)
        hyp_log = self.ramdump.read_physical(hyp_log_addr, hyp_log_size)
        if hyp_log is None:
            raise Exception('!!!Could not read hyp_log from address {0:x}'.format(addr))
        hyp_log_output = self.ramdump.open_file('hyp_log.txt')
        hyp_log_output.write(cleanupString(hyp_log.decode('ascii', 'ignore')))
        hyp_log_output.close()
        print_out.print_out_str('Hypervisor log successfully extracted!')