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


@register_parser('--dump-page-tables', 'Dumps page tables')
class PageTableDump(RamParser):

    def parse(self):
        with self.ramdump.open_file('page_tables.txt') as f:
            self.ramdump.mmu.dump_page_tables(f)
            print_out_str('Page tables dumped to page_tables.txt')
