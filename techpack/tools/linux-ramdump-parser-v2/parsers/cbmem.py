# Copyright (c) 2018-2019, 2021 The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from parser_util import cleanupString
from parser_util import register_parser, RamParser
from print_out import print_out_str
from re import finditer

# This is hardcoded since coreboot console is always at below physical address
#addr1 = 0xfffde000
#len1 = 0x20000
CBMC_CURSOR_MASK = ((1 << 28) - 1)
CBMC_OVERFLOW = (1 << 31)
COREBOOT_BOOTBLOCK = b'coreboot-[^\n]* bootblock starting.*\\.\\.\\.\n'

@register_parser('--cbmem', 'Print the coreboot console log', shortopt='-z')
class CBMEM(RamParser):


    def print_cbmem(self):
        #get starting address of cbmem_console
        #get the size of console
        cbmem_console_addr = self.ramdump.read_u64('cbmem_console')
        cbmem_console_size = self.ramdump.read_u32('cbmem_console_size')
        if (cbmem_console_addr is None) or (cbmem_console_size is None):
            print_out_str('cbmem_console stucture not found')
            return
        #read the valie of cbmem_console->cursor.  getting offset and reading u32
        cursor_offset = self.ramdump.field_offset('struct cbmem_cons', 'cursor')
        cbmem_console_cusor = self.ramdump.read_u32(cbmem_console_addr+cursor_offset)

        #convert the console address to Phy and read full untill size
        addr = self.ramdump.virt_to_phys(cbmem_console_addr)
        size = self.ramdump.sizeof('struct cbmem_console')
        cursor = cbmem_console_cusor & CBMC_CURSOR_MASK
        if (not(cbmem_console_cusor & CBMC_OVERFLOW) and cursor < cbmem_console_size):
            size = cursor
        else:
            size = cbmem_console_size
        if (cbmem_console_cusor & CBMC_OVERFLOW):
            if (cursor >= size):
                print_out_str("cbmem: ERROR: CBMEM console struct is illegal, "
                       "output may be corrupt or out of order!\n\n")
                cursor = 0
            cbmem = self.ramdump.read_physical(addr+cursor, size-cursor)
            cbmemPart1 = self.ramdump.read_physical(addr, cursor)
            cbmem = cbmem + cbmemPart1
        else:
            cbmem = self.ramdump.read_physical(addr, size)

        cbmem_console_out = self.ramdump.open_file('cbmem_console.txt')
        cbmem_console_out.write(cleanupString(cbmem.decode('ascii', 'ignore')) + '\n')
        cbmem_console_out.close()

        m = 0
        for match in finditer(COREBOOT_BOOTBLOCK, cbmem):
            m = match.start()
        cbmem = cbmem[m:]

        cbmem_out = self.ramdump.open_file('cbmem.txt')
        cbmem_out.write(cleanupString(cbmem.decode('ascii', 'ignore')) + '\n')
        cbmem_out.close()

        print_out_str('Wrote Coreboot console log to cbmem.txt')


    def parse(self):
        self.print_cbmem()
