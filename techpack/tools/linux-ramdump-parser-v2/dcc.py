# Copyright (c) 2015, 2017, 2020 The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import struct
import os
from print_out import print_out_str
from ramparse import VERSION

dcc_register_list = [
    'DCC_HW_VERSION',
    'DCC_HW_INFO',
    'DCC_CGC_CFG',
    'DCC_LL',
    'DCC_RAM_CFG',
    'DCC_CFG',
    'DCC_SW_CTL',
    'DCC_STATUS',
    'DCC_FETCH_ADDR',
    'DCC_SRAM_ADDR',
    'DCC_INT_ENABLE',
    'DCC_INT_STATUS',
    'DCC_QSB_CFG'
    ]

# DCC regs hash table
dcc_regs = {}

class DccRegDump():

    def __init__(self, start, end):
        self.start_addr = start
        self.end_addr = end

    def parse_all_regs(self, ram_dump):
        num_reg = len(dcc_register_list)
        if (self.start_addr + 4 * num_reg) > self.end_addr:
                return False

        for reg in dcc_register_list:
                dcc_regs[reg] = ram_dump.read_u32(self.start_addr, False)
                self.start_addr += 4
        return True

    def dump_all_regs(self, ram_dump):
        outfile = ram_dump.open_file('dcc_regs.txt')
        outfile.write('DCC registers:\n')
        for reg in dcc_register_list:
                outfile.write('{0} : 0x{1:08x}\n'.format(reg, dcc_regs[reg]))
        outfile.close()

class DccSramDump():
    def __init__(self, start, end, ram_dump):
        self.start_addr = start
        self.end_addr = end
        self.bin_dir = None
        if ram_dump.ram_addr is None:
            self.bin_dir = ram_dump.autodump
        else:
            self.bin_dir = ram_dump.ram_addr
            self.bin_dir="\\".join(self.bin_dir[0][0].split('\\')[:-1])
        self.dcc_bin = os.path.join(self.bin_dir, 'DCC_SRAM.BIN')

        if os.path.isfile(self.dcc_bin):
            self.start_addr = 0x6000
            self.end_addr = 0x8000

    def dump_sram_img(self, ram_dump):
        if self.start_addr >= self.end_addr:
                return False

        rsz = self.end_addr - self.start_addr

        if os.path.isfile(self.dcc_bin):
            return self.dump_sram_img_bin(ram_dump, self.dcc_bin)

        if 'DCC_HW_INFO' not in dcc_regs \
                        or dcc_regs['DCC_HW_INFO'] == 0:
            print_out_str('DCC HW Info missing! Skipping sram dump...')
            return False

        if dcc_regs['DCC_CFG'] & 0x1:
            print_out_str('DCC is configured in CRC mode. Skipping sram dump ...')
            return False

        if dcc_regs['DCC_RAM_CFG'] == 0:
            print_out_str('No config found in DCC SRAM. Skipping sram dump ...')
            return False

        sramfile = ram_dump.open_file('sram.bin')

        for i in range(0, rsz):
            val = ram_dump.read_byte(self.start_addr + i, False)
            sramfile.write(struct.pack('<B', val))

        sramfile.close()

        return True

    def dump_sram_img_bin(self, ram_dump, dcc_bin):
        if self.start_addr >= self.end_addr:
                return False

        f = open(dcc_bin, 'rb')
        f.seek(self.start_addr, 1)
        bin_data=f.read()
        sramfile = ram_dump.open_file('sram.bin')
        sramfile.write(bin_data)
        f.close()
        sramfile.close()

        return True
