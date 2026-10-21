# Copyright (c) 2015, The Linux Foundation. All rights reserved.
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
from print_out import print_out_str

MAX_FIFO_SIZE = 64
VSENS_ID_MAX = 6

sysdbg_vsens_idx_name = [
    'VSENS_ID_CX',
    'VSENS_ID_MX',
    'VSENS_ID_APC0',
    'VSENS_ID_APC1',
    'VSENS_ID_GFX',
    'VSENS_ID_EBI',
]

sysdbg_vsens_dump_name = [
    'ID',
    'CONFIG_REG0',
    'CONFIG_REG1',
    'STATUS_REG',
]

sysdbg_vsens_dump_type = ''.join([
    'I', #id
    'I', #config_reg_0
    'I', #config_reg_1
    'I', #status_reg
])

class VsensDumpType():

    def __init__(self, idx, regs, fifo):
        self.idx = idx
        self.regs = regs
        self.fifo = fifo

    def print_regs(self, outfile, ramdump):
        outfile.write('Voltage Sensor type: [{0}]\n'.format(
            sysdbg_vsens_idx_name[self.idx]))
        for i in range(0, 4):
            outfile.write(
                '{0} = {1:x}\n'.format(sysdbg_vsens_dump_name[i], self.regs[i]))

        outfile.write('FIFO DATA = \n')
        for i in range(0, MAX_FIFO_SIZE):
            if (i % 16) == 0:
                outfile.write('\n')
            outfile.write('{0:x} '.format(self.fifo[i]))
        outfile.write('\n\n\n\n')

class VsensData():

    def __init__(self):
        self.vsens_regs = []
        self.size = struct.calcsize(sysdbg_vsens_dump_type)

    def print_vsens_regs(self, ram_dump):
        vsens_file = ram_dump.open_file('vsens.txt')
        for idx in range(0, VSENS_ID_MAX):
            self.vsens_regs[idx].print_regs(vsens_file, ram_dump)
        vsens_file.close()

    def init_dump_regs(self, start_addr, end_addr, ram_dump):
        self.start_addr = start_addr
        self.end_addr = end_addr

        for i in range(0, VSENS_ID_MAX):
            regs = ram_dump.read_string(
                self.start_addr, sysdbg_vsens_dump_type, False)
            self.start_addr += self.size
            fifo = ram_dump.read_string(self.start_addr, 'B'*MAX_FIFO_SIZE, False)
            self.start_addr += struct.calcsize('B'*MAX_FIFO_SIZE)
            self.vsens_regs.append(VsensDumpType(i, regs, fifo))
        return True
