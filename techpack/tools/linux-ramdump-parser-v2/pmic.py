# Copyright (c) 2014, 2020 The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from datetime import datetime
from print_out import print_out_str
from ramparse import VERSION
from time import strftime

class PmicRegData():
    def __init__(self, regoffs, val):
        self.regoffs = regoffs
        self.val = val

class PmicRegDump():

    def __init__(self, start, end):
        self.start_addr = start
        self.end_addr = end
        self.num_entries = 0
        self.pmic_list = {}

    def parse_all_regs(self, ram_dump):

        if (self.start_addr + 4) > self.end_addr:
            return False

        # Read number of entries
        self.num_entries = ram_dump.read_u32(self.start_addr, False)
        self.start_addr += 4

        print_out_str('Dumping {0} PMIC registers'.format(self.num_entries))

        for i in range(0, self.num_entries):

            if (self.start_addr + 12) > self.end_addr:
                return False

            # Read slave ID
            slaveid = ram_dump.read_u32(self.start_addr, False)
            self.start_addr += 4;

            # Read register offset
            regoffs = 0xFFFF & (ram_dump.read_u32(self.start_addr, False))
            self.start_addr += 4;

            # Read register value
            val = ram_dump.read_u32(self.start_addr, False)
            self.start_addr += 4;

            if slaveid > 0xf:
                print_out_str("Maximum value of slave id is 0xF but found {0:x}. Aborting PMIC dump!".format(slaveid))
                return False;

            pmic_num = slaveid // 2;

            if pmic_num not in self.pmic_list:
                pmic_reg_list = []
                self.pmic_list[pmic_num] = pmic_reg_list
            else:
                pmic_reg_list = self.pmic_list[pmic_num]

            # For odd slave id prepend 0x10000 to offset
            if slaveid % 2 != 0:
                regoffs |= 0x10000

            regData = PmicRegData(regoffs, val)
            pmic_reg_list.append(regData)

        return True

    def dump_all_regs(self, ram_dump):
        outfile = ram_dump.open_file('pmicdump.xml')
        outfile.write('<pmicDump version=\'1\'>\n')
        outfile.write('\t<timestamp>{0}</timestamp>\n'.format(datetime.now().strftime('%d. %b %Y, %H:%M:%S')))
        outfile.write('\t<generator>Linux Ram Dump Parser Version {0}</generator>\n'.format(VERSION))
        outfile.write('\t<target name=\'{0}\'>\n'.format(ram_dump.hw_id))
        for  pmic in self.pmic_list.iterkeys():
            dump_pmic_reg_list = self.pmic_list[pmic]

            if not dump_pmic_reg_list:
                continue

            outfile.write('\t\t<pmic>\n')
            outfile.write('\t\t\t<bus type=\'spmi\' slaveid0=\'{0}\' slaveid1=\'{1}\'/>\n'.format(pmic, (pmic+1)))
            for reg_data in dump_pmic_reg_list:
                outfile.write('\t\t\t<register address=\'0x{0:x}\' value=\'0x{1:x}\'/>\n'.format(reg_data.regoffs, reg_data.val))
            outfile.write('\t\t</pmic>\n')
        outfile.write('\t</target>\n')
        outfile.write('</pmicDump>\n')
        outfile.close()
