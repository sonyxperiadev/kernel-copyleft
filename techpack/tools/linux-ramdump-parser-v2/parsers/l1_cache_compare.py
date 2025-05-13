# Copyright (c) 2017, 2020 The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import os
import re
from parser_util import register_parser, RamParser


file_names = ['MSM_DUMP_DATA_L1_DATA_CACHE_0x0',
              'MSM_DUMP_DATA_L1_DATA_CACHE_0x1',
              'MSM_DUMP_DATA_L1_DATA_CACHE_0x2',
              'MSM_DUMP_DATA_L1_DATA_CACHE_0x3',
              'MSM_DUMP_DATA_L1_DATA_CACHE_0x4',
              'MSM_DUMP_DATA_L1_DATA_CACHE_0x5',
              'MSM_DUMP_DATA_L1_DATA_CACHE_0x6',
              'MSM_DUMP_DATA_L1_DATA_CACHE_0x7']


@register_parser('--l1-compare', 'Compare L1 Cache Data with DDR contents')
class L1_Cache_Compare(RamParser):

    def parse(self):
        with self.ramdump.open_file('l1_cache.txt') as out_l1_cache:

            for file_name in file_names:
                file_path = os.path.join(self.ramdump.outdir, file_name)
                if os.path.isfile(file_path):

                    out_l1_cache.write('\n-----begin ' + file_name + '-----\n')

                    addr = None
                    values_from_file = None
                    values_from_dump = None

                    fin = self.ramdump.open_file(file_path, 'r')
                    line_no = 0
                    for line in fin:
                        line_no += 1
                        if line == '':
                            continue
                        elif re.match('^Way Set P MOESI RAW_MOESI N.*', line):
                            continue
                        elif len(line) >= 195:
                            colm = line.split()

                            if len(colm) < 28:
                                out_l1_cache.write('Unexepected file format\n')
                                break

                            # Read address value from file
                            addr = colm[6]

                            # Read values from file
                            values_from_file = []
                            for i in range(11, 27):
                                values_from_file.append(colm[i])

                            # Read values from dump
                            values_from_dump = []
                            temp_addr = int(addr, 16)
                            val = None
                            if temp_addr > self.ramdump.phys_offset:
                                for i in range(0, 16):
                                    try:
                                        val = self.ramdump.read_physical(
                                                temp_addr, 4)
                                    except:
                                        out_l1_cache.write(
                                            'Exception while reading {0:x}'
                                            .format(temp_addr))

                                    val = val[::-1]
                                    val = val.encode('hex')

                                    values_from_dump.append(val)
                                    temp_addr += 4

                        # compare both values
                        if values_from_dump != [] and \
                                values_from_dump != values_from_file:
                            out_l1_cache.write(
                                'phy addr: {0} Way:{1} Set:{2} P:{3} MOESI:{4}'
                                .format(addr,
                                        colm[0],
                                        colm[1],
                                        colm[2],
                                        colm[3]) +
                                '\n')

                            out_l1_cache.write('Cache content: ' +
                                               ' '.join(values_from_file) +
                                               '\n')
                            out_l1_cache.write('DDR content  : ' +
                                               ' '.join(values_from_dump) +
                                               '\n\n')

                    fin.close()

                    out_l1_cache.write('------end '+file_name+'------\n')
