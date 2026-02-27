# Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from print_out import print_out_str
from parser_util import register_parser, RamParser

MAX_DATA_NUM = 50

@register_parser('--thermal-temp', 'Print the tsensor/adc temp')
class ThermalTemp(RamParser):

    def __init__(self, *args):
        super(ThermalTemp, self).__init__(*args)

    def parse(self):

        def parse_thermal_minidump(name):
            data_addr = self.ramdump.read_word('md')
            if data_addr is None and self.ramdump.minidump:
                seg = next((s for s in self.ramdump.elffile.iter_sections() if s.name == name), None)
                if seg is None:
                    print_out_str('--md_{0} does not exist--'.format(name))
                if seg is not None:
                    data_addr = seg['sh_addr']

                    print_out_str('------{0} data------'.format(name))
                    count = self.ramdump.read_u32(data_addr + MAX_DATA_NUM * 24)
                    last_type = self.ramdump.read_cstring(data_addr + (MAX_DATA_NUM - 1) * 20)

                    if last_type == '':
                        for i in range(count):
                            sensor_type = self.ramdump.read_cstring(data_addr + i * 20)
                            sensor_temp = self.ramdump.read_u32(data_addr + MAX_DATA_NUM * 20 + i * 4)
                            print_out_str('sensor_type : {0}\nsensor_temp : {1}\n'.format(sensor_type, sensor_temp))

                    if last_type != '':
                        for i in range(count, MAX_DATA_NUM):
                            sensor_type = self.ramdump.read_cstring(data_addr + i * 20)
                            sensor_temp = self.ramdump.read_u32(data_addr + MAX_DATA_NUM * 20 + i * 4)
                            print_out_str('sensor_type : {0}\nsensor_temp : {1}\n'.format(sensor_type, sensor_temp))
                        for i in range(count):
                            sensor_type = self.ramdump.read_cstring(data_addr + i * 20)
                            sensor_temp = self.ramdump.read_u32(data_addr + MAX_DATA_NUM * 20 + i * 4)
                            print_out_str('sensor_type : {0}\nsensor_temp : {1}\n'.format(sensor_type, sensor_temp))

        minidump_table_name = ['adc5_gen3', 'tsens0', 'tsens1', 'tsens2']
        for i in range(len(minidump_table_name)):
            parse_thermal_minidump(minidump_table_name[i])


