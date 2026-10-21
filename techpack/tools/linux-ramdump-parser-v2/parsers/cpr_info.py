# Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import linux_list
from print_out import print_out_str
from parser_util import register_parser, RamParser, cleanupString
from collections import defaultdict

@register_parser('--cpr-info', 'Print CPR information')
class CPRInfo(RamParser):
    def __init__(self, *args):
        super(CPRInfo, self).__init__(*args)
        self.head = ''
        self.cprinfo_fields = ['speed_bin', 'cpr_fuse_revision', 'cpr_fuse_map_match', 'num_fuse_corners', 'num_corners', 'corner']
        self.voltages = ['ceiling_volt', 'open_loop_volt', 'last_volt', 'floor_volt']
        self.corner_info = ['cpr_fuse_target_quot', 'quot_adjust', 'corner_map']
        self.value_list = defaultdict(list)
        self.attr_list = defaultdict(list)
        self.output = []

    def print_cpr_target_quot(self):
        tmp = '{0:20}'.format('Target quotient')
        for i in range(self.attr_list['num_corners']):
            a = self.value_list['corner_map'][i]
            b = self.value_list['cpr_fuse_target_quot'][a-1] - self.value_list['quot_adjust'][i]
            tmp += '{0:10} '.format(b)
        tmp += '\n'
        self.output.append(tmp)

    def print_cpr_info(self):
        tmp = ''
        # Print RO_SEL value
        num_fuse_corn = self.attr_list['num_fuse_corners']
        if num_fuse_corn is not None:
            self.output.append('{:40}{:10d}\n'.format('ro_sel', self.value_list['cpr_fuse_ro_sel'][num_fuse_corn-1]))

        # Print all available RO_SEL values
        tmp += '{:40}'.format('cpr_fuse_ro_sel')
        for ro_sel in self.value_list['cpr_fuse_ro_sel']:
            tmp += '{:10} '.format(ro_sel)
        tmp += '\n\n'
        self.output.append(tmp)
        tmp = ''

        self.output.append('{:20}'.format('Corner'))
        for i in range(self.attr_list['num_corners']):
            tmp += '{:10} '.format(i + 1)
        tmp += '\n'
        for volt in self.voltages:
            tmp += '{:20}'.format(volt)
            for i in self.value_list[volt]:
                tmp += '{:10} '.format(i)
            tmp += '\n'
        self.output.append(tmp)

    def get_cpr(self):
        # Return if the cpr_regulator_list is not available
        cpr = self.ramdump.address_of('cpr_regulator_list')
        if cpr is None:
            self.output_file.write("NOTE: 'cpr_regulator_list' list not found to extract cpr information")
            return

        head = self.ramdump.read_word(cpr)
        self.head = cpr
        node_offset = self.ramdump.field_offset('struct cpr_regulator', 'list')
        cpr_walker = linux_list.ListWalker(self.ramdump, head, node_offset)
        cpr_walker.walk(head, self.cpr_walker)

    def get_cpr_fuse_ro_sel(self, node):
        entry_offset = self.ramdump.sibling_field_addr(node, 'struct cpr_regulator', 'list', 'cpr_fuse_ro_sel')
        entry_addr = self.ramdump.read_word(entry_offset)
        i = 1
        while i <= self.attr_list['num_fuse_corners']:
            value = self.ramdump.read_int(self.ramdump.array_index(entry_addr, "int", i))
            self.value_list['cpr_fuse_ro_sel'].append(value)
            i += 1

    def get_cpr_volts(self, node, listing):
        i = 1
        num_corn = self.attr_list['num_corners']
        while i <= num_corn:
            i += 1
            for entry in listing:
                entry_offset = self.ramdump.sibling_field_addr(node, 'struct cpr_regulator', 'list', entry)
                entry_addr = self.ramdump.read_word(entry_offset)
                i = 1
                while i <= num_corn:
                    value = self.ramdump.read_int(self.ramdump.array_index(entry_addr, "int", i))
                    self.value_list[entry].append(value)
                    i += 1

    def get_cpr_attrs(self, node):
        for attr in self.cprinfo_fields:
            attr_offset = self.ramdump.field_offset('struct cpr_regulator', attr)
            if attr_offset is not None:
                value = self.ramdump.read_s32(node + attr_offset)
                self.attr_list[attr] = value
                tmp = '{:40}{:10}\n'.format(attr, value)
                self.output.append(tmp)
        attr_offset = self.ramdump.field_offset('struct cpr_regulator', 'cpr_fuse_redundant')
        if attr_offset is not None:
            value = self.ramdump.read_bool(node + attr_offset)
            # add an extra line here as this is the last attribute before the corner table
            tmp = '{:40} {:10}\n'.format('cpr_fuse_redundant', int(value))
            self.output.append(tmp)

    def cpr_walker(self, node):
        if node == self.head:
            return

        rdesc_addr = self.ramdump.sibling_field_addr(node, 'struct cpr_regulator', 'list', 'rdesc')
        rdesc_ptr = self.ramdump.read_word(rdesc_addr + self.ramdump.field_offset('struct regulator_desc', 'name'))
        cpr_name = self.ramdump.read_cstring(rdesc_ptr, 48)
        cpr_enable = self.ramdump.read_u32(node + self.ramdump.field_offset('struct cpr_regulator', 'enable'))
        vdd_apc_addr = self.ramdump.read_word(self.ramdump.sibling_field_addr(node, 'struct cpr_regulator', 'list', 'vdd_apc'))
        vdd_apc_uv = self.ramdump.read_u32(vdd_apc_addr + self.ramdump.field_offset('struct regulator', 'min_uV'))

        self.output.append("{:40}{:10s}\n".format('CPR Regulator', cpr_name))
        self.output.append("{:40}{:10}\n".format('CPR Enabled', cpr_enable))
        self.output.append("{:40}{:10d}\n".format('Current Voltage', vdd_apc_uv))
        self.get_cpr_attrs(node)
        self.get_cpr_volts(node, self.voltages)
        self.get_cpr_volts(node, self.corner_info)
        self.get_cpr_fuse_ro_sel(node)
        self.print_cpr_info()
        self.print_cpr_target_quot()
        # print new line for each regulator struct
        self.output.append('\n')
        self.attr_list.clear()
        self.value_list.clear()

    def parse(self):
        self.output_file = self.ramdump.open_file('cprinfo.txt')
        self.get_cpr()
        for i in self.output:
            self.output_file.write(i)
        self.output_file.close()
