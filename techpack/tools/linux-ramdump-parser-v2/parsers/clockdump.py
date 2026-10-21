# Copyright (c) 2015,2017-2018 The Linux Foundation. All rights reserved.
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
from parser_util import register_parser, RamParser


@register_parser('--clock-dump', 'Dump all the clocks in the system')
class ClockDumps(RamParser):

    def __init__(self, *args):
        super(ClockDumps, self).__init__(*args)
        self.enabled_clocks = []
        self.disabled_clocks = []
        self.prepared_clocks = []
        self.head = ''

    def print_header(self, type, title):
        if type == 'CLK_PROVIDERS':
            self.output_file.write("--------------------------------------------\n")
            self.output_file.write("{0} from of_clk_providers list\n".format(title))
            self.output_file.write("--------------------------------------------\n")
            str = "  {0:40} {1:21} {2:25} {3:10} {4:45} {5:40}\n"
            self.output_file.write(str.format('CLOCK NAME',
                                            'COUNT/PREPARE_COUNT',
                                            'RATE', 'CUR_LEVEL',
                                            'CLOCK STRUCTURE', 'CLOCK_OPS'))
        elif type == 'CLOCKS':
            self.output_file.write("----------------------------------\n")
            self.output_file.write("{0} from clocks list\n".format(title))
            self.output_file.write("----------------------------------\n")
            self.output_file.write("  {0:40} {1:25} {2:20} {3:21} {4:25} {5:20} {6:40}\n".format('CLOCK NAME', 'DEVID', 'CONID', 'COUNT/PREPARE_COUNT', 'RATE', 'CUR_LEVEL', 'CLOCK STRUCTURE'))

    def printclocks(self, type):
        if len(self.disabled_clocks):
            self.print_header(type, "Disabled Clocks")
            for clocks in self.disabled_clocks:
                self.output_file.write('D ' + clocks)

        if len(self.enabled_clocks):
            self.output_file.write("\n")
            self.print_header(type, "Enabled Clocks")
            for clocks in self.enabled_clocks:
                self.output_file.write('E ' + clocks)

        if len(self.prepared_clocks):
            self.output_file.write("\n")
            self.print_header(type, "Prepared Clocks")
            for clocks in self.prepared_clocks:
                self.output_file.write('P ' + clocks)

    def get_clocks(self):
        clocks = self.ramdump.address_of('clocks')
        if clocks is None:
            self.output_file.write("NOTE: 'clocks' list not found to extract the clocks information")
            return

        head = self.ramdump.read_word(clocks, True)
        self.head = clocks
        node_offset = self.ramdump.field_offset('struct clk_lookup', 'node')
        clocks_walker = linux_list.ListWalker(self.ramdump, head, node_offset)
        clocks_walker.walk(head, self.clocks_walker)

    def clocks_walker(self, node):
        if node == self.head:
            return

        devid_address = node + self.ramdump.field_offset('struct clk_lookup', 'dev_id')
        devid = self.ramdump.read_cstring(self.ramdump.read_word(devid_address, True), 48)
        conid_address = node + self.ramdump.field_offset('struct clk_lookup', 'con_id')
        conid = self.ramdump.read_cstring(self.ramdump.read_word(conid_address, True), 48)

        clock_address = node + self.ramdump.field_offset('struct clk_lookup', 'clk')
        clk = self.ramdump.read_word(clock_address, True)
        dbg_name_address = clk + self.ramdump.field_offset('struct clk', 'dbg_name')
        dbg_name = self.ramdump.read_cstring(self.ramdump.read_word(dbg_name_address, True), 48)
        rate_address = clk + self.ramdump.field_offset('struct clk', 'rate')
        rate = self.ramdump.read_word(rate_address, True)
        count_address = clk + self.ramdump.field_offset('struct clk', 'count')
        count = self.ramdump.read_u32(count_address, True)
        prepare_count_address = clk + self.ramdump.field_offset('struct clk', 'prepare_count')
        prepare_count = self.ramdump.read_u32(prepare_count_address, True)
        vdd_class_address = clk + self.ramdump.field_offset('struct clk', 'vdd_class')
        vdd_class = self.ramdump.read_word(vdd_class_address, True)
        if vdd_class != 0:
            cur_level_address = vdd_class + self.ramdump.field_offset('struct clk_vdd_class', 'cur_level')
            cur_level = self.ramdump.read_word(cur_level_address, True)
        else:
            cur_level = "NULL"

        output = "{0:40} {1:<25} {2:20} {3:<2}/ {4:<17} {5:<25} {6:<10} v.v (struct clk *)0x{7:<20x}\n".format(
            dbg_name, devid, conid, count, prepare_count, rate, cur_level, clk)

        if count > 0:
            self.enabled_clocks.append(output)
        elif prepare_count > 0:
            self.prepared_clocks.append(output)
        else:
            self.disabled_clocks.append(output)

    def get_clk_providers(self):
        clocks = self.ramdump.address_of('of_clk_providers')
        if clocks is None:
            self.output_file.write("NOTE: 'of_clk_providers' list not found to extract the clocks information")
            return

        self.enabled_clocks = []
        self.disabled_clocks = []
        self.prepared_clocks = []
        self.head = clocks

        head = self.ramdump.read_word(clocks, True)
        node_offset = self.ramdump.field_offset('struct clk_lookup', 'node')
        clk_providers_walker = linux_list.ListWalker(self.ramdump, head, node_offset)
        clk_providers_walker.walk(head, self.clk_providers_walker)

    def print_clk_of_msm_provider_data(self, data):
        table_address = data + self.ramdump.field_offset('struct of_msm_provider_data', 'table')
        size_address = data + self.ramdump.field_offset('struct of_msm_provider_data', 'size')
        table = self.ramdump.read_word(table_address, True)
        size = self.ramdump.read_word(size_address, True)

        counter = 0
        while counter < size:
            clock_address = table + self.ramdump.field_offset('struct clk_lookup', 'clk')
            clk = self.ramdump.read_word(clock_address, True)
            dbg_name_address = clk + self.ramdump.field_offset('struct clk', 'dbg_name')
            dbg_name = self.ramdump.read_cstring(self.ramdump.read_word(dbg_name_address, True), 48)
            rate_address = clk + self.ramdump.field_offset('struct clk', 'rate')
            rate = self.ramdump.read_word(rate_address, True)
            count_address = clk + self.ramdump.field_offset('struct clk', 'count')
            count = self.ramdump.read_u32(count_address, True)
            prepare_count_address = clk + self.ramdump.field_offset('struct clk', 'prepare_count')
            prepare_count = self.ramdump.read_u32(prepare_count_address, True)
            vdd_class_address = clk + self.ramdump.field_offset('struct clk', 'vdd_class')
            vdd_class = self.ramdump.read_word(vdd_class_address, True)
            if vdd_class != 0:
                cur_level_address = vdd_class + self.ramdump.field_offset('struct clk_vdd_class', 'cur_level')
                cur_level = self.ramdump.read_word(cur_level_address, True)
            else:
                cur_level = "NULL"

            output = "{0:40} {1:<2}/ {2:<17} {3:<25} {4:<10} v.v (struct clk *)0x{5:<20x}\n".format(dbg_name, count, prepare_count, rate, cur_level, clk)

            if count > 0:
                self.enabled_clocks.append(output)
            elif prepare_count > 0:
                self.prepared_clocks.append(output)
            else:
                self.disabled_clocks.append(output)

            counter = counter + 1
            table = table + self.ramdump.sizeof('struct clk_lookup')

    def dump_clock(self,clk_core,clk_name):
        offset_vdd_cur_level = self.ramdump.field_offset(
                                'struct clk_vdd_class', 'cur_level')
        clk_prepare_count = self.ramdump.read_structure_field(
                                clk_core, 'struct clk_core', 'prepare_count')
        clk_enable_count = self.ramdump.read_structure_field(
                                clk_core, 'struct clk_core', 'enable_count')
        clk_rate = self.ramdump.read_structure_field(
                                    clk_core, 'struct clk_core', 'rate')
        vdd_class = self.ramdump.read_structure_field(
                                clk_core,'struct clk_core','vdd_class')
        clk_ops = self.ramdump.read_structure_field(
                                clk_core,'struct clk_core','ops')
        clk_ops = self.ramdump.unwind_lookup(clk_ops)
        if clk_ops is None:
            clk_ops = ["dynamic module"]
        cur_level = 0
        if vdd_class != 0 and vdd_class is not None:
            cur_level_address = (vdd_class + offset_vdd_cur_level)
            cur_level = self.ramdump.read_word(cur_level_address, True)
        formatStr = "{0:40} {1:<2}/ {2:<17} {3:<25} {4:<10} " \
                    "v.v (struct clk_core *)0x{5:<20x} {6:<40}\n"
        output = formatStr.format(
                                clk_name,
                                clk_enable_count,
                                clk_prepare_count,
                                clk_rate, cur_level,
                                clk_core,clk_ops[0])

        if clk_enable_count > 0:
            self.enabled_clocks.append(output)
        elif clk_prepare_count > 0:
            self.prepared_clocks.append(output)
        else:
            self.disabled_clocks.append(output)


    def print_clk_onecell_data(self, data):
        offset_clk_onecell_data_clks = (
                self.ramdump.field_offset('struct clk_onecell_data', 'clks'))
        offset_clk_onecell_data_clknum = (
                            self.ramdump.field_offset(
                                'struct clk_onecell_data', 'clk_num'))
        clks = self.ramdump.read_word(data + offset_clk_onecell_data_clks)
        if (clks == 0 or clks == None):
            return
        size = self.ramdump.read_int(data + offset_clk_onecell_data_clknum)
        sizeof_clk_pointer = self.ramdump.sizeof('struct clk *')
        offset_vdd_cur_level = self.ramdump.field_offset(
                                    'struct clk_vdd_class', 'cur_level')
        counter = 0

        while counter < size:
            clk = self.ramdump.read_word(clks + (sizeof_clk_pointer * counter))
            if clk == 0 or clk is None:
                counter = counter + 1
                continue
            clk_core = self.ramdump.read_structure_field(
                                                    clk, 'struct clk', 'core')
            if clk_core == 0 or clk_core is None:
                counter = counter + 1
                continue
            clk_name_addr = self.ramdump.read_structure_field(
                                        clk_core, 'struct clk_core', 'name')
            clk_name = self.ramdump.read_cstring(clk_name_addr, 48)
            if (clk_name == 0 or clk_name == None):
                break
            self.dump_clock(clk_core,clk_name)
            counter = counter + 1

    # qcom_cc_clk_hw_get clk is added in kernel 4.9
    def print_clk_qcom_cc_data(self, data):
        size = self.ramdump.read_structure_field(
                        data,'struct qcom_cc','num_rclks')
        clks = self.ramdump.read_structure_field(
                        data,'struct qcom_cc','rclks')
        sizeof_clk_regmap = self.ramdump.sizeof('struct clk_regmap *')
        offset_vdd_cur_level = self.ramdump.field_offset(
                        'struct clk_vdd_class', 'cur_level')
        counter = 0
        while counter < size:
            clk = self.ramdump.read_word(clks +
                                 (sizeof_clk_regmap * counter))
            clk_core = self.ramdump.read_structure_field(
                                clk,'struct clk_regmap','hw.core')
            if clk_core == 0 or clk_core is None:
                counter = counter + 1
                continue
            clk_name_addr = self.ramdump.read_structure_field(
                                clk_core, 'struct clk_core', 'name')
            clk_name = self.ramdump.read_cstring(clk_name_addr, 48)
            if (clk_name == 0 or clk_name == None):
                break
            self.dump_clock(clk_core,clk_name)
            counter = counter + 1

    # spmi_pmic_div_clk_hw_get clk is added kernel 4.14
    def print_clk_spmi_pmic_data(self, data):
        size = self.ramdump.read_structure_field(
                        data,'struct spmi_pmic_div_clk_cc','nclks')
        clks = self.ramdump.field_offset(
                        'struct spmi_pmic_div_clk_cc','clks')
        clks = data + clks
        sizeof_clk_regmap = self.ramdump.sizeof('struct clkdiv')
        offset_vdd_cur_level = self.ramdump.field_offset(
                    'struct clk_vdd_class', 'cur_level')
        counter = 0
        while counter < size:
            clk = clks + (sizeof_clk_regmap * counter)
            clk_core = self.ramdump.read_structure_field(
                                clk,'struct clkdiv','hw.core')
            if clk_core == 0 or clk_core is None:
                counter = counter + 1
                continue
            clk_name_addr = self.ramdump.read_structure_field(
                                clk_core, 'struct clk_core', 'name')
            clk_name = self.ramdump.read_cstring(clk_name_addr, 48)
            if (clk_name == 0 or clk_name == None):
                break
            self.dump_clock(clk_core,clk_name)
            counter = counter + 1

    def clk_providers_walker(self, node):
        if node == self.head:
            return
        data_address = node + self.ramdump.field_offset(
                                    'struct of_clk_provider', 'data')
        data = self.ramdump.read_word(data_address, True)

        getfunc = self.ramdump.read_structure_field(
                        node,'struct of_clk_provider','get')
        if getfunc == 0:
            getfunchw = self.ramdump.read_structure_field(
                            node,'struct of_clk_provider','get_hw')
            getfunchw = self.ramdump.unwind_lookup(getfunchw)
            if "spmi_pmic_div_clk_hw_get" in getfunchw[0]:
                self.print_clk_spmi_pmic_data(data)
                return
            elif "qcom_cc_clk_hw_get" in getfunchw[0]:
                self.print_clk_qcom_cc_data(data)
                return
            else:
                return

        if (self.ramdump.is_config_defined('CONFIG_COMMON_CLK_MSM')):
            self.print_clk_of_msm_provider_data(data)
        else:
            self.print_clk_onecell_data(data)

    def print_a7_cpu_frequency(self):
        cpu_clks_hws = self. ramdump.read_u32('cpu_clks_hws')
        if cpu_clks_hws is not None:
            clk_offset  = self.ramdump.read_u32(cpu_clks_hws + 0x4)
            core_offset = self.ramdump.read_u32(clk_offset)
            clk_rate_offset  = self.ramdump.field_offset('struct clk_core',
                                                    'rate')
            dbg_name_address = core_offset + self.ramdump.field_offset('struct clk_core', 'name')
            dbg_name = self.ramdump.read_cstring(self.ramdump.read_word(dbg_name_address, True), 48)
            cpu_frequency = self.ramdump.read_u32(core_offset + clk_rate_offset)
            self.output_file.write("{0} =  {1} \n".format(dbg_name, cpu_frequency))

    def parse(self):
        self.output_file = self.ramdump.open_file('ClockDumps.txt')
        if (self.ramdump.kernel_version < (4, 9, 0)):
            self.get_clocks()
            self.printclocks('CLOCKS')
        self.get_clk_providers()
        self.printclocks('CLK_PROVIDERS')
        self.print_a7_cpu_frequency()
        self.output_file.close()
        print_out_str("--- Wrote the output to ClockDumps.txt")
