# Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
# Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
# Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import sys
import os
import linux_list
from print_out import print_out_str
from parser_util import register_parser, RamParser
import rb_tree

@register_parser('--timer-list', 'Print all the linux timers')
class TimerList(RamParser) :

    def __init__(self, *args):
        super(TimerList, self).__init__(*args)
        self.vectors = {'tv1': 256, 'tv2': 64, 'tv3': 64, 'tv4': 64, 'tv5': 64}
        self.output = []
        major, minor, patch = self.ramdump.kernel_version
        self.timer_42 = False
        self.timer_has_data = True
        self.timer_jiffies = 'timer_jiffies'
        self.tvec_base = 'struct tvec_base'
        self.tvec_bases = 'tvec_bases'
        self.next_timer = 'next_timer'
        self.global_deferrable = 'tvec_base_deferrable'

        # As of kernel 4.15, timer list structure no longer has data field
        if (major, minor) >= (4, 15):
            self.timer_has_data = False
        HZ = self.ramdump.get_config_val("CONFIG_HZ")

        if HZ != None:
            self.HZ = float(HZ)
        else:
            self.HZ = 100.0

        if (major, minor) >= (4, 9):
            self.vectors = {'vectors': 512}
            self.timer_jiffies = 'clk'
            self.tvec_base = 'struct timer_base'
            self.tvec_bases = 'timer_bases'
            self.next_timer = 'next_expiry'
            self.global_deferrable = 'timer_base_deferrable'
        # Timerlist structure changed in kernel 4.2
        # Requires separate processing
        if (major, minor) >= (4, 2):
            self.timer_42 = True


    def timer_list_walker(self, node, type, index, base):
        if node == self.head:
            return

        remarks = ''
        function_addr = node + self.ramdump.field_offset('struct timer_list', 'function')
        expires_addr = node + self.ramdump.field_offset('struct timer_list', 'expires')
        try:
            function =  self.ramdump.unwind_lookup(self.ramdump.read_word(function_addr))[0]
        except TypeError:
            function = "<dynamic module>"
        expires = self.ramdump.read_word(expires_addr)

        if self.timer_has_data:
            data_addr = node + self.ramdump.field_offset('struct timer_list', 'data')
            try:
                data = hex(self.ramdump.read_word(data_addr)).rstrip('L')
            except TypeError:
                self.output_file.write("+ Corruption detected at index {0} in {1} list, found corrupted value: {2:x}\n".format(index, type, data_addr))
                return
        else:
           data = ""

        if function.split('[')[0] == "delayed_work_timer_fn":
            timer_list_offset = self.ramdump.field_offset('struct delayed_work', 'timer')
            work_addr = node - timer_list_offset
            func_addr = work_addr + self.ramdump.field_offset('struct work_struct', 'func')
            try:
                work_func = self.ramdump.unwind_lookup(self.ramdump.read_word(func_addr))[0]
                if self.timer_has_data:
                    data += " / " + work_func
                else:
                    data = work_func
            except TypeError:
                if self.timer_has_data:
                    data += " / " + hex(self.ramdump.read_word(func_addr)) + "<MODULE>"
                else:
                    data = hex(self.ramdump.read_word(func_addr)) + "<MODULE>"

        if not self.timer_42:
            timer_base_addr = node + self.ramdump.field_offset(
                'struct timer_list', 'base')
            timer_base = self.ramdump.read_word(timer_base_addr) & ~3
            if timer_base != base:
                remarks += "Timer Base Mismatch detected"

        expires_s = (expires-(0xFFFFFFFF - 300 * int(self.HZ)) )/(self.HZ)
        output = "\t{0:<6} {1:<18x} {2:<14} {3:<14} {4:<40} {5:<52} {6}\n".format(index, node, expires, str(expires_s) + 's', function, data, remarks)
        self.output.append(output)

    def iterate_vec(self, type, base):
        vec_addr = base + self.ramdump.field_offset(self.tvec_base, type)
        for i in range(0, self.vectors[type]):
            index = self.ramdump.array_index(vec_addr, 'struct list_head', i)
            self.head = index
            node_offset = self.ramdump.field_offset('struct list_head', 'next')
            timer_list_walker = linux_list.ListWalker(self.ramdump, index, node_offset)
            timer_list_walker.walk(index, self.timer_list_walker, type, i, base)

    def iterate_vec_v2(self, type, base):
        vec_addr = base + self.ramdump.field_offset(self.tvec_base, type)
        for i in range(0, self.vectors[type]):
            index = self.ramdump.array_index(vec_addr, 'struct hlist_head', i)
            self.head = index
            index = self.ramdump.read_word(index)
            node_offset = self.ramdump.field_offset(
                'struct hlist_node', 'next')
            timer_list_walker = linux_list.ListWalker(self.ramdump, index,
                                                      node_offset)
            timer_list_walker.walk(index, self.timer_list_walker, type, i,
                                   base)

    def print_vec(self, type):
        headers = ['INDEX', 'TIMER_LIST_ADDR', 'EXPIRES', 'FUNCTION', 'DATA/WORK', 'REMARKS']
        if not self.timer_has_data:
            headers[4] = 'WORK'
        if len(self.output):
            self.output_file.write("+ {0} Timers ({1})\n\n".format(type, len(self.output)))
            self.output_file.write("\t{0:6} {1:18} {2:14} {3:14} {4:40} {5:52} {6}\n".format(headers[0], headers[1], headers[2], 'EXPIRES(s)', headers[3], headers[4], headers[5]))
            for out in self.output:
                self.output_file.write(out)
            self.output_file.write("\n")
        else:
            self.output_file.write("+ No {0} Timers found\n\n".format(type))

    def get_timer_list(self):
        self.output_file.write("Timer List Dump\n\n")

        tvec_base_deferral_addr = self.ramdump.address_of(self.global_deferrable)
        if tvec_base_deferral_addr:
            timer_jiffies_addr = tvec_base_deferral_addr + self.ramdump.field_offset(self.tvec_base, self.timer_jiffies)
            next_timer_addr = tvec_base_deferral_addr + self.ramdump.field_offset(self.tvec_base, self.next_timer)

            timer_jiffies = self.ramdump.read_word(timer_jiffies_addr)
            next_timer = self.ramdump.read_word(next_timer_addr)
            active_timers_offset = self.ramdump.field_offset(self.tvec_base, 'active_timers')
            if active_timers_offset is not None:
                    active_timers_addr = tvec_base_deferral_addr + self.ramdump.field_offset(self.tvec_base, 'active_timers')
                    active_timers = self.ramdump.read_word(active_timers_addr)
            else:
                    active_timers = "NA"

            title = "(deferrable_base: {0:x} ".format(tvec_base_deferral_addr)
            title += "timer_jiffies: {0} next_timer: {1} active_timers: {2})\n".format(timer_jiffies, next_timer, active_timers)
            self.output_file.write("-" * len(title) + "\n")
            self.output_file.write(title)
            self.output_file.write("-" * len(title) + "\n\n")

            for vec in sorted(self.vectors):
                self.output = []
                if self.timer_42:
                      self.iterate_vec_v2(vec, tvec_base_deferral_addr)
                else:
                     self.iterate_vec(vec, tvec_base_deferral_addr)
                self.print_vec(vec)

        tvec_bases_addr = self.ramdump.address_of(self.tvec_bases)
        for cpu in self.ramdump.iter_cpus():
            title = "CPU {0}".format(cpu)

            base_addr = tvec_bases_addr + self.ramdump.per_cpu_offset(cpu)
            if self.timer_42:
                base = base_addr
            else:
                base = self.ramdump.read_word(base_addr)

            title += "(tvec_base: {0:x} ".format(base)

            timer_jiffies_addr = base + self.ramdump.field_offset(self.tvec_base, self.timer_jiffies)
            next_timer_addr = base + self.ramdump.field_offset(self.tvec_base, self.next_timer)

            timer_jiffies = self.ramdump.read_word(timer_jiffies_addr)
            next_timer = self.ramdump.read_word(next_timer_addr)
            active_timers_offset = self.ramdump.field_offset(self.tvec_base, 'active_timers')
            if active_timers_offset is not None:
                active_timers_addr = base + self.ramdump.field_offset(self.tvec_base, 'active_timers')
                active_timers = self.ramdump.read_word(active_timers_addr)
            else:
                active_timers = "NA"

            timer_jiffies_s = (timer_jiffies-(0xFFFFFFFF - 300 * int(self.HZ)) )/(self.HZ)
            next_timer_s = (next_timer-(0xFFFFFFFF - 300 * int(self.HZ)) )/(self.HZ)

            title += "timer_jiffies: {0}({1}s) next_timer: {2}({3}s) active_timers: {4})\n".format(timer_jiffies, timer_jiffies_s, next_timer, next_timer_s, active_timers)
            self.output_file.write("-" * len(title) + "\n")
            self.output_file.write(title)
            self.output_file.write("-" * len(title) + "\n\n")

            for vec in sorted(self.vectors):
                self.output = []
                if self.timer_42:
                    self.iterate_vec_v2(vec, base)
                else:
                    self.iterate_vec(vec, base)
                self.print_vec(vec)

        tick_do_timer_cpu_addr = self.ramdump.address_of('tick_do_timer_cpu')
        tick_do_timer_cpu_val = "tick_do_timer_cpu: {0}\n".format(self.ramdump.read_int(tick_do_timer_cpu_addr))
        self.output_file.write("=" * len(tick_do_timer_cpu_val) + "\n")
        self.output_file.write(tick_do_timer_cpu_val)
        self.output_file.write("=" * len(tick_do_timer_cpu_val) + "\n")

    def hrtimer_walker(self, hrtimer_base, extra):
        if hrtimer_base == None:
            print(" %s " % ("\n rbtree corrupted \n"), file=self.output_file)
            return
        node = self.ramdump.struct_field_addr(hrtimer_base , 'struct hrtimer', 'node')
        expires = self.ramdump.read_structure_field(node, 'struct timerqueue_node', 'expires')
        function = self.ramdump.read_structure_field(hrtimer_base, 'struct hrtimer', 'function')
        function_name = self.ramdump.unwind_lookup(function)
        if function_name == None:
            function_name = 'n/a'
        _softexpires = self.ramdump.read_structure_field(hrtimer_base, 'struct hrtimer', '_softexpires')
        '''
        in some case the rb tree is corrupt, the rt_node could be a valid pointer but the member value is invalid.
        '''
        if function != None and _softexpires != None and expires != None:
            self.hrtimer_list.append([hrtimer_base, function, function_name,  _softexpires, expires])

    def get_hrtimer(self):
        print(" %s " % ("\nhrtimer info: \n"), file=self.output_file)
        hrtimer_bases_addr = self.ramdump.address_of('hrtimer_bases')
        clock_base_offset = self.ramdump.field_offset('struct hrtimer_cpu_base', 'clock_base')
        for i in self.ramdump.iter_cpus():
            hrtimer_bases = hrtimer_bases_addr + self.ramdump.per_cpu_offset(i)
            clock_base = (hrtimer_bases + clock_base_offset)
            print(" CPU %d hrtimer_bases  v.v (struct hrtimer_cpu_base)0x%x  " % (i, hrtimer_bases), file = self.output_file)
            num_of_HRTIMER_MAX_CLOCK_BASES = self.ramdump.gdbmi.get_value_of('HRTIMER_MAX_CLOCK_BASES')
            self.hrtimer_list = []
            for j in range(0, num_of_HRTIMER_MAX_CLOCK_BASES):
                hrtimer_cpu_base_index  = self.ramdump.array_index(clock_base, 'struct  hrtimer_clock_base', j)
                if hrtimer_cpu_base_index != None and hrtimer_cpu_base_index != 0:
                    print("     hrtimer_cpu_base 0x%x " %(hrtimer_cpu_base_index), file = self.output_file)
                    active_offset = self.ramdump.field_offset('struct hrtimer_clock_base', 'active')
                    active = hrtimer_cpu_base_index + active_offset
                    rb_node = self.ramdump.read_pointer(active)
                    rb_walker = rb_tree.RbTreeWalker(self.ramdump)
                    rb_walker.walk(rb_node, self.hrtimer_walker)

                self.hrtimer_list = sorted(self.hrtimer_list, key=lambda l: l[4])
                print("		 hrtimer     								function    																		_softexpires					  _softexpires" , file=self.output_file)
                for item in self.hrtimer_list:
                    hrtimer_base = item[0]
                    function = item[1]
                    function_name = item[2]
                    _softexpires = item[3]
                    expires = item[4]
                    print("         v.v (struct hrtimer *)0x%x  0x%-16x %-64s   %-32ld  %-32ld" % (
                    hrtimer_base, function, function_name, _softexpires, expires), file = self.output_file)

    def parse(self):
        self.output_file= open(self.ramdump.outdir + "/timerlist.txt", "w")
        self.get_timer_list()
        self.get_hrtimer()
        self.output_file.close()
        print_out_str("--- Wrote the output to timerlist.txt")
