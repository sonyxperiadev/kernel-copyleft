# Copyright (c) 2013-2021, The Linux Foundation. All rights reserved.
# Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import rb_tree
from print_out import print_out_str
from parser_util import register_parser, RamParser


@register_parser('--print-runqueues', 'Print the runqueue status')
class RunQueues(RamParser):

    def __init__(self, *args):
        super(RunQueues, self).__init__(*args)
        self.domain_list = []
        self.tab_offset = 0

    def ffs(self, num):
        # Check there is at least one bit set.
        if num == 0:
            return None
        # Right-shift until we have the first set bit in the LSB position.
        i = 0
        while (num % 2) == 0:
            i += 1
            num = num >> 1
        return i

    def print_out_str_with_tab(self, string):
        string = self.tab_offset * ' |  ' + ' |--' + string
        print_out_str(string)

    def print_task_state(self, status, task_addr):
        pid_offset = self.ramdump.field_offset('struct task_struct', 'pid')
        comm_offset = self.ramdump.field_offset('struct task_struct', 'comm')

        if self.ramdump.kernel_version > (5, 2, 0):
            affinity_offset = self.ramdump.field_offset('struct task_struct', 'cpus_mask')
        else:
            affinity_offset = self.ramdump.field_offset('struct task_struct', 'cpus_allowed')

        if 0 < task_addr:
            pid = self.ramdump.read_int(task_addr + pid_offset)
            taskname = self.ramdump.read_cstring(task_addr + comm_offset, 16)
            affinity = self.ramdump.read_u64(task_addr + affinity_offset)
            se_offset = self.ramdump.field_offset('struct task_struct', 'se')
            if se_offset is None:
                vruntime = 0
            else:
                se = (task_addr + se_offset)
                vruntime_offset = self.ramdump.field_offset('struct sched_entity', 'vruntime')
                vruntime = self.ramdump.read_u64(se + vruntime_offset)
                if self.ramdump.is_config_defined('CONFIG_FAIR_GROUP_SCHED'):
                    cfs_rq_offset = self.ramdump.field_offset('struct sched_entity', 'cfs_rq')
                    cfs_rq = None
                    if se is not None and se > 0:
                        cfs_rq = self.ramdump.read_word(se + cfs_rq_offset)
                        tg_offset = self.ramdump.field_offset('struct cfs_rq', 'tg')
                    tg = None
                    if cfs_rq is not None and cfs_rq > 0:
                        tg = self.ramdump.read_word(cfs_rq + tg_offset)
                        css_offset = self.ramdump.field_offset('struct task_group', 'css')
                    css = None
                    if tg is not None and tg > 0:
                        css = self.ramdump.read_word(tg + css_offset)
                        cgroup_offset = self.ramdump.field_offset('struct cgroup_subsys_state', 'cgroup')
                    cgroup = None
                    if css is not None and css > 0:
                        cgroup = self.ramdump.read_word(css + cgroup_offset)
                        kn_offset = self.ramdump.field_offset('struct cgroup', 'kn')
                    kn = None
                    if cgroup is not None and cgroup > 0:
                        kn = self.ramdump.read_word(cgroup + kn_offset)
                        name_offset = self.ramdump.field_offset('struct kernfs_node', 'name')
                    name = 'n/a'
                    if kn is not None and kn > 0:
                        name_addr = self.ramdump.read_word(kn + name_offset)
                        name = self.ramdump.read_cstring(name_addr)
                else:
                    name ='n/a'
            print_out_str(
                '{0}: {1:16s}({2:6d}) [affinity=0x{3:02x}] [vruntime={4:16d}] {5:s}'.format(status, taskname, pid, affinity, vruntime, name))

        else:
            self.print_out_str_with_tab('{0}: None(0)'.format(status))

    def print_cgroup_state(self, status, se_addr):
        se_offset = self.ramdump.field_offset('struct task_struct', 'se')
        cfs_nr_running_offset = self.ramdump.field_offset(
            'struct cfs_rq', 'nr_running')
        my_q_offset = self.ramdump.field_offset('struct sched_entity', 'my_q')

        if se_addr == 0 or my_q_offset is None:
            task = self.ramdump.container_of(se_addr, 'struct task_struct', 'se')
            self.print_task_state(status, task)
        else:
            my_q_addr = self.ramdump.read_word(se_addr + my_q_offset)
            if my_q_addr == 0:
                self.print_task_state(status, se_addr - se_offset)
            else:
                cfs_nr_running = self.ramdump.read_int(
                    my_q_addr + cfs_nr_running_offset)
                self.print_out_str_with_tab(
                    '{0}: {1} process is grouping'.format(status, cfs_nr_running))
                self.tab_offset += 1
                self.print_cfs_state(my_q_addr)
                self.tab_offset -= 1

    def cfs_node_func(self, node, extra):
        run_node_offset = self.ramdump.field_offset(
            'struct sched_entity', 'run_node')

        task_se = node - run_node_offset
        self.print_cgroup_state('pend', task_se)

    def print_cfs_state(self, cfs_rq_addr):
        tasks_timeline_offset = self.ramdump.field_offset(
            'struct cfs_rq', 'tasks_timeline')
        curr_offset = self.ramdump.field_offset('struct cfs_rq', 'curr')
        next_offset = self.ramdump.field_offset('struct cfs_rq', 'next')
        last_offset = self.ramdump.field_offset('struct cfs_rq', 'last')
        skip_offset = self.ramdump.field_offset('struct cfs_rq', 'skip')

        tasks_timeline_addr = self.ramdump.read_word(
            cfs_rq_addr + tasks_timeline_offset)

        curr_se = self.ramdump.read_word(cfs_rq_addr + curr_offset)
        self.print_cgroup_state('curr', curr_se)
        next_se = self.ramdump.read_word(cfs_rq_addr + next_offset)
        self.print_cgroup_state('next', next_se)
        if last_offset:
            last_se = self.ramdump.read_word(cfs_rq_addr + last_offset)
            self.print_cgroup_state('last', last_se)
        if skip_offset:
            skip_se = self.ramdump.read_word(cfs_rq_addr + skip_offset)
            self.print_cgroup_state('skip', skip_se)

        rb_walker = rb_tree.RbTreeWalker(self.ramdump)
        rb_walker.walk(tasks_timeline_addr, self.cfs_node_func)

    def print_rt_cgroup_state(self, status, rt_addr):
        rt_offset = self.ramdump.field_offset('struct task_struct', 'rt')
        rt_nr_running_offset = self.ramdump.field_offset(
            'struct rt_rq', 'nr_running')
        my_q_offset = self.ramdump.field_offset(
            'struct sched_rt_entity', 'my_q')

        if rt_addr == 0:
            self.print_task_state(status, se_addr)
        else:
            my_q_addr = self.ramdump.read_word(rt_addr + my_q_offset)
            if my_q_addr == 0:
                self.print_task_state(status, rt_addr - rt_offset)
            else:
                rt_nr_running = self.ramdump.read_word(
                    my_q_addr + rt_nr_running_offset)
                self.print_out_str_with_tab(
                    '{0}: {1} process is grouping'.format(status, rt_nr_running))
                self.tab_offset += 1
                self.print_rt_state(my_q_addr)
                self.tab_offset -= 1

    def print_rt_state(self, rt_rq_addr):
        active_offset = self.ramdump.field_offset('struct rt_rq', 'active')
        queue_offset = self.ramdump.field_offset(
            'struct rt_prio_array', 'queue')
        rt_offset = self.ramdump.field_offset('struct task_struct', 'rt')

        array_addr = rt_rq_addr + active_offset

        if self.ramdump.arm64:
            bitmap_range = 2
            array_wsize = 8
            idx_size = 64
        else:
            bitmap_range = 4
            array_wsize = 4
            idx_size = 32

        seen_nodes = set()
        for i in range(0, bitmap_range):
            bitmap = self.ramdump.read_word(array_addr + i * array_wsize)
            while True:
                idx = self.ffs(bitmap)
                if idx is not None and idx + i * idx_size < 100:
                    bitmap &= ~(1 << idx)
                    idx = (idx + i * idx_size) * array_wsize * 2
                    queue_addr = self.ramdump.read_word(
                        array_addr + queue_offset + idx)
                    while queue_addr != array_addr + queue_offset + idx:
                        if queue_addr in seen_nodes:
                            break
                        seen_nodes.add(queue_addr)
                        task_addr = queue_addr - rt_offset
                        self.print_task_state('pend', task_addr)
                        queue_addr = self.ramdump.read_word(queue_addr)
                else:
                    break

    def print_latest_callstack_maybe(self, task_addr):
        text_start_addr = self.ramdump.address_of('_text')
        text_end_addr = self.ramdump.address_of('_etext')
        stack_offset = self.ramdump.field_offset('struct task_struct', 'stack')

        stack_addr = self.ramdump.read_word(task_addr + stack_offset)
        print_out_str('current callstack is maybe:')

        if self.ramdump.arm64:
            stack_align = 8
            stack_size = 0x4000
        else:
            stack_align = 4
            stack_size = 0x2000

        for i in range(stack_addr, stack_addr + stack_size, stack_align):
            callstack_addr = self.ramdump.read_word(i)
            if text_start_addr <= callstack_addr and callstack_addr < text_end_addr:
                wname = self.ramdump.unwind_lookup(callstack_addr)
                if wname is not None:
                    print_out_str('0x{0:x}:{1}'.format(i, wname))

    def print_md_latest_call_stack(self):
        text_start_addr = self.ramdump.address_of('_text')
        text_end_addr = self.ramdump.address_of('_etext')
        minidump_stack_addr = next((s for s in self.ramdump.elffile.iter_sections() if s.name == 'KSTACK0_0'), None)
        if minidump_stack_addr is None:
            core_stack_addr = self.ramdump.address_of('md_stack_data')
            if core_stack_addr is None:
                print_out_str("\nCurrent call stack support is not present\n")
                return
        print_out_str('\ncurrent callstack is maybe\n')
        no_of_cpus = self.ramdump.get_num_cpus()
        index = 0
        while index < no_of_cpus:
            if minidump_stack_addr is None:
                md_stack_addr = core_stack_addr + self.ramdump.per_cpu_offset(index)
                if self.ramdump.arm64:
                    md_stack_addr = (md_stack_addr & 0xffffffffffffffff)
                else:
                    md_stack_addr = (md_stack_addr & 0xffffffff)
                stack_mdr = md_stack_addr + self.ramdump.field_offset('struct md_stack_cpu_data', 'stack_mdr')
                stack_virt_addr = stack_mdr + self.ramdump.field_offset('struct md_region', 'virt_addr')
                stack_virt_addr = self.ramdump.read_u64(stack_virt_addr)

            if self.ramdump.arm64:
                stack_align = 8
                stack_size = 0x1000
                loop = 4
            else:
                stack_align = 4
                stack_size = 0x1000
                loop = 2
            print_out_str('\nCore_{} call stack :\n'.format(index))
            for i in range(loop):
                if minidump_stack_addr is not None:
                    section_name = 'KSTACK{0}_{1}'.format(index, i)
                    minidump_stack_addr = next((s for s in self.ramdump.elffile.iter_sections() if s.name == section_name), None)
                    stack_virt_addr = minidump_stack_addr['sh_addr']
                for j in range(stack_virt_addr, stack_virt_addr + stack_size, stack_align):
                    callstack_addr = self.ramdump.read_word(j)
                    if callstack_addr is None:
                        continue
                    if text_start_addr <= callstack_addr and callstack_addr < text_end_addr:
                        wname = self.ramdump.unwind_lookup(callstack_addr)
                        if wname is not None:
                            print_out_str('0x{0:x}:{1}'.format(j, wname))
                if minidump_stack_addr is None:
                    stack_mdr = stack_mdr + self.ramdump.sizeof('struct md_region')
                    stack_virt_addr = stack_mdr + self.ramdump.field_offset('struct md_region', 'virt_addr')
                    stack_virt_addr = self.ramdump.read_u64(stack_virt_addr)
            index = index + 1


    def print_irq_context(self):
        text_start_addr = self.ramdump.address_of('_text')
        text_end_addr = self.ramdump.address_of('_etext')
        print_out_str('\ncurrent irq context callstack is maybe\n')
        irq_stack_addr = self.ramdump.address_of('irq_stack_ptr')
        if irq_stack_addr is None:
            print_out_str("\nCurrent call stack support is not present\n")
            return
        no_of_cpus = self.ramdump.get_num_cpus()
        index = 0
        for index in self.ramdump.iter_cpus():
            stack_addr = irq_stack_addr + self.ramdump.per_cpu_offset(index)
            if self.ramdump.arm64:
                stack_addr = stack_addr & 0xffffffffffffffff
            else:
                stack_addr = stack_addr & 0xFFFFFFFF

            stack_addr = self.ramdump.read_u64(stack_addr)

            if self.ramdump.arm64:
                stack_align = 8
                stack_size = 0x4000
            else:
                stack_align = 4
                stack_size = 0x2000

            print_out_str('\nCore_{} call stack :\n'.format(index))

            for i in range(stack_addr, stack_addr + stack_size, stack_align):
                callstack_addr = self.ramdump.read_word(i)
                if callstack_addr is None:
                    continue
                if text_start_addr <= callstack_addr and callstack_addr < text_end_addr:
                    wname = self.ramdump.unwind_lookup(callstack_addr)
                    if wname is not None:
                        print_out_str('0x{0:x}:{1}'.format(i, wname))


    def parse(self):
        print_out_str(
            '======================= RUNQUEUE STATE ============================')
        if self.ramdump.minidump:
            self.print_md_latest_call_stack()
            self.print_irq_context()
            return
        runqueues_addr = self.ramdump.address_of('runqueues')
        nr_running_offset = self.ramdump.field_offset(
            'struct rq', 'nr_running')
        curr_offset = self.ramdump.field_offset('struct rq', 'curr')
        idle_offset = self.ramdump.field_offset('struct rq', 'idle')
        stop_offset = self.ramdump.field_offset('struct rq', 'stop')
        cfs_rq_offset = self.ramdump.field_offset('struct rq', 'cfs')
        rt_rq_offset = self.ramdump.field_offset('struct rq', 'rt')
        cfs_nr_running_offset = self.ramdump.field_offset(
            'struct cfs_rq', 'nr_running')
        rt_nr_running_offset = self.ramdump.field_offset(
            'struct rt_rq', 'rt_nr_running')

        for i in self.ramdump.iter_cpus():
            rq_addr = runqueues_addr + self.ramdump.per_cpu_offset(i)
            nr_running = self.ramdump.read_int(rq_addr + nr_running_offset)
            print_out_str(
                'CPU{0} {1} process is running'.format(i, nr_running))
            curr_addr = self.ramdump.read_word(rq_addr + curr_offset)
            self.print_task_state('curr', curr_addr)
            idle_addr = self.ramdump.read_word(rq_addr + idle_offset)
            self.print_task_state('idle', idle_addr)
            stop_addr = self.ramdump.read_word(rq_addr + stop_offset)
            self.print_task_state('stop', stop_addr)

            cfs_rq_addr = rq_addr + cfs_rq_offset
            cfs_nr_running = self.ramdump.read_int(
                cfs_rq_addr + cfs_nr_running_offset)
            print_out_str('CFS {0} process is pending'.format(cfs_nr_running))
            self.print_cfs_state(cfs_rq_addr)

            rt_rq_addr = rq_addr + rt_rq_offset
            rt_nr_running = self.ramdump.read_int(
                rt_rq_addr + rt_nr_running_offset)
            print_out_str('RT {0} process is pending'.format(rt_nr_running))
            self.print_rt_state(rt_rq_addr)

            self.print_latest_callstack_maybe(curr_addr)
            print_out_str('')
        self.print_irq_context()
