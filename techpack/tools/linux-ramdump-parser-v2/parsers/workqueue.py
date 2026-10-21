# Copyright (c) 2012-2015, 2017 The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import re
import linux_list
from print_out import print_out_str
from parser_util import register_parser, RamParser


@register_parser('--print-workqueues', 'Print the state of the workqueues', shortopt='-q')
class Workqueues(RamParser):

    def print_workqueue_state_3_0(self, ram_dump):
        per_cpu_offset_addr = ram_dump.address_of('__per_cpu_offset')
        global_cwq_sym_addr = ram_dump.address_of('global_cwq')

        idle_list_offset = ram_dump.field_offset(
            'struct global_cwq', 'idle_list')
        worklist_offset = ram_dump.field_offset(
            'struct global_cwq', 'worklist')
        busy_hash_offset = ram_dump.field_offset(
            'struct global_cwq', 'busy_hash')
        scheduled_offset = ram_dump.field_offset('struct worker', 'scheduled')
        worker_task_offset = ram_dump.field_offset('struct worker', 'task')
        worker_entry_offset = ram_dump.field_offset('struct worker', 'entry')
        offset_comm = ram_dump.field_offset('struct task_struct', 'comm')
        work_entry_offset = ram_dump.field_offset(
            'struct work_struct', 'entry')
        work_hentry_offset = ram_dump.field_offset('struct worker', 'hentry')
        work_func_offset = ram_dump.field_offset('struct work_struct', 'func')
        current_work_offset = ram_dump.field_offset(
            'struct worker', 'current_work')
        cpu_wq_offset = ram_dump.field_offset(
            'struct workqueue_struct', 'cpu_wq')
        unbound_gcwq_addr = ram_dump.address_of('unbound_global_cwq')

        if per_cpu_offset_addr is None:
            per_cpu_offset0 = 0
            per_cpu_offset1 = 0
        else:
            per_cpu_offset0 = ram_dump.read_word(per_cpu_offset_addr)
            per_cpu_offset1 = ram_dump.read_word(per_cpu_offset_addr + 4)

        global_cwq_cpu0_addr = global_cwq_sym_addr + per_cpu_offset0

        idle_list_addr0 = ram_dump.read_word(
            global_cwq_cpu0_addr + idle_list_offset)
        idle_list_addr1 = ram_dump.read_word(
            unbound_gcwq_addr + idle_list_offset)

        worklist_addr0 = ram_dump.read_word(
            global_cwq_cpu0_addr + worklist_offset)
        worklist_addr1 = ram_dump.read_word(
            unbound_gcwq_addr + worklist_offset)

        s = '<'
        for a in range(0, 64):
            s = s + 'I'

        busy_hash0 = ram_dump.read_string(
            global_cwq_cpu0_addr + busy_hash_offset, s)
        busy_hash1 = ram_dump.read_string(
            unbound_gcwq_addr + busy_hash_offset, s)
        busy_hash = []
        for a in busy_hash0:
            busy_hash.append(a)

        for a in busy_hash1:
            busy_hash.append(a)

        for k in range(0, 128):
            next_busy_worker = busy_hash[k]
            if busy_hash[k] != 0:
                cnt = 0
                while True:
                    worker_addr = next_busy_worker - work_hentry_offset
                    worker_task_addr = ram_dump.read_word(
                        worker_addr + worker_task_offset)
                    if worker_task_addr is None or worker_task_addr == 0:
                        break
                    taskname = ram_dump.read_cstring(
                        worker_task_addr + offset_comm, 16)
                    scheduled_addr = ram_dump.read_word(
                        worker_addr + scheduled_offset)
                    current_work_addr = ram_dump.read_word(
                        worker_addr + current_work_offset)
                    current_work_func = ram_dump.read_word(
                        current_work_addr + work_func_offset)
                    wname = ram_dump.unwind_lookup(current_work_func)
                    if wname is not None:
                        worker_name, a = wname
                    else:
                        worker_name = 'Worker at 0x{0:x}'.format(
                            current_work_func)
                    print_out_str(
                        'BUSY Workqueue worker: {0} current_work: {1}'.format(taskname, worker_name))
                    if cnt > 200:
                        break
                    cnt += 1
                    next_busy_worker = ram_dump.read_word(
                        worker_addr + work_hentry_offset)
                    if next_busy_worker == 0:
                        break

        for i in (0, 1):
            if i == 0:
                idle_list_addr = idle_list_addr0
            else:
                idle_list_addr = idle_list_addr1
            next_entry = idle_list_addr
            while True:
                worker_addr = next_entry - worker_entry_offset
                worker_task_addr = ram_dump.read_word(
                    next_entry - worker_entry_offset + worker_task_offset)
                if worker_task_addr is None or worker_task_addr == 0:
                    break

                taskname = ram_dump.read_cstring(
                    (worker_task_addr + offset_comm), 16)
                scheduled_addr = ram_dump.read_word(
                    worker_addr + scheduled_offset)
                current_work_addr = ram_dump.read_word(
                    worker_addr + current_work_offset)
                next_entry = ram_dump.read_word(next_entry)
                if current_work_addr != 0:
                    current_work_func = ram_dump.read_word(
                        current_work_addr + work_func_offset)
                    wname = ram_dump.unwind_lookup(current_work_func)
                    if wname is not None:
                        current_work_name, foo = wname
                    else:
                        current_work_name = 'worker at 0x{0:x}'.format(
                            current_work_func)
                else:
                    current_work_func = 0
                    current_work_name = '(null)'

                if next_entry == idle_list_addr:
                    break

                print_out_str('IDLE Workqueue worker: {0} current_work: {1}'.format(
                    taskname, current_work_name))
                if scheduled_addr == (worker_addr + scheduled_offset):
                    continue

                if (next_entry == idle_list_addr):
                    break

        print_out_str('Pending workqueue info')
        for i in (0, 1):
            if i == 0:
                worklist_addr = worklist_addr0
            else:
                worklist_addr = worklist_addr1
            next_work_entry = worklist_addr
            while True:
                work_func_addr = ram_dump.read_word(
                    next_work_entry - work_entry_offset + work_func_offset)
                next_work_temp = ram_dump.read_word(next_work_entry)
                if next_work_temp == next_work_entry:
                    print_out_str('!!! Cycle in workqueue!')
                    break
                next_work_entry = next_work_temp

                if ram_dump.virt_to_phys(work_func_addr) != 0:
                    wname = ram_dump.unwind_lookup(work_func_addr)
                    if wname is not None:
                        work_func_name, foo = wname
                    else:
                        work_func_name = 'worker at 0x{0:x}'.format(
                            work_func_addr)
                    if i == 0:
                        print_out_str(
                            'Pending unbound entry: {0}'.format(work_func_name))
                    else:
                        print_out_str(
                            'Pending bound entry: {0}'.format(work_func_name))
                if next_work_entry == worklist_addr:
                    break

    def print_workqueue_state_3_7(self, ram_dump):
        per_cpu_offset_addr = ram_dump.address_of('__per_cpu_offset')
        global_cwq_sym_addr = ram_dump.address_of('global_cwq')

        pools_offset = ram_dump.field_offset('struct global_cwq', 'pools')
        worklist_offset = ram_dump.field_offset(
            'struct global_cwq', 'worklist')
        busy_hash_offset = ram_dump.field_offset(
            'struct global_cwq', 'busy_hash')
        scheduled_offset = ram_dump.field_offset('struct worker', 'scheduled')
        worker_task_offset = ram_dump.field_offset('struct worker', 'task')
        worker_entry_offset = ram_dump.field_offset('struct worker', 'entry')
        offset_comm = ram_dump.field_offset('struct task_struct', 'comm')
        work_entry_offset = ram_dump.field_offset(
            'struct work_struct', 'entry')
        work_hentry_offset = ram_dump.field_offset('struct worker', 'hentry')
        work_func_offset = ram_dump.field_offset('struct work_struct', 'func')
        current_work_offset = ram_dump.field_offset(
            'struct worker', 'current_work')
        cpu_wq_offset = ram_dump.field_offset(
            'struct workqueue_struct', 'cpu_wq')
        pool_idle_offset = ram_dump.field_offset(
            'struct worker_pool', 'idle_list')
        worker_pool_size = ram_dump.sizeof('struct worker_pool')
        pending_work_offset = ram_dump.field_offset(
            'struct worker_pool', 'worklist')
        cpus = ram_dump.get_num_cpus()

        s = '<'
        for a in range(0, 64):
            s = s + 'I'

        for i in range(0, cpus):
            busy_hash = []
            if per_cpu_offset_addr is None:
                offset = 0
            else:
                offset = ram_dump.read_word(per_cpu_offset_addr + 4 * i)
            workqueue_i = global_cwq_sym_addr + offset
            busy_hashi = ram_dump.read_string(
                workqueue_i + busy_hash_offset, s)
            for a in busy_hashi:
                busy_hash.append(a)

            for k in range(0, 64):
                next_busy_worker = busy_hash[k]
                if busy_hash[k] != 0:
                    cnt = 0

                    while True:
                        worker_addr = next_busy_worker - work_hentry_offset
                        worker_task_addr = ram_dump.read_word(
                            worker_addr + worker_task_offset)
                        if worker_task_addr is None or worker_task_addr == 0:
                            break
                        taskname = ram_dump.read_cstring(
                            worker_task_addr + offset_comm, 16)
                        scheduled_addr = ram_dump.read_word(
                            worker_addr + scheduled_offset)
                        current_work_addr = ram_dump.read_word(
                            worker_addr + current_work_offset)
                        current_work_func = ram_dump.read_word(
                            current_work_addr + work_func_offset)
                        wname = ram_dump.unwind_lookup(current_work_func)
                        if wname is not None:
                            worker_name, a = wname
                        else:
                            worker_name = 'Worker at 0x{0:x}'.format(
                                current_work_func)
                        print_out_str(
                            'BUSY Workqueue worker: {0} current_work: {1}'.format(taskname, worker_name))
                        if cnt > 200:
                            break
                        cnt += 1
                        next_busy_worker = ram_dump.read_word(
                            worker_addr + work_hentry_offset)
                        if next_busy_worker == 0:
                            break

            worker_pool = workqueue_i + pools_offset
            seen = []
            # Need better way to ge the number of pools...
            for k in range(0, 2):
                worker_pool_i = worker_pool + k * worker_pool_size

                idle_list_addr = worker_pool_i + pool_idle_offset
                next_entry = ram_dump.read_word(idle_list_addr)
                while True:
                    worker_addr = next_entry - worker_entry_offset
                    worker_task_addr = ram_dump.read_word(
                        next_entry - worker_entry_offset + worker_task_offset)
                    if worker_task_addr is None or worker_task_addr == 0 or worker_task_addr in seen:
                        break

                    seen.append(worker_task_addr)

                    taskname = ram_dump.read_cstring(
                        (worker_task_addr + offset_comm), 16)
                    scheduled_addr = ram_dump.read_word(
                        worker_addr + scheduled_offset)
                    current_work_addr = ram_dump.read_word(
                        worker_addr + current_work_offset)
                    next_entry = ram_dump.read_word(next_entry)
                    if current_work_addr != 0:
                        current_work_func = ram_dump.read_word(
                            current_work_addr + work_func_offset)
                        wname = ram_dump.unwind_lookup(current_work_func)
                        if wname is not None:
                            current_work_name, foo = wname
                        else:
                            current_work_name = 'worker at 0x{0:x}'.format(
                                current_work_func)
                    else:
                        current_work_func = 0
                        current_work_name = '(null)'

                    if next_entry == idle_list_addr:
                        break

                    print_out_str(
                        'IDLE Workqueue worker: {0} current_work: {1}'.format(taskname, current_work_name))
                    if scheduled_addr == (worker_addr + scheduled_offset):
                        continue

                    if (next_entry == idle_list_addr):
                        break

                worklist_addr = worker_pool_i + pending_work_offset
                next_work_entry = worklist_addr
                while ram_dump.read_word(next_work_entry) != next_work_entry:
                    work_func_addr = ram_dump.read_word(
                        next_work_entry - work_entry_offset + work_func_offset)
                    next_work_temp = ram_dump.read_word(next_work_entry)
                    if next_work_temp == next_work_entry:
                        print_out_str('!!! Cycle in workqueue!')
                        break
                    next_work_entry = next_work_temp

                    if ram_dump.virt_to_phys(work_func_addr) != 0:
                        work_func_name, foo = ram_dump.unwind_lookup(
                            work_func_addr)
                        if i == 0:
                            print_out_str(
                                'Pending unbound entry: {0}'.format(work_func_name))
                        else:
                            print_out_str(
                                'Pending bound entry: {0}'.format(work_func_name))
                    if next_work_entry == worklist_addr:
                        break

    def walk_workers(self, worker_addr, state):
        worker_task_offset = self.ramdump.field_offset('struct worker', 'task')
        offset_comm = self.ramdump.field_offset('struct task_struct', 'comm')
        current_work_offset = self.ramdump.field_offset(
            'struct worker', 'current_work')
        work_func_offset = self.ramdump.field_offset('struct work_struct', 'func')

        worker_task_addr = self.ramdump.read_word(
                                 worker_addr + worker_task_offset)

        taskname = self.ramdump.read_cstring(
                                 worker_task_addr + offset_comm, 16)
        current_work_addr = self.ramdump.read_word(
                                 worker_addr + current_work_offset)
        current_func_addr = self.ramdump.read_word(
                                 current_work_addr + work_func_offset)
        try:
            phys = self.ramdump.virt_to_phys(current_func_addr)
            current_work_func = self.ramdump.read_word(
                                     current_work_addr + work_func_offset)
            wname = self.ramdump.unwind_lookup(current_work_func)
            if wname is not None:
                worker_name, a = wname
            else:
                worker_name = 'Worker at 0x{0:x}'.format(
                                    current_work_func)
        except:
            worker_name = '(None)'

        print_out_str(
            '{2} Workqueue worker: {0} current_work: {1}'.format(taskname, worker_name, state))

    def pending_list_walk(self, work):
        work_func_offset = self.ramdump.field_offset('struct work_struct', 'func')
        work_func_addr = self.ramdump.read_word(work + work_func_offset)

        try:
            # virt to phys may throw an exception if the virtual address is bad
            # if that happens, just skip any printing
            phys = self.ramdump.virt_to_phys(work_func_addr)
            work_func_name, foo = self.ramdump.unwind_lookup(work_func_addr)
            print_out_str(
                'Pending entry: {0}'.format(work_func_name))
        except:
            pass

    def print_workqueue_state_3_10(self, ram_dump):
        cpu_worker_pools_addr = ram_dump.address_of('cpu_worker_pools')

        busy_hash_offset = ram_dump.field_offset(
            'struct worker_pool', 'busy_hash')
        worker_entry_offset = ram_dump.field_offset('struct worker', 'entry')
        work_entry_offset = ram_dump.field_offset(
            'struct work_struct', 'entry')
        work_hentry_offset = ram_dump.field_offset('struct worker', 'hentry')
        pool_idle_offset = ram_dump.field_offset(
            'struct worker_pool', 'idle_list')
        worker_pool_size = ram_dump.sizeof('struct worker_pool')
        pending_work_offset = ram_dump.field_offset(
            'struct worker_pool', 'worklist')

        hash_size = 2 ** self.ramdump.gdbmi.get_value_of('BUSY_WORKER_HASH_ORDER')

        s = '<'
        if ram_dump.arm64:
            pool_char = 'Q'
        else:
            pool_char = 'I'
        for a in range(0, hash_size):
            s = s + pool_char

        for i in ram_dump.iter_cpus():
            busy_hash = []

            worker_pool = cpu_worker_pools_addr + ram_dump.per_cpu_offset(i)
            print_out_str('\nCPU {0}'.format(i))
            n_pools = self.ramdump.gdbmi.get_value_of('NR_STD_WORKER_POOLS')
            for k in range(0, n_pools):
                print_out_str('pool {0}'.format(k))
                worker_pool_i = worker_pool + k * worker_pool_size
                busy_hashi = ram_dump.read_string(
                    worker_pool_i + busy_hash_offset, s)
                for a in busy_hashi:
                    busy_hash.append(a)

                for k in range(0, hash_size):
                    next_busy_worker = busy_hash[k]
                    if busy_hash[k] != 0:
                        busy_list_walker = linux_list.ListWalker(ram_dump, next_busy_worker, work_hentry_offset)
                        busy_list_walker.walk(next_busy_worker, self.walk_workers, 'BUSY')

                idle_list_addr = worker_pool_i + pool_idle_offset

                idle_list_walker = linux_list.ListWalker(ram_dump, idle_list_addr, worker_entry_offset)
                idle_list_walker.walk(self.ramdump.read_word(idle_list_addr), self.walk_workers, 'IDLE')

                worklist_addr = worker_pool_i + pending_work_offset

                pending_list = linux_list.ListWalker(ram_dump, worklist_addr, work_entry_offset)
                pending_list.walk(self.ramdump.read_word(worklist_addr), self.pending_list_walk)

    def parse(self):
            major, minor, patch = self.ramdump.kernel_version
            if (major, minor) == (3, 0):
                    print_workqueue_state_3_0(self.ramdump)
            elif (major, minor) == (3, 4):
                    # somebody did a backport of 3.7 workqueue patches to msm so
                    # need to detect new vs. old versions
                    idle_list_offset = self.ramdump.field_offset(
                        'struct global_cwq', 'idle_list')
                    if idle_list_offset is None:
                        self.print_workqueue_state_3_7(self.ramdump)
                    else:
                        self.print_workqueue_state_3_0(self.ramdump)
            elif (major, minor) == (3, 7):
                    self.print_workqueue_state_3_7(self.ramdump)
            elif (major, minor) >= (3, 10):
                    self.print_workqueue_state_3_10(self.ramdump)
            else:
                    print_out_str('Kernel version {0}.{1} is not yet supported for parsing workqueues'.format(major, minor))
