# Copyright (c) 2012-2015, 2017 The Linux Foundation. All rights reserved.
# Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
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
from parser_util import register_parser, RamParser, cleanupString

@register_parser('--print-workqueues', 'Print the state of the workqueues', shortopt='-q')
class Workqueues(RamParser):
    def get_caller(self, caller):
        return self.ramdump.gdbmi.get_func_info(caller)

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
                    self.f.write(
                        'BUSY Workqueue worker: {0} current_work: {1}\n'.format(taskname, worker_name))
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

                self.f.write('IDLE Workqueue worker: {0} current_work: {1}\n'.format(
                    taskname, current_work_name))
                if scheduled_addr == (worker_addr + scheduled_offset):
                    continue

                if (next_entry == idle_list_addr):
                    break

        self.f.write('Pending workqueue info\n')
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
                    self.f.write('!!! Cycle in workqueue!\n')
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
                        self.f.write(
                            'Pending unbound entry: {0}\n'.format(work_func_name))
                    else:
                        self.f.write(
                            'Pending bound entry: {0}\n'.format(work_func_name))
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

        for i in ram_dump.iter_cpus():
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
                        self.f.write(
                            'BUSY Workqueue worker: {0} current_work: {1}\n'.format(taskname, worker_name))
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

                    self.f.write(
                        'IDLE Workqueue worker: {0} current_work: {1}\n'.format(taskname, current_work_name))
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
                        self.f.write('!!! Cycle in workqueue!\n')
                        break
                    next_work_entry = next_work_temp

                    if ram_dump.virt_to_phys(work_func_addr) != 0:
                        work_func_name, foo = ram_dump.unwind_lookup(
                            work_func_addr)
                        if i == 0:
                            self.f.write(
                                'Pending unbound entry: {0}\n'.format(work_func_name))
                        else:
                            self.f.write(
                                'Pending bound entry: {0}\n'.format(work_func_name))
                    if next_work_entry == worklist_addr:
                        break

    def walk_workers(self, worker_addr, state):
        worker_task_offset = self.ramdump.field_offset('struct worker', 'task')
        offset_comm = self.ramdump.field_offset('struct task_struct', 'comm')
        current_work_offset = self.ramdump.field_offset(
            'struct worker', 'current_work')
        work_func_offset = self.ramdump.field_offset('struct work_struct', 'func')

        try:
            last_func_offset = self.ramdump.field_offset('struct worker', 'last_func')
            last_work = self.ramdump.read_word(worker_addr + last_func_offset)
            last_func = self.ramdump.unwind_lookup(last_work)
        except Exception as e:
            last_func='(Unknown)'

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

        self.f.write(
            '{2} Workqueue worker: {0} current_work: {1} last_func: {3}\n'.format(taskname, worker_name, state, last_func))

    def pending_list_walk(self, work):
        work_func_offset = self.ramdump.field_offset('struct work_struct', 'func')
        work_func_addr = self.ramdump.read_word(work + work_func_offset)

        try:
            # virt to phys may throw an exception if the virtual address is bad
            # if that happens, just skip any printing
            work_func_name, foo = self.ramdump.unwind_lookup(work_func_addr)
            line  = self.get_caller(work_func_addr)
            self.f.write(
                '       Pending entry: v.v (struct work_struct)0x{0:x} {1}  {2}\n '.format(work, work_func_name, line))
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
            self.f.write('\nCPU {0}\n'.format(i))
            n_pools = self.ramdump.gdbmi.get_value_of('NR_STD_WORKER_POOLS')
            for k in range(0, n_pools):
                self.f.write('pool {0}\n'.format(k))
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

    def get_workqueues_func(self, workqueue_struct_base):
        name_offset = self.ramdump.field_offset('struct workqueue_struct', 'name')
        flags_offset = self.ramdump.field_offset('struct workqueue_struct', 'flags')
        name = self.ramdump.read_cstring(workqueue_struct_base + name_offset)
        flags = self.ramdump.read_int(workqueue_struct_base + flags_offset)
        aList = []
        for (d, x) in self.flags_array.items():
            if flags & x:
                aList.append(d)
        if name is None:
            name =''
        print("     v.v (struct workqueue_struct *)0x%x %-32s  flags 0x%-8x  %-64s "
              % (workqueue_struct_base, name, flags, aList), file=self.f)

    def get_workqueues_list(self):
        print("\n\n", file = self.f)
        self.flags_array = {'WQ_UNBOUND':1<<1,'WQ_FREEZABLE':1 << 2,'WQ_MEM_RECLAIM':1 << 3,
                            'WQ_HIGHPRI':1 << 4,'WQ_CPU_INTENSIVE':1 << 5,'WQ_SYSFS':1 << 6}
        workqueues = self.ramdump.address_of('workqueues')
        list_offset = self.ramdump.field_offset('struct workqueue_struct', 'list')
        list_walker = linux_list.ListWalker(self.ramdump, workqueues, list_offset)
        list_walker.walk(workqueues, self.get_workqueues_func)

    def get_busy_hash(self, worker_pool_addr):
        busy_hash_offset  = self.ramdump.field_offset('struct worker_pool', 'busy_hash')
        nr_busy_hash_entries = 64
        busy_hash_base_addr = worker_pool_addr + busy_hash_offset
        busy_hash_entry_size = self.ramdump.sizeof('struct  hlist_head')
        busy_hash_index = 0
        base_addr = busy_hash_base_addr
        busy_hash_entry = base_addr
        first_offset = self.ramdump.field_offset('struct hlist_head', 'first')
        current_work_offset  = self.ramdump.field_offset('struct worker', 'current_work')
        func_offset = self.ramdump.field_offset('struct work_struct', 'func')
        while busy_hash_index < nr_busy_hash_entries:
            first_worker_pool = self.ramdump.read_pointer(busy_hash_entry + first_offset)
            if first_worker_pool != 0:
                next_busy_worker = first_worker_pool
                current_work = self.ramdump.read_pointer(next_busy_worker + current_work_offset)
                func = self.ramdump.read_pointer(current_work + func_offset)
                wname = self.ramdump.unwind_lookup(func)
                print("     v.v (struct worker*)0x%x v.v (struct work_struct*)0x%x  %s"
                      % (next_busy_worker, current_work, wname), file=self.f)
            busy_hash_index =  busy_hash_index + 1
            busy_hash_entry =  base_addr + busy_hash_entry_size * busy_hash_index

    def get_unbound_pool_hash(self):
        unbound_pool_hash = self.ramdump.address_of('unbound_pool_hash')
        unbound_pool_hash_base = unbound_pool_hash
        hash_entry_size = self.ramdump.sizeof('struct  hlist_head')
        worker_pool_hlist_offset = self.ramdump.field_offset('struct worker_pool', 'hash_node')
        hash_index = 0
        nr_busy_hash_entries = 64
        first_offset  = self.ramdump.field_offset('struct hlist_head', 'first')
        next_offset = self.ramdump.field_offset('struct hlist_node', 'next')
        worklist_offset = self.ramdump.field_offset('struct worker_pool', 'worklist')
        print("==========>Unbound wqs", file = self.f)
        while (hash_index < nr_busy_hash_entries):
            first_worker_pool = self.ramdump.read_pointer(unbound_pool_hash + first_offset)

            if first_worker_pool != 0:
                next_worker_pool = first_worker_pool
                while next_worker_pool != 0:
                    worker_pool_addr = next_worker_pool - worker_pool_hlist_offset
                    print ( "v.v (struct worker_pool*)0x%x" %(worker_pool_addr), file=self.f)
                    worklist = (worker_pool_addr + worklist_offset)

                    list_offset = self.ramdump.field_offset('struct  work_struct', 'entry')
                    list_walker = linux_list.ListWalker(self.ramdump, worklist, list_offset)
                    list_walker.walk(worklist, self.pending_list_walk)
                    '''
                    walk the busy_hash
                    '''
                    self.get_busy_hash(worker_pool_addr)
                    next_worker_pool = self.ramdump.read_pointer(next_worker_pool + next_offset)
            hash_index = hash_index + 1
            unbound_pool_hash =  unbound_pool_hash_base + hash_entry_size * hash_index

    def parse(self):
            self.f = open(self.ramdump.outdir + "/workqueue.txt", "w")
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
                    self.f.write('Kernel version {0}.{1} is not yet supported for parsing workqueues\n'.format(major, minor))
            self.get_unbound_pool_hash()
            self.get_workqueues_list()
            self.f.close()
