# Copyright (c) 2019-2021 The Linux Foundation. All rights reserved.
# Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from parser_util import register_parser, RamParser, cleanupString
from print_out import print_out_str

def test_bit(nr, addr, ramdump, my_task_out):
    BITS_PER_LONG = 64
    if not ramdump.arm64:
        BITS_PER_LONG = 32
    index = int(nr / BITS_PER_LONG)
    data = ramdump.read_ulong(addr + index * ramdump.sizeof('unsigned long'))
    #my_task_out.write("\ntest_bit: index = 0x{:x}, data: 0x{:x}".format(index, data))
    if 1 & (data >> (nr & (BITS_PER_LONG - 1))):
        #my_task_out.write("\ntest bit returned true")
        return True
    #my_task_out.write("\ntest bit returned false")
    return False

def parse_held_locks(ramdump, task, my_task_out):
    task_held_locks_offset = ramdump.field_offset('struct task_struct',  'held_locks')
    held_locks = task + task_held_locks_offset
    sizeof_held_lock = ramdump.sizeof('struct held_lock')
    sizeof_lock_class = ramdump.sizeof('struct lock_class')
    lock_classes_in_use_bitmap = ramdump.address_of('lock_classes_in_use')
    lock_classes = ramdump.address_of('lock_classes')
    #my_task_out.write("lock_classes : {:x} , lock_classes_bitmap : {:x}, lock_class_size : {:x}".format(lock_classes, lock_classes_in_use_bitmap, sizeof_lock_class))
    task_lockdep_depth = ramdump.read_structure_field(task, 'struct task_struct', 'lockdep_depth')
    my_task_out.write('\nlockdep_depth: {0}\n'.format(hex(task_lockdep_depth)))

    for i in range (0, task_lockdep_depth):
        held_lock_indx = held_locks + (i * sizeof_held_lock)
        hl_prev_chain_key = ramdump.read_structure_field(held_lock_indx, 'struct held_lock', 'prev_chain_key')
        if not hl_prev_chain_key:
            break
        hl_acquire_ip = ramdump.read_structure_field(held_lock_indx, 'struct held_lock','acquire_ip')
        hl_acquire_ip_caller = ramdump.read_structure_field(held_lock_indx, 'struct held_lock','acquire_ip_caller')
        if hl_acquire_ip_caller is None:
            hl_acquire_ip_caller = 0x0
        hl_instance = ramdump.read_structure_field(held_lock_indx, 'struct held_lock', 'instance')
        hl_nest_lock = ramdump.read_structure_field(held_lock_indx, 'struct held_lock', 'nest_lock')
        if (ramdump.is_config_defined('CONFIG_LOCK_STAT')):
            hl_waittime_stamp = ramdump.read_structure_field(held_lock_indx, 'struct held_lock', 'waittime_stamp')
            hl_holdtime_stamp = ramdump.read_structure_field(held_lock_indx, 'struct held_lock', 'holdtime_stamp')
        hl_class_idx_full = hl_class_idx = ramdump.read_structure_field(held_lock_indx, 'struct held_lock', 'class_idx')
        hl_class_idx = hl_class_idx_full & 0x00001FFF
        hl_name = None
        if test_bit(hl_class_idx, lock_classes_in_use_bitmap, ramdump, my_task_out):
            #my_task_out.write("\nLock class stuct @ {:x}".format(lock_classes + sizeof_lock_class*hl_class_idx))
            hl_name = ramdump.read_structure_field(lock_classes + sizeof_lock_class*hl_class_idx, 'struct lock_class', 'name')
            hl_name = ramdump.read_cstring(hl_name)
        else:
            continue
        lock_type = ramdump.type_of(hl_name)
        hl_irq_context = (hl_class_idx_full & 0x00006000) >> 13
        hl_trylock = (hl_class_idx_full & 0x00008000) >> 15
        # 0 - exclusive
        # 1 - shared
        # 2 - shared_recursive
        hl_read = (hl_class_idx_full & 0x00030000) >> 16
        if hl_read:
            # Handling for percpu_rw_semaphore
            # if the task in writer is not NULL, it means that the reader is blocking the writer
            try:
                if "struct percpu_rw_semaphore" in lock_type:
                    my_task_out.write("\n lock type : {}".format(lock_type))
                    lock_struct = ramdump.container_of(hl_instance, lock_type, 'dep_map')
                    my_task_out.write("\n lock addr : 0x{:x}".format(lock_struct))
                    writer = lock_struct + ramdump.field_offset("struct percpu_rw_semaphore", "writer")
                    my_task_out.write("\n writer : 0x{:x}".format(writer))
                    writer_task = ramdump.read_structure_field(writer, 'struct rcuwait', 'task')
                    my_task_out.write("\n writer task : 0x{:x}".format(writer_task))
                    if writer_task != 0:
                        my_task_out.write("\n the reader task [Process: {0}, Pid: {1}] is blocking the writer task [Process: {2}, Pid: {3}]".format(
                            cleanupString(ramdump.read_cstring(task + ramdump.field_offset("struct task_struct", "comm"), 16)),
                            ramdump.read_int(task + ramdump.field_offset("struct task_struct", "pid")),
                            cleanupString(ramdump.read_cstring(writer_task + ramdump.field_offset("struct task_struct", "comm"), 16)),
                            ramdump.read_int(writer_task + ramdump.field_offset("struct task_struct", "pid"))))

            except Exception as err:
                my_task_out.write("\nError encountered while resolving read lock ownership")
                my_task_out.write("\n{}\n".format(err))
                pass

        hl_check = (hl_class_idx_full & 0x00040000) >> 18
        hl_hardirqs_off = (hl_class_idx_full & 0x00080000) >> 19
        hl_references = (hl_class_idx_full & 0xFFF00000) >> 20
        hl_pin_count = ramdump.read_structure_field(held_lock_indx, 'struct held_lock', 'pin_count')
        if (ramdump.is_config_defined('CONFIG_LOCKDEP_CROSSRELEASE')):
            hl_gen_id = ramdump.read_structure_field(held_lock_indx, 'struct held_lock', 'gen_id')
        hl_acquire_ip_name_func = 'n/a'
        wname = ramdump.unwind_lookup(hl_acquire_ip)
        if wname is not None:
            hl_acquire_ip_name_func, a = wname


        my_task_out.write(
                '\nheld_locks[{0}] [0x{1:x}]:\
                \n\tprev_chain_key = {2},\
                \n\tacquire_ip = {3},\
                \n\tacquire_ip_caller = {4},\
                \n\tinstance = {5},\
                \n\tnest_lock = {6}\
                \n\tclass_idx = {7},\
                \n\tirq_context = {8},\
                \n\ttrylock = {9},\
                \n\tread = {10},\
                \n\tcheck = {11},\
                \n\thardirqs_off = {12},\
                \n\treferences = {13},\
                \n\tpin_count = {14},\
                \n\tname = {15},\
                \n\tacquire_ip_func = {16}'.format(
                            i, held_lock_indx,
                            hex(hl_prev_chain_key),
                            hex(hl_acquire_ip),
                            hex(hl_acquire_ip_caller),
                            hex(hl_instance),
                            hex(hl_nest_lock),
                            hex(hl_class_idx),
                            hex(hl_irq_context),
                            hex(hl_trylock),
                            hex(hl_read),
                            hex(hl_check),
                            hex(hl_hardirqs_off),
                            hex(hl_references),
                            hex(hl_pin_count),
                            hl_name,
                            hl_acquire_ip_name_func))
        if (ramdump.is_config_defined('CONFIG_LOCK_STAT')):
            my_task_out.write(
                '\n\twaittime_stamp = {0}s\
                \n\tholdtime_stamp = {1}s'.format(
                            (hl_waittime_stamp / 1000000000.0),
                            (hl_holdtime_stamp / 1000000000.0)))

            """
            #define LOCK_CONTENDED(_lock, try, lock)			\
            do {								\
                if (!try(_lock)) {					\
                    lock_contended(&(_lock)->dep_map, _RET_IP_);	\
                    lock(_lock);					\
                }							\
                lock_acquired(&(_lock)->dep_map, _RET_IP_);			\
            } while (0)

            void __sched down_read(struct rw_semaphore *sem)
            {
                might_sleep();
                rwsem_acquire_read(&sem->dep_map, 0, 0, _RET_IP_);

                LOCK_CONTENDED(sem, __down_read_trylock, __down_read);
            }
            """
            # for rwlock_t and rw_semaphore, lock operation is expanded by macro LOCK_CONTENDED
            # there are 3 cases: here assume holdtime_stamp value is initialized to 100 and waittime_stamp will be initialized to 0.
            # 1. if success to acquire lock in try(_lock), waittime_stamp value won't be updated. also, holdtime_stamp will not be updated in lock_acquired().
            #    waittime_stamp: 0
            #    holdtime_stamp: 100
            # 2. if fail to acquire lock in try(_lock), waittime_stamp will be updated in lock_contended(). here assume it's updated to 105.
            #    a. if success to acquire lock in lock(_lock), holdtime_stamp will be updated in lock_acquired(). here assume it's updated to 110.
            #       waittime_stamp: 105
            #       holdtime_stamp: 110
            #    b. if fail to acquire lock in lock(_lock), will stuck at lock(_lock).
            #       waittime_stamp: 105
            #       holdtime_stamp: 100
            # based on above, we can say that if waittime_stamp is greater than holdtime_stamp, the lock is not acquired.
            if lock_type and ('struct rwlock_t' in lock_type or 'struct rw_semaphore' in lock_type):
                if hl_waittime_stamp > hl_holdtime_stamp:
                    lock_acquired = 0
                else:
                    lock_acquired = 1
                my_task_out.write(
                    '\n\tlock_type = {0}\
                    \n\tlock_acquired = {1}'.format(lock_type, lock_acquired))

        if (ramdump.is_config_defined('CONFIG_LOCKDEP_CROSSRELEASE')):
            my_task_out.write(
                '\n\tgen_id = {0}'.format( hex(hl_gen_id)))
        my_task_out.write('\n\n')

def parse_mytaskstruct(ramdump):
    my_task_out = ramdump.open_file('lockdep.txt')
    my_task_out.write('============================================\n')
    task_comm_offset = ramdump.field_offset('struct task_struct',  'comm')
    task_pid_offset = ramdump.field_offset('struct task_struct',  'pid')

    for process in ramdump.for_each_process():
        for task in ramdump.for_each_thread(process):
            thread_comm = task + task_comm_offset
            thread_task_name = cleanupString(ramdump.read_cstring(thread_comm, 16))
            thread_pid = task + task_pid_offset
            thread_task_pid = ramdump.read_int(thread_pid)
            my_task_out.write('\nProcess: {0}, [Pid: {1} Task: 0x{2:x}]\n'.format(
                                                        thread_task_name, thread_task_pid, task))
            parse_held_locks(ramdump, task, my_task_out)

    my_task_out.write('============================================\n')
    my_task_out.close()
    print_out_str('----wrote lockdep held locks info')

@register_parser('--lockdep-heldlocks', 'Extract lockdep held locks info per task from ramdump')

class LockdepParser(RamParser):
        def parse(self):
            if (self.ramdump.is_config_defined('CONFIG_LOCKDEP')):
                print_out_str('----dumping lockdep held locks info')
                parse_mytaskstruct(self.ramdump)
            else:
                print_out_str('CONFIG_LOCKDEP not present')

