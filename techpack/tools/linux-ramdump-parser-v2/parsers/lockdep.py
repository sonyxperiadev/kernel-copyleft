# Copyright (c) 2019-2021 The Linux Foundation. All rights reserved.
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
    test_bit_data = ((addr + index*ramdump.sizeof('unsigned long')) >> (nr & (BITS_PER_LONG-1)))
    #my_task_out.write("\ntest_bit : index = {:x}, test_bit : {}".format(index, test_bit_data))
    if 1 & ((addr + index*ramdump.sizeof('unsigned long')) >> (nr & (BITS_PER_LONG-1))):
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
        hl_irq_context = (hl_class_idx_full & 0x00006000) >> 13
        hl_trylock = (hl_class_idx_full & 0x00008000) >> 15
        hl_read = (hl_class_idx_full & 0x00030000) >> 16
        if hl_read:
            # Handling read write semaphores, lockdep doesn't do anything for down_reads
            # This leads to no way of differentiating if this task is waiting on read_lock
            # find the waiter from that lock to update accordingly
            try:
                lock_type = ramdump.type_of(hl_name)
                my_task_out.write("\n lock type is {}".format(lock_type))
                if "semaphore" in lock_type or "sem" in lock_type:
                    lock_struct = ramdump.container_of(hl_instance, lock_type, 'dep_map')
                    my_task_out.write("\n lock struct : {:x}".format(lock_struct))
                    rw_writer = ramdump.read_structure_field(lock_struct, lock_type, 'writer')
                    my_task_out.write("\n writer : {:x}".format(rw_writer))
                    writer_task = ramdump.read_structure_field(rw_writer, 'struct rcuwait', 'task')
                    my_task_out.write("\n writer task : {:x}, task : {:x}".format(writer_task, task))
                    if writer_task and (writer_task != task):
                        # Reader is blocked and change timestamp only if task is not the writer
                        if (ramdump.is_config_defined('CONFIG_LOCK_STAT')):
                            hl_waittime_stamp = hl_holdtime_stamp
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
                \n\tname = {15}'.format(
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
                            hl_name))
        if (ramdump.is_config_defined('CONFIG_LOCK_STAT')):
            my_task_out.write(
                '\n\twaittime_stamp = {0},\
                \n\tholdtime_stamp = {1}'.format(
                            hex(hl_waittime_stamp),
                            hex(hl_holdtime_stamp)))
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

