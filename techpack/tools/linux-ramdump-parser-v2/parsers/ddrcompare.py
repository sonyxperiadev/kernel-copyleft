# Copyright (c) 2014-2015, 2017, 2019-2020, The Linux Foundation. All rights reserved.
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
import struct
import re
import sys
import subprocess
from print_out import print_out_str
from parser_util import register_parser, RamParser

@register_parser('--ddr-compare', 'Sanity check the DDR data to find possible corruptions')
class DDRCompare(RamParser) :

    def compare_magic(self):
        self.output_file.write("----------------------------------------------------------------------------------------\n")
        self.output_file.write("Comparing statically initialized lock values from vmlinux and ramdumps\n")
        self.output_file.write("----------------------------------------------------------------------------------------\n")

        if not self.ramdump.is_config_defined('CONFIG_DEBUG_SPINLOCK'):
            self.output_file.write('Kernel Configuration for debug spinlocks is not enabled, cannot comapre the magic values!!\n\n')
            return
        elif self.ramdump.objdump_path is None:
            self.output_file.write("!!! Objdump path is not set, please use --objdump-path option to specify the path\n\n")
            return

        cmdarr = [self.ramdump.objdump_path, "-D", "-j.data", self.ramdump.vmlinux]
        p = subprocess.Popen(cmdarr, stdout=subprocess.PIPE, universal_newlines=True)
        output = p.communicate()[0]

        foundcorruption = 0;
        for line in output.split('\n'):
            m = re.search("^(.*?):\s+(dead4ead|deaf1eed?)\s+\.word\s+(0x\\2?)", line)
            if m:
                virtual = m.group(1)
                virtual = int(m.group(1), 16)
                bitcheck = virtual & 0x3
                if bitcheck:
                    virtual = virtual - bitcheck
                physical = self.ramdump.virt_to_phys(virtual + self.ramdump.get_kaslr_offset())

                magic = hex(self.ramdump.read_u32(physical, False)).rstrip("L").lstrip("0x").zfill(8)
                if (m.group(2) != magic):
                    foundcorruption = 1;
                    self.output_file.write("Magic didn't match for virtual address {0}\n".format("0x"+m.group(1)))
                    for i in range(2):
                        physical = physical - 4
                        dumpvalue = hex(self.ramdump.read_u32(physical, False)).rstrip("L").lstrip("0x").zfill(8)
                        self.output_file.write("{0}\n".format(dumpvalue))
                    physical = physical + 8
                    self.output_file.write("{0}\n".format(magic))
                    for i in range(2):
                        physical = physical + 4
                        dumpvalue = hex(self.ramdump.read_u32(physical, False)).rstrip("L").lstrip("0x").zfill(8)
                        self.output_file.write("{0}\n".format(dumpvalue))

        if (foundcorruption == 0):
            self.output_file.write("No Corruption found in the lock values\n\n")

    def validate_sched_class(self, address):
        sc_stop = self.ramdump.address_of('stop_sched_class')
        sc_rt = self.ramdump.address_of('rt_sched_class')
        sc_idle = self.ramdump.address_of('idle_sched_class')
        sc_fair = self.ramdump.address_of('fair_sched_class')

        sched_class_offset = address + self.ramdump.field_offset('struct task_struct', 'sched_class');
        sched_class_pointer = self.ramdump.read_word(sched_class_offset, True)

        if not ((sched_class_pointer == sc_stop) or (sched_class_pointer == sc_rt) or (sched_class_pointer == sc_idle) or (sched_class_pointer == sc_fair)):
            self.output_file.write(hex(address) + " seems to be corrupted! sched_class doesn't match with the defined ones\n")
            return -1;

    def validate_task_struct(self, address):
        thread_info_address = self.ramdump.get_thread_info_addr(address)
        if self.ramdump.is_thread_info_in_task():
            #Task is no longer found in thread_info
            task_struct = address
        else:
            task_address = thread_info_address + self.ramdump.field_offset('struct thread_info', 'task');
            task_struct = self.ramdump.read_word(task_address, True)

        cpu_number = self.ramdump.get_task_cpu(task_struct, thread_info_address)

        if((address != task_struct) or (thread_info_address == 0x0)):
            self.output_file.write(hex(address) + " seems to be corrupted! Please check task_struct and thread_info to find corruptions\n")
            return -1

        if((cpu_number < 0) or (cpu_number >= self.ramdump.get_num_cpus())):
            self.output_file.write(hex(address) + " seems to be corrupted! CPU number " + str(int(cpu_number)) +  " seems to be corrupted\n")
            return -1

    def check_thread_group(self, address, comm_offset_index):
        thread_group_offset_index = self.ramdump.field_offset('struct task_struct', 'thread_group')

        thread_group_offset = address + thread_group_offset_index;
        thread_group_pointer = self.ramdump.read_word(thread_group_offset, True)
        thread_group_pointer = thread_group_pointer - thread_group_offset_index

        next = thread_group_pointer;
        seen_thread = []
        if(thread_group_pointer != address):
            self.output_file.write("-----------------------------------\n")
            self.output_file.write("Threads of 0x{0:x}\n".format(address))
            self.output_file.write("-----------------------------------\n")
            while 1:
                tasks_offset = next + thread_group_offset_index;
                task_pointer = self.ramdump.read_word(tasks_offset, True)
                if(task_pointer <= 0x0):
                    self.output_file.write("task_pointer " + hex(task_pointer) + " calculcated from thread_group is corrupted\n")
                    return -1
                task_struct = task_pointer - thread_group_offset_index;
                if(task_struct <= 0x0):
                    self.output_file.write("task_struct " + hex(task_pointer) + " calculcated from thread_group is corrupted\n")
                    return -1
                comm_offset = task_struct + comm_offset_index
                comm = self.ramdump.read_cstring(comm_offset, 16, True)
                if (self.validate_task_struct(task_struct) == -1):
                    return -1
                if (self.validate_sched_class(task_struct) == -1):
                    return -1
                if task_struct in seen_thread:
                    self.output_file.write("!!!! Cycle in thread group! The list is corrupt!\n")
                    break
                self.output_file.write("Next = {0} ({1})\n".format(hex(task_struct).rstrip("L"), comm))
                seen_thread.append(task_struct)
                next = task_struct;
                if (next == thread_group_pointer):
                    break

            self.output_file.write("\n")

    def corruptionchecker(self):
        self.output_file.write("----------------------------------------------------------------------------------------\n")
        self.output_file.write("Checking for task list corruption.\n")
        self.output_file.write("----------------------------------------------------------------------------------------\n")

        init_task = self.ramdump.address_of('init_task')
        self.output_file.write("Init Task Address = {0}\n".format(hex(init_task)))
        tasks_offset = self.ramdump.field_offset('struct task_struct', 'tasks')
        self.output_file.write("Task Offset {0}\n".format(hex(tasks_offset).rstrip("L")))
        comm_offset = self.ramdump.field_offset('struct task_struct', 'comm')
        self.output_file.write("Comm Offset {0}\n\n".format(hex(comm_offset).rstrip("L")))
        seen_task = []
        next = init_task;
        found_corruption = 0

        while 1:
            tasks_pointer = self.ramdump.read_word(next + tasks_offset, True)
            if(tasks_pointer == 0x0):
                found_corruption = 1
                break

            task_struct = tasks_pointer - tasks_offset
            comm = self.ramdump.read_cstring(task_struct + comm_offset, 16, True)
            if (self.validate_task_struct(task_struct) == -1):
                found_corruption = 1
                break
            if (self.validate_sched_class(task_struct) == -1):
                found_corruption = 1
                break
            if (self.check_thread_group(task_struct, comm_offset) == -1):
                found_corruption = 1
                break
            if task_struct in seen_task:
                self.output_file.write("!!!! Cycle in task group! The list is corrupt!\n")
                break
            self.output_file.write("Next = {0} ({1})\n".format(hex(task_struct).rstrip("L"), comm))
            seen_task.append(task_struct)
            next = task_struct;
            if (next == init_task):
                break

        if(found_corruption):
            self.output_file.write("----------------------------------------\n")
            self.output_file.write("RESULT: Corruption found in the task list\n")
            self.output_file.write("----------------------------------------\n")
        else:
            self.output_file.write("----------------------------------------\n")
            self.output_file.write("RESULT: No issues found in the task list\n")
            self.output_file.write("----------------------------------------\n")

    def parse(self):
        self.output_file = self.ramdump.open_file('DDRCacheCompare.txt')

        self.compare_magic()
        self.corruptionchecker()

        self.output_file.close()
        print_out_str("--- Wrote the output to DDRCacheCompare.txt")
