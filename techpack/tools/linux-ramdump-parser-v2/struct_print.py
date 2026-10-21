# Copyright (c) 2021, The Linux Foundation. All rights reserved.
#
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
import ctypes

from print_out import print_out_str
from linux_list import ListWalker
from ramdump import Struct

class struct_print_field_class(object):
    def __init__(self, name, data_type, list_struct_name, list_field_name):
        self.name = name
        self.data_type = data_type
        self.list_struct_name = list_struct_name
        self.list_field_name = list_field_name
        self.val = None

class struct_print_class(object):
    """
    Helper class to make it easy to parse Kernel C structures. It has support to
    parse kernel linked lists and print them.

    Given the below C struct which has an linked-list of the second structure:
    -------------------------------------------------------------------------

            struct file_system_type {
                    const char *name;
                    int fs_flags;
                    struct module *owner;
                    struct hlist_head fs_supers; ------------------
                    struct file_system_type *next;                |
            };                                                    |
                                                                  |
                                                                  |
                                                                  |
                                                                  |
    ------> struct super_block { <---------------------------------
    |               dev_t s_dev;
    |               unsigned long s_flags;
    |               struct hlist_node s_instances; -----------------
    |       };                                                     |
    |                                                              |
    ----------------------------------------------------------------


        You can parse the above C structures as below:
        ---------------------------------------------

        from struct_print import struct_print_class

        out_file_fd = ramdump.open_file('output.txt')
        var = struct_print_class(ramdump, 'file_system_type', vaddr, out_file_fd)
        var.append('name', 'char *')
        var.append('fs_flags', 'u32')
        var.append('fs_supers', 'hlist_head', list_struct_name='super_block', list_field_name='s_instances')
        var.append('next', 'ptr')
        var.process()
        var.print_struct() # this will print the formatted struct in supplied output file as shown below

        sample output:
        -------------
                struct file_system_type : 0xffffffe3d604ff70 {
                        name         : 0xffffffe3d56ca51c : [ext4]
                        fs_flags     : 0x1 : 1
                        fs_supers    : 0xffffff81bc338938 : struct hlist_head {
                        struct super_block : 0xffffff81bc338800
                                struct super_block : 0xffffff81bb898800
                                struct super_block : 0xffffff81bc0c3800
                                struct super_block : 0xffffff81c43cb800
                                struct super_block : 0xffffff81c4385000
                                struct super_block : 0xffffff81c47b8800
                                struct super_block : 0xffffff81c47be800
                                struct super_block : 0xffffff81c432b800
                                struct super_block : 0xffffff81c46b8800
                        }
                        next         : 0xffffffe3d5f33bb8
                }

    """

    def __init__(self, ramdump, struct_name, struct_vaddr, out_file):
        self.ramdump = ramdump
        self.struct_name = struct_name
        self.struct_vaddr = struct_vaddr
        self.out_file = out_file
        self.key_list = []
        self.fields_dict = dict()
        self.struct_inst = None
        self.cur_list_struct_name = None

    def append(self, name, data_type, list_struct_name=None, list_field_name=None):
        node = struct_print_field_class(name, data_type, list_struct_name, list_field_name)
        self.key_list.append(node)
        if data_type == 'u8':
            func_name = Struct.get_u8
        elif data_type == 'u16':
            func_name = Struct.get_u16
        elif data_type == 'u32':
            func_name = Struct.get_u32
        elif data_type == 'u64':
            func_name = Struct.get_u64
        elif data_type == 'ptr':
            func_name = Struct.get_pointer
        elif data_type == 'uuid':
            func_name = Struct.get_pointer
        elif data_type == 'char *':
            func_name = Struct.get_pointer
        elif data_type.startswith('char['):
            func_name = Struct.get_cstring
        elif (node.data_type == 'list_head') or (data_type == 'hlist_head'):
            func_name = Struct.get_pointer
        elif data_type == 'rw_semaphore':
            func_name = Struct.get_address
        elif data_type == 'mutex':
            func_name = Struct.get_address
        else:
            s1 = 'error: unknown data type {0:s} for field {1:s}\n'.format(data_type, name)
            print(s1)
            assert(0)
        self.fields_dict[name] = func_name
        return node

    def process(self):
        struct_str = 'struct {0:s}'.format(self.struct_name)
        self.struct_inst = Struct(self.ramdump, self.struct_vaddr,
            struct_name=struct_str, fields=self.fields_dict)
        for node in self.key_list:
            try:
                node.val = self.struct_inst.__getattr__(node.name)
            except:
                print('key lookup exception [{0:s}] : [{1:s}]\n'.format(node.name, node.data_type))
                node.name = node.name + " : not found"
                node.val = 0x0
                pass

    def get_val(self, name):
        val = self.struct_inst.__getattr__(name)
        return val

    def to_hex(self, val):
        s1 = hex(val).rstrip('L')
        return s1

    def hlist_head_walk_func(self, vaddr):
        self.out_file.write('\t\t {0:s} : {1:s}\n'.format(self.cur_list_struct_name, self.to_hex(vaddr)))

    def print_struct(self):
        s1 = 'struct {0:s} : {1:s} {{\n'.format(self.struct_name, self.to_hex(self.struct_vaddr))
        self.out_file.write(s1)
        for node in self.key_list:
            if (node.data_type == 'char *'):
                val_str = self.ramdump.read_cstring(node.val)
                s1 = '\t{0:12s} : {1:s} : [{2:s}]\n'.format(node.name, self.to_hex(node.val), val_str)
                self.out_file.write(s1)
            elif (node.data_type.startswith('char[')):
                s1 = '\t{0:12s} : {1:s}\n'.format(node.name, node.val)
                self.out_file.write(s1)
            elif (node.data_type == 'u8') or (node.data_type == 'u16') or (node.data_type == 'u32') or (node.data_type == 'u64'):
                s1 = '\t{0:12s} : {1:s} : {2:d}\n'.format(node.name, self.to_hex(node.val), node.val)
                self.out_file.write(s1)
            elif (node.data_type == 'ptr'):
                s1 = '\t{0:12s} : {1:s}\n'.format(node.name, self.to_hex(node.val))
                self.out_file.write(s1)
            elif (node.data_type == 'uuid'):
                s1 = '\t{0:12s} : {1:s}\n'.format(node.name, self.to_hex(node.val))
                self.out_file.write(s1)
            elif (node.data_type == 'list_head') or (node.data_type == 'hlist_head'):
                s1 = '\t{0:12s} : {1:s} : struct {2:s} {{\n'.format(node.name, self.to_hex(node.val), node.data_type)
                self.out_file.write(s1)
                self.cur_list_struct_name = 'struct {0:s}'.format(node.list_struct_name)
                field_name = node.list_field_name
                offset = self.ramdump.field_offset(self.cur_list_struct_name, field_name)
                s1 = 'name=[{0:s}] offset=[{1:d}]\n'.format(field_name, offset)
                if node.val == 0xFFFFFFFF or node.val == 0xdead4ead00000000:
                    pass
                else:
                    list_walker = ListWalker(self.ramdump, node.val, offset)
                    list_walker.walk(node.val, self.hlist_head_walk_func)
                s1 = '\t}\n'
                self.out_file.write(s1)
            elif (node.data_type == 'rw_semaphore'):
                rw_semaphore_addr = node.val
                count = self.ramdump.read_structure_field(rw_semaphore_addr, 'struct rw_semaphore', 'count')
                owner = self.ramdump.read_structure_field(rw_semaphore_addr, 'struct rw_semaphore', 'owner')
                # Remove flag bits
                owner = owner & ~0x7
                s1 = '\t{0:12s} : {1:s} : count = {2:s} : owner = {3:s}\n'.format(node.name, self.to_hex(node.val), self.to_hex(count), self.to_hex(owner))
                self.out_file.write(s1)
                if owner:
                    comm_addr = owner + self.ramdump.field_offset('struct task_struct', 'comm')
                    comm = self.ramdump.read_cstring(comm_addr, max_length=16)
                    pid_addr = owner + self.ramdump.field_offset('struct task_struct', 'pid')
                    pid = self.ramdump.read_int(pid_addr)
                    self.out_file.write('\t\towner comm = {} pid = {}\n'.format(comm, pid))
            elif (node.data_type == 'mutex'):
                mutex_addr = node.val
                owner = self.ramdump.read_structure_field(mutex_addr, 'struct mutex', 'owner')
                # Remove flag bits
                owner = owner & ~0x7
                s1 = '\t{0:12s} : {1:s} : owner = {2:s}\n'.format(node.name, self.to_hex(node.val), self.to_hex(owner))
                self.out_file.write(s1)
                if owner:
                    comm_addr = owner + self.ramdump.field_offset('struct task_struct', 'comm')
                    comm = self.ramdump.read_cstring(comm_addr, max_length=16)
                    pid_addr = owner + self.ramdump.field_offset('struct task_struct', 'pid')
                    pid = self.ramdump.read_int(pid_addr)
                    self.out_file.write('\t\towner comm = {} pid = {}\n'.format(comm, pid))
        self.out_file.write('}\n')
