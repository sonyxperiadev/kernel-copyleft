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
import pdb

from parser_util import register_parser, RamParser
from print_out import print_out_str
from linux_list import ListWalker
from ramdump import Struct
from struct_print import struct_print_class


@register_parser('--fs-parser', 'FS report', optional=True)
class fs_parser_class(RamParser):
    def __init__(self, *args):
        super(fs_parser_class, self).__init__(*args)
        self.sb_vaddr_list = []
        self.inode_vaddr_list = []

    def output(self, str_val):
        self.output_file.write(str_val)

    def parse(self):
        self.output_file = self.ramdump.open_file('fs-report.txt')
        print_out_str("fs-parser:start")
        s1 = self.ramdump.get_kernel_version()
        s2 = 'Kernel version : [{0:d}.{0:d}.{0:d}]\n'.format(s1[0], s1[1], s1[2])
        self.output(s2)
        vaddr = self.ramdump.read_word('file_systems')
        self.file_systems_struct(vaddr)
        self.iterate_sb()
        #self.iterate_inode()
        print_out_str("fs-parser:end")
        self.output_file.close()
        return

    def file_systems_struct(self, fs_vaddr):
        if fs_vaddr == 0:
            return
        shc = struct_print_class(self.ramdump, 'file_system_type', fs_vaddr, self.output_file)
        shc.append('name', 'char *')
        shc.append('fs_flags', 'u32')
        node = shc.append('fs_supers', 'hlist_head', list_struct_name='super_block', list_field_name='s_instances')
        shc.append('next', 'ptr')
        shc.process()
        shc.print_struct()
        name_ptr = shc.get_val('name')
        name = self.ramdump.read_cstring(name_ptr)
        if name not in ['proc', 'sysfs', 'tmpfs', 'debugfs', 'tracefs']:
            fs_supers = shc.get_val('fs_supers')
            s_instance_offset = self.ramdump.field_offset('struct super_block', 's_instances')
            list_walker = ListWalker(self.ramdump, fs_supers, s_instance_offset)
            list_walker.walk(fs_supers, self.fs_sb_list_func)
        next_ptr = shc.get_val('next')
        self.file_systems_struct(next_ptr)

    def fs_sb_list_func(self, sb_vaddr):
        self.sb_vaddr_list.append(sb_vaddr)

    def iterate_sb(self):
        for sb_vaddr in self.sb_vaddr_list:
            self.print_sb(sb_vaddr)

    def print_sb(self, sb_vaddr):
        shc = struct_print_class(self.ramdump, 'super_block', sb_vaddr, self.output_file)
        shc.append('s_flags', 'u32')
        shc.append('s_iflags', 'u32')
        shc.append('s_magic', 'u32')
        shc.append('s_id', 'char[32]')
        shc.append('s_inodes', 'list_head', list_struct_name='inode', list_field_name='i_sb_list')
        shc.append('s_inodes_wb', 'list_head', list_struct_name='inode', list_field_name='i_wb_list')
        shc.process()
        shc.print_struct()

        s_inodes = shc.get_val('s_inodes')
        s_inodes_offset = self.ramdump.field_offset('struct inode', 'i_sb_list')
        list_walker = ListWalker(self.ramdump, s_inodes, s_inodes_offset)
        list_walker.walk(s_inodes, self.fs_inode_list_func)

        s_inodes_wb = shc.get_val('s_inodes_wb')
        s_inodes_wb_offset = self.ramdump.field_offset('struct inode', 'i_wb_list')
        list_walker = ListWalker(self.ramdump, s_inodes_wb, s_inodes_wb_offset)
        list_walker.walk(s_inodes_wb, self.fs_inode_list_func)

    def fs_inode_list_func(self, inode_vaddr):
        self.inode_vaddr_list.append(inode_vaddr)

    def iterate_inode(self):
        for inode_vaddr in self.inode_vaddr_list:
            self.print_inode(inode_vaddr)

    def print_inode(self, inode_vaddr):
        shc = struct_print_class(self.ramdump, 'inode', inode_vaddr, self.output_file)
        shc.append('i_opflags', 'u16')
        shc.append('i_flags', 'u32')
        shc.append('i_ino', 'u64')
        shc.append('i_size', 'u64')
        shc.append('i_bytes', 'u16')
        shc.append('i_state', 'u64')
        node = shc.append('i_lru', 'list_head', list_struct_name='inode', list_field_name='i_lru')
        shc.process()
        shc.print_struct()

if __name__ == '__main__':
    print('nothing to do yet\n')
