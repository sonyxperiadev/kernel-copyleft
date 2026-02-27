# Copyright (c) 2021-2023 Qualcomm Innovation Center, Inc. All rights reserved.

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from parser_util import register_parser, RamParser
from print_out import print_out_str
from utasklib import UTaskLib
from utasklib import ProcessNotFoundExcetion
import struct

@register_parser('--properties', 'Extract properties from ramdump ')
class Properties(RamParser):
    def __init__(self, *args):
        super(Properties, self).__init__(*args)
        self.f_path_offset = self.ramdump.field_offset('struct file', 'f_path')
        self.dentry_offset = self.ramdump.field_offset('struct path', 'dentry')
        self.d_iname_offset = self.ramdump.field_offset('struct dentry', 'd_iname')
        self.SIZEOF_PROP_BT=0x14
        self.SIZEOF_PROP_INFO=0x60
        self.SIZEOF_PROP_AREA=0x80
        self.OFFSET_MAGIC = 0x8
        self.OFFSET_DATA = self.SIZEOF_PROP_AREA
        self.PROP_NAME_MAX=100
        self.PROP_VALUE_MAX=92
        self.proplist = []
        self.data = ""
        self.header = ""

    def foreach_property(self, prop_bt):
        if prop_bt == 0 or self.OFFSET_DATA + prop_bt+ self.SIZEOF_PROP_BT > len(self.data):
            return False
        btEntry = struct.unpack('<IIIII', self.data[
            self.OFFSET_DATA+ prop_bt : self.OFFSET_DATA + prop_bt+ self.SIZEOF_PROP_BT])
        name_length = btEntry[0]
        prop = btEntry[1]
        left = btEntry[2]
        right = btEntry[3]
        children = btEntry[4]
        name = self.data[
                self.OFFSET_DATA + prop_bt+ self.SIZEOF_PROP_BT :
                self.OFFSET_DATA + prop_bt+ self.SIZEOF_PROP_BT + name_length]
        if left != 0:
            err = self.foreach_property(left)
            if not err:
                return False
        if prop != 0:
            if self.OFFSET_DATA + prop+ self.PROP_VALUE_MAX <= len(self.data):
                value = self.data[
                    self.OFFSET_DATA + prop + 0x4 :
                    self.OFFSET_DATA + prop + self.PROP_VALUE_MAX
                    ].decode('ascii', 'ignore').strip().split('\0')[0]
                name = self.data[
                    self.OFFSET_DATA + prop + self.SIZEOF_PROP_INFO :
                    self.OFFSET_DATA + prop + self.SIZEOF_PROP_INFO + self.PROP_NAME_MAX
                    ].decode('ascii', 'ignore').split('\0')[0]
                self.proplist.append([name, value])
        if children != 0:
            err = self.foreach_property(children)
            if not err:
                return False
        if right != 0:
            err = self.foreach_property(right)
            if not err:
                return False
        return True

    def parse_property(self, taskinfo):
        index = 0
        for vma in taskinfo.vmalist:
            if "u:object_r:" in vma.file_name:
                self.data = UTaskLib.read_binary(
                    self.ramdump, taskinfo.mmu, vma.vm_start, vma.vm_end - vma.vm_start)
                if index == 0 and self.OFFSET_MAGIC+8 <= len(self.data):
                    btEntry = struct.unpack('<II', self.data[self.OFFSET_MAGIC:self.OFFSET_MAGIC+8])
                    magic = btEntry[0]
                    version = btEntry[1]
                    self.header = "System Properties Magic:" + str(hex(magic)) \
                            + " Version:" + str(hex(version)) + "\n---------------------\n"
                    index = index + 1
                # root node of prop_bt
                if self.SIZEOF_PROP_AREA + self.SIZEOF_PROP_BT <= len(self.data):
                    btEntry = struct.unpack('<IIIII', self.data[
                                self.SIZEOF_PROP_AREA : self.SIZEOF_PROP_AREA + self.SIZEOF_PROP_BT])
                    root_child = btEntry[4]
                    if root_child != 0:
                        self.foreach_property(root_child)

    def find_property_from_file(self, taskinfo, prop_name, prop_file):
        for vma in taskinfo.vmalist:
            if vma.file_name == prop_file:
                self.data = UTaskLib.read_binary(
                    self.ramdump, taskinfo.mmu, vma.vm_start, vma.vm_end - vma.vm_start)
                if self.data and len(self.data) > 0:
                    value = self.find_property(prop_name)
                    return value
        return -1

    def find_property(self, prop_name):
        remaining_name = prop_name
        current = 0
        count = 0
        while True:
            idx = remaining_name.find('.')
            if idx == -1:
                cname = remaining_name
            else:
                cname = remaining_name[:idx]
            if self.OFFSET_DATA + current + self.SIZEOF_PROP_BT > len(self.data):
                return None
            btEntry = struct.unpack('<I', self.data[
                self.OFFSET_DATA + current + self.SIZEOF_PROP_BT - 4:
                self.OFFSET_DATA + current + self.SIZEOF_PROP_BT])
            current = btEntry[0]
            if current == 0:
                return None
            name = self.data[
                self.OFFSET_DATA+current+self.SIZEOF_PROP_BT :
                self.OFFSET_DATA + current+ self.SIZEOF_PROP_BT+self.PROP_NAME_MAX
                ].decode('ascii', 'ignore').split('\0')[0]
            current = self.find_prop_bt(current, cname)
            if current == 0:
                return None
            if idx == -1:
                break
            remaining_name = remaining_name[idx+1:]
            count = count + 1

        btEntry = struct.unpack('<IIIII', self.data[
            self.OFFSET_DATA + current :
            self.OFFSET_DATA + current + self.SIZEOF_PROP_BT])
        prop = btEntry[1]
        left = btEntry[2]
        right = btEntry[3]
        children = btEntry[4]

        if prop != 0:
            if self.OFFSET_DATA + prop+ self.PROP_VALUE_MAX < len(self.data):
                value = self.data[
                    self.OFFSET_DATA+prop+0x4 :
                    self.OFFSET_DATA + prop+ self.PROP_VALUE_MAX
                    ].decode('ascii', 'ignore').strip().split('\0')[0]
                name = self.data[
                    self.OFFSET_DATA+prop+self.SIZEOF_PROP_INFO :
                    self.OFFSET_DATA + prop+ self.SIZEOF_PROP_INFO+self.PROP_NAME_MAX+1
                    ].decode('ascii', 'ignore').split('\0')[0]
                return value

    def find_prop_bt(self, prop_bt, prop_name):
        current = prop_bt
        while True:
            if current == 0:
                break

            if self.OFFSET_DATA + current + self.SIZEOF_PROP_BT > len(self.data):
                return 0
            btEntry = struct.unpack('<IIIII', self.data[
                self.OFFSET_DATA + current :
                self.OFFSET_DATA + current + self.SIZEOF_PROP_BT])
            prop = btEntry[1]
            left = btEntry[2]
            right = btEntry[3]
            children = btEntry[4]
            name = self.data[
                self.OFFSET_DATA+current+self.SIZEOF_PROP_BT :
                self.OFFSET_DATA + current+ self.SIZEOF_PROP_BT+self.PROP_NAME_MAX
                ].decode('ascii', 'ignore').split('\0')[0]
            if name == prop_name:
                return current
            if self.cmp_prop_name(prop_name, name) < 0:
                if left == 0:
                    return 0
                current = left
            else:
                if right == 0:
                    return 0
                current = right

    def cmp_prop_name(self, name1,name2):
        if len(name1) < len(name2):
            return -1
        elif len(name1) > len(name2):
            return 1
        elif name1 > name2:
            return 1
        else:
            return -1

    def parse(self):
        try:
            taskinfo = UTaskLib(self.ramdump).get_utask_info("init", logging=True)
        except ProcessNotFoundExcetion:
            print_out_str("init process was not started")
            return False
        with self.ramdump.open_file("Properties.txt") as out_file:
            self.parse_property(taskinfo)

            if self.header:
                out_file.write(self.header)

            for name, value in self.proplist:
                out_file.write("{}={}\n".format(name, value))
        return len(self.proplist) > 0

    # get property from init mmap
    def property_get(self, prop_name, prop_file):
        try:
            taskinfo = UTaskLib(self.ramdump).get_utask_info("init")
        except ProcessNotFoundExcetion:
            print_out_str("init process was not started")
            return

        return self.find_property_from_file(taskinfo, prop_name, prop_file)

