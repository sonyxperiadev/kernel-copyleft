# Copyright (c) 2021 Qualcomm Innovation Center, Inc. All rights reserved.

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from parser_util import register_parser, RamParser, cleanupString
from mmu import Armv8MMU
from print_out import print_out_str
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
        self.proplist = ""
        self.data = ""

    def read_binary(self, mmu, ramdump, addr, length):
        """Reads binary data of specified length from addr_or_name."""
        min = 0
        msg = b''
        while length > 0:
            addr = addr + min
            # msg located in the same page
            if length < (0x1000 - addr % 0x1000):
                min = length
            # msg separated in two pages
            else:
                min = 0x1000 - addr % 0x1000
            length = length - min
            addr_phys = mmu.virt_to_phys(addr)
            msg_binary = ramdump.read_physical(addr_phys, min)
            if msg_binary is None or msg_binary == '':
                return msg
            msg = msg + msg_binary
        return msg

    def find_mmap_pgd(self):
        offset_comm = self.ramdump.field_offset('struct task_struct', 'comm')
        mm_offset = self.ramdump.field_offset('struct task_struct', 'mm')
        pgd = None
        mmap = None
        for task in self.ramdump.for_each_process():
            task_name = task + offset_comm
            task_name = cleanupString(self.ramdump.read_cstring(task_name, 16))
            if task_name == 'init':
                mm_addr = self.ramdump.read_word(task + mm_offset)
                mmap = self.ramdump.read_structure_field(mm_addr, 'struct mm_struct',
                                                   'mmap')
                pgd = self.ramdump.read_structure_field(mm_addr, 'struct mm_struct',
                                                   'pgd')
                break
        return mmap, pgd

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
                self.proplist = self.proplist+ (name+"="+value)+"\n"
        if children != 0:
            err = self.foreach_property(children)
            if not err:
                return False
        if right != 0:
            err = self.foreach_property(right)
            if not err:
                return False
        return True

    def parse_property(self, mmu, mmap):
        index = 0
        with self.ramdump.open_file("sys_prop.txt") as out_file:
            initmap = mmap
            while initmap != 0:
                tmpstartVm = self.ramdump.read_structure_field(
                                initmap, 'struct vm_area_struct', 'vm_start')
                tmpsEndVm = self.ramdump.read_structure_field(
                                initmap, 'struct vm_area_struct', 'vm_end')
                file = self.ramdump.read_structure_field(
                                initmap, 'struct vm_area_struct', 'vm_file')
                if file != 0:
                    dentry = self.ramdump.read_word(file + self.f_path_offset +
                                            self.dentry_offset)
                    file_name = cleanupString(self.ramdump.read_cstring(
                                            dentry + self.d_iname_offset, 32))
                    if "u:object_r:" in file_name:
                        self.data = self.read_binary(mmu, self.ramdump, tmpstartVm, tmpsEndVm - tmpstartVm)
                        if index == 0 and self.OFFSET_MAGIC+8 <= len(self.data):
                            btEntry = struct.unpack('<II', self.data[self.OFFSET_MAGIC:self.OFFSET_MAGIC+8])
                            magic = btEntry[0]
                            version = btEntry[1]
                            out_file.write("System Properties Magic:" + str(hex(magic))
                                    + " Version:" + str(hex(version)) + "\n---------------------\n")
                            index = index + 1
                        # root node of prop_bt
                        if self.SIZEOF_PROP_AREA + self.SIZEOF_PROP_BT <= len(self.data):
                            btEntry = struct.unpack('<IIIII', self.data[
                                        self.SIZEOF_PROP_AREA : self.SIZEOF_PROP_AREA + self.SIZEOF_PROP_BT])
                            root_child = btEntry[4]
                            if root_child != 0:
                                self.foreach_property(root_child)
                                out_file.write(self.proplist)
                                self.proplist = ""
                initmap = self.ramdump.read_structure_field(
                               initmap, 'struct vm_area_struct', 'vm_next') #next loop

    def find_property_from_file(self, mmu, mmap, prop_name, prop_file):
        index = 0
        initmap = mmap
        while initmap != 0:
            tmpstartVm = self.ramdump.read_structure_field(
                            initmap, 'struct vm_area_struct', 'vm_start')
            tmpsEndVm = self.ramdump.read_structure_field(
                            initmap, 'struct vm_area_struct', 'vm_end')
            file = self.ramdump.read_structure_field(
                            initmap, 'struct vm_area_struct', 'vm_file')
            if file != 0:
                dentry = self.ramdump.read_word(file + self.f_path_offset +
                                        self.dentry_offset)
                file_name = cleanupString(self.ramdump.read_cstring(
                                        dentry + self.d_iname_offset, 32))
                if file_name == prop_file:
                    self.data = self.read_binary(mmu, self.ramdump, tmpstartVm, tmpsEndVm - tmpstartVm)
                    if self.data and len(self.data) > 0:
                        value = self.find_property(prop_name)
                        return value
            initmap = self.ramdump.read_structure_field(
                            initmap, 'struct vm_area_struct', 'vm_next') #next loop
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
        mmap, pgd = self.find_mmap_pgd()
        if mmap is None:
            return
        pgdp = self.ramdump.virt_to_phys(pgd)
        mmu = Armv8MMU(self.ramdump, pgdp)
        self.parse_property(mmu, mmap)

    # get property from init mmap
    def property_get(self, prop_name, prop_file):
        mmap, pgd = self.find_mmap_pgd()
        if mmap is None:
            return None
        pgdp = self.ramdump.virt_to_phys(pgd)
        mmu = Armv8MMU(self.ramdump, pgdp)
        return self.find_property_from_file(mmu, mmap, prop_name,prop_file)
