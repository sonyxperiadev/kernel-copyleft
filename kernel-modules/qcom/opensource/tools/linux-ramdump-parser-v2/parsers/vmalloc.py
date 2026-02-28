# Copyright (c) 2012-2015,2017 The Linux Foundation. All rights reserved.
# Copyright (c) 2023-2025 Qualcomm Innovation Center, Inc. All rights reserved.
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
from print_out import print_out_str
from parser_util import register_parser, RamParser
import linux_list as llist

class Vmalloc:
    def __init__(self, ramdump):
        self.ramdump = ramdump
        self.vmalloc_out = None
        self.vmalloc_total_pages = 0
        self.VM_FLAGS = {
            "ioremap"     :0x00000001, #VM_IOREMAP
            "vmalloc"     :0x00000002, #VM_ALLOC
            "vmap"        :0x00000004, #VM_MAP
            "user"        :0x00000008, #VM_USERMAP
            "dma-coherent":0x00000010  #VM_DMA_COHERENT
        }
        self.vm_offset = self.ramdump.field_offset('struct vmap_area', 'vm')

    def print_header(self):
        print("VM_STRUCT                                  ADDRESS_RANGE                        " \
                "SIZE              PHYS_ADDR    			Flag          PAGES" \
                "							CALLER  ", file = self.vmalloc_out)
        return

    def print_vm(self, vm):
        if not vm:
           return

        addr = self.ramdump.read_structure_field(vm, 'struct vm_struct', 'addr')
        if addr is None:
           return

        caller = self.ramdump.read_structure_field(vm, 'struct vm_struct', 'caller')
        nr_pages = self.ramdump.read_structure_field(vm, 'struct vm_struct', 'nr_pages')
        phys_addr = self.ramdump.read_structure_field(vm, 'struct vm_struct', 'phys_addr')
        flags = self.ramdump.read_structure_field(vm, 'struct vm_struct', 'flags')
        size = self.ramdump.read_structure_field(vm, 'struct vm_struct', 'size')

        flags_str = ""
        for flag_name, flag_val in self.VM_FLAGS.items():
            if (flags & flag_val) != 0:
                flags_str += " {}".format(flag_name)

        line = ''
        func = ''
        if (caller != 0):
            line = self.ramdump.gdbmi.get_func_info(caller)
            if line == None:
                line = 'n/a'
            l = self.ramdump.unwind_lookup(caller)
            if l is not None:
                func, offset = l
            else:
                func = 'Unknown function'

        print("v.v (struct vm_struct)0x%x 0x%x-0x%x 0x%-16x  0x%-16x %-15s %-8d  %32s     %-s"
              %(vm, addr, addr + size, size , phys_addr, flags_str, nr_pages, func, line),\
                 file = self.vmalloc_out)
        return

    def traverse_vmap_area(self, vmhead_list, vmap_func=None):
        if not len(vmhead_list):
            return

        next_offset = self.ramdump.field_offset('struct vmap_area', 'list')
        for vmlist in vmhead_list:
            if not vmlist:
                continue
            list_walker = llist.ListWalker(self.ramdump, vmlist, next_offset)
            list_walker.walk(vmap_func)
        return

    def vmap_nodes(self, vmhead_list):
        vmap_nodes_addr = self.ramdump.read_pointer('vmap_nodes')
        if vmap_nodes_addr:
            nr_vmap_nodes = self.ramdump.read_int('nr_vmap_nodes')
            for vn_idx in range(0, nr_vmap_nodes):
                __vmap_nodes_addr = vmap_nodes_addr + vn_idx * self.ramdump.sizeof('struct vmap_node')
                vn = self.ramdump.struct_field_addr(__vmap_nodes_addr, 'struct vmap_node', 'busy.head')
                vmhead_list.append(vn)
        return

    def vmap_area_list(self, vmhead_list):
        vmap_area_list_addr = self.ramdump.address_of('vmap_area_list')
        if vmap_area_list_addr:
            vn = vmap_area_list_addr
            vmhead_list.append(vn)
        return

    def get_vmnodes(self, vmhead_list):
        vmap_types = {"vmap_nodes":self.vmap_nodes, "vmap_area_list":self.vmap_area_list}
        for vtype, vfunc in vmap_types.items():
            vfunc(vmhead_list)
            if len(vmhead_list):
                break
        return

    def vmap_list_func(self, va):
        vm = self.ramdump.read_word(va + self.vm_offset)
        self.print_vm(vm)
        return

    def parse_vmnodes(self, vmhead_list):
        self.print_header()

        if len(vmhead_list) == 0:
            print_out_str('vmap node is not found!')
            return

        self.traverse_vmap_area(vmhead_list, vmap_func=self.vmap_list_func)
        return

    def vmap_page_func(self, va):
        vm = self.ramdump.read_word(va + self.vm_offset)
        if not vm:
            return

        nr_pages = self.ramdump.read_structure_field(vm, 'struct vm_struct', 'nr_pages')
        flags = self.ramdump.read_structure_field(vm, 'struct vm_struct', 'flags')
        if flags is None:
            return
        if (flags & self.VM_FLAGS.get("vmalloc")) != 0:
            self.vmalloc_total_pages += nr_pages
        return

    def vmnode_pages(self, vmhead_list):
        self.traverse_vmap_area(vmhead_list, vmap_func=self.vmap_page_func)
        return

    def print_vmalloc_info_v2(self):
        vmhead_list = []

        self.get_vmnodes(vmhead_list)
        self.parse_vmnodes(vmhead_list)
        return

    def print_vmalloc_info_v1(self):
        self.print_header()

        vmlist = self.ramdump.read_word('vmlist')
        next_offset = self.ramdump.field_offset('struct vm_struct', 'next')

        while (vmlist is not None) and (vmlist != 0):
            self.print_vm(vmlist)
            vmlist = self.ramdump.read_word(vmlist + next_offset)
        return

    def get_vmalloc_pages(self):
        vmalloc_pages_ptr = self.ramdump.address_of('nr_vmalloc_pages')
        if vmalloc_pages_ptr is not None:
            self.vmalloc_total_pages = self.ramdump.read_word(vmalloc_pages_ptr)
        else:
            vmhead_list = []
            self.get_vmnodes(vmhead_list)
            self.vmnode_pages(vmhead_list)
        return self.vmalloc_total_pages

    def print_vmalloc_info(self):
        vmalloc_out = self.ramdump.open_file('vmalloc.txt')
        self.vmalloc_out = vmalloc_out

        major, minor, patch = self.ramdump.kernel_version
        if (major, minor) >= (3, 10):
            self.print_vmalloc_info_v2()
        else:
            self.print_vmalloc_info_v1()

        print_out_str('---wrote vmalloc to vmalloc.txt')
        vmalloc_out.close()
        return

@register_parser('--print-vmalloc', 'print vmalloc information')
class VmallocInfo(RamParser):
    def __init__(self, *args):
        super(VmallocInfo, self).__init__(*args)
        self.vmalloc = Vmalloc(self.ramdump)

    def parse(self):
        self.vmalloc.print_vmalloc_info()
