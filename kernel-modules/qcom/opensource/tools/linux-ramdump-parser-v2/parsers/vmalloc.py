# Copyright (c) 2012-2015,2017 The Linux Foundation. All rights reserved.
# Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
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

VM_IOREMAP = 0x00000001
VM_ALLOC = 0x00000002
VM_MAP = 0x00000004
VM_USERMAP = 0x00000008
VM_VPAGES = 0x00000010
VM_UNLIST = 0x00000020


@register_parser('--print-vmalloc', 'print vmalloc information')
class Vmalloc(RamParser):

    def print_vm(self, vm):
        if vm == 0 or vm is None:
           return
        addr = self.ramdump.read_structure_field(vm, 'struct vm_struct', 'addr')
        if addr is None:
           return
        caller = self.ramdump.read_structure_field(vm, 'struct vm_struct', 'caller')
        nr_pages = size = self.ramdump.read_structure_field(vm, 'struct vm_struct', 'nr_pages')
        phys_addr = self.ramdump.read_structure_field(vm, 'struct vm_struct', 'phys_addr')
        flags = self.ramdump.read_structure_field(vm, 'struct vm_struct', 'flags')
        size = self.ramdump.read_structure_field(vm, 'struct vm_struct', 'size')

        flags_str = ''
        if (flags & VM_IOREMAP) != 0:
            flags_str = ' ioremap'

        if (flags & VM_ALLOC) != 0:
            flags_str = ' vmalloc'

        if (flags & VM_MAP) != 0:
            flags_str = ' vmap'

        if (flags & VM_USERMAP) != 0:
            flags_str = ' user'

        if (flags & VM_VPAGES) != 0:
            flags_str = ' vpages'

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

        print("v.v (struct vm_struct)0x%x 0x%x-0x%x 0x%-16x  0x%-16x %-10s %-8d  %32s     %-s"
              %(vm, addr, addr + size, size , phys_addr, flags_str, nr_pages, func, line), file = self.vmalloc_out)

    def list_func(self, vmlist):
        vm_offset = self.ramdump.field_offset('struct vmap_area', 'vm')

        vm = self.ramdump.read_word(vmlist + vm_offset)
        self.print_vm(vm)


    def print_vmalloc_info_v2(self, out_path):
        vmalloc_out = self.ramdump.open_file('vmalloc.txt')

        next_offset = self.ramdump.field_offset('struct vmap_area', 'list')
        vmlist = self.ramdump.read_word('vmap_area_list')

        list_walker = llist.ListWalker(self.ramdump, vmlist, next_offset)
        self.vmalloc_out = vmalloc_out
        print("VM_STRUCT                                  ADDRESS_RANGE                        SIZE              PHYS_ADDR    			Flag     PAGES							CALLER  ", file =   self.vmalloc_out)
        list_walker.walk(vmlist, self.list_func)
        print_out_str('---wrote vmalloc to vmalloc.txt')
        vmalloc_out.close()


    def print_vmalloc_info(self, out_path):
        vmlist = self.ramdump.read_word('vmlist')
        next_offset = self.ramdump.field_offset('struct vm_struct', 'next')

        vmalloc_out = self.ramdump.open_file('vmalloc.txt')
        self.vmalloc_out = vmalloc_out

        while (vmlist is not None) and (vmlist != 0):
            self.print_vm(vmlist)

            vmlist = self.ramdump.read_word(vmlist + next_offset)

        print_out_str('---wrote vmalloc to vmalloc.txt')
        vmalloc_out.close()

    def parse(self):
        out_path = self.ramdump.outdir
        major, minor, patch = self.ramdump.kernel_version
        if (major, minor) >= (3, 10):
            self.print_vmalloc_info_v2(out_path)
        else:
            self.print_vmalloc_info(out_path)
