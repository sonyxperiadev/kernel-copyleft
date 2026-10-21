# Copyright (c) 2012-2015,2017 The Linux Foundation. All rights reserved.
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

        next_offset = self.ramdump.field_offset('struct vm_struct', 'next')
        addr_offset = self.ramdump.field_offset('struct vm_struct', 'addr')
        size_offset = self.ramdump.field_offset('struct vm_struct', 'size')
        flags_offset = self.ramdump.field_offset('struct vm_struct', 'flags')
        pages_offset = self.ramdump.field_offset('struct vm_struct', 'pages')
        nr_pages_offset = self.ramdump.field_offset('struct vm_struct', 'nr_pages')
        phys_addr_offset = self.ramdump.field_offset(
            'struct vm_struct', 'phys_addr')
        caller_offset = self.ramdump.field_offset('struct vm_struct', 'caller')

        addr = self.ramdump.read_word(vm + addr_offset)
        caller = self.ramdump.read_word(vm + caller_offset)
        nr_pages = self.ramdump.read_word(vm + nr_pages_offset)
        phys_addr = self.ramdump.read_word(vm + phys_addr_offset)
        flags = self.ramdump.read_word(vm + flags_offset)
        size = self.ramdump.read_word(vm + size_offset)

        if addr is None:
           return

        vmalloc_str = 'v.v (struct vmap_area)0x{0:16x} '.format(vm)
        vmalloc_str = vmalloc_str + '{0:18x}-{1:x} {2:8x}'\
            .format(addr, addr + size, size)

        vmalloc_str = vmalloc_str + ' {0:10x} '.format(phys_addr)

        if (caller != 0):
            a = self.ramdump.unwind_lookup(caller)
            if a is not None:
                symname, offset = a
                sym = symname + '0x' + str(hex(offset))
                vmalloc_str = vmalloc_str + \
                    ' {0:46}'.format(sym)

        if (flags & VM_IOREMAP) != 0:
            vmalloc_str = vmalloc_str + ' ioremap'

        if (flags & VM_ALLOC) != 0:
            vmalloc_str = vmalloc_str + ' vmalloc'

        if (flags & VM_MAP) != 0:
            vmalloc_str = vmalloc_str + ' vmap'

        if (flags & VM_USERMAP) != 0:
            vmalloc_str = vmalloc_str + ' user'

        if (flags & VM_VPAGES) != 0:
            vmalloc_str = vmalloc_str + ' vpages'

        if (nr_pages != 0):
            vmalloc_str = vmalloc_str + ' pages={0}'.format(nr_pages)

        vmalloc_str = vmalloc_str + '\n'
        self.vmalloc_out.write(vmalloc_str)

    def list_func(self, vmlist):
        vm_offset = self.ramdump.field_offset('struct vmap_area', 'vm')

        vm = self.ramdump.read_word(vmlist + vm_offset)
        self.print_vm(vm)


    def print_vmalloc_info_v2(self, out_path):
        vmalloc_out = self.ramdump.open_file('vmalloc.txt')

        next_offset = self.ramdump.field_offset('struct vmap_area', 'list')
        vmlist = self.ramdump.read_word('vmap_area_list')
        orig_vmlist = vmlist

        list_walker = llist.ListWalker(self.ramdump, vmlist, next_offset)
        self.vmalloc_out = vmalloc_out
        vmalloc_str = 'Memory mapped region allocated by Vmalloc\n\n'
        vmalloc_str = vmalloc_str + '{0:42} {1:36} {2:6} {3:12} {4:46} {5:8}'\
            .format('VM_STRUCT','ADDRESS_RANGE','SIZE','PHYS_ADDR','CALLER',
                    'Flag')
        vmalloc_str = vmalloc_str + '\n'
        self.vmalloc_out.write(vmalloc_str)
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
