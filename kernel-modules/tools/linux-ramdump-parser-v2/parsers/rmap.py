#SPDX-License-Identifier: GPL-2.0-only
#Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.

from print_out import print_out_str
from parser_util import register_parser, RamParser, cleanupString
from mm import pfn_to_page, page_buddy, page_count, for_each_pfn
from mm import page_to_pfn
from parsers.uvtop import do_get_task_info
import sys
import rb_tree

def save_task_info(ramdump):
    task_dict = dict()
    for task in ramdump.for_each_process():
        pid, thread_task_name, mm = do_get_task_info(ramdump, task)
        task_dict[mm] = (pid, thread_task_name, task)
    return task_dict

@register_parser('--print-rmap', 'print page rmap', optional=True)
class Rmap(RamParser):

    def rmap_one(self, rb_node, extra):
        page = extra[0]
        fout =  extra[1]
        anon_vma_chain = self.ramdump.container_of(rb_node, 'struct anon_vma_chain', 'rb')
        if anon_vma_chain == None or anon_vma_chain == 0:
            return
        vma = self.ramdump.read_structure_field(anon_vma_chain, 'struct anon_vma_chain', 'vma')
        if vma == 0 or vma == None:
            return
        vm_mm = self.ramdump.read_structure_field(vma, 'struct  vm_area_struct', 'vm_mm')
        if vm_mm == 0 or vm_mm == None:
            return
        if self.task_dict.get(vm_mm) == None:
            return
        anon_name = self.ramdump.read_structure_field(vma, 'struct  vm_area_struct', 'anon_name')
        name_addr =  self.ramdump.struct_field_addr(anon_name, 'struct  anon_vma_name', 'name')
        name = cleanupString(self.ramdump.read_cstring(name_addr))
        task_info = self.task_dict[vm_mm]
        pid = task_info[0]
        thread_task_name = task_info[1]
        print("         v.v (struct vm_area_struct)0x%x  pid %-8d comm %-16s  %s" % (vma, pid, thread_task_name, name), file = fout)

    def rmap_walk_anon(self, page, fout):
        PAGE_MAPPING_ANON = 1
        if page == None or page == 0:
            reurn
        addr_space = self.ramdump.read_structure_field(page, 'struct page', 'mapping')
        if addr_space == None or addr_space == 0:
            return
        if addr_space & 0x1 == 0:
            return
        anon_vma = addr_space - PAGE_MAPPING_ANON
        print(" v.v (struct page)0x%x" % (page), file=fout)

        rb_root_offset = self.ramdump.field_offset('struct anon_vma', 'rb_root')
        rb_root = rb_root_offset + anon_vma
        rb_node = self.ramdump.read_pointer(rb_root)
        rb_walker = rb_tree.RbTreeWalker(self.ramdump)
        extra = (page, fout)
        rb_walker.walk(rb_node, self.rmap_one, extra)
        return

    def parse(self):
        page = None
        g_optimization = False
        for arg in sys.argv:
            if "page=" in arg:
                g_optimization = True
                k, page = arg.split('=')
                page = int(page, 16)
                break
        self.task_dict = save_task_info(self.ramdump)
        out_tracking = self.ramdump.open_file('rmap.txt')

        if g_optimization:
            self.rmap_walk_anon(page, out_tracking)
        else:
            ranges = for_each_pfn(self.ramdump)
            for pfn in ranges:
                page = pfn_to_page(self.ramdump, pfn)
                if (page_buddy(self.ramdump, page) or \
                    page_count(self.ramdump, page) == 0):
                    continue
                self.rmap_walk_anon(page, out_tracking)
        out_tracking.close()
        print_out_str('---wrote file tracking information to rmap.txt')
