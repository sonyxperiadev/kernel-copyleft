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

import os
import rb_tree
import sys
import string
import maple_tree
import re
import time

from mmu import Armv8MMU
from mm import pfn_to_page, page_buddy, page_count, for_each_pfn, page_to_pfn, phys_to_virt
from print_out import print_out_str
from parser_util import register_parser, RamParser, cleanupString

PAGE_SHIFT = 12
PAGE_SIZE = 4096
PGDIR_SHIFT_L3_4K = 30
PTRS_PER_PGD_L3_4K = 512

PMD_SHIFT_L3_4K = (21)
PMD_SIZE_L3_4K = (1 << PMD_SHIFT_L3_4K)
PHYS_MASK_SHIFT = (48)
PHYS_MASK = ((1 << PHYS_MASK_SHIFT) - 1)
PTRS_PER_PMD_L3_4K = (512)

PTRS_PER_PTE_L3_4K = 512

pagemask = 0xfffffffffffff000
pageoffset = 0xfff



#help -m userspace_top: 0000008000000000
def arm64_is_uvaddr(addr):
    return (addr < 0x8000000000)

def PAGEOFFSET(x):
    return x & pageoffset

def phys_to_pfn(paddr):
    x = ((paddr) >> PAGE_SHIFT)
    return x

def ARM64_HW_PGTABLE_LEVEL_SHIFT(n):
    return ((PAGE_SHIFT - 3) * (4 - (n)) + 3)

VA_BITS = 39
CONFIG_PGTABLE_LEVELS = 3
PGDIR_SHIFT = ARM64_HW_PGTABLE_LEVEL_SHIFT(4 - CONFIG_PGTABLE_LEVELS)
#define PGDIR_SIZE		(_AC(1, UL) << PGDIR_SHIFT)
#define PGDIR_MASK		(~(PGDIR_SIZE-1))
PTRS_PER_PGD	=	(1 << (VA_BITS - PGDIR_SHIFT))

VM_NONE = 0x00000000
VM_READ	= 0x00000001
VM_WRITE = 0x00000002
VM_EXEC = 0x00000004
VM_SHARED = 0x00000008

def pgd_index(addr):
    return (addr >> PGDIR_SHIFT) & (PTRS_PER_PGD - 1)

def pgd_offset_raw(pgd, addr):
    return pgd + pgd_index(addr) * 8

def pgd_offset(pgd, addr):
    return  pgd_offset_raw(pgd, addr)

PTRS_PER_PTE =	(1 << (PAGE_SHIFT - 3))
PTRS_PER_PMD =	PTRS_PER_PTE
PMD_SHIFT = ARM64_HW_PGTABLE_LEVEL_SHIFT(2)

def pmd_index(addr):
    return (((addr) >> PMD_SHIFT) & (PTRS_PER_PMD - 1))

def pte_index(addr):
    return (((addr) >> PAGE_SHIFT) & (PTRS_PER_PTE_L3_4K - 1))

def pte_show(pte):
    aList = []
    pte_dict = {'PTE_VALID': 1 << 0, 'PTE_WRITE': 1 << 51, 'PTE_DIRTY': 1 << 55, 'PTE_SPECIAL': 1 << 56,
                'PTE_PROT_NONE': 1 << 58, 'PTE_USER': 1 << 6, 'PTE_RDONLY': 1 << 7, 'PTE_SHARED': 1 << 8,
                'PTE_AF': 1 << 10, 'PTE_NG': 1 << 11, 'PTE_CONT': 1 << 52, 'PTE_PXN': 1 << 53, 'PTE_UXN': 1 << 54}
    for (d, x) in pte_dict.items():
        if pte & x:
            aList.append(d)
    return aList

def remove_non_ascii(a_str):
    ascii_chars = set(string.printable)
    return ''.join(
        filter(lambda x: x in ascii_chars, a_str)
    )

PTE_VALID = 1 << 0
PTE_PROT_NONE = 58 << 1


def do_get_task_info(ramdump, task):
    offset_comm = ramdump.field_offset('struct task_struct', 'comm')
    offset_pid = ramdump.field_offset('struct task_struct', 'pid')
    tgid_offset = ramdump.field_offset('struct task_struct', 'tgid')
    tgid = ramdump.read_int(task + tgid_offset)
    thread_task_name = cleanupString(
        ramdump.read_cstring(task + offset_comm, 16))
    if thread_task_name is None or thread_task_name == "":
        return thread_task_pid, None, None
    thread_task_pid = ramdump.read_int(task + offset_pid)
    mm = ramdump.read_structure_field(task, 'struct task_struct', 'mm')
    return thread_task_pid, thread_task_name, mm

@register_parser('--print-uvtop', 'Print task virtual memory to page info', optional=True)
class uvtop(RamParser):
    def __init__(self, *args):
        super(uvtop, self).__init__(*args)
        self.task_list = []
        self.flags_offset = self.ramdump.field_offset('struct page', 'flags')
        self.page_flags = {
            'PG_locked        ':0x000001 ,
            'PG_referenced    ':0x000002 ,
            'PG_uptodate      ':0x000004 ,
            'PG_dirty         ':0x000008 ,
            'PG_lru           ':0x000010 ,
            'PG_active        ':0x000020 ,
            'PG_workingset    ':0x000040 ,
            'PG_waiters       ':0x000080 ,
            'PG_error         ':0x000100 ,
            'PG_slab          ':0x000200 ,
            'PG_owner_priv_1  ':0x000400 ,
            'PG_arch_1        ':0x000800 ,
            'PG_reserved      ':0x001000 ,
            'PG_private       ':0x002000 ,
            'PG_private_2     ':0x004000 ,
            'PG_writeback     ':0x008000 ,
            'PG_head          ':0x010000 ,
            'PG_mappedtodisk  ':0x020000 ,
            'PG_reclaim       ':0x040000 ,
            'PG_swapbacked    ':0x080000 ,
            'PG_unevictable   ':0x100000 ,
            'PG_mlocked       ':0x200000 ,
            'PG_checked       ':0x000400 ,
            'PG_swapcache     ':0x000400 ,
            'PG_fscache       ':0x004000 ,
            'PG_pinned        ':0x000400 ,
            'PG_savepinned    ':0x000008 ,
            'PG_foreign       ':0x000400 ,
            'PG_slob_free     ':0x002000 ,
            'PG_double_map    ':0x004000 ,
            'PG_isolated      ':0x040000 ,
        }

    def uvtop(self, mm_struct, vaddr):
        if mm_struct == 0:
            return 0, PAGE_SIZE
        """
                arch/arm64/include/asm/pgtable-prot.h
        """
        pgd = self.ramdump.read_structure_field(mm_struct, 'struct mm_struct', 'pgd')
        pgd_ptr = pgd_offset(pgd, vaddr)
        pgd_val = self.ramdump.read_word(pgd_ptr)
        if pgd_val == 0 or pgd_val is None:
            return  0, 0x40000000           # 1G
        pmd_addr_phy = pgd_val & PHYS_MASK & pagemask
        pmd_base = phys_to_virt(self.ramdump, pmd_addr_phy)
        pmd_ptr = pmd_base + pmd_index(vaddr) * 8
        pmd_val = self.ramdump.read_word(pmd_ptr)
        if pmd_val == 0 or pmd_val is None:
            return  0, (512 * PAGE_SIZE)   # 2M
        pte_base_phy = pmd_val & PHYS_MASK & pagemask
        pte_base = phys_to_virt(self.ramdump, pte_base_phy)
        pte_index_value =  pte_index(vaddr)
        pte_ptr = pte_base + (pte_index_value << 3)
        pte_val = self.ramdump.read_word(pte_ptr)
        if pte_val == 0 or pte_val is None:
            return 0, PAGE_SIZE
        # check if it is swap_pte
        if (pte_val & PTE_VALID) == 0:
            return 0, PAGE_SIZE
        if pte_val & PTE_PROT_NONE  == 1:
            return 0, PAGE_SIZE
        page_addr_phy = pte_val & pagemask & PHYS_MASK + PAGEOFFSET(vaddr)  #arm64_vtop_3level_4k
        pfn = phys_to_pfn(page_addr_phy)
        page = pfn_to_page(self.ramdump, pfn)

        self.fout.write(
            "       start 0x{0:x} ->  0x{1:x} \n"
                .format(vaddr, vaddr + 0x1000))
        phy = self.mmu.virt_to_phys(vaddr)
        if phy != None:
            self.fout.write("           phy: 0x{0:x}\n".format(phy))
        self.fout.write("           PGD:  0x{0:16x} => {1:16x} \n".format(pgd_ptr, pgd_val))
        self.fout.write("           PMD:  0x{0:16x} => {1:16x} \n".format(pmd_ptr, pmd_val))
        self.fout.write("           PTE:  0x{0:16x} => {1:16x} \n".format(pte_ptr, pte_val))
        self.fout.write("           {0} \n".format(pte_show(pte_val)))
        self.fout.write("           page: 0x{0:16x}\n".format(page))
        self.fout.write("           pfn: 0x{0:x}\n".format(pfn))
        self.fout.write("           phy: 0x{0:x}\n".format(page_addr_phy))
        return page, PAGE_SIZE

    def get_dname_of_dentry(self, dentry):
        ramdump = self.ramdump
        dentry_name_offset = ramdump.field_offset(
            'struct dentry', 'd_name')
        len_offset = ramdump.field_offset(
            'struct qstr', 'len')
        qst_name_offset = ramdump.field_offset(
            'struct qstr', 'name')

        if dentry == None or dentry_name_offset == None or qst_name_offset == None:
            return None
        name_address = ramdump.read_word(dentry + dentry_name_offset + qst_name_offset)
        len_address = dentry + dentry_name_offset + len_offset
        len = ramdump.read_u32(len_address)
        name = cleanupString(ramdump.read_cstring(
            name_address, len))
        return name

    def dump_page(self, page):
        flags = self.ramdump.read_word(page + self.flags_offset)
        if flags is None:
            flags = 0
        aList= []
        for (d,x) in self.page_flags.items():
            if flags & x:
               aList.append(d)
        self.fout.write("           flags: {0} \n".format(aList))
        return flags

    def vm_area_struct_dump(self, vm_area_struct, mm):
        vm_start = self.ramdump.read_structure_field(vm_area_struct, 'struct vm_area_struct', 'vm_start')
        vm_end  = self.ramdump.read_structure_field(vm_area_struct, 'struct vm_area_struct', 'vm_end')
        vm_flags = self.ramdump.read_structure_field(vm_area_struct, 'struct vm_area_struct', 'vm_flags')
        vm_file = self.ramdump.read_structure_field(vm_area_struct, 'struct vm_area_struct', 'vm_file')
        vm_page_prot = self.ramdump.read_structure_field(vm_area_struct, 'struct vm_area_struct', 'vm_page_prot')
        if vm_area_struct == None:
            return
        if vm_start == None or vm_end == None:
            return
        mount_dir_full_name = ''
        '''
        anonymous memory/page, vm_file could NULL.
        '''
        if vm_file != None and vm_file != 0:
            f_path = self.ramdump.struct_field_addr(vm_file, 'struct file', 'f_path')

            dentry = self.ramdump.read_structure_field(
                f_path, 'struct path', 'dentry')
            d_parent_offset = self.ramdump.field_offset(
                'struct dentry', 'd_parent')
            d_parent = dentry
            d_parent_pre = 0
            names = []
            while d_parent_pre != d_parent and d_parent != None:
                d_parent_pre = d_parent
                name = self.get_dname_of_dentry(d_parent)
                d_parent = self.ramdump.read_word(d_parent + d_parent_offset)
                if name == None or name == '/':
                    break
                names.append(name)
                if d_parent == 0:
                    break
            names.reverse()
            mount_dir_full_name = ''
            for item in names:
                mount_dir_full_name += '/' + item
        if mount_dir_full_name == None:
            mount_dir_full_name = ''
        self.fout.write(
            "      0x{0:x} ->  0x{1:x} 0x{2:x}  {3:8x}  {4:x} {5:48s}\n"
                .format(vm_start, vm_end, vm_area_struct, vm_flags, vm_page_prot, mount_dir_full_name))
        start = vm_start
        while start < vm_end:
            if arm64_is_uvaddr(start) == 0:
                break
            page, step_size = self.uvtop(mm, start)
            if page != 0:
                flags = self.dump_page(page)
                if flags != 0:
                    self.page_count += 1
            start += step_size
    '''
    5.15 kernel
    '''
    def vm_area_struct_walker_5_1(self, rb_node, mm):
        vm_area_struct = self.ramdump.container_of(rb_node, 'struct vm_area_struct', 'vm_rb')
        self.vm_area_struct_dump(vm_area_struct, mm)

    def dump_vma_5_15(self, mm, pid, comm):
        if mm != 0:
            file_name = "{0}_{1}.txt".format(pid, comm)
            f_path = os.path.join(self.output_dir, file_name)
            self.fout = open(f_path, "w")
            self.fout.write(
                "------------comm {0:16s}  pid {1:6d}, mm 0x{2:x} ---------------- \n ".format(
                    comm, pid, mm))
            self.fout.write("VMA           START       END     FLAGS FILE\n")
            mm_rb = self.ramdump.read_structure_field(mm, 'struct mm_struct', 'mm_rb')
            rb_walker = rb_tree.RbTreeWalker(self.ramdump)
            if mm_rb == None or mm_rb == 0:
                return
            pgd = self.ramdump.read_structure_field(mm, 'struct mm_struct', 'pgd')
            pgdp = self.ramdump.virt_to_phys(pgd)
            self.mmu = Armv8MMU(self.ramdump, pgdp)
            rb_walker.walk(mm_rb, self.vm_area_struct_walker_5_1, mm)
            self.fout.close()
    '''
    6.1 kernel
    '''
    def vm_area_struct_walker_6_1(self, node, mm):
        if node !=0 and node != None:
            self.vm_area_struct_dump(node, mm)

    def dump_vma_6_1(self, mm, pid, comm):
        file_name = "{0}_{1}.txt".format(pid, comm)
        f_path = os.path.join(self.output_dir, file_name)
        self.fout = open(f_path, "w")
        self.fout.write(
            "------------comm {0:16s}  pid {1:6d}, mm 0x{2:x} ---------------- \n ".format(
                comm, pid, mm))
        self.fout.write("VMA           START       END     FLAGS FILE\n")
        pgd = self.ramdump.read_structure_field(mm, 'struct mm_struct', 'pgd')
        pgdp = self.ramdump.virt_to_phys(pgd)
        self.mmu = Armv8MMU(self.ramdump, pgdp)
        mt_walk = maple_tree.MapleTreeWalker(self.ramdump)
        mm_mt = self.ramdump.struct_field_addr(mm, 'struct mm_struct', 'mm_mt')
        mt_walk.walk(mm_mt, self.vm_area_struct_walker_6_1, mm)
        self.fout.close()

    def do_one_task(self, task, given_pid = -1):
        pid, thread_task_name, mm = do_get_task_info(self.ramdump, task)
        if given_pid != -1:
            if pid != given_pid:
                return
        comm = remove_non_ascii(thread_task_name)
        comm = re.sub(r'[^a-zA-Z0-9]', '', comm)

        if mm != 0:
            starttime = time.time()
            print_out_str('start {0} {1}'.format(pid, comm))
            if (self.ramdump.kernel_version) < (6, 1, 0):
                self.dump_vma_5_15(mm, pid, comm)
            else:
                self.dump_vma_6_1(mm, pid, comm)
            endtime = time.time()
            print_out_str('done {0} {1}'.format(pid, comm))
            print_out_str("     Time taken to complete : {}".format(endtime - starttime))
    def parse(self):
        '''
        --print-uvtop pid=1   # dump a task by given the pid, for example 1
        --print-uvtop         # dump all tasks
        '''
        pid_vm = None
        for arg in sys.argv:
            if "pid=" in arg:
                k, pid_vm = arg.split('=')
                pid_vm = int(pid_vm, 16)
                break
        if pid_vm == None:
            print_out_str("invalid PID, will to dump all pid")
            pid_vm = -1
        else:
            print_out_str("PID {0}".format(pid_vm))

        print_out_str("PGDIR_SHIFT {0}".format(PGDIR_SHIFT))
        print_out_str("PTRS_PER_PGD {0}".format(PTRS_PER_PGD))
        self.page_count  = 0
        self.output_dir = os.path.join(os.path.abspath(self.ramdump.outdir), "uvtop")
        if os.path.exists(self.output_dir) is False:
            os.makedirs(self.output_dir)

        for task in self.ramdump.for_each_process():
            self.do_one_task(task, pid_vm)

