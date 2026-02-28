# Copyright (c) 2016-2021 The Linux Foundation. All rights reserved.
# Copyright (c) 2023-2025 Qualcomm Innovation Center, Inc. All rights reserved.
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from print_out import print_out_str
from parser_util import register_parser, RamParser, cleanupString
from linux_list import ListWalker
from parsers.filetracking import FileTracking
from .memstat import MemStats

# This will be reset again later by set_page_shift

def get_shmem_swap_usage(ramdump):
    # Returns memory in pages
    shmem_swaplist = ramdump.address_of("shmem_swaplist")
    if not shmem_swaplist:
        return 0

    offset = ramdump.field_offset('struct shmem_inode_info', 'swaplist')
    if not offset:
        return 0
    inode_offset = ramdump.field_offset('struct shmem_inode_info', 'vfs_inode')
    if not inode_offset:
        return 0

    iter = ListWalker(ramdump, shmem_swaplist, offset)

    total = 0
    seen = {}
    for shmem_inode_info in iter:
        swap_pages = ramdump.read_structure_field(
                    shmem_inode_info, 'struct shmem_inode_info', 'swapped')
        if swap_pages is None:
            print_out_str("Invalid addr is found: {}".format(hex(shmem_inode_info)))
            break
        inode = shmem_inode_info + inode_offset
        addres_space = ramdump.read_structure_field(inode, 'struct inode',
                                        'i_mapping')
        if addres_space in seen:
            seen[addres_space] = seen[addres_space] + swap_pages
        else:
            seen[addres_space] = swap_pages
        total += swap_pages

    sortlist = sorted(seen.items(),  key=lambda kv: kv[1],
                    reverse=True)
    i = 0
    string = "TOP 3 swapped SHMEM files are:\n"
    pathtracking = FileTracking(ramdump)
    for k,v in sortlist:
        #k is struct address_space
        if i < 3:
            i = i + 1
            addr_space_format = "Address_space 0x{0:x} Allocated {1} pages\n".format(k,v)
            string = string + addr_space_format
            inode = ramdump.read_structure_field(k, 'struct address_space',
                    'host')
            if inode is not None:
                dentry_list = ramdump.read_structure_field(inode, 'struct inode',
                        'i_dentry')
            if dentry_list is not None:
                dentry = ramdump.container_of(dentry_list, 'struct dentry',
                        'd_u')
            if dentry is not None:
                d_name_ptr = (dentry + ramdump.field_offset('struct dentry ',
                    'd_name')) + ramdump.field_offset('struct qstr', 'name')
                name = ramdump.read_cstring(ramdump.read_pointer(d_name_ptr),
                        100)
                if name is not None:
                    path, cycle_flag = pathtracking.get_filepath('', name, dentry)
                else:
                    path = 'None'
                path = "file name:  " + path + '\n'
                string = string + path
        else:
            break

    return total,string

def do_dump_process_memory(ramdump):
    memstat = MemStats(ramdump)

    memory_file = ramdump.open_file('memory.txt')

    #######################    Total RAM   #######################
    total_mem_pages = memstat.calculate_totalmem_pages()
    total_mem_kB = memstat.pages_to_kb(total_mem_pages)
    memory_file.write('Total RAM        : {:>10,} kB\n'.format(total_mem_kB))
    unaccounted = total_mem_kB

    #######################   Free memory  #######################
    total_free_pages = memstat.calculate_totalfree_pages()
    total_free_kB = memstat.pages_to_kb(total_free_pages)
    memory_file.write('Free memory      : {:>10,} kB ({:>5.1f}%)\n'.format(
            total_free_kB, (100.0 * total_free_kB) / total_mem_kB))
    unaccounted -= total_free_kB

    #######################   File Cache   #######################
    total_shmem_kB = memstat.pages_to_kb(ramdump.read_word('vm_node_stat[NR_SHMEM]'))
    file_cache_kB =  memstat.pages_to_kb(memstat.calculate_cached_pages())
    active_kB = memstat.pages_to_kb(ramdump.read_word('vm_node_stat[NR_ACTIVE_FILE]'))
    inactive_kB = memstat.pages_to_kb(ramdump.read_word('vm_node_stat[NR_INACTIVE_FILE]'))

    memory_file.write('File Cache       : {:>10,} kB ({:>5.1f}%)'.format(
            file_cache_kB, (100.0 * file_cache_kB / total_mem_kB)))
    memory_file.write(' (Active: {0:,} kB'.format(active_kB))
    memory_file.write(', Inactive {0:,} kB'.format(inactive_kB))
    memory_file.write(', Shmem {0:,} kB'.format(total_shmem_kB))
    memory_file.write(', Other {0:,} kB)\n'.format(file_cache_kB - active_kB - inactive_kB - total_shmem_kB))
    unaccounted -= file_cache_kB

    #######################      Slab     #######################
    slab_rec_pages, slab_unrec_pages = memstat.calculate_slab_mem_pages()
    slab_rec_kB = memstat.pages_to_kb(slab_rec_pages)
    slab_unrec_kB = memstat.pages_to_kb(slab_unrec_pages)
    total_slab_kB = (slab_rec_kB + slab_unrec_kB)

    memory_file.write('Slab             : {:>10,} kB ({:>5.1f}%)'.format(
            total_slab_kB, (100.0 * total_slab_kB / total_mem_kB)))
    memory_file.write(' (reclaimable: {:,} kB'.format(slab_rec_kB))
    memory_file.write(', unreclaimable {:,} kB)\n'.format(slab_unrec_kB))
    unaccounted -= total_slab_kB

    #######################    PageTable   #######################
    pagetable_kB = memstat.pages_to_kb(ramdump.read_word('vm_node_stat[NR_PAGETABLE]'))
    memory_file.write('PageTable        : {:>10,} kB ({:>5.1f}%)\n'.format(
        pagetable_kB, (pagetable_kB * 100 / total_mem_kB)))
    unaccounted -= pagetable_kB

    #######################    Modules    #######################
    # TODO: include modules memory consumption here
    # memory_file.write('Modules          : \n')
    # Do not remove modules from unaccounted as we will remove vmalloc from unaccounted
    # And vmalloc includes modules memory

    #######################  Kernel_Stack  #######################
    kstack_kB = ramdump.read_word('vm_node_stat[NR_KERNEL_STACK_KB]')
    memory_file.write('Kernel_Stack     : {:>10,} kB ({:>5.1f}%)\n'.format(
            kstack_kB, (100 * kstack_kB / total_mem_kB)))
    # If CONFIG_VMAP_STACK is defined, kernel stack is included as part of vmalloc.
    # So, do not remove kstack from unaccounted as we will remove vmalloc from unaccounted
    if not ramdump.is_config_defined('CONFIG_VMAP_STACK'):
        unaccounted -= kstack_kB

    #######################     Vmalloc    #######################
    vmalloc_pages = memstat.calculate_vmalloc_pages()
    vmalloc_kb = memstat.pages_to_kb(vmalloc_pages)
    memory_file.write('Vmalloc          : {:>10,} kB ({:>5.1f}%)'.format(
        vmalloc_kb, (100.0 * vmalloc_kb / total_mem_kB)))
    if ramdump.is_config_defined('CONFIG_VMAP_STACK'):
        memory_file.write(' (Kernel_Stack: {:,} kB'.format(kstack_kB))
        memory_file.write(', Modules + Others {:,} kB)\n'.format(vmalloc_kb - kstack_kB))
    else:
        memory_file.write('(Includes Modules)\n')
    unaccounted -= vmalloc_kb

    #######################      Shmem     #######################
    total_shmem_swap, shmem_swap_file = get_shmem_swap_usage(ramdump)
    total_shmem_swap_kB = memstat.pages_to_kb(total_shmem_swap)
    memory_file.write('Shmem            : {:>10,} kB ({:>5.1f}%)'.format(
            (total_shmem_kB + total_shmem_swap_kB), (100.0 * total_shmem_kB / total_mem_kB)))
    memory_file.write(' (PageCache: {:,} kB'.format(total_shmem_kB))
    memory_file.write(', Swap: {:,} kB)\n'.format(total_shmem_swap_kB))

    #######################      Anon      #######################
    anon = ramdump.read_word('vm_node_stat[NR_ANON_MAPPED]')
    if anon == None or anon == 0:
        anon =  ramdump.read_word('vm_stat[NR_ACTIVE_ANON]')
        anon += ramdump.read_word('vm_stat[NR_INACTIVE_ANON]')
    anon_kB = memstat.pages_to_kb(anon)
    memory_file.write('Anon (PSS)       : {:>10,} kB ({:>5.1f}%)\n'.format(
            anon_kB, (100.0 * anon_kB / total_mem_kB)))
    unaccounted -= anon_kB

    #######################     Graphics    #######################
    kgsl_pages = memstat.calculate_kgsl_mem_pages()
    kgsl_kb    = memstat.pages_to_kb(kgsl_pages)
    memory_file.write('Graphics (KGSL)  : {:>10,} kB ({:>5.1f}%)\n'.format(
            kgsl_kb, (100.0 * kgsl_kb / total_mem_kB)))
    unaccounted -= kgsl_kb

    #######################    ION (DMA)    #######################
    if ramdump.kernel_version >= (5, 10):
        ion_mem_pages = memstat.read_dma_heap_mem_pages()
        if ion_mem_pages > 0:
            ion_mem_kb = memstat.pages_to_kb(ion_mem_pages)
            memory_file.write('DMA (ION)        : {:>10,} kB ({:>5.1f}%)\n'.format(
                    ion_mem_kb, (100.0 * ion_mem_kb / total_mem_kB)))
            unaccounted -= ion_mem_kb
        else:
            memory_file.write('DMA (ION)        : {:<22} Please parse ionbuffer first, use --print-ionbuffer.\n'.format(" "))
    else:
        ion_mem_pages = memstat.calculate_ionmem_pages()
        ion_mem_kb = memstat.pages_to_kb(ion_mem_pages)
        memory_file.write('ION                : {:>10,} kB ({:>5.1f}%)\n'.format(
                ion_mem_kb, (100.0 * ion_mem_kb / total_mem_kB)))
        unaccounted -= ion_mem_kb
    #######################    ION POOL    #######################
    kern_misc_rec_pages = memstat.calculate_kernel_misc_rec_pages()
    Kern_misc_rec = memstat.pages_to_kb(kern_misc_rec_pages)
    memory_file.write('Kernel Misc Recl : {:>10,} kB ({:>5.1f}%)\n'.format(
            Kern_misc_rec, (100.0 * Kern_misc_rec / total_mem_kB)))
    unaccounted -= Kern_misc_rec

    #######################   CMA (Used)   #######################
    # TODO: include Used CMA memory as well
    # memory_file.write('CMA (Used)       : \n')

    #######################    ZRAM        #######################
    z_used_pages = memstat.calculate_zram_compressed_pages()
    z_used_kB = memstat.pages_to_kb(z_used_pages)
    if z_used_kB == None:
        memory_file.write('Zram (Used)      : {:<22} Unable to parse ZRAM.\n'.format(" "))
    else:
        memory_file.write('Zram (Used)      : {:>10,} kB ({:>5.1f}%)\n'.format(
                z_used_kB, (z_used_kB * 100 / total_mem_kB)))
        unaccounted -= z_used_kB

    #######################    Swapcached   #######################
    swapcached_kB = 0
    swapcached_pages = ramdump.read_word('vm_stat[NR_SWAPCACHE]')
    swapcached_kB = memstat.pages_to_kb(swapcached_pages)
    memory_file.write('Swapcached       : {:>10,} kB ({:>5.1f}%)\n'.format(
            swapcached_kB, (swapcached_kB * 100 / total_mem_kB)))
    unaccounted += swapcached_kB

    #######################   Unaccounted   #######################
    memory_file.write('Unaccounted      : {:>10,} kB ({:>5.1f}%)\n'.format(
            unaccounted, (unaccounted * 100 / total_mem_kB)))

    offset_comm = ramdump.field_offset('struct task_struct', 'comm')
    offset_signal = ramdump.field_offset('struct task_struct', 'signal')
    offset_adj = ramdump.field_offset('struct signal_struct', 'oom_score_adj')
    offset_pid = ramdump.field_offset('struct task_struct', 'pid')
    task_info = []

    memory_file.write('{0}\n'.format(shmem_swap_file))

    for task in ramdump.for_each_process():
        next_thread_comm = task + offset_comm
        thread_task_name = cleanupString(
            ramdump.read_cstring(next_thread_comm, 16))
        next_thread_pid = task + offset_pid
        thread_task_pid = ramdump.read_int(next_thread_pid)
        signal_struct = ramdump.read_word(task + offset_signal)
        if signal_struct == 0 or signal_struct is None:
            continue

        adj = ramdump.read_u16(signal_struct + offset_adj)
        if adj & 0x8000:
            adj = adj - 0x10000
        rss, swap, anon_rss, file_rss, shmem_rss = get_rss(ramdump, task)
        if rss != 0 or swap != 0:
            task_info.append([thread_task_name, thread_task_pid, rss, swap, rss + swap, adj, anon_rss, file_rss, shmem_rss])

    task_info = sorted(task_info, key=lambda l: l[4], reverse=True)
    str = '{0:<17s}{1:>8s}{2:>19s}{3:>19s}{4:>6}{5:>16}{6:>16}{7:>16}\n'.format(
        'Task name', 'PID', 'RSS in kB', 'SWAP in kB', 'ADJ', "anon_rss in kB", "file_rss in kB", "shmem_rss in kB")
    memory_file.write(str)
    for item in task_info:
        str = '{taskname:<17s}{pid:8d}{rss:13,d}({rss_pct:4.1f}%){swap:13,d}({swap_pct:2.1f}%){adj:6} {anon_rss:>16,d} {file_rss:>16,d} {shmem_rss:>10,d}\n'.format(
            taskname = item[0], pid = item[1],
            rss = item[2], rss_pct = (100.0 * item[2]) / total_mem_kB,
            swap = item[3], swap_pct = (100.0 * item[3]) / total_mem_kB,
            adj = item[5], anon_rss=item[6], file_rss=item[7], shmem_rss=item[8])
        memory_file.write(str)
    memory_file.close()
    print_out_str('---wrote meminfo to memory.txt')

def percpu_counter_rss_stat(ramdump, rss_stat):
    count = rss_stat.count
    for core in ramdump.iter_cpus():
        count += ramdump.read_int(rss_stat.counters + ramdump.per_cpu_offset(core))
    return count

def get_mm_counter(ramdump, rss_stat):
    count = rss_stat.count
    return count

def get_rss(ramdump, task_struct):
    offset_mm = ramdump.field_offset('struct task_struct', 'mm')
    offset_rss_stat = ramdump.field_offset('struct mm_struct', 'rss_stat')
    mm_struct = ramdump.read_word(task_struct + offset_mm)
    if mm_struct == 0:
        return 0, 0, 0, 0, 0
    if ramdump.kernel_version >= (6, 2):
        # /* 6.2: struct percpu_counter rss_stat[NR_MM_COUNTERS] */
        mm = ramdump.read_datatype(mm_struct, 'struct mm_struct')
        file_rss = get_mm_counter(ramdump, mm.rss_stat[0])
        anon_rss = get_mm_counter(ramdump, mm.rss_stat[1])
        swap_rss = get_mm_counter(ramdump, mm.rss_stat[2])
        shmem_rss = get_mm_counter(ramdump, mm.rss_stat[3])

    else:
        offset_file_rss = ramdump.field_offset('struct mm_rss_stat', 'count')
        offset_anon_rss = ramdump.field_offset('struct mm_rss_stat', 'count[1]')
        offset_swap_rss = ramdump.field_offset('struct mm_rss_stat', 'count[2]')
        if ramdump.kernel_version >= (4, 9):
            offset_shmem_rss = ramdump.field_offset('struct mm_rss_stat', 'count[3]')

        anon_rss = ramdump.read_word(mm_struct + offset_rss_stat + offset_anon_rss)
        swap_rss = ramdump.read_word(mm_struct + offset_rss_stat + offset_swap_rss)
        file_rss = ramdump.read_word(mm_struct + offset_rss_stat + offset_file_rss)
        if ramdump.kernel_version >= (4, 9):
            shmem_rss = ramdump.read_word(mm_struct + offset_rss_stat + offset_shmem_rss)
        else:
            shmem_rss = 0
    # Ignore negative RSS values
    if anon_rss > 0x80000000:
        anon_rss = 0
    if swap_rss > 0x80000000:
        swap_rss = 0
    if file_rss > 0x80000000:
        file_rss = 0
    if shmem_rss > 0x80000000:
        shmem_rss = 0
    total_rss = anon_rss + file_rss + shmem_rss
    return total_rss * 4 , swap_rss * 4 , anon_rss * 4 , file_rss * 4,  shmem_rss * 4


@register_parser('--print-memory-info', 'Print memory usage info')
class DumpProcessMemory(RamParser):
    def parse(self):
        do_dump_process_memory(self.ramdump)
