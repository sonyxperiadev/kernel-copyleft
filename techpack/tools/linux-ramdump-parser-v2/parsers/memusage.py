# Copyright (c) 2016-2021 The Linux Foundation. All rights reserved.
#
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

""" Returns number of pages """
def get_shmem_swap_usage(ramdump, memory_file):
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
        inode = shmem_inode_info + inode_offset
        addres_space = ramdump.read_structure_field(inode, 'struct inode',
                                        'i_mapping')
        if addres_space in seen:
            seen[addres_space] = seen[addres_space] + swap_pages
        else:
            seen[addres_space] = swap_pages
        total += ramdump.read_structure_field(
                    shmem_inode_info, 'struct shmem_inode_info', 'swapped')

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
                path, cycle_flag = pathtracking.get_filepath('', name, dentry)
                path = "file name:  " + path + '\n'
                string = string + path
        else:
            break

    return total,string


def do_dump_process_memory(ramdump):
    if ramdump.kernel_version < (4, 9):
        total_free = ramdump.read_word('vm_stat[NR_FREE_PAGES]')
        slab_rec = ramdump.read_word('vm_stat[NR_SLAB_RECLAIMABLE]')
        slab_unrec = ramdump.read_word('vm_stat[NR_SLAB_UNRECLAIMABLE]')
        total_shmem = ramdump.read_word('vm_stat[NR_SHMEM]')
    else:
        total_free = ramdump.read_word('vm_zone_stat[NR_FREE_PAGES]')
        # slab memory
        if ramdump.kernel_version >= (5, 10):
            slab_rec = ramdump.read_word('vm_node_stat[NR_SLAB_RECLAIMABLE_B]')
            slab_unrec = ramdump.read_word(
                            'vm_node_stat[NR_SLAB_UNRECLAIMABLE_B]')
        elif ramdump.kernel_version >= (4, 14):
            slab_rec = ramdump.read_word('vm_node_stat[NR_SLAB_RECLAIMABLE]')
            slab_unrec = ramdump.read_word(
                            'vm_node_stat[NR_SLAB_UNRECLAIMABLE]')
        else:
            slab_rec = ramdump.read_word('vm_zone_stat[NR_SLAB_RECLAIMABLE]')
            slab_unrec = ramdump.read_word(
                            'vm_zone_stat[NR_SLAB_UNRECLAIMABLE]')
        total_shmem = ramdump.read_word('vm_node_stat[NR_SHMEM]')

    memory_file = ramdump.open_file('memory.txt')
    total_shmem_swap, shmem_swap_file = get_shmem_swap_usage(ramdump,memory_file)
    total_slab = slab_rec + slab_unrec
    if(ramdump.kernel_version > (4, 20, 0)):
        total_mem = ramdump.read_word('_totalram_pages') * 4
    else:
        total_mem = ramdump.read_word('totalram_pages') * 4

    offset_comm = ramdump.field_offset('struct task_struct', 'comm')
    offset_signal = ramdump.field_offset('struct task_struct', 'signal')
    offset_adj = ramdump.field_offset('struct signal_struct', 'oom_score_adj')
    offset_pid = ramdump.field_offset('struct task_struct', 'pid')
    task_info = []
    memory_file.write('Total RAM: {0:,}kB\n'.format(total_mem))
    memory_file.write('Total free memory: {0:,}kB({1:.1f}%)\n'.format(
            total_free * 4, (100.0 * total_free * 4) / total_mem))
    memory_file.write('Slab reclaimable: {0:,}kB({1:.1f}%)\n'.format(
            slab_rec * 4, (100.0 * slab_rec * 4) / total_mem))
    memory_file.write('Slab unreclaimable: {0:,}kB({1:.1f}%)\n'.format(
            slab_unrec * 4, (100.0 * slab_unrec * 4) / total_mem))
    memory_file.write('Total Slab memory: {0:,}kB({1:.1f}%)\n'.format(
            total_slab * 4, (100.0 * total_slab * 4) / total_mem))
    memory_file.write('Total SHMEM (PAGECACHE): {0:,}kB({1:.1f}%)\n'.format(
        total_shmem * 4, (100.0 * total_shmem * 4) / total_mem))
    memory_file.write('Total SHMEM (SWAP): {0:,}kB({1:.1f}%)\n\n'.format(
        total_shmem_swap * 4, (100.0 * total_shmem_swap * 4) / total_mem))
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
        rss, swap = get_rss(ramdump, task)
        if rss != 0:
            task_info.append([thread_task_name, thread_task_pid, rss, swap, rss + swap, adj])

    task_info = sorted(task_info, key=lambda l: l[4], reverse=True)
    str = '{0:<17s}{1:>8s}{2:>19s}{3:>19s}{4:>6}\n'.format(
        'Task name', 'PID', 'RSS in kB', 'SWAP in kB', 'ADJ')
    memory_file.write(str)
    for item in task_info:
        str = '{taskname:<17s}{pid:8d}{rss:13,d}({rss_pct:2.1f}%){swap:13,d}({swap_pct:2.1f}%){adj:6}\n'.format(
            taskname = item[0], pid = item[1],
            rss = item[2], rss_pct = (100.0 * item[2]) / total_mem,
            swap = item[3], swap_pct = (100.0 * item[3]) / total_mem,
            adj = item[5])
        memory_file.write(str)
    memory_file.close()
    print_out_str('---wrote meminfo to memory.txt')


def get_rss(ramdump, task_struct):
    offset_mm = ramdump.field_offset('struct task_struct', 'mm')
    offset_rss_stat = ramdump.field_offset('struct mm_struct', 'rss_stat')
    offset_file_rss = ramdump.field_offset('struct mm_rss_stat', 'count')
    offset_anon_rss = ramdump.field_offset('struct mm_rss_stat', 'count[1]')
    offset_swap_rss = ramdump.field_offset('struct mm_rss_stat', 'count[2]')
    if ramdump.kernel_version >= (4, 9):
        offset_shmem_rss = ramdump.field_offset('struct mm_rss_stat', 'count[3]')
    mm_struct = ramdump.read_word(task_struct + offset_mm)
    if mm_struct == 0:
        return 0, 0
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
    return total_rss * 4, swap_rss * 4


@register_parser('--print-memory-info', 'Print memory usage info')
class DumpProcessMemory(RamParser):

    def parse(self):
        do_dump_process_memory(self.ramdump)
