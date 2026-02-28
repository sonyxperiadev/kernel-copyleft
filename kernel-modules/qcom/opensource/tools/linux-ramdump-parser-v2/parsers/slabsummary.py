# Copyright (c) 2012-2018, 2020, The Linux Foundation. All rights reserved.
# Copyright (c) 2022-2025, Qualcomm Innovation Center, Inc. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import math
import operator
from mm import pfn_to_page, get_pfn_range
from parser_util import register_parser, RamParser
# /kernel/msm-4.4/mm/slub.c
OO_SHIFT = 16
OO_MASK = (1 << OO_SHIFT) - 1

@register_parser('--slabsummary', 'print summary of slab', optional=True)
class Slabinfo_summary(RamParser):

    def cal_free_pages(self, start, slab_lru_offset, max_page, slab_addr, cpu):
        totalfree = 0
        page = self.ramdump.read_word(start)
        if page == 0:
            return totalfree
        seen = []
        count = 0
        total_objects = 0
        inuse = 0
        while page != start:
            valid_page = True
            if page is None:
                return totalfree
            #iterater partial slab may reach to NULL
            if page == 0:
                return totalfree
            if page in seen:
                return totalfree
            if page > max_page:
                return totalfree
            if page == 0xDEAD000000000100 or page == 0xDEAD000000000122:
                return totalfree
            seen.append(page)
            #c->slab page->insue always equal to page->objects, need NOT to consider
            #c->partial directly points to struct page(slab)
            if cpu == False:
                page = page - slab_lru_offset
            if (self.ramdump.kernel_version <= (4, 14)):
                    count = self.ramdump.read_structure_field(
                                page, 'struct page', '_mapcount')
            elif (self.ramdump.kernel_version <= (5, 17)):
                    count = self.ramdump.read_structure_field(
                                page, 'struct page', 'counters')
            else:
                    #struct slab are pulled out from struct page
                    count = self.ramdump.read_structure_field(
                                page, 'struct slab', 'counters')
                    slab_cache = self.ramdump.read_structure_field(
                                page, 'struct slab', 'slab_cache')
                    if slab_cache is None or slab_cache != slab_addr:
                        valid_page = False
            if valid_page:
                if not(count):
                    count = 0
                inuse = count & 0x0000FFFF
                total_objects = (count >> 16) & 0x00007FFF
                freeobj = total_objects - inuse
                totalfree = totalfree + freeobj
            page = self.ramdump.read_word(page + slab_lru_offset)
        return totalfree

    # Currently with assumption there is only one numa node
    def print_slab_summary(self, slab_out):
        total_freeobjects = 0
        slab_summary = {}
        nCounter = 0
        original_slab = self.ramdump.address_of('slab_caches')
        cpus = self.ramdump.get_num_cpus()
        slab_list_offset = self.ramdump.field_offset(
            'struct kmem_cache', 'list')
        slab_name_offset = self.ramdump.field_offset(
            'struct kmem_cache', 'name')
        slab_node_offset = self.ramdump.field_offset(
            'struct kmem_cache', 'node')
        cpu_partial_offset = self.ramdump.field_offset(
                'struct kmem_cache_cpu', 'partial')
        cpu_slab_offset = self.ramdump.field_offset(
            'struct kmem_cache', 'cpu_slab')
        slab_partial_offset = self.ramdump.field_offset(
            'struct kmem_cache_node', 'partial')
        slab = self.ramdump.read_word(original_slab)
        slab_lru_offset = self.ramdump.field_offset(
                                         'struct page', 'lru')
        if (self.ramdump.kernel_version > (5, 17)):
            slab_lru_offset = self.ramdump.field_offset(
                                             'struct slab', 'slab_list')
            slab_next = self.ramdump.field_offset(
                                             'struct slab', 'next')
        else:
            slab_next = slab_lru_offset
        pfn_range = get_pfn_range(self.ramdump)
        max_page = pfn_to_page(self.ramdump, pfn_range['max'])
        format_string = '\n{0:35} {1:9} {2:10} {3:10} {4:10} {5:8}K {6:8}' \
                        ' {7:10}K'
        slab_out.write(
            '{0:37} {1:12} {2:8} {3:10} {4:8} {5:12} {6:10} {7:10}'.format(
                                        "NAME", "OBJSIZE", "SIZE","ALLOCATED",
                                        "TOTAL", "TOTAL*SIZE", "SLABS",
                                        "SSIZE"))

        while slab != original_slab:
            total_freeobjects = 0
            slab = slab - slab_list_offset
            slab_name_addr = self.ramdump.read_word(
                                        slab + slab_name_offset)
            # actually an array but again, no numa
            slab_node_addr = self.ramdump.read_word(
                                        slab + slab_node_offset)
            slab_name = self.ramdump.read_cstring(
                                        slab_name_addr, 48)
            cpu_slab_addr = self.ramdump.read_word(
                                        slab + cpu_slab_offset)
            oo = self.ramdump.read_structure_field(
                        slab, 'struct kmem_cache', 'oo')
            obj_size = self.ramdump.read_structure_field(
                        slab, 'struct kmem_cache', 'object_size')
            objsize_w_metadata = self.ramdump.read_structure_field(
                        slab, 'struct kmem_cache', 'size')
            nr_total_objects = self.ramdump.read_structure_field(
                        slab_node_addr,
                        'struct kmem_cache_node', 'total_objects')
            num_slabs = self.ramdump.read_structure_field(
                        slab_node_addr,
                        'struct kmem_cache_node', 'nr_slabs')

            # per cpu slab
            for i in self.ramdump.iter_cpus():
                cpu_slabn_addr = cpu_slab_addr + self.ramdump.per_cpu_offset(i)
                if cpu_slabn_addr == 0 or cpu_slabn_addr is None:
                    break
                total_freeobjects = total_freeobjects + self.cal_free_pages(
                                (cpu_slabn_addr + cpu_partial_offset),
                                slab_next, max_page, slab, True)
            # partial node
            total_freeobjects = total_freeobjects + self.cal_free_pages(
                                slab_node_addr + slab_partial_offset,
                                slab_lru_offset, max_page, slab, False)

            total_allocated = nr_total_objects - total_freeobjects
            page_order = (oo >> OO_SHIFT) & OO_MASK
            slab_size = int(math.pow(2, page_order + self.ramdump.page_shift))
            slab_size = slab_size // 1024
            slab = self.ramdump.read_word(slab + slab_list_offset)
            slab_summary[nCounter] = [
                    slab_name, obj_size, objsize_w_metadata,
                    total_allocated, nr_total_objects,
                    (objsize_w_metadata * nr_total_objects) // 1024,
                    num_slabs, slab_size]
            nCounter += 1
        sorted_summary = sorted(slab_summary.values(),
                                key=operator.itemgetter(5), reverse=True)
        total_slab_object_size = 0
        total_slab_page_size = 0
        for val in sorted_summary:
            total_slab_object_size += val[5]
            total_slab_page_size += (val[6] * val[7])
            slab_out.write(format_string.format(
                                val[0], val[1], val[2], val[3], val[4],
                                val[5], val[6], val[7]))
        slab_out.write("\n\nTotal object size: {} KB\n".format(total_slab_object_size))
        slab_out.write("Total page size: {} KB\n".format(total_slab_page_size))
        return

    def parse(self):
        slab_out = self.ramdump.open_file('slabsummary.txt')
        self.print_slab_summary(slab_out)
        slab_out.close()
