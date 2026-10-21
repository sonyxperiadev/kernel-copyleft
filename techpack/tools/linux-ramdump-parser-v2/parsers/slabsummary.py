# Copyright (c) 2012-2018, 2020, The Linux Foundation. All rights reserved.
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
from mm import pfn_to_page
from parser_util import register_parser, RamParser
# /kernel/msm-4.4/mm/slub.c
OO_SHIFT = 16
PAGE_SHIFT = 12


@register_parser('--slabsummary', 'print summary of slab', optional=True)
class Slabinfo_summary(RamParser):

    def cal_free_pages(
                        self, ramdump,
                        start,  slab_lru_offset,
                        max_page):
        page = self.ramdump.read_word(start)
        totalfree = 0
        if page == 0:
            return totalfree
        seen = []
        count = 0
        total_objects = 0
        inuse = 0
        while page != start:
            if page is None:
                return totalfree
            if page in seen:
                return totalfree
            if page > max_page:
                return totalfree
            seen.append(page)
            page = page - slab_lru_offset
            if (self.ramdump.kernel_version <= (4, 14)):
                    count = self.ramdump.read_structure_field(
                                page, 'struct page', '_mapcount')
            else:
                    count = self.ramdump.read_structure_field(
                                page, 'struct page', 'counters')
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
        cpu_cache_page_offset = self.ramdump.field_offset(
            'struct kmem_cache_cpu', 'page')
        cpu_slab_offset = self.ramdump.field_offset(
            'struct kmem_cache', 'cpu_slab')
        slab_partial_offset = self.ramdump.field_offset(
            'struct kmem_cache_node', 'partial')
        slab = self.ramdump.read_word(original_slab)
        slab_lru_offset = self.ramdump.field_offset(
                                         'struct page', 'lru')
        max_pfn_addr = self.ramdump.address_of('max_pfn')
        max_pfn = self.ramdump.read_word(max_pfn_addr)
        max_page = pfn_to_page(self.ramdump, max_pfn)
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
            for i in range(0, cpus):
                cpu_slabn_addr = self.ramdump.read_word(
                                            cpu_slab_addr, cpu=i)
                if cpu_slabn_addr == 0 or cpu_slabn_addr is None:
                    break
                total_freeobjects = total_freeobjects + self.cal_free_pages(
                                self.ramdump,
                                (cpu_slabn_addr + cpu_cache_page_offset),
                                slab_lru_offset,
                                max_page)

            total_freeobjects = total_freeobjects + self.cal_free_pages(
                                self.ramdump,
                                slab_node_addr + slab_partial_offset,
                                slab_lru_offset, max_page)

            total_allocated = nr_total_objects - total_freeobjects
            page_order = oo >> OO_SHIFT
            slab_size = int(math.pow(2, page_order + PAGE_SHIFT))
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
        for val in sorted_summary:
            slab_out.write(format_string.format(
                                val[0], val[1], val[2], val[3], val[4],
                                val[5], val[6], val[7]))

    def parse(self):
        slab_out = self.ramdump.open_file('slabsummary.txt')
        if(self.ramdump.is_config_defined('CONFIG_SLUB_DEBUG_ON')):
            self.print_slab_summary(slab_out)
        else:
            slab_out.write('CONFIG_SLUB_DEBUG_ON is disabled in this build')
        slab_out.close()
