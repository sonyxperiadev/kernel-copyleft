# Copyright (c) 2016-2021 The Linux Foundation. All rights reserved.
# Copyright (c) 2022-2025 Qualcomm Innovation Center, Inc. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from parser_util import register_parser, RamParser
import os
import linux_radix_tree
from .vmalloc import Vmalloc

@register_parser('--print-memstat', 'Print memory stats ')
class MemStats(RamParser):
    def __init__(self, dump):
        super(MemStats, self).__init__(dump)

        if (self.ramdump.kernel_version >= (5, 4)):
            self.zram_dev_rtw = linux_radix_tree.RadixTreeWalker(self.ramdump)
            self.zram_mem_mb = 0

        self.out_mem_stat = None

    def pages_to_mb(self, pages):
        val = 0
        if pages != None and pages != 0:
            val = (pages * self.ramdump.get_page_size()) >> 20
        return val

    def pages_to_kb(self, pages):
        val = 0
        if pages != None and pages != 0:
            val = (pages * self.ramdump.get_page_size()) >> 10
        return val

    def bytes_to_pages(self, bytes):
        val = 0
        if bytes != None and bytes != 0:
            val = bytes >> self.ramdump.page_shift
        return val

    def bytes_to_mb(self, bytes):
        val = 0
        if bytes != None and bytes != 0:
            val = bytes >> 20
        return val

    def calculate_totalmem_pages(self):
        # Returns memory in pages
        if(self.ramdump.kernel_version > (4, 20, 0)):
            return self.ramdump.read_word('_totalram_pages')

        return self.ramdump.read_word('totalram_pages')

    def calculate_totalfree_pages(self):
        # Returns memory in pages
        if (self.ramdump.kernel_version < (4, 9, 0)):
           # Free Memory
           return self.ramdump.read_word('vm_stat[NR_FREE_PAGES]')

        return self.ramdump.read_word('vm_zone_stat[NR_FREE_PAGES]')

    def calculate_vm_stat_pages(self):
        # Returns memory in pages
        # Other memory :  NR_ANON_PAGES + NR_FILE_PAGES + NR_PAGETABLE \
        #                 - NR_SWAPCACHE
        vmstat_anon_pages = self.ramdump.read_word(
                            'vm_stat[NR_ANON_PAGES]')
        vmstat_file_pages = self.ramdump.read_word(
                            'vm_stat[NR_FILE_PAGES]')
        vmstat_pagetbl = self.ramdump.read_word(
                                'vm_stat[NR_PAGETABLE]')
        if self.ramdump.is_config_defined('CONFIG_VMAP_STACK'):
            # vm_stat[NR_KERNEL_STACK] is already part of vmalloc. So no need to include it
            vmstat_kernelstack = 0
        else:
            vmstat_kernelstack = self.ramdump.read_word(
                                'vm_stat[NR_KERNEL_STACK]')
        vmstat_swapcache = self.ramdump.read_word(
                            'vm_stat[NR_SWAPCACHE]')
        other_mem = (vmstat_anon_pages + vmstat_file_pages + vmstat_pagetbl +
                     vmstat_kernelstack - vmstat_swapcache)

        return other_mem

    def calculate_vm_node_zone_stat_pages(self):
        # Other memory :  NR_ANON_MAPPED + NR_FILE_PAGES + NR_PAGETABLE
        vmstat_anon_pages = self.ramdump.read_word(
                            'vm_node_stat[NR_ANON_MAPPED]')
        vmstat_file_pages = self.ramdump.read_word(
                            'vm_node_stat[NR_FILE_PAGES]')
        if self.ramdump.kernel_version >= (5, 15):
            vmstat_pagetbl = self.ramdump.read_word(
                                'vm_node_stat[NR_PAGETABLE]')
            if self.ramdump.is_config_defined('CONFIG_VMAP_STACK'):
                # vm_stat[NR_KERNEL_STACK] is already part of vmalloc. So no need to include it
                vmstat_kernelstack = 0
            else:
                vmstat_kernelstack = self.ramdump.read_word(
                                'vm_node_stat[NR_KERNEL_STACK_KB]')
        else:
            vmstat_pagetbl = self.ramdump.read_word(
                                'vm_zone_stat[NR_PAGETABLE]')
            if self.ramdump.is_config_defined('CONFIG_VMAP_STACK'):
                # vm_stat[NR_KERNEL_STACK] is already part of vmalloc. So no need to include it
                vmstat_kernelstack = 0
            else:
                vmstat_kernelstack = self.ramdump.read_word(
                                'vm_zone_stat[NR_KERNEL_STACK_KB]')
        other_mem = (vmstat_anon_pages + vmstat_file_pages + vmstat_pagetbl +
                     (vmstat_kernelstack // 4))

        return other_mem

    def calculate_other_mem_pages(self):
        # Returns memory in pages
        if (self.ramdump.kernel_version < (4, 9, 0)):
           return self.calculate_vm_stat_pages()

        return self.calculate_vm_node_zone_stat_pages()

    def calculate_vmalloc_pages(self):
        memstat_vmalloc = Vmalloc(self.ramdump)
        vmalloc_pages = memstat_vmalloc.get_vmalloc_pages()
        self.vmalloc_size = self.pages_to_mb(vmalloc_pages)
        return vmalloc_pages

    def calculate_cached_pages(self):
        # Returns memory in pages
        if self.ramdump.kernel_version >= (4, 9):
            return self.ramdump.read_word(
                            'vm_node_stat[NR_FILE_PAGES]')

        return self.ramdump.read_word(
                        'vm_stat[NR_FILE_PAGES]')

    def calculate_ionmem_pages(self):
        # Returns memory in MB
        if self.ramdump.kernel_version >= (5, 10):
            grandtotal = 0
        elif self.ramdump.kernel_version >= (5, 4):
            grandtotal = self.ramdump.read_u64('total_heap_bytes')
        else:
            number_of_ion_heaps = self.ramdump.read_int('num_heaps')
            heap_addr = self.ramdump.read_word('heaps')

            offset_total_allocated = \
            self.ramdump.field_offset(
                'struct ion_heap', 'total_allocated')
            size = self.ramdump.sizeof(
                '((struct ion_heap *)0x0)->total_allocated')

            if offset_total_allocated is None:
                return "ion buffer debugging change is not there in this kernel"
            if self.ramdump.arm64:
                addressspace = 8
            else:
                addressspace = 4
            heap_addr_array = []
            grandtotal = 0
            for i in range(0, number_of_ion_heaps):
                heap_addr_array.append(heap_addr + i * addressspace)
                temp = self.ramdump.read_word(heap_addr_array[i])
                if size == 4:
                    total_allocated = self.ramdump.read_int(
                                    temp + offset_total_allocated)
                if size == 8:
                    total_allocated = self.ramdump.read_u64(
                                    temp + offset_total_allocated)
                if total_allocated is None:
                    total_allocated = 0
                    break
                grandtotal = grandtotal + total_allocated
        grandtotal = self.bytes_to_pages(grandtotal)
        return grandtotal

    def calculate_zram_dev_mem_allocated_MB(self, zram):
        mem_pool = zram + self.ramdump.field_offset('struct zram', 'mem_pool')
        mem_pool = self.ramdump.read_word(mem_pool)

        pages_allocated = mem_pool + self.ramdump.field_offset('struct zs_pool',
                                                              'pages_allocated')

        stat_val = self.ramdump.read_word(pages_allocated)
        if stat_val is None:
            stat_val = 0
        else:
            stat_val = self.pages_to_mb(stat_val)

        self.zram_mem_mb += stat_val

    def calculate_slab_mem_pages(self):
        # Returns memory in pages
        if self.ramdump.kernel_version >= (5, 10):
            slab_rec = self.ramdump.read_word(
                'vm_node_stat[NR_SLAB_RECLAIMABLE_B]')
            slab_unrec = self.ramdump.read_word(
                'vm_node_stat[NR_SLAB_UNRECLAIMABLE_B]')

        elif (self.ramdump.kernel_version >= (4, 14)):
            slab_rec = self.ramdump.read_word(
                'vm_node_stat[NR_SLAB_RECLAIMABLE]')
            slab_unrec = self.ramdump.read_word(
                'vm_node_stat[NR_SLAB_UNRECLAIMABLE]')

        elif self.ramdump.kernel_version >= (4, 9, 0):
            slab_rec = self.ramdump.read_word(
                    'vm_zone_stat[NR_SLAB_RECLAIMABLE]')
            slab_unrec = self.ramdump.read_word(
                    'vm_zone_stat[NR_SLAB_UNRECLAIMABLE]')

        else:
           slab_rec = \
               self.ramdump.read_word('vm_stat[NR_SLAB_RECLAIMABLE]')
           slab_unrec = \
               self.ramdump.read_word('vm_stat[NR_SLAB_UNRECLAIMABLE]')

        return (slab_rec, slab_unrec)

    def calculate_zram_compressed_pages(self):
        if self.ramdump.kernel_version >= (4, 4):
            if self.ramdump.kernel_version >= (4, 14):
                zram_index_idr = self.ramdump.address_of('zram_index_idr')
            else:
                zram_index_idr = self.ramdump.read_word('zram_index_idr')
            if zram_index_idr is None:
                stat_val = 0
            else:
                #'struct radix_tree_root' was replaced by 'struct xarray' on kernel 5.4+
                if self.ramdump.kernel_version >= (5, 4):
                    self.zram_dev_rtw.walk_radix_tree(zram_index_idr,
                                          self.calculate_zram_dev_mem_allocated_MB)
                    stat_val = self.zram_mem_mb
                else:
                    if self.ramdump.kernel_version >= (4, 14):
                        idr_layer_ary_offset = self.ramdump.field_offset(
                                    'struct radix_tree_root', 'rnode')
                        idr_layer_ary = self.ramdump.read_word(zram_index_idr +
                                    idr_layer_ary_offset)
                    else:
                        idr_layer_ary_offset = self.ramdump.field_offset(
                                    'struct idr_layer', 'ary')
                        idr_layer_ary = self.ramdump.read_word(zram_index_idr +
                                                           idr_layer_ary_offset)
                    try:
                        zram_meta = idr_layer_ary + self.ramdump.field_offset(
                                        'struct zram', 'meta')
                        zram_meta = self.ramdump.read_word(zram_meta)
                        mem_pool = zram_meta + self.ramdump.field_offset(
                                'struct zram_meta', 'mem_pool')
                        mem_pool = self.ramdump.read_word(mem_pool)
                    except TypeError:
                        mem_pool = idr_layer_ary + self.ramdump.field_offset(
                                    'struct zram', 'mem_pool')
                        mem_pool = self.ramdump.read_word(mem_pool)
                    if mem_pool is None:
                        stat_val = 0
                    else:
                        page_allocated = mem_pool + self.ramdump.field_offset(
                                        'struct zs_pool', 'pages_allocated')
                        stat_val = self.ramdump.read_word(page_allocated)
                        if stat_val is None:
                            stat_val = 0
        else:
            zram_devices_word = self.ramdump.read_word('zram_devices')
            if zram_devices_word is not None:
                zram_devices_stat_offset = self.ramdump.field_offset(
                                        'struct zram', 'stats')
                stat_addr = zram_devices_word + zram_devices_stat_offset
                stat_val = self.ramdump.read_u64(stat_addr)
                stat_val = self.bytes_to_pages(stat_val)
            else:
                stat_val = 0

        return stat_val

    def read_dma_heap_mem_pages(self):
        if self.ramdump.kernel_version >= (5, 10):
            try:
                dma_heap_file = os.path.join(self.ramdump.outdir, "total_dma_heap.txt")
                if os.path.isfile(dma_heap_file):
                    fin = open(dma_heap_file, 'r')
                    fin_list = fin.readlines()
                    fin.close()
                    ion_bytes = int(fin_list[0].split("Bytes")[0].split()[-1])
                    return self.bytes_to_pages(ion_bytes)
                return -1
            except Exception as e:
                print(e)
                return -2
        else:
            return 0

    def calculate_kgsl_mem_pages(self):
        # Returns memory in Pages
        # Duplicates gpuinfo_510.py@parse_kgsl_mem()'s 'KGSL Total'
        try:
            kgsl_memory = self.ramdump.read_word(
                            'kgsl_driver.stats.page_alloc')
            kgsl_memory += self.ramdump.read_word(
                            'kgsl_driver.stats.coherent')
            kgsl_memory += self.ramdump.read_word(
                            'kgsl_driver.stats.secure')
            kgsl_memory = self.bytes_to_pages(kgsl_memory)
            return kgsl_memory
        except TypeError as e:
            if self.out_mem_stat is not None:
                self.out_mem_stat.write("Failed to retrieve total kgsl memory\n")
            return 0

    def calculate_kernel_misc_rec_pages(self):
        if self.ramdump.kernel_version >= (6, 6):
            return max(self.ramdump.read_s64('vm_node_stat[NR_KERNEL_MISC_RECLAIMABLE]'), 0)
        else:
            return max(self.ramdump.read_s64('vm_node_stat[NR_KERNEL_MISC_RECLAIMABLE]'), 0)


    def print_mem_stats(self, out_mem_stat):
        self.out_mem_stat = out_mem_stat
        # Total memory
        total_mem_pages = self.calculate_totalmem_pages()
        total_mem = self.pages_to_mb(total_mem_pages)

        total_free_pages = self.calculate_totalfree_pages()
        total_free = self.pages_to_mb(total_free_pages)

        other_mem_pages = self.calculate_other_mem_pages()
        other_mem = self.pages_to_mb(other_mem_pages)

        cached_pages = self.calculate_cached_pages()
        cached = self.pages_to_mb(cached_pages)

        slab_rec_pages, slab_unrec_pages = self.calculate_slab_mem_pages()
        total_slab_pages = slab_rec_pages + slab_unrec_pages
        total_slab = self.pages_to_mb(total_slab_pages)

        # kgsl memory
        kgsl_memory_pages = self.calculate_kgsl_mem_pages()
        kgsl_memory = self.pages_to_mb(kgsl_memory_pages)

        # zcompressed ram
        stat_val_pages = self.calculate_zram_compressed_pages()
        zram_mb = self.pages_to_mb(stat_val_pages)

        # vmalloc area
        self.vmalloc_size = 0
        self.calculate_vmalloc_pages()

        # Output prints
        out_mem_stat.write('{0:30}: {1:8} MB'.format(
                                "Total RAM", total_mem))
        out_mem_stat.write('\n{0:30}: {1:8} MB\n'.format(
                            "Free memory:", total_free))
        out_mem_stat.write('\n{0:30}: {1:8} MB'.format(
                            "Total Slab memory:", total_slab))
        if self.ramdump.kernel_version >= (5, 10):
            ion_mem = self.read_dma_heap_mem_pages()
            if ion_mem > 0:
                ion_mem = self.pages_to_mb(ion_mem)
                out_mem_stat.write("\n{0:30}: {1:8} MB".format("Total DMA memory", ion_mem))
            elif ion_mem == -1:
                out_mem_stat.write("\n{0:30}: Please parse ionbuffer first, use --print-ionbuffer.".format(
                        "Total DMA memory"))
            else:
                out_mem_stat.write('\nTotal ion memory: Please refer total_dma_heap.txt')
        else:
            ion_mem_pages = self.calculate_ionmem_pages()
            ion_mem = self.pages_to_mb(ion_mem_pages)
            out_mem_stat.write('\n{0:30}: {1:8} MB'.format(
                            "Total ion memory:", ion_mem))
        out_mem_stat.write('\n{0:30}: {1:8} MB'.format(
                            "KGSL ", kgsl_memory))
        out_mem_stat.write('\n{0:30}: {1:8} MB'.format(
                            "ZRAM compressed  ", zram_mb))
        out_mem_stat.write('\n{0:30}: {1:8} MB'.format(
                            "vmalloc  ", self.vmalloc_size))
        out_mem_stat.write('\n{0:30}: {1:8} MB'.format(
                            "Others  ", other_mem))
        out_mem_stat.write('\n{0:30}: {1:8} MB'.format(
                            "Cached  ",cached))

        accounted_mem = total_free + total_slab + kgsl_memory + zram_mb + \
                        self.vmalloc_size + other_mem

        if ion_mem > 0:
            accounted_mem += ion_mem

        unaccounted_mem = total_mem - accounted_mem

        out_mem_stat.write('\n\n{0:30}: {1:8} MB'.format(
                            "Total Unaccounted Memory ",unaccounted_mem))

    def parse(self):
        with self.ramdump.open_file('mem_stat.txt') as out_mem_stat:
            if (self.ramdump.kernel_version < (3, 18, 0)):
                out_mem_stat.write('Kernel version 3.18 \
                and above are supported, current version {0}.\
                {1}'.format(self.ramdump.kernel_version[0],
                            self.ramdump.kernel_version[1]))
                return
            self.print_mem_stats(out_mem_stat)
