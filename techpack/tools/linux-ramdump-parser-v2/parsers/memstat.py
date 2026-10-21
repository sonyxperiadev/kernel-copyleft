# Copyright (c) 2016-2021 The Linux Foundation. All rights reserved.
# Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
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
import linux_list as llist
import linux_radix_tree

VM_ALLOC = 0x00000002


@register_parser('--print-memstat', 'Print memory stats ')
class MemStats(RamParser):
    def __init__(self, dump):
        super(MemStats, self).__init__(dump)

        if (self.ramdump.kernel_version >= (5, 4)):
            self.zram_dev_rtw = linux_radix_tree.RadixTreeWalker(self.ramdump)
            self.zram_mem_mb = 0

    def list_func(self, vmlist):
        vm = self.ramdump.read_word(vmlist + self.vm_offset)
        if vm is None:
            return

        pages = self.ramdump.read_structure_field(
                vm, 'struct vm_struct', 'nr_pages')
        vm_flags = self.ramdump.read_structure_field(
                    vm, 'struct vm_struct', 'flags')
        if vm_flags is None:
            return
        if (vm_flags & VM_ALLOC == VM_ALLOC):
            self.vmalloc_size = self.vmalloc_size + pages

    def pages_to_mb(self, pages):
        val = 0
        if pages != 0:
            val = ((pages * 4) // 1024)
        return val

    def bytes_to_mb(self, bytes):
        val = 0
        if bytes != 0:
            val = (bytes // 1024) // 1024
        return val

    def pages_to_mb(self, pages):
        val = 0
        if pages != 0:
            val = (pages * 4) // 1024
        return val

    def calculate_vmalloc(self):
        if self.ramdump.address_of('nr_vmalloc_pages') is None:
            next_offset = self.ramdump.field_offset('struct vmap_area', 'list')
            vmlist = self.ramdump.read_word('vmap_area_list')
            vm_offset = self.ramdump.field_offset('struct vmap_area', 'vm')
            self.vm_offset = vm_offset
            list_walker = llist.ListWalker(self.ramdump, vmlist, next_offset)
            list_walker.walk(vmlist, self.list_func)
            self.vmalloc_size = self.pages_to_mb(self.vmalloc_size)
        else:
            val = self.ramdump.read_word('nr_vmalloc_pages')
            self.vmalloc_size = self.pages_to_mb(val)

    def calculate_vm_stat(self):
        # Other memory :  NR_ANON_PAGES + NR_FILE_PAGES + NR_PAGETABLE \
        # + NR_KERNEL_STACK - NR_SWAPCACHE
        vmstat_anon_pages = self.ramdump.read_word(
                            'vm_stat[NR_ANON_PAGES]')
        vmstat_file_pages = self.ramdump.read_word(
                            'vm_stat[NR_FILE_PAGES]')
        vmstat_pagetbl = self.ramdump.read_word(
                                'vm_stat[NR_PAGETABLE]')
        vmstat_kernelstack = self.ramdump.read_word(
                                'vm_stat[NR_KERNEL_STACK]')
        vmstat_swapcache = self.ramdump.read_word(
                            'vm_stat[NR_SWAPCACHE]')
        other_mem = (vmstat_anon_pages + vmstat_file_pages + vmstat_pagetbl +
                     vmstat_kernelstack - vmstat_swapcache)
        other_mem = self.pages_to_mb(other_mem)
        return other_mem

    def calculate_cached(self):
        if self.ramdump.kernel_version >= (4, 9):
            vmstat_file_pages = self.ramdump.read_word(
                            'vm_node_stat[NR_FILE_PAGES]')
            cached = self.pages_to_mb(vmstat_file_pages)
        else:
            vmstat_file_pages = self.ramdump.read_word(
                            'vm_stat[NR_FILE_PAGES]')
            cached = self.pages_to_mb(vmstat_file_pages)
        return cached

    def calculate_vm_node_zone_stat(self):
        # Other memory :  NR_ANON_MAPPED + NR_FILE_PAGES + NR_PAGETABLE \
        # + NR_KERNEL_STACK_KB
        vmstat_anon_pages = self.ramdump.read_word(
                            'vm_node_stat[NR_ANON_MAPPED]')
        vmstat_file_pages = self.ramdump.read_word(
                            'vm_node_stat[NR_FILE_PAGES]')
        if self.ramdump.kernel_version >= (5, 15):
            vmstat_pagetbl = self.ramdump.read_word(
                                'vm_node_stat[NR_PAGETABLE]')
            vmstat_kernelstack = self.ramdump.read_word(
                                'vm_node_stat[NR_KERNEL_STACK_KB]')
        else:
            vmstat_pagetbl = self.ramdump.read_word(
                                'vm_zone_stat[NR_PAGETABLE]')
            vmstat_kernelstack = self.ramdump.read_word(
                                'vm_zone_stat[NR_KERNEL_STACK_KB]')
        other_mem = (vmstat_anon_pages + vmstat_file_pages + vmstat_pagetbl +
                     (vmstat_kernelstack // 4))
        other_mem = self.pages_to_mb(other_mem)
        return other_mem

    def calculate_ionmem(self):
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
        grandtotal = self.bytes_to_mb(grandtotal)
        return grandtotal

    def calculate_zram_dev_mem_allocated(self, zram):
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

    def print_mem_stats(self, out_mem_stat):
        # Total memory
        if(self.ramdump.kernel_version > (4, 20, 0)):
            total_mem = self.ramdump.read_word('_totalram_pages')
        else:
            total_mem = self.ramdump.read_word('totalram_pages')

        total_mem = self.pages_to_mb(total_mem)

        if (self.ramdump.kernel_version < (4, 9, 0)):
           # Free Memory
           total_free = self.ramdump.read_word('vm_stat[NR_FREE_PAGES]')
           total_free = self.pages_to_mb(total_free)

           # slab Memory
           slab_rec = \
               self.ramdump.read_word('vm_stat[NR_SLAB_RECLAIMABLE]')
           slab_unrec = \
               self.ramdump.read_word('vm_stat[NR_SLAB_UNRECLAIMABLE]')
           total_slab = self.pages_to_mb(slab_rec + slab_unrec)

           #others
           other_mem = self.calculate_vm_stat()
        else:
            # Free Memory
            total_free = self.ramdump.read_word('vm_zone_stat[NR_FREE_PAGES]')
            total_free = self.pages_to_mb(total_free)

            # slab Memory
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
            else:
                slab_rec = self.ramdump.read_word(
                        'vm_zone_stat[NR_SLAB_RECLAIMABLE]')
                slab_unrec = self.ramdump.read_word(
                        'vm_zone_stat[NR_SLAB_UNRECLAIMABLE]')
            total_slab = self.pages_to_mb(slab_rec + slab_unrec)
            # others
            other_mem = self.calculate_vm_node_zone_stat()
        cached = self.calculate_cached()

        # ion memory
        ion_mem = self.calculate_ionmem()

        # kgsl memory
        kgsl_memory = self.ramdump.read_word(
                        'kgsl_driver.stats.page_alloc')
        if kgsl_memory is not None:
            kgsl_memory = self.bytes_to_mb(kgsl_memory)
        else:
            kgsl_memory = 0

        # zcompressed ram
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
                                          self.calculate_zram_dev_mem_allocated)
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
                        stat_val = self.pages_to_mb(stat_val)
        else:
            zram_devices_word = self.ramdump.read_word('zram_devices')
            if zram_devices_word is not None:
                zram_devices_stat_offset = self.ramdump.field_offset(
                                        'struct zram', 'stats')
                stat_addr = zram_devices_word + zram_devices_stat_offset
                stat_val = self.ramdump.read_u64(stat_addr)
                stat_val = self.bytes_to_mb(stat_val)
            else:
                stat_val = 0

        self.out_mem_stat = out_mem_stat
        self.vmalloc_size = 0
        # vmalloc area
        self.calculate_vmalloc()


        # Output prints
        out_mem_stat.write('{0:30}: {1:8} MB'.format(
                                "Total RAM", total_mem))
        out_mem_stat.write('\n{0:30}: {1:8} MB\n'.format(
                            "Free memory:", total_free))
        out_mem_stat.write('\n{0:30}: {1:8} MB'.format(
                            "Total Slab memory:", total_slab))
        if self.ramdump.kernel_version >= (5, 10):
            log_location = os.path.dirname(out_mem_stat.name)
            try:
                dma_heap_file = os.path.join(log_location, "total_dma_heap.txt")
                if os.path.isfile(dma_heap_file):
                    fin = open(dma_heap_file, 'r')
                    fin_list = fin.readlines()
                    fin.close()
                    ion_mem = int(fin_list[0].split(" ")[-1].replace("MB", ""))
                    out_mem_stat.write("\n{0:30}: {1:8} MB".format("Total DMA memory", ion_mem))
                else:
                    out_mem_stat.write("\n{0:30}: Please parse ionbuffer first, use --print-ionbuffer.".format(
                            "Total DMA memory"))
            except:
                ion_mem = "Please refer total_dma_heap.txt"
                out_mem_stat.write('\nTotal ion memory: Please refer total_dma_heap.txt')
        else:
            out_mem_stat.write('\n{0:30}: {1:8} MB'.format(
                            "Total ion memory:", ion_mem))
        out_mem_stat.write('\n{0:30}: {1:8} MB'.format(
                            "KGSL ", kgsl_memory))
        out_mem_stat.write('\n{0:30}: {1:8} MB'.format(
                            "ZRAM compressed  ", stat_val))
        out_mem_stat.write('\n{0:30}: {1:8} MB'.format(
                            "vmalloc  ", self.vmalloc_size))
        out_mem_stat.write('\n{0:30}: {1:8} MB'.format(
                            "Others  ", other_mem))
        out_mem_stat.write('\n{0:30}: {1:8} MB'.format(
                            "Cached  ",cached))

        if type(ion_mem) is str:
            accounted_mem = total_free + total_slab + kgsl_memory + stat_val + \
                            self.vmalloc_size + other_mem
        else:
            accounted_mem = total_free + total_slab + ion_mem + kgsl_memory + \
                        stat_val + self.vmalloc_size + other_mem

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
