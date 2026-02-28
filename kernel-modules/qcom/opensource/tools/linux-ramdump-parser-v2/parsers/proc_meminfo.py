# Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
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

@register_parser('--print-proc_meminfo', 'Print /proc/meminfo stats ')
class ProcMeminfo(RamParser):
    def __init__(self, dump):
        try:
            super(ProcMeminfo, self).__init__(dump)
        except Exception as e:
            print("Exception in init", e)

    def pages_to_kb(self, pages):
        val = 0
        if pages != None and pages != 0:
            val = (pages * self.ramdump.get_page_size()) >> 10
        return val

    def bytes_to_kb(self, bytes):
        val = 0
        if bytes != None and bytes != 0:
            val = (bytes >> 10)
        return val

    def getTotalMemKB(self):
        if(self.ramdump.kernel_version > (4, 20, 0)):
            total_mem = self.ramdump.read_word('_totalram_pages')
        else:
            total_mem = self.ramdump.read_word('totalram_pages')

        if total_mem == None:
            return -1

        return self.pages_to_kb(total_mem)

    def global_node_page_state (self, index, default = None):
        stat_pages = self.ramdump.read_word('vm_node_stat[{}]'.format(index))
        if stat_pages != None:
            # print("vm_node_stat[{}] = {}".format(index, stat_pages))
            return stat_pages

        # print("global_node_page_state_KB returning default {} for {}".format(default, index))
        return default

    def global_node_page_state_KB (self, index, default = None):
        stat_pages = self.global_node_page_state(index, default)
        if stat_pages != None:
            # print("vm_node_stat[{}] = {}".format(index, stat_pages))
            return self.pages_to_kb(stat_pages)

        # print("global_node_page_state_KB returning default {} for {}".format(default, index))
        return default

    def global_zone_page_state_KB(self, index, default = None):
        stat_pages = self.ramdump.read_word('vm_zone_stat[{}]'.format(index))

        if stat_pages == 0:
            default = 0

        if stat_pages != None and stat_pages > 0:
            # print("vm_zone_stat[{}] = {}".format(index, stat_pages))
            return self.pages_to_kb(stat_pages)

        return self.global_node_page_state_KB(index, default)

    def getVmStatKB(self, index, default = None):
        stat_pages = self.ramdump.read_word('vm_stat[{}]'.format(index))

        if stat_pages == 0:
            default = 0

        if stat_pages != None and stat_pages > 0:
            # print("vm_stat[{}] = {}".format(index, stat_pages))
            return self.pages_to_kb(stat_pages)

        return self.global_node_page_state_KB(index, default)

    def get_low_wmark(self):
        wmark_low = 0

        nr_watermark = self.ramdump.gdbmi.get_value_of('NR_WMARK')
        wmark_names = self.ramdump.gdbmi.get_enum_lookup_table(
            'zone_watermarks', nr_watermark)
        max_nr_zones = self.ramdump.gdbmi.get_value_of('__MAX_NR_ZONES')
        contig_page_data = self.ramdump.address_of('contig_page_data')
        node_zones_offset = self.ramdump.field_offset(
            'struct pglist_data', 'node_zones')
        sizeofzone = self.ramdump.sizeof('struct zone')
        zone = contig_page_data + node_zones_offset
        while zone < (contig_page_data + node_zones_offset + (max_nr_zones * sizeofzone)):
            if (self.ramdump.kernel_version < (4, 19, 0)):
                zwatermark_addr = zone + \
                    self.ramdump.field_offset('struct zone', 'watermark')
            else:
                zwatermark_addr = zone + \
                    self.ramdump.field_offset('struct zone', '_watermark')

            if "WMARK_LOW" in wmark_names:
                val = self.ramdump.read_word(
                                        self.ramdump.array_index(zwatermark_addr,
                                            'unsigned long', wmark_names.index("WMARK_LOW")))
                wmark_low += val
            zone = zone + sizeofzone

        return wmark_low

    def get_mem_mavailable(self):
        wmark_low = self.get_low_wmark()
        # wmark_low += low_wmark_pages(zone);
        available = self.global_zone_page_state_KB("NR_FREE_PAGES", default = 0)
        available -= self.ramdump.read_word('totalreserve_pages')

        pagecache = self.getVmStatKB("NR_ACTIVE_FILE",   default = 0)
        pagecache += self.getVmStatKB("NR_INACTIVE_FILE", default = 0)
        pagecache -= min(pagecache // 2, wmark_low)

        available += pagecache

        (slab_rec, slab_unrec) = self.get_slab_rec_unrec()
        reclaimable = slab_rec
        reclaimable += self.global_node_page_state_KB("NR_KERNEL_MISC_RECLAIMABLE", default = 0)
        reclaimable -= min(reclaimable // 2, wmark_low)

        available += reclaimable

        if (available < 0):
            return 0

        # print(available)
        return available

    def get_slab_rec_unrec(self):
        # for kernel >= 4.14: vm_node_stat[NR_SLAB_RECLAIMABLE] and vm_node_stat[NR_SLAB_UNRECLAIMABLE]
        slab_rec = self.global_node_page_state_KB("NR_SLAB_RECLAIMABLE")
        slab_unrec = self.global_node_page_state_KB("NR_SLAB_UNRECLAIMABLE")
        if slab_rec == None:
            slab_rec = self.global_node_page_state_KB("NR_SLAB_RECLAIMABLE_B")
            slab_unrec = self.global_node_page_state_KB("NR_SLAB_UNRECLAIMABLE_B")

        if slab_rec == None:
            slab_rec = self.global_zone_page_state_KB("NR_SLAB_RECLAIMABLE")
            slab_unrec = self.global_zone_page_state_KB("NR_SLAB_UNRECLAIMABLE")

        if slab_rec == None:
            slab_rec = slab_unrec = -1
        return (slab_rec, slab_unrec)

    def print_proc_meminfo(self, out_meminfo):
        try:
            # MemTotal:        5756748 kB
            print("MemTotal:        {:>8} kB".format(self.getTotalMemKB()), file = out_meminfo)
            # MemFree:         2463608 kB
            print("MemFree:         {:>8} kB".format(self.global_zone_page_state_KB("NR_FREE_PAGES", default = -1)), file = out_meminfo)
            # MemAvailable:    4166504 kB
            print("MemAvailable:    {:>8} kB".format(self.get_mem_mavailable()), file = out_meminfo)
            # Buffers:           11436 kB
            # TODO: Add buffers

            # Cached:          1482412 kB
            print("Cached:          {:>8} kB".format(self.getVmStatKB("NR_FILE_PAGES", default = -1)), file = out_meminfo)
            # SwapCached:          308 kB
            print("SwapCached:      {:>8} kB".format(self.global_node_page_state_KB("NR_SWAPCACHE", default = -1)), file = out_meminfo)

            # Active:          1200128 kB
            # Inactive:         556132 kB
            # Active(anon):     401800 kB
            # Inactive(anon):       80 kB
            # Active(file):     798328 kB
            # Inactive(file):   556052 kB
            vmstat_active_anon   = self.getVmStatKB("NR_ACTIVE_ANON",   default = -1)
            vmstat_inactive_anon = self.getVmStatKB("NR_INACTIVE_ANON", default = -1)
            vmstat_active_file   = self.getVmStatKB("NR_ACTIVE_FILE",   default = -1)
            vmstat_inactive_file = self.getVmStatKB("NR_INACTIVE_FILE", default = -1)
            print("Active:          {:>8} kB".format(vmstat_active_anon + vmstat_active_file), file = out_meminfo)
            print("Inactive:        {:>8} kB".format(vmstat_inactive_anon + vmstat_inactive_file), file = out_meminfo)
            print("Active(anon):    {:>8} kB".format(vmstat_active_anon), file = out_meminfo)
            print("Inactive(anon):  {:>8} kB".format(vmstat_inactive_anon), file = out_meminfo)
            print("Active(file):    {:>8} kB".format(vmstat_active_file), file = out_meminfo)
            print("Inactive(file):  {:>8} kB".format(vmstat_inactive_file), file = out_meminfo)

            # Unevictable:      138372 kB
            print("Unevictable:     {:>8} kB".format(self.getVmStatKB("NR_UNEVICTABLE", default = 0)), file = out_meminfo)
            # Mlocked:          112000 kB
            print("Mlocked:         {:>8} kB".format(self.global_zone_page_state_KB("NR_MLOCK", default = 0)), file = out_meminfo)
            # SwapTotal:       4194300 kB
            print("SwapTotal:       {:>8} kB".format(self.ramdump.read_word('total_swap_pages')), file = out_meminfo)
            # SwapFree:        3671036 kB
            print("SwapFree:        {:>8} kB".format(self.ramdump.read_word('nr_swap_pages')), file = out_meminfo)
            # Dirty:               304 kB
            print("Dirty:           {:>8} kB".format(self.global_node_page_state_KB("NR_FILE_DIRTY", default = 0)), file = out_meminfo)
            # Writeback:             0 kB
            print("Writeback:       {:>8} kB".format(self.global_zone_page_state_KB("NR_WRITEBACK", default = 0)), file = out_meminfo)
            # AnonPages:        364372 kB
            anon = self.global_node_page_state_KB("NR_ANON_MAPPED", default = vmstat_active_anon + vmstat_inactive_anon)
            print("AnonPages:       {:>8} kB".format(anon), file = out_meminfo)
            # Mapped:           771200 kB
            print("Mapped:          {:>8} kB".format(self.global_node_page_state_KB("NR_FILE_MAPPED", default = 0)), file = out_meminfo)
            # Shmem:             32348 kB
            print("Shmem:           {:>8} kB".format(self.getVmStatKB("NR_SHMEM", default = 0)), file = out_meminfo)

            # KReclaimable:     488212 kB
            # Slab:             339308 kB
            # SReclaimable:      93072 kB
            # SUnreclaim:       246236 kB
            (slab_rec, slab_unrec) = self.get_slab_rec_unrec()
            print("KReclaimable:    {:>8} kB".format(slab_rec + self.global_node_page_state_KB("NR_KERNEL_MISC_RECLAIMABLE", default = 0)), file = out_meminfo)
            print("Slab:            {:>8} kB".format(slab_rec + slab_unrec), file = out_meminfo)
            print("SReclaimable:    {:>8} kB".format(slab_rec), file = out_meminfo)
            print("SUnreclaim:      {:>8} kB".format(slab_unrec), file = out_meminfo)

            # KernelStack:       49408 kB
            print("KernelStack:     {:>8} kB".format(self.global_node_page_state("NR_KERNEL_STACK_KB", default = 0)), file = out_meminfo)

            if self.ramdump.is_config_defined('CONFIG_SHADOW_CALL_STACK'):
                # ShadowCallStack:   12384 kB
                print("ShadowCallStack: {:>8} kB".format(self.global_node_page_state("NR_KERNEL_SCS_KB", default = 0)), file = out_meminfo)

            # PageTables:       109156 kB
            print("PageTables:      {:>8} kB".format(self.global_node_page_state_KB("NR_PAGETABLE", default = 0)), file = out_meminfo)
            # SecPageTables:         0 kB
            print("SecPageTables:   {:>8} kB".format(self.global_node_page_state_KB("NR_SECONDARY_PAGETABLE", default = 0)), file = out_meminfo)
            # NFS_Unstable:          0 kB
            print("NFS_Unstable:    {:>8} kB".format(0), file = out_meminfo)
            # Bounce:                0 kB
            print("Bounce:          {:>8} kB".format(self.global_zone_page_state_KB("NR_BOUNCE", default = 0)), file = out_meminfo)
            # WritebackTmp:          0 kB
            print("WritebackTmp:    {:>8} kB".format(self.global_node_page_state_KB("NR_WRITEBACK_TEMP", default = 0)), file = out_meminfo)

            # CommitLimit:     7072672 kB
            # Committed_AS:   58260644 kB
            # VmallocTotal:   259653632 kB
            # VmallocUsed:      164704 kB
            # VmallocChunk:          0 kB
            print("VmallocChunk:    {:>8} kB".format(0), file = out_meminfo)

            # Percpu:             9280 kB
            if self.ramdump.is_config_defined('CONFIG_TRANSPARENT_HUGEPAGE'):
                # AnonHugePages:     32768 kB
                # ShmemHugePages:        0 kB
                # ShmemPmdMapped:        0 kB
                # FileHugePages:         0 kB
                # FilePmdMapped:         0 kB
                print("AnonHugePages:   {:>8} kB".format(self.global_node_page_state_KB("NR_ANON_THPS", default = 0)), file = out_meminfo)
                print("ShmemHugePages:  {:>8} kB".format(self.global_node_page_state_KB("NR_SHMEM_THPS", default = 0)), file = out_meminfo)
                print("ShmemPmdMapped:  {:>8} kB".format(self.global_node_page_state_KB("NR_SHMEM_PMDMAPPED", default = 0)), file = out_meminfo)
                print("FileHugePages:   {:>8} kB".format(self.global_node_page_state_KB("NR_FILE_THPS", default = 0)), file = out_meminfo)
                print("FilePmdMapped:   {:>8} kB".format(self.global_node_page_state_KB("NR_FILE_PMDMAPPED", default = 0)), file = out_meminfo)

            if self.ramdump.is_config_defined('CONFIG_CMA'):
                # CmaTotal:         188416 kB
                print("CmaTotal:        {:>8} kB".format(self.ramdump.read_word('totalcma_pages')), file = out_meminfo)
                # CmaFree:               0 kB
                print("CmaFree:         {:>8} kB".format(self.global_zone_page_state_KB("NR_FREE_CMA_PAGES", default = 0)), file = out_meminfo)

        except Exception as e:
            print("Exception in print_proc_meminfo", e)
            raise e

    def parse(self):
        with self.ramdump.open_file('meminfo.txt') as out_meminfo:
            self.print_proc_meminfo(out_meminfo)
