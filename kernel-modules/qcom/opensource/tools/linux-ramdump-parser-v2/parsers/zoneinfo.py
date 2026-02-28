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

zone_wmark_names = {"WMARK_MIN" : "min",
                    "WMARK_LOW" : "low",
                    "WMARK_HIGH" : "high"}

@register_parser('--print-proc_zoneinfo', 'Print /proc/zoneinfo stats ')
class ProcZoneinfo(RamParser):
    def __init__(self, dump):
        try:
            super(ProcZoneinfo, self).__init__(dump)
        except Exception as e:
            print("Exception in init", e)


    def print_zone_name(self, zone, zoneinfo_file):
        zone_name_offset = self.ramdump.field_offset('struct zone', 'name')
        zname_addr = self.ramdump.read_word(zone + zone_name_offset)
        zname = self.ramdump.read_cstring(zname_addr, 12)
        node = 0
        if self.ramdump.is_config_defined('CONFIG_NUMA'):
            node_addr = zone + self.ramdump.field_offset('struct zone', 'node')
            node = self.ramdump.read_word(node_addr)
        # Node 0, zone    DMA32
        print("Node {}, zone {:^9}".format(node, zname).strip(), file=zoneinfo_file)


    def get_present_pages(self, zone):
        present_pages_addr = zone + self.ramdump.field_offset('struct zone', 'present_pages')
        present_pages = self.ramdump.read_word(present_pages_addr)
        return present_pages

    def print_per_node_stats(self, zone, zoneinfo_file):
        #  per-node stats
        print("  per-node stats", file = zoneinfo_file)

        NR_VM_ZONE_STAT_ITEMS = self.ramdump.gdbmi.get_value_of('NR_VM_ZONE_STAT_ITEMS')
        NR_VM_NODE_STAT_ITEMS = self.ramdump.gdbmi.get_value_of('NR_VM_NODE_STAT_ITEMS')
        vmstat_text = self.ramdump.address_of('vmstat_text')
        vm_node_stats = self.ramdump.address_of('vm_node_stat')

        for i in range(NR_VM_NODE_STAT_ITEMS) :
            name_addr = self.ramdump.read_word(self.ramdump.array_index(vmstat_text, 'char *', i + NR_VM_ZONE_STAT_ITEMS))
            name = self.ramdump.read_cstring(name_addr, 32)
            #       nr_inactive_anon 18
            #       nr_active_anon 110935
            #       nr_inactive_file 139456
            #       nr_active_file 199705
            #       nr_unevictable 34593
            #       nr_slab_reclaimable 26474
            #       nr_slab_unreclaimable 61748
            #       nr_isolated_anon 0
            #       nr_isolated_file 0
            #       workingset_nodes 49
            #       workingset_refault_anon 73278
            #       workingset_refault_file 5235
            #       workingset_activate_anon 73278
            #       workingset_activate_file 5235
            #       workingset_restore_anon 73278
            #       workingset_restore_file 5091
            #       workingset_nodereclaim 0
            #       nr_anon_pages 101844
            #       nr_mapped    192858
            #       nr_file_pages 373828
            #       nr_dirty     25
            #       nr_writeback 0
            #       nr_writeback_temp 0
            #       nr_shmem     8106
            #       nr_shmem_hugepages 0
            #       nr_shmem_pmdmapped 0
            #       nr_file_hugepages 0
            #       nr_file_pmdmapped 0
            #       nr_anon_transparent_hugepages 16
            #       nr_vmscan_write 211043
            #       nr_vmscan_immediate_reclaim 0
            #       nr_dirtied   13748
            #       nr_written   240874
            #       nr_throttled_written 0
            #       nr_kernel_misc_reclaimable 98786
            #       nr_foll_pin_acquired 0
            #       nr_foll_pin_released 0
            #       nr_kernel_stack 49376
            #       nr_shadow_call_stack 12364
            #       nr_page_table_pages 27429
            #       nr_sec_page_table_pages 0
            #       nr_swapcached 75
            val = 0
            if self.ramdump.arm64:
                val = self.ramdump.read_s64(self.ramdump.array_index(vm_node_stats, 'atomic_long_t', i))
            else:
                val = self.ramdump.read_s32(self.ramdump.array_index(vm_node_stats, 'atomic_long_t', i))

            print("      {} {}".format(name, val), file = zoneinfo_file)

    def print_zone_stats(self, zone, zoneinfo_file):
        nr_watermark = self.ramdump.gdbmi.get_value_of('NR_WMARK')
        wmark_names = self.ramdump.gdbmi.get_enum_lookup_table(
            'zone_watermarks', nr_watermark)

        if (self.ramdump.kernel_version < (4, 19, 0)):
            zwatermark_addr = zone + \
                self.ramdump.field_offset('struct zone', 'watermark')
        else:
            zwatermark_addr = zone + \
                self.ramdump.field_offset('struct zone', '_watermark')

        present_pages_addr = zone + self.ramdump.field_offset('struct zone', 'present_pages')
        present_pages = self.ramdump.read_word(present_pages_addr)

        if present_pages == 0:
            #   pages free     0
            print("  pages free     0", file=zoneinfo_file)
            #         boost    0
            print("        boost    0", file=zoneinfo_file)
            #        min      0
            print("        min      0", file=zoneinfo_file)
            #        low      0
            print("        low      0", file=zoneinfo_file)
            #        high     0
            print("        high     0", file=zoneinfo_file)

            #         spanned  0
            print("        spanned  0", file=zoneinfo_file)
            #         present  0
            print("        present  0", file=zoneinfo_file)
            #         managed  0
            print("        managed  0", file=zoneinfo_file)
            #         cma      0
            print("        cma      0", file=zoneinfo_file)
            #         protection: (0, 0, 0, 0)
            num_zones = self.ramdump.gdbmi.get_value_of('__MAX_NR_ZONES')
            print("        protection: ({})".format(", ".join(['0'] * num_zones)), file=zoneinfo_file)
            return

        free_pages_addr = zone + self.ramdump.field_offset('struct zone', 'vm_stat')
        # first elment in zone->vm_stat[] is free.
        free_pages = self.ramdump.read_word(free_pages_addr)
        #   pages free     0
        print("  pages free     {}".format(free_pages), file=zoneinfo_file)

        boost_pages_addr = zone + self.ramdump.field_offset('struct zone', 'watermark_boost')
        boost_pages = self.ramdump.read_word(boost_pages_addr)
        #         boost    0
        print("        boost    {}".format(boost_pages), file=zoneinfo_file)

        spanned_pages_addr = zone + self.ramdump.field_offset('struct zone', 'spanned_pages')
        spanned_pages = self.ramdump.read_word(spanned_pages_addr)

        managed_pages_addr = zone + self.ramdump.field_offset('struct zone', 'managed_pages')
        managed_pages = self.ramdump.read_word(managed_pages_addr)

        cma_pages_offset = self.ramdump.field_offset('struct zone', 'cma_pages')
        cma_pages = 0
        if cma_pages_offset != None:
            cma_pages_addr = zone + cma_pages_offset
            cma_pages = self.ramdump.read_word(cma_pages_addr)

        for i in range(0, nr_watermark):
            val = self.ramdump.read_word(
                                    self.ramdump.array_index(zwatermark_addr,
                                        'unsigned long', i))
            #         min      0
            #         low      0
            #         high     0
            wmark_name = zone_wmark_names[wmark_names[i]] if wmark_names[i] in zone_wmark_names.keys() else wmark_names[i]
            print("        {:<8} {}".format(wmark_name, val), file=zoneinfo_file)

        #         spanned  0
        print("        spanned  {}".format(spanned_pages), file=zoneinfo_file)
        #         present  0
        print("        present  {}".format(present_pages), file=zoneinfo_file)
        #         managed  0
        print("        managed  {}".format(managed_pages), file=zoneinfo_file)
        #         cma      0
        print("        cma      {}".format(cma_pages), file=zoneinfo_file)
        #         protection: (0, 0, 0, 0)
        zprotection_addr = zone + \
                self.ramdump.field_offset('struct zone', 'lowmem_reserve')

        val = self.ramdump.read_word(self.ramdump.array_index(
                                        zprotection_addr, 'long', 0))
        print("        protection: ({}".format(val), file=zoneinfo_file, end="")
        for zi in range(1, self.ramdump.gdbmi.get_value_of('__MAX_NR_ZONES')):
            val = self.ramdump.read_word(self.ramdump.array_index(
                                  zprotection_addr, 'long', zi))
            print(", {}".format(val), file=zoneinfo_file, end="")
        print(")", file=zoneinfo_file)
        return


    def print_vm_zone_stats(self, zone, zoneinfo_file):
        NR_VM_ZONE_STAT_ITEMS = self.ramdump.gdbmi.get_value_of('NR_VM_ZONE_STAT_ITEMS')
        vmstat_text = self.ramdump.address_of('vmstat_text')
        start_addr = self.ramdump.read_word(vmstat_text)
        zstats_addr = zone + self.ramdump.field_offset('struct zone', 'vm_stat')
        #      nr_free_pages 604440
        #      nr_zone_inactive_anon 18
        #      nr_zone_active_anon 110935
        #      nr_zone_inactive_file 139456
        #      nr_zone_active_file 199705
        #      nr_zone_unevictable 34593
        #      nr_zone_write_pending 25
        #      nr_mlock     28000
        #      nr_bounce    0
        #      nr_zspages   30039
        #      nr_free_cma  0

        for i in range(NR_VM_ZONE_STAT_ITEMS):
            name_addr = self.ramdump.read_word(self.ramdump.array_index(vmstat_text, 'char *', i))
            name = self.ramdump.read_cstring(name_addr, 32)
            val = 0
            if self.ramdump.arm64:
                val = self.ramdump.read_s64(self.ramdump.array_index(zstats_addr, 'atomic_long_t', i))
            else:
                val = self.ramdump.read_s32(self.ramdump.array_index(zstats_addr, 'atomic_long_t', i))

            print("      {} {}".format(name, val), file=zoneinfo_file)
        #  start_pfn:           526593
        zone_start_pfn_addr = zone + self.ramdump.field_offset('struct zone', 'zone_start_pfn')
        zone_start_pfn = self.ramdump.read_word(zone_start_pfn_addr)
        print("  start_pfn:           {}".format(zone_start_pfn), file=zoneinfo_file)


    def parse(self):
        zoneinfo_file = self.ramdump.open_file("zoneinfo.txt", "w")
        max_nr_zones = self.ramdump.gdbmi.get_value_of('__MAX_NR_ZONES')

        contig_page_data = self.ramdump.address_of('contig_page_data')
        node_zones_offset = self.ramdump.field_offset(
            'struct pglist_data', 'node_zones')
        present_pages_offset = self.ramdump.field_offset(
            'struct zone', 'present_pages')
        sizeofzone = self.ramdump.sizeof('struct zone')
        zone = contig_page_data + node_zones_offset

        try:
            while zone < (contig_page_data + node_zones_offset + max_nr_zones * sizeofzone):
                present_pages = self.get_present_pages(zone)

                self.print_zone_name(zone, zoneinfo_file)
                if present_pages > 0:
                    self.print_per_node_stats(zone, zoneinfo_file)

                self.print_zone_stats(zone, zoneinfo_file)

                if present_pages > 0:
                    self.print_vm_zone_stats(zone, zoneinfo_file)

                zone = zone + sizeofzone

        except Exception as e:
            print(e)
        zoneinfo_file.close()

