# Copyright (c) 2013-2015, 2017, 2019-2020 The Linux Foundation. All rights reserved.
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
from parser_util import register_parser, RamParser


@register_parser('--print-vmstats', 'Print the information similar to /proc/zoneinfo and /proc/vmstat')
class ZoneInfo(RamParser):

    def print_atomic_long_counters(self, output_file, counter_name, addr, num):

        for i in range(0, num):
            val = 0
            if self.ramdump.arm64:
                val = self.ramdump.read_s64(self.ramdump.array_index(addr,
                    'atomic_long_t', i))

            else:
                val = self.ramdump.read_s32(self.ramdump.array_index(addr,
                    'atomic_long_t', i))
            output_file.write(('{0:30}: {1:10} {2:10}MB\n'.format(counter_name[i], val, val * 4 // 1024)))

    def print_zone_stats(self, zone, vmstat_names, max_zone_stats, output_file):
        nr_watermark = self.ramdump.gdbmi.get_value_of('NR_WMARK')
        wmark_names = self.ramdump.gdbmi.get_enum_lookup_table(
            'zone_watermarks', nr_watermark)

        zone_name_offset = self.ramdump.field_offset('struct zone', 'name')
        zname_addr = self.ramdump.read_word(zone + zone_name_offset)
        zname = self.ramdump.read_cstring(zname_addr, 12)

        zstats_addr = zone + \
            self.ramdump.field_offset('struct zone', 'vm_stat')
        if (self.ramdump.kernel_version < (4, 19, 0)):
            zwatermark_addr = zone + \
                self.ramdump.field_offset('struct zone', 'watermark')
        else:
            zwatermark_addr = zone + \
                self.ramdump.field_offset('struct zone', '_watermark')

        output_file.write('\nZone {0:8}\n'.format(zname))
        for i in range(0, nr_watermark):
            val = self.ramdump.read_word(
                                    self.ramdump.array_index(zwatermark_addr,
                                        'unsigned long', i))
            output_file.write('{0:30}: {1:10} {2:10}MB\n'.format(wmark_names[i],
                val, val * 4 // 1024))

        self.print_atomic_long_counters(output_file, vmstat_names, zstats_addr,
                max_zone_stats)

    def print_vm_node_states(self, output_file):
        vm_node_stats = self.ramdump.address_of('vm_node_stat')
        max_node_stat_item = self.ramdump.gdbmi.get_value_of('NR_VM_NODE_STAT_ITEMS')
        vmstat_names = self.ramdump.gdbmi.get_enum_lookup_table('node_stat_item', max_node_stat_item)
        self.print_atomic_long_counters(output_file, vmstat_names, vm_node_stats,
                max_node_stat_item)

    def print_vm_event_states(self, output_file):
        vm_event_states = self.ramdump.address_of('vm_event_states')
        max_node_stat_item = self.ramdump.gdbmi.get_value_of('NR_VM_EVENT_ITEMS')
        vmstat_names = self.ramdump.gdbmi.get_enum_lookup_table(
                  'vm_event_item', max_node_stat_item)
        rq_addr=[0,0,0,0,0,0,0,0]
        for i in range(0, max_node_stat_item):
            val = 0
            for j in self.ramdump.iter_cpus():
                rq_addr[j] = vm_event_states + self.ramdump.per_cpu_offset(j)
                if self.ramdump.arm64:
                    val += self.ramdump.read_word(self.ramdump.array_index(rq_addr[j],
                        'unsigned long', i))
                else:
                    val += self.ramdump.read_word(self.ramdump.array_index(rq_addr[j],
                        'unsigned long', i))

            if self.ramdump.arm64:
                output_file.write('{0:30}: {1:10} {2:10}MB\n'.format(vmstat_names[i],
                    val, val * 4 // 1024))
            else:
                output_file.write('{0:30}: {1:10} {2:10}MB\n'.format(vmstat_names[i],
                    val, val * 4 // 1024))

    def parse(self):
        output_file = self.ramdump.open_file("vmstats.txt", "w")
        max_zone_stats = self.ramdump.gdbmi.get_value_of(
            'NR_VM_ZONE_STAT_ITEMS')
        vmstat_names = self.ramdump.gdbmi.get_enum_lookup_table(
            'zone_stat_item', max_zone_stats)
        max_nr_zones = self.ramdump.gdbmi.get_value_of('__MAX_NR_ZONES')

        contig_page_data = self.ramdump.address_of('contig_page_data')
        node_zones_offset = self.ramdump.field_offset(
            'struct pglist_data', 'node_zones')
        present_pages_offset = self.ramdump.field_offset(
            'struct zone', 'present_pages')
        sizeofzone = self.ramdump.sizeof('struct zone')
        zone = contig_page_data + node_zones_offset

        while zone < (contig_page_data + node_zones_offset + max_nr_zones * sizeofzone):
            present_pages = self.ramdump.read_word(zone + present_pages_offset)
            if not not present_pages:
                self.print_zone_stats(zone, vmstat_names, max_zone_stats,
                        output_file)

            zone = zone + sizeofzone

        output_file.write("\nGlobal Stats\n")
        if self.ramdump.kernel_version < (4,9,0):
            vmstats_addr = self.ramdump.address_of('vm_stat')
        else:
            vmstats_addr = self.ramdump.address_of('vm_zone_stat')
            
        self.print_atomic_long_counters(output_file, vmstat_names,
                vmstats_addr,
                max_zone_stats)

        output_file.write('\nNode Stats\n')
        self.print_vm_node_states(output_file)

        output_file.write('\nVM EVENT Stats\n')
        self.print_vm_event_states(output_file)

        output_file.write('Total system pages:{0}\n'.format(self.ramdump.read_word(self.ramdump.address_of('totalram_pages'))))
        output_file.close()
