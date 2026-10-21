# Copyright (c) 2012-2015, 2020 The Linux Foundation. All rights reserved.
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
import linux_list as llist

@register_parser('--print-pagetypeinfo', 'Print the pagetypeinfo')
class Pagetypeinfo(RamParser):

    def pages_to_mb(self, nr_pages):
        return (nr_pages >> 8)

    def print_pagetype_info_per_zone(self, ramdump, zone, nr_migrate_types):

        free_area_offset = ramdump.field_offset('struct zone', 'free_area')
        free_area_size = ramdump.sizeof('struct free_area')
        free_list_offset = ramdump.field_offset(
            'struct free_area', 'free_list')
        list_head_size = ramdump.sizeof('struct list_head')
        migratetype_names = ramdump.address_of('migratetype_names')
        zone_name_offset = ramdump.field_offset('struct zone', 'name')
        zname_addr = ramdump.read_word(zone + zone_name_offset)
        zname = ramdump.read_cstring(zname_addr, 12)
        total_pages = 0
        total_orders = [0]*11
        prefix = "%12s"

        self.f.write("Zone %s\n" % (zname))
        self.f.write(prefix % ("order"))
        for order in range(0, 11):
            self.f.write("%8d" % (order))
        self.f.write('\n')

        self.f.write(prefix % ("type"))
        for order in range(0, 11):
            self.f.write("%6d%2s" % (1 << (order + 2), "KB"))
        self.f.write('\n')

        for mtype in range(0, nr_migrate_types):
            total_migratetype_pages = 0
            mname_addr = ramdump.read_word(ramdump.array_index(migratetype_names, 'char *', mtype))
            mname = ramdump.read_cstring(mname_addr, 12)
            self.f.write(prefix % (mname))

            for order in range(0, 11):
                area = zone + free_area_offset + order * free_area_size
                free_list = area + free_list_offset + list_head_size * mtype

                it = llist.ListWalker(ramdump, free_list, 0)
                nr = 0
                for i in it:
                    nr += 1

                self.f.write('%8s' % (nr))
                total_orders[order] += nr
                total_migratetype_pages += (nr << order)
                total_pages += (nr << order)

            self.f.write(' = %s MB %s pages\n' %
                (self.pages_to_mb(total_migratetype_pages),
                total_migratetype_pages))

        self.f.write(prefix % ("Total"))
        for order in range(0, 11):
            self.f.write('%8s' % (total_orders[order]))
        self.f.write('\n')

        self.f.write('Approximate total for zone {0}: {1} MB, {2} pages\n'.format(
            zname, self.pages_to_mb(total_pages), total_pages))

    def parse(self):
        self.f = open(self.ramdump.outdir + "/pagetypeinfo.txt", "w")
        max_nr_zones = self.ramdump.gdbmi.get_value_of('__MAX_NR_ZONES')
        nr_migrate_types = self.ramdump.gdbmi.get_value_of('MIGRATE_TYPES')

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
                self.print_pagetype_info_per_zone(
                    self.ramdump, zone, nr_migrate_types)

            zone = zone + sizeofzone

        self.f.close()
