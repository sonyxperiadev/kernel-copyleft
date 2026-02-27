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

import struct

from parser_util import register_parser, RamParser
from print_out import print_out_str
from qdss import QDSSDump
from parsers.cachedump import save_l1_dump, parse_cache_dump
from parsers.watchdog import TZRegDump
from debug_image_v2 import DebugImage_v2

QDSS_MAGIC = 0x5D1DB1Bf

print_table = {
    'MSM_CPU_CTXT': 'parse_cpu_ctx',
    'MSM_L1_CACHE': 'parse_l1_cache',
    'MSM_L2_CACHE': 'parse_l2_cache',
    'MSM_OCMEM': 'parse_ocmem',
    'MSM_TMC0_REG': 'parse_qdss_common',
    'MSM_TMC_ETFETB': 'parse_qdss_common',
    'MSM_TMC1_REG': 'parse_qdss_common',
    'MSM_ETM0_REG': 'parse_qdss_common',
    'MSM_ETM1_REG': 'parse_qdss_common',
    'MSM_ETM2_REG': 'parse_qdss_common',
    'MSM_ETM3_REG': 'parse_qdss_common',
}

tag_to_field_name = {
    'MSM_TMC0_REG': 'tmc_etr_start',
    'MSM_TMC_ETFETB': 'etf_start',
    'MSM_TMC1_REG': 'tmc_etf_start',
    'MSM_ETM0_REG': 'etm_regs0',
    'MSM_ETM1_REG': 'etm_regs1',
    'MSM_ETM2_REG': 'etm_regs2',
    'MSM_ETM3_REG': 'etm_regs3',
}


@register_parser('--parse-debug-image', 'Parse the debug image and associated information')
class DebugImage(RamParser):

    def __init__(self, *args):
        super(DebugImage, self).__init__(*args)
        self.qdss = QDSSDump()
        self.name_lookup_table = []

    def parse_cpu_ctx(self, start, end, tag):
        print_out_str(
            'Parsing CPU context start {0:x} end {1:x}'.format(start, end))
        # For historical reasons, we can't rely on the magic number to indicate if there
        # is context dumped. Check the magic number here instead
        magic = self.ramdump.read_word(start, False)
        if magic is None:
            print_out_str(
                "!!! Address {0:x} is bogus! Can't parse!".format(start))
            return

        if magic != 0x44434151:
            print_out_str(
                "!!! Magic {0:x} doesn't match! No context was dumped!".format(magic))
            return

        regs = TZRegDump(self.ramdump)
        regs.init_regs(start)
        for i in range(regs.ncores):
            regs.dump_core_pc(i)
        regs.dump_all_regs()

    def parse_l2_cache(self, start, end, tag):
        print_out_str(
            'Parsing L2 cache context start {0:x} end {1:x}'.format(start, end))
        magic = self.ramdump.read_word(start, False)
        if magic is None:
            print_out_str(
                "!!! Address {0:x} is bogus! Can't parse!".format(start))
            return

        if magic != 0xcac1ecac:
            print_out_str(
                "!!! Magic {0:x} doesn't match! No cache was dumped!".format(magic))
            return

        parse_cache_dump(self.ramdump, start)

    def parse_l1_cache(self, start, end, tag):
        print_out_str(
            'Parsing L1 cache context start {0:x} end {1:x}'.format(start, end))
        magic = self.ramdump.read_word(start, False)
        if magic is None:
            print_out_str(
                "!!! Address {0:x} is bogus! Can't parse!".format(start))
            return

        if magic != 0x314C4151:
            print_out_str(
                "!!! Magic {0:X} doesn't match! No cache was dumped!".format(magic))
            return
        print_out_str('Saving L1 cache')
        save_l1_dump(self.ramdump, start, end - start)

    def parse_ocmem(self, start, end, tag):
        print_out_str(
            '[!!!] Parsing not implemented yet start {0:x} end {1:x}'.format(start, end))

    def parse_qdss_common(self, start, end, tag):
        print_out_str(
            'Parsing {0} context start {1:x} end {2:x}'.format(tag, start, end))
        magic = self.ramdump.read_word(start, False)
        if magic is None:
            print_out_str(
                "!!! Address {0:x} is bogus! Can't parse!".format(start))
            return

        if magic != QDSS_MAGIC:
            print_out_str(
                "!!! Magic {0:X} doesn't match! Tracing was not dumped!".format(magic))
            return

        setattr(self.qdss, tag_to_field_name[tag], start + 4096)

    def parse_dump(self):
        out_dir = self.ramdump.outdir
        self.name_lookup_table = self.ramdump.gdbmi.get_enum_lookup_table(
            'dump_client_type', 32)
        dump_table_ptr_offset = self.ramdump.field_offset(
            'struct msm_memory_dump', 'dump_table_ptr')
        version_offset = self.ramdump.field_offset(
            'struct msm_dump_table', 'version')
        num_entries_offset = self.ramdump.field_offset(
            'struct msm_dump_table', 'num_entries')
        client_entries_offset = self.ramdump.field_offset(
            'struct msm_dump_table', 'client_entries')
        id_offset = self.ramdump.field_offset('struct msm_client_dump', 'id')
        start_addr_offset = self.ramdump.field_offset(
            'struct msm_client_dump', 'start_addr')
        end_addr_offset = self.ramdump.field_offset(
            'struct msm_client_dump', 'end_addr')
        client_dump_entry_size = self.ramdump.sizeof('struct msm_client_dump')

        mem_dump_data = self.ramdump.address_of('mem_dump_data')

        dump_table = self.ramdump.read_word(
            mem_dump_data + dump_table_ptr_offset)

        version = self.ramdump.read_word(dump_table + version_offset)
        if version is None:
            print_out_str('Version is bogus! Can\'t parse debug image')
            return
        num_entries = self.ramdump.read_word(dump_table + num_entries_offset)
        if num_entries is None or num_entries > 100:
            print_out_str('num_entries is bogus! Can\'t parse debug image')
            return

        print_out_str('\nDebug image version: {0}.{1} Number of entries {2}'.format(
            version >> 20, version & 0xFFFFF, num_entries))
        print_out_str('--------')

        for i in range(0, num_entries):
            this_client = dump_table + client_entries_offset + \
                i * client_dump_entry_size
            client_id = self.ramdump.read_word(this_client + id_offset)
            client_start = self.ramdump.read_word(
                this_client + start_addr_offset)
            client_end = self.ramdump.read_word(this_client + end_addr_offset)

            if client_id < 0 or client_id > len(self.name_lookup_table):
                print_out_str(
                    '!!! Invalid client id found {0:x}'.format(client_id))
                continue

            client_name = self.name_lookup_table[client_id]

            if client_name not in print_table:
                print_out_str(
                    '!!! {0} Does not have an associated function. The parser needs to be updated!'.format(client_name))
            else:
                print_out_str(
                    'Parsing debug information for {0}'.format(client_name))
                func = print_table[client_name]
                getattr(DebugImage, func)(self, client_start,
                                          client_end, client_name)
            print_out_str('--------')

        self.qdss.dump_standard(self.ramdump)
        if not self.ramdump.skip_qdss_bin:
            self.qdss.save_etf_bin(self.ramdump)
            self.qdss.save_etr_bin(self.ramdump)

    def parse(self):
        # use the mem_dump_data variable to detect if debug image feature was compiled in,
        # and memdump data variable for debug image v2 feature, rather than relying on
        # configuration option.
        if self.ramdump.address_of('mem_dump_data'):
            self.parse_dump()
        elif self.ramdump.address_of('memdump'):
            regs = DebugImage_v2(self.ramdump)
            regs.parse_dump_v2(self.ramdump)
        else:
            print_out_str(
                '!!! Debug image was not enabled. No debug dump will be provided')
            return

