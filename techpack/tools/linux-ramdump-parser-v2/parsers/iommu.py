# Copyright (c) 2013-2016, 2020 The Linux Foundation. All rights reserved.
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

from print_out import print_out_str
from parser_util import register_parser, RamParser
from sizes import SZ_4K, SZ_64K, SZ_1M, SZ_16M, get_order, order_size_strings
from iommulib import IommuLib, MSM_SMMU_DOMAIN, MSM_SMMU_AARCH64_DOMAIN, ARM_SMMU_DOMAIN
from lpaeiommulib import parse_long_form_tables
from aarch64iommulib import parse_aarch64_tables


@register_parser('--print-iommu-pg-tables', 'Print IOMMU page tables')
class IOMMU(RamParser):

    class FlatMapping(object):

        def __init__(self, virt, phys=-1, type='[]', size=SZ_4K, mapped=False):
            self.virt = virt
            self.phys = phys
            self.mapping_type = type
            self.mapping_size = size
            self.mapped = mapped

    class CollapsedMapping(object):

        def __init__(self, virt_start, virt_end, phys_start=-1, phys_end=-1, type='[]', size=SZ_4K, mapped=False):
            self.virt_start = virt_start
            self.virt_end = virt_end - 1
            self.phys_start = phys_start
            self.phys_end = phys_end - 1
            self.mapping_type = type
            self.mapping_size = size
            self.mapped = mapped

        def phys_size(self):
            return (self.phys_end - self.phys_start + 1)

        def virt_size(self):
            return (self.virt_end - self.virt_start + 1)

    def __init__(self, *args):
        super(IOMMU, self).__init__(*args)
        self.out_file = None
        self.NUM_FL_PTE = 4096
        self.NUM_SL_PTE = 256

        self.FL_BASE_MASK = 0xFFFFFC00
        self.FL_TYPE_TABLE = (1 << 0)
        self.FL_TYPE_SECT = (1 << 1)
        self.FL_SUPERSECTION = (1 << 18)
        self.FL_AP0 = (1 << 10)
        self.FL_AP1 = (1 << 11)
        self.FL_AP2 = (1 << 15)
        self.FL_SHARED = (1 << 16)
        self.FL_BUFFERABLE = (1 << 2)
        self.FL_CACHEABLE = (1 << 3)
        self.FL_TEX0 = (1 << 12)
        self.FL_NG = (1 << 17)

        self.SL_BASE_MASK_LARGE = 0xFFFF0000
        self.SL_BASE_MASK_SMALL = 0xFFFFF000
        self.SL_TYPE_LARGE = (1 << 0)
        self.SL_TYPE_SMALL = (2 << 0)
        self.SL_AP0 = (1 << 4)
        self.SL_AP1 = (2 << 4)
        self.SL_AP2 = (1 << 9)
        self.SL_SHARED = (1 << 10)
        self.SL_BUFFERABLE = (1 << 2)
        self.SL_CACHEABLE = (1 << 3)
        self.SL_TEX0 = (1 << 6)
        self.SL_NG = (1 << 11)

        self.node_offset = self.ramdump.field_offset(
            'struct msm_iova_data', 'node')

    def fl_offset(va):
        return (((va) & 0xFFF00000) >> 20)

    def sl_offset(va):
        return (((va) & 0xFF000) >> 12)

    def print_sl_page_table(self, pg_table):
        sl_pte = pg_table
        for i in range(0, self.NUM_SL_PTE):
            phy_addr = self.ramdump.read_u32(sl_pte, False)
            if phy_addr is not None:  # and phy_addr & self.SL_TYPE_SMALL:
                read_write = '[R/W]'
                if phy_addr & self.SL_AP2:
                    read_write = '[R]'

                if phy_addr & self.SL_TYPE_SMALL:
                    self.out_file.write('SL_PTE[%d] = %x %s\n' %
                                        (i, phy_addr & self.SL_BASE_MASK_SMALL, read_write))
                elif phy_addr & self.SL_TYPE_LARGE:
                    self.out_file.write('SL_PTE[%d] = %x %s\n' %
                                        (i, phy_addr & self.SL_BASE_MASK_LARGE, read_write))
                elif phy_addr != 0:
                    self.out_file.write(
                        'SL_PTE[%d] = %x NOTE: ERROR [Do not understand page table bits]\n' % (i, phy_addr))
            sl_pte += 4

    def print_page_table(self, pg_table):
        fl_pte = pg_table
        for i in range(0, self.NUM_FL_PTE):
        # for i in range(0,5):
            sl_pg_table_phy_addr = self.ramdump.read_u32(fl_pte)
            if sl_pg_table_phy_addr is not None:
                if sl_pg_table_phy_addr & self.FL_TYPE_TABLE:
                    self.out_file.write('FL_PTE[%d] = %x [4K/64K]\n' %
                                        (i, sl_pg_table_phy_addr & self.FL_BASE_MASK))
                    self.print_sl_page_table(
                        sl_pg_table_phy_addr & self.FL_BASE_MASK)
                elif sl_pg_table_phy_addr & self.FL_SUPERSECTION:
                    self.out_file.write('FL_PTE[%d] = %x [16M]\n' %
                                        (i, sl_pg_table_phy_addr & 0xFF000000))
                elif sl_pg_table_phy_addr & self.FL_TYPE_SECT:
                    self.out_file.write('FL_PTE[%d] = %x [1M]\n' %
                                        (i, sl_pg_table_phy_addr & 0xFFF00000))
                elif sl_pg_table_phy_addr != 0:
                    self.out_file.write(
                        'FL_PTE[%d] = %x NOTE: ERROR [Cannot understand first level page table entry]\n' % (i, sl_pg_table_phy_addr))
            else:
                self.out_file.write(
                    'FL_PTE[%d] NOTE: ERROR [Cannot understand first level page table entry]\n' % (i))
            fl_pte += 4

    def get_mapping_info(self, pg_table, index):
        sl_pte = pg_table + (index * 4)
        phy_addr = self.ramdump.read_u32(sl_pte, False)
        current_phy_addr = -1
        current_page_size = SZ_4K
        current_map_type = 0
        status = True
        if phy_addr is not None:
            if phy_addr & self.SL_AP2:
                current_map_type = self.SL_AP2
            if phy_addr & self.SL_TYPE_SMALL:
                current_phy_addr = phy_addr & self.SL_BASE_MASK_SMALL
                current_page_size = SZ_4K
            elif phy_addr & self.SL_TYPE_LARGE:
                current_phy_addr = phy_addr & self.SL_BASE_MASK_LARGE
                current_page_size = SZ_64K
            elif phy_addr != 0:
                current_phy_addr = phy_addr
                status = False

        return (current_phy_addr, current_page_size, current_map_type, status)

    def get_sect_mapping_info(self, addr):
        current_phy_addr = -1
        current_page_size = SZ_4K
        current_map_type = 0
        status = True
        if addr is not None:
            if addr & self.SL_AP2:
                current_map_type = self.SL_AP2
            if addr & self.FL_SUPERSECTION:
                current_phy_addr = addr & 0xFF000000
                current_page_size = SZ_16M
            elif addr & self.FL_TYPE_SECT:
                current_phy_addr = addr & 0xFFF00000
                current_page_size = SZ_1M
            elif addr != 0:
                current_phy_addr = addr
                status = False

        return (current_phy_addr, current_page_size, current_map_type, status)

    def add_flat_mapping(self, mappings, fl_idx, sl_idx, phy_adr, map_type, page_size, mapped):
        virt = (fl_idx << 20) | (sl_idx << 12)
        map_type_str = '[R/W]'
        if map_type == self.SL_AP2:
            map_type_str = '[R]'
        map = self.FlatMapping(virt, phy_adr, map_type_str, page_size, mapped)
        if virt not in mappings:
            mappings[virt] = map
        else:
            self.out_file.write(
                '[!] WARNING: FL_PTE[%d] SL_PTE[%d] ERROR [Duplicate mapping?]\n' % (fl_idx, sl_idx))
        return mappings

    def add_collapsed_mapping(self, mappings, virt_start, virt_end, phys_start, phys_end, map_type, page_size, mapped):
        map = self.CollapsedMapping(
            virt_start, virt_end, phys_start, phys_end, map_type, page_size, mapped)
        if virt_start not in mappings:
            mappings[virt_start] = map
        else:
            self.out_file.write(
                '[!] WARNING: ERROR [Duplicate mapping at virtual address 0x%08x?]\n' % (virt_start))
        return mappings

    def create_flat_mapping(self, pg_table):
        tmp_mapping = {}
        fl_pte = pg_table
        for fl_index in range(0, self.NUM_FL_PTE):
            fl_pg_table_entry = self.ramdump.read_u32(fl_pte)

            if fl_pg_table_entry is not None:
                if fl_pg_table_entry & self.FL_TYPE_SECT:
                    (phy_addr, page_size, map_type,
                     status) = self.get_sect_mapping_info(fl_pg_table_entry)
                    if status:
                        if phy_addr != -1:
                            tmp_mapping = self.add_flat_mapping(
                                tmp_mapping, fl_index, 0, phy_addr, map_type, page_size, True)
                        else:
                            # no mapping
                            tmp_mapping = self.add_flat_mapping(
                                tmp_mapping, fl_index, 0, -1, 0, 0, False)
                elif fl_pg_table_entry & self.FL_TYPE_TABLE:
                    sl_pte = fl_pg_table_entry & self.FL_BASE_MASK

                    for sl_index in range(0, self.NUM_SL_PTE):
                        (phy_addr, page_size, map_type,
                         status) = self.get_mapping_info(sl_pte, sl_index)
                        if status:
                            if phy_addr != -1:
                                tmp_mapping = self.add_flat_mapping(
                                    tmp_mapping, fl_index, sl_index, phy_addr, map_type, page_size, True)
                            else:
                                # no mapping
                                tmp_mapping = self.add_flat_mapping(
                                    tmp_mapping, fl_index, sl_index, -1, 0, 0, False)
                        else:
                            self.out_file.write(
                                '[!] WARNING: FL_PTE[%d] SL_PTE[%d] ERROR [Unknown error]\n' % (fl_index, sl_index))
                elif fl_pg_table_entry != 0:
                    self.out_file.write(
                        '[!] WARNING: FL_PTE[%d] = %x NOTE: ERROR [Cannot understand first level page table entry]\n' %
                        (fl_index, fl_pg_table_entry))
                else:
                    tmp_mapping = self.add_flat_mapping(
                        tmp_mapping, fl_index, 0, -1, 0, 0, False)
            else:
                self.out_file.write(
                    '[!] WARNING: FL_PTE[%d] NOTE: ERROR [Cannot understand first level page table entry]\n' % (fl_index))
            fl_pte += 4
        return tmp_mapping

    def create_collapsed_mapping(self, flat_mapping):
        collapsed_mapping = {}
        if len(flat_mapping.keys()) > 0:
            virt_addrs = sorted(flat_mapping.keys())
            start_map = prev_map = flat_mapping[virt_addrs[0]]
            last_mapping = False
            for virt in virt_addrs[1:]:
                map = flat_mapping[virt]
                new_mapping = False
                if map.mapping_size == prev_map.mapping_size and map.mapping_type == prev_map.mapping_type and map.mapped == prev_map.mapped:
                    if prev_map.mapping_size == SZ_4K:
                        if (map.phys - SZ_4K) != prev_map.phys and map.phys != prev_map.phys:
                            new_mapping = True
                    elif prev_map.mapping_size == SZ_64K:
                        if (map.phys - SZ_64K) != prev_map.phys and map.phys != prev_map.phys:
                            new_mapping = True
                    elif prev_map.mapping_size == SZ_1M:
                        if (map.phys - SZ_1M) != prev_map.phys and map.phys != prev_map.phys:
                            new_mapping = True
                    elif prev_map.mapping_size == SZ_16M:
                        if (map.phys - SZ_16M) != prev_map.phys and map.phys != prev_map.phys:
                            new_mapping = True
                    elif virt == virt_addrs[-1]:
                        # Last one
                        last_mapping = True
                else:
                    new_mapping = True
                if new_mapping:
                    collapsed_mapping = self.add_collapsed_mapping(
                        collapsed_mapping, start_map.virt, map.virt,
                        start_map.phys, prev_map.phys +
                        prev_map.mapping_size,
                        prev_map.mapping_type, prev_map.mapping_size, prev_map.mapped)
                    start_map = map
                elif last_mapping:
                    collapsed_mapping = self.add_collapsed_mapping(
                        collapsed_mapping, start_map.virt, 0xFFFFFFFF + 1,
                        start_map.phys, prev_map.phys +
                        prev_map.mapping_size,
                        prev_map.mapping_type, prev_map.mapping_size, prev_map.mapped)
                prev_map = map
        return collapsed_mapping

    def print_page_table_pretty(self, pg_table):
        flat_mapping = self.create_flat_mapping(pg_table)
        collapsed_mapping = self.create_collapsed_mapping(flat_mapping)

        for virt in sorted(collapsed_mapping.keys()):
            mapping = collapsed_mapping[virt]
            if mapping.mapped:
                self.out_file.write(
                    '0x%08x--0x%08x [0x%08x] A:0x%08x--0x%08x [0x%08x] %s[%s]\n' % (mapping.virt_start, mapping.virt_end, mapping.virt_size(),
                                                                                    mapping.phys_start, mapping.phys_end,
                                                                                    mapping.phys_size(), mapping.mapping_type, order_size_strings[get_order(mapping.mapping_size)]))
            else:
                self.out_file.write('0x%08x--0x%08x [0x%08x] [UNMAPPED]\n' %
                                    (mapping.virt_start, mapping.virt_end, mapping.virt_size()))

    def parse_short_form_tables(self, d, domain_num):
        self.out_file = self.ramdump.open_file(
            'msm_iommu_domain_%02d_0x%12X.txt' % (domain_num, d.pg_table))
        redirect = 'OFF'
        if d.redirect is None:
            redirect = 'UNKNOWN'
        elif d.redirect > 0:
            redirect = 'ON'
        iommu_context = 'None attached'
        if len(d.ctx_list) > 0:
            iommu_context = ''
            for (num, name) in d.ctx_list:
                iommu_context += '%s (%d) ' % (name, num)
        iommu_context = iommu_context.strip()

        self.out_file.write('IOMMU Context: %s. Domain: %s'
                            '[L2 cache redirect for page tables is %s]\n' % (
                                  iommu_context, d.client_name, redirect))
        self.out_file.write(
            '[VA Start -- VA End  ] [Size      ] [PA Start   -- PA End  ] [Size      ] [Read/Write][Page Table Entry Size]\n')
        if d.pg_table == 0:
            self.out_file.write(
                'No Page Table Found. (Probably a secure domain)\n')
        else:
            self.print_page_table_pretty(d.pg_table)
            self.out_file.write('\n-------------\nRAW Dump\n')
            self.print_page_table(d.pg_table)
        self.out_file.close()

    def parse(self):
        ilib = IommuLib(self.ramdump)
        self.domain_list = ilib.domain_list
        if self.domain_list is None:
            print_out_str(
                '[!] WARNING: IOMMU domains was not found in this build. No IOMMU page tables will be generated')
            return

        for (domain_num, d) in enumerate(self.domain_list):
            if self.ramdump.is_config_defined('CONFIG_IOMMU_LPAE'):
                parse_long_form_tables(self.ramdump, d, domain_num)
            elif (d.domain_type == MSM_SMMU_DOMAIN):
                self.parse_short_form_tables(d, domain_num)
            elif ((d.domain_type == ARM_SMMU_DOMAIN) or
                    (d.domain_type == MSM_SMMU_AARCH64_DOMAIN)):
                parse_aarch64_tables(self.ramdump, d, domain_num)
