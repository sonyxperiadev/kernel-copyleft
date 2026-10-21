# Copyright (c) 2016-2018, 2020 The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from sizes import SZ_4K, SZ_64K, SZ_2M, SZ_32M, SZ_1G, SZ_256G
from sizes import get_order, order_size_strings
import re

NUM_PT_LEVEL = 4
NUM_FL_PTE = 512
NUM_SL_PTE = 512
NUM_TL_PTE = 512
NUM_LL_PTE = 512

FLSL_BASE_MASK = 0xFFFFFFFFF000
FLSL_TYPE_BLOCK = (1 << 0)
FLSL_TYPE_TABLE = (3 << 0)
FLSL_PTE_TYPE_MASK = (3 << 0)

LL_TYPE_PAGE = (3 << 0)
LL_PAGE_MASK = 0xFFFFFFFFF000

LL_AP_BITS = (0x3 << 6)
LL_CH = (0x1 << 52)
LL_XN = (0x1 << 54)
LL_ATTR_INDX = (0x7 << 2)
LL_SH_BITS = (0x3 << 8)

ATTR_IDX_NONCACHED = 0x0
ATTR_IDX_CACHE = 0x1
ATTR_IDX_DEV = 0x2
ATTR_IDX_UPST = 0x3
ATTR_IDX_LLC_NWA = 0x4

SH_NON_SHARE = (0x0 << 8)
SH_RESERVED = (0x1 << 8)
SH_OUTER_SHARE = (0x2 << 8)
SH_INNER_SHARE = (0x3 << 8)

LL_AP_RO = (0x3 << 6)
LL_AP_RW = (0x1 << 6)
LL_AP_PR_RW = (0x0 << 6)
LL_AP_PR_RO = (0x2 << 6)

class FlatMapping(object):

    def __init__(self, virt, phys=-1, type='[]', size=SZ_4K, attr_indx_str='[]',
                shareability_str='[]', execute_never_str='[]', mapped=False):
        self.virt = virt
        self.phys = phys
        self.type = type
        self.map_size = size
        self.attr_indx_str = attr_indx_str
        self.shareability_str = shareability_str
        self.execute_never_str = execute_never_str
        self.mapped = mapped


class CollapsedMapping(object):
    def __init__(self, virt_start, virt_end, phys_start=-1, phys_end=-1,
                map_type='[]', map_size=SZ_4K, attr_indx_str='[]',
                shareability_str='[]', execute_never_str='[]', mapped=False):

        self.virt_start = virt_start
        self.virt_end = virt_end - 1
        self.phys_start = phys_start
        self.phys_end = phys_end - 1
        self.map_type = map_type
        self.map_size = map_size
        self.attr_indx_str = attr_indx_str
        self.shareability_str = shareability_str
        self.execute_never_str = execute_never_str
        self.mapped = mapped

"""
Create a single Collapsed mapping representing the FlatMappings between
first and last, inclusive.
"""
def add_collapsed_mapping(mappings, first, last):
    virt_start = first.virt

    map = CollapsedMapping(
               virt_start = virt_start,
               virt_end = last.virt + last.map_size,
               phys_start = first.phys,
               phys_end = (last.phys & 0xFFFFFFFFF000) + last.map_size,
               map_type = first.type,
               map_size = first.map_size,
               attr_indx_str = first.attr_indx_str,
               shareability_str = first.shareability_str,
               execute_never_str = first.execute_never_str,
               mapped = first.mapped)

    if virt_start not in mappings:
        mappings[virt_start] = map
    else:
        map.type = 'Duplicate'
        mappings[virt_start] = map

"""
Combine adjacent holes in the page table, but leave all valid entries
unchanged.
"""
def create_collapsed_mapping(flat_mapping):
    collapsed_mapping = {}

    if not len(flat_mapping.keys()):
        return collapsed_mapping

    virt_addrs = sorted(flat_mapping.keys())
    start_map = prev_map = flat_mapping[virt_addrs[0]]
    new_mapping = False

    for virt in virt_addrs[1:]:
        map = flat_mapping[virt]

        if map.map_size == prev_map.map_size \
           and map.type == prev_map.type \
           and map.mapped == prev_map.mapped \
           and map.attr_indx_str == prev_map.attr_indx_str \
           and not map.mapped:
            new_mapping = False
        else:
            new_mapping = True

        if new_mapping:
            add_collapsed_mapping(
                collapsed_mapping, start_map, prev_map)
            start_map = map

        prev_map = map

    """Add the last entry"""
    add_collapsed_mapping(collapsed_mapping, start_map, prev_map)
    return collapsed_mapping


def add_flat_mapping(mappings, fl_idx, sl_idx, tl_idx, ll_idx, phy_addr,
                    map_type, page_size, attr_indx, shareability,
                    xn_bit, mapped):
    virt = (fl_idx << 39) | (sl_idx << 30) | (tl_idx << 21) | (ll_idx << 12)
    map_type_str = '[R/W]'
    attr_indx_str ='[]'

    if map_type == LL_AP_RO:
        map_type_str = '[RO]'
    elif map_type == LL_AP_PR_RW:
        map_type_str = '[P R/W]'
    elif map_type == LL_AP_PR_RO:
        map_type_str = '[P RO]'

    shareability_str = 'N/A'
    if shareability != -1:
        if shareability == SH_NON_SHARE:
            shareability_str = 'Non-Shareable'
        if shareability == SH_RESERVED:
            shareability_str = 'Reserved'
        if shareability == SH_OUTER_SHARE:
            shareability_str = 'Outer-Shareable'
        if shareability == SH_INNER_SHARE:
            shareability_str = 'Inner-Shareable'

    attr_indx_str = 'N/A'
    if attr_indx != -1:
        if attr_indx == ATTR_IDX_NONCACHED:
            attr_indx_str = 'Non-Cached'
        elif attr_indx == ATTR_IDX_CACHE:
            attr_indx_str = 'Cached'
        elif attr_indx == ATTR_IDX_DEV:
            attr_indx_str = 'Device'
        elif attr_indx == ATTR_IDX_UPST:
            attr_indx_str = 'UPSTREAM'
        elif attr_indx == ATTR_IDX_LLC_NWA:
            attr_indx_str = 'LLC_NWA'

    if xn_bit == 1:
        execute_never_str = 'True'
    elif xn_bit == 0:
        execute_never_str = 'False'
    elif xn_bit == -1:
        execute_never_str = 'N/A'

    map = FlatMapping(virt, phy_addr, map_type_str, page_size, attr_indx_str,
                      shareability_str, execute_never_str, mapped)

    if virt not in mappings:
        mappings[virt] = map
    else:
        map.type = 'Duplicate'
        mappings[virt] = map

    return mappings


def get_super_section_mapping_info(ramdump, pg_table, index):
    phy_addr = ramdump.read_u64(pg_table, False)
    current_phy_addr = -1
    current_page_size = SZ_1G
    current_map_type = 0
    status = True

    if phy_addr is not None:
        current_map_type = phy_addr & LL_AP_BITS
        current_phy_addr = phy_addr & 0xFFFFC0000FFF
    else:
        status = False

    return (current_phy_addr, current_page_size, current_map_type, status)

def get_section_mapping_info(ramdump, pg_table, index):
    phy_addr = ramdump.read_u64(pg_table, False)
    current_phy_addr = -1
    current_page_size = SZ_2M
    current_map_type = 0
    status = True
    section_skip_count = 0
    attr_indx = 0
    sh_bits = -1
    xn_bit = 0


    if phy_addr is not None:
        current_map_type = phy_addr & LL_AP_BITS

        attr_indx = ( (phy_addr & LL_ATTR_INDX) >> 2 )
        if attr_indx == ATTR_IDX_NONCACHED or attr_indx == ATTR_IDX_CACHE:
            sh_bits = phy_addr & LL_SH_BITS   # Shareability info available
                                              # only for Normal Memory

        if phy_addr & LL_XN:
            xn_bit = 1

        if phy_addr & LL_CH:
            current_phy_addr = phy_addr & 0xFFFFFE000FFF
            current_page_size = SZ_32M
            section_skip_count = 15
            # Current + next 15 entries are contiguous
        else:
            current_phy_addr = phy_addr & 0xFFFFFFE00FFF
            current_page_size = SZ_2M

    return (current_phy_addr, current_page_size, current_map_type,
            status, section_skip_count, attr_indx, sh_bits, xn_bit)


def get_mapping_info(ramdump, pg_table, index):
    ll_pte = pg_table + (index * 8)
    phy_addr = ramdump.read_u64(ll_pte, False)
    current_phy_addr = -1
    current_page_size = SZ_4K
    current_map_type = 0
    status = True
    skip_count = 0
    attr_indx = 0
    sh_bits = -1
    xn_bit = 0

    if phy_addr is not None:
        current_map_type = phy_addr & LL_AP_BITS

        if phy_addr & LL_TYPE_PAGE:
            current_phy_addr = phy_addr & 0xFFFFFFFFFFFF
            attr_indx = ( (phy_addr & LL_ATTR_INDX) >> 2 )
            if attr_indx == ATTR_IDX_NONCACHED or attr_indx == ATTR_IDX_CACHE:
                sh_bits = phy_addr & LL_SH_BITS   # Shareability info available
                                                  # only for Normal Memory

            if phy_addr & LL_XN:
                xn_bit = 1

            if phy_addr & LL_CH:
                current_phy_addr = phy_addr & 0xFFFFFFFF0FFF
                current_page_size = SZ_64K
                skip_count = 15
                # Current + next 15 entries are contiguous

        elif phy_addr != 0:
            # Error condition if at last level it is not LL_TYPE_PAGE
            current_phy_addr = phy_addr
            status = False
    return (current_phy_addr, current_page_size, current_map_type,
            status, skip_count, attr_indx, sh_bits, xn_bit)


def fl_entry(ramdump, fl_pte, skip_fl):
    fl_pg_table_entry = ramdump.read_u64(fl_pte)

    if fl_pg_table_entry is None:
        return (0, 0)

    sl_pte = fl_pg_table_entry & FLSL_BASE_MASK

    if skip_fl == 1:
        fl_pg_table_entry = FLSL_TYPE_TABLE
        sl_pte = fl_pte

        # Make 1st level entry look like dummy entry of type table in
        # case of only 3 level page tables and make sl_pte = fl_pte
        # as we start parsing from second level.

    return (fl_pg_table_entry, sl_pte)


def parse_2nd_level_table(ramdump, sl_pg_table_entry, fl_index,
                          sl_index, tmp_mapping):
    tl_pte = sl_pg_table_entry & FLSL_BASE_MASK
    section_skip_count = 0

    for tl_index in range(0, NUM_TL_PTE):
        tl_pg_table_entry = ramdump.read_u64(tl_pte, False)

        if tl_pg_table_entry == 0 or tl_pg_table_entry is None:
            tmp_mapping = add_flat_mapping(
                          tmp_mapping, fl_index, sl_index,
                          tl_index, 0, -1,
                          -1, SZ_2M, -1, -1, -1, False)
            tl_pte += 8
            continue

        tl_entry_type = tl_pg_table_entry & FLSL_PTE_TYPE_MASK
        if tl_entry_type == FLSL_TYPE_TABLE:
            ll_pte = tl_pg_table_entry & FLSL_BASE_MASK
            skip_count = 0

            for ll_index in range(0, NUM_LL_PTE):
                if skip_count:
                    skip_count -= 1
                    continue

                (phy_addr, page_size, map_type, status,
                    skip_count, attr_indx, shareability,
                    xn_bit) = get_mapping_info(ramdump, ll_pte, ll_index)

                if status and phy_addr != -1:
                    tmp_mapping = add_flat_mapping(
                        tmp_mapping, fl_index, sl_index,
                        tl_index, ll_index, phy_addr, map_type,
                        page_size, attr_indx, shareability, xn_bit, True)
                else:
                    tmp_mapping = add_flat_mapping(
                        tmp_mapping, fl_index, sl_index,
                        tl_index, ll_index, -1,
                        -1, page_size, attr_indx, shareability, xn_bit, False)

        elif tl_entry_type == FLSL_TYPE_BLOCK:
            if section_skip_count:
                section_skip_count -= 1
                continue

            (phy_addr, page_size, map_type, status,
                section_skip_count, attr_indx, shareability,
                xn_bit) = get_section_mapping_info(ramdump, tl_pte, tl_index)
            if status and phy_addr != -1:
                tmp_mapping = add_flat_mapping(
                    tmp_mapping, fl_index, sl_index,
                    tl_index, 0, phy_addr,
                    map_type, page_size, attr_indx, shareability, xn_bit, True)

        tl_pte += 8
    return tmp_mapping

def create_flat_mappings(ramdump, pg_table, level):
    tmp_mapping = {}
    fl_pte = pg_table
    skip_fl = 0
    fl_range = NUM_FL_PTE
    read_virtual = False

    if level == 3:
        skip_fl = 1
        fl_range = 1
        read_virtual = True

    # In case we have only 3 level page table we want to skip first level
    # and just parse second, third and last level. To keep unify code for 3
    # level and 4 level parsing just run first level loop once and directly
    # jump to start parsing from second level

    for fl_index in range(0, fl_range):

        (fl_pg_table_entry, sl_pte) = fl_entry(ramdump, fl_pte, skip_fl)

        if fl_pg_table_entry == 0:
            tmp_mapping = add_flat_mapping(
                                        tmp_mapping, fl_index, 0, 0, 0,
                                        -1, -1, SZ_256G, -1, -1, -1, False)
            fl_pte += 8
            continue

        for sl_index in range(0, NUM_SL_PTE):

            sl_pg_table_entry = ramdump.read_u64(sl_pte, read_virtual)

            if sl_pg_table_entry == 0 or sl_pg_table_entry is None:
                tmp_mapping = add_flat_mapping(tmp_mapping,
                                               fl_index, sl_index, 0, 0,
                                               -1, -1, SZ_1G, -1, -1, -1, False)
                sl_pte += 8
                continue

            sl_entry_type = sl_pg_table_entry & FLSL_PTE_TYPE_MASK
            if sl_entry_type == FLSL_TYPE_TABLE:
                tmp_mapping = parse_2nd_level_table(ramdump, sl_pg_table_entry,
                                                    fl_index, sl_index,
                                                    tmp_mapping)
            elif sl_entry_type == FLSL_TYPE_BLOCK:
                (phy_addr, page_size, map_type, status) \
                    = get_super_section_mapping_info(ramdump, sl_pte, sl_index)

                if status and phy_addr != -1:
                    #TODO: Fix memory attributes for 2nd-level entry
                    tmp_mapping = add_flat_mapping(
                        tmp_mapping, fl_index, sl_index, 0, 0,
                        phy_addr, map_type, page_size, -1, -1, -1, True)

            sl_pte += 8
        fl_pte += 8
    return tmp_mapping


def parse_aarch64_tables(ramdump, d, domain_num):
    device_name  = re.sub("[^a-zA-Z]+", "_", d.client_name.strip())
    if device_name is None:
        device_name = "xxxx"
    fname = 'arm_iommu_domain_%02d_%s_0x%12X.txt' % (domain_num, device_name,
                                                     d.pg_table)
    with ramdump.open_file(fname) as outfile:

        redirect = 'OFF'
        if d.redirect is None:
            redirect = 'UNKNOWN'
        elif d.redirect > 0:
            redirect = 'ON'
        iommu_context = ' '.join('%s (%s)' % (name, num)
                                 for (name, num) in d.ctx_list)
        iommu_context = iommu_context or 'None attached'

        outfile.write(
            'IOMMU Context: %s. Domain: %s'
            '[L2 cache redirect for page tables is %s]\n' % (
                iommu_context, d.client_name, redirect))
        outfile.write(
            '[VA Start -- VA End  ] [Size      ] [PA Start   -- PA End  ] '
            '[Attributes][Page Table Entry Size] [Memory Type] '
            '[Shareability] [Non-Executable] \n')
        if d.pg_table == 0:
            outfile.write(
                'No Page Table Found. (Probably a secure domain)\n')
        else:
            flat_mapping = create_flat_mappings(ramdump, d.pg_table, d.level)
            collapsed_mapping = create_collapsed_mapping(flat_mapping)

            for virt in sorted(collapsed_mapping.keys()):
                mapping = collapsed_mapping[virt]

                if mapping.mapped:
                    outfile.write(
                        '0x%x--0x%x [0x%x] A:0x%x--0x%x [0x%x] %s[%s] [%s] '
                        '[%s] [%s]\n' %
                        (mapping.virt_start, mapping.virt_end,
                         mapping.map_size, mapping.phys_start,
                         mapping.phys_end, mapping.map_size, mapping.map_type,
                         order_size_strings[get_order(mapping.map_size)],
                         mapping.attr_indx_str, mapping.shareability_str,
                         mapping.execute_never_str))
                else:
                    outfile.write(
                        '0x%x--0x%x [0x%x] [UNMAPPED]\n' %
                        (mapping.virt_start, mapping.virt_end,
                         mapping.virt_end - mapping.virt_start + 1))
