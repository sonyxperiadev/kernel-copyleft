# Copyright (c) 2013-2014, 2020 The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

try:
    from collections import OrderedDict
except ImportError:
    from ordereddict import OrderedDict

from print_out import print_out_str
from register import Register
from mmu import Armv7LPAEMMU
import sizes

NUM_FL_PTE = 4
NUM_SL_PTE = 512
NUM_TL_PTE = 512

def print_lpae_mappings(mappings, outfile):
    """Dump some page tables. `mappings' should already be sorted."""
    fmt = '[0x{vstart:08x}--0x{vend:08x}] [0x{size:08x}] [A:0x{pstart:08x}--0x{pend:08x}] [{attrs}][{sizestring}]\n'
    fmt_unmapped = '[0x{vstart:08x}--0x{vend:08x}] [0x{size:08x}] [UNMAPPED]\n'
    for ((virt_start, virt_end), info) in mappings.items():
        if info is None:
            outfile.write(fmt_unmapped.format(
                vstart=virt_start,
                vend=virt_end,
                size=virt_end - virt_start,
            ))
        else:
            outfile.write(fmt.format(
                vstart=virt_start,
                vend=virt_end,
                size=info.page_size,
                pstart=info.phys,
                pend=info.phys + info.page_size,
                attrs=','.join(info.get_attributes_strings()),
                sizestring=sizes.get_size_string(info.page_size)
            ))

def get_flat_mappings(domain, mmu):
    """Walk some LPAE IOMMU page tables by iterating over all possible
    page table entries at each level. Returns a dictionary of the
    form: {(virt_start, virt_end): LeafMapping object, ...}

    """
    mappings = {}
    n = mmu.input_addr_split
    virt_r = Register(fl_index=(n + 26, 30),
                      sl_index=(29, 21),
                      tl_index=(20, 12),
                      page_index=(11, 0))
    for fl_index in range(0, NUM_FL_PTE):
        virt_r.zero()
        virt_r.fl_index = fl_index
        info1 = mmu.translate_first_level(virt_r)
        if info1 is None:
            continue
        if info1.leaf:
            virt = virt_r.value
            mappings[virt, virt + info1.page_size] = info1
            continue

        # this is a table. do the second-level lookup:
        for sl_index in range(0, NUM_SL_PTE):
            virt_r.sl_index = sl_index
            info2 = mmu.translate_second_level(virt_r, info1.next_table_addr)
            if info2 is None:
                continue
            if info2.leaf:
                virt = virt_r.value
                mappings[virt, virt + info2.page_size] = info2
                continue

            # this is a table. do the third-level lookup:
            for tl_index in range(0, NUM_TL_PTE):
                virt_r.tl_index = tl_index
                info3 = mmu.translate_third_level(virt_r, info2.next_table_addr)
                if info3 is None:
                    continue
                if not info3.leaf:
                    raise Exception('Non-leaf third-level PTE???')
                virt = virt_r.value
                mappings[virt, virt + info3.page_size] = info3

    return OrderedDict(sorted(mappings.items()))

def get_coalesced_mappings(flat_mappings):
    """Convert some "flat" mappings (from `get_flat_mappings') to a more
    compact representation where contiguous ranges are coalesced.

    """
    # fair warning: things are about to get a little hairy. have fun.

    flat_items = flat_mappings.items()
    # samers maps indices into flat_items to coalesced virtual
    # starting addresses for those items.
    samers = {}
    # mark adjacent equivalent mappings
    for i, (virt_range, info) in enumerate(flat_items):
        virt_start, virt_end = virt_range
        if i == 0:
            cur_virt = virt_start
            continue
        prev_range, prev_info = flat_items[i - 1]
        prev_start, prev_end = prev_range
        if virt_start == prev_end and \
           info.attributes == prev_info.attributes:
            samers[i] = cur_virt
        else:
            cur_virt = virt_start

    # merge adjacent equivalent mappings. coalesced_mappings will be
    # keyed by starting virtual address alone.
    coalesced_mappings = {}
    for i, (virt_range, info) in enumerate(flat_items):
        virt_start, virt_end = virt_range
        page_size = virt_end - virt_start
        if i in samers:
            coalesced_mappings[samers[i]].page_size += page_size
            continue
        if virt_start not in coalesced_mappings:
            coalesced_mappings[virt_start] = info
            continue
        else:
            raise ValueError('We should have either gotten a samer or something not in coalesced_mappings...')

    # convert coalesced_mappings to cc, which is keyed by a 2-tuple of
    # the form: (virt_start, virt_end). Still mapping to the same
    # LeafMapping objects.
    cc = dict(((virt_start, virt_start + info.page_size), info)
              for virt_start,info in coalesced_mappings.items())
    # maintain order to facilitate finding unmapped gaps
    cc = OrderedDict(sorted(cc.items()))

    # fill in the unmapped gaps by adding mappings to `None':
    if len(cc) > 0:
        (first_vstart, first_vend), info = cc.items()[0]
        (last_vstart, last_vend), info = cc.items()[-1]
        if first_vstart != 0:
            cc[0, first_vstart] = None
        if last_vend != 0xffffffff:
            cc[last_vend, 0xffffffff] = None
        cc = OrderedDict(sorted(cc.items()))
        keys = cc.keys()
        for i, ((vstart, vend), info) in enumerate(cc.items()[1:-1]):
            prev_start, prev_end = keys[i] # no need for -1 since we're iterating starting at 1
            if prev_end != vstart:
                cc[prev_end, vstart] = None
    cc = OrderedDict(sorted(cc.items()))
    return cc


def parse_long_form_tables(dump, d, domain_num):
    fname = 'msm_iommu_domain_%02d_0x%12X.txt' % (domain_num, d.pg_table)
    with dump.open_file(fname) as outfile:

        print_out_str('LPAE Iommu page tables: ' + fname)

        t0sz = 0
        mmu = Armv7LPAEMMU(dump, d.pg_table, t0sz, virt_for_fl=True)

        redirect = 'OFF'
        if d.redirect is None:
            redirect = 'UNKNOWN'
        elif d.redirect > 0:
            redirect = 'ON'
        iommu_context = ' '.join('%s (%s)' % (name, num)
                                 for (name, num) in d.ctx_list)
        iommu_context = iommu_context or 'None attached'

        outfile.write('IOMMU Context: %s. Domain: %s (%d) [L2 cache redirect for page tables is %s]\n' % (
            iommu_context, d.client_name, d.domain_num, redirect))
        outfile.write(
            '[VA Start -- VA End  ] [Size      ] [PA Start   -- PA End  ] [Attributes][Page Table Entry Size]\n')
        if d.pg_table == 0:
            outfile.write(
                'No Page Table Found. (Probably a secure domain)\n')
        else:
            mappings = get_flat_mappings(d, mmu)
            print_lpae_mappings(get_coalesced_mappings(mappings), outfile)
            outfile.write('\n-------------\nRAW Dump\n')
            outfile.write('Raw: ' + str(d) + '\n')
            print_lpae_mappings(mappings, outfile)
