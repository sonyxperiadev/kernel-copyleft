# Copyright (c) 2013-2020, The Linux Foundation. All rights reserved.
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

from bitops import bm, bvalsel
from register import Register
import sizes


class MMU(object):

    """Represents an MMU. Does virtual-to-physical address lookups,
    caching the results in a TLB.

    This is an abstract class that should not be used
    directly. Concrete subclasses should override the following
    methods:

    - load_page_tables()

    - page_table_walk(addr)

    - dump_page_tables(file_object)


    Interesting properties that will be set for usage in derived
    classes:

    - ramdump:: The RamDump instance being parsed

    """

    def __init__(self, ramdump, ttbr=None):
        self._tlb = {}
        self.ramdump = ramdump
        self.s2_walk = self.ramdump.s2_walk
        if self.s2_walk:
            self._tlbv2 = {}
            self.ttbr = self.ramdump.ttbr
            self.vttbr = self.ramdump.vttbr
        else:
            if ttbr is not None:
                self.ttbr = ttbr
            else:
                self.ttbr = self.ramdump.kernel_virt_to_phys(
                        self.ramdump.swapper_pg_dir_addr)
        self.load_page_tables()

    def virt_to_phys(self, addr, skip_tlb=False, save_in_tlb=True):
        """Do a virtual to physical address lookup and possibly cache the
        result in the "TLB".

        """
        if addr is None:
            return None

        page_addr = (addr >> 12) << 12
        page_offset = addr & 0xFFF

        if not skip_tlb:
            if page_addr in self._tlb:
                return self._tlb[page_addr] + page_offset

        phys_addr = self.page_table_walk(page_addr)
        if phys_addr is None:
            return None

        if self.s2_walk:
            ipa = phys_addr + page_offset
            phys_addr = (ipa >> 12) << 12
            page_offset = ipa & 0xFFF
            ipa2 = self.page_table_walkel2(phys_addr)
            pa = ipa2 + page_offset
            if save_in_tlb:
                self._tlb[phys_addr] = pa

            return pa
        else:
            if save_in_tlb:
                self._tlb[page_addr] = phys_addr
            return phys_addr + page_offset

    def load_page_tables(self):
        raise NotImplementedError

    def page_table_walk(self, virt):
        raise NotImplementedError

    def dump_page_tables(self, f):
        raise NotImplementedError


class Armv7MMU(MMU):

    """An MMU for ARMv7 (no LPAE)."""

    def load_page_tables(self):
        self.global_page_table = [0 for i in range(4096)]
        self.secondary_page_tables = [
            [0 for col in range(256)] for row in range(4096)]

        virt_address = 0x0
        gb_i = 0
        se_i = 0
        for l1_pte_ptr in range(self.ttbr, self.ttbr + (4096 * 4), 4):
            l1_pte = self.ramdump.read_word(l1_pte_ptr, False)
            self.global_page_table[gb_i] = l1_pte
            if l1_pte is None:
                gb_i += 1
                continue
            if (l1_pte & 3) == 0 or (l1_pte & 3) == 3:
                for k in range(0, 256):
                    virt_address += 0x1000
            elif (l1_pte & 3) == 2:
                if ((l1_pte & 0x40000) == 0):
                    l1_pte_counter = l1_pte & 0xFFF00000
                    for k in range(0, 256):
                        virt_address += 0x1000
                        l1_pte_counter += 0x1000
                else:
                    gb_i += 1
                    continue
            elif (l1_pte & 3) == 1:
                l2_pt_desc = l1_pte
                l2_pt_base = l2_pt_desc & (~0x3ff)
                for l2_pte_ptr in range(l2_pt_base, l2_pt_base + (256 * 4), 4):
                    virt_address += 0x1000
                    l2_pt_entry = self.ramdump.read_word(l2_pte_ptr, False)
                    self.secondary_page_tables[gb_i][se_i] = l2_pt_entry
                    se_i += 1
                se_i = 0
            gb_i += 1

    def page_table_walk(self, virt):
        global_offset = bvalsel(31, 20, virt)
        l1_pte = self.global_page_table[global_offset]
        if l1_pte is None:
            return None
        bit18 = (l1_pte & 0x40000) >> 18
        if (bvalsel(1, 0, l1_pte) == 1):
            l2_offset = bvalsel(19, 12, virt)
            l2_pte = self.secondary_page_tables[global_offset][l2_offset]
            if l2_pte is None:
                return None
            if (bvalsel(1, 0, l2_pte) == 2) or (bvalsel(1, 0, l2_pte) == 3):
                entry4kb = (l2_pte & bm(31, 12)) + bvalsel(11, 0, virt)
                return entry4kb
            elif (bvalsel(1, 0, l2_pte) == 1):
                entry64kb = (l2_pte & bm(31, 16)) + bvalsel(15, 0, virt)
                return entry64kb
        if (bvalsel(1, 0, l1_pte) == 2):
            onemb_entry = bm(31, 20) & l1_pte
            onemb_entry += bvalsel(19, 0, virt)
            return onemb_entry

        return 0

    def dump_page_tables(self, f):
        f.write(
            'Dumping page tables is not currently supported for Armv7MMU\n')
        f.flush()


class Armv7LPAEMMU(MMU):

    """An MMU for ARMv7 (with LPAE)"""
    # Descriptor types
    DESCRIPTOR_INVALID = 0x0
    DESCRIPTOR_BLOCK = 0x1
    DESCRIPTOR_TABLE = 0x3
    TL_DESCRIPTOR_RESERVED = 0x1
    TL_DESCRIPTOR_PAGE = 0x3

    # Mapping classes
    class MappingInfo(object): pass

    class LeafMapping(MappingInfo):
        def __init__(self, virt_r, descriptor, page_size, n):
            self.virt_r = virt_r

            self.descriptor = descriptor
            self.attributes = Register(
                descriptor.value,
                software=(58, 55),
                XN=(54, 54),
                PXN=(53, 53),
                contiguous_hint=(52, 52),
                nG=(11, 11),
                AF=(10, 10),
                sh_10=(9, 8),
                ap_21=(7, 6),
                ns=(5, 5),
                attr_index_20=(4, 2),
            )

            self.page_size = page_size
            self.leaf = True

            p = Register(output_address=(39, n), page_offset=(n - 1, 0))
            p.output_address = self.descriptor.output_address
            self.virt_r.add_field('rest', (n - 1, 0))
            p.page_offset |= self.virt_r.rest
            self.phys = p.value

        def __repr__(self):
            pstart, pend = self.phys_addr_range()
            return '[{:x}-{:x}][{:}]'.format(
                pstart, pend,
                ','.join(self.get_attributes_strings())
            )

        def phys_addr_range(self):
            return (self.phys, self.phys + self.page_size)

        def get_attributes_strings(self):
            attrs = [
                self.get_xn_string(), self.get_pxn_string(),
                self.get_contiguous_hint_string(),
                self.get_nG_string(), self.get_AF_string(),
                self.get_sh_string(), self.get_ap_21_string(),
                self.get_ns_string(), self.get_attr_index_20_string(),
            ]
            return [a for a in attrs if a != '']

        def get_xn_string(self):
            if self.attributes.XN == 1:
                return 'XN'
            return ''

        def get_pxn_string(self):
            if self.attributes.PXN == 1:
                return 'PXN'
            return ''

        def get_contiguous_hint_string(self):
            if self.attributes.contiguous_hint == 1:
                return 'Contiguous'
            return ''

        def get_nG_string(self):
            if self.attributes.nG == 1:
                return 'nG'
            return ''

        def get_AF_string(self):
            if self.attributes.AF == 1:
                return 'AF'
            return ''

        def get_sh_string(self):
            if self.attributes.sh_10 == 0b00:
                return 'Non-shareable'
            elif self.attributes.sh_10 == 0b01:
                return 'UNPREDICTABLE'
            elif self.attributes.sh_10 == 0b10:
                return 'Outer Shareable'
            elif self.attributes.sh_10 == 0b11:
                return 'Inner Shareable'
            raise ValueError('Impossible sh[1:0]: 0x%x' % self.attributes.sh_10)

        def get_ap_21_string(self):
            if self.attributes.ap_21 == 0b00:
                return 'R/W@PL1'
            elif self.attributes.ap_21 == 0b01:
                return 'R/W'
            elif self.attributes.ap_21 == 0b10:
                return 'R/O@PL1'
            elif self.attributes.ap_21 == 0b11:
                return 'R/O'
            raise ValueError('Impossible ap[2:1]: 0x%x' % self.attributes.ap_21)

        def get_ns_string(self):
            if self.attributes.ns == 1:
                return 'NS'
            return ''

        def get_attr_index_20_string(self):
            return 'AI=0x%x' % self.attributes.attr_index_20

    class TableMapping(MappingInfo):
        def __init__(self, next_table_addr):
            self.next_table_addr = next_table_addr
            self.leaf = False

        def __repr__(self):
            return '[Next Table: 0x%x]' % (
                self.next_table_addr
            )

    class FLBlockMapping(LeafMapping):
        def __init__(self, virt_r, desc):
            super(Armv7LPAEMMU.FLBlockMapping, self).__init__(
                virt_r, desc, sizes.SZ_1G, 30)

    class SLBlockMapping(LeafMapping):
        def __init__(self, virt_r, desc):
            super(Armv7LPAEMMU.SLBlockMapping, self).__init__(
                virt_r, desc, sizes.SZ_2M, 21)

    class TLPageMapping(LeafMapping):
        def __init__(self, virt_r, desc):
            super(Armv7LPAEMMU.TLPageMapping, self).__init__(
                virt_r, desc, sizes.SZ_4K, 12)

    class FLTableMapping(TableMapping): pass
    class SLTableMapping(TableMapping): pass


    # Exceptions
    class LookupException(Exception): pass
    class LookupExceptionFLSL(LookupException): pass
    class LookupExceptionTL(LookupException): pass

    def __init__(self, ramdump, pgtbl, txsz, virt_for_fl=False):
        """Constructor for Armv7LPAEMMU.

        - ramdump: RamDump instance

        - pgtbl: base address of page table

        - txsz: t0sz or t1sz (see ARM ARM B3.6.6 (rev 0406C.b))

        - virt_for_fl: whether we should do a virtual address lookup
          for the first-level page table. Note that it wouldn't make
          any sense to pass `True' here if this is the "main" mmu
          instance for a RamDump, because then the RamDump would try
          to invoke this very object to do the lookup, and we would
          recursively discover the higgs boson. This option is useful,
          though, for parsing LPAE page tables whose first-level page
          table is sitting in kernel address space (as is the case for
          the IOMMU LPAE page tables).

        """
        super(Armv7LPAEMMU, self).__init__(ramdump)
        self.pgtbl = pgtbl
        self.txsz = txsz
        self.virt_for_fl = virt_for_fl

        if (32 - txsz) > 30:
            self.initial_lkup_level = 1
            self.initial_block_split = 12
        else:
            self.initial_lkup_level = 2
            self.initial_block_split = 21

        if self.initial_lkup_level == 1:
            # see the ARMv7 ARM B3.6.6 (rev 0406C.b):
            self.input_addr_split = 5 - self.txsz
            if self.input_addr_split not in [4, 5]:
                raise ValueError("Invalid stage 1 first-level `n' value: 0x%x. Please check txsz."
                                % self.input_addr_split)
        else:
            # see the ARMv7 ARM B3.6.6 (rev 0406C.b):
            self.input_addr_split = 14 - self.txsz
            if self.input_addr_split not in range(7, 13):
                raise ValueError("Invalid stage 1 second-level (initial) `n' value: 0x%x. Please check txsz."
                                % self.input_addr_split)

    def do_fl_sl_level_lookup(self, table_base_address, table_index,
                              block_split, virtual=False):
        descriptor, addr = self.do_level_lookup(
            table_base_address, table_index, virtual=virtual)
        if descriptor.dtype == Armv7LPAEMMU.DESCRIPTOR_BLOCK:
            descriptor.add_field('output_address', (39, block_split))
        elif descriptor.dtype == Armv7LPAEMMU.DESCRIPTOR_TABLE:
            # we have bits 39:12 of the next-level table in
            # next_level_base_addr_upper
            descriptor.add_field('next_level_base_addr_upper', (39, 12))
        else:
            raise Armv7LPAEMMU.LookupExceptionFLSL(
                'Invalid stage 1 first- or second-level translation\ndescriptor: (%s)\naddr: (%s)'
                % (str(descriptor), str(addr))
            )
        return descriptor

    def do_tl_level_lookup(self, table_base_address, table_index):
        descriptor, addr = self.do_level_lookup(
            table_base_address, table_index)
        if descriptor.dtype == Armv7LPAEMMU.TL_DESCRIPTOR_PAGE:
            descriptor.add_field('output_address', (39, 12))
        else:
            raise Armv7LPAEMMU.LookupExceptionTL(
                'Invalid stage 1 third-level translation\ndescriptor: (%s)\naddr: (%s)'
                % (str(descriptor), str(addr))
            )
        return descriptor

    def do_level_lookup(self, table_base_address, table_index, virtual=False):
        """Does a base + index descriptor lookup.

        Returns a tuple with the Register object representing the found
        descriptor and a Register object representing the the computed
        descriptor address.

        """
        n = self.input_addr_split
        # these Registers are overkill but nice documentation:).
        table_base = Register(table_base_address, base=(39, n))
        descriptor_addr = Register(base=(39, n), offset=(n - 1, 3))
        descriptor_addr.base = table_base.base
        descriptor_addr.offset = table_index
        descriptor_val = self.ramdump.read_dword(
            descriptor_addr.value, virtual=virtual)
        descriptor = Register(descriptor_val, dtype=(1, 0))
        return descriptor, descriptor_addr

    def load_page_tables(self):
        pass

    def page_table_walk(self, virt):
        info = self.translate(virt)
        return info.phys if info is not None else None

    def translate_first_level(self, virt_r):
        try:
            fl_desc = self.do_fl_sl_level_lookup(self.pgtbl, virt_r.fl_index,
                                                 30, virtual=self.virt_for_fl)
        except Armv7LPAEMMU.LookupExceptionFLSL:
            return None

        # if we got a block descriptor we're done:
        if fl_desc.dtype == Armv7LPAEMMU.DESCRIPTOR_BLOCK:
            return Armv7LPAEMMU.FLBlockMapping(virt_r, fl_desc)

        base = Register(base=(39, 12))
        base.base = fl_desc.next_level_base_addr_upper
        return Armv7LPAEMMU.FLTableMapping(base.value)

    def translate_second_level(self, virt_r, level2_table_addr, block_split=None):
        if block_split is None:
            block_split = self.initial_block_split
        try:
            sl_desc = self.do_fl_sl_level_lookup(
                level2_table_addr, virt_r.sl_index, block_split)
                # res.next_table_addr, virt_r.sl_index, 12, 21)
        except Armv7LPAEMMU.LookupExceptionFLSL:
            return None

        # if we got a block descriptor we're done:
        if sl_desc.dtype == Armv7LPAEMMU.DESCRIPTOR_BLOCK:
            return Armv7LPAEMMU.SLBlockMapping(virt_r, sl_desc)

        base = Register(base=(39, 12))
        base.base = sl_desc.next_level_base_addr_upper
        return Armv7LPAEMMU.SLTableMapping(base.value)

    def translate_third_level(self, virt_r, level3_table_addr):
        try:
            tl_desc = self.do_tl_level_lookup(
                level3_table_addr, virt_r.tl_index)
        except Armv7LPAEMMU.LookupExceptionTL:
            return None

        return Armv7LPAEMMU.TLPageMapping(virt_r, tl_desc)

    def translate(self, virt):
        """Does a page table walk and returns a LeafMapping that describes the
        mapping (including the physical address and mapping
        attributes)

        """
        if self.initial_lkup_level == 1:
            virt_r = Register(virt,
                              fl_index=(self.input_addr_split + 26, 30),
                              sl_index=(29, 21),
                              tl_index=(20, 12),
                              page_index=(11, 0))

            res = self.translate_first_level(virt_r)

            if res is None or res.leaf:
                return res

            level2_table_addr = res.next_table_addr
        elif self.initial_lkup_level == 2:
            virt_r = Register(virt,
                              sl_index=(self.input_addr_split + 17, 21),
                              tl_index=(20, 12),
                              page_index=(11, 0))
            level2_table_addr = self.pgtbl
        else:
            raise ValueError('Invalid initial lookup level (0x%x). Should be 1 or 2.' %
                            self.initial_lkup_level)

        res = self.translate_second_level(virt_r, level2_table_addr)

        if res is None or res.leaf:
            return res

        level3_table_addr = res.next_table_addr
        return self.translate_third_level(virt_r, level3_table_addr)

    def dump_page_tables(self, f):
        f.write(
            'Dumping page tables is not currently supported for Armv7LPAEMMU\n')
        f.flush()

class Armv8MMU(MMU):

    """An MMU for ARMv8 VMSA"""
    # Descriptor types
    DESCRIPTOR_INVALID = 0x0
    DESCRIPTOR_BLOCK = 0x1
    DESCRIPTOR_TABLE = 0x3
    TL_DESCRIPTOR_RESERVED = 0x1
    TL_DESCRIPTOR_PAGE = 0x3


    def do_fl_sl_level_lookup(self, table_base_address, table_index,
                              input_addr_split, block_split):
        descriptor, addr = self.do_level_lookup(
            table_base_address, table_index,
            input_addr_split)
        if descriptor.dtype == Armv8MMU.DESCRIPTOR_BLOCK:
            descriptor.add_field('output_address', (47, block_split))
        elif descriptor.dtype == Armv8MMU.DESCRIPTOR_TABLE:
            # we have bits 39:12 of the next-level table in
            # next_level_base_addr_upper
            descriptor.add_field('next_level_base_addr_upper', (47, 12))
        else:
            raise Exception(
                'Invalid stage 1 first- or second-level translation\ndescriptor: (%s)\naddr: (%s)'
               % (str(descriptor), str(addr))
            )
        return descriptor
    def do_fl_sl_level_lookupel2v2(self, table_base_address, table_index,
                              input_addr_split, block_split):
        #print "====do_fl_sl_level_lookupel2v2"
        #print hex(table_base_address)
        #print hex(table_index)
        #print input_addr_split
        #print block_split
        descriptor, addr = self.do_level_lookupel2v2(
            table_base_address, table_index,
            input_addr_split)
        #print "descriptor do_fl_sl_level_lookupel2v2"
        #print descriptor
        if descriptor.dtype == Armv8MMU.DESCRIPTOR_BLOCK:
            descriptor.add_field('output_address', (47, block_split))
        elif descriptor.dtype == Armv8MMU.DESCRIPTOR_TABLE:
            # we have bits 39:12 of the next-level table in
            # next_level_base_addr_upper
            descriptor.add_field('next_level_base_addr_upper', (47, 12))
            #print "descriptor  addeding filed"
            #print descriptor
        else:
            raise Exception(
                'Invalid stage 1 first- or second-level translation\ndescriptor: (%s)\naddr: (%s)'
               % (str(descriptor), str(addr))
            )
        return descriptor
    def do_fl_sl_level_lookupel2(self, table_base_address, table_index,
                              input_addr_split, block_split):
        #print "===="
        #print hex(table_base_address)
        #print hex(table_index)
        #print input_addr_split
        #print block_split
        descriptor, addr = self.do_level_lookupel2v2(
            table_base_address, table_index,
            input_addr_split)
        #print "descriptor do_fl_sl_level_lookupel2"
        #print descriptor
        if descriptor.dtype == Armv8MMU.DESCRIPTOR_BLOCK:
            descriptor.add_field('output_address', (47, block_split))
        elif descriptor.dtype == Armv8MMU.DESCRIPTOR_TABLE:
            # we have bits 39:12 of the next-level table in
            # next_level_base_addr_upper
            descriptor.add_field('next_level_base_addr_upper', (47, 12))
            #print "descriptor  addeding filed"
            #print descriptor
        else:
            raise Exception(
                'Invalid stage 1 first- or second-level translation\ndescriptor: (%s)\naddr: (%s)'
               % (str(descriptor), str(addr))
            )
        return descriptor

    def do_fl_level_lookup(self, table_base_address, table_index,
                           input_addr_split):
        return self.do_fl_sl_level_lookup(table_base_address, table_index,
                                     input_addr_split, 30)

    def do_fl_level_lookupel2(self, table_base_address, table_index,
                           input_addr_split):
        return self.do_fl_sl_level_lookupel2(table_base_address, table_index,
                                     input_addr_split, 30)

    def do_sl_level_lookup(self, table_base_address, table_index):
        return self.do_fl_sl_level_lookup(table_base_address, table_index,
                                     12, 21)
    def do_sl_level_lookupel2(self, table_base_address, table_index):
        #print "do_sl_level_lookupel2 entry"
        return self.do_fl_sl_level_lookupel2v2(table_base_address, table_index,
                                     12, 21)

    def do_tl_level_lookup(self, table_base_address, table_index):
        #print "+++++ do_tl_level_lookup entry"
        descriptor, addr = self.do_level_lookup(
            table_base_address, table_index, 12)
        #print "descriptor in do_tl_level_lookup"
        #print descriptor
        #print descriptor.dtype
        if descriptor.dtype == Armv8MMU.TL_DESCRIPTOR_PAGE:
            #print "output_address in do_tl_level_lookup"
            descriptor.add_field('output_address', (47, 12))
        else:
            raise Exception(
                'Invalid stage 1 third-level translation\ndescriptor: (%s)\naddr: (%s)'
                % (str(descriptor), str(addr))
            )
        return descriptor

    def do_tl_level_lookupel2(self, table_base_address, table_index):
        descriptor, addr = self.do_level_lookupel2v2(
            table_base_address, table_index, 12)
        if descriptor.dtype == Armv8MMU.TL_DESCRIPTOR_PAGE:
            descriptor.add_field('output_address', (47, 12))
        else:
            raise Exception(
                'Invalid stage 1 third-level translation\ndescriptor: (%s)\naddr: (%s)'
                % (str(descriptor), str(addr))
            )
        return descriptor
    def do_level_lookupel2v2(self, table_base_address, table_index,
                        input_addr_split):
        """Does a base + index descriptor lookup.

        Returns a tuple with the Register object representing the found
        descriptor and a Register object representing the the computed
        descriptor address.

        """
        n = input_addr_split
        # these Registers are overkill but nice documentation:).
        table_base = Register(table_base_address, base=(47, n))
        descriptor_addr = Register(table_base_address, base=(47, n),
                                   offset=(n - 1, 3))

        descriptor_addr.offset = table_index
        descriptor_val = self.read_phys_dword(descriptor_addr.value)
        descriptor = Register(descriptor_val,
                              dtype=(1, 0))
        return descriptor, descriptor_addr
    def do_level_lookupel2(self, table_base_address, table_index,
                        input_addr_split):
        """Does a base + index descriptor lookup.

        Returns a tuple with the Register object representing the found
        descriptor and a Register object representing the the computed
        descriptor address.

        """
        n = input_addr_split
        table_base = Register(table_base_address, base=(47, n))
        descriptor_addr = Register(table_base_address, base=(47, n),
                                   offset=(n - 1, 3))
        descriptor_val = self.read_phys_dword(descriptor_addr.value)
        descriptor = Register(descriptor_val,
                              dtype=(1, 0))
        return descriptor, descriptor_addr
    def do_level_lookup(self, table_base_address, table_index,
                        input_addr_split):
        """Does a base + index descriptor lookup.

        Returns a tuple with the Register object representing the found
        descriptor and a Register object representing the the computed
        descriptor address.

        """
        n = input_addr_split
        if self.s2_walk:
            ttbr_phy = self.virt_to_physel2(table_base_address)
            table_base_address = ttbr_phy
        # these Registers are overkill but nice documentation:).
        table_base = Register(table_base_address, base=(47, n))
        descriptor_addr = Register(table_base_address, base=(47, n),
                                   offset=(n - 1, 3))
        descriptor_addr.offset = table_index
        descriptor_val = self.read_phys_dword(descriptor_addr.value)
        descriptor = Register(descriptor_val,
                              dtype=(1, 0))
        return descriptor, descriptor_addr

    def block_or_page_desc_2_phys(self, desc, virt_r, n):
        phys = Register(output_address=(47, n),
                        page_offset=(n - 1, 0))
        phys.output_address = desc.output_address
        virt_r.add_field('rest', (n - 1, 0))
        phys.page_offset |= virt_r.rest
        return phys.value
    def block_or_page_desc_2_physel2(self, desc, virt_r, n):
        phys = Register(output_address=(38, n),
                        page_offset=(n - 1, 0))
        phys.output_address = desc.output_address
        virt_r.add_field('rest', (n - 1, 0))
        phys.page_offset |= virt_r.rest
        return phys.value

    def fl_block_desc_2_phys(self, desc, virt_r):
        """Block descriptor to physical address."""
        return self.block_or_page_desc_2_phys(desc, virt_r, 30)

    def fl_block_desc_2_physel2(self, desc, virt_r):
        """Block descriptor to physical address."""
        return self.block_or_page_desc_2_phys(desc, virt_r, 30)

    def sl_block_desc_2_phys(self, desc, virt_r):
        """Block descriptor to physical address."""
        return self.block_or_page_desc_2_phys(desc, virt_r, 21)

    def sl_block_desc_2_physel2(self, desc, virt_r):
        """Block descriptor to physical address."""
        return self.block_or_page_desc_2_phys(desc, virt_r, 21)

    def tl_page_desc_2_phys(self, desc, virt_r):
        """Page descriptor to physical address."""
        return self.block_or_page_desc_2_phys(desc, virt_r, 12)

    def tl_page_desc_2_physel2(self, desc, virt_r):
        """Page descriptor to physical address."""
        return self.block_or_page_desc_2_phys(desc, virt_r, 12)

    def read_phys_dword(self, physaddr):
        return self.ramdump.read_dword(physaddr, virtual=False)

    def load_page_tables(self):
        pass

    def page_table_walk(self, virt):
        virt_r = Register(virt,
            zl_index=(47,39),
            fl_index=(38,30),
            sl_index=(29,21),
            tl_index=(20,12),
            page_index=(11,0))

        base = Register(base=(47, 12))
        base.value = self.ttbr

        if self.ramdump.va_bits == 48:
            try:
              zl_desc = self.do_fl_sl_level_lookup(self.ttbr, virt_r.zl_index, 12, 30)
            except:
              return None

            if zl_desc.dtype == Armv8MMU.DESCRIPTOR_BLOCK:
                return None

            base.base = zl_desc.next_level_base_addr_upper

        try:
          fl_desc = self.do_fl_sl_level_lookup(base.value, virt_r.fl_index, 12, 30)
        except:
          return None

        if fl_desc.dtype == Armv8MMU.DESCRIPTOR_BLOCK:
            return self.fl_block_desc_2_phys(fl_desc, virt_r)

        base.base = fl_desc.next_level_base_addr_upper
        try:
            sl_desc = self.do_sl_level_lookup(
                base.value, virt_r.sl_index)
        except:
            return None

        if sl_desc.dtype == Armv8MMU.DESCRIPTOR_BLOCK:
            r = self.sl_block_desc_2_phys(sl_desc, virt_r)
            return r

        base.base = sl_desc.next_level_base_addr_upper
        try:
            tl_desc = self.do_tl_level_lookup(base.value, virt_r.tl_index)
        except:
            return None

        r = self.tl_page_desc_2_phys(tl_desc, virt_r)
        return r
    def page_table_walkel2(self, virt):
        #print "page_table_walkel2 virt address = {0}".format(hex(virt))
        virt_r = Register(virt,
            zl_index=(47,37),
            fl_index=(36,30),
            sl_index=(29,21),
            tl_index=(20,12),
            page_index=(11,0))
        #print "page_table_walk entry"
        try:
          fl_desc = self.do_fl_sl_level_lookupel2(self.vttbr, virt_r.fl_index, 9, 30)
          #print "fl_desc.."
          #print fl_desc
          #print fl_desc.dtype
        except Exception as err:
          #print "err"
          #print err
          return None

        if fl_desc.dtype == Armv8MMU.DESCRIPTOR_BLOCK:
            #print "return from here ....."
            return self.fl_block_desc_2_physel2(fl_desc, virt_r)

        base = Register(base=(47, 12))
        #print "fl_desc.next_level_base_addr_upper = {0}".format(hex(fl_desc.next_level_base_addr_upper))
        base.base = fl_desc.next_level_base_addr_upper
        try:
            sl_desc = self.do_sl_level_lookupel2(
                base.value, virt_r.sl_index)
        except Exception as err:
            #print "err...+++"
            #print err
            return None

        if sl_desc.dtype == Armv8MMU.DESCRIPTOR_BLOCK:
            r = self.sl_block_desc_2_physel2(sl_desc, virt_r)
            return r

        base.base = sl_desc.next_level_base_addr_upper
        try:
            tl_desc = self.do_tl_level_lookupel2(base.value, virt_r.tl_index)
        except:
            return None

        r = self.tl_page_desc_2_physel2(tl_desc, virt_r)
        return r

    def virt_to_physel2(self, addr, skip_tlb=False, save_in_tlb=True):
        """Do a virtual to physical address lookup and possibly cache the
        result in the "TLB".

        """
        #print "virt_to_physel2 entry.. "
        if addr is None:
            return None

        page_addr = (addr >> 12) << 12
        page_offset = addr & 0xFFF

        if not skip_tlb:
            if page_addr in self._tlbv2:
                ##print "return from here..."
                return self._tlbv2[page_addr] + page_offset

        #print "page_addr virt_to_physel2 = {0}".format(hex(page_addr))
        phys_addr = self.page_table_walkel2(page_addr)
        if phys_addr is None:
            return None

        if save_in_tlb:
            self._tlbv2[page_addr] = phys_addr

        return phys_addr + page_offset
    def stage2_translation(self,ipa_addr,pa_page_offset):
        #print "stage2_translation.... {0}".format(hex(ipa_addr))
        pa = self.page_table_walkel2(ipa_addr)
        #print "stage2_translation.. {0}".format(hex(pa))
        pa2 = pa + pa_page_offset
        #print "stage2_translation.. pa2 = {0}".format(hex(pa2))
        return pa2

    def dump_page_tables(self, f):
        f.write(
            'Dumping page tables is not currently supported for Armv8MMU\n')
        f.flush()
