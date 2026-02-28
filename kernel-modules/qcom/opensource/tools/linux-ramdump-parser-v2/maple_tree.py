# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
# Maple Tree implementation
# Copyright (c) 2018-2022 Oracle Corporation
# Authors: Liam R. Howlett <Liam.Howlett@oracle.com>
#       Matthew Wilcox <willy@infradead.org>

import ctypes
from enum import Enum
from print_out import print_out_str

class MapleTreeWalker(object):
    def __init__(self, ram_dump, debug=False):
        self.ram_dump = ram_dump
        self.init_mtvars()

    def init_mtvars(self):
        self.ULONG_MAX = 0xFFFFFFFFFFFFFFFF
        self.XA_ZERO_ENTRY = self.xa_mk_internal(257)
        self.MAPLE_RESERVED_RANGE = 4096
        self.MAPLE_NODE_SLOTS = 31
        self.MAPLE_RANGE64_SLOTS = 16
        self.MAPLE_ARANGE64_SLOTS = 10
        self.MAPLE_NODE_MASK = 255
        self.MAPLE_NODE_TYPE_SHIFT = 0x03
        self.MAPLE_NODE_TYPE_MASK = 0x0F

        maple_type_list = self.ram_dump.gdbmi.get_enum_lookup_table('maple_type', self.ram_dump.sizeof('enum maple_type'))
        self.maple_type_enum = Enum('maple_type', maple_type_list, start=0)

        self.mt_max = [None] * len(self.maple_type_enum)
        self.mt_max[self.maple_type_enum.maple_dense.value] = self.MAPLE_NODE_SLOTS
        self.mt_max[self.maple_type_enum.maple_leaf_64.value] = self.ULONG_MAX
        self.mt_max[self.maple_type_enum.maple_range_64.value] = self.ULONG_MAX
        self.mt_max[self.maple_type_enum.maple_arange_64.value] = self.ULONG_MAX
        return

    def xa_mk_internal(self, val):
        return ((val << 2) | 2)

    def get_unsigned64(self, signedval):
        return ctypes.c_ulonglong(signedval).value

    def xa_is_internal(self, entry):
        return (entry & 3) == 2

    def xa_is_node(self, entry):
        return self.xa_is_internal(entry) and self.get_unsigned64(entry) > 4096

    def xa_is_value(self, entry):
        return (entry & 1)

    def xa_to_value(self, entry):
        return (entry >> 1)

    def xa_is_zero(self, entry):
        return (entry == self.XA_ZERO_ENTRY)

    def xa_to_internal(self, entry):
        return (entry >> 2)

    def mt_is_reserved(self, entry):
        return self.get_unsigned64(entry) > self.MAPLE_RESERVED_RANGE and self.xa_is_internal(entry)

    def mte_node_type(self, entry):
        return (entry >> self.MAPLE_NODE_TYPE_SHIFT) & self.MAPLE_NODE_TYPE_MASK

    def mte_to_node(self, entry):
        return (entry & ~self.MAPLE_NODE_MASK)

    def ma_is_leaf(self, type):
        return type < self.maple_type_enum.maple_range_64.value

    def mte_is_leaf(self, entry):
        return self.ma_is_leaf(self.mte_node_type(entry))

    def mt_dump_range(self, min, max, depth):
        return

    def mt_dump_entry(self, entry, min, max, depth, func, *args):
        self.mt_dump_range(min, max, depth)

        if self.xa_is_value(entry):
            return
        elif self.xa_is_zero(entry):
            return
        elif self.mt_is_reserved(entry):
            return
        else:
            pass

        if func:
            func(entry, *args)
        return

    def mt_dump_range64(self, mt, entry, min, max, depth, func, *args):
        node = self.mte_to_node(entry)
        node = self.ram_dump.struct_field_addr(node, 'struct maple_node', 'mr64')
        leaf = self.mte_is_leaf(entry)
        first = min

        for i in range(0, self.MAPLE_RANGE64_SLOTS):
            last = max

            slot = self.ram_dump.read_structure_field(node, 'struct maple_range_64', 'slot[{}]'.format(i))
            if i < (self.MAPLE_RANGE64_SLOTS - 1):
                last = self.ram_dump.read_structure_field(node, 'struct maple_range_64', 'pivot[{}]'.format(i))
            elif slot != 0 and max != self.mt_max[self.mte_node_type(entry)]:
                break
            if last == 0 and i > 0:
                break
            if leaf:
                mt_slot = self.ram_dump.read_structure_field(node, 'struct maple_range_64', 'slot[{}]'.format(i))
                self.mt_dump_entry(mt_slot, first, last, depth + 1, func, *args)
            elif slot:
                mt_slot = self.ram_dump.read_structure_field(node, 'struct maple_range_64', 'slot[{}]'.format(i))
                self.mt_dump_node(mt, mt_slot, first, last, depth + 1, func, *args)

            if last == max:
                break
            if self.get_unsigned64(last) > max:
                print_out_str("\n node 0x{0:x} last ({1:d}) > max ({2:d}) at pivot {3:d}!".format(node, last, max, i))
                break
            first = last + 1
        return

    def mt_dump_arange64(self, mt, entry, min, max, depth, func, *args):
        node = self.mte_to_node(entry)
        node = self.ram_dump.struct_field_addr(node, 'struct maple_node', 'ma64')
        leaf = self.mte_is_leaf(entry)
        first = min

        for i in range(0, self.MAPLE_ARANGE64_SLOTS):
            last = max

            slot = self.ram_dump.read_structure_field(node, 'struct maple_arange_64', 'slot[{}]'.format(i))
            if i < (self.MAPLE_ARANGE64_SLOTS - 1):
                last = self.ram_dump.read_structure_field(node, 'struct maple_arange_64', 'pivot[{}]'.format(i))
            elif slot == 0:
                break
            if last == 0 and i > 0:
                break
            if leaf:
                mt_slot = self.ram_dump.read_structure_field(node, 'struct maple_arange_64', 'slot[{}]'.format(i))
                self.mt_dump_entry(mt_slot, first, last, depth + 1, func, *args)
            elif slot:
                mt_slot = self.ram_dump.read_structure_field(node, 'struct maple_arange_64', 'slot[{}]'.format(i))
                self.mt_dump_node(mt, mt_slot, first, last, depth + 1, func, *args)

            if last == max:
                break
            if self.get_unsigned64(last) > max:
                print_out_str("\n node 0x{0:x} last ({1:d}) > max ({2:d}) at pivot {3:d}!".format(node, last, max, i))
                break
            first = last + 1
        return

    def mt_dump_node(self, mt, entry, min, max, depth, func, *args):
        node = self.mte_to_node(entry)
        type = self.mte_node_type(entry)

        self.mt_dump_range(min, max, depth)

        if type == self.maple_type_enum.maple_dense.value:
            for i in range(0, self.MAPLE_NODE_SLOTS):
                if min + i > max:
                    print_out_str("OUT OF RANGE: ")
                mt_slot = self.ram_dump.read_structure_field(node, 'struct maple_node', 'slot[{}]'.format(i))
                self.mt_dump_entry(mt_slot, min + i, min + i, depth, func, *args)
        elif type == self.maple_type_enum.maple_leaf_64.value or type == self.maple_type_enum.maple_range_64.value:
            self.mt_dump_range64(mt, entry, min, max, depth, func, *args)
        elif type == self.maple_type_enum.maple_arange_64.value:
            self.mt_dump_arange64(mt, entry, min, max, depth, func, *args)
        else:
            print_out_str("Unknown type entry({})".format(hex(entry)))
        return

    def walk(self, mt, func, *args):
        entry = self.ram_dump.read_structure_field(mt, 'struct maple_tree', 'ma_root')
        if not self.xa_is_node(entry):
            self.mt_dump_entry(entry, 0, 0, 0, func, *args)
        elif entry:
            self.mt_dump_node(mt, entry, 0, self.mt_max[self.mte_node_type(entry)], 0, func, *args)
        return
