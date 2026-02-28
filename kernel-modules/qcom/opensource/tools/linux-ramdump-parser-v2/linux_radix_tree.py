# Copyright (c) 2020 The Linux Foundation. All rights reserved.
# Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

'''
 :  radix_tree_root is the address of root node


            struct radix_tree_root {
               [0x0] spinlock_t xa_lock;
              [0x18] gfp_t gfp_mask;
              [0x20] struct radix_tree_node *rnode;
            }
    get_radix_tree_root will return the rnode
    walk_radix_tree_node start with this rnode
'''

class RadixTreeWalker(object):
    def __init__(self, ramdump):
        self.ramdump = ramdump
        self.RADIX_TREE_ENTRY_MASK = 3
        self.RADIX_TREE_INTERNAL_NODE = 1    # 1 for 4.19; 2 for 5.4
        self.RADIX_TREE_MAP_SHIFT = 6

        try:
            if int(self.ramdump.get_config_val("CONFIG_BASE_SMALL")) == 1:
                self.RADIX_TREE_MAP_SHIFT = 4
        except:
            # CONFIG_BASE_SMALL is not set
            pass
        self.RADIX_TREE_MAP_SIZE = (1 << self.RADIX_TREE_MAP_SHIFT)

        if (self.ramdump.kernel_version == (0, 0, 0) or
           self.ramdump.kernel_version >= (5, 4, 0)):
            self.root_struct = 'xarray'
            self.head_struct = 'xa_head'
            self.node_struct = 'xa_node'
            self.RADIX_TREE_INTERNAL_NODE = 2
        else:
            self.root_struct = 'radix_tree_root'
            self.head_struct = 'rnode'
            self.node_struct = 'radix_tree_node'
        self.node_shift_offset = self.ramdump.field_offset('struct ' + self.node_struct, 'shift')
        self.node_slots_offset = self.ramdump.field_offset('struct ' + self.node_struct, 'slots')
        self.node_size = self.ramdump.sizeof('struct ' + self.node_struct + ' *')

    def get_radix_tree_root(self, radix_tree_root):
        rnode_offset = self.ramdump.field_offset('struct ' + self.root_struct,self.head_struct)
        rnode_addr = self.ramdump.read_word(radix_tree_root + rnode_offset)
        return rnode_addr

    def entry_to_node(self, rbnode):
        return rbnode & ~self.RADIX_TREE_INTERNAL_NODE

    def is_internal_node(self, rbnode):
        # 10: Internal entry
        return (rbnode & self.RADIX_TREE_ENTRY_MASK) == self.RADIX_TREE_INTERNAL_NODE

    def walk_radix_tree_node(self, tree_node, height,func, *args):
        for off in range(0, self.RADIX_TREE_MAP_SIZE):
            slot = self.ramdump.read_word(tree_node + self.node_slots_offset + self.node_size * off)
            if slot is None or slot == 0:
                continue
            # RADIX_TREE_INTERNAL_NODE mean we are not leaf
            if self.is_internal_node(slot):
                slot = self.entry_to_node(slot)
            if height == 1:
                func(slot, *args)
            else:
                radix_tree_node_next = slot
                self.walk_radix_tree_node(radix_tree_node_next,(height -1), func, *args)

    def walk_radix_tree(self, radix_tree_root, func, *args):
        height = 0
        radix_tree_node = self.get_radix_tree_root(radix_tree_root)
        if radix_tree_node is None or radix_tree_node == 0:
            return
        if self.is_internal_node(radix_tree_node):
            radix_tree_node = self.entry_to_node(radix_tree_node)
            shift = self.ramdump.read_byte(radix_tree_node + self.node_shift_offset)
            height = (shift // self.RADIX_TREE_MAP_SHIFT) + 1
        if height == 0:
            func(radix_tree_node, *args)
        else:
            self.walk_radix_tree_node(radix_tree_node,height, func, *args)

    def walk_radix_tree_node_with_offset(self, tree_node, height,index, func, *args):
        for off in range(0, self.RADIX_TREE_MAP_SIZE):
            shift = (height - 1) * self.RADIX_TREE_MAP_SHIFT
            slot = self.ramdump.read_word(tree_node + self.node_slots_offset + self.node_size * off)
            if slot is None or slot == 0:
                continue
            # RADIX_TREE_INTERNAL_NODE mean we are not leaf
            if self.is_internal_node(slot):
                slot = self.entry_to_node(slot)
            if height == 1:
                func(slot, (index | off), *args)
            else:
                child_index = index | (off << shift)
                radix_tree_node_next = slot
                self.walk_radix_tree_node_with_offset(radix_tree_node_next,(height -1), child_index, func, *args)

    def walk_radix_tree_with_offset(self, radix_tree_root, func, *args):
        height = 0
        radix_tree_node = self.get_radix_tree_root(radix_tree_root)
        if radix_tree_node is None or radix_tree_node == 0:
            return
        if self.is_internal_node(radix_tree_node):
            radix_tree_node = self.entry_to_node(radix_tree_node)
            shift = self.ramdump.read_byte(radix_tree_node + self.node_shift_offset)
            height = (shift // self.RADIX_TREE_MAP_SHIFT) + 1
        if height == 0:
            func(radix_tree_node, 0, *args)
        else:
            self.walk_radix_tree_node_with_offset(radix_tree_node,height, 0,func, *args)
