# Copyright (c) 2012-2014, 2018 The Linux Foundation. All rights reserved.
# Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
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
import logging
"""
struct rb_node
{
	unsigned long  rb_parent_color;
#define	RB_RED		0
#define	RB_BLACK	1
	struct rb_node *rb_right;
	struct rb_node *rb_left;
} __attribute__((aligned(sizeof(long))));
"""

class RbTree(object):

    def __init__(self, ramdump, root,
                logger = None, debug = False):
        self.rd = ramdump
        self.rb_root = root

        if not logger:
            self.logger = logging.getLogger(__name__)
        else:
            self.logger = logger
        self.debug = debug

    def __rb_parent(self, node):
        parent_color = self.rd.read_structure_field(
                            node, 'struct rb_node', '__rb_parent_color')
        if  parent_color is None:
            self.logger.error("struct rb_node@0x{:x}: Memory access failed".format(node))
            return 0x0

        return parent_color & ~3

    def __rb_left(self, node):
        child = self.rd.read_structure_field(node, 'struct rb_node', 'rb_left')
        if child is None:
            self.logger.error("struct rb_node@0x{:x}: Memory access failed".format(node))
            return 0x0

        return child

    def __rb_right(self, node):
        child = self.rd.read_structure_field(node, 'struct rb_node', 'rb_right')
        if child is None:
            self.logger.error("struct rb_node@0x{:x}: Memory access failed".format(node))
            return 0x0

        return child

    def validate(self, parent, child):
        if not self.debug:
            return True

        if not parent or not child:
            return True

        _parent = self.__rb_parent(child)
        if parent != _parent:
            self.logger.error(
                            "Bad rb_tree child->parent pointer:\n"+
                            "Parent:    0x{:x}\n".format(parent)+
                            "Child:     0x{:x}\n".format(child))
            return False

        right = self.__rb_right(parent)
        left  = self.__rb_left(parent)
        if (child != left and child != right):
            self.logger.error(
                            "Bad rb_tree parent->child pointer:\n"+
                            "Parent:    0x{:x}\n".format(parent)+
                            "Child:     0x{:x}\n".format(child))
            return False

        return True


    def rb_parent(self, node):
        parent = self.__rb_parent(node)
        if self.validate(parent, node):
            return parent
        else:
            return 0

    def rb_left(self, node):
        child = self.__rb_left(node)
        if self.validate(node, child):
            return child
        else:
            return 0

    def rb_right(self, node):
        child = self.__rb_right(node)
        if self.validate(node, child):
            return child
        else:
            return 0

    def rb_first(self, node):
        if not node:
            return 0x0

        while node:
            prev = node
            node = self.rb_left(node)

        return prev

    def rb_next(self, node):
        if not node:
            return 0x0

        right = self.rb_right(node)
        if right:
            return self.rb_first(right)

        while True:
            parent = self.rb_parent(node)
            if not parent:
                return 0x0

            right = self.rb_right(parent)
            if node != right:
                return parent

            node = parent

    def __iter__(self):
        return self.RbTreeIter(self)

    class RbTreeIter(object):
        def __init__(self, rbtree):
            self.rbtree = rbtree
            self.is_head = True
            self.cur = None

        # for python3
        def __next__(self):
            if self.is_head is True:
                node = self.rbtree.rd.read_structure_field(
                    self.rbtree.rb_root, 'struct rb_root', 'rb_node')
                self.cur = self.rbtree.rb_first(node)
                self.is_head = False
            else:
                self.cur = self.rbtree.rb_next(self.cur)
            if self.cur is None or self.cur == 0:
                raise StopIteration
            return self.cur


class RbTreeWalker(object):

    def __init__(self, ram_dump):
        self.ram_dump = ram_dump
        self.right_offset = self.ram_dump.field_offset(
            'struct rb_node', 'rb_right')
        self.left_offset = self.ram_dump.field_offset(
            'struct rb_node', 'rb_left')

    def _walk(self, node, func, seen, extra):
        if node == None:
            print_out_str(" rb tree possible corrupted \n")
            return
        if node != 0:
            left_node_addr = node + self.left_offset
            left_node = self.ram_dump.read_word(left_node_addr)
            if left_node not in seen:
                seen.append(left_node)
                self._walk(left_node, func, seen, extra)

            func(node, extra)

            right_node_addr = node + self.right_offset
            right_node = self.ram_dump.read_word(right_node_addr)
            if right_node not in seen:
                seen.append(right_node)
                self._walk(right_node, func, seen, extra)

    def walk(self, node, func, extra=None):
        """Walks the RbTree, calling `func' on each iteration. `func' receives
        two arguments: the current `struct rb_node', and `extra'.

        """
        self._walk(node, func, [], extra)
