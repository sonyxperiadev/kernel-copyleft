# Copyright (c) 2013-2014, 2016-2017, 2020 The Linux Foundation. All rights reserved.
# Copyright (c) 2022-2023,2025 Qualcomm Innovation Center, Inc. All rights reserved.
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from print_out import print_out_str

'''
struct list_head {
	struct list_head *next, *prev;
};
'''

class ListWalker(object):

    '''
    ram_dump: Reference to the ram dump
    list_head_addr: The address of the list_head
    list_elem_offset: The offset of the list_head in the structure that this list is container for.
    '''

    def __init__(self, ram_dump, list_head_addr, list_elem_offset):

        self.ram_dump = ram_dump
        self.list_elem_offset = list_elem_offset
        self.list_head_addr = list_head_addr
        self.seen_nodes = []
        self.curr_node = list_head_addr

    def __iter__(self):
        return self

    def __next__(self):
        next_node_addr = self.curr_node + \
            self.ram_dump.field_offset('struct list_head', 'next')
        next_node = self.ram_dump.read_word(next_node_addr)

        self.curr_node = next_node
        if next_node == self.list_head_addr:
            raise StopIteration()
        elif next_node in self.seen_nodes:
            print_out_str(
               '[!] WARNING: Cycle found in attach list (0x{:x}). List is corrupted!'.format(self.list_head_addr))
            raise StopIteration()
        elif not next_node:
            raise StopIteration()
        else:
            self.seen_nodes.append(next_node)

            return next_node - self.list_elem_offset

    def is_empty(self):
        """
        Return True if the list is empty, False otherwise.
        """
        if self.list_head_addr is None:
            return True

        next_node_addr = self.list_head_addr + self.ram_dump.field_offset('struct list_head', 'next')
        next_node = self.ram_dump.read_word(next_node_addr)

        if next_node == self.list_head_addr:
            return True
        else:
            return False

    def walk(self, func, *args):
        """
        Walk the linked list, calling `func' on
        each node. `func' will be passed the current node and *args,
        if given.
        """
        self.seen_nodes = []
        self.curr_node = self.list_head_addr
        for obj in self:
            funcargs = [obj] + list(args)
            func(*funcargs)

    def walk_prev(self, func, *args):
        """
        traverse the list in reverse order,
        calling `func' on each node. `func' will be passed the current node and *args,
        if given.
        """
        self.seen_nodes = []
        self.curr_node = self.list_head_addr
        elements = []
        for obj in self:
            elements.insert(0, obj)

        for obj in elements:
            funcargs = [obj] + list(args)
            func(*funcargs)

