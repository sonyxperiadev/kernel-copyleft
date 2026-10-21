# Copyright (c) 2013-2014, 2016-2017, 2020 The Linux Foundation. All rights reserved.
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

'''
struct list_head {
	struct list_head *next, *prev;
};
'''

class ListWalker(object):

    '''
    ram_dump: Reference to the ram dump
    node_addr: The address of the first element of the list
    list_elem_offset: The offset of the list_head in the structure that this list is container for.
    '''

    def __init__(self, ram_dump, node_addr, list_elem_offset):

        self.ram_dump = ram_dump
        self.list_elem_offset = list_elem_offset
        self.last_node = node_addr
        self.seen_nodes = []
        self.curr_node = node_addr

    def __iter__(self):
        return self

    def __next__(self):
        next_node_addr = self.curr_node + \
            self.ram_dump.field_offset('struct list_head', 'next')
        next_node = self.ram_dump.read_word(next_node_addr)

        self.curr_node = next_node
        if next_node == self.last_node:
            raise StopIteration()
        elif next_node in self.seen_nodes:
            print_out_str(
               '[!] WARNING: Cycle found in attach list. List is corrupted!')
            raise StopIteration()
        else:
            self.seen_nodes.append(next_node)

            return next_node - self.list_elem_offset

    def next(self):
        return self.__next__()

    def is_empty(self):
        """Return True if the list is empty, False otherwise.

        """
        if self.last_node is None:
            return True

        next_node_addr = self.last_node + self.ram_dump.field_offset('struct list_head', 'next')
        next_node = self.ram_dump.read_word(next_node_addr)

        if next_node == self.last_node:
            return True
        else:
            return False

    def walk(self, node_addr, func, *args):
        """Walk the linked list starting at `node_addr', calling `func' on
        each node. `func' will be passed the current node and *args,
        if given.

        """
        if self.is_empty() == True:
            return

        while True:
            if node_addr == 0 or node_addr == None :
                break

            funcargs = [node_addr - self.list_elem_offset] + list(args)
            func(*funcargs)

            next_node_addr = node_addr + self.ram_dump.field_offset('struct list_head', 'next')
            next_node = self.ram_dump.read_word(next_node_addr)

            if next_node == self.last_node:
                break

            if next_node in self.seen_nodes:
                print_out_str(
                   '[!] WARNING: Cycle found in attach list. List is corrupted!')
                break
            node_addr = next_node
            self.seen_nodes.append(node_addr)

    def walk_prev(self, node_addr, func, *args):
        """Walk the linked list starting at `node_addr' previous node and traverse the list in
        reverse order, calling `func' on each node. `func' will be passed the current node and *args,
        if given.

        """
        if self.is_empty() == True:
            return

        node_addr = self.ram_dump.read_word(node_addr + self.ram_dump.field_offset('struct list_head', 'prev'))
        while True:
            if node_addr == 0:
                break

            funcargs = [node_addr - self.list_elem_offset] + list(args)
            func(*funcargs)

            prev_node_addr = node_addr + self.ram_dump.field_offset('struct list_head', 'prev')
            prev_node = self.ram_dump.read_word(prev_node_addr)

            if prev_node == self.last_node:
                break

            if prev_node in self.seen_nodes:
                print_out_str(
                   '[!] WARNING: Cycle found in attach list. List is corrupted!')
                break
            node_addr = prev_node
            self.seen_nodes.append(node_addr)
