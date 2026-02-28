#Copyright (c) 2023,2025 Qualcomm Innovation Center, Inc. All rights reserved.
#
#This program is free software; you can redistribute it and/or modify
#it under the terms of the GNU General Public License version 2 and
#only version 2 as published by the Free Software Foundation.
#
#This program is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU General Public License for more details.

from print_out import print_out_str
class hListWalker(object):

    '''
    ramdump: Reference to the ram dump
    list_head_addr: The address of the list_head
    list_elem_offset: The offset of the list_head in the structure that this list is container for.
    '''

    def __init__(self, ramdump, list_head_addr, list_elem_offset):
        self.ramdump = ramdump
        self.list_elem_offset = list_elem_offset
        self.list_head_addr = list_head_addr
        self.next_node = None
        self.seen_nodes = []

    def __iter__(self):
        return self

    def __next__(self):
        if self.next_node == None:
            offset = self.ramdump.field_offset('struct hlist_head', 'first')
            first_node_addr = self.list_head_addr + offset
            self.next_node = self.ramdump.read_word(first_node_addr)
        else:
            next_node_addr = self.next_node + self.ramdump.field_offset('struct hlist_node', 'next')
            self.next_node = self.ramdump.read_word(next_node_addr)

        if self.next_node in self.seen_nodes:
            print_out_str(
               '[!] WARNING: Cycle found in attach list (0x{:x}). List is corrupted!'.format(self.list_head_addr))
            raise StopIteration()
        elif self.next_node is not None and self.next_node != 0:
            self.seen_nodes.append(self.next_node)
            return self.next_node - self.list_elem_offset
        else:
            raise StopIteration()

    def walk(self, func, *args):
        """
        Walk the hash linked list, calling `func' on
        each node. `func' will be passed the current node and *args,
        if given.
        """
        self.next_node = None
        self.seen_nodes = []
        for obj in self:
            funcargs = [obj] + list(args)
            func(*funcargs)

