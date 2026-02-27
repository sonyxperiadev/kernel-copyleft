#Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
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
    node_addr: The address of the first element of the list
    list_elem_offset: The offset of the list_head in the structure that this list is container for.
    '''

    def __init__(self, ramdump, head_addr, list_elem_offset):
        self.ramdump = ramdump
        self.list_elem_offset = list_elem_offset
        self.first_node = head_addr
        self.next_node = None

    def __iter__(self):
        return self

    def __next__(self):
        if self.first_node:
            offset = self.ramdump.field_offset('struct hlist_head', 'first')
            first_node_addr = self.first_node + offset
            self.next_node = self.ramdump.read_word(first_node_addr)
            self.first_node = None
        else:
            if self.next_node:
                next_node_addr = self.next_node + self.ramdump.field_offset('struct hlist_node', 'next')
                self.next_node = self.ramdump.read_word(next_node_addr)
            else:
                raise StopIteration()

        if self.next_node is not None  and self.next_node != 0:
            return self.next_node - self.list_elem_offset
        else:
            raise StopIteration()

    def next(self):
        return self.__next__
