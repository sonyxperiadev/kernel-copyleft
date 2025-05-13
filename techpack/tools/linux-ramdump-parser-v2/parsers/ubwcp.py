# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
import linux_list
import enum
from print_out import print_out_str
from parser_util import RamParser, cleanupString, register_parser
from prettytable import PrettyTable

@register_parser('--ubwcp', 'Dump UBWC-P')
class Ubwcp(RamParser):
    def __init__(self, *args):
        super(Ubwcp, self).__init__(*args)

    def handle_dmabuf(self, dmabuf_addr, target_dict):
        file_addr = self.ramdump.read_structure_field(dmabuf_addr, 'struct dma_buf', 'file')
        target_dict['ino'] = "Null"
        if file_addr != 0:
            file_ino_addr = self.ramdump.read_structure_field(file_addr, 'struct file', 'f_inode')
            if file_ino_addr != 0:
                file_ino = self.ramdump.read_structure_field(file_ino_addr, 'struct inode', 'i_ino')
                target_dict['ino'] = "0x{0:x}".format(file_ino)
        return

    def store_data(self, target_dict, struct_obj, members=None):
        for k,v in struct_obj.__dict__.items():
            if members and k not in members:
                continue
            if "unused" in k:
                continue
            if isinstance(v, bool):
                target_dict[k] = v
            elif isinstance(v, int):
                target_dict[k] = "0x{0:x}".format(v)
                if "dma_buf" in k and v != 0:
                    self.handle_dmabuf(v, target_dict)
            elif isinstance(v, enum.Enum):
                target_dict[k] = v.name
            elif '__dict__' in dir(v):
                member_dict = {}
                self.store_data(member_dict, v)
                target_dict[k] = member_dict
            else:
                target_dict[k] = v
        return

    def parse_ubwcp_buffer(self, node, key, buffer_list:list, output_fd, members=None):
        buffer_dict = {}
        buf_addr = self.ramdump.container_of(node, 'struct ubwcp_buf', 'hnode')
        buf_members = ['address', 'key', 'buf_attr', 'perm', 'buf_attr_set', 'locked', 'lock_dir', 'lock_count', 'dma_buf', 'ula_pa', 'ula_size', 'mmdata']

        buffer_dict['hash_key'] = "0x{0:x}".format(key)
        buffer_dict['address'] = "0x{0:x}".format(buf_addr)

        ubwcp_buf_struct = self.ramdump.read_datatype(buf_addr, 'struct ubwcp_buf')
        self.store_data(buffer_dict, ubwcp_buf_struct, members=buf_members)

        buffer_list.append(buffer_dict)
        return

    def print_buf_dict(self, buf_dict, output_dict, parent=None):
        for key, value in buf_dict.items():
            if parent:
                key = "{}->{}".format(parent, key)
            if isinstance(value, dict):
                self.print_buf_dict(value, output_dict, parent=key)
            else:
                output_dict[key] = value
        return

    def print_column(self, table_ptr, column_list):
        table_ptr.field_names = column_list
        return

    def print_output(self, buffer_list, output_fd):
        output_fd.write("\n[Buffers]\n")

        if len(buffer_list) == 0:
            print_out_str('no buffer is found')
            return

        buf_print = PrettyTable()
        for i in range(0, len(buffer_list)):
            buf = buffer_list[i]
            output_dict = {}
            self.print_buf_dict(buf, output_dict)
            if i == 0:
                self.print_column(buf_print, output_dict.keys())

            buf_print.add_row(output_dict.values())

        output_fd.write(str(buf_print) + "\n")
        return

    def traverse_ubwcp_buffers(self, ubwcp_ptr, output_fd):
        buffer_list = []

        buf_table_offset = self.ramdump.field_offset('struct ubwcp_driver', 'buf_table')
        entry_size = self.ramdump.sizeof('((struct ubwcp_driver *)0x0)->buf_table[0]')
        hash_size = int(self.ramdump.sizeof('((struct ubwcp_driver *)0x0)->buf_table') / entry_size)
        next_offset = self.ramdump.field_offset('struct hlist_node', 'next')
        for i in range(0, hash_size):
            entry_addr = self.ramdump.array_index(ubwcp_ptr + buf_table_offset, 'struct hlist_head', i)
            first_entry_addr = self.ramdump.read_word(entry_addr)
            if first_entry_addr != 0:
                hash_list = linux_list.ListWalker(self.ramdump, first_entry_addr, next_offset)
                hash_list.walk(first_entry_addr, self.parse_ubwcp_buffer, i, buffer_list, output_fd)

        self.print_output(buffer_list, output_fd)
        return

    def ubwcp_info(self, ubwcp_ptr, output_fd):
        info_dict = {}
        info_members = ["address", "hw_ver_major", "hw_ver_minor", "ula_pool_base", "ula_pool_size"]

        info_dict['address'] = "0x{0:x}".format(ubwcp_ptr)
        ubwcp_driver_struct = self.ramdump.read_datatype(ubwcp_ptr, 'struct ubwcp_driver')
        self.store_data(info_dict, ubwcp_driver_struct, members=info_members)

        output_fd.write("[UBWC-P information]\n")
        info_print = PrettyTable()
        info_print.field_names = info_dict.keys()
        info_print.add_row(info_dict.values())
        output_fd.write(str(info_print) + "\n")
        return

    def parse_ubwcp(self, output_filename):
        ubwcp = self.ramdump.read_pointer('me')
        if ubwcp is None:
            print_out_str('ubwcp is not found')
            return

        output_fd = self.ramdump.open_file(output_filename)
        self.ubwcp_info(ubwcp, output_fd)
        self.traverse_ubwcp_buffers(ubwcp, output_fd)
        output_fd.close()
        return

    def parse(self):
        output_filename = "ubwcp.txt"
        self.parse_ubwcp(output_filename)
        print_out_str('---wrote the output to {}'.format(output_filename))