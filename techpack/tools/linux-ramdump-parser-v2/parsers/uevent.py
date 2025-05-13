# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
import linux_list
from print_out import print_out_str
from parser_util import RamParser, cleanupString, register_parser

@register_parser('--uevent', 'Dump uevent socket')
class Uevent(RamParser):
    def __init__(self, *args):
        super(Uevent, self).__init__(*args)

    def parse_sock(self, node, head, data_list:list, output_fd):
        data_dict = {}
        data_head = self.ramdump.read_structure_field(node, 'struct sk_buff', 'head')
        if node != head or data_head != 0:
            data_dict['head'] = data_head
            data_len = self.ramdump.read_structure_field(node, 'struct sk_buff', 'len')
            data_len = data_len if data_len < 1000 else 1000
            read_pos = 0
            data = ""
            while read_pos < data_len:
                read_str = self.ramdump.read_cstring(data_head + read_pos, data_len - read_pos)
                read_len = len(read_str)
                if read_len == 0:
                    read_str = " "
                    read_len = 1
                else:
                    read_str = read_str.strip()
                data = "{}{}".format(data, read_str)
                read_pos = read_pos + read_len

            data_dict['msg'] = data
            data_list.append(data_dict)
        return

    def parse_netlink(self, netlink_node, output_fd):
        liner = "-" * 133
        sk = self.ramdump.struct_field_addr(netlink_node, 'struct netlink_sock', 'sk')
        eth = self.ramdump.read_structure_field(netlink_node, 'struct netlink_sock', 'sk.sk_protocol')
        portid = self.ramdump.read_structure_field(netlink_node, 'struct netlink_sock', 'portid')
        groups = self.ramdump.read_structure_field(netlink_node, 'struct netlink_sock', 'groups')
        groups = self.ramdump.read_pointer(groups)
        flags = self.ramdump.read_structure_field(netlink_node, 'struct netlink_sock', 'flags')
        state = self.ramdump.read_structure_field(netlink_node, 'struct netlink_sock', 'state')
        sk_rmem = self.ramdump.read_structure_field(netlink_node, 'struct netlink_sock', 'sk.sk_backlog.rmem_alloc.counter')
        sk_wmem = self.ramdump.read_structure_field(netlink_node, 'struct netlink_sock', 'sk.sk_wmem_alloc.refs.counter') - 1
        cbrun = self.ramdump.struct_field_addr(netlink_node, 'struct netlink_sock', 'cb_running')
        cbrun = self.ramdump.read_bool(cbrun)
        sk_refcnt = self.ramdump.read_structure_field(netlink_node, 'struct netlink_sock', 'sk.__sk_common.skc_refcnt.refs.counter')
        sk_drops = self.ramdump.read_structure_field(netlink_node, 'struct netlink_sock', 'sk.sk_drops.counter')
        sk_socket = self.ramdump.read_structure_field(netlink_node, 'struct netlink_sock', 'sk.sk_socket')
        sk_vfsino = 0
        if sk_socket != 0:
            sk_vfsino_addr = self.ramdump.container_of(sk_socket, 'struct socket_alloc', 'socket')
            sk_vfsino = self.ramdump.read_structure_field(sk_vfsino_addr, 'struct socket_alloc', 'vfs_inode.i_ino')

        output_fd.write("|{0:^20x}|{1:^5d}|{2:^8d}|{3:^10x}|{4:^10d}|{5:^10d}|{6:^7d}|{7:^10d}|{8:^10d}|{9:^10d}|{10:^10x}|{11:^10x}|\n".format( \
                            sk, eth, portid, groups, sk_rmem, sk_wmem, cbrun, sk_refcnt, sk_drops, sk_vfsino, flags, state))
        output_fd.write("{}\n".format(liner))

        queue_lenth = self.ramdump.read_structure_field(netlink_node, 'struct netlink_sock', 'sk.sk_receive_queue.qlen')
        output_fd.write("* pending receive buffers : {0:d}\n".format(queue_lenth))
        data_list = []
        if queue_lenth > 0:
            entry_head = self.ramdump.struct_field_addr(netlink_node, 'struct netlink_sock', 'sk.sk_receive_queue')
            entry_addr = self.ramdump.read_structure_field(netlink_node, 'struct netlink_sock', 'sk.sk_receive_queue.next')
            node_offset = self.ramdump.field_offset('struct sk_buff', 'next')
            netlink_list = linux_list.ListWalker(self.ramdump, entry_addr, node_offset)
            netlink_list.walk(entry_addr, self.parse_sock, entry_head, data_list, output_fd)

            for i in range(0, len(data_list)):
                data_dict = data_list[i]
                output_fd.write(" [{0:03d}] data:0x{1:x} msg: {2}\n".format(i, data_dict['head'], data_dict['msg']))
        return

    def parse_bind_node(self, bind_node, output_fd):
        header = ["sk", "Eth", "PortId", "Groups", "Rmem", "Wmem", "Dump", "Locks", "Drops", "Inode", "Flags", "State"]
        header_liner = "=" * 133
        output_fd.write("{}\n".format(header_liner))
        output_fd.write("|{0:^20s}|{1:^5s}|{2:^8s}|{3:^10s}|{4:^10s}|{5:^10s}|{6:^7s}|{7:^10s}|{8:^10s}|{9:^10s}|{10:^10s}|{11:^10s}|\n".format( \
                            header[0], header[1], header[2], header[3], header[4], header[5], header[6], header[7], header[8], header[9], header[10], header[11]))
        output_fd.write("{}\n".format(header_liner))

        self.parse_netlink(self.ramdump.container_of(bind_node, 'struct netlink_sock', 'sk'), output_fd)
        return

    def iterate_protocol_node(self, protocol, output_fd):
        table_entry_addr = self.ramdump.array_index(self.ramdump.read_pointer('nl_table'), 'struct netlink_table', protocol)
        node_entry_addr = self.ramdump.read_structure_field(table_entry_addr, 'struct netlink_table', 'mc_list.first')

        if node_entry_addr != 0:
            node_offset = self.ramdump.field_offset('struct sock', '__sk_common.skc_bind_node')
            netlink_list = linux_list.ListWalker(self.ramdump, node_entry_addr, node_offset)
            netlink_list.walk(node_entry_addr, self.parse_bind_node, output_fd)

        return

    def parse_uevent(self, node, head, output_fd):
        if node == head:
            return

        uevent_addr = self.ramdump.container_of(node, 'struct uevent_sock', 'list')

        sk_addr = self.ramdump.read_structure_field(uevent_addr, 'struct uevent_sock', 'sk')
        sk_type = self.ramdump.read_structure_field(sk_addr, 'struct sock', 'sk_type')
        sk_protocol = self.ramdump.read_structure_field(sk_addr, 'struct sock', 'sk_protocol')

        self.iterate_protocol_node(sk_protocol, output_fd)

        return

    def traverse_uevent_sock(self, output_filename):
        uevent_list_head = self.ramdump.address_of('uevent_sock_list')
        if uevent_list_head is None:
            print_out_str('uevent_sock_list is not found')
            return

        output_fd = self.ramdump.open_file(output_filename)

        next_offset = self.ramdump.field_offset('struct list_head', 'next')
        if uevent_list_head != 0:
            uevent_list = linux_list.ListWalker(self.ramdump, uevent_list_head, next_offset)
            uevent_list.walk(uevent_list_head, self.parse_uevent, uevent_list_head, output_fd)

        output_fd.close()
        return

    def parse(self):
        output_filename = "uevent.txt"
        self.traverse_uevent_sock(output_filename)
        print_out_str('---wrote the output to {}'.format(output_filename))
