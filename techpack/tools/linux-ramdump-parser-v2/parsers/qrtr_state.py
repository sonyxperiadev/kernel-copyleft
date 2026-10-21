# Copyright (c) 2021 The Linux Foundation. All rights reserved.

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from print_out import print_out_str
from parser_util import register_parser, RamParser

import pdb
# Global variables
rnode_offset = 0
rnode_shift_offset = 0
slots_offset = 0
pointer_size = 0
count_offset = 0
sk_offset = 0
us_offset = 0
peer_offset = 0
node_offset = 0
port_offset = 0
sk_data_ready_offset = 0
qmi_data_read_addr = 0
node_id_offset = 0
ep_offset = 0
ep_xmit_offset = 0
read_data_offset = 0
rx_func_offset = 0
kobj_offset = 0
name_offset = 0
qrtr_tx_flow_offset = 0
tx_flow_pending_offset = 0

max_slots = 64


@register_parser('--qrtr-ports', 'Print all the qrtr ports information')
class QrtrParse(RamParser):
    def is_internal_node(self, addr):
        radix_tree_entry_mask = 0x3
        if self.ramdump.kernel_version > (4, 20, 0):
            radix_tree_internal_node = 0x2
        else:
            radix_tree_internal_node = 0x1
        isInternal = (addr & radix_tree_entry_mask) == radix_tree_internal_node
        return isInternal

    def entry_to_node(self, addr):
        if self.ramdump.kernel_version > (4, 20, 0):
            return addr & 0xfffffffffffffffd
        else:
            return addr & 0xfffffffffffffffe

    def print_qrtr_port_info(self, ram_dump, port_addr, qrtr_out):
        global us_offset, peer_offset, node_offset, port_offset, sk_data_ready_offset, sk_offset
        global qmi_data_read_addr

        us_node = ram_dump.read_int(port_addr + us_offset + node_offset)
        us_port = ram_dump.read_int(port_addr + us_offset + port_offset)
        peer_node = ram_dump.read_int(port_addr + peer_offset + node_offset)
        peer_port = ram_dump.read_int(port_addr + peer_offset + port_offset)
        qrtr_out.write("v.v (struct qrtr_sock){0}\n".format(hex(port_addr).rstrip('L'), sep=''))
        qrtr_out.write("    node:port   US[{0}:{1}]    PEER[{2}:{3}]\n".format(hex(us_node), hex(us_port).rstrip('L'), hex(peer_node),hex(peer_port),sep=''))

        sk_ready_data = ram_dump.read_word(port_addr + sk_offset + sk_data_ready_offset)
        #print(hex(sk_ready_data))
        #if sk_ready_data == qmi_data_read_addr:
            #qrtr_out.write("YES")
        return

    def print_qrtr_node_info(self, ram_dump, qn_desc, qrtr_out):
        global node_id_offset, ep_offset, ep_xmit_offset, read_data_offset, rx_func_offset
        global kobj_offset, name_offset

        # Get the values to be printed
        node_id = ram_dump.read_int(qn_desc + node_id_offset)

        if node_id == 7:
            dev_offset = ram_dump.field_offset('struct qrtr_mhi_dev', 'dev')
            struct_name = '(struct qrtr_mhi_dev)'
        else:
            dev_offset = ram_dump.field_offset('struct qrtr_smd_dev', 'dev')
            struct_name = '(struct qrtr_smd_dev)'

        tx_func_ptr_val = ram_dump.read_u64(qn_desc + ep_offset + ep_xmit_offset)
        tx_func_ptr = ram_dump.read_pointer(tx_func_ptr_val)
        tx_func_name = ram_dump.get_symbol_info1(tx_func_ptr)

        rx_func_ptr_val = ram_dump.read_pointer(qn_desc + read_data_offset + rx_func_offset)
        rx_func_name = ram_dump.get_symbol_info1(rx_func_ptr_val)

        ep_pointer = ram_dump.read_pointer(qn_desc + ep_offset)
        dev_pointer = ram_dump.read_pointer(ep_pointer + dev_offset)
        kobj_pointer = ram_dump.read_pointer(dev_pointer + kobj_offset)
        dev_name = ram_dump.read_cstring(kobj_pointer + name_offset)

        # Print qrtr_node details
        qrtr_out.write("\n")
        qrtr_out.write("===================================================================================\n")
        qrtr_out.write("IDX: {0} NID: {1} |  v.v (struct qrtr_node){2}\n".format(node_id, node_id, hex(qn_desc).rstrip('L'), sep=''))
        qrtr_out.write("===================================================================================\n")
        qrtr_out.write("tx: {0}\n".format(tx_func_name))
        qrtr_out.write("rx: {0}\n".format(rx_func_name.strip('.cfi_jt')))
        qrtr_out.write("\ndev: {0}| v.v {1}{2}\n".format(dev_name, struct_name, hex(ep_pointer).rstrip('L'), sep=''))

        # Get qrtr_tx_flow information
        self.process_qrtr_tx_flow(ram_dump, qn_desc, qrtr_out)

        qrtr_out.write("-----------------------------------------------------------------------------------\n")

        return

    def print_tx_flow_info(self, ram_dump, qn_desc, qrtr_out):
        global tx_flow_pending_offset
        pending = ram_dump.read_pointer(qn_desc + tx_flow_pending_offset)
        if pending >= 5:
            print(hex(qn_desc))
        return

    def process_xa_node_slot(self, ram_dump, rnode_addr, qrtr_type, qrtr_out):
        global slots_offset, pointer_size, count_offset, max_slots

        if self.is_internal_node(rnode_addr):
            node_addr = self.entry_to_node(rnode_addr)
            count = ram_dump.read_word(node_addr, count_offset)
            num_slots = (count >> 16) and 0xff
            for i in range(0, max_slots):
                slot = ram_dump.read_u64(node_addr + slots_offset + (pointer_size * i))
                if slot != 0:
                    self.process_xa_node_slot(ram_dump, slot, qrtr_type, qrtr_out)
                    num_slots = num_slots - 1
                    if num_slots == 0:
                        break
        else:
            if qrtr_type == 1:
                self.print_qrtr_node_info(ram_dump, rnode_addr, qrtr_out)
            if qrtr_type == 2:
                self.print_qrtr_port_info(ram_dump, rnode_addr, qrtr_out)
            if qrtr_type == 3:
                qrtr_out.write("TX flow: {0}\n".format(hex(rnode_addr).rstrip('L')))
                self.print_tx_flow_info(ram_dump, rnode_addr, qrtr_out)

        return

    def process_qrtr_tx_flow(self, ram_dump, qn_desc, qrtr_out):
        global qrtr_tx_flow_offset, rnode_offset

        tx_flow_addr = qn_desc + qrtr_tx_flow_offset
        rnode_addr = ram_dump.read_word(tx_flow_addr + rnode_offset)
        self.process_xa_node_slot(ram_dump, rnode_addr, 3, qrtr_out)

        return

    def parse_qrtr_nodes(self, ram_dump, root_addr, qrtr_out):
        rnode_addr = ram_dump.read_word(root_addr + rnode_offset)
        if (rnode_addr is None) or (rnode_addr == 0):
            qrtr_out.write("Error: qrtr nodes list not created!!!\n")
            return
        self.process_xa_node_slot(ram_dump, rnode_addr, 1, qrtr_out)

        return

    def parse_qrtr_ports(self, ram_dump, root_addr, qrtr_out):
        # Get offsets for required members in the qrtr_port struct
        idr_rt_offset = ram_dump.field_offset('struct idr', 'idr_rt')

        global rnode_offset
        xarray_addr = root_addr + idr_rt_offset
        rnode_addr = ram_dump.read_word(xarray_addr + rnode_offset)

        self.process_xa_node_slot(ram_dump, rnode_addr, 2, qrtr_out)

        return

    def qrtr_get_offsets(self):
        # Get offsets
        global rnode_offset, rnode_shift_offset, slots_offset, pointer_size, count_offset
        rnode_offset = self.ramdump.field_offset('struct xarray', 'xa_head')
        rnode_shift_offset = self.ramdump.field_offset('struct xa_node', 'shift')
        slots_offset = self.ramdump.field_offset('struct xa_node', 'slots')
        pointer_size = self.ramdump.sizeof('struct xa_node *')
        count_offset = self.ramdump.field_offset('struct xa_node', 'count')

        global us_offset, peer_offset, node_offset, port_offset, sk_offset, sk_data_ready_offset
        sk_offset = self.ramdump.field_offset('struct qrtr_sock', 'sk')
        sk_data_ready_offset = self.ramdump.field_offset('struct sock', 'sk_data_ready')
        us_offset = self.ramdump.field_offset('struct qrtr_sock', 'us')
        peer_offset = self.ramdump.field_offset('struct qrtr_sock', 'peer')
        node_offset = self.ramdump.field_offset('struct sockaddr_qrtr', 'sq_node')
        port_offset = self.ramdump.field_offset('struct sockaddr_qrtr', 'sq_port')
        qmi_data_read_addr = self.ramdump.address_of('qmi_data_ready$365cec2a77bf8c8c807d4b217be1be23')

        global node_id_offset, ep_offset, ep_xmit_offset, read_data_offset, rx_func_offset
        global kobj_offset, name_offset, qrtr_tx_flow_offset, tx_flow_pending_offset
        node_id_offset = self.ramdump.field_offset('struct qrtr_node', 'nid')
        ep_offset = self.ramdump.field_offset('struct qrtr_node', 'ep')
        ep_xmit_offset = self.ramdump.field_offset('struct qrtr_endpoint', 'xmit')
        read_data_offset = self.ramdump.field_offset('struct qrtr_node', 'read_data')
        rx_func_offset = self.ramdump.field_offset('struct kthread_work', 'func')
        kobj_offset = self.ramdump.field_offset('struct device', 'kobj')
        name_offset = self.ramdump.field_offset('struct kobject', 'name')
        qrtr_tx_flow_offset = self.ramdump.field_offset('struct qrtr_node', 'qrtr_tx_flow')
        tx_flow_pending_offset = self.ramdump.field_offset('struct qrtr_tx_flow', 'pending')

    def parse(self):
        qrtr_out = self.ramdump.open_file('qrtr_port.txt')

        major, minor, patch = self.ramdump.kernel_version
        if (major, minor) < (5, 4):
            qrtr_out.write("Kernel version not supported\n")
            return

        # Get offsets from structure members used in qrtr_ports & qrtr_nodes
        self.qrtr_get_offsets()

        # Parse qrtr_nodes
        qrtr_nodes = self.ramdump.address_of('qrtr_nodes')
        if qrtr_nodes is not None:
            qrtr_out.write("\n\n********************* NODE INFO *********************\n")
            self.parse_qrtr_nodes(self.ramdump, qrtr_nodes, qrtr_out)
            qrtr_out.write("\n\n")
        else:
            qrtr_out.write("Error: qrtr_nodes symbol not found in the ram dump!!!\n")

        # Parse qrtr_ports
        qrtr_ports = self.ramdump.address_of('qrtr_ports')
        if qrtr_ports is not None:
            qrtr_out.write("********************* PORT INFO *********************\n")
            self.parse_qrtr_ports(self.ramdump, qrtr_ports, qrtr_out)
            qrtr_out.write("\n***********************************************\n")
        else:
            qrtr_out.write("Error: qrtr_ports symbol not found in the ram dump!!!\n")

        qrtr_out.close()

        return