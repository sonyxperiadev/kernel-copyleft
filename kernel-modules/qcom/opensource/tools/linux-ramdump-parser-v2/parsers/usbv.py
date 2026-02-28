# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.

from parsers.usb import UsbReader
from parser_util import register_parser

trb_completion_codes = ["invalid", "success", "data buffer err", "babble detected",
                        "USB transaction err", "trb err", "stall", "resource err", "bandwidth err",
                        "no slots avail", "invalid stream type", "device slot disabled",
                        "ep disabled", "short packet", "ring underrun", "ring overrun",
                        "vf event ring full", "param err", "bandwidth overrun", "context state err",
                        "no ping response", "event ring full", "incompatible device",
                        "missed service", "command ring stopped", "command aborted", "stopped",
                        "stopped - length invalid", "stopped - short packet",
                        "max exit latency too large", "rzvd", "isoch buffer overrun", "event lost",
                        "undefined err", "invalid stream id", "secondary bandwidth err",
                        "split transaction"]

trb_mask = 0x3f << 10

@register_parser('--usbv', 'Detailed usb device info')
class UsbVerboseReader(UsbReader):
    """
    A UsbReader subclass that prints more verbose debugging information, such as entire TRB rings or
    human-readable interpretations of command and event rings.
    """
    def __init__(self, dump):
        super(UsbVerboseReader, self).__init__(dump)
        self.outfile_name = 'usbv_output.txt'

    def get_trb_cmds(self, trb, length):
        """
        Interprets the commands in the given TRB, up to length. Each command is 16 bytes long.
        :param trb: TRB address
        :param length: TRB length
        :return: List of strings containing the interpreted commands
        """
        cmds = []

        for i in range(length):
            curr = trb + (i * 16)
            ctrl = self.ramdump.read_u32(curr + 12)
            if ctrl != 0:
                cmd = (ctrl & trb_mask) >> 10
                if cmd == 1:
                    # Normal Transfer
                    status = self.ramdump.read_u32(curr + 8)
                    transfer_length = status & 0x1ffff
                    td_size = (status & (0x1f << 17)) >> 17
                    ioc = (ctrl & (1 << 5)) >> 5
                    cmds.append(
                        f"READ {transfer_length} bytes. {td_size} packets remain. IOC = {ioc}")
                elif cmd == 2:
                    # Setup Stage Transfer
                    transfer_length = (self.ramdump.read_u32(curr + 4) & 0xffff0000) >> 16
                    ioc = (ctrl & (1 << 5)) >> 5
                    trt = (ctrl & (1 << 17)) >> 17
                    read_write = "ERROR"
                    types = ['no data transfer of', 'reserve', 'write', 'read']
                    if trt <= 3:
                        read_write = types[trt]
                    cmds.append(f"SETUP {read_write} {transfer_length} bytes. IOC = {ioc}")
                elif cmd == 3:
                    # Data Stage Transfer
                    status = self.ramdump.read_u32(curr + 8)
                    transfer_length = status & 0x1ffff
                    td_size = (status & (0x1f << 17)) >> 17
                    direction = (ctrl & (1 << 15)) >> 15
                    ioc = (ctrl & (1 << 5)) >> 5
                    read_write = "read" if direction == 1 else "write"
                    cmds.append(
                        f"DATA STAGE {read_write} {transfer_length} bytes. "
                        f"{td_size} packets remain. IOC = {ioc}")
                elif cmd == 4:
                    # Status Stage Transfer
                    ioc = (ctrl & (1 << 5)) >> 5
                    cmds.append(f"STATUS. IOC = {ioc}")
                elif cmd == 5:
                    # Isoch Transfer
                    status = self.ramdump.read_u32(curr + 8)
                    transfer_length = status & 0xffff
                    td_size = (status & (0x1f << 17)) >> 17
                    ioc = (ctrl & (1 << 5)) >> 5
                    cmds.append(
                        f"ISOCHronous transfer length {transfer_length}. "
                        f"{td_size} packets remain. IOC = {ioc}")
                elif cmd == 6:
                    # Link
                    seg_ptr = self.ramdump.read_u64(curr) >> 4
                    cmds.append(f"LINK, return to {hex(seg_ptr)}")
                elif cmd == 7:
                    # Event Data
                    event_data = self.ramdump.read_u64(curr)
                    ioc = (ctrl & (1 << 5)) >> 5
                    cmds.append(f"EVENT DATA, sending {hex(event_data)}. IOC = {ioc}")
                elif cmd == 8:
                    # No-op
                    cmds.append("NOP")
                elif cmd == 9:
                    # Enable Slot
                    slot_type = (ctrl & (0x1f << 16)) >> 16
                    cmds.append(f"ENABLE SLOT, Type {slot_type}")
                elif cmd == 10:
                    # Disable Slot
                    slot_id = (ctrl & (0xff << 24)) >> 24
                    cmds.append(f"DISABLE SLOT {slot_id}")
                elif cmd == 11:
                    # Address Device
                    slot_id = (ctrl & (0xff << 24)) >> 24
                    cmds.append(f"ADDRESS DEVICE {slot_id}")
                elif cmd == 12:
                        # Configure Endpoint
                        slot_id = (ctrl & (0xff << 24)) >> 24
                        cmds.append(f"CONFIGURE EP, Device {slot_id}")
                elif cmd == 13:
                    # Evaluate Context
                    ctxt_addr = self.ramdump.read_u64(curr)
                    slot_id = (ctrl & (0xff << 24)) >> 24
                    cmds.append(
                        f"EVALUATE CONTEXT {hex(ctxt_addr)} which interacts with device "
                        f"{slot_id}")
                elif cmd == 14:
                    # Reset Endpoint
                    slot_id = (ctrl & (0xff << 24)) >> 24
                    ep_id = (ctrl & (0x1f << 16)) >> 16
                    tsp = (ctrl & (1 << 9)) >> 1
                    cmds.append(f"RESET EP {ep_id}, Device {slot_id}")
                elif cmd == 15:
                    # Stop Endpoint
                    dvc_id = (ctrl & (0xff << 24)) >> 24
                    ep_id = (ctrl & (0x1f << 16)) >> 16
                    suspend = (ctrl & (1 << 23)) >> 23
                    s = f"STOP EP {ep_id}, Device {dvc_id}"
                    if suspend == 1:
                        s += " about to be suspended"
                    cmds.append(s)
                elif cmd == 16:
                    # Set TR Dequeue Pointer
                    dq_ptr = self.ramdump.read_u64(curr) >> 4
                    endpoint_id = (ctrl & (0x1f << 16)) >> 16
                    slot_id = (ctrl & (0xff << 24)) >> 24
                    cmds.append(
                        f"SET TR DEQUEUE POINTER to {hex(dq_ptr)}, Endpoint {endpoint_id}, "
                        f"Device {slot_id}")
                elif cmd == 17:
                    # Reset Device
                    slot_id = (ctrl & (0xff << 24)) >> 24
                    cmds.append(f"RESET DEVICE {slot_id}")
                elif cmd == 22:
                    # Force Header
                    data_lower = self.ramdump.read_u64(curr)
                    packet_type = data_lower & 0x1f
                    type_name = "RESERVED"
                    if packet_type == 0:
                        type_name = "link management"
                    elif packet_type == 4:
                        type_name = "transaction"
                    elif packet_type == 8:
                        type_name = "data"
                    elif packet_type == 12:
                        type_name = "isochronous timestamp"
                    header_mid_low = data_lower >> 5
                    header_hi = self.ramdump.read_u32(curr + 8)
                    header = (header_hi << 59) + header_mid_low
                    root_hub = (ctrl & (0xff << 24)) >> 24
                    cmds.append(
                        f"FORCE HEADER {hex(header)} of type {type_name}, "
                        f"Root hub port {root_hub}")
                elif cmd == 23:
                    # Nop
                    cmds.append("NOP Command")
                elif cmd == 32:
                    # Transfer Event
                    status = self.ramdump.read_u32(curr + 8)
                    transfer_length = status & 0xffffff
                    completion_code = (status & (0xff << 24)) >> 24
                    endpoint_id = (ctrl & (0x1f << 16)) >> 16
                    complete = "Failed"
                    if 37 <= completion_code <= 191:
                        complete += ": (reserved)"
                    elif completion_code > 191:
                        complete += ": vendor defined"
                    elif completion_code == 1 or (
                            completion_code == 13 and endpoint_id % 2 == 1):
                        complete = "Successful"
                    else:
                        complete += ": " + trb_completion_codes[completion_code]
                    slot_id = (ctrl & (0xff << 24)) >> 24
                    cmds.append(
                        f"TRANSFER EVENT {complete} of {transfer_length} bytes, "
                        f"Endpoint {endpoint_id}, Device {slot_id}")
                elif cmd == 33:
                    # Command Completion Event
                    status = self.ramdump.read_u32(curr + 8)
                    completion_code = (status & (0xff << 24)) >> 24
                    complete = "COMPLETE" if completion_code == 1 else "FAILED"
                    slot_id = (ctrl & (0xff << 24)) >> 24
                    cmds.append(f"COMMAND {complete} for device {slot_id}")
                elif cmd == 34:
                    # Port Status Change
                    port_id = (self.ramdump.read_u32(curr) & (0xff << 24)) >> 24
                    completion_code = (self.ramdump.read_u32(curr + 8) & (0xff << 24)) >> 24
                    if completion_code == 1:
                        cmds.append(f"PORT {port_id} STATUS CHANGE")
                    else:
                        cmds.append("ERROR during Port Status Change command")
                elif cmd == 37:
                    # Host Controller
                    cmds.append("xHC STATUS CHANGE")
                elif cmd == 38:
                    # Device Notification
                    cmd_trb = self.ramdump.read_u64(curr)
                    notif_type = (cmd_trb & (0xf << 4)) >> 4
                    notif_data = cmd_trb >> 8
                    completion_code = self.ramdump.read_u32(curr + 8)
                    complete = "SUCCESS" if completion_code == 1 else "FAILED"
                    slot_id = (ctrl & (0xff << 8)) >> 8
                    cmds.append(f"NOTIFICATION of type {notif_type} {complete} to device "
                                f"{slot_id}. Notification data: {notif_data}")
                elif cmd == 39:
                    # MFINDEX Wrap
                    cmds.append("MFINDEX register wrapping from 0x3FFFh to 0")
                else:
                    cmds.append("RESERVED")
        return cmds

    def read_xhci_ring(self, xhci_ring, name):
        """
        Returns a summary and the structure of the xhci ring given
        :param xhci_ring: Address of the xhci ring
        :param name: Name of the xhci ring
        :return: String containing the summary and structure
        """
        first_seg_addr = self.ramdump.struct_field_addr(xhci_ring, 'struct xhci_ring', 'first_seg')
        eq = self.ramdump.read_structure_field(xhci_ring, 'struct xhci_ring', 'enqueue')
        dq = self.ramdump.read_structure_field(xhci_ring, 'struct xhci_ring', 'dequeue')
        ring_struct = f"{name}\n\tEnqueue pointer located at: {hex(eq)}" \
                      f"\n\tDequeue pointer located at: {hex(dq)}\n"

        xhci_seg_trb_offset = self.ramdump.field_offset('struct xhci_segment', 'trbs')
        xhci_seg_next_offset = self.ramdump.field_offset('struct xhci_segment', 'next')

        first_seg = self.ramdump.read_pointer(first_seg_addr)
        eq_seg = self.ramdump.read_pointer(
            self.ramdump.struct_field_addr(xhci_ring, 'struct xhci_ring', 'enq_seg'))
        dq_seg = self.ramdump.read_pointer(
            self.ramdump.struct_field_addr(xhci_ring, 'struct xhci_ring', 'deq_seg'))
        num_segs = self.ramdump.read_u32(
            self.ramdump.struct_field_addr(xhci_ring, 'struct xhci_ring', 'num_segs'))
        curr_seg = first_seg

        ring_header = "Ring Structure:\n" + " " * (10 + self.ptr_size * 2) + "0" + " " * 9 + \
                      "4" + " " * 9 + "8" + " " * 8 + "12"

        for i in range(num_segs):
            trb = self.ramdump.read_pointer(curr_seg + xhci_seg_trb_offset)
            trb_ids = self.get_trb_cmds(trb, 256)

            trb_cmds = "\nTRB Commands:\n"
            for j in range(len(trb_ids)):
                trb_cmds += f"\t{j}: {trb_ids[j]}\n"

            eq_idx = -1
            if curr_seg == eq_seg:
                eq_idx = int((eq - trb) / 16)

            dq_idx = -1
            if curr_seg == dq_seg:
                dq_idx = int((dq - trb) / 16)

            ring_struct += f"{trb_cmds} \nSegment {i}:\n{ring_header}\n" \
                           f"{self.display_ring(trb, eq_idx, dq_idx)}\n\n"
            curr_seg = self.ramdump.read_pointer(curr_seg + xhci_seg_next_offset)

        return ring_struct

    def print_device_endpoints(self, eps):
        """
        Prints all endpoints given and their ring structures
        :param eps: Endpoints to be printed
        :return: None
        """
        ep_trb_pool_offset = self.ramdump.field_offset('struct dwc3_ep', 'trb_pool')
        ep_name_offset = self.ramdump.field_offset('struct dwc3_ep', 'name')
        ep_eq_offset = self.ramdump.field_offset('struct dwc3_ep', 'trb_enqueue')
        ep_dq_offset = self.ramdump.field_offset('struct dwc3_ep', 'trb_dequeue')

        ring_structs = "Detailed Ring Structures:\n"
        ring_summaries = "\nActive endpoint summaries:\n"

        for ep in eps:
            trb_pool = self.ramdump.read_pointer(ep + ep_trb_pool_offset)
            name = self.ramdump.read_cstring(ep + ep_name_offset)
            trb_eq = self.ramdump.read_byte(ep + ep_eq_offset)
            trb_dq = self.ramdump.read_byte(ep + ep_dq_offset)

            ring_summaries += f"{name}:\n\tAddress: {hex(ep)}\n\tActive trbs: " \
                              f"{self.count_active_trbs(trb_pool)}\n\tEnqueue Pointer: " \
                              f"Trb {trb_eq}\n\tDequeue Pointer: Trb {trb_dq}\n"
            ring_struct = f'Ring {name}\n{" " * (10 + self.ptr_size * 2)}0{" " * 9}4' \
                          f'{" " * 9}8{" " * 8}12\n' \
                          f'{self.display_ring(trb_pool, trb_eq, trb_dq)}\n\n'
            ring_structs += ring_struct

        print(f"{ring_summaries}\n{ring_structs}\n", file=self.outfile)

    def parse(self):
        # The methods above will be called instead of the superclass methods they override
        super().parse()
