# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.

import re

import parser_util
import print_out
from parser_util import register_parser, RamParser


gsi_set = {'rmnet', 'rndis', 'ecm', 'mbim'}

dwc3_cmd_types = ["Disconnected", "USB Reset", "U/L State Change", "Wakeup/Resume",
                  "Hibernation Req", "Suspend Entry", "Microframe Start", "L1 Suspend",
                  "Erratic Error", "Command Complete", "EV Buffer Overflow",
                  "Vendor DevTest Received", "Stopped on Disconnect", "L1 Resume Detected",
                  "LDM Response Received"]

# maps driver names to struct names
hsphy_name_map = {'msm-qusb-phy': 'qusb_phy', 'msm_eusb2_phy': 'msm_eusb2_phy',
                  'msm_usb_hsphy': 'msm_hsphy', 'msm-qusb-phy-v2': 'qusb_phy'}

phy_clk_enabled_map = {'struct msm_hsphy': 'clocks_enabled',
                       'struct msm_eusb2_phy': 'clocks_enabled',
                       'struct qusb_phy': 'None', 'struct msm_ssphy_qmp': 'clk_enabled'}

connected_mask = 0b01
softconnect_mask = 0b10


def parse_dwc3_ep_event(evt):
    """
    Parses dwc3 endpoint related event from the event buffer
    EP events will end with a 0 bit
    :param evt: 4 byte event
    :return: String command interpreting the event
    """
    ep_num = "EP " + str((evt & (0x1f << 1)) >> 1)
    evt_type = (evt & (0xf << 6)) >> 6
    evt_status = (evt & (0xf << 12)) >> 12
    upper_bits = (evt & (0xffff << 16)) >> 16
    if evt_type == 1:
        # XferComplete
        trb_ioc = (evt_status & 0b100) >> 2
        return f"{ep_num} Xfer Complete, TRB IOC = {trb_ioc}"
    elif evt_type == 2:
        # Xfer In Progress
        trb_ioc = (evt_status & 0b100) >> 2
        return f"{ep_num} Xfer In Progress, TRB IOC = {trb_ioc}"
    elif evt_type == 3:
        # Xfer Not Ready
        xfer_active = (evt_status & 0b1000) >> 3
        req_type = evt_status & 0b11
        xfer_reason = "Requested transfer not present in hardware"
        if xfer_active == 1:
            xfer_reason = "No valid TRBs available"
        ctrl_req_type = "Ctrl Data Req"
        if req_type == 0b10:
            ctrl_req_type = "Ctrl Status Req"
        return f"{ep_num} Xfer Not Ready, Type: {ctrl_req_type}. {xfer_reason}"
    elif evt_type == 6:
        # Stream Event
        s = "ERROR"
        if evt_status == 1:
            s = "StreamFound"
        elif evt_status == 2:
            s = "StreamNotFound"
        return f"{ep_num} Stream Event. {s}"
    elif evt_type == 7:
        # EP Command Complete
        cmd_type = (upper_bits & 0xf00) >> 8
        xfer_rsc_idx = upper_bits & 0x7f
        return f"{ep_num} Command Complete, Type: {cmd_type}, Xfer Resource Idx: " \
               f"{xfer_rsc_idx}, Status: {evt_status}"
    else:
        return f"UNKNOWN EVENT TYPE: {evt_type}"


def parse_dwc3_dvc_event(evt):
    """
    Parses a dwc3 device event from the event buffer
    Device events will end with 0x01
    :param evt: 4 byte event
    :return: String command interpreting the event
    """
    evt_type = (evt & (0xf << 8)) >> 8
    return dwc3_cmd_types[evt_type]


def trb_status(num_active, name, is_tx=False):
    """
    Returns a message about how full the given trb is based on num_active
    :param num_active: Number of active (non-empty, in use) TRBs in the ring
    :param name: Ring name
    :param is_tx: Transfer or receive ring (used for case where num_active == 0)
    :return: String message
    """
    if num_active == 0:
        s = "Host PC has no data to send"
        if is_tx:
            s = "No data received from IPA GSI. Please check with IPA GSI team"
        return f"{name} is EMPTY. {s}"
    elif num_active == 256:
        return f"{name} is Full with pending data to be picked up by GSI"
    elif num_active >= 256 * 0.9:
        return f"{name} is almost Full with pending data to be picked up by GSI"
    else:
        return f"{name} has {num_active} buffers to be picked up by GSI"


def read_endpoint_mask(mask):
    """
    Given a bitmask of endpoints (such as from struct usb_function), returns a string of all
    endpoints used by the bitmask in ascending order, with outbound eps before inbound eps
    :param mask: 32 bit mask representing
    :return: String of format 'name1 name2 name3 ...'
    """
    ret = ""
    for i in range(32):
        if mask & (1 << i) == (1 << i):
            io = "out "
            if i >= 16:
                io = "in "
            ret += "ep" + str(i % 16) + io
    return ret


# Optional = True here because if --everything is called we only want --usbv to be run
@register_parser('--usb', 'Usb device info', optional=True)
class UsbReader(RamParser):
    def __init__(self, dump):
        super(UsbReader, self).__init__(dump)
        self.outfile = None  # initialized in parse so that usbv can have a separate output file
        self.outfile_name = 'usb_output.txt'
        # Lets us know if we are on 64 or 32 bit architecture
        self.ptr_size = self.ramdump.sizeof("int*")

        self.device_driver_data_offset = self.ramdump.field_offset('struct device', 'driver_data')
        self.device_driver_offset = self.ramdump.field_offset('struct device', 'driver')
        self.device_parent_offset = self.ramdump.field_offset('struct device', 'parent')
        self.device_kobj_offset = self.ramdump.field_offset('struct device', 'kobj')
        self.kobj_entry_offset = self.ramdump.field_offset('struct kobject', 'entry')
        self.kobj_name_offset = self.ramdump.field_offset('struct kobject', 'name')
        self.dwc3_ep_trb_pool_offset = self.ramdump.field_offset('struct dwc3_ep', 'trb_pool')

    def display_ring(self, addr, enqueue=-1, dequeue=-1):
        """
        Creates a string that displays the ring contents at addr
        :param addr: Ring address
        :param enqueue: Position of the enqueue pointer, -1 if not in this segment
        :param dequeue: Position of the dequeue pointer, -1 if not in this segment
        :return: String containing ring contents
        """
        hdump = self.ramdump.hexdump(addr, 256 * 16, little_endian=False)
        if enqueue >= 0:
            split = hdump.splitlines()
            split[enqueue] += "<- Enqueue pointer "
            hdump = "\n".join(split)
        if dequeue >= 0:
            split = hdump.splitlines()
            split[dequeue] += "<- Dequeue pointer"
            hdump = "\n".join(split)

        pattern = re.compile(
            "(([a-fA-F0-9]{" + str(self.ptr_size * 2) + "}: (0000 ){6}([0-9]{4} ){2} \\.+\n){2,})")

        # TRBs are skipped if BPL, BPH, and CTRL are 0
        for match in pattern.findall(hdump):
            num_lines = match[0].count("\n")
            hdump = re.sub(pattern, "Skipping " + str(num_lines) + " empty TRBs\n", hdump, count=1)
        return hdump

    def count_active_trbs(self, trb_ptr):
        """
        Finds the number of active (HWO bit == 1) trbs in the given endpoint
        :param trb_ptr: Address of the first trb in the endpoint
        :return: Number of active trbs
        """
        num_active = 0
        for i in range(256):
            ctrl = self.ramdump.read_u32(
                trb_ptr + (i * 16) + self.ramdump.field_offset('struct dwc3_trb', 'ctrl'))
            if ctrl % 2 == 1:
                num_active += 1
        return num_active

    def find_faulty_request(self, ep, fault_addr):
        """
        Helper method for self.findCrashAddr()
        Examines each trb in ep to see if it tried to read memory from faultaddr
        Prints
        :param ep: Device endpoint containing TRB that caused the fault
        :param fault_addr: TRB address that caused the arm-smmu fault
        :return: Address of the faulty request if found, or None
        """
        # offsets
        list_head_next_offset = self.ramdump.field_offset('struct list_head', 'next')
        dwc_req_num_trbs_offset = self.ramdump.field_offset('struct dwc3_request', 'num_trbs')
        dwc_req_trb_offset = self.ramdump.field_offset('struct dwc3_request', 'trb')
        dwc3_req_list_offset = self.ramdump.field_offset('struct dwc3_request', 'list')

        req_list_h = self.ramdump.struct_field_addr(ep, 'struct dwc3_ep', 'started_list')
        curr_req_node = self.ramdump.read_pointer(req_list_h)

        while curr_req_node != req_list_h:
            curr_req = curr_req_node - dwc3_req_list_offset
            num_trbs = self.ramdump.read_int(curr_req + dwc_req_num_trbs_offset)
            trbs = curr_req + dwc_req_trb_offset

            for i in range(num_trbs):
                curr_trb = self.ramdump.read_pointer(trbs + (i * self.ptr_size))
                if curr_trb == fault_addr:
                    return curr_req

            curr_req_node = self.ramdump.read_pointer(curr_req_node + list_head_next_offset)

        return None

    def find_crash_addr(self, eps, fault_addr):
        """
        Finds the TRB that caused an arm-smmu context fault, if possible
        Prints the address of the context fault and the address of the TRB causing it,
        or that none was found

        :param eps: Address of the device endpoints
        :param fault_addr: Address that was accessed to cause the fault
        :return: Tuple: (Endpoint addr holding trb that caused the fault,
                         TRB addr that caused the fault)"""
        # offsets
        bpl_offset = self.ramdump.field_offset('struct dwc3_trb', 'bpl')
        bph_offset = self.ramdump.field_offset('struct dwc3_trb', 'bph')
        size_offset = self.ramdump.field_offset('struct dwc3_trb', 'size')
        ctrl_offset = self.ramdump.field_offset('struct dwc3_trb', 'ctrl')

        for ep in eps:
            trb_pool = self.ramdump.read_pointer(ep + self.dwc3_ep_trb_pool_offset)
            for i in range(256):
                trb = trb_pool + (i * 16)
                bpl = self.ramdump.read_u32(trb + bpl_offset)
                bph = self.ramdump.read_u32(trb + bph_offset)
                bpl += (bph << 32)
                size = self.ramdump.read_u32(trb + size_offset)
                hwo = self.ramdump.read_byte(trb + ctrl_offset)
                if bpl <= fault_addr < bpl + size and (hwo & 0x01) == 1:
                    return ep, trb

        return None, None

    def read_xhci_ring(self, xhci_ring, name):
        """
        Returns a string containing the name of the xhci ring and the locations of its enqueue and
        dequeue pointers and its first segment

        :param xhci_ring: Address of the xhci_ring struct
        :param name: Name of the xhci_ring
        :return: String containing the details of the xhci ring
        """
        first_seg_addr = self.ramdump.struct_field_addr(xhci_ring, 'struct xhci_ring', 'first_seg')
        eq = self.ramdump.read_structure_field(xhci_ring, 'struct xhci_ring', 'enqueue')
        dq = self.ramdump.read_structure_field(xhci_ring, 'struct xhci_ring', 'dequeue')
        ring_struct = f"{name}\n\tEnqueue pointer located at: {hex(eq)}" \
                      f"\n\tDequeue pointer located at: {hex(dq)}\n" \
                      f"\tFirst segment located at: {hex(first_seg_addr)}\n"

        return ring_struct

    def print_ev_buf(self, buf, num_evts, lpos):
        """
        Prints the last num_evts events in the event buffer, interpreted as human-readable commands
        :param buf: Event Buffer address
        :param num_evts: Number of events to print
        :param lpos: Position of the last event in the event buffer
        :return: None
        """
        print(f"\nEvent Buffer {hex(buf)}, last {num_evts} events:", file=self.outfile)
        for i in reversed(range(lpos - (num_evts * 4) + 4, lpos + 4, 4)):
            evt = self.ramdump.read_u32(buf + i)
            # events can be device-specific (final 8 bits = 0x01) or ep-specific (final bit = 0)
            if evt is not None:
                s = ""
                if evt & 0xff == 1:
                    s = parse_dwc3_dvc_event(evt)
                elif evt & 1 == 0:
                    s = parse_dwc3_ep_event(evt)
                print(f"\t{hex(buf + i)}: {s}", file=self.outfile)

    def clock_is_enabled(self, phy_addr, phy_name, clk_name):
        """
        Returns whether the phy's given clock is enabled or not
        :param phy_addr: Phy address
        :param phy_name: Phy name
        :param clk_name: Clock name
        :return: "Disabled" if clk->core.enabled_count == 0, "Enabled" otherwise
        """
        clk = self.ramdump.read_pointer(self.ramdump.struct_field_addr(phy_addr, phy_name,
                                                                       clk_name))
        if clk is not None:
            core = self.ramdump.read_pointer(
                self.ramdump.struct_field_addr(clk, 'struct clk', 'core'))
            enabled = self.ramdump.read_u32(
                self.ramdump.struct_field_addr(core, 'struct clk_core', 'enabled_count'))
            return "disabled" if enabled == 0 else "enabled"

    def regulator_is_enabled(self, phy_addr, phy_name, reg_name):
        """
        Returns whether the phy's given regulator is enabled or not
        :param phy_addr: Phy address
        :param phy_name: Phy name
        :param reg_name: Regulator name
        :return: "Disabled" if regulator.enabled_count == 0, "Enabled" otherwise
        """
        reg = self.ramdump.read_pointer(self.ramdump.struct_field_addr(phy_addr, phy_name,
                                                                       reg_name))
        if reg is not None:
            enabled = self.ramdump.read_u32(
                self.ramdump.struct_field_addr(reg, 'struct regulator', 'enabled_count'))
            return "disabled" if enabled == 0 else "enabled"

    def find_device_fields(self, device, field_names):
        """
        Returns the fields of all properties in field_names
        :param device: Device
        :param field_names: Names of properties to investigate
        :return: 2D-Array containing property values in the order of their corresponding field_names
        """
        ret_fields = [[]] * len(field_names)
        of_node = self.ramdump.read_pointer(
            self.ramdump.struct_field_addr(device, 'struct device', 'of_node'))
        properties = self.ramdump.read_pointer(
            self.ramdump.struct_field_addr(of_node, 'struct device_node', 'properties'))

        while properties != 0:
            name = self.ramdump.read_cstring(self.ramdump.read_pointer(properties))

            if name in field_names:
                length = self.ramdump.read_int(
                    self.ramdump.struct_field_addr(properties, 'struct property', 'length'))
                val = self.ramdump.read_binarystring(self.ramdump.read_pointer(
                    self.ramdump.struct_field_addr(properties, 'struct property', 'value')),
                    length).decode()
                ret_fields[field_names.index(name)] = val.split('\x00')[:-1]

            properties = self.ramdump.read_pointer(
                self.ramdump.struct_field_addr(properties, 'struct property', 'next'))

        return ret_fields

    def find_usb_devs(self):
        """
        Searches usb_bus_type's klist for all usb devices
        :return: A list of all usb devices found
        """
        usb_bus = self.ramdump.read('usb_bus_type')
        usb_type = self.ramdump.read('usb_device_type')
        p = self.ramdump.read_pointer(
            self.ramdump.struct_field_addr(usb_bus, 'struct bus_type', 'p'))
        list_h = self.ramdump.struct_field_addr(self.ramdump.struct_field_addr(
            p, 'struct subsys_private', 'klist_devices'), 'struct klist', 'k_list')
        next_dv = self.ramdump.read_pointer(list_h)
        ret = []

        klist_nnode_offset = self.ramdump.field_offset('struct klist_node', 'n_node')
        dev_priv_knode_bus_offset = self.ramdump.field_offset('struct device_private', 'knode_bus')
        dev_priv_device_offset = self.ramdump.field_offset('struct device_private', 'device')
        dev_type_offset = self.ramdump.field_offset('struct device', 'type')

        while next_dv != list_h:
            knode_bus = next_dv - klist_nnode_offset
            dev_priv = knode_bus - dev_priv_knode_bus_offset
            dev = self.ramdump.read_pointer(dev_priv + dev_priv_device_offset)
            dev_type = self.ramdump.read_pointer(dev + dev_type_offset)

            if dev_type and dev_type == usb_type:
                ret.append(dev)

            next_dv = self.ramdump.read_pointer(next_dv)

        return ret

    def cleanup(self):
        """
        Closes self.outfile and writes a short message to ramparse's outfile
        :return: None
        """
        print_out.print_out_str('--- Wrote the output to ' + self.outfile_name)
        self.outfile.close()

    def search_udc_list(self, pattern):
        """
        Searches udc_list for the first device whose name matches the given regex
        :param pattern: Compiled regex
        :return: The address of the matching device, or None
        """
        usb_udc_list_offset = self.ramdump.field_offset('struct usb_udc', 'list')
        usb_udc_dev_offset = self.ramdump.field_offset('struct usb_udc', 'dev')

        list_h = self.ramdump.address_of('udc_list')
        next_dv = self.ramdump.read_pointer(list_h)
        while next_dv != list_h:
            dvc_addr = next_dv - usb_udc_list_offset
            dev = dvc_addr + usb_udc_dev_offset
            kobj = dev + self.device_kobj_offset
            name = self.ramdump.read_cstring(
                self.ramdump.read_pointer(kobj + self.kobj_name_offset))

            if name and pattern.match(name) is not None:
                return self.ramdump.read_pointer(dev + self.device_parent_offset)

            next_dv = self.ramdump.read_pointer(next_dv)

        return None

    def search_device_kset(self, name_regexes):
        """
        Search devices_kset for all devices whose names match the input regexes
        :param name_regexes: List of compiled regexes
        :return: List of the last devices found to match each regex,
                    in the same index as the matching regex
        """
        devices_kset = self.ramdump.read('devices_kset')
        list_h = self.ramdump.struct_field_addr(devices_kset, 'struct kset', 'list')
        next_dv = self.ramdump.read_pointer(list_h)
        devs = [0] * len(name_regexes)

        while next_dv != list_h and next_dv is not None:
            dvc_addr = next_dv - (self.device_kobj_offset + self.kobj_entry_offset)
            name = self.ramdump.read_cstring(self.ramdump.read_pointer(dvc_addr))

            if name:
                for i in range(len(name_regexes)):
                    if name_regexes[i].search(name) and self.ramdump.read_pointer(
                            dvc_addr + self.device_driver_data_offset) != 0:
                        devs[i] = dvc_addr

            next_dv = self.ramdump.read_pointer(next_dv)

        return devs

    def read_substream_formats(self, substream):
        """
        Given a snd_usb_substream, print all of its formats
        :param substream: Substream address
        :return: Formatted string containing substream format details
        """
        audio_fmt_list_offset = self.ramdump.field_offset('struct audioformat', 'list')
        audio_fmt_channels_offset = self.ramdump.field_offset('struct audioformat', 'channels')
        audio_fmt_frame_size_offset = self.ramdump.field_offset('struct audioformat',
                                                                'frame_size')
        audio_fmt_implicit_fb_offset = self.ramdump.field_offset('struct audioformat',
                                                                 'implicit_fb')
        audio_fmt_rate_table_offset = self.ramdump.field_offset('struct audioformat',
                                                                'rate_table')
        audio_fmt_nr_rates_offset = self.ramdump.field_offset('struct audioformat', 'nr_rates')
        audio_fmt_protocol_offset = self.ramdump.field_offset('struct audioformat', 'protocol')
        audio_fmt_formats_offset = self.ramdump.field_offset('struct audioformat', 'formats')
        audio_fmt_max_ps_offset = self.ramdump.field_offset('struct audioformat', 'maxpacksize')
        int_size = self.ramdump.sizeof('int')

        direction = self.ramdump.read_int(
            self.ramdump.struct_field_addr(substream, 'struct snd_usb_substream', 'direction'))
        fmt_list = self.ramdump.struct_field_addr(substream, 'struct snd_usb_substream', 'fmt_list')
        num_formats = self.ramdump.read_int(
            self.ramdump.struct_field_addr(substream, 'struct snd_usb_substream', 'num_formats'))
        audio_fmt = self.ramdump.read_pointer(fmt_list) - audio_fmt_list_offset

        d = "Playback" if direction == 0 else "Recording" if direction == 1 else "INVALID"

        ret = f"\tSubstream {hex(substream)}:\n" \
              f"\t\tDirection: {d}\n"
        for i in range(num_formats):
            channels = self.ramdump.read_int(audio_fmt + audio_fmt_channels_offset)
            frame_size = self.ramdump.read_int(audio_fmt + audio_fmt_frame_size_offset)
            implicit_fb = self.ramdump.read_bool(audio_fmt + audio_fmt_implicit_fb_offset)
            rate_table = audio_fmt + audio_fmt_rate_table_offset
            nr_rates = self.ramdump.read_int(audio_fmt + audio_fmt_nr_rates_offset)
            formats = self.ramdump.read_int(audio_fmt + audio_fmt_formats_offset)
            protocol = self.ramdump.read_byte(audio_fmt + audio_fmt_protocol_offset)
            maxpacksize = self.ramdump.read_int(audio_fmt + audio_fmt_max_ps_offset)
            rate_str = ""
            for j in range(nr_rates):
                rate_str += str(self.ramdump.read_int(self.ramdump.read_pointer(rate_table)
                                                      + j * int_size)) + ", "
            ret += f'\t\tFormat {i+1} ({hex(audio_fmt)}):\n' \
                   f'\t\t\tChannels: {channels}\n' \
                   f'\t\t\tFrame Size: {frame_size}\n' \
                   f'\t\t\tImplicit Feedback: {implicit_fb}\n' \
                   f'\t\t\tPossible rates: {rate_str[:-2]}\n' \
                   f'\t\t\tFormats: {bin(formats)}\n' \
                   f'\t\t\tProtocol: {hex(protocol)}\n' \
                   f'\t\t\tMax packet size: {maxpacksize}\n'
            audio_fmt = self.ramdump.read_pointer(audio_fmt)

        return ret

    def usb_audio_details(self, snd_usb_audio):
        """
        Finds the various parameters of the snd_audio driver passed in
        :param snd_usb_audio: Address of the snd_usb_audio driver
        :return: String with audio summary
        """
        pcm_list = self.ramdump.struct_field_addr(snd_usb_audio, 'struct snd_usb_audio', 'pcm_list')
        next_pcm = self.ramdump.read_pointer(pcm_list)
        ret = ""

        if next_pcm is not None:
            # offsets
            snd_stream_list_offset = self.ramdump.field_offset('struct snd_usb_stream', 'list')
            snd_stream_substream_offset = self.ramdump.field_offset('struct snd_usb_stream',
                                                                    'substream')

            visited_pcm = [pcm_list]

            while next_pcm not in visited_pcm:
                snd_stream = next_pcm - snd_stream_list_offset
                substream = snd_stream + snd_stream_substream_offset
                substream2 = substream + self.ramdump.sizeof('struct snd_usb_substream')
                ret += self.read_substream_formats(substream)
                ret += self.read_substream_formats(substream2)

                visited_pcm.append(next_pcm)
                next_pcm = self.ramdump.read_pointer(next_pcm)

        return ret

    def print_clocks_and_regs(self, addr, phy_type):
        """
        Prints the clocks and regulators associated with phy 'addr'
        :param addr: Phy address
        :param phy_type: Phy struct name ('struct {type}')
        :return:
        """
        phy = self.ramdump.read_pointer(addr + self.device_driver_data_offset)
        clk_enabled = self.ramdump.read_bool(
            self.ramdump.struct_field_addr(phy, phy_type, phy_clk_enabled_map[phy_type]))
        power_enabled = self.ramdump.read_bool(
            self.ramdump.struct_field_addr(phy, phy_type, 'power_enabled'))
        print(
            f"\t{phy_type}:\n\t\tClock Enabled: {clk_enabled}\n\t\tPower Enabled: "
            f"{power_enabled}", file=self.outfile)

        fields = self.find_device_fields(addr, ['clock-names', 'reg-names'])
        clocks, regs = fields[0], fields[1]

        for clk in clocks:
            is_enabled = self.clock_is_enabled(phy, phy_type, clk)
            if is_enabled is not None:
                print(f"\t\tClock {clk}: {is_enabled}", file=self.outfile)
        for reg in regs:
            is_enabled = self.regulator_is_enabled(phy, phy_type, reg)
            if is_enabled is not None:
                print(f"\t\tRegulator {reg}: {is_enabled}", file=self.outfile)

    def print_function_drivers(self, cdev):
        """
        Prints all configs and the associated function drivers to self.outfile
        :param cdev: usb_composite_dev associated with dwc3
        :return:
        """
        config_list_offset = self.ramdump.field_offset('struct usb_configuration', 'list')
        config_label_offset = self.ramdump.field_offset('struct usb_configuration', 'label')
        config_functions_offset = self.ramdump.field_offset('struct usb_configuration',
                                                            'functions')
        function_list_offset = self.ramdump.field_offset('struct usb_function', 'list')
        function_ep_offset = self.ramdump.field_offset('struct usb_function', 'endpoints')
        function_name_offset = self.ramdump.field_offset('struct usb_function', 'name')

        active_config = self.ramdump.read_pointer(
            self.ramdump.struct_field_addr(cdev, 'struct usb_composite_dev', 'config'))
        configs_h = self.ramdump.struct_field_addr(cdev, 'struct usb_composite_dev',
                                                   'configs')
        configs = self.ramdump.read_pointer(configs_h)

        while configs != configs_h:
            cfg = configs - config_list_offset
            label = self.ramdump.read_cstring(
                self.ramdump.read_pointer(cfg + config_label_offset))

            is_active = " (Active Cfg)" if cfg == active_config else ""
            print(f"Configuration {label}{is_active} Function Drivers:", file=self.outfile)

            functions_h = cfg + config_functions_offset
            functions = self.ramdump.read_pointer(functions_h)

            while functions != functions_h:
                fun = functions - function_list_offset
                endpoints = self.ramdump.read_pointer(fun + function_ep_offset)
                name = self.ramdump.read_cstring(
                    self.ramdump.read_pointer(fun + function_name_offset))
                print(f"{name}\n\tEndpoints: " + read_endpoint_mask(endpoints),
                      file=self.outfile)

                if gsi_set.intersection([name]):  # if this function is part of the gsi set
                    d_port = self.ramdump.struct_field_addr(fun, 'struct f_gsi', 'd_port')
                    in_ep = self.ramdump.read_pointer(
                        self.ramdump.struct_field_addr(d_port, 'struct gsi_data_port',
                                                       'in_ep'))
                    out_ep = self.ramdump.read_pointer(
                        self.ramdump.struct_field_addr(d_port, 'struct gsi_data_port',
                                                       'out_ep'))
                    active_in = self.count_active_trbs(
                        self.ramdump.read_pointer(in_ep + self.dwc3_ep_trb_pool_offset)) - 1
                    active_out = self.count_active_trbs(
                        self.ramdump.read_pointer(out_ep + self.dwc3_ep_trb_pool_offset)) - 1
                    print(f"\t{trb_status(max(active_in, 0), 'TxRing', is_tx=True)}",
                          file=self.outfile)
                    print(f"\t{trb_status(max(active_out, 0), 'RxRing')}", file=self.outfile)

                functions = self.ramdump.read_pointer(functions)

            configs = self.ramdump.read_pointer(configs)

    def get_drivers(self, interfaces, length):
        """
        For a host mode device, returns all drivers associated with its interfaces
        :param interfaces: Address of the interface list
        :param length: Number of interfaces
        :return: Names of all drivers, audio details
        """
        usb_interface_condition_offset = self.ramdump.field_offset('struct usb_interface',
                                                                   'condition')
        usb_interface_dev_offset = self.ramdump.field_offset('struct usb_interface', 'dev')
        driver_names = []
        audio = ""
        for j in range(length):
            iface = self.ramdump.read_pointer(interfaces + (j * self.ptr_size))
            condition = self.ramdump.read_byte(
                iface + usb_interface_condition_offset)
            if condition == 2:
                dev = iface + usb_interface_dev_offset
                driver = self.ramdump.read_pointer(dev + self.device_driver_offset)
                driver_data = self.ramdump.read_pointer(dev + self.device_driver_data_offset)
                name = self.ramdump.read_cstring(
                    self.ramdump.read_pointer(driver))
                if name == 'snd-usb-audio':
                    details = self.usb_audio_details(driver_data)
                    if details != "":
                        audio += f'Driver {j}:\n' + details
                if name not in driver_names:
                    driver_names.append(name)

        driver_name = "Driver(s):"
        if not driver_names:  # empty
            driver_name += " None,"
        for name in driver_names:
            driver_name += f" {name},"

        return driver_name[:-1], audio

    def find_xhci_addrs(self):
        """
        Returns all usb devices that have the xhci driver
        :return: List of all address of 'struct usb_device's whose driver == xhci_plat_hc_driver
        """
        usb_device_dev_offset = self.ramdump.field_offset('struct usb_device', 'dev')
        usb_device_parent_offset = self.ramdump.field_offset('struct usb_device', 'parent')
        usb_device_bus_offset = self.ramdump.field_offset('struct usb_device', 'bus')
        usb_hcd_bus_offset = self.ramdump.field_offset('struct usb_hcd', 'self')
        usb_hcd_driver_offset = self.ramdump.field_offset('struct usb_hcd', 'driver')
        usb_hcd_priv_offset = self.ramdump.field_offset('struct usb_hcd', 'hcd_priv')

        xhci_plat_hc_drv = self.ramdump.read('xhci_plat_hc_driver')
        xhci_addrs = []

        for dev in self.find_usb_devs():
            usb_dev = dev - usb_device_dev_offset
            parent = self.ramdump.read_pointer(usb_dev + usb_device_parent_offset)
            bus = self.ramdump.read_pointer(usb_dev + usb_device_bus_offset)
            if parent == 0:
                dev_hcd = bus - usb_hcd_bus_offset
                hv_hcd = self.ramdump.read_pointer(dev_hcd + usb_hcd_driver_offset)

                if hv_hcd == xhci_plat_hc_drv:
                    dev_xhci = dev_hcd + usb_hcd_priv_offset
                    xhci_addrs.append(dev_xhci)

        return xhci_addrs

    def print_device_endpoints(self, eps):
        """
        Prints out a summary of all endpoints given
        :param eps: List of addresses pointing to 'struct dwc3_ep's
        :return: None
        """
        ep_trb_pool_offset = self.ramdump.field_offset('struct dwc3_ep', 'trb_pool')
        ep_name_offset = self.ramdump.field_offset('struct dwc3_ep', 'name')
        ep_eq_offset = self.ramdump.field_offset('struct dwc3_ep', 'trb_enqueue')
        ep_dq_offset = self.ramdump.field_offset('struct dwc3_ep', 'trb_dequeue')

        ring_summaries = "\nActive endpoint summaries:\n"

        for ep in eps:
            trb_pool = self.ramdump.read_pointer(ep + ep_trb_pool_offset)
            name = self.ramdump.read_cstring(ep + ep_name_offset)
            trb_eq = self.ramdump.read_byte(ep + ep_eq_offset)
            trb_dq = self.ramdump.read_byte(ep + ep_dq_offset)
            ring_summaries += f"{name}:\n\tAddress: {hex(ep)}\n\tActive trbs: " \
                              f"{self.count_active_trbs(trb_pool)}\n\tEnqueue Pointer: " \
                              f"Trb {trb_eq}\n\tDequeue Pointer: Trb {trb_dq}\n"

        print(f"{ring_summaries}\n", file=self.outfile)

    def parse(self):
        """
        Main method for UsbReader. Determines which mode the ram dump is in (device, host,
        disconnected), then processes it accordingly. All outputs are written to self.outfile
        :return: None
        """
        self.outfile = self.ramdump.open_file(self.outfile_name)
        # find dwc3 for processing host mode or device mode
        # find hsphy and ssphy in case we crashed during init
        pat_hsphy = re.compile('^[0-9a-fA-F]{1,16}\\.hsphy$')
        pat_ssphy = re.compile('^[0-9a-fA-F]{1,16}\\.ssphy$')
        pat_dwc3 = re.compile('^[0-9a-fA-F]{1,16}\\.dwc3$')
        name_regexs = [pat_dwc3, pat_hsphy, pat_ssphy]

        devs = self.search_device_kset(name_regexs)
        dwc3_dev, hsphy, ssphy = devs[0], devs[1], devs[2]

        # in case we couldn't find dwc3 this way, try backups through udc_list,
        # usb_bus_type, and android_device
        if dwc3_dev == 0:
            print("dwc3 is not on the platform bus", file=self.outfile)
            if self.ramdump.read('udc_list.next') != self.ramdump.read('udc_list.prev'):
                # Device mode backup access to dwc3_msm
                print('Accessing dwc3 through udc_list\n', file=self.outfile)
                pattern = re.compile('[0-9a-fA-F]{1,16}\\.dwc3')
                dwc3_dev = self.search_udc_list(pattern)
            else:
                # Host mode backup access to dwc3_msm
                udev = self.find_usb_devs()
                if udev:  # non-empty
                    print("accessing dwc3 through usb_bus_type\n", file=self.outfile)
                    xhci = self.ramdump.read_pointer(udev[0] + self.device_parent_offset)
                    dwc3_dev = self.ramdump.read_pointer(xhci + self.device_parent_offset)
                else:
                    android_device = self.ramdump.read('android_device')
                    if android_device != 0:
                        gadget_info = self.ramdump.read_pointer(
                            android_device + self.device_driver_data_offset)
                        cdev = self.ramdump.struct_field_addr(gadget_info, 'struct gadget_info',
                                                              'cdev')
                        gadget = self.ramdump.read_pointer(
                            self.ramdump.struct_field_addr(cdev, 'struct usb_composite_dev',
                                                           'gadget'))
                        if gadget != 0:
                            print("accessing dwc3 through android_device\n", file=self.outfile)
                            dev = self.ramdump.struct_field_addr(gadget, 'struct usb_gadget', 'dev')
                            dwc3_dev = self.ramdump.read_pointer(dev + self.device_parent_offset)
                    else:
                        print("dwc3 is not enumerated. Backup access to dwc3 also failed.\n"
                              "The device likely crashed during init\n", file=self.outfile)

                        hsphy_drv = self.ramdump.read_pointer(hsphy + self.device_driver_offset)
                        hsphy_drv_name = self.ramdump.read_cstring(self.ramdump.read_pointer(
                            self.ramdump.struct_field_addr(hsphy_drv, 'struct device_driver',
                                                           'name')))
                        hsphy_type = hsphy_name_map.get(
                            hsphy_drv_name) or hsphy_drv_name  # default to drv_name

                        print(f'Device Summary:', file=self.outfile)
                        if ssphy != 0:
                            self.print_clocks_and_regs(ssphy, 'struct msm_ssphy_qmp')
                        self.print_clocks_and_regs(hsphy, f'struct {hsphy_type}')
                        print("", file=self.outfile)

                        self.cleanup()
                        return

        # derive dwc3_msm from dwc3 so that we can see what mode we're in
        dwc3_msm_dev = self.ramdump.read_pointer(dwc3_dev + self.device_parent_offset)
        dwc3 = self.ramdump.read_pointer(dwc3_dev + self.device_driver_data_offset)
        dwc3_msm = self.ramdump.read_pointer(dwc3_msm_dev + self.device_driver_data_offset)
        is_host = self.ramdump.read_bool(
            self.ramdump.struct_field_addr(dwc3_msm, 'struct dwc3_msm', 'in_host_mode'))
        is_dev = self.ramdump.read_bool(
            self.ramdump.struct_field_addr(dwc3_msm, 'struct dwc3_msm', 'in_device_mode'))

        # Device mode parsing
        if is_dev:
            print("Reading device mode crash dump\n", file=self.outfile)
            eps = self.ramdump.struct_field_addr(dwc3, 'struct dwc3', 'eps')

            # Create list of active endpoints
            active_eps = []
            for i in range(2, 32):
                ep = self.ramdump.read_pointer(eps + (i * self.ptr_size))
                if ep != 0:
                    flags = self.ramdump.read_u32(
                        self.ramdump.struct_field_addr(ep, 'struct dwc3_ep', 'flags'))
                    if (flags & 0x1) != 0:
                        active_eps.append(ep)

            # Print device summary
            connected = self.ramdump.read_byte(
                self.ramdump.struct_field_addr(dwc3, 'struct dwc3', 'connected'))
            connected = connected & connected_mask
            soft_connect = connected & softconnect_mask
            vbus_active = self.ramdump.read_bool(
                self.ramdump.struct_field_addr(dwc3_msm, 'struct dwc3_msm',
                                               'vbus_active'))
            eud_active = self.ramdump.read_bool(
                self.ramdump.struct_field_addr(dwc3_msm, 'struct dwc3_msm',
                                               'eud_active'))
            print(f"Device Summary:\n-- Active Endpoints: {len(active_eps)}\n-- Connected: "
                  f"{connected}\n-- SoftConnect: {soft_connect}\n-- VBus Active: {vbus_active}\n"
                  f"-- EUD Active: {eud_active}\n", file=self.outfile)

            # Function drivers
            gadget = self.ramdump.read_pointer(
                self.ramdump.struct_field_addr(dwc3, 'struct dwc3', 'gadget'))
            dev = self.ramdump.struct_field_addr(gadget, 'struct usb_gadget', 'dev')
            cdev = self.ramdump.read_pointer(dev + self.device_driver_data_offset)
            if cdev != 0:
                self.print_function_drivers(cdev)
                print("", file=self.outfile)
            else:
                print("Cannot display function drivers", file=self.outfile)

            if len(active_eps) != 0:
                # Device specific offsets

                # Find smmu fault and trb that caused it, if applicable
                with self.ramdump.open_file('dmesg_TZ.txt', mode='r') as f:
                    pattern = "Unhandled arm-smmu context fault from .+\\.dwc3!\\n.+FAR" \
                              "\\s+= 0x([0-9a-f]{16})"
                    match = re.search(pattern, f.read())
                    if match is not None:
                        print(f"Arm-smmu context fault detected at FAR: {match.group(1)}, "
                              f"searching for the TRB where it occurred.", file=self.outfile)
                        fault_addr = int(match.group(1), base=16)
                        ep, trb = self.find_crash_addr(active_eps, fault_addr)

                        if ep is not None and trb is not None:

                            name = self.ramdump.read_cstring(
                                self.ramdump.struct_field_addr(ep, 'struct dwc3_ep', 'name'))
                            print(f"Fault occurred in ring {name} at trb memory address "
                                  f"{self.ramdump.hexdump(trb, 16, little_endian=False)}",
                                  file=self.outfile)

                            request = self.find_faulty_request(ep, trb)
                            if request is not None:
                                print(f"Exception thrown by request {hex(request)}",
                                      file=self.outfile)

                        else:
                            print("TRB that caused the fault not found. It may have been unmapped",
                                  file=self.outfile)
                    else:
                        print("No arm-smmu fault detected", file=self.outfile)

                # Print last num_events in the dwc3 event buffer
                num_evts = 20
                ev_buf = self.ramdump.read_pointer(
                    self.ramdump.struct_field_addr(dwc3, 'struct dwc3', 'ev_buf'))
                lpos = self.ramdump.read_int(
                    self.ramdump.struct_field_addr(ev_buf, 'struct dwc3_event_buffer', 'lpos'))
                buf = self.ramdump.read_pointer(
                    self.ramdump.struct_field_addr(ev_buf, 'struct dwc3_event_buffer', 'buf'))
                self.print_ev_buf(buf, num_evts, lpos)

                # Dump endpoint summaries. usbv (verbose mode) will print rings in detail
                self.print_device_endpoints(active_eps)

            else:
                print("No active endpoints to display", file=self.outfile)

        # Host mode
        elif is_host:
            # Host specific offsets
            usb_dev_product_offset = self.ramdump.field_offset('struct usb_device', 'product')
            usb_dev_manufacturer_offset = self.ramdump.field_offset('struct usb_device',
                                                                    'manufacturer')
            usb_dev_config_offset = self.ramdump.field_offset('struct usb_device', 'config')
            usb_hconfig_interface_offset = self.ramdump.field_offset('struct usb_host_config',
                                                                     'interface')
            usb_hconfig_desc_offset = self.ramdump.field_offset('struct usb_host_config', 'desc')
            usb_cfg_desc_length_offset = self.ramdump.field_offset('struct usb_config_descriptor',
                                                                   'wTotalLength')
            xhci_cmd_ring_offset = self.ramdump.field_offset('struct xhci_hcd', 'cmd_ring')
            xhci_evt_ring_offset = self.ramdump.field_offset('struct xhci_hcd', 'event_ring')
            xhci_devs_offset = self.ramdump.field_offset('struct xhci_hcd', 'devs')
            xhci_virt_eps_offset = self.ramdump.field_offset('struct xhci_virt_device', 'eps')
            xhci_virt_udev_offset = self.ramdump.field_offset('struct xhci_virt_device', 'udev')
            xhci_virt_ep_ring_offset = self.ramdump.field_offset('struct xhci_virt_ep', 'ring')
            ep_size = self.ramdump.sizeof('struct xhci_virt_ep')

            print("Reading host mode crash dump\n", file=self.outfile)

            # Find xhci ports
            xhci_addrs = self.find_xhci_addrs()

            for xhci in xhci_addrs:
                # Command Ring and Event Ring summaries. If in verbose mode, also print the rings
                cmd_ring = self.ramdump.read_pointer(xhci + xhci_cmd_ring_offset)
                event_ring = self.ramdump.read_pointer(xhci + xhci_evt_ring_offset)
                print(f"xchi_hcd: {hex(xhci)}\n", file=self.outfile)

                if cmd_ring != 0:
                    print(self.read_xhci_ring(cmd_ring, "Command Ring") + "\n", file=self.outfile)
                else:
                    print("Command Ring is NULL", file=self.outfile)

                if event_ring != 0:
                    print(self.read_xhci_ring(event_ring, "Event Ring") + "\n", file=self.outfile)
                else:
                    print("Event Ring is NULL", file=self.outfile)

                # Dump device info. If in verbose mode, also print xhci rings
                devs = xhci + xhci_devs_offset
                for i in range(256):
                    dev = self.ramdump.read_pointer(devs + i * self.ptr_size)
                    if dev != 0:
                        eps = dev + xhci_virt_eps_offset
                        udev = self.ramdump.read_pointer(dev + xhci_virt_udev_offset)
                        product = self.ramdump.read_cstring(
                            self.ramdump.read_pointer(udev + usb_dev_product_offset))
                        manufacturer = self.ramdump.read_cstring(self.ramdump.read_pointer(
                            udev + usb_dev_manufacturer_offset))
                        config = self.ramdump.read_pointer(udev + usb_dev_config_offset)
                        desc = config + usb_hconfig_desc_offset
                        length = self.ramdump.read_u16(desc + usb_cfg_desc_length_offset)
                        interfaces = config + usb_hconfig_interface_offset

                        driver_names, audio = self.get_drivers(interfaces, length)

                        if audio != "":
                            audio = "Audio Details:\n" + audio + "\n"

                        print(f"Device {i}:\nProduct: {product}\nManufacturer: {manufacturer}\n"
                              f"{driver_names}\n{audio}", file=self.outfile)

                        for j in range(31):
                            ep = eps + j * ep_size
                            ring = self.ramdump.read_pointer(ep + xhci_virt_ep_ring_offset)
                            if ring != 0:
                                print(self.read_xhci_ring(ring, f'Endpoint {j}'), file=self.outfile)

        else:
            print("Reading disconnected mode crash dump\n", file=self.outfile)
            in_lpm = self.ramdump.struct_field_addr(dwc3_msm, 'struct dwc3_msm', 'in_lpm')
            if in_lpm == 0:
                power = self.ramdump.struct_field_addr(dwc3_dev, 'struct device', 'power')
                runtime_status = self.ramdump.read_u32(
                    self.ramdump.struct_field_addr(power, 'struct dev_pm_info', 'runtime_status'))
                usage_count = self.ramdump.read_int(
                    self.ramdump.struct_field_addr(power, 'struct dev_pm_info', 'usage_count'))
                print(f"DWC Power:\nRuntime Status: {runtime_status}\nUsage Count: {usage_count}",
                      file=self.outfile)
            else:
                print("Cable disconnected, device is in Low Power Mode", file=self.outfile)

        self.cleanup()
