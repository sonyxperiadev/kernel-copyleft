# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.

from parser_util import register_parser, RamParser, cleanupString
from print_out import print_out_str
import linux_list as llist
import parsers.linux_devices as ldevices

@register_parser('--parse-device-link', 'Parse device link')
class DeviceLink(RamParser):
    def __init__(self, *args):
        super(DeviceLink, self).__init__(*args)
        self.device_lists = []
        self.tips = """================================================================================
device_link_state:
DL_STATE_NONE: The presence of the drivers is not being tracked.
DL_STATE_DORMANT: None of the supplier/consumer drivers is present.
DL_STATE_AVAILABLE: The supplier driver is present, but the consumer is not.
DL_STATE_CONSUMER_PROBE: The consumer is probing (supplier driver present).
DL_STATE_ACTIVE: Both the supplier and consumer drivers are present.
DL_STATE_SUPPLIER_UNBIND: The supplier driver is unbinding.

dl_dev_state:
DL_DEV_NO_DRIVER: There is no driver attached to the device.
DL_DEV_PROBING: A driver is probing.
DL_DEV_DRIVER_BOUND: The driver has been bound to the device.
DL_DEV_UNBINDING: The driver is unbinding from the device.
================================================================================
"""
        self.device_link_state  = (
            "DL_STATE_NONE",
            "DL_STATE_DORMANT",
            "DL_STATE_AVAILABLE",
            "DL_STATE_CONSUMER_PROBE",
            "DL_STATE_ACTIVE",
            "DL_STATE_SUPPLIER_UNBIND",
        )
        self.dl_dev_state = (
            "DL_DEV_NO_DRIVER",
            "DL_DEV_PROBING",
            "DL_DEV_DRIVER_BOUND",
            "DL_DEV_UNBINDING",
        )

    def parse(self):
        if self.ramdump.kernel_version < (5, 4, 0):
            print_out_str('device link is not supported.')
            return

        fout = self.ramdump.open_file("device_link.txt")
        fout.write(self.tips)
        device = ldevices.DevicesList(self.ramdump)
        self.device_lists = device.get_device_list()
        self.device_lists = sorted(self.device_lists, key=lambda l: l[2] if l is None else l, reverse=True)

        device_kobj_offset  = self.ramdump.field_offset('struct device', 'kobj')
        device_links_offset = self.ramdump.field_offset('struct device', 'links')
        dev_links_info_consumers_offset = self.ramdump.field_offset('struct dev_links_info', 'consumers')
        dev_links_info_suppliers_offset = self.ramdump.field_offset('struct dev_links_info', 'suppliers')
        dev_links_info_status_offset    = self.ramdump.field_offset('struct dev_links_info', 'status')
        device_link_s_node_offset   = self.ramdump.field_offset('struct device_link', 's_node')
        device_link_c_node_offset   = self.ramdump.field_offset('struct device_link', 'c_node')
        device_link_status_offset   = self.ramdump.field_offset('struct device_link', 'status')
        device_link_consumer_offset = self.ramdump.field_offset('struct device_link', 'consumer')
        device_link_supplier_offset = self.ramdump.field_offset('struct device_link', 'supplier')
        kobject_name_offset = self.ramdump.field_offset('struct kobject', 'name')
        for item in self.device_lists:
            if item[1] is None:
                continue

            links = item[0] + device_links_offset
            #consumer
            consumers = links + dev_links_info_consumers_offset
            consumers_list = llist.ListWalker(self.ramdump, consumers, device_link_s_node_offset)
            consumer_summary = ""
            for i in consumers_list:
                device_link = i
                device_link_status = self.ramdump.read_s32(device_link + device_link_status_offset)
                # ignore non-tracked device
                if device_link_status == -1:
                    continue
                consumers = self.ramdump.read_pointer(device_link + device_link_consumer_offset)
                if consumers is None:
                    continue
                kobj = self.ramdump.read_word(consumers + device_kobj_offset)
                name = cleanupString(self.ramdump.read_cstring(kobj + kobject_name_offset, 128))
                dev_status = self.ramdump.read_s32(consumers + device_links_offset + dev_links_info_status_offset)
                # highlight the consumer when the driver is not bound.
                if dev_status != 2:
                    consumer_summary += ">>>>[consumer] link_status: {:<24} dev_status: {:<20} device: {}\n".format(
                                        self.device_link_state[device_link_status + 1],
                                        self.dl_dev_state[dev_status], name)
                else:
                    consumer_summary += "    [consumer] link_status: {:<24} dev_status: {:<20} device: {}\n".format(
                                        self.device_link_state[device_link_status + 1],
                                        self.dl_dev_state[dev_status], name)

            # suppliers
            suppliers = links + dev_links_info_suppliers_offset
            suppliers_list = llist.ListWalker(self.ramdump, suppliers, device_link_c_node_offset)
            supplier_summary = ""
            for i in suppliers_list:
                device_link = i
                device_link_status = self.ramdump.read_s32(device_link + device_link_status_offset)
                # ignore non-tracked device
                if device_link_status == -1:
                    continue
                supplier = self.ramdump.read_pointer(device_link + device_link_supplier_offset)
                if supplier is None:
                    continue
                kobj = self.ramdump.read_word(supplier + device_kobj_offset)
                name = cleanupString(self.ramdump.read_cstring(kobj + kobject_name_offset, 128))
                dev_status = self.ramdump.read_s32(supplier + device_links_offset + dev_links_info_status_offset)
                # highlight the supplier when the driver is not bound.
                if dev_status != 2:
                    supplier_summary += ">>>>[supplier] link_status: {:<24} dev_status: {:<20} device: {}\n".format(
                                        self.device_link_state[device_link_status + 1],
                                        self.dl_dev_state[dev_status], name)
                else:
                    supplier_summary += "    [supplier] link_status: {:<24} dev_status: {:<20} device: {}\n".format(
                                        self.device_link_state[device_link_status + 1],
                                        self.dl_dev_state[dev_status], name)

            if len(consumer_summary) != 0 or len(supplier_summary) != 0:
                fout.write ("\ndevice: {:<32}\n".format(item[1]))
                fout.write(consumer_summary)
                fout.write(supplier_summary)
