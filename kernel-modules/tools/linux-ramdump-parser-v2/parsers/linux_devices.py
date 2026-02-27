# Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from parser_util import register_parser, RamParser, cleanupString
import linux_list as llist

@register_parser('--print-devices', 'Print devices info')
class DevicesList(RamParser):

    def __init__(self, ramdump):
        self.ramdump = ramdump

        self.kobj_offset = self.ramdump.field_offset('struct device', 'kobj')
        self.driver_data_offset = self.ramdump.field_offset('struct device', 'driver_data')
        self.dev_busname_offset = self.ramdump.field_offset('struct device', 'bus')
        self.dma_ops_offset = self.ramdump.field_offset('struct device', 'dma_ops')

        self.entry_offset = self.ramdump.field_offset('struct kobject', 'entry')
        self.name_offset = self.ramdump.field_offset('struct kobject', 'name')

    def list_func(self, device):
        kobj = self.ramdump.read_word(device + self.kobj_offset)
        name = cleanupString(
            self.ramdump.read_cstring(kobj + self.name_offset, 128))
        if name == None or name == "":
            return
        driver_data = self.ramdump.read_word(device + self.driver_data_offset)
        bus_name_addr = self.ramdump.read_word(device + self.dev_busname_offset)
        bus_name = self.ramdump.read_cstring(self.ramdump.read_word(bus_name_addr))
        dma_ops = self.ramdump.read_structure_field(device, 'struct device', 'dma_ops')
        cma_area = self.ramdump.read_structure_field(device, 'struct device', 'cma_area')
        cma_name = ''
        if cma_area != 0:
            cma_name_addr_offset = self.ramdump.field_offset('struct cma', 'name')
            cma_name = self.ramdump.read_cstring(cma_area + cma_name_addr_offset, 48)
        a_ops_name = self.ramdump.unwind_lookup(dma_ops)
        dma_ops_name = ''
        if a_ops_name is not None:
            dma_ops_name, a = a_ops_name
        a_ops_string = '[{0}]'.format(dma_ops_name)
        print("0x%x   %-100s %-16s 0x%-32x 0x%-16x %s %-48s"
              % (device, name, bus_name, driver_data, cma_area, cma_name, a_ops_string, ), file=self.f)

    def get_device_list(self):
        devices_kset_addr = self.ramdump.address_of('devices_kset')
        list_head = devices_kset_addr
        list_offset = self.kobj_offset + self.entry_offset
        list_walker = llist.ListWalker(self.ramdump, list_head, list_offset)
        list_walker.walk(list_head, self.list_func)

    def parse(self):
        self.f = open(self.ramdump.outdir + "/devices.txt", "w")
        print("v.v (struct device)  name                                                                                                 bus_name         driver_data                       v.v (struct cma)    dma_ops \n", file=self.f)
        self.get_device_list()
        self.f.close()
