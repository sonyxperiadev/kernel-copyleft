# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.

from print_out import print_out_str
from parser_util import RamParser, cleanupString, register_parser
import linux_list as llist
from struct_print import struct_print_class
import parsers.linux_devices as ldevices

@register_parser('--print-svmdmesg', 'Print the svmdmesg')
class svmdmesg(RamParser):

    def __init__(self, *args):
        super(svmdmesg, self).__init__(*args)

    def get_svm_device(self, fout):
        device = ldevices.DevicesList(self.ramdump)
        self.device_lists = device.get_device_list()
        for item in self.device_lists:
            name = item[1]
            device_item = item[0]
            if name == None:
                name = 'n/a'
            if 'soc:dmesg-dump' in name:
                drvdata = self.ramdump.read_structure_field(device_item, 'struct device', 'driver_data')
                if drvdata != 0:
                    qcom_dmesg_dumper_address = drvdata
                    qcom_dmesg_dumper = self.ramdump.read_datatype(qcom_dmesg_dumper_address, 'struct qcom_dmesg_dumper')
                    base = qcom_dmesg_dumper.base
                    size = qcom_dmesg_dumper.size
                    log = self.ramdump.get_bin_data(base, size)
                    fout.write(cleanupString(log.decode('ascii', 'ignore')))
                break
    def parse(self):
        self.f = self.ramdump.open_file("svmdmesg.txt")
        self.get_svm_device(self.f)
        self.f.close()
