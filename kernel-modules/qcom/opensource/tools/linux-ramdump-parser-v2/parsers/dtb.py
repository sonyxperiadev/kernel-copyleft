"""
Copyright (c) 2020 The Linux Foundation. All rights reserved.
SPDX-License-Identifier: GPL-2.0-only
Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License version 2 and
only version 2 as published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

"""

import os,sys
import struct
import subprocess
from print_out import print_out_str
from parser_util import register_parser, RamParser
import local_settings

@register_parser('--dtb', 'Dump the devicetree blob information')
class dtb_parsing(RamParser):

    def __init__(self, *args):
        super(dtb_parsing, self).__init__(*args)

    def dtb_parse(self, ram_dump):
        initial_boot_params_addr = ram_dump.address_of('initial_boot_params')
        if initial_boot_params_addr:
            initial_boot_params = ram_dump.read_u64(initial_boot_params_addr)
            magic = ram_dump.read_u32(initial_boot_params)
            if (magic == 0xEDFE0DD0):
                db_size = ram_dump.read_u64(initial_boot_params + 0x4)
                dtbsize=((db_size&0xFF)<<24)|((db_size&0xFF00)<<8)|((db_size&0xFF0000)>>8)|((db_size&0xFF000000)>>24)
                dtb_file = "devicetree.dtb"
                dtb_path = os.path.join(ram_dump.outdir,dtb_file)
                dtb_fd = open(dtb_path,'wb')
                dtb_data = ram_dump.read_physical(ram_dump.virt_to_phys(initial_boot_params),dtbsize)
                dtb_fd.write(dtb_data)
                dtb_fd.close()


    def parse(self):
        self.dtb_parse(self.ramdump)
        dts_output_file = "{0}/{1}".format(self.ramdump.outdir, "dts.txt")
        with open(dts_output_file, 'w') as dts_out:
            devicetree_dtb =  os.path.join(self.ramdump.outdir, "devicetree.dtb")
            if os.path.exists(devicetree_dtb):
                try:
                    retcode = subprocess.Popen([local_settings.dtc_path, '-f', '-I', 'dtb', '-O', 'dts', devicetree_dtb], stdout=dts_out, stderr=dts_out, shell=False)
                except OSError as e:
                    print_out_str("exception is {0} dtc used {1}".format(str(e), local_settings.dtc_path))