# Copyright (c) 2017, 2020 The Linux Foundation. All rights reserved.
# Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import struct
import os
from print_out import print_out_str
from ramparse import VERSION

class FCM_Dump():

    def __init__(self, start, end):
        self.start_addr = start
        self.end_addr = end

    def dump_fcm_img(self,ram_dump):
        if self.start_addr >= self.end_addr:
            return False
        rsz = self.end_addr - self.start_addr
        fcmfile = ram_dump.open_file('fcm.bin', mode='wb')
        fcm_data= ram_dump.read_physical(self.start_addr, rsz)
        fcmfile.write(fcm_data)
        fcmfile.close()
