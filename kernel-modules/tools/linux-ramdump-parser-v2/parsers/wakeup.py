"""
Copyright (c) 2020 The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""

import os,sys
import struct

from print_out import print_out_str
from parser_util import register_parser, RamParser


wakeup_log = "wakeup_resource.txt"


def setup_out_file(path, self):
    global out_file
    try:
        out_file = self.ramdump.open_file(path, 'wb')
        return out_file
    except:
        print_out_str("could not open path {0}".format(path))
        print_out_str("Do you have write/read permissions on the path?")


def print_out_ip(string):
    out_file.write((string + '\n').encode('ascii', 'ignore'))


@register_parser('--wakeup', 'print wakelock wakeup resource information')
class wakeup_logging(RamParser):

    def __init__(self, *args):
        super(wakeup_logging, self).__init__(*args)

    def wakeup_parse(self, ram_dump):
        curr_chan=0
        ep_idx=0
        wakeup_sources_addr = ram_dump.address_of('wakeup_sources')
        next = ram_dump.read_u64(wakeup_sources_addr)
        entry_offset = ram_dump.field_offset('struct wakeup_source','entry')
        name_offset = ram_dump.field_offset('struct wakeup_source','name')
        active_count_offset = ram_dump.field_offset('struct wakeup_source','active_count')
        event_count_offset = ram_dump.field_offset('struct wakeup_source','event_count')
        wakeup_count_offset = ram_dump.field_offset('struct wakeup_source','wakeup_count')
        expire_count_offset = ram_dump.field_offset('struct wakeup_source','expire_count')
        active_offset = ram_dump.field_offset('struct wakeup_source','active')
        total_time_offset = ram_dump.field_offset('struct wakeup_source','total_time')
        last_time_offset = ram_dump.field_offset('struct wakeup_source','last_time')
        start_prevent_time_offset = ram_dump.field_offset('struct wakeup_source','start_prevent_time')
        prevent_sleep_time_offset = ram_dump.field_offset('struct wakeup_source','prevent_sleep_time')   
        header_str = ('{0:11} {1:10} {2:10} {3:13} {4:14} {5:11} {6:13} {7:16} {8:16} {9:10}'
                    .format('active_count', 'event_count', 'wakeup_count',
                        'expire_count', 'total_time', 'last_time', 'start_prevent_time', 'prevent_suspend_time', 'active', 'name'))
        print_out_ip(header_str)
        while wakeup_sources_addr != next:
            idx = 0
            next = next - entry_offset
            name_addr = ram_dump.read_u64(next + name_offset)
            name = ram_dump.read_cstring(name_addr)
            active_count = ram_dump.read_u64(next + active_count_offset)
            event_count = ram_dump.read_u64(next + event_count_offset)
            wakeup_count = ram_dump.read_u64(next + wakeup_count_offset)
            expire_count = ram_dump.read_u64(next + expire_count_offset)
            active = ram_dump.read_u64(next + active_offset)
            total_time_addr = ram_dump.read_s64(next + total_time_offset)
            last_time_addr = ram_dump.read_s64(next + last_time_offset)
            start_prevent_time_addr = ram_dump.read_s64(next + start_prevent_time_offset)
            prevent_sleep_time_addr = ram_dump.read_s64(next + prevent_sleep_time_offset)
            
            
            total_time0 = total_time_addr
            total_time1 = ram_dump.read_u32(next + total_time_offset +0x4)
            total_time1= (total_time1<<0x20)
            total_time = (total_time0 | total_time1)


            last_time0 = last_time_addr
            last_time1 = ram_dump.read_u32(next + last_time_offset +0x4)
            last_time1 = (last_time1<<0x20)
            last_time = (last_time0 | last_time1)

            start_prevent_time0 = start_prevent_time_addr
            start_prevent_time1 = ram_dump.read_u32(next + start_prevent_time_offset +0x4)
            start_prevent_time1= (start_prevent_time1<<0x20)
            start_prevent_time = (start_prevent_time0 | start_prevent_time1)

            prevent_sleep_time0 = prevent_sleep_time_addr
            prevent_sleep_time1 = ram_dump.read_u32(next + prevent_sleep_time_offset +0x4)
            prevent_sleep_time1= (prevent_sleep_time1<<0x20)
            prevent_sleep_time = (prevent_sleep_time0 | prevent_sleep_time1)

            data_str = ('{0:11} {1:10} {2:10} {3:13} {4:14} {5:11} {6:13} {7:16} {8:13} \t\t{9:10}'
                    .format(int(active_count), int(event_count), int(wakeup_count),
                        int(expire_count), total_time, last_time, start_prevent_time, prevent_sleep_time, active, name))
            print_out_ip(data_str)
            next = ram_dump.read_u64(next + entry_offset)


    def parse(self):
        setup_out_file(wakeup_log, self)
        self.wakeup_parse(self.ramdump)
