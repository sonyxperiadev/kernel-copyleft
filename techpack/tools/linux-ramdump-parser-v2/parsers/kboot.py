# Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from parser_util import register_parser, RamParser


@register_parser('--kbootlog', 'Print the kernel boot log', shortopt='-k')
class KBootLog(RamParser):

    def __init__(self, *args):
        super(KBootLog, self).__init__(*args)
        self.wrap_cnt = 0
        self.outfile = self.ramdump.open_file('kernel_boot_log.txt')
        if (self.ramdump.sizeof('struct printk_log') is None):
            self.struct_name = 'struct log'
        else:
            self.struct_name = 'struct printk_log'

    def parse(self):
        if self.ramdump.kernel_version >= (5, 10):
            self.extract_kernel_boot_log()
        else:
            self.extract_kboot_log()

    def log_next(self, idx, logbuf):
        len_offset = self.ramdump.field_offset(self.struct_name, 'len')
        msg = idx

        msg_len = self.ramdump.read_u16(msg + len_offset)
        if (msg_len == 0):
            self.wrap_cnt += 1
            return logbuf
        else:
            return idx + msg_len

    def extract_kernel_boot_log(self):
        logbuf_addr = self.ramdump.read_word(self.ramdump.address_of(
                                     'boot_log_buf'))
        logbuf_size = self.ramdump.read_u32("boot_log_buf_size")
        if logbuf_size is None:
            logbuf_size = 524288
        if logbuf_addr:
            data = self.ramdump.read_cstring(logbuf_addr, logbuf_size)
            self.outfile.write(data)
        else:
            self.outfile.write("kernel boot log support is not present\n")
            return

    def extract_kboot_log(self):
        last_idx_addr = self.ramdump.address_of('log_next_idx')
        logbuf_addr = None
        if self.ramdump.kernel_version >= (5, 4):
            logbuf_addr = self.ramdump.read_word(self.ramdump.address_of(
                                                'boot_log_buf'))
        elif self.ramdump.kernel_version >= (4, 19):
            logbuf_addr = self.ramdump.read_word(self.ramdump.address_of(
                                                'init_log_buf'))
        if logbuf_addr is None:
            self.outfile.write("kernel boot log support is not present\n")
            return
        time_offset = self.ramdump.field_offset(self.struct_name, 'ts_nsec')
        text_len_offset = self.ramdump.field_offset(self.struct_name,
                                                    'text_len')
        log_size = self.ramdump.sizeof(self.struct_name)
        first_idx = 0
        last_idx = self.ramdump.read_u32(last_idx_addr)

        curr_idx = logbuf_addr + first_idx

        while curr_idx != logbuf_addr + last_idx and self.wrap_cnt < 1:
            timestamp = self.ramdump.read_dword(curr_idx + time_offset)
            text_len = self.ramdump.read_u16(curr_idx + text_len_offset)
            text_str = self.ramdump.read_cstring(curr_idx + log_size, text_len)
            for partial in text_str.split('\n'):
                if text_len == 0:
                    break
                f = '[{0:>5}.{1:0>6d}] {2}\n'.format(
                    timestamp // 1000000000, (timestamp % 1000000000) //
                    1000, partial)
                self.outfile.write(f)
            curr_idx = self.log_next(curr_idx, logbuf_addr)
