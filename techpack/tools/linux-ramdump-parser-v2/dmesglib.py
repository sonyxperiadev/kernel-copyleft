# Copyright (c) 2014-2015, 2020-2021 The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import re
import string

from parser_util import cleanupString

LOG_MAGIC = 0x5d7aefca

class DmesgLib(object):

    def __init__(self, ramdump, outfile):
        self.ramdump = ramdump
        self.wrap_cnt = 0
        self.outfile = outfile
        if (self.ramdump.sizeof('struct printk_log') is None):
           self.struct_name = 'struct log'
        else:
           self.struct_name = 'struct printk_log'

    def log_from_idx(self, idx, logbuf):
        len_offset = self.ramdump.field_offset(self.struct_name, 'len')

        msg = logbuf + idx
        msg_len = self.ramdump.read_u16(msg + len_offset)
        if (msg_len == 0):
            return logbuf
        else:
            return msg

    def log_next(self, idx, logbuf):
        len_offset = self.ramdump.field_offset(self.struct_name, 'len')
        msg = idx

        msg_len = self.ramdump.read_u16(msg + len_offset)
        if (msg_len == 0):
            self.wrap_cnt += 1
            return logbuf
        else:
            return idx + msg_len

    def verify_log_helper(self, msg, verbose):
        # return early if CONFIG_LOG_BUF_MAGIC is not defined
        log_align_addr = self.ramdump.address_of('__log_align')
        if (log_align_addr is None):
            return True

        len_offset = self.ramdump.field_offset(self.struct_name, 'len')
        text_offset = self.ramdump.field_offset(self.struct_name, 'text_len')
        dict_offset = self.ramdump.field_offset(self.struct_name, 'dict_len')
        magic_offset = self.ramdump.field_offset(self.struct_name, 'magic')
        msg_len = self.ramdump.read_u16(msg + len_offset)
        text_len = self.ramdump.read_u16(msg + text_offset)
        dict_len = self.ramdump.read_u16(msg + dict_offset)
        magic = self.ramdump.read_u32(msg + magic_offset)
        log_size = self.ramdump.sizeof(self.struct_name)
        log_align = self.ramdump.read_u32(log_align_addr)
        is_logwrap_marker = not bool(text_len | msg_len | dict_len)

        err = []
        if (magic != LOG_MAGIC):
            err.append('Bad Magic')

        computed_msg_len = (text_len + dict_len + log_size + log_align - 1) & ~(log_align - 1)
        if (not is_logwrap_marker and (msg_len != computed_msg_len)):
            err.append('Bad length')

        err = ' '.join(err)
        if (err):
            if (verbose):
                f = '--------- Corrupted Dmesg {} for record @ {:x} ---------\n'.format(err, msg)
                self.outfile.write(f)
                f = self.ramdump.hexdump(msg - 0x40, 0xC0)
                self.outfile.write(f)
            return False
        return True

    def verify_log(self, msg, logbuf_addr, last_idx):
        logbuf_size = self.ramdump.sizeof('__log_buf')
        log_size = self.ramdump.sizeof(self.struct_name)

        verbose = True
        while msg != logbuf_addr + last_idx:
            if (self.verify_log_helper(msg, verbose)):
                return msg
            verbose = False
            msg = msg + 0x4
            if (msg > logbuf_addr + logbuf_size - log_size):
                msg = logbuf_addr
                self.wrap_cnt += 1

        return logbuf_addr + last_idx

    def extract_dmesg_flat(self):
        addr = self.ramdump.read_word(self.ramdump.address_of('log_buf'))
        size = self.ramdump.read_word(self.ramdump.address_of('log_buf_len'))
        dmesg = self.ramdump.read_physical(self.ramdump.virt_to_phys(addr), size)
        self.outfile.write(cleanupString(dmesg.decode('ascii', 'ignore')) + '\n')

    def extract_dmesg_binary(self):
        first_idx_addr = self.ramdump.address_of('log_first_idx')
        last_idx_addr = self.ramdump.address_of('log_next_idx')
        logbuf_addr = self.ramdump.read_word(
            self.ramdump.address_of('log_buf'))
        time_offset = self.ramdump.field_offset(self.struct_name, 'ts_nsec')
        len_offset = self.ramdump.field_offset(self.struct_name, 'len')
        text_len_offset = self.ramdump.field_offset(self.struct_name, 'text_len')
        log_size = self.ramdump.sizeof(self.struct_name)

        first_idx = self.ramdump.read_u32(first_idx_addr)
        if self.ramdump.is_config_defined('CONFIG_PRINTK_CALLER'):
            callerid_off = self.ramdump.field_offset(self.struct_name, 'caller_id')
        last_idx = self.ramdump.read_u32(last_idx_addr)

        curr_idx = logbuf_addr + first_idx

        while curr_idx != logbuf_addr + last_idx and self.wrap_cnt < 2:
            timestamp = self.ramdump.read_dword(curr_idx + time_offset)
            if self.ramdump.is_config_defined('CONFIG_PRINTK_CALLER'):
                caller_data = self.ramdump.read_u32(curr_idx + callerid_off)
                tid_info = "T"
                if (caller_data & 0x80000000):
                    tid_info = "C"
                caller_id_data = caller_data & ~0x80000000
                caller_id_data = tid_info + str(caller_id_data)
            text_len = self.ramdump.read_u16(curr_idx + text_len_offset)
            text_str = self.ramdump.read_cstring(curr_idx + log_size, text_len)
            if text_str is not None:
                for partial in text_str.split('\n'):
                    if self.ramdump.is_config_defined('CONFIG_PRINTK_CALLER'):
                        f = '[{0:>5}.{1:0>6d}] [{caller_id_data:>6}] {2}\n'.format(
                        timestamp // 1000000000, (timestamp % 1000000000) // 1000, partial, caller_id_data=caller_id_data)
                    else:
                        f = '[{0:>5}.{1:0>6d}] {2}\n'.format(
                            timestamp // 1000000000, (timestamp % 1000000000) // 1000, partial)
                    self.outfile.write(f)
                curr_idx = self.log_next(curr_idx, logbuf_addr)
                curr_idx = self.verify_log(curr_idx, logbuf_addr, last_idx)
            else:
                self.outfile.write("[ Log wraps around ] at {0} \n".format(hex(curr_idx)))
                curr_idx = logbuf_addr
                self.wrap_cnt += 1

    def extract_lockless_dmesg(self):
        prb_addr = self.ramdump.read_pointer('prb')
        off = self.ramdump.field_offset('struct printk_ringbuffer', 'desc_ring')
        desc_ring_addr = prb_addr + off

        off = self.ramdump.field_offset('struct prb_desc_ring', 'count_bits')
        desc_ring_count = 1 << self.ramdump.read_u32(desc_ring_addr + off)
        desc_sz = self.ramdump.sizeof('struct prb_desc')
        off = self.ramdump.field_offset('struct prb_desc_ring', 'descs')
        descs_addr = self.ramdump.read_ulong(desc_ring_addr + off)

        info_sz = self.ramdump.sizeof('struct printk_info')
        off = self.ramdump.field_offset('struct prb_desc_ring', 'infos')
        infos_addr = self.ramdump.read_ulong(desc_ring_addr + off)

        off = self.ramdump.field_offset(
                                'struct printk_ringbuffer', 'text_data_ring')
        text_data_ring_addr = prb_addr + off

        off = self.ramdump.field_offset('struct prb_data_ring', 'size_bits')
        text_data_sz = 1 << self.ramdump.read_u32(text_data_ring_addr + off)
        off = self.ramdump.field_offset('struct prb_data_ring', 'data')
        data_addr = self.ramdump.read_ulong(text_data_ring_addr + off)

        sv_off = self.ramdump.field_offset('struct prb_desc', 'state_var')
        off = self.ramdump.field_offset('struct prb_desc','text_blk_lpos')
        begin_off = off + self.ramdump.field_offset(
                                        'struct prb_data_blk_lpos', 'begin')
        next_off = off + self.ramdump.field_offset(
                                        'struct prb_data_blk_lpos', 'next')
        ts_off = self.ramdump.field_offset('struct printk_info', 'ts_nsec')
        callerid_off = self.ramdump.field_offset('struct printk_info', 'caller_id')
        len_off = self.ramdump.field_offset('struct printk_info', 'text_len')

        desc_committed = 1
        desc_finalized = 2
        desc_sv_bits = self.ramdump.sizeof('long') * 8
        desc_flags_shift = desc_sv_bits - 2
        desc_flags_mask = 3 << desc_flags_shift
        desc_id_mask = ~desc_flags_mask

        off = self.ramdump.field_offset('struct prb_desc_ring','tail_id')
        tail_id = self.ramdump.read_u64(desc_ring_addr + off)
        off = self.ramdump.field_offset('struct prb_desc_ring','head_id')
        head_id = self.ramdump.read_u64(desc_ring_addr + off)

        did = tail_id
        while True:
            ind = did % desc_ring_count
            desc_off = desc_sz * ind
            info_off = info_sz * ind

            # skip non-committed record
            state = 3 & (self.ramdump.read_u64(descs_addr + desc_off +
                                            sv_off) >> desc_flags_shift)
            if state != desc_committed and state != desc_finalized:
                if did == head_id:
                    break
                did = (did + 1) & desc_id_mask
                continue

            begin = self.ramdump.read_ulong(descs_addr + desc_off +
                                                begin_off) % text_data_sz
            end = self.ramdump.read_ulong(descs_addr + desc_off +
                                                next_off) % text_data_sz

            if begin & 1 == 1:
                text = ""
            else:
                if begin > end:
                    begin = 0

                text_start = begin + self.ramdump.sizeof('long')
                text_len = self.ramdump.read_u16(infos_addr +
                                                    info_off + len_off)

                if end - text_start < text_len:
                    text_len = end - text_start
                if text_len < 0:
                    text_len = 0

                text = self.ramdump.read_cstring(data_addr +
                                                    text_start, text_len)

            time_stamp = self.ramdump.read_u64(infos_addr +
                                                    info_off + ts_off)
            caller_data = self.ramdump.read_u32(infos_addr +
                                                    info_off + callerid_off)
            tid_info = "T"
            if (caller_data & 0x80000000):
                tid_info = "C"

            caller_id_data = caller_data & ~0x80000000
            caller_id_data = tid_info + str(caller_id_data)
            for line in text.splitlines():
                msg = u"[{time:12.6f}][{caller_id_data:>6}] {line}\n".format(
                    time=time_stamp / 1000000000.0,
                    line=line,caller_id_data=caller_id_data)
                self.outfile.write(msg)

            if did == head_id:
                break
            did = (did + 1) & desc_id_mask

    def extract_dmesg(self):
        major, minor, patch = self.ramdump.kernel_version
        if (major, minor) >= (5, 10):
            self.extract_lockless_dmesg()
            return
        if (major, minor) >= (3, 7):
            self.extract_dmesg_binary()
            return
        self.extract_dmesg_flat()
