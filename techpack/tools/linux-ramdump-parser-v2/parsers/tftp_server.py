# Copyright (c) 2013-2018,2021, The Linux Foundation. All rights reserved.
#
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import os
import ctypes
import struct
import sys
import pdb

from parser_util import register_parser, RamParser, cleanupString
from mmu import Armv8MMU, Armv7MMU
from print_out import print_out_str
from linux_list import ListWalker
from ramdump import Struct
from struct_print import struct_print_class

class tftp_hash_defines:
    dbg_evt_array_size = 100
    pkt_array_size = 50
    pkt_payload_size = 50

class tftp_connection_debug_id:
    invalid = 0
    open = 1
    tx = 2
    rx = 3
    get_new_addr = 4
    connect = 5
    close = 6
    set_tx_timeout = 7
    max = 8

class tftp_connection_debug_id_string:
    str = dict([
    (0, "invalid"),
    (1, "open"),
    (2, "tx"),
    (3, "rx"),
    (4, "get_new_addr"),
    (5, "connect"),
    (6, "close"),
    (7, "set_tx_timeout"),
    (8, "max")])

class tftp_sock_result:
    success = 0
    failed = 1
    timeout = 2
    hangup = 3

class tftp_sock_result_string:
    str = dict([
    (0, "success"),
    (1, "failed"),
    (2, "timeout"),
    (3, "hangup"),])

class tftp_pkt_opcode_string:
    str = dict([
    (0, "unknown"),
    (1, "rrq"),
    (2, "wrq"),
    (3, "data"),
    (4, "ack"),
    (5, "error"),
    (6, "oack"),
    (7, "max")])

class tftp_sock_addr(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('pad_1', ctypes.c_uint8 * 32),
    ]

class tftp_pkt_debug_info(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('id', ctypes.c_uint32),
        ('seq_no', ctypes.c_uint32),
        ('time_delta', ctypes.c_uint64),
        ('sd', ctypes.c_uint32),
        ('local_addr', tftp_sock_addr),
        ('remote_addr', tftp_sock_addr),
        ('new_remote_addr', tftp_sock_addr),
        ('pkt_payload', ctypes.c_uint8 * tftp_hash_defines.pkt_payload_size),
        ('sock_res', ctypes.c_uint32),
        ('err_num', ctypes.c_uint32)
    ]

class tftp_events_debug_info(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('id', ctypes.c_uint32),
        ('seq_no', ctypes.c_uint32),
        ('time_delta', ctypes.c_uint64),
        ('sd', ctypes.c_uint32),
        ('sock_res', ctypes.c_uint32),
        ('err_num', ctypes.c_uint32)
    ]

class tftp_connection_info(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('bootup_timetick', ctypes.c_uint64),
        ('seq_no', ctypes.c_uint32),
        ('dbg_evt', tftp_events_debug_info * tftp_hash_defines.dbg_evt_array_size),
        ('dbg_evt_idx', ctypes.c_uint32),
        ('tx_pkts', tftp_pkt_debug_info * tftp_hash_defines.pkt_array_size),
        ('tx_evt_idx', ctypes.c_uint32),
        ('rx_pkts', tftp_pkt_debug_info * tftp_hash_defines.pkt_array_size),
        ('rx_evt_idx', ctypes.c_uint32)
    ]

class tftp_debug_item(object):
    def __init__(self):
        self.seqno = None
        self.time_delta = None
        self.adj_time_in_ms = None
        self.sd = None
        self.event = None
        self.sock_res = None
        self.err_num = None
        self.payload = None

class tftp_debug(object):
    def __init__(self):
        self.out_fd = None
        self.raw_buf = None
        self.tftpd = None
        self.dbg_list = {}
        self.sanity_checks()

    def print_usage(self):
        print("usage:", file=self.out_fd)
        print("tftp_debug.py <filename>", file=self.out_fd)

    def sanity_checks(self):
        assert(ctypes.sizeof(tftp_events_debug_info) == 28)
        assert(ctypes.sizeof(tftp_sock_addr) == 32)
        assert(ctypes.sizeof(tftp_pkt_debug_info) == 174)
        assert(ctypes.sizeof(tftp_connection_info) == 20224)

    def process_dbg_array(self):
        print("", file=self.out_fd)
        print("Events list:", file=self.out_fd)
        print("-" * 85, file=self.out_fd)
        line_str = "{0:<4s} | {1:<16s} | {2:<6s} | {3:<12s} | {4:<12s} | {5:<10s} | {6:<8s}".format(
        "idx", "time", "seqno", "sd", "event", "result", "errno")
        print(line_str, file=self.out_fd)
        print("-" * 85, file=self.out_fd)
        for i in range(tftp_hash_defines.dbg_evt_array_size):
            dbg_evt = self.tftpd.dbg_evt[i]
            if dbg_evt.id == 0:
                continue
            assert(dbg_evt.id > tftp_connection_debug_id.invalid and dbg_evt.id < tftp_connection_debug_id.max)
            evt_id_str = tftp_connection_debug_id_string.str[dbg_evt.id]
            sock_res_str = tftp_sock_result_string.str[dbg_evt.sock_res]
            line_str = "{0:<4d} | {1:<16d} | {2:<6d} | {3:<12d} | {4:<12s} | {5:<10s} | {6:<8d}".format(
            i, dbg_evt.time_delta, dbg_evt.seq_no, dbg_evt.sd, evt_id_str, sock_res_str, dbg_evt.err_num)
            print(line_str, file=self.out_fd)

            dbg_item = tftp_debug_item()
            dbg_item.seqno = dbg_evt.seq_no
            dbg_item.time_delta = dbg_evt.time_delta
            dbg_item.sd = dbg_evt.sd
            dbg_item.event = evt_id_str
            dbg_item.sock_res = sock_res_str
            dbg_item.err_num = dbg_evt.err_num
            self.dbg_list[dbg_evt.seq_no] = dbg_item
        print("-" * 80, file=self.out_fd)

    def process_pkt_array(self, pkt_array, name_str, event_id):
        print('', file=self.out_fd)
        print(name_str, file=self.out_fd)
        print("-" * 110, file=self.out_fd)
        line_str = "{0:<4s} | {1:<16s} | {2:<6s} | {3:<12s} | {4:<10s} | {5:<8s} | {6:<8s}".format(
        "idx", "time", "seqno", "sd", "result", "errno", "payload")
        print(line_str, file=self.out_fd)
        print("-" * 110, file=self.out_fd)
        for i in range(tftp_hash_defines.pkt_array_size):
            pkt_evt = pkt_array[i]
            if pkt_evt.id == 0:
                continue
            assert(pkt_evt.id == tftp_connection_debug_id.tx or pkt_evt.id == tftp_connection_debug_id.rx)
            assert(pkt_evt.id == event_id)
            sock_res_str = tftp_sock_result_string.str[pkt_evt.sock_res]

            line_str = "{0:<4d} | {1:<16d} | {2:<6d} | {3:<12d} | {4:<10s} | {5:<8d} | ".format(
            i, pkt_evt.time_delta, pkt_evt.seq_no, pkt_evt.sd, sock_res_str, pkt_evt.err_num)

            payload_str = ""
            pkt = pkt_evt.pkt_payload
            if sys.version_info > (3,):
                mv = memoryview(pkt)
                pkt_buf = mv.tobytes()
            else:
                pkt_buf = buffer(pkt)[:]
            (opcode,) = struct.unpack('>H', pkt_buf[:2])
            opcode_str = tftp_pkt_opcode_string.str[opcode]
            payload_str += opcode_str
            payload_str += " : "
            if opcode == 1 or opcode == 2:
                path_buf = str(pkt_buf[2:])
                path_buf = path_buf.split('\0', 1)[0]
                for i in range(tftp_hash_defines.pkt_payload_size):
                    b = pkt_buf[i]
                    if (b > 32 and b < 128):
                        payload_str += chr(b)
                    else:
                        payload_str += ' '
                #payload_str += path_buf
            elif opcode == 3:
                (block_no,) = struct.unpack('>H', pkt_buf[2:4])
                payload_str += str(block_no)
                payload_str += " : "
                data_buf = str(pkt_buf[4:])
                #pdb.set_trace()
                data_buf = data_buf.split('\0', 1)[0]
                #payload_str += data_buf
            elif opcode == 4:
                (block_no,) = struct.unpack('>H', pkt_buf[2:4])
                payload_str += str(block_no)
            elif opcode == 5:
                (err_code,) = struct.unpack('>H', pkt_buf[2:4])
                payload_str += str(err_code)
                payload_str += ","
                for i in range(tftp_hash_defines.pkt_payload_size):
                    b = pkt_buf[i]
                    if (b > 32 and b < 128):
                        payload_str += chr(b)
            elif opcode == 6:
                for i in range(1,tftp_hash_defines.pkt_payload_size):
                    b = pkt_buf[i]
                    if (b > 32 and b < 128):
                        payload_str += chr(b)
            line_str += payload_str
            print(line_str, file=self.out_fd)
            dbg_item = tftp_debug_item()
            dbg_item.seqno = pkt_evt.seq_no
            dbg_item.time_delta = pkt_evt.time_delta
            dbg_item.sd = pkt_evt.sd
            dbg_item.event = tftp_connection_debug_id_string.str[pkt_evt.id]
            dbg_item.sock_res = sock_res_str
            dbg_item.err_num = pkt_evt.err_num
            dbg_item.payload = payload_str
            self.dbg_list[pkt_evt.seq_no] = dbg_item
        print("-" * 80, file=self.out_fd)

    def print_consolidated(self):
        print('', file=self.out_fd)
        print("combined list:", file=self.out_fd)
        print("-" * 110, file=self.out_fd)
        line_str = "{:<6s} | {:<12s} | {:<12s} | {:<12s} | {:<10s} | {:<8s} | {:<8s}".format(
            "seqno", "adj-time(ms)", "sd", "event", "result", "errno", "payload")
        print(line_str, file=self.out_fd)
        print("-" * 110, file=self.out_fd)
        first_time = None
        for seqno in sorted(self.dbg_list.keys()):
            if first_time == None:
                first_time = self.dbg_list[seqno].time_delta

            self.dbg_list[seqno].adj_time_in_ms = (self.dbg_list[seqno].time_delta - first_time) / 1000
            payload = ""
            if (self.dbg_list[seqno].payload != None):
                payload = self.dbg_list[seqno].payload

            line_str = "{:<6d} | {:<12d} | {:<12d} | {:<12s} | {:<10s} | {:<8d} | {:<s}".format(
            seqno, int(self.dbg_list[seqno].adj_time_in_ms), self.dbg_list[seqno].sd,
            self.dbg_list[seqno].event, self.dbg_list[seqno].sock_res, self.dbg_list[seqno].err_num, payload)
            print(line_str, file=self.out_fd)
        print("-" * 80, file=self.out_fd)

    def open_file(self, filename):
        fd = open(filename, "rb")
        filesize = os.stat(filename).st_size
        print("opened file [%d]bytes" % filesize, file=self.out_fd)
        raw_buf = fd.read()
        self.open_buf(raw_buf)

    def open_buf(self, buf):
        self.raw_buf = buf
        self.tftpd = tftp_connection_info.from_buffer_copy(self.raw_buf, 0)
        print("", file=self.out_fd)
        print("-" * 25, file=self.out_fd)
        print('seq_no      : ', self.tftpd.seq_no, file=self.out_fd)
        print('dbg_evt_idx : ', self.tftpd.dbg_evt_idx, file=self.out_fd)
        print('tx_evt_idx  : ', self.tftpd.tx_evt_idx, file=self.out_fd)
        print('rx_evt_idx  : ', self.tftpd.rx_evt_idx, file=self.out_fd)
        print("-" * 25, file=self.out_fd)
        self.process_dbg_array()
        self.process_pkt_array(self.tftpd.tx_pkts, "tx_pkts:", tftp_connection_debug_id.tx)
        self.process_pkt_array(self.tftpd.rx_pkts, "rx_pkts:", tftp_connection_debug_id.rx)
        self.print_consolidated()


@register_parser('--tftp-server', 'TFTP Server report', optional=True)
class tftp_server_class(RamParser):
    def __init__(self, *args):
        super(tftp_server_class, self).__init__(*args)
        self.tftp_debug = tftp_debug()
        self.bss_start = None
        self.pgdp = None
        self.tftp_mmu = None
        self.tftp_bss_region = None

    def output(self, str_val):
        self.output_file.write(str_val)

    def parse(self):
        self.output_file = self.ramdump.open_file('tftp-server-report.txt')
        self.tftp_debug.out_fd = self.output_file
        print_out_str("tftp-server-parser:start")
        s1 = self.ramdump.get_kernel_version()
        s2 = 'Kernel version : [{0:d}.{0:d}.{0:d}]\n'.format(s1[0], s1[1], s1[2])
        self.output(s2)
        self.locate_tftp_server_mmap()
        self.read_tftp_bss_region()
        self.tftp_debug.open_buf(self.tftp_bss_region)
        #self.dump_tftp_bss_region('tftp_bss_region.bin')
        print_out_str("tftp-server-parser:end")
        self.output_file.close()

    def locate_tftp_server_mmap(self):
        addr_length = 8 if self.ramdump.arm64 else 4
        offset_comm = self.ramdump.field_offset('struct task_struct', 'comm')
        mm_offset = self.ramdump.field_offset('struct task_struct', 'mm')
        bss_start = 0
        for task in self.ramdump.for_each_process():
            task_name = task + offset_comm
            task_name = cleanupString(self.ramdump.read_cstring(task_name, 16))
            if task_name == 'tftp_server':
                mm_addr    = self.ramdump.read_word(task + mm_offset)
                mmap       = self.ramdump.read_structure_field(mm_addr, 'struct mm_struct', 'mmap')
                pgd        = self.ramdump.read_structure_field(mm_addr, 'struct mm_struct', 'pgd')
                self.pgdp  = self.ramdump.virt_to_phys(pgd)
                start_data = self.ramdump.read_structure_field(mm_addr, 'struct mm_struct', 'start_data')
                end_data   = self.ramdump.read_structure_field(mm_addr, 'struct mm_struct', 'end_data')
                mmap_next    = mmap
                # bss section is after data section
                map_count = 0
                while mmap_next != 0:
                    tmpstartVm = self.ramdump.read_structure_field(mmap_next, 'struct vm_area_struct', 'vm_start')
                    tmpendVm   = self.ramdump.read_structure_field(mmap_next, 'struct vm_area_struct', 'vm_end')
                    map_count = map_count + 1
                    if (end_data > tmpstartVm) and (end_data < tmpendVm):
                        # android P and older : 3 logd vma, bss section is just after end_data
                        if map_count < 3:
                            # data section is 4bytes align while bss section is 8 bytes align
                            bss_start = (end_data + 7) & 0x000000fffffffff8
                        else:
                            # android Q: 2 code vma + 2 data vma + 1bss, bss section is individual vma after end_data
                            if (start_data < tmpstartVm):
                                mmap_next   = self.ramdump.read_structure_field(mmap_next, 'struct vm_area_struct', 'vm_next')
                                bss_start = self.ramdump.read_structure_field(mmap_next, 'struct vm_area_struct', 'vm_start')
                            else:
                                # android R: 3 code vma and 1 data+bss vma, bss section is just after end_data, data section is addr_length align and bss section is 8 bytes align
                                if self.ramdump.arm64:
                                    bss_start = end_data
                                else:
                                    bss_start = (end_data + 7) & 0xfffffff8
                        print_out_str("bss_start:2 0x%x\n" %(bss_start))
                        break
                    mmap_next = self.ramdump.read_structure_field(mmap_next, 'struct vm_area_struct', 'vm_next')
                break
        self.bss_start = bss_start
        if self.ramdump.arm64:
            self.tftp_mmu = Armv8MMU(self.ramdump, self.pgdp)
        else:
            self.tftp_mmu = Armv7MMU(self.ramdump, self.pgdp)

    def read_tftp_bss_region(self):
        va = self.bss_start
        cur_size = 0
        self.tftp_bss_region = bytes()
        while cur_size < 32768:
            pa = self.tftp_mmu.virt_to_phys(va)
            tftp_buf = self.ramdump.read_physical(pa, 1)
            if (tftp_buf is None) or (tftp_buf == ''):
                break
            self.tftp_bss_region += tftp_buf
            va = va + 1
            cur_size += 1

    def dump_tftp_bss_region(self, filename):
        fd = self.ramdump.open_file(filename, mode='wb')
        fd.write(self.tftp_bss_region)
        fd.close()

if __name__ == '__main__':
    tftp_inst = tftp_debug()
    tftp_inst.out_fd = sys.stdout
    if len(sys.argv) != 2:
        tftp_inst.print_usage()
        die
    tftp_inst.open_file(sys.argv[1])
