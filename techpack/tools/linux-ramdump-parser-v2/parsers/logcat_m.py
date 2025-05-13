# Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from parser_util import register_parser, RamParser, cleanupString
from print_out import print_out_str
import struct
from utasklib import UTaskLib
from utasklib import ProcessNotFoundExcetion
import datetime

class Logcat_m(RamParser):
    '''
    Logcat parser to support extract logcat from Androd M ram dump
    '''
    LOG_NAME = [ "main", "radio", "events", "system", "crash", "kernel"]
    #log id
    LOG_ID_MIN = 0
    LOG_ID_MAIN = 0
    LOG_ID_RADIO = 1
    LOG_ID_EVENTS = 2
    LOG_ID_SYSTEM = 3
    LOG_ID_CRASH = 4
    LOG_ID_KERNEL = 5
    LOG_ID_MAX = 5

    #log level
    ANDROID_LOG_DEFAULT =1
    ANDROID_LOG_VERBOSE = 2
    ANDROID_LOG_DEBUG = 3
    ANDROID_LOG_INFO = 4
    ANDROID_LOG_WARN = 5
    ANDROID_LOG_ERROR = 6
    ANDROID_LOG_FATAL = 7
    ANDROID_LOG_SILENT = 8

    def __init__(self, ramdump, logd_task):
        super().__init__(ramdump)
        self.vmas = []
        self.logd_task = logd_task
        if self.ramdump.arm64:
            self.addr_length    = 8
        else:
            self.addr_length    = 4

    def find_logbuf_elem_list_addr(self, vma):
        vma_size = vma["size"]
        vma_data = vma["data"]
        offset = 0
        is_chunk = False
        while offset < vma_size:
            is_chunk = self.is_logbuf_elem_list_addr(vma_data, offset)
            if is_chunk:
                break
            offset = offset + 4
        if is_chunk:
            return offset
        return 0

    def parse(self):
        startTime = datetime.datetime.now()
        self.get_vmas_with_rw()

        # find address of std::list<LogBufferElement *> mLogElements
        logbuf_addr = 0
        for vma in self.vmas:
            offset = self.find_logbuf_elem_list_addr(vma)
            if offset != 0:
                logbuf_addr = vma["vmstart"]+offset
                print("found address",hex(logbuf_addr))
                break
        if logbuf_addr:
            print_out_str("LogBuffer address 0x%x"% logbuf_addr)
            self.process_logbuf_and_save(logbuf_addr)
            print_out_str("Logcat parse cost "+str((datetime.datetime.now()-startTime).total_seconds())+" s")
        else:
            print_out_str("LogBuffer address was not found")

    def is_logbuf_elem_list_addr(self, vma_data, offset):
        if offset+24 > len(vma_data):
            return False
        nodes = struct.unpack('<QQQ', vma_data[offset:offset+24])
        tail_node_addr = nodes[0]
        head_node_addr = nodes[1]
        list_count = nodes[2]
        if tail_node_addr ==0  or head_node_addr ==0:
            return False
        next_node_addr = head_node_addr
        prev_node = self.read_bytes(head_node_addr, self.addr_length) # prev_node = next_node_addr->prev
        if prev_node == 0:
            return False
        if list_count == 0 and head_node_addr == tail_node_addr: # empty list
            return False
        index = 0
        while next_node_addr != 0 and index < list_count:
            next_prev_node = self.read_bytes(next_node_addr, self.addr_length)
            if not next_prev_node or next_prev_node != prev_node:
                return False
            current_node_addr = next_node_addr + self.addr_length *2
            current_node = self.read_bytes(current_node_addr, self.addr_length)
            is_chunk = self.is_log_elem(current_node)
            if not is_chunk:
                return False
            if next_node_addr == tail_node_addr: # loop complete
                return True
            prev_node = next_node_addr
            next_node_addr = self.read_bytes(next_node_addr + self.addr_length, self.addr_length)
            index = index +1
        return False

    def is_log_elem(self, addr):
        log_id = self.read_bytes(addr + 0x8, 4)
        uid = self.read_bytes(addr + 0xc, 4)
        pid = self.read_bytes(addr + 0x10, 4)
        msg_addr = self.read_bytes(addr + 0x18, 8)
        msg_len = self.read_bytes(addr + 0x20, 2)
        dropped = msg_len
        if log_id < 0 or log_id > self.LOG_ID_MAX:
            return False

        if msg_len < 1 or msg_len > 4068:  # max_payload
            return False

        if (pid < 0 or pid >= 65536) or (uid < 0 or uid >= 65536):
            return False
        if (log_id == 2) or (dropped == 1):
            return True
        else:
            priority = self.read_bytes(msg_addr, 1)
            if priority > self.ANDROID_LOG_SILENT or priority < self.ANDROID_LOG_DEFAULT:
                return False
        return True

    def process_logbuf_and_save(self, logbuf_addr):
        head_node_addr = self.read_bytes(logbuf_addr + self.addr_length, self.addr_length)
        list_count = self.read_bytes(logbuf_addr + self.addr_length*2, self.addr_length)

        next_node_addr = head_node_addr
        index = 0
        log_file = [0] * (self.LOG_ID_MAX + 1)
        while index <= self.LOG_ID_MAX:
            log_file[index] = self.ramdump.open_file(self.get_output_filename(index))
            index = index + 1

        index = 0
        log_list = []
        while next_node_addr != 0 and index < list_count:
            current_node = self.read_bytes(next_node_addr + self.addr_length *2, self.addr_length)
            self.save_log_elem(current_node, log_list)
            next_node_addr = self.read_bytes(next_node_addr + self.addr_length, self.addr_length)
            index = index +1

        log_list.sort(key=lambda x: float(x['timestamp']))

        for item in log_list:
            log_file[item["logid"]].write(item["msg"])

        index = 0
        while index <= self.LOG_ID_MAX:
            log_file[index].close()
            index = index + 1

        return

    def save_log_elem(self, addr, log_list):
        log_id = self.read_bytes(addr + 0x8, 4)
        uid = self.read_bytes(addr + 0xc, 4)
        pid = self.read_bytes(addr + 0x10, 4)
        tid = self.read_bytes(addr + 0x14, 4)
        msg_addr = self.read_bytes(addr + 0x18, 8)
        msg_len = self.read_bytes(addr + 0x20, 2)
        dropped = msg_len
        tv_sec = self.read_bytes(addr + 0x30, 4)
        tv_nsec = self.read_bytes(addr + 0x34, 4)
        if (log_id == 2) or (dropped == 1):
            return

        if msg_len == 0:
            return

        priority = self.read_bytes(msg_addr, 1)
        msg = self.read_binary(msg_addr + 1, msg_len - 1)
        if not msg:
            return

        msgList = msg.decode('ascii', 'ignore').split('\0')
        timestamp = self.format_time(tv_sec, tv_nsec)

        if len(msgList) >= 2:
            time_f = float("{}.{}".format(tv_sec,tv_nsec))
            msg = "%s %5d %5d %5d %c %-8.*s: %s\n" % (timestamp, uid, pid, tid,
                                                      self.filter_pri_to_char(priority), len(msgList[0]),
                                                      cleanupString(msgList[0].strip()),
                                                      cleanupString(msgList[1].strip()))

        log = {}
        log['timestamp'] = time_f
        log['msg'] = msg
        log["logid"] = log_id
        log_list.append(log)

    def filter_pri_to_char(self, pri) :
        if pri == self.ANDROID_LOG_VERBOSE:
            return 'V'
        elif pri == self.ANDROID_LOG_DEBUG:
            return 'D'
        elif pri == self.ANDROID_LOG_INFO:
            return 'I'
        elif pri == self.ANDROID_LOG_WARN:
            return 'W'
        elif pri == self.ANDROID_LOG_ERROR:
            return 'E'
        elif pri == self.ANDROID_LOG_FATAL:
            return 'F'
        elif pri == self.ANDROID_LOG_SILENT:
            return 'S'
        else:
            return '?'

    def get_vmas_with_rw(self):
        '''
        return vma list with read+write permissions
        '''
        for vma in self.logd_task.vmalist:
            if vma.flags & 0b11 != 0b11:
                continue
            item = {}
            item["vmstart"] = vma.vm_start
            item["size"] = vma.vm_end - vma.vm_start
            item["data"] = self.read_binary(item["vmstart"], item["size"])
            self.vmas.append(item)

    def read_bytes(self, addr, len):
        val = UTaskLib.read_bytes(self.ramdump, self.logd_task.mmu, addr, len)
        if not val:
            val = 0
        return val

    def read_binary(self, addr, len):
        return UTaskLib.read_binary(self.ramdump, self.logd_task.mmu, addr, len)

    def get_output_filename(self, log_id):
        if log_id >= self.LOG_ID_MIN and log_id <= self.LOG_ID_MAX:
            return "Logcat_{}.txt".format(self.LOG_NAME[log_id])
        else:
            return None

    def format_time(self, tv_sec, tv_nsec):
        tv_nsec = str(tv_nsec // 1000000)
        tv_nsec = str(tv_nsec).zfill(3)
        date = datetime.datetime.utcfromtimestamp(tv_sec)
        timestamp = date.strftime("%m-%d %H:%M:%S") + '.' + tv_nsec
        return timestamp
