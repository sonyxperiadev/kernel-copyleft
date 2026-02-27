# Copyright (c) 2020-2021 The Linux Foundation. All rights reserved.
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
from mmu import Armv8MMU, Armv7MMU
from print_out import print_out_str
import struct
import linecache
import datetime
import os
import subprocess

def read_bytes(mmu, ramdump, addr, len):
    addr = mmu.virt_to_phys(addr)
    s = ramdump.read_physical(addr, len)
    if (s is None) or (s == ''):
        return None
    if len == 8:
        s = struct.unpack('<Q', s)
    elif len == 4:
        s = struct.unpack('<I', s)
    elif len == 2:
        s = struct.unpack('<H', s)
    elif len == 1:
        s = struct.unpack('<B', s)
    else:
        print_out_str("This api used to unpack 1/2/4/8 bytes data, check the len\n")
        exit()
    return s[0]

def parse_logBuf_offset(ramdump):
    space = " "
    bss_sec_start = 0
    logBuf_addr   = 0
    file_name = "symbols/logd"
    objdump_path = ramdump.objdump_path
    option_header = "-h"
    option_symbols = "-t"
    logd_file_path = os.path.join(ramdump.outdir, file_name)

    if not os.path.exists(logd_file_path):
        logd_tmp = ramdump.vmlinux.split("product")
        product_tmp = logd_tmp[1].split("\\")
        logd_file_path = logd_tmp[0] + "product/" + product_tmp[1] + "\\" + "symbols/system/bin/logd"

    if not os.path.exists(logd_file_path):
        logd_tmp = ramdump.vmlinux.split("product")
        ## android Q
        logd_file_path = logd_tmp[0] + "product/" + product_tmp[0] + "\\" + "symbols/system/bin/logd"
        if not os.path.exists(logd_file_path):
            ##android R
            logd_file_path = logd_tmp[0] + "product/" + "qssi/symbols/system/bin/logd"

    if os.path.exists(logd_file_path):
        print_out_str("logd_file_path = %s\n" %(logd_file_path))
        cmd = objdump_path + space + option_header + space + logd_file_path
        objdump = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, universal_newlines=True)
        out, err = objdump.communicate()
        ## merge the spaces
        out = ' '.join(out.split())
        ## split by space
        data = out.split(space)
        i = 0
        while True:
            if i >=len(data):
                break
            if ".bss" in data[i]:
                bss_sec_start = int(data[i+2], 16)
                break
            i = i + 1
        if bss_sec_start == 0:
            print_out_str("bss section not found in the logd")
            exit()

        cmd = objdump_path + space + option_symbols + space + logd_file_path
        objdump = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, universal_newlines=True)
        out, err = objdump.communicate()
        ## merge the spaces
        out = ' '.join(out.split())
        ## split by space
        data = out.split(space)
        i = 0
        while True:
            if i >=len(data):
                break
            if ("logBuf" in data[i]) and ("ZL" in data[i]):
                logBuf_addr = int(data[i-5], 16)
                break
            i = i + 1
        if logBuf_addr == 0:
            print_out_str("logBuf not found in logd\n")
            exit()
    else:
        print_out_str("%s is not found. you can copy the logd symbols file to here.\n" %(logd_file_path));

    return logBuf_addr - bss_sec_start;

def parse_logcat_v2(ramdump):
    timestamp="timestamp"
    MSG="msg"
    uid="uid"
    pid="pid"
    tid="tid"
    log_type="log_type"
    priority="level"
    logcat_file_list = ["logcat_v2_all.txt", "logcat_v2_main.txt", "logcat_v2_radio.txt", "logcat_v2_event.txt", "logcat_v2_system.txt",
                            "logcat_v2_crash.txt", "logcat_v2_stats.txt", "logcat_v2_security.txt", "logcat_v2_kernel.txt"]
    log_type_list = ["0:main", "1:radio", "2:event", "3:system", "4:crash", "5:stats", "6:security", "7:kernel", "unknown"]
    priority_list = ["0:UNKNOWN", "1:DEFAULT", "2:VERBOSE", "3:DEBUG", "4:INFO", "5:WARN", "6:ERR", "7:FATAL", "8:SILENT", "wrong priority"]
    logcat_file_id = 0
    log_id_max     = 7
    priority_max   = 8

    offset_comm = ramdump.field_offset('struct task_struct', 'comm')
    mm_offset = ramdump.field_offset('struct task_struct', 'mm')
    f_path_offset = ramdump.field_offset('struct file', 'f_path')
    dentry_offset = ramdump.field_offset('struct path', 'dentry')
    d_iname_offset = ramdump.field_offset('struct dentry', 'd_iname')

    if ramdump.arm64:
        addr_length    = 8
    else:
        addr_length    = 4

    first_node_offset  = addr_length * 2
    next_node_offset   = addr_length
    element_obj_offset = addr_length * 2
    msg_addr_offset    = 0x14
    msg_size_offset    = msg_addr_offset + addr_length

    logdaddr = 0
    bss_start = 0
    for task in ramdump.for_each_process():
        task_name = task + offset_comm
        task_name = cleanupString(ramdump.read_cstring(task_name, 16))
        if task_name == 'logd':
            mm_addr    = ramdump.read_word(task + mm_offset)
            mmap       = ramdump.read_structure_field(mm_addr, 'struct mm_struct', 'mmap')
            pgd        = ramdump.read_structure_field(mm_addr, 'struct mm_struct', 'pgd')
            pgdp       = ramdump.virt_to_phys(pgd)
            start_data = ramdump.read_structure_field(mm_addr, 'struct mm_struct', 'start_data')
            end_data   = ramdump.read_structure_field(mm_addr, 'struct mm_struct', 'end_data')
            logdmap    = mmap

            # bss section is after data section
            logd_count = 0
            while logdmap != 0:
                tmpstartVm = ramdump.read_structure_field(logdmap, 'struct vm_area_struct', 'vm_start')
                tmpendVm   = ramdump.read_structure_field(logdmap, 'struct vm_area_struct', 'vm_end')
                logd_count = logd_count + 1
                if (end_data > tmpstartVm) and (end_data < tmpendVm):
                    # android P and older : 3 logd vma, bss section is just after end_data
                    if logd_count < 3:
                        # data section is 4bytes align while bss section is 8 bytes align
                        bss_start = (end_data + 7) & 0x000000fffffffff8
                    else:
                        # android Q: 2 code vma + 2 data vma + 1bss, bss section is individual vma after end_data
                        if (start_data < tmpstartVm):
                            logdmap   = ramdump.read_structure_field(logdmap, 'struct vm_area_struct', 'vm_next')
                            bss_start = ramdump.read_structure_field(logdmap, 'struct vm_area_struct', 'vm_start')
                        else:
                            # android R: 3 code vma and 1 data+bss vma, bss section is just after end_data, data section is addr_length align and bss section is 8 bytes align
                            if ramdump.arm64:
                                bss_start = end_data
                            else:
                                bss_start = (end_data + 7) & 0xfffffff8
                            first_node_offset = addr_length
                    print_out_str("bss_start: 0x%x\n" %(bss_start))
                    break
                logdmap = ramdump.read_structure_field(logdmap, 'struct vm_area_struct', 'vm_next')

            break

    if ramdump.arm64:
        mmu = Armv8MMU(ramdump, pgdp)
    else:
        mmu = Armv7MMU(ramdump, pgdp)

    logbuf_offset = parse_logBuf_offset(ramdump);
    print_out_str("logbuf_offset = 0x%x" %(logbuf_offset))
    logbuf_addr = read_bytes(mmu, ramdump, bss_start + logbuf_offset, addr_length)
    print_out_str("logbuf_addr = 0x%x" %(logbuf_addr))

    first_node_addr = read_bytes(mmu, ramdump, logbuf_addr + first_node_offset, addr_length)
    next_node_addr = first_node_addr
    if next_node_addr is not None:
        print_out_str("first_node_addr = 0x%x\n" %(next_node_addr))

    index = 0
    log_file = [0] * 9
    while index < 9:
        log_file[index] = ramdump.open_file(logcat_file_list[index])
        log_file[index].write("\n\n========== PARSE START ==========\n\n\n")
        log_file[index].write(timestamp.ljust(32))
        log_file[index].write(uid.rjust(8))
        log_file[index].write(pid.rjust(8))
        log_file[index].write(tid.rjust(8))
        log_file[index].write("\t")
        log_file[index].write(log_type.ljust(16))
        log_file[index].write(priority.center(16))
        log_file[index].write(MSG.ljust(8))
        log_file[index].write("\n")
        index = index + 1
    while next_node_addr != 0:
        logbuffer_ele_obj = read_bytes(mmu, ramdump, next_node_addr + element_obj_offset, addr_length)
        next_node_addr = read_bytes(mmu, ramdump, next_node_addr + next_node_offset, addr_length)
        if next_node_addr == first_node_addr:
            break

        logbuffer_ele_obj_phys = mmu.virt_to_phys(logbuffer_ele_obj)

        ## uid pid tid and etc parsed from LogBufferElement
        uid = read_bytes(mmu, ramdump, logbuffer_ele_obj, 4)
        pid = read_bytes(mmu, ramdump, logbuffer_ele_obj + 0x4, 4)
        tid = read_bytes(mmu, ramdump, logbuffer_ele_obj + 0x8, 4)
        tv_second = read_bytes(mmu, ramdump, logbuffer_ele_obj + 0xC, 4)
        tv_second_nano = read_bytes(mmu, ramdump, logbuffer_ele_obj + 0x10, 4)
        msg_addr = read_bytes(mmu, ramdump, logbuffer_ele_obj + msg_addr_offset, addr_length)
        msg_size = read_bytes(mmu, ramdump, logbuffer_ele_obj + msg_size_offset, 2)
        log_id   = read_bytes(mmu, ramdump, logbuffer_ele_obj + msg_size_offset + 0x2, 1)
        dropped  = read_bytes(mmu, ramdump, logbuffer_ele_obj + msg_size_offset + 0x3, 1)
        if (log_id == 2) or (dropped == 1) :
            continue

        if logbuffer_ele_obj_phys is None or msg_size == 0:
            break

        uid = str(uid)
        pid = str(pid)
        tid = str(tid)
        tv_second_nano = str(tv_second_nano // 1000)
        tv_second_nano = str(tv_second_nano).zfill(6)
        date = datetime.datetime.utcfromtimestamp(tv_second)
        timestamp = date.strftime("%Y-%m-%d %H:%M:%S") + '.' + tv_second_nano

        min = 0
        x = 0
        level = -1
        MSG = ""
        while msg_size != 0:
            i = 0
            msg_addr = msg_addr + min
            # msg located in the same page
            if msg_size < (0x1000 - msg_addr % 0x1000):
                min = msg_size
            # msg separated in two pages
            else:
                min = 0x1000 - msg_addr % 0x1000
            msg_size = msg_size - min
            msg_addr_phys = mmu.virt_to_phys(msg_addr)
            if msg_addr_phys is None:
                break
            if x == 0:
                ## level is at the first Byte of MSG
                level = read_bytes(mmu, ramdump, msg_addr, 1)
                MSG_tmp = ramdump.read_physical(msg_addr_phys + 1, min - 1)
            else:
                if min == 0:
                    break
                else:
                    MSG_tmp = ramdump.read_physical(msg_addr_phys, min)
            MSG_ascii = list(MSG_tmp)
            while i < len(MSG_ascii):
                MSG_ascii[i] = chr(MSG_tmp[i] & 0x7F)
                i = i + 1
            MSG = MSG + ''.join(MSG_ascii)
            MSG = MSG.replace('\000', '\040')
            MSG = MSG.replace('\015', '\040')
            MSG = MSG.replace('\012', '\040')
            x = x + 1

        if (log_id > log_id_max) or (log_id < 0):
            continue
        else:
            logcat_file_id = log_id + 1

        if (level > priority_max) or (level < 0):
            level = priority_max + 1

        log_type = log_type_list[log_id]
        priority = priority_list[level]
        log_file[0].write(timestamp.ljust(32))
        log_file[0].write(uid.rjust(8))
        log_file[0].write(pid.rjust(8))
        log_file[0].write(tid.rjust(8))
        log_file[0].write("\t")
        log_file[0].write(log_type.ljust(16))
        log_file[0].write(priority.center(16))
        log_file[0].write(MSG)
        log_file[0].write("\n")
        log_file[logcat_file_id].write(timestamp.ljust(32))
        log_file[logcat_file_id].write(uid.rjust(8))
        log_file[logcat_file_id].write(pid.rjust(8))
        log_file[logcat_file_id].write(tid.rjust(8))
        log_file[logcat_file_id].write("\t")
        log_file[logcat_file_id].write(log_type.ljust(16))
        log_file[logcat_file_id].write(priority.center(16))
        log_file[logcat_file_id].write(MSG)
        log_file[logcat_file_id].write("\n")

        if next_node_addr == first_node_addr:
            break

    index = 0
    while index < 9:
        log_file[index].close()
        index = index + 1
    return

@register_parser('--logcat-v2', 'Extract logcat logs from ramdump ')
class Logcat_v2(RamParser):

    def parse(self):
        parse_logcat_v2(self.ramdump)
