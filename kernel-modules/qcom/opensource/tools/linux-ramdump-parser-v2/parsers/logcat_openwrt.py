# Copyright (C) 2013 Felix Fietkau <nbd@openwrt.org>
# Copyright (C) 2013 John Crispin <blogic@openwrt.org>
# Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from parser_util import RamParser
from print_out import print_out_str
from utasklib import UTaskLib
import datetime
import logging
import os

class Logcat_openwrt(RamParser):
    '''
    Logcat parser to support extract logcat from Openwrt platform
    '''
    LOG_EMERG = 0
    LOG_ALERT = 1
    LOG_CRIT = 2
    LOG_ERR = 3
    LOG_WARNING = 4
    LOG_NOTICE = 5
    LOG_INFO = 6
    LOG_DEBUG = 7

    LOG_PRIMASK = 7
    INTERNAL_NOPRI = 0x10

    LOG_KERN = (0 << 3)
    LOG_USER = (1 << 3)
    LOG_MAIL = (2 << 3)
    LOG_DAEMON = (3 << 3)
    LOG_AUTH = (4 << 3)
    LOG_SYSLOG = (5 << 3)
    LOG_LPR = (6 << 3)
    LOG_NEWS = (7 << 3)
    LOG_UUCP = (8 << 3)
    LOG_CRON = (9 << 3)
    LOG_AUTHPRIV = (10 << 3)
    LOG_FTP = (11 << 3)

    LOG_LOCAL0 = (16 << 3)
    LOG_LOCAL1 = (17 << 3)
    LOG_LOCAL2 = (18 << 3)
    LOG_LOCAL3 = (19 << 3)
    LOG_LOCAL4 = (20 << 3)
    LOG_LOCAL5 = (21 << 3)
    LOG_LOCAL6 = (22 << 3)
    LOG_LOCAL7 = (23 << 3)

    LOG_NFACILITIES = 24

    INTERNAL_MARK = (LOG_NFACILITIES << 3)
    LOG_PID = 0x01
    LOG_CONS = 0x02
    LOG_ODELAY = 0x04
    LOG_NDELAY = 0x08
    LOG_NOWAIT = 0x10
    LOG_PERROR = 0x20
    LOG_FACMASK = 0x3f8
    prioritynames = {
        "alert": LOG_ALERT,
        "crit": LOG_CRIT,
        "debug": LOG_DEBUG,
        "emerg": LOG_EMERG,
        "err": LOG_ERR,
        "error": LOG_ERR,
        "info": LOG_INFO,
        "none": INTERNAL_NOPRI,
        "notice": LOG_NOTICE,
        "panic": LOG_EMERG,
        "warn": LOG_WARNING,
        "warning": LOG_WARNING,
            "0": -1
        }
    facilitynames = {
        "auth": LOG_AUTH,
        "authpriv": LOG_AUTHPRIV,
        "cron": LOG_CRON,
        "daemon": LOG_DAEMON,
        "ftp": LOG_FTP,
        "kern": LOG_KERN,
        "lpr": LOG_LPR,
        "mail": LOG_MAIL,
        "mark": INTERNAL_MARK,
        "news": LOG_NEWS,
        "security": LOG_AUTH,
        "sysLOG": LOG_SYSLOG,
        "user": LOG_USER,
        "uucp": LOG_UUCP,
        "local0": LOG_LOCAL0,
        "local1": LOG_LOCAL1,
        "local2": LOG_LOCAL2,
        "local3": LOG_LOCAL3,
        "local4": LOG_LOCAL4,
        "local5": LOG_LOCAL5,
        "local6": LOG_LOCAL6,
        "local7": LOG_LOCAL7,
        "0": -1
    }

    def __init__(self, ramdump, logd_task):
        super().__init__(ramdump)
        self.logd_task = logd_task
        if self.ramdump.arm64:
            self.addr_length = 8
        else:
            self.addr_length = 4
        self.SIZEOF_LOG_HEAD = 32

        # logger init
        self.logger = logging.getLogger(__name__)
        path = os.path.join(self.ramdump.outdir, 'logcat_debug_log.txt')
        self.logger.addHandler(logging.FileHandler(path, mode='w'))
        self.logger.setLevel(logging.INFO)

    def LOG_PRI(self, p):
        return (p) & self.LOG_PRIMASK

    def LOG_FAC(self, p):
        return ((p) & self.LOG_FACMASK) >> 3

    def getcodetext(self, value, codetable):
        for name, val in codetable.items():
            if val == value:
                return name
        return "<unknown>"

    def PAD(self, x):
        return (((x) - (x % 4)) + 4) if (x % 4) else (x)

    def log_next(self, log_head_addr, size):
        n = log_head_addr + self.SIZEOF_LOG_HEAD + self.PAD(size)
        return self.log_start_addr if n > self.log_end_addr else n

    def log_list(self):
        h = self.log_oldest_addr
        while h != self.log_newest_addr and h <= self.log_end_addr and h >= self.log_start_addr:
            size = self.read_bytes(h, 4)
            if size > 1024:
                self.logger.warning("invalid size(%x), expected <= 1024" % size)
                break
            yield h

            # read next log element
            h = self.log_next(h, size)
            size = self.read_bytes(h, 4)
            if size == 0 and h > self.log_newest_addr: # reach to end of buffer
                h = self.log_start_addr

    def process_logbuf_and_save(self):
        log_file = self.ramdump.open_file("Logcat_openwrt.txt")
        for log_head_addr in self.log_list():
            size = self.read_bytes(log_head_addr, 4)
            id = self.read_bytes(log_head_addr + 4, 4)
            priority = self.read_bytes(log_head_addr + 8, 4)
            source = self.read_bytes(log_head_addr + 12, 4)
            tv_sec = self.read_bytes(log_head_addr + 16, 8)
            tv_nsec = self.read_bytes(log_head_addr + 24, 8)
            timestamp = self.format_time(tv_sec, tv_nsec)
            msg = self.read_binary(log_head_addr + self.SIZEOF_LOG_HEAD, size)
            msg = msg.decode('ascii', 'ignore').strip('\0')
            fmt_msg = "%s %s.%s %s %s\n" % (timestamp,
                                        self.getcodetext(self.LOG_FAC(priority) << 3, self.facilitynames),
                                        self.getcodetext(self.LOG_PRI(priority), self.prioritynames),
                                        "" if source else (" kernel:"), msg)
            log_file.write(fmt_msg)

    def read_bytes(self, addr, len):
        val = UTaskLib.read_bytes(self.ramdump, self.logd_task.mmu, addr, len)
        if not val:
            val = 0
        return val

    def read_binary(self, addr, len):
        return UTaskLib.read_binary(self.ramdump, self.logd_task.mmu, addr, len)

    def format_time(self, tv_sec, tv_nsec):
        tv_sec = tv_sec & 0xffffffff
        tv_nsec = str(tv_nsec // 1000000)
        tv_nsec = str(tv_nsec).zfill(3)
        date = datetime.datetime.utcfromtimestamp(tv_sec)
        timestamp = date.strftime("%m-%d %H:%M:%S") + '.' + tv_nsec
        return timestamp

    def is_valid_logd_addr(self, addr):
        try:

            self.log_start_addr = self.read_bytes(addr + self.addr_length, self.addr_length)
            self.log_end_addr = self.read_bytes(addr, self.addr_length)
            self.log_oldest_addr = self.read_bytes(addr + 11 * self.addr_length, self.addr_length)
            self.log_newest_addr = self.read_bytes(addr + 10 * self.addr_length, self.addr_length)

            if not self.log_start_addr or not self.log_end_addr \
                    or not self.log_oldest_addr or not self.log_newest_addr:
                self.logger.debug("Can't find log/log_end/oldest/newest address, LogBuffer address was not found on addr=0x%x" % addr)
                return False

            if self.log_start_addr < self.log_end_addr and \
                    self.log_start_addr <= self.log_oldest_addr <= self.log_end_addr and \
                    self.log_start_addr <= self.log_newest_addr <= self.log_end_addr:

                self.logger.debug("log_start 0x%x log_end 0x%x log_oldest 0x%x log_newest 0x%x" % (
                self.log_start_addr, self.log_end_addr, self.log_oldest_addr, self.log_newest_addr))

                ## check buffer size is valid
                buffer_size = int((self.log_end_addr - self.log_start_addr)/1024)
                if buffer_size > 1024:
                    return False
                print_out_str("log buffer size %d k" % buffer_size)
                return True
            else:
                self.logger.debug("invalid address--> log_start 0x%x log_end 0x%x log_oldest 0x%x log_newest 0x%x" % (
                self.log_start_addr, self.log_end_addr, self.log_oldest_addr, self.log_newest_addr))
                return False
        except:
            return False

    def parse(self):
        startTime = datetime.datetime.now()

        data_start = 0
        for vma in self.logd_task.vmalist:
            if vma.file_name == "logd" and vma.flags & 0b11 == 0b11:
                data_start = vma.vm_start
                break

        if not data_start:
            print_out_str("logd process didn't have data section, LogBuffer address was not found")
            return

        if self.ramdump.arm64:
            log_end_offset = 0x328
            addr_length = 8
        else:
            log_end_offset = 0x1e0
            addr_length = 4

        valid = self.is_valid_logd_addr(vma.vm_start + log_end_offset)
        if not valid:
            ## continue to search
            print_out_str("log address was not found with offset 0x%x, go through whole vma" % log_end_offset)
            addr = vma.vm_start
            while addr < (vma.vm_end):
                valid = self.is_valid_logd_addr(addr)
                if valid:
                    print_out_str("found logd addr offset is 0x%x" % (addr - vma.vm_start))
                    break
                addr += addr_length

        if valid:
            self.process_logbuf_and_save()
        print_out_str("Logcat parse cost " + str((datetime.datetime.now() - startTime).total_seconds()) + " s")
