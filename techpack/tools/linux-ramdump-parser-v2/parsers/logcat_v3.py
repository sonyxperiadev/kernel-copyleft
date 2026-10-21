# Copyright (c) 2021-2022 Qualcomm Innovation Center, Inc. All rights reserved.

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from parser_util import RamParser, cleanupString
from print_out import print_out_str
import struct
import datetime
import os
from concurrent import futures

class Logcat_base(RamParser):
    LOG_NAME = [ "main", "radio", "events", "system", "crash", "stats", "security", "kernel"]
    #log id
    LOG_ID_MIN = 0
    LOG_ID_MAIN = 0
    LOG_ID_RADIO = 1
    LOG_ID_EVENTS = 2
    LOG_ID_SYSTEM = 3
    LOG_ID_CRASH = 4
    LOG_ID_STATS = 5
    LOG_ID_SECURITY = 6
    LOG_ID_KERNEL = 7
    LOG_ID_MAX = 7

    #log level
    ANDROID_LOG_DEFAULT =1
    ANDROID_LOG_VERBOSE = 2
    ANDROID_LOG_DEBUG = 3
    ANDROID_LOG_INFO = 4
    ANDROID_LOG_WARN = 5
    ANDROID_LOG_ERROR = 6
    ANDROID_LOG_FATAL = 7
    ANDROID_LOG_SILENT = 8

    SIZEOF_LOG_ENTRY = 30
    SIZEOF_HEADER_T = 4
    SIZEOF_EVT_LIST_T = 2
    SIZEOF_EVT_INT_T = 5
    SIZEOF_EVT_LONG_T = 9
    SIZEOF_EVT_FLOAT_T = 5
    SIZEOF_EVT_STRING_T = 5

    #event type
    EVENT_TYPE_INT = 0  #/* int32_t */
    EVENT_TYPE_LONG = 1 #/* int64_t */
    EVENT_TYPE_STRING = 2
    EVENT_TYPE_LIST = 3
    EVENT_TYPE_FLOAT = 4

    def __init__(self, ramdump, mmu):
        super().__init__(ramdump)
        self.mmu = mmu
        self.sizeUsed = {}
        self.maxSize = {}
        self.is_success = False
        self.zstd = None
        try:
            self.zstd = __import__('zstandard')
        except ImportError as result:
            print_out_str(str(result)+", try to use command 'py -3 -m pip install zstandard' to install")
            print("\033[1;31m" + str(result) +
                    ", try to use command 'py -3 -m pip install zstandard' to install \033[0m")
        if self.ramdump.arm64:
            self.addr_length    = 8
        else:
            self.addr_length    = 4
        self.extra_offset = 0
        sys_tz_addr = self.ramdump.address_of('sys_tz')
        tz_minuteswest_offset = self.ramdump.field_offset(
                    'struct timezone ', 'tz_minuteswest')
        self.tz_minuteswest = self.ramdump.read_s32(sys_tz_addr + tz_minuteswest_offset)
        print_out_str("struct timezone --> tz_minuteswest= "+str(self.tz_minuteswest)+"min")

    def read_bytes(self, addr, len):
        addr = self.mmu.virt_to_phys(addr)
        s = self.ramdump.read_physical(addr, len)
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

    def read_binary(self, addr, length):
        """Reads binary data of specified length from addr_or_name."""
        min = 0
        msg = b''
        size = length
        while length > 0:
            addr = addr + min
            # msg located in the same page
            if length < (0x1000 - addr % 0x1000):
                min = length
            # msg separated in two pages
            else:
                min = 0x1000 - addr % 0x1000
            length = length - min
            addr_phys = self.mmu.virt_to_phys(addr)
            msg_binary = self.ramdump.read_physical(addr_phys, min)
            if msg_binary is None or msg_binary == '':
                return msg
            msg = msg + msg_binary
        return msg

    def format_time(self, tv_sec, tv_nsec):
        tv_nsec = str(tv_nsec // 1000000)
        tv_nsec = str(tv_nsec).zfill(3)
        tv_sec -= 60 * self.tz_minuteswest
        date = datetime.datetime.utcfromtimestamp(tv_sec)
        timestamp = date.strftime("%m-%d %H:%M:%S") + '.' + tv_nsec
        return timestamp

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

    def get_output_filename(self, log_id):
        if log_id >= self.LOG_ID_MIN and log_id <= self.LOG_ID_MAX:
            return "{}_{}.txt".format(self.__class__.__name__, self.LOG_NAME[log_id])
        else:
            return None

    def get_evt_data(self, data_array, pos):
        if (pos + 1) > len(data_array):
            return -1, -1, -1
        evt_type = struct.unpack('<B', data_array[pos : pos + 1])[0]
        length = 0
        msg=""
        if evt_type == self.EVENT_TYPE_INT :
            if (pos + self.SIZEOF_EVT_INT_T) > len(data_array):
                return -1, -1, -1
            msg = str(struct.unpack('<I', data_array[pos+1 : pos + self.SIZEOF_EVT_INT_T])[0])
            length = self.SIZEOF_EVT_INT_T
        elif evt_type == self.EVENT_TYPE_LONG:
            if (pos + self.SIZEOF_EVT_LONG_T) > len(data_array):
                return -1, -1, -1
            msg = str(struct.unpack('<Q', data_array[pos+1 : pos + self.SIZEOF_EVT_LONG_T])[0])
            length = self.SIZEOF_EVT_LONG_T
        elif evt_type == self.EVENT_TYPE_FLOAT:
            if (pos + self.SIZEOF_EVT_FLOAT_T) > len(data_array):
                return -1, -1, -1
            msg = str(struct.unpack('<f', data_array[pos+1 : pos + self.SIZEOF_EVT_FLOAT_T])[0])
            length = self.SIZEOF_EVT_FLOAT_T
        elif evt_type == self.EVENT_TYPE_STRING:
            if (pos + self.SIZEOF_EVT_STRING_T) > len(data_array):
                return -1, -1, -1
            #for event log, msg_len may be 0 like "I 1397638484: [121035042,4294967295,]"
            msg_len = struct.unpack('I', data_array[pos+1 : pos + self.SIZEOF_EVT_STRING_T])[0]
            # last msg_len-1 bytes
            tmpmsg = data_array[pos + self.SIZEOF_EVT_STRING_T : pos+self.SIZEOF_EVT_STRING_T+msg_len]
            length = self.SIZEOF_EVT_STRING_T + msg_len
            msg = tmpmsg.decode('ascii', 'ignore').strip()
        return evt_type, msg, length

    def process_log_and_save(self, _data):
        ret=""
        pos = 0
        while pos < len(_data):
            if pos +self.SIZEOF_LOG_ENTRY > len(_data):
                break
            # first [0-30] total 31 bytes
            logEntry = struct.unpack('<IIIQIIHB', _data[pos:pos+self.SIZEOF_LOG_ENTRY+1])
            pos = pos+self.SIZEOF_LOG_ENTRY + 1 + self.extra_offset
            uid = logEntry[0]
            pid = logEntry[1]
            tid = logEntry[2]
            sequence = logEntry[3]
            tv_sec = logEntry[4]
            tv_nsec = logEntry[5]
            msg_len = logEntry[6]
            priority = logEntry[7]
            if msg_len is None or msg_len < 1:
                break;
            msg = _data[pos:pos+msg_len-1] # last msg_len-1 bytes
            msgList = msg.decode('ascii', 'ignore').split('\0')
            pos = pos+msg_len-1
            timestamp = self.format_time(tv_sec, tv_nsec)
            if len(msgList) >= 2 :
                multilines = msgList[1].split("\n")
                preifx_line = "%s %5d %5d %5d %c %-8.*s: " % (
                    timestamp, uid, pid, tid, self.filter_pri_to_char(priority), len(msgList[0]),
                    cleanupString(msgList[0].strip()))
                if len(multilines) > 1:
                    for line in multilines:
                        if(len(cleanupString(line.strip())) > 0):
                            ret += preifx_line + "%s\n" % cleanupString(line)
                else:
                    ret += preifx_line + "%s\n" % cleanupString(msgList[1].strip())
        return ret

    def process_binary_log_and_save(self, _data):
        ret=""
        pos = 0
        while pos < len(_data):
            if pos +self.SIZEOF_LOG_ENTRY > len(_data):
                break
            # first [0-30] total 31 bytes
            logEntry = struct.unpack('<IIIQIIH', _data[pos : pos + self.SIZEOF_LOG_ENTRY])
            pos = pos + self.SIZEOF_LOG_ENTRY + self.extra_offset
            uid = logEntry[0]
            pid = logEntry[1]
            tid = logEntry[2]
            sequence = logEntry[3]
            tv_sec = logEntry[4]
            tv_nsec = logEntry[5]
            msg_len = logEntry[6]
            priority = self.ANDROID_LOG_INFO
            timestamp = self.format_time(tv_sec, tv_nsec)
            if pos + self.SIZEOF_HEADER_T > len(_data):
                break
            tagidx = struct.unpack('<I', _data[pos : pos + self.SIZEOF_HEADER_T])[0] #4 bytes
            pos = pos + self.SIZEOF_HEADER_T
            evt_type, tmpmsg, length = self.get_evt_data(_data,pos)
            pos = pos + length
            if evt_type == -1:
                break
            if evt_type != self.EVENT_TYPE_LIST:
                msg =str(tagidx) + ": " + tmpmsg
                fmt_msg = "%s %5d %5d %5d %c %s\n" % (timestamp,
                    uid, pid, tid, self.filter_pri_to_char(priority), cleanupString(msg))

                ret += fmt_msg
                continue #--> read next log entry
            if pos + self.SIZEOF_EVT_LIST_T > len(_data):
                break
            list_t = struct.unpack('<BB', _data[pos : pos + self.SIZEOF_EVT_LIST_T])
            pos = pos + self.SIZEOF_EVT_LIST_T
            evt_type = list_t[0]
            evt_cnt = list_t[1]
            i = 0
            msg = ""
            while i < evt_cnt:
                evt_type, tmpmsg, length = self.get_evt_data(_data,pos)
                if evt_type == -1:
                    break
                pos = pos + length
                msg = msg + tmpmsg
                if i < evt_cnt -1:
                    msg = msg + ","
                i = i+1
            msg = str(tagidx) + ": [" + msg + "]"
            fmt_msg = "%s %5d %5d %5d %c %s\n" % (timestamp,
                    uid, pid, tid, self.filter_pri_to_char(priority), cleanupString(msg))
            ret += fmt_msg
        return ret

    def process_work_chunk(self, _data, log_id, section, is_binary, write_active):
        if write_active == 0:##parse zipped buffer
            if not self.zstd: # no zstd library
                return log_id, section, None
            try:
                _data = self.zstd.ZstdDecompressor().decompress(_data)
            except:
                _data = None

        ret = None
        if _data:
            if is_binary:
                ret = self.process_binary_log_and_save(_data)
            else:
                ret = self.process_log_and_save(_data)

        return log_id, section, ret

    def save_log_to_file(self, loglist):
        if not loglist or len(loglist) == 0:
            return
        if not self.is_success: # parse success and save to file
            self.is_success = True

        for log_id in loglist.keys():
            sections  = loglist[log_id]
            if not sections:
                continue
            filename = self.get_output_filename(log_id)
            if filename is None:
                return
            log_file = self.ramdump.open_file(filename)
            for section in sorted(sections.keys()):
                if sections[section] and len(sections[section]) >= 0:
                    if section ==0 or len(sections[section]) == 1:
                        head = "{} log buffer used: {}k   Max size:{}k\n".format(
                            self.LOG_NAME[log_id],
                            round(self.sizeUsed[log_id]/1024,1),
                            round(self.maxSize[log_id]/1024,1))
                        log_file.write(head)
                    head="--------- beginning of {} section: {}\n".format(
                        self.LOG_NAME[log_id], str(section))
                    log_file.write(head)
                    log_file.write(sections[section])

    def process_chunklist_and_save(self, logchunk_list_addr):
        log_id = 0
        threads = []
        with futures.ThreadPoolExecutor(8) as executor:
            while log_id <= self.LOG_ID_MAX:
                is_binary = (log_id == self.LOG_ID_EVENTS) or (
                        log_id == self.LOG_ID_STATS) or (log_id == self.LOG_ID_SECURITY)
                #--> address of std::list<SerializedLogChunk>
                _addr = logchunk_list_addr + log_id * 0x18
                #the first element of std::list<SerializedLogChunk>
                first_node_addr = self.read_bytes(_addr + self.addr_length, self.addr_length)
                list_count = self.read_bytes(_addr + self.addr_length *2, self.addr_length )
                section = 0;
                next_node_addr = first_node_addr
                self.sizeUsed[log_id] = 0
                self.maxSize[log_id] = 0
                while (section < list_count):
                    current_node = next_node_addr + self.addr_length * 2 #-->SerializedLogChunk
                    write_offset = self.read_bytes(current_node + 0x10, 4) #--write_offset_
                    write_active = self.read_bytes(current_node + 0x18, 1) #--write_active_
                    _data = None
                    if write_active == 0: ##parse zipped buffer
                        if self.zstd:
                            compressed_log_addr = current_node + 0x28
                            _data_addr = self.read_bytes(compressed_log_addr, self.addr_length)
                            _data_size = self.read_bytes(
                            compressed_log_addr + self.addr_length, self.addr_length)
                            _data = self.read_binary(_data_addr, _data_size)
                            self.sizeUsed[log_id] = self.sizeUsed[log_id] + _data_size
                    else:
                        _data_addr = self.read_bytes(current_node, self.addr_length)
                        _data_size = self.read_bytes(current_node + self.addr_length, self.addr_length)
                        self.sizeUsed[log_id] = self.sizeUsed[log_id] + write_offset
                        self.maxSize[log_id] = _data_size * 4
                        # write_offset is data size for uncompressed secion
                        _data = self.read_binary(_data_addr, write_offset)
                    if _data:
                        future = executor.submit(self.process_work_chunk, _data, log_id, section, is_binary, write_active)
                        threads.append(future)

                    section = section + 1 # next loop
                    next_node_addr = self.read_bytes(next_node_addr + self.addr_length, self.addr_length)
                log_id = log_id + 1

        loglist = {}
        for future in futures.as_completed(threads):
            log_id, section, ret = future.result()
            if not ret:
                continue
            if log_id in loglist:
                sections = loglist[log_id]
            else:
                sections = {}
                loglist[log_id] = sections
            sections[section] = ret
        self.save_log_to_file(loglist)

class Logcat_v3(Logcat_base):
    def __init__(self, ramdump, mmu, logd_task):
        super().__init__(ramdump, mmu)
        self.task = logd_task

    def get_logbuffer_addr(self):
        stack_offset = self.ramdump.field_offset('struct task_struct', 'stack')
        stack_addr = self.ramdump.read_word(self.task + stack_offset)
        pt_regs_size = self.ramdump.sizeof('struct pt_regs')
        pt_regs_addr = self.ramdump.thread_size + stack_addr - pt_regs_size
        user_regs_addr = pt_regs_addr + self.ramdump.field_offset('struct pt_regs', 'user_regs')
        #find x22 register value
        x22_r_addr = self.ramdump.array_index(user_regs_addr, 'unsigned long', 22)
        x22_value    = self.ramdump.read_word(x22_r_addr)
        x22_logbuf_addr = self.read_bytes(x22_value + 0x88, self.addr_length)

        logbuf_addrs = []
        if x22_logbuf_addr and x22_logbuf_addr != 0: # for logd orginal code
            logbuf_addrs.append(x22_logbuf_addr)
            print_out_str("logbuf_addr from x22 = 0x%x" %(x22_logbuf_addr))

        x21_r_addr = self.ramdump.array_index(user_regs_addr, 'unsigned long', 21)
        x21_value    = self.ramdump.read_word(x21_r_addr)
        x21_logbuf_addr = self.read_bytes(x21_value + 0x88, self.addr_length)
        if x21_logbuf_addr and x21_logbuf_addr != 0:
            logbuf_addrs.append(x21_logbuf_addr)
            print_out_str("logbuf_addr from x21 = 0x%x" %(x21_logbuf_addr))

        x23_r_addr = self.ramdump.array_index(user_regs_addr, 'unsigned long', 23)
        x23_value    = self.ramdump.read_word(x23_r_addr)
        x23_logbuf_addr = self.read_bytes(x23_value + 0x88, self.addr_length)
        if x23_logbuf_addr or x23_logbuf_addr == 0:
            logbuf_addrs.append(x23_logbuf_addr)
            print_out_str("logbuf_addr from x23 = 0x%x" %(x23_logbuf_addr))
        return logbuf_addrs

    def parse(self):
        logbuf_addrs = self.get_logbuffer_addr()
        for __logbuf_addr in logbuf_addrs:
            logchunk_list_addr = __logbuf_addr + 0x60
            self.process_chunklist_and_save(logchunk_list_addr)
            if self.is_success:
                print_out_str("logbuf_addr = 0x%x" %(__logbuf_addr))
                break
        return self.is_success

class Logcat_vma(Logcat_base):
    def __init__(self, ramdump, mmu, bin_file):
        super().__init__(ramdump, mmu)
        self.bin_file = bin_file
        self.HEAD_SIZE = 32
        self.vmas = []
        if int(ramdump.get_config_val("CONFIG_BASE_SMALL")) == 0:
            self.PID_MAX = 0x8000
        else:
            self.PID_MAX = 0x1000
        print_out_str("max pid = " + str(self.PID_MAX))

    def read_bytes(self, addr, len):
        addr = addr & 0x0000ffffffffffff
        for vma in self.vmas:
            if addr >= vma["vmstart"] and addr+len <= vma["vmstart"]+vma["size"]:
                offset = addr - vma["vmstart"]
                if len == 8 and offset+8 <=vma["size"]:
                    s = struct.unpack('<Q', vma["data"][offset:offset+8])
                    return s[0]
                elif len == 4 and offset+4 <=vma["size"]:
                    s = struct.unpack('<I', vma["data"][offset:offset+4])
                    return s[0]
                elif len == 2 and offset+2 <=vma["size"]:
                    s = struct.unpack('<H', vma["data"][offset:offset+2])
                    return s[0]
                elif len == 1 and offset+1 <=vma["size"]:
                    s = struct.unpack('<B', vma["data"][offset:offset+1])
                    return s[0]
                else:
                    print_out_str("This api used to unpack 1/2/4/8 bytes data, check the len\n")
                    exit()
        return 0

    def read_binary(self, addr, len):
        addr = addr & 0x0000ffffffffffff
        for vma in self.vmas:
            if addr >= vma["vmstart"] and addr+len <= vma["vmstart"]+vma["size"]:
                offset = addr - vma["vmstart"]
                if offset + len >= vma["size"]:
                    len = vma["size"] - offset
                return vma["data"][offset:offset+len]
        return b''

    def get_all_vmas(self):
        file_path = os.path.join(self.ramdump.outdir, self.bin_file)
        if not os.path.exists(file_path):
            print_out_str(file_path+" not exist!!")
            return

        with self.ramdump.open_file(self.bin_file, "rb") as f:
            size = os.path.getsize(file_path)
            data = f.read(size)
            pos = 0
            while pos < size:
                if pos + self.HEAD_SIZE > size:
                    break
                header = struct.unpack('<QQQQ', data[pos:pos+self.HEAD_SIZE])
                magic = header[0]
                min_addr = header[1]
                vma_size = header[2]
                vma_offset = header[3]
                if pos + self.HEAD_SIZE > size:
                    break
                vma_data = data[vma_offset: vma_offset + vma_size]
                vma = {}
                vma["vmstart"] = min_addr
                vma["size"] = vma_size
                vma["data"] = vma_data
                self.vmas.append(vma)
                pos = pos + self.HEAD_SIZE + vma_size

    def has_valid_log(self, main_chunklist_addr):
        end_node_addr = self.read_bytes(main_chunklist_addr, self.addr_length)
        current_node = end_node_addr + self.addr_length * 2 #-->SerializedLogChunk
        _data_addr = self.read_bytes(current_node, self.addr_length)
        _data_size = self.read_bytes(current_node + self.addr_length, self.addr_length)
        if _data_size <= self.SIZEOF_LOG_ENTRY:
            return False

        valid = self.has_valid_LogEntrys(_data_addr, _data_size, 10)
        if valid is False: ## retry
            extra_log_entry_offset = 4
            valid = self.has_valid_LogEntrys(_data_addr, _data_size, 10, extra_log_entry_offset)
            if valid:
                self.extra_offset = extra_log_entry_offset
        return valid

    def has_valid_LogEntrys(self, _addr, _data_size, valid_count, extra_offset=0):
        i = 0
        valid = False
        offset = 0
        while i < valid_count:
            valid, length = self.is_valid_LogEntry(_addr+offset)
            if not valid:
                return False
            offset = offset + length + extra_offset
            if offset > _data_size:
                return True
            i = i + 1
        return True

    def is_valid_LogEntry(self, _addr):
        uid = self.read_bytes(_addr, 4)
        pid = self.read_bytes(_addr + 0x4, 4)
        tid = self.read_bytes(_addr + 0x8, 4)
        sequence = self.read_bytes(_addr + 0xc, 8)
        tv_sec = self.read_bytes(_addr + 0x14, 4)
        tv_nsec = self.read_bytes(_addr + 0x18, 4)
        msg_len = self.read_bytes(_addr + 0x1c, 2)
        priority = self.read_bytes(_addr + self.SIZEOF_LOG_ENTRY, 1)
        if pid <= self.PID_MAX and uid <= self.PID_MAX and priority \
            <= self.ANDROID_LOG_SILENT and priority >= self.ANDROID_LOG_VERBOSE:
            if  msg_len >=1 and  msg_len <= 4068: #max_payload
                return True, 30 + msg_len
        return False, 0

    def parse(self):
        startTime = datetime.datetime.now()
        self.get_all_vmas()
        # find address of std::list<SerializedLogChunk>
        chunklist_addr = 0
        for vma in self.vmas:
            offset = self.find_log_chunklist_addr(vma)
            if offset != 0:
                chunklist_addr = vma["vmstart"]+offset
                break
        if chunklist_addr == 0:
            return False
        # start parsing
        is_valid_chunklist = self.has_valid_log(chunklist_addr + self.LOG_ID_MAIN * 0x18)
        if not is_valid_chunklist:
            print_out_str("There is no valid log in logbuf_addr = 0x%x" %(chunklist_addr-0x60))
            return False
        # start parsing
        print("LogBuffer address",hex(chunklist_addr-0x60))
        self.process_chunklist_and_save(chunklist_addr)
        print_out_str("logbuf_addr = 0x%x" %(chunklist_addr-0x60))
        print("logcat_vma parse logcat cost "+str((datetime.datetime.now()-startTime).total_seconds())+" s")
        return self.is_success

    def find_log_chunklist_addr(self, vma):
        vma_size = vma["size"]
        vma_data = vma["data"]
        vma_startaddr = vma["vmstart"]
        offset = 0
        is_chunk = False
        while offset < vma_size:
            is_chunk = self.is_log_chunklist_addr(vma_data, offset)
            if is_chunk:
                log_id = 1
                while log_id <= self.LOG_ID_MAX:
                    is_chunk = self.is_log_chunklist_addr(vma_data, offset+0x18*log_id)
                    if not is_chunk:
                        return 0
                    log_id = log_id + 1
                break
            offset = offset + 4
        if is_chunk:
            return offset
        return 0

    def is_log_chunklist_addr(self, vma_data, offset):
        if offset+24 > len(vma_data):
            return False
        nodes = struct.unpack('<QQQ', vma_data[offset:offset+24])
        tail_node_addr = nodes[0]
        head_node_addr = nodes[1]
        list_count = nodes[2]
        if tail_node_addr ==0  or head_node_addr ==0 or list_count > 1000:
            return False
        next_node_addr = head_node_addr
        prev_node = self.read_bytes(head_node_addr, self.addr_length) # prev_node = next_node_addr->prev
        if prev_node == 0:
            return False
        if list_count == 0 and head_node_addr == tail_node_addr: # empty list
            return True
        index = 0
        while next_node_addr != 0 and index < list_count:
            next_prev_node = self.read_bytes(next_node_addr, self.addr_length)
            if not next_prev_node or next_prev_node != prev_node:
                return False
            current_node = next_node_addr + self.addr_length *2
            is_chunk = self.is_log_chunk_addr(current_node)
            if not is_chunk:
                 return False
            if next_node_addr == tail_node_addr: # loop complete
                return True
            prev_node = next_node_addr
            next_node_addr = self.read_bytes(next_node_addr + self.addr_length, self.addr_length)
            index = index +1
        return False

    def is_log_chunk_addr(self, addr):
        data_addr = self.read_bytes(addr, self.addr_length)
        write_offset = self.read_bytes(addr + 0x10, 4) #--write_offset_
        write_active = self.read_bytes(addr + 0x18, 1) #--write_active_
        data_size = self.read_bytes(addr + self.addr_length, self.addr_length)
        compress_data_addr = self.read_bytes(addr + 0x28, self.addr_length)
        compress_data_size = self.read_bytes(addr + 0x28 + self.addr_length, self.addr_length)
        if (write_active == 1 and  data_addr != 0 and (data_size
                !=0) and write_offset !=0   and write_offset < data_size): #uncompressed chunk
            return True
        elif write_active == 0 and compress_data_addr != 0 and (compress_data_size
                        != 0) and write_offset !=0 and compress_data_size<write_offset: #compressed chunk
            return True
        return False
