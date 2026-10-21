# Copyright (c) 2018, 2020-2021 The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from parser_util import cleanupString
from parser_util import register_parser, RamParser
from print_out import print_out_str
import struct
import datetime
import re
import collections

#PERSISTENT_RAM_SIG /* DBGC */
PERSISTENT_RAM_SIG_SIZE = 4

@register_parser('--pstore', 'Extract event logs from pstore')
class PStore(RamParser):

    def calculate_percpu_eventbuf_size(self, base_addr):
        try:
            event_zone_addr = self.ramdump.read_u64(base_addr +
                              self.ramdump.field_offset('struct ramoops_context', 'eprzs'))
            event_zone_addr = self.ramdump.read_u64(event_zone_addr)
            start_addr = self.ramdump.read_u64(event_zone_addr +
                         self.ramdump.field_offset('struct persistent_ram_zone', 'paddr'))
            percpu_size = self.ramdump.read_u64(event_zone_addr +
                          self.ramdump.field_offset('struct persistent_ram_zone', 'size'))
        except:
            return None, None
        return start_addr, percpu_size

    def print_console_logs(self, pstore_out, addr, size):
        pstore = self.ramdump.read_physical(addr, size)
        pstore_out.write(cleanupString(pstore.decode('ascii', 'ignore')) + '\n')

    def sort_event_data(self, event_data, pstore_out):
        '''
        Event log buffer is circular, so this function
        sorts the logs in ascending timestamp manner
        '''
        ordered_event_data = collections.OrderedDict(sorted(event_data.items()))
        for ts, log in ordered_event_data.items():
            pstore_out.write(log)

    def print_event_logs(self, pstore_out, addr, size):
        '''
        This function tries to format the event logs before logging them to
        the core specific rtb file. Raw Data will be in the format given below
        io_read: type=readl cpu=1 ts:58270610802 data=0xffffff8009de8614
                 caller=qcom_geni_serial_start_tx+0x114/0x150
        io_write: type=writel cpu=1 ts:58270618875 data=0xffffff8009de880c
                 caller=qcom_geni_serial_start_tx+0x130/0x150

        Timestamp is extracted from raw data and converted to secs format and cpu
        field is removed since it is redundant.

        Final Formatting will be as shown below
        [644.279442] io_write : writel from address 0xffffff8009673120(None) called
                     from qcom_cpufreq_hw_target_index+0x60/0x64
        '''
        pstore = self.ramdump.read_physical(addr, size)
        event_log_data = cleanupString(pstore.decode('ascii', 'ignore'))
        event_data = event_log_data.split('\n')
        formatted_event_data = {}
        for line in event_data:
            expr = r'.*(io_.*):.*type=(.*)cpu=(.*)ts:(.*)data=(.*)caller=(.*).*'
            regEx = re.search(expr, line)
            if regEx:
                event_type = regEx.group(2).strip()
                timestamp = regEx.group(4).strip()
                timestamp = round(float(timestamp)/10**9,9)
                timestamp = format(timestamp,'.9f')
                data = regEx.group(5).strip()
                caller = regEx.group(6).strip()
                log_string = "[{0}] {1} : {2} from address {3} called from {4}\n".format(timestamp,
                              regEx.group(1), event_type, data, caller)
                formatted_event_data[timestamp] = log_string
            else:
                continue
        self.sort_event_data(formatted_event_data, pstore_out)

    def calculate_console_size(self, base_addr):
        size = self.ramdump.read_u64(base_addr +
                            self.ramdump.field_offset('struct ramoops_context', 'console_size'))
        if size == 0:
            return None, size

        console_zone_addr = self.ramdump.read_u64(base_addr +
                            self.ramdump.field_offset('struct ramoops_context', 'cprz'))
        start_addr = self.ramdump.read_u64(console_zone_addr +
                     self.ramdump.field_offset('struct persistent_ram_zone', 'paddr'))
        console_buf = self.ramdump.read_u64(console_zone_addr +
                      self.ramdump.field_offset('struct persistent_ram_zone', 'buffer'))
        console_size = self.ramdump.read_u32(console_buf +
                       self.ramdump.field_offset('struct persistent_ram_zone', 'size'))
        header = self.ramdump.sizeof('((struct persistent_ram_zone *)0x0)->paddr')
        return start_addr + PERSISTENT_RAM_SIG_SIZE, console_size + header

    def extract_console_logs(self, base_addr):
        '''
        Parses the console logs from pstore
        '''
        start_addr, console_size = self.calculate_console_size(base_addr)
        if start_addr is None:
            return
        pstore_out = self.ramdump.open_file('console_logs.txt')
        self.print_console_logs(pstore_out, start_addr, console_size)
        pstore_out.close()

    def extract_io_event_logs(self, base_addr):
        '''
        Parses the RTB data (register read/writes) stored in the persistent
        ram zone. Data is extracted on per cpu basis into separate per core
        files.
        '''
        start_addr, percpu_size = self.calculate_percpu_eventbuf_size(base_addr)
        if start_addr is None:
            return
        nr_cpus = self.ramdump.get_num_cpus()
        for cpu in range(0,nr_cpus):
            pstore_out = self.ramdump.open_file('msm_rtb{0}.txt'.format(cpu))
            cpu_offset = percpu_size*cpu
            self.print_event_logs(pstore_out, start_addr+cpu_offset, percpu_size)
            pstore_out.close()

    def print_clockdata_info(self):
        '''
        Extracts the epoch data from clock data struct. This helps in converting
        HLOS timestamps to non-HLOS timestamps and vica-versa. This also
        additionally logs the Linux Banner for ease in post-processing.
        '''
        out_file = self.ramdump.open_file('epoch_info.txt')
        banner_addr = self.ramdump.address_of('linux_banner')
        if banner_addr is not None:
            banner_addr = self.ramdump.kernel_virt_to_phys(banner_addr)
            vm_v = self.ramdump.gdbmi.get_value_of_string('linux_banner')
            if vm_v is not None:
                out_file.write('Linux Banner : {}\n'.format(vm_v))
        epoch_ns = self.ramdump.read_word('cd.read_data[0].epoch_ns')
        epoch_cyc = self.ramdump.read_word('cd.read_data[0].epoch_cyc')
        out_file.write('\nepoch_ns: {0}ns  epoch_cyc: {1}\n'.format(epoch_ns,epoch_cyc))
        out_file.close()

    def extract_pmsg_logs(self, pmsg, pmsg_size):
        '''
        Parses the pmsg log stored in the persistent
        ram zone.
        '''
        log_type_list = ["0:main", "1:radio", "2:event", "3:system", "4:crash",
                            "5:stats", "6:security", "7:kernel"]
        priority_list = ["0:UNKNOWN", "1:DEFAULT", "2:VERBOSE", "3:DEBUG", "4:INFO",
                            "5:WARNING", "6:ERROR", "7:FATAL", "8:SILENT"]
        pmsg_out = self.ramdump.open_file('pmsg_logs.txt')
        pmsg_out.write("timestamp".ljust(26))
        pmsg_out.write("uid".rjust(6))
        pmsg_out.write("pid".rjust(6))
        pmsg_out.write("tid".rjust(6))
        pmsg_out.write("  ")
        pmsg_out.write("log_type".ljust(10))
        pmsg_out.write("level".ljust(12))
        pmsg_out.write("message")
        pmsg_out.write("\n")

        next_addr = 4
        content_err = False
        while next_addr < (pmsg_size - 19):
            magic, len, uid, pid, id, tid, tv_sec, tv_nsec, outtag = \
                    struct.unpack('<c3HbH2Ib', pmsg[next_addr:next_addr + 19])
            if magic == b'l' and 0 <= outtag < 9 and 0 <= id < 7 and len > 19 and \
                    (content_err or pmsg[next_addr-1:next_addr] == b'\x00'):
                tv_nsec = str(tv_nsec // 1000)
                tv_nsec = tv_nsec.zfill(6)
                date = datetime.datetime.utcfromtimestamp(tv_sec)
                timestamp = date.strftime("%y-%m-%d %H:%M:%S") + '.' + tv_nsec
                msg = pmsg[next_addr + 19:next_addr + len].replace(b'\x0a', b'\x20')
                msg = msg.replace(b'\x00', b'\x20')
                pmsg_out.write(timestamp.ljust(26))
                pmsg_out.write(str(uid).rjust(6))
                pmsg_out.write(str(pid).rjust(6))
                pmsg_out.write(str(tid).rjust(6))
                pmsg_out.write("  ")
                pmsg_out.write(str(log_type_list[id]).ljust(10))
                pmsg_out.write(str(priority_list[outtag]).ljust(12))
                if next_addr + len < pmsg_size:
                    if pmsg[next_addr + len:next_addr + len + 1] != b'l':
                        content_err = True
                        next_addr += 19
                        pmsg_out.write("Content Missing!\n")
                        continue
                pmsg_out.write(msg.decode("ascii","ignore"))
                pmsg_out.write("\n")
                content_err = False
                next_addr += len
            else:
                next_addr += 1
        pmsg_out.close()

    def extract_pmsg_logs_fd(self, base_addr):
        size = self.ramdump.read_u64(base_addr +
                            self.ramdump.field_offset('struct ramoops_context', 'pmsg_size'))
        if size == 0:
            return

        pmsg_zone_addr = self.ramdump.read_u64(base_addr +
                    self.ramdump.field_offset('struct ramoops_context', 'mprz'))
        start_addr = self.ramdump.read_u64(pmsg_zone_addr +
                    self.ramdump.field_offset('struct persistent_ram_zone', 'paddr'))
        pmsg_size = self.ramdump.read_u64(pmsg_zone_addr +
                    self.ramdump.field_offset('struct persistent_ram_zone', 'size'))

        zone_buffer_addr = self.ramdump.read_u64(pmsg_zone_addr +
                    self.ramdump.field_offset('struct persistent_ram_zone', 'buffer'))

        buffer_start = self.ramdump.read_u32(zone_buffer_addr +
                    self.ramdump.field_offset('struct persistent_ram_buffer', 'start'))
        buffer_size = self.ramdump.read_u32(zone_buffer_addr +
                    self.ramdump.field_offset('struct persistent_ram_buffer', 'size'))
        pmsg = b""
        if buffer_start < buffer_size:
            pmsg = self.ramdump.read_physical(start_addr+buffer_start, buffer_size-buffer_start)
            pmsg = pmsg + self.ramdump.read_physical(start_addr, buffer_start)
        else:
            pmsg = self.ramdump.read_physical(start_addr, buffer_size)
        self.extract_pmsg_logs(pmsg, buffer_size)

    def parse(self):
        if not self.ramdump.minidump:
            if not self.ramdump.is_config_defined('CONFIG_PSTORE'):
                print_out_str('CONFIG_PSTORE is not defined')
                return
            base_addr = self.ramdump.address_of('oops_cxt')
            self.extract_io_event_logs(base_addr)
            self.extract_console_logs(base_addr)
            self.print_clockdata_info()
            self.extract_pmsg_logs_fd(base_addr)
        else:
            pmsg_addr = list(self.ramdump.ebi_pa_name_map.keys())[
                list(self.ramdump.ebi_pa_name_map.values()).index("KPMSG")]
            if pmsg_addr is not None:
                for idx, pa, end, va, size in self.ramdump.ebi_files_minidump:
                    if pa == pmsg_addr:
                        break
                pmsg_cxt = self.ramdump.read_physical(pmsg_addr, size)
                self.extract_pmsg_logs(pmsg_cxt, size)
