# Copyright (c) 2020-2021 The Linux Foundation. All rights reserved.
# Copyright (c) 2022-2025 Qualcomm Innovation Center, Inc. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from collections import OrderedDict
import re

from parser_util import register_parser, RamParser
from print_out import print_out_str
from tempfile import NamedTemporaryFile
from struct_print import struct_print_class

comm_pid_dict = {}

softirq_action_list = {}
softirq_action_list[0] = "HI_SOFTIRQ"
softirq_action_list[1] = "TIMER_SOFTIRQ"
softirq_action_list[2] = "NET_TX_SOFTIRQ"
softirq_action_list[3] = "NET_RX_SOFTIRQ"
softirq_action_list[4] = "BLOCK_IOPOLL_SOFTIRQ"
softirq_action_list[5] = "TASKLET_SOFTIRQ"
softirq_action_list[6] = "TASKLET_SOFTIRQ"
softirq_action_list[7] = "SCHED_SOFTIRQ"
softirq_action_list[8] = "HRTIMER_SOFTIRQ"
softirq_action_list[9] = "RCU_SOFTIRQ"
softirq_action_list[10] = "NR_SOFTIRQS"


TRACE_FLAG_IRQS_OFF = 0x01
TRACE_FLAG_IRQS_NOSUPPORT = 0x02
TRACE_FLAG_NEED_RESCHED = 0x04
TRACE_FLAG_HARDIRQ = 0x08
TRACE_FLAG_SOFTIRQ = 0x10
TRACE_FLAG_PREEMPT_RESCHED = 0x20
TRACE_FLAG_NMI = 0x40
TRACE_FLAG_BH_OFF = 0x80

class BufferedWrite(object):
    """
        Helper class to facilitate batch
        writes into the files.
        Default batch size is 8000
    """
    def __init__(self, fdesc):
        self.buffer = []
        self.fdesc = fdesc
        self.batchsize = 8000
        self.count = 0

    def write(self, data):
        self.buffer.append(data)
        self.count += 1
        if self.count >= 8000:
            self.flush()
            self.count = 0

    def flush(self):
        if len(self.buffer):
            self.fdesc.write("".join(self.buffer))
            self.buffer = []

    def close(self):
        if self.fdesc:
            self.flush()
            self.fdesc.close()
            self.fdesc = None

    def __del__(self):
        if self.fdesc:
            self.close()

class FtraceParser_Event(object):
    def __init__(self,ramdump,ftrace_out,cpu,trace_event_type,ftrace_raw_struct_type,ftrace_time_data,format_event_map,savedcmd):
        self.cpu = "[{:03d}]".format(cpu)
        self.ramdump = ramdump
        self.ftrace_out = ftrace_out
        self.nr_ftrace_events = 0
        self.ftrace_event_type = trace_event_type
        self.ftrace_raw_struct_type = ftrace_raw_struct_type
        self.ftrace_time_data = ftrace_time_data
        self.format_event_map = format_event_map
        self.buffer_page_real_end_offset = self.ramdump.field_offset(
            'struct buffer_page ', 'real_end')
        self.buffer_page_data_page_offset = self.ramdump.field_offset(
            'struct buffer_page ', 'page')
        self.buffer_data_page_commit_offset = self.ramdump.field_offset(
            'struct buffer_data_page ', 'commit')
        self.buffer_data_page_time_stamp_offset = self.ramdump.field_offset(
            'struct buffer_data_page ', 'time_stamp')
        self.buffer_data_page_data_offset = self.ramdump.field_offset(
            'struct buffer_data_page ', 'data')
        self.rb_event_array_offset = self.ramdump.field_offset(
            'struct ring_buffer_event', 'array')
        self.rb_event_timedelta_offset = self.ramdump.field_offset(
            'struct ring_buffer_event', 'time_delta')
        self.rb_event_typelen_offset = self.ramdump.field_offset(
            'struct ring_buffer_event', 'type_len')
        self.trace_entry_type_offset = self.ramdump.field_offset('struct trace_entry ', 'type')
        self.pid_max = self.ramdump.read_int("pid_max")
        self.map_cmdline_to_pid_offset = self.ramdump.field_offset(
            'struct saved_cmdlines_buffer', 'map_cmdline_to_pid')
        self.saved_cmdlines_offset = self.ramdump.field_offset(
            'struct saved_cmdlines_buffer', 'saved_cmdlines')
        self.pid_offset = self.ramdump.field_offset("struct trace_entry" , "pid")
        self.preempt_count_offset = self.ramdump.field_offset("struct trace_entry", "preempt_count")
        self.flags_offset = self.ramdump.field_offset("struct trace_entry", "flags")
        self.comm_pid_dict = comm_pid_dict
        self.savedcmd = savedcmd

    def get_event_length(self, rb_event, rb_event_type, time_delta,  buffer_data_page_end):
        type_len = rb_event_type
        if(type_len == 0):
            length = self.ramdump.read_u32(rb_event + self.rb_event_array_offset)
            return length
        elif(type_len <= 28):
            return (type_len << 2)
        elif(type_len == 29):
            if(time_delta == 1):
                length = self.ramdump.read_u32(rb_event + self.rb_event_array_offset)
                return length
            else:
                if rb_event > buffer_data_page_end:
                    print_out_str("rb_event({}) is bigger than buffer_data_page_end({})".format(hex(rb_event), hex(buffer_data_page_end)))
                    return -1
                return buffer_data_page_end - rb_event #Padding till end of page
        elif(type_len == 30):
            # Accounts for header size + one u32 array entry
            return 8
        elif(type_len == 31):
            return 8
        else:
            print_out_str("Unknown type_len {}".format(type_len))
            return -1

    def parse_buffer_page_entry(self, buffer_page_entry):
        buffer_data_page = None
        buffer_data_page_end = None
        #buffer_data_page_data_offset = None
        rb_event = None
        rb_event_timestamp = 0
        time_delta = 0
        record_length = 0
        #rb_event_array_offset = 0
        tr_entry  = None
        tr_event_type = None
        commit = 0

        '''
        struct buffer_page {
            [0x0] struct list_head list;
            [0x10] local_t write;
            [0x18] unsigned int read;
            [0x20] local_t entries;
            [0x28] unsigned long real_end;
            [0x30] struct buffer_data_page *page;
        }
        '''
        buffer_data_page = self.ramdump.read_pointer(buffer_page_entry + self.buffer_page_data_page_offset)

        '''
        struct buffer_data_page {
            [0x0] u64 time_stamp;
            [0x8] local_t commit;
            [0x10] unsigned char data[];
        }
        '''
        commit = 0
        commit = self.ramdump.read_pointer(
            buffer_data_page + self.buffer_data_page_commit_offset)
        if commit and commit > 0:
            buffer_data_page_end = buffer_data_page + commit
            time_stamp = self.ramdump.read_u64(
                buffer_data_page + self.buffer_data_page_time_stamp_offset)
            rb_event = buffer_data_page + self.buffer_data_page_data_offset
            total_read = 0
            while (total_read < commit):
                time_delta = self.ramdump.read_u32(rb_event + self.rb_event_timedelta_offset)
                time_delta = time_delta >> 5
                # print_out_str("time_delta after = {0} ".format(time_delta))
                rb_event_timestamp = rb_event_timestamp + time_delta

                rb_event_length_old = self.ramdump.read_u32(rb_event + self.rb_event_typelen_offset)
                rb_event_type = (((1 << 5) - 1) & rb_event_length_old);

                record_length = self.get_event_length(rb_event, rb_event_type, time_delta,  buffer_data_page_end)
                if record_length == -1:
                    break
                #print("rb_event_type is ", rb_event_type)
                if rb_event_type == 0:
                    # This could be that type_len * 4 > 112
                    # so type_len is set to 0 and 32 bit array filed holds length
                    # while payload starts afterwards at array[1]
                    tr_entry = rb_event + self.rb_event_array_offset + 0x4
                    tr_event_type = self.ramdump.read_u16( tr_entry + self.trace_entry_type_offset)
                    if tr_event_type < self.nr_ftrace_events:
                        #self.ftrace_out.write("unknown event \n")
                        pass
                    else:
                        self.parse_trace_entry(tr_entry, tr_event_type, time_stamp + rb_event_timestamp)
                    record_length = record_length + 0x4   #Header Size

                elif rb_event_type <= 28: #Data Events
                    tr_entry = rb_event + self.rb_event_array_offset
                    tr_event_type = self.ramdump.read_u16(tr_entry + self.trace_entry_type_offset)
                    if tr_event_type < self.nr_ftrace_events:
                        #self.ftrace_out.write("unknown event \n")
                        pass
                    else:
                        self.parse_trace_entry(tr_entry, tr_event_type, time_stamp + rb_event_timestamp)
                    record_length = record_length + 0x4

                elif rb_event_type == 29:
                    """
                        Padding event or discarded event
                        time_delta here can be 0 or 1
                        time delta is set 0 when event is bigger than minimum size (8 bytes)
                        in this case we consider rest of the page as padding

                        time delta is set to 1 for discarded event
                        Here the size is stored in array[0]
                    """
                    record_length = record_length + 0x4
                    pass

                elif rb_event_type == 30:
                    # This is a time extend event so we need to use the 32 bit field from array[0](28..59)
                    # if time delta actually exceeds 2^27 nanoseconds which is > what 27 bit field can hold
                    # We are accounting for a complete time stamp stored in this field (59 bits)
                    rb_event_timestamp = rb_event_timestamp + (self.ramdump.read_u32(rb_event + self.rb_event_array_offset) << 27)

                elif rb_event_type == 31:
                    # Accounts for an absolute timestamp
                    rb_event_timestamp = 0
                rb_event = rb_event + record_length
                total_read += record_length
                #alignment = 4 - (rb_event % 4)
                #rb_event += alignment

    def find_cmdline(self, pid):
        comm = "<TBD>"
        if self.savedcmd is not None:
            if pid == 0:
                comm = "<idle>"
            else:
                tpid = pid & (self.pid_max - 1)
                cmdline_map = self.savedcmd.map_pid_to_cmdline[tpid]
                if cmdline_map != -1 and cmdline_map != None:
                    map_cmdline_to_pid = self.savedcmd.map_cmdline_to_pid
                    cmdline_tpid = self.ramdump.read_int(map_cmdline_to_pid + cmdline_map * 4)
                    if cmdline_tpid == pid:
                        saved_cmdlines = self.savedcmd.saved_cmdlines
                        comm = self.ramdump.read_cstring(saved_cmdlines + cmdline_map * 16, 16) #TASK_COMM_LEN
        comm = "{}-{}".format(comm, pid)
        self.comm_pid_dict[pid] = comm
        return comm

    def get_lat_fmt(self, flags, preempt_count):
        lat_fmt = ''

        nmi = flags & TRACE_FLAG_NMI
        hardirq = flags & TRACE_FLAG_HARDIRQ
        softirq = flags & TRACE_FLAG_SOFTIRQ
        bh_off = flags & TRACE_FLAG_BH_OFF
        if (flags & TRACE_FLAG_IRQS_OFF) and bh_off:
            irqs_off = 'D'
        elif (flags & TRACE_FLAG_IRQS_OFF):
            irqs_off = 'd'
        elif bh_off:
            irqs_off = 'b'
        elif (flags & TRACE_FLAG_IRQS_NOSUPPORT):
            irqs_off = 'X'
        else:
            irqs_off = '.'
        resched = flags & (TRACE_FLAG_NEED_RESCHED | TRACE_FLAG_PREEMPT_RESCHED)
        if (resched == (TRACE_FLAG_NEED_RESCHED | TRACE_FLAG_PREEMPT_RESCHED)):
            need_resched = 'N'
        elif (resched == TRACE_FLAG_NEED_RESCHED):
            need_resched = 'n'
        elif (resched == TRACE_FLAG_PREEMPT_RESCHED):
            need_resched = 'p'
        else:
            need_resched = '.'

        if nmi and hardirq:
            hardsoft_irq = 'Z'
        elif nmi:
            hardsoft_irq = 'z'
        elif hardirq and softirq:
            hardsoft_irq = 'H'
        elif hardirq:
            hardsoft_irq = 'h'
        elif softirq:
            hardsoft_irq = 's'
        else:
            hardsoft_irq = '.'

        lat_fmt += irqs_off
        lat_fmt += need_resched
        lat_fmt += hardsoft_irq

        if (preempt_count & 0xf):
            lat_fmt += '{0:x}'.format((preempt_count & 0xf))
        else:
            lat_fmt += '.'
        if (preempt_count & 0xf0):
            lat_fmt += '{0:x}'.format((preempt_count >> 4))
        else:
            lat_fmt += '.'
        return lat_fmt


    def parse_trace_entry(self, entry, type, time):
        ftrace_raw_entry = None
        event_name = ""
        local_timestamp = None
        pid  = 0
        preempt_count = 0
        struct_type = None
        next_comm = None
        next_pid  = 0
        next_prio = 0
        work = None
        print_ip = None
        print_buffer = None
        vector = None

        local_timestamp = time / 1000000000.0
        if not (local_timestamp in self.ftrace_time_data):
            self.ftrace_time_data[local_timestamp] = []
        #print("type = {0}".format(type))
        if str(type) not in self.ftrace_event_type:
            #print_out_str("unknown event type = {0}".format(str(type)))
            return
        event_name = str(self.ftrace_event_type[str(type)])
        #print("event_name  {0}".format(event_name))
        if event_name is None or event_name == 'None' or 'None' in event_name or len(event_name) <= 1:
            return
        ftrace_raw_entry = entry
        struct_type = self.ftrace_raw_struct_type[str(type)]

        pid = self.ramdump.read_u32(ftrace_raw_entry + self.pid_offset)
        if pid > self.pid_max:
            return
        preempt_count = self.ramdump.read_u16(ftrace_raw_entry + self.preempt_count_offset) & 0xFF
        flags = self.ramdump.read_u16(ftrace_raw_entry + self.flags_offset) & 0xFF
        if pid in self.comm_pid_dict.keys():
            comm = self.comm_pid_dict[pid]
        else:
            comm = self.find_cmdline(pid)
        lat_fmt = self.get_lat_fmt(flags, preempt_count)
        head_data_fmt = " {0:>25}   {1} {2}  {3:.6f}: {4:<25}".format(comm,self.cpu,lat_fmt,
                                        round(local_timestamp,6),"{}:".format(event_name))
        #print("ftrace_raw_entry of {0} = {1}".format(event_name, hex(ftrace_raw_entry)))
        if event_name == "scm_call_start":
            trace_event_raw_offset = self.ramdump.field_offset('struct ' + struct_type, "x0")
            trace_event_raw_next_comm = self.ramdump.field_offset('struct ' + struct_type, "arginfo")
            trace_event_raw_next_pid = self.ramdump.field_offset('struct ' + struct_type, "args")
            trace_event_raw_next_prio = self.ramdump.field_offset('struct ' + struct_type, "x5")
            x0 = self.ramdump.read_pointer(ftrace_raw_entry + trace_event_raw_offset)
            arginfo = self.ramdump.read_u32(ftrace_raw_entry + trace_event_raw_next_comm)
            args = (ftrace_raw_entry + trace_event_raw_next_pid)
            x5 = self.ramdump.read_pointer(ftrace_raw_entry + trace_event_raw_next_prio)
            arr = []
            ptr_size = self.ramdump.sizeof('void *')
            for i in range(1, 9):
                ptr = self.ramdump.read_pointer(args + (i*ptr_size))
                arr.append(hex(ptr))
            temp_data = "{0} func id={1}:(args:{2}, {3}, {4} ,{5} ,{6})\n".format(
                            head_data_fmt,hex(x0),hex(arginfo),arr[0],arr[1],arr[2],hex(x5))
            self.ftrace_time_data[local_timestamp].append(temp_data)
            #self.ftrace_out.write(" <TBD>-{}".format(temp_data))
        elif event_name == "scm_call_end":
            trace_event_raw_offset = self.ramdump.field_offset('struct ' + struct_type, "ret")
            rets = self.ramdump.read_pointer(ftrace_raw_entry + trace_event_raw_offset)
            arr = []
            ptr_size = self.ramdump.sizeof('void *')
            for i in range(1, 4):
                ptr = self.ramdump.read_pointer(rets + (i*ptr_size))
                arr.append(ptr)
            temp_data = "{0} ret:{1}, {2}, {3}\n)\n".format(head_data_fmt,arr[0],arr[1],arr[2])
            self.ftrace_time_data[local_timestamp].append(temp_data)
        elif event_name == "sched_switch":
                trace_event_raw_offset = self.ramdump.field_offset('struct ' + struct_type, "prev_state")
                trace_event_raw_next_comm = self.ramdump.field_offset('struct ' + struct_type, "next_comm")
                trace_event_raw_next_pid = self.ramdump.field_offset('struct ' + struct_type, "next_pid")
                trace_event_raw_next_prio = self.ramdump.field_offset('struct ' + struct_type, "next_prio")
                trace_event_raw_prev_comm = self.ramdump.field_offset('struct ' + struct_type, "prev_comm")
                trace_event_raw_prev_pid = self.ramdump.field_offset('struct ' + struct_type, "prev_pid")
                trace_event_raw_prev_prio = self.ramdump.field_offset('struct ' + struct_type, "prev_prio")
                trace_event_raw_prev_state = self.ramdump.field_offset('struct ' + struct_type, "prev_state")
                next_comm = self.ramdump.read_cstring(ftrace_raw_entry + trace_event_raw_next_comm)
                next_pid = self.ramdump.read_u32(ftrace_raw_entry + trace_event_raw_next_pid)
                next_prio = self.ramdump.read_u32(ftrace_raw_entry + trace_event_raw_next_prio)
                prev_comm = self.ramdump.read_cstring(ftrace_raw_entry + trace_event_raw_prev_comm)
                prev_pid = self.ramdump.read_u32(ftrace_raw_entry + trace_event_raw_prev_pid)
                prev_prio = self.ramdump.read_u32(ftrace_raw_entry + trace_event_raw_prev_prio)
                prev_state1 = self.ramdump.read_u32(ftrace_raw_entry + trace_event_raw_prev_state)
                prev_state_info = (prev_state1 & ((((0x0000 | 0x0001 | 0x0002 | 0x0004 | 0x0008 | 0x0010 | 0x0020 | 0x0040) + 1) << 1) - 1))
                if ( prev_state_info == 0):
                    prev_state_info = "R"
                elif ( prev_state_info == 1):
                    prev_state_info = "S"
                elif ( prev_state_info == 2):
                    prev_state_info = "D"
                elif ( prev_state_info == 4):
                    prev_state_info = "T"
                elif ( prev_state_info == 8):
                    prev_state_info = "t"
                elif ( prev_state_info == 16):
                    prev_state_info = "X"
                elif ( prev_state_info == 32):
                    prev_state_info = "Z"
                elif ( prev_state_info == 64):
                    prev_state_info = "P"
                elif ( prev_state_info == 128):
                    prev_state_info = "I"
                prev_state_info2 = ""
                if prev_state_info:
                    prev_state_info2 = "+"
                temp_data = "{0} prev_comm={1} prev_pid={2} prev_prio={3} prev_state={4} ==> next_comm={5} next_pid={6} next_prio={7}\n".format(
                                head_data_fmt,prev_comm,prev_pid,prev_prio,prev_state_info,next_comm,next_pid,next_prio)
                self.ftrace_time_data[local_timestamp].append(temp_data)
        elif event_name == "softirq_raise":
            trace_event_softirq_vec_offset = self.ramdump.field_offset('struct ' + 'trace_event_raw_softirq', "vec")
            if trace_event_softirq_vec_offset is not None:
                vector = self.ramdump.read_u32(ftrace_raw_entry + trace_event_softirq_vec_offset)
                try:
                    action = softirq_action_list[vector]
                except Exception as err:
                    print_out_str("failed to find a softirq action = {0}".format(vector))
                    action = "unknown vector"
                temp_data = "{0} vec={1} [action={2}]\n".format(head_data_fmt,vector,action)
            else:
                action = "unknown vector"
                temp_data = "{0} vec=unknown\n".format(head_data_fmt)
            self.ftrace_time_data[local_timestamp].append(temp_data)
        elif event_name == "workqueue_activate_work":
            trace_event_raw_work_offset = self.ramdump.field_offset('struct ' + 'trace_event_raw_workqueue_activate_work', "work")
            function_offset = self.ramdump.field_offset('struct ' + 'work_struct', "func")
            if trace_event_raw_work_offset is not None:
                work = self.ramdump.read_pointer(ftrace_raw_entry + trace_event_raw_work_offset)
                function = self.ramdump.read_pointer(work + function_offset)
                if function != None:
                    function_name = self.ramdump.unwind_lookup(function)
                    if function_name == None:
                        function_name = 'none'
                else:
                    function = 0
                    function_name = 'none'
                temp_data = "{0} work struct {1} function 0x{2:x} {3}\n".format(head_data_fmt,
                             str(hex(work)).replace("L",""),function,function_name)
            else:
                temp_data = "{0} work struct unknown\n".format(head_data_fmt)
            self.ftrace_time_data[local_timestamp].append(temp_data)
        elif event_name == "workqueue_execute_start" or event_name == "workqueue_execute_end" or event_name == "workqueue_queue_work":
            trace_event_raw_work_offset = None
            function_offset = 0
            if event_name == "workqueue_execute_start":
                function_offset = self.ramdump.field_offset(
                    'struct ' + 'trace_event_raw_workqueue_execute_start', "function")
                trace_event_raw_work_offset = self.ramdump.field_offset(
                    'struct ' + 'trace_event_raw_workqueue_execute_start', "work")
            elif event_name == "workqueue_execute_end":
                function_offset = self.ramdump.field_offset(
                    'struct ' + 'trace_event_raw_workqueue_queue_work', "function")
                trace_event_raw_work_offset = self.ramdump.field_offset(
                    'struct ' + 'trace_event_raw_workqueue_queue_work', "work")
            elif event_name == "workqueue_queue_work":
                function_offset = self.ramdump.field_offset(
                    'struct ' + 'trace_event_raw_workqueue_execute_end', "function")
                trace_event_raw_work_offset = self.ramdump.field_offset(
                    'struct ' + 'trace_event_raw_workqueue_execute_end', "work")
            function = 0
            if function_offset is not None:
                function = self.ramdump.read_pointer(ftrace_raw_entry + function_offset)
            function_name = 'none'
            if function:
                function_name = self.ramdump.unwind_lookup(function)
                if function_name == None:
                    function_name = 'none'
            if trace_event_raw_work_offset is not None:
                work = self.ramdump.read_pointer(ftrace_raw_entry + trace_event_raw_work_offset)
                temp_data = "{0} work_struct {1} function 0x{2:x} {3}\n".format(head_data_fmt,
                            str(hex(work)).replace("L",""),function,function_name)
            else:
                temp_data = "{0} work_struct unknown\n".format(head_data_fmt)
            self.ftrace_time_data[local_timestamp].append(temp_data)
        elif event_name == "bprint":
            MAX_LEN = 1000
            print_entry_ip_offset = self.ramdump.field_offset('struct bprint_entry' , "ip")
            print_entry_buf_offset = self.ramdump.field_offset('struct bprint_entry', "buf")
            print_entry_fmt_offset = self.ramdump.field_offset('struct bprint_entry', "fmt")
            print_ip = self.ramdump.read_word(ftrace_raw_entry + print_entry_ip_offset)
            print_entry_fmt = self.ramdump.read_pointer(ftrace_raw_entry + print_entry_fmt_offset)
            print_entry_fmt_data = self.ramdump.read_cstring(print_entry_fmt, MAX_LEN)
            """
            ['%px', '%llx', '%ps', '%p']
            Supported :
                d for integers
                f for floating-point numbers
                b for binary numbers
                o for octal numbers
                x for octal hexadecimal numbers
                s for string
                e for floating-point in an exponent format
            """
            regex = re.compile('%[\*]*[a-zA-Z]+')
            length = 0
            print_buffer = []
            print_buffer_offset = ftrace_raw_entry + print_entry_buf_offset
            if print_entry_fmt_data:
                function = self.ramdump.get_symbol_info1(print_ip)
                prev_match = None
                unaligned_print_buffer_offset = None
                for match in regex.finditer(print_entry_fmt_data):
                    replacement = match.group()
                    if 'c' in match.group():
                        replacement = '%s'
                        print_buffer.append(self.ramdump.read_byte(print_buffer_offset))
                        print_buffer_offset += self.ramdump.sizeof('char')
                    elif "%*pbl" in match.group():
                        replacement = "%s"
                        print_entry_fmt_data = print_entry_fmt_data.replace(match.group(), replacement)
                        align = self.ramdump.sizeof("int") - 1
                        #Read precision/width
                        print_buffer_offset += self.ramdump.sizeof('unsigned int')
                        print_buffer_offset = (print_buffer_offset + (align)) & (~align)
                        #Read bitmask
                        nr_cpu_ids = self.ramdump.address_of("nr_cpu_ids")
                        nr_cpu_ids = self.ramdump.read_u32(nr_cpu_ids)
                        #single element of long is enough to accomodate all cpus
                        cpu_bits = self.ramdump.read_u64(print_buffer_offset)
                        # Trim bits to valid mask only
                        def getValidBits(num,k,p):
                             binary = bin(num)
                             binary = binary[2:]
                             end = len(binary) - p
                             start = end - k
                             return binary[start : end+1]
                        cpu_bits = getValidBits(cpu_bits, nr_cpu_ids, 0)
                        print_buffer.append(cpu_bits)
                        print_buffer_offset += self.ramdump.sizeof('unsigned long')
                        print_buffer_offset = (print_buffer_offset + (align)) & (~align)
                        continue
                    elif '%ps' in match.group():
                        replacement = "%s%x"
                        if self.ramdump.arm64:
                            addr = self.ramdump.read_u64(print_buffer_offset)
                            wname = self.ramdump.unwind_lookup(addr)
                            if wname is None:
                                wname = 'na'
                            print_buffer.append(wname)
                            print_buffer.append(addr)
                            print_buffer_offset += 8
                        else:
                            addr = self.ramdump.read_u32(print_buffer_offset)
                            wname = self.ramdump.unwind_lookup(addr)
                            if wname is None:
                                wname = 'na'
                            print_buffer.append(wname)
                            print_buffer.append(addr)
                            print_buffer_offset += 4
                    elif '%pS' in match.group():
                        replacement = "%s(%x)"
                        if self.ramdump.arm64:
                            addr = self.ramdump.read_u64(print_buffer_offset)
                            wname = self.ramdump.unwind_lookup(addr)
                            if wname is None:
                                wname = 'na'
                            else:
                                wname = '{}+{}'.format(wname[0], hex(wname[1]))
                            print_buffer.append(wname)
                            print_buffer.append(addr)
                            print_buffer_offset += 8
                        else:
                            addr = self.ramdump.read_u32(print_buffer_offset)
                            wname = self.ramdump.unwind_lookup(addr)
                            if wname is None:
                                wname = 'na'
                            else:
                                wname = '{}+{}'.format(wname[0], hex(wname[1]))
                            print_buffer.append(wname)
                            print_buffer.append(addr)
                            print_buffer_offset += 4
                    elif '%p' in match.group() and '%ps' not in match.group() and '%pS' not in match.group():
                        replacement = "%x"
                        if self.ramdump.arm64:
                            print_buffer.append(self.ramdump.read_u64(print_buffer_offset))
                            print_buffer_offset += 8
                        else:
                            print_buffer.append(self.ramdump.read_u32(print_buffer_offset))
                            print_buffer_offset += 4
                    elif 'x' in match.group():
                        replacement = "%x"
                        if self.ramdump.arm64:
                            print_buffer.append(self.ramdump.read_u64(print_buffer_offset))
                            print_buffer_offset += 8
                        else:
                            print_buffer.append(self.ramdump.read_u32(print_buffer_offset))
                            print_buffer_offset += 4
                    elif 's' in match.group():
                        replacement = "%s"
                        if prev_match is not None and '%s' in prev_match:
                            print_buffer_offset = unaligned_print_buffer_offset
                        sdata = self.ramdump.read_cstring(print_buffer_offset)
                        print_buffer.append(sdata)
                        print_buffer_offset = print_buffer_offset + len(sdata) + 1
                    elif 'll' in match.group() or 'l' in match.group():
                        replacement = "%d"
                        if self.ramdump.arm64:
                            print_buffer.append(self.ramdump.read_u64(print_buffer_offset))
                            print_buffer_offset += 8
                        else:
                            print_buffer.append(self.ramdump.read_u32(print_buffer_offset))
                            print_buffer_offset += 4
                    elif 'h' in match.group():
                        print_buffer.append(self.ramdump.read_u16(print_buffer_offset))
                        print_buffer_offset += self.ramdump.sizeof('short')
                    elif 'd' in match.group():
                        replacement = "%d"
                        print_buffer.append(self.ramdump.read_int(print_buffer_offset))
                        print_buffer_offset += self.ramdump.sizeof('int')
                    elif 'u' in match.group():
                        replacement = "%d"
                        if 'll' in match.group() or 'l' in match.group():
                            if self.ramdump.arm64:
                                print_buffer.append(self.ramdump.read_u64(print_buffer_offset))
                                print_buffer_offset += 8
                            else:
                                print_buffer.append(self.ramdump.read_u32(print_buffer_offset))
                                print_buffer_offset += 4
                        else:
                            print_buffer.append(self.ramdump.read_u32(print_buffer_offset))
                            print_buffer_offset += self.ramdump.sizeof('unsigned int')
                    elif 'f' in match.group():
                        replacement = "%f"
                        print_buffer.append(self.ramdump.read_u32(print_buffer_offset))
                        print_buffer_offset += self.ramdump.sizeof('float')
                    if replacement != match.group():
                        print_entry_fmt_data = print_entry_fmt_data.replace(match.group(), replacement, 1)
                    length += 1
                    prev_match = match.group()
                    unaligned_print_buffer_offset = print_buffer_offset
                    align = self.ramdump.sizeof("int") - 1
                    print_buffer_offset = (print_buffer_offset + (align)) & (~align)
                try:
                    temp_data = "{0} {1}: {2}\n".format(head_data_fmt,
                                    function,print_entry_fmt_data % (tuple(print_buffer)))
                except Exception as err:
                    temp_data = "{0} Error parsing bprint event entry".format(head_data_fmt)
                self.ftrace_time_data[local_timestamp].append(temp_data)
        elif event_name == "print":
                print_entry_ip_offset = self.ramdump.field_offset('struct print_entry' , "ip")
                print_entry_buf_offset = self.ramdump.field_offset('struct print_entry', "buf")
                print_ip = self.ramdump.read_word(ftrace_raw_entry + print_entry_ip_offset)
                print_buffer = self.ramdump.read_cstring(ftrace_raw_entry + print_entry_buf_offset)
                function_lookup = self.ramdump.unwind_lookup(print_ip)
                if function_lookup is not None:
                    function, offset = function_lookup
                else:
                    function = "unknown"
                temp_data = "{0} {1}: {2}".format(head_data_fmt,function,print_buffer)
                if not print_buffer.endswith("\n"):
                    temp_data += "\n"
                self.ftrace_time_data[local_timestamp].append(temp_data)
        elif event_name == "bputs":
                entry_ip_offset = self.ramdump.field_offset('struct bputs_entry' , "ip")
                entry_str_offset = self.ramdump.field_offset('struct bputs_entry', "str")
                ip = self.ramdump.read_word(ftrace_raw_entry + entry_ip_offset)
                str_addr = self.ramdump.read_pointer(ftrace_raw_entry + entry_str_offset)
                str_buffer = self.ramdump.read_cstring(str_addr)
                function_lookup = self.ramdump.unwind_lookup(ip)
                if function_lookup is not None:
                    function, offset = function_lookup
                else:
                    function = "unknown"
                temp_data = "{0} {1}: {2}".format(head_data_fmt,function,str_buffer)
                if not str_buffer.endswith("\n"):
                    temp_data += "\n"
                self.ftrace_time_data[local_timestamp].append(temp_data)
        elif event_name == "kernel_stack":
                '''
                  FTRACE_ENTRY(kernel_stack, stack_entry,

                    TRACE_STACK,

                    F_STRUCT(
                        __field(	int,		size	)
                        __stack_array(	unsigned long,	caller,	FTRACE_STACK_ENTRIES, size)
                    ),

                    F_printk("\t=> %ps\n\t=> %ps\n\t=> %ps\n"
                        "\t=> %ps\n\t=> %ps\n\t=> %ps\n"
                        "\t=> %ps\n\t=> %ps\n",
                        (void *)__entry->caller[0], (void *)__entry->caller[1],
                        (void *)__entry->caller[2], (void *)__entry->caller[3],
                        (void *)__entry->caller[4], (void *)__entry->caller[5],
                        (void *)__entry->caller[6], (void *)__entry->caller[7])
                );
                '''
                size_offset = self.ramdump.field_offset('struct stack_entry' , "size")
                caller_offset = self.ramdump.field_offset('struct stack_entry', "caller")
                caller = self.ramdump.read_word(ftrace_raw_entry + caller_offset)
                size = self.ramdump.read_int(ftrace_raw_entry + size_offset)
                call_string_hex =''
                if size != 0 and caller != 0:
                    #print("v.v (struct stack_entry)0x%x"%(ftrace_raw_entry))
                    for i in range(0, 8):
                        caller = self.ramdump.read_word(ftrace_raw_entry + caller_offset + i * 8)
                        function = "unknown"
                        function_lookup = self.ramdump.unwind_lookup(caller)
                        if function_lookup is not None:
                            function, offset = function_lookup
                        print_buffer = " 0x{0:x} {1} ".format(caller, function)
                        call_string_hex +=print_buffer
                    call_string_hex +="\n"
                    temp_data = "{0} {1}: {2}".format(head_data_fmt, 'kernel_stack', call_string_hex)
                    if not call_string_hex.endswith("\n"):
                        temp_data += "\n"
                    self.ftrace_time_data[local_timestamp].append(temp_data)
                    #print(temp_data)
        elif event_name == "user_stack":
                '''
                    crash> struct userstack_entry -o -x
                    struct userstack_entry {
                    [0x0] struct trace_entry ent;
                    [0x8] unsigned int tgid;
                    [0x10] unsigned long caller[8];
                    }
                    SIZE: 0x50
                '''
                ent_offset = self.ramdump.field_offset('struct userstack_entry' , "ent")
                tgid_offset = self.ramdump.field_offset('struct userstack_entry', "tgid")
                caller_offset = self.ramdump.field_offset('struct userstack_entry', "caller")
                ent = self.ramdump.read_word(ftrace_raw_entry + ent_offset)
                tgid = self.ramdump.read_word(ftrace_raw_entry + tgid_offset)
                caller = self.ramdump.read_word(ftrace_raw_entry + caller_offset)
                call_string_hex =''
                if caller != 0:
                    #print("v.v (struct userstack_entry)0x%x"%(ftrace_raw_entry))
                    for i in range(0, 8):
                        caller = self.ramdump.read_word(ftrace_raw_entry + caller_offset + i * 8)
                        print_buffer = " 0x{0:x} ".format(caller)
                        call_string_hex +=print_buffer
                    call_string_hex +="\n"
                    temp_data = "{0} {1}: {2}".format(head_data_fmt, tgid, call_string_hex)
                    if not call_string_hex.endswith("\n"):
                        temp_data += "\n"
                    self.ftrace_time_data[local_timestamp].append(temp_data)
                    #print(temp_data)
        else:
            event_data = self.format_event_map[event_name]
            fmt_str = event_data[1]
            if "rpmh" in event_name:
                fmt_str =  fmt_str.replace('send-msg:','send-msg')
                fmt_str =  fmt_str.replace(': ','')
            elif "workqueue" in event_name:
                fmt_str =  fmt_str.replace('work struct','work_struct')
            offset_data = event_data[0]
            fmt_name_value_map = OrderedDict()
            try:
                d = str(fmt_str.split('",')[1].replace("'", ''))
                pr = str(fmt_str.split('",')[0].replace("'", ''))
                pr = str(pr.split('",')[0].replace('"', ''))
                pr = str(pr.split('",')[0].replace('[', ''))
                pr = str(pr.split('",')[0].replace(']', ''))
                if "cpuhp_latency" == event_name:
                    pr = pr.replace("USEC ret: %d","USEC_ret:%d")
                if "thermal_device_update" == event_name:
                    pr = pr.replace("received event","received_event")
                temp_a = []
                for ii in d.split(","):
                    ii = str(ii).replace("'","").replace(" ","")
                    temp_a.append(ii)
                j = 0
                temp_a = []
                pr_f = []
                if "workqueue_execute" in event_name:
                    for ki in pr.split(": "):
                        pr_f.append(str(ki))
                else:
                    if ", " in pr and event_name != 'user_fault':
                        for ki in pr.split(", "):
                            if len(ki) >= 1:
                                pr_f.append(str(ki).replace(" ",""))
                    else:
                        for ki in pr.split(" "):
                            if len(ki) >= 1:
                                pr_f.append(str(ki).replace(" ",""))
                for item,item_list in offset_data.items():
                    type_str,offset,size = item_list
                    if 'unsigned long' in type_str or 'u64' in type_str or '*' in type_str:
                        v = self.ramdump.read_pointer(ftrace_raw_entry + offset)
                        if "rwmmio" in event_name and "addr" in item:
                            phys = self.ramdump.virt_to_phys(v)
                            fmt_name_value_map[item] = "{}({})".format(hex(int(v)), hex(phys))
                        elif "func" not in item:
                            fmt_name_value_map[item] = hex(int(v))
                        else:
                            fmt_name_value_map[item] = v
                    elif 'long' in type_str or 'int' in type_str or 'u32' in type_str or 'bool' in type_str or 'pid_t' in type_str:
                        v = self.ramdump.read_u32(ftrace_raw_entry + offset)
                        fmt_name_value_map[item] = v
                    elif 'u8' in type_str:
                        v = self.ramdump.read_byte(ftrace_raw_entry + offset)
                        fmt_name_value_map[item] = v
                    elif 'const' in type_str and 'char *' in type_str:
                        v = self.ramdump.read_pointer(ftrace_raw_entry + offset)
                        v = self.ramdump.read_cstring(v)
                        fmt_name_value_map[item] = v
                    elif type_str.startswith('__data_loc') and type_str.endswith('char[]'):
                        v = self.ramdump.read_u32(ftrace_raw_entry + offset)
                        v = self.ramdump.read_cstring(ftrace_raw_entry + (v & 0xffff), (v >> 16))
                        if isinstance(v, bytes):
                            v = self.ramdump.read_cstring(ftrace_raw_entry + (offset*4))
                        fmt_name_value_map[item] = v
                    elif 'char[' in type_str:
                        length = re.match(r'(?:unsigned )?char\[(\d+)\]', type_str)
                        if length:
                            length = int(length.group(1))
                        else:
                            if "[TASK_COMM_LEN]" in type_str:
                                length = 16
                            else:
                                print_out_str("ftrace: unknown length for {} ({})".format(item, type_str))
                                length = 12  # Chosen arbitrarily
                        v = self.ramdump.read_cstring(ftrace_raw_entry + offset, max_length=length)
                        fmt_name_value_map[item] = v
                    elif 'char' in type_str:
                        v = self.ramdump.read_byte(ftrace_raw_entry + offset)
                        fmt_name_value_map[item] = v
                    elif 'unsigned short' in type_str or 'u16' in type_str:
                        v = self.ramdump.read_u16(ftrace_raw_entry + offset)
                        fmt_name_value_map[item] = v
                    elif 'short' in type_str or 'signed short' in type_str or 's16' in type_str:
                        v = self.ramdump.read_s32(ftrace_raw_entry + offset)
                        fmt_name_value_map[item] = v
                    elif 's64' in type_str:
                        v = self.ramdump.read_s64(ftrace_raw_entry + offset)
                        fmt_name_value_map[item] = v
                    else:
                        v = self.ramdump.read_u32(ftrace_raw_entry + offset)
                        fmt_name_value_map[item] = v

                    if "softirq" in event_name:
                        if v > len(softirq_action_list) -1:
                            action = v
                        else:
                            action = softirq_action_list[v]
                        fmt_name_value_map['action'] = action
                    if "rwmmio" in event_name and "caller" in item:
                        symbol = self.ramdump.read_word(ftrace_raw_entry + offset)
                        if symbol is not None:
                            fmt_name_value_map[item] = self.ramdump.get_symbol_info1(symbol)
                    temp_a.append(v)
                    j = j + 1
                temp = ""
                try:
                    for keyinfo in fmt_name_value_map:
                        if "function" == keyinfo and isinstance(fmt_name_value_map[keyinfo], int):
                            wq_function1 = self.ramdump.get_symbol_info1(fmt_name_value_map[keyinfo])
                            tt = keyinfo + "=" + wq_function1
                        if "func" in keyinfo and isinstance(fmt_name_value_map[keyinfo], int):
                            wq_function1 = self.ramdump.get_symbol_info1(fmt_name_value_map[keyinfo])
                            if wq_function1 and len(wq_function1) > 1 and wq_function1 != 'No':
                                tt = keyinfo + "=" + wq_function1
                            else:
                                tt = keyinfo + "=" + str(hex(fmt_name_value_map[keyinfo]))
                        else:
                            tt = keyinfo + "=" + str(fmt_name_value_map[keyinfo])
                        temp = temp + tt + "  "
                except Exception as err:
                    #print_out_str("missing event = {0} err = {1}".format(event_name,str(err)))
                    pass
                try:
                    if not temp.endswith("\n"):
                        temp += "\n"
                    temp_data = "{0} {1}".format(head_data_fmt,temp)
                    self.ftrace_time_data[local_timestamp].append(temp_data)
                    temp = ""
                except Exception as err:
                    #print_out_str("missing event = {0} err = {1}".format(event_name,str(err)))
                    pass
            except Exception as err:
                #print_out_str("missing event = {0} err = {1}".format(event_name,str(err)))
                pass

    def ring_buffer_per_cpu_parsing(self, ring_trace_buffer_cpu, max_page=None):
        page_index = 0
        buffer_page_list_offset = self.ramdump.field_offset(
            'struct buffer_page ', 'list')
        buffer_page_list_prev_offset = self.ramdump.field_offset(
            'struct list_head ', 'prev')
        trace_ring_buffer_per_cpu_data = struct_print_class(self.ramdump, 'ring_buffer_per_cpu', ring_trace_buffer_cpu, None)
        '''
            crash> struct ring_buffer_per_cpu -x -o
            struct ring_buffer_per_cpu {
                [0x0] int cpu;
                [0x4] atomic_t record_disabled;
                [0x8] atomic_t resize_disabled;
            [0x10] struct trace_buffer *buffer;
            [0x80] unsigned long nr_pages;
            [0x88] unsigned int current_context;
            [0x90] struct list_head *pages;
            [0x98] struct buffer_page *head_page;
            [0xa0] struct buffer_page *tail_page;   // parser this
            [0xa8] struct buffer_page *commit_page;
            [0xb0] struct buffer_page *reader_page;
        '''
        if self.ramdump.arm64:
            trace_ring_buffer_per_cpu_data.append('nr_pages', 'u64')
        else:
            trace_ring_buffer_per_cpu_data.append('nr_pages', 'u32')
        trace_ring_buffer_per_cpu_data.append('tail_page', 'ptr')
        trace_ring_buffer_per_cpu_data.process()
        nr_pages = trace_ring_buffer_per_cpu_data.get_val('nr_pages')
        buffer_page_entry = trace_ring_buffer_per_cpu_data.get_val('tail_page')

        while page_index < nr_pages:
            if buffer_page_entry:
                self.parse_buffer_page_entry(buffer_page_entry)
                buffer_page_entry_list = buffer_page_entry + buffer_page_list_offset
                buffer_page_entry = self.ramdump.read_pointer(buffer_page_entry_list + buffer_page_list_prev_offset)
            if max_page and page_index >= max_page:
                print_out_str("Reached to the max page = {0} nr_pages = {1}".format(max_page, nr_pages))
                break
            page_index = page_index + 1
