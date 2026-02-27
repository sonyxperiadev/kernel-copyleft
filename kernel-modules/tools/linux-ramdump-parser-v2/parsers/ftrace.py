# Copyright (c) 2017-2022, The Linux Foundation. All rights reserved.
# Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import parser_util
import local_settings
import os
import subprocess
import re
from collections import OrderedDict

from parser_util import register_parser, RamParser
from print_out import print_out_str
from tempfile import NamedTemporaryFile
from .ftrace_event_list import FtraceParser_Event_List
from .ftrace_event import FtraceParser_Event, BufferedWrite
import linux_list as llist
from . import taskdump
#import time

@register_parser(
    '--dump-ftrace',
    'Use \'crash\' to extract ftrace and trace-cmd to parse it.',
    optional=True)
class FtraceParser(RamParser):

    def __init__(self, *args):
        super(FtraceParser, self).__init__(*args)
        self.format_event_map = OrderedDict()
        self.format_event_field_map = OrderedDict()
        self.event_call = 'struct trace_event_call'
        self.event_class = 'struct trace_event_class'
        self.trace_names = ["binder", "bootreceiver", "clock_reg", "kgsl-fence",
                            "memory", "mmc", "rproc_qcom", "suspend", "ufs",
                            "usb", "wifi", "rwmmio"]
        self.whitelisted_trace_names =[]
        self.ftrace_buffer_size_kb = None
        self.per_cpu_buffer_pages = None

        if len(self.ramdump.ftrace_args):
            self.whitelisted_trace_names = self.ramdump.ftrace_args

        if self.ramdump.ftrace_max_size:
            self.per_cpu_buffer_pages = self.ramdump.ftrace_max_size / 4

    def ftrace_field_func(self, common_list, ram_dump):
        name_offset = ram_dump.field_offset('struct ftrace_event_field', 'name')
        type_offset = ram_dump.field_offset('struct ftrace_event_field', 'type')
        filter_type_offset = ram_dump.field_offset('struct ftrace_event_field', 'filter_type')
        field_offset = ram_dump.field_offset('struct ftrace_event_field', 'offset')
        size_offset = ram_dump.field_offset('struct ftrace_event_field', 'size')
        signed_offset = ram_dump.field_offset('struct ftrace_event_field', 'is_signed')

        name = ram_dump.read_word(common_list + name_offset)
        field_name = ram_dump.read_cstring(name, 256)
        type_name = ram_dump.read_word(common_list + type_offset)
        type_str = ram_dump.read_cstring(type_name, 256)
        offset = ram_dump.read_u32(common_list + field_offset)
        size = ram_dump.read_u32(common_list + size_offset)
        signed = ram_dump.read_u32(common_list + signed_offset)

        if re.match('(.*)\[(.*)', type_str) and not (re.match('__data_loc', type_str)):
            s = re.split('\[', type_str)
            s[1] = '[' + s[1]
            self.formats_out.write(
                "\tfield:{0} {1}{2};\toffset:{3};\tsize:{4};\tsigned:{5};\n".format(s[0], field_name, s[1], offset,
                                                                                    size, signed))
            if "common_type" == field_name or "common_flags" == field_name or "common_preempt_count" == field_name or "common_pid" == field_name:
                temp = 0
            else:
                format_list = []
                format_list.append(type_str)
                format_list.append(offset)
                format_list.append(size)
                self.format_event_field_map[field_name] = format_list
        else:
            self.formats_out.write(
                "\tfield:{0} {1};\toffset:{2};\tsize:{3};\tsigned:{4};\n".format(type_str, field_name, offset, size,

                                                                                 signed))
            #self.format_event_field_map = {}

            if "common_type" == field_name or "common_flags" == field_name or "common_preempt_count" == field_name or "common_pid" == field_name:
                temp = 0
            else:
                format_list = []
                format_list.append(type_str)
                format_list.append(offset)
                format_list.append(size)
                self.format_event_field_map[field_name] = format_list

    def ftrace_events_func(self, ftrace_list, ram_dump):
        event_offset = ram_dump.field_offset(self.event_call, 'event')
        fmt_offset = ram_dump.field_offset(self.event_call, 'print_fmt')
        class_offset = ram_dump.field_offset(self.event_call, 'class')
        flags_offset = ram_dump.field_offset(self.event_call, 'flags')
        flags = ram_dump.read_word(ftrace_list + flags_offset)
        if ram_dump.kernel_version >= (4, 14):
            TRACE_EVENT_FL_TRACEPOINT = 0x10
        elif ram_dump.kernel_version >= (4, 9):
            TRACE_EVENT_FL_TRACEPOINT = 0x20
        else:
            TRACE_EVENT_FL_TRACEPOINT = 0x40
        if (ram_dump.kernel_version >= (3, 18) and (flags & TRACE_EVENT_FL_TRACEPOINT)):
            tp_offset = ram_dump.field_offset(self.event_call, 'tp')
            tp_name_offset = ram_dump.field_offset('struct tracepoint', 'name')
            tp = ram_dump.read_word(ftrace_list + tp_offset)
            name = ram_dump.read_word(tp + tp_name_offset)
        else:
            name_offset = ram_dump.field_offset(self.event_call, 'name')
            name = ram_dump.read_word(ftrace_list + name_offset)

        type_offset = ram_dump.field_offset('struct trace_event', 'type')
        fields_offset = ram_dump.field_offset(self.event_class, 'fields')
        common_field_list = ram_dump.address_of('ftrace_common_fields')
        field_next_offset = ram_dump.field_offset('struct ftrace_event_field', 'link')

        name_str = ram_dump.read_cstring(name, 512)
        event_id = ram_dump.read_word(ftrace_list + event_offset + type_offset)
        fmt = ram_dump.read_word(ftrace_list + fmt_offset)
        fmt_str = ram_dump.read_cstring(fmt, 2048)

        self.formats_out.write("name: {0}\n".format(name_str))
        self.formats_out.write("ID: {0}\n".format(event_id))
        self.formats_out.write("format:\n")

        #self.format_event_map[name_str] = format_event_field_map

        list_walker = llist.ListWalker(ram_dump, common_field_list, field_next_offset)
        list_walker.walk_prev(common_field_list, self.ftrace_field_func, ram_dump)
        self.formats_out.write("\n")

        event_class = ram_dump.read_word(ftrace_list + class_offset)
        field_list = event_class + fields_offset
        list_walker = llist.ListWalker(ram_dump, field_list, field_next_offset)
        list_walker.walk_prev(field_list, self.ftrace_field_func, ram_dump)
        self.formats_out.write("\n")
        self.formats_out.write("print fmt: {0}\n".format(fmt_str))
        fmt_list = []
        fmt_list.append(self.format_event_field_map)
        fmt_list.append(fmt_str)
        self.format_event_map[name_str] = fmt_list
        self.format_event_field_map = OrderedDict()

    def ftrace_extract(self):
        #ftrace_event_time = 0
        #post_ftrace_event_time = 0
        #taskdump_time = 0
        #parse_trace_entry_time = 0
        global_trace_data_org = self.ramdump.address_of('ftrace_trace_arrays')
        global_trace_data_offset = self.ramdump.field_offset(
            'struct list_head ', 'next')
        if self.ramdump.arm64:
            global_trace_data_next = self.ramdump.read_u64(global_trace_data_org + global_trace_data_offset)
        else:
            global_trace_data_next = self.ramdump.read_u32(global_trace_data_org + global_trace_data_offset)

        if self.ramdump.kernel_version >= (5, 10):
            trace_buffer_ptr = self.ramdump.field_offset(
                'struct trace_array', 'array_buffer')
        else:
            trace_buffer_ptr = self.ramdump.field_offset(
                'struct trace_array', 'trace_buffer')
        trace_buffer_name_offset = self.ramdump.field_offset(
            'struct trace_array', 'name')
        if self.ramdump.kernel_version >= (5, 10):
            ring_trace_buffer_ptr = self.ramdump.field_offset(
                'struct array_buffer', 'buffer')
        else:
            ring_trace_buffer_ptr = self.ramdump.field_offset(
                'struct trace_buffer', 'buffer')
        if self.ramdump.kernel_version >= (5, 10):
            ring_trace_buffer_cpus_ptr = self.ramdump.field_offset(
                'struct trace_buffer', 'cpus')
            ring_trace_buffer_base_addr = self.ramdump.field_offset(
                'struct trace_buffer', 'buffers')
        else:
            ring_trace_buffer_cpus_ptr = self.ramdump.frame_field_offset(
                'rb_wake_up_waiters','struct ring_buffer', 'cpus')
            if ring_trace_buffer_cpus_ptr is None:
                ring_trace_buffer_cpus_ptr = 0x4
            ring_trace_buffer_base_addr = self.ramdump.frame_field_offset(
                'rb_wake_up_waiters','struct ring_buffer', 'buffers')
            if ring_trace_buffer_base_addr is None:
                ring_trace_buffer_base_addr = self.ramdump.field_offset(
                        'struct ring_buffer', 'buffers')
            if ring_trace_buffer_base_addr is None:
                if self.ramdump.arm64:
                    ring_trace_buffer_base_addr = 0x58
                else:
                    ring_trace_buffer_base_addr = 0x38
        ring_trace_buffer_nr_pages = self.ramdump.field_offset(
            'struct ring_buffer_per_cpu', 'nr_pages')

        formats_out = self.ramdump.open_file('formats.txt')
        self.formats_out = formats_out
        fevent_list = FtraceParser_Event_List(self.ramdump)
        #print(fevent_list.ftrace_raw_struct_type)

        ftrace_events_list = self.ramdump.address_of('ftrace_events')
        next_offset = self.ramdump.field_offset(self.event_call, 'list')
        list_walker = llist.ListWalker(self.ramdump, ftrace_events_list, next_offset)
        list_walker.walk_prev(ftrace_events_list, self.ftrace_events_func, self.ramdump)
        self.formats_out.close
        #taskdump_time = time.time()
        taskdump.do_dump_stacks(self.ramdump, 0)
        #print("Taskdump took {} secs".format(time.time()-taskdump_time))

        while(global_trace_data_org != global_trace_data_next):
            global_trace_data = global_trace_data_next

            trace_buffer_name = self.ramdump.read_word(global_trace_data + trace_buffer_name_offset)
            if not (trace_buffer_name):
                trace_name = None
            else:
                trace_name = self.ramdump.read_cstring(trace_buffer_name, 256)

            if self.ramdump.arm64:
                trace_buffer_ptr_data = self.ramdump.read_u64(
                    global_trace_data + trace_buffer_ptr)
            else:
                trace_buffer_ptr_data = self.ramdump.read_u32(
                    global_trace_data + trace_buffer_ptr)


            ring_trace_buffer_data = trace_buffer_ptr_data + trace_buffer_ptr
            ring_trace_buffer_cpus = self.ramdump.read_u32(
                ring_trace_buffer_data + ring_trace_buffer_cpus_ptr)
            if self.ramdump.arm64:
                ring_trace_buffer_base_data = self.ramdump.read_u64(
                    ring_trace_buffer_data + ring_trace_buffer_ptr)
            else:
                ring_trace_buffer_base_data = self.ramdump.read_u32(
                    ring_trace_buffer_data + ring_trace_buffer_ptr)

            if self.ramdump.arm64:
                ring_trace_buffer_base_data1 = self.ramdump.read_u64(
                    ring_trace_buffer_base_data + ring_trace_buffer_base_addr)
            else:
                ring_trace_buffer_base_data1 = self.ramdump.read_u32(
                    ring_trace_buffer_base_data + ring_trace_buffer_base_addr)

            if trace_name is None or trace_name == 0x0 or trace_name == "0x0" or trace_name == "None" or trace_name == "null" or len(trace_name) < 1:
                #ftrace_out = self.ramdump.open_file('ftrace.txt','w')
                fout = self.ramdump.open_file('ftrace.txt','w')
                ftrace_out = BufferedWrite(fout)
                header_data = "# tracer: nop \n" \
                              "#\n" \
                              "# entries-in-buffer/entries-written: 315882/1727030   #P:8\n" \
                              "#\n" \
                              "#                              _-----=> irqs-off\n" \
                              "#                             / _----=> need-resched\n" \
                              "#                            | / _---=> hardirq/softirq\n" \
                              "#                            || / _--=> preempt-depth\n" \
                              "#                            ||| /     delay\n" \
                              "#           TASK-PID   CPU#  ||||    TIMESTAMP  FUNCTION\n" \
                              "#              | |       |   ||||       |         |\n"
                ftrace_out.write(header_data)
            else:
                if trace_name in self.whitelisted_trace_names or self.whitelisted_trace_names == ["all"]:
                    #ftrace_out = self.ramdump.open_file('ftrace_parser/' + 'ftrace_' + trace_name + '.txt','w')
                    fout = self.ramdump.open_file('ftrace_parser/' + 'ftrace_' + trace_name + '.txt','w')
                    ftrace_out = BufferedWrite(fout)
                else:
                    if self.ramdump.arm64:
                        global_trace_data_next =  self.ramdump.read_u64(global_trace_data_next)
                    else:
                        global_trace_data_next =  self.ramdump.read_u32(global_trace_data_next)
                    continue
            #    ftrace_out = self.ramdump.open_file('ftrace_parser/' + 'ftrace_' + trace_name + '.txt','w')

            ftrace_time_data = {}
            nr_total_buffer_pages = 0
            rb_per_cpu = []
            nr_pages_per_buffer = []
            #taskdump.do_dump_stacks(self.ramdump, 0)
            for cpu_idx in range(0,8):
                #array_ptr = self.ramdump.read_u64(ring_trace_buffer_base_data1 + self.ramdump.sizeof('void *') * cpu_idx)
                array_ptr = (ring_trace_buffer_base_data1 + self.ramdump.sizeof('void *') * cpu_idx)
                #print "array_ptr = {0}".format(hex(array_ptr))
                if self.ramdump.arm64:
                    b = self.ramdump.read_u64(array_ptr)
                else:
                    b = self.ramdump.read_u32(array_ptr)
                if b is None or b == 0x0:
                    continue
                if self.ramdump.arm64:
                    nr_pages =  self.ramdump.read_u64(
                        b + ring_trace_buffer_nr_pages)
                else:
                    nr_pages =  self.ramdump.read_u32(
                        b + ring_trace_buffer_nr_pages)
                if nr_pages is None:
                    continue
                if self.per_cpu_buffer_pages and self.per_cpu_buffer_pages < nr_pages:
                    nr_pages = self.per_cpu_buffer_pages

                nr_total_buffer_pages = nr_total_buffer_pages +  nr_pages

                nr_pages_per_buffer.append(nr_pages)
                rb_per_cpu.append(b)
                #print "ring_trace_buffer_cpus nr_pages = %d" % nr_pages
                #print "cpu_buffer = {0}".format(hex(b))

            print("\nTotal pages across cpu trace buffers = {}".format(round(nr_total_buffer_pages)))

            #start = time.time()
            for cpu_idx in range(0,len(rb_per_cpu)):
                nr_pages_per_buffer_item = nr_pages_per_buffer[cpu_idx]
                per_cpu_buffer = rb_per_cpu[cpu_idx]
                if per_cpu_buffer is not None:
                    evt = FtraceParser_Event(self.ramdump,ftrace_out,cpu_idx,per_cpu_buffer,nr_pages_per_buffer_item,nr_total_buffer_pages
                                       ,fevent_list.ftrace_event_type,fevent_list.ftrace_raw_struct_type,ftrace_time_data,self.format_event_map)
                    evt.ftrace_event_parsing()
                    #parse_trace_entry_time += evt.parse_trace_entry_time
            #ftrace_event_time += (time.time()-start)

            if self.ramdump.arm64:
                global_trace_data_next =  self.ramdump.read_u64(global_trace_data_next)
            else:
                global_trace_data_next =  self.ramdump.read_u32(global_trace_data_next)
            swapper_entry = False
            prev_comm = None
            curr_comm = None
            pending_update_list = []
            pendig_process = False
            switch_map = {}
            ftrace_file_map = {}
            if trace_name is None or trace_name == 0x0 or trace_name == "0x0" or trace_name == "None" or trace_name == "null" or len(trace_name) < 1:
                ftrace_core0_fd = self.ramdump.open_file('ftrace_core0.txt', 'w')
                ftrace_core1_fd = self.ramdump.open_file('ftrace_core1.txt', 'w')
                ftrace_core2_fd = self.ramdump.open_file('ftrace_core2.txt', 'w')
                ftrace_core3_fd = self.ramdump.open_file('ftrace_core3.txt', 'w')
                ftrace_core4_fd = self.ramdump.open_file('ftrace_core4.txt', 'w')
                ftrace_core5_fd = self.ramdump.open_file('ftrace_core5.txt', 'w')
                ftrace_core6_fd = self.ramdump.open_file('ftrace_core6.txt', 'w')
                ftrace_core7_fd = self.ramdump.open_file('ftrace_core7.txt', 'w')
            else:
                if trace_name in self.whitelisted_trace_names or self.whitelisted_trace_names == ["all"]:
                    ftrace_core0_fd = self.ramdump.open_file('ftrace_parser/' + 'ftrace_' + trace_name + '_core0.txt','w')
                    ftrace_core1_fd = self.ramdump.open_file('ftrace_parser/' + 'ftrace_' + trace_name + '_core1.txt','w')
                    ftrace_core2_fd = self.ramdump.open_file('ftrace_parser/' + 'ftrace_' + trace_name + '_core2.txt','w')
                    ftrace_core3_fd = self.ramdump.open_file('ftrace_parser/' + 'ftrace_' + trace_name + '_core3.txt','w')
                    ftrace_core4_fd = self.ramdump.open_file('ftrace_parser/' + 'ftrace_' + trace_name + '_core4.txt','w')
                    ftrace_core5_fd = self.ramdump.open_file('ftrace_parser/' + 'ftrace_' + trace_name + '_core5.txt','w')
                    ftrace_core6_fd = self.ramdump.open_file('ftrace_parser/' + 'ftrace_' + trace_name + '_core6.txt','w')
                    ftrace_core7_fd = self.ramdump.open_file('ftrace_parser/' + 'ftrace_' + trace_name + '_core7.txt','w')
                else:
                    continue

            ftrace_file_map["000"] = BufferedWrite(ftrace_core0_fd)
            ftrace_file_map["001"] = BufferedWrite(ftrace_core1_fd)
            ftrace_file_map["002"] = BufferedWrite(ftrace_core2_fd)
            ftrace_file_map["003"] = BufferedWrite(ftrace_core3_fd)
            ftrace_file_map["004"] = BufferedWrite(ftrace_core4_fd)
            ftrace_file_map["005"] = BufferedWrite(ftrace_core5_fd)
            ftrace_file_map["006"] = BufferedWrite(ftrace_core6_fd)
            ftrace_file_map["007"] = BufferedWrite(ftrace_core7_fd)

            #start = time.time()
            sorted_dict = {k: ftrace_time_data[k] for k in sorted(ftrace_time_data)}
            for key in sorted(sorted_dict.keys()):
                for i in range(0,len(ftrace_time_data[key])):
                    line = str(ftrace_time_data[key][i])
                    if "sched_switch:" in line:
                        cpu_number = line.split("[")[1]
                        cpu_number = cpu_number.split("]")[0].replace("]","")
                        swapper_entry =  True
                        comm_flag = False
                        comm_flag_dash = False
                        prev_comm = line.split("prev_comm=")[1].split(" ")[0]
                        prev_id = line.split("prev_pid=")[1].split(" ")[0]

                        curr_comm = line.split("next_comm=")[1].split(" ")[0]
                        curr_id = line.split("next_pid=")[1].split(" ")[0]

                        switch_map[cpu_number] = curr_comm + "-" + curr_id
                        if "swapper" not in prev_comm:
                            #pid = prev_comm.split(":")[1]
                            temp_prev_comm = prev_comm
                            pid = prev_id

                            if pendig_process == False:
                                for pi in pending_update_list:
                                    #print "pending line process ++ = {0}".format(pi)
                                    if "swapper" in temp_prev_comm:
                                        ftrace_out.write(pi.replace("<TBD>", "<idle>-0"))
                                        ftrace_file_map[str(cpu_number)].write(pi.replace("<TBD>", "<idle>-0"))
                                    else:
                                        ftrace_out.write(pi.replace("<TBD>", temp_prev_comm+pid))
                                        ftrace_file_map[str(cpu_number)].write(pi.replace("<TBD>", temp_prev_comm + "-" + pid))

                                pendig_process = True
                                pending_update_list = []
                            if "swapper" in temp_prev_comm:
                                ftrace_out.write(pi.replace("<TBD>", "<idle>-0"))
                                ftrace_file_map[str(cpu_number)].write(line.replace("<TBD>", "<idle>-0"))
                            else:
                                ftrace_out.write(line.replace("<TBD>", temp_prev_comm + "-" + pid))
                                ftrace_file_map[str(cpu_number)].write(line.replace("<TBD>", temp_prev_comm + "-" + pid))

                        else:
                            #pid = prev_comm.split(":")
                            #ftrace_out.write(line.replace("next_pid", pid))
                            if pendig_process == False:
                                for pi in pending_update_list:
                                    #print "pending line process && = {0}".format(pi)
                                    ftrace_out.write(pi.replace("<TBD>", "<idle>-0"))
                                    ftrace_file_map[str(cpu_number)].write(pi.replace("<TBD>", "<idle>-0"))
                                pendig_process = True
                                pending_update_list = []
                            ftrace_out.write(line.replace("<TBD>", "<idle>-0"))
                            ftrace_file_map[str(cpu_number)].write(line.replace("<TBD>", "<idle>-0"))
                            #ctask = "<idle>-" + str(pid)
                            #switch_map[cpu_number] = ctask
                    else:
                        if "[" in line:
                            cpu_number = line.split("[")[1]
                            cpu_number = cpu_number.split("]")[0].replace("]","")
                            if cpu_number in switch_map:
                                currcomm_ctask = switch_map[cpu_number]
                            else:
                                currcomm_ctask = '<idle>-0'
                            temp_curr_comm = currcomm_ctask
                            if swapper_entry and currcomm_ctask:
                                if "swapper" not in currcomm_ctask:
                                    ftrace_out.write(line.replace("<TBD>", temp_curr_comm))
                                    ftrace_file_map[str(cpu_number)].write(line.replace("<TBD>", temp_curr_comm))
                                else:
                                    ftrace_out.write(line.replace("<TBD>", "<idle>-0"))
                                    ftrace_file_map[str(cpu_number)].write(line.replace("<TBD>", "<idle>-0"))
                            else:
                                ftrace_out.write(line.replace("<TBD>", "<idle>-0"))
                                ftrace_file_map[str(cpu_number)].write(line.replace("<TBD>", "<idle>-0"))
            #post_ftrace_event_time += (time.time()-start)
        #print("Ftrace Event Parsing took {} secs".format(ftrace_event_time))
        #print("Post Ftrace Event Sorting and Write took {} secs".format(post_ftrace_event_time))
        #print("Parse Ftrace Entry function took {} secs".format(parse_trace_entry_time))


    def parse(self):
        if parser_util.get_system_type() != 'Linux':
            self.ftrace_extract()
            return False

        try:
            crashtool = local_settings.crashtool
            trace_ext = local_settings.trace_ext
            tracecmdtool = local_settings.tracecmdtool
        except AttributeError:
            print_out_str("One of crashtool, the trace extension or" +
                          " trace-cmd is missing from local-settings.py")
            return False

        if not os.path.exists(crashtool):
            print_out_str("Couldn't find the crash tool")
            return False
        if not os.path.exists(trace_ext):
            print_out_str("Couldn't find the crash tool trace extension")
            return False
        if not os.path.exists(tracecmdtool):
            print_out_str("Couldn't find the trace-cmd tool")
            return False

        print_out_str(crashtool)
        dumps=""
        for (f, start, end, filename) in self.ramdump.ebi_files:
                if "DDR" in filename or "dram" in filename:
                    dumps += '{0}@0x{1:x},'.format(filename, start)
        pagesize = "-p {}".format(self.ramdump.get_page_size())

        commandsfile = NamedTemporaryFile(mode='w', delete=False,
                              dir=self.ramdump.outdir)
        commandsfile.write("extend " + trace_ext + "\n")
        commandsfile.write("trace dump -t " + self.ramdump.outdir +
                       "/rawtracedata\n")
        commandsfile.write("quit\n")
        commandsfile.close()

        commands = "-i " + commandsfile.name

        crashargs = [crashtool]

        kaslr_offset = self.ramdump.get_kaslr_offset()
        if kaslr_offset != 0:
            kaslroffset = "--kaslr={0}".format(hex(kaslr_offset))
            crashargs.append(kaslroffset)

        if self.ramdump.kimage_voffset is not None:
            kimagevoff="kimage_voffset={0}".format(hex(self.ramdump.kimage_voffset).replace('L',''))
            crashargs.append("--machdep")
            crashargs.append(kimagevoff)

        crashargs.extend([dumps, self.ramdump.vmlinux,
                     pagesize, commands])

        print_out_str('args to crash: {0}'.format(crashargs))

        sp = subprocess.Popen(crashargs,
                              stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE,
                              universal_newlines = True)
        out, err = sp.communicate()

        if out:
             print_out_str("crash standard output recorded.")
             std = self.ramdump.open_file('stdout_crashtool.txt')
             std.write(out);
             std.close();
        if err:
             print_out_str("crash standard error recorded.")
             std = self.ramdump.open_file('stderr_crashtool.txt')
             std.write(err);
             std.close();

        os.remove(commandsfile.name)

        if not os.path.exists(self.ramdump.outdir + "/rawtracedata"):
             print_out_str("crash failed to extract raw ftrace data")
             return False

        tracecmd_arg = self.ramdump.outdir + "/rawtracedata"
        sp = subprocess.Popen([tracecmdtool, "report", "-l", tracecmd_arg],
                              stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE,
                              universal_newlines=True)
        out, err = sp.communicate();

        if out:
             ftrace_out = self.ramdump.open_file('ftrace.txt')
             ftrace_out.write(out);
             ftrace_out.close();
             print_out_str("Ftrace successfully extracted.");
        if err:
             print_out_str("trace-cmd standard error recorded.")
             std = self.ramdump.open_file('stderr_tracecmd.txt')
             std.write(err);
             std.close();

        return True
