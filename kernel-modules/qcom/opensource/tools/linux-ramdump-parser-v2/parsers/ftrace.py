# Copyright (c) 2017-2022, The Linux Foundation. All rights reserved.
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

import os,re
from collections import OrderedDict

from parser_util import register_parser, RamParser
from print_out import print_out_str
from tempfile import NamedTemporaryFile
from .ftrace_event_list import FtraceParser_Event_List
from .ftrace_event import FtraceParser_Event, BufferedWrite
import linux_list as llist
#import time

@register_parser('--dump-ftrace', 'extract ftrace by iterate the ring buffer page',optional=True)
class FtraceParser(RamParser):

    def __init__(self, *args):
        super(FtraceParser, self).__init__(*args)
        self.format_event_map = OrderedDict()
        self.format_event_field_map = OrderedDict()
        self.event_call = 'struct trace_event_call'
        self.event_class = 'struct trace_event_class'
        self.whitelisted_trace_names = []
        self.ftrace_buffer_size_kb = None
        self.per_cpu_buffer_pages = None
        self.savedcmd = self.ramdump.read_pdatatype('savedcmd')
        if len(self.ramdump.ftrace_args):
            self.whitelisted_trace_names = self.ramdump.ftrace_args
        if self.ramdump.ftrace_max_size:
            self.per_cpu_buffer_pages = int(self.ramdump.ftrace_max_size / (self.ramdump.get_page_size() >> 10))
        self.trace_buffers = {}

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
        if type_str == None or field_name == None:
            str_error =  'v.v (struct ftrace_event_field)0x{0:x} type_str or field_name is None'.format(common_list)
            print_out_str(str_error)
            return
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

        list_walker = llist.ListWalker(ram_dump, common_field_list, field_next_offset)
        list_walker.walk_prev(self.ftrace_field_func, ram_dump)
        self.formats_out.write("\n")

        event_class = ram_dump.read_word(ftrace_list + class_offset)
        field_list = event_class + fields_offset
        list_walker = llist.ListWalker(ram_dump, field_list, field_next_offset)
        list_walker.walk_prev(self.ftrace_field_func, ram_dump)
        self.formats_out.write("\n")
        self.formats_out.write("print fmt: {0}\n".format(fmt_str))
        fmt_list = []
        fmt_list.append(self.format_event_field_map)
        fmt_list.append(fmt_str)
        self.format_event_map[name_str] = fmt_list
        self.format_event_field_map = OrderedDict()

    def ftrace_get_format(self):
        self.formats_out = self.ramdump.open_file('formats.txt')
        fevent_list = FtraceParser_Event_List(self.ramdump)

        ftrace_events_list = self.ramdump.address_of('ftrace_events')
        next_offset = self.ramdump.field_offset(self.event_call, 'list')
        list_walker = llist.ListWalker(self.ramdump, ftrace_events_list, next_offset)
        list_walker.walk_prev(self.ftrace_events_func, self.ramdump)
        self.formats_out.close()
        return fevent_list

    def ftrace_get_buffers(self, trace_buffer, trace_buffer_name_offset):
        trace_buffer_name = self.ramdump.read_word(trace_buffer + trace_buffer_name_offset)
        if not (trace_buffer_name):
            trace_name = None
        else:
            trace_name = self.ramdump.read_cstring(trace_buffer_name, 256)
        if trace_name is None or trace_name == 0x0 or trace_name == "0x0" or trace_name == "None" or trace_name == "null" or len(trace_name) < 1:
            trace_name = "global"
        trace_buffer_info = {}
        trace_buffer_info['addr'] = trace_buffer
        self.trace_buffers[trace_name] = trace_buffer_info
        return

    def ftrace_get_buffer_pages(self):
        if self.ramdump.kernel_version >= (5, 10):
            trace_buffer_offset = self.ramdump.field_offset(
                'struct trace_array', 'array_buffer')
            ring_trace_buffer_ptr = self.ramdump.field_offset(
                'struct array_buffer', 'buffer')
            ring_trace_buffer_base_addr = self.ramdump.field_offset(
                'struct trace_buffer', 'buffers')
            cpu_offset = self.ramdump.field_offset(
                'struct trace_buffer', 'cpus')
        else:
            trace_buffer_offset = self.ramdump.field_offset(
                'struct trace_array', 'trace_buffer')
            ring_trace_buffer_ptr = self.ramdump.field_offset(
                'struct trace_buffer', 'buffer')
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
            cpu_offset = self.ramdump.field_offset(
                'struct ring_buffer', 'cpus')

        ring_trace_buffer_nr_pages = self.ramdump.field_offset(
            'struct ring_buffer_per_cpu', 'nr_pages')

        for buffer_name, trace_buffer_info in self.trace_buffers.items():
            trace_array = trace_buffer_info['addr']
            trace_buffer_ptr_data = self.ramdump.read_pointer(trace_array + trace_buffer_offset)
            ring_trace_buffer_data = trace_buffer_ptr_data + trace_buffer_offset
            ring_trace_buffer_base_data = self.ramdump.read_pointer(ring_trace_buffer_data + ring_trace_buffer_ptr)
            ring_trace_buffer_base_data1 = self.ramdump.read_pointer(ring_trace_buffer_base_data + ring_trace_buffer_base_addr)
            numcpus = self.ramdump.read_int(ring_trace_buffer_base_data + cpu_offset)
            numcpus = numcpus if numcpus < 0x20 else 8

            nr_total_buffer_pages = 0
            trace_buffer_info['parse'] = True
            trace_buffer_info['sibling'] = None
            trace_buffer_info['cpus'] = numcpus
            trace_buffer_info['rb_per_cpu'] = [None] * numcpus
            trace_buffer_info['nr_pages_per_buffer'] = [None] * numcpus
            for cpu_idx in range(0, numcpus):
                array_ptr = (ring_trace_buffer_base_data1 + self.ramdump.sizeof('void *') * cpu_idx)
                b = self.ramdump.read_pointer(array_ptr)
                if b is None or b == 0x0:
                    continue
                nr_pages =  self.ramdump.read_pointer(b + ring_trace_buffer_nr_pages)
                if nr_pages is None:
                    continue
                if self.per_cpu_buffer_pages and self.per_cpu_buffer_pages < nr_pages:
                    nr_pages = self.per_cpu_buffer_pages
                nr_total_buffer_pages = nr_total_buffer_pages +  nr_pages
                trace_buffer_info['nr_pages_per_buffer'][cpu_idx] = nr_pages
                trace_buffer_info['rb_per_cpu'][cpu_idx] = b
            trace_buffer_info['nr_total_buffer_pages'] = nr_total_buffer_pages
        return

    def ftrace_main_buffer(self):
        main_trace_name = "global"
        if "main" in self.trace_buffers:
            main_trace_name = "main"
            if "global" in self.trace_buffers:
                self.trace_buffers[main_trace_name]['sibling'] = "global"
                self.trace_buffers['global']['parse'] = False
        if main_trace_name not in self.whitelisted_trace_names:
            self.whitelisted_trace_names.append(main_trace_name)
        return main_trace_name

    def ftrace_parse_buffers(self, trace_buffer_name, ftrace_out, fevent_list):
        trace_buffer_info = self.trace_buffers[trace_buffer_name]
        print_out_str("Total pages across cpu trace buffers = {} for {}".format(
            round(trace_buffer_info['nr_total_buffer_pages']), trace_buffer_name))
        ftrace_time_data = {}
        for cpu_idx in range(0, len(trace_buffer_info['rb_per_cpu'])):
            nrpages_limit = trace_buffer_info['nr_pages_per_buffer'][cpu_idx] if self.per_cpu_buffer_pages else None
            per_cpu_buffer = trace_buffer_info['rb_per_cpu'][cpu_idx]
            if per_cpu_buffer is not None:
                evt = FtraceParser_Event(self.ramdump,ftrace_out,cpu_idx,fevent_list.ftrace_event_type,
                        fevent_list.ftrace_raw_struct_type,ftrace_time_data,self.format_event_map,self.savedcmd)
                evt.ring_buffer_per_cpu_parsing(per_cpu_buffer, nrpages_limit)
        return ftrace_time_data

    def ftrace_extract(self):
        trace_array_list = self.ramdump.address_of('ftrace_trace_arrays')
        list_offset = self.ramdump.field_offset('struct trace_array', 'list')
        trace_buffer_name_offset = self.ramdump.field_offset(
            'struct trace_array', 'name')
        list_walker = llist.ListWalker(self.ramdump, trace_array_list, list_offset)
        list_walker.walk_prev(self.ftrace_get_buffers, trace_buffer_name_offset)
        if len(self.trace_buffers) == 0:
            list_walker.walk(self.ftrace_get_buffers, trace_buffer_name_offset)
        if len(self.trace_buffers) == 0:
            print_out_str("A ftrace buffer is not found")
            return
        self.ftrace_get_buffer_pages()
        main_trace = self.ftrace_main_buffer()

        ftrace_event_time = 0
        post_ftrace_event_time = 0
        log_pattern = re.compile(r'\s*(.*)-(\d+)\s*\[(\d+)\]\s*.*')
        fevent_list = self.ftrace_get_format();
        for trace_buffer_name, trace_buffer_info in self.trace_buffers.items():
            trace_array = trace_buffer_info['addr']
            trace_name = trace_buffer_name
            if trace_name == main_trace:
                trace_filename = "ftrace"
            else:
                trace_filename = os.path.join("ftrace_parser", "ftrace_{}".format(trace_name))

            ftrace_out = None
            if trace_name in self.whitelisted_trace_names or "all" in self.whitelisted_trace_names:
                fout = self.ramdump.open_file('{}.txt'.format(trace_filename),'w')
                ftrace_out = BufferedWrite(fout)
                """
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
                """
            else:
                continue

            if trace_buffer_info['parse'] == False:
                if ftrace_out:
                    ftrace_out.close()
                continue

            #start = time.time()
            ftrace_time_data = self.ftrace_parse_buffers(trace_buffer_name, ftrace_out, fevent_list)
            if trace_buffer_info['sibling'] and trace_buffer_info['sibling'] in self.trace_buffers:
                sibling_ftrace_time_data = self.ftrace_parse_buffers(
                    trace_buffer_info['sibling'], ftrace_out, fevent_list)
                if len(sibling_ftrace_time_data):
                    ftrace_time_data.update(sibling_ftrace_time_data)
            #ftrace_event_time += (time.time()-start)

            switch_map = {}
            ftrace_file_map = {}
            for cpu_idx in range(0, trace_buffer_info['cpus']):
                ftrace_file_map["{:03d}".format(cpu_idx)] = BufferedWrite(
                    self.ramdump.open_file('{}_core{}.txt'.format(trace_filename, cpu_idx), 'w'))

            #start = time.time()
            sorted_dict = {k: ftrace_time_data[k] for k in sorted(ftrace_time_data)}
            for key in sorted(sorted_dict.keys()):
                for i in range(0,len(ftrace_time_data[key])):
                    line = str(ftrace_time_data[key][i])
                    replaced_line = line
                    trace_log = log_pattern.match(line)
                    bestguess_pid = None
                    bestguess_comm = None
                    if bool(trace_log):
                        cpu_number = trace_log.group(3)
                        entry_pid = trace_log.group(2)
                    else:
                        cpu_number = None
                        entry_pid = None

                    if "sched_switch:" in line:
                        prev_comm = line.split("prev_comm=")[1].split(" ")[0]
                        prev_pid = line.split("prev_pid=")[1].split(" ")[0]
                        curr_comm = line.split("next_comm=")[1].split(" ")[0]
                        curr_pid = line.split("next_pid=")[1].split(" ")[0]
                        if cpu_number not in switch_map:
                            switch_map[cpu_number] = {}
                        switch_map[cpu_number]["comm"] = curr_comm
                        switch_map[cpu_number]["pid"] = curr_pid
                        bestguess_pid = prev_pid
                        bestguess_comm = prev_comm
                    elif "<TBD>" in line and cpu_number in switch_map:
                        bestguess_comm = switch_map[cpu_number]["comm"]
                        bestguess_pid = switch_map[cpu_number]["pid"]

                    if "<TBD>" in line:
                        if entry_pid is not None and bestguess_pid is not None and int(entry_pid) == int(bestguess_pid):
                            replaced_line = line.replace("<TBD>", bestguess_comm)
                        else:
                            replaced_line = line.replace("<TBD>", "<...>")
                    ftrace_out.write(replaced_line)
                    ftrace_file_map[str(cpu_number)].write(replaced_line)

            ftrace_out.close()
            for cpu_idx in range(0, trace_buffer_info['cpus']):
                ftrace_file_map["{:03d}".format(cpu_idx)].close()
            #post_ftrace_event_time += (time.time()-start)

        #print("Ftrace Event Parsing took {} secs".format(ftrace_event_time))
        #print("Post Ftrace Event Sorting and Write took {} secs".format(post_ftrace_event_time))
        return

    def parse(self):
        if self.ramdump.ftrace_limit_time == 0:
            self.ftrace_extract()
        else:
            from func_timeout import func_timeout
            print_out_str("Limit ftrace parser running time to {}s".format(self.ramdump.ftrace_limit_time))
            func_timeout(self.ramdump.ftrace_limit_time, self.ftrace_extract)
