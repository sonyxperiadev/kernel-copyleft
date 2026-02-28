# Copyright (c) 2020, The Linux Foundation. All rights reserved.
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

from parser_util import register_parser, RamParser
from print_out import print_out_str

class FtraceParser_Event_List(object):
    def __init__(self, ramdump):
        self.ramdump = ramdump
        ftrace_event_call_list_offset = self.ramdump.field_offset("struct trace_event_call" , "list")
        #print ("self.ftrace_event_call_list_offset = {0}".format(hex(self.ftrace_event_call_list_offset)))
        ftrace_events_head = self.ramdump.address_of("ftrace_events")
        ftrace_events_entry_offset = self.ramdump.field_offset("struct list_head", "next")
        ftrace_events_entry = self.ramdump.read_pointer(ftrace_events_head + ftrace_events_entry_offset)
        #print ("self.ftrace_events_entry = {0}".format(hex(self.ftrace_events_entry)))

        ftrace_event_call_offset = self.ramdump.field_offset("struct trace_event_call" , "event")
        tp_offset = self.ramdump.field_offset("struct trace_event_call", "tp")
        ftrace_event_call_name_offset = self.ramdump.field_offset("struct tracepoint", "name")
        ftrace_event_offset = self.ramdump.field_offset("struct trace_event", "type")

        self.ftrace_event_type = {}
        self.ftrace_raw_struct_type = {}

        while ftrace_events_entry != ftrace_events_head:
            ftrace_event = ftrace_events_entry - ftrace_event_call_list_offset
            #ftrace_event_data = self.ramdump.read_u64(ftrace_event + ftrace_event_call_offset)
            ftrace_event_data = ftrace_event + ftrace_event_call_offset
            tp_data = ftrace_event + tp_offset
            if ftrace_event_data:
                #print ("ftrace_event_data +++ {0}".format(hex(ftrace_event_data)))
                event_type = self.ramdump.read_u16(ftrace_event_data + ftrace_event_offset)
                if self.ramdump.arm64:
                    event_name = self.ramdump.read_u64(tp_data)
                    #print ("event_name +++ {0}".format((hex(event_name))))

                    event_name_value = self.ramdump.read_u64(event_name + ftrace_event_call_name_offset)
                else:
                    event_name = self.ramdump.read_u32(tp_data)
                    #print ("event_name +++ {0}".format((hex(event_name))))

                    event_name_value = self.ramdump.read_u32(event_name + ftrace_event_call_name_offset)
                #print ("event_name_value +++ {0}".format((event_name_value)))

                event_name1 = self.ramdump.read_cstring(event_name_value)
                event_name2 = self.ramdump.read_cstring(event_name)
                if "6" == str(event_type):  #TRACE_BPRINT
                    print_out_str("ftrace_event_data => {0} ftrace_event >> {1} tp_data >> {2} event_name >> {3} "
                        "event_name_value >> {4} event_name2 {5} event_type {6}".format(hex(ftrace_event_data),
                         hex(ftrace_event),hex(tp_data),hex(event_name),hex(event_name_value),event_name2,event_type))
                    self.ftrace_event_type[str(event_type)] = "bprint"
                    self.ftrace_raw_struct_type[str(event_type)] = "bprint"
                elif "5" == str(event_type):
                    print_out_str("ftrace_event_data => {0} ftrace_event >> {1} tp_data >> {2} event_name >> {3} "
                        "event_name_value >> {4} event_name2 {5} event_type {6}".format(hex(ftrace_event_data),
                        hex(ftrace_event),hex(tp_data),hex(event_name),hex(event_name_value),event_name2,event_type))
                    self.ftrace_event_type[str(event_type)] = "print"
                    self.ftrace_raw_struct_type[str(event_type)] = "print"
                elif '4' == str(event_type):  #TRACE_STACK
                    print_out_str("ftrace_event_data => {0} ftrace_event >> {1} tp_data >> {2} event_name >> {3} "
                        "event_name_value >> {4} event_name2 {5} event_type {6}".format(hex(ftrace_event_data),
                        hex(ftrace_event),hex(tp_data),hex(event_name),hex(event_name_value),event_name2,event_type))
                    self.ftrace_event_type[str(event_type)] = "kernel_stack"
                    self.ftrace_raw_struct_type[str(event_type)] = "kernel_stack"
                elif '12' == str(event_type):  #TRACE_USER_STACK
                    print_out_str("ftrace_event_data => {0} ftrace_event >> {1} tp_data >> {2} event_name >> {3} "
                        "event_name_value >> {4} event_name2 {5} event_type {6}".format(hex(ftrace_event_data),
                        hex(ftrace_event),hex(tp_data),hex(event_name),hex(event_name_value),event_name2,event_type))
                    self.ftrace_event_type[str(event_type)] = "user_stack"
                    self.ftrace_raw_struct_type[str(event_type)] = "user_stack"
                elif "14" == str(event_type):
                    print_out_str("ftrace_event_data => {0} ftrace_event >> {1} tp_data >> {2} event_name >> {3} "
                        "event_name_value >> {4} event_name2 {5} event_type {6}".format(hex(ftrace_event_data),
                        hex(ftrace_event),hex(tp_data),hex(event_name),hex(event_name_value),event_name2,event_type))
                    self.ftrace_event_type[str(event_type)] = "bputs"
                    self.ftrace_raw_struct_type[str(event_type)] = "bputs"
                else:
                    self.ftrace_event_type[str(event_type)] = str(event_name1)
                    self.ftrace_raw_struct_type[str(event_type)] = "trace_event_raw_" + str(event_name1)
                ftrace_events_entry = self.ramdump.read_pointer(ftrace_events_entry + ftrace_events_entry_offset)
            else:
                break