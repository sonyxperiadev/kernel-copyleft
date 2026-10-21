# Copyright (c) 2012-2015, 2020-2021, The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from tempfile import NamedTemporaryFile

from print_out import print_out_str
from parser_util import register_parser, RamParser

# struct msm_rtb_layout {
#        unsigned char sentinel[3];
#        unsigned char log_type;
#        unsigned long idx;
#        void *caller;
#        void *data;
#        void *timestamp;
#        void *cycle_count;
#} __attribute__ ((__packed__));

print_table = {
    'LOGK_NONE': 'print_none',
    'LOGK_READL': 'print_readlwritel',
    'LOGK_WRITEL': 'print_readlwritel',
    'LOGK_LOGBUF': 'print_logbuf',
    'LOGK_HOTPLUG': 'print_hotplug',
    'LOGK_CTXID': 'print_ctxid',
    'LOGK_TIMESTAMP': 'print_timestamp',
    'LOGK_L2CPREAD': 'print_cp_rw',
    'LOGK_L2CPWRITE': 'print_cp_rw',
    'LOGK_IRQ': 'print_irq',
}


@register_parser('--print-rtb', 'Print RTB (if enabled)', shortopt='-r')
class RTB(RamParser):

    def __init__(self, *args):
        super(RTB, self).__init__(*args)
        self.name_lookup_table = []

    def get_caller(self, caller):
        return self.ramdump.gdbmi.get_func_info(caller)

    def get_fun_name(self, addr):
        l = self.ramdump.unwind_lookup(addr)
        if l is not None:
            symname, offset = l
        else:
            symname = 'Unknown function'
        return symname

    def get_timestamp(self, rtb_ptr):
        timestamp = self.ramdump.read_structure_field(rtb_ptr, 'struct msm_rtb_layout', 'timestamp')
        if timestamp == None:
            return 0
        timestamp = round(float(timestamp)/10**9,6)
        timestamp = format(timestamp,'.6f')
        return timestamp

    def get_cycle_count(self, rtb_ptr):
        cycle_count = self.ramdump.read_structure_field(rtb_ptr, 'struct msm_rtb_layout', 'cycle_count')
        if cycle_count == None:
            return 0
        return cycle_count

    def print_none(self, rtbout, rtb_ptr, logtype):
        rtbout.write('{0} No data\n'.format(logtype))

    def print_readlwritel(self, rtbout, rtb_ptr, logtype):
        data = self.ramdump.read_structure_field(rtb_ptr, 'struct msm_rtb_layout', 'data')
        try:
            physical = self.ramdump.virt_to_phys(data)
        except:
            physical = None

        if physical is None:
            physical = "no translation found"
        else:
            physical = hex(physical).rstrip("L").lstrip("0x")

        caller = self.ramdump.read_structure_field(rtb_ptr, 'struct msm_rtb_layout', 'caller')
        func = self.get_fun_name(caller)
        line = self.get_caller(caller)
        timestamp = self.get_timestamp(rtb_ptr)
        cycle_count = self.get_cycle_count(rtb_ptr)
        rtbout.write('[{0}] [{1}] : {2} from address {3:x}({4}) called from addr {5:x} {6} {7}\n'.format(
            timestamp, cycle_count, logtype, data, physical, caller, func, line))

    def print_logbuf(self, rtbout, rtb_ptr, logtype):
        data = self.ramdump.read_structure_field(rtb_ptr, 'struct msm_rtb_layout', 'data')
        caller = self.ramdump.read_structure_field(rtb_ptr, 'struct msm_rtb_layout', 'caller')
        func = self.get_fun_name(caller)
        line = self.get_caller(caller)
        timestamp = self.get_timestamp(rtb_ptr)
        cycle_count = self.get_cycle_count(rtb_ptr)
        rtbout.write('[{0}] [{1}] : {2} log end {3:x} called from addr {4:x} {5} {6}\n'.format(
            timestamp, cycle_count, logtype, data, caller, func, line))

    def print_hotplug(self, rtbout, rtb_ptr, logtype):
        data = self.ramdump.read_structure_field(rtb_ptr, 'struct msm_rtb_layout', 'data')
        caller = self.ramdump.read_structure_field(rtb_ptr, 'struct msm_rtb_layout', 'caller')
        func = self.get_fun_name(caller)
        line = self.get_caller(caller)
        timestamp = self.get_timestamp(rtb_ptr)
        cycle_count = self.get_cycle_count(rtb_ptr)
        rtbout.write('[{0}] [{1}] : {2} cpu data {3:x} called from addr {4:x} {5} {6}\n'.format(
            timestamp, cycle_count, logtype, data, caller, func, line))

    def print_ctxid(self, rtbout, rtb_ptr, logtype):
        data = self.ramdump.read_structure_field(rtb_ptr, 'struct msm_rtb_layout', 'data')
        caller = self.ramdump.read_structure_field(rtb_ptr, 'struct msm_rtb_layout', 'caller')
        func = self.get_fun_name(caller)
        line = self.get_caller(caller)
        timestamp = self.get_timestamp(rtb_ptr)
        cycle_count = self.get_cycle_count(rtb_ptr)
        rtbout.write('[{0}] [{1}] : {2} context id {3:x} called from addr {4:x} {5} {6}\n'.format(
            timestamp, cycle_count, logtype, data, caller, func, line))

    def print_timestamp(self, rtbout, rtb_ptr, logtype):
        data = self.ramdump.read_structure_field(rtb_ptr, 'struct msm_rtb_layout', 'data')
        caller = self.ramdump.read_structure_field(rtb_ptr, 'struct msm_rtb_layout', 'caller')
        cycle_count = self.get_cycle_count(rtb_ptr)
        rtbout.write('[{0}] : [{1}] Timestamp: {2:x}{3:x}\n'.format(
            cycle_count, logtype, data, caller))

    def print_cp_rw(self, rtbout, rtb_ptr, logtype):
        data = self.ramdump.read_structure_field(rtb_ptr, 'struct msm_rtb_layout', 'data')
        caller = self.ramdump.read_structure_field(rtb_ptr, 'struct msm_rtb_layout', 'caller')
        func = self.get_fun_name(caller)
        line = self.get_caller(caller)
        timestamp = self.get_timestamp(rtb_ptr)
        cycle_count = self.get_cycle_count(rtb_ptr)
        rtbout.write('[{0}] [{1}] : {2} from offset {3:x} called from addr {4:x} {5} {6}\n'.format(
            timestamp, cycle_count, logtype, data, caller, func, line))

    def print_irq(self, rtbout, rtb_ptr, logtype):
        data = self.ramdump.read_structure_field(rtb_ptr, 'struct msm_rtb_layout', 'data')
        caller = self.ramdump.read_structure_field(rtb_ptr, 'struct msm_rtb_layout', 'caller')
        func = self.get_fun_name(caller)
        line = self.get_caller(caller)
        timestamp = self.get_timestamp(rtb_ptr)
        cycle_count = self.get_cycle_count(rtb_ptr)
        rtbout.write('[{0}] [{1}] : {2} interrupt 0x{3:x} {4} handled from addr {5:x} {6} {7}\n'.format(
            timestamp, cycle_count, logtype, data, data, caller, func, line))

    def next_rtb_entry(self, index, step_size, mask):
        unused_buffer_size = (mask + 1) % step_size
        #check for wraparound
        if ((index + step_size + unused_buffer_size) & mask) < (index & mask):
            return (index + step_size + unused_buffer_size) & mask
        return (index + step_size) & mask

    def parse(self):
        rtb_ptr = self.ramdump.address_of('msm_rtb_ptr')
        if rtb_ptr is None:
            rtb = self.ramdump.address_of('msm_rtb')
        else:
            rtb = self.ramdump.read_pointer(rtb_ptr)
        if rtb is None and self.ramdump.minidump:
            rtb_seg = next((s for s in self.ramdump.elffile.iter_sections() if s.name == 'KRTB_BUF'), None)
            if rtb_seg is not None:
                rtb = rtb_seg['sh_addr']
                rtb_ptr = True # Not a pointer, but used below to determine whether new RTB layout is used
        if rtb is None:
            print_out_str(
                '[!] RTB was not enabled in this build. No RTB files will be generated')
            return
        self.name_lookup_table = self.ramdump.gdbmi.get_enum_lookup_table(
            'logk_event_type', 32)
        step_size_offset = self.ramdump.field_offset(
            'struct msm_rtb_state', 'step_size')
        nentries_offset = self.ramdump.field_offset(
            'struct msm_rtb_state', 'nentries')
        rtb_entry_offset = self.ramdump.field_offset(
            'struct msm_rtb_state', 'rtb')
        idx_offset = self.ramdump.field_offset('struct msm_rtb_layout', 'idx')
        rtb_entry_size = self.ramdump.sizeof('struct msm_rtb_layout')
        step_size = self.ramdump.read_u32(rtb + step_size_offset)
        total_entries = self.ramdump.read_int(rtb + nentries_offset)
        rtb_read_ptr = self.ramdump.read_word(rtb + rtb_entry_offset)

        if rtb_ptr is not None:
            rtb_idx_array_offset = self.ramdump.field_offset('struct msm_rtb_state', 'msm_rtb_idx')
            rtb_idx_size = self.ramdump.sizeof('struct rtb_idx')
            rtb_idx_atomic_offset = self.ramdump.field_offset('struct rtb_idx', 'idx')

        if step_size is None or step_size > self.ramdump.get_num_cpus():
            print_out_str('RTB dump looks corrupt! Got step_size=%s' %
                          hex(step_size) if step_size is not None else None)
            return
        for i in range(0, step_size):
            rtb_out = self.ramdump.open_file('msm_rtb{0}.txt'.format(i))
            gdb_cmd = NamedTemporaryFile(mode='w+t', delete=False)
            gdb_out = NamedTemporaryFile(mode='w+t', delete=False)
            mask = self.ramdump.read_int(rtb + nentries_offset) - 1
            if rtb_ptr is not None:
                last = self.ramdump.read_int(rtb +
                                             rtb_idx_array_offset + (i * rtb_idx_size) +
                                             rtb_idx_atomic_offset)
            else:
                if step_size == 1:
                    last = self.ramdump.read_int(
                        self.ramdump.address_of('msm_rtb_idx'))
                else:
                    last = self.ramdump.read_int(self.ramdump.address_of(
                        'msm_rtb_idx_cpu'), cpu=i )
            last = last & mask
            last_ptr = 0
            next_ptr = 0
            next_entry = 0
            while True:
                next_entry = self.next_rtb_entry(last, step_size, mask)
                last_ptr = rtb_read_ptr + last * rtb_entry_size + idx_offset
                next_ptr = rtb_read_ptr + next_entry * \
                    rtb_entry_size + idx_offset
                a = self.ramdump.read_int(last_ptr)
                b = self.ramdump.read_int(next_ptr)
                if a < b:
                    last = next_entry
                if next_entry != last:
                    break
            stop = 0
            rtb_logtype_offset = self.ramdump.field_offset(
                'struct msm_rtb_layout', 'log_type')
            rtb_idx_offset = self.ramdump.field_offset(
                'struct msm_rtb_layout', 'idx')
            while True:
                ptr = rtb_read_ptr + next_entry * rtb_entry_size
                stamp = self.ramdump.read_int(ptr + rtb_idx_offset)
                if stamp is None:
                    break
                rtb_out.write('{0:x} '.format(stamp))
                item = self.ramdump.read_byte(ptr + rtb_logtype_offset)
                item = item & 0x7F
                name_str = '(unknown)'
                if item >= len(self.name_lookup_table) or item < 0:
                    self.print_none(rtb_out, ptr, None)
                else:
                    name_str = self.name_lookup_table[item]
                    if name_str not in print_table:
                        self.print_none(rtb_out, ptr, None)
                    else:
                        func = print_table[name_str]
                        getattr(RTB, func)(self, rtb_out, ptr, name_str)
                if next_entry == last:
                    stop = 1
                next_entry = self.next_rtb_entry(next_entry, step_size, mask)
                if (stop == 1):
                    break
            print_out_str('Wrote RTB to msm_rtb{0}.txt'.format(i))
            rtb_out.close()
