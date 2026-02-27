# Copyright (c) 2012,2014-2015,2017-2020 The Linux Foundation. All rights reserved.
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

from print_out import print_out_str
from parser_util import register_parser, RamParser
from mm import pfn_to_page, page_buddy, page_count, for_each_pfn
from mm import page_to_pfn, pfn_to_section
import sys
import os
from collections import defaultdict

class StackDepot(object):
    def __init__(self, ramdump):
        self.ramdump = ramdump

        if self.ramdump.field_offset('union handle_parts', 'pool_index') is not None:
            self.stack_slabs = self.ramdump.address_of('stack_pools')
        else:
            self.stack_slabs = self.ramdump.address_of('stack_slabs')
        self.stack_slabs_size = self.ramdump.sizeof('void *')
        self.stack_trace_entry_size = self.ramdump.sizeof('unsigned long')
        self.stack_trace_entries_offset = self.ramdump.field_offset(
                'struct stack_record', 'entries')
        depot_stack_bits = self.ramdump.sizeof('depot_stack_handle_t') * 8
        stack_alloc_null_protection_bits = 1
        stack_alloc_order = 2
        self.stack_alloc_align = 4
        self.stack_alloc_offset_bits = stack_alloc_order + self.ramdump.page_shift - self.stack_alloc_align
        self.stack_alloc_index_bits = depot_stack_bits - stack_alloc_null_protection_bits - self.stack_alloc_offset_bits
        if self.ramdump.field_offset('union handle_parts', 'extra') is not None:
            self.stack_alloc_index_bits -= 5

    def stack_depot_fetch(self, handle, symbol=True):
        if handle == 0 or handle == None:
            return -1, None, None

        slabindex = handle & ((1 << self.stack_alloc_index_bits) - 1)
        handle_offset = (handle >> self.stack_alloc_index_bits) & \
                            ((1 << self.stack_alloc_offset_bits) - 1)
        handle_offset = handle_offset << self.stack_alloc_align

        slab = self.ramdump.read_word(
            self.stack_slabs + (self.stack_slabs_size * slabindex))
        if slab is None:
            return -1, None, None
        stack = slab + handle_offset

        stack_nr_entries = self.ramdump.read_structure_field(
            stack, 'struct stack_record', 'size')

        if stack_nr_entries is None:
            return -1, None, None
        if stack_nr_entries <= 0 or stack_nr_entries > 16:
            return -1, None, None

        stack_addr = []
        stack_str = ''
        for i in range(0, stack_nr_entries):
            addr = self.ramdump.read_word(
                    stack + self.stack_trace_entries_offset + i *
                    self.stack_trace_entry_size)

            if not addr:
                break
            stack_addr.append(addr)
            if symbol:
                look = self.ramdump.unwind_lookup(addr)
                if look is None:
                    break
                symname, offset = look
                unwind_dat = '      [<{0:x}>] {1}+0x{2:x}\n'.format(
                    addr, symname, offset)
                stack_str = stack_str + unwind_dat
            else:
                stack_str += str(addr)
        return stack_nr_entries, stack_addr, stack_str

class PageTrace(object):
    def __init__(self, ramdump):
        self.ramdump = ramdump

        self.trace_entry_size = self.ramdump.sizeof('unsigned long')
        self.trace_offset = 0
        self.nr_entries_offset = 0
        self.trace_entries_offset = 0

        if (self.ramdump.kernel_version <= (3, 19, 0)):
            self.trace_offset = self.ramdump.field_offset('struct page', 'trace')
            self.nr_entries_offset = self.ramdump.field_offset(
                'struct stack_trace', 'nr_entries')
            self.trace_entries_offset = self.ramdump.field_offset(
                'struct page', 'trace_entries')

        else:
            self.trace_offset = self.ramdump.field_offset(
                            'struct page_ext', 'trace')
            if self.ramdump.is_config_defined('CONFIG_STACKDEPOT'):
                self.stackdepot = StackDepot(self.ramdump)
            else:
                self.trace_entries_offset = self.ramdump.field_offset(
                            'struct page_ext', 'trace_entries')

            self.nr_entries_offset = self.ramdump.field_offset(
                        'struct page_ext', 'nr_entries')

            self.page_ext_size = self.ramdump.sizeof("struct page_ext")
            if self.ramdump.kernel_version >= (4, 9, 0):
                self.page_owner_size = self.ramdump.sizeof("struct page_owner")
                if self.ramdump.address_of("page_ext_size"):
                    self.page_ext_size = self.ramdump.read("page_ext_size")
                else:
                    self.page_ext_size = self.page_ext_size + self.page_owner_size
                self.page_owner_ops_offset = self.ramdump.read_structure_field(
                    'page_owner_ops', 'struct page_ext_operations', 'offset')

        '''
        Following based upon definition in include/linux/mmzone.h

        #ifndef CONFIG_FORCE_MAX_ZONEORDER
        #define MAX_ORDER 11
        #else
        #define MAX_ORDER CONFIG_FORCE_MAX_ZONEORDER
        #endif
        '''
        try:
            self.max_order = int(self.ramdump.get_config_val(
                                "CONFIG_FORCE_MAX_ZONEORDER"))
        except:
            self.max_order = 11
        self.offset_comm = self.ramdump.field_offset('struct page_owner', 'comm')

    def page_trace(self, pfn, alloc):
        offset = 0
        struct_holding_trace_entries = 0
        gfp = 0
        comm = -1

        if not alloc and self.ramdump.kernel_version < (5, 4, 0):
            return -1, -1, -1, -1, -1, -1

        page = pfn_to_page(self.ramdump, pfn)
        order = 0

        if (self.ramdump.kernel_version <= (3, 19, 0)):
            nr_trace_entries = self.ramdump.read_int(
                page + self.trace_offset + self.nr_entries_offset)
            struct_holding_trace_entries = page
            order = self.ramdump.read_structure_field(
                page, 'struct page', 'order')
        else:
            phys = pfn << self.ramdump.page_shift
            if phys is None or phys == 0:
                return -1, -1, -1, -1, -1, -1
            page_ext = self.ramdump.mm.lookup_page_ext(pfn)
            """
            page_ext will be null here if the first page of a section is not valid.
            See page_ext_init().
            """
            if not page_ext:
                return -1, -1, -1, -1, -1, -1

            if self.ramdump.arm64:
                temp_page_ext = page_ext + (pfn * self.page_ext_size)
            else:
                pfn_index = pfn - (self.ramdump.phys_offset >> self.ramdump.page_shift)
                temp_page_ext = page_ext + (pfn_index * self.page_ext_size)

            if self.ramdump.kernel_version >= (4, 9, 0):
                temp_page_ext = temp_page_ext + self.page_owner_ops_offset
                order = self.ramdump.read_structure_field(
                            temp_page_ext, 'struct page_owner', 'order')
                if alloc:
                    pid = self.ramdump.read_structure_field(
                                temp_page_ext, 'struct page_owner', 'pid')
                    ts_nsec = self.ramdump.read_structure_field(
                                temp_page_ext, 'struct page_owner', 'ts_nsec')
                    gfp = self.ramdump.read_structure_field(
                                temp_page_ext, 'struct page_owner', 'gfp_mask')
                    if self.offset_comm is not None:
                        comm = self.ramdump.read_cstring(temp_page_ext + self.offset_comm, 16)
                else:
                    pid = -1
                    ts_nsec = self.ramdump.read_structure_field(temp_page_ext,
                              'struct page_owner', 'free_ts_nsec')
                    gfp = self.ramdump.read_structure_field(
                                temp_page_ext, 'struct page_owner', 'gfp_mask')
                    if ts_nsec is None:
                        ts_nsec = -1
            else:
                order = self.ramdump.read_structure_field(
                            temp_page_ext, 'struct page_ext', 'order')

            alloc_str = ''
            if not self.ramdump.is_config_defined('CONFIG_STACKDEPOT'):
                nr_trace_entries = self.ramdump.read_int(
                    temp_page_ext + self.nr_entries_offset)
                struct_holding_trace_entries = temp_page_ext

                if nr_trace_entries is not None:
                    if  nr_trace_entries > 0 and nr_trace_entries <= 16:
                        for i in range(0, nr_trace_entries):
                            addr = self.ramdump.read_word(
                                    struct_holding_trace_entries + self.trace_entries_offset + i *
                                    self.trace_entry_size)
                            if not addr:
                                break
                            look = self.ramdump.unwind_lookup(addr)
                            if look is None:
                                break
                            symname, offset = look
                            unwind_dat = '      [<{0:x}>] {1}+0x{2:x}\n'.format(
                                addr, symname, offset)
                            alloc_str = alloc_str + unwind_dat
            else:
                if self.ramdump.kernel_version >= (4, 9, 0):
                    if not alloc:
                        handle_str = 'free_handle'
                    else:
                        handle_str = 'handle'
                    handle = self.ramdump.read_structure_field(
                        temp_page_ext, 'struct page_owner', handle_str)
                else:
                    handle = self.ramdump.read_structure_field(
                        temp_page_ext, 'struct page_ext', 'handle')

                if handle == 0 or handle == None:
                    return -1, -1, -1, -1, -1, -1

                nr_trace_entries, stack_addrs, alloc_str = self.stackdepot.stack_depot_fetch(handle)

        if nr_trace_entries is None:
            return -1, -1, -1, -1, -1, -1
        if nr_trace_entries <= 0 or nr_trace_entries > 16:
            return -1, -1, -1, -1, -1, -1
        if order >= self.max_order:
            return -1, -1, -1, -1, -1, -1

        return alloc_str, order, pid, ts_nsec, gfp, comm

@register_parser('--print-pagetracking', 'print page tracking information (if available)')
class PageTracking(RamParser):

    def __init__(self, *args):
        super(PageTracking, self).__init__(*args)
        '''
        Following based upon definition in include/linux/mmzone.h

        #ifndef CONFIG_FORCE_MAX_ZONEORDER
        #define MAX_ORDER 11
        #else
        #define MAX_ORDER CONFIG_FORCE_MAX_ZONEORDER
        #endif
        '''
        try:
            self.max_order = int(self.ramdump.get_config_val(
                                "CONFIG_FORCE_MAX_ZONEORDER"))
        except:
            self.max_order = 11

        self.pagetrace = PageTrace(self.ramdump)
        return

    def parse_output(self, pfn, out_tracking, out_tracking_freed,
                     page_size, sorted_pages):
        str_f = "PFN : 0x{0:x}-0x{1:x} Page : 0x{2:x} Order : {3} PID : {4} {5}ts_nsec {6} gfp 0x{7:x}\n" \
              "{8}\n"
        page = pfn_to_page(self.ramdump, pfn)
        order = 0
        if (page_buddy(self.ramdump, page) or
            page_count(self.ramdump, page) == 0):
            function_list, order, pid, ts_nsec, gfp, comm = self.pagetrace.page_trace(pfn, False)
            if function_list == -1:
                return
            out_tracking_freed.write(str_f.format(pfn, pfn + (1 << order) - 1,
                                                  page, order, pid,
                                                  "Comm: {} ".format(comm) if comm != -1 else "",
                                                  ts_nsec, gfp, function_list))
            return

        function_list, order, pid, ts_nsec, gfp, comm = self.pagetrace.page_trace(pfn, True)
        if function_list == -1:
            return
        if order >= self.max_order:
            out_tracking.write('PFN 0x{:x} page 0x{:x} skip as order '
                               '0x{:x}\n'.format(pfn, page, order))
        out_tracking.write(str_f.format(pfn, pfn + (1 << order) - 1,
                            page, order, pid, "Comm: {} ".format(comm) if comm != -1 else "",
                            ts_nsec, gfp, function_list))

        if comm != -1:
            pid = "{}-{}".format(pid, comm)
        if function_list in sorted_pages:
            sorted_pages[function_list]["page_count"] = \
                sorted_pages[function_list]["page_count"] + 1
            sorted_pages[function_list]["memory"] += page_size * (1 << int(order))
            if pid in sorted_pages[function_list]:
                if order in sorted_pages[function_list][pid]:
                    sorted_pages[function_list][pid][order] = \
                        sorted_pages[function_list][pid][order] + 1
                else:
                    sorted_pages[function_list][pid][order] = 1
            else:
                sorted_pages[function_list][pid] = {}
                sorted_pages[function_list][pid][order] = 1
        else:
            sorted_pages[function_list] = {}
            sorted_pages[function_list]["page_count"] = 1
            sorted_pages[function_list]["memory"] = page_size * (1<<int(order))
            sorted_pages[function_list][pid] = {}
            sorted_pages[function_list][pid][order] = 1

    def parse(self):
        ranges = None
        if self.ramdump.minidump:
            for eb_file in self.ramdump.ebi_files:
                path1 = eb_file[3]
            path = os.path.join(os.path.dirname(path1), "md_PAGEOWNER.bin")
            if not os.path.exists(path):
                print_out_str(path + " not found")
                return
            input_file = open(path, 'r', errors="ignore")
            lines = input_file.readlines()
            i = 0
            functions = defaultdict(list)
            pfns = defaultdict(list)
            pfns_size = defaultdict(list)
            while i < len(lines):
                line = lines[i];
                try:
                    pfn, handle, n = [int(x) for x in line.split()]
                except:
                    break
                i = i + 1
                if not functions[handle]:
                    for j in range(0, n):
                        line = lines[i]
                        try:
                            int(line, 16)
                        except:
                            break
                        functions[handle].append(line)
                        i = i+1
                pfns[handle].append(pfn)

            i = 0
            for key in pfns:
                pfns_size[key] = len(pfns[key])
            output_file = self.ramdump.open_file("pageowner_dump.txt", 'w')
            for key, value in sorted(pfns_size.items(), key=lambda item: item[1], reverse = True):
                output_file.write("No of pfns :" + str(value))
                output_file.write('\n')
                output_file.write("Pfns :" + str(pfns[key]))
                output_file.write('\n')
                for key2 in functions:
                    if (key == key2):
                        output_file.write("call stack :\n")
                        for i in range(0,len(functions[key])):
                            look = self.ramdump.unwind_lookup(int(functions[key][i], 16))
                            if look is None:
                                continue
                            symname, offset = look
                            unwind_dat = '      [<{0:x}>] {1}+0x{2:x}\n'.format(
                                    int(functions[key][i], 16), symname, offset)
                            output_file.write(unwind_dat)
                output_file.write("\n")
            output_file.close()
            print_out_str(
                    '---wrote page tracking information to pageowner_dump.txt')
            return

        for arg in sys.argv:
            if "ranges=" in arg:
                g_optimization = True
                k, ranges = arg.split("=")
                start, end = ranges.split('-')
                start_pfn = int(start, 16) >> self.ramdump.page_shift
                end_pfn = int(end, 16) >> self.ramdump.page_shift
                break
            elif "page=" in arg:
                g_optimization = True
                k, page = arg.split('=')
                page = int(page, 16)
                start_pfn = page_to_pfn(self.ramdump, page)
                end_pfn = start_pfn + 1
                break
            else:
                g_optimization = False
        if not self.ramdump.is_config_defined('CONFIG_PAGE_OWNER'):
            print_out_str('CONFIG_PAGE_OWNER not defined')
            return

        if self.ramdump.kernel_version >= (4, 4):
            if self.ramdump.kernel_version >= (5, 10):
                page_owner_inited = self.ramdump.address_of("page_owner_inited.key.enabled.counter")
                if self.ramdump.read_int(page_owner_inited) != 1:
                    print_out_str("page_owner is not set in cmdline")
                    return
            else:
                if not self.ramdump.is_config_defined('CONFIG_PAGE_OWNER_ENABLE_DEFAULT'):
                    print_out_str('CONFIG_PAGE_OWNER_ENABLE_DEFAULT not defined')
                    return

        out_tracking = self.ramdump.open_file('page_tracking.txt')
        out_frequency = self.ramdump.open_file('page_frequency.txt')
        out_tracking_freed = self.ramdump.open_file('page_tracking_freed.txt')
        sorted_pages = {}
        page_size = self.ramdump.get_page_size() >> 10 #KB

        if g_optimization is True:
            for pfn in range(start_pfn, end_pfn):
                self.parse_output(pfn, out_tracking,
                                  out_tracking_freed, page_size, sorted_pages)
        else:
            for pfn in for_each_pfn(self.ramdump):
                self.parse_output(pfn, out_tracking,
                                  out_tracking_freed, page_size, sorted_pages)

        sortdict = {}
        for i in sorted_pages:
            sortdict[i] = sorted_pages[i]["memory"]
        sortdict = sorted(sortdict, key = lambda k: sortdict[k], reverse = True)
        try:
            for i in sortdict:
                out_frequency.write('Allocated {0} times, total memory: {1} KB\n'.
                    format(sorted_pages[i]["page_count"], sorted_pages[i]["memory"]))
                out_frequency.write(i)
                for j in sorted_pages[i]:
                    if str(j) != "page_count" and str(j) != "memory":
                        for order in sorted_pages[i][j]:
                            memory_order = page_size * (1<<int(order)) * sorted_pages[i][j][order]
                            out_frequency.write("pid:{0} order:{1} frequency:{2} memory:{3} KB\n"\
                                .format(j, order, sorted_pages[i][j][order], memory_order))
                out_frequency.write('\n')
        except Exception as e:
            print_out_str(str(e))

        out_tracking.close()
        out_frequency.close()
        out_tracking_freed.close()
        print_out_str(
            '---wrote allocated page tracking information to page_tracking.txt')
        print_out_str(
            '---wrote freed page tracking information to page_tracking_freed.txt')
        print_out_str(
            '---wrote page frequency information to page_frequency.txt')
