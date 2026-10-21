# Copyright (c) 2012-2021, The Linux Foundation. All rights reserved.
# Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import sys

from mm import page_address, pfn_to_page
from print_out import print_out_str
from parser_util import register_parser, RamParser
import operator
import os
from collections import defaultdict

SLAB_RED_ZONE = 0x400
SLAB_POISON = 0x800
SLAB_STORE_USER = 0x10000
OBJECT_POISON = 0x80000000

SLUB_RED_INACTIVE = 0xbb
SLUB_RED_ACTIVE = 0xcc
POISON_INUSE = 0x5a
POISON_FREE = 0x6b
POISON_END = 0xa5

FREQUENCY = 0
CALLSTACK_INDEX = 1
CALL_STACK = 2
TRACK_TYPE = 3
TRACK_PIDS = 4

# g_Optimization - if false will print :
# 1. Free objects callstacks
# 2. Object address range of allocated and FREE objects
g_Optimization = True

# When optimization is true, below two parameters will be used
# 1. MAX_NO_OF_CALLSTACK_TO_PRINT : Maximum number of callstacks to print
# in a slab.
# 2. CALLSTACK_FREQ_IN_SLAB: Print callstack only if occurance of
# each unique callstack in a slab is more than particular
# value(CALLSTACK_FREQ_IN_SLAB)

MAX_NO_OF_CALLSTACK_TO_PRINT = 5
MAX_NO_OF_PID_TO_PRINT = 5
CALLSTACK_FREQ_IN_SLAB = 10


class kmem_cache(object):
    def __init__(self, ramdump, addr):
        self.valid = False
        offset = ramdump.field_offset(
            'struct kmem_cache', 'flags')
        self.flags = ramdump.read_word(addr + offset)
        if self.flags is None:
            return
        offset = ramdump.field_offset(
            'struct kmem_cache', 'size')
        self.size = ramdump.read_int(addr + offset)
        if self.size is None:
            return
        offset = ramdump.field_offset(
            'struct kmem_cache', 'object_size')
        self.object_size = ramdump.read_int(addr + offset)
        if self.object_size is None:
            return
        offset = ramdump.field_offset(
            'struct kmem_cache', 'offset')
        self.offset = ramdump.read_int(addr + offset)
        if self.offset is None:
            return
        offset = ramdump.field_offset(
            'struct kmem_cache', 'inuse')
        self.inuse = ramdump.read_int(addr + offset)
        if self.inuse is None:
            return
        offset = ramdump.field_offset(
            'struct kmem_cache', 'red_left_pad')
        self.red_left_pad = ramdump.read_int(addr + offset)
        if self.red_left_pad is None:
            self.red_left_pad = 0
        self.addr = addr
        self.valid = True

        if ramdump.is_config_defined('CONFIG_SLAB_FREELIST_HARDENED'):
            self.random = ramdump.read_structure_field(
                                addr, 'struct kmem_cache', 'random')

class struct_member_offset(object):
    def __init__(self, ramdump):
        self.kmemcache_list = ramdump.field_offset(
            'struct kmem_cache', 'list')
        self.kmemcache_name = ramdump.field_offset(
            'struct kmem_cache', 'name')
        self.kmemcache_node = ramdump.field_offset(
            'struct kmem_cache', 'node')
        self.kmemcache_cpu_page = ramdump.field_offset(
            'struct kmem_cache_cpu', 'page')
        self.kmemcpucache_cpu_slab = ramdump.field_offset(
            'struct kmem_cache', 'cpu_slab')
        if ramdump.is_config_defined('CONFIG_SLUB_CPU_PARTIAL'):
            self.kmemcache_cpu_partial = ramdump.field_offset(
            'struct kmem_cache', 'cpu_partial')
        self.kmemcachenode_partial = ramdump.field_offset(
            'struct kmem_cache_node', 'partial')
        self.kmemcachenode_full = ramdump.field_offset(
                            'struct kmem_cache_node', 'full')
        self.page_next = ramdump.field_offset(
                            'struct page', 'next')
        self.page_lru = ramdump.field_offset(
                            'struct page', 'lru')
        self.page_flags = ramdump.field_offset(
                            'struct page', 'flags')

        self.page_objects = ramdump.field_offset(
                            'struct page', 'objects')
        self.track_cpu = ramdump.field_offset(
                            'struct track', 'cpu')
        self.track_when = ramdump.field_offset(
                            'struct track', 'when')
        self.track_addrs = ramdump.field_offset(
                            'struct track', 'addrs')
        self.track_pid = ramdump.field_offset(
                            'struct track', 'pid')
        self.page_freelist = ramdump.field_offset(
                            'struct page', 'freelist')
        self.sizeof_struct_track = ramdump.sizeof(
                                            'struct track')
        self.cpu_cache_page_offset = ramdump.field_offset(
                            'struct kmem_cache_cpu', 'page')
        self.cpu_partial_offset = ramdump.field_offset(
                            'struct kmem_cache_cpu', 'partial')
        self.sizeof_void_pointer = ramdump.sizeof(
                                             "void *")
        self.sizeof_unsignedlong = ramdump.sizeof(
                                             "unsigned long")


@register_parser('--slabinfo', 'print information about slabs', optional=True)
class Slabinfo(RamParser):
    g_allstacks = {}  # hold callstack stack
    g_index = 0
    g_offsetof = None

    def get_free_pointer(self, ramdump, s, obj):
        # just like validate_slab_slab!
        ptr = obj + s.offset
        val = self.ramdump.read_word(ptr)
        if self.ramdump.is_config_defined('CONFIG_SLAB_FREELIST_HARDENED'):
            return ptr ^ s.random ^ val
        else:
            return val

    def slab_index(self, ramdump, p, addr, slab):
        return (p - addr) // slab.size

    def get_map(self, ramdump, slab, page, bitarray):
        freelist = self.ramdump.read_word(page + g_offsetof.page_freelist)
        p = freelist
        addr = page_address(self.ramdump, page)
        seen = []
        if addr is None:
            return
        while p != 0 and p is not None and p not in seen:
            index = self.slab_index(self.ramdump, p, addr, slab)
            if index >= len(bitarray) or index < 0:
                return
            bitarray[index] = 1
            seen.append(p)
            p = self.get_free_pointer(self.ramdump, slab, p)

    def get_track(self, ramdump,  slab, obj, track_type):
        track_size = g_offsetof.sizeof_struct_track
        if slab.offset != 0:
            p = obj + slab.red_left_pad + slab.offset + g_offsetof.sizeof_void_pointer
        else:
            p = obj + slab.red_left_pad + slab.inuse
        return p + track_type * track_size

    def extract_callstack(self, ramdump, stack, out_file):
        for a in stack:
            look = ramdump.unwind_lookup(a)
            if look is None:
                out_file.write("Unknown symbol\n")
                continue
            symname, offset = look
            out_file.write(
                '      [<{0:x}>] {1}+0x{2:x}\n'.format(a, symname, offset))
        return

    def print_track(self, ramdump, slab, obj, track_type, out_file):
        stack = []
        stackstr = ""
        p = self.get_track(ramdump, slab, obj, track_type)
        track_addrs_offset = g_offsetof.track_addrs
        start = p + track_addrs_offset
        pointer_size = g_offsetof.sizeof_unsignedlong
        for i in range(0, 16):
            a = self.ramdump.read_word(start + pointer_size * i)
            if a == 0:
                continue
            stack += [a]
            stackstr += str(a)
        stackstr_len = len(stackstr)
        if stackstr_len == 0:
            return

        pid = self.ramdump.read_int(p + g_offsetof.track_pid)
        if pid is None:
            pid = -1

        try:
            self.g_allstacks[stackstr][FREQUENCY] += 1
            pids = self.g_allstacks[stackstr][TRACK_PIDS]
            if pid in pids:
                pids[pid] += 1
            else:
                pids[pid] = 1

            if self.g_allstacks[stackstr][FREQUENCY] > 1:
                return
        except KeyError:
            if g_Optimization is True:
                if track_type != 0:
                    # if free object and g_Optimization is True,
                    # ignore it for printing its call stack
                    return
            pids = dict()
            pids[pid] = 1
            self.g_allstacks[stackstr] = [1, self.g_index, stack, track_type, pids]
            self.g_index += 1

    def print_slab(
                self, ramdump, slab, page,
                out_file, map_fn, out_slabs_addrs):

        page_addr = page_address(ramdump, page)
        p = page_addr
        if page is None or page_addr is None:
            return
        n_objects = self.ramdump.read_word(page + g_offsetof.page_objects)
        if n_objects is None:
            return
        n_objects = (n_objects >> 16) & 0x00007FFF
        bitarray = [0] * n_objects
        addr = page_address(self.ramdump, page)
        self.get_map(self.ramdump, slab, page, bitarray)
        while p < page_addr + (n_objects * slab.size):
            bitidx = self.slab_index(self.ramdump, p, addr, slab)
            if bitidx >= n_objects or bitidx < 0:
                return
            map_fn(
                        ramdump, p, bitarray[bitidx], slab,
                        page, out_file, out_slabs_addrs)
            p = p + slab.size

    def printpidsummary(self, track_pids, slab_out):
        track_pids = sorted(
                            track_pids.items(),
                            key=operator.itemgetter(1), reverse=True)

        nr = min(len(track_pids), MAX_NO_OF_PID_TO_PRINT)
        for i in range(0, nr):
            slab_out.write(" pid:{0} frequency:{1}\n".format(
                        track_pids[i][0], track_pids[i][1]))

    def printsummary(self, slabs_output_summary, slab_out):
        nCounter = 0
        write_output = False
        str = " Call stack index:{0} frequency: {1}\n"
        sorted_val = sorted(
                            self.g_allstacks.items(),
                            key=operator.itemgetter(1), reverse=True)

        if g_Optimization is False:
            write_output = True

        for key, val in sorted_val:
            if g_Optimization is True:
                if nCounter >= MAX_NO_OF_CALLSTACK_TO_PRINT:
                    break
                if(
                    (nCounter < MAX_NO_OF_CALLSTACK_TO_PRINT)
                        and (val[FREQUENCY] > CALLSTACK_FREQ_IN_SLAB)):
                        write_output = True
                else:
                        write_output = False
            nCounter = nCounter + 1
            if write_output is True:
                if val[TRACK_TYPE] == 1:  # free object
                    slab_out.write(
                        "\nFREE" + str.format(
                                val[CALLSTACK_INDEX], val[FREQUENCY]))
                else:  # allocated object
                    slab_out.write(
                        "\nALLOCATED" + str.format(
                                val[CALLSTACK_INDEX], val[FREQUENCY]))
                self.extract_callstack(self.ramdump, val[CALL_STACK], slab_out)
                self.printpidsummary(val[TRACK_PIDS], slab_out)
            slabs_output_summary.write(
                " stack index:{0} frequency:{1}\n".format(
                        val[CALLSTACK_INDEX], val[FREQUENCY]))

    def print_slab_page_info(
                self, ramdump, slab_obj, slab_node, start,
                out_file, map_fn, out_slabs_addrs, cpu_partial=False):
        page = self.ramdump.read_word(start)
        if page == 0:
            return
        seen = []
        max_pfn_addr = self.ramdump.address_of('max_pfn')
        max_pfn = self.ramdump.read_word(max_pfn_addr)
        max_page = pfn_to_page(ramdump, max_pfn)
        while True:
            if page == start:
                return
            if page is None or page == 0:
                return
            if page in seen:
                return
            if page > max_page:
                return
            if cpu_partial is False:
                page = page - g_offsetof.page_lru
            seen.append(page)
            self.print_slab(
                self.ramdump, slab_obj, page, out_file, map_fn,
                out_slabs_addrs)
            if cpu_partial is False:
                page = self.ramdump.read_word(page + g_offsetof.page_lru)
            else:
                page = self.ramdump.read_word(page + g_offsetof.page_next)

    def print_per_cpu_slab_info(
            self, ramdump, slab, slab_node, start, out_file, map_fn,
            out_slabs_addrs):
        page = self.ramdump.read_word(start)
        if page == 0:
            return
        if page is None:
            return
        self.print_slab(
            self.ramdump, slab, page, out_file, map_fn, out_slabs_addrs)

    def print_objects_detail(self, ramdump, slab, obj, type):
        stack = []
        p = self.get_track(ramdump, slab, obj, type)
        track_addrs_offset = g_offsetof.track_addrs
        start = p + track_addrs_offset
        pointer_size = g_offsetof.sizeof_unsignedlong

        pid = self.ramdump.read_int(p + g_offsetof.track_pid)
        if pid is None:
            pid = -1
        when = self.ramdump.read_ulong(p + g_offsetof.track_when)
        if when is None:
            when = 0
        cpu = self.ramdump.read_int(p + g_offsetof.track_cpu)
        if cpu is None:
            cpu = -1
        print("object 0x%x cpu %d pid %d when %d" % (obj, cpu, pid, when), file=self.slabs_object_out)
        if type == 0:
            print("ALLOCATED", file=self.slabs_object_out)
        else:
            print("FREE", file=self.slabs_object_out)
        for i in range(0, 16):
            a = self.ramdump.read_word(start + pointer_size * i)
            if a == 0:
                continue
            stack += [a]

        for a in stack:
            look = ramdump.unwind_lookup(a)
            if look is None:
                self.slabs_object_out.write("Unknown symbol\n")
                continue
            symname, offset = look
            self.slabs_object_out.write(
                '      [<{0:x}>] {1}+0x{2:x}\n'.format(a, symname, offset))

    def print_all_objects(
        self, ramdump, p, free, slab, page,
            out_file, out_slabs_addrs):

        if g_Optimization is False:
            if free:
                out_slabs_addrs.write(
                    '\n   Object {0:x}-{1:x} FREE'.format(
                                        p, p + slab.size))
            else:
                out_slabs_addrs.write(
                    '\n   Object {0:x}-{1:x} ALLOCATED'.format(
                                    p, p + slab.size))
        if slab.flags & SLAB_STORE_USER:
            if g_Optimization is False:
                self.print_track(ramdump, slab, p, 0, out_file)
                self.print_track(ramdump, slab, p, 1, out_file)
                self.print_objects_detail(ramdump, slab, p, 0)
                self.print_objects_detail(ramdump, slab, p, 1)
            else:
                self.print_track(ramdump, slab, p, free, out_file)

    def print_check_poison(self, p, free, slab, page, out_file):
        if free:
            self.check_object(slab, page, p, SLUB_RED_INACTIVE, out_file)
        else:
            self.check_object(slab, page, p, SLUB_RED_ACTIVE, out_file)

    def initializeOffset(self):
        global g_offsetof
        g_offsetof = struct_member_offset(self.ramdump)

    # based on validate_slab_cache. Currently assuming there
    # is only one numa node in the system because the code to
    # do that correctly is a big pain. This will
    # need to be changed if we ever do NUMA properly.
    def validate_slab_cache(self, slab_out, input_slabname,  map_fn):
        slab_name_found = False
        original_slab = self.ramdump.address_of('slab_caches')
        cpus = self.ramdump.get_num_cpus()

        offsetof = struct_member_offset(self.ramdump)
        self.initializeOffset()
        slab_list_offset = g_offsetof.kmemcache_list
        slab_name_offset = g_offsetof.kmemcache_name
        slab_node_offset = g_offsetof.kmemcache_node
        cpu_slab_offset = g_offsetof.kmemcpucache_cpu_slab
        slab_partial_offset = g_offsetof.kmemcachenode_partial
        slab_full_offset = g_offsetof.kmemcachenode_full
        slab = self.ramdump.read_word(original_slab)
        slabs_output_summary = self.ramdump.open_file('slabs_output.txt')
        out_slabs_addrs = self.ramdump.open_file('out_slabs_addrs.txt')
        if g_Optimization is True:
            msg = (
                    "To optimize slab traverse time,"
                    "print of object address are skipped."
                    " Supply option perf_off in command"
                    "prompt to print object address as well."
                    )

            out_slabs_addrs.write(msg)

        while slab != original_slab:
            slab = slab - slab_list_offset
            slab_obj = kmem_cache(self.ramdump, slab)
            if not slab_obj.valid:
                slab_out.write(
                        'Invalid slab object {0:x}'.format(slab))
                slab = self.ramdump.read_word(slab + slab_list_offset)
                continue
            slab_name_addr = self.ramdump.read_word(
                                        slab + slab_name_offset)
            slab_name = self.ramdump.read_cstring(
                                        slab_name_addr, 48)
            if input_slabname is not None:
                if input_slabname != slab_name:
                    slab = self.ramdump.read_word(slab + slab_list_offset)
                    continue
                else:
                    slab_name_found = True
            # actually an array but again, no numa
            slab_node_addr = self.ramdump.read_word(
                                        slab + slab_node_offset)
            slab_node = self.ramdump.read_word(
                                        slab_node_addr)
            print_out_str(
                '\nExtracting slab details of : {0}'.format(
                                                                    slab_name))
            if g_Optimization is False:
                out_slabs_addrs.write(
                    '\nslab address of : {0}'.format(slab_name))
            cpu_slab_addr = self.ramdump.read_word(
                                        slab + cpu_slab_offset)

            if self.ramdump.is_config_defined('CONFIG_SLUB_DEBUG'):
                nr_total_objects = self.ramdump.read_structure_field(
                        slab_node_addr,
                        'struct kmem_cache_node', 'total_objects')
            else:
                nr_total_objects = 'NA'

            slab_out.write(
                '\n {0:x} slab {1} {2:x}  total objects: {3}\n'.format(
                        slab, slab_name, slab_node_addr, nr_total_objects))

            self.slabs_object_out.write(
                '\n {0:x} slab {1} {2:x}  total objects: {3}\n'.format(
                    slab, slab_name, slab_node_addr, nr_total_objects))

            self.print_slab_page_info(
                self.ramdump, slab_obj, slab_node,
                slab_node_addr + slab_partial_offset,
                slab_out, map_fn, out_slabs_addrs)

            if self.ramdump.is_config_defined('CONFIG_SLUB_DEBUG'):
                self.print_slab_page_info(
                    self.ramdump, slab_obj, slab_node,
                    slab_node_addr + slab_full_offset,
                    slab_out, map_fn, out_slabs_addrs)

            # per cpu slab
            for i in range(0, cpus):
                cpu_slabn_addr = cpu_slab_addr + self.ramdump.per_cpu_offset(i)
                if cpu_slabn_addr == 0 or None:
                    break

                self.print_per_cpu_slab_info(
                    self.ramdump, slab_obj,
                    slab_node, cpu_slabn_addr + offsetof.cpu_cache_page_offset,
                    slab_out, map_fn, out_slabs_addrs)
                if self.ramdump.is_config_defined('CONFIG_SLUB_CPU_PARTIAL'):
                    cpu_partial_val = self.ramdump.read_int(
                                        slab + offsetof.kmemcache_cpu_partial)
                if self.ramdump.is_config_defined('CONFIG_SLUB_CPU_PARTIAL') and cpu_partial_val != 0:
                    self.print_slab_page_info(self.ramdump, slab_obj, slab_node,
                            cpu_slabn_addr + offsetof.cpu_partial_offset,
                            slab_out, map_fn, out_slabs_addrs, True)

            self.printsummary(slabs_output_summary, slab_out)
            self.g_allstacks.clear()
            if slab_name_found is True:
                break
            slab = self.ramdump.read_word(slab + slab_list_offset)
        out_slabs_addrs.close()
        slabs_output_summary.close()

    def parse(self):
        if self.ramdump.minidump:
            for eb_file in self.ramdump.ebi_files:
                path1 = eb_file[3]
            path = os.path.join(os.path.dirname(path1), "md_SLABOWNER.bin")
            if not os.path.exists(path):
                print_out_str(path + " not found")
                return
            input_file = open(path, 'r', errors="ignore")
            output_file = self.ramdump.open_file("slabowner_dump.txt", 'w')
            lines = input_file.readlines()
            i = 0
            while i < len(lines):
                functions = defaultdict(list)
                objs = defaultdict(list)
                objs_size = defaultdict(list)
                line = lines[i];
                output_file.write(line)
                i = i+1
                while (i < len(lines)):
                    if ('kmalloc-' not in lines[i]):
                        line = lines[i]
                        try:
                            txt = line.split()
                            obj =  txt[0]
                            handle = int(txt[1])
                            n = int(txt[2])
                            objs[handle].append(obj)
                        except:
                            i = i + 1
                            continue
                        i = i+1
                        j = 0
                        if not functions[handle]:
                            for j in range(0, n):
                                line = lines[i]
                                try:
                                    int(line, 16)                                    
                                except:
                                    i = i+1
                                    continue
                                functions[handle].append(line)
                                i = i+1                        
                    else:
                        break
                j = 0
                for key in objs:
                    objs_size[key] = len(objs[key])
                for key, value in sorted(objs_size.items(), key=lambda item: item[1], reverse = True):
                    output_file.write("No of objects :" + str(value))
                    output_file.write('\n')
                    output_file.write("Objs :" + str(objs[key]))
                    output_file.write('\n')
                    for key2 in functions:
                        if (key == key2):
                            output_file.write("call stack :\n")
                            for j in range(0,len(functions[key])):
                                look = self.ramdump.unwind_lookup(int(functions[key][j], 16))
                                if look is None:
                                    continue
                                symname, offset = look
                                unwind_dat = '      [<{0:x}>] {1}+0x{2:x}\n'.format(
                                int(functions[key][j], 16), symname, offset)
                                output_file.write(unwind_dat)
                    output_file.write("\n")
            output_file.close()
            return
        global g_Optimization
        slabname = None
        for arg in sys.argv:
            if 'slabname=' in arg:
                k, slabname = arg.split('=')
            if 'perf_off' in arg:
                g_Optimization = False
        slab_out = self.ramdump.open_file('slabs.txt')
        self.slabs_object_out = self.ramdump.open_file('slabs_object.txt')
        self.validate_slab_cache(slab_out, slabname, self.print_all_objects)
        self.slabs_object_out.close()
        slab_out.close()


@register_parser('--slabpoison', 'check slab poison', optional=True)
class Slabpoison(Slabinfo):
    """Note that this will NOT find any slab errors which are printed out by the
    kernel, because the slab object is removed from the freelist while being
    processed"""

    def print_section(self, text, addr, length, out_file):
        out_file.write('{}\n'.format(text))
        output = self.ramdump.hexdump(addr, length)
        out_file.write(output)

    def print_trailer(self, s, page, p, out_file):
        addr = page_address(self.ramdump, page)

        if self.ramdump.is_config_defined('CONFIG_SLUB_DEBUG_ON'):
            self.print_track(self.ramdump, s.addr, p, 0, out_file)
            self.print_track(self.ramdump, s.addr, p, 1, out_file)

        out_file.write('INFO: Object 0x{:x} @offset=0x{:x} fp=0x{:x}\n\n'.format(
            p, p - addr, self.get_free_pointer(self.ramdump, s, p)))

        if (p > addr + 16):
            self.print_section('Bytes b4 ', p - 16, 16, out_file)

        self.print_section('Object ', p, min(s.object_size, 4096), out_file)
        if (s.flags & SLAB_RED_ZONE):
            self.print_section('Redzone ', p + s.object_size,
                s.inuse - s.object_size, out_file)

        if (s.offset):
            off = s.offset + self.ramdump.sizeof('void *')
        else:
            off = s.inuse

        if (s.flags & SLAB_STORE_USER):
            off += 2 * self.ramdump.sizeof('struct track')

        if (off != s.size):
            # Beginning of the filler is the free pointer
            self.print_section('Padding ', p + off, s.size - off, out_file)

    def memchr_inv(self, addr, value, size):
        data = self.read_byte_array(addr, size)
        if data is None:
            return 0
        for i in range(len(data)):
            if data[i] != value:
                return i + addr
        return 0

    def check_bytes_and_report(self, s, page, object, what, start,
             value, bytes, out_file):
        fault = self.memchr_inv(start, value, bytes)
        if (not fault):
            return True

        end = start + bytes
        while (end > fault and (self.read_byte_array(end - 1, 1)[0]) == value):
            end -= 1

        out_file.write('{0} overwritten\n'.format(what))
        out_file.write('INFO: 0x{:x}-0x{:x}. First byte 0x{:x} instead of 0x{:x}\n'.format(
            fault, end - 1, self.ramdump.read_byte(fault), value))

        self.print_trailer(s, page, object, out_file)
        return False

    def check_pad_bytes(self, s, page, p, out_file):
        off = s.inuse

        if (s.offset):
            # Freepointer is placed after the object
            off += self.ramdump.sizeof('void *')

        if (s.flags & SLAB_STORE_USER):
            # We also have user information there
            off += 2 * self.ramdump.sizeof('struct track')

        if (s.size == off):
            return True

        return self.check_bytes_and_report(s, page, p, 'Object padding',
            p + off, POISON_INUSE, s.size - off, out_file)

    def check_object(self, s, page, object, val, out_file):
        p = object
        endobject = object + s.object_size

        if (s.flags & SLAB_RED_ZONE):
            if (not self.check_bytes_and_report(s, page, object, 'Redzone',
                endobject, val, s.inuse - s.object_size, out_file)):
                return
        else:
            if ((s.flags & SLAB_POISON) and s.object_size < s.inuse):
                self.check_bytes_and_report(s, page, p, 'Alignment padding',
                    endobject, POISON_INUSE,
                    s.inuse - s.object_size, out_file)

        if (s.flags & SLAB_POISON):
            if (val != SLUB_RED_ACTIVE and (s.flags & OBJECT_POISON) and
                (not self.check_bytes_and_report(s, page, p, 'Poison', p,
                    POISON_FREE, s.object_size - 1, out_file) or
                not self.check_bytes_and_report(s, page, p, 'Poison',
                    p + s.object_size - 1, POISON_END, 1, out_file))):
                return

            # check_pad_bytes cleans up on its own.
            self.check_pad_bytes(s, page, p, out_file)

    # since slabs are relatively "packed", caching has a large
    # performance benefit
    def read_byte_array(self, addr, size):
        page_addr = addr & -0x1000
        end_page_addr = (addr + size) & -0x1000
        # in cache
        if page_addr == end_page_addr and page_addr == self.cache_addr:
            idx = addr - self.cache_addr
            return self.cache[idx:idx + size]
        # accessing only one page
        elif page_addr == end_page_addr:
            fmtstr = '<{}B'.format(4096)
            self.cache = self.ramdump.read_string(page_addr, fmtstr)
            self.cache_addr = page_addr
            idx = addr - self.cache_addr
            return self.cache[idx:idx + size]
        else:
            fmtstr = '<{}B'.format(size)
            return self.ramdump.read_string(addr, fmtstr)

    def parse(self):
        self.cache = None
        self.cache_addr = None
        slab_out = self.ramdump.open_file('slabpoison.txt')
        self.validate_slab_cache(slab_out, None, self.print_check_poison)
        print_out_str('---wrote slab information to slabpoison.txt')
