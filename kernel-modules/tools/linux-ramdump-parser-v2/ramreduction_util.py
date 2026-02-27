# Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: GPL-2.0-only

import sys
import re
import os
from print_out import print_out_str
from importlib import import_module
from bisect import bisect_left as lower_bound
from array import array

mod = import_module('elftools.elf.elffile')
ELFFile = mod.ELFFile

cachemap = {}
cachehits = 0
cachemiss = 0
cache = [None]

def iter_segments(elffile):
    """ Yield all the segments in the file
    """
    for i in range(elffile.header['e_phnum']):
        yield elffile.get_segment(i)

def setup_elfmappings(elffiles):
    # Elfiles are present in options.elf_addr
    vector = []
    htable = {}
    filemap = {}

    # Parse the elf and set up segment mappings
    for file in elffiles:
        _fd = open(file, 'rb')
        elf_file = ELFFile(_fd)
        nsegments = elf_file.num_segments()
        # First segment is a note segment, ignore
        start = elf_file.get_segment(1)
        start = int(start['p_paddr'])
        end = elf_file.get_segment(nsegments-1)
        end = int(end['p_paddr']) + int(end['p_filesz']) - 1
        tempbuf = array('B')
        _fd.seek(0)
        try:
            tempbuf.fromfile(_fd, os.stat(file).st_size)
            #tempbuf.fromfile(_fd, 0x200000000)
            #0x400000000 == 16 GB
        except EOFError:
            # We have read the entire data dump from file
            # This is to avoid the need to know size to read
            # So we read as far as we can
            pass

        tempbuf = tempbuf.tobytes()
        filemap[file] = [start, end, elf_file, tempbuf]

        for idx, s in enumerate(elf_file.iter_segments()):
            start = int(s['p_paddr'])
            size = int(s['p_filesz'])
            end = start + size
            vector.extend([start, end])
            htable[start] = [1, idx]
            htable[end] = [2, idx]
    return vector, htable, filemap


# With growing cachemap
def read_physical(vector, htable, filemap, ebi_files, addr, length):
    global cachemap, cachehits, cachemiss
    # Check in OCIMEM first
    ebi = (-1, -1, -1)
    for a in ebi_files:
        fd, start, end, path = a
        #print("Start : {:x}, end : {:x}".format(start, end))
        if addr >= start and addr <= end:
            ebi = a
            break
    if ebi[0] != -1:
        offset = addr - ebi[1]
        ebi[0].seek(offset)
        a = ebi[0].read(length)
        return a

    hashkey = addr>>12
    if cachemap.get(hashkey):
        cachehits += 1
        offset = addr - cachemap[hashkey][0] + cachemap[hashkey][1]
        return cachemap[hashkey][2][offset:offset+length]
    else:
        cachemiss += 1
        # Check in the ELF files
        # get the number same or greater than addr
        ind = lower_bound(vector, addr)
        segment_index = None
        try:
            if vector[ind] == addr or htable[vector[ind]][0] == 2:
                segment_index = htable[vector[ind]][1]
        except IndexError:
            raise IndexError

        #else:
        #    print("Invalid physical address !!!!")
        #    sys.exit(1)

        #if addr == 0x83cc09268:
        #    print("addr read 0x83cc09268 : {}, {}, {}, {}".format(ind, segment_index, vector[ind], vector[ind+1]))
        #    print(htable[vector[ind]])

        if segment_index is not None:
            for fname, (start, end, elffile, _fd) in filemap.items():
                if addr >= start and addr <= end:
                    segment = elffile.get_segment(segment_index)
                    start = int(segment['p_offset'])
                    offset = start + addr - int(segment['p_paddr'])
                    # Save to the cachemap
                    cachemap[addr>>12] = [addr, offset, _fd]
                    #buff = array('B')
                    ##_fd.seek(offset)
                    #buff.fromfile(_fd, length)
                    #return buff.tobytes()
                    ##return _fd.read(length)
                    return _fd[offset:offset+length]

    #print("Address {} not found in the elf data dump", hex(addr))
    #sys.exit(1)


# Without cachemap
def read_physical(vector, htable, filemap, ebi_files, addr, length):
    # Check in BIN's first
    ebi = (-1, -1, -1)
    for a in ebi_files:
        fd, start, end, path = a
        #print("Start : {:x}, end : {:x}".format(start, end))
        if addr >= start and addr <= end:
            ebi = a
            break
    if ebi[0] != -1:
        offset = addr - ebi[1]
        ebi[0].seek(offset)
        a = ebi[0].read(length)
        return a

    # Check in the ELF files
    # get the number same or greater than addr
    ind = lower_bound(vector, addr)
    segment_index = None
    p_offset = None

    if vector[ind] == addr or htable[vector[ind]][0] == 2:
        segment_index = htable[vector[ind]][1]

    if segment_index is not None:
        for fname, (start, end, elffile, _fd) in filemap.items():
            if addr >= start and addr <= end:
                segment = elffile.get_segment(segment_index)
                start = int(segment['p_offset'])
                offset = start + addr - int(segment['p_paddr'])
                return _fd[offset:offset+length]

    #print("Address {} not found in the elf data dump", hex(addr))
    return None


 # Supports getting offsets for parse debug image
def get_read_physical_offset_helper(vector, htable, filemap, ebi_files, addr):
    offset = None
    input = None
    for file in ebi_files:
        fd, start, end, path = file
        if addr >= start and addr <= end:
            input = path
            offset = addr - start
            break
    if offset is None:
        # Check in the ELF files
        # get the number same or greater than addr
        ind = lower_bound(vector, addr)
        segment_index = None
        if vector[ind] == addr or htable[vector[ind]][0] == 2:
            segment_index = htable[vector[ind]][1]

        if segment_index is not None:
            for fname, (start, end, elffile, _fd) in filemap.items():
                if addr >= start and addr <= end:
                    input = os.path.join(os.path.dirname(ebi_files[0][-1]), fname)
                    segment = elffile.get_segment(segment_index)
                    start = int(segment['p_offset'])
                    offset = start + addr - int(segment['p_paddr'])
                    break

    print_out_str("get_read_physical_offset{:x} : offset : {:x}, input : {}".format(addr, offset, input))
    return offset, input
