# Copyright (c) 2012-2021, The Linux Foundation. All rights reserved.
# Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from __future__ import print_function
import sys
import re
import os
import struct
import gzip
import functools
import string
import random
import platform
import stat
import subprocess
import enum
import copy

from boards import get_supported_boards, get_supported_ids
from tempfile import NamedTemporaryFile

import gdbmi
from print_out import print_out_str
from mmu import Armv7MMU, Armv7LPAEMMU, Armv8MMU
import parser_util
import minidump_util
import ramreduction_util as elfutil
from importlib import import_module
import module_table
from mm import mm_init
from register import Register

SP = 13
LR = 14
PC = 15

# The smem code is very stable and unlikely to go away or be changed.
# Rather than go through the hassel of parsing the id through gdb,
# just hard code it

SMEM_HW_SW_BUILD_ID = 0x89
SMEM_PRIVATE_CANARY = 0xa5a5
PARTITION_MAGIC = 0x54525024
BUILD_ID_LENGTH = 32

primary_types = ["int", "unsigned", "unsigned int", "signed", "signed int",
                 "char", "signed char", "unsigned char", "long unsigned int", "long signed int",
                 "short", "short int", "unsigned short", "unsigned short int", "signed short", "signed short int",
                 "long", "long int", "unsigned long", "signed long", "unsigned long int", "signed long int",
                 "long long", "long long int", "signed long long", "signed long long int", "unsigned long long",
                 "long long unsigned int", "unsigned long long int", "float", "double", "long double",
                 "u8", "u16", "u32", "u64", "bool", "uint32_t",
                 "s8", "s16", "s32", "s64", "uint8_t", "uint16_t", "uint64_t", "__u32", "size_t", "_Bool", "boolean"
                 ]
storage_classes = ["static", "volatile", "extern", "register", "auto", "const"]

class InvalidDatatype(Exception):
    """
    This exception will be raised when a vaiable of invalid
    datatype is passed as an argument to a function.
    """
    pass

class InvalidInput(Exception):
    """
    This exception will be raised when a vaiable of invalid
    value is passed as an argument to a function.
    """
    pass

class SymbolNotFound(Exception):
    """
    This exception will be raised when an invalid symbol
    is passed as an argument to a function.
    """
    pass

def is_ramdump_file(val, minidump):
    if not minidump:
        ddr = re.compile(r'(DDR|EBI|VMDDR)[0-9_CS]+[.]BIN', re.IGNORECASE)
        imem = re.compile(r'.*IMEM.BIN', re.IGNORECASE)
        if ddr.match(val) or imem.match(val) and not ("md_" in val):
            return True
    else:
        ddr = re.compile(r'(md_)[0-9_A-Z]+[.]BIN', re.IGNORECASE)
        if ddr.match(val):
            return True
    return False

def is_reduceddump_file(val, is_vm):
    hlos = re.compile(r'MR_HLOS.*[.]ELF', re.IGNORECASE)
    smem = re.compile(r'MR_smem.*[.]bin', re.IGNORECASE)
    imem = re.compile(r'.*IMEM.BIN', re.IGNORECASE)
    memdump = re.compile(r'MR_(mem_dump|linux,cma).*[.]bin', re.IGNORECASE)
    hyp = re.compile(r'MR_(hyp|hyp_region).*[.]bin', re.IGNORECASE)

    if hyp.match(val) or smem.match(val) or hlos.match(val) or imem.match(val) or memdump.match(val):
        return True

    if is_vm:
        vm = re.compile(r'.*MR_.*(trustedvm).*[.]bin', re.IGNORECASE)
        hyp_carve = hyp = re.compile(r'MR_hyp_(smmu|trace)_carveout.*[.]bin', re.IGNORECASE)
        if vm.match(val) or hyp_carve.match(val):
            return True
    return False


class AutoDumpInfo(object):
    priority = 0
    def __init__(self, autodumpdir, minidump, reduceddump, svm=None):
        self.autodumpdir = autodumpdir
        self.minidump = minidump
        self.reduceddump = reduceddump
        self.svm = svm
        self.ebi_files = []
        self.elf_files = []

    def parse(self):
        for (filename, base_addr) in self._parse():
            fullpath = os.path.join(self.autodumpdir, filename)
            if not os.path.exists(fullpath):
                continue
            if self.reduceddump and filename.lower().endswith(".elf"):
                self.elf_files.append(fullpath)
            else:
                end = base_addr + os.path.getsize(fullpath) - 1
                self.ebi_files.append((open(fullpath, 'rb'), base_addr, end, fullpath))
                # sort by addr, DDR files first. The goal is for
                # self.ebi_files[0] to be the DDR file with the lowest address.
                self.ebi_files.sort(key=lambda x: (x[1]))

    def _parse(self):
        # Implementations should return an interable of (filename, base_addr)
        raise NotImplementedError


class AutoDumpInfoCMM(AutoDumpInfo):
    # Parses CMM scripts (like load.cmm)
    def _parse(self):
        filename = 'load.cmm'
        if not os.path.exists(os.path.join(self.autodumpdir, filename)):
            print_out_str('!!! AutoParse could not find load.cmm!')
            return

        with open(os.path.join(self.autodumpdir, filename)) as f:
            for line in f.readlines():
                words = line.split()
                if len(words) == 4 and is_ramdump_file(words[1], self.minidump):
                    fname = words[1]
                    start = int(words[2], 16)
                    yield fname, start


class AutoDumpInfoDumpInfoTXT(AutoDumpInfo):
    # Parses dump_info.txt
    priority = 1

    def _parse(self):
        filename = 'dump_info.txt'
        if not os.path.exists(os.path.join(self.autodumpdir, filename)):
            print_out_str('!!! AutoParse could not find dump_info.txt!')
            return

        with open(os.path.join(self.autodumpdir, filename)) as f:
            try:
                for line in f.readlines():
                    words = line.split()
                    if not words or not is_ramdump_file(words[-1],
                                                        self.minidump):
                        continue
                    fname = words[-1]
                    start = int(words[1], 16)
                    size = int(words[2])
                    filesize = os.path.getsize(
                        os.path.join(self.autodumpdir, fname))
                    if size != filesize:
                        print_out_str(
                            ("!!! Size of %s on disk (%d) doesn't match size " +
                             "from dump_info.txt (%d). Skipping...")
                            % (fname, filesize, size))
                        continue
                    yield fname, start
            except:
                print_out_str('!!! Cannot parse dump_info.txt due to improper format!')
                return

class AutoDumpInfoReducedDump(AutoDumpInfo):
    # Parses binoffsets.txt, dump_info.txt
    # Finds HLOS elf file
    priority = 1
    def _parse(self):
        filename = 'binoffsets.txt'
        added = []
        if not os.path.exists(os.path.join(self.autodumpdir, filename)):
            print_out_str('!!! AutoParse could not find {}!'.format(filename))
            return

        # Find smem, hyp, memdump file
        with open(os.path.join(self.autodumpdir, filename)) as f:
            for line in f.readlines():
                words = line.split()
                if not words or not is_reduceddump_file(words[0], self.svm):
                    continue
                fname = words[0].strip()
                start = int(words[1], 16)
                size = int(words[2], 16)
                added.append(fname)
                yield fname, start

        # Find HLOS elf's
        hlos = re.compile(r'.*MR_HLOS.*[.]ELF', re.IGNORECASE)
        for file in os.listdir(self.autodumpdir):
            if hlos.match(file):
                yield file, None

        # Find *IMEM bins
        filename = 'load.cmm'
        if not os.path.exists(os.path.join(self.autodumpdir, filename)):
            print_out_str('!!! AutoParse could not find load.cmm!')
            return

        with open(os.path.join(self.autodumpdir, filename)) as f:
            for line in f.readlines():
                words = line.split()
                if len(words) != 4 or not is_reduceddump_file(words[1], self.svm):
                    continue
                fname = words[1]
                if fname in added:
                    continue
                start = int(words[2], 16)
                yield fname, start

class AutoDumpInfodram_cs(AutoDumpInfo):
    # Parses dump_info.txt
    priority = 2

    def _parse(self):
        if not os.path.exists(self.autodumpdir):
            print_out_str('!!! AutoParse could not find path {}!'.format(self.autodumpdir))
            return

        filename_lst = os.listdir(self.autodumpdir)
        regex = re.compile(r'^(dram_cs|ocimem)\S*(0x[A-Fa-f0-9]+)\S+(0x[A-Fa-f0-9]+)')

        for filename in filename_lst:
            m = re.search(regex, filename)
            if m:
                start = int(m.group(2), 16)
                end = int(m.group(3), 16)

                filesize = os.path.getsize(
                    os.path.join(self.autodumpdir, filename))
                if end - start + 1 != filesize:
                    print_out_str(
                        ("!!! Size of %s on disk (%d) doesn't match size " +
                         "from dump_info.txt (%d). Skipping...")
                        % (filename, filesize, end - start + 1))
                    continue
                yield filename, start

class RamDump():
    """The main interface to the RAM dump"""

    class Unwinder ():

        class Stackframe ():

            def __init__(self, fp, sp, lr, pc):
                self.fp = fp
                self.sp = sp
                self.lr = lr
                self.pc = pc

        class UnwindCtrlBlock ():

            def __init__(self):
                self.vrs = 16 * [0]
                self.insn = 0
                self.entries = -1
                self.byte = -1
                self.index = 0

        def __init__(self, ramdump):
            start = ramdump.address_of('__start_unwind_idx')
            end = ramdump.address_of('__stop_unwind_idx')
            self.ramdump = ramdump
            if (start is None) or (end is None):
                if ramdump.arm64:
                    self.unwind_frame = self.unwind_frame_generic64
                else:
                    self.unwind_frame = self.unwind_frame_generic
                return None
            # addresses
            self.unwind_frame = self.unwind_frame_tables
            self.start_idx = start
            self.stop_idx = end
            self.unwind_table = []
            i = 0
            for addr in range(start, end, 8):
                r = ramdump.read_string(addr, '<II')
                if r is None:
                    break
                (a, b) = r
                self.unwind_table.append((a, b, start + 8 * i))
                i += 1

            ver = ramdump.get_kernel_version()
            if (ver[0] == 3 and ver[1] == 0):
                self.search_idx = self.search_idx_3_0
            else:
                self.search_idx = self.search_idx_3_4
                # index into the table
                self.origin = self.unwind_find_origin()

        def unwind_find_origin(self):
            start = 0
            stop = len(self.unwind_table)
            while (start < stop):
                mid = start + ((stop - start) >> 1)
                if (self.unwind_table[mid][0] >= 0x40000000):
                    start = mid + 1
                else:
                    stop = mid
            return stop

        def unwind_frame_generic64(self, frame, cpu_work_state=''):
            fp = frame.fp
            try:
                frame.sp = fp + 0x10
                frame.fp = self.ramdump.read_word(fp)
                frame.pc = self.ramdump.read_word(fp + 8)
                if ((frame.fp == 0 and frame.pc == 0)
                        or frame.pc is None or frame.lr is None):
                    return -1
            except:
                return -1
            return 0

        def unwind_frame_generic(self, frame, cpu_work_state=''):
            high = 0
            fp = frame.fp

            low = frame.sp
            mask = (self.ramdump.thread_size) - 1

            high = (low + mask) & (~mask)  # ALIGN(low, THREAD_SIZE)

            # /* check current frame pointer is within bounds */
            if (fp < (low + 12) or fp + 4 >= high):
                return -1

            fp_is_at = self.ramdump.read_word(frame.fp - 12)
            sp_is_at = self.ramdump.read_word(frame.fp - 8)
            pc_is_at = self.ramdump.read_word(frame.fp - 4)

            frame.fp = fp_is_at
            frame.sp = sp_is_at
            frame.pc = pc_is_at

            return 0

        def walk_stackframe_generic(self, frame):
            while True:
                symname = self.ramdump.addr_to_symbol(frame.pc)
                print_out_str(symname)

                ret = self.unwind_frame_generic(frame)
                if ret < 0:
                    break

        def unwind_backtrace_generic(self, sp, fp, pc):
            frame = self.Stackframe()
            frame.fp = fp
            frame.pc = pc
            frame.sp = sp
            walk_stackframe_generic(frame)

        def search_idx_3_4(self, addr):
            start = 0
            stop = len(self.unwind_table)
            orig = addr

            if (addr < self.start_idx):
                stop = self.origin
            else:
                start = self.origin

            if (start >= stop):
                return None

            addr = (addr - self.unwind_table[start][2]) & 0x7fffffff

            while (start < (stop - 1)):
                mid = start + ((stop - start) >> 1)

                dif = (self.unwind_table[mid][2]
                       - self.unwind_table[start][2])
                if ((addr - dif) < self.unwind_table[mid][0]):
                    stop = mid
                else:
                    addr = addr - dif
                    start = mid

            if self.unwind_table[start][0] <= addr:
                return self.unwind_table[start]
            else:
                return None

        def search_idx_3_0(self, addr):
            first = 0
            last = len(self.unwind_table)
            while (first < last - 1):
                mid = first + ((last - first + 1) >> 1)
                if (addr < self.unwind_table[mid][0]):
                    last = mid
                else:
                    first = mid

            return self.unwind_table[first]

        def unwind_get_byte(self, ctrl):

            if (ctrl.entries <= 0):
                print_out_str('unwind: Corrupt unwind table')
                return 0

            val = self.ramdump.read_word(ctrl.insn)

            ret = (val >> (ctrl.byte * 8)) & 0xff

            if (ctrl.byte == 0):
                ctrl.insn += 4
                ctrl.entries -= 1
                ctrl.byte = 3
            else:
                ctrl.byte -= 1

            return ret

        def unwind_exec_insn(self, ctrl):
            insn = self.unwind_get_byte(ctrl)

            if ((insn & 0xc0) == 0x00):
                ctrl.vrs[SP] += ((insn & 0x3f) << 2) + 4
            elif ((insn & 0xc0) == 0x40):
                ctrl.vrs[SP] -= ((insn & 0x3f) << 2) + 4
            elif ((insn & 0xf0) == 0x80):
                vsp = ctrl.vrs[SP]
                reg = 4

                insn = (insn << 8) | self.unwind_get_byte(ctrl)
                mask = insn & 0x0fff
                if (mask == 0):
                    print_out_str("unwind: 'Refuse to unwind' instruction")
                    return -1

                # pop R4-R15 according to mask */
                load_sp = mask & (1 << (13 - 4))
                while (mask):
                    if (mask & 1):
                        ctrl.vrs[reg] = self.ramdump.read_word(vsp)
                        if ctrl.vrs[reg] is None:
                            return -1
                        vsp += 4
                    mask >>= 1
                    reg += 1
                if not load_sp:
                    ctrl.vrs[SP] = vsp

            elif ((insn & 0xf0) == 0x90 and (insn & 0x0d) != 0x0d):
                ctrl.vrs[SP] = ctrl.vrs[insn & 0x0f]
            elif ((insn & 0xf0) == 0xa0):
                vsp = ctrl.vrs[SP]
                a = list(range(4, 4 + (insn & 7)))
                a.append(4 + (insn & 7))
                # pop R4-R[4+bbb] */
                for reg in (a):
                    ctrl.vrs[reg] = self.ramdump.read_word(vsp)
                    if ctrl.vrs[reg] is None:
                        return -1
                    vsp += 4
                if (insn & 0x80):
                    ctrl.vrs[14] = self.ramdump.read_word(vsp)
                    if ctrl.vrs[14] is None:
                        return -1
                    vsp += 4
                ctrl.vrs[SP] = vsp
            elif (insn == 0xb0):
                if (ctrl.vrs[PC] == 0):
                    ctrl.vrs[PC] = ctrl.vrs[LR]
                ctrl.entries = 0
            elif (insn == 0xb1):
                mask = self.unwind_get_byte(ctrl)
                vsp = ctrl.vrs[SP]
                reg = 0

                if (mask == 0 or mask & 0xf0):
                    print_out_str('unwind: Spare encoding')
                    return -1

                # pop R0-R3 according to mask
                while mask:
                    if (mask & 1):
                        ctrl.vrs[reg] = self.ramdump.read_word(vsp)
                        if ctrl.vrs[reg] is None:
                            return -1
                        vsp += 4
                    mask >>= 1
                    reg += 1
                ctrl.vrs[SP] = vsp
            elif (insn == 0xb2):
                uleb128 = self.unwind_get_byte(ctrl)
                ctrl.vrs[SP] += 0x204 + (uleb128 << 2)
            else:
                print_out_str('unwind: Unhandled instruction')
                return -1

            return 0

        def prel31_to_addr(self, addr):
            value = self.ramdump.read_word(addr)
            # offset = (value << 1) >> 1
            # C wants this sign extended. Python doesn't do that.
            # Sign extend manually.
            if (value & 0x40000000):
                offset = value | 0x80000000
            else:
                offset = value

            # This addition relies on integer overflow
            # Emulate this behavior
            temp = addr + offset
            return (temp & 0xffffffff) + ((temp >> 32) & 0xffffffff)

        def unwind_frame_tables(self, frame, cpu_work_state):
            low = frame.sp
            high = ((low + (self.ramdump.thread_size - 1)) & \
                ~(self.ramdump.thread_size - 1)) + self.ramdump.thread_size
            idx = self.search_idx(frame.pc)

            if (idx is None):
                return -1

            if cpu_work_state == "thumb":
                FP = 7
            else:
                FP = 11

            ctrl = self.UnwindCtrlBlock()
            ctrl.vrs[FP] = frame.fp
            ctrl.vrs[SP] = frame.sp
            ctrl.vrs[LR] = frame.lr
            ctrl.vrs[PC] = 0

            if (idx[1] == 1):
                return -1

            elif ((idx[1] & 0x80000000) == 0):
                ctrl.insn = self.prel31_to_addr(idx[2] + 4)

            elif (idx[1] & 0xff000000) == 0x80000000:
                ctrl.insn = idx[2] + 4
            else:
                print_out_str('not supported')
                return -1

            val = self.ramdump.read_word(ctrl.insn)

            if ((val & 0xff000000) == 0x80000000):
                ctrl.byte = 2
                ctrl.entries = 1
            elif ((val & 0xff000000) == 0x81000000):
                ctrl.byte = 1
                ctrl.entries = 1 + ((val & 0x00ff0000) >> 16)
            else:
                return -1

            while (ctrl.entries > 0):
                urc = self.unwind_exec_insn(ctrl)
                if (urc < 0):
                    return urc
                if (ctrl.vrs[SP] < low or ctrl.vrs[SP] >= high):
                    return -1

            if (ctrl.vrs[PC] == 0):
                ctrl.vrs[PC] = ctrl.vrs[LR]

            # check for infinite loop */
            if (frame.pc == ctrl.vrs[PC]):
                return -1

            frame.fp = ctrl.vrs[FP]
            frame.sp = ctrl.vrs[SP]
            frame.lr = ctrl.vrs[LR]
            frame.pc = ctrl.vrs[PC]

            return 0

        def pac_frame_update(self,frame):
            frame.fp = self.ramdump.pac_ignore(frame.fp)
            frame.sp = self.ramdump.pac_ignore(frame.sp)
            frame.lr = self.ramdump.pac_ignore(frame.lr)
            frame.pc = self.ramdump.pac_ignore(frame.pc)
        def unwind_backtrace(self, sp, fp, pc, lr, extra_str='',
                             out_file=None):
            offset = 0
            max_frames = 128
            frame_count = 0
            frame = self.Stackframe(fp, sp, lr, pc)
            frame.fp = fp
            frame.sp = sp
            frame.lr = lr
            frame.pc = pc
            self.pac_frame_update(frame)
            backtrace = '\n'

            cpu_work_state = ''
            if (pc & 0x1) or (lr & 0x1):
                cpu_work_state = 'thumb'

            while True:
                where = frame.pc
                offset = 0

                if frame.pc is None:
                    break

                r = self.ramdump.unwind_lookup(frame.pc)
                if r is None:
                    symname = 'UNKNOWN'
                    offset = 0x0
                else:
                    symname, offset = r
                pstring = (
                    extra_str + '[<{0:x}>] {1}+0x{2:x}'.format(frame.pc, symname, offset))
                if out_file:
                    out_file.write(pstring + '\n')
                else:
                    print_out_str(pstring)
                backtrace += pstring + '\n'

                urc = self.unwind_frame(frame, cpu_work_state)
                self.pac_frame_update(frame)
                if urc < 0:
                    break
                frame_count = frame_count + 1
                if frame_count >= max_frames:
                    if out_file != None:
                        out_file.write("Max stack depth reached")
                    break
            return backtrace

    def createMask(self,a, b):
       r = 0
       i = a
       while(i <= b):
           r |= 1 << i
           i = i +1

       return r;

    def pac_ignore(self,data):
        pac_check = self.createMask(self.va_bits, 63)
        top_bit_ignore = 0xff00000000000000
        if data is None or not self.arm64:
            return data
        if (data & pac_check) == pac_check or (data & pac_check) == 0:
            return data
        # When address tagging is used
        # The PAC field is Xn[54:bottom_PAC_bit].
        # In the PAC field definitions, bottom_PAC_bit == 64-TCR_ELx.TnSZ,
        # TCR_ELx.TnSZ is set to 25. so 64-25=39
        pac_mack = self.createMask(self.va_bits, 54)
        result = pac_mack | data
        result = result | top_bit_ignore
        return result

    def load_phys_range(self, path):
        phys_base, phys_end = 0xffffffff, 0
        with open(path, 'r') as _fd:
            phys_base, phys_end = _fd.read().strip().split("--")
        return int(phys_base), int(phys_end)

    def determine_phys_offset(self):
        vmalloc_start = self.modules_end - self.kaslr_offset
        for min_image_align in [0x00200000, 0x00080000, 0x00008000]:

            phys_base = 0x1ffffffff
            phys_end = 0

            if self.reduceddump:
                # Load the phys range from phys_range.txt
                path = os.path.join(os.path.dirname(self.elf_addr[0]), 'phys_range.txt')
                if os.path.exists(path):
                    phys_base, phys_end = self.load_phys_range(path)
                else:
                    print_out_str("Unable to locate the phys_range.txt file !!! Linux banner will not be found !!!")
            else:
                for a in self.ebi_files:
                    _, start, end, path = a
                    if start < phys_base:
                        phys_base = start
                    if end > phys_end:
                        phys_end = end

            if phys_end > 0x1ffffffff:
                phys_end = 0x1ffffffff
            #mask phys_base lower address for alignment
            phys_base = phys_base & 0xfffff0000

            print_out_str("phys_base: {0:x} phys_end: {1:x} step: {2:x}".format(
                            phys_base, phys_end, min_image_align))

            kimage_load_addr = phys_base
            while (kimage_load_addr < phys_end):
                kimage_voffset = self.modules_end - kimage_load_addr
                addr = self.address_of("kimage_voffset") - self.kaslr_offset - \
                    vmalloc_start + kimage_load_addr
                if self.arm64:
                    val = self.read_u64(addr, False)
                else:
                    val = self.read_u32(addr, False)
                if val is None:
                    val = 0
                val = int(val)
                if (int(kimage_voffset) == val):
                    return kimage_load_addr
                kimage_load_addr = kimage_load_addr + min_image_align

        return 0

    def get_kimage_vaddr(self, need_aslr=True):
        kimage_vaddr = None
        if self.get_kernel_version() >= (6, 1, 0):
            kimage_vaddr = self.address_of('_text')
            if need_aslr:
                kimage_vaddr -= self.kaslr_offset
        elif self.get_kernel_version() > (4, 20, 0):
            if self.get_kernel_version() >= (6, 4, 0):
                modules_vsize = 0x80000000
            else:
                modules_vsize = 0x08000000
            bpf_jit_vsize = 0x08000000
            self.page_end = (0xffffffffffffffff << (
                        self.va_bits - 1)) & 0xffffffffffffffff
            if self.address_of("kasan_init") is None:
                self.kasan_shadow_size = 0
            else:
                if self.is_config_defined("CONFIG_KASAN_SW_TAGS"):
                    self.kasan_shadow_size = 1 << (self.va_bits - 4)
                else:
                    self.kasan_shadow_size = 1 << (self.va_bits - 3)
            kimage_vaddr = self.page_end + modules_vsize
            if self.get_kernel_version() < (5, 10, 0):
                kimage_vaddr += bpf_jit_vsize

            # new since v5.11: https://lore.kernel.org/all/20201008153602.9467-3-ardb@kernel.org/
            # The KASAN shadow region is reconfigured so that it ends at the start of
            # the vmalloc region, and grows downwards. That way, the arrangement of
            # the vmalloc space (which contains kernel mappings, modules, BPF region,
            # the vmemmap array etc) is identical between non-KASAN and KASAN builds,
            # which aids debugging.

            if self.get_kernel_version() < (5, 11, 0):
                kimage_vaddr = kimage_vaddr + self.kasan_shadow_size
        else:
            modules_vsize = 0x08000000
            self.va_start = (0xffffffffffffffff << self.va_bits) & 0xffffffffffffffff
            if self.address_of("kasan_init") is None:
                self.kasan_shadow_size = 0
            else:
                self.kasan_shadow_size = 1 << (self.va_bits - 3)
            kimage_vaddr = self.va_start + self.kasan_shadow_size + \
                           modules_vsize
        return kimage_vaddr

    def __init__(self, options, nm_path, gdb_path, objdump_path,gdb_ndk_path):
        self.ebi_files = []
        self.ebi_files_minidump = []
        self.ebi_pa_name_map = {}
        self.md_dict = {}
        self.phys_offset = None
        self.kaslr_offset = options.kaslr_offset
        self.tz_start = 0
        self.ebi_start = 0
        self.hyp_diag_addr = None
        self.rm_debug_addr = None
        self.cpu_type = None
        self.tbi_mask = None
        self.svm_kaslr_offset = None
        self.iommu_pg_table_format = options.iommu_pg_table_format
        self.hw_id = options.force_hardware or None
        self.hw_version = options.force_hardware_version or None
        self.offset_table = []
        self.vmlinux = options.vmlinux
        self.nm_path = nm_path
        self.gdb_path = gdb_path
        self.gdb_ndk_path = gdb_ndk_path
        self.objdump_path = objdump_path
        self.outdir = options.outdir
        self.ftrace_args = options.ftrace_args
        self.ftrace_max_size = options.ftrace_max_size
        self.imem_fname = None
        self.gdbmi = None
        self.gdbmi_hyp = None
        self.arm64 = options.arm64
        self.logcat_limit_time = options.logcat_limit_time
        self.ndk_compatible = False
        self.lookup_table = []
        self.ko_file_names = []
        self.datatype_dict = {}
        self.enum_data = {}
        self.available_cores = []
        self.skip_TLB_Cache_parse = options.skip_TLB_Cache_parse

        if gdb_ndk_path:
            self.gdbmi = gdbmi.GdbMI(self.gdb_ndk_path, self.vmlinux,
                                     0)
            self.gdbmi.open()
            sanity_data = self.address_of("kimage_voffset")
            self.kernel_version = (0, 0, 0)
            if self.arm64:
                self.gdbmi.setup_aarch('aarch64')
                if (sanity_data and (sanity_data & 0xFF000000000000) == 0):
                    print_out_str('RELR tags not compatible with NDK GDB')
                elif sanity_data is not None and self.get_kernel_version() >= (5, 10):
                    print_out_str('vmlinux is ndk-compatible')
                    self.ndk_compatible = True
            if not self.ndk_compatible:
                print_out_str("vmlinux not ndk compatible\n")
                self.gdbmi.close()

        if not self.ndk_compatible:
            self.gdbmi = gdbmi.GdbMI(self.gdb_path, self.vmlinux,
                        0)
            self.gdbmi.open()
            if self.arm64:
                self.gdbmi.setup_aarch('aarch64')
        self.gdbmi.set_gdbmi_aslr_offset()
        self.page_offset = 0xc0000000
        self.thread_size = 8192
        self.qtf_path = options.qtf_path
        self.ftrace_format = options.ftrace_format
        self.skip_qdss_bin = options.skip_qdss_bin
        self.debug = options.debug
        self.dcc = False
        self.sysreg = False
        self.t32_host_system = options.t32_host_system or None
        self.use_stdout = options.stdout
        self.kernel_version = (0, 0, 0)
        self.linux_banner = None
        self.minidump = options.minidump
        self.reduceddump = options.reduceddump
        self.svm = options.svm
        self.elffile = None
        self.ram_elf_file = None
        self.ram_addr = options.ram_addr
        self.autodump = options.autodump
        self.elf_addr = None
        self.module_table = module_table.module_table_class()
        self.hyp = options.hyp
        # Save all paths given from --mod_path option. These will be searched for .ko.unstripped files
        if options.mod_path_list:
            for path in options.mod_path_list:
                self.module_table.add_sym_path(path)
        self.dump_module_symbol_table = options.dump_module_symbol_table
        self.dump_kernel_symbol_table = options.dump_kernel_symbol_table
        self.dump_module_kallsyms = options.dump_module_kallsyms
        self.dump_global_symbol_table = options.dump_global_symbol_table
        self.currentEL = options.currentEL or None
        self.hyp_dump = None
        self.ttbr = None
        self.vttbr = None
        self.hlos_tcr_el1 = None
        self.hlos_sctlr_el1 = None

        if self.hyp:
            self.gdbmi_hyp = gdbmi.GdbMI(self.gdb_path, self.hyp,
                                     0)
            self.gdbmi_hyp.open()

        if self.minidump or self.reduceddump:
            try:
                mod = import_module('elftools.elf.elffile')
                ELFFile = mod.ELFFile
                StringTableSection = mod.StringTableSection
                mod = import_module('elftools.common.py3compat')
                bytes2str = mod.bytes2str
            except ImportError:
                print("Oops, missing required library for minidump. Check README")
                sys.exit(1)

        if options.ram_addr is not None:
            # TODO sanity check to make sure the memory regions don't overlap
            for file_path, start, end in options.ram_addr:
                fd = open(file_path, 'rb')
                if not fd:
                    print_out_str(
                        'Could not open {0}. Will not be part of dump'.format(file_path))
                    continue
                self.ebi_files.append((fd, start, end, file_path))

        elif not options.reduceddump:
            if not self.auto_parse(options.autodump, options.minidump, options.svm):
                print("Oops, auto-parse option failed. Please specify vmlinux & DDR files manually.")
                sys.exit(1)

        elif options.reduceddump:
            if not self.auto_parse(options.autodump, options.minidump, options.svm):
                print("Oops, auto-parse option failed. Please specify vmlinux & HLOS elf files manually.")
                sys.exit(1)

        if self.elf_addr is not None:
            # Setup the needed vector and hash table for binary search in read_physical
            vector, htable, filemap = elfutil.setup_elfmappings(self.elf_addr)
            self.elf_vector, self.elf_htable, self.elf_filemap = vector, htable, filemap


        if options.minidump:
            if not options.autodump:
                file_path = options.ram_elf_addr
            else:
                file_path = os.path.join(options.outdir, 'ap_minidump.elf')
            self.ram_elf_file = file_path
            if not os.path.exists(file_path):
                print_out_str("ELF file not exists, try to generate")
                if minidump_util.generate_elf(options.autodump, options.outdir, self.svm):
                    print_out_str("!!! ELF file generate failed")
                    sys.exit(1)
            fd = open(file_path, 'rb')
            self.elffile = ELFFile(fd)
            for idx, s in enumerate(self.elffile.iter_segments()):
                pa = int(s['p_paddr'])
                va = int(s['p_vaddr'])
                size = int(s['p_filesz'])
                end_addr = pa + size - 1
                for section in self.elffile.iter_sections():
                    if (not section.is_null() and
                            s.section_in_segment(section)):
                        self.ebi_pa_name_map[pa] = section.name
                        if section.name == "KVA_DUMP":
                            kva_dump_addr = pa
                self.ebi_files_minidump.append((idx, pa, end_addr, va,size))

            if options.autodump and os.path.exists(os.path.join(options.autodump, "md_KVA_DUMP.BIN")):
                file_path = os.path.join(options.autodump, "md_KVA_DUMP.BIN")
                fd = open(file_path, 'rb')
                kva_elf = ELFFile(fd)
                for s in kva_elf.iter_sections():
                    start = int(s.header['sh_addr'])
                    size = int(s.header['sh_size'])
                    offset = int(s.header['sh_offset'])
                    pa = kva_dump_addr + offset
                    end_addr = pa + size - 1
                    if start == 0x0:
                        continue
                    self.ebi_files_minidump.append((idx, pa, end_addr, start, size))
                    if s.name not in self.md_dict.keys():
                        self.md_dict[s.name] = [[start,size]]
                    else:
                        self.md_dict[s.name].append([start,size])

        if options.minidump:
            if self.ebi_start == 0:
                self.ebi_start = self.ebi_files_minidump[0][1]
        elif options.reduceddump:
            if self.ebi_start == 0:
                # options.elf_addr needs to be sorted for filename
                self.ebi_start = self.elf_filemap[options.elf_addr[0]][0]
        else:
            if self.ebi_start == 0:
                self.ebi_start = self.ebi_files[0][1]
        if self.phys_offset is None:
            self.get_hw_id()

        if options.phys_offset is not None:
            print_out_str(
                '[!!!] Phys offset was set to {0:x}'.format(\
                    options.phys_offset))
            self.phys_offset = options.phys_offset
        self.s2_walk = False
        if self.svm and not self.minidump:
            from extensions.hyp_trace import HypDump
            hyp_dump = HypDump(self)
            hyp_dump.vmtype = self.svm
            hyp_dump.determine_kaslr()
            self.gdbmi_hyp.kaslr_offset = hyp_dump.hyp_kaslr_addr_offset
            hyp_dump.get_trace_phy()
            if hyp_dump.ttbr1 is None:
                print_out_str('!!! Could not find {}'.format(self.svm))
                print_out_str('!!! Exiting now')
                sys.exit(1)
            self.ttbr = hyp_dump.ttbr1
            self.vttbr = hyp_dump.vttbr
            self.TTBR0_EL1 = hyp_dump.TTBR0_EL1
            self.SCTLR_EL1 = hyp_dump.SCTLR_EL1
            self.TCR_EL1 = hyp_dump.TCR_EL1
            self.VTCR_EL2 = hyp_dump.VTCR_EL2
            self.HCR_EL2 = hyp_dump.HCR_EL2
            self.ttbr_data = hyp_dump.ttbr1_data_info
            self.vttbr_data = hyp_dump.vttbr_el2_data
            self.s2_walk = True

        self.config = []
        self.config_dict = {}
        if not self.get_config():
            print_out_str('!!! Could not get saved configuration')
            print_out_str(
                '!!! This is really bad and probably indicates RAM corruption')
            print_out_str('!!! Some features may be disabled!')

        try:
            self.va_bits = int(self.get_config_val("CONFIG_ARM64_VA_BITS"))
        except:
            self.va_bits = 39
        try:
            self.page_shift = int(self.get_config_val("CONFIG_ARM64_PAGE_SHIFT"))
        except:
            self.page_shift = 12
        try:
            self.pgtable_levels = int(self.get_config_val("CONFIG_PGTABLE_LEVELS"))
        except:
            self.pgtable_levels = 3

        if self.kaslr_offset is None:
            self.determine_kaslr_offset()
            self.gdbmi.kaslr_offset = self.get_kaslr_offset()
        else:
            self.gdbmi.kaslr_offset = self.kaslr_offset

        self.wlan = options.wlan
        if self.arm64:
            if self.get_kernel_version() >= (5, 4):
                self.page_offset = -(1 << self.va_bits) % (1 << 64)
                self.thread_size = self.address_of('__end_init_task') - self.address_of('__start_init_task')
            else:
                self.page_offset = 0xffffffc000000000
                self.thread_size = 16384
        if options.page_offset is not None:
            print_out_str(
                '[!!!] Page offset was set to {0:x}'.format(page_offset))
            self.page_offset = options.page_offset
        self.setup_symbol_tables()
        kimage_vaddr = self.get_kimage_vaddr()
        print_out_str("Kernel version vmlinux: {0}".format(self.kernel_version))
        self.field_offset("struct trace_entry", "preempt_count")
        print_out_str("kimage_vaddr is" ": {:x}".format(kimage_vaddr))
        self.kimage_vaddr = kimage_vaddr + self.get_kaslr_offset()
        self.modules_end = self.page_offset
        if self.arm64:
            self.kimage_voffset = self.address_of("kimage_voffset")
            if self.kimage_voffset is not None:
                self.kimage_voffset = self.kimage_vaddr - self.phys_offset
                self.modules_end = self.kimage_vaddr
                if not (options.phys_offset or self.minidump or self.svm):
                    phys_offset_dyn = self.determine_phys_offset()
                    if phys_offset_dyn:
                        print_out_str("Dynamically determined phys offset is"
                                      ": {:x}".format(phys_offset_dyn))
                        self.phys_offset = phys_offset_dyn

                if self.kimage_vaddr > self.phys_offset:
                    self.kimage_voffset = self.kimage_vaddr - self.phys_offset
                else:
                    self.kimage_voffset = self.phys_offset
                print_out_str("The kimage_voffset extracted is: {:x}".format(self.kimage_voffset))
        else:
            self.kimage_voffset = self.address_of("kimage_voffset")
            if self.kimage_voffset is not None:
                if not (options.phys_offset or self.minidump):
                    phys_offset_dyn = self.determine_phys_offset()
                    if phys_offset_dyn:
                        print_out_str("Dynamically determined phys offset is"
                                      ": {:x}".format(phys_offset_dyn))
                        self.phys_offset = phys_offset_dyn
            self.kimage_voffset = None

        # The address of swapper_pg_dir can be used to determine
        # whether or not we're running with LPAE enabled since an
        # extra 4k is needed for LPAE. If it's 0x5000 below
        # PAGE_OFFSET + TEXT_OFFSET then we know we're using LPAE. For
        # non-LPAE it should be 0x4000 below PAGE_OFFSET + TEXT_OFFSET
        self.swapper_pg_dir_addr = self.address_of('swapper_pg_dir')
        if self.swapper_pg_dir_addr is None:
            print_out_str('!!! Could not get the swapper page directory!')
            if not self.minidump:
                print_out_str(
                    '!!! Your vmlinux is probably wrong for these dumps')

                print_out_str('!!! Exiting now')
                sys.exit(1)
        if self.get_kernel_version() > (5, 7, 0) and self.arm64:
            stext = self.address_of('primary_entry')
        else:
            stext = self.address_of('stext')
        if self.kimage_voffset is None:
            self.kernel_text_offset = stext - self.page_offset
        else:
            self.kernel_text_offset = stext - self.kimage_vaddr

        pg_dir_size = self.kernel_text_offset + self.page_offset \
            - self.swapper_pg_dir_addr

        if self.arm64:
            print_out_str('Using 64bit MMU')
            self.mmu = Armv8MMU(self)
        elif pg_dir_size == 0x4000:
            print_out_str('Using non-LPAE MMU')
            if self.minidump:
                self.mmu = None
            else:
                self.mmu = Armv7MMU(self)
        elif pg_dir_size == 0x5000:
            print_out_str('Using LPAE MMU')
            text_offset = 0x8000
            pg_dir_size = 0x5000    # 0x4000 for non-LPAE
            swapper_pg_dir_addr = self.phys_offset + text_offset - pg_dir_size

            # We deduce ttbr1 and ttbcr.t1sz based on the value of
            # PAGE_OFFSET. This is based on v7_ttb_setup in
            # arch/arm/mm/proc-v7-3level.S:

            # * TTBR0/TTBR1 split (PAGE_OFFSET):
            # *   0x40000000: T0SZ = 2, T1SZ = 0 (not used)
            # *   0x80000000: T0SZ = 0, T1SZ = 1
            # *   0xc0000000: T0SZ = 0, T1SZ = 2
            if self.page_offset == 0x40000000:
                t1sz = 0
            elif self.page_offset == 0x80000000:
                t1sz = 1
            elif self.page_offset == 0xc0000000:
                t1sz = 2
                # need to fixup ttbr1 since we'll be skipping the
                # first-level lookup (see v7_ttb_setup):
                # /* PAGE_OFFSET == 0xc0000000, T1SZ == 2 */
                # add      \ttbr1, \ttbr1, #4096 * (1 + 3) @ only L2 used, skip
                # pgd+3*pmd
                swapper_pg_dir_addr += (4096 * (1 + 3))
            else:
                raise Exception(
                    'Invalid phys_offset for page_table_walk: 0x%x'
                    % self.page_offset)
            self.mmu = Armv7LPAEMMU(self, swapper_pg_dir_addr, t1sz)
        else:
            print_out_str(
                "!!! Couldn't determine whether or not we're using LPAE!")
            print_out_str(
                '!!! This is a BUG in the parser and should be reported.')
            sys.exit(1)

        if not self.match_version():
            print_out_str('!!! Could not get the Linux version!')
            print_out_str(
                '!!! Your vmlinux is probably wrong for these dumps')
            print_out_str('!!! Exiting now')
            sys.exit(1)

        self.unwind = self.Unwinder(self)
        if self.module_table.sym_paths_exist():
            self.setup_module_symbols()
            self.gdbmi.setup_module_table(self.module_table)
            if self.dump_global_symbol_table:
                self.dump_global_symbol_lookup_table()

        mm_init(self)
        self.set_available_cores()


    def get_section_address(self,section):
        """
        Function to return address and size corresponding to the section name in elf files.

        :param section: name of the section.
        :type addr: str

        :return: A list of list of addresses and sizes corresponding to the section name.
        """
        res_dict = self.md_dict
        if (section in res_dict.keys()):
            return res_dict[section]
        else:
            raise InvalidInput

    def __del__(self):
        self.gdbmi.close()
        if self.hyp:
            self.gdbmi_hyp.close()

    def open_file(self, file_name, mode='wt'):
        """Open a file in the out directory.

        Example:

        >>> with self.ramdump.open_file('pizza.txt') as p:
                p.write('Pizza is the best\\n')
        """
        file_path = os.path.join(self.outdir, file_name)
        f = None
        try:
            dir_path = os.path.dirname(file_path)
            if not os.path.exists(dir_path) and ('w' in mode or 'a' in mode):
                os.makedirs(dir_path)
            f = open(file_path, mode)
        except:
            print_out_str('Could not open path {0}'.format(file_path))
            print_out_str('Do you have write/read permissions on the path?')
            sys.exit(1)
        return f

    def chmod(self, file_name, mode):
        file_path = os.path.join(self.outdir, file_name)
        return os.chmod(file_path, mode)

    def remove_file(self, file_name):
        file_path = os.path.join(self.outdir, file_name)
        try:
            if (os.path.exists(file_path)):
                os.remove(file_path)
        except:
            print_out_str('Could not remove file {0}'.format(file_path))
            print_out_str('Do you have write/read permissions on the path?')
            sys.exit(1)

    def get_srcdir(self):
        """ Returns absolute path of directory containing ramdump.py """
        return os.path.dirname(os.path.abspath(__file__))

    def get_config(self):
        kconfig_addr = self.address_of('kernel_config_data')
        if kconfig_addr is None:
            return
        if self.get_kernel_version() > (5, 0, 0):
            kconfig_addr_end = self.address_of('kernel_config_data_end')
            if kconfig_addr_end is None:
                return
            kconfig_size = kconfig_addr_end - kconfig_addr
            # magic is 8 bytes before kconfig_addr and data
            # starts at kconfig_addr for kernel > 5.0.0
            kconfig_addr = kconfig_addr - 8
        else:
            kconfig_size = self.sizeof('kernel_config_data')
            # size includes magic, offset from it
            kconfig_size = kconfig_size - 16 - 1

        # kconfig data starts with magic 8 byte string, go past that
        zconfig = os.path.join(self.outdir, "elf_temp.txt")
        temp_file = open(zconfig, 'wb+')
        size = kconfig_addr + 8
        s = self.read_elf_memory(kconfig_addr, size, temp_file)
        temp_file.close()
        if s != 'IKCFG_ST':
            return
        temp_file = open(zconfig, 'wb+')
        kconfig_addr = kconfig_addr + 8
        val = self.read_elf_memory(kconfig_addr, kconfig_size + kconfig_addr,
                                      temp_file)

        temp_file.close()
        zconfig_in = gzip.open(temp_file.name, 'rt')
        try:
            t = zconfig_in.readlines()
        except:
            return False
        zconfig_in.close()
        os.remove(zconfig)
        for l in t:
            self.config.append(l.rstrip())
            if not l.startswith('#') and l.strip() != '':
                eql = l.find('=')
                cfg = l[:eql]
                val = l[eql+1:]
                self.config_dict[cfg] = val.strip()
        return True

    def get_config_val(self, config):
        """Gets the value of a kernel config option.

        Example:

        >>> va_bits = int(dump.get_config_val("CONFIG_ARM64_VA_BITS"))
        39
        """
        return self.config_dict.get(config)

    def is_config_defined(self, config):
        return config in self.config_dict

    def get_kernel_version(self):
        if self.kernel_version == (0, 0, 0):
            vm_v = self.gdbmi.get_value_of_string('linux_banner')
            if vm_v is None:
                print_out_str('!!! Could not read linux_banner from vmlinux!')
                sys.exit(1)
            v = re.search('Linux version (\d{0,2}\.\d{0,2}\.\d{0,3})', vm_v)
            if v is None:
                print_out_str('!!! Could not extract version info!')
                sys.exit(1)
            self.version = v.group(1)
            match = re.search('(\d+)\.(\d+)\.(\d+)', self.version)
            if match is not None:
                self.version = tuple(map(int, match.groups()))
                self.kernel_version = self.version
                self.linux_banner = vm_v
            else:
                print_out_str('!!! Could not extract version info! {0}'.format(self.version))
                sys.exit(1)

        return self.kernel_version

    def kernel_virt_to_phys(self, addr):
        if self.minidump:
            return minidump_util.minidump_virt_to_phys(self.ebi_files_minidump,addr)
        else:
            if self.kimage_voffset is None:
                return addr - self.page_offset + self.phys_offset
            else:
                if self.kernel_version > (4, 20, 0):
                    if not (addr & (1 << (self.va_bits - 1))):
                        return addr - self.page_offset + self.phys_offset
                    else:
                        return addr - (self.kimage_voffset)
                else:
                    if addr & (1 << (self.va_bits - 1)):
                        return addr - self.page_offset + self.phys_offset
                    else:
                        return addr - (self.kimage_voffset)

    def match_version(self):
        banner_addr = self.address_of('linux_banner')
        if banner_addr is not None:
            banner_addr = self.kernel_virt_to_phys(banner_addr)
            banner_len = len(self.linux_banner)
            b = self.read_cstring(banner_addr, banner_len, False)
            if b is None:
                print_out_str('!!! Banner not found in dumps!')
                return False
            print_out_str('Linux Banner: ' + b.rstrip())
            if str(self.linux_banner) in str(b):
                print_out_str("Linux banner from vmlinux = %s" % self.linux_banner)
                print_out_str("Linux banner from dump = %s" % b)
                return True
            else:
                print_out_str("Expected Linux banner = %s" % self.linux_banner)
                print_out_str("Linux banner in Dumps = %s" % b)
                return False
        else:
            print_out_str('!!! linux_banner sym not found in vmlinux')
            return False

    def print_command_line(self):
        command_addr = self.address_of('saved_command_line')
        if command_addr is not None:
            command_addr = self.read_word(command_addr)
            b = self.read_cstring(command_addr, 2048)
            if b is None:
                print_out_str('!!! could not read saved command line address')
                return False
            print_out_str('Command Line: ' + b)
            return True
        else:
            print_out_str('!!! Could not lookup saved command line address')
            return False

    def print_socinfo_minidump(self):
        content_socinfo = None
        boards = get_supported_boards()
        for board in boards:
            if self.hw_id == board.board_num:
                content_socinfo = board.ram_start + board.smem_addr_buildinfo
                break
        sernum_offset = self.field_offset('struct socinfo_v10', 'serial_number')
        if sernum_offset is None:
            sernum_offset = self.field_offset('struct socinfo_v0_10', 'serial_number')
            if sernum_offset is None:
                print_out_str("No serial number information available")
                return False
        if content_socinfo:
            addr_of_sernum = content_socinfo + sernum_offset
            serial_number = self.read_u32(addr_of_sernum, False)
            if serial_number is not None:
                print_out_str('Serial number %s' % hex(serial_number))
                return True
            return False

        return False

    def print_socinfo(self):
        if self.kernel_version < (5, 4, 0):
            try:
                if self.read_pointer('socinfo') is None:
                  return None
                content_socinfo = hex(self.read_pointer('socinfo'))
                content_socinfo = content_socinfo.strip('L')

                sernum_offset = self.field_offset('struct socinfo_v10', 'serial_number')
                if sernum_offset is None:
                    sernum_offset = self.field_offset('struct socinfo_v0_10', 'serial_number')
                    if sernum_offset is None:
                        print_out_str("No serial number information available")
                        return False
                addr_of_sernum = hex(int(content_socinfo, 16) + sernum_offset)
                addr_of_sernum = addr_of_sernum.strip('L')
                serial_number = self.read_u32(int(addr_of_sernum, 16))
                if serial_number is not None:
                    print_out_str('Serial number %s' % hex(serial_number))
                    return True
            except:
                pass
            return False
        else:
            socinfo = self.address_of('socinfo')
            if socinfo is None:
                return None
            socinfo = self.read_pointer(socinfo)
            if socinfo is None:
                return None
            ver = int(self.read_structure_field(socinfo, 'struct socinfo', 'ver') or 0)
            chip_ver_major = (ver & 0xFFFF0000) >> 16
            chip_ver_minor = (ver & 0x0000FFFF)
            print_out_str("Chip Version: v{0}.{1}".format(chip_ver_major, chip_ver_minor))
            serial_num = int(self.read_structure_field(socinfo, 'struct socinfo', 'serial_num') or 0)
            print_out_str("Chip Serial Number 0x{0:x}".format(serial_num))
            return True

    def auto_parse(self, file_path, minidump, svm):
        if self.reduceddump:
            elfflag, ebiflag = False, False
            cls = None

            info = AutoDumpInfoReducedDump(file_path, minidump, self.reduceddump, svm)
            info.parse()
            if info is not None:
                if len(info.ebi_files) > 0:
                    self.ebi_files = info.ebi_files
                    self.phys_offset = self.ebi_files[0][1]
                    if self.get_hw_id():
                        for (f, start, end, filename) in self.ebi_files:
                            print_out_str('Adding {0} {1:x}--{2:x}'.format(
                                filename, start, end))
                        ebiflag = True
                    else:
                        return False

                if len(info.elf_files) > 0:
                    self.elf_addr = info.elf_files
                    for filename in self.elf_addr:
                        print_out_str('Adding {0}'.format(filename))
                    elfflag = True
            return elfflag and ebiflag

        else:
            for cls in sorted(AutoDumpInfo.__subclasses__(),
                              key=lambda x: x.priority, reverse=True):
                info = cls(file_path, minidump, self.reduceddump, svm)
                info.parse()
                if info is not None and len(info.ebi_files) > 0:
                    self.ebi_files = info.ebi_files
                    self.phys_offset = self.ebi_files[0][1]
                    if self.get_hw_id():
                        for (f, start, end, filename) in self.ebi_files:
                            print_out_str('Adding {0} {1:x}--{2:x}'.format(
                                filename, start, end))
                        return True
            self.ebi_files = None
            return False

    def create_t32_launcher(self):
        out_path = os.path.abspath(self.outdir)

        t32_host_system = self.t32_host_system or platform.system()

        launch_config = self.open_file('t32_config.t32')
        launch_config.write('OS=\n')
        launch_config.write('ID=T32_1000002\n')
        if t32_host_system != 'Linux':
            launch_config.write('TMP=C:\\TEMP\n')
            launch_config.write('SYS=C:\\T32\n')
            launch_config.write('HELP=C:\\T32\\pdf\n')
        else:
            launch_config.write('TMP=/tmp\n')
            launch_config.write('SYS=/opt/t32\n')
            launch_config.write('HELP=/opt/t32/pdf\n')
        launch_config.write('\n')
        launch_config.write('PBI=SIM\n')
        launch_config.write('\n')
        launch_config.write('SCREEN=\n')
        launch_config.write('FONT=LARGE\n')
        launch_config.write('HEADER=Trace32-ScorpionSimulator\n')
        launch_config.write('\n')
        if t32_host_system != 'Linux':
            launch_config.write('PRINTER=WINDOWS\n')
        launch_config.write('\n')
        launch_config.write('RCL=NETASSIST\n')
        launch_config.write('PACKLEN=1024\n')
        launch_config.write('PORT=%d\n' % random.randint(20000, 30000))
        launch_config.write('\n')

        launch_config.close()

        startup_script = self.open_file('t32_startup_script.cmm')

        startup_script.write('title \"' + out_path + '\"\n')

        is_cortex_a53 = self.hw_id in ["8916", "8939", "8936", "bengal", "scuba"]

        if self.arm64 and is_cortex_a53:
            startup_script.write('sys.cpu CORTEXA53\n')
        else:
            startup_script.write('sys.cpu {0}\n'.format(self.cpu_type))
            if self.minidump:
                startup_script.write('SYStem.Option MMUSPACES OFF\n')
            else:
                startup_script.write('SYStem.Option MMUSPACES ON\n')
            startup_script.write('SYStem.Option ZONESPACES OFF\n')

        startup_script.write('sys.up\n')

        if is_cortex_a53 and not self.arm64:
            startup_script.write('r.s m 0x13\n')
        for ram in self.ebi_files:
            ebi_path = os.path.abspath(ram[3])
            startup_script.write('data.load.binary {0} 0x{1:x}\n'.format(
                ebi_path, ram[1]))
        if self.minidump:
            dload_ram_elf = 'data.load.elf {} /LOGLOAD /nosymbol\n'.format(os.path.abspath(self.ram_elf_file))
            startup_script.write(dload_ram_elf)
        # Check to include Reduced dump elf's
        if self.elf_addr:
            for file in self.elf_addr:
                startup_script.write('data.load.elf {0} /noclear\n'.format(file))

        if not self.minidump:
            if self.arm64:
                if self.svm:
                    startup_script.write('Data.Set SPR:0x30201 %Quad 0x{0:x}\n'.format(
                    self.ttbr_data))
                    startup_script.write('Data.Set SPR:0x34210 %Quad 0x{0:x}\n'.format(
                    self.vttbr_data))
                    startup_script.write('Data.Set SPR:0x30100 %Quad 0x{0:x}\n'.format(
                    self.SCTLR_EL1))
                    startup_script.write('Data.Set SPR:0x30200 %Quad 0x{0:x}\n'.format(
                    self.TTBR0_EL1))
                    startup_script.write('Data.Set SPR:0x30202 %Quad 0x{0:x}\n'.format(
                    self.TCR_EL1))
                    startup_script.write('Data.Set SPR:0x34110 %Quad 0x{0:x}\n'.format(
                    self.HCR_EL2))
                    startup_script.write('Data.Set SPR:0x34212 %Quad 0x{0:x}\n'.format(
                    self.VTCR_EL2))
                    startup_script.write('R.S M 5\n')
                else:
                    startup_script.write('Data.Set SPR:0x30201 %Quad 0x{0:x}\n'.format(
                        self.kernel_virt_to_phys(self.swapper_pg_dir_addr)))
                    tcr_el1 = self.get_tcr_el1(is_cortexa=is_cortex_a53)
                    if is_cortex_a53:
                        startup_script.write('Data.Set SPR:0x30202 %Quad 0x{0:016X}\n'.format(tcr_el1))
                        startup_script.write('Data.Set SPR:0x30A20 %Quad 0x000000FF440C0400\n')
                        startup_script.write('Data.Set SPR:0x30A30 %Quad 0x0000000000000000\n')
                        startup_script.write('Data.Set SPR:0x30100 %Quad 0x0000000034D5D91D\n')
                    elif self.cpu_type == 'ARMV9-A':
                        if self.hlos_tcr_el1:
                            startup_script.write('Data.Set SPR:0x30202 %Quad 0x{0:x}\n'.format(
                            self.hlos_tcr_el1))
                        else:
                            startup_script.write('Data.Set SPR:0x30202 %Quad 0x{0:016X}\n'.format(tcr_el1))
                        startup_script.write('Data.Set SPR:0x30A20 %Quad 0x000000FF440C0400\n')
                        startup_script.write('Data.Set SPR:0x30A30 %Quad 0x0000000000000000\n')
                        if self.hlos_sctlr_el1:
                            startup_script.write('Data.Set SPR:0x30100 %Quad 0x{0:x}\n'.format(
                            self.hlos_sctlr_el1))
                        else:
                            startup_script.write('Data.Set SPR:0x30100 %Quad 0x0000000084C5D93D\n')
                        corevcpu_path = os.path.join(self.outdir,'corevcpu0_regs.cmm')
                        if os.path.exists(corevcpu_path):
                            startup_script.write('do ' + corevcpu_path + '\n')
                    else:
                        startup_script.write('Data.Set SPR:0x30202 %Quad 0x{0:016X}\n'.format(tcr_el1))
                        startup_script.write('Data.Set SPR:0x30A20 %Quad 0x000000FF440C0400\n')
                        startup_script.write('Data.Set SPR:0x30A30 %Quad 0x0000000000000000\n')
                        startup_script.write('Data.Set SPR:0x30100 %Quad 0x0000000004C5D93D\n')

                    startup_script.write('Register.Set NS 1\n')
                    startup_script.write('Register.Set CPSR 0x1C5\n')
            else:
                # ARM-32: MMU is enabled by default on most platforms.
                mmu_enabled = 1
                if self.mmu is None:
                    mmu_enabled = 0
                startup_script.write(
                    'PER.S.simple C15:0x1 %L 0x{0:x}\n'.format(mmu_enabled))
                startup_script.write(
                    'PER.S.simple C15:0x2 %L 0x{0:x}\n'.format(self.mmu.ttbr))
                if isinstance(self.mmu, Armv7LPAEMMU):
                    # TTBR1. This gets setup once and never change again even if TTBR0
                    # changes
                    startup_script.write('PER.S.F C15:0x102 %L 0x{0:x}\n'.format(
                        self.mmu.ttbr + 0x4000))
                    # TTBCR with EAE and T1SZ set approprately
                    startup_script.write(
                        'PER.S.F C15:0x202 %L 0x80030000\n')
                startup_script.write('mmu.on\n')
                startup_script.write('mmu.scan\n')

        where = os.path.abspath(self.vmlinux)
        kaslr_offset = self.get_kaslr_offset()
        if kaslr_offset != 0:
            where += ' 0x{0:x}'.format(kaslr_offset)
        dloadelf = 'data.load.elf {} /nocode\n'.format(where)
        startup_script.write(dloadelf)

        if self.arm64 and not self.minidump:
            startup_script.write('TRANSlation.COMMON NS:0xF000000000000000--0xffffffffffffffff\n')
            startup_script.write('trans.tablewalk on\n')
            startup_script.write('trans.on\n')
            if not self.svm and self.cpu_type != 'ARMV9-A':
                startup_script.write('MMU.Delete\n')
                startup_script.write('MMU.SCAN PT 0xFFFFFF8000000000--0xFFFFFFFFFFFFFFFF\n')
                startup_script.write('mmu.on\n')
                startup_script.write('mmu.pt.list 0xffffff8000000000\n')

        if self.minidump:
            startup_script.write('y.pointer x29\n')
            startup_script.write('frame.config.eabi on\n')
            if self.arm64:
                startup_script.write('Register.Set CPSR 0x1C5\n')

        if t32_host_system != 'Linux':
            if self.arm64:
                startup_script.write('IF OS.DIR("C:\\T32\\demo\\arm64")\n')
                startup_script.write('(\n')
                startup_script.write(
                     'task.config C:\\T32\\demo\\arm64\\kernel\\linux\\awareness\\linux.t32 /ACCESS NS:\n')
                startup_script.write(
                     'menu.reprogram C:\\T32\\demo\\arm64\\kernel\\linux\\awareness\\linux.men\n')
                startup_script.write(')\n')
                startup_script.write('ELSE\n')
                startup_script.write('(\n')
                startup_script.write(
                    'task.config C:\\T32\\demo\\arm\\kernel\\linux\\awareness\\linux.t32 /ACCESS NS:\n')
                startup_script.write(
                    'menu.reprogram C:\\T32\\demo\\arm\\kernel\\linux\\awareness\\linux.men\n')
                startup_script.write(')\n')
            else:
                if self.kernel_version > (3, 0, 0):
                    startup_script.write(
                        'task.config c:\\t32\\demo\\arm\\kernel\\linux\\linux-3.x\\linux3.t32\n')
                    startup_script.write(
                        'menu.reprogram c:\\t32\\demo\\arm\\kernel\\linux\\linux-3.x\\linux.men\n')
                else:
                    startup_script.write(
                        'task.config c:\\t32\\demo\\arm\\kernel\\linux\\linux.t32\n')
                    startup_script.write(
                        'menu.reprogram c:\\t32\\demo\\arm\\kernel\\linux\\linux.men\n')
        else:
            if self.arm64:
                startup_script.write('IF OS.DIR("/opt/t32/demo/arm64")\n')
                startup_script.write('(\n')
                startup_script.write(
                    'task.config /opt/t32/demo/arm64/kernel/linux/linux-3.x/linux3.t32\n')
                startup_script.write(
                    'menu.reprogram /opt/t32/demo/arm64/kernel/linux/linux-3.x/linux.men\n')
                startup_script.write(')\n')
                startup_script.write('ELSE\n')
                startup_script.write('(\n')
                startup_script.write(
                    'task.config /opt/t32/demo/arm/kernel/linux/linux-3.x/linux3.t32\n')
                startup_script.write(
                    'menu.reprogram /opt/t32/demo/arm/kernel/linux/linux-3.x/linux.men\n')
                startup_script.write(')\n')
            else:
                if self.kernel_version > (3, 0, 0):
                    startup_script.write(
                        'task.config /opt/t32/demo/arm/kernel/linux/linux-3.x/linux3.t32\n')
                    startup_script.write(
                        'menu.reprogram /opt/t32/demo/arm/kernel/linux/linux-3.x/linux.men\n')
                else:
                    startup_script.write(
                        'task.config /opt/t32/demo/arm/kernel/linux/linux.t32\n')
                    startup_script.write(
                        'menu.reprogram /opt/t32/demo/arm/kernel/linux/linux.men\n')

        if self.get_kernel_version() >= (5, 10) and not self.minidump:
            mod_dir = os.path.dirname(self.vmlinux)
            mod_dir = os.path.abspath(mod_dir)
            if t32_host_system != 'Linux':
                startup_script.write('IF OS.DIR("C:\\T32\\demo\\arm64")\n')
                startup_script.write('(\n')
                startup_script.write('sYmbol.AUTOLOAD.CHECKCOMMAND  ' + '"do C:\\T32\\demo\\arm64\\kernel\\linux\\awareness\\autoload.cmm"' + '\n')
                startup_script.write(')\n')
                startup_script.write('ELSE\n')
                startup_script.write('(\n')
                startup_script.write('sYmbol.AUTOLOAD.CHECKCOMMAND  ' + '"do C:\\T32\\demo\\arm\\kernel\\linux\\etc\\gdb\\gdb_autoload.cmm"' + '\n')
                startup_script.write(')\n')
            else:
                startup_script.write('IF OS.DIR("/opt/t32/demo/arm64")\n')
                startup_script.write('(\n')
                startup_script.write('sYmbol.AUTOLOAD.CHECKCOMMAND  ' + '"do /opt/t32/demo/arm64/kernel/linux/awareness/autoload.cmm"' + '\n')
                startup_script.write(')\n')
                startup_script.write('ELSE\n')
                startup_script.write('(\n')
                startup_script.write('sYmbol.AUTOLOAD.CHECKCOMMAND  ' + '"do /opt/t32/demo/arm/kernel/linux/etc/gdb/gdb_autoload.cmm"' + '\n')
                startup_script.write(')\n')
            if self.module_table.sym_path_list:
                startup_script.write("y.spath =  " +'"{0}"'.format(self.module_table.sym_path_list[0])+ '\n')
                if len(self.module_table.sym_path_list) > 1 :
                    for path in self.module_table.sym_path_list[1:]:
                        startup_script.write("y.spath +=  " +'"{0}"'.format(path)+ '\n')
            else:
                startup_script.write('sYmbol.SourcePATH.Set ' + '"' + mod_dir + '"' + "\n")
            startup_script.write('TASK.sYmbol.Option AutoLoad Module\n')
            startup_script.write('TASK.sYmbol.Option AutoLoad noprocess\n')
            startup_script.write('sYmbol.AutoLOAD.List\n')
            startup_script.write('sYmbol.AutoLOAD.CHECK\n')
        else:
            for mod_tbl_ent in self.module_table.module_table:
                mod_sym_path = mod_tbl_ent.get_sym_path()
                if mod_sym_path != '':
                    ld_mod_sym = ''
                    where = os.path.abspath(mod_sym_path)
                    if self.minidump:
                        if mod_tbl_ent.section_offsets:
                            ld_mod_sym = "Data.LOAD.Elf " + where + " /NoClear /CODESEC /RELOC .text at " + str(hex(mod_tbl_ent.module_offset))
                            if ".data" in mod_tbl_ent.section_offsets.keys():
                                ld_mod_sym += " /RELOC .data at " + str(hex(mod_tbl_ent.section_offsets['.data']))
                            if ".bss" in mod_tbl_ent.section_offsets.keys() :
                                ld_mod_sym += " /RELOC .bss at " + str(hex(mod_tbl_ent.section_offsets['.bss']))
                            ld_mod_sym += "\n"
                    elif 'wlan' in mod_tbl_ent.name:
                        ld_mod_sym = "Data.LOAD.Elf " + where + " " + str(hex(mod_tbl_ent.module_offset)) +  " /NoCODE /NoClear /NAME " + mod_tbl_ent.name + " /reloctype 0x3" + "\n"
                    else:
                        ld_mod_sym = "Data.LOAD.Elf " + where + " /NoCODE /NoClear /NAME " + mod_tbl_ent.name + " /reloctype 0x3" + "\n"
                    startup_script.write(ld_mod_sym)

        if not self.minidump:
            startup_script.write('task.dtask\n')

        startup_script.write(
            'v.v  %ASCII %STRING linux_banner\n')
        if os.path.exists(os.path.join(out_path, 'regs_panic.cmm')):
            startup_script.write(
                'do {0}\n'.format(out_path + '/regs_panic.cmm'))
        elif os.path.exists(os.path.join(out_path, '/core0_regs.cmm')):
            startup_script.write(
                'do {0}\n'.format(out_path + '/core0_regs.cmm'))
        startup_script.close()

        if t32_host_system != 'Linux':
            launch_file = 'launch_t32.bat'
            t32_bat = self.open_file(launch_file)
            if self.arm64:
                t32_binary = 'C:\\T32\\bin\\windows64\\t32MARM64.exe'
            elif is_cortex_a53:
                t32_binary = 'C:\\T32\\bin\\windows64\\t32MARM64.exe'
            else:
                t32_binary = 'c:\\T32\\bin\\windows64\\t32MARM.exe'
            t32_bat.write(('start '+ t32_binary + ' -c ' + out_path + '/t32_config.t32, ' +
                          out_path + '/t32_startup_script.cmm'))
            t32_bat.close()
        else:
            launch_file = 'launch_t32.sh'
            t32_sh = self.open_file(launch_file)
            if self.arm64:
                t32_binary = '/opt/t32/bin/pc_linux64/t32marm64-qt'
            elif is_cortex_a53:
                t32_binary = '/opt/t32/bin/pc_linux64/t32marm-qt'
            else:
                t32_binary = '/opt/t32/bin/pc_linux64/t32marm-qt'
            t32_sh.write('#!/bin/sh\n\n')
            t32_sh.write('{0} -c {1}/t32_config.t32, {1}/t32_startup_script.cmm &\n'.format(t32_binary, out_path))
            t32_sh.close()
            self.chmod(launch_file, stat.S_IRWXU)

        print_out_str(
            '--- Created a T32 Simulator launcher (run {})'.format(launch_file))

    def get_tcr_el1(self, is_cortexa=False):
        if not is_cortexa:
            tcr_el1 = Register(
                0x00000032B5193519,
                TG1=(31, 30),
                T1SZ=(21, 16),
                TG0=(15, 14),
                T0SZ=(5, 0)
            )
            tg1_granule_size = {
                4096  : 0b10,
                16384 : 0b01,
                65536 : 0b11
            }
            tg0_granule_size = {
                4096  : 0b00,
                16384 : 0b10,
                65536 : 0b01
            }
            tcr_el1.TG1 = tg1_granule_size.get(self.get_page_size(), 0b10)
            tcr_el1.TG0 = tg0_granule_size.get(self.get_page_size(), 0b00)
            tcr_el1.T1SZ = tcr_el1.T0SZ = 64 - self.va_bits
        else:
            tcr_el1 = Register(
                0x00000012B5193519,
                RES0=(63, 39),
                RES1=(38, 0)
            )
        return tcr_el1.value

    def read_tz_offset(self):
        if self.tz_addr == 0:
            print_out_str(
                'No TZ address was given, cannot read the magic value!')
            return None
        else:
            return self.read_word(self.tz_addr, False)

    def get_kaslr_offset(self):
        return self.kaslr_offset

    def determine_kaslr_offset(self):
        if self.svm and self.svm_kaslr_offset:
            self.kaslr_offset = self.svm_kaslr_offset
            self.kaslr_addr = None
            return
        elif self.svm and not self.svm_kaslr_offset:
            self.kaslr_offset = 0
            self.kaslr_addr = None
            return
        else:
            self.kaslr_offset = 0
            if self.kaslr_addr is None:
                print_out_str('!!!! Kaslr addr is not provided.')
            else:
                if self.minidump:
                    for a in self.ebi_files:
                        if "md_SHRDIMEM".lower() in a[3].lower():
                            self.kaslr_addr = a[1] + 0x6d0
                            break
                kaslr_magic = self.read_u32(self.kaslr_addr, False)
                self.kaslr_offset = self.read_u64(self.kaslr_addr + 4, False)

                try:
                    self.dynamic_kaslr_offset = None
                    kimage_vaddr_va = self.address_of('kimage_vaddr')
                    kimage_vaddr = self.get_kimage_vaddr(need_aslr=False)
                    kimage_vaddr_phy = self.phys_offset + kimage_vaddr_va - kimage_vaddr
                    kimage_va_temp = self.read_physical(kimage_vaddr_phy, 8)
                    kimage_va = struct.unpack('<Q', kimage_va_temp)
                    kimage_va = int(kimage_va[0])
                    if kimage_va >= kimage_vaddr:
                        self.dynamic_kaslr_offset = kimage_va - kimage_vaddr
                        print_out_str("dynamic_kaslr_offset is: "  + str(hex(self.dynamic_kaslr_offset)))
                except:
                    self.dynamic_kaslr_offset = None
                    pass

                if kaslr_magic != 0xdead4ead:
                    if self.is_config_defined("CONFIG_RANDOMIZE_BASE"):
                        if self.dynamic_kaslr_offset is not None:
                            self.kaslr_offset = self.dynamic_kaslr_offset
                        else:
                            print_out_str('!!!! Could not get the dynamic_kaslr_offset.')
                    else:
                        print_out_str('!!!! Kaslr feature is not enabled.')
                        self.kaslr_offset = 0x0
                else:
                    print_out_str("The kaslr_offset extracted is: " + str(hex(self.kaslr_offset)))

    def get_page_size(self):
        return 1 << self.page_shift

    def get_hw_id(self, add_offset=True):
        socinfo_format = -1
        socinfo_id = -1
        socinfo_version = 0
        socinfo_build_id = 'DUMMY'
        chosen_board = None
        use_predefined = False

        boards = get_supported_boards()

        if (self.hw_id is None):
            if not self.minidump:
                heap_toc_offset = self.field_offset('struct smem_shared', 'heap_toc')
                global_partition_offset_offset = self.field_offset('struct smem_header', 'free_offset')
                if not heap_toc_offset is None:
                    smem_heap_entry_size = self.sizeof('struct smem_heap_entry')
                    offset_offset = self.field_offset('struct smem_heap_entry', 'offset')
                elif not global_partition_offset_offset is None:
                    smem_partition_header_size = self.sizeof('struct smem_partition_header')
                    uncached_offset_offset = self.field_offset('struct smem_partition_header', 'offset_free_uncached')
                    smem_private_entry_size = self.sizeof('struct smem_private_entry')
                    entry_item_offset = self.field_offset('struct smem_private_entry', 'item')
                    item_size_offset = self.field_offset('struct smem_private_entry', 'size')
                else:
                    print_out_str('!!!! Could not get a necessary offset for auto detection!')
                    print_out_str('!!!! Try to use predefined offset!')
                    use_predefined = True
            for board in boards:
                if self.minidump or use_predefined:
                    if hasattr(board, 'smem_addr_buildinfo'):
                        socinfo_start = board.smem_addr_buildinfo
                        if add_offset:
                            socinfo_start += board.ram_start
                    else:
                        continue
                elif not heap_toc_offset is None:
                    socinfo_start_addr = board.smem_addr + heap_toc_offset + smem_heap_entry_size * SMEM_HW_SW_BUILD_ID + offset_offset
                    if add_offset:
                        socinfo_start_addr += board.ram_start
                    soc_start = self.read_int(socinfo_start_addr, False)
                    if soc_start is None:
                        continue
                    socinfo_start = board.smem_addr + soc_start
                    if add_offset:
                        socinfo_start += board.ram_start
                else:
                    found = False
                    global_partition_offset_addr = board.smem_addr + global_partition_offset_offset
                    uncached_offset_addr = board.smem_addr + uncached_offset_offset
                    if add_offset:
                        global_partition_offset_addr += board.ram_start
                        uncached_offset_addr += board.ram_start
                    global_partition_offset = self.read_int(global_partition_offset_addr, False)
                    if global_partition_offset is None:
                        continue
                    uncached_offset_addr += global_partition_offset
                    uncached_offset = self.read_int(uncached_offset_addr, False)
                    partition_magic_addr = board.smem_addr + global_partition_offset
                    if add_offset:
                        partition_magic_addr += board.ram_start
                    if self.read_int(partition_magic_addr, False) != PARTITION_MAGIC:
                        continue
                    entry_addr = board.smem_addr + global_partition_offset + smem_partition_header_size
                    uncached_end_addr = board.smem_addr + global_partition_offset + uncached_offset
                    if add_offset:
                        entry_addr += board.ram_start
                        uncached_end_addr += board.ram_start
                    while entry_addr < uncached_end_addr:
                        if self.read_u16(entry_addr, False) != SMEM_PRIVATE_CANARY:
                            break
                        if self.read_u16(entry_addr + entry_item_offset, False) == SMEM_HW_SW_BUILD_ID:
                            found = True
                            socinfo_start = entry_addr + smem_private_entry_size
                            break
                        entry_addr += self.read_int(entry_addr + item_size_offset, False)
                        entry_addr += smem_private_entry_size
                    if not found:
                        continue
                socinfo_id = self.read_int(socinfo_start + 4, False)
                if socinfo_id != board.socid:
                    continue

                socinfo_format = self.read_int(socinfo_start, False)
                socinfo_version = self.read_int(socinfo_start + 8, False)
                socinfo_build_id = self.read_cstring(
                    socinfo_start + 12, BUILD_ID_LENGTH, virtual=False)

                chosen_board = board
                break

            if chosen_board is None:
                print_out_str('!!!! Could not find hardware')
                print_out_str("!!!! The SMEM didn't match anything")
                print_out_str(
                    '!!!! You can use --force-hardware to use a specific set of values')
                sys.exit(1)

        else:
            for board in boards:
                if self.hw_id == board.board_num:
                    print_out_str(
                        '!!! Hardware id found! The socinfo values given are bogus')
                    print_out_str('!!! Proceed with caution!')
                    chosen_board = board
                    break
            if chosen_board is None:
                print_out_str(
                    '!!! A bogus hardware id was specified: {0}'.format(self.hw_id))
                print_out_str('!!! Supported ids:')
                ids = get_supported_ids()
                if not len(ids):
                    print_out_str('!!! No registered Boards found - check extensions/board_def.py')
                for b in ids:
                    print_out_str('    {0}'.format(b))
                sys.exit(1)

        print_out_str('\nHardware match: {0}'.format(board.board_num))
        print_out_str('Socinfo id = {0}, version {1:x}.{2:x}'.format(
            socinfo_id, socinfo_version >> 16, socinfo_version & 0xFFFF))
        print_out_str('Socinfo build = {0}'.format(socinfo_build_id))
        print_out_str(
            'Now setting phys_offset to {0:x}'.format(board.phys_offset))
        if board.wdog_addr is not None:
            print_out_str(
            'TZ address: {0:x}'.format(board.wdog_addr))
        if board.phys_offset is not None:
            self.phys_offset = board.phys_offset
        self.tz_addr = board.wdog_addr
        self.ebi_start = board.ram_start
        self.tz_start = board.imem_start
        self.hw_id = board.board_num
        self.cpu_type = board.cpu
        self.imem_fname = board.imem_file_name
        if hasattr(board, 'hyp_diag_addr'):
            self.hyp_diag_addr = board.hyp_diag_addr
        else:
            self.hyp_diag_addr = None
        if hasattr(board, 'rm_debug_addr'):
            self.rm_debug_addr = board.rm_debug_addr
        else:
            self.rm_debug_addr = None
        if hasattr(board, 'tbi_mask'):
            self.tbi_mask = board.tbi_mask
        if hasattr(board, 'kaslr_addr'):
            self.kaslr_addr = board.kaslr_addr
        else:
            self.kaslr_addr = None
        if hasattr(board, 'svm_kaslr_offset'):
            self.svm_kaslr_offset = board.svm_kaslr_offset
        self.board = board
        return True

    def resolve_virt(self, virt_or_name):
        """Takes a virtual address or variable name, returns a virtual
        address
        """
        if not isinstance(virt_or_name, str):
            return virt_or_name
        return self.address_of(virt_or_name)

    def virt_to_phys(self, virt_or_name):
        """Does a virtual-to-physical address lookup of the virtual address or
        variable name."""
        if self.minidump:
            return minidump_util.minidump_virt_to_phys(self.ebi_files_minidump,self.resolve_virt(virt_or_name))
        else:
            return self.mmu.virt_to_phys(self.resolve_virt(virt_or_name))

    def setup_symbol_tables(self):
        args = [self.nm_path, '-n', self.vmlinux]
        p = subprocess.run(args, stdout=subprocess.PIPE)
        symbols = p.stdout.decode().splitlines()
        kaslr = self.get_kaslr_offset()

        # The beginning and ending of kernel image, from vmlinux.lds.S
        _text = self.address_of('_text')
        if _text is None:
            _text = 0

        _end = self.address_of('_end')
        if _end is None:
            _end = 0xFFFFFFFFFFFFFFFF

        for line in symbols:
            s = line.split(' ')
            if len(s) != 3:
                continue
            if (("$x" in s[2].rstrip()) or ("$d" in s[2].rstrip())):
                continue
            entry = (int(s[0], 16) + kaslr, s[2].rstrip(),"")

            # The symbol file contains many artificial symbols which we don't care about.
            if entry[0] < _text or entry[0] >= _end:
                continue

            self.lookup_table.append(entry)

        if not len(self.lookup_table):
            print_out_str('!!! Unable to retrieve symbols... Exiting')
            sys.exit(1)

        if self.dump_kernel_symbol_table:
            self.dump_mod_sym_table('vmlinux', self.lookup_table)

    def retrieve_modules(self):
        mod_list = self.address_of('modules')
        next_offset = self.field_offset('struct list_head', 'next')
        list_offset = self.field_offset('struct module', 'list')
        name_offset = self.field_offset('struct module', 'name')
        if self.is_config_defined('CONFIG_SMP'):
            percpu_offset = self.field_offset('struct module', 'percpu')
            percpu_size_offset = self.field_offset('struct module', 'percpu_size')

        if self.kernel_version >= (6, 4, 0):
            module_core_offset = self.field_offset('struct module', 'mem[0].base')
        elif self.kernel_version > (4, 9, 0):
            module_core_offset = self.field_offset('struct module', 'core_layout.base')
        else:
            module_core_offset = self.field_offset('struct module', 'module_core')

        if self.field_offset('struct module_sect_attr', 'battr') is not None:
            sect_name_offset = self.field_offset('struct module_sect_attr', 'battr') + self.field_offset('struct bin_attribute', 'attr') + self.field_offset('struct attribute', 'name')
        else:
            sect_name_offset = self.field_offset('struct module_sect_attr', 'name')

        kallsyms_offset = self.field_offset('struct module', 'kallsyms')
        sect_addr_offset = self.field_offset('struct module_sect_attr', 'address')
        nsections_offset = self.field_offset('struct module_sect_attrs', 'nsections')
        section_attrs_offset = self.field_offset('struct module_sect_attrs', 'attrs')
        section_attr_size = self.sizeof('struct module_sect_attr')
        mod_sect_attrs_offset = self.field_offset('struct module', 'sect_attrs')
        mod_state_offset = self.field_offset('struct module', 'state')
        mod_attr_grp_name_offest = self.field_offset('struct module_sect_attrs', 'grp') + self.field_offset('struct attribute_group', 'name')
        module_states = self.gdbmi.get_enum_lookup_table('module_state', 5)

        next_list_ent = self.read_pointer(mod_list + next_offset)
        while next_list_ent and next_list_ent != mod_list:
            module = next_list_ent - list_offset

            mod_tbl_ent = module_table.module_table_entry()
            mod_tbl_ent.name = self.read_cstring(module + name_offset)
            state = self.read_u32(module + mod_state_offset)
            if mod_tbl_ent.name is None or state is None or state > len(module_states) or module_states[state] not in ['MODULE_STATE_LIVE']:
                msg = 'module state @{:x}'.format(module)
                if mod_tbl_ent.name:
                    msg += ' [{}]'.format(mod_tbl_ent.name)
                msg += ' is {}'.format(state)
                if state is not None and state < len(module_states):
                    msg += '({})'.format(module_states[state])
                print_out_str(msg)
                next_list_ent = self.read_pointer(next_list_ent + next_offset)
                continue
            mod_tbl_ent.module_offset = self.read_pointer(module + module_core_offset)
            if mod_tbl_ent.module_offset is None:
                mod_tbl_ent.module_offset = 0
            mod_tbl_ent.kallsyms_addr = self.read_pointer(module + kallsyms_offset)
            # Loop through sect_attrs
            mod_tbl_ent.section_offsets = {}
            mod_sect_attrs = self.read_pointer(module + mod_sect_attrs_offset)  # module.sect_attrs
            if self.read_cstring(self.read_pointer(mod_sect_attrs + mod_attr_grp_name_offest))  != 'sections':
                # Observed some ramdumps did not have proper attribute set up yet when module is being loaded.
                # "LIVE" state check not good enough, so add one more sanity check
                print_out_str('Unexpected variation in module section group name, skipping loading sections for {}'.format(mod_tbl_ent.name))
                next_list_ent = self.read_pointer(next_list_ent + next_offset)
                continue
            for i in range(0, self.read_u32(mod_sect_attrs + nsections_offset)):
                # attr_ptr = module.sect_attrs.attrs[i]
                attr_ptr = mod_sect_attrs + section_attrs_offset + (i * section_attr_size)
                # sect_name = attr_ptr.battr.attr.name (for 5.4+)
                sect_name = self.read_cstring(self.read_pointer(attr_ptr + sect_name_offset))
                # sect_addr = attr_ptr.address
                sect_addr = self.read_word(attr_ptr + sect_addr_offset)
                # https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/scripts/gdb/linux/symbols.py?h=v5.14#n102
                if sect_name not in ['.data', '.data..read_mostly', '.rodata', '.bss',
                                     '.text', '.text.bss', '.text.hot', '.text.unlikely']:
                    continue
                mod_tbl_ent.section_offsets[sect_name] = sect_addr
            if self.is_config_defined('CONFIG_SMP'):
                percpu_size = self.read_u32(module + percpu_size_offset)
                if percpu_size is not 0:
                    percpu_pointer = self.read_pointer(module + percpu_offset)
                    mod_tbl_ent.section_offsets['.data..percpu'] = percpu_pointer
            self.module_table.add_entry(mod_tbl_ent)

            next_list_ent = self.read_pointer(next_list_ent + next_offset)

    def retrieve_minidump_modules(self):
        kmodules_seg = next((s for s in self.elffile.iter_sections() if s.name == 'KMODULES'), None)
        if kmodules_seg is None:
            return

        kmodules_lines = self.read_cstring(kmodules_seg['sh_addr'], max_length=kmodules_seg['sh_size'])
        for line in kmodules_lines.splitlines():
            m = re.fullmatch(r"^name: (.+), base: (?:0x)?([0-9a-fA-F]+)\b.*", line)
            if m is not None:
                mod_tbl_ent = module_table.module_table_entry()
                mod_tbl_ent.name = m.group(1)
                mod_tbl_ent.module_offset = int(m.group(2), base=16)
                n = re.search(r"\.bss: (?:0x)?([0-9a-fA-F]+).*", line)
                if n is not None:
                    mod_tbl_ent.section_offsets['.bss'] = int(n.group(1), base=16)
                n = re.search(r"\.data: (?:0x)?([0-9a-fA-F]+).*", line)
                if n is not None:
                    mod_tbl_ent.section_offsets['.data'] = int(n.group(1), base=16)
                self.module_table.add_entry(mod_tbl_ent)

    def parse_symbols_of_one_module(self, mod_tbl_ent, ko_file_list):
        name_index = [s for s in ko_file_list.keys() if mod_tbl_ent.name in s]
        if len(name_index) == 0:
            print_out_str('!! Object not found for {}'.format(mod_tbl_ent.name))
            return

        if mod_tbl_ent.name not in ko_file_list and name_index[0] in ko_file_list:
            temp_data = ko_file_list[name_index[0]]
            del ko_file_list[name_index[0]]
            ko_file_list[mod_tbl_ent.name] = temp_data
        if not mod_tbl_ent.set_sym_path(ko_file_list[mod_tbl_ent.name]):
            return

        if self.is_config_defined("CONFIG_KALLSYMS") and not self.minidump:
            symtab_offset = self.field_offset('struct mod_kallsyms', 'symtab')
            num_symtab_offset = self.field_offset('struct mod_kallsyms', 'num_symtab')
            strtab_offset = self.field_offset('struct mod_kallsyms', 'strtab')

            if self.arm64:
                sym_struct_name = 'struct elf64_sym'
                sym_struct_size = self.sizeof(sym_struct_name)
            else:
                sym_struct_name = 'struct elf32_sym'
                sym_struct_size = self.sizeof(sym_struct_name)

            st_info_offset = self.field_offset(sym_struct_name, 'st_info')
            symtab = self.read_pointer(mod_tbl_ent.kallsyms_addr + symtab_offset)
            num_symtab = self.read_pointer(mod_tbl_ent.kallsyms_addr + num_symtab_offset)
            strtab = self.read_pointer(mod_tbl_ent.kallsyms_addr + strtab_offset)

            if symtab is None or num_symtab is None or strtab is None:
                return

            KSYM_NAME_LEN = 128
            for i in range(0, num_symtab):
                elf_sym = symtab + sym_struct_size * i
                st_value = self.read_structure_field(elf_sym, sym_struct_name, 'st_value')
                st_info = self.read_byte(elf_sym + st_info_offset)
                sym_type = chr(st_info)
                st_name = self.read_structure_field(elf_sym, sym_struct_name, 'st_name')
                sym_addr = st_value
                sym_name = self.read_cstring(strtab + st_name, KSYM_NAME_LEN)
                st_shndx = self.read_structure_field(elf_sym, sym_struct_name, 'st_shndx')
                st_size = self.read_structure_field(elf_sym, sym_struct_name, 'st_size')

                ###
                # FORMAT of record:
                # sym_addr, syn_name[mod_name], sym_type, idx_elf_sym, st_name, st_shndx, st_size
                ###
                if (sym_name is None or mod_tbl_ent.name is None):
                    continue
                if sym_addr:
                    # when sym_addr is 0, it means the symbol is undefined
                    # will not add undefined symbols here to avoid address 0x0
                    # being treated as belonging to a particular kernel module
                    mod_tbl_ent.kallsyms_table.append(
                        (sym_addr, sym_name + '[' + mod_tbl_ent.name + ']', sym_type, i,
                         st_name, st_shndx, st_size,sym_name))
            mod_tbl_ent.kallsyms_table.sort()
            if self.dump_module_kallsyms:
                self.dump_mod_kallsyms_sym_table(mod_tbl_ent.name, mod_tbl_ent.kallsyms_table)
        else:
            args = [self.nm_path, '-n', mod_tbl_ent.get_sym_path()]
            p = subprocess.run(args, stdout=subprocess.PIPE)
            symbols = p.stdout.decode().splitlines()
            for line in symbols:
                s = line.split(' ')
                if len(s) == 3:
                    mod_tbl_ent.sym_lookup_table.append(
                        (int(s[0], 16) + mod_tbl_ent.module_offset,
                        s[2].rstrip() + '[' + mod_tbl_ent.name + ']'))
            mod_tbl_ent.sym_lookup_table.sort()
            if self.dump_module_symbol_table:
                self.dump_mod_sym_table(mod_tbl_ent.name, mod_tbl_ent.sym_lookup_table)

    def walk_depth(self, path, on_file, depth=10):
        if depth <= 0:
            return
        for basename in os.listdir(path):
            file = os.path.join(path, basename)
            if os.path.isdir(file) and not os.path.islink(file):
                self.walk_depth(file, on_file, depth=depth-1)
            elif os.path.isfile(file):
                on_file(file)

    def parse_module_symbols(self):
        # Recursively search all files under mod_path ending in '.ko.unstripped' and store in a list
        ko_file_list = {}
        for path in self.module_table.sym_path_list:
            def on_file(file):
                if file.endswith('.ko.unstripped'):
                    name = file[:-len('.ko.unstripped')]
                elif file.endswith('.ko'):
                    name = file[:-len('.ko')]
                else:
                    return
                name = os.path.basename(name)
                name = name.replace("-","_")
                # Prefer .ko.unstripped
                if ko_file_list.get(name, '').endswith('.ko.unstripped') and file.endswith('.ko'):
                    return
                ko_file_list[name] = file
                self.ko_file_names.append(name)
            self.walk_depth(path, on_file)

        for mod_tbl_ent in self.module_table.module_table:
            if mod_tbl_ent.name is None:
                print_out_str('!! Object name not extracted properly..checking next!!')
                continue
            self.parse_symbols_of_one_module(mod_tbl_ent, ko_file_list)

    def add_symbols_to_global_lookup_table(self):
        if self.is_config_defined("CONFIG_KALLSYMS") and not self.minidump:
            for mod_tbl_ent in self.module_table.module_table:
                for sym in mod_tbl_ent.kallsyms_table:
                    if sym[1].startswith('$x') or sym[1].startswith('$d'):
                        continue
                    self.lookup_table.append((sym[0], sym[1],sym[7]))
        else:
            for mod_tbl_ent in self.module_table.module_table:
                for sym in mod_tbl_ent.sym_lookup_table:
                    self.lookup_table.append(sym)
        self.lookup_table.sort()

    def setup_module_symbols(self):
        if self.minidump:
            self.retrieve_minidump_modules()
        else:
            self.retrieve_modules()
        self.parse_module_symbols();
        self.add_symbols_to_global_lookup_table()

    def dump_mod_sym_table(self, mod_name, sym_lookup_tbl):
        sym_dump_file = self.open_file('sym_tbl_'+mod_name+'.txt')
        for line in sym_lookup_tbl:
            sym_dump_file.write('0x{0:x} {1}\n'.format(line[0], line[1]))
        sym_dump_file.close()

    def dump_mod_kallsyms_sym_table(self, mod_name, mod_kallsyms_table):
        kallsyms_header_format = '{0: >18} {1} {2: >64} {3} {4} {5} {6}\n'
        kallsyms_record_format = '0x{0:0>16x} {1: >8} {2: >64} {3: >11} {4: >7} {5: >8} {6: >7}\n'
        kallsyms_file = self.open_file('sym_tbl_kallsyms_'+mod_name+'.txt')
        kallsyms_file.write('KALLSYMS symbol lookup table['+mod_name+']\n')
        kallsyms_file.write(
            kallsyms_header_format.format(
                'sym_addr', 'sym_type', 'syn_name[mod_name]', 'idx_elf_sym',
                'st_name', 'st_shndx', 'st_size'))
        for mod_sym_line in mod_kallsyms_table:
            kallsyms_file.write(
                kallsyms_record_format.format(
                    mod_sym_line[0], mod_sym_line[2], mod_sym_line[1], mod_sym_line[3],
                    hex(mod_sym_line[4]), mod_sym_line[5], mod_sym_line[6]))
        kallsyms_file.close()

    def dump_global_symbol_lookup_table(self):
        sym_dump_file = self.open_file('sym_table.txt')
        for line in self.lookup_table:
            sym_dump_file.write('0x{0:x} {1}\n'.format(line[0], line[1]))
        sym_dump_file.close()

    def address_of(self, symbol):
        """Returns the address of a symbol.

        :param symbol: name of the symbol.
        :type symbol: str

        :return: address value

        Example:

        >>> hex(dump.address_of('linux_banner'))
        '0xffffffc000c7a0a8L'
        """
        try:
            return self.gdbmi.address_of(symbol)
        except gdbmi.GdbMIException:
            if self.hyp:
                try:
                    return self.gdbmi_hyp.address_of(symbol)
                except gdbmi.GdbMIException:
                    pass

    def symbol_at(self, addr):
        """
        Function to return symbol using gdbmi.

        :param addr: address value.
        :type addr: int

        :return: symbol value
        """
        try:
            return self.gdbmi.symbol_at(addr)
        except gdbmi.GdbMIException:
            if self.hyp:
                try:
                    return self.gdbmi_hyp.symbol_at(addr)
                except gdbmi.GdbMIException:
                    pass

    def sizeof(self, the_type):
        try:
            return self.gdbmi.sizeof(the_type)
        except gdbmi.GdbMIException:
            if self.hyp:
                try:
                    return self.gdbmi_hyp.sizeof(the_type)
                except gdbmi.GdbMIException:
                    pass

    def array_index(self, addr, the_type, index):
        """Index into the array of type ``the_type`` located at ``addr``.

        I.e., given::

            int my_arr[3];
            my_arr[2] = 42;

        You could do the following:

        >>> addr = dump.address_of("my_arr")
        >>> dump.read_word(dump.array_index(addr, "int", 2))
        42
        """
        offset = self.gdbmi.sizeof(the_type) * index
        return addr + offset

    def frame_field_offset(self, frame_name, the_type, field):
        try:
            return self.gdbmi.frame_field_offset(frame_name, the_type, field)
        except gdbmi.GdbMIException:
            if self.hyp:
                try:
                    return self.gdbmi_hyp.frame_field_offset(frame_name, the_type, field)
                except gdbmi.GdbMIException:
                    pass

    def get_symbol_info1(self,addr):
        kaslr = self.get_kaslr_offset()
        if kaslr:
            addr1 = addr - kaslr
        else:
            addr1 = addr
        #print "hex of address in get_symbol_info1 {0}".format(hex(addr1))
        addr1, desc = self.step_through_jump_table(addr1)
        symbol_obj =  self.gdbmi.get_symbol_info(addr1)
        module = symbol_obj.section.split('\\\\')[-1]
        if self.minidump:
            if module == 'vmlinux':
                return symbol_obj.symbol + desc + " " + str(symbol_obj.offset)
            else:
                return symbol_obj.symbol + desc + " " + str(symbol_obj.offset) + " [" + module + "]"
        return symbol_obj.symbol + desc

    def type_of(self, symbol):
        """
            this will be helpful to get the type of symbol.
            eg :
            >>>dump.type_of("kgsl_driver")
            struct kgsl_driver

        """
        try:
            return self.gdbmi.type_of(symbol)
        except gdbmi.GdbMIException:
            pass

    def field_offset(self, the_type, field):
        """Gets the offset of a field from the base of its containing struct.

        This can be useful when reading struct fields, although you should
        consider using :func:`~read_structure_field` if
        you're reading a word-sized value.

        Example:

        >>> dump.field_offset('struct device', 'bus')
        168
        """
        try:
            return self.gdbmi.field_offset(the_type, field)
        except gdbmi.GdbMIException:
            if self.hyp:
                try:
                    return self.gdbmi_hyp.field_offset(the_type, field)
                except gdbmi.GdbMIException:
                    pass

    def container_of(self, ptr, the_type, member):
        """Like ``container_of`` in the kernel."""
        try:
            return self.gdbmi.container_of(ptr, the_type, member)
        except gdbmi.GdbMIException:
            if self.hyp:
                try:
                    return self.gdbmi_hyp.container_of(ptr, the_type, member)
                except gdbmi.GdbMIException:
                    pass

    def sibling_field_addr(self, ptr, parent_type, member, sibling):
        """Gets the address of a sibling structure field.

        Given the address of some field within a structure, returns the
        address of the requested sibling field.
        """
        try:
            return self.gdbmi.sibling_field_addr(ptr, parent_type, member, sibling)
        except gdbmi.GdbMIException:
            if self.hyp:
                try:
                    return self.gdbmi_hyp.sibling_field_addr(ptr, parent_type, member, sibling)
                except gdbmi.GdbMIException:
                    pass

    def step_through_jump_table(self, addr):
        """
        Steps through a jump table, if the address points to a unconditional branch
        """

        if addr is None:
            return addr, ''

        fn_addr = addr
        if self.is_config_defined('CONFIG_ARM64_BTI_KERNEL'):
            # Skip past BTI instruction to the real branch instr
            fn_addr += 4
        if self.cpu_type in ['ARMV9-A', 'CORTEXA53']:
            instr = self.read_u32(fn_addr)
            if instr is None or (instr & 0xFC000000) != 0x14000000:
                return addr, ''
            imm26_mask = 0x3FFFFFF
            offset = instr & imm26_mask
            if (offset & imm26_mask) >> 25:
                offset -= (imm26_mask + 1)
            fn_addr += 4 * offset
            return fn_addr, '[jt]'
        return addr, ''

    def unwind_lookup(self, addr, symbol_size=0):
        """
        Returns closest symbols <= addr and either the relative offset
        or the symbol size.

        The loop constant is:
        table[low] <= addr <= table[high]
        """

        addr = self.pac_ignore(addr)
        table = self.lookup_table
        low = 0
        high = len(self.lookup_table) - 1

        addr, desc = self.step_through_jump_table(addr)

        if self.minidump:
            symbol_str = self.get_symbol_info1(addr)
            words = symbol_str.split(" ")
            symbol = words[0]
            offset = words[1]
            if len(words) == 3:
                module = words[2]
                return (symbol + ' ' + module, int(offset))
            return (symbol, int(offset))

        if addr is None or addr < table[low][0] or addr > table[high][0]:
            return None

        while(True):
            # Python now complains about division producing floats
            mid = (high + low) >> 1

            if mid == low or mid == high:
                break

            if addr <= table[mid][0]:
                high = mid
            elif addr >= table[mid][0]:
                low = mid

        if addr == table[low][0]:
            high = low
        elif addr == table[high][0]:
            low = high

        offset = addr - table[low][0]
        #how to calculate size for the last symbol?
        if low == len(self.lookup_table) - 1:
            size = 0
        else:
            size = table[low + 1][0] - table[low][0]

        if symbol_size == 0:
            return (table[low][1] + desc, offset)
        else:
            return (table[low][1] + desc, size)

    def read_elf_memory(self, addr, length, temp_file):
        s = self.gdbmi.read_elf_memory(addr, length, temp_file)
        if s is not None:
            a = s.decode('ascii', 'ignore')
            return a.split('\0')[0]
        else:
            return s

    def read_physical(self, addr, length):
        if not isinstance(addr, int) or not isinstance(length, int):
            return None

        if self.minidump:
            addr_data = minidump_util.read_physical_minidump(
                        self.ebi_files_minidump, self.ebi_files,self.elffile,
                        addr, length)
            return addr_data

        elif self.reduceddump:
            data = elfutil.read_physical(self.elf_vector, self.elf_htable,
                                            self.elf_filemap, self.ebi_files,
                                            addr, length)
            #if addr == 0x83cc09268:
            #    print("addr read yielded none : {:x}".format(addr))
            return data

        else:
            ebi = (-1, -1, -1)
            for a in self.ebi_files:
                fd, start, end, path = a
                if addr >= start and addr <= end:
                    ebi = a
                    break
            if ebi[0] == -1:
                return None
            offset = addr - ebi[1]
            ebi[0].seek(offset)
            a = ebi[0].read(length)
            return a

    def read_dword(self, addr_or_name, virtual=True, cpu=None, allow_elf=False):
        s = self.read_string(addr_or_name, '<Q', virtual, cpu, allow_elf)
        return s[0] if s is not None else None

    def read_word(self, addr_or_name, virtual=True, cpu=None, allow_elf=False):
        """returns a word size (pointer) read from ramdump"""
        if self.arm64:
            s = self.read_string(addr_or_name, '<Q', virtual, cpu, allow_elf)
        else:
            s = self.read_string(addr_or_name, '<I', virtual, cpu, allow_elf)
        return s[0] if s is not None else None

    def read_halfword(self, addr_or_name, virtual=True, cpu=None, allow_elf=False):
        """returns a value corresponding to half the word size"""
        if self.arm64:
            s = self.read_string(addr_or_name, '<I', virtual, cpu, allow_elf)
        else:
            s = self.read_string(addr_or_name, '<H', virtual, cpu, allow_elf)
        return s[0] if s is not None else None

    def read_slong(self, addr_or_name, virtual=True, cpu=None, allow_elf=False):
        """returns a value corresponding to half the word size"""
        if self.arm64:
            s = self.read_string(addr_or_name, '<q', virtual, cpu, allow_elf)
        else:
            s = self.read_string(addr_or_name, '<i', virtual, cpu, allow_elf)
        return s[0] if s is not None else None

    def read_ulong(self, addr_or_name, virtual=True, cpu=None, allow_elf=False):
        """returns a value corresponding to half the word size"""
        if self.arm64:
            s = self.read_string(addr_or_name, '<Q', virtual, cpu, allow_elf)
        else:
            s = self.read_string(addr_or_name, '<I', virtual, cpu, allow_elf)
        return s[0] if s is not None else None

    def read_byte(self, addr_or_name, virtual=True, cpu=None, allow_elf=False):
        """Reads a single byte."""
        s = self.read_string(addr_or_name, '<B', virtual, cpu, allow_elf)
        return s[0] if s is not None else None

    def read_bool(self, addr_or_name, virtual=True, cpu=None, allow_elf=False):
        """Reads a bool."""
        s = self.read_string(addr_or_name, '<?', virtual, cpu, allow_elf)
        return s[0] if s is not None else None

    def read_s64(self, addr_or_name, virtual=True, cpu=None, allow_elf=False):
        """returns a value guaranteed to be 64 bits"""
        s = self.read_string(addr_or_name, '<q', virtual, cpu, allow_elf)
        return s[0] if s is not None else None

    def read_u64(self, addr_or_name, virtual=True, cpu=None, allow_elf=False):
        """returns a value guaranteed to be 64 bits"""
        s = self.read_string(addr_or_name, '<Q', virtual, cpu, allow_elf)
        return s[0] if s is not None else None

    def read_s32(self, addr_or_name, virtual=True, cpu=None, allow_elf=False):
        """returns a value guaranteed to be 32 bits"""
        s = self.read_string(addr_or_name, '<i', virtual, cpu, allow_elf)
        return s[0] if s is not None else None

    def read_u32(self, addr_or_name, virtual=True, cpu=None, allow_elf=False):
        """returns a value guaranteed to be 32 bits"""
        s = self.read_string(addr_or_name, '<I', virtual, cpu, allow_elf)
        return s[0] if s is not None else None

    def read_int(self, addr_or_name, virtual=True,  cpu=None, allow_elf=False):
        """Alias for :func:`~read_u32`"""
        return self.read_u32(addr_or_name, virtual, cpu, allow_elf)

    def read_u16(self, addr_or_name, virtual=True, cpu=None, allow_elf=False):
        """returns a value guaranteed to be 16 bits"""
        s = self.read_string(addr_or_name, '<H', virtual, cpu, allow_elf)
        return s[0] if s is not None else None

    def read_pointer(self, addr_or_name, virtual=True, cpu=None, allow_elf=False):
        """Reads ``addr_or_name`` as a pointer variable.

        The read length is either 32-bit or 64-bit depending on the
        architecture.  This returns the *value* of the pointer variable
        (i.e. the address it contains), not the data it points to.
        """
        fn = self.read_u32 if self.sizeof('void *') == 4 else self.read_u64
        return fn(addr_or_name, virtual, cpu)

    def struct_field_addr(self, addr, the_type, field):
        try:
            return self.gdbmi.field_offset(the_type, field) + addr
        except gdbmi.GdbMIException:
            pass

    def read_structure_field(self, addr_or_name, struct_name, field, virtual=True):
        """reads a 4 or 8 byte field from a structure"""
        size = self.sizeof("(({0} *)0)->{1}".format(struct_name, field))
        addr = self.resolve_virt(addr_or_name)
        if addr is None or size is None:
            return None

        addr += self.field_offset(struct_name, field)
        if size == 2:
            return self.read_u16(addr, virtual)
        if size == 4:
            return self.read_u32(addr, virtual)
        if size == 8:
            return self.read_u64(addr, virtual)
        return None

    def read(self, cmd):
        """Reads the value of a C-style expression provided it doesn't have a
        pointer.

        Supported syntax for cmd: dump.read('device_3d0.dev.open_count')
        """
        size = self.sizeof("{0}".format(cmd))
        addr = self.address_of(cmd)

        if addr is None or size is None:
            return None

        if size == 2:
            return self.read_u16(addr)
        if size == 4:
            return self.read_u32(addr)
        if size == 8:
            return self.read_u64(addr)
        if size != 0 and size != 4 and size != 8:
            return addr
        return None

    def read_structure_cstring(self, addr_or_name, struct_name, field,
                               max_length=100):
        """reads a C string from a structure field.  The C string field will be
        dereferenced before reading, so it should be a ``char *``, not a
        ``char []``.
        """
        virt = self.resolve_virt(addr_or_name)
        cstring_addr = virt + self.field_offset(struct_name, field)
        return self.read_cstring(self.read_pointer(cstring_addr), max_length)

    def read_cstring(self, addr_or_name, max_length=100, virtual=True,
                     cpu=None, allow_elf=False):
        """Reads a C string."""
        addr = addr_or_name
        s = None
        if virtual:
            if cpu is not None:
                pcpu_offset = self.per_cpu_offset(cpu)
                addr_or_name = self.resolve_virt(addr_or_name)
                addr_or_name += pcpu_offset + self.per_cpu_offset(cpu)
            addr = self.virt_to_phys(addr_or_name)
            if allow_elf and addr is None:
                s = self.gdbmi.read_memory(addr_or_name, '{}+{}'.format(addr_or_name, max_length))
        if not s:
            s = self.read_physical(addr, max_length)
        if s is not None:
            a = s.decode('ascii', 'ignore')
            return a.split('\0')[0]
        else:
            return s

    def read_binarystring(self, addr_or_name, length, virtual=True, cpu=None, allow_elf=False):
        """Reads binary data of specified length from addr_or_name."""
        addr = addr_or_name
        s = None
        if virtual:
            if cpu is not None:
                pcpu_offset = self.per_cpu_offset(cpu)
                addr_or_name = self.resolve_virt(addr_or_name)
                addr_or_name += pcpu_offset
            addr = self.virt_to_phys(addr_or_name)
            if allow_elf and addr is None:
                s = self.gdbmi.read_memory(addr_or_name, '{}+{}'.format(addr_or_name, length))
        if not s:
            s = self.read_physical(addr, length)
        return s

    def read_string(self, addr_or_name, format_string, virtual=True, cpu=None, allow_elf=False):
        """Reads data using a format string.

        Reads data from addr_or_name using format_string (which should be a
        struct.unpack format).

        Returns the tuple returned by struct.unpack.
        """
        addr = addr_or_name
        per_cpu_string = ''
        s = None
        if virtual:
            if cpu is not None:
                pcpu_offset = self.per_cpu_offset(cpu)
                addr_or_name = self.resolve_virt(addr_or_name)
                addr_or_name += pcpu_offset
                per_cpu_string = ' with per-cpu offset of ' + hex(pcpu_offset)
            addr = self.virt_to_phys(addr_or_name)
            if allow_elf and addr is None:
                s = self.gdbmi.read_memory(addr_or_name, '{}+{}'.format(addr_or_name, struct.calcsize(format_string)))
        if not s:
            s = self.read_physical(addr, struct.calcsize(format_string))
        if (s is None) or (s == ''):
            return None
        return struct.unpack(format_string, s)

    def hexdump(self, addr_or_name, length, virtual=True, file_object=None, little_endian=True):
        """Returns a string with a hexdump (in the format of ``xxd``).

        ``length`` is in bytes.

        Example (intentionally not in doctest format since it would require
        a specific dump to be loaded to pass as a doctest):

        >>> print(dump.hexdump('linux_banner', 0x80))
        ffffffc000c610a8: 4c69 6e75 7820 7665 7273 696f 6e20 332e  Linux version 3.
        ffffffc000c610b8: 3138 2e32 302d 6761 3762 3238 6539 2d31  18.20-ga7b28e9-1
        ffffffc000c610c8: 3333 3830 2d67 3036 3032 6531 3020 286c  3380-g0602e10 (l
        ffffffc000c610d8: 6e78 6275 696c 6440 6162 6169 7431 3532  nxbuild@abait152
        ffffffc000c610e8: 2d73 642d 6c6e 7829 2028 6763 6320 7665  -sd-lnx) (gcc ve
        ffffffc000c610f8: 7273 696f 6e20 342e 392e 782d 676f 6f67  rsion 4.9.x-goog
        ffffffc000c61108: 6c65 2032 3031 3430 3832 3720 2870 7265  le 20140827 (pre
        ffffffc000c61118: 7265 6c65 6173 6529 2028 4743 4329 2029  release) (GCC) )

        If little_endian = False, each 4 byte chunk will be printed in big endian form, like how
        Trace32 displays memory. The string below looks jumbled but this format is useful when
        decoding USB TRB rings, for instance.
        Ex:

        >>> print(dump.hexdump('linux_banner', 0x80, little_endian=False))
        ffffffe03b562920: 756e 694c 6576 2078 6f69 7372 2e35 206e  uniLev xoisr.5 n
        ffffffe03b562930: 372e 3531 6b71 2d38 6f63 2d69 6c6f 736e  7.51kq-8oc-ilosn
        ffffffe03b562940: 7461 6469 6e61 2d65 696f 7264 2d33 3164  tadina-eiord-31d
        ffffffe03b562950: 6667 2d38 3934 3035 6632 3139 2034 3731  fg-89405f219 471
        ffffffe03b562960: 6975 6228 752d 646c 4072 6573 6c69 7562  iub(u-dl@resliub
        ffffffe03b562970: 6f68 2d64 2029 7473 646e 4128 6469 6f72  oh-d )tsdnA(dior
        ffffffe03b562980: 3538 2820 3036 3830 6220 2c38 6465 7361  58( 0680b ,8desa
        ffffffe03b562990: 206e 6f20 3035 3472 6534 3837 6c63 2029   no 054re487lc )
        """
        from io import StringIO
        sio = StringIO()
        address = self.resolve_virt(addr_or_name)

        if little_endian:
            parser_util.xxd(
                address,
                [self.read_byte(address + i, virtual=virtual) or 0
                 for i in range(length)],
                file_object=sio)
        else:
            places = []
            for i in range(int(length / 4)):
                places.extend([(i + 1) * 4 - 1, (i + 1) * 4 - 2, (i + 1) * 4 - 3, i * 4])
            parser_util.xxd(
                address,
                [self.read_byte(address + i, virtual=virtual) or 0
                 for i in places],
                file_object=sio)

        ret = sio.getvalue()
        sio.close()
        return ret

    def get_read_physical_offset(self, addr):
        if not self.reduceddump:
            offset = None
            input = None
            for file in self.ebi_files:
                fd, start, end, path = file
                if addr >= start and addr <= end:
                    input = path
                    offset = addr - start
                    break
            return offset, input
        return elfutil.get_read_physical_offset_helper(self.elf_vector, self.elf_htable,
                                            self.elf_filemap, self.ebi_files,
                                            addr)

    def per_cpu_offset(self, cpu):
        """ __per_cpu_offset has been observed to be a negative number
        on kernel 5.4, even though it is stored in a unsigned long.
        Since python supports numbers larger than 64 bits, the behavior
        on overflow differs with that of C. Therefore, read it as a
        signed value instead. """

        per_cpu_offset_addr = self.address_of('__per_cpu_offset')
        if per_cpu_offset_addr is None:
            return 0
        per_cpu_offset_addr_indexed = self.array_index(
            per_cpu_offset_addr, 'unsigned long', cpu)
        return self.read_slong(per_cpu_offset_addr_indexed)


    def set_available_cores(self):
        """set available core numbers in the system."""
        major, minor, patch = self.kernel_version
        cpu_present_bits_addr = self.address_of('cpu_present_bits')
        cpu_present_bits = self.read_word(cpu_present_bits_addr)
        ind = 0
        if (major, minor) >= (4, 5):
            cpu_present_bits_addr = self.address_of('__cpu_present_mask')
            bits_offset = self.field_offset('struct cpumask', 'bits')
            cpu_present_bits = self.read_word(cpu_present_bits_addr + bits_offset)
        self.available_cores.clear()
        while cpu_present_bits:
            if cpu_present_bits & 1:
                self.available_cores.append(ind)
            cpu_present_bits = cpu_present_bits >> 1
            ind += 1


    def get_num_cpus(self):
        """Gets the number of CPUs in the system."""
        if not(len(self.available_cores)):
            self.set_available_cores()
        return len(self.available_cores)

    def iter_cpus(self):
        """Returns an iterator over all CPUs in the system.

        Example:

        >>> list(dump.iter_cpus())
        [0, 1, 2, 3]
        """
        return (self.available_cores)

    def is_thread_info_in_task(self):
        return self.is_config_defined('CONFIG_THREAD_INFO_IN_TASK')

    def get_thread_info_addr(self, task_addr):
        if self.is_thread_info_in_task():
            thread_info_address = task_addr + self.field_offset('struct task_struct', 'thread_info')
        else:
            thread_info_ptr = task_addr + self.field_offset('struct task_struct', 'stack')
            thread_info_address = self.read_word(thread_info_ptr, True)
        return thread_info_address

    def get_task_cpu(self, task_struct_addr, thread_info_struct_addr):
        if self.is_thread_info_in_task() and self.get_kernel_version() < (5, 19, 0):
            offset_cpu = self.field_offset('struct task_struct', 'cpu')
            cpu = self.read_int(task_struct_addr + offset_cpu)
        else:
            offset_cpu = self.field_offset('struct thread_info', 'cpu')
            cpu = self.read_int(thread_info_struct_addr + offset_cpu)
        return cpu

    def thread_saved_field_common_32(self, task, reg_offset):
        thread_info = self.get_thread_info_addr(task)
        cpu_context_offset = self.field_offset('struct thread_info', 'cpu_context')
        val = self.read_word(thread_info + cpu_context_offset + reg_offset)
        return val

    def thread_saved_field_common_64(self, task, reg_offset):
        thread_offset = self.field_offset('struct task_struct', 'thread')
        cpu_context_offset = self.field_offset('struct thread_struct', 'cpu_context')
        val = self.read_word(task + thread_offset + cpu_context_offset + reg_offset)
        return val

    def thread_saved_pc(self, task):
        if self.arm64:
            return self.thread_saved_field_common_64(task, self.field_offset('struct cpu_context', 'pc'))
        else:
            return self.thread_saved_field_common_32(task, self.field_offset('struct cpu_context_save', 'pc'))

    def thread_saved_sp(self, task):
        if self.arm64:
            return self.thread_saved_field_common_64(task, self.field_offset('struct cpu_context', 'sp'))
        else:
            return self.thread_saved_field_common_32(task, self.field_offset('struct cpu_context_save', 'sp'))

    def thread_saved_fp(self, task):
        if self.arm64:
            return self.thread_saved_field_common_64(task, self.field_offset('struct cpu_context', 'fp'))
        else:
            pc  = self.thread_saved_pc(task)
            # PC value last bit is 1 for Thumb and 0 for ARM
            if (pc & 0x1):
                return self.thread_saved_field_common_32(task, self.field_offset('struct cpu_context_save', 'r7'))
            return self.thread_saved_field_common_32(task, self.field_offset('struct cpu_context_save', 'fp'))

    def for_each_process(self):
        """ create a generator for traversing through each valid process"""
        init_task = self.address_of('init_task')
        tasks_offset = self.field_offset('struct task_struct', 'tasks')
        prev_offset = self.field_offset('struct list_head', 'prev')
        next = init_task
        seen_tasks = []

        while (1):
            task_pointer = self.read_word(next + tasks_offset, True)
            if not task_pointer:
                break

            task_struct = task_pointer - tasks_offset
            if ((self.validate_task_struct(task_struct) == -1) or (
                    self.validate_sched_class(task_struct) == -1)):
                next = init_task
                while (1):
                    task_pointer = self.read_word(next + tasks_offset +
                                                  prev_offset, True)

                    if not task_pointer:
                        break
                    task_struct = task_pointer - tasks_offset
                    if (self.validate_task_struct(task_struct) == -1):
                        break
                    if (self.validate_sched_class(task_struct) == -1):
                        break
                    if task_struct in seen_tasks:
                        break

                    yield task_struct
                    seen_tasks.append(task_struct)
                    next = task_struct
                    if (next == init_task):
                        break
                break

            if task_struct in seen_tasks:
                break
            yield task_struct
            seen_tasks.append(task_struct)
            next = task_struct
            if (next == init_task):
                break

    '''
    task_struct->signal->thread_head
                struct list_head {
                    struct list_head *next; -->task_struct->thread_node
                    struct list_head *prev; -->task_struct->thread_node
                } thread_head;
    '''
    def for_each_thread(self, task_addr):
        offset_thread_node =self.field_offset(
            'struct task_struct', 'thread_node')
        offset_signal = self.field_offset(
            'struct task_struct', 'signal')
        offset_thread_head = self.field_offset(
            'struct signal_struct', 'thread_head')
        signal_addr = self.read_word(task_addr + offset_signal)
        thread_head_addr = self.read_word(signal_addr + offset_thread_head)
        next_thread_head = thread_head_addr
        seen_threads = []
        while True:
            task_addr = next_thread_head - offset_thread_node
            if (self.validate_task_struct(task_addr) == -1) or (
                    self.validate_sched_class(task_addr) == -1):
                    break

            yield task_addr

            next_thr = self.read_word(next_thread_head)
            if (next_thr == next_thread_head) and (next_thr != thread_head_addr):
                print_out_str('!!!! Cycle in thread group! The list is corrupt!\n')
                break

            if (next_thr in seen_threads):
                break

            seen_threads.append(next_thr)
            next_thread_head = next_thr
            if next_thread_head == thread_head_addr:
                break

    def validate_task_struct(self, task):
        thread_info_address = self.get_thread_info_addr(task)
        if self.is_thread_info_in_task():
            task_struct = task
        else:
            task_address = thread_info_address + self.field_offset(
                                    'struct thread_info', 'task')
            task_struct = self.read_word(task_address, True)

        cpu_number = self.get_task_cpu(task_struct, thread_info_address)
        if ((task != task_struct) or (thread_info_address == 0x0)):
            return -1
        if ((cpu_number < 0) or (cpu_number > self.get_num_cpus())):
            return -1

    def validate_sched_class(self, task):
        sc_top = self.address_of('stop_sched_class')
        sc_rt = self.address_of('rt_sched_class')
        sc_idle = self.address_of('idle_sched_class')
        sc_fair = self.address_of('fair_sched_class')

        sched_class = self.read_structure_field(
                                task, 'struct task_struct', 'sched_class')

        if not ((sched_class == sc_top) or (sched_class == sc_rt) or (
                sched_class == sc_idle) or (sched_class == sc_fair)):
            return -1

    def __ignore_storage_class(self, line):
        line_split = line.split()
        result_words = [word for word in line_split if word.lower() not in storage_classes]
        return ' '.join(result_words)

    def __ignore_expanded_pointer(self, text, d_type):
        if '} *' in text[-1].lstrip():
            name_re = re.search(r"type = ([a-zA-Z0-9_ ]+){", text[0])
            if name_re:
                name = name_re.group(1) + "*"
                return name
        return d_type

    def __is_primary_type(self, d_type):
        d_type = d_type.rstrip()
        if d_type[-1] == "]":
            re_obj = re.search("(.*)\[\d+\]",d_type)
            d_type = re_obj.group(1)
        if "*" in d_type or "enum " in d_type or d_type == "enum":
            return True
        if d_type.lstrip().rstrip() in primary_types:
            return True
        else:
            return False

    def __create_object(self, text, base_offset, curr_index):
        """
        Function to create a python object from the gdb text output with meta data
        like size and offset of all the members, needed to populate the values from
        the binary dump files.

        :param text: text gdb output for a particular symbol/type.
        :type the_type: str

        :param base_offset: base offset value.
        :type field: int

        :param curr_index: current line index in 'text'.
        :type field: int

        :return: py object created based on 'text', array check flag, current index
        """
        if curr_index == 0:
            d_type = text[0].split("{")[0]
        else:
            d_type = text[curr_index-1].split("{")[0]
        d_type = d_type.split("[")[0]
        d_type = d_type.strip()
        d_type = d_type.split()[-1]
        newclass = type(d_type,(), {})
        curr_obj = newclass()
        curr_offset = base_offset
        total_size = len(text)
        size = 0
        while total_size > curr_index:
            line = text[curr_index]
            curr_index = curr_index + 1
            if line is None:
                break
            if "/* offset | size */" in line or line.lstrip().rstrip() == "":
                continue
            re1 = re2 = 0
            for i in range(1):           # using a one iteration loop to implement break
                # sample match : "/*    0      |    40 */    struct thread_info {"
                re1 = re.search('\s+(\d+)\s+[|]\s+(\d+) \*\/\s+(struct|union) .*{', line)   #sample match:"/*    0      |    40 */    struct thread_info {"
                if re1:
                    curr_offset = int(re1.group(1))
                    size = int(re1.group(2))
                    break
                # sample match : "/*                 8 */            struct {"
                re2 = re.search('\/\*\s+(\d+) \*\/\s+(struct|union) .*{', line)
                if re2:
                    size = int(re2.group(1))
            if re1 or re2:
                obj, attr_name, curr_index = self.__create_object(text, curr_offset, curr_index)
                if attr_name is not None:
                    setattr(curr_obj, attr_name, [obj, curr_offset - base_offset, size])
                else:
                    # adding anonimous union members to parent
                    for attr, value in vars(obj).items():
                        temp_offset = curr_offset - base_offset
                        if isinstance(value[0], int) or isinstance(value[0], float):
                            value[0] += temp_offset
                        else:
                            value[1] += temp_offset
                        setattr(curr_obj, attr, value)
                continue
            else:
                re1 = re2 = re3 = re4 = 0
                for i in range(1):              # using a one iteration loop to implement break
                    # sample match : "/*   20      |     4 */                u32 need_resched;"
                    re1 = re.search('/\*\s+(\d+)\s+[|]\s+(\d+)\s\*/\s+([^:]+) (\S+);', line)
                    if re1 is not None:
                        curr_offset = int(re1.group(1))
                        size = int(re1.group(2))
                        datatype = re1.group(3)
                        attr_name = (re1.group(4))
                        break
                    # sample match : "/*                 4 */    uint32_t v;"
                    re2 = re.search('/\*\s+(\d+)\s\*/\s+([^:]+) (\S+);', line)
                    if re2 is not None:
                        size = int(re2.group(1))
                        datatype = re2.group(2)
                        attr_name = (re2.group(3))
                        break
                    # sample match : "/*  868: 3   |     4 */        unsigned int dl_overrun : 1;"
                    re3 = re.search('/\*\s+(\d+)[:]\s*(\d+)\s+[|]\s+(\d+)\s\*/\s+([^:]+) (\S+) [:] (\d+);', line)
                    if re3 is not None:
                        curr_offset = int(re3.group(1)) + (int(re3.group(2))/100)
                        size = int(re3.group(3)) + (int(re3.group(6))/100)
                        datatype = re3.group(4)
                        attr_name = (re3.group(5))
                        break
                    # sample match : "/*                  4 */        unsigned int x : 1;"
                    re4 = re.search('/\*\s+(\d+)\s\*/\s+([^:]+) (\S+) [:] (\d+);', line)
                    if re4 is not None:
                        size = int(re4.group(1)) + (int(re4.group(4))/100)
                        datatype = re4.group(2)
                        attr_name = (re4.group(3))
                if re1 or re2 or re3 or re4:
                    if ")(" in datatype:
                        attr_name = datatype.split(")(")[0].split("(")[1]
                    if attr_name.lstrip()[0] == '*':
                        datatype = datatype + " *"
                        attr_name = attr_name.lstrip('*')
                    if not self.__is_primary_type(datatype):
                        temp_obj = self.__get_type_info(datatype)
                        if isinstance(temp_obj[0], str):
                            setattr(curr_obj, attr_name, [curr_offset - base_offset, size, temp_obj[0]])
                        else:
                            setattr(curr_obj, attr_name, [temp_obj[0], curr_offset - base_offset, size])
                    else:
                        setattr(curr_obj, attr_name, [curr_offset - base_offset, size, datatype])
                    continue
                re_obj = re.search('\s*} (\S+);', line)
                if re_obj is not None:
                    return curr_obj, re_obj.group(1), curr_index
                re_obj = re.search('\s*};', line)
                if re_obj:
                    return curr_obj, None, curr_index
                re_obj = re.search('\s*}\s*(\[\d+\])', line)
                if re_obj:
                    return curr_obj, re_obj.group(1), curr_index
        # None means unnamed union or struct
        return curr_obj, None, curr_index

    def __get_datatype_from_ptr(self, ptr_addr_or_name):
        if isinstance(ptr_addr_or_name, str):
            var_type, vsize, temp_name = self.__get_type_info(ptr_addr_or_name)
            if var_type[-1] != "*":
                raise InvalidDatatype
            else:
                return var_type[:-1]
        else:
            raise InvalidDatatype

    def __unpack_format(self, size, ty):
        if ty == "char":
            return "<B"
        elif ty == "bool" or ty == "_Bool":
            return "<?"
        elif "float" in ty:
            return "<f"
        elif "double" in ty:
            return "<d"
        elif size == 8:
            if ("unsigned" in ty or '*' in ty) or (ty[0] == 'u'):
                return "<Q"
            else:
                return "<q"
        elif size == 4:
            if ("unsigned" in ty or '*' in ty) or (ty[0] == 'u'):
                return "<I"
            else:
                return "<i"
        elif size == 2:
            if ("unsigned" in ty or '*' in ty) or (ty[0] == 'u'):
                return "<H"
            else:
                return "<h"
        elif size == 1:
            if ("unsigned" in ty or '*' in ty) or (ty[0] == 'u'):
                return "<B"
            else:
                return "<b"
        else:
            return None

    def __get_type_info(self, the_type):
        """
        Function to return type info for the type.

        :param the_type: type of the structure field.
        :type the_type: str

        :return: d_type, size
        """
        if the_type in self.datatype_dict.keys():
            return self.datatype_dict[the_type]
        else:
            text = []
            try:
                text = self.gdbmi.getStructureData(the_type)
                size = self.sizeof(the_type)
            except gdbmi.GdbMIException:
                print("GDB Exception")
                pass

            if text:
                d_type = text[0].split("type = ")[1]
                d_type = self.__ignore_storage_class(d_type)
                d_type = self.__ignore_expanded_pointer(text, d_type)
                if not self.__is_primary_type(d_type):
                    master_obj = self.__create_object(text, 0, 0)
                    self.datatype_dict[the_type] = master_obj[0], size, master_obj[1]
                    return master_obj[0], size, master_obj[1]
                self.datatype_dict[the_type] = d_type, size, None
                return d_type, size, None

    def __item_to_dict(self, item, temp_dict):
        if "." not in item:
            temp_dict[item] = None
        else:
            key = item.split(".", 1)[0]
            if key in temp_dict.keys():
                temp_dict[key] = self.__item_to_dict(item.split(".", 1)[1], temp_dict[key])
            else:
                temp_dict[key] = self.__item_to_dict(item.split(".", 1)[1], {})
        return temp_dict

    def __attr_list_to_dict(self,attr_list):
        attr_dict = {}
        for item in attr_list:
            attr_dict = self.__item_to_dict(item, attr_dict)
        return attr_dict

    def __get_populated_object(self, addr, the_type, size, attr_list=None):
        """
        Function to populate value for the given data type and address.

        :param addr: address of the structure field type.
        :type addr: int

        :param the_type: structure type field
        :type the_type: str

        :param size: size of the structure field.
        :type size: int

        :param attr_list: list of attributes to be read(optional)
        :type attr_list: list

        :return: The data read from the dumps.
        """
        var_type, vsize, temp_name = self.__get_type_info(the_type)
        if vsize is None:
            vsize = size
        data = self.__get_bin_data(addr, vsize)
        if attr_list != None:
            attr_dict = self.__attr_list_to_dict(attr_list)
            return self.__object_value(var_type, data, 0, temp_name, attr_dict)
        else:
            return self.__object_value(var_type, data, 0, temp_name)

    def __get_bin_data(self, addr, size):
        """
        Function to return binary data of 'size' bytes read
        from the given address.

        :param addr: address of the structure field type.
        :type addr: str

        :param size: size of the structure field.
        :type size: int

        :return: The data read from the dumps.
        """
        bin_data = b""
        PAGE_SIZE = self.get_page_size()
        length  = PAGE_SIZE - (addr & (PAGE_SIZE-1))
        while(size > length):
            bin_data += self.read_physical(self.virt_to_phys(addr), length)
            addr +=length
            size -= length
            length  = PAGE_SIZE
        if(size > 0):
            addr = self.virt_to_phys(addr)
            bin_data += self.read_physical(addr, size)
        return bin_data

    def enum_lookup(self, enum, val):
        """
        Function to return string corresponding to the value for an enum.

        :param enum: enum type / gdb output string for a typedef enum.
        :type enum: str

        :param val: enum value.
        :type val: int

        :return: string corresponding to the enum value.
        """
        if "{" in enum and "}" in enum:
            temp = enum.split("{")[1].split("}")[0]
            temp = temp.split(",")
            res_dict = {}
            count = 0
            for i in temp:
                if "=" in i:
                    v = i.split("=")[0].strip()
                    k = int(i.split("=")[1].strip())
                    res_dict[k] = v
                    count = k+1
                else:
                    v = i.strip()
                    res_dict[count] = v
                    count += 1
            if val in res_dict.keys():
                return res_dict[val]
            else:
                return None
        else:
            if "enum " in enum:
                enum = enum.split()[1]
            if val >= 0:
                return self.gdbmi.get_enum_lookup_table(enum, val+1)[val]
            else:
                return None

    def __populate_primary(self, struct_bin_data, t):
        length = len(struct_bin_data)
        st_format = self.__unpack_format(length, t)
        if st_format is None:
            return None
        else:
            return struct.unpack_from(st_format, struct_bin_data)[0]

    def __populate_bitfield(self, struct_bin_data, t, bit_offset, bit_length):
        length = len(struct_bin_data)
        st_format = self.__unpack_format(length, t)
        if st_format is None:
            return None
        else:
            temp_data = struct_bin_data
            temp_bin_data = bin(int.from_bytes(temp_data, byteorder="little"))
            temp_bin_data = temp_bin_data[2:]    #remove 0b
            temp_bin_data = temp_bin_data.rjust(length*8, '0')
            temp_bin_data = temp_bin_data[::-1]  #reverse string
            temp_bin_data = temp_bin_data[bit_offset:bit_offset+bit_length]
            temp_bin_data = temp_bin_data[::-1]  #reverse string
            temp_bin_data = int(temp_bin_data, 2)
            temp_bin_data = temp_bin_data.to_bytes(length, byteorder="little")
            return struct.unpack_from(st_format, temp_bin_data)[0]

    def __populate_enum(self, struct_bin_data, var_type):
        val = self.__populate_primary(struct_bin_data, 'unsigned int')
        enum_var,enum_ty = self.__get_enum(var_type)
        enum_vals = [member.value for member in enum_var]
        if val not in enum_vals:
            val = self.__populate_primary(struct_bin_data, 'int')
        if val not in enum_vals:
            temp_key = "UNKNOWN_" + str(val)
            enum_dict = {i.name:i.value for i in enum_var}
            enum_dict.update({temp_key:val})
            enum_var = enum.Enum(enum_ty,enum_dict)
        res = enum_var(val)
        return res

    def __populate_array(self, struct_bin_data, var_type):
        t = var_type.split("[")[0].strip()
        temp_s = var_type.split("[")[1].split("]")[0]
        if temp_s == '' or temp_s == '0':
            return None
        else:
            s = int(temp_s)
        length = len(struct_bin_data)
        arr_len = s
        l = length // arr_len
        arr = []
        st_format = self.__unpack_format(l, t)
        if st_format is None:
            return None
        for i in range(arr_len):
            start = (i * l)
            end = ((i + 1) * l)
            if "enum" in var_type:
                res = self.__populate_enum(struct_bin_data[start:end], var_type)
                arr.append(res)
            else:
                arr.append(struct.unpack_from(st_format, struct_bin_data[start:end])[0])
        if t == 'char' or t == 'unsigned char' or t == 'signed char':
            temp = ''.join(chr(x) for x in arr)
            temp = temp.split('\0')[0]
            return temp
        return arr

    def get_enum_data(self, enum):
        if "{" not in enum:
            enum_data = self.gdbmi.getStructureData(enum)
            if enum_data:
                enum = enum_data[0].split("type = ")[1]
        res_dict = {}
        if "{" in enum and "}" in enum:
            temp = filter(None, enum.split("{")[1].split("}")[0].split(","))
            count = 0
            for i in temp:
                if "=" in i:
                    k = i.split("=")[0].strip()
                    v = int(i.split("=")[1].strip())
                    res_dict[k] = v
                    count = v+1
                else:
                    k = i.strip()
                    res_dict[k] = count
                    count += 1
        return res_dict

    def __get_enum(self, var_type):
        if var_type not in self.enum_data.keys():
            temp_enum = self.get_enum_data(var_type)
            enum_ty = var_type.split()[1].split("[")[0]
            if "{" in enum_ty:
                enum_ty = "enum"
            enum_var = enum.Enum(enum_ty,temp_enum)
            self.enum_data[var_type] = (enum_var,enum_ty)
        return self.enum_data[var_type]

    def __object_value(self, var_type, struct_bin_data, bin_offset, temp_name, attr_dict=None):
        """
        Function to return structure value for the given type and type offset.

        :param var_type: name of the structure field type.
        :type var_type: str

        :param struct_bin_data: bin data structure.
        :type struct_bin_data: str

        :param bin_offset: offset value.
        :type bin_offset: int

        :param temp_name: temporary name of the structure field type.
        :type temp_name: str

        :param attr_dict: dictionary of attributes to be read(optional)
        :type attr_list: dictionary

        :return: The data read from the dumps.
        """
        if temp_name is None:
            temp_name = ""
        if isinstance(var_type, str):
            t = var_type
            if (t not in primary_types) and ('*' not in t) and ('enum' not in t) and ("[" not in t):
                return None
            elif "enum" in var_type and "*" not in var_type and "[" not in var_type:
                return self.__populate_enum(struct_bin_data, var_type)
            elif "[" not in var_type:
                return self.__populate_primary(struct_bin_data, t)
            else:
                return self.__populate_array(struct_bin_data, var_type)
        else:
            if "[" in temp_name:
                temp_s = temp_name.split("[")[1].split("]")[0]
                if temp_s == '' or temp_s == '0':
                    return None
                else:
                    ar_len = int(temp_s)
                length = len(struct_bin_data)
                length = length // ar_len
                ar = []
                for i in range(ar_len):
                    ar.append(self.__object_value(var_type, struct_bin_data, i * length, None, attr_dict))
                return ar
            else:
                newclass = type(var_type)
                temp_structure = newclass()
                for key, value in var_type.__dict__.items():
                    if (attr_dict == None) or (key.split("[")[0] in list(attr_dict.keys())):
                        if isinstance(value[0], int) or isinstance(value[0], float):
                            offset = int(value[0])
                            length = int(value[1])
                            bit_offset = int(round((value[0] - offset),2)*100)
                            bit_length = int(round((value[1] - length),2)*100)
                            ty = value[2]
                            if (ty not in primary_types) and ('*' not in ty) and ('enum' not in ty):
                                setattr(temp_structure, key, None)
                                continue
                            if ("enum" in ty) and ('*' not in ty) and ("[" not in key):
                                res = self.__populate_enum(struct_bin_data[bin_offset + offset:bin_offset + offset + length], ty)
                                setattr(temp_structure, key, res)
                                continue
                            if "[" not in key:  # member in neither an array nor another struct/union
                                if (bit_length == 0) and (bit_offset == 0):
                                    res = self.__populate_primary(struct_bin_data[bin_offset + offset:bin_offset + offset + length], ty)
                                    setattr(temp_structure, key, res)
                                else:
                                    res = self.__populate_bitfield(struct_bin_data[bin_offset + offset:bin_offset + offset + length], ty, bit_offset, bit_length)
                                    setattr(temp_structure, key, res)
                            else:  # member is an array but not of struct/union
                                temp_ty = key.split("[")[1]
                                temp_ty = ty + "[" + temp_ty
                                res = self.__populate_array(struct_bin_data[bin_offset + offset:bin_offset + offset + length], temp_ty)
                                setattr(temp_structure, key.split("[")[0], res)
                        else:
                            if "[" not in key:  # member is another struct/union/obj but not an array
                                if attr_dict != None:
                                    setattr(temp_structure, key, self.__object_value(value[0], struct_bin_data,
                                                                                   value[1] + bin_offset, None, attr_dict[key]))
                                else:
                                    setattr(temp_structure, key, self.__object_value(value[0], struct_bin_data,
                                                                                   value[1] + bin_offset, None))
                            else:  # member is another struct/union/obj and an array
                                temp_s = key.split("[")[1].split("]")[0]
                                if temp_s == '' or temp_s == '0':
                                    setattr(temp_structure, key.split("[")[0], None)
                                    continue
                                else:
                                    arr_len = int(temp_s)
                                l = value[2] // arr_len
                                arr = [None] * arr_len
                                for i in range(arr_len):
                                    if attr_dict != None:
                                        arr[i] = self.__object_value(value[0], struct_bin_data, value[1] + (i * l) + bin_offset, None, attr_dict[key.split("[")[0]])
                                    else:
                                        arr[i] = self.__object_value(value[0], struct_bin_data, value[1] + (i * l) + bin_offset, None)
                                setattr(temp_structure, key.split("[")[0], arr)
                return temp_structure

    def rgetattr(self, obj, attr, *args):
        """
        Function to get attributes.

        :param obj: name of the object.
        :type obj: object

        :param attr: attribute type.
        :type attr: str

        :param args: arguments.
        :type args: str

        :return: The data read from the dumps.
        """
        def _getattr(obj, attr):
            return getattr(obj, attr, *args)
        return functools.reduce(_getattr, [obj] + attr.split('.'))

    def read_linkedlist(self, the_type, member, address, callback=None, callback_data=None, attr_list=None, ignore_head=True):
        """
        Function to read linked list structure for the given structure type / member.

        :param the_type: structure type field.
        :type the_type: str

        :param member: member of the structure field.
        :type member: str

        :param address: address of the structure field.
        :type address: int

        :param callback: address of the structure field.
        :type callback: int

        :param callback_data: call back data of the structure field.
        :type callback_data: str

        :param attr_list: list of attributes to be read(optional)
        :type attr_list: list

        :return: The data read from the dumps.
        """
        linked_list = []
        if isinstance(address,str):
            address = self.read_word(address)
            if address == None:
                raise SymbolNotFound(address + "Symbol not found")
        if address == 0x0:
            return linked_list
        if address is None:
            raise InvalidInput("None address passed to read_linkedlist")
        offset = self.field_offset(the_type, member)
        first_node = address + offset
        size = self.sizeof(the_type)
        if (self.read_word(first_node) == first_node) and ignore_head:
            return linked_list
        while True:
            entry = self.__get_populated_object(address, the_type, size, attr_list)
            if callback is not None:
                callback(self, entry, callback_data)
            linked_list.append(entry)
            next_member_addr = self.rgetattr(entry, member)
            next_next_addr = self.read_word(next_member_addr)
            if ignore_head:
                if next_member_addr != 0x0 and next_next_addr != first_node:
                    address = next_member_addr - offset
                else:
                    break
            else:
                if next_member_addr != 0x0 and next_member_addr != first_node:
                    address = next_member_addr - offset
                else:
                    break
        return linked_list


    def read_parray(self, ptr_addr_or_name, count, data_type=None, attr_list=None):
        """
        Function to read array for the given pointer address / pointer name.

        :param ptr_addr_or_name: address or name of the pointer.
        :type ptr_addr_or_name: str

        :param count: count of the array.
        :type count: int

        :param data_type: the structure field type
        :type data_type: str

        :param attr_list: list of attributes to be read(optional)
        :type attr_list: list

        :return: The data read from the dumps.
        """
        ptr_address = self.resolve_virt(ptr_addr_or_name)
        if ptr_address is None:
            if isinstance(ptr_addr_or_name, str):
                raise SymbolNotFound(ptr_addr_or_name + " symbol not found")
            else:
                raise InvalidInput("None passed to read_parray")
        address = self.read_pointer(ptr_address)
        if address is None:
            raise InvalidInput("Pointer passed to read_parray points to None")
        if data_type is None:
            data_type = self.__get_datatype_from_ptr(ptr_addr_or_name)
        size = self.sizeof(data_type)
        array = []
        while count >= 1:
            count = count - 1
            array.append(self.__get_populated_object(address, data_type, size, attr_list))
            address = address + size
        return array

    def read_pdatatype(self, ptr_addr_or_name, data_type=None, attr_list=None):
        """
        Function to read data type for the given pointer address / pointer name .

        :param ptr_addr_or_name: address or name of the pointer.
        :type ptr_addr_or_name: str

        :param data_type: the structure field type
        :type data_type: str

        :param attr_list: list of attributes to be read(optional)
        :type attr_list: list

        :return: The data read from the dumps.
        """
        ptr_address = self.resolve_virt(ptr_addr_or_name)
        if ptr_address is None:
            if isinstance(ptr_addr_or_name, str):
                raise SymbolNotFound(ptr_addr_or_name + " symbol not found")
            else:
                raise InvalidInput("None passed to read_pdatatype")
        address = self.read_pointer(ptr_address)
        if address is None:
            raise InvalidInput("Pointer passed to read_pdatatype points to None")
        if data_type is None:
            data_type = self.__get_datatype_from_ptr(ptr_addr_or_name)
        size = self.sizeof(data_type)
        return self.__get_populated_object(address, data_type, size, attr_list)

    def read_datatype(self, addr_or_name, data_type=None, attr_list=None):
        """
        Function to read data type for the given address /  name.

        :param addr_or_name: address or name of the pointer.
        :type addr_or_name: str

        :param data_type: the structure field type
        :type data_type: str

        :param attr_list: list of attributes to be read(optional)
        :type attr_list: list

        :return: The data read from the dumps.
        """
        address = self.resolve_virt(addr_or_name)
        if address is None:
            if isinstance(addr_or_name,str):
                raise SymbolNotFound(addr_or_name + " symbol not found")
            else:
                raise InvalidInput("None passed to read_datatype")
        if data_type is None:
            if isinstance(addr_or_name, str):
                data_type = addr_or_name
            else:
                raise InvalidDatatype
        size = self.sizeof(data_type)
        return self.__get_populated_object(address, data_type, size, attr_list)

    def read_multi(self, items):
        """
        Function to read multiple structures at a time.

        :param items: structure type fields from the list
        :type items: list

        :return: dictionary of data type.
        """
        out_dict = {}
        for var in items:
            if not (isinstance(var[0], str) or isinstance(var[0], int)):
                raise InvalidDatatype
            if not (isinstance(var[1], str) or var[1] is None):
                raise InvalidDatatype
            if (var[1] is None) or (var[1].rstrip() == ""):
                out_dict[var[0]] = self.read_datatype(var[0])
            elif var[1].rstrip() == "*":
                out_dict[var[0]] = self.read_pdatatype(var[0])
            elif var[1].rstrip()[-1] == "*":
                out_dict[var[0]] = self.read_pdatatype(var[0], var[1].rstrip()[:-1].rstrip())
            else:
                out_dict[var[0]] = self.read_datatype(var[0], var[1].rstrip())
        return out_dict

    def pretty_print(self, clas, fop, format, indent=0):
        indent += 4
        if isinstance(clas, list):
            for i in range(len(clas)):
                fop.write("\n" + ' ' * indent + "[{}] = (\n".format(i))
                self.pretty_print(clas[i], fop, format, indent)
            return
        for k, v in clas.__dict__.items():
            if '__dict__' in dir(v):
                fop.write(' ' * indent + k + ":\n")
                self.pretty_print(v, fop, format, indent)
            elif isinstance(v, list):
                fop.write(' ' * indent + k + '= (\n')
                indent += 4
                for i in range(0, len(v)):
                    if '__dict__' in dir(v[i]):
                        fop.write(' ' * indent + k + "[" + str(i) + "]: \n")
                        self.pretty_print(v[i], fop, format, indent)
                    else:
                        if isinstance(v[i], int) and format == "hex":
                            fop.write(' ' * indent + '[' + str(i) + '] : ' + "0x{0:X}".format(v[i]) + "\n")
                        else:
                            fop.write(' ' * indent + '[' + str(i) + '] : ' + str(v[i]) + "\n")
                indent -= 4
            else:
                if isinstance(v, int) and format == "hex":
                    fop.write(' ' * indent + k + ' = ' + "0x{0:X}".format(v) + "\n")
                else:
                    fop.write(' ' * indent + k + ' = ' + str(v) + "\n")

    def print_struct(self, struct_obj, fop, members=None, fmt_str=None, format=None):
        """
        Function to print the complete structure or member of
        structure with some given format.

        :param struct_obj: struct object
        :type struct_obj: object

        :param fop: output file handle
        :type fop: file handle

        :param members: list of member of structure (optional argument)
        :type members: list

        :param fmt_str: format specifier for each member (optional argument)
        :type fmt_str: list

        `Example`:
        1. Print Complete structure::
            vpp_device = self.ramdump.read_datatype('vpp_device')
            self.ramdump_util.print_struct(vpp_device, fop)

        2. Print member of structure without format::
            vpp_device = self.ramdump.read_datatype('vpp_device')
            self.ramdump_util.print_struct(vpp_device, fop, ["chip_ver", "foundry_id"])

        3. Print member of structure with format::
            vpp_device = self.ramdump.read_datatype('vpp_device')
            self.ramdump_util.print_struct(vpp_device, fop, ["chip_ver", "foundry_id"], ["0x{:08x}", "{}"])
        """
        if members is None:
            self.pretty_print(struct_obj, fop, format, 0)
        else:
            for i in range(0, len(members)):
                value = getattr(struct_obj, members[i])
                if fmt_str is None:
                    fop.write(members[i] + " : {}\n".format(value))
                elif fmt_str[i].count("{") == 1:
                    fop.write(fmt_str[i].format(value))
                else:
                    fop.write(fmt_str[i].format(members[i], value))

class Struct(object):
    """
    Helper class to abstract C structs retrieval by providing a map of fields
    to functions on how to retrieve these

    Given C struct::

        struct my_struct {
            char label[MAX_STR_SIZE];
            u32 number;
            void *address;
        }

    You can abstract as:

    >>> var = Struct(ramdump, var_name, struct_name="struct my_struct",
                                        fields={'label': Struct.get_cstring,
                                                'number': Struct.get_u32,
                                                'address': Struct.get_pointer})
    >>> var.label
    'label string'
    >>> var.number
    1234
    """
    _struct_name = None
    _fields = None

    def __init__(self, ramdump, base, struct_name=None, fields=None):
        """
        :param ram_dump:    Reference to the ram dump
        :param base:        The virtual address or variable name of struct
        :param struct_name: Name of the structure, should start with 'struct'.
                            Ex: 'struct my_struct'
        :param fields:  Dictionary with key being the element name and value
                        being a function pointer to method used to retrieve it.
        """
        self.ramdump = ramdump
        self._base = self.ramdump.resolve_virt(base)
        self._data = {}
        if struct_name:
            self._struct_name = struct_name
        if fields:
            self._fields = fields

    def is_empty(self):
        """
        :return: true if struct is empty
        """
        return self._base == 0 or self._base is None or self._fields is None

    def get_address(self, key):
        """
        :param key: struct field name
        :return: returns address of the named field within the struct
        """
        return self._base + self.ramdump.field_offset(self._struct_name, key)

    def get_pointer(self, key):
        """
        :param key: struct field name
        :return: returns the addressed pointed by field within the struct

        example struct::

            struct {
                void *key;
            };
        """
        address = self.get_address(key)
        return self.ramdump.read_pointer(address)

    def get_struct_sizeof(self, key):
        """
        :param key: struct field name
        :return: returns the size of a field within struct

        Given C struct::

            struct my_struct {
                char key1[10];
                u32 key2;
            };

        You could do:

        >>> struct = Struct(ramdump, 0, struct="struct my_struct",
                                        fields={"key1": Struct.get_cstring,
                                                "key2": Struct.get_u32})
        >>> struct.get_struct_sizeof(key1)
        10
        >>> struct.get_struct_sizeof(key2)
        4
        """
        return self.ramdump.sizeof('((%s *) 0)->%s' % (self._struct_name, key))

    def get_cstring(self, key):
        """
        :param key: struct field name
        :return: returns a string that is contained within struct memory

        Example C struct::

            struct {
                char key[10];
            };
        """
        address = self.get_address(key)
        length = self.get_struct_sizeof(key)
        return self.ramdump.read_cstring(address, length)

    def get_cstring_from_pointer(self, key):
        """
        :param key: struct field name
        :return: returns a string that is contained within struct memory

        Example C struct::

            struct {
                char *key;
            };
        """
        pointer = self.get_pointer(key)
        return self.ramdump.read_cstring(pointer)

    def get_u8(self, key):
        """
        :param key: struct field name
        :return: returns a u8 integer within the struct

        Example C struct::

            struct {
                u8 key;
            };
        """
        address = self.get_address(key)
        return self.ramdump.read_byte(address)

    def get_u16(self, key):
        """
        :param key: struct field name
        :return: returns a u16 integer within the struct

        Example C struct::

            struct {
                u16 key;
            };
        """
        address = self.get_address(key)
        return self.ramdump.read_u16(address)

    def get_u32(self, key):
        """
        :param key: struct field name
        :return: returns a u32 integer within the struct

        Example C struct::

            struct {
                u32 key;
            };
        """
        address = self.get_address(key)
        return self.ramdump.read_u32(address)

    def get_u64(self, key):
        """
        :param key: struct field name
        :return: returns a u64 integer within the struct

        Example C struct::

            struct {
                u64 key;
            };
        """
        address = self.get_address(key)
        return self.ramdump.read_u64(address)

    def get_array_ptrs(self, key):
        """
        :param key: struct field name
        :return: returns an array of pointers

        Example C struct::

            struct {
                void *key[4];
            };
        """
        ptr_size = self.ramdump.sizeof('void *')
        length = self.get_struct_sizeof(key) // ptr_size
        address = self.get_address(key)
        arr = []
        for i in range(0, length - 1):
            ptr = self.ramdump.read_pointer(address + (ptr_size * i))
            arr.append(ptr)
        return arr

    def __setattr__(self, key, value):
        if self._fields and key in self._fields:
            raise ValueError(key + "is read-only")
        else:
            super(Struct, self).__setattr__(key, value)

    def __getattr__(self, key):
        if not self.is_empty():
            if key in self._data:
                return self._data[key]
            elif key in self._fields:
                fn = self._fields[key]
                value = fn(self, key)
                self._data[key] = value
                return value
        return None
