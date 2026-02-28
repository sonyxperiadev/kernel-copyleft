#SPDX-License-Identifier: GPL-2.0-only
#Copyright (c) 2023-2025 Qualcomm Innovation Center, Inc. All rights reserved.

from print_out import print_out_str
from parser_util import cleanupString
import maple_tree
from mmu import Armv8MMU, Armv7MMU
import struct
from kstructlib import StructParser
from collections import namedtuple

class UTaskInfo:
    def __init__(self):
        self.vmalist=[]
        self.name = ""
        self.task_addr = 0
        self.mmu = None

    def __repr__(self):
        olist = []
        olist.append("Task {} 0x{}".format(self.name, hex(self.task_addr) if self.task_addr else 0))
        for vma in self.vmalist:
            olist.append(str(vma))
        return "\n".join(olist)

class Vma:
    def __init__(self):
        self.vm_start = 0
        self.vm_end = 0
        self.flags = 0
        self.file = 0
        self.file_name = ""
        self.vm_pgoff = ""
        self.vm_addr = 0

    def __repr__(self):
        return "0x{0:x}-0x{1:x} flags:0x{2:x} file_name:{3}".format(self.vm_start, self.vm_end, self.flags, self.file_name)

class ProcessNotFoundExcetion(Exception):
    pass

class UTaskLib:
    def __init__(self, ramdump):
        self.offset_comm = ramdump.field_offset('struct task_struct', 'comm')
        self.mm_offset = ramdump.field_offset('struct task_struct', 'mm')
        self.f_path_offset = ramdump.field_offset('struct file', 'f_path')
        self.dentry_offset = ramdump.field_offset('struct path', 'dentry')
        self.d_iname_offset = ramdump.field_offset('struct dentry', 'd_iname')
        self.active_mm_offset = ramdump.field_offset('struct task_struct', 'active_mm')
        self.pid_offset = ramdump.field_offset('struct task_struct', 'pid')
        self.ramdump = ramdump

    def get_utask_info(self, process_name, logging=False):
        '''
        support get task info by process name or process id

        process_name: str --> process name
                      int -->process id
        '''
        for task in self.ramdump.for_each_process():
            task_name = cleanupString(self.ramdump.read_cstring(task + self.offset_comm, 16))
            if isinstance(process_name, str):
                if task_name != process_name:
                    continue
            else:
                pid = self.ramdump.read_u32(task + self.pid_offset)
                if pid != process_name:
                    continue

            mm_addr = self.ramdump.read_word(task + self.mm_offset)
            if process_name == "init" and mm_addr == 0:
                mm_addr = self.ramdump.read_word(task + self.active_mm_offset)
            if mm_addr == 0:
                if logging is True:
                    print_out_str("mm for {} is null\n".format(process_name))
                _utask = UTaskInfo()
                _utask.name = task_name
                _utask.task_addr = task
                return _utask
            pgd = self.ramdump.read_structure_field(mm_addr, 'struct mm_struct',
                                                'pgd')
            if self.ramdump.s2_walk:
                pgdp = self.ramdump.mmu.virt_to_physel1(pgd)
            else:
                pgdp = self.ramdump.virt_to_phys(pgd)
            if self.ramdump.arm64:
                mmu = Armv8MMU(self.ramdump, pgdp)
            else:
                mmu = Armv7MMU(self.ramdump, pgdp)

            vmalist = []
            for vmaobj in self.for_each_vma_list(mm_addr):
                vmalist.append(vmaobj)

            _utask = UTaskInfo()
            _utask.name = task_name
            _utask.vmalist = vmalist
            _utask.task_addr = task
            _utask.mmu = mmu
            return _utask
        raise ProcessNotFoundExcetion("Process:{} was not found!".format(process_name))

    @staticmethod
    def read_bytes(ramdump, mmu, addr, len1, zram_parser=None):
        vmaddr = addr
        addr = mmu.virt_to_phys(addr)
        s = ramdump.read_physical(addr, len1)
        if s is None and zram_parser:
            pte = mmu.get_swap_pte(vmaddr)
            if pte:
                zdata = zram_parser.read_data(vmaddr, pte)
                if zdata and len1 <= len(zdata):
                    #print("read content from ptr:0x%x in zram memory"% addr)
                    s = zdata[0:len1]
        if (s is None) or (s == ''):
            return None
        if len1 == 8:
            s = struct.unpack('<Q', s)
        elif len1 == 4:
            s = struct.unpack('<I', s)
        elif len1 == 2:
            s = struct.unpack('<H', s)
        elif len1 == 1:
            s = struct.unpack('<B', s)
        else:
            print_out_str("This api used to unpack 1/2/4/8 bytes data, check the len\n")
            exit()

        return s[0]

    @staticmethod
    def read_binary(ramdump, mmu, addr, length, zram_parser=None):
        """Reads binary data of specified length from addr_or_name."""
        min = 0
        msg = b''
        while length > 0:
            msg_binary = None
            addr = addr + min
            # msg located in the same page
            if length < (0x1000 - addr % 0x1000):
                min = length
            # msg separated in two pages
            else:
                min = 0x1000 - addr % 0x1000
            length = length - min
            addr_phys = mmu.virt_to_phys(addr)

            if addr_phys is None and zram_parser:
                pte = mmu.get_swap_pte(addr)
                if pte:
                    zdata = zram_parser.read_data(addr, pte)
                    if zdata and min <= len(zdata):
                        #print("read content from ptr:0x%x in zram memory"% addr)
                        msg_binary = zdata[0:min]
            else:
                msg_binary = ramdump.read_physical(addr_phys, min)

            if msg_binary is None or msg_binary == '':
                msg = msg + b'\x00' * min
            else:
                msg = msg + msg_binary
        return msg

    def for_each_vma_list(self, mm_addr, count=None):
        self.read_count = 0
        if (self.ramdump.kernel_version) < (6, 1, 0):
            # struct vm_area_struct *mmap
            mmap = self.ramdump.read_structure_field(mm_addr, 'struct mm_struct',
                                                        'mmap')
            _map = mmap
            while _map:
                self.read_count +=1
                if count and self.read_count > count:
                    break
                vma_obj = self._make_vma_object(_map)
                _map = self.ramdump.read_structure_field(
                    _map, 'struct vm_area_struct', 'vm_next')  # next loop
                yield vma_obj
        else:
            mm_mt = self.ramdump.struct_field_addr(mm_addr, 'struct mm_struct',
                                                    'mm_mt')
            mt_walk = maple_tree.MapleTreeWalker(self.ramdump)
            vmalist = []
            try:
                mt_walk.walk(mm_mt, self._save_vma_list, vmalist, count)
            except:
                pass
            for vmaobj in vmalist:
                yield vmaobj

    def _make_vma_object(self, vma_addr):
        '''
        read vma info from vma_addr. then save it to vma object
        '''
        tmpstartVm = self.ramdump.read_structure_field(
            vma_addr, 'struct vm_area_struct', 'vm_start')
        tmpsEndVm = self.ramdump.read_structure_field(
            vma_addr, 'struct vm_area_struct', 'vm_end')
        flags = self.ramdump.read_structure_field(
            vma_addr, 'struct vm_area_struct', 'vm_flags')
        file = self.ramdump.read_structure_field(
            vma_addr, 'struct vm_area_struct', 'vm_file')
        vm_pgoff = self.ramdump.read_structure_field(
            vma_addr, 'struct vm_area_struct', 'vm_pgoff')

        file_name = ""
        if file:
            dentry = self.ramdump.read_word(file + self.f_path_offset +
                                       self.dentry_offset)
            file_name = cleanupString(self.ramdump.read_cstring(
                dentry + self.d_iname_offset, 32))
        vma_obj = Vma()
        vma_obj.vm_start = tmpstartVm
        vma_obj.vm_end = tmpsEndVm
        vma_obj.flags = flags
        vma_obj.file = file
        vma_obj.file_name = file_name
        vma_obj.vm_pgoff = vm_pgoff
        vma_obj.vm_addr = vma_addr
        return vma_obj

    def _save_vma_list(self, node, vmalist, count=None):
        if node:
            self.read_count +=1
            if count and self.read_count > count:
                raise StopIteration
            vmalist.append(self._make_vma_object(node))

    def __dentry_d_name(self, dentry):
        dentry_struct = self.struct_parser.read_struct(dentry, 'struct dentry', ['d_name', 'd_parent'])
        d_parent = dentry_struct.d_parent
        hash_len = dentry_struct.d_name.hash_len
        d_len = hash_len >> 32
        d_name_addr = dentry_struct.d_name.name
        if (d_len == 0 or d_name_addr == 0):
            return ''
        d_name = self.ramdump.read_cstring(d_name_addr, d_len)
        return d_parent, d_name

    def __get_path_name(self, dentry, vfsmnt):
        tmp_vfsmnt = vfsmnt
        d_parent = dentry
        path_name = ''

        ### first step to find out file path
        max_count = 8
        index = 0
        while True:
            index += 1
            if index > max_count:
                # make an error
                break
            tmp_dentry = d_parent
            d_parent, d_name = self.__dentry_d_name(tmp_dentry)
            if d_name == "/":
                break

            if (tmp_dentry != d_parent):
                path_name = d_name + '/' + path_name if path_name else d_name
            else:
                break

        if tmp_vfsmnt == 0:
            return path_name

        ### second step to find out mount path
        tmp_mount = tmp_vfsmnt - self.mnt_offset
        index = 0
        while True:
            index += 1
            if index > max_count:
                # make an error
                break
            mount_struct = self.struct_parser.read_struct(tmp_mount, 'struct mount', ['mnt_parent', 'mnt_mountpoint'])
            if tmp_mount == mount_struct.mnt_parent:
                break
            _, d_name = self.__dentry_d_name(mount_struct.mnt_mountpoint)
            tmp_mount = mount_struct.mnt_parent
            if d_name != "/":
                path_name = d_name + '/' + path_name
                break
        return path_name

    def for_each_cmdline(self):
        '''
        As task_struct->comm[16] stored a small part of process name.
        developer can't distinct which process it is.
        this method is to dump full cmdline for each user space process
        '''
        if not hasattr(self, "struct_parser"):
            self.struct_parser = StructParser(self.ramdump)

        cmdline_obj = namedtuple("TaskCmdline", ["pid", "comm", "cmdline"])
        self.mnt_offset = self.ramdump.field_offset('struct mount', 'mnt')

        for task in self.ramdump.for_each_process():
            mm_addr = self.ramdump.read_word(task + self.mm_offset)
            if mm_addr == 0:
                # skip kernel process
                continue

            mm_addr = self.ramdump.read_word(task + self.mm_offset)
            pid = self.ramdump.read_u32(task + self.pid_offset)
            task_name = cleanupString(self.ramdump.read_cstring(task + self.offset_comm, 16))
            path_struct = None

            for vmaobj in self.for_each_vma_list(mm_addr, count=1):# only read the first vma
                if vmaobj.file != 0:
                    path_struct = self.struct_parser.read_struct(
                                vmaobj.file + self.f_path_offset, 'struct path', ['dentry', 'mnt'])

            if not path_struct:
                continue

            dentry_addr = path_struct.dentry
            vfsmnt = path_struct.mnt
            if (dentry_addr != 0x0):
                file_name = self.__get_path_name(dentry_addr, vfsmnt)
                yield cmdline_obj(pid, task_name, file_name)
