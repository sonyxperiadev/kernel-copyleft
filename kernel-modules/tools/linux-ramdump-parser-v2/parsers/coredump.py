# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.

from parser_util import register_parser, RamParser
import print_out
import struct
import os
from utasklib import UTaskLib
from utasklib import ProcessNotFoundExcetion
from kstructlib import StructParser
import sys
import traceback

PT_LOAD = 1
PT_NOTE = 4
VM_READ=0x00000001
VM_WRITE=0x00000002
VM_EXEC=0x00000004
VM_DONTDUMP = 0x04000000
VM_SHARED = 0x00000008
VM_HUGETLB = 0x00400000
VM_IO=0x00004000
PF_R=0x4
PF_W=0x2
PF_X=0x1
NT_PRSTATUS = 1
NT_FPREGSET = 2
NT_PRPSINFO = 3
NT_TASKSTRUCT = 4
NT_AUXV = 6
NT_ARM_TLS = 0x401
NT_ARM_HW_BREAK	= 0x402
NT_ARM_HW_WATCH	= 0x403
NT_ARM_SYSTEM_CALL = 0x404
NT_ARM_SVE = 0x405
NT_ARM_PAC_MASK	= 0x406
NT_ARM_TAGGED_ADDR_CTRL	= 0x409
NT_ARM_PAC_ENABLED_KEYS	= 0x40a

NT_SIGINFO= 0x53494749
NT_FILE = 0x46494c45
SHT_NULL = 0
SHN_UNDEF = 0
AT_NULL = 0
PIDTYPE_PID = 0
PIDTYPE_PGID = 1
PIDTYPE_SID = 2
CLOCK_TICK_RATE = 1193182
NSEC_PER_USEC = 1000
NSEC_PER_SEC = 1000000000
MMF_DUMP_ANON_PRIVATE=2
MMF_DUMP_ANON_SHARED=3
MMF_DUMP_MAPPED_PRIVATE=4
MMF_DUMP_MAPPED_SHARED=5
MMF_DUMP_ELF_HEADERS=6
MMF_DUMP_HUGETLB_PRIVATE=7
MMF_DUMP_HUGETLB_SHARED=8
MMF_DUMP_DAX_PRIVATE=9
MMF_DUMP_DAX_SHARED=10

VM_HIGH_ARCH_BIT_0 = 32
VM_HIGH_ARCH_BIT_1 = 33

VM_HIGH_ARCH_0 = 1<< VM_HIGH_ARCH_BIT_0 #BIT(VM_HIGH_ARCH_BIT_0)
VM_HIGH_ARCH_1 = 1 << VM_HIGH_ARCH_BIT_1
VM_MTE = VM_HIGH_ARCH_0
VM_MTE_ALLOWED = VM_HIGH_ARCH_1

###for arm64
SCTLR_ELx_ENIA = 1 << 31
SCTLR_ELx_ENIB = 1 << 30
SCTLR_ELx_ENDA = 1 << 27
SCTLR_ELx_ENDB = 1 << 13


PR_PAC_APIAKEY = 1 << 0
PR_PAC_APIBKEY = 1 << 1
PR_PAC_APDAKEY = 1 << 2
PR_PAC_APDBKEY = 1 << 3

NT_TYPE_MAPS = {
    NT_PRSTATUS: "NT_PRSTATUS",
    NT_FPREGSET: "NT_FPREGSET",
    NT_PRPSINFO: "NT_PRPSINFO",
    NT_TASKSTRUCT: "NT_TASKSTRUCT",
    NT_AUXV: "NT_AUXV",
    NT_ARM_TLS: "NT_ARM_TLS",
    NT_ARM_HW_BREAK: "NT_ARM_HW_BREAK",
    NT_ARM_HW_WATCH: "NT_ARM_HW_WATCH",
    NT_ARM_SYSTEM_CALL: "NT_ARM_SYSTEM_CALL",
    NT_ARM_SVE: "NT_ARM_SVE",
    NT_ARM_PAC_MASK: "NT_ARM_PAC_MASK",
    NT_ARM_TAGGED_ADDR_CTRL: "NT_ARM_TAGGED_ADDR_CTRL",
    NT_ARM_PAC_ENABLED_KEYS: "NT_ARM_PAC_ENABLED_KEYS",
    NT_SIGINFO: "NT_SIGINFO",
    NT_FILE: "NT_FILE"
}

def nt_type_to_str(type):
    try:
        return NT_TYPE_MAPS[type]
    except:
        return "Unknown note type"


@register_parser('--coredump', 'output coredump')
class coredump(RamParser):
    '''
    generate coredump of userspace process

    --coredump pid_proc=[PID|PROCESS_NAME] log_level=[DEBUG|INFO|WARN|ERROR]
        # generate coredump of the given pid or given process name
                                    eg: --coredump pid_proc=1  generate init coredump
                                    eg: --coredump pid_proc=logd generate logd coredump
        # default log level is WARN.

        --coredump pid_proc=surfaceflinger log_level=DEBUG
            Request coredump for surfaceflinger and print debug log to dmesg_TZ.txt
            Core file "core.[PID].surfaceflinger" will be generate in output folder
    '''
    def __init__(self, *args):
        super(coredump, self).__init__(*args)

        self.log_level = print_out.get_req_loglevel()
        coredump_pid = None
        try:
            coredump_pid = self.get_request_pid()
            self.taskinfo = UTaskLib(self.ramdump).get_utask_info(coredump_pid)
        except ProcessNotFoundExcetion as e:
            if coredump_pid:
                self.printe("process {} is not started".format(coredump_pid))
        except Exception as e:
            self.printe(str(e))
            traceback.print_exc()

        self.task_vaddr = self.taskinfo.task_addr
        self.process_mmu = self.taskinfo.mmu
        self.struct_parser = StructParser(self.ramdump)
        self.task = self.struct_parser.read_struct(self.taskinfo.task_addr, 'struct task_struct')
        self.printd(self.task)
        self.mm = self.struct_parser.read_struct(self.task.mm,"struct mm_struct")
        if self.mm is None:
            raise Exception("Coredump is not supported!!, please give userspace process name or pid")
        corefile_name = "core.{0:d}.{1:s}".format(self.task.pid, self.to_string(self.task.comm))
        self.corefile = self.ramdump.open_file(corefile_name,'wb+')
        try:
            self.gate_vma = self.ramdump.address_of("gate_vma")
        except:
            self.gate_vma = None
        self.gcore_elf_struct = dict()
        self.user_regset_view = self.task_user_regset_view()
        self.elf_note_info = None
        self.large_vma_size = 50 *1024 * 2034 # 50M
        self.PAGE_SIZE =  self.ramdump.get_page_size()

    def printi(self, *msg):
        print_out.printi(self, *msg)

    def printd(self, *msg):
        print_out.printd(self, *msg)

    def printe(self, *msg):
        print_out.printe(self, *msg)

    def __del__(self):
        if hasattr(self, "corefile"):
            self.corefile.close()

    def to_string(self, p_data):
        return bytes(p_data).decode('ascii', 'ignore').split("\0")[0]

    def get_request_pid(self):
        '''
            parse pid_proc value from sys.argv

            --coredump pid_proc=[PID|PROCESS_NAME]  # generate coredump of the given pid or given process name
                                            eg: --coredump pid_proc=1  generate init coredump
                                            eg: --coredump pid_proc=logd generate logd coredump
        '''
        pid_proc_val = None
        for arg in sys.argv:
            if "pid_proc=" in arg:
                _, val = arg.split('=')
                pid_proc_val = int(val) if val.isdigit() else val.strip()
                break
        if pid_proc_val == None:
            msg = "Invalid pid or process name, Please provide correct argument!!\n" \
                            +"   --coredump pid_proc=[PID|PROCESS_NAME]"
            self.printe(msg)
            raise Exception(msg)
        else:
            self.printe("Request coredump for PID {0}".format(pid_proc_val))
            return pid_proc_val


    # ##################################################################
    #     +-----------------------+ <----- stacktop
    #     |                       |
    #     |          stack        |
    #     |                       |
    #     +-----------------------+ <----- stackbase
    #     |                       |
    #     |   struct thread_info  |
    #     |                       |
    #     +-----------------------+
    # ##################################################################
    def stack_size(self):
        if self.ramdump.arm64:
            return self.ramdump.thread_size
        else:
            return self.PAGE_SIZE * 2

    def task_to_stackbase(self,task_vaddr):
        task = self.struct_parser.read_struct(task_vaddr,"struct task_struct",["stack"])
        self.printd("stack={0:x}".format(task.stack))
        if task.stack is not None and task.stack != 0:
            stackbase = task.stack
            return stackbase
        else:
            return task_vaddr & ~(self.stack_size()-1)

    def generic_get_stacktop(self,task_vaddr):
        # stackbase = self.task_to_stackbase(task_vaddr)
        stacktop =  self.task_to_stackbase(task_vaddr) + self.stack_size()
        return stacktop

    def gpr_get(self,task_vaddr, size):
        '''
        general purpose register set, floating point and debug registers.
        return: user space register set
        '''
        vaddr = self.generic_get_stacktop(task_vaddr) - self.ramdump.sizeof("struct pt_regs")
        user_pt_regs= self.struct_parser.read_struct(vaddr,"struct user_pt_regs")
        self.printd("gpr_get addr[{0:x}],task[{1:x}], stacktop[{2:x}]"
                        .format(vaddr,task_vaddr,self.generic_get_stacktop(task_vaddr)))
        self.printd(user_pt_regs)
        if self.ramdump.sizeof("struct user_pt_regs") != size:
            raise Exception("gpr_get sizeof(struct user_pt_regs)=0x%x expected size 0x%x"
                            % (self.ramdump.sizeof("struct user_pt_regs"), size))
        return self.ramdump.read_physical(self.ramdump.virt_to_phys(vaddr), size)

    def fpr_get(self, task_vaddr, size):
        '''
        return (struct task_struct *)->thread->uw->fpsimd_state;
        '''
        task_struct_def = self.struct_parser.parser_struct_def("struct task_struct")
        offset = task_struct_def.thread.uw.get_field("fpsimd_state")._p_offset
        fpsimd_state_addr = task_vaddr + offset
        if self.ramdump.sizeof("struct user_fpsimd_state") != size:
            raise Exception("fpr_get data expected size 0x%x" % (size))
        fps = self.ramdump.read_binarystring(fpsimd_state_addr, self.ramdump.sizeof("struct user_fpsimd_state"))
        self.printd("fpr_get-->struct user_fpsimd_state", fps)
        return fps

    def tls_get(self, task_vaddr, size):
        '''
        return (struct task_struct *)->thread->uw->tp_value;
        '''
        task_struct_def = self.struct_parser.parser_struct_def("struct task_struct")
        offset = task_struct_def.thread.uw.get_field("tp_value")._p_offset
        tp_value_addr = task_vaddr + offset
        tp = self.ramdump.read_binarystring(tp_value_addr, size)
        self.printd("tls_get",tp)
        return tp

    def system_call_get(self, task_vaddr, size):
        vaddr = self.generic_get_stacktop(task_vaddr) - self.ramdump.sizeof("struct pt_regs")
        syscallno_addr= self.ramdump.struct_field_addr(vaddr, "struct pt_regs", "syscallno")
        syscallno = self.ramdump.read_binarystring(syscallno_addr, size)
        self.printd("pt_regs.syscallno]", syscallno)
        return syscallno

    def default_get(self, task_vaddr, size):
        return size * b'0x\00'

    def ptrauth_user_pac_mask(self, h, l):
        if h < l:
            raise Exception("ptrauth_user_pac_mask failed")
        return ((1 << (h-l+1))-1) << l

    def pac_mask_get(self, task_vaddr, size):
        mask = self.ptrauth_user_pac_mask(54, self.ramdump.va_bits)
        mask_byte = struct.pack('<Q', mask)
        if len(mask_byte) * 2 != size:
            raise Exception("pac_mask_get data expected size 0x%x" % (size))
        return mask_byte * 2

    def pac_enabled_keys_get(self, task_vaddr, size):
        task_struct = self.struct_parser.read_struct(task_vaddr,"struct task_struct",  ["thread"])
        sctlr_user = task_struct.thread.sctlr_user

        retval = 0
        if sctlr_user & SCTLR_ELx_ENIA:
            retval |= PR_PAC_APIAKEY
        if sctlr_user & SCTLR_ELx_ENIB:
            retval |= PR_PAC_APIBKEY
        if sctlr_user & SCTLR_ELx_ENDA:
            retval |= PR_PAC_APDAKEY
        if sctlr_user & SCTLR_ELx_ENDB:
            retval |= PR_PAC_APDBKEY

        if size != 8:
            raise Exception("pac_enabled_keys_get data expected size 0x%x" % (size))

        self.printi("pac_enabled_keys_get", hex(retval))
        return  struct.pack('<Q', retval)

    def task_user_regset_view(self):
        '''
        struct user_regset_view user_aarch64_view
        '''
        user_arm_view = self.struct_parser.parser_struct_def("struct user_regset_view")
        vaddr = self.ramdump.address_of("user_aarch64_view")
        arm_view = self.struct_parser.read_struct(vaddr,"struct user_regset_view")
        user_arm_view.name = arm_view.name
        user_arm_view.n = arm_view.n
        user_arm_view.e_machine = arm_view.e_machine
        user_arm_view.regsets = []

        user_regset_size = self.ramdump.sizeof("struct user_regset")

        self.printe("user_aarch64_view->name[%s] user_aarch64_view->n[%d] user_aarch64_view->e_machine[%d]"
                    % (arm_view.name, arm_view.n, arm_view.e_machine))

        for i in range (0,int(arm_view.n)):
            user_regset_vaddr = arm_view.regsets + (i * user_regset_size)
            user_regset = self.struct_parser.read_struct(user_regset_vaddr,"struct user_regset")
            user_regset.active = True
            if user_regset.core_note_type == NT_PRSTATUS:
                user_regset.regset_get = self.gpr_get
            elif user_regset.core_note_type == NT_FPREGSET:
                user_regset.regset_get = self.fpr_get
                user_regset.active = user_regset.n
            elif user_regset.core_note_type == NT_ARM_TLS:
                user_regset.regset_get = self.tls_get
            elif user_regset.core_note_type == NT_ARM_SVE:
                user_regset.active = False
            elif user_regset.core_note_type == NT_ARM_SYSTEM_CALL:
                user_regset.regset_get = self.system_call_get
            elif user_regset.core_note_type == NT_ARM_HW_BREAK:
                user_regset.regset_get = self.default_get
            elif user_regset.core_note_type == NT_ARM_HW_WATCH:
                user_regset.regset_get = self.default_get
            elif user_regset.core_note_type == NT_ARM_PAC_MASK:
                user_regset.regset_get = self.pac_mask_get
            elif user_regset.core_note_type == NT_ARM_PAC_ENABLED_KEYS:
                user_regset.regset_get = self.pac_enabled_keys_get
            elif user_regset.core_note_type == NT_ARM_TAGGED_ADDR_CTRL:
                user_regset.regset_get = self.default_get
            else:
                user_regset.regset_get = self.default_get

            user_arm_view.regsets.append(user_regset)
        return user_arm_view

    def pack_string(self, value):
        if value is None or value == 0:
            value = ""
        s = struct.Struct("{0:d}s".format(len(value)))
        buf = s.pack(value.encode('utf-8'))
        return buf

    def pack_data(self, data_type, value, total_size=0):
        if value is None :
            value = 0

        size = self.ramdump.sizeof(data_type)
        fmt = getattr(self.ramdump, "_RamDump__unpack_format")(size, data_type)
        return struct.pack(fmt, value)

    def elf_core_extra_phdrs(self):
        mte_vma_count = 0
        for vma in self.taskinfo.vmalist:
            if vma.flags & VM_MTE:
               mte_vma_count += 1
        return mte_vma_count

    def ffz(self,word):
        num = 0
        if ((word & 0xffff) == 0):
            num = num + 16
            word = word >> 16

        if ((word & 0xff) == 0):
            num = num + 8
            word = word >> 8

        if ((word & 0xf) == 0):
            num = num + 4
            word = word >> 4

        if ((word & 0x3) == 0):
            num = num + 2
            word = word >> 2

        if ((word & 0x1) == 0):
            num = num + 1
        return num

    def FILTER(self,type):
        return self.mm.flags & (1 << type)

    def vma_dump_size(self, vma, vma_addr):

        vm_start = vma.vm_start
        vm_end = vma.vm_end
        vm_flags = vma.vm_flags
        vm_file = vma.vm_file
        anon_vma = vma.anon_vma
        whole = vm_end - vm_start
        if self.always_dump_vma(vma_addr) == 1:
            return whole
        if vm_flags & VM_DONTDUMP:
            return 0
        # support for DAX
        if self.vma_is_dax(vma):
            if (vm_flags & VM_SHARED is True) and self.FILTER(MMF_DUMP_DAX_SHARED):
                return whole
            if (vm_flags & VM_SHARED is False) and self.FILTER(MMF_DUMP_DAX_PRIVATE):
                return whole
            return 0
        # Hugetlb memory check
        if vm_flags & VM_HUGETLB:
            if (vm_flags & VM_SHARED is True) and self.FILTER(MMF_DUMP_HUGETLB_SHARED):
                return whole
            if (vm_flags & VM_SHARED is False) and self.FILTER(MMF_DUMP_HUGETLB_PRIVATE):
                return whole
            return 0
        # Do not dump I/O mapped devices or special mappings */
        if vm_flags & VM_IO:
            return 0
        # By default, dump shared memory if mapped from an anonymous file.
        if vm_flags & VM_SHARED:
            if self.get_inode_i_nlink_v0(vm_file) == 0:
                res = self.FILTER(MMF_DUMP_ANON_SHARED)
            else:
                res = self.FILTER(MMF_DUMP_MAPPED_SHARED)
            if res is True:
                return whole
            return 0

        if (self.ramdump.is_config_defined("CONFIG_MMU") is False or anon_vma) and self.FILTER(MMF_DUMP_ANON_PRIVATE):
            return whole
        if vm_file is None and vm_file == 0:
            return 0
        if self.FILTER(MMF_DUMP_MAPPED_PRIVATE):
            return whole
        return 0

    def get_inode_i_nlink_v0(self, file):
        path_offset = self.ramdump.field_offset('struct file', 'f_path')
        entry_offset = self.ramdump.field_offset('struct path', 'dentry')
        inode_offset = self.ramdump.field_offset('struct dentry', 'd_inode')
        link_offset = self.ramdump.field_offset('struct inode', 'i_nlink')
        entry = self.ramdump.read_word(file + path_offset + entry_offset)
        inode = self.ramdump.read_word(entry + inode_offset)
        link = self.ramdump.read_int(inode + link_offset)

        file_struct = self.struct_parser.read_struct(file, 'struct file', 'f_inode')
        i_nlink = self.struct_parser.read_struct(file_struct.f_inode, 'struct inode', 'i_nlink')
        return i_nlink


    def always_dump_vma(self, vma_addr):
        if (self.get_gate_vma(vma_addr)):
            return 1

        vma = self.struct_parser.read_struct(vma_addr, "struct vm_area_struct",["vm_ops"])
        vm_ops_addr = vma.vm_ops
        if vm_ops_addr:
            vm_operations_struct = self.struct_parser.read_struct(vm_ops_addr, "struct vm_operations_struct",["name"])
            if "name" not in vm_operations_struct.fields().keys():
                return 0
        return self.arch_vma_name(vma_addr)

    def arch_vma_name(self, vma_addr):
        return "[vectors]" if self.is_gate_vma(vma_addr) else None

    def is_gate_vma(self, vma_addr):
        return self.gate_vma == vma_addr

    def get_gate_vma(self, vma_addr):
        if self.gate_vma is None or self.gate_vma == 0:
            return 0
        else:
            return self.gate_vma == vma_addr

    def vma_is_dax(self,vma):
        host = 0
        try:
            vm_file = vma.fileds["vm_file"]
            file_struct = self.read_struct(vm_file, "struct file", ["f_mapping"])
            address_space = self.read_struct(file_struct.f_mapping, "struct address_space", ["host"])
        except:
            pass
        return host != 0

    def elf_write_vma_data(self,pt_load_header,index):
        '''
        pt_load_header: struct elf64_phdr object
        '''
        p_startvaddr = pt_load_header.p_vaddr
        vma_len = pt_load_header.p_memsz
        if vma_len >= self.large_vma_size:
            self.printe("{1:>016x} ~ {2:>016x}, filesz:{3:x} mb"
                          .format(index, p_startvaddr, p_startvaddr + vma_len, int(vma_len/1024/1024)))

        self.printi("PT_LOAD[{0}]: 0x{1:>016x} ~ 0x{2:>016x}, filesz:0x{3:x}  at offset 0x{4:x}"
                    .format(index, p_startvaddr, p_startvaddr + vma_len, vma_len, self.corefile.tell()))
        data = UTaskLib.read_binary(self.ramdump, self.process_mmu, p_startvaddr, vma_len)
        if not data or len(data) != vma_len:
            error_msg = "reading data size mismatch" if data else "reading data is null"
            self.printi("reading vma failed due to %s" % error_msg)
            data = b'\x00' * vma_len

        self.corefile.write(data)

    def fill_elf_header(self,phnum):
        '''
        phnum: program header number
        '''
        elf64_hdr = self.struct_parser.parser_struct_def("struct elf64_hdr")
        if len(elf64_hdr.e_ident) != 16:
            raise Exception("Failed to parse (struct elf64_hdr) elf64_hdr.e_ident length should be 16, but found %d"
                            % len(elf64_hdr.e_ident))

        e_ident = [0 for x in range(0, 16)]
        e_ident[0] = 0x7f
        e_ident[1] = ord('E')
        e_ident[2] = ord('L')
        e_ident[3] = ord('F')
        # EI_CLASS
        ELFCLASS64 = 2
        e_ident[4] = ELFCLASS64
        # EI_DATA
        ELFDATA2LSB = 1
        ELFDATA2MSB = 2
        e_ident[5] = ELFDATA2LSB
        # EI_VERSION
        EV_CURRENT = 1
        e_ident[6] = EV_CURRENT
        # EI_OSABI
        e_ident[7] = 0

        elf64_hdr.e_ident = e_ident
        elf64_hdr.e_ehsize = self.ramdump.sizeof("struct elf64_hdr")
        elf64_hdr.e_phentsize = self.ramdump.sizeof("struct elf64_phdr")
        elf64_hdr.e_phnum = phnum
        elf64_hdr.e_type = 4
        elf64_hdr.e_machine = self.user_regset_view.e_machine
        elf64_hdr.e_version = 1
        elf64_hdr.e_phoff = elf64_hdr.e_ehsize
        elf64_hdr.e_flags = self.user_regset_view.e_flags
        self.gcore_elf_struct["ehdr"] = elf64_hdr

    def write_elf_header(self):
        elf_hdr = self.gcore_elf_struct["ehdr"]
        if elf_hdr is not None:
            self.corefile.seek(0)
            self.printi("write elf_header at offset 0 \n",elf_hdr)
            self.corefile.write(bytes(elf_hdr))

    def elf_fill_program_header(self,p_type,p_flags,p_offset,p_vaddr,p_filesz,p_memsz,p_align):
        elf_phdr = self.struct_parser.parser_struct_def("struct elf64_phdr")
        elf_phdr.p_type = p_type
        elf_phdr.p_flags = p_flags
        elf_phdr.p_offset = p_offset
        elf_phdr.p_vaddr = p_vaddr
        elf_phdr.p_filesz = p_filesz
        elf_phdr.p_memsz = p_memsz
        elf_phdr.p_align = p_align
        return elf_phdr

    def elf_write_vma_program_header(self,vma_addr,offset,index):
        vma = self.struct_parser.read_struct(vma_addr,"struct vm_area_struct")
        p_memsz = vma.vm_end - vma.vm_start
        if vma.vm_flags & VM_READ:
            p_flags = PF_R
        else:
            p_flags = 0
        if vma.vm_flags & VM_WRITE:
            p_flags |= PF_W
        if vma.vm_flags & VM_EXEC:
            p_flags |= PF_X

        p_filesz = self.vma_dump_size(vma,vma_addr)
        elf_phdr = self.elf_fill_program_header(PT_LOAD, p_flags, offset, vma.vm_start, p_filesz, p_memsz, self.PAGE_SIZE)
        offset += p_filesz
        self.corefile.write(bytes(elf_phdr))
        self.printi("PT_LOAD header[{0}]: {1:>016x} ~ {2:>016x}, filesz:{3:x},offset:{4},flag:{5:x}"
                    .format(index, vma.vm_start, vma.vm_end, p_filesz,offset,vma.vm_flags))
        self.printd(elf_phdr)
        #save PT_LOAD header
        if "pt_load_header" not in self.gcore_elf_struct:
            self.gcore_elf_struct["pt_load_header"] = []
        self.gcore_elf_struct["pt_load_header"].append(elf_phdr)

        return offset

    def elf_fill_note_header(self,n_namesz,n_descsz,n_type):
        elf_nhdr = self.struct_parser.parser_struct_def("struct elf64_note")
        elf_nhdr.n_namesz = n_namesz
        elf_nhdr.n_descsz = n_descsz
        elf_nhdr.n_type = n_type
        return elf_nhdr

    def elf_get_e_phoff(self):
        return self.gcore_elf_struct["ehdr"].e_phoff

    def elf_get_e_shoff(self):
        return self.gcore_elf_struct["ehdr"].e_shoff

    def elf_header_offset(self):
        elf_header = None
        section_header = None
        if "ehdr" in self.gcore_elf_struct.keys():
            elf_header = self.gcore_elf_struct["ehdr"]
            return elf_header.e_ehsize + (elf_header.e_phnum * elf_header.e_phentsize)
        raise Exception("elf header is None")

    #     +-------------------------------------------------------+
    #     |    note header    |   name  | align |  data   | align |
    #     +-------------------------------------------------------+
    def writenote(self,memelfnote,name=""):
        note_start = self.corefile.tell()
        # write note header
        n_namesz = len(memelfnote.name) + 1
        n_descsz = memelfnote.datasz
        n_type = memelfnote.type
        elf_note = self.elf_fill_note_header(n_namesz,n_descsz,n_type)
        data = bytes(elf_note)
        self.corefile.write(data)

        # write note name
        buffer = self.pack_string(memelfnote.name)
        self.corefile.write(buffer)

        self.corefile.write(struct.pack('<b', 0)) # because namesize = len + 1
        self.alignfile(self.corefile.tell())

        # write note data
        self.corefile.write(memelfnote.data)
        self.alignfile(self.corefile.tell())
        note_size = self.corefile.tell() - note_start
        self.elf_note_info.size += note_size
        return note_start, note_size

    def notesize(self,memelfnote):
        note_size = self.ramdump.sizeof("struct elf64_note")
        note_size += self.roundup(len(memelfnote.name) + 1, 4)
        note_size += self.roundup(memelfnote.datasz, 4)
        return note_size

    def alignfile(self,size):
        len = self.roundup(size,4) - size
        for i in range(len):
            self.corefile.write(struct.pack('<b', 0))
        return len

    def roundup(self,x,y):
        return int((((x)+((y)-1))//(y))*(y))

    def fill_note(self,name,type,sz,data):
        memelfnote = self.struct_parser.parser_struct_def("struct memelfnote")
        memelfnote.name = name
        memelfnote.type = type
        memelfnote.datasz = sz
        if sz ==0:
            return memelfnote

        if data:
            if len(data) > sz:
                memelfnote.data = data[0:sz]
            else:
                memelfnote.data =  data + b'\x00' * (sz - len(data))
        else:
            memelfnote.data =  b'\x00' * (sz - len(data))

        return memelfnote

    def thread_group_cputime(self,task_vaddr):
        task = self.struct_parser.read_struct(task_vaddr, "struct task_struct", ["utime", "stime","signal"])
        task_utime = task.utime
        task_stime = task.stime
        signal = self.struct_parser.read_struct(task.signal, "struct signal_struct", ["utime", "stime"])
        signal_utime = signal.utime
        signal_stime = signal.stime
        utime = task_utime + signal_utime
        stime = task_stime + signal_stime
        return utime,stime

    def read_HZ(self):
        saved_auxv_size = self.ramdump.sizeof("unsigned long")
        saved_auxv_offset = self.ramdump.field_offset("struct mm_struct", "saved_auxv")
        i = 0
        while True:
            saved_auxv_vaddr = (self.task.mm + saved_auxv_offset) + (i * saved_auxv_size)
            saved_auxv_paddr = self.ramdump.virt_to_phys(saved_auxv_vaddr)
            saved_auxv_val = int.from_bytes(self.ramdump.read_physical(saved_auxv_paddr, saved_auxv_size),
                                            byteorder='little', signed=False)
            i += 1
            if saved_auxv_val == 17:
                break
        saved_auxv_vaddr = (self.task.mm + saved_auxv_offset) + (i * saved_auxv_size)
        saved_auxv_paddr = self.ramdump.virt_to_phys(saved_auxv_vaddr)
        hz = int.from_bytes(self.ramdump.read_physical(saved_auxv_paddr, saved_auxv_size),
                                        byteorder='little', signed=False)
        return hz

    def cputime_to_timeval(self,cputime):
        timeval = self.struct_parser.parser_struct_def("struct __kernel_old_timeval")
        rem,res = self.div_u64_rem(cputime * self.tick_nsec())
        timeval.tv_sec = res
        timeval.tv_usec = int(rem // NSEC_PER_USEC)
        return timeval.tv_sec, timeval.tv_usec

    def div_u64_rem(self,dividend):
        remainder = int(dividend % NSEC_PER_SEC)
        res = int(dividend // NSEC_PER_SEC)
        return remainder,res

    def sh_div(self,x, y, z):
        return (int(x // y) << z) + ((int(x % y) << z) + y / 2) / y

    def tick_nsec(self):
        HZ = self.read_HZ()
        latch = (CLOCK_TICK_RATE + HZ / 2) / HZ
        act_hz = self.sh_div(CLOCK_TICK_RATE, latch, 8)
        return self.sh_div(1000000 * 1000, act_hz, 8)

    def pid_alive(self,task_addr):
        if self.has_member("struct task_struct","pids"):
            task = self.struct_parser.read_struct(task_addr, "struct task_struct", ["pids"])
            pids_addr = task.pids
            pids_addr += PIDTYPE_PID * self.ramdump.sizeof("struct pid_link")
            pids_addr += self.ramdump.field_offset("struct pid_link","pid")
            pid = self.ramdump.read_pointer(pids_addr)
        else:
            task = self.struct_parser.read_struct(task_addr, "struct task_struct", ["thread_pid"])
            pid = task.thread_pid
        if pid > 0:
            return True
        return False

    def task_pid(self,task_addr):
        if self.ramdump.address_of("pid_nr_ns") is not None:
            try:
                return self.task_pid_nr_ns(task_addr,PIDTYPE_PID)
            except:
                task = self.struct_parser.read_struct(task_addr, "struct task_struct", ["pid"])
                return task.pid
        else:
            task = self.struct_parser.read_struct(task_addr, "struct task_struct", ["pid"])
            return task.pid

    def process_group(self,task_addr):
        if self.ramdump.address_of("pid_nr_ns") is not None:
            try:
                pgrp = self.task_pid_nr_ns(task_addr,PIDTYPE_PGID)
                if pgrp == 0:
                    raise Exception("invalid pgrp")
            except:
                pgrp = 0
                if self.has_member("struct signal_struct","pgrp"):
                    task = self.struct_parser.read_struct(task_addr, "struct task_struct", ["signal"])
                    signal = self.struct_parser.read_struct(task.signal, "struct signal_struct", ["pgrp"])
                    pgrp = signal.pgrp
            return pgrp
        else:
            pgrp = 0
            if self.has_member("struct signal_struct","pgrp"):
                task = self.struct_parser.read_struct(task_addr, "struct task_struct", ["signal"])
                signal = self.struct_parser.read_struct(task.signal, "struct signal_struct", ["pgrp"])
                pgrp = signal.pgr
            return pgrp

    def has_member(self,name,member):
        struct_name = 'struct ' + name
        offset = self.ramdump.field_offset(struct_name,member)
        if offset is not None and offset >= 0 :
            return True
        return False

    def task_session(self,task_addr):
        session = 0
        if self.ramdump.address_of("pid_nr_ns") is not None:
            try:
                return self.task_pid_nr_ns(task_addr,PIDTYPE_SID)
            except:
                session = 0
                if self.has_member("struct signal_struct", "session"):
                    task = self.struct_parser.read_struct(task_addr, "struct task_struct", ["signal"])
                    signal = self.struct_parser.read_struct(task.signal, "struct signal_struct", ["session"])
                    session = signal.session
                return session

        if self.has_member("struct signal_struct", "session"):
            task = self.struct_parser.read_struct(task_addr, "struct task_struct", ["signal"])
            signal = self.struct_parser.read_struct(task.signal, "struct signal_struct", ["session"])
            session = signal.session
        return session

    def task_pid_nr_ns(self,task_addr,type):
        pid_ns = 0
        pids_type_pid = 0
        if self.pid_alive(task_addr):
            task = self.struct_parser.read_struct(task_addr, "struct task_struct", ["nsproxy"])
            nsproxy_vaddr = task.nsproxy
            if self.has_member("struct nsproxy","pid_ns"):
                nsproxy = self.struct_parser.read_struct(nsproxy_vaddr, "struct nsproxy", ["pid_ns"])
                pid_ns = nsproxy.pid_ns
            else:
                nsproxy = self.struct_parser.read_struct(nsproxy_vaddr, "struct nsproxy", ["pid_ns_for_children"])
                pid_ns = nsproxy.pid_ns_for_children
            if type != PIDTYPE_PID:
                task = self.struct_parser.read_struct(task_addr, "struct task_struct", ["group_leader"])
                task_addr = task.group_leader
            if self.has_member("struct task_struct","pids"):
                task = self.struct_parser.read_struct(task_addr, "struct task_struct", ["pids"])
                pids_addr = task.pids
                pids_addr += type * self.ramdump.sizeof("struct pid_link")
                pids_addr += self.ramdump.field_offset("struct pid_link", "pid")
                pids_type_pid = self.ramdump.read_pointer(pids_addr)
            else:
                if type == PIDTYPE_PID:
                    task = self.struct_parser.read_struct(task_addr, "struct task_struct", ["thread_pid"])
                    pids_type_pid = task.thread_pid
                else:
                    task = self.struct_parser.read_struct(task_addr, "struct task_struct", ["signal"])
                    signal_vaddr = task.signal
                    signal_vaddr += self.ramdump.field_offset("struct signal_struct", "pids")
                    signal_vaddr += type * self.ramdump.sizeof("void *")
                    pids_type_pid = self.ramdump.read_int(signal_vaddr)
        return self.pid_nr_ns(pids_type_pid,pid_ns)

    def pid_nr_ns(self,pid_vaddr,pid_ns_vaddr):
        nr = 0
        pid_namespace = self.struct_parser.read_struct(pid_ns_vaddr, "struct pid_namespace", ["level"])
        ns_level = pid_namespace.level
        pid = self.struct_parser.read_struct(pid_vaddr, "struct pid", ["level"])
        pid_level = pid.level
        if pid_vaddr is not None and pid_vaddr > 0 and ns_level <= pid_level:
            upid_vaddr = pid_vaddr \
                         + self.ramdump.field_offset("struct pid","numbers") \
                         + self.ramdump.sizeof("struct upid") * ns_level
            upid = self.struct_parser.read_struct(upid_vaddr, "struct upid", ["ns","nr"])
            if upid.ns == pid_ns_vaddr:
                nr = upid.nr
        return nr

    def get_path_name(self, dentry, vfsmnt):
        tmp_vfsmnt = vfsmnt
        d_parent = dentry
        path_name = ''
        while True:
            tmp_dentry = d_parent
            dentry_struct = self.struct_parser.read_struct(tmp_dentry, 'struct dentry', ['d_name', 'd_parent'])
            d_parent = dentry_struct.d_parent
            hash_len = dentry_struct.d_name.hash_len
            d_len = hash_len >> 32
            d_name_addr = dentry_struct.d_name.name
            if (d_len == 0 or d_name_addr == 0):
                return ''
            d_name = self.ramdump.read_cstring(d_name_addr, d_len)
            if (tmp_dentry != dentry):
                if ((d_len > 1 or d_name != '/') and path_name[0] != '/'):
                    path_name = d_name + '/' + path_name
                else:
                    path_name = d_name + path_name
            else:
                path_name = d_name

            if (tmp_dentry == d_parent and tmp_vfsmnt != 0x0):
                if path_name.find('//') == 0:
                    path_name = path_name[1:]
                mount_mnt = tmp_vfsmnt - 0x20
                mount_struct = self.struct_parser.read_struct(mount_mnt, 'struct mount', ['mnt_parent', 'mnt_mountpoint'])
                d_parent = mount_struct.mnt_mountpoint
                mnt_parent = mount_struct.mnt_parent
                if (mount_mnt == mnt_parent):
                    break
                else:
                    tmp_vfsmnt = mnt_parent + 0x20

            if (tmp_dentry == d_parent and tmp_vfsmnt != 0x0):
                if path_name.find('//') == 0:
                    path_name = path_name[1:]
                mount_mnt = tmp_vfsmnt - 0x20
                mount_struct = self.ramdump.read_struct(mount_mnt, 'mount', ['mnt_parent', 'mnt_mountpoint'])
                d_parent = mount_struct.mnt_mountpoint
                mnt_parent = mount_struct.mnt_parent
                if (mount_mnt == mnt_parent):
                    break
                else:
                    tmp_vfsmnt = mnt_parent + 0x20

            if (dentry == d_parent or d_parent == 0x0):
                break

        if path_name:
            path_name = path_name.replace("/system/apex/com.android.vndk.current","")
            path_name = path_name.replace("/system/apex/com.android.runtime","")
        return path_name

    # ##################################################################
    #                  memelfnote (files)  format
    #     +--------------------------------------------------+
    #     |    count                                         |
    #     +--------------------------------------------------+
    #     |    page_size                                     |
    #     +--------------------------------------------------+
    #     |    array of [COUNT] elements                     |
    #     +--------------------------------------------------+
    #     |          start     <file1>                       |
    #     +--------------------------------------------------+
    #     |          end       <file1>                       |
    #     +--------------------------------------------------+
    #     |          file_ofs  <file1>                       |
    #     +--------------------------------------------------+
    #     |          start     <file2>                       |
    #     +--------------------------------------------------+
    #     |          end       <file2>                       |
    #     +--------------------------------------------------+
    #     |          file_ofs  <file2>                       |
    #     +--------------------------------------------------+
    #     |           ..........                             |
    #     +--------------------------------------------------+
    #     |          file name  <file1>                      |
    #     +--------------------------------------------------+
    #     |          file name  <file2>                      |
    #     +--------------------------------------------------+
    #     |          file name  <.....>                      |
    #     +--------------------------------------------------+
    # ##################################################################
    def fill_files_note(self):
        MAX_FILE_NOTE_SIZE = (4*1024*1024)
        map_count = self.mm.map_count
        size = map_count * 64
        if (size > MAX_FILE_NOTE_SIZE):
            return self.fill_note("CORE", NT_FILE, 0, None)

        vma_count = 0
        vma_list = []
        file_name_list = []
        for vma in self.taskinfo.vmalist:
            if (vma.file == 0x0):
                continue

            vma_index = []
            vma_index.append(vma.vm_start)
            vma_index.append(vma.vm_end)
            vma_index.append(vma.vm_pgoff)
            vma_list.append(vma_index)
            self.printd("{0:x} ~ {1:x} vma_file={2:x} file_name={3:s}"
                        .format(vma.vm_start, vma.vm_end, vma.file, vma.file_name))

            #Part2 - get file name list
            file_struct = self.struct_parser.read_struct(vma.file, 'struct file', ['f_path'])
            dentry_addr = file_struct.f_path.dentry
            vfsmnt = file_struct.f_path.mnt
            if (dentry_addr != 0x0):
                file_name = self.get_path_name(dentry_addr, vfsmnt)
                file_name_list.append(file_name)
            vma_count += 1

        #pack data
        size = 0
        map_count = vma_count
        self.printd("vma count:[%d] PAGE_SIZE:[%d]"%(vma_count, self.PAGE_SIZE))
        data = self.pack_data('long', map_count)
        data += self.pack_data('long', self.PAGE_SIZE)
        size += self.ramdump.sizeof('long') * 2

        #Part1 array of [COUNT] elements
        for index in vma_list:
            data += self.pack_data('unsigned long', index[0])
            data += self.pack_data('unsigned long', index[1])
            data += self.pack_data('unsigned long', index[2])
            size += (self.ramdump.sizeof('long') * len(index))

        #Part2 - save file name list
        for index in file_name_list:
            data += self.pack_string(index + '\0')
            size += len(index) + 1
        return self.fill_note("CORE", NT_FILE, size, data)

    # ##################################################################
    #                  memelfnote (psinfo)  format
    #     +--------------------------------------------------+
    #     |              char   pr_state                     |
    #     +--------------------------------------------------+
    #     |              char   pr_sname                     |
    #     +--------------------------------------------------+
    #     |              char   pr_zomb                      |
    #     +--------------------------------------------------+
    #     |              char   pr_nice                      |
    #     +--------------------------------------------------+
    #     |          unsigned long  pr_flag                  |
    #     +--------------------------------------------------+
    #     |           __kernel_uid_t pr_uid                  |
    #     +--------------------------------------------------+
    #     |           __kernel_uid_t pr_gid                  |                      |
    #     +--------------------------------------------------+
    #     |              pid_t pr_pid                        |
    #     +--------------------------------------------------+
    #     |              pid_t pr_ppid                       |
    #     +--------------------------------------------------+
    #     |              pid_t pr_pgrp                       |
    #     +--------------------------------------------------+
    #     |              pid_t pr_sid                        |
    #     +--------------------------------------------------+
    #     |              char pr_fname[16]                   |
    #     +--------------------------------------------------+
    #     |              char pr_psargs[80]                  |
    #     +--------------------------------------------------+
    # ##################################################################
    def fill_psinfo_note(self):
        elf_prpsinfo = self.struct_parser.parser_struct_def("struct elf_prpsinfo")
        self.printd(elf_prpsinfo)
        task_struct = self.task
        pr_state = 0
        pr_sname = 0
        pr_zomb = 0
        pr_nice = 0
        pr_flag = 0
        pr_uid = 0
        pr_gid = 0
        pr_pid = pr_ppid = pr_pgrp = pr_sid = 0
        pr_psargs = ""
        state = task_struct.__state
        if ( state ):
            pr_state = self.ffz(~state) + 1
        else:
            pr_state = 0

        if (pr_state > 5):
            pr_sname = 0x2E
        else:
            pr_sname = ord("RSDTZW"[pr_state])

        if (pr_sname == ord('Z')):
            pr_zomb = 1
        else:
            pr_zomb = 0

        check_nice = task_struct.static_prio
        pr_nice = check_nice -120

        parent = task_struct.real_parent
        parent_task_struct = self.struct_parser.read_struct(parent,"struct task_struct")
        pr_ppid = parent_task_struct.pid
        pr_pid = task_struct.pid
        pr_pgrp = self.process_group(self.task_vaddr)
        pr_sid = self.task_session(self.task_vaddr)
        pr_flag = task_struct.flags
        cred = task_struct.cred
        if (cred == 0):
            pr_uid = task_struct.uid
            pr_gid = task_struct.gid
        else:
            read_cred = task_struct.real_cred
            offset_uid = self.ramdump.field_offset('struct cred', 'uid')
            offset_gid = self.ramdump.field_offset('struct cred', 'gid')
            pr_uid = self.ramdump.read_u16(read_cred + offset_uid)
            pr_gid = self.ramdump.read_u16(read_cred + offset_gid)

        arg_start = self.mm.arg_start
        arg_end = self.mm.arg_end
        comm_len = arg_end - arg_start
        if comm_len > 80:
            comm_len = 80
        pr_psargs = UTaskLib.read_binary(self.ramdump, self.process_mmu, arg_start, comm_len)

        elf_prpsinfo.pr_state = pr_state
        elf_prpsinfo.pr_sname = pr_sname
        elf_prpsinfo.pr_zomb = pr_zomb
        elf_prpsinfo.pr_nice = pr_nice
        elf_prpsinfo.pr_flag = pr_flag
        elf_prpsinfo.pr_uid = pr_uid

        elf_prpsinfo.pr_gid = pr_gid
        elf_prpsinfo.pr_pid = pr_pid
        elf_prpsinfo.pr_ppid = pr_ppid
        elf_prpsinfo.pr_pgrp = pr_pgrp if pr_pgrp else 0
        elf_prpsinfo.pr_sid = pr_sid
        elf_prpsinfo.pr_fname = task_struct.comm

        for i in range(0, len(pr_psargs)):
            elf_prpsinfo.pr_psargs[i] = pr_psargs[i]
        return self.fill_note("CORE",NT_PRPSINFO, elf_prpsinfo._p_size, bytes(elf_prpsinfo))

    # ##################################################################
    #                  memelfnote (siginfo)  format
    #     +--------------------------------------------------+
    #     |              _si_pad[0]                          |
    #     +--------------------------------------------------+
    #     |              _si_pad[1]                          |
    #     +--------------------------------------------------+
    #     |              ..........                          |
    #     +--------------------------------------------------+
    #     |              _si_pad[31]                         |

    def fill_siginfo_note(self):
        data = bytes()
        siginfo_size = self.ramdump.sizeof("struct siginfo")
        pad = [0 for i in range(siginfo_size)]
        pad[0] = 6
        pad[12]=0x3f
        pad[13]=0x0b

        for item in pad:
            data += struct.pack('<b', item)
        if siginfo_size != len(data):
            self.printi("the lengh of siginfo is not correct ! %d, %d",(siginfo_size, len(data)))
            return
        else:
            self.printd(data.hex())
        return self.fill_note("CORE", NT_SIGINFO, siginfo_size, data)

    def fill_auxv_note(self):
        # init auxv
        saved_auxv_size = self.ramdump.sizeof("unsigned long")
        saved_auxv_offset = self.ramdump.field_offset("struct mm_struct","saved_auxv")
        i = 0
        while True:
            saved_auxv_vaddr = (self.task.mm + saved_auxv_offset) + (i * saved_auxv_size)
            saved_auxv_paddr = self.ramdump.virt_to_phys(saved_auxv_vaddr)
            saved_auxv_val = int.from_bytes(self.ramdump.read_physical(saved_auxv_paddr,saved_auxv_size),
                                            byteorder='little',
                                            signed=False)
            if i % 2 == 0 and saved_auxv_val == AT_NULL:
                i += 2
                break
            i += 2
        saved_auxv_paddr = self.ramdump.virt_to_phys(self.task.mm + saved_auxv_offset)
        saved_auxv_size = i * self.ramdump.sizeof("unsigned long")
        auxv = self.ramdump.read_physical(saved_auxv_paddr,saved_auxv_size)
        return self.fill_note("CORE", NT_AUXV, saved_auxv_size, auxv)

    def fill_thread_prstatus(self,task_vaddr):
        task = self.struct_parser.read_struct(task_vaddr, "struct task_struct", ["group_leader", "utime", "stime",
                                                                    "parent","pending","signal"])
        elf_prstatus = self.struct_parser.parser_struct_def("struct elf_prstatus")
        regset_get_size = int(self.user_regset_view.regsets[0].n) * int(self.user_regset_view.regsets[0].size)
        elf_prstatus.get_field("pr_reg").fill_data(
                    self.user_regset_view.regsets[0]
                        .regset_get(task_vaddr, regset_get_size))
        elf_prstatus.pr_fpvalid = 1
        elf_prstatus.common.pr_info.si_signo = 6
        elf_prstatus.common.pr_info.si_code = 0
        elf_prstatus.common.pr_info.si_errno = 0

        parent_task = task.parent
        elf_prstatus.common.pr_ppid = self.task_pid(parent_task)
        elf_prstatus.common.pr_pid = self.task_pid(task_vaddr)
        elf_prstatus.common.pr_pgrp = self.process_group(task_vaddr)
        elf_prstatus.common.pr_sid = self.task_session(task_vaddr)

        elf_prstatus.common.pr_cursig = 6
        #elf_prstatus.common.pr_sighold = self.task.blocked
        elf_prstatus.common.pr_sighold = 0
        elf_prstatus.common.pr_sigpend = task.pending.signal.sig

        if task.group_leader == task_vaddr:
            utime,stime = self.thread_group_cputime(task_vaddr)
        else:
            utime = task.utime
            stime = task.stime
        signal = self.struct_parser.read_struct(task.signal, "struct signal_struct", ["cutime","cstime"])
        cutime = signal.cutime
        cstime = signal.cstime

        elf_prstatus.common.pr_utime.tv_sec, elf_prstatus.common.pr_utime.tv_usec = self.cputime_to_timeval(utime)
        elf_prstatus.common.pr_stime.tv_sec, elf_prstatus.common.pr_stime.tv_usec = self.cputime_to_timeval(stime)
        elf_prstatus.common.pr_cutime.tv_sec, elf_prstatus.common.pr_cutime.tv_usec = self.cputime_to_timeval(cutime)
        elf_prstatus.common.pr_cstime.tv_sec, elf_prstatus.common.pr_cstime.tv_usec = self.cputime_to_timeval(cstime)

        return elf_prstatus

    def fill_note_info(self):
        if self.elf_note_info is not None:
            return self.elf_note_info
        elf_note_info = self.struct_parser.parser_struct_def("struct elf_note_info")
        elf_note_info.psinfo = self.fill_psinfo_note()
        elf_note_info.signote = self.fill_siginfo_note()
        elf_note_info.auxv = self.fill_auxv_note()
        elf_note_info.files = self.fill_files_note()
        elf_note_info.size = 0
        elf_note_info.thread_notes = 0
        elf_note_info.thread = []
        return elf_note_info

    def fill_write_thread_core_info(self,task_vaddr):
        # first thread prstatus note
        elf_prstatus = self.fill_thread_prstatus(task_vaddr)
        prstatus_note = self.fill_note("CORE", NT_PRSTATUS, elf_prstatus._p_size, bytes(elf_prstatus))
        note_start, note_size = self.writenote(prstatus_note,"REGSET_GPR")
        self.printi("Writing Tid:[%d] NOTE INFO((NT_PRSTATUS) at 0x%x size 0x%x"
                        % (elf_prstatus.common.pr_pid, note_start, note_size))
        self.printd(elf_prstatus)

        if task_vaddr == self.task_vaddr:

            note_start, note_size = self.writenote(self.elf_note_info.psinfo,'psinfo')
            self.printi("Writing Tid:[%d] NOTE INFO(NT_PRPSINFO) at 0x%x size 0x%x"
                        % (elf_prstatus.common.pr_pid, note_start, note_size))

            note_start, note_size = self.writenote(self.elf_note_info.signote, 'signote')
            self.printi("Writing Tid:[%d] NOTE INFO(NT_SIGINFO) at 0x%x size 0x%x"
                        % (elf_prstatus.common.pr_pid, note_start, note_size))

            note_start, note_size = self.writenote(self.elf_note_info.auxv,'auxv')
            self.printi("Writing Tid:[%d] NOTE INFO(NT_AUXV) at 0x%x size 0x%x"
                        % (elf_prstatus.common.pr_pid, note_start, note_size))

            note_start, note_size = self.writenote(self.elf_note_info.files,'files')
            self.printi("Writing Tid:[%d] NOTE INFO(NT_FILE) at 0x%x size 0x%x"
                        % (elf_prstatus.common.pr_pid, note_start, note_size))

        for i in range(1, int(self.user_regset_view.n)):
            user_regset = self.user_regset_view.regsets[i]
            type = int(user_regset.core_note_type)
            size = int(user_regset.n) * int(user_regset.size)
            if user_regset.active:
                data = user_regset.regset_get(task_vaddr, size)
                if int(user_regset.core_note_type) == NT_FPREGSET:
                    note_name = "REGSET_FPR"
                    memelfnote = self.fill_note("CORE", type, size, data)
                else:
                    note_name = "REGSET_VFP"
                    memelfnote = self.fill_note("LINUX", type, size, data)

                note_start, note_size = self.writenote(memelfnote,note_name)
                self.printi("Writing Tid:[%d] NOTE INFO(%s) at 0x%x, size 0x%x"
                            % (elf_prstatus.common.pr_pid, nt_type_to_str(type), note_start, note_size))

    # ##################################################################
    #                 struct elf_note_info format
    #     +-----------------------------------------+
    #     |    memelfnote thread[0]->notes[0]    NT_PRSTATUS   |    struct elf_prstatus
    #     +-----------------------------------------+
    #     |    memelfnote (psinfo)  NT_PRPSINFO     |               struct elf_prpsinfo
    #     +-----------------------------------------+
    #     |    memelfnote (signote) NT_SIGINFO      |               struct siginfo
    #     +-----------------------------------------+
    #     |    memelfnote (auxv)    NT_AUXV         |
    #     +-----------------------------------------+
    #     |    memelfnote (files)   NT_FILE         |
    #     +-----------------------------------------+
    #     |    thread[0]->notes[1]  NT_FPREGSET     |
    #     +-----------------------------------------+
    #     |    thread[0]->notes[2]  NT_ARM_TLS      |
    #     +-----------------------------------------+
    #     |    thread[0]->notes[3]  NT_ARM_HW_BREAK |
    #     +-----------------------------------------+
    #     |    ...................                  |
    #     +-----------------------------------------+
    #     |    thread[1]->notes[0]  NT_PRSTATUS     |
    #     +-----------------------------------------+
    #     |    thread[1]->notes[1]  NT_FPREGSET     |
    #     +-----------------------------------------+
    #     |    thread[1]->notes[2]  NT_ARM_TLS      |
    #     +-----------------------------------------+
    #     |    thread[1]->notes[3]  NT_ARM_HW_BREAK |
    #     +-----------------------------------------+
    #     |                  ....                   |
    #     +-----------------------------------------+
    # ##################################################################
    def fill_write_note_info(self):
        self.elf_note_info = self.fill_note_info()
        for i in range(0, int(self.user_regset_view.n)):
            user_regset = self.user_regset_view.regsets[i]
            if int(user_regset.core_note_type) != 0:
                self.elf_note_info.thread_notes += 1

        self.fill_write_thread_core_info(self.task_vaddr)
        for thread_task in self.ramdump.for_each_thread(self.task_vaddr):
            if thread_task != self.task_vaddr:
                self.fill_write_thread_core_info(thread_task)

    # ##################################################################
    #                  coredump file format
    #     +--------------------------------------------------+
    #     |    Elf header (struct elfhdr)                    |
    #     +--------------------------------------------------+
    #     |    Program headers (struct elf_phdr)     PT_NOTE |
    #     +--------------------------------------------------+
    #     |    VMA Program headers (struct elf_phdr) PT_LOAD |
    #     +--------------------------------------------------+
    #     |                      .......                     |
    #     +--------------------------------------------------+
    #     |    VMA Program headers (struct elf_phdr) PT_LOAD |
    #     +--------------------------------------------------+
    #     |    extra Program headers (struct elf_phdr)       |
    #     +--------------------------------------------------+
    #     |                    note data                     |
    #     +--------------------------------------------------+
    #     |                    vma data                      |
    #     +--------------------------------------------------+
    # ##################################################################

    def parse(self):
        if self.ramdump.is_config_defined("CONFIG_COREDUMP") is False \
            or self.ramdump.is_config_defined("CONFIG_BINFMT_ELF") is False:
            self.printe("Coredump is not supported!!")
            return

        PN_XNUM = 0xffff
        phnum = self.mm.map_count + self.elf_core_extra_phdrs()
        phnum = phnum + 1 #add 1 for note information
        if self.gate_vma is not None:
            phnum = phnum + 1
        if phnum > PN_XNUM:
            phnum = PN_XNUM
        self.printi("program header count", phnum)

        ##### ELF Header ######
        self.fill_elf_header(phnum)
        # Writing ELF header ...
        self.write_elf_header()
        ##### ELF Header ######

        offset = self.elf_header_offset()

        ##### Note info data ######
        # Writing note information
        self.printi("Writing note information at {0}".format(offset))
        self.corefile.seek(offset)
        self.fill_write_note_info()
        self.printi("note total size= {0}".format(self.elf_note_info.size))
        ##### Note info data ######

        ##### PT_NOTE program header ######
        self.printi("Writing PT_NOTE program header at {0}".format(self.elf_get_e_phoff()))
        self.corefile.seek(self.elf_get_e_phoff())
        elf_phdr = self.elf_fill_program_header(PT_NOTE,0,offset,0,self.elf_note_info.size,0,0)
        self.corefile.write(bytes(elf_phdr))
        self.gcore_elf_struct["phdr"] = elf_phdr
        self.printi("PT Note Info header ", elf_phdr)
        ##### PT_NOTE program header ######

        ##### VMA PT_LOAD program header ######
        offset = self.elf_header_offset() + self.elf_note_info.size
        offset = self.roundup(offset, self.PAGE_SIZE)
        self.printi("Writing PT_LOAD program header at {0}".format(offset))
        index = 0
        for vma in self.taskinfo.vmalist:
            offset = self.elf_write_vma_program_header(vma.vm_addr,offset,index)
            index += 1
        ##### VMA PT_LOAD program header ######

        ##### VMA data ######
        offset = self.elf_header_offset() + self.elf_note_info.size
        offset = self.roundup(offset, self.PAGE_SIZE)
        self.corefile.seek(offset)
        self.printi("Writing vma data at {0}".format(offset))

        index = 0
        for header in self.gcore_elf_struct["pt_load_header"]:
            if header.p_filesz > 0:
                self.elf_write_vma_data(header,index)
                index +=1
        ##### VMA data ######

        self.printe("{} saved, size {} mb".format(self.corefile.name, int(os.path.getsize(self.corefile.name)/1024/1024)))
        self.corefile.close()
        self.corefile.close()