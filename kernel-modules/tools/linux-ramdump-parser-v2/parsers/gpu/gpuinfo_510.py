# Copyright (c) 2021 The Linux Foundation. All rights reserved.
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

import linux_list
import linux_radix_tree
import traceback

from math import log2
from parser_util import RamParser
from parsers.gpu.gmu_info import generate_gmu_t32_files
from parsers.gpu.gpu_snapshot import create_snapshot_from_ramdump
from parsers.gpu.gpu_snapshot import extract_gmu_mem_from_snapshot
from parsers.gpu.gpu_eventlog import parse_eventlog_buffer
from print_out import print_out_str


# Global Configurations
ADRENO_DISPATCH_DRAWQUEUE_SIZE = 128
KGSL_DEVMEMSTORE_SIZE = 40
KGSL_PRIORITY_MAX_RB_LEVELS = 4
KGSL_MAX_PWRLEVELS = 16
KGSL_MAX_POOLS = 6
PAGE_SIZE = 4096

KGSL_CACHEMODE_MASK = 0x0C000000
KGSL_CACHEMODE_SHIFT = 26
KGSL_MEMALIGN_MASK = 0x00FF0000
KGSL_MEMALIGN_SHIFT = 16
KGSL_MEMTYPE_MASK = 0x0000FF00
KGSL_MEMTYPE_SHIFT = 8

KGSL_CONTEXT_SECURE = 0x00020000

KGSL_MEMDESC_GLOBAL = (1 << 1)
KGSL_MEMDESC_SECURE = (1 << 4)
KGSL_MEMDESC_PRIVILEGED = (1 << 6)
KGSL_MEMDESC_UCODE = (1 << 7)
KGSL_MEMDESC_RANDOM = (1 << 8)

KGSL_MEMFLAGS_GPUREADONLY = (1 << 24)
KGSL_MEMFLAGS_USE_CPU_MAP = (1 << 28)
KGSL_MEMFLAGS_VBO = (1 << 34)

kgsl_cachemode = ['-', 'u', 't', 'b']

kgsl_ctx_type = ['ANY', 'GL', 'CL', 'C2D', 'RS', 'VK']

kgsl_ctx_priv = [
    ((1 << 0), 's', 'submitted'),           # KGSL_CONTEXT_PRIV_SUBMITTED
    ((1 << 1), 'd', 'detached'),            # KGSL_CONTEXT_PRIV_DETACHED
    ((1 << 2), 'i', 'invalid'),             # KGSL_CONTEXT_PRIV_INVALID
    ((1 << 3), 'p', 'pagefault'),           # KGSL_CONTEXT_PRIV_PAGEFAULT
    ((1 << 16), 'F', 'Fault'),              # ADRENO_CONTEXT_FAULT
    ((1 << 17), 'H', 'GPU Hang'),           # ADRENO_CONTEXT_GPU_HANG
    ((1 << 18), 'T', 'GPU Hang FT'),        # ADRENO_CONTEXT_GPU_HANG_FT
    ((1 << 19), 'E', 'Skip EOF'),           # ADRENO_CONTEXT_SKIP_EOF
    ((1 << 20), 'P', 'Force Preamble'),     # ADRENO_CONTEXT_FORCE_PREAMBLE
    ((1 << 21), 'C', 'Skip CMD'),           # ADRENO_CONTEXT_SKIP_CMD
    ((1 << 22), 'L', 'Fence Log')           # ADRENO_CONTEXT_FENCE_LOG
]

kgsl_memtype = [
                'any(0)',
                'framebuffer',
                'renderbuffer',
                'arraybuffer',
                'elementarraybuffer',
                'vertexarraybuffer',
                'texture',
                'surface',
                'egl_surface',
                'gl',
                'cl',
                'cl_buffer_map',
                'cl_buffer_nomap',
                'cl_image_map',
                'cl_image_nomap',
                'cl_kernel_stack',
                'command',
                '2d',
                'egl_image',
                'egl_shadow',
                'egl_multisample',
                'kernel'
]

adreno_preempt_state = ['NONE', 'START', 'TRIGGERED', 'FAULTED', 'PENDING',
                        'COMPLETE']


def strhex(x): return str(hex(x))
def str_convert_to_kb(x): return str(x//1024) + 'kb'


class GpuParser_510(RamParser):
    def __init__(self, dump):
        super(GpuParser_510, self).__init__(dump)

        # List of all sub-parsers as (func, info, outfile) tuples.
        self.parser_list = [
            (self.parse_kgsl_data, "KGSL", 'gpuinfo.txt'),
            (self.parse_pwrctrl_data, "KGSL Power", 'gpuinfo.txt'),
            (self.parse_kgsl_mem, "KGSL Memory Stats", 'gpuinfo.txt'),
            (self.parse_rb_inflight_data, "Ringbuffer and Inflight Queues",
             'gpuinfo.txt'),
            (self.parse_globals, "Globals", 'gpuinfo.txt'),
            (self.parse_preempt_data, "Preemption", 'gpuinfo.txt'),
            (self.parse_dispatcher_data, "Dispatcher", 'gpuinfo.txt'),
            (self.parse_mutex_data, "KGSL Mutexes", 'gpuinfo.txt'),
            (self.parse_scratch_memory, "Scratch Memory", 'gpuinfo.txt'),
            (self.parse_memstore_memory, "Memstore", 'gpuinfo.txt'),
            (self.parse_context_data, "Open Contexts", 'gpuinfo.txt'),
            (self.parse_active_context_data, "Active Contexts", 'gpuinfo.txt'),
            (self.parse_open_process_data, "Open Processes", 'gpuinfo.txt'),
            (self.parse_pagetables, "Process Pagetables", 'gpuinfo.txt'),
            (self.parse_gmu_data, "GMU Details", 'gpuinfo.txt'),
            (self.dump_gpu_snapshot, "GPU Snapshot", 'gpuinfo.txt'),
            (self.dump_atomic_snapshot, "Atomic GPU Snapshot", 'gpuinfo.txt'),
            (self.parse_fence_data, "Fences", 'gpu_sync_fences.txt'),
            (self.parse_open_process_mementry, "Open Process Mementries",
             'open_process_mementries.txt'),
            (self.parse_eventlog_data, "Eventlog Buffer", 'eventlog.txt'),
            (self.parse_gpu_dcvs_data, "GPU DCVS Info",
             'gpuinfo.txt'),
        ]

        self.rtw = linux_radix_tree.RadixTreeWalker(dump)

    def parse(self):
        self.devp = self.ramdump.read_pointer('kgsl_driver.devp')
        for subparser in self.parser_list:
            try:
                self.out = self.ramdump.open_file('gpu_parser/' + subparser[2],
                                                  'a')
                self.write(subparser[1].center(90, '-') + '\n')
                subparser[0](self.ramdump)
                self.writeln()
                self.out.close()
            except Exception:
                print_out_str("GPU info: Parsing failed in "
                              + subparser[0].__name__)
                print_out_str(traceback.format_exc())

    def write(self, string):
        self.out.write(string)

    def writeln(self, string=""):
        self.out.write(string + '\n')

    def print_context_data(self, ctx_addr, format_str):
        dump = self.ramdump
        context_id = str(dump.read_structure_field(
            ctx_addr, 'struct kgsl_context', 'id'))
        if context_id == "0":
            return

        proc_priv_offset = dump.field_offset('struct kgsl_context',
                                             'proc_priv')
        proc_priv = dump.read_pointer(ctx_addr + proc_priv_offset)
        pid = dump.read_structure_field(proc_priv,
                                        'struct kgsl_process_private', 'pid')
        upid_offset = dump.field_offset('struct pid', 'numbers')
        upid = dump.read_int(pid + upid_offset)

        comm_offset = dump.field_offset('struct kgsl_process_private', 'comm')
        comm = str(dump.read_cstring(proc_priv + comm_offset))
        if comm == '' or upid is None:
            return

        ctx_type = dump.read_structure_field(ctx_addr,
                                             'struct adreno_context', 'type')
        flags = dump.read_structure_field(ctx_addr,
                                          'struct kgsl_context', 'flags')
        priv = dump.read_structure_field(ctx_addr,
                                         'struct kgsl_context', 'priv')
        is_secure = bool(flags & KGSL_CONTEXT_SECURE)
        priv_str = ['-'] * 11
        for i, (bit, char, prop) in enumerate(kgsl_ctx_priv):
            if bool(bit & priv):
                priv_str[i] = char
        priv_str = ''.join(priv_str)

        ktimeline_offset = dump.field_offset('struct kgsl_context',
                                             'ktimeline')
        ktimeline_addr = dump.read_pointer(ctx_addr + ktimeline_offset)
        ktimeline_last_ts = dump.read_structure_field(
                ktimeline_addr, 'struct kgsl_sync_timeline', 'last_timestamp')

        memstore_obj = dump.read_structure_field(self.devp,
                                                 'struct kgsl_device',
                                                 'memstore')
        hostptr = dump.read_structure_field(memstore_obj,
                                            'struct kgsl_memdesc', 'hostptr')
        ctx_memstr_offset = int(context_id) * KGSL_DEVMEMSTORE_SIZE
        soptimestamp = dump.read_s32(hostptr + ctx_memstr_offset)
        eoptimestamp = dump.read_s32(hostptr + ctx_memstr_offset + 8)

        self.writeln(format_str.format(context_id, str(upid), comm,
                     strhex(ctx_addr), kgsl_ctx_type[ctx_type], strhex(flags),
                     str(is_secure), priv_str, str(ktimeline_last_ts),
                     str(soptimestamp), str(eoptimestamp)))

    def parse_context_data(self, dump):
        format_str = '{0:10} {1:10} {2:20} {3:28} {4:12} ' + \
                     '{5:12} {6:12} {7:14} {8:16} {9:14} {10:14}'
        self.writeln(format_str.format("CTX_ID", "PID", "PROCESS_NAME",
                                       "ADRENO_DRAWCTX_PTR", "CTX_TYPE",
                                       "FLAGS", "IS_SECURE", "PRIV",
                                       "TIMELINE_LST_TS", "SOP_TS", "EOP_TS"))
        context_idr = dump.struct_field_addr(self.devp, 'struct kgsl_device',
                                             'context_idr')
        self.rtw.walk_radix_tree(context_idr,
                                 self.print_context_data, format_str)
        self.writeln('\nPriv key:')
        for (bit, char, prop) in kgsl_ctx_priv:
            self.write('\'' + char + '\'' + ': ' + prop + ', ')
        self.writeln()

    def parse_active_context_data(self, dump):
        format_str = '{0:10} {1:10} {2:20} {3:28} {4:12} ' + \
                     '{5:12} {6:12} {7:14} {8:16} {9:14} {10:14}'
        self.writeln(format_str.format("CTX_ID", "PID", "PROCESS_NAME",
                                       "ADRENO_DRAWCTX_PTR", "CTX_TYPE",
                                       "FLAGS", "IS_SECURE", "PRIV",
                                       "TIMELINE_LST_TS", "SOP_TS", "EOP_TS"))
        node_addr = dump.struct_field_addr(self.devp, 'struct adreno_device',
                                           'active_list')
        list_elem_offset = dump.field_offset('struct adreno_context',
                                             'active_node')
        active_context_list_walker = linux_list.ListWalker(dump, node_addr,
                                                           list_elem_offset)
        active_context_list_walker.walk(node_addr,
                                        self.print_context_data, format_str)
        self.writeln('\nPriv key:')
        for (bit, char, prop) in kgsl_ctx_priv:
            self.write('\'' + char + '\'' + ': ' + prop + ', ')
        self.writeln()

    def parse_globals(self, dump):
        format_str = '{0:30} {1:30} {2:30} {3:20} {4:30} {5:12}'
        self.writeln(format_str.format("NAME", "MEMDESC_ADDR", "HOSTPTR",
                                       "MEMDESC_SIZE", "GPUADDR", "FLAGS"))
        node_addr = dump.struct_field_addr(self.devp, 'struct kgsl_device',
                                           'globals')
        list_elem_offset = dump.field_offset('struct kgsl_global_memdesc',
                                             'node')
        globals_list_walker = linux_list.ListWalker(dump, node_addr,
                                                    list_elem_offset)
        globals_list_walker.walk(node_addr, self.__print_global_memdesc,
                                 format_str)

    def __print_global_memdesc(self, kgsl_global_memdesc_base, format_str):
        dump = self.ramdump
        name = dump.read_structure_field(kgsl_global_memdesc_base,
                                         'struct kgsl_global_memdesc', 'name')
        name = dump.read_cstring(name)
        gpuaddr = dump.read_structure_field(kgsl_global_memdesc_base,
                                            'struct kgsl_memdesc', 'gpuaddr')
        if name is None or gpuaddr == 0:
            return

        hostptr = dump.read_structure_field(kgsl_global_memdesc_base,
                                            'struct kgsl_memdesc', 'hostptr')
        size = dump.read_structure_field(kgsl_global_memdesc_base,
                                         'struct kgsl_memdesc', 'size')
        flags = self.prepare_global_memdesc_flags(kgsl_global_memdesc_base)

        self.writeln(format_str.format(name, hex(kgsl_global_memdesc_base),
                                       hex(hostptr), str(size), hex(gpuaddr),
                                       str(flags)))
        if (flags[2] != 's'):
            filename = 'gpu_parser/globals/{0}.bin'.format(
                name + '_' + hex(kgsl_global_memdesc_base))
            file = dump.open_file(filename, 'wb')
            data = dump.read_binarystring(hostptr, size)
            file.write(data)
            file.close()

    def prepare_global_memdesc_flags(self, memdesc_addr):
        '''
            Returns flag string with following legend:
            is_priveleged | is_readonly | is_secure | is_random | is_ucode
                  p/-     |    -/w      |    s/-    |    r/-    |   u/-
        '''
        dump = self.ramdump
        priv = dump.read_structure_field(memdesc_addr,
                                         'struct kgsl_memdesc', 'priv')
        mflags = dump.read_structure_field(memdesc_addr,
                                           'struct kgsl_memdesc', 'flags')
        flags = ['-'] * 5

        if bool(priv & KGSL_MEMDESC_PRIVILEGED):
            flags[0] = 'p'

        if not bool(mflags & KGSL_MEMFLAGS_GPUREADONLY):
            flags[1] = 'w'

        if bool(priv & KGSL_MEMDESC_SECURE):
            flags[2] = 's'

        if bool(priv & KGSL_MEMDESC_RANDOM):
            flags[3] = 'r'

        if bool(priv & KGSL_MEMDESC_UCODE):
            flags[4] = 'u'

        return ''.join(flags)

    def parse_open_process_mementry(self, dump):
        self.writeln('WARNING: Some nodes can be corrupted one, Ignore them.')
        format_str = '{0:20} {1:20} {2:12} {3:30} ' \
                     '{4:20} {5:20} {6:12} {7:20} {8:20}'
        self.writeln(format_str.format("PID", "PNAME", "INDEX", "MEMDESC_ADDR",
                                       "MEMDESC_SIZE", "GPUADDR", "FLAGS",
                                       "USAGE", "PENDING_FREE"))

        node_addr = dump.read('kgsl_driver.process_list.next')
        list_elem_offset = dump.field_offset(
            'struct kgsl_process_private', 'list')
        open_process_list_walker = linux_list.ListWalker(
            dump, node_addr, list_elem_offset)
        open_process_list_walker.walk(
            node_addr, self.__walk_process_mementry, dump, format_str)

    def __walk_process_mementry(self, kgsl_private_base_addr, dump,
                                format_str):
        pid = dump.read_structure_field(kgsl_private_base_addr,
                                        'struct kgsl_process_private', 'pid')
        upid_offset = dump.field_offset('struct pid', 'numbers')
        upid = dump.read_int(pid + upid_offset)

        comm_offset = dump.field_offset('struct kgsl_process_private', 'comm')
        pname = str(dump.read_cstring(kgsl_private_base_addr + comm_offset))
        if pname == '' or upid is None:
            return

        mem_idr_offset = dump.field_offset('struct kgsl_process_private',
                                           'mem_idr')
        mementry_rt = kgsl_private_base_addr + mem_idr_offset
        total_size = [0]

        try:
            self.rtw.walk_radix_tree(mementry_rt, self.__print_mementry_info,
                                     upid, pname, [True], total_size,
                                     format_str)
            self.writeln(format_str.format("", "", "", "",
                         str_convert_to_kb(total_size[0]), "", "", "", ""))
        except Exception:
            self.writeln("Ramdump has a corrupted mementry: pid: " + str(upid)
                         + " comm: " + pname)

    def __print_mementry_info(self, mementry_addr, pid, pname, print_header,
                              total_size, format_str):
        dump = self.ramdump
        memdesc_offset = dump.field_offset('struct kgsl_mem_entry', 'memdesc')
        kgsl_memdesc_address = mementry_addr + memdesc_offset

        size = dump.read_structure_field(kgsl_memdesc_address,
                                         'struct kgsl_memdesc', 'size')
        gpuaddr = dump.read_structure_field(kgsl_memdesc_address,
                                            'struct kgsl_memdesc', 'gpuaddr')
        idr_id = dump.read_structure_field(mementry_addr,
                                           'struct kgsl_mem_entry', 'id')
        mflags = dump.read_structure_field(kgsl_memdesc_address,
                                           'struct kgsl_memdesc', 'flags')
        map_count = dump.read_structure_field(mementry_addr,
                                              'struct kgsl_mem_entry',
                                              'map_count')
        flags = self.prepare_process_memdesc_flags(kgsl_memdesc_address,
                                                   map_count)
        mtype = ((mflags & KGSL_MEMTYPE_MASK) >> KGSL_MEMTYPE_SHIFT)
        try:
            usage = kgsl_memtype[mtype]
        except IndexError:
            usage = "unknown: " + str(mtype)
        pending_free = dump.read_structure_field(mementry_addr,
                                                 'struct kgsl_mem_entry',
                                                 'pending_free')
        total_size[0] += size

        if print_header[0] is True:
            self.writeln(format_str.format(
                str(pid), pname, hex(idr_id), hex(kgsl_memdesc_address),
                str(size), hex(gpuaddr), str(flags), usage, str(pending_free)))
            # Set to False to skip printing pid and pname for the rest
            print_header[0] = False
        else:
            self.writeln(format_str.format(
                "", "", hex(idr_id), hex(kgsl_memdesc_address), str(size),
                hex(gpuaddr), str(flags), usage, str(pending_free)))

    '''
    Returns flag string with following legend:
    is_global|is_readonly|align|cachemode|is_cpu_mapped|mapped|is_secure|is_vbo
      g/-    |    -/w    |L/l/-| u/t/b/- |    p/-      |  Y/N |    s/-  |  v/-
    '''
    def prepare_process_memdesc_flags(self, memdesc_addr, map_count):
        dump = self.ramdump
        priv = dump.read_structure_field(memdesc_addr,
                                         'struct kgsl_memdesc', 'priv')
        mflags = dump.read_structure_field(memdesc_addr,
                                           'struct kgsl_memdesc', 'flags')
        flags = ['-'] * 8

        if bool(priv & KGSL_MEMDESC_GLOBAL):
            flags[0] = 'g'

        if not bool(mflags & KGSL_MEMFLAGS_GPUREADONLY):
            flags[1] = 'w'

        align = ((mflags & KGSL_MEMALIGN_MASK) >> KGSL_MEMALIGN_SHIFT)
        if align >= log2(PAGE_SIZE << 8):
            flags[2] = 'L'
        elif align >= log2(PAGE_SIZE << 4):
            flags[2] = 'l'

        cachemode = ((mflags & KGSL_CACHEMODE_MASK) >> KGSL_CACHEMODE_SHIFT)
        flags[3] = kgsl_cachemode[cachemode]

        if bool(mflags & KGSL_MEMFLAGS_USE_CPU_MAP):
            flags[4] = 'p'

        if bool(map_count):
            flags[5] = 'Y'
        else:
            flags[5] = 'N'

        if bool(priv & KGSL_MEMDESC_SECURE):
            flags[6] = 's'

        if bool(mflags & KGSL_MEMFLAGS_VBO):
            flags[7] = 'v'

        return ''.join(flags)

    def parse_kgsl_data(self, dump):
        open_count = dump.read_structure_field(self.devp, 'struct kgsl_device',
                                               'open_count')
        active_cnt = dump.read_structure_field(self.devp, 'struct kgsl_device',
                                               'active_cnt')
        state = dump.read_structure_field(self.devp,
                                          'struct kgsl_device', 'state')
        requested_state = dump.read_structure_field(self.devp,
                                                    'struct kgsl_device',
                                                    'requested_state')
        ft_policy = dump.read_structure_field(self.devp,
                                              'struct adreno_device',
                                              'ft_policy')
        kgsl_mmu = dump.struct_field_addr(self.devp, 'struct kgsl_device',
                                          'mmu')
        pfpolicy = dump.read_structure_field(kgsl_mmu, 'struct kgsl_mmu',
                                             'pfpolicy')
        long_ib_addr = dump.struct_field_addr(self.devp,
                                              'struct adreno_device',
                                              'long_ib_detect')
        long_ib_detect = dump.read_bool(long_ib_addr)
        lm_addr = dump.struct_field_addr(self.devp,
                                         'struct adreno_device', 'lm_enabled')
        lm_enabled = dump.read_bool(lm_addr)
        acd_enabled_addr = dump.struct_field_addr(self.devp,
                                                  'struct adreno_device',
                                                  'acd_enabled')
        acd_enabled = dump.read_bool(acd_enabled_addr)
        hwcg_enabled_addr = dump.struct_field_addr(self.devp,
                                                   'struct adreno_device',
                                                   'hwcg_enabled')
        hwcg_enabled = dump.read_bool(hwcg_enabled_addr)
        throttling_addr = dump.struct_field_addr(self.devp,
                                                 'struct adreno_device',
                                                 'throttling_enabled')
        throttling_enabled = dump.read_bool(throttling_addr)
        sptp_pc_addr = dump.struct_field_addr(self.devp,
                                              'struct adreno_device',
                                              'sptp_pc_enabled')
        sptp_pc_enabled = dump.read_bool(sptp_pc_addr)
        bcl_enabled_addr = dump.struct_field_addr(self.devp,
                                                  'struct adreno_device',
                                                  'bcl_enabled')
        bcl_enabled = dump.read_bool(bcl_enabled_addr)
        ifpc_count = dump.read_structure_field(self.devp,
                                               'struct adreno_device',
                                               'ifpc_count')
        speed_bin = dump.read_structure_field(self.devp, 'struct kgsl_device',
                                              'speed_bin')
        cur_rb = dump.read_structure_field(self.devp,
                                           'struct adreno_device', 'cur_rb')
        next_rb = dump.read_structure_field(self.devp,
                                            'struct adreno_device', 'next_rb')
        prev_rb = dump.read_structure_field(self.devp,
                                            'struct adreno_device', 'prev_rb')
        cur_rb_id = dump.read_structure_field(cur_rb,
                                              'struct adreno_ringbuffer', 'id')
        next_rb_id = dump.read_structure_field(next_rb,
                                               'struct adreno_ringbuffer',
                                               'id')
        prev_rb_id = dump.read_structure_field(prev_rb,
                                               'struct adreno_ringbuffer',
                                               'id')
        firmware_offset = dump.field_offset('struct adreno_device', 'fw')
        cp_ucode_ver = dump.read_structure_field(self.devp + firmware_offset,
                                                 'struct adreno_firmware',
                                                 'version')

        self.writeln('open_count: ' + str(open_count))
        self.writeln('active_cnt: ' + str(active_cnt))
        self.writeln('state: ' + str(state))
        self.writeln('requested_state: ' + str(requested_state))
        self.writeln('ft_policy: ' + str(ft_policy))
        self.writeln('pfpolicy: ' + str(pfpolicy))
        self.writeln('long_ib_detect: ' + str(long_ib_detect))
        self.writeln('lm_enabled: ' + str(lm_enabled))
        self.writeln('acd_enabled: ' + str(acd_enabled))
        self.writeln('hwcg_enabled: ' + str(hwcg_enabled))
        self.writeln('throttling_enabled: ' + str(throttling_enabled))
        self.writeln('sptp_pc_enabled: ' + str(sptp_pc_enabled))
        self.writeln('bcl_enabled: ' + str(bcl_enabled))
        self.writeln('ifpc_count: ' + str(ifpc_count))
        self.writeln('speed_bin: ' + str(speed_bin))
        self.writeln('cur_rb: ' + strhex(cur_rb))
        self.writeln('cur_rb_id: ' + str(cur_rb_id))
        self.writeln('next_rb: ' + strhex(next_rb))
        self.writeln('next_rb_id: ' + str(next_rb_id))
        self.writeln('prev_rb: ' + strhex(prev_rb))
        self.writeln('prev_rb_id: ' + str(prev_rb_id))
        self.writeln('CP ucode version: ' + strhex(cp_ucode_ver))

    def parse_kgsl_mem(self, dump):
        page_alloc = dump.read('kgsl_driver.stats.page_alloc')
        coherent = dump.read('kgsl_driver.stats.coherent')
        secure = dump.read('kgsl_driver.stats.secure')

        self.writeln('KGSL Total: ' + str_convert_to_kb(secure +
                     page_alloc + coherent))
        self.writeln('\tsecure: ' + str_convert_to_kb(secure))
        self.writeln('\tnon-secure: ' + str_convert_to_kb(page_alloc +
                     coherent))
        self.writeln('\t\tpage_alloc: ' + str_convert_to_kb(page_alloc))
        self.writeln('\t\tcoherent: ' + str_convert_to_kb(coherent))

        pools_base_addr = dump.address_of('kgsl_pools')
        shift = dump.sizeof('struct kgsl_page_pool')
        pool_order = []
        pool_size = []
        order_offset = dump.field_offset('struct kgsl_page_pool', 'pool_order')
        page_count_offset = dump.field_offset('struct kgsl_page_pool',
                                              'page_count')
        for i in range(KGSL_MAX_POOLS):
            p_order = dump.read_int(pools_base_addr + order_offset)
            pool_order.append(p_order)
            page_count = dump.read_int(pools_base_addr + page_count_offset)

            pool_size.append(page_count * (1 << p_order))
            pools_base_addr += shift

        self.writeln('\nKGSL Pool Size: ' +
                     str_convert_to_kb(sum(pool_size)*PAGE_SIZE))
        for i in range(KGSL_MAX_POOLS):
            self.writeln('\t' + str(pool_order[i]) + ' order pool size: ' +
                         str_convert_to_kb(pool_size[i]*PAGE_SIZE))

    def parse_eventlog_data(self, dump):
        parse_eventlog_buffer(self.writeln, dump)

    def parse_preempt_data(self, dump):
        preempt_addr = dump.struct_field_addr(self.devp,
                                              'struct adreno_device',
                                              'preempt')
        state = dump.read_structure_field(preempt_addr,
                                          'struct adreno_preemption', 'state')
        count = dump.read_structure_field(preempt_addr,
                                          'struct adreno_preemption', 'count')
        preempt_level = dump.read_structure_field(preempt_addr,
                                                  'struct adreno_preemption',
                                                  'preempt_level')

        self.writeln('state: ' + adreno_preempt_state[state])
        self.writeln('count: ' + str(count))
        self.writeln('preempt_level: ' + str(preempt_level))

    def parse_dispatcher_queues(self, arr_base, shift, queue_name):
        self.write(queue_name + ': ')
        active_jobs = False
        for i in range(16):
            first = self.ramdump.read_structure_field(
                                arr_base, 'struct llist_head', 'first')
            if first != 0:
                if not active_jobs:
                    self.writeln('')
                self.writeln('\t' + queue_name + '[' + str(i) +
                             '].first: ' + strhex(first))
                active_jobs = True
            arr_base += shift

        if not active_jobs:
            self.writeln('0x0')

    def parse_dispatcher_data(self, dump):
        dispatcher_addr = dump.struct_field_addr(self.devp,
                                                 'struct adreno_device',
                                                 'dispatcher')
        inflight = dump.read_structure_field(dispatcher_addr,
                                             'struct adreno_dispatcher',
                                             'inflight')
        jobs_base_addr = dump.struct_field_addr(dispatcher_addr,
                                                'struct adreno_dispatcher',
                                                'jobs')
        requeue_base_addr = dump.struct_field_addr(dispatcher_addr,
                                                   'struct adreno_dispatcher',
                                                   'requeue')
        fault_counter = dump.read_structure_field(dispatcher_addr,
                                                  'struct adreno_dispatcher',
                                                  'fault')

        self.writeln('inflight: ' + str(inflight))
        shift = dump.sizeof('struct llist_head')
        self.parse_dispatcher_queues(jobs_base_addr, shift, 'jobs')
        self.parse_dispatcher_queues(requeue_base_addr, shift, 'requeue')
        self.writeln('fault_counter: ' + str(fault_counter))

    def parse_rb_inflight_data(self, dump):
        rb_base_addr = dump.struct_field_addr(self.devp,
                                              'struct adreno_device',
                                              'ringbuffers')
        ringbuffers = []
        inflight_queue_result = []

        for i in range(0, KGSL_PRIORITY_MAX_RB_LEVELS):
            ringbuffers_temp = []
            rb_array_index_addr = dump.array_index(
                rb_base_addr, "struct adreno_ringbuffer", i)
            wptr = dump.read_structure_field(rb_array_index_addr,
                                             'struct adreno_ringbuffer',
                                             'wptr')
            _wptr = dump.read_structure_field(rb_array_index_addr,
                                              'struct adreno_ringbuffer',
                                              '_wptr')
            last_wptr = dump.read_structure_field(
                        rb_array_index_addr,
                        'struct adreno_ringbuffer', 'last_wptr')
            id = dump.read_structure_field(rb_array_index_addr,
                                           'struct adreno_ringbuffer', 'id')
            flags = dump.read_structure_field(rb_array_index_addr,
                                              'struct adreno_ringbuffer',
                                              'flags')

            drawctxt_active = dump.read_structure_field(
                rb_array_index_addr, 'struct adreno_ringbuffer',
                'drawctxt_active')

            if drawctxt_active != 0:
                kgsl_context_id = dump.read_structure_field(
                    drawctxt_active, 'struct kgsl_context', 'id')
                proc_priv_val = dump.read_structure_field(
                    drawctxt_active, 'struct kgsl_context', 'proc_priv')
                if proc_priv_val != 0:
                    comm_offset = dump.field_offset(
                        'struct kgsl_process_private', 'comm')
                    process_name = dump.read_cstring(proc_priv_val
                                                     + comm_offset)
                else:
                    process_name = "NULL"
            else:
                kgsl_context_id = "NULL"
                process_name = "NULL"

            dispatch_q_address = rb_array_index_addr + \
                dump.field_offset('struct adreno_ringbuffer', 'dispatch_q')
            inflight = dump.read_structure_field(
                dispatch_q_address, 'struct adreno_dispatcher_drawqueue',
                'inflight')
            cmd_q_address = dispatch_q_address + dump.field_offset(
                'struct adreno_dispatcher_drawqueue', 'cmd_q')
            head = dump.read_structure_field(
                dispatch_q_address, 'struct adreno_dispatcher_drawqueue',
                'head')
            tail = dump.read_structure_field(
                dispatch_q_address, 'struct adreno_dispatcher_drawqueue',
                'tail')

            dispatcher_result = []
            while head is not tail:
                dispatcher_temp = []
                inflight_queue_index_address = dump.array_index(
                    cmd_q_address, 'struct kgsl_drawobj_cmd *', head)
                inflight_queue_index_value = dump.read_word(
                    inflight_queue_index_address)

                if inflight_queue_index_value != 0:
                    global_ts = dump.read_structure_field(
                        inflight_queue_index_value,
                        'struct kgsl_drawobj_cmd', 'global_ts')
                    fault_policy = dump.read_structure_field(
                        inflight_queue_index_value,
                        'struct kgsl_drawobj_cmd', 'fault_policy')
                    fault_recovery = dump.read_structure_field(
                        inflight_queue_index_value,
                        'struct kgsl_drawobj_cmd', 'fault_recovery')

                    base_address = inflight_queue_index_value + \
                        dump.field_offset('struct kgsl_drawobj_cmd', 'base')
                    drawobj_type = dump.read_structure_field(
                        base_address, 'struct kgsl_drawobj', 'type')
                    timestamp = dump.read_structure_field(
                        base_address, 'struct kgsl_drawobj', 'timestamp')
                    flags = dump.read_structure_field(
                        base_address, 'struct kgsl_drawobj', 'flags')
                    context_pointer = dump.read_pointer(dump.field_offset(
                        'struct kgsl_drawobj', 'context')+base_address)
                    context_id = dump.read_structure_field(
                        context_pointer, 'struct kgsl_context', 'id')
                    proc_priv = dump.read_structure_field(
                        context_pointer, 'struct kgsl_context', 'proc_priv')
                    pid = dump.read_structure_field(
                        proc_priv, 'struct kgsl_process_private', 'pid')
                    upid_offset = dump.field_offset('struct pid', 'numbers')
                    upid = dump.read_int(pid + upid_offset)
                else:
                    global_ts = 'NULL'
                    fault_policy = 'NULL'
                    fault_recovery = 'NULL'
                    drawobj_type = 'NULL'
                    timestamp = 'NULL'
                    flags = 'NULL'
                    context_id = 'NULL'
                    pid = 'NULL'

                dispatcher_temp.extend([i, global_ts, fault_policy,
                                        fault_recovery, drawobj_type,
                                        timestamp, flags, context_id, upid])

                dispatcher_result.append(dispatcher_temp)
                head = (head + 1) % ADRENO_DISPATCH_DRAWQUEUE_SIZE

            inflight_queue_result.append(dispatcher_result)

            ringbuffers_temp.append(rb_array_index_addr)
            ringbuffers_temp.append(wptr)
            ringbuffers_temp.append(_wptr)
            ringbuffers_temp.append(last_wptr)
            ringbuffers_temp.append(id)
            ringbuffers_temp.append(flags)
            ringbuffers_temp.append(kgsl_context_id)
            ringbuffers_temp.append(process_name)
            ringbuffers_temp.append(inflight)
            ringbuffers.append(ringbuffers_temp)

        format_str = "{0:20} {1:20} {2:20} {3:20} " \
            "{4:20} {5:20} {6:20} {7:20} {8:20}"

        print_str = format_str.format('INDEX', 'WPTR', '_WPTR', 'LAST_WPTR',
                                      'ID', 'FLAGS', 'KGSL_CONTEXT_ID',
                                      'PROCESS_NAME', 'INFLIGHT')
        self.writeln(print_str)

        index = 0
        for rb in ringbuffers:
            print_str = format_str.format(str(index), str(rb[1]), str(rb[2]),
                                          str(rb[3]), str(rb[4]), str(rb[5]),
                                          str(rb[6]), str(rb[7]), str(rb[8]))
            self.writeln(print_str)
            index = index + 1

        self.writeln()
        self.writeln("Inflight Queues:")

        format_str = "{0:20} {1:20} {2:20} {3:20} {4:20} " \
            "{5:20} {6:20} {7:20} {8:20}"

        print_str = format_str.format("ringbuffer", "global_ts",
                                      "fault_policy", "fault_recovery",
                                      "type", "timestamp", "flags",
                                      "context_id", "pid")
        self.writeln(print_str)

        # Flatten the 3D list to 1D list
        queues = sum(inflight_queue_result, [])
        for queue in queues:
            # Skip if all the entries excluding rb of the queue are empty
            if all([i == 'NULL' for i in queue[1:]]):
                pass

            self.writeln(format_str.format(str(queue[0]), str(queue[1]),
                                           str(queue[2]), str(queue[3]),
                                           str(queue[4]), str(queue[5]),
                                           str(queue[6]), str(queue[7]),
                                           str(queue[8])))

    def parse_pwrctrl_data(self, dump):
        pwrctrl_address = dump.struct_field_addr(self.devp,
                                                 'struct kgsl_device',
                                                 'pwrctrl')
        active_pwrlevel = dump.read_structure_field(pwrctrl_address,
                                                    'struct kgsl_pwrctrl',
                                                    'active_pwrlevel')
        prev_pwrlevel = dump.read_structure_field(pwrctrl_address,
                                                  'struct kgsl_pwrctrl',
                                                  'previous_pwrlevel')
        default_pwrlevel = dump.read_structure_field(pwrctrl_address,
                                                     'struct kgsl_pwrctrl',
                                                     'default_pwrlevel')
        thermal_pwrlevel = dump.read_structure_field(pwrctrl_address,
                                                     'struct kgsl_pwrctrl',
                                                     'thermal_pwrlevel')
        power_flags = dump.read_structure_field(pwrctrl_address,
                                                'struct kgsl_pwrctrl',
                                                'power_flags')
        ctrl_flags = dump.read_structure_field(pwrctrl_address,
                                               'struct kgsl_pwrctrl',
                                               'ctrl_flags')
        min_pwrlevel = dump.read_structure_field(pwrctrl_address,
                                                 'struct kgsl_pwrctrl',
                                                 'min_pwrlevel')
        max_pwrlevel = dump.read_structure_field(pwrctrl_address,
                                                 'struct kgsl_pwrctrl',
                                                 'max_pwrlevel')
        bus_percent_ab = dump.read_structure_field(pwrctrl_address,
                                                   'struct kgsl_pwrctrl',
                                                   'bus_percent_ab')
        bus_width = dump.read_structure_field(pwrctrl_address,
                                              'struct kgsl_pwrctrl',
                                              'bus_width')
        bus_ab_mbytes = dump.read_structure_field(pwrctrl_address,
                                                  'struct kgsl_pwrctrl',
                                                  'bus_ab_mbytes')
        cur_buslevel = dump.read_structure_field(pwrctrl_address,
                                                 'struct kgsl_pwrctrl',
                                                 'cur_buslevel')
        idle_timer = dump.read_structure_field(pwrctrl_address,
                                               'struct kgsl_pwrctrl',
                                               'interval_timeout')
        pwr_levels_result = []
        pwrlevels_base_address = pwrctrl_address + \
            dump.field_offset('struct kgsl_pwrctrl', 'pwrlevels')

        for i in range(0, KGSL_MAX_PWRLEVELS):
            pwr_levels_temp = []
            pwrlevels_array_idx_addr = dump.array_index(
                pwrlevels_base_address, "struct kgsl_pwrlevel", i)
            gpu_freq = dump.read_structure_field(
                pwrlevels_array_idx_addr, 'struct kgsl_pwrlevel', 'gpu_freq')
            bus_freq = dump.read_structure_field(
                pwrlevels_array_idx_addr, 'struct kgsl_pwrlevel', 'bus_freq')
            bus_min = dump.read_structure_field(
                pwrlevels_array_idx_addr, 'struct kgsl_pwrlevel', 'bus_min')
            bus_max = dump.read_structure_field(
                pwrlevels_array_idx_addr, 'struct kgsl_pwrlevel', 'bus_max')
            acd_level = dump.read_structure_field(
                pwrlevels_array_idx_addr, 'struct kgsl_pwrlevel', 'acd_level')
            if acd_level is None:
                acd_level = 0
            pwr_levels_temp.append(pwrlevels_array_idx_addr)
            pwr_levels_temp.append(gpu_freq)
            pwr_levels_temp.append(bus_freq)
            pwr_levels_temp.append(bus_min)
            pwr_levels_temp.append(bus_max)
            pwr_levels_temp.append(acd_level)
            pwr_levels_result.append(pwr_levels_temp)

        self.writeln('pwrctrl_address:  ' + strhex(pwrctrl_address))
        self.writeln('active_pwrlevel:  ' + str(active_pwrlevel))
        self.writeln('previous_pwrlevel:  ' + str(prev_pwrlevel))
        self.writeln('default_pwrlevel:  ' + str(default_pwrlevel))
        self.writeln('thermal_pwrlevel:  ' + str(thermal_pwrlevel))
        self.writeln('power_flags:  ' + strhex(power_flags))
        self.writeln('ctrl_flags:  ' + strhex(ctrl_flags))
        self.writeln('min_pwrlevel:  ' + str(min_pwrlevel))
        self.writeln('max_pwrlevel:  ' + str(max_pwrlevel))
        self.writeln('bus_percent_ab:  ' + str(bus_percent_ab))
        self.writeln('bus_width:  ' + str(bus_width))
        self.writeln('bus_ab_mbytes:  ' + str(bus_ab_mbytes))
        self.writeln('cur_buslevel:  ' + str(cur_buslevel))
        self.writeln('idle_timer:  ' + str(idle_timer))
        self.writeln()

        self.writeln('pwrlevels_base_address:  '
                     + strhex(pwrlevels_base_address))
        format_str = '{0:<20} {1:<20} {2:<20} {3:<20} {4:<20} {5:<20}'
        self.writeln(format_str.format("INDEX", "GPU_FREQ", "BUS_FREQ",
                                       "BUS_MIN", "BUS_MAX", "ACD_LEVEL"))

        index = 0
        for powerlevel in pwr_levels_result:
            print_str = format_str.format(index, powerlevel[1], powerlevel[2],
                                          powerlevel[3], powerlevel[4],
                                          strhex(powerlevel[5]))
            self.writeln(print_str)
            index = index + 1

    def parse_mutex_data(self, dump):
        self.writeln("device_mutex:")
        device_mutex = dump.read_structure_field(self.devp,
                                                 'struct kgsl_device', 'mutex')
        dispatcher_mutex = dump.read_structure_field(self.devp,
                                                     'struct adreno_device',
                                                     'dispatcher')
        mutex_val = dump.read_word(device_mutex)

        if mutex_val:
            tgid = dump.read_structure_field(
                mutex_val, 'struct task_struct', 'tgid')
            comm_add = mutex_val + \
                dump.field_offset('struct task_struct', 'comm')
            comm_val = dump.read_cstring(comm_add)
            self.writeln("tgid: " + str(tgid))
            self.writeln("comm: " + comm_val)
        else:
            self.writeln("UNLOCKED")

        self.writeln()
        self.writeln("dispatcher_mutex:")
        dispatcher_mutex_val = dump.read_word(dispatcher_mutex)

        if dispatcher_mutex_val:
            tgid = dump.read_structure_field(
                dispatcher_mutex_val, 'struct task_struct', 'tgid')
            comm_add = dispatcher_mutex_val + \
                dump.field_offset('struct task_struct', 'comm')
            comm_val = dump.read_cstring(comm_add)
            self.writeln("tgid: " + str(tgid))
            self.writeln("comm: " + str(comm_val))
        else:
            self.writeln("UNLOCKED")

    def parse_scratch_memory(self, dump):
        scratch_obj = dump.read_structure_field(self.devp,
                                                'struct kgsl_device',
                                                'scratch')
        hostptr = dump.read_structure_field(scratch_obj, 'struct kgsl_memdesc',
                                            'hostptr')
        self.write("hostptr:  " + strhex(hostptr) + "\n")

        def add_increment(x): return x + 4

        format_str = '{0:20} {1:20} {2:20}'
        self.writeln(format_str.format("Ringbuffer_id", "RPTR_Value",
                                       "CTXT_RESTORE_ADD"))

        rptr_0 = dump.read_s32(hostptr)
        hostptr = add_increment(hostptr)
        rptr_1 = dump.read_s32(hostptr)
        hostptr = add_increment(hostptr)
        rptr_2 = dump.read_s32(hostptr)
        hostptr = add_increment(hostptr)
        rptr_3 = dump.read_s32(hostptr)
        hostptr = add_increment(hostptr)
        ctxt_0 = dump.read_s32(hostptr)
        hostptr = add_increment(hostptr)
        ctxt_1 = dump.read_s32(hostptr)
        hostptr = add_increment(hostptr)
        ctxt_2 = dump.read_s32(hostptr)
        hostptr = add_increment(hostptr)
        ctxt_3 = dump.read_s32(hostptr)

        self.writeln(format_str.format(str(0), str(rptr_0), strhex(ctxt_0)))
        self.writeln(format_str.format(str(1), str(rptr_1), strhex(ctxt_1)))
        self.writeln(format_str.format(str(2), str(rptr_2), strhex(ctxt_2)))
        self.writeln(format_str.format(str(3), str(rptr_3), strhex(ctxt_3)))

    def parse_memstore_memory(self, dump):
        memstore_obj = dump.read_structure_field(self.devp,
                                                 'struct kgsl_device',
                                                 'memstore')
        hostptr = dump.read_structure_field(memstore_obj,
                                            'struct kgsl_memdesc', 'hostptr')
        size = dump.read_structure_field(memstore_obj,
                                         'struct kgsl_memdesc', 'size')
        preempted = dump.read_s32(hostptr + 16)
        current_context = dump.read_s32(hostptr + 32)

        self.writeln("hostptr: " + strhex(hostptr))
        self.writeln("current_context: " + str(current_context))
        self.writeln("preempted: " + str(preempted) + " [Deprecated]")

        def add_increment(x): return x + 4

        self.writeln("\nrb contexts:")
        format_str = '{0:^20} {1:^20} {2:^20} {3:^20}'
        self.writeln(format_str.format("rb_index", "soptimestamp",
                                       "eoptimestamp", "current_context"))

        # Skip process contexts since their timestamps are
        # displayed in open/active context sections
        hostptr = hostptr + size - (5 * KGSL_DEVMEMSTORE_SIZE) - 8
        for rb_id in range(KGSL_PRIORITY_MAX_RB_LEVELS):
            soptimestamp = dump.read_s32(hostptr)
            hostptr = add_increment(hostptr)
            # skip unused entry
            hostptr = add_increment(hostptr)
            eoptimestamp = dump.read_s32(hostptr)
            hostptr = add_increment(hostptr)
            # skip unused entry
            hostptr = add_increment(hostptr)
            # skip preempted entry
            hostptr = add_increment(hostptr)
            # skip unused entry
            hostptr = add_increment(hostptr)
            # skip ref_wait_ts entry
            hostptr = add_increment(hostptr)
            # skip unused entry
            hostptr = add_increment(hostptr)
            current_context = dump.read_s32(hostptr)
            hostptr = add_increment(hostptr)
            # skip unused entry
            hostptr = add_increment(hostptr)

            self.writeln(format_str.format(str(rb_id), str(soptimestamp),
                                           str(eoptimestamp), current_context))

    def parse_fence_data(self, dump):
        context_idr = dump.struct_field_addr(self.devp,
                                             'struct kgsl_device',
                                             'context_idr')
        self.rtw.walk_radix_tree(context_idr, self.__print_fence_info)
        return

    def __print_fence_info(self, context_addr):
        dump = self.ramdump
        context_id = dump.read_structure_field(context_addr,
                                               'struct kgsl_context', 'id')
        ktimeline_offset = dump.field_offset('struct kgsl_context',
                                             'ktimeline')
        ktimeline_addr = dump.read_pointer(context_addr + ktimeline_offset)

        name_offset = dump.field_offset('struct kgsl_sync_timeline',
                                        'name')
        name_addr = ktimeline_addr + name_offset
        kgsl_sync_timeline_name = dump.read_cstring(name_addr)

        kgsl_sync_timeline_last_ts = dump.read_structure_field(
            ktimeline_addr, 'struct kgsl_sync_timeline',
            'last_timestamp')
        kgsl_sync_timeline_kref_counter = dump.read_byte(
            ktimeline_addr)

        self.writeln("context id: " + str(context_id))
        self.writeln("\t" + "kgsl_sync_timeline_name: "
                     + str(kgsl_sync_timeline_name))
        self.writeln("\t" + "kgsl_sync_timeline_last_timestamp: "
                     + str(kgsl_sync_timeline_last_ts))
        self.writeln("\t" + "kgsl_sync_timeline_kref_counter: "
                     + str(kgsl_sync_timeline_kref_counter))

    def parse_open_process_data(self, dump):
        format_str = '{0:10} {1:20} {2:20} {3:30} {4:20} {5:20} {6:10} {7:20}'
        self.writeln(format_str.format("PID", "PNAME", "PROCESS_PRIVATE_PTR",
                                       "KGSL_PAGETABLE_ADDRESS",
                                       "KGSL_CUR_MEMORY", "DMABUF_CUR_MEMORY",
                                       "CTX_CNT", "CMDLINE STRING"))

        node_addr = dump.read('kgsl_driver.process_list.next')
        list_elem_offset = dump.field_offset(
                            'struct kgsl_process_private', 'list')
        open_process_list_walker = linux_list.ListWalker(
                                    dump, node_addr, list_elem_offset)
        open_process_list_walker.walk(node_addr, self.walk_process_private,
                                      dump, format_str)

    def walk_process_private(self, kgsl_private_base_addr, dump, format_str):
        pid = dump.read_structure_field(
            kgsl_private_base_addr, 'struct kgsl_process_private', 'pid')
        upid_offset = dump.field_offset('struct pid', 'numbers')
        upid = dump.read_int(pid + upid_offset)

        comm_offset = dump.field_offset('struct kgsl_process_private', 'comm')
        pname = dump.read_cstring(kgsl_private_base_addr + comm_offset)
        if pname == '' or upid is None:
            return

        kgsl_pagetable_address = dump.read_structure_field(
            kgsl_private_base_addr, 'struct kgsl_process_private', 'pagetable')

        stats_offset = dump.field_offset('struct kgsl_process_private',
                                         'stats')
        stats_addr = kgsl_private_base_addr + stats_offset

        kgsl_mem = dump.read_slong(stats_addr)
        dmabuf_mem = dump.read_slong(stats_addr + (16 * 4))

        ctxt_count = dump.read_structure_field(kgsl_private_base_addr,
                                               'struct kgsl_process_private',
                                               'ctxt_count')
        cmdline_offset = dump.field_offset('struct kgsl_process_private',
                                           'cmdline')
        cmdline_string = dump.read_cstring(dump.read_pointer(
                                           kgsl_private_base_addr +
                                           cmdline_offset))

        self.writeln(format_str.format(
            str(upid), str(pname), hex(kgsl_private_base_addr),
            hex(kgsl_pagetable_address), str_convert_to_kb(kgsl_mem),
            str_convert_to_kb(dmabuf_mem), str(ctxt_count),
            str(cmdline_string)))

    def parse_pagetables(self, dump):
        format_str = '{0:14} {1:16} {2:20}'
        self.writeln(format_str.format("PID", "pt_base", "ttbr0"))
        node_addr = dump.read('kgsl_driver.pagetable_list.next')
        list_elem_offset = dump.field_offset(
                            'struct kgsl_pagetable', 'list')
        pagetable_list_walker = linux_list.ListWalker(
                                    dump, node_addr, list_elem_offset)
        pagetable_list_walker.walk(node_addr, self.walk_pagetable,
                                   dump, format_str)

    def walk_pagetable(self, pt_base_addr, dump, format_str):
        pid = dump.read_structure_field(
            pt_base_addr, 'struct kgsl_pagetable', 'name')
        if pid == 0 or pid == 1:
            return

        iommu_pt_base_addr = dump.container_of(pt_base_addr,
                                               'struct kgsl_iommu_pt', 'base')
        ttbr0_mask = 0xFFFFFFFFFFFF
        ttbr0_val = dump.read_structure_field(iommu_pt_base_addr,
                                              'struct kgsl_iommu_pt', 'ttbr0')
        pt_base = ttbr0_val & ttbr0_mask

        self.writeln(format_str.format(
            str(pid), strhex(pt_base), strhex(ttbr0_val)))

    def parse_gmu_data(self, dump):
        gmu_core = dump.struct_field_addr(self.devp,
                                          'struct kgsl_device', 'gmu_core')
        gmu_on = dump.read_structure_field(gmu_core,
                                           'struct gmu_core_device', 'flags')
        if not ((gmu_on >> 4) & 1):
            self.writeln('GMU not enabled.')
            return

        gpucore = dump.read_structure_field(self.devp,
                                        'struct adreno_device', 'gpucore')
        gpurev = dump.read_structure_field(gpucore,
                                       'struct adreno_gpu_core', 'gpurev')
        if gpurev >= 0x80000:
            gmu_device = 'struct gen8_gmu_device'
            gmu_dev_addr = dump.sibling_field_addr(self.devp,
                                                   'struct gen8_device',
                                                   'adreno_dev', 'gmu')
        elif gpurev >= 0x70000:
            gmu_device = 'struct gen7_gmu_device'
            gmu_dev_addr = dump.sibling_field_addr(self.devp,
                                                   'struct gen7_device',
                                                   'adreno_dev', 'gmu')
        else:
            gmu_device = 'struct a6xx_gmu_device'
            gmu_dev_addr = dump.sibling_field_addr(self.devp,
                                                   'struct a6xx_device',
                                                   'adreno_dev', 'gmu')
            preall_addr = dump.struct_field_addr(gmu_dev_addr,
                                                 gmu_device, 'preallocations')
            preallocations = dump.read_bool(preall_addr)

        log_stream_addr = dump.struct_field_addr(gmu_dev_addr,
                                                 gmu_device,
                                                 'log_stream_enable')
        log_stream_enable = dump.read_bool(log_stream_addr)

        gmu_fw_ver = dump.read_u32(gmu_dev_addr)
        pwr_fw_ver = dump.read_u32(gmu_dev_addr + 8)
        flags = dump.read_structure_field(gmu_dev_addr, gmu_device, 'flags')
        idle_level = dump.read_structure_field(gmu_dev_addr, gmu_device,
                                               'idle_level')
        global_entries = dump.read_structure_field(gmu_dev_addr, gmu_device,
                                                   'global_entries')
        cm3_fault = dump.read_structure_field(gmu_dev_addr, gmu_device,
                                              'cm3_fault')

        self.writeln('GMU Firmware Version: ' + strhex(gmu_fw_ver))
        self.writeln('Power Firmware Version: ' + strhex(pwr_fw_ver))
        self.writeln()
        self.writeln('idle_level: ' + str(idle_level))
        self.writeln('internal gmu flags: ' + strhex(flags))
        self.writeln('global_entries: ' + str(global_entries))
        if gpurev < 0x70000:
            self.writeln('preallocations: ' + str(preallocations))
        self.writeln('log_stream_enable: ' + str(log_stream_enable))
        self.writeln('cm3_fault: ' + str(cm3_fault))

        domain = dump.read_structure_field(gmu_dev_addr, gmu_device, 'domain')
        arm_smmu = dump.container_of(domain,
                                     'struct arm_smmu_domain', 'domain')
        pgtbl_ops = dump.read_structure_field(arm_smmu,
                                              'struct arm_smmu_domain',
                                              'pgtbl_ops')
        pgtbl_cfg = dump.sibling_field_addr(pgtbl_ops,
                                            'struct io_pgtable', 'ops', 'cfg')
        ttbr0_val = dump.read_structure_field(pgtbl_cfg,
                                              'struct io_pgtable_cfg',
                                              'arm_lpae_s1_cfg.ttbr')
        self.writeln('ttbr0: ' + strhex(ttbr0_val))

        num_clks = dump.read_structure_field(gmu_dev_addr, gmu_device,
                                             'num_clks')
        clks = dump.read_structure_field(gmu_dev_addr, gmu_device, 'clks')
        clk_id_addr = dump.read_structure_field(clks,
                                                'struct clk_bulk_data', 'id')
        clk_id = dump.read_cstring(clk_id_addr)

        self.writeln('num_clks: ' + str(num_clks))
        self.writeln('clock consumer ID: ' + str(clk_id))

        gmu_logs = dump.read_structure_field(gmu_dev_addr, gmu_device,
                                             'gmu_log')
        hostptr = dump.read_structure_field(gmu_logs,
                                            'struct kgsl_memdesc', 'hostptr')
        size = dump.read_structure_field(gmu_logs,
                                         'struct kgsl_memdesc', 'size')

        self.writeln('\nTrace Details:')
        self.writeln('\tStart Address: ' + strhex(hostptr))
        self.writeln('\tSize: ' + str_convert_to_kb(size))

        if size == 0:
            self.writeln('Invalid size. Aborting gmu trace dump.')
            return
        else:
            file = self.ramdump.open_file('gpu_parser/gmu_trace.bin', 'wb')
            self.writeln('Dumping ' + str_convert_to_kb(size) +
                         ' starting from ' + strhex(hostptr) +
                         ' to gmu_trace.bin')
            data = self.ramdump.read_binarystring(hostptr, size)
            file.write(data)
            file.close()

    def dump_gpu_snapshot(self, dump):
        snapshot_faultcount = dump.read_structure_field(self.devp,
                                                        'struct kgsl_device',
                                                        'snapshot_faultcount')
        self.writeln(str(snapshot_faultcount) + ' snapshot fault(s) detected.')

        if snapshot_faultcount == 0:
            self.writeln('No GPU hang, skipping snapshot dumping.')
            return

        snapshot_offset = dump.field_offset('struct kgsl_device', 'snapshot')
        snapshot_memory_offset = dump.field_offset('struct kgsl_device',
                                                   'snapshot_memory')
        snapshot_memory_size = dump.read_u32(self.devp +
                                             snapshot_memory_offset +
                                             dump.sizeof('void *') +
                                             dump.sizeof('dma_addr_t'))
        snapshot_base_addr = dump.read_pointer(self.devp + snapshot_offset)

        if snapshot_base_addr == 0:
            snapshot_memory_ptr = dump.read_pointer(self.devp +
                                                    snapshot_memory_offset)
            if snapshot_memory_ptr is None or snapshot_memory_ptr == 0:
                self.writeln('Snapshot not found.')
                return
            file_name = 'gpu_snapshot_memory.bpmd'
            file = self.ramdump.open_file('gpu_parser/' + file_name, 'wb')
            self.write('Snapshot start not found, ')
            self.writeln('dumping entire region to ' + file_name)
            data = self.ramdump.read_binarystring(snapshot_memory_ptr,
                                                  snapshot_memory_size)
            file.write(data)
            file.close()
            return

        snapshot_start = dump.read_structure_field(
            snapshot_base_addr, 'struct kgsl_snapshot', 'start')
        snapshot_size = dump.read_structure_field(
            snapshot_base_addr, 'struct kgsl_snapshot', 'size')
        snapshot_timestamp = dump.read_structure_field(
            snapshot_base_addr, 'struct kgsl_snapshot', 'timestamp')
        snapshot_process_offset = dump.field_offset('struct kgsl_snapshot',
                                                    'process')
        snapshot_process = dump.read_pointer(snapshot_base_addr +
                                             snapshot_process_offset)
        snapshot_pid = dump.read_structure_field(
            snapshot_process, 'struct kgsl_process_private', 'pid')

        self.writeln('Snapshot Details:')
        self.writeln('\tStart Address: ' + strhex(snapshot_start))
        self.writeln('\tSize: ' + str_convert_to_kb(snapshot_size))
        self.writeln('\tTimestamp: ' + str(snapshot_timestamp))
        self.writeln('\tProcess PID: ' + str(snapshot_pid))

        file_name = 'gpu_snapshot_' + str(snapshot_timestamp) + '.bpmd'
        file = self.ramdump.open_file('gpu_parser/' + file_name, 'wb')

        if snapshot_size == 0:
            self.write('Snapshot freeze not completed.')
            self.writeln('Dumping entire region to ' + file_name)
            data = self.ramdump.read_binarystring(snapshot_start,
                                                  snapshot_memory_size)
        else:
            self.writeln('\nDumping ' + str_convert_to_kb(snapshot_size) +
                         ' starting from ' + strhex(snapshot_start) +
                         ' to ' + file_name)
            data = self.ramdump.read_binarystring(snapshot_start,
                                                  snapshot_size)
        file.write(data)
        file.close()

        extract_gmu_mem_from_snapshot(dump, "gpu_parser/" + file_name)
        generate_gmu_t32_files(dump)

    def dump_atomic_snapshot(self, dump):
        atomic_snapshot_addr = dump.struct_field_addr(self.devp,
                                                      'struct kgsl_device',
                                                      'snapshot_atomic')
        atomic_snapshot = dump.read_bool(atomic_snapshot_addr)
        if not atomic_snapshot:
            self.writeln('No atomic snapshot detected.')
            self.create_mini_snapshot(dump)
            return

        atomic_snapshot_offset = dump.field_offset(
            'struct kgsl_device', 'snapshot_memory_atomic')
        atomic_snapshot_base = dump.read_pointer(self.devp +
                                                 atomic_snapshot_offset)
        atomic_snapshot_size = dump.read_u32(self.devp +
                                             atomic_snapshot_offset + 8)

        if atomic_snapshot_base == 0 or atomic_snapshot_size == 0:
            self.writeln('Invalid atomic snapshot.')
            self.create_mini_snapshot(dump)
            return

        self.writeln('Atomic Snapshot Details:')
        self.writeln('\tStart Address: ' + strhex(atomic_snapshot_base))
        self.writeln('\tSize: ' + str_convert_to_kb(atomic_snapshot_size))

        file = self.ramdump.open_file('gpu_parser/atomic_snapshot.bpmd', 'wb')
        data = self.ramdump.read_binarystring(atomic_snapshot_base,
                                              atomic_snapshot_size)
        self.writeln('\nData dumped to atomic_snapshot.bpmd')
        file.write(data)
        file.close()

    def create_mini_snapshot(self, dump):
        create_snapshot_from_ramdump(self.devp, dump)

    def parse_gpu_dcvs_data(self, dump):
        state = dump.read_structure_field(self.devp,
                                          'struct kgsl_device', 'state')

        # Skip dumping and extraction of the DCVS data if GPU is not
        # in active state.
        if state == 2:
            adreno_tz_data_addr = dump.address_of('adreno_tz_data')

            bin_addr = dump.struct_field_addr(
                adreno_tz_data_addr, 'struct devfreq_msm_adreno_tz_data',
                'bin')
            total_time = dump.read_s64(bin_addr)
            busy_time = dump.read_s64(bin_addr + 8)

            bus_addr = dump.struct_field_addr(
                adreno_tz_data_addr, 'struct devfreq_msm_adreno_tz_data',
                'bus')
            ram_time = dump.read_u64(bus_addr + 8)
            ram_wait = dump.read_u64(bus_addr + 16)

            mod_percent = dump.read_structure_field(
                adreno_tz_data_addr, 'struct devfreq_msm_adreno_tz_data',
                'mod_percent')

            self.writeln("total_time: " + str(total_time))
            self.writeln("busy_time: " + str(busy_time))
            self.writeln("ram_time: " + str(ram_time))
            self.writeln("ram_wait: " + str(ram_wait))
            self.writeln("mod_percent: " + str(mod_percent))
        else:
            self.writeln("DCVS data dump skipped if GPU state is not ACTIVE")
