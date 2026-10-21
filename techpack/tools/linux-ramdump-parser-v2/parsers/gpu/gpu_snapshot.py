# Copyright (c) 2021 The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from ctypes import c_int, c_int64, c_uint, c_ushort, sizeof, Structure

# High word is static, low word is snapshot version ID
SNAPSHOT_MAGIC = 0x504D0002

# Section header
SNAPSHOT_SECTION_MAGIC = 0xABCD

# Snapshot Sections
KGSL_SNAPSHOT_SECTION_OS            =   0x0101
KGSL_SNAPSHOT_SECTION_END           =   0xFFFF
KGSL_SNAPSHOT_SECTION_RB_V2         =   0x0302
KGSL_SNAPSHOT_SECTION_GPU_OBJECT_V2 =   0x0B02
KGSL_SNAPSHOT_SECTION_MEMLIST_V2    =   0x0E02
KGSL_SNAPSHOT_SECTION_MEM_HISTORY   =   0x1202
KGSL_SNAPSHOT_SECTION_IB_V2         =   0x0402
KGSL_SNAPSHOT_SECTION_REGS          =   0x0201
KGSL_SNAPSHOT_SECTION_MVC           =   0x1501
KGSL_SNAPSHOT_SECTION_INDEXED_REGS  =   0x0501
KGSL_SNAPSHOT_SECTION_DEBUG         =   0x0901
KGSL_SNAPSHOT_SECTION_DEBUGBUS      =   0x0A01
KGSL_SNAPSHOT_SECTION_SHADER        =   0x1201
KGSL_SNAPSHOT_SECTION_GMU           =   0x1601
KGSL_SNAPSHOT_SECTION_GMU_MEMORY    =   0x1701

# Ringbuffer
KGSL_RB_SIZE                =   (32 * 1024)
KGSL_RB_DWORDS              =   (KGSL_RB_SIZE >> 2)

# GMU Sections
GMU_SECTION_TYPE_OTHER      =   0
GMU_SECTION_TYPE_HFIMEM     =   1
GMU_SECTION_TYPE_LOG        =   2
GMU_SECTION_TYPE_BWMEM      =   3
GMU_SECTION_TYPE_DEBUG      =   4
GMU_SECTION_TYPE_DCACHE     =   5
GMU_SECTION_TYPE_ICACHE     =   6
GMU_SECTION_TYPE_UNCACHE    =   7
GMU_SECTION_TYPE_REASON     =   8
GMU_SECTION_TYPE_VERSION    =   9

# GMU Memory Sections
SNAPSHOT_GMU_MEM_UNKNOWN    =   0
SNAPSHOT_GMU_MEM_HFI        =   1
SNAPSHOT_GMU_MEM_LOG        =   2
SNAPSHOT_GMU_MEM_BWTABLE    =   3
SNAPSHOT_GMU_MEM_DEBUG      =   4
SNAPSHOT_GMU_MEM_BIN_BLOCK  =   5


# KGSL structures
class kgsl_snapshot_header(Structure):
    _pack_ = 1
    _fields_ = [('magic', c_uint),
                ('gpuid', c_uint),
                ('chipid', c_uint)]


class kgsl_snapshot_section_header(Structure):
    _pack_ = 1
    _fields_ = [('magic', c_ushort),
                ('id', c_ushort),
                ('size', c_uint)]


class kgsl_snapshot_rb_v2(Structure):
    _pack_ = 1
    _fields_ = [('start', c_int),
                ('end', c_int),
                ('rbsize', c_int),
                ('wptr', c_int),
                ('rptr', c_int),
                ('count', c_int),
                ('timestamp_queued', c_uint),
                ('timestamp_retired', c_uint),
                ('gpuaddr', c_int64),
                ('id', c_uint)]


class kgsl_snapshot_gmu_mem(Structure):
    _pack_ = 1
    _fields_ = [('type', c_int),
                ('hostaddr', c_int64),
                ('gmuaddr', c_int64),
                ('gpuaddr', c_int64)]


def gmu_log(devp, dump, chipid):
    if chipid >= 0x7000000:
        gmu_dev = dump.sibling_field_addr(devp, 'struct gen7_device',
                                          'adreno_dev', 'gmu')
        gmu_logs = dump.read_structure_field(gmu_dev,
                                             'struct gen7_gmu_device',
                                             'gmu_log')
    else:
        gmu_dev = dump.sibling_field_addr(devp, 'struct a6xx_device',
                                          'adreno_dev', 'gmu')
        gmu_logs = dump.read_structure_field(gmu_dev,
                                             'struct a6xx_gmu_device',
                                             'gmu_log')
    if dump.kernel_version >= (5, 10, 0):
        gmu_log_hostptr = dump.read_structure_field(gmu_logs,
                                                    'struct kgsl_memdesc',
                                                    'hostptr')
        gmu_log_size = dump.read_structure_field(gmu_logs,
                                                 'struct kgsl_memdesc', 'size')
        gmu_log_gpuaddr = dump.read_structure_field(gmu_logs,
                                                    'struct kgsl_memdesc',
                                                    'gpuaddr')
        # Set gmuaddr to 0 since it is not present in kgsl_memdesc
        gmu_log_gmuaddr = 0
    else:
        gmu_log_hostptr = dump.read_structure_field(gmu_logs,
                                                    'struct gmu_memdesc',
                                                    'hostptr')
        gmu_log_size = dump.read_structure_field(gmu_logs,
                                                 'struct gmu_memdesc', 'size')
        gmu_log_gmuaddr = dump.read_structure_field(gmu_logs,
                                                    'struct gmu_memdesc',
                                                    'gmuaddr')
        gmu_log_gpuaddr = 0
    return (gmu_log_hostptr, gmu_log_size, gmu_log_gmuaddr, gmu_log_gpuaddr)


def hfi_mem(devp, dump, chipid):
    if chipid >= 0x7000000:
        gmu_dev = dump.sibling_field_addr(devp, 'struct gen7_device',
                                          'adreno_dev', 'gmu')
        hfi = dump.struct_field_addr(gmu_dev, 'struct gen7_gmu_device',
                                     'hfi')
        hfi_mem = dump.read_structure_field(hfi, 'struct gen7_hfi',
                                            'hfi_mem')
    else:
        gmu_dev = dump.sibling_field_addr(devp, 'struct a6xx_device',
                                          'adreno_dev', 'gmu')
        hfi = dump.struct_field_addr(gmu_dev, 'struct a6xx_gmu_device',
                                     'hfi')
        hfi_mem = dump.read_structure_field(hfi, 'struct a6xx_hfi',
                                            'hfi_mem')
    if dump.kernel_version >= (5, 10, 0):
        hfi_mem_hostptr = dump.read_structure_field(hfi_mem,
                                                    'struct kgsl_memdesc',
                                                    'hostptr')
        hfi_mem_size = dump.read_structure_field(hfi_mem,
                                                 'struct kgsl_memdesc', 'size')
        hfi_mem_gpuaddr = dump.read_structure_field(hfi_mem,
                                                    'struct kgsl_memdesc',
                                                    'gpuaddr')
        # Set gmuaddr to 0 since it is not present in kgsl_memdesc
        hfi_mem_gmuaddr = 0
    else:
        hfi_mem_hostptr = dump.read_structure_field(hfi_mem,
                                                    'struct gmu_memdesc',
                                                    'hostptr')
        hfi_mem_size = dump.read_structure_field(hfi_mem,
                                                 'struct gmu_memdesc', 'size')
        hfi_mem_gmuaddr = dump.read_structure_field(hfi_mem,
                                                    'struct gmu_memdesc',
                                                    'gmuaddr')
        hfi_mem_gpuaddr = 0
    return (hfi_mem_hostptr, hfi_mem_size, hfi_mem_gmuaddr, hfi_mem_gpuaddr)


def snapshot_gmu_mem_section(devp, dump, chipid, file, hdr_type):
    if hdr_type == SNAPSHOT_GMU_MEM_HFI:
        (gmu_mem_hostptr, gmu_mem_size, gmu_mem_gmuaddr, gmu_mem_gpuaddr) = \
            hfi_mem(devp, dump, chipid)
    elif hdr_type == SNAPSHOT_GMU_MEM_LOG:
        (gmu_mem_hostptr, gmu_mem_size, gmu_mem_gmuaddr, gmu_mem_gpuaddr) = \
            gmu_log(devp, dump, chipid)
    else:
        return

    section_header = kgsl_snapshot_section_header()
    section_header.magic = SNAPSHOT_SECTION_MAGIC
    section_header.id = KGSL_SNAPSHOT_SECTION_GMU_MEMORY
    section_header.size = (gmu_mem_size + sizeof(kgsl_snapshot_gmu_mem) +
                           sizeof(kgsl_snapshot_section_header))
    file.write(section_header)

    mem_hdr = kgsl_snapshot_gmu_mem()
    mem_hdr.type = hdr_type
    mem_hdr.hostaddr = gmu_mem_hostptr
    mem_hdr.gmuaddr = gmu_mem_gmuaddr
    mem_hdr.gpuaddr = gmu_mem_gpuaddr
    file.write(mem_hdr)

    data = dump.read_binarystring(gmu_mem_hostptr, gmu_mem_size)
    file.write(data)


def snapshot_rb_section(devp, dump, file, rb_type):
    # Scratch buffer information
    scratch_obj = dump.read_structure_field(devp,
                                            'struct kgsl_device',
                                            'scratch')
    scratch_hostptr = dump.read_structure_field(scratch_obj,
                                                'struct kgsl_memdesc',
                                                'hostptr')

    # Memstore information
    memstore_obj = dump.read_structure_field(devp,
                                             'struct kgsl_device',
                                             'memstore')
    memstore_hostptr = dump.read_structure_field(memstore_obj,
                                                 'struct kgsl_memdesc',
                                                 'hostptr')

    # RB information
    rb = dump.read_structure_field(devp,
                                   'struct adreno_device', rb_type)
    if (not rb):
        return

    rb_id = dump.read_structure_field(rb,
                                      'struct adreno_ringbuffer',
                                      'id')
    rb_wptr = dump.read_structure_field(rb,
                                        'struct adreno_ringbuffer',
                                        'wptr')
    rb_rptr = dump.read_s32(scratch_hostptr + rb_id * 4)
    rb_queued_ts = dump.read_structure_field(rb,
                                             'struct adreno_ringbuffer',
                                             'timestamp')
    rb_buffer_desc = dump.read_structure_field(rb,
                                               'struct adreno_ringbuffer',
                                               'buffer_desc')
    rb_gpuaddr = dump.read_structure_field(rb_buffer_desc,
                                           'struct kgsl_memdesc',
                                           'gpuaddr')
    rb_hostptr = dump.read_structure_field(rb_buffer_desc,
                                           'struct kgsl_memdesc',
                                           'hostptr')
    rb_size = dump.read_structure_field(rb_buffer_desc,
                                        'struct kgsl_memdesc', 'size')
    rb_retired_ts = dump.read_s32(memstore_hostptr +
                                  ((rb_id + 0x32E) * 0x28 + 0x8))

    # RB section
    section_header = kgsl_snapshot_section_header()
    section_header.magic = SNAPSHOT_SECTION_MAGIC
    section_header.id = KGSL_SNAPSHOT_SECTION_RB_V2
    section_header.size = (KGSL_RB_SIZE + sizeof(kgsl_snapshot_rb_v2) +
                           sizeof(kgsl_snapshot_section_header))
    file.write(section_header)

    rb_header = kgsl_snapshot_rb_v2()
    rb_header.start = 0
    rb_header.end = KGSL_RB_DWORDS
    rb_header.wptr = rb_wptr
    rb_header.rptr = rb_rptr
    rb_header.rbsize = KGSL_RB_DWORDS
    rb_header.count = KGSL_RB_DWORDS
    rb_header.timestamp_queued = rb_queued_ts
    rb_header.timestamp_retired = rb_retired_ts
    rb_header.gpuaddr = rb_gpuaddr
    rb_header.id = rb_id
    file.write(rb_header)

    data = dump.read_binarystring(rb_hostptr, rb_size)
    file.write(data)


def create_snapshot_from_ramdump(devp, dump):
    # GPU revision
    gpucore = dump.read_structure_field(devp,
                                        'struct adreno_device', 'gpucore')
    gpurev = dump.read_structure_field(gpucore,
                                       'struct adreno_gpu_core', 'gpurev')

    # Gpu chip id
    chipid = dump.read_structure_field(devp, 'struct adreno_device', 'chipid')

    file_name = 'mini_snapshot.bpmd'
    file = dump.open_file('gpu_parser/' + file_name, 'wb')

    # Dump snapshot header
    header = kgsl_snapshot_header()
    header.magic = SNAPSHOT_MAGIC
    header.gpuid = (0x0003 << 16) | gpurev
    header.chipid = chipid
    file.write(header)

    # Dump RBs
    snapshot_rb_section(devp, dump, file, 'cur_rb')
    snapshot_rb_section(devp, dump, file, 'prev_rb')

    # Check & dump GMU info
    gmu_core = dump.struct_field_addr(devp, 'struct kgsl_device', 'gmu_core')
    gmu_on = dump.read_structure_field(gmu_core,
                                       'struct gmu_core_device', 'flags')
    if ((gmu_on >> 4) & 1):
        snapshot_gmu_mem_section(devp,
                                 dump, chipid, file, SNAPSHOT_GMU_MEM_HFI)
        snapshot_gmu_mem_section(devp,
                                 dump, chipid, file, SNAPSHOT_GMU_MEM_LOG)

    # Dump last section
    last_section = kgsl_snapshot_section_header()
    last_section.magic = SNAPSHOT_SECTION_MAGIC
    last_section.id = KGSL_SNAPSHOT_SECTION_END
    last_section.size = sizeof(kgsl_snapshot_section_header)
    file.write(last_section)

    file.close()
