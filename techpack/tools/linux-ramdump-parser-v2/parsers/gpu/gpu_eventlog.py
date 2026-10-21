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

from ctypes import c_uint32, c_uint64, c_char, sizeof, Structure

MAGIC = 0xabbaabba

LOG_SKIP = 1
LOG_FIRE_EVENT = 2
LOG_CMDBATCH_SUBMITTED_EVENT = 3
LOG_CMDBATCH_RETIRED_EVENT = 4
LOG_SYNCPOINT_FENCE_EVENT = 5
LOG_SYNCPOINT_FENCE_EXPIRE_EVENT = 6
LOG_TIMELINE_FENCE_ALLOC_EVENT = 7
LOG_TIMELINE_FENCE_RELEASE_EVENT = 8

KGSL_EVENT_RETIRED = 1
KGSL_EVENT_CANCELLED = 2

NANO_TO_SEC = 1000000000


class fire_event (Structure):
    _fields_ = [('id', c_uint32),
                ('ts', c_uint32),
                ('type', c_uint32),
                ('age', c_uint32)]


class cmdbatch_submitted (Structure):
    _fields_ = [('id', c_uint32),
                ('ts', c_uint32),
                ('prio', c_uint32),
                ('flags', c_uint64)]


class cmdbatch_retired (Structure):
    _fields_ = [('id', c_uint32),
                ('ts', c_uint32),
                ('prio', c_uint32),
                ('flag', c_uint64),
                ('start', c_uint64),
                ('retire', c_uint64)]


class syncpoint_fence (Structure):
    _fields_ = [('id', c_uint32),
                ('name', c_char*74)]


class timeline_fence (Structure):
    _fields_ = [('id', c_uint32),
                ('seqno', c_uint64)]


def fire_event_write(func_writeln, dump, kgsl_eventlog_buffer, header):
    if header["payload_size"] != sizeof(fire_event):
        return True
    id = dump.read_u32(kgsl_eventlog_buffer + fire_event.id.offset)
    ts = dump.read_u32(kgsl_eventlog_buffer + fire_event.ts.offset)
    type = dump.read_u32(kgsl_eventlog_buffer + fire_event.type.offset)
    age = dump.read_u32(kgsl_eventlog_buffer + fire_event.age.offset)
    if type == KGSL_EVENT_RETIRED:
        type = "retired"
    elif type == KGSL_EVENT_CANCELLED:
        type = "cancelled"

    format_str = "pid:{0:<12}{1:<16.6f}" \
                 "kgsl_fire_event: ctx={2} ts={3} type={4} age={5}"
    func_writeln(format_str.format(header["pid"], header["time"],
                                   id, ts, type, age))
    return False


def cmdbatch_submitted_event_write(func_writeln, dump, kgsl_eventlog_buffer,
                                   header):
    if header["payload_size"] != sizeof(cmdbatch_submitted):
        return True
    id = dump.read_u32(kgsl_eventlog_buffer + cmdbatch_submitted.id.offset)
    ts = dump.read_u32(kgsl_eventlog_buffer + cmdbatch_submitted.ts.offset)
    prio = dump.read_u32(kgsl_eventlog_buffer +
                         cmdbatch_submitted.prio.offset)
    flags = dump.read_u64(kgsl_eventlog_buffer +
                          cmdbatch_submitted.flags.offset)
    format_str = "pid:{0:<12}{1:<16.6f}" \
                 "adreno_cmdbatch_submitted: ctx={2} ctx_prio={3} " \
                 "ts={4} flags={5}"
    func_writeln(format_str.format(header["pid"], header["time"],
                                   id, prio, ts, flags))
    return False


def cmdbatch_retired_event_write(func_writeln, dump, kgsl_eventlog_buffer,
                                 header):
    if header["payload_size"] != sizeof(cmdbatch_retired):
        return True
    id = dump.read_u32(kgsl_eventlog_buffer + cmdbatch_retired.id.offset)
    ts = dump.read_u32(kgsl_eventlog_buffer + cmdbatch_retired.ts.offset)
    prio = dump.read_u32(kgsl_eventlog_buffer + cmdbatch_retired.prio.offset)
    flag = dump.read_u64(kgsl_eventlog_buffer + cmdbatch_retired.flag.offset)
    start = dump.read_u64(kgsl_eventlog_buffer +
                          cmdbatch_retired.start.offset)
    retire = dump.read_u64(kgsl_eventlog_buffer +
                           cmdbatch_retired.retire.offset)
    format_str = "pid:{0:<12}{1:<16.6f}" \
                 "adreno_cmdbatch_retired: ctx={2} ctx_prio={3} " \
                 "ts={4} flags={5} start={6} retire={7}"
    func_writeln(format_str.format(header["pid"], header["time"], id,
                 prio, ts, flag, start, retire))
    return False


def syncpoint_fence_event_write(func_writeln, dump, kgsl_eventlog_buffer,
                                header):
    if header["payload_size"] != sizeof(syncpoint_fence):
        return True
    id = dump.read_u32(kgsl_eventlog_buffer + syncpoint_fence.id.offset)
    name = dump.read_cstring(kgsl_eventlog_buffer +
                             syncpoint_fence.name.offset, 74)
    if header["event_id"] == LOG_SYNCPOINT_FENCE_EVENT:
        event = "syncpoint_fence"
    elif header["event_id"] == LOG_SYNCPOINT_FENCE_EXPIRE_EVENT:
        event = "syncpoint_fence_expire"

    format_str = "pid:{0:<12}{1:<16.6f}" \
                 "{2}: ctx={3} name={4}"
    func_writeln(format_str.format(header["pid"], header["time"],
                                   event, id, name))
    return False


def timeline_fence_event_write(func_writeln, dump, kgsl_eventlog_buffer,
                               header):
    if header["payload_size"] != sizeof(timeline_fence):
        return True
    id = dump.read_u32(kgsl_eventlog_buffer + timeline_fence.id.offset)
    seqno = dump.read_u64(kgsl_eventlog_buffer + timeline_fence.seqno.offset)
    if header["event_id"] == LOG_TIMELINE_FENCE_ALLOC_EVENT:
        event = "kgsl_timeline_fence_alloc"
    elif header["event_id"] == LOG_TIMELINE_FENCE_RELEASE_EVENT:
        event = "kgsl_timeline_fence_release"

    format_str = "pid:{0:<12}{1:<16.6f}" \
                 "{2}: timeline={3} seqno={4}"
    func_writeln(format_str.format(header["pid"], header["time"], event,
                                   id, seqno))
    return False


def parse_eventlog_buffer(func_writeln, dump):
    kgsl_eventlog_buffer = dump.read('kgsl_eventlog')
    format_str = '{0:<15} {1:<16}{2}'
    func_writeln(format_str.format("PID", "Timestamp", "Function"))
    min_timestamp = None
    offset = 0
    ret = True
    header = {}
    while offset < 8192:
        header["magic_num"] = dump.read_structure_field(
                              kgsl_eventlog_buffer, 'struct kgsl_log_header',
                              'magic')
        header["event_id"] = dump.read_structure_field(
                             kgsl_eventlog_buffer,
                             'struct kgsl_log_header', 'eventid')
        header["time"] = dump.read_structure_field(
                         kgsl_eventlog_buffer,
                         'struct kgsl_log_header', 'time') / NANO_TO_SEC
        header["pid"] = dump.read_structure_field(
                        kgsl_eventlog_buffer,
                        'struct kgsl_log_header', 'pid')
        header["payload_size"] = dump.read_structure_field(
                                 kgsl_eventlog_buffer,
                                 'struct kgsl_log_header', 'size')

        if (header["magic_num"] != MAGIC) or (header["event_id"] > 8) or \
           (header["event_id"] < 1):
            offset += 1
            kgsl_eventlog_buffer += 1
            continue

        if min_timestamp is None:
            min_timestamp = header["time"]

        kgsl_eventlog_buffer += dump.sizeof('struct kgsl_log_header')
        if min_timestamp > header["time"]:
            func_writeln("End of logging".center(90, '-') + '\n')
            min_timestamp = header["time"]
        if header["event_id"] == LOG_SKIP:
            break
        elif header["event_id"] == LOG_FIRE_EVENT:
            ret = fire_event_write(func_writeln, dump, kgsl_eventlog_buffer,
                                   header)
        elif header["event_id"] == LOG_CMDBATCH_SUBMITTED_EVENT:
            ret = cmdbatch_submitted_event_write(func_writeln, dump,
                                                 kgsl_eventlog_buffer, header)
        elif header["event_id"] == LOG_CMDBATCH_RETIRED_EVENT:
            ret = cmdbatch_retired_event_write(func_writeln, dump,
                                               kgsl_eventlog_buffer, header)
        elif (header["event_id"] == LOG_SYNCPOINT_FENCE_EVENT) or \
             (header["event_id"] == LOG_SYNCPOINT_FENCE_EXPIRE_EVENT):
            ret = syncpoint_fence_event_write(func_writeln, dump,
                                              kgsl_eventlog_buffer, header)
        elif (header["event_id"] == LOG_TIMELINE_FENCE_ALLOC_EVENT) or \
             (header["event_id"] == LOG_TIMELINE_FENCE_RELEASE_EVENT):
            ret = timeline_fence_event_write(func_writeln, dump,
                                             kgsl_eventlog_buffer, header)
        if ret:
            offset += 1
            kgsl_eventlog_buffer += 1
            continue

        offset += dump.sizeof('struct kgsl_log_header') + \
            header["payload_size"]
        kgsl_eventlog_buffer += header["payload_size"]
