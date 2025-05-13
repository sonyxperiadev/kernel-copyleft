#SPDX-License-Identifier: GPL-2.0-only
#Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.

import os
import sys
import struct
import traceback

class Lzo1xParser():

    LZO_E_OK                  = 0
    LZO_E_ERROR               = -1
    LZO_E_OUT_OF_MEMORY       = -2
    LZO_E_NOT_COMPRESSIBLE    = -3
    LZO_E_INPUT_OVERRUN       = -4
    LZO_E_OUTPUT_OVERRUN      = -5
    LZO_E_LOOKBEHIND_OVERRUN  = 6
    LZO_E_EOF_NOT_FOUND       = -7
    LZO_E_INPUT_NOT_CONSUMED  = -8
    LZO_E_NOT_YET_IMPLEMENTED = -9
    LZO_E_INVALID_ARGUMENT    = -10
    MAX_255_COUNT             = 0x10100ff
    MIN_ZERO_RUN_LENGTH       = 4

    M1_MAX_OFFSET = 0x0400
    M2_MAX_OFFSET = 0x0800
    M3_MAX_OFFSET = 0x4000
    def __init__(self, config_have_efficient_unaligned_access=True):
        self.indata = None
        self.oudata = None
        self.ip = 0
        self.op = 0
        self.ou_len = 0
        self.in_len = 0
        self.error = self.LZO_E_ERROR
        self.CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS = config_have_efficient_unaligned_access

    def read_bytes(self, data, offset, len):
        if len == 8:
            s = struct.unpack('<Q', data[offset:offset+8])
            return s[0]
        elif len == 4:
            s = struct.unpack('<I', data[offset:offset+4])
            return s[0]
        elif len == 2:
            s = struct.unpack('<H', data[offset:offset+2])
            return s[0]
        elif len == 1:
            s = struct.unpack('<B', data[offset:offset+1])
            return s[0]
        else:
            exit(-1)

    def HAVE_IP(self, x):
        return self.in_len - self.ip >= x

    def HAVE_OP(self, x):
        return self.ou_len - self.op >= x

    def NEED_IP(self, x):
        if not self.HAVE_IP(x):
            self.input_overrun()

    def NEED_OP(self, x):
        if not self.HAVE_OP(x):
            self.output_overrun()

    def TEST_LB(self,m_pos):
        if m_pos < 0:
            self.lookbehind_overrun()

    def input_overrun(self):
        self.error = self.LZO_E_INPUT_OVERRUN
        self.ou_len = self.op
        raise Exception("LZO_E_INPUT_OVERRUN")

    def output_overrun(self):
        self.error = self.LZO_E_INPUT_OVERRUN
        self.ou_len = self.op
        raise Exception("LZO_E_OUTPUT_OVERRUN")

    def lookbehind_overrun(self):
        self.error = self.LZO_E_LOOKBEHIND_OVERRUN
        self.ou_len = self.op
        raise Exception("LZO_E_LOOKBEHIND_OVERRUN")

    # need to copy one by one
    def COPY(self, odata, op, idata, ip, size):
            idx = 0
            while idx < size:
                odata[op + idx] = idata[ip + idx]
                idx += 1

    def COPY4(self, odata, op, idata, ip):
            self.COPY(odata, op, idata, ip, 4)

    def COPY8(self, odata, op, idata, ip):
        self.COPY(odata, op, idata, ip, 8)

    def match_next(self, next):
        t = next
        if self.CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS:
            if self.HAVE_IP(6) and self.HAVE_OP(4):
                self.COPY4(self.oudata, self.op, self.indata, self.ip)
                self.op += t
                self.ip += t
            else:
                self.NEED_IP(t + 3)
                self.NEED_OP(t)
                self.COPY(self.oudata, self.op, self.indata, self.ip, t)
                self.ip = self.ip + t
                self.op = self.op + t
                t = 0
        else:
                self.NEED_IP(t + 3)
                self.NEED_OP(t)
                self.COPY(self.oudata, self.op, self.indata, self.ip, t)
                self.ip = self.ip + t
                self.op = self.op + t
                t = 0
        return t

    def error_to_str(self, err):
        if err == self.LZO_E_ERROR:
            return "LZO_E_ERROR"
        elif err == self.LZO_E_OK:
            return "LZO_E_OK"
        elif err == self.LZO_E_INPUT_NOT_CONSUMED:
            return "LZO_E_INPUT_NOT_CONSUMED"
        elif err == self.LZO_E_INPUT_OVERRUN:
            return "LZO_E_INPUT_OVERRUN"

    def eof_found(self, t):
        self.ou_len = self.op
        if t != 3:
            self.error = self.LZO_E_ERROR
        elif self.ip == self.in_len:
            self.error = self.LZO_E_OK
        elif self.ip < self.in_len:
            self.error = self.LZO_E_INPUT_NOT_CONSUMED
        else:
            self.error = self.LZO_E_INPUT_OVERRUN
        raise Exception(self.error_to_str(self.error))

    def lzo1x_decompress_safe(self, indata, in_len, oudata, ou_len):
        self.indata = indata
        self.oudata = oudata
        self.in_len = in_len
        self.ou_len = ou_len
        self.ip = 0
        self.op = 0
        copy_literal_run = False
        SIZEOF_CHAR = 1
        m_pos = 0
        next = 0
        state = 0
        if self.in_len < 3:
            self.input_overrun()
        if self.in_len >= 5 and self.read_bytes(self.indata, self.ip, SIZEOF_CHAR) == 17:
            bitstream_version = self.read_bytes(self.indata, (self.ip + SIZEOF_CHAR), SIZEOF_CHAR)
            self.ip += SIZEOF_CHAR * 2
        else:
            bitstream_version = 0

        if self.read_bytes(self.indata, self.ip, SIZEOF_CHAR) > 17:
            t = self.read_bytes(self.indata, self.ip, SIZEOF_CHAR) - 17
            self.ip += SIZEOF_CHAR
            if t < 4:
                next = t
                state = next
                t = self.match_next(next)
            else:
                copy_literal_run = True
                _bc_t = t
                _bc_ip =self.ip
        while True:
            t = self.read_bytes(self.indata, self.ip, SIZEOF_CHAR)
            self.ip += SIZEOF_CHAR
            if t < 16 or copy_literal_run:
                if state == 0 or copy_literal_run:
                    if t == 0 and not copy_literal_run:
                        ip_last = self.ip
                        while self.read_bytes(self.indata, self.ip, SIZEOF_CHAR) == 0:
                            self.ip += 1
                            self.NEED_IP(1)
                        offset = self.ip - ip_last
                        if offset > self.MAX_255_COUNT:
                            self.error = self.LZO_E_ERROR
                            return self.LZO_E_ERROR, self.ou_len

                        offset = (offset << 8) - offset
                        t = t + offset + 15 + self.read_bytes(self.indata, self.ip, SIZEOF_CHAR)
                        self.ip += 1
                    t += 3
                    if copy_literal_run:
                        copy_literal_run = False
                        t = _bc_t
                        self.ip = _bc_ip
                    if self.CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS:
                        if self.HAVE_IP(t + 15) and self.HAVE_OP(t + 15):
                            ie = self.ip + t
                            oe = self.op + t
                            while True:
                                self.COPY8(self.oudata, self.op, self.indata, self.ip)
                                self.ip += 8
                                self.op += 8
                                self.COPY8(self.oudata, self.op, self.indata, self.ip)
                                self.ip += 8
                                self.op += 8

                                if self.ip >= ie:
                                    break
                            self.ip = ie
                            self.op = oe
                        else:
                            self.NEED_OP(t)
                            self.NEED_IP(t + 3)
                            self.COPY(self.oudata, self.op, self.indata, self.ip, t)
                            self.ip = self.ip + t
                            self.op = self.op + t
                            t = 0
                    else:
                            self.NEED_OP(t)
                            self.NEED_IP(t + 3)
                            self.COPY(self.oudata, self.op, self.indata, self.ip, t)
                            self.ip = self.ip + t
                            self.op = self.op + t
                            t = 0
                    state = 4
                    continue

                elif state != 4:
                    next = t & 3
                    m_pos = self.op - 1
                    m_pos -= t >> 2
                    m_pos -= self.read_bytes(self.indata, self.ip, SIZEOF_CHAR) << 2
                    self.ip += 1
                    self.TEST_LB(m_pos)
                    self.NEED_OP(2)
                    self.oudata[self.op] = self.oudata[m_pos]
                    self.oudata[self.op+1] = self.oudata[m_pos+1]
                    self.op += 2
                    state = next
                    t = self.match_next(next)
                    continue
                else:
                    next = t & 3
                    m_pos = self.op - (1 + self.M2_MAX_OFFSET)
                    m_pos -= t >> 2
                    m_pos -= self.read_bytes(self.indata, self.ip, SIZEOF_CHAR) << 2
                    self.ip += 1
                    t = 3
            elif t >= 64:
                next = t & 3
                m_pos = self.op - 1
                m_pos -= (t >> 2) & 7
                m_pos -= self.read_bytes(self.indata, self.ip, SIZEOF_CHAR) << 3
                self.ip += 1
                t = (t >> 5) - 1 + (3 - 1)
            elif t >= 32:
                t = (t & 31) + (3 - 1)
                if t == 2:
                    ip_last = self.ip
                    while self.read_bytes(self.indata, self.ip, SIZEOF_CHAR) == 0:
                        self.ip += 1
                        self.NEED_IP(1)
                    offset = self.ip -ip_last
                    if offset > self.MAX_255_COUNT:
                        self.error = self.LZO_E_ERROR
                        return self.LZO_E_ERROR, self.ou_len

                    offset = (offset << 8) - offset
                    t = t + offset + 31 + self.read_bytes(self.indata, self.ip, SIZEOF_CHAR)
                    self.ip += 1
                    self.NEED_IP(2)
                m_pos = self.op - 1
                next = self.read_bytes(self.indata, self.ip, 2)
                self.ip += 2
                m_pos -= next >> 2
                next &= 3
            else:
                self.NEED_IP(2)
                next = self.read_bytes(self.indata, self.ip, 2)
                if ((next & 0xfffc) == 0xfffc) and ((t & 0xf8) == 0x18) and bitstream_version:
                    self.NEED_IP(3)
                    t &= 7
                    t |= self.indata[self.ip + 2] << 3
                    t += self.MIN_ZERO_RUN_LENGTH
                    self.NEED_OP(t)
                    idx = 0
                    while idx < t:
                        self.oudata[self.op + idx] = 0
                        idx += 1
                    idx = 0

                    self.op += t
                    next &= 3
                    self.ip += 3
                    state = next
                    t = self.match_next(next)
                    continue
                else:
                    m_pos = self.op
                    m_pos -= (t & 8) << 11
                    t = (t & 7) + (3 - 1)
                    if t == 2:
                        ip_last = self.ip
                        while self.read_bytes(self.indata, self.ip, SIZEOF_CHAR) == 0:
                            self.ip += 1
                            self.NEED_IP(1)
                        offset = self.ip - ip_last

                        if offset > self.MAX_255_COUNT:
                            self.error = self.LZO_E_ERROR
                            return self.LZO_E_ERROR, self.ou_len

                        offset = (offset << 8) - offset
                        t += offset + 7 + self.read_bytes(self.indata, self.ip, SIZEOF_CHAR)
                        self.ip += 1
                        self.NEED_IP(2)
                        next = self.read_bytes(self.indata, self.ip, 2)
                    self.ip += 2
                    m_pos -= next >> 2
                    next &= 3
                    if (m_pos == self.op):
                        self.eof_found(t)
                    m_pos -= 0x4000

            self.TEST_LB(m_pos)
            if self.CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS:
                if (self.op - m_pos >= 8):
                    oe = self.op + t
                    if self.HAVE_OP(t + 15):
                        oe = self.op + t
                        while True:
                            self.COPY8(self.oudata, self.op, self.oudata, m_pos)
                            m_pos += 8
                            self.op += 8
                            self.COPY8(self.oudata, self.op, self.oudata, m_pos)
                            m_pos += 8
                            self.op += 8

                            if self.op >= oe:
                                break
                        self.op = oe
                        if self.HAVE_IP(6):
                            state = next
                            self.COPY4(self.oudata, self.op, self.indata, self.ip)
                            self.op += next
                            self.ip += next
                            continue
                    else:
                        self.NEED_OP(t)
                        offset = oe - self.op
                        self.COPY(self.oudata, self.op, self.oudata, m_pos, offset)
                        self.op += offset
                        m_pos += offset
                else:
                    oe = self.op + t
                    self.NEED_OP(t)
                    self.oudata[self.op] = self.oudata[m_pos]
                    self.oudata[self.op+1] = self.oudata[m_pos+1]
                    self.op += 2
                    m_pos += 2
                    offset = oe - self.op
                    self.COPY(self.oudata, self.op, self.oudata, m_pos, offset)
                    self.op += offset
                    m_pos += offset
            else:
                oe = self.op + t
                self.NEED_OP(t)
                self.oudata[self.op] = self.oudata[m_pos]
                self.oudata[self.op+1] = self.oudata[m_pos+1]
                self.op += 2
                m_pos += 2
                offset = oe - self.op
                self.COPY(self.oudata, self.op, self.oudata, m_pos, offset)
                self.op += offset
                m_pos += offset
            state = next
            t = self.match_next(next)
            continue
        self.eof_found(t)

