# Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import ctypes


def bm(msb, lsb):
    'Creates a bitmask from msb to lsb'
    return int(('1' * (msb - lsb + 1)) + ('0' * lsb), 2)


def bvalsel(msb, lsb, val):
    'Masks and returns the bits from msb to lsb in val'
    return ((val & bm(msb, lsb)) >> lsb)


def is_set(val, bit):
    'Checks whether particular bit is set in val'
    if (val >> bit) & 0x1:
        return True
    return False


def align(x, a):
    """Round x up to the nearest multiple of a"""
    # See include/uapi/linux/kernel.h
    notmask = ctypes.c_uint64(~(a - 1)).value
    return (x + a - 1) & notmask
