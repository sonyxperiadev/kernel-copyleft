# Copyright (c) 2014, 2020 The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

"""
This module provides some convenience variables and functions for
working with common sizes.

>>> get_size_string(SZ_4K)
'4K'
>>> get_size_string(SZ_2G)
'2G'
>>> get_size_string(SZ_2G * 3)
'2G*3'
>>> get_size_string(SZ_2G + 1)
'2147483649'
>>> order_size_strings[8]
'1M'

"""

import math
try:
    from collections import OrderedDict
except ImportError:
    from ordereddict import OrderedDict

SZ_1   = 0x00000001
SZ_2   = 0x00000002
SZ_4   = 0x00000004
SZ_8   = 0x00000008
SZ_16  = 0x00000010
SZ_32  = 0x00000020
SZ_64  = 0x00000040
SZ_128 = 0x00000080
SZ_256 = 0x00000100
SZ_512 = 0x00000200

SZ_1K   = 0x00000400
SZ_2K   = 0x00000800
SZ_4K   = 0x00001000
SZ_8K   = 0x00002000
SZ_16K  = 0x00004000
SZ_32K  = 0x00008000
SZ_64K  = 0x00010000
SZ_128K = 0x00020000
SZ_256K = 0x00040000
SZ_512K = 0x00080000

SZ_1M   = 0x00100000
SZ_2M   = 0x00200000
SZ_4M   = 0x00400000
SZ_8M   = 0x00800000
SZ_16M  = 0x01000000
SZ_32M  = 0x02000000
SZ_64M  = 0x04000000
SZ_128M = 0x08000000
SZ_256M = 0x10000000
SZ_512M = 0x20000000

SZ_1G = 0x40000000
SZ_2G = 0x80000000
SZ_256G = 0x8000000000

size_strings = OrderedDict((
    (SZ_1, "1"),
    (SZ_2, "2"),
    (SZ_4, "4"),
    (SZ_8, "8"),
    (SZ_16, "16"),
    (SZ_32, "32"),
    (SZ_64, "64"),
    (SZ_128, "128"),
    (SZ_256, "256"),
    (SZ_512, "512"),
    (SZ_1K, "1K"),
    (SZ_2K, "2K"),
    (SZ_4K, "4K"),
    (SZ_8K, "8K"),
    (SZ_16K, "16K"),
    (SZ_32K, "32K"),
    (SZ_64K, "64K"),
    (SZ_128K, "128K"),
    (SZ_256K, "256K"),
    (SZ_512K, "512K"),
    (SZ_1M, "1M"),
    (SZ_2M, "2M"),
    (SZ_4M, "4M"),
    (SZ_8M, "8M"),
    (SZ_16M, "16M"),
    (SZ_32M, "32M"),
    (SZ_64M, "64M"),
    (SZ_128M, "128M"),
    (SZ_256M, "256M"),
    (SZ_512M, "512M"),
    (SZ_1G, "1G"),
    (SZ_2G, "2G"),
    (SZ_256G, "256G"),
))

def is_power_of_two(n):
    """
    >>> is_power_of_two(1024)
    True
    >>> is_power_of_two(0)
    False
    >>> is_power_of_two(1 << 9)
    True
    """
    return n != 0 and ((n & (n - 1)) == 0)

def get_size_string(size):
    """
    >>> get_size_string(4096)
    '4K'
    >>> get_size_string(0x02000000)
    '32M'
    >>> get_size_string(24576)
    '8K*3'
    >>> get_size_string(393216)
    '128K*3'
    """
    if size in size_strings:
        return size_strings[size]
    for s in reversed(size_strings):
        if s < SZ_1K: break
        if (size % s) == 0:
            b = size // s
            return "%s*%d" % (size_strings[s], b)
    return "%d" % size

def get_order(size):
    """
    >>> get_order(SZ_4K)
    0
    >>> get_order(SZ_8K)
    1
    >>> get_order(SZ_16K)
    2
    >>> get_order(SZ_1M)
    8
    >>> get_order(SZ_2M)
    9
    >>> get_order(SZ_2G)
    19
    """
    if not is_power_of_two(size):
        raise ValueError('ERROR: Number is not a power of 2: %x' % (size))
    return int(math.log(size, 2) - math.log(SZ_4K, 2))

order_size_strings = {}
for order in range(20):
    order_size_strings[order] = size_strings[SZ_4K << order]

if __name__ == "__main__":
    import doctest
    doctest.testmod()
