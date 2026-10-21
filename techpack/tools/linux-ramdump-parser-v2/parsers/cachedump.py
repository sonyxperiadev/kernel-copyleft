# Copyright (c) 2012-2015, 2020 The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import struct

from parser_util import register_parser, RamParser
from print_out import print_out_str

# assuming cache way size of 8, fix this for badger probably
cache_way = 8


def save_l1_dump(ram_dump, cache_base, size):
    with ram_dump.open_file('l1_cache_dump.bin', 'wb') as cache_file:

        for i in range(0, size):
            val = ram_dump.read_byte(cache_base + i, False)
            cache_file.write(struct.pack('<B', val))
        print_out_str('--- Wrote cache dump to l1_cache_dump.bin')


def parse_cache_dump(ram_dump, cache_base):

    magic_num_offset = ram_dump.field_offset(
        'struct l2_cache_dump', 'magic_number')
    version_offset = ram_dump.field_offset('struct l2_cache_dump', 'version')
    line_size_offset = ram_dump.field_offset(
        'struct l2_cache_dump', 'line_size')
    total_lines_offset = ram_dump.field_offset(
        'struct l2_cache_dump', 'total_lines')
    cache_offset_struct = ram_dump.field_offset(
        'struct l2_cache_dump', 'cache')
    l2dcrtr0_offset_struct = ram_dump.field_offset(
        'struct l2_cache_line_dump', 'l2dcrtr0_val')
    l2dcrtr1_offset_struct = ram_dump.field_offset(
        'struct l2_cache_line_dump', 'l2dcrtr1_val')
    cache_line_data_offset_struct = ram_dump.field_offset(
        'struct l2_cache_line_dump', 'cache_line_data')
    cache_line_struct_size = ram_dump.sizeof('struct l2_cache_line_dump')

    magic = ram_dump.read_word(cache_base + magic_num_offset, False)
    version = ram_dump.read_word(cache_base + version_offset, False)
    line_size = ram_dump.read_word(cache_base + line_size_offset, False)
    total_lines = ram_dump.read_word(cache_base + total_lines_offset, False)
    cache = ram_dump.read_word(cache_base + cache_offset_struct, False)

    cache_file = ram_dump.open_file('l2_cache_dump.txt')

    cache_file.write('Magic = {0:x}\n'.format(magic))
    cache_file.write('version = {0:x}\n'.format(version))
    cache_file.write('line size = {0:x}\n'.format(line_size))

    select = 0
    lines = total_lines // cache_way

    header_str = '({0:4},{1:1}) {2:5} {3:8} '.format(
        'Set', 'Way', 'valid', 'Address')
    # currently assumes 32 bit word like everything else...
    for i in range(0, 32):
        header_str = header_str + '{0:8} '.format('Word{0}'.format(i))

    header_str = header_str + '{0:8} {1:8}\n'.format('L2DCRTR0', 'L2DCRTR0')

    cache_ptr = cache_base + cache_offset_struct

    for i in range(0, lines):

        cache_file.write(header_str)

        for j in range(0, cache_way):
            cache_line_ptr = cache_ptr + (i * cache_way + j) * line_size

            l2dcrtr0_val = ram_dump.read_word(
                cache_line_ptr + l2dcrtr0_offset_struct, False)
            l2dcrtr1_val = ram_dump.read_word(
                cache_line_ptr + l2dcrtr1_offset_struct, False)

            # this is valid for krait, will probably need to be more generic

            addr = l2dcrtr1_val & 0xFFFE0000
            addr = addr | (select & 0x0001ff80)
            valid = (l2dcrtr0_val >> 14) & 0x3

            out_str = '({0:4},{1:1}) {2:5} {3:8x} '.format(i, j, valid, addr)

            cache_line_data_ptr = cache_line_ptr + \
                cache_line_data_offset_struct

            for k in range(0, 32):
                out_str = out_str + \
                    '{0:0=8x} '.format(
                        ram_dump.read_word(cache_line_data_ptr + 4 * k, False))

            out_str = out_str + \
                '{0:0=8x} {1:0=8x}\n'.format(l2dcrtr0_val, l2dcrtr1_val)

            cache_file.write(out_str)
            select = select + 0x10

    cache_file.close()
    print_out_str('--- Wrote cache dump to l2_cache_dump.txt')


@register_parser('--print-cache-dump', 'Print L2 cache dump', optional=True)
class CacheDump(RamParser):

    def parse(self):
        if not self.ramdump.is_config_defined('CONFIG_MSM_CACHE_DUMP'):
            print_out_str(
                '!!! Cache dumping was not enabled. No cache will be dumped')
            return

        cache_base_addr = self.ramdump.address_of('l2_dump')
        cache_base = self.ramdump.read_word(cache_base_addr)

        parse_cache_dump(self.ramdump, cache_base)
