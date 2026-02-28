# Copyright (c) 2013-2015, 2020-2021 The Linux Foundation. All rights reserved.
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
import string

from print_out import print_out_str
from collections import namedtuple
from parser_util import register_parser, RamParser

ELF32HEADERFORMAT = '<16sHHIIIIIHHHHHH'
ELF32HEADERSIZE = struct.calcsize(ELF32HEADERFORMAT)
ELF64HEADERFORMAT = '<16sHHIQQQIHHHHHH'
ELF64HEADERSIZE = struct.calcsize(ELF64HEADERFORMAT)
PRG32HEADERFORMAT = 'IIIIIIII'
PRG32HEADERSIZE = struct.calcsize(PRG32HEADERFORMAT)
PRG64HEADERFORMAT = 'IIQQQQQQ'
PRG64HEADERSIZE = struct.calcsize(PRG64HEADERFORMAT)
PF_W = 2
OUTPUT_SIZE=256
LUMP_SIZE=4096

@register_parser('--check-rodata', 'check rodata in dump against the static image')
class ROData(RamParser):

    def parse(self):

        with self.ramdump.open_file('roareadiff.txt') as roarea_out:

            if self.ramdump.arm64:
                elfheaderformat = ELF64HEADERFORMAT
                elfheadersize = ELF64HEADERSIZE
                prgheaderformat = PRG64HEADERFORMAT
                prgheadersize = PRG64HEADERSIZE
            else:
                elfheaderformat = ELF32HEADERFORMAT
                elfheadersize = ELF32HEADERSIZE
                prgheaderformat = PRG32HEADERFORMAT
                prgheadersize = PRG32HEADERSIZE

            fd = open(self.ramdump.vmlinux, 'rb')
            if not fd:
                print_out_str('Could not open {0}.'.format(file_path))
                return

            ElfHeader = namedtuple(
                'ElfHeader', 'ident type machine version entry phoff shoff flags ehsize phentsize phnum shentsize shnum shstrndx')
            raw_elfheader = fd.read(elfheadersize)
            elfheader = ElfHeader._make(
                struct.unpack(elfheaderformat, raw_elfheader))

            if self.ramdump.arm64:
                PrgHeader = namedtuple(
                    'Prgheader', 'type flags offset vaddr paddr filesz memsz align')
            else:
                PrgHeader = namedtuple(
                    'Prgheader', 'type offset vaddr paddr filesz memsz flags align')
            for i in range(elfheader.phnum):
                fd.seek(elfheader.phoff + i * prgheadersize)
                raw_prgheader = fd.read(prgheadersize)
                prgheader = PrgHeader._make(
                    struct.unpack(prgheaderformat, raw_prgheader))

                if not prgheader.flags & PF_W:
                    fd.seek(prgheader.offset)
                    count = prgheader.paddr
                    while count < prgheader.paddr + prgheader.memsz:
                        if prgheader.paddr + prgheader.memsz - count < LUMP_SIZE:
                            max_read_size = prgheader.paddr + prgheader.memsz - count
                        else:
                            max_read_size = LUMP_SIZE

                        ram_values = self.ramdump.read_physical(self.ramdump.virt_to_phys(count), max_read_size)
                        vm_values = fd.read(max_read_size)
                        if ram_values and ram_values != vm_values:
                            detect = 0xFFFFFFFF
                            print_out_str(
                                'Differences found! Differences written to roareadiff.txt')
                            i = 0
                            while i < max_read_size:
                                try:
                                    ram_value = struct.unpack_from('I', ram_values, i)[0]
                                    vm_value = struct.unpack_from('I', vm_values, i)[0]

                                    if detect == 64:
                                        ddr_str += ddr_ascii + '\n\n'
                                        vmlinux_str += vm_ascii + '\n\n'
                                        roarea_out.write(ddr_str)
                                        roarea_out.write(vmlinux_str)
                                        detect = 0xFFFFFFFF
                                    if detect == 0xFFFFFFFF and ram_value != vm_value:
                                        ddr_str = 'detect RO area differences between vmlinux and DDR at 0x{0:0>8x}\n'.format(
                                            count + i)
                                        ddr_str += 'from DDR:'
                                        vmlinux_str = 'from vmlinux:'
                                        ddr_ascii = '  '
                                        vm_ascii = '  '
                                        detect = 0
                                        if max_read_size < i + OUTPUT_SIZE and max_read_size == LUMP_SIZE:
                                            max_read_size = i + OUTPUT_SIZE
                                            ram_values = self.ramdump.read_physical(self.ramdump.virt_to_phys(count), max_read_size)
                                            fd.seek(prgheader.offset + count - prgheader.paddr)
                                            vm_values = fd.read(max_read_size)
                                    if 0 <= detect and detect < 64:
                                        if detect % 8 == 0:
                                            ddr_str += ddr_ascii + '\n{0:0>8x} '.format(count + i)
                                            vmlinux_str += vm_ascii + '\n{0:0>8x} '.format(count + i)
                                            ddr_ascii = '  '
                                            vm_ascii = '  '
                                        ddr_str += ' '
                                        vmlinux_str += ' '
                                        if ram_value != vm_value:
                                            ddr_str += '*'
                                            vmlinux_str += '*'
                                        else:
                                            ddr_str += ' '
                                            vmlinux_str += ' '
                                        ddr_str += '{0:0>8x}'.format(ram_value)
                                        vmlinux_str += '{0:0>8x}'.format(vm_value)
                                        for j in range(4):
                                            c = '{0:c}'.format(struct.unpack_from('B', ram_values, i + j)[0]).rstrip()
                                            ddr_ascii += c if c in string.printable else '.'
                                            c = '{0:c}'.format(struct.unpack_from('B', vm_values, i + j)[0]).rstrip()
                                            vm_ascii += c if c in string.printable else '.'
                                        detect += 1
                                    i = i + 4
                                except Exception as err:
                                    roarea_out.write("Failed roaddress = {0}".format(count))
                            if detect != 0xFFFFFFFF:
                                ddr_str += '          ' * (7 - ((detect - 1) % 8)) + ddr_ascii + '\n\n'
                                vmlinux_str += '          ' * (7 - ((detect - 1) % 8)) + vm_ascii + '\n\n'
                                roarea_out.write(ddr_str)
                                roarea_out.write(vmlinux_str)
                        count += max_read_size
            fd.close()
