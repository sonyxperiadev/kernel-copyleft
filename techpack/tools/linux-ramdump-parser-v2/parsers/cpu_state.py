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

from itertools import cycle

from parser_util import register_parser, RamParser
from print_out import print_out_str


@register_parser('--cpu-state', "Reads register values of non-panic'ing CPUs")
class CpuState(RamParser):

    def parse(self):
        regs_before_stop_addr = self.ramdump.address_of('regs_before_stop')
        if regs_before_stop_addr is None:
            print_out_str('regs_before_stop not found. Nothing to do.')
            return

        # see pt_regs and associated #defines in
        # arch/arm/include/asm/ptrace.h
        regs = (
            'r0',
            'r1',
            'r2',
            'r3',
            'r4',
            'r5',
            'r6',
            'r7',
            'r8',
            'r9',
            'r10',
            'fp',
            'ip',
            'sp',
            'lr',
            'pc',
            'cpsr',
        )

        max_len = max([len(s) for s in regs])

        for cpu in self.ramdump.iter_cpus():
            print_out_str('CPU %d' % cpu)
            lines = []
            for index, reg in enumerate(regs):
                reg_addr = self.ramdump.array_index(
                    regs_before_stop_addr, 'unsigned long', index)
                reg_val = self.ramdump.read_word(reg_addr, cpu=cpu)
                lines.append(
                    '   {0:{width}} = 0x{1:x}'.format(reg, reg_val, width=max_len))

            c = cycle([', ', ', ', ', ', '\n'])
            output = ''
            for line in lines:
                output += line + next(c)
            print_out_str(output)
