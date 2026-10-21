# Copyright (c) 2016-2018,2021 The Linux Foundation. All rights reserved.

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.


import re
import os
from print_out import print_out_str

scandump_registers_32 = [
    ('r0', 'r0', False),
    ('r1', 'r1', False),
    ('r2', 'r2', False),
    ('r3', 'r3', False),
    ('r4', 'r4', False),
    ('r5', 'r5', False),
    ('r6', 'r6', False),
    ('r7', 'r7', False),
    ('r8', 'r8', False),
    ('r9', 'r9', False),
    ('r10', 'r10', False),
    ('r11', 'r11', False),
    ('r12', 'r12', False),
    ('r13_usr', 'r13_usr', False),
    ('r14_usr', 'r14_usr', False),
    ('r13_hyp', 'r13_hyp', False),
    ('r14_irq', 'r14_irq', True),
    ('r13_irq', 'r13_irq', False),
    ('r14_svc', 'r14_svc', True),
    ('r13_svc', 'r13_svc', False),
    ('r14_abt', 'r14_abt', True),
    ('r13_abt', 'r13_abt', False),
    ('r14_und', 'r14_und', True),
    ('r13_und', 'r13_und', False),
    ('r8_fiq', 'r8_fiq', False),
    ('r9_fiq', 'r9_fiq', False),
    ('r10_fiq', 'r10_fiq', False),
    ('r11_fiq', 'r11_fiq', False),
    ('r12_fiq', 'r12_fiq', False),
    ('r13_fiq', 'r13_fiq', False),
    ('r14_fiq', 'r14_fiq', True),
    ('pc', 'pc', True),
    ('cpsr', 'cpsr', False),
    ('r13_mon', 'r13_mon', False),
    ('r14_mon', 'r14_mon', True),
    ('r14_hyp', 'elr_hyp', True),
    ('_reserved', '_reserved', False),
    ('__reserved1', '__reserved1', False),
    ('__reserved2', '__reserved2', False),
    ('__reserved3', '__reserved3', False),
    ('__reserved4', '__reserved4', False),
]

scandump_registers_64 = [
    ('x0', 'x0', False),
    ('x1', 'x1', False),
    ('x2', 'x2', False),
    ('x3', 'x3', False),
    ('x4', 'x4', False),
    ('x5', 'x5', False),
    ('x6', 'x6', False),
    ('x7', 'x7', False),
    ('x8', 'x8', False),
    ('x9', 'x9', False),
    ('x10', 'x10', False),
    ('x11', 'x11', False),
    ('x12', 'x12', False),
    ('x13', 'x13', False),
    ('x14', 'x14', False),
    ('x15', 'x15', False),
    ('x16', 'x16', False),
    ('x17', 'x17', False),
    ('x18', 'x18', False),
    ('x19', 'x19', False),
    ('x20', 'x20', False),
    ('x21', 'x21', False),
    ('x22', 'x22', False),
    ('x23', 'x23', False),
    ('x24', 'x24', False),
    ('x25', 'x25', False),
    ('x26', 'x26', False),
    ('x27', 'x27', False),
    ('x28', 'x28', False),
    ('x29', 'x29', False),
    ('x30', 'x30', True),
    ('pc', 'pc', True),
    ('currentEL', None, False),
    ('sp_el3', 'sp_el3', False),
    ('elr_el3', 'elr_el3', True),
    ('spsr_el3', 'spsr_el3', False),
    ('sp_el2', 'sp_el2', False),
    ('elr_el2', 'elr_el2', True),
    ('spsr_el2', 'spsr_el2', False),
    ('sp_el1', 'sp_el1', False),
    ('elr_el1', 'elr_el1', True),
    ('spsr_el1', 'spsr_el1', False),
    ('sp_el0', 'sp_el0', False),
    ('cpu_state_0', 'cpu_state_0', False),
    ('cpu_state_1', 'cpu_state_1', False),
    ('cpu_state_3', 'cpu_state_3', False),
    ('cpu_state_4', 'cpu_state_4', False),
    ('cpu_state_5', 'cpu_state_5', False),
    ('__reserved1', '__reserved1', False),
    ('__reserved2', '__reserved2', False),
    ('__reserved3', '__reserved3', False),
    ('__reserved4', '__reserved4', False),
]


class Scandump_v2():

    def __init__(self, core, ramdump, version):
        self.core = core
        self.regs = {}
        self.version = version
        self.ramdump = ramdump
        self.init_missing_regs()
        self.max_32 = "0xFFFFFFFF"

    def init_missing_regs(self):
        self.regs['currentEL'] = 0
        self.regs['spsr_el1'] = 0
        self.regs['spsr_el2'] = 0
        self.regs['spsr_el3'] = 0
        self.regs['cpu_state_0'] = 0
        self.regs['cpu_state_1'] = 0
        self.regs['cpu_state_3'] = 0
        self.regs['cpu_state_4'] = 0
        self.regs['cpu_state_5'] = 0

    def get_reg(self, reg_name):
        try:
            return self.regs[reg_name]
        except KeyError as err:
            print_out_str('{0} not found in scandump'.format(reg_name))
            return None

    def alt_pc_selection(self, pc_val):
        max_32_int = int(self.max_32, 16)
        if pc_val != "":
          pc_val_int = int(pc_val, 16)
          if pc_val_int <= max_32_int:
            return True
        elif pc_val == "":
          return True
        return False

    def prepare_dict(self):
        input_file = "scandump"
        input_file_name = "{0}_core_{1}.cmm".format(input_file, (self.core))
        output = os.path.join(self.ramdump.outdir, input_file_name)
        alt_pc_1 = None
        alt_pc_2 = None
        pc_val = ""
        match_flag = False
        if os.path.exists(output):
            fd = open(output, "r")
            for line in fd:
                match_flag = False
                matchObj = re.match('^REGISTER.SET ([xse].*[0-9]+)\s+(0x[0-9a-f]{0,})', line, re.M | re.I)
                if matchObj:
                    regVal = matchObj.group(2)
                    if regVal == "0x":
                        regVal = "0x0000000000000000"
                    self.regs[(matchObj.group(1)).lower()] = int(regVal, 16)
                    match_flag = True
                else:
                    matchObj = re.match('^REGISTER.SET (PC)\s+(0x[0-9a-f]{0,})', line, re.M | re.I)
                    if matchObj:
                        regVal = matchObj.group(2)
                        if regVal == "0x":
                            regVal = "0x0000000000000000"
                        pc_val = regVal

                    matchObj_altpc = re.match('^REGISTER.SET (ALT_PC_1)\s+(0x[0-9a-f]{0,})', line, re.M | re.I)
                    if matchObj_altpc:
                        alt_pc_1 = matchObj_altpc.group(2)
                        if alt_pc_1 == "0x":
                            alt_pc_1 = "0x0000000000000000"
                    matchObj_altpc = re.match('^REGISTER.SET (ALT_PC_2)\s+(0x[0-9a-f]{0,})', line, re.M | re.I)
                    if matchObj_altpc:
                        alt_pc_2 = matchObj_altpc.group(2)
                        if alt_pc_2 == "0x":
                            alt_pc_2 = "0x0000000000000000"
                matchObj = re.match('^REGISTER.SET ([xse].*[0-9]+)\s+([0-9a-f])', line, re.M | re.I)
                if matchObj and match_flag == False:
                    regVal = matchObj.group(2)
                    if regVal == "0":
                        regVal = "0x0000000000000000"
                    self.regs[(matchObj.group(1)).lower()] = int(regVal, 16)


            result_32 = self.alt_pc_selection(pc_val)
            if result_32:
                if alt_pc_1 and alt_pc_2:
                    diff_alt_pc1 = int(alt_pc_1, 16) - int(pc_val, 16)
                    diff_alt_pc2 = int(alt_pc_2, 16) - int(pc_val, 16)

                    if diff_alt_pc1 > diff_alt_pc2:
                        self.regs['pc'] = int(alt_pc_2, 16)
                    else:
                        self.regs['pc'] = int(alt_pc_1, 16)

                elif alt_pc_1 and not alt_pc_2:
                    self.regs['pc'] = int(alt_pc_1, 16)
                elif alt_pc_2 and not alt_pc_1:
                    self.regs['pc'] = int(alt_pc_2, 16)
                else:
                    if pc_val != "":
                      self.regs['pc'] = int(pc_val, 16)
                    else:
                      self.regs['pc'] = 0
            else:
                if pc_val != "":
                  self.regs['pc'] = int(pc_val, 16)
                else:
                  self.regs['pc'] = 0
            return self.regs
        else:
            return None

    def print_regs(self, ramdump):
        if ramdump.arm64:
            register_names = scandump_registers_64
        else:
            register_names = scandump_registers_32
        for reg_name, t32_name, print_pc in register_names:
            if re.match('(.*)reserved(.*)', reg_name):
                continue
            if not reg_name in self.regs:
                print_out_str (reg_name + " not captured in scandump")
                continue
            if print_pc:
                a = ramdump.unwind_lookup(self.get_reg(reg_name))
                if a is not None:
                    symname, offset = ramdump.unwind_lookup(
                        self.get_reg(reg_name))
                    pc_string = '[{0}+0x{1:x}]'.format(symname, offset)
                else:
                    pc_string = None
            else:
                pc_string = None
            if pc_string is not None:
                try:
                    print_out_str('   {0:8} = 0x{1:016x} {2}'.format(
                                  reg_name, self.get_reg(reg_name), pc_string))
                except TypeError as err:
                    print_out_str(str(err))
            else:
                try:
                    print_out_str('   {0:8} = 0x{1:016x}'.format(
                              reg_name, self.get_reg(reg_name)))
                except TypeError as err:
                    print_out_str(str(err))
    def dump_all_regs(self, ram_dump):
        print_out_str('core{0} regs:'.format(self.core))
        self.print_regs(ram_dump)

    def dump_core_pc(self, ram_dump):
        pc = self.get_reg('pc')
        if ram_dump.arm64:
            lr = self.get_reg('x30')
            bt = self.get_reg('sp_el1')
            fp = self.get_reg('x29')
        else:
            lr = self.get_reg('r14_svc')
            bt = self.get_reg('r13_svc')
            fp = self.get_reg('r11')

        if pc is not None:
            a = ram_dump.unwind_lookup(pc)
            if a is not None:
                symname, offset = a
            else:
                symname = 'UNKNOWN'
                offset = 0
            try:
                print_out_str(
                    'Core {3} PC: {0}+{1:x} <{2:x}>'.format(symname, offset,
                                                            pc, self.core))
            except TypeError as err:
                print_out_str(str(err))

        if lr is not None:
            a = ram_dump.unwind_lookup(lr)
            if a is not None:
                symname, offset = a
            else:
                symname = 'UNKNOWN'
                offset = 0

            try:
                print_out_str(
                    'Core {3} LR: {0}+{1:x} <{2:x}>'.format(symname, offset,
                                                            lr, self.core))
            except TypeError as err:
                print_out_str(str(err))
        print_out_str('')
        if pc is not None and lr is not None:
            ram_dump.unwind.unwind_backtrace(bt, fp, pc, lr, '')
        print_out_str('')
