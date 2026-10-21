# Copyright (c) 2021 The Linux Foundation. All rights reserved.
# Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import sys
import re
import os
import struct
import string
import glob

from parser_util import register_parser, RamParser
@register_parser('--print-svm-vcpu-ctx', 'Print svm vcpu context')

class svm_vcpu_context_parse(RamParser):

    def svm_vcpu_context_parse(self,fop):
        opfile = fop
        for core in range(0,2):
            input_file = "corevcpu{0}_vm_*.cmm".format(core)
            file_path_list = glob.glob(os.path.join(self.ramdump.outdir + "\..", input_file))
            for each_file in file_path_list:
                input_file_cmm = each_file
                if os.path.exists(input_file_cmm):
                    fd = open(input_file_cmm, "r")
                else:
                    print("not exisit")
                    continue
                fp = 0
                sp = 0
                lr = 0
                pc = 0
                pc_flag = False
                lr_flag = False
                sp_flag = False
                fp_flag = False
                for line in fd:
                    columns = line.split()
                    if "r.s pc" in line and pc_flag is False:
                        pc_flag = True
                        pc = int(columns[-1], 16)
                    if "r.s x30" in line  and lr_flag is False:
                        lr_flag = True
                        lr = int(columns[-1], 16)
                    if "r.s x29" in line and fp_flag is False:
                        fp_flag = True
                        fp = int(columns[-1], 16)
                    if "r.s sp_el1" in line and sp_flag is False:
                        sp_flag = True
                        sp = int(columns[-1], 16)

                opfile.write("Core {0} context\n".format(core))
                a = self.ramdump.unwind_lookup(pc)
                if a is not None:
                    symname, offset = a
                else:
                    symname = 'UNKNOWN'
                    offset = 0
                opfile.write(
                    'Core {3} PC: {0}+{1:x} <0x{2:x}>'.format(symname, offset,
                                                            pc, core))
                opfile.write("\n")
                a = self.ramdump.unwind_lookup(lr)
                if a is not None:
                    symname, offset = a
                else:
                    symname = 'UNKNOWN'
                    offset = 0
                opfile.write(
                    'Core {3} LR: {0}+{1:x} <0x{2:x}>'.format(symname, offset,
                                                            lr, core))
                opfile.write("\n")

                opfile.write('')
                self.ramdump.unwind.unwind_backtrace(sp, fp, pc, lr, '',opfile)
                opfile.write('')
                opfile.write("\n")
        opfile.close()

    def parse(self):
        with self.ramdump.open_file('vm_vcpu_context.txt') as fop:
            self.svm_vcpu_context_parse(fop)
        return
