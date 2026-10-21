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

import print_out
from parser_util import RamParser, cleanupString, register_parser
import module_table
@register_parser('--modules_table', 'Dump modules_table')
class Modules_table(RamParser):
    def retrieve_modules_cn(self):
        mod_list = self.ramdump.address_of('modules')
        next_offset = self.ramdump.field_offset('struct list_head', 'next')
        list_offset = self.ramdump.field_offset('struct module', 'list')
        name_offset = self.ramdump.field_offset('struct module', 'name')
        scmversion_offset = self.ramdump.field_offset('struct module', 'scmversion')
        if self.ramdump.kernel_version > (4, 9, 0):
            module_core_offset = self.ramdump.field_offset('struct module', 'core_layout.base')
        else:
            module_core_offset = self.ramdump.field_offset('struct module', 'module_core')

        kallsyms_offset = self.ramdump.field_offset('struct module', 'kallsyms')

        next_list_ent = self.ramdump.read_pointer(mod_list + next_offset)
        while next_list_ent and next_list_ent != mod_list:
            mod_tbl_ent = module_table.module_table_entry()
            module = next_list_ent - list_offset
            name_ptr = module + name_offset
            mod_tbl_ent.name = self.ramdump.read_cstring(name_ptr)
            svmversion_addr = self.ramdump.read_pointer(scmversion_offset+module)
            svmversion = self.ramdump.read_cstring(svmversion_addr)
            mod_tbl_ent.module_offset = self.ramdump.read_pointer(module + module_core_offset)
            if mod_tbl_ent.module_offset is None:
                mod_tbl_ent.module_offset = 0
            mod_tbl_ent.kallsyms_addr = self.ramdump.read_pointer(module + kallsyms_offset)
            self.module_table_cn.add_entry(mod_tbl_ent)

            self.modules_list.append((mod_tbl_ent.module_offset, mod_tbl_ent.name, module, svmversion))
            next_list_ent = self.ramdump.read_pointer(next_list_ent + next_offset)
        self.modules_list.sort()

        for item in self.modules_list:
            print("%-32s 0x%-32x v.v (struct module)0x%-32x %s" % (item[1], item[0], item[2], item[3]), file=self.f)
    def parse(self):
        self.module_table_cn = module_table.module_table_class()
        self.f = open(self.ramdump.outdir + "/modules_table.txt", "w")
        self.modules_list =[]
        self.retrieve_modules_cn()
        self.f.close();
