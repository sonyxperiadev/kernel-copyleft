# Copyright (c) 2015, The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import linux_list
from parser_util import register_parser, RamParser

@register_parser('--spm', 'Parse SPM Driver info')
class spm(RamParser):
    def __init__(self, *args):
        super(spm, self).__init__(*args)
        self.head = ''
        self.output = []
        self.spm_shadow_reg = ('MSM_SPM_REG_SAW2_CFG',
            'MSM_SPM_REG_SAW2_AVS_CTL',
            'MSM_SPM_REG_SAW2_AVS_HYSTERESIS',
            'MSM_SPM_REG_SAW2_SPM_CTL',
            'MSM_SPM_REG_SAW2_PMIC_DLY',
            'MSM_SPM_REG_SAW2_AVS_LIMIT',
            'MSM_SPM_REG_SAW2_AVS_DLY',
            'MSM_SPM_REG_SAW2_SPM_DLY',
            'MSM_SPM_REG_SAW2_PMIC_DATA_0',
            'MSM_SPM_REG_SAW2_PMIC_DATA_1',
            'MSM_SPM_REG_SAW2_PMIC_DATA_2',
            'MSM_SPM_REG_SAW2_PMIC_DATA_3',
            'MSM_SPM_REG_SAW2_PMIC_DATA_4',
            'MSM_SPM_REG_SAW2_PMIC_DATA_5',
            'MSM_SPM_REG_SAW2_PMIC_DATA_6',
            'MSM_SPM_REG_SAW2_PMIC_DATA_7',
            'MSM_SPM_REG_SAW2_RST',
            'MSM_SPM_REG_SAW2_ID',
            'MSM_SPM_REG_SAW2_SECURE',
            'MSM_SPM_REG_SAW2_STS0',
            'MSM_SPM_REG_SAW2_STS1',
            'MSM_SPM_REG_SAW2_STS2',
            'MSM_SPM_REG_SAW2_VCTL',
            'MSM_SPM_REG_SAW2_SEQ_ENTRY',
            'MSM_SPM_REG_SAW2_SPM_STS',
            'MSM_SPM_REG_SAW2_AVS_STS',
            'MSM_SPM_REG_SAW2_PMIC_STS',
            'MSM_SPM_REG_SAW2_VERSION')

    def spm_walker(self, spm):
        if spm == self.head:
                return

        offset = self.ramdump.field_offset('struct msm_spm_device', 'initialized')
        if self.ramdump.read_bool(spm + offset) is False:
                return

        offset = self.ramdump.field_offset('struct msm_spm_device', 'name')
        name = self.ramdump.read_cstring(self.ramdump.read_word(spm + offset, True), 48)
        self.output.append("{:35}:{}\n".format("SPM Device Name", name))

        offset = self.ramdump.field_offset('struct msm_spm_device', 'reg_data')
        reg_data = spm + offset

        offset = self.ramdump.field_offset('struct msm_spm_driver_data', 'major')
        addr = reg_data + offset
        major = self.ramdump.read_int(addr)
        self.output.append("{:35}:{}".format("version", major))

        offset = self.ramdump.field_offset('struct msm_spm_driver_data', 'minor')
        addr = reg_data + offset
        minor = self.ramdump.read_int(addr)
        self.output.append(".{}\n".format(minor))

        self.output.append("\n{}\n".format("Shadow Registers"))
        self.output.append("{}{}".format("-" * 20, "\n"))

        offset = self.ramdump.field_offset('struct msm_spm_driver_data', 'reg_shadow')

        for i in range(len(self.spm_shadow_reg)):
                addr = reg_data + offset + i * self.ramdump.sizeof('uint32_t')
                val = self.ramdump.read_int(addr)
                self.output.append("{:35}:{}\n".format(self.spm_shadow_reg[i], hex(val).rstrip("L")))

        self.output.append("{}{}".format("-" * 81, "\n\n"))

    def get_spm(self):
        lpm_root_node = self.ramdump.read_word(
            self.ramdump.address_of('lpm_root_node'), True)
        if lpm_root_node is None:
                self.output_file.write("NOTE: 'lpm_root_node' not found\n")
                return

        offset = self.ramdump.field_offset('struct lpm_cluster', 'lpm_dev')
        lpm_dev = self.ramdump.read_word(lpm_root_node + offset, True)

        offset = self.ramdump.field_offset('struct low_power_ops', 'spm')
        spm = self.ramdump.read_word(lpm_dev + offset, True)

        self.head = lpm_dev + offset

        offset = self.ramdump.field_offset('struct msm_spm_device', 'list')
        spm_walker = linux_list.ListWalker(self.ramdump, spm, offset)
        spm_walker.walk(spm, self.spm_walker)

    def parse(self):
        self.output_file = self.ramdump.open_file('spm.txt')
        self.get_spm()
        for i in self.output:
                self.output_file.write(i)
        self.output_file.close()
