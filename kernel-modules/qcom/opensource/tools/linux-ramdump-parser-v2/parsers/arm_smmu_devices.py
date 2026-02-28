# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.

import os
from parser_util import register_parser, RamParser
from dmesglib import DmesgLib
from print_out import print_out_str
import linux_list as llist
import parsers.linux_devices as ldevices
from struct_print import struct_print_class

def ARM_SMMU_GR0_S2CR(n):
    return  (0xc00 + ((n) << 2))

@register_parser('--arm_smmu_device', 'arm_smmu_device')
class arm_smmu_device(RamParser):
    def get_arm_smmu_device_info(self):
        ramdump = self.ramdump
        f_path = os.path.join(self.ramdump.outdir, "arm_smmu_devices.txt")
        fout = open(f_path, "w")

        arm_smmu_devices_sid = os.path.join(self.ramdump.outdir, "arm_smmu_devices_sid.txt")
        arm_smmu_devices_sid_fout = open(arm_smmu_devices_sid, "w")
        device = ldevices.DevicesList(self.ramdump)
        device_lists = device.get_device_list(None)

        for item in device_lists:
            name = item[1]
            device = item[0]
            if name == None:
                continue
            if 'apps-smmu' in name:
                drvdata = self.ramdump.read_structure_field(device, 'struct device', 'driver_data')
                if drvdata != 0:
                    try:
                        print ("v.v (struct device)0x%x   %-64s %-32s 0x%-32x  0x%x" % (item[0], item[1], item[2], item[3], item[4]), file = fout)
                        print ("    v.v (struct arm_smmu_device *)0x%x  " % (drvdata), file = fout)
                        arm_smmu_device = drvdata
                        num_mapping_groups = self.ramdump.read_structure_field(arm_smmu_device, 'struct arm_smmu_device', 'num_mapping_groups')
                        print("v.v (struct arm_smmu_device *)0x%x  num_mapping_groups %d"%(arm_smmu_device, num_mapping_groups), file = fout)
                        arm_smmu_device_datatype = ramdump.read_datatype(arm_smmu_device, 'struct arm_smmu_device')
                        s2crs = arm_smmu_device_datatype.s2crs
                        #s2crs = ramdump.read_structure_field(arm_smmu_device, 'struct arm_smmu_device', 's2crs')
                        ptr_size = ramdump.sizeof('struct arm_smmu_s2cr')
                        smrs =  arm_smmu_device_datatype.smrs
                        ptr_size_smrs = ramdump.sizeof('struct arm_smmu_smr')
                    except Exception as e: print_out_str(str(e))
                    for i in range(0, num_mapping_groups):
                        try:
                            s2crs += i * ptr_size
                            s2crs_data = struct_print_class(ramdump, 'arm_smmu_s2cr', s2crs, fout)
                            s2crs_data.append('group', 'ptr')
                            s2crs_data.append('count', 'u32')
                            s2crs_data.append('type', 'u32')
                            s2crs_data.append('privcfg', 'u32')
                            s2crs_data.append('cbndx', 'u8')
                            s2crs_data.append('pinned', 'u8')
                            s2crs_data.process()
                            cbndx =  s2crs_data.get_val('cbndx')
                            reg_addr = ARM_SMMU_GR0_S2CR(cbndx)
                            print("=================================================================0x%x index %x"%(reg_addr, i), file = fout)
                            print("----------------------------- %x"%(s2crs), file = fout)
                            s2crs_data.print_struct()
                            smrs += i * ptr_size_smrs
                            print("----------------------------- %x"%(smrs), file = fout)
                            smrs_data = struct_print_class(ramdump, 'arm_smmu_smr', smrs, fout)
                            smrs_data.append('mask', 'u16')
                            smrs_data.append('id', 'u16')
                            smrs_data.append('valid', 'u8')
                            smrs_data.append('pinned', 'u8')
                            smrs_data.process()
                            smrs_data.print_struct()
                        except Exception as e: print_out_str(e)
            iommu = self.ramdump.read_structure_field(device, 'struct device', 'iommu')
            ids = 0
            ARM_SMMU_SMR_ID = 0xaa
            ARM_SMMU_SMR_MASK = 0xaa
            try:
                if iommu != None and iommu != 0:
                    fwspec = self.ramdump.read_structure_field(iommu, 'struct dev_iommu', ' fwspec')
                    if fwspec != None and fwspec != 0:
                        ids_offset = self.ramdump.field_offset('struct iommu_fwspec', 'ids')
                        ids = self.ramdump.read_u32(ids_offset + fwspec)
                        ARM_SMMU_SMR_ID = ids & 0xffff
                        ARM_SMMU_SMR_MASK = (ids >> 16) & 0xffff
                        print("name = %-64s SID = 0x%-8x" %(name, ARM_SMMU_SMR_ID), file = arm_smmu_devices_sid_fout)
            except Exception as e: print_out_str(str(e))
        arm_smmu_devices_sid_fout.close()
        fout.close()

    def parse(self):
        self.get_arm_smmu_device_info()
