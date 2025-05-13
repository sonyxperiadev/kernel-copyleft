# Copyright (c) 2012-2021, The Linux Foundation. All rights reserved.
# Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
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
import linux_list as llist
import re
import shutil
import os
import platform
import random
import subprocess
import sys
import time
import local_settings
from scandump_reader import Scandump_v2
from collections import OrderedDict

from dcc import DccRegDump, DccSramDump
from pmic import PmicRegDump
from print_out import print_out_str, print_out_exception
from qdss import QDSSDump
from watchdog_v2 import TZRegDump_v2
from cachedumplib import lookup_cache_type
from tlbdumplib import lookup_tlb_type
from vsens import VsensData
from sysregs import SysRegDump
from fcmdump import FCM_Dump

MEMDUMPV2_MAGIC = 0x42445953
MEMDUMPV_HYP_MAGIC = 0x42444832
MAX_NUM_ENTRIES = 0x164
IMEM_OFFSET_MEM_DUMP_TABLE = 0x3f010

msm_dump_cpu_type = ['MSM_DUMP_CPU_TYPE_INVALID',
        'MSM_DUMP_CPU_TYPE_AARCH32',
        'MSM_DUMP_CPU_TYPE_AARCH64',
         'MSM_DUMP_CPU_TYPE_HYDRA']

msm_dump_ctx_type= [
        'MSM_DUMP_CTX_TYPE_PHYS_NS_CPU_CTX',
        'MSM_DUMP_CTX_TYPE_PHYS_SEC_CPU_CTX',
        'MSM_DUMP_CTX_TYPE_NS_VCPU_EL10_CTX',
        'MSM_DUMP_CTX_TYPE_SEC_VCPU_EL10_CTX',
        'MSM_DUMP_CTX_TYPE_NS_VCPU_NESTED_EL2_CTX']

msm_dump_regset_ids = {}
msm_dump_regset_ids[0] = 'MSM_DUMP_REGSET_IDS_INVALID'
msm_dump_regset_ids[16] = 'MSM_DUMP_REGSET_IDS_AARCH64_GPRS'
msm_dump_regset_ids[17] = 'MSM_DUMP_REGSET_IDS_AARCH64_NEON'
msm_dump_regset_ids[18] = 'MSM_DUMP_REGSET_IDS_AARCH64_SVE'
msm_dump_regset_ids[19] = 'MSM_DUMP_REGSET_IDS_AARCH64_SYSREGS_EL0'
msm_dump_regset_ids[20] = 'MSM_DUMP_REGSET_IDS_AARCH64_EL1'
msm_dump_regset_ids[21] = 'MSM_DUMP_REGSET_IDS_AARCH64_EL2'
msm_dump_regset_ids[22] = 'MSM_DUMP_REGSET_IDS_AARCH64_VM_EL2'
msm_dump_regset_ids[23] = 'MSM_DUMP_REGSET_IDS_AARCH64_EL3'
msm_dump_regset_ids[24] = 'MSM_DUMP_REGSET_IDS_AARCH64_DBG_EL1'
msm_dump_regset_ids[25] = 'MSM_DUMP_REGSET_IDS_AARCH64_CNTV_EL10'
msm_dump_regset_ids[26]	= 'MSM_DUMP_REGSET_IDS_AARCH64_CNTP_EL10'
msm_dump_regset_ids[27] = 'MSM_DUMP_REGSET_IDS_AARCH64_CNT_EL2'



class client(object):
    MSM_DUMP_DATA_CPU_CTX = 0x00
    MSM_DUMP_DATA_L1_INST_TLB = 0x20
    MSM_DUMP_DATA_L1_DATA_TLB = 0x40
    MSM_DUMP_DATA_L1_INST_CACHE = 0x60
    MSM_DUMP_DATA_L1_DATA_CACHE = 0x80
    MSM_DUMP_DATA_ETM_REG = 0xA0
    MSM_DUMP_DATA_L2_CACHE = 0xC0
    MSM_DUMP_DATA_L3_CACHE = 0xD0
    MSM_DUMP_DATA_OCMEM = 0xE0
    MSM_DUMP_DATA_DBGUI_REG = 0xE5
    MSM_DUMP_DATA_MISC = 0xE8
    MSM_DUMP_DATA_VSENSE = 0xE9
    MSM_DUMP_DATA_TMC_ETF = 0xF0
    MSM_DUMP_DATA_TMC_ETF_SWAO = 0xF1
    MSM_DUMP_DATA_TMC_REG = 0x100
    MSM_DUMP_DATA_TMC_ETR_REG = 0x100
    MSM_DUMP_DATA_TMC_ETF_REG = 0x101
    MSM_DUMP_DATA_TMC_ETR1_REG = 0x105
    MSM_DUMP_DATA_TMC_ETF_SWAO_REG = 0x102
    MSM_DUMP_DATA_LOG_BUF = 0x110
    MSM_DUMP_DATA_LOG_BUF_FIRST_IDX = 0x111
    MSM_DUMP_DATA_L2_TLB = 0x120
    MSM_DUMP_DATA_DCC_REG = 0xE6
    MSM_DUMP_DATA_DCC_SRAM = 0xE7
    MSM_DUMP_DATA_SCANDUMP = 0xEB
    MSM_DUMP_DATA_RPMH = 0xEC
    MSM_DUMP_DATA_FCMDUMP = 0xEE
    MSM_DUMP_DATA_CPUSS = 0xEF
    MSM_DUMP_DATA_SCANDUMP_PER_CPU = 0x130
    MSM_DUMP_DATA_LLC_CACHE = 0x140
    MSM_DUMP_DATA_MHM = 0x161
    MSM_DUMP_DATA_MAX = MAX_NUM_ENTRIES

# Client functions will be executed in top-to-bottom order
client_types = [
    ('MSM_DUMP_DATA_SCANDUMP', 'parse_scandump'),
    ('MSM_DUMP_DATA_SCANDUMP_PER_CPU', 'parse_scandump'),
    ('MSM_DUMP_DATA_FCMDUMP', 'parse_fcmdump'),
    ('MSM_DUMP_DATA_CPUSS', 'parse_cpuss'),
    ('MSM_DUMP_DATA_CPU_CTX', 'parse_cpu_ctx'),
    ('MSM_DUMP_DATA_L1_INST_TLB', 'parse_tlb_common'),
    ('MSM_DUMP_DATA_L1_DATA_TLB', 'parse_tlb_common'),
    ('MSM_DUMP_DATA_L1_INST_CACHE', 'parse_cache_common'),
    ('MSM_DUMP_DATA_L1_DATA_CACHE', 'parse_cache_common'),
    ('MSM_DUMP_DATA_L2_CACHE', 'parse_cache_common'),
    ('MSM_DUMP_DATA_L3_CACHE', 'parse_l3_cache'),
    ('MSM_DUMP_DATA_OCMEM', 'parse_ocmem'),
    ('MSM_DUMP_DATA_DBGUI_REG', 'parse_qdss_common'),
    ('MSM_DUMP_DATA_VSENSE', 'parse_vsens'),
    ('MSM_DUMP_DATA_PMIC', 'parse_pmic'),
    ('MSM_DUMP_DATA_DCC_REG', 'parse_dcc_reg'),
    ('MSM_DUMP_DATA_DCC_SRAM', 'parse_dcc_sram'),
    ('MSM_DUMP_DATA_TMC_ETF', 'parse_qdss_common'),
    ('MSM_DUMP_DATA_TMC_ETF_SWAO', 'parse_qdss_common'),
    ('MSM_DUMP_DATA_TMC_ETR_REG', 'parse_qdss_common'),
    ('MSM_DUMP_DATA_TMC_ETR1_REG', 'parse_qdss_common'),
    ('MSM_DUMP_DATA_TMC_ETF_SWAO_REG', 'parse_qdss_common'),
    ('MSM_DUMP_DATA_TMC_REG', 'parse_qdss_common'),
    ('MSM_DUMP_DATA_L2_TLB', 'parse_tlb_common'),
    ('MSM_DUMP_DATA_LLC_CACHE', 'parse_system_cache_common'),
    ('MSM_DUMP_DATA_MISC', 'parse_sysdbg_regs'),
    ('MSM_DUMP_DATA_MHM', 'parse_mhm_dump')
]

qdss_tag_to_field_name = {
    'MSM_DUMP_DATA_TMC_ETR_REG': 'tmc_etr_start',
    'MSM_DUMP_DATA_TMC_ETR1_REG': 'tmc_etr1_start',
    'MSM_DUMP_DATA_TMC_REG': 'tmc_etr_start',
    'MSM_DUMP_DATA_TMC_ETF': 'etf_start',
    'MSM_DUMP_DATA_TMC_ETF_SWAO': 'tmc_etf_swao_start',
    'MSM_DUMP_DATA_TMC_ETF_SWAO_REG': 'tmc_etf_swao_reg_start',
    'MSM_DUMP_DATA_DBGUI_REG': 'dbgui_start',
}

# Client functions will be executed in top-to-bottom order
minidump_dump_table_type = [
    ('MSM_DUMP_DATA_SCANDUMP', 'KSCANDUMP'),
    ('MSM_DUMP_DATA_CPU_CTX', 'KCPU_CTX'),
    ('MSM_DUMP_DATA_L1_INST_TLB', 'KCPUSS'),
    ('MSM_DUMP_DATA_L1_DATA_TLB','KCPUSS'),
    ('MSM_DUMP_DATA_L1_INST_CACHE', 'KCPUSS'),
    ('MSM_DUMP_DATA_L1_DATA_CACHE', 'KCPUSS'),
    ('MSM_DUMP_DATA_L2_CACHE', 'KCPUSS'),
    ('MSM_DUMP_DATA_L3_CACHE', 'KCPUSS'),
    ('MSM_DUMP_DATA_VSENSE', 'KVSENSE'),
    ('MSM_DUMP_DATA_PMIC', 'KPMIC'),
    ('MSM_DUMP_DATA_DCC_REG', 'KDCC_REG'),
    ('MSM_DUMP_DATA_DCC_SRAM', 'KDCC_SRAM'),
    ('MSM_DUMP_DATA_TMC_ETF', 'KTMC_ETF'),
    ('MSM_DUMP_DATA_TMC_ETR_REG', 'KTMC_REG'),
    ('MSM_DUMP_DATA_TMC_REG', 'KTMC_REG'),
    ('MSM_DUMP_DATA_MISC', 'KMISC')

]

class DebugImage_v2():

    def __init__(self, ramdump):
        self.qdss = QDSSDump()
        self.dump_type_lookup_table = []
        self.dump_data_id_lookup_table  = {}
        if ramdump.kernel_version > (3, 9, 9):
            self.event_call = 'struct trace_event_call'
            self.event_class = 'struct trace_event_class'
        else:
            self.event_call = 'struct ftrace_event_call'
            self.event_class = 'struct ftrace_event_class'

    def parse_scandump(self, version, start, end, client_id, ram_dump):
        scandump_file_prefix = "scandump_core"
        core_bin_prefix = "core"
        chipset = ram_dump.hw_id
        try:
            scan_wrapper_path = local_settings.scandump_parser_path
        except AttributeError:
            print_out_str('Could not find scandump_parser_path . Please define scandump_parser_path in local_settings')
            return
        if ram_dump.arm64:
            arch = "aarch64"
        else:
            arch = "aarch32"
        if client_id == client.MSM_DUMP_DATA_SCANDUMP:
            output = os.path.join(ram_dump.outdir, scandump_file_prefix)
            input = os.path.join(ram_dump.outdir, "core.bin")
            core_num = client_id & 0xF
        elif client_id >= client.MSM_DUMP_DATA_SCANDUMP_PER_CPU:
            core_num = client_id & 0xF
            output = '{0}_{1:x}'.format(scandump_file_prefix, core_num)
            output = os.path.join(ram_dump.outdir, output)

            input_filename = '{0}_{1:x}.bin'.format(core_bin_prefix, core_num)
            input = os.path.join(ram_dump.outdir, input_filename)
        print_out_str(
            'Parsing scandump context start {0:x} end {1:x} {2} {3}'.format(start, end, output, input))
        header_bin = ram_dump.open_file(input, 'wb')
        header_bin.write(ram_dump.read_physical(start, end - start))
        header_bin.close()
        subprocess.call('py -2 {0} -d {1} -o {2} -f {3} -c {4}'.format(scan_wrapper_path, input, output, arch, chipset))
        sv2 = Scandump_v2(core_num,ram_dump,version)
        reg_info = sv2.prepare_dict()
        if reg_info is not None:
            sv2.dump_core_pc(ram_dump)
            sv2.dump_all_regs(ram_dump)
        return
    def parse_cpuss(self, version, start, end, client_id, ram_dump):
        client_name = self.dump_data_id_lookup_table[client_id]
        cpuss_file_prefix = "cpuss_reg_dump.xml"
        print_out_str(
                'Parsing {0} context start {1:x} end {2:x}'.format(client_name, start, end))
        try:
            cpuss_parser_path = local_settings.cpuss_parser_path
            cpuss_parser_json = local_settings.cpuss_parser_json
        except AttributeError:
            print_out_str('Could not find cpuss_parser_path . Please define cpuss_parser_path in local_settings')
            return
        offset = None
        offset, input = ram_dump.get_read_physical_offset(start)

        if offset is None:
            print_out_str("parse_cpuss start address {0} not found".format(start))
        print_out_str("parse_cpuss offset address = {0} input = {1} cpuss_parser_json = {2}".format(hex(int(offset)),input,cpuss_parser_json))
        subprocess.call('py -2 {0} -i {1} -O {2} -o {3} -j {4}'.format(cpuss_parser_path, input, hex(int(offset)), ram_dump.outdir, cpuss_parser_json))

    def parse_fcmdump(self, version, start, end, client_id, ram_dump):
        client_name = self.dump_data_id_lookup_table[client_id]

        print_out_str(
                'Parsing {0} context start {1:x} end {2:x}'.format(client_name, start, end))

        fcmdumps = FCM_Dump(start, end)
        if fcmdumps.dump_fcm_img(ram_dump) is False:
            print_out_str('!!! Could not dump FCM')

        return

    def parse_cpu_ctx(self, version, start, end, client_id, ram_dump,dump_data_name=None):
        core = client_id - client.MSM_DUMP_DATA_CPU_CTX

        if version == 32 or version == "32":
            try:
                cpu_type_offset  = ram_dump.field_offset(
                                'struct msm_dump_cpu_ctx', 'cpu_type')
                if cpu_type_offset is None:
                    cpu_type_offset = 0x0
                ctx_type_offset  = ram_dump.field_offset(
                                'struct msm_dump_cpu_ctx', 'ctx_type')
                if ctx_type_offset is None:
                    ctx_type_offset = 0x4
                cpu_id_offset  = ram_dump.field_offset(
                                'struct msm_dump_cpu_ctx', 'cpu_id')
                if cpu_id_offset is None:
                    cpu_id_offset = 0xC
                cpu_index_offset  = ram_dump.field_offset(
                                'struct msm_dump_cpu_ctx', 'affinity')
                if cpu_index_offset is None:
                    cpu_index_offset = 0x10
                machine_id_offset  = ram_dump.field_offset(
                                'struct msm_dump_cpu_ctx', 'machine_id')
                if machine_id_offset is None:
                    machine_id_offset = 0x14
                registers_offset   = ram_dump.field_offset(
                                'struct msm_dump_cpu_ctx', 'registers')
                if registers_offset is None:
                    registers_offset = 0x20
                regset_num_register_offset  = ram_dump.field_offset(
                                'struct msm_dump_cpu_ctx', 'num_register_sets')
                if regset_num_register_offset is None:
                    regset_num_register_offset = 0x1C
                regset_id_offset  = ram_dump.field_offset(
                                'struct msm_dump_cpu_register_entry', 'regset_id')
                if regset_id_offset is None:
                    regset_id_offset = 0x0
                regset_addr_offset  = ram_dump.field_offset(
                                'struct msm_dump_cpu_register_entry', 'regset_addr')
                if regset_addr_offset is None:
                    regset_addr_offset = 0x8
                cpu_index = ram_dump.read_u32(start + cpu_index_offset,False)
                print_out_str(
                    'Parsing CPU{2:x} context start {0:x} end {1:x} version {3} client_id-> {4:x}'.format(start, end, cpu_index,version,client_id))
                cpu_type = ram_dump.read_u32(start + cpu_type_offset,False)
                print_out_str("cpu_type = {0}".format(msm_dump_cpu_type[cpu_type]))
                ctx_type = ram_dump.read_u32(start + ctx_type_offset,False)
                print_out_str("ctx_type = {0}".format(msm_dump_ctx_type[ctx_type]))
                print_out_str("cpu_index = {0}".format(cpu_index))
                regset_num_register = ram_dump.read_u32(start + regset_num_register_offset,False)
                registers = start + registers_offset
                registers_size = ram_dump.sizeof('struct msm_dump_cpu_register_entry')
                if registers_size is None:
                    registers_size = 0x10
                regset_name_addr = OrderedDict()
                for i in range(0,regset_num_register):
                    registers_addr = registers + registers_size * i
                    regset_id = ram_dump.read_u32(registers_addr + regset_id_offset,False)
                    if regset_id == 0:
                        break
                    regset_name = msm_dump_regset_ids[regset_id]
                    print_out_str("regset_name = {0}".format(regset_name))
                    regset_addr = ram_dump.read_u64(registers_addr + regset_addr_offset,False)
                    regset_size = ram_dump.sizeof('struct msm_dump_aarch64_gprs')
                    if regset_size is None:
                        regset_size = 0x110
                    regset_end = regset_addr + regset_size
                    regset_name_addr[regset_name] = [regset_addr,regset_end]
                regs = TZRegDump_v2()
                cpu_index_num = "{0:x}".format(cpu_index >> 8)
                print("dump_data_name = {0}".format(dump_data_name))
                if dump_data_name and "vm_3" not in dump_data_name:
                    core = "vcpu" + str(cpu_index_num) + "_" + dump_data_name.split('_vcpu_')[0]
                else:
                    core = "vcpu"+str(cpu_index_num)
                regs_flag = regs.init_regs_v2(version, regset_name_addr, core, ram_dump)
                if regs_flag == False:
                    print_out_str('!!! Could not get registers from TZ dump')
                    return
                regs.dump_core_pc_gprs(ram_dump)
                regs.dump_all_regs_gprs(ram_dump)
            except Exception as err:
                pass
        else:
            regs = TZRegDump_v2()
            if regs.init_regs(version, start, end, core, ram_dump) is False:
                print_out_str('!!! Could not get registers from TZ dump')
                return
            regs.dump_core_pc(ram_dump)
            regs.dump_all_regs(ram_dump)

    def parse_pmic(self, version, start, end, client_id, ram_dump):
        client_name = self.dump_data_id_lookup_table[client_id]

        print_out_str(
            'Parsing {0} context start {1:x} end {2:x}'.format(client_name, start, end))

        regs = PmicRegDump(start, end)
        if regs.parse_all_regs(ram_dump) is False:
            print_out_str('!!! Could not get registers from PMIC dump')
            return

        regs.dump_all_regs(ram_dump)

    def parse_dcc_reg(self, version, start, end, client_id, ram_dump):
        client_name = self.dump_data_id_lookup_table[client_id]

        print_out_str(
            'Parsing {0} context start {1:x} end {2:x}'.format(client_name, start, end))

        regs = DccRegDump(start, end)
        if regs.parse_all_regs(ram_dump) is False:
            print_out_str('!!! Could not get registers from DCC register dump')
            return

        regs.dump_all_regs(ram_dump)
        return

    def parse_dcc_sram(self, version, start, end, client_id, ram_dump):
        client_name = self.dump_data_id_lookup_table[client_id]

        print_out_str(
            'Parsing {0} context start {1:x} end {2:x}'.format(client_name, start, end))

        regs = DccSramDump(start, end, ram_dump)
        if regs.dump_sram_img(ram_dump) is False:
            print_out_str('!!! Could not dump SRAM')
        else:
            ram_dump.dcc = True
        return

    def parse_sysdbg_regs(self, version, start, end, client_id, ram_dump):
        client_name = self.dump_data_id_lookup_table[client_id]

        print_out_str(
            'Parsing {0} context start {1:x} end {2:x}'.format(client_name, start, end))

        sysregs = SysRegDump(start, end)
        if sysregs.dump_sysreg_img(ram_dump) is False:
            print_out_str('!!! Could not dump sysdbg_regs')
        else:
            ram_dump.sysreg = True
        return

    def parse_vsens(self, version, start, end, client_id, ram_dump):
        client_name = self.dump_data_id_lookup_table[client_id]

        print_out_str(
            'Parsing {0} context start {1:x} end {2:x}'.format(client_name, start, end))

        regs = VsensData()
        if regs.init_dump_regs(start, end, ram_dump) is False:
            print_out_str('!!! Could not get registers from Vsens Dump')
            return
        regs.print_vsens_regs(ram_dump)

    def parse_qdss_common(self, version, start, end, client_id, ram_dump):
        client_name = self.dump_data_id_lookup_table[client_id]

        print_out_str(
            'Parsing {0} context start {1:x} end {2:x}'.format(client_name, start, end))

        if client_id == client.MSM_DUMP_DATA_TMC_ETF_REG:
            setattr(self.qdss, 'tmc_etf_start', start)
        else:
            setattr(self.qdss, qdss_tag_to_field_name[client_name], start)

    def parse_cache_common(self, version, start, end, client_id, ramdump):
        if ramdump.skip_TLB_Cache_parse:
            return
        client_name = self.dump_data_id_lookup_table[client_id]
        core = client_id & 0xF
        filename = '{0}_0x{1:x}'.format(client_name, core)
        outfile = ramdump.open_file(filename)
        cache_type = lookup_cache_type(ramdump.hw_id, client_id, version)
        try:
            cache_type.parse(start, end, ramdump, outfile)
        except NotImplementedError:
            print_out_str('Cache dumping not supported for %s on this target'
                          % client_name)
        except:
            # log exceptions and continue by default
            if not ramdump.debug:
                print_out_str('!!! Unhandled exception while running {0}'.format(client_name))
                print_out_exception()
            else:
                raise
        outfile.close()

    def parse_system_cache_common(self, version, start, end, client_id, ramdump):
        if ramdump.skip_TLB_Cache_parse:
            return
        client_name = self.dump_data_id_lookup_table[client_id]
        bank_number = client_id - client.MSM_DUMP_DATA_LLC_CACHE
        filename = '{0}_0x{1:x}'.format(client_name, bank_number)
        outfile = ramdump.open_file(filename)
        cache_type = lookup_cache_type(ramdump.hw_id, client_id, version)
        try:
            cache_type.parse(start, end, ramdump, outfile)
        except NotImplementedError:
            print_out_str('System cache dumping not supported %s'
                          % client_name)
        except:
            # log exceptions and continue by default
            if not ramdump.debug:
                print_out_str('!!! Unhandled exception while running {0}'.format(client_name))
                print_out_exception()
            else:
                raise
        outfile.close()

    def parse_tlb_common(self, version, start, end, client_id, ramdump):
        if ramdump.skip_TLB_Cache_parse:
            return
        client_name = self.dump_data_id_lookup_table[client_id]
        core = client_id & 0xF
        filename = '{0}_0x{1:x}'.format(client_name, core)
        outfile = ramdump.open_file(filename)
        cache_type = lookup_tlb_type(ramdump.hw_id, client_id, version)
        try:
            cache_type.parse(start, end, ramdump, outfile)
        except NotImplementedError:
            print_out_str('TLB dumping not supported for %s on this target'
                          % client_name)
        except:
            # log exceptions and continue by default
            if not ramdump.debug:
                print_out_str('!!! Unhandled exception while running {0}'.format(client_name))
                print_out_exception()
            else:
                raise
        outfile.close()

    def ftrace_field_func(self, common_list, ram_dump):
        name_offset = ram_dump.field_offset('struct ftrace_event_field', 'name')
        type_offset = ram_dump.field_offset('struct ftrace_event_field', 'type')
        filter_type_offset = ram_dump.field_offset('struct ftrace_event_field', 'filter_type')
        field_offset = ram_dump.field_offset('struct ftrace_event_field', 'offset')
        size_offset = ram_dump.field_offset('struct ftrace_event_field', 'size')
        signed_offset = ram_dump.field_offset('struct ftrace_event_field', 'is_signed')

        name = ram_dump.read_word(common_list + name_offset)
        field_name = ram_dump.read_cstring(name, 256)
        type_name = ram_dump.read_word(common_list + type_offset)
        type_str = ram_dump.read_cstring(type_name, 256)
        offset = ram_dump.read_u32(common_list + field_offset)
        size = ram_dump.read_u32(common_list + size_offset)
        signed = ram_dump.read_u32(common_list + signed_offset)

        if re.match('(.*)\[(.*)', type_str) and not(re.match('__data_loc', type_str)):
            s = re.split('\[', type_str)
            s[1] = '[' + s[1]
            self.formats_out.write("\tfield:{0} {1}{2};\toffset:{3};\tsize:{4};\tsigned:{5};\n".format(s[0], field_name, s[1], offset, size, signed))
        else:
            self.formats_out.write("\tfield:{0} {1};\toffset:{2};\tsize:{3};\tsigned:{4};\n".format(type_str, field_name, offset, size, signed))

    def ftrace_events_func(self, ftrace_list, ram_dump):
        event_offset = ram_dump.field_offset(self.event_call, 'event')
        fmt_offset = ram_dump.field_offset(self.event_call,'print_fmt')
        class_offset = ram_dump.field_offset(self.event_call, 'class')
        flags_offset = ram_dump.field_offset(self.event_call, 'flags')
        flags = ram_dump.read_word(ftrace_list + flags_offset)
        if ram_dump.kernel_version >= (4, 14):
            TRACE_EVENT_FL_TRACEPOINT = 0x10
        elif ram_dump.kernel_version >= (4, 9):
            TRACE_EVENT_FL_TRACEPOINT = 0x20
        else:
            TRACE_EVENT_FL_TRACEPOINT = 0x40
        if (ram_dump.kernel_version >= (3, 18) and (flags & TRACE_EVENT_FL_TRACEPOINT)):
            tp_offset = ram_dump.field_offset(self.event_call, 'tp')
            tp_name_offset = ram_dump.field_offset('struct tracepoint', 'name')
            tp = ram_dump.read_word(ftrace_list + tp_offset)
            name = ram_dump.read_word(tp + tp_name_offset)
        else:
            name_offset = ram_dump.field_offset(self.event_call, 'name')
            name = ram_dump.read_word(ftrace_list + name_offset)

        type_offset = ram_dump.field_offset('struct trace_event', 'type')
        fields_offset = ram_dump.field_offset(self.event_class, 'fields')
        common_field_list = ram_dump.address_of('ftrace_common_fields')
        field_next_offset = ram_dump.field_offset('struct ftrace_event_field', 'link')

        name_str = ram_dump.read_cstring(name, 512)
        event_id = ram_dump.read_word(ftrace_list + event_offset + type_offset)
        fmt = ram_dump.read_word(ftrace_list + fmt_offset)
        fmt_str = ram_dump.read_cstring(fmt, 2048)

        self.formats_out.write("name: {0}\n".format(name_str))
        self.formats_out.write("ID: {0}\n".format(event_id))
        self.formats_out.write("format:\n")

        list_walker = llist.ListWalker(ram_dump, common_field_list, field_next_offset)
        list_walker.walk_prev(common_field_list, self.ftrace_field_func, ram_dump)
        self.formats_out.write("\n")

        event_class = ram_dump.read_word(ftrace_list + class_offset)
        field_list =  event_class + fields_offset
        list_walker = llist.ListWalker(ram_dump, field_list, field_next_offset)
        list_walker.walk_prev(field_list, self.ftrace_field_func, ram_dump)
        self.formats_out.write("\n")
        self.formats_out.write("print fmt: {0}\n".format(fmt_str))

    def collect_ftrace_format(self, ram_dump):
        self.formats_out = ram_dump.open_file('formats.txt')

        ftrace_events_list = ram_dump.address_of('ftrace_events')
        next_offset = ram_dump.field_offset(self.event_call, 'list')
        list_walker = llist.ListWalker(ram_dump, ftrace_events_list, next_offset)
        list_walker.walk_prev(ftrace_events_list, self.ftrace_events_func, ram_dump)

        self.formats_out.close

    def parse_sysreg(self,ram_dump):
        out_dir = ram_dump.outdir
        sysreg_parser_path_minidump = os.path.join(os.path.dirname(__file__), '..', 'dcc_parser',
                                                'sysregs_parser_minidump.py')
        if sysreg_parser_path_minidump is None:
            print_out_str("!!! Incorrect path for SYSREG specified.")
            return
        if not os.path.exists(sysreg_parser_path_minidump):
            print_out_str("!!! sysreg_parser_path_minidump {0} does not exist! "
                          "Check your settings!"
                          .format(sysreg_parser_path_minidump))
            return
        if os.path.getsize(os.path.join(out_dir, 'sysdbg_regs.bin')) > 0:
            sysdbg_file = os.path.join(out_dir, 'sysdbg_regs.bin')
        else:
            return
        p = subprocess.Popen([sys.executable, sysreg_parser_path_minidump, '-s', sysdbg_file, '--out-dir', out_dir],
                             stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT,
                             universal_newlines=True)
        print_out_str('--------')
        print_out_str(p.communicate()[0])

    def sorted_dump_data_clients(self, ram_dump, table, table_num_entries):
        """ Returns a sorted list of (client_name, func, client_address) where

        client_address --
            the (struct msm_dump_entry*) which contains a client_id mapping to
            client_name

        func --
            registered function in client_types to parse entries of
            this type

        the return value is sorted in the same order as the client names
        in client_types
        """

        dump_entry_id_offset = ram_dump.field_offset(
            'struct msm_dump_entry', 'id')
        dump_entry_size = ram_dump.sizeof('struct msm_dump_entry')
        results = list()

        client_table = dict(client_types)
        # get first column of client_types
        client_names = [x[0] for x in client_types]

        for j in range(0, table_num_entries):
            client_entry = table + j * dump_entry_size
            client_id = ram_dump.read_u32(
                            client_entry + dump_entry_id_offset, False)
            if client_id in self.dump_data_id_lookup_table:
                client_name = self.dump_data_id_lookup_table[client_id]
                if client_name not in client_table:
                    print_out_str(
                        '!!! client_id = 0x{0:x} client_name = {1} Does not have an associated function. Skipping!'.format(client_id,client_name))
                    continue
            else:
                print_out_str(
                    '!!! Invalid dump client id found 0x{0:x}'.format(client_id))
                continue

            results.append((client_name, client_table[client_name], client_entry))

        results.sort(key=lambda x: client_names.index(x[0]))
        return results
    def minidump_data_clients(self, ram_dump, client_id,entry_pa_addr,
                                      end_addr):
        results = list()
        client_table = dict(client_types)
        # get first column of client_types

        if client_id not in self.dump_data_id_lookup_table:
            print_out_str(
                '!!! {0} Unknown client id. Skipping!'.format(client_id))
            return None

        client_name = self.dump_data_id_lookup_table[client_id]

        if client_name not in client_table:
            print_out_str(
                '!!! {0} Does not have an associated function. Skipping!'.format(client_name))
            return None

        results.append((client_name, client_id,client_table[client_name], entry_pa_addr,end_addr))
        return results
    def parse_mhm_dump(self, version, start, end, client_id, ram_dump):
        if ram_dump.arm64:
            arch = "aarch64"
        else:
            arch = "aarch32"
        if client_id == client.MSM_DUMP_DATA_MHM:
            input = os.path.join(ram_dump.outdir, "mhm_scandump.bin")
        print_out_str(
            'Parsing mhm dump start {0:x} end {1:x} {2}'.format(start, end, input))
        header_bin = ram_dump.open_file(input, mode='wb')
        header_bin.write(ram_dump.read_physical(start, end - start))
        header_bin.close()
        return
    class MsmDumpTable(object):
        def __init__(self):
            self.name = "Anon"
            self.phys_addr = 0x0
            self.version = 0x0
            self.num_entries = 0x0

    """ Create an instance of MsmDumpTable, or None on error """
    def validateMsmDumpTable(self, ram_dump, name, table_phys):
        if table_phys is None:
            print_out_str('debug_image.py: Table {}: Unable to read dump table base address'.format(name))
            return None

        version = ram_dump.read_structure_field(
                    table_phys, 'struct msm_dump_table', 'version',
                    virtual = False)
        if version is None:
            print_out_str('Table {}: Version is bogus! Can\'t parse debug image'.format(name))
            return None

        num_entries = ram_dump.read_structure_field(
                        table_phys, 'struct msm_dump_table', 'num_entries',
                        virtual = False)
        if num_entries is None or num_entries > 100:
            print_out_str('Table {}: num_entries is bogus! Can\'t parse debug image'.format(name))
            return None

        table = self.MsmDumpTable()
        table.name = name
        table.phys_addr = table_phys
        table.version = version
        table.num_entries = num_entries
        return table

    def parse_dump_v2(self, ram_dump):
        self.dump_type_lookup_table = ram_dump.gdbmi.get_enum_lookup_table(
            'msm_dump_type', 2)
        cpus = ram_dump.get_num_cpus()
        # per cpu entries
        for i in ram_dump.iter_cpus():

                self.dump_data_id_lookup_table[
                    client.MSM_DUMP_DATA_CPU_CTX + i] = 'MSM_DUMP_DATA_CPU_CTX'
                self.dump_data_id_lookup_table[
                    client.MSM_DUMP_DATA_L1_INST_TLB + i] = 'MSM_DUMP_DATA_L1_INST_TLB'
                self.dump_data_id_lookup_table[
                    client.MSM_DUMP_DATA_L1_DATA_TLB + i] = 'MSM_DUMP_DATA_L1_DATA_TLB'
                self.dump_data_id_lookup_table[
                    client.MSM_DUMP_DATA_L1_INST_CACHE + i] = 'MSM_DUMP_DATA_L1_INST_CACHE'
                self.dump_data_id_lookup_table[
                    client.MSM_DUMP_DATA_L1_DATA_CACHE + i] = 'MSM_DUMP_DATA_L1_DATA_CACHE'
                self.dump_data_id_lookup_table[
                    client.MSM_DUMP_DATA_L2_CACHE + i] = 'MSM_DUMP_DATA_L2_CACHE'
                self.dump_data_id_lookup_table[
                    client.MSM_DUMP_DATA_L3_CACHE + i] = 'MSM_DUMP_DATA_L3_CACHE'
                self.dump_data_id_lookup_table[
                    client.MSM_DUMP_DATA_ETM_REG + i] = 'MSM_DUMP_DATA_ETM_REG'
                self.dump_data_id_lookup_table[
                    client.MSM_DUMP_DATA_SCANDUMP_PER_CPU + i] = 'MSM_DUMP_DATA_SCANDUMP_PER_CPU'

        for i in range(0, 4):
                self.dump_data_id_lookup_table[
                    client.MSM_DUMP_DATA_LLC_CACHE + i] = 'MSM_DUMP_DATA_LLC_CACHE'
        self.dump_data_id_lookup_table[
            client.MSM_DUMP_DATA_OCMEM] = 'MSM_DUMP_DATA_OCMEM'
        self.dump_data_id_lookup_table[
            client.MSM_DUMP_DATA_DBGUI_REG] = 'MSM_DUMP_DATA_DBGUI_REG'
        self.dump_data_id_lookup_table[
            client.MSM_DUMP_DATA_MISC] = 'MSM_DUMP_DATA_MISC'
        self.dump_data_id_lookup_table[
            client.MSM_DUMP_DATA_VSENSE] = 'MSM_DUMP_DATA_VSENSE'
        self.dump_data_id_lookup_table[
            client.MSM_DUMP_DATA_TMC_ETF] = 'MSM_DUMP_DATA_TMC_ETF'
        self.dump_data_id_lookup_table[
            client.MSM_DUMP_DATA_TMC_ETF_SWAO] = 'MSM_DUMP_DATA_TMC_ETF_SWAO'
        self.dump_data_id_lookup_table[
            client.MSM_DUMP_DATA_TMC_ETF_REG] = 'MSM_DUMP_DATA_TMC_ETF_REG'
        self.dump_data_id_lookup_table[
            client.MSM_DUMP_DATA_DCC_REG] = 'MSM_DUMP_DATA_DCC_REG'
        self.dump_data_id_lookup_table[
            client.MSM_DUMP_DATA_DCC_SRAM] = 'MSM_DUMP_DATA_DCC_SRAM'
        self.dump_data_id_lookup_table[
            client.MSM_DUMP_DATA_SCANDUMP] = 'MSM_DUMP_DATA_SCANDUMP'
        self.dump_data_id_lookup_table[
            client.MSM_DUMP_DATA_LLC_CACHE] = 'MSM_DUMP_DATA_LLC_CACHE'
        self.dump_data_id_lookup_table[
            client.MSM_DUMP_DATA_TMC_ETF_SWAO_REG] = 'MSM_DUMP_DATA_TMC_ETF_SWAO_REG'
        self.dump_data_id_lookup_table[
            client.MSM_DUMP_DATA_TMC_REG] = 'MSM_DUMP_DATA_TMC_REG'

        self.dump_data_id_lookup_table[
            client.MSM_DUMP_DATA_FCMDUMP] = 'MSM_DUMP_DATA_FCMDUMP'
        self.dump_data_id_lookup_table[
            client.MSM_DUMP_DATA_CPUSS] = 'MSM_DUMP_DATA_CPUSS'
        # 0x100 - tmc-etr registers and 0x101 - for tmc-etf registers
        self.dump_data_id_lookup_table[
            client.MSM_DUMP_DATA_TMC_ETR_REG + 1] = 'MSM_DUMP_DATA_TMC_ETR_REG'
        self.dump_data_id_lookup_table[
            client.MSM_DUMP_DATA_TMC_ETR1_REG] = 'MSM_DUMP_DATA_TMC_ETR1_REG'
        self.dump_data_id_lookup_table[
            client.MSM_DUMP_DATA_LOG_BUF] = 'MSM_DUMP_DATA_LOG_BUF'
        self.dump_data_id_lookup_table[
            client.MSM_DUMP_DATA_LOG_BUF_FIRST_IDX] = 'MSM_DUMP_DATA_LOG_BUF_FIRST_IDX'
        self.dump_data_id_lookup_table[
            client.MSM_DUMP_DATA_MHM] = 'MSM_DUMP_DATA_MHM'
        for i in ram_dump.iter_cpus():
            self.dump_data_id_lookup_table[client.MSM_DUMP_DATA_L2_TLB + i] = 'MSM_DUMP_DATA_L2_TLB'

        if not ram_dump.minidump or (ram_dump.minidump and ram_dump.kernel_version > (5,10,0)):
            dump_table_ptr_offset = ram_dump.field_offset(
                'struct msm_memory_dump', 'table')
            dump_table_version_offset = ram_dump.field_offset(
                'struct msm_dump_table', 'version')
            dump_table_num_entry_offset = ram_dump.field_offset(
                'struct msm_dump_table', 'num_entries')
            dump_table_entry_offset = ram_dump.field_offset(
                'struct msm_dump_table', 'entries')
            dump_entry_id_offset = ram_dump.field_offset(
                'struct msm_dump_entry', 'id')
            dump_entry_name_offset = ram_dump.field_offset(
                'struct msm_dump_entry', 'name')
            dump_entry_type_offset = ram_dump.field_offset(
                'struct msm_dump_entry', 'type')
            dump_entry_addr_offset = ram_dump.field_offset(
                'struct msm_dump_entry', 'addr')
            dump_data_version_offset = ram_dump.field_offset(
                'struct msm_dump_data', 'version')
            dump_data_magic_offset =  ram_dump.field_offset(
                'struct msm_dump_data', 'magic')
            dump_data_name_offset = ram_dump.field_offset(
                'struct msm_dump_data', 'name')
            dump_data_addr_offset = ram_dump.field_offset(
                'struct msm_dump_data', 'addr')
            dump_data_len_offset = ram_dump.field_offset(
                'struct msm_dump_data', 'len')
            dump_data_reserved_offset = ram_dump.field_offset(
                'struct msm_dump_data', 'reserved')
            dump_entry_size = ram_dump.sizeof('struct msm_dump_entry')
            dump_data_size = ram_dump.sizeof('struct msm_dump_data')

            """
            Some multi-guest hypervisor systems override the imem location
            with a table for a crashed guest. So the value from IMEM may
            not match the value saved in the linux variable 'memdump'.
            """
            if hasattr(ram_dump.board, 'imem_offset_memdump_table'):
                imem_dump_table_offset = ram_dump.board.imem_offset_memdump_table
            else:
                imem_dump_table_offset = IMEM_OFFSET_MEM_DUMP_TABLE
            if ram_dump.minidump and ram_dump.kernel_version >= (5, 10):
                for a in ram_dump.ebi_files:
                    md_pattern = re.compile(r'md_shrdimem', re.IGNORECASE)
                    if re.search(md_pattern, a[3]):
                        table_phys = ram_dump.read_word(a[1] + 0x10, virtual=False)
                        break
            else:
                table_phys = ram_dump.read_word(
                    ram_dump.board.imem_start + imem_dump_table_offset,
                    virtual = False)
            root_table = self.validateMsmDumpTable(ram_dump, "IMEM", table_phys)

            if root_table is None:
                table_phys = ram_dump.read_structure_field(
                    'memdump', 'struct msm_memory_dump', 'table_phys')
                root_table = self.validateMsmDumpTable(ram_dump, "RAM", table_phys)

            if root_table is None:
                return

            print_out_str('\nDebug image version: {0}.{1} Number of table entries {2}'.format(
                root_table.version >> 20, root_table.version & 0xFFFFF, root_table.num_entries))
            print_out_str('--------')

            out_dir =  ram_dump.outdir
            sdi_dump_out = open(os.path.join(out_dir , 'sdi_dump_table.txt') , 'w')
            sdi_dump_out.write("DumpTable base = 0x{0:02x} \n".format(root_table.phys_addr))
            sdi_dump_out.write("DumpTable Src = {0}\n".format(root_table.name))

            for i in range(0, root_table.num_entries):
                this_entry = root_table.phys_addr + dump_table_entry_offset + \
                    i * dump_entry_size
                entry_id = ram_dump.read_u32(this_entry + dump_entry_id_offset, virtual = False)
                entry_type = ram_dump.read_u32(this_entry + dump_entry_type_offset, virtual = False)
                entry_addr = ram_dump.read_word(this_entry + dump_entry_addr_offset, virtual = False)
                if entry_type > len(self.dump_type_lookup_table):
                    print_out_str(
                        '!!! Invalid dump table entry type found {0:x}'.format(entry_type))
                    continue
                table_version = ram_dump.read_u32(
                    entry_addr + dump_table_version_offset, False)
                if table_version is None:
                    print_out_str('Dump table entry version is bogus! Can\'t parse debug image')
                    continue
                table_num_entries = ram_dump.read_u32(
                    entry_addr + dump_table_num_entry_offset, False)
                if table_num_entries is None or table_num_entries > MAX_NUM_ENTRIES:
                    print_out_str('Dump table entry num_entries is bogus! Can\'t parse debug image')
                    continue

                print_out_str(
                    'Debug image version: {0}.{1} Entry type: {2} Number of entries: {3}'.format(
                        table_version >> 20, table_version & 0xFFFFF,
                        self.dump_type_lookup_table[entry_type], table_num_entries))

                lst = self.sorted_dump_data_clients(
                        ram_dump, entry_addr + dump_table_entry_offset,
                        table_num_entries)
                for (client_name, func, client_entry) in lst:
                    print_out_str('--------')
                    client_id = ram_dump.read_u32(
                                    client_entry + dump_entry_id_offset, False)
                    client_type = ram_dump.read_u32(
                                    client_entry + dump_entry_type_offset, False)
                    client_addr = ram_dump.read_word(
                                    client_entry + dump_entry_addr_offset, False)

                    if client_type > len(self.dump_type_lookup_table):
                        print_out_str(
                            '!!! Invalid dump client type found {0:x}'.format(client_type))
                        continue
                    dump_data_magic = ram_dump.read_u32(
                                    client_addr + dump_data_magic_offset, False)
                    dump_data_version = ram_dump.read_u32(
                                    client_addr + dump_data_version_offset, False)
                    dump_data_name = ram_dump.read_cstring(
                            client_addr + dump_data_name_offset,
                            ram_dump.sizeof('((struct msm_dump_data *)0x0)->name'),
                            False)
                    dump_data_addr = ram_dump.read_dword(
                                        client_addr + dump_data_addr_offset, False)
                    dump_data_len = ram_dump.read_dword(
                                        client_addr + dump_data_len_offset, False)
                    if dump_data_magic is None:
                        print_out_str("!!! Address {0:x} is bogus! Can't parse!".format(
                                    client_addr + dump_data_magic_offset))
                        continue
                    print_out_str('Parsing debug information for {0}. Version: {1} Magic: {2:x} Source: {3}'.format(
                        client_name, dump_data_version, dump_data_magic,
                        dump_data_name))
                    sdi_dump_out.write("Id = {0} type = {1} Addr = 0x{2:02x} "
                    "version {3}  magic {4} DataAddr 0x{5:02x}  DataLen {6} "
                    "Dataname {7} \n"
                    .format(client_id,client_type,client_addr,
                    dump_data_version,dump_data_magic,dump_data_addr,
                    dump_data_len,dump_data_name))
                    if dump_data_magic != MEMDUMPV2_MAGIC and dump_data_magic != MEMDUMPV_HYP_MAGIC:
                        print_out_str("!!! Magic {0:x} doesn't match! No context will be parsed".format(dump_data_magic))
                        continue
                    if "parse_cpu_ctx" in func:
                        getattr(DebugImage_v2, func)(
                            self, dump_data_version, dump_data_addr,
                            dump_data_addr + dump_data_len, client_id, ram_dump,dump_data_name=dump_data_name)
                    else:
                        getattr(DebugImage_v2, func)(
                            self, dump_data_version, dump_data_addr,
                            dump_data_addr + dump_data_len, client_id, ram_dump)
            sdi_dump_out.close()
        else:
            dump_table_num_entry_offset = ram_dump.field_offset(
                'struct md_table', 'num_regions')
            dump_table_entry_offset = ram_dump.field_offset(
                'struct md_table', 'entry')
            dump_entry_name_offset = ram_dump.field_offset(
                'struct md_region', 'name')
            dump_entry_id_offset = ram_dump.field_offset(
                'struct md_region', 'id')
            dump_entry_va_offset = ram_dump.field_offset(
                'struct md_region', 'virt_addr')
            dump_entry_pa_offset = ram_dump.field_offset(
                'struct md_region', 'phys_addr')
            dump_entry_size_offset = ram_dump.field_offset(
                'struct md_region', 'size')

            dump_entry_size = ram_dump.sizeof('struct md_region')

            mem_dump_data = ram_dump.address_of('minidump_table')

            mem_dump_table = ram_dump.read_word(
                mem_dump_data + dump_table_entry_offset)

            mem_table_num_entry = ram_dump.read_u32(
                mem_dump_data + dump_table_num_entry_offset)

            print_out_str('--------')

            for i in range(0, mem_table_num_entry):
                this_entry = mem_dump_data + dump_table_entry_offset + \
                             i * dump_entry_size
                entry_id = ram_dump.read_u32(this_entry + dump_entry_id_offset)
                entry_va_addr = ram_dump.read_u64(this_entry + dump_entry_va_offset)
                entry_pa_addr = ram_dump.read_u64(this_entry + dump_entry_pa_offset)
                entry_size = ram_dump.read_u64(this_entry + dump_entry_size_offset)

                end_addr = entry_pa_addr + entry_size
                minidump_dump_table_value = dict(minidump_dump_table_type)
                if entry_pa_addr in ram_dump.ebi_pa_name_map:
                    section_name = ram_dump.ebi_pa_name_map[entry_pa_addr]
                    section_name = re.sub("\d+", "", section_name)
                    #if section_name in minidump_dump_table_value.values():
                    lst = self.minidump_data_clients(
                        ram_dump, entry_id,entry_pa_addr,end_addr)
                    if lst:
                        client_name, client_id,func,\
                            client_entry,client_end = lst[0]
                        print_out_str('--------')
                        getattr(DebugImage_v2, func)(
                            self, 20, client_entry,
                            client_end, client_id, ram_dump)
        if ram_dump.sysreg:
            self.parse_sysreg(ram_dump)
        self.qdss.dump_standard(ram_dump)
        if not ram_dump.skip_qdss_bin and not ram_dump.minidump:
            self.qdss.save_etf_bin(ram_dump)
            self.qdss.save_etf_swao_bin(ram_dump)
            self.qdss.save_etr_bin(ram_dump)
        if ram_dump.ftrace_format:
            self.collect_ftrace_format(ram_dump)

