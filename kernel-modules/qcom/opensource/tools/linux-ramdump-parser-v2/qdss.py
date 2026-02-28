# Copyright (c) 2012, 2014-2018, 2020-2021 The Linux Foundation. All rights reserved.
# Copyright (c) 2022, 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
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
import itertools
import linux_list as llist
import ctypes
from print_out import print_out_str
from iommulib import IommuLib, MSM_SMMU_DOMAIN, MSM_SMMU_AARCH64_DOMAIN, ARM_SMMU_DOMAIN
from aarch64iommulib import create_flat_mappings, create_collapsed_mapping
from ramdump import Struct
from struct_print import struct_print_class

tmc_registers = {
    'RSZ': (0x004, 'RAM Size'),
    'STS': (0x00C, 'Status Register'),
    'RRD': (0x010, 'RAM Read Data Register'),
    'RRP': (0x014, 'RAM Read Pointer Register'),
    'RWP': (0x018, 'RAM Write Pointer Register'),
    'TRG': (0x01C, 'Trigger Counter Register'),
    'CTL': (0x020, 'Control Register'),
    'RWD': (0x024, 'RAM Write Data Register'),
    'MODE': (0x028, 'Mode Register'),
    'LBUFLEVEL': (0x02C, 'Latched Buffer Fill Level'),
    'CBUFLEVEL': (0x030, 'Current Buffer Fill Level'),
    'BUFWM': (0x034, 'Buffer Level Water Mark'),
    'RRPHI': (0x038, 'RAM Read Pointer High Register'),
    'RWPHI': (0x03C, 'RAM Write Pointer High Register'),
    'AXICTL': (0x110, 'AXI Control Register'),
    'DBALO': (0x118, 'Data Buffer Address Low Register'),
    'DBAHI': (0x11C, 'Data Buffer Address High Register'),
    'FFSR': (0x300, 'Formatter and Flush Status Register'),
    'FFCR': (0x304, 'Formatter and Flush Control Register'),
    'PSCR': (0x308, 'Periodic Synchronization Counter Register'),
    'ITATBMDATA0': (0xED0, 'Integration Test ATB Master Data Register 0'),
    'ITATBMCTR2': (0xED4, 'Integration Test ATB Master Interface Control 2 Register'),
    'ITATBMCTR1': (0xED8, 'Integration Test ATB Master Control Register 1'),
    'ITATBMCTR0': (0xEDC, 'Integration Test ATB Master Interface Control 0 Register'),
    'ITMISCOP0': (0xEE0, 'Integration Test Miscellaneous Output Register 0'),
    'ITTRFLIN': (0xEE8, 'Integration Test Trigger In and Flush In Register'),
    'ITATBDATA0': (0xEEC, 'Integration Test ATB Data Register 0'),
    'ITATBCTR2': (0xEF0, 'Integration Test ATB Control 2 Register'),
    'ITATBCTR1': (0xEF4, 'Integration Test ATB Control 1 Register'),
    'ITATBCTR0': (0xEF8, 'Integration Test ATB Control 0 Register'),
    'ITCTRL': (0xF00, 'Integration Mode Control Register'),
    'CLAIMSET': (0xFA0, 'Claim Tag Set Register'),
    'CLAIMCLR': (0xFA4, 'Claim Tag Clear Register'),
    'LAR': (0xFB0, 'Lock Access Register'),
    'LSR': (0xFB4, 'Lock Status Register'),
    'AUTHSTATUS': (0xFB8, 'Authentication Status Register'),
    'DEVID': (0xFC8, 'Device Configuration Register'),
    'DEVTYPE': (0xFCC, 'Device Type Identifier Register'),
    'PERIPHID4': (0xFD0, 'Peripheral ID4 Register'),
    'PERIPHID5': (0xFD4, 'Peripheral ID5 Register'),
    'PERIPHID6': (0xFD8, 'Peripheral ID6 Register'),
    'PERIPHID7': (0xFDC, 'Peripheral ID7 Register'),
    'PERIPHID0': (0xFE0, 'Peripheral ID0 Register'),
    'PERIPHID1': (0xFE4, 'Peripheral ID1 Register'),
    'PERIPHID2': (0xFE8, 'Peripheral ID2 Register'),
    'PERIPHID3': (0xFEC, 'Peripheral ID3 Register'),
    'COMPID0': (0xFF0, 'Component ID0 Register'),
    'COMPID1': (0xFF4, 'Component ID1 Register'),
    'COMPID2': (0xFF8, 'Component ID2 Register'),
    'COMPID3': (0xFFC, 'Component ID3 Register'),
}

etm_registers = {
    'ETMCR': (0x000, 'Main Control Register'),
    'ETMCCR': (0x001, 'Configuration Code Register'),
    'ETMTRIGGER': (0x002, 'Trigger Event Register'),
    'ETMSR': (0x004, 'Status Register'),
    'ETMCSR': (0x005, 'System Configuration Register'),
    'ETMTSSCR': (0x006, 'TraceEnable Start/Stop Control Register'),
    'ETMTEEVR': (0x008, 'TraceEnable Event Register'),
    'ETMTECR1': (0x009, 'TraceEnable Control Register'),
    'ETMFFLR': (0x00B, 'FIFOFULL Level Register'),
    'ETMACVR0': (0x10, 'Address Comparator Value Register 0'),
    'ETMACVR1': (0x11, 'Address Comparator Value Register 1'),
    'ETMACVR2': (0x12, 'Address Comparator Value Register 2'),
    'ETMACVR3': (0x13, 'Address Comparator Value Register 3'),
    'ETMACVR4': (0x14, 'Address Comparator Value Register 4'),
    'ETMACVR5': (0x15, 'Address Comparator Value Register 5'),
    'ETMACVR6': (0x16, 'Address Comparator Value Register 6'),
    'ETMACVR7': (0x17, 'Address Comparator Value Register 7'),
    'ETMACVR8': (0x18, 'Address Comparator Value Register 8'),
    'ETMACVR9': (0x19, 'Address Comparator Value Register 9'),
    'ETMACVRA': (0x1A, 'Address Comparator Value Register A'),
    'ETMACVRB': (0x1B, 'Address Comparator Value Register B'),
    'ETMACVRC': (0x1C, 'Address Comparator Value Register C'),
    'ETMACVRD': (0x1D, 'Address Comparator Value Register D'),
    'ETMACVRE': (0x1E, 'Address Comparator Value Register E'),
    'ETMACVRF': (0x1F, 'Address Comparator Value Register F'),

    'ETMACVT0': (0x20, 'Address Comparator Type Register 0'),
    'ETMACVT1': (0x21, 'Address Comparator Type Register 1'),
    'ETMACVT2': (0x22, 'Address Comparator Type Register 2'),
    'ETMACVT3': (0x23, 'Address Comparator Type Register 3'),
    'ETMACVT4': (0x24, 'Address Comparator Type Register 4'),
    'ETMACVT5': (0x25, 'Address Comparator Type Register 5'),
    'ETMACVT6': (0x26, 'Address Comparator Type Register 6'),
    'ETMACVT7': (0x27, 'Address Comparator Type Register 7'),
    'ETMACVT8': (0x28, 'Address Comparator Type Register 8'),
    'ETMACVT9': (0x29, 'Address Comparator Type Register 9'),
    'ETMACVTA': (0x2A, 'Address Comparator Type Register A'),
    'ETMACVTB': (0x2B, 'Address Comparator Type Register B'),
    'ETMACVTC': (0x2C, 'Address Comparator Type Register C'),
    'ETMACVTD': (0x2D, 'Address Comparator Type Register D'),
    'ETMACVTE': (0x2E, 'Address Comparator Type Register E'),
    'ETMACVTF': (0x2F, 'Address Comparator Type Register F'),

    'ETMCNTRLDVR0': (0x050, 'Counter Reload Value Register 0'),
    'ETMCNTRLDVR1': (0x051, 'Counter Reload Value Register 1'),
    'ETMCNTRLDVR2': (0x052, 'Counter Reload Value Register 2'),
    'ETMCNTRLDVR3': (0x053, 'Counter Reload Value Register 3'),

    'ETMCNTRENR0': (0x054, 'Counter Enable Event Register 0'),
    'ETMCNTRENR1': (0x055, 'Counter Enable Event Register 1'),
    'ETMCNTRENR2': (0x056, 'Counter Enable Event Register 2'),
    'ETMCNTRENR3': (0x057, 'Counter Enable Event Register 3'),

    'ETMCNTRLDEVR0': (0x058, 'Counter Reload Event Registers 0'),
    'ETMCNTRLDEVR1': (0x059, 'Counter Reload Event Registers 1'),
    'ETMCNTRLDEVR2': (0x05A, 'Counter Reload Event Registers 2'),
    'ETMCNTRLDEVR3': (0x05B, 'Counter Reload Event Registers 3'),

    'ETMCNTVR0': (0x05C, 'Counter Value Register 0'),
    'ETMCNTVR1': (0x05D, 'Counter Value Register 1'),
    'ETMCNTVR2': (0x05E, 'Counter Value Register 2'),
    'ETMCNTVR3': (0x05F, 'Counter Value Register 3'),

    'ETMSQabEVR0': (0x060, 'Sequencer State Transition Event Register 0'),
    'ETMSQabEVR1': (0x061, 'Sequencer State Transition Event Register 1'),
    'ETMSQabEVR2': (0x062, 'Sequencer State Transition Event Register 2'),
    'ETMSQabEVR3': (0x063, 'Sequencer State Transition Event Register 3'),
    'ETMSQabEVR4': (0x064, 'Sequencer State Transition Event Register 4'),
    'ETMSQabEVR5': (0x065, 'Sequencer State Transition Event Register 5'),

    'ETMSQR': (0x067, 'Current Sequencer State Register'),

    'ETMEXTOUTEVR0': (0x068, 'External Output Event Registers 0'),
    'ETMEXTOUTEVR1': (0x069, 'External Output Event Registers 1'),
    'ETMEXTOUTEVR2': (0x06A, 'External Output Event Registers 2'),
    'ETMEXTOUTEVR3': (0x06B, 'External Output Event Registers 3'),

    'ETMCIDCVR0': (0x06C, 'Context ID Comparator Value Register 0'),
    'ETMCIDCVR1': (0x06D, 'Context ID Comparator Value Register 1'),
    'ETMCIDCVR2': (0x06E, 'Context ID Comparator Value Register 2'),

    'ETMCIDCMR0': (0x06F, 'Context ID Mask Register'),

    'ETMIMPSPEC0': (0x070, 'Implementation Specific Register 0'),
    'ETMIMPSPEC1': (0x071, 'Implementation Specific Register 1'),
    'ETMIMPSPEC2': (0x072, 'Implementation Specific Register 2'),
    'ETMIMPSPEC3': (0x073, 'Implementation Specific Register 3'),
    'ETMIMPSPEC4': (0x074, 'Implementation Specific Register 4'),
    'ETMIMPSPEC5': (0x075, 'Implementation Specific Register 5'),
    'ETMIMPSPEC6': (0x076, 'Implementation Specific Register 6'),
    'ETMIMPSPEC7': (0x077, 'Implementation Specific Register 7'),

    'ETMSYNCFR': (0x078, 'Synchronization Frequency Register'),
    'ETMIDR': (0x079, 'ID register'),
    'ETMCCER': (0x07A, 'Configuration Code Extension Register'),
    'ETMEXTINSELR': (0x07B, 'Extended External Input Selection Register'),
    'ETMTESSEICR': (0x07C, 'TraceEnable Start/Stop EmbeddedICE Control Register'),
    'ETMEIBCR': (0x07D, 'EmbeddedICE Behavior COntrol Register'),
    'ETMTSEVR': (0x07E, 'Timestamp Event Register'),
    'ETMAUXCR': (0x07F, 'Auxilary Control Register'),
    'ETMTRACEIDR': (0x080, 'CoreSight Trace ID Register'),
    'ETMVMIDCVR': (0x090, 'VMID Comparator Value Register'),

    'ETMOSLAR': (0x0C0, 'OS Lock Access Register'),
    'ETMOSLSR': (0x0C1, 'OS Lock Status Register'),
    'ETMPDCR': (0x0C4, 'Device Power-DOwn Control Register'),
    'ETMPDSR': (0x0C5, 'Device Power Down Status Register'),

    'ETMITCTRL': (0x3C0, 'Integration Mode Control Register'),
    'ETMCLAIMSET': (0x3E8, 'Claim Tag Set Register'),
    'ETMCLAIMCLR': (0x3E9, 'Claim Tag Clear Register'),
    'ETMLAR': (0x3Ec, 'Lock Access Register'),
    'ETMLSR': (0x3ED, 'Lock Status Register'),
    'ETMAUTHSTATUS': (0x3EE, 'Authentication Status Register'),
    'ETMDEVID': (0x3F2, 'Device Configuration Register'),
    'ETMDEVTYPE': (0x3F3, 'Device Type Register'),
    'ETMPIDR4': (0x3F4, 'Peripheral ID4 Register'),
    'ETMPIDR0': (0x3F8, 'Peripheral ID0 Register'),
    'ETMPIDR1': (0x3F9, 'Peripheral ID1 Register'),
    'ETMPIDR2': (0x3FA, 'Peripheral ID2 Register'),
    'ETMPIDR3': (0x3FB, 'Peripheral ID3 Register'),
    'ETMCIDR0': (0x3FC, 'Component ID0 Register'),
    'ETMCIDR1': (0x3FD, 'Component ID1 Register'),
    'ETMCIDR2': (0x3FE, 'Component ID2 Register'),
}

dbgui_registers = {
    'DBGUI_SECURE' : (0x000, 'Secure Register'),
    'DBGUI_CTL' : (0x004, 'Clear Register'),
    'DBGUI_CTL_MASK' : (0x008, 'CTL Mask Register'),
    'DBGUI_SWTRIG' : (0x00C, 'Software Trigger Register'),
    'DBGUI_STATUS' : (0x010, 'Status Register Register'),
    'DBGUI_HWE_MASK' : (0x014, 'Hardware Event Mask Register'),
    'DBGUI_CTR_VAL' : (0x018, 'Timeout Counter Terminal Value Register'),
    'DBGUI_CTR_EN' : (0x01C, 'Timeout Counter Enable Register'),
    'DBGUI_NUM_REGS_RD' : (0x020, 'Number of Register Read Control Register'),
    'DBGUI_ATB_REG' : (0x024, 'ATB Configuration Register'),
}

driver_types = [
    ('coresight-stm', 'parse_single_atid'),
    ('coresight-tpdm', 'parse_single_atid'),
    ('coresight-remote-etm', 'parse_remote_etm_atid'),
    ('coresight-etm4x', 'parse_single_atid'),
    ('coresight-dummy', 'parse_single_atid'),
    ('coresight-uetm', 'parse_single_atid'),
]

driver_structs = [
    ('coresight-stm', 'struct stm_drvdata'),
    ('coresight-tpdm', 'struct tpdm_drvdata'),
    ('coresight-remote-etm', 'struct remote_etm_drvdata'),
    ('coresight-etm4x', 'struct etmv4_drvdata'),
    ('coresight-dummy', 'struct dummy_drvdata'),
    ('coresight-uetm', 'struct uetm_drvdata'),
]

qdss_atid_fields = [
    ('coresight-stm', 'traceid'),
    ('coresight-tpdm', 'traceid'),
    ('coresight-remote-etm', 'traceid'),
    ('coresight-etm4x', 'trcid'),
    ('coresight-dummy', 'traceid'),
    ('coresight-uetm', 'traceid'),
]

qdss_component_func = [
	('coresight-tpdm', 'parse_tpdm_component'),
	('coresight-tpda', 'parse_tpda_component'),
	('coresight-stm', 'parse_stm_component'),
	('coresight-uetm', 'parse_uetm_component'),
	('coresight-qmi', 'parse_qmi_component'),
	('coresight-static-tpdm', 'parse_static_tpdm_component'),
	('coresight-trace-noc', 'parse_trace_noc_component'),
	('coresight-remote-etm', 'parse_remote_etm_component'),
	('coresight-tmc', 'parse_tmc_component'),
	('coresight-csr', 'parse_csr_component'),
	('coresight-dummy', 'parse_dummy_component'),
	('coresight-tgu', 'parse_tgu_component'),
	('coresight-cti', 'parse_cti_component'),
	('coresight-secure-etr', 'parse_secure_etr_component'),
	('coresight-etm4x', 'parse_etm4_platform_component'),
	('coresight-dynamic-funnel', 'parse_dynamic_funnel_component'),
	('coresight-static-funnel', 'parse_static_funnel_component'),
	('coresight-dynamic-replicator', 'parse_dynamic_replicator_component'),
	('coresight-static-replicator', 'parse_static_replicator_component'),
]
class QDSSDump():

    def __init__(self):
        self.tmc_etr_start = None
        self.tmc_etr1_start= None
        self.etf_start = None
        self.tmc_etf_start = None
        self.etm_regs0 = None
        self.etm_regs1 = None
        self.etm_regs2 = None
        self.etm_regs3 = None
        self.dbgui_start = None
        self.tmc_etf_swao_start = None
        self.tmc_etf_swao_reg_start = None

    # Assumptions: Any address given here has been checked for correct magic
    def print_tmc_etf(self, ram_dump):
        if self.tmc_etf_start is None:
            print_out_str(
                "!!! TMC-ETF address has not been set! I can't continue!")
            return

        print_out_str('Now printing TMC-ETF registers to file')
        tmc_etf_out = ram_dump.open_file('tmc_etf.txt')
        for a, b in tmc_registers.items():
            offset, name = b
            tmc_etf_out.write('{0} ({1}): {2:x}\n'.format(
                a, name, ram_dump.read_u32(self.tmc_etf_start + offset, False)))
        tmc_etf_out.close()

    def print_tmc_etf_swao(self, ram_dump):
        if self.tmc_etf_swao_reg_start is None:
            print_out_str(
                "!!! TMC-ETF-SWAO address has not been set! I can't continue!")
            return

        print_out_str('Now printing TMC-ETF-SWAO registers to file')
        tmc_etf_out = ram_dump.open_file('tmc_etf_swao.txt')
        for a, b in tmc_registers.items():
            offset, name = b
            tmc_etf_out.write('{0} ({1}): {2:x}\n'.format(
                a, name, ram_dump.read_u32(self.tmc_etf_swao_reg_start + offset, False)))
        tmc_etf_out.close()

    def print_tmc_etr(self, ram_dump):
        if self.tmc_etr_start is None:
            print_out_str(
                "!!! TMC-ETR address has not been set! I can't continue!")
            return

        print_out_str('Now printing TMC-ETR registers to file')
        tmc_etf_out = ram_dump.open_file('tmc_etr.txt')
        for a, b in tmc_registers.items():
            offset, name = b
            tmc_etf_out.write('{0} ({1}): {2:x}\n'.format(
                a, name, ram_dump.read_u32(self.tmc_etr_start + offset, False)))
        tmc_etf_out.close()

        if self.tmc_etr1_start is None:
            print_out_str(
                "!!! TMC-ETR1 address has not been set! I can't continue!")
            return

        print_out_str('Now printing TMC-ETR1 registers to file')
        tmc_etf_out = ram_dump.open_file('tmc_etr1.txt')
        for a, b in tmc_registers.items():
            offset, name = b
            tmc_etf_out.write('{0} ({1}): {2:x}\n'.format(
                a, name, ram_dump.read_u32(self.tmc_etr1_start + offset, False)))
        tmc_etf_out.close()

    def print_etm_registers(self, ram_dump, base, fname):
        etm_out = ram_dump.open_file(fname)
        for a, b in etm_registers.items():
            offset, name = b
            etm_out.write('{0} ({1}): {2:x})\n'.format(
                a, name, ram_dump.read_u32(base + offset * 4, False)))
        etm_out.close()

    def print_all_etm_register(self, ram_dump):
        if self.etm_regs0 is None:
            print_out_str(
                '!!! ETM REGS 0 address was not set! Nothing will be parsed')
        else:
            self.print_etm_registers(ram_dump, self.etm_regs0, 'etm_regs0')

        if self.etm_regs1 is None:
            print_out_str(
                '!!! ETM REGS 1 address was not set! Nothing will be parsed')
        else:
            self.print_etm_registers(ram_dump, self.etm_regs1, 'etm_regs1')

        if self.etm_regs2 is None:
            print_out_str(
                '!!! ETM REGS 2 address was not set! Nothing will be parsed')
        else:
            self.print_etm_registers(ram_dump, self.etm_regs2, 'etm_regs2')

        if self.etm_regs3 is None:
            print_out_str(
                '!!! ETM REGS 3 address was not set! Nothing will be parsed')
        else:
            self.print_etm_registers(ram_dump, self.etm_regs3, 'etm_regs3')

    def save_etf_bin(self, ram_dump):
        tmc_etf = ram_dump.open_file('tmc-etf.bin', mode='wb')
        if self.tmc_etf_start is None or self.etf_start is None:
            print_out_str('!!! ETF was not the current sink!')
            tmc_etf.close()
            return

        ctl_offset, ctl_desc = tmc_registers['CTL']
        mode_offset, mode_desc = tmc_registers['MODE']
        rsz_offset, rsz_desc = tmc_registers['RSZ']

        ctl = ram_dump.read_u32(self.tmc_etf_start + ctl_offset, False)
        mode = ram_dump.read_u32(self.tmc_etf_start + mode_offset, False)
        rsz = ram_dump.read_u32(self.tmc_etf_start + rsz_offset, False)
        # rsz is given in words so convert to bytes
        rsz = 4 * rsz

        if (ctl & 0x1) == 1 and (mode == 0):
            for i in range(0, rsz):
                val = ram_dump.read_byte(self.etf_start + i, False)
                tmc_etf.write(struct.pack('<B', val))
        else:
            print_out_str('!!! ETF was not the current sink!')

        tmc_etf.close()

    def save_etf_swao_bin(self, ram_dump):
        tmc_etf_swao = ram_dump.open_file('tmc-etf-swao.bin', mode='wb')
        if self.tmc_etf_swao_reg_start is None or self.tmc_etf_swao_start is None:
            print_out_str('!!! ETF SWAO was not the current sink!')
            tmc_etf_swao.close()
            return

        ctl_offset, ctl_desc = tmc_registers['CTL']
        mode_offset, mode_desc = tmc_registers['MODE']
        rsz_offset, rsz_desc = tmc_registers['RSZ']

        ctl = ram_dump.read_u32(self.tmc_etf_swao_reg_start + ctl_offset, False)
        mode = ram_dump.read_u32(self.tmc_etf_swao_reg_start + mode_offset, False)
        rsz = ram_dump.read_u32(self.tmc_etf_swao_reg_start + rsz_offset, False)
        # rsz is given in words so convert to bytes
        rsz = 4 * rsz

        if (ctl & 0x1) == 1 and (mode == 0):
            for i in range(0, rsz):
                val = ram_dump.read_byte(self.tmc_etf_swao_start + i, False)
                tmc_etf_swao.write(struct.pack('<B', val))
        else:
            print_out_str('!!! ETF SWAO was not the current sink!')

        tmc_etf_swao.close()

    def read_sg_data(self, dbaddr, sts, rwpval, ram_dump, tmc_etr):
        start = dbaddr
        continue_looping = True
        if (sts & 0x1) == 1:
            bottom_delta_read = False
            while continue_looping:
                entry = ram_dump.read_u32(start, False)
                blk = (entry >> 4) << 12
                if (entry & 0x3) == 3:
                    start = blk
                    continue
                elif (entry & 0x2) == 2:
                    if blk <= rwpval and rwpval < (blk + 4096):
                        if not bottom_delta_read:
                            it = range(rwpval, blk + 4096)
                            bottom_delta_read = True
                        else:
                            it = range(blk, blk + (rwpval - blk))
                            continue_looping = False
                    elif bottom_delta_read:
                        it = range(blk, blk + 4096)
                    else:
                        start += 4
                        continue
                    start += 4
                elif (entry & 0x1) == 1:
                    if blk <= rwpval and rwpval < (blk + 4096):
                        if not bottom_delta_read:
                            it = range(rwpval, blk + 4096)
                            bottom_delta_read = True
                        else:
                            it = range(blk, blk + (rwpval - blk))
                            continue_looping = False
                    elif bottom_delta_read:
                        it = range(blk, blk + 4096)
                    else:
                        start = dbaddr
                        continue
                    start = dbaddr
                else:
                    break
                tmc_etr.write(ram_dump.read_physical(it[0], len(it)))
        else:
            while continue_looping:
                entry = ram_dump.read_u32(start, False)
                blk = (entry >> 4) << 12
                if (entry & 0x3) == 3:
                    start = blk
                    continue
                elif (entry & 0x2) == 2:
                    it = range(blk, blk + 4096)
                    start += 4
                elif (entry & 0x1) == 1:
                    it = range(blk, blk + 4096)
                    continue_looping = False
                else:
                    break
                tmc_etr.write(ram_dump.read_physical(it[0], len(it)))

    def dump_etr_iova(self, start, size, ram_dump, tmc_etr, collapsed_mapping):
        pyh_start = None;
        for virt in sorted(collapsed_mapping.keys()):
            mapping = collapsed_mapping[virt]
            if mapping.mapped and size != 0:
                if start in range(mapping.virt_start, mapping.virt_end):
                    dump_size = min(size, mapping.virt_end - start + 1)
                    pyh_start = mapping.phys_start + (start - mapping.virt_start)
                    it = range(pyh_start, pyh_start + dump_size)
                    size = size - dump_size
                    start = start + dump_size
                    #pyh_start lower 12 bit is PTE flag, so mask the flag.
                    tmc_etr.write(ram_dump.read_physical((it[0] & 0xFFFFFFFFF000), len(it)))
        if pyh_start is None:
            return False
        else:
            return True

    def parse_domain(self, dbaddr, rsz, sts, rwpval, ram_dump, tmc_etr, d, domain_num):
        if d.client_name.endswith(".tmc") or d.client_name.endswith(".etr"):
            flat_mapping = create_flat_mappings(ram_dump, d.pg_table, d.level)
            collapsed_mapping = create_collapsed_mapping(flat_mapping)
            if (sts & 0x1) == 1:
                self.dump_etr_iova(rwpval, dbaddr + rsz - rwpval, ram_dump, tmc_etr, collapsed_mapping)
                return self.dump_etr_iova(dbaddr, rwpval - dbaddr, ram_dump, tmc_etr, collapsed_mapping)
            else:
                return self.dump_etr_iova(dbaddr, rsz, ram_dump, tmc_etr, collapsed_mapping)
        else:
            return False;
        return True;

    def read_data_iova(self, dbaddr, rsz, sts, rwpval, ram_dump, tmc_etr):
        ilib = IommuLib(ram_dump)
        domain_list = ilib.domain_list
        if domain_list is None:
            return False
        for (domain_num, d) in enumerate(domain_list):
            if ((d.domain_type == ARM_SMMU_DOMAIN) or
                    (d.domain_type == MSM_SMMU_AARCH64_DOMAIN)):
                if self.parse_domain(dbaddr, rsz, sts, rwpval, ram_dump, tmc_etr, d, domain_num):
                    print_out_str("Found a correct domain for tmc")
                    return True
        return False

    def save_etr_bin(self, ram_dump):
        if self.tmc_etr_start is None:
            print_out_str('!!! ETR was not enabled!')
            return
        tmc_etr = ram_dump.open_file('tmc-etr.bin', mode='wb')
        self.do_save_etr_bin(ram_dump, tmc_etr, self.tmc_etr_start)
        tmc_etr.close()

        if self.tmc_etr1_start is None:
            print_out_str('!!! ETR1 was not enabled!')
            return
        tmc_etr1 = ram_dump.open_file('tmc-etr1.bin', mode='wb')
        self.do_save_etr_bin(ram_dump, tmc_etr1, self.tmc_etr1_start)
        tmc_etr1.close()

    def do_save_etr_bin(self, ram_dump, tmc_etr, tmc_etr_start):
        ctl_offset, ctl_desc = tmc_registers['CTL']
        mode_offset, mode_desc = tmc_registers['MODE']

        ctl = ram_dump.read_u32(tmc_etr_start + ctl_offset, False)
        mode = ram_dump.read_u32(tmc_etr_start + mode_offset, False)

        if (ctl & 0x1) == 1 and (mode == 0):
            sts_offset, sts_desc = tmc_registers['STS']
            sts = ram_dump.read_u32(tmc_etr_start + sts_offset, False)

            dbalo_offset, dbalo_desc = tmc_registers['DBALO']
            dbalo = ram_dump.read_u32(
                tmc_etr_start + dbalo_offset, False)
            dbahi_offset, dbahi_desc = tmc_registers['DBAHI']
            dbahi = ram_dump.read_u32(
                tmc_etr_start + dbahi_offset, False)
            dbaddr = (dbahi << 32) + dbalo

            rsz_offset, rsz_desc = tmc_registers['RSZ']
            rsz = ram_dump.read_u32(tmc_etr_start + rsz_offset, False)
            # rsz is given in words so convert to bytes
            rsz = 4 * rsz

            rwp_offset, rwp_desc = tmc_registers['RWP']
            rwp = ram_dump.read_u32(tmc_etr_start + rwp_offset, False)
            rwphi_offset, rwphi_desc = tmc_registers['RWPHI']
            rwphi = ram_dump.read_u32(tmc_etr_start + rwphi_offset, False)
            rwpval = (rwphi << 32) + rwp

            axictl_offset, axictl_desc = tmc_registers["AXICTL"]
            axictl = ram_dump.read_u32(tmc_etr_start + axictl_offset, False)

            if ((axictl >> 7) & 0x1) == 1:
                print_out_str('Scatter gather memory type was selected for TMC ETR')
                self.read_sg_data(dbaddr, sts, rwpval, ram_dump, tmc_etr)
            else:
                if self.read_data_iova(dbaddr, rsz, sts, rwpval, ram_dump, tmc_etr) == False:
                    print_out_str('Contiguous memory type was selected for TMC ETR')
                    if (sts & 0x1) == 1:
                        it1 = range(rwpval, dbaddr+rsz)
                        it2 = range(dbaddr, rwpval)
                        tmc_etr.write(ram_dump.read_physical(it1[0], len(it1)))
                        tmc_etr.write(ram_dump.read_physical(it2[0], len(it2)))
                    else:
                        it = range(dbaddr, dbaddr+rsz)
                        tmc_etr.write(ram_dump.read_physical(it[0], len(it)))
        else:
            print_out_str ('!!! ETR was not the current sink!')

    def print_dbgui_registers(self, ram_dump):
        if self.dbgui_start is None:
            print_out_str(
                "!!!DBGUI address has not been  set! I can't continue!")
            return

        print_out_str('Now printing DBGUI registers to file')
        dbgui_out = ram_dump.open_file('dbgui.txt')
        for a, b in dbgui_registers.items():
            offset, name = b
            dbgui_out.write('{0} ({1}): {2:x}\n'.format(
                a, name, ram_dump.read_u32(self.dbgui_start + offset, False)))

        addr = ram_dump.read_word(ram_dump.address_of('dbgui_drvdata'))
        addr_offset_offset = ram_dump.field_offset('struct dbgui_drvdata', 'addr_offset')
        data_offset_offset = ram_dump.field_offset('struct dbgui_drvdata', 'data_offset')
        size_offset = ram_dump.field_offset('struct dbgui_drvdata', 'size')
        addr_offset = ram_dump.read_u32(addr + addr_offset_offset, True)
        data_offset = ram_dump.read_u32(addr + data_offset_offset, True)
        size = ram_dump.read_u32(addr + size_offset, True)

        for i in range(0, size):
            dbgui_out.write('ADDR_{0} ({1:x}) : {2:x}\n'.format(
                i, ram_dump.read_u32(self.dbgui_start + addr_offset + (4 * i), False),
                ram_dump.read_u32(self.dbgui_start + data_offset + (4 * i), False)))
        dbgui_out.close()

    def parse_single_atid(self, driver_name, drvdata, struct_name, atid_field):
        atid_offset = self.ramdump.struct_field_addr(drvdata, struct_name, atid_field)
        if atid_offset is None:
            return
        atid = self.ramdump.read_byte(atid_offset)
        csdev = self.ramdump.read_structure_field(drvdata, struct_name, 'csdev')
        dev = self.ramdump.struct_field_addr(csdev, 'struct coresight_device', 'dev')
        csname = self.ramdump.read_cstring(self.ramdump.read_word(dev + self.name_offset))
        print("{:<50} : {:#04x}".format(csname, atid),file = self.f)

    def parse_remote_etm_atid(self, driver_name, drvdata, struct_name, atid_field):
        atid_str = ''
        atid_num = self.ramdump.read_structure_field(drvdata, 'struct remote_etm_drvdata', 'num_trcid')
        if atid_num is None:
            return

        atid_addr = self.ramdump.read_structure_field(drvdata, "struct remote_etm_drvdata", "traceids")
        for i in range(atid_num):
            atid = self.ramdump.read_byte(atid_addr)
            atid_str = "{:#04x}".format(atid) + " " + atid_str
            atid_addr = atid_addr + 1

        remote_etm_csdev = self.ramdump.read_structure_field(drvdata, 'struct remote_etm_drvdata', 'csdev')
        cs_dev = self.ramdump.struct_field_addr(remote_etm_csdev, 'struct coresight_device', 'dev')
        csname = self.ramdump.read_cstring(self.ramdump.read_word(cs_dev + self.name_offset))
        print("{:<50} : {}".format(csname, atid_str), file = self.f)

    def list_qdss_atid(self, device):
        drv = self.ramdump.read_structure_field(device, 'struct device', 'driver')
        drvdata = self.ramdump.read_structure_field(device, 'struct device', 'driver_data')
        drvname = self.ramdump.read_cstring(self.ramdump.read_word(drv + self.dev_drv_offset))

        if drvname in self.qdss_drivers :
            getattr(QDSSDump, self.qdss_drivers[drvname])(self, drvname, drvdata,
                            self.qdss_structs[drvname], self.atid_fields[drvname])

    def parse_qdss_component_atid(self, ramdump):
        self.ramdump = ramdump
        self.entry_offset = self.ramdump.field_offset('struct kobject', 'entry')
        self.name_offset = self.ramdump.field_offset('struct kobject', 'name')
        self.dev_drv_offset = self.ramdump.field_offset('struct device_driver', 'name')
        self.kobj_offset = self.ramdump.field_offset('struct device', 'kobj')
        self.qdss_drivers = dict(driver_types)
        self.qdss_structs = dict(driver_structs)
        self.atid_fields = dict(qdss_atid_fields)
        self.f = open(self.ramdump.outdir + "/ATID.txt", "w")
        print("{:<50} {}".format("Source Name", "ATID"), file = self.f)
        print("{}".format("=" * 60), file = self.f)
        devices_kset = self.ramdump.read_pointer('devices_kset')
        list_head = devices_kset + self.ramdump.field_offset('struct kset', 'list')
        list_offset = self.kobj_offset + self.entry_offset
        list_walker = llist.ListWalker(self.ramdump, list_head, list_offset)
        list_walker.walk(self.list_qdss_atid)
        self.f.close()

    def parse_qdss_field(self, addr, stru, field, tab=1, HEX=False, str=False, indent=1):
        if str:
            val = self.ramdump.read_structure_cstring(addr, stru, field)
        else:
            val = self.ramdump.read_structure_field(addr, stru, field)
            if HEX:
                val = hex(val)
        print("{}{}{}= {},".format('\t'*indent, field, '\t'*tab, val), file = self.f)

    def parse_clk_core(self, core):
        print("struct clk_core {} :".format(hex(core)), file = self.f)
        print("{", file = self.f)
        self.parse_qdss_field(core, 'struct clk_core', 'name', 2, str=True)
        self.parse_qdss_field(core, 'struct clk_core', 'hw', 3, HEX=True)
        self.parse_qdss_field(core, 'struct clk_core', 'dev', 3, HEX=True)
        self.parse_qdss_field(core, 'struct clk_core', 'of_node', 2, HEX=True)
        self.parse_qdss_field(core, 'struct clk_core', 'rate', 2)
        self.parse_qdss_field(core, 'struct clk_core', 'req_rate')
        self.parse_qdss_field(core, 'struct clk_core', 'new_rate')
        self.parse_qdss_field(core, 'struct clk_core', 'flags', 2)
        self.parse_qdss_field(core, 'struct clk_core', 'orphan', 2)
        self.parse_qdss_field(core, 'struct clk_core', 'rpm_enabled')
        self.parse_qdss_field(core, 'struct clk_core', 'need_sync')
        self.parse_qdss_field(core, 'struct clk_core', 'boot_enabled')
        self.parse_qdss_field(core, 'struct clk_core', 'enable_count')
        self.parse_qdss_field(core, 'struct clk_core', 'prepare_count')
        self.parse_qdss_field(core, 'struct clk_core', 'protect_count')
        self.parse_qdss_field(core, 'struct clk_core', 'phase', 2)
        self.parse_qdss_field(core, 'struct clk_core', 'notifier_count')
        self.parse_qdss_field(core, 'struct clk_core', 'ref', 3)
        print("}", file = self.f)

    def parse_clk(self, clk):
        errno_max = -1000
        if clk == 0 or clk > ctypes.c_uint64(errno_max).value:
            return
        core = self.ramdump.read_structure_field(clk, 'struct clk', 'core')

        print("struct clk {} :".format(hex(clk)), file = self.f)
        print("{", file = self.f)
        self.parse_qdss_field(clk, 'struct clk', 'core', tab=2, HEX=True)
        self.parse_qdss_field(clk, 'struct clk', 'dev_id', tab=2, str=True)
        self.parse_qdss_field(clk, 'struct clk', 'con_id', tab=2, str=True)
        self.parse_qdss_field(clk, 'struct clk', 'min_rate', HEX=True)
        self.parse_qdss_field(clk, 'struct clk', 'max_rate', HEX=True)
        self.parse_qdss_field(clk, 'struct clk', 'exclusive_count')
        print("}", file = self.f)
        self.clk_core_set.add(core)

    def parse_csdev(self, csdev):
        cstype = self.ramdump.read_structure_field(csdev, 'struct coresight_device', 'type')
        cstype = self.ramdump.enum_lookup('enum coresight_dev_type', cstype)

        print("struct coresight_device {} :".format(hex(csdev)), file = self.f)
        print("{", file = self.f)
        print("\ttype\t\t= {},".format(cstype), file =self.f)
        self.parse_qdss_field(csdev, 'struct coresight_device', 'refcnt', tab=2)
        self.parse_qdss_field(csdev, 'struct coresight_device', 'orphan', tab=2)
        if (self.ramdump.kernel_version < (6, 9, 0)):
            self.parse_qdss_field(csdev, 'struct coresight_device', 'enable', tab=2)
            self.parse_qdss_field(csdev, 'struct coresight_device', 'activated')
        else:
            mode = self.ramdump.read_structure_field(csdev, 'struct coresight_device', 'mode')
            mode = self.ramdump.enum_lookup('enum cs_mode', mode)
            print("\tmode\t\t= {},".format(mode), file =self.f)
            self.parse_qdss_field(csdev, 'struct coresight_device', 'sysfs_sink_activated')
        self.parse_qdss_field(csdev, 'struct coresight_device', 'def_sink')
        self.parse_qdss_field(csdev, 'struct coresight_device', 'nr_links')
        self.parse_qdss_field(csdev, 'struct coresight_device', 'has_conns_grp')
        print("}", file = self.f)

    def parse_gpr_dataset(self, gpr):
        gpr_dirty = self.read_array(gpr + self.ramdump.field_offset('struct gpr_dataset', 'gpr_dirty'), 3, 'u64')
        gp_regs = self.read_array(gpr + self.ramdump.field_offset('struct gpr_dataset', 'gp_regs'), 3, 'u64')

        print("struct gpr_dataset {} :".format(hex(gpr)), file = self.f)
        print("{", file = self.f)
        print("\tgpr_dirty\t= {},".format(gpr_dirty), file =self.f)
        print("\tgp_regs\t\t= {}".format(gp_regs), file =self.f)
        print("}", file = self.f)

    def parse_bc_dataset(self, bc):
        capture_mode = self.ramdump.read_structure_field(bc, 'struct bc_dataset', 'capture_mode')
        capture_mode = self.ramdump.enum_lookup('enum tpdm_mode', capture_mode)
        retrieval_mode = self.ramdump.read_structure_field(bc, 'struct bc_dataset', 'retrieval_mode')
        retrieval_mode = self.ramdump.enum_lookup('enum tpdm_mode', retrieval_mode)
        trig_val_lo = self.read_array(bc + self.ramdump.field_offset('struct bc_dataset', 'trig_val_lo'), 32, 'u32')
        trig_val_hi = self.read_array(bc + self.ramdump.field_offset('struct bc_dataset', 'trig_val_hi'), 32, 'u32')
        overflow_val = self.read_array(bc + self.ramdump.field_offset('struct bc_dataset', 'overflow_val'), 6, 'u32')
        msr = self.read_array(bc + self.ramdump.field_offset('struct bc_dataset', 'msr'), 4, 'u32')

        print("struct bc_dataset {} :".format(hex(bc)), file = self.f)
        print("{", file = self.f)
        print("\tcapture_mode\t= {},".format(capture_mode), file =self.f)
        print("\tretrieval_mode\t= {},".format(retrieval_mode), file =self.f)
        self.parse_qdss_field(bc, 'struct bc_dataset', 'sat_mode', 2)
        self.parse_qdss_field(bc, 'struct bc_dataset', 'enable_counters')
        self.parse_qdss_field(bc, 'struct bc_dataset', 'clear_counters')
        self.parse_qdss_field(bc, 'struct bc_dataset', 'enable_irq', 2)
        self.parse_qdss_field(bc, 'struct bc_dataset', 'clear_irq', 2)
        print("\ttrig_val_lo\t= {},".format(trig_val_lo), file =self.f)
        print("\ttrig_val_hi\t= {},".format(trig_val_hi), file =self.f)
        self.parse_qdss_field(bc, 'struct bc_dataset', 'enable_ganging')
        print("\toverflow_val\t= {},".format(overflow_val), file =self.f)
        print("\tmsr\t\t\t= {}".format(msr), file =self.f)
        print("}", file = self.f)

    def parse_tc_dataset(self, tc):
        capture_mode = self.ramdump.read_structure_field(bc, 'struct bc_dataset', 'capture_mode')
        capture_mode = self.ramdump.enum_lookup('enum tpdm_mode', capture_mode)
        retrieval_mode = self.ramdump.read_structure_field(bc, 'struct bc_dataset', 'retrieval_mode')
        retrieval_mode = self.ramdump.enum_lookup('enum tpdm_mode', retrieval_mode)
        trig_val_lo = self.read_array(tc + self.ramdump.field_offset('struct tc_dataset', 'trig_val_lo'), 8, 'u32')
        trig_val_hi = self.read_array(tc + self.ramdump.field_offset('struct tc_dataset', 'trig_val_hi'), 8, 'u32')
        trig_sel = self.read_array(tc + self.ramdump.field_offset('struct tc_dataset', 'trig_sel'), 8, 'u32')
        msr = self.read_array(tc + self.ramdump.field_offset('struct tc_dataset', 'msr'), 6, 'u32')
        print("struct tc_dataset {} :".format(hex(tc)), file = self.f)
        print("{", file = self.f)
        print("\tcapture_mode\t= {},".format(capture_mode), file =self.f)
        print("\tretrieval_mode\t= {},".format(retrieval_mode), file =self.f)
        self.parse_qdss_field(bc, 'struct tc_dataset', 'sat_mode', 2)
        self.parse_qdss_field(bc, 'struct tc_dataset', 'enable_counters')
        self.parse_qdss_field(bc, 'struct tc_dataset', 'clear_counters')
        self.parse_qdss_field(bc, 'struct tc_dataset', 'enable_irq', 2)
        self.parse_qdss_field(bc, 'struct tc_dataset', 'clear_irq', 2)
        print("\ttrig_sel\t\t= {},".format(trig_sel), file =self.f)
        print("\ttrig_val_lo\t\t= {},".format(trig_val_lo), file =self.f)
        print("\ttrig_val_hi\t\t= {},".format(trig_val_hi), file =self.f)
        print("\tmsr\t\t\t= {}".format(msr), file =self.f)
        print("}", file = self.f)

    def parse_dsb_dataset(self, dsb):
        mode = self.ramdump.read_structure_field(dsb, 'struct dsb_dataset', 'mode')
        edge_ctrl = self.read_array(dsb + self.ramdump.field_offset('struct dsb_dataset', 'edge_ctrl'), 16, 'u32')
        edge_ctrl_mask = self.read_array(dsb + self.ramdump.field_offset('struct dsb_dataset', 'edge_ctrl_mask'), 8, 'u32')
        patt_val = self.read_array(dsb + self.ramdump.field_offset('struct dsb_dataset', 'patt_val'), 8, 'u32')
        patt_mask = self.read_array(dsb + self.ramdump.field_offset('struct dsb_dataset', 'patt_mask'), 8, 'u32')
        trig_patt_val = self.read_array(dsb + self.ramdump.field_offset('struct dsb_dataset', 'trig_patt_val'), 8, 'u32')
        trig_patt_mask = self.read_array(dsb + self.ramdump.field_offset('struct dsb_dataset', 'trig_patt_mask'), 8, 'u32')
        select_val = self.read_array(dsb + self.ramdump.field_offset('struct dsb_dataset', 'select_val'), 8, 'u32')
        msr = self.read_array(dsb + self.ramdump.field_offset('struct dsb_dataset', 'msr'), 8, 'u32')
        print("struct dsb_dataset {} :".format(hex(dsb)), file = self.f)
        print("{", file = self.f)
        self.parse_qdss_field(dsb, 'struct dsb_dataset', 'mode', 2)
        print("\tedge_ctrl\t= {},".format(edge_ctrl), file =self.f)
        print("\tedge_ctrl_mask\t= {},".format(edge_ctrl_mask), file =self.f)
        print("\tpatt_val\t= {},".format(patt_val), file =self.f)
        print("\tpatt_mask\t= {},".format(patt_mask), file =self.f)
        self.parse_qdss_field(dsb, 'struct dsb_dataset', 'patt_ts', 2)
        self.parse_qdss_field(dsb, 'struct dsb_dataset', 'patt_type')
        print("\ttrig_patt_val\t= {},".format(trig_patt_val), file =self.f)
        print("\ttrig_patt_mask\t= {},".format(trig_patt_mask), file =self.f)
        self.parse_qdss_field(dsb, 'struct dsb_dataset', 'trig_ts', 2)
        self.parse_qdss_field(dsb, 'struct dsb_dataset', 'trig_type')
        print("\tselect_val\t= {},".format(select_val), file =self.f)
        print("\tmsr\t\t\t= {}".format(msr), file =self.f)
        print("}", file = self.f)

    def parse_cmb_dataset(self, cmb):
        patt_val = self.read_array(cmb + self.ramdump.field_offset('struct cmb_dataset', 'patt_val'), 2, 'u32')
        patt_mask = self.read_array(cmb + self.ramdump.field_offset('struct cmb_dataset', 'patt_mask'), 2, 'u32')
        trig_patt_val = self.read_array(cmb + self.ramdump.field_offset('struct cmb_dataset', 'trig_patt_val'), 2, 'u32')
        trig_patt_mask = self.read_array(cmb + self.ramdump.field_offset('struct cmb_dataset', 'trig_patt_mask'), 2, 'u32')
        msr = self.read_array(cmb + self.ramdump.field_offset('struct cmb_dataset', 'msr'), 8, 'u32')
        print("struct cmb_dataset {} :".format(hex(cmb)), file = self.f)
        print("{", file = self.f)
        self.parse_qdss_field(cmb, 'struct cmb_dataset', 'trace_mode')
        self.parse_qdss_field(cmb, 'struct cmb_dataset', 'cycle_acc')
        print("\tpatt_val\t= {},".format(patt_val), file =self.f)
        print("\tpatt_mask\t= {},".format(patt_mask), file =self.f)
        self.parse_qdss_field(cmb, 'struct cmb_dataset', 'patt_ts', 2)
        print("\ttrig_patt_val\t= {},".format(trig_patt_val), file =self.f)
        print("\ttrig_patt_mask\t= {},".format(trig_patt_mask), file =self.f)
        self.parse_qdss_field(cmb, 'struct cmb_dataset', 'trig_ts', 2)
        self.parse_qdss_field(cmb, 'struct cmb_dataset', 'ts_all', 2)
        print("\tmsr\t\t\t= {},".format(msr), file =self.f)
        self.parse_qdss_field(cmb, 'struct cmb_dataset', 'read_ctl_reg')
        self.parse_qdss_field(cmb, 'struct cmb_dataset', 'mcmb', 2, HEX=True)
        print("}", file = self.f)

    def parse_tpdm_drvdata(self, drvdata, device):
        csdev = self.ramdump.read_structure_field(drvdata, 'struct tpdm_drvdata', 'csdev')
        base = self.ramdump.read_structure_field(drvdata, 'struct tpdm_drvdata', 'base')
        if base == 0:
            base_phy = 0
        else:
            base_phy = self.ramdump.virt_to_phys(base)
        nr_tclk = self.ramdump.read_structure_field(drvdata, 'struct tpdm_drvdata', 'nr_tclk')
        nr_treg = self.ramdump.read_structure_field(drvdata, 'struct tpdm_drvdata', 'nr_treg')
        gpr = self.ramdump.read_structure_field(drvdata, 'struct tpdm_drvdata', 'gpr')
        bc = self.ramdump.read_structure_field(drvdata, 'struct tpdm_drvdata', 'bc')
        tc = self.ramdump.read_structure_field(drvdata, 'struct tpdm_drvdata', 'tc')
        dsb = self.ramdump.read_structure_field(drvdata, 'struct tpdm_drvdata', 'dsb')
        cmb = self.ramdump.read_structure_field(drvdata, 'struct tpdm_drvdata', 'cmb')
        atclk = self.ramdump.read_structure_field(drvdata, 'struct tpdm_drvdata', 'atclk')

        print("struct tpdm_drvdata {} :".format(hex(drvdata)), file = self.f)
        print("{", file = self.f)
        print("\tbase\t\t= {} (phys:{}),".format(hex(base),hex(base_phy)), file =self.f)
        print("\tcsdev\t\t= {},".format(hex(csdev)), file =self.f)
        print("\tnr_tclk\t\t= {},".format(ctypes.c_int32(nr_tclk).value), file =self.f)
        self.parse_qdss_field(drvdata, 'struct tpdm_drvdata', 'tclk', 2, HEX=True)
        print("\tnr_treg\t\t= {},".format(ctypes.c_int32(nr_treg).value), file =self.f)
        self.parse_qdss_field(drvdata, 'struct tpdm_drvdata', 'treg', 2, HEX=True)
        self.parse_qdss_field(drvdata, 'struct tpdm_drvdata', 'enable', 2)
        self.parse_qdss_field(drvdata, 'struct tpdm_drvdata', 'clk_enable')
        self.parse_qdss_field(drvdata, 'struct tpdm_drvdata', 'datasets')
        self.parse_qdss_field(drvdata, 'struct tpdm_drvdata', 'enable_ds')
        self.parse_qdss_field(drvdata, 'struct tpdm_drvdata', 'tc_trig_type')
        self.parse_qdss_field(drvdata, 'struct tpdm_drvdata', 'bc_trig_type')
        self.parse_qdss_field(drvdata, 'struct tpdm_drvdata', 'bc_gang_type')
        self.parse_qdss_field(drvdata, 'struct tpdm_drvdata', 'bc_counters_avail')
        self.parse_qdss_field(drvdata, 'struct tpdm_drvdata', 'tc_counters_avail')
        print("\tgpr\t\t\t= {},".format(hex(gpr)), file =self.f)
        print("\tbc\t\t\t= {},".format(hex(bc)), file =self.f)
        print("\ttc\t\t\t= {},".format(hex(tc)), file =self.f)
        print("\tdsb\t\t\t= {},".format(hex(dsb)), file =self.f)
        print("\tcmb\t\t\t= {},".format(hex(cmb)), file =self.f)
        self.parse_qdss_field(drvdata, 'struct tpdm_drvdata', 'traceid', 2)
        self.parse_qdss_field(drvdata, 'struct tpdm_drvdata', 'version', 2)
        self.parse_qdss_field(drvdata, 'struct tpdm_drvdata', 'msr_support')
        self.parse_qdss_field(drvdata, 'struct tpdm_drvdata', 'msr_fix_req')
        self.parse_qdss_field(drvdata, 'struct tpdm_drvdata', 'cmb_msr_skip')
        print("\tatclk\t\t= {}".format(hex(atclk)), file =self.f)
        print("}", file = self.f)
        if gpr:
            self.parse_gpr_dataset(gpr)
        if bc:
            self.parse_bc_dataset(bc)
        if tc:
            self.parse_tc_dataset(tc)
        if dsb:
            self.parse_dsb_dataset(dsb)
        if cmb:
            self.parse_cmb_dataset(cmb)
        self.parse_csdev(csdev)
        self.parse_clk(atclk)

    def parse_tpdm_component(self, drvdata, device):
        clk = self.ramdump.read_structure_field(device - self.amba_dev_offset, 'struct amba_device', 'pclk')
        self.parse_tpdm_drvdata(drvdata, device)
        self.parse_clk(clk)

    def parse_static_tpdm_component(self, drvdata, device):
        self.parse_tpdm_drvdata(drvdata, device)

    def read_array(self, addr, count, datatype):
        array = [0] *count
        if (datatype == 'u8'):
            for i in range(count):
                array[i] = self.ramdump.read_byte(addr)
                addr += 1
        if (datatype == 'u32'):
            for i in range(count):
                array[i] = self.ramdump.read_u32(addr)
                addr += 4
        elif (datatype == 'u64'):
            for i in range(count):
                array[i] = hex(self.ramdump.read_u64(addr))
                addr += 8
        return array

    def parse_tpda_component(self, drvdata, device):
        clk = self.ramdump.read_structure_field(device - self.amba_dev_offset, 'struct amba_device', 'pclk')
        csdev = self.ramdump.read_structure_field(drvdata, 'struct tpda_drvdata', 'csdev')
        base = self.ramdump.read_structure_field(drvdata, 'struct tpda_drvdata', 'base')
        base_phy = self.ramdump.virt_to_phys(base)
        bc_esize = self.read_array(drvdata + self.ramdump.field_offset('struct tpda_drvdata', 'bc_esize'), 32, 'u32')
        tc_esize = self.read_array(drvdata + self.ramdump.field_offset('struct tpda_drvdata', 'tc_esize'), 32, 'u32')
        dsb_esize = self.read_array(drvdata + self.ramdump.field_offset('struct tpda_drvdata', 'dsb_esize'), 32, 'u32')
        cmb_esize = self.read_array(drvdata + self.ramdump.field_offset('struct tpda_drvdata', 'cmb_esize'), 32, 'u32')
        atclk = self.ramdump.read_structure_field(drvdata, 'struct tpda_drvdata', 'atclk')
        print("struct tpda_drvdata {} :".format(hex(drvdata)), file = self.f)
        print("{", file = self.f)
        print("\tbase\t\t= {} (phys:{}),".format(hex(base),hex(base_phy)), file =self.f)
        print("\tcsdev\t\t= {},".format(hex(csdev)), file =self.f)
        self.parse_qdss_field(drvdata, 'struct tpda_drvdata', 'enable', 2)
        self.parse_qdss_field(drvdata, 'struct tpda_drvdata', 'atid', 2)
        print("\tbc_esize\t= {},".format(bc_esize), file =self.f)
        print("\ttc_esize\t= {},".format(tc_esize), file =self.f)
        print("\tdsb_esize\t= {},".format(dsb_esize), file =self.f)
        print("\tcmb_esize\t= {},".format(cmb_esize), file =self.f)
        self.parse_qdss_field(drvdata, 'struct tpda_drvdata', 'trig_async')
        self.parse_qdss_field(drvdata, 'struct tpda_drvdata', 'trig_flag_ts')
        self.parse_qdss_field(drvdata, 'struct tpda_drvdata', 'trig_freq')
        self.parse_qdss_field(drvdata, 'struct tpda_drvdata', 'freq_ts', 2)
        self.parse_qdss_field(drvdata, 'struct tpda_drvdata', 'freq_req_val')
        self.parse_qdss_field(drvdata, 'struct tpda_drvdata', 'freq_req')
        self.parse_qdss_field(drvdata, 'struct tpda_drvdata', 'cmbchan_mode')
        print("\tatclk\t\t= {}".format(hex(atclk)), file =self.f)
        print("}", file = self.f)
        self.parse_clk(clk)
        self.parse_clk(atclk)
        self.parse_csdev(csdev)

    def parse_stm_component(self, drvdata, device):
        clk = self.ramdump.read_structure_field(device - self.amba_dev_offset, 'struct amba_device', 'pclk')
        csdev = self.ramdump.read_structure_field(drvdata, 'struct stm_drvdata', 'csdev')
        base = self.ramdump.read_structure_field(drvdata, 'struct stm_drvdata', 'base')
        base_phy = self.ramdump.virt_to_phys(base)
        atclk = self.ramdump.read_structure_field(drvdata, 'struct stm_drvdata', 'atclk')
        chs = drvdata + self.ramdump.field_offset('struct stm_drvdata', 'chs')
        stm = drvdata + self.ramdump.field_offset('struct stm_drvdata', 'stm')

        print("struct stm_drvdata {} :".format(hex(drvdata)), file = self.f)
        print("{", file = self.f)
        print("\tbase\t\t= {} (phys:{}),".format(hex(base),hex(base_phy)), file =self.f)
        print("\tatclk\t\t= {},".format(hex(atclk)), file =self.f)
        print("\tcsdev\t\t= {},".format(hex(csdev)), file =self.f)
        print("\tchs\t\t\t= {", file = self.f)
        self.parse_qdss_field(chs, 'struct channel_space', 'base', 2, HEX=True, indent=2)
        self.parse_qdss_field(chs, 'struct channel_space', 'phys', 2, HEX=True, indent=2)
        self.parse_qdss_field(chs, 'struct channel_space', 'guaranteed', HEX=True, indent=2)
        print("\t},", file = self.f)
        print("\tstm\t\t\t= {", file = self.f)
        self.parse_qdss_field(stm, 'struct stm_data', 'name', 2, str=True, indent=2)
        self.parse_qdss_field(stm, 'struct stm_data', 'stm', 3, HEX=True, indent=2)
        self.parse_qdss_field(stm, 'struct stm_data', 'sw_start', indent=2)
        self.parse_qdss_field(stm, 'struct stm_data', 'sw_end', 2, indent=2)
        self.parse_qdss_field(stm, 'struct stm_data', 'sw_nchannels', indent=2)
        self.parse_qdss_field(stm, 'struct stm_data', 'sw_mmiosz', indent=2)
        self.parse_qdss_field(stm, 'struct stm_data', 'hw_override', indent=2)
        print("\t},", file = self.f)
        self.parse_qdss_field(drvdata, 'struct stm_drvdata', 'traceid', 2)
        self.parse_qdss_field(drvdata, 'struct stm_drvdata', 'write_bytes')
        self.parse_qdss_field(drvdata, 'struct stm_drvdata', 'stmsper', 2)
        self.parse_qdss_field(drvdata, 'struct stm_drvdata', 'stmspscr')
        self.parse_qdss_field(drvdata, 'struct stm_drvdata', 'numsp', 2)
        self.parse_qdss_field(drvdata, 'struct stm_drvdata', 'stmheer', 2)
        self.parse_qdss_field(drvdata, 'struct stm_drvdata', 'stmheter')
        self.parse_qdss_field(drvdata, 'struct stm_drvdata', 'stmhebsr')
        self.parse_qdss_field(drvdata, 'struct stm_drvdata', 'static_atid')
        print("}", file = self.f)
        self.parse_clk(clk)
        self.parse_clk(atclk)
        self.parse_csdev(csdev)

    def parse_uetm_component(self, drvdata, device):
        csdev = self.ramdump.read_structure_field(drvdata, 'struct uetm_drvdata', 'csdev')
        base = self.ramdump.read_structure_field(drvdata, 'struct uetm_drvdata', 'base')
        base_phy = self.ramdump.virt_to_phys(base)
        print("struct uetm_drvdata {} :".format(hex(drvdata)), file = self.f)
        print("{", file = self.f)
        print("\tbase\t\t= {} (phys:{}),".format(hex(base),hex(base_phy)), file =self.f)
        print("\tcsdev\t\t= {},".format(hex(csdev)), file =self.f)
        self.parse_qdss_field(drvdata, 'struct uetm_drvdata', 'lane', 2)
        self.parse_qdss_field(drvdata, 'struct uetm_drvdata', 'base_address', HEX=True)
        self.parse_qdss_field(drvdata, 'struct uetm_drvdata', 'uetm_id', 2)
        self.parse_qdss_field(drvdata, 'struct uetm_drvdata', 'traceid', 2)
        self.parse_qdss_field(drvdata, 'struct uetm_drvdata', 'size', 2)
        self.parse_qdss_field(drvdata, 'struct uetm_drvdata', 'core_id', 2)
        self.parse_qdss_field(drvdata, 'struct uetm_drvdata', 'cluster_id')
        self.parse_qdss_field(drvdata, 'struct uetm_drvdata', 'state_idx')
        self.parse_qdss_field(drvdata, 'struct uetm_drvdata', 'lane_idx')
        self.parse_qdss_field(drvdata, 'struct uetm_drvdata', 'enable', 2)
        self.parse_qdss_field(drvdata, 'struct uetm_drvdata', 'uncore_uetm')
        self.parse_qdss_field(drvdata, 'struct uetm_drvdata', 'config', 2, HEX=True)
        print("}", file = self.f)
        self.parse_csdev(csdev)

    def parse_qmi_component(self, drvdata, device):
        csdev = self.ramdump.read_structure_field(drvdata, 'struct qmi_drvdata', 'csdev')
        s_addr = drvdata + self.ramdump.field_offset('struct qmi_drvdata', 's_addr')
        print("struct qmi_drvdata {} :".format(hex(drvdata)), file = self.f)
        print("{", file = self.f)
        print("\tcsdev\t\t= {},".format(hex(csdev)), file =self.f)
        self.parse_qdss_field(drvdata, 'struct qmi_drvdata', 'inst_id', 2, HEX=True)
        self.parse_qdss_field(drvdata, 'struct qmi_drvdata', 'service_connected')
        self.parse_qdss_field(drvdata, 'struct qmi_drvdata', 'security')
        print("\ts_addr\t\t= {", file =self.f)
        self.parse_qdss_field(s_addr, 'struct sockaddr_qrtr', 'sq_family', indent=2)
        self.parse_qdss_field(s_addr, 'struct sockaddr_qrtr', 'sq_node', 2, indent=2)
        self.parse_qdss_field(s_addr, 'struct sockaddr_qrtr', 'sq_port', 2, indent=2)
        print("\t}", file = self.f)
        print("}", file = self.f)
        self.parse_csdev(csdev)

    def parse_trace_noc_component(self, drvdata, device):
        clk = self.ramdump.read_structure_field(device - self.amba_dev_offset, 'struct amba_device', 'pclk')
        csdev = self.ramdump.read_structure_field(drvdata, 'struct trace_noc_drvdata', 'csdev')
        base = self.ramdump.read_structure_field(drvdata, 'struct trace_noc_drvdata', 'base')
        base_phy = self.ramdump.virt_to_phys(base)
        print("struct trace_noc_drvdata {} :".format(hex(drvdata)), file = self.f)
        print("{", file = self.f)
        print("\tbase\t\t= {} (phys:{}),".format(hex(base),hex(base_phy)), file =self.f)
        print("\tcsdev\t\t= {},".format(hex(csdev)), file =self.f)
        self.parse_qdss_field(drvdata, 'struct trace_noc_drvdata', 'version', 2)
        self.parse_qdss_field(drvdata, 'struct trace_noc_drvdata', 'enable', 2)
        self.parse_qdss_field(drvdata, 'struct trace_noc_drvdata', 'flushReq')
        self.parse_qdss_field(drvdata, 'struct trace_noc_drvdata', 'freqTsReq')
        self.parse_qdss_field(drvdata, 'struct trace_noc_drvdata', 'atid', 2)
        self.parse_qdss_field(drvdata, 'struct trace_noc_drvdata', 'freq_req_val')
        self.parse_qdss_field(drvdata, 'struct trace_noc_drvdata', 'flushStatus')
        self.parse_qdss_field(drvdata, 'struct trace_noc_drvdata', 'freqType')
        self.parse_qdss_field(drvdata, 'struct trace_noc_drvdata', 'flagType')
        print("}", file = self.f)
        self.parse_clk(clk)
        self.parse_csdev(csdev)

    def parse_replicator_drvdata(self, drvdata, device):
        csdev = self.ramdump.read_structure_field(drvdata, 'struct replicator_drvdata', 'csdev')
        base = self.ramdump.read_structure_field(drvdata, 'struct replicator_drvdata', 'base')
        if base == 0:
            base_phy = 0
        else:
            base_phy = self.ramdump.virt_to_phys(base)
        atclk = self.ramdump.read_structure_field(drvdata, 'struct replicator_drvdata', 'atclk')
        print("struct replicator_drvdata {} :".format(hex(drvdata)), file = self.f)
        print("{", file = self.f)
        print("\tbase\t\t= {} (phys:{}),".format(hex(base),hex(base_phy)), file =self.f)
        print("\tatclk\t\t= {},".format(hex(atclk)), file =self.f)
        print("\tcsdev\t\t= {},".format(hex(csdev)), file =self.f)
        self.parse_qdss_field(drvdata, 'struct replicator_drvdata', 'check_idfilter_val')
        self.parse_qdss_field(drvdata, 'struct replicator_drvdata', 'delayed', 2)
        print("}", file = self.f)
        self.parse_clk(atclk)
        self.parse_csdev(csdev)

    def parse_static_replicator_component(self, drvdata, device):
        self.parse_replicator_drvdata(drvdata, device)

    def parse_dynamic_replicator_component(self, drvdata, device):
        clk = self.ramdump.read_structure_field(device - self.amba_dev_offset, 'struct amba_device', 'pclk')
        self.parse_replicator_drvdata(drvdata, device)
        self.parse_clk(clk)

    def parse_csr_component(self, drvdata, device):
        csdev = self.ramdump.read_structure_field(drvdata, 'struct csr_drvdata', 'csdev')
        clk = self.ramdump.read_structure_field(drvdata, 'struct csr_drvdata', 'clk')

        print("struct csr_drvdata {} :".format(hex(drvdata)), file = self.f)
        print("{", file = self.f)
        self.parse_qdss_field(drvdata, 'struct csr_drvdata', 'base', 2, HEX=True)
        self.parse_qdss_field(drvdata, 'struct csr_drvdata', 'pbase', 2, HEX=True)
        self.parse_qdss_field(drvdata, 'struct csr_drvdata', 'msr_start', HEX=True)
        self.parse_qdss_field(drvdata, 'struct csr_drvdata', 'msr_end', 2, HEX=True)
        print("\tcsdev\t\t= {},".format(hex(csdev)), file =self.f)
        self.parse_qdss_field(drvdata, 'struct csr_drvdata', 'msr', 3, HEX=True)
        self.parse_qdss_field(drvdata, 'struct csr_drvdata', 'msr_refcnt', HEX=True)
        self.parse_qdss_field(drvdata, 'struct csr_drvdata', 'blksize', 2)
        self.parse_qdss_field(drvdata, 'struct csr_drvdata', 'flushperiod')
        self.parse_qdss_field(drvdata, 'struct csr_drvdata', 'hbeat_val0')
        self.parse_qdss_field(drvdata, 'struct csr_drvdata', 'hbeat_val1')
        self.parse_qdss_field(drvdata, 'struct csr_drvdata', 'hbeat_mask0')
        self.parse_qdss_field(drvdata, 'struct csr_drvdata', 'hbeat_mask1')
        print("\tclk\t\t= {},".format(hex(clk)), file =self.f)
        self.parse_qdss_field(drvdata, 'struct csr_drvdata', 'usb_bam_support')
        self.parse_qdss_field(drvdata, 'struct csr_drvdata', 'perflsheot_set_support')
        self.parse_qdss_field(drvdata, 'struct csr_drvdata', 'hwctrl_set_support')
        self.parse_qdss_field(drvdata, 'struct csr_drvdata', 'set_byte_cntr_support')
        self.parse_qdss_field(drvdata, 'struct csr_drvdata', 'timestamp_support')
        self.parse_qdss_field(drvdata, 'struct csr_drvdata', 'enable_flush')
        self.parse_qdss_field(drvdata, 'struct csr_drvdata', 'msr_support')
        self.parse_qdss_field(drvdata, 'struct csr_drvdata', 'aodbg_csr_support')
        self.parse_qdss_field(drvdata, 'struct csr_drvdata', 'atid_offset')
        print("}", file = self.f)
        self.parse_clk(clk)
        self.parse_csdev(csdev)

    def parse_tgu_component(self, drvdata, device):
        pclk = self.ramdump.read_structure_field(device - self.amba_dev_offset, 'struct amba_device', 'pclk')
        csdev = self.ramdump.read_structure_field(drvdata, 'struct tgu_drvdata', 'csdev')
        base = self.ramdump.read_structure_field(drvdata, 'struct tgu_drvdata', 'base')
        base_phy = self.ramdump.virt_to_phys(base)
        clk = self.ramdump.read_structure_field(drvdata, 'struct tgu_drvdata', 'clk')
        max_steps = self.ramdump.read_structure_field(drvdata, 'struct tgu_drvdata', 'max_steps')
        max_conditions = self.ramdump.read_structure_field(drvdata, 'struct tgu_drvdata', 'max_conditions')
        max_regs = self.ramdump.read_structure_field(drvdata, 'struct tgu_drvdata', 'max_regs')
        max_timer_counter = self.ramdump.read_structure_field(drvdata, 'struct tgu_drvdata', 'max_timer_counter')
        grp_data = self.ramdump.read_structure_field(drvdata, 'struct tgu_drvdata', 'grp_data')
        condition_data = self.ramdump.read_structure_field(drvdata, 'struct tgu_drvdata', 'condition_data')
        select_data = self.ramdump.read_structure_field(drvdata, 'struct tgu_drvdata', 'select_data')
        timer_data = self.ramdump.read_structure_field(drvdata, 'struct tgu_drvdata', 'timer_data')
        counter_data = self.ramdump.read_structure_field(drvdata, 'struct tgu_drvdata', 'counter_data')

        print("struct tgu_drvdata {} :".format(hex(drvdata)), file = self.f)
        print("{", file = self.f)
        print("\tbase\t\t= {} (phys:{}),".format(hex(base),hex(base_phy)), file =self.f)
        print("\tcsdev\t\t= {},".format(hex(csdev)), file =self.f)
        print("\tclk\t\t\t= {},".format(hex(clk)), file =self.f)
        self.parse_qdss_field(drvdata, 'struct tgu_drvdata', 'max_steps')
        self.parse_qdss_field(drvdata, 'struct tgu_drvdata', 'max_conditions')
        self.parse_qdss_field(drvdata, 'struct tgu_drvdata', 'max_regs')
        self.parse_qdss_field(drvdata, 'struct tgu_drvdata', 'max_timer_counter')
        print("\tgrp_data\t= {} :".format(hex(grp_data)), file =self.f)
        print("\t{", file = self.f)
        self.parse_qdss_field(grp_data, 'struct Trigger_group_data', 'grpaddr', 2, HEX=True, indent=2)
        self.parse_qdss_field(grp_data, 'struct Trigger_group_data', 'value', 2, indent=2)
        print("\t},", file = self.f)
        print("\tcondition_data\t= {} :".format(hex(condition_data)), file =self.f)
        print("\t{", file = self.f)
        self.parse_qdss_field(condition_data, 'struct Trigger_condition_data', 'condaddr', HEX=True, indent=2)
        self.parse_qdss_field(condition_data, 'struct Trigger_condition_data', 'value', 2, indent=2)
        print("\t},", file = self.f)
        print("\tselect_data\t= {} :".format(hex(select_data)), file =self.f)
        print("\t{", file = self.f)
        self.parse_qdss_field(select_data, 'struct Trigger_select_data', 'selectaddr', HEX=True, indent=2)
        self.parse_qdss_field(select_data, 'struct Trigger_select_data', 'value', 2, indent=2)
        print("\t},", file = self.f)
        print("\ttimer_data\t= {} :".format(hex(timer_data)), file =self.f)
        print("\t{", file = self.f)
        self.parse_qdss_field(timer_data, 'struct Trigger_timer_data', 'timeraddr', HEX=True, indent=2)
        self.parse_qdss_field(timer_data, 'struct Trigger_timer_data', 'value', 2, indent=2)
        print("\t},", file = self.f)
        print("\tcounter_data\t= {} :".format(hex(counter_data)), file =self.f)
        print("\t{", file = self.f)
        self.parse_qdss_field(counter_data, 'struct Trigger_counter_data', 'counteraddr', HEX=True, indent=2)
        self.parse_qdss_field(counter_data, 'struct Trigger_counter_data', 'value', 2, indent=2)
        print("\t},", file = self.f)
        self.parse_qdss_field(drvdata, 'struct tgu_drvdata', 'grp_refcnt')
        self.parse_qdss_field(drvdata, 'struct tgu_drvdata', 'cond_refcnt')
        self.parse_qdss_field(drvdata, 'struct tgu_drvdata', 'select_refcnt')
        self.parse_qdss_field(drvdata, 'struct tgu_drvdata', 'timer_refcnt')
        self.parse_qdss_field(drvdata, 'struct tgu_drvdata', 'counter_refcnt')
        self.parse_qdss_field(drvdata, 'struct tgu_drvdata', 'enable', 2)
        print("}", file = self.f)
        self.parse_clk(pclk)
        self.parse_csdev(csdev)

    def parse_cti_component(self, drvdata, device):
        clk = self.ramdump.read_structure_field(device - self.amba_dev_offset, 'struct amba_device', 'pclk')
        csdev = self.ramdump.read_structure_field(drvdata, 'struct cti_drvdata', 'csdev')
        base = self.ramdump.read_structure_field(drvdata, 'struct cti_drvdata', 'base')
        base_phy = self.ramdump.virt_to_phys(base)
        ctidev = drvdata + self.ramdump.field_offset('struct cti_drvdata', 'ctidev')
        config = drvdata + self.ramdump.field_offset('struct cti_drvdata', 'config')
        trig_in_use = self.read_array(config + self.ramdump.field_offset('struct cti_config','trig_in_use'), 2, 'u64')
        trig_out_use = self.read_array(config + self.ramdump.field_offset('struct cti_config','trig_out_use'), 2, 'u64')
        trig_out_filter = self.read_array(config + self.ramdump.field_offset('struct cti_config','trig_out_filter'), 2, 'u64')
        ctiinen = self.read_array(config + self.ramdump.field_offset('struct cti_config','ctiinen'), 128, 'u32')
        ctiouten = self.read_array(config + self.ramdump.field_offset('struct cti_config','ctiouten'), 128, 'u32')
        atclk = self.ramdump.read_structure_field(drvdata, 'struct cti_drvdata', 'atclk')

        print("struct cti_drvdata {} :".format(hex(drvdata)), file = self.f)
        print("{", file = self.f)
        print("\tbase\t\t= {} (phys:{}),".format(hex(base),hex(base_phy)), file =self.f)
        print("\tcsdev\t\t= {},".format(hex(csdev)), file =self.f)
        self.parse_qdss_field(drvdata, 'struct cti_drvdata', 'extended_cti')
        print("\tctidev\t\t= {", file =self.f)
        self.parse_qdss_field(ctidev, 'struct cti_device', 'nr_trig_con', indent=2)
        self.parse_qdss_field(ctidev, 'struct cti_device', 'ctm_id', indent=2)
        self.parse_qdss_field(ctidev, 'struct cti_device', 'cpu', 2, HEX=True, indent=2)
        print("\t}", file = self.f)
        print("\tconfig\t\t= {", file =self.f)
        self.parse_qdss_field(config, 'struct cti_config', 'nr_ctm_channels', indent=2)
        self.parse_qdss_field(config, 'struct cti_config', 'nr_trig_max', indent=2)
        self.parse_qdss_field(config, 'struct cti_config', 'enable_req_count', indent=2)
        self.parse_qdss_field(config, 'struct cti_config', 'hw_enabled', indent=2)
        self.parse_qdss_field(config, 'struct cti_config', 'hw_powered', indent=2)
        self.parse_qdss_field(config, 'struct cti_config', 'hw_enabled_store', indent=2)
        print("\t\ttrig_in_use\t\t= {},".format(trig_in_use), file =self.f)
        print("\t\ttrig_out_use\t= {},".format(trig_out_use), file =self.f)
        print("\t\ttrig_out_filter\t= {},".format(trig_out_filter), file =self.f)
        self.parse_qdss_field(config, 'struct cti_config', 'trig_filter_enable', indent=2)
        self.parse_qdss_field(config, 'struct cti_config', 'xtrig_rchan_sel', indent=2)
        self.parse_qdss_field(config, 'struct cti_config', 'ctiappset', indent=2)
        self.parse_qdss_field(config, 'struct cti_config', 'ctiinout_sel', indent=2)
        print("\t\tctiinen\t\t= {},".format(ctiinen), file =self.f)
        print("\t\tctiouten\t= {},".format(ctiouten), file =self.f)
        self.parse_qdss_field(config, 'struct cti_config', 'ctigate', indent=2)
        self.parse_qdss_field(config, 'struct cti_config', 'asicctl', indent=2)
        print("\t}", file = self.f)
        self.parse_qdss_field(drvdata, 'struct cti_drvdata', 'gpio_trigin', HEX=True)
        self.parse_qdss_field(drvdata, 'struct cti_drvdata', 'gpio_trigout', HEX=True)
        print("\tatclk\t\t= {}".format(hex(atclk)), file =self.f)
        print("}", file = self.f)
        self.parse_clk(clk)
        self.parse_clk(atclk)
        self.parse_csdev(csdev)

    def parse_secure_etr_component(self, drvdata, device):
        csdev = self.ramdump.read_structure_field(drvdata, 'struct secure_etr_drvdata', 'csdev')
        clk = self.ramdump.read_structure_field(drvdata, 'struct secure_etr_drvdata', 'clk')

        print("struct secure_etr_drvdata {} :".format(hex(drvdata)), file = self.f)
        print("{", file = self.f)
        self.parse_qdss_field(drvdata, 'struct secure_etr_drvdata', 'dev', 3, HEX=True)
        print("\tcsdev\t\t= {},".format(hex(csdev)), file =self.f)
        self.parse_qdss_field(drvdata, 'struct secure_etr_drvdata', 'sram_node', str=True)
        self.parse_qdss_field(drvdata, 'struct secure_etr_drvdata', 'sram_class', HEX=True)
        self.parse_qdss_field(drvdata, 'struct secure_etr_drvdata', 'real_name', str=True)
        self.parse_qdss_field(drvdata, 'struct secure_etr_drvdata', 'real_sink', HEX=True)
        self.parse_qdss_field(drvdata, 'struct secure_etr_drvdata', 'etm_inst_id')
        self.parse_qdss_field(drvdata, 'struct secure_etr_drvdata', 'reading', 2)
        self.parse_qdss_field(drvdata, 'struct secure_etr_drvdata', 'mode', 2)
        self.parse_qdss_field(drvdata, 'struct secure_etr_drvdata', 'etr_buf', 2, HEX=True)
        self.parse_qdss_field(drvdata, 'struct secure_etr_drvdata', 'csr', 3, HEX=True)
        self.parse_qdss_field(drvdata, 'struct secure_etr_drvdata', 'csr_name', str=True)
        self.parse_qdss_field(drvdata, 'struct secure_etr_drvdata', 'atid_offset')
        self.parse_qdss_field(drvdata, 'struct secure_etr_drvdata', 'mem_size')
        print("\tclk\t\t\t= {}".format(hex(clk)), file =self.f)
        print("}", file = self.f)
        self.parse_clk(clk)
        self.parse_csdev(csdev)

    def parse_funnel_drvdata(self, drvdata, device):
        csdev = self.ramdump.read_structure_field(drvdata, 'struct funnel_drvdata', 'csdev')
        base = self.ramdump.read_structure_field(drvdata, 'struct funnel_drvdata', 'base')
        if base == 0:
            base_phy = 0
        else:
            base_phy = self.ramdump.virt_to_phys(base)
        atclk = self.ramdump.read_structure_field(drvdata, 'struct funnel_drvdata', 'atclk')

        print("struct funnel_drvdata {} :".format(hex(drvdata)), file = self.f)
        print("{", file = self.f)
        print("\tbase\t\t= {} (phys:{}),".format(hex(base),hex(base_phy)), file =self.f)
        print("\tatclk\t\t= {},".format(hex(atclk)), file =self.f)
        print("\tcsdev\t\t= {},".format(hex(csdev)), file =self.f)
        self.parse_qdss_field(drvdata, 'struct funnel_drvdata', 'priority')
        print("}", file = self.f)
        self.parse_clk(atclk)
        self.parse_csdev(csdev)

    def parse_static_funnel_component(self, drvdata, device):
        self.parse_funnel_drvdata(drvdata, device)

    def parse_dynamic_funnel_component(self, drvdata, device):
        clk = self.ramdump.read_structure_field(device - self.amba_dev_offset, 'struct amba_device', 'pclk')
        self.parse_funnel_drvdata(drvdata, device)
        self.parse_clk(clk)

    def parse_dummy_component(self, drvdata, device):
        csdev = self.ramdump.read_structure_field(drvdata, 'struct dummy_drvdata', 'csdev')

        print("struct dummy_drvdata {} :".format(hex(drvdata)), file = self.f)
        print("{", file = self.f)
        self.parse_qdss_field(drvdata, 'struct dummy_drvdata', 'dev', 3, HEX=True)
        print("\tcsdev\t\t= {},".format(hex(csdev)), file =self.f)
        self.parse_qdss_field(drvdata, 'struct dummy_drvdata', 'traceid', 2)
        self.parse_qdss_field(drvdata, 'struct dummy_drvdata', 'static_atid')
        print("}", file = self.f)
        self.parse_csdev(csdev)

    def parse_remote_etm_component(self, drvdata, device):
        csdev = self.ramdump.read_structure_field(drvdata, 'struct remote_etm_drvdata', 'csdev')
        traceids = self.ramdump.read_structure_field(drvdata, 'struct remote_etm_drvdata', 'traceids')
        num_trcid = self.ramdump.read_structure_field(drvdata, 'struct remote_etm_drvdata', 'num_trcid')
        traceid = self.read_array(traceids, num_trcid, 'u8')

        print("struct remote_etm_drvdata {} :".format(hex(drvdata)), file = self.f)
        print("{", file = self.f)
        self.parse_qdss_field(drvdata, 'struct remote_etm_drvdata', 'dev', 3, HEX=True)
        print("\tcsdev\t\t= {},".format(hex(csdev)), file =self.f)
        self.parse_qdss_field(drvdata, 'struct remote_etm_drvdata', 'enable', 2)
        print("\ttraceids\t= {} : {},".format(hex(traceids), traceid), file =self.f)
        print("\tnum_trcid\t= {},".format(num_trcid), file =self.f)
        self.parse_qdss_field(drvdata, 'struct remote_etm_drvdata', 'static_atid')
        print("}", file = self.f)
        self.parse_csdev(csdev)

    def parse_tmc_component(self, drvdata, device):
        clk = self.ramdump.read_structure_field(device - self.amba_dev_offset, 'struct amba_device', 'pclk')
        base = self.ramdump.read_structure_field(drvdata, 'struct tmc_drvdata', 'base')
        base_phy = self.ramdump.virt_to_phys(base)
        csdev = self.ramdump.read_structure_field(drvdata, 'struct tmc_drvdata', 'csdev')
        config_type = self.ramdump.read_structure_field(drvdata, 'struct tmc_drvdata', 'config_type')
        config_type = self.ramdump.enum_lookup('enum tmc_config_type', config_type)
        memwidth = self.ramdump.read_structure_field(drvdata, 'struct tmc_drvdata', 'memwidth')
        memwidth = self.ramdump.enum_lookup('enum tmc_mem_intf_width', memwidth)
        out_mode = self.ramdump.read_structure_field(drvdata, 'struct tmc_drvdata', 'out_mode')
        out_mode = self.ramdump.enum_lookup('enum tmc_etr_out_mode', out_mode)
        atclk = self.ramdump.read_structure_field(drvdata, 'struct tmc_drvdata', 'atclk')

        print("struct tmc_drvdata {} :".format(hex(drvdata)), file = self.f)
        print("{", file = self.f)
        print("\tbase\t\t= {} (phys:{}),".format(hex(base),hex(base_phy)), file =self.f)
        print("\tcsdev\t\t= {},".format(hex(csdev)), file =self.f)
        self.parse_qdss_field(drvdata, 'struct tmc_drvdata', 'pid', 3, HEX=True)
        self.parse_qdss_field(drvdata, 'struct tmc_drvdata', 'reading', 2)
        self.parse_qdss_field(drvdata, 'struct tmc_drvdata', 'busy', 2)
        self.parse_qdss_field(drvdata, 'struct tmc_drvdata', 'buf', 3, HEX=True)
        self.parse_qdss_field(drvdata, 'struct tmc_drvdata', 'len', 3)
        self.parse_qdss_field(drvdata, 'struct tmc_drvdata', 'size', 2)
        self.parse_qdss_field(drvdata, 'struct tmc_drvdata', 'max_burst_size')
        self.parse_qdss_field(drvdata, 'struct tmc_drvdata', 'mode', 2)
        print("\tconfig_type\t= {},".format(config_type), file =self.f)
        print("\tmemwidth\t= {},".format(memwidth), file =self.f)
        self.parse_qdss_field(drvdata, 'struct tmc_drvdata', 'trigger_cntr')
        self.parse_qdss_field(drvdata, 'struct tmc_drvdata', 'etr_caps')
        self.parse_qdss_field(drvdata, 'struct tmc_drvdata', 'sysfs_buf', HEX=True)
        self.parse_qdss_field(drvdata, 'struct tmc_drvdata', 'perf_buf', HEX=True)
        self.parse_qdss_field(drvdata, 'struct tmc_drvdata', 'byte_cntr', HEX=True)
        self.parse_qdss_field(drvdata, 'struct tmc_drvdata', 'csr', 3, HEX=True)
        self.parse_qdss_field(drvdata, 'struct tmc_drvdata', 'csr_name', str=True)
        self.parse_qdss_field(drvdata, 'struct tmc_drvdata', 'atid_offset')
        print("\tout_mode\t= {},".format(out_mode), file =self.f)
        self.parse_qdss_field(drvdata, 'struct tmc_drvdata', 'usb_data', HEX=True)
        self.parse_qdss_field(drvdata, 'struct tmc_drvdata', 'stop_on_flush')
        self.parse_qdss_field(drvdata, 'struct tmc_drvdata', 'delayed', 2, HEX=True)
        print("\tatclk\t\t= {}".format(hex(atclk)), file =self.f)
        print("}", file = self.f)
        self.parse_clk(atclk)
        self.parse_clk(clk)
        self.parse_csdev(csdev)

    def parse_etm4_platform_component(self, drvdata, device):
        pclk = self.ramdump.read_structure_field(drvdata, 'struct etmv4_drvdata', 'pclk')
        base = self.ramdump.read_structure_field(drvdata, 'struct etmv4_drvdata', 'base')
        csdev = self.ramdump.read_structure_field(drvdata, 'struct etmv4_drvdata', 'csdev')
        config = drvdata + self.ramdump.field_offset('struct etmv4_drvdata', 'config')
        seq_ctrl = self.read_array(config + self.ramdump.field_offset('struct etmv4_config', 'seq_ctrl'), 4 , 'u32')
        cntrldvr = self.read_array(config + self.ramdump.field_offset( 'struct etmv4_config', 'cntrldvr'), 4, 'u32')
        cntr_ctrl = self.read_array(config + self.ramdump.field_offset('struct etmv4_config', 'cntr_ctrl'), 4, 'u32')
        cntr_val = self.read_array(config + self.ramdump.field_offset('struct etmv4_config', 'cntr_val'), 4, 'u32')
        res_ctrl = self.read_array(config + self.ramdump.field_offset('struct etmv4_config', 'res_ctrl'), 32, 'u32')
        ss_ctrl = self.read_array(config + self.ramdump.field_offset('struct etmv4_config', 'ss_ctrl'), 8, 'u32')
        ss_status = self.read_array(config + self.ramdump.field_offset('struct etmv4_config', 'ss_status'), 8, 'u32')
        ss_pe_cmp = self.read_array(config + self.ramdump.field_offset('struct etmv4_config', 'ss_pe_cmp'), 8, 'u32')
        addr_val = self.read_array(config + self.ramdump.field_offset('struct etmv4_config', 'addr_val'), 16, 'u64')
        addr_acc = self.read_array(config + self.ramdump.field_offset('struct etmv4_config', 'addr_acc'), 16, 'u64')
        addr_type = self.read_array(config + self.ramdump.field_offset('struct etmv4_config', 'addr_type'), 16, 'u8')
        ctxid_pid = self.read_array(config + self.ramdump.field_offset('struct etmv4_config', 'ctxid_pid'), 8, 'u64')
        vmid_val = self.read_array(config + self.ramdump.field_offset('struct etmv4_config', 'vmid_val'), 8, 'u64')

        print("struct etmv4_drvdata {} :".format(hex(drvdata)), file = self.f)
        print("{", file = self.f)
        print("\tpclk\t\t= {},".format(hex(pclk)), file =self.f)
        print("\tbase\t\t= {},".format(hex(base)), file =self.f)
        print("\tcsdev\t\t= {},".format(hex(csdev)), file =self.f)
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'mode', 2)
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'cpu', 3)
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'arch', 2)
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'nr_pe', 2)
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'nr_pe_cmp')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'nr_addr_cmp')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'nr_cntr', 2)
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'ext_inp_sel')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'nr_ext_inp')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'numcidc', 2)
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'numvmidc')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'nrseqstate')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'nr_event')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'nr_resource')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'nr_ss_cmp')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'trcid', 2)
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'trcid_size')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'ts_size', 2)
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'ctxid_size')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'vmid_size')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'ccsize', 2)
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'ccitmin', 2)
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 's_ex_level')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'ns_ex_level')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'q_support')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'os_lock_model')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'sticky_enable')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'boot_enable')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'os_unlock')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'instrp0', 2)
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'q_filt', 2)
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'trcbb', 2)
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'trccond', 2)
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'retstack')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'trccci', 2)
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'trc_error')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'syncpr', 2)
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'stallctl')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'sysstall')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'nooverflow')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'atbtrig', 2)
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'lpoverride')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'trfcr', 2)
        print("\tconfig\t= {", file = self.f)
        self.parse_qdss_field(config, 'struct etmv4_config', 'mode', 2, HEX=True, indent=2)
        self.parse_qdss_field(config, 'struct etmv4_config', 'pe_sel', 2, indent=2)
        self.parse_qdss_field(config, 'struct etmv4_config', 'cfg', 3, indent=2)
        self.parse_qdss_field(config, 'struct etmv4_config', 'eventctrl0', indent=2)
        self.parse_qdss_field(config, 'struct etmv4_config', 'eventctrl1', indent=2)
        self.parse_qdss_field(config, 'struct etmv4_config', 'stall_ctrl', indent=2)
        self.parse_qdss_field(config, 'struct etmv4_config', 'ts_ctrl', 2, indent=2)
        self.parse_qdss_field(config, 'struct etmv4_config', 'syncfreq', indent=2)
        self.parse_qdss_field(config, 'struct etmv4_config', 'ccctlr', 2, indent=2)
        self.parse_qdss_field(config, 'struct etmv4_config', 'bb_ctrl', 2, indent=2)
        self.parse_qdss_field(config, 'struct etmv4_config', 'vinst_ctrl', indent=2)
        self.parse_qdss_field(config, 'struct etmv4_config', 'viiectlr', indent=2)
        self.parse_qdss_field(config, 'struct etmv4_config', 'vissctlr', indent=2)
        self.parse_qdss_field(config, 'struct etmv4_config', 'vipcssctlr', indent=2)
        self.parse_qdss_field(config, 'struct etmv4_config', 'seq_idx', 2, indent=2)
        print("\t\tseq_ctrl\t= {},".format(seq_ctrl), file =self.f)
        self.parse_qdss_field(config, 'struct etmv4_config', 'seq_rst', 2, indent=2)
        self.parse_qdss_field(config, 'struct etmv4_config', 'seq_state', indent=2)
        self.parse_qdss_field(config, 'struct etmv4_config', 'cntr_idx', indent=2)

        print("\t\tcntrldvr\t= {},".format(cntrldvr), file =self.f)
        print("\t\tcntr_ctrl\t= {},".format(cntr_ctrl), file =self.f)
        print("\t\tcntr_val\t= {},".format(cntr_val), file =self.f)
        self.parse_qdss_field(config, 'struct etmv4_config', 'res_idx', 2, indent=2)
        print("\t\tres_ctrl\t= {},".format(res_ctrl), file =self.f)
        self.parse_qdss_field(config, 'struct etmv4_config', 'ss_idx', 2, indent=2)
        print("\t\tss_ctrl\t\t= {},".format(ss_ctrl), file =self.f)
        print("\t\tss_status\t= {},".format(ss_status), file =self.f)
        print("\t\tss_pe_cmp\t= {},".format(ss_pe_cmp), file =self.f)
        self.parse_qdss_field(config, 'struct etmv4_config', 'addr_idx', indent=2)
        print("\t\taddr_val\t= {},".format(addr_val), file =self.f)
        print("\t\taddr_acc\t= {},".format(addr_acc), file =self.f)
        print("\t\taddr_type\t= {},".format(addr_type), file =self.f)
        self.parse_qdss_field(config, 'struct etmv4_config', 'ctxid_idx', indent=2)
        print("\t\tctxid_pid\t= {},".format(ctxid_pid), file =self.f)
        self.parse_qdss_field(config, 'struct etmv4_config', 'ctxid_mask0', indent=2)
        self.parse_qdss_field(config, 'struct etmv4_config', 'ctxid_mask1', indent=2)
        self.parse_qdss_field(config, 'struct etmv4_config', 'vmid_idx', indent=2)
        print("\t\tvmid_val\t= {},".format(vmid_val), file =self.f)
        self.parse_qdss_field(config, 'struct etmv4_config', 'vmid_mask0', indent=2)
        self.parse_qdss_field(config, 'struct etmv4_config', 'vmid_mask1', indent=2)
        self.parse_qdss_field(config, 'struct etmv4_config', 'ext_inp', 2, indent=2)
        self.parse_qdss_field(config, 'struct etmv4_config', 's_ex_level', indent=2)
        print("\t}", file = self.f)
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'save_trfcr')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'save_state')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'state_needs_restore')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'skip_power_up')
        self.parse_qdss_field(drvdata, 'struct etmv4_drvdata', 'arch_features')
        print("}", file = self.f)
        self.parse_clk(pclk)
        self.parse_csdev(csdev)

    def list_qdss_component(self, device):
        bus_type = self.ramdump.read_structure_field(device, 'struct device', 'bus')
        if bus_type == self.coresight_bus :
            dev_name = self.ramdump.read_cstring(self.ramdump.read_word(device + self.kobj_offset + self.name_offset))
            parent = self.ramdump.read_structure_field(device, 'struct device', 'parent')
            parent_name = self.ramdump.read_cstring(self.ramdump.read_word(parent + self.kobj_offset + self.name_offset))
            parent_drv = self.ramdump.read_structure_field(parent, 'struct device', 'driver')
            drvdata = self.ramdump.read_structure_field(parent, 'struct device', 'driver_data')
            drvname = self.ramdump.read_cstring(self.ramdump.read_word(parent_drv + self.drvname_offset))

            if drvname in self.qdss_components :
                print("{}   {}".format(dev_name,hex(device)), file = self.f)
                print("parent: {}   {}".format(parent_name, hex(parent)), file =self.f)
                print("{}".format(drvname), file = self.f)
                getattr(QDSSDump, self.qdss_components[drvname])(self, drvdata, parent)
                print("{}".format("=" * 60), file = self.f)
                print("", file = self.f)

    def parse_qdss_component(self, ramdump):
        self.ramdump = ramdump
        self.qdss_components = dict(qdss_component_func)
        self.entry_offset = self.ramdump.field_offset('struct kobject', 'entry')
        self.name_offset = self.ramdump.field_offset('struct kobject', 'name')
        self.dev_drv_offset = self.ramdump.field_offset('struct device_driver', 'name')
        self.kobj_offset = self.ramdump.field_offset('struct device', 'kobj')
        self.bus_offset = self.ramdump.field_offset('struct device', 'bus')
        self.drvname_offset = self.ramdump.field_offset('struct device_driver', 'name')
        self.amba_dev_offset = self.ramdump.field_offset('struct amba_device', 'dev')
        self.coresight_bus = self.ramdump.address_of('coresight_bustype')
        devices_kset = self.ramdump.address_of('devices_kset')
        self.clk_core_set = set()
        self.f = self.ramdump.open_file("coresight.txt")

        list_head = devices_kset + self.ramdump.field_offset('struct kset', 'list')
        list_offset = self.kobj_offset + self.entry_offset
        list_walker = llist.ListWalker(self.ramdump, list_head, list_offset)
        list_walker.walk(self.list_qdss_component)
        for core in self.clk_core_set:
            self.parse_clk_core(core)
        self.f.close()

    def dump_standard(self, ram_dump):
        self.print_tmc_etf(ram_dump)
        self.print_tmc_etf_swao(ram_dump)
        self.print_tmc_etr(ram_dump)
        self.print_dbgui_registers(ram_dump)
        self.print_all_etm_register(ram_dump)
        self.parse_qdss_component_atid(ram_dump)
        self.parse_qdss_component(ram_dump)
