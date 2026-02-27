# Copyright (c) 2012, 2014-2018, 2020-2021 The Linux Foundation. All rights reserved.
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

import struct
import itertools
from print_out import print_out_str
from iommulib import IommuLib, MSM_SMMU_DOMAIN, MSM_SMMU_AARCH64_DOMAIN, ARM_SMMU_DOMAIN
from aarch64iommulib import create_flat_mappings, create_collapsed_mapping

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
        if d.client_name.endswith(".tmc"):
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

    def dump_standard(self, ram_dump):
        self.print_tmc_etf(ram_dump)
        self.print_tmc_etf_swao(ram_dump)
        self.print_tmc_etr(ram_dump)
        self.print_dbgui_registers(ram_dump)
        self.print_all_etm_register(ram_dump)
