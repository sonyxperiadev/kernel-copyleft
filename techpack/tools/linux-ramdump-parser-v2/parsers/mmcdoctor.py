# Copyright (c) 2017, 2020 The Linux Foundation. All rights reserved.
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

import os
from parser_util import register_parser, RamParser
from parsers.irqstate import IrqParse
from dmesglib import DmesgLib
from print_out import print_out_str

MMC0_IRQ = 0
MMC1_IRQ = 0
SDHCI_IRQ_NAME = "sdhci"
SDHCI_IRQ_CD_NAME = "sdhci cd"

F_MMCDEBUG = "mmcreport.txt"
F_MMCTEMP = "mmctemp.txt"

PATTERN_ST = "========== REGISTER DUMP (mmc0)=========="
PATTERN_END = "sdhci: ===================="

MMC_DATA_HEAD = "#################### MMC(mmc%d) INFO START #######################\n"
MMC_DATA_FOOT = "#################### MMC(mmc%d) INFO END #########################\n\n"

MMC_TRACE_HEADER = "\n################ MMC(mmc%d) RING BUFFER ################\n"
MMC_TRACE_BUF_EVENTS = 64
MMC_TRACE_EVENT_SIZE = 256

card_data = {
    0x2: "SANDISK",
    0x11: "TOSHIBA",
    0x13: "MICRON",
    0x15: "SAMSUNG",
    0x70: "KINGSTON",
    0x90: "HYNIX",
}

card_err_data = {
    "Card Timeout"      : ["err: -110", "error: -110"],
    "Soft Timeout"      : ["Timeout waiting for hardware interrupt", "request with tag"],
    "CRC ERROR"         : ["err: -84", "error: -84"],
    "Resp_error"        : ["status: 0x00000006", "status: 0x00000004"],
    "Spurious_cmd_intr" : ["Got command interrupt", "even though no command"],
    "Spurious_data_intr": ["Got data interrupt", "even though no data"],
    "Spurious_intr"     : ["Unexpected interrupt", "Unexpected interrupt"],
}


#
# This class defines parsing function
# to parse for mmclog
#
class ParseMmcLog():

    def __init__(self, ramdump):
        self.ramdump = ramdump
        self.error = 0
        self.error_line = ""

    def dumptemplog(self):
        fd_temp = self.ramdump.open_file(F_MMCTEMP, "w+")
        d = DmesgLib(self.ramdump, fd_temp)
        d.extract_dmesg()
        fd_temp.close()

    def check_for_error(self, line):
        if (self.error):
            return
        for key, item in card_err_data.items():
            if (item[0] in line or item[1] in line):
                self.error = key
                self.error_line = line
                break

    def dumpmemreport(self):
        self.dumptemplog()
        self.error = 0
        self.error_line = ""
        fdr = self.ramdump.open_file(F_MMCTEMP, "r")
        fdw = self.ramdump.open_file(F_MMCDEBUG, "a")
        line = fdr.readline()
        while (line):
            if ("mmc" in line):
                fdw.write(line)
                self.check_for_error(line)
            if (PATTERN_ST in line):
                line = fdr.readline()
                while (line and (PATTERN_END not in line)):
                    fdw.write("ERROR" + line)
                    line = fdr.readline()
                if (line and PATTERN_END in line):
                    fdw.write("ERROR" + line)
            line = fdr.readline()
        fdw.close()
        fdr.close()
        self.ramdump.remove_file(F_MMCTEMP)


#
# Function definition to find all relevant structure
#

def get_sdhci_irqs(ram_dump):
    irqs = IrqParse(ram_dump)
    count = 0
    global MMC0_IRQ
    global MMC1_IRQ
    irq_action_offset = ram_dump.field_offset('struct irq_desc', 'action')
    action_name_offset = ram_dump.field_offset('struct irqaction', 'name')
    irq_desc_tree = ram_dump.address_of('irq_desc_tree')
    nr_irqs = ram_dump.read_int(ram_dump.address_of('nr_irqs'))

    if nr_irqs > 50000:
        return
    for i in range(0, nr_irqs):
        if (ram_dump.kernel_version >= (4,9,0)):
            irq_desc = irqs.radix_tree_lookup_element_v2(
                ram_dump, irq_desc_tree, i)
        else:
            irq_desc = irqs.radix_tree_lookup_element(
                ram_dump, irq_desc_tree, i)
        if irq_desc is None:
            continue
        action = ram_dump.read_word(irq_desc + irq_action_offset)
        if action != 0 and count < 2:
            name_addr = ram_dump.read_word(action + action_name_offset)
            name = ram_dump.read_cstring(name_addr, 48)
            if (not name):
                continue
            if (SDHCI_IRQ_NAME in name and not SDHCI_IRQ_CD_NAME in name):
                if (count == 0):
                    MMC0_IRQ = i
                elif (count == 1):
                    MMC1_IRQ = i
                count += 1
    return count

def find_sdhci_host(ramdump, irq):
    irq_desc_tree = ramdump.address_of('irq_desc_tree')
    irq_action_offset = ramdump.field_offset('struct irq_desc', 'action')
    dev_id = ramdump.field_offset('struct irqaction', 'dev_id')
    irqs = IrqParse(ramdump)
    if (ramdump.kernel_version >= (4,9,0)):
        sdhci_irq_desc = irqs.radix_tree_lookup_element_v2(
            ramdump, irq_desc_tree, irq)
    else:
        sdhci_irq_desc = irqs.radix_tree_lookup_element(
            ramdump, irq_desc_tree, irq)
    sdhci_irq_action = ramdump.read_word(sdhci_irq_desc + irq_action_offset)
    sdhci_host = ramdump.read_word(sdhci_irq_action + dev_id)
    return sdhci_host


def find_mmc_host(ramdump, sdhci_host):
    mmc_offset = ramdump.field_offset('struct sdhci_host', 'mmc')
    mmc_host = ramdump.read_word(sdhci_host + mmc_offset)
    return mmc_host


def find_mmc_card(ramdump, mmc_host):
    card_offset = ramdump.field_offset('struct mmc_host', 'card')
    mmc_card = ramdump.read_word(mmc_host + card_offset)
    return mmc_card


def find_sdhci_msm_host(ramdump, index):
    sdhci_slot_ptr = ramdump.address_of('sdhci_slot')
    sdhci_msm_index = ramdump.array_index(sdhci_slot_ptr, 'struct sdhci_msm_host *', index)
    sdhci_msm_host_addr = ramdump.read_word(sdhci_msm_index)
    return sdhci_msm_host_addr


class MmcCardInfo():

    def __init__(self, ramdump, mmc_card):
        self.ramdump = ramdump
        self.card = mmc_card
        self.type = self.find_mmc_cid()
        self.ext_csd_fwrev = self.find_ext_csd()
        self.cmdq_init = self.find_cmdq_init()

    def find_ext_csd(self):
        if (not self.card):
            return 0
        ext_csd_off = self.card + \
            self.ramdump.field_offset('struct mmc_card', 'ext_csd')
        self.mmc_ext_csd = ext_csd_off
        fw_version_offset = self.ramdump.field_offset('struct mmc_ext_csd', 'fw_version')
        if (fw_version_offset is None):
            return -2
        ext_csd_fwrev = ext_csd_off + fw_version_offset
        return self.ramdump.read_byte(ext_csd_fwrev)

    def find_mmc_cid(self):
        if (not self.card):
            return 0
        cid = self.card + self.ramdump.field_offset('struct mmc_card', 'cid')
        self.mmc_cid = cid
        manfid = self.ramdump.read_u32(cid)
        if manfid in card_data.keys():
            return card_data[manfid]
        else:
            return manfid

    def find_cmdq_init(self):
        if (not self.card):
            return 0
        offset = self.ramdump.field_offset('struct mmc_host', 'cmdq_init')
        if (not offset):
            return -1
        cmdq_init = self.card + offset
        return self.ramdump.read_bool(cmdq_init)


class MmcHostInfo():

    def __init__(self, ramdump, mmc_host):
        self.ramdump = ramdump
        self.host = mmc_host
        if (self.ramdump.field_offset('struct mmc_host', 'clk_gated')):
            self.clk_gated = self.ramdump.read_bool(self.host +
                        self.ramdump.field_offset('struct mmc_host', 'clk_gated'))
            self.clk_requests = self.ramdump.read_int(self.host +
                        self.ramdump.field_offset('struct mmc_host', 'clk_requests'))
            self.clk_old = self.ramdump.read_int(self.host +
                        self.ramdump.field_offset('struct mmc_host', 'clk_old'))
        else:
            self.clk_gated = self.clk_requests = self.clk_old = -23

        offset = self.ramdump.field_offset('struct mmc_host', 'err_occurred')
        if (offset):
            self.err_occurred = self.ramdump.read_bool(self.host + offset)
        else:
            self.err_occurred = -1
        self.ios = self.find_ios()
        self.ios_clock = self.ramdump.read_int(self.ios +
                    self.ramdump.field_offset('struct mmc_ios', 'clock'))
        offset = self.ramdump.field_offset('struct mmc_ios', 'old_rate')
        if (offset) :
            self.ios_old_rate = self.ramdump.read_int(self.ios + offset)
        else:
            self.ios_old_rate = -1
        self.ios_vdd = self.ramdump.read_u16(self.ios +
                    self.ramdump.field_offset('struct mmc_ios', 'vdd'))
        self.ios_bus_width = self.ramdump.read_byte(self.ios +
                    self.ramdump.field_offset('struct mmc_ios', 'bus_width'))
        self.ios_timing = self.ramdump.read_byte(self.ios +
                    self.ramdump.field_offset('struct mmc_ios', 'timing'))
        self.ios_signal_voltage = self.ramdump.read_byte(self.ios +
                    self.ramdump.field_offset('struct mmc_ios', 'signal_voltage'))
        self.ios_drv_type= self.ramdump.read_byte(self.ios +
                    self.ramdump.field_offset('struct mmc_ios', 'drv_type'))

    def find_ios(self):
        return (self.host + self.ramdump.field_offset('struct mmc_host', 'ios'))


#
# Base class which save all mmc data structure info
# This uses other classes for card and host related
# info
#
class MmcDataStructure():
    global MMC0_IRQ
    global MMC1_IRQ

    def __init__(self, ramdump, index):
        self.mmclist_irq = {0 : MMC0_IRQ, 1 : MMC1_IRQ}
        self.index = index
        self.irq = self.mmclist_irq[index]
        if (self.irq == 0):
            return
        self.ramdump = ramdump
        self.sdhci_host = find_sdhci_host(self.ramdump, self.irq)
        self.mmc_host = find_mmc_host(self.ramdump, self.sdhci_host)
        self.mmc_card = find_mmc_card(self.ramdump, self.mmc_host)
        self.sdhci_msm_host = find_sdhci_msm_host(self.ramdump, self.index)
        self.cardinfo = MmcCardInfo(self.ramdump, self.mmc_card)
        self.hostinfo = MmcHostInfo(self.ramdump, self.mmc_host)
        self.parse = ParseMmcLog(self.ramdump)
        return

    def dump_trace_buf(self, fd):
        if (not self.mmc_host or (fd == None) or (self.ramdump.kernel_version < (4, 4))):
            return
        mmc_host = self.mmc_host
        ramdump = self.ramdump

        fd.write(MMC_TRACE_HEADER % self.index)
        buf_offset = ramdump.field_offset('struct mmc_host', 'trace_buf')
        if (not buf_offset):
            return 0

        data_offset = ramdump.field_offset('struct mmc_trace_buffer', 'data')
        if (not data_offset):
            return 0

        trace_buf = mmc_host + buf_offset
        wr_idx = ramdump.read_int(trace_buf)
        if (wr_idx >= 0xFFFFFFFF):
            fd.write("mmc%d trace buffer empty\n" %(self.index))
            return 0

        dataptr = ramdump.read_word(trace_buf + data_offset)
        if (not dataptr):
            return

        datastr = []
        for i in range(MMC_TRACE_BUF_EVENTS):
            trace_str = ramdump.read_cstring(dataptr, MMC_TRACE_EVENT_SIZE)
            dataptr += MMC_TRACE_EVENT_SIZE
            datastr.append(trace_str)
        num = MMC_TRACE_BUF_EVENTS - 1
        idx = wr_idx & num
        cur_idx = (idx + 1) & num
        for i in range(MMC_TRACE_BUF_EVENTS):
            fd.write("event[%d] = %s" %(cur_idx, datastr[cur_idx]))
            cur_idx = (cur_idx + 1 ) & num

    def dump_data(self, mode):
        fd = self.ramdump.open_file(F_MMCDEBUG, mode)
        fd.write(MMC_DATA_HEAD % self.index)
        fd.write("mmc_card = 0x%x\n" % self.mmc_card)
        fd.write("mmc_host = 0x%x\n" % self.mmc_host)
        fd.write("sdhci_host = 0x%x\n" % self.sdhci_host)
        fd.write("sdhci_msm_host = 0x%x\n\n" % self.sdhci_msm_host)
        fd.write("mmc_host->err_occurred = %d\n" % self.hostinfo.err_occurred)
        fd.write("Grep MMC_ERROR TYPE at End of File\n\n")
        fd.write("CARD MANFID = %s\n" %self.cardinfo.type)
        fd.write("CARD Fw_rev = 0x%x\n" %self.cardinfo.ext_csd_fwrev)
        fd.write("CARD CMDQ INIT = %d\n\n" %self.cardinfo.cmdq_init)
        fd.write("Host clk_requests = %d\n" %self.hostinfo.clk_requests)
        fd.write("Host clk_gated = %d\n" %self.hostinfo.clk_gated)
        fd.write("Host clk_old = %dHz\n" %self.hostinfo.clk_old)
        fd.write("Host ios_clock = %dHz\n" %self.hostinfo.ios_clock)
        fd.write("Host ios_old_rate = %uHz\n" %self.hostinfo.ios_old_rate)
        fd.write("Host ios_vdd = %d\n" %self.hostinfo.ios_vdd)
        fd.write("Host ios_bus_width = %d\n" %self.hostinfo.ios_bus_width)
        fd.write("Host ios_timing = %d\n" %self.hostinfo.ios_timing)
        fd.write("Host ios_signal_voltage = %d\n" %self.hostinfo.ios_signal_voltage)
        fd.write("Host ios_drv_type = %d\n" %self.hostinfo.ios_drv_type)
        self.dump_trace_buf(fd)
        fd.write(MMC_DATA_FOOT % self.index)
        fd.close()
        return


def dump_mmc_info(ramdump, count):
    mmc0 = MmcDataStructure(ramdump, 0)
    mmc0.dump_data("w+")

    if (count > 1):
        mmc1 = MmcDataStructure(ramdump, 1)
        mmc1.dump_data("a")

    mmc0.parse.dumpmemreport()
    fd = ramdump.open_file(F_MMCDEBUG, "a")
    fd.write("\n\n\n#########MMC_ERROR#########\n")
    if (mmc0.parse.error):
        fd.write("ERROR TYPE = %s\n" %mmc0.parse.error)
    else:
       fd.write("ERROR TYPE = Unknown Error\n")
    fd.write("ERROR SIGN = %s\n\n" % mmc0.parse.error_line)
    fd.close()


@register_parser('--mmcdoctor', 'Generate MMC diagnose report')
class MmcDebug(RamParser):
    def parse(self):
        count_irq = 0
        if self.ramdump.is_config_defined('CONFIG_SPARSE_IRQ'):
            count_irq = get_sdhci_irqs(self.ramdump)
            if (count_irq == 0):
                return
            dump_mmc_info(self.ramdump, count_irq)
        else:
            print_out_str("\n Could not generate MMC diagnose report\n")
            return
