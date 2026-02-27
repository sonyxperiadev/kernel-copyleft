# Copyright (c) 2017, 2020 The Linux Foundation. All rights reserved.
# Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
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

MMC_DATA_HEAD = "#################### MMC(mmc%d) INFO START #######################\n"
MMC_DATA_FOOT = "#################### MMC(mmc%d) INFO END #########################\n\n"

MMC_TRACE_HEADER = "\n################ MMC(mmc%d) RING BUFFER ################\n"
MMC_TRACE_BUF_EVENTS = 64
MMC_TRACE_EVENT_SIZE = 256

mmc_rpm_request_l = ['RPM_REQ_NONE', 'RPM_REQ_IDLE', 'RPM_REQ_SUSPEND', 'RPM_REQ_AUTOSUSPEND', 'RPM_REQ_RESUME']
mmc_rpm_status_l = ['RPM_ACTIVE', 'RPM_RESUMING', 'RPM_SUSPENDED', 'RPM_SUSPENDING']

ios_power_mode = {
    0: "MMC_POWER_OFF",
    1: "MMC_POWER_UP",
    2: "MMC_POWER_ON",
    3: "MMC_POWER_UNDEFINED",
}

card_bus_width = {
    0: "EXT_CSD_BUS_WIDTH_1",
    1: "EXT_CSD_BUS_WIDTH_4",
    2: "EXT_CSD_BUS_WIDTH_8",
    5: "EXT_CSD_DDR_BUS_WIDTH_4",
    6: "EXT_CSD_DDR_BUS_WIDTH_8",
}

card_type = {
    0: "MMC_TYPE_eMMC",
    1: "MMC_TYPE_SD",
    2: "MMC_TYPE_SDIO",
    3: "MMC_TYPE_SD_COMBO",
}

card_state = ["MMC_STATE_PRESENT", "MMC_STATE_READONLY", "MMC_STATE_BLOCKADDR", "MMC_CARD_SDXC",
"MMC_CARD_REMOVED", "MMC_STATE_SUSPENDED"]

req_flags_arr = ["Rsvd", "Rsvd", "Rsvd", "Rsvd", "Rsvd", "Rsvd", "Rsvd", "Rsvd",
"MMC_DATA_WRITE", "MMC_DATA_READ", "MMC_DATA_QBR", "MMC_DATA_PRIO", "MMC_DATA_REL_WR",
 "MMC_DATA_DAT_TAG", "MMC_DATA_FORCED_PRG"]

ios_timing = {
    0: "LEGACY",
    1: "MMC_HS",
    2: "SD_HS",
    3: "UHS_SDR12",
    4: "UHS_SDR25",
    5: "UHS_SDR50",
    6: "UHS_SDR104",
    7: "UHS_DDR50",
    8: "MMC_DDR52",
    9: "MMC_HS200",
    10: "MMC_HS400",
}

cid_manfid = {
    0x2: "SANDISK",
    0x3: "SANDISK_SD",
    0x9: "ATP",
    0x11: "TOSHIBA",
    0x13: "MICRON",
    0x15: "SAMSUNG",
    0x27: "APACER",
    0x70: "KINGSTON",
    0x90: "HYNIX",
    0xFE: "NUMONYX",
}

card_err_data = {
    "Card Timeout"      : ["err: -110", "error: -110"],
    "Soft Timeout"      : ["Timeout waiting for hardware interrupt", "request with tag"],
    "CRC ERROR"         : ["err: -84", "error: -84"],
    "Resp_error"        : ["status: 0x00000006", "status: 0x00000004"],
    "Spurious_cmd_intr" : ["Got command interrupt", "even though no command"],
    "Spurious_data_intr": ["Got data interrupt", "even though no data"],
    "Spurious_intr"     : ["Unexpected interrupt", "Unexpected interrupt"],
    "CMDQ Engine issue" : ["running CQE recovery", "timeout for tag"],
}

def setup_out_file(path, self):
    global out_file
    try:
        out_file = self.ramdump.open_file(path, 'wb')
        return out_file
    except:
        print_out_str("could not open path {0}".format(path))
        print_out_str("Do you have write/read permissions on the path?")


def print_out_mmc(string, newline=False):
    if newline:
        out_file.write((string + '\n').encode('ascii', 'ignore'))
        return
    out_file.write((string).encode('ascii', 'ignore'))

def print_set_bits(num,arr):
    for i in range(len(arr)):
        if ((num>>i) & 1 ):
            print_out_mmc("[%s] " %arr[i])
    print_out_mmc("\n")

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
        line = fdr.readline()
        print_out_mmc("#################### dmesg MMC Info #########################\n")
        while (line):
            if ("mmc" in line or "sdhci" in line or "cqhci" in line):
                print_out_mmc(line)
                self.check_for_error(line)
            line = fdr.readline()
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
    nr_irqs = ram_dump.read_int(ram_dump.address_of('nr_irqs'))

    if nr_irqs > 50000:
        return

    irq_descs = irqs.get_all_irqs_desc(ram_dump)
    for i in range(len(irq_descs)):
        action = ram_dump.read_word(irq_descs[i] + irq_action_offset)
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
    irq_action_offset = ramdump.field_offset('struct irq_desc', 'action')
    dev_id = ramdump.field_offset('struct irqaction', 'dev_id')
    irqs = IrqParse(ramdump)

    irq_descs = irqs.get_all_irqs_desc(ramdump)
    if irq > (len(irq_descs) - 1):
        print_out_str("Invalid irq index")
        return
    sdhci_irq_action = ramdump.read_word(irq_descs[irq] + irq_action_offset)
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
    if not sdhci_slot_ptr :
        print_out_str("sdhci_msm_host struct not found in sdhci_slot\n")
        return
    sdhci_msm_index = ramdump.array_index(sdhci_slot_ptr, 'struct sdhci_msm_host *', index)
    sdhci_msm_host_addr = ramdump.read_word(sdhci_msm_index)
    return sdhci_msm_host_addr

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
        self.parse = ParseMmcLog(self.ramdump)
        if not self.sdhci_host:
            self.sdhci_host = 0
        if not self.mmc_host:
            self.mmc_host = 0
        if not self.mmc_card:
            self.mmc_card = 0
        if not self.sdhci_msm_host:
            self.sdhci_msm_host = 0
        return

    def dump_trace_buf(self):
        if (not self.mmc_host):
            return
        mmc_host = self.mmc_host
        ramdump = self.ramdump

        buf_offset = ramdump.field_offset('struct mmc_host', 'trace_buf')
        if (not buf_offset):
            return 0

        data_offset = ramdump.field_offset('struct mmc_trace_buffer', 'data')
        if (not data_offset):
            return 0

        print_out_mmc(MMC_TRACE_HEADER % self.index)
        trace_buf = mmc_host + buf_offset
        wr_idx = ramdump.read_int(trace_buf)
        if (wr_idx >= 0xFFFFFFFF):
            print_out_mmc("mmc%d trace buffer empty\n" %(self.index))
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
            print_out_mmc("event[%d] = %s" %(cur_idx, datastr[cur_idx]))
            cur_idx = (cur_idx + 1 ) & num

    def dump_sdhci_host(self):
        if not self.sdhci_host :
           print_out_mmc("\nstruct sdhci_host { NULL }\n")
           return
        print_out_mmc("\nstruct sdhci_host = 0x%x {\n" %self.sdhci_host)
        print_out_mmc("\tquirks = %d\n" %self.ramdump.read_int(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'quirks')))
        print_out_mmc("\tquirks2 = %d\n" %self.ramdump.read_int(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'quirks2')))
        print_out_mmc("\tirq = %d\n" %self.ramdump.read_int(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'irq')))
        print_out_mmc("\tioaddr = 0x%x\n" %self.ramdump.read_pointer(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'ioaddr')))
        print_out_mmc("\tbounce_buffer_size = %d\n" %self.ramdump.read_int(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'bounce_buffer_size')))
        print_out_mmc("\tflags = %d\n" %self.ramdump.read_int(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'flags')))
        print_out_mmc("\tversion = %d\n" %self.ramdump.read_int(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'version')))
        print_out_mmc("\tmax_clk = %d\n" %self.ramdump.read_int(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'max_clk')))
        print_out_mmc("\ttimeout_clk = %d\n" %self.ramdump.read_int(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'timeout_clk')))
        print_out_mmc("\tclk_mul = %d\n" %self.ramdump.read_int(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'clk_mul')))
        print_out_mmc("\tclock = %d\n" %self.ramdump.read_int(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'clock')))
        print_out_mmc("\tdata_early = %d\n" %self.ramdump.read_int(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'data_early')))
        print_out_mmc("\tblocks = %d\n" %self.ramdump.read_int(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'blocks')))
        print_out_mmc("\tsg_count = %d\n" %self.ramdump.read_int(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'sg_count')))
        if (self.ramdump.kernel_version >= (5,10,0)):
            print_out_mmc("\tmax_adma = %d\n" %self.ramdump.read_int(self.sdhci_host +
                        self.ramdump.field_offset('struct sdhci_host', 'max_adma')))
            print_out_mmc("\tuse_external_dma = %d\n" %self.ramdump.read_bool(self.sdhci_host +
                        self.ramdump.field_offset('struct sdhci_host', 'use_external_dma')))
            print_out_mmc("\tmax_timeout_count = %d\n" %self.ramdump.read_byte(self.sdhci_host +
                        self.ramdump.field_offset('struct sdhci_host', 'max_timeout_count')))
            print_out_mmc("\tsdhci_core_to_disable_vqmmc = %d\n" %self.ramdump.read_bool(self.sdhci_host +
                        self.ramdump.field_offset('struct sdhci_host', 'sdhci_core_to_disable_vqmmc')))
        print_out_mmc("\tdesc_sz = %d\n" %self.ramdump.read_int(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'desc_sz')))
        print_out_mmc("\talloc_desc_sz = %d\n" %self.ramdump.read_int(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'alloc_desc_sz')))
        print_out_mmc("\tadma_table_sz = %d\n" %self.ramdump.read_u32(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'adma_table_sz')))
        print_out_mmc("\talign_buffer_sz = %d\n" %self.ramdump.read_u32(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'align_buffer_sz')))
        print_out_mmc("\tcaps = %d\n" %self.ramdump.read_u32(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'caps')))
        print_out_mmc("\tcaps1 = %d\n" %self.ramdump.read_u32(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'caps1')))
        print_out_mmc("\tocr_mask = %d\n" %self.ramdump.read_u32(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'ocr_mask')))
        print_out_mmc("\tpwr = %d\n" %self.ramdump.read_byte(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'pwr')))
        print_out_mmc("\truntime_suspended = %d\n" %self.ramdump.read_bool(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'runtime_suspended')))
        print_out_mmc("\tbus_on = %d\n" %self.ramdump.read_bool(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'bus_on')))
        print_out_mmc("\tpreset_enabled = %d\n" %self.ramdump.read_bool(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'preset_enabled')))
        print_out_mmc("\tpending_reset = %d\n" %self.ramdump.read_bool(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'pending_reset')))
        print_out_mmc("\tirq_wake_enabled = %d\n" %self.ramdump.read_bool(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'irq_wake_enabled')))
        print_out_mmc("\tv4_mode = %d\n" %self.ramdump.read_bool(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'v4_mode')))
        print_out_mmc("\talways_defer_done = %d\n" %self.ramdump.read_bool(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'always_defer_done')))
        print_out_mmc("\tread_caps = %d\n" %self.ramdump.read_bool(self.sdhci_host +
                    self.ramdump.field_offset('struct sdhci_host', 'read_caps')))
        print_out_mmc("}\n")
        return

    def get_vreg_info(self, vreg_addr, vreg):
        vreg_p = self.ramdump.read_pointer(vreg_addr + self.ramdump.field_offset('struct sdhci_msm_vreg_data', vreg))
        if not vreg_p :
            return
        print_out_mmc("\t\t[%s]{\n"%vreg)
        print_out_mmc("\t\t\taddr = 0x%x\n" %vreg_p)
        print_out_mmc("\t\t\tname = %s\n" %(self.ramdump.read_structure_cstring(
                                        vreg_p, 'struct sdhci_msm_reg_data', 'name')))
        print_out_mmc("\t\t\tis_always_on = %d\n" %(self.ramdump.read_bool(vreg_p +
                                        self.ramdump.field_offset(
                                        'struct sdhci_msm_reg_data',
                                        'is_always_on'))))
        print_out_mmc("\t\t\tis_enabled = %d\n" %(self.ramdump.read_bool(vreg_p +
                                        self.ramdump.field_offset(
                                        'struct sdhci_msm_reg_data',
                                        'is_enabled'))))
        print_out_mmc("\t\t\tset_voltage_sup = %d\n" %(self.ramdump.read_bool(vreg_p +
                                        self.ramdump.field_offset(
                                        'struct sdhci_msm_reg_data',
                                        'set_voltage_sup'))))
        reg_p = self.ramdump.read_pointer(vreg_p + self.ramdump.field_offset('struct sdhci_msm_reg_data', 'reg'))
        rdev_p = self.ramdump.read_pointer(reg_p + self.ramdump.field_offset('struct regulator', 'rdev'))
        rdesc_p = self.ramdump.read_pointer(rdev_p + self.ramdump.field_offset('struct regulator_dev', 'desc'))
        print_out_mmc("\t\t\tregulator: %s\n" %(self.ramdump.read_structure_cstring(
                                        rdesc_p, 'struct regulator_desc', 'name')))
        print_out_mmc("\t\t\tsupply: %s\n" %(self.ramdump.read_structure_cstring(
                                        rdesc_p, 'struct regulator_desc', 'supply_name')))
        print_out_mmc("\t\t}\n")
        return

    def dump_mmc_vreg(self):
        vreg_addr = self.ramdump.read_pointer(self.sdhci_msm_host + self.ramdump.field_offset('struct sdhci_msm_host', 'vreg_data'))
        print_out_mmc("\tstruct sdhci_msm_vreg_data = 0x%x {\n" %vreg_addr)
        if not vreg_addr:
           print_out_mmc("\tNULL\n}\n")
           return
        self.get_vreg_info(vreg_addr, "vdd_data")
        self.get_vreg_info(vreg_addr, "vdd_io_data")
        print_out_mmc("\t}\n")
        return

    def dump_clk_scailing_info(self, offset):
        self.sdhci_msm_mmc_devfeq_clk_scaling = self.sdhci_msm_host + offset
        print_out_mmc("\tstruct sdhci_msm_mmc_devfeq_clk_scaling = 0x%x {\n" %self.sdhci_msm_mmc_devfeq_clk_scaling)
        if not self.sdhci_msm_mmc_devfeq_clk_scaling :
           print_out_mmc("\tNULL\n}\n")
           return
        dev_pro = self.sdhci_msm_mmc_devfeq_clk_scaling + self.ramdump.field_offset('struct sdhci_msm_mmc_devfeq_clk_scaling', 'devfreq_profile')
        print_out_mmc("\t\ttimer = %d [DEVFREQ_TIMER_DELAYED]\n" %self.ramdump.read_int(dev_pro +
                    self.ramdump.field_offset('struct devfreq_dev_profile', 'timer')))
        print_out_mmc("\t\tfreq_table_sz = %d\n" %self.ramdump.read_int(self.sdhci_msm_mmc_devfeq_clk_scaling +
                    self.ramdump.field_offset('struct sdhci_msm_mmc_devfeq_clk_scaling', 'freq_table_sz')))
        print_out_mmc("\t\tupthreshold = %d\n" %self.ramdump.read_int(self.sdhci_msm_mmc_devfeq_clk_scaling +
                    self.ramdump.field_offset('struct sdhci_msm_mmc_devfeq_clk_scaling', 'upthreshold')))
        print_out_mmc("\t\tdownthreshold = %d\n" %self.ramdump.read_int(self.sdhci_msm_mmc_devfeq_clk_scaling +
                    self.ramdump.field_offset('struct sdhci_msm_mmc_devfeq_clk_scaling', 'downthreshold')))
        print_out_mmc("\t\tlower_bus_speed_mode = %d\n" %self.ramdump.read_int(self.sdhci_msm_mmc_devfeq_clk_scaling +
                    self.ramdump.field_offset('struct sdhci_msm_mmc_devfeq_clk_scaling', 'lower_bus_speed_mode')))
        print_out_mmc("\t\tstart_busy = %d\n" %self.ramdump.read_ulong(self.sdhci_msm_mmc_devfeq_clk_scaling +
                    self.ramdump.field_offset('struct sdhci_msm_mmc_devfeq_clk_scaling', 'start_busy')))
        print_out_mmc("\t\tmeasure_interval_start = %d\n" %self.ramdump.read_ulong(self.sdhci_msm_mmc_devfeq_clk_scaling +
                    self.ramdump.field_offset('struct sdhci_msm_mmc_devfeq_clk_scaling', 'measure_interval_start')))
        print_out_mmc("\t\tdevfreq_abort = %d\n" %self.ramdump.read_ulong(self.sdhci_msm_mmc_devfeq_clk_scaling +
                    self.ramdump.field_offset('struct sdhci_msm_mmc_devfeq_clk_scaling', 'devfreq_abort')))
        print_out_mmc("\t\ttotal_busy_time_us = %d\n" %self.ramdump.read_ulong(self.sdhci_msm_mmc_devfeq_clk_scaling +
                    self.ramdump.field_offset('struct sdhci_msm_mmc_devfeq_clk_scaling', 'total_busy_time_us')))
        print_out_mmc("\t\ttarget_freq = %d\n" %self.ramdump.read_ulong(self.sdhci_msm_mmc_devfeq_clk_scaling +
                    self.ramdump.field_offset('struct sdhci_msm_mmc_devfeq_clk_scaling', 'target_freq')))
        print_out_mmc("\t\tcurr_freq = %d\n" %self.ramdump.read_ulong(self.sdhci_msm_mmc_devfeq_clk_scaling +
                    self.ramdump.field_offset('struct sdhci_msm_mmc_devfeq_clk_scaling', 'curr_freq')))
        print_out_mmc("\t\tpolling_delay_ms = %d\n" %self.ramdump.read_ulong(self.sdhci_msm_mmc_devfeq_clk_scaling +
                    self.ramdump.field_offset('struct sdhci_msm_mmc_devfeq_clk_scaling', 'polling_delay_ms')))
        print_out_mmc("\t\tskip_clk_scale_freq_update = %d\n" %self.ramdump.read_bool(self.sdhci_msm_mmc_devfeq_clk_scaling +
                    self.ramdump.field_offset('struct sdhci_msm_mmc_devfeq_clk_scaling', 'skip_clk_scale_freq_update')))
        print_out_mmc("\t\tneed_freq_change = %d\n" %self.ramdump.read_bool(self.sdhci_msm_mmc_devfeq_clk_scaling +
                    self.ramdump.field_offset('struct sdhci_msm_mmc_devfeq_clk_scaling', 'need_freq_change')))
        print_out_mmc("\t\tis_busy_started = %d\n" %self.ramdump.read_bool(self.sdhci_msm_mmc_devfeq_clk_scaling +
                    self.ramdump.field_offset('struct sdhci_msm_mmc_devfeq_clk_scaling', 'is_busy_started')))
        print_out_mmc("\t\tenable = %d\n" %self.ramdump.read_bool(self.sdhci_msm_mmc_devfeq_clk_scaling +
                    self.ramdump.field_offset('struct sdhci_msm_mmc_devfeq_clk_scaling', 'enable')))
        print_out_mmc("\t\tis_suspended = %d\n" %self.ramdump.read_bool(self.sdhci_msm_mmc_devfeq_clk_scaling +
                    self.ramdump.field_offset('struct sdhci_msm_mmc_devfeq_clk_scaling', 'is_suspended')))
        print_out_mmc("\t}\n")
        return
    def dump_sdhci_msm_host(self):
        if not self.sdhci_msm_host :
           print_out_mmc("\nstruct sdhci_msm_host { NULL }\n")
           print_out_mmc("Error: Missing sdhci_msm_host address in dumps, Please add sdhci-msm.ko path in --mod_path or might issue with MMC driver\n")
           return
        print_out_mmc("\nstruct sdhci_msm_host = 0x%x {\n" %self.sdhci_msm_host)
        offset = self.ramdump.field_offset('struct sdhci_msm_host', 'clk_scaling')
        if offset :
            self.dump_clk_scailing_info(offset)
        self.dump_mmc_vreg()
        if (self.ramdump.kernel_version >= (5,10,0)):
            print_out_mmc("\tactive_reqs = %lu\n" %self.ramdump.read_ulong(self.sdhci_msm_host +
                        self.ramdump.field_offset('struct sdhci_msm_host', 'active_reqs')))
            print_out_mmc("\tpart_curr = %d\n" %self.ramdump.read_int(self.sdhci_msm_host +
                        self.ramdump.field_offset('struct sdhci_msm_host', 'part_curr')))
            print_out_mmc("\tclk_scale_init_done = %d\n" %self.ramdump.read_int(self.sdhci_msm_host +
                        self.ramdump.field_offset('struct sdhci_msm_host', 'clk_scale_init_done')))
            print_out_mmc("\tdefer_clk_scaling_resume = %d\n" %self.ramdump.read_int(self.sdhci_msm_host +
                        self.ramdump.field_offset('struct sdhci_msm_host', 'defer_clk_scaling_resume')))
            print_out_mmc("\tscaling_suspended = %d\n" %self.ramdump.read_int(self.sdhci_msm_host +
                        self.ramdump.field_offset('struct sdhci_msm_host', 'scaling_suspended')))
            print_out_mmc("\tdll_config = %d\n" %self.ramdump.read_u32(self.sdhci_msm_host +
                        self.ramdump.field_offset('struct sdhci_msm_host', 'dll_config')))
            print_out_mmc("\tddr_config = %d\n" %self.ramdump.read_u32(self.sdhci_msm_host +
                        self.ramdump.field_offset('struct sdhci_msm_host', 'ddr_config')))
            print_out_mmc("\tlast_cmd = %d\n" %self.ramdump.read_u16(self.sdhci_msm_host +
                        self.ramdump.field_offset('struct sdhci_msm_host', 'last_cmd')))
            print_out_mmc("\tdll_lock_bist_fail_wa = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                        self.ramdump.field_offset('struct sdhci_msm_host', 'dll_lock_bist_fail_wa')))
            print_out_mmc("\tdbg_en = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                        self.ramdump.field_offset('struct sdhci_msm_host', 'dbg_en')))
            print_out_mmc("\terr_occurred = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                        self.ramdump.field_offset('struct sdhci_msm_host', 'err_occurred')))
            print_out_mmc("\tcrash_on_err = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                        self.ramdump.field_offset('struct sdhci_msm_host', 'crash_on_err')))
            print_out_mmc("\tcore_3_0v_support = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                        self.ramdump.field_offset('struct sdhci_msm_host', 'core_3_0v_support')))
            print_out_mmc("\tscale_caps = %d\n" %self.ramdump.read_int(self.sdhci_msm_host +
                        self.ramdump.field_offset('struct sdhci_msm_host', 'scale_caps')))
            print_out_mmc("\traw_ext_csd_cmdq = %d\n" %self.ramdump.read_byte(self.sdhci_msm_host +
                        self.ramdump.field_offset('struct sdhci_msm_host', 'raw_ext_csd_cmdq')))
            print_out_mmc("\traw_ext_csd_cache_ctrl = %d\n" %self.ramdump.read_byte(self.sdhci_msm_host +
                        self.ramdump.field_offset('struct sdhci_msm_host', 'raw_ext_csd_cache_ctrl')))
            bus_width = self.ramdump.read_byte(self.sdhci_msm_host +
                        self.ramdump.field_offset('struct sdhci_msm_host', 'raw_ext_csd_bus_width'))
            print_out_mmc("\tuses_tassadar_dll = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                        self.ramdump.field_offset('struct sdhci_msm_host', 'uses_tassadar_dll')))
            print_out_mmc("\tuses_level_shifter = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                        self.ramdump.field_offset('struct sdhci_msm_host', 'uses_level_shifter')))
            print_out_mmc("\tvqmmc_enabled = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                        self.ramdump.field_offset('struct sdhci_msm_host', 'vqmmc_enabled')))
            if  bus_width in card_bus_width.keys():
                print_out_mmc("\traw_ext_csd_bus_width = %d [%s]\n" %(bus_width,card_bus_width[bus_width]))
            else:
                print_out_mmc("\t\traw_ext_csd_bus_width = %d [Unknown]\n" %(bus_width))
            print_out_mmc("\traw_ext_csd_hs_timing = %d\n" %self.ramdump.read_byte(self.sdhci_msm_host +
                        self.ramdump.field_offset('struct sdhci_msm_host', 'raw_ext_csd_hs_timing')))
            print_out_mmc("\thas_opp_table = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                        self.ramdump.field_offset('struct sdhci_msm_host', 'has_opp_table')))
        else:
            vote = self.ramdump.read_pointer(self.sdhci_msm_host +
                                    self.ramdump.field_offset('struct sdhci_msm_host', 'bus_vote_data'))
            print_out_mmc("\tvote = %u\n" %self.ramdump.read_u32(vote +
                    self.ramdump.field_offset('struct sdhci_msm_bus_vote_data', 'curr_vote')))
        print_out_mmc("\tclk_rate = %d\n" %self.ramdump.read_ulong(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'clk_rate')))
        print_out_mmc("\tpwr_irq = %d\n" %self.ramdump.read_int(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'pwr_irq')))
        print_out_mmc("\tcurr_pwr_state = %d\n" %self.ramdump.read_u32(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'curr_pwr_state')))
        print_out_mmc("\tcurr_io_level = %d\n" %self.ramdump.read_u32(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'curr_io_level')))
        print_out_mmc("\tcaps_0 = %d\n" %self.ramdump.read_u32(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'caps_0')))
        print_out_mmc("\ttransfer_mode = %d\n" %self.ramdump.read_u32(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'transfer_mode')))
        print_out_mmc("\tclk_gating_delay = %d\n" %self.ramdump.read_u32(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'clk_gating_delay')))
        print_out_mmc("\tpm_qos_delay = %d\n" %self.ramdump.read_u32(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'pm_qos_delay')))
        print_out_mmc("\tice_clk_max = %d\n" %self.ramdump.read_u32(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'ice_clk_max')))
        print_out_mmc("\tice_clk_min = %d\n" %self.ramdump.read_u32(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'ice_clk_min')))
        print_out_mmc("\tice_clk_rate = %d\n" %self.ramdump.read_u32(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'ice_clk_rate')))
        print_out_mmc("\tsaved_tuning_phase = %d\n" %self.ramdump.read_byte(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'saved_tuning_phase')))
        print_out_mmc("\tsup_ice_clk_cnt = %d\n" %self.ramdump.read_byte(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'sup_ice_clk_cnt')))
        print_out_mmc("\tuse_14lpp_dll_reset = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'use_14lpp_dll_reset')))
        print_out_mmc("\ttuning_done = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'tuning_done')))
        print_out_mmc("\tcalibration_done = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'calibration_done')))
        print_out_mmc("\tuse_cdclp533 = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'use_cdclp533')))
        print_out_mmc("\tpwr_irq_flag = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'pwr_irq_flag')))
        print_out_mmc("\tmci_removed = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'mci_removed')))
        print_out_mmc("\trestore_dll_config = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'restore_dll_config')))
        print_out_mmc("\tuse_cdr = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'use_cdr')))
        print_out_mmc("\tupdated_ddr_cfg = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'updated_ddr_cfg')))
        print_out_mmc("\tskip_bus_bw_voting = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'skip_bus_bw_voting')))
        print_out_mmc("\tcqhci_offset_changed = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'cqhci_offset_changed')))
        print_out_mmc("\treg_store = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'reg_store')))
        vbias_skip_w = self.ramdump.field_offset('struct sdhci_msm_host', 'vbias_skip_wa')
        if vbias_skip_w:
            print_out_mmc("\tvbias_skip_wa = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host + vbias_skip_w))
        print_out_mmc("\tpltfm_init_done = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'pltfm_init_done')))
        print_out_mmc("\tfake_core_3_0v_support = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'fake_core_3_0v_support')))
        print_out_mmc("\tuse_7nm_dll = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                    self.ramdump.field_offset('struct sdhci_msm_host', 'use_7nm_dll')))
        rst_n_disable = self.ramdump.field_offset('struct sdhci_msm_host', 'rst_n_disable')
        if rst_n_disable:
            print_out_mmc("\trst_n_disable = %d\n" %self.ramdump.read_bool(self.sdhci_msm_host +
                    rst_n_disable))
        print_out_mmc("}\n")
        return

    def dump_mmc_host(self):
        if not self.mmc_host :
           print_out_mmc("\nstruct mmc_host { NULL }\n")
           return
        print_out_mmc("\nstruct mmc_host = 0x%x {\n" %self.mmc_host)
        self.mmc_request = self.ramdump.read_pointer( self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'ongoing_mrq'))
        if self.mmc_request:
            self.get_mmc_request_info()
        if self.ramdump.get_kernel_version() < (4, 19, 0):
            print_out_mmc("\tclk_gated = %d\n" %self.ramdump.read_bool(self.mmc_host +
                        self.ramdump.field_offset('struct mmc_host', 'clk_gated')))
            print_out_mmc("\tclk_requests = %d\n" %self.ramdump.read_int(self.mmc_host +
                        self.ramdump.field_offset('struct mmc_host', 'clk_requests')))
            print_out_mmc("\tclk_old = %dHz\n" %self.ramdump.read_int(self.mmc_host +
                        self.ramdump.field_offset('struct mmc_host', 'clk_old')))
            print_out_mmc("\terr_occurred = %d\n" %self.ramdump.read_bool(self.mmc_host +
                        self.ramdump.field_offset('struct mmc_host', 'err_occurred')))
        print_out_mmc("\tcap2 = 0x%x\n" %self.ramdump.read_u32(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'caps2')))
        print_out_mmc("\tocr_avail = 0x%x\n" %self.ramdump.read_u32(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'ocr_avail')))
        print_out_mmc("\tocr_avail_sdio = 0x%x\n" %self.ramdump.read_u32(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'ocr_avail_sdio')))
        print_out_mmc("\tocr_avail_sd = 0x%x\n" %self.ramdump.read_u32(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'ocr_avail_sd')))
        print_out_mmc("\tocr_avail_mmc = 0x%x\n" %self.ramdump.read_u32(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'ocr_avail_mmc')))
        print_out_mmc("\tmax_current_330 = %d\n" %self.ramdump.read_u32(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'max_current_330')))
        print_out_mmc("\tmax_current_300 = %d\n" %self.ramdump.read_u32(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'max_current_300')))
        print_out_mmc("\tmax_current_180 = %d\n" %self.ramdump.read_u32(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'max_current_180')))
        print_out_mmc("\tcaps = 0x%x\n" %self.ramdump.read_u32(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'caps')))
        print_out_mmc("\tcaps2 = 0x%x\n" %self.ramdump.read_u32(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'caps2')))
        print_out_mmc("\tdsr = 0x%x\n" %self.ramdump.read_u32(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'dsr')))
        print_out_mmc("\ttrigger_card_event = %d\n" %self.ramdump.read_bool(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'trigger_card_event')))
        print_out_mmc("\tsdio_irq_pending = %d\n" %self.ramdump.read_bool(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'sdio_irq_pending')))
        print_out_mmc("\tregulator_enabled = %d\n" %self.ramdump.read_bool(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'regulator_enabled')))
        print_out_mmc("\tcqe_enabled = %d\n" %self.ramdump.read_bool(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'cqe_enabled')))
        print_out_mmc("\tcqe_on = %d\n" %self.ramdump.read_bool(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'cqe_on')))

        if (self.ramdump.kernel_version >= (5,10,0)):
            try:
                print_out_mmc("\tcqe_recovery_reset_always = %d\n" %self.ramdump.read_bool(self.mmc_host +
                           self.ramdump.field_offset('struct mmc_host', 'cqe_recovery_reset_always')))
            except:
                print_out_mmc("\tmember tcqe_recovery_reset_alwaysnot is not included in struct mmc_host")
            print_out_mmc("\tdoing_init_tune = %d\n" %self.ramdump.read_int(self.mmc_host +
                       self.ramdump.field_offset('struct mmc_host', 'doing_init_tune')))
        print_out_mmc("\thsq_enabled = %d\n" %self.ramdump.read_bool(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'hsq_enabled')))
        print_out_mmc("\tindex = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'index')))
        print_out_mmc("\tf_min = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'f_min')))
        print_out_mmc("\tf_max = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'f_max')))
        print_out_mmc("\tf_init = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'f_init')))
        print_out_mmc("\tfixed_drv_type = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'fixed_drv_type')))
        print_out_mmc("\tmax_seg_size = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'max_seg_size')))
        print_out_mmc("\tmax_req_size = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'max_req_size')))
        print_out_mmc("\tmax_blk_size = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'max_blk_size')))
        print_out_mmc("\tmax_blk_count = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'max_blk_count')))
        print_out_mmc("\tmax_busy_timeout = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'max_busy_timeout')))
        print_out_mmc("\tuse_spi_crc = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'use_spi_crc')))
        print_out_mmc("\tclaimed = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'claimed')))
        print_out_mmc("\tcan_retune = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'can_retune')))
        print_out_mmc("\tdoing_retune = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'doing_retune')))
        print_out_mmc("\tretune_now = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'retune_now')))
        print_out_mmc("\tretune_paused = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'retune_paused')))
        print_out_mmc("\tretune_crc_disable = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'retune_crc_disable')))
        print_out_mmc("\tcan_dma_map_merge = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'can_dma_map_merge')))
        print_out_mmc("\trescan_disable = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'rescan_disable')))
        print_out_mmc("\trescan_entered = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'rescan_entered')))
        print_out_mmc("\tneed_retune = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'need_retune')))
        print_out_mmc("\thold_retune = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'hold_retune')))
        print_out_mmc("\tretune_period = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'retune_period')))
        print_out_mmc("\tclaim_cnt = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'claim_cnt')))
        print_out_mmc("\tdetect_change = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'detect_change')))
        print_out_mmc("\tsdio_irqs = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'sdio_irqs')))
        print_out_mmc("\tactual_clock = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'actual_clock')))
        print_out_mmc("\tslotno = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'slotno')))
        print_out_mmc("\tdsr_req = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'dsr_req')))
        print_out_mmc("\tcqe_qdepth = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'cqe_qdepth')))
        print_out_mmc("\tpm_caps = %d\n" %self.ramdump.read_int(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'pm_caps')))
        print_out_mmc("\tmax_segs = %d\n" %self.ramdump.read_u16(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'max_segs')))
        print_out_mmc("\tunused = %d\n" %self.ramdump.read_u16(self.mmc_host +
                    self.ramdump.field_offset('struct mmc_host', 'unused')))

        self.get_mmc_ios_info()
        self.dump_cqe_private()
        print_out_mmc("}\n")
        return

    def get_mmc_command_info(self):
        print_out_mmc("\t\t\t\t\topcode = %d\n" %self.ramdump.read_u32(self.mmc_command +
                    self.ramdump.field_offset('struct mmc_command', 'opcode')))
        print_out_mmc("\t\t\t\t\tflags = %d\n" %self.ramdump.read_int(self.mmc_command +
                    self.ramdump.field_offset('struct mmc_command', 'flags')))
        print_out_mmc("\t\t\t\t\tretries = %d\n" %self.ramdump.read_int(self.mmc_command +
                    self.ramdump.field_offset('struct mmc_command', 'retries')))
        print_out_mmc("\t\t\t\t\terror = %d\n" %self.ramdump.read_int(self.mmc_command +
                    self.ramdump.field_offset('struct mmc_command', 'error')))
        print_out_mmc("\t\t\t\t\tbusy_timeout = %d\n" %self.ramdump.read_int(self.mmc_command +
                    self.ramdump.field_offset('struct mmc_command', 'busy_timeout')))
        print_out_mmc("\t\t\t\t\targ = %d\n" %self.ramdump.read_u32(self.mmc_command +
                    self.ramdump.field_offset('struct mmc_command', 'arg')))
        print_out_mmc("\t\t\t\t\tresp[4] = %d\n" %self.ramdump.read_u32(self.mmc_command +
                    self.ramdump.field_offset('struct mmc_command', 'resp[4]')))
        print_out_mmc("\t\t\t\t}\n")

    def get_mmc_request_info(self):
        print_out_mmc("\t\t\tstruct mmc_request = 0x%x {\n" %self.mmc_request)
        print_out_mmc("\t\t\t\ttag = %d\n" %self.ramdump.read_int(self.mmc_request +
                    self.ramdump.field_offset('struct mmc_request', 'tag')))
        print_out_mmc("\t\t\t\tcrypto_key_slot = %d\n" %self.ramdump.read_int(self.mmc_request +
                    self.ramdump.field_offset('struct mmc_request', 'crypto_key_slot')))
        print_out_mmc("\t\t\t\tcap_cmd_during_tfr = %d\n" %self.ramdump.read_bool(self.mmc_request +
                    self.ramdump.field_offset('struct mmc_request', 'cap_cmd_during_tfr')))

        self.mmc_data = self.ramdump.read_pointer(self.mmc_request +
                self.ramdump.field_offset('struct mmc_request', 'data'))
        print_out_mmc("\t\t\t\tstruct mmc_data = 0x%x {\n" %self.mmc_request)
        if not self.mmc_data:
            print_out_mmc("\t\t\t\t\tNULL\n\t\t\t\t}\n")
        else :
            print_out_mmc("\t\t\t\t\ttimeout_ns = %d\n" %self.ramdump.read_int(self.mmc_data +
                        self.ramdump.field_offset('struct mmc_data', 'timeout_ns')))
            print_out_mmc("\t\t\t\t\ttimeout_clks = %d\n" %self.ramdump.read_int(self.mmc_data +
                        self.ramdump.field_offset('struct mmc_data', 'timeout_clks')))
            print_out_mmc("\t\t\t\t\tblksz = %d\n" %self.ramdump.read_int(self.mmc_data +
                        self.ramdump.field_offset('struct mmc_data', 'blksz')))
            print_out_mmc("\t\t\t\t\tblocks = %d\n" %self.ramdump.read_int(self.mmc_data +
                        self.ramdump.field_offset('struct mmc_data', 'blocks')))
            print_out_mmc("\t\t\t\t\tblk_addr = %d\n" %self.ramdump.read_int(self.mmc_data +
                        self.ramdump.field_offset('struct mmc_data', 'blk_addr')))
            print_out_mmc("\t\t\t\t\terror = %d\n" %self.ramdump.read_int(self.mmc_data +
                        self.ramdump.field_offset('struct mmc_data', 'error')))
            flags = self.ramdump.read_int(self.mmc_data +
                        self.ramdump.field_offset('struct mmc_data', 'flags'))
            print_out_mmc("\t\t\t\t\tflags = %d " %flags)
            print_set_bits(flags, req_flags_arr)
            print_out_mmc("\t\t\t\t\tbytes_xfered = %d\n" %self.ramdump.read_int(self.mmc_data +
                        self.ramdump.field_offset('struct mmc_data', 'bytes_xfered')))
            print_out_mmc("\t\t\t\t\tsg_len = %d\n" %self.ramdump.read_int(self.mmc_data +
                        self.ramdump.field_offset('struct mmc_data', 'sg_len')))
            print_out_mmc("\t\t\t\t\tsg_count = %d\n" %self.ramdump.read_int(self.mmc_data +
                        self.ramdump.field_offset('struct mmc_data', 'sg_count')))
            print_out_mmc("\t\t\t\t}\n")

        self.mmc_command = self.ramdump.read_pointer(self.mmc_request +
                self.ramdump.field_offset('struct mmc_request', 'sbc'))
        print_out_mmc("\t\t\t\tstruct mmc_command *sbc= 0x%x {\n" %self.mmc_request)
        if not self.mmc_command:
            print_out_mmc("\t\t\t\t\tNULL\n\t\t\t\t}\n")
        else :
            self.get_mmc_command_info()
        self.mmc_command = self.ramdump.read_pointer(self.mmc_request +
                self.ramdump.field_offset('struct mmc_request', 'cmd'))
        print_out_mmc("\t\t\t\tstruct mmc_command *cmd= 0x%x {\n" %self.mmc_request)
        if not self.mmc_command:
            print_out_mmc("\t\t\t\t\tNULL\n\t\t\t\t}\n")
        else :
            self.get_mmc_command_info()
        self.mmc_command = self.ramdump.read_pointer(self.mmc_request +
                self.ramdump.field_offset('struct mmc_request', 'stop'))
        print_out_mmc("\t\t\t\tstruct mmc_command *stop= 0x%x {\n" %self.mmc_request)
        if not self.mmc_command:
            print_out_mmc("\t\t\t\t\tNULL\n\t\t\t\t}\n")
        else :
            self.get_mmc_command_info()
        print_out_mmc("\t\t\t}\n")

    def dump_cqe_private(self):
        self.cqhci_host = self.ramdump.read_pointer(self.mmc_host + self.ramdump.field_offset('struct mmc_host', 'cqe_private'))
        print_out_mmc("\tstruct cqhci_host = 0x%x {\n" %self.cqhci_host)
        if not self.cqhci_host :
           print_out_mmc("\t\tNULL\n\t}\n")
           return
        var_addr = self.ramdump.field_offset('struct cqhci_host', 'slot')
        if not var_addr :
            print_out_mmc("Error: Missing cqhci_host address in dumps, Please add cqhci.ko path in --mod_path, or might issue with MMC driver\n")
            return

        slot_p = self.ramdump.read_pointer(self.cqhci_host + var_addr)

        if slot_p :
            slot_size = self.ramdump.sizeof('struct cqhci_slot')
            for x in range(32):
                slot_addr = slot_p + (x * slot_size)
                self.mmc_request = self.ramdump.read_pointer(slot_addr +
                    self.ramdump.field_offset('struct cqhci_slot', 'mrq'))
                print_out_mmc("\t\tslot [%d] = 0x%x {\n" %(x, slot_addr))
                if self.mmc_request:
                    self.get_mmc_request_info()
                    print_out_mmc("\t\t\tflags = 0x%x\n" %self.ramdump.read_int(slot_addr +
                                self.ramdump.field_offset('struct cqhci_slot', 'flags')))
                    print_out_mmc("\t\t}\n")
                else :
                    print_out_mmc("\t\t\tNULL\n\t\t}\n")

        print_out_mmc("\t\trca = %d\n" %self.ramdump.read_int(self.cqhci_host +
                    self.ramdump.field_offset('struct cqhci_host', 'rca')))
        print_out_mmc("\t\tnum_slots = %d\n" %self.ramdump.read_int(self.cqhci_host +
                    self.ramdump.field_offset('struct cqhci_host', 'num_slots')))
        print_out_mmc("\t\tqcnt = %d\n" %self.ramdump.read_int(self.cqhci_host +
                    self.ramdump.field_offset('struct cqhci_host', 'qcnt')))
        print_out_mmc("\t\tdcmd_slot = %d\n" %self.ramdump.read_u32(self.cqhci_host +
                    self.ramdump.field_offset('struct cqhci_host', 'dcmd_slot')))
        print_out_mmc("\t\tcaps = %d\n" %self.ramdump.read_u32(self.cqhci_host +
                    self.ramdump.field_offset('struct cqhci_host', 'caps')))
        print_out_mmc("\t\tquirks = %d\n" %self.ramdump.read_u32(self.cqhci_host +
                    self.ramdump.field_offset('struct cqhci_host', 'quirks')))
        print_out_mmc("\t\tcrypto_cfg_register = %d\n" %self.ramdump.read_u32(self.cqhci_host +
                    self.ramdump.field_offset('struct cqhci_host', 'crypto_cfg_register')))
        print_out_mmc("\t\tdesc_size = %d\n" %self.ramdump.read_u32(self.cqhci_host +
                    self.ramdump.field_offset('struct cqhci_host', 'desc_size')))
        print_out_mmc("\t\tdata_size = %d\n" %self.ramdump.read_u32(self.cqhci_host +
                    self.ramdump.field_offset('struct cqhci_host', 'data_size')))
        print_out_mmc("\t\tslot_sz = %d\n" %self.ramdump.read_byte(self.cqhci_host +
                    self.ramdump.field_offset('struct cqhci_host', 'slot_sz')))
        print_out_mmc("\t\ttask_desc_len = %d\n" %self.ramdump.read_byte(self.cqhci_host +
                    self.ramdump.field_offset('struct cqhci_host', 'task_desc_len')))
        print_out_mmc("\t\tlink_desc_len = %d\n" %self.ramdump.read_byte(self.cqhci_host +
                    self.ramdump.field_offset('struct cqhci_host', 'link_desc_len')))
        print_out_mmc("\t\ttrans_desc_len = %d\n" %self.ramdump.read_byte(self.cqhci_host +
                    self.ramdump.field_offset('struct cqhci_host', 'trans_desc_len')))
        print_out_mmc("\t\tdma64 = %d\n" %self.ramdump.read_bool(self.cqhci_host +
                    self.ramdump.field_offset('struct cqhci_host', 'dma64')))
        print_out_mmc("\t\tenabled = %d\n" %self.ramdump.read_bool(self.cqhci_host +
                    self.ramdump.field_offset('struct cqhci_host', 'enabled')))
        print_out_mmc("\t\thalted = %d\n" %self.ramdump.read_bool(self.cqhci_host +
                    self.ramdump.field_offset('struct cqhci_host', 'halted')))
        print_out_mmc("\t\tinit_done = %d\n" %self.ramdump.read_bool(self.cqhci_host +
                    self.ramdump.field_offset('struct cqhci_host', 'init_done')))
        print_out_mmc("\t\tactivated = %d\n" %self.ramdump.read_bool(self.cqhci_host +
                    self.ramdump.field_offset('struct cqhci_host', 'activated')))
        print_out_mmc("\t\twaiting_for_idle = %d\n" %self.ramdump.read_bool(self.cqhci_host +
                    self.ramdump.field_offset('struct cqhci_host', 'waiting_for_idle')))
        print_out_mmc("\t\trecovery_halt = %d\n" %self.ramdump.read_bool(self.cqhci_host +
                    self.ramdump.field_offset('struct cqhci_host', 'recovery_halt')))
        print_out_mmc("\t\toffset_changed = %d\n" %self.ramdump.read_bool(self.cqhci_host +
                    self.ramdump.field_offset('struct cqhci_host', 'offset_changed')))
        print_out_mmc("\t}\n")
        return

    def get_mmc_ios_info(self):
        self.ios = self.mmc_host + self.ramdump.field_offset('struct mmc_host', 'ios')
        if not self.ios :
           print_out_mmc("\tstruct mmc_ios { NULL }\n")
           return
        print_out_mmc("\tstruct mmc_ios = 0x%x {\n" %self.ios)
        print_out_mmc("\t\tclock = %dHz\n" %self.ramdump.read_int(self.ios +
                        self.ramdump.field_offset('struct mmc_ios', 'clock')))
        offset = self.ramdump.field_offset('struct mmc_ios', 'old_rate')
        if offset :
            print_out_mmc("\t\told_rate = %uHz\n" %self.ramdump.read_int(self.ios + offset))
        print_out_mmc("\t\tvdd = %d\n" %self.ramdump.read_u16(self.ios +
            self.ramdump.field_offset('struct mmc_ios', 'vdd')))
        print_out_mmc("\t\tpower_delay_ms = %d\n" %self.ramdump.read_int(self.ios +
            self.ramdump.field_offset('struct mmc_ios', 'power_delay_ms')))
        print_out_mmc("\t\tbus_mode = %d\n" %self.ramdump.read_byte(self.ios +
            self.ramdump.field_offset('struct mmc_ios', 'bus_mode')))
        print_out_mmc("\t\tchip_select = %d\n" %self.ramdump.read_byte(self.ios +
            self.ramdump.field_offset('struct mmc_ios', 'chip_select')))
        power_mode = self.ramdump.read_byte(self.ios +
        self.ramdump.field_offset('struct mmc_ios', 'power_mode'))
        if power_mode in ios_power_mode.keys():
            print_out_mmc("\t\tpower_mode = %d [%s]\n" %(power_mode,ios_power_mode[power_mode]))
        else:
            print_out_mmc("\t\tpower_mode = %d [Unknown]\n" %(power_mode))
        width = self.ramdump.read_byte(self.ios + self.ramdump.field_offset('struct mmc_ios', 'bus_width'))
        print_out_mmc("\t\tbus_width = %d [%d Buslines]\n" %(width,pow(2,width)))
        timing = self.ramdump.read_byte(self.ios + self.ramdump.field_offset('struct mmc_ios', 'timing'))
        if timing in ios_timing.keys():
            print_out_mmc("\t\ttiming = %d [%s]\n" %(timing,ios_timing[timing]))
        else:
            print_out_mmc("\t\ttiming = %d [Unknown]\n" %timing)
        print_out_mmc("\t\tsignal_voltage = %d\n" %self.ramdump.read_byte(self.ios +
                self.ramdump.field_offset('struct mmc_ios', 'signal_voltage')))
        print_out_mmc("\t\tdrv_type = %d\n" %self.ramdump.read_byte(self.ios +
                self.ramdump.field_offset('struct mmc_ios', 'drv_type')))
        print_out_mmc("\t\tenhanced_strobe = %d\n" %self.ramdump.read_byte(self.ios +
                self.ramdump.field_offset('struct mmc_ios', 'enhanced_strobe')))
        print_out_mmc("\t}\n")
        return

    def get_pm_info(self):
        power = self.dev + self.ramdump.field_offset('struct device', 'power')
        if not self.mmc_card :
           print_out_mmc("\t\tstruct dev_pm_info { NULL }\n")
           return
        print_out_mmc("\t\tstruct dev_pm_info = 0x%x {\n" %power)
        print_out_mmc("\t\t\tusage_count = %d\n" %(self.ramdump.read_int(power + self.ramdump.field_offset(
                                        'struct dev_pm_info', 'usage_count'))))
        print_out_mmc("\t\t\tchild_count = %d\n" %(self.ramdump.read_int(power + self.ramdump.field_offset(
                                        'struct dev_pm_info', 'child_count'))))
        print_out_mmc("\t\t\tautosuspend_delay = %d\n" %(self.ramdump.read_int(power + self.ramdump.field_offset(
                                        'struct dev_pm_info', 'autosuspend_delay'))))
        print_out_mmc("\t\t\trequest = %s\n" %mmc_rpm_request_l[(self.ramdump.read_int(
                                        power + self.ramdump.field_offset(
                                        'struct dev_pm_info',
                                        'request')))])
        print_out_mmc("\t\t\truntime_status = %s\n" %mmc_rpm_status_l[(self.ramdump.read_int(
                                        power + self.ramdump.field_offset(
                                        'struct dev_pm_info',
                                        'runtime_status')))])
        print_out_mmc("\t\t}\n")
        return

    def get_queue_info(self, queue):
        print_out_mmc("\t\t\tstruct request_queue = 0x%x {\n" %queue)
        print_out_mmc("\t\t\tqueue_flags = 0x%x\n" %(self.ramdump.read_u64(queue + self.ramdump.field_offset(
                                        'struct request_queue', 'queue_flags'))))
        print_out_mmc("\t\t\tpm_only = %d\n" %(self.ramdump.read_int(queue + self.ramdump.field_offset(
                                        'struct request_queue', 'pm_only'))))
        print_out_mmc("\t\t\tnr_requests = %d\n" %(self.ramdump.read_int(queue + self.ramdump.field_offset(
                                        'struct request_queue', 'nr_requests'))))
        try:
            print_out_mmc("\t\t\trpm_status = %s\n" %mmc_rpm_status_l[(self.ramdump.read_int(
                                        queue + self.ramdump.field_offset(
                                        'struct request_queue',
                                        'rpm_status')))])
        except:
            print_out_mmc("\t\t\tNo CONFIG_PM")

        if self.ramdump.get_kernel_version() > (4, 20, 0):
            hw_ctx_p = self.ramdump.read_pointer(self.ramdump.read_pointer(queue + self.ramdump.field_offset(
                                        'struct request_queue',
                                        'queue_hw_ctx')))
            print_out_mmc("\t\t\tblk_mq_hw_ctx = 0x%x\n" %hw_ctx_p)
            print_out_mmc("\t\t\tnr_active = %d\n" %(self.ramdump.read_int(queue + self.ramdump.field_offset(
                                        'struct blk_mq_hw_ctx', 'nr_active'))))
        return

    def get_mmc_blk_data(self):
        self.mmc_blk_data =  self.ramdump.read_pointer(self.dev + self.ramdump.field_offset(
                                        'struct device', 'driver_data'))
        print_out_mmc("\t\tstruct mmc_blk_data = 0x%x {\n" %self.mmc_blk_data)
        if not self.mmc_blk_data :
           print_out_mmc("\t\t\tNULL\n\t}\n")
           return
        print_out_mmc("\t\t\tflags = 0x%x\n" %(self.ramdump.read_int( self.mmc_blk_data + self.ramdump.field_offset(
                                        'struct mmc_blk_data', 'flags'))))
        queue = self.mmc_blk_data + self.ramdump.field_offset(
                                         'struct mmc_blk_data', 'queue')
        print_out_mmc("\t\tstruct mmc_queue = 0x%x {\n" %queue)
        if not queue :
           print_out_mmc("\t\tNULL\n\t}\n")
           return
        print_out_mmc("\t\t\tin_flight[MMC_ISSUE_SYNC] = %d\n" %(self.ramdump.read_int(queue + self.ramdump.field_offset(
                                        'struct mmc_queue', 'in_flight[0]'))))
        print_out_mmc("\t\t\tin_flight[MMC_ISSUE_DCMD] = %d\n" %(self.ramdump.read_int(queue + self.ramdump.field_offset(
                                        'struct mmc_queue', 'in_flight[1]'))))
        print_out_mmc("\t\t\tin_flight[MMC_ISSUE_ASYNC] = %d\n" %(self.ramdump.read_int(queue + self.ramdump.field_offset(
                                        'struct mmc_queue', 'in_flight[2]'))))
        queue_p = self.ramdump.read_pointer(queue + self.ramdump.field_offset(
                                        'struct mmc_queue', 'queue'))
        self.get_queue_info(queue_p)
        print_out_mmc("\t\t\t}\n")
        print_out_mmc("\t\t}\n")
        return

    def dump_mmc_card(self):
        if not self.mmc_card :
           print_out_mmc("\nstruct mmc_card { NULL }\n")
           return
        print_out_mmc("\nstruct mmc_card = 0x%x {\n" %self.mmc_card)
        self.dev = self.mmc_card + self.ramdump.field_offset(
                                         'struct mmc_card', 'dev')
        print_out_mmc("\tstruct device = 0x%x {\n" %self.dev)
        print_out_mmc("\t\tname: %s\n" %(self.ramdump.read_structure_cstring(
                                        self.dev + self.ramdump.field_offset(
                                        'struct device', 'kobj'),
                                        'struct kobject', 'name')))
        self.get_pm_info()
        self.get_mmc_blk_data()
        print_out_mmc("\t}\n")
        print_out_mmc("\trca = %d\n" %self.ramdump.read_int(self.mmc_card +
                    self.ramdump.field_offset('struct mmc_card', 'rca')))
        mmctype = self.ramdump.read_int(self.mmc_card +
                    self.ramdump.field_offset('struct mmc_card', 'type'))
        if mmctype in card_type.keys():
            print_out_mmc("\ttype = %d [%s]\n" %(mmctype,card_type[mmctype]))
        else:
            print_out_mmc("\ttype = %d [Unknown]\n" %mmctype)
        state = self.ramdump.read_int(self.mmc_card +
                    self.ramdump.field_offset('struct mmc_card', 'state'))
        print_out_mmc("\tstate = %d " %(state))
        print_set_bits(state,card_state)
        print_out_mmc("\tquirks = %d\n" %self.ramdump.read_int(self.mmc_card +
                    self.ramdump.field_offset('struct mmc_card', 'quirks')))
        print_out_mmc("\tquirk_max_rate = %d\n" %self.ramdump.read_int(self.mmc_card +
                    self.ramdump.field_offset('struct mmc_card', 'quirk_max_rate')))
        print_out_mmc("\terase_size = %d\n" %self.ramdump.read_int(self.mmc_card +
                    self.ramdump.field_offset('struct mmc_card', 'erase_size')))
        print_out_mmc("\terase_shift = %d\n" %self.ramdump.read_int(self.mmc_card +
                    self.ramdump.field_offset('struct mmc_card', 'erase_shift')))
        print_out_mmc("\tpref_erase = %d\n" %self.ramdump.read_int(self.mmc_card +
                    self.ramdump.field_offset('struct mmc_card', 'pref_erase')))
        print_out_mmc("\teg_boundary = %d\n" %self.ramdump.read_int(self.mmc_card +
                    self.ramdump.field_offset('struct mmc_card', 'eg_boundary')))
        print_out_mmc("\terase_arg = %d\n" %self.ramdump.read_int(self.mmc_card +
                    self.ramdump.field_offset('struct mmc_card', 'erase_arg')))
        print_out_mmc("\tsdio_funcs = %d\n" %self.ramdump.read_int(self.mmc_card +
                    self.ramdump.field_offset('struct mmc_card', 'sdio_funcs')))
        print_out_mmc("\tsd_bus_speed = %d\n" %self.ramdump.read_int(self.mmc_card +
                    self.ramdump.field_offset('struct mmc_card', 'sd_bus_speed')))
        print_out_mmc("\tmmc_avail_type = %d\n" %self.ramdump.read_int(self.mmc_card +
                    self.ramdump.field_offset('struct mmc_card', 'mmc_avail_type')))
        print_out_mmc("\tdrive_strength = %d\n" %self.ramdump.read_int(self.mmc_card +
                    self.ramdump.field_offset('struct mmc_card', 'drive_strength')))
        print_out_mmc("\tnr_parts = %d\n" %self.ramdump.read_int(self.mmc_card +
                    self.ramdump.field_offset('struct mmc_card', 'nr_parts')))
        print_out_mmc("\tocr = %d\n" %self.ramdump.read_u32(self.mmc_card +
                    self.ramdump.field_offset('struct mmc_card', 'ocr')))
        if mmctype == 1:
            print_out_mmc("\traw_scr[0] = %08x\n" %self.ramdump.read_u32(self.mmc_card +
                        self.ramdump.field_offset('struct mmc_card', 'raw_scr[0]')))
            print_out_mmc("\traw_scr[1] = %08x\n" %self.ramdump.read_u32(self.mmc_card +
                        self.ramdump.field_offset('struct mmc_card', 'raw_scr[1]')))
            for x in range(16):
                string = "raw_ssr[%d]" % (x)
                print_out_mmc("\traw_ssr[%d] = %08x\n" %(x,self.ramdump.read_u32(self.mmc_card +
                            self.ramdump.field_offset('struct mmc_card', string))))
        print_out_mmc("\terased_byte = %d\n" %self.ramdump.read_byte(self.mmc_card +
                    self.ramdump.field_offset('struct mmc_card', 'erased_byte')))
        if (self.ramdump.kernel_version >= (5,10,0)):
            print_out_mmc("\tmajor_rev = %d\n" %self.ramdump.read_byte(self.mmc_card +
                        self.ramdump.field_offset('struct mmc_card', 'major_rev')))
            print_out_mmc("\tminor_rev = %d\n" %self.ramdump.read_byte(self.mmc_card +
                        self.ramdump.field_offset('struct mmc_card', 'minor_rev')))
        print_out_mmc("\treenable_cmdq = %d\n" %self.ramdump.read_bool(self.mmc_card +
                    self.ramdump.field_offset('struct mmc_card', 'reenable_cmdq')))
        self.get_mmc_ext_csd_info()
        self.get_mmc_cid_info()
        self.get_mmc_csd_info()
        print_out_mmc("}\n")
        return

    def get_mmc_ext_csd_info(self):
        self.mmc_ext_csd = self.mmc_card + self.ramdump.field_offset('struct mmc_card', 'ext_csd')
        if not self.mmc_ext_csd :
           print_out_mmc("\tstruct mmc_ext_csd { NULL }\n")
           return
        print_out_mmc("\tstruct mmc_ext_csd = 0x%x {\n" %self.mmc_ext_csd)
        print_out_mmc("\t\trev = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'rev')))
        print_out_mmc("\t\terase_group_def = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'erase_group_def')))
        print_out_mmc("\t\tsec_feature_support = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'sec_feature_support')))
        print_out_mmc("\t\trel_sectors = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'rel_sectors')))
        print_out_mmc("\t\trel_param = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'rel_param')))
        print_out_mmc("\t\tpart_config = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'part_config')))
        print_out_mmc("\t\tcache_ctrl = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'cache_ctrl')))
        print_out_mmc("\t\trst_n_function = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'rst_n_function')))
        print_out_mmc("\t\tmax_packed_writes = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'max_packed_writes')))
        print_out_mmc("\t\tmax_packed_reads = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'max_packed_reads')))
        print_out_mmc("\t\tpacked_event_en = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'packed_event_en')))
        print_out_mmc("\t\tpower_off_notification = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'power_off_notification')))
        print_out_mmc("\t\tfwrev = 0x%x\n" %self.ramdump.read_u64(self.mmc_ext_csd +
                     self.ramdump.field_offset('struct mmc_ext_csd', 'fwrev')))
        print_out_mmc("\t\traw_exception_status = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_exception_status')))
        print_out_mmc("\t\traw_partition_support = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_partition_support')))
        print_out_mmc("\t\traw_rpmb_size_mult = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_rpmb_size_mult')))
        print_out_mmc("\t\traw_erased_mem_count = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_erased_mem_count')))
        print_out_mmc("\t\tstrobe_support = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'strobe_support')))
        print_out_mmc("\t\traw_ext_csd_structure = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_ext_csd_structure')))
        print_out_mmc("\t\traw_card_type = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_card_type')))
        print_out_mmc("\t\traw_driver_strength = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_driver_strength')))
        print_out_mmc("\t\tout_of_int_time = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'out_of_int_time')))
        print_out_mmc("\t\traw_pwr_cl_52_195 = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_pwr_cl_52_195')))
        print_out_mmc("\t\traw_pwr_cl_26_195 = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_pwr_cl_26_195')))
        print_out_mmc("\t\traw_pwr_cl_52_360 = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_pwr_cl_52_360')))
        print_out_mmc("\t\traw_pwr_cl_26_360 = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_pwr_cl_26_360')))
        print_out_mmc("\t\traw_s_a_timeout = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_s_a_timeout')))
        print_out_mmc("\t\traw_hc_erase_gap_size = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_hc_erase_gap_size')))
        print_out_mmc("\t\traw_erase_timeout_mult = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_erase_timeout_mult')))
        print_out_mmc("\t\traw_hc_erase_grp_size = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_hc_erase_grp_size')))
        if (self.ramdump.kernel_version >= (5,10,0)):
            print_out_mmc("\t\traw_boot_mult = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                        self.ramdump.field_offset('struct mmc_ext_csd', 'raw_boot_mult')))
            print_out_mmc("\t\tenhanced_rpmb_supported = %d\n" %self.ramdump.read_bool(self.mmc_ext_csd +
                        self.ramdump.field_offset('struct mmc_ext_csd', 'enhanced_rpmb_supported')))
        print_out_mmc("\t\traw_sec_trim_mult = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_sec_trim_mult')))
        print_out_mmc("\t\traw_sec_erase_mult = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_sec_erase_mult')))
        print_out_mmc("\t\traw_sec_feature_support = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_sec_feature_support')))
        print_out_mmc("\t\traw_trim_mult = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_trim_mult')))
        print_out_mmc("\t\traw_pwr_cl_200_195 = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_pwr_cl_200_195')))
        print_out_mmc("\t\traw_pwr_cl_200_360 = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_pwr_cl_200_360')))
        print_out_mmc("\t\traw_pwr_cl_ddr_52_195 = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_pwr_cl_ddr_52_195')))
        print_out_mmc("\t\traw_pwr_cl_ddr_52_360 = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_pwr_cl_ddr_52_360')))
        print_out_mmc("\t\traw_pwr_cl_ddr_200_360 = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_pwr_cl_ddr_200_360')))
        print_out_mmc("\t\traw_bkops_status = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_bkops_status')))
        print_out_mmc("\t\traw_sectors[4] = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'raw_sectors[4]')))
        print_out_mmc("\t\tpre_eol_info = %d\n" %self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'pre_eol_info')))
        print_out_mmc("\t\tdevice_life_time_est_typ_a = %d%%\n" %(self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'device_life_time_est_typ_a')) * 10))
        print_out_mmc("\t\tdevice_life_time_est_typ_b = %d%%\n" %(self.ramdump.read_byte(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'device_life_time_est_typ_b')) * 10))
        print_out_mmc("\t\tpartition_setting_completed = %d\n" %self.ramdump.read_bool(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'partition_setting_completed')))
        print_out_mmc("\t\thpi_en = %d\n" %self.ramdump.read_bool(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'hpi_en')))
        print_out_mmc("\t\thpi = %d\n" %self.ramdump.read_bool(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'hpi')))
        print_out_mmc("\t\tbkops = %d\n" %self.ramdump.read_bool(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'bkops')))
        print_out_mmc("\t\tman_bkops_en = %d\n" %self.ramdump.read_bool(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'man_bkops_en')))
        print_out_mmc("\t\tauto_bkops_en = %d\n" %self.ramdump.read_bool(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'auto_bkops_en')))
        print_out_mmc("\t\tboot_ro_lockable = %d\n" %self.ramdump.read_bool(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'boot_ro_lockable')))
        print_out_mmc("\t\tffu_capable = %d\n" %self.ramdump.read_bool(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'ffu_capable')))
        print_out_mmc("\t\tcmdq_en = %d\n" %self.ramdump.read_bool(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'cmdq_en')))
        print_out_mmc("\t\tcmdq_support = %d\n" %self.ramdump.read_bool(self.mmc_ext_csd +
                    self.ramdump.field_offset('struct mmc_ext_csd', 'cmdq_support')))
        print_out_mmc("\t}\n")
        return

    def get_mmc_cid_info(self):
        self.mmc_cid = self.mmc_card + self.ramdump.field_offset('struct mmc_card', 'cid')
        if not self.mmc_cid :
           print_out_mmc("\tstruct mmc_cid { NULL }\n")
           return
        print_out_mmc("\tstruct mmc_cid = 0x%x {\n" %self.mmc_cid)
        manfid = self.ramdump.read_int(self.mmc_cid +
                    self.ramdump.field_offset('struct mmc_cid', 'manfid'))
        if manfid in cid_manfid.keys():
            print_out_mmc("\t\tmanfid = 0x%06x [%s]\n" %(manfid,cid_manfid[manfid]))
        else:
            print_out_mmc("\t\tmanfid = 0x%06x [Unknown]\n" %manfid)
        print_out_mmc("\t\tyear = %d\n" %self.ramdump.read_u16(self.mmc_cid +
                    self.ramdump.field_offset('struct mmc_cid', 'year')))
        print_out_mmc("\t\tmonth = %d\n" %self.ramdump.read_byte(self.mmc_cid +
                    self.ramdump.field_offset('struct mmc_cid', 'month')))
        print_out_mmc("\t\tserial = 0x%08x\n" %self.ramdump.read_int(self.mmc_cid +
                    self.ramdump.field_offset('struct mmc_cid', 'serial')))
        print_out_mmc("\t\toemid = 0x%04x\n" %self.ramdump.read_u16(self.mmc_cid +
                    self.ramdump.field_offset('struct mmc_cid', 'oemid')))
        print_out_mmc("\t\tprod_name = %s\n" %self.ramdump.read_cstring(self.mmc_cid +
                    self.ramdump.field_offset('struct mmc_cid', 'prod_name'), 8))
        print_out_mmc("\t\tprv = 0x%x\n" %self.ramdump.read_byte(self.mmc_cid +
                    self.ramdump.field_offset('struct mmc_cid', 'prv')))
        print_out_mmc("\t\thwrev = 0x%x\n" %self.ramdump.read_byte(self.mmc_cid +
                    self.ramdump.field_offset('struct mmc_cid', 'hwrev')))
        print_out_mmc("\t\tfwrev = 0x%x\n" %self.ramdump.read_byte(self.mmc_cid +
                    self.ramdump.field_offset('struct mmc_cid', 'fwrev')))
        print_out_mmc("\t}\n")
        return

    def get_mmc_csd_info(self):
        self.mmc_csd = self.mmc_card + self.ramdump.field_offset('struct mmc_card', 'csd')
        if not self.mmc_csd :
           print_out_mmc("\tstruct mmc_csd { NULL }\n")
           return
        print_out_mmc("\tstruct mmc_csd = 0x%x {\n" %self.mmc_csd)
        print_out_mmc("\t\ttaac_ns = %d\n" %self.ramdump.read_int(self.mmc_csd +
                    self.ramdump.field_offset('struct mmc_csd', 'taac_ns')))
        print_out_mmc("\t\tc_size = %d\n" %self.ramdump.read_int(self.mmc_csd +
                    self.ramdump.field_offset('struct mmc_csd', 'c_size')))
        print_out_mmc("\t\tr2w_factor = %d\n" %self.ramdump.read_int(self.mmc_csd +
                    self.ramdump.field_offset('struct mmc_csd', 'r2w_factor')))
        print_out_mmc("\t\tmax_dtr = %d\n" %self.ramdump.read_int(self.mmc_csd +
                    self.ramdump.field_offset('struct mmc_csd', 'max_dtr')))
        print_out_mmc("\t\terase_size = %d\n" %self.ramdump.read_int(self.mmc_csd +
                    self.ramdump.field_offset('struct mmc_csd', 'erase_size')))
        print_out_mmc("\t\tread_blkbits = %d\n" %self.ramdump.read_int(self.mmc_csd +
                    self.ramdump.field_offset('struct mmc_csd', 'read_blkbits')))
        print_out_mmc("\t\twrite_blkbits = %d\n" %self.ramdump.read_int(self.mmc_csd +
                    self.ramdump.field_offset('struct mmc_csd', 'write_blkbits')))
        print_out_mmc("\t\tcapacity = %d\n" %self.ramdump.read_int(self.mmc_csd +
                    self.ramdump.field_offset('struct mmc_csd', 'capacity')))
        print_out_mmc("\t\tread_partial = %d\n" %self.ramdump.read_int(self.mmc_csd +
                    self.ramdump.field_offset('struct mmc_csd', 'read_partial')))
        print_out_mmc("\t\tcmdclass = %d\n" %self.ramdump.read_u16(self.mmc_csd +
                    self.ramdump.field_offset('struct mmc_csd', 'cmdclass')))
        print_out_mmc("\t\ttaac_clks = %d\n" %self.ramdump.read_u16(self.mmc_csd +
                    self.ramdump.field_offset('struct mmc_csd', 'taac_clks')))
        print_out_mmc("\t\tstructure = %d\n" %self.ramdump.read_byte(self.mmc_csd +
                    self.ramdump.field_offset('struct mmc_csd', 'structure')))
        print_out_mmc("\t\tmmca_vsn = %d\n" %self.ramdump.read_byte(self.mmc_csd +
                    self.ramdump.field_offset('struct mmc_csd', 'mmca_vsn')))

        print_out_mmc("\t}\n")
        return

    def dump_data(self):

        print_out_mmc(MMC_DATA_HEAD % self.index)
        print_out_mmc("\nList of T32 commands for MMC to access full struct details:\n")
        print_out_mmc("\tv.v (struct mmc_card*)0x%x\n" %self.mmc_card)
        print_out_mmc("\tv.v (struct sdhci_host*)0x%x\n" %self.sdhci_host)
        print_out_mmc("\tv.v (struct mmc_host*)0x%x\n" %self.mmc_host)
        print_out_mmc("\tv.v (struct sdhci_msm_host*)0x%x\n" %self.sdhci_msm_host)
        print_out_mmc("\tIf the driver is compiled as a dynamic module, they must be loaded into the T32 tool :\n")
        print_out_mmc("\tData.LOAD.Elf <ModulePath>\sdhci-msm.ko /NoCODE /NoClear /NAME sdhci_msm /reloctype 0x3\n")
        print_out_mmc("\tData.LOAD.Elf <ModulePath>\cqhci.ko /NoCODE /NoClear /NAME cqhci /reloctype 0x3\n\n")
        self.dump_mmc_card()
        self.dump_sdhci_host()
        self.dump_mmc_host()
        self.dump_sdhci_msm_host()
        self.dump_trace_buf()
        print_out_mmc(MMC_DATA_FOOT % self.index)
        return


def dump_mmc_info(ramdump, count):

    if (count == 1):
        mmc0 = MmcDataStructure(ramdump, 0)
        mmc0.dump_data()

    if (count == 2):
        mmc1 = MmcDataStructure(ramdump, 1)
        mmc1.dump_data()

    if (count == 3):
        mmc2 = MmcDataStructure(ramdump, 2)
        mmc2.dump_data()

    mmc0.parse.dumpmemreport()
    if (mmc0.parse.error):
        print_out_mmc("\n\n\n######### MMC_ERROR_INFO #########\n")
        print_out_mmc("ERROR TYPE = %s\n" %mmc0.parse.error)
        print_out_mmc("ERROR SIGN = %s\n\n" % mmc0.parse.error_line)
    return

@register_parser('--mmcdoctor', 'Generate MMC diagnose report')
class MmcDebug(RamParser):
    def parse(self):
        count_irq = 0
        setup_out_file(F_MMCDEBUG, self)
        print_out_mmc("\nKernel Version = %d.%d.%d\n\n" %self.ramdump.kernel_version)
        if self.ramdump.is_config_defined('CONFIG_SPARSE_IRQ'):
            count_irq = get_sdhci_irqs(self.ramdump)
            if (count_irq == 0):
                print_out_mmc("Error: No %s %s IRQs are Present count_irq = %d\n"
                    %(SDHCI_IRQ_NAME, SDHCI_IRQ_CD_NAME, count_irq))
                return
            dump_mmc_info(self.ramdump, count_irq)
        else:
            print_out_mmc("Error: CONFIG_SPARSE_IRQ Not Found\n")
            print_out_str("\n Could not generate MMC diagnose report\n")
            return
