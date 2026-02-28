# Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
# Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
#
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
import csv

from parser_util import register_parser, RamParser
from parsers.irqstate import IrqParse
from dmesglib import DmesgLib
from print_out import print_out_str
from linux_list import ListWalker

# Refer to the Header before changing this
VERSION = "1.0"

F_UFSIPC = "ipc_logging/ufs-qcom.txt"
F_UFSDEBUG = "ufsreport.txt"
UFS_HEADER = "__UFS_Dumps__"


def setup_out_file(path, self):
    global out_file
    try:
        out_file = self.ramdump.open_file(path, 'wb')
        out_file.write(UFS_HEADER + '__v' + VERSION + '__' + '\n\n' + '\t\t\t\t' + '-- Begin --' + '\n\n')
        return out_file
    except:
        print_out_str("could not open path {0}".format(path))
        print_out_str("Do you have write/read permissions on the path?")


def print_out_ufs(string, newline=True):
    if newline:
        out_file.write((string + '\n').encode('ascii', 'ignore'))
        return
    out_file.write((string).encode('ascii', 'ignore'))

class UfsHba():
    ufs_dev_pwr_mode_l = ['INVALID_MODE', 'UFS_ACTIVE_PWR_MODE', 'UFS_SLEEP_PWR_MODE', 'UFS_POWERDOWN_PWR_MODE']
    ufs_link_state_l = ['UIC_LINK_OFF_STATE', 'UIC_LINK_ACTIVE_STATE', 'UIC_LINK_HIBERN8_STATE']
    ufs_pm_level_l = ['UFS_PM_LVL_0', 'UFS_PM_LVL_1', 'UFS_PM_LVL_2', 'UFS_PM_LVL_3', 'UFS_PM_LVL_4', 'UFS_PM_LVL_5']
    ufs_ref_clk_l = ['REF_CLK_FREQ_19_2_MHZ', 'REF_CLK_FREQ_26_MHZ', 'REF_CLK_FREQ_38_4_MHZ', 'REF_CLK_FREQ_52_MHZ']
    ufs_clk_gating_l = ['CLKS_OFF', 'CLKS_ON', 'REQ_CLKS_OFF', 'REQ_CLKS_ON']
    ufs_bkops_status_l = ['BKOPS_STATUS_NO_OP', 'BKOPS_STATUS_NON_CRITICAL', 'BKOPS_STATUS_PERF_IMPACT',
                            'BKOPS_STATUS_CRITICAL']
    ufs_sdev_state_l = ['INVALID_MODE', 'SDEV_CREATED', 'SDEV_RUNNING', 'SDEV_CANCEL', 'SDEV_DEL', 'SDEV_QUIESCE',
                                'SDEV_TRANSPORT_OFFLINE', 'SDEV_TRANSPORT_OFFLINE', 'SDEV_BLOCK', 'SDEV_CREATED_BLOCK']
    ufs_rpm_request_l = ['RPM_REQ_NONE', 'RPM_REQ_IDLE', 'RPM_REQ_SUSPEND', 'RPM_REQ_AUTOSUSPEND', 'RPM_REQ_RESUME']
    ufs_rpm_status_l = ['RPM_ACTIVE', 'RPM_RESUMING', 'RPM_SUSPENDED', 'RPM_SUSPENDING']
    ufs_evt_bf_k515_l = ['UFS_EVT_PA_ERR', 'UFS_EVT_DL_ERR', 'UFS_EVT_NL_ERR', 'UFS_EVT_TL_ERR', 'UFS_EVT_DME_ERR',
                    'UFS_EVT_AUTO_HIBERN8_ERR', 'UFS_EVT_FATAL_ERR', 'UFS_EVT_LINK_STARTUP_FAIL', 'UFS_EVT_RESUME_ERR',
                        'UFS_EVT_SUSPEND_ERR',  'UFS_EVT_DEV_RESET','UFS_EVT_HOST_RESET', 'UFS_EVT_ABORT']
    ufs_evt_af_k515_l = ['UFS_EVT_PA_ERR', 'UFS_EVT_DL_ERR', 'UFS_EVT_NL_ERR', 'UFS_EVT_TL_ERR', 'UFS_EVT_DME_ERR',
                    'UFS_EVT_AUTO_HIBERN8_ERR', 'UFS_EVT_FATAL_ERR', 'UFS_EVT_LINK_STARTUP_FAIL', 'UFS_EVT_RESUME_ERR',
                        'UFS_EVT_SUSPEND_ERR', 'UFS_EVT_WL_SUSP_ERR' , 'UFS_EVT_WL_RES_ERR' , 'UFS_EVT_DEV_RESET',
                            'UFS_EVT_HOST_RESET', 'UFS_EVT_ABORT']

    def __init__(self, ramdump, ufs_hba_addr):
        self.ramdump = ramdump
        self.ufs_hba_addr = ufs_hba_addr

    def dump_ufs_hba_params(self):
        pass

    def get_scsi_host(self):
        scsi_host_offset = self.ramdump.field_offset('struct ufs_hba', 'host')
        return self.ramdump.read_pointer(self.ufs_hba_addr + scsi_host_offset)

    def get_scsi_device(self):
        if self.ramdump.get_kernel_version() < (6, 1, 0):
            scsi_dev_offset = self.ramdump.field_offset('struct ufs_hba', 'sdev_ufs_device')
        else:
            scsi_dev_offset = self.ramdump.field_offset('struct ufs_hba', 'ufs_device_wlun')
        return self.ramdump.read_pointer(self.ufs_hba_addr + scsi_dev_offset)

    def dump_ufs_lrbs(self):
        pass

    def dump_uic_cmd(self):
        active_uic_cmd_addr = self.ramdump.read_pointer(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'active_uic_cmd'))
        print_out_ufs('\tstruct uic_command = 0x%x {' %(active_uic_cmd_addr))
        try:
            print_out_ufs('\t\tcommand = 0x%x' %(self.ramdump.read_int(
                active_uic_cmd_addr + self.ramdump.field_offset('struct uic_command', 'command'))))
            print_out_ufs('\t\targument1 =  0x%x' % (self.ramdump.read_int(
                active_uic_cmd_addr + self.ramdump.field_offset('struct uic_command', 'argument1'))))
            print_out_ufs('\t\targument2 = 0x%x' % (self.ramdump.read_int(
                active_uic_cmd_addr + self.ramdump.field_offset('struct uic_command', 'argument2'))))
            print_out_ufs('\t\targument3 = 0x%x' % (self.ramdump.read_int(
                active_uic_cmd_addr + self.ramdump.field_offset('struct uic_command', 'argument3'))))
            print_out_ufs('\t\tcmd_active = %d' % (self.ramdump.read_int(
                active_uic_cmd_addr + self.ramdump.field_offset('struct uic_command', 'cmd_active'))))
            print_out_ufs('\t\tresult = %d' % (self.ramdump.read_int(
                active_uic_cmd_addr + self.ramdump.field_offset('struct uic_command', 'result'))))
            print_out_ufs('\t}')
        except:
            print_out_ufs('\t\tactive_uic_cmd = 0x0\n\t}')

    def dump_ufs_event_bfk515(self, ufs_stats_addr):
        print_out_ufs("\t\tufs_event {")
        ufs_evt_base = ufs_stats_addr + self.ramdump.field_offset('struct ufs_stats', 'event')
        evt_sz = self.ramdump.sizeof('struct ufs_event_hist')
        ts_sz = self.ramdump.sizeof('ktime_t')
        val_sz = self.ramdump.sizeof('u32')
        for x in range(13):
            ufs_evt_addr = ufs_evt_base + (x * evt_sz)
            ufs_evt_cnt = self.ramdump.read_int(ufs_evt_addr + self.ramdump.field_offset(
                                'struct ufs_event_hist', 'cnt'))
            print_out_ufs("\t\t\t%s:" %self.ufs_evt_bf_k515_l[x], False)
            print_out_ufs("\tCnt: %d" %ufs_evt_cnt)
            if ufs_evt_cnt != 0:
                ts_base = ufs_evt_addr + self.ramdump.field_offset('struct ufs_event_hist', 'tstamp')
                val_base = ufs_evt_addr + self.ramdump.field_offset('struct ufs_event_hist', 'val')
                for y in range(8):
                    ts_addr = ts_base + (y * ts_sz)
                    if self.ramdump.read_s64(ts_addr) != 0:
                        print_out_ufs("\t\t\t\tval: 0x%x" %self.ramdump.read_u32(val_base + (y * val_sz)), False)
                        print_out_ufs("\tts: %d" %self.ramdump.read_s64(ts_addr))

        print_out_ufs("\t\t}")

    def dump_ufs_event_afk515(self, ufs_stats_addr):
        print_out_ufs("\t\tufs_event {")
        ufs_evt_base = ufs_stats_addr + self.ramdump.field_offset('struct ufs_stats', 'event')
        evt_sz = self.ramdump.sizeof('struct ufs_event_hist')
        ts_sz = self.ramdump.sizeof('ktime_t')
        val_sz = self.ramdump.sizeof('u32')
        for x in range(15):
            ufs_evt_addr = ufs_evt_base + (x * evt_sz)
            ufs_evt_cnt = self.ramdump.read_int(ufs_evt_addr + self.ramdump.field_offset(
                                'struct ufs_event_hist', 'cnt'))
            print_out_ufs("\t\t\t%s:" %self.ufs_evt_af_k515_l[x], False)
            print_out_ufs("\tCnt: %d" %ufs_evt_cnt)
            if ufs_evt_cnt != 0:
                ts_base = ufs_evt_addr + self.ramdump.field_offset('struct ufs_event_hist', 'tstamp')
                val_base = ufs_evt_addr + self.ramdump.field_offset('struct ufs_event_hist', 'val')
                for y in range(8):
                    ts_addr = ts_base + (y * ts_sz)
                    if self.ramdump.read_s64(ts_addr) != 0:
                        print_out_ufs("\t\t\t\tval: 0x%x" %self.ramdump.read_u32(val_base + (y * val_sz)), False)
                        print_out_ufs("\tts: %d" %self.ramdump.read_s64(ts_addr))

        print_out_ufs("\t\t}")

    def dump_ufs_stats(self):
        ufs_stats_addr = self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'ufs_stats')
        print_out_ufs('\tstruct ufs_stats = 0x%x {' %(ufs_stats_addr))
        print_out_ufs('\t\tlast_intr_status = %d' %(self.ramdump.read_int(
            ufs_stats_addr + self.ramdump.field_offset('struct ufs_stats', 'last_intr_status'))))
        print_out_ufs('\t\tlast_intr_ts = %d' %(self.ramdump.read_u64(
            ufs_stats_addr + self.ramdump.field_offset('struct ufs_stats', 'last_intr_ts'))))
        print_out_ufs('\t\thibern8_exit_cnt = %d' %(self.ramdump.read_int(
            ufs_stats_addr + self.ramdump.field_offset('struct ufs_stats', 'hibern8_exit_cnt'))))
        print_out_ufs('\t\tlast_hibern8_exit_tstamp = %d' % (self.ramdump.read_u64(
            ufs_stats_addr + self.ramdump.field_offset('struct ufs_stats', 'last_hibern8_exit_tstamp'))))
        try:
            # GKI doesn't have these
            print_out_ufs('\t\tenabled = %d' % (self.ramdump.read_bool(
                ufs_stats_addr + self.ramdump.field_offset('struct ufs_stats', 'enabled'))))
            print_out_ufs('\t\tpa_err_cnt_total = %d' %(self.ramdump.read_int(
                ufs_stats_addr + self.ramdump.field_offset('struct ufs_stats', 'pa_err_cnt_total'))))
            print_out_ufs('\t\tdl_err_cnt_total = %d' % (self.ramdump.read_int(
                ufs_stats_addr + self.ramdump.field_offset('struct ufs_stats', 'dl_err_cnt_total'))))
            print_out_ufs('\t\tdme_err_cnt = %d' % (self.ramdump.read_int(
                ufs_stats_addr + self.ramdump.field_offset('struct ufs_stats', 'dme_err_cnt'))))
            print_out_ufs('\t\tlast_intr_status = 0x%x' % (self.ramdump.read_int(
                ufs_stats_addr + self.ramdump.field_offset('struct ufs_stats', 'last_intr_status'))))
            print_out_ufs('\t\tlast_intr_ts = %d' % (self.ramdump.read_s64(
                ufs_stats_addr + self.ramdump.field_offset('struct ufs_stats', 'last_intr_ts'))))
        except:
            print_out_ufs('\t\tenabled = 0')

        if self.ramdump.get_kernel_version() < (5, 14, 0):
            self.dump_ufs_event_bfk515(ufs_stats_addr)
        else:
            self.dump_ufs_event_afk515(ufs_stats_addr)

        print_out_ufs("\t}")

    def dump_ufs_dev_info(self):
        dev_info_addr = self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'dev_info')
        print_out_ufs('\tstruct ufs_dev_info = 0x%x {' %(dev_info_addr))
        print_out_ufs('\t\tf_power_on_wp_en = %d' % (self.ramdump.read_bool(
            dev_info_addr + self.ramdump.field_offset('struct ufs_dev_info', 'f_power_on_wp_en'))))
        print_out_ufs('\t\tis_lu_power_on_wp = %d' % (self.ramdump.read_bool(
            dev_info_addr + self.ramdump.field_offset('struct ufs_dev_info', 'is_lu_power_on_wp'))))
        print_out_ufs('\t\tmodel = %s' % (self.ramdump.read_structure_cstring(
            dev_info_addr, 'struct ufs_dev_info', 'model')))
        print_out_ufs('\t\twspecversion = %d' % (self.ramdump.read_int(
            dev_info_addr + self.ramdump.field_offset('struct ufs_dev_info', 'wspecversion'))))
        print_out_ufs('\t\tclk_gating_wait_us = %d' % (self.ramdump.read_int(
            dev_info_addr + self.ramdump.field_offset('struct ufs_dev_info', 'clk_gating_wait_us'))))
        print_out_ufs("\t}")

    def dump_ufs_pa_layer_attr(self, drp_addr):
        print_out_ufs("\t\tgear_rx = %d" % (self.ramdump.read_int(
            drp_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'gear_rx'))))
        print_out_ufs("\t\tgear_tx = %d" % (self.ramdump.read_int(
            drp_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'gear_tx'))))
        print_out_ufs("\t\tlane_rx = %d" % (self.ramdump.read_int(
            drp_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'lane_rx'))))
        print_out_ufs("\t\tlane_tx = %d" % (self.ramdump.read_int(
            drp_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'lane_tx'))))
        print_out_ufs("\t\tpwr_rx = %d" % (self.ramdump.read_int(
            drp_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'pwr_rx'))))
        print_out_ufs("\t\tpwr_tx = %d" % (self.ramdump.read_int(
            drp_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'pwr_tx'))))
        print_out_ufs("\t\ths_rate = %d" % (self.ramdump.read_int(
            drp_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'hs_rate'))))
        return

    def get_vreg_info(self, vreg_addr, vreg):
        vreg_p = self.ramdump.read_pointer(vreg_addr + self.ramdump.field_offset('struct ufs_vreg_info', vreg))
        if vreg_p == 0:
            return
        print_out_ufs("\t\t[%s]{"%vreg)
        print_out_ufs("\t\t\taddr = 0x%x" %vreg_p)
        print_out_ufs("\t\t\tname = %s" %(self.ramdump.read_structure_cstring(
                                        vreg_p, 'struct ufs_vreg', 'name')))
        print_out_ufs("\t\t\talways_on = %d" %(self.ramdump.read_bool(vreg_p +
                                        self.ramdump.field_offset(
                                        'struct ufs_vreg',
                                        'always_on'))))
        print_out_ufs("\t\t\tenabled = %d" %(self.ramdump.read_bool(vreg_p +
                                        self.ramdump.field_offset(
                                        'struct ufs_vreg',
                                        'enabled'))))
        reg_p = self.ramdump.read_pointer(vreg_p + self.ramdump.field_offset('struct ufs_vreg', 'reg'))
        rdev_p = self.ramdump.read_pointer(reg_p + self.ramdump.field_offset('struct regulator', 'rdev'))
        rdesc_p = self.ramdump.read_pointer(rdev_p + self.ramdump.field_offset('struct regulator_dev', 'desc'))
        print_out_ufs("\t\t\tregulator: %s" %(self.ramdump.read_structure_cstring(
                                        rdesc_p, 'struct regulator_desc', 'name')))
        print_out_ufs("\t\t\tsupply: %s" %(self.ramdump.read_structure_cstring(
                                        rdesc_p, 'struct regulator_desc', 'supply_name')))
        print_out_ufs("\t\t}")

    def dump_ufs_vreg(self):
        print_out_ufs("\tstruct ufs_vreg {")
        vreg_addr = self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'vreg_info')
        self.get_vreg_info(vreg_addr, "vcc")
        self.get_vreg_info(vreg_addr, "vccq")
        self.get_vreg_info(vreg_addr, "vccq2")
        self.get_vreg_info(vreg_addr, "vdd_hba")
        print_out_ufs("\t}")

    def get_clk_info(self, clk):
        print_out_ufs("\t\t[%s]" %(self.ramdump.read_structure_cstring(
            clk, 'struct ufs_clk_info', 'name')))
        print_out_ufs("\t\t\taddr = 0x%x" %clk)
        print_out_ufs("\t\t\tmax_freq = %d" % (self.ramdump.read_u32(clk +
                                        self.ramdump.field_offset(
                                        'struct ufs_clk_info',
                                        'max_freq'))))
        print_out_ufs("\t\t\tmin_freq = %d" % (self.ramdump.read_u32(clk +
                                        self.ramdump.field_offset(
                                        'struct ufs_clk_info',
                                        'min_freq'))))
        print_out_ufs("\t\t\tcurr_freq = %d" % (self.ramdump.read_u32(clk +
                                        self.ramdump.field_offset(
                                        'struct ufs_clk_info',
                                        'curr_freq'))))
        print_out_ufs("\t\t\tkeep_link_active = %d" % (self.ramdump.read_bool(clk +
                                        self.ramdump.field_offset(
                                        'struct ufs_clk_info',
                                        'keep_link_active'))))
        print_out_ufs("\t\t\tenabled = %d" % (self.ramdump.read_bool(clk +
                                        self.ramdump.field_offset(
                                        'struct ufs_clk_info',
                                        'enabled'))))
        print_out_ufs("\t\t}")

    def dump_ufs_clk(self):
        print_out_ufs("\tstruct ufs_clk_inifo {")
        clk_list_h = self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'clk_list_head')
        clk_base_p = self.ramdump.read_pointer(clk_list_h + self.ramdump.field_offset('struct list_head', 'next'))
        clk_next_p = self.ramdump.read_pointer(clk_base_p + self.ramdump.field_offset('struct list_head', 'next'))
        self.get_clk_info(clk_base_p)
        while clk_next_p != clk_list_h:
            self.get_clk_info(clk_next_p)
            clk_next_p = self.ramdump.read_pointer(clk_next_p + self.ramdump.field_offset('struct list_head', 'next'))
        print_out_ufs("\t}")

    def get_scsi_cmd(self, lrbp):
        try:
            scsi_cmd_offset = self.ramdump.field_offset('struct ufshcd_lrb', 'cmd')
            scsi_cmd_addr = self.ramdump.read_pointer(scsi_cmd_offset + lrbp)
            print_out_ufs("\t\t\tcmd = %s" %(hex(ord(self.ramdump.read_structure_cstring(
                scsi_cmd_addr, 'struct scsi_cmnd', 'cmnd')))))
        except:
            print_out_ufs("\t\t\tcmd = None")
            return

    def dump_ufs_lrb(self):
        lrb_p = self.ramdump.read_pointer(self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'lrb'))
        lrb_sz = self.ramdump.sizeof('struct ufshcd_lrb')
        print_out_ufs("\tstruct ufshcd_lrb {")
        for x in range(32):
            print_out_ufs("\t\t[%d] {" %(x))
            lrb_addr = lrb_p + (x * lrb_sz)
            self.get_scsi_cmd(lrb_addr)

            if self.ramdump.get_kernel_version() < (6, 1, 0):
                print_out_ufs("\t\t\tsense_bufflen = %d" %(self.ramdump.read_int(lrb_addr +
                                                       self.ramdump.field_offset('struct ufshcd_lrb', 'sense_bufflen'))))

            print_out_ufs("\t\t\tscsi_status = %d" % (self.ramdump.read_int(lrb_addr +
                                                    self.ramdump.field_offset('struct ufshcd_lrb', 'scsi_status'))))
            print_out_ufs("\t\t\tcommand_type = %d" % (self.ramdump.read_int(lrb_addr +
                                                                          self.ramdump.field_offset('struct ufshcd_lrb',
                                                                                                    'command_type'))))
            print_out_ufs("\t\t\ttask_tag = %d" % (self.ramdump.read_int(lrb_addr +
                                                                           self.ramdump.field_offset(
                                                                               'struct ufshcd_lrb',
                                                                               'task_tag'))))
            print_out_ufs("\t\t\tlun = %d" % (self.ramdump.read_byte(lrb_addr +
                                                                       self.ramdump.field_offset(
                                                                           'struct ufshcd_lrb',
                                                                           'lun'))))
            print_out_ufs("\t\t\tintr_cmd = %d" % (self.ramdump.read_int(lrb_addr +
                                                                   self.ramdump.field_offset(
                                                                       'struct ufshcd_lrb',
                                                                       'intr_cmd'))))
            print_out_ufs("\t\t\tissue_time_stamp = %d" % (self.ramdump.read_u64(lrb_addr +
                                                                   self.ramdump.field_offset(
                                                                       'struct ufshcd_lrb',
                                                                       'issue_time_stamp'))))
            print_out_ufs("\t\t\tcompl_time_stamp = %d" % (self.ramdump.read_u64(lrb_addr +
                                                                               self.ramdump.field_offset(
                                                                                   'struct ufshcd_lrb',
                                                                                   'compl_time_stamp'))))

            print_out_ufs("\t\t}")
        print_out_ufs("\t}")

    def get_pm_info(self, power):
        print_out_ufs("\t\t\tusage_count = %d" %(self.ramdump.read_int(power + self.ramdump.field_offset(
                                        'struct dev_pm_info', 'usage_count'))))
        print_out_ufs("\t\t\tchild_count = %d" %(self.ramdump.read_int(power + self.ramdump.field_offset(
                                        'struct dev_pm_info', 'child_count'))))
        print_out_ufs("\t\t\tautosuspend_delay = %d" %(self.ramdump.read_int(power + self.ramdump.field_offset(
                                        'struct dev_pm_info', 'autosuspend_delay'))))
        print_out_ufs("\t\t\trequest = %s" %self.ufs_rpm_request_l[(self.ramdump.read_int(
                                        power + self.ramdump.field_offset(
                                        'struct dev_pm_info',
                                        'request')))])
        print_out_ufs("\t\t\truntime_status = %s" %self.ufs_rpm_status_l[(self.ramdump.read_int(
                                        power + self.ramdump.field_offset(
                                        'struct dev_pm_info',
                                        'runtime_status')))])

    def get_queue_info(self, queue):
        print_out_ufs("\t\t\taddr = 0x%x" %queue)
        print_out_ufs("\t\t\tqueue_flags = 0x%x" %(self.ramdump.read_u64(queue + self.ramdump.field_offset(
                                        'struct request_queue', 'queue_flags'))))
        print_out_ufs("\t\t\tpm_only = %d" %(self.ramdump.read_int(queue + self.ramdump.field_offset(
                                        'struct request_queue', 'pm_only'))))
        try:
            print_out_ufs("\t\t\tnr_pending = %d" %(self.ramdump.read_int(queue + self.ramdump.field_offset(
                                        'struct request_queue', 'nr_pending'))))
            print_out_ufs("\t\t\trpm_status = %s" %self.ufs_rpm_status_l[(self.ramdump.read_int(
                                        queue + self.ramdump.field_offset(
                                        'struct request_queue',
                                        'rpm_status')))])
        except:
            print_out_ufs("\t\t\tNo CONFIG_PM")

        if self.ramdump.get_kernel_version() > (4, 20, 0):
            hw_ctx_p = self.ramdump.read_pointer(self.ramdump.read_pointer(queue + self.ramdump.field_offset(
                                        'struct request_queue',
                                        'queue_hw_ctx')))
            print_out_ufs("\t\t\tblk_mq_hw_ctx = 0x%x" %hw_ctx_p)
            print_out_ufs("\t\t\tnr_active = %d" %(self.ramdump.read_int(queue + self.ramdump.field_offset(
                                        'struct blk_mq_hw_ctx', 'nr_active'))))

    def get_sdev_info(self, sdev):
        print_out_ufs("\tsdev [%d] {" %(self.ramdump.read_u64(sdev + self.ramdump.field_offset(
                                        'struct scsi_device', 'lun'))))
        print_out_ufs("\t\taddr = 0x%x" %sdev)
        if self.ramdump.get_kernel_version() < (5, 14, 0):
            print_out_ufs("\t\tdevice_busy = %d" %(self.ramdump.read_int(sdev + self.ramdump.field_offset(
                                            'struct scsi_device', 'device_busy'))))
        print_out_ufs("\t\tdevice_blocked = %d" %(self.ramdump.read_int(sdev + self.ramdump.field_offset(
                                        'struct scsi_device', 'device_blocked'))))
        print_out_ufs("\t\tsdev_state = %s" %self.ufs_sdev_state_l[(self.ramdump.read_int(
                                        sdev + self.ramdump.field_offset(
                                        'struct scsi_device',
                                        'sdev_state')))])

        try:
            sdev_gendev_addr = sdev + self.ramdump.field_offset(
                                         'struct scsi_device', 'sdev_gendev')
            print_out_ufs("\t\tname: %s" %(self.ramdump.read_structure_cstring(
                                        sdev_gendev_addr+self.ramdump.field_offset(
                                        'struct device', 'kobj'),
                                        'struct kobject', 'name')))
            power_addr = sdev_gendev_addr + self.ramdump.field_offset(
                                         'struct device', 'power')
            print_out_ufs("\t\tpm{")
            self.get_pm_info(power_addr)
        except:
            print_out_ufs("\t\t\tdevice or power = NULL")
        print_out_ufs("\t\t}")

        print_out_ufs("\t\trequest_queue{")
        try:
            queue_p = self.ramdump.read_pointer(sdev + self.ramdump.field_offset(
                                        'struct scsi_device', 'request_queue'))
            self.get_queue_info(queue_p)
        except:
            print_out_ufs("\t\trequest_queue = NULL")
        print_out_ufs("\t\t}")

        print_out_ufs("\t}")

    def dump_sdev_info(self):
        shost_p = self.ramdump.read_pointer(self.ufs_hba_addr + self.ramdump.field_offset(
                                        'struct ufs_hba', 'host'))
        sdev_list_addr = shost_p + self.ramdump.field_offset(
                                        'struct Scsi_Host', '__devices')
        sdev_list_first = self.ramdump.read_pointer(sdev_list_addr + self.ramdump.field_offset(
                                        'struct list_head', 'next'))
        sdev_list_next = self.ramdump.read_pointer(sdev_list_first + self.ramdump.field_offset(
                                        'struct list_head', 'next'))
        sdev_base_p = self.get_scsi_device()
        sdev_offest = sdev_list_first - sdev_base_p
        self.get_sdev_info(sdev_base_p)

        while sdev_list_next != sdev_list_addr:
            self.get_sdev_info(sdev_list_next - sdev_offest)
            sdev_list_next = self.ramdump.read_pointer(sdev_list_next + self.ramdump.field_offset(
                                        'struct list_head', 'next'))

    def dump_ufs_hba_params(self):
        print_out_ufs("struct ufs_hba = 0x%x {" % (self.ufs_hba_addr))

        if self.ramdump.get_kernel_version() < (6, 1, 0):
            ufs_scsi_device_addr = self.ramdump.read_pointer(
                                self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'sdev_ufs_device'))
        else:
            ufs_scsi_device_addr = self.ramdump.read_pointer(
                                self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'ufs_device_wlun'))

        print_out_ufs("\tvendor = %s" %(self.ramdump.read_structure_cstring(
                                        ufs_scsi_device_addr,'struct scsi_device', 'vendor')))
        print_out_ufs("\tmodel = %s" %(self.ramdump.read_structure_cstring(
                                        ufs_scsi_device_addr, 'struct scsi_device', 'model')))
        print_out_ufs("\trev = %s" %(self.ramdump.read_structure_cstring(
                                        ufs_scsi_device_addr, 'struct scsi_device', 'rev')))
        ufs_dev_pwr_mode = self.ramdump.read_int(
                            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'curr_dev_pwr_mode'))
        print_out_ufs("\tcurr_dev_pwr_mode = %s" %(self.ufs_dev_pwr_mode_l[ufs_dev_pwr_mode]))
        ufs_link_state = self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'uic_link_state'))
        print_out_ufs("\tuic_link_state = %s" % (self.ufs_link_state_l[ufs_link_state]))
        ufs_pm_level = self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'rpm_lvl'))
        print_out_ufs("\trpm_lvl = %s" % (self.ufs_pm_level_l[ufs_pm_level]))
        ufs_pm_level = self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'spm_lvl'))
        print_out_ufs("\tspm_lvl = %s" % (self.ufs_pm_level_l[ufs_pm_level]))
        print_out_ufs("\tpm_op_in_progress = %d" %(self.ramdump.read_int(
                            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'pm_op_in_progress'))))
        print_out_ufs("\tahit = 0x%x" % (self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'ahit'))))
        if self.ramdump.get_kernel_version() < (5, 10, 0):
            print_out_ufs("\tlrb_in_use = 0x%x" % (self.ramdump.read_int(
                self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'lrb_in_use'))))
        print_out_ufs("\toutstanding_tasks = 0x%x" % (self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'outstanding_tasks'))))
        print_out_ufs("\toutstanding_reqs = 0x%x" % (self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'outstanding_reqs'))))
        print_out_ufs("\tcapabilities = 0x%x" % (self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'capabilities'))))
        print_out_ufs("\tnutrs = %d" % (self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'nutrs'))))
        print_out_ufs("\tnutmrs = %d" % (self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'nutmrs'))))
        print_out_ufs("\tufs_version = 0x%x" % (self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'ufs_version'))))
        print_out_ufs("\tsg_entry_size = %d" % (self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'sg_entry_size'))))
        print_out_ufs("\tirq = %d" % (self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'irq'))))
        print_out_ufs("\tis_irq_enabled = %d" % (self.ramdump.read_bool(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'is_irq_enabled'))))
        dev_ref_clk_freq = self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'dev_ref_clk_freq'))
        print_out_ufs("\tdev_ref_clk_freq = %s" % (self.ufs_ref_clk_l[dev_ref_clk_freq]))
        print_out_ufs("\tquirks = 0x%x" % (self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'quirks'))))
        print_out_ufs("\tdev_quirks = 0x%x" % (self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'dev_quirks'))))
        if self.ramdump.get_kernel_version() < (5, 10, 0):
            print_out_ufs("\ttm_condition = 0x%x" % (self.ramdump.read_ulong(
                self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'tm_condition'))))
            print_out_ufs("\ttm_slots_in_use = 0x%x" % (self.ramdump.read_ulong(
                self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'tm_slots_in_use'))))
        self.dump_uic_cmd()
        print_out_ufs("\tufshcd_state = %d" % (self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'ufshcd_state'))))
        print_out_ufs("\teh_flags = %d" % (self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'eh_flags'))))
        print_out_ufs("\tintr_mask = 0x%x" % (self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'intr_mask'))))
        print_out_ufs("\tee_ctrl_mask = 0x%x" % (self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'ee_ctrl_mask'))))
        print_out_ufs("\tis_powered = %d" % (self.ramdump.read_bool(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'is_powered'))))
        print_out_ufs("\terrors = %d" % (self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'errors'))))
        print_out_ufs("\tuic_error = %d" % (self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'uic_error'))))
        print_out_ufs("\tsaved_err = %d" % (self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'saved_err'))))
        print_out_ufs("\tsaved_uic_err = %d" % (self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'saved_uic_err'))))
        self.dump_ufs_stats()
        print_out_ufs("\tsilence_err_logs = %d" % (self.ramdump.read_bool(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'silence_err_logs'))))
        print_out_ufs("\tlast_dme_cmd_tstamp = %d" % (self.ramdump.read_u64(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'last_dme_cmd_tstamp'))))
        self.dump_ufs_dev_info()
        print_out_ufs("\tauto_bkops_enabled = %d" % (self.ramdump.read_bool(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'auto_bkops_enabled'))))
        if self.ramdump.get_kernel_version() < (5, 14, 0):
            print_out_ufs("\twlun_dev_clr_ua = %d" % (self.ramdump.read_bool(
                    self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'wlun_dev_clr_ua'))))
        print_out_ufs("\treq_abort_count = %d" % (self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'req_abort_count'))))
        print_out_ufs("\tlanes_per_direction = %d" % (self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'lanes_per_direction'))))
        # PA Layer Attributes
        drp_offset = self.ramdump.field_offset('struct ufs_hba', 'pwr_info')
        drp_addr = self.ufs_hba_addr + drp_offset
        print_out_ufs("\tpwr_info = 0x%x {" %(drp_addr))
        self.dump_ufs_pa_layer_attr(drp_addr)
        print_out_ufs("\t}")
        # Power Mode Info
        mpi_offset = self.ramdump.field_offset('struct ufs_hba', 'max_pwr_info')
        mpi_addr = self.ufs_hba_addr + mpi_offset
        is_valid = self.ramdump.read_bool(mpi_addr + self.ramdump.field_offset('struct ufs_pwr_mode_info', 'is_valid'))
        if is_valid == 1:
            print_out_ufs("\tmax_pwr_info = 0x%x {" %(mpi_addr))
            print_out_ufs("\t\tis_valid = %d" %(is_valid))
            self.dump_ufs_pa_layer_attr(mpi_addr + self.ramdump.field_offset('struct ufs_pwr_mode_info', 'info'))
            print_out_ufs("\t}")
        # Clock Gating
        cg_addr = self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'clk_gating')
        print_out_ufs("\tufs_clk_gating = 0x%x {" %(cg_addr))
        print_out_ufs("\t\tstate = %s" % (self.ufs_clk_gating_l[(self.ramdump.read_int(
            cg_addr + self.ramdump.field_offset('struct ufs_clk_gating', 'state')))]))
        print_out_ufs("\t\tdelay_ms = %d" % (self.ramdump.read_ulong(
            cg_addr + self.ramdump.field_offset('struct ufs_clk_gating', 'delay_ms'))))
        print_out_ufs("\t\tis_suspended = %d" % (self.ramdump.read_bool(
            cg_addr + self.ramdump.field_offset('struct ufs_clk_gating', 'is_suspended'))))
        print_out_ufs("\t\tis_enabled = %d" % (self.ramdump.read_bool(
            cg_addr + self.ramdump.field_offset('struct ufs_clk_gating', 'is_enabled'))))
        print_out_ufs("\t\tactive_reqs = %d" % (self.ramdump.read_int(
            cg_addr + self.ramdump.field_offset('struct ufs_clk_gating', 'active_reqs'))))
        print_out_ufs("\t}")
        # Clock Scaling
        cs_addr = self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'clk_scaling')
        print_out_ufs("\tufs_clk_scaling = 0x%x {" % (cs_addr))
        print_out_ufs("\t\tactive_reqs = %d" % (self.ramdump.read_int(
            cs_addr + self.ramdump.field_offset('struct ufs_clk_scaling', 'active_reqs'))))
        print_out_ufs("\t\ttot_busy_t = %d" % (self.ramdump.read_ulong(
            cs_addr + self.ramdump.field_offset('struct ufs_clk_scaling', 'tot_busy_t'))))
        print_out_ufs("\t\twindow_start_t = %d" % (self.ramdump.read_ulong(
            cs_addr + self.ramdump.field_offset('struct ufs_clk_scaling', 'window_start_t'))))
        print_out_ufs("\t\tbusy_start_t = %d" % (self.ramdump.read_u64(
            cs_addr + self.ramdump.field_offset('struct ufs_clk_scaling', 'busy_start_t'))))
        spi_addr = cs_addr + self.ramdump.field_offset('struct ufs_clk_scaling', 'saved_pwr_info')
        print_out_ufs("\t\tstruct ufs_saved_pwr_info = 0x%x {" %(spi_addr))
        if self.ramdump.get_kernel_version() < (6, 5, 0):
            drp_addr = spi_addr + self.ramdump.field_offset('struct ufs_saved_pwr_info', 'info')
            print_out_ufs("\t\t\tgear_rx = %d" % (self.ramdump.read_int(
                drp_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'gear_rx'))))
            print_out_ufs("\t\t\tgear_tx = %d" % (self.ramdump.read_int(
                drp_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'gear_tx'))))
            print_out_ufs("\t\t\tlane_rx = %d" % (self.ramdump.read_int(
                drp_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'lane_rx'))))
            print_out_ufs("\t\t\tlane_tx = %d" % (self.ramdump.read_int(
                drp_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'lane_tx'))))
            print_out_ufs("\t\t\tpwr_rx = %d" % (self.ramdump.read_int(
                drp_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'pwr_rx'))))
            print_out_ufs("\t\t\tpwr_tx = %d" % (self.ramdump.read_int(
                drp_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'pwr_tx'))))
            print_out_ufs("\t\t\ths_rate = %d" % (self.ramdump.read_int(
                drp_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'hs_rate'))))
            print_out_ufs("\t\t}")
            print_out_ufs("\t\tis_valid = %d" % (self.ramdump.read_bool(
                cs_addr + self.ramdump.field_offset('struct ufs_saved_pwr_info', 'is_valid'))))
        else:
            print_out_ufs("\t\t\tgear_rx = %d" % (self.ramdump.read_int(
                spi_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'gear_rx'))))
            print_out_ufs("\t\t\tgear_tx = %d" % (self.ramdump.read_int(
                spi_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'gear_tx'))))
            print_out_ufs("\t\t\tlane_rx = %d" % (self.ramdump.read_int(
                spi_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'lane_rx'))))
            print_out_ufs("\t\t\tlane_tx = %d" % (self.ramdump.read_int(
                spi_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'lane_tx'))))
            print_out_ufs("\t\t\tpwr_rx = %d" % (self.ramdump.read_int(
                spi_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'pwr_rx'))))
            print_out_ufs("\t\t\tpwr_tx = %d" % (self.ramdump.read_int(
                spi_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'pwr_tx'))))
            print_out_ufs("\t\t\ths_rate = %d" % (self.ramdump.read_int(
                spi_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'hs_rate'))))
            print_out_ufs("\t\t}")
        print_out_ufs("\t\tis_allowed = %d" % (self.ramdump.read_bool(
            cs_addr + self.ramdump.field_offset('struct ufs_clk_scaling', 'is_allowed'))))
        print_out_ufs("\t\tis_busy_started = %d" % (self.ramdump.read_bool(
            cs_addr + self.ramdump.field_offset('struct ufs_clk_scaling', 'is_busy_started'))))
        print_out_ufs("\t\tis_suspended = %d" % (self.ramdump.read_bool(
            cs_addr + self.ramdump.field_offset('struct ufs_clk_scaling', 'is_suspended'))))
        print_out_ufs("\t}")
        print_out_ufs("\tis_sys_suspended = %d" % (self.ramdump.read_bool(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'is_sys_suspended'))))
        print_out_ufs("\turgent_bkops_lvl = %s" % (self.ufs_bkops_status_l[(self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'urgent_bkops_lvl')))]))
        print_out_ufs("\tis_urgent_bkops_lvl_checked = %d" % (self.ramdump.read_bool(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'is_urgent_bkops_lvl_checked'))))
        print_out_ufs("\tscsi_block_reqs_cnt = %d" % (self.ramdump.read_int(
            self.ufs_hba_addr + self.ramdump.field_offset('struct ufs_hba', 'scsi_block_reqs_cnt'))))
        self.dump_ufs_vreg()
        self.dump_ufs_clk()
        self.dump_ufs_lrb()
        self.dump_sdev_info()
        print_out_ufs("}")
        return

class UfsQcHost():

    def __init__(self, ramdump, ufs_qc_host_addr):
        self.ramdump = ramdump
        self.ufs_qc_host_addr = ufs_qc_host_addr

    def get_qphy_vreg_info(self, qphy_vreg_addr):
        if qphy_vreg_addr == 0:
            return
        reg_p = self.ramdump.read_pointer(qphy_vreg_addr + self.ramdump.field_offset(
                                        'struct ufs_qcom_phy_vreg', 'reg'))
        if reg_p == 0:
            return

        print_out_ufs("\t\tphy-vreg-%s{" %(self.ramdump.read_structure_cstring(
                                        qphy_vreg_addr,
                                        'struct ufs_qcom_phy_vreg',
                                        'name')))
        rdev_p = self.ramdump.read_pointer(reg_p + self.ramdump.field_offset(
                                        'struct regulator', 'rdev'))
        rdesc_p = self.ramdump.read_pointer(rdev_p + self.ramdump.field_offset(
                                        'struct regulator_dev','desc'))
        print_out_ufs("\t\t\tregulator: %s" %(self.ramdump.read_structure_cstring(
                                        rdesc_p, 'struct regulator_desc', 'name')))
        print_out_ufs("\t\t\tsupply: %s" %(self.ramdump.read_structure_cstring(
                                        rdesc_p,
                                        'struct regulator_desc',
                                        'supply_name')))
        print_out_ufs("\t\t\tenabled = %d" % (self.ramdump.read_bool(qphy_vreg_addr +
                                        self.ramdump.field_offset(
                                        'struct ufs_qcom_phy_vreg',
                                        'enabled'))))
        print_out_ufs("\t\t}")

    def dump_ufs_phy(self):
        gphy_p = self.ramdump.read_pointer(self.ufs_qc_host_addr + self.ramdump.field_offset(
                                        'struct ufs_qcom_host',
                                        'generic_phy'))
        dev_addr = gphy_p + self.ramdump.field_offset('struct phy', 'dev')
        qphy_p = self.ramdump.read_pointer(dev_addr + self.ramdump.field_offset(
                                        'struct device',
                                        'driver_data'))
        print_out_ufs("\tufs_phy: %s {" %(self.ramdump.read_cstring(
                                        qphy_p + self.ramdump.field_offset(
                                        'struct ufs_qcom_phy',
                                        'name'))))
        print_out_ufs("\t\tiface_clk_enabled = %d" % (self.ramdump.read_bool(qphy_p +
                                        self.ramdump.field_offset(
                                        'struct ufs_qcom_phy',
                                        'is_iface_clk_enabled'))))
        print_out_ufs("\t\tref_clk_enabled = %d" % (self.ramdump.read_bool(qphy_p +
                                        self.ramdump.field_offset(
                                        'struct ufs_qcom_phy',
                                        'is_ref_clk_enabled'))))
        print_out_ufs("\t\tdev_ref_clk_enabled = %d" % (self.ramdump.read_bool(qphy_p +
                                        self.ramdump.field_offset(
                                        'struct ufs_qcom_phy',
                                        'is_dev_ref_clk_enabled'))))
        print_out_ufs("\t\tquirks = 0x%x" % (self.ramdump.read_s64(qphy_p +
                                        self.ramdump.field_offset(
                                        'struct ufs_qcom_phy',
                                        'quirks'))))
        print_out_ufs("\t\tlanes_per_direction = %d" % (self.ramdump.read_u32(qphy_p +
                                        self.ramdump.field_offset(
                                        'struct ufs_qcom_phy',
                                        'lanes_per_direction'))))

        vdda_pll_addr = qphy_p + self.ramdump.field_offset('struct ufs_qcom_phy', 'vdda_pll')
        vdda_phy_addr = qphy_p + self.ramdump.field_offset('struct ufs_qcom_phy', 'vdda_phy')
        vddp_ref_clk_addr = qphy_p + self.ramdump.field_offset('struct ufs_qcom_phy', 'vddp_ref_clk')
        self.get_qphy_vreg_info(vdda_pll_addr)
        self.get_qphy_vreg_info(vdda_phy_addr)
        self.get_qphy_vreg_info(vddp_ref_clk_addr)

        if self.ramdump.get_kernel_version() > (5, 11, 0):
            vdd_phy_gdsc_addr = qphy_p + self.ramdump.field_offset('struct ufs_qcom_phy', 'vdd_phy_gdsc')
            vdda_qref_addr = qphy_p + self.ramdump.field_offset('struct ufs_qcom_phy', 'vdda_qref')
            self.get_qphy_vreg_info(vdd_phy_gdsc_addr)
            self.get_qphy_vreg_info(vdda_qref_addr)

        print_out_ufs("\t}")

    def dump_ufs_qc_params(self):

        print_out_ufs("struct ufs_qc_host = 0x%x {" % (self.ufs_qc_host_addr))
        caps_offset = self.ramdump.field_offset('struct ufs_qcom_host', 'caps')
        caps_addr = self.ufs_qc_host_addr + caps_offset
        print_out_ufs("\tcaps = %d" %(self.ramdump.read_int(caps_addr)))

        # Bus Vote
        bus_vote_offset = self.ramdump.field_offset('struct ufs_qcom_host', 'bus_vote')
        bus_vote_addr = self.ufs_qc_host_addr + bus_vote_offset
        print_out_ufs("\tbus_vote = {")
        print_out_ufs("\t\tcurr_vote = %d" %(self.ramdump.read_int(
            bus_vote_addr + self.ramdump.field_offset('struct ufs_qcom_bus_vote', 'curr_vote'))))
        print_out_ufs("\t\tmin_vote = %d" %(self.ramdump.read_int(
            bus_vote_addr + self.ramdump.field_offset('struct ufs_qcom_bus_vote', 'min_bw_vote'))))
        print_out_ufs("\t\tmax_vote = %d" % (self.ramdump.read_int(
            bus_vote_addr + self.ramdump.field_offset('struct ufs_qcom_bus_vote', 'max_bw_vote'))))
        print_out_ufs("\t\tsaved_vote = %d" % (self.ramdump.read_int(
            bus_vote_addr + self.ramdump.field_offset('struct ufs_qcom_bus_vote', 'saved_vote'))))
        print_out_ufs("\t\tis_max_bw_needed = %d" % (self.ramdump.read_int(
            bus_vote_addr + self.ramdump.field_offset('struct ufs_qcom_bus_vote', 'is_max_bw_needed'))))
        print_out_ufs("\t}")

        # PA Layer Attributes
        drp_offset = self.ramdump.field_offset('struct ufs_qcom_host', 'dev_req_params')
        drp_addr = self.ufs_qc_host_addr + drp_offset
        print_out_ufs("\tdev_req_params = {")
        print_out_ufs("\t\tgear_rx = %d" % (self.ramdump.read_int(
            drp_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'gear_rx'))))
        print_out_ufs("\t\tgear_tx = %d" % (self.ramdump.read_int(
            drp_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'gear_tx'))))
        print_out_ufs("\t\tlane_rx = %d" % (self.ramdump.read_int(
            drp_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'lane_rx'))))
        print_out_ufs("\t\tlane_tx = %d" % (self.ramdump.read_int(
            drp_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'lane_tx'))))
        print_out_ufs("\t\tpwr_rx = %d" % (self.ramdump.read_int(
            drp_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'pwr_rx'))))
        print_out_ufs("\t\tpwr_tx = %d" % (self.ramdump.read_int(
            drp_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'pwr_tx'))))
        print_out_ufs("\t\ths_rate = %d" % (self.ramdump.read_int(
            drp_addr + self.ramdump.field_offset('struct ufs_pa_layer_attr', 'hs_rate'))))
        print_out_ufs("\t}")

        print_out_ufs("\tis_lane_clks_enabled = %d" %(self.ramdump.read_bool(
            self.ufs_qc_host_addr + self.ramdump.field_offset('struct ufs_qcom_host', 'is_lane_clks_enabled'))))

        # HW Version
        print_out_ufs("\thw_ver = {")
        hw_ver_offset = self.ramdump.field_offset('struct ufs_qcom_host', 'hw_ver')
        hw_ver_addr = self.ufs_qc_host_addr + hw_ver_offset
        print_out_ufs("\t\tstep = %d" %(self.ramdump.read_byte(
            hw_ver_addr + self.ramdump.field_offset('struct ufs_hw_version', 'step'))))
        print_out_ufs("\t\tminor = %d" % (self.ramdump.read_byte(
            hw_ver_addr + self.ramdump.field_offset('struct ufs_hw_version', 'minor'))))
        print_out_ufs("\t\tmajor = %d" % (self.ramdump.read_byte(
            hw_ver_addr + self.ramdump.field_offset('struct ufs_hw_version', 'major'))))
        print_out_ufs("\t}")

        print_out_ufs("\tdev_ref_clk_en_mask = 0x%x" %(self.ramdump.read_int(
            self.ufs_qc_host_addr + self.ramdump.field_offset('struct ufs_qcom_host', 'dev_ref_clk_en_mask'))))
        print_out_ufs("\tdbg_print_en = 0x%x" %(self.ramdump.read_int(
            self.ufs_qc_host_addr + self.ramdump.field_offset('struct ufs_qcom_host', 'dbg_print_en'))))
        if self.ramdump.get_kernel_version() < (6, 5, 0):
            print_out_ufs("\tlimit_tx_hs_gear = %d" % (self.ramdump.read_int(
                self.ufs_qc_host_addr + self.ramdump.field_offset('struct ufs_qcom_host', 'limit_tx_hs_gear'))))
            print_out_ufs("\tlimit_rx_hs_gear = %d" % (self.ramdump.read_int(
                self.ufs_qc_host_addr + self.ramdump.field_offset('struct ufs_qcom_host', 'limit_rx_hs_gear'))))
            print_out_ufs("\tlimit_tx_pwm_gear = %d" % (self.ramdump.read_int(
                self.ufs_qc_host_addr + self.ramdump.field_offset('struct ufs_qcom_host', 'limit_tx_pwm_gear'))))
            print_out_ufs("\tlimit_rx_pwm_gear = %d" % (self.ramdump.read_int(
                self.ufs_qc_host_addr + self.ramdump.field_offset('struct ufs_qcom_host', 'limit_rx_pwm_gear'))))
            print_out_ufs("\tlimit_rate = %d" % (self.ramdump.read_int(
                self.ufs_qc_host_addr + self.ramdump.field_offset('struct ufs_qcom_host', 'limit_rate'))))
            print_out_ufs("\tlimit_phy_submode = %d" % (self.ramdump.read_int(
                self.ufs_qc_host_addr + self.ramdump.field_offset('struct ufs_qcom_host', 'limit_phy_submode'))))
        else:
            pwr_cap_addr = self.ufs_qc_host_addr + self.ramdump.field_offset('struct ufs_qcom_host', 'host_pwr_cap')
            print_out_ufs("\tlimit_tx_hs_gear = %d" % (self.ramdump.read_u32(
                pwr_cap_addr + self.ramdump.field_offset('struct ufs_qcom_dev_params', 'hs_tx_gear'))))
            print_out_ufs("\tlimit_rx_hs_gear = %d" % (self.ramdump.read_u32(
                pwr_cap_addr + self.ramdump.field_offset('struct ufs_qcom_dev_params', 'hs_rx_gear'))))
            print_out_ufs("\tlimit_tx_pwm_gear = %d" % (self.ramdump.read_u32(
                pwr_cap_addr + self.ramdump.field_offset('struct ufs_qcom_dev_params', 'pwm_tx_gear'))))
            print_out_ufs("\tlimit_rx_pwm_gear = %d" % (self.ramdump.read_u32(
                pwr_cap_addr + self.ramdump.field_offset('struct ufs_qcom_dev_params', 'pwm_rx_gear'))))
            print_out_ufs("\tlimit_rate = %d" % (self.ramdump.read_int(
                pwr_cap_addr + self.ramdump.field_offset('struct ufs_qcom_dev_params', 'hs_rate'))))
            print_out_ufs("\tlimit_phy_submode = %d" % (self.ramdump.read_int(
                pwr_cap_addr + self.ramdump.field_offset('struct ufs_qcom_dev_params', 'phy_submode'))))
        print_out_ufs("\tdisable_lpm = %d" % (self.ramdump.read_bool(
            self.ufs_qc_host_addr + self.ramdump.field_offset('struct ufs_qcom_host', 'disable_lpm'))))
        print_out_ufs("\twork_pending = %d" % (self.ramdump.read_bool(
            self.ufs_qc_host_addr + self.ramdump.field_offset('struct ufs_qcom_host', 'work_pending'))))
        print_out_ufs("\tis_phy_pwr_on = %d" % (self.ramdump.read_bool(
            self.ufs_qc_host_addr + self.ramdump.field_offset('struct ufs_qcom_host', 'is_phy_pwr_on'))))
        print_out_ufs("\terr_occurred = %d" % (self.ramdump.read_bool(
            self.ufs_qc_host_addr + self.ramdump.field_offset('struct ufs_qcom_host', 'err_occurred'))))

        #QoS
        print_out_ufs("\tqos = {\n\t\tNot Implemented\n\t}")
        self.dump_ufs_phy()
        print_out_ufs("}")
        return

    def get_ufs_hba(self):
        hba_offset = self.ramdump.field_offset('struct ufs_qcom_host', 'hba')
        return self.ramdump.read_pointer(self.ufs_qc_host_addr + hba_offset)

class UfsIpc():
    phase_l = ['PRE ', 'POST ']
    state_l = ['CLK_OFF ', 'CLK_ON ']
    pmop_l = ['RT_PM ', 'SYS_PM', 'SHUTDOWN_PM ']
    link_l = ['OFF ', 'ACTIVE', 'HIBERN8 ', 'BROKEN ']
    pwrmod_l = ['NONE ', 'ACTIVE ', 'SLEEP ', 'PWRDOWN ', 'DPSLEEP ']

    def __init__(self, ramdump, ufs_ipc_file):
        self.ramdump = ramdump
        self.ipc_file = ufs_ipc_file

    def ufsipcdict(self, line2, line):
        return{
            '<': lambda: ' [Send cmd] ' + str(line[3]) + ' tag:' + str(line[4]) + ' DBR/HWQ:' + str(line[5]) + ' size=' + str(line[6]),
            '>': lambda: ' [Done cmd] ' + str(line[3]) + ' tag:' + str(line[4]) + ' DBR/HWQ:' + str(line[5]) + ' size=' + str(line[6]),
            '(': lambda: ' [Send uic] ' + str(line[3]) + ' arg1=' + str(line[4]) + ' arg2=' + str(line[5]) + ' arg3=' + str(line[6]),
            ')': lambda: ' [Done uic] ' +str(line[3]) + ' arg1=' + str(line[4]) + ' arg2=' + str(line[5]) + ' arg3=' + str(line[6]),
            '#': lambda: ' [CLK GATE] ' + self.phase_l[int(line[3])] + self.state_l[int(line[4])] + 'err=' + str(line[5]),
            '^': lambda: ' [CLK SCALE UP] ' + self.phase_l[int(line[3])] + 'err=' + str(line[4]),
            'v': lambda: ' [CLK SCALE DOWN] ' + self.phase_l[int(line[3])] + 'err =' + str(line[4]),
            '@': lambda: ' [PWR CHANGE] ' + self.phase_l[int(line[3])] + 'GEAR' + str(line[4]) + ' cur_dev_pwr_mode=' + str(line[5]) + ' rate=' + str(line[6]) + ' err=' + str(line[7]),
            '&': lambda: ' [SUSPEND] ' + self.pmop_l[int(line[3])] + 'rpm_lvl=' + str(line[4]) + ' spm_lvl=' + str(line[5]) + ' link_state: ' + self.link_l[int(line[6])]
                                                + 'cur_dev_pwr_mode: ' + self.pwrmod_l[int(line[7])] + 'err=' + str(line[8]),
            '$': lambda: ' [RESUME] ' +  self.pmop_l[int(line[3])] + 'rpm_lvl=' + str(line[4]) + ' spm_lvl=' + str(line[5]) + ' link_state: ' + self.link_l[int(line[6])]
                                                + 'cur_dev_pwr_mode: ' + self.pwrmod_l[int(line[7])] + 'err = ' + str(line[8]),
            '-': lambda: ' [HCE] ' + self.phase_l[int(line[3])] + 'err= ' + str(line[4]),
            '*': lambda: ' [LSS] ' + self.phase_l[int(line[3])] + 'err= ' + str(line[4]),
            '_': lambda: ' [ERROR] ' + ' hba_err=' + str(line[3]) + ' uic_err=' + str(line[4]),
            '!': lambda: ' [ESI] ' + ' irq=' + str(line[3]) + ' msi_index=' + str(line[4]),
            '0xdead': lambda: ' [SHUTDOWN] ',
        }.get(line2, None)()

    def parse_ufs_ipc(self):
        print_out_ufs('\n\n==================Parsed UFS IPC Log===========================')
        file_path = os.path.join(self.ramdump.outdir, self.ipc_file)
        if not os.path.exists(file_path):
            print_out_ufs("\tThe file %s dose not exist!!!" %file_path)
            return
        with open(file_path) as csv_file:
            csv_reader = csv.reader(csv_file)
            for line in csv_reader:
                if len(line) < 3:
                    continue
                try:
                    result = str(line[0]) + ' cpu' + str(line[1])
                    result += self.ufsipcdict(line[2], line)
                except:
                    print_out_ufs("\texception occurred during parsing '%s'" %str(line))
                    continue
                print_out_ufs("\t%s" %result)

class UfsRegs():
    def __init__(self, ramdump, ufs_qc_host_addr):
        self.ramdump = ramdump
        self.ufs_qc_host_addr = ufs_qc_host_addr

    def dump_ufs_regs(self, buf, word, prefix):
        for i in range(word):
            if i%4 == 0:
                offest = (i//4)*0x10
                print_out_ufs('\t%s:' %prefix, False)
                print_out_ufs('\t%04x:' %offest, False)

            if i%4 == 3 or i == word-1:
                print_out_ufs('\t%08x' %self.ramdump.read_u32(buf + i*4))
            else:
                print_out_ufs('\t%08x' %self.ramdump.read_u32(buf + i*4), False)

    def get_ufs_regs(self, regs_list_addr):
        regs_list_next = self.ramdump.read_pointer(regs_list_addr + self.ramdump.field_offset('struct list_head', 'next'))
        while regs_list_next != regs_list_addr:
            prefix = self.ramdump.read_structure_cstring(regs_list_next, 'struct ufs_qcom_regs', 'prefix')
            len = self.ramdump.read_u32(regs_list_next + self.ramdump.field_offset('struct ufs_qcom_regs', 'len'))
            buf = self.ramdump.read_pointer(regs_list_next + self.ramdump.field_offset('struct ufs_qcom_regs', 'ptr'))
            self.dump_ufs_regs(buf, len//4, prefix)
            regs_list_next = self.ramdump.read_pointer(regs_list_next + self.ramdump.field_offset(
                                        'struct list_head', 'next'))

    def parse_ufs_regs(self):
        gphy_p = self.ramdump.read_pointer(self.ufs_qc_host_addr + self.ramdump.field_offset(
                                        'struct ufs_qcom_host',
                                        'generic_phy'))
        dev_addr = gphy_p + self.ramdump.field_offset('struct phy', 'dev')
        qphy_p = self.ramdump.read_pointer(dev_addr + self.ramdump.field_offset(
                                        'struct device',
                                        'driver_data'))
        host_regs_list_addr = self.ufs_qc_host_addr + self.ramdump.field_offset('struct ufs_qcom_host', 'regs_list_head')
        phy_regs_list_addr = qphy_p + self.ramdump.field_offset('struct ufs_qcom_phy', 'regs_list_head')

        print_out_ufs('\n\n===========================Parsed UFS Registers====================================')
        self.get_ufs_regs(host_regs_list_addr)
        self.get_ufs_regs(phy_regs_list_addr)


@register_parser('--ufs-parser', 'Generate UFS diagnose report', optional=True)

class UfsParser(RamParser):
    def parse(self):
        try:
            if self.ramdump.get_kernel_version() < (4, 20, 0):
                return
        except:
            return

        setup_out_file(F_UFSDEBUG, self)
        print_out_ufs("Kernel Version - %d.%d.%d" %(self.ramdump.get_kernel_version()))

        # struct ufs_qcom dump
        ufs_qc_hosts_ptr = self.ramdump.read_pointer('ufs_qcom_hosts')
        ufs_qc_host_addr = self.ramdump.array_index(ufs_qc_hosts_ptr, 'struct ufs_qcom_host', 0)
        ufsqc_0 = UfsQcHost(self.ramdump, ufs_qc_host_addr)
        ufsqc_0.dump_ufs_qc_params()

        # struct ufs_hba dump
        ufs_hba_0 = UfsHba(self.ramdump, ufsqc_0.get_ufs_hba())
        ufs_hba_0.dump_ufs_hba_params()

        # ufs ipc log parser
        ufs_ipc_0 = UfsIpc(self.ramdump, F_UFSIPC)
        ufs_ipc_0.parse_ufs_ipc()

        # ufs register parser
        ufs_regs_0 = UfsRegs(self.ramdump, ufs_qc_host_addr)
        ufs_regs_0.parse_ufs_regs()

        return

