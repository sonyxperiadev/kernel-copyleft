# Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
# Copyright (c) 2016, 2018, 2020-2021 The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.


from parser_util import register_parser, RamParser
from print_out import print_out_str
from linux_list import ListWalker
from ramdump import Struct
import sys
import re
import string
import os
from os import path
import fileinput
import time

class MdssDbgBase(Struct):
    _struct_name = "struct mdss_debug_base"
    _fields = {
        'name': Struct.get_cstring,
        'base': Struct.get_pointer,
        'max_offset': Struct.get_u32,
        'dump_list': Struct.get_address,
        'reg_dump': Struct.get_pointer,
    }

class MdssDbgXlog(Struct):
    def get_dbgbase_arr(self, key):
        arr = self.get_array_ptrs(key)
        return [MdssDbgBase(self.ramdump, b) for b in arr]

    _struct_name = "struct mdss_dbg_xlog"
    _fields = {
        'blk_arr': get_dbgbase_arr,
    }

class SdeDbgBase(Struct):
    _struct_name = "struct sde_dbg_base"
    _fields = {
            'evtlog': Struct.get_pointer,
            'reglog': Struct.get_pointer,
            'reg_base_list': Struct.get_address,
            'enable_reg_dump' : Struct.get_u32,
            'panic_on_err' : Struct.get_u32,
            'dbgbus_sde': Struct.get_address,
            'dbgbus_vbif_rt': Struct.get_address,
            'dbgbus_dsi': Struct.get_address,
            'dbgbus_rsc': Struct.get_address,
            'dbgbus_lutdma': Struct.get_address,
            'dbgbus_dp': Struct.get_address,
        }

class RangeDumpFbNode(Struct):
    def get_offset(self, key):
        return Struct(self.ramdump, self.get_address(key),
                      struct_name="struct dump_offset",
                      fields={
                          'start': Struct.get_u32,
                          'end': Struct.get_u32,
                      })

    _struct_name = "struct range_dump_node"
    _fields = {
        'offset': get_offset,
        'range_name': Struct.get_cstring,
        'reg_dump': Struct.get_pointer,
    }
class RangeDumpSdeNode(Struct):
    def get_offset(self, key):
        return Struct(self.ramdump, self.get_address(key),
                      struct_name="struct sde_dbg_reg_offset",
                      fields={
                          'start': Struct.get_u32,
                          'end': Struct.get_u32,
                      })

    _struct_name = "struct sde_dbg_reg_range"
    _fields = {
        'offset': get_offset,
        'range_name': Struct.get_cstring,
        'reg_dump': Struct.get_pointer,
    }

def get_u64(self, key):
        address = self.get_address(key)
        return self.ramdump.read_u64(address)

@register_parser('--print-mdssinfo', 'print mdss info')
class MDPinfo(RamParser):
    def __init__(self, *args):
        super(MDPinfo, self).__init__(*args)
        self.outfile = None
        self.optionalfile = 0
        self.prev_timestamp_rd_ptr = -1
        self.prev_timestamp_vsync_irq = -1
        self.log_1 = -1
        self.log_2 = -1
        self.log_3 = -1
        self.log_4 = -1
        self.sde_dbg_evtlog_log_optional = []
        self.counter = 0
        self.ctl_flag = 0
        self.vbif_flag = 0

    def BIT(self, val):
        return (1 << val)

    def mdss_dump_reg(self, addr, length, reg_dump, offset):
        if reg_dump == 0:
            return

        # Making length multiple of 16
        length = int((length + 15) / 16)
        base = addr

        # Print out registers
        for i in range(0, length):
            self.outfile.write('0x{0:x}| '.format(addr-base+offset, 10))
            for j in range(0, 4):
                read = reg_dump + (16 * i) + (4 * j)
                self.outfile.write('{0:08x} '
                                   .format(self.ramdump.read_u32(read), 10))

            self.outfile.write('\n')
            addr += 16

    def print_range(self, blk, node):
        rng = RangeDumpFbNode(self.ramdump, node)

        if (rng.offset.start > rng.offset.end) or (rng.offset.end == 0):
            print_out_str("Invalid offsets (%d, %d) for range: %s" %
                          (rng.offset.start, rng.offset.end, rng.range_name))
            return

        addr = blk.base + rng.offset.start

        self.outfile.write('{0}: base=0x{1:x} start=0x{2:x} end=0x{3:x}\n'
                           .format(rng.range_name, addr,
                                   rng.offset.start, rng.offset.end))
        self.outfile.write('start_addr:{0:x} end_addr:{1:x} reg_addr={2:x}\n'
                           .format(rng.offset.start, rng.offset.end, addr))

        # Calculating length
        length = min(blk.max_offset, rng.offset.end) - rng.offset.start

        self.mdss_dump_reg(addr, length, rng.reg_dump, rng.offset.start)

    def print_sderange(self, node):
        rng = RangeDumpSdeNode(self.ramdump, node)

        if (rng.offset.start is None) or (rng.offset.end is None):
            return

        if (rng.offset.start > rng.offset.end) or (rng.offset.end == 0):
            print_out_str("Invalid offsets (%d, %d) for range: %s" %
                          (rng.offset.start, rng.offset.end, rng.range_name))
            return

        addr = node + rng.offset.start

        self.outfile.write('\n---------------------------------------------------------------------\n')
        self.outfile.write('[{0}]  0x{1:x}\n'.format(rng.range_name, rng.offset.start))
        self.outfile.write('---------------------------------------------------------------------\n')
        # Calculating length

        length =  rng.offset.end - rng.offset.start

        self.mdss_dump_reg(addr, length, rng.reg_dump, rng.offset.start)


    def default_parse(self, sde_dbg_evtlog_log_, output_fd) :
        line_new = '{:<54}{:<29}{:<6}{:<3}{:<200}'.format(sde_dbg_evtlog_log_["fun_name"]+":"+str(sde_dbg_evtlog_log_["line_number"]), "==>"+str(sde_dbg_evtlog_log_["timestamp"])+": "+str(sde_dbg_evtlog_log_["time_diff"]), "["+sde_dbg_evtlog_log_["pid"]+"]","["+sde_dbg_evtlog_log_["cpu"]+"]",sde_dbg_evtlog_log_["data"])
        self.outfile.write(line_new)
        self.outfile.write("\n")



    def decode_dsi_ctrl_isr(self, sde_dbg_evtlog_log_, output_fd):
        xlog_data = str(sde_dbg_evtlog_log_["data"])
        data = xlog_data.split(" ")
        tmp = xlog_data
        # input param check

        if len(data) != 3:
            sde_dbg_evtlog_log_["data"] = tmp
            self.default_parse(sde_dbg_evtlog_log_, output_fd)
            return

        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        tmp = tmp + "==> "
        status_bit = {
            0: 'CMD_MODE_DMA_DONE',
            1: 'CMD_STREAM0_FRAME_DONE',
            2: 'CMD_STREAM1_FRAME_DONE',
            3: 'CMD_STREAM2_FRAME_DONE',
            4: 'VIDEO_MODE_FRAME_DONE',
            5: 'BTA_DONE',
            6: 'CMD_FRAME_DONE',
            7: 'DYN_REFRESH_DONE',
            8: 'DESKEW_DONE',
            9: 'DYN_BLANK_DMA_DONE',
            10: 'ERROR',
        }


        data[0] = 'ctrl' + data[0]
        status = int(data[1], 16)
        status_str = []
        temp_status = status
        for i in range(len(status_bit)):
            if (temp_status == 0x0):
                break
            if (temp_status %2):
                status_str.append(status_bit[i])
            temp_status = temp_status >> 1
        #status_str = [status_bit[i] for i in range(11) if (status & BIT(i))]
        data[1] = 'status: [' + '|'.join(status_str) + ']'
        if status & self.BIT(10):
            error = int(data[2], 16)
            err_list = []
            # Reference: fucntion dsi_ctrl_handle_error_status() in dsi/dsi_ctrl.c
            if error & 0x3000E00:
                err_list.append('PHY contention')
            if error & 0xE0:
                err_list.append('TX timeout')
            if error & 0xF0000:
                err_list.append('FIFO overflow')
            if error & 0xF00000:
                err_list.append('FIFO underflow')
            if error & self.BIT(8):
                err_list.append('PLL unlock')
            if error & 0xF:
                err_list.append('ACK')
            data[2] = 'error: [' + '|'.join(err_list) + ']'
        else:
            del data[2]
        tmp = tmp +' '.join(data)
        sde_dbg_evtlog_log_["data"] = tmp
        self.default_parse(sde_dbg_evtlog_log_, output_fd)

    def decode_dsi_kickoff_msg_tx(self, sde_dbg_evtlog_log_, output_fd):

        flag_bit = {
            0: 'READ',
            1: 'BROADCAST',
            2: 'BROADCAST_MASTER',
            3: 'DEFER_TRIGGER',
            4: 'FIFO_STORE',
            5: 'FETCH_MEMORY',
            6: 'LAST_COMMAND',
            7: 'NON_EMBEDDED_MODE',
            8: 'CUSTOM_DMA_SCHED',
            9: 'ASYNC_WAIT',
        }
        mipi_flag_bit = {
            0: 'REQ_ACK',
            1: 'USE_LPM',
            2: 'UNICAST',
            3: 'LASTCOMMAND',
        }

        xlog_data = str(sde_dbg_evtlog_log_["data"])
        data = xlog_data.split(" ")
        tmp = xlog_data

        # input param check
        if len(data) != 4:
            sde_dbg_evtlog_log_["data"] = tmp
            self.default_parse(sde_dbg_evtlog_log_, output_fd)
            return
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        tmp = tmp + "==> "

        data[0] = 'ctrl' + data[0]
        data[1] = 'IN' #'FUNC_ENTRY'

        flags = int(data[2], 16)
        flags_str = [flag_bit[i] for i in range(10) if (flags & (1<<i))]
        data[2] = 'flags: [' + '|'.join(flags_str) + ']'

        mipi_flags = int(data[3], 16)
        mipi_flags_str = [mipi_flag_bit[i] for i in range(10) if (mipi_flags & (1<<i))]
        data[3] = 'MIPI_DSI_MSG_[' + '|'.join(mipi_flags_str) + ']'
        tmp = tmp +' '.join(data)
        sde_dbg_evtlog_log_["data"] = tmp
        self.default_parse(sde_dbg_evtlog_log_, output_fd)



    def decode_dsi_display_set_mode_sub(self, sde_dbg_evtlog_log_, output_fd):

        mode_flag_bit = {
            0: 'SEAMLESS',
            1: 'DFPS',
            2: 'VBLANK_PRE_MODESET',
            3: 'DMS',
            4: 'VRR',
            5: 'POMS',
            6: 'DYN_CLK',
        }
        xlog_data = str(sde_dbg_evtlog_log_["data"])
        data = xlog_data.split(" ")
        tmp = xlog_data
        # input param check
        if len(data) < 2:
            sde_dbg_evtlog_log_["data"] = tmp
            self.default_parse(sde_dbg_evtlog_log_, output_fd)
            return
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        tmp = tmp + "==> "

        flags = int(data[0], 16)
        flags_str = []
        tmp_flags = flags
        for i in range(len(mode_flag_bit)):
            if (tmp_flags == 0x0):
                break
            if (tmp_flags %2):
                flags_str.append(mode_flag_bit[i])
            tmp_flags = tmp_flags >> 1
        #flags_str = [mode_flag_bit[i] for i in range(10) if (flags & BIT[i])]
        data[0] = 'mode flags: [' + '|'.join(flags_str) + ']'

        mode = int(data[1], 16)
        if mode:
            data[1] = 'cmd panel'
        else:
            data[1] = 'video panel'
        #data[1] = 'cmd panel' if mode else 'video panel'
        tmp = tmp +' '.join(data)
        sde_dbg_evtlog_log_["data"] = tmp
        self.default_parse(sde_dbg_evtlog_log_, output_fd)


    def decode_dsi_display_validate_mode_change(self, sde_dbg_evtlog_log_, output_fd):
        xlog_data = str(sde_dbg_evtlog_log_["data"])
        data = xlog_data.split(" ")
        tmp = xlog_data
        # input param check
        if len(data) != 7 and len(data) != 3:
            sde_dbg_evtlog_log_["data"] = tmp
            self.default_parse(sde_dbg_evtlog_log_, output_fd)
            return
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        tmp = tmp + "==> "

        if len(data) == 3:
            # Dynamic clk change detected; DSI_MODE_FLAG_DYN_CLK
            sde_dbg_evtlog_log_["data"] = tmp+ 'Dynamic clk change: pclk[{}kHz -> {}kHz]'.format(int(data[1], 16), int(data[2], 16))
            self.default_parse(sde_dbg_evtlog_log_, output_fd)

        else:
            # Mode switch is seamless variable refresh; DSI_MODE_FLAG_VRR
            sde_dbg_evtlog_log_["data"] = tmp+ 'Mode switch VRR: fps[{} -> {}] hfp[{} -> {}] vfp[{} -> {}]'.format(
                        int(data[1], 16), int(data[2], 16),
                        int(data[3], 16), int(data[4], 16),
                        int(data[5], 16), int(data[6], 16)
                    )
            self.default_parse(sde_dbg_evtlog_log_, output_fd)

    def decode_dsi_panel_tx_cmd_set(self, sde_dbg_evtlog_log_, output_fd):

        cmd_types = [
                'DSI_CMD_SET_PRE_ON',
                'DSI_CMD_SET_ON',
                'DSI_CMD_SET_POST_ON',
                'DSI_CMD_SET_PRE_OFF',
                'DSI_CMD_SET_OFF',
                'DSI_CMD_SET_POST_OFF',
                'DSI_CMD_SET_PRE_RES_SWITCH',
                'DSI_CMD_SET_RES_SWITCH',
                'DSI_CMD_SET_POST_RES_SWITCH',
                'DSI_CMD_SET_CMD_TO_VID_SWITCH',
                'DSI_CMD_SET_POST_CMD_TO_VID_SWITCH',
                'DSI_CMD_SET_VID_TO_CMD_SWITCH',
                'DSI_CMD_SET_POST_VID_TO_CMD_SWITCH',
                'DSI_CMD_SET_PANEL_STATUS',
                'DSI_CMD_SET_LP1',
                'DSI_CMD_SET_LP2',
                'DSI_CMD_SET_NOLP',
                'DSI_CMD_SET_PPS',
                'DSI_CMD_SET_ROI',
                'DSI_CMD_SET_TIMING_SWITCH',
                'DSI_CMD_SET_POST_TIMING_SWITCH',
                'DSI_CMD_SET_QSYNC_ON',
                'DSI_CMD_SET_QSYNC_OFF',
                'DSI_CMD_SET_MAX'
        ]

        cmd_states = [
                'LP',
                'HS',
                'MAX'
        ]

        xlog_data = str(sde_dbg_evtlog_log_["data"])
        data = xlog_data.split(" ")
        tmp = xlog_data
        # input param check
        if len(data) != 3:
            sde_dbg_evtlog_log_["data"] = tmp
            self.default_parse(sde_dbg_evtlog_log_, output_fd)
            return
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        tmp = tmp + "==> "

        cmd_type = int(data[0], 16)
        if cmd_type < len(cmd_types):
            data[0] = 'type: ' + cmd_types[cmd_type]
        else:
            data[0] = 'type: ' + cmd_type

        data[1] = 'state: {}'.format(cmd_states[int(data[1])])
        data[2] = 'count: {}'.format(int(data[2], 16))
        sde_dbg_evtlog_log_["data"] = tmp + ' '.join(data)
        self.default_parse(sde_dbg_evtlog_log_, output_fd)




    def decode_dsi_display_cont_splash_config(self, sde_dbg_evtlog_log_, output_fd):
        xlog_data = str(sde_dbg_evtlog_log_["data"])
        data = xlog_data.split(" ")
        tmp = xlog_data
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        tmp = tmp + "==> "

        sde_dbg_evtlog_log_["data"] = tmp + 'continuous splash: start'
        self.default_parse(sde_dbg_evtlog_log_, output_fd)


    def decode_dsi_display_splash_res_cleanup(self, sde_dbg_evtlog_log_, output_fd):
        xlog_data = str(sde_dbg_evtlog_log_["data"])
        data = xlog_data.split(" ")
        tmp = xlog_data
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        tmp = tmp + "==> "

        sde_dbg_evtlog_log_["data"] = tmp + 'continuous splash: end'
        self.default_parse(sde_dbg_evtlog_log_, output_fd)


    def decode_dsi_display_check_status(self, sde_dbg_evtlog_log_, output_fd):

        xlog_data = str(sde_dbg_evtlog_log_["data"])
        data = xlog_data.split(" ")
        tmp = xlog_data

        # input param check
        if len(data) != 3 and len(data) != 2:
            sde_dbg_evtlog_log_["data"] = tmp
            self.default_parse(sde_dbg_evtlog_log_, output_fd)
            return
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        tmp = tmp + "==> "

        if data[0] == '1111':
            if int(data[2], 16) == 1:
                sde_dbg_evtlog_log_["data"] = tmp + 'ESD check with te_check_override'
            else:
                sde_dbg_evtlog_log_["data"] = tmp + 'ESD check'

        if data[0] == '2222':
            if int(data[1], 16):
                sde_dbg_evtlog_log_["data"] = tmp + 'ESD check passed'
            else:
                sde_dbg_evtlog_log_["data"] = tmp + 'ESD check failed'
        self.default_parse(sde_dbg_evtlog_log_, output_fd)


    def decode_dsi_ctrl_hw_cmn_set_video_timing(self, sde_dbg_evtlog_log_, output_fd):

        xlog_data = str(sde_dbg_evtlog_log_["data"])
        data = xlog_data.split(" ")
        tmp = xlog_data

        # input param check
        if len(data) != 2:
            sde_dbg_evtlog_log_["data"] = tmp
            self.default_parse(sde_dbg_evtlog_log_, output_fd)
            return
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        tmp = tmp + "==> "

        sde_dbg_evtlog_log_["data"] = tmp + 'ctrl video param updated: v_tolal {}, h_total {}'.format(int(data[0], 16), int(data[1], 16))
        self.default_parse(sde_dbg_evtlog_log_, output_fd)


    def decode_dsi_display_update_dsi_bitrate(self, sde_dbg_evtlog_log_, output_fd):

        xlog_data = str(sde_dbg_evtlog_log_["data"])
        data = xlog_data.split(" ")
        tmp = xlog_data

        # input param check
        if len(data) != 4:
            sde_dbg_evtlog_log_["data"] = tmp
            self.default_parse(sde_dbg_evtlog_log_, output_fd)
            return
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        tmp = tmp + "==> "

        data[0] = 'ctrl' + data[0]
        data[1] = 'bit_clk {}'.format(int(data[1], 16))
        data[2] = 'byte_clk {}'.format(int(data[2], 16))
        data[3] = 'pclk {}'.format(int(data[3], 16))

        sde_dbg_evtlog_log_["data"] = tmp + ' '.join(data)
        self.default_parse(sde_dbg_evtlog_log_, output_fd)


    def decode_dsi_ctrl_update_link_freqs(self, sde_dbg_evtlog_log_, output_fd):

        xlog_data = str(sde_dbg_evtlog_log_["data"])
        data = xlog_data.split(" ")
        tmp = xlog_data

        # input param check
        if len(data) != 4:
            sde_dbg_evtlog_log_["data"] = tmp
            self.default_parse(sde_dbg_evtlog_log_, output_fd)
            return
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        tmp = tmp + "==> "

        data[0] = 'ctrl' + data[0]
        data[1] = 'bit_clk {}'.format(int(data[1], 16))
        data[2] = 'byte_clk {}'.format(int(data[2], 16))
        data[3] = 'pclk {}'.format(int(data[3], 16))

        sde_dbg_evtlog_log_["data"] = tmp + ' '.join(data)
        self.default_parse(sde_dbg_evtlog_log_, output_fd)

    def getrmblktype(self, blktype):
        if(blktype == '0'):
            return "TOP"
        elif(blktype == '1'):
            return "SSPP"
        elif(blktype == '2'):
            return "LM"
        elif(blktype == '3'):
            return "DSPP"
        elif(blktype == '4'):
            return "DS"
        elif(blktype == '5'):
            return "CTL"
        elif(blktype == '6'):
            return "CDM"
        elif(blktype == '7'):
            return "PINGPONG"
        elif(blktype == '8'):
            return "INTF"
        elif(blktype == '9'):
            return "WB"
        elif(blktype == 'a'):
            return "DSC"
        elif(blktype == 'b'):
            return "VDC"
        elif(blktype == 'c'):
            return "MERGE_3D"
        elif(blktype == 'd'):
            return "QDSS"

    def gettopotype(self, topotype):
        if (topotype == '0'):
            return "NONE"
        if (topotype == '1'):
            return "SINGLEPIPE"
        if (topotype == '2'):
            return "SINGLEPIPE_DSC"
        if (topotype == '3'):
            return "SINGLEPIPE_VDC"
        if (topotype == '4'):
            return "DUALPIPE"
        if (topotype == '5'):
            return "DUALPIPE_DSC"
        if (topotype == '6'):
            return "DUALPIPE_3DMERGE"
        if (topotype == '7'):
            return "DUALPIPE_3DMERGE_DSC"
        if (topotype == '8'):
            return "DUALPIPE_3DMERGE_VDC"
        if (topotype == '9'):
            return "DUALPIPE_DSCMERGE"
        if (topotype == 'a'):
            return "PPSPLIT"
        if (topotype == 'b'):
            return "QUADPIPE_3DMERGE"
        if (topotype == 'c'):
            return "QUADPIPE_3DMERGE_DSC"
        if (topotype == 'd'):
            return "QUADPIPE_DSCMERGE"
        if (topotype == 'e'):
            return "QUADPIPE_DSC4HSMERGE"

    def rm_print_rsvps(self, sde_dbg_evtlog_log_, output_fd):
        xlog_data = str(sde_dbg_evtlog_log_["data"])
        data_arr = xlog_data.split(" ")
        tmp = xlog_data
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        if (data_arr[0] =='3'):
            tmp = tmp + "==> "
            if (len(data_arr) == 7):
                tmp = tmp+'BLK : '+self.getrmblktype(data_arr[5])+ ((10 - len(self.getrmblktype(data_arr[5]))) * " ")+' ID: '+ str(int(data_arr[6])-1)
            elif (len(data_arr) == 4):
                tmp = tmp+'TOPO: '+self.gettopotype(data_arr[3])
        sde_dbg_evtlog_log_["data"] = tmp
        self.default_parse(sde_dbg_evtlog_log_, output_fd)

    def crtc_dest_scaler_setup(self, sde_dbg_evtlog_log_, output_fd):
        xlog_data = str(sde_dbg_evtlog_log_["data"])
        data_arr = xlog_data.split(" ")
        tmp = xlog_data
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        if(data_arr[4] != '0'):
            tmp = tmp + "==> "
            tmp = tmp + "DEST_SCALAR_ENABLED"
        sde_dbg_evtlog_log_["data"] = tmp
        self.default_parse(sde_dbg_evtlog_log_, output_fd)

    def crtc_handle_power_event(self, sde_dbg_evtlog_log_, output_fd):
        xlog_data = str(sde_dbg_evtlog_log_["data"])
        data_arr = xlog_data.split(" ")
        tmp = xlog_data
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        tmp = tmp + "==>"
        if(data_arr[1] == '8'):
            tmp = tmp + "SDE_POWER_EVENT_POST_ENABLE"
        elif(data_arr[1] == '4'):
            tmp = tmp + "SDE_POWER_EVENT_PRE_ENABLE"
        elif(data_arr[1] == '2'):
            tmp = tmp + "SDE_POWER_EVENT_POST_DISABLE"
        elif(data_arr[1] == '1'):
            tmp = tmp + "SDE_POWER_EVENT_PRE_DISABLE"
        sde_dbg_evtlog_log_["data"] = tmp
        self.default_parse(sde_dbg_evtlog_log_, output_fd)

    def decode_dsi_bridge_mode_fixup(self, sde_dbg_evtlog_log_, output_fd):

        xlog_data = str(sde_dbg_evtlog_log_["data"])
        data = xlog_data.split(" ")
        tmp = xlog_data

        # input param check
        if len(data) != 6:
            sde_dbg_evtlog_log_["data"] = tmp
            self.default_parse(sde_dbg_evtlog_log_, output_fd)
            return
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        tmp = tmp + "==> "

        if int(data[0], 16) == 0x3333: # SDE_EVTLOG_FUNC_CASE1
            # DSI_MODE_FLAG_POMS
            sde_dbg_evtlog_log_["data"] = tmp + 'Mode change is POMS: {}x{} {}fps {}kHz pclk, {} mode'.format(
                int(data[1], 16), int(data[2], 16),
                int(data[3], 16), int(data[4], 16),
                'cmd' if int(data[5], 16) else 'video'
            )
            self.default_parse(sde_dbg_evtlog_log_, output_fd)
        elif int(data[0], 16) == 0x4444: # SDE_EVTLOG_FUNC_CASE2
            # DSI_MODE_FLAG_DMS
            sde_dbg_evtlog_log_["data"] = tmp + 'Mode change is DMS: {}x{} {}fps {}kHz pclk, {} mode'.format(
                int(data[1], 16), int(data[2], 16),
                int(data[3], 16), int(data[4], 16),
                'cmd' if int(data[5], 16) else 'video'
            )
            self.default_parse(sde_dbg_evtlog_log_, output_fd)

        else:
            # unknown case
            sde_dbg_evtlog_log_["data"]=xlog_data
            default_parse(sde_dbg_evtlog_log_, output_fd)


    def flush_parse(self, flush) :
        ret = ""
        flush_int = int(flush, 16)
        if flush_int & (1 << 0) :
            ret = ret + "ViG0|"
            flush_int = flush_int & ~(1 << 0)
        if flush_int & (1 << 1) :
            ret = ret + "ViG1|"
            flush_int = flush_int & ~(1 << 1)
        if flush_int & (1 << 2) :
            ret = ret + "ViG2|"
            flush_int = flush_int & ~(1 << 2)
        if flush_int & (1 << 3) :
            ret = ret + "DSPP_PA_LUTV_0|"
            flush_int = flush_int & ~(1 << 3)
        if flush_int & (1 << 4) :
            ret = ret + "DSPP_PA_LUTV_1|"
            flush_int = flush_int & ~(1 << 4)
        if flush_int & (1 << 5) :
            ret = ret + "DSPP_PA_LUTV_2|"
            flush_int = flush_int & ~(1 << 5)
        if flush_int & (1 << 6) :
            ret = ret + "LM0|"
            flush_int = flush_int & ~(1 << 6)
        if flush_int & (1 << 7) :
            ret = ret + "LM1|"
            flush_int = flush_int & ~(1 << 7)
        if flush_int & (1 << 8) :
            ret = ret + "LM2|"
            flush_int = flush_int & ~(1 << 8)
        if flush_int & (1 << 9) :
            ret = ret + "LM3|"
            flush_int = flush_int & ~(1 << 9)
        if flush_int & (1 << 10) :
            ret = ret + "LM4|"
            flush_int = flush_int & ~(1 << 10)
        if flush_int & (1 << 11) :
            ret = ret + "DMA0|"
            flush_int = flush_int & ~(1 << 11)
        if flush_int & (1 << 12) :
            ret = ret + "DMA1|"
            flush_int = flush_int & ~(1 << 12)
        if flush_int & (1 << 13) :
            ret = ret + "DSPP0|"
            flush_int = flush_int & ~(1 << 13)
        if flush_int & (1 << 14) :
            ret = ret + "DSPP1|"
            flush_int = flush_int & ~(1 << 14)
        if flush_int & (1 << 15) :
            ret = ret + "DSPP2|"
            flush_int = flush_int & ~(1 << 15)
        if flush_int & (1 << 16) :
            ret = ret + "WB|"
            flush_int = flush_int & ~(1 << 16)
        if flush_int & (1 << 17) :
            ret = ret + "CTL|"
            flush_int = flush_int & ~(1 << 17)
        if flush_int & (1 << 18) :
            ret = ret + "ViG3|"
            flush_int = flush_int & ~(1 << 18)
        if flush_int & (1 << 19) :
            ret = ret + "DSPP_PA_LUTV_3|"
            flush_int = flush_int & ~(1 << 19)
        if flush_int & (1 << 20) :
            ret = ret + "LM5|"
            flush_int = flush_int & ~(1 << 20)
        if flush_int & (1 << 21) :
            ret = ret + "DSPP3|"
            flush_int = flush_int & ~(1 << 21)
        if flush_int & (1 << 22) :
            ret = ret + "DSC|"
            flush_int = flush_int & ~(1 << 22)
        if flush_int & (1 << 23) :
            ret = ret + "MERGE_3D|"
            flush_int = flush_int & ~(1 << 23)
        if flush_int & (1 << 24) :
            ret = ret + "DMA2|"
            flush_int = flush_int & ~(1 << 24)
        if flush_int & (1 << 25) :
            ret = ret + "DMA3|"
            flush_int = flush_int & ~(1 << 25)
        if flush_int & (1 << 26) :
            ret = ret + "CDM0|"
            flush_int = flush_int & ~(1 << 26)
        if flush_int & (1 << 27) :
            ret = ret + "ROT|"
            flush_int = flush_int & ~(1 << 27)
        if flush_int & (1 << 28) :
            ret = ret + "CWB|"
            flush_int = flush_int & ~(1 << 28)
        if flush_int & (1 << 30) :
            ret = ret + "PERIPH|"
            flush_int = flush_int & ~(1 << 30)
        if flush_int & (1 << 31) :
            ret = ret + "INTF"
            flush_int = flush_int & ~(1 << 31)
        return ret


    def flushbits_parse(self, sde_dbg_evtlog_log_, output_fd) :

        xlog_data = str(sde_dbg_evtlog_log_["data"])
        data_arr = xlog_data.split(" ")
        tmp = xlog_data
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        tmp = tmp + "==> "
        tmp = tmp + "INTF:" + data_arr[1] + " CTL:" + data_arr[2] + " | "
        flush = data_arr[3]
        tmp = tmp + self.flush_parse(flush)
        sde_dbg_evtlog_log_["data"] = tmp
        self.default_parse(sde_dbg_evtlog_log_, output_fd)

    def diff_calc_parse1(self, sde_dbg_evtlog_log_, output_fd) :

        curr_timestamp = sde_dbg_evtlog_log_["timestamp"]
        diff = 0
        if (self.prev_timestamp_rd_ptr != -1) :
            diff = int(curr_timestamp) - int(self.prev_timestamp_rd_ptr)
            self.prev_timestamp_rd_ptr = curr_timestamp
        else :
            self.prev_timestamp_rd_ptr = curr_timestamp
        tmp = sde_dbg_evtlog_log_["data"]
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        tmp = tmp +"==>"
        tmp = tmp + "elapsed " + str(diff) + " ns"
        sde_dbg_evtlog_log_["data"] = tmp
        self.default_parse(sde_dbg_evtlog_log_, output_fd)

    def diff_calc_parse3(self, sde_dbg_evtlog_log_, output_fd) :

        curr_timestamp = sde_dbg_evtlog_log_["timestamp"]
        diff = 0
        if (self.prev_timestamp_rd_ptr != -1) :
            diff = int(curr_timestamp) - int(self.prev_timestamp_rd_ptr)
        tmp = sde_dbg_evtlog_log_["data"]
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        tmp = tmp +"==>"
        tmp = tmp + "elapsed from rd_ptr " + str(diff) + " ns"
        sde_dbg_evtlog_log_["data"] = tmp
        self.default_parse(sde_dbg_evtlog_log_, output_fd)

    def diff_calc_parse2(self, sde_dbg_evtlog_log_, output_fd) :

        curr_timestamp = sde_dbg_evtlog_log_["timestamp"]
        diff  = 0
        if (self.prev_timestamp_vsync_irq!= -1) :
            diff = int(curr_timestamp) - int(self.prev_timestamp_vsync_irq)
            self.prev_timestamp_vsync_irq = curr_timestamp
        else :
            self.prev_timestamp_vsync_irq = curr_timestamp
        tmp = sde_dbg_evtlog_log_["data"]
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        tmp = tmp +"==>"
        tmp = tmp + "elapsed " + str(abs(diff)) + " ns"
        sde_dbg_evtlog_log_["data"] = tmp
        self.default_parse(sde_dbg_evtlog_log_, output_fd)

    def pipe(self, pipe_id ) :

        pipe_id = int(pipe_id)

        if(self.ramdump.kernel_version  <= (5, 4)) :
            if pipe_id == 0 :
                pipe = "ViG0"
            elif pipe_id == 1 :
                pipe = "ViG1"
            elif pipe_id == 2 :
                pipe = "ViG2"
            elif pipe_id == 3 :
                pipe = "ViG3"
            elif pipe_id == 4 :
                pipe = "RGB0"
            elif pipe_id == 5 :
                pipe = "RGB1"
            elif pipe_id == 6 :
                pipe = "RGB2"
            elif pipe_id == 7 :
                pipe = "RGB3"
            elif pipe_id == 8 :
                pipe = "DMA0"
            elif pipe_id == 9 :
                pipe = "DMA1"
            elif pipe_id == 10 :
                pipe = "DMA2"
            elif pipe_id == 11 :
                pipe = "DMA3"
            elif pipe_id == 12 :
                pipe = "SSPP_CURSOR0"
            elif pipe_id == 13 :
                pipe = "SSPP_CURSOR1"
            else :
                pipe = "PIPE_INVALID"

        elif(self.ramdump.kernel_version  == (5, 10)) :
            if pipe_id == 0 :
                pipe = "ViG0"
            elif pipe_id == 1 :
                pipe = "ViG1"
            elif pipe_id == 2 :
                pipe = "ViG2"
            elif pipe_id == 3 :
                pipe = "ViG3"
            elif pipe_id == 4 :
                pipe = "ViG3"
            elif pipe_id == 5 :
                pipe = "RGB0"
            elif pipe_id == 6 :
                pipe = "RGB1"
            elif pipe_id == 7 :
                pipe = "RGB2"
            elif pipe_id == 8 :
                pipe = "RGB3"
            elif pipe_id == 9 :
                pipe = "RGB3"
            elif pipe_id == 10 :
                pipe = "DMA0"
            elif pipe_id == 11 :
                pipe = "DMA1"
            elif pipe_id == 12 :
                pipe = "DMA2"
            elif pipe_id == 13 :
                pipe = "DMA3"
            elif pipe_id == 14 :
                pipe = "DMA4"
            elif pipe_id == 15 :
                pipe = "DMA5"
            elif pipe_id == 16 :
                pipe = "DMA5"
            elif pipe_id == 17 :
                pipe = "SSPP_CURSOR0"
            elif pipe_id == 18 :
                pipe = "SSPP_CURSOR1"
            elif pipe_id == 19 :
                pipe = "SSPP_CURSOR1"
            else :
                pipe = "PIPE_INVALID"

        elif(self.ramdump.kernel_version  >= (5, 15)) :
            if pipe_id == 0 :
                pipe = "ViG0"
            elif pipe_id == 1 :
                pipe = "ViG1"
            elif pipe_id == 2 :
                pipe = "ViG2"
            elif pipe_id == 3 :
                pipe = "ViG3"
            elif pipe_id == 4 :
                pipe = "ViG3"
            elif pipe_id == 5 :
                pipe = "DMA0"
            elif pipe_id == 6 :
                pipe = "DMA1"
            elif pipe_id == 7 :
                pipe = "DMA2"
            elif pipe_id == 8 :
                pipe = "DMA3"
            elif pipe_id == 9 :
                pipe = "DMA4"
            elif pipe_id == 10 :
                pipe = "DMA5"
            elif pipe_id == 11 :
                pipe = "DMA5"
            else :
                pipe = "PIPE_INVALID"

        return pipe

    def multirect_index(self, rect_index):

        rect_index = int(rect_index)
        if rect_index == 0 :
            rect = "RECT_SOLO"
        elif rect_index == 1 :
            rect = "RECT0"
        elif rect_index == 2 :
            rect = "RECT1"
        else :
            rect = "RECT_INDEX_INVALID"
        return rect

    def multirect_mode(self, rect_mode) :

        rect_mode = int(rect_mode)
        if rect_mode == 0 :
            mode = "MULTI_RECT_NONE"
        elif rect_mode == 1 :
            mode = "MULTI_RECT_PARALLEL"
        elif rect_mode == 2 :
            mode = "MULTI_RECT_SERIAL"
        else :
            mode = "INVALID_MULTIRECT_MODE"
        return mode


    def rotation(self, rot):

        rot = int(rot)
        rotation = ""
        if rot & (1 << 0) :
            rotation = "ROT_0|"
            rot = rot & ~(1 << 0)
        if rot & (1 << 1):
            rotation = rotation + "ROT_90|"
            rot = rot & ~(1 << 1)
        if rot & (1 << 4) :
            rotation = rotation + "REFLECT_X|"
            rot = rot & ~(1 << 4)
        if rot & (1 << 5) :
            rotation = rotation + "REFLECT_Y|"
            rot = rot & ~(1 << 5)
        return rotation

    def secure(self, sec):

        sec = int(sec)
        secure_ = ""
        if (sec==0):
            secure_ = "FB_NON_SEC"
        elif (sec==1):
            secure_ = "FB_SEC"
        elif (sec==2):
            secure_ = "FB_NON_SEC_DIR_TRANS"
        elif (sec==3):
            secure_ = "FB_SEC_DIR_TRANS"
        else:
            secure_ = "FB_INVALID"
        return secure_

    def fmt_modifier(self, data):

        mod=int(data)
        if(mod==0):
            tmp="map"
        elif(mod==1 or mod==9):
            tmp ="map_ubwc"
        elif(mod==3 or mod==11):
            tmp="map_p010_ubwc"
        elif(mod==2):
            tmp= "map_p010"
        elif(mod==10):
            tmp="map_p010_tile"
        elif(mod==14):
            tmp="map_tp10_tile"
        elif(mod==4):
            tmp="map_tp10"
        elif(mod==7):
            tmp="map_tp10_ubwc"
        elif(mod==8):
            tmp="map_tile"
        else:
            tmp="UNSUPPORTED"
        return tmp


    def composition_parse(self, sde_dbg_evtlog_log_, output_fd) :
        xlog_data = str(sde_dbg_evtlog_log_["data"])
        data_arr = xlog_data.split(" ")
        tmp = xlog_data
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        tmp = tmp + "==>"
        if self.log_1 != -1:
            tmp = tmp + self.rotation(str(int(data_arr[11], 16)))
            tmp = tmp + "CRTC:" + str(int(data_arr[0], 16)) + "|PLANE:" + str(int(data_arr[1], 16)) +"|FB-id:" + str(int(data_arr[2], 16))
            tmp = tmp + "|SRC:[ " + str(int(data_arr[3], 16)) + " " + str(int(data_arr[4], 16)) + " " + str(int(data_arr[5], 16)) + " " + str(int(data_arr[6], 16)) + " ]"
            tmp = tmp + "|DST:[ " + str(int(data_arr[7], 16)) + " " + str(int(data_arr[8], 16)) + " " + str(int(data_arr[9], 16)) + " " + str(int(data_arr[10], 16)) + " ]"
            tmp = tmp + "|" +self.secure(str(int(data_arr[12], 16)))
        elif self.log_2 != -1 :
            self.counter= self.counter+1
            tmp = tmp +  self.pipe(str(int(data_arr[3], 16))) + "|STAGE:" + str(int(data_arr[4], 16)) + "|"
            tmp = tmp +  self.multirect_index(str(int(data_arr[5], 16)))
            a=str(data_arr[7])
            txt="".join(reversed([a[i:i+2] for i in range(0, len(a), 2)]))
            tmp = tmp + "/" + self.multirect_mode(str(int(data_arr[6], 16))) + "|" +  "FMT :" +str(''.join([chr(int(''.join(c), 16)) for c in zip(txt[0::2],txt[1::2])]))
            tmp= tmp + "|MODIFIER: "+ self.fmt_modifier(str(data_arr[8]))
        sde_dbg_evtlog_log_["data"] = tmp
        self.default_parse(sde_dbg_evtlog_log_, output_fd)

    def perf_calc(self, sde_dbg_evtlog_log_,output_fd):
        xlog_data = str(sde_dbg_evtlog_log_["data"])
        data_arr = xlog_data.split(" ")
        tmp = xlog_data
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        if str(self.log_3) in str(sde_dbg_evtlog_log_["line_number"]) and len(data_arr)>=8:
            tmp = tmp + "==> "
            tmp=tmp+"core_clk: "+str(int(data_arr[1], 16))+" "
            tmp=tmp+"mnoc_ab: "+str(int(data_arr[2]+data_arr[3],16))+" "
            tmp=tmp+"llcc_ab: "+str(int(data_arr[4]+data_arr[5],16))+" "
            tmp=tmp+"ebi_ab: "+str(int(data_arr[6]+data_arr[7],16))
        elif str(self.log_4) in str(sde_dbg_evtlog_log_["line_number"]) and len(data_arr)>=7:
            tmp = tmp + "==> "
            tmp=tmp+"mnoc_ib: "+str(int(data_arr[1]+data_arr[2],16))+" "
            tmp=tmp+"llcc_ib: "+str(int(data_arr[3]+data_arr[4],16))+" "
            tmp=tmp+"ebi_ib: "+str(int(data_arr[5]+data_arr[6],16))
        sde_dbg_evtlog_log_["data"] = tmp
        self.default_parse(sde_dbg_evtlog_log_, output_fd)

    def setallbitgivenrange(n, l, r):
        if(l<=r):
            range = (((1 << (l - 1)) - 1) ^((1 << (r)) - 1))
        else:
            range=0
        return (n | range)

    def bitmask_function(key,value,data):
        tmp = value +" "
        matchObj=re.match(r'(bit)(_)([0-9]+)(_)([0-9]+)',key)
        res = int(data, 16)
        n, l, r = 0, int(matchObj.group(3))+1, int(matchObj.group(5))+1
        tmp = tmp+str(res&setallbitgivenrange(n, l, r))+ " "
        return tmp

    def parser_optional(self, sde_dbg_evtlog_log_,output_fd):
        xlog_data = str(sde_dbg_evtlog_log_["data"])
        data_arr = xlog_data.split(" ")
        tmp = xlog_data
        for i in range(len(self.sde_dbg_evtlog_log_optional)):
            if (str(self.sde_dbg_evtlog_log_optional[i]['function_name']) in str(sde_dbg_evtlog_log_["fun_name"])) and (int(self.sde_dbg_evtlog_log_optional[i]['number_of_data']) == len(data_arr)) :
                tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
                tmp = tmp +"==>"
                for x in range(len(data_arr)):
                    if(str('data_'+str(x)) in self.sde_dbg_evtlog_log_optional[i]):
                            matchObj=re.match(r'(print)(\s")(.*)("\s)(format)\s"(.*)"(.*)',self.sde_dbg_evtlog_log_optional[i]['data_'+ str(x)])
                            tmp=tmp+" "+matchObj.group(3)
                            if "decimal" in matchObj.group(6):
                                tmp=tmp+str(int(data_arr[x], 16))
                            elif "string" in matchObj.group(6):
                                a=str(data_arr[x])
                                txt="".join(reversed([a[i:i+2] for i in range(0, len(a), 2)]))
                                try:
                                    tmp = tmp + str(''.join([chr(int(''.join(c), 16)) for c in zip(txt[0::2],txt[1::2])]))
                                except:
                                    tmp = tmp + "Wrong String Encoding "
                            elif "bitmap" in matchObj.group(6):
                                tempstring=(matchObj.group(7)).strip()
                                bitmapDict = dict((x.strip(), y.strip()) for x, y in (element.split(':') for element in (tempstring).split(', ')))
                                for z in bitmapDict:
                                    tmp = tmp + bitmask_function(z,bitmapDict[z],data_arr[x])
                break
        sde_dbg_evtlog_log_["data"] = tmp
        self.default_parse(sde_dbg_evtlog_log_, output_fd)

    def commit_printing(self, sde_dbg_evtlog_log_, output_fd):
        line_new =sde_dbg_evtlog_log_["fun_name"]
        self.outfile.write(line_new)

    def sec_only(self, enum_value):
        if(enum_value==1):
            return "SEC_ONLY"
        elif(enum_value==0):
            return "SEC_NON_SEC"
        return "INVALID"

    def dir_trans(self, enum_value):
        if (enum_value==0):
            return "FB_NON_SEC"
        elif(enum_value==1):
            return "FB_SEC"
        elif(enum_value==2):
            return "FB_NON_SEC_DIR_TRANS"
        elif(enum_value==3):
            return "FB_SEC_DIR_TRANS"
        return "INVALID"

    def state(self, enum_value):
        if (enum_value==0):
            return "ATTACHED"
        elif(enum_value==1):
            return "DETACHED"
        elif(enum_value==2):
            return "DETACHED_SEC"
        elif(enum_value==3):
            return "ATTACH_ALL_REQ"
        elif(enum_value==4):
            return "DETACH_ALL_REQ"
        elif(enum_value==5):
            return "DETACH_SEC_REQ"
        elif(enum_value==6):
            return "ATTACH_SEC_REQ"
        return "INVALID"

    def trans_type(self, enum_value):
        if (enum_value==0):
            return "NONE"
        elif(enum_value==1):
            return "PRE_COMMIT"
        elif(enum_value==2):
            return "POST_COMMIT"
        return "INVALID"

    def secure_transition(self, sde_dbg_evtlog_log_,output_fd):
        xlog_data = str(sde_dbg_evtlog_log_["data"])
        data_arr = xlog_data.split(" ")
        tmp = xlog_data
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        if(len(data_arr)>=6):
            tmp = tmp + "==> "
            tmp=tmp+"sec_lvl: "+self.sec_only(int(data_arr[1], 16))+" "
            tmp=tmp+"trans_mode: "+self.dir_trans(int(data_arr[2],16))+" "
            tmp=tmp+"smmu_state->state: "+self.state(int(data_arr[3],16))+" "
            tmp=tmp+"smmu_state->trans_type: "+self.trans_type(int(data_arr[4],16))+" "
            tmp=tmp+"smmu_state->sec_lvl: "+self.sec_only(int(data_arr[5], 16))
        sde_dbg_evtlog_log_["data"] = tmp
        self.default_parse(sde_dbg_evtlog_log_, output_fd)

    def sde_kms_sui_misr_state(self, enum_value):
        if (enum_value==0):
            return "SUI_MISR_NONE"
        elif(enum_value==1):
            return "SUI_MISR_ENABLE_REQ"
        elif(enum_value==2):
            return "SUI_MISR_DISABLE_REQ"
        return "INVALID"

    def kms_secure(self, sde_dbg_evtlog_log_,output_fd):
        xlog_data = str(sde_dbg_evtlog_log_["data"])
        data_arr = xlog_data.split(" ")
        tmp = xlog_data
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        if(len(data_arr)==10):
            tmp = tmp + "==> "
            tmp=tmp+"smmu_state->state: "+self.state(int(data_arr[1],16))+" "
            tmp=tmp+"smmu_state->prev_state: "+self.state(int(data_arr[2],16))+" "
            tmp=tmp+"smmu_state->trans_type: "+self.trans_type(int(data_arr[3],16))+" "
            if int(data_arr[4],16)==0:
                tmp=tmp+"smmu_state->transition_error: "+"false"+" "
            else:
                tmp=tmp+"smmu_state->transition_error: "+"true"+" "
            tmp=tmp+"smmu_state->sec_lvl: "+self.sec_only(int(data_arr[5], 16))+" "
            tmp=tmp+"smmu_state->prev_sec_lvl: "+self.sec_only(int(data_arr[6], 16))+" "
            tmp=tmp+"smmu_state->sui_misr_state: "+self.sde_kms_sui_misr_state(int(data_arr[7], 16))
        if(len(data_arr)==7):
            tmp = tmp + "==> "
            tmp=tmp+"smmu_state->state: "+self.state(int(data_arr[1],16))+" "
            tmp=tmp+"smmu_state->trans_type: "+self.trans_type(int(data_arr[2],16))+" "
            tmp=tmp+"smmu_state->sui_misr_state: "+self.sde_kms_sui_misr_state(int(data_arr[4], 16))+" "
            tmp=tmp+"smmu_state->sec_lvl: "+self.sec_only(int(data_arr[5], 16))
        sde_dbg_evtlog_log_["data"] = tmp
        self.default_parse(sde_dbg_evtlog_log_, output_fd)

    def vmid(self, enum_value):
        if (enum_value=="3"):
            return "VMID_HLOS"
        elif(enum_value=="8"):
            return "VMID_CP_TOUCH"
        elif(enum_value=="9"):
            return "VMID_CP_BITSTREAM"
        elif(enum_value=="A"):
            return "VMID_CP_PIXEL"
        elif(enum_value=="B"):
            return "VMID_CP_NON_PIXEL"
        elif(enum_value=="D"):
            return "VMID_CP_CAMERA"
        elif(enum_value=="E"):
            return "VMID_HLOS_FREE"
        elif(enum_value=="F"):
            return "VMID_MSS_MSA"
        elif(enum_value=="10"):
            return "VMID_MSS_NONMSA"
        elif(enum_value=="11"):
            return "VMID_CP_SEC_DISPLAY"
        elif(enum_value=="12"):
            return "VMID_CP_APP"
        elif(enum_value=="18"):
            return "VMID_WLAN"
        elif(enum_value=="19"):
            return "VMID_WLAN_CE"
        elif(enum_value=="1A"):
            return "VMID_CP_SPSS_SP"
        elif(enum_value=="1D"):
            return "VMID_CP_CAMERA_PREVIEW"
        elif(enum_value=="22"):
            return "VMID_CP_SPSS_SP_SHARED"
        elif(enum_value=="24"):
            return "VMID_CP_SPSS_HLOS_SHARED"
        elif(enum_value=="2A"):
            return "VMID_CP_CDSP"
        elif(enum_value=="2B"):
            return "VMID_NAV"
        elif(enum_value=="2C"):
            return "VMID_LAST"
        elif(enum_value=="-1"):
            return "VMID_INVAL"
        return "INVALID"

    def kms_scm_call(self, sde_dbg_evtlog_log_,output_fd):
        xlog_data = str(sde_dbg_evtlog_log_["data"])
        data_arr = xlog_data.split(" ")
        tmp = xlog_data
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        if(len(data_arr)==7):
            tmp = tmp + "==> "
            tmp=tmp+"vmid: "+self.vmid((data_arr[3]).upper())
        sde_dbg_evtlog_log_["data"] = tmp
        self.default_parse(sde_dbg_evtlog_log_, output_fd)

    def output_roi(self, sde_dbg_evtlog_log_,output_fd):
        xlog_data = str(sde_dbg_evtlog_log_["data"]).strip()
        data_arr = xlog_data.split(" ")
        tmp = xlog_data
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        if(len(data_arr)>5):
            tmp = tmp + "==>LM_ROI(X Y W H): "
            tmp = tmp +"("+str(int(data_arr[2],16))+" "
            tmp= tmp +str(int(data_arr[3],16))+" "
            tmp = tmp +str(int(data_arr[4],16))+" "
            tmp= tmp +str(int(data_arr[5],16))+")"
        sde_dbg_evtlog_log_["data"] = tmp
        self.default_parse(sde_dbg_evtlog_log_, output_fd)

    def flags(self, value):
        tmp= ""
        if(value==1):
            tmp=tmp+"FLAG_PHSYNC"
        if(value==2):
            tmp=tmp+"FLAG_NHSYNC"
        if(value==4):
            tmp=tmp+"FLAG_PVSYNC"
        if(value==8):
            tmp=tmp+"FLAG_NVSYNC"
        if(value==16):
            tmp=tmp+"FLAG_INTERLACE"
        if(value==32):
            tmp=tmp+"FLAG_DBLSCAN"
        if(value==64):
            tmp=tmp+"FLAG_CSYNC"
        if(value==128):
            tmp=tmp+"FLAG_PCSYNC"
        if(value==256):
            tmp=tmp+"FLAG_NCSYNC"
        if(value==512):
            tmp=tmp+"FLAG_HSKEW"
        if(value==1024):
            tmp=tmp+"FLAG_BCAST"
        if(value==2048):
            tmp=tmp+"FLAG_PIXMUX"
        if(value==4096):
            tmp=tmp+"FLAG_DBLCLK"
        if(value==8192):
            tmp=tmp+"FLAG_CLKDIV2"
        if(value==32505856):
            tmp=tmp+"FLAG_3D_MASK"
        if(value==0):
            tmp=tmp+"FLAG_3D_NONE"
        if(value==16384):
            tmp=tmp+"FLAG_3D_FRAME_PACKING"
        if(value==32768):
            tmp=tmp+"FLAG_3D_FIELD_ALTERNATIVE"
        if(value==49152):
            tmp=tmp+"FLAG_3D_LINE_ALTERNATIVE"
        if(value==65536):
            tmp=tmp+"FLAG_3D_SIDE_BY_SIDE_FULL"
        if(value==81920):
            tmp=tmp+"FLAG_3D_L_DEPTH"
        if(value==98304):
            tmp=tmp+"FLAG_3D_L_DEPTH_GFX_GFX_DEPTH"
        if(value==114688):
            tmp=tmp+"FLAG_3D_TOP_AND_BOTTOM"
        if(value==131072):
            tmp=tmp+"FLAG_3D_SIDE_BY_SIDE_HALF"
        if(tmp==""):
            tmp="NIL"
        return tmp

    def private_flags(self, value):
        tmp=""
        if(value==1):
            tmp=tmp+"FLAG_DFPS"
        if(value==2):
            tmp=tmp+"FLAG_VBLANK_PRE_MODESET"
        if(value==4):
            tmp=tmp+"FLAG_DMS"
        if(value==8):
            tmp=tmp+"FLAG_VRR"
        if(value==16):
            tmp=tmp+"FLAG_POMS"
        if(value==32):
            tmp=tmp+"FLAG_DYN_CLK"
        if(tmp==""):
            tmp="NIL"
        return tmp

    def atomic_check(self, sde_dbg_evtlog_log_,output_fd):
        xlog_data = str(sde_dbg_evtlog_log_["data"])
        data_arr = xlog_data.split(" ")
        tmp = xlog_data
        tmp  = tmp + ((50 - len(str(sde_dbg_evtlog_log_["data"]))) * " ")
        if(len(data_arr)==4):
            if (int(data_arr[1],16)==1):
                tmp=tmp+"==> crtc_state->mode_changed"+"  "
            if (int(data_arr[2],16)==1):
                tmp=tmp+"==> crtc_state->active_changed"+" "
            if (int(data_arr[3],16)==1):
                tmp=tmp+"==> crtc_state->connectors_changed"
        elif(len(data_arr)==3):
            tmp = tmp + "==> "
            tmp=tmp+"flags: "+self.flags(int(data_arr[1],16))+"  "
            tmp=tmp+"private_flags: "+self.private_flags(int(data_arr[2],16))
        elif(len(data_arr)==9):
            tmp = tmp + "==> "
            tmp=tmp+"flags: "+self.flags(int(data_arr[1],16))+"  "
            tmp=tmp+"private_flags: "+self.private_flags(int(data_arr[2],16))
            tmp=tmp+" top: "+self.gettopotype(data_arr[3])
            tmp=tmp+" refresh: "+ str(int(data_arr[4],16))
            tmp=tmp+" wxh: "+ str(int(data_arr[5],16)) + "x" + str(int(data_arr[6],16))
        sde_dbg_evtlog_log_["data"] = tmp
        self.default_parse(sde_dbg_evtlog_log_, output_fd)

    def stage2_parse(self, sde_dbg_evtlog_log, output_fd) :
        i=0
        while i <len(sde_dbg_evtlog_log) :
            try:
                if "_sde_crtc_blend_setup_mixer" in str(sde_dbg_evtlog_log[i]["fun_name"]):
                    if i==0:
                        k=0
                        while k < len(sde_dbg_evtlog_log):
                            if "_sde_crtc_blend_setup_mixer" in str(sde_dbg_evtlog_log[k]["fun_name"]):
                                k=k+1
                            else:
                                break
                        if k%2 == 1:
                            self.default_parse(sde_dbg_evtlog_log[0], output_fd)
                        else:
                            self.composition_parse(sde_dbg_evtlog_log[0], output_fd)
                    else:
                        if self.log_1 == -1 and int(sde_dbg_evtlog_log[i]["line_number"])< int(sde_dbg_evtlog_log[i+1]["line_number"]):
                            self.log_1 = int(sde_dbg_evtlog_log[i]["line_number"])
                            self.log_2 = -1

                        elif self.log_2 == -1:
                            self.log_2 = int(sde_dbg_evtlog_log[i]["line_number"])
                            self.log_1 = -1

                        self.composition_parse(sde_dbg_evtlog_log[i], output_fd)

                    #self.default_parse(sde_dbg_evtlog_log[i], output_fd)
                elif "_sde_encoder_trigger_flush" in str(sde_dbg_evtlog_log[i]["fun_name"]) :
                    self.flushbits_parse(sde_dbg_evtlog_log[i], output_fd)
                elif "sde_encoder_phys_cmd_te_rd_ptr_irq" in sde_dbg_evtlog_log[i]["fun_name"] :
                    self.diff_calc_parse1(sde_dbg_evtlog_log[i], output_fd)
                elif "sde_encoder_phys_vid_vblank_irq" in sde_dbg_evtlog_log[i]["fun_name"] :
                    self.diff_calc_parse2(sde_dbg_evtlog_log[i], output_fd)
                elif "sde_encoder_phys_cmd_pp_tx_done_irq" in sde_dbg_evtlog_log[i]["fun_name"] :
                    self.diff_calc_parse3(sde_dbg_evtlog_log[i], output_fd)
                elif "sde_kms_commit" in sde_dbg_evtlog_log[i]["fun_name"] :
                    self.default_parse(sde_dbg_evtlog_log[i], output_fd)
                elif "_sde_core_perf_calc_crtc" in sde_dbg_evtlog_log[i]["fun_name"] :
                    if i==0:
                        k=0
                        while k < len(sde_dbg_evtlog_log):
                            if "_sde_core_perf_calc_crtc" in str(sde_dbg_evtlog_log[k]["fun_name"]):
                                k=k+1
                            else:
                                break
                        if k%2 == 1:
                            self.default_parse(sde_dbg_evtlog_log[0], output_fd)
                        else:
                            self.perf_calc(sde_dbg_evtlog_log[i], output_fd)
                    else:
                        if self.log_3 == -1 and int(sde_dbg_evtlog_log[i]["line_number"])<int(sde_dbg_evtlog_log[i+1]["line_number"]):
                            self.log_3 = int(sde_dbg_evtlog_log[i]["line_number"])

                        elif self.log_4 == -1:
                            self.log_4 = int(sde_dbg_evtlog_log[i]["line_number"])
                        self.perf_calc(sde_dbg_evtlog_log[i], output_fd)
                elif "_sde_crtc_program_lm_output_roi" in sde_dbg_evtlog_log[i]["fun_name"] :
                    self.output_roi(sde_dbg_evtlog_log[i], output_fd)
                elif "sde_encoder_virt_atomic_check" in sde_dbg_evtlog_log[i]["fun_name"] :
                    self.atomic_check(sde_dbg_evtlog_log[i], output_fd)
                elif "sde_crtc_get_secure_transition_ops" in sde_dbg_evtlog_log[i]["fun_name"]:
                    self.secure_transition(sde_dbg_evtlog_log[i], output_fd)
                elif "_sde_kms_secure_ctrl" in sde_dbg_evtlog_log[i]["fun_name"]:
                    self.kms_secure(sde_dbg_evtlog_log[i], output_fd)
                elif "_sde_kms_scm_call" in sde_dbg_evtlog_log[i]["fun_name"]:
                    self.kms_scm_call(sde_dbg_evtlog_log[i], output_fd)

                elif "dsi_ctrl_isr" in sde_dbg_evtlog_log[i]["fun_name"]:
                    self.decode_dsi_ctrl_isr(sde_dbg_evtlog_log[i], output_fd)
                elif "dsi_kickoff_msg_tx" in sde_dbg_evtlog_log[i]["fun_name"]:
                    self.decode_dsi_kickoff_msg_tx(sde_dbg_evtlog_log[i], output_fd)
                elif "dsi_display_set_mode_sub" in sde_dbg_evtlog_log[i]["fun_name"]:
                    self.decode_dsi_display_set_mode_sub(sde_dbg_evtlog_log[i], output_fd)
                elif "dsi_display_validate_mode_change" in sde_dbg_evtlog_log[i]["fun_name"]:
                    self.decode_dsi_display_validate_mode_change(sde_dbg_evtlog_log[i], output_fd)
                elif "dsi_panel_tx_cmd_set" in sde_dbg_evtlog_log[i]["fun_name"]:
                    self.decode_dsi_panel_tx_cmd_set(sde_dbg_evtlog_log[i], output_fd)
                elif "dsi_display_cont_splash_config" in sde_dbg_evtlog_log[i]["fun_name"]:
                    self.decode_dsi_display_cont_splash_config(sde_dbg_evtlog_log[i], output_fd)
                elif "dsi_display_splash_res_cleanup" in sde_dbg_evtlog_log[i]["fun_name"]:
                    self.decode_dsi_display_splash_res_cleanup(sde_dbg_evtlog_log[i], output_fd)
                elif "dsi_display_check_status" in sde_dbg_evtlog_log[i]["fun_name"]:
                    self.decode_dsi_display_check_status(sde_dbg_evtlog_log[i], output_fd)
                elif "dsi_ctrl_hw_cmn_set_video_timing" in sde_dbg_evtlog_log[i]["fun_name"]:
                    self.decode_dsi_ctrl_hw_cmn_set_video_timing(sde_dbg_evtlog_log[i], output_fd)
                elif "dsi_display_update_dsi_bitrate" in sde_dbg_evtlog_log[i]["fun_name"]:
                    self.decode_dsi_display_update_dsi_bitrate(sde_dbg_evtlog_log[i], output_fd)
                elif "dsi_ctrl_update_link_freqs" in sde_dbg_evtlog_log[i]["fun_name"]:
                    self.decode_dsi_ctrl_update_link_freqs(sde_dbg_evtlog_log[i], output_fd)
                elif "dsi_bridge_mode_fixup" in sde_dbg_evtlog_log[i]["fun_name"]:
                    self.decode_dsi_bridge_mode_fixup(sde_dbg_evtlog_log[i], output_fd)
                elif "sde_crtc_handle_power_event" in sde_dbg_evtlog_log[i]["fun_name"]:
                    self.crtc_handle_power_event(sde_dbg_evtlog_log[i], output_fd)
                elif "_sde_rm_print_rsvps" in sde_dbg_evtlog_log[i]["fun_name"]:
                    self.rm_print_rsvps(sde_dbg_evtlog_log[i], output_fd)
                elif "_sde_crtc_dest_scaler_setup" in sde_dbg_evtlog_log[i]["fun_name"]:
                    self.crtc_dest_scaler_setup(sde_dbg_evtlog_log[i], output_fd)
                elif "---------------" in sde_dbg_evtlog_log[i]["fun_name"]:
                    self.commit_printing(sde_dbg_evtlog_log[i],output_fd)
                else:
                    if self.optionalfile==0:
                        self.default_parse(sde_dbg_evtlog_log[i], output_fd)
                    else:
                        self.parser_optional(sde_dbg_evtlog_log[i],output_fd)
                i=i+1
            except:
                self.default_parse(sde_dbg_evtlog_log[i], output_fd)
                i=i+1

    def optional(advanced_input):
        self.optionalfile=1
        f_outoptional=open(advanced_input,"r+")
        for eachLine in f_outoptional:
            if 'function_name' in eachLine:
                sde_dbg_evtlog_log_symbol={}
                sde_dbg_evtlog_log_symbol[(eachLine.split('-')[0]).strip()]=(eachLine.split('-')[1]).strip()
                eachLine=f_outoptional.readline()
                sde_dbg_evtlog_log_symbol[(eachLine.split('-')[0]).strip()]=(eachLine.split('-')[1]).strip()
                for i in range(int(eachLine.split('-')[1])):
                    eachLine=f_outoptional.readline()
                    if(eachLine == "") or (eachLine =="\n") :
                        break
                    sde_dbg_evtlog_log_symbol[(eachLine.split('-')[0]).strip()]=(eachLine.split('-')[1]).strip()
                self.sde_dbg_evtlog_log_optional.append(sde_dbg_evtlog_log_symbol)

    def split_dic(self, output2):
        f_out=self.ramdump.open_file(output2,"r")
        n=f_out.readline()
        output3="sde_evtlog_parsed.txt"
        self.outfile = self.ramdump.open_file(output3, "w")
        sde_dbg_evtlog_log = [dict() for x in range( len(f_out.readlines(  )))]
        f_out.seek(0,0)
        i=0
        for eachLine in f_out:
            try:
                matchObj=re.match(r'([a-zA-Z0-9_\-\.]+)(\s*)([0-9]+)(\s*)([0-9]+)(\s*)([0-9]+)(\s*)([a-zA-Z0-9_\-\.]+)(\s*)(.*)',eachLine)
                sde_dbg_evtlog_log[i]["fun_name"] = matchObj.group(1)
                sde_dbg_evtlog_log[i]["line_number"] =int(matchObj.group(3))
                sde_dbg_evtlog_log[i]["timestamp"] =int(matchObj.group(5))
                if i == 0:
                    sde_dbg_evtlog_log[i]["time_diff"] =0
                else:
                    if "----" in sde_dbg_evtlog_log[i-1]["fun_name"] and i!=1:
                        sde_dbg_evtlog_log[i]["time_diff"] =sde_dbg_evtlog_log[i]["timestamp"]-sde_dbg_evtlog_log[i-2]["timestamp"]
                    else:
                        sde_dbg_evtlog_log[i]["time_diff"] =sde_dbg_evtlog_log[i]["timestamp"]-sde_dbg_evtlog_log[i-1]["timestamp"]
                sde_dbg_evtlog_log[i]["pid"] = matchObj.group(7)
                if "NA" in matchObj.group(9):
                    sde_dbg_evtlog_log[i]["cpu"] = 'NA'
                else:
                    sde_dbg_evtlog_log[i]["cpu"] = matchObj.group(9)
                sde_dbg_evtlog_log[i]["data"] = re.sub(r'\s+$', '', matchObj.group(11))
            except:
                if("----" in eachLine):
                    sde_dbg_evtlog_log[i]["fun_name"]=eachLine
                    sde_dbg_evtlog_log[i]["timestamp"]=0
                else:
                    continue
            i=i+1
        f_out.close()
        title = '{:<54}{:<28}{:<6}{:<3}{:<200}'.format('FUNCTION_NAME:LINE_NO',"==>TIMESTAMP:Delta", "[PID]", "[CPU]", "DATA"  )
        self.outfile.write(title)
        self.outfile.write("\n")
        self.stage2_parse(sde_dbg_evtlog_log, self.outfile)
        self.outfile.close()
        return sde_dbg_evtlog_log

    def reg_parser(self, outputfile):
        f_in=self.ramdump.open_file(outputfile,"r")
        contents_for_analysis = [dict() for eachLine in f_in if "[" in eachLine]
        f_in.seek(0,0)
        i=-1
        for eachLine in f_in:
            if "[" in eachLine:
                i=i+1
                try:
                    matchObj=re.match('\[([a-zA-Z0-9_]+)\]\s*([0-9a-zA-Z]+)',eachLine)
                    contents_for_analysis[i]["reg_name"]=matchObj.group(1)
                    contents_for_analysis[i]["offset"]=(matchObj.group(2)).lower()
                    contents_for_analysis[i]["data"]=[]
                except:
                    continue
            else:
                try:
                    matchObj=re.match('(.*)\|(.*)',eachLine)
                    word=str(matchObj.group(2)).strip().split(" ")
                    contents_for_analysis[i]["data"].extend(word)
                except:
                    continue

        while {} in contents_for_analysis:
            contents_for_analysis.remove({})

        self.outfile = self.ramdump.open_file('reg_stage_3.txt', 'w')
        for  i in range(len(contents_for_analysis)):
            if "0x5000" in contents_for_analysis[i]["offset"].strip():
                self.outfile.write("VIG_0 ")
            elif "0x7000" in contents_for_analysis[i]["offset"].strip():
                self.outfile.write("VIG_1 ")
            elif "0x9000" in contents_for_analysis[i]["offset"]:
                self.outfile.write("VIG_2 ")
            elif "0xb000" in contents_for_analysis[i]["offset"]:
                self.outfile.write("VIG_3 ")
            elif "0x25000" in contents_for_analysis[i]["offset"]:
                self.outfile.write("DMA_0 ")
            elif "0x27000" in contents_for_analysis[i]["offset"]:
                self.outfile.write("DMA_1 ")
            elif "0x29000" in contents_for_analysis[i]["offset"]:
                self.outfile.write("DMA_2 ")
            elif "0x2b000" in contents_for_analysis[i]["offset"]:
                self.outfile.write("DMA_3 ")
            elif "intf_0" in contents_for_analysis[i]["reg_name"]:
                self.outfile.write("INTF_0 ")
            elif "intf_1" in contents_for_analysis[i]["reg_name"]:
                self.outfile.write("INTF_1 ")
            elif "intf_2" in contents_for_analysis[i]["reg_name"]:
                self.outfile.write("INTF_2 ")
            elif "intf_3" in contents_for_analysis[i]["reg_name"]:
                self.outfile.write("INTF_3 ")
            else:
                self.outfile.write(contents_for_analysis[i]["reg_name"]+" ")

            self.outfile.write(contents_for_analysis[i]["offset"]+ " ")

            for k in range(len(contents_for_analysis[i]["data"])):
                flag_to_check=1
                self.outfile.write(contents_for_analysis[i]["data"][k])
                self.outfile.write(" ")
            self.outfile.write("\n")
        self.outfile.close()
        return contents_for_analysis

    def ctl_layer_ext0(self,data):
        value_2_0=int(str(data),16)&(7)
        temp=""
        if(value_2_0!=0): temp=temp+"VIG_0_OUT|"
        value_5_3=int(str(data),16)&(56)
        if(value_5_3!=0): temp=temp+"VIG_1_OUT|"
        value_8_6=int(str(data),16)&(448)
        if(value_8_6!=0): temp=temp+"VIG_2_OUT|"
        value_11_9=int(str(data),16)&(3584)
        if(value_11_9!=0): temp=temp+"RGB_0_OUT|"
        value_14_12=int(str(data),16)&(28672)
        if(value_14_12!=0): temp=temp+"RGB_1_OUT|"
        value_17_15=int(str(data),16)&(229376)
        if(value_17_15!=0): temp=temp+"RGB_2_OUT|"
        value_20_18=int(str(data),16)&(1835008)
        if(value_20_18!=0): temp=temp+"DMA_0_OUT|"
        value_23_21=int(str(data),16)&(14680064)
        if(value_23_21!=0): temp=temp+"DMA_1_OUT|"
        value_24=int(str(data),16)&(16777216)
        if(value_24!=0): temp=temp+"BORDER_OUT|"
        value_25=int(str(data),16)&(33554432)
        if(value_25!=0): temp=temp+"CURSOR_OUT|"
        value_28_26=int(str(data),16)&(469762048)
        if(value_28_26!=0): temp=temp+"VIG_3_OUT|"
        value_31_29=int(str(data),16)&(3758096384)
        if(value_31_29!=0): temp=temp+"RGB_3_OUT|"

        return temp



    def ctl_layer_ext1(self,data):
        value_0=int(str(data),16)&(1)
        temp=""
        if(value_0!=0): temp=temp+"VIG_0_OUT_BIT3|"
        value_2=int(str(data),16)&(4)
        if(value_2!=0): temp=temp+"VIG_1_OUT_BIT3|"
        value_4=int(str(data),16)&(16)
        if(value_4!=0): temp=temp+"VIG_2_OUT_BIT3|"
        value_6=int(str(data),16)&(64)
        if(value_6!=0): temp=temp+"VIG_3_OUT_BIT3|"
        value_8=int(str(data),16)&(256)
        if(value_8!=0): temp=temp+"RGB_0_OUT_BIT3 |"
        value_10=int(str(data),16)&(1024)
        if(value_10!=0): temp=temp+"RGB_1_OUT_BIT3 |"
        value_12=int(str(data),16)&(4096)
        if(value_12!=0): temp=temp+"RGB_2_OUT_BIT3 |"
        value_14=int(str(data),16)&(16384)
        if(value_14!=0): temp=temp+"RGB_3_OUT_BIT3 |"
        value_16=int(str(data),16)&(65536)
        if(value_16!=0): temp=temp+"DMA_0_OUT_BIT3|"
        value_18=int(str(data),16)&(262144)
        if(value_18!=0): temp=temp+"DMA_1_OUT_BIT3|"
        value_23_20=int(str(data),16)&(15728640)
        if(value_23_20!=0): temp=temp+"CURSOR_0_OUT|"
        value_29_26=int(str(data),16)&(1006632960)
        if(value_29_26!=0): temp=temp+"CURSOR_1_OUT|"

        return temp

    def ctl_layer_ext2(self,data):
        temp=""
        value_3_0=int(str(data),16)&(15)
        if(value_3_0!=0): temp=temp+"DMA_2_OUT|"
        value_7_4=int(str(data),16)&(240)
        if(value_7_4!=0): temp=temp+"DMA_3_OUT|"
        value_11_8=int(str(data),16)&(3840)
        if(value_11_8!=0): temp=temp+"DMA_0_REC1_OUT|"
        value_15_12=int(str(data),16)&(61440)
        if(value_15_12!=0): temp=temp+"DMA_1_REC1_OUT|"
        value_19_16=int(str(data),16)&(983040)
        if(value_19_16!=0): temp=temp+"DMA_2_REC1_OUT|"
        value_23_20=int(str(data),16)&(1835008)
        if(value_23_20!=0): temp=temp+"DMA_3_REC1_OUT|"

        return temp

    def ctl_layer_ext3(self,data):
        temp=""
        value_3_0=int(str(data),16)&(15)
        if(value_3_0!=0): temp=temp+"VIG_0_REC1_OUT|"
        value_7_4=int(str(data),16)&(240)
        if(value_7_4!=0): temp=temp+"VIG_1_REC1_OUT|"
        value_11_8=int(str(data),16)&(3840)
        if(value_11_8!=0): temp=temp+"VIG_2_REC1_OUT|"
        value_15_12=int(str(data),16)&(61440)
        if(value_15_12!=0): temp=temp+"VIG_3_REC1_OUT|"

        return temp

    def vig_dma_sspp_status(self,data):
        value_0=int(str(data),16)&(1)
        temp=""
        if(value_0!=0): temp=temp+"UNPACK_IDLE|"
        if(value_0==0): temp=temp+"UNPACK_BUSY|"
        value_1=int(str(data),16)&(2)
        if(value_1!=0): temp=temp+"FETCH_IDLE|"
        if(value_1==0): temp=temp+"FETCH_BUSY|"
        value_4=int(str(data),16)&(16)
        if(value_4!=0): temp=temp+"C03_UNPACK_VALID|"
        if(value_4==0): temp=temp+"C03_UNPACK_NO_VALID|"
        value_5=int(str(data),16)&(32)
        if(value_5!=0): temp=temp+"C12_UNPACK_VALID|"
        if(value_5==0): temp=temp+"C12_UNPACK_NO_VALID|"
        value_8=int(str(data),16)&(256)
        if(value_8!=0): temp=temp+"C03_UNPACK_REQ_VALID|"
        if(value_8==0): temp=temp+"C03_UNPACK_REQ_NO_VALID|"
        value_9=int(str(data),16)&(512)
        if(value_9!=0): temp=temp+"C12_UNPACK_REQ_VALID|"
        if(value_9==0): temp=temp+"C12_UNPACK_REQ_NO_VALID|"
        value_16=int(str(data),16)&(65536)
        if(value_16!=0): temp=temp+"UBWC_IDLE|"
        if(value_16==0): temp=temp+"UBWC_BUSY|"

        return temp




    def sspp_status_sub(self,pipes,contents_for_analysis,register_name):
        for x in pipes:
            for k in  range(len(contents_for_analysis)):
                if (contents_for_analysis[k]["offset"] == x):                             #CTL_0:VIG0_STATUS:"\n"UNPACK_IDLE|FETCH_IDLE|UBWC_IDLE(do for busy also)
                    break


    def sspp_status(self,contents_for_analysis):
        for i in  range(len(contents_for_analysis)):
            if (contents_for_analysis[i]["reg_name"] == "ctl_0" or contents_for_analysis[i]["reg_name"] == "ctl_1" or contents_for_analysis[i]["reg_name"] == "ctl_2" or contents_for_analysis[i]["reg_name"] == "ctl_3" or contents_for_analysis[i]["reg_name"] == "ctl_4" or contents_for_analysis[i]["reg_name"] == "ctl_5" ):
                if (len(contents_for_analysis[i]["data"])<1):
                    return contents_for_analysis
                layer =[]
                layer.append(ctl_layer_ext0(contents_for_analysis[i]["data"][0]))
                layer.append(ctl_layer_ext0(contents_for_analysis[i]["data"][1]))
                layer.append(ctl_layer_ext0(contents_for_analysis[i]["data"][2]))
                layer.append(ctl_layer_ext0(contents_for_analysis[i]["data"][3]))
                layer.append(ctl_layer_ext0(contents_for_analysis[i]["data"][4]))
                layer.append(ctl_layer_ext0(contents_for_analysis[i]["data"][8]))

                layer.append(ctl_layer_ext1(contents_for_analysis[i]["data"][16]))
                layer.append(ctl_layer_ext1(contents_for_analysis[i]["data"][17]))
                layer.append(ctl_layer_ext1(contents_for_analysis[i]["data"][18]))
                layer.append(ctl_layer_ext1(contents_for_analysis[i]["data"][19]))
                layer.append(ctl_layer_ext1(contents_for_analysis[i]["data"][20]))
                layer.append(ctl_layer_ext1(contents_for_analysis[i]["data"][21]))

                layer.append(ctl_layer_ext2(contents_for_analysis[i]["data"][28]))
                layer.append(ctl_layer_ext2(contents_for_analysis[i]["data"][29]))
                layer.append(ctl_layer_ext2(contents_for_analysis[i]["data"][30]))
                layer.append(ctl_layer_ext2(contents_for_analysis[i]["data"][31]))
                layer.append(ctl_layer_ext2(contents_for_analysis[i]["data"][32]))
                layer.append(ctl_layer_ext2(contents_for_analysis[i]["data"][33]))

                layer.append(ctl_layer_ext3(contents_for_analysis[i]["data"][40]))
                layer.append(ctl_layer_ext3(contents_for_analysis[i]["data"][41]))
                layer.append(ctl_layer_ext3(contents_for_analysis[i]["data"][42]))
                layer.append(ctl_layer_ext3(contents_for_analysis[i]["data"][43]))
                layer.append(ctl_layer_ext3(contents_for_analysis[i]["data"][44]))
                layer.append(ctl_layer_ext3(contents_for_analysis[i]["data"][45]))

                pipes = set([])
                for j in range(len(layer)):
                    if "VIG_0" in layer[j] :
                        pipes.add("0x5000")

                    if "VIG_1" in layer[j] :
                        pipes.add("0x7000")

                    if  "VIG_2" in layer[j] :
                        pipes.add("0x9000")

                    if  "VIG_3" in layer[j] :
                        pipes.add("0xb000")

                    if  "DMA_0" in layer[j] :
                        pipes.add("0x25000")

                    if  "DMA_1" in layer[j] :
                        pipes.add("0x27000")

                    if  "DMA_2" in layer[j] :
                        pipes.add("0x29000")

                    if  "DMA_3" in layer[j] :
                        pipes.add("0x2b000")




                sspp_status_sub(sorted(pipes),contents_for_analysis,contents_for_analysis[i]["reg_name"])

    def flush_expansion(self,value,offset):
        used_pipes=" ==> MDP_CTL_FLUSH("+str(offset)+"): "
        if(value&1==1):used_pipes=used_pipes+"VIG0|"
        if(value&2==2):used_pipes=used_pipes+"VIG1|"
        if(value&4==4):used_pipes=used_pipes+"VIG2|"
        if(value&8==8):used_pipes=used_pipes+"dspp_pa_lutv_0|"
        if(value&16==16):used_pipes=used_pipes+"dspp_pa_lutv_1|"
        if(value&32==32):used_pipes=used_pipes+"dspp_pa_lutv_2|"
        if(value&64==64):used_pipes=used_pipes+"LM0|"
        if(value&128==128):used_pipes=used_pipes+"LM1|"
        if(value&256==256):used_pipes=used_pipes+"LM2|"
        if(value&512==512):used_pipes=used_pipes+"LM3|"
        if(value&1024==1024):used_pipes=used_pipes+"LM4|"
        if(value&2048==2048):used_pipes=used_pipes+"DMA0|"
        if(value&4096==4096):used_pipes=used_pipes+"DMA1|"
        if(value&8192==8129):used_pipes=used_pipes+"DSPP0|"
        if(value&16384==16384):used_pipes=used_pipes+"DSPP1|"
        if(value&32768==32768):used_pipes=used_pipes+"DSPP2|"
        if(value&65536==65536):used_pipes=used_pipes+"WB|"
        if(value&131072==131072):used_pipes=used_pipes+"CTL|"
        if(value&262144==262144):used_pipes=used_pipes+"VIG3|"
        if(value&524288==524288):used_pipes=used_pipes+"dspp_pa_lutv_3|"
        if(value&1048576==1048576):used_pipes=used_pipes+"LM5|"
        if(value&2097152==2097152):used_pipes=used_pipes+"DSPP3|"
        if(value&4194304==4194304):used_pipes=used_pipes+"DSC|"
        if(value&8388608==8388608):used_pipes=used_pipes+"MERGE3D|"
        if(value&16777216==16777216):used_pipes=used_pipes+"DMA2|"
        if(value&33554432==33554432):used_pipes=used_pipes+"DMA3|"
        if(value&67108864==67108864):used_pipes=used_pipes+"CDM0|"
        if(value&134217728==134217728):used_pipes=used_pipes+"ROT|"
        if(value&268435456==268435456):used_pipes=used_pipes+"CWB|"

        if(value&1073741824==1073741824):used_pipes=used_pipes+"PERIPH|"
        if(value&2147483648==2147483648):used_pipes=used_pipes+"INTIF"



        return used_pipes



    def ctl_top(self,value):
        temp=""

        value_3_0=int(str(value),16)&(15)
        if(value_3_0==1):
            temp=temp+"MEM_BLOCK_ROT0|"
        if(value_3_0==2):
            temp=temp+"MEM_BLOCK_ROT1|"
        if(value_3_0==3):
            temp=temp+"MEM_LINE_ROT0|"
        if(value_3_0==4):
            temp=temp+"MEM_LINE_ROT1|"
        if(value_3_0==5):
            temp=temp+"MEM_LINE_LINEAR_MEM|"



        value_7_4=int(str(value),16)&(240)
        if(value_7_4==16):
            temp=temp+"INTF0|"
        if(value_7_4==32):
            temp=temp+"INTF1|"
        if(value_7_4==48):
            temp=temp+"INTF2|"
        if(value_7_4==64):
            temp=temp+"INTF3|"
        if(value_7_4==80):
            temp=temp+"INTF4|"


        value_10_8=int(str(value),16)&(1792)
        if(value_10_8==0):
            temp=temp+"PP_BUFF0"
        if(value_10_8==256):
            temp=temp+"PP_BUFF1|"
        if(value_10_8==512):
            temp=temp+"PP_BUFF2|"
        if(value_10_8==1792):
            temp=temp+"NO_CON_PP_BUFF|"


        value_12_11=int(str(value),16)&(6144)
        if(value_12_11==0):
            temp=temp+"DSPP0"
        if(value_12_11==2048):
            temp=temp+"DSPP1|"
        if(value_10_8==4096):
            temp=temp+"DSPP2|"
        if(value_10_8==6144):
            temp=temp+"NO_DSPP|"



        return temp

    def merge_3d_wb(self,value):
        temp=""
        if(value&1==0): temp=temp+"0->NOT_ACTIVE|"
        if(value&1==1): temp=temp+"0->ACTIVE|"
        if(value&2==0): temp=temp+"1->NOT_ACTIVE|"
        if(value&2==2): temp=temp+"1->ACTIVE|"
        if(value&4==0): temp=temp+"2->NOT_ACTIVE|"
        if(value&4==4): temp=temp+"2->ACTIVE|"

        return temp

    def dsc_cwb_active(self,value):
        temp=""
        if(value&1==0): temp=temp+"0->NOT_ACTIVE|"
        if(value&1==1): temp=temp+"0->ACTIVE|"
        if(value&2==0): temp=temp+"1->NOT_ACTIVE|"
        if(value&2==2): temp=temp+"1->ACTIVE|"
        if(value&4==0): temp=temp+"2->NOT_ACTIVE|"
        if(value&4==4): temp=temp+"2->ACTIVE|"

        if(value&8==0): temp=temp+"3->NOT_ACTIVE|"
        if(value&8==8): temp=temp+"3->ACTIVE|"
        if(value&16==0): temp=temp+"4->NOT_ACTIVE|"
        if(value&16==16): temp=temp+"4->ACTIVE|"
        if(value&32==0): temp=temp+"5->NOT_ACTIVE|"
        if(value&32==32): temp=temp+"5->ACTIVE|"

        return temp



    def cdm_active(self,value):
        temp=""
        if(value&1==0): temp=temp+"0->NOT_ACTIVE|"
        if(value&1==1): temp=temp+"0->ACTIVE|"

        return temp


    def ctl_status(self,value):
        value_6_4= value&(112)
        value_6_4=value_6_4>>4
        tmp=""

        if(value_6_4==1):
            tmp=tmp+ "Wake up received and waiting for commit"
        if(value_6_4==2):
            tmp=tmp+"Wake up and ctl start received and waiting for lutdma busy to de-assert (lutdma complete)"
        if(value_6_4==3):
            tmp=tmp+"Frame start processing, waiting for done"
        if(value_6_4==4):
            tmp=tmp+"Wait for done in multiple updates"

        return tmp



    def ctl(self,contents_for_analysis):
        for i in  range(len(contents_for_analysis)):
            if (contents_for_analysis[i]["reg_name"] == "ctl_0" or contents_for_analysis[i]["reg_name"] == "ctl_1" or contents_for_analysis[i]["reg_name"] == "ctl_2" or contents_for_analysis[i]["reg_name"] == "ctl_3" or contents_for_analysis[i]["reg_name"] == "ctl_4" or contents_for_analysis[i]["reg_name"] == "ctl_5" ):
                if len(contents_for_analysis[i]["data"])<1:
                    return contents_for_analysis
                if(int(str(contents_for_analysis[i]["data"][7]),16)!=0):
                    self.ctl_flag=1


                length_for_printing=len(contents_for_analysis[i]["data"][0])*4+len(str(contents_for_analysis[i]["offset"]))+5

                contents_for_analysis[i]["data"][7]=str(contents_for_analysis[i]["data"][7])+self.flush_expansion(int(str(contents_for_analysis[i]["data"][6]),16),hex(int(str(contents_for_analysis[i]["offset"]),16)+6*4))
                if(int(str(contents_for_analysis[i]["data"][25]),16) == 1 and  int(str(contents_for_analysis[i]["data"][25])[-1:],16)==1 ):
                    #Do nothing
                    pass
                else:
                    self.ctl_flag=1


                x=' '

                if (int(contents_for_analysis[i]["data"][3],16) != 0):
                    contents_for_analysis[i]["data"][3]=contents_for_analysis[i]["data"][3]+"\n"+str(length_for_printing*x)+" ==> LM3("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+3*4))+"):"+self.ctl_layer_ext0(contents_for_analysis[i]["data"][3])
                if (int(contents_for_analysis[i]["data"][2],16) != 0):
                    contents_for_analysis[i]["data"][3]=contents_for_analysis[i]["data"][3]+"\n"+str(length_for_printing*x)+" ==> LM2("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+2*4))+"):"+self.ctl_layer_ext0(contents_for_analysis[i]["data"][2])
                if (int(contents_for_analysis[i]["data"][1],16) != 0):
                    contents_for_analysis[i]["data"][3]=contents_for_analysis[i]["data"][3]+"\n"+str(length_for_printing*x)+" ==> LM1("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+1*4))+"):"+self.ctl_layer_ext0(contents_for_analysis[i]["data"][1])
                if (int(contents_for_analysis[i]["data"][0],16) != 0):
                    contents_for_analysis[i]["data"][3]=contents_for_analysis[i]["data"][3]+"\n"+str(length_for_printing*x)+" ==> LM0("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+0*4))+"):"+self.ctl_layer_ext0(contents_for_analysis[i]["data"][0])
                if (int(contents_for_analysis[i]["data"][4],16) != 0):
                    contents_for_analysis[i]["data"][7]=contents_for_analysis[i]["data"][7]+"\n"+str(length_for_printing*x)+" ==> LM4("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+4*4))+"):"+self.ctl_layer_ext0(contents_for_analysis[i]["data"][4])
                if (int(contents_for_analysis[i]["data"][8],16) != 0):
                    contents_for_analysis[i]["data"][11]=contents_for_analysis[i]["data"][11]+"\n"+str(length_for_printing*x)+" ==> LM5("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+8*4))+"):"+self.ctl_layer_ext0(contents_for_analysis[i]["data"][8])

                contents_for_analysis[i]["data"][7]=contents_for_analysis[i]["data"][7]+"\n"+str(length_for_printing*x)+" ==> TOP("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+5*4))+"):"+self.ctl_top(contents_for_analysis[i]["data"][5])


                if (int(contents_for_analysis[i]["data"][19],16) != 0):
                    contents_for_analysis[i]["data"][19]=contents_for_analysis[i]["data"][19]+"\n"+str(length_for_printing*x)+" ==> LM3_EXT("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+19*4))+"):"+self.ctl_layer_ext1(contents_for_analysis[i]["data"][19])
                if (int(contents_for_analysis[i]["data"][18],16) != 0):
                    contents_for_analysis[i]["data"][19]=contents_for_analysis[i]["data"][19]+"\n"+str(length_for_printing*x)+" ==> LM2_EXT("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+18*4))+"):"+self.ctl_layer_ext1(contents_for_analysis[i]["data"][18])
                if (int(contents_for_analysis[i]["data"][17],16) != 0):
                    contents_for_analysis[i]["data"][19]=contents_for_analysis[i]["data"][19]+"\n"+str(length_for_printing*x)+" ==> LM1_EXT("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+17*4))+"):"+self.ctl_layer_ext1(contents_for_analysis[i]["data"][17])
                if (int(contents_for_analysis[i]["data"][16],16) != 0):
                    contents_for_analysis[i]["data"][19]=contents_for_analysis[i]["data"][19]+"\n"+str(length_for_printing*x)+" ==> LM0_EXT("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+16*4))+"):"+self.ctl_layer_ext1(contents_for_analysis[i]["data"][16])
                if (int(contents_for_analysis[i]["data"][20],16) != 0):
                    contents_for_analysis[i]["data"][23]=contents_for_analysis[i]["data"][23]+"\n"+str(length_for_printing*x)+" ==> LM4_EXT("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+20*4))+"):"+self.ctl_layer_ext1(contents_for_analysis[i]["data"][20])
                if (int(contents_for_analysis[i]["data"][21],16) != 0):
                    contents_for_analysis[i]["data"][23]=contents_for_analysis[i]["data"][23]+"\n"+str(length_for_printing*x)+" ==> LM5_EXT("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+21*4))+"):"+self.ctl_layer_ext1(contents_for_analysis[i]["data"][21])


                if (int(contents_for_analysis[i]["data"][31],16) != 0):
                    contents_for_analysis[i]["data"][31]=contents_for_analysis[i]["data"][31]+"\n"+str(length_for_printing*x)+" ==> LM3_EXT2("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+31*4))+"):"+self.ctl_layer_ext2(contents_for_analysis[i]["data"][31])
                if (int(contents_for_analysis[i]["data"][30],16) != 0):
                    contents_for_analysis[i]["data"][31]=contents_for_analysis[i]["data"][31]+"\n"+str(length_for_printing*x)+" ==> LM2_EXT2("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+30*4))+"):"+self.ctl_layer_ext2(contents_for_analysis[i]["data"][30])
                if (int(contents_for_analysis[i]["data"][29],16) != 0):
                    contents_for_analysis[i]["data"][31]=contents_for_analysis[i]["data"][31]+"\n"+str(length_for_printing*x)+" ==> LM1_EXT2("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+29*4))+"):"+self.ctl_layer_ext2(contents_for_analysis[i]["data"][29])
                if (int(contents_for_analysis[i]["data"][28],16) != 0):
                    contents_for_analysis[i]["data"][31]=contents_for_analysis[i]["data"][31]+"\n"+str(length_for_printing*x)+" ==> LM0_EXT2("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+28*4))+"):"+self.ctl_layer_ext2(contents_for_analysis[i]["data"][28])
                if (int(contents_for_analysis[i]["data"][32],16) != 0):
                    contents_for_analysis[i]["data"][35]=contents_for_analysis[i]["data"][35]+"\n"+str(length_for_printing*x)+" ==> LM4_EXT2("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+32*4))+"):"+self.ctl_layer_ext2(contents_for_analysis[i]["data"][32])
                if (int(contents_for_analysis[i]["data"][33],16) != 0):
                    contents_for_analysis[i]["data"][35]=contents_for_analysis[i]["data"][35]+"\n"+str(length_for_printing*x)+" ==> LM5_EXT2("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+33*4))+"):"+self.ctl_layer_ext2(contents_for_analysis[i]["data"][33])


                if (int(contents_for_analysis[i]["data"][43],16) != 0):
                    contents_for_analysis[i]["data"][43]=contents_for_analysis[i]["data"][43]+"\n"+str(length_for_printing*x)+" ==> LM3_EXT3("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+43*4))+"):"+self.ctl_layer_ext3(contents_for_analysis[i]["data"][43])
                if (int(contents_for_analysis[i]["data"][42],16) != 0):
                    contents_for_analysis[i]["data"][43]=contents_for_analysis[i]["data"][43]+"\n"+str(length_for_printing*x)+" ==> LM2_EXT3("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+42*4))+"):"+self.ctl_layer_ext3(contents_for_analysis[i]["data"][42])
                if (int(contents_for_analysis[i]["data"][41],16) != 0):
                    contents_for_analysis[i]["data"][43]=contents_for_analysis[i]["data"][43]+"\n"+str(length_for_printing*x)+" ==> LM1_EXT3("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+41*4))+"):"+self.ctl_layer_ext3(contents_for_analysis[i]["data"][41])
                if (int(contents_for_analysis[i]["data"][40],16) != 0):
                    contents_for_analysis[i]["data"][43]=contents_for_analysis[i]["data"][43]+"\n"+str(length_for_printing*x)+" ==> LM0_EXT3("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+40*4))+"):"+self.ctl_layer_ext3(contents_for_analysis[i]["data"][40])
                if (int(contents_for_analysis[i]["data"][44],16) != 0):
                    contents_for_analysis[i]["data"][47]=contents_for_analysis[i]["data"][47]+"\n"+str(length_for_printing*x)+" ==> LM4_EXT3("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+44*4))+"):"+self.ctl_layer_ext3(contents_for_analysis[i]["data"][44])
                if (int(contents_for_analysis[i]["data"][45],16) != 0):
                    contents_for_analysis[i]["data"][47]=contents_for_analysis[i]["data"][47]+"\n"+str(length_for_printing*x)+" ==> LM5_EXT3("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+45*4))+"):"+self.ctl_layer_ext3(contents_for_analysis[i]["data"][45])


                contents_for_analysis[i]["data"][59] =  contents_for_analysis[i]["data"][59]+"\n"+str(length_for_printing*x)+" ==>WB "+self.merge_3d_wb(int(contents_for_analysis[i]["data"][59],16))
                contents_for_analysis[i]["data"][59] =  contents_for_analysis[i]["data"][59]+"\n"+str(length_for_printing*x)+" ==>DSC "+self.dsc_cwb_active(int(contents_for_analysis[i]["data"][58],16))
                contents_for_analysis[i]["data"][59] =  contents_for_analysis[i]["data"][59]+"\n"+str(length_for_printing*x)+" ==>MERGE_3D "+self.merge_3d_wb(int(contents_for_analysis[i]["data"][57],16))
                contents_for_analysis[i]["data"][63] =  contents_for_analysis[i]["data"][63]+"\n"+str(length_for_printing*x)+" ==>CWB "+self.dsc_cwb_active(int(contents_for_analysis[i]["data"][60],16))
                contents_for_analysis[i]["data"][63] =  contents_for_analysis[i]["data"][63]+"\n"+str(length_for_printing*x)+" ==>CDM "+self.cdm_active(int(contents_for_analysis[i]["data"][62],16))

                if(int(str(contents_for_analysis[i]["data"][25]),16) == 1 and  int(str(contents_for_analysis[i]["data"][25])[-1:],16)==1 ):
                    pass
                else:
                    value_6_4= int(str(contents_for_analysis[i]["data"][25]),16)&(112)
                    value_6_4=value_6_4>>4
                    tmp=" ==>CTL_STATUS("+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+25*4))+"): "
                    if(value_6_4==1):
                        tmp=tmp+ "Wake up received and waiting for commit|"
                    if(value_6_4==2):
                        tmp=tmp+"Wake up and ctl start received and waiting for lutdma busy to de-assert (lutdma complete)|"
                    if(value_6_4==3):
                        tmp=tmp+"Frame start processing, waiting for done|"
                    if(value_6_4==4):
                        tmp=tmp+"Wait for done in multiple updates|"
                    contents_for_analysis[i]["data"][27]=str(contents_for_analysis[i]["data"][27])+tmp
        if(self.ctl_flag==1):
            self.ctl_flag=0
        return contents_for_analysis

    def rsc(self,contents_for_analysis):

        for i in range(len(contents_for_analysis)):
            if("rsc_drv" in contents_for_analysis[i]["reg_name"] and len(contents_for_analysis[i]["data"])>1):

                contents_for_analysis[i]["data"][259]=contents_for_analysis[i]["data"][259]+"==> SDE_RSCC_RSC_SEQ_PROGRAM_COUNTER_DRV:"+str(hex(int(str(contents_for_analysis[i]["offset"]),16)+259*4))+"|"+str(int(str(contents_for_analysis[i]["data"][258]),16))
        return contents_for_analysis

    def idle_clients(self,value,offset):
        length_for_printing=(8*4)+10
        x= ' '
        error="\n"+str(length_for_printing*x)+" ==> Clients Busy("+offset+"): "
        value=value>>16
        if(value&1!=1):
            error=error+"client0|"
        if(value&2!=2):
            error=error+"client1|"
        if(value&4!=4):
            error=error+"client2|"
        if(value&8!=8):
            error=error+"client3|"
        if(value&16!=16):
            error=error+"client4|"
        if(value&32!=32):
            error=error+"client5|"
        if(value&64!=64):
            error=error+"client6|"
        if(value&128!=128):
            error=error+"client7|"
        if(value&256!=256):
            error=error+"client8|"
        if(value&512!=512):
            error=error+"client9|"
        if(value&1024!=1024):
            error=error+"client10|"
        if(value&2048!=2048):
            error=error+"client11|"
        if(value&4096!=4096):
            error=error+"client12|"
        if(value&8192!=8192):
            error=error+"client13|"
        return error

    def idle_axi(self,value,offset):
        length_for_printing=(8*4)+10
        x= ' '
        non_idle="\n"+str(length_for_printing*x)+" ==> AXI_BUSY("+offset+"): "
        if(value&16!=16):
            non_idle=non_idle+"A1|"
        if(value&32!=32):
            non_idle=non_idle+"A0|"
        return non_idle

    def client_error_pnd(self,value,offset):
        error = "==> PND_ERR("+offset+"): "
        if(value&1!=1):
            error=error+"client0|"
        if(value&2!=2):
            error=error+"client1|"
        if(value&4!=4):
            error=error+"client2|"
        if(value&8!=8):
            error=error+"client3|"
        if(value&16!=16):
            error=error+"client4|"
        if(value&32!=32):
            error=error+"client5|"
        if(value&64!=64):
            error=error+"client6|"
        if(value&128!=128):
            error=error+"client7|"
        if(value&256!=256):
            error=error+"client8|"
        if(value&512!=512):
            error=error+"client9|"
        if(value&1024!=1024):
            error=error+"client10|"
        if(value&2048!=2048):
            error=error+"client11|"
        if(value&4096!=4096):
            error=error+"client12|"
        if(value&8192!=8192):
            error=error+"client13|"
        return error

    def client_error_src(self,value,offset):
        error = "==> SRC_ERR("+offset+"): "
        if(value&1!=1):
            error=error+"client0|"
        if(value&2!=2):
            error=error+"client1|"
        if(value&4!=4):
            error=error+"client2|"
        if(value&8!=8):
            error=error+"client3|"
        if(value&16!=16):
            error=error+"client4|"
        if(value&32!=32):
            error=error+"client5|"
        if(value&64!=64):
            error=error+"client6|"
        if(value&128!=128):
            error=error+"client7|"
        if(value&256!=256):
            error=error+"client8|"
        if(value&512!=512):
            error=error+"client9|"
        if(value&1024!=1024):
            error=error+"client10|"
        if(value&2048!=2048):
            error=error+"client11|"
        if(value&4096!=4096):
            error=error+"client12|"
        if(value&8192!=8192):
            error=error+"client13|"
        return error

    def error_info(self,value,offset):
        length_for_printing=(8*4)+len(str(offset))+5
        x= ' '
        error= "\n"+str(length_for_printing*x)+" ==> VBIF_ERR_INFO("+offset+"): "
        if(value&15!=0):
            error=error+"CLIENT_SEL:"+str(value&15)+"|"
        if(value&112!=0):
            error=error+"ERR_TYPE:"+str(value&112)+"|"
        if(value&896!=0):
            error=error+"AXI_RESP:"+str(value&896)+"|"
        if(value&261120!=0):
            error=error+"CLIENT_MID:"+str(value&261120)+"|"
        if(value&262144!=0):
            error=error+"CLIENT_TID:"+str(value&262144)+"|"
        return error

    def error_info_ubwc(self,value,offset):
        length_for_printing=(8*4)+len(str(offset))+5
        x= ' '
        error="\n"+str(length_for_printing*x)+" ==> VBIF_ERR_INFO_1("+offset+"): "
        if(value&255 !=0):error =error+"UBWC_ERR_TYPE:"+str(value&255)+"|"
        return error

    def vbif(self,contents_for_analysis):
        for i in range(len(contents_for_analysis)):
            if("vbif_rt" in contents_for_analysis[i]["reg_name"] and len(contents_for_analysis[i]["data"])>1):
                if(int(str(contents_for_analysis[i]["data"][131]),16) &48!= 48):
                    contents_for_analysis[i]["data"][131]=str(contents_for_analysis[i]["data"][131])+self.idle_axi(int(str(contents_for_analysis[i]["data"][131]),16),hex(int(str(contents_for_analysis[i]["offset"]),16)+131*4))
                    self.vbif_flag=1
                if(int(str(contents_for_analysis[i]["data"][129]),16) &1073676288 != 1073676288):
                    contents_for_analysis[i]["data"][131]=str(contents_for_analysis[i]["data"][131])+self.idle_clients(int(str(contents_for_analysis[i]["data"][129]),16),hex(int(str(contents_for_analysis[i]["offset"]),16)+129*4))
                    self.vbif_flag=1
                if(int(str(contents_for_analysis[i]["data"][100]),16)  != 0):
                    contents_for_analysis[i]["data"][103]=str(contents_for_analysis[i]["data"][103])+self.client_error_pnd(int(str(contents_for_analysis[i]["data"][100]),16),hex(int(str(contents_for_analysis[i]["offset"]),16)+100*4))
                    self.vbif_flag=1
                if(int(str(contents_for_analysis[i]["data"][101]),16)  != 0):
                    contents_for_analysis[i]["data"][103]=str(contents_for_analysis[i]["data"][103])+self.client_error_src(int(str(contents_for_analysis[i]["data"][101]),16),hex(int(str(contents_for_analysis[i]["offset"]),16)+101*4))
                    self.vbif_flag=1
                if(int(str(contents_for_analysis[i]["data"][104]),16)  != 0):
                    contents_for_analysis[i]["data"][107]=str(contents_for_analysis[i]["data"][107])+self.error_info(int(str(contents_for_analysis[i]["data"][104]),16),hex(int(str(contents_for_analysis[i]["offset"]),16)+104*4))
                    self.vbif_flag=1
                if(int(str(contents_for_analysis[i]["data"][105]),16)  != 0):
                    contents_for_analysis[i]["data"][107]=str(contents_for_analysis[i]["data"][107])+self.error_info_ubwc(int(str(contents_for_analysis[i]["data"][105]),16),hex(int(str(contents_for_analysis[i]["offset"]),16)+105*4))
                    self.vbif_flag=1
        if(self.vbif_flag==1):
            self.vbif_flag=0
        return contents_for_analysis

    def vig_dma_sspp_status_expansion(self,data,offset):
        value_0=int(str(data),16)&(1)
        temp=" ==> SSPP_STATUS:"+offset+"|"
        if(value_0!=0): temp=temp+"UNPACK_IDLE|"
        value_1=int(str(data),16)&(2)
        if(value_1!=0): temp=temp+"FETCH_IDLE|"
        value_4=int(str(data),16)&(16)
        if(value_4!=0): temp=temp+"C03_UNPACK_VALID|"
        value_5=int(str(data),16)&(32)
        if(value_5!=0): temp=temp+"C12_UNPACK_VALID|"
        value_8=int(str(data),16)&(256)
        if(value_8!=0): temp=temp+"C03_UNPACK_REQ |"
        value_9=int(str(data),16)&(512)
        if(value_9!=0): temp=temp+"C12_UNPACK_REQ |"
        value_16=int(str(data),16)&(65536)
        if(value_16!=0): temp=temp+"UBWC_IDLE|"
        return temp

    def ubwc_decode_errors(self,value,offset):
        if value!=0:
            temp=" ==> SSPP_UBWC_ERROR_STATUS:"+offset+"|"
            if(value&255!=0): temp=temp+"Code:"+(value&255)+"|"
            if(value&16711680!=0): temp=temp+"Error_Count:"+(value&16711680)+"|"
            if(value&2147483648!=0): temp=temp+"Code_Clear:"+(value&2147483648)+"|"
            return temp
        return ""

    def src_format(self,value):
        temp= " ==> "
        if(value&32768==0): temp=temp+"COLOR_SPACE:RGB|"
        if(value&32768==1): temp=temp+"COLOR_SPACE:YUV|"
        if(value&2048==1): temp=temp+ "ROT90|"
        if(value&3221225472== 0): temp=temp+"FRAME_FORMAT:Linear|"
        if(value&3221225472==3221225472): temp=temp+"FRAME_FORMAT:unused|"
        elif(value&1073741824==1073741824): temp=temp+"FRAME_FORMAT:Macrotile_A4x|"
        elif(value&2147483648==2147483648): temp=temp+"FRAME_FORMAT:Macrotile_A5x|"
        if(value&58720256==0):  temp=temp+"CHROMA_SAMP:NO_SAMP|"
        elif(value&25165824==25165824):  temp=temp+"CHROMA_SAMP:VER_HORI_SAMP|"
        elif(value&8388608==8388608):  temp=temp+"CHROMA_SAMP:HORI_SAMP|"
        elif(value&16777216==16777216):  temp=temp+"CHROMA_SAMP:VER_SAMP|"
        if(value&1572864==0): temp=temp+"Fetch_Plane:InterLeaved|"
        elif(value&524288==524288): temp=temp+"Fetch_Plane:Planar|"
        elif(value&1048576==1048576): temp=temp+"Fetch_Plane:PsudoPlanar|"
        bpp_temp1=str(bin(value).replace("0b", "") )
        bpp_temp=bpp_temp1[::-1]
        if(int(bpp_temp,2)!=0):
            bpp_temp2=""+bpp_temp[16]+bpp_temp[10]+bpp_temp[9]
            if(int(bpp_temp2,2)<4):
                temp=temp+"SRC_BPP:" +str(int(bpp_temp2,2)+1)+" bytes"
            else:
                temp=temp+"SRC_BPP:" +str(int(bpp_temp2,2))+" bytes"
        else:
            temp=temp+"SRC_BPP:0 bytes"
        return temp

    def sspp_dma_vig(self,contents_for_analysis):
        pipes = set([])
        pipes.add("0x5000")
        pipes.add("0x7000")
        pipes.add("0x9000")
        pipes.add("0xb000")
        pipes.add("0x25000")
        pipes.add("0x27000")
        pipes.add("0x29000")
        pipes.add("0x2b000")
        for x in pipes:
            for k in  range(len(contents_for_analysis)):
                if (contents_for_analysis[k]["offset"] == x and len(contents_for_analysis[k]["data"])>1):
                    contents_for_analysis[k]["data"][63]= str(contents_for_analysis[k]["data"][63])+self.vig_dma_sspp_status_expansion(int(str(contents_for_analysis[k]["data"][61]),16),hex(int(str(contents_for_analysis[k]["offset"]),16)+61*4))
                    contents_for_analysis[k]["data"][79]= str(contents_for_analysis[k]["data"][79])+self.ubwc_decode_errors(int(str(contents_for_analysis[k]["data"][78]),16),hex(int(str(contents_for_analysis[k]["offset"]),16)+78*4))
                    contents_for_analysis[k]["data"][55]= str(contents_for_analysis[k]["data"][55])+" ==> write_counter_0:"+str(int(str(contents_for_analysis[k]["data"][52]),16))+"|write_counter_1:"+str(int(str(contents_for_analysis[k]["data"][53]),16))+"|write_counter_2:"+str(int(str(contents_for_analysis[k]["data"][54]),16))
                    contents_for_analysis[k]["data"][59]= str(contents_for_analysis[k]["data"][59])+" ==> read_counter_0:"+str(int(str(contents_for_analysis[k]["data"][56]),16))+"|read_counter_1:"+str(int(str(contents_for_analysis[k]["data"][57]),16))+"|read_counter_2:"+str(int(str(contents_for_analysis[k]["data"][58]),16))
                    contents_for_analysis[k]["data"][11]= str(contents_for_analysis[k]["data"][11])+" ==> ADDR_0: 0x"+str(contents_for_analysis[k]["data"][5])+" ADDR_1 0x:"+str(contents_for_analysis[k]["data"][6])+" ADDR_2 0x:"+str(contents_for_analysis[k]["data"][7])+" ADDR_3 0x:"+str(contents_for_analysis[k]["data"][8])
                    contents_for_analysis[k]["data"][7]= str(contents_for_analysis[k]["data"][7])+" ==> DST ["+str(int(str((contents_for_analysis[k]["data"][4])[4:8]),16))+" "+str(int(str((contents_for_analysis[k]["data"][4])[0:4]),16))+" "+str(int(str((contents_for_analysis[k]["data"][3])[4:8]),16))+" "+str(int(str((contents_for_analysis[k]["data"][3])[0:4]),16))+"]"
                    contents_for_analysis[k]["data"][3]= str(contents_for_analysis[k]["data"][3])+" ==> SRC ["+str(int(str((contents_for_analysis[k]["data"][2])[4:8]),16))+" "+str(int(str((contents_for_analysis[k]["data"][2])[0:4]),16))+" "+str(int(str((contents_for_analysis[k]["data"][0])[4:8]),16))+" "+str(int(str((contents_for_analysis[k]["data"][0])[0:4]),16))+"]"
                    contents_for_analysis[k]["data"][15]= str(contents_for_analysis[k]["data"][15])+self.src_format(int(str(contents_for_analysis[k]["data"][12]),16))
        return contents_for_analysis

    def throttling_error(self,value,offset,reg_name):
        #Throttling Error found. Panel Underrun occurred.
        #Timing Engine Status : Enabled/Disabled
        if(value!=0):
            temp=" ==> INTF_STATUS("+str(offset)+"): "
            if(value&1!=0):
                temp=temp+"Timing_engine_status_enable|"
            if(value&2!=0): temp=temp+"Throttiling_error|"
            if(value&4!=0): temp=temp+"error_clear|"
            return temp
        return ""

    def intf_mux(self,value,offset):
        temp=" ==> SEL("+str(offset)+"): "
        if(value&15==0): temp=temp+"PP_0|"
        if(value&15==1): temp=temp+"PP_1|"
        if(value&15==2): temp=temp+"PP_2|"
        if(value&15==3): temp=temp+"PP_3|"
        if(value&15==4): temp=temp+"PP_4|"
        if(value&15==5): temp=temp+"PP_5|"
        if(value&15==6): temp=temp+"PP_SPLIT|"
        if(value&15==15): temp=temp+"DIS|"
        temp=temp+" ==> SPLIT_SEL: "
        if(value&983040==0): temp=temp+"PP_0|"
        if(value&983040==65536): temp=temp+"PP_1|"
        if(value&983040==131072): temp=temp+"PP_2|"
        if(value&983040==196608): temp=temp+"PP_3|"
        if(value&983040==262144): temp=temp+"PP_4|"
        if(value&983040==327680): temp=temp+"PP_5|"
        if(value&983040==393216): temp=temp+"PP_SPLIT|"
        if(value&983040==983040): temp=temp+"DIS|"
        return temp

    def intf(self,contents_for_analysis):
        for k in range(len(contents_for_analysis)):
            if("intf" in contents_for_analysis[k]["reg_name"]):
                contents_for_analysis[k]["data"][47]= str(contents_for_analysis[k]["data"][47])+" ==> LINE_COUNT("+str(hex(int(str(contents_for_analysis[k]["offset"]),16)+44*4))+"): "+str(int(str(contents_for_analysis[k]["data"][44]),16))
                contents_for_analysis[k]["data"][43]= str(contents_for_analysis[k]["data"][43])+" ==> FRAME_COUNT("+str(hex(int(str(contents_for_analysis[k]["offset"]),16)+43*4))+"): "+str(int(str(contents_for_analysis[k]["data"][43]),16))
                contents_for_analysis[k]["data"][155]= str(contents_for_analysis[k]["data"][155])+self.throttling_error(int(str(contents_for_analysis[k]["data"][155]),16),hex(int(str(contents_for_analysis[k]["offset"]),16)+155*4),contents_for_analysis[k]["reg_name"])
                contents_for_analysis[k]["data"][151]= str(contents_for_analysis[k]["data"][151])+self.intf_mux(int(str(contents_for_analysis[k]["data"][151]),16),hex(int(str(contents_for_analysis[k]["offset"]),16)+151*4))
        return contents_for_analysis

    def file_write(self, contents_for_analysis,output_file):
        self.outfile= self.ramdump.open_file(output_file,"w+")
        for  i in range(len(contents_for_analysis)):
            self.outfile.write("["+contents_for_analysis[i]["reg_name"]+"] "+contents_for_analysis[i]["offset"])
            self.outfile.write("\n")
            counter_for_printing=0
            self.outfile.write(contents_for_analysis[i]["offset"]+"| ")
            offset_inc=16
            for k in range(len(contents_for_analysis[i]["data"])):
                if(counter_for_printing==4):
                    self.outfile.write("\n")
                    self.outfile.write(hex(int(contents_for_analysis[i]["offset"],16)+offset_inc)+"| ")
                    offset_inc=offset_inc+16
                    counter_for_printing=0
                counter_for_printing=counter_for_printing+1
                self.outfile.write(contents_for_analysis[i]["data"][k])
                self.outfile.write(" ")
            self.outfile.write("\n")
        self.outfile.close()

    def file_writex(contents_for_analysis,output_file):
        f_out= open(output_file,"w+")
        for  i in range(len(contents_for_analysis)):
            if "0x5000" in contents_for_analysis[i]["offset"].strip():
                f_out.write("[ViG0] ")
            elif "0x7000" in contents_for_analysis[i]["offset"]:
                f_out.write("[ViG1] ")
            elif "0x9000" in contents_for_analysis[i]["offset"]:
                f_out.write("[ViG2] ")
            elif "0xb000" in contents_for_analysis[i]["offset"]:
                f_out.write("[ViG3] ")
            elif "0x25000" in contents_for_analysis[i]["offset"]:
                f_out.write("[DMA0] ")
            elif "0x27000" in contents_for_analysis[i]["offset"]:
                f_out.write("[DMA1] ")
            elif "0x29000" in contents_for_analysis[i]["offset"]:
                f_out.write("[DMA2] ")
            elif "0x2b000" in contents_for_analysis[i]["offset"]:
                f_out.write("[DMA3] ")
            elif "intf_0" in contents_for_analysis[i]["reg_name"]:
                f_out.write("[INTF0] ")
            elif "intf_1" in contents_for_analysis[i]["reg_name"]:
                f_out.write("[INTF1] ")
            elif "intf_2" in contents_for_analysis[i]["reg_name"]:
                f_out.write("[INTF2] ")
            elif "intf_3" in contents_for_analysis[i]["reg_name"]:
                f_out.write("[INTF3] ")
            else:
                f_out.write("["+contents_for_analysis[i]["reg_name"]+"] ")
            for k in range(len(contents_for_analysis[i]["data"])):
                f_out.write(contents_for_analysis[i]["data"][k])
                f_out.write(" ")
            f_out.write("\n")
        f_out.close()

    def dest_scalar(self,value):
        temp=""
        if (value&1==0):
            temp=temp+"QSEED3_DIS|"
        if(value&1==1):
            temp=temp+"QSEED3_EN|"
        if (value&16==0):
            temp=temp+"DirDet_DIS|"
        if(value&16==1):
            temp=temp+"DirDet_EN|"
        if (value&256==0):
            temp=temp+"DE_DIS|"
        if(value&256==1):
            temp=temp+"DE_EN|"
        if (value&16==1024):
            temp=temp+"ALPHA_DIS|"
        if(value&16==1024):
            temp=temp+"ALPHA_EN|"
        if (value&4096==0):
            temp=temp+"RGB_MOD|"
        if(value&4096==1):
            temp=temp+"YUV_MOD|"
        if (value&8192==0):
            temp=temp+"RGB_DYN_EXP_DIS|"
        if(value&8192==1):
            temp=temp+"RGB_DYN_EXP_EN|"
        if(value&16384==0):
            temp=temp+"BIT_WIDTH:10bits|"
        if(value&16384==1):
            temp=temp+"BIT_WIDTH:8bits|"
        return temp


    def ds0(self,contents_for_analysis):
        for k in range(len(contents_for_analysis)):
            if("ds_0" in contents_for_analysis[k]["reg_name"] and len(contents_for_analysis[k]["data"])>1):
                contents_for_analysis[k]["data"][3]= str(contents_for_analysis[k]["data"][3])+" ==> DEST_SCALAR_OP_MODE("+str(hex(int(str(contents_for_analysis[k]["offset"]),16)+1*4))+"): "+self.dest_scalar(int(str(contents_for_analysis[k]["data"][1]),16))
        return contents_for_analysis

    def reg_verbose(self, contents_for_analysis):
        contents_for_analysis=self.ctl(contents_for_analysis)
        contents_for_analysis=self.rsc(contents_for_analysis)
        contents_for_analysis=self.vbif(contents_for_analysis)
        contents_for_analysis=self.sspp_dma_vig(contents_for_analysis)
        contents_for_analysis=self.intf(contents_for_analysis)
        contents_for_analysis=self.ds0(contents_for_analysis)
        return contents_for_analysis

    def check_rc_hang(self):
        try:
            dbgbus = self.ramdump.open_file('sde_dbgbus.txt', 'r')
            lines = dbgbus.readlines()
            rc0_debugbus_point = '00000348 00000034 00000002'
            rc1_debugbus_point = '00000348 00000035 00000002'
            for eachLine in lines:
                if rc0_debugbus_point in eachLine or rc1_debugbus_point in eachLine:
                    rc = eachLine
                    rc_data=rc.split(' ')
                    if len(rc_data) == 6 and (int(rc_data[4], 16) & int('6000000', 16)) == int('6000000',16):
                        return True
        except:
            pass
        return False

    def check_LTM_status(self):
        try:
            reg_dump = self.ramdump.open_file('sde_regdump.txt', 'r')
            lines = reg_dump.readlines()
            ltm0_status_reg = '0x6a360'
            ltm1_status_reg = '0x6b360'
            ltm2_status_reg = '0x6c360'
            ltm3_status_reg = '0x6d360'
            for eachLine in lines:
                if ltm0_status_reg in eachLine or ltm1_status_reg in eachLine or ltm2_status_reg in eachLine or ltm3_status_reg in eachLine:
                    ltm = eachLine
                    ltm_data = ltm.split(' ')
                    if len(ltm_data) == 6 and (int(ltm_data[2], 16) & int('300', 16)) > 0:
                        return True
        except:
            pass
        return False

    def check_LUTDMA_hang(self):
        try:
            reg_dump = self.ramdump.open_file('sde_regdump_parsed.txt', 'r')
            lines = reg_dump.readlines()
            LUT_regsister_dump = 'reg_dma'
            reg_dma0_status = '--NA--'
            reg_dma1_status = '--NA--'
            ctl0 = '0x16060'
            ctl1 = '0x17060'
            ctl2 = '0x18060'
            ctl3 = '0x19060'
            ctl4 = '0x1a060'
            ctl5 = '0x1b060'
            for eachLine in lines:
                if ctl0 in eachLine or ctl1 in eachLine or ctl2 in eachLine or ctl3 in eachLine or ctl4 in eachLine or ctl5 in eachLine:
                    ctl_data = eachLine.split(' ')
                    if len(ctl_data) == 6 and (int(ctl_data[2], 16) & int('20', 16)) > 0:
                        return True
                if LUT_regsister_dump in eachLine:
                    reg_dma_dump = True
                    reg_dma_start = eachLine.split(' ')
                    reg_dma0_status = hex(int(reg_dma_start[1],16) + int('0x170',16))
                    reg_dma1_status = hex(int(reg_dma_start[1],16) + int('0x570',16))
                elif reg_dma0_status in eachLine or reg_dma1_status in eachLine:
                    reg_dma_data = eachLine.split(' ')
                    if int(reg_dma_data[1],16) > 0:
                        return True
        except:
            pass
        return False

    def sde_initial_analysis(self):
        rc_hang = False
        LTM_busy = False
        LUTDMA_hang = False

        rc_hang = self.check_rc_hang()
        LTM_busy = self.check_LTM_status()
        LUTDMA_hang = self.check_LUTDMA_hang()
        try:
            self.file = self.ramdump.open_file('sde_evtlog_parsed.txt', 'r+')
            content = self.file.read()
            self.file.seek(0, 0)
            self.file.write('%s \n' % ("---------------------------------------  SDE INITIAL ANALYSIS  ---------------------------------------"))
            if rc_hang:
                self.file.write('%s \n' % ("RC hang observed\n"))
            else:
                self.file.write('%s \n' % ("No RC hang\n"))
            if LTM_busy:
                self.file.write('%s \n' % ("LTM Hist/WB busy\n"))
            else:
                self.file.write('%s \n' % ("No LTM busy\n"))
            if LUTDMA_hang:
                self.file.write('%s \n' % ("LUT DMA hang observed\n"))
            else:
                self.file.write('%s \n' % ("No LUT DMA hang\n"))
            self.file.write('%s \n' % ("------------------------------------------------------------------------------------------------------"))
            self.file.write(content)
            self.file.close()
        except:
            pass

    def parse(self):

        mdss_dbg = MdssDbgXlog(self.ramdump, 'mdss_dbg_xlog')

        if mdss_dbg.is_empty():
            mdss_dbg = SdeDbgBase(self.ramdump, 'sde_dbg_base')
            if mdss_dbg.is_empty():
                return
            # EVENT LOGS
            try:
                self.outfile = self.ramdump.open_file('sde_evtlog.txt')

                evt_log = Struct(self.ramdump, mdss_dbg.evtlog,
                          struct_name="struct sde_dbg_evtlog",
                          fields={'name': Struct.get_cstring,
                                  'enable': Struct.get_u32,
                                  'last': Struct.get_u32,
                                  'curr': Struct.get_u32,
                                  'logs': Struct.get_address,
                                  'last_dump': Struct.get_u32})

                SDE_EVTLOG_ENTRY = int(self.ramdump.field_offset('struct sde_dbg_evtlog', 'first') / self.ramdump.sizeof('struct sde_dbg_evtlog_log'))
                self.outfile.write('%-60.50s%-20.5s%-8.5s%-8.5s%s\n' % ("FUNC", "TIME", "PID", "CPU", "DATA"))
                sde_evtlog_start = 0
                sde_evtlog_repeat = 0
                sde_evtlog_curr = abs(evt_log.curr % SDE_EVTLOG_ENTRY)
                if (evt_log.curr != evt_log.last):
                        sde_evtlog_start = sde_evtlog_curr + 1
                else:
                        sde_evtlog_repeat = 1
                sde_evtlog_count = 0
                while(sde_evtlog_count < SDE_EVTLOG_ENTRY):
                        i = sde_evtlog_start % SDE_EVTLOG_ENTRY
                        if (i == sde_evtlog_curr):
                                if (sde_evtlog_repeat):
                                        break
                                else:
                                        sde_evtlog_repeat = 1
                        addr = evt_log.logs + self.ramdump.sizeof('struct sde_dbg_evtlog_log') * i
                        log_log = addr
                        sde_evtlog_start = sde_evtlog_start + 1
                        sde_evtlog_count = sde_evtlog_count + 1
                        log_log = Struct(self.ramdump, log_log,
                                         struct_name="struct sde_dbg_evtlog_log",
                                         fields={'name': Struct.get_pointer,
                                                 'cpu': Struct.get_u32,
                                                 'line': Struct.get_u32,
                                                 'data_cnt': Struct.get_u32,
                                                 'pid': Struct.get_u32,
                                                 'data' : Struct.get_address,
                                                 'time': get_u64})

                        if (log_log.time == 0x0):
                                continue
                        func_name = self.ramdump.read_cstring(log_log.name)
                        func_name = func_name + ':' + str(log_log.line)
                        self.outfile.write('%-60.50s' % (func_name))
                        self.outfile.write('%-20.0d' % (log_log.time))
                        self.outfile.write('%-8.1d' % (log_log.pid))
                        self.outfile.write('%-8.1d' % (log_log.cpu))
                        SDE_EVTLOG_MAX_DATA = 15

                        for i in range(min(log_log.data_cnt, SDE_EVTLOG_MAX_DATA)):
                                self.outfile.write('%x ' % (self.ramdump.read_u32(log_log.data+(i*4))))
                        self.outfile.write('\n' % ())
                self.outfile.close()

                #Event log Parser
                tmp_outfile = self.ramdump.open_file('sde_evtlog.txt', 'r')
                self.outfile = self.ramdump.open_file('evtlog_stage_1.txt')
                lines = tmp_outfile.readlines()

                for eachLine in lines:
                    if ':' in eachLine:
                        try:
                            eachLine1=re.sub(r'\s+', ' ', eachLine)
                            matchObj=re.match(r'([a-zA-Z0-9_\-\.]+)(:)([0-9]+\s)([0-9]+\s)([0-9]+\s)([0-9]+\s)(.*)',eachLine1)
                            line_new = '{:<50}  {:<10}  {:<15}  {:<15}  {:<15}  {:<100}'.format(matchObj.group(1), matchObj.group(3), matchObj.group(4), matchObj.group(5), matchObj.group(6), matchObj.group(7))
                            self.outfile.write(line_new)
                            self.outfile.write("\n")
                        except:
                            continue
                tmp_outfile.close()
                self.outfile.close()

                #Event log Verbose
                tmp_outfile = self.ramdump.open_file('evtlog_stage_1.txt', 'r')
                self.outfile = self.ramdump.open_file('evtlog_stage_2.txt')
                title = '{:<50}  {:<10}  {:<15}  {:<15}  {:<15}  {:<100}'.format('FUNCTION_NAME', "LINE_NO", "TIMESTAMP", "PID", "CPU", "DATA"  )
                self.outfile.write(title)
                self.outfile.write("\n")
                for line in tmp_outfile:
                    if not line.isspace():
                        self.outfile.write(line)
                self.outfile.close()
                tmp_outfile.close()

                self.outfile = self.ramdump.open_file('evtlog_stage_2.txt', 'r')
                self.outfile.readline()
                contents_list = []
                for line in self.outfile:
                    matchObj=re.match(r'([a-zA-Z0-9_\-\.]+)\s*([0-9]+)\s*([0-9]+)\s*([0-9]+)\s*([A-Z0-9]+)\s*(.*)',line)
                    sub_list=[matchObj.group(1).strip(),matchObj.group(2).strip(),matchObj.group(3).strip(),matchObj.group(4).strip(),matchObj.group(5).strip(),matchObj.group(6).strip()]
                    contents_list.append(sub_list)
                self.outfile.close()
                contents_list_for_analysis = contents_list
                contents_list.reverse()
                flag=0
                finalflag=0
                z=0

                for i in range(len(contents_list)):
                    if(i>=len(contents_list)):
                        break
                    if(contents_list[i][0]=="_sde_crtc_blend_setup_mixer") and flag==0:
                        k=contents_list[i][3]
                        while True:
                            i=i+1
                            z=i
                            if(i>=len(contents_list)):
                                contents_list.append((["\n-----------------------------------------------------------COMMIT SCHEDULED-------------------------------------------------------------------------"]))
                                flag=1
                                break
                            elif( contents_list[i][3]==k):
                                continue
                            else:
                                contents_list[i].extend(["\n-----------------------------------------------------------COMMIT SCHEDULED-------------------------------------------------------------------------"])
                                finalflag=0
                                flag=1
                                break

                    elif(contents_list[i][0]=="sde_encoder_virt_atomic_check") and flag==1 and len(contents_list[i][5].split())==2:
                        k=contents_list[i][3]
                        while True and flag==1:
                            i=i+1
                            if(i>=len(contents_list)):
                                break
                            elif(contents_list[i][0]=="sde_fence_create") and finalflag==0:
                                while True:
                                    i=i+1
                                    if(i>=len(contents_list)):
                                        contents_list.append((["---------------------------------------------------------------CHECK START---------------------------------------------------------------------------"]))
                                        flag=2
                                        break
                                    elif(contents_list[i][0]=="sde_fence_create"):
                                        continue
                                    else:
                                        contents_list[i].extend(["\n--------------------------------------------------------------CHECK START--------------------------------------------------------------------------"])

                                        flag=2
                                        break

                    elif(contents_list[i][0]=="sde_encoder_virt_atomic_check") and flag==2 and len(contents_list[i][5].split())==2:
                        k=contents_list[i][3]
                        while True:
                            i=i+1
                            if(i>=len(contents_list)):
                                contents_list.append((["------------------------------------------------------------CHECK ONLY START------------------------------------------------------------------------"]))
                                break
                            elif(contents_list[i][3]==k):
                                continue
                            else:
                                contents_list[i].extend(["\n-----------------------------------------------------------CHECK ONLY START-----------------------------------------------------------------------"])
                                flag=0
                                break

                    elif((contents_list[i][0]=="_sde_crtc_blend_setup_mixer") and flag==2):
                        flag=0

                    else:
                        continue

                contents_list.reverse()
                self.outfile=open('evtlog_stage_2.txt',"w")
                for i in range(len(contents_list)) :
                    for j in range(len(contents_list[i])):
                        self.outfile.write(contents_list[i][j])
                        self.outfile.write(" ")
                    self.outfile.write("\n")
                self.outfile.write("\n")
                self.outfile.close()

                self.split_dic('evtlog_stage_2.txt')
            except:
                pass
            self.ramdump.remove_file('evtlog_stage_1.txt')
            self.ramdump.remove_file('evtlog_stage_2.txt')

            #Dumps
            try:
                self.outfile = self.ramdump.open_file('sde_regdump.txt')
                dump_list = ListWalker(self.ramdump, mdss_dbg.reg_base_list, 0)
                if dump_list.is_empty():
                    self.outfile.write('%s \n' % ("BLK DUMPLIST IS EMPTY!!!"))
                    return
                for blk in dump_list:
                        reg_blk = Struct(self.ramdump, blk, struct_name="struct sde_dbg_reg_base",
                                fields={'name': Struct.get_cstring,
                                'base': Struct.get_pointer,
                                'max_offset': Struct.get_u32,
                                'sub_range_list': Struct.get_address,
                                'reg_dump': Struct.get_pointer})
                        if (reg_blk.base == 0x0000) or (reg_blk.name == ""):
                                continue
                        self.outfile.write('\n---------------------------------------------------------------------\n')
                        self.outfile.write('[%s]   0x%x\n' % (reg_blk.name, reg_blk.base))
                        self.outfile.write('---------------------------------------------------------------------\n')
                        headoffset_2 = self.ramdump.field_offset('struct sde_dbg_reg_range', 'head')
                        sub_blk_list = ListWalker(self.ramdump, reg_blk.sub_range_list, headoffset_2)

                        if sub_blk_list.is_empty():
                            self.mdss_dump_reg(reg_blk.base, reg_blk.max_offset, reg_blk.reg_dump, 0)
                        else:
                            for node in sub_blk_list:
                                self.print_sderange(node)
                #Dumps Parse
                self.outfile.close()
                f_in = self.ramdump.open_file('sde_regdump.txt', 'r')
                self.outfile = self.ramdump.open_file('reg_stage_1.txt', 'w')
                for eachLine in f_in:
                    if not eachLine.isspace() and "-----" not in eachLine and "======" not in eachLine and '["' not in eachLine :
                        self.outfile.write(eachLine)
                f_in.close()
                self.outfile.close()

                f_in = self.ramdump.open_file("reg_stage_1.txt", 'r')
                self.outfile = self.ramdump.open_file('reg_stage_2.txt','w')
                for eachLine in f_in:
                    if '[' in eachLine:
                        break
                self.outfile.write(eachLine)
                for eachLine in f_in:
                    self.outfile.write(eachLine)
                f_in.close()
                self.outfile.close()
                contents_for_analysis=self.reg_parser("reg_stage_2.txt")

                #Regdump Verbose
                contents_for_analysis=self.reg_verbose(contents_for_analysis)
                self.file_write(contents_for_analysis, "sde_regdump_parsed.txt")
            except:
                pass
            self.ramdump.remove_file('reg_stage_1.txt')
            self.ramdump.remove_file('reg_stage_2.txt')
            self.ramdump.remove_file('reg_stage_3.txt')
            try:
                self.outfile = self.ramdump.open_file('sde_reglog.txt')
                reg_log = Struct(self.ramdump, mdss_dbg.reglog,
                          struct_name="struct sde_dbg_reglog",
                          fields={'enable': Struct.get_u32,
                                  'last': Struct.get_u32,
                                  'curr': Struct.get_u64,
                                  'logs': Struct.get_address,
                                  'last_dump': Struct.get_u32})
                SDE_REGLOG_ENTRY = 1024
                self.outfile.write('%-21.5s%-11.5s%-13.7s%-13.5s%s\n' % ("TIME", "PID", "ADDRESS", "VAL", "BLK_ID"))
                sde_reglog_start = 0
                sde_reglog_repeat = 0
                sde_reglog_curr = abs(reg_log.curr % SDE_REGLOG_ENTRY)
                if (sde_reglog_curr != reg_log.last):
                    sde_reglog_start = sde_reglog_curr + 1
                else:
                    sde_reglog_repeat = 1
                sde_reglog_count = 0
                while (sde_reglog_count < SDE_REGLOG_ENTRY):
                    i = sde_reglog_start % SDE_REGLOG_ENTRY
                    if (i == sde_reglog_curr):
                        if (sde_reglog_repeat):
                            break
                        else:
                            sde_reglog_repeat = 1
                    addr = reg_log.logs + self.ramdump.sizeof('struct sde_dbg_reglog_log') * i
                    log_log = addr
                    sde_reglog_start = sde_reglog_start + 1
                    sde_reglog_count = sde_reglog_count + 1
                    log_log = Struct(self.ramdump, log_log,
                                     struct_name="struct sde_dbg_reglog_log",
                                     fields={'time':Struct.get_u64,
                                            'pid':Struct.get_u32,
                                            'addr':Struct.get_u32,
                                            'val':Struct.get_u32,
                                            'blk_id':Struct.get_u8})
                    if (log_log.time == 0x0):
                        continue
                    self.outfile.write('%-20.5d ' % (log_log.time))
                    self.outfile.write('%-10.1d ' % (log_log.pid))
                    self.outfile.write('%-10.1x ' % (log_log.addr))
                    self.outfile.write('%-10.1x ' % (log_log.val))
                    self.outfile.write('%-10.1x ' % (log_log.blk_id))
                    self.outfile.write('\n')
                self.outfile.close()
            except:
                pass

            #Combining reglog with eventlog
            f_evtlog = self.ramdump.open_file('sde_evtlog_parsed.txt', 'r')
            f_reglog = self.ramdump.open_file('sde_reglog.txt', 'r')
            self.outfile = self.ramdump.open_file('sde_evtlog_reglog_merged.txt', 'w')
            evtlog = f_evtlog.readline()
            reglog = f_reglog.readline()
            curr_timestamp = 0
            self.outfile.write('{:<54}{:<29}{:<6}{:<3}{:<200}\n'.format("EVTLOG_REGLOG_MERGED", "TIMESTAMP", "PID", "CPU", "DATA"))
            while (evtlog and reglog):
                if "FUNCTION_NAME" in evtlog:
                    evtlog = f_evtlog.readline()
                if "TIME" in reglog:
                    reglog = f_reglog.readline()
                if "COMMIT" in evtlog:
                    evtlog = f_evtlog.readline()
                if not evtlog or not reglog:
                    break

                evtlog_tmp = re.split(r'==>|: |\n', evtlog, 2)
                data_tmp = evtlog_tmp[2].split("[", 1)
                reglog_tmp = re.sub(r'\s+', ' ', reglog)
                reglog_tmp = reglog.split()

                if (int(evtlog_tmp[1]) > int(reglog_tmp[0])):
                    self.outfile.write('{:<54}{:<29}{:<6}{:<3}{:<200}'.format("sde_reg_write", "==>"+str(reglog_tmp[0])+": "+str(int(reglog_tmp[0]) - curr_timestamp), "["+reglog_tmp[1]+"]","[N]", reglog_tmp[2] + " " + reglog_tmp[3] + " " + reglog_tmp[4]))
                    reglog = f_reglog.readline()
                    curr_timestamp = int(reglog_tmp[0])
                    self.outfile.write('\n')
                else:
                    self.outfile.write('{:<54}{:<29}{:<200}'.format(evtlog_tmp[0], "==>"+str(evtlog_tmp[1])+": "+str(int(evtlog_tmp[1]) - curr_timestamp),"["+data_tmp[1]))
                    evtlog = f_evtlog.readline()
                    curr_timestamp = int(evtlog_tmp[1])

            while(evtlog):
                evtlog_tmp = re.split(r'==>|: |\n', evtlog, 2)
                data_tmp = evtlog_tmp[2].split("[", 1)
                self.outfile.write('{:<54}{:<29}{:<200}'.format(evtlog_tmp[0], "==>"+str(evtlog_tmp[1])+": "+str(int(evtlog_tmp[1]) - curr_timestamp),"["+data_tmp[1]))
                evtlog = f_evtlog.readline()
                curr_timestamp = int(evtlog_tmp[1])

            while(reglog):
                reglog_tmp = re.sub(r'\s+', ' ', reglog)
                reglog_tmp = reglog.split()
                self.outfile.write('{:<54}{:<29}{:<6}{:<3}{:<200}'.format("sde_reg_write", "==>"+str(reglog_tmp[0])+": "+str(int(reglog_tmp[0]) - curr_timestamp), "["+reglog_tmp[1]+"]","[N]", reglog_tmp[2] + " " + reglog_tmp[3] + " " + reglog_tmp[4]))
                self.outfile.write('\n')
                curr_timestamp = int(reglog_tmp[0])
                reglog = f_reglog.readline()

            f_evtlog.close()
            f_reglog.close()
            self.outfile.close()
            self.ramdump.remove_file('sde_reglog.txt')

            dbgbus_sde = Struct(self.ramdump, mdss_dbg.dbgbus_sde,
                            struct_name="struct sde_dbg_sde_debug_bus",
                            fields={'cmn': Struct.get_address})
            dbgbus_sde_cmn = Struct(self.ramdump, dbgbus_sde.cmn,
                            struct_name="struct sde_dbg_debug_bus_common",
                            fields={'dumped_content': Struct.get_pointer,
                                    'content_size': Struct.get_u32})

            dbgbus_vbif = Struct(self.ramdump, mdss_dbg.dbgbus_vbif_rt,
                            struct_name="struct sde_dbg_sde_debug_bus",
                            fields={'cmn': Struct.get_address})
            dbgbus_vbif_cmn = Struct(self.ramdump, dbgbus_vbif.cmn,
                            struct_name="struct sde_dbg_debug_bus_common",
                            fields={'dumped_content': Struct.get_pointer,
                                    'content_size': Struct.get_u32})

            dbgbus_dsi = Struct(self.ramdump, mdss_dbg.dbgbus_dsi,
                            struct_name="struct sde_dbg_sde_debug_bus",
                            fields={'cmn': Struct.get_address})
            dbgbus_dsi_cmn = Struct(self.ramdump, dbgbus_dsi.cmn,
                            struct_name="struct sde_dbg_debug_bus_common",
                            fields={'dumped_content': Struct.get_pointer,
                                    'content_size': Struct.get_u32})

            dbgbus_rsc = Struct(self.ramdump, mdss_dbg.dbgbus_rsc,
                            struct_name="struct sde_dbg_sde_debug_bus",
                            fields={'cmn': Struct.get_address})
            dbgbus_rsc_cmn = Struct(self.ramdump, dbgbus_rsc.cmn,
                            struct_name="struct sde_dbg_debug_bus_common",
                            fields={'dumped_content': Struct.get_pointer,
                                    'content_size': Struct.get_u32})

            dbgbus_lutdma = Struct(self.ramdump, mdss_dbg.dbgbus_lutdma,
                            struct_name="struct sde_dbg_sde_debug_bus",
                            fields={'cmn': Struct.get_address})
            dbgbus_lutdma_cmn = Struct(self.ramdump, dbgbus_lutdma.cmn,
                            struct_name="struct sde_dbg_debug_bus_common",
                            fields={'dumped_content': Struct.get_pointer,
                                    'content_size': Struct.get_u32})

            dbgbus_dp = Struct(self.ramdump, mdss_dbg.dbgbus_dp,
                            struct_name="struct sde_dbg_sde_debug_bus",
                            fields={'cmn': Struct.get_address})
            dbgbus_dp_cmn = Struct(self.ramdump, dbgbus_dp.cmn,
                            struct_name="struct sde_dbg_debug_bus_common",
                            fields={'dumped_content': Struct.get_pointer,
                                    'content_size': Struct.get_u32})

            self.outfile = self.ramdump.open_file('sde_dbgbus.txt', 'w')

            i = 0
            self.outfile.write('---------------------------------\n[dbgbus_sde]\n---------------------------------\n')
            while (i < dbgbus_sde_cmn.content_size):
                j = 0
                self.outfile.write("0x%-.8x| " % (i))
                while (j < 16):
                    self.outfile.write('%-.8x ' % (self.ramdump.read_u32(dbgbus_sde_cmn.dumped_content + (i*4) + j)))
                    j = j + 4
                self.outfile.write('\n')
                i = i + 4

            i = 0
            self.outfile.write('\n---------------------------------\n[dbgbus_vbif_rt]\n---------------------------------\n')
            while (i < dbgbus_vbif_cmn.content_size):
                j = 0
                self.outfile.write("0x%-.8x| " % (i))
                while (j < 16):
                    self.outfile.write('%-.8x ' % (self.ramdump.read_u32(dbgbus_vbif_cmn.dumped_content + (i*4) + j)))
                    j = j + 4
                self.outfile.write('\n')
                i = i + 4

            i = 0
            self.outfile.write('\n---------------------------------\n[dbgbus_dsi]\n---------------------------------\n')
            while (i < dbgbus_dsi_cmn.content_size):
                j = 0
                self.outfile.write("0x%-.8x| " % (i))
                while (j < 16):
                    self.outfile.write('%-.8x ' % (self.ramdump.read_u32(dbgbus_dsi_cmn.dumped_content + (i*4) + j)))
                    j = j + 4
                self.outfile.write('\n')
                i = i + 4

            i = 0
            self.outfile.write('\n---------------------------------\n[dbgbus_rsc]\n---------------------------------\n')
            while (i < dbgbus_rsc_cmn.content_size):
                j = 0
                self.outfile.write("0x%-.8x| " % (i))
                while (j < 16):
                    self.outfile.write('%-.8x ' % (self.ramdump.read_u32(dbgbus_rsc_cmn.dumped_content + (i*4) + j)))
                    j = j + 4
                self.outfile.write('\n')
                i = i + 4

            i = 0
            self.outfile.write('\n---------------------------------\n[dbgbus_lutdma]\n---------------------------------\n')
            while (i < dbgbus_lutdma_cmn.content_size):
                j = 0
                self.outfile.write("0x%-.8x| " % (i))
                while (j < 16):
                    self.outfile.write('%-.8x ' % (self.ramdump.read_u32(dbgbus_lutdma_cmn.dumped_content + (i*4) + j)))
                    j = j + 4
                self.outfile.write('\n')
                i = i + 4

            i = 0
            self.outfile.write('\n---------------------------------\n[dbgbus_dp]\n---------------------------------\n')
            while (i < dbgbus_dp_cmn.content_size):
                j = 0
                self.outfile.write("0x%-.8x| " % (i))
                while (j < 16):
                    self.outfile.write('%-.8x ' % (self.ramdump.read_u32(dbgbus_dp_cmn.dumped_content + (i*4) + j)))
                    j = j + 4
                self.outfile.write('\n')
                i = i + 4
            self.outfile.close()
            self.sde_initial_analysis()
        else:
            for blk in mdss_dbg.blk_arr:
                if blk.is_empty():
                    continue

                # Delays file creation until we have found a non-null array element
                if not self.outfile:
                    self.outfile = self.ramdump.open_file('mdpinfo_out.txt')

                self.outfile.write('mdss_dump_reg_by_ranges:'
                               '=========%s DUMP=========\n' % blk.name)

                head_offset = self.ramdump.field_offset('struct range_dump_node',
                                                        'head')

                dump_list = ListWalker(self.ramdump, blk.dump_list, head_offset)
                if dump_list.is_empty():
                        self.outfile.write('Ranges not found, '
                                           'will dump full registers\n')
                        self.outfile.write('base:0x%x length:%d\n' %
                                           (blk.base, blk.max_offset))

                        self.mdss_dump_reg(blk.base, blk.max_offset, blk.reg_dump, 0)
                else:
                    for node in dump_list:
                        self.print_range(blk, node)

        # Close the file only if it was created
        if self.outfile:
            self.outfile.close()
            self.outfile = None
