"""
Copyright (c) 2017-2018, 2020 The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

from parser_util import register_parser, RamParser
import linux_list as llist

@register_parser('--print-vidc',  'Print vidc driver information',  optional=True)
class DumpVidc(RamParser):

    def __init__(self, ramdump):
        self.ramdump = ramdump
        self.msg_dict = {
            '0x00021001':'HFI_MSG_EVENT_NOTIFY',
            '0x00021000':'HFI_MSG_SESSION_COMMON_START',
            '0x00121001':'HFI_MSG_SESSION_DECODE_FRAME_DONE',
            '0x00121000':'HFI_MSG_SESSION_DX_START',
            '0x00221007':'HFI_MSG_SESSION_EMPTY_BUFFER_DONE',
            '0x00121004':'HFI_MSG_SESSION_ENCODE_EOS_COMPLETE_DONE',
            '0x00121003':'HFI_MSG_SESSION_ENCODE_FLUSH_DONE',
            '0x00121002':'HFI_MSG_SESSION_ENCODE_FRAME_DONE',
            '0x00221008':'HFI_MSG_SESSION_FILL_BUFFER_DONE',
            '0x00221006':'HFI_MSG_SESSION_FLUSH_DONE',
            '0x00021002':'HFI_MSG_SESSION_GET_SEQUENCE_HEADER_DONE',
            '0x00221001':'HFI_MSG_SESSION_LOAD_RESOURCES_DONE',
            '0x00221000':'HFI_MSG_SESSION_OX_START',
            '0x0022100B':'HFI_MSG_SESSION_PARSE_SEQUENCE_HEADER_DONE',
            '0x00221009':'HFI_MSG_SESSION_PROPERTY_INFO',
            '0x0022100C':'HFI_MSG_SESSION_RELEASE_BUFFERS_DONE',
            '0x0022100A':'HFI_MSG_SESSION_RELEASE_RESOURCES_DONE',
            '0x00221005':'HFI_MSG_SESSION_RESUME_DONE',
            '0x00221002':'HFI_MSG_SESSION_START_DONE',
            '0x00221003':'HFI_MSG_SESSION_STOP_DONE',
            '0x00221004':'HFI_MSG_SESSION_SUSPEND_DONE',
            '0x0022100D':'HFI_MSG_SESSION_SYNC_DONE',
            '0x00020000':'HFI_MSG_START_OFFSET',
            '0x00020000':'HFI_MSG_SYS_COMMON_START',
            '0x00020009':'HFI_MSG_SYS_COV',
            '0x00020004':'HFI_MSG_SYS_DEBUG',
            '0x00000100':'HFI_MSG_SYS_DEBUG_PACKET_MAX_STRNG_SIZE',
            '0x00120000':'HFI_MSG_SYS_DX_START',
            '0x00020008':'HFI_MSG_SYS_IDLE',
            '0x00020001':'HFI_MSG_SYS_INIT_DONE',
            '0x00220000':'HFI_MSG_SYS_OX_START',
            '0x00020002':'HFI_MSG_SYS_PC_PREP_DONE',
            '0x00220002':'HFI_MSG_SYS_PING_ACK',
            '0x0002000A':'HFI_MSG_SYS_PROPERTY_INFO',
            '0x00020003':'HFI_MSG_SYS_RELEASE_RESOURCE',
            '0x00220004':'HFI_MSG_SYS_SESSION_ABORT_DONE',
            '0x00020007':'HFI_MSG_SYS_SESSION_END_DONE',
            '0x00020006':'HFI_MSG_SYS_SESSION_INIT_DONE'
        }
        self.cmd_dict = {
            '0x00011000':'HFI_CMD_SESSION_COMMON_START',
            '0x0021100D':'HFI_CMD_SESSION_CONTINUE',
            '0x00111001':'HFI_CMD_SESSION_DECODE_FRAME',
            '0x00111000':'HFI_CMD_SESSION_DX_START',
            '0x00211004':'HFI_CMD_SESSION_EMPTY_BUFFER',
            '0x00111004':'HFI_CMD_SESSION_ENCODE_EOS_COMPLETE',
            '0x00111003':'HFI_CMD_SESSION_ENCODE_FLUSH',
            '0x00111002':'HFI_CMD_SESSION_ENCODE_FRAME',
            '0x00111006':'HFI_CMD_SESSION_ENCODE_INIT_RESOURCES',
            '0x00111005':'HFI_CMD_SESSION_ENCODE_RECONFIG',
            '0x00211005':'HFI_CMD_SESSION_FILL_BUFFER',
            '0x00211008':'HFI_CMD_SESSION_FLUSH',
            '0x00211009':'HFI_CMD_SESSION_GET_PROPERTY',
            '0x00011003':'HFI_CMD_SESSION_GET_SEQUENCE_HEADER',
            '0x00211001':'HFI_CMD_SESSION_LOAD_RESOURCES',
            '0x00211000':'HFI_CMD_SESSION_OX_START',
            '0x0021100A':'HFI_CMD_SESSION_PARSE_SEQUENCE_HEADER',
            '0x0021100B':'HFI_CMD_SESSION_RELEASE_BUFFERS',
            '0x0021100C':'HFI_CMD_SESSION_RELEASE_RESOURCES',
            '0x00211007':'HFI_CMD_SESSION_RESUME',
            '0x00011002':'HFI_CMD_SESSION_SET_BUFFERS',
            '0x00011001':'HFI_CMD_SESSION_SET_PROPERTY',
            '0x00211002':'HFI_CMD_SESSION_START',
            '0x00211003':'HFI_CMD_SESSION_STOP',
            '0x00211006':'HFI_CMD_SESSION_SUSPEND',
            '0x0021100E':'HFI_CMD_SESSION_SYNC',
            '0x00010000':'HFI_CMD_START_OFFSET',
            '0x00010000':'HFI_CMD_SYS_COMMON_START',
            '0x00110000':'HFI_CMD_SYS_DX_START',
            '0x00010006':'HFI_CMD_SYS_GET_PROPERTY',
            '0x00010001':'HFI_CMD_SYS_INIT',
            '0x00210000':'HFI_CMD_SYS_OX_START',
            '0x00010002':'HFI_CMD_SYS_PC_PREP',
            '0x00210002':'HFI_CMD_SYS_PING',
            '0x00010004':'HFI_CMD_SYS_RELEASE_RESOURCE',
            '0x00210001':'HFI_CMD_SYS_SESSION_ABORT',
            '0x00010008':'HFI_CMD_SYS_SESSION_END',
            '0x00010007':'HFI_CMD_SYS_SESSION_INIT',
            '0x00110002':'HFI_CMD_SYS_SET_ACTIVE_FENCE',
            '0x00110004':'HFI_CMD_SYS_SET_ACTIVE_FENCE_V2',
            '0x00010009':'HFI_CMD_SYS_SET_BUFFERS',
            '0x00010005':'HFI_CMD_SYS_SET_PROPERTY',
            '0x00010003':'HFI_CMD_SYS_SET_RESOURCE',
            '0x00110001':'HFI_CMD_SYS_SYNC_PROCESS',
            '0x00110003':'HFI_CMD_SYS_SYNC_PROCESS_V2',
            '0x00010100':'HFI_CMD_SYS_TEST_START',
            '0x00000001':'HFI_CMD_TYPE_OFFSET_IN_UWORD32'
        }
        self.hal_buf_type_dict = {
            '0x0':'HAL_BUFFER_NONE',
            '0x1':'HAL_BUFFER_INPUT',
            '0x2':'HAL_BUFFER_OUTPUT',
            '0x4':'HAL_BUFFER_OUTPUT2',
            '0x8':'HAL_BUFFER_EXTRADATA_INPUT',
            '0x10':'HAL_BUFFER_EXTRADATA_OUTPUT',
            '0x20':'HAL_BUFFER_EXTRADATA_OUTPUT2',
            '0x40':'HAL_BUFFER_INTERNAL_SCRATCH',
            '0x80':'HAL_BUFFER_INTERNAL_SCRATCH_1',
            '0x100':'HAL_BUFFER_INTERNAL_SCRATCH_2',
            '0x200':'HAL_BUFFER_INTERNAL_PERSIST',
            '0x400':'HAL_BUFFER_INTERNAL_PERSIST_1',
            '0x800':'HAL_BUFFER_INTERNAL_CMD_QUEUE',
            '0x1000':'HAL_BUFFER_INTERNAL_RECON'
        }
        self.inst_state_dict = {
            '0x0001':'MSM_VIDC_CORE_UNINIT_DONE',
            '0x0002':'MSM_VIDC_CORE_INIT',
            '0x0003':'MSM_VIDC_CORE_INIT_DONE',
            '0x0004':'MSM_VIDC_OPEN',
            '0x0005':'MSM_VIDC_OPEN_DONE',
            '0x0006':'MSM_VIDC_LOAD_RESOURCES',
            '0x0007':'MSM_VIDC_LOAD_RESOURCES_DONE',
            '0x0008':'MSM_VIDC_START',
            '0x0009':'MSM_VIDC_START_DONE',
            '0x000a':'MSM_VIDC_STOP',
            '0x000b':'MSM_VIDC_STOP_DONE',
            '0x000c':'MSM_VIDC_RELEASE_RESOURCES',
            '0x000d':'MSM_VIDC_RELEASE_RESOURCES_DONE',
            '0x000e':'MSM_VIDC_CLOSE',
            '0x000f':'MSM_VIDC_CLOSE_DONE',
            '0x0010':'MSM_VIDC_CORE_UNINIT',
            '0x0011':'MSM_VIDC_CORE_INVALID'
        }
    def parse(self):
        self.vidc_info = self.ramdump.open_file('vidc_info.txt')
        if (self.ramdump.kernel_version < (4, 9, 0)):
            self.vidc_info.write('Kernel version 4.9 \
            and above are supported, current version {0}.\
            {1}'.format(self.ramdump.kernel_version[0],
                        self.ramdump.kernel_version[1]))
            self.vidc_info.close()
            return

        vidc_driver_addr = self.ramdump.read_word(self.ramdump.address_of('vidc_driver'))

        head_core = vidc_driver_addr + self.ramdump.field_offset('struct msm_vidc_drv', 'cores')
        offset_core = self.ramdump.field_offset('struct msm_vidc_core', 'list')

        core_walker = llist.ListWalker(self.ramdump, head_core, offset_core)
        if not core_walker.is_empty():
            head_core = core_walker.next()
            core_walker.walk(head_core,self.core_walker)
        self.vidc_info.close()

    def core_walker(self,head):
        self.vidc_info.write('\n========CORE STATS========\n')

        self.vidc_info.write('Core Addr: \t0x{:1x}\n'.format(head).expandtabs(20))

        core_id = self.ramdump.read_structure_field(
            head, 'struct msm_vidc_core', 'id')
        self.vidc_info.write('core: \t{0}\n'.format(core_id).expandtabs(20))

        state = self.ramdump.read_structure_field(
            head, 'struct msm_vidc_core', 'state')
        self.vidc_info.write('state: \t{0}\n'.format(state).expandtabs(20))

        hfi_type = self.ramdump.read_structure_field(
            head, 'struct msm_vidc_core', 'hfi_type')
        self.vidc_info.write('hfi_type: \t{0}\n'.format(hfi_type).expandtabs(20))

        codec_count = self.ramdump.read_structure_field(
            head, 'struct msm_vidc_core', 'codec_count')
        self.vidc_info.write('codec_count: \t{0}\n'.format(codec_count).expandtabs(20))

        smmu_fault_handled_addr = head + self.ramdump.field_offset(
            'struct msm_vidc_core', 'trigger_ssr')
        smmu_fault_handled = self.ramdump.read_bool(smmu_fault_handled_addr)
        self.vidc_info.write('smmu_fault_handled: {0}\n'.format(smmu_fault_handled))

        trigger_ssr_addr = head + self.ramdump.field_offset(
            'struct msm_vidc_core', 'trigger_ssr')
        trigger_ssr = self.ramdump.read_bool(trigger_ssr_addr)
        self.vidc_info.write('trigger_ssr: \t{0}\n'.format(trigger_ssr).expandtabs(20))

        min_freq = self.ramdump.read_structure_field(
            head, 'struct msm_vidc_core', 'min_freq')
        self.vidc_info.write('min freq: \t{0}\n'.format(min_freq).expandtabs(20))

        curr_freq = self.ramdump.read_structure_field(
            head, 'struct msm_vidc_core', 'curr_freq')
        self.vidc_info.write('curr_freq: \t{0}\n'.format(curr_freq).expandtabs(20))

        self.vidc_info.write('\nCompletions:\n------------\n')
        for i in  range(0,27):
            completion = head + self.ramdump.field_offset('struct msm_vidc_core',\
                                                          'completions['+str(i)+']')
            done = self.ramdump.read_structure_field(
                head, 'struct completion', 'done')

            self.vidc_info.write('[{0:02d}]: {1}  '.format(i,'done' if done else 'pending'))
            if (i+1)%9 == 0 or i == 26:
                self.vidc_info.write('\n')

        self.vidc_info.write('\n=======DEVICE STATS=======\n')

        head_hfi_device_data = self.ramdump.read_structure_field(
            head, 'struct msm_vidc_core', 'device')
        head_hfi_device_data = self.ramdump.read_word(head_hfi_device_data)

        self.vidc_info.write('Device Addr: \t0x{:1x}\n'.format(head_hfi_device_data).expandtabs(20))

        self.get_hfi_device(head_hfi_device_data)

        head_inst = head + self.ramdump.field_offset('struct msm_vidc_core', 'instances')
        offset_inst = self.ramdump.field_offset('struct msm_vidc_inst','list')

        inst_walker = llist.ListWalker(self.ramdump, head_inst, offset_inst)
        if not inst_walker.is_empty():
            head_inst = inst_walker.next()
            inst_walker.walk(head_inst,self.inst_walker)

    def inst_walker(self,head):
        self.vidc_info.write('\n========INST STATS========\n')

        self.vidc_info.write('Instance Addr: \t0x{:1x}\n'.format(head).expandtabs(20))

        in_reconfig_addr = head + self.ramdump.field_offset(
            'struct msm_vidc_inst', 'in_reconfig')
        in_reconfig = self.ramdump.read_bool(in_reconfig_addr)
        self.vidc_info.write('Reconfig: \t{0}\n'.format(in_reconfig).expandtabs(20))

        reconfig_width = self.ramdump.read_structure_field(
            head, 'struct msm_vidc_inst', 'reconfig_width')
        self.vidc_info.write('Reconfig width: \t{0}\n'.format(reconfig_width).expandtabs(20))

        reconfig_height = self.ramdump.read_structure_field(
            head, 'struct msm_vidc_inst', 'reconfig_height')
        self.vidc_info.write('Reconfig height: \t{0}\n'.format(reconfig_height).expandtabs(20))

        session_type = self.ramdump.read_structure_field(
            head, 'struct msm_vidc_inst', 'session_type')
        session_type = hex(session_type)
        session_type_name = 'unknown'
        if session_type=="0x0":
            session_type_name="MSM_VIDC_ENCODER"
        elif session_type=="0x1":
            session_type_name="MSM_VIDC_DECODER"
        elif session_type=="0x2":
            session_type_name="MSM_VIDC_CVP"
        '''self.vidc_info.write('Session Type: \t{0}\n'\
                             .format('DECODER' if session_type else 'ENCODER').expandtabs(20))'''

        self.vidc_info.write('Session Type: \t{0}\n'\
                             .format(session_type_name).expandtabs(20))
        cap_out = head + self.ramdump.field_offset('struct msm_vidc_inst', 'capability')
        cap_codec_offset = self.ramdump.field_offset('struct msm_vidc_capability', 'codec')
        codec_name = self.ramdump.read_u64(cap_out + cap_codec_offset)
        codec_name = hex(codec_name)

        #print "codec_name = {0}".format(codec_name)
        codec_data = 'unknwon'
        if codec_name=="0x0":
            codec_data="HAL_VIDEO_CODEC_UNKNOWN"
        elif codec_name=="0x1":
            codec_data="HAL_VIDEO_CODEC_MVC"
        elif codec_name=="0x2":
            codec_data="HAL_VIDEO_CODEC_H264"
        elif codec_name=="0x4":
            codec_data="HAL_VIDEO_CODEC_H263"
        elif codec_name=="0x8":
            codec_data="HAL_VIDEO_CODEC_MPEG1"
        elif codec_name=="0x10":
            codec_data="HAL_VIDEO_CODEC_MPEG2"
        elif codec_name=="0x20":
            codec_data="HAL_VIDEO_CODEC_MPEG4"
        elif codec_name=="0x40":
            codec_data="HAL_VIDEO_CODEC_DIVX_311"
        elif codec_name=="0x80":
            codec_data="HAL_VIDEO_CODEC_DIVX"
        elif codec_name=="0x100":
            codec_data="HAL_VIDEO_CODEC_VC1"
        elif codec_name=="0x200":
            codec_data="HAL_VIDEO_CODEC_SPARK"
        elif codec_name=="0x400":
            codec_data="HAL_VIDEO_CODEC_VP6"
        elif codec_name=="0x800":
            codec_data="HAL_VIDEO_CODEC_VP7"
        elif codec_name=="0x1000":
            codec_data="HAL_VIDEO_CODEC_VP8"
        elif codec_name=="0x2000":
            codec_data="HAL_VIDEO_CODEC_HEVC"
        elif codec_name=="0x4000":
            codec_data="HAL_VIDEO_CODEC_VP9"
        elif codec_name=="0x8000":
            codec_data="HAL_VIDEO_CODEC_TME"
        elif codec_name=="0x10000":
            codec_data="HAL_VIDEO_CODEC_CVP"
        elif codec_name=="0x80000000":
            codec_data="HAL_VIDEO_CODEC_HEVC_HYBRID"
        elif codec_name=="0x10000000":
            codec_data="HAL_UNUSED_CODEC"

        self.vidc_info.write('Codec: \t{0}\n'\
                             .format(codec_data).expandtabs(20))

        session_addr = self.ramdump.read_structure_field(
            head, 'struct msm_vidc_inst', 'session')
        session_id = (session_addr>>32)^(session_addr&0x00000000FFFFFFFF)
        self.vidc_info.write('Session_id: \t0x{:1x}\n'.format(session_id).expandtabs(20))

        state = self.ramdump.read_structure_field(
            head, 'struct msm_vidc_inst', 'state')
        state_str = self.inst_state_dict[format(state,'#06x')]
        self.vidc_info.write('State: \t{0}\n'.format(state_str).expandtabs(20))

        in_flush_addr = head + self.ramdump.field_offset(
            'struct msm_vidc_inst', 'in_flush')
        in_flush = self.ramdump.read_bool(in_flush_addr)
        self.vidc_info.write('In flush: \t{0}\n'.format(in_flush).expandtabs(20))

        pic_struct = self.ramdump.read_structure_field(
            head, 'struct msm_vidc_inst', 'pic_struct')
        self.vidc_info.write('Pic Struct: \t{0}\n'.format(pic_struct).expandtabs(20))

        colour_space = self.ramdump.read_structure_field(
            head, 'struct msm_vidc_inst', 'colour_space')
        self.vidc_info.write('Colour Space: \t{0}\n'.format(colour_space).expandtabs(20))

        profile = self.ramdump.read_structure_field(
            head, 'struct msm_vidc_inst', 'profile')
        self.vidc_info.write('Profile: \t{0}\n'.format(profile).expandtabs(20))

        level = self.ramdump.read_structure_field(
            head, 'struct msm_vidc_inst', 'level')
        self.vidc_info.write('Level: \t{:1x}\n'.format(level).expandtabs(20))

        entropy_mode = self.ramdump.read_structure_field(
            head, 'struct msm_vidc_inst', 'entropy_mode')
        self.vidc_info.write('Entropy Mode: \t{0}\n'.format(entropy_mode).expandtabs(20))

        '''smem_client = self.ramdump.read_structure_field(
            head,'struct msm_vidc_inst', 'mem_client')

        client = self.ramdump.read_structure_field(
           smem_client , 'struct smem_client', 'clnt')

        display_name = self.ramdump.read_structure_cstring(
           client , 'struct ion_client', 'display_name')

        self.vidc_info.write('Client: \t{0}\n'.format(display_name).expandtabs(20))'''

        output_fmt = head + self.ramdump.field_offset('struct msm_vidc_inst', 'fmts[0]')
        description_addr = output_fmt
        + self.ramdump.field_offset('struct msm_vidc_format', 'description')
        desc_out = self.ramdump.read_cstring(description_addr)

        v4l2_fmt_addr = output_fmt + self.ramdump.field_offset('struct msm_vidc_format', 'v4l2_fmt')
        #v4l2_fmt_out = self.ramdump.read_u64(v4l2_fmt_addr)
        v4l2_fmt_out = v4l2_fmt_addr

#        print "v4l2_fmt_out = {0}".format(hex(v4l2_fmt_out))
        fmt_offset = self.ramdump.field_offset('struct v4l2_format', 'fmt')
        fmt_addr = v4l2_fmt_out + fmt_offset
        #print "fmt_addr = {0}".format(hex(v4l2_fmt_out))

        pix_fmt_offset = self.ramdump.field_offset('struct v4l2_pix_format_mplane' ,'pixelformat')
        pix_fmt_addr = self.ramdump.read_int(fmt_addr + pix_fmt_offset)
        pix_fmt_addr = hex(pix_fmt_addr)

        #print "pix_fmt_addr = {0}".format(pix_fmt_addr)
        output_v4l2_fmt_name = "Unknown"
        if pix_fmt_addr == "0x3231564e":
            output_v4l2_fmt_name="V4L2_PIX_FMT_NV12"
            output_color_fmt_name="YCbCr Semiplanar 4:2:0"
        elif pix_fmt_addr == "0x3132564e":
            output_v4l2_fmt_name="V4L2_PIX_FMT_NV21"
            output_color_fmt_name="YCrCb Semiplanar 4:2:0"
        elif pix_fmt_addr == "0x32313551":
            output_v4l2_fmt_name="V4L2_PIX_FMT_NV12_512"
            output_color_fmt_name="YCbCr Semiplanar 4:2:0 512 aligned"
        elif pix_fmt_addr == "0x30315051":
            output_v4l2_fmt_name="V4L2_PIX_FMT_SDE_Y_CBCR_H2V2_P010_VENUS"
            output_color_fmt_name="YCbCr Semiplanar 4:2:0 10bit"
        elif pix_fmt_addr == "0x38323151":
            output_v4l2_fmt_name="V4L2_PIX_FMT_NV12_UBWC"
            output_color_fmt_name="UBWC YCbCr Semiplanar 4:2:0"
        elif pix_fmt_addr == "0x41323151":
            output_v4l2_fmt_name="V4L2_PIX_FMT_NV12_TP10_UBWC"
            output_color_fmt_name="TP10 UBWC 4:2:0"
        elif pix_fmt_addr == "0x3247504d":
            output_v4l2_fmt_name="V4L2_PIX_FMT_MPEG2"
            output_color_fmt_name="Mpeg2"
        elif pix_fmt_addr == "0x34363248":
            output_v4l2_fmt_name="V4L2_PIX_FMT_H264"
            output_color_fmt_name="H264"
        elif pix_fmt_addr=="0x43564548":
            output_v4l2_fmt_name="V4L2_PIX_FMT_HEVC"
            output_color_fmt_name="HEVC"
        elif pix_fmt_addr=="0x30385056":
            output_v4l2_fmt_name="V4L2_PIX_FMT_VP8"
            output_color_fmt_name="VP8"
        elif pix_fmt_addr=="0x30395056":
            output_v4l2_fmt_name="V4L2_PIX_FMT_VP9"
            output_color_fmt_name="VP9"
        elif pix_fmt_addr=="0x30454d54":
            output_v4l2_fmt_name="V4L2_PIX_FMT_TME"
            output_color_fmt_name="TME"

        caputre_fmt = head + self.ramdump.field_offset('struct msm_vidc_inst', 'fmts[1]')
        description_addr = caputre_fmt
        + self.ramdump.field_offset('struct msm_vidc_format', 'description')
        desc_cap = self.ramdump.read_cstring(description_addr)

        v4l2_fmt_addr_cap = caputre_fmt + self.ramdump.field_offset('struct msm_vidc_format', 'v4l2_fmt')
        #v4l2_fmt_out = self.ramdump.read_u64(v4l2_fmt_addr)
        v4l2_fmt_cap = v4l2_fmt_addr_cap

        #print "v4l2_fmt_cap = {0}".format(hex(v4l2_fmt_cap))
        fmt_addr_cap = v4l2_fmt_cap + fmt_offset
        #print "fmt_addr_cap = {0}".format(hex(fmt_addr_cap))
        pix_fmt_addr_cap = self.ramdump.read_int(fmt_addr_cap + pix_fmt_offset)
        pix_fmt_addr_cap = hex(pix_fmt_addr_cap)
        #print "pix_fmt_addr_cap = {0}".format(pix_fmt_addr_cap)

        capture_v4l2_fmt_name = "unkown"
        if pix_fmt_addr_cap=="0x3231564e":
            capture_v4l2_fmt_name="V4L2_PIX_FMT_NV12"
            capture_color_fmt_name="YCbCr Semiplanar 4:2:0"
        elif pix_fmt_addr_cap=="0x3132564e":
            capture_v4l2_fmt_name="V4L2_PIX_FMT_NV21"
            capture_color_fmt_name="YCrCb Semiplanar 4:2:0"
        elif pix_fmt_addr_cap=="0x32313551":
            capture_v4l2_fmt_name="V4L2_PIX_FMT_NV12_512"
            capture_color_fmt_name="YCbCr Semiplanar 4:2:0 512 aligned"
        elif pix_fmt_addr_cap=="0x30315051":
            capture_v4l2_fmt_name="V4L2_PIX_FMT_SDE_Y_CBCR_H2V2_P010_VENUS"
            capture_color_fmt_name="YCbCr Semiplanar 4:2:0 10bit"
        elif pix_fmt_addr_cap=="0x38323151":
            capture_v4l2_fmt_name="V4L2_PIX_FMT_NV12_UBWC"
            capture_color_fmt_name="UBWC YCbCr Semiplanar 4:2:0"
        elif pix_fmt_addr_cap=="0x41323151":
            capture_v4l2_fmt_name="V4L2_PIX_FMT_NV12_TP10_UBWC"
            capture_color_fmt_name="TP10 UBWC 4:2:0"
        elif pix_fmt_addr_cap=="0x3247504d":
            capture_v4l2_fmt_name="V4L2_PIX_FMT_MPEG2"
            capture_color_fmt_name="Mpeg2"
        elif pix_fmt_addr_cap == "0x34363248":
            capture_v4l2_fmt_name="V4L2_PIX_FMT_H264"
            capture_color_fmt_name="H264"
        elif pix_fmt_addr_cap=="0x43564548":
            capture_v4l2_fmt_name="V4L2_PIX_FMT_HEVC"
            capture_color_fmt_name="HEVC"
        elif pix_fmt_addr_cap=="0x30385056":
            capture_v4l2_fmt_name="V4L2_PIX_FMT_VP8"
            capture_color_fmt_name="VP8"
        elif pix_fmt_addr_cap=="0x30395056":
            capture_v4l2_fmt_name="V4L2_PIX_FMT_VP9"
            capture_color_fmt_name="VP9"
        elif pix_fmt_addr_cap=="0x30454d54":
            capture_v4l2_fmt_name="V4L2_PIX_FMT_TME"
            capture_color_fmt_name="TME"

        session_prop = head + self.ramdump.field_offset('struct msm_vidc_inst', 'prop')
        fps = self.ramdump.read_structure_field(
            session_prop, 'struct session_prop', 'fps')
        self.vidc_info.write('FPS: \t{0}\n'.format(fps).expandtabs(20))

        bitrate = self.ramdump.read_structure_field(
            session_prop, 'struct session_prop', 'bitrate')
        self.vidc_info.write('Bitrate: \t{0}\n'.format(bitrate).expandtabs(20))

        width_out = self.ramdump.read_structure_field(
            session_prop, 'struct session_prop', 'width[0]')
        height_out = self.ramdump.read_structure_field(
            session_prop, 'struct session_prop', 'height[0]')

        bufq_out = head + self.ramdump.field_offset('struct msm_vidc_inst', 'bufq[0]')
        vb2_bufq_offset = self.ramdump.field_offset('struct buf_queue', 'vb2_bufq')
        vb2_bufq_out = bufq_out + vb2_bufq_offset

        num_buffers_offset = self.ramdump.field_offset('struct vb2_queue','num_buffers')
        num_buffers = self.ramdump.read_u64(vb2_bufq_out + num_buffers_offset)
        #print "num_buffers = {0}".format(num_buffers)

        bufq_cap = head + self.ramdump.field_offset('struct msm_vidc_inst', 'bufq[1]')
        vb2_bufq_cap = bufq_cap + vb2_bufq_offset

        num_buffers_cap = self.ramdump.read_u64(vb2_bufq_cap + num_buffers_offset)
        #print "num_buffers_cap = {0}".format(num_buffers_cap)

        '''num_planes_out = self.ramdump.read_structure_field(
            bufq_out, 'struct buf_queue', 'num_planes')
        plane_sizes_out = []
        for i in  range(0,num_planes_out):
            plane_sizes_out.append(self.ramdump.read_structure_field(
                bufq_out, 'struct buf_queue', 'plane_sizes['+str(i)+']'))'''

        '''width_cap = self.ramdump.read_structure_field(
            session_prop, 'struct session_prop', 'width[1]')
        height_cap = self.ramdump.read_structure_field(
            session_prop, 'struct session_prop', 'height[1]')'''
        width_cap_offset = self.ramdump.field_offset('struct v4l2_pix_format_mplane' ,'width')
        width_cap = self.ramdump.read_int(fmt_addr_cap + width_cap_offset)

        height_cap_offset = self.ramdump.field_offset('struct v4l2_pix_format_mplane' ,'height')
        height_cap = self.ramdump.read_int(fmt_addr_cap + height_cap_offset)

        #print "height_cap {0}".format(height_cap)

        self.vidc_info.write('Resolution: \t{0} x {1}\n'\
                             .format(width_cap,height_cap).expandtabs(20))

        #bufq_cap = head + self.ramdump.field_offset('struct msm_vidc_inst', 'bufq[1]')
        '''num_planes_cap = self.ramdump.read_structure_field(
            bufq_cap, 'struct buf_queue', 'num_planes')
        plane_sizes_cap = []
        for i in  range(0,num_planes_cap):
            plane_sizes_cap.append(self.ramdump.read_structure_field(
                bufq_cap, 'struct buf_queue', 'plane_sizes['+str(i)+']'))'''

        num_planes_offset = self.ramdump.field_offset('struct v4l2_pix_format_mplane' ,'num_planes')
        num_planes_out = self.ramdump.read_u16(fmt_addr + num_planes_offset)
        num_planes_cap = self.ramdump.read_u16(fmt_addr_cap + num_planes_offset)

        #print "num_planes_out = {0}".format(num_planes_out)
        plane_sizes_out = []
        for i in  range(0,num_planes_out):
            #plane_sizes_out.append(self.ramdump.read_structure_field(
             #   fmt_addr, 'struct v4l2_pix_format_mplane', 'plane_fmt['+str(i)+']'))
            plane_fmt_out = fmt_addr + self.ramdump.field_offset('struct v4l2_pix_format_mplane', 'plane_fmt['+str(i)+']')
            plan_image_offset = self.ramdump.field_offset('struct v4l2_plane_pix_format', 'sizeimage')
            imagesize = self.ramdump.read_int(plane_fmt_out + plan_image_offset)
            #print "imagesize = {0}".format(imagesize)
            plane_sizes_out.append(imagesize)

        plane_sizes_cap = []
        for i in  range(0,num_planes_cap):
            #plane_sizes_out.append(self.ramdump.read_structure_field(
             #   fmt_addr, 'struct v4l2_pix_format_mplane', 'plane_fmt['+str(i)+']'))
            plane_fmt_cap = fmt_addr_cap + self.ramdump.field_offset('struct v4l2_pix_format_mplane', 'plane_fmt['+str(i)+']')
            plan_image_offset = self.ramdump.field_offset('struct v4l2_plane_pix_format', 'sizeimage')
            imagesize_cap = self.ramdump.read_int(plane_fmt_cap + plan_image_offset)
            #print "imagesize_cap = {0}".format(imagesize_cap)
            plane_sizes_cap.append(imagesize_cap)

        self.vidc_info.write('\noutput capability: \n')
        self.vidc_info.write('color format: \t{0}\n'\
                             .format(desc_out).expandtabs(20))
        self.vidc_info.write('v4l2 format: \t{0}\n'\
                             .format(output_v4l2_fmt_name).expandtabs(20))
        self.vidc_info.write('num_of_planes: \t{0}\n'\
                             .format(num_planes_out).expandtabs(20))
        self.vidc_info.write('buffers_cnt: \t{0}\n'\
                             .format(num_buffers).expandtabs(20))
        for idx,item in enumerate(plane_sizes_out):
            self.vidc_info.write('size for plane: {0}\t {1}\n'\
                             .format(idx,item).expandtabs(20))

        self.vidc_info.write('\ncapture capability: \n')
        self.vidc_info.write('color format: \t{0}\n'\
                             .format(desc_cap).expandtabs(20))
        self.vidc_info.write('v4l2 format: \t{0}\n'\
                             .format(capture_v4l2_fmt_name).expandtabs(20))
        self.vidc_info.write('num_of_planes: \t{0}\n'\
                             .format(num_planes_cap).expandtabs(20))
        self.vidc_info.write('buffers_cnt: \t{0}\n'\
                             .format(num_buffers_cap).expandtabs(20))
        for idx,item in enumerate(plane_sizes_cap):
            self.vidc_info.write('size for plane: {0}\t {1}\n'\
                             .format(idx,item).expandtabs(20))

        self.vidc_info.write('\nPORT\tWIDTH\tHEIGHT\t'.expandtabs(20)+'COLOR FMT\tV4l2 FMT\t'\
                             .expandtabs(30) +'NUM_PLANES\tBUFFER_CNT\tPLANE_SIZES\n'\
                             .expandtabs(20) + '-'*172 + '\n')
        self.vidc_info.write('OUTPUT\t{}\t{}\t'.format(width_out,height_out)\
                             .expandtabs(20)+'{}\t{}\t'.format(desc_out,output_v4l2_fmt_name).expandtabs(30)\
                             +'{}\t{}\t{}\n'.format(num_planes_out,num_buffers,plane_sizes_out)\
                             .expandtabs(20))
        self.vidc_info.write('CAPUTRE\t{}\t{}\t'.format(width_cap,height_cap)\
                             .expandtabs(20)+ '{}\t{}\t'.format(desc_cap,capture_v4l2_fmt_name).expandtabs(30)\
                             +'{}\t{}\t{}\n'.format(num_planes_cap,num_buffers_cap,plane_sizes_cap)
                             .expandtabs(20))
        '''session_crop = session_prop + self.ramdump.field_offset('struct session_prop','crop_info' )

        left = self.ramdump.read_structure_field(
            session_crop, 'struct session_crop', 'left')
        top = self.ramdump.read_structure_field(
            session_crop, 'struct session_crop', 'top')
        width = self.ramdump.read_structure_field(
            session_crop, 'struct session_crop', 'width')
        height = self.ramdump.read_structure_field(
            session_crop, 'struct session_crop', 'height')

        self.vidc_info.write('\nCROP_INFO\n=========\nLeft: \t{0}\
                             \nTop: \t{1}\nWidth: \t{2}\nHeight: \t{3}\
                             \n'.format(left,top,width,height).expandtabs(20))'''

        buf_count = head + self.ramdump.field_offset('struct msm_vidc_inst', 'count')

        etb = self.ramdump.read_structure_field(
            buf_count, 'struct buf_count', 'etb')
        ftb = self.ramdump.read_structure_field(
            buf_count, 'struct buf_count', 'ftb')
        ebd = self.ramdump.read_structure_field(
            buf_count, 'struct buf_count', 'ebd')
        fbd = self.ramdump.read_structure_field(
            buf_count, 'struct buf_count', 'fbd')

        self.vidc_info.write('\nBUFFER_COUNTS\n=============\nETB: \t{0}\
                             \nFTB: \t{1}\nEBD: \t{2}\nFBD: \t{3}\n'\
                             .format(etb,ftb,ebd,fbd).expandtabs(20))

        self.vidc_info.write('\nBUFFER REQUIREMENTS\n===================\n')

        self.vidc_info.write('TYPE\t'.expandtabs(40))
        self.vidc_info.write('SIZE\tREGION_SIZE\tCOUNT_MIN\t'.expandtabs(20))
        self.vidc_info.write('COUNT_MIN_HOST\tCOUNT_ACTUAL\tCONTIGUOUS\tALIGNMENT\n'.expandtabs(20))
        self.vidc_info.write('-'*180+'\n')

        buff_req = head + self.ramdump.field_offset('struct msm_vidc_inst', 'buff_req')

        for i in range(0,14):#HAL_BUFFER_MAX = 14
            buff_req_i = buff_req
            + self.ramdump.field_offset('struct buffer_requirements', 'buffer['+str(i)+']')
            self.buff_requirements(buff_req_i)
            i = i+1

        self.vidc_info.write('\nBUFFER INFO\n===========\n')

        offset_vidc_list = self.ramdump.field_offset('struct msm_vidc_list','list')

        scratchbufs = head + self.ramdump.field_offset('struct msm_vidc_inst', 'scratchbufs')
        internal_buf_walker = llist.ListWalker(self.ramdump, scratchbufs, offset_vidc_list)
        self.vidc_info.write('\nScratch Buffers:\n')
        self.vidc_info.write('='*40+'\n')
        if not(internal_buf_walker.is_empty()):
            scratchbufs = internal_buf_walker.next()
            self.vidc_info.write('Type\t'.expandtabs(40))
            self.vidc_info.write('Device_Addr\tSize\tFlags\n'.expandtabs(10)+'-'*80+'\n')
            internal_buf_walker.walk(scratchbufs,self.internal_buf_walker)
        else:
            self.vidc_info.write('NONE\n')

        persistbufs = head + self.ramdump.field_offset('struct msm_vidc_inst', 'persistbufs')
        internal_buf_walker = llist.ListWalker(self.ramdump, persistbufs, offset_vidc_list)
        self.vidc_info.write('\nPersist Buffers:\n')
        self.vidc_info.write('='*40+'\n')
        if not(internal_buf_walker.is_empty()) :
            persistbufs = internal_buf_walker.next()
            self.vidc_info.write('Type\t'.expandtabs(40))
            self.vidc_info.write('Device_Addr\tSize\tFlags\n'.expandtabs(10)+'-'*80+'\n')
            internal_buf_walker.walk(persistbufs,self.internal_buf_walker)
        else:
            self.vidc_info.write('NONE\n')

        pending_getpropq = head
        + self.ramdump.field_offset('struct msm_vidc_inst', 'pending_getpropq')
        getprop_buf_walker = llist.ListWalker(self.ramdump, pending_getpropq, offset_vidc_list)
        self.vidc_info.write('\nPending Getpropq Buffers:\n')
        self.vidc_info.write('='*40+'\n')
        if not(getprop_buf_walker.is_empty()) :
            pending_getpropq = getprop_buf_walker.next()
            getprop_buf_walker.walk(pending_getpropq,self.getprop_buf_walker)
        else:
            self.vidc_info.write('NONE\n')

        outputbufs = head + self.ramdump.field_offset('struct msm_vidc_inst', 'outputbufs')
        internal_buf_walker = llist.ListWalker(self.ramdump, outputbufs, offset_vidc_list)
        self.vidc_info.write('\nOutput Buffers:\n')
        self.vidc_info.write('='*40+'\n')
        if not(internal_buf_walker.is_empty()) :
            outputbufs = internal_buf_walker.next()
            self.vidc_info.write('Type\t'.expandtabs(40))
            self.vidc_info.write('Device_Addr\tSize\tFlags\n'.expandtabs(10)+'-'*80+'\n')
            internal_buf_walker.walk(outputbufs,self.internal_buf_walker)
        else:
            self.vidc_info.write('NONE\n')

        '''reconbufs = head + self.ramdump.field_offset('struct msm_vidc_inst', 'reconbufs')
        recon_buf_walker = llist.ListWalker(self.ramdump, reconbufs, offset_vidc_list)
        self.vidc_info.write('\nReconstruction Buffers:\n')
        self.vidc_info.write('='*40+'\n')
        if not(recon_buf_walker.is_empty()) :
            reconbufs = recon_buf_walker.next()
            self.vidc_info.write('Index\tCR\tCF\n'.expandtabs(10)+'-'*30+'\n')
            recon_buf_walker.walk(reconbufs,self.recon_buf_walker)
        else:
            self.vidc_info.write('NONE\n')'''

        eosbufs = head + self.ramdump.field_offset('struct msm_vidc_inst', 'eosbufs')
        eos_buf_walker = llist.ListWalker(self.ramdump, eosbufs, offset_vidc_list)
        self.vidc_info.write('\nEOS Buffers:\n')
        self.vidc_info.write('='*40+'\n')
        if not(eos_buf_walker.is_empty()) :
            eosbufs = eos_buf_walker.next()
            self.vidc_info.write('Type\t'.expandtabs(40))
            self.vidc_info.write('Device_Addr\tSize\tFlags\tRefCnt\tFd\tOffset\n'\
                                 .expandtabs(10)+'-'*120+'\n')
            eos_buf_walker.walk(eosbufs,self.eos_buf_walker)
        else:
            self.vidc_info.write('NONE\n')

        registeredbufs = head + self.ramdump.field_offset('struct msm_vidc_inst', 'registeredbufs')
        vidc_buf_walker = llist.ListWalker(self.ramdump, registeredbufs, offset_vidc_list)
        self.vidc_info.write('\nRegistered Buffers:\n')
        self.vidc_info.write('='*40+'\n')
        if not(vidc_buf_walker.is_empty()) :
            registeredbufs = vidc_buf_walker.next()
            self.vidc_info.write('Vb2_Idx\tPlane\t'.expandtabs(10))
            self.vidc_info.write('Type\t'.expandtabs(40))
            self.vidc_info.write('Device_Addr\tSize\tFlags\tRefCnt\tFd\tOffset\n'
                                 .expandtabs(10)+'-'*140+'\n')
            vidc_buf_walker.walk(registeredbufs,self.vidc_buf_walker)
        else:
            self.vidc_info.write('NONE\n')

    def buff_requirements (self,head):

        buffer_type = self.ramdump.read_structure_field(
            head, 'struct hal_buffer_requirements ', 'buffer_type')
        try:
            buffer_type_str = self.hal_buf_type_dict[hex(buffer_type)]
        except:
            return

        if buffer_type_str == 'HAL_BUFFER_NONE':
            return

        self.vidc_info.write('{0}\t'.format(buffer_type_str).expandtabs(40))

        buffer_size = self.ramdump.read_structure_field(
            head, 'struct hal_buffer_requirements ', 'buffer_size')
        self.vidc_info.write('{0}\t'.format(buffer_size).expandtabs(20))

        buffer_region_size = self.ramdump.read_structure_field(
            head, 'struct hal_buffer_requirements ', 'buffer_region_size')
        self.vidc_info.write('{0}\t'.format(buffer_region_size).expandtabs(20))

        buffer_count_min = self.ramdump.read_structure_field(
            head, 'struct hal_buffer_requirements ', 'buffer_count_min')
        self.vidc_info.write('{0}\t'.format(buffer_count_min).expandtabs(20))

        buffer_count_min_host = self.ramdump.read_structure_field(
            head, 'struct hal_buffer_requirements ', 'buffer_count_min_host')
        self.vidc_info.write('{0}\t'.format(buffer_count_min_host).expandtabs(20))

        buffer_count_actual = self.ramdump.read_structure_field(
            head, 'struct hal_buffer_requirements ', 'buffer_count_actual')
        self.vidc_info.write('{0}\t'.format(buffer_count_actual).expandtabs(20))

        contiguous = self.ramdump.read_structure_field(
            head, 'struct hal_buffer_requirements ', 'contiguous')
        self.vidc_info.write('{0}\t'.format(contiguous).expandtabs(20))

        buffer_alignment = self.ramdump.read_structure_field(
            head, 'struct hal_buffer_requirements ', 'buffer_alignment')
        self.vidc_info.write('{0}\n'.format(buffer_alignment))



    def internal_buf_walker(self,head):

        smem = head + self.ramdump.field_offset('struct internal_buf', 'smem')
        internal= True

        self.print_smem_buf(smem,internal)

    def getprop_buf_walker(self,head):

        data = self.ramdump.read_structure_field(
            head, 'struct getprop_buf ', 'data')
        self.vidc_info.write('data: \t{0}\n'.format(data))


    def recon_buf_walker(self,head):

        buffer_index = self.ramdump.read_structure_field(
            head, 'struct recon_buf ', 'buffer_index')
        self.vidc_info.write('{0}\t'.format(buffer_index).expandtabs(10))

        CR = self.ramdump.read_structure_field(
            head, 'struct recon_buf ', 'CR')
        self.vidc_info.write('{0}\t'.format(CR).expandtabs(10))

        CF = self.ramdump.read_structure_field(
            head, 'struct recon_buf ', 'CF')
        self.vidc_info.write('{0}\n'.format(CF))

    def eos_buf_walker(self,head):

        smem = head + self.ramdump.field_offset('struct eos_buf', 'smem')

        self.print_smem_buf(smem)

    def vidc_buf_walker(self,head):

        vb2_buffer = head + self.ramdump.field_offset('struct msm_vidc_buffer', 'vvb')
        + self.ramdump.field_offset('struct vb2_v4l2_buffer', 'vb2_buf')

        index_vb2 = self.ramdump.read_structure_field(
            vb2_buffer, 'struct vb2_buffer', 'index')

        num_planes = self.ramdump.read_structure_field(
            vb2_buffer, 'struct vb2_buffer', 'num_planes')

        for i in range(0,num_planes,1):
            self.vidc_info.write('{0}\t'.format(index_vb2).expandtabs(10))
            self.vidc_info.write('{0}\t'.format(i).expandtabs(10))
            smem = head + self.ramdump.field_offset('struct msm_vidc_buffer', 'smem['+str(i)+']')
            self.print_smem_buf(smem)

    def print_smem_buf (self,smem,internal = False):

        buffer_type = self.ramdump.read_structure_field(
            smem, 'struct msm_smem', 'buffer_type')
        buffer_type_str = self.hal_buf_type_dict[hex(buffer_type)]
        self.vidc_info.write('{0}\t'.format(buffer_type_str).expandtabs(40))

        device_addr = self.ramdump.read_structure_field(
            smem, 'struct msm_smem', 'device_addr')
        self.vidc_info.write('0x{:01x}\t'.format(device_addr).expandtabs(10))

        size = self.ramdump.read_structure_field(
            smem, 'struct msm_smem', 'size')
        self.vidc_info.write('{0}\t'.format(size).expandtabs(10))

        flags = self.ramdump.read_structure_field(
            smem, 'struct msm_smem', 'flags')
        self.vidc_info.write('{0}\t'.format(flags).expandtabs(10))

        if not internal:
            refcount = self.ramdump.read_structure_field(
                smem, 'struct msm_smem', 'refcount')
            self.vidc_info.write('{0}\t'.format(refcount).expandtabs(10))

            fd = self.ramdump.read_structure_field(
                smem, 'struct msm_smem', 'fd')
            self.vidc_info.write('{0}\t'.format(fd).expandtabs(10))

            offset = self.ramdump.read_structure_field(
                smem, 'struct msm_smem', 'offset')
            self.vidc_info.write('{0}\t'.format(offset).expandtabs(10))

        self.vidc_info.write('\n')

    def get_hfi_device(self,head):

        intr_status = self.ramdump.read_structure_field(
            head, 'struct venus_hfi_device', 'intr_status')
        self.vidc_info.write('intr_status: \t{0}\n'.format(intr_status).expandtabs(20))

        device_id = self.ramdump.read_structure_field(
            head, 'struct venus_hfi_device', 'device_id')
        self.vidc_info.write('device_id: \t{0}\n'.format(device_id).expandtabs(20))

        clk_freq = self.ramdump.read_structure_field(
            head, 'struct venus_hfi_device', 'clk_freq')
        self.vidc_info.write('clk_freq: \t{0}\n'.format(clk_freq).expandtabs(20))

        last_packet_type = self.ramdump.read_structure_field(
            head, 'struct venus_hfi_device', 'last_packet_type')
        try:
            last_packet_type = self.msg_dict[format(last_packet_type,'#010x')]
        except:
            last_packet_type = self.cmd_dict[format(last_packet_type,'#010x')]

        self.vidc_info.write('last_packet_type: \t{0}\n'.format(last_packet_type).expandtabs(20))

        clk_bitrate = self.ramdump.read_structure_field(
            head, 'struct venus_hfi_device', 'clk_bitrate')
        self.vidc_info.write('clk_bitrate: \t{0}\n'.format(clk_bitrate).expandtabs(20))

        scaled_rate = self.ramdump.read_structure_field(
            head, 'struct venus_hfi_device', 'scaled_rate')
        self.vidc_info.write('scaled_rate: \t{0}\n'.format(scaled_rate).expandtabs(20))

        power_enabled_addr = head + self.ramdump.field_offset(
            'struct venus_hfi_device', 'power_enabled')
        power_enabled = self.ramdump.read_bool(power_enabled_addr)
        self.vidc_info.write('power_enabled: \t{0}\n'.format(power_enabled).expandtabs(20))

        spur_count = self.ramdump.read_structure_field(
            head, 'struct venus_hfi_device', 'spur_count')
        self.vidc_info.write('spur_count: \t{0}\n'.format(spur_count).expandtabs(20))

        reg_count = self.ramdump.read_structure_field(
            head, 'struct venus_hfi_device', 'reg_count')
        self.vidc_info.write('reg_count: \t{0}\n'.format(reg_count).expandtabs(20))

        state = self.ramdump.read_structure_field(
            head, 'struct venus_hfi_device', 'state')
        self.vidc_info.write('state: \t{0}\n'.format(state).expandtabs(20))

        packetization_type = self.ramdump.read_structure_field(
            head, 'struct venus_hfi_device', 'packetization_type')
        self.vidc_info.write('packetization_type: {0}\n'.format(packetization_type))

        skip_pc_count = self.ramdump.read_structure_field(
            head, 'struct venus_hfi_device', 'skip_pc_count')
        self.vidc_info.write('skip_pc_count: \t{0}\n'.format(skip_pc_count).expandtabs(20))

        self.queue_info = self.ramdump.open_file('cmd_queue_info.txt')

        self.queue_info.write('\nCMD QUEUE\n=========\n')
        q_cmd = head + self.ramdump.field_offset('struct venus_hfi_device','iface_queues[0]')
        try:
            self.read_interface_queue(q_cmd)
        except:
            self.queue_info.write('\n Unable to read command queue');

        self.queue_info.close()

        self.queue_info = self.ramdump.open_file('msg_queue_info.txt')

        self.queue_info.write('\nMSG QUEUE\n=========\n')
        q_msg = head + self.ramdump.field_offset('struct venus_hfi_device','iface_queues[1]')
        try:
            self.read_interface_queue(q_msg)
        except:
            self.queue_info.write('\n Unable to read message queue');

        self.queue_info.close()

    def read_interface_queue (self,q_addr):

        q_hdr = self.ramdump.read_structure_field(
            q_addr, 'struct vidc_iface_q_info', 'q_hdr')

        qhdr_status = self.ramdump.read_structure_field(
            q_hdr, 'struct hfi_queue_header', 'qhdr_status')
        self.queue_info.write('qhdr_status: \t{0}\n'.format(qhdr_status).expandtabs(20))

        qhdr_start_addr = self.ramdump.read_structure_field(
            q_hdr, 'struct hfi_queue_header', 'qhdr_start_addr')
        self.queue_info.write('qhdr_start_addr: \t0x{:1x}\n'.format(qhdr_start_addr).expandtabs(20))

        qhdr_type = self.ramdump.read_structure_field(
            q_hdr, 'struct hfi_queue_header', 'qhdr_type')
        qhdr_type = int(hex(qhdr_type)[-1])
        self.queue_info.write('qhdr_type: \t{0}\n'\
                              .format('Q_MSG' if qhdr_type else 'Q_CMD').expandtabs(20))

        qhdr_q_size = self.ramdump.read_structure_field(
            q_hdr, 'struct hfi_queue_header', 'qhdr_q_size')
        self.queue_info.write('qhdr_q_size: \t{0}\n'.format(qhdr_q_size).expandtabs(20))

        qhdr_pkt_drop_cnt = self.ramdump.read_structure_field(
            q_hdr, 'struct hfi_queue_header', 'qhdr_pkt_drop_cnt')
        self.queue_info.write('qhdr_pkt_drop_cnt: \t{0}\n'.format(qhdr_pkt_drop_cnt).expandtabs(20))

        qhdr_rx_wm = self.ramdump.read_structure_field(
            q_hdr, 'struct hfi_queue_header', 'qhdr_rx_wm')
        self.queue_info.write('qhdr_rx_wm: \t{0}\n'.format(qhdr_rx_wm).expandtabs(20))

        qhdr_tx_wm = self.ramdump.read_structure_field(
            q_hdr, 'struct hfi_queue_header', 'qhdr_tx_wm')
        self.queue_info.write('qhdr_tx_wm: \t{0}\n'.format(qhdr_tx_wm).expandtabs(20))

        qhdr_rx_req = self.ramdump.read_structure_field(
            q_hdr, 'struct hfi_queue_header', 'qhdr_rx_req')
        self.queue_info.write('qhdr_rx_req: \t{0}\n'.format(qhdr_rx_req).expandtabs(20))

        qhdr_tx_req = self.ramdump.read_structure_field(
            q_hdr, 'struct hfi_queue_header', 'qhdr_tx_req')
        self.queue_info.write('qhdr_tx_req: \t{0}\n'.format(qhdr_tx_req).expandtabs(20))

        qhdr_rx_irq_status = self.ramdump.read_structure_field(
            q_hdr, 'struct hfi_queue_header', 'qhdr_rx_irq_status')
        self.queue_info.write('qhdr_rx_irq_status: {0}\n'.format(qhdr_rx_irq_status))

        qhdr_tx_irq_status = self.ramdump.read_structure_field(
            q_hdr, 'struct hfi_queue_header', 'qhdr_tx_irq_status')
        self.queue_info.write('qhdr_tx_irq_status: {0}\n'.format(qhdr_tx_irq_status))

        qhdr_read_idx = self.ramdump.read_structure_field(
            q_hdr, 'struct hfi_queue_header', 'qhdr_read_idx')
        self.queue_info.write('qhdr_read_idx: \t{0}\n'.format(qhdr_read_idx).expandtabs(20))

        qhdr_write_idx = self.ramdump.read_structure_field(
            q_hdr, 'struct hfi_queue_header', 'qhdr_write_idx')
        self.queue_info.write('qhdr_write_idx: \t{0}\n'.format(qhdr_write_idx).expandtabs(20))

        vidc_mem_addr = q_addr + self.ramdump.field_offset('struct vidc_iface_q_info','q_array')
        align_virtual_addr = self.ramdump.read_structure_field(
            vidc_mem_addr, 'struct vidc_mem_addr', 'align_virtual_addr')
        self.queue_info.write('base_addr: \t0x{:1x}\n'.format(align_virtual_addr).expandtabs(20))

        self.read_packets(qhdr_read_idx,qhdr_write_idx,align_virtual_addr,qhdr_type)

    def read_packets (self,read_idx,write_idx,base_addr,qhdr_type):

        self.queue_info.write('\n=============PACKET STATS=============\n')

        packet_num = 0
        self.queue_info.write('\nINDEX\tREAD\tSESSION_ID\t'.expandtabs(20))
        self.queue_info.write('TYPE\t'.expandtabs(40))
        self.queue_info.write('SIZE\tPACKETS\n'.expandtabs(20)+'-'*200+'\n')
        while(write_idx >0):

            packet_num = packet_num + 1

            self.queue_info.write('{0}\t{1}\t'\
                                  .format(packet_num,(write_idx <= read_idx)).expandtabs(20))

            write_idx = write_idx-1
            bytes_read = 4
            pkt_size_addr = write_idx*4 + base_addr
            end_addr = pkt_size_addr

            while((((bytes_read!=self.ramdump.read_u32(pkt_size_addr)) or (bytes_read == 4)))
                 and (write_idx>0)):
                bytes_read = bytes_read + 4
                pkt_size_addr = pkt_size_addr - 4
                write_idx = write_idx - 1

                while ((bytes_read == self.ramdump.read_u32(pkt_size_addr)) and (write_idx > 0)):
                    try:
                        pkt_type_addr_temp = pkt_size_addr + 4
                        pkt_type_temp = format(self.ramdump.read_u32(pkt_type_addr_temp),'#010x')
                        if qhdr_type:
                            pkt_type_str = self.msg_dict[pkt_type_temp]
                        else :
                            pkt_type_str = self.cmd_dict[pkt_type_temp]
                        break
                    except:
                        bytes_read = bytes_read + 4
                        pkt_size_addr = pkt_size_addr - 4
                        write_idx = write_idx - 1

            start_addr = pkt_size_addr

            pkt_size = self.ramdump.read_u32(pkt_size_addr)

            pkt_type_addr = pkt_size_addr + 4
            pkt_type = self.ramdump.read_u32(pkt_type_addr)
            pkt_type = format(pkt_type,'#010x')

            try:
                pkt_type_str = self.msg_dict[pkt_type] if qhdr_type else self.cmd_dict[pkt_type]
            except:
                pkt_type_str = "UNKNOWN_PKT_TYPE"

            session_id_addr = pkt_size_addr + 8
            session_id = self.ramdump.read_u32(session_id_addr)
            self.queue_info.write('0x{:1x}\t'.format(session_id).expandtabs(20))
            self.queue_info.write('{0}\t'.format(pkt_type_str).expandtabs(40))
            self.queue_info.write('{0}\t'.format(pkt_size).expandtabs(20))

            while(start_addr <= end_addr):
                pkt = self.ramdump.read_u32(start_addr)
                self.queue_info.write('{0:08x} '.format(pkt))
                start_addr = start_addr + 4

            self.queue_info.write('\n')
