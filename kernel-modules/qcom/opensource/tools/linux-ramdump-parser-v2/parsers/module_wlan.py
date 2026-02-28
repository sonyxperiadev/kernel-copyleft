# Copyright (c) 2016 The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import parser_util
from parser_util import register_parser, RamParser
from print_out import print_out_str


@register_parser(
    '--print-wlan',
    'Print WLAN debugging information(if enabled)',
    optional=True)
class ModuleWlan(RamParser):
    """ This class defines when WLAN module is loaded """
    def __init__(self, *args):
        super(ModuleWlan, self).__init__(*args)

        self.dir_char = ''
        self.dir_char_out = ''
        self.opt_dbg = False

        self.wlan_path = ''
        self.wlan_module_addr = 0
        self.wlan_text_addr = 0
        self.wlan_data_addr = 0
        self.wlan_bss_addr = 0

    def convert_dir_for_arg(self, ori):
        """
            Convert full path as an argument of function
        """
        if self.opt_dbg is True:
            print_out_str('** convert_dir_for_arg() **')

        if self.dir_char == '':
            if parser_util.get_system_type() == 'Linux':
                self.dir_char = '/'
                self.dir_char_out = '/'
            else:
                self.dir_char = '\\'
                self.dir_char_out = '\\\\'
        dst = ''
        for c in ori:
            if c == self.dir_char:
                dst = dst + self.dir_char_out
            else:
                dst = dst + c

        if self.opt_dbg is True:
            print_out_str('ori - [{}]'.format(ori))
            print_out_str('dst - [{}]'.format(dst))

        return dst

    def load_wlan_ko(self):
        """ Load wlan.ko to GDB """

        if self.opt_dbg is True:
            print_out_str('** load_wlan_ko() **')

        if self.wlan_text_addr == 0:
            print_out_str('self.wlan_text_addr is zero')
            return False

        cmd = 'add-symbol-file '
        cmd = cmd + self.convert_dir_for_arg(self.wlan_path)
        cmd = cmd + ' {} -s .data {} -s .bss {}'.format(
                self.wlan_text_addr, self.wlan_data_addr,
                self.wlan_bss_addr)

        return self.ramdump.gdbmi._run_for_first(cmd)

    def get_sections_of_wlan(self):
        """
            Get wlan.ko's sectino addresses
        """
        if self.opt_dbg is True:
            print_out_str('** get_sections_of_wlan() **')

        # Step-A) Find wlan.ko
        modules_addr = self.ramdump.address_of('modules')
        next_module_addr = self.ramdump.read_structure_field(
                                modules_addr, 'struct list_head', 'next')
        name_offset = self.ramdump.field_offset('struct module', 'name')
        module_addr = 0
        idx = 0

        while modules_addr != next_module_addr:
            module_addr = self.ramdump.container_of(
                                next_module_addr, 'struct module', 'list')
            module_name_addr = module_addr + name_offset
            module_name_str = self.ramdump.read_cstring(
                                module_name_addr, 32, True)

            if module_name_str == 'wlan':
                self.wlan_module_addr = module_addr
                break

            if self.opt_dbg is True:
                print_out_str(
                    '[{}]th - next_module[{}], module[{}], name[{}]'.format(
                        hex(idx), hex(next_module_addr),
                        hex(module_addr), module_name_str))

            # move the list entry to the next
            next_module_addr = self.ramdump.read_structure_field(
                                modules_addr, 'struct list_head', 'next')
            idx = idx + 1

        if self.wlan_module_addr == 0:
            print_out_str('[Caution] Fail to find wlan.ko')
            return False

        # Step-B) get sections in wlan.ko
        sect_attrs_addr = self.ramdump.read_structure_field(
                            module_addr, 'struct module', 'sect_attrs')
        nsections = self.ramdump.read_structure_field(
                            sect_attrs_addr,
                            'struct module_sect_attrs',
                            'nsections')
        attrs_offset = self.ramdump.field_offset(
                            'struct module_sect_attrs', 'attrs')
        attrs_addr = sect_attrs_addr + attrs_offset
        module_sect_attr_size = self.ramdump.sizeof('struct module_sect_attr')

        if self.opt_dbg is True:
            print_out_str('module_addr : {}'.format(hex(module_addr)))
            print_out_str('sect_attrs_addr : {}'.format(hex(sect_attrs_addr)))
            print_out_str('nsections : {}'.format(hex(nsections)))
            print_out_str('attrs_offset : {}'.format(hex(attrs_offset)))
            if attrs_addr is not None:
                print_out_str('attrs_addr : {}'.format(hex(attrs_addr)))
            else:
                print_out_str('attrs_addr : {}'.format(attrs_addr))

        section_name_offset = self.ramdump.field_offset(
                                            'struct module_sect_attr', 'name')
        idx = 0
        while idx < nsections:
            section_attr_address = attrs_addr + idx * module_sect_attr_size
            section_name_addr = self.ramdump.read_pointer(
                                section_attr_address + section_name_offset)
            section_name_str = self.ramdump.read_cstring(
                                        section_name_addr, 32, True)
            section_address = self.ramdump.read_structure_field(
                                        section_attr_address,
                                        'struct module_sect_attr', 'address')

            if self.opt_dbg is True:
                print_out_str('section[{}]th - name[{}], attr[{}]'.format(
                            hex(idx), section_name_str, hex(section_address)))

            if section_name_str == '.text':
                self.wlan_text_addr = section_address
            if section_name_str == '.data':
                self.wlan_data_addr = section_address
            if section_name_str == '.bss':
                self.wlan_bss_addr = section_address
            idx = idx + 0x1

        print_out_str('wlan_text_addr : {}'.format(hex(self.wlan_text_addr)))
        print_out_str('wlan_data_addr : {}'.format(hex(self.wlan_data_addr)))
        print_out_str('wlan_bss_addr : {}'.format(hex(self.wlan_bss_addr)))

        return True

    def run(self):
        """
            Main
        """

        if self.ramdump.arm64 is None:
            print_out_str('[Caution] this script supports on ARM64')
            return False

        if self.ramdump.wlan == "INTEGRATED":
            print_out_str('self.wlan doen\'t exist, skip')
        else:
            print_out_str('self.wlan exist {}'.format(self.ramdump.wlan))
            self.wlan_path = self.ramdump.wlan

            if self.get_sections_of_wlan() is False:
                print_out_str('wlan.ko is not loaded.')
                return False
            else:
                print_out_str('** Find WLAN Module **')
                self.load_wlan_ko()

        self.get_wmi_command_log_buffer()
        self.get_host_wmi_command_tx_cmp_buf()
        self.get_host_wmi_event_buf()
        self.get_host_wmi_rx_event_buf()
        self.get_host_extract_log()

        return True

    ###################################
    # Parse internal variables
    def get_wmi_command_log_buffer(self):
        """
            Parse 'struct wmi_command_debug'
        """

        if self.opt_dbg is True:
            print_out_str('*** get_wmi_command_log_buffer() ***')

        element_size = self.ramdump.sizeof('struct wmi_command_debug')
        if (element_size is None):
            print_out_str('[Caution] symbols of host driver do not exist')
            return False

        out_file = self.ramdump.open_file("wmi_command_log_buffer.txt")

        wmi_total_size = self.ramdump.sizeof('wmi_command_log_buffer')
        num_elements = wmi_total_size / element_size

        if self.opt_dbg is True:
            print_out_str('** wlan_host_wmi_command_log_buffer **')
            print_out_str('*************************************')
            print_out_str('wmi_total_size({})'.format(hex(wmi_total_size)))
            print_out_str('element_size({})'.format(hex(element_size)))
            print_out_str('num_elements({})'.format(hex(num_elements)))
            print_out_str('*************************************')

        # info of the data structure
        command_offset = self.ramdump.field_offset(
                            'struct wmi_command_debug', 'command')
        data0_offset = self.ramdump.field_offset(
                            'struct wmi_command_debug', 'data[0]')
        data1_offset = self.ramdump.field_offset(
                            'struct wmi_command_debug', 'data[1]')
        data2_offset = self.ramdump.field_offset(
                            'struct wmi_command_debug', 'data[2]')
        data3_offset = self.ramdump.field_offset(
                            'struct wmi_command_debug', 'data[3]')
        time_offset = self.ramdump.field_offset(
                            'struct wmi_command_debug', 'time')

        if self.opt_dbg is True:
            print_out_str('command_offset({})'.format(command_offset))
            print_out_str('data0_offset({})'.format(data0_offset))
            print_out_str('data1_offset({})'.format(data1_offset))
            print_out_str('data2_offset({})'.format(data2_offset))
            print_out_str('data3_offset({})'.format(data3_offset))
            print_out_str('time_offset({})'.format(time_offset))
            print_out_str('*************************************')

        buffer_start_address = self.ramdump.address_of(
                                    'wmi_command_log_buffer')
        wmi_command_buf_idx = self.ramdump.read_u32(
                                self.ramdump.address_of(
                                    'g_wmi_command_buf_idx'))

        cnt = 0
        idx = wmi_command_buf_idx
        while cnt < num_elements:
            if idx == num_elements:
                idx = 0

            buffer_address = buffer_start_address + idx * element_size
            command = self.ramdump.read_u32(buffer_address)
            data0 = self.ramdump.read_u32(buffer_address + data0_offset)
            data1 = self.ramdump.read_u32(buffer_address + data1_offset)
            data2 = self.ramdump.read_u32(buffer_address + data2_offset)
            data3 = self.ramdump.read_u32(buffer_address + data3_offset)
            time = self.ramdump.read_u64(buffer_address + time_offset)

            idx = idx + 1
            cnt = cnt + 1

            out_buf = '{0} us'.format(float(time/100000.0))
            out_buf = out_buf + ' : command({})'.format(hex(command))
            out_buf = out_buf + ', data[{}'.format(hex(data0))
            out_buf = out_buf + ', {}'.format(hex(data1))
            out_buf = out_buf + ', {}'.format(hex(data2))
            out_buf = out_buf + ', {}]'.format(hex(data3))
            if self.opt_dbg is True:
                print_out_str(out_buf)
            out_file.write(out_buf + '\n')
        out_file.close()

        return True

    def get_host_wmi_command_tx_cmp_buf(self):
        """
            Parse 'struct wmi_command_debug wmi_command_tx_cmp_log_buffer'
        """
        if self.opt_dbg is True:
            print_out_str('*** get_host_wmi_command_tx_cmp_buf() ***')

        element_size = self.ramdump.sizeof('struct wmi_command_debug')
        if (element_size is None):
            print_out_str('[Caution] symbols of host driver do not exist')
            return False

        out_file = self.ramdump.open_file("wmi_command_tx_cmp_buf.txt")

        wmi_total_size = self.ramdump.sizeof('wmi_command_tx_cmp_log_buffer')
        num_elements = wmi_total_size / element_size

        if self.opt_dbg is True:
            print_out_str('** wlan_host_wmi_command_tx_cmp_buf **')
            print_out_str('*************************************')
            print_out_str('wmi_total_size({})'.format(hex(wmi_total_size)))
            print_out_str('element_size({})'.format(hex(element_size)))
            print_out_str('num_elements({})'.format(hex(num_elements)))
            print_out_str('*************************************')

        # info of the data structure
        command_offset = self.ramdump.field_offset(
                            'struct wmi_command_debug', 'command')
        data0_offset = self.ramdump.field_offset(
                            'struct wmi_command_debug', 'data[0]')
        data1_offset = self.ramdump.field_offset(
                            'struct wmi_command_debug', 'data[1]')
        data2_offset = self.ramdump.field_offset(
                            'struct wmi_command_debug', 'data[2]')
        data3_offset = self.ramdump.field_offset(
                            'struct wmi_command_debug', 'data[3]')
        time_offset = self.ramdump.field_offset(
                            'struct wmi_command_debug', 'time')

        if self.opt_dbg is True:
            print_out_str("command_offset({})".format(command_offset))
            print_out_str("data0_offset({})".format(data0_offset))
            print_out_str("data1_offset({})".format(data1_offset))
            print_out_str("data2_offset({})".format(data2_offset))
            print_out_str("data3_offset({})".format(data3_offset))
            print_out_str("time_offset({})".format(time_offset))
            print_out_str('*************************************')

        log_buffer_address = self.ramdump.address_of(
                                    'wmi_command_tx_cmp_log_buffer')
        wmi_command_buf_idx = self.ramdump.read_u32(
                                self.ramdump.address_of(
                                    'g_wmi_command_tx_cmp_buf_idx'))

        cnt = 0
        idx = wmi_command_buf_idx
        while cnt < num_elements:
            if idx == num_elements:
                idx = 0

            buffer_address = log_buffer_address + idx * element_size
            command = self.ramdump.read_u32(buffer_address)
            data0 = self.ramdump.read_u32(buffer_address + data0_offset)
            data1 = self.ramdump.read_u32(buffer_address + data1_offset)
            data2 = self.ramdump.read_u32(buffer_address + data2_offset)
            data3 = self.ramdump.read_u32(buffer_address + data3_offset)
            time = self.ramdump.read_u64(buffer_address + time_offset)

            idx = idx + 1
            cnt = cnt + 1

            out_buf = '{0} us'.format(float(time/100000.0))
            out_buf = out_buf + ' : command({})'.format(hex(command))
            out_buf = out_buf + ', data[{}'.format(hex(data0))
            out_buf = out_buf + ', {}'.format(hex(data1))
            out_buf = out_buf + ', {}'.format(hex(data2))
            out_buf = out_buf + ', {}]'.format(hex(data3))
            if self.opt_dbg is True:
                print_out_str(out_buf)
            out_file.write(out_buf + '\n')

        out_file.close()
        return True

    def get_host_wmi_event_buf(self):
        """
            Parse 'struct wmi_event_debug wmi_event_log_buffer[]'
        """

        if self.opt_dbg is True:
            print_out_str('*** get_host_wmi_event_buf() ***')

        element_size = self.ramdump.sizeof('struct wmi_event_debug')
        if (element_size is None):
            print_out_str('[Caution] symbols of host driver do not exist')
            return False

        out_file = self.ramdump.open_file("wmi_event_log_buffer.txt")

        wmi_total_size = self.ramdump.sizeof('wmi_event_log_buffer')
        num_elements = wmi_total_size / element_size

        if self.opt_dbg is True:
            print_out_str('[Debug] wmi_total_size({})'.format(
                            hex(wmi_total_size)))
            print_out_str('[Debug] element_size({})'.format(hex(element_size)))
            print_out_str('[Debug] num_elements({})'.format(hex(num_elements)))

        # info of the data structure
        event_offset = self.ramdump.field_offset(
                            'struct wmi_event_debug', 'event')
        data0_offset = self.ramdump.field_offset(
                            'struct wmi_event_debug', 'data[0]')
        data1_offset = self.ramdump.field_offset(
                            'struct wmi_event_debug', 'data[1]')
        data2_offset = self.ramdump.field_offset(
                            'struct wmi_event_debug', 'data[2]')
        data3_offset = self.ramdump.field_offset(
                            'struct wmi_event_debug', 'data[3]')
        time_offset = self.ramdump.field_offset(
                            'struct wmi_event_debug', 'time')

        if self.opt_dbg is True:
            print_out_str("[Debug] event_offset({})".format(event_offset))
            print_out_str("[Debug] data0_offset({})".format(data0_offset))
            print_out_str("[Debug] data1_offset({})".format(data1_offset))
            print_out_str("[Debug] data2_offset({})".format(data2_offset))
            print_out_str("[Debug] data3_offset({})".format(data3_offset))
            print_out_str("[Debug] time_offset({})".format(time_offset))

        wmi_log_address = self.ramdump.address_of('wmi_event_log_buffer')
        wmi_event_buf_idx = self.ramdump.read_u32(
                                self.ramdump.address_of('g_wmi_event_buf_idx'))

        cnt = 0
        idx = wmi_event_buf_idx
        while cnt < num_elements:
            if idx == num_elements:
                idx = 0

            buffer_address = wmi_log_address + idx * element_size
            event = self.ramdump.read_u32(buffer_address)
            data0 = self.ramdump.read_u32(buffer_address + data0_offset)
            data1 = self.ramdump.read_u32(buffer_address + data1_offset)
            data2 = self.ramdump.read_u32(buffer_address + data2_offset)
            data3 = self.ramdump.read_u32(buffer_address + data3_offset)
            time = self.ramdump.read_u64(buffer_address + time_offset)

            idx = idx + 1
            cnt = cnt + 1

            out_buf = '{0} us'.format(float(time/100000.0))
            out_buf = out_buf + ' : event({})'.format(hex(event))
            out_buf = out_buf + ', data[{}'.format(hex(data0))
            out_buf = out_buf + ', {}'.format(hex(data1))
            out_buf = out_buf + ', {}'.format(hex(data2))
            out_buf = out_buf + ', {}]'.format(hex(data3))
            if self.opt_dbg is True:
                print_out_str(out_buf)
            out_file.write(out_buf + '\n')

        out_file.close()
        return True

    def get_host_wmi_rx_event_buf(self):
        """
            Parse 'struct wmi_event_debug wmi_rx_event_log_buffer'
        """

        if self.opt_dbg is True:
            print_out_str('*** get_host_wmi_rx_event_buf() ***')

        wmi_elem_size = self.ramdump.sizeof('struct wmi_event_debug')
        if (wmi_elem_size is None):
            print_out_str('[Caution] symbols of host driver do not exist')
            return False
        wmi_total_size = self.ramdump.sizeof('wmi_rx_event_log_buffer')
        num_elements = wmi_total_size / wmi_elem_size

        out_file = self.ramdump.open_file("wmi_rx_event_log_buffer.txt")

        # info of the data structure
        event_offset = self.ramdump.field_offset(
                                'struct wmi_event_debug', 'event')
        data0_offset = self.ramdump.field_offset(
                                'struct wmi_event_debug', 'data[0]')
        data1_offset = self.ramdump.field_offset(
                                'struct wmi_event_debug', 'data[1]')
        data2_offset = self.ramdump.field_offset(
                                'struct wmi_event_debug', 'data[2]')
        data3_offset = self.ramdump.field_offset(
                                'struct wmi_event_debug', 'data[3]')
        time_offset = self.ramdump.field_offset(
                                'struct wmi_event_debug', 'time')

        wmi_event_address = self.ramdump.address_of('wmi_rx_event_log_buffer')
        wmi_event_buf_idx = self.ramdump.read_u32(
                                self.ramdump.address_of(
                                    'g_wmi_rx_event_buf_idx'))

        if self.opt_dbg is True:
            print_out_str('[Debug] wmi_total_size({})'.format(wmi_total_size))
            print_out_str('[Debug] wmi_elem_size({})'.format(wmi_elem_size))
            print_out_str('[Debug] num_elements({})'.format(num_elements))
            print_out_str('[Debug] event_offset({})'.format(event_offset))
            print_out_str('[Debug] data0_offset({})'.format(data0_offset))
            print_out_str('[Debug] data1_offset({})'.format(data1_offset))
            print_out_str('[Debug] data2_offset({})'.format(data2_offset))
            print_out_str('[Debug] data3_offset({})'.format(data3_offset))
            print_out_str('[Debug] time_offset({})'.format(time_offset))

        cnt = 0
        idx = wmi_event_buf_idx
        while cnt < num_elements:
            if idx == num_elements:
                idx = 0

            buffer_address = wmi_event_address + idx * wmi_elem_size
            event = self.ramdump.read_u32(buffer_address)
            data0 = self.ramdump.read_u32(buffer_address + data0_offset)
            data1 = self.ramdump.read_u32(buffer_address + data1_offset)
            data2 = self.ramdump.read_u32(buffer_address + data2_offset)
            data3 = self.ramdump.read_u32(buffer_address + data3_offset)
            time = self.ramdump.read_u64(buffer_address + time_offset)

            out_buf = '{0} us'.format(float(time/100000.0))
            out_buf = out_buf + ' : event({})'.format(hex(event))
            out_buf = out_buf + ', data[{}'.format(hex(data0))
            out_buf = out_buf + ', {}'.format(hex(data1))
            out_buf = out_buf + ', {}'.format(hex(data2))
            out_buf = out_buf + ', {}]'.format(hex(data3))
            if self.opt_dbg is True:
                print_out_str(out_buf)
            out_file.write(out_buf + '\n')

            idx = idx + 1
            cnt = cnt + 1

        out_file.close()
        return True

    def get_host_extract_log(self):
        """
            refer functions in wlan_logging_sock_svc.c
        """

        if self.opt_dbg is True:
            print_out_str('*** wlan_host_extract_log() ***')

        out_file = self.ramdump.open_file("gwlan_logging.txt")

        # get number of struct wlan_logging
        num_buf = self.ramdump.read_s32(
                        self.ramdump.address_of('gwlan_logging') +
                        self.ramdump.field_offset(
                                'struct wlan_logging', 'num_buf'))
        if self.opt_dbg is True:
            print_out_str('num_buf : {}'.format(num_buf))

        # gwlan_logging
        element_size = self.ramdump.sizeof('struct log_msg')
        if element_size % 32:
            elem_aligned_size = element_size + (element_size % 32)
            if self.opt_dbg is True:
                print_out_str('element_size({})'.format(hex(element_size)))
                print_out_str('element_align_size({})'.format(
                                                hex(elem_aligned_size)))
        else:
            elem_aligned_size = element_size
            if self.opt_dbg is True:
                print_out_str('element_size({})'.format(hex(element_size)))
                print_out_str('element_align_size({})'.format(
                                                hex(elem_aligned_size)))

        filled_length_offset = self.ramdump.field_offset(
                                                'struct log_msg',
                                                'filled_length')
        logbuf_offset = self.ramdump.field_offset(
                                'struct log_msg', 'logbuf')
        logbuf_size = element_size - logbuf_offset
        gplog_msg_address = self.ramdump.read_pointer('gplog_msg')

        if self.opt_dbg is True:
            print_out_str('filled_length_offset : {}'.format(
                                                hex(filled_length_offset)))
            print_out_str('logbuf_size : {}'.format(hex(logbuf_size)))
            print_out_str('gplog_msg_address : {}'.format(
                                                hex(gplog_msg_address)))

        cnt = 0
        while cnt < num_buf:
            buffer_address = gplog_msg_address + cnt * elem_aligned_size

            filled_length = self.ramdump.read_u32(
                                buffer_address + filled_length_offset)

            v_address = buffer_address + logbuf_offset + 4
            p_address = self.ramdump.virt_to_phys(v_address)
            if self.opt_dbg is True:
                print_out_str('** gplog_msg[{}] : {}, {}, VA{}-PA{} **'.format(
                                    cnt,
                                    hex(buffer_address),
                                    hex(filled_length),
                                    hex(v_address),
                                    hex(p_address)))

            out_file.write('** gplog_msg[{}] : {}, {}, VA{}-PA{} **\n'.format(
                                cnt,
                                hex(buffer_address),
                                hex(filled_length),
                                hex(v_address),
                                hex(p_address)))

            if filled_length != 0:
                left_bytes = filled_length
                logbuf_str = ""
                while left_bytes > 0:
                    p_address = self.ramdump.virt_to_phys(v_address)
                    logbuf_out = self.ramdump.read_physical(p_address, 4)
                    logbuf_str = logbuf_str + logbuf_out

                    v_address = v_address + 4
                    left_bytes = left_bytes - 4

                if self.opt_dbg is True:
                    print_out_str(logbuf_str)
                out_file.write(logbuf_str)
                out_file.write('\n')

                # We may be able to delete first []
                # like [VosMCThread] or [kworker/0:0]
            cnt = cnt + 1

        out_file.close()
        return True

    def parse(self):
        self.run()
        return True
