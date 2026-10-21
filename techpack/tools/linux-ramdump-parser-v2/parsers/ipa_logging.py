"""
Copyright (c) 2020 The Linux Foundation. All rights reserved.

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

import os
import struct

from print_out import print_out_str
from parser_util import register_parser, RamParser


ipa_log = "ipalog.txt"


def setup_out_file(path, self):
    global out_file
    try:
        out_file = self.ramdump.open_file(path, 'wb')
        return out_file
    except:
        print_out_str("could not open path {0}".format(path))
        print_out_str("Do you have write/read permissions on the path?")


def print_out_ip(string):
    out_file.write((string + '\n').encode('ascii', 'ignore'))


@register_parser('--ipa', 'print ipa_logging information')
class ipa_logging(RamParser):

    def __init__(self, *args):
        super(ipa_logging, self).__init__(*args)

    def ipa_parse(self, ram_dump):
        curr_chan=0
        ep_idx=0
        ipa3_ctx_addr_offset = ram_dump.address_of('ipa3_ctx')
        ipa3_ctx_addr = ram_dump.read_u64(ipa3_ctx_addr_offset)
        ep_offset = ram_dump.field_offset('struct ipa3_context','ep')
        ipa_num_pipes_offset = ram_dump.field_offset('struct ipa3_context','ipa_num_pipes')
        valid_offset = ram_dump.field_offset('struct ipa3_ep_context','valid')
        gsi_chan_hdl_offset = ram_dump.field_offset('struct ipa3_ep_context','gsi_chan_hdl')
        gsi_evt_ring_hdl_offset = ram_dump.field_offset('struct ipa3_ep_context','gsi_evt_ring_hdl')
        client_offset = ram_dump.field_offset('struct ipa3_ep_context','client')
        sys_offset = ram_dump.field_offset('struct ipa3_ep_context','sys')

        repl_offset = ram_dump.field_offset('struct ipa3_sys_context','repl')
        len_offset = ram_dump.field_offset('struct ipa3_sys_context','len')
        curr_polling_state_offset = ram_dump.field_offset('struct ipa3_sys_context','curr_polling_state')

        head_idx_offset = ram_dump.field_offset('struct ipa3_repl_ctx','head_idx')
        tail_idx_offset = ram_dump.field_offset('struct ipa3_repl_ctx','tail_idx')
        capacity_offset = ram_dump.field_offset('struct ipa3_repl_ctx','capacity')
        pending_offset = ram_dump.field_offset('struct ipa3_repl_ctx','pending')
        page_recycle_repl_offset = ram_dump.field_offset('struct ipa3_sys_context','page_recycle_repl')
        page_recycle_repl_offset = ram_dump.field_offset('struct ipa3_sys_context','page_recycle_repl')

        ep_addr = ipa3_ctx_addr + ep_offset
        gsi_ctx_offset = ram_dump.address_of('gsi_ctx')
        gsi_ctx_addr = ram_dump.read_u64(gsi_ctx_offset)
        chan_offset = ram_dump.field_offset('struct gsi_ctx','chan')
        state_offset = ram_dump.field_offset('struct gsi_chan_ctx','state')
        poll_mode_offset = ram_dump.field_offset('struct gsi_chan_ctx','poll_mode')
        ring_offset = ram_dump.field_offset('struct gsi_chan_ctx','ring')
        stats_offset = ram_dump.field_offset('struct gsi_chan_ctx','stats')
        props_offset = ram_dump.field_offset('struct gsi_chan_ctx','props')
        evtr_offset = ram_dump.field_offset('struct gsi_chan_ctx','evtr')
        prot_offset = ram_dump.field_offset('struct gsi_chan_props','prot')
        ch_id_offset = ram_dump.field_offset('struct gsi_chan_props','ch_id')

        queued_offset = ram_dump.field_offset('struct gsi_chan_stats', 'queued')
        completed_offset = ram_dump.field_offset('struct gsi_chan_stats', 'completed')
        callback_to_poll_offset = ram_dump.field_offset('struct gsi_chan_stats', 'callback_to_poll')
        poll_to_callback_offset = ram_dump.field_offset('struct gsi_chan_stats', 'poll_to_callback')
        poll_pending_irq_offset = ram_dump.field_offset('struct gsi_chan_stats', 'poll_pending_irq')
        invalid_tre_error_offset = ram_dump.field_offset('struct gsi_chan_stats', 'invalid_tre_error')
        poll_ok_offset = ram_dump.field_offset('struct gsi_chan_stats', 'poll_ok')
        poll_empty_offset = ram_dump.field_offset('struct gsi_chan_stats', 'poll_empty')
        userdata_in_use_offset = ram_dump.field_offset('struct gsi_chan_stats', 'userdata_in_use')
        dp_offset = ram_dump.field_offset('struct gsi_chan_stats', 'dp')

        ch_below_lo_offset = ram_dump.field_offset('struct gsi_chan_dp_stats', 'ch_below_lo')
        ch_below_hi_offset = ram_dump.field_offset('struct gsi_chan_dp_stats', 'ch_below_hi')
        ch_above_hi_offset = ram_dump.field_offset('struct gsi_chan_dp_stats', 'ch_above_hi')
        empty_time_offset = ram_dump.field_offset('struct gsi_chan_dp_stats', 'empty_time')
        last_timestamp_offset = ram_dump.field_offset('struct gsi_chan_dp_stats', 'last_timestamp')

        base_va_offset = ram_dump.field_offset('struct gsi_ring_ctx', 'base_va')
        base_offset = ram_dump.field_offset('struct gsi_ring_ctx', 'base')
        wp_offset = ram_dump.field_offset('struct gsi_ring_ctx', 'wp')
        rp_offset = ram_dump.field_offset('struct gsi_ring_ctx', 'rp')
        wp_local_offset = ram_dump.field_offset('struct gsi_ring_ctx', 'wp_local')
        rp_local_offset = ram_dump.field_offset('struct gsi_ring_ctx', 'rp_local')
        rlen_offset = ram_dump.field_offset('struct gsi_ring_ctx', 'len')

        evtr_state_offset = ram_dump.field_offset('struct gsi_evt_ctx', 'state')
        evtr_id_offset = ram_dump.field_offset('struct gsi_evt_ctx', 'id')
        evtr_ring_offset = ram_dump.field_offset('struct gsi_evt_ctx', 'ring')
        evtr_stats_offset = ram_dump.field_offset('struct gsi_evt_ctx', 'stats')
        evt_completed_offset = ram_dump.field_offset('struct gsi_evt_stats', 'completed')
        IPA_EP_MAX = ram_dump.read_u32(ipa_num_pipes_offset + ipa3_ctx_addr)

        while ep_idx < IPA_EP_MAX:
            ep_addr_data = ep_addr + ram_dump.sizeof('struct ipa3_ep_context') * ep_idx
            valid = ram_dump.read_u32(valid_offset + ep_addr_data)
            if valid != 0:
                curr_chan = ram_dump.read_u64(gsi_chan_hdl_offset + ep_addr_data)
                client = ram_dump.read_u32(client_offset + ep_addr_data)
                ipa_client_enum = self.ramdump.gdbmi.get_value_of('IPA_CLIENT_MHI_QDSS_CONS')
                client_names = ram_dump.gdbmi.get_enum_lookup_table(
                    'ipa_client_type', ipa_client_enum)
                print_out_ip("IPA Pipe:  {0}".format(ep_idx))
                print_out_ip("Pipe Name: {0}".format(client_names[client]))
                sys = ram_dump.read_u64(sys_offset + ep_addr_data)
                if sys and sys != 0x0:
                    len = ram_dump.read_u32(len_offset + sys)
                    counter = ram_dump.read_u32(curr_polling_state_offset + sys)
                    print_out_ip("sys_len:  {0}".format(len))
                    print_out_ip("Current Polling State:  {0}".format(counter))
                    repl = ram_dump.read_u64(repl_offset + sys)
                    page_recycle_repl = ram_dump.read_u64(page_recycle_repl_offset + sys)
                    if repl and repl != 0x0:
                        head_counter = ram_dump.read_u32(head_idx_offset + repl)
                        tail_counter = ram_dump.read_u32(tail_idx_offset + repl)
                        capacity = ram_dump.read_u32(capacity_offset + repl)
                        pending_counter = ram_dump.read_u32(pending_offset + repl)
                        print_out_ip("Repl Cache Head Index: {0}".format(head_counter))
                        print_out_ip("Repl Cache Tail Index:  {0}".format(tail_counter))
                        print_out_ip("Repl Capacity:  {0}".format(capacity))
                        print_out_ip("Repl Wq Pending:  {0}".format(pending_counter))
                    if page_recycle_repl and page_recycle_repl != 0x0:
                        head_counter = ram_dump.read_u32(head_idx_offset + page_recycle_repl)
                        tail_counter = ram_dump.read_u32(tail_idx_offset + page_recycle_repl)
                        capacity = ram_dump.read_u32(capacity_offset + page_recycle_repl)

                        print_out_ip("Page Repl Cache Head Index: {0}".format(head_counter))
                        print_out_ip("Page Repl Cache Tail Index:  {0}".format(tail_counter))
                        print_out_ip("page Repl Capacity:  {0}".format(capacity))
                    else:
                        print_out_ip("sys is null")

                print_out_ip("gsi_chan_hdl:  0x{0}".format(curr_chan))
                gsi_evt_ring_hdl = ram_dump.read_u64(gsi_evt_ring_hdl_offset + ep_addr_data)
                print_out_ip("gsi_evt_ring_hdl:  0x{0}".format(gsi_evt_ring_hdl))
                chan = chan_offset + gsi_ctx_addr
                chan_idx_addr = (chan + ram_dump.sizeof('struct gsi_chan_ctx') * curr_chan)
                pros = props_offset + chan_idx_addr
                prot = ram_dump.read_u32(prot_offset + pros)
                prot_enum = ram_dump.gdbmi.get_value_of('GSI_CHAN_PROT_11AD')
                prot_names = ram_dump.gdbmi.get_enum_lookup_table(
                    'gsi_chan_prot', prot_enum)
                ch_id = ram_dump.read_u16(ch_id_offset + chan_idx_addr)
                state = ram_dump.read_u16(state_offset + chan_idx_addr)
                state_enum = ram_dump.gdbmi.get_value_of('GSI_CHAN_STATE_ERROR')
                state_names = ram_dump.gdbmi.get_enum_lookup_table(
                    'gsi_chan_state', state_enum)

                poll_mode = ram_dump.read_u32(poll_mode_offset + chan_idx_addr)
                ring = ring_offset + chan_idx_addr
                base_va = ram_dump.read_u64(ring + base_va_offset)
                base = ram_dump.read_u64(ring + base_offset)
                wp = ram_dump.read_u64(ring + wp_offset)
                rp = ram_dump.read_u64(ring + rp_offset)
                wp_local = ram_dump.read_u64(ring + wp_local_offset)
                rp_local = ram_dump.read_u64(ring + rp_local_offset)
                rlen = ram_dump.read_u16(ring + rlen_offset)
                stats = stats_offset + chan_idx_addr
                queued = ram_dump.read_u64(stats + queued_offset)
                completed = ram_dump.read_u64(stats + completed_offset)
                callback_to_poll = ram_dump.read_u64(stats + callback_to_poll_offset)
                poll_to_callback = ram_dump.read_u64(stats + poll_to_callback_offset)
                poll_pending_irq = ram_dump.read_u64(stats + poll_pending_irq_offset)
                invalid_tre_error = ram_dump.read_u64(stats + invalid_tre_error_offset)
                poll_ok = ram_dump.read_u64(stats + poll_ok_offset)
                poll_empty = ram_dump.read_u64(stats + poll_empty_offset)
                userdata_in_use = ram_dump.read_u64(stats + userdata_in_use_offset)

                dp = stats + dp_offset
                ch_below_lo = ram_dump.read_u64(dp + ch_below_lo_offset)
                ch_below_hi = ram_dump.read_u64(dp + ch_below_hi_offset)
                ch_above_hi = ram_dump.read_u64(dp + ch_above_hi_offset)
                empty_time = ram_dump.read_u64(dp + empty_time_offset)
                last_timestamp = ram_dump.read_u64(dp + last_timestamp_offset)

                print_out_ip("prot: {0}".format(prot_names[prot]))
                print_out_ip("chan_id: {0}".format(ch_id))
                if state > state_enum:
                    print_out_ip("state: {0}".format(state))
                else:
                    print_out_ip("state: {0}".format(state_names[state]))
                print_out_ip("poll_mode_counter: {0}".format(poll_mode))
                print_out_ip("ring (")
                print_out_ip("	base_va:0x{0:02x}".format(base_va))
                print_out_ip("	base:0x{0:02x}".format(base))
                print_out_ip("	wp:0x{0:02x}".format(wp))
                print_out_ip("	rp:0x{0:02x}".format(rp))
                print_out_ip("	wp_local:0x{0:02x}".format(wp_local))
                print_out_ip("	rp_local:0x{0:02x}".format(rp_local))
                print_out_ip("	ring_len: {0}".format(rlen))
                print_out_ip(")")
                print_out_ip("stats (")
                print_out_ip("	queued: {0}".format(queued))
                print_out_ip("	completed:  {0}".format(completed))
                print_out_ip("	callback_to_poll: {0}".format(callback_to_poll))
                print_out_ip("	poll_to_callback: {0}".format(poll_to_callback))
                print_out_ip("poll_pending_irq: {0}".format(poll_pending_irq))
                print_out_ip("	invalid_tre_error: {0}".format(invalid_tre_error))
                print_out_ip("	poll_ok:  {0}".format(poll_ok))
                print_out_ip("	poll_empty: {0}".format(poll_empty))
                print_out_ip("	userdata_in_use: {0}".format(userdata_in_use))
                print_out_ip("	ep (")
                print_out_ip("		ch_below_lo:  {0}".format(ch_below_lo))
                print_out_ip("		ch_below_hi: {0}".format(ch_below_hi))
                print_out_ip("		ch_above_hi: {0}".format(ch_above_hi))
                print_out_ip("		empty_time: {0}".format(empty_time))
                print_out_ip("		last_timestamp: {0}".format(last_timestamp))
                print_out_ip("	)")
                print_out_ip(")")

                evtr = ram_dump.read_u64(chan_idx_addr + evtr_offset)
                if evtr and evtr != 0x0:
                    evtr_state = ram_dump.read_u16(evtr + evtr_state_offset)
                    evtr_state_enum = self.ramdump.gdbmi.get_value_of('GSI_EVT_RING_STATE_ERROR')
                    evtr_state_names = ram_dump.gdbmi.get_enum_lookup_table(
                    'gsi_evt_ring_state', evtr_state_enum)
                    evtr_id = ram_dump.read_byte(evtr + evtr_id_offset)
                    evtr_ring = evtr + evtr_ring_offset
                    evtr_ring_base_va = ram_dump.read_u64(evtr_ring + base_va_offset)
                    evtr_ring_base = ram_dump.read_u64(evtr_ring + base_offset)
                    evtr_ring_wp = ram_dump.read_u64(evtr_ring + wp_offset)
                    evtr_ring_rp = ram_dump.read_u64(evtr_ring + rp_offset)
                    evtr_ring_wp_local = ram_dump.read_u64(evtr_ring + wp_local_offset)
                    evtr_ring_rp_local = ram_dump.read_u64(evtr_ring + rp_local_offset)
                    evt_len = ram_dump.read_u16(evtr_ring + rlen_offset)

                    evtr_stats = evtr + evtr_stats_offset
                    evtr_stats_completed = ram_dump.read_u64(evtr_stats + evt_completed_offset)
                    print_out_ip("evtr (")
                    print_out_ip("	evtr_state: {0}".format(evtr_state_names[evtr_state]))
                    print_out_ip("	evtr_id: {0}".format(evtr_id))
                    print_out_ip("	evtr_base_va:0x{0:02x}".format(evtr_ring_base_va))
                    print_out_ip("	evtr_base:0x{0:02x}".format(evtr_ring_base))
                    print_out_ip("	evtr_wp:0x{0:02x}".format(evtr_ring_wp))
                    print_out_ip("	evtr_rp:0x{0:02x}".format(evtr_ring_rp))
                    print_out_ip("	evtr_wp_local:0x{0:02x} ".format(evtr_ring_wp_local))
                    print_out_ip("	evtr_rp_local:0x{0:02x} ".format(evtr_ring_rp_local))
                    print_out_ip("	evtr_len: {0}".format(evt_len))
                    print_out_ip("	evtr_completed: {0}".format(evtr_stats_completed))
                    print_out_ip(")")
                else:
                    print_out_ip("evtr ()")

                print_out_ip("  --- ")

            ep_idx = ep_idx + 1
        #HOLB Stats
        uc_ctx_offset = ram_dump.field_offset('struct ipa3_context','uc_ctx')
        uc_inited_offset = ram_dump.field_offset('struct ipa3_uc_ctx','uc_inited')
        uc_loaded_offset = ram_dump.field_offset('struct ipa3_uc_ctx','uc_loaded')
        uc_failed_offset = ram_dump.field_offset('struct ipa3_uc_ctx','uc_failed')
        holb_monitor_offset = ram_dump.field_offset('struct ipa3_uc_ctx','holb_monitor')
        num_holb_clients_offset = ram_dump.field_offset('struct ipa_holb_monitor','num_holb_clients')
        poll_period_offset = ram_dump.field_offset('struct ipa_holb_monitor','poll_period')
        max_cnt_wlan_offset = ram_dump.field_offset('struct ipa_holb_monitor','max_cnt_wlan')
        max_cnt_usb_offset = ram_dump.field_offset('struct ipa_holb_monitor','max_cnt_usb')
        max_cnt_11ad_offset = ram_dump.field_offset('struct ipa_holb_monitor','max_cnt_11ad')

        uc_ctx = ipa3_ctx_addr + uc_ctx_offset
        holb_monitor = holb_monitor_offset + uc_ctx
        num_holb_clients = ram_dump.read_u32(holb_monitor + num_holb_clients_offset)
        max_cnt_wlan = ram_dump.read_u32(holb_monitor + max_cnt_wlan_offset)
        max_cnt_usb = ram_dump.read_u32(holb_monitor + max_cnt_usb_offset)
        max_cnt_11ad = ram_dump.read_u32(holb_monitor + max_cnt_11ad_offset)

        hlob_monitor_client_offset = ram_dump.field_offset('struct ipa_holb_monitor','client')
        gsi_chan_hdl_offset  = ram_dump.field_offset('struct ipa_uc_holb_client_info','gsi_chan_hdl')
        action_mask_offset  = ram_dump.field_offset('struct ipa_uc_holb_client_info','action_mask')
        max_stuck_cnt_offset  = ram_dump.field_offset('struct ipa_uc_holb_client_info','max_stuck_cnt')
        ee_offset  = ram_dump.field_offset('struct ipa_uc_holb_client_info','ee')
        debugfs_param_offset  = ram_dump.field_offset('struct ipa_uc_holb_client_info','debugfs_param')
        state_offset  = ram_dump.field_offset('struct ipa_uc_holb_client_info','state')
        enable_cnt_offset  = ram_dump.field_offset('struct ipa_uc_holb_client_info','enable_cnt')
        disable_cnt_offset  = ram_dump.field_offset('struct ipa_uc_holb_client_info','disable_cnt')
        current_idx_offset  = ram_dump.field_offset('struct ipa_uc_holb_client_info','current_idx')
        events_offset  = ram_dump.field_offset('struct ipa_uc_holb_client_info','events')

        hlob_monitor_client_addr = holb_monitor + hlob_monitor_client_offset
        hold_idx=0
        print_out_ip("HOLB Clients Info:")
        while hold_idx < num_holb_clients:
            print_out_ip("")
            hlob_monitor_client = hlob_monitor_client_addr + ram_dump.sizeof('struct ipa_uc_holb_client_info') * hold_idx
            print_out_ip("HOLB Client index: {0}".format(hold_idx))
            gsi_chan_hdl = ram_dump.read_u16(hlob_monitor_client + gsi_chan_hdl_offset)
            action_mask = ram_dump.read_u32(hlob_monitor_client + action_mask_offset)
            max_stuck_cnt = ram_dump.read_u32(hlob_monitor_client + max_stuck_cnt_offset)
            ee = ram_dump.read_byte(hlob_monitor_client + ee_offset)
            debugfs_param = ram_dump.read_bool(hlob_monitor_client + debugfs_param_offset)
            state = ram_dump.read_u64(hlob_monitor_client + state_offset)
            enable_cnt = ram_dump.read_u64(hlob_monitor_client + enable_cnt_offset)
            disable_cnt = ram_dump.read_u64(hlob_monitor_client + disable_cnt_offset)
            current_idx = ram_dump.read_u64(hlob_monitor_client + current_idx_offset)
            events = ram_dump.read_u64(hlob_monitor_client + events_offset)

            print_out_ip("HOLB GSI Channel Handle: {0}".format(gsi_chan_hdl))
            print_out_ip("HOLB Action Mask: {0}".format(action_mask))
            print_out_ip("HOLB MAX Stuck Count: {0}".format(max_stuck_cnt))
            print_out_ip("HOLB EE: {0}".format(ee))
            print_out_ip("HOLB enabled via debugfs: {0}".format(debugfs_param))
            print_out_ip("HOLB State: {0}".format(state))
            print_out_ip("HOLB enable cnt: {0}".format(enable_cnt))
            print_out_ip("HOLB disable cnt: {0}".format(disable_cnt))
            print_out_ip("HOLB Events current idx: {0}".format(current_idx))
            print_out_ip("")
            print_out_ip("Events {0}".format(events))
            print_out_ip("")
            hold_idx=hold_idx+1

    def parse(self):
        setup_out_file(ipa_log, self)
        self.ipa_parse(self.ramdump)
