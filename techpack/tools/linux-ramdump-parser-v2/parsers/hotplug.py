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

import os,sys
import struct

from print_out import print_out_str
from parser_util import register_parser, RamParser


ipa_log = "hotplug.txt"


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


@register_parser('--hotplug', 'print hotplug notifier information')
class hp_logging(RamParser):

    def __init__(self, *args):
        super(hp_logging, self).__init__(*args)

    def hp_parse(self, ram_dump):
        curr_chan=0
        ep_idx=0
        cpuhp_hp_states_addr = ram_dump.address_of('cpuhp_hp_states')
        if cpuhp_hp_states_addr:
            idx = 0
            cpuhp_sz = ram_dump.sizeof('struct cpuhp_step')
            hp_states_sz = ram_dump.sizeof('cpuhp_hp_states')
            max_idx = hp_states_sz/cpuhp_sz
            name_offset = ram_dump.field_offset('struct cpuhp_step','name')
            startup_soffset = ram_dump.field_offset('struct cpuhp_step','startup')
            teardown_soffset = ram_dump.field_offset('struct cpuhp_step','teardown')
            can_stop_offset = ram_dump.field_offset('struct cpuhp_step','cant_stop')
            multi_instance_offset = ram_dump.field_offset('struct cpuhp_step','multi_instance')
            while (idx < max_idx):
                name_off = ram_dump.read_u64(cpuhp_hp_states_addr + name_offset)
                name = ram_dump.read_cstring(name_off)
                startup_soffset_addr = ram_dump.read_u64(cpuhp_hp_states_addr + startup_soffset)
                if startup_soffset_addr != 0:
                    startup_soffset_addr = ram_dump.get_symbol_info1(startup_soffset_addr)
                teardown_soffset_addr = ram_dump.read_u64(cpuhp_hp_states_addr + teardown_soffset)
                if teardown_soffset_addr != 0:
                    teardown_soffset_addr = ram_dump.get_symbol_info1(teardown_soffset_addr)
                can_stop = ram_dump.read_bool(cpuhp_hp_states_addr + can_stop_offset)
                multi_instance = ram_dump.read_bool(cpuhp_hp_states_addr + multi_instance_offset)
                if (name_off == 0 or name_off is None):
                    idx = idx + 1
                    cpuhp_hp_states_addr = cpuhp_hp_states_addr + cpuhp_sz
                    continue
                temp = name + "  startup = (single = {0} = , multi = {1} = ),teardown = ( single = {2}  = , multi = {3} = ),cant_stop = {4}, multi_instance = {5}".format(startup_soffset_addr,startup_soffset_addr,teardown_soffset_addr,teardown_soffset_addr,can_stop,multi_instance)
                print_out_ip(temp)
                idx = idx + 1
                cpuhp_hp_states_addr = cpuhp_hp_states_addr + cpuhp_sz


    def parse(self):
        setup_out_file(ipa_log, self)
        self.hp_parse(self.ramdump)
