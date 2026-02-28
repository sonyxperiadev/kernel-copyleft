# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2022,2025 Qualcomm Innovation Center, Inc. All rights reserved.

from parser_util import register_parser, RamParser, cleanupString
from print_out import print_out_str, print_out_exception

def extract_va_minidump(ramdump):
    va_minidump_map = {
        'msm_drm_priv'  : 'struct msm_drm_private',
        'sde_evtlog'    : 'struct sde_dbg_evtlog',
        'sde_reglog'    : 'struct sde_dbg_reglog',
        'sde_kms'       : 'struct sde_kms',
        'ipa3_ctx'      : 'struct ipa3_context',
        'ipa_pm_ctx'    : 'struct ipa_pm_ctx',
    }

    kva_out = ramdump.open_file('va_minidump_output.txt')
    for sym, type in va_minidump_map.items():
        try:
            kva_out.write("\n************** {} **************\n".format(sym))
            addr = ramdump.get_section_address(sym)[0][0]
            obj = ramdump.read_datatype(addr, type)
            ramdump.print_struct(obj, kva_out, format='hex')
        except:
            print_out_str("\n************** {} **************\n".format(sym))
            print_out_exception()
            pass
    kva_out.close()

@register_parser('--va-minidump', 'Print all data structures saved in va minidump')
class VaMinidump(RamParser):
    def parse(self):
        extract_va_minidump(self.ramdump)

