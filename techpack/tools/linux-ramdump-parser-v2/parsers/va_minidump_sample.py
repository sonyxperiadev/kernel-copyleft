# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.

from parser_util import register_parser, RamParser, cleanupString

def extract_va_minidump(ramdump):
    try:
        kva_out = ramdump.open_file('kva_output.txt')

        kva_out.write("\n************** msm_drm_priv **************\n")

        msm_drm_priv_addr = ramdump.get_section_address("msm_drm_priv")[0][0]
        msm_drm_priv = ramdump.read_datatype(msm_drm_priv_addr,
                                              "struct msm_drm_private")
        ramdump.print_struct(msm_drm_priv, kva_out)

        kva_out.write("\n************** sde_evtlog **************\n")
        sde_evtlog_addr = ramdump.get_section_address("sde_evtlog")[0][0]
        sde_evtlog = ramdump.read_datatype(sde_evtlog_addr,
                                            "sde_dbg_base_evtlog")
        kva_out.write(str(sde_evtlog))

        kva_out.write("\n************** sde_reglog **************\n")
        sde_reglog_addr = ramdump.get_section_address("sde_reglog")[0][0]
        sde_reglog = ramdump.read_datatype(sde_reglog_addr,
                                            "struct sde_dbg_reglog")
        ramdump.print_struct(sde_reglog, kva_out)

        kva_out.write("\n************** sde_kms **************\n")
        sde_kms_addr = ramdump.get_section_address("sde_kms")[0][0]
        sde_kms = ramdump.read_datatype(sde_kms_addr, "struct sde_kms")
        ramdump.print_struct(sde_kms, kva_out)
        kva_out.close()

    except Exception as err:
        traceback.print_exc()

@register_parser('--vm-minidump-sample', 'Print reserved memory info ')
class VaMinidump(RamParser):

    def parse(self):
        extract_va_minidump(self.ramdump)
