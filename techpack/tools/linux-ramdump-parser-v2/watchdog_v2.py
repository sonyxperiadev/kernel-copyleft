# Copyright (c) 2012-2021, The Linux Foundation. All rights reserved.
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

import struct
import re

from print_out import print_out_str
from bitops import is_set
from parser_util import register_parser, RamParser
from sched_info import cpu_isolation_mask

# name from tz dump, corresponding T32 register, whether or not to
# print_out_str (the function name)
sysdbg_cpu64_register_names_default = [
    ('x0', 'x0', False),
    ('x1', 'x1', False),
    ('x2', 'x2', False),
    ('x3', 'x3', False),
    ('x4', 'x4', False),
    ('x5', 'x5', False),
    ('x6', 'x6', False),
    ('x7', 'x7', False),
    ('x8', 'x8', False),
    ('x9', 'x9', False),
    ('x10', 'x10', False),
    ('x11', 'x11', False),
    ('x12', 'x12', False),
    ('x13', 'x13', False),
    ('x14', 'x14', False),
    ('x15', 'x15', False),
    ('x16', 'x16', False),
    ('x17', 'x17', False),
    ('x18', 'x18', False),
    ('x19', 'x19', False),
    ('x20', 'x20', False),
    ('x21', 'x21', False),
    ('x22', 'x22', False),
    ('x23', 'x23', False),
    ('x24', 'x24', False),
    ('x25', 'x25', False),
    ('x26', 'x26', False),
    ('x27', 'x27', False),
    ('x28', 'x28', False),
    ('x29', 'x29', False),
    ('x30', 'x30', True),
    ('pc', 'pc', True),
    ('currentEL', None, False),
    ('sp_el3', 'sp_el3', False),
    ('elr_el3', 'elr_el3', True),
    ('spsr_el3', 'spsr_el3', False),
    ('sp_el2', 'sp_el2', False),
    ('elr_el2', 'elr_el2', True),
    ('spsr_el2', 'spsr_el2', False),
    ('sp_el1', 'sp_el1', False),
    ('elr_el1', 'elr_el1', True),
    ('spsr_el1', 'spsr_el1', False),
    ('sp_el0', 'sp_el0', False),
    ('__reserved1', '__reserved1', False),
    ('__reserved2', '__reserved2', False),
    ('__reserved3', '__reserved3', False),
    ('__reserved4', '__reserved4', False),
]

sysdbg_cpu64_ctxt_regs_type_default = ''.join([
    'Q',  # x0
    'Q',  # x1
    'Q',  # x2
    'Q',  # x3
    'Q',  # x4
    'Q',  # x5
    'Q',  # x6
    'Q',  # x7
    'Q',  # x8
    'Q',  # x9
    'Q',  # x10
    'Q',  # x11
    'Q',  # x12
    'Q',  # x13
    'Q',  # x14
    'Q',  # x15
    'Q',  # x16
    'Q',  # x17
    'Q',  # x18
    'Q',  # x19
    'Q',  # x20
    'Q',  # x21
    'Q',  # x22
    'Q',  # x23
    'Q',  # x24
    'Q',  # x25
    'Q',  # x26
    'Q',  # x27
    'Q',  # x28
    'Q',  # x29
    'Q',  # x30
    'Q',  # pc
    'Q',  # currentEL
    'Q',  # sp_el3
    'Q',  # elr_el3
    'Q',  # spsr_el3
    'Q',  # sp_el2
    'Q',  # elr_el2
    'Q',  # spsr_el2
    'Q',  # sp_el1
    'Q',  # elr_el1
    'Q',  # spsr_el1
    'Q',  # sp_el0
    'Q',  # __reserved1
    'Q',  # __reserved2
    'Q',  # __reserved3
    'Q',  # __reserved4
])

sysdbg_cpu32_register_names_default = [
    ('r0', 'r0', False),
    ('r1', 'r1', False),
    ('r2', 'r2', False),
    ('r3', 'r3', False),
    ('r4', 'r4', False),
    ('r5', 'r5', False),
    ('r6', 'r6', False),
    ('r7', 'r7', False),
    ('r8', 'r8', False),
    ('r9', 'r9', False),
    ('r10', 'r10', False),
    ('r11', 'r11', False),
    ('r12', 'r12', False),
    ('r13_usr', 'r13_usr', False),
    ('r14_usr', 'r14_usr', False),
    ('r13_hyp', 'r13_hyp', False),
    ('r14_irq', 'r14_irq', True),
    ('r13_irq', 'r13_irq', False),
    ('r14_svc', 'r14_svc', True),
    ('r13_svc', 'r13_svc', False),
    ('r14_abt', 'r14_abt', True),
    ('r13_abt', 'r13_abt', False),
    ('r14_und', 'r14_und', True),
    ('r13_und', 'r13_und', False),
    ('r8_fiq', 'r8_fiq', False),
    ('r9_fiq', 'r9_fiq', False),
    ('r10_fiq', 'r10_fiq', False),
    ('r11_fiq', 'r11_fiq', False),
    ('r12_fiq', 'r12_fiq', False),
    ('r13_fiq', 'r13_fiq', False),
    ('r14_fiq', 'r14_fiq', True),
    ('pc', 'pc', True),
    ('cpsr', 'cpsr', False),
    ('r13_mon', 'r13_mon', False),
    ('r14_mon', 'r14_mon', True),
    ('r14_hyp', 'elr_hyp', True),
    ('_reserved', '_reserved', False),
    ('__reserved1', '__reserved1', False),
    ('__reserved2', '__reserved2', False),
    ('__reserved3', '__reserved3', False),
    ('__reserved4', '__reserved4', False),
]

sysdbg_cpu32_ctxt_regs_type_default = ''.join([
    'Q',  # r0
    'Q',  # r1
    'Q',  # r2
    'Q',  # r3
    'Q',  # r4
    'Q',  # r5
    'Q',  # r6
    'Q',  # r7
    'Q',  # r8
    'Q',  # r9
    'Q',  # r10
    'Q',  # r11
    'Q',  # r12
    'Q',  # r13_usr
    'Q',  # r14_usr
    'Q',  # r13_hyp
    'Q',  # r14_irq
    'Q',  # r13_irq
    'Q',  # r14_svc
    'Q',  # r13_svc
    'Q',  # r14_abt
    'Q',  # r13_abt
    'Q',  # r14_und
    'Q',  # r13_und
    'Q',  # r8_fiq
    'Q',  # r9_fiq
    'Q',  # r10_fiq
    'Q',  # r11_fiq
    'Q',  # r12_fiq
    'Q',  # r13_fiq
    'Q',  # r14_fiq
    'Q',  # pc
    'Q',  # cpsr
    'Q',  # r13_mon
    'Q',  # r14_mon
    'Q',  # r14_hyp
    'Q',  # _reserved
    'Q',  # __reserved1
    'Q',  # __reserved2
    'Q',  # __reserved3
    'Q',  # __reserved4
])

sysdbg_cpu64_register_names_v1_3 = [
    ('x0', 'x0', False),
    ('x1', 'x1', False),
    ('x2', 'x2', False),
    ('x3', 'x3', False),
    ('x4', 'x4', False),
    ('x5', 'x5', False),
    ('x6', 'x6', False),
    ('x7', 'x7', False),
    ('x8', 'x8', False),
    ('x9', 'x9', False),
    ('x10', 'x10', False),
    ('x11', 'x11', False),
    ('x12', 'x12', False),
    ('x13', 'x13', False),
    ('x14', 'x14', False),
    ('x15', 'x15', False),
    ('x16', 'x16', False),
    ('x17', 'x17', False),
    ('x18', 'x18', False),
    ('x19', 'x19', False),
    ('x20', 'x20', False),
    ('x21', 'x21', False),
    ('x22', 'x22', False),
    ('x23', 'x23', False),
    ('x24', 'x24', False),
    ('x25', 'x25', False),
    ('x26', 'x26', False),
    ('x27', 'x27', False),
    ('x28', 'x28', False),
    ('x29', 'x29', False),
    ('x30', 'x30', True),
    ('pc', 'pc', True),
    ('currentEL', None, False),
    ('sp_el3', 'sp_el3', False),
    ('elr_el3', 'elr_el3', True),
    ('spsr_el3', 'spsr_el3', False),
    ('sp_el2', 'sp_el2', False),
    ('elr_el2', 'elr_el2', True),
    ('spsr_el2', 'spsr_el2', False),
    ('sp_el1', 'sp_el1', False),
    ('elr_el1', 'elr_el1', True),
    ('spsr_el1', 'spsr_el1', False),
    ('sp_el0', 'sp_el0', False),
    ('cpumerrsr_el1', None, False),
    ('l2merrsr_el1',  None, False),
    ('__reserved1', '__reserved1', False),
    ('__reserved2', '__reserved2', False),
]

sysdbg_cpu64_ctxt_regs_type_v1_3 = ''.join([
    'Q',  # x0
    'Q',  # x1
    'Q',  # x2
    'Q',  # x3
    'Q',  # x4
    'Q',  # x5
    'Q',  # x6
    'Q',  # x7
    'Q',  # x8
    'Q',  # x9
    'Q',  # x10
    'Q',  # x11
    'Q',  # x12
    'Q',  # x13
    'Q',  # x14
    'Q',  # x15
    'Q',  # x16
    'Q',  # x17
    'Q',  # x18
    'Q',  # x19
    'Q',  # x20
    'Q',  # x21
    'Q',  # x22
    'Q',  # x23
    'Q',  # x24
    'Q',  # x25
    'Q',  # x26
    'Q',  # x27
    'Q',  # x28
    'Q',  # x29
    'Q',  # x30
    'Q',  # pc
    'Q',  # currentEL
    'Q',  # sp_el3
    'Q',  # elr_el3
    'Q',  # spsr_el3
    'Q',  # sp_el2
    'Q',  # elr_el2
    'Q',  # spsr_el2
    'Q',  # sp_el1
    'Q',  # elr_el1
    'Q',  # spsr_el1
    'Q',  # sp_el0
    'Q',  # cpumerrsr_el1
    'Q',  # l2merrsr_el1
    'Q',  # __reserved1
    'Q',  # __reserved2
])

sysdbg_cpu64_register_names_v1_4 = [
    ('x0', 'x0', False),
    ('x1', 'x1', False),
    ('x2', 'x2', False),
    ('x3', 'x3', False),
    ('x4', 'x4', False),
    ('x5', 'x5', False),
    ('x6', 'x6', False),
    ('x7', 'x7', False),
    ('x8', 'x8', False),
    ('x9', 'x9', False),
    ('x10', 'x10', False),
    ('x11', 'x11', False),
    ('x12', 'x12', False),
    ('x13', 'x13', False),
    ('x14', 'x14', False),
    ('x15', 'x15', False),
    ('x16', 'x16', False),
    ('x17', 'x17', False),
    ('x18', 'x18', False),
    ('x19', 'x19', False),
    ('x20', 'x20', False),
    ('x21', 'x21', False),
    ('x22', 'x22', False),
    ('x23', 'x23', False),
    ('x24', 'x24', False),
    ('x25', 'x25', False),
    ('x26', 'x26', False),
    ('x27', 'x27', False),
    ('x28', 'x28', False),
    ('x29', 'x29', False),
    ('x30', 'x30', True),
    ('pc', 'pc', True),
    ('currentEL', None, False),
    ('sp_el3', 'sp_el3', False),
    ('elr_el3', 'elr_el3', True),
    ('spsr_el3', 'spsr_el3', False),
    ('sp_el2', 'sp_el2', False),
    ('elr_el2', 'elr_el2', True),
    ('spsr_el2', 'spsr_el2', False),
    ('sp_el1', 'sp_el1', False),
    ('elr_el1', 'elr_el1', True),
    ('spsr_el1', 'spsr_el1', False),
    ('sp_el0', 'sp_el0', False),
    ('cpu_state_0', 'cpu_state_0', False),
    ('cpu_state_1', 'cpu_state_1', False),
    ('cpu_state_3', 'cpu_state_3', False),
    ('cpu_state_4', 'cpu_state_4', False),
    ('cpu_state_5', 'cpu_state_5', False),
    ('__reserved1', '__reserved1', False),
    ('__reserved2', '__reserved2', False),
    ('__reserved3', '__reserved3', False),
]

sysdbg_cpu64_ctxt_regs_type_v1_4 = ''.join([
    'Q',  # x0
    'Q',  # x1
    'Q',  # x2
    'Q',  # x3
    'Q',  # x4
    'Q',  # x5
    'Q',  # x6
    'Q',  # x7
    'Q',  # x8
    'Q',  # x9
    'Q',  # x10
    'Q',  # x11
    'Q',  # x12
    'Q',  # x13
    'Q',  # x14
    'Q',  # x15
    'Q',  # x16
    'Q',  # x17
    'Q',  # x18
    'Q',  # x19
    'Q',  # x20
    'Q',  # x21
    'Q',  # x22
    'Q',  # x23
    'Q',  # x24
    'Q',  # x25
    'Q',  # x26
    'Q',  # x27
    'Q',  # x28
    'Q',  # x29
    'Q',  # x30
    'Q',  # pc
    'Q',  # currentEL
    'Q',  # sp_el3
    'Q',  # elr_el3
    'Q',  # spsr_el3
    'Q',  # sp_el2
    'Q',  # elr_el2
    'Q',  # spsr_el2
    'Q',  # sp_el1
    'Q',  # elr_el1
    'Q',  # spsr_el1
    'Q',  # sp_el0
    'Q',  # cpu_state_0
    'Q',  # cpu_state_1
    'Q',  # cpu_state_3
    'Q',  # cpu_state_4
    'Q',  # cpu_state_5
    'Q',  # __reserved1
    'Q',  # __reserved2
    'Q',  # __reserved3
])



sysdbg_cpu64_gprs_register_names_v2_0 = [
    ('x0', 'x0', False),
    ('x1', 'x1', False),
    ('x2', 'x2', False),
    ('x3', 'x3', False),
    ('x4', 'x4', False),
    ('x5', 'x5', False),
    ('x6', 'x6', False),
    ('x7', 'x7', False),
    ('x8', 'x8', False),
    ('x9', 'x9', False),
    ('x10', 'x10', False),
    ('x11', 'x11', False),
    ('x12', 'x12', False),
    ('x13', 'x13', False),
    ('x14', 'x14', False),
    ('x15', 'x15', False),
    ('x16', 'x16', False),
    ('x17', 'x17', False),
    ('x18', 'x18', False),
    ('x19', 'x19', False),
    ('x20', 'x20', False),
    ('x21', 'x21', False),
    ('x22', 'x22', False),
    ('x23', 'x23', False),
    ('x24', 'x24', False),
    ('x25', 'x25', False),
    ('x26', 'x26', False),
    ('x27', 'x27', False),
    ('x28', 'x28', False),
    ('x29', 'x29', False),
    ('x30', 'x30', True),
    ('pc', 'pc', True),
    ('sp_el0', 'sp_el0', False),
    ('pstate', 'pstate', False),    
]

sysdbg_cpu64_gprs_ctxt_regs_type_v2_0 = ''.join([
    'Q',  # x0
    'Q',  # x1
    'Q',  # x2
    'Q',  # x3
    'Q',  # x4
    'Q',  # x5
    'Q',  # x6
    'Q',  # x7
    'Q',  # x8
    'Q',  # x9
    'Q',  # x10
    'Q',  # x11
    'Q',  # x12
    'Q',  # x13
    'Q',  # x14
    'Q',  # x15
    'Q',  # x16
    'Q',  # x17
    'Q',  # x18
    'Q',  # x19
    'Q',  # x20
    'Q',  # x21
    'Q',  # x22
    'Q',  # x23
    'Q',  # x24
    'Q',  # x25
    'Q',  # x26
    'Q',  # x27
    'Q',  # x28
    'Q',  # x29
    'Q',  # x30
    'Q',  # pc
    'Q',  # sp_el0
    'Q',  # pstate
])

sysdbg_cpu64_el1_register_names_v2_0 = [
    ('cpacr_el1', 'cpacr_el1', False),
    ('csselr_el1', 'csselr_el1', False),
    ('elr_el1', 'elr_el1', False),
    ('esr_el1', 'esr_el1', False),
    ('far_el1', 'far_el1', False),
    ('isr_el1', 'isr_el1', False),
    ('par_el1', 'par_el1', False),
    ('mair_el1', 'mair_el1', False),
    ('sctlr_el1', 'sctlr_el1', False),
    ('sp_el1', 'sp_el1', False),
    ('spsr_el1', 'spsr_el1', False),
    ('tcr_el1', 'tcr_el1', False),
    ('tpidr_el1', 'tpidr_el1', False),
    ('ttbr0_el1', 'ttbr0_el1', False),
    ('ttbr1_el1', 'ttbr1_el1', False),
    ('vbar_el1', 'vbar_el1', False),
        
]

sysdbg_cpu64_el1_ctxt_regs_type_v2_0 = ''.join([
    'Q',  # x0
    'Q',  # x1
    'Q',  # x2
    'Q',  # x3
    'Q',  # x4
    'Q',  # x5
    'Q',  # x6
    'Q',  # x7
    'Q',  # x8
    'Q',  # x9
    'Q',  # x10
    'Q',  # x11
    'Q',  # x12
    'Q',  # x13
    'Q',  # x14
    'Q',  # x15    
])

sysdbg_cpu64_el0_register_names_v2_0 = [
    ('tpidr_el0', 'tpidr_el0', False),
    ('tpidrro_el0', 'tpidrro_el0', False),        
]

sysdbg_cpu64_el0_ctxt_regs_type_v2_0 = ''.join([
    'Q',  # x0
    'Q',  # x1    
])

sysdbg_cpu64_vm_el2_register_names_v2_0 = [
    ('cptr_el2', 'cptr_el2', False),
    ('hcr_el2', 'hcr_el2', False),
    ('mdcr_el2', 'mdcr_el2', False),
    ('vtcr_el2', 'vtcr_el2', False),
    ('vttbr_el2', 'vttbr_el2', False),
    
]

sysdbg_cpu64_vm_el2_ctxt_regs_type_v2_0 = ''.join([
    'Q',
    'Q',
    'Q',
    'Q',
    'Q',    
])

sysdbg_cpu64_cntv_el10_register_names_v2_0 = [
    ('cntkctl_el1', 'cntkctl_el1', False),
    ('cntv_ctl_el0', 'cntv_ctl_el0', False),
    ('cntv_cval_el0', 'cntv_cval_el0', False),
    ('cntv_tval_el0', 'cntv_tval_el0', False),
]

sysdbg_cpu64_cntv_el10_ctxt_regs_type_v2_0 = ''.join([
    'Q',
    'Q',
    'Q',
    'Q',    
])

sysdbg_cpu64_cntp_el10_register_names_v2_0 = [
    ('cntp_ctl_el0', 'cntp_ctl_el0', False),
    ('cntp_cval_el0', 'cntp_cval_el0', False),
    ('cntp_tval_el0', 'cntp_tval_el0', False),
]

sysdbg_cpu64_cntp_el10_ctxt_regs_type_v2_0 = ''.join([
    'Q',
    'Q',
    'Q',
])

sysdbg_cpu64_cnt_el2_register_names_v2_0 = [
    ('cnthctl_el2', 'cnthctl_el2', False),
    ('cnthp_ctl_el2', 'cnthp_ctl_el2', False),
    ('cnthp_cval_el2', 'cnthp_cval_el2', False),
    ('cnthp_tval_el2', 'cnthp_tval_el2', False),
]

sysdbg_cpu64_cnt_el2_ctxt_regs_type_v2_0 = ''.join([
    'Q',
    'Q',
    'Q',
    'Q',
])


sysdbg_neon128_register_names_v1_4 = [
    ('q0-lower', 'v0-lower', False),
    ('q0-upper', 'v0-upper', False),
    ('q1-lower', 'v1-lower', False),
    ('q1-upper', 'v1-upper', False),
    ('q2-lower', 'v2-lower', False),
    ('q2-upper', 'v2-upper', False),
    ('q3-lower', 'v3-lower', False),
    ('q3-upper', 'v3-upper', False),
    ('q4-lower', 'v4-lower', False),
    ('q4-upper', 'v4-upper', False),
    ('q5-lower', 'v5-lower', False),
    ('q5-upper', 'v5-upper', False),
    ('q6-lower', 'v6-lower', False),
    ('q6-upper', 'v6-upper', False),
    ('q7-lower', 'v7-lower', False),
    ('q7-upper', 'v7-upper', False),
    ('q8-lower', 'v8-lower', False),
    ('q8-upper', 'v8-upper', False),
    ('q9-lower', 'v9-lower', False),
    ('q9-upper', 'v9-upper', False),
    ('q10-lower', 'v10-lower', False),
    ('q10-upper', 'v10-upper', False),
    ('q11-lower', 'v11-lower', False),
    ('q11-upper', 'v11-upper', False),
    ('q12-lower', 'v12-lower', False),
    ('q12-upper', 'v12-upper', False),
    ('q13-lower', 'v13-lower', False),
    ('q13-upper', 'v13-upper', False),
    ('q14-lower', 'v14-lower', False),
    ('q14-upper', 'v14-upper', False),
    ('q15-lower', 'v15-lower', False),
    ('q15-upper', 'v15-upper', False),
    ('q16-lower', 'v16-lower', False),
    ('q16-upper', 'v16-upper', False),
    ('q17-lower', 'v17-lower', False),
    ('q17-upper', 'v17-upper', False),
    ('q18-lower', 'v18-lower', False),
    ('q18-upper', 'v18-upper', False),
    ('q19-lower', 'v19-lower', False),
    ('q19-upper', 'v19-upper', False),
    ('q20-lower', 'v20-lower', False),
    ('q20-upper', 'v20-upper', False),
    ('q21-lower', 'v21-lower', False),
    ('q21-upper', 'v21-upper', False),
    ('q22-lower', 'v22-lower', False),
    ('q22-upper', 'v22-upper', False),
    ('q23-lower', 'v23-lower', False),
    ('q23-upper', 'v23-upper', False),
    ('q24-lower', 'v24-lower', False),
    ('q24-upper', 'v24-upper', False),
    ('q25-lower', 'v25-lower', False),
    ('q25-upper', 'v25-upper', False),
    ('q26-lower', 'v26-lower', False),
    ('q26-upper', 'v26-upper', False),
    ('q27-lower', 'v27-lower', False),
    ('q27-upper', 'v27-upper', False),
    ('q28-lower', 'v28-lower', False),
    ('q28-upper', 'v28-upper', False),
    ('q29-lower', 'v29-lower', False),
    ('q29-upper', 'v29-upper', False),
    ('q30-lower', 'v30-lower', False),
    ('q30-upper', 'v30-upper', False),
    ('q31-lower', 'v31-lower', False),
    ('q31-upper', 'v31-upper', False),
]

sysdbg_neon128_register_type_v1_4 = ''.join([
    'Q',  # q0-lower
    'Q',  # q0-upper
    'Q',  # q1-lower
    'Q',  # q1-upper
    'Q',  # q2-lower
    'Q',  # q2-upper
    'Q',  # q3-lower
    'Q',  # q3-upper
    'Q',  # q4-lower
    'Q',  # q4-upper
    'Q',  # q5-lower
    'Q',  # q5-upper
    'Q',  # q6-lower
    'Q',  # q6-upper
    'Q',  # q7-lower
    'Q',  # q7-upper
    'Q',  # q8-lower
    'Q',  # q8-upper
    'Q',  # q9-lower
    'Q',  # q9-upper
    'Q',  # q10-lower
    'Q',  # q10-upper
    'Q',  # q11-lower
    'Q',  # q11-upper
    'Q',  # q12-lower
    'Q',  # q12-upper
    'Q',  # q13-lower
    'Q',  # q13-upper
    'Q',  # q14-lower
    'Q',  # q14-upper
    'Q',  # q15-lower
    'Q',  # q15-upper
    'Q',  # q16-lower
    'Q',  # q16-upper
    'Q',  # q17-lower
    'Q',  # q17-upper
    'Q',  # q18-lower
    'Q',  # q18-upper
    'Q',  # q19-lower
    'Q',  # q19-upper
    'Q',  # q20-lower
    'Q',  # q20-upper
    'Q',  # q21-lower
    'Q',  # q21-upper
    'Q',  # q22-lower
    'Q',  # q22-upper
    'Q',  # q23-lower
    'Q',  # q23-upper
    'Q',  # q24-lower
    'Q',  # q24-upper
    'Q',  # q25-lower
    'Q',  # q25-upper
    'Q',  # q26-lower
    'Q',  # q26-upper
    'Q',  # q27-lower
    'Q',  # q27-upper
    'Q',  # q28-lower
    'Q',  # q28-upper
    'Q',  # q29-lower
    'Q',  # q29-upper
    'Q',  # q30-lower
    'Q',  # q30-upper
    'Q',  # q31-lower
    'Q',  # q31-upper
])



sysdbg_neon128_register_names_v2_0 = [
    ('v0-lower', 'v0-lower', False),
    ('v0-upper', 'v0-upper', False),
    ('v1-lower', 'v1-lower', False),
    ('v1-upper', 'v1-upper', False),
    ('v2-lower', 'v2-lower', False),
    ('v2-upper', 'v2-upper', False),
    ('v3-lower', 'v3-lower', False),
    ('v3-upper', 'v3-upper', False),
    ('v4-lower', 'v4-lower', False),
    ('v4-upper', 'v4-upper', False),
    ('v5-lower', 'v5-lower', False),
    ('v5-upper', 'v5-upper', False),
    ('v6-lower', 'v6-lower', False),
    ('v6-upper', 'v6-upper', False),
    ('v7-lower', 'v7-lower', False),
    ('v7-upper', 'v7-upper', False),
    ('v8-lower', 'v8-lower', False),
    ('v8-upper', 'v8-upper', False),
    ('v9-lower', 'v9-lower', False),
    ('v9-upper', 'v9-upper', False),
    ('v10-lower', 'v10-lower', False),
    ('v10-upper', 'v10-upper', False),
    ('v11-lower', 'v11-lower', False),
    ('v11-upper', 'v11-upper', False),
    ('v12-lower', 'v12-lower', False),
    ('v12-upper', 'v12-upper', False),
    ('v13-lower', 'v13-lower', False),
    ('v13-upper', 'v13-upper', False),
    ('v14-lower', 'v14-lower', False),
    ('v14-upper', 'v14-upper', False),
    ('v15-lower', 'v15-lower', False),
    ('v15-upper', 'v15-upper', False),
    ('v16-lower', 'v16-lower', False),
    ('v16-upper', 'v16-upper', False),
    ('v17-lower', 'v17-lower', False),
    ('v17-upper', 'v17-upper', False),
    ('v18-lower', 'v18-lower', False),
    ('v18-upper', 'v18-upper', False),
    ('v19-lower', 'v19-lower', False),
    ('v19-upper', 'v19-upper', False),
    ('v20-lower', 'v20-lower', False),
    ('v20-upper', 'v20-upper', False),
    ('v21-lower', 'v21-lower', False),
    ('v21-upper', 'v21-upper', False),
    ('v22-lower', 'v22-lower', False),
    ('v22-upper', 'v22-upper', False),
    ('v23-lower', 'v23-lower', False),
    ('v23-upper', 'v23-upper', False),
    ('v24-lower', 'v24-lower', False),
    ('v24-upper', 'v24-upper', False),
    ('v25-lower', 'v25-lower', False),
    ('v25-upper', 'v25-upper', False),
    ('v26-lower', 'v26-lower', False),
    ('v26-upper', 'v26-upper', False),
    ('v27-lower', 'v27-lower', False),
    ('v27-upper', 'v27-upper', False),
    ('v28-lower', 'v28-lower', False),
    ('v28-upper', 'v28-upper', False),
    ('v29-lower', 'v29-lower', False),
    ('v29-upper', 'v29-upper', False),
    ('v30-lower', 'v30-lower', False),
    ('v30-upper', 'v30-upper', False),
    ('v31-lower', 'v31-lower', False),
    ('v31-upper', 'v31-upper', False),
    
]

sysdbg_neon128_register_type_v2_0 = ''.join([
    'Q',  # q0-upper
    'Q',  # q1-lower
    'Q',  # q1-upper
    'Q',  # q2-lower
    'Q',  # q2-upper
    'Q',  # q3-lower
    'Q',  # q3-upper
    'Q',  # q4-lower
    'Q',  # q4-upper
    'Q',  # q5-lower
    'Q',  # q5-upper
    'Q',  # q6-lower
    'Q',  # q6-upper
    'Q',  # q7-lower
    'Q',  # q7-upper
    'Q',  # q8-lower
    'Q',  # q8-upper
    'Q',  # q9-lower
    'Q',  # q9-upper
    'Q',  # q10-lower
    'Q',  # q10-upper
    'Q',  # q11-lower
    'Q',  # q11-upper
    'Q',  # q12-lower
    'Q',  # q12-upper
    'Q',  # q13-lower
    'Q',  # q13-upper
    'Q',  # q14-lower
    'Q',  # q14-upper
    'Q',  # q15-lower
    'Q',  # q15-upper
    'Q',  # q16-lower
    'Q',  # q0-lower
    'Q',  # q0-upper
    'Q',  # q1-lower
    'Q',  # q1-upper
    'Q',  # q2-lower
    'Q',  # q2-upper
    'Q',  # q3-lower
    'Q',  # q3-upper
    'Q',  # q4-lower
    'Q',  # q4-upper
    'Q',  # q5-lower
    'Q',  # q5-upper
    'Q',  # q6-lower
    'Q',  # q6-upper
    'Q',  # q7-lower
    'Q',  # q7-upper
    'Q',  # q8-lower
    'Q',  # q8-upper
    'Q',  # q9-lower
    'Q',  # q9-upper
    'Q',  # q10-lower
    'Q',  # q10-upper
    'Q',  # q11-lower
    'Q',  # q11-upper
    'Q',  # q12-lower
    'Q',  # q12-upper
    'Q',  # q13-lower
    'Q',  # q13-upper
    'Q',  # q14-lower
    'Q',  # q14-upper
    'Q',  # q15-lower
    'Q',  # q15-upper
])

cpu_per_regiter_no_format = ['pstate']

cpu_per_regiter_format = {}
cpu_per_regiter_format['tpidr_el0'] = 'Data.Set SPR:0x33D02 %Quad '
cpu_per_regiter_format['tpidrro_el0'] = 'Data.Set SPR:0x33D03 %Quad '
cpu_per_regiter_format['cpacr_el1'] = 'Data.Set SPR:0x30102 %Quad '
cpu_per_regiter_format['csselr_el1'] = 'Data.Set SPR:0x32000 %Quad '
cpu_per_regiter_format['esr_el1'] = 'Data.Set SPR:0x30520 %Quad '
cpu_per_regiter_format['far_el1'] = 'Data.Set SPR:0x30600 %Quad '
cpu_per_regiter_format['isr_el1'] = 'Data.Set SPR:0x30C10 %Quad '
cpu_per_regiter_format['par_el1'] = 'Data.Set SPR:0x30740 %Quad '
cpu_per_regiter_format['mair_el1'] = 'Data.Set SPR:0x30A20 %Quad '
cpu_per_regiter_format['sctlr_el1'] = 'Data.Set SPR:0x30100 %Quad '
cpu_per_regiter_format['tcr_el1'] = 'Data.Set SPR:0x30202 %Quad '
cpu_per_regiter_format['tpidr_el1'] = 'Data.Set SPR:0x30D04 %Quad '
cpu_per_regiter_format['ttbr0_el1'] = 'Data.Set SPR:0x30200 %Quad '
cpu_per_regiter_format['ttbr1_el1'] = 'Data.Set SPR:0x30201 %Quad '
cpu_per_regiter_format['vbar_el1'] = 'Data.Set SPR:0x30C00 %Quad '
cpu_per_regiter_format['cptr_el2'] = 'Data.Set SPR:0x34112 %Quad '
cpu_per_regiter_format['hcr_el2'] = 'Data.Set SPR:0x34110 %Quad '
cpu_per_regiter_format['mdcr_el2'] = 'Data.Set SPR:0x34111 %Quad '
cpu_per_regiter_format['vtcr_el2'] = 'Data.Set SPR:0x34212 %Quad '
cpu_per_regiter_format['vttbr_el2'] = 'Data.Set SPR:0x34210 %Quad '
cpu_per_regiter_format['cntkctl_el1'] = 'Data.Set SPR:0x30E10 %Quad '
cpu_per_regiter_format['cntv_ctl_el0'] = 'Data.Set SPR:0x33E31 %Quad '
cpu_per_regiter_format['cntv_cval_el0'] = 'Data.Set SPR:0x33E32 %Quad '
cpu_per_regiter_format['cntv_tval_el0'] = 'Data.Set SPR:0x33E30 %Quad '
cpu_per_regiter_format['cntp_ctl_el0'] = 'Data.Set SPR:0x33E21 %Quad '
cpu_per_regiter_format['cntp_cval_el0'] = 'Data.Set SPR:0x33E22 %Quad '
cpu_per_regiter_format['cntp_tval_el0'] = 'Data.Set SPR:0x33E20 %Quad '
cpu_per_regiter_format['cnthctl_el2'] = 'Data.Set SPR:0x34E10 %Quad '
cpu_per_regiter_format['cnthp_ctl_el2'] = 'Data.Set SPR:0x34E21 %Quad '
cpu_per_regiter_format['cnthp_cval_el2'] = 'Data.Set SPR:0x34E22 %Quad '
cpu_per_regiter_format['cnthp_tval_el2'] = 'Data.Set SPR:0x34E20 %Quad '


cpu_name = (
    'Invalid',
    'A53',
    'A57',
    'Hydra',
)

msm_dump_regset_ids = []
msm_dump_regset_ids.append('MSM_DUMP_REGSET_IDS_AARCH64_GPRS')
msm_dump_regset_ids.append('MSM_DUMP_REGSET_IDS_AARCH64_NEON')
msm_dump_regset_ids.append('MSM_DUMP_REGSET_IDS_AARCH64_SYSREGS_EL0')
msm_dump_regset_ids.append('MSM_DUMP_REGSET_IDS_AARCH64_EL1')
msm_dump_regset_ids.append('MSM_DUMP_REGSET_IDS_AARCH64_VM_EL2')
msm_dump_regset_ids.append('MSM_DUMP_REGSET_IDS_AARCH64_CNTV_EL10')
msm_dump_regset_ids.append('MSM_DUMP_REGSET_IDS_AARCH64_CNTP_EL10')
msm_dump_regset_ids.append('MSM_DUMP_REGSET_IDS_AARCH64_CNT_EL2')

sysdbg_cpu64_register_names = {}
sysdbg_cpu64_ctxt_regs_type = {}
sysdbg_cpu32_register_names = {}
sysdbg_cpu32_ctxt_regs_type = {}

sysdbg_cpu64_register_namesv20 = {}
sysdbg_cpu64_ctxt_regs_typev20 = {}


sysdbg_cpu64_register_names['default'] = sysdbg_cpu64_register_names_default
sysdbg_cpu64_ctxt_regs_type['default'] = sysdbg_cpu64_ctxt_regs_type_default
sysdbg_cpu32_register_names['default'] = sysdbg_cpu32_register_names_default
sysdbg_cpu32_ctxt_regs_type['default'] = sysdbg_cpu32_ctxt_regs_type_default

# Version 1.3
sysdbg_cpu64_register_names['1.3'] = sysdbg_cpu64_register_names_v1_3
sysdbg_cpu64_ctxt_regs_type['1.3'] = sysdbg_cpu64_ctxt_regs_type_v1_3

# Version 1.4
sysdbg_cpu64_register_names['1.4'] = sysdbg_cpu64_register_names_v1_4
sysdbg_cpu64_ctxt_regs_type['1.4'] = sysdbg_cpu64_ctxt_regs_type_v1_4


# Version 2.0
sysdbg_cpu64_register_namesv20['MSM_DUMP_REGSET_IDS_AARCH64_GPRS'] = sysdbg_cpu64_gprs_register_names_v2_0
sysdbg_cpu64_ctxt_regs_typev20['MSM_DUMP_REGSET_IDS_AARCH64_GPRS'] = sysdbg_cpu64_gprs_ctxt_regs_type_v2_0

sysdbg_cpu64_register_namesv20['MSM_DUMP_REGSET_IDS_AARCH64_EL1'] = sysdbg_cpu64_el1_register_names_v2_0
sysdbg_cpu64_ctxt_regs_typev20['MSM_DUMP_REGSET_IDS_AARCH64_EL1'] = sysdbg_cpu64_el1_ctxt_regs_type_v2_0

sysdbg_cpu64_register_namesv20['MSM_DUMP_REGSET_IDS_AARCH64_SYSREGS_EL0'] = sysdbg_cpu64_el0_register_names_v2_0
sysdbg_cpu64_ctxt_regs_typev20['MSM_DUMP_REGSET_IDS_AARCH64_SYSREGS_EL0'] = sysdbg_cpu64_el0_ctxt_regs_type_v2_0

sysdbg_cpu64_register_namesv20['MSM_DUMP_REGSET_IDS_AARCH64_VM_EL2'] = sysdbg_cpu64_vm_el2_register_names_v2_0
sysdbg_cpu64_ctxt_regs_typev20['MSM_DUMP_REGSET_IDS_AARCH64_VM_EL2'] = sysdbg_cpu64_vm_el2_ctxt_regs_type_v2_0

sysdbg_cpu64_register_namesv20['MSM_DUMP_REGSET_IDS_AARCH64_CNTV_EL10'] = sysdbg_cpu64_cntv_el10_register_names_v2_0
sysdbg_cpu64_ctxt_regs_typev20['MSM_DUMP_REGSET_IDS_AARCH64_CNTV_EL10'] = sysdbg_cpu64_cntv_el10_ctxt_regs_type_v2_0

sysdbg_cpu64_register_namesv20['MSM_DUMP_REGSET_IDS_AARCH64_CNTP_EL10'] = sysdbg_cpu64_cntp_el10_register_names_v2_0
sysdbg_cpu64_ctxt_regs_typev20['MSM_DUMP_REGSET_IDS_AARCH64_CNTP_EL10'] = sysdbg_cpu64_cntp_el10_ctxt_regs_type_v2_0

sysdbg_cpu64_register_namesv20['MSM_DUMP_REGSET_IDS_AARCH64_CNT_EL2'] = sysdbg_cpu64_cnt_el2_register_names_v2_0
sysdbg_cpu64_ctxt_regs_typev20['MSM_DUMP_REGSET_IDS_AARCH64_CNT_EL2'] = sysdbg_cpu64_cnt_el2_ctxt_regs_type_v2_0


sysdbg_cpu64_register_namesv20['MSM_DUMP_REGSET_IDS_AARCH64_NEON'] = sysdbg_neon128_register_names_v2_0
sysdbg_cpu64_ctxt_regs_typev20['MSM_DUMP_REGSET_IDS_AARCH64_NEON'] = sysdbg_neon128_register_type_v2_0
sysdbg_cpu64_register_names['2.0'] = sysdbg_cpu64_register_namesv20
sysdbg_cpu64_ctxt_regs_type['2.0'] = sysdbg_cpu64_ctxt_regs_typev20

class NeonCtxType():

    def __init__(self, regs_t, ramdump):
        i = 0
        self.regs = {}

        if ramdump.arm64 is None:
            return

        register_name = sysdbg_neon128_register_names_v1_4
        for r in regs_t:
            self.regs[register_name[i][0]] = r
            i += 1

class NeonCtxTypev2():

    def __init__(self, regs_t, ramdump):
        i = 0
        self.regs = {}

        if ramdump.arm64 is None:
            return

        register_name = sysdbg_neon128_register_names_v2_0
        for r in regs_t:
            self.regs[register_name[i][0]] = r
            i += 1

class TZCpuCtx_v2():

    def compute_pc(self, neon_regs):
        pstate = self.regs['cpu_state_1']
        cpu_state_3 = self.regs['cpu_state_3']
        cpu_state_5 = self.regs['cpu_state_5']
        pc = self.regs['pc']

        # AArch32 Mode
        if is_set(pstate, 4):
            val = pstate & 0xF
            if val == 0x0 and is_set(cpu_state_3, 14):
                self.regs['pc'] = self.regs['x14']
            elif val == 0x1 and is_set(cpu_state_3, 30):
                self.regs['pc'] = self.regs['x30']
            elif val == 0x2 and is_set(cpu_state_3, 16):
                self.regs['pc'] = self.regs['x16']
            elif val == 0x3 and is_set(cpu_state_3, 18):
                self.regs['pc'] = self.regs['x18']
            elif val == 0x7 and is_set(cpu_state_3, 20):
                self.regs['pc'] = self.regs['x20']
            elif val == 0xB and is_set(cpu_state_3, 22):
                self.regs['pc'] = self.regs['x22']
            elif val == 0x6 and is_set(cpu_state_5, 31):
                self.regs['pc'] = neon_regs['q31-upper']
            elif val == 0xA:
                self.regs['pc'] = self.regs['elr_el2']
            elif val == 0xF and is_set(cpu_state_3, 14):
                self.regs['pc'] = self.regs['x14']
            else:
                print_out_str('!!! AArch32 PC Approximation Logic Failed!')
        # AArch64 Mode
        else:
            if is_set(cpu_state_3, 30):
                self.regs['pc'] = self.regs['x30']
            else:
                val = (pstate >> 2) & 0x3
                if val == 0x0:
                    self.regs['pc'] = self.regs['elr_el1']
                elif val == 0x1:
                    self.regs['pc'] = self.regs['elr_el1']
                elif val == 0x2:
                    self.regs['pc'] = self.regs['elr_el2']
                elif val == 0x3:
                    self.regs['pc'] = self.regs['elr_el3']
                else:
                    print_out_str('!!! AArch64 PC Approximation Logic Failed!')

        if pc and pc != self.regs['pc']:
            print_out_str(
                '!!! PC computed by SDI {0} and Parser {1} are different!'
                .format(hex(pc), hex(self.regs['pc'])))
    
    def compute_pcv2(self, neon_regs):
        pstate = self.regs['pstate']
        pc = self.regs['pc']
        if is_set(cpu_state_3, 30):
            self.regs['pc'] = self.regs['x30']
        else:
            val = (pstate >> 2) & 0x3
            if val == 0x0:
                self.regs['pc'] = self.regs['elr_el1']
            elif val == 0x1:
                self.regs['pc'] = self.regs['elr_el1']
            else:
                print_out_str('!!! AArch64 PC Approximation Logic Failed!')

        if pc and pc != self.regs['pc']:
            print_out_str(
                '!!! PC computed by SDI {0} and Parser {1} are different!'
                .format(hex(pc), hex(self.regs['pc'])))

    def __init__(self, version, regs_t, neon_regs, ramdump):
        i = 0
        self.regs = {}
        self.version = version
        if ramdump.arm64:
            if self.version == '2.0':
                register_names = sysdbg_cpu64_register_names[self.version]
                register_name = []
                for item in msm_dump_regset_ids:
                    if item in register_names and 'MSM_DUMP_REGSET_IDS_AARCH64_NEON' not in item:
                        for each_reg in register_names[item]:
                            register_name.append(each_reg)
            else:
                register_name = sysdbg_cpu64_register_names[self.version]
        else:
            register_name = sysdbg_cpu32_register_names[self.version]
        
        for r in regs_t:
            self.regs[register_name[i][0]] = r      
            i += 1
        if self.version == '1.4' and self.regs['cpu_state_0'] == 0x1:
            print_out_str(
                '!!! PC is invalid, applying "PC Approximation Logic"!')
            self.compute_pc(neon_regs)
        if self.version == '2.0' and self.regs['pstate'] == 0x1:
            print_out_str(
                '!!! PC is invalid, applying "PC Approximation Logic....++++++"!')
            self.compute_pcv2(neon_regs)

    def print_regs(self, outfile, ramdump):
        if ramdump.arm64:
            if self.version == '2.0':
                register_name_all = sysdbg_cpu64_register_names[self.version]
                register_names = []
                for item in msm_dump_regset_ids:
                    if item in register_name_all and 'MSM_DUMP_REGSET_IDS_AARCH64_NEON' not in item:
                        for each_reg in register_name_all[item]:
                            register_names.append(each_reg)
            else:
                register_names = sysdbg_cpu64_register_names[self.version]
        else:
            register_names = sysdbg_cpu32_register_names[self.version]
        for reg_name, t32_name, print_pc in register_names:
            if re.match('(.*)reserved(.*)', reg_name):
                continue
            if print_pc:
                pc_string = None
                if reg_name in self.regs:
                    a = ramdump.unwind_lookup(self.regs[reg_name])
                    if a is not None:
                        symname, offset = ramdump.unwind_lookup(
                            self.regs[reg_name])
                        pc_string = '[{0}+0x{1:x}]'.format(symname, offset)
                    else:
                        pc_string = None
            else:
                pc_string = None
            if reg_name in self.regs:
                if pc_string is not None:
                    print_out_str('   {0:8} = 0x{1:016x} {2}'.format(
                                  reg_name, self.regs[reg_name], pc_string))
                else:
                    print_out_str('   {0:8} = 0x{1:016x}'.format(
                                  reg_name, self.regs[reg_name]))
                if t32_name is not None:
                    if reg_name.startswith('cpu_state_') or reg_name.startswith('pstate') :
                        continue
                    if t32_name in cpu_per_regiter_format.keys():
                        outfile.write(
                            '{0} 0x{1:x}\n'.format(cpu_per_regiter_format[t32_name], self.regs[reg_name]))
                        if 'sctlr_el1' in t32_name and ramdump.hlos_sctlr_el1 is None:
                            ramdump.hlos_sctlr_el1 = self.regs[reg_name]
                        if 'tcr_el1' in t32_name and ramdump.hlos_tcr_el1 is None:
                            ramdump.hlos_tcr_el1 = self.regs[reg_name]
                    else:
                        outfile.write(
                            'r.s {0} 0x{1:x}\n'.format(t32_name, self.regs[reg_name]))


class TZRegDump_v2():

    def __init__(self):
        self.core_regs = None
        self.sec_regs = None
        self.neon_regs = {}
        self.version = 0
        self.start_addr = 0
        self.end_addr = 0
        self.core = 0
        self.status = []
        self.neon_fields = []

    def dump_all_regs(self, ram_dump):
        coren_regs = ram_dump.open_file('core{0}_regs.cmm'.format(self.core))

        print_out_str('core{0} regs:'.format(self.core))
        self.core_regs.print_regs(coren_regs, ram_dump)
        coren_regs.close()

        secure_regs = ram_dump.open_file(
            'secure_world_core{0}_regs.cmm'.format(self.core))
        print_out_str('\n=============== secure contex ===========')
        self.sec_regs.print_regs(secure_regs, ram_dump)
        print_out_str('============ end secure context ===========')
        secure_regs.close()

    def dump_core_pc(self, ram_dump):
        pc = self.core_regs.regs['pc']
        if ram_dump.arm64:
            lr = self.core_regs.regs['x30']
            if ram_dump.currentEL == 0x2:
                bt = self.core_regs.regs['sp_el2']
            else:
                bt = self.core_regs.regs['sp_el1']
            fp = self.core_regs.regs['x29']
        else:
            lr = self.core_regs.regs['r14_svc']
            bt = self.core_regs.regs['r13_svc']
            fp = self.core_regs.regs['r11']

        pc = ram_dump.pac_ignore(pc)
        a = ram_dump.unwind_lookup(pc)
        if a is not None:
            symname, offset = a
        else:
            symname = 'UNKNOWN'
            offset = 0
        print_out_str(
            'Core {3} PC: {0}+{1:x} <{2:x}>'.format(symname, offset,
                                                    pc, self.core))
        lr = ram_dump.pac_ignore(lr)
        a = ram_dump.unwind_lookup(lr)
        if a is not None:
            symname, offset = a
        else:
            symname = 'UNKNOWN'
            offset = 0
        print_out_str(
            'Core {3} LR: {0}+{1:x} <{2:x}>'.format(symname, offset,
                                                    lr, self.core))
        print_out_str('')
        ram_dump.unwind.unwind_backtrace(bt, fp, pc, lr, '')
        print_out_str('')
    
    def dump_all_regs_gprs(self, ram_dump):
        coren_regs = ram_dump.open_file('core{0}_regs.cmm'.format(self.core))

        print_out_str('core{0} regs:'.format(self.core))
        self.core_regs.print_regs(coren_regs, ram_dump)
        coren_regs.close()

    def dump_core_pc_gprs(self, ram_dump):
        pc = self.core_regs.regs['pc']
        if ram_dump.arm64:
            lr = self.core_regs.regs['x30']
            if self.core_regs.regs['pstate'] & 1 == 1: 
                bt = self.core_regs.regs['sp_el1']
            elif self.core_regs.regs['pstate'] & 1 == 0: 
                bt = self.core_regs.regs['sp_el0']
            fp = self.core_regs.regs['x29']
        else:
            lr = self.core_regs.regs['r14_svc']
            bt = self.core_regs.regs['r13_svc']
            fp = self.core_regs.regs['r11']

        pc = ram_dump.pac_ignore(pc)
        a = ram_dump.unwind_lookup(pc)
        if a is not None:
            symname, offset = a
        else:
            symname = 'UNKNOWN'
            offset = 0
        print_out_str(
            'Core {3} PC: {0}+{1:x} <{2:x}>'.format(symname, offset,
                                                    pc, self.core))
        lr = ram_dump.pac_ignore(lr)
        a = ram_dump.unwind_lookup(lr)
        if a is not None:
            symname, offset = a
        else:
            symname = 'UNKNOWN'
            offset = 0
        print_out_str(
            'Core {3} LR: {0}+{1:x} <{2:x}>'.format(symname, offset,
                                                    lr, self.core))
        print_out_str('')
        ram_dump.unwind.unwind_backtrace(bt, fp, pc, lr, '')
        print_out_str('')
    def init_regs_v2(self, version, regset_name_addr, core, ram_dump):
        self.version = '{0}.{1}'.format(version >> 4, version & 0xF)
        sc_regs = []
        if ram_dump.arm64:
            register_names = sysdbg_cpu64_register_names[self.version]
            ctx_types = sysdbg_cpu64_ctxt_regs_type[self.version]
            for regset_key_name in regset_name_addr:
                start_addr = regset_name_addr[regset_key_name][0]
                end_addr = regset_name_addr[regset_key_name][1]
                self.start_addr = start_addr
                self.end_addr = end_addr
                self.core = core
                if 'MSM_DUMP_REGSET_IDS_AARCH64_NEON' in regset_key_name:
                    neon_ctx_regs = ram_dump.read_string(
                        self.start_addr,
                        sysdbg_neon128_register_type_v2_0,
                        False)
                    neon = NeonCtxTypev2(neon_ctx_regs, ram_dump)
                    self.neon_regs = neon.regs
                else:
                    temp_sc_regs = ram_dump.read_string(
                        self.start_addr,
                        ctx_types[regset_key_name],
                        False)
                    for each_addr_item in temp_sc_regs:
                        sc_regs.append(each_addr_item)
                
                
            self.core_regs = TZCpuCtx_v2(self.version, sc_regs,
                                     self.neon_regs, ram_dump)
        return True
    
    def init_regs(self, version, start_addr, end_addr, core, ram_dump):
        self.start_addr = start_addr
        self.end_addr = end_addr
        self.core = core

        self.version = '{0}.{1}'.format(version >> 4, version & 0xF)
        if ram_dump.arm64:
            register_names = sysdbg_cpu64_register_names
        else:
            register_names = sysdbg_cpu32_register_names

        if self.version not in register_names:
            self.version = 'default'

        # uint32 status[4]; -- status fields
        # sdi_cpu_ctxt_regs_type cpu_regs; -- ctxt for all cpus
        # sdi_cpu_ctxt_regs_type __reserved3; -- secure ctxt
        for i in range(0, 4):
            self.status.append(ram_dump.read_u32(self.start_addr, False))
            self.start_addr += 4

        if ram_dump.arm64:
            sc_regs = ram_dump.read_string(
                self.start_addr,
                sysdbg_cpu64_ctxt_regs_type[self.version],
                False)
            self.start_addr += struct.calcsize(
                sysdbg_cpu64_ctxt_regs_type[self.version])
            sc_secure = ram_dump.read_string(
                self.start_addr,
                sysdbg_cpu64_ctxt_regs_type[self.version],
                False)
            self.start_addr += struct.calcsize(
                sysdbg_cpu64_ctxt_regs_type[self.version])

            if self.version == '1.4':
                for i in range(0, 3):
                    self.neon_fields.append(ram_dump.read_u32(
                                            self.start_addr, False))
                    self.start_addr += 4

                neon_ctx_regs = ram_dump.read_string(
                    self.start_addr,
                    sysdbg_neon128_register_type_v1_4,
                    False)
                self.start_addr += struct.calcsize(
                    sysdbg_neon128_register_type_v1_4)

                neon = NeonCtxType(neon_ctx_regs, ram_dump)
                self.neon_regs = neon.regs
        else:
            sc_regs = ram_dump.read_string(
                self.start_addr,
                sysdbg_cpu32_ctxt_regs_type[self.version],
                False)
            self.start_addr += struct.calcsize(
                sysdbg_cpu32_ctxt_regs_type[self.version])
            sc_secure = ram_dump.read_string(
                self.start_addr,
                sysdbg_cpu32_ctxt_regs_type[self.version],
                False)
            self.start_addr += struct.calcsize(
                sysdbg_cpu32_ctxt_regs_type[self.version])

        self.core_regs = TZCpuCtx_v2(self.version, sc_regs,
                                     self.neon_regs, ram_dump)

        self.sec_regs = TZCpuCtx_v2(self.version, sc_secure,
                                    self.neon_regs, ram_dump)
        return True


@register_parser('--watchdog', 'Extracts NS Watchdog related info')
class Watchdog(RamParser):
    def parse(self):
        get_wdog_timing(self.ramdump)


def ns_to_sec(ns):
    return ns/1000000000.0


def get_wdog_timing(ramdump):
    logical_map = []
    runq_online_bits = 0
    jiffies = ramdump.read_word('jiffies')
    last_jiffies_update = ramdump.read_s64('last_jiffies_update')
    tick_do_timer_cpu = ramdump.read_s32('tick_do_timer_cpu')
    wdog_data_addr = ramdump.read_word('wdog_data')
    if wdog_data_addr is None and ramdump.minidump:
        wdog_seg = next((s for s in ramdump.elffile.iter_sections() if s.name == 'KWDOGDATA'), None)
        if wdog_seg is not None:
            wdog_data_addr = wdog_seg['sh_addr']
    pet_timer_off = ramdump.field_offset(
        'struct msm_watchdog_data', 'pet_timer')
    timer_expires_off = ramdump.field_offset('struct timer_list', 'expires')
    pet_timer_expires = ramdump.read_word(
        wdog_data_addr + pet_timer_off + timer_expires_off)
    timer_flags_off = ramdump.field_offset('struct timer_list', 'flags')
    pet_timer_flags = ramdump.read_word(
        wdog_data_addr + pet_timer_off + timer_flags_off)
    wdog_last_pet = ramdump.read_structure_field(
        wdog_data_addr, 'struct msm_watchdog_data', 'last_pet')
    #For kernel version less than 4.4, as the member variable timer_expired
    #  is not available we need to get watchdog timer expire status
    if (ramdump.kernel_version < (4, 4)):
        pet_timer_entry_offset = ramdump.field_offset('struct timer_list', 'entry')
        pet_timer_prev_offset = ramdump.field_offset('struct list_head', 'prev')
        pet_timer_entry_prev = ramdump.read_word(
            wdog_data_addr + pet_timer_off + pet_timer_entry_offset + pet_timer_prev_offset)

        if(pet_timer_entry_prev == '0x200'):
            pet_timer_expired = 1
        else:
            pet_timer_expired = 0
    else:
        timer_expired_off = ramdump.field_offset(
            'struct msm_watchdog_data', 'timer_expired')
        pet_timer_expired = ramdump.read_word(wdog_data_addr + timer_expired_off)
    pet_time_off = ramdump.field_offset('struct msm_watchdog_data', 'pet_time')
    bark_time_off = ramdump.field_offset(
        'struct msm_watchdog_data', 'bark_time')
    pet_time = ramdump.read_int(wdog_data_addr + pet_time_off)
    bark_time = ramdump.read_int(wdog_data_addr + bark_time_off)
    wdog_alive_mask = ramdump.read_structure_field(
        wdog_data_addr, 'struct msm_watchdog_data', 'alive_mask.bits')
    if not ramdump.minidump:
        tick_bc_mask = ramdump.read_word('tick_broadcast_oneshot_mask')
        tick_bc_pending_mask = ramdump.read_word('tick_broadcast_pending_mask')
        tick_bc_force_mask = ramdump.read_word('tick_broadcast_force_mask')
        tick_bc_evt_dev = ramdump.read_structure_field(
            'tick_broadcast_device', 'struct tick_device', 'evtdev')
        tick_bc_next_evt = ramdump.read_structure_field(
            tick_bc_evt_dev, 'struct clock_event_device', 'next_event')
        tick_bc_next_evt = ns_to_sec(tick_bc_next_evt)
        tick_bc_cpumask = ramdump.read_structure_field(
            tick_bc_evt_dev, 'struct clock_event_device', 'cpumask')
        tick_bc_cpumask_bits = ramdump.read_structure_field(
            tick_bc_cpumask, 'struct cpumask', 'bits')
        if tick_bc_cpumask_bits is None:
            tick_bc_cpumask_bits = 0
    if ramdump.is_config_defined('CONFIG_SMP'):
        runqueues_addr = ramdump.address_of('runqueues')
        online_offset = ramdump.field_offset('struct rq', 'online')

        for i in ramdump.iter_cpus():
            rq_addr = runqueues_addr + ramdump.per_cpu_offset(i)
            online = ramdump.read_int(rq_addr + online_offset)
            runq_online_bits |= (online << i)

    if (ramdump.kernel_version >= (4, 9, 0)):
        cpu_online_bits = ramdump.read_word('__cpu_online_mask')
    else:
        cpu_online_bits = ramdump.read_word('cpu_online_bits')

    cpu_isolated_bits = cpu_isolation_mask(ramdump)

    if not ramdump.minidump:
        wdog_task = ramdump.read_structure_field(
            wdog_data_addr, 'struct msm_watchdog_data', 'watchdog_task')
        wdog_task_state = ramdump.read_structure_field(
            wdog_task, 'struct task_struct', 'state')
        wdog_task_threadinfo = ramdump.read_structure_field(
            wdog_task, 'struct task_struct', 'stack')
        wdog_task_cpu = ramdump.get_task_cpu(wdog_task, wdog_task_threadinfo)
        wdog_task_oncpu = ramdump.read_structure_field(
            wdog_task, 'struct task_struct', 'on_cpu')
        wdog_task_arrived = ramdump.read_structure_field(
            wdog_task, 'struct task_struct', 'sched_info.last_arrival')
        wdog_task_queued = ramdump.read_structure_field(
            wdog_task, 'struct task_struct', 'sched_info.last_queued')
    logical_map_addr = ramdump.address_of('__cpu_logical_map')
    for i in range(0, ramdump.get_num_cpus()):
        cpu_logical_map_addr = logical_map_addr + (i * 8)
        core_id = ramdump.read_u64(cpu_logical_map_addr)
        logical_map.append(core_id)
    print_out_str('Non-secure Watchdog data')
    print_out_str('Pet time: {0}s'.format(pet_time / 1000.0))
    print_out_str('Bark time: {0}s'.format(bark_time / 1000.0))
    if wdog_last_pet > 1000000000:
        last_pet_sec = ns_to_sec(wdog_last_pet)
        print_out_str('Watchdog last pet: {0:.9f}'.format(last_pet_sec))
    else:
        print_out_str('Watchdog last pet: {0:.3f}'.format(wdog_last_pet))

    if not ramdump.minidump:
        if wdog_task_state == 0 and wdog_task_oncpu == 1:
            print_out_str("Watchdog task running on core {0} from {1:.6f}".format(
                wdog_task_cpu, ns_to_sec(wdog_task_arrived)))
            print_out_str("CPUs responded to pet(alive_mask): {0:08b}".format(
                wdog_alive_mask))
            alive_cpus = wdog_alive_mask | (~cpu_online_bits) | cpu_isolated_bits
            for i in range(0, ramdump.get_num_cpus()):
                if (alive_cpus & 1):
                    alive_cpus = alive_cpus >> 1
                else:
                    print_out_str("CPU which didn't respond to pet: {0}".format(i))
                    break

        elif wdog_task_state == 0:
            print_out_str(
                "Watchdog task is waiting on core {0} from {1:.6f}".format(
                    wdog_task_cpu, ns_to_sec(wdog_task_queued)))

        elif wdog_task_state == 1 and pet_timer_expired == 1:
            print_out_str("Pet timer expired but Watchdog task is not queued")

        else:
            print_out_str('Watchdog pet timer not expired')
            if jiffies > pet_timer_expires:
                print_out_str('Current jiffies crossed pet_timer expires jiffies')

    print_out_str('CPU online bits: {0:08b}'.format(cpu_online_bits))
    print_out_str('CPU runqueue online bits: {0:08b}'.format(runq_online_bits))
    print_out_str('CPU isolated bits: {0:08b}'.format(cpu_isolated_bits))
    print_out_str('pet_timer_flags: 0x{0:x}'.format(pet_timer_flags))
    print_out_str('pet_timer_expires: {0}'.format(pet_timer_expires))
    print_out_str('Current jiffies  : {0}'.format(jiffies))
    print_out_str(
        'Timestamp of last timer interrupt(last_jiffies_update): {0}'.format(
            ns_to_sec(last_jiffies_update)))
    print_out_str("tick_do_timer_cpu: {0}".format(tick_do_timer_cpu))
    print_out_str('CPU logical map: {0}'.format(logical_map))
    if not ramdump.minidump:
        print_out_str('tick_broadcast_oneshot_mask: {0:08b}'.format(tick_bc_mask))
        print_out_str(
            'tick_broadcast_pending_mask: {0:08b}'.format(tick_bc_pending_mask))
        print_out_str(
            'tick_broadcast_force_mask: {0:08b}'.format(tick_bc_force_mask))
        print_out_str(
            'tick_broadcast_device cpumask: {0:08b}'.format(tick_bc_cpumask_bits))
        print_out_str(
            'tick_broadcast_device next_event: {0:.6f}'.format(tick_bc_next_evt))
        for i in range(0, ramdump.get_num_cpus()):
            tick_cpu_device = ramdump.address_of(
                'tick_cpu_device') + ramdump.per_cpu_offset(i)
            evt_dev = ramdump.read_structure_field(
                tick_cpu_device, 'struct tick_device', 'evtdev')
            if evt_dev != 0:
                next_event = ramdump.read_structure_field(
                    evt_dev, 'struct clock_event_device', 'next_event')
                next_event = ns_to_sec(next_event)
                print_out_str(
                    "CPU{0} tick_device next_event: {1:.6f}".format(i, next_event))
    epoch_ns = ramdump.read_u64('cd.read_data[0].epoch_ns')
    epoch_cyc = ramdump.read_u64('cd.read_data[0].epoch_cyc')
    print_out_str('epoch_ns: {0}ns  epoch_cyc: {1}'.format(epoch_ns,epoch_cyc))
    if (ramdump.kernel_version >= (4, 14)):
        ping_start_time_offset = ramdump.field_offset(
                        'struct msm_watchdog_data', 'ping_start')
        ping_end_time_offset = ramdump.field_offset(
                        'struct msm_watchdog_data', 'ping_end')
        for i in range(0, ramdump.get_num_cpus()):
            ping_start_time = ramdump.read_u64(wdog_data_addr +
                                                 ping_start_time_offset + i*8)
            ping_end_time = ramdump.read_u64(wdog_data_addr +
                                                 ping_end_time_offset + i*8)
            print_out_str("CPU#{0} : ping_start: {1:.6f} : ping_end: {2:.6f}"
                          .format(i, ns_to_sec(ping_start_time),
                                  ns_to_sec(ping_end_time)))
        timer_fired = ramdump.read_structure_field(
            wdog_data_addr, 'struct msm_watchdog_data', 'timer_fired')
        print_out_str('timer_fired : {0:.6f}'.format(ns_to_sec(timer_fired)))
        thread_start = ramdump.read_structure_field(
            wdog_data_addr, 'struct msm_watchdog_data', 'thread_start')
        print_out_str('thread_start : {0:.6f}'.format(ns_to_sec(
            thread_start)))
