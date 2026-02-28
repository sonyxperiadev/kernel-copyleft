# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.

from parser_util import register_parser, RamParser

@register_parser('--print-preempt-count', 'Print the preempt_count')
class PrintPreemptCount(RamParser):
    def __init__(self, *args):
        super(PrintPreemptCount, self).__init__(*args)

    def parse(self):
        if not self.ramdump.arm64:
            return;

        f = self.ramdump.open_file('preempt_count.txt')
        runqueues_addr = self.ramdump.address_of('runqueues')
        curr_offset = self.ramdump.field_offset('struct rq', 'curr')
        pid_offset = self.ramdump.field_offset('struct task_struct', 'pid')
        comm_offset = self.ramdump.field_offset('struct task_struct', 'comm')

        for i in self.ramdump.iter_cpus():
            rq_addr = runqueues_addr + self.ramdump.per_cpu_offset(i)
            curr_addr = self.ramdump.read_word(rq_addr + curr_offset)
            pid = self.ramdump.read_int(curr_addr + pid_offset)
            comm = self.ramdump.read_cstring(curr_addr + comm_offset, 16)
            f.write('CPU{} current task comm: {:<16} pid: {} \n'.format(i, comm, pid))
            preempt_count = self.ramdump.read_structure_field(curr_addr, 'struct thread_info', 'preempt.count')
            f.write('preempt.count: 0x{:x} \n'.format(preempt_count))
            # PREEMPT_MASK:	0x000000ff
            # SOFTIRQ_MASK:	0x0000ff00
            # HARDIRQ_MASK:	0x000f0000
            #     NMI_MASK:	0x00f00000
            f.write('\t preemption           count: {0} \n'.format(preempt_count  & 0x000000ff))
            f.write('\t softirq              count: {0} \n'.format((preempt_count & 0x0000ff00) >> 8))
            f.write('\t hardirq              count: {0} \n'.format((preempt_count & 0x000f0000) >> 16))
            f.write('\t nmi                  count: {0} \n'.format((preempt_count & 0x00f00000) >> 20))
            preempt_need_resched = self.ramdump.read_structure_field(curr_addr, 'struct thread_info', 'preempt.need_resched')
            f.write('preempt.need_resched: 0x{:x} \n'.format(preempt_need_resched))
            flags = self.ramdump.read_structure_field(curr_addr, 'struct thread_info', 'flags')
            f.write('thread information flags: 0x{:x} \n'.format(flags))
            f.write('\t TIF_NEED_RESCHED          : {0} \n'.format((flags & 0x2) >> 1))
            f.write('\n')

        f.close()
