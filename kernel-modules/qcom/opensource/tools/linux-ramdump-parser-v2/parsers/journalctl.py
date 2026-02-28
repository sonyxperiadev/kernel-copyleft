#SPDX-License-Identifier: GPL-2.0-only
#Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.

from parser_util import register_parser, RamParser, cleanupString
from print_out import print_out_str
import struct
import traceback
import os
from utasklib import UTaskLib
from utasklib import ProcessNotFoundExcetion

@register_parser('--journalctl', 'Extract journalctl logs from ramdump ')
class Journald(RamParser):
    JOURNALCTL_FILE = "systemd.journald"
    def __init__(self, *args):
        super(Journald, self).__init__(*args)
        self.vma_list = []

    def generate_file(self, mmu):
        count = 0
        base, ext = os.path.splitext(self.JOURNALCTL_FILE)

        if len(self.vma_list) == 0:
            print_out_str("Failed to generate "+self.JOURNALCTL_FILE)
        else:
            for vma_info in self.vma_list:
                outfile_name = f"{base}{f'@{count}' if count != 0 else ''}{ext}"
                print_out_str(outfile_name+" base address is {0:x}".format(vma_info['start']))
                self.ramdump.remove_file(outfile_name)
                count = count + 1
                with self.ramdump.open_file(outfile_name, 'ab') as out_file:
                    min = vma_info['start']
                    size = vma_info['size']
                    max = min + size
                    while(min < max):
                        phys = mmu.virt_to_phys(min)
                        if phys is None:
                            min = min + 0x1000
                            out_file.write(b'\x00' * 0x1000)
                            continue

                        out_file.write(self.ramdump.read_physical(phys, 0x1000))
                        min = min + 0x1000
        return

    def generate_journalctl_file(self, taskinfo):
        '''
        generate systemd.journald for linux systemd.
        param taskinfo: utasklib.UTaskInfo
        '''
        for vma in taskinfo.vmalist:
            size = vma.vm_end - vma.vm_start
            if vma.file_name != "system.journal":
                continue
            vma_info = {}
            vma_info['start'] = vma.vm_start
            vma_info['size'] = size
            self.vma_list.append(vma_info)
        self.generate_file(taskinfo.mmu)

    def parse(self):
        try:
            try:
                taskinfo = UTaskLib(self.ramdump).get_utask_info("systemd-journal")
            except ProcessNotFoundExcetion:
                print_out_str("systemd-journald process is not started")
                return
            self.generate_journalctl_file(taskinfo)
        except Exception as result:
            print_out_str(str(result))
            traceback.print_exc()

