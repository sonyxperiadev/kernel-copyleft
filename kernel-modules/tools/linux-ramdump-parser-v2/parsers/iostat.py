#SPDX-License-Identifier: GPL-2.0-only
#Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.

import linux_list as llist
from parser_util import register_parser, RamParser, cleanupString
from print_out import print_out_str

@register_parser('--print-iostat', 'Print iostat info and block device list')
class iostat(RamParser):

    def __init__(self, *args):
        super(iostat, self).__init__(*args)

    def io_per_cpu_offset(self, cpu):
        per_cpu_offset_addr = self.ramdump.address_of('__per_cpu_offset')
        if per_cpu_offset_addr is None:
            return 0
        per_cpu_offset_addr_indexed = self.ramdump.array_index(
            per_cpu_offset_addr, 'unsigned long', cpu)
        #instead of read_slong, bd_stats + u64
        return self.ramdump.read_u64(per_cpu_offset_addr_indexed)

    def klist_node_to_dev(self, klist_node):
        dev = self.ramdump.container_of(klist_node, 'struct device', 'knode_class')
        if dev is None:
            dev_private = self.ramdump.container_of(klist_node, 'struct device_private',
                                                    'knode_class')
            if dev_private is None:
                return None
            dev = self.ramdump.read_structure_field(dev_private, 'struct device_private',
                                                    'device')
            if dev is None:
                return None
        return dev

    def block_class_init_walker(self, klist_node):
        dev = self.klist_node_to_dev(klist_node)
        if dev is None:
            return
        block_device = self.ramdump.container_of(dev, 'struct block_device', 'bd_device')
        bd_disk = self.ramdump.read_structure_field(block_device, 'struct block_device', 'bd_disk')
        disk_name_addr = self.ramdump.struct_field_addr(bd_disk, 'struct gendisk', 'disk_name')
        disk_name = self.ramdump.read_cstring(disk_name_addr)
        major = self.ramdump.read_structure_field(bd_disk, 'struct gendisk', 'major')
        queue = self.ramdump.read_structure_field(bd_disk, 'struct gendisk', 'queue')
        part0 = self.ramdump.read_structure_field(bd_disk, 'struct gendisk', 'part0')
        bd_stats = self.ramdump.read_structure_field(part0, 'struct  block_device', 'bd_stats')
        count0 = 0
        count1 = 0
        if disk_name == None or disk_name in self.disk_name:
            return
        else:
            self.disk_name.append(disk_name)
        for i in self.ramdump.iter_cpus():
            disk_stats = bd_stats + self.io_per_cpu_offset(i)
            in_flight = self.ramdump.struct_field_addr(disk_stats, 'struct disk_stats', 'in_flight')
            if self.ramdump.arm64:
                in_flight0 = self.ramdump.read_s64(in_flight)
                in_flight1 = self.ramdump.read_s64(in_flight + 8)
            else:
                in_flight0 = self.ramdump.read_s32(in_flight)
                in_flight1 = self.ramdump.read_s32(in_flight + 4)
            count0 += in_flight0
            count1 += in_flight1

        self.list_ouput.append([major, bd_disk, disk_name, queue, count0, count1, count0 + count1])

    def init_block_class(self):
        self.disk_name = []
        block_class = self.ramdump.address_of('block_class')
        if block_class is None:
            self.f.write("ERROR: 'block_class' not found\n")
            return
        p = self.ramdump.read_structure_field(block_class, 'struct class', 'p')
        list_head = (p + self.ramdump.field_offset('struct subsys_private', 'klist_devices')
                    + self.ramdump.field_offset('struct klist', 'k_list'))
        list_offset = self.ramdump.field_offset('struct klist_node', 'n_node')

        init_list_walker = llist.ListWalker(self.ramdump, list_head, list_offset)
        if not init_list_walker.is_empty():
            init_list_walker.walk(init_list_walker.next() + list_offset,
                                  self.block_class_init_walker)

        sorted_list = sorted(self.list_ouput, key=lambda l: l[6], reverse=True)
        print("MAJOR     gendisk          NAME       request_queue      TOTAL     ASYNC     SYNC \n", file = self.f)
        for item in sorted_list:
            major = item[0]
            bd_disk = item[1]
            disk_name = item[2]
            queue = item[3]
            count0 = item[4]
            count1 = item[5]
            print("%-6d 0x%x  %-8s 0x%x   %-8d   %-8d   %-8d  "
                % (major, bd_disk, disk_name, queue, count0 + count1, count0, count1), file=self.f)

    def parse(self):
        if (self.ramdump.kernel_version) < (5, 15, 0):
            print_out_str('\n kernel 5.15 or above supported \n')
            return
        self.list_ouput = []
        self.f = open(self.ramdump.outdir + "/iostat.txt", "w")
        self.init_block_class()
        self.f.close()