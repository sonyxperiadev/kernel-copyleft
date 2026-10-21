# Copyright (c) 2012-2015, 2017, 2020 The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from print_out import print_out_str
from parser_util import register_parser, RamParser


@register_parser('--print-irqs', 'Print all the irq information', shortopt='-i')
class IrqParse(RamParser):

    def print_irq_state_3_0(self, ram_dump):
        print_out_str(
            '=========================== IRQ STATE ===============================')
        per_cpu_offset_addr = ram_dump.address_of('__per_cpu_offset')
        cpus = ram_dump.get_num_cpus()
        irq_desc = ram_dump.address_of('irq_desc')
        foo, irq_desc_size = ram_dump.unwind_lookup(irq_desc, 1)
        h_irq_offset = ram_dump.field_offset('struct irq_desc', 'handle_irq')
        irq_num_offset = ram_dump.field_offset('struct irq_data', 'irq')
        hwirq_num_offset = ram_dump.field_offset('struct irq_data', 'hwirq')
        if ram_dump.kernel_version >= (4,4,0) :
            affinity_offset = ram_dump.field_offset(
                'struct irq_common_data', 'affinity')
            irq_common_data_offset = ram_dump.field_offset(
                'struct irq_desc', 'irq_common_data')
        else:
            affinity_offset = ram_dump.field_offset(
                'struct irq_data', 'affinity')
        irq_data_offset = ram_dump.field_offset('struct irq_desc', 'irq_data')
        irq_count_offset = ram_dump.field_offset(
            'struct irq_desc', 'irq_count')
        irq_chip_offset = ram_dump.field_offset('struct irq_data', 'chip')
        irq_action_offset = ram_dump.field_offset('struct irq_desc', 'action')
        action_name_offset = ram_dump.field_offset('struct irqaction', 'name')
        kstat_irqs_offset = ram_dump.field_offset(
            'struct irq_desc', 'kstat_irqs')
        chip_name_offset = ram_dump.field_offset('struct irq_chip', 'name')
        irq_desc_entry_size = ram_dump.sizeof('irq_desc[0]')
        cpu_str = ''

        for i in range(0, cpus):
            cpu_str = cpu_str + '{0:10} '.format('CPU{0}'.format(i))

        print_out_str('{0:4} {1:12} {2:10} {3} {4:30} {5:10}'
                    .format('IRQ', 'HWIRQ', 'affinity',
                        cpu_str, 'Name', 'Chip'))
        for i in range(0, irq_desc_size, irq_desc_entry_size):
            irqnum = ram_dump.read_word(irq_desc + i + irq_num_offset)
            hwirq = ram_dump.read_word(irq_desc + i + hwirq_num_offset)
            if ram_dump.kernel_version >= (4,4,0):
                affinity = ram_dump.read_int(
                    irq_desc + irq_common_data_offset + affinity_offset)
            else:
                affinity = ram_dump.read_int(
                    irq_desc + affinity_offset)
            irqcount = ram_dump.read_word(irq_desc + i + irq_count_offset)
            action = ram_dump.read_word(irq_desc + i + irq_action_offset)
            kstat_irqs_addr = ram_dump.read_word(
                irq_desc + i + kstat_irqs_offset)
            irq_stats_str = ''

            for j in range(0, cpus):
                if per_cpu_offset_addr is None:
                    offset = 0
                else:
                    offset = ram_dump.read_word(per_cpu_offset_addr + 4 * j)
                irq_statsn = ram_dump.read_word(kstat_irqs_addr + offset)
                irq_stats_str = irq_stats_str + \
                    '{0:10} '.format('{0}'.format(irq_statsn))

            chip = ram_dump.read_word(
                irq_desc + i + irq_data_offset + irq_chip_offset)
            chip_name_addr = ram_dump.read_word(chip + chip_name_offset)
            chip_name = ram_dump.read_cstring(chip_name_addr, 48)

            if action != 0:
                name_addr = ram_dump.read_word(action + action_name_offset)
                name = ram_dump.read_cstring(name_addr, 48)
                str = '{0:4} {1:12} {2:10} {3} {4:30} {5:10}'
                print_out_str(
                    str.format(irqnum, hex(hwirq), hex(affinity),
                    irq_stats_str, name, chip_name))

    def shift_to_maxindex(self, shift):
        radix_tree_map_shift = 6
        radix_tree_map_size = 1 << radix_tree_map_shift
        return (radix_tree_map_size << shift) - 1

    def is_internal_node(self, addr):
        radix_tree_entry_mask = 0x3
        if self.ramdump.kernel_version > (4, 20, 0):
            radix_tree_internal_node = 0x2
        else:
            radix_tree_internal_node = 0x1
        return (addr & radix_tree_entry_mask) == radix_tree_internal_node

    def entry_to_node(self, addr):
        if self.ramdump.kernel_version > (4, 20, 0):
            return addr & 0xfffffffffffffffd
        else:
            return addr & 0xfffffffffffffffe

    def radix_tree_lookup_element_v2(self, ram_dump, root_addr, index):
        if self.ramdump.kernel_version > (4, 20,0 ):
            rnode_offset = ram_dump.field_offset('struct xarray', 'xa_head')
            rnode_shift_offset = ram_dump.field_offset('struct xa_node', 'shift')
            slots_offset = ram_dump.field_offset('struct xa_node', 'slots')
            pointer_size = ram_dump.sizeof('struct xa_node *')
        else:
            rnode_offset = ram_dump.field_offset('struct radix_tree_root', 'rnode')
            rnode_shift_offset = ram_dump.field_offset('struct radix_tree_node', 'shift')
            slots_offset = ram_dump.field_offset('struct radix_tree_node', 'slots')
            pointer_size = ram_dump.sizeof('struct radix_tree_node *')

        # if CONFIG_BASE_SMALL=0: radix_tree_map_shift = 6
        maxindex = 0
        radix_tree_map_shift = 6
        radix_tree_map_mask = 0x3f

        rnode_addr = ram_dump.read_word(root_addr + rnode_offset)
        if self.is_internal_node(rnode_addr):
            node_addr = self.entry_to_node(rnode_addr)
            shift = ram_dump.read_byte(node_addr + rnode_shift_offset)
            maxindex = self.shift_to_maxindex(shift)

        if index > maxindex:
            return None

        while self.is_internal_node(rnode_addr):
            parent_addr = self.entry_to_node(rnode_addr)
            parent_shift = ram_dump.read_byte(parent_addr + rnode_shift_offset)
            offset = (index >> parent_shift) & radix_tree_map_mask
            rnode_addr = ram_dump.read_word(parent_addr + slots_offset +
                (offset * pointer_size))

        if rnode_addr == 0:
            return None

        return rnode_addr

    def radix_tree_lookup_element(self, ram_dump, root_addr, index):
        rnode_offset = ram_dump.field_offset('struct radix_tree_root', 'rnode')
        if (ram_dump.kernel_version[0], ram_dump.kernel_version[1]) >= (3, 18):
            rnode_height_offset = ram_dump.field_offset(
                'struct radix_tree_node', 'path')
        else:
            rnode_height_offset = ram_dump.field_offset(
                'struct radix_tree_node', 'height')
        slots_offset = ram_dump.field_offset('struct radix_tree_node', 'slots')
        pointer_size = ram_dump.sizeof('struct radix_tree_node *')

        # if CONFIG_BASE_SMALL=0: radix_tree_map_shift = 6
        radix_tree_map_shift = 6
        radix_tree_map_mask = 0x3f
        radix_tree_height_mask = 0xfff
        height_to_maxindex = [0x0, 0x3F, 0x0FFF,
                              0x0003FFFF, 0x00FFFFFF, 0x3FFFFFFF, 0xFFFFFFFF]

        if ram_dump.read_word(root_addr + rnode_offset) & 1 == 0:
            if index > 0:
                return None
            return (ram_dump.read_word(root_addr + rnode_offset) & 0xfffffffffffffffe)

        node_addr = ram_dump.read_word(root_addr + rnode_offset) & 0xfffffffffffffffe
        height = ram_dump.read_int(node_addr + rnode_height_offset)

        if (ram_dump.kernel_version[0], ram_dump.kernel_version[1]) >= (3, 18):
            height = height & radix_tree_height_mask

        if height > len(height_to_maxindex):
            return None

        if index > height_to_maxindex[height]:
            return None

        shift = (height - 1) * radix_tree_map_shift
        for h in range(height, 0, -1):
            node_addr = ram_dump.read_word(
                node_addr + slots_offset + ((index >> shift) & radix_tree_map_mask) * pointer_size)
            if node_addr == 0:
                return None
            shift -= radix_tree_map_shift
        return (node_addr & 0xfffffffffffffffe)

    def print_irq_state_sparse_irq(self, ram_dump):
        h_irq_offset = ram_dump.field_offset('struct irq_desc', 'handle_irq')
        irq_num_offset = ram_dump.field_offset('struct irq_data', 'irq')
        hwirq_num_offset = ram_dump.field_offset('struct irq_data', 'hwirq')
        if ram_dump.kernel_version >= (4,4,0):
            affinity_offset = ram_dump.field_offset(
                'struct irq_common_data', 'affinity')
            irq_common_data_offset = ram_dump.field_offset(
                'struct irq_desc', 'irq_common_data')
        else:
            affinity_offset = ram_dump.field_offset(
                'struct irq_data', 'affinity')
        irq_data_offset = ram_dump.field_offset('struct irq_desc', 'irq_data')
        irq_count_offset = ram_dump.field_offset(
            'struct irq_desc', 'irq_count')
        irq_chip_offset = ram_dump.field_offset('struct irq_data', 'chip')
        irq_action_offset = ram_dump.field_offset('struct irq_desc', 'action')
        action_name_offset = ram_dump.field_offset('struct irqaction', 'name')
        kstat_irqs_offset = ram_dump.field_offset(
            'struct irq_desc', 'kstat_irqs')
        chip_name_offset = ram_dump.field_offset('struct irq_chip', 'name')
        cpu_str = ''

        irq_desc_tree = ram_dump.address_of('irq_desc_tree')
        nr_irqs = ram_dump.read_int(ram_dump.address_of('nr_irqs'))

        for i in ram_dump.iter_cpus():
            cpu_str = cpu_str + '{0:10} '.format('CPU{0}'.format(i))

        print_out_str('{0:4} {1:12} {2:10} {3} {4:30} {5:15} {6:20}'\
            .format('IRQ', 'HWIRQ', 'affinity', cpu_str, 
                'Name', 'Chip', 'IRQ Structure'))

        if nr_irqs > 50000:
            return

        major, minor, patch = ram_dump.kernel_version
        for i in range(0, nr_irqs):

            if (major, minor) >= (4, 9):
                irq_desc = self.radix_tree_lookup_element_v2(
                    ram_dump, irq_desc_tree, i)
            else:
                irq_desc = self.radix_tree_lookup_element(
                    ram_dump, irq_desc_tree, i)

            if irq_desc is None:
                continue

            irqnum = ram_dump.read_int(irq_desc + irq_data_offset + irq_num_offset)
            hwirq = ram_dump.read_int(
                irq_desc + irq_data_offset + hwirq_num_offset)
            if ram_dump.kernel_version >= (4,4,0):
                affinity = ram_dump.read_int(
                    irq_desc + irq_common_data_offset + affinity_offset)
            else:
                affinity = ram_dump.read_int(
                        irq_desc + affinity_offset)
            irqcount = ram_dump.read_int(irq_desc + irq_count_offset)
            action = ram_dump.read_word(irq_desc + irq_action_offset)
            kstat_irqs_addr = ram_dump.read_word(irq_desc + kstat_irqs_offset)
            irq_stats_str = ''

            if kstat_irqs_addr is None:
                break

            for j in ram_dump.iter_cpus():
                irq_statsn = ram_dump.read_int(kstat_irqs_addr, cpu=j)
                irq_stats_str = irq_stats_str + \
                    '{0:10} '.format('{0}'.format(irq_statsn))

            chip = ram_dump.read_word(
                irq_desc + irq_data_offset + irq_chip_offset)
            chip_name_addr = ram_dump.read_word(chip + chip_name_offset)
            chip_name = ram_dump.read_cstring(chip_name_addr, 48)

            if action != 0:
                name_addr = ram_dump.read_word(action + action_name_offset)
                if not name_addr:
                    name = "None"
                else:
                    name = ram_dump.read_cstring(name_addr, 48)
                str = "{0:4} {1:12} {2:10} {3} {4:30} {5:15} " \
                       "v.v (struct irq_desc *)0x{6:<20x}"
                print_out_str(
                    str.format(irqnum, hex(hwirq), hex(affinity),
                    irq_stats_str, name, chip_name, irq_desc))

    def parse(self):
        irq_desc = self.ramdump.address_of('irq_desc')
        if self.ramdump.is_config_defined('CONFIG_SPARSE_IRQ'):
            self.print_irq_state_sparse_irq(self.ramdump)

        if irq_desc is None:
            return

        self.print_irq_state_3_0(self.ramdump)
