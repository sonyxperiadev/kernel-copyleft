# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.

from parser_util import register_parser, RamParser, cleanupString


class memory_area:
    def __init__(self, name, base, size):
        self.name = name
        self.base = base
        self.size = size

@register_parser('--print-memory_map', 'Print memory_map information')
class memory_map(RamParser):

    def get_reserved_mem(self, ramdump, list_memory_area):
        reserved_mem_addr = ramdump.address_of('reserved_mem')
        reserved_mem_count_addr = ramdump.address_of('reserved_mem_count')
        reserved_mem_count = ramdump.read_int(reserved_mem_count_addr)
        base_offset = ramdump.field_offset('struct reserved_mem', 'base')
        size_offset = ramdump.field_offset('struct reserved_mem', 'size')

        for i in range(0, reserved_mem_count):
            addr_index = ramdump.array_index(reserved_mem_addr, 'struct reserved_mem', i)
            name = ramdump.read_structure_cstring(addr_index, 'struct reserved_mem', 'name')
            if  ramdump.arm64:
                base = ramdump.read_u64(addr_index + base_offset)
                size = ramdump.read_word(addr_index + size_offset)
            else:
                base = ramdump.read_u32(addr_index + base_offset)
                size = ramdump.read_u32(addr_index + size_offset)
            memory_area_instance = memory_area(name, base, size)
            list_memory_area.append(memory_area_instance)

        list_memory_area.sort(key=lambda c: c.base)

    def get_kernel_resource(self, list_memory_area):
        mem_res_mem_addr = self.ramdump.address_of('mem_res')
        start_offset = self.ramdump.field_offset('struct resource', 'start')
        end_offset = self.ramdump.field_offset('struct resource', 'end')
        start = 0
        end = 0
        for i in range(0, 2):
            mem_res_mem_addr_index = self.ramdump.array_index(mem_res_mem_addr, 'struct resource', i)
            name = self.ramdump.read_structure_cstring(mem_res_mem_addr_index, 'struct resource', 'name')
            if self.ramdump.arm64:
                start = self.ramdump.read_u64(mem_res_mem_addr_index + start_offset)
                end = self.ramdump.read_u64(mem_res_mem_addr_index + end_offset)
            else:
                start = self.ramdump.read_u32(mem_res_mem_addr_index + start_offset)
                end = self.ramdump.read_u32(mem_res_mem_addr_index + end_offset)

            memory_area_instance = memory_area(name, start, end - start)
            list_memory_area.append(memory_area_instance)

    def get_cma_areas(self, ramdump, list_memory_area):
        cma_area_count = ramdump.read_u32('cma_area_count')
        cma_area_base_addr = ramdump.address_of('cma_areas')
        for cma_index in range(0, cma_area_count):
            cma_area = ramdump.array_index(cma_area_base_addr, 'struct cma', cma_index)
            base_pfn = ramdump.read_structure_field(
                cma_area, 'struct cma', 'base_pfn')
            cma_size = ramdump.read_structure_field(
                cma_area, 'struct cma', 'count')
            if (ramdump.kernel_version >= (5, 10, 0)):
                name_addr_offset = ramdump.field_offset('struct cma', 'name')
                name_addr = (cma_area + name_addr_offset)
                name = ramdump.read_cstring(name_addr, 64)
            else:
                name_addr = ramdump.read_structure_field(cma_area, 'struct cma', 'name')
                name = ramdump.read_cstring(name_addr, 48)
            memory_area_instance = memory_area(name, base_pfn << 12, cma_size << 12)

            if any(x.base == memory_area_instance.base for x in list_memory_area) == False:
                list_memory_area.append(memory_area_instance)


    def get_memory_block(self, ramdump, list_memory_area):
        cnt = ramdump.read_structure_field('memblock', 'struct memblock', 'memory.cnt')
        total_size = ramdump.read_structure_field('memblock', 'struct memblock', 'memory.total_size')
        region = ramdump.read_structure_field('memblock', 'struct memblock', 'memory.regions')

        for i in range(cnt):
            start = ramdump.read_structure_field(region, 'struct memblock_region', 'base')
            size = ramdump.read_structure_field(region, 'struct memblock_region', 'size')
            memory_area_instance = memory_area("memory", start, size)
            if any(x.base == memory_area_instance.base for x in list_memory_area) == False:
                list_memory_area.append(memory_area_instance)

        cnt = ramdump.read_structure_field('memblock', 'struct memblock', 'reserved.cnt')
        region = ramdump.read_structure_field('memblock', 'struct memblock', 'reserved.regions')
        for i in range(cnt):
            start = ramdump.read_structure_field(region, 'struct memblock_region', 'base')
            size = ramdump.read_structure_field(region, 'struct memblock_region', 'size')
            memory_area_instance = memory_area("reserved", start, size)
            if any(x.base == memory_area_instance.base for x in list_memory_area) == False:
                list_memory_area.append(memory_area_instance)


    def get_iomem_resource(self, ramdump, list_memory_area):
        iomem_resource_addr = ramdump.address_of('iomem_resource')
        iomem_resource_start = iomem_resource_addr

        start = ramdump.read_structure_field(iomem_resource_start, 'struct resource', 'start')
        end = ramdump.read_structure_field(iomem_resource_start, 'struct resource', 'end')
        offset_name = ramdump.field_offset('struct resource', 'name')
        sibling = ramdump.read_structure_field(iomem_resource_start, 'struct resource', 'sibling')
        child = ramdump.read_structure_field(iomem_resource_start, 'struct resource', 'child')
        name_address = ramdump.read_pointer(iomem_resource_start + offset_name)
        name = cleanupString(ramdump.read_cstring(name_address, 16))
        iomem_resource_start = child

        while iomem_resource_start != 0:
            start = ramdump.read_structure_field(iomem_resource_start, 'struct resource', 'start')
            end = ramdump.read_structure_field(iomem_resource_start, 'struct resource', 'end')
            offset_name = ramdump.field_offset('struct resource', 'name')
            sibling = ramdump.read_structure_field(iomem_resource_start, 'struct resource', 'sibling')
            child = ramdump.read_structure_field(iomem_resource_start, 'struct resource', 'child')
            name_address = ramdump.read_pointer(iomem_resource_start + offset_name)
            name = cleanupString(ramdump.read_cstring(name_address))
            memory_area_instance = memory_area(name, start, end - start)
            list_memory_area.append(memory_area_instance)
            if sibling == 0:
                break
            iomem_resource_start = sibling

    def parse(self):
        list_memory_area = []
        self.get_reserved_mem(self.ramdump, list_memory_area)
        self.get_cma_areas(self.ramdump, list_memory_area)
        self.get_kernel_resource(list_memory_area)
        self.get_memory_block(self.ramdump, list_memory_area)
        self.get_iomem_resource(self.ramdump, list_memory_area)
        fmap = open(self.ramdump.outdir + "/memory_map.txt", "w")
        print("name											                  base 				end 				size 						  size in KB\n", file = fmap)
        new_list = sorted(list_memory_area, key=lambda c: c.base, reverse=False)
        for i in range(len(new_list)):
            memory_area_instance = new_list[i]
            print("----------------------------------------------------------------------------------------------------------------------------------------------",
                file=fmap)
            print("%-64s 0x%-16x 0x%-16x 0x%-16x  %16d" % (memory_area_instance.name,
                                                           memory_area_instance.base,
                                                           memory_area_instance.base + memory_area_instance.size,
                                                           memory_area_instance.size, memory_area_instance.size / 1024),
                  file=fmap)
        fmap.close()
