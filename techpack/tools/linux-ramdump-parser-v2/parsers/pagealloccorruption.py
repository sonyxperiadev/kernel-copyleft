# Copyright (c) 2012,2014-2017 The Linux Foundation. All rights reserved.
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
from print_out import print_out_str
from parser_util import register_parser, RamParser
from mm import pfn_to_page, page_buddy , page_address ,get_debug_flags

@register_parser('--print-pagealloccorruption', 'print pagealloc corruption information (if available)')
class PageallocCorruption(RamParser):
    def parse(self):
        if not self.ramdump.is_config_defined('CONFIG_DEBUG_PAGEALLOC'):
           print_out_str('CONFIG_DEBUG_PAGEALLOC Not enabled')
           return
        out_corruption_summary = self.ramdump.open_file('page_corruption_summary.txt')
        out_pfn_ranges = self.ramdump.open_file('page_ranges.txt')
        memblock_addr = self.ramdump.address_of('memblock')
        memblock_memory_offset = self.ramdump.field_offset('struct memblock', 'memory')
        memblock_memory_cnt_offset = self.ramdump.field_offset('struct memblock_type', 'cnt')
        cnt = self.ramdump.read_word(memblock_addr + memblock_memory_offset + memblock_memory_cnt_offset)
        region_offset = self.ramdump.field_offset('struct memblock_type', 'regions')
        regions_baseaddr = self.ramdump.read_word(memblock_addr + memblock_memory_offset + region_offset)
        page_ext_offset = self.ramdump.field_offset(
                                    'struct mem_section', 'page_ext')
        page_flags_offset = self.ramdump.field_offset(
                'struct page_ext', 'flags')
        mem_section_size = self.ramdump.sizeof("struct mem_section")
        mem_section = self.ramdump.read_word('mem_section')
        page_ext_size = self.ramdump.sizeof("struct page_ext")

        for r in range(0,cnt) :
            region_addr = regions_baseaddr + r * self.ramdump.sizeof('struct memblock_region')
            start_addr_offset =  self.ramdump.field_offset('struct memblock_region', 'base')
            start_addr = self.ramdump.read_u32(region_addr + start_addr_offset)
            size_offset = self.ramdump.field_offset('struct memblock_region', 'size')
            region_size =  self.ramdump.read_u32(region_addr + size_offset)
            end_addr = start_addr + region_size
            min_pfn = start_addr >> 12
            max_pfn = end_addr >> 12
            out_pfn_ranges.write("min_pfn : %s,max_pfn: %s\n" %(hex(min_pfn),hex(max_pfn)))

            for pfn in range(min_pfn, max_pfn):
                page = pfn_to_page(self.ramdump, pfn)
                page_pa = (pfn << 12)
                if (self.ramdump.kernel_version > (3, 18, 0)):
                    free = 0
                    offset = page_pa >> 30
                    mem_section_0_offset = (
                        mem_section + (offset * mem_section_size))
                    page_ext = self.ramdump.read_word(
                        mem_section_0_offset + page_ext_offset)
                    temp_page_ext = page_ext + (pfn * page_ext_size)
                    page_ext_flags = self.ramdump.read_word(
                        temp_page_ext + page_flags_offset)
                    # enum PAGE_EXT_DEBUG_POISON ( == 0th bit is set ) for page poisioning
                    free = page_ext_flags & 1
                else:
                    # debug_flags value should be 1 for pages having poisoned value 0xaa
                    free = get_debug_flags(self.ramdump, page)

                if free == 1:
                   flag = 0;
                   for i in range(0,1024):
                       readval = self.ramdump.read_u32(page_pa+i*4, False)
                       if readval == None:
                          break
                       if readval!=0xaaaaaaaa:
                          flag = 1
                          diff = 0xaaaaaaaa-readval
                          if diff < 0:
                             diff = diff * (-1)
                          isBitFlip = not (diff & diff-1)
                          if isBitFlip:
                             out_corruption_summary.write("Single Bit Error at %s" %("%#0.8x"%(page_pa+i*4)))
                             out_corruption_summary.write("\n")
                          else:
                             out_corruption_summary.write("Corruption at %s" %("%#0.8x"%(page_pa+i*4)))
                             out_corruption_summary.write("\n")
                          end_addr = page_pa + i*4 + 0x00000100
                          end_page_addr = page_pa | 0x00000fff
                          if end_addr > end_page_addr:
                             end_addr = end_page_addr
                          count = 0
                          for wordaddr in range(page_pa + i*4,end_addr,0x00000004):
                              if count == 0:
                                 out_corruption_summary.write("%s    " %("%#0.8x"%(wordaddr)))
                              readval = self.ramdump.read_u32(wordaddr, False)
                              out_corruption_summary.write("%s " %("%#0.8x"%(readval)))
                              count = count+1
                              if count == 8:
                                 count = 0
                                 out_corruption_summary.write ("\n");
                          break
                   if flag == 1 :
                          out_corruption_summary.write("\n")

        out_corruption_summary.close()
        out_pfn_ranges.close()


