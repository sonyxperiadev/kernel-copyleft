# Copyright (c) 2012,2014-2015,2017-2020 The Linux Foundation. All rights reserved.
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
from mm import pfn_to_page, page_buddy, page_count, for_each_pfn
from mm import page_to_pfn
import sys

files = {
    '' : {
            'filename' : 'unknown',
            'filepath' : 'unknown',
            'a_ops' : 'unknown',
            'addr_space' : 'unknown',
            'total_pages' : 0,
            'total_size' : 0,
        }
    }

@register_parser('--print-filetracking', 'print file tracking information (if available)')
class FileTracking(RamParser):

    def get_filepath(self, path, name, parent):
        cycle_flag = 0
        cycle_detection = 12
        while True:
            if name == '/' or name == '':
                return path, cycle_flag
            path = '/' + name + path
            parent = self.ramdump.read_structure_field(parent, 'struct dentry', 'd_parent')
            if parent is not None:
                d_name_ptr = (parent + self.ramdump.field_offset('struct dentry ', 'd_name')) + \
                        self.ramdump.field_offset('struct qstr', 'name')
                test_name = self.ramdump.read_cstring(self.ramdump.read_pointer(d_name_ptr), 100)
                if  test_name == name and name.find('/') == -1 and test_name.find('/') == -1:
                    cycle_flag = 2
                    break
                else:
                    if name.find('/') != -1 and test_name.find('/') != -1:
                        break
                    name = test_name
            else:
                break
            cycle_detection -= 1

            if cycle_detection < 1:
                cycle_flag = 1
                break

        if path[1] == '/':
             path = path[2:]

        return path, cycle_flag

    def get_file_metadata(self, page):
        addr_space = inode = dentry_list = dentry = a_ops = None
        name = path = ''
        if page is not None:
            addr_space = self.ramdump.read_structure_field(page, 'struct page', 'mapping')
        if addr_space is not None:
            inode = self.ramdump.read_structure_field(addr_space, 'struct address_space', 'host')
            a_ops = self.ramdump.read_structure_field(addr_space, 'struct address_space ', 'a_ops')
        else:
            return '', '', None, None, 0
        if inode is not None:
            dentry_list = self.ramdump.read_structure_field(inode, 'struct inode', 'i_dentry')
        if dentry_list is not None:
            dentry = self.ramdump.container_of(dentry_list, 'struct dentry', 'd_u')
        if dentry is not None:
            d_name_ptr = (dentry + self.ramdump.field_offset('struct dentry ', 'd_name')) + \
                    self.ramdump.field_offset('struct qstr', 'name')
            name = self.ramdump.read_cstring(self.ramdump.read_pointer(d_name_ptr), 100)
        else:
            return '', '', None, None, 0
        if name is None:
            return '', '', None, None, 0

        path, cycle_flag = self.get_filepath(path, name, dentry)

        return name, path, a_ops, addr_space, cycle_flag

    def update_file_list(self, name, path, a_ops, addr_space, cycle_flag):
        if addr_space not in files:
            files[addr_space] = {}
            files[addr_space]['total_pages'] = 0
            files[addr_space]['total_size'] = 0
        if 'filename' not in files[addr_space]:
            files[addr_space]['filename'] = name
            if cycle_flag == 1:
                files[addr_space]['filepath'] = 'PATH CYCLE DETECTED: ' + path
            elif cycle_flag == 2:
                files[addr_space]['filepath'] = 'PARENT == CURRENT FILE: ' + path
            else:
                files[addr_space]['filepath'] = path
            files[addr_space]['a_ops'] = (self.ramdump.unwind_lookup(a_ops))[0]
            files[addr_space]['addr_space'] = addr_space
        files[addr_space]['total_pages'] += 1
        files[addr_space]['total_size'] += 4

    def parse(self):
        ranges = None
        g_optimization = False
        for arg in sys.argv:
            if "ranges=" in arg:
                g_optimization = True
                k, ranges = arg.split("=")
                start, end = ranges.split('-')
                start_pfn = int(start, 16) >> 12
                end_pfn = int(end, 16) >> 12
                break
            elif "page=" in arg:
                g_optimization = True
                k, page = arg.split('=')
                page = int(page, 16)
                start_pfn = page_to_pfn(self.ramdump, page)
                end_pfn = start_pfn + 1
                break

        out_tracking = self.ramdump.open_file('file_tracking.txt')

        if g_optimization:
            ranges = range(start_pfn, end_pfn)
        else:
            ranges = for_each_pfn(self.ramdump)

        for pfn in ranges:
            page = pfn_to_page(self.ramdump, pfn)
            if (page_buddy(self.ramdump, page) or \
                page_count(self.ramdump, page) == 0):
                continue
            name, path, a_ops, addr_space, cycle_flag = self.get_file_metadata(page)
            if a_ops is None:
                continue
            self.update_file_list(name, path, a_ops, addr_space, cycle_flag)

        total_sizes = {}
        file_list = sorted(files, key=lambda x: (files[x]['total_size']), reverse=True)
        for file in range(len(file_list)):
            if files[file_list[file]]['filename'] == 'unknown':
                continue
            name = files[file_list[file]]['filename']
            path = files[file_list[file]]['filepath']
            a_ops = files[file_list[file]]['a_ops']
            addr_space = files[file_list[file]]['addr_space']
            pages = files[file_list[file]]['total_pages']
            size = files[file_list[file]]['total_size']

            out_str =  'File       : {0}\nPath       : {1}\na_ops      : {2}\nAddr Space : 0x{3:x}\nNo. Pages  : {4}\nSize (KB)  : {5}\n\n\n'
            out_tracking.write(out_str.format(name, path, a_ops, addr_space, pages, size))
            if a_ops not in total_sizes:
                total_sizes[a_ops] = 0
            total_sizes[a_ops] += size

        out_tracking.write('-----------= Total Sizes (KB) =-----------\n\n')
        total_sizes_list = sorted(total_sizes, key=lambda x: (total_sizes[x]), reverse=True)
        for aops in range(len(total_sizes_list)):
            out_str =  '{0} : {1}\n'
            out_tracking.write(out_str.format(total_sizes_list[aops], total_sizes[total_sizes_list[aops]]))
        out_tracking.close()
        print_out_str('---wrote file tracking information to file_tracking.txt')
