# Copyright (c) 2021 The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import sys
import os
import getopt
import re
import string
import struct


class minidump():

    out_folder = '.'
    out_file = None

    def print_output(self, string):
        self.out_file.write(string + "\n")

    def open_file(self, name, op):
        path = os.path.join(self.out_folder, name)
        self.out_file = open(path, op)
        return self.out_file

    def split_rawdump(self, rawdump, option='file'):

        head = "<8sIIQ8sIQQI";
        head_size = struct.calcsize(head)

        section_head = "<IIIQQQQ20s";
        section_head_size = struct.calcsize(section_head)
        print("headsize:%d %d" % (head_size, section_head_size))

        filename = os.path.join("", rawdump)
        if not os.path.exists(filename):
            print ("rawdump not exist")
            return False
        f = open(filename, "rb")
        head_buf = f.read(head_size)
        (sig, version, valid, data, context, reset_trigger, dump_size, total_size, sections_count) = struct.unpack_from(head, head_buf, 0)
        if sig != b'Raw_Dmp!':
            print ("rawdump signal is not Raw_Dmp!")
            return False
        if valid != 1:
            if valid == 2:
                print ("Ramdump not complete: no space was left on device during the dump operation.")
                print ("This usually happens when a full dump is set to be written to a rawdump partion.")
                print ("Please check if the value of /sys/kernel/dload/dload_mode is set to 'mini'.")
            else:
                print ("Valid tag not set!")
            return False
        count = 0
        section_list = []
        file_list ={}

        while count < sections_count:
            section_buf = f.read(section_head_size)
            section_list.append(section_buf)
            count += 1

        if option == 'list':
            print ("File list in rawdump:")
            print("name, section_offset, paddr, section_size, section_type, valid, version")
        elif option == 'split':
            print("Split rawdump %s to seperate file to %s ..." % (rawdump, self.out_folder))

        print("sig:\t%s\nversion:\t%d\nvalid:\t%d\ndata:\t0x%lx\ncontext:\t%s\nreset_trigger:\t%x\ndump_size:\t0x%lx\nTotal Size:\t0x%lx\nSection Count:\t%d\n"

             % (sig, version, valid, data, context, reset_trigger, dump_size, total_size, sections_count))
        for i, one_section in enumerate(section_list):
            valid, version, section_type, section_offset, section_size, paddr, info, name = struct.unpack_from(section_head, one_section, 0)
            name = name.decode().strip('\0')
            file_list[name] = {}
            file_list[name]['size'] = section_size
            file_list[name]['paddr'] = paddr
            if name == 'load.cmm':
                f.seek(section_offset)
                cmmbuf = f.read(section_size)

            if option == 'list':
                print("%s\t0x%x\t0x%x\t0x%x\t%d %d %d " % (name, section_offset, paddr, section_size, section_type, valid, version ))
                continue

            if option == 'split' or name.startswith('md_K'):
                print ("Writing %s @0x%x len:0x%x ..." % (name, section_offset, section_size))
                fout = self.open_file(name, "wb")
                f.seek(section_offset)
                allread = 0
                block_size = 0x100000 * 10
                left = section_size
                size = block_size if left > block_size else left
                while left > 0:
                    buf = f.read(size)
                    fout.write(buf)
                    allread += size
                    left -= size
                    if allread % (block_size * 20) == 0:
                        print(("%d MB ") % (allread / 1024 / 1024))
                    size = block_size if left > block_size else left
                fout.close()

        f.close()

        #   filename = os.path.join(out_folder, "load.cmm")
        #       with open(filename) as f:
        for line in cmmbuf.decode().split('\n'):
            m = re.search("d.load.binary\s+(\S+)\s+([0-9a-fA-FxX]+)", line)
            if m:
                filename = m.group(1)
                base = int(m.group(2), 16)
                file_list[filename]['base'] = base

        if option != 'list':
            f = self.open_file("dump_info.txt", "w")
        else:
            return

        for fname,bs in iter(file_list.items()):
            if 'base' in bs.keys():
                base = bs['base']
            elif 'paddr' in bs.keys():
                base = bs['paddr']
            else:
                base = 0
            f.write("1 0x{:0>16x}\t{:0>16}\t{:<20}\t{}\r\n".format( base, bs['size'], fname, fname))
        if option != 'list':
            f.close()
        return True

    def __init__(self, out_folder):
        if not os.path.isdir(out_folder):
            os.mkdir(out_folder)
        self.out_folder = out_folder

def help():
        print (" minidump tool v1.1")
        print ("%s [-s|-l|-c|-d|-m|-k] <rawdump> [out_folder] " % sys.argv[0])
        print (" -l  list files in sbldump.bin")
        print (" -s  split to seperate files")

if __name__=='__main__':
    try:
        opts, args = getopt.gnu_getopt(sys.argv[1:], "ls")
    except getopt.GetoptError:
        print ("please check usage")
        help()
        sys.exit(-1)

    if len(args) > 0:
        rawdump = args[0]
        if len(args) > 1:
            out_folder = args[1]
        else:
            out_folder = "."
    else:
        help()
        sys.exit(-1)

    mdump = minidump(out_folder)
    option = 'file'
    for opt, val in opts:
        if opt=="-l":
            option = 'list'
            break;
        if opt=="-s":
            option = 'split'
            break;
    mdump.split_rawdump(rawdump, option)
