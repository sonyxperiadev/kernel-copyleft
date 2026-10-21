# Copyright (c) 2020-2021 The Linux Foundation. All rights reserved.
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

import sys
import os
import string
import struct

outputfile = "ap_minidump.elf"


def add_file(fo, path, delete):
    try:
        fi = open_file(path, "rb")
    except:
        print("Not able to open file %s" % (path))
        return -1
    while True:
        buf = fi.read(4096)
        if len(buf) == 0:
            break
        fo.write(buf)
    fi.close()
    if delete:
        os.unlink(os.path.join(out_folder, path))
    return 0


def open_file(name, op):
    path = os.path.join(out_folder, name)
    out_file = open(path, op)
    return out_file


def get_strings(buf, length):
    offset = 0
    string = ""
    nlist = []
    begin = False
    str_tbl = ""
    while offset < length:
        str_tbl = ""
        if not begin:
            (ch,) = struct.unpack_from("<B", buf, offset)
            str_tbl += chr(ch)
            (ch,) = struct.unpack_from("<B", buf, offset+1)
            str_tbl += chr(ch)
            (ch,) = struct.unpack_from("<B", buf, offset+2)
            str_tbl += chr(ch)
            (ch,) = struct.unpack_from("<B", buf, offset+3)
            str_tbl += chr(ch)
            (ch,) = struct.unpack_from("<B", buf, offset+4)
            str_tbl += chr(ch)
            (ch,) = struct.unpack_from("<B", buf, offset+5)
            str_tbl += chr(ch)
            (ch,) = struct.unpack_from("<B", buf, offset+6)
            str_tbl += chr(ch)
            if str_tbl == "STR_TBL":
                begin = True
                offset += 6
            else:
                offset += 1
            continue
        (ch,) = struct.unpack_from("<B", buf, offset)
        offset += 1
        if (ch >= 0x30 and ch < 0x80) or ch==0x2e:
            string += chr(ch)
        elif (string != "" and len(string) >= 3 and
              string != 'linux_banner' and string != 'minidump_table'):
            nlist.append(string)
            string = ""
        else:
            string = ""
        if ch == 0:
            (ch1,) = struct.unpack_from("<B", buf, offset+1)
            if ch1 == 0:
                (ch2,) = struct.unpack_from("<B", buf, offset+2)
                if ch2 == 0:
                    return nlist
    return nlist


def generate_elf():
    delete = False
    elfhd = os.path.join(out_folder, "md_KELF_HEADER.BIN")
    elfhd_new = os.path.join(out_folder, "md_KELF_HDR.BIN")
    fo = open_file(outputfile, "wb")
    if os.path.exists(elfhd):
        fi = open_file("md_KELF_HEADER.BIN", "rb")
        hsize = os.path.getsize(elfhd)
    elif os.path.exists(elfhd_new):
        fi = open_file("md_KELF_HDR.BIN", "rb")
        hsize = os.path.getsize(elfhd_new)
    else:
        print("ELF header binary is missing")
        sys.exit(2)
    print("ELF header size %d" % hsize)
    buf = fi.read(hsize)
    fo.write(buf)
    nlist = get_strings(buf, len(buf))
    for names in nlist:
        filepath = "md_" + names + ".BIN"
#           print ("%s size %x\n" % (filepath, os.path.getsize(filepath)))
        ret = add_file(fo, filepath, delete)
        if ret == -1:
            print("Exting!!")
            fo.close()
            fi.close()
            os.unlink(os.path.join(out_folder, outputfile))
            return
    fi.close()
    fo.close()


def help():
    print("\nThis script generates ap_minidump.elf from QPST dump for LDRP\n")
    print("\nProvide QPST minidump location\n")


if __name__ == '__main__':
    args = sys.argv
    if len(args) > 1:
        out_folder = args[1]
    else:
        help()
        sys.exit(2)

    print("using outfodler %s" % out_folder)
    generate_elf()
