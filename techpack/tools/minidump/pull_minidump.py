# Copyright (c) 2020, The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import subprocess
import os
import string
import struct
import sys


def wait_for_root_access():
    subprocess.call('adb wait-for-devices', shell=True)
    subprocess.call('adb root', shell=True)
    subprocess.call('adb wait-for-devices', shell=True)


def pulldump():

    file_name = out_folder + "sbldump_temp.bin"

    subprocess.call('adb pull /dev/block/bootdevice/by-name/rawdump ' +
                    file_name, shell=True)
    head = "<8sIIQ8sIQQI"
    head_size = struct.calcsize(head)

    section_head = "<IIIQQQQ20s"
    section_head_size = struct.calcsize(section_head)

    if not os.path.exists(file_name):
        print("Failed to pull Minidump from device")
        return False

    f = open(file_name, "rb")
    head_buf = f.read(head_size)
    (sig, version, valid, data, context, reset_trigger, dump_size, total_size,
     sections_count) = struct.unpack_from(head, head_buf, 0)

    if sig != "Raw_Dmp!":
        print("rawdump signal is not Raw_Dmp!")
        return False
    if valid != 1:
        print("Valid tag not set!")
        return False

    print("Dump size %d" % dump_size)

    f.seek(0)
    fo = open(out_folder + "sbldump.bin", "wb")
    tbuf = f.read(dump_size)
    fo.write(tbuf)

    f.close()
    fo.close()
    os.unlink(file_name)

    # clear rawdump partition
    # FIXME: clear only single dump size? if multi dump support is present
    subprocess.call('adb shell "dd if=/dev/zero
                    of=/dev/block/bootdevice/by-name/rawdump count=128
                    bs=1048576"', shell=True)

    subprocess.call('adb shell sync', shell=True)


def help():
    print("\nThis script pulls sbldump.bin Minidump from rawdump partition!!")
    print("\nProvide QPST minidump location\n")

if __name__ == '__main__':
    args = sys.argv
    if len(args) > 1:
        out_folder = args[1]
    else:
        help()
        sys.exit(2)

    print("Using outfodler %s" % out_folder)

    wait_for_root_access()
    pulldump()
