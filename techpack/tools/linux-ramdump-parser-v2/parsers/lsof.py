"""
Copyright (c) 2016, 2020 The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

from parser_util import register_parser, RamParser
from parser_util import cleanupString

TASK_NAME_LENGTH = 16


def do_dump_lsof_info(self, ramdump, lsof_info):
    for task_struct in ramdump.for_each_process():
        parse_task(self, ramdump, task_struct, lsof_info)
        lsof_info.write("\n*********************************")

def get_dname_of_dentry(self, dentry):
    ramdump = self.ramdump
    dentry_name_offset = ramdump.field_offset(
        'struct dentry', 'd_name')
    len_offset = ramdump.field_offset(
        'struct qstr', 'len')
    qst_name_offset = ramdump.field_offset(
        'struct qstr', 'name')

    name_address = ramdump.read_word(dentry + dentry_name_offset + qst_name_offset)
    len_address = dentry + dentry_name_offset + len_offset
    len = ramdump.read_u32(len_address)
    name = cleanupString(ramdump.read_cstring(
        name_address, len))
    return name

def get_pathname_by_file(self, ramdump, file):
    f_pathoffset = ramdump.field_offset(
        'struct file', 'f_path')
    f_path = f_pathoffset + file

    mnt_offset_in_path = ramdump.field_offset('struct path', 'mnt')
    mnt = ramdump.read_word(f_path + mnt_offset_in_path)
    mnt_offset_in_mount = self.ramdump.field_offset(
        'struct mount', 'mnt')
    mnt_parent_offset = ramdump.field_offset('struct mount', 'mnt_parent')
    mount = mnt - mnt_offset_in_mount
    mnt_mountpoint_offset = self.ramdump.field_offset(
        'struct mount', 'mnt_mountpoint')
    mnt_parent_pre = 0
    mnt_parent = mount
    mount_name = []
    while  mnt_parent_pre !=  mnt_parent:
        mnt_parent_pre = mnt_parent
        mnt_mountpoint = ramdump.read_word(mnt_parent +  mnt_mountpoint_offset)
        name = get_dname_of_dentry(self, mnt_mountpoint)
        mnt_parent = ramdump.read_word(mnt_parent + mnt_parent_offset)
        if name == '/':
            break
        mount_name.append(name)

    dentry = ramdump.read_structure_field(
        f_path, 'struct path', 'dentry')
    d_parent_offset = ramdump.field_offset(
        'struct dentry', 'd_parent')
    d_parent = dentry
    d_parent_pre = 0
    names = []
    while d_parent_pre != d_parent:
        d_parent_pre = d_parent
        name = get_dname_of_dentry(self, d_parent)
        d_parent = ramdump.read_word(d_parent + d_parent_offset)
        if name == '/':
            break
        names.append(name)
    full_name = ''
    for item in mount_name:
        names.append(item)
    names.reverse()
    for item in names:
        full_name += '/' + item
    return full_name

def parse_task(self, ramdump, task, lsof_info):
    index = 0
    if self.ramdump.arm64:
        addressspace = 8
    else:
        addressspace = 4

    task_comm_offset = ramdump.field_offset(
                        'struct task_struct',  'comm')
    task_comm_offset = task + task_comm_offset
    client_name = ramdump.read_cstring(
                    task_comm_offset, TASK_NAME_LENGTH)
    task_pid = ramdump.read_structure_field(
                    task, 'struct task_struct', 'pid')
    files = ramdump.read_structure_field(
                    task, 'struct task_struct', 'files')
    if files == 0x0:
        return

    str_task_file = '\n Task: 0x{0:x}, comm: {1}, pid : {2:1}, files : 0x{3:x}'
    lsof_info.write(str_task_file.format(
                    task, client_name, task_pid, files))
    fdt = ramdump.read_structure_field(
                    files, 'struct files_struct', 'fdt')
    max_fds = ramdump.read_structure_field(
                    fdt, 'struct fdtable', 'max_fds')
    fd = ramdump.read_structure_field(
                    fdt, 'struct fdtable', 'fd')
    ion_str = "\n {0:8d} file : 0x{1:16x} {2:32s} {3:32s} client : 0x{4:x}"
    str = "\n {0:8d} file : 0x{1:16x} {2:32s} {3:32s}"

    while index < max_fds:
        file = ramdump.read_word(fd + (index * addressspace))
        if file != 0:
            fop = ramdump.read_structure_field(
                        file, 'struct file', 'f_op')
            priv_data = ramdump.read_structure_field(
                        file, 'struct file', 'private_data')
            look = ramdump.unwind_lookup(fop)
            if look is None:
                index = index + 1
                continue
            fop, offset = look
            iname = get_pathname_by_file(self, self.ramdump, file)
            if fop.find("ion_fops", 0, 8) != -1:
                lsof_info.write(ion_str.format(
                        index, file, fop, iname, priv_data))
            else:
                lsof_info.write(str.format(index, file, fop, iname))
        index = index + 1


@register_parser('--print-lsof',  'Print list of open files',  optional=True)
class DumpLsof(RamParser):

    def parse(self):
        with self.ramdump.open_file('lsof.txt') as lsof_info:
            if (self.ramdump.kernel_version < (3, 18, 0)):
                lsof_info.write('Kernel version 3.18 \
                and above are supported, current version {0}.\
                {1}'.format(self.ramdump.kernel_version[0],
                            self.ramdump.kernel_version[1]))
                return
            do_dump_lsof_info(self, self.ramdump, lsof_info)
