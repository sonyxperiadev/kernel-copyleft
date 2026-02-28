# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
from parser_util import register_parser, RamParser, cleanupString
from print_out import print_out_str
from utasklib import UTaskLib
from utasklib import ProcessNotFoundExcetion
import linux_list as llist
from collections import namedtuple
import rb_tree

class BaseFs(RamParser):

    def __init__(self, ramdump):
        super().__init__(ramdump)
        self.mnt_tuple = namedtuple("MntTuple", ["mount", "vfsmount", "super_block", "dname", "mpath"])

    def get_dname_of_dentry(self, dentry):
        dentry_name_offset = self.ramdump.field_offset(
            'struct dentry', 'd_name')
        len_offset = self.ramdump.field_offset(
            'struct qstr', 'len')
        qst_name_offset = self.ramdump.field_offset(
            'struct qstr', 'name')

        name_address = self.ramdump.read_word(dentry + dentry_name_offset + qst_name_offset)
        len_address = dentry + dentry_name_offset + len_offset
        len = self.ramdump.read_u32(len_address)
        name = cleanupString(self.ramdump.read_cstring(name_address, len))
        return name

    def get_pathname(self, vfsmount):
        mnt_root = self.ramdump.read_structure_field(vfsmount, 'struct vfsmount', 'mnt_root')

        mnt_offset_in_mount = self.ramdump.field_offset('struct mount', 'mnt')
        mnt_parent_offset = self.ramdump.field_offset('struct mount', 'mnt_parent')
        mount = vfsmount - mnt_offset_in_mount
        mnt_mountpoint_offset = self.ramdump.field_offset(
            'struct mount', 'mnt_mountpoint')
        d_parent_offset = self.ramdump.field_offset(
            'struct dentry', 'd_parent')
        mnt_parent_pre = 0
        mnt_parent = mount
        mount_name = []
        while  mnt_parent_pre != mnt_parent:
            mnt_parent_pre = mnt_parent
            mnt_mountpoint =  self.ramdump.read_word(mnt_parent + mnt_mountpoint_offset)
            name = self.get_dname_of_dentry(mnt_mountpoint)
            mnt_parent = self.ramdump.read_word(mnt_parent + mnt_parent_offset)
            if name == None or name == '/':
                break
            if mnt_parent == 0:
                break
            mount_name.append(name)

            # walk to get the fullname of mountpoint
            d_parent = self.ramdump.read_word(mnt_mountpoint + d_parent_offset)
            d_parent_pre = 0
            while d_parent_pre != d_parent:
                d_parent_pre = d_parent
                name = self.get_dname_of_dentry(d_parent)
                d_parent = self.ramdump.read_word(d_parent + d_parent_offset)
                if name == None or name == '/':
                    break
                mount_name.append(name)
                if d_parent == 0:
                    break

        full_name = ''
        names = []
        for item in mount_name:
            names.append(item)
        names.reverse()
        for item in names:
            full_name += '/' + item
        if len(names) == 0:
            return '/'
        return full_name

    def mnt_rbnodes(self, node, offset):
        mount = node - offset
        self.__show_info(mount)
        return

    def mnt_list(self, mount):
        self.__show_info(mount)
        return

    def parse(self):
        pid = 1
        try:
            args = self.parse_param()
            try:
                pid = int(args["pid"])
            except:
                pid = args["proc"]
        except:
            pid = 1
        finally:
            print_out_str("Dump info from process {}".format(pid))

        try:
            taskinfo = UTaskLib(self.ramdump).get_utask_info(pid)
        except ProcessNotFoundExcetion:
            print_out_str("pid={} process is not started".format(pid))
            return

        nsproxy = self.ramdump.read_structure_field(taskinfo.task_addr, 'struct task_struct', 'nsproxy')
        mnt_ns = self.ramdump.read_structure_field(nsproxy, 'struct nsproxy', 'mnt_ns')
        self.output.write(f"Process: {taskinfo.name}, (struct task_struct*)=0x{taskinfo.task_addr:x} \
                          (struct nsproxy*)=0x{nsproxy:x} (struct mnt_namespace*)=0x{mnt_ns:x}\n\n\n")
        self.print_header()

        mount_list_offset = self.ramdump.field_offset("struct mnt_namespace", 'list')
        if mount_list_offset is None:
            mount_rbnode_addr = mnt_ns + self.ramdump.field_offset("struct mnt_namespace", 'mounts')
            mntnode_offset = self.ramdump.field_offset("struct mount", 'mnt_node')
            rb_node = self.ramdump.read_pointer(mount_rbnode_addr)
            rb_walker = rb_tree.RbTreeWalker(self.ramdump)
            rb_walker.walk(rb_node, self.mnt_rbnodes, mntnode_offset)
        else:
            mount_list_addr = mnt_ns + mount_list_offset
            field_next_offset = self.ramdump.field_offset('struct mount', 'mnt_list')
            list_walker = llist.ListWalker(self.ramdump, mount_list_addr, field_next_offset)
            list_walker.walk(self.mnt_list)

    def print_header(self):
        pass

    def __show_info(self, mount):
        vfsmount = mount + self.ramdump.field_offset('struct mount', 'mnt')
        d_name_addr = self.ramdump.read_word(mount + self.ramdump.field_offset('struct mount', 'mnt_devname'))
        d_name = self.ramdump.read_cstring(d_name_addr, 48)
        if d_name == "rootfs":
            return
        mount_path = self.get_pathname(vfsmount)
        mnt_root = self.ramdump.read_structure_field(vfsmount, 'struct vfsmount', 'mnt_root')
        sb = self.ramdump.read_structure_field(mnt_root, 'struct dentry', 'd_sb')
        mtuple = self.mnt_tuple(mount, vfsmount, sb, d_name,  mount_path)
        self.show_info(mtuple)

@register_parser('--mount', 'Extract mount info logs from ramdump', optional=True)
class Mount(BaseFs):
    def __init__(self, ramdump):
        super().__init__(ramdump)
        self.output = self.ramdump.open_file("mounts.txt")

    def is_anon_ns(self, mnt_namespace):
        return self.ramdump.read_u64(mnt_namespace, "seq") == 0

    def show_type(self, super_block):
        s_type = self.ramdump.read_structure_field(super_block, "struct super_block", 's_type')
        name = self.ramdump.read_structure_field(s_type, "struct file_system_type", 'name')

        type_name = self.ramdump.read_cstring(name, 24)
        s_subtype = self.ramdump.read_structure_field(super_block, "struct super_block", 's_subtype')

        if s_subtype:
            subname = self.ramdump.read_cstring(s_subtype, 24)
            type_name = type_name + "." + subname
        return type_name

    def mnt_is_readonly(self, vfsmount):
        MNT_READONLY =	0x40
        SB_RDONLY = 1
        self.mnt_flags = self.ramdump.read_int(vfsmount + self.ramdump.field_offset("struct vfsmount", 'mnt_flags'))
        mnt_sb = self.ramdump.read_structure_field(vfsmount, "struct vfsmount", 'mnt_sb')
        self.s_flags = self.ramdump.read_word(mnt_sb + self.ramdump.field_offset('struct super_block', 's_flags'))
        return (self.mnt_flags & MNT_READONLY) or (self.s_flags &SB_RDONLY)

    def show_sb_opts(self, super_block):
        SB_SYNCHRONOUS = 1 << 4
        SB_DIRSYNC = 1 << 7
        SB_MANDLOCK = 1 << 6
        SB_LAZYTIME = 1 << 25
        fs_opts= {
                SB_SYNCHRONOUS : ",sync",
                SB_DIRSYNC : ",dirsync",
                SB_MANDLOCK : ",mand",
                SB_LAZYTIME :",lazytime",
                }
        ret = ""
        for flag, flag_str in fs_opts.items():
            if self.s_flags & flag:
                ret = ret + flag_str

        self.output.write(ret)
        self.selinux_sb_show_options(super_block)

    def selinux_superblock(self, super_block):
        s_security = self.ramdump.read_structure_field(super_block, "struct super_block", 's_security')
        selinux_blob_sizes = self.ramdump.address_of('selinux_blob_sizes')
        try:
            ##lbs_superblock not exist on kernel 5.1
            lbs_superblock = self.ramdump.read_int(
                selinux_blob_sizes + self.ramdump.field_offset("struct lsm_blob_sizes", 'lbs_superblock'))
        except:
            lbs_superblock = 0
        return s_security + lbs_superblock

    def selinux_initialized(self):
        selinux_state = self.ramdump.address_of('selinux_state')
        initialized = self.ramdump.read_bool(
            selinux_state + self.ramdump.field_offset("struct selinux_state", 'initialized'))
        return initialized

    def selinux_sb_show_options(self, super_block):
        SE_SBINITIALIZED =	0x0100

        sbsec = self.selinux_superblock(super_block)
        s_flags = self.ramdump.read_u16(sbsec + self.ramdump.field_offset("struct superblock_security_struct", 'flags'))
        if (s_flags & SE_SBINITIALIZED) == 0:
            return
        if not self.selinux_initialized():
            return

        CONTEXT_MNT =	0x01
        FSCONTEXT_MNT =	0x02
        ROOTCONTEXT_MNT =	0x04
        DEFCONTEXT_MNT =	0x08
        SBLABEL_MNT =	0x10

        CONTEXT_STR =	"context"
        FSCONTEXT_STR =	"fscontext"
        ROOTCONTEXT_STR =	"rootcontext"
        DEFCONTEXT_STR =	"defcontext"
        SECLABEL_STR = "seclabel"
        if s_flags & FSCONTEXT_MNT:
            self.output.write("," + FSCONTEXT_STR)
            return
        if s_flags & CONTEXT_MNT:
            self.output.write("," + CONTEXT_STR)
            return
        if s_flags & DEFCONTEXT_MNT:
            self.output.write("," + DEFCONTEXT_STR)
            return
        if s_flags & ROOTCONTEXT_MNT:
            self.output.write("," + ROOTCONTEXT_STR)
            return
        if s_flags & SBLABEL_MNT:
            self.output.write("," + SECLABEL_STR)
            return

    def show_mnt_opts(self, vfsmount):
        MNT_NOSUID =	0x01
        MNT_NODEV =	0x02
        MNT_NOEXEC =	0x04
        MNT_NOATIME =	0x08
        MNT_NODIRATIME =	0x10
        MNT_RELATIME =	0x20
        MNT_READONLY =	0x40
        MNT_NOSYMFOLLOW =	0x80
        mnt_opts = {
            MNT_NOSUID : ",nosuid",
            MNT_NODEV : ",nodev",
            MNT_NOEXEC : ",noexec",
            MNT_NOATIME : ",noatime",
            MNT_NODIRATIME : ",nodiratime",
            MNT_RELATIME : ",relatime",
            MNT_NOSYMFOLLOW : ",nosymfollow",
		}
        ret = ""
        for flag, flag_str in mnt_opts.items():
            if self.mnt_flags & flag:
                ret = ret + flag_str

        if self.is_idmapped_mnt(vfsmount):
            ret = ret + ",idmapped"
        self.output.write(ret)

    def is_idmapped_mnt(self, vfsmount):
        mnt_idmap = self.ramdump.read_structure_field(vfsmount, "struct vfsmount", 'mnt_idmap')
        nop_mnt_idmap = self.ramdump.address_of('nop_mnt_idmap')
        return mnt_idmap != nop_mnt_idmap

    def print_header(self):
        self.output.write("{:<16s} {:<16s} {:<29s} {:<32s} {:<16s} {:<16s}\n".format(
            "(struct mount *)", "(struct super_block *)", "dev_name", "mount_path", "fs_type", "flags"))

    def show_info(self, mtuple):
        mount = mtuple.mount
        vfsmount = mtuple.vfsmount
        sb = mtuple.super_block
        dname = mtuple.dname
        mount_path = mtuple.mpath

        typename = self.show_type(sb)
        self.output.write("0x{:16x} 0x{:16x} {:<32s} {:<32s} {:<16s} ".format(
                                mount, sb, dname, mount_path, typename))
        if self.mnt_is_readonly(vfsmount):
            self.output.write("ro")
        else:
            self.output.write("rw")

        self.show_sb_opts(sb)
        self.show_mnt_opts(vfsmount)
        self.output.write("\n")

@register_parser('--df', 'Extract df info from ramdump', optional=True)
class Df(BaseFs):
    def __init__(self, ramdump):
        super().__init__(ramdump)
        self.output = self.ramdump.open_file("df.txt")
        self.dev_ids = []

    def devid(self, s_dev):
        MINORBITS = 20
        MINORMASK =	((1 << MINORBITS) - 1)
        major = s_dev >> MINORBITS
        minor = s_dev & MINORMASK
        return (minor & 0xff) | (major << 8) | ((minor & ~0xff) << 12)

    def ext4_statfs(self, super_block, s_fs_info):
        sbi = self.ramdump.read_datatype(s_fs_info, 'struct ext4_sb_info',
                                         ["s_es", "s_overhead", "s_resv_clusters",
                                           "s_cluster_bits", "s_mount_opt",
                                           "s_freeclusters_counter", "s_dirtyclusters_counter"])
        _es = sbi.s_es
        es = self.ramdump.read_datatype(_es, 'struct ext4_super_block',
                                        ["s_r_blocks_count_hi", "s_r_blocks_count_lo",
                                         "s_blocks_count_hi", "s_blocks_count_lo"])

        s_resv_clusters = sbi.s_resv_clusters.counter
        s_cluster_bits = sbi.s_cluster_bits

        resv_blocks = s_resv_clusters << s_cluster_bits
        s_mount_opt = sbi.s_mount_opt
        EXT4_MOUNT_MINIX_DF	=	0x00080
        overhead = 0
        if s_mount_opt & EXT4_MOUNT_MINIX_DF == 0:
            overhead = sbi.s_overhead

        f_bsize = self.ramdump.read_word(super_block + self.ramdump.field_offset("struct super_block", "s_blocksize"))
        s_blocks_count =  es.s_blocks_count_hi << 32 | es.s_blocks_count_lo
        f_blocks = s_blocks_count - overhead << s_cluster_bits
        bfree = self.percpu_counter(sbi.s_freeclusters_counter.count, sbi.s_freeclusters_counter.counters) + \
                    self.percpu_counter(sbi.s_dirtyclusters_counter.count, sbi.s_dirtyclusters_counter.counters)
        bfree = bfree if bfree > 0 else 0
        f_bfree = bfree << s_cluster_bits
        self.writeback(f_bsize, f_blocks, f_bfree)

    def writeback(self, f_bsize, f_blocks, f_bfree):
        if f_bsize * f_blocks <= 0:
            self.output.write("{:<6s} {:<6s} {:<6s} {:<6.0%}".format("_","_","_",0))
            return

        total = f_bsize * f_blocks
        used = (f_blocks - f_bfree) * f_bsize
        free = f_bfree * f_bsize
        used_per = used/total
        self.output.write("{:<6s} {:<6s} {:<6s} {:<6.0%}".format(self.human_str(total), self.human_str(used), self.human_str(free), used_per))

    def f2fs_statfs(self, super_block, s_fs_info):
        sbi = self.ramdump.read_datatype(s_fs_info, 'struct f2fs_sb_info',
                                         ["raw_super", "blocksize", "user_block_count",
                                          "total_valid_block_count", "current_reserved_blocks"])
        raw_super = self.ramdump.read_datatype(sbi.raw_super, 'struct f2fs_super_block',
                                               ["block_count", "segment0_blkaddr"])
        total_count = raw_super.block_count
        start_count = raw_super.segment0_blkaddr
        f_bsize = sbi.blocksize
        f_blocks = total_count - start_count

        f_bfree = sbi.user_block_count - sbi.total_valid_block_count - sbi.current_reserved_blocks

        self.writeback(f_bsize, f_blocks, f_bfree)

    def shmem_statfs(self, super_block, s_fs_info):
        sbinfo = self.ramdump.read_datatype(s_fs_info, 'struct shmem_sb_info', ["max_blocks", "used_blocks"])

        f_bsize = self.ramdump.get_page_size()
        f_blocks = 0
        f_bfree = 0
        if sbinfo.max_blocks > 0:
            f_blocks = sbinfo.max_blocks
            used_blocks = self.percpu_counter(sbinfo.used_blocks.count, sbinfo.used_blocks.counters)
            f_bfree = sbinfo.max_blocks - used_blocks

        self.writeback(f_bsize, f_blocks, f_bfree)

    def fat_statfs(self, super_block, s_fs_info):
        sbi = self.ramdump.read_datatype(s_fs_info, 'struct msdos_sb_info',
                                         ["cluster_size", "max_cluster", "free_clusters"])
        FAT_START_ENT = 2
        f_bsize =  sbi.cluster_size
        f_blocks = sbi.max_cluster - FAT_START_ENT
        f_bfree = sbi.free_clusters

        self.writeback(f_bsize, f_blocks, f_bfree)

    def fuse_statfs(self, super_block, s_fs_info):
        return 0

    def efivarfs_statfs(self, super_block, s_fs_info):
        self.writeback(0, 0, 0)

    def human_str(self, size):
        if size < 1024:
            return " %.0f " % (size)
        if size < 1024 * 1024:
            return " %.0fK " % (size/1024)
        elif size < 1024 * 1024 * 1024:
            return " %.0fM " % (size/(1024 * 1024))
        else:
            return " %.1fG "  % (size/(1024 * 1024 * 1024))

    def percpu_counter(self, count, counters):
        for core in self.ramdump.iter_cpus():
            try:
                count += self.ramdump.read_int(counters + self.ramdump.per_cpu_offset(core))
            except:
                continue
        return count
    def print_header(self):
        self.output.write("{:<16s}  {:<16s}  {:<28s} {:<6s} {:6s} {:<6s} {:<6s} {:<16s}\n".format(
                            "(struct mount *)", "(struct super_block *)", "dev_name",
                            "Size", "Used", "Avail", "Use%", "Mounted On"))

    def show_info(self, mtuple):
        mount = mtuple.mount
        vfsmount = mtuple.vfsmount
        sb = mtuple.super_block
        dname = mtuple.dname
        mount_path = mtuple.mpath

        sbi = self.ramdump.read_datatype(sb, 'struct super_block', ['s_fs_info', "s_dev"])
        if sbi.s_dev in self.dev_ids:
            return
        self.dev_ids.append(sbi.s_dev)
        s_op = self.ramdump.read_structure_field(sb, 'struct super_block', 's_op')
        statfs = self.ramdump.read_structure_field(s_op, 'struct super_operations', 'statfs')

        look = self.ramdump.unwind_lookup(statfs)
        if look:
            fop, _ = look
            if hasattr(self, fop):
                self.output.write("0x{:16x} 0x{:16x} {:<32s}".format(mount, sb, dname))
                eval("self." + fop)(sb, sbi.s_fs_info)
                self.output.write("{:<32s} {:<16s}".format(mount_path, fop))
                self.output.write("\n")