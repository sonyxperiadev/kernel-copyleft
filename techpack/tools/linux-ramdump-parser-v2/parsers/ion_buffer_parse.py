"""
Copyright (c) 2016, 2018, 2020-2021 The Linux Foundation. All rights reserved.
Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.

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
from print_out import print_out_str
from rb_tree import RbTree
import logging
import os

RB_PARENT_COLOR_MASK = 0xFFFFFFFFFFFFFFFC
grand_total = 0
TASK_NAME_LENGTH = 16
ion_heap_buffers = []
dmabuf_heap_names = ["qcom_dma_heaps"]


def bytes_to_KB(bytes):
    kb_val = 0
    if bytes != 0:
        kb_val = bytes // 1024
    return kb_val

def bytes_to_mb(bytes):
        val = 0
        if bytes != 0:
            val = (bytes // 1024) // 1024
        return val

def get_dmabuf_heap_names(self, ramdump, ion_info):
    heap_list = ramdump.address_of('heap_list')
    if heap_list is None:
        ion_info.write("NOTE: 'heap_list' list not found to extract the "
                        "dmabuf heap type list")
        return False

    list_offset = ramdump.field_offset('struct dma_heap', 'list')
    name_offset = ramdump.field_offset('struct dma_heap', 'name')
    next_offset = ramdump.field_offset('struct list_head', 'next')
    prev_offset = ramdump.field_offset('struct list_head', 'prev')

    head = ramdump.read_word(heap_list + next_offset)
    while (head != heap_list):
        dma_heap_addr = head - list_offset
        dma_heap_name_addr = ramdump.read_word(dma_heap_addr + name_offset)
        dma_heap_name = ramdump.read_cstring(dma_heap_name_addr, 48)
        dmabuf_heap_names.append(dma_heap_name)

        head = ramdump.read_word(head + next_offset)
        prev = ramdump.read_word(head + prev_offset)
        if head == 0 or prev == 0:
            ion_info.write("NOTE: 'dmabuf heap_list' is corrupted")
            return False

    return True

def ion_buffer_info(self, ramdump, ion_info):
    ion_info = ramdump.open_file('ionbuffer.txt')
    db_list = ramdump.address_of('db_list')
    if db_list is None:
        ion_info.write("NOTE: 'db_list' list not found to extract the ion "
                       "buffer information")
        return
    total_dma_heap = 0
    total_dma_info = ramdump.open_file('total_dma_heap.txt')
    ion_info.write("*****Parsing dma buf info for ion leak debugging*****\n\n")
    head_offset = ramdump.field_offset('struct dma_buf_list', 'head')
    head = ramdump.read_word(db_list + head_offset)
    next_offset = ramdump.field_offset('struct list_head', 'next')
    prev_offset = ramdump.field_offset('struct list_head', 'prev')
    list_node_offset = ramdump.field_offset('struct dma_buf', 'list_node')
    size_offset = ramdump.field_offset('struct dma_buf', 'size')
    file_offset = ramdump.field_offset('struct dma_buf', 'file')
    f_count_offset = ramdump.field_offset('struct file', 'f_count')
    name_offset = ramdump.field_offset('struct dma_buf', 'buf_name')
    if name_offset is None:
        name_offset = ramdump.field_offset('struct dma_buf', 'name')
    exp_name_offset = ramdump.field_offset('struct dma_buf', 'exp_name')
    ion_info.write("{0:40} {1:4} {2:15} {3:10} {4:10} {5:10} {6:20}\n".format(
            'File_addr', 'REF', 'Name', 'Size', 'Exp', 'Heap', 'Size in KB'))
    dma_buf_info = []
    if (ramdump.kernel_version >= (5, 10)):
        if get_dmabuf_heap_names(self, ramdump, ion_info) is False:
            return

    while (head != db_list):
        dma_buf_addr = head - list_node_offset
        size = ramdump.read_word(dma_buf_addr + size_offset)
        total_dma_heap = total_dma_heap + size
        file = ramdump.read_word(dma_buf_addr + file_offset)
        f_count = ramdump.read_u64(file + f_count_offset)
        exp_name = ramdump.read_word(dma_buf_addr + exp_name_offset)
        exp_name = ramdump.read_cstring(exp_name, 48)
        ionheap_name = None
        if (ramdump.kernel_version >= (5, 10)):
            if exp_name in dmabuf_heap_names:
                ion_buffer = ramdump.read_structure_field(dma_buf_addr, 'struct dma_buf', 'priv')
                ion_heap = ramdump.read_structure_field(ion_buffer, 'struct qcom_sg_buffer', 'heap')
                ionheap_name_addr = ramdump.read_structure_field(ion_heap, 'struct dma_heap', 'name')
                ionheap_name = ramdump.read_cstring(ionheap_name_addr, TASK_NAME_LENGTH)
        else:
            if exp_name == 'ion':
                ion_buffer = ramdump.read_structure_field(dma_buf_addr, 'struct dma_buf', 'priv')
                ion_heap = ramdump.read_structure_field(ion_buffer, 'struct ion_buffer', 'heap')
                ionheap_name_addr = ramdump.read_structure_field(ion_heap, 'struct ion_heap', 'name')
                ionheap_name = ramdump.read_cstring(ionheap_name_addr, TASK_NAME_LENGTH)
        if ionheap_name is None:
            ionheap_name = "None"
        name = ramdump.read_word(dma_buf_addr + name_offset)
        if not name:
            name = "None"
        else:
            name = ramdump.read_cstring(name, 48)
        dma_buf_info.append([file, f_count, name, hex(size), exp_name,
                             ionheap_name, bytes_to_KB(size)])
        head = ramdump.read_word(head)
        next_node = ramdump.read_word(head + next_offset)
        if next_node == 0:
            print_out_str("db_list is corrupted!\nComplete ion buffer "
                          "ionformation may not be available")
            head = ramdump.read_word(db_list + head_offset + prev_offset)
            while (head != db_list):
                dma_buf_addr = head - list_node_offset
                size = ramdump.read_word(dma_buf_addr + size_offset)
                total_dma_heap = total_dma_heap + size
                file = ramdump.read_word(dma_buf_addr + file_offset)
                f_count = ramdump.read_u64(file + f_count_offset)
                exp_name = ramdump.read_word(dma_buf_addr + exp_name_offset)
                exp_name = ramdump.read_cstring(exp_name, 48)
                if (ramdump.kernel_version >= (5, 10)):
                    if exp_name in dmabuf_heap_names:
                        ion_buffer = ramdump.read_structure_field(dma_buf_addr, 'struct dma_buf', 'priv')
                        ion_heap = ramdump.read_structure_field(ion_buffer, 'struct qcom_sg_buffer', 'heap')
                        ionheap_name_addr = ramdump.read_structure_field(ion_heap, 'struct dma_heap', 'name')
                        ionheap_name = ramdump.read_cstring(ionheap_name_addr, TASK_NAME_LENGTH)
                else:
                    if exp_name == 'ion':
                        ion_buffer = ramdump.read_structure_field(dma_buf_addr, 'struct dma_buf', 'priv')
                        ion_heap = ramdump.read_structure_field(ion_buffer, 'struct ion_buffer', 'heap')
                        ionheap_name_addr = ramdump.read_structure_field(ion_heap, 'struct ion_heap', 'name')
                        ionheap_name = ramdump.read_cstring(ionheap_name_addr, TASK_NAME_LENGTH)
                if ionheap_name is None:
                    ionheap_name = "None"
                name = ramdump.read_word(dma_buf_addr + name_offset)
                if not name:
                    name = "None"
                else:
                    name = ramdump.read_cstring(name, 48)
                dma_buf_info.append([file, f_count, name, hex(size), exp_name,
                                     ionheap_name, bytes_to_KB(size)])
                head = ramdump.read_word(head + prev_offset)
                prev_node = ramdump.read_word(head + prev_offset)
                if prev_node == 0:
                    break
            break

    dma_buf_info = sorted(dma_buf_info, key=lambda l: l[6], reverse=True)
    total_dma_heap_mb = bytes_to_mb(total_dma_heap)
    total_dma_heap_mb = str(total_dma_heap_mb) + "MB"
    total_dma_info.write("Total dma memory: {0}".format(total_dma_heap_mb))
    for item in dma_buf_info:
        str_data = "v.v (struct file *)0x{0:x}\t {1:2}   {2:15} {3:10} {4:10} {" \
              "5:10} ({6} KB)\n".format(item[0], item[1], item[2], item[3],
                                        item[4], item[5], item[6])
        ion_info.write(str_data)


def get_bufs(self, task, bufs, ion_info, ramdump):
    t_size = 0
    dma_buf_fops = ramdump.address_of('dma_buf_fops')
    if dma_buf_fops is None:
        ion_info.write("NOTE: 'dma_buf_fops' not found for file information\n")
        return 0

    if task is None:
        return 0
    files = ramdump.read_pointer(task + self.files_offset)
    if files is None:
        return 0
    fdt = ramdump.read_pointer(files + self.fdt_offset)
    if fdt is None:
        return 0
    fd = ramdump.read_pointer(fdt + self.fd_offset)
    max_fds = ramdump.read_halfword(fdt + self.max_fds_offset)
    stime = ramdump.read_word(self.timekeeper + self.stime_offset)
    ctime_offset = ramdump.field_offset('struct dma_buf', 'ktime')
    if ctime_offset is not None:
        ctime_offset += ramdump.field_offset('struct timespec', 'tv_sec')
    for i in range(max_fds):
        file = ramdump.read_pointer(fd + i*8)
        if (file == 0):
            continue
        f_op = ramdump.read_pointer(file + self.f_op_offset)
        if (f_op != dma_buf_fops):
            continue
        dmabuf = ramdump.read_pointer(file + self.private_data_offset)
        size = ramdump.read_word(dmabuf + self.size_offset)
        time = 0
        if ctime_offset is not None:
            ctime = ramdump.read_word(dmabuf + ctime_offset)
            ctime = ctime // 1000000000
            time = stime - ctime
        name = ramdump.read_word(dmabuf + self.exp_name_offset)
        if not name:
            name = "None"
        else:
            name = ramdump.read_cstring(name, 48)

        item = [name, hex(size), bytes_to_KB(size), str(time), hex(file)]
        if item not in bufs:
            t_size = t_size + size
            bufs.append(item)
    bufs.sort(key=lambda item: -item[2])
    return t_size


def get_proc_bufs(self, task, bufs, ion_info, ramdump):
    size = 0
    for curr in ramdump.for_each_thread(task):
        size += get_bufs(self, curr, bufs, ion_info, ramdump)
    return size


def ion_proc_info(self, ramdump, ion_info):
    ion_info = ramdump.open_file('ionproc.txt')
    ion_info.write("*****Parsing dma proc info for ion leak debugging*****\n")
    pid_offset = ramdump.field_offset('struct task_struct', 'tgid')
    comm_offset = ramdump.field_offset('struct task_struct', 'comm')
    dma_procs = []
    for task in ramdump.for_each_process():
        bufs = []
        size = get_proc_bufs(self, task, bufs, ion_info, ramdump)
        if (size == 0):
            continue
        comm = ramdump.read_cstring(task + comm_offset)
        pid = ramdump.read_int(task + pid_offset)
        dma_procs.append([comm, pid, bytes_to_KB(size), bufs])

    dma_procs.sort(key=lambda item: -item[2])
    for proc in dma_procs:
        str = "\n{0} (PID {1}) size (KB): {2}\n"\
            .format(proc[0], proc[1], proc[2])
        ion_info.write(str)
        ion_info.write("{0:15} {1:15} {2:10} {3:20} {4:25}\n".format(
                'Name', 'Size', 'Size in KB', 'Time Alive(sec)', '(struct file *)'))
        for item in proc[3]:
            str = "{0:15} {1:15} {2:10} {3:20} {4:25}\n".\
                format(item[0], item[1], item[2], item[3], item[4])
            ion_info.write(str)


def do_dump_ionbuff_info(self, ramdump, ion_info):
    addressspace = 8
    heap_addr_array = []
    ion_info = ramdump.open_file('ionbuffer.txt')
    # read num of heaps
    number_of_heaps = ramdump.read_word('num_heaps')
    ion_info.write('Number of heaps:{0} \n'.format(number_of_heaps))

    # get heap starting address
    heap_addr = ramdump.read_pointer('heaps')

    if self.ramdump.arm64:
        addressspace = 8
    else:
        addressspace = 4

    # get address of all heaps
    nIndex = 0
    for nIndex in range(0, number_of_heaps):
        heap_addr_array.append(heap_addr + (nIndex*addressspace))

    # parse a heap
    nIndex = 0
    for nIndex in range(0, number_of_heaps):
        str = "\n\n parsing {0:0} of {1:0} heap    Heap: 0x{2:x}"
        ion_info.write(str.format(
                                    nIndex + 1,
                                    number_of_heaps,
                                    ramdump.read_word(
                                                heap_addr_array[nIndex])))
        parse_heap(self, ramdump, heap_addr_array[nIndex], ion_info)
    ion_info.write(
                    '\n Total ION buffer size: {0:1} KB'.format(
                                            bytes_to_KB(grand_total)))


def parse_heap(self, ramdump, heap_addr,  ion_info):
    global grand_total

    nr_clients = 0
    total_orphan_buffer_size = 0
    ion_heap = ramdump.read_word(heap_addr)
    ionheap_id = ramdump.read_structure_field(
                        ion_heap, 'struct ion_heap',  'id')
    ionheap_name_addr = ramdump.read_structure_field(
                    ion_heap, 'struct ion_heap',  'name')
    ionheap_name = ramdump.read_cstring(ionheap_name_addr, TASK_NAME_LENGTH)
    ionheap_type = ramdump.read_structure_field(
                    ion_heap, 'struct ion_heap',  'type')
    ionheap_total_allocated = ramdump.read_structure_field(
                    ion_heap, 'struct ion_heap',  'total_allocated.counter')
    ionheap_total_handles = ramdump.read_structure_field(
                    ion_heap, 'struct ion_heap',  'total_handles.counter')
    self.ion_handle_node_offset = ramdump.field_offset(
                                    'struct ion_handle', 'node')

    ion_info.write("\n*********************************************")
    str = "\n Heap ID : {0} Heap Type: {1} Heap Name : {2}\n"
    ion_info.write(str.format(ionheap_id, ionheap_type, ionheap_name))
    ion_info.write('\n Total allocated : {0:1} KB'.format(
                    bytes_to_KB(ionheap_total_allocated)))
    ion_info.write('\n Total Handles   : {0:1} KB'.format(
                            bytes_to_KB(ionheap_total_handles)))
    orphan = bytes_to_KB(ionheap_total_allocated - ionheap_total_handles)
    ion_info.write('\n Orphan          : {0:1} KB'.format(orphan))
    ion_info.write("\n*********************************************")

    ion_dev = ramdump.read_structure_field(
                    ion_heap, 'struct ion_heap',  'dev')

    clients_rb_root = ion_dev + ramdump.field_offset('struct ion_device',  'clients')

    if ionheap_total_allocated != 0:
        nr_clients = show_ion_dev_client(
                            self, ramdump,
                            clients_rb_root,
                            ionheap_id, ion_info)

        str = "\n \nTotal number of clients: {0:1}"
        ion_info.write(str.format(nr_clients))
        ion_info.write("\n ----------------------------------")
        str = "\n orphaned allocations (info is from last known client):\n"
        ion_info.write(str)
        total_orphan_buffer_size,  total_buffer_size = \
            parse_orphan_buffers(self, ramdump, ion_dev, ionheap_id, ion_info)
        ion_info.write("\n ----------------------------------")
        ion_info.write(
                '\n total orphan size: {0} KB'.format(
                        bytes_to_KB(total_orphan_buffer_size)))
        ion_info.write(
                    '\n total buffer size: {0} KB'.format(
                            bytes_to_KB(total_buffer_size)))
        ion_info.write("\n ----------------------------------")
        grand_total = grand_total + total_buffer_size


def parse_orphan_buffers(self, ramdump, ion_dev, heap_id, ion_info):
    orphan_buffer_size = 0
    total_buffer_size = 0

    rbtree = RbTree(ramdump, ion_dev + ramdump.field_offset('struct ion_device', 'buffers'),
                    logger = self.logger, debug = True)

    ion_buffer_rb_node_offset = ramdump.field_offset(
                                'struct ion_buffer', 'node')
    ion_buffer_task_comm_offset = ramdump.field_offset(
                                'struct ion_buffer', 'task_comm')
    ion_buffer_ref_offset = ramdump.field_offset(
                                'struct ion_buffer', 'ref')
    str = "\n buffer: 0x{0:x}, Buffer size: {1} KB "
    str = str + "comm: {2} PID: {3} kmap count: {4} ref_count : {5}"
    for rb_node in rbtree:
        ion_buffer = rb_node - ion_buffer_rb_node_offset
        ion_buffer_ref_add = ion_buffer + ion_buffer_ref_offset
        ion_buffer_heap = ramdump.read_structure_field(
                ion_buffer, 'struct ion_buffer', 'heap')
        ion_heap_id = ramdump.read_structure_field(
                ion_buffer_heap, 'struct ion_heap', 'id')
        ion_buffer_size = ramdump.read_structure_field(
                ion_buffer, 'struct ion_buffer', 'size')
        ion_buffer_handlecount = ramdump.read_structure_field(
                ion_buffer, 'struct ion_buffer', 'handle_count')
        ref_counter = ramdump.read_structure_field(
                ion_buffer_ref_add, 'struct kref', 'refcount.counter')
        if heap_id == ion_heap_id:
            total_buffer_size = total_buffer_size + ion_buffer_size
            # if orphaned allocation
            if ion_buffer_handlecount == 0:
                ion_buffer_pid = ramdump.read_structure_field(
                                ion_buffer, 'struct ion_buffer', 'pid')
                ion_buffer_kmap_count = ramdump.read_structure_field(
                                ion_buffer, 'struct ion_buffer', 'kmap_cnt')
                client_name = ramdump.read_cstring(
                                (ion_buffer + ion_buffer_task_comm_offset),
                                TASK_NAME_LENGTH)
                ion_info.write(str.format(
                                            ion_buffer,
                                            bytes_to_KB(ion_buffer_size),
                                            client_name,
                                            ion_buffer_pid,
                                            ion_buffer_kmap_count,
                                            ref_counter))
                orphan_buffer_size = orphan_buffer_size + ion_buffer_size
    return orphan_buffer_size,  total_buffer_size


def show_ion_dev_client(
                        self,
                        ramdump,
                        rb_root,
                        ionheap_id, ion_info):
    global ion_heap_buffers
    nr_clients = 0
    client_name = 0

    rbtree = RbTree(ramdump, rb_root, logger = self.logger, debug = True)

    ion_client_node_offset = ramdump.field_offset(
                            'struct ion_client',  'node')
    task_comm_offset = ramdump.field_offset(
                        'struct task_struct',  'comm')
    tempstr = "\n\n CLIENT: (struct ion_client *)0x{0:x} ,  "
    str = tempstr + "task : {1} / ion_client : {2} / PID: {3} / Size : {4} KB"
    str1 = tempstr + "ion_client : {1} / PID: {2} / Size : {3} KB"
    if True:
        for rb_node in rbtree:
            ion_client = rb_node - ion_client_node_offset
            heap_size = traverse_ion_heap_buffer(
                                                            self,
                                                            ramdump,
                                                            ion_client,
                                                            ionheap_id,
                                                            ion_info)
            if heap_size > 0:
                nr_clients = nr_clients + 1
                ion_client_task = ramdump.read_structure_field(
                                    ion_client, 'struct ion_client', 'task')
                task_comm = ion_client_task + task_comm_offset
                client_name = ramdump.read_cstring(
                                task_comm, TASK_NAME_LENGTH)
                ion_client_name = ramdump.read_structure_field(
                                    ion_client,
                                    'struct ion_client',
                                    'display_name')
                ion_client_name = ramdump.read_cstring(
                                    ion_client_name,
                                    TASK_NAME_LENGTH)

                client_PID = ramdump.read_structure_field(
                                ion_client, 'struct ion_client', 'pid')
                if ion_client_task != 0:
                    ion_info.write(str.format(
                                    ion_client, client_name, ion_client_name,
                                    client_PID, bytes_to_KB(heap_size)))
                else:
                    ion_info.write(str1.format(
                                    ion_client,  ion_client_name,
                                    client_PID, bytes_to_KB(heap_size)))
                for heap_buffer in ion_heap_buffers:
                    ion_info.write(heap_buffer)
    return nr_clients


def traverse_ion_heap_buffer(self, ramdump, ion_client,  ionheap_id, ion_info):
    global ion_heap_buffers
    ion_handle_root_offset = ramdump.field_offset(
                            'struct ion_client', 'handles')
    ion_handle_root_address = ion_client + ion_handle_root_offset
    ion_buffer_heap_size = 0
    ion_heap_buffers = []
    str = "\n (+) ion_buffer: 0x{0:x} size: {1:0} KB Handle Count: {2:0}"

    rbtree = RbTree(ramdump, ion_handle_root_address,
                    logger=self.logger, debug = True)

    for ion_handle_rb_node in rbtree:
        ion_handle = ion_handle_rb_node - self.ion_handle_node_offset
        ion_buffer = ramdump.read_structure_field(
                            ion_handle, 'struct ion_handle', 'buffer')
        ion_buffer_size = ramdump.read_structure_field(
                            ion_buffer, 'struct ion_buffer', 'size')
        ion_buffer_heap = ramdump.read_structure_field(
                            ion_buffer, 'struct ion_buffer', 'heap')
        ion_heap_id = ramdump.read_structure_field(
                            ion_buffer_heap, 'struct ion_heap', 'id')
        if ionheap_id == ion_heap_id:
            ion_buffer_heap_size = ion_buffer_heap_size + ion_buffer_size
            ion_buffer_handlecount = ramdump.read_structure_field(
                                        ion_buffer,
                                        'struct ion_buffer', 'handle_count')
            temp = str.format(
                            ion_buffer,
                            bytes_to_KB(ion_buffer_size),
                            ion_buffer_handlecount)
            ion_heap_buffers.append(temp)
    return ion_buffer_heap_size


def parser(self, arg, ramdump, node, ion_info):
    rb_root = 0
    last_node = 0
    self.orphan_size = 0
    rbnode_left_offset = ramdump.field_offset('struct rb_node',  'rb_left')
    temp = ramdump.read_word(node)
    if temp == 0:
        return 0
    if arg == 1:
        rb_root = find_rb_root(self, ramdump, node, ion_info)
        last_node = find_rb_first(
                    self, ramdump,  rb_root, rbnode_left_offset, ion_info)
    if arg == 2:
        last_node = find_rb_next(
                    self, arg, ramdump, node, rbnode_left_offset, ion_info)
    return last_node


def find_rb_next(self, arg, ramdump, node, rbnode_left_offset, ion_info):
    parent = cal_rb_parent(self, ramdump, node, ion_info)
    tmp_node = 0
    if parent == node:
        ion_info.write("RETURNING NULL")
        return 0
    rbnode_right_offset = ramdump.field_offset('struct rb_node',  'rb_right')
    rb_right = ramdump.read_word(node + rbnode_right_offset)
    if rb_right != 0:  # right node exist
        next_rb_node = find_rb_first(
                        self, ramdump, rb_right, rbnode_left_offset,  ion_info)
        return next_rb_node
    else:  # no right node,  parse left node
        flag = 1
        while flag:
            if parent == 0 or None:
                tmp_node = 0
                parent = 0
            else:
                parent = cal_rb_parent(self, ramdump, node, ion_info)
                tmp_node = ramdump.read_word(parent + rbnode_right_offset)
            if tmp_node == node:
                node = parent
                continue
            else:
                return parent
    return 0


def find_rb_first(self, ramdump, node, rbnode_left_offset, ion_info):
    last_node = node
    while node != 0:
        last_node = node
        node = ramdump.read_word(node + rbnode_left_offset)
    return last_node


def cal_rb_parent(self, ramdump, ion_dev_rb_root, ion_info):
    rbnode_color_offset = ramdump.field_offset(
                        'struct rb_node',  '__rb_parent_color')
    color = ramdump.read_word(ion_dev_rb_root + rbnode_color_offset)
    color = color & RB_PARENT_COLOR_MASK
    return color


def find_rb_root(self, ramdump, ion_dev_rb_root, ion_info):
    parent = ion_dev_rb_root
    rbnode_color_offset = ramdump.field_offset(
                        'struct rb_node',  '__rb_parent_color')
    color = ramdump.read_word(ion_dev_rb_root + rbnode_color_offset)
    while color != 1:
        parent = cal_rb_parent(self,  ramdump,  parent,  ion_info)
        color = ramdump.read_word(parent + rbnode_color_offset)
    return parent


@register_parser('--print-ionbuffer',  'Print ion buffer',  optional=True)
class DumpIonBuffer(RamParser):

    def __init__(self, *args):
        super(DumpIonBuffer, self).__init__(*args)
        self.timekeeper = self.ramdump.address_of('shadow_timekeeper')
        self.files_offset = self.ramdump.field_offset(
                                     'struct task_struct', 'files')
        self.fdt_offset = self.ramdump.field_offset(
                                    'struct files_struct', 'fdt')
        self.fd_offset = self.ramdump.field_offset('struct fdtable', 'fd')
        self.max_fds_offset = self.ramdump.field_offset(
                                    'struct fdtable', 'max_fds')
        self.f_op_offset = self.ramdump.field_offset('struct file', 'f_op')
        self.private_data_offset = self.ramdump.field_offset('struct file',
                                                   'private_data')
        self.size_offset = self.ramdump.field_offset('struct dma_buf', 'size')
        self.name_offset = self.ramdump.field_offset('struct dma_buf', 'buf_name')
        if self.name_offset is None:
            self.name_offset = self.ramdump.field_offset('struct dma_buf', 'name')
        self.exp_name_offset = self.ramdump.field_offset('struct dma_buf', 'exp_name')
        self.stime_offset = self.ramdump.field_offset('struct timekeeper',
                                                'ktime_sec')

    def parse(self):
        with self.ramdump.open_file('ionbuffer.txt') as ion_info:
            if (self.ramdump.kernel_version < (3, 18, 0)):
                ion_info.write('Kernel version 3.18 \
                and above are supported, current version {0}.\
                {1}'.format(self.ramdump.kernel_version[0],
                            self.ramdump.kernel_version[1]))
                return

            self.logger = logging.getLogger(__name__)
            path = os.path.join(self.ramdump.outdir, 'print-ionbuffer.stderr')
            self.logger.addHandler(logging.FileHandler(path, mode='w'))
            self.logger.setLevel(logging.INFO)
            self.logger.info("Starting --print-ionbuffer")
            if (self.ramdump.kernel_version >= (4, 14)):
                ion_buffer_info(self, self.ramdump, ion_info)
                ion_proc_info(self, self.ramdump, ion_info)
            else:
                do_dump_ionbuff_info(self, self.ramdump, ion_info)
