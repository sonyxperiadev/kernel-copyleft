# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.

import linux_list
import rb_tree
from print_out import print_out_str
from parser_util import register_parser, RamParser, cleanupString

@register_parser('--binder-dump', 'Extract binder information in binder driver', optional=True)
class BinderParser(RamParser) :

    def __init__(self, *args):
        super(BinderParser, self).__init__(*args)

        self.SEPARATOR      = "|=================================================================================================|"
        self.TITLE_PROC     = "|  pid   |       task       |    binder_proc   | requested | started |  dead |        todo        |"
        self.SEPARATOR_SUB1 = "|--------|------------------|------------------|---------------------|-------|--------------------|"
        self.TITLE_THREAD   = "|        |                  |   binder_thread  |  transaction_stack  |  pid  |                    |"
        self.SEPARATOR_SUB2 = "|        |                  |------------------|---------------------|-------|--------------------|"
        self.TITLE_NODE     = "|        |                  |    binder_node   |has_async_transaction|       |     async_todo     |"
        self.TITLE_REF      = "|        |                  |    binder_ref    |  remote binder_node |       |                    |"

        self.PROC_OUTPUT_FMT    = "| {0:^6} | {1:^16s} |0x{2:^16X}|     {3:^2}    |    {4:^2}   |   {5:^1}   |                    |"
        self.THREAD_OUTPUT_FMT1 = "|        |                  |0x{0:^16X}|" + "         NULL        " + "| {1:^6}|                    |"
        self.THREAD_OUTPUT_FMT2 = "|        |                  |0x{0:^16X}|" +     "  0x{1:^16X} "     + "| {2:^6}|                    |"
        self.NODE_OUTPUT_FMT = "|        |                  |0x{0:^16X}|          {1:^1}          |       |                    |"
        self.REF_OUTPUT_FMT  = "|        |                  |0x{0:^16X}|  0x{1:^16X} | desc {2:^8}|                    |"
        self.TRANS_FMT       = "|        |                  |                  |                     |       |"
        self.TRANS_FMT1      = self.TRANS_FMT + "        NULL        |"
        self.TRANS_FMT2      = self.TRANS_FMT + " 0x{0:^16X} |"

        self.binder_node_offset = self.ramdump.field_offset('struct binder_node', 'rb_node')
        self.binder_ref_offset  = self.ramdump.field_offset('struct binder_ref', 'rb_node_desc')

        self.work_entry_offset  = self.ramdump.field_offset('struct binder_work', 'entry')


    def transactions_walker(self, work):
        if work == self.work_head:
            return
        transaction_work_offset = self.ramdump.field_offset('struct binder_transaction', 'work')
        trans = work - transaction_work_offset

        print (self.TRANS_FMT2.format(trans), file=self.outfd)


    # Parse binder_thread node one by one.
    def binder_threads_walker(self, node, extra):
        thread_node_offset = self.ramdump.field_offset('struct binder_thread', 'rb_node')
        thread = node - thread_node_offset

        thread_pid_offset   = self.ramdump.field_offset('struct binder_thread', 'pid')
        thread_stack_offset = self.ramdump.field_offset('struct binder_thread', 'transaction_stack')
        thread_todo_offset  = self.ramdump.field_offset('struct binder_thread', 'todo')

        thread_pid   = self.ramdump.read_s32(thread + thread_pid_offset)
        thread_stack = self.ramdump.read_word(thread + thread_stack_offset)
        print (self.SEPARATOR_SUB2, file=self.outfd)

        if thread_stack == 0:
            print (self.THREAD_OUTPUT_FMT1.format(thread, thread_pid), file=self.outfd)
        else:
            print (self.THREAD_OUTPUT_FMT2.format(thread, thread_stack, thread_pid), file=self.outfd)

        # Walk binder_thread.todo list.
        todo_head = thread + thread_todo_offset
        fist_node = self.ramdump.read_word(todo_head)
        works_walker = linux_list.ListWalker(self.ramdump, fist_node, self.work_entry_offset)
        if works_walker.is_empty():
            print (self.TRANS_FMT1, file=self.outfd)
        else:
            self.work_head = todo_head - self.work_entry_offset
            works_walker.walk(fist_node, self.transactions_walker)


    # Parse binder_thread node one by one.
    def binder_nodes_walker(self, node, extra):
        bnode_offset = self.ramdump.field_offset('struct binder_node', 'rb_node')
        bnode = node - bnode_offset

        bnode_async_offset = self.ramdump.field_offset('struct binder_node', 'has_async_transaction')
        bnode_todo_offset  = self.ramdump.field_offset('struct binder_node', 'async_todo')

        has_async = self.ramdump.read_u32(bnode + bnode_async_offset)
        has_async = (has_async >> 4) & 1
        print (self.SEPARATOR_SUB2, file=self.outfd)
        print (self.NODE_OUTPUT_FMT.format(bnode, has_async), file=self.outfd)


        # Walk binder_node.async_todo list.
        todo_head = bnode + bnode_todo_offset
        fist_node = self.ramdump.read_word(todo_head)
        works_walker = linux_list.ListWalker(self.ramdump, fist_node, self.work_entry_offset)
        if works_walker.is_empty():
            print (self.TRANS_FMT1, file=self.outfd)
        else:
            self.work_head = todo_head - self.work_entry_offset
            works_walker.walk(fist_node, self.transactions_walker)


    # Parse binder_ref node one by one.
    def binder_refs_walker(self, node, extra):
        bref_offset = self.ramdump.field_offset('struct binder_ref', 'rb_node_desc')
        bref = node - bref_offset

        bref_node_offset = self.ramdump.field_offset('struct binder_ref', 'node')

        bref_node = self.ramdump.read_word(bref + bref_node_offset)

        data_offset = self.ramdump.field_offset('struct binder_ref', 'data')
        desc_offset = self.ramdump.field_offset('struct binder_ref_data', 'desc')
        desc_addr = bref + desc_offset + data_offset
        desc = self.ramdump.read_u32(desc_addr)

        print (self.SEPARATOR_SUB2, file=self.outfd)
        print (self.REF_OUTPUT_FMT.format(bref, bref_node, desc), file=self.outfd)

    def binder_alloc_allocated_buffers_walker(self, node, extra):
        rb_node_offset = self.ramdump.field_offset('struct binder_buffer', 'rb_node')
        binder_buffer = node - rb_node_offset
        print("v.v %s (binder_buffer)0x%-32x" % ("%h", binder_buffer), file=self.outfd, end = ' ')
        user_data =  self.ramdump.read_structure_field(binder_buffer, 'struct binder_buffer', 'user_data')
        free = self.ramdump.read_structure_field(binder_buffer, 'struct binder_buffer', 'free')
        if free == None:
            free = 2
        else:
            free = free & 0x1
        if user_data == None:
            user_data = 0xaa55
        print(" user_data 0x%-32x free = %-16d" % (user_data, free), file=self.outfd)
        return

    def parse_binder_procs(self):
        hlist_first_offset = self.ramdump.field_offset('struct hlist_head', 'first')
        hlist_next_offset  = self.ramdump.field_offset('struct hlist_node', 'next')
        proc_node_offset   = self.ramdump.field_offset('struct binder_proc', 'proc_node')

        proc_pid_offset    = self.ramdump.field_offset('struct binder_proc', 'pid')
        proc_task_offset   = self.ramdump.field_offset('struct binder_proc', 'tsk')
        task_comm_offset   = self.ramdump.field_offset('struct task_struct', 'comm')
        requested_threads_offset = self.ramdump.field_offset('struct binder_proc', 'requested_threads')
        started_threads_offset   = self.ramdump.field_offset('struct binder_proc', 'requested_threads_started')
        is_dead_offset     = self.ramdump.field_offset('struct binder_proc', 'is_dead')
        proc_todo_offset   = self.ramdump.field_offset('struct binder_proc', 'todo')
        alloc_offset = self.ramdump.field_offset('struct binder_proc', 'alloc')

        binder_procs_addr  = self.ramdump.address_of('binder_procs')
        first_proc_node    = self.ramdump.read_word(binder_procs_addr + hlist_first_offset)

        proc_node = first_proc_node
        while proc_node != 0:
            proc = proc_node - proc_node_offset

            # Get binder_proc fields.
            proc_pid  = self.ramdump.read_s32(proc + proc_pid_offset)
            task      = self.ramdump.read_word(proc + proc_task_offset)
            task_name = cleanupString(self.ramdump.read_cstring(task + task_comm_offset, 16))
            requested = self.ramdump.read_s32(proc + requested_threads_offset)
            requested_started = self.ramdump.read_s32(proc + started_threads_offset)
            is_dead   = self.ramdump.read_s32(proc + is_dead_offset)
            alloc = proc + alloc_offset
            print("v.v %s (binder_proc)0x%x" % ("%h", proc), file=self.outfd)
            print("v.v %s (binder_alloc)0x%x" %("%h", alloc), file=self.outfd)
            print(" allocated_buffers", file=self.outfd)


            allocated_buffers_offset = self.ramdump.field_offset('struct binder_alloc', 'allocated_buffers')
            allocated_buffers_node = allocated_buffers_offset + alloc
            allocated_buffers_node = self.ramdump.read_pointer(allocated_buffers_node)
            rb_walker = rb_tree.RbTreeWalker(self.ramdump)
            rb_walker.walk(allocated_buffers_node, self.binder_alloc_allocated_buffers_walker)

            print(" free_buffers", file=self.outfd)
            allocated_buffers_offset = self.ramdump.field_offset('struct binder_alloc', 'free_buffers')
            allocated_buffers_node = allocated_buffers_offset + alloc
            allocated_buffers_node = self.ramdump.read_pointer(allocated_buffers_node)
            rb_walker = rb_tree.RbTreeWalker(self.ramdump)
            rb_walker.walk(allocated_buffers_node, self.binder_alloc_allocated_buffers_walker)

            print (self.SEPARATOR, file=self.outfd)
            print (self.TITLE_PROC, file=self.outfd)
            print (self.SEPARATOR_SUB1, file=self.outfd)
            print (self.PROC_OUTPUT_FMT.format(proc_pid, task_name, proc, requested, requested_started, is_dead), file=self.outfd)
            # Walk binder_proc.todo list.
            todo_head = proc + proc_todo_offset

            fist_node = self.ramdump.read_word(todo_head)
            works_walker = linux_list.ListWalker(self.ramdump, fist_node, self.work_entry_offset)
            if works_walker.is_empty():
                print (self.TRANS_FMT1, file=self.outfd)
            else:
                self.work_head = todo_head - self.work_entry_offset
                works_walker.walk(fist_node, self.transactions_walker)

            print (self.SEPARATOR_SUB2, file=self.outfd)
            print (self.TITLE_THREAD, file=self.outfd)

            # Walk binder_proc.threads rb_tree.
            proc_threads_offset = self.ramdump.field_offset('struct binder_proc', 'threads')
            proc_threads_node   = self.ramdump.read_word(proc + proc_threads_offset)
            rb_walker = rb_tree.RbTreeWalker(self.ramdump)
            rb_walker.walk(proc_threads_node, self.binder_threads_walker)

            print (self.SEPARATOR_SUB2, file=self.outfd)
            print (self.TITLE_NODE, file=self.outfd)

            # Walk binder_proc.nodes rb_tree.
            proc_nodes_offset = self.ramdump.field_offset('struct binder_proc', 'nodes')
            proc_nodes_node   = self.ramdump.read_word(proc + proc_nodes_offset)
            rb_walker = rb_tree.RbTreeWalker(self.ramdump)
            rb_walker.walk(proc_nodes_node, self.binder_nodes_walker)

            print (self.SEPARATOR_SUB2, file=self.outfd)
            print (self.TITLE_REF, file=self.outfd)

            # Walk binder_proc.refs_by_desc rb_tree.
            proc_refs_offset = self.ramdump.field_offset('struct binder_proc', 'refs_by_desc')
            proc_refs_node   = self.ramdump.read_word(proc + proc_refs_offset)
            rb_walker = rb_tree.RbTreeWalker(self.ramdump)
            rb_walker.walk(proc_refs_node, self.binder_refs_walker)

            # Get the next binder_node to parse.
            proc_node = self.ramdump.read_word(proc_node + hlist_next_offset)

        print (self.SEPARATOR, file=self.outfd)

    def parse(self):
        self.outfd = open(self.ramdump.outdir + "/binder_output.txt", "w")
        self.parse_binder_procs()
        self.outfd.close()
        print_out_str("--- Wrote the output to binder_output.txt")
