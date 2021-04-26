/* SPDX-License-Identifier: GPL-2.0 */
/*
 * NOTE: This file has been modified by Sony Mobile Communications Inc.
 * Modifications are Copyright (c) 2019 Sony Mobile Communications Inc,
 * and licensed under the license of the file.
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM task

#if !defined(_TRACE_TASK_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_TASK_H
#include <linux/tracepoint.h>

TRACE_EVENT(task_newtask,

	TP_PROTO(struct task_struct *task, unsigned long clone_flags),

	TP_ARGS(task, clone_flags),

	TP_STRUCT__entry(
		__field(	pid_t,	pid)
		__array(	char,	comm, TASK_COMM_LEN)
		__field( unsigned long, clone_flags)
		__field(	short,	oom_score_adj)
	),

	TP_fast_assign(
		__entry->pid = task->pid;
		memcpy(__entry->comm, task->comm, TASK_COMM_LEN);
		__entry->clone_flags = clone_flags;
		__entry->oom_score_adj = task->signal->oom_score_adj;
	),

	TP_printk("pid=%d comm=%s clone_flags=%lx oom_score_adj=%hd",
		__entry->pid, __entry->comm,
		__entry->clone_flags, __entry->oom_score_adj)
);

TRACE_EVENT(task_rename,

	TP_PROTO(struct task_struct *task, const char *comm),

	TP_ARGS(task, comm),

	TP_STRUCT__entry(
		__field(	pid_t,	pid)
		__array(	char, oldcomm,  TASK_COMM_LEN)
		__array(	char, newcomm,  TASK_COMM_LEN)
		__field(	short,	oom_score_adj)
	),

	TP_fast_assign(
		__entry->pid = task->pid;
		memcpy(entry->oldcomm, task->comm, TASK_COMM_LEN);
		strlcpy(entry->newcomm, comm, TASK_COMM_LEN);
		__entry->oom_score_adj = task->signal->oom_score_adj;
	),

	TP_printk("pid=%d oldcomm=%s newcomm=%s oom_score_adj=%hd",
		__entry->pid, __entry->oldcomm,
		__entry->newcomm, __entry->oom_score_adj)
);

TRACE_EVENT(task_exit,

	TP_PROTO(struct task_struct *task),

	TP_ARGS(task),

	TP_STRUCT__entry(
		__field(pid_t,	pid)
		__field(short,	oom_score_adj)
		__field(int,	exit_signal)
		__field(int,	exit_code)
		__field(int,	exit_state)
		__field(bool,	leader)
		__array(char,	comm, TASK_COMM_LEN)
	),

	TP_fast_assign(
		__entry->pid = task->pid;
		__entry->oom_score_adj = task->signal->oom_score_adj;
		__entry->exit_signal = task->exit_signal;
		__entry->exit_code = task->exit_code;
		__entry->exit_state = task->exit_state;
		__entry->leader = thread_group_leader(task);
		memcpy(__entry->comm, task->comm, TASK_COMM_LEN);
	),

	TP_printk("pid=%d oom_score_adj=%hd exit_signal=%d exit_code=%d exit_state=0x%x leader=%d comm=%s",
		  __entry->pid,
		  __entry->oom_score_adj, __entry->exit_signal,
		  __entry->exit_code, __entry->exit_state,
		  __entry->leader, __entry->comm)
);

#endif

/* This part must be outside protection */
#include <trace/define_trace.h>
