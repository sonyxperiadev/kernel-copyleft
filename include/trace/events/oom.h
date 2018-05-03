#undef TRACE_SYSTEM
#define TRACE_SYSTEM oom

#if !defined(_TRACE_OOM_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_OOM_H
#include <linux/tracepoint.h>

TRACE_EVENT(oom_score_adj_update,

	TP_PROTO(struct task_struct *task),

	TP_ARGS(task),

	TP_STRUCT__entry(
		__field(	pid_t,	pid)
		__array(	char,	comm,	TASK_COMM_LEN )
		__field(	short,	oom_score_adj)
	),

	TP_fast_assign(
		__entry->pid = task->pid;
		memcpy(__entry->comm, task->comm, TASK_COMM_LEN);
		__entry->oom_score_adj = task->signal->oom_score_adj;
	),

	TP_printk("pid=%d comm=%s oom_score_adj=%hd",
		__entry->pid, __entry->comm, __entry->oom_score_adj)
);

DECLARE_EVENT_CLASS(oom_kill,
	TP_PROTO(int pid,
		 const char *comm,
		 int score,
		 unsigned long size,
		 int gfp_mask),
	TP_ARGS(pid, comm, score, size, gfp_mask),

	TP_STRUCT__entry(
		__field(int, pid)
		__field(const char *, comm)
		__field(int, score)
		__field(unsigned long, size)
		__field(int, gfp_mask)
	),

	TP_fast_assign(
		__entry->pid		= pid;
		__entry->comm		= comm;
		__entry->score		= score;
		__entry->size		= size;
		__entry->gfp_mask	= gfp_mask;
	),

	TP_printk("pid=%d comm=%s score=%d size=%ld gfp_mask=%d",
		  __entry->pid, __entry->comm,
		  __entry->score, __entry->size,
		  __entry->gfp_mask)

);

DEFINE_EVENT(oom_kill, oom_sigkill,
	TP_PROTO(int pid,
		 const char *comm,
		 int score,
		 unsigned long size,
		 int gfp_mask),

	TP_ARGS(pid, comm, score, size, gfp_mask)
);



#endif

/* This part must be outside protection */
#include <trace/define_trace.h>
