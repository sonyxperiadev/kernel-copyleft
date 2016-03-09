/*
 * License terms: GNU General Public License (GPL) version 2
 *
 * Author: Peter Enderborg <Peter.Enderborg@sonymobile.com>
 */
/*
 * Copyright (C) 2015 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM lmk

#if !defined(_TRACE_LMK_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_LMK_H

#include <linux/types.h>
#include <linux/tracepoint.h>

DECLARE_EVENT_CLASS(lmk_kill,
	TP_PROTO(int pid,
		 const char *comm,
		 int score,
		 int size,
		 int gfp_mask),
	TP_ARGS(pid, comm, score, size, gfp_mask),

	TP_STRUCT__entry(
		__field(int, pid)
		__field(const char *, comm)
		__field(int, score)
		__field(int, size)
		__field(int, gfp_mask)
	),

	TP_fast_assign(
		__entry->pid		= pid;
		__entry->comm		= comm;
		__entry->score		= score;
		__entry->size		= size;
		__entry->gfp_mask	= gfp_mask;
	),

	TP_printk("pid=%d comm=%s score=%d size=%d gfp_mask=%d",
		  __entry->pid, __entry->comm,
		  __entry->score, __entry->size,
		  __entry->gfp_mask)

);

DEFINE_EVENT(lmk_kill, lmk_sigkill,
	TP_PROTO(int pid,
		 const char *comm,
		 int score,
		 int size,
		 int gfp_mask),

	TP_ARGS(pid, comm, score, size, gfp_mask)
);

DECLARE_EVENT_CLASS(lmk_remain,
		    TP_PROTO(int rem, int nr_to_scan, int gfp_mask),
		    TP_ARGS(rem, nr_to_scan, gfp_mask),

	TP_STRUCT__entry(
		__field(int, rem)
		__field(int, nr_to_scan)
		__field(int, gfp_mask)
	),

	TP_fast_assign(
		       __entry->rem		= rem;
		       __entry->nr_to_scan	= nr_to_scan;
		       __entry->gfp_mask	= gfp_mask;
	),

	TP_printk("rem=%d scan=%d gfp_mask=%d",
		  __entry->rem, __entry->nr_to_scan, __entry->gfp_mask)

);

DEFINE_EVENT(lmk_remain, lmk_remain_scan,
	     TP_PROTO(int rem, int nr_to_scan, int gfp_mask),

	     TP_ARGS(rem, nr_to_scan, gfp_mask)
);


#endif /* _TRACE_LMK_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
