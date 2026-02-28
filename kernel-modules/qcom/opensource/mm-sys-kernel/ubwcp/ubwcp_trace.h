/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#if !defined(TRACE_UBWCP_H) || defined(TRACE_HEADER_MULTI_READ)
#define TRACE_UBWCP_H

#undef TRACE_SYSTEM
#define TRACE_SYSTEM ubwcp

/* Path must be relative to location of 'define_trace.h' header in kernel */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../../../vendor/qcom/opensource/mm-sys-kernel/ubwcp

/* Name of trace header file */
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE ubwcp_trace

#include <linux/tracepoint.h>

struct dma_buf;
struct platform_device;

DECLARE_EVENT_CLASS(ubwcp_platform_device_event,

	TP_PROTO(struct platform_device *pdev),

	TP_ARGS(pdev),

	TP_STRUCT__entry(
		__field(struct platform_device *, pdev)
	),

	TP_fast_assign(
		__entry->pdev = pdev;
	),

	TP_printk("platform_device:0x%lx",
		__entry->pdev)
);

DEFINE_EVENT(ubwcp_platform_device_event, ubwcp_probe,

	TP_PROTO(struct platform_device *pdev),

	TP_ARGS(pdev)
);

DEFINE_EVENT(ubwcp_platform_device_event, ubwcp_remove,

	TP_PROTO(struct platform_device *pdev),

	TP_ARGS(pdev)
);

DECLARE_EVENT_CLASS(ubwcp_dmabuf_event,

	TP_PROTO(struct dma_buf *dbuf_addr),

	TP_ARGS(dbuf_addr),

	TP_STRUCT__entry(
		__field(struct dma_buf *, dbuf_addr)
	),

	TP_fast_assign(
		__entry->dbuf_addr = dbuf_addr;
	),

	TP_printk("dma-buf:0x%lx",
		__entry->dbuf_addr)
);

DECLARE_EVENT_CLASS(ubwcp_size_event,

	TP_PROTO(size_t size),

	TP_ARGS(size),

	TP_STRUCT__entry(
		__field(size_t, size)
	),

	TP_fast_assign(
		__entry->size = size;
	),

	TP_printk("size:%zu", __entry->size)
);

DEFINE_EVENT(ubwcp_dmabuf_event, ubwcp_init_buffer_start,

	TP_PROTO(struct dma_buf *dbuf_addr),

	TP_ARGS(dbuf_addr)
);

DEFINE_EVENT(ubwcp_dmabuf_event, ubwcp_init_buffer_end,

	TP_PROTO(struct dma_buf *dbuf_addr),

	TP_ARGS(dbuf_addr)
);

DEFINE_EVENT(ubwcp_size_event, ubwcp_memremap_pages_start,

	TP_PROTO(size_t size),

	TP_ARGS(size)
);

DEFINE_EVENT(ubwcp_size_event, ubwcp_memremap_pages_end,

	TP_PROTO(size_t size),

	TP_ARGS(size)
);

DEFINE_EVENT(ubwcp_dmabuf_event, ubwcp_set_buf_attrs_start,

	TP_PROTO(struct dma_buf *dbuf_addr),

	TP_ARGS(dbuf_addr)
);

DEFINE_EVENT(ubwcp_dmabuf_event, ubwcp_set_buf_attrs_end,

	TP_PROTO(struct dma_buf *dbuf_addr),

	TP_ARGS(dbuf_addr)
);

DEFINE_EVENT(ubwcp_dmabuf_event, ubwcp_lock_start,

	TP_PROTO(struct dma_buf *dbuf_addr),

	TP_ARGS(dbuf_addr)
);

DEFINE_EVENT(ubwcp_dmabuf_event, ubwcp_lock_end,

	TP_PROTO(struct dma_buf *dbuf_addr),

	TP_ARGS(dbuf_addr)
);

DEFINE_EVENT(ubwcp_dmabuf_event, ubwcp_unlock_start,

	TP_PROTO(struct dma_buf *dbuf_addr),

	TP_ARGS(dbuf_addr)
);

DEFINE_EVENT(ubwcp_dmabuf_event, ubwcp_unlock_end,

	TP_PROTO(struct dma_buf *dbuf_addr),

	TP_ARGS(dbuf_addr)
);

DEFINE_EVENT(ubwcp_size_event, ubwcp_offline_sync_start,

	TP_PROTO(size_t size),

	TP_ARGS(size)
);

DEFINE_EVENT(ubwcp_size_event, ubwcp_offline_sync_end,

	TP_PROTO(size_t size),

	TP_ARGS(size)
);

DEFINE_EVENT(ubwcp_size_event, ubwcp_dma_sync_single_for_device_start,

	TP_PROTO(size_t size),

	TP_ARGS(size)
);

DEFINE_EVENT(ubwcp_size_event, ubwcp_dma_sync_single_for_device_end,

	TP_PROTO(size_t size),

	TP_ARGS(size)
);

DEFINE_EVENT(ubwcp_size_event, ubwcp_dma_sync_single_for_cpu_start,

	TP_PROTO(size_t size),

	TP_ARGS(size)
);

DEFINE_EVENT(ubwcp_size_event, ubwcp_dma_sync_single_for_cpu_end,

	TP_PROTO(size_t size),

	TP_ARGS(size)
);

DEFINE_EVENT(ubwcp_size_event, ubwcp_hw_flush_start,

	TP_PROTO(size_t size),

	TP_ARGS(size)
);

DEFINE_EVENT(ubwcp_size_event, ubwcp_hw_flush_end,

	TP_PROTO(size_t size),

	TP_ARGS(size)
);

DEFINE_EVENT(ubwcp_size_event, ubwcp_memunmap_pages_start,

	TP_PROTO(size_t size),

	TP_ARGS(size)
);

DEFINE_EVENT(ubwcp_size_event, ubwcp_memunmap_pages_end,

	TP_PROTO(size_t size),

	TP_ARGS(size)
);

DEFINE_EVENT(ubwcp_size_event, ubwcp_set_direct_map_range_uncached_start,

	TP_PROTO(size_t size),

	TP_ARGS(size)
);

DEFINE_EVENT(ubwcp_size_event, ubwcp_set_direct_map_range_uncached_end,

	TP_PROTO(size_t size),

	TP_ARGS(size)
);

DEFINE_EVENT(ubwcp_dmabuf_event, ubwcp_free_buffer_start,

	TP_PROTO(struct dma_buf *dbuf_addr),

	TP_ARGS(dbuf_addr)
);

DEFINE_EVENT(ubwcp_dmabuf_event, ubwcp_free_buffer_end,

	TP_PROTO(struct dma_buf *dbuf_addr),

	TP_ARGS(dbuf_addr)
);

#endif

/* This part must be outside protection */
#include <trace/define_trace.h>
