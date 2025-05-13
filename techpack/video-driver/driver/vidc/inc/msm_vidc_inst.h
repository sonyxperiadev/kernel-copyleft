/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _MSM_VIDC_INST_H_
#define _MSM_VIDC_INST_H_

#include "msm_vidc_internal.h"
#include "msm_vidc_memory.h"
#include "msm_vidc_state.h"
#include "hfi_property.h"

struct msm_vidc_inst;

#define call_session_op(c, op, ...)			\
	(((c) && (c)->session_ops && (c)->session_ops->op) ? \
	((c)->session_ops->op(__VA_ARGS__)) : 0)

struct msm_vidc_session_ops {
	u64 (*calc_freq)(struct msm_vidc_inst *inst, u32 data_size);
	int (*calc_bw)(struct msm_vidc_inst *inst,
		       struct vidc_bus_vote_data *vote_data);
	int (*decide_work_route)(struct msm_vidc_inst *inst);
	int (*decide_work_mode)(struct msm_vidc_inst *inst);
	int (*decide_quality_mode)(struct msm_vidc_inst *inst);
	int (*buffer_size)(struct msm_vidc_inst *inst, enum msm_vidc_buffer_type type);
	int (*min_count)(struct msm_vidc_inst *inst, enum msm_vidc_buffer_type type);
	int (*extra_count)(struct msm_vidc_inst *inst, enum msm_vidc_buffer_type type);
	int (*ring_buf_count)(struct msm_vidc_inst *inst, u32 data_size);
};

struct msm_vidc_mem_list_info {
	struct msm_vidc_mem_list        bin;
	struct msm_vidc_mem_list        arp;
	struct msm_vidc_mem_list        comv;
	struct msm_vidc_mem_list        non_comv;
	struct msm_vidc_mem_list        line;
	struct msm_vidc_mem_list        dpb;
	struct msm_vidc_mem_list        persist;
	struct msm_vidc_mem_list        vpss;
	struct msm_vidc_mem_list        partial_data;
};

struct msm_vidc_buffers_info {
	struct msm_vidc_buffers        input;
	struct msm_vidc_buffers        output;
	struct msm_vidc_buffers        read_only;
	struct msm_vidc_buffers        input_meta;
	struct msm_vidc_buffers        output_meta;
	struct msm_vidc_buffers        bin;
	struct msm_vidc_buffers        arp;
	struct msm_vidc_buffers        comv;
	struct msm_vidc_buffers        non_comv;
	struct msm_vidc_buffers        line;
	struct msm_vidc_buffers        dpb;
	struct msm_vidc_buffers        persist;
	struct msm_vidc_buffers        vpss;
	struct msm_vidc_buffers        partial_data;
};

struct buf_queue {
	struct vb2_queue *vb2q;
};

struct msm_vidc_inst {
	struct list_head                   list;
	struct mutex                       lock;
	struct mutex                       ctx_q_lock;
	struct mutex                       client_lock;
	enum msm_vidc_state                state;
	int                              (*event_handle)(struct msm_vidc_inst *inst,
							 enum msm_vidc_event event,
							 void *data);
	enum msm_vidc_sub_state            sub_state;
	char                               sub_state_name[MAX_NAME_LENGTH];
	enum msm_vidc_domain_type          domain;
	enum msm_vidc_codec_type           codec;
	void                              *core;
	struct kref                        kref;
	u32                                session_id;
	u8                                 debug_str[24];
	void                              *packet;
	u32                                packet_size;
	struct v4l2_format                 fmts[MAX_PORT];
	struct v4l2_ctrl_handler           ctrl_handler;
	struct v4l2_fh                     fh;
	struct v4l2_m2m_dev               *m2m_dev;
	struct v4l2_m2m_ctx               *m2m_ctx;
	u32                                num_ctrls;
	enum hfi_rate_control              hfi_rc_type;
	enum hfi_layer_encoding_type       hfi_layer_type;
	bool                               request;
	struct buf_queue                   bufq[MAX_PORT];
	struct msm_vidc_rectangle          crop;
	struct msm_vidc_rectangle          compose;
	struct msm_vidc_power              power;
	struct vidc_bus_vote_data          bus_data;
	struct msm_memory_pool             pool[MSM_MEM_POOL_MAX];
	struct msm_vidc_buffers_info       buffers;
	struct msm_vidc_mem_list_info      mem_info;
	struct msm_vidc_timestamps         timestamps;
	struct msm_vidc_timestamps         ts_reorder; /* struct msm_vidc_timestamp */
	struct msm_vidc_subscription_params       subcr_params[MAX_PORT];
	struct msm_vidc_hfi_frame_info     hfi_frame_info;
	struct msm_vidc_decode_batch       decode_batch;
	struct msm_vidc_decode_vpp_delay   decode_vpp_delay;
	struct msm_vidc_session_idle       session_idle;
	struct delayed_work                stats_work;
	struct work_struct                 stability_work;
	struct msm_vidc_stability          stability;
	struct workqueue_struct           *workq;
	struct list_head                   enc_input_crs;
	struct list_head                   dmabuf_tracker; /* struct msm_memory_dmabuf */
	struct list_head                   input_timer_list; /* struct msm_vidc_input_timer */
	struct list_head                   caps_list;
	struct list_head                   children_list; /* struct msm_vidc_inst_cap_entry */
	struct list_head                   firmware_list; /* struct msm_vidc_inst_cap_entry */
	struct list_head                   pending_pkts; /* struct hfi_pending_packet */
	struct list_head                   fence_list; /* struct msm_vidc_fence */
	struct list_head                   buffer_stats_list; /* struct msm_vidc_buffer_stats */
	bool                               once_per_session_set;
	bool                               ipsc_properties_set;
	bool                               opsc_properties_set;
	bool                               caps_list_prepared;
	struct dentry                     *debugfs_root;
	struct msm_vidc_debug              debug;
	struct debug_buf_count             debug_count;
	struct msm_vidc_statistics         stats;
	struct msm_vidc_inst_cap           capabilities[INST_CAP_MAX + 1];
	struct completion                  completions[MAX_SIGNAL];
	struct msm_vidc_fence_context      fence_context;
	bool                               active;
	u64                                last_qbuf_time_ns;
	u64                                initial_time_us;
	u32                                max_input_data_size;
	u32                                dpb_list_payload[MAX_DPB_LIST_ARRAY_SIZE];
	bool                               input_dpb_list_enabled;
	bool                               output_dpb_list_enabled;
	u32                                auto_framerate;
	u32                                max_rate;
	bool                               has_bframe;
	bool                               ir_enabled;
	u32                                adjust_priority;
	bool                               iframe;
	u32                                fw_min_count;
};

#endif // _MSM_VIDC_INST_H_
