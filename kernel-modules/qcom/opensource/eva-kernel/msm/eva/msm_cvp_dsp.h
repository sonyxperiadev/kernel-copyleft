/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef MSM_CVP_DSP_H
#define MSM_CVP_DSP_H

#include <linux/types.h>
#include <linux/refcount.h>
#include "msm_cvp_debug.h"
#include "cvp_core_hfi.h"

#include <linux/pid.h>
#include <linux/sched.h>

#ifdef CVP_FASTRPC_ENABLED
#include <fastrpc.h>
#else
struct fastrpc_device {
	int handle;
};

enum fastrpc_driver_status {
	FASTRPC_CVP_B,
};

enum fastrpc_driver_invoke_nums {
	FASTRPC_DEV_MAP_DMA = 1,
	FASTRPC_DEV_UNMAP_DMA,
	FASTRPC_DEV_GET_HLOS_PID,
};

struct fastrpc_driver {
	struct device_driver driver;
	int handle;
	int (*probe)(struct fastrpc_device *dev);
	int (*callback)(struct fastrpc_device *dev,
			enum fastrpc_driver_status status);
};
#endif	/* End of CVP_FASTRPC_ENABLED */

#define CVP_APPS_DSP_GLINK_GUID "cvp-glink-apps-dsp"
#define CVP_APPS_DSP_SMD_GUID "cvp-smd-apps-dsp"

#define VMID_CDSP_Q6 (30)
#define HLOS_VM_NUM 1
#define DSP_VM_NUM 2
#define CVP_DSP_MAX_RESERVED 5
#define CVP_DSP2CPU_RESERVED 8
#define CVP_DSP_RESPONSE_TIMEOUT 1000
#define CVP_INVALID_RPMSG_TYPE 0xBADDFACE
#define MAX_FRAME_BUF_NUM 16

#define BITPTRSIZE32 (4)
#define BITPTRSIZE64 (8)
#define HIGH32                      (0xFFFFFFFF00000000LL)
#define LOW32                       (0xFFFFFFFFLL)

#define CVP_FASTRPC_DRIVER_NAME_SIZE    16

/* Supports up to 8 DSP sessions in 8 processes */
#define MAX_DSP_SESSION_NUM			(8)
#define MAX_FASTRPC_DRIVER_NUM		(MAX_DSP_SESSION_NUM)

int cvp_dsp_device_init(void);
void cvp_dsp_device_exit(void);
void cvp_dsp_send_hfi_queue(void);
void cvp_dsp_init_hfi_queue_hdr(struct iris_hfi_device *device);

enum CPU2DSP_STATUS {
	CPU2DSP_SUCCESS = 0,
	CPU2DSP_EFAIL = 1,
	CPU2DSP_EFATAL = 2,
	CPU2DSP_EUNAVAILABLE = 3,
	CPU2DSP_EINVALSTATE = 4,
	CPU2DSP_EUNSUPPORTED = 5,
};

enum CVP_DSP_COMMAND {
	CPU2DSP_SEND_HFI_QUEUE = 0,
	CPU2DSP_SUSPEND = 1,
	CPU2DSP_RESUME = 2,
	CPU2DSP_SHUTDOWN = 3,
	CPU2DSP_REGISTER_BUFFER = 4,
	CPU2DSP_DEREGISTER_BUFFER = 5,
	CPU2DSP_INIT = 6,
	CPU2DSP_SET_DEBUG_LEVEL = 7,
	CPU2DSP_MAX_CMD = 8,
	DSP2CPU_POWERON = 11,
	DSP2CPU_POWEROFF = 12,
	DSP2CPU_CREATE_SESSION = 13,
	DSP2CPU_DETELE_SESSION = 14,
	DSP2CPU_POWER_REQUEST = 15,
	DSP2CPU_POWER_CANCEL = 16,
	DSP2CPU_REGISTER_BUFFER = 17,
	DSP2CPU_DEREGISTER_BUFFER = 18,
	DSP2CPU_MEM_ALLOC = 19,
	DSP2CPU_MEM_FREE = 20,
	DSP2CPU_START_SESSION = 21,
	DSP2CPU_STOP_SESSION = 22,
	CVP_DSP_MAX_CMD = 23,
};

struct eva_power_req {
	uint32_t clock_fdu;
	uint32_t clock_ica;
	uint32_t clock_od;
	uint32_t clock_mpu;
	uint32_t clock_fw;
	uint32_t bw_ddr;
	uint32_t bw_sys_cache;
	uint32_t op_clock_fdu;
	uint32_t op_clock_ica;
	uint32_t op_clock_od;
	uint32_t op_clock_mpu;
	uint32_t op_clock_fw;
	uint32_t op_bw_ddr;
	uint32_t op_bw_sys_cache;
};

struct eva_mem_remote {
	uint32_t type;
	uint32_t size;
	uint32_t fd;
	uint32_t offset;
	uint32_t index;
	uint32_t iova;
	uint32_t dsp_remote_map;
	uint64_t v_dsp_addr;
};

/*
 * command: defined as a packet initiated from one party.
 * message: defined as a packet sent as response to a command
 */

/*
 * cvp_dsp_cmd_msg contains
 * the message sent from CPU to DSP
 * or
 * the command sent from CPU to DSP
 */
struct cvp_dsp_cmd_msg {
	uint32_t type;
	int32_t ret;
	uint64_t msg_ptr;
	uint32_t msg_ptr_len;
	uint32_t buff_fd_iova;
	uint32_t buff_index;
	uint32_t buff_size;
	uint32_t session_id;
	int32_t ddr_type;
	uint32_t buff_fd;
	uint32_t buff_offset;
	uint32_t buff_fd_size;

	uint32_t eva_dsp_debug_mask;

	/* Create Session */
	uint32_t session_cpu_low;
	uint32_t session_cpu_high;

	struct eva_mem_remote sbuf;

	uint32_t reserved1;
	uint32_t reserved2;
};

/* cvp_dsp_rsp_msg contains the message sent from DSP to CPU */
struct cvp_dsp_rsp_msg {
	uint32_t type;
	int32_t ret;
	uint32_t dsp_state;
	uint32_t reserved[CVP_DSP_MAX_RESERVED - 1];
};

/* cvp_dsp2cpu_cmd contains the command sent from DSP to cpu*/
struct cvp_dsp2cpu_cmd {
	uint32_t type;
	uint32_t ver;
	uint32_t len;

	/* Create Session */
	uint32_t session_type;
	uint32_t kernel_mask;
	uint32_t session_prio;
	uint32_t is_secure;
	uint32_t dsp_access_mask;

	uint32_t session_id;
	uint32_t session_cpu_low;
	uint32_t session_cpu_high;
	int32_t pid;
	struct eva_power_req power_req;
	struct eva_mem_remote sbuf;

	uint32_t data[CVP_DSP2CPU_RESERVED];
};

struct driver_name {
    uint32_t status;
    char name[CVP_FASTRPC_DRIVER_NAME_SIZE];
};

enum DRIVER_NAME_STATUS {
	DRIVER_NAME_INVALID = 0,
	DRIVER_NAME_AVAILABLE = 1,
	DRIVER_NAME_USED = 2,
};

struct cvp_dsp_fastrpc_driver_entry {
	struct list_head list;
	uint32_t handle;	/*handle is not PID*/
	uint32_t session_cnt;
	uint32_t driver_name_idx;
	atomic_t refcount;
	struct fastrpc_driver cvp_fastrpc_driver;
	struct fastrpc_device *cvp_fastrpc_device;
	struct completion fastrpc_probe_completion;
	/* all dsp sessions list */
	struct msm_cvp_list dsp_sessions;
};

struct cvp_dsp_apps {
	/*
	 * tx_lock for sending CPU2DSP cmds or msgs
	 * and dsp state change
	 */
	struct mutex tx_lock;
	/* rx_lock for receiving DSP2CPU cmds or msgs */
	struct mutex rx_lock;
	struct mutex driver_name_lock;
	struct rpmsg_device *chan;
	uint32_t state;
	uint32_t debug_mask;
	bool hyp_assigned;
	uint64_t addr;
	uint32_t size;
	struct completion completions[CPU2DSP_MAX_CMD + 1];
	struct cvp_dsp2cpu_cmd pending_dsp2cpu_cmd;
	struct cvp_dsp_rsp_msg pending_dsp2cpu_rsp;
	struct task_struct *dsp_thread;
	/* dsp buffer mapping, set of dma function pointer */
	const struct file_operations *dmabuf_f_op;
	uint32_t buf_num;
	struct msm_cvp_list fastrpc_driver_list;
	struct driver_name cvp_fastrpc_name[MAX_FASTRPC_DRIVER_NUM];
};

#define EVA_TRACE_MAX_SESSION_NUM       16
#define EVA_TRACE_MAX_INSTANCE_NUM      6
#define EVA_TRACE_MAX_BUF_NUM           256

#define CONFIG_SIZE_IN_BYTES        2048
#define CONFIG_SIZE_IN_WORDS        (CONFIG_SIZE_IN_BYTES >> 2)

// iova is eva_dsp_buf->iova
// pkt_type is frame packet type using the buffer
// buf_idx is the index of the buffer in a frame packet
// transaction_id is the transaction id of frame packet
struct cvp_dsp_trace_buf {
	u32	iova;
	u32	pkt_type;
	u32	buf_idx;
	u32	transaction_id;
	u32	fd;
};

// Saving config packet for each intance
struct cvp_dsp_trace_instance {
	u32    feature_type;
	u32    config_pkt[CONFIG_SIZE_IN_WORDS];
};

struct cvp_dsp_trace_session {
	u32                session_id;
	u32                buf_cnt;
	u32                inst_cnt;
	struct cvp_dsp_trace_instance  instance[EVA_TRACE_MAX_INSTANCE_NUM];
	struct cvp_dsp_trace_buf       buf[EVA_TRACE_MAX_BUF_NUM];
};

struct cvp_dsp_trace {
	struct cvp_dsp_trace_session   sessions[EVA_TRACE_MAX_SESSION_NUM];
};

extern struct cvp_dsp_apps gfa_cv;
/*
 * API for CVP driver to suspend CVP session during
 * power collapse
 */
int cvp_dsp_suspend(bool force);

/*
 * API for CVP driver to resume CVP session during
 * power collapse
 */
int cvp_dsp_resume(void);

/*
 * API for CVP driver to shutdown CVP session during
 * cvp subsystem error.
 */
int cvp_dsp_shutdown(void);

int cvp_dsp_fastrpc_unmap(uint32_t handle, struct cvp_internal_buf *buf);

int cvp_dsp_del_sess(uint32_t handle, struct msm_cvp_inst *inst);

void cvp_dsp_send_debug_mask(void);

#endif // MSM_CVP_DSP_H

