/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021-2022 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2016-2019, The Linux Foundation. All rights reserved.
 */

#ifndef _SDE_FENCE_H_
#define _SDE_FENCE_H_

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/soc/qcom/msm_hw_fence.h>

#ifndef CHAR_BIT
#define CHAR_BIT 8 /* define this if limits.h not available */
#endif

#define HW_FENCE_TRIGGER_SEL_CTRL_DONE       0x0
#define HW_FENCE_TRIGGER_SEL_PROG_LINE_COUNT 0x1

#define SDE_INPUT_HW_FENCE_TIMESTAMP         BIT(0)
#define SDE_OUTPUT_HW_FENCE_TIMESTAMP        BIT(1)

#define SDE_FENCE_NAME_SIZE	24

#define MAX_SDE_HFENCE_OUT_SIGNAL_PING_PONG 2
/**
 * struct sde_fence_context - release/retire fence context/timeline structure
 * @commit_count: Number of detected commits since bootup
 * @done_count: Number of completed commits since bootup
 * @drm_id: ID number of owning DRM Object
 * @ref: kref counter on timeline
 * @lock: spinlock for fence counter protection
 * @list_lock: spinlock for timeline protection
 * @context: fence context
 * @list_head: fence list to hold all the fence created on this context
 * @name: name of fence context/timeline
 */
struct sde_fence_context {
	unsigned int commit_count;
	unsigned int done_count;
	uint32_t drm_id;
	struct kref kref;
	spinlock_t lock;
	spinlock_t list_lock;
	u64 context;
	struct list_head fence_list_head;
	char name[SDE_FENCE_NAME_SIZE];
};

/**
 * enum sde_fence_event - sde fence event as hint fence operation
 * @SDE_FENCE_SIGNAL: Signal the fence cleanly with current timeline
 * @SDE_FENCE_RESET_TIMELINE: Reset timeline of the fence context
 * @SDE_FENCE_SIGNAL: Signal the fence but indicate error throughfence status
 */
enum sde_fence_event {
	SDE_FENCE_SIGNAL,
	SDE_FENCE_RESET_TIMELINE,
	SDE_FENCE_SIGNAL_ERROR
};

/**
 * struct sde_hw_fence_data - contains the information of each display-client of the hw-fences
 *                       to communicate with the fence controller.
 * @client_id: client_id enum for the display driver.
 * @hw_fence_client_id: client_id enum for the hw-fence driver.
 * @mem_descriptor: memory descriptor with the hfi for the rx/tx queues mapping.
 * @ipcc_in_client: ipcc client triggering the signal: IN_CLIENT (APPS) -> DPU
 * @ipcc_in_signal: ipcc signal triggered from client to dpu: IN_SIGNAL (APPS) -> DPU
 * @ipcc_out_signal_pp: output signal from dpu to fctl, ping-pongs between two signals
 * @ipcc_out_signal_pp_idx: index of the output signal ping-pong
 * @ipcc_out_client: destination client id (APPS for the FCTL)
 * @ipcc_this_client: ipcc dpu client id (For Waipio: APPS, For Kailua: DPU HW)
 * @dma_context: per client dma context used to create join fences
 * @hw_fence_array_seqno: per-client seq number counter for join fences
 */
struct sde_hw_fence_data {
	int client_id;
	enum hw_fence_client_id hw_fence_client_id;
	void *hw_fence_handle;
	struct msm_hw_fence_mem_addr mem_descriptor;
	u32 ipcc_in_client;
	u32 ipcc_in_signal;
	u32 ipcc_out_signal_pp[MAX_SDE_HFENCE_OUT_SIGNAL_PING_PONG];
	u32 ipcc_out_signal_pp_idx;
	u32 ipcc_out_client;
	u32 ipcc_this_client;
	u64 dma_context;
	u32 hw_fence_array_seqno;
};

#if IS_ENABLED(CONFIG_SYNC_FILE)
/**
 * sde_sync_get - Query sync fence object from a file handle
 *
 * On success, this function also increments the refcount of the sync fence
 *
 * @fd: Integer sync fence handle
 *
 * Return: Pointer to sync fence object, or NULL
 */
void *sde_sync_get(uint64_t fd);

/**
 * sde_sync_put - Releases a sync fence object acquired by @sde_sync_get
 *
 * This function decrements the sync fence's reference count; the object will
 * be released if the reference count goes to zero.
 *
 * @fence: Pointer to sync fence
 */
void sde_sync_put(void *fence);

/**
 * sde_sync_wait - Query sync fence object from a file handle
 *
 * @fence: Pointer to sync fence
 * @timeout_ms: Time to wait, in milliseconds. Waits forever if timeout_ms < 0
 *
 * Return:
 * Zero if timed out
 * -ERESTARTSYS if wait interrupted
 * remaining jiffies in all other success cases.
 */
signed long sde_sync_wait(void *fence, long timeout_ms);

/**
 * sde_sync_get_name_prefix - get integer representation of fence name prefix
 *
 * @fence: Pointer to opaque fence structure
 *
 * Return: 32-bit integer containing first 4 characters of fence name,
 *         big-endian notation
 */
uint32_t sde_sync_get_name_prefix(void *fence);

/**
 * sde_fence_init - initialize fence object
 *
 * @drm_id: ID number of owning DRM Object
 * @name: Timeline name
 *
 * Returns: fence context object on success
 */
struct sde_fence_context *sde_fence_init(const char *name,
		uint32_t drm_id);

/**
 * sde_fence_hw_fence_init - initialize hw-fence clients
 *
 * @hw_ctl: hw ctl client to init.
 * @use_ipcc: boolean to indicate if hw should use dpu ipcc signals.
 *
 * Returns: Zero on success, otherwise returns an error code.
 */
int sde_hw_fence_init(struct sde_hw_ctl *hw_ctl, bool use_dpu_ipcc);

/**
 * sde_fence_hw_fence_deinit - deinitialize hw-fence clients
 *
 * @hw_ctl: hw ctl client to init.
 */
void sde_hw_fence_deinit(struct sde_hw_ctl *hw_ctl);

/**
 * sde_fence_register_hw_fences_wait - registers dpu-client for wait on hw fence or fences
 *
 * @hw_ctl: hw ctl client used to register for wait.
 * @fences: list of dma-fences that have hw-fence support to wait-on
 * @num_fences: number of fences in the above list
 *
 * Returns: Zero on success, otherwise returns an error code.
 */
int sde_fence_register_hw_fences_wait(struct sde_hw_ctl *hw_ctl, struct dma_fence **fences,
	u32 num_fences);

/**
 * sde_fence_update_hw_fences_txq - updates the hw-fence txq with the list of hw-fences to signal
 *                                  upon triggering the ipcc signal.
 *
 * @ctx: sde fence context
 * @vid_mode: is video-mode update
 * @line_count: prog line count value, must be non-zero
 *
 * Returns: Zero on success, otherwise returns an error code.
 */
int sde_fence_update_hw_fences_txq(struct sde_fence_context *ctx, bool vid_mode, u32 line_count,
	u32 debugfs_hw_fence);

/**
 * sde_fence_update_input_hw_fence_signal - updates input-fence ipcc signal in dpu and enables
 *                                  hw-fences for the ctl.
 *
 * @ctl: hw ctl to update the input-fence and enable hw-fences
 * @debugfs_hw_fence: hw-fence timestamp debugfs value
 * @hw_mdp: pointer to hw_mdp to get timestamp registers
 * @disable: bool to indicate if we should disable hw-fencing for this commit
 *
 * Returns: Zero on success, otherwise returns an error code.
 */
int sde_fence_update_input_hw_fence_signal(struct sde_hw_ctl *ctl, u32 debugfs_hw_fence,
	struct sde_hw_mdp *hw_mdp, bool disable);

/**
 * sde_fence_deinit - deinit fence container
 * @fence: Pointer fence container
 */
void sde_fence_deinit(struct sde_fence_context *fence);

/**
 * sde_fence_prepare - prepare to return fences for current commit
 * @fence: Pointer fence container
 * Returns: Zero on success
 */
void sde_fence_prepare(struct sde_fence_context *fence);
/**
 * sde_fence_create - create output fence object
 * @fence: Pointer fence container
 * @val: Pointer to output value variable, fence fd will be placed here
 * @offset: Fence signal commit offset, e.g., +1 to signal on next commit
 * @hw_ctl: Ctl for hw fences
 * Returns: Zero on success
 */
int sde_fence_create(struct sde_fence_context *fence, uint64_t *val,
				uint32_t offset, struct sde_hw_ctl *hw_ctl);

/**
 * sde_fence_signal - advance fence timeline to signal outstanding fences
 * @fence: Pointer fence container
 * @ts: fence timestamp
 * @fence_event: fence event to indicate nature of fence signal.
 * @hw_ctl: ctl to signal fences for the timeline rest event
 */
void sde_fence_signal(struct sde_fence_context *fence, ktime_t ts,
		enum sde_fence_event fence_event, struct sde_hw_ctl *hw_ctl);

/**
 * sde_fence_timeline_status - prints fence timeline status
 * @fence: Pointer fence container
 * @drm_obj Pointer to drm object associated with fence timeline
 */
void sde_fence_timeline_status(struct sde_fence_context *ctx,
					struct drm_mode_object *drm_obj);

/**
 * sde_fence_timeline_dump - utility to dump fence list info in debugfs node
 * @fence: Pointer fence container
 * @drm_obj: Pointer to drm object associated with fence timeline
 * @s: used to writing on debugfs node
 */
void sde_debugfs_timeline_dump(struct sde_fence_context *ctx,
		struct drm_mode_object *drm_obj, struct seq_file **s);

/**
 * sde_fence_timeline_status - dumps fence timeline in debugfs node
 * @fence: Pointer fence container
 * @s: used to writing on debugfs node
 */
void sde_fence_list_dump(struct dma_fence *fence, struct seq_file **s);

#else
static inline void *sde_sync_get(uint64_t fd)
{
	return NULL;
}

static inline void sde_sync_put(void *fence)
{
}

static inline signed long sde_sync_wait(void *fence, long timeout_ms)
{
	return 0;
}

static inline uint32_t sde_sync_get_name_prefix(void *fence)
{
	return 0x0;
}

static inline struct sde_fence_context *sde_fence_init(const char *name,
		uint32_t drm_id)
{
	/* do nothing */
	return NULL;
}

static inline void sde_fence_deinit(struct sde_fence_context *fence)
{
	/* do nothing */
}

static inline int sde_fence_get(struct sde_fence_context *fence, uint64_t *val)
{
	return -EINVAL;
}

static inline void sde_fence_signal(struct sde_fence_context *fence,
						ktime_t ts, bool reset_timeline)
{
	/* do nothing */
}

static inline void sde_fence_prepare(struct sde_fence_context *fence)
{
	/* do nothing */
}

static inline int sde_fence_create(struct sde_fence_context *fence,
						uint64_t *val, uint32_t offset)
{
	return 0;
}

static inline void sde_fence_timeline_status(struct sde_fence_context *ctx,
					struct drm_mode_object *drm_obj);
{
	/* do nothing */
}

void sde_debugfs_timeline_dump(struct sde_fence_context *ctx,
		struct drm_mode_object *drm_obj, struct seq_file **s)
{
	/* do nothing */
}

void sde_fence_list_dump(struct dma_fence *fence, struct seq_file **s)
{
	/* do nothing */
}

#endif /* IS_ENABLED(CONFIG_SW_SYNC) */

#endif /* _SDE_FENCE_H_ */
