// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2025, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "adreno.h"
#include "adreno_hfi.h"
#include "adreno_snapshot.h"
#include "adreno_sysfs.h"
#include "adreno_trace.h"
#include "kgsl_eventlog.h"
#include "kgsl_timeline.h"
#include "kgsl_trace.h"
#include <linux/msm_kgsl.h>
#include <linux/sched/clock.h>
#include <soc/qcom/msm_performance.h>

#define POLL_SLEEP_US 100

/*
 * Number of commands that can be queued in a context before it sleeps
 *
 * Our code that "puts back" a command from the context is much cleaner
 * if we are sure that there will always be enough room in the ringbuffer
 * so restrict the size of the context queue to ADRENO_CONTEXT_DRAWQUEUE_SIZE - 1
 */
static u32 _context_drawqueue_size = ADRENO_CONTEXT_DRAWQUEUE_SIZE - 1;

/* Number of milliseconds to wait for the context queue to clear */
static unsigned int _context_queue_wait = 10000;

/*
 * GFT throttle parameters. If GFT recovered more than
 * X times in Y ms invalidate the context and do not attempt recovery.
 * X -> _fault_throttle_burst
 * Y -> _fault_throttle_time
 */
static unsigned int _fault_throttle_time = 2000;
static unsigned int _fault_throttle_burst = 3;

/* Use a kmem cache to speed up allocations for dispatcher jobs */
static struct kmem_cache *jobs_cache;
/* Use a kmem cache to speed up allocations for inflight command objects */
static struct kmem_cache *obj_cache;

inline bool adreno_hwsched_context_queue_enabled(struct adreno_device *adreno_dev)
{
	return test_bit(ADRENO_HWSCHED_CONTEXT_QUEUE, &adreno_dev->hwsched.flags);
}

static bool is_cmdobj(struct kgsl_drawobj *drawobj)
{
	return (drawobj->type & CMDOBJ_TYPE);
}

static bool _check_context_queue(struct adreno_context *drawctxt, u32 count)
{
	bool ret;

	spin_lock(&drawctxt->lock);

	/*
	 * Wake up if there is room in the context or if the whole thing got
	 * invalidated while we were asleep
	 */

	if (kgsl_context_invalid(&drawctxt->base))
		ret = false;
	else
		ret = ((drawctxt->queued + count) < _context_drawqueue_size) ? 1 : 0;

	spin_unlock(&drawctxt->lock);

	return ret;
}

static void _pop_drawobj(struct adreno_context *drawctxt)
{
	drawctxt->drawqueue_head = DRAWQUEUE_NEXT(drawctxt->drawqueue_head,
		ADRENO_CONTEXT_DRAWQUEUE_SIZE);
	drawctxt->queued--;
}

static int _retire_syncobj(struct adreno_device *adreno_dev,
	struct kgsl_drawobj_sync *syncobj, struct adreno_context *drawctxt)
{
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;

	if (!kgsl_drawobj_events_pending(syncobj)) {
		_pop_drawobj(drawctxt);
		kgsl_drawobj_destroy(DRAWOBJ(syncobj));
		return 0;
	}

	/*
	 * If hardware fences are enabled, and this SYNCOBJ is backed by hardware fences,
	 * send it to the GMU
	 */
	if (test_bit(ADRENO_HWSCHED_HW_FENCE, &hwsched->flags) &&
		((syncobj->flags & KGSL_SYNCOBJ_HW)))
		return 1;

	/*
	 * If we got here, there are pending events for sync object.
	 * Start the canary timer if it hasnt been started already.
	 */
	if (!syncobj->timeout_jiffies) {
		syncobj->timeout_jiffies = jiffies + msecs_to_jiffies(5000);
			mod_timer(&syncobj->timer, syncobj->timeout_jiffies);
	}

	return -EAGAIN;
}

static bool _marker_expired(struct kgsl_drawobj_cmd *markerobj)
{
	struct kgsl_drawobj *drawobj = DRAWOBJ(markerobj);

	return (drawobj->flags & KGSL_DRAWOBJ_MARKER) &&
		kgsl_check_timestamp(drawobj->device, drawobj->context,
		markerobj->marker_timestamp);
}

/* Only retire the timestamp. The drawobj will be destroyed later */
static void _retire_timestamp_only(struct kgsl_drawobj *drawobj)
{
	struct kgsl_context *context = drawobj->context;
	struct kgsl_device *device = context->device;

	/*
	 * Write the start and end timestamp to the memstore to keep the
	 * accounting sane
	 */
	kgsl_sharedmem_writel(device->memstore,
		KGSL_MEMSTORE_OFFSET(context->id, soptimestamp),
		drawobj->timestamp);

	kgsl_sharedmem_writel(device->memstore,
		KGSL_MEMSTORE_OFFSET(context->id, eoptimestamp),
		drawobj->timestamp);

	msm_perf_events_update(MSM_PERF_GFX, MSM_PERF_RETIRED,
		pid_nr(context->proc_priv->pid),
		context->id, drawobj->timestamp,
		!!(drawobj->flags & KGSL_DRAWOBJ_END_OF_FRAME));

	if (drawobj->flags & KGSL_DRAWOBJ_END_OF_FRAME) {
		atomic64_inc(&drawobj->context->proc_priv->frame_count);
		atomic_inc(&drawobj->context->proc_priv->period->frames);
	}

	/* Retire pending GPU events for the object */
	kgsl_process_event_group(device, &context->events);
}

static void _retire_timestamp(struct kgsl_drawobj *drawobj)
{
	_retire_timestamp_only(drawobj);

	kgsl_drawobj_destroy(drawobj);
}

static int _retire_markerobj(struct adreno_device *adreno_dev, struct kgsl_drawobj_cmd *cmdobj,
	struct adreno_context *drawctxt)
{
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;

	if (_marker_expired(cmdobj)) {
		set_bit(CMDOBJ_MARKER_EXPIRED, &cmdobj->priv);
		/*
		 * There may be pending hardware fences that need to be signaled upon retiring
		 * this MARKER object. Hence, send it to the target specific layers to trigger
		 * the hardware fences.
		 */
		if (test_bit(ADRENO_HWSCHED_HW_FENCE, &hwsched->flags)) {
			_retire_timestamp_only(DRAWOBJ(cmdobj));
			return 1;
		}
		_pop_drawobj(drawctxt);
		_retire_timestamp(DRAWOBJ(cmdobj));
		return 0;
	}

	/*
	 * If the marker isn't expired but the SKIP bit
	 * is set then there are real commands following
	 * this one in the queue. This means that we
	 * need to dispatch the command so that we can
	 * keep the timestamp accounting correct. If
	 * skip isn't set then we block this queue
	 * until the dependent timestamp expires
	 */

	return test_bit(CMDOBJ_SKIP, &cmdobj->priv) ? 1 : -EAGAIN;
}

static int _retire_timelineobj(struct kgsl_drawobj *drawobj,
		struct adreno_context *drawctxt)
{
	_pop_drawobj(drawctxt);
	kgsl_drawobj_destroy(drawobj);
	return 0;
}

static int drawqueue_retire_bindobj(struct kgsl_drawobj *drawobj,
		struct adreno_context *drawctxt)
{
	struct kgsl_drawobj_bind *bindobj = BINDOBJ(drawobj);

	if (test_bit(KGSL_BINDOBJ_STATE_DONE, &bindobj->state)) {
		_pop_drawobj(drawctxt);
		_retire_timestamp(drawobj);
		return 0;
	}

	if (!test_and_set_bit(KGSL_BINDOBJ_STATE_START, &bindobj->state)) {
		/*
		 * Take a reference to the drawobj and the context because both
		 * get referenced in the bind callback
		 */
		_kgsl_context_get(&drawctxt->base);
		kref_get(&drawobj->refcount);

		kgsl_sharedmem_bind_ranges(bindobj->bind);
	}

	return -EAGAIN;
}

/*
 * Retires all expired marker and sync objs from the context
 * queue and returns one of the below
 * a) next drawobj that needs to be sent to ringbuffer
 * b) -EAGAIN for syncobj with syncpoints pending.
 * c) -EAGAIN for markerobj whose marker timestamp has not expired yet.
 * c) NULL for no commands remaining in drawqueue.
 */
static struct kgsl_drawobj *_process_drawqueue_get_next_drawobj(
	struct adreno_device *adreno_dev, struct adreno_context *drawctxt)
{
	struct kgsl_drawobj *drawobj;
	unsigned int i = drawctxt->drawqueue_head;
	struct kgsl_drawobj_cmd *cmdobj;
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	int ret = 0;

	if (drawctxt->drawqueue_head == drawctxt->drawqueue_tail)
		return NULL;

	for (i = drawctxt->drawqueue_head; i != drawctxt->drawqueue_tail;
			i = DRAWQUEUE_NEXT(i, ADRENO_CONTEXT_DRAWQUEUE_SIZE)) {

		drawobj = drawctxt->drawqueue[i];

		if (!drawobj)
			return NULL;

		switch (drawobj->type) {
		case CMDOBJ_TYPE:
			cmdobj = CMDOBJ(drawobj);

			/* We only support one big IB inflight */
			if ((cmdobj->numibs > HWSCHED_MAX_DISPATCH_NUMIBS) &&
				hwsched->big_cmdobj)
				return ERR_PTR(-ENOSPC);

			return drawobj;
		case SYNCOBJ_TYPE:
			ret = _retire_syncobj(adreno_dev, SYNCOBJ(drawobj), drawctxt);
			if (ret == 1)
				return drawobj;
			break;
		case MARKEROBJ_TYPE:
			ret = _retire_markerobj(adreno_dev, CMDOBJ(drawobj), drawctxt);
			/* Special case where marker needs to be sent to GPU */
			if (ret == 1)
				return drawobj;
			break;
		case BINDOBJ_TYPE:
			ret = drawqueue_retire_bindobj(drawobj, drawctxt);
			break;
		case TIMELINEOBJ_TYPE:
			ret = _retire_timelineobj(drawobj, drawctxt);
			break;
		default:
			ret = -EINVAL;
			break;
		}

		if (ret)
			return ERR_PTR(ret);
	}

	return NULL;
}

/**
 * hwsched_dispatcher_requeue_drawobj() - Put a draw objet back on the context
 * queue
 * @drawctxt: Pointer to the adreno draw context
 * @drawobj: Pointer to the KGSL draw object to requeue
 *
 * Failure to submit a drawobj to the ringbuffer isn't the fault of the drawobj
 * being submitted so if a failure happens, push it back on the head of the
 * context queue to be reconsidered again unless the context got detached.
 */
static inline int hwsched_dispatcher_requeue_drawobj(
		struct adreno_context *drawctxt,
		struct kgsl_drawobj *drawobj)
{
	unsigned int prev;

	spin_lock(&drawctxt->lock);

	if (kgsl_context_is_bad(&drawctxt->base)) {
		spin_unlock(&drawctxt->lock);
		/* get rid of this drawobj since the context is bad */
		kgsl_drawobj_destroy(drawobj);
		return -ENOENT;
	}

	prev = drawctxt->drawqueue_head == 0 ?
		(ADRENO_CONTEXT_DRAWQUEUE_SIZE - 1) :
		(drawctxt->drawqueue_head - 1);

	/*
	 * The maximum queue size always needs to be one less then the size of
	 * the ringbuffer queue so there is "room" to put the drawobj back in
	 */

	WARN_ON(prev == drawctxt->drawqueue_tail);

	drawctxt->drawqueue[prev] = drawobj;
	drawctxt->queued++;

	/* Reset the command queue head to reflect the newly requeued change */
	drawctxt->drawqueue_head = prev;
	if (is_cmdobj(drawobj)) {
		struct kgsl_drawobj_cmd *cmdobj = CMDOBJ(drawobj);

		cmdobj->requeue_cnt++;
	}
	spin_unlock(&drawctxt->lock);
	return 0;
}

/**
 * hwsched_queue_context() - Queue a context in the dispatcher list of jobs
 * @adreno_dev: Pointer to the adreno device structure
 * @drawctxt: Pointer to the adreno draw context
 *
 * Add a context to the dispatcher list of jobs.
 */
static int hwsched_queue_context(struct adreno_device *adreno_dev,
		struct adreno_context *drawctxt)
{
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	struct adreno_dispatch_job *job;

	/* Refuse to queue a detached context */
	if (kgsl_context_detached(&drawctxt->base))
		return 0;

	if (!_kgsl_context_get(&drawctxt->base))
		return 0;

	job = kmem_cache_alloc(jobs_cache, GFP_ATOMIC);
	if (!job) {
		kgsl_context_put(&drawctxt->base);
		return -ENOMEM;
	}

	job->drawctxt = drawctxt;

	trace_dispatch_queue_context(drawctxt);
	llist_add(&job->node, &hwsched->jobs[drawctxt->base.priority]);

	return 0;
}

/**
 * is_marker_skip() - Check if the draw object is a MARKEROBJ_TYPE and CMDOBJ_SKIP bit is set
 */
static bool is_marker_skip(struct kgsl_drawobj *drawobj)
{
	struct kgsl_drawobj_cmd *cmdobj = NULL;

	if (drawobj->type != MARKEROBJ_TYPE)
		return false;

	cmdobj = CMDOBJ(drawobj);

	if (test_bit(CMDOBJ_SKIP, &cmdobj->priv))
		return true;

	return false;
}

static bool _abort_submission(struct adreno_device *adreno_dev)
{
	/* We only need a single barrier before reading all the atomic variables below */
	smp_rmb();

	if (atomic_read(&adreno_dev->halt) || atomic_read(&adreno_dev->scheduler_fault))
		return true;

	return false;
}

/**
 * sendcmd() - Send a drawobj to the GPU hardware
 * @dispatcher: Pointer to the adreno dispatcher struct
 * @drawobj: Pointer to the KGSL drawobj being sent
 *
 * Send a KGSL drawobj to the GPU hardware
 */
static int hwsched_sendcmd(struct adreno_device *adreno_dev,
	struct kgsl_drawobj *drawobj)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	struct kgsl_context *context = drawobj->context;
	int ret;
	struct cmd_list_obj *obj;
	int is_current_rt = rt_task(current);
	int nice = task_nice(current);

	obj = kmem_cache_alloc(obj_cache, GFP_KERNEL);
	if (!obj)
		return -ENOMEM;

	mutex_lock(&device->mutex);

	/* Elevating thread’s priority to avoid context switch with holding device mutex */
	if (!is_current_rt)
		sched_set_fifo(current);

	if (_abort_submission(adreno_dev)) {
		ret = -EBUSY;
		goto done;
	}

	if (kgsl_context_detached(context)) {
		ret = -ENOENT;
		goto done;
	}

	hwsched->inflight++;

	if (hwsched->inflight == 1 &&
		!test_bit(ADRENO_HWSCHED_POWER, &hwsched->flags)) {
		ret = adreno_active_count_get(adreno_dev);
		if (ret) {
			hwsched->inflight--;
			goto done;
		}
		set_bit(ADRENO_HWSCHED_POWER, &hwsched->flags);
	}

	ret = hwsched->hwsched_ops->submit_drawobj(adreno_dev, drawobj);
	if (ret) {
		/*
		 * If the first submission failed, then put back the active
		 * count to relinquish active vote
		 */
		if (hwsched->inflight == 1) {
			adreno_active_count_put(adreno_dev);
			clear_bit(ADRENO_HWSCHED_POWER, &hwsched->flags);
		}

		hwsched->inflight--;
		goto done;
	}

	if ((hwsched->inflight == 1) &&
		!test_and_set_bit(ADRENO_HWSCHED_ACTIVE, &hwsched->flags))
		reinit_completion(&hwsched->idle_gate);

	if (is_cmdobj(drawobj)) {
		struct kgsl_drawobj_cmd *cmdobj = CMDOBJ(drawobj);

		/* If this MARKER object is already retired, we can destroy it here */
		if ((test_bit(CMDOBJ_MARKER_EXPIRED, &cmdobj->priv))) {
			kmem_cache_free(obj_cache, obj);
			kgsl_drawobj_destroy(drawobj);
			goto done;
		}

		if (cmdobj->numibs > HWSCHED_MAX_DISPATCH_NUMIBS) {
			hwsched->big_cmdobj = cmdobj;
			kref_get(&drawobj->refcount);
		}
	}

	obj->drawobj = drawobj;
	list_add_tail(&obj->node, &hwsched->cmd_list);

done:
	if (!is_current_rt)
		sched_set_normal(current, nice);
	mutex_unlock(&device->mutex);
	if (ret)
		kmem_cache_free(obj_cache, obj);

	return ret;
}

/**
 * hwsched_sendcmds() - Send commands from a context to the GPU
 * @adreno_dev: Pointer to the adreno device struct
 * @drawctxt: Pointer to the adreno context to dispatch commands from
 *
 * Dequeue and send a burst of commands from the specified context to the GPU
 * Returns postive if the context needs to be put back on the pending queue
 * 0 if the context is empty or detached and negative on error
 */
static int hwsched_sendcmds(struct adreno_device *adreno_dev,
		struct adreno_context *drawctxt)
{
	int count = 0;
	int ret = 0;

	while (1) {
		struct kgsl_drawobj *drawobj;
		struct kgsl_drawobj_cmd *cmdobj = NULL;
		struct kgsl_context *context;

		spin_lock(&drawctxt->lock);
		drawobj = _process_drawqueue_get_next_drawobj(adreno_dev,
				drawctxt);

		/*
		 * adreno_context_get_drawobj returns -EAGAIN if the current
		 * drawobj has pending sync points so no more to do here.
		 * When the sync points are satisfied then the context will get
		 * reqeueued
		 */

		if (IS_ERR_OR_NULL(drawobj)) {
			if (IS_ERR(drawobj))
				ret = PTR_ERR(drawobj);
			spin_unlock(&drawctxt->lock);
			break;
		}
		_pop_drawobj(drawctxt);
		spin_unlock(&drawctxt->lock);

		if (is_cmdobj(drawobj) || is_marker_skip(drawobj)) {
			cmdobj = CMDOBJ(drawobj);
			context = drawobj->context;
			trace_adreno_cmdbatch_ready(context->id,
				context->priority, drawobj->timestamp,
				cmdobj->requeue_cnt);
		}
		ret = hwsched_sendcmd(adreno_dev, drawobj);

		/*
		 * On error from hwsched_sendcmd() try to requeue the cmdobj
		 * unless we got back -ENOENT which means that the context has
		 * been detached and there will be no more deliveries from here
		 */
		if (ret != 0) {
			/* Destroy the cmdobj on -ENOENT */
			if (ret == -ENOENT)
				kgsl_drawobj_destroy(drawobj);
			else {
				/*
				 * If we couldn't put it on dispatch queue
				 * then return it to the context queue
				 */
				int r = hwsched_dispatcher_requeue_drawobj(
					drawctxt, drawobj);
				if (r)
					ret = r;
			}

			break;
		}

		if (cmdobj)
			drawctxt->submitted_timestamp = drawobj->timestamp;

		count++;
	}

	/*
	 * Wake up any snoozing threads if we have consumed any real commands
	 * or marker commands and we have room in the context queue.
	 */

	if (_check_context_queue(drawctxt, 0))
		wake_up_all(&drawctxt->wq);

	if (!ret)
		ret = count;

	/* Return error or the number of commands queued */
	return ret;
}

static void hwsched_handle_jobs_list(struct adreno_device *adreno_dev,
	int id, unsigned long *map, struct llist_node *list)
{
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	struct adreno_dispatch_job *job, *next;

	if (!list)
		return;

	/* Reverse the list so we deal with oldest submitted contexts first */
	list = llist_reverse_order(list);

	llist_for_each_entry_safe(job, next, list, node) {
		int ret;

		if (kgsl_context_is_bad(&job->drawctxt->base)) {
			kgsl_context_put(&job->drawctxt->base);
			kmem_cache_free(jobs_cache, job);
			continue;
		}

		/*
		 * Due to the nature of the lockless queue the same context
		 * might have multiple jobs on the list. We allow this so we
		 * don't have to query the list on the producer side but on the
		 * consumer side we only want each context to be considered
		 * once. Use a bitmap to remember which contexts we've already
		 * seen and quietly discard duplicate jobs
		 */
		if (test_and_set_bit(job->drawctxt->base.id, map)) {
			kgsl_context_put(&job->drawctxt->base);
			kmem_cache_free(jobs_cache, job);
			continue;
		}

		ret = hwsched_sendcmds(adreno_dev, job->drawctxt);

		/*
		 * If the context had nothing queued or the context has been
		 * destroyed then drop the job
		 */
		if (!ret || ret == -ENOENT) {
			kgsl_context_put(&job->drawctxt->base);
			kmem_cache_free(jobs_cache, job);
			continue;
		}

		/*
		 * If the dispatch queue is full then requeue the job to be
		 * considered first next time. Otherwise the context
		 * either successfully submmitted to the GPU or another error
		 * happened and it should go back on the regular queue
		 */
		if (ret == -ENOSPC)
			llist_add(&job->node, &hwsched->requeue[id]);
		else
			llist_add(&job->node, &hwsched->jobs[id]);
	}
}

static void hwsched_handle_jobs(struct adreno_device *adreno_dev, int id)
{
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	unsigned long map[BITS_TO_LONGS(KGSL_MEMSTORE_MAX)];
	struct llist_node *requeue, *jobs;

	memset(map, 0, sizeof(map));

	requeue = llist_del_all(&hwsched->requeue[id]);
	jobs = llist_del_all(&hwsched->jobs[id]);

	hwsched_handle_jobs_list(adreno_dev, id, map, requeue);
	hwsched_handle_jobs_list(adreno_dev, id, map, jobs);
}

/**
 * hwsched_issuecmds() - Issue commmands from pending contexts
 * @adreno_dev: Pointer to the adreno device struct
 *
 * Issue as many commands as possible (up to inflight) from the pending contexts
 * This function assumes the dispatcher mutex has been locked.
 */
static void hwsched_issuecmds(struct adreno_device *adreno_dev)
{
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	int i;

	for (i = 0; i < ARRAY_SIZE(hwsched->jobs); i++)
		hwsched_handle_jobs(adreno_dev, i);
}

static inline void _decrement_submit_now(struct kgsl_device *device)
{
	spin_lock(&device->submit_lock);
	device->submit_now--;
	spin_unlock(&device->submit_lock);
}

/**
 * adreno_hwsched_issuecmds() - Issue commmands from pending contexts
 * @adreno_dev: Pointer to the adreno device struct
 *
 * Lock the dispatcher and call hwsched_issuecmds
 */
static void adreno_hwsched_issuecmds(struct adreno_device *adreno_dev)
{
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	spin_lock(&device->submit_lock);
	/* If GPU state is not ACTIVE, schedule the work for later */
	if (device->skip_inline_submit) {
		spin_unlock(&device->submit_lock);
		goto done;
	}
	device->submit_now++;
	spin_unlock(&device->submit_lock);

	/* If the dispatcher is busy then schedule the work for later */
	if (!mutex_trylock(&hwsched->mutex)) {
		_decrement_submit_now(device);
		goto done;
	}

	if (!adreno_gpu_fault(adreno_dev))
		hwsched_issuecmds(adreno_dev);

	if (hwsched->inflight > 0) {
		mutex_lock(&device->mutex);
		kgsl_pwrscale_update(device);
		kgsl_start_idle_timer(device);
		mutex_unlock(&device->mutex);
	}

	mutex_unlock(&hwsched->mutex);
	_decrement_submit_now(device);
	return;

done:
	adreno_scheduler_queue(adreno_dev);
}

/**
 * get_timestamp() - Return the next timestamp for the context
 * @drawctxt - Pointer to an adreno draw context struct
 * @drawobj - Pointer to a drawobj
 * @timestamp - Pointer to a timestamp value possibly passed from the user
 * @user_ts - user generated timestamp
 *
 * Assign a timestamp based on the settings of the draw context and the command
 * batch.
 */
static int get_timestamp(struct adreno_context *drawctxt,
		struct kgsl_drawobj *drawobj, unsigned int *timestamp,
		unsigned int user_ts)
{

	if (drawctxt->base.flags & KGSL_CONTEXT_USER_GENERATED_TS) {
		/*
		 * User specified timestamps need to be greater than the last
		 * issued timestamp in the context
		 */
		if (timestamp_cmp(drawctxt->timestamp, user_ts) >= 0)
			return -ERANGE;

		drawctxt->timestamp = user_ts;
	} else
		drawctxt->timestamp++;

	*timestamp = drawctxt->timestamp;
	drawobj->timestamp = *timestamp;
	return 0;
}

static inline int _wait_for_room_in_context_queue(
	struct adreno_context *drawctxt, u32 count)
{
	int ret = 0;

	/*
	 * There is always a possibility that dispatcher may end up pushing
	 * the last popped draw object back to the context drawqueue. Hence,
	 * we can only queue up to _context_drawqueue_size - 1 here to make
	 * sure we never let drawqueue->queued exceed _context_drawqueue_size.
	 */
	if ((drawctxt->queued + count) > (_context_drawqueue_size - 1)) {
		trace_adreno_drawctxt_sleep(drawctxt);
		spin_unlock(&drawctxt->lock);

		ret = wait_event_interruptible_timeout(drawctxt->wq,
			_check_context_queue(drawctxt, count),
			msecs_to_jiffies(_context_queue_wait));

		spin_lock(&drawctxt->lock);
		trace_adreno_drawctxt_wake(drawctxt);

		/*
		 * Account for the possibility that the context got invalidated
		 * while we were sleeping
		 */
		if (ret > 0)
			ret = kgsl_check_context_state(&drawctxt->base);
		else if (ret == 0)
			ret = -ETIMEDOUT;
	}

	return ret;
}

static unsigned int _check_context_state_to_queue_cmds(
	struct adreno_context *drawctxt, u32 count)
{
	int ret = kgsl_check_context_state(&drawctxt->base);

	if (ret)
		return ret;

	return _wait_for_room_in_context_queue(drawctxt, count);
}

static void _queue_drawobj(struct adreno_context *drawctxt,
	struct kgsl_drawobj *drawobj)
{
	struct kgsl_context *context = drawobj->context;

	/* Put the command into the queue */
	drawctxt->drawqueue[drawctxt->drawqueue_tail] = drawobj;
	drawctxt->drawqueue_tail = (drawctxt->drawqueue_tail + 1) %
			ADRENO_CONTEXT_DRAWQUEUE_SIZE;
	drawctxt->queued++;
	msm_perf_events_update(MSM_PERF_GFX, MSM_PERF_QUEUE,
		pid_nr(context->proc_priv->pid),
		context->id, drawobj->timestamp,
		!!(drawobj->flags & KGSL_DRAWOBJ_END_OF_FRAME));
	trace_adreno_cmdbatch_queued(drawobj, drawctxt->queued);
}

static int _queue_cmdobj(struct adreno_device *adreno_dev,
	struct adreno_context *drawctxt, struct kgsl_drawobj_cmd *cmdobj,
	uint32_t *timestamp, unsigned int user_ts)
{
	struct kgsl_drawobj *drawobj = DRAWOBJ(cmdobj);
	u32 j;
	int ret;

	ret = get_timestamp(drawctxt, drawobj, timestamp, user_ts);
	if (ret)
		return ret;

	/*
	 * If this is a real command then we need to force any markers
	 * queued before it to dispatch to keep time linear - set the
	 * skip bit so the commands get NOPed.
	 */
	j = drawctxt->drawqueue_head;

	while (j != drawctxt->drawqueue_tail) {
		if (drawctxt->drawqueue[j]->type == MARKEROBJ_TYPE) {
			struct kgsl_drawobj_cmd *markerobj =
				CMDOBJ(drawctxt->drawqueue[j]);

			set_bit(CMDOBJ_SKIP, &markerobj->priv);
		}

		j = DRAWQUEUE_NEXT(j, ADRENO_CONTEXT_DRAWQUEUE_SIZE);
	}

	drawctxt->queued_timestamp = *timestamp;

	_queue_drawobj(drawctxt, drawobj);

	return 0;
}

static void _queue_syncobj(struct adreno_context *drawctxt,
	struct kgsl_drawobj_sync *syncobj, uint32_t *timestamp)
{
	struct kgsl_drawobj *drawobj = DRAWOBJ(syncobj);

	*timestamp = 0;
	drawobj->timestamp = 0;

	_queue_drawobj(drawctxt, drawobj);
}

static int _queue_markerobj(struct adreno_device *adreno_dev,
	struct adreno_context *drawctxt, struct kgsl_drawobj_cmd *markerobj,
	u32 *timestamp, u32 user_ts)
{
	struct kgsl_drawobj *drawobj = DRAWOBJ(markerobj);
	int ret;

	ret = get_timestamp(drawctxt, drawobj, timestamp, user_ts);
	if (ret)
		return ret;

	/*
	 * See if we can fastpath this thing - if nothing is queued
	 * and nothing is inflight retire without bothering the GPU
	 */
	if (!drawctxt->queued && kgsl_check_timestamp(drawobj->device,
		drawobj->context, drawctxt->queued_timestamp)) {
		_retire_timestamp(drawobj);
		return 1;
	}

	/*
	 * Remember the last queued timestamp - the marker will block
	 * until that timestamp is expired (unless another command
	 * comes along and forces the marker to execute)
	 */
	 markerobj->marker_timestamp = drawctxt->queued_timestamp;
	 drawctxt->queued_timestamp = *timestamp;

	_queue_drawobj(drawctxt, drawobj);

	return 0;
}

static int _queue_bindobj(struct adreno_context *drawctxt,
		struct kgsl_drawobj *drawobj, u32 *timestamp, u32 user_ts)
{
	int ret;

	ret = get_timestamp(drawctxt, drawobj, timestamp, user_ts);
	if (ret)
		return ret;

	drawctxt->queued_timestamp = *timestamp;
	_queue_drawobj(drawctxt, drawobj);

	return 0;
}

static void _queue_timelineobj(struct adreno_context *drawctxt,
		struct kgsl_drawobj *drawobj)
{
	/*
	 * This drawobj is not submitted to the GPU so use a timestamp of 0.
	 * Update the timestamp through a subsequent marker to keep userspace
	 * happy.
	 */
	drawobj->timestamp = 0;

	_queue_drawobj(drawctxt, drawobj);
}

static int adreno_hwsched_queue_cmds(struct kgsl_device_private *dev_priv,
	struct kgsl_context *context, struct kgsl_drawobj *drawobj[],
	u32 count, u32 *timestamp)

{
	struct kgsl_device *device = dev_priv->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_context *drawctxt = ADRENO_CONTEXT(context);
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	struct adreno_dispatch_job *job;
	int ret;
	unsigned int i, user_ts;

	/*
	 * There is always a possibility that dispatcher may end up pushing
	 * the last popped draw object back to the context drawqueue. Hence,
	 * we can only queue up to _context_drawqueue_size - 1 here to make
	 * sure we never let drawqueue->queued exceed _context_drawqueue_size.
	 */
	if (!count || count > _context_drawqueue_size - 1)
		return -EINVAL;

	for (i = 0; i < count; i++) {
		struct kgsl_drawobj_cmd *cmdobj;
		struct kgsl_memobj_node *ib;

		if (!is_cmdobj(drawobj[i]))
			continue;

		cmdobj = CMDOBJ(drawobj[i]);

		list_for_each_entry(ib, &cmdobj->cmdlist, node)
			cmdobj->numibs++;

		if (cmdobj->numibs > HWSCHED_MAX_IBS)
			return -EINVAL;
	}

	ret = kgsl_check_context_state(&drawctxt->base);
	if (ret)
		return ret;

	ret = adreno_verify_cmdobj(dev_priv, context, drawobj, count);
	if (ret)
		return ret;

	/* wait for the suspend gate */
	wait_for_completion(&device->halt_gate);

	job = kmem_cache_alloc(jobs_cache, GFP_KERNEL);
	if (!job)
		return -ENOMEM;

	job->drawctxt = drawctxt;

	spin_lock(&drawctxt->lock);

	ret = _check_context_state_to_queue_cmds(drawctxt, count);
	if (ret) {
		spin_unlock(&drawctxt->lock);
		kmem_cache_free(jobs_cache, job);
		return ret;
	}

	user_ts = *timestamp;

	/*
	 * If there is only one drawobj in the array and it is of
	 * type SYNCOBJ_TYPE, skip comparing user_ts as it can be 0
	 */
	if (!(count == 1 && drawobj[0]->type == SYNCOBJ_TYPE) &&
		(drawctxt->base.flags & KGSL_CONTEXT_USER_GENERATED_TS)) {
		/*
		 * User specified timestamps need to be greater than the last
		 * issued timestamp in the context
		 */
		if (timestamp_cmp(drawctxt->timestamp, user_ts) >= 0) {
			spin_unlock(&drawctxt->lock);
			kmem_cache_free(jobs_cache, job);
			return -ERANGE;
		}
	}

	for (i = 0; i < count; i++) {

		switch (drawobj[i]->type) {
		case MARKEROBJ_TYPE:
			ret = _queue_markerobj(adreno_dev, drawctxt,
					CMDOBJ(drawobj[i]),
					timestamp, user_ts);
			if (ret == 1) {
				spin_unlock(&drawctxt->lock);
				kmem_cache_free(jobs_cache, job);
				return 0;
			} else if (ret) {
				spin_unlock(&drawctxt->lock);
				kmem_cache_free(jobs_cache, job);
				return ret;
			}
			break;
		case CMDOBJ_TYPE:
			ret = _queue_cmdobj(adreno_dev, drawctxt,
						CMDOBJ(drawobj[i]),
						timestamp, user_ts);
			if (ret) {
				spin_unlock(&drawctxt->lock);
				kmem_cache_free(jobs_cache, job);
				return ret;
			}
			break;
		case SYNCOBJ_TYPE:
			_queue_syncobj(drawctxt, SYNCOBJ(drawobj[i]),
						timestamp);
			break;
		case BINDOBJ_TYPE:
			ret = _queue_bindobj(drawctxt, drawobj[i], timestamp,
						user_ts);
			if (ret) {
				spin_unlock(&drawctxt->lock);
				kmem_cache_free(jobs_cache, job);
				return ret;
			}
			break;
		case TIMELINEOBJ_TYPE:
			_queue_timelineobj(drawctxt, drawobj[i]);
			break;
		default:
			spin_unlock(&drawctxt->lock);
			kmem_cache_free(jobs_cache, job);
			return -EINVAL;
		}

	}

	adreno_track_context(adreno_dev, NULL, drawctxt);

	spin_unlock(&drawctxt->lock);

	/* Add the context to the dispatcher pending list */
	if (_kgsl_context_get(&drawctxt->base)) {
		trace_dispatch_queue_context(drawctxt);
		llist_add(&job->node, &hwsched->jobs[drawctxt->base.priority]);
		adreno_hwsched_issuecmds(adreno_dev);

	} else
		kmem_cache_free(jobs_cache, job);

	if (test_and_clear_bit(ADRENO_CONTEXT_FAULT, &context->priv))
		return -EPROTO;

	return 0;
}

void adreno_hwsched_retire_cmdobj(struct adreno_hwsched *hwsched,
	struct kgsl_drawobj_cmd *cmdobj)
{
	struct kgsl_drawobj *drawobj = DRAWOBJ(cmdobj);
	struct kgsl_mem_entry *entry;
	struct kgsl_drawobj_profiling_buffer *profile_buffer;
	struct kgsl_context *context = drawobj->context;

	msm_perf_events_update(MSM_PERF_GFX, MSM_PERF_RETIRED,
		pid_nr(context->proc_priv->pid),
		context->id, drawobj->timestamp,
		!!(drawobj->flags & KGSL_DRAWOBJ_END_OF_FRAME));

	if (drawobj->flags & KGSL_DRAWOBJ_END_OF_FRAME) {
		atomic64_inc(&drawobj->context->proc_priv->frame_count);
		atomic_inc(&drawobj->context->proc_priv->period->frames);
	}

	entry = cmdobj->profiling_buf_entry;
	if (entry) {
		profile_buffer = kgsl_gpuaddr_to_vaddr(&entry->memdesc,
			cmdobj->profiling_buffer_gpuaddr);

		if (profile_buffer == NULL)
			return;

		kgsl_memdesc_unmap(&entry->memdesc);
	}

	trace_adreno_cmdbatch_done(drawobj->context->id,
		drawobj->context->priority, drawobj->timestamp);

	if (hwsched->big_cmdobj == cmdobj) {
		hwsched->big_cmdobj = NULL;
		kgsl_drawobj_put(drawobj);
	}

	kgsl_drawobj_destroy(drawobj);
}

void adreno_hwsched_syncobj_kfence_put(struct kgsl_drawobj_sync *syncobj)
{
	int i;

	for (i = 0; i < syncobj->num_hw_fence; i++) {
		kgsl_context_put(syncobj->hw_fences[i].context);
		syncobj->hw_fences[i].context = NULL;
	}
}

static bool drawobj_retired(struct adreno_device *adreno_dev,
	struct kgsl_drawobj *drawobj)
{
	struct adreno_context *drawctxt = ADRENO_CONTEXT(drawobj->context);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_drawobj_cmd *cmdobj;
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;

	if ((drawobj->type & SYNCOBJ_TYPE) != 0) {
		struct gmu_context_queue_header *hdr =
			drawctxt->gmu_context_queue.hostptr;

		if (timestamp_cmp(drawobj->timestamp, hdr->sync_obj_ts) > 0)
			return false;

		adreno_hwsched_syncobj_kfence_put(SYNCOBJ(drawobj));
		kgsl_drawobj_destroy(drawobj);
		return true;
	}

	cmdobj = CMDOBJ(drawobj);

	if (!kgsl_check_timestamp(device, drawobj->context,
		drawobj->timestamp))
		return false;

	adreno_hwsched_retire_cmdobj(hwsched, cmdobj);
	return true;
}

static void retire_drawobj_list(struct adreno_device *adreno_dev)
{
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	struct cmd_list_obj *obj, *tmp;

	list_for_each_entry_safe(obj, tmp, &hwsched->cmd_list, node) {
		struct kgsl_drawobj *drawobj = obj->drawobj;

		if (!drawobj_retired(adreno_dev, drawobj))
			continue;

		list_del_init(&obj->node);

		kmem_cache_free(obj_cache, obj);

		hwsched->inflight--;
	}
}

/* Take down the dispatcher and release any power states */
static void hwsched_power_down(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;

	mutex_lock(&device->mutex);

	if (test_and_clear_bit(ADRENO_HWSCHED_ACTIVE, &hwsched->flags))
		complete_all(&hwsched->idle_gate);

	if (test_bit(ADRENO_HWSCHED_POWER, &hwsched->flags)) {
		adreno_active_count_put(adreno_dev);
		clear_bit(ADRENO_HWSCHED_POWER, &hwsched->flags);
	}

	mutex_unlock(&device->mutex);
}

static void adreno_hwsched_queue_context(struct adreno_device *adreno_dev,
	struct adreno_context *drawctxt)
{
	hwsched_queue_context(adreno_dev, drawctxt);
	adreno_scheduler_queue(adreno_dev);
}

void adreno_hwsched_start(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	complete_all(&device->halt_gate);

	adreno_scheduler_queue(adreno_dev);
}

static void change_preemption(struct adreno_device *adreno_dev, void *priv)
{
	change_bit(ADRENO_DEVICE_PREEMPTION, &adreno_dev->priv);
}

static int _preemption_store(struct adreno_device *adreno_dev, bool val)
{
	if (!adreno_preemption_feature_set(adreno_dev) ||
		(test_bit(ADRENO_DEVICE_PREEMPTION, &adreno_dev->priv) == val))
		return 0;

	return adreno_power_cycle(adreno_dev, change_preemption, NULL);
}

static bool _preemption_show(struct adreno_device *adreno_dev)
{
	return adreno_is_preemption_enabled(adreno_dev);
}

static unsigned int _preempt_count_show(struct adreno_device *adreno_dev)
{
	const struct adreno_hwsched_ops *hwsched_ops =
		adreno_dev->hwsched.hwsched_ops;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	u32 count;

	mutex_lock(&device->mutex);

	count = hwsched_ops->preempt_count(adreno_dev);

	mutex_unlock(&device->mutex);

	return count;
}

static int _ft_long_ib_detect_store(struct adreno_device *adreno_dev, bool val)
{
	return adreno_power_cycle_bool(adreno_dev, &adreno_dev->long_ib_detect,
			val);
}

static bool _ft_long_ib_detect_show(struct adreno_device *adreno_dev)
{
	return adreno_dev->long_ib_detect;
}

static ADRENO_SYSFS_BOOL(preemption);
static ADRENO_SYSFS_RO_U32(preempt_count);
static ADRENO_SYSFS_BOOL(ft_long_ib_detect);

static const struct attribute *_hwsched_attr_list[] = {
	&adreno_attr_preemption.attr.attr,
	&adreno_attr_preempt_count.attr.attr,
	&adreno_attr_ft_long_ib_detect.attr.attr,
	NULL,
};

void adreno_hwsched_deregister_hw_fence(struct adreno_device *adreno_dev)
{
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;

	if (!test_bit(ADRENO_HWSCHED_HW_FENCE, &hwsched->flags))
		return;

	kgsl_hw_fence_close(KGSL_DEVICE(adreno_dev));

	if (hwsched->hw_fence_md.sgt)
		sg_free_table(hwsched->hw_fence_md.sgt);

	memset(&hwsched->hw_fence_md, 0x0, sizeof(hwsched->hw_fence_md));

	kmem_cache_destroy(hwsched->hw_fence_cache);

	clear_bit(ADRENO_HWSCHED_HW_FENCE, &hwsched->flags);
}

static void adreno_hwsched_dispatcher_close(struct adreno_device *adreno_dev)
{
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	if (!IS_ERR_OR_NULL(adreno_dev->scheduler_worker))
		kthread_destroy_worker(adreno_dev->scheduler_worker);

	adreno_set_dispatch_ops(adreno_dev, NULL);

	kmem_cache_destroy(jobs_cache);
	kmem_cache_destroy(obj_cache);

	sysfs_remove_files(&device->dev->kobj, _hwsched_attr_list);

	kfree(hwsched->ctxt_bad);

	adreno_hwsched_deregister_hw_fence(adreno_dev);

	if (hwsched->global_ctxtq.hostptr)
		kgsl_sharedmem_free(&hwsched->global_ctxtq);
}

static void force_retire_timestamp(struct kgsl_device *device,
	struct kgsl_drawobj *drawobj)
{
	kgsl_sharedmem_writel(device->memstore,
		KGSL_MEMSTORE_OFFSET(drawobj->context->id, soptimestamp),
		drawobj->timestamp);

	kgsl_sharedmem_writel(device->memstore,
		KGSL_MEMSTORE_OFFSET(drawobj->context->id, eoptimestamp),
		drawobj->timestamp);
}

/* Return true if drawobj needs to replayed, false otherwise */
static bool drawobj_replay(struct adreno_device *adreno_dev,
	struct kgsl_drawobj *drawobj)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_drawobj_cmd *cmdobj;
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;

	if ((drawobj->type & SYNCOBJ_TYPE) != 0) {

		if (kgsl_drawobj_events_pending(SYNCOBJ(drawobj)))
			return true;

		adreno_hwsched_syncobj_kfence_put(SYNCOBJ(drawobj));
		kgsl_drawobj_destroy(drawobj);
		return false;
	}

	cmdobj = CMDOBJ(drawobj);

	if (kgsl_check_timestamp(device, drawobj->context,
		drawobj->timestamp) || kgsl_context_is_bad(drawobj->context)) {
		adreno_hwsched_retire_cmdobj(hwsched, cmdobj);
		return false;
	}

	return true;
}

void adreno_hwsched_replay(struct adreno_device *adreno_dev)
{
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	const struct adreno_gpudev *gpudev  = ADRENO_GPU_DEVICE(adreno_dev);
	struct cmd_list_obj *obj, *tmp;
	u32 retired = 0;

	list_for_each_entry_safe(obj, tmp, &hwsched->cmd_list, node) {
		struct kgsl_drawobj *drawobj = obj->drawobj;

		/*
		 * Get rid of retired objects or objects that belong to detached
		 * or invalidated contexts
		 */
		if (drawobj_replay(adreno_dev, drawobj)) {
			hwsched->hwsched_ops->submit_drawobj(adreno_dev, drawobj);
			continue;
		}

		retired++;

		list_del_init(&obj->node);
		kmem_cache_free(obj_cache, obj);
		hwsched->inflight--;
	}

	if (hwsched->recurring_cmdobj) {
		u32 event;

		if (kgsl_context_invalid(
			hwsched->recurring_cmdobj->base.context)) {
			clear_bit(CMDOBJ_RECURRING_START,
					&hwsched->recurring_cmdobj->priv);
			set_bit(CMDOBJ_RECURRING_STOP,
					&hwsched->recurring_cmdobj->priv);
			event = GPU_SSR_FATAL;
		} else {
			event = GPU_SSR_END;
		}
		gpudev->send_recurring_cmdobj(adreno_dev,
			hwsched->recurring_cmdobj);
		srcu_notifier_call_chain(&device->nh, event, NULL);
	}

	/* Signal fences */
	if (retired)
		kgsl_process_event_groups(device);
}

static void do_fault_header(struct adreno_device *adreno_dev,
	struct kgsl_drawobj *drawobj, int fault)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	const struct adreno_gpudev *gpudev  = ADRENO_GPU_DEVICE(adreno_dev);
	struct adreno_context *drawctxt;
	u32 status = 0, rptr = 0, wptr = 0, ib1sz = 0, ib2sz = 0;
	u64 ib1base = 0, ib2base = 0;
	bool gx_on = adreno_gx_is_on(adreno_dev);
	u32 ctxt_id = 0, ts = 0;
	int rb_id = -1;

	dev_err(device->dev, "Fault id:%d and GX is %s\n", fault, gx_on ? "ON" : "OFF");

	if (!gx_on && !drawobj)
		return;

	if (gpudev->fault_header)
		return gpudev->fault_header(adreno_dev, drawobj);

	if (gx_on) {
		adreno_readreg(adreno_dev, ADRENO_REG_RBBM_STATUS, &status);
		adreno_readreg(adreno_dev, ADRENO_REG_CP_RB_RPTR, &rptr);
		adreno_readreg(adreno_dev, ADRENO_REG_CP_RB_WPTR, &wptr);
		adreno_readreg64(adreno_dev, ADRENO_REG_CP_IB1_BASE,
				ADRENO_REG_CP_IB1_BASE_HI, &ib1base);
		adreno_readreg(adreno_dev, ADRENO_REG_CP_IB1_BUFSZ, &ib1sz);
		adreno_readreg64(adreno_dev, ADRENO_REG_CP_IB2_BASE,
				ADRENO_REG_CP_IB2_BASE_HI, &ib2base);
		adreno_readreg(adreno_dev, ADRENO_REG_CP_IB2_BUFSZ, &ib2sz);

		dev_err(device->dev,
			"status %8.8X rb %4.4x/%4.4x ib1 %16.16llX/%4.4x ib2 %16.16llX/%4.4x\n",
			status, rptr, wptr, ib1base, ib1sz, ib2base, ib2sz);
	}

	if (drawobj) {
		drawctxt = ADRENO_CONTEXT(drawobj->context);
		drawobj->context->last_faulted_cmd_ts = drawobj->timestamp;
		drawobj->context->total_fault_count++;
		ctxt_id = drawobj->context->id;
		ts = drawobj->timestamp;
		rb_id = adreno_get_level(drawobj->context);

		pr_context(device, drawobj->context,
			"ctx %u ctx_type %s ts %u policy %lX dispatch_queue=%d\n",
			drawobj->context->id, kgsl_context_type(drawctxt->type),
			drawobj->timestamp, CMDOBJ(drawobj)->fault_recovery,
			drawobj->context->gmu_dispatch_queue);

		pr_context(device, drawobj->context,
			   "cmdline: %s\n", drawctxt->base.proc_priv->cmdline);
	}

	trace_adreno_gpu_fault(ctxt_id, ts, status, rptr, wptr, ib1base, ib1sz,
			       ib2base, ib2sz, rb_id);
}

static struct cmd_list_obj *get_active_cmdobj_lpac(
	struct adreno_device *adreno_dev)
{
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	struct cmd_list_obj *obj, *tmp, *active_obj = NULL;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	u32 consumed = 0, retired = 0;
	struct kgsl_drawobj *drawobj = NULL;

	list_for_each_entry_safe(obj, tmp, &hwsched->cmd_list, node) {
		drawobj = obj->drawobj;

		if (!(kgsl_context_is_lpac(drawobj->context)))
			continue;

		kgsl_readtimestamp(device, drawobj->context,
			KGSL_TIMESTAMP_CONSUMED, &consumed);
		kgsl_readtimestamp(device, drawobj->context,
			KGSL_TIMESTAMP_RETIRED, &retired);

		if (!consumed)
			continue;

		if (consumed == retired)
			continue;

		/*
		 * Find the first submission that started but didn't finish
		 * We only care about one ringbuffer for LPAC so just look for the
		 * first unfinished submission
		 */
		if (!active_obj)
			active_obj = obj;
	}

	if (active_obj) {
		drawobj = active_obj->drawobj;

		if (kref_get_unless_zero(&drawobj->refcount)) {
			struct kgsl_drawobj_cmd *cmdobj = CMDOBJ(drawobj);

			set_bit(CMDOBJ_FAULT, &cmdobj->priv);
			return active_obj;
		}
	}

	return NULL;
}

static struct cmd_list_obj *get_active_cmdobj(
	struct adreno_device *adreno_dev)
{
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	struct cmd_list_obj *obj, *tmp, *active_obj = NULL;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	u32 consumed = 0, retired = 0, prio = UINT_MAX;
	struct kgsl_drawobj *drawobj = NULL;

	list_for_each_entry_safe(obj, tmp, &hwsched->cmd_list, node) {
		drawobj = obj->drawobj;

		/* We track LPAC separately */
		if (!is_cmdobj(drawobj) || kgsl_context_is_lpac(drawobj->context))
			continue;

		kgsl_readtimestamp(device, drawobj->context,
			KGSL_TIMESTAMP_CONSUMED, &consumed);
		kgsl_readtimestamp(device, drawobj->context,
			KGSL_TIMESTAMP_RETIRED, &retired);

		if (!consumed)
			continue;

		if (consumed == retired)
			continue;

		/* Find the first submission that started but didn't finish */
		if (!active_obj) {
			active_obj = obj;
			prio = adreno_get_level(drawobj->context);
			continue;
		}

		/* Find the highest priority active submission */
		if (adreno_get_level(drawobj->context) < prio) {
			active_obj = obj;
			prio = adreno_get_level(drawobj->context);
		}
	}

	if (active_obj) {
		struct kgsl_drawobj_cmd *cmdobj;

		drawobj = active_obj->drawobj;
		cmdobj = CMDOBJ(drawobj);

		if (kref_get_unless_zero(&drawobj->refcount)) {
			set_bit(CMDOBJ_FAULT, &cmdobj->priv);
			return active_obj;
		}
	}

	return NULL;
}

static struct cmd_list_obj *get_fault_cmdobj(struct adreno_device *adreno_dev,
				u32 ctxt_id, u32 ts)
{
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	struct cmd_list_obj *obj, *tmp;

	list_for_each_entry_safe(obj, tmp, &hwsched->cmd_list, node) {
		struct kgsl_drawobj *drawobj = obj->drawobj;

		if (!is_cmdobj(drawobj))
			continue;

		if ((ctxt_id == drawobj->context->id) &&
			(ts == drawobj->timestamp)) {
			if (kref_get_unless_zero(&drawobj->refcount)) {
				struct kgsl_drawobj_cmd *cmdobj = CMDOBJ(drawobj);

				set_bit(CMDOBJ_FAULT, &cmdobj->priv);
				return obj;
			}
		}
	}

	return NULL;
}

static bool context_is_throttled(struct kgsl_device *device,
	struct kgsl_context *context)
{
	if (ktime_ms_delta(ktime_get(), context->fault_time) >
		_fault_throttle_time) {
		context->fault_time = ktime_get();
		context->fault_count = 1;
		return false;
	}

	context->fault_count++;

	if (context->fault_count > _fault_throttle_burst) {
		pr_context(device, context,
			"gpu fault threshold exceeded %d faults in %d msecs\n",
			_fault_throttle_burst, _fault_throttle_time);
		return true;
	}

	return false;
}

static void _print_syncobj(struct adreno_device *adreno_dev, struct kgsl_drawobj *drawobj)
{
	int i;
	struct kgsl_drawobj_sync *syncobj = SYNCOBJ(drawobj);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	for (i = 0; i < syncobj->num_hw_fence; i++) {
		struct dma_fence *fence = syncobj->hw_fences[i].fence;
		bool kgsl = is_kgsl_fence(fence);
		bool signaled = test_bit(DMA_FENCE_FLAG_SIGNALED_BIT, &fence->flags);
		char value[32] = "unknown";

		if (fence->ops->timeline_value_str)
			fence->ops->timeline_value_str(fence, value, sizeof(value));

		dev_err(device->dev,
			"dma fence[%d] signaled:%d kgsl:%d ctx:%llu seqno:%llu value:%s\n",
			i, signaled, kgsl, fence->context, fence->seqno, value);
	}

}

static void print_fault_syncobj(struct adreno_device *adreno_dev,
				u32 ctxt_id, u32 ts)
{
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	struct cmd_list_obj *obj;

	list_for_each_entry(obj, &hwsched->cmd_list, node) {
		struct kgsl_drawobj *drawobj = obj->drawobj;

		if (drawobj->type == SYNCOBJ_TYPE) {
			if ((ctxt_id == drawobj->context->id) &&
			(ts == drawobj->timestamp))
				_print_syncobj(adreno_dev, drawobj);
		}
	}
}

static void adreno_hwsched_reset_and_snapshot_legacy(struct adreno_device *adreno_dev, int fault)
{
	struct kgsl_drawobj *drawobj = NULL;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_context *context = NULL;
	struct cmd_list_obj *obj;
	const struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	struct hfi_context_bad_cmd_legacy *cmd = hwsched->ctxt_bad;

	if (device->state != KGSL_STATE_ACTIVE)
		return;

	if (hwsched->recurring_cmdobj)
		srcu_notifier_call_chain(&device->nh, GPU_SSR_BEGIN, NULL);

	if (cmd->error == GMU_SYNCOBJ_TIMEOUT_ERROR) {
		print_fault_syncobj(adreno_dev, cmd->ctxt_id, cmd->ts);
		gmu_core_fault_snapshot(device, GMU_FAULT_PANIC_NONE);
		goto done;
	}

	/*
	 * First, try to see if the faulted command object is marked
	 * in case there was a context bad hfi. But, with stall-on-fault,
	 * we know that GMU cannot send context bad hfi. Hence, attempt
	 * to walk the list of active submissions to find the one that
	 * faulted.
	 */
	obj = get_fault_cmdobj(adreno_dev, cmd->ctxt_id, cmd->ts);
	if (!obj && (fault & ADRENO_IOMMU_STALL_ON_PAGE_FAULT))
		obj = get_active_cmdobj(adreno_dev);

	if (obj) {
		drawobj = obj->drawobj;
		trace_adreno_cmdbatch_fault(CMDOBJ(drawobj), fault);
	} else if (hwsched->recurring_cmdobj &&
		hwsched->recurring_cmdobj->base.context->id == cmd->ctxt_id) {
		drawobj = DRAWOBJ(hwsched->recurring_cmdobj);
		trace_adreno_cmdbatch_fault(hwsched->recurring_cmdobj, fault);
		if (!kref_get_unless_zero(&drawobj->refcount))
			drawobj = NULL;
	}

	if (drawobj && drawobj->context)
		set_bit(ADRENO_CONTEXT_FAULT, &drawobj->context->priv);

	adreno_gpufault_stats(adreno_dev, drawobj, NULL, fault);

	if (!drawobj) {
		if (fault & ADRENO_GMU_FAULT)
			gmu_core_fault_snapshot(device, GMU_FAULT_PANIC_NONE);
		else
			kgsl_device_snapshot(device, NULL, NULL, false);
		goto done;
	}

	context = drawobj->context;

	do_fault_header(adreno_dev, drawobj, fault);

	kgsl_device_snapshot(device, context, NULL, false);

	force_retire_timestamp(device, drawobj);

	if ((context->flags & KGSL_CONTEXT_INVALIDATE_ON_FAULT) ||
		(context->flags & KGSL_CONTEXT_NO_FAULT_TOLERANCE) ||
		(cmd->error == GMU_GPU_SW_HANG) ||
		(cmd->error == GMU_GPU_SW_FUSE_VIOLATION) ||
		context_is_throttled(device, context)) {
		adreno_drawctxt_set_guilty(device, context);
	}

	/*
	 * Put back the reference which we incremented while trying to find
	 * faulted command object
	 */
	kgsl_drawobj_put(drawobj);
done:
	memset(hwsched->ctxt_bad, 0x0, HFI_MAX_MSG_SIZE);
	gpudev->reset(adreno_dev);
}

static void adreno_hwsched_reset_and_snapshot(struct adreno_device *adreno_dev, int fault)
{
	struct kgsl_drawobj *drawobj = NULL;
	struct kgsl_drawobj *drawobj_lpac = NULL;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_context *context = NULL;
	struct kgsl_context *context_lpac = NULL;
	struct cmd_list_obj *obj;
	struct cmd_list_obj *obj_lpac;
	const struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	struct hfi_context_bad_cmd *cmd = hwsched->ctxt_bad;

	if (device->state != KGSL_STATE_ACTIVE)
		return;

	if (hwsched->recurring_cmdobj)
		srcu_notifier_call_chain(&device->nh, GPU_SSR_BEGIN, NULL);

	if (cmd->error == GMU_SYNCOBJ_TIMEOUT_ERROR) {
		print_fault_syncobj(adreno_dev, cmd->gc.ctxt_id, cmd->gc.ts);
		gmu_core_fault_snapshot(device, GMU_FAULT_PANIC_NONE);
		goto done;
	}

	/*
	 * First, try to see if the faulted command object is marked
	 * in case there was a context bad hfi. But, with stall-on-fault,
	 * we know that GMU cannot send context bad hfi. Hence, attempt
	 * to walk the list of active submissions to find the one that
	 * faulted.
	 */
	obj = get_fault_cmdobj(adreno_dev, cmd->gc.ctxt_id, cmd->gc.ts);
	obj_lpac = get_fault_cmdobj(adreno_dev, cmd->lpac.ctxt_id, cmd->lpac.ts);

	if (!obj && (fault & ADRENO_IOMMU_STALL_ON_PAGE_FAULT))
		obj = get_active_cmdobj(adreno_dev);

	if (obj) {
		drawobj = obj->drawobj;
		CMDOBJ(drawobj)->fault_recovery = cmd->gc.policy;
	} else if (hwsched->recurring_cmdobj &&
		hwsched->recurring_cmdobj->base.context->id == cmd->gc.ctxt_id) {
		drawobj = DRAWOBJ(hwsched->recurring_cmdobj);
		CMDOBJ(drawobj)->fault_recovery = cmd->gc.policy;
		if (!kref_get_unless_zero(&drawobj->refcount))
			drawobj = NULL;
	}

	if (drawobj && drawobj->context)
		set_bit(ADRENO_CONTEXT_FAULT, &drawobj->context->priv);

	do_fault_header(adreno_dev, drawobj, fault);

	if (!obj_lpac && (fault & ADRENO_IOMMU_STALL_ON_PAGE_FAULT))
		obj_lpac = get_active_cmdobj_lpac(adreno_dev);

	if (!obj && !obj_lpac) {
		if (fault & ADRENO_GMU_FAULT)
			gmu_core_fault_snapshot(device, GMU_FAULT_PANIC_NONE);
		else
			kgsl_device_snapshot(device, NULL, NULL, false);

		adreno_gpufault_stats(adreno_dev, NULL, NULL, fault);
		goto done;
	}

	if (obj)
		context = drawobj->context;

	if (obj_lpac) {
		drawobj_lpac = obj_lpac->drawobj;
		CMDOBJ(drawobj_lpac)->fault_recovery = cmd->lpac.policy;
		context_lpac  = drawobj_lpac->context;
		if (gpudev->lpac_fault_header)
			gpudev->lpac_fault_header(adreno_dev, drawobj_lpac);
	}

	if (drawobj_lpac && drawobj_lpac->context)
		set_bit(ADRENO_CONTEXT_FAULT, &drawobj_lpac->context->priv);

	kgsl_device_snapshot(device, context, context_lpac, false);
	adreno_gpufault_stats(adreno_dev, drawobj, drawobj_lpac, fault);

	if (drawobj) {
		force_retire_timestamp(device, drawobj);
		if (context && ((context->flags & KGSL_CONTEXT_INVALIDATE_ON_FAULT) ||
			(context->flags & KGSL_CONTEXT_NO_FAULT_TOLERANCE) ||
			(cmd->error == GMU_GPU_SW_HANG) ||
			(cmd->error == GMU_GPU_SW_FUSE_VIOLATION) ||
			context_is_throttled(device, context)))
			adreno_drawctxt_set_guilty(device, context);
		/*
		 * Put back the reference which we incremented while trying to find
		 * faulted command object
		 */
		kgsl_drawobj_put(drawobj);
	}

	if (drawobj_lpac) {
		force_retire_timestamp(device, drawobj_lpac);
		if (context_lpac && ((context_lpac->flags & KGSL_CONTEXT_INVALIDATE_ON_FAULT) ||
			(context_lpac->flags & KGSL_CONTEXT_NO_FAULT_TOLERANCE) ||
			(cmd->error == GMU_GPU_SW_HANG) ||
			(cmd->error == GMU_GPU_SW_FUSE_VIOLATION) ||
			context_is_throttled(device, context_lpac)))
			adreno_drawctxt_set_guilty(device, context_lpac);
		/*
		 * Put back the reference which we incremented while trying to find
		 * faulted command object
		 */
		kgsl_drawobj_put(drawobj_lpac);
	}
done:
	memset(hwsched->ctxt_bad, 0x0, HFI_MAX_MSG_SIZE);
	gpudev->reset(adreno_dev);
}

static bool adreno_hwsched_do_fault(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	int fault;

	fault = adreno_gpu_fault(adreno_dev);
	if (fault == 0)
		return false;

	/*
	 * Return early if there is a concurrent suspend in progress. The suspend thread will error
	 * out in the presence of this hwsched fault and requeue the dispatcher to handle this fault
	 */
	if (!mutex_trylock(&adreno_dev->fault_recovery_mutex))
		return true;

	/*
	 * Wait long enough to allow the system to come out of suspend completely, which can take
	 * variable amount of time especially if it has to rewind suspend processes and devices.
	 */
	if (!wait_for_completion_timeout(&adreno_dev->suspend_recovery_gate,
			msecs_to_jiffies(ADRENO_SUSPEND_RECOVERY_GATE_TIMEOUT_MS))) {
		dev_err(device->dev, "suspend recovery gate timeout\n");
		adreno_scheduler_queue(adreno_dev);
		mutex_unlock(&adreno_dev->fault_recovery_mutex);
		return true;
	}

	mutex_lock(&device->mutex);

	if (test_bit(ADRENO_HWSCHED_CTX_BAD_LEGACY, &hwsched->flags))
		adreno_hwsched_reset_and_snapshot_legacy(adreno_dev, fault);
	else
		adreno_hwsched_reset_and_snapshot(adreno_dev, fault);

	adreno_scheduler_queue(adreno_dev);

	mutex_unlock(&device->mutex);
	mutex_unlock(&adreno_dev->fault_recovery_mutex);

	return true;
}

static void adreno_hwsched_work(struct kthread_work *work)
{
	struct adreno_device *adreno_dev = container_of(work,
			struct adreno_device, scheduler_work);
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	mutex_lock(&hwsched->mutex);

	if (adreno_hwsched_do_fault(adreno_dev)) {
		mutex_unlock(&hwsched->mutex);
		return;
	}

	/*
	 * As long as there are inflight commands, process retired comamnds from
	 * all drawqueues
	 */
	retire_drawobj_list(adreno_dev);

	/* Signal fences */
	kgsl_process_event_groups(device);

	/* Run the scheduler for to dispatch new commands */
	hwsched_issuecmds(adreno_dev);

	if (hwsched->inflight == 0) {
		hwsched_power_down(adreno_dev);
	} else {
		mutex_lock(&device->mutex);
		kgsl_pwrscale_update(device);
		kgsl_start_idle_timer(device);
		mutex_unlock(&device->mutex);
	}

	mutex_unlock(&hwsched->mutex);
}

static void adreno_hwsched_create_hw_fence(struct adreno_device *adreno_dev,
	struct kgsl_sync_fence *kfence)
{
	struct kgsl_sync_timeline *ktimeline = kfence->parent;
	struct kgsl_context *context = ktimeline->context;
	const struct adreno_hwsched_ops *hwsched_ops =
				adreno_dev->hwsched.hwsched_ops;

	if (!test_bit(ADRENO_HWSCHED_HW_FENCE, &adreno_dev->hwsched.flags))
		return;

	/* Do not create a hardware backed fence, if this context is bad or going away */
	if (kgsl_context_is_bad(context))
		return;

	if (!kgsl_hw_fence_tx_slot_available(KGSL_DEVICE(adreno_dev),
		&adreno_dev->hwsched.hw_fence_count))
		return;

	hwsched_ops->create_hw_fence(adreno_dev, kfence);
}

static const struct adreno_dispatch_ops hwsched_ops = {
	.close = adreno_hwsched_dispatcher_close,
	.queue_cmds = adreno_hwsched_queue_cmds,
	.queue_context = adreno_hwsched_queue_context,
	.create_hw_fence = adreno_hwsched_create_hw_fence,
};

static void hwsched_lsr_check(struct work_struct *work)
{
	struct adreno_hwsched *hwsched = container_of(work,
		struct adreno_hwsched, lsr_check_ws);
	struct adreno_device *adreno_dev = container_of(hwsched,
		struct adreno_device, hwsched);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	mutex_lock(&device->mutex);
	kgsl_pwrscale_update_stats(device);
	kgsl_pwrscale_update(device);
	mutex_unlock(&device->mutex);

	mod_timer(&hwsched->lsr_timer, jiffies + msecs_to_jiffies(10));
}

static void hwsched_lsr_timer(struct timer_list *t)
{
	struct adreno_hwsched *hwsched = container_of(t, struct adreno_hwsched,
					lsr_timer);

	kgsl_schedule_work(&hwsched->lsr_check_ws);
}

int adreno_hwsched_init(struct adreno_device *adreno_dev,
	const struct adreno_hwsched_ops *target_hwsched_ops)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	int i;

	memset(hwsched, 0, sizeof(*hwsched));

	hwsched->ctxt_bad = kzalloc(HFI_MAX_MSG_SIZE, GFP_KERNEL);
	if (!hwsched->ctxt_bad)
		return -ENOMEM;

	adreno_dev->scheduler_worker = kthread_create_worker(0, "kgsl_hwsched");
	if (IS_ERR(adreno_dev->scheduler_worker)) {
		kfree(hwsched->ctxt_bad);
		return PTR_ERR(adreno_dev->scheduler_worker);
	}

	mutex_init(&hwsched->mutex);

	kthread_init_work(&adreno_dev->scheduler_work, adreno_hwsched_work);

	jobs_cache = KMEM_CACHE(adreno_dispatch_job, 0);
	obj_cache = KMEM_CACHE(cmd_list_obj, 0);

	INIT_LIST_HEAD(&hwsched->cmd_list);

	for (i = 0; i < ARRAY_SIZE(hwsched->jobs); i++) {
		init_llist_head(&hwsched->jobs[i]);
		init_llist_head(&hwsched->requeue[i]);
	}

	sched_set_fifo(adreno_dev->scheduler_worker->task);

	WARN_ON(sysfs_create_files(&device->dev->kobj, _hwsched_attr_list));
	adreno_set_dispatch_ops(adreno_dev, &hwsched_ops);
	hwsched->hwsched_ops = target_hwsched_ops;
	init_completion(&hwsched->idle_gate);
	complete_all(&hwsched->idle_gate);

	if (ADRENO_FEATURE(adreno_dev, ADRENO_LSR)) {
		INIT_WORK(&hwsched->lsr_check_ws, hwsched_lsr_check);
		timer_setup(&hwsched->lsr_timer, hwsched_lsr_timer, 0);
	}

	return 0;
}

void adreno_hwsched_parse_fault_cmdobj(struct adreno_device *adreno_dev,
	struct kgsl_snapshot *snapshot)
{
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	struct cmd_list_obj *obj, *tmp;

	/*
	 * During IB parse, vmalloc is called which can sleep and
	 * should not be called from atomic context. Since IBs are not
	 * dumped during atomic snapshot, there is no need to parse it.
	 */
	if (adreno_dev->dev.snapshot_atomic)
		return;

	list_for_each_entry_safe(obj, tmp, &hwsched->cmd_list, node) {
		struct kgsl_drawobj *drawobj = obj->drawobj;
		struct kgsl_drawobj_cmd *cmdobj;

		if (!is_cmdobj(drawobj))
			continue;

		cmdobj = CMDOBJ(drawobj);

		if (test_bit(CMDOBJ_FAULT, &cmdobj->priv)) {
			struct kgsl_memobj_node *ib;

			list_for_each_entry(ib, &cmdobj->cmdlist, node) {
				if (drawobj->context->flags & KGSL_CONTEXT_LPAC)
					adreno_parse_ib_lpac(KGSL_DEVICE(adreno_dev),
						snapshot, snapshot->process_lpac,
						ib->gpuaddr, ib->size >> 2);
				else
					adreno_parse_ib(KGSL_DEVICE(adreno_dev),
						snapshot, snapshot->process,
						ib->gpuaddr, ib->size >> 2);
			}
			clear_bit(CMDOBJ_FAULT, &cmdobj->priv);
		}
	}
}

static int unregister_context(int id, void *ptr, void *data)
{
	struct kgsl_context *context = ptr;
	struct adreno_context *drawctxt = ADRENO_CONTEXT(context);

	if (drawctxt->gmu_context_queue.gmuaddr != 0) {
		struct gmu_context_queue_header *header =  drawctxt->gmu_context_queue.hostptr;

		header->read_index = header->write_index;
		/* This is to make sure GMU sees the correct indices after recovery */
		mb();
	}

	/*
	 * We don't need to send the unregister hfi packet because
	 * we are anyway going to lose the gmu state of registered
	 * contexts. So just reset the flag so that the context
	 * registers with gmu on its first submission post slumber.
	 */
	context->gmu_registered = false;

	/*
	 * Consider the scenario where non-recurring submissions were made
	 * by a context. Here internal_timestamp of context would be non
	 * zero. After slumber, last retired timestamp is not held by GMU.
	 * If this context submits a recurring workload, the context is
	 * registered again, but the internal timestamp is not updated. When
	 * the context is unregistered in send_context_unregister_hfi(),
	 * we could be waiting on old internal_timestamp which is not held by
	 * GMU. This can result in GMU errors. Hence set internal_timestamp
	 * to zero when entering slumber.
	 */
	drawctxt->internal_timestamp = 0;

	return 0;
}

void adreno_hwsched_unregister_contexts(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;

	read_lock(&device->context_lock);
	idr_for_each(&device->context_idr, unregister_context, NULL);
	read_unlock(&device->context_lock);

	if (hwsched->global_ctxtq.hostptr) {
		struct gmu_context_queue_header *header = hwsched->global_ctxtq.hostptr;

		header->read_index = header->write_index;
		/* This is to make sure GMU sees the correct indices after recovery */
		mb();
	}

	hwsched->global_ctxt_gmu_registered = false;
}

int adreno_gmu_context_queue_read(struct adreno_context *drawctxt, u32 *output,
	u32 read_idx, u32 size_dwords)
{
	struct gmu_context_queue_header *hdr = drawctxt->gmu_context_queue.hostptr;
	u32 *queue = drawctxt->gmu_context_queue.hostptr + sizeof(*hdr);
	u32 i;

	if ((size_dwords > hdr->queue_size) || (read_idx >= hdr->queue_size))
		return -EINVAL;

	/* Clear the output data before populating */
	memset(output, 0, size_dwords << 2);

	for (i = 0; i < size_dwords; i++) {
		output[i] = queue[read_idx];
		read_idx = (read_idx + 1) % hdr->queue_size;
	}

	return 0;
}

static int hwsched_idle(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	int ret;

	/* Block any new submissions from being submitted */
	adreno_get_gpu_halt(adreno_dev);

	mutex_unlock(&device->mutex);

	/*
	 * Flush the worker to make sure all executing
	 * or pending dispatcher works on worker are
	 * finished
	 */
	kthread_flush_worker(adreno_dev->scheduler_worker);

	if (adreno_gpu_fault(adreno_dev)) {
		ret = -EDEADLK;
	} else {

		ret = wait_for_completion_timeout(&hwsched->idle_gate,
				msecs_to_jiffies(ADRENO_IDLE_TIMEOUT));
		if (ret == 0) {
			ret = -ETIMEDOUT;
			WARN(1, "hwsched halt timeout\n");
		} else if (ret < 0) {
			dev_err(device->dev, "hwsched halt failed %d\n", ret);
		} else {
			ret = 0;
		}

		if (adreno_gpu_fault(adreno_dev))
			ret = -EDEADLK;
	}

	mutex_lock(&device->mutex);

	/*
	 * This will allow the dispatcher to start submitting to
	 * hardware once device mutex is released
	 */
	adreno_put_gpu_halt(adreno_dev);

	return ret;
}

int adreno_hwsched_idle(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	unsigned long wait = jiffies + msecs_to_jiffies(ADRENO_IDLE_TIMEOUT);
	const struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	int ret;

	if (WARN_ON(!mutex_is_locked(&device->mutex)))
		return -EDEADLK;

	if (!kgsl_state_is_awake(device))
		return 0;

	ret = hwsched_idle(adreno_dev);
	if (ret)
		return ret;

	do {
		if (adreno_gpu_fault(adreno_dev))
			return -EDEADLK;

		if (gpudev->hw_isidle(adreno_dev))
			return 0;
	} while (time_before(jiffies, wait));

	/*
	 * Under rare conditions, preemption can cause the while loop to exit
	 * without checking if the gpu is idle. check one last time before we
	 * return failure.
	 */
	if (adreno_gpu_fault(adreno_dev))
		return -EDEADLK;

	if (gpudev->hw_isidle(adreno_dev))
		return 0;

	return -ETIMEDOUT;
}

void adreno_hwsched_register_hw_fence(struct adreno_device *adreno_dev)
{
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	int ret;

	if (!ADRENO_FEATURE(adreno_dev, ADRENO_HW_FENCE))
		return;

	if (test_bit(ADRENO_HWSCHED_HW_FENCE, &hwsched->flags))
		return;

	if (kgsl_hw_fence_init(device))
		return;

	/*
	 * We need to set up the memory descriptor with the physical address of the Tx/Rx Queues so
	 * that these buffers can be imported in to GMU VA space
	 */
	kgsl_memdesc_init(device, &hwsched->hw_fence_md, 0);
	kgsl_hw_fence_populate_md(device, &hwsched->hw_fence_md);

	ret = kgsl_memdesc_sg_dma(&hwsched->hw_fence_md, hwsched->hw_fence_md.physaddr,
		hwsched->hw_fence_md.size);
	if (ret) {
		dev_err(device->dev, "Failed to setup HW fences memdesc: %d\n",
			ret);
		kgsl_hw_fence_close(device);
		memset(&hwsched->hw_fence_md, 0x0, sizeof(hwsched->hw_fence_md));
		return;
	}

	hwsched->hw_fence_cache = KMEM_CACHE(adreno_hw_fence_entry, 0);

	set_bit(ADRENO_HWSCHED_HW_FENCE, &hwsched->flags);
}

int adreno_hwsched_wait_ack_completion(struct adreno_device *adreno_dev,
	struct device *dev, struct pending_cmd *ack,
	void (*process_msgq)(struct adreno_device *adreno_dev))
{
	int rc;
	/* Only allow a single log in a second */
	static DEFINE_RATELIMIT_STATE(_rs, HZ, 1);
	static u32 unprocessed, processed;
	const struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	u64 start, end;

	start = gpudev->read_alwayson(adreno_dev);
	rc = wait_for_completion_timeout(&ack->complete,
		msecs_to_jiffies(HFI_RSP_TIMEOUT));
	/*
	 * A non-zero return value means the completion is complete, whereas zero indicates
	 * timeout
	 */
	if (rc) {
		/*
		 * If an ack goes unprocessed, keep track of processed and unprocessed acks
		 * because we may not log each unprocessed ack due to ratelimiting
		 */
		if (unprocessed)
			processed++;
		return 0;
	}

	/*
	 * It is possible the ack came, but due to HLOS latencies in processing hfi interrupt
	 * and/or the f2h daemon, the ack isn't processed yet. Hence, process the msgq one last
	 * time.
	 */
	process_msgq(adreno_dev);
	end = gpudev->read_alwayson(adreno_dev);
	if (completion_done(&ack->complete)) {
		unprocessed++;
		if (__ratelimit(&_rs))
			dev_err(dev, "Ack unprocessed for id:%d sequence=%d count=%d/%d ticks=0x%llx/0x%llx\n",
				MSG_HDR_GET_ID(ack->sent_hdr), MSG_HDR_GET_SEQNUM(ack->sent_hdr),
				unprocessed, processed, start, end);
		return 0;
	}

	dev_err(dev, "Ack timeout for id:%d sequence=%d ticks=0x%llx/0x%llx\n",
		MSG_HDR_GET_ID(ack->sent_hdr), MSG_HDR_GET_SEQNUM(ack->sent_hdr), start, end);
	gmu_core_fault_snapshot(KGSL_DEVICE(adreno_dev), GMU_FAULT_WAIT_ACK_COMPLETION);
	return -ETIMEDOUT;
}

int adreno_hwsched_ctxt_unregister_wait_completion(
	struct adreno_device *adreno_dev,
	struct device *dev, struct pending_cmd *ack,
	void (*process_msgq)(struct adreno_device *adreno_dev),
	struct hfi_unregister_ctxt_cmd *cmd)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	const struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	int ret;
	u64 start, end;

	start = gpudev->read_alwayson(adreno_dev);
	mutex_unlock(&device->mutex);

	ret = wait_for_completion_timeout(&ack->complete,
		msecs_to_jiffies(msecs_to_jiffies(30 * 1000)));

	mutex_lock(&device->mutex);
	if (ret)
		return 0;

	/*
	 * It is possible the ack came, but due to HLOS latencies in processing hfi interrupt
	 * and/or the f2h daemon, the ack isn't processed yet. Hence, process the msgq one last
	 * time.
	 */
	process_msgq(adreno_dev);
	end = gpudev->read_alwayson(adreno_dev);

	if (completion_done(&ack->complete)) {
		dev_err_ratelimited(dev,
			"Ack unprocessed for context unregister seq: %d ctx: %u ts: %u ticks=0x%llx/0x%llx\n",
			MSG_HDR_GET_SEQNUM(ack->sent_hdr), cmd->ctxt_id,
			cmd->ts, start, end);
		return 0;
	}

	dev_err_ratelimited(dev,
		"Ack timeout for context unregister seq: %d ctx: %u ts: %u ticks=0x%llx/0x%llx\n",
		MSG_HDR_GET_SEQNUM(ack->sent_hdr), cmd->ctxt_id, cmd->ts, start, end);
	return -ETIMEDOUT;
}

u32 adreno_hwsched_parse_payload(struct payload_section *payload, u32 key)
{
	u32 i;

	/* Each key-value pair is 2 dwords */
	for (i = 0; i < payload->dwords; i += 2) {
		if (payload->data[i] == key)
			return payload->data[i + 1];
	}

	return 0;
}

void adreno_hwsched_log_destroy_pending_hw_fences(struct adreno_device *adreno_dev,
	struct device *dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct adreno_hw_fence_entry entries[5];
	struct adreno_hw_fence_entry *entry, *next;
	struct kgsl_context *context;
	int id, i;
	u32 count = 0;

	read_lock(&device->context_lock);
	idr_for_each_entry(&device->context_idr, context, id) {
		struct adreno_context *drawctxt = ADRENO_CONTEXT(context);

		spin_lock(&drawctxt->lock);

		list_for_each_entry_safe(entry, next, &drawctxt->hw_fence_list, node) {
			if (count < ARRAY_SIZE(entries))
				memcpy(&entries[count], entry, sizeof(*entry));
			count++;
			kgsl_hw_fence_destroy(entry->kfence);
			adreno_hwsched_remove_hw_fence_entry(adreno_dev, entry);
		}

		spin_unlock(&drawctxt->lock);
	}
	read_unlock(&device->context_lock);

	if (!count)
		return;

	dev_err(dev, "%d hw fences may not be signaled. %s are:\n", count,
		count > 5 ? "First 5" : "They");

	for (i = 0; (i < count) && (i < ARRAY_SIZE(entries)); i++)
		dev_err(dev, "%d: ctx=%llu seqno=%llu\n", i, entries[i].cmd.ctxt_id,
			entries[i].cmd.ts);
}

static void adreno_hwsched_lookup_key_value(struct adreno_device *adreno_dev,
		u32 type, u32 key, u32 *ptr, u32 num_values)
{
	struct hfi_context_bad_cmd *cmd = adreno_dev->hwsched.ctxt_bad;
	u32 i = 0, payload_bytes;
	void *start;

	if (!cmd->hdr)
		return;

	payload_bytes = (MSG_HDR_GET_SIZE(cmd->hdr) << 2) -
			offsetof(struct hfi_context_bad_cmd, payload);

	start = &cmd->payload[0];

	while (i < payload_bytes) {
		struct payload_section *payload = start + i;

		/* key-value pair is 'num_values + 1' dwords */
		if ((payload->type == type) && (payload->data[i] == key)) {
			u32 j = 1;

			while (num_values--) {
				ptr[j - 1] = payload->data[i + j];
				j++;
			}
			break;
		}

		i += struct_size(payload, data, payload->dwords);
	}
}

bool adreno_hwsched_log_nonfatal_gpu_fault(struct adreno_device *adreno_dev,
		struct device *dev, u32 error)
{
	bool non_fatal = true;

	switch (error) {
	case GMU_CP_AHB_ERROR: {
		u32 err_details[2];

		adreno_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
						KEY_CP_AHB_ERROR, err_details, 2);
		dev_crit_ratelimited(dev,
			"CP: AHB bus error, CP_RL_ERROR_DETAILS_0:0x%x CP_RL_ERROR_DETAILS_1:0x%x\n",
			err_details[0], err_details[1]);
		break;
	}
	case GMU_ATB_ASYNC_FIFO_OVERFLOW:
		dev_crit_ratelimited(dev, "RBBM: ATB ASYNC overflow\n");
		break;
	case GMU_RBBM_ATB_BUF_OVERFLOW:
		dev_crit_ratelimited(dev, "RBBM: ATB bus overflow\n");
		break;
	case GMU_UCHE_OOB_ACCESS:
		dev_crit_ratelimited(dev, "UCHE: Out of bounds access\n");
		break;
	case GMU_UCHE_TRAP_INTR:
		dev_crit_ratelimited(dev, "UCHE: Trap interrupt\n");
		break;
	case GMU_TSB_WRITE_ERROR: {
		u32 addr[2];

		adreno_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
						KEY_TSB_WRITE_ERROR, addr, 2);
		dev_crit_ratelimited(dev, "TSB: Write error interrupt: Address: 0x%lx MID: %lu\n",
			FIELD_GET(GENMASK(16, 0), addr[1]) << 32 | addr[0],
			FIELD_GET(GENMASK(31, 23), addr[1]));
		break;
	}
	default:
		non_fatal = false;
		break;
	}

	return non_fatal;
}

int adreno_hwsched_poll_msg_queue_write_index(struct kgsl_memdesc *hfi_mem)
{
	struct hfi_queue_table *tbl = hfi_mem->hostptr;
	struct hfi_queue_header *hdr = &tbl->qhdr[HFI_MSG_ID];
	unsigned long timeout = jiffies + msecs_to_jiffies(HFI_RSP_TIMEOUT);

	while (time_before(jiffies, timeout)) {
		if (hdr->write_index != hdr->read_index)
			goto done;

		/* Wait for upto 100 us before trying again */
		usleep_range((POLL_SLEEP_US >> 2) + 1, POLL_SLEEP_US);
		cpu_relax();
	}

	/* Check if the write index has advanced */
	if (hdr->write_index == hdr->read_index)
		return -ETIMEDOUT;

done:
	/*
	 * This is to ensure that the queue is not read speculatively before the
	 * polling condition is evaluated.
	 */
	rmb();

	return 0;
}

/* This function can be called while holding the drawctxt spinlock */
void adreno_hwsched_remove_hw_fence_entry(struct adreno_device *adreno_dev,
	struct adreno_hw_fence_entry *entry)
{
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	struct adreno_context *drawctxt = entry->drawctxt;

	atomic_dec(&hwsched->hw_fence_count);
	drawctxt->hw_fence_count--;

	dma_fence_put(&entry->kfence->fence);
	list_del_init(&entry->node);
	kmem_cache_free(hwsched->hw_fence_cache, entry);
	kgsl_context_put_deferred(&drawctxt->base);
}

void adreno_hwsched_add_profile_events(struct adreno_device *adreno_dev,
	struct kgsl_drawobj_cmd *cmdobj, struct adreno_submit_time *time)
{
	unsigned long flags;
	u64 time_in_s;
	unsigned long time_in_ns;
	struct kgsl_drawobj *drawobj = DRAWOBJ(cmdobj);
	struct kgsl_context *context = drawobj->context;
	struct submission_info info = {0};
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	const struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);

	if (!time)
		return;

	/*
	 * Here we are attempting to create a mapping between the
	 * GPU time domain (alwayson counter) and the CPU time domain
	 * (local_clock) by sampling both values as close together as
	 * possible. This is useful for many types of debugging and
	 * profiling. In order to make this mapping as accurate as
	 * possible, we must turn off interrupts to avoid running
	 * interrupt handlers between the two samples.
	 */

	local_irq_save(flags);

	/* Read always on registers */
	time->ticks = gpudev->read_alwayson(adreno_dev);

	/* Trace the GPU time to create a mapping to ftrace time */
	trace_adreno_cmdbatch_sync(context->id, context->priority,
		drawobj->timestamp, time->ticks);

	/* Get the kernel clock for time since boot */
	time->ktime = local_clock();

	/* Get the timeofday for the wall time (for the user) */
	ktime_get_real_ts64(&time->utime);

	local_irq_restore(flags);

	/* Return kernel clock time to the client if requested */
	time_in_s = time->ktime;
	time_in_ns = do_div(time_in_s, 1000000000);

	info.inflight = hwsched->inflight;
	info.rb_id = adreno_get_level(context);
	info.gmu_dispatch_queue = context->gmu_dispatch_queue;

	cmdobj->submit_ticks = time->ticks;

	msm_perf_events_update(MSM_PERF_GFX, MSM_PERF_SUBMIT,
		pid_nr(context->proc_priv->pid),
		context->id, drawobj->timestamp,
		!!(drawobj->flags & KGSL_DRAWOBJ_END_OF_FRAME));
	trace_adreno_cmdbatch_submitted(drawobj, &info, time->ticks,
		(unsigned long) time_in_s, time_in_ns / 1000, 0);

	log_kgsl_cmdbatch_submitted_event(context->id, drawobj->timestamp,
			context->priority, drawobj->flags);
}

int adreno_gmu_context_queue_write(struct adreno_device *adreno_dev,
	struct kgsl_memdesc *gmu_context_queue, u32 *msg, u32 size_bytes,
	struct kgsl_drawobj *drawobj, struct adreno_submit_time *time)
{
	struct gmu_context_queue_header *hdr = gmu_context_queue->hostptr;
	const struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	u32 *queue = gmu_context_queue->hostptr + sizeof(*hdr);
	u32 i, empty_space, write_idx = hdr->write_index, read_idx = hdr->read_index;
	u32 size_dwords = size_bytes >> 2;
	u32 align_size = ALIGN(size_dwords, SZ_4);
	u32 id = MSG_HDR_GET_ID(*msg);
	struct kgsl_drawobj_cmd *cmdobj = NULL;

	empty_space = (write_idx >= read_idx) ?
			(hdr->queue_size - (write_idx - read_idx))
			: (read_idx - write_idx);

	if (empty_space <= align_size)
		return -ENOSPC;

	if (!IS_ALIGNED(size_bytes, sizeof(u32)))
		return -EINVAL;

	for (i = 0; i < size_dwords; i++) {
		queue[write_idx] = msg[i];
		write_idx = (write_idx + 1) % hdr->queue_size;
	}

	/* Cookify any non used data at the end of the write buffer */
	for (; i < align_size; i++) {
		queue[write_idx] = 0xfafafafa;
		write_idx = (write_idx + 1) % hdr->queue_size;
	}

	/* Ensure packet is written out before proceeding */
	wmb();

	if (!drawobj)
		goto done;

	if (drawobj->type & SYNCOBJ_TYPE) {
		struct kgsl_drawobj_sync *syncobj = SYNCOBJ(drawobj);

		trace_adreno_syncobj_submitted(drawobj->context->id, drawobj->timestamp,
			syncobj->num_hw_fence, gpudev->read_alwayson(adreno_dev));
		goto done;
	}

	cmdobj = CMDOBJ(drawobj);

	adreno_hwsched_add_profile_events(adreno_dev, cmdobj, time);

	/*
	 * Put the profiling information in the user profiling buffer.
	 * The hfi_update_write_idx below has a wmb() before the actual
	 * write index update to ensure that the GMU does not see the
	 * packet before the profile data is written out.
	 */
	adreno_profile_submit_time(time);

done:
	trace_kgsl_hfi_send(id, size_dwords, MSG_HDR_GET_SEQNUM(*msg));

	hfi_update_write_idx(&hdr->write_index, write_idx);

	return 0;
}
