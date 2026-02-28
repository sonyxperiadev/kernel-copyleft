// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/moduleparam.h>
#include "rmnet_offload_knob.h"
#include "rmnet_offload_main.h"
#include "rmnet_offload_engine.h"

/* OK, this whole song and dance requires some explanation.
 *
 * There are several things that I needed to satisfy when creating this
 * framework:
 *  1) The setting of any parameter NEEDS to be synchronized with the rest
 *     of the module. We have to take the lock BEFORE the value is changed,
 *     allow the module to react to the new value as necessary, set the
 *     new value, then unlock. This requires custom SET operations for each
 *     parameter.
 *  2) Each parameter has a different range of acceptable values, and a
 *     different starting value. The handler for each knob must be aware of
 *     these values and enfore them.
 *  3) The external parameter name should be purposely vague (knob0, knob1)
 *     and different than the internal stat name.
 *
 * (1) and (2) taken together results in the core of the knob framework. Since
 * much of the handling of setting a knob value is the same, having a common
 * handler is desirable. Handlers for each knob value should tell this main
 * handler what knob they are, and what their acceptable value range is. As the
 * arguments to the SET handlers for kernel params lack any information about
 * knob is being set, each knob requires its own handler that passes a hard-
 * coded value to the main handler to look up the appropriate value ranges.
 * This means that each knob must be exported individually; we cannot use an
 * array of module parameters like we can with the statistics as there's no eay
 * of specifying a different set of of kernel_param_ops for each element.
 *
 * Unfortunately, this API requirement makes (3) more difficult for the
 * programmer because of the C preprocessor. You can't simply
 * make macros for each configuratble knob and append them to some
 * vague stem name with ##. The C preprocessor will not resolve arithmetic, and
 * the resulting name, say rmnet_offload_knob(x-y+z), is definitely not a valid
 * identifier name. GCC rejects this as an invalid symbol when concatenating
 * with ## and terminates the compilation. As such, you have to name each knob
 * by hand. Sorry about that...
 *
 * Thus, the final workflow is this:
 *  1) Call RMNET_OFFLOAD_KNOB_HANDLER() with the knob's enum value to set up
 *     the custom SET function
 *  2) Add RMNET_OFFLOAD_KNOB_DECLARE() to the main rmnet_offload_knobs[] with
 *     the range of acceptable values, the starting value, and any callback
 *     needed for the module to take any appropriate action before the value
 *     is changed.
 *  3) Call RMNET_OFFLOAD_KNOB_INIT() with the external name for your new knob
 *     to register the final module param with the kernel.
 */

#define RMNET_OFFLOAD_KNOB_HANDLER(knob) \
	static int __ ## knob(const char *val, const struct kernel_param *kp) \
	{ \
		return __rmnet_offload_knob_set(val, kp, knob); \
	}

#define RMNET_OFFLOAD_KNOB_DECLARE(knob, def_val, min_val, max_val, cb) \
	(struct __rmnet_offload_knob) { \
		.knob_val = def_val, \
		.knob_min = min_val, \
		.knob_max = max_val, \
		.knob_cb = cb, \
		.knob_ops = { \
			.set = __ ## knob, \
			.get = param_get_ullong, \
		}, \
	}

#define RMNET_OFFLOAD_KNOB_INIT(knob_name, knob) \
	module_param_cb(knob_name, &rmnet_offload_knobs[knob].knob_ops, \
			&rmnet_offload_knobs[knob].knob_val, 0644)

struct __rmnet_offload_knob {
	u64 knob_val;
	u64 knob_min;
	u64 knob_max;
	int (*knob_cb)(u64 old_val, u64 new_val);
	struct kernel_param_ops knob_ops;
};

/* Forward declaration of our main value setting handler */
static int __rmnet_offload_knob_set(const char *val,
				    const struct kernel_param *kp, u32 knob);

/* Initialize the handlers for each knob */
RMNET_OFFLOAD_KNOB_HANDLER(RMNET_OFFLOAD_KNOB_TCP_BYTE_LIMIT);
RMNET_OFFLOAD_KNOB_HANDLER(RMNET_OFFLOAD_KNOB_UDP_BYTE_LIMIT);
RMNET_OFFLOAD_KNOB_HANDLER(RMNET_OFFLOAD_KNOB_ENGINE_MODE);

/* Our knob array. This stores the knob metadata (range of values, get and set
 * operations, callback, initial value), and the current value of the knob.
 */
static struct __rmnet_offload_knob
rmnet_offload_knobs[RMNET_OFFLOAD_KNOB_MAX] = {
	RMNET_OFFLOAD_KNOB_DECLARE(RMNET_OFFLOAD_KNOB_TCP_BYTE_LIMIT, 65000,
				   0, 65000, NULL),
	RMNET_OFFLOAD_KNOB_DECLARE(RMNET_OFFLOAD_KNOB_UDP_BYTE_LIMIT, 65000,
				   0, 65000, NULL),
	RMNET_OFFLOAD_KNOB_DECLARE(RMNET_OFFLOAD_KNOB_ENGINE_MODE,
				   RMNET_OFFLOAD_ENGINE_MODE_ALL,
				   RMNET_OFFLOAD_ENGINE_MODE_MIN,
				   RMNET_OFFLOAD_ENGINE_MODE_MAX,
				   rmnet_offload_engine_mode_change),
};

/* Handle changing the knob value. Checks to make sure the value given is in
 * range, and informs the rest of the module of the change if needed.
 */
static int __rmnet_offload_knob_set(const char *val,
				    const struct kernel_param *kp, u32 knob)
{
	struct __rmnet_offload_knob *knob_def;
	unsigned long long new_val;
	u64 old_val;
	int rc;

	/* Protext us from ourselves */
	if (knob >= RMNET_OFFLOAD_KNOB_MAX)
		return -EINVAL;

	/* Extract the value from the string. Very similar to param_set_ullong,
	 * but I don't want to trash the old value immediately.
	 */
	rc = kstrtoull(val, 0, &new_val);
	if (rc < 0)
		return rc;

	/* Ensure value is within bounds */
	knob_def = &rmnet_offload_knobs[knob];
	if ((u64)new_val < knob_def->knob_min ||
	    (u64)new_val > knob_def->knob_max)
		return -ERANGE;

	/* Lock ourselves down for synchronization with packet processing */
	rmnet_offload_lock();
	old_val = *(u64 *)kp->arg;
	if ((u64)new_val == old_val) {
		/* Nothing to change. Let's bail early */
		rmnet_offload_unlock();
		return 0;
	}

	if (knob_def->knob_cb) {
		rc = knob_def->knob_cb(old_val, (u64)new_val);
		if (rc < 0) {
			rmnet_offload_unlock();
			return rc;
		}
	}

	/* Set the new value */
	*(u64 *)kp->arg = (u64)new_val;
	rmnet_offload_unlock();
	return 0;
}

/* Create the module parameters. */
RMNET_OFFLOAD_KNOB_INIT(rmnet_offload_knob0, RMNET_OFFLOAD_KNOB_TCP_BYTE_LIMIT);
RMNET_OFFLOAD_KNOB_INIT(rmnet_offload_knob1, RMNET_OFFLOAD_KNOB_UDP_BYTE_LIMIT);
RMNET_OFFLOAD_KNOB_INIT(rmnet_offload_knob2, RMNET_OFFLOAD_KNOB_ENGINE_MODE);

/* Retrieve the value of a knob */
u64 rmnet_offload_knob_get(u32 knob) {
	struct __rmnet_offload_knob *knob_def;

	if (knob >= RMNET_OFFLOAD_KNOB_MAX)
		return (u64)~0;

	knob_def = &rmnet_offload_knobs[knob];
	return knob_def->knob_val;
}
