// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023, 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#define pr_fmt(fmt) "smcinvoke_kernel: %s: " fmt, __func__

#include "IClientEnv.h"

#if IS_ENABLED(CONFIG_QSEECOM_COMPAT)
#include "../IQSEEComCompat.h"
#include "../IQSEEComCompatAppLoader.h"
#include "linux/qseecom_api.h"
#if IS_ENABLED(CONFIG_QSEECOM_PROXY)
#include <linux/qseecom_kernel.h>
#else
#include "misc/qseecom_kernel.h"
#endif
#endif

#undef OBJECT_OP_METHOD_MASK
#undef OBJECT_OP_METHOD_ID
#undef OBJECT_OP_RELEASE
#undef OBJECT_OP_RETAIN
#include "smci_impl.h"

static int32_t do_invoke(void *context, uint32_t op,
	union ObjectArg *args, uint32_t counts);

/* Marshaling APIs. */

static void marshal_in(struct si_arg u[], union ObjectArg args[], uint32_t counts)
{
	int i;

	FOR_ARGS(i, counts, BI) {
		u[i].b.addr = args[i].b.ptr;
		u[i].b.size = args[i].b.size;

		u[i].type = SI_AT_IB;
	}

	FOR_ARGS(i, counts, BO) {
		u[i].b.addr = args[i].b.ptr;
		u[i].b.size = args[i].b.size;

		u[i].type = SI_AT_OB;
	}

	FOR_ARGS(i, counts, OI) {
		u[i].o = args[i].o.context;

		/* No need to temporarily get objects; we trust the kernel clients. */

		u[i].type = SI_AT_IO;
	}

	FOR_ARGS(i, counts, OO)
		u[i].type = SI_AT_OO;
}

static void marshal_out(union ObjectArg args[], struct si_arg u[], uint32_t counts)
{
	int i;

	FOR_ARGS(i, counts, BI) {
		/* NOTHING TO DO FOR INPUT BUFFERS. */
	}

	FOR_ARGS(i, counts, BO) {
		args[i].b.size = u[i].b.size;
	}

	FOR_ARGS(i, counts, OI) {
		/* NOTHING TO DO FOR INPUT OBJECTS. */
	}

	FOR_ARGS(i, counts, OO) {
		args[i].o.context = u[i].o;
		args[i].o.invoke = do_invoke;
	}
}

/* Invocation function. */

static int32_t do_invoke(void *context, uint32_t op,
	union ObjectArg *args, uint32_t counts)
{
	int ret, result;

	struct si_object *object = context;

	struct si_arg *u;
	struct si_object_invoke_ctx *oic;

	if (ObjectOp_isLocal(op)) {
		switch (ObjectOp_methodID(op)) {
		case Object_OP_retain:
			get_si_object(object);

			return OBJECT_OK;
		case Object_OP_release:
			put_si_object(object);

			return OBJECT_OK;
		default:

			break;
		}

		return OBJECT_ERROR_REMOTE;
	}

	/* Allocate resources. */

	oic = kzalloc(sizeof(*oic), GFP_KERNEL);
	if (!oic)
		return -OBJECT_ERROR_KMEM;

	u = kcalloc(OBJECT_COUNTS_TOTAL(counts) + 1, sizeof(struct si_arg), GFP_KERNEL);
	if (!u) {
		ret = -OBJECT_ERROR_KMEM;

		goto out_failed;
	}

	pr_info("%s object invocation with %zu arguments (%04x) and op %d.\n",
		si_object_name(object), OBJECT_COUNTS_TOTAL(counts), counts, op);

	/* + INITIATE an invocation. */

	marshal_in(u, args, counts);

	ret = si_object_do_invoke(oic, object, op, u, &result);
	if (ret) {
		pr_err("si_object_do_invoke failed with %d.\n", ret);
		ret = OBJECT_ERROR_UNAVAIL;

		goto out_failed;
	}

	if (!result)
		marshal_out(args, u, counts);

	ret = result;

out_failed:

	pr_info("%s object invocation returned with %d.\n",	si_object_name(object), ret);

	kfree(u);
	kfree(oic);

	return ret;
}

int get_root_obj(struct Object *rootObj)
{
	rootObj->context = ROOT_SI_OBJECT;
	rootObj->invoke = do_invoke;

	return 0;
}

int32_t get_client_env_object(struct Object *clientEnvObj)
{
	int ret;
	struct Object rootObj;

	get_root_obj(&rootObj);

	ret = IClientEnv_registerWithCredentials(rootObj, Object_NULL, clientEnvObj);
	if (ret)
		pr_err("Failed to get ClientEnvObject (ret = %d).\n", ret);

	return ret;
}
EXPORT_SYMBOL_GPL(get_client_env_object);
