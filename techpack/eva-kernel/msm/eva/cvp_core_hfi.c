// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/slab.h>
#include "msm_cvp_debug.h"
#include "cvp_hfi_api.h"
#include "cvp_core_hfi.h"

struct cvp_hfi_ops *cvp_hfi_initialize(enum msm_cvp_hfi_type hfi_type,
		struct msm_cvp_platform_resources *res,
		hfi_cmd_response_callback callback)
{
	struct cvp_hfi_ops *ops_tbl = NULL;
	int rc = 0;

	ops_tbl = kzalloc(sizeof(struct cvp_hfi_ops), GFP_KERNEL);
	if (!ops_tbl) {
		dprintk(CVP_ERR, "%s: failed to allocate ops_tbl\n", __func__);
		return NULL;
	}

	rc = cvp_iris_hfi_initialize(ops_tbl, res, callback);

	if (rc) {
		if (rc != -EPROBE_DEFER)
			dprintk(CVP_ERR, "%s device init failed rc = %d",
				__func__, rc);
		goto err_hfi_init;
	}

	return ops_tbl;

err_hfi_init:
	kfree(ops_tbl);
	return ERR_PTR(rc);
}

void cvp_hfi_deinitialize(enum msm_cvp_hfi_type hfi_type,
			struct cvp_hfi_ops *ops_tbl)
{
	if (!ops_tbl) {
		dprintk(CVP_ERR, "%s invalid device %pK", __func__, ops_tbl);
		return;
	}

	cvp_iris_hfi_delete_device(ops_tbl->hfi_device_data);

	kfree(ops_tbl);
}

