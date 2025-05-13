// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#include "btfm_codec.h"
#include "btfm_codec_hw_interface.h"
#include "btfm_codec_interface.h"

int btfmcodec_register_hw_ep (struct hwep_data *ep_info)
{
	struct btfmcodec_data *btfmcodec;
	struct hwep_data *hwep_info;
	int ret = 0;

	btfmcodec = btfm_get_btfmcodec();
	mutex_lock(&btfmcodec->hwep_drv_lock);
	if (!btfmcodec) {
		BTFMCODEC_ERR("btfm codec driver it not initialized");
		ret = -EPERM;
		goto end;
	}

	if (ep_info->num_dai == 0) {
		BTFMCODEC_ERR("no active information provided by hw ep interface");
		ret = -EPERM;
		goto end;

	}

	hwep_info = btfmcodec->hwep_info;
	if (hwep_info) {
		BTFMCODEC_ERR("driver already holds hardware endpoint info");
		ret = -EPERM;
		goto end;
	}

	hwep_info = kzalloc(sizeof(struct hwep_data), GFP_KERNEL);
	if (!hwep_info) {
		BTFMCODEC_ERR("%s: failed to allocate memory\n", __func__);
		ret = -ENOMEM;
		goto end;
	}

	btfmcodec->hwep_info = hwep_info;
	memcpy(hwep_info, ep_info, sizeof(struct hwep_data));

	BTFMCODEC_INFO("Below driver registered with btfm codec\n");
	BTFMCODEC_INFO("Driver name: %s\n", hwep_info->driver_name);
	BTFMCODEC_INFO("Num of dai: %d supported", hwep_info->num_dai);
	BTFMCODEC_INFO("Master config enabled: %u\n", test_bit(BTADV_AUDIO_MASTER_CONFIG,
		&hwep_info->flags));
	ret = btfm_register_codec(hwep_info);
end:
	mutex_unlock(&btfmcodec->hwep_drv_lock);
	return ret;
}

int btfmcodec_unregister_hw_ep (char *driver_name)
{
	struct btfmcodec_data *btfmcodec;
	struct hwep_data *hwep_info;
	int ret;

	btfmcodec = btfm_get_btfmcodec();
	mutex_lock(&btfmcodec->hwep_drv_lock);
	if (!btfmcodec) {
		BTFMCODEC_ERR("btfm codec driver it not initialized");
		ret = -EPERM;
		goto end;
	}

	hwep_info = btfmcodec->hwep_info;
	if (!hwep_info) {
		BTFMCODEC_ERR("%s: no active hardware endpoint registered\n", __func__);
		ret = -EPERM;
		goto end;
	}

	if(!strncmp(hwep_info->driver_name, driver_name, DEVICE_NAME_MAX_LEN)) {
		btfm_unregister_codec();
		kfree(hwep_info);
		BTFMCODEC_INFO("%s: deleted %s hardware endpoint\n", __func__, driver_name);
		ret = -1;
		goto end;
	} else {
		BTFMCODEC_ERR("%s: No hardware endpoint registered with %s\n", __func__, driver_name);
		ret = -1;
		goto end;
	}
end:
	mutex_unlock(&btfmcodec->hwep_drv_lock);
	return ret;
}

EXPORT_SYMBOL(btfmcodec_register_hw_ep);
EXPORT_SYMBOL(btfmcodec_unregister_hw_ep);
