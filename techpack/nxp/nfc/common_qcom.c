// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 *
 ***************************************************************************/
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 ***************************************************************************/

#include "common.h"

/*
 * Inside nfc_ioctl_nfcc_info
 *
 * @brief   nfc_ioctl_nfcc_info
 *
 * Check the NFC Chipset and firmware version details
 */
unsigned int nfc_ioctl_nfcc_info(struct file *filp, unsigned long arg)
{
	unsigned int r = 0;
	struct nfc_dev *nfc_dev = filp->private_data;

	r = nfc_dev->nqx_info.i;
	pr_debug("nfc : %s r = 0x%x\n", __func__, r);

	return r;
}

/*
 * Inside is_nfc_data_available_for_read
 *
 * @nfc_dev: nfc device data structure
 *
 * Checks if the data is available for reading
 * on waiting queue.
 *
 * @Return: status value
 *
 */
int is_nfc_data_available_for_read(struct nfc_dev *nfc_dev)
{
	int ret;

	nfc_dev->nfc_enable_intr(nfc_dev);

	ret = wait_event_interruptible_timeout(nfc_dev->read_wq,
			!nfc_dev->i2c_dev.irq_enabled,
			msecs_to_jiffies(MAX_IRQ_WAIT_TIME));
	return ret;
}

/**
 * nfc_ldo_vote()
 * @nfc_dev: NFC device containing regulator handle
 *
 * LDO voting based on voltage and current entries in DT
 *
 * Return: 0 on success and -ve on failure
 */
int nfc_ldo_vote(struct nfc_dev *nfc_dev)
{
	int ret;

	ret =  regulator_set_voltage(nfc_dev->reg,
			nfc_dev->configs.ldo.vdd_levels[0],
			nfc_dev->configs.ldo.vdd_levels[1]);
	if (ret < 0) {
		pr_err("%s: set voltage failed\n", __func__);
		return ret;
	}

	/* pass expected current from NFC in uA */
	ret = regulator_set_load(nfc_dev->reg, nfc_dev->configs.ldo.max_current);
	if (ret < 0) {
		pr_err("%s: set load failed\n", __func__);
		return ret;
	}

	ret = regulator_enable(nfc_dev->reg);
	if (ret < 0)
		pr_err("%s: regulator_enable failed\n", __func__);
	else
		nfc_dev->is_vreg_enabled = true;
	return ret;
}

/**
 * nfc_ldo_config()
 * @dev: device instance to read DT entry
 * @nfc_dev: NFC device containing regulator handle
 *
 * Configure LDO if entry is present in DT file otherwise
 * return with success as it's optional
 *
 * Return: 0 on success and -ve on failure
 */
int nfc_ldo_config(struct device *dev, struct nfc_dev *nfc_dev)
{
	int ret;

	if (of_get_property(dev->of_node, NFC_LDO_SUPPLY_NAME, NULL)) {
		// Get the regulator handle
		nfc_dev->reg = regulator_get(dev, NFC_LDO_SUPPLY_DT_NAME);
		if (IS_ERR(nfc_dev->reg)) {
			ret = PTR_ERR(nfc_dev->reg);
			nfc_dev->reg = NULL;
			pr_err("%s: regulator_get failed, ret = %d\n",
				__func__, ret);
			return ret;
		}
	} else {
		nfc_dev->reg = NULL;
		pr_err("%s: regulator entry not present\n", __func__);
		// return success as it's optional to configure LDO
		return 0;
	}

	// LDO config supported by platform DT
	ret = nfc_ldo_vote(nfc_dev);
	if (ret < 0) {
		pr_err("%s: LDO voting failed, ret = %d\n", __func__, ret);
		regulator_put(nfc_dev->reg);
	}
	return ret;
}

/**
 * nfc_ldo_unvote()
 * @nfc_dev: NFC device containing regulator handle
 *
 * set voltage and load to zero and disable regulator
 *
 * Return: 0 on success and -ve on failure
 */
int nfc_ldo_unvote(struct nfc_dev *nfc_dev)
{
	int ret;

	if (!nfc_dev->is_vreg_enabled) {
		pr_err("%s: regulator already disabled\n", __func__);
		return -EINVAL;
	}

	ret = regulator_disable(nfc_dev->reg);
	if (ret < 0) {
		pr_err("%s: regulator_disable failed\n", __func__);
		return ret;
	}
	nfc_dev->is_vreg_enabled = false;

	ret =  regulator_set_voltage(nfc_dev->reg, 0, NFC_VDDIO_MAX);
	if (ret < 0) {
		pr_err("%s: set voltage failed\n", __func__);
		return ret;
	}

	ret = regulator_set_load(nfc_dev->reg, 0);
	if (ret < 0)
		pr_err("%s: set load failed\n", __func__);
	return ret;
}

/*
 * Routine to enable clock.
 * this routine can be extended to select from multiple
 * sources based on clk name.
 */
int nfc_clock_select(struct nfc_dev *nfc_dev)
{
	int r = 0;

	nfc_dev->s_clk = clk_get(&nfc_dev->i2c_dev.client->dev, "nfc_ref_clk");

	if (IS_ERR(nfc_dev->s_clk))
		return PTR_ERR(nfc_dev->s_clk);

	if (!nfc_dev->clk_run)
		r = clk_prepare_enable(nfc_dev->s_clk);

	if (r)
		return r;

	nfc_dev->clk_run = true;
	return r;
}

/*
 * Routine to disable clocks
 */
int nfc_clock_deselect(struct nfc_dev *nfc_dev)
{
	int r = -EINVAL;

	if (nfc_dev->s_clk != NULL) {
		if (nfc_dev->clk_run) {
			clk_disable_unprepare(nfc_dev->s_clk);
			nfc_dev->clk_run = false;
		}
		return 0;
	}
	return r;
}
