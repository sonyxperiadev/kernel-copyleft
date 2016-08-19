/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
/*
 * Copyright (C) 2014 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

/*
 * qpnp-smbcharger.c macros
 */
#define IDEV_STS	0x8

#define USB_DP		67
#define USB_DM		68


/*
 * qpnp-smbcharger_extension_usb.c macros
 */
#define DEFAULT_PROP1000_MA		1000
#define DEFAULT_PROP500_MA		500
#define THR_DP_DM_FLOAT_UV		3030000


/*
 * qpnp-smbcharger.c functions
 */
static int smbchg_read(struct smbchg_chip *chip, u8 *val, u16 addr, int count);
static int get_type(u8 type_reg);


/*
 * qpnp-smbcharger_extension_usb.c functions
 */
static int get_usb_type(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->misc_base + IDEV_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read status 5 rc = %d\n", rc);
		return rc;
	}
	return get_type(reg);
}

static int get_prop_usb_dp_voltage_now(struct smbchg_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if (IS_ERR_OR_NULL(chip->usb_params.vadc_usb_dp)) {
		chip->usb_params.vadc_usb_dp =
					qpnp_get_vadc(chip->dev, "usb_dp");
		if (IS_ERR(chip->usb_params.vadc_usb_dp))
			return PTR_ERR(chip->usb_params.vadc_usb_dp);
	}

	rc = qpnp_vadc_read(chip->usb_params.vadc_usb_dp, USB_DP, &results);
	if (rc) {
		pr_err("Unable to read usb_dp rc=%d\n", rc);
		return 0;
	} else {
		return results.physical;
	}
}

static int get_prop_usb_dm_voltage_now(struct smbchg_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if (IS_ERR_OR_NULL(chip->usb_params.vadc_usb_dm)) {
		chip->usb_params.vadc_usb_dm =
					qpnp_get_vadc(chip->dev, "usb_dm");
		if (IS_ERR(chip->usb_params.vadc_usb_dm))
			return PTR_ERR(chip->usb_params.vadc_usb_dm);
	}

	rc = qpnp_vadc_read(chip->usb_params.vadc_usb_dm, USB_DM, &results);
	if (rc) {
		pr_err("Unable to read usb_dm rc=%d\n", rc);
		return 0;
	} else {
		return results.physical;
	}
}

static int get_prop_proprietary_charger(struct smbchg_chip *chip)
{
	#define IN_RANGE(val, hi, lo) ((hi >= val) && (val >= lo))
	struct {
		struct {
			int hi;
			int lo;
		} dp, dm;
		int type;
		char *name;
	} chgs[] = {
		{{3600000, 3000000}, {3600000, 3000000},
				POWER_SUPPLY_SUB_TYPE_PROPRIETARY,
				"Sony"},
		{{3000000, 2400000}, {3000000, 2400000},
				POWER_SUPPLY_SUB_TYPE_PROPRIETARY,
				"Proprietary 12w"},
		{{3000000, 2400000}, {2300000, 1700000},
				POWER_SUPPLY_SUB_TYPE_PROPRIETARY,
				"Proprietary 10w"},
		{{2300000, 1700000}, {3000000, 2400000},
				POWER_SUPPLY_SUB_TYPE_PROPRIETARY_1000MA,
				"Proprietary 5w"},
		{{2300000, 1700000}, {2300000, 1700000},
				POWER_SUPPLY_SUB_TYPE_PROPRIETARY_500MA,
				"Proprietary 2.5w"},
	};
	int dp, dm;
	int ret;
	int i;

	dp = get_prop_usb_dp_voltage_now(chip);
	if (IS_ERR_VALUE(dp)) {
		dev_err(chip->dev, "%s: read D+ voltage fail\n", __func__);
		return dp;
	}

	dm = get_prop_usb_dm_voltage_now(chip);
	if (IS_ERR_VALUE(dm)) {
		dev_err(chip->dev, "%s: read D- voltage fail\n", __func__);
		return dm;
	}

	ret = POWER_SUPPLY_SUB_TYPE_UNKNOWN;
	for (i = 0; i < (sizeof(chgs) / sizeof(chgs[0])); i++) {
		if (IN_RANGE(dp, chgs[i].dp.hi, chgs[i].dp.lo) &&
				IN_RANGE(dm, chgs[i].dm.hi, chgs[i].dm.lo)) {
			pr_smb(PR_MISC, "%s: %s charger, D+=%d, D-=%d\n",
					__func__, chgs[i].name, dp, dm);
			ret = chgs[i].type;
			break;
		}
	}

	if (ret == POWER_SUPPLY_SUB_TYPE_UNKNOWN) {
		dev_warn(chip->dev,
				"%s: voltage not in range, D+=%d, D-=%d\n",
							__func__, dp, dm);
		ret = -ERANGE;
	}

	return ret;
}

static int is_floated_charger(struct smbchg_chip *chip)
{
	union power_supply_propval dp_dm = {0, };
	int dp = 0;
	int dm = 0;
	int ret;

	ret = chip->usb_psy->get_property(chip->usb_psy,
					POWER_SUPPLY_PROP_DP_DM, &dp_dm);
	if (IS_ERR_VALUE(ret)) {
		dev_err(chip->dev, "%s: read D+/D- state fail\n", __func__);
		return ret;
	}

	ret = power_supply_set_dp_dm(chip->usb_psy,
					POWER_SUPPLY_DP_DM_DP3P3_DM3P3);
	if (IS_ERR_VALUE(ret)) {
		dev_err(chip->dev, "%s: set D+/D- 3.3V fail\n", __func__);
		return ret;
	}

	msleep(100);

	ret = get_prop_usb_dp_voltage_now(chip);
	if (IS_ERR_VALUE(ret)) {
		dev_err(chip->dev, "%s: read D+ voltage fail\n", __func__);
		goto out;
	}
	dp = ret;

	ret = get_prop_usb_dm_voltage_now(chip);
	if (IS_ERR_VALUE(ret)) {
		dev_err(chip->dev, "%s: read D- voltage fail\n", __func__);
		goto out;
	}
	dm = ret;
	ret = (dp > THR_DP_DM_FLOAT_UV || dm > THR_DP_DM_FLOAT_UV);
out:
	power_supply_set_dp_dm(chip->usb_psy, dp_dm.intval);
	pr_smb(PR_MISC, "%s: is_float=%d, D+=%dV, D-=%dV\n", __func__, ret,
									dp, dm);
	return ret;
}

/*
 * USB OCP part
 */
static const char *rdev_get_name(struct regulator_dev *rdev)
{
	if (rdev->constraints && rdev->constraints->name)
		return rdev->constraints->name;
	else if (rdev->desc->name)
		return rdev->desc->name;
	else
		return "";
}

static int somc_usb_otg_regulator_register_ocp_notification(
				struct regulator_dev *rdev,
				struct regulator_ocp_notification *notification)
{
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);
	struct somc_usb_ocp *ocp = &chip->usb_params.ocp;
	unsigned long flags;

	spin_lock_irqsave(&ocp->lock, flags);
	if (notification)
		/* register ocp notification */
		ocp->notification = *notification;
	else
		/* unregister ocp notification */
		memset(&ocp->notification, 0, sizeof(ocp->notification));
	spin_unlock_irqrestore(&ocp->lock, flags);

	pr_smb(PR_MISC, "%s: registered ocp notification(notify=%p, ctxt=%p)\n",
						rdev_get_name(rdev),
						ocp->notification.notify,
						ocp->notification.ctxt);

	return 0;
}

static int somc_usb_otg_regulator_ocp_notify(struct smbchg_chip *chip)
{
	struct somc_usb_ocp *ocp = &chip->usb_params.ocp;
	unsigned long flags;

	spin_lock_irqsave(&ocp->lock, flags);
	if (ocp->notification.notify)
		ocp->notification.notify(ocp->notification.ctxt);
	spin_unlock_irqrestore(&ocp->lock, flags);

	return 0;
}

static int somc_usb_register(struct smbchg_chip *chip)
{
	struct usb_somc_params *params = &chip->usb_params;
	struct somc_usb_ocp *ocp = &params->ocp;

	memset(&ocp->notification, 0, sizeof(ocp->notification));
	spin_lock_init(&ocp->lock);

	pr_smb(PR_MISC, "somc usb register success\n");
	return 0;
}

static void somc_usb_unregister(struct smbchg_chip *chip)
{
	;
}
