// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/*
 * Bluetooth Power Switch Module
 * controls power to external Bluetooth device
 * with interface to power management device
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <linux/skbuff.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/of_device.h>
#include <soc/qcom/cmd-db.h>
#include <linux/kdev_t.h>
#include <linux/refcount.h>
#include <linux/idr.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/pinctrl/qcom-pinctrl.h>
#include <linux/soc/qcom/qcom_aoss.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/devinfo.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinctrl.h>

#include "btpower.h"
#ifdef CONFIG_FMD_ENABLE
#include "cnss_utils.h"
#endif
#if (defined CONFIG_BT_SLIM)
#include "btfm_slim.h"
#endif
#include <linux/fs.h>
#include <linux/nvmem-consumer.h>

#ifdef CONFIG_BT_HW_SECURE_DISABLE
#include "linux/smcinvoke_object.h"
#include "linux/IClientEnv.h"

#define PERISEC_HW_STATE_UID 0x108
#define PERISEC_HW_OP_GET_STATE 1
#define PERISEC_HW_BLUETOOTH_UID 0x502
#define PERISEC_FEATURE_NOT_SUPPORTED 12
#define PERISEC_PERIPHERAL_NOT_FOUND 10
#endif

#define PWR_SRC_NOT_AVAILABLE -2
#define DEFAULT_INVALID_VALUE -1
#define PWR_SRC_INIT_STATE_IDX 0
#define BTPOWER_MBOX_MSG_MAX_LEN 64
#define BTPOWER_MBOX_TIMEOUT_MS 1000
#define XO_CLK_RETRY_COUNT_MAX 5
#define MAX_PROP_SIZE 32
#define BTPOWER_CONFIG_MAX_TIMEOUT 600

#define SIGIO_OOBS_SINGAL         0x00010000
#define SIGIO_NOTIFICATION_SIGNAL 0x00020000
#define SIGIO_SOC_ACCESS_SIGNAL   0x00040000

#define SIGIO_GPIO_HIGH           0x00000001
#define SIGIO_GPIO_LOW            0x00000000
#define SIGIO_SSR_ON_UWB          0x00000001
#define SIGIO_UWB_SSR_COMPLETED   0x00000002

#define RESETB_GPIO_HIGH         0x00000001
#define RESETB_GPIO_LOW          0x00000000

#define CRASH_REASON_NOT_FOUND  ((char *)"Crash reason not found")

#define PERI_SS	(0x00)
#define BT_SS	(0x01)
#define UWB_SS	(0x02)
#define TME_SS	(0x03)

#define INVALID_SOC                 0x00
#define PEACH_SOC_VERSION_1_0       0x01
#define PEACH_SOC_VERSION_2_0       0x02
#define OTHER_FMD_SUPPORTED_BT_SOC  0x03

/**
 * enum btpower_vreg_param: Voltage regulator TCS param
 * @BTPOWER_VREG_VOLTAGE: Provides voltage level to be configured in TCS
 * @BTPOWER_VREG_MODE: Regulator mode
 * @BTPOWER_VREG_TCS_ENABLE: Set Voltage regulator enable config in TCS
 */
enum btpower_vreg_param {
	BTPOWER_VREG_VOLTAGE,
	BTPOWER_VREG_MODE,
	BTPOWER_VREG_ENABLE,
};

/**
 * enum btpower_tcs_seq: TCS sequence ID for trigger
 * BTPOWER_TCS_UP_SEQ: TCS Sequence based on up trigger / Wake TCS
 * BTPOWER_TCS_DOWN_SEQ: TCS Sequence based on down trigger / Sleep TCS
 * BTPOWER_TCS_ALL_SEQ: Update for both up and down triggers
 */
enum btpower_tcs_seq {
	BTPOWER_TCS_UP_SEQ,
	BTPOWER_TCS_DOWN_SEQ,
	BTPOWER_TCS_ALL_SEQ,
};

enum power_src_pos {
	BT_RESET_GPIO = PWR_SRC_INIT_STATE_IDX,
	BT_SW_CTRL_GPIO,
	BT_VDD_AON_LDO,
	BT_VDD_DIG_LDO,
	BT_VDD_RFA1_LDO,
	BT_VDD_RFA2_LDO,
	BT_VDD_ASD_LDO,
	BT_VDD_XTAL_LDO,
	BT_VDD_PA_LDO,
	BT_VDD_CORE_LDO,
	BT_VDD_IO_LDO,
	BT_VDD_LDO,
	BT_VDD_RFA_0p8,
	BT_VDD_RFACMN,
	BT_VDD_ANT_LDO,
	BT_VDD_WLAN_AON_LDO,

	// these indexes GPIOs/regs value are fetched during crash.
	BT_RESET_GPIO_CURRENT,
	BT_SW_CTRL_GPIO_CURRENT,
	BT_VDD_AON_LDO_CURRENT,
	BT_VDD_DIG_LDO_CURRENT,
	BT_VDD_RFA1_LDO_CURRENT,
	BT_VDD_RFA2_LDO_CURRENT,
	BT_VDD_ASD_LDO_CURRENT,
	BT_VDD_XTAL_LDO_CURRENT,
	BT_VDD_PA_LDO_CURRENT,
	BT_VDD_CORE_LDO_CURRENT,
	BT_VDD_IO_LDO_CURRENT,
	BT_VDD_LDO_CURRENT,
	BT_VDD_RFA_0p8_CURRENT,
	BT_VDD_RFACMN_CURRENT,
	BT_VDD_IPA_2p2,
	BT_VDD_IPA_2p2_CURRENT,
	BT_VDD_ANT_LDO_CURRENT,
	BT_VDD_WLAN_AON_LDO_CURRENT,

	/* The below bucks are voted for HW WAR on some platform which supports
	 * WNC39xx.
	 */
	BT_VDD_SMPS,
	BT_VDD_SMPS_CURRENT,
	/* New entries need to be added before PWR_SRC_SIZE.
	 * Its hold the max size of power sources states.
	 */
	BT_POWER_SRC_SIZE,
};

// Regulator structure for QCA6174/QCA9377/QCA9379 BT SoC series
static struct vreg_data bt_vregs_info_qca61x4_937x[] = {
	{NULL, "qcom,bt-vdd-aon", 928000, 928000, 0, false, false,
		{BT_VDD_AON_LDO, BT_VDD_AON_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd-io", 1710000, 3460000, 0, false, false,
		{BT_VDD_IO_LDO, BT_VDD_IO_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd-core", 3135000, 3465000, 0, false, false,
		{BT_VDD_CORE_LDO, BT_VDD_CORE_LDO_CURRENT}},
};

// Regulator structure for QCA6390,QCA6490 and WCN6750 BT SoC series
static struct vreg_data bt_vregs_info_qca6xx0[] = {
	{NULL, "qcom,bt-vdd-io",      1800000, 1800000, 0, false, true,
		{BT_VDD_IO_LDO, BT_VDD_IO_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd-aon",     966000,  966000,  0, false, true,
		{BT_VDD_AON_LDO, BT_VDD_AON_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd-rfacmn",  950000,  950000,  0, false, true,
		{BT_VDD_RFACMN, BT_VDD_RFACMN_CURRENT}},
	/* BT_CX_MX */
	{NULL, "qcom,bt-vdd-dig",      966000,  966000,  0, false, true,
		{BT_VDD_DIG_LDO, BT_VDD_DIG_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd-rfa-0p8",  950000,  952000,  0, false, true,
		{BT_VDD_RFA_0p8, BT_VDD_RFA_0p8_CURRENT}},
	{NULL, "qcom,bt-vdd-rfa1",     1900000, 1900000, 0, false, true,
		{BT_VDD_RFA1_LDO, BT_VDD_RFA1_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd-rfa2",     1900000, 1900000, 0, false, true,
		{BT_VDD_RFA2_LDO, BT_VDD_RFA2_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd-asd",      2800000, 2800000, 0, false, true,
		{BT_VDD_ASD_LDO, BT_VDD_ASD_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd-ipa-2p2",  2200000, 2210000, 0, false, true,
		{BT_VDD_IPA_2p2, BT_VDD_IPA_2p2_CURRENT}},
};

// Regulator structure for kiwi BT SoC series
static struct vreg_data bt_vregs_info_kiwi[] = {
	{NULL, "qcom,bt-vdd18-aon",      1800000, 1800000, 0, false, true,
		{BT_VDD_LDO, BT_VDD_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd12-io",      1200000, 1200000, 0, false, true,
		{BT_VDD_IO_LDO, BT_VDD_IO_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd-aon",     950000,  950000,  0, false, true,
		 {BT_VDD_AON_LDO, BT_VDD_AON_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd-rfaOp8",  950000,  950000,  0, false, true,
		{BT_VDD_RFACMN, BT_VDD_RFACMN_CURRENT}},
	/* BT_CX_MX */
	{NULL, "qcom,bt-vdd-dig",      950000,  950000,  0, false, true,
		{BT_VDD_DIG_LDO, BT_VDD_DIG_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd-rfaOp8",  950000,  952000,  0, false, true,
		{BT_VDD_RFA_0p8, BT_VDD_RFA_0p8_CURRENT}},
	{NULL, "qcom,bt-vdd-rfa1",     1350000, 1350000, 0, false, true,
		{BT_VDD_RFA1_LDO, BT_VDD_RFA1_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd-rfa2",     1900000, 1900000, 0, false, true,
		{BT_VDD_RFA2_LDO, BT_VDD_RFA2_LDO_CURRENT}},
	{NULL, "qcom,bt-ant-ldo",  1776000, 1776000, 0, false, true,
		{BT_VDD_ANT_LDO, BT_VDD_ANT_LDO_CURRENT}},
};

static struct vreg_data platform_vregs_info_peach[] = {
	/* VDD1P8_AON */
	{NULL, "qcom,bt-vdd18-aon",      1620000, 1980000, 0, false, true,
		{BT_VDD_LDO, BT_VDD_LDO_CURRENT}},
	/* VDD1P2_IOAV91C_VDD Extractor */
	{NULL, "qcom,bt-vdd12-io",      1080000, 1980000, 0, false, true,
		{BT_VDD_IO_LDO, BT_VDD_IO_LDO_CURRENT}},
	/* AV91C_VDD Extractor */
	{NULL, "qcom,bt-ant-ldo",  1776000, 1776000, 0, false, true,
		{BT_VDD_ANT_LDO, BT_VDD_ANT_LDO_CURRENT}},
	/* BT_CX_MX */
	{NULL, "qcom,bt-vdd-dig",      700000,  2100000,  0, false, true,
		{BT_VDD_DIG_LDO, BT_VDD_DIG_LDO_CURRENT}},
	/* RFA_CMN/AON */
	{NULL, "qcom,bt-vdd-aon",     800000,  2100000,  0, false, true,
		{BT_VDD_AON_LDO, BT_VDD_AON_LDO_CURRENT}},
	/* RFA_OP75 */
	{NULL, "qcom,bt-vdd-rfa0p75",  800000,  2100000,  0, false, true,
		{BT_VDD_RFA_0p8, BT_VDD_RFA_0p8_CURRENT}},
	/* RFA_1P8 */
	{NULL, "qcom,bt-vdd-rfa1p8",     1850000, 2100000, 0, false, true,
		{BT_VDD_RFA2_LDO, BT_VDD_RFA2_LDO_CURRENT}},
	/* RFA_1P25 */
	{NULL, "qcom,bt-vdd-rfa1p25",     1300000, 2100000, 0, false, true,
		{BT_VDD_RFA1_LDO, BT_VDD_RFA1_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd-wlan-aon", 950000, 950000, 0, false, false,
		{BT_VDD_WLAN_AON_LDO, BT_VDD_WLAN_AON_LDO_CURRENT}},
};

// Regulator structure for WCN399x BT SoC series
static struct pwr_data vreg_info_wcn399x = {
	.compatible = "qcom,wcn3990",
	.bt_vregs = (struct vreg_data []) {
		{NULL, "qcom,bt-vdd-smps", 984000,  984000, 0, false, false,
			{BT_VDD_SMPS, BT_VDD_SMPS_CURRENT}},
		{NULL, "qcom,bt-vdd-io",   1700000, 1900000, 0, false, false,
			{BT_VDD_IO_LDO, BT_VDD_IO_LDO_CURRENT}},
		{NULL, "qcom,bt-vdd-core", 1304000, 1304000, 0, false, false,
			{BT_VDD_CORE_LDO, BT_VDD_CORE_LDO_CURRENT}},
		{NULL, "qcom,bt-vdd-pa",   3000000, 3312000, 0, false, false,
			{BT_VDD_PA_LDO, BT_VDD_PA_LDO_CURRENT}},
		{NULL, "qcom,bt-vdd-xtal", 1700000, 1900000, 0, false, false,
			{BT_VDD_XTAL_LDO, BT_VDD_XTAL_LDO_CURRENT}},
	},
	.bt_num_vregs = 5,
};

static struct pwr_data vreg_info_qca6174 = {
	.compatible = "qcom,qca6174",
	.bt_vregs = bt_vregs_info_qca61x4_937x,
	.bt_num_vregs = ARRAY_SIZE(bt_vregs_info_qca61x4_937x),
};

static struct pwr_data vreg_info_qca6390 = {
	.compatible = "qcom,qca6390",
	.bt_vregs = bt_vregs_info_qca6xx0,
	.bt_num_vregs = ARRAY_SIZE(bt_vregs_info_qca6xx0),
};

static struct pwr_data vreg_info_qca6490 = {
	.compatible = "qcom,qca6490",
	.bt_vregs = bt_vregs_info_qca6xx0,
	.bt_num_vregs = ARRAY_SIZE(bt_vregs_info_qca6xx0),
};

static struct pwr_data vreg_info_kiwi = {
	.compatible = "qcom,kiwi",
	.bt_vregs = bt_vregs_info_kiwi,
	.bt_num_vregs = ARRAY_SIZE(bt_vregs_info_kiwi),
};

static struct pwr_data vreg_info_kiwi_no_share_ant_power = {
	.compatible = "qcom,kiwi-no-share-ant-power",
	.bt_vregs = bt_vregs_info_kiwi,
	.bt_num_vregs = ARRAY_SIZE(bt_vregs_info_kiwi),
};

static struct pwr_data vreg_info_converged = {
	.compatible = "qcom,bt-qca-converged",
	.bt_vregs = bt_vregs_info_kiwi,
	.bt_num_vregs = ARRAY_SIZE(bt_vregs_info_kiwi),
};

static struct pwr_data vreg_info_wcn6750 = {
	.compatible = "qcom,wcn6750-bt",
	.bt_vregs = bt_vregs_info_qca6xx0,
	.bt_num_vregs = ARRAY_SIZE(bt_vregs_info_qca6xx0),
};

/* Peach supports both BT & UWB SS. For now it requires
 * only platform regulators to be powered ON.
 */
static struct pwr_data vreg_info_peach = {
	.compatible = "qcom,peach-bt",
	.platform_vregs = platform_vregs_info_peach,
	.platform_num_vregs = ARRAY_SIZE(platform_vregs_info_peach),
};

static struct pwr_data vreg_info_wcn786x = {
	.compatible = "qcom,wcn786x",
	.platform_vregs = platform_vregs_info_peach,
	.platform_num_vregs = ARRAY_SIZE(platform_vregs_info_peach),
};

static struct pwr_data bt_vreg_info_wcn7750 = {
	.compatible = "qcom,wcn7750-bt",
	.bt_vregs = bt_vregs_info_qca6xx0,
	.bt_num_vregs = ARRAY_SIZE(bt_vregs_info_qca6xx0),
};

static const struct of_device_id bt_power_match_table[] = {
	{	.compatible = "qcom,qca6174", .data = &vreg_info_qca6174},
	{	.compatible = "qcom,wcn3990", .data = &vreg_info_wcn399x},
	{	.compatible = "qcom,qca6390", .data = &vreg_info_qca6390},
	{	.compatible = "qcom,qca6490", .data = &vreg_info_qca6490},
	{	.compatible = "qcom,kiwi",    .data = &vreg_info_kiwi},
	{	.compatible = "qcom,kiwi-no-share-ant-power",
			.data = &vreg_info_kiwi_no_share_ant_power},
	{	.compatible = "qcom,wcn6750-bt", .data = &vreg_info_wcn6750},
	{	.compatible = "qcom,bt-qca-converged", .data = &vreg_info_converged},
	{	.compatible = "qcom,peach-bt", .data = &vreg_info_peach},
	{	.compatible = "qcom,wcn786x", .data = &vreg_info_wcn786x},
	{	.compatible = "qcom,wcn7750-bt", .data = &bt_vreg_info_wcn7750},
	{},
};

static struct platform_pwr_data *pwr_data;
static bool previous;
static struct class *bt_class;
static int bt_major;
static int soc_id;
static bool probe_finished;
static struct fmdOperationStruct fmdStruct;
char *default_crash_reason = "Crash reason not found";

static int btpower_enable_ipa_vreg(struct platform_pwr_data *pdata);
static inline int btpower_get_retenion_mode_state(void);
static void bt_power_vote(struct work_struct *work);

static struct {
	int platform_state[BT_POWER_SRC_SIZE];
	int bt_state[BT_POWER_SRC_SIZE];
	int uwb_state[BT_POWER_SRC_SIZE];
} power_src;

struct Crash_struct {
//	char SubSystem[10];
	char PrimaryReason[50];
	char SecondaryReason[100];
} CrashInfo;

#ifdef CONFIG_BT_HW_SECURE_DISABLE
int perisec_cnss_bt_hw_disable_check(struct platform_pwr_data *plat_priv)
{
	struct Object client_env;
	struct Object app_object;
	int bt_uid = PERISEC_HW_BLUETOOTH_UID;
	union ObjectArg obj_arg[2] = {{{0, 0}}};
	int ret;
	u8 state = 0;

	/* Once this flag is set, secure peripheral feature
	 * will not be supported till next reboot
	 */
	if (plat_priv->sec_peri_feature_disable)
		return 0;

	/* get rootObj */
	ret = get_client_env_object(&client_env);
	if (ret) {
		pr_err("Failed to get client_env_object, ret: %d\n", ret);
		goto end;
	}
	ret = IClientEnv_open(client_env, PERISEC_HW_STATE_UID, &app_object);
	if (ret) {
		pr_err("Failed to get app_object, ret: %d\n",  ret);
		if (ret == PERISEC_FEATURE_NOT_SUPPORTED) {
			ret = 0; /* Do not Assert */
			plat_priv->sec_peri_feature_disable = true;
			pr_debug("Secure HW feature not supported\n");
		}
		goto exit_release_clientenv;
	}

	obj_arg[0].b = (struct ObjectBuf) {&bt_uid, sizeof(u32)};
	obj_arg[1].b = (struct ObjectBuf) {&state, sizeof(u8)};
	ret = Object_invoke(app_object, PERISEC_HW_OP_GET_STATE, obj_arg,
			    ObjectCounts_pack(1, 1, 0, 0));

	pr_info("SMC invoke ret: %d state: %d\n", ret, state);
	if (ret) {
		if (ret == PERISEC_PERIPHERAL_NOT_FOUND) {
			ret = 0; /* Do not Assert */
			plat_priv->sec_peri_feature_disable = true;
			pr_info("Secure HW mode is not updated. Peripheral not found\n");
		}
	} else {
		if (state == 1)
			plat_priv->bt_sec_hw_disable = 1;
		else
			plat_priv->bt_sec_hw_disable = 0;
	}
	Object_release(app_object);

exit_release_clientenv:
	Object_release(client_env);
end:
	if (ret) {
		pr_err("SecMode:Unable to get sec mode BT Hardware status\n");
	}
	return ret;
}
#else
int perisec_cnss_bt_hw_disable_check(struct platform_pwr_data *plat_priv)
{
	return 0;
}
#endif


#ifdef CONFIG_MSM_BT_OOBS
static void btpower_uart_transport_locked(struct platform_pwr_data *drvdata,
					  bool locked)
{
	pr_debug("%s: %s\n", __func__, (locked ? "busy" : "idle"));
}

static irqreturn_t btpower_host_wake_isr(int irq, void *data)
{
	struct platform_pwr_data *drvdata = data;
	struct kernel_siginfo siginfo;
	int host_waking = (SIGIO_OOBS_SINGAL |
			   gpio_get_value(drvdata->bt_gpio_host_wake));
	int rc = 0;

	pr_debug("%s: bt-hostwake-gpio(%d) IRQ(%d) value(%d)\n", __func__,
		drvdata->bt_gpio_host_wake, drvdata->irq, host_waking);

	if (drvdata->reftask_bt == NULL) {
		pr_info("%s: ignore BT-HOSTWAKE IRQ\n", __func__);
		return IRQ_HANDLED;
	}

	// Sending signal to HAL layer
	memset(&siginfo, 0, sizeof(siginfo));
	siginfo.si_signo = SIGIO;
	siginfo.si_code = SI_QUEUE;
	siginfo.si_int = host_waking;
	rc = send_sig_info(siginfo.si_signo, &siginfo, drvdata->reftask_bt);
	if (rc < 0) {
		pr_err("%s: failed (%d) to send SIG to HAL(%d)\n", __func__,
			rc, drvdata->reftask_bt->pid);
	}
	return IRQ_HANDLED;
}
#endif

static int vreg_configure(struct vreg_data *vreg, bool retention)
{
	int rc = 0;

	if ((vreg->min_vol != 0) && (vreg->max_vol != 0)) {
		rc = regulator_set_voltage(vreg->reg,
					   (retention == false ? vreg->min_vol: 0),
					   vreg->max_vol);
		if (rc < 0) {
			pr_err("%s: regulator_set_voltage(%s) failed rc=%d\n",
				__func__, vreg->name, rc);
			return rc;
		}
	}

	if (vreg->load_curr >= 0) {
		rc = regulator_set_load(vreg->reg,
				(retention == false ? vreg->load_curr: 0));
		if (rc < 0) {
			pr_err("%s: regulator_set_load(%s) failed rc=%d\n",
			__func__, vreg->name, rc);
			return rc;
		}
	}

	return rc;
}

static int vreg_enable(struct vreg_data *vreg)
{
	int rc = 0;

	pr_debug("%s: vreg_en for : %s\n", __func__, vreg->name);

	if (!vreg->is_enabled) {
		if (vreg_configure(vreg, false) < 0) {
			return rc;
		}
		rc = regulator_enable(vreg->reg);
		if (rc < 0) {
			pr_err("%s: regulator_enable(%s) failed. rc=%d\n",
					__func__, vreg->name, rc);
			return rc;
		}
		vreg->is_enabled = true;
	}

	return rc;
}

static int vreg_disable_retention(struct vreg_data *vreg)
{
	int rc = 0;

	if (!vreg)
		return rc;

	pr_debug("%s: disable_retention for : %s\n", __func__, vreg->name);

	if ((vreg->is_enabled) && (vreg->is_retention_supp))
		rc = vreg_configure(vreg, false);

	return rc;
}

static int vreg_enable_retention(struct vreg_data *vreg)
{
	int rc = 0;

	if (!vreg)
		return rc;

	pr_debug("%s: enable_retention for : %s\n", __func__, vreg->name);

	if ((vreg->is_enabled) && (vreg->is_retention_supp))
		if ((vreg->min_vol != 0) && (vreg->max_vol != 0))
			rc = vreg_configure(vreg, true);

	return rc;
}

static int vreg_disable(struct vreg_data *vreg)
{
	int rc = 0;

	if (!vreg)
		return rc;

	pr_debug("%s for : %s\n", __func__, vreg->name);

	if (vreg->is_enabled) {
		rc = regulator_disable(vreg->reg);
		if (rc < 0) {
			pr_err("%s, regulator_disable(%s) failed. rc=%d\n",
					__func__, vreg->name, rc);
			goto out;
		}
		vreg->is_enabled = false;

		if ((vreg->min_vol != 0) && (vreg->max_vol != 0)) {
			/* Set the min voltage to 0 */
			rc = regulator_set_voltage(vreg->reg, 0,
					vreg->max_vol);
			if (rc < 0) {
				pr_err("%s: regulator_set_voltage(%s) failed rc=%d\n",
				__func__, vreg->name, rc);
				goto out;
			}
		}
		if (vreg->load_curr >= 0) {
			rc = regulator_set_load(vreg->reg, 0);
			if (rc < 0) {
				pr_err("%s: regulator_set_load(%s) failed rc=%d\n",
				__func__, vreg->name, rc);
			}
		}
	}
out:
	return rc;
}

static int bt_clk_enable(struct bt_power_clk_data *clk)
{
	int rc = 0;

	pr_info("%s: %s\n", __func__, clk->name);

	/* Get the clock handle for vreg */
	if (!clk->clk || clk->is_enabled) {
		pr_err("%s: error - node: %p, clk->is_enabled:%d\n",
			__func__, clk->clk, clk->is_enabled);
		return -EINVAL;
	}

	rc = clk_prepare_enable(clk->clk);
	if (rc) {
		pr_err("%s: failed to enable %s, rc(%d)\n",
				__func__, clk->name, rc);
		return rc;
	}

	clk->is_enabled = true;
	return rc;
}

static int bt_clk_disable(struct bt_power_clk_data *clk)
{
	int rc = 0;

	pr_debug("%s: %s\n", __func__, clk->name);

	/* Get the clock handle for vreg */
	if (!clk->clk || !clk->is_enabled) {
		pr_err("%s: error - node: %p, clk->is_enabled:%d\n",
			__func__, clk->clk, clk->is_enabled);
		return -EINVAL;
	}
	clk_disable_unprepare(clk->clk);

	clk->is_enabled = false;
	return rc;
}

static void btpower_set_xo_clk_gpio_state(bool enable)
{
	int xo_clk_gpio =  pwr_data->xo_gpio_clk;
	int retry = 0;
	int rc = 0;

	if (xo_clk_gpio < 0)
		return;

retry_gpio_req:
	rc = gpio_request(xo_clk_gpio, "bt_xo_clk_gpio");
	if (rc) {
		if (retry++ < XO_CLK_RETRY_COUNT_MAX) {
			/* wait for ~(10 - 20) ms */
			usleep_range(10000, 20000);
			goto retry_gpio_req;
		}
	}

	if (rc) {
		pr_err("%s: unable to request XO clk gpio %d (%d)\n",
			__func__, xo_clk_gpio, rc);
		return;
	}

	if (enable) {
		gpio_direction_output(xo_clk_gpio, 1);
		/*XO CLK must be asserted for some time before BT_EN */
		usleep_range(100, 200);
	} else {
		/* Assert XO CLK ~(2-5)ms before off for valid latch in HW */
		usleep_range(4000, 6000);
		gpio_direction_output(xo_clk_gpio, 0);
	}

	pr_info("%s:gpio(%d) success\n", __func__, xo_clk_gpio);

	gpio_free(xo_clk_gpio);
}

#ifdef CONFIG_MSM_BT_OOBS
void bt_configure_wakeup_gpios(int on)
{
	int bt_gpio_dev_wake = pwr_data->bt_gpio_dev_wake;
	int bt_host_wake_gpio = pwr_data->bt_gpio_host_wake;
	int rc;

	if (on) {
		if (gpio_is_valid(bt_gpio_dev_wake)) {
			gpio_set_value(bt_gpio_dev_wake, 1);
			pr_debug("%s: BT-ON asserting BT_WAKE(%d)\n", __func__,
				 bt_gpio_dev_wake);
		}

		if (gpio_is_valid(bt_host_wake_gpio)) {
			pwr_data->irq = gpio_to_irq(bt_host_wake_gpio);
			pr_debug("%s: BT-ON bt-host_wake-gpio(%d) IRQ(%d)\n",
				__func__, bt_host_wake_gpio, pwr_data->irq);
			rc = request_irq(pwr_data->irq,
					 btpower_host_wake_isr,
					 IRQF_TRIGGER_FALLING |
					 IRQF_TRIGGER_RISING,
					 "btpower_hostwake_isr", pwr_data);
			if (rc)
				pr_err("%s: unable to request IRQ %d (%d)\n",
				__func__, bt_host_wake_gpio, rc);
		}
	} else {
		if (gpio_is_valid(bt_host_wake_gpio)) {
			pr_debug("%s: BT-OFF bt-hostwake-gpio(%d) IRQ(%d) value(%d)\n",
				 __func__, bt_host_wake_gpio, pwr_data->irq,
				 gpio_get_value(bt_host_wake_gpio));
			free_irq(pwr_data->irq, pwr_data);
		}

		if (gpio_is_valid(bt_gpio_dev_wake))
			gpio_set_value(bt_gpio_dev_wake, 0);
	}
}
#endif

static int get_fmd_mode(void)
{
	return pwr_data->is_fmd_mode_enable;
}

static int bt_pull_resetb(int resetb_gpio, int value)
{
	int rc = 0;

	rc = gpio_direction_output(resetb_gpio, value);
	if (rc) {
		pr_err("%s: Unable to set direction\n", __func__);
		return rc;
	}
	return rc;
}

static int bt_resetb_operation(int resetb)
{
	int rc = 0;

	/* making resetb to low */
	pr_info("BTON: Turn bt_resetb_gpio to low\n");
	rc = bt_pull_resetb(resetb, RESETB_GPIO_LOW);
	if (rc)
		return rc;
	msleep(20);
	/* making resetb to high after delay */
	pr_info("BTON: Turn bt_resetb_gpio to High\n");
	rc = bt_pull_resetb(resetb, RESETB_GPIO_HIGH);
	return rc;
}

static int bt_configure_gpios(int on)
{
	int rc = 0;
	int bt_reset_gpio = pwr_data->bt_gpio_sys_rst;
	int wl_reset_gpio = pwr_data->wl_gpio_sys_rst;
	int bt_sw_ctrl_gpio  =  pwr_data->bt_gpio_sw_ctrl;
	int bt_debug_gpio  =  pwr_data->bt_gpio_debug;
	int bt_resetb_gpio = pwr_data->bt_gpio_resetb;
	int assert_dbg_gpio = 0;

	if (on) {
		rc = gpio_request(bt_reset_gpio, "bt_sys_rst_n");
		if (rc) {
			pr_err("%s: unable to request gpio %d (%d)\n",
					__func__, bt_reset_gpio, rc);
			return rc;
		}
		if (bt_resetb_gpio  >=  0) {
			rc = gpio_request(bt_resetb_gpio, "bt_resetb_gpio_n");
			if (rc) {
				pr_err("%s: unable to request gpio %d (%d)\n",
						__func__, bt_resetb_gpio, rc);
				return rc;
			}
		}

		pr_info("BTON:Turn Bt OFF asserting BT_EN to low\n");
		pr_info("bt-reset-gpio(%d) value(%d)\n", bt_reset_gpio,
			gpio_get_value(bt_reset_gpio));
		rc = gpio_direction_output(bt_reset_gpio, 0);
		if (rc) {
			pr_err("%s: Unable to set direction\n", __func__);
			return rc;
		}
		power_src.platform_state[BT_RESET_GPIO] =
			gpio_get_value(bt_reset_gpio);
		msleep(50);
		pr_info("BTON:Turn Bt OFF post asserting BT_EN to low\n");
		pr_info("bt-reset-gpio(%d) value(%d)\n", bt_reset_gpio,
			gpio_get_value(bt_reset_gpio));

		if (bt_sw_ctrl_gpio >= 0) {
			power_src.platform_state[BT_SW_CTRL_GPIO] =
			gpio_get_value(bt_sw_ctrl_gpio);
			if (pwr_data->sw_cntrl_gpio > 0) {
				rc = msm_gpio_mpm_wake_set(pwr_data->sw_cntrl_gpio, 1);
				if (rc < 0) {
					pr_err("Failed to set msm_gpio_mpm_wake_set for sw_cntrl gpio, ret: %d\n",
						rc);
					return rc;
				}
				pr_info("Set msm_gpio_mpm_wake_set for sw_cntrl gpio successful\n");
			}
			pr_info("BTON:Turn Bt OFF bt-sw-ctrl-gpio(%d) value(%d)\n",
				bt_sw_ctrl_gpio,
				power_src.platform_state[BT_SW_CTRL_GPIO]);
		}
		if (wl_reset_gpio >= 0)
			pr_info("BTON:Turn Bt ON wl-reset-gpio(%d) value(%d)\n",
				wl_reset_gpio, gpio_get_value(wl_reset_gpio));

		if ((wl_reset_gpio < 0) ||
			((wl_reset_gpio >= 0) && gpio_get_value(wl_reset_gpio))) {

			btpower_set_xo_clk_gpio_state(true);
			pr_info("BTON: WLAN ON Asserting BT_EN to high\n");
			rc = gpio_direction_output(bt_reset_gpio, 1);
			if (rc) {
				pr_err("%s: Unable to set direction\n", __func__);
				return rc;
			}
			power_src.platform_state[BT_RESET_GPIO] =
				gpio_get_value(bt_reset_gpio);
			btpower_set_xo_clk_gpio_state(false);
		}
		if ((wl_reset_gpio >= 0) && (gpio_get_value(wl_reset_gpio) == 0)) {
			if (gpio_get_value(bt_reset_gpio)) {
				pr_info("BTON: WLAN OFF and BT ON are too close\n");
				pr_info("reset BT_EN, enable it after delay\n");
				rc = gpio_direction_output(bt_reset_gpio, 0);
				if (rc) {
					pr_err("%s: Unable to set direction\n",
						 __func__);
					return rc;
				}
				power_src.platform_state[BT_RESET_GPIO] =
					gpio_get_value(bt_reset_gpio);
				if (bt_resetb_gpio  >=  0) {
					pr_err("BTON:Turn resetb High\n");
					bt_pull_resetb(bt_resetb_gpio, RESETB_GPIO_HIGH);
				}
			}
			pr_info("BTON: WLAN OFF waiting for 100ms delay\n");
			pr_info("for AON output to fully discharge\n");
			msleep(100);
			pr_info("BTON: WLAN OFF Asserting BT_EN to high\n");
			btpower_set_xo_clk_gpio_state(true);
			if (bt_resetb_gpio  >=  0)
				bt_resetb_operation(bt_resetb_gpio);
			rc = gpio_direction_output(bt_reset_gpio, 1);
			if (rc) {
				pr_err("%s: Unable to set direction\n", __func__);
				return rc;
			}
			power_src.platform_state[BT_RESET_GPIO] =
				gpio_get_value(bt_reset_gpio);
			btpower_set_xo_clk_gpio_state(false);
		}
		/* Below block of code executes if WL_EN is pulled high when
		 * BT_EN is about to pull high. so above two if conditions are
		 * not executed.
		 */
		if (!gpio_get_value(bt_reset_gpio)) {
			btpower_set_xo_clk_gpio_state(true);
			pr_info("BTON: WLAN ON and BT ON are too close\n");
			pr_info("Asserting BT_EN to high\n");
			rc = gpio_direction_output(bt_reset_gpio, 1);
			if (rc) {
				pr_err("%s: Unable to set direction\n", __func__);
				return rc;
			}
			power_src.platform_state[BT_RESET_GPIO] =
				gpio_get_value(bt_reset_gpio);
			btpower_set_xo_clk_gpio_state(false);
		}
		msleep(50);
#ifdef CONFIG_MSM_BT_OOBS
		bt_configure_wakeup_gpios(on);
#endif
		/*  Check  if  SW_CTRL  is  asserted  */
		if  (bt_sw_ctrl_gpio  >=  0)  {
			rc  =  gpio_direction_input(bt_sw_ctrl_gpio);
			if  (rc)  {
				pr_err("%s:SWCTRL Dir Set Problem:%d\n",
					__func__, rc);
			}  else  if  (!gpio_get_value(bt_sw_ctrl_gpio))  {
				/* SW_CTRL not asserted, assert debug GPIO */
				if  (bt_debug_gpio  >=  0)
					assert_dbg_gpio = 1;
			}
		}
		if (assert_dbg_gpio) {
			rc  =  gpio_request(bt_debug_gpio, "bt_debug_n");
			if  (rc)  {
				pr_err("unable to request Debug Gpio\n");
			}  else  {
				rc = gpio_direction_output(bt_debug_gpio,  1);
				if (rc)
					pr_err("%s:Prob Set Debug-Gpio\n",
						__func__);
			}
		}
		pr_info("BTON: Turn BT ON bt-reset-gpio(%d) value(%d)\n",
			bt_reset_gpio, gpio_get_value(bt_reset_gpio));
		if (bt_sw_ctrl_gpio >= 0) {
			power_src.platform_state[BT_SW_CTRL_GPIO] =
			gpio_get_value(bt_sw_ctrl_gpio);
			pr_info("BTON: Turn BT ON bt-sw-ctrl-gpio(%d) value(%d)\n",
				bt_sw_ctrl_gpio,
				power_src.platform_state[BT_SW_CTRL_GPIO]);
		}
	} else {
#ifdef CONFIG_MSM_BT_OOBS
		bt_configure_wakeup_gpios(on);
#endif
		gpio_set_value(bt_reset_gpio, 0);
		msleep(100);
		pr_info("BT-OFF:bt-reset-gpio(%d) value(%d)\n",
			bt_reset_gpio, gpio_get_value(bt_reset_gpio));
		if (bt_sw_ctrl_gpio >= 0) {
			pr_info("BT-OFF:bt-sw-ctrl-gpio(%d) value(%d)\n",
				bt_sw_ctrl_gpio,
				gpio_get_value(bt_sw_ctrl_gpio));
		}
	}

	pr_info("%s: bt_gpio= %d on: %d\n", __func__, bt_reset_gpio, on);

	return rc;
}

static int handle_pwr_disable_req(int core, int reg_num, int retenion_state, int fmd_state)
{
	struct vreg_data *vregs;
	int rc;

	for (int i = 0; i < reg_num; i++) {
		if (core == BT_CORE)
			vregs = &pwr_data->bt_vregs[i];
		else if (core == UWB_CORE)
			vregs = &pwr_data->uwb_vregs[i];
		else if (core == PLATFORM_CORE)
			vregs = &pwr_data->platform_vregs[i];

		if (fmd_state && vregs->fmd_mode_set) {
			if (retenion_state != RETENTION_IDLE) {
				vreg_disable_retention(vregs);
				pr_err("%s: Brought %s reg out-of retention for FMD\n",
					__func__, vregs->name);
			}
			pr_err("%s: Keeping %s reg on power-on state for FMD\n",
				__func__, vregs->name);
		} else {
			rc = vreg_disable(vregs);
		}
	}
	return rc;
}

static int bt_regulators_pwr(int pwr_state)
{
	int i, log_indx, bt_num_vregs, rc = 0;
	struct vreg_data *bt_vregs = NULL;

	rc = perisec_cnss_bt_hw_disable_check(pwr_data);

	bt_num_vregs =  pwr_data->bt_num_vregs;

	if (!bt_num_vregs) {
		pr_warn("%s: not avilable to %s\n",
			__func__, ConvertRegisterModeToString(pwr_state));
		return 0;
	}

	pr_info("%s: %s\n", __func__, ConvertRegisterModeToString(pwr_state));

	if (pwr_state == POWER_ENABLE) {
		/* Power On */
		if (pwr_data->bt_sec_hw_disable) {
			pr_err("%s:secure hw mode on,BT ON not allowed",
				 __func__);
			return -EINVAL;
		}

		for (i = 0; i < bt_num_vregs; i++) {
			bt_vregs = &pwr_data->bt_vregs[i];
			log_indx = bt_vregs->indx.init;
			if (bt_vregs->reg) {
				power_src.bt_state[log_indx] = DEFAULT_INVALID_VALUE;
				rc = vreg_enable(bt_vregs);
				if (rc < 0) {
					pr_err("%s: bt_power regulators config failed\n",
						__func__);
					goto regulator_fail;
				}
				if (bt_vregs->is_enabled)
					power_src.bt_state[log_indx] =
						regulator_get_voltage(bt_vregs->reg);
			}
		}

		if (pwr_data->bt_gpio_sys_rst > 0) {
			power_src.bt_state[BT_RESET_GPIO] = DEFAULT_INVALID_VALUE;
			power_src.bt_state[BT_SW_CTRL_GPIO] = DEFAULT_INVALID_VALUE;
			rc = bt_configure_gpios(POWER_ENABLE);
			if (rc < 0) {
				pr_err("%s: bt_power gpio config failed\n",
					__func__);
				goto gpio_fail;
			}
		}
	} else if (pwr_state == POWER_DISABLE) {
		/* Power Off */
		if (pwr_data->bt_gpio_sys_rst > 0) {
			if (pwr_data->bt_sec_hw_disable) {
				pr_err("%s: secure hw mode on, not allowed to access gpio",
					__func__);
			}else {
				if (!get_fmd_mode())
					bt_configure_gpios(POWER_DISABLE);
			}
		}
gpio_fail:
		if (!get_fmd_mode()) {
			if (pwr_data->bt_gpio_sys_rst > 0)
				gpio_free(pwr_data->bt_gpio_sys_rst);
			if (pwr_data->bt_gpio_debug  >  0)
				gpio_free(pwr_data->bt_gpio_debug);
			if (pwr_data->bt_chip_clk)
				bt_clk_disable(pwr_data->bt_chip_clk);
			if (pwr_data->bt_gpio_resetb  >  0)
				gpio_free(pwr_data->bt_gpio_resetb);
		}
regulator_fail:
		rc = handle_pwr_disable_req(BT_CORE,
			bt_num_vregs,
			btpower_get_retenion_mode_state(),
			get_fmd_mode());
	} else if (pwr_state == POWER_RETENTION) {
		/* Retention mode */
		for (i = 0; i < bt_num_vregs; i++) {
			bt_vregs = &pwr_data->bt_vregs[i];
			rc = vreg_enable_retention(bt_vregs);
		}
	} else {
		pr_err("%s: Invalid power mode: %d\n", __func__, pwr_state);
		rc = -1;
	}
	return rc;
}

static int uwb_regulators_pwr(int pwr_state)
{
	int i, log_indx, uwb_num_vregs, rc = 0;
	struct vreg_data *uwb_vregs = NULL;

	rc = perisec_cnss_bt_hw_disable_check(pwr_data);

	uwb_num_vregs =  pwr_data->uwb_num_vregs;

	if (!uwb_num_vregs) {
		pr_warn("%s: not avilable to %s\n",
			__func__, ConvertRegisterModeToString(pwr_state));
		return 0;
	}

	pr_info("%s: %s\n", __func__, ConvertRegisterModeToString(pwr_state));

	switch (pwr_state) {
	case POWER_ENABLE:
		for (i = 0; i < uwb_num_vregs; i++) {
			uwb_vregs = &pwr_data->uwb_vregs[i];
			log_indx = uwb_vregs->indx.init;
			if (uwb_vregs->reg) {
				power_src.uwb_state[log_indx] = DEFAULT_INVALID_VALUE;
				rc = vreg_enable(uwb_vregs);
				if (rc < 0) {
					pr_err("%s: UWB regulators config failed\n",
						__func__);
					goto regulator_failed;
				}
				if (uwb_vregs->is_enabled) {
					power_src.uwb_state[log_indx] =
						regulator_get_voltage(uwb_vregs->reg);
				}
			}
		}
		break;
	case POWER_DISABLE:
regulator_failed:
		for (i = 0; i < uwb_num_vregs; i++) {
			uwb_vregs = &pwr_data->uwb_vregs[i];
			rc = vreg_disable(uwb_vregs);
		}
		break;
	case POWER_RETENTION:
		for (i = 0; i < uwb_num_vregs; i++) {
			uwb_vregs = &pwr_data->uwb_vregs[i];
			rc = vreg_enable_retention(uwb_vregs);
		}
		break;
	}
	return rc;
}

static bool is_wlan_mx_buck(struct vreg_data *reg)
{

	if (strcmp(reg->name, "qcom,bt-vdd-wlan-aon"))
		return false;
	else
		return true;
}

static int vote_wlan_reg_for_fmd(void)
{
	int log_indx;
	struct vreg_data *vregs = pwr_data->wlan_vregs;

	if ((vregs == NULL) || (!is_wlan_mx_buck(vregs))) {
		pr_err("%s: Regulator qcom,bt-vdd-wlan-aon is not avilable\n", __func__);
		return -1;
	}

	if (vregs->reg == NULL)
		return -1;

	log_indx = vregs->indx.init;
	power_src.platform_state[log_indx] = DEFAULT_INVALID_VALUE;

	if (vreg_enable(vregs) < 0) {
		pr_err("%s: Platform regulators config failed\n", __func__);
		return -1;
	}

	if (vregs->is_enabled) {
		power_src.platform_state[log_indx] = regulator_get_voltage(vregs->reg);
		pr_err("%s: Regulator %s voted-on Successfully\n", __func__, vregs->name);
	} else {
		pr_err("%s: Regulator qcom,bt-vdd-wlan-aon is not enabled\n", __func__);
		return -1;
	}
	return 0;
}

static int platform_regulators_pwr(int pwr_state)
{
	int i, log_indx, platform_num_vregs, rc = 0;
	struct vreg_data *platform_vregs = NULL;

	rc = perisec_cnss_bt_hw_disable_check(pwr_data);

	platform_num_vregs =  pwr_data->platform_num_vregs;

	if (!platform_num_vregs) {
		pr_warn("%s: not avilable to %s\n",
			__func__, ConvertRegisterModeToString(pwr_state));
		return 0;
	}

	pr_info("%s: %s\n", __func__, ConvertRegisterModeToString(pwr_state));

	switch (pwr_state) {
	case POWER_ENABLE:
		for (i = 0; i < platform_num_vregs; i++) {
			platform_vregs = &pwr_data->platform_vregs[i];
			log_indx = platform_vregs->indx.init;
			if (platform_vregs->reg) {
				power_src.platform_state[log_indx] = DEFAULT_INVALID_VALUE;
				rc = vreg_enable(platform_vregs);
				if (rc < 0) {
					pr_err("%s: Platform regulators config failed\n",
						__func__);
					goto regulator_failed;
				}
				if (platform_vregs->is_enabled)
					power_src.platform_state[log_indx] =
						regulator_get_voltage(platform_vregs->reg);
			}
		}
		rc = bt_configure_gpios(POWER_ENABLE);
		if (rc < 0) {
			pr_err("%s: bt_power gpio config failed\n",
				__func__);
			goto gpio_failed;
		}
		break;
	case POWER_DISABLE:
		if (!get_fmd_mode()) {
			rc = bt_configure_gpios(POWER_DISABLE);
			if (rc < 0) {
				pr_err("%s: bt_power gpio config failed\n",
					__func__);
			}
		}
gpio_failed:
		if (!get_fmd_mode()) {
			if (pwr_data->bt_gpio_sys_rst > 0)
				gpio_free(pwr_data->bt_gpio_sys_rst);
			if (pwr_data->bt_gpio_debug  >  0)
				gpio_free(pwr_data->bt_gpio_debug);
		}
regulator_failed:
		rc = handle_pwr_disable_req(PLATFORM_CORE,
			platform_num_vregs,
			btpower_get_retenion_mode_state(),
			get_fmd_mode());
		break;
	case POWER_RETENTION:
		for (i = 0; i < platform_num_vregs; i++) {
			platform_vregs = &pwr_data->platform_vregs[i];
			rc = vreg_enable_retention(platform_vregs);
		}
		break;
	case POWER_DISABLE_RETENTION:
		for (i = 0; i < platform_num_vregs; i++) {
			platform_vregs = &pwr_data->platform_vregs[i];
			rc = vreg_disable_retention(platform_vregs);
		}
		break;
	}
	return rc;
}

static int power_regulators(int core_type, int mode)
{
	int ret = 0;

	if ((mode != POWER_DISABLE) && (mode != POWER_ENABLE) &&
		(mode != POWER_RETENTION)) {
		pr_err("%s: Received wrong Mode to do regulator operation\n",
			__func__);
		return -1;
	}

	switch (core_type) {
	case BT_CORE:
		ret = bt_regulators_pwr(mode);
		if (ret)
			pr_err("%s: Failed to configure BT regulators to mode(%d)\n",
			__func__, mode);
		break;
	case UWB_CORE:
		ret = uwb_regulators_pwr(mode);
		if (ret)
			pr_err("%s: Failed to configure UWB regulators to mode(%d)\n",
			__func__, mode);
		break;
	case PLATFORM_CORE:
		ret = platform_regulators_pwr(mode);
		if (ret)
			pr_err("%s: Failed to configure platform regulators to mode(%d)\n",
			__func__, mode);
		break;
	default:
		pr_err("%s: Received wrong Core Type to do regulator operation\n",
			__func__);
		return -1;
	}
	return ret;
}

static int btpower_toggle_radio(void *data, bool blocked)
{
	int ret = 0;
	int (*power_control)(int Core, int enable);

	power_control =
		((struct platform_pwr_data *)data)->power_setup;

	if (previous != blocked)
		ret = (*power_control)(BT_CORE, !blocked);
	if (!ret)
		previous = blocked;
	return ret;
}

static const struct rfkill_ops btpower_rfkill_ops = {
	.set_block = btpower_toggle_radio,
};

static ssize_t extldo_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return scnprintf(buf, 6, "false\n");
}

static DEVICE_ATTR_RO(extldo);

static int btpower_rfkill_probe(struct platform_device *pdev)
{
	struct rfkill *rfkill;
	int ret;

	rfkill = rfkill_alloc("bt_power", &pdev->dev, RFKILL_TYPE_BLUETOOTH,
						&btpower_rfkill_ops,
						pdev->dev.platform_data);

	if (!rfkill) {
		dev_err(&pdev->dev, "rfkill allocate failed\n");
		return -ENOMEM;
	}

	/* add file into rfkill0 to handle LDO27 */
	ret = device_create_file(&pdev->dev, &dev_attr_extldo);
	if (ret < 0)
		pr_err("%s: device create file error\n", __func__);

	/* force Bluetooth off during init to allow for user control */
	rfkill_init_sw_state(rfkill, 1);
	previous = true;

	ret = rfkill_register(rfkill);
	if (ret) {
		dev_err(&pdev->dev, "rfkill register failed=%d\n", ret);
		rfkill_destroy(rfkill);
		return ret;
	}

	platform_set_drvdata(pdev, rfkill);

	return 0;
}

static void btpower_rfkill_remove(struct platform_device *pdev)
{
	struct rfkill *rfkill;

	pr_debug("%s\n", __func__);

	rfkill = platform_get_drvdata(pdev);
	if (rfkill)
		rfkill_unregister(rfkill);
	rfkill_destroy(rfkill);
	platform_set_drvdata(pdev, NULL);
}

static int dt_parse_vreg_info(struct device *dev, struct device_node *child,
		struct vreg_data *vreg_data)
{
	int len, ret = 0;
	const __be32 *prop;
	char prop_name[MAX_PROP_SIZE];
	struct vreg_data *vreg = vreg_data;
	struct device_node *np = child;
	const char *vreg_name = vreg_data->name;

	if (!child)
		np = dev->of_node;

	snprintf(prop_name, sizeof(prop_name), "%s-supply", vreg_name);
	if (of_parse_phandle(np, prop_name, 0)) {
		vreg->reg = regulator_get(dev, vreg_name);
		if (IS_ERR(vreg->reg)) {
			ret = PTR_ERR(vreg->reg);
			vreg->reg = NULL;
			pr_err("%s: failed to get: %s error:%d\n", __func__,
				vreg_name, ret);
			return ret;
		}

		snprintf(prop_name, sizeof(prop_name), "%s-config", vreg->name);
		prop = of_get_property(np, prop_name, &len);
		if (!prop) {
			pr_err("%s: Property %s %s, use default\n",
				__func__, prop_name,
				prop ? "invalid format" : "doesn't exist");
		} else if (len == (5 * sizeof(__be32))) {
			vreg->min_vol = be32_to_cpup(&prop[0]);
			vreg->max_vol = be32_to_cpup(&prop[1]);
			vreg->load_curr = be32_to_cpup(&prop[2]);
			vreg->is_retention_supp = be32_to_cpup(&prop[3]);
			vreg->fmd_mode_set = be32_to_cpup(&prop[4]);
			pr_err("%s: FMD mode %d for regulator %s\n", __func__,
							vreg->fmd_mode_set, vreg->name);
		} else if (len != (4 * sizeof(__be32))) {
			pr_err("%s: Property %s %s, use default\n",
				__func__, prop_name,
				prop ? "invalid format" : "doesn't exist");
		} else {
			vreg->min_vol = be32_to_cpup(&prop[0]);
			vreg->max_vol = be32_to_cpup(&prop[1]);
			vreg->load_curr = be32_to_cpup(&prop[2]);
			vreg->is_retention_supp = be32_to_cpup(&prop[3]);
		}

		pr_err("%s: Got regulator: %s, min_vol: %u, max_vol: %u, load_curr: %u,is_retention_supp: %u\n",
			__func__, vreg->name, vreg->min_vol, vreg->max_vol,
			vreg->load_curr, vreg->is_retention_supp);
	} else {
		pr_err("%s: %s is not provided in device tree\n", __func__,
			vreg_name);
	}
	return ret;
}

static int bt_dt_parse_clk_info(struct device *dev,
		struct bt_power_clk_data **clk_data)
{
	int ret = -EINVAL;
	struct bt_power_clk_data *clk = NULL;
	struct device_node *np = dev->of_node;

	pr_debug("%s\n", __func__);

	*clk_data = NULL;
	if (of_parse_phandle(np, "clocks", 0)) {
		clk = devm_kzalloc(dev, sizeof(*clk), GFP_KERNEL);
		if (!clk) {
			ret = -ENOMEM;
			goto err;
		}

		/* Allocated 20 bytes size buffer for clock name string */
		clk->name = devm_kzalloc(dev, 20, GFP_KERNEL);

		/* Parse clock name from node */
		ret = of_property_read_string_index(np, "clock-names", 0,
				&(clk->name));
		if (ret < 0) {
			pr_err("%s: reading \"clock-names\" failed\n",
				__func__);
			return ret;
		}

		clk->clk = devm_clk_get(dev, clk->name);
		if (IS_ERR(clk->clk)) {
			ret = PTR_ERR(clk->clk);
			pr_err("%s: failed to get %s, ret (%d)\n",
				__func__, clk->name, ret);
			clk->clk = NULL;
			return ret;
		}

		*clk_data = clk;
	} else {
		pr_err("%s: clocks is not provided in device tree\n", __func__);
	}

err:
	return ret;
}

static void bt_power_vreg_put(void)
{
	int i = 0;
	struct vreg_data *bt_vregs = NULL;
	int bt_num_vregs = pwr_data->bt_num_vregs;

	for (; i < bt_num_vregs; i++) {
		bt_vregs = &pwr_data->bt_vregs[i];
		if (bt_vregs->reg)
			regulator_put(bt_vregs->reg);
	}
}

static int get_gpio_dt_pinfo(struct platform_device *pdev)
{
	int ret;
	struct device_node *child;
	struct pinctrl *pinctrl1;
#ifdef CONFIG_FMD_ENABLE
	struct pinctrl_state *sw_ctrl;
	struct pinctrl_state *bt_en;
#endif
	child = pdev->dev.of_node;

	pinctrl1 =  devm_pinctrl_get(&pdev->dev);
	pwr_data->bt_gpio_sys_rst =
		of_get_named_gpio(child,
					"qcom,bt-reset-gpio", 0);
	if (pwr_data->bt_gpio_sys_rst < 0)
		pr_warn("bt-reset-gpio not provided in devicetree\n");

	pwr_data->wl_gpio_sys_rst =
		of_get_named_gpio(child,
					"qcom,wl-reset-gpio", 0);
	if (pwr_data->wl_gpio_sys_rst < 0)
		pr_err("%s: wl-reset-gpio not provided in device tree\n",
			__func__);

	pwr_data->bt_gpio_resetb =
		of_get_named_gpio(child,
					"qcom,wl-resetb-gpio", 0);
	if (pwr_data->bt_gpio_resetb < 0)
		pr_err("%s: bt_gpio_resetb not provided in device tree\n",
			__func__);

	pwr_data->bt_gpio_sw_ctrl  =
		of_get_named_gpio(child,
					"qcom,bt-sw-ctrl-gpio",  0);
	if (pwr_data->bt_gpio_sw_ctrl < 0)
		pr_warn("bt-sw-ctrl-gpio not provided in devicetree\n");

	pwr_data->bt_gpio_fmd_clk_ctrl  =
		of_get_named_gpio(child,
					"qcom,bt-fmd-clk-gpio",  0);
	if (pwr_data->bt_gpio_fmd_clk_ctrl < 0)
		pr_warn("bt-fmd-clk-gpio not provided in devicetree\n");

	ret = of_property_read_u32(child, "mpm_wake_set_gpios", &pwr_data->sw_cntrl_gpio);

	if (ret) {
		pr_warn("sw_cntrl-gpio not provided in devicetree\n");
	}
#ifdef CONFIG_FMD_ENABLE
	if (pinctrl1) {
		sw_ctrl = pinctrl_lookup_state(pinctrl1, "sw_ctrl");
		if (IS_ERR_OR_NULL(sw_ctrl)) {
			ret = PTR_ERR(sw_ctrl);
			pr_err("Failed to get sw_ctrl state, err = %d\n", ret);
		} else {
			ret = pinctrl_select_state(pinctrl1, sw_ctrl);
			if (ret)
				pr_err("Failed to select sw_ctrl state, err = %d\n", ret);
		}
		bt_en = pinctrl_lookup_state(pinctrl1, "bt_en");
		if (IS_ERR_OR_NULL(bt_en)) {
			ret = PTR_ERR(bt_en);
			pr_err("Failed to get bt_en state, err = %d\n", ret);
		} else {
			ret = pinctrl_select_state(pinctrl1, bt_en);
			if (ret)
				pr_err("Failed to select bt_en state, err = %d\n", ret);
		}
	} else {
		pr_err("%s: pinctrl is null\n", __func__);
	}
#endif
	pwr_data->bt_gpio_debug  = of_get_named_gpio(child,
							"qcom,bt-debug-gpio", 0);
	if (pwr_data->bt_gpio_debug < 0)
		pr_warn("bt-debug-gpio not provided in devicetree\n");

	pwr_data->xo_gpio_clk =
		of_get_named_gpio(child,
					"qcom,xo-clk-gpio", 0);
	if (pwr_data->xo_gpio_clk < 0)
		pr_warn("xo-clk-gpio not provided in devicetree\n");

#ifdef CONFIG_MSM_BT_OOBS
	pwr_data->bt_gpio_dev_wake =
		of_get_named_gpio(child,
				  "qcom,btwake_gpio", 0);
	if (pwr_data->bt_gpio_dev_wake < 0)
		pr_warn("%s: btwake-gpio not provided in device tree\n",
			__func__);

	pwr_data->bt_gpio_host_wake =
		of_get_named_gpio(child,
				  "qcom,bthostwake_gpio", 0);
	if (pwr_data->bt_gpio_host_wake < 0)
		pr_warn("%s: bthostwake_gpio not provided in device tree\n",
			__func__);
#endif
	return true;
}

static int get_power_dt_pinfo(struct platform_device *pdev)
{
	int rc, i;
	const struct pwr_data *data;

	data = of_device_get_match_data(&pdev->dev);
	if (!data) {
		pr_err("%s: failed to get dev node\n", __func__);
		return -EINVAL;
	}

	memcpy(&pwr_data->compatible, &data->compatible, MAX_PROP_SIZE);
	pwr_data->bt_vregs = data->bt_vregs;
	pwr_data->bt_num_vregs = data->bt_num_vregs;

	if (pwr_data->is_ganges_dt) {
		pwr_data->uwb_vregs = data->uwb_vregs;
		pwr_data->platform_vregs = data->platform_vregs;
		pwr_data->uwb_num_vregs = data->uwb_num_vregs;
		pwr_data->platform_num_vregs = data->platform_num_vregs;
	}

	for (i = 0; i < pwr_data->bt_num_vregs; i++) {
		rc = dt_parse_vreg_info(&(pdev->dev), pwr_data->bt_of_node,
			&pwr_data->bt_vregs[i]);
		/* No point to go further if failed to get regulator handler */
		if (rc)
			return rc;
	}

	if (pwr_data->is_ganges_dt) {
		for (i = 0; i < pwr_data->platform_num_vregs; i++) {
			if (is_wlan_mx_buck(&pwr_data->platform_vregs[i])) {
				pwr_data->wlan_vregs = &pwr_data->platform_vregs[i];
				pwr_data->platform_num_vregs--;
				pr_err("%s: Found wlan regulator for FMD Operations '%s'\n",
					__func__, (pwr_data->wlan_vregs)->name);
			}
			rc = dt_parse_vreg_info(&(pdev->dev), NULL,
				&pwr_data->platform_vregs[i]);
			/* No point to go further if failed to get regulator handler */
			if (rc)
				return rc;
		}

		for (i = 0; i < pwr_data->uwb_num_vregs; i++) {
			rc = dt_parse_vreg_info(&(pdev->dev), pwr_data->uwb_of_node,
				&pwr_data->uwb_vregs[i]);
			/* No point to go further if failed to get regulator handler */
			if (rc)
				return rc;
		}

		pr_info("%s: platform_name = %s: bt_num_vregs =%d uwb_num_vregs =%d platform_num_vregs=%d\n",
			__func__, pwr_data->compatible, pwr_data->bt_num_vregs,
			pwr_data->uwb_num_vregs, pwr_data->platform_num_vregs);

	} else {

		pr_info("%s: platform_name = %s: bt_num_vregs =%d\n", __func__,
			pwr_data->compatible, pwr_data->bt_num_vregs);

	}
	return rc;
}

static int bt_power_populate_dt_pinfo(struct platform_device *pdev)
{
	struct device_node *of_node;
	int rc;

	if (!pwr_data)
		return -ENOMEM;

	if (pwr_data->is_ganges_dt) {
		for_each_available_child_of_node(pdev->dev.of_node, of_node) {
			if (!strcmp(of_node->name, "bt_ganges")) {
				pwr_data->bt_of_node = of_node;
				pr_info("%s: %s device node found\n", __func__,
					pwr_data->bt_of_node->name);
			} else if (!strcmp(of_node->name, "uwb_ganges")) {
				pwr_data->uwb_of_node = of_node;
				pr_info("%s: %s device node found\n", __func__,
					pwr_data->uwb_of_node->name);
			}
		}
	}

	rc = get_power_dt_pinfo(pdev);

	if (rc < 0)
		return rc;

	get_gpio_dt_pinfo(pdev);
	bt_dt_parse_clk_info(&pdev->dev, &pwr_data->bt_chip_clk);
	pwr_data->power_setup = power_regulators;
	return 0;
}

static void bt_power_pdc_init_params(struct platform_pwr_data *pdata)
{
	int ret;
	struct device *dev = &pdata->pdev->dev;
	pdata->pdc_init_table_len = of_property_count_strings(dev->of_node,
				"qcom,pdc_init_table");
	if (pdata->pdc_init_table_len > 0) {
		pdata->pdc_init_table = kcalloc(pdata->pdc_init_table_len,
				sizeof(char *), GFP_KERNEL);
		ret = of_property_read_string_array(dev->of_node, "qcom,pdc_init_table",
			pdata->pdc_init_table, pdata->pdc_init_table_len);
		if (ret < 0)
			pr_err("Failed to get PDC Init Table\n");
		else
			pr_info("PDC Init table configured\n");
	} else {
		pr_debug("PDC Init Table not configured\n");
	}
}

static void bt_signal_handler(struct work_struct *w_arg)
{
	struct kernel_siginfo siginfo;
	int rc = 0;

	memset(&siginfo, 0, sizeof(siginfo));
	siginfo.si_signo = SIGIO;
	siginfo.si_code = SI_QUEUE;
	siginfo.si_int = pwr_data->wrkq_signal_state;
	rc = send_sig_info(siginfo.si_signo, &siginfo, pwr_data->reftask_bt);
	if (rc < 0)
		pr_err("%s: failed (%d) to send SIG to HAL(%d)\n", __func__,
				rc, pwr_data->reftask_bt->pid);
	else
		pr_err("%s: Signal to BT HAL (PID-%d) succesfull\n", __func__,
				pwr_data->reftask_bt->pid);
}

static void uwb_signal_handler(struct work_struct *w_arg)
{
	struct kernel_siginfo siginfo;
	int rc = 0;

	memset(&siginfo, 0, sizeof(siginfo));
	siginfo.si_signo =  SIGIO;
	siginfo.si_code = SI_QUEUE;
	siginfo.si_int = pwr_data->wrkq_signal_state;
	rc = send_sig_info(siginfo.si_signo, &siginfo, pwr_data->reftask_uwb);
	if (rc < 0)
		pr_err("%s: failed (%d) to send SIG to HAL(%d)\n", __func__,
				rc, pwr_data->reftask_uwb->pid);
	else
		pr_err("%s: Signal to UWB HAL (PID-%d) succesfull\n", __func__,
				pwr_data->reftask_uwb->pid);
}

static int bt_power_probe(struct platform_device *pdev)
{
	int ret = 0;
	int itr;

	/* Fill whole array with -2 i.e NOT_AVAILABLE state by default
	 * for any GPIO or Reg handle.
	 */
	for (itr = PWR_SRC_INIT_STATE_IDX; itr < BT_POWER_SRC_SIZE; ++itr) {
		power_src.bt_state[itr] = PWR_SRC_NOT_AVAILABLE;
		power_src.platform_state[itr] = PWR_SRC_NOT_AVAILABLE;
		power_src.uwb_state[itr] = PWR_SRC_NOT_AVAILABLE;
	}

	pwr_data = kzalloc(sizeof(*pwr_data), GFP_KERNEL);

	if (!pwr_data)
		return -ENOMEM;

	pwr_data->pdev = pdev;
	pwr_data->reftask_bt = NULL;
	pwr_data->reftask_uwb = NULL;

	struct device *devi = &pwr_data->pdev->dev;
	int rc = 0;
	pr_info("%s: Get FMD nvmem-cells\n", __func__);
/* Get fmd_set NVMEM  Cell Handler */
	pwr_data->nvmem_cell_fmd_set =
		devm_nvmem_cell_get(devi, "fmd_set");
	if (IS_ERR(pwr_data->nvmem_cell_fmd_set)) {
		rc = PTR_ERR(pwr_data->nvmem_cell_fmd_set);
		pr_err("%s:Failed to get fmd_set nvmem-cells %d\n",
			__func__, rc);
		pr_err("%s:Skip, Reading of other 2 Cells have no use\n",
			__func__);
		pwr_data->nvmem_cell_fmd_chg_pon =
			pwr_data->nvmem_cell_fmd_set;
		pwr_data->nvmem_cell_fmd_cnt2_stop =
			pwr_data->nvmem_cell_fmd_set;
	} else {
		pr_info("%s: Got fmd_set nvmem-cells\n", __func__);
/* Get fmd_chg_pon NVMEM Cell Handler */
		pwr_data->nvmem_cell_fmd_chg_pon =
			devm_nvmem_cell_get(devi, "fmd_chg_pon");
		if (IS_ERR(pwr_data->nvmem_cell_fmd_chg_pon)) {
			rc = PTR_ERR(pwr_data->nvmem_cell_fmd_chg_pon);
			pr_err("%s:Failed to get fmd_chg_pon nvmem-cells %d\n",
				__func__, rc);
		} else {
			pr_info("%s: Got fmd_chg_pon nvmem-cells\n", __func__);
		}
/* Get fmd_cnt2_stop NVMEM Cell Handler */
		pwr_data->nvmem_cell_fmd_cnt2_stop =
			devm_nvmem_cell_get(devi, "fmd_cnt2_stop");
		if (IS_ERR(pwr_data->nvmem_cell_fmd_cnt2_stop)) {
			rc = PTR_ERR(pwr_data->nvmem_cell_fmd_cnt2_stop);
			pr_err("%s:Failed to get fmd_cnt2_stop nvmem-cells %d\n",
				__func__, rc);
		} else {
			pr_info("%s: Got fmd_cnt2_stop nvmem-cells\n", __func__);
		}
	}
	pr_info("%s: FMD nvmem-cells read completed\n", __func__);

	pwr_data->is_ganges_dt = of_property_read_bool(pdev->dev.of_node,
							"qcom,peach-bt") ||
							of_property_read_bool(pdev->dev.of_node,
							"qcom,wcn786x");
	pr_info("%s: is_ganges_dt = %d\n", __func__, pwr_data->is_ganges_dt);

	pwr_data->workq = alloc_workqueue("workq", WQ_HIGHPRI, WQ_DFL_ACTIVE);
	if (!pwr_data->workq) {
		pr_err("%s: Failed to creat the Work Queue (workq)\n",
			__func__);
		return -ENOMEM;
	}

	INIT_WORK(&pwr_data->uwb_wq, uwb_signal_handler);
	INIT_WORK(&pwr_data->bt_wq, bt_signal_handler);
	INIT_WORK(&pwr_data->wq_pwr_voting, bt_power_vote);

	for (itr = 0; itr < BTPWR_MAX_REQ; itr++) {
		init_waitqueue_head(&pwr_data->rsp_wait_q[itr]);
	}

	skb_queue_head_init(&pwr_data->rxq);
	mutex_init(&pwr_data->pwr_mtx);
	mutex_init(&pwr_data->btpower_state.state_machine_lock);
	pwr_data->btpower_state.power_state = IDLE;
	pwr_data->btpower_state.retention_mode = RETENTION_IDLE;
	pwr_data->btpower_state.grant_state = NO_GRANT_FOR_ANY_SS;
	pwr_data->btpower_state.grant_pending = NO_OTHER_CLIENT_WAITING_FOR_GRANT;

	perisec_cnss_bt_hw_disable_check(pwr_data);

	if (pdev->dev.of_node) {
		ret = bt_power_populate_dt_pinfo(pdev);
		if (ret < 0) {
			pr_err("%s, Failed to populate device tree info\n",
				__func__);
			goto free_pdata;
		}
		if (pwr_data->bt_sec_hw_disable) {
			pr_info("%s: bt is in secure mode\n", __func__);
		} else {
			pr_info(" %s:send platform data of btpower\n", __func__);
			pdev->dev.platform_data = pwr_data;
		}
	} else if (pdev->dev.platform_data) {
		/* Optional data set to default if not provided */
		if (!((struct platform_pwr_data *)
			(pdev->dev.platform_data))->power_setup)
			((struct platform_pwr_data *)
				(pdev->dev.platform_data))->power_setup =
						power_regulators;

		memcpy(pwr_data, pdev->dev.platform_data,
			sizeof(struct platform_pwr_data));
	} else {
		pr_err("%s: Failed to get platform data\n", __func__);
		goto free_pdata;
	}

	if (btpower_rfkill_probe(pdev) < 0)
		goto free_pdata;

	bt_power_pdc_init_params(pwr_data);
	btpower_aop_mbox_init(pwr_data);

	probe_finished = true;
	return 0;

free_pdata:
	kfree(pwr_data);
	return ret;
}

static int bt_power_remove(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "%s\n", __func__);
	probe_finished = false;
	btpower_rfkill_remove(pdev);
	bt_power_vreg_put();
	if (pwr_data->is_ganges_dt)
		destroy_workqueue(pwr_data->workq);
	kfree(pwr_data);
	return 0;
}

int btpower_register_slimdev(struct device *dev)
{
	pr_debug("%s\n", __func__);
	if (!pwr_data || (dev == NULL)) {
		pr_err("%s: Failed to allocate memory\n", __func__);
		return -EINVAL;
	}
	pwr_data->slim_dev = dev;
	return 0;
}
EXPORT_SYMBOL(btpower_register_slimdev);

int btpower_get_chipset_version(void)
{
	pr_debug("%s\n", __func__);
	return soc_id;
}
EXPORT_SYMBOL(btpower_get_chipset_version);

static void  set_pwr_srcs_status(struct vreg_data *handle,
	int core_type)
{
	int power_src_state;

	if (!handle) {
		pr_err("%s: invalid handler received\n", __func__);
		return;
	}

	if (handle->is_enabled)
		power_src_state = (int)regulator_get_voltage(handle->reg);
	else
		power_src_state = DEFAULT_INVALID_VALUE;

	switch (core_type) {
	case BT_CORE:
		power_src.bt_state[handle->indx.crash] = power_src_state;
		if (power_src_state != DEFAULT_INVALID_VALUE) {
			pr_err("%s(%p) value(%d)\n", handle->name, handle,
				power_src.bt_state[handle->indx.crash]);
		} else {
			pr_err("%s:%s is_enabled: %d\n", __func__, handle->name,
				handle->is_enabled);
		}
		break;
	case UWB_CORE:
		power_src.uwb_state[handle->indx.crash] = power_src_state;
		if (power_src_state != DEFAULT_INVALID_VALUE) {
			pr_err("%s(%p) value(%d)\n", handle->name, handle,
				power_src.uwb_state[handle->indx.crash]);
		} else {
			pr_err("%s:%s is_enabled: %d\n", __func__, handle->name,
				handle->is_enabled);
		}
		break;
	case PLATFORM_CORE:
		power_src.platform_state[handle->indx.crash] = power_src_state;
		if (power_src_state != DEFAULT_INVALID_VALUE) {
			pr_err("%s(%p) value(%d)\n", handle->name, handle,
				power_src.platform_state[handle->indx.crash]);
		} else {
			pr_err("%s:%s is_enabled: %d\n", __func__, handle->name,
				handle->is_enabled);
		}
		break;
	default:
		pr_err("%s: invalid core type received = %d\n", __func__, core_type);
		break;
	}
}

static inline void update_pwr_state(int state)
{
	mutex_lock(&pwr_data->btpower_state.state_machine_lock);
	pwr_data->btpower_state.power_state = state;
	mutex_unlock(&pwr_data->btpower_state.state_machine_lock);
}

static inline int get_pwr_state(void)
{
	int state;
	mutex_lock(&pwr_data->btpower_state.state_machine_lock);
	state = (int)pwr_data->btpower_state.power_state;
	mutex_unlock(&pwr_data->btpower_state.state_machine_lock);
	return state;
}

static inline void btpower_set_retenion_mode_state(int state)
{
	mutex_lock(&pwr_data->btpower_state.state_machine_lock);
	pwr_data->btpower_state.retention_mode = state;
	mutex_unlock(&pwr_data->btpower_state.state_machine_lock);
}

static inline int btpower_get_retenion_mode_state(void)
{
	int state;
	mutex_lock(&pwr_data->btpower_state.state_machine_lock);
	state = (int)pwr_data->btpower_state.retention_mode;
	mutex_unlock(&pwr_data->btpower_state.state_machine_lock);
	return state;
}

static inline void btpower_set_grant_pending_state(enum grant_states state)
{
	mutex_lock(&pwr_data->btpower_state.state_machine_lock);
	pwr_data->btpower_state.grant_pending = state;
	mutex_unlock(&pwr_data->btpower_state.state_machine_lock);
}

static inline enum grant_states btpower_get_grant_pending_state(void)
{
	enum grant_states state;
	mutex_lock(&pwr_data->btpower_state.state_machine_lock);
	state = pwr_data->btpower_state.grant_pending;
	mutex_unlock(&pwr_data->btpower_state.state_machine_lock);
	return state;
}

static inline void btpower_set_grant_state(enum grant_states state)
{
	mutex_lock(&pwr_data->btpower_state.state_machine_lock);
	pwr_data->btpower_state.grant_state = state;
	mutex_unlock(&pwr_data->btpower_state.state_machine_lock);
}

static inline enum grant_states btpower_get_grant_state(void)
{
	enum grant_states state;
	mutex_lock(&pwr_data->btpower_state.state_machine_lock);
	state = pwr_data->btpower_state.grant_state;
	mutex_unlock(&pwr_data->btpower_state.state_machine_lock);
	return state;
}

static int get_sub_state(void)
{
	return (int)pwr_data->sub_state;
}

static void update_sub_state(int state)
{
	pwr_data->sub_state = state;
}

int power_enable (enum SubSystem SubSystemType)
{
	int ret;

	switch (get_pwr_state()) {
	case IDLE:
		ret = power_regulators(PLATFORM_CORE, POWER_ENABLE);
		if (SubSystemType == BLUETOOTH) {
			ret = power_regulators(BT_CORE, POWER_ENABLE);
			update_pwr_state(BT_ON);
		} else {
			ret = power_regulators(UWB_CORE, POWER_ENABLE);
			update_pwr_state(UWB_ON);
		}
		break;
	case BT_ON:
		if (SubSystemType == BLUETOOTH) {
			pr_err("%s: BT Regulators already Voted-On\n",
				__func__);
			return 0;
		}
		ret = power_regulators(UWB_CORE, POWER_ENABLE);
		update_pwr_state(ALL_CLIENTS_ON);
		break;
	case UWB_ON:
		if (SubSystemType == UWB) {
			pr_err("%s: UWB Regulators already Voted-On\n",
				__func__);
			return 0;
		}
		ret = power_regulators(BT_CORE, POWER_ENABLE);
		update_pwr_state(ALL_CLIENTS_ON);
		break;
	case ALL_CLIENTS_ON:
		pr_err("%s: Both BT and UWB Regulators already Voted-On\n",
			__func__);
		return 0;
	}
	return ret;
}

void send_signal_to_subsystem(int SubSystemType, int state)
{
	pwr_data->wrkq_signal_state = state;
	if (SubSystemType == BLUETOOTH) {
		if (!pwr_data->reftask_bt) {
			pr_err("%s: BT client is not register to send signal\n",
				__func__);
			return;
		}
		queue_work(pwr_data->workq, &pwr_data->bt_wq);
	} else {
		if (!pwr_data->reftask_uwb) {
			pr_err("%s: UWB client is not register to send signal\n",
				__func__);
			return;
		}
		queue_work(pwr_data->workq, &pwr_data->uwb_wq);
	}
}

int power_disable (enum SubSystem SubSystemType)
{
	int ret = 0;
	int ret_mode_state = btpower_get_retenion_mode_state();
	enum grant_states grant_state = btpower_get_grant_state();
	enum grant_states grant_pending = btpower_get_grant_pending_state();

	switch (get_pwr_state()) {
	case IDLE:
		pr_err("%s: both BT and UWB regulators already voted-Off\n", __func__);
		return 0;
	case ALL_CLIENTS_ON:
		if (SubSystemType == BLUETOOTH) {
			ret = power_regulators(BT_CORE, POWER_DISABLE);
			update_pwr_state(UWB_ON);
			if (ret_mode_state == BOTH_CLIENTS_IN_RETENTION)
				btpower_set_retenion_mode_state(UWB_IN_RETENTION);
			else if (ret_mode_state == BT_IN_RETENTION)
				btpower_set_retenion_mode_state(RETENTION_IDLE);
			if(get_sub_state() == SSR_ON_BT) {
				send_signal_to_subsystem(UWB, BT_SSR_COMPLETED);
			}
			if (grant_state == BT_HAS_GRANT) {
				if (grant_pending == UWB_WAITING_FOR_GRANT) {
					send_signal_to_subsystem(UWB, SIGIO_SOC_ACCESS_SIGNAL|(ACCESS_GRANTED + 1));
					btpower_set_grant_state(UWB_HAS_GRANT);
				} else {
					btpower_set_grant_state(NO_GRANT_FOR_ANY_SS);
				}
			}
			if (grant_pending == BT_WAITING_FOR_GRANT)
				btpower_set_grant_pending_state(NO_OTHER_CLIENT_WAITING_FOR_GRANT);
		} else {
			ret  = power_regulators(UWB_CORE, POWER_DISABLE);
			update_pwr_state(BT_ON);
			if (ret_mode_state == BOTH_CLIENTS_IN_RETENTION)
				btpower_set_retenion_mode_state(BT_IN_RETENTION);
			else if (ret_mode_state == UWB_IN_RETENTION)
				btpower_set_retenion_mode_state(RETENTION_IDLE);
			if(get_sub_state() == SSR_ON_UWB) {
				send_signal_to_subsystem(BLUETOOTH,
					(SIGIO_NOTIFICATION_SIGNAL|SIGIO_UWB_SSR_COMPLETED));
			}
			if (grant_state == UWB_HAS_GRANT) {
				if (grant_pending == BT_WAITING_FOR_GRANT) {
					send_signal_to_subsystem(BLUETOOTH, SIGIO_SOC_ACCESS_SIGNAL|(ACCESS_GRANTED + 1));
					btpower_set_grant_state(BT_HAS_GRANT);
				} else {
					btpower_set_grant_state(NO_GRANT_FOR_ANY_SS);
				}
			}
			if (grant_pending == UWB_WAITING_FOR_GRANT)
				btpower_set_grant_pending_state(NO_OTHER_CLIENT_WAITING_FOR_GRANT);
		}
		break;
	case UWB_ON:
		if (SubSystemType == BLUETOOTH) {
			pr_err("%s: BT Regulator already Voted-Off\n", __func__);
			return 0;
		}
		ret = power_regulators(UWB_CORE, POWER_DISABLE);
		ret = power_regulators(PLATFORM_CORE, POWER_DISABLE);
		update_pwr_state(IDLE);
		update_sub_state(SUB_STATE_IDLE);
		btpower_set_retenion_mode_state(RETENTION_IDLE);
		btpower_set_grant_state(NO_GRANT_FOR_ANY_SS);
		btpower_set_grant_pending_state(NO_OTHER_CLIENT_WAITING_FOR_GRANT);
		break;
	case BT_ON:
		if (SubSystemType == UWB) {
			pr_err("%s: UWB Regulator already Voted-Off\n", __func__);
			return 0;
		}
		ret = power_regulators(BT_CORE, POWER_DISABLE);
		ret = power_regulators(PLATFORM_CORE, POWER_DISABLE);
		update_pwr_state(IDLE);
		update_sub_state(SUB_STATE_IDLE);
		btpower_set_retenion_mode_state(RETENTION_IDLE);
		btpower_set_grant_state(NO_GRANT_FOR_ANY_SS);
		btpower_set_grant_pending_state(NO_OTHER_CLIENT_WAITING_FOR_GRANT);
		break;
	}
	return ret;
}

static int client_state_notified(int SubSystemType)
{
	if (get_sub_state() != SUB_STATE_IDLE) {
		pr_err("%s: SSR is already running on other Sub-system\n", __func__);
		return -1;
	}

	if (SubSystemType == BLUETOOTH) {
		update_sub_state(SSR_ON_BT);
		if (get_pwr_state() == ALL_CLIENTS_ON)
			send_signal_to_subsystem(UWB, SSR_ON_BT);
	} else {
		update_sub_state(SSR_ON_UWB);
		if (get_pwr_state() == ALL_CLIENTS_ON)
			send_signal_to_subsystem(BLUETOOTH,
				(SIGIO_NOTIFICATION_SIGNAL|SIGIO_SSR_ON_UWB));
	}
	return 0;
}

void btpower_register_client(int client, int cmd)
{
	if (cmd == REG_BT_PID) {
		pwr_data->reftask_bt = get_current();
		pr_info("%s: Registering BT Service(PID-%d) with Power driver\n",
			__func__, pwr_data->reftask_bt->tgid);
		return;
	} else if (cmd == REG_UWB_PID) {
		pwr_data->reftask_uwb = get_current();
		pr_info("%s: Registering UWB Service(PID-%d) with Power driver\n",
			__func__, pwr_data->reftask_uwb->tgid);
		return;
	}

	if (client == BLUETOOTH)
		client_state_notified(BLUETOOTH);
	else
		client_state_notified(UWB);
}

void log_power_src_val(void)
{
	int itr = 0;

	power_src.platform_state[BT_SW_CTRL_GPIO_CURRENT] =
		gpio_get_value(pwr_data->bt_gpio_sw_ctrl);
	power_src.platform_state[BT_RESET_GPIO_CURRENT] =
		gpio_get_value(pwr_data->bt_gpio_sys_rst);

	for (itr = 0; itr < pwr_data->bt_num_vregs; itr++)
		set_pwr_srcs_status(&pwr_data->bt_vregs[itr], BT_CORE);

	for (itr = 0; itr < pwr_data->platform_num_vregs; itr++)
		set_pwr_srcs_status(&pwr_data->platform_vregs[itr], PLATFORM_CORE);

	for (itr = 0; itr < pwr_data->uwb_num_vregs; itr++)
		set_pwr_srcs_status(&pwr_data->uwb_vregs[itr], UWB_CORE);
}


int btpower_retenion(enum plt_pwr_state client)
{
	int ret;
	int current_pwr_state = get_pwr_state();
	int retention_mode_state = btpower_get_retenion_mode_state();

	if (current_pwr_state == IDLE) {
		pr_err("%s: invalid retention_mode request", __func__);
		return -1;
	}
	
	ret = power_regulators((client == POWER_ON_BT_RETENION ? BT_CORE : UWB_CORE),
				POWER_RETENTION);
	if (ret < 0)
		return ret;

	if ((current_pwr_state == BT_ON || current_pwr_state == UWB_ON) &&
	     retention_mode_state == IDLE) {
		ret = power_regulators(PLATFORM_CORE, POWER_RETENTION);
		if (ret < 0)
			return ret;
		btpower_set_retenion_mode_state(client == POWER_ON_BT_RETENION ? BT_IN_RETENTION: UWB_IN_RETENTION);
	} else if (current_pwr_state == ALL_CLIENTS_ON && retention_mode_state == IDLE) {
		btpower_set_retenion_mode_state(client == POWER_ON_BT_RETENION ? BT_IN_RETENTION: UWB_IN_RETENTION);
	} else if (current_pwr_state == ALL_CLIENTS_ON &&
		   (retention_mode_state == BT_IN_RETENTION ||
		    retention_mode_state == UWB_IN_RETENTION)) {
		ret = power_regulators(PLATFORM_CORE, POWER_RETENTION);
		if (ret < 0)
			return ret;
		btpower_set_retenion_mode_state(BOTH_CLIENTS_IN_RETENTION);
	} else if (retention_mode_state == UWB_OUT_OF_RETENTION ||
		   retention_mode_state == BT_OUT_OF_RETENTION) {
		ret = power_regulators(PLATFORM_CORE, POWER_RETENTION);
		if (ret < 0)
			return ret;
		btpower_set_retenion_mode_state(BOTH_CLIENTS_IN_RETENTION);
	}

	return ret;
}

int btpower_off(enum plt_pwr_state client)
{
	return power_disable((client == POWER_OFF_BT) ? BLUETOOTH : UWB);
}

int btpower_on(enum plt_pwr_state client)
{
	int ret = 0;
	int current_ssr_state = get_sub_state();
	int retention_mode_state = btpower_get_retenion_mode_state();

	if (retention_mode_state == UWB_IN_RETENTION ||
	    retention_mode_state == BT_IN_RETENTION) {
		ret = platform_regulators_pwr(POWER_DISABLE_RETENTION);
		if (ret < 0)
			return ret;
		if (retention_mode_state == BT_IN_RETENTION) 
			btpower_set_retenion_mode_state(BT_OUT_OF_RETENTION);
		else
			btpower_set_retenion_mode_state(UWB_OUT_OF_RETENTION);
	}

	/* No Point in going further if SSR is on any subsystem */
	if (current_ssr_state != SUB_STATE_IDLE) {
		pr_err("%s: %s not allowing to power on\n", __func__,
			ConvertSsrStatusToString(current_ssr_state));
		return -1;
	}

	ret = power_enable(client == POWER_ON_BT ? BLUETOOTH : UWB);

	/* Return current state machine to clients */
	if (!ret) {
		ret = (int)get_pwr_state();
	}
	return ret;
}

int STREAM_TO_UINT32 (struct sk_buff *skb)
{
	return (skb->data[0] | (skb->data[1] << 8) |
		(skb->data[2] << 16) | (skb->data[3] << 24));
}

int btpower_access_ctrl(enum plt_pwr_state request)
{
	enum grant_states grant_state = btpower_get_grant_state();
	enum grant_states grant_pending = btpower_get_grant_pending_state();
	int current_ssr_state = get_sub_state();

	if (current_ssr_state != SUB_STATE_IDLE &&
		(request == BT_ACCESS_REQ || request == UWB_ACCESS_REQ)) {
		pr_err("%s: not allowing this request as %s\n", __func__,
			ConvertSsrStatusToString(current_ssr_state));
		return (int)ACCESS_DISALLOWED;
	}

	if ((grant_state == NO_GRANT_FOR_ANY_SS &&
		grant_pending != NO_OTHER_CLIENT_WAITING_FOR_GRANT)) {
		pr_err("%s: access ctrl gone for toss, reseting it back", __func__ );
		grant_pending = NO_OTHER_CLIENT_WAITING_FOR_GRANT;
		btpower_set_grant_pending_state(NO_OTHER_CLIENT_WAITING_FOR_GRANT);
	}

	if (request == BT_ACCESS_REQ && grant_state == NO_GRANT_FOR_ANY_SS) {
		btpower_set_grant_state(BT_HAS_GRANT);
		return ACCESS_GRANTED;
	} else if (request == UWB_ACCESS_REQ && grant_state == NO_GRANT_FOR_ANY_SS) {
		btpower_set_grant_state(UWB_HAS_GRANT);
		return ACCESS_GRANTED;
	} else if (request == BT_ACCESS_REQ && grant_state == UWB_HAS_GRANT) {
		btpower_set_grant_pending_state(BT_WAITING_FOR_GRANT);
		return ACCESS_DENIED;
	} else if (request == UWB_ACCESS_REQ && grant_state == BT_HAS_GRANT) {
		btpower_set_grant_pending_state(UWB_WAITING_FOR_GRANT);
		return ACCESS_DENIED;
	} else if (request == BT_RELEASE_ACCESS && grant_state == BT_HAS_GRANT) {
		if (grant_pending == UWB_WAITING_FOR_GRANT) {
			if (!pwr_data->reftask_uwb) {
				pr_err("%s: UWB service got killed\n", 	__func__);
			} else {
				send_signal_to_subsystem(UWB, SIGIO_SOC_ACCESS_SIGNAL|(ACCESS_GRANTED + 1));
				btpower_set_grant_state(UWB_HAS_GRANT);
			}
			btpower_set_grant_pending_state(NO_OTHER_CLIENT_WAITING_FOR_GRANT);
			return ACCESS_RELEASED;
			
		} else {
			btpower_set_grant_state(NO_GRANT_FOR_ANY_SS);
			btpower_set_grant_pending_state(NO_OTHER_CLIENT_WAITING_FOR_GRANT);
			return ACCESS_RELEASED; 
		}
	} else if (request == UWB_RELEASE_ACCESS && grant_state == UWB_HAS_GRANT) {
		if (grant_pending == BT_WAITING_FOR_GRANT) {
			if (!pwr_data->reftask_uwb) {
				pr_err("%s: BT service got killed\n", 	__func__);
			} else {
				send_signal_to_subsystem(BLUETOOTH, SIGIO_SOC_ACCESS_SIGNAL|(ACCESS_GRANTED+1));
				btpower_set_grant_state(BT_HAS_GRANT);
			}
		} else {
			btpower_set_grant_state(NO_GRANT_FOR_ANY_SS);
		}
		btpower_set_grant_pending_state(NO_OTHER_CLIENT_WAITING_FOR_GRANT);
		return ACCESS_RELEASED;
	} else {
		pr_err("%s: unhandled event", __func__);
	}
	return ACCESS_DISALLOWED;
}

static void bt_power_vote(struct work_struct *work)
{
	struct sk_buff *skb;
	int request;
	int ret;

	while (1) {
		mutex_lock(&pwr_data->pwr_mtx);
		if (!(skb = skb_dequeue(&pwr_data->rxq))) {
			mutex_unlock(&pwr_data->pwr_mtx);
			break;
		}
		request = STREAM_TO_UINT32(skb);
		skb_pull(skb, sizeof(uint32_t));
		mutex_unlock(&pwr_data->pwr_mtx);
		pr_info("%s: Start %s %s, %s state access %s pending %s\n",
			__func__,
			ConvertPowerStatusToString(get_pwr_state()),
			ConvertSsrStatusToString(get_sub_state()),
			ConvertRetentionModeToString(btpower_get_retenion_mode_state()),
			ConvertGrantToString(btpower_get_grant_state()),
			ConvertGrantToString(btpower_get_grant_pending_state()));
		if (request == POWER_ON_BT || request == POWER_ON_UWB)
			ret = btpower_on((enum plt_pwr_state)request);
		else if (request == POWER_OFF_UWB || request == POWER_OFF_BT)
			ret = btpower_off((enum plt_pwr_state)request);
		else if (request == POWER_ON_BT_RETENION || request == POWER_ON_UWB_RETENION)
			ret = btpower_retenion(request);
		else if (request >= BT_ACCESS_REQ && request <= UWB_RELEASE_ACCESS) {
			ret = btpower_access_ctrl(request);
			pr_info("%s: grant status %s", __func__, ConvertGrantRetToString((int)ret));
		}
		pr_info("%s: Completed %s %s, %s state access %s pending %s\n",
			__func__,
			ConvertPowerStatusToString(get_pwr_state()),
			ConvertSsrStatusToString(get_sub_state()),
			ConvertRetentionModeToString(btpower_get_retenion_mode_state()),
			ConvertGrantToString(btpower_get_grant_state()),
			ConvertGrantToString(btpower_get_grant_pending_state()));

		pwr_data->wait_status[request] = ret;
		kfree_skb(skb);
		wake_up_interruptible(&pwr_data->rsp_wait_q[request]);
	}
}

int schedule_client_voting(enum plt_pwr_state request)
{
	struct sk_buff *skb;
	wait_queue_head_t *rsp_wait_q;
	int *status;
	int ret = 0;
	uint32_t req = (uint32_t)request;

	mutex_lock(&pwr_data->pwr_mtx);
	skb = alloc_skb(sizeof(uint32_t), GFP_KERNEL);

	if (!skb) {
		pr_err("%s: Unable to create skbuff\n", __func__);
		mutex_unlock(&pwr_data->pwr_mtx);
		return -1;
	}

	rsp_wait_q = &pwr_data->rsp_wait_q[(u8)request];
	status = &pwr_data->wait_status[(u8)request];
	*status = PWR_WAITING_RSP;
	skb_put_data(skb, &req, sizeof(uint32_t));
	skb_queue_tail(&pwr_data->rxq, skb);
	queue_work(system_highpri_wq, &pwr_data->wq_pwr_voting);
	mutex_unlock(&pwr_data->pwr_mtx);
	ret = wait_event_interruptible_timeout(*rsp_wait_q, (*status) != PWR_WAITING_RSP,
					       msecs_to_jiffies(BTPOWER_CONFIG_MAX_TIMEOUT));
	pr_err("%s: %d", __func__, *status);
	if (ret == 0) {
		pr_err("%s: failed to vote %d due to timeout", __func__, request);
		ret = -ETIMEDOUT;
	} else {
		ret = *status;
	}

	return ret;
}

int btpower_handle_client_request(unsigned int cmd, int arg)
{
	int ret = -1;

	pr_info("%s: Start of %s cmd request to %s.\n",
		__func__,
		(cmd == BT_CMD_PWR_CTRL ? "BT_CMD_PWR_CTRL" : "UWB_CMD_PWR_CTRL"),
		ConvertClientReqToString(arg));

	if (cmd == BT_CMD_PWR_CTRL) {
		switch ((int)arg) {
		case POWER_DISABLE:
			ret = schedule_client_voting(POWER_OFF_BT);
			break;
		case POWER_ENABLE:
			ret = schedule_client_voting(POWER_ON_BT);
			break;
		case POWER_RETENTION:
			ret = schedule_client_voting(POWER_ON_BT_RETENION);
			break;
		}
	} else if (cmd == UWB_CMD_PWR_CTRL) {
		switch ((int)arg) {
		case POWER_DISABLE:
			ret = schedule_client_voting(POWER_OFF_UWB);
			break;
		case POWER_ENABLE:
			ret = schedule_client_voting(POWER_ON_UWB);
			break;
		case POWER_RETENTION:
			ret = schedule_client_voting(POWER_ON_UWB_RETENION);
			break;
		}
	}


	return ret;
}

int btpower_process_access_req(unsigned int cmd, int req)
{
	int ret = -1;

	pr_info("%s: by %s: request type %s\n", __func__,
		cmd == BT_CMD_ACCESS_CTRL ? "BT_CMD_ACCESS_CTRL" : "UWB_CMD_ACCESS_CTRL",
		req == 1 ? "Request" : "Release");
	if (cmd == BT_CMD_ACCESS_CTRL && req == 1)
		ret = schedule_client_voting(BT_ACCESS_REQ);
	else if (cmd == BT_CMD_ACCESS_CTRL && req == 2)
		ret = schedule_client_voting(BT_RELEASE_ACCESS);
	else if (cmd == UWB_CMD_ACCESS_CTRL && req == 1)
		ret = schedule_client_voting(UWB_ACCESS_REQ);
	else if (cmd == UWB_CMD_ACCESS_CTRL && req == 2)
		ret = schedule_client_voting(UWB_RELEASE_ACCESS);
	else 
		pr_err("%s: unhandled command %04x req %02x", __func__, cmd, req);

	return ret;
}

char* GetUwbSecondaryCrashReason(enum UwbSecondaryReasonCode reason)
{
  for(int i =0; i < (int)(sizeof(uwbSecReasonMap)/sizeof(UwbSecondaryReasonMap)); i++)
    if (uwbSecReasonMap[i].reason == reason)
      return uwbSecReasonMap[i].reasonstr;

  return CRASH_REASON_NOT_FOUND;
}

char* GetUwbPrimaryCrashReason(enum UwbPrimaryReasonCode reason)
{
  for(int i =0; i < (int)(sizeof(uwbPriReasonMap)/sizeof(UwbPrimaryReasonMap)); i++)
    if (uwbPriReasonMap[i].reason == reason)
      return uwbPriReasonMap[i].reasonstr;

  return CRASH_REASON_NOT_FOUND;
}

const char *GetSourceSubsystemString(uint32_t source_subsystem)
{
	switch (source_subsystem) {
	case PERI_SS:
		return "Peri SS";
	case BT_SS:
		return "BT SS";
	case UWB_SS:
		return "UWB SS";
	default:
		return "Unknown Subsystem";
	}
}

void fmd_set_sdam_bit(unsigned char arg)
{
	int rc = 0;
	u8 *buf;
	size_t len;
	struct device *devi = &pwr_data->pdev->dev;

	if (IS_ERR(pwr_data->nvmem_cell_fmd_set)) {
		pr_err("%s: 'fmd_set' cell is not avilable to configure\n",
			__func__);
		return;
	}

	rc = nvmem_cell_write(pwr_data->nvmem_cell_fmd_set,
						 &arg,
						 sizeof(arg));
	if (rc < 0) {
		pr_err("%s: Write SDAM BIT for FMD operation Failed %d\n",
			__func__, rc);
		return;
	}

	buf = nvmem_cell_read(pwr_data->nvmem_cell_fmd_set, &len);
	if (IS_ERR(buf)) {
		dev_err(devi, "Failed to read fmd_set: %ld\n",
			PTR_ERR(buf));
		pr_err("%s: Failed to read SDAM BIT (fmd_set = %d)\n",
			__func__, buf[0]);
		kfree(buf);
		return;
	}

	if (buf[0] == arg) {
		dev_info(devi, "Successfully configured the fmd_set\n");
		pr_info("%s: Successfully configured SDAM BIT (fmd_set: %u)\n",
			__func__, buf[0]);
	} else {
		dev_err(devi, "Failed to configure fmd_set: %ld\n",
			PTR_ERR(buf));
		pr_err("%s: Failed to configure SDAM BIT (fmd_set = %u)\n",
			__func__, buf[0]);
	}
	kfree(buf);
}

void fmd_reboot_on_usb_detection(unsigned char arg)
{
	int rc = 0;
	u8 *buf;
	size_t len;
	struct device *devi = &pwr_data->pdev->dev;

	if (IS_ERR(pwr_data->nvmem_cell_fmd_chg_pon)) {
		pr_err("%s: 'fmd_chg_pon' cell is not avilable to configure\n",
			__func__);
		return;
	}

	rc = nvmem_cell_write(pwr_data->nvmem_cell_fmd_chg_pon,
						&arg,
						sizeof(arg));
	if (rc < 0) {
		pr_err("%s: Write fmd_chg_pon for FMD operation Failed %d\n",
			__func__, rc);
		return;
	}

	buf = nvmem_cell_read(pwr_data->nvmem_cell_fmd_chg_pon, &len);
	if (IS_ERR(buf)) {
		dev_err(devi, "Failed to read fmd_chg_pon: %ld\n",
			PTR_ERR(buf));
		pr_err("%s: Failed to read fmd_chg_pon = %d\n",
			__func__, buf[0]);
		kfree(buf);
		return;
	}

	if (buf[0] == arg) {
		dev_info(devi, "Successfully configured the fmd_chg_pon\n");
		pr_info("%s: Successfully configured (fmd_chg_pon: %u)\n",
			__func__, buf[0]);
	} else {
		dev_err(devi, "Failed to configure fmd_chg_pon: %ld\n",
			PTR_ERR(buf));
		pr_err("%s: Failed to configure fmd_chg_pon = %u\n",
			__func__, buf[0]);
	}
	kfree(buf);
}

void fmd_write_stop_counter(unsigned char arg)
{
	int rc = 0;
	u8 *buf;
	size_t len;
	struct device *devi = &pwr_data->pdev->dev;

	if (IS_ERR(pwr_data->nvmem_cell_fmd_cnt2_stop)) {
		pr_err("%s: 'fmd_cnt2_stop' cell is not avilable to configure\n",
			__func__);
		return;
	}

	rc = nvmem_cell_write(pwr_data->nvmem_cell_fmd_cnt2_stop,
						&arg,
						sizeof(arg));
	if (rc < 0) {
		pr_err("%s: Write cnt2_stop for FMD operation Failed %d\n",
			__func__, rc);
		return;
	}

	buf = nvmem_cell_read(pwr_data->nvmem_cell_fmd_cnt2_stop, &len);
	if (IS_ERR(buf)) {
		dev_err(devi, "Failed to read fmd_cnt2_stop: %ld\n",
			PTR_ERR(buf));
		pr_err("%s: Failed to read fmd_cnt2_stop = %d\n",
			__func__, buf[0]);
		kfree(buf);
		return;
	}

	if (buf[0] == arg) {
		dev_info(devi, "Successfully configured the fmd_cnt2_stop\n");
		pr_info("%s: Successfully configured (fmd_cnt2_stop: %u)\n",
			__func__, buf[0]);
	} else {
		dev_err(devi, "Failed to configure fmd_cnt2_stop: %ld\n",
			PTR_ERR(buf));
		pr_err("%s: Failed to configure fmd_cnt2_stop = %u\n",
			__func__, buf[0]);
	}
	kfree(buf);
}

int perform_fmd_operation(void)
{
	int ret = 0;
	switch ((enum FmdOperation) fmdStruct.fmdOperation) {
		case UPDATE_SOC_VER: {
			if ((fmdStruct.socFwVer == INVALID_SOC) ||
				(fmdStruct.socFwVer > OTHER_FMD_SUPPORTED_BT_SOC)) {
				pr_err("%s: Invalid SOC VERSION sent = %d\n",
					__func__, fmdStruct.socFwVer);
				return -EINVAL;
			}
			pwr_data->is_fmd_mode_enable = true;
			if (fmdStruct.socFwVer == PEACH_SOC_VERSION_1_0) {
				pr_info("%s: UPDATE_SOC_VER :: PEACH_SOC_VER_1_0\n",
					__func__);
			} else if (fmdStruct.socFwVer == PEACH_SOC_VERSION_2_0) {
				pr_info("%s: UPDATE_SOC_VER :: PEACH_SOC_VERSION_2_0\n",
					__func__);
#ifdef CONFIG_FMD_ENABLE
				cnss_utils_fmd_status(true);
#endif
				if (vote_wlan_reg_for_fmd() < 0) {
					pr_err("%s: failed to vote wlan_reg\n", __func__);
					return -EINVAL;
				}
			} else {
				pr_info("%s: UPDATE_SOC_VER :: OTHER_FMD_SUPPORT_BT_SOC\n",
					__func__);
			}

			if (pwr_data->bt_chip_clk) {
				ret = bt_clk_enable(pwr_data->bt_chip_clk);
				if (ret < 0) {
					pr_err("%s: failed to bt_chip_clk\n", __func__);
					return -EINVAL;
				}
			}
			break;
		}
		case ENABLE_FMD: {
			pr_info("%s: ENABLE_FMD\n", __func__);

			fmd_set_sdam_bit((unsigned char)POWER_ENABLE);

			if (fmdStruct.rebootStatus != -1)
				fmd_reboot_on_usb_detection((unsigned char)fmdStruct.rebootStatus);
			else
				pr_err("%s: Reboot status upon usb detection is not configured\n",
					__func__);

			if (fmdStruct.fmdCycles != -1)
				fmd_write_stop_counter((unsigned char)fmdStruct.fmdCycles);
			else
				pr_err("%s: Fmd stop_counter is not configured\n", __func__);

			break;
		}
		case DISABLE_FMD: {
			pr_info("%s: DISABLE_FMD\n", __func__);
			pwr_data->is_fmd_mode_enable = false;
			break;
		}
		default: {
			pr_err("%s: invalid fmd operation received = %d\n",
				__func__, fmdStruct.fmdOperation);
			ret = -EINVAL;
			break;
		}
	}
	return ret;
}

int bt_kernel_panic(char *arg) {
	int ret = 0;

	pr_info("%s\n", __func__);

	if (copy_from_user(&CrashInfo, (char *)arg, sizeof(CrashInfo))) {
		pr_err("%s: failed copy to panic reason from BT-Transport\n",
			__func__);
		memset(&CrashInfo, 0, sizeof(CrashInfo));
		strscpy(CrashInfo. PrimaryReason,
			default_crash_reason, strlen(default_crash_reason));
		strscpy(CrashInfo. SecondaryReason,
			default_crash_reason, strlen(default_crash_reason));
		ret = -EFAULT;
	}

	pr_err("%s: BT kernel panic Primary reason = %s, Secondary reason = %s\n",
		__func__, CrashInfo.PrimaryReason, CrashInfo.SecondaryReason);

	panic("%s: BT kernel panic Primary reason = %s, Secondary reason = %s\n",
		__func__, CrashInfo.PrimaryReason, CrashInfo.SecondaryReason);

	return ret;
}

#ifdef CONFIG_MSM_BT_OOBS
int bt_oobs_handler(enum btpower_obs_param clk_cntrl)
{
	if (!gpio_is_valid(pwr_data->bt_gpio_dev_wake)) {
		pr_debug("%s: BT_CMD_OBS_VOTE_CLOCK bt_dev_wake_n(%d) not configured\n",
			__func__, pwr_data->bt_gpio_dev_wake);
		return -EIO;
	}

	switch (clk_cntrl) {
	case BTPOWER_OBS_CLK_OFF:
		btpower_uart_transport_locked(pwr_data, false);
		break;
	case BTPOWER_OBS_CLK_ON:
		btpower_uart_transport_locked(pwr_data, true);
		break;
	case BTPOWER_OBS_DEV_OFF:
		gpio_set_value(pwr_data->bt_gpio_dev_wake, 0);
		break;
	case BTPOWER_OBS_DEV_ON:
		gpio_set_value(pwr_data->bt_gpio_dev_wake, 1);
		break;
	default:
		pr_debug("%s: BT_CMD_OBS_VOTE_CLOCK clk_cntrl(%d)\n",
			__func__, clk_cntrl);
		return -EINVAL;
	}
	pr_debug("%s: BT_CMD_OBS_VOTE_CLOCK clk_cntrl(%d) %s\n",
		__func__, clk_cntrl,
		gpio_get_value(pwr_data->bt_gpio_dev_wake) ?
			"Assert" : "Deassert");
	return 0;
}
#endif

static long bt_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int chipset_version = 0;
	unsigned long panic_reason = 0;
	unsigned short primary_reason = 0, sec_reason = 0, source_subsystem = 0;
	int current_ssr_state = SUB_STATE_IDLE;

	if (!pwr_data || !probe_finished) {
		pr_err("%s: BTPower Probing Pending.Try Again\n", __func__);
		return -EAGAIN;
	}

	switch (cmd) {
#ifdef CONFIG_MSM_BT_OOBS
	case BT_CMD_OBS_VOTE_CLOCK:
		ret = bt_oobs_handler((enum btpower_obs_param)arg);
		break;
#endif
	case BT_CMD_SLIM_TEST:
#if (defined CONFIG_BT_SLIM)
		if (!pwr_data->slim_dev) {
			pr_err("%s: slim_dev is null\n", __func__);
			return -EINVAL;
		}
		ret = btfm_slim_hw_init(
			pwr_data->slim_dev->platform_data
		);
#endif
		break;
	case BT_CMD_PWR_CTRL:
	case UWB_CMD_PWR_CTRL: {
		ret = btpower_handle_client_request(cmd, (int)arg);
		break;
	}
	case BT_CMD_FMD_OPERATION: {
		pr_err("%s: BT_CMD_FMD_OPERATION\n", __func__);
		if (copy_from_user(&fmdStruct, (char *)arg, sizeof(fmdStruct))) {
			pr_err("%s: copy to user failed\n", __func__);
			ret = -EFAULT;
		}
		ret = perform_fmd_operation();
		break;
	}
	case BT_CMD_REGISTRATION:
		btpower_register_client(BLUETOOTH, (int)arg);
		break;
	case UWB_CMD_REGISTRATION:
		btpower_register_client(UWB, (int)arg);
		break;
	case BT_CMD_ACCESS_CTRL:
	case UWB_CMD_ACCESS_CTRL: {
		ret = btpower_process_access_req(cmd, (int)arg);
		break;
	}	  
	case BT_CMD_CHIPSET_VERS:
		chipset_version = (int)arg;
		pr_warn("%s: unified Current SOC Version : %x\n", __func__,
			chipset_version);
		if (chipset_version) {
			soc_id = chipset_version;
		} else {
			pr_err("%s: got invalid soc version\n", __func__);
			soc_id = 0;
		}
		break;
	case BT_CMD_GET_CHIPSET_ID:
		pr_err("%s: BT_CMD_GET_CHIPSET_ID = %s\n", __func__,
			pwr_data->compatible);
		if (copy_to_user((void __user *)arg, pwr_data->compatible,
		    MAX_PROP_SIZE)) {
			pr_err("%s: copy to user failed\n", __func__);
			ret = -EFAULT;
		}
		break;
	case BT_CMD_CHECK_SW_CTRL:
		/*  Check  if  SW_CTRL  is  asserted  */
		pr_info("BT_CMD_CHECK_SW_CTRL\n");
		if (pwr_data->bt_gpio_sw_ctrl > 0) {
			power_src.bt_state[BT_SW_CTRL_GPIO] =
				DEFAULT_INVALID_VALUE;
			ret  =  gpio_direction_input(
				pwr_data->bt_gpio_sw_ctrl);
			if (ret) {
				pr_err("%s:gpio_direction_input api\n",
					 __func__);
				pr_err("%s:failed for SW_CTRL:%d\n",
					__func__, ret);
			} else {
				power_src.bt_state[BT_SW_CTRL_GPIO] =
					gpio_get_value(
					pwr_data->bt_gpio_sw_ctrl);
				pr_info("bt-sw-ctrl-gpio(%d) value(%d)\n",
					pwr_data->bt_gpio_sw_ctrl,
					power_src.bt_state[BT_SW_CTRL_GPIO]);
			}
		} else {
			pr_err("bt_gpio_sw_ctrl not configured\n");
			return -EINVAL;
		}
		break;
	case BT_CMD_GETVAL_POWER_SRCS:
		pr_info("BT_CMD_GETVAL_POWER_SRCS\n");
		log_power_src_val();
		if (copy_to_user((void __user *)arg, &power_src, sizeof(power_src))) {
			pr_err("%s: copy to user failed\n", __func__);
			ret = -EFAULT;
		}
		break;
	case BT_CMD_SET_IPA_TCS_INFO:
		pr_info("%s: BT_CMD_SET_IPA_TCS_INFO\n", __func__);
		btpower_enable_ipa_vreg(pwr_data);
		break;
	case BT_CMD_KERNEL_PANIC:

		pr_err("%s: BT_CMD_KERNEL_PANIC\n", __func__);

		ret = bt_kernel_panic((char *)arg);

		break;
	case UWB_CMD_KERNEL_PANIC:
		pr_err("%s: UWB_CMD_KERNEL_PANIC\n", __func__);
		panic_reason = arg;
		primary_reason = panic_reason & 0xFFFF;
		sec_reason = (panic_reason & 0xFFFF0000) >> 16;
		source_subsystem = (panic_reason & 0xFFFF00000000) >> 32;
		pr_err("%s: UWB kernel panic PrimaryReason = (0x%02x)[%s] | SecondaryReason = (0x%02x)[%s] | SourceSubsystem = (0x%02x)[%s]\n",
			__func__, primary_reason, GetUwbPrimaryCrashReason(primary_reason),
			sec_reason, GetUwbSecondaryCrashReason(sec_reason),
			source_subsystem, GetSourceSubsystemString(source_subsystem));
		panic("%s: UWB kernel panic PrimaryReason = (0x%02x)[%s] | SecondaryReason = (0x%02x)[%s] | SourceSubsystem = (0x%02x)[%s]\n",
			__func__, primary_reason, GetUwbPrimaryCrashReason(primary_reason),
			sec_reason, GetUwbSecondaryCrashReason(sec_reason),
			source_subsystem, GetSourceSubsystemString(source_subsystem));
		break;
	case UWB_GET_SSR_STATE:
		current_ssr_state = get_sub_state();
		pr_err("%s: UWB_GET_SSR_STATE current_ssr_state:%d\n", __func__,
			current_ssr_state);
		if (copy_to_user((void __user *)arg, &current_ssr_state,
			sizeof(current_ssr_state))) {
			pr_err("%s: copy to user failed\n", __func__);
			ret = -EFAULT;
		}
		break;

	default:
		return -ENOIOCTLCMD;
	}
	return ret;
}
static struct platform_driver bt_power_driver = {
	.probe = bt_power_probe,
	.remove = bt_power_remove,
	.driver = {
		.name = "bt_power",
		.of_match_table = bt_power_match_table,
	},
};

static const struct file_operations bt_dev_fops = {
	.unlocked_ioctl = bt_ioctl,
	.compat_ioctl = bt_ioctl,
};

static int __init btpower_init(void)
{
	int ret = 0;

	probe_finished = false;
	ret = platform_driver_register(&bt_power_driver);
	if (ret) {
		pr_err("%s: platform_driver_register error: %d\n",
			__func__, ret);
		goto driver_err;
	}

	bt_major = register_chrdev(0, "bt", &bt_dev_fops);
	if (bt_major < 0) {
		pr_err("%s: failed to allocate char dev\n", __func__);
		ret = -1;
		goto chrdev_err;
	}

	bt_class = class_create("bt-dev");
	if (IS_ERR(bt_class)) {
		pr_err("%s: coudn't create class\n", __func__);
		ret = -1;
		goto class_err;
	}

	if (device_create(bt_class, NULL, MKDEV(bt_major, 0),
		NULL, "btpower") == NULL) {
		pr_err("%s: failed to allocate char dev\n", __func__);
		goto device_err;
	}
	return 0;

device_err:
	class_destroy(bt_class);
class_err:
	unregister_chrdev(bt_major, "bt");
chrdev_err:
	platform_driver_unregister(&bt_power_driver);
driver_err:
	return ret;
}

/**
 * bt_aop_send_msg: Sends json message to AOP using QMP
 * @plat_priv: Pointer to cnss platform data
 * @msg: String in json format
 *
 * AOP accepts JSON message to configure WLAN/BT resources. Format as follows:
 * To send VReg config: {class: wlan_pdc, ss: <pdc_name>,
 *                       res: <VReg_name>.<param>, <seq_param>: <value>}
 * To send PDC Config: {class: wlan_pdc, ss: <pdc_name>, res: pdc,
 *                      enable: <Value>}
 * QMP returns timeout error if format not correct or AOP operation fails.
 *
 * Return: 0 for success
 */
int bt_aop_send_msg(struct platform_pwr_data *plat_priv, char *mbox_msg)
{
	int ret = 0;
	ret = qmp_send(plat_priv->qmp, mbox_msg, BTPOWER_MBOX_MSG_MAX_LEN);
	if (ret < 0)
		pr_err("Failed to send AOP qmp  xmsg: %s\n", mbox_msg);
	else
		ret =0;
	return ret;

}

int bt_aop_pdc_reconfig(struct platform_pwr_data *pdata)
{
	unsigned int i;
	int ret;
	if (pdata->pdc_init_table_len <= 0 || !pdata->pdc_init_table)
		return 0;
	pr_debug("Setting PDC defaults\n");
	for (i = 0; i < pdata->pdc_init_table_len; i++) {
		ret = bt_aop_send_msg(pdata, (char *)pdata->pdc_init_table[i]);
		if (ret < 0)
			break;
	}
	return ret;
}

int btpower_aop_mbox_init(struct platform_pwr_data *pdata)
{
	int ret = 0;
// ret = of_find_property(pdata->pdev->dev.of_node, "qcom,qmp", NULL);
	pdata->qmp = qmp_get(&pdata->pdev->dev);
	if (IS_ERR(pdata->qmp)) {
		pr_err("%s: failed to get qmp\n", __func__);
		return PTR_ERR(pdata->qmp);
	}
	ret = of_property_read_string(pdata->pdev->dev.of_node,
				      "qcom,vreg_ipa",
				      &pdata->vreg_ipa);
	if (ret)
		pr_info("%s: vreg for iPA not configured\n", __func__);
	else
		pr_info("%s: qmp initialized\n", __func__);

	ret = bt_aop_pdc_reconfig(pdata);
	if (ret)
		pr_err("Failed to reconfig BT WLAN PDC, err = %d\n", ret);

	return 0;
}

static int btpower_aop_set_vreg_param(struct platform_pwr_data *pdata,
				   const char *vreg_name,
				   enum btpower_vreg_param param,
				   enum btpower_tcs_seq seq, int val)
{
	char mbox_msg[BTPOWER_MBOX_MSG_MAX_LEN];
	static const char * const vreg_param_str[] = {"v", "m", "e"};
	static const char *const tcs_seq_str[] = {"upval", "dwnval", "enable"};
	int ret = 0;

	if (param > BTPOWER_VREG_ENABLE || seq > BTPOWER_TCS_ALL_SEQ || !vreg_name)
		return -EINVAL;

	snprintf(mbox_msg, BTPOWER_MBOX_MSG_MAX_LEN,
		 "{class: wlan_pdc, res: %s.%s, %s: %d}", vreg_name,
		 vreg_param_str[param], tcs_seq_str[seq], val);

	pr_info("%s: sending AOP Mbox msg: %s\n", __func__, mbox_msg);
	ret = qmp_send(pdata->qmp, mbox_msg, BTPOWER_MBOX_MSG_MAX_LEN);
	if (ret < 0)
		pr_err("%s:Failed to send AOP mbox msg(%s), err(%d)\n",
					__func__, mbox_msg, ret);

	return ret;
}

static int btpower_enable_ipa_vreg(struct platform_pwr_data *pdata)
{
	int ret = 0;
	static bool config_done;

	if (config_done) {
		pr_info("%s: IPA Vreg already configured\n", __func__);
		return 0;
	}

	if (!pdata->vreg_ipa) {
		pr_info("%s: mbox/iPA vreg not configured\n", __func__);
	} else {
		ret = btpower_aop_set_vreg_param(pdata,
					       pdata->vreg_ipa,
					       BTPOWER_VREG_ENABLE,
					       BTPOWER_TCS_UP_SEQ, 1);
		if (ret >=  0) {
			pr_info("%s:Enabled iPA\n", __func__);
			config_done = true;
		}
	}

	return ret;
}

static void __exit btpower_exit(void)
{
	qmp_put(pwr_data->qmp);
	platform_driver_unregister(&bt_power_driver);
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MSM Bluetooth power control driver");

module_init(btpower_init);
module_exit(btpower_exit);
