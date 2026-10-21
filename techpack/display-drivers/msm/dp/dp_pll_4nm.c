// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021-2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/*
 * Display Port PLL driver block diagram for branch clocks
 *
 * +------------------------+       +------------------------+
 * |   dp_phy_pll_link_clk  |       | dp_phy_pll_vco_div_clk |
 * +------------------------+       +------------------------+
 *             |                               |
 *             |                               |
 *             V                               V
 *        dp_link_clk                     dp_pixel_clk
 *
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/regmap.h>
#include "clk-regmap-mux.h"
#include "dp_hpd.h"
#include "dp_debug.h"
#include "dp_pll.h"

#define DP_PHY_CFG				0x0010
#define DP_PHY_CFG_1				0x0014
#define DP_PHY_PD_CTL				0x0018
#define DP_PHY_MODE				0x001C

#define DP_PHY_AUX_CFG1				0x0024
#define DP_PHY_AUX_CFG2				0x0028

#define DP_PHY_TX0_TX1_LANE_CTL			0x0078
#define DP_PHY_TX2_TX3_LANE_CTL			0x009C

#define DP_PHY_SPARE0				0x00C8
#define DP_PHY_STATUS				0x00E4

/* Tx registers */
#define TXn_CLKBUF_ENABLE			0x0008
#define TXn_TX_EMP_POST1_LVL			0x000C

#define TXn_TX_DRV_LVL				0x0014

#define TXn_RESET_TSYNC_EN			0x001C
#define TXn_PRE_STALL_LDO_BOOST_EN		0x0020
#define TXn_TX_BAND				0x0024
#define TXn_INTERFACE_SELECT			0x002C

#define TXn_RES_CODE_LANE_OFFSET_TX		0x003C
#define TXn_RES_CODE_LANE_OFFSET_RX		0x0040

#define TXn_TRANSCEIVER_BIAS_EN			0x0054
#define TXn_HIGHZ_DRVR_EN			0x0058
#define TXn_TX_POL_INV				0x005C
#define TXn_PARRATE_REC_DETECT_IDLE_EN		0x0060

/* PLL register offset */
#define QSERDES_COM_BG_TIMER			0x00BC
#define QSERDES_COM_SSC_EN_CENTER		0x00C0
#define QSERDES_COM_SSC_ADJ_PER1		0x00C4
#define QSERDES_COM_SSC_PER1			0x00CC
#define QSERDES_COM_SSC_PER2			0x00D0
#define QSERDES_COM_SSC_STEP_SIZE1_MODE0	0x0060
#define QSERDES_COM_SSC_STEP_SIZE2_MODE0	0X0064
#define QSERDES_COM_BIAS_EN_CLKBUFLR_EN		0x00DC
#define QSERDES_COM_CLK_ENABLE1			0x00E0
#define QSERDES_COM_SYS_CLK_CTRL		0x00E4
#define QSERDES_COM_SYSCLK_BUF_ENABLE		0x00E8
#define QSERDES_COM_PLL_IVCO			0x00F4

#define QSERDES_COM_CP_CTRL_MODE0		0x0070
#define QSERDES_COM_PLL_RCTRL_MODE0		0x0074
#define QSERDES_COM_PLL_CCTRL_MODE0		0x0078
#define QSERDES_COM_SYSCLK_EN_SEL		0x0110
#define QSERDES_COM_RESETSM_CNTRL		0x0118
#define QSERDES_COM_LOCK_CMP_EN			0x0120
#define QSERDES_COM_LOCK_CMP1_MODE0		0x0080
#define QSERDES_COM_LOCK_CMP2_MODE0		0x0084

#define QSERDES_COM_DEC_START_MODE0		0x0088
#define QSERDES_COM_DIV_FRAC_START1_MODE0	0x0090
#define QSERDES_COM_DIV_FRAC_START2_MODE0	0x0094
#define QSERDES_COM_DIV_FRAC_START3_MODE0	0x0098
#define QSERDES_COM_INTEGLOOP_GAIN0_MODE0	0x00A0
#define QSERDES_COM_INTEGLOOP_GAIN1_MODE0	0x00A4
#define QSERDES_COM_VCO_TUNE_CTRL		0x013C
#define QSERDES_COM_VCO_TUNE_MAP		0x0140

#define QSERDES_COM_CMN_STATUS			0x01D0
#define QSERDES_COM_CLK_SEL			0x0164
#define QSERDES_COM_HSCLK_SEL_1			0x003C

#define QSERDES_COM_CORECLK_DIV_MODE0		0x007C

#define QSERDES_COM_CORE_CLK_EN			0x0170
#define QSERDES_COM_C_READY_STATUS		0x01F8
#define QSERDES_COM_CMN_CONFIG_1		0x0174

#define QSERDES_COM_SVS_MODE_CLK_SEL		0x017C
#define QSERDES_COM_BIN_VCOCAL_CMP_CODE1_MODE0  0x0058
#define QSERDES_COM_BIN_VCOCAL_CMP_CODE2_MODE0  0x005C
/* Tx tran offsets */
#define DP_TRAN_DRVR_EMP_EN			0x00C0
#define DP_TX_INTERFACE_MODE			0x00C4

/* Tx VMODE offsets */
#define DP_VMODE_CTRL1				0x00C8

#define DP_PHY_PLL_POLL_SLEEP_US		500
#define DP_PHY_PLL_POLL_TIMEOUT_US		10000

#define DP_VCO_RATE_8100MHZDIV1000		8100000UL
#define DP_VCO_RATE_9720MHZDIV1000		9720000UL
#define DP_VCO_RATE_10800MHZDIV1000		10800000UL

#define DP_PLL_NUM_CLKS				2

#define DP_4NM_C_READY		BIT(0)
#define DP_4NM_FREQ_DONE	BIT(0)
#define DP_4NM_PLL_LOCKED	BIT(1)
#define DP_4NM_PHY_READY	BIT(1)
#define DP_4NM_TSYNC_DONE	BIT(0)

static const struct dp_pll_params pll_params_v1[HSCLK_RATE_MAX] = {
	{0x05, 0x3f, 0x00, 0x04, 0x01, 0x69, 0x00, 0x80, 0x07, 0x6f, 0x08, 0x45, 0x06, 0x36, 0x01,
		0xe2, 0x18, 0x0f, 0x0e, 0x1f, 0x0a, 0x11},
	{0x03, 0x3f, 0x00, 0x08, 0x01, 0x69, 0x00, 0x80, 0x07, 0x0f, 0x0e, 0x13, 0x06, 0x40, 0x01,
		0xe2, 0x18, 0x0f, 0x0e, 0x1f, 0x0a, 0x11},
	{0x01, 0x3f, 0x00, 0x08, 0x02, 0x8c, 0x00, 0x00, 0x0a, 0x1f, 0x1c, 0x1a, 0x08, 0x40, 0x01,
		0x2e, 0x21, 0x0f, 0x0e, 0x1f, 0x0a, 0x11},
	{0x00, 0x3f, 0x00, 0x08, 0x00, 0x69, 0x00, 0x80, 0x07, 0x2f, 0x2a, 0x13, 0x06, 0x40, 0x01,
		0xe2, 0x18, 0x0f, 0x0e, 0x1f, 0x0a, 0x11},
};

static const struct dp_pll_params pll_params_v1_1[HSCLK_RATE_MAX] = {
	{0x05, 0x3f, 0x00, 0x04, 0x01, 0x34, 0x00, 0xc0, 0x0b, 0x37, 0x04, 0x92, 0x01, 0x6b, 0x02,
		0x71, 0x0c, 0x0f, 0x0a, 0x0f, 0x0c, 0x0c},
	{0x03, 0x3f, 0x00, 0x08, 0x01, 0x34, 0x00, 0xc0, 0x0b, 0x07, 0x07, 0x92, 0x01, 0x6b, 0x02,
		0x71, 0x0c, 0x0f, 0x0a, 0x0f, 0x0c, 0x0c},
	{0x01, 0x3f, 0x00, 0x08, 0x02, 0x46, 0x00, 0x00, 0x05, 0x0f, 0x0e, 0x18, 0x02, 0x6b, 0x02,
		0x97, 0x10, 0x0f, 0x0a, 0x0f, 0x0c, 0x0c},
	{0x00, 0x3f, 0x00, 0x08, 0x00, 0x34, 0x00, 0xc0, 0x0b, 0x17, 0x15, 0x92, 0x01, 0x6b, 0x02,
		0x71, 0x0c, 0x0f, 0x0a, 0x0f, 0x0c, 0x0c}

};

static int set_vco_div(struct dp_pll *pll, unsigned long rate)
{
	u32 div, val;

	if (!pll)
		return -EINVAL;

	if (is_gdsc_disabled(pll))
		return -EINVAL;

	val = dp_pll_read(dp_phy, DP_PHY_VCO_DIV);
	val &= ~0x03;

	switch (rate) {
	case DP_VCO_HSCLK_RATE_1620MHZDIV1000:
	case DP_VCO_HSCLK_RATE_2700MHZDIV1000:
		div = 2;
		val |= 1;
		break;
	case DP_VCO_HSCLK_RATE_5400MHZDIV1000:
		div = 4;
		val |= 2;
		break;
	case DP_VCO_HSCLK_RATE_8100MHZDIV1000:
		div = 6;
		/* val = 0 for this case, so no update needed */
		break;
	default:
		/* No other link rates are supported */
		return -EINVAL;
	}

	dp_pll_write(dp_phy, DP_PHY_VCO_DIV, val);
	/* Make sure the PHY registers writes are done */
	wmb();

	/*
	 * Set the rate for the link and pixel clock sources so that the
	 * linux clock framework can appropriately compute the MND values
	 * whenever the pixel clock rate is set.
	 */
	clk_set_rate(pll->clk_data->clks[0], pll->vco_rate / 10);
	clk_set_rate(pll->clk_data->clks[1], pll->vco_rate / div);

	DP_DEBUG("val=%#x div=%x link_clk rate=%lu vco_div_clk rate=%lu\n",
			val, div, pll->vco_rate / 10, pll->vco_rate / div);

	return 0;
}

static int dp_vco_pll_init_db_4nm(struct dp_pll_db *pdb,
		unsigned long rate)
{
	struct dp_pll *pll = pdb->pll;
	u32 spare_value = 0;

	spare_value = dp_pll_read(dp_phy, DP_PHY_SPARE0);
	pdb->lane_cnt = spare_value & 0x0F;
	pdb->orientation = (spare_value & 0xF0) >> 4;

	DP_DEBUG("spare_value=0x%x, ln_cnt=0x%x, orientation=0x%x\n",
			spare_value, pdb->lane_cnt, pdb->orientation);

	switch (rate) {
	case DP_VCO_HSCLK_RATE_1620MHZDIV1000:
		DP_DEBUG("VCO rate: %ld\n", DP_VCO_RATE_9720MHZDIV1000);
		pdb->rate_idx = HSCLK_RATE_1620MHZ;
		break;
	case DP_VCO_HSCLK_RATE_2700MHZDIV1000:
		DP_DEBUG("VCO rate: %ld\n", DP_VCO_RATE_10800MHZDIV1000);
		pdb->rate_idx = HSCLK_RATE_2700MHZ;
		break;
	case DP_VCO_HSCLK_RATE_5400MHZDIV1000:
		DP_DEBUG("VCO rate: %ld\n", DP_VCO_RATE_10800MHZDIV1000);
		pdb->rate_idx = HSCLK_RATE_5400MHZ;
		break;
	case DP_VCO_HSCLK_RATE_8100MHZDIV1000:
		DP_DEBUG("VCO rate: %ld\n", DP_VCO_RATE_8100MHZDIV1000);
		pdb->rate_idx = HSCLK_RATE_8100MHZ;
		break;
	default:
		DP_ERR("unsupported rate %ld\n", rate);
		return -EINVAL;
	}
	return 0;
}

static int dp_config_vco_rate_4nm(struct dp_pll *pll,
		unsigned long rate)
{
	int rc = 0;
	struct dp_pll_db *pdb = (struct dp_pll_db *)pll->priv;
	const struct dp_pll_params *params;

	rc = dp_vco_pll_init_db_4nm(pdb, rate);
	if (rc < 0) {
		DP_ERR("VCO Init DB failed\n");
		return rc;
	}

	dp_pll_write(dp_phy, DP_PHY_CFG_1, 0x0F);

	if (pdb->lane_cnt != 4) {
		if (pdb->orientation == ORIENTATION_CC2)
			dp_pll_write(dp_phy, DP_PHY_PD_CTL, 0x6d);
		else
			dp_pll_write(dp_phy, DP_PHY_PD_CTL, 0x75);
	} else {
		dp_pll_write(dp_phy, DP_PHY_PD_CTL, 0x7d);
	}

	if (pdb->rate_idx < HSCLK_RATE_MAX) {
		params = &pdb->pll_params[pdb->rate_idx];
	} else {
		DP_ERR("link rate not set\n");
		return -EINVAL;
	}

	/* Make sure the PHY register writes are done */
	wmb();

	dp_pll_write(dp_pll, QSERDES_COM_SVS_MODE_CLK_SEL, 0x15);
	dp_pll_write(dp_pll, QSERDES_COM_SYSCLK_EN_SEL, 0x3b);
	dp_pll_write(dp_pll, QSERDES_COM_SYS_CLK_CTRL, 0x02);
	dp_pll_write(dp_pll, QSERDES_COM_CLK_ENABLE1, 0x0c);
	dp_pll_write(dp_pll, QSERDES_COM_SYSCLK_BUF_ENABLE, 0x06);
	dp_pll_write(dp_pll, QSERDES_COM_CLK_SEL, 0x30);
	/* Make sure the PHY register writes are done */
	wmb();

	/* PLL Optimization */
	dp_pll_write(dp_pll, QSERDES_COM_PLL_IVCO, params->pll_ivco);
	dp_pll_write(dp_pll, QSERDES_COM_PLL_CCTRL_MODE0, 0x36);
	dp_pll_write(dp_pll, QSERDES_COM_PLL_RCTRL_MODE0, 0x16);
	dp_pll_write(dp_pll, QSERDES_COM_CP_CTRL_MODE0, 0x06);
	/* Make sure the PLL register writes are done */
	wmb();

	/* link rate dependent params */
	dp_pll_write(dp_pll, QSERDES_COM_HSCLK_SEL_1, params->hsclk_sel);
	dp_pll_write(dp_pll, QSERDES_COM_DEC_START_MODE0, params->dec_start_mode0);
	dp_pll_write(dp_pll,
		QSERDES_COM_DIV_FRAC_START1_MODE0, params->div_frac_start1_mode0);
	dp_pll_write(dp_pll,
		QSERDES_COM_DIV_FRAC_START2_MODE0, params->div_frac_start2_mode0);
	dp_pll_write(dp_pll,
		QSERDES_COM_DIV_FRAC_START3_MODE0, params->div_frac_start3_mode0);
	dp_pll_write(dp_pll, QSERDES_COM_LOCK_CMP1_MODE0, params->lock_cmp1_mode0);
	dp_pll_write(dp_pll, QSERDES_COM_LOCK_CMP2_MODE0, params->lock_cmp2_mode0);
	dp_pll_write(dp_pll, QSERDES_COM_LOCK_CMP_EN, params->lock_cmp_en);
	dp_pll_write(dp_phy, DP_PHY_VCO_DIV, params->phy_vco_div);
	/* Make sure the PLL register writes are done */
	wmb();

	dp_pll_write(dp_pll, QSERDES_COM_CMN_CONFIG_1, 0x12);
	dp_pll_write(dp_pll, QSERDES_COM_INTEGLOOP_GAIN0_MODE0,
		params->integloop_gain0_mode0);
	dp_pll_write(dp_pll, QSERDES_COM_INTEGLOOP_GAIN1_MODE0,
		params->integloop_gain1_mode0);
	dp_pll_write(dp_pll, QSERDES_COM_VCO_TUNE_MAP, 0x00);
	/* Make sure the PHY register writes are done */
	wmb();

	dp_pll_write(dp_pll, QSERDES_COM_BG_TIMER, params->bg_timer);
	dp_pll_write(dp_pll, QSERDES_COM_CORECLK_DIV_MODE0, 0x14);
	dp_pll_write(dp_pll, QSERDES_COM_VCO_TUNE_CTRL, 0x00);

	if (pll->bonding_en)
		dp_pll_write(dp_pll, QSERDES_COM_BIAS_EN_CLKBUFLR_EN, 0x1f);
	else
		dp_pll_write(dp_pll, QSERDES_COM_BIAS_EN_CLKBUFLR_EN, 0x17);

	dp_pll_write(dp_pll, QSERDES_COM_CORE_CLK_EN, params->core_clk_en);
	dp_pll_write(dp_pll, QSERDES_COM_BIN_VCOCAL_CMP_CODE1_MODE0,
		params->cmp_code1_mode0);
	dp_pll_write(dp_pll, QSERDES_COM_BIN_VCOCAL_CMP_CODE2_MODE0,
		params->cmp_code2_mode0);
	/* Make sure the PHY register writes are done */
	wmb();

	if (pll->ssc_en) {
		dp_pll_write(dp_pll, QSERDES_COM_SSC_EN_CENTER, 0x01);
		dp_pll_write(dp_pll, QSERDES_COM_SSC_ADJ_PER1, 0x00);
		dp_pll_write(dp_pll, QSERDES_COM_SSC_PER1, params->ssc_per1);
		dp_pll_write(dp_pll, QSERDES_COM_SSC_PER2, params->ssc_per2);
		dp_pll_write(dp_pll, QSERDES_COM_SSC_STEP_SIZE1_MODE0,
				params->ssc_step_size1_mode0);
		dp_pll_write(dp_pll, QSERDES_COM_SSC_STEP_SIZE2_MODE0,
				params->ssc_step_size2_mode0);
	}

	if (pdb->orientation == ORIENTATION_CC2)
		dp_pll_write(dp_phy, DP_PHY_MODE, 0x4c);
	else
		dp_pll_write(dp_phy, DP_PHY_MODE, 0x5c);

	dp_pll_write(dp_phy, DP_PHY_AUX_CFG1, 0x13);
	dp_pll_write(dp_phy, DP_PHY_AUX_CFG2, 0xA4);
	/* Make sure the PLL register writes are done */
	wmb();

	/* TX-0 register configuration */
	dp_pll_write(dp_phy, DP_PHY_TX0_TX1_LANE_CTL, 0x05);
	dp_pll_write(dp_ln_tx0, DP_VMODE_CTRL1, 0x40);
	dp_pll_write(dp_ln_tx0, TXn_PRE_STALL_LDO_BOOST_EN, 0x30);
	dp_pll_write(dp_ln_tx0, TXn_INTERFACE_SELECT, 0x3b);
	dp_pll_write(dp_ln_tx0, TXn_CLKBUF_ENABLE, 0x0f);
	dp_pll_write(dp_ln_tx0, TXn_RESET_TSYNC_EN, 0x03);
	dp_pll_write(dp_ln_tx0, DP_TRAN_DRVR_EMP_EN, 0xf);
	dp_pll_write(dp_ln_tx0, TXn_PARRATE_REC_DETECT_IDLE_EN, 0x00);
	dp_pll_write(dp_ln_tx0, DP_TX_INTERFACE_MODE, 0x00);
	dp_pll_write(dp_ln_tx0, TXn_RES_CODE_LANE_OFFSET_TX, params->lane_offset_tx);
	dp_pll_write(dp_ln_tx0, TXn_RES_CODE_LANE_OFFSET_RX, params->lane_offset_rx);
	dp_pll_write(dp_ln_tx0, TXn_TX_BAND, 0x04);
	/* Make sure the PLL register writes are done */
	wmb();

	/* TX-1 register configuration */
	dp_pll_write(dp_phy, DP_PHY_TX2_TX3_LANE_CTL, 0x05);
	dp_pll_write(dp_ln_tx1, DP_VMODE_CTRL1, 0x40);
	dp_pll_write(dp_ln_tx1, TXn_PRE_STALL_LDO_BOOST_EN, 0x30);
	dp_pll_write(dp_ln_tx1, TXn_INTERFACE_SELECT, 0x3b);
	dp_pll_write(dp_ln_tx1, TXn_CLKBUF_ENABLE, 0x0f);
	dp_pll_write(dp_ln_tx1, TXn_RESET_TSYNC_EN, 0x03);
	dp_pll_write(dp_ln_tx1, DP_TRAN_DRVR_EMP_EN, 0xf);
	dp_pll_write(dp_ln_tx1, TXn_PARRATE_REC_DETECT_IDLE_EN, 0x00);
	dp_pll_write(dp_ln_tx1, DP_TX_INTERFACE_MODE, 0x00);
	dp_pll_write(dp_ln_tx1, TXn_RES_CODE_LANE_OFFSET_TX, params->lane_offset_tx);
	dp_pll_write(dp_ln_tx1, TXn_RES_CODE_LANE_OFFSET_RX, params->lane_offset_rx);
	dp_pll_write(dp_ln_tx1, TXn_TX_BAND, 0x04);
	/* Make sure the PHY register writes are done */
	wmb();

	return set_vco_div(pll, rate);
}

enum dp_4nm_pll_status {
	C_READY,
	FREQ_DONE,
	PLL_LOCKED,
	PHY_READY,
	TSYNC_DONE,
};

char *dp_4nm_pll_get_status_name(enum dp_4nm_pll_status status)
{
	switch (status) {
	case C_READY:
		return "C_READY";
	case FREQ_DONE:
		return "FREQ_DONE";
	case PLL_LOCKED:
		return "PLL_LOCKED";
	case PHY_READY:
		return "PHY_READY";
	case TSYNC_DONE:
		return "TSYNC_DONE";
	default:
		return "unknown";
	}
}

static bool dp_4nm_pll_get_status(struct dp_pll *pll,
		enum dp_4nm_pll_status status)
{
	u32 reg, state, bit;
	void __iomem *base;
	bool success = true;

	switch (status) {
	case C_READY:
		base = dp_pll_get_base(dp_pll);
		reg = QSERDES_COM_C_READY_STATUS;
		bit = DP_4NM_C_READY;
		break;
	case FREQ_DONE:
		base = dp_pll_get_base(dp_pll);
		reg = QSERDES_COM_CMN_STATUS;
		bit = DP_4NM_FREQ_DONE;
		break;
	case PLL_LOCKED:
		base = dp_pll_get_base(dp_pll);
		reg = QSERDES_COM_CMN_STATUS;
		bit = DP_4NM_PLL_LOCKED;
		break;
	case PHY_READY:
		base = dp_pll_get_base(dp_phy);
		reg = DP_PHY_STATUS;
		bit = DP_4NM_PHY_READY;
		break;
	case TSYNC_DONE:
		base = dp_pll_get_base(dp_phy);
		reg = DP_PHY_STATUS;
		bit = DP_4NM_TSYNC_DONE;
		break;
	default:
		return false;
	}

	if (readl_poll_timeout_atomic((base + reg), state,
			((state & bit) > 0),
			DP_PHY_PLL_POLL_SLEEP_US,
			DP_PHY_PLL_POLL_TIMEOUT_US)) {
		DP_ERR("%s failed, status=%x\n",
			dp_4nm_pll_get_status_name(status), state);

		success = false;
	}

	return success;
}

static int dp_pll_enable_4nm(struct dp_pll *pll)
{
	int rc = 0;

	pll->aux->state &= ~DP_STATE_PLL_LOCKED;

	dp_pll_write(dp_phy, DP_PHY_CFG, 0x01);
	dp_pll_write(dp_phy, DP_PHY_CFG, 0x05);
	dp_pll_write(dp_phy, DP_PHY_CFG, 0x01);
	dp_pll_write(dp_phy, DP_PHY_CFG, 0x09);
	dp_pll_write(dp_pll, QSERDES_COM_RESETSM_CNTRL, 0x20);
	wmb();	/* Make sure the PLL register writes are done */

	if (!dp_4nm_pll_get_status(pll, C_READY)) {
		rc = -EINVAL;
		goto lock_err;
	}

	if (!dp_4nm_pll_get_status(pll, FREQ_DONE)) {
		rc = -EINVAL;
		goto lock_err;
	}

	if (!dp_4nm_pll_get_status(pll, PLL_LOCKED)) {
		rc = -EINVAL;
		goto lock_err;
	}

	dp_pll_write(dp_phy, DP_PHY_CFG, 0x19);
	/* Make sure the PHY register writes are done */
	wmb();

	if (!dp_4nm_pll_get_status(pll, TSYNC_DONE)) {
		rc = -EINVAL;
		goto lock_err;
	}

	if (!dp_4nm_pll_get_status(pll, PHY_READY)) {
		rc = -EINVAL;
		goto lock_err;
	}

	pll->aux->state |= DP_STATE_PLL_LOCKED;
	DP_DEBUG("PLL is locked\n");

lock_err:
	return rc;
}

static void dp_pll_disable_4nm(struct dp_pll *pll)
{
	/* Assert DP PHY power down */
	dp_pll_write(dp_phy, DP_PHY_PD_CTL, 0x2);
	/*
	 * Make sure all the register writes to disable PLL are
	 * completed before doing any other operation
	 */
	wmb();
}

static int dp_vco_set_rate_4nm(struct dp_pll *pll, unsigned long rate)
{
	int rc = 0;

	if (!pll) {
		DP_ERR("invalid input parameters\n");
		return -EINVAL;
	}

	DP_DEBUG("DP lane CLK rate=%ld\n", rate);

	rc = dp_config_vco_rate_4nm(pll, rate);
	if (rc < 0) {
		DP_ERR("Failed to set clk rate\n");
		return rc;
	}

	return rc;
}

static int dp_regulator_enable_4nm(struct dp_parser *parser,
		enum dp_pm_type pm_type, bool enable)
{
	int rc = 0;
	struct dss_module_power mp;

	if (pm_type < DP_CORE_PM || pm_type >= DP_MAX_PM) {
		DP_ERR("invalid resource: %d %s\n", pm_type,
				dp_parser_pm_name(pm_type));
		return -EINVAL;
	}

	mp = parser->mp[pm_type];
	rc = msm_dss_enable_vreg(mp.vreg_config, mp.num_vreg, enable);
	if (rc) {
		DP_ERR("failed to '%s' vregs for %s\n",
				enable ? "enable" : "disable",
				dp_parser_pm_name(pm_type));
		return rc;
	}

	DP_DEBUG("success: '%s' vregs for %s\n", enable ? "enable" : "disable",
			dp_parser_pm_name(pm_type));
	return rc;
}

static int dp_pll_configure(struct dp_pll *pll, unsigned long rate)
{
	int rc = 0;

	if (!pll || !rate) {
		DP_ERR("invalid input parameters rate = %lu\n", rate);
		return -EINVAL;
	}

	rate = rate * 10;

	if (rate <= DP_VCO_HSCLK_RATE_1620MHZDIV1000)
		rate = DP_VCO_HSCLK_RATE_1620MHZDIV1000;
	else if (rate <= DP_VCO_HSCLK_RATE_2700MHZDIV1000)
		rate = DP_VCO_HSCLK_RATE_2700MHZDIV1000;
	else if (rate <= DP_VCO_HSCLK_RATE_5400MHZDIV1000)
		rate = DP_VCO_HSCLK_RATE_5400MHZDIV1000;
	else
		rate = DP_VCO_HSCLK_RATE_8100MHZDIV1000;

	pll->vco_rate = rate;
	rc = dp_vco_set_rate_4nm(pll, rate);
	if (rc < 0) {
		DP_ERR("pll rate %s set failed\n", rate);
		pll->vco_rate = 0;
		return rc;
	}

	DP_DEBUG("pll rate %lu set success\n", rate);
	return rc;
}

static int dp_pll_prepare(struct dp_pll *pll)
{
	int rc = 0;

	if (!pll) {
		DP_ERR("invalid input parameters\n");
		return -EINVAL;
	}

	/*
	 * Enable DP_PM_PLL regulator if the PLL revision is 4nm-V1 and the
	 * link rate is 8.1Gbps. This will result in voting to place Mx rail in
	 * turbo as required for V1 hardware PLL functionality.
	 */
	if (pll->revision >= DP_PLL_4NM_V1 &&
	    pll->vco_rate == DP_VCO_HSCLK_RATE_8100MHZDIV1000) {
		rc = dp_regulator_enable_4nm(pll->parser, DP_PLL_PM, true);
		if (rc < 0) {
			DP_ERR("enable pll power failed\n");
			return rc;
		}
	}

	rc = dp_pll_enable_4nm(pll);
	if (rc < 0)
		DP_ERR("ndx=%d failed to enable dp pll\n", pll->index);

	return rc;
}

static int dp_pll_unprepare(struct dp_pll *pll)
{
	int rc = 0;

	if (!pll) {
		DP_ERR("invalid input parameter\n");
		return -EINVAL;
	}

	if (pll->revision >= DP_PLL_4NM_V1 &&
			pll->vco_rate == DP_VCO_HSCLK_RATE_8100MHZDIV1000) {
		rc = dp_regulator_enable_4nm(pll->parser, DP_PLL_PM, false);
		if (rc < 0) {
			DP_ERR("disable pll power failed\n");
			return rc;
		}
	}

	dp_pll_disable_4nm(pll);
	pll->vco_rate = 0;

	return rc;
}

unsigned long dp_vco_recalc_rate_4nm(struct dp_pll *pll)
{
	u32 hsclk_sel, link_clk_divsel, hsclk_div, link_clk_div = 0;
	unsigned long vco_rate = 0;

	if (!pll) {
		DP_ERR("invalid input parameters\n");
		return -EINVAL;
	}

	if (is_gdsc_disabled(pll))
		return 0;

	hsclk_sel = dp_pll_read(dp_pll, QSERDES_COM_HSCLK_SEL_1);
	hsclk_sel &= 0x0f;

	switch (hsclk_sel) {
	case 5:
		hsclk_div = 5;
		break;
	case 3:
		hsclk_div = 3;
		break;
	case 1:
		hsclk_div = 2;
		break;
	case 0:
		hsclk_div = 1;
		break;
	default:
		DP_DEBUG("unknown divider. forcing to default\n");
		hsclk_div = 5;
		break;
	}

	link_clk_divsel = dp_pll_read(dp_phy, DP_PHY_AUX_CFG2);
	link_clk_divsel >>= 2;
	link_clk_divsel &= 0x3;

	if (link_clk_divsel == 0)
		link_clk_div = 5;
	else if (link_clk_divsel == 1)
		link_clk_div = 10;
	else if (link_clk_divsel == 2)
		link_clk_div = 20;
	else
		DP_ERR("unsupported div. Phy_mode: %d\n", link_clk_divsel);

	if (link_clk_div == 20) {
		vco_rate = DP_VCO_HSCLK_RATE_2700MHZDIV1000;
	} else {
		if (hsclk_div == 5)
			vco_rate = DP_VCO_HSCLK_RATE_1620MHZDIV1000;
		else if (hsclk_div == 3)
			vco_rate = DP_VCO_HSCLK_RATE_2700MHZDIV1000;
		else if (hsclk_div == 2)
			vco_rate = DP_VCO_HSCLK_RATE_5400MHZDIV1000;
		else
			vco_rate = DP_VCO_HSCLK_RATE_8100MHZDIV1000;
	}

	DP_DEBUG("hsclk: sel=0x%x, div=0x%x; lclk: sel=%u, div=%u, rate=%lu\n",
		hsclk_sel, hsclk_div, link_clk_divsel, link_clk_div, vco_rate);

	return vco_rate;
}

static unsigned long dp_pll_link_clk_recalc_rate(struct clk_hw *hw,
						 unsigned long parent_rate)
{
	struct dp_pll *pll = NULL;
	struct dp_pll_vco_clk *pll_link = NULL;
	unsigned long rate = 0;

	if (!hw) {
		DP_ERR("invalid input parameters\n");
		return -EINVAL;
	}

	pll_link = to_dp_vco_hw(hw);
	pll = pll_link->priv;

	rate = pll->vco_rate;
	rate = pll->vco_rate / 10;

	return rate;
}

static long dp_pll_link_clk_round(struct clk_hw *hw, unsigned long rate,
			unsigned long *parent_rate)
{
	struct dp_pll *pll = NULL;
	struct dp_pll_vco_clk *pll_link = NULL;

	if (!hw) {
		DP_ERR("invalid input parameters\n");
		return -EINVAL;
	}

	pll_link = to_dp_vco_hw(hw);
	pll = pll_link->priv;

	rate = pll->vco_rate / 10;

	return rate;
}

static unsigned long dp_pll_vco_div_clk_get_rate(struct dp_pll *pll)
{
	if (pll->vco_rate == DP_VCO_HSCLK_RATE_8100MHZDIV1000)
		return (pll->vco_rate / 6);
	else if (pll->vco_rate == DP_VCO_HSCLK_RATE_5400MHZDIV1000)
		return (pll->vco_rate / 4);
	else
		return (pll->vco_rate / 2);
}

static unsigned long dp_pll_vco_div_clk_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	struct dp_pll *pll = NULL;
	struct dp_pll_vco_clk *pll_link = NULL;

	if (!hw) {
		DP_ERR("invalid input parameters\n");
		return -EINVAL;
	}

	pll_link = to_dp_vco_hw(hw);
	pll = pll_link->priv;

	return dp_pll_vco_div_clk_get_rate(pll);
}

static long dp_pll_vco_div_clk_round(struct clk_hw *hw, unsigned long rate,
			unsigned long *parent_rate)
{
	return dp_pll_vco_div_clk_recalc_rate(hw, *parent_rate);
}

static const struct clk_ops pll_link_clk_ops = {
	.recalc_rate = dp_pll_link_clk_recalc_rate,
	.round_rate = dp_pll_link_clk_round,
};

static const struct clk_ops pll_vco_div_clk_ops = {
	.recalc_rate = dp_pll_vco_div_clk_recalc_rate,
	.round_rate = dp_pll_vco_div_clk_round,
};

static struct dp_pll_vco_clk dp0_phy_pll_clks[DP_PLL_NUM_CLKS] = {
	{
	.hw.init = &(struct clk_init_data) {
		.name = "dp0_phy_pll_link_clk",
		.ops = &pll_link_clk_ops,
		},
	},
	{
	.hw.init = &(struct clk_init_data) {
		.name = "dp0_phy_pll_vco_div_clk",
		.ops = &pll_vco_div_clk_ops,
		},
	},
};

static struct dp_pll_vco_clk dp_phy_pll_clks[DP_PLL_NUM_CLKS] = {
	{
	.hw.init = &(struct clk_init_data) {
		.name = "dp_phy_pll_link_clk",
		.ops = &pll_link_clk_ops,
		},
	},
	{
	.hw.init = &(struct clk_init_data) {
		.name = "dp_phy_pll_vco_div_clk",
		.ops = &pll_vco_div_clk_ops,
		},
	},
};

static struct dp_pll_db dp_pdb;

int dp_pll_clock_register_4nm(struct dp_pll *pll)
{
	int rc = 0;
	struct platform_device *pdev;
	struct dp_pll_vco_clk *pll_clks;

	if (!pll) {
		DP_ERR("pll data not initialized\n");
		return -EINVAL;
	}
	pdev = pll->pdev;

	pll->clk_data = kzalloc(sizeof(*pll->clk_data), GFP_KERNEL);
	if (!pll->clk_data)
		return -ENOMEM;

	pll->clk_data->clks = kcalloc(DP_PLL_NUM_CLKS, sizeof(struct clk *),
			GFP_KERNEL);
	if (!pll->clk_data->clks) {
		kfree(pll->clk_data);
		return -ENOMEM;
	}

	pll->clk_data->clk_num = DP_PLL_NUM_CLKS;
	pll->priv = &dp_pdb;
	dp_pdb.pll = pll;

	if (pll->revision == DP_PLL_4NM_V1_1)
		dp_pdb.pll_params = pll_params_v1_1;
	else
		dp_pdb.pll_params = pll_params_v1;

	pll->pll_cfg = dp_pll_configure;
	pll->pll_prepare = dp_pll_prepare;
	pll->pll_unprepare = dp_pll_unprepare;

	if (pll->dp_core_revision >= 0x10040000)
		pll_clks = dp0_phy_pll_clks;
	else
		pll_clks = dp_phy_pll_clks;

	rc = dp_pll_clock_register_helper(pll, pll_clks, DP_PLL_NUM_CLKS);
	if (rc) {
		DP_ERR("Clock register failed rc=%d\n", rc);
		goto clk_reg_fail;
	}

	rc = of_clk_add_provider(pdev->dev.of_node,
			of_clk_src_onecell_get, pll->clk_data);
	if (rc) {
		DP_ERR("Clock add provider failed rc=%d\n", rc);
		goto clk_reg_fail;
	}

	DP_DEBUG("success\n");
	return rc;

clk_reg_fail:
	dp_pll_clock_unregister_4nm(pll);
	return rc;
}

void dp_pll_clock_unregister_4nm(struct dp_pll *pll)
{
	kfree(pll->clk_data->clks);
	kfree(pll->clk_data);
}
