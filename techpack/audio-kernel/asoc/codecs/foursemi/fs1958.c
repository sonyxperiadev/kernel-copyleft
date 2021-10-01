/**
 * Copyright (C) Fourier Semiconductor Inc. 2016-2020. All rights reserved.
 * 2019-01-29 File created.
 */

#include "fsm_public.h"
#if defined(CONFIG_FSM_FS1958)

#define FS1958_STATUS         0xF000
#define FS1958_BOVDS          0x0000
#define FS1958_PLLS           0x0100
#define FS1958_OTPDS          0x0200
#define FS1958_OVDS           0x0300
#define FS1958_UVDS           0x0400
#define FS1958_OCDS           0x0500
#define FS1958_CLKS           0x0600
#define FS1958_OTWDS          0x0800
#define FS1958_BOPS           0x0900
#define FS1958_BOLDS          0x0A00
#define FS1958_INDS           0x0B00
#define FS1958_VDDPS          0x0E00

#define FS1958_REVID          0xF004

#define FS1958_DIGSTAT        0xF006
#define FS1958_ADCRUN         0x0006
#define FS1958_DACRUN         0x0106

#define FS1958_CHIPINI        0xF00E

#define FS1958_PWRCTRL        0xF010
#define FS1958_PWDN           0x0010
#define FS1958_I2CR           0x0110

#define FS1958_SYSCTRL        0xF011
#define FS1958_OSCEN          0x0011
#define FS1958_VBGEN          0x0111
#define FS1958_ZMEN           0x0211
#define FS1958_ZDPEN          0x0311
#define FS1958_PLLEN          0x0511
#define FS1958_BSTEN          0x0611
#define FS1958_AMPEN          0x0711

#define FS1958_SPKCOEF        0xF014
#define FS1958_AUDIOCTRL      0xF016

#define FS1958_I2SCTRL        0xF017
#define FS1958_I2SSR          0x3C17
#define FS1958_I2SDOE         0x0B17
#define FS1958_CHS12          0x1317

#define FS1958_TDMCTRL        0xF019

#define FS1958_BSTCTRL        0xF020
#define FS1958_SSEND          0x0F20

#define FS1958_DACCTRL        0xF030

#define FS1958_TSCTRL         0xF04C
#define FS1958_OFFSTA         0x0E4C
#define FS1958_OFF_AUTOEN     0x0D4C
#define FS1958_TSEN           0x034C

#define FS1958_PLLCTRL1       0xF0A1
#define FS1958_PLLCTRL2       0xF0A2
#define FS1958_PLLCTRL3       0xF0A3

#define FS1958_FW_NAME        "fs1958.fsm"
#define FS1958_RS2RL_RATIO    (2700)

static const struct fsm_srate g_fs1958_srate_tbl[] = {
	{   8000, 0x1 },
	{  16000, 0x3 },
	{  32000, 0x7 },
	{  44100, 0x8 },
	{  48000, 0x9 },
	{  88200, 0xA },
	{  96000, 0xB },
	{ 176400, 0xC },
	{ 192000, 0xD },
};

const static fsm_pll_config_t g_fs1958_pll_tbl[] = {
	/* bclk,    0xA1,   0xA2,   0xA3 */
	{  256000, 0x0260, 0x0540, 0x0001 }, //  8000*16*2
	{  512000, 0x0260, 0x0540, 0x0002 }, // 16000*16*2 &  8000*32*2
	{ 1024000, 0x0260, 0x0540, 0x0004 }, //            & 16000*32*2
	{ 1024032, 0x0160, 0x0380, 0x0004 }, // 32000*16*2+32
	{ 1411200, 0x0260, 0x0460, 0x0005 }, // 44100*16*2
	{ 1536000, 0x0260, 0x0540, 0x0006 }, // 48000*16*2
	{ 2048032, 0x0160, 0x0380, 0x0008 }, //            & 32000*32*2+32
	{ 2822400, 0x0260, 0x0460, 0x000A }, //            & 44100*32*2
	{ 3072000, 0x0260, 0x0540, 0x000C }, //            & 48000*32*2
};

int fs1958_check_stable(fsm_dev_t *fsm_dev, int type)
{
	uint16_t value;
	int ready;
	int ret;

	if (fsm_dev == NULL) {
		return -EINVAL;
	}
	switch (type) {
	case FSM_WAIT_STATUS_ON:
		ret = fsm_reg_multiread(fsm_dev, REG(FS1958_STATUS), &value);
		ready = get_bf_val(FS1958_CLKS, value);
		ready &= get_bf_val(FS1958_PLLS, value);
		break;
	case FSM_WAIT_AMP_ADC_ON:
	case FSM_WAIT_TSIGNAL_OFF:
		if (fsm_dev->id == 0)
			fsm_delay_ms(30);
		ready = 1;
		ret = 0;
		break;
	case FSM_WAIT_AMP_ON:
	case FSM_WAIT_AMP_ADC_OFF:
	case FSM_WAIT_AMP_ADC_PLL_OFF:
	case FSM_WAIT_AMP_OFF:
		ready = 1;
		ret = 0;
		break;
	case FSM_WAIT_BOOST_SSEND:
		ret = fsm_get_bf(fsm_dev, FS1958_SSEND, &value);
		ready = value;
		break;
	default:
		ret = -EINVAL;
		ready = 0;
		break;
	}
	if (!ret && !ready) {
		ret = -EINVAL;
	}

	return ret;
}

static int fs1958_i2c_reset(fsm_dev_t *fsm_dev)
{
	uint16_t val;
	int ret;
	int i;

	if (!fsm_dev) {
		return -EINVAL;
	}

	fsm_dev->acc_count = 0;
	for (i = 0; i < FSM_I2C_RETRY; i++) {
		fsm_reg_write(fsm_dev, REG(FS1958_PWRCTRL), 0x0002); // reset nack
		fsm_reg_read(fsm_dev, REG(FS1958_PWRCTRL), NULL); // dummy read
		fsm_delay_ms(15); // 15ms
		ret = fsm_reg_write(fsm_dev, REG(FS1958_PWRCTRL), 0x0001);
		ret |= fsm_reg_read(fsm_dev, REG(FS1958_CHIPINI), &val);
		if ((val == 0x0003) || (val == 0x0300)) { // init finished
			break;
		}
	}
	if (i == FSM_I2C_RETRY) {
		pr_addr(err, "retry timeout");
		ret = -ETIMEDOUT;
	}

	return ret;
}

int fs1958_get_srate_bits(fsm_dev_t *fsm_dev, uint32_t srate)
{
	int size;
	int idx;

	if (!fsm_dev) {
		return -EINVAL;
	}
	size = sizeof(g_fs1958_srate_tbl)/ sizeof(struct fsm_srate);
	for (idx = 0; idx < size; idx++) {
		if (srate == g_fs1958_srate_tbl[idx].srate)
			return g_fs1958_srate_tbl[idx].bf_val;
	}
	pr_addr(err, "invalid srate:%d", srate);
	return -EINVAL;
}

int fs1958_dev_init(fsm_dev_t *fsm_dev)
{
	fsm_config_t *cfg = fsm_get_config();
	int ret;

	if (fsm_dev == NULL || cfg == NULL) {
		return -EINVAL;
	}

	if (cfg->force_init) {
		pr_addr(info, "force init");
	}
	if (!cfg->force_init && fsm_dev->state.dev_inited) {
		return 0;
	}
	ret = fs1958_i2c_reset(fsm_dev);
	ret |= fsm_init_dev_list(fsm_dev);
	if (ret || !fsm_dev->dev_list) {
		pr_addr(err, "init dev_list fail:%d", ret);
		return ret;
	}
	fsm_dev->state.dev_inited = true;
	ret = fsm_init_info(fsm_dev);
	ret |= fsm_reg_write(fsm_dev, REG(FS1958_SPKCOEF), (fsm_dev->tcoef << 1));
	ret |= fsm_reg_write(fsm_dev, REG(FS1958_PWRCTRL), 0x0000);
	ret |= fsm_write_reg_tbl(fsm_dev, FSM_SCENE_COMMON);
	ret |= fsm_write_reg_tbl(fsm_dev, cfg->next_scene);

	ret |= fsm_reg_write(fsm_dev, REG(FS1958_SYSCTRL), 0x0000);
	ret |= fsm_reg_write(fsm_dev, REG(FS1958_PWRCTRL), 0x0001);
	fsm_delay_ms(30);
	if (ret) {
		pr_addr(err, "init failed:%d", ret);
		fsm_dev->state.dev_inited = false;
	}
	fsm_dev->state.calibrated = !ret;
	fsm_dev->errcode = ret;

	return ret;
}

static int fs1958_config_pll(fsm_dev_t *fsm_dev, bool on)
{
	fsm_config_t *cfg = fsm_get_config();
	const struct fsm_pll_config *pll_cfg;
	int tbl_size;
	int idx;
	int ret;

	if (!fsm_dev || !cfg) {
		return -EINVAL;
	}
	// config pll need disable pll firstly
	if (!on) { // disable pll
		return 0;
	}

	pll_cfg = g_fs1958_pll_tbl;
	tbl_size = ARRAY_SIZE(g_fs1958_pll_tbl);
	for (idx = 0; idx < tbl_size; idx++) {
		if (pll_cfg[idx].bclk == cfg->i2s_bclk) {
			break;
		}
	}
	if (idx >= tbl_size) {
		pr_addr(err, "invalid bclk/rate:%d,%d", cfg->i2s_bclk, cfg->i2s_srate);
		return -EINVAL;
	}
	pr_addr(debug, "bclk[%d]: %d", idx, cfg->i2s_bclk);
	ret = fsm_access_key(fsm_dev, 1);

	ret |= fsm_reg_write(fsm_dev, REG(FS1958_PLLCTRL1), pll_cfg[idx].c1);
	ret |= fsm_reg_write(fsm_dev, REG(FS1958_PLLCTRL2), pll_cfg[idx].c2);
	ret |= fsm_reg_write(fsm_dev, REG(FS1958_PLLCTRL3), pll_cfg[idx].c3);

	ret |= fsm_access_key(fsm_dev, 0);

	FSM_FUNC_EXIT(ret);
	return ret;
}

static int fs1958_config_i2s(fsm_dev_t *fsm_dev)
{
	fsm_config_t *cfg = fsm_get_config();
	uint16_t i2sctrl;
	uint16_t revid;
	int i2ssr;
	int ret;

	if (!fsm_dev || !cfg) {
		return -EINVAL;
	}
	i2ssr = fs1958_get_srate_bits(fsm_dev, cfg->i2s_srate);
	if (i2ssr < 0) {
		pr_addr(err, "unsupport srate:%d", cfg->i2s_srate);
		return -EINVAL;
	}
	ret = fsm_reg_read(fsm_dev, REG(FS1958_I2SCTRL), &i2sctrl);
	ret |= fsm_reg_read(fsm_dev, REG(FS1958_REVID), &revid);
	if (LOW8(revid) == 0xA1) {
		i2ssr = 7;
	}
	set_bf_val(&i2sctrl, FS1958_I2SSR, i2ssr);
	set_bf_val(&i2sctrl, FS1958_I2SDOE, 1);
	pr_addr(debug, "srate:%d, val:%04X", cfg->i2s_srate, i2sctrl);
	ret |= fsm_reg_write(fsm_dev, REG(FS1958_I2SCTRL), i2sctrl);

	FSM_FUNC_EXIT(ret);
	return ret;
}

int fs1958_switch_preset(fsm_dev_t *fsm_dev)
{
	fsm_config_t *cfg = fsm_get_config();
	reg_temp_t sysctrl;
	int ret;

	pr_addr(debug, "%s switching",
			(cfg->force_scene ? "force" : "auto"));
	if (fsm_dev == NULL) {
		return -EINVAL;
	}
	if (!cfg->force_scene && fsm_dev->cur_scene == cfg->next_scene) {
		pr_addr(debug, "same scene, skip");
		return 0;
	}
	// need amplifier off
	ret = fsm_reg_read(fsm_dev, REG(FS1958_SYSCTRL), &sysctrl.new_val);
	sysctrl.old_val = sysctrl.new_val;
	set_bf_val(&sysctrl.new_val, FS1958_AMPEN, 0);
	// enable pll osc
	set_bf_val(&sysctrl.new_val, FS1958_OSCEN, 1);
	ret |= fsm_reg_write(fsm_dev, REG(FS1958_SYSCTRL), sysctrl.new_val);
	// wait stable
	fsm_delay_ms(30);

	// load reg table
	ret |= fsm_write_reg_tbl(fsm_dev, cfg->next_scene);
	ret |= fsm_reg_write(fsm_dev, REG(FS1958_SYSCTRL), sysctrl.old_val);
	fsm_dev->cur_scene = cfg->next_scene;

	FSM_FUNC_EXIT(ret);
	return ret;
}

int fs1958_start_up(fsm_dev_t *fsm_dev)
{
	int ret;

	if (!fsm_dev) {
		return -EINVAL;
	}
	ret = fs1958_config_i2s(fsm_dev);
	ret |= fs1958_config_pll(fsm_dev, true);
	// 0x1100EF; all enable
	ret |= fsm_reg_write(fsm_dev, REG(FS1958_SYSCTRL), 0x00EF);
	// 0x100000; power up
	ret |= fsm_reg_write(fsm_dev, REG(FS1958_PWRCTRL), 0x0000);

	fsm_dev->errcode = ret;

	return ret;
}

int fs1958_set_mute(fsm_dev_t *fsm_dev, bool mute)
{
	// move to set tsignal
	return 0;
}

int fs1958_set_tsignal(fsm_dev_t *fsm_dev, bool enable)
{
	fsm_config_t *cfg = fsm_get_config();
	uint16_t dacctrl;
	uint16_t volume;
	uint16_t tsctrl;
	int ret;

	if (!cfg || !fsm_dev) {
		return -EINVAL;
	}
	volume = ((cfg->volume * 2 + 1) << 7);
	dacctrl = (enable ? 0x0210 : 0x0310);
	ret = fsm_reg_write(fsm_dev, REG(FS1958_AUDIOCTRL), volume);
	ret |= fsm_reg_write(fsm_dev, REG(FS1958_DACCTRL), dacctrl);
	// make sure ts on
	ret = fsm_reg_multiread(fsm_dev, REG(FS1958_TSCTRL), &tsctrl);
	if (set_bf_val(&tsctrl, FS1958_TSEN, 1) != tsctrl) {
		ret |= fsm_reg_write(fsm_dev, REG(FS1958_TSCTRL), tsctrl);
	}

	return ret;
}

int fs1958_shut_down(fsm_dev_t *fsm_dev)
{
	int ret;

	if (fsm_dev == NULL) {
		return -EINVAL;
	}
	ret = fsm_reg_write(fsm_dev, REG(FS1958_PWRCTRL), 0x0001);
	ret |= fsm_reg_write(fsm_dev, REG(FS1958_SYSCTRL), 0x0000);

	FSM_FUNC_EXIT(ret);
	return ret;
}

int fs1958_pre_test(fsm_dev_t *fsm_dev)
{
	fsm_config_t *cfg = fsm_get_config();
	uint16_t tsctrl;
	int ret;

	if (!fsm_dev || !cfg) {
		return -EINVAL;
	}
	if (fsm_skip_device(fsm_dev)) {
		return 0;
	}
	if (cfg->test_type == TEST_RE25) {
		fsm_dev->re25 = 0;
		fsm_dev->state.re25_runin = true;
		if (fsm_dev->tdata) {
			fsm_dev->tdata->sum_re25   = 0;
			fsm_dev->tdata->count_re25 = 0;
			fsm_dev->tdata->test_re25  = 0;
			memset(&fsm_dev->tdata->re25, 0, sizeof(struct re25_data));
		}
	}
	else if (cfg->test_type == TEST_F0) {
		fsm_dev->f0 = 0;
		fsm_dev->state.f0_runing = true;
		if (fsm_dev->tdata) {
			fsm_dev->tdata->sum_f0   = 0;
			fsm_dev->tdata->count_f0 = 0;
			fsm_dev->tdata->test_f0  = 0;
			memset(&fsm_dev->tdata->f0, 0, sizeof(struct f0_data));
		}
	}
	ret = fsm_set_bf(fsm_dev, 0x0F3C, 0); // disable lpm
	ret = fsm_reg_multiread(fsm_dev, REG(FS1958_TSCTRL), &tsctrl);
	if (ret) {
		pr_addr(err, "read reg failed:%d", ret);
		return ret;
	}
	if (get_bf_val(FS1958_OFF_AUTOEN, tsctrl) && get_bf_val(FS1958_TSEN, tsctrl)) {
		set_bf_val(&tsctrl, FS1958_TSEN, 0); // disable ts
		ret |= fsm_reg_write(fsm_dev, REG(FS1958_TSCTRL), tsctrl);
		fsm_delay_ms(35);
	}
	set_bf_val(&tsctrl, FS1958_TSEN, 1);
	set_bf_val(&tsctrl, FS1958_OFF_AUTOEN, 0);
	ret |= fsm_reg_write(fsm_dev, REG(FS1958_TSCTRL), tsctrl);

	return ret;
}

static int fs1958_cal_zmdata(fsm_dev_t *fsm_dev)
{
	fsm_config_t *cfg = fsm_get_config();
	uint32_t payload[FSM_PAYLOAD_SIZE];
	struct fsm_test_data *tdata;
	fsm_msg_t data;
	int ret;

	if (!fsm_dev || !cfg) {
		return -EINVAL;
	}
	if (fsm_skip_device(fsm_dev)) {
		return 0;
	}
	if (!fsm_dev->tdata) {
		fsm_dev->tdata = fsm_alloc_mem(sizeof(struct fsm_test_data));
		if (!fsm_dev->tdata) {
			pr_addr(err, "alloc test data fail");
			return -EINVAL;
		}
		memset(fsm_dev->tdata, 0, sizeof(struct fsm_test_data));
	}
	if (cfg->test_type != TEST_RE25 && cfg->test_type != TEST_F0) {
		pr_addr(err, "invalid test type:%d", cfg->test_type);
		return -EINVAL;
	}
	data.size = sizeof(uint32_t) * FSM_PAYLOAD_SIZE;
	data.buf = payload;
	ret = fsm_get_livedata(&data);
	if (ret) {
		pr_err("get livedata fail:%d", ret);
		return ret;
	}
	tdata = fsm_dev->tdata;
	if (cfg->test_type == TEST_RE25) {
		if (!fsm_dev->state.re25_runin) {
			return 0;
		}
		pr_addr(info, "re25[%d]:%d", tdata->count_re25, payload[0]);
		tdata->sum_re25 += payload[0];
		tdata->count_re25++;
		if(tdata->count_re25 >= RE25_TEST_COUNT) {
			// test finished
			tdata->test_re25 = tdata->sum_re25 / tdata->count_re25;
			fsm_dev->re25 = tdata->test_re25;
			return 0;
		}
	}
	else if(cfg->test_type == TEST_F0) {
		if (!fsm_dev->state.f0_runing) {
			return 0;
		}
		pr_addr(info, "f0[%d]:%d", tdata->count_f0, payload[3]);
		tdata->sum_f0 += payload[3];
		tdata->count_f0++;
		if (tdata->count_f0 >= cfg->freq_count) {
			// test finished
			tdata->test_f0 = tdata->sum_f0 / tdata->count_f0;
			fsm_dev->f0 = tdata->test_f0;
			return 0;
		}
	}

	return -EINVAL;
}

int fs1958_post_test(fsm_dev_t *fsm_dev)
{
	fsm_config_t *cfg = fsm_get_config();
	uint16_t tsctrl;
	uint16_t temp;
	int ret;

	if (!cfg || !fsm_dev || !fsm_dev->tdata) {
		return -EINVAL;
	}
	if (fsm_skip_device(fsm_dev)) {
		return 0;
	}
	if (cfg->test_type == TEST_RE25) {
		ret = fsm_stub_store_re25(fsm_dev);
		if (ret) {
			fsm_dev->tdata->test_re25 = 0;
			fsm_dev->re25 = 0;
		}
		fsm_dev->state.re25_runin = false;
	}
	else if (cfg->test_type == TEST_F0) {
		fsm_dev->state.f0_runing = false;
	}
	ret = fsm_reg_multiread(fsm_dev, REG(FS1958_TSCTRL), &tsctrl);
	temp = tsctrl;
	set_bf_val(&tsctrl, FS1958_TSEN, 1);
	set_bf_val(&tsctrl, FS1958_OFF_AUTOEN, 1);
	if (temp != tsctrl) {
		ret |= fsm_reg_write(fsm_dev, REG(FS1958_TSCTRL), tsctrl);
	}
	ret |= fsm_set_bf(fsm_dev, 0x0F3C, 1); // enable lpm
	return ret;
}

void fs1958_ops(fsm_dev_t *fsm_dev)
{
	fsm_config_t *cfg = fsm_get_config();

	if (!fsm_dev || !cfg) {
		return;
	}

	fsm_set_fw_name(FS1958_FW_NAME);
	fsm_dev->dev_ops.dev_init = fs1958_dev_init;
	fsm_dev->dev_ops.i2s_config = fs1958_config_i2s;
	fsm_dev->dev_ops.pll_config = fs1958_config_pll;
	fsm_dev->dev_ops.check_stable = fs1958_check_stable;
	fsm_dev->dev_ops.switch_preset = fs1958_switch_preset;
	fsm_dev->dev_ops.start_up = fs1958_start_up;
	fsm_dev->dev_ops.set_mute = fs1958_set_mute;
	fsm_dev->dev_ops.set_tsignal = fs1958_set_tsignal;
	fsm_dev->dev_ops.shut_down = fs1958_shut_down;
	fsm_dev->dev_ops.pre_calib = fs1958_pre_test;
	fsm_dev->dev_ops.cal_zmdata = fs1958_cal_zmdata;
	fsm_dev->dev_ops.post_calib = fs1958_post_test;
	fsm_dev->dev_ops.pre_f0_test = fs1958_pre_test;
	//fsm_dev->dev_ops.f0_test = fs1958_cal_zmdata;
	fsm_dev->dev_ops.post_f0_test = fs1958_post_test;
	fsm_dev->compat.RS2RL_RATIO = FS1958_RS2RL_RATIO;
	cfg->nondsp_mode = true;
	cfg->store_otp = false;
}
#endif
