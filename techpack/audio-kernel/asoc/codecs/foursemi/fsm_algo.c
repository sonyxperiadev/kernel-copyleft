/**
 * Copyright (C) Fourier Semiconductor Inc. 2016-2020. All rights reserved.
 * 2019-12-09 File created.
 */

#include "fsm_public.h"
#if defined(CONFIG_FSM_NONDSP)
#include <linux/types.h>
#include <sound/soc.h>
#include <linux/slab.h>

#define FSM_CAL_BOOST(idx) (600 + 25 * (idx)) // step: 25
#define FSM_CAL_ILIMT(idx) (140 + 20 * (idx)) // step: 20

#define FSADSP_RE25_CMD_VERSION_V1 0xA001
struct fsadsp_cal_data {
	uint16_t rstrim;
	uint16_t channel;
	int re25;
} __packed;

struct fsadsp_cmd_re25 {
	uint16_t version;
	uint16_t ndev;
	struct fsadsp_cal_data cal_data[FSM_DEV_MAX];
} __packed;


static struct fsm_calib_v2 g_calib_data = { 0 };
static atomic_t g_fsm_algo_vi_tx_port;
static atomic_t g_fsm_algo_tx_fmt;
static atomic_t g_fsm_algo_scene;
static atomic_t g_fsm_algo_status;

#if defined(CONFIG_FSM_QCOM_NONDSP)
static int fsm_get_boost_limit(uint16_t scene, uint8_t pos, uint32_t *data)
{
	fsm_config_t *cfg = fsm_get_config();
	reg_scene_t *reg_scene;
	fsm_dev_t *fsm_dev;
	regs_unit_t *regs;
	int idx;
	int ret;

	if (!cfg || !data) {
		return -EINVAL;
	}
	for (idx = 0; idx < cfg->dev_count; idx++) {
		fsm_dev = fsm_get_fsm_dev_by_id(idx);
		if (fsm_dev == NULL) continue;
		if (fsm_dev->pos_mask == pos) {
			break;
		}
	}
	if (idx >= cfg->dev_count) {
		pr_err("not device matched pos:%02X", pos);
		return -EINVAL;
	}
	reg_scene = fsm_get_data_list(fsm_dev, FSM_DSC_REG_SCENES);
	if (!reg_scene) {
		pr_err("non reg scene data");
		return -EINVAL;
	}
	regs = reg_scene->regs;
	// pr_addr(debug, "reg scene len: %d", reg_scene->len);
	for (idx = 0; idx < reg_scene->len; idx++) {
		if ((regs[idx].scene & scene) == 0
				|| regs[idx].reg.addr != 0xC0) {
			continue;
		}
		if (regs[idx].reg.len == 15) {
			data[0] = FSM_CAL_BOOST(get_bf_val(FSM_VOUT_SEL, regs[idx].reg.value));
			data[1] = FSM_CAL_ILIMT(get_bf_val(FSM_ILIM_SEL, regs[idx].reg.value));
		}
		if (regs[idx].reg.len == 3 && regs[idx].reg.pos == 0xA) {
			data[0] = FSM_CAL_BOOST(regs[idx].reg.value);
		}
		if (regs[idx].reg.len == 3 && regs[idx].reg.pos == 0x6) {
			data[1] = FSM_CAL_ILIMT(regs[idx].reg.value);
		}
	}
	if (data[0] == 975) {
		data[0] = 1000;
	}
	if (data[1] > 420) {
		data[1] = 420;
	}
	pr_addr(info, "POS:%02X, VOUT:%d, ILIM:%d", pos, data[0], data[1]);

	return ret;
}

static int fsm_get_rstrim_by_chn(int chn)
{
	fsm_config_t *cfg = fsm_get_config();
	fsm_dev_t *fsm_dev;
	int rstrim = FSM_RS_TRIM_DEFAULT;
	int dev;
	int ret;

	for (dev = 0; dev < cfg->dev_count; dev++) {
		fsm_dev = fsm_get_fsm_dev_by_id(dev);
		if (fsm_dev == NULL || fsm_dev->pos_mask != chn) {
			continue;
		}
		if (fsm_dev->rstrim == 0) {
			ret = fsm_get_rstrim(fsm_dev);
			if (ret) {
				break;
			}
		}
		rstrim = fsm_dev->rstrim;
		break;
	}

	return rstrim;
}

static int fsm_set_cfg_data(fsm_msg_t *data)
{
	fsm_config_t *cfg = fsm_get_config();
	struct fsm_calib_v2 *calib;
	uint32_t *payload;
	int cmd_count = 0;
	int idx;
	int dev;

	if (!data) {
		return -EINVAL;
	}
	// re25
	calib = &g_calib_data;
	if (fsm_read_efsdata(calib)) {
		pr_err("read efs data failed");
		calib->dev_count = 0;
	}

	payload = (uint32_t *)data->buf;
	idx = 1;
	// 1. app mode
	payload[idx++] = FSADSP_PARAM_CMD(FSADSP_CFG_APP_MODE, 1, 0xFF);
	cmd_count++;
	payload[idx++] = cfg->next_scene;
	// 2. tx format
	payload[idx++] = FSADSP_PARAM_CMD(FSADSP_CFG_TX_FMT, 1, 0xFF);
	cmd_count++;
	payload[idx++] = cfg->tx_fmt;
	// 3. re25 of all devices, and boost/limit config
	for (dev = 0; dev < calib->dev_count; dev++) {
		// re25 and channel
		payload[idx++] = FSADSP_PARAM_CMD(FSADSP_CFG_RE25, 1,
				calib->cal_data[dev].channel);
		cmd_count++;
		payload[idx++] = calib->cal_data[dev].cal_re;
		pr_info("POS:%02X, RE25:%d", calib->cal_data[dev].channel,
				calib->cal_data[dev].cal_re);
		// rstrim of device by channel
		payload[idx++] = FSADSP_PARAM_CMD(FSADSP_CFG_RSTRIM, 1,
				calib->cal_data[dev].channel);
		cmd_count++;
		payload[idx++] = fsm_get_rstrim_by_chn(calib->cal_data[dev].channel);
		// boost/clipping limit and channel
		payload[idx++] = FSADSP_PARAM_CMD(FSADSP_CFG_CLIPPING, 2,
				calib->cal_data[dev].channel);
		cmd_count++;
		fsm_get_boost_limit(cfg->next_scene, calib->cal_data[dev].channel,
				payload + idx);
		idx += 2; // data[0]=VOUT*100, data[1]=ILIMT*100
	}
	payload[0] = cmd_count;

	data->size = sizeof(uint32_t) * idx;

	return 0;
}

// #if defined(CONFIG_FSM_QCOM_NONDSP)
static int g_fsm_port = FSM_PORT_PRI_MI2S;

static int fsm_get_ports_id(struct afe_fsm_header *hdr, int fsm_port)
{
	if (!hdr) {
		return -EINVAL;
	}
	switch (fsm_port) {
	case FSM_PORT_PRI_MI2S:
		hdr->rx_port = AFE_PORT_ID_PRIMARY_MI2S_RX;
		hdr->tx_port = AFE_PORT_ID_PRIMARY_MI2S_TX;
		break;
	case FSM_PORT_SEC_MI2S:
		hdr->rx_port = AFE_PORT_ID_SECONDARY_MI2S_RX;
		hdr->tx_port = AFE_PORT_ID_SECONDARY_MI2S_TX;
		break;
	case FSM_PORT_TERT_MI2S:
		hdr->rx_port = AFE_PORT_ID_TERTIARY_MI2S_RX;
		hdr->tx_port = AFE_PORT_ID_TERTIARY_MI2S_TX;
		break;
	case FSM_PORT_QUAT_MI2S:
		hdr->rx_port = AFE_PORT_ID_QUATERNARY_MI2S_RX;
		hdr->tx_port = AFE_PORT_ID_QUATERNARY_MI2S_TX;
		break;
	case FSM_PORT_QUIN_MI2S:
		hdr->rx_port = AFE_PORT_ID_QUINARY_MI2S_RX;
		hdr->tx_port = AFE_PORT_ID_QUINARY_MI2S_TX;
		break;
	case FSM_PORT_QUAT_TDM:
		hdr->rx_port = AFE_PORT_ID_QUATERNARY_TDM_RX;
		hdr->tx_port = AFE_PORT_ID_QUATERNARY_TDM_TX;
		break;
	case FSM_PORT_ZERO: // do nothing
		break;
	default:
		pr_err("invalid fsm port:%d", fsm_port);
		hdr->rx_port = 0;
		hdr->tx_port = 0;
		return -EINVAL;
	}

	return 0;
}

static int fsm_set_iv_fb_ports(int port, bool stereo)
{
	struct afe_fsm_port_config port_cfg;
	struct afe_fsm_header hdr;
	int ret;

	if (port == FSM_PORT_ZERO) { // disable tx
		port_cfg.enable = false;
		port_cfg.rx_port = 0;
		port_cfg.tx_port = 0;
	}
	else {
		ret = fsm_get_ports_id(&hdr, port);
		if (ret) {
			pr_err("get ports id fail:%d", ret);
			return ret;
		}
		port_cfg.enable = true;
		port_cfg.rx_port = hdr.rx_port;
		port_cfg.tx_port = hdr.tx_port;
	}
	port_cfg.stereo = stereo;
	pr_info("PORT:%d, RX:%X, TX:%X", port, port_cfg.rx_port, port_cfg.tx_port);
	ret = afe_set_fsm_iv_fb_port(&port_cfg);

	return ret;
}

int fsm_afe_re25_ctrl(struct fsm_calib_v2 *calib_data, int opcode)
{
	struct fsadsp_cmd_re25 params;
	struct afe_fsm_header header;
	struct fsm_msg data;
	int ret;

	if (!calib_data) {
		return -EINVAL;
	}
	header.module_id = AFE_MODULE_ID_FSADSP_RX;
	ret = fsm_get_ports_id(&header, g_fsm_port);
	if (ret) {
		pr_err("get ports id fail:%d", ret);
		return ret;
	}
	header.param_id  = CAPI_V2_PARAM_FSADSP_RE25;
	header.opcode    = opcode;
	if (opcode == FSM_OPCODE_SET_PARAMS) {
		ret = fsm_read_efsdata(calib_data);
		if (ret) {
			pr_err("read efs data fail:%d", ret);
			calib_data->dev_count = 0;
			return ret;
		}
	}
	pr_info("re25:%d", calib_data->cal_data[0].cal_re);
	memset(&params, 0, sizeof(struct fsadsp_cmd_re25));
	params.version = FSADSP_RE25_CMD_VERSION_V1;
	params.ndev = 1;
	params.cal_data[0].rstrim = 0;
	params.cal_data[0].channel = 0; // mono
	params.cal_data[0].re25 = calib_data->cal_data[0].cal_re;
	data.size = sizeof(struct fsadsp_cmd_re25);
	data.buf = &params;
	ret = afe_send_fsm_apr_v2(&header, &data);
	if (ret) {
		pr_err("send firmware fail:%d", ret);
		return ret;
	}

	return ret;
}

int fsm_afe_mode_ctrl(uint32_t *mode, int opcode)
{
	struct afe_fsm_header header;
	struct fsm_msg data;
	int ret;

	if (mode == NULL) {
		return -EINVAL;
	}
	header.module_id = AFE_MODULE_ID_FSADSP_RX;
	ret = fsm_get_ports_id(&header, g_fsm_port);
	if (ret) {
		pr_err("get ports id fail:%d", ret);
		return ret;
	}
	header.param_id  = CAPI_V2_PARAM_FSADSP_RE25;
	header.opcode    = opcode;
	data.size = sizeof(uint32_t);
	data.buf = mode;
	ret = afe_send_fsm_apr_v2(&header, &data);
	if (ret) {
		pr_err("send firmware fail:%d", ret);
		return ret;
	}

	return ret;
}

int fsm_afe_livedata_ctrl(fsm_msg_t *data, int opcode)
{
	struct afe_fsm_header header;
	int ret;

	if (!data) {
		return -EINVAL;
	}
	header.module_id = AFE_MODULE_ID_FSADSP_RX;
	ret = fsm_get_ports_id(&header, g_fsm_port);
	if (ret) {
		pr_err("get ports id fail:%d", ret);
		return ret;
	}
	header.param_id = CAPI_V2_PARAM_FSADSP_LIVEDATA;
	header.opcode   = opcode;
	ret = afe_send_fsm_apr_v2(&header, data);
	if (ret) {
		pr_err("send apr error:%d", ret);
	}

	return ret;
}

int fsm_afe_cfg_ctrl(fsm_msg_t *data, int opcode)
{
	struct afe_fsm_header header;
	int ret;

	if (!data) {
		return -EINVAL;
	}
	header.module_id = AFE_MODULE_ID_FSADSP_RX;
	ret = fsm_get_ports_id(&header, g_fsm_port);
	if (ret) {
		pr_err("get ports id fail:%d", ret);
		return ret;
	}

	header.param_id = CAPI_V2_PARAM_FSADSP_CFG;
	header.opcode   = opcode;
	if (opcode == FSM_OPCODE_SET_PARAMS) {
		ret = fsm_set_cfg_data(data);
	}
	ret |= afe_send_fsm_apr_v2(&header, data);
	if (ret) {
		pr_err("send apr error:%d", ret);
	}

	return ret;
}

int fsm_send_afe_cal(void *buffer)
{
	struct afe_fsm_header header;
	struct fsm_msg data;
	uint32_t data_size;
	uint32_t *buf;
	int ret;

	if (!buffer) {
		return -EINVAL;
	}
	buf = (uint32_t *)buffer;
	data_size = buf[0] - FSM_AFE_HEADER_LEN * sizeof(uint32_t);
	header.module_id = buf[2];
	header.param_id = buf[3];
	header.opcode   = buf[4];
	ret = fsm_get_ports_id(&header, g_fsm_port);
	if (ret) {
		pr_err("get ports id fail:%d", ret);
		return ret;
	}
	data.size = data_size;
	data.buf = &buf[FSM_AFE_HEADER_LEN];
	ret = afe_send_fsm_apr_v2(&header, &data);
	if (ret) {
		pr_err("send apr error:%d", ret);
	}
	return ret;
}

static int fsm_afe_module_ctrl(bool enable)
{
	struct afe_fsm_params fsm_params;
	struct afe_fsm_header header;
	struct fsm_msg data;
	uint32_t param;
	int ret;

	header.module_id = AFE_MODULE_ID_FSADSP_RX;
	ret = fsm_get_ports_id(&header, g_fsm_port);
	if (ret) {
		pr_err("get ports id fail:%d", ret);
		return ret;
	}

	// enable/disable module
	header.param_id  = CAPI_V2_PARAM_FSADSP_MODULE_ENABLE;
	header.opcode    = FSM_OPCODE_SET_PARAMS;
	param = enable;
	data.size = sizeof(param);
	data.buf = &param;
	ret = afe_send_fsm_apr_v2(&header, &data);
	if (ret) {
		pr_err("%s module fail:%d", param ? "enable" : "disable", ret);
		return ret;
	}
	if (!param) {
		return ret;
	}
	// old version cmd: set mode and re25
	// param = fsm_get_config()->next_scene;
	// ret |= fsm_afe_mode_ctrl(&param, FSM_OPCODE_SET_PARAMS);
	ret |= fsm_afe_re25_ctrl(&g_calib_data, FSM_OPCODE_SET_PARAMS);
	return ret;

	data.size = sizeof(fsm_params.payload);
	data.buf = fsm_params.payload;
	ret |= fsm_afe_cfg_ctrl(&data, FSM_OPCODE_SET_PARAMS);

	return ret;
}

#else
#define fsm_set_iv_fb_ports(...)   (-1010)
#define fsm_afe_re25_ctrl(...)     (-1010)
#define fsm_afe_livedata_ctrl(...) (-1010)
#define fsm_send_afe_cal(...)      (-1010)
#define fsm_afe_module_ctrl(...)   (-1010)
#endif

void fsm_algo_reset_calib_data(void)
{
	memset(&g_calib_data, 0, sizeof(struct fsm_calib_v2));
}

int fsm_algo_send_afe_cal(void *buf)
{
	int ret;

	if (!buf) {
		return -EINVAL;
	}
	fsm_mutex_lock();
	ret = fsm_send_afe_cal(buf);
	fsm_mutex_unlock();

	return ret;
}

int fsm_algo_get_livedata(fsm_msg_t *data)
{
	if (!data) {
		return -EINVAL;
	}
	//fsm_mutex_lock();
	return fsm_afe_livedata_ctrl(data, FSM_OPCODE_GET_PARAMS);
	//fsm_mutex_unlock();
}

void fsm_algo_module_ctrl(bool enable)
{
	int ret;

	fsm_mutex_lock();
	ret = fsm_afe_module_ctrl(enable);
	if (ret) {
		pr_err("control fail:%d", ret);
	}
	fsm_mutex_unlock();
}

static const char *fsm_algo_status_text[] = {
	"None", "Disable", "Enable"
};

static const struct soc_enum fsm_algo_status_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(fsm_algo_status_text),
			fsm_algo_status_text)
};

static const char *fsm_algo_vi_tx_port_text[] = {
	"ZERO", "PRI_MI2S", "SEC_MI2S", "TERT_MI2S",
	"QUAT_MI2S", "QUIN_MI2S"
};

static const struct soc_enum fsm_algo_vi_tx_port_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(fsm_algo_vi_tx_port_text),
			fsm_algo_vi_tx_port_text)
};

static const char *fsm_algo_tx_fmt_text[] = {
	// sync to fsm_tx_fmt @ fsm-dev.h
	"None", "I2S_S16", "I2S_S32", "TDM_MONO", "TDM_STEREO"
};

static const struct soc_enum fsm_algo_tx_fmt_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(fsm_algo_tx_fmt_text),
			fsm_algo_tx_fmt_text)
};

static int fsm_algo_status_get(struct snd_kcontrol *pKcontrol,
				struct snd_ctl_elem_value *pUcontrol)
{
	uint32_t index = atomic_read(&g_fsm_algo_status);

	pUcontrol->value.integer.value[0] = index;
	pr_info("algo_status[%d]: %s", index, fsm_algo_status_text[index]);

	return 0;
}

static int fsm_algo_status_set(struct snd_kcontrol *pKcontrol,
				struct snd_ctl_elem_value *pUcontrol)
{
	int index = pUcontrol->value.integer.value[0];

	if (index < 0 || index >= ARRAY_SIZE(fsm_algo_status_text)) {
		pr_err("invalid index:%d", index);
		return -EINVAL;
	}

	pr_info("algo_status[%d]: %s", index, fsm_algo_status_text[index]);
	switch (index) {
	case 1: // disable module
		fsm_algo_module_ctrl(false);
		break;
	case 2: // init and enable module
		fsm_algo_module_ctrl(true);
		break;
	case 0: // none status
	default:
		pr_err("invalid index:%d", index);
		break;
	}
	atomic_set(&g_fsm_algo_status, index);

	return 0;
}

static int fsm_algo_tx_port_get(struct snd_kcontrol *pKcontrol,
				struct snd_ctl_elem_value *pUcontrol)
{
	int index = atomic_read(&g_fsm_algo_vi_tx_port);
	pUcontrol->value.integer.value[0] = index;
	pr_info("TX: %s", fsm_algo_vi_tx_port_text[index]);

	return 0;
}

static int fsm_algo_tx_port_set(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int port = ucontrol->value.integer.value[0];
	// fsm_config_t *cfg = fsm_get_config();
	// bool stereo;
	int ret;

	// stereo = ((cfg->dev_count > 1) ? true : false);
	// pr_info("set TX: %s, stereo: %d", fsm_algo_vi_tx_port_text[port], stereo);
	pr_info("set TX: %s", fsm_algo_vi_tx_port_text[port]);
	ret = fsm_set_iv_fb_ports(port, false);
	if (ret) {
		pr_err("error:%d", ret);
		atomic_set(&g_fsm_algo_vi_tx_port, 0);
		return ret;
	}
	atomic_set(&g_fsm_algo_vi_tx_port, port);

	return 0;
}

static int fsm_algo_tx_fmt_get(struct snd_kcontrol *pKcontrol,
				struct snd_ctl_elem_value *pUcontrol)
{
	fsm_config_t *cfg = fsm_get_config();
	int index = atomic_read(&g_fsm_algo_tx_fmt);

	pUcontrol->value.integer.value[0] = index;
	pr_info("format: %s, tx_fmt:%d", fsm_algo_tx_fmt_text[index], cfg->tx_fmt);

	return 0;
}

static int fsm_algo_tx_fmt_set(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int index = ucontrol->value.integer.value[0];

	pr_info("TX fmt: %s", fsm_algo_tx_fmt_text[index]);
	fsm_set_tx_format(index);
	atomic_set(&g_fsm_algo_tx_fmt, index);

	return 0;
}

static const struct snd_kcontrol_new fsm_algo_afe_controls[] =
{
	SOC_ENUM_EXT("FSM_Algo_Status", fsm_algo_status_enum[0],
		fsm_algo_status_get, fsm_algo_status_set),
	SOC_ENUM_EXT("FSM_VI_TX_Port", fsm_algo_vi_tx_port_enum[0],
		fsm_algo_tx_port_get, fsm_algo_tx_port_set),
	SOC_ENUM_EXT("FSM_VI_TX_Format", fsm_algo_tx_fmt_enum[0],
		fsm_algo_tx_fmt_get, fsm_algo_tx_fmt_set)
};

int fsm_algo_init(struct snd_soc_codec *codec)
{
	int ret;

	if (!codec) {
		pr_err("invalid parameters");
		return -EINVAL;
	}
	atomic_set(&g_fsm_algo_status, 0);
	atomic_set(&g_fsm_algo_scene, 0);
	atomic_set(&g_fsm_algo_vi_tx_port, 0);
	atomic_set(&g_fsm_algo_tx_fmt, 0);
	ret = snd_soc_add_codec_controls(codec, fsm_algo_afe_controls,
			ARRAY_SIZE(fsm_algo_afe_controls));
	if (ret) {
		pr_err("add condec controls error:%d", ret);
	}
	// fsm_read_efsdata(&g_calib_data);

	return ret;
}

int fsm_algo_deinit(struct snd_soc_codec *codec)
{
	return 0;
}
#endif
