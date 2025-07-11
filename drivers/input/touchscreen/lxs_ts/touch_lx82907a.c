/*
 * Copyright 2024 Sony Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */
// SPDX-License-Identifier: GPL-2.0
/*
 * LXS touch entry for LX82907A
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "lxs_ts.h"

#define MODE_ALLOWED_LX82907A		(0 |	\
									LCD_MODE_BIT_U0 |	\
									LCD_MODE_BIT_U3 |	\
									LCD_MODE_BIT_STOP |	\
									0)

#define BUS_TYPE_LX82907A			BUS_I2C

static const struct lxs_ts_reg_quirk reg_quirks_tbl_lx82907a[] = {
	{ .old_addr = INFO_PTR, .new_addr = 0x020, },
	{ .old_addr = SPR_CHIP_TEST, .new_addr = 0x022, },
	{ .old_addr = SPR_RST_CTL, .new_addr = 0x004, },
	/* */
	{ .old_addr = SERIAL_DATA_OFFSET, .new_addr = 0x045, },
	/* */
	{ .old_addr = ~0, .new_addr = ~0 },		// End signal
};

#if defined(__LXS_SUPPORT_DISP_FRAME_RATE)
enum {
	/* NP */
	TS_TOUCH_FRAME_60HZ = 1,	// 60Hz/60Hz (Active/Doze)
	TS_TOUCH_FRAME_120HZ,		// 120Hz/120Hz
	TS_TOUCH_FRAME_240HZ_H,		// 240Hz/120Hz
	TS_TOUCH_FRAME_240HZ,		// 240Hz/240Hz
	/* LP */
	TS_TOUCH_FRAME_L60HZ = 5,	// 60Hz/30Hz
	TS_TOUCH_FRAME_L120HZ,		// 120Hz/30Hz
};

enum {
	TS_DISP_FRAME_ASYNC = 0,
	TS_DISP_FRAME_30HZ,
	TS_DISP_FRAME_60HZ,
	TS_DISP_FRAME_120HZ,
};

static const int __d_frame_set_as[] = {
	TS_DISP_FRAME_ASYNC,
	-1,
};

static const int __d_frame_set_np[] = {
	TS_DISP_FRAME_ASYNC,
	TS_DISP_FRAME_60HZ,
	TS_DISP_FRAME_120HZ,
	-1,
};

static const int __d_frame_set_lp[] = {
	TS_DISP_FRAME_ASYNC,
	TS_DISP_FRAME_30HZ,
	-1,
};

static const int __t_frame_set_np[] = {
	TS_TOUCH_FRAME_60HZ,
	TS_TOUCH_FRAME_120HZ,
	TS_TOUCH_FRAME_240HZ_H,
	TS_TOUCH_FRAME_240HZ,
	-1,
};

static const int __t_frame_set_lp[] = {
	TS_TOUCH_FRAME_L60HZ,
	TS_TOUCH_FRAME_L120HZ,
	-1,
};

const int *__d_frame_tbl_np;
const int *__d_frame_tbl_lp;
const int *__t_frame_tbl_np;
const int *__t_frame_tbl_lp;

static int touch_lookup_frame_tbl(const int *table, int value)
{
	int val = 0;

	if (table == NULL)
		return -1;

	while (1) {
		val = *table++;
		if (value == val)
			return val;
		if (val < 0)
			break;
	}
	return -1;
}

static void __used lx82907a_setup(struct lxs_ts *ts)
{
	struct device_node *np = ts->dev->of_node;
	int frame_tbl_type = 0;
	u32 val = 0;

	if (!of_property_read_u32(np, "frame_tbl_type", &val))
		frame_tbl_type = val;

	if (frame_tbl_type)
		t_dev_info(ts->dev, " frame_tbl_type  = %d\n", frame_tbl_type);

	t_dev_info(ts->dev, "%s\n", __func__);

	ts->d_frame_rate_np = TS_DISP_FRAME_60HZ;
	ts->d_frame_rate_lp = TS_DISP_FRAME_ASYNC;
	ts->d_frame_rate_off = TS_DISP_FRAME_ASYNC;
	ts->d_frame_rate_aod = TS_DISP_FRAME_30HZ;

	ts->t_frame_rate_np = TS_TOUCH_FRAME_120HZ;
	ts->t_frame_rate_lp = TS_TOUCH_FRAME_L120HZ;

	__d_frame_tbl_np = __d_frame_set_np;
	__d_frame_tbl_lp = __d_frame_set_lp;
	__t_frame_tbl_np = __t_frame_set_np;
	__t_frame_tbl_lp = __t_frame_set_lp;

	switch (frame_tbl_type) {
	case 1:
		ts->d_frame_rate_np = TS_DISP_FRAME_ASYNC;
		ts->d_frame_rate_lp = TS_DISP_FRAME_ASYNC;
		ts->d_frame_rate_off = TS_DISP_FRAME_ASYNC;
		ts->d_frame_rate_aod = TS_DISP_FRAME_ASYNC;

		__d_frame_tbl_np = __d_frame_set_as;
		__d_frame_tbl_lp = __d_frame_set_as;
		break;
	}
}

static int lx82907a_disp_frame_rate(struct lxs_ts *ts, int mode)
{
	struct lxs_ts_chip *chip = &ts->chip;
	struct lxs_hal_reg *reg = chip->reg;
	int is_resume = !!(ts->state_mode == TS_MODE_RESUME);
	int is_u3 = !!(mode == LCD_MODE_U3);
	const int *d_frame_tbl = (is_u3) ? __d_frame_tbl_np : __d_frame_tbl_lp;
	int value = (is_u3) ? ts->d_frame_rate_np : ts->d_frame_rate_lp;

	if (mode == LCD_MODE_STOP)
		return 0;

	if (lxs_addr_is_invalid(reg->frame_rate))
		return 0;

	//AOD or OFF(EWU)
	if (!is_u3) {
		int __value = (is_resume) ? ts->d_frame_rate_aod : ts->d_frame_rate_off;
		if (value != __value)
			t_dev_dbg_trace(ts->dev, "display frame rate: invalid %d\n", value);
		value = __value;
	}

	if (touch_lookup_frame_tbl(d_frame_tbl, value) < 0)
		return 0;

	t_dev_info(ts->dev, "%s: %d\n", __func__, value);

	lxs_hal_write_value(ts, reg->frame_rate, value);

	return 0;
}

static int lx82907a_touch_frame_rate(struct lxs_ts *ts, int mode)
{
	struct lxs_ts_chip *chip = &ts->chip;
	struct lxs_hal_reg *reg = chip->reg;
	int is_u3 = !!(mode == LCD_MODE_U3);
	const int *t_frame_tbl = (is_u3) ? __t_frame_tbl_np : __t_frame_tbl_lp;
	int value = (is_u3) ? ts->t_frame_rate_np : ts->t_frame_rate_lp;

	if (mode == LCD_MODE_STOP)
		return 0;

	if (lxs_addr_is_invalid(reg->t_frame_rate))
		return 0;

	if (touch_lookup_frame_tbl(t_frame_tbl, value) < 0)
		return 0;

	t_dev_info(ts->dev, "%s: %d\n", __func__, value);

	lxs_hal_write_value(ts, reg->t_frame_rate, value);

	return 0;
}

static int __used lx82907a_fps_set(struct lxs_ts *ts, int mode)
{
	lx82907a_disp_frame_rate(ts, mode);
	lx82907a_touch_frame_rate(ts, mode);
	return 0;
}

static ssize_t show_frame_table(struct device *dev, char *buf)
{
	struct lxs_ts *ts = dev_get_drvdata(dev);
	char *sbuf = NULL;
	const int *d_frame_tbl_np = __d_frame_tbl_np;
	const int *d_frame_tbl_lp = __d_frame_tbl_lp;
	const int *t_frame_tbl_np = __t_frame_tbl_np;
	const int *t_frame_tbl_lp = __t_frame_tbl_lp;
	int slen = 128;
	int size = 0;
	int l_sz = 0;
	int val = 0;

	sbuf = (char *)kzalloc(slen, GFP_KERNEL);
	if (sbuf == NULL)
		return (size_t)size;

	if (d_frame_tbl_np) {
		l_sz = __lxs_snprintf(sbuf, slen, 0, "d: np ");
		while (1) {
			val = *d_frame_tbl_np++;
			if (val < 0)
				break;
			l_sz += __lxs_snprintf(sbuf, slen, l_sz, "%d ", val);
		}
	}

	if (d_frame_tbl_lp) {
		l_sz += __lxs_snprintf(sbuf, slen, l_sz, "| lp ");
		while (1) {
			val = *d_frame_tbl_lp++;
			if (val < 0)
				break;
			l_sz += __lxs_snprintf(sbuf, slen, l_sz, "%d ", val);
		}
	}

	l_sz += __lxs_snprintf(sbuf, slen, l_sz, "(off %d, aod %d)",
		ts->d_frame_rate_off, ts->d_frame_rate_aod);

	size += lxs_snprintf(buf, size, "%s\n", sbuf);
	t_dev_info(dev, "%s: %s\n", __func__, sbuf);

	memset(sbuf, 0, slen);
	l_sz = 0;

	if (t_frame_tbl_np) {
		l_sz = __lxs_snprintf(sbuf, slen, 0, "t: np ");
		while (1) {
			val = *t_frame_tbl_np++;
			if (val < 0)
				break;
			l_sz += __lxs_snprintf(sbuf, slen, l_sz, "%d ", val);
		}
	}

	if (__t_frame_tbl_lp) {
		l_sz += __lxs_snprintf(sbuf, slen, l_sz, "| lp ");
		while (1) {
			val = *t_frame_tbl_lp++;
			if (val < 0)
				break;
			l_sz += __lxs_snprintf(sbuf, slen, l_sz, "%d ", val);
		}
	}

	if (l_sz) {
		size += lxs_snprintf(buf, size, "%s\n", sbuf);
		t_dev_info(dev, "%s: %s\n", __func__, sbuf);
	}

	memset(sbuf, 0, slen);
	l_sz = __lxs_snprintf(sbuf, slen, 0, "c: d %d %d | t %d %d",
		ts->d_frame_rate_np, ts->d_frame_rate_lp, ts->t_frame_rate_np, ts->t_frame_rate_lp);

	size += lxs_snprintf(buf, size, "%s\n", sbuf);
	t_dev_info(dev, "%s: %s\n", __func__, sbuf);

	kfree(sbuf);

	return (size_t)size;
}
static TS_ATTR(frame_table, show_frame_table, NULL);

static ssize_t __show_frame_rate(struct device *dev, char *buf, int mode)
{
	struct lxs_ts *ts = dev_get_drvdata(dev);
	int is_u3 = (mode == LCD_MODE_U3);
	char *str = (is_u3) ? "NP" : "LP";
	int d_rate = (is_u3) ? ts->d_frame_rate_np : ts->d_frame_rate_lp;
	int t_rate = (is_u3) ? ts->t_frame_rate_np : ts->t_frame_rate_lp;
	int size = 0;

	t_dev_info(dev, "%s: %s: disp %d, touch %d\n", __func__, str, d_rate, t_rate);

	size += lxs_snprintf(buf, size, "%d,%d\n", d_rate, t_rate);

	return (ssize_t)size;
}

static ssize_t __store_frame_rate(struct device *dev,
				const char *buf, size_t count, int mode)
{
	struct lxs_ts *ts = dev_get_drvdata(dev);
	struct lxs_ts_chip *chip = &ts->chip;
	int is_u3 = (mode == LCD_MODE_U3);
	char *str = (is_u3) ? "NP" : "LP";
	const int *d_frame_tbl = (is_u3) ? __d_frame_tbl_np : __d_frame_tbl_lp;
	const int *t_frame_tbl = (is_u3) ? __t_frame_tbl_np : __t_frame_tbl_lp;
	int *d_frame_rate = (is_u3) ? &ts->d_frame_rate_np : &ts->d_frame_rate_lp;
	int *t_frame_rate = (is_u3) ? &ts->t_frame_rate_np : &ts->t_frame_rate_lp;
	int d_idx = 0;
	int t_idx = 0;
	int d_idx_old = 0;
	int t_idx_old = 0;
	int tc_driving = 1;
	int d_valid = 0;
	int t_valid = 0;

	if (ts->state_core != TS_CORE_NORMAL)
		return count;

	if (sscanf(buf, "%d %d", &d_idx, &t_idx) <= 0)
		return count;

	d_valid = (touch_lookup_frame_tbl(d_frame_tbl, d_idx) >= 0);
	if (!d_valid)
		t_dev_err(dev, "%s: %s: invalid display index %d\n",
			__func__, str, d_idx);

	t_valid = (touch_lookup_frame_tbl(t_frame_tbl, t_idx) >= 0);
	if (!t_valid)
		t_dev_err(dev, "%s: %s: invalid touch index %d\n",
			__func__, str, t_idx);

	if (!d_valid || !t_valid)
		return count;

	if ((*d_frame_rate == d_idx) && (*t_frame_rate == t_idx)) {
		t_dev_info(dev, "%s: display index %d, touch index %d\n",
			__func__, d_idx, t_idx);
		return count;
	}

	mutex_lock(&ts->lock);

	d_idx_old = *d_frame_rate;
	t_idx_old = *t_frame_rate;

	*d_frame_rate = d_idx;
	*t_frame_rate = t_idx;

	t_dev_info(dev, "%s: %s display index %d(%d), touch index %d(%d)\n",
		__func__, str, d_idx, d_idx_old, t_idx, t_idx_old);

	if (!ts->probe_done)
		tc_driving = 0;
	else if (mode != chip->driving_mode)
		tc_driving = 0;

	if (tc_driving)
		lxs_hal_tc_driving(ts, -1);
	else
		t_dev_info(dev, "%s: skip tc_driving\n", __func__);

	mutex_unlock(&ts->lock);

	return count;
}

#define TS_ATTR_FRAME_RATE(_name, _mode)	\
static ssize_t show_frame_rate_##_name(struct device *dev, char *buf)	\
{	\
	return __show_frame_rate(dev, buf, _mode);	\
}	\
static ssize_t store_frame_rate_##_name(struct device *dev, const char *buf, size_t count)	\
{	\
	return __store_frame_rate(dev, buf, count, _mode);	\
}	\
static TS_ATTR(frame_rate_##_name, show_frame_rate_##_name, store_frame_rate_##_name)

TS_ATTR_FRAME_RATE(np, LCD_MODE_U3);
TS_ATTR_FRAME_RATE(lp, LCD_MODE_U0);
#endif	/* __LXS_SUPPORT_DISP_FRAME_RATE */

#if defined(__LXS_SUPPORT_LPWG)
static ssize_t show_enable_lpm(struct device *dev, char *buf)
{
	struct lxs_ts *ts = dev_get_drvdata(dev);
	struct lxs_ts_chip *chip = &ts->chip;

	return lxs_snprintf(buf, 0, "%d\n", chip->enable_aod);
}

static ssize_t store_enable_lpm(struct device *dev,
				const char *buf, size_t count)
{
	struct lxs_ts *ts = dev_get_drvdata(dev);
	struct lxs_ts_chip *chip = &ts->chip;
	int value;

	if (ts->state_core != TS_CORE_NORMAL)
		return count;

	if (kstrtoint(buf, 10, &value))
		return count;

	if (!ts->plat_data->use_lpwg) {
		t_dev_info(dev, "use_lpwg blocked\n");
		return count;
	}

	mutex_lock(&ts->lock);

	chip->enable_aod = value;

	t_dev_info(dev, "set_lowpower_mode: %d\n", chip->enable_aod);

	mutex_unlock(&ts->lock);

	return count;
}
static TS_ATTR(set_lowpower_mode, show_enable_lpm, store_enable_lpm);
#endif	/* __LXS_SUPPORT_LPWG */

#if defined(__LXS_SUPPORT_FILTER_CON)
struct lxs_ts_filter_ctrl {
	char *name;
	u32 addr;
	int *value;
};

static int lx82907a_chk_filter(struct lxs_ts *ts, int idx, int wr)
{
	struct lxs_ts_filter_ctrl filter_table_wr[] = {
		{	.name = "blend_filter",
			.addr = 0xD10,
			.value = &ts->blend_filter,
		},
		{	.name = "stop_filter",
			.addr = 0xD11,
			.value = &ts->stop_filter,
		},
	};
	struct lxs_ts_filter_ctrl filter_table_rd[] = {
		{	.name = "wet_mode",
			.addr = 0xD12,
			.value = &ts->wet_mode,
		},
	};
	struct lxs_ts_filter_ctrl *filter;
	int idx_ovr = (wr) ? ARRAY_SIZE(filter_table_wr) : ARRAY_SIZE(filter_table_rd);
	int ret = 0;

	if ((idx < 0) || (idx >= idx_ovr))
		return -EINVAL;

	if (wr) {
		filter = &filter_table_wr[idx];

		ret = lxs_hal_write_value(ts, filter->addr, *filter->value);
		if (ret < 0) {
			t_dev_err(ts->dev, "%s(set): failed to write\n", filter->name);
			return ret;
		}

		t_dev_info(ts->dev, "%s(set): %d\n", filter->name, *filter->value);
	} else {
		filter = &filter_table_rd[idx];

		ret = lxs_hal_read_value(ts, filter->addr, filter->value);
		if (ret < 0) {
			t_dev_err(ts->dev, "%s(get): failed to read\n", filter->name);
			return ret;
		}

		t_dev_info(ts->dev, "%s(get): %d\n", filter->name, *filter->value);
	}

	return 0;
}

#define lx82907a_blend_filter(_ts)	lx82907a_chk_filter(_ts, 0, 1)
#define lx82907a_stop_filter(_ts)	lx82907a_chk_filter(_ts, 1, 1)
#define lx82907a_wet_mode(_ts)		lx82907a_chk_filter(_ts, 0, 0)

static int lx82907a_tc_restore(struct lxs_ts *ts)
{
	lx82907a_blend_filter(ts);
	lx82907a_stop_filter(ts);
	return 0;
}

static ssize_t show_blend_filter(struct device *dev, char *buf)
{
	struct lxs_ts *ts = dev_get_drvdata(dev);
	int filter = ts->blend_filter;

	t_dev_info(ts->dev, "%s: %d\n", __func__, filter);

	return lxs_snprintf(buf, 0, "%d\n", filter);
}

/*
 * blend filter
 * 0 : off, 1 : weak, 2 : normal, 3 : strong
 */
static ssize_t store_blend_filter(struct device *dev,
				const char *buf, size_t count)
{
	struct lxs_ts *ts = dev_get_drvdata(dev);
	int value;

	if (ts->state_core != TS_CORE_NORMAL)
		return count;

	if (kstrtoint(buf, 10, &value))
		return count;

	if (value < 0 || value > 3) {
		t_dev_err(ts->dev, "%s: invalid %d\n", __func__, value);
		return count;
	}

	t_dev_info(dev, "%s: value %d\n", __func__, value);

	mutex_lock(&ts->lock);

	ts->blend_filter = value;

	lx82907a_blend_filter(ts);

	mutex_unlock(&ts->lock);

	return count;
}
static TS_ATTR(blend_filter, show_blend_filter, store_blend_filter);

static ssize_t show_stop_filter(struct device *dev, char *buf)
{
	struct lxs_ts *ts = dev_get_drvdata(dev);
	int filter = ts->stop_filter;

	t_dev_info(ts->dev, "%s: %d\n", __func__, filter);

	return lxs_snprintf(buf, 0, "%d\n", filter);
}

/*
 * stop filter
 * 0 : off, 1 : weak, 2 : normal, 3 : strong
 */
static ssize_t store_stop_filter(struct device *dev,
				const char *buf, size_t count)
{
	struct lxs_ts *ts = dev_get_drvdata(dev);
	int value;

	if (ts->state_core != TS_CORE_NORMAL)
		return count;

	if (kstrtoint(buf, 10, &value))
		return count;

	if (value < 0 || value > 3) {
		t_dev_err(ts->dev, "%s: invalid %d\n", __func__, value);
		return count;
	}

	t_dev_info(dev, "%s: value %d\n", __func__, value);

	mutex_lock(&ts->lock);

	ts->stop_filter = value;

	lx82907a_stop_filter(ts);

	mutex_unlock(&ts->lock);

	return count;
}
static TS_ATTR(stop_filter, show_stop_filter, store_stop_filter);

static ssize_t show_wet_mode(struct device *dev, char *buf)
{
	struct lxs_ts *ts = dev_get_drvdata(dev);

	mutex_lock(&ts->lock);

	lx82907a_wet_mode(ts);

	mutex_unlock(&ts->lock);

	return lxs_snprintf(buf, 0, "%d\n", ts->wet_mode);
}
static TS_ATTR(wet_mode, show_wet_mode, NULL);
#endif	/* __LXS_SUPPORT_FILTER_CON */

#if defined(__SOMC_CUSTOM_AFTER_PROBE)
static ssize_t after_probe_start_store(struct device *dev,
				const char *buf, size_t count)
{
	struct lxs_ts *ts = dev_get_drvdata(dev);

	t_dev_info(dev, "after probe start \n");
	if (ts->probe_done) {
		t_dev_info(dev, "already post probed, need reboot\n");
	} else {
		t_dev_info(dev, "start init work\n");
		queue_delayed_work(ts->wq, &ts->after_probe.start, 0);
	}

	return count;
}
static TS_ATTR(after_probe, NULL, after_probe_start_store);
#endif	/* __SOMC_CUSTOM_AFTER_PROBE */

#if defined(__SOMC_CUSTOM_INPUT_CTRL)
static ssize_t show_rejection_mode(struct device *dev, char *buf)
{
	struct lxs_ts *ts = dev_get_drvdata(dev);

	t_dev_info(ts->dev, "rejection_mode: mode %d, flag %d\n",
		ts->rejection_mode, ts->report_rejected_event_flag);

	return lxs_snprintf(buf, 0, "mode %d, flag %d\n",
		ts->rejection_mode, ts->report_rejected_event_flag);
}

static ssize_t store_rejection_mode(struct device *dev,
				const char *buf, size_t count)
{
	struct lxs_ts *ts = dev_get_drvdata(dev);
	int value, old;

	if (kstrtoint(buf, 10, &value))
		return count;

	if (value < 0 || value > 1) {
		t_dev_err(dev, "rejection_mode: param out of range\n");
		return count;
	}

	mutex_lock(&ts->lock);

	old = ts->rejection_mode;
	ts->rejection_mode = value;

	ts->report_rejected_event_flag = (value) ? false : true;

	t_dev_info(dev, "rejection_mode: mode %d(%d), flag %d\n",
		ts->rejection_mode, old, ts->report_rejected_event_flag);

	mutex_unlock(&ts->lock);

	return count;
}
static TS_ATTR(rejection_mode, show_rejection_mode, store_rejection_mode);

static ssize_t show_orientation_change(struct device *dev, char *buf)
{
	struct lxs_ts *ts = dev_get_drvdata(dev);

	t_dev_info(ts->dev, "orientation_change: landscape %d\n", ts->landscape);

	return lxs_snprintf(buf, 0, "landscape %d\n", ts->landscape);
}

static ssize_t store_orientation_change(struct device *dev,
				const char *buf, size_t count)
{
	struct lxs_ts *ts = dev_get_drvdata(dev);
	struct lxs_ts_chip *chip = &ts->chip;
	int value, old;

	if (kstrtoint(buf, 10, &value))
		return count;

	if (value < 0 || value > 1) {
		t_dev_err(dev, "orientation_change: param out of range\n");
		return count;
	}

	mutex_lock(&ts->lock);

	old = ts->landscape;
	ts->landscape = value;

	t_dev_info(dev, "orientation_change: landscape %d(%d)\n",
		ts->landscape, old);

	chip->grab = !value;
	lxs_hal_tc_con(ts, TCON_GRAB, NULL);

	mutex_unlock(&ts->lock);

	return count;
}
static TS_ATTR(orientation_change, show_orientation_change, store_orientation_change);

static void update_game_enhancer_grip_rejection_para(struct lxs_ts *ts, bool portrait, int location, int value)
{
	int portrait_offset = 4;

	if (portrait) {
		location -= portrait_offset;
		ts->radius_portrait[location] = value;
		ts->circle_range_p[location] = value * value;

		t_dev_info(ts->dev, "%s: radius_portrait[%d] = %d\n",
			__func__, location, value);
		t_dev_info(ts->dev, "%s: circle_range_p[%d] = %d\n",
			__func__, location, value * value);
	} else {
		ts->radius_landscape[location] = value;
		ts->circle_range_l[location] = value * value;

		t_dev_info(ts->dev, "%s: radius_landscape[%d] = %d\n",
			__func__, location, value);
		t_dev_info(ts->dev, "%s: circle_range_l[%d] = %d\n",
			__func__, location, value * value);
	}
}

static void update_grip_rejection_para(struct lxs_ts *ts, int *value)
{
	int i;

	if (value[0] == 6) {
		memcpy(ts->portrait_buffer, value + 1, sizeof(ts->portrait_buffer));
		for (i = 0; i < ARRAY_SIZE(ts->portrait_buffer); i++) {
			t_dev_info(ts->dev, "%s: portrait_buffer[%d] = %d\n",
				__func__, i, ts->portrait_buffer[i]);
		}
	} else {
		memcpy(ts->landscape_buffer, value + 1, sizeof(ts->landscape_buffer));
		for (i = 0; i < ARRAY_SIZE(ts->landscape_buffer); i++) {
			t_dev_info(ts->dev, "%s: landscape_buffer[%d] = %d\n",
				__func__, i, ts->landscape_buffer[i]);
		}
	}
}

static int __show_range_changer_log_func(struct lxs_ts *ts, char *pbuf, int psize,
			char *name, u32 *data, int len)
{
	char sbuf[64] = { 0, };
	int slen = 0;
	char *estr = ",";
	int i;

	for (i = 0; i < len; i++) {
		if (i == (len - 1))
			estr = "";
		slen += snprintf(sbuf + slen, sizeof(sbuf) - slen, "%d%s", data[i], estr);
	}

	t_dev_info(ts->dev, "range_changer: %18s[0...%d] = %s\n", name, len-1, sbuf);

	return lxs_snprintf(pbuf, psize, "%18s[0...%d] = %s\n", name, len-1, sbuf);
}

#define __show_range_changer_log(_ts, _pbuf, _psize, _name)	\
		__show_range_changer_log_func(_ts, _pbuf, _psize, #_name, _ts->_name, ARRAY_SIZE(_ts->_name))

static ssize_t show_range_changer(struct device *dev, char *buf)
{
	struct lxs_ts *ts = dev_get_drvdata(dev);
	int size = 0;

	size += __show_range_changer_log(ts, buf, size, portrait_buffer);

	size += __show_range_changer_log(ts, buf, size, radius_portrait);

	size += __show_range_changer_log(ts, buf, size, circle_range_p);

	size += __show_range_changer_log(ts, buf, size, landscape_buffer);

	size += __show_range_changer_log(ts, buf, size, radius_landscape);

	size += __show_range_changer_log(ts, buf, size, circle_range_l);

	return (ssize_t)size;
}

static ssize_t store_range_changer(struct device *dev,
				const char *buf, size_t count)
{
	struct lxs_ts *ts = dev_get_drvdata(dev);
	int value[5] = {0, };

	if (sscanf(buf, "%d, %d, %d, %d, %d", &value[0], &value[1], &value[2], &value[3], &value[4]) <= 0)
		return count;

	if (value[0] < 0|| value[0] > 7) {
		t_dev_err(dev, "range_changer: param out of range\n");
		return count;
	}

	mutex_lock(&ts->lock);

	if (value[0] <= 3)
		update_game_enhancer_grip_rejection_para(ts, false, value[0], value[1]);
	else if (value[0] <= 5)
		update_game_enhancer_grip_rejection_para(ts, true, value[0], value[1]);
	else
		update_grip_rejection_para(ts, value);

	mutex_unlock(&ts->lock);

	return count;
}
static TS_ATTR(range_changer, show_range_changer, store_range_changer);
#endif	/* __SOMC_CUSTOM_INPUT_CTRL */


static struct attribute *lxs_ts_attr_add[] = {
#if defined(__LXS_SUPPORT_DISP_FRAME_RATE)
	&ts_attr_frame_table.attr,
	&ts_attr_frame_rate_np.attr,
	&ts_attr_frame_rate_lp.attr,
#endif	/* __LXS_SUPPORT_DISP_FRAME_RATE */
#if defined(__LXS_SUPPORT_LPWG)
	&ts_attr_set_lowpower_mode.attr,
#endif
	/* */
#if defined(__LXS_SUPPORT_FILTER_CON)
	__ts_attr_def(blend_filter)
	__ts_attr_def(stop_filter)
	__ts_attr_def(wet_mode)
#endif
	/* */
#if defined(__SOMC_CUSTOM_AFTER_PROBE)
	__ts_attr_def(after_probe)
#endif
#if defined(__SOMC_CUSTOM_INPUT_CTRL)
	__ts_attr_def(range_changer)
	__ts_attr_def(orientation_change)
	__ts_attr_def(rejection_mode)
#endif
	/* */
	NULL,
};

static struct attribute_group lxs_ts_attr_group_add = {
	.attrs = lxs_ts_attr_add,
};

static const struct of_device_id of_match_tbl_lx82907a[] = {
	{ .compatible = "lxs,lx82907a" },
	{ },
};
MODULE_DEVICE_TABLE(of, of_match_tbl_lx82907a);

static const struct lxs_ts_entry_data ts_entry_data_lx82907a = {
	.chip_type			= CHIP_LX82907A,
	.of_match_table		= of_match_ptr(of_match_tbl_lx82907a),
	.chip_id			= "7601",
	.chip_name			= "LX82907A",
	.mode_allowed		= MODE_ALLOWED_LX82907A,
	.fw_size			= (128<<10),
	/* */
	.bus_type			= BUS_TYPE_LX82907A,
	/* */
	.reg_quirks			= reg_quirks_tbl_lx82907a,
	.fquirks = {
	#if defined(__LXS_SUPPORT_DISP_FRAME_RATE)
		.setup			= lx82907a_setup,
		.fps_set_pre	= lx82907a_fps_set,
	#endif
	#if defined(__LXS_SUPPORT_FILTER_CON)
		.tc_restore		= lx82907a_tc_restore,
	#endif
		.sysfs_group	= &lxs_ts_attr_group_add,
	},
};

struct lxs_ts_if_driver ts_if_drv_lx82907a = {
	.entry_data = &ts_entry_data_lx82907a,
};

static int __init lxs_ts_driver_init(void)
{
	t_pr_info("%s: LX82907A driver init - %s\n", LXS_TS_NAME, LXS_DRV_VERSION);
	return lxs_ts_register_driver(&ts_if_drv_lx82907a);
}

static void __exit lxs_ts_driver_exit(void)
{
	lxs_ts_unregister_driver(&ts_if_drv_lx82907a);
	t_pr_info("%s: LX82907A driver exit - %s\n", LXS_TS_NAME, LXS_DRV_VERSION);
}
module_init(lxs_ts_driver_init);
module_exit(lxs_ts_driver_exit);

MODULE_AUTHOR("kimhh@lxsemicon.com");
MODULE_DESCRIPTION("LXS Touch LX82907A Driver");
MODULE_VERSION(LXS_DRV_VERSION);
MODULE_LICENSE("GPL");


