/* drivers/input/touchscreen/max1187x.c
 *
 * Copyright (c)2013 Maxim Integrated Products, Inc.
 * Copyright (C) 2012-2013 Sony Mobile Communications AB.
 *
 * Driver Version: 3.1.8
 * Release Date: May 10, 2013
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#elif defined CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/firmware.h>
#include <linux/crc16.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/jiffies.h>
#include <asm/byteorder.h>
#include <linux/input/max1187x.h>
#include <linux/input/max1187x_config.h>

#ifdef pr_fmt
#undef pr_fmt
#define pr_fmt(fmt) MAX1187X_NAME "(%s:%d): " fmt, __func__, __LINE__
#endif

#define pr_info_if(a, b, ...) do { if (debug_mask & a) \
			pr_info(b, ##__VA_ARGS__);	\
			} while (0)
#define debugmask_if(a) (debug_mask & a)

#define NWORDS(a)    (sizeof(a) / sizeof(u16))
#define BYTE_SIZE(a) ((a) * sizeof(u16))
#define BYTEH(a)     ((a) >> 8)
#define BYTEL(a)     ((a) & 0xFF)

#define PDATA(a)      (ts->pdata->a)

static u16 debug_mask = 0x8;
#ifdef MAX1187X_LOCAL_PDATA
struct max1187x_pdata local_pdata = { };
#endif

#define PWR_OFF_WAIT_MIN_US  1000
#define PWR_OFF_WAIT_MAX_US  2000
#define PWR_ON_WAIT_MIN_US   1000
#define PWR_ON_WAIT_MAX_US   2000
#define PWR_RESET_WAIT_MS    30

#define MAX_FW_UPDATE_DEFAULT 0 /* flashing with dflt_cfg if fw is corrupted */
#define MAX_FW_UPDATE_FORCE   1 /* force flashing with dflt_cfg */

static const char * const fw_update_mode[] = {
	[MAX_FW_UPDATE_DEFAULT] = "default",
	[MAX_FW_UPDATE_FORCE] = "force",
};

struct report_reader {
	u16 report_id;
	u16 reports_passed;
	struct semaphore sem;
	int status;
};

struct data {
	struct max1187x_pdata *pdata;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct input_dev *input_pen;
	char phys[32];
#ifdef TOUCH_WAKEUP_FEATURE
	struct input_dev *input_dev_key;
	char phys_key[32];
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#elif defined CONFIG_FB
	struct notifier_block fb_notif;
#endif

	u8 is_suspended;
	struct regulator *vreg_touch_vdd;
	wait_queue_head_t waitqueue_all;
	struct workqueue_struct *wq;
	struct work_struct work_irq;
	u32 irq_receive_time;

	u16 chip_id;
	u16 config_id;

	struct mutex fw_mutex;
	struct mutex irq_mutex;
	struct mutex i2c_mutex;
	struct mutex report_mutex;
	struct semaphore report_sem;
	struct report_reader report_readers[MAX_REPORT_READERS];
	u8 report_readers_outstanding;

	u16 cmd_buf[CMD_LEN_MAX];
	u16 cmd_len;
	struct semaphore sema_cmd;
	struct work_struct work_cmd;

	struct semaphore sema_rbcmd;
	wait_queue_head_t waitqueue_rbcmd;
	u8 rbcmd_waiting;
	u8 rbcmd_received;
	u16 rbcmd_report_id;
	u16 rbcmd_rx_report[RPT_LEN_MAX];
	u16 rbcmd_rx_report_len;

	u16 rx_report[RPT_LEN_MAX]; /* with header */
	u16 rx_report_len;
	u16 rx_packet[RPT_LEN_PACKET_MAX + 1]; /* with header */
	u32 irq_count;
	u16 framecounter;
	u16 list_finger_ids;
	u8 fw_update_mode;
	u8 sysfs_created;
	u8 is_raw_mode;

	u16 button0:1;
	u16 button1:1;
	u16 button2:1;
	u16 button3:1;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void early_suspend(struct early_suspend *h);
static void late_resume(struct early_suspend *h);
#elif defined CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data);
#endif
static int vreg_configure(struct data *ts, int enable);

static void validate_fw(struct data *ts);
static int device_init(struct i2c_client *client);
static int device_deinit(struct i2c_client *client);

static int bootloader_enter(struct data *ts);
static int bootloader_exit(struct data *ts);
static int bootloader_get_crc(struct data *ts, u16 *crc16,
		u16 addr, u16 len, u16 delay);
static int bootloader_set_byte_mode(struct data *ts);
static int bootloader_erase_flash(struct data *ts);
static int bootloader_write_flash(struct data *ts, const u8 *image, u16 length);

static void propagate_report(struct data *ts, int status, u16 *report);
static int get_report(struct data *ts, u16 report_id, ulong timeout);
static void release_report(struct data *ts);
static int cmd_send(struct data *ts, u16 *buf, u16 len);
static int rbcmd_send_receive(struct data *ts, u16 *buf,
		u16 len, u16 report_id, u16 timeout);

#ifdef MAX1187X_PRESSURE_SHAPING
static u16 max1187x_sqrt(u32 num);
#endif

#if MAX1187X_TOUCH_REPORT_MODE == 2
#ifndef MAX1187X_REPORT_FAST_CALCULATION
static u16 binary_search(const u16 *array, u16 len, u16 val);
#ifndef MAX1187X_PRESSURE_SHAPING
static u16 max1187x_sqrt(u32 num);
#endif
static s16 max1187x_orientation(s16 x, s16 y);
#endif
#endif

static u8 init_state;

/* I2C communication */
/* debug_mask |= 0x1 for I2C RX communication */
static int i2c_rx_bytes(struct data *ts, u8 *buf, u16 len)
{
	int i, ret, written;
	char debug_string[DEBUG_STRING_LEN_MAX];
	do {
		ret = i2c_master_recv(ts->client, (char *) buf, (int) len);
	} while (ret == -EAGAIN);
	if (ret < 0) {
		pr_err("I2C RX fail (%d)", ret);
		return ret;
	}

	len = ret;

	if (debugmask_if(1)) {
		pr_info("I2C RX (%d):", len);
		written = 0;
		for (i = 0; i < len; i++) {
			written += snprintf(debug_string + written, 6,
					"0x%02X,", buf[i]);
			if (written + 6 >= DEBUG_STRING_LEN_MAX) {
				pr_info("%s", debug_string);
				written = 0;
			}
		}
		if (written > 0)
			pr_info("%s", debug_string);
	}

	return len;
}

static int i2c_rx_words(struct data *ts, u16 *buf, u16 len)
{
	int i, ret, written;
	char debug_string[DEBUG_STRING_LEN_MAX];

	do {
		ret = i2c_master_recv(ts->client,
			(char *) buf, (int) (len * 2));
	} while (ret == -EAGAIN);
	if (ret < 0) {
		pr_err("I2C RX fail (%d)", ret);
		return ret;
	}

	if ((ret % 2) != 0) {
		pr_err("I2C words RX fail: odd number of bytes (%d)", ret);
		return -EIO;
	}

	len = ret/2;

	for (i = 0; i < len; i++)
		buf[i] = cpu_to_le16(buf[i]);

	if (debugmask_if(1)) {
		pr_info("I2C RX (%d):", len);
		written = 0;
		for (i = 0; i < len; i++) {
			written += snprintf(debug_string + written,
					8, "0x%04X,", buf[i]);
			if (written + 8 >= DEBUG_STRING_LEN_MAX) {
				pr_info("%s", debug_string);
				written = 0;
			}
		}
		if (written > 0)
			pr_info("%s", debug_string);
	}

	return len;
}

/* debug_mask |= 0x2 for I2C TX communication */
static int i2c_tx_bytes(struct data *ts, u8 *buf, u16 len)
{
	int i, ret, written;
	char debug_string[DEBUG_STRING_LEN_MAX];

	do {
		ret = i2c_master_send(ts->client, (char *) buf, (int) len);
	} while (ret == -EAGAIN);
	if (ret < 0) {
		pr_err("I2C TX fail (%d)", ret);
		return ret;
	}

	len = ret;

	if (debugmask_if(2)) {
		pr_info("I2C TX (%d):", len);
		written = 0;
		for (i = 0; i < len; i++) {
			written += snprintf(debug_string + written, 6,
					"0x%02X,", buf[i]);
			if (written + 6 >= DEBUG_STRING_LEN_MAX) {
				pr_info("%s", debug_string);
				written = 0;
			}
		}
		if (written > 0)
			pr_info("%s", debug_string);
	}

	return len;
}

static int i2c_tx_words(struct data *ts, u16 *buf, u16 len)
{
	int i, ret, written;
	char debug_string[DEBUG_STRING_LEN_MAX];

	for (i = 0; i < len; i++)
		buf[i] = cpu_to_le16(buf[i]);

	do {
		ret = i2c_master_send(ts->client,
			(char *) buf, (int) (len * 2));
	} while (ret == -EAGAIN);
	if (ret < 0) {
		pr_err("I2C TX fail (%d)", ret);
		return ret;
	}
	if ((ret % 2) != 0) {
		pr_err("I2C words TX fail: odd number of bytes (%d)", ret);
		return -EIO;
	}

	len = ret/2;

	if (debugmask_if(2)) {
		pr_info("I2C TX (%d):", len);
		written = 0;
		for (i = 0; i < len; i++) {
			written += snprintf(debug_string + written, 8,
					"0x%04X,", buf[i]);
			if (written + 8 >= DEBUG_STRING_LEN_MAX) {
				pr_info("%s", debug_string);
				written = 0;
			}
		}
		if (written > 0)
			pr_info("%s", debug_string);
	}

	return len;
}

/* Read report */
static int read_mtp_report(struct data *ts, u16 *buf)
{
	int words = 1, words_tx, words_rx;
	int ret = 0, remainder = 0, offset = 0;
	u16 address = 0x000A;

	mutex_lock(&ts->i2c_mutex);
	/* read header, get size, read entire report */
	{
		words_tx = i2c_tx_words(ts, &address, 1);
		if (words_tx != 1) {
			mutex_unlock(&ts->i2c_mutex);
			pr_err("Report RX fail: failed to set address");
			return -EIO;
		}

		if (ts->is_raw_mode == 0) {
			words_rx = i2c_rx_words(ts, buf, 2);
			if (words_rx != 2 ||
					BYTEL(buf[0]) > RPT_LEN_PACKET_MAX) {
				ret = -EIO;
				pr_err("Report RX fail: received (%d) " \
						"expected (%d) words, " \
						"header (%04X)",
						words_rx, words, buf[0]);
				mutex_unlock(&ts->i2c_mutex);
				return ret;
			}

			if ((((BYTEH(buf[0])) & 0xF) == 0x1)
				&& buf[1] == 0x0800)
				ts->is_raw_mode = 1;

			words = BYTEL(buf[0]) + 1;

			words_tx = i2c_tx_words(ts, &address, 1);
			if (words_tx != 1) {
				mutex_unlock(&ts->i2c_mutex);
				pr_err("Report RX fail:" \
					"failed to set address");
				return -EIO;
			}

			words_rx = i2c_rx_words(ts, &buf[offset], words);
			if (words_rx != words) {
				mutex_unlock(&ts->i2c_mutex);
				pr_err("Report RX fail 0x%X: received (%d) " \
					"expected (%d) words",
					address, words_rx, remainder);
				return -EIO;

			}

		} else {

			words_rx = i2c_rx_words(ts, buf,
					(u16) PDATA(i2c_words));
			if (words_rx != (u16) PDATA(i2c_words) || BYTEL(buf[0])
					> RPT_LEN_PACKET_MAX) {
				ret = -EIO;
				pr_err("Report RX fail: received (%d) " \
					"expected (%d) words, header (%04X)",
					words_rx, words, buf[0]);
				mutex_unlock(&ts->i2c_mutex);
				return ret;
			}

			if ((((BYTEH(buf[0])) & 0xF) == 0x1)
				&& buf[1] != 0x0800)
				ts->is_raw_mode = 0;

			words = BYTEL(buf[0]) + 1;
			remainder = words;

			if (remainder - (u16) PDATA(i2c_words) > 0) {
				remainder -= (u16) PDATA(i2c_words);
				offset += (u16) PDATA(i2c_words);
				address += (u16) PDATA(i2c_words);
			}

			words_tx = i2c_tx_words(ts, &address, 1);
			if (words_tx != 1) {
				mutex_unlock(&ts->i2c_mutex);
				pr_err("Report RX fail: failed to set " \
					"address 0x%X", address);
				return -EIO;
			}

			words_rx = i2c_rx_words(ts, &buf[offset], remainder);
			if (words_rx != remainder) {
				mutex_unlock(&ts->i2c_mutex);
				pr_err("Report RX fail 0x%X: received (%d) " \
						"expected (%d) words",
						address, words_rx, remainder);
				return -EIO;
			}
		}
	}
	mutex_unlock(&ts->i2c_mutex);
	return ret;
}

/* Send command */
static int send_mtp_command(struct data *ts, u16 *buf, u16 len)
{
	u16 tx_buf[CMD_LEN_PACKET_MAX + 2]; /* with address and header */
	u16 packets, words, words_tx;
	int i, ret = 0;

	/* check basics */
	if (len < 2 || len > CMD_LEN_MAX || (buf[1] + 2) != len) {
		pr_err("Command length is not valid");
		ret = -EINVAL;
		goto err_send_mtp_command;
	}

	/* packetize and send */
	packets = len / CMD_LEN_PACKET_MAX;
	if (len % CMD_LEN_PACKET_MAX)
		packets++;
	tx_buf[0] = 0x0000;

	mutex_lock(&ts->i2c_mutex);
	for (i = 0; i < packets; i++) {
		words = (i == (packets - 1)) ? len : CMD_LEN_PACKET_MAX;
		tx_buf[1] = (packets << 12) | ((i + 1) << 8) | words;
		memcpy(&tx_buf[2], &buf[i * CMD_LEN_PACKET_MAX],
			BYTE_SIZE(words));
		words_tx = i2c_tx_words(ts, tx_buf, words + 2);
		if (words_tx != (words + 2)) {
			pr_err("Command TX fail: transmitted (%d) " \
				"expected (%d) words, packet (%d)",
				words_tx, words + 2, i);
			ret = -EIO;
			mutex_unlock(&ts->i2c_mutex);
			goto err_send_mtp_command;
		}
		len -= CMD_LEN_PACKET_MAX;
	}
	mutex_unlock(&ts->i2c_mutex);

	return ret;

err_send_mtp_command:
	return ret;
}

static void max1187x_wfxn_cmd(struct work_struct *work)
{
	struct data *ts = container_of(work, struct data, work_cmd);

	send_mtp_command(ts, ts->cmd_buf, ts->cmd_len);

	up(&ts->sema_cmd);

}

/* Integer math operations */
#if defined(MAX1187X_PRESSURE_SHAPING) || \
	(MAX1187X_TOUCH_REPORT_MODE == 2 && \
		!defined(MAX1187X_REPORT_FAST_CALCULATION))
u16 max1187x_sqrt(u32 num)
{
	u16 mask = 0x8000;
	u16 guess = 0;
	u32 prod = 0;

	if (num < 2)
		return num;

	while (mask) {
		guess = guess ^ mask;
		prod = guess*guess;
		if (num < prod)
			guess = guess ^ mask;
		mask = mask>>1;
	}
	if (guess != 0xFFFF) {
		prod = guess*guess;
		if ((num - prod) > (prod + 2*guess + 1 - num))
			guess++;
	}

	return guess;
}
#endif

#if MAX1187X_TOUCH_REPORT_MODE == 2
#ifndef MAX1187X_REPORT_FAST_CALCULATION
/* Returns index of element in array closest to val */
static u16 binary_search(const u16 *array, u16 len, u16 val)
{
	s16 lt, rt, mid;
	if (len < 2)
		return 0;

	lt = 0;
	rt = len - 1;

	while (lt <= rt) {
		mid = (lt + rt)/2;
		if (val == array[mid])
			return mid;
		if (val < array[mid])
			rt = mid - 1;
		else
			lt = mid + 1;
	}

	if (lt >= len)
		return len - 1;
	if (rt < 0)
		return 0;
	if (array[lt] - val > val - array[lt-1])
		return lt-1;
	else
		return lt;
}
/* Given values of x and y, it calculates the orientation
 * with respect to y axis by calculating atan(x/y)
 */
static s16 max1187x_orientation(s16 x, s16 y)
{
	u16 sign = 0;
	s16 angle;
	u16 len = sizeof(tanlist)/sizeof(tanlist[0]);
	u32 quotient;

	if (x == y) {
		angle = 45;
		return angle;
	}
	if (x == 0) {
		angle = 0;
		return angle;
	}
	if (y == 0) {
		if (x > 0)
			angle = 90;
		else
			angle = -90;
		return angle;
	}

	if (x < 0) {
		sign = ~sign;
		x = -x;
	}
	if (y < 0) {
		sign = ~sign;
		y = -y;
	}

	if (x == y)
		angle = 45;
	else if (x < y) {
		quotient = ((u32)x << 16) - (u32)x;
		quotient = quotient / y;
		angle = binary_search(tanlist, len, quotient);
	} else {
		quotient = ((u32)y << 16) - (u32)y;
		quotient = quotient / x;
		angle = binary_search(tanlist, len, quotient);
		angle = 90 - angle;
	}
	if (sign == 0)
		return angle;
	else
		return -angle;
}
#endif
#endif

/* Returns time difference between time_later and time_earlier.
 * time is measures in units of jiffies32 */
u32 time_difference(u32 time_later, u32 time_earlier)
{
	u64	time_elapsed;
	if (time_later >= time_earlier)
		time_elapsed = time_later - time_earlier;
	else
		time_elapsed = time_later +
					0x100000000 - time_earlier;
	return (u32)time_elapsed;
}

/* debug_mask |= 0x4 for touch reports */
static void invalidate_all_fingers(struct data *ts)
{
#ifndef MAX1187X_PROTOCOL_A
	u32 i;
#endif
	pr_info_if(4, "(TOUCH): Finger up (all)\n");
#ifdef MAX1187X_PROTOCOL_A
	input_mt_sync(ts->input_dev);
#else
	for (i = 0; i < MAX1187X_TOUCH_COUNT_MAX; i++) {
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev,
					MT_TOOL_FINGER, 0);
		input_mt_slot(ts->input_pen, i);
		input_mt_report_slot_state(ts->input_pen,
					MT_TOOL_PEN, 0);
	}
#endif
	input_sync(ts->input_dev);
	input_sync(ts->input_pen);
	ts->list_finger_ids = 0;
}

static void process_report(struct data *ts, u16 *buf)
{
	u32 i, j;
	u16 x, y, z, swap_u16, curr_finger_ids, tool_type;
	struct input_dev *idev = ts->input_dev;
#if MAX1187X_TOUCH_REPORT_MODE == 2
	u32 area;
	s16 swap_s16;
	u32 major_axis, minor_axis;
	s16 xsize, ysize, orientation;
#endif

	struct max1187x_touch_report_header *header;
	struct max1187x_touch_report_basic *reportb;
	struct max1187x_touch_report_extended *reporte;

	header = (struct max1187x_touch_report_header *) buf;

	if (BYTEH(header->header) != 0x11)
		goto err_process_report_header;

#ifdef TOUCH_WAKEUP_FEATURE
	if (device_may_wakeup(&ts->client->dev) && ts->is_suspended == 1) {
		pr_info_if(4, "Received gesture: (0x%04X)\n", buf[3]);
		if (header->report_id == MAX1187X_REPORT_POWER_MODE
				&& buf[3] == 0x0102) {
			pr_info_if(4, "Received touch wakeup report\n");
			input_report_key(ts->input_dev_key,	KEY_POWER, 1);
			input_sync(ts->input_dev_key);
			input_report_key(ts->input_dev_key,	KEY_POWER, 0);
			input_sync(ts->input_dev_key);
		}
		goto process_report_complete;
	}
#endif

	if (header->report_id != MAX1187X_REPORT_TOUCH_BASIC &&
			header->report_id != MAX1187X_REPORT_TOUCH_EXTENDED)
		goto err_process_report_reportid;

	if (ts->framecounter == header->framecounter) {
		pr_err("Same framecounter (%u) encountered at irq (%u)!\n",
				ts->framecounter, ts->irq_count);
		goto err_process_report_framecounter;
	}
	ts->framecounter = header->framecounter;

	if (header->button0 != ts->button0) {
		input_report_key(ts->input_dev, PDATA(button_code0),
				header->button0);
		input_sync(ts->input_dev);
		ts->button0 = header->button0;
	}
	if (header->button1 != ts->button1) {
		input_report_key(ts->input_dev, PDATA(button_code1),
				header->button1);
		input_sync(ts->input_dev);
		ts->button1 = header->button1;
	}
	if (header->button2 != ts->button2) {
		input_report_key(ts->input_dev, PDATA(button_code2),
				header->button2);
		input_sync(ts->input_dev);
		ts->button2 = header->button2;
	}
	if (header->button3 != ts->button3) {
		input_report_key(ts->input_dev, PDATA(button_code3),
				header->button3);
		input_sync(ts->input_dev);
		ts->button3 = header->button3;
	}

	if (header->touch_count > 10) {
		pr_err("Touch count (%u) out of bounds [0,10]!",
				header->touch_count);
		goto err_process_report_touchcount;
	}

	if (header->touch_count == 0) {
		invalidate_all_fingers(ts);
	} else {
		curr_finger_ids = 0;
		reportb = (struct max1187x_touch_report_basic *)
				((u8 *)buf + sizeof(*header));
		reporte = (struct max1187x_touch_report_extended *)
				((u8 *)buf + sizeof(*header));
		for (i = 0; i < header->touch_count; i++) {
			x = reportb->x;
			y = reportb->y;
			z = reportb->z;
			if (PDATA(coordinate_settings) & MAX1187X_SWAP_XY) {
				swap_u16 = x;
				x = y;
				y = swap_u16;
			}
			if (PDATA(coordinate_settings) & MAX1187X_REVERSE_X) {
				x = PDATA(panel_margin_xl) + PDATA(lcd_x)
					+ PDATA(panel_margin_xh) - 1 - x;
			}
			if (PDATA(coordinate_settings) & MAX1187X_REVERSE_Y) {
				y = PDATA(panel_margin_yl) + PDATA(lcd_y)
					+ PDATA(panel_margin_yh) - 1 - y;
			}

			tool_type = reportb->tool_type;
			if (tool_type == 1) {
				idev = ts->input_pen;
				tool_type = MT_TOOL_PEN;
			} else {
				idev = ts->input_dev;
				tool_type = MT_TOOL_FINGER;
			}

			curr_finger_ids |= (1<<reportb->finger_id);
#ifdef MAX1187X_PROTOCOL_A
			input_report_abs(idev,
				ABS_MT_TRACKING_ID, reportb->finger_id);
			input_report_abs(idev,
				ABS_MT_TOOL_TYPE, tool_type);
#else
			input_mt_slot(idev, reportb->finger_id);
			input_mt_report_slot_state(idev,
					tool_type, 1);
#endif
			input_report_abs(idev, ABS_MT_POSITION_X, x);
			input_report_abs(idev, ABS_MT_POSITION_Y, y);
#ifdef MAX1187X_PRESSURE_SHAPING
			z = (PRESSURE_MAX_SQRT >> 2) + max1187x_sqrt(z);
			if (z > PRESSURE_MAX_SQRT)
				z = PRESSURE_MAX_SQRT;
#endif
			input_report_abs(idev, ABS_MT_PRESSURE, z);

			pr_info_if(4, "(TOUCH): (%u) Finger %u: "\
				"X(%d) Y(%d) Z(%d) TT(%d)",
				header->framecounter, reportb->finger_id,
				x, y, z, tool_type);

			if (header->report_id
				== MAX1187X_REPORT_TOUCH_EXTENDED) {
#if MAX1187X_TOUCH_REPORT_MODE == 2
				if (PDATA(coordinate_settings)
					& MAX1187X_SWAP_XY) {
					swap_s16 = reporte->xpixel;
					reporte->xpixel = reporte->ypixel;
					reporte->ypixel = swap_s16;
				}
				if (PDATA(coordinate_settings)
						& MAX1187X_REVERSE_X)
					reporte->xpixel = -reporte->xpixel;
				if (PDATA(coordinate_settings)
						& MAX1187X_REVERSE_Y)
					reporte->ypixel = -reporte->ypixel;
				area = reporte->area
					* (PDATA(lcd_x)/PDATA(num_sensor_x))
					* (PDATA(lcd_y)/PDATA(num_sensor_y));
				xsize = reporte->xpixel
				* (s16)(PDATA(lcd_x)/PDATA(num_sensor_x));
				ysize = reporte->ypixel
				* (s16)(PDATA(lcd_y)/PDATA(num_sensor_y));
				pr_info_if(4, "(TOUCH): pixelarea (%u) " \
					"xpixel (%d) ypixel (%d) " \
					"xsize (%d) ysize (%d)\n",
					reporte->area,
					reporte->xpixel, reporte->ypixel,
					xsize, ysize);

#ifndef MAX1187X_REPORT_FAST_CALCULATION
				/* Calculate orientation as
				 * arctan of xsize/ysize) */
				orientation =
					max1187x_orientation(xsize, ysize);
				/* Major axis of ellipse if hypotenuse
				 * formed by xsize and ysize */
				major_axis = xsize*xsize + ysize*ysize;
				major_axis = max1187x_sqrt(major_axis);
				/* Minor axis can be reverse calculated
				 * using the area of ellipse:
				 * Area of ellipse =
				 *		pi / 4 * Major axis * Minor axis
				 * Minor axis =
				 *		4 * Area / (pi * Major axis)
				 */
				minor_axis = (2 * area) / major_axis;
				minor_axis = (minor_axis<<17) / MAX1187X_PI;
#else
				if (xsize < 0)
					xsize = -xsize;
				if (ysize < 0)
					ysize = -ysize;
				orientation = (xsize > ysize) ? 0 : 90;
				major_axis = (xsize > ysize) ? xsize : ysize;
				minor_axis = (xsize > ysize) ? ysize : xsize;
#endif
				pr_info_if(4, "(TOUCH): Finger %u: " \
					"Orientation(%d) Area(%u) Major_axis(%u) Minor_axis(%u)",
					reportb->finger_id,	orientation,
					area, major_axis, minor_axis);
				input_report_abs(idev,
					ABS_MT_ORIENTATION, orientation);
				input_report_abs(idev,
						ABS_MT_TOUCH_MAJOR, major_axis);
				input_report_abs(idev,
						ABS_MT_TOUCH_MINOR, minor_axis);
#endif
				reporte++;
				reportb = (struct max1187x_touch_report_basic *)
						((u8 *) reporte);
			} else {
				reportb++;
			}
#ifdef MAX1187X_PROTOCOL_A
			input_mt_sync(idev);
#endif
		}

		i = 0;
		j = 1;
		while (i < 10) {
			if ((ts->list_finger_ids & j) != 0 &&
					(curr_finger_ids & j) == 0) {
				pr_info_if(4, "(TOUCH): Finger up (%d)\n", i);
#ifndef MAX1187X_PROTOCOL_A
				input_mt_slot(idev, i);
				input_mt_report_slot_state(idev,
						MT_TOOL_FINGER, 0);
#endif
			}
			i++;
			j <<= 1;
		}

		input_sync(idev);
		ts->list_finger_ids = curr_finger_ids;
	}

#ifdef TOUCH_WAKEUP_FEATURE
process_report_complete:
#endif
	return;

err_process_report_touchcount:
err_process_report_header:
err_process_report_reportid:
err_process_report_framecounter:
	return;
}

static void max1187x_wfxn_irq(struct work_struct *work)
{
	struct data *ts = container_of(work, struct data, work_irq);
	int ret;
	u32	time_elapsed;

	if (gpio_get_value(ts->pdata->gpio_tirq) != 0)
		return;

	ret = read_mtp_report(ts, ts->rx_packet);

	time_elapsed = time_difference(jiffies, ts->irq_receive_time);

	/* Verify time_elapsed < 1s */
	if (ret == 0 || time_elapsed > HZ) {
		process_report(ts, ts->rx_packet);
		propagate_report(ts, 0, ts->rx_packet);
	}
	enable_irq(ts->client->irq);
}

/* debug_mask |= 0x20 for irq_handler */
static irqreturn_t irq_handler(int irq, void *context)
{
	struct data *ts = (struct data *) context;

	pr_info_if(0x20, "Enter\n");

	if (gpio_get_value(ts->pdata->gpio_tirq) != 0)
		goto irq_handler_complete;

	disable_irq_nosync(ts->client->irq);
	ts->irq_receive_time = jiffies;
	ts->irq_count++;

	queue_work(ts->wq, &ts->work_irq);

irq_handler_complete:
	pr_info_if(0x20, "Exit\n");
	return IRQ_HANDLED;
}

static ssize_t init_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", init_state);
}

static ssize_t init_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int value, ret;

	if (sscanf(buf, "%d", &value) != 1) {
		pr_err("bad parameter");
		return -EINVAL;
	}
	switch (value) {
	case 0:
		if (init_state == 0)
			break;
		ret = device_deinit(to_i2c_client(dev));
		if (ret != 0) {
			pr_err("deinit error (%d)", ret);
			return ret;
		}
		break;
	case 1:
		if (init_state == 1)
			break;
		ret = device_init(to_i2c_client(dev));
		if (ret != 0) {
			pr_err("init error (%d)", ret);
			return ret;
		}
		break;
	case 2:
		if (init_state == 1) {
			ret = device_deinit(to_i2c_client(dev));
			if (ret != 0) {
				pr_err("deinit error (%d)", ret);
				return ret;
			}
		}
		ret = device_init(to_i2c_client(dev));
		if (ret != 0) {
			pr_err("init error (%d)", ret);
			return ret;
		}
		break;
	default:
		pr_err("bad value");
		return -EINVAL;
	}

	return count;
}

static ssize_t i2c_reset_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);
	int ret;

	ret = bootloader_exit(ts);
	if (ret) {
		pr_err("Failed to do i2c reset.");
		goto exit;
	}
	pr_info("i2c reset occured\n");
exit:
	return count;
}

static int reset_power(struct data *ts)
{
	int ret;

	invalidate_all_fingers(ts);

	ret = vreg_configure(ts, 0);
	if (ret)
		goto exit;
	usleep_range(PWR_OFF_WAIT_MIN_US, PWR_OFF_WAIT_MAX_US);
	ret = gpio_direction_output(PDATA(gpio_pwr_en), 1);
	if (ret)
		goto exit;
	msleep(PWR_RESET_WAIT_MS);
	ret = gpio_direction_output(PDATA(gpio_pwr_en), 0);
	if (ret)
		goto exit;
	usleep_range(PWR_ON_WAIT_MIN_US, PWR_ON_WAIT_MAX_US);
	ret = vreg_configure(ts, 1);
	if (ret)
		goto exit;

	msleep(30);

	pr_info("Power on Reset\n");
exit:
	return ret;
}

static ssize_t power_on_reset_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);
	int ret;

	ret = reset_power(ts);
	if (ret) {
		pr_err("Failed to do power on reset.");
		goto exit;
	}
	pr_info("hw reset occured\n");
exit:
	return count;
}

static ssize_t sreset_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);
	u16 cmd_buf[] = {0x00E9, 0x0000};
	int ret;

	ret = rbcmd_send_receive(ts, cmd_buf, 2, 0x01A0, 3 * HZ);
	if (ret)
		pr_err("Failed to do soft reset.");
	return count;
}

static ssize_t fw_update_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);
	int ret;

	if (init_state != 1) {
		pr_err("Device is not ready\n");
		return -ENODEV;
	}

	mutex_lock(&ts->fw_mutex);
	if (sysfs_streq(buf, fw_update_mode[MAX_FW_UPDATE_DEFAULT])) {
		ts->fw_update_mode = MAX_FW_UPDATE_DEFAULT;
	} else if (sysfs_streq(buf, fw_update_mode[MAX_FW_UPDATE_FORCE])) {
		ts->fw_update_mode = MAX_FW_UPDATE_FORCE;
	} else {
		pr_err("Invalid argument: %s\n", buf);
		ret = -EINVAL;
		goto end;
	}
	pr_info("firmware update (%s)\n", fw_update_mode[ts->fw_update_mode]);
	validate_fw(ts);
	ret = count;

end:
	ts->fw_update_mode = MAX_FW_UPDATE_DEFAULT;
	mutex_unlock(&ts->fw_mutex);
	return ret;
}

static ssize_t irq_count_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);

	return snprintf(buf, PAGE_SIZE, "%u\n", ts->irq_count);
}

static ssize_t irq_count_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);

	ts->irq_count = 0;
	return count;
}

static ssize_t dflt_cfg_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);

	return snprintf(buf, PAGE_SIZE, "%u 0x%x 0x%x\n", PDATA(defaults_allow),
			PDATA(default_config_id), PDATA(default_chip_id));
}

static ssize_t dflt_cfg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);

	(void) sscanf(buf, "%u 0x%x 0x%x", &PDATA(defaults_allow),
			&PDATA(default_config_id), &PDATA(default_chip_id));
	return count;
}

static ssize_t panel_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);

	return snprintf(buf, PAGE_SIZE, "%u %u %u %u %u %u\n",
			PDATA(panel_margin_xl), PDATA(panel_margin_xh),
			PDATA(panel_margin_yl), PDATA(panel_margin_yh),
			PDATA(lcd_x), PDATA(lcd_y));
}

static ssize_t panel_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);

	(void) sscanf(buf, "%u %u %u %u %u %u", &PDATA(panel_margin_xl),
			&PDATA(panel_margin_xh), &PDATA(panel_margin_yl),
			&PDATA(panel_margin_yh), &PDATA(lcd_x),
			&PDATA(lcd_y));
	return count;
}

static ssize_t fw_ver_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);

	int ret, count = 0;
	u16 cmd_buf[2];

	/* Read firmware version */
	cmd_buf[0] = 0x0040;
	cmd_buf[1] = 0x0000;

	ret = rbcmd_send_receive(ts, cmd_buf, 2, 0x0140, HZ/4);

	if (ret)
		goto err_fw_ver_show;

	ts->chip_id = BYTEH(ts->rbcmd_rx_report[4]);
	count += snprintf(buf, PAGE_SIZE, "fw_ver (%u.%u.%u) " \
					"chip_id (0x%02X)\n",
					BYTEH(ts->rbcmd_rx_report[3]),
					BYTEL(ts->rbcmd_rx_report[3]),
					ts->rbcmd_rx_report[5],
					ts->chip_id);

	/* Read touch configuration */
	cmd_buf[0] = 0x0002;
	cmd_buf[1] = 0x0000;

	ret = rbcmd_send_receive(ts, cmd_buf, 2, 0x0102, HZ/4);

	if (ret) {
		pr_err("Failed to receive chip config\n");
		goto err_fw_ver_show;
	}

	ts->config_id = ts->rbcmd_rx_report[3];

	count += snprintf(buf + count, PAGE_SIZE, "config_id (0x%04X) ",
					ts->config_id);
	count += snprintf(buf + count, PAGE_SIZE,
			"customer_info[1:0] (0x%04X, 0x%04X)\n",
					ts->rbcmd_rx_report[43],
					ts->rbcmd_rx_report[42]);
	return count;

err_fw_ver_show:
	return count;
}

static ssize_t chip_id_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);

	return snprintf(buf, PAGE_SIZE, "0x%02X\n", ts->chip_id);
}

static ssize_t config_id_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);

	return snprintf(buf, PAGE_SIZE, "0x%04X\n", ts->config_id);
}

static ssize_t driver_ver_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "3.1.8: May 10, 2013\n");
}

static ssize_t debug_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%04X\n", debug_mask);
}

static ssize_t debug_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	if (sscanf(buf, "%hx", &debug_mask) != 1) {
		pr_err("bad parameter");
		return -EINVAL;
	}

	return count;
}

static ssize_t command_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);
	u16 buffer[CMD_LEN_MAX];
	char scan_buf[5];
	int i, ret;

	count--; /* ignore carriage return */
	if ((count % 4) != 0) {
		pr_err("words not properly defined");
		return -EINVAL;
	}
	scan_buf[4] = '\0';
	for (i = 0; i < count; i += 4) {
		memcpy(scan_buf, &buf[i], 4);
		if (sscanf(scan_buf, "%hx", &buffer[i / 4]) != 1) {
			pr_err("bad word (%s)", scan_buf);
			return -EINVAL;
		}
	}
	ret = cmd_send(ts, buffer, count / 4);
	if (ret)
		pr_err("MTP command failed");
	return ++count;
}

static ssize_t report_read(struct file *file, struct kobject *kobj,
	struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	struct i2c_client *client = kobj_to_i2c_client(kobj);
	struct data *ts = i2c_get_clientdata(client);
	int printed, i, offset = 0, payload;
	int full_packet;
	int num_term_char;

	if (get_report(ts, 0xFFFF, 0xFFFFFFFF))
		return 0;

	payload = ts->rx_report_len;
	full_packet = payload;
	num_term_char = 2; /* number of term char */
	if (count < (4 * full_packet + num_term_char))
		return -EIO;
	if (count > (4 * full_packet + num_term_char))
		count = 4 * full_packet + num_term_char;

	for (i = 1; i <= payload; i++) {
		printed = snprintf(&buf[offset], PAGE_SIZE, "%04X\n",
			ts->rx_report[i]);
		if (printed <= 0)
			return -EIO;
		offset += printed - 1;
	}
	snprintf(&buf[offset], PAGE_SIZE, ",\n");
	release_report(ts);

	return count;
}

static DEVICE_ATTR(init, S_IRUGO | S_IWUSR, init_show, init_store);
static DEVICE_ATTR(i2c_reset, S_IWUSR, NULL, i2c_reset_store);
static DEVICE_ATTR(por, S_IWUSR, NULL, power_on_reset_store);
static DEVICE_ATTR(sreset, S_IWUSR, NULL, sreset_store);
static DEVICE_ATTR(fw_update, S_IWUSR, NULL, fw_update_store);
static DEVICE_ATTR(irq_count, S_IRUGO | S_IWUSR, irq_count_show,
		irq_count_store);
static DEVICE_ATTR(dflt_cfg, S_IRUGO | S_IWUSR, dflt_cfg_show, dflt_cfg_store);
static DEVICE_ATTR(panel, S_IRUGO | S_IWUSR, panel_show, panel_store);
static DEVICE_ATTR(fw_ver, S_IRUGO, fw_ver_show, NULL);
static DEVICE_ATTR(chip_id, S_IRUGO, chip_id_show, NULL);
static DEVICE_ATTR(config_id, S_IRUGO, config_id_show, NULL);
static DEVICE_ATTR(driver_ver, S_IRUGO, driver_ver_show, NULL);
static DEVICE_ATTR(debug, S_IRUGO | S_IWUSR, debug_show, debug_store);
static DEVICE_ATTR(command, S_IWUSR, NULL, command_store);
static struct bin_attribute dev_attr_report = {
		.attr = {.name = "report", .mode = S_IRUGO},
		.read = report_read };

static struct device_attribute *dev_attrs[] = {
		&dev_attr_i2c_reset,
		&dev_attr_por,
		&dev_attr_sreset,
		&dev_attr_fw_update,
		&dev_attr_irq_count,
		&dev_attr_dflt_cfg,
		&dev_attr_panel,
		&dev_attr_fw_ver,
		&dev_attr_chip_id,
		&dev_attr_config_id,
		&dev_attr_driver_ver,
		&dev_attr_debug,
		&dev_attr_command,
		NULL };

/* Send command to chip.
 */
static int cmd_send(struct data *ts, u16 *buf, u16 len)
{
	int ret;

	ret = down_interruptible(&ts->sema_cmd);
	if (ret != 0)
		goto err_cmd_send_sema_cmd;

	memcpy(ts->cmd_buf, buf, len * sizeof(buf[0]));
	ts->cmd_len = len;
	queue_work(ts->wq, &ts->work_cmd);

	return 0;

err_cmd_send_sema_cmd:
	return -ERESTARTSYS;
}

/* Send command to chip and expect a report with
 * id == report_id within timeout time.
 * timeout is measured in jiffies. 1s = HZ jiffies
 */
static int rbcmd_send_receive(struct data *ts, u16 *buf,
		u16 len, u16 report_id, u16 timeout)
{
	int ret;

	ret = down_interruptible(&ts->sema_rbcmd);
	if (ret != 0)
		goto err_rbcmd_send_receive_sema_rbcmd;

	ts->rbcmd_report_id = report_id;
	ts->rbcmd_received = 0;
	ts->rbcmd_waiting = 1;

	ret = cmd_send(ts, buf, len);
	if (ret)
		goto err_rbcmd_send_receive_cmd_send;

	ret = wait_event_interruptible_timeout(ts->waitqueue_rbcmd,
			ts->rbcmd_received != 0, timeout);
	if (ret < 0 || ts->rbcmd_received == 0)
		goto err_rbcmd_send_receive_timeout;

	ts->rbcmd_waiting = 0;
	up(&ts->sema_rbcmd);

	return 0;

err_rbcmd_send_receive_timeout:
err_rbcmd_send_receive_cmd_send:
	ts->rbcmd_waiting = 0;
	up(&ts->sema_rbcmd);
err_rbcmd_send_receive_sema_rbcmd:
	return -ERESTARTSYS;
}

/* debug_mask |= 0x8 for all driver INIT */
static int read_chip_data(struct data *ts)
{
	int ret;
	u16 loopcounter;
	u16 cmd_buf[2];

	/* Read firmware version */
	cmd_buf[0] = 0x0040;
	cmd_buf[1] = 0x0000;

	loopcounter = 0;
	ret = -1;
	while (loopcounter < MAX_FW_RETRIES && ret != 0) {
		ret = rbcmd_send_receive(ts, cmd_buf, 2, 0x0140, HZ/4);
		loopcounter++;
	}

	if (ret) {
		pr_err("Failed to receive fw version\n");
		goto err_read_chip_data;
	}

	ts->chip_id = BYTEH(ts->rbcmd_rx_report[4]);
	pr_info_if(8, "(INIT): fw_ver (%u.%u) " \
					"chip_id (0x%02X)\n",
					BYTEH(ts->rbcmd_rx_report[3]),
					BYTEL(ts->rbcmd_rx_report[3]),
					ts->chip_id);

	/* Read touch configuration */
	cmd_buf[0] = 0x0002;
	cmd_buf[1] = 0x0000;

	loopcounter = 0;
	ret = -1;
	while (loopcounter < MAX_FW_RETRIES && ret != 0) {
		ret = rbcmd_send_receive(ts, cmd_buf, 2, 0x0102, HZ/4);
		loopcounter++;
	}

	if (ret) {
		pr_err("Failed to receive chip config\n");
		goto err_read_chip_data;
	}

	ts->config_id = ts->rbcmd_rx_report[3];

	pr_info_if(8, "(INIT): config_id (0x%04X)\n",
					ts->config_id);
	return 0;

err_read_chip_data:
	return ret;
}

static int device_fw_load(struct data *ts, const struct firmware *fw,
	u16 fw_index)
{
	u16 filesize, file_codesize, loopcounter;
	u16 file_crc16_1, file_crc16_2, local_crc16;
	int chip_crc16_1 = -1, chip_crc16_2 = -1, ret;

	filesize = PDATA(fw_mapping[fw_index]).filesize;
	file_codesize = PDATA(fw_mapping[fw_index]).file_codesize;

	if (fw->size != filesize) {
		pr_err("filesize (%d) is not equal to expected size (%d)",
				fw->size, filesize);
		return -EIO;
	}

	file_crc16_1 = crc16(0, fw->data, file_codesize);

	loopcounter = 0;
	do {
		ret = bootloader_enter(ts);
		if (ret == 0)
			ret = bootloader_get_crc(ts, &local_crc16,
				0, file_codesize, 200);
		if (ret == 0)
			chip_crc16_1 = local_crc16;
		ret = bootloader_exit(ts);
		loopcounter++;
	} while (loopcounter < MAX_FW_RETRIES && chip_crc16_1 == -1);

	pr_info_if(8, "(INIT): file_crc16_1 = 0x%04x, chip_crc16_1 = 0x%04x\n",
			file_crc16_1, chip_crc16_1);

	if (ts->fw_update_mode == MAX_FW_UPDATE_FORCE ||
	    file_crc16_1 != chip_crc16_1) {
		loopcounter = 0;
		file_crc16_2 = crc16(0, fw->data, filesize);

		while (loopcounter < MAX_FW_RETRIES && file_crc16_2
				!= chip_crc16_2) {
			pr_info_if(8, "(INIT): Reprogramming chip. Attempt %d",
					loopcounter+1);
			ret = bootloader_enter(ts);
			if (ret == 0)
				ret = bootloader_erase_flash(ts);
			if (ret == 0)
				ret = bootloader_set_byte_mode(ts);
			if (ret == 0)
				ret = bootloader_write_flash(ts, fw->data,
					filesize);
			if (ret == 0)
				ret = bootloader_get_crc(ts, &local_crc16,
					0, filesize, 200);
			if (ret == 0)
				chip_crc16_2 = local_crc16;
			pr_info_if(8, "(INIT): file_crc16_2 = 0x%04x, "\
					"chip_crc16_2 = 0x%04x\n",
					file_crc16_2, chip_crc16_2);
			ret = bootloader_exit(ts);
			loopcounter++;
		}

		if (file_crc16_2 != chip_crc16_2)
			return -EAGAIN;
	}

	loopcounter = 0;
	do {
		ret = bootloader_exit(ts);
		loopcounter++;
	} while (loopcounter < MAX_FW_RETRIES && ret != 0);

	if (ret != 0)
		return -EIO;

	return 0;
}

static int is_booting(void)
{
	unsigned long long t;
	unsigned long nanosec_rem;

	t = cpu_clock(smp_processor_id());
	nanosec_rem = do_div(t, 1000000000);
	return (t < 30) ? 1 : 0;
}

static void validate_fw(struct data *ts)
{
	const struct firmware *fw;
	u16 config_id, chip_id;
	int i, ret;
	u16 cmd_buf[3];

	if (ts->fw_update_mode == MAX_FW_UPDATE_FORCE) {
		ts->chip_id = 0;
		ts->config_id = 0;
		goto set_id;
	}

	ret = read_chip_data(ts);
	if (ret && PDATA(defaults_allow) == 0) {
		pr_err("Firmware is not responsive "\
				"and default update is disabled\n");
		return;
	}

set_id:
	if (ts->chip_id != 0)
		chip_id = ts->chip_id;
	else
		chip_id = PDATA(default_chip_id);

	if (ts->config_id != 0)
		config_id = ts->config_id;
	else
		config_id = PDATA(default_config_id);

#ifdef FW_DOWNLOAD_FEATURE

	for (i = 0; i < PDATA(num_fw_mappings); i++) {
		if (PDATA(fw_mapping[i]).config_id == config_id &&
			PDATA(fw_mapping[i]).chip_id == chip_id)
			break;
	}

	if (i == PDATA(num_fw_mappings)) {
		pr_err("FW not found for configID(0x%04X) and chipID(0x%04X)",
			config_id, chip_id);
		return;
	}

	pr_info_if(8, "(INIT): Firmware file (%s)",
		PDATA(fw_mapping[i]).filename);

	ret = request_firmware(&fw, PDATA(fw_mapping[i]).filename,
					&ts->client->dev);

	if (ret || fw == NULL) {
		pr_err("firmware request failed (ret = %d, fwptr = %p)",
			ret, fw);
		return;
	}

	ret = down_interruptible(&ts->sema_cmd);
	if (ret) {
		release_firmware(fw);
		pr_err("Could not get lock\n");
		return;
	}
	disable_irq(ts->client->irq);
	flush_workqueue(ts->wq);
	if (device_fw_load(ts, fw, i)) {
		release_firmware(fw);
		pr_err("firmware download failed");
		enable_irq(ts->client->irq);
		up(&ts->sema_cmd);
		return;
	}

	release_firmware(fw);
	pr_info_if(8, "(INIT): firmware okay\n");
	enable_irq(ts->client->irq);
	up(&ts->sema_cmd);
	ret = read_chip_data(ts);

#endif

	cmd_buf[0] = 0x0018;
	cmd_buf[1] = 0x0001;
	cmd_buf[2] = MAX1187X_TOUCH_REPORT_MODE;
	ret = cmd_send(ts, cmd_buf, 3);
	if (ret) {
		pr_err("Failed to set up touch report mode");
		return;
	}
}

static int regulator_handler(struct regulator *regulator,
				struct device *dev,
				const char *func_str,
				const char *reg_str,
				int sw)
{
	int rc, enabled;

	if (IS_ERR_OR_NULL(regulator)) {
		rc = regulator ? PTR_ERR(regulator) : -EINVAL;
		pr_err("%s: regulator '%s' invalid",
			func_str ? func_str : "?",
			reg_str ? reg_str : "?");
		goto exit;
	}

	if (sw)
		rc = regulator_enable(regulator);
	else
		rc = regulator_disable(regulator);
	if (rc) {
		enabled = regulator_is_enabled(regulator);
		dev_warn(dev,
			"%s: regulator '%s' status is %d",
			func_str ? func_str : "?",
			reg_str ? reg_str : "?",
			enabled);
		if ((!enabled && !sw) || (enabled > 0 && sw))
			rc = 0;
	}
exit:
	return rc;
}

static int vreg_suspend(struct data *ts, int enable)
{
	int rc = 0;

	if (IS_ERR(ts->vreg_touch_vdd)) {
		pr_err("%s: vreg_touch_vdd is not initialized\n", __func__);
		rc = -ENODEV;
		goto exit;
	}

	if (enable)
		rc = regulator_set_optimum_mode(ts->vreg_touch_vdd, 2000);
	else
		rc = regulator_set_optimum_mode(ts->vreg_touch_vdd, 15000);

	if (rc < 0) {
		pr_err("%s: vdd: set mode (%s) failed, rc=%d\n",
				__func__, (enable ? "LPM" : "HPM"), rc);
		goto exit;
	} else {
		pr_info_if(8, "%s: vdd: set mode (%s) ok, new mode=%d\n",
				__func__, (enable ? "LPM" : "HPM"), rc);
		rc = 0;
	}
exit:
	return rc;
}

static int vreg_configure(struct data *ts, int enable)
{
	int rc = 0;
	struct device *dev = &ts->client->dev;

	if (enable) {
		ts->vreg_touch_vdd = regulator_get(dev, MAXIM_VDD);
		if (IS_ERR(ts->vreg_touch_vdd)) {
			pr_err("%s: get vdd failed\n", __func__);
			rc = -ENODEV;
			goto err_ret;
		}
		rc = regulator_set_voltage(ts->vreg_touch_vdd,
						3000000, 3000000);
		if (rc) {
			pr_err("%s: set voltage vdd failed, rc=%d\n",
								__func__, rc);
			goto err_put_vdd;
		}
		rc = regulator_handler(ts->vreg_touch_vdd, dev,
						__func__, MAXIM_VDD, 1);
		if (rc)
			goto err_put_vdd;
		rc = vreg_suspend(ts, 0);
		if (rc) {
			pr_err("%s: set vdd mode failed, rc=%d\n",
							__func__, rc);
			goto err_disable_vdd;
		}
	} else {
		if (!IS_ERR(ts->vreg_touch_vdd)) {
			rc = regulator_set_voltage(ts->vreg_touch_vdd,
					0, 3000000);
			if (rc)
				pr_err("%s: set voltage vdd failed, rc=%d\n",
								__func__, rc);
			regulator_handler(ts->vreg_touch_vdd,
					dev, __func__, MAXIM_VDD, 0);
			regulator_put(ts->vreg_touch_vdd);
		}
	}
	return rc;
err_disable_vdd:
	regulator_handler(ts->vreg_touch_vdd, dev, __func__,
					MAXIM_VDD, 0);
err_put_vdd:
	regulator_put(ts->vreg_touch_vdd);
err_ret:
	return rc;
}

/* #ifdef CONFIG_OF */
static struct max1187x_pdata *max1187x_get_platdata_dt(struct device *dev)
{
	struct max1187x_pdata *pdata = NULL;
	struct device_node *devnode = dev->of_node;
	u32 i;
	u32 datalist[MAX1187X_NUM_FW_MAPPINGS_MAX];

	if (!devnode)
		return NULL;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err("Failed to allocate memory for pdata\n");
		return NULL;
	}

	/* Parse gpio_tirq */
	if (of_property_read_u32(devnode, "gpio_tirq", &pdata->gpio_tirq)) {
		pr_err("Failed to get property: gpio_tirq\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse gpio_pwr_en */
	if (of_property_read_u32(devnode, "gpio_pwr_en", &pdata->gpio_pwr_en))
		pr_err("Failed to get property: gpio_pwr_en\n");

	/* Parse num_fw_mappings */
	if (of_property_read_u32(devnode, "num_fw_mappings",
		&pdata->num_fw_mappings)) {
		pr_err("Failed to get property: num_fw_mappings\n");
		goto err_max1187x_get_platdata_dt;
	}

	if (pdata->num_fw_mappings > MAX1187X_NUM_FW_MAPPINGS_MAX)
		pdata->num_fw_mappings = MAX1187X_NUM_FW_MAPPINGS_MAX;

	/* Parse config_id */
	if (of_property_read_u32_array(devnode, "config_id", datalist,
			pdata->num_fw_mappings)) {
		pr_err("Failed to get property: config_id\n");
		goto err_max1187x_get_platdata_dt;
	}

	for (i = 0; i < pdata->num_fw_mappings; i++)
		pdata->fw_mapping[i].config_id = datalist[i];

	/* Parse chip_id */
	if (of_property_read_u32_array(devnode, "chip_id", datalist,
			pdata->num_fw_mappings)) {
		pr_err("Failed to get property: chip_id\n");
		goto err_max1187x_get_platdata_dt;
	}

	for (i = 0; i < pdata->num_fw_mappings; i++)
		pdata->fw_mapping[i].chip_id = datalist[i];

	/* Parse filename */
	for (i = 0; i < pdata->num_fw_mappings; i++) {
		if (of_property_read_string_index(devnode, "filename", i,
			(const char **) &pdata->fw_mapping[i].filename)) {
				pr_err("Failed to get property: "\
					"filename[%d]\n", i);
				goto err_max1187x_get_platdata_dt;
			}
	}

	/* Parse filesize */
	if (of_property_read_u32_array(devnode, "filesize", datalist,
		pdata->num_fw_mappings)) {
		pr_err("Failed to get property: filesize\n");
		goto err_max1187x_get_platdata_dt;
	}

	for (i = 0; i < pdata->num_fw_mappings; i++)
		pdata->fw_mapping[i].filesize = datalist[i];

	/* Parse file_codesize */
	if (of_property_read_u32_array(devnode, "file_codesize", datalist,
		pdata->num_fw_mappings)) {
		pr_err("Failed to get property: file_codesize\n");
		goto err_max1187x_get_platdata_dt;
	}

	for (i = 0; i < pdata->num_fw_mappings; i++)
		pdata->fw_mapping[i].file_codesize = datalist[i];

	/* Parse defaults_allow */
	if (of_property_read_u32(devnode, "defaults_allow",
		&pdata->defaults_allow)) {
		pr_err("Failed to get property: defaults_allow\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse default_config_id */
	if (of_property_read_u32(devnode, "default_config_id",
		&pdata->default_config_id)) {
		pr_err("Failed to get property: default_config_id\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse default_chip_id */
	if (of_property_read_u32(devnode, "default_chip_id",
		&pdata->default_chip_id)) {
		pr_err("Failed to get property: default_chip_id\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse i2c_words */
	if (of_property_read_u32(devnode, "i2c_words", &pdata->i2c_words)) {
		pr_err("Failed to get property: i2c_words\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse coordinate_settings */
	if (of_property_read_u32(devnode, "coordinate_settings",
		&pdata->coordinate_settings)) {
		pr_err("Failed to get property: coordinate_settings\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse panel_margin_xl */
	if (of_property_read_u32(devnode, "panel_margin_xl",
		&pdata->panel_margin_xl)) {
		pr_err("Failed to get property: panel_margin_xl\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse lcd_x */
	if (of_property_read_u32(devnode, "lcd_x", &pdata->lcd_x)) {
		pr_err("Failed to get property: lcd_x\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse panel_margin_xh */
	if (of_property_read_u32(devnode, "panel_margin_xh",
		&pdata->panel_margin_xh)) {
		pr_err("Failed to get property: panel_margin_xh\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse panel_margin_yl */
	if (of_property_read_u32(devnode, "panel_margin_yl",
		&pdata->panel_margin_yl)) {
		pr_err("Failed to get property: panel_margin_yl\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse lcd_y */
	if (of_property_read_u32(devnode, "lcd_y", &pdata->lcd_y)) {
		pr_err("Failed to get property: lcd_y\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse panel_margin_yh */
	if (of_property_read_u32(devnode, "panel_margin_yh",
		&pdata->panel_margin_yh)) {
		pr_err("Failed to get property: panel_margin_yh\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse row_count */
	if (of_property_read_u32(devnode, "num_sensor_x",
		&pdata->num_sensor_x)) {
		pr_err("Failed to get property: num_sensor_x\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse num_sensor_y */
	if (of_property_read_u32(devnode, "num_sensor_y",
		&pdata->num_sensor_y)) {
		pr_err("Failed to get property: num_sensor_y\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse button_code0 */
	if (of_property_read_u32(devnode, "button_code0",
		&pdata->button_code0)) {
		pr_err("Failed to get property: button_code0\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse button_code1 */
	if (of_property_read_u32(devnode, "button_code1",
		&pdata->button_code1)) {
		pr_err("Failed to get property: button_code1\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse button_code2 */
	if (of_property_read_u32(devnode, "button_code2",
		&pdata->button_code2)) {
		pr_err("Failed to get property: button_code2\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse button_code3 */
	if (of_property_read_u32(devnode, "button_code3",
		&pdata->button_code3)) {
		pr_err("Failed to get property: button_code3\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse enable_resume_por */
	if (of_property_read_u32(devnode, "enable_resume_por",
		&pdata->enable_resume_por)) {
		pr_err("Failed to get property: enable_resume_por\n");
		goto err_max1187x_get_platdata_dt;
	}

	return pdata;

err_max1187x_get_platdata_dt:
	devm_kfree(dev, pdata);
	return NULL;
}
/*
#else
static inline struct max1187x_pdata *
	max1187x_get_platdata_dt(struct device *dev)
{
	return NULL;
}
#endif
*/

static int validate_pdata(struct max1187x_pdata *pdata)
{
	if (pdata == NULL) {
		pr_err("Platform data not found!\n");
		goto err_validate_pdata;
	}

	if (pdata->gpio_tirq == 0) {
		pr_err("gpio_tirq (%u) not defined!\n", pdata->gpio_tirq);
		goto err_validate_pdata;
	}

	if (pdata->gpio_pwr_en == 0)
		pr_err("gpio_pwr_en (%u) not defined!\n", pdata->gpio_pwr_en);

	if (pdata->lcd_x < 480 || pdata->lcd_x > 0x7FFF) {
		pr_err("lcd_x (%u) out of range!\n", pdata->lcd_x);
		goto err_validate_pdata;
	}

	if (pdata->lcd_y < 240 || pdata->lcd_y > 0x7FFF) {
		pr_err("lcd_y (%u) out of range!\n", pdata->lcd_y);
		goto err_validate_pdata;
	}

	if (pdata->num_sensor_x == 0 || pdata->num_sensor_x > 40) {
		pr_err("num_sensor_x (%u) out of range!\n",
				pdata->num_sensor_x);
		goto err_validate_pdata;
	}

	if (pdata->num_sensor_y == 0 || pdata->num_sensor_y > 40) {
		pr_err("num_sensor_y (%u) out of range!\n",
				pdata->num_sensor_y);
		goto err_validate_pdata;
	}

	return 0;

err_validate_pdata:
	return -ENXIO;
}

static int max1187x_chip_init(struct max1187x_pdata *pdata, int value)
{
	int  ret;

	if (value) {
		ret = gpio_request(pdata->gpio_tirq, "max1187x_tirq");
		if (ret) {
			pr_err("GPIO request failed for max1187x_tirq (%d)\n",
				pdata->gpio_tirq);
			return -EIO;
		}
		ret = gpio_direction_input(pdata->gpio_tirq);
		if (ret) {
			pr_err("GPIO set input direction failed for "\
				"max1187x_tirq (%d)\n", pdata->gpio_tirq);
			gpio_free(pdata->gpio_tirq);
			return -EIO;
		}
	} else {
		gpio_free(pdata->gpio_tirq);
	}

	return 0;
}

static void max1187x_gpio_pwr_en_init(struct max1187x_pdata *pdata,
								bool enable)
{
	int  ret;

	if (enable) {
		ret = gpio_request(pdata->gpio_pwr_en, "max1187x_gpio_pwr_en");
		if (ret) {
			pr_err("GPIO request failed for gpio_pwr_en (%d)\n",
				pdata->gpio_pwr_en);
			return;
		}
		ret = gpio_direction_output(pdata->gpio_pwr_en, 0);
		if (ret) {
			pr_err("GPIO set output direction failed for "\
				"max1187x_gpio_pwr_en (%d)\n",
				pdata->gpio_pwr_en);
			gpio_free(pdata->gpio_pwr_en);
		}
	} else {
		gpio_free(pdata->gpio_pwr_en);
	}
}

static int device_init_thread(void *arg)
{
	return device_init((struct i2c_client *) arg);
}

static int device_init(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct data *ts = NULL;
	struct max1187x_pdata *pdata = NULL;
	struct device_attribute **dev_attr = dev_attrs;
	struct kobject *parent;
	int ret = 0;

	init_state = 1;
	dev_info(dev, "(INIT): Start");

	/* if I2C functionality is not present we are done */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("I2C core driver does not support I2C functionality");
		ret = -ENXIO;
		goto err_device_init;
	}
	pr_info_if(8, "(INIT): I2C functionality OK");

	/* allocate control block; nothing more to do if we can't */
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts) {
		pr_err("Failed to allocate control block memory");
		ret = -ENOMEM;
		goto err_device_init;
	}

	/* Get platform data */
#ifdef MAX1187X_LOCAL_PDATA
	pdata = &local_pdata;
	if (!pdata) {
		pr_err("Platform data is missing");
		ret = -ENXIO;
		goto err_device_init_pdata;
	}
#else
	pdata = dev_get_platdata(dev);
	/* If pdata is missing, try to get pdata from device tree (dts) */
	if (!pdata)
		pdata = max1187x_get_platdata_dt(dev);

	/* Validate if pdata values are okay */
	ret = validate_pdata(pdata);
	if (ret < 0)
		goto err_device_init_pdata;
	pr_info_if(8, "(INIT): Platform data OK");
#endif

	ts->pdata = pdata;
	ts->client = client;
	i2c_set_clientdata(client, ts);
	mutex_init(&ts->fw_mutex);
	mutex_init(&ts->irq_mutex);
	mutex_init(&ts->i2c_mutex);
	mutex_init(&ts->report_mutex);
	sema_init(&ts->report_sem, 1);
	sema_init(&ts->sema_cmd, 1);
	sema_init(&ts->sema_rbcmd, 1);
	ts->button0 = 0;
	ts->button1 = 0;
	ts->button2 = 0;
	ts->button3 = 0;

	/* Create workqueue with maximum 1 running task */
	ts->wq = alloc_workqueue("max1187x_wq", WQ_UNBOUND | WQ_HIGHPRI, 1);
	if (ts->wq == NULL) {
		pr_err("Not able to create workqueue\n");
		ret = -ENOMEM;
		goto err_device_init_memalloc;
	}
	INIT_WORK(&ts->work_irq, max1187x_wfxn_irq);
	INIT_WORK(&ts->work_cmd, max1187x_wfxn_cmd);
	init_waitqueue_head(&ts->waitqueue_all);
	init_waitqueue_head(&ts->waitqueue_rbcmd);

	pr_info_if(8, "(INIT): Memory allocation OK");

	ret = vreg_configure(ts, 1);
	if (ret < 0) {
			pr_err("Failed to configure VREG");
			return 0;
	}
	pr_info_if(8, "(INIT): VREG OK");
	msleep(100);

	/* Initialize GPIO pins */
	if (max1187x_chip_init(ts->pdata, 1) < 0) {
		ret = -EIO;
		goto err_device_init_gpio;
	}
	pr_info_if(8, "(INIT): chip init OK");

	if (pdata->gpio_pwr_en)
		max1187x_gpio_pwr_en_init(ts->pdata, true);

	/* allocate and register touch device */
	ts->input_dev = input_allocate_device();
	if (!ts->input_dev) {
		pr_err("Failed to allocate touch input device");
		ret = -ENOMEM;
		goto err_device_init_inputdev;
	}
	snprintf(ts->phys, sizeof(ts->phys), "%s/input0",
			dev_name(dev));
	ts->input_dev->name = MAX1187X_TOUCH;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;
	__set_bit(EV_SYN, ts->input_dev->evbit);
	__set_bit(EV_ABS, ts->input_dev->evbit);
	__set_bit(EV_KEY, ts->input_dev->evbit);

#ifdef MAX1187X_PROTOCOL_A
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 10, 0, 0);
#else
	input_mt_init_slots(ts->input_dev, MAX1187X_TOUCH_COUNT_MAX);
#endif
	ts->list_finger_ids = 0;
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
			PDATA(panel_margin_xl),
			PDATA(panel_margin_xl) + PDATA(lcd_x), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
			PDATA(panel_margin_yl),
			PDATA(panel_margin_yl) + PDATA(lcd_y), 0, 0);
#ifndef MAX1187X_PRESSURE_SHAPING
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 0xFFFF, 0, 0);
#else
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE,
			0, PRESSURE_MAX_SQRT, 0, 0);
#endif
	input_set_abs_params(ts->input_dev, ABS_MT_TOOL_TYPE,
			MT_TOOL_FINGER, MT_TOOL_FINGER, 0, 0);
#if MAX1187X_TOUCH_REPORT_MODE == 2
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
			0, PDATA(lcd_x) + PDATA(lcd_y), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MINOR,
			0, PDATA(lcd_x) + PDATA(lcd_y), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_ORIENTATION, -90, 90, 0, 0);
#endif
	if (PDATA(button_code0) != KEY_RESERVED)
		set_bit(pdata->button_code0, ts->input_dev->keybit);
	if (PDATA(button_code1) != KEY_RESERVED)
		set_bit(pdata->button_code1, ts->input_dev->keybit);
	if (PDATA(button_code2) != KEY_RESERVED)
		set_bit(pdata->button_code2, ts->input_dev->keybit);
	if (PDATA(button_code3) != KEY_RESERVED)
		set_bit(pdata->button_code3, ts->input_dev->keybit);

	ret = input_register_device(ts->input_dev);
	if (ret) {
		pr_err("Failed to register touch input device");
		ret = -EPERM;
		input_free_device(ts->input_dev);
		goto err_device_init_inputdev;
	}
	pr_info_if(8, "(INIT): Input touch device OK");

	/* allocate and register touch device */
	ts->input_pen = input_allocate_device();
	if (!ts->input_pen) {
		pr_err("Failed to allocate touch input pen device");
		ret = -ENOMEM;
		goto err_device_init_inputpendev;
	}
	snprintf(ts->phys, sizeof(ts->phys), "%s/input0",
			dev_name(dev));
	ts->input_pen->name = MAX1187X_PEN;
	ts->input_pen->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;
	__set_bit(EV_SYN, ts->input_pen->evbit);
	__set_bit(EV_ABS, ts->input_pen->evbit);

#ifdef MAX1187X_PROTOCOL_A
	input_set_abs_params(ts->input_pen, ABS_MT_TRACKING_ID, 0, 10, 0, 0);
#else
	input_mt_init_slots(ts->input_pen, MAX1187X_TOUCH_COUNT_MAX);
#endif
	ts->list_finger_ids = 0;
	input_set_abs_params(ts->input_pen, ABS_MT_POSITION_X,
			PDATA(panel_margin_xl),
			PDATA(panel_margin_xl) + PDATA(lcd_x), 0, 0);
	input_set_abs_params(ts->input_pen, ABS_MT_POSITION_Y,
			PDATA(panel_margin_yl),
			PDATA(panel_margin_yl) + PDATA(lcd_y), 0, 0);
	input_set_abs_params(ts->input_pen, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);
	input_set_abs_params(ts->input_pen, ABS_MT_TOOL_TYPE,
			0, MT_TOOL_MAX, 0, 0);
#if MAX1187X_TOUCH_REPORT_MODE == 2
	input_set_abs_params(ts->input_pen, ABS_MT_TOUCH_MAJOR,
			0, PDATA(lcd_x) + PDATA(lcd_y), 0, 0);
	input_set_abs_params(ts->input_pen, ABS_MT_TOUCH_MINOR,
			0, PDATA(lcd_x) + PDATA(lcd_y), 0, 0);
	input_set_abs_params(ts->input_pen, ABS_MT_ORIENTATION, -90, 90, 0, 0);
#endif
	ret = input_register_device(ts->input_pen);
	if (ret) {
		pr_err("Failed to register touch pen input device");
		ret = -EPERM;
		input_free_device(ts->input_pen);
		goto err_device_init_inputpendev;
	}
	pr_info_if(8, "(INIT): Input touch pen device OK");


#ifdef TOUCH_WAKEUP_FEATURE
	ts->input_dev_key = input_allocate_device();
	if (!ts->input_dev_key) {
		pr_err("Failed to allocate touch input key device");
		ret = -ENOMEM;
		goto err_device_init_inputdevkey;
	}
	snprintf(ts->phys_key, sizeof(ts->phys_key), "%s/input1",
		dev_name(&client->dev));
	ts->input_dev_key->name = MAX1187X_KEY;
	ts->input_dev_key->phys = ts->phys_key;
	ts->input_dev_key->id.bustype = BUS_I2C;
	__set_bit(EV_KEY, ts->input_dev_key->evbit);
	set_bit(KEY_POWER, ts->input_dev_key->keybit);
	ret = input_register_device(ts->input_dev_key);
	if (ret) {
		pr_err("Failed to register touch input key device");
		ret = -EPERM;
		input_free_device(ts->input_dev_key);
		goto err_device_init_inputdevkey;
	}
	pr_info_if(8, "(INIT): Input key device OK");
#endif

	/* Setup IRQ and handler */
	ret = request_irq(client->irq, irq_handler,
			IRQF_TRIGGER_FALLING, client->name, ts);
	if (ret != 0) {
			pr_err("Failed to setup IRQ handler");
			ret = -EIO;
			goto err_device_init_irq;
	}
	pr_info_if(8, "(INIT): IRQ handler OK");

	/* collect controller ID and configuration ID data from firmware   */
	mutex_lock(&ts->fw_mutex);
	ret = read_chip_data(ts);
	if (ret)
		pr_warning("No firmware response (%d)", ret);
	mutex_unlock(&ts->fw_mutex);

	/* configure suspend/resume */
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	ts->early_suspend.suspend = early_suspend;
	ts->early_suspend.resume = late_resume;
	register_early_suspend(&ts->early_suspend);
#elif defined CONFIG_FB
	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if (ret)
		pr_err("Unable to register fb_notifier");
#endif

	ts->is_suspended = 0;
	pr_info_if(8, "(INIT): suspend/resume registration OK");

	/* set up debug interface */
	while (*dev_attr) {
		if (device_create_file(&client->dev, *dev_attr) < 0) {
			pr_err("failed to create sysfs file");
			return 0;
		}
		ts->sysfs_created++;
		dev_attr++;
	}

	if (device_create_bin_file(&client->dev, &dev_attr_report) < 0) {
		pr_err("failed to create sysfs file [report]");
		return 0;
	}
	ts->sysfs_created++;

	/* create symlink */
	parent = ts->input_dev->dev.kobj.parent;
	ret = sysfs_create_link(parent, &client->dev.kobj, MAX1187X_NAME);
	if (ret)
		pr_err("sysfs_create_link error\n");

#ifdef TOUCH_WAKEUP_FEATURE
	pr_info("Touch Wakeup Feature enabled\n");
	device_init_wakeup(&client->dev, 1);
	device_wakeup_disable(&client->dev);
#endif

	pr_info("(INIT): Done\n");
	return 0;

err_device_init_irq:
#ifdef TOUCH_WAKEUP_FEATURE
	input_unregister_device(ts->input_dev_key);
err_device_init_inputdevkey:
#endif
	input_unregister_device(ts->input_pen);
err_device_init_inputpendev:
	input_unregister_device(ts->input_dev);
err_device_init_inputdev:
err_device_init_gpio:
err_device_init_memalloc:
err_device_init_pdata:
	kfree(ts);
err_device_init:
	return ret;
}

static int device_deinit(struct i2c_client *client)
{
	struct data *ts = i2c_get_clientdata(client);
	struct max1187x_pdata *pdata;
	struct device_attribute **dev_attr = dev_attrs;

	if (ts == NULL)
		return 0;

	pdata = ts->pdata;

	propagate_report(ts, -1, NULL);

	init_state = 0;

#ifdef TOUCH_WAKEUP_FEATURE
	device_init_wakeup(&client->dev, 0);
#endif

	while (*dev_attr) {
		if (ts->sysfs_created && ts->sysfs_created--)
			device_remove_file(&client->dev, *dev_attr);
		dev_attr++;
	}
	if (ts->sysfs_created && ts->sysfs_created--)
		device_remove_bin_file(&client->dev, &dev_attr_report);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#elif defined CONFIG_FB
	if (fb_unregister_client(&ts->fb_notif))
		pr_err("Error occurred while unregistering fb_notifier.");
#endif

	if (client->irq)
			free_irq(client->irq, ts);

#ifdef TOUCH_WAKEUP_FEATURE
	input_unregister_device(ts->input_dev_key);
#endif
	sysfs_remove_link(ts->input_dev->dev.kobj.parent,
						MAX1187X_NAME);
	input_unregister_device(ts->input_dev);
	input_unregister_device(ts->input_pen);

	flush_workqueue(ts->wq);
	destroy_workqueue(ts->wq);
	(void) max1187x_chip_init(pdata, 0);
	if (pdata->gpio_pwr_en)
		max1187x_gpio_pwr_en_init(pdata, false);
	vreg_configure(ts, 0);
	kfree(ts);

	pr_info("(INIT): Deinitialized\n");
	return 0;
}

static int probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	if (device_create_file(&client->dev, &dev_attr_init) < 0) {
		pr_err("failed to create sysfs file [init]");
		return 0;
	}

	if (!is_booting())
		return device_init(client);
	if (IS_ERR(kthread_run(device_init_thread, (void *) client,
			MAX1187X_NAME))) {
		pr_err("failed to start kernel thread");
		return -EAGAIN;
	}
	return 0;
}

static int remove(struct i2c_client *client)
{
	int ret = device_deinit(client);

	device_remove_file(&client->dev, &dev_attr_init);
	return ret;
}

/*
 Commands
 */

static void process_rbcmd(struct data *ts)
{
	if (ts->rbcmd_waiting == 0)
		return;
	if (ts->rbcmd_report_id != ts->rx_report[1])
		return;

	ts->rbcmd_received = 1;
	memcpy(ts->rbcmd_rx_report, ts->rx_report, (ts->rx_report_len + 1)<<1);
	ts->rbcmd_rx_report_len = ts->rx_report_len;
	wake_up_interruptible(&ts->waitqueue_rbcmd);
}

static int combine_multipacketreport(struct data *ts, u16 *report)
{
	u16 packet_header = report[0];
	u8 packet_seq_num = BYTEH(packet_header);
	u8 packet_size = BYTEL(packet_header);
	u16 total_packets, this_packet_num, offset;
	static u16 packet_seq_combined;

	if (packet_seq_num == 0x11) {
		memcpy(ts->rx_report, report, (packet_size + 1) << 1);
		ts->rx_report_len = packet_size;
		packet_seq_combined = 1;
		return 0;
	}

	total_packets = (packet_seq_num & 0xF0) >> 4;
	this_packet_num = packet_seq_num & 0x0F;

	if (this_packet_num == 1) {
		if (report[1] == 0x0800) {
			ts->rx_report_len = report[2] + 2;
			packet_seq_combined = 1;
			memcpy(ts->rx_report, report, (packet_size + 1) << 1);
			return -EAGAIN;
		} else {
			return -EIO;
		}
	} else if (this_packet_num == packet_seq_combined + 1) {
		packet_seq_combined++;
		offset = (this_packet_num - 1) * 0xF4 + 1;
		memcpy(ts->rx_report + offset, report + 1, packet_size << 1);
		if (total_packets == this_packet_num)
			return 0;
		else
			return -EIO;
	}
	return -EIO;
}

static void propagate_report(struct data *ts, int status, u16 *report)
{
	int i, ret;

	down(&ts->report_sem);
	mutex_lock(&ts->report_mutex);

	if (report) {
		ret = combine_multipacketreport(ts, report);
		if (ret) {
			up(&ts->report_sem);
			mutex_unlock(&ts->report_mutex);
			return;
		}
	}
	process_rbcmd(ts);

	for (i = 0; i < MAX_REPORT_READERS; i++) {
		if (status == 0) {
			if (ts->report_readers[i].report_id == 0xFFFF
				|| (ts->rx_report[1] != 0
				&& ts->report_readers[i].report_id
				== ts->rx_report[1])) {
				up(&ts->report_readers[i].sem);
				ts->report_readers[i].reports_passed++;
				ts->report_readers_outstanding++;
			}
		} else {
			if (ts->report_readers[i].report_id != 0) {
				ts->report_readers[i].status = status;
				up(&ts->report_readers[i].sem);
			}
		}
	}
	if (ts->report_readers_outstanding == 0)
		up(&ts->report_sem);
	mutex_unlock(&ts->report_mutex);
}

static int get_report(struct data *ts, u16 report_id, ulong timeout)
{
	int i, ret, status;

	mutex_lock(&ts->report_mutex);
	for (i = 0; i < MAX_REPORT_READERS; i++)
		if (ts->report_readers[i].report_id == 0)
			break;
	if (i == MAX_REPORT_READERS) {
		mutex_unlock(&ts->report_mutex);
		pr_err("maximum readers reached");
		return -EBUSY;
	}
	ts->report_readers[i].report_id = report_id;
	sema_init(&ts->report_readers[i].sem, 1);
	down(&ts->report_readers[i].sem);
	ts->report_readers[i].status = 0;
	ts->report_readers[i].reports_passed = 0;
	mutex_unlock(&ts->report_mutex);

	if (timeout == 0xFFFFFFFF)
		ret = down_interruptible(&ts->report_readers[i].sem);
	else
		ret = down_timeout(&ts->report_readers[i].sem,
			(timeout * HZ) / 1000);

	mutex_lock(&ts->report_mutex);
	if (ret && ts->report_readers[i].reports_passed > 0)
		if (--ts->report_readers_outstanding == 0)
			up(&ts->report_sem);
	status = ts->report_readers[i].status;
	ts->report_readers[i].report_id = 0;
	mutex_unlock(&ts->report_mutex);

	return (status == 0) ? ret : status;
}

static void release_report(struct data *ts)
{
	mutex_lock(&ts->report_mutex);
	if (--ts->report_readers_outstanding == 0)
		up(&ts->report_sem);
	mutex_unlock(&ts->report_mutex);
}

/* debug_mask |= 0x10 for pm functions */

static void set_suspend_mode(struct data *ts)
{
	u16 cmd_buf[] = {0x0020, 0x0001, 0x0000};
	int ret;

	pr_info_if(0x10, "Enter\n");

	disable_irq(ts->client->irq);
	ts->is_suspended = 1;

	flush_workqueue(ts->wq);

#ifdef TOUCH_WAKEUP_FEATURE
	if (device_may_wakeup(&ts->client->dev))
		cmd_buf[2] = 0x6;
#endif
	ret = cmd_send(ts, cmd_buf, 3);
	if (ret)
		pr_err("Failed to set sleep mode");

	flush_workqueue(ts->wq);

	if (cmd_buf[2] != 0x6) {
		usleep_range(1000, 2000);
		vreg_suspend(ts, 1);
	}

#ifdef TOUCH_WAKEUP_FEATURE
	if (device_may_wakeup(&ts->client->dev))
		enable_irq(ts->client->irq);
#endif

	pr_info_if(0x10, "Exit\n");
	return;
}

static void set_resume_mode(struct data *ts)
{
	u16 cmd_buf[] = {0x0020, 0x0001, 0x0002};
	int ret;

	pr_info_if(0x10, "Enter\n");

	vreg_suspend(ts, 0);
	usleep_range(1000, 2000);

#ifdef TOUCH_WAKEUP_FEATURE
	if (device_may_wakeup(&ts->client->dev))
		disable_irq(ts->client->irq);
#endif

	if (PDATA(enable_resume_por))
		if (reset_power(ts))
			pr_err("Failed to do power on reset");

	ret = cmd_send(ts, cmd_buf, 3);
	if (ret)
		pr_err("Failed to set active mode");

	cmd_buf[0] = 0x0018;
	cmd_buf[1] = 0x0001;
	cmd_buf[2] = MAX1187X_TOUCH_REPORT_MODE;
	ret = cmd_send(ts, cmd_buf, 3);
	if (ret)
		pr_err("Failed to set up touch report mode");

	flush_workqueue(ts->wq);

	ts->is_suspended = 0;

	enable_irq(ts->client->irq);

	pr_info_if(0x10, "Exit\n");

	return;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void early_suspend(struct early_suspend *h)
{
	struct data *ts = container_of(h, struct data, early_suspend);

	pr_info_if(0x10, "Enter\n");

	set_suspend_mode(ts);

	pr_info_if(0x10, "Exit\n");
	return;
}

static void late_resume(struct early_suspend *h)
{
	struct data *ts = container_of(h, struct data, early_suspend);

	pr_info_if(0x10, "Enter\n");

	set_resume_mode(ts);

	pr_info_if(0x10, "Exit\n");
}
#elif defined CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct data *ts = container_of(self, struct data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK && ts &&
			ts->client) {
		blank = evdata->data;
		if (ts->is_suspended == 0 && *blank != FB_BLANK_UNBLANK) {
			pr_info_if(0x10, "FB_BLANK_BLANKED\n");
			set_suspend_mode(ts);
		} else if (ts->is_suspended == 1
				&& *blank == FB_BLANK_UNBLANK) {
			pr_info_if(0x10, "FB_BLANK_UNBLANK\n");
			set_resume_mode(ts);
		}
	}
	return 0;
}
#endif

static int suspend(struct device *dev)
{
#ifdef TOUCH_WAKEUP_FEATURE
	struct i2c_client *client = to_i2c_client(dev);
#endif
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	struct data *ts = i2c_get_clientdata(to_i2c_client(dev));
#endif

	pr_info_if(0x10, "Enter\n");

#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	set_suspend_mode(ts);
#endif

#ifdef TOUCH_WAKEUP_FEATURE
	if (device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);
#endif

	pr_info_if(0x10, "Exit\n");

	return 0;
}

static int resume(struct device *dev)
{
#ifdef TOUCH_WAKEUP_FEATURE
	struct i2c_client *client = to_i2c_client(dev);
#endif
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	struct data *ts = i2c_get_clientdata(to_i2c_client(dev));
#endif

	pr_info_if(0x10, "Enter\n");

#ifdef TOUCH_WAKEUP_FEATURE
	if (device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);
#endif

#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	set_resume_mode(ts);
#endif

	pr_info_if(0x10, "Exit\n");

	return 0;
}

static const struct dev_pm_ops max1187x_pm_ops = {
	.resume = resume,
	.suspend = suspend,
};

#define STATUS_ADDR_H 0x00
#define STATUS_ADDR_L 0xFF
#define DATA_ADDR_H   0x00
#define DATA_ADDR_L   0xFE
#define STATUS_READY_H 0xAB
#define STATUS_READY_L 0xCC
#define RXTX_COMPLETE_H 0x54
#define RXTX_COMPLETE_L 0x32
static int bootloader_read_status_reg(struct data *ts, const u8 byteL,
	const u8 byteH)
{
	u8 buffer[] = { STATUS_ADDR_L, STATUS_ADDR_H }, i;

	for (i = 0; i < 3; i++) {
		if (i2c_tx_bytes(ts, buffer, 2) != 2) {
			pr_err("TX fail");
			return -EIO;
		}
		if (i2c_rx_bytes(ts, buffer, 2) != 2) {
			pr_err("RX fail");
			return -EIO;
		}
		if (buffer[0] == byteL && buffer[1] == byteH)
			break;
	}
	if (i == 3) {
		pr_err("Unexpected status => %02X%02X vs %02X%02X",
				buffer[0], buffer[1], byteL, byteH);
		return -EIO;
	}

	return 0;
}

static int bootloader_write_status_reg(struct data *ts, const u8 byteL,
	const u8 byteH)
{
	u8 buffer[] = { STATUS_ADDR_L, STATUS_ADDR_H, byteL, byteH };

	if (i2c_tx_bytes(ts, buffer, 4) != 4) {
		pr_err("TX fail");
		return -EIO;
	}
	return 0;
}

static int bootloader_rxtx_complete(struct data *ts)
{
	return bootloader_write_status_reg(ts, RXTX_COMPLETE_L,
				RXTX_COMPLETE_H);
}

static int bootloader_read_data_reg(struct data *ts, u8 *byteL, u8 *byteH)
{
	u8 buffer[] = { DATA_ADDR_L, DATA_ADDR_H, 0x00, 0x00 };

	if (i2c_tx_bytes(ts, buffer, 2) != 2) {
		pr_err("TX fail");
		return -EIO;
	}
	if (i2c_rx_bytes(ts, buffer, 4) != 4) {
		pr_err("RX fail");
		return -EIO;
	}
	if (buffer[2] != 0xCC && buffer[3] != 0xAB) {
		pr_err("Status is not ready");
		return -EIO;
	}

	*byteL = buffer[0];
	*byteH = buffer[1];
	return bootloader_rxtx_complete(ts);
}

static int bootloader_write_data_reg(struct data *ts, const u8 byteL,
	const u8 byteH)
{
	u8 buffer[6] = { DATA_ADDR_L, DATA_ADDR_H, byteL, byteH,
			RXTX_COMPLETE_L, RXTX_COMPLETE_H };

	if (bootloader_read_status_reg(ts, STATUS_READY_L,
		STATUS_READY_H) < 0) {
		pr_err("read status register fail");
		return -EIO;
	}
	if (i2c_tx_bytes(ts, buffer, 6) != 6) {
		pr_err("TX fail");
		return -EIO;
	}
	return 0;
}

static int bootloader_rxtx(struct data *ts, u8 *byteL, u8 *byteH,
	const int tx)
{
	if (tx > 0) {
		if (bootloader_write_data_reg(ts, *byteL, *byteH) < 0) {
			pr_err("write data register fail");
			return -EIO;
		}
		return 0;
	}

	if (bootloader_read_data_reg(ts, byteL, byteH) < 0) {
		pr_err("read data register fail");
		return -EIO;
	}
	return 0;
}

static int bootloader_get_cmd_conf(struct data *ts, int retries)
{
	u8 byteL, byteH;

	do {
		if (bootloader_read_data_reg(ts, &byteL, &byteH) >= 0) {
			if (byteH == 0x00 && byteL == 0x3E)
				return 0;
		}
		retries--;
	} while (retries > 0);

	return -EIO;
}

static int bootloader_write_buffer(struct data *ts, u8 *buffer, int size)
{
	u8 byteH = 0x00;
	int k;

	for (k = 0; k < size; k++) {
		if (bootloader_rxtx(ts, &buffer[k], &byteH, 1) < 0) {
			pr_err("bootloader RX-TX fail");
			return -EIO;
		}
	}
	return 0;
}

static int bootloader_enter(struct data *ts)
{
	int i;
	u16 enter[3][2] = { { 0x7F00, 0x0047 }, { 0x7F00, 0x00C7 }, { 0x7F00,
			0x0007 } };

	for (i = 0; i < 3; i++) {
		if (i2c_tx_words(ts, enter[i], 2) != 2) {
			pr_err("Failed to enter bootloader");
			return -EIO;
		}
	}

	if (bootloader_get_cmd_conf(ts, 5) < 0) {
		pr_err("Failed to enter bootloader mode");
		return -EIO;
	}
	return 0;
}

static int bootloader_exit(struct data *ts)
{
	int i;
	u16 exit[3][2] = { { 0x7F00, 0x0040 }, { 0x7F00, 0x00C0 }, { 0x7F00,
			0x0000 } };

	for (i = 0; i < 3; i++) {
		if (i2c_tx_words(ts, exit[i], 2) != 2) {
			pr_err("Failed to exit bootloader");
			return -EIO;
		}
	}

	return 0;
}

static int bootloader_get_crc(struct data *ts, u16 *crc16,
		u16 addr, u16 len, u16 delay)
{
	u8 crc_command[] = {0x30, 0x02, BYTEL(addr),
			BYTEH(addr), BYTEL(len), BYTEH(len)};
	u8 byteL = 0, byteH = 0;
	u16 rx_crc16 = 0;

	if (bootloader_write_buffer(ts, crc_command, 6) < 0) {
		pr_err("write buffer fail");
		return -EIO;
	}
	msleep(delay);

	/* reads low 8bits (crcL) */
	if (bootloader_rxtx(ts, &byteL, &byteH, 0) < 0) {
		pr_err("Failed to read low byte of crc response!");
		return -EIO;
	}
	rx_crc16 = (u16) byteL;

	/* reads high 8bits (crcH) */
	if (bootloader_rxtx(ts, &byteL, &byteH, 0) < 0) {
		pr_err("Failed to read high byte of crc response!");
		return -EIO;
	}
	rx_crc16 = (u16)(byteL << 8) | rx_crc16;

	if (bootloader_get_cmd_conf(ts, 5) < 0) {
		pr_err("CRC get failed!");
		return -EIO;
	}
	*crc16 = rx_crc16;

	return 0;
}

static int bootloader_set_byte_mode(struct data *ts)
{
	u8 buffer[2] = { 0x0A, 0x00 };

	if (bootloader_write_buffer(ts, buffer, 2) < 0) {
		pr_err("write buffer fail");
		return -EIO;
	}
	if (bootloader_get_cmd_conf(ts, 10) < 0) {
		pr_err("command confirm fail");
		return -EIO;
	}
	return 0;
}

static int bootloader_erase_flash(struct data *ts)
{
	u8 byteL = 0x02, byteH = 0x00;
	int i, verify = 0;

	if (bootloader_rxtx(ts, &byteL, &byteH, 1) < 0) {
		pr_err("bootloader RX-TX fail");
		return -EIO;
	}

	for (i = 0; i < 10; i++) {
		msleep(60); /* wait 60ms */

		if (bootloader_get_cmd_conf(ts, 0) < 0)
			continue;

		verify = 1;
		break;
	}

	if (verify != 1) {
		pr_err("Flash Erase failed");
		return -EIO;
	}

	return 0;
}

static int bootloader_write_flash(struct data *ts, const u8 *image, u16 length)
{
	u8 buffer[130];
	u8 length_L = length & 0xFF;
	u8 length_H = (length >> 8) & 0xFF;
	u8 command[] = { 0xF0, 0x00, length_H, length_L, 0x00 };
	u16 blocks_of_128bytes;
	int i, j;

	if (bootloader_write_buffer(ts, command, 5) < 0) {
		pr_err("write buffer fail");
		return -EIO;
	}

	blocks_of_128bytes = length >> 7;

	for (i = 0; i < blocks_of_128bytes; i++) {
		for (j = 0; j < 100; j++) {
			usleep_range(1500, 2000);
			if (bootloader_read_status_reg(ts, STATUS_READY_L,
			STATUS_READY_H)	== 0)
				break;
		}
		if (j == 100) {
			pr_err("Failed to read Status register!");
			return -EIO;
		}

		buffer[0] = ((i % 2) == 0) ? 0x00 : 0x40;
		buffer[1] = 0x00;
		memcpy(buffer + 2, image + i * 128, 128);

		if (i2c_tx_bytes(ts, buffer, 130) != 130) {
			pr_err("Failed to write data (%d)", i);
			return -EIO;
		}
		if (bootloader_rxtx_complete(ts) < 0) {
			pr_err("Transfer failure (%d)", i);
			return -EIO;
		}
	}

	usleep_range(10000, 11000);
	if (bootloader_get_cmd_conf(ts, 5) < 0) {
		pr_err("Flash programming failed");
		return -EIO;
	}
	return 0;
}

/****************************************
 *
 * Standard Driver Structures/Functions
 *
 ****************************************/
static const struct i2c_device_id id[] = { { MAX1187X_NAME, 0 }, { } };

MODULE_DEVICE_TABLE(i2c, id);

static struct of_device_id max1187x_dt_match[] = {
	{ .compatible = "maxim,max1187x_tsc" },	{ } };

static struct i2c_driver driver = {
		.probe = probe,
		.remove = remove,
		.id_table = id,
		.driver = {
			.name = MAX1187X_NAME,
			.owner	= THIS_MODULE,
			.of_match_table = max1187x_dt_match,
			.pm = &max1187x_pm_ops,
		},
};

static int __devinit max1187x_init(void)
{
	return i2c_add_driver(&driver);
}

static void __exit max1187x_exit(void)
{
	i2c_del_driver(&driver);
}

module_init(max1187x_init);
module_exit(max1187x_exit);

MODULE_AUTHOR("Maxim Integrated Products, Inc.");
MODULE_DESCRIPTION("MAX1187X Touchscreen Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("3.1.8");
