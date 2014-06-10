/******************************************************************************
 *
 *  file name       : tuner_drv_hw.c
 *  brief note      : HW Control Layer of Tmm Tuner Driver
 *
 *  creation data   : 2011.07.25
 *  author          : K.Kitamura(*)
 *  special affairs : none
 *
 *  $Rev:: 322                        $ Revision of Last commit
 *  $Date:: 2011-10-26 13:33:02 +0900#$ Date of last commit
 *
 *              Copyright (C) 2011 by Panasonic Co., Ltd.
 *              Copyright (C) 2012 Sony Mobile Communications AB.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 ******************************************************************************
 * HISTORY      : 2011/07/25    K.Kitamura(*)
 *                001 new creation
 ******************************************************************************/
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include "tuner_drv.h"

int set_enable_bit(
	struct _tuner_data_rw *data,
	int loop_cnt,
	unsigned char *ena_data)
{
	int cnt;

	if (!data || 0 > loop_cnt || !ena_data)
		return -EINVAL;

	if ((data[loop_cnt].sbit == SET_ENABIT) &&
		(data[loop_cnt].ebit == SET_ENABIT)) {
		*ena_data = (unsigned char)data[loop_cnt].enabit;
	} else {
		for (cnt = 0; cnt < TUNER_CHARNUM_MAX; cnt++) {
			if ((cnt >= data[loop_cnt].sbit) &&
				(cnt <= data[loop_cnt].ebit)) {
				*ena_data |= (unsigned char)(1 << cnt);
			}
		}
	}
	return 0;
}

int tuner_drv_hw_access(
	unsigned int ucommand,
	struct _tuner_data_rw *data,
	unsigned short param_len)
{
	int                ret;
	struct i2c_adapter *adap;
	struct i2c_msg     msgs[2];
	unsigned short     addr;
	unsigned short     flags;
	unsigned char      buf[TUNER_I2C_MSG_DATA_NUM];
	unsigned char      *buf_all;
	unsigned char      read_data;
	unsigned char      ena_data;
	unsigned char      write_data;
	unsigned short     lp_cnt;

	adap = i2c_get_adapter(TUNER_CONFIG_I2C_BUSNUM);

	if (!adap || !data)
		return -EINVAL;

	memset(msgs, 0x00, sizeof(struct i2c_msg) * 2);
	ena_data = 0x00;
	flags = 0;

	if (param_len >= TUNER_I2C_WRITE_ALL_NUM) {
		buf_all = vmalloc(param_len + 1);
		if (!buf_all) {
			i2c_put_adapter(adap);
			return -EINVAL;
		}

		memset(buf_all, 0x00, param_len + 1);
		msgs[0].addr  = data[0].slave_adr;
		msgs[0].flags = 0;
		msgs[0].len   = (unsigned short)(param_len + 1);
		msgs[0].buf   = buf_all;

		*buf_all = (unsigned char)data[0].adr;

		for (lp_cnt = 0; lp_cnt < param_len; lp_cnt++) {
			buf_all[lp_cnt + 1]
				= (unsigned char)data[lp_cnt].param;
		}

		ret = i2c_transfer(adap, msgs, TUNER_W_MSGNUM);

		vfree(buf_all);

		if (ret < 0) {
			i2c_put_adapter(adap);
			return -EINVAL;
		}
		i2c_put_adapter(adap);
		return 0;
	}

	for (lp_cnt = 0; lp_cnt < param_len; lp_cnt++) {
		memset(msgs, 0x00, sizeof(struct i2c_msg) * 2);
		ena_data = 0x00;

		switch (ucommand) {
		case TUNER_IOCTL_VALGET:
			addr   = data[lp_cnt].slave_adr;
			flags  = I2C_M_RD;
			buf[0] = (unsigned char)data[lp_cnt].adr;
			buf[1] = 0;
			break;
		case TUNER_IOCTL_VALSET:
			addr   = data[lp_cnt].slave_adr;
			flags  = 0;
			buf[0] = (unsigned char)data[lp_cnt].adr;
			buf[1] = (unsigned char)data[lp_cnt].param;
			break;
		default:
			i2c_put_adapter(adap);
			return -EINVAL;
		}

		if (flags != I2C_M_RD) {
			if (!((data[lp_cnt].sbit == 0) &&
				(data[lp_cnt].ebit == 7))) {
				if (set_enable_bit(data, lp_cnt, &ena_data)
					== -EINVAL) {
					return -EINVAL;
				}
				if (ena_data != 0xFF) {
					msgs[0].addr  = addr;
					msgs[0].flags = 0;
					msgs[0].len   = TUNER_R_MSGLEN;
					msgs[0].buf   = &buf[0];
					msgs[1].addr  = addr;
					msgs[1].flags = I2C_M_RD;
					msgs[1].len   = TUNER_R_MSGLEN;
					msgs[1].buf   = &read_data;

					ret = i2c_transfer(
						adap,
						msgs,
						TUNER_R_MSGNUM);
					if (ret < 0) {
						i2c_put_adapter(adap);
						return -EINVAL;
					}

					memset(
						msgs,
						0x00,
						sizeof(struct i2c_msg) * 2);

					read_data &= (unsigned char)(~ena_data);

					write_data =
						(unsigned char)(ena_data &
							data[lp_cnt].param);
					buf[1] =
						(unsigned char)(write_data |
							read_data);
				}
			}
			msgs[0].addr  = addr;
			msgs[0].flags = flags;
			msgs[0].len   = TUNER_W_MSGLEN;
			msgs[0].buf   = buf;

			ret = i2c_transfer(adap, msgs, TUNER_W_MSGNUM);
			if (ret < 0) {
				i2c_put_adapter(adap);
				return -EINVAL;
			}
		} else {
			msgs[0].addr  = addr;
			msgs[0].flags = 0;
			msgs[0].len   = TUNER_R_MSGLEN;
			msgs[0].buf   = &buf[0];
			msgs[1].addr  = addr;
			msgs[1].flags = flags;
			msgs[1].len   = TUNER_R_MSGLEN;
			msgs[1].buf   = &buf[1];

			ret = i2c_transfer(adap, msgs, TUNER_R_MSGNUM);
			if (ret < 0) {
				i2c_put_adapter(adap);
				return -EINVAL;
			}
			data[lp_cnt].param = buf[1];
		}
	}
	i2c_put_adapter(adap);
	return 0;
}
