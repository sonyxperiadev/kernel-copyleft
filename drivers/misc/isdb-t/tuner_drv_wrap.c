/******************************************************************************
 *
 *  file name       : tuner_drv_wrap.c
 *  brief note      : The Wrapper Layer for Tmm Tuner Driver
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
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/mfd/pm8xxx/pm8821.h>
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <mach/isdbt_tunerpm.h>
#include "tuner_drv.h"
#include "tuner_drv_sys.h"

#define TUNER_POWER_ON 1
#define TUNER_POWER_OFF 0
#define TUNER_RESET_ON 1
#define TUNER_RESET_OFF 0
#define PM8821_MPP1_ON 1
#define PM8821_MPP1_OFF 0
#define PM8821_MPP2_ON 1
#define PM8821_MPP2_OFF 0
#define PM8821_MPP3_ON 1
#define PM8821_MPP3_OFF 0
#define TUNER_INT 36
#define PM8821_MPP1 0x1
#define PM8821_MPP2 0x2
#define PM8821_MPP3 0x3
#define D_ISDBT_DEVICE_RST_OFF_WAITTIME1 1
#define D_ISDBT_DEVICE_RST_OFF_WAITTIME2 2
#define D_ISDBT_DEVICE_RST_WAITTIME 10
#define D_ISDBT_DEVICE_RST_DELAY 2

struct mutex mutex_lock_power;
static int ant_switch_cnt;
static int irq_num;
static struct regulator *tuner_avdh;

int tunerpm_dev_init(void)
{
	mutex_init(&mutex_lock_power);
	mutex_lock(&mutex_lock_power);
	if (isdbt_tunerpm_init()) {
		mutex_unlock(&mutex_lock_power);
		mutex_destroy(&mutex_lock_power);
		return -EINVAL;
	}
	mutex_unlock(&mutex_lock_power);
	return 0;
}

void tunerpm_dev_finalize(void)
{
	mutex_lock(&mutex_lock_power);
	isdbt_tunerpm_free();
	mutex_unlock(&mutex_lock_power);
	mutex_destroy(&mutex_lock_power);
}

int tuner_drv_ctl_power(int data)
{
	int rc;

	switch (data) {
	case TUNER_DRV_CTL_POWON:
		mutex_lock(&mutex_lock_power);
		isdbt_tunerpm_power_control(TUNER_POWER_ON);
		tuner_avdh = regulator_get(NULL, "8921_l21");
		if (IS_ERR(tuner_avdh))
			tuner_avdh = NULL;
		rc = regulator_set_voltage(tuner_avdh, 1800000, 1800000);
		if (rc) {
			mutex_unlock(&mutex_lock_power);
			goto error_regulator_set_voltage;
		}
		rc = regulator_enable(tuner_avdh);
		if (rc) {
			mutex_unlock(&mutex_lock_power);
			goto error_regulator_enable;
		}
		mutex_unlock(&mutex_lock_power);
		break;
	case TUNER_DRV_CTL_RESET:
		mutex_lock(&mutex_lock_power);
		rc = isdbt_tunerpm_reset_control(TUNER_RESET_ON);
		if (rc) {
			mutex_unlock(&mutex_lock_power);
			goto error_tunerpm_reset;
		}
		msleep(D_ISDBT_DEVICE_RST_DELAY);
		mutex_unlock(&mutex_lock_power);
		break;
	case TUNER_DRV_CTL_POWOFF:
		mutex_lock(&mutex_lock_power);
		msleep(D_ISDBT_DEVICE_RST_OFF_WAITTIME1);
		isdbt_tunerpm_reset_control(TUNER_RESET_OFF);
		msleep(D_ISDBT_DEVICE_RST_OFF_WAITTIME2);
		isdbt_tunerpm_power_control(TUNER_POWER_OFF);
		regulator_disable(tuner_avdh);
		gpio_free(TUNER_INT);
		ant_switch_cnt = 0;
		mutex_unlock(&mutex_lock_power);
		break;
	default:
		goto error_data;
	}
	return 0;

error_regulator_set_voltage:
error_regulator_enable:
error_tunerpm_reset:
error_data:
	return -EINVAL;
}

void tuner_drv_ant_switch(int antmode)
{
	switch (antmode) {
	case ANTMODE_WHIP:
		isdbt_tunerpm_mpp1_control(PM8821_MPP1_OFF);
		isdbt_tunerpm_mpp2_control(PM8821_MPP2_OFF);
		isdbt_tunerpm_mpp3_control(PM8821_MPP3_OFF);
		break;
	case ANTMODE_EARPHONE:
		isdbt_tunerpm_mpp1_control(PM8821_MPP1_OFF);
		isdbt_tunerpm_mpp2_control(PM8821_MPP2_ON);
		isdbt_tunerpm_mpp3_control(PM8821_MPP3_OFF);
		break;
	case ANTMODE_USB:
		isdbt_tunerpm_mpp1_control(PM8821_MPP1_OFF);
		isdbt_tunerpm_mpp2_control(PM8821_MPP2_OFF);
		isdbt_tunerpm_mpp3_control(PM8821_MPP3_ON);
		break;
	case ANTMODE_CHARGER:
		isdbt_tunerpm_mpp1_control(PM8821_MPP1_OFF);
		isdbt_tunerpm_mpp2_control(PM8821_MPP2_ON);
		isdbt_tunerpm_mpp3_control(PM8821_MPP3_ON);
		break;
	case ANTMODE_AUTO:
		isdbt_tunerpm_mpp1_control(PM8821_MPP1_ON);
		isdbt_tunerpm_mpp2_control(PM8821_MPP2_OFF);
		isdbt_tunerpm_mpp3_control(PM8821_MPP3_ON);
		break;
	case ANTMODE_NOTUSE:
		isdbt_tunerpm_mpp1_control(PM8821_MPP1_ON);
		isdbt_tunerpm_mpp2_control(PM8821_MPP2_OFF);
		isdbt_tunerpm_mpp3_control(PM8821_MPP3_ON);
		break;
	default:
		return;
	}
	set_current_ant(antmode);
}

int tuner_drv_set_interrupt(void)
{
	int ret;

	ret = gpio_request(TUNER_INT, "ISDB-T INT");
	if (ret)
		return ret;

	irq_num = gpio_to_irq(TUNER_INT);

	ret = request_threaded_irq(
		irq_num,
		tuner_interrupt,
		NULL,
		IRQF_TRIGGER_RISING,
		"mm_tuner",
		NULL);

	if (ret) {
		gpio_free(TUNER_INT);
		return -EINVAL;
	}
	return 0;
}

void tuner_drv_release_interrupt(void)
{
	free_irq(irq_num, NULL);
}
