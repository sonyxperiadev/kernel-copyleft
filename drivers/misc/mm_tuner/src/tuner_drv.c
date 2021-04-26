/**************************************************************************//**
 *
 *  @file		tuner_drv.c
 *
 *  @brief		The implementation that is independence in physical I/F.
 *
 ***************************************************************************//*
 * Copyright (c) 2015 Socionext Inc.
 ******************************************************************************
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
 ******************************************************************************/
/*
 * Copyright (C) 2019 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */
/*..+....1....+....2....+....3....+....4....+....5....+....6....+....7....+...*/

/******************************************************************************
 * include
 ******************************************************************************/
#include "tuner_drv.h"
#include "tuner_drv_hw.h"
#include "version.h"

#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <uapi/linux/sched/types.h>
/* For Timeout procedure */
#include <linux/jiffies.h>
#include <linux/timer.h>

#if defined(DPATH_GPIF)
/* For DMA Mapping for gpif */
#include <linux/dma-mapping.h>
#endif

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#define D_TUNER_POWER_ON_WAIT_US           10000	/* 10ms */
#define D_TUNER_POWER_ON_WAIT_RANGE_US     10100
#define D_TUNER_NRST_WAIT_US               10000	/* 10ms */
#define D_TUNER_NRST_WAIT_RANGE_US         10100
#define D_TUNER_WAIT_1SEG_MS               10

#define D_TUNER_CONFIG_PLATFORM_DRIVER_NAME "mmtuner_pdev"
#define D_TUNER_CONFIG_SYSFS_DEV_NAME       "mmtuner_pdev"
#define D_TUNER_CONFIG_MATCH_TABLE          "socionext,mn88553"

/*
 * spi buffer size
 *  tuner max buffer : 695
 *  Data UnReady Thresh(lower limit) : 1/2
 *  TS packet size : 188
 */
#define D_TUNER_SPI_BUFFER_SIZE            (((695/2) & 0xffc) * 188)

enum gpio_id {
	TUNER_POWER_PIN = 0,
	TUNER_RESET_PIN,
	TUNER_INT_PIN,
};

static char const * const gpio_rsrcs[] = {
	"ISDB-T tuner power",
	"ISDB-T tuner reset",
	"ISDB-T tuner int",
};

enum TUNER_DRV_CTL {
	TUNER_DRV_CTL_POWON,
	TUNER_DRV_CTL_POWOFF
};

enum TUNER_DTV_SIG_LEVEL {
	TUNER_DRV_SIG_L,
	TUNER_DRV_SIG_H
};

struct tuner_drvdata {
	struct device *dev;
	struct device sysfs_dev;
	struct mutex mutex_lock;
	unsigned int gpios[ARRAY_SIZE(gpio_rsrcs)];
};

struct g_tuner_device {
	struct mutex g_tuner_mutex;
	unsigned int gpios[ARRAY_SIZE(gpio_rsrcs)];
};
static struct g_tuner_device tnr_dev;

static enum gpio_id req_ids[] = {
	TUNER_POWER_PIN,
	TUNER_RESET_PIN,
	TUNER_INT_PIN,
};

/******************************************************************************
 * variables
 ******************************************************************************/

/******************************************************************************
 * data
 ******************************************************************************/
static struct _mmtuner_cntxt g_cnt;	/* mm_tuner driver context */

#if defined(DPATH_SPI) || defined(DPATH_SDIO) || defined(DPATH_GPIF)
static struct _tsif_cntxt	g_tscnt;	/* TS I/F context */
#endif

/** @brief TS packet size.
 *
 * Array index of this constant array is
 * equivalent to ::_ts_pkt_type (enumeration type).
 */
static const size_t g_ts_pkt_size[3] = { 188, 204, 192 };

static const struct _mmtuner_version mmtuner_version = {
		MMTUNER_DEVICE,
		MMTUNER_MAJOR, MMTUNER_MINOR, MMTUNER_HOTFIX,
		MMTUNER_RC, MMTUNER_DESC
};

enum _tuner_cpathid cpath_id = DVR_MASTER;
int TUNER_CONFIG_INT;

/******************************************************************************
 * function
 ******************************************************************************/
static ssize_t tuner_module_entry_read(struct file *FIle, char __user *Buffer,
		size_t Count, loff_t *OffsetPosition);
static ssize_t tuner_module_entry_write(struct file *FIle,
		const char __user *Buffer, size_t Count,
		loff_t *OffsetPosition);
#ifdef TUNER_CONFIG_IRQ_ENABLE
static unsigned int tuner_module_entry_poll(struct file *file,
		struct poll_table_struct *poll_tbl);
#endif

static long tuner_module_entry_ioctl(struct file *file,
		unsigned int uCommand,
		unsigned long uArgument);

static int tuner_module_entry_open(struct inode *Inode, struct file *FIle);
static int tuner_module_entry_close(struct inode *Inode, struct file *FIle);
static int tuner_probe(struct platform_device *pdev);
static int __exit tuner_remove(struct platform_device *pdev);
static int tuner_pm_suspend(struct device *dev);
static int tuner_pm_resume(struct device *dev);
static int __init tuner_drv_start(void);
static void __exit tuner_drv_end(void);
static void tuner_device_release(struct device *dev);
#ifdef TUNER_CONFIG_IRQ_ENABLE
static int tuner_drv_irq_th(void *arc);
#endif
static int tuner_drv_read_regs(union _tuner_data_rw *rw, int num);
static int tuner_drv_write_regs(union _tuner_data_rw *rw, int num);

#if defined(DPATH_SPI) || defined(DPATH_SDIO) || defined(DPATH_GPIF)
static int tuner_drv_tsif_set_cntxt(struct _tsif_cntxt *tc);
static inline int  tuner_drv_bwseg(enum _bw_seg *pbw);
static int tuner_drv_tsif_start(void);
static int tuner_drv_tsif_stop(void);
static int tuner_drv_tsif_pktsize(void);
static int tuner_drv_tsif_blksize(void);
static int tuner_drv_tsif_th(void *arg);

static void tsread_timer_handler(struct timer_list *t);
static struct timer_list tsread_timer;
static unsigned long tsread_timer_data;
#endif

/* entry point */
static const struct file_operations TunerFileOperations = {
		.owner = THIS_MODULE,
		.read = tuner_module_entry_read,
		.write = tuner_module_entry_write,
#ifdef TUNER_CONFIG_IRQ_ENABLE
		.poll = tuner_module_entry_poll,
#endif
		.unlocked_ioctl = tuner_module_entry_ioctl,
		.open = tuner_module_entry_open,
		.release = tuner_module_entry_close
};

static const struct dev_pm_ops mmtuner_driver_pm_ops = {
		.suspend = tuner_pm_suspend,
		.resume = tuner_pm_resume,
};

static const struct of_device_id mn88553_match_table[] = {
{	.compatible = D_TUNER_CONFIG_MATCH_TABLE,
},
{}
};

static struct platform_driver mmtuner_driver = {
		.probe = tuner_probe,
		.remove = __exit_p(tuner_remove),
		.driver = {
			.name = D_TUNER_CONFIG_PLATFORM_DRIVER_NAME,
			.owner = THIS_MODULE,
			.of_match_table = of_match_ptr(mn88553_match_table),
		}
};

static struct platform_device *mmtuner_device;
static struct class *device_class;

/******************************************************************************
 * code area
 ******************************************************************************/

static void dtv_tunerpm_power_control(struct tuner_drvdata *drvdata, int on)
{
	gpio_set_value_cansleep(drvdata->gpios[TUNER_POWER_PIN], on);
}

static void dtv_tuner_reset_control(struct g_tuner_device *a_tnr_dev,
			int on_off)
{
	gpio_set_value_cansleep(a_tnr_dev->gpios[TUNER_RESET_PIN], on_off);
}

static int tuner_drv_ctl_reset(struct g_tuner_device *a_tnr_dev, int data)
{
	int ret = 0;

	pr_debug("%s,sig=%d\n", __func__, data);

	switch (data) {
	/* reset */
	case TUNER_DRV_SIG_L:
		mutex_lock(&a_tnr_dev->g_tuner_mutex);
		dtv_tuner_reset_control(a_tnr_dev, 0);
		mutex_unlock(&a_tnr_dev->g_tuner_mutex);
		usleep_range(D_TUNER_NRST_WAIT_US,
			D_TUNER_NRST_WAIT_RANGE_US);
		break;
	/* reset off*/
	case TUNER_DRV_SIG_H:
		mutex_lock(&a_tnr_dev->g_tuner_mutex);
		dtv_tuner_reset_control(a_tnr_dev, 1);
		mutex_unlock(&a_tnr_dev->g_tuner_mutex);
		usleep_range(D_TUNER_NRST_WAIT_US,
			D_TUNER_NRST_WAIT_RANGE_US);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static int tuner_drv_ctl_power(struct tuner_drvdata *drvdata, int data)
{
	int ret = 0;

	pr_debug("%s,data=%d\n", __func__, data);

	switch (data) {
	case TUNER_DRV_CTL_POWON:
		mutex_lock(&drvdata->mutex_lock);
		dtv_tunerpm_power_control(drvdata, 1);
		mutex_unlock(&drvdata->mutex_lock);
#ifdef CONFIG_CONTROL_NPDLOG
		/* if npdlog is used, waiting is necessary. */
		usleep_range(D_TUNER_POWER_ON_WAIT_US,
			D_TUNER_POWER_ON_WAIT_RANGE_US);
#endif
		break;

	case TUNER_DRV_CTL_POWOFF:
		mutex_lock(&drvdata->mutex_lock);
		dtv_tunerpm_power_control(drvdata, 0);
		mutex_unlock(&drvdata->mutex_lock);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static int tunerpm_dev_init(struct platform_device *pdev,
	struct tuner_drvdata *drvdata)
{
	int i, ret, gpio;
	unsigned int flags;
	struct device_node *of_node = pdev->dev.of_node;

	mutex_init(&drvdata->mutex_lock);

	for (i = 0; i < ARRAY_SIZE(gpio_rsrcs); i++) {
		gpio = of_get_gpio_flags(of_node, i, &flags);

		pr_debug("%s,%d\n", __func__, gpio);

		if (!gpio_is_valid(gpio)) {
			if (gpio < 0) {
				ret = gpio;
			} else {
				ret = -EINVAL;
			}
			goto error_gpio;
		}
		drvdata->gpios[i] = gpio;
		tnr_dev.gpios[i] = gpio;
	}

	for (i = 0; i < ARRAY_SIZE(req_ids); i++) {

		ret = gpio_request(drvdata->gpios[req_ids[i]],
				gpio_rsrcs[req_ids[i]]);

		if (ret)
			goto error_gpio_request;
	}

	TUNER_CONFIG_INT = gpio_to_irq(drvdata->gpios[TUNER_INT_PIN]);
	tuner_drv_ctl_reset(&tnr_dev, (int)TUNER_DRV_SIG_L);
	tuner_drv_ctl_power(drvdata, TUNER_DRV_CTL_POWOFF);

	return 0;

error_gpio_request:
	for (i--; i >= 0; i--)
		gpio_free(drvdata->gpios[req_ids[i]]);
error_gpio:
	return ret;
}

static ssize_t tuner_module_power_ctrl(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct tuner_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned long value;
	int ret;

	pr_debug("%s\n", __func__);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	if (!value) {
		ret = tuner_drv_ctl_power(drvdata, TUNER_DRV_CTL_POWON);
		if (ret)
			return -EINVAL;
	} else {
		tuner_drv_ctl_power(drvdata, TUNER_DRV_CTL_POWOFF);
	}

	return count;
}

static struct device_attribute tuner_sysfs_attrs[] = {
	__ATTR(power_ctrl, S_IWUSR, 0, tuner_module_power_ctrl),
};

static void tuner_device_release(struct device *dev)
{
	pr_debug("%s\n", __func__);

	unregister_chrdev(TUNER_CONFIG_DRV_MAJOR,
		TUNER_CONFIG_DRIVER_NAME);
}

/**************************************************************************//**
 * probe control of a driver
 *
 * @retval		0		normal
 * @retval		<0		error
 *
 * @param [in] pdev	pointer to the structure "platform_device"
 ******************************************************************************/
static int tuner_probe(struct platform_device *pdev)
{
	int ret = 0, i, retval = 0;
	struct tuner_drvdata *drvdata;
	struct device *dev = NULL;

	pr_debug("%s-S,%p\n", __func__, pdev);

	/* memory allocation */
	mmtuner_device = platform_device_alloc(TUNER_CONFIG_DRIVER_NAME, -1);

	if (!mmtuner_device) {
		pr_err("platform_device_alloc() failed.\n");
		return -ENOMEM;
	}

	/* add device */
	ret = platform_device_add(mmtuner_device);
	if (ret) {
		pr_err("platform_device_add() failed.\n");
		retval = ret;
		goto err_platform_device_add;
	}

	/* create the node of device */
	device_class = class_create(THIS_MODULE, TUNER_CONFIG_DRIVER_NAME);
	if (IS_ERR(device_class)) {
		pr_err("class_create() failed.\n");
		retval = PTR_ERR(device_class);
		goto err_class_create;
	}

	/* create the logical device */
	dev = device_create(device_class, NULL,
		MKDEV(TUNER_CONFIG_DRV_MAJOR, TUNER_CONFIG_DRV_MINOR), NULL,
		TUNER_CONFIG_DRIVER_NAME);
	if (IS_ERR(dev)) {
		pr_err("device_create() failed.\n");
		retval = PTR_ERR(dev);
		goto err_device_create;
	}

	drvdata = kzalloc(sizeof(struct tuner_drvdata), GFP_KERNEL);
	if (!drvdata) {
		retval = -ENOMEM;
		goto err_alloc_data;
	}

	drvdata->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, drvdata);
	drvdata->sysfs_dev.init_name = D_TUNER_CONFIG_SYSFS_DEV_NAME;
	drvdata->sysfs_dev.release = tuner_device_release;
	dev_set_drvdata(&drvdata->sysfs_dev, drvdata);

	ret = device_register(&drvdata->sysfs_dev);
	if (ret) {
		retval = -EINVAL;
		goto err_set_dev;
	}

	/* register the driver */
	if (register_chrdev(TUNER_CONFIG_DRV_MAJOR, TUNER_CONFIG_DRIVER_NAME,
			&TunerFileOperations)) {
		pr_err("register_chrdev() failed (Major:%d).\n",
				TUNER_CONFIG_DRV_MAJOR);
		retval = -EINVAL;
		goto err_register_device;
	}

	mutex_init(&tnr_dev.g_tuner_mutex);
	ret = tunerpm_dev_init(pdev, drvdata);
	if (ret) {
		pr_err("tunerpm_dev_init():[%d] failed.\n", ret);
		retval = ret;
		goto out;
	}

	for (i = 0; i < ARRAY_SIZE(tuner_sysfs_attrs); i++) {
		ret = device_create_file(&drvdata->sysfs_dev,
			&tuner_sysfs_attrs[i]);
		if (ret) {
			for (; i >= 0; --i)
				device_remove_file(&drvdata->sysfs_dev,
					&tuner_sysfs_attrs[i]);
				retval = -EINVAL;
				goto out;
		}
	}

#if defined(DPATH_SPI) || defined(DPATH_SDIO) || defined(DPATH_GPIF)
	/* register the TS I/F driver */
	ret = tuner_drv_hw_tsif_register();
	if (ret) {
		pr_err("tuner_drv_hw_tsif_register() failed.\n");
		retval = ret;
		goto out;
	}
#endif

	/* dispatch the kernel thread to handle the interrupt (IRQ) */
#ifdef TUNER_CONFIG_IRQ_ENABLE
	g_cnt.irqth_flag = TUNER_IRQTH_NONE;
	init_waitqueue_head(&g_cnt.irqth_waitq);
	g_cnt.irqth_id = kthread_create(tuner_drv_irq_th, NULL,
					"tuner_drv_irq_th");
	if (IS_ERR(g_cnt.irqth_id)) {
		retval = PTR_ERR(g_cnt.irqth_id);
		g_cnt.irqth_id = NULL;
		goto out;
	}

	init_waitqueue_head(&g_cnt.poll_waitq);
	spin_lock_init(&g_cnt.poll_lock);
	g_cnt.poll_flag = 0x00;

	g_cnt.ev.pack = 0;
	g_cnt.opcnt = 0;

	wake_up_process(g_cnt.irqth_id);
#endif

#if defined(DPATH_SPI) || defined(DPATH_SDIO) || defined(DPATH_GPIF)
	/* dispatch a kernel thread to receive TS data */
	g_tscnt.tsifth_wait = true;
	g_tscnt.tsifth_flag = TUNER_TSIFTH_NONE;
	init_waitqueue_head(&g_tscnt.tsifth_waitq);
	g_tscnt.tsifth_id = kthread_create(tuner_drv_tsif_th, NULL,
		"tuner_drv_tsif_th");
	if (IS_ERR(g_tscnt.tsifth_id)) {
		pr_err("kthread_create() failed.\n");
		retval = PTR_ERR(g_tscnt.tsifth_id);
		g_tscnt.tsifth_id = NULL;
		goto out;
	}

	pr_debug("%s, kthread_create(tuner_drv_tsif_th)-after debug\n",
		__func__);

	/* spi buffer allocate */
	g_tscnt.spibuf = NULL;
	pr_debug("%s, spi buffer allocate size[%d]\n",
		__func__, D_TUNER_SPI_BUFFER_SIZE);
	g_tscnt.spibuf = kmalloc(D_TUNER_SPI_BUFFER_SIZE, GFP_KERNEL);
	if (g_tscnt.spibuf == NULL) {
		pr_err("spi buffer allocation failed.\n");
	}
	pr_debug("spi buffer pointer[probe]: %p\n", g_tscnt.spibuf);
	g_tscnt.pktbuf = NULL;
	g_tscnt.pwr = g_tscnt.prd = 0;
	g_tscnt.ovf = 0;

	wake_up_process(g_tscnt.tsifth_id);

	/* initialize the status flag and
	 *the wait-queue for Ts Buffering Thread
	*/
	g_tscnt.tsread_flag = TUNER_TSREAD_IDLE;
	init_waitqueue_head(&g_tscnt.tsread_waitq);
#endif

	pr_debug("%s-E\n", __func__);
	return 0;

out:
err_register_device:
	device_unregister(&drvdata->sysfs_dev);
err_set_dev:
	kzfree(drvdata);
err_alloc_data:
	device_destroy(device_class, MKDEV(TUNER_CONFIG_DRV_MAJOR,
		TUNER_CONFIG_DRV_MINOR));
err_device_create:
	class_destroy(device_class);
err_class_create:
	platform_device_del(mmtuner_device);
err_platform_device_add:
	platform_device_put(mmtuner_device);

	pr_debug("%s-Err(%d)\n", __func__, retval);
	return retval;
}

/**************************************************************************//**
 * remove control of a driver
 *
 * @retval		0	normal
 * @retval		<0	error
 *
 * @param [in] pdev	pointer to the structure "platform_device"
 ******************************************************************************/
static int __exit tuner_remove(struct platform_device *pdev)
{
	pr_debug("%s\n", __func__);

	/* release interrupt (IRQ) */
	tuner_drv_hw_freeirq();

	/* un-register driver */
	unregister_chrdev(TUNER_CONFIG_DRV_MAJOR, TUNER_CONFIG_DRIVER_NAME);

	return 0;
}

/*************************************************************************//***
 * suspend control of a driver
 *
 * @retval		0	normal
 * @retval		<0	error
 *
 * @param [in] dev	device
 ******************************************************************************/
static int tuner_pm_suspend(struct device *dev)
{
	pr_debug("%s\n", __func__);

	return 0;
}

/*************************************************************************//***
 * resume control of a driver
 *
 * @retval		0	normal
 * @retval		<0	error
 *
 * @param [in] dev	device
 ******************************************************************************/
static int tuner_pm_resume(struct device *dev)
{
	int ret = 0;

	pr_debug("%s\n", __func__);

#if defined(DPATH_SPI) || defined(DPATH_SDIO) || defined(DPATH_GPIF)
	ret = tuner_drv_hw_tsif_set_tpm();
	if (ret) {
		pr_err("write TPM bits failed.\n");
		return ret;
	}
#endif
	return ret;
}

/**************************************************************************//**
 * Interrupt handling thread function
 *
 * @retval		0	normal
 * @retval		<0	error
 *
 * @param [in] arg	(no effect)
 ******************************************************************************/
#ifdef TUNER_CONFIG_IRQ_ENABLE
static int tuner_drv_irq_th(void *arg)
{
	int ret = 0;
	unsigned long flags;
	unsigned long kthread_flg;

	mm_segment_t oldfs;
	struct sched_param param;

	uint8_t buf[2] = { 0x00, 0x00 };

	pr_debug("%s\n", __func__);

	/* initialize inner values */
	ret = 0;
	flags = 0;
	kthread_flg = 0;
	param.sched_priority = TUNER_CONFIG_KTH_PRI;

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	ret = sched_setscheduler(g_cnt.irqth_id, SCHED_FIFO, &param);
	set_fs(oldfs);

	while (1) {
		buf[0] = buf[1] = 0;

		pr_debug("%s waiting...\n", __func__);
		wait_event_interruptible(g_cnt.irqth_waitq, g_cnt.irqth_flag);

		spin_lock_irqsave(&g_cnt.poll_lock, flags);
		kthread_flg = g_cnt.irqth_flag;
		g_cnt.irqth_flag &= ~TUNER_IRQKTH_CATCHIRQ;
		spin_unlock_irqrestore(&g_cnt.poll_lock, flags);

		if ((kthread_flg & TUNER_IRQKTH_CATCHIRQ) ==
			TUNER_IRQKTH_CATCHIRQ) {
			/* receive IRQ */
			pr_debug("%s IRQHANDLER start\n", __func__);

			/* Read  INTCND, INTST */
			ret = tuner_drv_hw_read_reg(
					Main1,
					0xE3,
					2,
					buf);
			if (ret) {
				pr_err("Read IRQ factor/state, failed.\n");
				kthread_flg |= TUNER_KTH_END;
			}
			/* write (back) INTCND to clear
			 * the factor (register) of interrupt (IRQ)
			*/
			ret = tuner_drv_hw_write_reg(
					Main1,
					0xE3,
					1,
					buf);
			if (ret) {
				pr_err("Clear IRQ factor, failed.");
				kthread_flg |= TUNER_KTH_END;
			}

			g_cnt.ev.get.intcnd |= buf[0];
			g_cnt.ev.get.intst |= buf[1];
			g_cnt.ev.get.irqnum++;

			pr_debug("IRQ factor update (%d): INTCND:0x%02x INTST:0x%02x\n",
				g_cnt.ev.get.irqnum, g_cnt.ev.get.intcnd,
				g_cnt.ev.get.intst);

			/* release poll/select */
			g_cnt.poll_flag = 0x01;
			wake_up(&g_cnt.poll_waitq);

			pr_debug("%s main loop go to next turn.\n", __func__);

#ifdef TUNER_CONFIG_IRQ_LEVELTRIGGER
			/* activate (re-activate) the interrupt,
			 *if the level-interrupt is active
			*/
			tuner_drv_hw_enable_interrupt();
#endif /* TUNER_CONFIG_IRQ_LEVELTRIGGER */
		}

		/* request to finish the interrupt kernel thread */
		if ((kthread_flg & TUNER_KTH_END) == TUNER_KTH_END) {
			pr_debug("%s caught a stop request.\n", __func__);

			spin_lock_irqsave(&g_cnt.poll_lock, flags);
			g_cnt.irqth_flag &= ~TUNER_KTH_END;
			spin_unlock_irqrestore(&g_cnt.poll_lock, flags);

			break;
		}
	}

	pr_debug("Tail of thread function %s.\n", __func__);
	return 0;
}
#endif

/**************************************************************************//**
 * initialization control of a driver
 *
 * @retval		0	normal
 * @retval		<0	error
 ******************************************************************************/
static int __init tuner_drv_start(void)
{
	int ret = 0;

	pr_debug("%s-S\n", __func__);

	pr_debug("mmtuner (for MN885%02x) ver. %d.%d.%d %s (%s)\n",
		mmtuner_version.device, mmtuner_version.major,
		mmtuner_version.minor, mmtuner_version.hotfix,
		mmtuner_version.rc, mmtuner_version.describe);

	/* register "mmtuner" driver */
	ret = platform_driver_register(&mmtuner_driver);
	if (ret != 0) {
		pr_err("platform_driver_register failed.\n");
		return ret;
	}

	pr_debug("%s-E\n", __func__);

	return 0;	/* normal */
}

/**************************************************************************//**
 * exit control of a driver
 *
 ******************************************************************************/
static void __exit tuner_drv_end(void)
{
	pr_debug("%s\n", __func__);

#ifdef TUNER_CONFIG_IRQ_ENABLE
	g_cnt.irqth_flag |= TUNER_KTH_END;
	/* exit the kernel thread (for IRQ) */
	if (waitqueue_active(&g_cnt.irqth_waitq))
		wake_up(&g_cnt.irqth_waitq);

	/* unregister the IRQ kernel thread */
	if (g_cnt.irqth_id)
		kthread_stop(g_cnt.irqth_id);
#endif

#if defined(DPATH_SPI) || defined(DPATH_SDIO) || defined(DPATH_GPIF)
	g_tscnt.tsifth_flag = TUNER_TSIFTH_END;
	/* exit the kernel thread (for TS) */
	if (waitqueue_active(&g_tscnt.tsifth_waitq))
		wake_up_interruptible(&g_tscnt.tsifth_waitq);

	g_tscnt.tsifth_wait = true;

	/* unregister the TS kernel thread */
	if (g_tscnt.tsifth_id)
		kthread_stop(g_tscnt.tsifth_id);

	/* execute the unregister scheme of TS I/F */
	tuner_drv_hw_tsif_unregister();
#endif

	/* Destroy device */
	device_destroy(device_class,
			MKDEV(TUNER_CONFIG_DRV_MAJOR, TUNER_CONFIG_DRV_MINOR));
	/* delete a entry of class */
	class_destroy(device_class);
	/* unregister the driver */
	platform_device_unregister(mmtuner_device);
	/* unregister the platform entry */
	platform_driver_unregister(&mmtuner_driver);

}

/**************************************************************************//**
 * open control of a driver
 *
 * @retval		0	normal
 * @retval		<0	error
 *
 * @param [in] Inode, FIle
 ******************************************************************************/
static int tuner_module_entry_open(struct inode *Inode, struct file *FIle)
{
	pr_debug("%s\n", __func__);

#ifndef TUNER_CONFIG_DRV_MULTI
	if (g_cnt.opcnt > 0) {
		pr_err("It's being used.\n");
		return -EBUSY;
	}
#endif /* TUNER_CONFIG_DRV_MULTI */

	g_cnt.opcnt++;
	pr_debug("user count: %d\n", g_cnt.opcnt);

	return 0;
}

/**************************************************************************//**
 * close control of a driver
 *
 * @retval		0	normal
 * @retval		<0	error
 *
 * @param [in] Inode, FIle	regular argument for linux system call
 ******************************************************************************/
static int tuner_module_entry_close(struct inode *Inode, struct file *FIle)
{
	struct devone_data *dev;

	pr_debug("%s\n", __func__);

	if (g_cnt.opcnt == 0) {	/* not open */
		pr_err("NOT IN USE.\n");
		return -ENODEV;
	}
	g_cnt.opcnt--;

	if (g_cnt.opcnt == 0) { /* All logical device is closed. */
		/* free IRQ */
		tuner_drv_hw_freeirq();
		if (FIle == NULL)
			return -EINVAL;
		dev = FIle->private_data;
		kfree(dev);
	}

	return 0;
}

/**************************************************************************//**
 * @brief "read" system call of mm_tuner device
 *
 * The "read" system-call acquires the TS data stream of
 * the designated byte size.
 *
 * @retval >=0		normal (byte size)
 * @retval <0			error
 ******************************************************************************/
static ssize_t tuner_module_entry_read(struct file *FIle, char __user *Buffer,
		size_t Count, loff_t *OffsetPosition)
{
#if defined(DPATH_SPI) || defined(DPATH_SDIO) || defined(DPATH_GPIF)
	ssize_t size = 0;
	int ret;
	int copy_size;

	if (!Buffer) {
		pr_err("arg. \"Buffer\" is Null\n");
		return -EINVAL;
	}
	if (Count <= 0) {
		pr_err("arg. \"Count\" is illegal (%zu)\n", Count);
		return -EINVAL;
	}

	if (g_tscnt.tsifth_wait) {
		pr_warn("TS I/F thread is NOT active (waiting...)\n");
		return -EAGAIN;
	}
	if (g_tscnt.tsread_flag != TUNER_TSREAD_IDLE) {
		pr_warn("read (TS) system call is executed in duplicate flag:%d\n",
			g_tscnt.tsread_flag);
		return -EBUSY;
	}
	g_tscnt.tsread_flag = TUNER_TSREAD_ACTIVE;

	/* clear the over-flow flag of the packet buffer */
	g_tscnt.ovf = 0;

	do {
		int able_size = 0;

		/* TS FIFO is empty */
		if (g_tscnt.prd == g_tscnt.pwr) {
			/* TS-Read timer */
			timer_setup(&tsread_timer, tsread_timer_handler, 0);
			tsread_timer.expires = jiffies +
				msecs_to_jiffies(TUNER_CONFIG_TSREAD_TIMEOUT);
			tsread_timer_data = (unsigned long)jiffies;
			add_timer(&tsread_timer);

			g_tscnt.tsread_flag = TUNER_TSREAD_WAIT;
			usleep_range(100, 500);
			wait_event_interruptible(g_tscnt.tsread_waitq,
				g_tscnt.tsread_flag);

			del_timer(&tsread_timer);
			if (g_tscnt.tsread_flag != TUNER_TSREAD_IDLE) {
				pr_warn("Stop reading TS data.\n");
				pr_debug("TS-Read status flag: 0x%08x.\n",
					g_tscnt.tsread_flag);
				break;
			}
			g_tscnt.tsread_flag = TUNER_TSREAD_ACTIVE;
			continue;
		}

		/* readable data size of TS FIFO */
		able_size = (g_tscnt.pwr > g_tscnt.prd) ?
				(g_tscnt.pwr - g_tscnt.prd) :
				(g_tscnt.ts_pktbuf_size - g_tscnt.prd);

		/* decide copy_size */
		copy_size = ((size + able_size) > Count) ?
				(Count - size) :
				able_size;
		ret = copy_to_user(
				Buffer + size,
				g_tscnt.pktbuf + g_tscnt.prd,
				copy_size);
		if (ret) {
			pr_err("copy_to_user() failed.\n");
			return -EFAULT;
		}

		/* increment read position */
		g_tscnt.prd += copy_size;
		if (g_tscnt.prd == g_tscnt.ts_pktbuf_size)
			g_tscnt.prd = 0;

		/* increment total copied size */
		size += copy_size;

	} while (size != Count);

	g_tscnt.tsread_flag  = TUNER_TSREAD_IDLE;

	return size;
#else
	pr_warn("NOT support the TS Slave I/F of the tuner device.\n");
	return -EIO;
#endif
}

/**************************************************************************//**
 * write control of a driver
 *
 * @caution	The previous version of mm_tuner driver have "write"
 *			system call to write registers continuously.
 *			But, ioctl(TUNER_IOCTL_CNTSET) is implemented in this
 *			version.
 *
 * @retval		0			normal
 * @retval		-ENOSYS	(not implemented)
 *
 * @param [in] FIle, Buffer, Count, OffsetPosition
 *		These are regular argument of the system call "write"
 ******************************************************************************/
static ssize_t tuner_module_entry_write(struct file *FIle,
		const char __user *Buffer, size_t Count, loff_t *OffsetPosition)
{
	pr_warn("The \"write\" system-call is not supported.\n");
	return -EIO;
}

/**************************************************************************//**
 * ioctl system call
 *
 * @retval 0			normal
 * @retval <0			error
 ******************************************************************************/
static long tuner_module_entry_ioctl(struct file *file,
		unsigned int uCommand, unsigned long uArgument)
{
	int ret = 0;
	int retval = 0;
	union _tuner_data_rw rw;
	union _tuner_data_event ev;
	uint8_t *buf;
	uint32_t ver;
	uint8_t data;
#if defined(CPATH_I2C)
	enum _tuner_cpathid cid;
#endif
	unsigned int sig = 0;

	pr_debug("[IOCTL(uCommand):   %x]\n", uCommand);

	switch (uCommand) {
	/* get a parameter of the register of Tuner */
	case TUNER_IOCTL_VALGET:
		if (copy_from_user(&rw,
			(union _tuner_data_rw __user *)uArgument,
			sizeof(union _tuner_data_rw))) {
			pr_err("copy_from_user() failed.\n");
			return -EFAULT;
		}
		ret = tuner_drv_read_regs(&rw, 1);
		if (!ret) {
			if (copy_to_user(
				(union _tuner_data_rw __user *)uArgument, &rw,
				sizeof(union _tuner_data_rw))) {
				pr_err("copy_to_user() failed.\n");
				return -EFAULT;
			}
		} else {
			pr_err("read a register, failed.\n");
			return ret;
		}
		return 0;
	/* write a parameters to the register of Tuner */
	case TUNER_IOCTL_VALSET:
		if (copy_from_user(&rw,
			(union _tuner_data_rw __user *)uArgument,
			sizeof(union _tuner_data_rw))) {
			pr_err("copy_from_user() failed.\n");
			return -EFAULT;
		}
		ret = tuner_drv_write_regs(&rw, 1);
		if (ret) {
			pr_err("write a register, failed.\n");
			return ret;
		}
		return 0;
	/* write registers continuously */
	case TUNER_IOCTL_CNTSET:
		if (copy_from_user(&rw,
			(union _tuner_data_rw __user *)uArgument,
			sizeof(union _tuner_data_rw))) {
			pr_err("copy_from_user() failed.\n");
			return -EFAULT;
		}
		buf = kmalloc(rw.cont.len, GFP_KERNEL);
		if (!buf) {
			pr_err("memory allocation failed(CNTSET).\n");
			return -ENOMEM;
		}
		if (copy_from_user(buf, (void __user *)rw.cont.buf,
			rw.cont.len)) {
			pr_err("copy_from_user() failed.\n");
			retval = -EFAULT;
		} else {
#if defined(CPATH_SPI) || defined(CPATH_SDIO) || defined(CPATH_GPIF)
			if ((rw.cont.adr == 0xF6) && ((rw.cont.bank == 0) ||
				(rw.cont.bank == 2))) {
				ret = tuner_drv_hw_write_prg(rw.cont.bank,
					rw.cont.adr, rw.cont.len, buf);
				if (ret) {
					pr_err("write program continuously, failed.\n");
					retval = ret;
				}
			} else {
#endif
				ret = tuner_drv_hw_write_reg((int)rw.cont.bank +
					(int)cpath_id, rw.cont.adr,
					rw.cont.len, buf);
				if (ret) {
					pr_err("write continuously, failed.\n");
					retval = ret;
				}
#if defined(CPATH_SPI) || defined(CPATH_SDIO) || defined(CPATH_GPIF)
			}
#endif
		}
		kfree(buf);
		buf = NULL;
		return retval;
	/* read registers continuously */
	case TUNER_IOCTL_CNTGET:
		if (copy_from_user(&rw,
			(union _tuner_data_rw __user *)uArgument,
			sizeof(union _tuner_data_rw))) {
			pr_err("copy_from_user() failed.\n");
			return -EFAULT;
		}
		buf = kmalloc(rw.cont.len, GFP_KERNEL);
		if (!buf) {
			pr_err("memory allocation failed(CNTGET).\n");
			return -ENOMEM;
		}
		ret = tuner_drv_hw_read_reg((int)rw.cont.bank +
			(int)cpath_id, rw.cont.adr, rw.cont.len, buf);
		if (ret) {
			pr_err("read continuously, failed.\n");
			retval = ret;
		} else {
			if (copy_to_user((void __user *)rw.cont.buf,
				buf, rw.cont.len)) {
				pr_err("copy_to_user() failed.\n");
				retval = -EFAULT;
			}
		}
		kfree(buf);
		buf = NULL;
		return retval;
	/* get the interrupt factor and status */
	case TUNER_IOCTL_EVENT_GET:
		if (copy_to_user((union _tuner_data_event __user *)uArgument,
			&g_cnt.ev,
			sizeof(union _tuner_data_event))) {
			pr_err("copy_to_user() failed.\n");
			return -EFAULT;
		}
		pr_debug("IRQ factor send: intset1:0x%02x intcnd:0x%02x intst:0x%02x\n",
		g_cnt.ev.get.intset1, g_cnt.ev.get.intcnd, g_cnt.ev.get.intst);

		/* initialize the variables for the interrupt information */
		g_cnt.ev.pack = 0;
		return 0;
	/* be available some interrupts */
	case TUNER_IOCTL_EVENT_SET:
		pr_debug("*** VALSET_EVENT ***\n");
		if (copy_from_user(&ev,
			(union _tuner_data_event __user *)uArgument,
			sizeof(union _tuner_data_event))) {
			pr_err("copy_from_user() failed.\n");
			return -EFAULT;
		}
		ret = tuner_drv_hw_setev(&ev);
		if (ret) {
			pr_err("tuner_drv_setev() failed.\n");
			return ret;
		}
		return 0;
	/* be disable some interrupts */
	case TUNER_IOCTL_EVENT_REL:
		if (copy_from_user(&ev,
			(union _tuner_data_event __user *)uArgument,
			sizeof(union _tuner_data_event))) {
			pr_err("copy_from_user() failed.\n");
			return -EFAULT;
		}
		ret = tuner_drv_hw_relev(&ev);
		if (ret) {
			pr_err("tuner_drv_relev() failed.\n");
			return ret;
		}
		return 0;

	/* start to receive TS data from Tuner */
	case TUNER_IOCTL_TSIF_START:
#if defined(DPATH_SPI) || defined(DPATH_SDIO) || defined(DPATH_GPIF)
		if (!g_tscnt.tsifth_wait) {
			pr_warn("TS data buffering had already been started.\n");
			return -EBUSY;
		}
		if (g_tscnt.tsif != NULL) {
			kfree(g_tscnt.tsif);
			g_tscnt.tsif = NULL;
		}
		g_tscnt.tsif =
		kmalloc(sizeof(struct _tuner_data_tsif), GFP_KERNEL);
		if (g_tscnt.tsif == NULL)
			return -ENOMEM;
		if (copy_from_user(g_tscnt.tsif,
			(struct _tuner_data_tsif __user *)uArgument,
			sizeof(struct _tuner_data_tsif))) {
			pr_err("copy_from_user() failed.\n");
			retval = -EFAULT;
		} else {
			ret = tuner_drv_tsif_start();
			if (ret) {
				pr_err("tuner_drv_tsif_start() failed.\n");
				retval = ret;
			} else {
				return 0;
			}
		}
		kfree(g_tscnt.tsif);
		g_tscnt.tsif = NULL;
		return retval;
#else
		return -EINVAL;
#endif
	/* stop to receive TS data from Tuner */
	case TUNER_IOCTL_TSIF_STOP:
#if defined(DPATH_SPI) || defined(DPATH_SDIO) || defined(DPATH_GPIF)
		return tuner_drv_tsif_stop();
#else
		return -EINVAL;
#endif
	/* return byte num of a TS packet */
	case TUNER_IOCTL_TSIF_PKTSIZE:
#if defined(DPATH_SPI) || defined(DPATH_SDIO) || defined(DPATH_GPIF)
		ret = tuner_drv_tsif_pktsize();
		if (ret < 0) {
			pr_err("tuner_drv_tsif_pktsize() failed.\n");
			return ret;
		}
		if (copy_to_user((unsigned int __user *)uArgument,
			&ret, sizeof(unsigned int))) {
			pr_err("copy_to_user() failed.\n");
			return -EFAULT;
		}
		return 0;
#else
		pr_info("NOT support the TS Slave I/F of the tuner device.\n");
		return -EINVAL;
#endif
	case TUNER_IOCTL_GETVER:
		ver =	(mmtuner_version.device << 24) |
				(mmtuner_version.major << 16) |
				(mmtuner_version.minor << 8) |
				mmtuner_version.hotfix;
		put_user(ver, (unsigned int __user *)uArgument);

		ret = tuner_drv_hw_read_reg(Sub, 0xFF, 1, &data);
		if (ret) {
			pr_err("Read CHIPID, failed.\n");
			return -EFAULT;
		}
		if (data != MMTUNER_DEVICE) {
			pr_err("Unexpected CHIPRD (0x%02x).\n", data);
			return -ENXIO;
		}
		return 0;
	case TUNER_IOCTL_TSIF_BLKSIZE:
#if defined(DPATH_SPI) || defined(DPATH_SDIO) || defined(DPATH_GPIF)
		ret = tuner_drv_tsif_blksize();
		if (ret < 0) {
			pr_err("tuner_drv_tsif_blksize() failed.\n");
			return ret;
		}
		if (copy_to_user((int __user *)uArgument, &ret, sizeof(int))) {
			pr_err("copy_to_user() failed.\n");
			return -EFAULT;
		}
		return 0;
#else
		pr_info("NOT support the TS Slave I/F of the tuner device.\n");
		return -EINVAL;
#endif
	case TUNER_IOCTL_TSIF_GET_IV: {
		pr_info("NOT support the TS Slave I/F of the tuner device.\n");
		return -EINVAL;
	}
	case TUNER_IOCTL_TSIF_INIT_IV: {
		pr_info("NOT support the TS encryption mode\n");
		return -EINVAL;
	}
	case TUNER_IOCTL_CPATHID:
#ifdef CPATH_I2C
		if (copy_from_user(&cid,
			(enum _tuner_cpathid __user *)uArgument,
			sizeof(enum _tuner_cpathid))) {
			pr_err("copy_from_user() failed.\n");
			return -EFAULT;
		}
		ret = tuner_drv_hw_set_id(cid);
		if (ret) {
			pr_err("set id, failed.\n");
			retval = ret;
		}
		return retval;
#else
		return -EINVAL;
#endif
	/* write a SIG to the nrst of Tuner */
	case TUNER_IOCTL_NRST_CTL:

		if (copy_from_user(&sig, (unsigned int __user *)uArgument,
			sizeof(unsigned int))) {
			pr_err("copy_from_user() failed.\n");
			return -EFAULT;
		}

		ret = tuner_drv_ctl_reset(&tnr_dev, (int)sig);

		if (ret) {
			pr_err("write a nrst, failed.\n");
			return ret;
		}
		return 0;

	/* return byte num of a TS packet */
	default:
		pr_err("illegal ioctl request.\n");
		return -EINVAL;
	}
}

/**************************************************************************//**
 * poll control of a driver
 *
 * @retval		0	normal
 * @retval		<0	error
 ******************************************************************************/
#ifdef TUNER_CONFIG_IRQ_ENABLE
static unsigned int tuner_module_entry_poll(struct file *file,
		struct poll_table_struct *poll_tbl)
{
	unsigned long tuner_flags;
	unsigned int tuner_mask;

	/* initialize */
	tuner_mask = 0;

	/* wait */
	poll_wait(file, &g_cnt.poll_waitq, poll_tbl);

	/* disable the interrupt */
	spin_lock_irqsave(&g_cnt.poll_lock, tuner_flags);

	/* release */
	if (g_cnt.poll_flag == 0x01)
		tuner_mask = (POLLIN | POLLRDNORM);

	g_cnt.poll_flag = 0x00;

	/* enable the interrupt */
	spin_unlock_irqrestore(&g_cnt.poll_lock, tuner_flags);

	return tuner_mask;
}
#endif

/**************************************************************************//**
 * interrupt control of a driver
 *
 * @retval IRQ_NONE        interrupt was not from this device
 * @retval IRQ_HANDLED     interrupt was handled by this device
 * @retval IRQ_WAKE_THREADhandler requests to wake the handler
 *         thread
 *
 * @param [in] irq		irq#
 * @parma [in] dev_id	device ID
 ******************************************************************************/
#ifdef TUNER_CONFIG_IRQ_ENABLE
irqreturn_t tuner_interrupt(int irq, void *dev_id)
{
	pr_debug("%s\n", __func__);

	g_cnt.irqth_flag |= TUNER_IRQKTH_CATCHIRQ;
	/* The main scheme for IRQ is in the IRQ kernel thread.
	 * Here is the wake up operation, only.
	 */
	if (waitqueue_active(&g_cnt.irqth_waitq)) {
#ifdef TUNER_CONFIG_IRQ_LEVELTRIGGER
		/* disabling IRQ, when the level interrupt is avail. */
		tuner_drv_hw_disable_interrupt();
#endif /* TUNER_CONFIG_IRQ_LEVELTRIGGER */
		wake_up(&g_cnt.irqth_waitq);
	} else {
		pr_err("waitqueue_active() failed.\n");
		/* When the wake-up scheme don't work, stop the receiving IRQ */
	}

	return IRQ_HANDLED;
}
#endif

/**************************************************************************//**
 * @brief Read some registers.
 *
 * Repeat single read transaction.
 *
 * @param [in] arg	Address to a argument of ioctl() in unsigned long.
 * @num [in] num		Repeat count.
 *
 * @returns	0 on success.
 * @returns	Negative on error.
 ******************************************************************************/
static int tuner_drv_read_regs(union _tuner_data_rw *rw, int num)
{
	int i;
	int ret;

	for (i = 0; i < num; i++) {
		ret = tuner_drv_hw_read_reg(
				rw[i].sngl.bank + (int)cpath_id,
				rw[i].sngl.adr,
				1,
				&(rw[i].sngl.param));
		if (ret) {
			pr_err("copy_to_user() failed.\n");
			return ret;
		}
	}
	return 0;
}

/**************************************************************************//**
 * write some registers of the tuner device.
 *
 * (no description)
 *
 * @retval 0			normal
 * @retval -EINVAL	error
 ******************************************************************************/
static int tuner_drv_write_regs(union _tuner_data_rw *rw, int num)
{
	int i;
	int ret;

	for (i = 0; i < num; i++) {
		ret = tuner_drv_hw_rmw_reg(
				rw->sngl.bank + (int)cpath_id,
				rw->sngl.adr,
				rw->sngl.enabit,
				rw->sngl.param);
		if (ret)
			return ret;
	}

	return 0;
}

#if defined(DPATH_SPI) || defined(DPATH_SDIO) || defined(DPATH_GPIF)
/**************************************************************************//**
 * TS I/F (buffering) thread function
 *
 * @caption
 * Original is "tuner_tsbufferring_thread" implemented by
 *
 * @retval 0			Normal end
 * @retval <0			error (refer the errno)
 *
 * @param [in] arg	pointer to the TUNER_DATA_TSIF structure
 ******************************************************************************/
static int tuner_drv_tsif_th(void *arg)
{
	int ret = 0;
	int retval = 0;
	struct sched_param param;
	mm_segment_t oldfs;
	int dataready = 0;
	int fifo_err = 0;
	int maxwaitcnt = 8;
	int waitcnt;

	pr_debug("%s\n", __func__);

	/* set the priority of thread */
	param.sched_priority = TUNER_CONFIG_TSBTH_PRI;
	/* daemonize("mmtuner_tsifth"); */

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	ret = sched_setscheduler(g_tscnt.tsifth_id, SCHED_FIFO, &param);
	set_fs(oldfs);

	waitcnt = maxwaitcnt;

	/* thread main loop */
	while (1) {
		wait_event_interruptible(g_tscnt.tsifth_waitq,
			g_tscnt.tsifth_flag);

		/* TS buffering */
		if (g_tscnt.tsifth_flag & TUNER_TSIFTH_ACTIVE) {
			ret = tuner_drv_hw_tsif_get_dready();
			if (ret < 0) {
				pr_debug("get DATAREADY, failed.\n");
				retval = ret;
				goto out;
			} else {
				dataready = (uint8_t)ret;
			}
			if (dataready & 0x06) {	/* OVER/UNDER-Run */
				if (fifo_err++ >= TUNER_TSIFTH_FIFOERROR_MAX) {
					pr_err("FIFO is in serious status!!!\n");
					pr_warn("TS I/F thread go to waiting.\n");

					g_tscnt.tsifth_flag = TUNER_TSIFTH_NONE;
					g_tscnt.tsifth_wait = true;
					fifo_err = 0;
					continue;
				}
				pr_warn("OVER/UNER-Run(0x%02x) (%d).\n",
					dataready, fifo_err);
				dataready = 0x00;
				ret = tuner_drv_hw_tsif_sync_pkt();
				if (ret) {
					pr_err("tuner_drv_hw_tsif_sync_pkt() failed.\n");
					retval = ret;
					goto out;
				}
				waitcnt = maxwaitcnt;
			}
			if (!(dataready & 0x01)) { /* not DATAREADY */

				if (g_tscnt.bw == TUNER_DRV_BW1)
					msleep(D_TUNER_WAIT_1SEG_MS);
				else
					usleep_range(500, 1000);

				if (waitcnt > 0)
					waitcnt--;
				continue;
			}
			waitcnt = maxwaitcnt;

			/* TS buffering */
			fifo_err = 0;
			ret = tuner_drv_hw_tsif_get_pkts(&g_tscnt);
			if (ret < 0) {
				pr_err("Receive/Store TS data, failed\n");
				retval = ret;
				goto out;
			}

			/* TS packet buffer over-flow detection */
			if (g_tscnt.pwr <= g_tscnt.prd &&
				g_tscnt.prd <
				g_tscnt.pwr + g_tscnt.ts_rx_size) {
				if (g_tscnt.tsread_flag)
					g_tscnt.ovf++;
				if ((g_tscnt.ovf % 100) == 1) {
					pr_info("packet buffer over flow (%d)",
						g_tscnt.ovf);
				}
			}

			/* wake up the TS read */
			if (g_tscnt.tsread_flag == TUNER_TSREAD_WAIT) {
				g_tscnt.tsread_flag = TUNER_TSREAD_IDLE;
				/* wake up the TS read */
				if (waitqueue_active(&g_tscnt.tsread_waitq))
					wake_up_interruptible(
					&g_tscnt.tsread_waitq);
			}
		}

		/* check the stop request to the TS I/F thread */
		if (g_tscnt.tsifth_flag & TUNER_TSIFTH_END) {
			pr_debug("Caught the stop request.\n");
			g_tscnt.tsifth_flag = TUNER_TSIFTH_NONE;
			break;
		}
	} /* while */
	pr_debug("Tail of thread function %s.\n", __func__);

out:
	return retval;
}

/**************************************************************************//**
 * Set TS I/F context
 *
 * This function detect active OFDM circuit, and calculate suitable
 * memory size for handling TS data.
 * After execution, the following member variables of "_tsif_context"
 * structure are update.
 *		bw: active OFDM circuit
 *		ts_packet_size: byte num of an TS packet
 *		ts_record_size: RX transfer size a transaction.
 *		ts_pktbuf_size: buffer size stored by TS I/F thread
 *
 * @retval 0			Normal end
 * @retval <0			error
 *
 * @param [out] ptscnt	pointer to TS I/F context structure
 ******************************************************************************/
static int tuner_drv_tsif_set_cntxt(struct _tsif_cntxt *tc)
{
	int ret = 0;
	uint8_t pbuf_max_size;
	uint16_t pktbuf_size;
	uint32_t lowerlimit;
	uint32_t maxbank = 48;
	uint32_t rxsize;

#if TUNER_TSPKTBUF_MODE == 2
	const uint16_t bufsize_tbl[8] = {
		1915, 1851, 1787, 1659, 1403, 891, 635, 379};
#else
	const uint16_t bufsize_tbl[8] = {
		695, 631, 567, 439, 183, 695, 695, 695};
#endif

#ifdef DEBUG
	const char *bwseg[3] = { "BW13", "BW1", "BW3" };
#endif

	if (tc == NULL || tc->tsif == NULL) {
		pr_err("illegal argument.\n");
		return -EINVAL;
	}
	if (!tc->tsifth_wait) {
		pr_warn("TS buffering have been already started.\n");
		return -EINPROGRESS;
	}

	/* detect the reception segment system */
	ret = tuner_drv_bwseg(&tc->bw);
	if (ret) {
		pr_err("tuner_drv_bwseg(), failed.\n");
		return ret;
	}

#ifdef DEBUG
	pr_debug("Reception segment system: %s", bwseg[tc->bw]);
#endif

	/* TS record size a RX transaction */
	/* Readable packet number, when DATAREADY is high. */

	/* Check packet buffer size configuration. */
	ret = tuner_drv_hw_read_reg(2, 0x62, 1, &pbuf_max_size);
	if (ret) {
		pr_err("Main2 register read failed.\n");
		return ret;
	}

	pktbuf_size = bufsize_tbl[(pbuf_max_size>>4)&0x7];

	/* Calculate hardware lower limit value */
	switch ((tc->tsif->thl[tc->bw]&0x7)) {
	case 0:
	case 6:
	case 7:
		lowerlimit = 16;
		break;
	default:
		lowerlimit = (pktbuf_size)>>(6-(tc->tsif->thl[tc->bw]&0x7));
		break;
	}

	rxsize = lowerlimit;
	/* re-calculate lowerlimit when use fixed rxsize */
	if (tc->tsif->thl[tc->bw] >= 0x10) {
		rxsize = 4<<(tc->tsif->thl[tc->bw]>>4);
		if (rxsize > lowerlimit)
			rxsize = lowerlimit;
	}

	tc->ts_rxpkt_num = rxsize&0x07fc;

	tc->ts_rx_size =
	tc->ts_rxpkt_num * g_ts_pkt_size[tc->tsif->ts_pkt_type];
	if (D_TUNER_SPI_BUFFER_SIZE < tc->ts_rx_size) {
		pr_debug("spi buffer over :%d < %d\n",
				D_TUNER_SPI_BUFFER_SIZE,
				(int)tc->ts_rx_size);
		tc->ts_rx_size = D_TUNER_SPI_BUFFER_SIZE;
	}

	/* TS buffer size for the TS I/F thread */
	maxbank = TUNER_MAX_TSPKTBUF_SIZE/tc->ts_rx_size;
	if (maxbank > TUNER_MAX_TSPKTBUF_BANK)
		maxbank = TUNER_MAX_TSPKTBUF_BANK;

	tc->ts_pktbuf_size = (tc->ts_rx_size) * maxbank;

#ifdef DEBUG
	{
		uint32_t datawindow;

		if (tc->tsif->dwind[tc->bw] == 0) {
			datawindow = 16;
		} else {
		  datawindow = 0;
		  if (tc->tsif->dwind[tc->bw] & 0x01)
			datawindow += (pktbuf_size>>5);
		  if (tc->tsif->dwind[tc->bw] & 0x02)
			datawindow += (pktbuf_size>>4);
		  if (tc->tsif->dwind[tc->bw] & 0x04)
			datawindow += (pktbuf_size>>3);
		  if (tc->tsif->dwind[tc->bw] & 0x08)
			datawindow += (pktbuf_size>>2);
		  if (tc->tsif->dwind[tc->bw] & 0x10)
			datawindow += (pktbuf_size>>1);
		}
		pr_info("MEMSIZE=0x%02x(%d) THL=%02x/DWIND=%02x WL(%d-%d) RXPKT=%zu BANK=%d BUFSIZE=%zu\n",
			pbuf_max_size, pktbuf_size,
			tc->tsif->thl[tc->bw], tc->tsif->dwind[tc->bw],
			lowerlimit, lowerlimit+datawindow,
			tc->ts_rxpkt_num, maxbank, tc->ts_pktbuf_size);

	}
#endif

	if ((tc->bw == 0 && tc->ts_rx_size <
		(64*g_ts_pkt_size[tc->tsif->ts_pkt_type])) ||
		(tc->bw == 2 && tc->ts_rx_size <
		(32*g_ts_pkt_size[tc->tsif->ts_pkt_type]))) {
		pr_warn("Looks waterline of interrput is too low. Please confirm it.\n");
	}

	return 0;
}

/**************************************************************************//**
 * Start to receive TS data.
 *
 * This function activate to receive TS data.
 * The "tsifth" which is the kernel thread to receive and store
 * the TS data from the tuner device had been dispatched and wait.
 *
 * @retval 0			normal
 * @retval <0			error
 ******************************************************************************/
static int tuner_drv_tsif_start(void)
{
	int ret = 0;
	int buffer_size;

	pr_debug("%s\n", __func__);

	if (!g_tscnt.tsifth_wait) {
		/* TS I/F active! */
		pr_warn("TS buffering had already been started.\n");
		return -EINPROGRESS;
	}

	ret = tuner_drv_tsif_set_cntxt(&g_tscnt);

	if (ret) {
		pr_err("tuner_drv_hw_tsif_set_cntxt() failed.\n");
		return ret;
	}

	pr_debug("spi buffer pointer       : %p\n", g_tscnt.spibuf);
	if (g_tscnt.spibuf == NULL) {
		pr_err("memory allocation failed.(spibuf)\n");
		return -ENOMEM;
	}

	pr_debug("PKTSIZE:%zu PKTNUM:%zu RXSIZE:%zu BUFSIZE:%zu\n",
			g_ts_pkt_size[g_tscnt.tsif->ts_pkt_type],
			g_tscnt.ts_rxpkt_num, g_tscnt.ts_rx_size,
			g_tscnt.ts_pktbuf_size);

	vfree(g_tscnt.pktbuf);
	g_tscnt.pktbuf = NULL;

#ifdef TUNER_CONFIG_SPI_ALIGN
	buffer_size = g_tscnt.ts_pktbuf_size + TUNER_CONFIG_SPI_ALIGN - 1;
#else
	buffer_size = g_tscnt.ts_pktbuf_size;
#endif

#if defined(DPATH_GPIF) && defined(TUNER_CONFIG_GPIF_DMA)
	g_tscnt.pktbuf = kmalloc(buffer_size, GFP_KERNEL | GFP_DMA);
#else
	g_tscnt.pktbuf = vmalloc_user(buffer_size);
#endif

	if (g_tscnt.pktbuf == NULL) {
		pr_err("memory allocation failed.\n");
		return -ENOMEM;
	}

	memset(g_tscnt.pktbuf, 0, g_tscnt.ts_pktbuf_size);
	memset(g_tscnt.spibuf, 0, g_tscnt.ts_rx_size);
	g_tscnt.pwr = g_tscnt.prd = 0;
	g_tscnt.ovf = 0;

	ret = tuner_drv_hw_tsif_config(&g_tscnt);
	if (ret) {
		pr_err("tuner_drv_hw_tsif_config() failed.\n");
		vfree(g_tscnt.pktbuf);
		g_tscnt.pktbuf = NULL;
		return ret;
	}

	/* TS read operation is IDLE state. */
	g_tscnt.tsread_flag = TUNER_TSREAD_IDLE;
	pr_debug("TS-Read Timeout limit: %lu [ms]\n",
		TUNER_CONFIG_TSREAD_TIMEOUT);

	ret = tuner_drv_hw_tsif_sync_pkt();
	if (ret) {
		pr_err("tuner_drv_hw_tsif_sync_pkt failed.\n");
		vfree(g_tscnt.pktbuf);
		g_tscnt.pktbuf = NULL;
		return ret;
	}

	g_tscnt.tsifth_flag = TUNER_TSIFTH_ACTIVE;
	/* re-activate TS I/F */
	if (waitqueue_active(&g_tscnt.tsifth_waitq))
		wake_up_interruptible(&g_tscnt.tsifth_waitq);
	g_tscnt.tsifth_wait = false;

	return ret;
}

/**************************************************************************//**
 * Stop receiving TS data.
 *
 * This function make the TS I/F thread wait status.
 *
 * @retval 0			normal
 * @retval <0			error
 ******************************************************************************/
static int tuner_drv_tsif_stop(void)
{
	int ret = 0;
	uint32_t i = 0;

	pr_debug("%s\n", __func__);

	if (g_tscnt.tsifth_wait) {
		/* TS I/F is NOT active! */
		pr_warn("TS buffering is not active.\n");
		return 0;
	}

	/* stop (be waiting status) TS I/F thread */
	g_tscnt.tsifth_flag = TUNER_TSIFTH_NONE;
	g_tscnt.tsifth_wait = true;

	/* confirm the status of the TS I/F thread */
	while (1) {
		/*
		 * NOTE
		 * It takes about 20-30[ms] to become able to detect the TS
		 * I/F thread changed in waiting state in the return value of
		 * the waitqueue_activate() function.
		 */
		if (waitqueue_active(&g_tscnt.tsifth_waitq)) {
			pr_debug("TS I/F thread is going to do the stop procedure.\n");
			break;
		}

		usleep_range(TUNER_TSIFTH_SLEEP_MIN, TUNER_TSIFTH_SLEEP_MAX);

		i++;
		if (i >= TUNER_TSIFTH_SLEEP_RETRY) {
			pr_crit("cannot stop TS I/F thread.\n");
			break;
		}
	}

	/* release the waiting in TS read operation */
	if (g_tscnt.tsread_flag == TUNER_TSREAD_WAIT) {
		g_tscnt.tsread_flag = TUNER_TSREAD_END;
		/* release the waiting in TS read operation */
		if (waitqueue_active(&g_tscnt.tsread_waitq))
			wake_up_interruptible(&g_tscnt.tsread_waitq);
	}

	g_tscnt.pwr = g_tscnt.prd = 0;
	g_tscnt.ovf = 0;
	g_tscnt.ts_rxpkt_num = 0;
	g_tscnt.ts_rx_size = 0;
	g_tscnt.ts_pktbuf_size = 0;

	vfree(g_tscnt.pktbuf);
	kfree(g_tscnt.tsif);

	g_tscnt.pktbuf = NULL;
	g_tscnt.tsif = NULL;

	return ret;
}

/**************************************************************************//**
 * return TS packet size
 *
 * @retval		>0	packet size
 * @retval		<0	error
 ******************************************************************************/
static int tuner_drv_tsif_pktsize(void)
{
	if (g_tscnt.tsifth_wait) {
		pr_warn("TS buffering is not started.\n");
		return -EAGAIN;
	}

	return (int)(g_ts_pkt_size[g_tscnt.tsif->ts_pkt_type]);
}

/**************************************************************************//**
 * @brief Call-back function for TS-Read Timeout.
 *
 ******************************************************************************/
static void tsread_timer_handler(struct timer_list *t)
{
	pr_info("Timeout (registered %ld, now %ld).", tsread_timer_data, jiffies);

	if (g_tscnt.tsread_flag != TUNER_TSREAD_WAIT) {
		pr_info("Timer is not to be restarted.");
		return;
	}

	g_tscnt.tsread_flag = TUNER_TSREAD_TIMEOUT;
	/* time out wake up*/
	if (waitqueue_active(&g_tscnt.tsread_waitq))
		wake_up_interruptible(&g_tscnt.tsread_waitq);
}

/**************************************************************************//**
 * return TS block count
 *
 * @retval		>0	block size
 * @retval		<0	error
 ******************************************************************************/
static int tuner_drv_tsif_blksize(void)
{
	if (g_tscnt.tsifth_wait) {
		pr_warn("TS buffering is not started.\n");
		return -EAGAIN;
	}

	return (int)(g_tscnt.ts_rxpkt_num);
}

/**************************************************************************//**
 * detect active OFDM block (BW13 or BW01)
 *
 * @retval 0			normal
 * @retval <0			error
 ******************************************************************************/
static inline int  tuner_drv_bwseg(enum _bw_seg *pbw)
{
	int ret;
	uint8_t rd;
	uint8_t sys;

	ret = tuner_drv_hw_read_reg(Main1, 0x02, 1, &rd);	/* SYSSET */
	if (ret) {
		pr_debug("register SYSSET, read fail.\n");
		return ret;
	}
	sys = (rd & 0xC0) >> 6;	/* SYS is SYSSET[7:6]. */
	switch (sys) {
	case 0:
		*pbw = TUNER_DRV_BW13;
		break;
	case 1:
		*pbw = TUNER_DRV_BW1;
		break;
	case 3:
		*pbw = TUNER_DRV_BW3;
		break;
	default:
		pr_err("illegal SYS parameter.\n");
		return -EFAULT;
	}

	return 0;
}

#endif /* TUNER_CONFIG_DPATH != TUNER_DPATH_NONE */

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Socionext Inc.");
MODULE_DESCRIPTION("MM Tuner Driver");

module_init(tuner_drv_start);
module_exit(tuner_drv_end);
/*******************************************************************************
 * Copyright (c) 2015 Socionext Inc.
 ******************************************************************************/
