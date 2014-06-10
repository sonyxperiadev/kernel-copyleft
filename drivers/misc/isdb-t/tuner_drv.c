/******************************************************************************
 *
 *  file name       : tuner_drv.c
 *  brief note      : The Control Layer of Tmm Tuner Driver
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
#include <asm/irq.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <mach/isdbt_tunerpm.h>
#include "tuner_api.h"
#include "tuner_drv.h"
#include "tuner_drv_sys.h"

struct g_tuner_kthread {
	wait_queue_head_t poll_wait_q;
	spinlock_t lock;
	unsigned long wakeup_flag;
	unsigned char intcnd_f;
	unsigned char intcnd_s;
	struct task_struct *kthread_id;
	u32 kthread_flag;
	wait_queue_head_t kthread_wait_q;
} tnr_thr;

struct g_tuner_device {
	struct mutex g_tuner_mutex;
	unsigned long open_cnt;
	struct platform_device *mmtuner_device;
	struct class *device_class;
} tnr_dev;

int tuner_kernel_thread(void *arg)
{
	unsigned long flags;
	unsigned long ktread_flg;
	int err_cnt;
	int non_err_cnt;
	int err_comp_cnt;
	int cir_info;
	unsigned int sync1;
	unsigned int sync2;
	unsigned int stm;
	unsigned int read2;
	unsigned int i2c;
	unsigned int berrdy;
	unsigned int pkt_err_flag;
	unsigned int per_tmp;

	struct _tune_data_easy_bper per_mon;
	memset(&per_mon, 0x00,
		(sizeof(struct _tune_data_easy_bper)));

	while (true) {
		wait_event_interruptible(tnr_thr.kthread_wait_q,
			tnr_thr.kthread_flag);

		spin_lock_irqsave(&tnr_thr.lock, flags);
		ktread_flg = tnr_thr.kthread_flag;
		spin_unlock_irqrestore(&tnr_thr.lock, flags);

		err_cnt = 0;
		non_err_cnt = 0;
		err_comp_cnt = 0;
		cir_info = get_cir_info();

		if ((ktread_flg & TUNER_KTH_IRQHANDLER) ==
			TUNER_KTH_IRQHANDLER) {
			ant_switch_thread_ena(THR_WAIT);

			while (tnr_dev.open_cnt) {
				sync1 = tuner_get_sync(cir_info, TUNER_SYNC);
				if (sync1 == AUTO_ANT_NOT_SYNC ||
					sync1 == -EINVAL) {
					tuner_set_status(TNR_UNLOCK);
					break;
				}

				msleep(BPER_POLE_WAIT);

				per_tmp = tuner_get_per(&per_mon);
				if (per_tmp == -EINVAL) {
					tuner_set_status(TNR_UNLOCK);
					break;
				}

				sync1 = tuner_get_sync(
					cir_info,
					TUNER_SYNC);
				sync2 = tuner_get_sync(
						cir_info,
						TUNER_SYNC_SEQ);
				stm = per_mon.info_stm[F_LYR-1];
				read2 = per_mon.info_read2[F_LYR-1];
				i2c = per_mon.info_i2c;
				berrdy = per_mon.info_berrdy;
				if ((sync1 == AUTO_ANT_SYNC) &&
					(sync2 == AUTO_ANT_SYNC) &&
					(per_tmp == 0) &&
					((stm && read2) || (i2c && berrdy))) {
					err_cnt = 0;
					non_err_cnt++;
				} else {
					err_cnt++;
					non_err_cnt = 0;
				}
				if (non_err_cnt >=
					NON_ERROR_COUNT) {
					clear_interrupt_mask(
						&tnr_thr.intcnd_f,
						&tnr_thr.intcnd_s);
					sync1 = tuner_get_sync(
						cir_info,
						TUNER_SYNC);
					sync2 = tuner_get_sync(
						cir_info,
						TUNER_SYNC_SEQ);
					pkt_err_flag =
						get_pkt_err_flag(cir_info);
					if ((sync1 == AUTO_ANT_SYNC) &&
						(sync2 == AUTO_ANT_SYNC) &&
						(pkt_err_flag ==
						AUTO_ANT_NO_PKT_ERR)) {
						tuner_set_status(TNR_LOCK);
						tnr_thr.wakeup_flag = TUNER_ON;
						wake_up(&tnr_thr.poll_wait_q);

						spin_lock_irqsave(
							&tnr_thr.lock,
							flags);
						tnr_thr.kthread_flag &=
							~TUNER_KTH_IRQHANDLER;
						spin_unlock_irqrestore(
							&tnr_thr.lock,
							flags);

						ant_switch_thread_ena(THR_GO);
						break;
					}
					err_cnt = 0;
					non_err_cnt = 0;
				} else if (err_cnt >= ERROR_COUNT) {
					tuner_set_status(TNR_UNLOCK);
					break;
				} else {
					err_comp_cnt++;
					if (err_comp_cnt >=
						ERROR_COMPARING_COUNT) {
						tuner_set_status(TNR_UNLOCK);
						break;
					}
				}
			}
			tnr_thr.wakeup_flag = TUNER_ON;
			wake_up(&tnr_thr.poll_wait_q);

			spin_lock_irqsave(&tnr_thr.lock, flags);
			tnr_thr.kthread_flag &= ~TUNER_KTH_IRQHANDLER;
			spin_unlock_irqrestore(&tnr_thr.lock, flags);

			ant_switch_thread_ena(THR_GO);
		}
		if ((ktread_flg & TUNER_KTH_END) == TUNER_KTH_END) {
			spin_lock_irqsave(&tnr_thr.lock, flags);
			tnr_thr.kthread_flag &= ~TUNER_KTH_END;
			spin_unlock_irqrestore(&tnr_thr.lock, flags);
			break;
		}
	}
	return 0;
}

static int tuner_module_entry_open(
	struct inode *inode,
	struct file *file)
{
	if (tnr_dev.open_cnt > 0)
		return -EBUSY;
	else
		tnr_dev.open_cnt++;

	tnr_thr.kthread_flag = TUNER_KTH_NONE;

	init_waitqueue_head(&tnr_thr.kthread_wait_q);

	tnr_thr.kthread_id = kthread_create(tuner_kernel_thread,
		NULL,
		"tuner_kthread");
	if (IS_ERR(tnr_thr.kthread_id)) {
		tnr_thr.kthread_id = NULL;
		platform_device_put(tnr_dev.mmtuner_device);
		return -EIO;
	}

	wake_up_process(tnr_thr.kthread_id);

	if (ant_switch_thread_createthread()) {
		kthread_stop(tnr_thr.kthread_id);
		return -EIO;
	}

	return 0;
}

static int tuner_module_entry_close(
	struct inode *inode,
	struct file *file)
{
	struct devone_data *dev;

	tuner_drv_ant_switch(ANTMODE_NOTUSE);

	if (tnr_dev.open_cnt <= 0)
		return -ESRCH;
	else
		tnr_dev.open_cnt--;

	if (tnr_dev.open_cnt == 0) {
		tuner_drv_release_interrupt();
		if (!file)
			return -ESRCH;

		dev = file->private_data;
	}

	tnr_thr.kthread_flag |= TUNER_KTH_END;
	if (waitqueue_active(&tnr_thr.kthread_wait_q))
		wake_up(&tnr_thr.kthread_wait_q);

	if (tnr_thr.kthread_id)
		kthread_stop(tnr_thr.kthread_id);

	ant_switch_thread_destroythread();

	return 0;
}

static ssize_t tuner_module_entry_write(
	struct file *file, const char *buffer,
	size_t count, loff_t *offsetposition)
{
	int ret;
	unsigned long copy_ret;
	struct _tuner_data_rw *write_arg;

	write_arg = vmalloc(count);
	if (!write_arg || count % sizeof(struct _tuner_data_rw))
		return -EINVAL;

	copy_ret = copy_from_user(write_arg, buffer, count);
	if (copy_ret) {
		vfree(write_arg);
		return -EINVAL;
	}

	ret = tuner_drv_hw_access(
		TUNER_IOCTL_VALSET,
		write_arg,
		(unsigned short)(count / sizeof(struct _tuner_data_rw)));

	vfree(write_arg);

	return ret;
}

static long tuner_module_entry_ioctl(struct file *file,
	unsigned int ucommand, unsigned long uargument)
{
	int ret;
	struct _tuner_data_rw data;
	unsigned long copy_ret;
	int param;
	int antmode;
	int cir_info;
	int current_ant;
	int tuning_cnt;
	struct _tuner_data_rw event_status[TUNER_EVENT_REGNUM];

	mutex_lock(&tnr_dev.g_tuner_mutex);

	if (uargument == 0) {
		mutex_unlock(&tnr_dev.g_tuner_mutex);
		goto err_uargument;
	}

	switch (ucommand) {
	case TUNER_IOCTL_VALGET:
		copy_ret = copy_from_user(
			&data,
			&(*(struct _tuner_data_rw *)uargument),
			sizeof(struct _tuner_data_rw));
		if (copy_ret) {
			mutex_unlock(&tnr_dev.g_tuner_mutex);
			goto err_copy_from_user;
		}

		ret = tuner_drv_hw_access(ucommand, &data, 1);

		if (ret == 0) {
			copy_ret = copy_to_user(
				&(*(struct _tuner_data_rw *)uargument),
				&data,
				sizeof(struct _tuner_data_rw));
			if (copy_ret) {
				mutex_unlock(&tnr_dev.g_tuner_mutex);
				goto err_copy_to_user;
			}
		}
		break;
	case TUNER_IOCTL_VALSET:
		copy_ret = copy_from_user(&data,
			&(*(struct _tuner_data_rw *)uargument),
			 sizeof(struct _tuner_data_rw));
		if (copy_ret) {
			mutex_unlock(&tnr_dev.g_tuner_mutex);
			goto err_copy_from_user;
		}

		ret = tuner_drv_hw_access(ucommand, &data, 1);
		break;
	case TUNER_IOCTL_VALGET_EVENT:
		copy_ret = copy_to_user(&(*(unsigned char *)uargument),
			&tnr_thr.intcnd_f,
			sizeof(unsigned char));
		if (copy_ret) {
			mutex_unlock(&tnr_dev.g_tuner_mutex);
			goto err_copy_to_user;
		}

		copy_ret = copy_to_user(
			&(*(unsigned char *)(uargument + 1)),
			&tnr_thr.intcnd_s,
			sizeof(unsigned char));
		if (copy_ret) {
			mutex_unlock(&tnr_dev.g_tuner_mutex);
			goto err_copy_to_user;
		}
		tnr_thr.intcnd_f = 0x00;
		tnr_thr.intcnd_s = 0x00;
		ret = copy_ret;
		break;
	case TUNER_IOCTL_VALSET_EVENT:
		copy_ret = copy_from_user(&data,
			&(*(struct _tuner_data_rw *)uargument),
			sizeof(struct _tuner_data_rw));
		if (copy_ret) {
			mutex_unlock(&tnr_dev.g_tuner_mutex);
			goto err_copy_from_user;
		}

		event_status[0].slave_adr = TUNER_SLAVE_ADR_M1;
		event_status[0].adr       = TUNER_ADR_INTDEF1_F;
		event_status[0].sbit      = SIG_ENAS_INTDEF1_F;
		event_status[0].ebit      = SIG_ENAE_INTDEF1_F;
		event_status[0].param     = 0x00;
		event_status[0].enabit    = SIG_ENA_INTDEF1_F;
		event_status[1].slave_adr = TUNER_SLAVE_ADR_M1;
		event_status[1].adr       = TUNER_ADR_INTDEF2_F;
		event_status[1].sbit      = SIG_ENAS_INTDEF2_F;
		event_status[1].ebit      = SIG_ENAE_INTDEF2_F;
		event_status[1].param     = 0x00;
		event_status[1].enabit    = SIG_ENA_INTDEF2_F;
		event_status[2].slave_adr = TUNER_SLAVE_ADR_M2;
		event_status[2].adr       = TUNER_ADR_INTDEF_S;
		event_status[2].sbit      = SIG_ENAS_IERR_S;
		event_status[2].ebit      = SIG_ENAE_SEGERRS;
		event_status[2].param     = 0x00;
		event_status[2].enabit    = (SIG_ENA_IERR_S
			|SIG_ENA_EMGFLG_S
			|SIG_ENA_TMCCERR_S
			|SIG_ENA_ISYNC_S
			|SIG_ENA_CDFLG_S
			|SIG_ENA_SSEQ_S
			|SIG_ENA_MGERR_S
			|SIG_ENA_SEGERRS);

		ret = tuner_drv_hw_access(
			TUNER_IOCTL_VALGET, event_status,
			TUNER_EVENT_REGNUM);
		if (ret) {
			mutex_unlock(&tnr_dev.g_tuner_mutex);
			goto err_tuner_drv_hw_access;
		}

		if (((event_status[0].param&event_status[0].enabit)
			== SIG_ENAIRQ_INTDEF1_NONE) &&
			((event_status[1].param&event_status[1].enabit)
			== SIG_ENAIRQ_INTDEF2_NONE) &&
			((event_status[2].param&event_status[2].enabit)
			== SIG_ENAIRQ_INTDEF_NONE)) {
			ret = tuner_drv_set_interrupt();
			if (ret) {
				mutex_unlock(&tnr_dev.g_tuner_mutex);
				goto err_tuner_drv_set_interrupt;
			}
		}
		ret = tuner_drv_hw_access(TUNER_IOCTL_VALSET, &data, 1);
		break;
	case TUNER_IOCTL_VALREL_EVENT:
		copy_ret = copy_from_user(&data,
			&(*(struct _tuner_data_rw *)uargument),
			sizeof(struct _tuner_data_rw));
		if (copy_ret) {
			mutex_unlock(&tnr_dev.g_tuner_mutex);
			goto err_copy_from_user;
		}

		ret = tuner_drv_hw_access(TUNER_IOCTL_VALSET, &data, 1);
		if (ret) {
			mutex_unlock(&tnr_dev.g_tuner_mutex);
			goto err_tuner_drv_hw_access;
		}

		event_status[0].slave_adr = TUNER_SLAVE_ADR_M1;
		event_status[0].adr       = TUNER_ADR_INTDEF1_F;
		event_status[0].sbit      = SIG_ENAS_INTDEF1_F;
		event_status[0].ebit      = SIG_ENAE_INTDEF1_F;
		event_status[0].param     = 0x00;
		event_status[0].enabit    = SIG_ENA_INTDEF1_F;
		event_status[1].slave_adr = TUNER_SLAVE_ADR_M1;
		event_status[1].adr       = TUNER_ADR_INTDEF2_F;
		event_status[1].sbit      = SIG_ENAS_INTDEF2_F;
		event_status[1].ebit      = SIG_ENAE_INTDEF2_F;
		event_status[1].param     = 0x00;
		event_status[1].enabit    = SIG_ENA_INTDEF2_F;
		event_status[2].slave_adr = TUNER_SLAVE_ADR_M2;
		event_status[2].adr       = TUNER_ADR_INTDEF_S;
		event_status[2].sbit      = SIG_ENAS_IERR_S;
		event_status[2].ebit      = SIG_ENAE_SEGERRS;
		event_status[2].param     = 0x00;
		event_status[2].enabit    = (SIG_ENA_IERR_S
			|SIG_ENA_EMGFLG_S
			|SIG_ENA_TMCCERR_S
			|SIG_ENA_ISYNC_S
			|SIG_ENA_CDFLG_S
			|SIG_ENA_SSEQ_S
			|SIG_ENA_MGERR_S
			|SIG_ENA_SEGERRS);

		ret = tuner_drv_hw_access(TUNER_IOCTL_VALGET, event_status,
			TUNER_EVENT_REGNUM);

		if (((event_status[0].param&event_status[0].enabit)
			== SIG_ENAIRQ_INTDEF1_NONE) &&
				((event_status[1].param&event_status[1].enabit)
			== SIG_ENAIRQ_INTDEF2_NONE) &&
				((event_status[2].param&event_status[2].enabit)
			== SIG_ENAIRQ_INTDEF_NONE)) {
			tuner_drv_release_interrupt();
		}
		break;
	case TUNER_IOCTL_VALSET_POWER:
		copy_ret = copy_from_user(&param,
			&(*(int *)uargument),
			sizeof(int));
		if (copy_ret) {
			mutex_unlock(&tnr_dev.g_tuner_mutex);
			goto err_copy_from_user;
		}

		ret = tuner_drv_ctl_power(param);
		break;
	case TUNER_IOCTL_ANT_SWITCH:
		copy_ret = copy_from_user(&antmode,
			&(*(int *)uargument),
			sizeof(int));
		if (copy_ret) {
			mutex_unlock(&tnr_dev.g_tuner_mutex);
			goto err_copy_from_user;
		}

		if (antmode == ANTMODE_AUTO) {
			auto_ant_search_ena(AUTO_ANT_SEARCH_ON);
			tuner_set_status(TNR_UNLOCK);
		} else {
			auto_ant_search_ena(AUTO_ANT_SEARCH_OFF);
			if (antmode == ANTMODE_NOTUSE)
				tuner_drv_ant_switch(ANTMODE_WHIP);
			else
				tuner_drv_ant_switch(antmode);
		}
		break;
	case TUNER_IOCTL_ANT_SEARCH:
		copy_ret = copy_from_user(&tuning_cnt,
			&(*(int *)uargument),
			sizeof(int));
		if (copy_ret) {
			mutex_unlock(&tnr_dev.g_tuner_mutex);
			goto err_copy_from_user;
		}

		set_tuning_cnt(tuning_cnt);
		if (get_tuning_cnt() == 1) {
			tuner_set_status(TNR_UNLOCK);
			ant_switch_thread_activequeue();
		}
		break;
	case TUNER_IOCTL_CIRCUIT_INFO:
		copy_ret = copy_from_user(&cir_info,
			&(*(int *)uargument),
			sizeof(int));
		if (copy_ret) {
			mutex_unlock(&tnr_dev.g_tuner_mutex);
			goto err_copy_from_user;
		}
		set_cir_info(cir_info);
		break;
	case TUNER_IOCTL_GET_CURRENT_ANT:
		current_ant = get_current_ant();
		copy_ret = copy_to_user(&(*(int *)uargument),
			&current_ant,
			sizeof(int));
		if (copy_ret) {
			mutex_unlock(&tnr_dev.g_tuner_mutex);
			goto err_copy_to_user;
		}
		break;
	default:
		goto err_ucommand;
	}
	mutex_unlock(&tnr_dev.g_tuner_mutex);
	return 0;

err_uargument:
err_ucommand:
err_copy_from_user:
err_copy_to_user:
err_tuner_drv_hw_access:
err_tuner_drv_set_interrupt:
	return -EINVAL;
}

static unsigned int tuner_module_entry_poll(
	struct file *file,
	struct poll_table_struct *poll_tbl)
{
	unsigned long tuner_flags;
	unsigned int  tuner_mask;

	tuner_mask = 0;

	poll_wait(file, &tnr_thr.poll_wait_q, poll_tbl);

	spin_lock_irqsave(&tnr_thr.lock, tuner_flags);

	if (tnr_thr.wakeup_flag == TUNER_ON)
		tuner_mask = (POLLIN | POLLRDNORM);

	tnr_thr.wakeup_flag = TUNER_OFF;

	spin_unlock_irqrestore(&tnr_thr.lock, tuner_flags);

	return tuner_mask;
}

static const struct file_operations tuner_file_operations = {
	.owner   = THIS_MODULE,
	.write   = tuner_module_entry_write,
	.poll    = tuner_module_entry_poll,
	.unlocked_ioctl = tuner_module_entry_ioctl,
	.open    = tuner_module_entry_open,
	.release = tuner_module_entry_close
};

irqreturn_t tuner_interrupt(int irq, void *dev_id)
{
	tnr_thr.kthread_flag |= TUNER_KTH_IRQHANDLER;
	if ((waitqueue_active(&tnr_thr.kthread_wait_q)) &&
		(get_auto_ant_search_ena() == ANT_SEARCH_AUTO_ON) &&
		(tuner_get_status() == TNR_LOCK))
		wake_up(&tnr_thr.kthread_wait_q);

	return IRQ_HANDLED;
}

static int tuner_probe(struct platform_device *pdev)
{
	int ret;

	ret = register_chrdev(
		TUNER_CONFIG_DRV_MAJOR,
		TUNER_CONFIG_DRIVER_NAME, &tuner_file_operations);

	if (ret < 0)
		return ret;

	ret = tunerpm_dev_init();
	if (ret)
		return ret;

	init_waitqueue_head(&tnr_thr.poll_wait_q);
	spin_lock_init(&tnr_thr.lock);
	tnr_thr.wakeup_flag = TUNER_OFF;
	tnr_thr.intcnd_f = 0x00;
	tnr_thr.intcnd_s = 0x00;
	tnr_dev.open_cnt = 0;

	mutex_init(&tnr_dev.g_tuner_mutex);

	return 0;
}

int __devexit tuner_remove(struct platform_device *pdev)
{
	tuner_drv_release_interrupt();

	tunerpm_dev_finalize();

	unregister_chrdev(TUNER_CONFIG_DRV_MAJOR, TUNER_CONFIG_DRIVER_NAME);

	return 0;
}

static struct platform_driver mmtuner_driver = {
	.probe  = tuner_probe,
	.remove = __exit_p(tuner_remove),
	.driver = {
		.name = D_TUNER_CONFIG_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init tuner_drv_start(void)
{
	int ret;
	struct device *dev = NULL;

	ret = platform_driver_register(&mmtuner_driver);

	if (ret)
		goto err_platform_driver_register;

	tnr_dev.mmtuner_device = platform_device_alloc("mmtuner", -1);

	if (!tnr_dev.mmtuner_device) {
		ret = -ENOMEM;
		goto err_platform_device_alloc;
	}

	ret = platform_device_add(tnr_dev.mmtuner_device);
	if (ret)
		goto err_platform_device_add;

	tnr_dev.device_class = class_create(THIS_MODULE, "mmtuner");
	if (IS_ERR(tnr_dev.device_class)) {
		ret = PTR_ERR(tnr_dev.device_class);
		goto err_class_create;
	}

	dev = device_create(
		tnr_dev.device_class,
		NULL,
		MKDEV(TUNER_CONFIG_DRV_MAJOR, TUNER_CONFIG_DRV_MINOR),
		NULL,
		"mmtuner");

	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		goto err_device_create;
	}

	return 0;

err_device_create:
err_class_create:
err_platform_device_add:
	platform_device_put(tnr_dev.mmtuner_device);
err_platform_device_alloc:
	platform_driver_unregister(&mmtuner_driver);
err_platform_driver_register:

	return ret;
}

static  void __exit tuner_drv_end(void)
{
	device_destroy(tnr_dev.device_class,
		MKDEV(TUNER_CONFIG_DRV_MAJOR,
			TUNER_CONFIG_DRV_MINOR));
	class_destroy(tnr_dev.device_class);
	platform_device_unregister(tnr_dev.mmtuner_device);
	platform_driver_unregister(&mmtuner_driver);
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MM Tuner Driver");

module_init(tuner_drv_start);
module_exit(tuner_drv_end);
