/* linux/drivers/misc/isdb-t/tuner_drv_ant_switch.c
 *
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * Author: Tatsuya Ooka <Tatsuya.Ooka@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#include <asm/irq.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
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
#include "tuner_api.h"
#include "tuner_drv.h"
#include "tuner_drv_sys.h"
#include "tuner_register.h"

/* parameter check for tuner_moni_easy_bper_prmchk() */
#define TUNER_PARAM_EBPER_BERDSEL_NG        0 /*BERDSEL_F check parameter */
#define TUNER_PARAM_EBPER_BERDSEL_MAX       3 /*BERDSEL_F check parameter */
#define TUNER_PARAM_EBPER_BERVSEL_MAX       1 /*BERVSEL_F check parameter */
#define TUNER_PARAM_EBPER_BERTIMMD_MAX      1 /*BERTIMMD_F check parameter*/
#define TUNER_PARAM_EBPER_BERTIM_MAX       15 /*BERTI_F    check parameter*/
#define TUNER_PARAM_EBPER_BERVLEN_A         0 /*BERVLENA_F check parameter*/
#define TUNER_PARAM_EBPER_BERVLEN_B         1 /*BERVLENB_F check parameter*/
#define TUNER_PARAM_EBPER_BERVLEN_C         2 /*BERVLENV_F check parameter*/
#define TUNER_PARAM_EBPER_BERVLEN_MAX      15 /*BERVLENx_F check parameter*/
#define TUNER_PARAM_EBPER_BERRDSEL_MAX      1 /*BERRDSEL_F check parameter*/
#define TUNER_PARAM_EBPER_BERLAY_MAX        3 /*BERLAY_F  check parameter */
#define TUNER_PARAM_EBPER_BERLEN_MAX       13 /*BERLEN_S  check parameter */
#define TUNER_PARAM_EBPER_BERSEL_MAX        1 /*BERSEL_S  check parameter */
/* BER/PER read */
#define TUNER_MONI_BPER_CHK_MAX       0xFFFF  /*BPERCHK wait loop           */
#define TUNER_MONI_FULLSEG_LAYNUM          3  /*Full-segment number of layer*/
#define TUNER_MONI_ONESEG_LAYNUM           1  /*1-segment number of layer   */
#define TUNER_MONI_BPERNUM                 2  /*BERPER                      */
#define TUNER_MONI_BERSET                  0  /*BER function setting        */
#define TUNER_MONI_PERSET                  1  /*PER function setting        */
#define TUNER_MONI_BERLAY_A                1  /*A layer setting             */
#define TUNER_MONI_BERLAY_B                2  /*B layer setting             */
#define TUNER_MONI_BERLAY_C                3  /*C layer setting             */
#define TUNER_MONI_BPER_OFF                0  /*BER/PER function OFF        */
/* RSSI monitor calculation */
#define TUNER_MONI_RSSI_PARAM1            -1  /*RSSI calculation */
#define TUNER_MONI_RSSI_PARAM2           256  /*RSSI calculation */
#define TUNER_MONI_RSSI_CHECK            128  /*pos/neg decision */
/* Data count mode calculation */
#define TUNER_MONI_OPE_BER1              203  /*After Viterbi BER calculation*/
#define TUNER_MONI_OPE_BER2                8  /*After Viterbi BER calculation*/
/* Monitor information */
#define TUNER_MONI_MEABIT_UPPSHIFT     0x100
#define TUNER_MONI_ERRBIT_UPPSHIFT   0x10000
#define TUNER_MONI_ERRBIT_MIDSHIFT     0x100
#define TUNER_MONI_RDATANUM 20
#define TUNER_MONI_MEABIT_UPPSHIFT 0x100

#define TUNER_LOCK_SLEEP            500
#define TUNER_UNLOCK_SLEEP          200

#define THREAD_FLAG(x, y) (((x) & (y) ? (1) : (0)))

static struct g_ant_switch_kthread {
	spinlock_t lock;
	struct task_struct *kthread_id;
	u32 kthread_flag;
	unsigned char intcnd_f_dummy;
	unsigned char intcnd_s_dummy;
	int wait_kthread_flag;
} ant_thr;

static struct g_tuner_device {
	int ODR[ANT_NUM];
	int ODR_ANT[ANT_NUM];
	int cir_info;
	int status;
	int enabler;
	int current_ant;
	int tuning_cnt;
} ant_swt;

int ant_switch_thread(void *arg)
{
	int ret;
	int tuner_status;
	int tuning_count;
	unsigned long flags;
	unsigned long ktread_flg;
	unsigned int auto_ant_search;

	while (true) {
		spin_lock_irqsave(&ant_thr.lock, flags);
		ktread_flg = ant_thr.kthread_flag;
		spin_unlock_irqrestore(&ant_thr.lock, flags);

		auto_ant_search = get_auto_ant_search_ena();

		if (THREAD_FLAG(ktread_flg, TUNER_KTH_IRQHANDLER) &&
			(auto_ant_search != ANT_SEARCH_AUTO_OFF)) {
			while (!THREAD_FLAG(
				ant_thr.kthread_flag, TUNER_KTH_END) &&
				(ant_thr.wait_kthread_flag == 1)) {
				tuner_status = tuner_get_status();
				tuning_count = get_tuning_cnt();
				if ((tuning_count == 1 &&
					tuner_status == TNR_UNLOCK) ||
					tuner_status == TNR_TIMEOUT) {
					ret = tuner_search_ant();
					if (ret)
						break;
					ret = tuner_stability();
					if (ret)
						break;
				} else if (
					(tuning_count > 1 &&
						tuner_status == TNR_UNLOCK)) {
					ret = tuner_stability();
					if (ret)
						break;
				}

				if (tuner_get_status() == TNR_LOCK) {
					msleep(TUNER_LOCK_SLEEP);
					break;
				}
			}
		}
		msleep(TUNER_UNLOCK_SLEEP);

		if (ktread_flg & TUNER_KTH_END) {
			spin_lock_irqsave(&ant_thr.lock, flags);
			ant_thr.kthread_flag &= ~TUNER_KTH_END;
			spin_unlock_irqrestore(&ant_thr.lock, flags);
			break;
		}
	}
	return 0;
}

void ant_switch_thread_ena(int on)
{
	ant_thr.wait_kthread_flag = on;
}

void ant_switch_thread_activequeue(void)
{
	ant_thr.kthread_flag |= TUNER_KTH_IRQHANDLER;
	ant_thr.wait_kthread_flag = 1;
}

int ant_switch_thread_createthread(void)
{
	ant_thr.kthread_flag = TUNER_KTH_NONE;
	ant_thr.wait_kthread_flag |= TUNER_KTH_IRQHANDLER;
	ant_thr.kthread_id =
		kthread_create(ant_switch_thread, NULL, "ant_switch_kthread");
	if (IS_ERR(ant_thr.kthread_id)) {
		ant_thr.kthread_id = NULL;
		return -EIO;
	}
	wake_up_process(ant_thr.kthread_id);

	return 0;
}

void ant_switch_thread_destroythread(void)
{
	ant_thr.kthread_flag |= TUNER_KTH_END;
	ant_thr.wait_kthread_flag |= TUNER_KTH_END;

	if (ant_thr.kthread_id)
		kthread_stop(ant_thr.kthread_id);

	set_tuning_cnt(0);
}

int tuner_search_ant(void)
{
	int d_RSSI[ANT_NUM];
	int ODR_TMP[ANT_NUM];
	int ODR_ANT_TMP[ANT_NUM];
	int i, j;

	for (i = 0; i < ANT_NUM; i++) {
		ant_swt.ODR[i] = 0;
		ant_swt.ODR_ANT[i] = 0;
		ODR_TMP[i] = 0;
		ODR_ANT_TMP[i] = 0;
	}

	for (i = 0; i < ANT_NUM; i++) {
		d_RSSI[i] = tuner_set_ant_get_drssi(i);

		if ((d_RSSI[i] >= ODR_TMP[0]) &&
			(d_RSSI[i] > ODR_TMP[1])) {
			ant_swt.ODR[0] = d_RSSI[i];
			ant_swt.ODR[1] = ODR_TMP[0];
			ant_swt.ODR_ANT[0] = i;
			ant_swt.ODR_ANT[1] = ODR_ANT_TMP[0];
		} else {
			ant_swt.ODR[1] = d_RSSI[i];
			ant_swt.ODR_ANT[1] = i;
		}

		for (j = 0; j < ANT_NUM; j++) {
			ODR_TMP[j] = ant_swt.ODR[j];
			ODR_ANT_TMP[j] = ant_swt.ODR_ANT[j];
		}
	}

	set_ant(ant_swt.ODR_ANT[0]);

	msleep(BPER_STABLE_WAIT1);

	return 0;
}

int tuner_set_ant_get_drssi(int ant_num)
{
	int ret;
	struct _tune_data_rssi rssi_monitor;

	memset(&rssi_monitor, 0x00, (sizeof(struct _tune_data_rssi)));

	set_ant(ant_num);

	msleep(ANT_SET_WAIT);

	switch (ant_num) {
	case CONNECT_USB:
		ret = tuner_get_rssi(
			get_cir_info(),
			&rssi_monitor);
		if (ret)
			return ret;

		return rssi_monitor.info_rssi - DEF_RSSI1;
		break;
	case CONNECT_EARPHONE:
		ret = tuner_get_rssi(
			get_cir_info(),
			&rssi_monitor);
		if (ret)
			return ret;

		return rssi_monitor.info_rssi - DEF_RSSI2;
		break;
	case CONNECT_WHIP:
		ret = tuner_get_rssi(
			get_cir_info(),
			&rssi_monitor);
		if (ret)
			return ret;

		return rssi_monitor.info_rssi - DEF_RSSI3;
		break;
	default:
		break;
	}
	return -EINVAL;
}

void set_ant(int ant_num)
{
	switch (ant_num) {
	case CONNECT_USB:
		tuner_drv_ant_switch(ANTMODE_USB);
		break;
	case CONNECT_EARPHONE:
		tuner_drv_ant_switch(ANTMODE_EARPHONE);
		break;
	case CONNECT_WHIP:
		tuner_drv_ant_switch(ANTMODE_WHIP);
		break;
	default:
		break;
	}
}

int tuner_stability(void)
{
	unsigned int sync1;
	unsigned int sync2;
	unsigned int pkt_err_flag;
	int count;
	int cir_info;
	int a_num;

	count = 0;
	a_num = 0;

	while (true) {
		cir_info = get_cir_info();
		sync1 = tuner_get_sync(cir_info, TUNER_SYNC);
		sync2 = tuner_get_sync(cir_info, TUNER_SYNC_SEQ);
		pkt_err_flag = get_pkt_err_flag(cir_info);
		if ((sync1 == AUTO_ANT_SYNC) &&
			(sync2 == AUTO_ANT_SYNC) &&
			(pkt_err_flag == AUTO_ANT_NO_PKT_ERR)) {
			clear_interrupt_mask(
				&ant_thr.intcnd_f_dummy,
				&ant_thr.intcnd_s_dummy);
			sync1 = tuner_get_sync(cir_info, TUNER_SYNC);
			sync2 = tuner_get_sync(cir_info, TUNER_SYNC_SEQ);
			pkt_err_flag = get_pkt_err_flag(cir_info);
			if ((sync1 == AUTO_ANT_SYNC) &&
				(sync2 == AUTO_ANT_SYNC) &&
				(pkt_err_flag == AUTO_ANT_NO_PKT_ERR)) {
				tuner_set_status(TNR_LOCK);
				break;
			}
		} else {
			if (count >= BPER_TIME_OUT_COUNT) {
				if (a_num == (ANT_NUM - 1)) {
					tuner_set_status(TNR_TIMEOUT);
					break;
				} else {
					a_num++;

					if (ant_swt.ODR[a_num] >
						RSSI_MIN_THREATHOLD) {
						set_ant(ant_swt.ODR_ANT[a_num]);
						msleep(BPER_STABLE_WAIT2);
						count = 0;
					} else {
						tuner_set_status(TNR_TIMEOUT);
						break;
					}
				}
			} else {
				msleep(BPER_STABLE_WAIT2);
				count++;
			}
		}
	}

	return 0;
}

int tuner_get_bper(int param, void *data)
{
	int ret;

	if (!data)
		return -EINVAL;

	ret = tuner_moni_easy_bper_prmchk(
		param,
		(struct _tune_data_easy_bper *)data);
	if (ret)
		return -EINVAL;

	switch (param) {
	case TUNER_STATE_ONESEG:
		ret = tuner_moni_easy_bper_1seg(
			(struct _tune_data_easy_bper *)data);
		break;
	case TUNER_STATE_FULLSEG:
	case TUNER_STATE_FULL_ONE_SEG:
		ret = tuner_moni_easy_bper_fullseg(
			(struct _tune_data_easy_bper *)data);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

int tuner_moni_easy_bper_prmchk(int param, struct _tune_data_easy_bper *data)
{
	int ret;

	switch (param) {
	case TUNER_STATE_ONESEG:
		if (data->set_len > TUNER_PARAM_EBPER_BERLEN_MAX ||
			data->set_sel > TUNER_PARAM_EBPER_BERSEL_MAX)
			ret = -EINVAL;
		else
			ret = 0;
		break;
	case TUNER_STATE_FULLSEG:
	case TUNER_STATE_FULL_ONE_SEG:
		if (data->set_rank > TUNER_PARAM_EBPER_BERDSEL_MAX ||
			data->set_rank == TUNER_PARAM_EBPER_BERDSEL_NG ||
			data->set_read > TUNER_PARAM_EBPER_BERVSEL_MAX ||
			data->set_mode > TUNER_PARAM_EBPER_BERTIMMD_MAX ||
			data->set_time > TUNER_PARAM_EBPER_BERTIM_MAX ||
			data->set_data[TUNER_PARAM_EBPER_BERVLEN_A] >
				TUNER_PARAM_EBPER_BERVLEN_MAX ||
			data->set_data[TUNER_PARAM_EBPER_BERVLEN_B] >
				TUNER_PARAM_EBPER_BERVLEN_MAX ||
			data->set_data[TUNER_PARAM_EBPER_BERVLEN_C] >
				TUNER_PARAM_EBPER_BERVLEN_MAX ||
			data->set_set > TUNER_PARAM_EBPER_BERRDSEL_MAX ||
			data->set_lay > TUNER_PARAM_EBPER_BERLAY_MAX)
			ret = -EINVAL;
		else
			ret = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

int tuner_moni_easy_bper_1seg(struct _tune_data_easy_bper *data)
{
	struct _tuner_data_rw rw_data[TUNER_MONI_RDATANUM];
	int ret;
	unsigned long berrdu;
	unsigned long berrdl;
	unsigned long berrd;
	unsigned long berlend_tmp;
	unsigned long berlend;
	unsigned long  ber_long;
	unsigned long  per_long;
	unsigned long ber_tmp;

	if (!data)
		return -EINVAL;

	rw_data[0].adr    = TUNER_ADR_BERSET_S;
	rw_data[0].sbit   = SIG_ENAS_BERST_S;
	rw_data[0].ebit   = SIG_ENAE_BERLEN_S;
	rw_data[0].enabit = (SIG_ENA_BERLEN_S
		| SIG_ENA_BERSEL_S
		| SIG_ENA_BERLAY_S
		| SIG_ENA_BERST_S);
	rw_data[0].param  = (unsigned short)
		((data->set_len << SIG_ENAS_BERLEN_S)
		| (data->set_sel << SIG_ENAS_BERSEL_S)
		| (SIG_PARAM_BERLAY_S)
		| (SIG_PARAM_BERST_S));

	ret = tuner_write_reg(rw_data, 1, TUNER_SLAVE_ADR_M2);
	if (ret)
		return -EINVAL;

	rw_data[0].adr    = TUNER_ADR_BERFLG_S;
	rw_data[0].sbit   = SIG_ENAS_ALL;
	rw_data[0].ebit   = SIG_ENAE_ALL;
	rw_data[0].enabit = SIG_ENA_ALL;
	rw_data[0].param  = 0x00;

	ret = tuner_read_reg(rw_data, 1, TUNER_SLAVE_ADR_M2);
	if (ret)
		return -EINVAL;

	data->info_i2c
		= (unsigned char) (((rw_data[0].param) & SIG_ENA_BERCHK_S) >>
			SIG_ENAS_BERCHK_S);
	data->info_berrdy
		= (unsigned char) (((rw_data[0].param) & SIG_ENA_BERRDY_S) >>
			SIG_ENAS_BERRDY_S);

	rw_data[0].adr    = TUNER_ADR_BERRDU_S;
	rw_data[0].sbit   = SIG_ENAS_ALL;
	rw_data[0].ebit   = SIG_ENAE_ALL;
	rw_data[0].enabit = SIG_ENA_ALL;
	rw_data[0].param  = 0x00;
	rw_data[1].adr    = TUNER_ADR_BERRDL_S;
	rw_data[1].sbit   = SIG_ENAS_ALL;
	rw_data[1].ebit   = SIG_ENAE_ALL;
	rw_data[1].enabit = SIG_ENA_ALL;
	rw_data[1].param  = 0x00;
	rw_data[2].adr    = TUNER_ADR_BERSET_S;
	rw_data[2].sbit   = SIG_ENAS_ALL;
	rw_data[2].ebit   = SIG_ENAE_ALL;
	rw_data[2].enabit = SIG_ENA_ALL;
	rw_data[2].param  = 0x00;

	ret = tuner_read_reg(rw_data, 3, TUNER_SLAVE_ADR_M2);
	if (ret)
		return -EINVAL;

	berrdu = rw_data[0].param;
	berrdl = rw_data[1].param;
	berrd  = berrdu * TUNER_MONI_MEABIT_UPPSHIFT + berrdl;

	berlend_tmp = ((rw_data[2].param) >> SIG_ENAS_BERLEN_S);
	berlend     = 256;

	if (data->set_sel == TUNER_MONI_BERSET) {
		ber_tmp = (berlend * TUNER_MONI_OPE_BER1 * TUNER_MONI_OPE_BER2);
		ber_long = (TUNER_CONFIG_PRECISION * berrd) / ber_tmp;
		data->info_data = (unsigned int)(ber_long);
	} else if (data->set_sel == TUNER_MONI_PERSET) {
		per_long = (TUNER_CONFIG_PRECISION * berrd) / berlend;
		data->info_data = (unsigned int)(per_long);
	} else {
		return -EINVAL;
	}

	data->info_berrd = (unsigned int)berrd;
	data->info_berlenrd = (unsigned int)berlend;

	return 0;
}

int tuner_moni_easy_bper_fullseg(struct _tune_data_easy_bper *data)
{
	struct _tuner_data_rw rw_data[TUNER_MONI_RDATANUM];
	unsigned char read_adr;
	int i;
	int ret;
	unsigned long berrdu;
	unsigned long berrdm;
	unsigned long berrdl;
	unsigned long berrd;
	unsigned long berlenrdu;
	unsigned long berlenrdl;
	unsigned long berlenrd;
	unsigned long ber_long;
	unsigned long per_long;
	unsigned int ber_tmp;

	if (!data)
		return -EINVAL;

	rw_data[0].adr    = TUNER_ADR_BERSET1_F;
	rw_data[0].sbit   = SIG_ENAS_BERTIM_F;
	rw_data[0].ebit   = SIG_ENAE_BERDSEL_F;
	rw_data[0].enabit = (SIG_ENA_BERDSEL_F
		| SIG_ENA_BERVSEL_F
		| SIG_ENA_BERTIMMD_F
		| SIG_ENA_BERTIM_F);
	rw_data[0].param = (unsigned short)
		(((data->set_rank) << SIG_ENAS_BERDSEL_F)
		| ((data->set_read) << SIG_ENAS_BERVSEL_F)
		| ((data->set_mode) << SIG_ENAS_BERTIMMD_F)
		| ((data->set_time) << SIG_ENAS_BERTIM_F));

	rw_data[1].adr    = TUNER_ADR_BERSET2_F;
	rw_data[1].sbit   = SIG_ENAS_BERVLENA_F;
	rw_data[1].ebit   = SIG_ENAE_BERVLENA_F;
	rw_data[1].enabit = SIG_ENA_BERVLENA_F;
	rw_data[1].param  = data->set_data[0];

	rw_data[2].adr    = TUNER_ADR_BERSET3_F;
	rw_data[2].sbit   = SIG_ENAS_BERVLENC_F;
	rw_data[2].ebit   = SIG_ENAE_BERVLENB_F;
	rw_data[2].enabit = (SIG_ENA_BERVLENB_F | SIG_ENA_BERVLENC_F);
	rw_data[2].param  = (unsigned short)
		((data->set_data[1] << SIG_ENAS_BERVLENB_F)
		| (data->set_data[2] << SIG_ENAS_BERVLENC_F));

	rw_data[3].adr    = TUNER_ADR_BERRDSET_F;
	rw_data[3].sbit   = SIG_ENAS_BERLAY_F;
	rw_data[3].ebit   = SIG_ENAE_BERRDSEL_F;
	rw_data[3].enabit = (SIG_ENA_BERRDSEL_F | SIG_ENA_BERLAY_F);
	rw_data[3].param  = (unsigned short)
		(((data->set_set) << SIG_ENAS_BERRDSEL_F)
		| ((data->set_lay) << SIG_ENAS_BERLAY_F));

	ret = tuner_write_reg(rw_data, 4, TUNER_SLAVE_ADR_M1);
	if (ret)
		return -EINVAL;

	rw_data[0].adr    = TUNER_ADR_BERFLG_F;
	rw_data[0].sbit   = SIG_ENAS_ALL;
	rw_data[0].ebit   = SIG_ENAE_ALL;
	rw_data[0].enabit = SIG_ENA_ALL;
	rw_data[0].param  = 0x00;

	ret = tuner_read_reg(rw_data, 1, TUNER_SLAVE_ADR_M1);
	if (ret)
		return -EINVAL;

	data->info_berd_st
		= (unsigned char)
			(((rw_data[0].param) & SIG_ENA_BERDRDY_F)
			>> SIG_ENAS_BERDRDY_F);
	data->info_read1
		= (unsigned char)
			(((rw_data[0].param) & SIG_ENA_BERDCHK_F)
			>> SIG_ENAS_BERDCHK_F);
	data->info_stm[0]
		= (unsigned char)
			(((rw_data[0].param) & SIG_ENA_BERVRDYA_F)
			>> SIG_ENAS_BERVRDYA_F);
	data->info_stm[1]
		= (unsigned char)
			(((rw_data[0].param) & SIG_ENA_BERVRDYB_F)
			>> SIG_ENAS_BERVRDYB_F);
	data->info_stm[2]
		= (unsigned char)
			(((rw_data[0].param) & SIG_ENA_BERVRDYC_F)
			>> SIG_ENAS_BERVRDYC_F);
	data->info_read2[0]
		= (unsigned char)
			(((rw_data[0].param) & SIG_ENA_BERVCHKA_F)
			>> SIG_ENAS_BERVCHKA_F);
	data->info_read2[1]
		= (unsigned char)
			(((rw_data[0].param) & SIG_ENA_BERVCHKB_F)
			>> SIG_ENAS_BERVCHKB_F);
	data->info_read2[2]
		= (unsigned char)
			(((rw_data[0].param) & SIG_ENA_BERVCHKC_F)
			>> SIG_ENAS_BERVCHKC_F);

	read_adr = TUNER_ADR_BERRDU_F;

	for (i = 0; i < TUNER_MONI_RDATANUM; i++) {
		rw_data[i].adr    = read_adr;
		rw_data[i].sbit   = SIG_ENAS_ALL;
		rw_data[i].ebit   = SIG_ENAE_ALL;
		rw_data[i].enabit = SIG_ENA_ALL;
		rw_data[i].param  = 0x00;

		read_adr++;
		if (read_adr > TUNER_ADR_BERLENRDL_F)
			break;
	}
	ret = tuner_read_reg(
		rw_data,
		(unsigned short)(i + 1),
		TUNER_SLAVE_ADR_M1);
	if (ret)
		return -EINVAL;

	berrdu = (unsigned long)rw_data[0].param;
	berrdm = (unsigned long)rw_data[1].param;
	berrdl = (unsigned long)rw_data[2].param;
	berrd  = (berrdu * TUNER_MONI_ERRBIT_UPPSHIFT)
			+ (berrdm * TUNER_MONI_ERRBIT_MIDSHIFT)
			+ (berrdl);

	berlenrdu = (unsigned long)rw_data[3].param;
	berlenrdl = (unsigned long)rw_data[4].param;
	berlenrd  = (berlenrdu * TUNER_MONI_MEABIT_UPPSHIFT) + berlenrdl;

	if (data->set_read == TUNER_MONI_BERSET) {
		if (berlenrd) {
			ber_tmp = berlenrd *
				TUNER_MONI_OPE_BER1 * TUNER_MONI_OPE_BER2;
			ber_long =
				(TUNER_CONFIG_PRECISION * berrd) / ber_tmp;
		} else {
			ber_long = 0;
		}
		data->info_data = (unsigned int)(ber_long);
	} else if (data->set_read == TUNER_MONI_PERSET) {
		per_long = berlenrd != 0 ?
			(TUNER_CONFIG_PRECISION * berrd) / berlenrd : 0;

		data->info_data = (unsigned int)(per_long);
	} else {
		return -EINVAL;
	}

	data->info_berrd = (unsigned int)berrd;
	data->info_berlenrd = (unsigned int)berlenrd;

	return 0;
}

int tuner_get_rssi(int param, void *data)
{
	struct _tuner_data_rw rw_data[TUNER_MONI_RDATANUM];
	int ret;
	int rssi_value;

	if (!data)
		return -EINVAL;

	tuner_rw_set_data(&rw_data[0], TUNER_ADR_REG0, SIG_ENAS_CLPON,
		SIG_ENAE_CLPON, SIG_ENA_CLPON, 0x00);

	tuner_rw_set_data(&rw_data[1], TUNER_ADR_REG1, SIG_ENAS_RLPON,
		SIG_ENAE_RLPON, SIG_ENA_RLPON, 0x00);

	ret = tuner_read_reg(rw_data, 2, TUNER_SLAVE_ADR_S);
	if (ret)
		return -EINVAL;

	tuner_rw_set_data(&rw_data[0], TUNER_ADR_TNCOP8, SIG_ENAS_RSSI,
		SIG_ENAE_RSSI, SIG_ENA_RSSI, 0x00);

	ret = tuner_read_reg(rw_data, 1, TUNER_SLAVE_ADR_S);
	if (ret)
		return -EINVAL;

	rssi_value = (int)(rw_data[0].param);

	if (rssi_value >= TUNER_MONI_RSSI_CHECK) {
		rssi_value =
			(TUNER_MONI_RSSI_PARAM1 *
			(TUNER_MONI_RSSI_PARAM2 - rssi_value));
	}

	((struct _tune_data_rssi *)data)->info_rssi = rssi_value;

	tuner_rw_set_data(&rw_data[0], TUNER_ADR_REG0, SIG_ENAS_CLPON,
		SIG_ENAE_CLPON, SIG_ENA_CLPON, 0x00);

	tuner_rw_set_data(&rw_data[1], TUNER_ADR_REG1, SIG_ENAS_RLPON,
		SIG_ENAE_RLPON, SIG_ENA_RLPON, 0x00);

	ret = tuner_read_reg(rw_data, 2, TUNER_SLAVE_ADR_S);
	if (ret)
		return -EINVAL;

	return 0;
}

int tuner_read_reg(
	struct _tuner_data_rw *adr_data,
	unsigned short num,
	unsigned short slave_adr)
{
	int     ret;
	unsigned short  i;

	if (!adr_data)
		return -EINVAL;

	for (i = 0; i < num; i++) {
		adr_data[i].slave_adr = slave_adr;
		ret = module_entry_ioctl(TUNER_IOCTL_VALGET, &adr_data[i]);
		if (ret)
			return -EINVAL;
	}
	return 0;
}

int tuner_write_reg(
	struct _tuner_data_rw *adr_data,
	unsigned short num,
	unsigned short slave_adr)
{
	int    ret;
	unsigned short i;

	if (!adr_data)
		return -EINVAL;

	for (i = 0; i < num; i++) {
		adr_data[i].slave_adr = slave_adr;
		ret = module_entry_ioctl(
			TUNER_IOCTL_VALSET,
			&adr_data[i]);
		if (ret)
			return -EINVAL;
	}
	return 0;
}

void tuner_rw_set_data(
	struct _tuner_data_rw *adr_data, unsigned short adr,
	unsigned short sbit, unsigned short ebit,
	unsigned short enabit, unsigned short param)
{
	adr_data->adr    = adr;
	adr_data->sbit   = sbit;
	adr_data->ebit   = ebit;
	adr_data->enabit = enabit;
	adr_data->param  = param;
}

int module_entry_ioctl(unsigned int ucommand, struct _tuner_data_rw *data)
{
	if (!ucommand || !data)
		return -EINVAL;

	return tuner_drv_hw_access(ucommand, data, 1);
}

void tuner_set_status(int status)
{
	ant_swt.status = status;
}

int tuner_get_status(void)
{
	return ant_swt.status;
}

void set_cir_info(int cir)
{
	ant_swt.cir_info = cir;
}

int get_cir_info(void)
{
	return ant_swt.cir_info;
}

void set_current_ant(int ant)
{
	ant_swt.current_ant = ant;
}

int get_current_ant(void)
{
	return ant_swt.current_ant;
}

void auto_ant_search_ena(int on)
{
	ant_swt.enabler = on;
}

int get_auto_ant_search_ena(void)
{
	return ant_swt.enabler;
}

void set_tuning_cnt(int cnt)
{
	ant_swt.tuning_cnt = cnt;
}

int get_tuning_cnt(void)
{
	return ant_swt.tuning_cnt;
}

int tuner_get_sync(int param, int sync_mode)
{
	int ret;
	struct _tuner_data_rw rw_data;
	unsigned short read_reg = 0x00;
	unsigned short comp_reg = 0x00;

	switch (sync_mode) {
	case TUNER_SYNC:
		if (param == TUNER_STATE_ONESEG)
			read_reg = TUNER_ADR_LCKMON2_S;
		else if (param == TUNER_STATE_FULLSEG ||
			param == TUNER_STATE_FULL_ONE_SEG)
			read_reg = TUNER_ADR_SYN2FLG_F;

		comp_reg = TUNER_SYNC_COMP;
		break;
	case TUNER_SYNC_SEQ:
		if (param == TUNER_STATE_ONESEG) {
			read_reg = TUNER_ADR_SSEQRD_S;
			comp_reg = TUNER_SYNC_COMP_SEQ1;
		} else if (param == TUNER_STATE_FULLSEG ||
			param == TUNER_STATE_FULL_ONE_SEG) {
			read_reg = TUNER_ADR_SSEQRD_F;
			comp_reg = TUNER_SYNC_COMP_SEQ2;
		}
		break;
	default:
		return -EINVAL;
	}

	ret = get_sync_register_value(param, &rw_data, read_reg);
	if (ret)
		return ret;

	if (rw_data.param & comp_reg)
		ret = AUTO_ANT_SYNC;
	else
		ret = AUTO_ANT_NOT_SYNC;

	return ret;
}

int get_pkt_err_flag(int param)
{
	int ret;
	struct _tuner_data_rw rw_data;

	ret = get_pkt_err_register_value(param, &rw_data);
	if (ret)
		return ret;

	switch (param) {
	case TUNER_STATE_ONESEG:
		if ((rw_data.param & 0x01) == 0x01)
			ret = AUTO_ANT_PKT_ERR;
		else
			ret = AUTO_ANT_NO_PKT_ERR;
		break;
	case TUNER_STATE_FULLSEG:
	case TUNER_STATE_FULL_ONE_SEG:
		if ((rw_data.param & 0x01) == 0x01)

			ret = AUTO_ANT_PKT_ERR;
		else
			ret = AUTO_ANT_NO_PKT_ERR;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

int get_sync_register_value(
	int param,
	struct _tuner_data_rw *rw_data,
	unsigned short reg)
{
	int ret;
	unsigned short read_core;

	read_core = 0x00;

	if (param == TUNER_STATE_ONESEG)
		read_core = TUNER_SLAVE_ADR_M2;
	else if (param == TUNER_STATE_FULLSEG ||
		param == TUNER_STATE_FULL_ONE_SEG)
		read_core = TUNER_SLAVE_ADR_M1;

	rw_data->adr   = reg;
	rw_data->sbit  = SIG_ENAS_ALL;
	rw_data->ebit  = SIG_ENAE_ALL;
	rw_data->param = SIG_ENAS_ALL;

	ret = tuner_read_reg(rw_data, 1, read_core);
	if (ret)
		ret = -EINVAL;

	return ret;
}

int get_pkt_err_register_value(int param, struct _tuner_data_rw *rw_data)
{
	int ret;

	switch (param) {
	case TUNER_STATE_ONESEG:
		rw_data->adr   = TUNER_ADR_INTST2_S;
		rw_data->sbit  = SIG_ENAS_ALL;
		rw_data->ebit  = SIG_ENAE_ALL;
		rw_data->param = SIG_ENAS_ALL;

		ret = tuner_read_reg(rw_data, 1, TUNER_SLAVE_ADR_M2);
		if (ret)
			ret = -EINVAL;
		break;
	case TUNER_STATE_FULLSEG:
	case TUNER_STATE_FULL_ONE_SEG:
		rw_data->adr   = TUNER_ADR_INTSET1_S;
		rw_data->sbit  = SIG_ENAS_ALL;
		rw_data->ebit  = SIG_ENAE_ALL;
		rw_data->param = SIG_ENAS_ALL;

		ret = tuner_read_reg(rw_data, 1, TUNER_SLAVE_ADR_M1);
		if (ret)
			ret = -EINVAL;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

int tuner_get_per(struct _tune_data_easy_bper *per_monitor)
{
	int ret;
	int read_count;
	int per_timeout_cnt;
	int cir_info;
	unsigned int per_tmp = 0;

	cir_info = get_cir_info();

	if (cir_info == TUNER_STATE_FULLSEG) {
		per_monitor->set_rank = 1;
		per_monitor->set_read = 1;
		per_monitor->set_mode = 1;
		per_monitor->set_time = 5;
		per_monitor->set_mode = 0;
		per_monitor->set_data[F_LYR-1] = 10;
		per_monitor->set_data[0] = 10;
		per_monitor->set_data[1] = 10;
		per_monitor->set_data[2] = 10;
		per_monitor->set_set  = 1;
		per_monitor->set_lay  = F_LYR;
	} else if (cir_info == TUNER_STATE_ONESEG) {
		per_monitor->set_len  = 8;
		per_monitor->set_sel  = 1;
	}

	read_count = 0;

	for (per_timeout_cnt = 0; per_timeout_cnt < 5; per_timeout_cnt++) {
		ret = tuner_get_bper(cir_info, per_monitor);
		if (ret)
			return -EINVAL;

		if ((cir_info == TUNER_STATE_FULLSEG) &&
			(per_monitor->info_stm[F_LYR-1] == 1) &&
			(per_monitor->info_read2[F_LYR-1] == 1)) {
			if (read_count == 1) {
				per_tmp = per_monitor->info_data;
				break;
			} else {
				read_count++;
			}
		} else if ((cir_info == TUNER_STATE_ONESEG) &&
			(per_monitor->info_i2c == 1) &&
			(per_monitor->info_berrdy == 1)) {
			if (read_count == 1) {
				per_tmp = per_monitor->info_berrd;
				break;
			} else {
				read_count++;
			}
		}
		msleep(BPER_POLE_WAIT);
	}
	return per_tmp;
}

int clear_interrupt_mask(
	unsigned char *g_tuner_intcnd_f,
	unsigned char *g_tuner_intcnd_s)
{
	struct _tuner_data_rw rw_data[2];
	int ret;

	rw_data[0].slave_adr = TUNER_SLAVE_ADR_M1;
	rw_data[0].adr       = TUNER_DRV_ADR_INTCND_F;
	rw_data[0].sbit      = TUNER_DRV_ENAS_ALL;
	rw_data[0].ebit      = TUNER_DRV_ENAE_ALL;
	rw_data[0].enabit    = TUNER_DRV_ENA_ALL;
	rw_data[0].param     = TUNER_DRV_PARAM_RINIT;
	rw_data[1].slave_adr = TUNER_SLAVE_ADR_M2;
	rw_data[1].adr       = TUNER_DRV_ADR_INTCND_S;
	rw_data[1].sbit      = TUNER_DRV_ENAS_ALL;
	rw_data[1].ebit      = TUNER_DRV_ENAE_ALL;
	rw_data[1].enabit    = TUNER_DRV_ENA_ALL;
	rw_data[1].param     = TUNER_DRV_PARAM_RINIT;

	ret = tuner_drv_hw_access(TUNER_IOCTL_VALGET, rw_data, 2);
	if (ret) {
		rw_data[0].param = TUNER_DRV_PARAM_ALL;
		rw_data[1].param = TUNER_DRV_PARAM_ALL;
	} else {
		if (!g_tuner_intcnd_f || !g_tuner_intcnd_s)
			return -EINVAL;

		*g_tuner_intcnd_f |= (unsigned char)rw_data[0].param;
		*g_tuner_intcnd_s |= (unsigned char)rw_data[1].param;
	}

	ret = tuner_drv_hw_access(TUNER_IOCTL_VALSET, rw_data, 2);
	if (ret)
		return -EINVAL;

	return ret;
}
