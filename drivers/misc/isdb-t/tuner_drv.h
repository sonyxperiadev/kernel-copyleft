/******************************************************************************
 *
 *  file name       : tuner_drv.h
 *  brief note      : Driver Common Header
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
 * HISTORY      : 2011/08/25    K.Kitamura(*)
 *                001 new creation
 ******************************************************************************/
#ifndef _TUNER_DRV_H
#define _TUNER_DRV_H

#include <linux/interrupt.h>
#include "tuner_drv_config.h"
#include "tuner_drv_sys.h"
#include "tuner_api.h"

#define IOCTL_VALGET _IOW(TUNER_IOC_MAGIC, 1, struct ioctl_cmd)
#define IOCTL_VALSET _IOR(TUNER_IOC_MAGIC, 2, struct ioctl_cmd)

/* General-purpose setting value */
#define TUNER_OFF                     0x00       /* Setting OFF */
#define TUNER_ON                      0x01       /* Setting ON  */

/* Number of valid bits */
#define TUNER_CHARNUM_MAX             0x08    /* Number of valid bits of char */

/* Slave address */
#define TUNER_SLAVE_ADR_M1            0x6D       /* Main#1 Full-segment */
#define TUNER_SLAVE_ADR_M2            0x69       /* Main#2 1-segment    */
#define TUNER_SLAVE_ADR_S             0x6B       /* Sub Tuner control   */

/* Register address */
/* Main#1 */
#define TUNER_ADR_INTDEF1_F           0xD3
#define TUNER_ADR_INTDEF2_F           0xD4
/* Main#2 */
#define TUNER_ADR_INTDEF_S            0xC0

/* INTDEF1_F */
#define SIG_ENAS_INTDEF1_F             (0)       /* INTDEF1_F start bit       */
#define SIG_ENAE_INTDEF1_F             (7)       /* INTDEF1_F end bit         */
#define SIG_ENA_INTDEF1_F           (0xFF)       /* INTDEF1_F enable bit      */
#define SIG_ENAIRQ_INTDEF1_NONE     (0x00)       /* INTDEF1_F irq_enable_none */
/* INTDEF2_F */
#define SIG_ENAS_INTDEF2_F             (0)       /* INTDEF2_F start bit       */
#define SIG_ENAE_INTDEF2_F             (3)       /* INTDEF2_F end bit         */
#define SIG_ENA_INTDEF2_F           (0x0F)       /* INTDEF2_F enable bit      */
#define SIG_ENAIRQ_INTDEF2_NONE     (0x00)       /* INTDEF2_F irq_enable_none */
/* INTDEF_S */
#define SIG_ENAS_IERR_S                (0)
#define SIG_ENAE_SEGERRS               (7)
#define SIG_ENA_SEGERRS             (0x80)
#define SIG_ENA_MGERR_S             (0x40)
#define SIG_ENA_SSEQ_S              (0x20)
#define SIG_ENA_CDFLG_S             (0x10)
#define SIG_ENA_ISYNC_S             (0x08)
#define SIG_ENA_TMCCERR_S           (0x04)
#define SIG_ENA_EMGFLG_S            (0x02)
#define SIG_ENA_IERR_S              (0x01)
#define SIG_ENAIRQ_INTDEF_NONE      (0x00)

/* Interrupt register */
#define TUNER_DRV_ADR_INTCND_F        0xDA
#define TUNER_DRV_ADR_INTCND_S        0xC3
#define TUNER_DRV_ENAS_ALL               0
#define TUNER_DRV_ENAE_ALL               7
#define TUNER_DRV_ENA_ALL             0xFF
#define TUNER_DRV_PARAM_RINIT         0x00
#define TUNER_DRV_PARAM_ALL           0xFF

/* Number of transmit data */
#define TUNER_R_MSGLEN                0x01       /* Read data length     */
#define TUNER_W_MSGLEN                0x02       /* Write data length    */
#define TUNER_R_MSGNUM                0x02       /* Number of read data  */
#define TUNER_W_MSGNUM                0x01       /* Number of write data */

/* Burst write setting */
#define TUNER_I2C_WRITE_ALL_NUM     (1000)/* Burst write data threshold       */
#define TUNER_I2C_MSG_DATA_NUM         (2)/* Number of I2C setting data table */

/* Number of tuner event registers */
#define TUNER_EVENT_REGNUM             (3) /* Number of tuner event registers */

/* Kernel thread event flag */
#define TUNER_KTH_NONE          0x00000000/* Kernel thread flag initial value */
#define TUNER_KTH_IRQHANDLER    0x00000001/* Interrupt handler flag           */
#define TUNER_KTH_END           0x80000000/* Kernel thread end flag           */

/* Kernel thread end flag           */
#define TUNER_KTH_ENDFLG_OFF          0x00       /* Kernel thread running    */
#define TUNER_KTH_ENDFLG_ON           0x01       /* Kernel thread terminated */

/* Compare value of ANT sync register */
#define TUNER_SYNC_COMP               0x10
#define TUNER_SYNC_COMP_SEQ1          0x28
#define TUNER_SYNC_COMP_SEQ2          0x0A

/* Auto ant switch states */
enum TNR_STATUS {
	TNR_LOCK,
	TNR_UNLOCK,
	TNR_TIMEOUT
};

/* Auto ant switch thread enabler */
enum THR_ENA {
	THR_WAIT,
	THR_GO
};

extern int tuner_drv_hw_access(unsigned int ucommand,
	struct _tuner_data_rw *data,
	unsigned short param_len);
extern int tunerpm_dev_init(void);
extern void tunerpm_dev_finalize(void);
extern int tuner_drv_ctl_power(int data);
extern void tuner_drv_ant_switch(int antmode);
extern void tuner_set_status(int status);
extern int tuner_get_status(void);
extern void set_cir_info(int cir_info);
extern int get_cir_info(void);
extern void set_current_ant(int ant);
extern int get_current_ant(void);
extern void auto_ant_search_ena(int on);
extern void set_tuning_cnt(int cnt);
extern int get_tuning_cnt(void);
extern int get_auto_ant_search_ena(void);
extern int get_bper(int param, void *data);
extern int ant_switch_thread(void *arg);
extern void ant_switch_thread_activequeue(void);
extern int ant_switch_thread_createthread(void);
extern void ant_switch_thread_destroythread(void);
extern void ant_switch_thread_ena(int on);
extern int tuner_search_ant(void);
extern int tuner_stability(void);
extern int tuner_get_sync(int param, int sync_mode);
extern int get_pkt_err_flag(int param);
extern int clear_interrupt_mask(
	unsigned char *g_tuner_intcnd_f,
	unsigned char *g_tuner_intcnd_s);
extern int tuner_get_per(
	struct _tune_data_easy_bper *per_monitor);
extern int tuner_drv_set_interrupt(void);
extern void tuner_drv_release_interrupt(void);
extern void tuner_drv_enable_interrupt(void);
extern void tuner_drv_disable_interrupt(void);
extern irqreturn_t tuner_interrupt(int irq, void *dev_id);

int tuner_set_ant_get_drssi(int ant_num);
void set_ant(int ant_num);
int tuner_get_rssi(int param, void *data);
int tuner_moni_easy_bper_prmchk(
	int param,
	struct _tune_data_easy_bper *data);
int tuner_moni_easy_bper_1seg(struct _tune_data_easy_bper *data);
int tuner_moni_easy_bper_fullseg(struct _tune_data_easy_bper *data);
int tuner_read_reg(
	struct _tuner_data_rw *adr_data,
	unsigned short num,
	unsigned short slave_adr);
int tuner_write_reg(struct _tuner_data_rw *adr_data,
	unsigned short num,
	unsigned short slave_adr);
void tuner_rw_set_data(struct _tuner_data_rw *adr_data, unsigned short adr,
	unsigned short sbit, unsigned short ebit,
	unsigned short enabit, unsigned short param);
int module_entry_ioctl(unsigned int ucommand,
	struct _tuner_data_rw *data);
int get_sync_register_value(
	int param,
	struct _tuner_data_rw *rw_data, unsigned short reg);
int get_pkt_err_register_value(int param, struct _tuner_data_rw *rw_data);

#endif /* _TUNER_DRV_H */
