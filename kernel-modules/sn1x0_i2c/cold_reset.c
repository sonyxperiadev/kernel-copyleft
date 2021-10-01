/******************************************************************************
 *  Copyright (C) 2020 NXP
 *   *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 ******************************************************************************/

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/version.h>
#include <linux/fs.h>
#include "cold_reset.h"
#include "pn553.h"

/*ESE_COLD_RESET MACROS */
#define MAX_BUFFER_SIZE 512 /* copied from pn553.c */
#define MSG_NFCC_CMD         0x20
#define NCI_PROP_RST_RSP_SIZE 0x04


/* Evaluates to 1 If cold reset is in progress or the guard timer is still running */
#define IS_COLD_RESET_REQ_IN_PROGRESS(flags)                                         \
               (flags & (MASK_ESE_COLD_RESET | MASK_ESE_COLD_RESET_GUARD_TIMER))

#define IS_RESET_PROTECTION_ENABLED(flags) (flags & RST_PROTECTION_ENABLED)

#define IS_COLD_RESET_ALLOWED(flags, src)  (!IS_COLD_RESET_REQ_IN_PROGRESS(flags)    \
                && (!IS_RESET_PROTECTION_ENABLED(flags) || src == ESE_COLD_RESET_SOURCE_SPI))

#define IS_COLD_RESET_ALLOWED_NFC(flags, src)  ((IS_RESET_PROTECTION_ENABLED(flags) \
      && src == ESE_COLD_RESET_SOURCE_NFC))

static struct pn544_dev *pn544_dev;
struct mutex        ese_cold_reset_sync_mutex;
struct mutex        nci_send_cmd_mutex;
bool is_force_reset_allowed = false;

extern ssize_t pn544_dev_read(struct file *filp, char __user *buf,
        size_t count, loff_t *offset);
int do_reset_protection(bool type);
static int8_t prop_nci_rsp[NCI_PROP_RST_RSP_SIZE];
static struct timer_list ese_cold_reset_timer;
static struct completion prop_cmd_resp_sema;
static struct completion ese_cold_reset_sema;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
static void ese_cold_reset_gaurd_timer_callback(unsigned long data);
#else
static void ese_cold_reset_gaurd_timer_callback(struct timer_list *unused);
#endif
static long start_ese_cold_reset_guard_timer(void);

extern struct pn544_dev * get_nfcc_dev_data(void);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
static void ese_cold_reset_gaurd_timer_callback(unsigned long data)
{
    (void)data;
#else
static void ese_cold_reset_gaurd_timer_callback(struct timer_list *unused)
{
#endif
    pr_info("%s: Enter\n",__func__);
    pn544_dev->state_flags &= ~MASK_ESE_COLD_RESET_GUARD_TIMER;
    return;
}

static long start_ese_cold_reset_guard_timer(void)
{
    long ret = -EINVAL;
    printk( KERN_INFO "starting ese_cold_reset_timer \n");
    if(timer_pending(&ese_cold_reset_timer) == 1)
    {
        pr_info("ese_cold_reset_guard_timer: delete pending timer \n");
        /* delete timer if already pending */
        del_timer(&ese_cold_reset_timer);
    }
    pn544_dev->state_flags |= MASK_ESE_COLD_RESET_GUARD_TIMER;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
    init_timer(&ese_cold_reset_timer);
    setup_timer( &ese_cold_reset_timer, ese_cold_reset_gaurd_timer_callback, 0 );
#else
    timer_setup(&ese_cold_reset_timer, ese_cold_reset_gaurd_timer_callback, 0);
#endif
    ret = mod_timer(&ese_cold_reset_timer, jiffies + msecs_to_jiffies(ESE_COLD_RESET_GUARD_TIME));
    if (ret)
      printk( KERN_INFO "%s: Error in mod_timer\n",__func__);
    return ret;
}

void set_force_reset(bool value) {
    is_force_reset_allowed = value;
}

void ese_reset_resource_init(void) {
    mutex_init(&ese_cold_reset_sync_mutex);
    mutex_init(&nci_send_cmd_mutex);
}
void ese_reset_resource_destroy(void) {
    mutex_destroy(&ese_cold_reset_sync_mutex);
    mutex_destroy(&nci_send_cmd_mutex);
}
void rcv_prop_resp_status(const char * const buf)
{
    int ret = -1;
    char tmp[MAX_BUFFER_SIZE];
    size_t rcount = 0;
    memset(&prop_nci_rsp, 0, sizeof(prop_nci_rsp));
    memcpy(prop_nci_rsp, buf, 3);
    rcount = (size_t)prop_nci_rsp[2];

    /* Read data: No need to wait for the interrupt */
    ret = i2c_master_recv(pn544_dev->client, tmp, rcount);
    if(ret == rcount){
        prop_nci_rsp[3] = tmp[0];
        pr_info("%s NxpNciR : len = 4 > %02X%02X%02X%02X\n", __func__,prop_nci_rsp[0],
                prop_nci_rsp[1],prop_nci_rsp[2],prop_nci_rsp[3]);
    }else{
        pr_err("%s : Failed to receive payload of the cold_rst_cmd\n",__func__);
        prop_nci_rsp[3] = -EIO;
    }
    if(pn544_dev->state_flags &(P544_FLAG_NFC_ON)){
        complete(&prop_cmd_resp_sema);
    }
}

/******************************************************************************
 * Function    : send_nci_transceive
 *
 * Description : Common NCI command send utility function.
 *
 * Parameters  : prop_cmd      : Data to be sent to NFCC
 *               prop_cmd_size : Length of the data to be sent
 *
 * Returns     : 0 (SUCCESS)/ (-1)otherwise

 *****************************************************************************/
static int send_nci_transceive(uint8_t *prop_cmd, size_t prop_cmd_size) {
    int ret = 0;
    unsigned int loop=0x03;
    struct file filp ={{{NULL}}};
    int retry = 1;

    pr_info("%s: Enter", __func__);

    filp.private_data = pn544_dev;
    if(pn544_dev->state_flags & P544_FLAG_FW_DNLD) {
      /* If FW DNLD, Operation is not permitted */
      pr_err("%s : Operation is not permitted during fwdnld\n", __func__);
      return -ECANCELED;
    }
    mutex_lock(&nci_send_cmd_mutex);
    init_completion(&prop_cmd_resp_sema);
    /* write command to I2C line*/
    do {
      ret = i2c_master_send(pn544_dev->client, prop_cmd, prop_cmd_size);
      if (ret == prop_cmd_size) {
        break;
      }
      usleep_range(5000, 6000);
    } while(loop--);
    if(!loop && (ret != prop_cmd_size)) {
      pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
      mutex_unlock(&nci_send_cmd_mutex);
      return -EREMOTEIO;
    }
    ret = 0x00;

    do {
      if(pn544_dev->state_flags & P544_FLAG_NFC_ON)/* NFC_ON */ {
        /* Read is pending from the NFC service which will complete the prop_cmd_resp_sema */
        if(wait_for_completion_timeout(&prop_cmd_resp_sema,
            msecs_to_jiffies(NCI_CMD_RSP_TIMEOUT)) == 0){
          pr_err("%s: Timeout", __func__);
          ret = prop_nci_rsp[3] = -EAGAIN; // Failure case
        }
      } else { /* NFC_OFF */
        /* call the pn544_dev_read() */
        filp.f_flags &= ~O_NONBLOCK;
        ret = pn544_dev_read(&filp, NULL,3, 0);
        if(!ret)
          break;
        usleep_range(3500, 4000);
      }
    } while((retry-- >= 0) && ret == -ERESTARTSYS);

    mutex_unlock(&nci_send_cmd_mutex);
    if(0x00 == ret && prop_nci_rsp[3])
        ret = -1 * prop_nci_rsp[3];
    /* Return the status to the SPI/UWB Driver */
    pr_info("%s: exit, Status:%d", __func__, ret);
    return ret;
}

/******************************************************************************
 * Function    : do_reset_protection
 *
 * Description : It shall be called by SPI driver to enable/disable reset
 *               protection
 *
 * Parameters  : Enable(TRUE)/Disable(FALSE)
 *
 * Returns     :
 *     0           :    OK             < Success case >
 *    -EPERM(-1)   :    REJECTED       < NFCC rejects the cold reset cmd>
 *    -3           :    FAILED         < NFCC responds to cold reset cmd>
 *    -EIO(-5)     :    SYNTAX_ERROR   < NFCC cmd framing is wrong >
 *    -6           :    SEMANTIC_ERROR < NFCC rsp to cold reset cmd >
 *    -9           :    INAVLID_PARAM  < NFCC rsp to cold reset cmd >
 *    -EAGAIN(-11) :    < 1. mod_timer(): temp error during kernel alloc >
 *                      < 2. No rsp received from NFCC for cold reset cmd >
 *    -ENOMEM(-12) :    < mod_timer(): failed to allocate memory >
 *    -EINVAL(-22) :    < 1. cold rst req is received from unknown source >
 *                      < 2. mod_timer(): invalid arg is passed>
 *    -EREMOTEIO(-121): < Reset cmd write failure over I2C >
 *    -ECANCELED(-125): < FW DWNLD is going on so driver canceled operation >
 *    -ERESTARTSYS(-512): < Userspace process is restarted during read operation >
 *****************************************************************************/
int do_reset_protection(bool type) {
    int ret = 0;
    uint8_t prop_cmd[] = {0x2F, 0x1F, 0x01, 0x00};
    pn544_dev = get_nfcc_dev_data();

    pr_info("%s: Enter cmd type: %d", __func__, type);

    prop_cmd[RST_PROTECTION_CMD_INDEX] = (type) ? 1 : 0;

    if(type ) {
      pn544_dev->state_flags |= RST_PROTECTION_ENABLED;
    } else {
      if(!(pn544_dev->state_flags & RST_PROTECTION_ENABLED)) {
        return ret;
      }
    }
    pr_info("%s: NxpNciX: %d > %02X%02X%02X%02X \n", __func__, ret,prop_cmd[0],
             prop_cmd[1],prop_cmd[2],prop_cmd[3]);

    ret = send_nci_transceive(prop_cmd, sizeof(prop_cmd));
    if(ret) {
      pr_err("%s : send_nci_command returned %d\n", __func__, ret);
    }
    if(!type) {
      pn544_dev->state_flags &= ~RST_PROTECTION_ENABLED;
    }
    pr_info("%s: exit, Status:%d state_flag : %x ", __func__, ret,
            pn544_dev->state_flags);
    return ret;
}
EXPORT_SYMBOL(do_reset_protection);

/******************************************************************************
 * Function    : ese_cold_reset
 *
 * Description : It shall be called by NFC/SPI/UWB driver to perform driver to
 *               to driver eSE cold reset.
 *
 * Parameters  : src Source of the cold reset request
 *
 * Returns     :
 *     0           :    OK             < Success case >
 *    -EPERM(-1)   :    REJECTED       < Guard timer running>
 *    -3           :    FAILED         < NFCC responds to cold reset cmd>
 *    -EIO(-5)     :    SYNTAX_ERROR   < NFCC cmd framing is wrong >
 *    -6           :    SEMANTIC_ERROR < NFCC rsp to cold reset cmd >
 *    -9           :    INAVLID_PARAM  < NFCC rsp to cold reset cmd >
 *    -EAGAIN(-11) :    < 1. mod_timer(): temp error during kernel alloc >
 *                      < 2. No rsp received from NFCC for cold reset cmd >
 *    -ENOMEM(-12) :    < mod_timer(): failed to allocate memory >
 *    -EBUSY(-16)  :    < eSE busy, in updater mode>
 *    -EINVAL(-22) :    < 1. cold rst req is received from unknown source >
 *                      < 2. mod_timer(): invalid arg is passed>
 *    -EREMOTEIO(-121): < Reset cmd write failure over I2C >
 *    -ECANCELED(-125): < FW DWNLD is going on so driver canceled operation >
 *    -ERESTARTSYS(-512): < Userspace process is restarted during read operation >
 *****************************************************************************/
int ese_cold_reset(ese_cold_reset_origin_t src)
{
    int ret = 0;
    bool status = false;
    uint8_t ese_cld_reset[] = {0x2F, 0x1E, 0x00};

    pr_info("%s: Enter origin:%d", __func__, src);

    switch(src) {
    case ESE_COLD_RESET_SOURCE_NFC:
    case ESE_COLD_RESET_SOURCE_SPI:
    case ESE_COLD_RESET_SOURCE_UWB:
      break;
    default:
      pr_info("%s: Invalid argument", __func__);
      return -EINVAL;
    }
    pn544_dev = get_nfcc_dev_data();
    mutex_lock(&ese_cold_reset_sync_mutex);
    if(is_force_reset_allowed)
      status = IS_COLD_RESET_ALLOWED_NFC(pn544_dev->state_flags, src);
    else
      status = IS_COLD_RESET_ALLOWED(pn544_dev->state_flags, src);
    if(status) {
      ret = start_ese_cold_reset_guard_timer();
      if(ret) {
        mutex_unlock(&ese_cold_reset_sync_mutex);
        return ret; /* EAGAIN/EINVAL/ENOMEM*/
      }
      pn544_dev->state_flags |= src << ESE_COLD_RESET_ORIGIN_FLAGS_POS;
      init_completion(&ese_cold_reset_sema);
      pr_info("%s: NxpNciX: %d > %02X%02X%02X \n", __func__, ret,ese_cld_reset[0],
              ese_cld_reset[1],ese_cld_reset[2]);
      ret = send_nci_transceive(ese_cld_reset, sizeof(ese_cld_reset));
      if(ret) {
        pn544_dev->state_flags &= ~(MASK_ESE_COLD_RESET | MASK_ESE_COLD_RESET_GUARD_TIMER);
        mutex_unlock(&ese_cold_reset_sync_mutex);
        return ret;
      }
      /* wait for reboot guard timer*/
      if(!ret && wait_for_completion_timeout(&ese_cold_reset_sema,
          msecs_to_jiffies(ESE_COLD_RESET_REBOOT_GUARD_TIME)) == 0){
        pr_info("%s: guard Timeout", __func__);
      }
    } else {
        if(IS_RESET_PROTECTION_ENABLED(pn544_dev->state_flags)) {
          pr_err("%s :  Not allowed resource busy \n", __func__);
          ret = -EBUSY;
        }
        else if(IS_COLD_RESET_REQ_IN_PROGRESS(pn544_dev->state_flags)) {
          pr_err("%s :  Operation not permitted \n", __func__);
          ret = -EPERM;
        }
        else {
          /*No Action required*/
        }
    }
    pn544_dev->state_flags &= ~(src << ESE_COLD_RESET_ORIGIN_FLAGS_POS);
    mutex_unlock(&ese_cold_reset_sync_mutex);

    /* Return the status to the SPI/UWB Driver */
    pr_info("%s:%d exit, Status:%d", __func__, src, ret);
    return ret;
}

EXPORT_SYMBOL(ese_cold_reset);
