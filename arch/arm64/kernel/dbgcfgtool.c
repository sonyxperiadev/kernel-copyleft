/**+===========================================================================
  File: dbgcfgtool.c

  Description:


  Note:


  Author:
        Chance Li Mar-12-2009
-------------------------------------------------------------------------------
** FIHSPEC CONFIDENTIAL
** Copyright(C) 2011-2014 Foxconn International Holdings, Ltd. All rights reserved.
** Copyright(c) 2009 FIHSPEC Corporation. All Rights Reserved.
**^M
** The source code contained or described herein and all documents related
** to the source code (Material) are owned by FIHSPEC Technology Corporation.
** The Material is protected by worldwide copyright and trade secret laws and
** treaty provisions. No part of the Material may be used, copied, reproduced,
** modified, published, uploaded, posted, transmitted, distributed, or disclosed
** in any way without FIHSPEC prior express written permission.
============================================================================+*/

#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include "../../../arch/arm64/include/dbgcfgtool.h" //CORE-KH-DebugToolPorting_forL-00-a
#include <linux/random.h>	//CORE-KH-InitPanicStamp_forHWWD-00-a

static char* write_buf;
#define DBGCFG_COMMAND_BUFFER_SIZE 128
#define TEMP_BUF_SIZE 10

#define DEVICE_NAME "dbgcfgtool"

//CORE-KH-WO_S1BOOT-02-m[
#ifdef CONFIG_FIH_USER_DEBUG_FLAG
unsigned int debug_uartmsg_enable = 1; //default enable
#else
unsigned int debug_uartmsg_enable = 0; 
#endif
unsigned int debug_cpu_usage_enable = 0;
//CORE-KH-WO_S1BOOT-02-m]
unsigned int debug_printf_uartmsg_enable = 0;
unsigned int debug_android_uartmsg_enable = 0;
unsigned int debug_force_trigger_panic_enable = 1; //CORE-KH-WO_S1BOOT-00-m /* Enable by default temporarily */
unsigned int debug_tcxo_sd_during_display_on = 0;
//CORE-KH- unsigned int debug_ramdump_to_sdcard_enable = 1;  //CORE-KH-WO_S1BOOT-00-m /*CORE-HC-RAMDUMP-00+*/
unsigned int debug_ramdump_to_sdcard_enable = 0;  //CORE-KH-DbgCfgTool-TemporaryDisable 
unsigned int debug_sensor_temperature_recording = 0; //CORE-KH-DbgCfgTool-RecordingTemp-00-a
unsigned int debug_dmesg_restrict = 1; //CORE-KH-DbgCfgTool-klogd-00-a

static int __init debug_cmdline_parameter_parser(char *str)
{
    unsigned int option;

    if ((str != NULL) && (*str != '\0')) 
    {
        option = simple_strtoul(str, NULL, 16);
        pr_err("[dbgcfgtool] Parsed option = 0x%x \n", option);
        
        if (option & (1 << DBG_UARTMSG_CFG))
            debug_uartmsg_enable = 1;
//CORE-KH-WO_S1BOOT-01-a[
        else
            debug_uartmsg_enable = 0;
//CORE-KH-WO_S1BOOT-01-a]
        if (option & (1 << DBG_PRINTF_UARTMSG_CFG))
            debug_printf_uartmsg_enable = 1;
        if (option & (1 << DBG_ANDROID_UARTMSG_MAIN_CFG))
            debug_android_uartmsg_enable |= (1 << LOG_ID_MAIN);
        if (option & (1 << DBG_ANDROID_UARTMSG_RADIO_CFG))
            debug_android_uartmsg_enable |= (1 << LOG_ID_RADIO);
        if (option & (1 << DBG_ANDROID_UARTMSG_EVENTS_CFG))
            debug_android_uartmsg_enable |= (1 << LOG_ID_EVENTS);
        if (option & (1 << DBG_ANDROID_UARTMSG_SYSTEM_CFG))
            debug_android_uartmsg_enable |= (1 << LOG_ID_SYSTEM);
        if (option & (1 << DBG_FORCE_TRIGGER_PANIC_CFG))
            debug_force_trigger_panic_enable = 1;
        else
        	debug_force_trigger_panic_enable = 0; //CORE-KH-WO_S1BOOT-00-a
        /*CORE-HC-RAMDUMP-00+[*/
        if (option & (1 << DBG_RAMDUMP_TO_SDCARD_CFG))
            debug_ramdump_to_sdcard_enable = 1;
        else
        	debug_ramdump_to_sdcard_enable = 0;	//CORE-KH-WO_S1BOOT-00-a
        /*CORE-HC-RAMDUMP-00+]*/
    }
    return 0;
}

early_param("oemandroidboot.babeee48", debug_cmdline_parameter_parser); //CORE-KH-DbgCfgTool_MiscTA_forL-00-m


static void getNextWord(char** buf, char* result, int max)
{
    char * cur = *buf;
    int i = 0;
    while(i < max) 
    {
        if((cur[i] == ' ') || (cur[i] == '\0') || (cur[i]== '\n')) 
        {
            break;
        }
        result[i] = cur[i];
        i++;
    }
    result[i] = '\0';
    *buf = &(cur[i]) + 1;
}

long dbgcfgtool_ioctl(struct file *filp, unsigned int cmd, unsigned long arg){
    int ret = 0;
    unsigned int* pTargetVar = NULL;
    dbgcfg_ioctl_arg DbgArg,*pDbgArg = (dbgcfg_ioctl_arg *)arg;

    switch( cmd )
    {
        case DBG_IOCTL_CMD_HANDLE_DBGCFG:
            if (copy_from_user(&DbgArg, (void*)pDbgArg, sizeof(dbgcfg_ioctl_arg)))
            {
                ret = -EINVAL;
                pr_err("[dbgcfgtool] %s() LINE:%d, copy_from_user failed in DBG_IOCTL_CMD_HANDLE_DBGCFG.(ret: %d)\n", __func__, __LINE__, ret);
            }

            switch (DbgArg.id)
            {
                case DBG_UARTMSG_CFG:
                    pTargetVar = &debug_uartmsg_enable;
                    break;
                case DBG_PRINTF_UARTMSG_CFG:
                    pTargetVar = &debug_printf_uartmsg_enable;
                    break;
                case DBG_ANDROID_UARTMSG_CFG:
                    pTargetVar = &debug_android_uartmsg_enable;
                    break;
                case DBG_FORCE_TRIGGER_PANIC_CFG:
                    pTargetVar = &debug_force_trigger_panic_enable;
                    break;
                /*CORE-HC-RAMDUMP-00+[*/
                case DBG_RAMDUMP_TO_SDCARD_CFG:
                    pTargetVar = &debug_ramdump_to_sdcard_enable;
                    break;
                /*CORE-HC-RAMDUMP-00+]*/
                case DBG_CPU_USAGE_CFG:
                    pTargetVar = &debug_cpu_usage_enable;
                    break;
                case DBG_TCXOSD_DISPLAY_ON_CFG:
                    pTargetVar = &debug_tcxo_sd_during_display_on;
                    break;
                //CORE-KH-DbgCfgTool-RecordingTemp-00-a[
                case DBG_SENSOR_TEMP_RECORDING_CFG:
                    pTargetVar = &debug_sensor_temperature_recording;
                    break;
                //CORE-KH-DbgCfgTool-RecordingTemp-00-a]
                //CORE-KH-DbgCfgTool-klogd-00-a[
                case DBG_DMESG_RESTRICT_CFG:
                    pTargetVar = &debug_dmesg_restrict;
                    break;
                //CORE-KH-DbgCfgTool-klogd-00-a]
                
                default:
                    pr_err("[dbgcfgtool] %s() LINE:%d, Unsupported dbgcfg_id in DBG_IOCTL_CMD_HANDLE_DBGCFG.(ret: %d)\n", __func__, __LINE__, ret);
                    return -EINVAL;  /* CORE-HC-Fix_Coverity-00* */
            }

            if (DbgArg.action == DBGCFG_READ)
            {
                DbgArg.value = *pTargetVar;
            }
            else if (DbgArg.action == DBGCFG_WRITE)
            {
                *pTargetVar = DbgArg.value;
            }
            else
            {
                ret = -EINVAL;
                pr_err("[dbgcfgtool] %s() LINE:%d, Wrong action in DBG_IOCTL_CMD_HANDLE_DBGCFG.(ret: %d)\n", __func__, __LINE__, ret);
            }
            
            if (copy_to_user(pDbgArg, &DbgArg, sizeof(dbgcfg_ioctl_arg)))
            {
                ret = -EINVAL;
                pr_err("[dbgcfgtool] %s() LINE:%d, copy_to_user failed in DBG_IOCTL_CMD_HANDLE_DBGCFG.(ret: %d)\n", __func__, __LINE__, ret);
            }
            break;

        default:
            pr_err("[dbgcfgtool_ioctl] Unknown IOCTL(Line: %d).\n", __LINE__);
            ret = -EINVAL;
    }

    return ret;
}

static ssize_t dbgcfgtool_write(struct file *fp, const char __user *buf,
			 size_t count, loff_t *pos)
{
    char* buf_idx;
    char szCommand[TEMP_BUF_SIZE * 3];
    char szMaskName[TEMP_BUF_SIZE];
    int mask;
    
    if(count > (DBGCFG_COMMAND_BUFFER_SIZE - 1))
    {
        return -EFAULT;
    }

    write_buf = kmalloc(DBGCFG_COMMAND_BUFFER_SIZE, GFP_KERNEL);
    if(write_buf)
    {
        memset(write_buf, 0, DBGCFG_COMMAND_BUFFER_SIZE);
    }
    else
    {
        return 0;
    }

    if(copy_from_user(write_buf, buf, count))
    {
        return -EFAULT;
    }
	
    write_buf[count] = '\0';
    pr_debug(KERN_INFO "dbgcfgtool_write: write_buf=%s=\n", write_buf);	
	
    buf_idx = write_buf;
    getNextWord(&buf_idx, szCommand, 30);

    if(!strncmp(szCommand, "HANG_LINUX", DBGCFG_COMMAND_BUFFER_SIZE))
    {
        pr_info("dbgcfgtool_write: hang linux\n");
        panic("dbgcfgtool");
    }
    else if(!strncmp(szCommand, "DALVIK_HEAP_DUMP", DBGCFG_COMMAND_BUFFER_SIZE))
    {
        getNextWord(&buf_idx, szMaskName, TEMP_BUF_SIZE);  //this is pid value.
        pr_info("dbgcfgtool_write: %s %s\n", szCommand, szMaskName);
        mask  = (int)simple_strtol(szMaskName, NULL, TEMP_BUF_SIZE);
        pr_info("dbgcfgtool_write: pid=%d sig=%d\n", mask, SIGUSR1);
        kill_proc_info(SIGUSR1, SEND_SIG_PRIV, mask);
    }

    *pos += count;

    if(write_buf)
    {
        kfree(write_buf);
        write_buf = NULL;
    }

    return count;
}


static int dbgcfgtool_open(struct inode *ip, struct file *fp)
{
    return 0;
}

static int dbgcfgtool_release(struct inode *ip, struct file *fp)
{
    return 0;
}

static struct file_operations dbgcfgtool_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = dbgcfgtool_ioctl,
    .write = dbgcfgtool_write,
    .open = dbgcfgtool_open,
    .release = dbgcfgtool_release,
};

static struct miscdevice dbgcfgtool_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "dbgcfgtool",
    .fops = &dbgcfgtool_fops,
};

/* CORE-TH-GetBufferFirst-00+[ */
void * get_alog_buffer_virt_addr(void);
void * get_timestamp_buffer_virt_addr(void);
void * get_productinfo_virt_addr(void);
/* CORE-TH-GetBufferFirst-00+] */

static int __init dbgcfgtool_init(void)
{

    int ret = 0;
	/* CORE-TH-GetBufferFirst-00+[ */
	void *crash_timestamp_buffer_virt_addr = 0;
	void *fih_panic_ram_data_ptr = 0;
	void *productinfo_virt_addr = 0;
	/* CORE-TH-GetBufferFirst-00+] */

//CORE-KH-InitPanicStamp_forHWWD-00-a[
	char Randombuf[15];
	u64	random_tick = 0;
//CORE-KH-InitPanicStamp_forHWWD-00-a]
	ret = misc_register(&dbgcfgtool_dev);

	/* CORE-TH-GetBufferFirst-00+[ */
	fih_panic_ram_data_ptr = get_alog_buffer_virt_addr();
	if (fih_panic_ram_data_ptr == NULL)
		pr_err("[dbgcfgtool] fih_panic_ram_data_ptr Null\n");

	crash_timestamp_buffer_virt_addr = get_timestamp_buffer_virt_addr();
	if (crash_timestamp_buffer_virt_addr == NULL)
		pr_err("[dbgcfgtool] crash_timestamp_buffer_virt_addr Null\n");
	/* CORE-TH-GetBufferFirst-00+] */
//CORE-KH-InitPanicStamp_forHWWD-00-a[
	else{
		get_random_bytes_arch(&random_tick, sizeof(random_tick));
		snprintf(Randombuf, sizeof(Randombuf), "99%llu", random_tick);
		memcpy(crash_timestamp_buffer_virt_addr, Randombuf, sizeof(Randombuf));
		printk(KERN_EMERG "Init Crash temp stamp for panic: %s\n", Randombuf);
	}
//CORE-KH-InitPanicStamp_forHWWD-00-a]
	/* CORE-HC-productinfo-00+[ */
	productinfo_virt_addr = get_productinfo_virt_addr();
	if (productinfo_virt_addr == NULL)
		pr_err("[dbgcfgtool] productinfo_virt_addr Null\n");
	/* CORE-HC-productinfo-00+] */

    if ( ret )
    {
        pr_err("Cannot register miscdev on minor(MISC_DYNAMIC_MINOR)=%d (err=%d)\n", MISC_DYNAMIC_MINOR, ret);
    }

    return ret;
}

module_init(dbgcfgtool_init);

static void __exit dbgcfgtool_deinit( void )
{
    misc_deregister(&dbgcfgtool_dev);
}

module_exit(dbgcfgtool_deinit);

MODULE_DESCRIPTION("dbgcfgtool");
MODULE_LICENSE("GPL v2");
