/**+===========================================================================
  File: alog_ram_console.c

  Description:


  Note:


  Author:
        Bryan Hsieh May-30-2010
-------------------------------------------------------------------------------
** FIHSPEC CONFIDENTIAL
** Copyright(C) 2011-2013 Foxconn International Holdings, Ltd. All rights reserved.
** Copyright(c) 2009 FIHSPEC Corporation. All Rights Reserved.
**^M
** The source code contained or described herein and all documents related
** to the source code (Material) are owned by FIHSPEC Technology Corporation.
** The Material is protected by worldwide copyright and trade secret laws and
** treaty provisions. No part of the Material may be used, copied, reproduced,
** modified, published, uploaded, posted, transmitted, distributed, or disclosed
** in any way without FIHSPEC prior express written permission.
============================================================================+*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <asm/io.h>
#include <linux/ctype.h>
#include <linux/time.h>
#include <linux/kobject.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <asm/current.h>
//CORE-KH-DebugToolPorting-00-a[
#include <linux/slab.h>
#include "../../../fs/proc/internal.h"
//CORE-KH-DebugToolPorting-00-a]
#include <linux/vmalloc.h> //CORE-BH-ParseRamdumpLastAlog-01+
//CORE-BH-LastAlogL64+[
#include "alog_ram_console.h"
#include "logger.h"
//CORE-BH-LastAlogL64+]

//MTD-kernel-BH-ParseRamdumpLastAlog-00+[
#define DRIVER_DESC    "alog ram console driver"
#define DRIVER_VERSION "0.2"

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(DRIVER_VERSION);
//MTD-kernel-BH-ParseRamdumpLastAlog-00+]

spinlock_t		bufflock;	/* spinlock for buffer */

/*
 * Android log priority values, in ascending priority order.
 */
typedef enum android_LogPriority {
    ANDROID_LOG_UNKNOWN = 0,
    ANDROID_LOG_DEFAULT,    /* only for SetMinPriority() */
    ANDROID_LOG_VERBOSE,
    ANDROID_LOG_DEBUG,
    ANDROID_LOG_INFO,
    ANDROID_LOG_WARN,
    ANDROID_LOG_ERROR,
    ANDROID_LOG_FATAL,
    ANDROID_LOG_SILENT,     /* only for SetMinPriority(); must be last */
} android_LogPriority;

typedef struct LoggerEntry_t {
	__u16		len;	/* length of the payload */
	__u16		__pad;	/* no matter what, we get 2 bytes of padding */
	__s32		pid;	/* generating process's pid */
	__s32		tid;	/* generating process's tid */
	__s32		sec;	/* seconds since Epoch */
	__s32		nsec;	/* nanoseconds */
	char		msg[0];	/* the entry's payload */
} LoggerEntry;

char gTempBuff[LOGGER_ENTRY_MAX_PAYLOAD + sizeof(LoggerEntry)]; //CORE-BH-LastAlogL64+

#define ALOG_RAM_CONSOLE_MAIN_SIG (0x4E49414D) /* MAIN */
#define ALOG_RAM_CONSOLE_RADIO_SIG (0x49444152) /* RADI */
#define ALOG_RAM_CONSOLE_EVENTS_SIG (0x4E455645) /* EVEN */
#define ALOG_RAM_CONSOLE_SYSTEM_SIG (0x54535953) /* SYST */	
#define ALOG_RAM_CONSOLE_CRASH_SIG (0x53415243) /* CRAS */	//CORE-KH-DbgCfgTool-LogcatCrashCmd-00-a

typedef struct alog_ram_console_buffer_t {
	uint32_t    sig;
	uint32_t    start;
	uint32_t    size;
	uint32_t    reader_pos;
	uint8_t     data[0];
}AlogRamConsoleBuffer;

static char *alog_ram_console_old_log[LOG_TYPE_NUM];
static uint32_t alog_ram_console_old_log_size[LOG_TYPE_NUM];

static AlogRamConsoleBuffer *alog_ram_console_buffer[LOG_TYPE_NUM]; /* alog ram console structure identity */
//MTD-kernel-BH-ParseRamdumpLastAlog-00+[
static AlogRamConsoleBuffer *alog_ram_console_buffer_ext =  NULL; //CORE-BH-ParseRamdumpLastAlog-01*
struct proc_dir_entry *lastalog_entry[LOG_TYPE_NUM] =  {NULL,NULL,NULL,NULL,NULL}; //CORE-KH-DbgCfgTool-LogcatCrashCmd-00-m
//MTD-kernel-BH-ParseRamdumpLastAlog-00+]
static int need_alloc_ext_buffer = 1; //CORE-BH-ParseRamdumpLastAlog-01+
static uint32_t alog_ram_console_payload_size[LOG_TYPE_NUM];           /* maximum payload size of allog ram console buffer */
static uint32_t alog_ram_console_signature[LOG_TYPE_NUM] = {        /* alog ram console signature */
	ALOG_RAM_CONSOLE_MAIN_SIG,
	ALOG_RAM_CONSOLE_RADIO_SIG,
	ALOG_RAM_CONSOLE_EVENTS_SIG,
	ALOG_RAM_CONSOLE_SYSTEM_SIG,
	ALOG_RAM_CONSOLE_CRASH_SIG	//CORE-KH-DbgCfgTool-LogcatCrashCmd-00-a
};
static char* alog_ram_console_proc_fn[LOG_TYPE_NUM] = {  /* alog ram console proc file name */
	"last_alog_main",
	"last_alog_radio",
	"last_alog_events",
	"last_alog_system",
	"last_alog_crash"	//CORE-KH-DbgCfgTool-LogcatCrashCmd-00-a
};
//MTD-kernel-BH-ParseRamdumpLastAlog-00+[
static char* alog_ram_console_ext_proc_fn[LOG_TYPE_NUM] = {  /* alog ram console proc file name */
	"last_alog_main_ext",
	"last_alog_radio_ext",
	"last_alog_events_ext",
	"last_alog_system_ext",
	"last_alog_crash_ext"	//CORE-KH-DbgCfgTool-LogcatCrashCmd-00-a
};
//MTD-kernel-BH-ParseRamdumpLastAlog-00+]
static char* alog_ram_console_res_name[LOG_TYPE_NUM] = { /* alog ram console resource name */
	"alog_main_buffer",
	"alog_radio_buffer",
	"alog_events_buffer",
	"alog_system_buffer",
	"alog_crash_buffer"	//CORE-KH-DbgCfgTool-LogcatCrashCmd-00-a
};
	
static int need_fix_reader[LOG_TYPE_NUM];
static char *  next_log_entry[LOG_TYPE_NUM];

/*CAUTION!!!this function have to be called within a block of logger buffer lock, otherwise it might disorder log entry. */
void alog_ram_console_sync_time(const LogType log_type, const SyncType sync_type)
{
	struct timespec now;
	LoggerEntry header;
	char tbuf[50] = {0};/*FIH-KERNEL-SC-fix_coverity-issues-03*[*/
	unsigned tlen;
	unsigned long long t;
	unsigned long nanosec_rem;
	unsigned int prio = ANDROID_LOG_INFO;	
	int i;
			
	header.pid = current->tgid;
	header.tid = current->pid;

	now = current_kernel_time();
	header.sec = now.tv_sec;
	header.nsec = now.tv_nsec;

	t = cpu_clock(raw_smp_processor_id()); /* CORE-HC-fix_warning_message-00* */
	nanosec_rem = do_div(t, 1000000000);
	tlen = snprintf(tbuf, sizeof(tbuf), "[%5lu.%06lu] ", (unsigned long) t, (nanosec_rem / 1000));/*FIH-KERNEL-SC-fix_coverity-issues-03*[*/
	
	header.len = tlen + 8;

	if (log_type == LOG_TYPE_ALL)
	{
		if (sync_type == SYNC_AFTER)
			pr_info("[LastAlog] alog_ram_console_sync_time %s for all log_type - After\n", tbuf);
		else
			pr_info("[LastAlog] alog_ram_console_sync_time %s for all log_type - Before\n", tbuf);
	
		for (i =LOG_TYPE_MAIN; i<LOG_TYPE_NUM; i++)
		{
			alog_ram_console_write_log(i, NULL, (char *)&header, (int)sizeof(LoggerEntry)); //FIH-SW3-KERNEL-BH-last_alog-01*
			alog_ram_console_write_log(i, NULL, (char *)&prio, 1); //FIH-SW3-KERNEL-BH-last_alog-01*
			if (sync_type == SYNC_AFTER)
				alog_ram_console_write_log(i, NULL, "SYNCA", 6); //FIH-SW3-KERNEL-BH-last_alog-01*
			else
				alog_ram_console_write_log(i, NULL, "SYNCB", 6); //FIH-SW3-KERNEL-BH-last_alog-01*
			alog_ram_console_write_log(i, NULL, tbuf, tlen+1); //FIH-SW3-KERNEL-BH-last_alog-01*
		}
	}
	else
	{
		alog_ram_console_write_log(log_type, NULL, (char *)&header, (int)sizeof(LoggerEntry)); //FIH-SW3-KERNEL-BH-last_alog-01*
		alog_ram_console_write_log(log_type, NULL, (char *)&prio, 1); //FIH-SW3-KERNEL-BH-last_alog-01*
		if (sync_type == SYNC_AFTER)
		{
			pr_info("[LastAlog] alog_ram_console_sync_time %s for log_type:%d - After\n", tbuf, log_type);
			alog_ram_console_write_log(log_type, NULL, "SYNCA", 6); //FIH-SW3-KERNEL-BH-last_alog-01*
		}
		else
		{
			pr_info("[LastAlog] alog_ram_console_sync_time %s for log_type:%d - Before\n", tbuf, log_type);
			alog_ram_console_write_log(log_type, NULL, "SYNCB", 6); //FIH-SW3-KERNEL-BH-last_alog-01*
		}
		alog_ram_console_write_log(log_type, NULL, tbuf, tlen+1); //FIH-SW3-KERNEL-BH-last_alog-01*
	}
}	

/* there is no spinlock inside this function, please take care the lock mechanism outside this function. */
int alog_ram_console_write_log(const LogType log_type, const void __user *user_buf, char *kernel_buf, int count){  //FIH-SW3-KERNEL-BH-last_alog-01*
	int rem,next_entry_len,skip_len,cut_len;
	AlogRamConsoleBuffer *buffer = alog_ram_console_buffer[log_type];
	char *temp_entry;
	char tempbuffer[20];
	int overrun = 0;
	char *s; //MTD-KERNEL-BH-last_alog-01+
	char *orig_s;

	//pr_err("[LastAlog] write_log: type:%d start:%d size:%d rem:%d count:%d", log_type, buffer->start, buffer->size, alog_ram_console_payload_size[log_type] - buffer->start, count); //debug-temp+
	//MTD-KERNEL-BH-last_alog-01+[
	if (user_buf && kernel_buf)
	{
		pr_err("[LastAlog] user_buf and kernel_buf must be exclusive!\n");
		return 0;
	}
	else if (user_buf)
	{
		orig_s = s = (char *)user_buf;
	}
	else if (kernel_buf)
	{
		orig_s = s = kernel_buf;
	}
	else
	{
		pr_err("[LastAlog] user_buf and kernel_buf both are null!\n");
		return 0;	
	}
	//MTD-KERNEL-BH-last_alog-01+]
	rem = alog_ram_console_payload_size[log_type] - buffer->start;

	/* if buffer remain length < message length, update ram console by remain part, then restart pointer to 0 */
	if (rem < count) 
	{
		s += rem;
		buffer->size = alog_ram_console_payload_size[log_type];
		need_fix_reader[log_type] = 1; /* That means the reader positon must be fixed once overrun happens */
		overrun = 1; /* overrun flag, default value is 0 in every log written */
	}

	/* fixe up the reader position to get the next readable entry when this data will overwrite next log entry */
	if (need_fix_reader[log_type])
	{
		/* init skip len between reader_pos and start */
		if (buffer->reader_pos >=  buffer->start)
			skip_len=buffer->reader_pos - buffer->start; 
		else
			skip_len=buffer->size - buffer->start + buffer->reader_pos;
		
		if (count > skip_len)
		{		
			/* fix reader position for overwite case */
			do
			{
				/* get next entry hader info */
				if ((unsigned long)next_log_entry[log_type] + sizeof(LoggerEntry) > (unsigned long)buffer->data + buffer->size)
				{	
					/* if entry header is over run, adjust it to get correct entry header */
					cut_len = (unsigned long)buffer->data + buffer->size-(unsigned long)next_log_entry[log_type];
					memcpy_fromio(tempbuffer, next_log_entry[log_type], cut_len);
					memcpy_fromio(tempbuffer+(unsigned long)buffer->data + buffer->size-(unsigned long)next_log_entry[log_type], buffer->data, sizeof(LoggerEntry) - cut_len);
					temp_entry = tempbuffer;
				}
				else
					temp_entry = next_log_entry[log_type];

 				next_entry_len = ((int)temp_entry[1]<<8) + temp_entry[0] + sizeof(LoggerEntry);
				
				/* find the first readable entry header after this log write */
				skip_len += next_entry_len;
				/* go to next log entry */
				if ((unsigned long)next_log_entry[log_type] + next_entry_len > (unsigned long)buffer->data + buffer->size)
				{
					/* if next log entry is over run, adjust it to turm around to begin */
					next_log_entry[log_type] = buffer->data + (next_entry_len - (buffer->size - ((unsigned long)next_log_entry[log_type] - (unsigned long)buffer->data)));
				}
				else
					next_log_entry[log_type] = next_log_entry[log_type] + next_entry_len;
					
			} while (skip_len < count);

			buffer->reader_pos = next_log_entry[log_type] - (char *)buffer->data;
		}
	}
	else
		buffer->reader_pos = buffer->start + count;	
	
	/* if overrun happens, copy the preceding string to start position */
	if (overrun)
	{
	  	//MTD-KERNEL-BH-last_alog-01*[
	  	if (user_buf)
	  	{
			//CORE-BH-LastAlogL64*[
			if (copy_from_user(gTempBuff, (const void __user *)orig_s, rem))
			{
				memset_io(buffer->data + buffer->start,0,rem);		
				pr_err("[LastAlog] copy from user failed!\n");
			}
			memcpy_toio(buffer->data + buffer->start, gTempBuff, rem);
			//CORE-BH-LastAlogL64*]
	  	}
		else
			memcpy_toio(buffer->data + buffer->start, orig_s, rem);
	    //MTD-KERNEL-BH-last_alog-01*]
		buffer->start = 0;
		count -= rem;
	}
	/*  copy whole or rest string to start position or 0 (overrun) */
	//MTD-KERNEL-BH-last_alog-01*[
	if (user_buf)
	{
//CORE-BH-LastAlogL64*[
		if (copy_from_user(gTempBuff, (const void __user *)s, count))
		{
			memset_io(buffer->data + buffer->start,0,count);			
			pr_err("[LastAlog] copy from user failed!\n");
		}
		memcpy_toio(buffer->data + buffer->start, gTempBuff, count);
//CORE-BH-LastAlogL64*]
	}
	else
		memcpy_toio(buffer->data + buffer->start, s, count);
    //MTD-KERNEL-BH-last_alog-01*]
	buffer->start += count; /* update start pointer */
	if (buffer->size < alog_ram_console_payload_size[log_type])
		buffer->size += count; /* update used buffer size */

	return overrun;
}

static int alog_ram_console_save_old(const LogType log_type, char *dest, int isExt) //CORE-BH-ParseRamdumpLastAlog-01*
{
	//MTD-kernel-BH-ParseRamdumpLastAlog-00*[
	uint32_t old_log_size;
	int reader_pos;
	int start_pos;
	AlogRamConsoleBuffer *src;

	if (isExt)
		src = alog_ram_console_buffer_ext;  //CORE-BH-ParseRamdumpLastAlog-01*
	else
		src = alog_ram_console_buffer[log_type];
	
	//CORE-BH-ParseRamdumpLastAlog-01+[
	if (src == NULL)
	{
		pr_err("[LastAlog] save_old: alog ram console buffer is NULL for [%d]\n", log_type);
		return -EFAULT;
	}
	//CORE-BH-ParseRamdumpLastAlog-01+]
	
	old_log_size = src->size;
	reader_pos = src->reader_pos;
	start_pos = src->start;
	//MTD-kernel-BH-ParseRamdumpLastAlog-00*]
	 
	if (dest == NULL) {
		dest = vmalloc(old_log_size); //MTD-kernel-BH-ParseRamdumpLastAlog-01*
		if (dest == NULL) {
			pr_err("[LastAlog] save_old: failed to allocate old log buffer, old_log_size:%d\n", old_log_size);
			return -ENOMEM; //CORE-BH-ParseRamdumpLastAlog-01*
		}
	}

	/* copy old log by sequence */
	alog_ram_console_old_log[log_type] = dest;
	//CORE-BH-ParseRamdumpLastAlog-01*[
	if (old_log_size < alog_ram_console_payload_size[log_type])
	{
		memcpy_fromio(dest, src->data, old_log_size);
		alog_ram_console_old_log_size[log_type]   = old_log_size;
	}
	else
	{
		//CORE-BH-LastAlog-01+[
		if (start_pos >= reader_pos)
		{
			if ((start_pos - reader_pos) > old_log_size)
			{
				pr_info("[LastAlog] save_old: invalid len, old_log_size:%d reader_pos:%d start_pos:%d", old_log_size, reader_pos, start_pos);
				vfree(alog_ram_console_old_log[log_type]);
				alog_ram_console_old_log[log_type] = NULL;
				alog_ram_console_old_log_size[log_type] = 0;
			}
			else
			{
				memcpy_fromio(alog_ram_console_old_log[log_type], &src->data[reader_pos], start_pos - reader_pos);
				alog_ram_console_old_log_size[log_type] = start_pos - reader_pos ;
			}
		}
		else
		{
			if ((old_log_size - reader_pos + start_pos) > old_log_size)
			{
				pr_info("[LastAlog] save_old: invalid len old_log_size:%d reader_pos:%d start_pos:%d", old_log_size, reader_pos, start_pos);
				vfree(alog_ram_console_old_log[log_type]);
				alog_ram_console_old_log[log_type] = NULL;
				alog_ram_console_old_log_size[log_type] = 0;
			}
			else
			{
		//CORE-BH-LastAlog-01+]
				memcpy_fromio(alog_ram_console_old_log[log_type], &src->data[reader_pos], old_log_size - reader_pos);
				memcpy_fromio(alog_ram_console_old_log[log_type] + old_log_size - reader_pos, src->data, start_pos);
				alog_ram_console_old_log_size[log_type] = old_log_size - reader_pos + start_pos;
			}
		}
	}
	//CORE-BH-ParseRamdumpLastAlog-01*]
	//CORE-BH-ParseRamdumpLastAlog-01+[
	if (isExt && alog_ram_console_buffer_ext)
	{
		vfree(alog_ram_console_buffer_ext); //need to free alog ram console buffer for external usage after saving old log
		alog_ram_console_buffer_ext = NULL;
		need_alloc_ext_buffer = 1;
	}
	pr_info("[LastAlog] save_old: alog_ram_console_old_log[%d]=0x%lx alog_ram_console_old_log_size[%d]=%d reader_pos=%d start_pos=%d buffer_size=%d\n", log_type, (unsigned long)dest, log_type, alog_ram_console_old_log_size[log_type], reader_pos, start_pos, old_log_size);
	return 0;
	//CORE-BH-ParseRamdumpLastAlog-01+]
}

static int __init alog_ram_console_init(const LogType log_type, AlogRamConsoleBuffer *buffer, uint32_t buffer_size, char *old_buf)
{
	int ret; //CORE-BH-ParseRamdumpLastAlog-01+
	alog_ram_console_buffer[log_type] = buffer;

	/* set maximum of  usable alog ram console buffer size */
	//MTD-kernel-BH-ParseRamdumpLastAlog-00*[
	alog_ram_console_payload_size[log_type] = buffer_size - sizeof(AlogRamConsoleBuffer) ;
	pr_info("[LastAlog] init for log type:%d alog_ram_console_buffer:0x%lx alog_ram_console_payload_size:%d(0x%x) buffer->data:0x%lx\n", log_type, (unsigned long)alog_ram_console_buffer[log_type], alog_ram_console_payload_size[log_type], alog_ram_console_payload_size[log_type], (unsigned long)buffer->data); //MTD-kernel-BH-ParseRamdumpLastAlog-00*

	/* if alog ram console has correct signature, that means there are logs in ram console */
	if (buffer->sig == alog_ram_console_signature[log_type]) 
	{
		/*CORE-TH-AlogRamConsoleBufferSize-00+*/
		if ((buffer->size > alog_ram_console_payload_size[log_type]) || 
			(buffer->start > buffer->size) ||
			(buffer->reader_pos) > (buffer->size))
			pr_info("[LastAlog] init for log type:%d found existing invalid buffer, size %d, start %d\n", log_type, buffer->size, buffer->start);
		else
		{
			pr_info("[LastAlog] init for log type:%d found existing buffer, size %d, start %d, reader_pos %d\n", log_type, buffer->size, buffer->start, buffer->reader_pos);
			ret = alog_ram_console_save_old(log_type, old_buf,0); /* save old log */ //CORE-BH-ParseRamdumpLastAlog-01*
			//CORE-BH-ParseRamdumpLastAlog-01+[
			if (ret)
				return ret;
			//CORE-BH-ParseRamdumpLastAlog-01+]
		}
		/*CORE-TH-AlogRamConsoleBufferSize-00-*/
	} //MTD-kernel-BH-ParseRamdumpLastAlog-00*]
	else 
	{
		pr_info("[LastAlog] init for log type:%d no valid data in buffer (sig = 0x%08x)\n", log_type, buffer->sig);
		/* init alog_ram_console_old_log[] */
		alog_ram_console_old_log[log_type]=NULL;	
		alog_ram_console_old_log_size[log_type]=0;		
	}

	/* reset alog ram console */
	buffer->sig = alog_ram_console_signature[log_type]; /* mark alog ram console signature */
	buffer->start = 0; /* reset start pointer to 0 */
	buffer->size = 0; /* reset used buffer size to 0 */
	buffer->reader_pos = 0; /* reset reader position */
	need_fix_reader[log_type] = 0; /* reset fixed reader flag */
	next_log_entry[log_type] = buffer->data; /* reset next_log_entry to beginning of buffer */
	alog_ram_console_sync_time(log_type, SYNC_AFTER); /* sync time in beginning */
	return 0;
}

static int alog_ram_console_driver_probe(struct platform_device *pdev)
{
	struct resource *res;
	uint32_t start;
	uint32_t whole_buffer_size;
	void * whole_buffer;
	int i,ret=0;	

	spin_lock_init(&bufflock); //MTD-SW3-KERNEL-BH-FixLastAlog-00*
	for (i=0; i<LOG_TYPE_NUM; i++)
	{
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,alog_ram_console_res_name[i]);
		if (res == NULL) 
		{
			pr_err("[LastAlog] driver_probe: invalid resource -%s\n",alog_ram_console_res_name[i]);
			return -EFAULT;
		}
		else
		{
			whole_buffer_size = res->end - res->start + 1;
			start = res->start;
			pr_info("[LastAlog] driver_probe: got physical %s at 0x%x, size %d(0x%x)\n", alog_ram_console_res_name[i], start, whole_buffer_size,whole_buffer_size);
			whole_buffer = ioremap(res->start, whole_buffer_size);
			if (whole_buffer == NULL)
			{
				pr_err("[LastAlog] driver_probe: failed to map memory for %s!\n", alog_ram_console_res_name[i]);
				return -ENOMEM;
			}
			ret += alog_ram_console_init(i, (AlogRamConsoleBuffer *)whole_buffer, whole_buffer_size, NULL);
		}
	}
	return ret;
}

/* CORE-HC-New_RAM_Console-00*[ */
static struct of_device_id alog_ram_console_dt_match[] = {
	{
		.compatible = "qcom,alog_ram_console",
	},
	{}
};

static struct platform_driver alog_ram_console_driver = {
	.probe = alog_ram_console_driver_probe,
	.driver		= {
		.name	= "qcom,alog_ram_console",
		.of_match_table = alog_ram_console_dt_match,
	},
};
/* CORE-HC-New_RAM_Console-00*] */

static int __init alog_ram_console_module_init(void)
{
	int err;

	err = platform_driver_register(&alog_ram_console_driver);
	if (err)
		pr_err("[LastAlog] alog_ram_console_module_init ret:%d\n", err);
	
	return err;
}

//MTD-kernel-BH-ParseRamdumpLastAlog-00+[
static int alog_ram_console_read_old(const LogType log_type, char __user *buf, 
											size_t len, loff_t pos, ssize_t *count)
{
	LoggerEntry *pLogEntry;
	
	if (pos > alog_ram_console_old_log_size[log_type])
	{
		pr_err("[LastAlog] read_old: stop by pos:%d > alog_ram_console_old_log_size[%d]:%d", (int)pos, log_type,  alog_ram_console_old_log_size[log_type]);
		return -EFAULT;
	}

	if (pos == alog_ram_console_old_log_size[log_type])
	{
		*count=0;
		return 0;
	}
	
	pLogEntry = (LoggerEntry *)(alog_ram_console_old_log[log_type] + pos);
	if ( (pos + sizeof(LoggerEntry) + pLogEntry->len) > alog_ram_console_old_log_size[log_type])
	{
		pr_info("[LastAlog] read_old: stop by (pos:%d + 20 + pLogEntry->len:%d) > alog_ram_console_old_log_size[%d]:%d", (int)pos, pLogEntry->len, log_type,  alog_ram_console_old_log_size[log_type]);	
		return -EFAULT;
	}
	
	*count = min((ssize_t)len, (ssize_t)(pLogEntry->len + sizeof(LoggerEntry)));

	if (copy_to_user(buf, alog_ram_console_old_log[log_type] + pos, *count))
	{
    	pr_err("[LastAlog] read_old: fail to copy to user\n");
		return -EFAULT;
	}

	return 0;
}
//MTD-kernel-BH-ParseRamdumpLastAlog-00+]
//MTD-kernel-BH-ParseRamdumpLastAlog-00*[
static ssize_t alog_ram_console_read_old_main(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	ssize_t count;

	if (alog_ram_console_read_old(LOG_TYPE_MAIN, buf, len, *offset, &count))
		return -EFAULT;

	*offset += count;
	return count;
}

static ssize_t alog_ram_console_read_old_radio(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	ssize_t count;

	if (alog_ram_console_read_old(LOG_TYPE_RADIO, buf, len, *offset, &count))
		return -EFAULT;

	*offset += count;
	return count;
}

static ssize_t alog_ram_console_read_old_events(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	ssize_t count;

	if (alog_ram_console_read_old(LOG_TYPE_EVENTS, buf, len, *offset, &count))
		return -EFAULT;

	*offset += count;
	return count;
}

static ssize_t alog_ram_console_read_old_system(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	ssize_t count;

	if (alog_ram_console_read_old(LOG_TYPE_SYSTEM, buf, len, *offset, &count))
		return -EFAULT;

	*offset += count;
	return count;
}
//MTD-kernel-BH-ParseRamdumpLastAlog-00*]

//CORE-KH-DbgCfgTool-LogcatCrashCmd-00-a[
static ssize_t alog_ram_console_read_old_crash(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	ssize_t count;

	if (alog_ram_console_read_old(LOG_TYPE_CRASH, buf, len, *offset, &count))
		return -EFAULT;

	*offset += count;
	return count;
}
//CORE-KH-DbgCfgTool-LogcatCrashCmd-00-a]

static struct file_operations alog_ram_console_file_ops[LOG_TYPE_NUM] = {
	{
	.owner = THIS_MODULE,
	.read = alog_ram_console_read_old_main,
	},
	{
	.owner = THIS_MODULE,
	.read = alog_ram_console_read_old_radio,
	},
	{
	.owner = THIS_MODULE,
	.read = alog_ram_console_read_old_events,
	},
	{
	.owner = THIS_MODULE,
	.read = alog_ram_console_read_old_system,
	}
//CORE-KH-DbgCfgTool-LogcatCrashCmd-00-a[
	,
	{
	.owner = THIS_MODULE,
	.read = alog_ram_console_read_old_crash,
	}
//CORE-KH-DbgCfgTool-LogcatCrashCmd-00-a]
};
//MTD-kernel-BH-ParseRamdumpLastAlog-00+[
static ssize_t alog_ram_console_write_ext(const LogType log_type, struct file *file, const char __user *buf,
				    size_t len, loff_t *offset)
{
	int whole_buffer_size = alog_ram_console_payload_size[log_type] + sizeof(AlogRamConsoleBuffer);

	//CORE-BH-ParseRamdumpLastAlog-01+[
	if (need_alloc_ext_buffer)
	{
		alog_ram_console_buffer_ext = (AlogRamConsoleBuffer *)vmalloc(whole_buffer_size);
		if (alog_ram_console_buffer_ext == NULL) {
			pr_err("[LastAlog] write_ext: failed to allocate alog ram console buffer for [%d] whole_buffer_size:%d\n", log_type, whole_buffer_size);
			return -ENOMEM;
		}
		need_alloc_ext_buffer = 0;
	}
	//CORE-BH-ParseRamdumpLastAlog-01+]
	//CORE-BH-ParseRamdumpLastAlog-01*[
	if ((*offset + len) > whole_buffer_size)
	{
		pr_err("[LastAlog] write_ext: stop by (offset:%d + len:%d) > whole_buffer_size:%d\n", (int)*offset, (int)len, whole_buffer_size);
		return -EFAULT;
	}

    if(copy_from_user((char*)alog_ram_console_buffer_ext +*offset, buf, len))
    {
    	pr_info("[LastAlog] write_ext: fail to copy alog_ram_console_buffer_ext for [%d] from user\n", log_type);
        return -EFAULT;
    }
	//CORE-BH-ParseRamdumpLastAlog-01*]
    *offset +=  len;
		
	return len;
}

static ssize_t alog_ram_console_write_ext_main(struct file *file, const char __user *buf,
				    size_t len, loff_t *offset)
{
	return alog_ram_console_write_ext(LOG_TYPE_MAIN, file, buf, len, offset);
}

static ssize_t alog_ram_console_write_ext_radio(struct file *file, const char __user *buf,
				    size_t len, loff_t *offset)
{
	return alog_ram_console_write_ext(LOG_TYPE_RADIO, file, buf, len, offset);
}

static ssize_t alog_ram_console_write_ext_events(struct file *file, const char __user *buf,
				    size_t len, loff_t *offset)
{
	return alog_ram_console_write_ext(LOG_TYPE_EVENTS, file, buf, len, offset);
}

static ssize_t alog_ram_console_write_ext_system(struct file *file, const char __user *buf,
				    size_t len, loff_t *offset)
{
	return alog_ram_console_write_ext(LOG_TYPE_SYSTEM, file, buf, len, offset);
}

//CORE-KH-DbgCfgTool-LogcatCrashCmd-00-a[
static ssize_t alog_ram_console_write_ext_crash(struct file *file, const char __user *buf,
				    size_t len, loff_t *offset)
{
	return alog_ram_console_write_ext(LOG_TYPE_CRASH, file, buf, len, offset);
}
//CORE-KH-DbgCfgTool-LogcatCrashCmd-00-a]

static int alog_ram_console_read_ext(const LogType log_type, char __user *buf,
				    size_t len, loff_t pos, ssize_t *count)
{
	int ret = 0; //CORE-BH-ParseRamdumpLastAlog-01+
	int whole_buffer_size = alog_ram_console_payload_size[log_type] + sizeof(AlogRamConsoleBuffer);

	*count = 0;

	if (pos > whole_buffer_size)
	{
		pr_err("[LastAlog] read_ext: stopy by pos:%d > whole_buffer_size:%d\n", (int)pos, whole_buffer_size);		
		return -EFAULT;
	}

	if (pos == 0)
	{
		if (alog_ram_console_old_log[log_type])
		{
			//CORE-BH-ParseRamdumpLastAlog-01*[
			pr_info("[LastAlog] read_ext: alog_ram_console_old_log[%d] is exist, free to re-allocate for\n", log_type); 
			vfree(alog_ram_console_old_log[log_type]);
			alog_ram_console_old_log[log_type] = NULL;
			//CORE-BH-ParseRamdumpLastAlog-01*]
		}
		ret = alog_ram_console_save_old(log_type,NULL,1); //CORE-BH-ParseRamdumpLastAlog-01*

		if (!lastalog_entry[log_type])
		{
//CORE-KH-DebugToolPorting-01-m[
			lastalog_entry[log_type]  = proc_create_data(alog_ram_console_proc_fn[log_type], S_IFREG | S_IRUGO, NULL, &alog_ram_console_file_ops[log_type] ,NULL);
//CORE-KH-DebugToolPorting-01-m]
			if (!lastalog_entry[log_type]) 
			{
				pr_err("[LastAlog] alog_ram_console_read_ext_main: failed to create proc entry - /proc/%s\n", alog_ram_console_proc_fn[log_type]);
				vfree(alog_ram_console_old_log[log_type]);
				alog_ram_console_old_log[log_type] = NULL;
				return -EFAULT;
			}

//CORE-KH-DebugToolPorting-01-d			lastalog_entry[log_type] ->proc_fops = &alog_ram_console_file_ops[log_type];
		}
		lastalog_entry[log_type] ->size = alog_ram_console_old_log_size[log_type];		

		*count = whole_buffer_size;
	}
	return ret; //CORE-BH-ParseRamdumpLastAlog-01*
}


static ssize_t alog_ram_console_read_ext_main(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	ssize_t count;

	if (alog_ram_console_read_ext(LOG_TYPE_MAIN, buf, len, *offset, &count))
		return -EFAULT;

	*offset += count;
	return count;
}

static ssize_t alog_ram_console_read_ext_radio(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	ssize_t count;

	if (alog_ram_console_read_ext(LOG_TYPE_RADIO, buf, len, *offset, &count))
		return -EFAULT;

	*offset += count;
	return count;
}

static ssize_t alog_ram_console_read_ext_events(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	ssize_t count;

	if (alog_ram_console_read_ext(LOG_TYPE_EVENTS, buf, len, *offset, &count))
		return -EFAULT;

	*offset += count;
	return count;
}

static ssize_t alog_ram_console_read_ext_system(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	ssize_t count;
	
	if (alog_ram_console_read_ext(LOG_TYPE_SYSTEM, buf, len, *offset, &count))
		return -EFAULT;
		
	*offset += count;
	return count;
}

//CORE-KH-DbgCfgTool-LogcatCrashCmd-00-a[
static ssize_t alog_ram_console_read_ext_crash(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	ssize_t count;
	
	if (alog_ram_console_read_ext(LOG_TYPE_CRASH, buf, len, *offset, &count))
		return -EFAULT;
		
	*offset += count;
	return count;
}
//CORE-KH-DbgCfgTool-LogcatCrashCmd-00-a]

static struct file_operations alog_ram_console_ext_file_ops[LOG_TYPE_NUM] = {
	{
	.owner = THIS_MODULE,
	.write = alog_ram_console_write_ext_main,
	.read = alog_ram_console_read_ext_main,
	},
	{
	.owner = THIS_MODULE,
	.write = alog_ram_console_write_ext_radio,
	.read = alog_ram_console_read_ext_radio,
	},
	{
	.owner = THIS_MODULE,
	.write = alog_ram_console_write_ext_events,
	.read = alog_ram_console_read_ext_events,
	},
	{
	.owner = THIS_MODULE,
	.write = alog_ram_console_write_ext_system,
	.read = alog_ram_console_read_ext_system,
	}
//CORE-KH-DbgCfgTool-LogcatCrashCmd-00-a[
	,
	{
	.owner = THIS_MODULE,
	.write = alog_ram_console_write_ext_crash,
	.read = alog_ram_console_read_ext_crash,
	}
//CORE-KH-DbgCfgTool-LogcatCrashCmd-00-a]
};
//MTD-kernel-BH-ParseRamdumpLastAlog-00+]
static int __init alog_ram_console_late_init(void)
{
	struct proc_dir_entry *entry;
	int i;

	/* create a proc file to read old log for alog ram console */
	for (i=0; i<LOG_TYPE_NUM; i++)
		{
		if (alog_ram_console_old_log[i])
		{
			//MTD-kernel-BH-ParseRamdumpLastAlog-00*[
//CORE-KH-DebugToolPorting-01-m[
			lastalog_entry[i]  = proc_create_data(alog_ram_console_proc_fn[i], S_IFREG | S_IRUGO, NULL, &alog_ram_console_file_ops[i], NULL);
//CORE-KH-DebugToolPorting-01-m]
			if (!lastalog_entry[i]) 
			{
				pr_err("[LastAlog] late_init: failed to create proc entry - /proc/%s\n", alog_ram_console_proc_fn[i]);
				vfree(alog_ram_console_old_log[i]);
				alog_ram_console_old_log[i] = NULL;
				return -EFAULT;
			}

//CORE-KH-DebugToolPorting-01-d			lastalog_entry[i] ->proc_fops = &alog_ram_console_file_ops[i];
			lastalog_entry[i] ->size = alog_ram_console_old_log_size[i];
			//MTD-kernel-BH-ParseRamdumpLastAlog-00*]
		}
		//MTD-kernel-BH-ParseRamdumpLastAlog-00+[
//CORE-KH-DebugToolPorting-01-m[
		entry = proc_create_data(alog_ram_console_ext_proc_fn[i], S_IFREG | S_IRUGO | S_IWUGO, NULL, &alog_ram_console_ext_file_ops[i], NULL);
//CORE-KH-DebugToolPorting-01-m]
		if (!entry) 
		{
			pr_err("[LastAlog] late_init: failed to create proc entry - /proc/%s\n", alog_ram_console_ext_proc_fn[i]);
			return -EFAULT;
		}
//CORE-KH-DebugToolPorting-01-d		entry->proc_fops = &alog_ram_console_ext_file_ops[i];
		entry->size = alog_ram_console_payload_size[i] + sizeof(AlogRamConsoleBuffer);
		//MTD-kernel-BH-ParseRamdumpLastAlog-00+]
		}
		
	return 0;
}

//module_init(alog_ram_console_module_init);
postcore_initcall(alog_ram_console_module_init);
late_initcall(alog_ram_console_late_init);
