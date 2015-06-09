/*************************************************************************************
 *
 * FIH Project
 *
 * General Description
 * 
 * Linux kernel: Definitions of SW related information for SEMC.
 *	
 * Copyright(C) 2011-2014 Foxconn International Holdings, Ltd. All rights reserved.
 * Copyright (C) 2011, Foxconn Corporation (BokeeLi@fih-foxconn.com)
 *
 */
/*************************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/string.h>
//CORE-KH-DebugToolPorting-00-d #include <linux/sysdev.h>
#include <linux/fih_sw_info.h>
#include <linux/fih_hw_info.h> //BSP-REXER-HWID-02+
#include <asm/setup.h>

// For HW ID
//#include <linux/mfd/88pm860x.h>
// For SW_ID SW_INFO sync
#include <linux/mtd/mtd.h>
//#include <mach/Testflag.h>

/*CORE-TH-manual_crash-00+*/
#include <linux/rtc.h>
#include <linux/ktime.h>
/*CORE-TH-manual_crash-00+*/

/* FIH-SW3-KERNEL-EL-write_panic_file-00+[ */
#ifdef CONFIG_FEATURE_FIH_SW3_PANIC_FILE
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#endif
/* FIH-SW3-KERNEL-EL-write_panic_file-00+] */

#include <linux/delay.h> /* CORE-EL-trigger_hwwd_00+ */

#ifdef CONFIG_MTD	/* MTD-SW3-BSP-AC-Disable_MTD-01+ */
extern uint32_t fih_get_flash_id(void);
#endif /* MTD-SW3-BSP-AC-Disable_MTD-01+ */

#define ID2NAME(ID,NAME)  {(unsigned int)(ID), (char*)(NAME)}

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))
#endif

typedef struct 
{
	unsigned int flash_id;
	char *flash_name;
}flash_id_number_name_map;

typedef struct
{
	unsigned int hw_id_number;
	char *hw_id_name;
}hw_id_number_name_map;

//	ID2NAME(0xBC2C, "Micron"),
flash_id_number_name_map flash_id_map_table[] = 
{
	ID2NAME(0x00000000, "ONFI"),
	ID2NAME(0x1500aaec, "Sams"),
	ID2NAME(0x5500baec, "Sams"),
	ID2NAME(0x1500aa98, "Tosh"),
	ID2NAME(0x5500ba98, "Tosh"),
	ID2NAME(0xd580b12c, "Micr"),
	ID2NAME(0x5590bc2c, "Micr"),
	ID2NAME(0x1580aa2c, "Micr"),
	ID2NAME(0x1590aa2c, "Micr"),
	ID2NAME(0x1590ac2c, "Micr"),
	ID2NAME(0x5580baad, "Hynx"),
	ID2NAME(0x5510baad, "Hynx"),
	ID2NAME(0x004000ec, "Sams"),
	ID2NAME(0x005c00ec, "Sams"),
	ID2NAME(0x005800ec, "Sams"),
	ID2NAME(0x6600bcec, "Sams"),
	ID2NAME(0x5580ba2c, "Hynx"),
	ID2NAME(0x6600b3ec, "Sams"),
	ID2NAME(0x6601b3ec, "Sams"),
	ID2NAME(0xffffffff, "unknown flash")
};

static struct kobject *sw_info_kobj = NULL;

#define sw_info_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0644,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}

#define sw_info_func_init(type,name,initval)   \
	static type name = (initval);       \
	type get_##name(void)               \
	{                                   \
	    return name;                    \
	}                                   \
	void set_##name(type __##name)      \
	{                                   \
	    name = __##name;                \
	}                                   \
	EXPORT_SYMBOL(set_##name);          \
	EXPORT_SYMBOL(get_##name);


/* FIH-SW3-KERNEL-EL-modem_crash_info-00+[ */  
extern unsigned int fih_get_power_on_cause(void);


/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*[ */
static ssize_t fih_complete_sw_version_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;

	/* PartNumber_R1AAxxx --> ex: 1248-5695_R1AA001_DEV */
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), SEMC_PartNumber);
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "_");
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), SEMC_SwRevison);
	if(strlen(SEMC_ReleaseFlag) != 0)
	{
		s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "_");
		s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), SEMC_ReleaseFlag);
	}
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "\n");
	
	return (s - buf);
}
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*] */

static ssize_t fih_complete_sw_version_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

sw_info_attr(fih_complete_sw_version);


/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*[ */
static ssize_t fih_sw_version_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;

	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), SEMC_SwRevison);
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "\n");
	
	return (s - buf);
}
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*] */

static ssize_t fih_sw_version_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

sw_info_attr(fih_sw_version);

/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*[ */
static ssize_t fih_part_number_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;

	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), SEMC_PartNumber);
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "\n");
	
	return (s - buf);
}
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*] */
static ssize_t fih_part_number_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

sw_info_attr(fih_part_number);

/* CORE-EL-trigger_hwwd_00+[ */
static struct work_struct		busy_loop_wrk_struct;

static void busy_loop_func(struct work_struct *work)
{
	while(1);
}
/* CORE-EL-trigger_hwwd_00+] */

static ssize_t fih_crash_test_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	return 0;
}

/* FIH-SW3-KERNEL-EL-trigger_panic_but_don't_send_mtbf-00+ */
extern int send_mtbf; /* determine if send mtbf report */

static ssize_t fih_crash_test_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
  int *p = NULL;
  int val = 100;
  int zero = 0;
  
  //MTD-BSP-REXER-DEFECT*[
  if(buf!=NULL)
  {
	printk(KERN_ERR "fih_crash_test_store %s %lu %lu\n", buf, strlen(buf), n);
	
	if (strncmp(buf, "panic", 5) == 0)
		panic("simulate panic");
	else if (strncmp(buf, "null", 4) == 0)
		*p = 651105; //We write any number to the location 0 of memory to trigger the ramdump for SEMC test
	else if (strncmp(buf, "divide", 6) == 0)
		val = val / zero;
	else if (strncmp(buf, "anr_ramdump", 11) == 0)
	{
	  send_mtbf = 0;
	  *p = 19761105; //We write any number to the location 0 of memory to trigger the ramdump for SEMC test
	}
	/* CORE-EL-trigger_hwwd_00+[ */
	else if (strncmp(buf, "hwwd", 4) == 0)
	{
		unsigned int cpu_itr;
		DEFINE_SPINLOCK(mr_lock);
		unsigned long flags;

		spin_lock_irqsave(&mr_lock, flags);
		for_each_cpu(cpu_itr, cpu_online_mask) {
			schedule_work_on(cpu_itr,&busy_loop_wrk_struct);
		}
		while (1);
		spin_unlock_irqrestore(&mr_lock, flags);
	
	}
	/* CORE-EL-trigger_hwwd_00+] */
  }
  else
  {
  	printk(KERN_ERR "%s: buf is null pointer\n",__FUNCTION__);
  }
  //MTD-BSP-REXER-DEFECT*]
  
  return n;
}

sw_info_attr(fih_crash_test);

/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*[ */
static ssize_t fih_hw_id_version_number_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;

	unsigned int hw_id = 0;
	//hw_id = fih_get_product_phase();
	
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "%d", hw_id);
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "\n");

	return (s - buf);
}
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*] */

static ssize_t fih_hw_id_version_number_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

sw_info_attr(fih_hw_id_version_number);


static ssize_t fih_hw_id_version_name_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;
//CORE-KH-DebugToolPorting-00-d[
//	unsigned int hw_id = 0;
//	unsigned int i;
//	//BSP-REXER-HWID-02*[
//	hw_id = fih_get_product_phase();
//
//	for(i=0 ; phase_id_map[i].ID != PHASE_MAX; i++)
//	{
//	  if(phase_id_map[i].ID == hw_id)
//	  {
//	    s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "%s", phase_id_map[i].STR);
//		s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "\r\n");
//		break;
//	  }
//	}
    //BSP-REXER-HWID-02*]
//CORE-KH-DebugToolPorting-00-d]
	return (s - buf);
}

static ssize_t fih_hw_id_version_name_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

sw_info_attr(fih_hw_id_version_name);


static ssize_t fih_flash_id_version_number_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;

	unsigned int flash_id =0 ;
	#ifdef CONFIG_MTD /* MTD-SW3-BSP-AC-Disable_MTD-01+ */
	flash_id = fih_get_flash_id();
	#endif /* MTD-SW3-BSP-AC-Disable_MTD-01+ */
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*[ */
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "%X", flash_id);
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "\n");
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*] */
	return (s - buf);
}

static ssize_t fih_flash_id_version_number_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

sw_info_attr(fih_flash_id_version_number);

static ssize_t fih_flash_id_version_name_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;
	unsigned int flash_id = 0;
	int idx;
	int array_max_idx = ARRAY_SIZE(flash_id_map_table) - 1;
	#ifdef CONFIG_MTD /* MTD-SW3-BSP-AC-Disable_MTD-01+ */
	flash_id = fih_get_flash_id();
	#endif /* MTD-SW3-BSP-AC-Disable_MTD-01+ */

	for (idx = 0; idx <= array_max_idx; idx ++)
	{
		if(flash_id_map_table[idx].flash_id == flash_id)
		{
			break;
		}
	}

	if (idx > array_max_idx)
		idx = array_max_idx;
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*[ */
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "%s", flash_id_map_table[idx].flash_name);
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "\n");
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*] */
	return (s - buf);
}

static ssize_t fih_flash_id_version_name_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

sw_info_attr(fih_flash_id_version_name);


static ssize_t fih_blob_version_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*[ */
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "%s", SEMC_BlobVersion);
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "\n");
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*] */	

	return (s - buf);
}

static ssize_t fih_blob_version_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

sw_info_attr(fih_blob_version);


static ssize_t fih_bp_version_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;
	
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*[ */
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "%s", SEMC_BpVersion);
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "\n");
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*] */	

	return (s - buf);
}

static ssize_t fih_bp_version_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

sw_info_attr(fih_bp_version);


static ssize_t fih_svn_version_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;

/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*[ */
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "%s", SEMC_SvnVersion);
	s += snprintf(s, PAGE_SIZE - ((size_t)(s-buf)), "\n");
/* FIH-SW3-KERNEL-EL-fix_coverity-issues-02*] */

	return (s - buf);
}

static ssize_t fih_svn_version_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

sw_info_attr(fih_svn_version);

/* FIH-SW3-KERNEL-EL-get_last_alog_buffer_virt_addr-01+ */
void * get_alog_buffer_virt_addr(void);

/* FIH-SW3-KERNEL-EL-write_panic_file-00+[ */  
static ssize_t fih_crash_info_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    ssize_t ret = 0;
    struct fih_panic_ram_data *fih_panic_ram_data_ptr = NULL;
	/*CORE-TH-manual_crash-00+*/
	struct timespec tmp_time;
	struct rtc_time rtc_new_rtc_time;
	/*CORE-TH-manual_crash-00-*/

/* FIH-SW3-KERNEL-EL-get_last_alog_buffer_virt_addr-01*[ */
    fih_panic_ram_data_ptr = (struct fih_panic_ram_data *)(struct fih_panic_ram_data *) get_alog_buffer_virt_addr();
/* FIH-SW3-KERNEL-EL-get_last_alog_buffer_virt_addr-01*] */

/* FIH-SW3-KERNEL-EL-write_panic_2_file-02*[ */  

	/*CORE-TH-manual_crash-00+*/
		if (fih_get_power_on_cause()& MTD_PWR_ON_EVENT_FORCE_TRIGGER_PANIC){
			getnstimeofday(&tmp_time);
			rtc_time_to_tm(tmp_time.tv_sec, &rtc_new_rtc_time);
		
			snprintf(buf, PAGE_SIZE, "manual:1, crash info: %04d%02d%02d%02d%02d%02d\n", 
					rtc_new_rtc_time.tm_year + 1900,
					rtc_new_rtc_time.tm_mon + 1,
					rtc_new_rtc_time.tm_mday,				
					rtc_new_rtc_time.tm_hour, 
					rtc_new_rtc_time.tm_min,
					rtc_new_rtc_time.tm_sec);
			printk("manual:1, crash info: %04d%02d%02d%02d%02d%02d\n",
					rtc_new_rtc_time.tm_year + 1900,
					rtc_new_rtc_time.tm_mon + 1,
					rtc_new_rtc_time.tm_mday,				
					rtc_new_rtc_time.tm_hour, 
					rtc_new_rtc_time.tm_min,
					rtc_new_rtc_time.tm_sec);
			ret = 40;
			fih_panic_ram_data_ptr->signature = 0x1105;
		}
		else if (fih_panic_ram_data_ptr != NULL) {
			if (fih_panic_ram_data_ptr->signature == PANIC_RAM_SIGNATURE) {
	
				/*CORE-TH-manual_crash-00+*/
				snprintf(buf, PAGE_SIZE, "manual:0, crash info: %s\n", fih_panic_ram_data_ptr->data);
				printk("manual:0, crash info: %s\n", fih_panic_ram_data_ptr->data);
				/*CORE-TH-manual_crash-00-*/
				
				ret = fih_panic_ram_data_ptr->length+25;
				
				/* erase the signature after application read the data */
				fih_panic_ram_data_ptr->signature = 0x1105;
			}
		}
		/*CORE-TH-manual_crash-00-*/

/* FIH-SW3-KERNEL-EL-write_panic_2_file-02*] */    
	return (ret);
}

static ssize_t fih_crash_info_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

sw_info_attr(fih_crash_info);
/* FIH-SW3-KERNEL-EL-write_panic_file-00+] */  

/* MTD-CORE-EL-AddInitStringForMtbf-00+[ */
void *get_fatal_error_buffer_virt_addr(void) {
	static void *virt_addr = 0;

	if (unlikely(virt_addr == 0)){
		virt_addr = ioremap(STORE_FATAL_ERROR_REASON, DIAG_BUFFER_LEN);
	}

	return virt_addr;
}
/* MTD-CORE-EL-AddInitStringForMtbf-00+] */


/* MTD-CORE-EL-handle_SSR-00*[ */

/* MTD-CORE-EL-handle_SSR-02*[ */
void log_ss_failure_reason(const char* ss_name, u32 size, char* err_str){
	char *fatal_error_buffer_virt_addr = (char*) get_fatal_error_buffer_virt_addr();

	if (fatal_error_buffer_virt_addr)  {

		if (size == 0)
			size = strlen(err_str);

		if (size)			
			snprintf(fatal_error_buffer_virt_addr, DIAG_BUFFER_LEN,
				"%s%s:%s", MODEM_ERR_TAG, ss_name, err_str);
		else
			snprintf(fatal_error_buffer_virt_addr, DIAG_BUFFER_LEN,
				"%s%s", MODEM_ERR_TAG, ss_name);
	}
	else
		pr_err("%s fatal error: fatal_error_buffer_virt_addr is Null!\n", __func__);
}
/* MTD-CORE-EL-handle_SSR-02*] */


/* MTD-CORE-EL-handle_SSR-02+[ */
extern void clear_all_modem_pwron_cause (void);
/* MTD-CORE-EL-handle_SSR-02+] */

extern unsigned int latest_modem_err;
extern unsigned int fih_power_on_cause; /* CORE-EL-HWWD-00+ */


static ssize_t fih_modem_crash_info_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	enum err_type {
		ERR_TYPE_MFE = 0,
		ERR_TYPE_SWD,
		ERR_TYPE_FWD,
		ERR_TYPE_HWD,
		ERR_TYPE_NA,
		ERR_TYPE_MAX
	};

	ssize_t ret = 0;
	char *fatal_error_buffer_virt_addr;
	unsigned int power_on_cause;

	char *err_string[ERR_TYPE_MAX] = { "Modem SW_WD Reset", "Modem SW_WD Reset", "Modem FW_WD Reset", "HW_WD timeout", "N/A"};
	enum err_type etype = ERR_TYPE_NA;
	char *err_reason = err_string[ERR_TYPE_NA];

	power_on_cause =  latest_modem_err;

	/* read clear */
	latest_modem_err = 0;

	/* %%TBTA: if AP doesn't read and human reboot when SS has 
	     error fatal (SSR enabled), it will show SS error string after reboot. 
	     I don't want to fix this issue now. 	Because human reset the device 
	     may due to the device is not stable because of SSR occurring. */
	if (power_on_cause & MTD_PWR_ON_EVENT_MODEM_FATAL_ERROR) {
		etype = ERR_TYPE_MFE;
	} else if (power_on_cause & MTD_PWR_ON_EVENT_MODEM_SW_WD_RESET) {
		etype = ERR_TYPE_SWD;
	} else if (power_on_cause & MTD_PWR_ON_EVENT_MODEM_FW_WD_RESET) {
		etype = ERR_TYPE_FWD;
/* CORE-EL-HWWD-00*[ */
	} else if (fih_power_on_cause & MTD_PWR_ON_EVENT_HW_WD_RESET) {
		etype = ERR_TYPE_HWD;

		/* read clear */
		fih_power_on_cause &= ~MTD_PWR_ON_EVENT_HW_WD_RESET;
	}
/* CORE-EL-HWWD-00*] */

	if (etype < ERR_TYPE_HWD) {
		if ((fatal_error_buffer_virt_addr =  (char *) get_fatal_error_buffer_virt_addr())) {
			if (!strncmp(fatal_error_buffer_virt_addr, MODEM_ERR_TAG, MODEM_ERR_TAG_LEN)) {
				err_reason = &fatal_error_buffer_virt_addr[MODEM_ERR_TAG_LEN];
				
				/* %%TBTA: in tulip, need to do more test */
#if 0
				/* venus is a special sub-system, we should clear fatal error here */
				if (!strncmp(err_reason, "venus", 5)) {
					clear_all_modem_pwron_cause();
					printk(KERN_ERR "This is venus sub-system, clear power on cause here\n");
				}
#endif				
			}
		}
		else
			pr_err("%s fatal error: fatal_error_buffer_virt_addr is Null!\n", __func__);
	}

	ret = snprintf(buf, PAGE_SIZE, "%s: %s", err_string[etype], err_reason);
	
	return (ret);
}
/* MTD-CORE-EL-handle_SSR-00*] */

static ssize_t fih_modem_crash_info_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}

sw_info_attr(fih_modem_crash_info);
/* FIH-SW3-KERNEL-EL-modem_crash_info-00+] */  

static struct attribute * g[] = {
	&fih_complete_sw_version_attr.attr,
	&fih_sw_version_attr.attr,
	&fih_part_number_attr.attr,
	&fih_hw_id_version_number_attr.attr,
	&fih_hw_id_version_name_attr.attr,
	&fih_flash_id_version_number_attr.attr,
	&fih_flash_id_version_name_attr.attr,
	&fih_blob_version_attr.attr,
	&fih_bp_version_attr.attr,
	&fih_svn_version_attr.attr,
	&fih_crash_test_attr.attr,
#ifdef CONFIG_FEATURE_FIH_SW3_PANIC_FILE	
	&fih_crash_info_attr.attr, /* FIH-SW3-KERNEL-EL-write_panic_file-00+ */  
#endif
	&fih_modem_crash_info_attr.attr, /* FIH-SW3-KERNEL-EL-modem_crash_info-00+ */
	NULL,
};


static struct attribute_group attr_group = {
	.attrs = g,
};


static int sw_info_sync(void)
{
	return 0;
}

static int __init sw_info_init(void)
{
	int ret = -ENOMEM;

	get_fatal_error_buffer_virt_addr(); /* MTD-CORE-EL-ioremap_now-00+ */
	
	sw_info_kobj = kobject_create_and_add("fih_sw_info", NULL);
	if (sw_info_kobj == NULL) {
		printk("sw_info_init: subsystem_register failed\n");
		goto fail;
	}

	ret = sysfs_create_group(sw_info_kobj, &attr_group);
	if (ret) {
		printk("sw_info_init: subsystem_register failed\n");
		goto sys_fail;
	}

	sw_info_sync();

	/* CORE-EL-trigger_hwwd_00+ */
	INIT_WORK(&busy_loop_wrk_struct, busy_loop_func);

	return ret;

sys_fail:
	kobject_del(sw_info_kobj);
fail:
	return ret;

}

static void __exit sw_info_exit(void)
{
	if (sw_info_kobj) {
		sysfs_remove_group(sw_info_kobj, &attr_group);
		kobject_del(sw_info_kobj);
	}
}

late_initcall(sw_info_init);
module_exit(sw_info_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Eric Liu <huaruiliu@fihspec.com>");
MODULE_DESCRIPTION("SW information collector");
