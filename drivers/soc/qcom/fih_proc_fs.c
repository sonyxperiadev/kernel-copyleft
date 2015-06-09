/*
* Copyright(C) 2011-2014 Foxconn International Holdings, Ltd. All rights reserved
*/
#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/fih_hw_info.h>
#include <linux/version_host.h>
#include <linux/seq_file.h>

/* CORE-HC-productinfo-00+[ */
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/fih_sw_info.h>
/* CORE-HC-productinfo-00+] */

/* CORE-HC-Last_AMSS_Log-00+[ */
void * get_pwron_cause_virt_addr(void);
void *get_fatal_error_buffer_virt_addr(void);

char *modem_error_log_ptr = NULL;
/* CORE-HC-Last_AMSS_Log-00+] */

/************************************************
 *  Implement specific File operations for each proc entry *
 ************************************************/

/* CORE-HC-productinfo-00+[ */
void *get_productinfo_virt_addr(void) {
  static void *virt_addr = NULL;

  if (unlikely(virt_addr == NULL)){
    virt_addr = ioremap(FIH_PRODUCTINFO_ADDR, FIH_PRODUCTINFO_LEN);
  }
  
  return virt_addr;
}
EXPORT_SYMBOL(get_productinfo_virt_addr);

static int official_ver_read_proc(struct seq_file *m, void *v)
{
    char *productinfo_virt_addr = (char*) get_productinfo_virt_addr();

    if (productinfo_virt_addr != NULL)
    {
        seq_printf(m, "%s\n", productinfo_virt_addr+OFFICIAL_VER_OFFSET);
    }
    else
    {
        pr_err("productinfo_virt_addr = NULL\n");
    }

    return 0;
}

static int official_ver_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, official_ver_read_proc, NULL);
}

static ssize_t official_ver_write_proc(struct file *file,
	const char __user *buf, size_t count, loff_t *pos)
{
    char item[PRODUCTINFO_ITEM_LEN];
    char *productinfo_virt_addr;

    if (!count)
    {
        pr_err("count = 0\n");
        return 0;
    }
    if (count > PRODUCTINFO_ITEM_LEN - 1) {
        pr_err("buffer overflow, count = %zu\n", count);
        return -EINVAL;
    }
    if (copy_from_user(item, buf, count)) {
        pr_err("copy from user failed\n");
        return -EFAULT;
    }

    item[count-1] = '\0';

    productinfo_virt_addr = (char*) get_productinfo_virt_addr();
    if (productinfo_virt_addr != NULL)
    {
        memcpy_toio(productinfo_virt_addr+OFFICIAL_VER_OFFSET, item, count);
    }
    else
    {
        pr_err("productinfo_virt_addr = NULL\n");
        return 0;
    }
    return count;
}

static int vender_ver_read_proc(struct seq_file *m, void *v)
{
    char *productinfo_virt_addr = (char*) get_productinfo_virt_addr();

    if (productinfo_virt_addr != NULL)
    {
        seq_printf(m, "%s\n", productinfo_virt_addr+VENDER_VER_OFFSET);
    }
    else
    {
        pr_err("productinfo_virt_addr = NULL\n");
    }

    return 0;
}

static int vender_ver_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, vender_ver_read_proc, NULL);
}

static ssize_t vender_ver_write_proc(struct file *file,
	const char __user *buf, size_t count, loff_t *pos)
{
    char item[PRODUCTINFO_ITEM_LEN];
    char *productinfo_virt_addr;

    if (!count) {
        pr_err("count = 0\n");
        return 0;
    }
    if (count > PRODUCTINFO_ITEM_LEN - 1) {
        pr_err("buffer overflow, count = %zu\n", count);
        return -EINVAL;
    }
    if (copy_from_user(item, buf, count)) {
        pr_err("copy from user failed\n");
        return -EFAULT;
    }

    item[count-1] = '\0';

    productinfo_virt_addr = (char*) get_productinfo_virt_addr();
    if (productinfo_virt_addr != NULL)
    {
        memcpy_toio(productinfo_virt_addr+VENDER_VER_OFFSET, item, count);
    }
    else
    {
        pr_err("productinfo_virt_addr = NULL\n");
        return 0;
    }
    return count;
}

static int product_model_read_proc(struct seq_file *m, void *v)
{
    char *productinfo_virt_addr = (char*) get_productinfo_virt_addr();

    if (productinfo_virt_addr != NULL)
    {
        seq_printf(m, "%s\n", productinfo_virt_addr+PRODUCT_MODEL_OFFSET);
    }
    else
    {
        pr_err("productinfo_virt_addr = NULL\n");
    }

    return 0;
}

static int product_model_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, product_model_read_proc, NULL);
}

static ssize_t product_model_write_proc(struct file *file,
	const char __user *buf, size_t count, loff_t *pos)
{
    char item[PRODUCTINFO_ITEM_LEN];
    char *productinfo_virt_addr;

    if (!count) {
        pr_err("count = 0\n");
        return 0;
    }
    if (count > PRODUCTINFO_ITEM_LEN - 1) {
        pr_err("buffer overflow, count = %zu\n", count);
        return -EINVAL;
    }
    if (copy_from_user(item, buf, count)) {
        pr_err("copy from user failed\n");
        return -EFAULT;
    }

    item[count-1] = '\0';

    productinfo_virt_addr = (char*) get_productinfo_virt_addr();
    if (productinfo_virt_addr != NULL)
    {
        memcpy_toio(productinfo_virt_addr+PRODUCT_MODEL_OFFSET, item, count);
    }
    else
    {
        pr_err("productinfo_virt_addr = NULL\n");
        return 0;
    }
    return count;
}

static int build_type_read_proc(struct seq_file *m, void *v)
{
    char *productinfo_virt_addr = (char*) get_productinfo_virt_addr();

    if (productinfo_virt_addr != NULL)
    {
        seq_printf(m, "%s\n", productinfo_virt_addr+BUILD_TYPE_OFFSET);
    }
    else
    {
        pr_err("productinfo_virt_addr = NULL\n");
    }

    return 0;
}

static int build_type_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, build_type_read_proc, NULL);
}

static ssize_t build_type_write_proc(struct file *file,
	const char __user *buf, size_t count, loff_t *pos)
{
    char item[PRODUCTINFO_ITEM_LEN];
    char *productinfo_virt_addr;


    if (!count) {
        pr_err("count = 0\n");
        return 0;
    }
    if (count > PRODUCTINFO_ITEM_LEN - 1) {
        pr_err("buffer overflow, count = %zu\n", count);
        return -EINVAL;
    }
    if (copy_from_user(item, buf, count)) {
        pr_err("copy from user failed\n");
        return -EFAULT;
    }

    item[count-1] = '\0';

    productinfo_virt_addr = (char*) get_productinfo_virt_addr();
    if (productinfo_virt_addr != NULL)
    {
        memcpy_toio(productinfo_virt_addr+BUILD_TYPE_OFFSET, item, count);
    }
    else
    {
        pr_err("productinfo_virt_addr = NULL\n");
        return 0;
    }
    return count;
}
/* CORE-HC-productinfo-00+] */

/*-------------------------------
   Show the Device Model information  
  --------------------------------*/
static int devmodel_proc_show(struct seq_file *m, void *v)
{
	int i;
	unsigned int project = fih_get_product_id();
	char ver[16]= {0} ;
	
	for(i=0; project_id_map[i].ID != PROJECT_MAX; i++)
	{
	  if(project_id_map[i].ID == project)
	  {
		strncpy(ver, project_id_map[i].STR ,project_id_map[i].LEN);
		ver[project_id_map[i].LEN]='\0';
		seq_printf(m, "%s\n", ver);
		break;
	  }
	}

	return 0;

}


static int devmodel_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, devmodel_proc_show, NULL);
}



/*---------------------------------
   Show the Device Phase ID information 
  ----------------------------------*/

static int phaseid_read_proc_show(struct seq_file *m, void *v)
{
	int i;
	int phase = fih_get_product_phase();
	char ver[16]= {0};
	
	for(i=0; phase_id_map[i].ID != PHASE_MAX; i++)
	{
	  if(phase_id_map[i].ID == phase)
	  {
		strncpy(ver, phase_id_map[i].STR ,phase_id_map[i].LEN);
		ver[phase_id_map[i].LEN]='\0';
		seq_printf(m, "%s\n", ver);
		break;
	  }
	}

	return 0;

}


static int phaseid_read_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, phaseid_read_proc_show, NULL);
}



/*-------------------------------
    Show the Device BAND information 
   -------------------------------*/
static int bandinfo_read_proc_show(struct seq_file *m, void *v)
{
	int i;
	int band = fih_get_band_id();
	char ver[16]= {0};
	
	for(i=0; band_id_map[i].ID != BAND_MAX; i++)
	{
	  if(band_id_map[i].ID == band)
	  {
		strncpy(ver, band_id_map[i].STR ,band_id_map[i].LEN);
		ver[band_id_map[i].LEN]='\0';
		seq_printf(m, "%s\n", ver);
		break;
	  }
	}

	return 0;

}


static int bandinfo_read_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, bandinfo_read_proc_show, NULL);
}



/*------------------------------
    Show the Device SIM information 
   ------------------------------*/
static int siminfo_read_proc_show(struct seq_file *m, void *v)
{
	int i;
	int sim = fih_get_sim_id();
	char ver[16]= {0} ;
	
	for(i=0; sim_id_map[i].ID != SIM_MAX; i++)
	{
	  if(sim_id_map[i].ID == sim)
	  {
		strncpy(ver, sim_id_map[i].STR ,sim_id_map[i].LEN);
		ver[sim_id_map[i].LEN]='\0';
		seq_printf(m, "%s\n", ver);
		break;
	  }
	}

	return 0;

}


static int siminfo_read_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, siminfo_read_proc_show, NULL);
}



/*---------------------------
    Show the Non-HLOS versioin 
   ---------------------------*/
static int nonHLOS_ver_read_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", fih_get_nonHLOS_version());

	return 0;

}


static int nonHLOS_ver_read_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, nonHLOS_ver_read_proc_show, NULL);
}



/*-----------------------
     Show the HLOS versioin 
   -----------------------*/

static int HLOS_ver_read_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s.%s.%s.%s\n",
		       VER_HOST_BSP_VERSION,
			   VER_HOST_PLATFORM_NUMBER,
			   VER_HOST_BRANCH_NUMBER,
			   VER_HOST_BUILD_NUMBER);
	
	return 0;

}


static int HLOS_ver_read_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, HLOS_ver_read_proc_show, NULL);
}



/*-------------------------------------
    Show the Non-HLOS git head information 
   -------------------------------------*/
static int nonHLOS_githd_read_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", fih_get_nonHLOS_git_head());

	return 0;

}


static int nonHLOS_githd_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, nonHLOS_githd_read_proc_show, NULL);
}



/*---------------------------------
     Show the HLOS git head information 
   ---------------------------------*/
static int HLOS_githd_read_proc_show(struct seq_file *m, void *v)
{

	seq_printf(m, "%s\n", VER_HOST_GIT_COMMIT);

	return 0;

}


static int HLOS_githd_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, HLOS_githd_read_proc_show, NULL);
}

/* CORE-HC-Last_AMSS_Log-00+[ */
static int last_amsslog_proc_show(struct seq_file *m, void *v)
{
	if (modem_error_log_ptr != NULL)
		seq_printf(m, "%s\n", modem_error_log_ptr);
	
	return 0;
}

static int last_amsslog_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, last_amsslog_proc_show, NULL);
}
/* CORE-HC-Last_AMSS_Log-00+] */


/**************************
  * Definition of File operations  *
  **************************/

static const struct file_operations fih_devmodel_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= devmodel_proc_open,
	.read		= seq_read,
};

static const struct file_operations fih_phaseid_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= phaseid_read_proc_open,
	.read		= seq_read,
};

static const struct file_operations fih_bandinfo_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= bandinfo_read_proc_open,
	.read		= seq_read,
};

static const struct file_operations fih_siminfo_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= siminfo_read_proc_open,
	.read		= seq_read,
};

static const struct file_operations fih_nonHLOS_ver_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= nonHLOS_ver_read_proc_open,
	.read		= seq_read,
};

static const struct file_operations fih_HLOS_ver_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= HLOS_ver_read_proc_open,
	.read		= seq_read,
};

static const struct file_operations fih_nonHLOS_githd_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= nonHLOS_githd_proc_open,
	.read		= seq_read,
};

static const struct file_operations fih_HLOS_githd_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= HLOS_githd_proc_open,
	.read		= seq_read,
};

/* CORE-HC-productinfo-00+[ */
	static const struct file_operations fih_official_ver_proc_fops = {
	.owner	= THIS_MODULE,
	.open		= official_ver_proc_open,
	.read		= seq_read,
	.write	= official_ver_write_proc,
	};
	
	static const struct file_operations fih_vender_ver_proc_fops = {
	.owner	= THIS_MODULE,
	.open		= vender_ver_proc_open,
	.read		= seq_read,
	.write	= vender_ver_write_proc,
	};
	
	static const struct file_operations fih_product_model_proc_fops = {
	.owner	= THIS_MODULE,
	.open		= product_model_proc_open,
	.read		= seq_read,
	.write	= product_model_write_proc,
	};
	
	static const struct file_operations fih_build_type_proc_fops = {
	.owner	= THIS_MODULE,
	.open		= build_type_proc_open,
	.read		= seq_read,
	.write	= build_type_write_proc,
	};
/* CORE-HC-productinfo-00+] */

/* CORE-HC-Last_AMSS_Log-00+[ */
	static const struct file_operations fih_last_amsslog_proc_fops = {
	.owner	= THIS_MODULE,
	.open		= last_amsslog_proc_open,
	.read		= seq_read,
	};
/* CORE-HC-Last_AMSS_Log-00+] */

/***************************
  * Init FIH's PROC enrty            *
  ***************************/
void fih_proc_init(void)
{
	/* CORE-HC-Last_AMSS_Log-00+[ */
	unsigned int *pwron_cause_ptr;
	char *fatal_error_buffer_virt_addr;
	
	pwron_cause_ptr = get_pwron_cause_virt_addr();
	
	if ((*pwron_cause_ptr & MTD_PWR_ON_EVENT_MODEM_FATAL_ERROR) ||
		(*pwron_cause_ptr & MTD_PWR_ON_EVENT_MODEM_SW_WD_RESET) ||
		(*pwron_cause_ptr & MTD_PWR_ON_EVENT_MODEM_FW_WD_RESET))
	{
		printk("%s: Modem fatal error occurred. Create /proc/last_amsslog\r\n", __func__);
		if ((fatal_error_buffer_virt_addr =  (char *) get_fatal_error_buffer_virt_addr())) {
			if (!strncmp(fatal_error_buffer_virt_addr, MODEM_ERR_TAG, MODEM_ERR_TAG_LEN)) {
				modem_error_log_ptr = &fatal_error_buffer_virt_addr[MODEM_ERR_TAG_LEN];
				proc_create_data("last_amsslog", 0, NULL, &fih_last_amsslog_proc_fops, NULL);
			}
		}
		else{
			pr_err("%s: fatal_error_buffer_virt_addr is Null!\n", __func__);
		}
	}
	/* CORE-HC-Last_AMSS_Log-00+] */
	
	/* Device Model */
	proc_create_data("devmodel", 0, NULL, &fih_devmodel_proc_fops, NULL);
	
	/* Device Phase ID */
	proc_create_data("phaseid", 0, NULL, &fih_phaseid_proc_fops, NULL);

	/* Band Information */
	proc_create_data("bandinfo", 0, NULL, &fih_bandinfo_proc_fops, NULL);

	/* SIM Information */
	proc_create_data("siminfo", 0, NULL, &fih_siminfo_proc_fops, NULL);
 
	/* NON-HLOS image Version */
	proc_create_data("nonHLOS_ver", 0, NULL, &fih_nonHLOS_ver_proc_fops, NULL);

	/* HLOS image Version */
	proc_create_data("HLOS_ver", 0, NULL, &fih_HLOS_ver_proc_fops, NULL);

	/* NON-HLOS git haed number */
	proc_create_data("nonHLOS_git_head", 0, NULL, &fih_nonHLOS_githd_proc_fops, NULL);

	/* HLOS git haed number */
	proc_create_data("HLOS_git_head", 0, NULL, &fih_HLOS_githd_proc_fops, NULL);

	/* CORE-HC-productinfo-00+[ */
	proc_create_data("official_ver", 0, NULL, &fih_official_ver_proc_fops, NULL);
	proc_create_data("vender_ver", 0, NULL, &fih_vender_ver_proc_fops, NULL);
	proc_create_data("product_model", 0, NULL, &fih_product_model_proc_fops, NULL);
	proc_create_data("build_type", 0, NULL, &fih_build_type_proc_fops, NULL);
	/* CORE-HC-productinfo-00+] */

}
EXPORT_SYMBOL(fih_proc_init);
