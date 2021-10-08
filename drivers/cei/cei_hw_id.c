#include <linux/module.h>
#include <linux/cei_hw_id.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/kernel.h>

#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#include <soc/qcom/smsm.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#define CEI_PHASE_NAME_STRLEN 8
#define MSM_TLMM_GPIO14_BASE 0x0390E000
#define MSM_TLMM_SIZE 0x72000
#define MSM_TLMM_REG_SIZE 0x1000
#define FPC_DETECT_GPIO 14
static char ceiprojectname[HARDWARE_INFO_COMMANDLINE_LENGTH];
static char ceiphasename[HARDWARE_INFO_COMMANDLINE_LENGTH];
static char ceiskuname[HARDWARE_INFO_COMMANDLINE_LENGTH];
static char ceisimslotname[HARDWARE_INFO_COMMANDLINE_LENGTH];
static char ceimbname[HARDWARE_INFO_COMMANDLINE_LENGTH];
static char ceifpname[HARDWARE_INFO_COMMANDLINE_LENGTH];
static char ceiandroidbootmode[HARDWARE_INFO_COMMANDLINE_LENGTH];

static const char *cei_phase_name[] = {
	"PDP",
	"DP",
	"SP",
	"AP1",
	"AP2"
	"TP",
	"PQ",
	"MP",
	"PHASE_NAME_INVALID"
};

static const char *cei_fp_name[] = {
	"fpc1035",
	"et516",
	"null"
};

static const char *cei_simslot_name[] = {
	"ss",
	"dsds",
};

struct hardware_info_node {
	const char *filename;
	const struct file_operations fops;
};

static int cei_phase_id_read(struct seq_file *m, void *v)
{
	if (!strncmp(ceiphasename, "", strlen(ceiphasename))) {
		pr_info(KERN_INFO "CEI PHASE ID in invalid\n");
		seq_printf(m, "%s\n", "NULL");
	} else {
		seq_printf(m, "%s\n", ceiphasename);
	}

	return 0;
}

static int cei_phase_id_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cei_phase_id_read, NULL);
}

static int cei_project_id_read(struct seq_file *m, void *v)
{
	if (!strncmp(ceiprojectname, "", strlen(ceiprojectname))) {
		pr_info(KERN_INFO "CEI PROJECT ID in invalid\n");
		seq_printf(m, "%s\n", "NULL");
	} else {
		seq_printf(m, "%s\n", ceiprojectname);
	}

	return 0;
}

static int cei_project_id_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cei_project_id_read, NULL);
}

static int cei_sku_id_read(struct seq_file *m, void *v)
{
	if (!strncmp(ceiskuname, "", strlen(ceiskuname))) {
		pr_info(KERN_INFO "CEI SKU ID in invalid\n");
		seq_printf(m, "%s\n", "NULL");
	} else {
		seq_printf(m, "%s\n", ceiskuname);
	}

	return 0;
}

static int cei_sku_id_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cei_sku_id_read, NULL);
}

static int cei_simslot_id_read(struct seq_file *m, void *v)
{
	if (!strncmp(ceisimslotname, "", strlen(ceisimslotname))) {
		pr_info(KERN_INFO "CEI SIMSLOT ID in invalid\n");
		seq_printf(m, "%s\n", "NULL");
	} else {
		seq_printf(m, "%s\n", ceisimslotname);
	}

	return 0;
}

static int cei_simslot_id_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cei_simslot_id_read, NULL);
}

static int cei_mb_id_read(struct seq_file *m, void *v)
{
	if (!strncmp(ceimbname, "", strlen(ceimbname))) {
		pr_info(KERN_INFO "CEI MAINBOARD ID in invalid\n");
		seq_printf(m, "%s\n", "NULL");
	} else {
		seq_printf(m, "%s\n", ceimbname);
	}

	return 0;
}

static int cei_fp_id_read(struct seq_file *m, void *v)
{
	if (!strncmp(ceifpname, "", strlen(ceifpname))) {
		pr_info(KERN_INFO "CEI FPMODULE ID in invalid\n");
		seq_printf(m, "%s\n", "NULL");
	} else {
		seq_printf(m, "%s\n", ceifpname);
	}

	return 0;
}

static void cei_fp_module_detect(void)
{
	int value;
	void __iomem *cfg_reg = NULL;
	uint32_t cei_current_dir;
	uint32_t cei_current_settings;

	pr_err("cei_fp module detect\n");

	cfg_reg = ioremap(MSM_TLMM_GPIO14_BASE, MSM_TLMM_SIZE);
	cei_current_settings = readl_relaxed(cfg_reg + 0x0);
	writel_relaxed(0, cfg_reg);
	msleep(100);
	cei_current_dir = readl_relaxed(cfg_reg + 0x4);
	value = cei_current_dir & 0x1;

	if (value) {
		pr_err("cei_fp module is fpc1035");
		strlcpy(ceifpname, cei_fp_name[0], sizeof(ceifpname));
		return;
	}
	strlcpy(ceifpname, cei_fp_name[1], sizeof(ceifpname));
	pr_err("cei_fp module is et516 or null");
	return;
}

static int cei_mb_id_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cei_mb_id_read, NULL);
}

static int cei_fp_id_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cei_fp_id_read, NULL);
}

static const struct hardware_info_node hw_binfo[] = {
	{
		.filename   = "cei_phase_id",
		.fops.open  = cei_phase_id_proc_open,
		.fops.read  = seq_read,
		.fops.release   = single_release,
		.fops.llseek    = seq_lseek,
	},
	{
		.filename   = "cei_project_id",
		.fops.open  = cei_project_id_proc_open,
		.fops.read  = seq_read,
		.fops.release   = single_release,
		.fops.llseek    = seq_lseek,
	},
	{
		.filename   = "cei_sku_id",
		.fops.open  = cei_sku_id_proc_open,
		.fops.read  = seq_read,
		.fops.release   = single_release,
		.fops.llseek    = seq_lseek,
	},
	{
		.filename   = "cei_simslot_id",
		.fops.open  = cei_simslot_id_proc_open,
		.fops.read  = seq_read,
		.fops.release   = single_release,
		.fops.llseek    = seq_lseek,
	},
	{
		.filename   = "cei_mainboard_id",
		.fops.open  = cei_mb_id_proc_open,
		.fops.read  = seq_read,
		.fops.release   = single_release,
		.fops.llseek    = seq_lseek,
	},
	{
		.filename   = "cei_fp_id",
		.fops.open  = cei_fp_id_proc_open,
		.fops.read  = seq_read,
		.fops.release   = single_release,
		.fops.llseek    = seq_lseek,
	},
};

static int __init cei_phase_id_from_cmdline(char *str)
{
	strlcpy(ceiphasename, str, sizeof(ceiphasename));
	return 1;
}
__setup("cei_phase_id=", cei_phase_id_from_cmdline);

static int __init cei_project_id_from_cmdline(char *str)
{
	strlcpy(ceiprojectname, str, sizeof(ceiprojectname));
	return 1;
}
__setup("cei_project_id=", cei_project_id_from_cmdline);

static int __init cei_sku_id_from_cmdline(char *str)
{
	strlcpy(ceiskuname, str, sizeof(ceiskuname));
	return 1;
}
__setup("cei_sku_id=", cei_sku_id_from_cmdline);

static int __init cei_simslot_id_from_cmdline(char *str)
{
	strlcpy(ceisimslotname, str, sizeof(ceisimslotname));
	return 1;
}
__setup("cei_simslot_id=", cei_simslot_id_from_cmdline);

static int __init cei_mb_id_from_cmdline(char *str)
{
	strlcpy(ceimbname, str, sizeof(ceimbname));
	return 1;
}
__setup("cei_mainboard_id=", cei_mb_id_from_cmdline);

static int __init cei_android_boot_mode_from_cmdline(char *str)
{
	strlcpy(ceiandroidbootmode, str, sizeof(ceiandroidbootmode));
	return 1;
}
__setup("androidboot.mode=", cei_android_boot_mode_from_cmdline);

char *get_cei_phase_id(void)
{
	return ceiphasename;
}
EXPORT_SYMBOL(get_cei_phase_id);

char *get_cei_project_id(void)
{
	return ceiprojectname;
}
EXPORT_SYMBOL(get_cei_project_id);

char *get_cei_sku_id(void)
{
	return ceiskuname;
}
EXPORT_SYMBOL(get_cei_sku_id);

char *get_cei_simslot_id(void)
{
	return ceisimslotname;
}
EXPORT_SYMBOL(get_cei_simslot_id);

char *get_cei_mb_id(void)
{
	return ceimbname;
}
EXPORT_SYMBOL(get_cei_mb_id);

char *get_cei_fp_id(void)
{
	return ceifpname;
}
EXPORT_SYMBOL(get_cei_fp_id);

char *get_cei_android_boot_mode(void)
{
	return ceiandroidbootmode;
}
EXPORT_SYMBOL(get_cei_android_boot_mode);

int get_phase_name_index(char *name)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cei_phase_name); i++) {
		if (!strncmp(name, cei_phase_name[i], strlen(cei_phase_name[i])))
			return i;
	}
	pr_err("get_phase_name_index_error\n");
	return -EINVAL;
}
EXPORT_SYMBOL(get_phase_name_index);

int get_simslot_name_index(char *name)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cei_simslot_name); i++) {
		if (!strncmp(name, cei_simslot_name[i], strlen(cei_simslot_name[i])))
			return i;
	}
	pr_err("get_simslot_name_index_error\n");
	return -EINVAL;
}
EXPORT_SYMBOL(get_simslot_name_index);

static void create_hwinfo_device_node(void)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(hw_binfo); i++) {
		const struct hardware_info_node *f = &hw_binfo[i];

		if ((!strcmp(f->filename, "cei_phase_id")) || (!strcmp(f->filename, "cei_mainboard_id"))) {
			if (!proc_create(f->filename, S_IRUSR | S_IRGRP | S_IROTH, NULL, &f->fops))
				goto error;
		} else {
			if (!proc_create(f->filename, S_IRUSR | S_IRGRP, NULL, &f->fops))
				goto error;
		}
	}

	return;

error:
	while (--i >= 0) {
		const struct hardware_info_node *f = &hw_binfo[i];
		remove_proc_entry(f->filename, NULL);
	}
}

static int reg_set_load_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_load(reg, load_uA) : 0;
}

static int ceihwid_probe(struct platform_device *pdev);
static int ceihwid_remove(struct platform_device *pdev);

static struct of_device_id ceihwid_match_table[] = {
	{ .compatible = "ceihwid",},
	{},
};
MODULE_DEVICE_TABLE(of, ceihwid_match_table);

static struct platform_driver ceihwid_driver = {
	.driver = {
		.name		= "ceihwid",
		.owner		= THIS_MODULE,
		.of_match_table = ceihwid_match_table,
	},
    .probe =    ceihwid_probe,
    .remove =   ceihwid_remove,
};

static int __init cei_hw_id_init(void)
{
	int rc = platform_driver_register(&ceihwid_driver);

	if (!rc)
		pr_info("%s OK\n", __func__);
	else
		pr_err("%s %d\n", __func__, rc);

	create_hwinfo_device_node();

	return rc;
}

static void __exit cei_hw_id_exit(void)
{
	pr_info("%s\n", __func__);
	platform_driver_unregister(&ceihwid_driver);
	pr_info(KERN_INFO "cei_hwid_exit enter\n");
}

static struct regulator *vcc_ana;
static int ceihwid_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int error = 0;

	pr_err("cei hw id probe\n");
	if (of_find_property(np, "ceihwid_vcc-supply", NULL)) {
		vcc_ana = regulator_get(&pdev->dev, "ceihwid_vcc");
		if (IS_ERR(vcc_ana)) {
			error = PTR_ERR(vcc_ana);
			pr_err("%s: regulator get failed vcc_ana rc=%d\n",
				__func__, error);
			return error;
		}

		if (regulator_count_voltages(vcc_ana) > 0) {
			error = regulator_set_voltage(vcc_ana,
				3300000, 3300000);
			if (error) {
				pr_err("%s: regulator set vtg failed vcc_ana rc=%d\n",
					__func__, error);
				regulator_put(vcc_ana);
				return error;
			}
		}

		error = regulator_enable(vcc_ana);
		if (error) {
			pr_err("%s: Regulator vcc_ana enable failed rc=%d\n",
				__func__, error);
			reg_set_load_check(vcc_ana, 0);
			return error;
		}
	}
	cei_fp_module_detect();

	/* Disable l6 regulator */
	if (vcc_ana) {
		if (regulator_is_enabled(vcc_ana))
			regulator_disable(vcc_ana);
		regulator_put(vcc_ana);
	}
	return 0;
}

static int ceihwid_remove(struct platform_device *pdev)
{
	return 0;
}

subsys_initcall(cei_hw_id_init);
module_exit(cei_hw_id_exit);
MODULE_DESCRIPTION("cei hardware ID driver");
MODULE_LICENSE("GPL");
