/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef WSA884X_INTERNAL_H
#define WSA884X_INTERNAL_H

#include <asoc/wcd-irq.h>
#include "wsa884x.h"
#include "wsa884x-registers.h"

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/uaccess.h>

#define SWR_SLV_MAX_REG_ADDR	0x2009
#define SWR_SLV_START_REG_ADDR	0x40
#define SWR_SLV_MAX_BUF_LEN	20
#define BYTES_PER_LINE		12
#define SWR_SLV_RD_BUF_LEN	8
#define SWR_SLV_WR_BUF_LEN	32
#define SWR_SLV_MAX_DEVICES	2
#endif /* CONFIG_DEBUG_FS */

#define WSA884X_DRV_NAME "wsa884x-codec"
#define WSA884X_NUM_RETRY	5

#define WSA884X_VERSION_ENTRY_SIZE 32
#define WSA884X_VARIANT_ENTRY_SIZE 32

#define WSA884X_VERSION_1_0 0

enum {
	G_21_DB = 0,
	G_19P5_DB,
	G_18_DB,
	G_16P5_DB,
	G_15_DB,
	G_13P5_DB,
	G_12_DB,
	G_10P5_DB,
	G_9_DB,
	G_7P5_DB,
	G_6_DB,
	G_4P5_DB,
	G_3_DB,
	G_1P5_DB,
	G_0_DB,
	G_M1P5_DB,
	G_M3_DB,
	G_M4P5_DB,
	G_M6_DB,
	G_MAX_DB,
};

enum {
	PA_AUX_DISABLE = 0,
	PA_AUX_M6_DB =   3,
	PA_AUX_M4P5_DB = 4,
	PA_AUX_M3_DB =   5,
	PA_AUX_M1P5_DB = 6,
	PA_AUX_0_DB =    7,
	PA_AUX_7P5_DB =  8,
	PA_AUX_12_DB =   9,
	PA_AUX_18_DB = 0xA,
};

enum {
	ISENSE_6_DB = 0,
	ISENSE_12_DB,
	ISENSE_15_DB,
	ISENSE_18_DB
};

enum {
	VSENSE_M12_DB = 0,
	VSENSE_M15_DB,
	VSENSE_M18_DB,
	VSENSE_M21_DB,
	VSENSE_M24_DB
};

enum {
	DISABLE = 0,
	ENABLE,
};

enum {
	SPEAKER,
	RECEIVER,
	MAX_DEV_MODE
};

enum {
	SWR_DAC_PORT = 0,
	SWR_COMP_PORT,
	SWR_BOOST_PORT,
	SWR_PBR_PORT,
	SWR_VISENSE_PORT,
	SWR_CPS_PORT
};

enum {
	EXT_ABOVE_3S,
	CONFIG_1S,
	CONFIG_2S,
	CONFIG_3S,
	EXT_1S,
	EXT_2S,
	EXT_3S,
	CONFIG_MAX,
};

enum {
	WSA_4_OHMS = 0,
	WSA_6_OHMS,
	WSA_8_OHMS,
	WSA_32_OHMS,
	WSA_MAX_OHMS,
};

struct wsa_ctrl_platform_data {
	void *handle;
	int (*update_wsa_event)(void *handle, u16 event, u32 data);
	int (*register_notifier)(void *handle, struct notifier_block *nblock,
				bool enable);
};

struct swr_port {
	u8 port_id;
	u8 ch_mask;
	u32 ch_rate;
	u8 num_ch;
	u8 port_type;
};

extern struct regmap_config wsa884x_regmap_config;

/* Aux gain from system gain */
static const u8 pa_aux_no_comp[G_MAX_DB] = {
	PA_AUX_18_DB,  /* G_21_DB   */
	PA_AUX_18_DB,  /* G_19P5_DB */
	PA_AUX_18_DB,  /* G_18_DB   */
	PA_AUX_18_DB,  /* G_16P5_DB */
	PA_AUX_18_DB,  /* G_15_DB   */
	PA_AUX_12_DB,  /* G_13P5_DB */
	PA_AUX_12_DB,  /* G_12_DB   */
	PA_AUX_12_DB,  /* G_10P5_DB */
	PA_AUX_7P5_DB, /* G_9_DB    */
	PA_AUX_7P5_DB, /* G_7P5_DB  */
	PA_AUX_7P5_DB, /* G_6_DB    */
	PA_AUX_7P5_DB, /* G_4P5_DB  */
	PA_AUX_0_DB,   /* G_3_DB    */
	PA_AUX_0_DB,   /* G_1P5_DB  */
	PA_AUX_0_DB,   /* G_0_DB    */
	PA_AUX_M1P5_DB,/* G_M1P5_DB */
	PA_AUX_M3_DB,  /* G_M3_DB   */
	PA_AUX_M4P5_DB,/* G_M4P5_DB */
	PA_AUX_M6_DB   /* G_M6_DB   */
};

/*
 * Isense data indexed by system_gain and rload
 * WSA_4_OHMS, WSA_6_OHMS, WSA_8_OHMS, WSA_32_OHMS
 */
static const u8 isense_gain_data[G_MAX_DB][WSA_MAX_OHMS] = {
	{ISENSE_6_DB, ISENSE_6_DB, ISENSE_12_DB, ISENSE_18_DB},  /*G_21_DB   */
	{ISENSE_6_DB, ISENSE_6_DB, ISENSE_12_DB, ISENSE_18_DB},  /*G_19P5_DB */
	{ISENSE_6_DB, ISENSE_6_DB, ISENSE_15_DB, ISENSE_18_DB},  /*G_18_DB   */
	{ISENSE_12_DB, ISENSE_12_DB, ISENSE_15_DB, ISENSE_18_DB}, /*G_16P5_DB */
	{ISENSE_12_DB, ISENSE_12_DB, ISENSE_18_DB, ISENSE_18_DB}, /*G_15_DB   */
	{ISENSE_12_DB, ISENSE_12_DB, ISENSE_18_DB, ISENSE_18_DB}, /*G_13P5_DB */
	{ISENSE_12_DB, ISENSE_12_DB, ISENSE_18_DB, ISENSE_18_DB}, /*G_12_DB   */
	{ISENSE_12_DB, ISENSE_12_DB, ISENSE_18_DB, ISENSE_18_DB}, /*G_10P5_DB */
	{ISENSE_12_DB, ISENSE_12_DB, ISENSE_18_DB, ISENSE_18_DB}, /*G_9_DB    */
	{ISENSE_12_DB, ISENSE_12_DB, ISENSE_18_DB, ISENSE_18_DB}, /*G_7P5_DB  */
	{ISENSE_12_DB, ISENSE_12_DB, ISENSE_18_DB, ISENSE_18_DB}, /*G_6_DB    */
	{ISENSE_12_DB, ISENSE_12_DB, ISENSE_18_DB, ISENSE_18_DB}, /*G_4P5_DB  */
	{ISENSE_12_DB, ISENSE_12_DB, ISENSE_18_DB, ISENSE_18_DB}, /*G_3_DB    */
	{ISENSE_12_DB, ISENSE_12_DB, ISENSE_18_DB, ISENSE_18_DB}, /*G_1P5_DB  */
	{ISENSE_12_DB, ISENSE_12_DB, ISENSE_18_DB, ISENSE_18_DB}, /*G_0_DB    */
	{ISENSE_12_DB, ISENSE_12_DB, ISENSE_18_DB, ISENSE_18_DB}, /*G_M1P5_DB */
	{ISENSE_12_DB, ISENSE_12_DB, ISENSE_18_DB, ISENSE_18_DB}, /*G_M3_DB   */
	{ISENSE_12_DB, ISENSE_12_DB, ISENSE_18_DB, ISENSE_18_DB}, /*G_M4P5_DB */
	{ISENSE_12_DB, ISENSE_12_DB, ISENSE_18_DB, ISENSE_18_DB}, /*G_M6_DB   */
};

/* Vsense gain from system gain */
static const u8 vsense_gain_data[G_MAX_DB] = {
	VSENSE_M24_DB, /* G_21_DB   */
	VSENSE_M24_DB, /* G_19P5_DB */
	VSENSE_M21_DB, /* G_18_DB   */
	VSENSE_M21_DB, /* G_16P5_DB */
	VSENSE_M18_DB, /* G_15_DB   */
	VSENSE_M18_DB, /* G_13P5_DB */
	VSENSE_M15_DB, /* G_12_DB   */
	VSENSE_M15_DB, /* G_10P5_DB */
	VSENSE_M12_DB, /* G_9_DB    */
	VSENSE_M12_DB, /* G_7P5_DB  */
	VSENSE_M12_DB, /* G_6_DB    */
	VSENSE_M12_DB, /* G_4P5_DB  */
	VSENSE_M12_DB, /* G_3_DB    */
	VSENSE_M12_DB, /* G_1P5_DB  */
	VSENSE_M12_DB, /* G_0_DB    */
	VSENSE_M12_DB, /* G_M1P5_DB */
	VSENSE_M12_DB, /* G_M3_DB   */
	VSENSE_M12_DB, /* G_M4P5_DB */
	VSENSE_M12_DB  /* G_M6_DB   */
};

/*
 * PBR Thresholds from system_gain, bat_cfg, and rload
 * EXT_ABOVE_3S: WSA_4_OHMS, WSA_6_OHMS, WSA_8_OHMS, WSA_32_OHMS, CONFIG_1S: ...
 * Values are X100 to avoid 'float' issues, divided in WSA884X_VTH_TO_REG macro
 */
static const int pbr_vth1_data[G_MAX_DB][CONFIG_MAX][WSA_MAX_OHMS] = {
	/* G_21_DB */
	{
		{0, 0, 0, 0}, {2000, 2000, 863, 0},
		{2000, 2000, 1098, 0}, {2000, 2000, 1404, 0}
	},
	/* G_19P5_DB */
	{
		{0, 0, 0, 0}, {2000, 769, 2000, 0},
		{2000, 957, 2000, 0}, {2000, 1216, 2000, 0}
	},
	/* G_18_DB */
	{
		{0, 0, 0, 0}, {643, 2000, 2000, 0},
		{808, 2000, 2000, 0}, {1027, 2000, 2000, 0}
	},
	/* G_16P5_DB */
	{
		{0, 0, 0, 0}, {1575, 1575, 1575, 0},
		{1575, 1575, 1575, 0}, {1575, 1575, 1575, 0}
	},
	/* G_15_DB */
	{
		{0, 0, 0, 0}, {1400, 1400, 1400, 0},
		{1400, 1400, 1400, 0}, {1400, 1400, 1400, 0}
	},
	 /* G_13P5_DB */
	{
		{0, 0, 0, 0}, {1200, 1200, 1200, 0},
		{1200, 1200, 1200, 0}, {1200, 1200, 1200, 0}
	},
	/* G_12_DB */
	{
		{0, 0, 0, 0}, {1050, 1050, 1050, 0},
		{1050, 1050, 1050, 0}, {1050, 1050, 1050, 0}
	},
};
static const int pbr_vth2_data[G_MAX_DB][CONFIG_MAX][WSA_MAX_OHMS] = {
	{ {0, 0, 0, 0}, {0, 0, 918, 0}, {0, 0, 1161, 0}, {0, 0, 1443, 0} }, /* G_21_DB   */
	{ {0, 0, 0, 0}, {0, 824, 0, 0}, {0, 988, 0, 0},  {0, 1231, 0, 0} }, /* G_19P5_DB */
	{ {0, 0, 0, 0}, {667, 0, 0, 0}, {839, 0, 0, 0},  {1035, 0, 0, 0} }, /* G_18_DB   */
};

static const int pbr_vth3_data[G_MAX_DB][CONFIG_MAX][WSA_MAX_OHMS] = {
	{ {0, 0, 0, 0}, {0, 0, 980, 0}, {0, 0, 1224, 0}, {0, 0, 1482, 0} }, /* G_21_DB   */
	{ {0, 0, 0, 0}, {0, 878, 0, 0}, {0, 1051, 0, 0}, {0, 1278, 0, 0} }, /* G_19P5_DB */
	{ {0, 0, 0, 0}, {722, 0, 0, 0}, {894, 0, 0, 0},  {1075, 0, 0, 0} }, /* G_18_DB   */
};

static const int pbr_vth4_data[G_MAX_DB][CONFIG_MAX][WSA_MAX_OHMS] = {
	{ {0, 0, 0, 0}, {0, 0, 1043, 0}, {0, 0, 1278, 0}, {0, 0, 1522, 0} }, /* G_21_DB   */
	{ {0, 0, 0, 0}, {0, 933, 0, 0},  {0, 1082, 0, 0}, {0, 1294, 0, 0} }, /* G_19P5_DB */
	{ {0, 0, 0, 0}, {753, 0, 0, 0},  {925, 0, 0, 0},  {1090, 0, 0, 0} }, /* G_18_DB   */
};

static const int pbr_vth5_data[G_MAX_DB][CONFIG_MAX][WSA_MAX_OHMS] = {
	{ {0, 0, 0, 0}, {0, 0, 1098, 0}, {0, 0, 1341, 0}, {0, 0, 1561, 0} }, /* G_21_DB   */
	{ {0, 0, 0, 0}, {0, 988, 0, 0},  {0, 1137, 0, 0}, {0, 1333, 0, 0} }, /* G_19P5_DB */
	{ {0, 0, 0, 0}, {800, 0, 0, 0},  {973, 0, 0, 0},  {1129, 0, 0, 0} }, /* G_18_DB   */
};

static const int pbr_vth6_data[G_MAX_DB][CONFIG_MAX][WSA_MAX_OHMS] = {
	{ {0, 0, 0, 0}, {0, 0, 1137, 0}, {0, 0, 1380, 0}, {0, 0, 1584, 0} }, /* G_21_DB   */
	{ {0, 0, 0, 0}, {0, 1027, 0, 0}, {0, 1176, 0, 0}, {0, 1349, 0, 0} }, /* G_19P5_DB */
	{ {0, 0, 0, 0}, {831, 0, 0, 0},  {996, 0, 0, 0},  {1137, 0, 0, 0} }, /* G_18_DB   */
};

static const int pbr_vth7_data[G_MAX_DB][CONFIG_MAX][WSA_MAX_OHMS] = {
	{ {0, 0, 0, 0}, {0, 0, 1184, 0}, {0, 0, 1420, 0}, {0, 0, 1600, 0} }, /* G_21_DB   */
	{ {0, 0, 0, 0}, {0, 1059, 0, 0}, {0, 1208, 0, 0}, {0, 1365, 0, 0} }, /* G_19P5_DB */
	{ {0, 0, 0, 0}, {918, 0, 0, 0},  {1051, 0, 0, 0}, {1153, 0, 0, 0} }, /* G_18_DB   */
};

static const int pbr_vth8_data[G_MAX_DB][CONFIG_MAX][WSA_MAX_OHMS] = {
	{ {0, 0, 0, 0}, {0, 0, 1239, 0}, {0, 0, 1482, 0}, {0, 0, 1639, 0} }, /* G_21_DB   */
	{ {0, 0, 0, 0}, {0, 1114, 0, 0}, {0, 1263, 0, 0}, {0, 1380, 0, 0} }, /* G_19P5_DB */
	{ {0, 0, 0, 0}, {988, 0, 0, 0},  {1114, 0, 0, 0}, {1192, 0, 0, 0} }, /* G_18_DB   */
};

static const int pbr_vth9_data[G_MAX_DB][CONFIG_MAX][WSA_MAX_OHMS] = {
	{ {0, 0, 0, 0}, {0, 0, 1278, 0}, {0, 0, 1522, 0}, {0, 0, 1663, 0} }, /* G_21_DB   */
	{ {0, 0, 0, 0}, {0, 1192, 0, 0}, {0, 1302, 0, 0}, {0, 1420, 0, 0} }, /* G_19P5_DB */
	{ {0, 0, 0, 0}, {1051, 0, 0, 0}, {1184, 0, 0, 0}, {1224, 0, 0, 0} }, /* G_18_DB   */
};

static const int pbr_vth10_data[G_MAX_DB][CONFIG_MAX][WSA_MAX_OHMS] = {
	{ {0, 0, 0, 0}, {0, 0, 1380, 0}, {0, 0, 1600, 0}, {0, 0, 1686, 0} }, /* G_21_DB   */
	{ {0, 0, 0, 0}, {0, 1286, 0, 0}, {0, 1404, 0, 0}, {0, 1459, 0, 0} }, /* G_19P5_DB */
	{ {0, 0, 0, 0}, {1145, 0, 0, 0}, {1255, 0, 0, 0}, {1278, 0, 0, 0} }, /* G_18_DB   */
};

static const int pbr_vth11_data[G_MAX_DB][CONFIG_MAX][WSA_MAX_OHMS] = {
	{ {0, 0, 0, 0}, {0, 0, 1482, 0}, {0, 0, 1686, 0}, {0, 0, 1725, 0} }, /* G_21_DB   */
	{ {0, 0, 0, 0}, {0, 1357, 0, 0}, {0, 1498, 0, 0}, {0, 1498, 0, 0} }, /* G_19P5_DB */
	{ {0, 0, 0, 0}, {1208, 0, 0, 0}, {1318, 0, 0, 0}, {1341, 0, 0, 0} }, /* G_18_DB   */
};

static const int pbr_vth12_data[G_MAX_DB][CONFIG_MAX][WSA_MAX_OHMS] = {
	{ {0, 0, 0, 0}, {0, 0, 1584, 0}, {0, 0, 1765, 0}, {0, 0, 1780, 0} }, /* G_21_DB   */
	{ {0, 0, 0, 0}, {0, 1514, 0, 0}, {0, 1600, 0, 0}, {0, 1529, 0, 0} }, /* G_19P5_DB */
	{ {0, 0, 0, 0}, {1427, 0, 0, 0}, {1467, 0, 0, 0}, {1404, 0, 0, 0} }, /* G_18_DB   */
};

static const int pbr_vth13_data[G_MAX_DB][CONFIG_MAX][WSA_MAX_OHMS] = {
	{ {0, 0, 0, 0}, {0, 0, 1663, 0}, {0, 0, 1843, 0}, {0, 0, 1867, 0} }, /* G_21_DB   */
	{ {0, 0, 0, 0}, {0, 1671, 0, 0}, {0, 1702, 0, 0}, {0, 1569, 0, 0} }, /* G_19P5_DB */
	{ {0, 0, 0, 0}, {1561, 0, 0, 0}, {1616, 0, 0, 0}, {1475, 0, 0, 0} }, /* G_18_DB   */
};

static const int pbr_vth14_data[G_MAX_DB][CONFIG_MAX][WSA_MAX_OHMS] = {
	{ {0, 0, 0, 0}, {0, 0, 1780, 0}, {0, 0, 1922, 0}, {0, 0, 1945, 0} }, /* G_21_DB   */
	{ {0, 0, 0, 0}, {0, 1827, 0, 0}, {0, 1835, 0, 0}, {0, 1631, 0, 0} }, /* G_19P5_DB */
	{ {0, 0, 0, 0}, {1741, 0, 0, 0}, {1788, 0, 0, 0}, {1537, 0, 0, 0} }, /* G_18_DB   */
};

static const int pbr_vth15_data[G_MAX_DB][CONFIG_MAX][WSA_MAX_OHMS] = {
	{ {0, 0, 0, 0}, {0, 0, 2000, 0}, {0, 0, 2000, 0}, {0, 0, 2000, 0} }, /* G_21_DB   */
	{ {0, 0, 0, 0}, {0, 2000, 0, 0}, {0, 2000, 0, 0}, {0, 1686, 0, 0} }, /* G_19P5_DB */
	{ {0, 0, 0, 0}, {2000, 0, 0, 0}, {2000, 0, 0, 0}, {1600, 0, 0, 0} }, /* G_18_DB   */
};


/*
 * Private data Structure for wsa884x. All parameters related to
 * WSA884X codec needs to be defined here.
 */
struct wsa884x_priv {
	struct regmap *regmap;
	struct device *dev;
	struct swr_device *swr_slave;
	struct snd_soc_component *component;
	bool comp_enable;
	bool visense_enable;
	bool cps_enable;
	bool pbr_enable;
	bool ext_vdd_spk;
	bool dapm_bias_off;
	struct swr_port port[WSA884X_MAX_SWR_PORTS];
	int global_pa_cnt;
	int dev_mode;
	int comp_offset;
	struct mutex res_lock;
	struct snd_info_entry *entry;
	struct snd_info_entry *version_entry;
	struct snd_info_entry *variant_entry;
	struct device_node *wsa_rst_np;
	int pa_mute;
	int curr_temp;
	int variant;
	int version;
	u8 pa_gain;
	u32 bat_cfg;
	u32 rload;
	u8 system_gain;
	u32 sys_gains[MAX_DEV_MODE * 2];
	bool default_dev_mode;
	int min_gain;
	int pa_aux_gain;
	u8 dev_index;
	struct irq_domain *virq;
	struct wcd_irq_info irq_info;
	struct swr_port_params wsa_port_params[SWR_UC_MAX][WSA884X_MAX_SWR_PORTS];
	struct swr_dev_frame_config swr_wsa_port_params[SWR_UC_MAX];
#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_dent;
	struct dentry *debugfs_peek;
	struct dentry *debugfs_poke;
	struct dentry *debugfs_reg_dump;
	unsigned int read_data;
#endif
	struct device_node *parent_np;
	struct device_node *macro_np;
	struct platform_device *parent_dev;
	struct platform_device *macro_dev;
	struct notifier_block parent_nblock;
	void *handle;
	int (*register_notifier)(void *handle,
				struct notifier_block *nblock, bool enable);
	struct cdc_regulator *regulator;
	int num_supplies;
	struct regulator_bulk_data *supplies;
	unsigned long status_mask;
	unsigned long port_status_mask;
	struct snd_soc_dai_driver *dai_driver;
	struct snd_soc_component_driver *driver;
	int noise_gate_mode;
};

#endif /* WSA884X_INTERNAL_H */
