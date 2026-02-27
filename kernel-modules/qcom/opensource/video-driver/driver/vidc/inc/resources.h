/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020-2022, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _MSM_VIDC_RESOURCES_H_
#define _MSM_VIDC_RESOURCES_H_

struct icc_path;
struct regulator;
struct clk;
struct reset_control;
struct llcc_slice_desc;
struct iommu_domain;
struct device;
struct msm_vidc_core;

/*
 * These are helper macros to iterate over various lists within
 * msm_vidc_core->resource. The intention is to cut down on a lot
 * of boiler-plate code
 */

/* Read as "for each 'thing' in a set of 'thingies'" */
#define venus_hfi_for_each_thing(__device, __thing, __thingy) \
	venus_hfi_for_each_thing_continue(__device, __thing, __thingy, 0)

#define venus_hfi_for_each_thing_reverse(__device, __thing, __thingy) \
	venus_hfi_for_each_thing_reverse_continue(__device, __thing, __thingy, \
			(__device)->resource->__thingy##_set.count - 1)

/* TODO: the __from parameter technically not required since we can figure it
 * out with some pointer magic (i.e. __thing - __thing##_tbl[0]).  If this macro
 * sees extensive use, probably worth cleaning it up but for now omitting it
 * since it introduces unnecessary complexity.
 */
#define venus_hfi_for_each_thing_continue(__device, __thing, __thingy, __from) \
	for (__thing = &(__device)->resource->\
			__thingy##_set.__thingy##_tbl[__from]; \
		__thing < &(__device)->resource->__thingy##_set.__thingy##_tbl[0] + \
			((__device)->resource->__thingy##_set.count - __from); \
		++__thing)

#define venus_hfi_for_each_thing_reverse_continue(__device, __thing, __thingy, \
		__from) \
	for (__thing = &(__device)->resource->\
			__thingy##_set.__thingy##_tbl[__from]; \
		__thing >= &(__device)->resource->__thingy##_set.__thingy##_tbl[0]; \
		--__thing)

/* Bus set helpers */
#define venus_hfi_for_each_bus(__device, __binfo) \
	venus_hfi_for_each_thing(__device, __binfo, bus)
#define venus_hfi_for_each_bus_reverse(__device, __binfo) \
	venus_hfi_for_each_thing_reverse(__device, __binfo, bus)

/* Regular set helpers */
#define venus_hfi_for_each_regulator(__device, __rinfo) \
	venus_hfi_for_each_thing(__device, __rinfo, regulator)
#define venus_hfi_for_each_regulator_reverse(__device, __rinfo) \
	venus_hfi_for_each_thing_reverse(__device, __rinfo, regulator)
#define venus_hfi_for_each_regulator_reverse_continue(__device, __rinfo, \
		__from) \
	venus_hfi_for_each_thing_reverse_continue(__device, __rinfo, \
			regulator, __from)

/* Power domain set helpers */
#define venus_hfi_for_each_power_domain(__device, __pdinfo) \
	venus_hfi_for_each_thing(__device, __pdinfo, power_domain)

/* Clock set helpers */
#define venus_hfi_for_each_clock(__device, __cinfo) \
	venus_hfi_for_each_thing(__device, __cinfo, clock)
#define venus_hfi_for_each_clock_reverse(__device, __cinfo) \
	venus_hfi_for_each_thing_reverse(__device, __cinfo, clock)

/* Reset clock set helpers */
#define venus_hfi_for_each_reset_clock(__device, __rcinfo) \
	venus_hfi_for_each_thing(__device, __rcinfo, reset)
#define venus_hfi_for_each_reset_clock_reverse(__device, __rcinfo) \
	venus_hfi_for_each_thing_reverse(__device, __rcinfo, reset)
#define venus_hfi_for_each_reset_clock_reverse_continue(__device, __rinfo, \
		__from) \
	venus_hfi_for_each_thing_reverse_continue(__device, __rinfo, \
			reset, __from)

/* Subcache set helpers */
#define venus_hfi_for_each_subcache(__device, __sinfo) \
	venus_hfi_for_each_thing(__device, __sinfo, subcache)
#define venus_hfi_for_each_subcache_reverse(__device, __sinfo) \
	venus_hfi_for_each_thing_reverse(__device, __sinfo, subcache)

/* Contextbank set helpers */
#define venus_hfi_for_each_context_bank(__device, __sinfo) \
	venus_hfi_for_each_thing(__device, __sinfo, context_bank)
#define venus_hfi_for_each_context_bank_reverse(__device, __sinfo) \
	venus_hfi_for_each_thing_reverse(__device, __sinfo, context_bank)

/* Device region set helper */
#define venus_hfi_for_each_device_region(__device, __sinfo) \
	venus_hfi_for_each_thing(__device, __sinfo, device_region)

enum msm_vidc_branch_mem_flags {
	MSM_VIDC_CLKFLAG_RETAIN_PERIPH,
	MSM_VIDC_CLKFLAG_NORETAIN_PERIPH,
	MSM_VIDC_CLKFLAG_RETAIN_MEM,
	MSM_VIDC_CLKFLAG_NORETAIN_MEM,
	MSM_VIDC_CLKFLAG_PERIPH_OFF_SET,
	MSM_VIDC_CLKFLAG_PERIPH_OFF_CLEAR,
};

struct bus_info {
	struct icc_path           *icc;
	const char                *name;
	u32                        min_kbps;
	u32                        max_kbps;
};

struct bus_set {
	struct bus_info           *bus_tbl;
	u32                        count;
};

struct regulator_info {
	struct regulator          *regulator;
	const char                *name;
	bool                       hw_power_collapse;
};

struct regulator_set {
	struct regulator_info     *regulator_tbl;
	u32                        count;
};

struct power_domain_info {
	struct device             *genpd_dev;
	const char                *name;
};

struct power_domain_set {
	struct power_domain_info  *power_domain_tbl;
	u32                        count;
};

struct clock_residency {
	struct list_head           list;
	u64                        rate;
	u64                        start_time_us;
	u64                        total_time_us;
};

struct clock_info {
	struct clk                *clk;
	const char                *name;
	u32                        clk_id;
	bool                       has_scaling;
	u64                        prev;
#ifdef CONFIG_MSM_MMRM
	struct mmrm_client        *mmrm_client;
#endif
	struct list_head           residency_list;  /* list of struct clock_residency */
};

struct clock_set {
	struct clock_info         *clock_tbl;
	u32                        count;
};

struct reset_info {
	struct reset_control      *rst;
	const char                *name;
	bool                       exclusive_release;
};

struct reset_set {
	struct reset_info         *reset_tbl;
	u32                        count;
};

struct subcache_info {
	struct llcc_slice_desc    *subcache;
	const char                *name;
	u32                        llcc_id;
	bool                       isactive;
};

struct subcache_set {
	struct subcache_info      *subcache_tbl;
	u32                        count;
	bool                       set_to_fw;
};

struct addr_range {
	u32                        start;
	u32                        size;
};

struct context_bank_info {
	const char                *name;
	struct addr_range          addr_range;
	bool                       secure;
	bool                       dma_coherant;
	struct device             *dev;
	struct iommu_domain       *domain;
	u32                        region;
	u64                        dma_mask;
};

struct context_bank_set {
	struct context_bank_info  *context_bank_tbl;
	u32                        count;
};

struct frequency_table {
	unsigned long freq;
};

struct freq_set {
	struct frequency_table    *freq_tbl;
	u32                        count;
};

struct device_region_info {
	const char          *name;
	phys_addr_t          phy_addr;
	u32                  size;
	u32                  dev_addr;
	u32                  region;
};

struct device_region_set {
	struct device_region_info  *device_region_tbl;
	u32                         count;
};

struct msm_vidc_resource {
	u8 __iomem                *register_base_addr;
	u32                        irq;
	struct bus_set             bus_set;
	struct regulator_set       regulator_set;
	struct power_domain_set    power_domain_set;
	struct clock_set           clock_set;
	struct reset_set           reset_set;
	struct subcache_set        subcache_set;
	struct context_bank_set    context_bank_set;
	struct freq_set            freq_set;
	struct device_region_set   device_region_set;
	int                        fw_cookie;
};

#define call_res_op(c, op, ...)                  \
	(((c) && (c)->res_ops && (c)->res_ops->op) ? \
	((c)->res_ops->op(__VA_ARGS__)) : 0)

struct msm_vidc_resources_ops {
	int (*init)(struct msm_vidc_core *core);

	int (*reset_bridge)(struct msm_vidc_core *core);
	int (*reset_control_acquire)(struct msm_vidc_core *core,
				     const char *name);
	int (*reset_control_release)(struct msm_vidc_core *core,
				     const char *name);
	int (*reset_control_assert)(struct msm_vidc_core *core,
				    const char *name);
	int (*reset_control_deassert)(struct msm_vidc_core *core,
				      const char *name);

	int (*gdsc_init)(struct msm_vidc_core *core);
	int (*gdsc_on)(struct msm_vidc_core *core, const char *name);
	int (*gdsc_off)(struct msm_vidc_core *core, const char *name);
	int (*gdsc_hw_ctrl)(struct msm_vidc_core *core);
	int (*gdsc_sw_ctrl)(struct msm_vidc_core *core);

	int (*llcc)(struct msm_vidc_core *core, bool enable);
	int (*set_bw)(struct msm_vidc_core *core, unsigned long bw_ddr,
		      unsigned long bw_llcc);
	int (*set_clks)(struct msm_vidc_core *core, u64 rate);

	int (*clk_disable)(struct msm_vidc_core *core, const char *name);
	int (*clk_enable)(struct msm_vidc_core *core, const char *name);
	int (*clk_set_flag)(struct msm_vidc_core *core,
			    const char *name,
			    enum msm_vidc_branch_mem_flags flag);
	int (*clk_print_residency_stats)(struct msm_vidc_core *core);
	int (*clk_reset_residency_stats)(struct msm_vidc_core *core);
	int (*clk_update_residency_stats)(struct msm_vidc_core *core,
					  struct clock_info *cl, u64 rate);
};

const struct msm_vidc_resources_ops *get_resources_ops(void);

#endif
