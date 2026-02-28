/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __UBWCP_HW_H_
#define __UBWCP_HW_H_

#define HW_BUFFER_FORMAT_RGBA	0x0
#define HW_BUFFER_FORMAT_NV12	0x2
#define HW_BUFFER_FORMAT_NV124R	0x4
#define HW_BUFFER_FORMAT_P010	0x6
#define HW_BUFFER_FORMAT_TP10	0x8
#define HW_BUFFER_FORMAT_P016	0xA
#define HW_BUFFER_FORMAT_LINEAR	0xF

/* interrupt id. also bit location for set/clear */
#define INTERRUPT_READ_ERROR   0
#define INTERRUPT_WRITE_ERROR  1
#define INTERRUPT_DECODE_ERROR 2
#define INTERRUPT_ENCODE_ERROR 3

/**
 * struct msm_ubwcp_- UBWCP hardware instance
 * dev:UBWCP device
 * irq:Interrupt number
 * clk:The bus clock for this IOMMU hardware instance
 * pclk:The clock for the IOMMU IOMMU bus interconnect
 */
struct ubwcp_dev {
	void __iomem *base;
	struct device *dev;
	int irq;
	struct clk *clk;
	struct clk *pclk;
	/* TBD:
	 * struct list_head dev_node;
	 * struct list_head dom_node;
	 * struct list_head ctx_list;
	 * DECLARE_BITMAP(context_map, IOMMU_MAX_CBS)
	 * struct iommu_device iommu;
	 */
};


struct __packed ubwcp_hw_meta_metadata {
	u64 uv_start_addr : 48;	/* uv start address */
	u16 format : 16;	/* format */
	u16 stride;		/* image stride (bytes) */
	u16 stride_ubwcp;	/* p010 stride for tp10 image (bytes) */
	u32 metadata_base_y;	/* 24-bit page address */
	u32 metadata_base_uv;	/* 24-bit page address */
	u16 buffer_y_offset;	/* 4KB offset from meta_data_base_y */
	u16 buffer_uv_offset;	/* 4KB offset from meta_data_base_y */
	u32 width_height;	/* image width (bytes) */
};

void ubwcp_hw_version(void __iomem *base, u32 *major, u32 *minor);
void ubwcp_hw_set_buf_desc(void __iomem *base, u64 desc_addr, u16 desc_stride);
void ubwcp_hw_enable_range_check(void __iomem *base, u16 index);
int ubwcp_hw_disable_range_check_with_flush(void __iomem *base, u16 index);
void ubwcp_hw_set_range_check(void __iomem *base, u16 index, phys_addr_t pa, size_t size);
u64 ubwcp_hw_interrupt_src_address(void __iomem *base, u16 interrupt);
void ubwcp_hw_interrupt_clear(void __iomem *base, u16 interrupt);
void ubwcp_hw_interrupt_enable(void __iomem *base, u16 interrupt, bool enable);
void ubwcp_hw_power_on(void __iomem *pwr_ctrl, bool power_on);
void ubwcp_hw_one_time_init(void __iomem *base);
int ubwcp_hw_flush(void __iomem *base);
void ubwcp_hw_trace_set(bool value);
void ubwcp_hw_trace_get(bool *value);

#endif /* __UBWCP_HW_H_ */
