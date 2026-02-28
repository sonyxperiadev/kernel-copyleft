// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#define pr_fmt(fmt) "%s: hw: %s(): " fmt, KBUILD_MODNAME, __func__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/dma-buf.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/qcom_scm.h>
#include <linux/delay.h>

#include "ubwcp_hw.h"

static bool ubwcp_hw_trace_en;
//#define DBG(fmt, args...)
#define DBG(fmt, args...) \
	do { \
		if (unlikely(ubwcp_hw_trace_en)) \
			pr_err(fmt "\n", ##args); \
	} while (0)
#define ERR(fmt, args...) pr_err_ratelimited(": %d: ~~~ERROR~~~: " fmt "\n", __LINE__, ##args)

MODULE_LICENSE("GPL");

#define PAGE_ADDR_4K(_x) ((_x) >> 12)

/* register offsets from base */
#define RANGE_LOWER			0x0000
#define RANGE_HIGHER			0x0800
#define DESC_BASE			0x1000
#define DESC_BASE_STRIDE		0x1004
#define CONFIG				0x1008
#define ENCODER_CONFIG			0x100C
#define ENCODER_STATUS			0x1010
#define DECODER_CONFIG			0x1014
#define DECODER_STATUS			0x1018
#define RANGE_CHECK_FAIL		0x101C
#define RANGE_CHECK_CONTROL		0x1020
#define RANGE_CHECK_STATUS		0x1060
#define FLUSH_CONTROL			0x10A0
#define FLUSH_STATUS			0x10A4
#define INTERRUPT_SET			0x10B0
#define INTERRUPT_STATUS_READ		0x10C0
#define INTERRUPT_STATUS_WRITE		0x10C4
#define INTERRUPT_STATUS_ENCODE		0x10C8
#define INTERRUPT_STATUS_DECODE		0x10CC
#define INTERRUPT_READ_SRC_LOW		0x1100
#define INTERRUPT_READ_SRC_HIGH		0x1104
#define INTERRUPT_WRITE_SRC_LOW		0x1108
#define INTERRUPT_WRITE_SRC_HIGH	0x110C
#define INTERRUPT_ENCODE_SRC_LOW	0x1110
#define INTERRUPT_ENCODE_SRC_HIGH	0x1114
#define INTERRUPT_DECODE_SRC_LOW	0x1118
#define INTERRUPT_DECODE_SRC_HIGH	0x111C
#define INTERRUPT_CLEAR			0x1120
#define QNS4_PARAMS			0x1124
#define OVERRIDE			0x112C
#define VERSION_CONTROL			0x1130
#define SPARE				0x1188


#define UBWCP_DEBUG_REG_RW

/* read/write register */
#if defined(UBWCP_DEBUG_REG_RW)
#define UBWCP_REG_READ(_base, _offset) \
			({u32 _reg; \
			_reg = ioread32(_base + _offset); \
			DBG("READ : 0x%x -> 0x%08x", _offset, _reg); \
			_reg; })

#define UBWCP_REG_WRITE(_base, _offset, _value) \
			{ \
			DBG("WRITE: 0x%x <- 0x%08x", _offset, _value); \
			iowrite32(_value, _base + _offset); \
			}
#else
#define UBWCP_REG_READ(_base, _offset)           ioread32(_base + _offset)
#define UBWCP_REG_WRITE(_base, _offset, _value)  iowrite32(_value, _base + _offset)
#endif

#define UBWCP_REG_READ_NO_DBG(_base, _offset)           ioread32(_base + _offset)
#define UBWCP_REG_WRITE_NO_DBG(_base, _offset, _value)  iowrite32(_value, _base + _offset)


void ubwcp_hw_interrupt_enable(void __iomem *base, u16 interrupt, bool enable)
{
	u32 value;

	value = UBWCP_REG_READ(base, INTERRUPT_SET);

	if (enable)
		value = value | (1 << interrupt);
	else
		value = value & ~(1 << interrupt);

	UBWCP_REG_WRITE(base, INTERRUPT_SET, value);
}
EXPORT_SYMBOL(ubwcp_hw_interrupt_enable);

void ubwcp_hw_interrupt_clear(void __iomem *base, u16 interrupt)
{
	UBWCP_REG_WRITE_NO_DBG(base, INTERRUPT_CLEAR, (1 << interrupt));
}
EXPORT_SYMBOL(ubwcp_hw_interrupt_clear);

int ubwcp_hw_interrupt_status(void __iomem *base, u16 interrupt)
{
	int value = -1;

	switch (interrupt) {
	case INTERRUPT_READ_ERROR:
		value = UBWCP_REG_READ(base, INTERRUPT_STATUS_READ) & 0x1;
		break;
	case INTERRUPT_WRITE_ERROR:
		value = UBWCP_REG_READ(base, INTERRUPT_STATUS_WRITE) & 0x1;
		break;
	case INTERRUPT_DECODE_ERROR:
		value = UBWCP_REG_READ(base, INTERRUPT_STATUS_DECODE) & 0x1;
		break;
	case INTERRUPT_ENCODE_ERROR:
		value = UBWCP_REG_READ(base, INTERRUPT_STATUS_ENCODE) & 0x1;
		break;
	default:
		/* TBD: fatal error? */
		break;
	}

	return value;
}

/* returns the address which caused this interrupt */
u64 ubwcp_hw_interrupt_src_address(void __iomem *base, u16 interrupt)
{
	u32 addr_low;
	u32 addr_high;

	switch (interrupt) {
	case INTERRUPT_READ_ERROR:
		addr_low  = UBWCP_REG_READ_NO_DBG(base, INTERRUPT_READ_SRC_LOW);
		addr_high = UBWCP_REG_READ_NO_DBG(base, INTERRUPT_READ_SRC_HIGH) & 0xF;
		break;
	case INTERRUPT_WRITE_ERROR:
		addr_low  = UBWCP_REG_READ_NO_DBG(base, INTERRUPT_WRITE_SRC_LOW);
		addr_high = UBWCP_REG_READ_NO_DBG(base, INTERRUPT_WRITE_SRC_HIGH) & 0xF;
		break;
	case INTERRUPT_DECODE_ERROR:
		addr_low  = UBWCP_REG_READ_NO_DBG(base, INTERRUPT_DECODE_SRC_LOW);
		addr_high = UBWCP_REG_READ_NO_DBG(base, INTERRUPT_DECODE_SRC_HIGH) & 0xF;
		break;
	case INTERRUPT_ENCODE_ERROR:
		addr_low  = UBWCP_REG_READ_NO_DBG(base, INTERRUPT_ENCODE_SRC_LOW);
		addr_high = UBWCP_REG_READ_NO_DBG(base, INTERRUPT_ENCODE_SRC_HIGH) & 0xF;
		break;
	default:
		/* TBD: fatal error? */
		addr_low = 0x0;
		addr_high = 0x0;
		break;
	}

	return ((addr_high << 31) | addr_low);
}
EXPORT_SYMBOL(ubwcp_hw_interrupt_src_address);

/*
 * @index: index of buffer (from 0 to 255)
 * @pa   : ULA PA start address
 * @size : size of ULA PA address range
 */
void ubwcp_hw_set_range_check(void __iomem *base, u16 index, phys_addr_t pa, size_t size)
{
	u32 lower;
	u32 higher;

	lower  = PAGE_ADDR_4K(pa);
	higher = PAGE_ADDR_4K(pa + size);

	UBWCP_REG_WRITE(base, RANGE_LOWER  + index*4, lower);
	UBWCP_REG_WRITE(base, RANGE_HIGHER + index*4, higher);
}
EXPORT_SYMBOL(ubwcp_hw_set_range_check);

/* enable range ck:
 * identify control register for this index.
 * 32bits in each ctrl reg. upto 8 regs for 256 indexes
 */
void ubwcp_hw_enable_range_check(void __iomem *base, u16 index)
{
	u32 val;
	u16 ctrl_reg = index >> 5;

	val = UBWCP_REG_READ(base, RANGE_CHECK_CONTROL + ctrl_reg*4);
	val |= (1 << (index & 0x1F));
	UBWCP_REG_WRITE(base, RANGE_CHECK_CONTROL + ctrl_reg*4, val);
}
EXPORT_SYMBOL(ubwcp_hw_enable_range_check);


/* Disable range check with flush */
int ubwcp_hw_disable_range_check_with_flush(void __iomem *base, u16 index)
{
	u32 flush_complete = 0;
	u32 count = 20;
	u32 val;
	u16 ctrl_reg = index >> 5;

	//assert flush
	UBWCP_REG_WRITE(base, FLUSH_CONTROL, 0x3);

	//poll for flush done
	do {
		flush_complete = UBWCP_REG_READ(base, FLUSH_STATUS) & 0x1;
		if (flush_complete) {
			//disable range ck
			val = UBWCP_REG_READ(base, RANGE_CHECK_CONTROL + ctrl_reg*4);
			val &= ~(1 << (index & 0x1F));
			UBWCP_REG_WRITE(base, RANGE_CHECK_CONTROL + ctrl_reg*4, val);

			//clear flush
			UBWCP_REG_WRITE(base, FLUSH_CONTROL, 0x0);
			return 0;
		}
		udelay(100);
	} while (count--);

	ERR("~~~~~ FLUSH FAILED ~~~~~");
	return -1;
}
EXPORT_SYMBOL(ubwcp_hw_disable_range_check_with_flush);

void ubwcp_hw_set_buf_desc(void __iomem *base, u64 desc_addr, u16 desc_stride)
{
	UBWCP_REG_WRITE(base, DESC_BASE, PAGE_ADDR_4K(desc_addr));
	UBWCP_REG_WRITE(base, DESC_BASE_STRIDE, desc_stride);
}
EXPORT_SYMBOL(ubwcp_hw_set_buf_desc);


/* Value set here is returned upon read of an address that fails range check.
 * Writes are ignored.
 * Will also generate range_check_fail interrupt if enabled.
 * if we don't program, default value is: 0x92929292
 */
void ubwcp_hw_set_default_range_check_value(void __iomem *base, u32 val)
{
	UBWCP_REG_WRITE(base, RANGE_CHECK_FAIL, val);
}


void ubwcp_hw_version(void __iomem *base, u32 *major, u32 *minor)
{
	u32 version;

	version = UBWCP_REG_READ(base, VERSION_CONTROL);
	*major = version & 0xF;
	*minor = (version & 0xF0) >> 4;
}
EXPORT_SYMBOL(ubwcp_hw_version);

/* TBD: */
void ubwcp_hw_macro_tile_config(void __iomem *base)
{
	//TODO: In future add in support for LP4
	//May be able to determine DDR version via call to
	//of_fdt_get_ddrtype()

	/*
	 * For Lanai assume 4 Channel LP5 DDR so from HSR
	 * MAL Size					32B
	 * Highest Bank Bit				16
	 * Level 1 Bank Swizzling			Disable
	 * Level 2 Bank Swizzling			Enable
	 * Level 3 Bank Swizzling			Enable
	 * Bank Spreading				Enable
	 * Macrotiling Configuration (Num Channels)	8
	 */
	UBWCP_REG_WRITE(base, CONFIG, 0x1E3);
}

/* TBD: */
void ubwcp_hw_decoder_config(void __iomem *base)
{
	/*
	 * For Lanai assume AMSBC (UBWC4.4/4.3) algorithm is used == b11
	 * For Lanai assume 4 Channel LP5 DDR so MAL Size 32B == b0
	 */
	UBWCP_REG_WRITE(base, DECODER_CONFIG, 0x7);
}

/* TBD: */
void ubwcp_hw_encoder_config(void __iomem *base)
{
	/*
	 * For Lanai assume AMSBC (UBWC4.4/4.3) algorithm is used == b11
	 * For Lanai assume 4 Channel LP5 DDR so MAL Size 32B == b0
	 */
	UBWCP_REG_WRITE(base, ENCODER_CONFIG, 0x7);
}


int ubwcp_hw_flush(void __iomem *base)
{
	u32 flush_complete = 0;
	u32 count = 20;

	UBWCP_REG_WRITE(base, FLUSH_CONTROL, 0x3);
	do {
		flush_complete = UBWCP_REG_READ(base, FLUSH_STATUS) & 0x1;
		if (flush_complete) {
			UBWCP_REG_WRITE(base, FLUSH_CONTROL, 0x0);
			return 0;
		}
		udelay(100);
	} while (count--);

	ERR("~~~~~ FLUSH FAILED ~~~~~");
	return -1;
}
EXPORT_SYMBOL(ubwcp_hw_flush);


void ubwcp_hw_power_vote_status(void __iomem *pwr_ctrl, u8 *vote, u8 *status)
{
	u32 reg;

	reg = UBWCP_REG_READ(pwr_ctrl, 0);
	*vote =   (reg & BIT(0)) >> 0;
	*status = (reg & BIT(31)) >> 31;
}

void ubwcp_hw_one_time_init(void __iomem *base)
{
	u32 reg;

	/* Spare reg config: set bit-9: SCC & bit-1: padding */
	reg = UBWCP_REG_READ(base, SPARE);
	reg |= BIT(9) | BIT(1);
	UBWCP_REG_WRITE(base, SPARE, reg);

	/* Configure SID */
	reg = UBWCP_REG_READ(base, QNS4_PARAMS);
	reg &= ~(0x3F);
	reg |= 0x1;      /* desc buffer */
	reg |= (0 << 3); /* pixel data  */
	UBWCP_REG_WRITE(base, QNS4_PARAMS, reg);

	ubwcp_hw_decoder_config(base);
	ubwcp_hw_encoder_config(base);
	ubwcp_hw_macro_tile_config(base);
}
EXPORT_SYMBOL(ubwcp_hw_one_time_init);

void ubwcp_hw_trace_set(bool value)
{
	ubwcp_hw_trace_en = value;
}
EXPORT_SYMBOL(ubwcp_hw_trace_set);

void ubwcp_hw_trace_get(bool *value)
{
	*value = ubwcp_hw_trace_en;
}
EXPORT_SYMBOL(ubwcp_hw_trace_get);
