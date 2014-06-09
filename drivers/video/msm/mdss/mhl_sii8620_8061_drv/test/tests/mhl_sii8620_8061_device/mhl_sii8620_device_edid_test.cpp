/*
 * Copyright (C) 2013 Sony Mobile Communications AB.
 * Copyright (C) 2011 Silicon Image Inc.
 *
 * Author: [Hirokuni Kawasaki <hirokuni.kawasaki@sonymobile.com>]
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

extern "C" {

#include "mhl_sii8620_8061_device.h"
#include "si_8620_regs.h"
#include "MockRegAccess.h"
#include "mhl_common.h"
#include "string.h"
#include "mhl_lib_edid.h"
#include "edid_data.h"
#include <linux/errno.h>
typedef enum {
	block_0 = 0,
	block_1 = 128
} EDID_FIFO_BLOCK_OFFSET;

#define MHL_DEVICE_TEST_ALL


extern void mhl_sii8620_device_edid_reset_ddc_fifo(void);
extern int mhl_sii8620_device_set_edid_block_size(int size);
extern int mhl_sii8620_device_get_edid_fifo_next_block(uint8_t *edid_buf);
extern int read_edid_data_from_edid_fifo(uint8_t *edid_buf, EDID_FIFO_BLOCK_OFFSET block_offset);
extern void set_current_edid_request_block(int block_num);
extern uint8_t *get_stored_edid_block(void);
extern int set_upstream_edid(uint8_t *edid_buf, uint16_t length);
extern void filter_remove_vic16_1080p60fps(uint8_t *ext_edid);
extern uint8_t get_data_block_tag_code(uint8_t data_block_collection);
extern void remove_head_from_data(uint8_t *data, int data_length);
extern void set_data_block_length(uint8_t *data_block, uint8_t length);
extern uint8_t calculate_generic_checksum(uint8_t *info_frame_data, uint8_t checksum, uint8_t length);

extern void edid_filter_prune_ext_blk(uint8_t *edid);

extern CBUS_MODE_TYPE cbus_mode;

extern int mhl_lib_edid_remove_vic(uint8_t *svd, uint8_t svd_size, uint8_t *filter, uint8_t filter_size);
extern int mhl_lib_edid_pull_up_and_padding(uint8_t *head, int data_size, int pull_up_size);
extern int mhl_lib_edid_remove_hdmi_vic_and_3d(uint8_t *vsd, uint8_t vsd_size);
extern const uint8_t *hdmi_edid_find_block(const uint8_t *in_buf, u32 start_offset, u8 type, u8 *len);
extern void edid_filter_prune_block_0(uint8_t *blk0);
extern bool mhl_dev_edid_is_fifo_hw_good_cond(uint8_t *edid_buf);

extern bool isMhlSuptFilter;
extern int get_converted_hdmi_vic(uint8_t vic);
extern int mhl_lib_edid_add_hdmi_vic(uint8_t *vsd, uint8_t vsd_size, int hdmi_vic_flag);
extern bool is_supp_hdmi_vic(uint8_t hdmi_vic, uint8_t *supp_vic, uint8_t supp_vic_size);


extern uint8_t mhl_3_support_vic_array[SUPPORT_MHL_3_VIDEO_NUM];
extern const struct mhl_video_timing_info mhl_3_support_video[SUPPORT_MHL_3_VIDEO_NUM];

extern void mhl_device_edid_add_mhlsink_sprt_hev(const uint8_t *hev, int length);
int squash_data_with_prune_0(uint8_t *data, int length);
extern bool mhl_lib_is_ieee_reg_id(const uint8_t* ext_edid);
}

#define VAL_INTR9_EDID_ERROR_MASK	VAL_PAGE_2_INTR9_MASK_INTR9_MASK6_ENABLE
#define VAL_INTR9_EDID_DONE_MASK	VAL_PAGE_2_INTR9_MASK_INTR9_MASK5_ENABLE

#define BIT_GPIO_0_DISABLED	0x01
#define BIT_GPIO_0_HIGH 	0x02
#define BIT_GPIO_1_DISABLED	0x04
#define BIT_GPIO_1_HIGH		0x08
#define BITS_GPIO_01_HPD_HIGH					(BIT_GPIO_0_HIGH | BIT_GPIO_1_HIGH)
#define BITS_HPD_CTRL_PUSH_PULL_HIGH			(BITS_GPIO_01_HPD_HIGH | 0x30 )


#define BIT_INTR9_EDID_ERROR					BIT_PAGE_2_INTR9_INTR9_STAT6
#define BIT_INTR9_EDID_DONE						BIT_PAGE_2_INTR9_INTR9_STAT5
#define BIT_INTR9_DEVCAP_DONE					BIT_PAGE_2_INTR9_INTR9_STAT4
#define BIT_INTR9_EDID_ERROR_MASK				BIT_PAGE_2_INTR9_MASK_INTR9_MASK6
#define BIT_INTR9_EDID_DONE_MASK				BIT_PAGE_2_INTR9_MASK_INTR9_MASK5
#define BIT_INTR9_DEVCAP_DONE_MASK				BIT_PAGE_2_INTR9_MASK_INTR9_MASK4


#define PRINT_DATA(_data, _count) \
	{ \
		int i; \
		pr_info("----- \n"); \
		for(i = 0; i<_count; i++){ \
			pr_info("0x%02x ",_data[i]); \
			if ((i+1) %16 == 0) \
				pr_info("\n"); \
		} \
		pr_info("----- \n"); \
	}



/*dummy edid*/
#define EDID_BLOCK_DUMMY_0 	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x4D, 0xD9, 0x01, 0xF2, 0x01, 0x01, 0x01, 0x01,\
	0x01, 0x14, 0x01, 0x03, 0x80, 0xA0, 0x5A, 0x78, 0x0A, 0x0D, 0xC9, 0xA0, 0x57, 0x47, 0x98, 0x27,\
	0x12, 0x48, 0x4C, 0x21, 0x08, 0x00, 0x81, 0x80, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,\
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3A, 0x80, 0x18, 0x71, 0x38, 0x2D, 0x40, 0x58, 0x2C,\
	0x45, 0x00, 0x40, 0x84, 0x63, 0x00, 0x00, 0x1E, 0x01, 0x1D, 0x00, 0x72, 0x51, 0xD0, 0x1E, 0x20,\
	0x6E, 0x28, 0x55, 0x00, 0x40, 0x84, 0x63, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0xFD, 0x00, 0x3A,\
	0x3E, 0x0F, 0x46, 0x0F, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFC,\
	0x00, 0x53, 0x4F, 0x4E, 0x59, 0x20, 0x54, 0x56, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0xAF\

#define EDID_BLOCK_DUMMY_1 	0x02, 0x03, 0x28, 0xF0, 0x49, 0x10, 0x04, 0x05, 0x03, 0x02, 0x07, 0x06, 0x20, 0x01, 0x29, 0x09,\
	0x07, 0x07, 0x15, 0x07, 0x50, 0x35, 0x05, 0x50, 0x83, 0x01, 0x00, 0x00, 0x68, 0x03, 0x0C, 0x00,\
	0x20, 0x00, 0xB8, 0x2D, 0x0F, 0xE2, 0x00, 0x7B, 0x01, 0x1D, 0x80, 0x18, 0x71, 0x1C, 0x16, 0x20,\
	0x58, 0x2C, 0x25, 0x00, 0x40, 0x84, 0x63, 0x00, 0x00, 0x9E, 0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0,\
	0x2D, 0x10, 0x10, 0x3E, 0x96, 0x00, 0x40, 0x84, 0x63, 0x00, 0x00, 0x18, 0x8C, 0x0A, 0xD0, 0x8A,\
	0x20, 0xE0, 0x2D, 0x10, 0x10, 0x3E, 0x96, 0x00, 0xB0, 0x84, 0x43, 0x00, 0x00, 0x18, 0x8C, 0x0A,\
	0xA0, 0x14, 0x51, 0xF0, 0x16, 0x00, 0x26, 0x7C, 0x43, 0x00, 0xB0, 0x84, 0x43, 0x00, 0x00, 0x98,\
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40\

#define EDID_BLOCK_DUMMY_2_ILLEGAL_DATA 	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,\
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,\
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,\
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,\
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,\
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,\
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,\
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF\

#define EDID_BLOCK_DUMMY_3_ILLEGAL_DATA	0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, \
	0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, \
	0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, \
	0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, \
	0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, \
	0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, \
	0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, \
	0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA \

#define EDID_BLOCK_DUMMY_4_ILLEGAL_DATA 	0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, \
	0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, \
	0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, \
	0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, \
	0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, \
	0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, \
	0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, \
	0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB \


static uint8_t dummy_edid_block_0_data [EDID_BLOCK_SIZE] = { 
	EDID_BLOCK_DUMMY_0
};

static uint8_t dummy_edid_block_1_data [EDID_BLOCK_SIZE] = { 
	EDID_BLOCK_DUMMY_1
};


static uint8_t dummy_edid_2_blocks_data [EDID_BLOCK_SIZE*4] = {
	EDID_BLOCK_DUMMY_0, 
	EDID_BLOCK_DUMMY_1
};

static uint8_t dummy_edid_3_blocks_data [EDID_BLOCK_SIZE*4] = {
	EDID_BLOCK_DUMMY_0, 
	EDID_BLOCK_DUMMY_1,
	EDID_BLOCK_DUMMY_2_ILLEGAL_DATA
};

static uint8_t dummy_edid_5_blocks_data [EDID_BLOCK_SIZE*5] = {
	EDID_BLOCK_DUMMY_0, 
	EDID_BLOCK_DUMMY_1,
	EDID_BLOCK_DUMMY_2_ILLEGAL_DATA,
	EDID_BLOCK_DUMMY_3_ILLEGAL_DATA,
	EDID_BLOCK_DUMMY_4_ILLEGAL_DATA
};

static uint8_t dummy_edid_block_2_data [EDID_BLOCK_SIZE] = { 
	EDID_BLOCK_DUMMY_2_ILLEGAL_DATA
};

static uint8_t dummy_edid_block_3_data [EDID_BLOCK_SIZE] = { 
	EDID_BLOCK_DUMMY_3_ILLEGAL_DATA
};

static uint8_t dummy_edid_block_4_data [EDID_BLOCK_SIZE] = { 
	EDID_BLOCK_DUMMY_4_ILLEGAL_DATA
};



TEST_GROUP(MhlTxEDID) {
	void setup() {
		mock().ignoreOtherCalls();
		mock("IO").strictOrder();
		MOCK_mhl_pf_i2c_print(false);
		mhl_sii8620_device_edid_init();
		total_edid_block = 0;
		//mhl_device_initialize(NULL);
	}
	void teardown() {
		//mock("IO").checkExpectations();
		mock().clear();
		mhl_sii8620_device_edid_release();
	}

#define BITS_GPIO_01_HPD_HIGH	(BIT_GPIO_0_HIGH | BIT_GPIO_1_HIGH)
#define BITS_GPIO_01_HPD_LOW	0
#define BITS_HPD_CTRL_PUSH_PULL_HIGH  (BITS_GPIO_01_HPD_HIGH | 0x30 )
#define BITS_HPD_CTRL_PUSH_PULL_LOW  (BITS_GPIO_01_HPD_LOW  | 0x10)
#define BIT_PAGE_2_TMDS_CSTAT_P3_DISABLE_AUTO_AVIF_CLEAR	0x04

	void setup_hpd_low(void)
	{
		mock("IO").expectOneCall("mhl_pf_modify_reg")
			.withParameter("page",(int)TX_PAGE_2)
			.withParameter("offset",(int)0xE3)
			.withParameter("mask",(int)BIT_PAGE_2_EDID_CTRL_EDID_PRIME_VALID)
			.withParameter("value",(int)VAL_PAGE_2_EDID_CTRL_EDID_PRIME_VALID_DISABLE);

		mock("IO").expectOneCall("mhl_pf_write_reg")
			.withParameter("page",(int)TX_PAGE_0)
			.withParameter("offset",(int)0x79)
			.withParameter("value",BITS_HPD_CTRL_PUSH_PULL_LOW);

		mock("IO").expectOneCall("mhl_pf_write_reg")
			.withParameter("page",(int)TX_PAGE_0)
			.withParameter("offset",(int)0x7C)
			.withParameter("value", 0x00);
	}

	void setup_hpd_high(void)
	{
		/* REG_PAGE_2_TMDS_CSTAT_P3 */
		mock("IO").expectOneCall("mhl_pf_read_reg")
			.withParameter("page",(int)TX_PAGE_2)
			.withParameter("offset",(int)0xA0)
			.andReturnValue(0);

		/* REG_PAGE_2_TMDS_CSTAT_P3 */
		mock("IO").expectOneCall("mhl_pf_write_reg")
			.withParameter("page",(int)TX_PAGE_2)
			.withParameter("offset",(int)0xA0)
			.withParameter("value", BIT_PAGE_2_TMDS_CSTAT_P3_DISABLE_AUTO_AVIF_CLEAR);

		mock("IO").expectOneCall("mhl_pf_write_reg")
			.withParameter("page",(int)TX_PAGE_0)
			.withParameter("offset",(int)0x79)
			.withParameter("value", BITS_HPD_CTRL_PUSH_PULL_HIGH);
	}

	void set_hpd_int_status(bool isHPDHigh)
	{
		int reg_hpd_active;
		if (isHPDHigh)
			reg_hpd_active = BIT_PAGE_5_CBUS_STATUS_CBUS_HPD;
		else
			reg_hpd_active = ~BIT_PAGE_5_CBUS_STATUS_CBUS_HPD;
		mock("IO").expectOneCall("mhl_pf_read_reg")
			.withParameter("page",(int)TX_PAGE_5)
			.withParameter("offset",(int)0x91)
			.andReturnValue(reg_hpd_active);
	}

	void setup_edid_reset_ddc_fifo_ddc_ack(bool isDdcNoAckTest)
	{
		int ddc_status;

		/*
		 * reset ddc fifo
		 */
		 
		if (isDdcNoAckTest) {
			ddc_status = 0xFF & BIT_PAGE_0_DDC_STATUS_DDC_NO_ACK;
		} else {
			ddc_status = BIT_PAGE_0_DDC_STATUS_DDC_I2C_IN_PROG ;
		}

		/* read REG_PAGE_0_DDC_STATUS  */
		mock("IO").expectOneCall("mhl_pf_read_reg")
			.withParameter("page",(int)TX_PAGE_0)
			.withParameter("offset",(int)0xF2)
			.andReturnValue(ddc_status);

		/* modify REG_PAGE_0_LM_DDC */
#if 0
		mock("IO").expectOneCall("mhl_pf_read_reg")
			.withParameter("page",(int)TX_PAGE_0)
			.withParameter("offset",(int)0xC7)
			.andReturnValue((int)0x00);
		mock("IO").expectOneCall("mhl_pf_write_reg")
			.withParameter("page",(int)TX_PAGE_0)
			.withParameter("offset",0xC7)
			.withParameter("value",VAL_PAGE_0_LM_DDC_SW_TPI_EN_ENABLED);
#endif
		mock("IO").expectOneCall("mhl_pf_modify_reg")
			.withParameter("page",(int)TX_PAGE_0)
			.withParameter("offset",(int)0xC7)
			.withParameter("mask",(int)BIT_PAGE_0_LM_DDC_SW_TPI_EN)
			.withParameter("value",(int)VAL_PAGE_0_LM_DDC_SW_TPI_EN_ENABLED);

		if (isDdcNoAckTest) {
			/* clean ddc ack status  */
			mock("IO").expectOneCall("mhl_pf_write_reg")
			.withParameter("page",(int)TX_PAGE_0)
			.withParameter("offset",0xF2)
			.withParameter("value",ddc_status & ~BIT_PAGE_0_DDC_STATUS_DDC_NO_ACK);
		}

		/* modify REG_PAGE_0_DDC_CMD */
#if 0
		mock("IO").expectOneCall("mhl_pf_read_reg")
			.withParameter("page",(int)TX_PAGE_0)
			.withParameter("offset",(int)0xF3)
			.andReturnValue((int)MSK_PAGE_0_DDC_CMD_DDC_CMD);

		mock("IO").expectOneCall("mhl_pf_write_reg")
			.withParameter("page",(int)TX_PAGE_0)
			.withParameter("offset",(int)0xF3)
			.withParameter("value", VAL_PAGE_0_DDC_CMD_DDC_CMD_CLEAR_FIFO);
#endif
		mock("IO").expectOneCall("mhl_pf_modify_reg")
			.withParameter("page",(int)TX_PAGE_0)
			.withParameter("offset",(int)0xF3)
			.withParameter("mask",(int)MSK_PAGE_0_DDC_CMD_DDC_CMD)
			.withParameter("value",(int)VAL_PAGE_0_DDC_CMD_DDC_CMD_CLEAR_FIFO);

		/* modify REG_PAGE_0_LM_DDC */
#if 0
		mock("IO").expectOneCall("mhl_pf_read_reg")
			.withParameter("page",(int)TX_PAGE_0)
			.withParameter("offset",(int)0xC7)
			.andReturnValue((int)BIT_PAGE_0_LM_DDC_SW_TPI_EN);
		
		mock("IO").expectOneCall("mhl_pf_write_reg")
			.withParameter("page",(int)TX_PAGE_0)
			.withParameter("offset",(int)0xC7)
			.withParameter("value", VAL_PAGE_0_LM_DDC_SW_TPI_EN_DISABLED);
#endif
		mock("IO").expectOneCall("mhl_pf_modify_reg")
			.withParameter("page",(int)TX_PAGE_0)
			.withParameter("offset",(int)0xC7)
			.withParameter("mask",(int)BIT_PAGE_0_LM_DDC_SW_TPI_EN)
			.withParameter("value",(int)VAL_PAGE_0_LM_DDC_SW_TPI_EN_DISABLED);
	}

	void setup_edid_read_request(bool isHPDHigh, bool isDdcNoAck, uint8_t edid_block_num)
	{
		int hpd_reg = ~BIT_PAGE_5_CBUS_STATUS_CBUS_HPD;

		/*hpd setup*/
		if (isHPDHigh)
			hpd_reg = BIT_PAGE_5_CBUS_STATUS_CBUS_HPD;

		mock("IO").expectOneCall("mhl_pf_read_reg")
			.withParameter("page",(int)TX_PAGE_5)
			.withParameter("offset",(int)0x91)
			.andReturnValue(hpd_reg);

		if (!isHPDHigh)
			return;

		mock("IO").expectOneCall("mhl_pf_modify_reg")
			.withParameter("page",(int)TX_PAGE_2)
			.withParameter("offset",(int)0xE3)
			.withParameter("mask",(int)BIT_PAGE_2_EDID_CTRL_DEVCAP_SEL)
			.withParameter("value",(int)VAL_PAGE_2_EDID_CTRL_DEVCAP_SELECT_DEVCAP);

		mock("IO")
			.expectOneCall("mhl_pf_write_reg")
			.withParameter("page", TX_PAGE_2)
			.withParameter("offset", 0xE3)
			.withParameter("value", VAL_PAGE_2_EDID_CTRL_EDID_PRIME_VALID_DISABLE
									| VAL_PAGE_2_EDID_CTRL_DEVCAP_SELECT_EDID
									| VAL_PAGE_2_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE
									| ((edid_block_num & 0x01) << 2)
									| VAL_PAGE_2_EDID_CTRL_EDID_MODE_EN_ENABLE);

		setup_edid_reset_ddc_fifo_ddc_ack(isDdcNoAck);

		if (edid_block_num == 0) {
			/*
			 * Setup : Enable EDID interrupt
			 */

		/* modify */
#if 0
		mock("IO").expectOneCall("mhl_pf_read_reg")
			.withParameter("page",(int)TX_PAGE_2)
			.withParameter("offset",(int)0xE1)
			.andReturnValue((int)0xFF);
		mock("IO").expectOneCall("mhl_pf_write_reg")
			.withParameter("page",(int)TX_PAGE_2)
			.withParameter("offset",0xE1)
			.withParameter("value",0xFF);
#endif
		mock("IO").expectOneCall("mhl_pf_modify_reg")
			.withParameter("page",(int)TX_PAGE_2)
			.withParameter("offset",(int)0xE1)
			.withParameter("mask",(int)BIT_INTR9_EDID_ERROR_MASK | BIT_INTR9_EDID_DONE_MASK)
			.withParameter("value",(int)VAL_INTR9_EDID_ERROR_MASK | VAL_INTR9_EDID_DONE_MASK);

/*
			mock("IO").expectOneCall("mhl_pf_write_reg")
			.withParameter("page",(int)TX_PAGE_2)
			.withParameter("offset",(int)0xE1)
			.withParameter("value", BIT_INTR9_DEVCAP_DONE_MASK
								| BIT_INTR9_EDID_DONE_MASK
								| BIT_INTR9_EDID_ERROR);
*/

			/*
			* Trigger EDID block Read for a specific block
			 */
			/* block 0. start reading edid. */
			mock("IO")
				.expectOneCall("mhl_pf_write_reg")
				.withParameter("page", TX_PAGE_2)
				.withParameter("offset", 0xE2)
				.withParameter("value", BIT_PAGE_2_TPI_CBUS_START_GET_EDID_START_0);
		} else {
			/*
			 * Trigger EDID block Read for a specific block
			 */
			int ext_edid_block = -1;
			
			switch (edid_block_num) {
				case 1:
					/* ext_edid_block = BIT_PAGE_2_EDID_START_EXT_GET_EDID_START_1; */
					ext_edid_block = 1; 
					break;
				case 2:
					/* ext_edid_block = BIT_PAGE_2_EDID_START_EXT_GET_EDID_START_2; */
					ext_edid_block = 2;
					break;
				case 3:
					/* ext_edid_block = BIT_PAGE_2_EDID_START_EXT_GET_EDID_START_3; */
					ext_edid_block = 4;
					break;
				case 4:
					/* ext_edid_block = BIT_PAGE_2_EDID_START_EXT_GET_EDID_START_4; */
					ext_edid_block = 8;
					break;
				default:
					return;
			}
			mock("IO").expectOneCall("mhl_pf_write_reg")
				.withParameter("page",(int)TX_PAGE_2)
				.withParameter("offset",(int)0xED)
				.withParameter("value",ext_edid_block);
		}
	}
	
	void setup_edid_fifo_2(uint8_t expect_read_start_byte,
						 uint8_t expect_read_length,
						 uint8_t *dummy_edid_in_fifo,
						 uint8_t fifo_offset_pointer)
	{
		/* modify. Change EDID for FIFO read*/
		mock("IO").expectOneCall("mhl_pf_modify_reg")
			.withParameter("page",(int)TX_PAGE_2)
			.withParameter("offset",(int)0xE3)
			.withParameter("mask",(int)BIT_PAGE_2_EDID_CTRL_DEVCAP_SEL)
			.withParameter("value",(int)VAL_PAGE_2_EDID_CTRL_DEVCAP_SELECT_EDID);

		/* set offset of the reading pointer in edid fifo */
		mock("IO").expectOneCall("mhl_pf_write_reg")
			.withParameter("page",(int)TX_PAGE_2)
			.withParameter("offset",(int)0xE9)
			.withParameter("value", expect_read_start_byte);

		/* Read edid data from the EDID FIFO  */
		mock("IO").expectOneCall("mhl_pf_read_reg_block")
					.withParameter("page",(int)TX_PAGE_2)
			.withParameter("offset",(int)0xEC)
			.withParameter("count",(int)expect_read_length)
						.andReturnValue((void *)(dummy_edid_in_fifo + fifo_offset_pointer));

		pr_debug("%s:fifo_offset_pointer=%d\n", __func__ ,fifo_offset_pointer);
	}

	void setup_edid_fifo_with_offset(uint8_t expect_read_start_byte, uint8_t *fake_edid_data, bool isHPDHigh, uint8_t fifo_offset_pointer)
	{

		setup_edid_fifo_2(expect_read_start_byte, EDID_BLOCK_SIZE, fake_edid_data, fifo_offset_pointer);

		/* HPD is high in the CBUS status  */
		set_hpd_int_status(isHPDHigh);
	}

	void setup_edid_fifo(uint8_t expect_read_start_byte, uint8_t *fake_edid_data, bool isHPDHigh)
	{

		setup_edid_fifo_with_offset(expect_read_start_byte, fake_edid_data, isHPDHigh, 0);
#if 0
		setup_edid_fifo_2(expect_read_start_byte, EDID_BLOCK_SIZE, fake_edid_data, 0);
		/* HPD is high in the CBUS status  */
		set_hpd_int_status(isHPDHigh);
#endif
		#if 0
		if (isHPDHigh)
			reg_hpd_active = BIT_PAGE_5_CBUS_STATUS_CBUS_HPD;
		else
			reg_hpd_active = ~BIT_PAGE_5_CBUS_STATUS_CBUS_HPD;

		mock("IO").expectOneCall("mhl_pf_read_reg")
			.withParameter("page",(int)TX_PAGE_5)
			.withParameter("offset",(int)0x91)
			.andReturnValue(reg_hpd_active);
		#endif
	}
	
	void verify_edid_partial_block(uint8_t start, uint8_t length, uint8_t *edid_buf, 
					int res, int returned_block_number, 
					uint8_t *dummy_edid_in_fifo, uint8_t compare_size)
	{
		LONGS_EQUAL(res, mhl_sii8620_device_get_edid_fifo_partial_block(start
										,length
										,edid_buf));
		LONGS_EQUAL(returned_block_number,mhl_sii8620_device_edid_get_block_number());
#if 0
		{
			int i;
			for (i = 0; i<20; i++){
				pr_debug("0x%x", edid_buf[i]);
			}
			pr_debug("\n--\n");
			for (i = 0; i<20; i++){
				pr_debug("0x%x", dummy_edid_in_fifo[start+i]);
			}
		}
#endif
		LONGS_EQUAL(0, memcmp((const void *)edid_buf, (const void *)(start + dummy_edid_in_fifo), compare_size));
	}

	void setup_read_SetHpd_status(bool isSetHPD)
	{
		int reg_hpd_active;
		
		if (isSetHPD) {
			reg_hpd_active = BIT_PAGE_5_CBUS_STATUS_CBUS_HPD;
		} else {
			reg_hpd_active = ~BIT_PAGE_5_CBUS_STATUS_CBUS_HPD;
		}

		mock("IO").expectOneCall("mhl_pf_read_reg")
				.withParameter("page",(int)TX_PAGE_5)
				.withParameter("offset",(int)0x91)
				.andReturnValue((uint8_t)reg_hpd_active);
	}

	int total_edid_block;
	/* "dummy_edid_5_blocks_data" is used for dummy edid data*/
	void setup_int_edid_isr_edid_block_arrived(uint8_t *edid ,uint8_t arrived_block_num, bool is_next_block_requested) {


		if (arrived_block_num > 5) {
			pr_warn("block num > 5");
			return;
		}

		/* expected */
		/* clear BIT_INTR9_EDID_DONE of EDID FIFO Interrupt (REG_EDID_FIFO_INT) */
/*
		mock("IO").expectOneCall("mhl_pf_write_reg")
			.withParameter("page",(int)TX_PAGE_2)
			.withParameter("offset",(int)0xE0)
			.withParameter("value", BIT_INTR9_EDID_DONE);
*/
		/* set hpd check  */
		setup_read_SetHpd_status(true);

		/* ddc status check. ddc is not no ack. */
		mock("IO").expectOneCall("mhl_pf_read_reg")
			.withParameter("page",(int)TX_PAGE_0)
			.withParameter("offset",(int)0xF2)
			.andReturnValue(~BIT_PAGE_0_DDC_STATUS_DDC_NO_ACK);
		{
			bool isDdcNoAck = false;
			setup_edid_reset_ddc_fifo_ddc_ack(isDdcNoAck);
			/*9 call */
		}

		/* set up for fifo access */
		{
			uint8_t offset = 0;
			uint8_t *edid_buf;
			bool HPD_is_high = true;
			uint8_t fifo_offset_pointer = 0;
			if((arrived_block_num & 0x01) == 0) {
				offset = 0;
			} else {
				offset = 128;
			}

			pr_debug("%s:test offset=%d, arrived_block_num=%d \n" ,__func__ ,offset ,arrived_block_num);
			setup_edid_fifo_with_offset(offset, &edid[arrived_block_num*EDID_BLOCK_SIZE],
										HPD_is_high, fifo_offset_pointer);
			total_edid_block += EDID_BLOCK_SIZE;
#if 0
			setup_edid_fifo(offset,
							 &dummy_edid_5_blocks_data[arrived_block_num],
							  HPD_is_high);
#endif
			/*11 call*/
		}
		/* read request is executed */
		{
			bool isHPDHigh = true;
			bool isDdcNoAck = false;
			if(is_next_block_requested)
				setup_edid_read_request(isHPDHigh, isDdcNoAck, arrived_block_num + 1 );
		}
	}

#define BITS_GPIO_01_HPD_HIGH	(BIT_GPIO_0_HIGH | BIT_GPIO_1_HIGH)
#define BITS_GPIO_01_HPD_LOW	0
#define BITS_HPD_CTRL_PUSH_PULL_HIGH  (BITS_GPIO_01_HPD_HIGH | 0x30 )
#define BITS_HPD_CTRL_PUSH_PULL_LOW  (BITS_GPIO_01_HPD_LOW  | 0x10)

#define RX_DPD_BITS (BIT_PAGE_0_DPD_PDNRX12 \
			  | BIT_PAGE_0_DPD_PDIDCK_N \
			  | BIT_PAGE_0_DPD_PD_MHL_CLK_N)

	void setup_set_upstream_edid (uint8_t *edid, uint16_t length)
	{
		/* hpd low */
		setup_hpd_low();

		/* REG_PAGE_0_DPD */
		mock("IO").expectOneCall("mhl_pf_modify_reg")
			.withParameter("page",(int)TX_PAGE_0)
			.withParameter("offset", 0x0B)
			.withParameter("mask", RX_DPD_BITS)
			.withParameter("value", RX_DPD_BITS);

		/* REG_PAGE_2_RX_HDMI_CTRL3 */
		mock("IO").expectOneCall("mhl_pf_write_reg")
			.withParameter("page",(int)TX_PAGE_2)
			.withParameter("offset",0xA4)
			.withParameter("value", 0x00);

		/*Packet Filter0 Register*/
		mock("IO").expectOneCall("mhl_pf_write_reg")
			.withParameter("page",(int)TX_PAGE_2)
			.withParameter("offset",0x90)
			.withParameter("value",0xFF);

		/*Packet Filter1 Register*/
		mock("IO").expectOneCall("mhl_pf_write_reg")
			.withParameter("page",(int)TX_PAGE_2)
			.withParameter("offset",0x91)
			.withParameter("value",0xFF);


		mock("IO").expectOneCall("mhl_pf_modify_reg")
			.withParameter("page",(int)TX_PAGE_2)
			.withParameter("offset",(int)0xAC)
			.withParameter("mask",(int)BIT_PAGE_2_RX_HDMI_CLR_BUFFER_VSI_CLR_EN)
			.withParameter("value",(int)VAL_PAGE_2_RX_HDMI_CLR_BUFFER_VSI_CLR_EN_CLEAR);


		/* choose EDID instead of devcap to appear at the FIFO */
		/* REG_PAGE_2_EDID_CTR */
		mock("IO").expectOneCall("mhl_pf_write_reg")
			.withParameter("page",(int)TX_PAGE_2)
			.withParameter("offset",0xE3)
			.withParameter("value",
							VAL_PAGE_2_EDID_CTRL_EDID_PRIME_VALID_DISABLE
							| VAL_PAGE_2_EDID_CTRL_DEVCAP_SELECT_EDID
							| VAL_PAGE_2_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE
							| VAL_PAGE_2_EDID_CTRL_EDID_MODE_EN_ENABLE);
	
		/* clear address with 0 */
		mock("IO").expectOneCall("mhl_pf_write_reg")
			.withParameter("page",(int)TX_PAGE_2)
			.withParameter("offset",0xE9)
			.withParameter("value",0x00);
		/* write edid data into the FIFO */
		mock("IO").expectOneCall("mhl_pf_write_reg_block")
			.withParameter("page",(int)TX_PAGE_2)
			.withParameter("offset",0xEA)
			.withParameter("count",length)
			.withParameter("values",edid);
	
		mock("IO").expectOneCall("mhl_pf_write_reg")
			.withParameter("page",(int)TX_PAGE_2)
			.withParameter("offset",0xE3)
			.withParameter("value"
						, VAL_PAGE_2_EDID_CTRL_EDID_PRIME_VALID_ENABLE
						| VAL_PAGE_2_EDID_CTRL_DEVCAP_SELECT_EDID
						| VAL_PAGE_2_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE
						| VAL_PAGE_2_EDID_CTRL_EDID_MODE_EN_ENABLE);


		/* HB */
		mock("IO").expectOneCall("mhl_pf_modify_reg")
			.withParameter("page",(int)TX_PAGE_5)
			.withParameter("offset",(int)0xE0)
			.withParameter("mask",(int)BIT_PAGE_5_DISC_CTRL1_HB_EN)
			.withParameter("value",(int)BIT_PAGE_5_DISC_CTRL1_HB_EN);

		mock("IO").expectOneCall("mhl_pf_write_reg")
			.withParameter("page",(int)TX_PAGE_5)
			.withParameter("offset", 0xC4)
			.withParameter("value", 0xA7);

		/* Enable SCDT interrupt to detect stable incoming clock 
		 * #define BIT_INTR_SCDT_CHANGE : BIT_PAGE_0_INTR5_INTR5_STAT0
		 */
		/* Enable SCDT interrupt */
		mock("IO").expectOneCall("mhl_pf_write_reg")
			.withParameter("page",(int)TX_PAGE_0)
			.withParameter("offset", 0x78)
			.withParameter("value", BIT_PAGE_0_INTR5_MASK_INTR5_MASK0);
		 
		/* Disable EDID interrupt */ 
		mock("IO").expectOneCall("mhl_pf_modify_reg")
			.withParameter("page",(int)TX_PAGE_2)
			.withParameter("offset", 0xE1)
			.withParameter("mask",BIT_INTR9_EDID_ERROR_MASK | BIT_INTR9_EDID_DONE_MASK)
			.withParameter("value", 0x00);
	
		/*Packet Filter0 Register*/
		mock("IO").expectOneCall("mhl_pf_write_reg")
			.withParameter("page",(int)TX_PAGE_2)
			.withParameter("offset",0x90)
			.withParameter("value",0xFF);
	
		/*Packet Filter1 Register*/
		mock("IO").expectOneCall("mhl_pf_write_reg")
			.withParameter("page",(int)TX_PAGE_2)
			.withParameter("offset",0x91)
			.withParameter("value",0xFF);

		setup_hpd_high();

	}
};

/* 
 * TEST API NAME : number + category + name
 *  (e.g. 01_nc_edid_read_request)
 *
 * category
 *   nc  : normal case
 *   snc : subnormality case
 *   ec  : error case
 */


/* Actual data of reset_ddc_fifo
---*---*---*---*---*---*[  324.858473] mhl sii-8620: si_mhl_tx_drv_reset_ddc_fifo:430 IN
---*---*---*---*---*---*---*[  324.858635] mhl sii-8620:    I2C 76.F2 R 04
---*---*---*---*---*---*---*[  324.858796] mhl sii-8620:    I2C 76.C7 R 00
---*---*---*---*---*---*---*[  324.858806] mhl sii-8620:    I2C 76.C7 W 80
---*---*---*---*---*---*---*[  324.859083] mhl sii-8620:    I2C 76.F3 R 30
---*---*---*---*---*---*---*[  324.859094] mhl sii-8620:    I2C 76.F3 W 39
---*---*---*---*---*---*---*[  324.859370] mhl sii-8620:    I2C 76.C7 R 80
---*---*---*---*---*---*---*[  324.859380] mhl sii-8620:    I2C 76.C7 W 00
---*---*---*---*---*---*[  324.859512] mhl sii-8620: si_mhl_tx_drv_reset_ddc_fifo:450 OUT
*/
#ifdef MHL_DEVICE_TEST_ALL
TEST(MhlTxEDID, 01_snc_ddc_fifo_reset__ddc_no_ack) {
	bool isDdcNoAckTest_True = true;

	setup_edid_reset_ddc_fifo_ddc_ack(isDdcNoAckTest_True);
	mhl_sii8620_device_edid_reset_ddc_fifo();
}

TEST(MhlTxEDID, 02_nc_ddc_fifo_reset) {
	bool isDdcNoAckTest_False = false;

	setup_edid_reset_ddc_fifo_ddc_ack(isDdcNoAckTest_False);
	mhl_sii8620_device_edid_reset_ddc_fifo();
}

/*
TEST(MhlTxEDID, 03_nc_edid_read_request) {
	bool isHPDHigh = true;
	bool isDdcNoAck = false;
	uint8_t  edid_block_num = 0;

	setup_edid_read_request(isHPDHigh, isDdcNoAck, edid_block_num);

	LONGS_EQUAL(0,mhl_sii8620_device_edid_read_request(0));
}
*/

/*
TEST(MhlTxEDID, 04_nc_edid_read_request_block_1) {
	bool isHPDHigh = true;
	bool isDdcNoAck = false;
	uint8_t  edid_block_num = 1;

	setup_edid_read_request(isHPDHigh, isDdcNoAck, edid_block_num);
	
	MOCK_mhl_pf_i2c_print(false);

	LONGS_EQUAL(0,mhl_sii8620_device_edid_read_request(edid_block_num));
}
*/
/*
TEST(MhlTxEDID, 05_nc_edid_read_request_block_2) {
	bool isHPDHigh = true;
	bool isDdcNoAck = false;
	uint8_t  edid_block_num = 2;

	setup_edid_read_request(isHPDHigh, isDdcNoAck, edid_block_num);
	
	MOCK_mhl_pf_i2c_print(false);

	LONGS_EQUAL(0,mhl_sii8620_device_edid_read_request(edid_block_num));
}
*/

/*
TEST(MhlTxEDID, 06_nc_edid_read_request_block_1_without_HPD) {
	bool isHPDHigh = false;
	bool isDdcNoAck = false;
	uint8_t  edid_block_num = 1;

	MOCK_mhl_pf_i2c_print(false);

	setup_edid_read_request(isHPDHigh, isDdcNoAck, edid_block_num);

	LONGS_EQUAL(MHL_FAIL,mhl_sii8620_device_edid_read_request(edid_block_num));
}
*/
/*
TEST(MhlTxEDID, 07_nc_edid_read_request_block_3) {
	bool isHPDHigh = true;
	bool isDdcNoAck = false;
	uint8_t  edid_block_num = 3;

	setup_edid_read_request(isHPDHigh, isDdcNoAck, edid_block_num);

	MOCK_mhl_pf_i2c_print(false);

	LONGS_EQUAL(MHL_SUCCESS,mhl_sii8620_device_edid_read_request(edid_block_num));
}

*/
TEST(MhlTxEDID, 08_nc_edid_read_request_block_4) {
	uint8_t  edid_block_num = EDID_BLOCK_MAX_NUM;
	LONGS_EQUAL(MHL_FAIL,mhl_sii8620_device_edid_read_request(edid_block_num));
}


TEST(MhlTxEDID, 09_nc_read_edid_block_0_data_from_EDID_FIFO) {
	uint8_t offset = 0;
	uint8_t edid_buf[EDID_BLOCK_SIZE];
	bool HPD_is_high = true;

	MOCK_mhl_pf_i2c_print(false);

	setup_edid_fifo(offset, dummy_edid_block_0_data, HPD_is_high);

	LONGS_EQUAL(MHL_SUCCESS,mhl_sii8620_device_get_edid_fifo_next_block(edid_buf));
	/*both data must be same*/
	LONGS_EQUAL(0, memcmp((const void *)dummy_edid_block_0_data, (const void *)edid_buf, EDID_BLOCK_SIZE));
}


TEST(MhlTxEDID, 10_nc_read_edid_block_1_data_from_EDID_FIFO) {
	bool HPD_is_high = true;
	/*Setup data for the 1st call of "mhl_sii8620_device_get_edid_fifo_next_block" */
	uint8_t expect_offset_for_1st = 0;
	uint8_t edid_buf_for_1st[EDID_BLOCK_SIZE];
	/*Setup data for the 2nd call of "mhl_sii8620_device_get_edid_fifo_next_block" */
	uint8_t expect_offset_for_2nd = 128;
	uint8_t edid_buf_for_2nd[EDID_BLOCK_SIZE];

	MOCK_mhl_pf_i2c_print(false);
	setup_edid_fifo(expect_offset_for_1st, dummy_edid_block_0_data, HPD_is_high);
	setup_edid_fifo(expect_offset_for_2nd, dummy_edid_block_1_data, HPD_is_high);

	/* 1st call */
	LONGS_EQUAL(MHL_SUCCESS, mhl_sii8620_device_get_edid_fifo_next_block(edid_buf_for_1st));
	LONGS_EQUAL(0, memcmp((const void *)dummy_edid_block_0_data, (const void *)edid_buf_for_1st, EDID_BLOCK_SIZE));

	/* 2nd call  */
	LONGS_EQUAL(MHL_SUCCESS, mhl_sii8620_device_get_edid_fifo_next_block(edid_buf_for_2nd));
	LONGS_EQUAL(0, memcmp((const void *)dummy_edid_block_1_data, (const void *)edid_buf_for_2nd, EDID_BLOCK_SIZE));

}
#if 1
TEST(MhlTxEDID, 11_nc_read_edid_block_2_data_from_EDID_FIFO) {
	bool HPD_is_high = true;
	/*Setup data for the 1st call of "mhl_sii8620_device_get_edid_fifo_next_block" */
	uint8_t expect_offset_for_1st = 0;
	uint8_t edid_buf_for_1st[EDID_BLOCK_SIZE];
	/*Setup data for the 2nd call of "mhl_sii8620_device_get_edid_fifo_next_block" */
	uint8_t expect_offset_for_2nd = 128;
	uint8_t edid_buf_for_2nd[EDID_BLOCK_SIZE];
	/*Setup data for the 3rd call of "mhl_sii8620_device_get_edid_fifo_next_block" */
	uint8_t expect_offset_for_3rd = 0;
	uint8_t edid_buf_for_3rd[EDID_BLOCK_SIZE];

	MOCK_mhl_pf_i2c_print(false);
	setup_edid_fifo(expect_offset_for_1st, dummy_edid_block_0_data, HPD_is_high);
	setup_edid_fifo(expect_offset_for_2nd, dummy_edid_block_1_data, HPD_is_high);
	setup_edid_fifo(expect_offset_for_3rd, dummy_edid_block_2_data, HPD_is_high);

	/* 1st call */
	LONGS_EQUAL(MHL_SUCCESS, mhl_sii8620_device_get_edid_fifo_next_block(edid_buf_for_1st));
	LONGS_EQUAL(0, memcmp((const void *)dummy_edid_block_0_data, (const void *)edid_buf_for_1st, EDID_BLOCK_SIZE));

	/* 2nd call  */
	LONGS_EQUAL(MHL_SUCCESS, mhl_sii8620_device_get_edid_fifo_next_block(edid_buf_for_2nd));
	LONGS_EQUAL(0, memcmp((const void *)dummy_edid_block_1_data, (const void *)edid_buf_for_2nd, EDID_BLOCK_SIZE));

	/* 3rd call  */
	LONGS_EQUAL(MHL_SUCCESS, mhl_sii8620_device_get_edid_fifo_next_block(edid_buf_for_3rd));
	LONGS_EQUAL(0, memcmp((const void *)dummy_edid_block_2_data, (const void *)edid_buf_for_3rd, EDID_BLOCK_SIZE));
}
#endif

#if 1
TEST(MhlTxEDID, 12_nc_read_edid_block_3_data_from_EDID_FIFO) {
	bool HPD_is_high = true;
	/*Setup data for the 1st call of "mhl_sii8620_device_get_edid_fifo_next_block" */
	uint8_t expect_offset_for_1st = 0;
	uint8_t edid_buf_for_1st[EDID_BLOCK_SIZE];
	/*Setup data for the 2nd call of "mhl_sii8620_device_get_edid_fifo_next_block" */
	uint8_t expect_offset_for_2nd = 128;
	uint8_t edid_buf_for_2nd[EDID_BLOCK_SIZE];
	/*Setup data for the 3rd call of "mhl_sii8620_device_get_edid_fifo_next_block" */
	uint8_t expect_offset_for_3rd = 0;
	uint8_t edid_buf_for_3rd[EDID_BLOCK_SIZE];
	/*Setup data for the 3rd call of "mhl_sii8620_device_get_edid_fifo_next_block" */
	uint8_t expect_offset_for_4th = 128;
	uint8_t edid_buf_for_4th[EDID_BLOCK_SIZE];

	MOCK_mhl_pf_i2c_print(false);
	setup_edid_fifo(expect_offset_for_1st, dummy_edid_block_0_data, HPD_is_high);
	setup_edid_fifo(expect_offset_for_2nd, dummy_edid_block_1_data, HPD_is_high);
	setup_edid_fifo(expect_offset_for_3rd, dummy_edid_block_2_data, HPD_is_high);
	setup_edid_fifo(expect_offset_for_4th, dummy_edid_block_3_data, HPD_is_high);

	/* 1st call */
	LONGS_EQUAL(MHL_SUCCESS, mhl_sii8620_device_get_edid_fifo_next_block(edid_buf_for_1st));
	LONGS_EQUAL(0, memcmp((const void *)dummy_edid_block_0_data, (const void *)edid_buf_for_1st, EDID_BLOCK_SIZE));

	/* 2nd call  */
	LONGS_EQUAL(MHL_SUCCESS, mhl_sii8620_device_get_edid_fifo_next_block(edid_buf_for_2nd));
	LONGS_EQUAL(0, memcmp((const void *)dummy_edid_block_1_data, (const void *)edid_buf_for_2nd, EDID_BLOCK_SIZE));

	/* 3rd call  */
	LONGS_EQUAL(MHL_SUCCESS, mhl_sii8620_device_get_edid_fifo_next_block(edid_buf_for_3rd));
	LONGS_EQUAL(0, memcmp((const void *)dummy_edid_block_2_data, (const void *)edid_buf_for_3rd, EDID_BLOCK_SIZE));

	/* 4th call  */
	LONGS_EQUAL(MHL_SUCCESS, mhl_sii8620_device_get_edid_fifo_next_block(edid_buf_for_4th));
	LONGS_EQUAL(0, memcmp((const void *)dummy_edid_block_3_data, (const void *)edid_buf_for_4th, EDID_BLOCK_SIZE));
}
#endif
/*  mhl_sii8620_device_get_edid_fifo_next_block is not used in code. So this test is obsoleted.
TEST(MhlTxEDID, 14_nc_read_edid_block_fails_with_hpd_low) {
	uint8_t offset = 0;
	uint8_t edid_buf[EDID_BLOCK_SIZE];
	bool HPD_is_high = false;

	MOCK_mhl_pf_i2c_print(false);

	setup_edid_fifo(offset, dummy_edid_block_0_data, HPD_is_high);

	LONGS_EQUAL(MHL_FAIL,mhl_sii8620_device_get_edid_fifo_next_block(edid_buf));
}
*/
#if 1
TEST(MhlTxEDID, 14_nc_read_edid_block_0) {
	uint8_t offset = 0;
	uint8_t edid_buf[EDID_BLOCK_SIZE];
	bool HPD_is_high = true;

	setup_edid_fifo(offset, dummy_edid_block_0_data, HPD_is_high);

	LONGS_EQUAL(MHL_SUCCESS, read_edid_data_from_edid_fifo(edid_buf, block_0));
}
#endif
#if 1
TEST(MhlTxEDID, 14_nc_read_edid_block_1) {
	uint8_t offset = 0;
	uint8_t edid_buf[EDID_BLOCK_SIZE];
	bool isHPDHigh = true;

	setup_edid_fifo_with_offset(block_1, dummy_edid_2_blocks_data, isHPDHigh, 128);
	LONGS_EQUAL(MHL_SUCCESS, read_edid_data_from_edid_fifo(edid_buf, block_1));
#if 0
		{
			int i;
			for (i = 0; i<20; i++){
				pr_debug("0x%x", edid_buf[i]);
			}
			pr_debug("\n--\n");
			for (i = 0; i<20; i++){
				pr_debug("0x%x", (dummy_edid_2_blocks_data)[block_1 + i]);
			}
		}
#endif
	LONGS_EQUAL(0, memcmp((const void *)edid_buf, (const void *)(dummy_edid_2_blocks_data + block_1), EDID_BLOCK_SIZE));
}
#endif
#if 0
TEST(MhlTxEDID, 15_nc_get_extention_block_number) {

	uint8_t block_number = 0;

	LONGS_EQUAL(1, mhl_edid_parser_get_num_cea_861_extensions(dummy_edid_block_0_data, block_number));
}
#endif



/*
 * mhl_sii8620_device_get_edid_fifo_partial_block(start, length, edid_buf)
 * 
 * Precondition : EDID FIFO contains 2 blocks edid data (128byte + 128byte).
 * 0 block data is read from the FIFO.
 */
#if 1
TEST(MhlTxEDID, 16_nc_get_edid_fifo_partial_block) {
	/*for setup*/
	uint8_t expect_read_length = 128;
	bool isHPDHigh = true;
	uint8_t expect_read_start_byte = 0;
	uint8_t fifo_offset_pointer = 0;
	
	/*for execution & verification*/
	uint8_t start = expect_read_start_byte;
	uint8_t length = expect_read_length;
	uint8_t edid_buf[EDID_BLOCK_SIZE];
	int returned_block_number = 1;

	setup_edid_fifo_2(expect_read_start_byte, expect_read_length, dummy_edid_2_blocks_data, fifo_offset_pointer);

	set_hpd_int_status(isHPDHigh);

	verify_edid_partial_block(start, length, edid_buf,
							 MHL_SUCCESS, returned_block_number,
							  dummy_edid_2_blocks_data, EDID_BLOCK_SIZE);

}
#endif

#define EDID_BLOCK_DUMMY_0_HEADER_SIZE 8
#if 1
TEST(MhlTxEDID, 16_nc_get_edid_header) {
	/*for setup*/
	uint8_t expect_read_length = EDID_BLOCK_DUMMY_0_HEADER_SIZE;
	bool isHPDHigh = true;
	uint8_t expect_read_start_byte = 0;
	uint8_t fifo_offset_pointer = 0;
	
	/*for execution & verification*/
	uint8_t start = expect_read_start_byte;
	uint8_t length = expect_read_length;
	uint8_t edid_buf[EDID_BLOCK_SIZE];
	int returned_block_number = 0;

	setup_edid_fifo_2(expect_read_start_byte, expect_read_length, dummy_edid_2_blocks_data, fifo_offset_pointer);

	set_hpd_int_status(isHPDHigh);

	verify_edid_partial_block(start, length, edid_buf,
							 MHL_SUCCESS, returned_block_number,
							  dummy_edid_2_blocks_data, EDID_BLOCK_DUMMY_0_HEADER_SIZE);

}
#endif
#if 1
TEST(MhlTxEDID, 16_nc_get_remaining_data_after_edid_header) {
	/*for setup*/
	uint8_t expect_read_length = EDID_BLOCK_SIZE - EDID_BLOCK_DUMMY_0_HEADER_SIZE;
	bool isHPDHigh = true;
	uint8_t expect_read_start_byte = EDID_BLOCK_DUMMY_0_HEADER_SIZE;
	uint8_t fifo_offset_pointer = 0;
	
	/*for execution & verification*/
	uint8_t start = expect_read_start_byte;
	uint8_t length = expect_read_length;
	uint8_t edid_buf[EDID_BLOCK_SIZE];
	int returned_block_number = 1;

	setup_edid_fifo_2(expect_read_start_byte,
					  expect_read_length,
					  start + dummy_edid_2_blocks_data,
					   fifo_offset_pointer);

	set_hpd_int_status(isHPDHigh);

	verify_edid_partial_block(start, length, edid_buf,
							 MHL_SUCCESS, returned_block_number,
							  dummy_edid_2_blocks_data, expect_read_length);
							  


}
#endif

/*
 * mhl_sii8620_device_get_edid_fifo_partial_block(start, length, edid_buf)
 * 
 * Precondition : EDID FIFO contains 2 blocks edid data (128byte + 128byte).
 * 0 block data is read from the FIFO.
 *
 * Fifo pointer goes forwared by 128 byte (edid block size unit) with
 * read start byte is 0, read length is the 128.
 */
#if 1
TEST(MhlTxEDID, 17_nc_get_edid_fifo_partial_block) {
	/*  for 1st setup */
	uint8_t expect_read_length = 128;
	bool isHPDHigh = true;
	uint8_t expect_read_start_byte = 0;
	uint8_t fifo_offset_pointer = 0;

	/*  for 2nd setup */
	uint8_t offset_128 = 128;
	
	uint8_t start = expect_read_start_byte;
	uint8_t length = expect_read_length;
	uint8_t edid_buf[EDID_BLOCK_SIZE];

	int ret_block_num_1 = 1;
	int ret_block_num_0 = 0;

	/* setup for 1st call */
	setup_edid_fifo_2(expect_read_start_byte, expect_read_length, dummy_edid_2_blocks_data, fifo_offset_pointer);
	set_hpd_int_status(isHPDHigh);

	/* setup for 2nd call */
	/* In the following 1st call, edid_fifo_block_number is incremented.
	 * the offset value is supposed to be 
	 * 128 = EDID_BLOCK_SIZE * (edid_fifo_block_number & 0x01)
	 */
	setup_edid_fifo_2((uint8_t)(expect_read_start_byte + offset_128),
		expect_read_length, 
		dummy_edid_2_blocks_data, 
		(uint8_t)(fifo_offset_pointer + offset_128));
	set_hpd_int_status(isHPDHigh);

	verify_edid_partial_block(start, length, edid_buf,
							MHL_SUCCESS, ret_block_num_1,
							dummy_edid_2_blocks_data, EDID_BLOCK_SIZE);

	verify_edid_partial_block(start/*0*/, length/*128*/, edid_buf,
							MHL_SUCCESS, ret_block_num_1,/*block is not incremented*/
							(dummy_edid_2_blocks_data+128),EDID_BLOCK_SIZE); /*data must be read from 128 byte*/
}
#endif

TEST(MhlTxEDID, 17_nc_get_extension_block_number) {

	LONGS_EQUAL(1,mhl_edid_parser_get_num_cea_861_extensions(dummy_edid_block_0_data));
}


#if 0 
TEST(MhlTxEDID, retry) {
	/*TODO*/
}
#endif

#if 0
TEST(MhlTxEDID, edid_int_does_nothing_without_set_hpd) {
	bool isSetHPD = false;
	uint8_t int_val = 0x10;	/* int_val can be any value in this test */


	setup_read_SetHpd_status(isSetHPD);

	/* the arg can be any value in this test*/
	mhl_8620_dev_int_edid_isr(int_val);
}
#endif
/*
TEST(MhlTxEDID, 17_nc_active_upstream_hdmi) {
	uint16_t length = 256;
	uint8_t *edid = dummy_edid_2_blocks_data;

	setup_set_upstream_edid (edid,  length);

	LONGS_EQUAL(MHL_SUCCESS, set_upstream_edid(dummy_edid_2_blocks_data, 256));

}
*/

/* 3rd edid block receive (TODO!!!)*/

/*
		mock("IO").expectOneCall("mhl_pf_read_reg")
			.withParameter("page",(int)TX_PAGE_0)
			.withParameter("offset",(int)0xC7)
			.andReturnValue((int)0x00);
	mock("IO").expectOneCall("mhl_pf_write_reg")
		.withParameter("page",(int)TX_PAGE_0)
		.withParameter("offset",0xC7)
		.withParameter("value",VAL_PAGE_0_LM_DDC_SW_TPI_EN_ENABLED);
*/



/* */



/*no request when exceeds 5 block*/

/* when more than limit block is required, limit is used as last block read. */




#if 0
TEST(MhlTxEDID, 09_nc_issue_edid_request_when_ext_blocks_remains_at_edid_int) {
	uint8_t int_9_status;

	/* setup int val */
	int_9_status = BIT_PAGE_2_INTR9_INTR9_STAT5; /*0x20*/

	/* clear EDID FIFO Interrupt */
	mock("IO").expectOneCall("mhl_pf_write_reg")
		.withParameter("page",(int)TX_PAGE_2)
		.withParameter("offset",(int)0xE0)
		.withParameter("value", BIT_PAGE_2_INTR9_INTR9_STAT5);

	/* HPD status check. (HIGH) */

	/* ddc check. read ddc status register (REG_PAGE_0_DDC_STATUS) */
	mock("IO").expectOneCall("mhl_pf_read_reg")
		.withParameter("page",(int)TX_PAGE_0)
		.withParameter("offset",(int)0xF2)
		.withParameter("value", BIT_PAGE_0_DDC_STATUS_DDC_FIFO_EMPTY);

	/* get 861 ext block number (1)*/

	/* current processing ext block number check */

	/* read request is executed */

/*	LONGS_EQUAL(MHL_SUCCESS, int_edid_isr(int_9_status));*/
}
#endif

/************  edid filter function *************/
/*
 * test category : normal case
 * description : edidi filter/video link mode/no support SUPP_PPIXEL
 */
TEST(MhlTxEDID, edid_filter_remove_1080p60fps_from_edid_data) {
	uint8_t ext_edid[EDID_BLOCK_SIZE];

	memcpy((void *)ext_edid, (const void *)edid_sharp_lc24mx1_ext_block_1, EDID_BLOCK_SIZE);


	//input edid
	mhl_edid_parser_remove_vic16_1080p60fps(ext_edid);

#if 0
	{
	int i;
	for(i =0;i<128;i++)
		pr_debug("0x%02x ",ext_edid[i]);
	}
	pr_debug("\n");
#endif

	//check the vic is removed from edid or not
	LONGS_EQUAL(0, memcmp((const void *)(edid_sharp_lc24mx1_ext_block_1_remove_vic16),
				(const void *)ext_edid, EDID_BLOCK_SIZE));
	//check the checksum re-calcuration
}


/*
 * test category : normal case
 * description : edidi filter/video link mode/no support SUPP_PPIXEL
 */
TEST(MhlTxEDID, edid_filter_remove_native_flag_1080p60fps_from_edid_data) {
	uint8_t ext_edid[EDID_BLOCK_SIZE];

	memcpy((void *)ext_edid, (const void *)edid_sharp_lc24mx1_ext_block_1_native_vic16, EDID_BLOCK_SIZE);

#if 0
PRINT_DATA(ext_edid, EDID_BLOCK_SIZE)
#endif

	//input edid
	mhl_edid_parser_remove_vic16_1080p60fps(ext_edid);


#if 0
PRINT_DATA(ext_edid, EDID_BLOCK_SIZE)
PRINT_DATA(edid_sharp_lc24mx1_ext_block_1_native_vic16, EDID_BLOCK_SIZE)
#endif


	//check the vic is removed from edid or not
	LONGS_EQUAL(0, memcmp((const void *)(edid_sharp_lc24mx1_ext_block_1_remove_vic16),
				(const void *)ext_edid, EDID_BLOCK_SIZE));
	//check the checksum re-calcuration
}



TEST(MhlTxEDID, calculated_check_sum_becomes_0_in_the_check) {
	bool res;
	uint8_t checksum = 0;
	
	edid_sharp_lc24mx1_ext_block_1_remove_vic16[127] = 0x00;
	
	checksum = calculate_generic_checksum(edid_sharp_lc24mx1_ext_block_1_remove_vic16, 0x00, 128);
	
	edid_sharp_lc24mx1_ext_block_1_remove_vic16[127] = checksum;
		
	pr_debug("checksum = 0x%02x \n" , checksum);
	
	res = mhl_lib_edid_is_valid_checksum(edid_sharp_lc24mx1_ext_block_1_remove_vic16);
	pr_debug("res = %d \n" ,res);
	LONGS_EQUAL(1,res);
}



TEST(MhlTxEDID, change_length_of_data_block_header) {
	uint8_t test_target = 0x00;
	const uint8_t input = 0x0E;

	set_data_block_length(&test_target, input);

	LONGS_EQUAL(input,test_target);
}

TEST(MhlTxEDID, max_length_test_for_length_of_data_block_header) {
	uint8_t test_target = 0x00;
	const uint8_t input = 0xFF;
	const uint8_t limitation = 0x1F;
	
	set_data_block_length(&test_target, input);

	LONGS_EQUAL(limitation,test_target);
}

TEST(MhlTxEDID, not_changed_tag_code_in_data_block_header) {
	uint8_t test_target = 0xEF;/*1110 1111*/
	const uint8_t input = 0x0A;
	const uint8_t limitation = 0x1F;
	
	set_data_block_length(&test_target, input);

	LONGS_EQUAL(0xEA,test_target); /* 1110 1010 */
}


TEST(MhlTxEDID, true_when_video_tag_code) {

	CHECK_TRUE(get_data_block_tag_code(*(edid_sharp_lc24mx1_ext_block_1+4)) == DBTC_VIDEO_DATA_BLOCK);
}

TEST(MhlTxEDID, remove_head_from_data) {
	uint8_t test_list_target[10] = {1,2,3,4,5,6,7,8,9,  10};/*10 is termination. must not be moved*/
	uint8_t test_list_answer[10] = {1,2,3,4,6,7,8,9,0,  10};/*10 is termination. must not be moved*/
	remove_head_from_data(&test_list_target[4],4);
#if 0
	{
	int i;
	for(i =0;i<10;i++)
		pr_debug("%d ",test_list_target[i]);
	}
	pr_debug("\n");
#endif
	LONGS_EQUAL(0, memcmp((const void *)test_list_target,(const void *)test_list_answer,10));
	
}





/*
 * OUI : Organizationally Unique Identifier
 * http://standards.ieee.org/develop/regauth/oui/public.html
 * HDMI is defined with 000C03.
 */
TEST(MhlTxEDID, edid_parser__dvi_when_IEEE_OUI_is_not_hdmi) {

}


uint8_t original_short_video_descriptor [11] = {
	0x84,
	0x3E,
	0x10, /* 16  1920x1080p : 59.94/60Hz 16:9 */
	0x20, /* 32  1920x1080p : 23.97/24Hz 16:9 */
	0x22, /* 34  1920x1080p : 29.97/30Hz 16:9 */
	0x05, /* 5   1920x1080i : 59.94/60Hz 16:9 */
	0x03, /* 3   720x480p   : 59.94/60Hz 16:9 */
	0x02, /* 2   */
	0x07, /* 7   */
	0x06, /* 6   */
	0x01, /* 1   640x480p   : 59.94/60Hz 4:3 */
};

/* Scenario : In MHL 1/2, Remove vics except for 640x480p60_4_3,
 *720x480p60_16_9, 1280x720p60_16_9, 1920x1080p24_16_9, 1920x1080p30_16_9, 1920x1080p60_16_9 (Packed Pixel)
 */
TEST(MhlTxEDID, edid_lib__remove_vic_except_for_1_3_4_16_32_34_fromd_svd) {
/* svd : short video descriptor */
	int size = 0;

	uint8_t filter[6] = {
		0x01,
		0x03,
		0x04,
		0x10,
		0x20,
		0x22
	};


	uint8_t filtered_short_video_descriptor [6] = {
		0x84, /* native vit is ignored (0x84 is regarded as 0x04) */
		0x10, /* 16  1920x1080p : 59.94/60Hz 16:9 */
		0x20, /* 32  1920x1080p : 23.97/24Hz 16:9 */
		0x22, /* 34  1920x1080p : 29.97/30Hz 16:9 */
		0x03, /* 3   720x480p   : 59.94/60Hz 16:9 */
		0x01, /* 1   640x480p   : 59.94/60Hz 4:3 */
	};

	size = mhl_lib_edid_remove_vic(original_short_video_descriptor, sizeof(original_short_video_descriptor),
	filter, sizeof(filter));

	LONGS_EQUAL(sizeof(filtered_short_video_descriptor), size);

	LONGS_EQUAL(0,
				memcmp((const void *)original_short_video_descriptor,
				 (const void *)(filtered_short_video_descriptor),
				 sizeof(filtered_short_video_descriptor)));
}

TEST(MhlTxEDID, edid_lib__pull_up) {
	uint8_t array[10] =           {1,2,3,4,5,6,7,8,9,10};
	uint8_t pulled_up_array[10] = {3,4,5,6,7,8,9,10,0,0};
	int pull_up_size = 2;

	LONGS_EQUAL(0, mhl_lib_edid_pull_up_and_padding(array, sizeof(array), pull_up_size));

	LONGS_EQUAL(0,
				memcmp((const void *)array,
				 (const void*) pulled_up_array,
				 sizeof(array)));
}

TEST(MhlTxEDID, edid_lib__pull_up_2) {
	uint8_t array[11] =           {1,2,3,4,5,6,7,8,9,10,11};
	uint8_t pulled_up_array[11] = {3,4,5,6,7,8,9,10,11,0,0};
	int pull_up_size = 2;

	LONGS_EQUAL(0, mhl_lib_edid_pull_up_and_padding(array, sizeof(array), pull_up_size));

	LONGS_EQUAL(0,
				memcmp((const void *)array,
				 (const void*) pulled_up_array,
				 sizeof(array)));
}


TEST(MhlTxEDID, edid_lib__pull_up_3) {
	uint8_t array[11] =           {1,2,3,4,5,6,7,8,9,10,11};
	uint8_t pulled_up_array[11] = {4,5,6,7,8,9,10,11,0,0,0};
	int pull_up_size = 3;

	LONGS_EQUAL(0, mhl_lib_edid_pull_up_and_padding(array, sizeof(array), pull_up_size));

	LONGS_EQUAL(0,
				memcmp((const void *)array,
				 (const void*) pulled_up_array,
				 sizeof(array)));
}

TEST(MhlTxEDID, edid_lib__pull_up_4) {
	uint8_t array[11] =           {1,2,3,4,5,6,7,8,9,10,11};
	uint8_t pulled_up_array[11] = {4,5,6,7,8,9,10,11,0,0,0};
	int pull_up_size = 3;

	LONGS_EQUAL(0, mhl_lib_edid_pull_up_and_padding(array, sizeof(array), pull_up_size));

	LONGS_EQUAL(0,
				memcmp((const void *)array,
				 (const void*) pulled_up_array,
				 sizeof(array)));
}

TEST(MhlTxEDID, edid_lib__pull_up_5) {
	uint8_t array[11] =           {1,2,3,4,5,6,7,8,9,10,11};
	uint8_t pulled_up_array[11] = {6,7,8,9,10,11,0,0,0,0,0};
	int pull_up_size = 5;

	LONGS_EQUAL(0, mhl_lib_edid_pull_up_and_padding(array, sizeof(array), pull_up_size));

	LONGS_EQUAL(0,
				memcmp((const void *)array,
				 (const void*) pulled_up_array,
				 sizeof(array)));
}

TEST(MhlTxEDID, edid_lib__pull_up_6) {
	uint8_t array[10] =           {1,2,3,4,5,6,7,8,9,10};
	uint8_t pulled_up_array[10] = {6,7,8,9,10,0,0,0,0,0};
	int pull_up_size = 5;

	LONGS_EQUAL(0, mhl_lib_edid_pull_up_and_padding(array, sizeof(array), pull_up_size));

	LONGS_EQUAL(0,
				memcmp((const void *)array,
				 (const void*) pulled_up_array,
				 sizeof(array)));
}

TEST(MhlTxEDID, edid_lib__pull_up_7) {
	uint8_t array[9] =           {1,2,3,4,5,6,7,8,9};
	uint8_t pulled_up_array[9] = {6,7,8,9,0,0,0,0,0};
	int pull_up_size = 5;

	LONGS_EQUAL(0, mhl_lib_edid_pull_up_and_padding(array, sizeof(array), pull_up_size));

	LONGS_EQUAL(0,
				memcmp((const void *)array,
				 (const void*) pulled_up_array,
				 sizeof(array)));
}




/*
 *  MHL1/2 support vic (vic number)
 *    640x480p60_4_3    (1)
 *    720x480p60_16_9   (3)
 *    1280x720p60_16_9  (4)
 *    1920x1080p60_16_9 (16) <-- Needs packed pixel
 *    1920x1080p24_16_9 (32)
 *    1920x1080p30_16_9 (34)
 */

TEST(MhlTxEDID, edid_lib__remove_vic_except_for_1_3_4_16_32_34_in_mhl_1_2_cbus_mode) {
	uint8_t ext_edid[EDID_BLOCK_SIZE];

	/* setup */
	memcpy((void *)ext_edid, (const void *)edid_sharp_lc24mx1_ext_block_1, EDID_BLOCK_SIZE);
	cbus_mode = CBUS_oCBUS_PEER_IS_MHL1_2;
	mock("CBUS_CONTROL").expectOneCall("packed_pixel_available")
		.andReturnValue(1);/* pp support*/

	/* exe */
	edid_filter_prune_ext_blk(ext_edid);

#if 0
PRINT_DATA(edid_sharp_lc24mx1_ext_block_1, EDID_BLOCK_SIZE)
PRINT_DATA(ext_edid, EDID_BLOCK_SIZE)
PRINT_DATA(edid_sharp_lc24mx1_ext_block_1_remove_vic_62_5_2_7_6, EDID_BLOCK_SIZE)
#endif
	/* verify */
	LONGS_EQUAL(0,
				memcmp((const void *)ext_edid,
						(const void *)edid_sharp_lc24mx1_ext_block_1_remove_vic_62_5_2_7_6,
						 EDID_BLOCK_SIZE));
}

#if 0
/* When 2nd EDID block is receved */
TEST(MhlTxEDID, edid_block_0_then_1_has_been_arrived) {
	bool is_next_block_requested_in_1st_call = true;
	bool is_next_block_requested_in_2nd_call = false;

	/* setup int val */
	uint8_t int_9_status = BIT_INTR9_EDID_DONE; /*0x20*/
	/* to avoid order error */
	mock().clear();

	mock("CBUS_CONTROL").expectOneCall("packed_pixel_available")
		.andReturnValue(1);//pp support
//	mock("CBUS_CONTROL").ignoreOtherCalls();

	set_current_edid_request_block(0);

	/* setup for 1st edid block. (edid block 0)*/
	setup_int_edid_isr_edid_block_arrived(&edid_sharp_lc24mx1_all_blocks[0], 0, is_next_block_requested_in_1st_call/*true*/);

	/* setup for 2nd edid block. (edid block 1)*/
	setup_int_edid_isr_edid_block_arrived(edid_sharp_lc24mx1_all_blocks, 1, is_next_block_requested_in_2nd_call/*false*/);

/*
	{
		uint16_t length = total_edid_block;
		uint8_t *edid = dummy_edid_2_blocks_data;

		setup_set_upstream_edid (get_stored_edid_block(),  length);
	}
*/
	/*mock("IO").ignoreOtherCalls();*/

	/* 1st int occurs for edid block 0  */
	pr_info("--1\n");
	LONGS_EQUAL(int_9_status,mhl_8620_dev_int_edid_isr(int_9_status));
	LONGS_EQUAL(0, memcmp((const void *)get_stored_edid_block(),
				(const void *)(edid_sharp_lc24mx1_all_blocks), EDID_BLOCK_SIZE));

	pr_info("--2\n");
	/* 2nd int occurs for edid block 1  */
	LONGS_EQUAL(int_9_status,mhl_8620_dev_int_edid_isr(int_9_status));

#if 0
	PRINT_DATA(get_stored_edid_block(), EDID_BLOCK_SIZE*2)
#endif

	LONGS_EQUAL(0, memcmp((const void *)get_stored_edid_block(),
				(const void *)(edid_sharp_lc24mx1_all_blocks), EDID_BLOCK_SIZE*2));
}
#endif

/* filter */

TEST(MhlTxEDID, filter_vic16_is_removed_when_sink_does_not_support_packed_pixel) {
	uint8_t sink_original_edid [sizeof(edid_sharp_lc24mx1_ext_block_1)] ;

	cbus_mode = CBUS_oCBUS_PEER_IS_MHL1_2;

	memcpy((void *)sink_original_edid ,(const void *)edid_sharp_lc24mx1_ext_block_1 , sizeof(edid_sharp_lc24mx1_ext_block_1));

	mock("CBUS_CONTROL").expectOneCall("mhl_cbus_packed_pixel_available")
		.andReturnValue(0);/* pp no support */

	edid_filter_prune_ext_blk(sink_original_edid);

#if 1
PRINT_DATA(sink_original_edid, EDID_BLOCK_SIZE)
#endif
#if 1
PRINT_DATA(edid_sharp_lc24mx1_ext_block_1_remove_specific_vic, EDID_BLOCK_SIZE)
#endif

	LONGS_EQUAL(0, memcmp((const void *)(sink_original_edid),
						(const void *)edid_sharp_lc24mx1_ext_block_1_remove_specific_vic,
						EDID_BLOCK_SIZE));
}

/* ext edid parse errors when the revision is not 3  */

TEST(MhlTxEDID, edid_lib__cbus_mode_is_set_device_file_internally) {

	/* cbus_mode is defined in mhl_xxx_device.c */
	cbus_mode = CBUS_NO_CONNECTION;
	LONGS_EQUAL(CBUS_NO_CONNECTION, mhl_device_get_cbus_mode());

	cbus_mode = CBUS_oCBUS_PEER_IS_MHL1_2;
	LONGS_EQUAL(CBUS_oCBUS_PEER_IS_MHL1_2, mhl_device_get_cbus_mode());
}

TEST(MhlTxEDID, edid_lib__remove_vic_except_for_1_3_4_16_32_34_from_edid) {
	uint8_t ext_edid[EDID_BLOCK_SIZE];
	memcpy((void *)ext_edid, (const void *)edid_sharp_lc24mx1_ext_block_1, EDID_BLOCK_SIZE);

	uint8_t not_removed_vic[6] = {
		0x01,
		0x03,
		0x04,
		0x10,
		0x20,
		0x22
	};

#if 0
PRINT_DATA(ext_edid, EDID_BLOCK_SIZE)
#endif

	mhl_lib_edid_remove_vic_from_svd(ext_edid, not_removed_vic,sizeof(not_removed_vic));

#if 1
PRINT_DATA(ext_edid, EDID_BLOCK_SIZE)
#endif

	LONGS_EQUAL(0,
				memcmp((const void *)ext_edid,
						(const void *)edid_sharp_lc24mx1_ext_block_1_remove_vic_62_5_2_7_6,
				 EDID_BLOCK_SIZE));

}


/*
//TODO
//mhl_lib_edid_pull_up_and_padding :
//if (head + pull_up_size < last_address)
//			memcpy((void *)head, (const void *)(head + pull_up_size), (size_t)pull_up_size);
//read shouldn't exceed the last address around the end.
*/


/*
//not_removed_vic_array is NULL, then every HDMI and 3D are removed
*/
TEST(MhlTxEDID, edid_lib___remove_hdmi_vic_and_3d_from_vsd) {
	uint8_t edid_ext[EDID_BLOCK_SIZE];

	memcpy((void *)edid_ext, (const void *)edid_sony_kld_rb1_block_ext_1, EDID_BLOCK_SIZE);

PRINT_DATA(edid_ext, 128)

	 mhl_lib_edid_remove_hdmi_vic_and_3d_from_vsd(edid_ext,
							 NULL,/*uint8_t *not_removed_vic_array,*/
							  0);/*uint8_t not_removed_vic_array_size)*/

PRINT_DATA(edid_ext, 128)

	LONGS_EQUAL(0,
				memcmp((const void *)edid_ext,
						(const void *)edid_sony_kld_rb1_block_ext_1_remove_hdmi_and_3d_from_vsd,
						 sizeof(edid_ext)));

}

TEST(MhlTxEDID, edid_lib___remove_hdmi_vic_and_3d_from_vsd_2__edid_ext_is_not_changed_with_vsd_length_is_8) {

	uint8_t edid_ext[EDID_BLOCK_SIZE];
	memcpy((void *)edid_ext, (const void *)edid_sony_kdl_32ex700_ext_block_1, EDID_BLOCK_SIZE);

#if 0
PRINT_DATA(edid_ext, EDID_BLOCK_SIZE)
#endif

	 mhl_lib_edid_remove_hdmi_vic_and_3d_from_vsd(edid_ext,
							 NULL,/*uint8_t *not_removed_vic_array,*/
							  0);/*uint8_t not_removed_vic_array_size)*/

#if 0
PRINT_DATA(edid_ext, EDID_BLOCK_SIZE)
#endif

	LONGS_EQUAL(0,
				memcmp((const void *)edid_ext,
						(const void *)edid_sony_kdl_32ex700_ext_block_1,
						 sizeof(edid_ext)));

}


TEST(MhlTxEDID, edid_lib__remove_1280x1024p60_5_4_and_1024x768p60_4_3_in_standard_timing) {

	struct mhl_video_timing_info video_timing_info[6];

	/* refer to mhl_sii8620_device_edid.c about the format */
	video_timing_info[0].vic = 1;
	video_timing_info[0].h_pixcel = 0x3B;
	video_timing_info[0].aspect_refresh = 0x40;

	video_timing_info[1].vic = 3;
	video_timing_info[1].h_pixcel = 0x3B;
	video_timing_info[1].aspect_refresh =  0xC0;

	video_timing_info[2].vic = 4;
	video_timing_info[2].h_pixcel = 0x81;
	video_timing_info[2].aspect_refresh = 0xC0;

	video_timing_info[3].vic = 16;
	video_timing_info[3].h_pixcel = 0xD1;
	video_timing_info[3].aspect_refresh = 0xC0;

	video_timing_info[4].vic = 32;
	video_timing_info[4].h_pixcel = 0xD1;
	video_timing_info[4].aspect_refresh = 0x68;

	video_timing_info[5].vic = 34;
	video_timing_info[5].h_pixcel = 0xD1;
	video_timing_info[5].aspect_refresh = 0xE2;

	uint8_t test_data [16] = {
		0x81, 0x80,/* 1280x1024p60_5_4 */
		0x61, 0x40,/* 1024x768p60_4_3 */
		0x01, 0x01,/* blank */
		0x01, 0x01,/* blank */
		0x01, 0x01,/* blank */
		0x01, 0x01,/* blank */
		0x01, 0x01,/* blank */
		0x01, 0x01,/* blank */
	};

	uint8_t expected_data [16] = {
		0x01, 0x01,/* blank */
		0x01, 0x01,/* blank */
		0x01, 0x01,/* blank */
		0x01, 0x01,/* blank */
		0x01, 0x01,/* blank */
		0x01, 0x01,/* blank */
		0x01, 0x01,/* blank */
		0x01, 0x01,/* blank */
	};

	mhl_lib_edid_remove_standard_timing(video_timing_info, 6, test_data);

	LONGS_EQUAL(0,
		memcmp((const void *)test_data,
				(const void *)expected_data,
				16));


}


TEST(MhlTxEDID, edid_lib__does_not_remove_1280x1024p60_5_4_and_1024x768p60_4_3_in_standard_timing) {

	struct mhl_video_timing_info video_timing_info[6];

	/* refer to mhl_sii8620_device_edid.c about the format */

	/* 1280x1024p60_5_4 */
	video_timing_info[0].vic = 150;
	video_timing_info[0].h_pixcel = 0x81;
	video_timing_info[0].aspect_refresh = 0x80;

	/* 1024x768p60_4_3 */
	video_timing_info[1].vic = 151;
	video_timing_info[1].h_pixcel = 0x61;
	video_timing_info[1].aspect_refresh =  0x40;

	video_timing_info[2].vic = 4;
	video_timing_info[2].h_pixcel = 0x81;
	video_timing_info[2].aspect_refresh = 0xC0;

	video_timing_info[3].vic = 16;
	video_timing_info[3].h_pixcel = 0xD1;
	video_timing_info[3].aspect_refresh = 0xC0;

	video_timing_info[4].vic = 32;
	video_timing_info[4].h_pixcel = 0xD1;
	video_timing_info[4].aspect_refresh = 0x68;

	video_timing_info[5].vic = 34;
	video_timing_info[5].h_pixcel = 0xD1;
	video_timing_info[5].aspect_refresh = 0xE2;

	uint8_t test_data [16] = {
		0x81, 0x80,/* 1280x1024p60_5_4 */
		0x61, 0x40,/* 1024x768p60_4_3 */
		0x01, 0x01,/* blank */
		0x01, 0x01,/* blank */
		0x01, 0x01,/* blank */
		0x01, 0x01,/* blank */
		0x01, 0x01,/* blank */
		0x01, 0x01,/* blank */
	};

	uint8_t expected_data [16] = {
		0x81, 0x80,/* 1280x1024p60_5_4 */
		0x61, 0x40,/* 1024x768p60_4_3 */
		0x01, 0x01,/* blank */
		0x01, 0x01,/* blank */
		0x01, 0x01,/* blank */
		0x01, 0x01,/* blank */
		0x01, 0x01,/* blank */
		0x01, 0x01,/* blank */
	};

	mhl_lib_edid_remove_standard_timing(video_timing_info, 6, test_data);

#if 1
PRINT_DATA(test_data, 16)
#endif

	LONGS_EQUAL(0,
		memcmp((const void *)test_data,
				(const void *)expected_data,
				16));
}


/*
 * This test is for establish timing. So the other than
 * established data are not meaning.
 */
TEST(MhlTxEDID, edid_lib__remove_all_in_established_timing) {

	struct mhl_video_timing_info video_timing_info[6];

	/* refer to mhl_sii8620_device_edid.c about the format */

	video_timing_info[0].vic = 1;
	video_timing_info[0].h_pixcel = 0x3B;
	video_timing_info[0].aspect_refresh = 0x40;
	video_timing_info[0].est_timing_1 = 0x20;
	video_timing_info[0].est_timing_2 = 0x00;
	video_timing_info[0].est_timing_3 = 0x00;

	/* 1024x768p60_4_3 */
	video_timing_info[1].vic = 151;
	video_timing_info[1].h_pixcel = 0x61;
	video_timing_info[1].aspect_refresh =  0x40;
	video_timing_info[1].est_timing_1 = 0x40;
	video_timing_info[1].est_timing_2 = 0x00;
	video_timing_info[1].est_timing_3 = 0x00;

	video_timing_info[2].vic = 4;
	video_timing_info[2].h_pixcel = 0x81;
	video_timing_info[2].aspect_refresh = 0xC0;
	video_timing_info[2].est_timing_1 = 0x00;
	video_timing_info[2].est_timing_2 = 0x08;
	video_timing_info[2].est_timing_3 = 0x00;

	video_timing_info[3].vic = 16;
	video_timing_info[3].h_pixcel = 0xD1;
	video_timing_info[3].aspect_refresh = 0xC0;
	video_timing_info[3].est_timing_1 = 0x00;
	video_timing_info[3].est_timing_2 = 0x80;
	video_timing_info[3].est_timing_3 = 0x00;

	video_timing_info[4].vic = 32;
	video_timing_info[4].h_pixcel = 0xD1;
	video_timing_info[4].aspect_refresh = 0x68;
	video_timing_info[4].est_timing_1 = 0x00;
	video_timing_info[4].est_timing_2 = 0x10;
	video_timing_info[4].est_timing_3 = 0x10;

	video_timing_info[5].vic = 34;
	video_timing_info[5].h_pixcel = 0xD1;
	video_timing_info[5].aspect_refresh = 0xE2;
	video_timing_info[5].est_timing_1 = 0x00;
	video_timing_info[5].est_timing_2 = 0x00;
	video_timing_info[5].est_timing_3 = 0x01;

	uint8_t test_data [3] = {
		0x21,/* Established Timing I */
		0x08,/* Established Timing II */
		0x31,/* Established Timing III */
	};

	uint8_t expected_data [3] = {
		0x20,/* Established Timing I */
		0x08,/* Established Timing II */
		0x11,/* Established Timing III */
	};

	mhl_lib_edid_remove_established_timing(video_timing_info, 6, test_data);

#if 1
PRINT_DATA(test_data, 3)
#endif

	LONGS_EQUAL(0,
		memcmp((const void *)test_data,
				(const void *)expected_data,
				3));
}



TEST(MhlTxEDID, edid_lib__remove_all_in_established_timing_2) {

	struct mhl_video_timing_info video_timing_info[6];

	/* refer to mhl_sii8620_device_edid.c about the format */
#define HDMI_VFRMT_640x480p60_4_3_TIMING    {1,  0x31, 0x40, 0x20, 0, 0}
#define HDMI_VFRMT_720x480p60_16_9_TIMING   {3,  0x3B, 0xC0, 0, 0, 0}
#define HDMI_VFRMT_1280x720p60_16_9_TIMING  {4,  0x81, 0xC0, 0, 0, 0}
#define HDMI_VFRMT_1920x1080p60_16_9_TIMING	{16, 0xD1, 0xC0, 0, 0, 0}
#define HDMI_VFRMT_1920x1080p24_16_9_TIMING {32, 0xD1, 0x68, 0, 0, 0}
#define HDMI_VFRMT_1920x1080p30_16_9_TIMING {34, 0xD1, 0xE2, 0, 0, 0}
#define SUPPORT_MHL_1_2_VIDEO_NUM 6

const struct mhl_video_timing_info mhl_1_2_support_video[SUPPORT_MHL_1_2_VIDEO_NUM] = {
	HDMI_VFRMT_640x480p60_4_3_TIMING,
	HDMI_VFRMT_720x480p60_16_9_TIMING,
	HDMI_VFRMT_1280x720p60_16_9_TIMING,
	HDMI_VFRMT_1920x1080p60_16_9_TIMING,
	HDMI_VFRMT_1920x1080p24_16_9_TIMING,
	HDMI_VFRMT_1920x1080p30_16_9_TIMING
};

	uint8_t test_data [3] = {
		0x21,/* Established Timing I */
		0x08,/* Established Timing II */
		0x31,/* Established Timing III */
	};

	uint8_t expected_data [3] = {
		0x20,/* Established Timing I */
		0x00,/* Established Timing II */
		0x00,/* Established Timing III */
	};

	mhl_lib_edid_remove_established_timing(mhl_1_2_support_video, SUPPORT_MHL_1_2_VIDEO_NUM, test_data);

#if 0
PRINT_DATA(test_data, 3)
#endif

	LONGS_EQUAL(0,
		memcmp((const void *)test_data,
				(const void *)expected_data,
				3));
}




TEST(MhlTxEDID, edid_lib__remove_blk0_unused_video) {

	uint8_t blk0[EDID_BLOCK_SIZE];
	memcpy((void *)blk0, (const void *)edid_cts_simplay_block_0, EDID_BLOCK_SIZE);

	const struct mhl_video_timing_info mhl_1_2_support_video[SUPPORT_MHL_1_2_VIDEO_NUM] = {
		HDMI_VFRMT_640x480p60_4_3_TIMING,
		HDMI_VFRMT_720x480p60_16_9_TIMING,
		HDMI_VFRMT_1280x720p60_16_9_TIMING,
		HDMI_VFRMT_1920x1080p60_16_9_TIMING,
		HDMI_VFRMT_1920x1080p24_16_9_TIMING,
		HDMI_VFRMT_1920x1080p30_16_9_TIMING};

#if 0
PRINT_DATA(blk0, EDID_BLOCK_SIZE)
#endif

	edid_filter_prune_block_0(blk0);

#if 0
PRINT_DATA(blk0, EDID_BLOCK_SIZE)
#endif

	LONGS_EQUAL(0,
		memcmp((const void *)blk0,
				(const void *)edid_cts_simplay_block_0__remain_bit_5_at_et1_and_st_is_all_blank,
				EDID_BLOCK_SIZE));
}





TEST(MhlTxEDID, return_true_when_not_all_data_are_same) {

uint8_t test_list[128] = {
	0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xFF, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
	0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
	0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
	0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
	0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
	0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
	0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
	0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0 };

	LONGS_EQUAL(true, mhl_dev_edid_is_fifo_hw_good_cond(test_list));
}

TEST(MhlTxEDID, return_false_when_all_data_are_same) {

uint8_t test_list[128] = {
	0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
	0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
	0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
	0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
	0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
	0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
	0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
	0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0 };

	LONGS_EQUAL(false, mhl_dev_edid_is_fifo_hw_good_cond(test_list));
	LONGS_EQUAL(true, mhl_lib_edid_is_valid_checksum(test_list));
}


TEST(MhlTxEDID, return_false_when_all_data_are_same_2) {

uint8_t test_list[128] = {
	0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79,
	0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79,
	0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79,
	0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79,
	0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79,
	0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79,
	0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79,
	0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79 };

	LONGS_EQUAL(false, mhl_lib_edid_is_valid_checksum(test_list));
}





#if 0
TEST(MhlTxEDID, protocol_condition_set) {

	mhl_dev_set_cbusp_cond_processing(DEV_EDID_READ);

	LONGS_EQUAL(DEV_EDID_READ, mhl_device_get_cbusp_cond());
	LONGS_EQUAL(true, mhl_device_is_cbusb_executing());
}

TEST(MhlTxEDID, protocol_condition_initialize) {
	LONGS_EQUAL(0, mhl_device_get_cbusp_cond());
	LONGS_EQUAL(false, mhl_device_is_cbusb_executing());
}
#endif
#if 0
TEST(MhlTxEDID, protocol_condition_clear_part_of_condition) {

	mhl_dev_set_cbusp_cond_processing(DEV_EDID_READ);
	mhl_dev_set_cbusp_cond_processing(DEV_RESERVE_P_1);
	mhl_dev_clear_cbusp_cond_processing(DEV_EDID_READ);

	LONGS_EQUAL(DEV_RESERVE_P_1, mhl_device_get_cbusp_cond());
	LONGS_EQUAL(true, mhl_device_is_cbusb_executing());
}


TEST(MhlTxEDID, protocol_condition_clear_all_condition) {

	mhl_dev_set_cbusp_cond_processing(DEV_EDID_READ);
	mhl_dev_set_cbusp_cond_processing(DEV_RESERVE_P_1);
	mhl_dev_clear_cbusp_cond_processing(DEV_EDID_READ);
		mhl_dev_clear_cbusp_cond_processing(DEV_RESERVE_P_1);

	LONGS_EQUAL(0, mhl_device_get_cbusp_cond());
	LONGS_EQUAL(false, mhl_device_is_cbusb_executing());
}
#endif

#if 0
/* receive edid block 0 */
TEST(MhlTxEDID, edid_block_0_has_been_arrived) {
	bool is_next_block_requested = true;

	/* setup int val */
	uint8_t int_9_status = BIT_INTR9_EDID_DONE; /*0x20*/

	set_current_edid_request_block(0);
	setup_int_edid_isr_edid_block_arrived(dummy_edid_5_blocks_data, 0, is_next_block_requested);

	LONGS_EQUAL(int_9_status,mhl_8620_dev_int_edid_isr(int_9_status));

	/* the check size is only 128 byte */
	LONGS_EQUAL(0, memcmp((const void *)get_stored_edid_block(),
				(const void *)(dummy_edid_5_blocks_data), EDID_BLOCK_SIZE));
}
#endif


TEST(MhlTxEDID, set_hev_sprt_2_times) {
	const uint8_t in_hev[3] = {93, 94, 95};
	const uint8_t in_hev2[3] = {98, 99};
	const uint8_t *hev = NULL;
	uint8_t length;

	isMhlSuptFilter = false;

	mhl_device_edid_add_mhlsink_sprt_hev(in_hev, 3);
	mhl_device_edid_add_mhlsink_sprt_hev(in_hev2, 2);
	
	hev = mhl_device_edid_get_mhlsink_sprt_hev(&length);

	LONGS_EQUAL(5, length);
	LONGS_EQUAL(93, hev[0]);
	LONGS_EQUAL(94, hev[1]);
	LONGS_EQUAL(95, hev[2]);
	LONGS_EQUAL(98, hev[3]);
	LONGS_EQUAL(99, hev[4]);
}

/* The support vic can be referred in "mhl_3_support_video" */
TEST(MhlTxEDID, ignore_non_support_hev) {
	const uint8_t in_hev[5] = {93, 94, 95, 98, 99};
	const uint8_t *hev = NULL;
	uint8_t length;

	/* isMhlSuptFilter = false; */

	mhl_device_edid_add_mhlsink_sprt_hev(in_hev, 5);

	hev = mhl_device_edid_get_mhlsink_sprt_hev(&length);

	LONGS_EQUAL(2, length);
	LONGS_EQUAL(93, hev[0]);
	LONGS_EQUAL(95, hev[1]);

}

TEST(MhlTxEDID, max_20_hev_info_can_be_added) {
	const uint8_t in_he_max[20] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
	const uint8_t *hev = NULL;
	uint8_t length;

	isMhlSuptFilter = false;

	mhl_device_edid_add_mhlsink_sprt_hev(in_he_max, 20);

	hev = mhl_device_edid_get_mhlsink_sprt_hev(&length);


	LONGS_EQUAL(20, length);
	LONGS_EQUAL(0, memcmp((const void *)in_he_max, (const void *)hev, length));
}

TEST(MhlTxEDID, max_20_hev_info_can_be_added_even_exceeded) {
	const uint8_t in_he_exceed_max[21] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21};
	const uint8_t *hev = NULL;
	uint8_t length;

	isMhlSuptFilter = false;

	mhl_device_edid_add_mhlsink_sprt_hev(in_he_exceed_max, 21);

	hev = mhl_device_edid_get_mhlsink_sprt_hev(&length);


	LONGS_EQUAL(20, length);
	LONGS_EQUAL(0, memcmp((const void *)in_he_exceed_max, (const void *)hev, 20));
}

TEST(MhlTxEDID, hev_convert_from_vic_to_hdmi_vic) {
	const uint8_t vic_95 = 95;/* 3840x2160@30fps > 0x01*/
	const uint8_t vic_94 = 94;/* 3840x2160@25fps > 0x02*/
	const uint8_t vic_93 = 93;/* 3840x2160@24fps > 0x03*/
	const uint8_t vic_98 = 98;/* 4096x2160@24fps > 0x04*/

	LONGS_EQUAL(1, get_converted_hdmi_vic(vic_95));
	LONGS_EQUAL(2, get_converted_hdmi_vic(vic_94));
	LONGS_EQUAL(3, get_converted_hdmi_vic(vic_93));
	LONGS_EQUAL(4, get_converted_hdmi_vic(vic_98));
}

TEST(MhlTxEDID, error_hev_convert_from_vic_to_hdmi_vic) {
	const uint8_t vic = 100;

	LONGS_EQUAL(-EINVAL, get_converted_hdmi_vic(vic));
}

 

/* when vsd size is less than 19, then return 0.  */
TEST(MhlTxEDID, add_hdmi_vic_pattern_2) {
	int ret = mhl_lib_edid_add_hdmi_vic(NULL, 14 + MHL_MAX_HDMI_BIC_NUM - 1, 0);
		LONGS_EQUAL(-EINVAL, ret);
}

/* when vsd size is less than 19, then return 0. shoudl be less than hdmi vic maximum number  */

#if 0
	uint8_t vsd_body[20] = {
		0x03, 0x0c, 0x00, /* HDMI OUI , 1~3th byte*/\
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0x2f,
		0xd0,
		0x0a, /* 10th byte */
		0x01,
		0x40,
		0xF8,/* 13th byte */
		0x7f,/* 14th byte */
		0x20, 0x30, 0x70, 0x80, 0x90, 0x76 /* 20th byte*/
	};
	uint8_t expected_vsd_body[20] = {
		0x03, 0x0c, 0x00, /* 1~3th byte  HDMI OUI */
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		/*0x2f*/0x0F, /* 8th byte*/
		0xd0,
		0x0a, /* 10th byte */
		0x01,
		0x40,
		0x00,/* 13th byte */
		/*0x7f*/0x00, /* 14th byte HDMI_VIC_LEN / HDMI_3D_LEN*/
		/*0x20, 0x30, 0x70, 0x80, 0x90, 0x76*/0x00, 0x00, 0x00, 0x00, 0x00, 0x00/* 20th byte*/
	};
#endif




/*
TEST(MhlTxEDID, add_hdmi_vic_to_ext_blcok) {
	int hdmi_vic = MHL_BIT_HDMI_VIC_1 | MHL_BIT_HDMI_VIC_2 | MHL_BIT_HDMI_VIC_3 | MHL_BIT_HDMI_VIC_4;
	uint8_t ext_block[EDID_BLOCK_SIZE];

	memcpy((void *)ext_block, (const void *)edid_sharp_lc24mx1_ext_block_1, EDID_BLOCK_SIZE);

	mhl_lib_edid_remove_hdmi_vic_and_3d_from_vsd(ext_block);
	mhl_lib_edid_add_hdmi_vic(ext_block, hdmi_vic);
}
*/







TEST(MhlTxEDID, edid_lib__is_supp_hdmi_vic) {
	uint8_t hdmi_vic_3 = 3;
	uint8_t hdmi_vic_1 = 1;
	uint8_t hdmi_vic_2 = 2;
	uint8_t hdmi_vic_4 = 4;
	uint8_t supp_vic[2] = {93,95};

	bool is_supp = is_supp_hdmi_vic(hdmi_vic_3, supp_vic, sizeof(supp_vic));
	LONGS_EQUAL(true, is_supp);

	is_supp = is_supp_hdmi_vic(hdmi_vic_1, supp_vic, sizeof(supp_vic));
	LONGS_EQUAL(true, is_supp);

	is_supp = is_supp_hdmi_vic(hdmi_vic_2, supp_vic, sizeof(supp_vic));
	LONGS_EQUAL(false, is_supp);

	is_supp = is_supp_hdmi_vic(hdmi_vic_4, supp_vic, sizeof(supp_vic));
	LONGS_EQUAL(false, is_supp);
}








TEST(MhlTxEDID, edid_lib__remove_hdmi_vic_and_3d_v4) {
	uint8_t vsd_body[21] = {
		0x74,
		0x03, 0x0c, 0x00, /* HDMI OUI , 1~3th byte*/\
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0x2f,
		0xd0,
		0x67, /* 10th byte */
		0x00,
		0x01,
		0x03,/* 13th byte */
		0x7f,/* 14th byte */
		0x00, 0x01, 0x03, 0x80, 0x90, 0x76 /* 20th byte*/
	};
	const uint8_t expected_vsd_body[21] = {
		0x6c,
		0x03, 0x0c, 0x00, /* 1~3th byte  HDMI OUI */
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0x2f, /* 8th byte*/
		0x10,
		0x40, /* 10th byte */
		0x01,
		0x03,
		0x00,/* 13th byte */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00/* 20th byte*/
	};

	int size = 0;
	uint8_t vsd[21];
	uint8_t not_removed_vic[2] = {93/* hdmi vic 3 */, 95/* hdmi vic 1 */};

	memcpy((void *)vsd, (const void *)vsd_body, sizeof(vsd));

	mhl_lib_edid_replace_hdmi_vic_and_remove_3d(vsd, 20, not_removed_vic, sizeof(not_removed_vic));

	LONGS_EQUAL(0,
				memcmp((const void *)vsd,
						(const void *)expected_vsd_body,
						 20));

}


TEST(MhlTxEDID, edid_lib__remove_hdmi_vic_and_3d_with_latency_fields) {
	uint8_t vsd_body[25] = {
		0x78,
		0x03, 0x0c, 0x00, /* HDMI OUI , 1~3th byte*/\
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0xef,
		0xaa,
		0xbb,
		0xcc,
		0xdd,
		0xd0,
		0x86, /* HDMI VIC LEN : 3   10th byte */
		0x01,
		0x02,
		0x03,/* 13th byte */
		0x04,/* 14th byte */
		0x02, 0x01, 0x03, 0x80, 0x90, 0x76 /* 20th byte*/
	};
	const uint8_t expected_vsd_body[25] = {
		0x70,
		0x03, 0x0c, 0x00, /* 1~3th byte  HDMI OUI */
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0xef, /* 8th byte*/
		0xaa,
		0xbb,
		0xcc,
		0xdd,
		0x10,
		0x40, /* HDMI VIC LEN : 2  10th byte */
		0x01,
		0x03,
		0x00,/* 13th byte */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00/* 20th byte*/
	};

	int size = 0;
	uint8_t vsd[25];
	uint8_t not_removed_vic[2] = {93/* hdmi vic 3 */, 95/* hdmi vic 1 */};

	memcpy((void *)vsd, (const void *)vsd_body, sizeof(vsd));

	mhl_lib_edid_replace_hdmi_vic_and_remove_3d(vsd, 24, not_removed_vic, sizeof(not_removed_vic));

	LONGS_EQUAL(0,
				memcmp((const void *)vsd,
						(const void *)expected_vsd_body,
						 24));

}

TEST(MhlTxEDID, edid_lib__remove_hdmi_vic_and_3d_with_hdmi_vic_0) {
	uint8_t vsd_body[25] = {
		0x78,
		0x03, 0x0c, 0x00, /* HDMI OUI , 1~3th byte*/\
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0xef,
		0xaa,
		0xbb,
		0xcc,
		0xdd,
		0xd0,
		0x06, /* HDMI VIC LEN : 0   10th byte */
		0x02, 0x01, 0x03, 0x80, 0x90, 0x76 /* 20th byte*/
	};
	const uint8_t expected_vsd_body[25] = {
		0x6e,
		0x03, 0x0c, 0x00, /* 1~3th byte  HDMI OUI */
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0xef, /* 8th byte*/
		0xaa,
		0xbb,
		0xcc,
		0xdd,
		0x10,
		0x00, /* HDMI VIC LEN : 2  10th byte */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00/* 20th byte*/
	};

	int size = 0;
	uint8_t vsd[25];
	uint8_t not_removed_vic[2] = {93/* hdmi vic 3 */, 95/* hdmi vic 1 */};

	memcpy((void *)vsd, (const void *)vsd_body, sizeof(vsd));

	mhl_lib_edid_replace_hdmi_vic_and_remove_3d(vsd, 24, not_removed_vic, sizeof(not_removed_vic));

	LONGS_EQUAL(0,
				memcmp((const void *)vsd,
						(const void *)expected_vsd_body,
						 20));

}





TEST(MhlTxEDID, edid_lib__remove_hdmi_vic_and_3d__over_vsd_size) {
	uint8_t vsd_body[25] = {
		0x68,
		0x03, 0x0c, 0x00, /* HDMI OUI , 1~3th byte*/\
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0xef,
		0xaa,
		0xbb,
		0xcc,
		0xdd,
		0xd0,
		0x06, /* HDMI VIC LEN : 0   10th byte */
		0x02, 0x01, 0x03, 0x80, 0x90, 0x76 /* 20th byte*/
	};

	int size = 0;
	uint8_t vsd[25];
	uint8_t not_removed_vic[2] = {93/* hdmi vic 3 */, 95/* hdmi vic 1 */};

	memcpy((void *)vsd, (const void *)vsd_body, sizeof(vsd));

	mhl_lib_edid_replace_hdmi_vic_and_remove_3d(vsd, 8, not_removed_vic, sizeof(not_removed_vic));

	LONGS_EQUAL(0,
				memcmp((const void *)vsd,
						(const void *)vsd_body,
						 20));

}


TEST(MhlTxEDID, edid_lib__remove_hdmi_vic_and_3d__illegal_vsd_size) {
	uint8_t vsd_body[25] = {
		0x68,
		0x03, 0x0c, 0x00, /* HDMI OUI , 1~3th byte*/\
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0xef,
		0xaa,
		0xbb,
		0xcc,
		0xdd,
		0xd0,
		0x06, /* HDMI VIC LEN : 0   10th byte */
		0x02, 0x01, 0x03, 0x80, 0x90, 0x76 /* 20th byte*/
	};

	int size = 0;
	uint8_t vsd[25];
	uint8_t not_removed_vic[2] = {93/* hdmi vic 3 */, 95/* hdmi vic 1 */};

	memcpy((void *)vsd, (const void *)vsd_body, sizeof(vsd));

	mhl_lib_edid_replace_hdmi_vic_and_remove_3d(vsd, 9, not_removed_vic, sizeof(not_removed_vic));

	LONGS_EQUAL(0,
				memcmp((const void *)vsd,
						(const void *)vsd_body,
						 25));

}


TEST(MhlTxEDID, set_4k_filter_factor) {
	const uint8_t in_hev[3] = {93, 94, 95};
	const uint8_t *hev = NULL;
	uint8_t length;
	isMhlSuptFilter = false;

	mhl_device_edid_add_mhlsink_sprt_hev(in_hev, 3);
	
	hev = mhl_device_edid_get_mhlsink_sprt_hev(&length);
	
	LONGS_EQUAL(3, length);
	LONGS_EQUAL(93, hev[0]);
	LONGS_EQUAL(94, hev[1]);
	LONGS_EQUAL(95, hev[2]);
	
}




TEST(MhlTxEDID, mhl_3_support_vic) {
	int i;

	LONGS_EQUAL(ARRAY_SIZE(mhl_3_support_video), ARRAY_SIZE(mhl_3_support_vic_array));

	for (i = 0; i < ARRAY_SIZE(mhl_3_support_video); i++) {
		LONGS_EQUAL(mhl_3_support_video[i].vic, mhl_3_support_vic_array[i]);
	}
}


TEST(MhlTxEDID, edid_lib__remove_hdmi_vic_and_3d) {
	uint8_t vsd_body[21] = {
		0x74,
		0x03, 0x0c, 0x00, /* HDMI OUI , 1~3th byte*/
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0x2f,/* HDMI video Present*/
		0xd0,
		0x0a, /* 10th byte */
		0x01,
		0x40,
		0xF8,/* 13th byte */
		0x7f,/* 14th byte */
		0x20, 0x30, 0x70, 0x80, 0x90, 0x76 /* 20th byte*/
	};
	uint8_t expected_vsd_body[21] = {
		0x69,
		0x03, 0x0c, 0x00, /* 1~3th byte  HDMI OUI */
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0x0F, /* 8th byte*/
		0x10,
		0x00, /* 10th byte */
		0x00,
		0x00,
		0x00,/* 13th byte */
		/*0x7f*/0x00, /* 14th byte HDMI_VIC_LEN / HDMI_3D_LEN*/
		/*0x20, 0x30, 0x70, 0x80, 0x90, 0x76*/0x00, 0x00, 0x00, 0x00, 0x00, 0x00/* 20th byte*/
	};

	int size = 0;
	uint8_t vsd[21];

	memcpy((void *)vsd, (const void *)vsd_body, 21);

	size = mhl_lib_edid_remove_hdmi_vic_and_3d(vsd, 20);

	LONGS_EQUAL(0,
				memcmp((const void *)vsd,
						(const void *)expected_vsd_body,
						 9));
	LONGS_EQUAL(9, size);
}

TEST(MhlTxEDID, edid_lib__not_remove_hdmi_vic_and_3d_when_hdmi_video_present_is_not) {
	uint8_t vsd_body[21] = {
		0x74,
		0x03, 0x0c, 0x00, /* HDMI OUI , 1~3th byte*/\
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0x0f,/* HDMI_Video_present is not set*/
		0xd0,
		0x0a,
		0x01,
		0x40,
		0xF8,
		0x7f,
		0x20, 0x30, 0x70, 0x80, 0x90, 0x76 /* 20th byte*/
	};
	uint8_t expected_vsd_body[22] = {
		0x69,
		0x03, 0x0c, 0x00, /* HDMI OUI , 1~3th byte*/\
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0x0f,/* HDMI_Video_present is not set*/
		0xd0,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAA /* 20th byte*/
	};

	int size = 0;
	uint8_t vsd[21];

	memcpy((void *)vsd, (const void *)vsd_body, 21);

	size = mhl_lib_edid_remove_hdmi_vic_and_3d(vsd, 20);

	LONGS_EQUAL(0,
				memcmp((const void *)vsd,
						(const void *)expected_vsd_body,
						 sizeof(21)));
	LONGS_EQUAL(0xAA, expected_vsd_body[21]);/* check memory break */
	LONGS_EQUAL(9, size);
}



TEST(MhlTxEDID, edid_lib__not_remove_hdmi_vic_and_3d_when_length_is_less_than_9) {
	uint8_t vsd_body[21] = {
		0x68,
		0x03, 0x0c, 0x00, /* HDMI OUI , 1~3th byte*/\
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0x2f,/* HDMI_Video_present is set*/
		0xd0,/* Video_Latency*/
		0x0a,/* 10th byte Audio Latency */
		0x01,/* Interlaced_Video_Latency */
		0x40,/* Interlaced_Audio_Latency */
		0xF8,/* 13th byte */
		0x7f,/* 14th byte */
		0x20, 0x30, 0x70, 0x80, 0x90, 0x76 /* 20th byte*/
	};
	int size = 0;
	uint8_t vsd[21];

	memcpy((void *)vsd, (const void *)vsd_body,21);

	size = mhl_lib_edid_remove_hdmi_vic_and_3d(vsd, 8);

	LONGS_EQUAL(0,
				memcmp((const void *)vsd,
						(const void *)vsd_body,
						 sizeof(vsd)));
	LONGS_EQUAL(8,size);
}



TEST(MhlTxEDID, edid_lib__remove_hdmi_vic_and_3d_with_latencies) {
	uint8_t vsd_body[16] = {
		0x6f,
		0x03, 0x0c, 0x00, /* HDMI OUI , 1~3th byte*/\
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0xff, /* HDMI_Video_present is set*/
		0xd0,/* Video_Latency*/
		0x0a,/* 10th byte Audio Latency */
		0x01,/* Interlaced_Video_Latency */
		0x40,/* Interlaced_Audio_Latency */
		0xFF,/* 13th byte */
		0xFF,/* 14th byte */
		0x20 };
	uint8_t expected_vsd_body[16] = {
		0x6d,
		0x03, 0x0c, 0x00, /* HDMI OUI , 1~3th byte*/\
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0xdf, /* HDMI_Video_present is set*/
		0xd0,/* Video_Latency*/
		0x0a,/* 10th byte Audio Latency */
		0x01,/* Interlaced_Video_Latency */
		0x40,/* Interlaced_Audio_Latency */
		0x18,/* 13th byte */
		0x00,/* 14th byte */
		0x00 };
	int size = 0;
	uint8_t vsd[16];

	memcpy((void *)vsd, (const void *)vsd_body,sizeof(vsd));

	size = mhl_lib_edid_remove_hdmi_vic_and_3d(vsd, 15);

	LONGS_EQUAL(0,
				memcmp((const void *)vsd,
						(const void *)expected_vsd_body,
						 sizeof(vsd)));
	LONGS_EQUAL(13, size);
}



TEST(MhlTxEDID, edid_lib__remove_hdmi_vic_and_3d_without_latencies) {
	uint8_t vsd_body[16] = {
		0x6f,
		0x03, 0x0c, 0x00, /* HDMI OUI , 1~3th byte*/\
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0x3f, /* HDMI_Video_present is set*/
		0xd0,
		0x0a,
		0x01,
		0x40,
		0xFF,
		0xFF,
		0x20 };
	uint8_t expected_vsd_body[16] = {
		0x69,
		0x03, 0x0c, 0x00, /* HDMI OUI , 1~3th byte*/\
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0x1f, /* HDMI_Video_present is cleared */
		0x10,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00 };
	int size = 0;
	uint8_t vsd[16];

	memcpy((void *)vsd, (const void *)vsd_body,sizeof(vsd));

	size = mhl_lib_edid_remove_hdmi_vic_and_3d(vsd, 15);

	LONGS_EQUAL(0,
				memcmp((const void *)vsd,
						(const void *)expected_vsd_body,
						 sizeof(vsd)));
	LONGS_EQUAL(9, size);
}


TEST(MhlTxEDID, edid_lib__remove_hdmi_vic_and_3d_different_btw_vsd_size_and_arg_in_length) {
	uint8_t vsd_body[16] = {
		0x6f,
		0x03, 0x0c, 0x00, /* HDMI OUI , 1~3th byte*/\
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0x3f, /* HDMI_Video_present is set*/
		0xd0,
		0x0a,
		0x01,
		0x40,
		0xFF,
		0xFF,
		0x20 };
	int size = 0;
	uint8_t vsd[16];

	memcpy((void *)vsd, (const void *)vsd_body,sizeof(vsd));

	size = mhl_lib_edid_remove_hdmi_vic_and_3d(vsd, 14);

	LONGS_EQUAL(0,
				memcmp((const void *)vsd,
						(const void *)vsd_body,
						 sizeof(vsd)));
	LONGS_EQUAL(15,size);
}




const static uint8_t ext_block_dummy_4k[EDID_BLOCK_SIZE] = {
0x02, 0x03,
0x3e, /* offset d is 62 */
0xf0,
/* video data block */ 
0x4c,
0x10, 0x05, 0x04, 0x20, 0x22, 0x3c, 0x3e, 0x03, 0x07, 0x02, 0x06, 0x01, /* <offset 16> */
/* */ 
0x29, 0x09, 0x07, 0x07, 0x15, 0x07, 0x50, 0x35, 0x05, 0x50, 0x83, 0x01, 0x00, 0x00, /* <offset 30> */
/* vendor specific data */ 
0x77, /* length : 23*/
0x03, 0x0c, 0x00, /* 1 ~ 3 byte */
0x20, 0x00, 0xb8, 0x3c, 0x2f, /* hdmi video present */
/* there is no latency info */
0xd0, /* 3D_present 1, 3D_Multi_present 10, Image_Size 10, Rsvd 0, Rsvd 0, Rsvd 0 */
0x89, /* HDMI VIC LEN : 4 / HDMI 3D LEN : 9 */
0x01, 0x02, 0x03, 0x04, /* hdmi vic */
0x01, 0x40, 0x00, 0x0f, 0x10, 0x40, 0x50, 0x60, 0x46, /* 3d */
/* Colorimetry Data Block  */
0xe2, 0x00, 0xfb, 0xe3, 0x05, 0x1f, 0x01,
/* Detail Timing Discriptions <offset 62 ~ > */
0x8c, 0x0a, 0xd0, 0x8a, 0x20, 0xe0, 0x2d, 0x10, 0x10, 0x3e, 0x96, 0x00, 0xc2, 0xad, 0x42, 0x00, 0x00, 0x18,
0x01, 0x1d, 0x80, 0x18, 0x71, 0x1c, 0x16, 0x20, 0x58, 0x2c, 0x25, 0x00, 0xc2, 0xad, 0x42, 0x00, 0x00, 0x9e,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xed};

const static uint8_t filtered_ext_block_dummy_4k[EDID_BLOCK_SIZE] = {
0x02, 0x03,
0x33, /* offset d is 62 -11 = 51*/
0xf0,
/* video data block */
0x4c,
0x10, 0x05, 0x04, 0x20, 0x22, 0x3c, 0x3e, 0x03, 0x07, 0x02, 0x06, 0x01, /* <offset 16> */
/* */
0x29, 0x09, 0x07, 0x07, 0x15, 0x07, 0x50, 0x35, 0x05, 0x50, 
0x83, 0x01, 0x00, 0x00, /* <offset 30> */
/* vendor specific data */
0x6c, /* length : 23 -> 12*/
0x03, 0x0c, 0x00, /* 1 ~ 3 byte */
0x20, 0x00, 0xb8, 0x3c, 0x2f, /* hdmi video present */
/* there is no latency info */\
0x10, /* 3D_present 0, 3D_Multi_present 00, Image_Size 10(no change), Rsvd 0, Rsvd 0, Rsvd 0 */
0x40, /* HDMI VIC LEN : 2 / HDMI 3D LEN : 0 */
0x01, 0x03,  /* hdmi vic */\
/*  */\
0xe2, 0x00, 0xfb, 0xe3, 0x05, 0x1f, 0x01,
/* Detail Timing Discriptions <offset 62 ~ > */
0x8c, 0x0a, 0xd0, 0x8a, 0x20, 0xe0, 0x2d, 0x10, 0x10, 0x3e, 0x96, 0x00, 0xc2, 0xad, 0x42, 0x00, 0x00, 0x18,
0x01, 0x1d, 0x80, 0x18, 0x71, 0x1c, 0x16, 0x20, 0x58, 0x2c, 0x25, 0x00, 0xc2, 0xad, 0x42, 0x00, 0x00, 0x9e,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa8};

#if 0
0x02, 0x03,
0x3e, /* offset d is 62 */
0xf0,
/* video data block */
0x4c,
0x10, 0x05, 0x04, 0x20, 0x22, 0x3c, 0x3e, 0x03, 0x07, 0x02, 0x06, 0x01, /* <offset 16> */
/* */
0x29, 0x09, 0x07, 0x07, 0x15, 0x07, 0x50, 0x35, 0x05, 0x50, 
0x83, 0x01, 0x00, 0x00, /* <offset 30> */
/* vendor specific data */
0x77, /* length : 23*/
0x03, 0x0c, 0x00, /* 1 ~ 3 byte */
0x20, 0x00, 0xb8, 0x3c, 0x2f, /* hdmi video present */
/* there is no latency info */\
0x10, /* 3D_present 0, 3D_Multi_present 00, Image_Size 10(no change), Rsvd 0, Rsvd 0, Rsvd 0 */
0x40, /* HDMI VIC LEN : 4 / HDMI 3D LEN : 9 */
0x01, 0x03, 0x00, 0x00, /* hdmi vic */\
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 3d */\
0xe2, 0x00, 0xfb, 0xe3, 0x05, 0x1f, 0x01,
/* Detail Timing Discriptions <offset 62 ~ > */
0x8c, 0x0a, 0xd0, 0x8a, 0x20, 0xe0, 0x2d, 0x10, 0x10, 0x3e, 0x96, 0x00, 0xc2, 0xad, 0x42, 0x00, 0x00, 0x18,
0x01, 0x1d, 0x80, 0x18, 0x71, 0x1c, 0x16, 0x20, 0x58, 0x2c, 0x25, 0x00, 0xc2, 0xad, 0x42, 0x00, 0x00, 0x9e,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x92
#endif


TEST(MhlTxEDID, edid_lib__remove_vic_except_for_1_3_4_16_32_34_from_edid_for_4k) {
	uint8_t ext_edid[EDID_BLOCK_SIZE];
	memcpy((void *)ext_edid, (const void *)ext_block_dummy_4k, EDID_BLOCK_SIZE);

	uint8_t support_vic[8] = {1, 3, 4, 16, 32, 34, 93, 95 };

	mhl_lib_edid_remove_hdmi_vic_and_3d_from_vsd(ext_edid, support_vic,sizeof(support_vic));

	LONGS_EQUAL(0,
				memcmp((const void *)ext_edid,
						(const void *)filtered_ext_block_dummy_4k,
				 EDID_BLOCK_SIZE));

}


TEST(MhlTxEDID, edid_lib__remove_hdmi_vic_and_3d_v2) {
	uint8_t vsd_body[21] = {
		0x74,
		0x03, 0x0c, 0x00, /* HDMI OUI , 1~3th byte*/\
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0x2f, /* 8th byte */
		0xd0,
		0x29, /* 10th byte */
		0x01,
		0x40,
		0xF9,/* 13th byte */
		0x7f,/* 14th byte */
		0x01, 0x02, 0x03, 0x80, 0x90, 0x76 /* 20th byte*/
	};
	const uint8_t expected_vsd_body[21] = {
		0x6b,
		0x03, 0x0c, 0x00, /* 1~3th byte  HDMI OUI */
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0x2f, /* 8th byte*/
		0x10,
		0x20, /* 10th byte */
		0x01,
		0x00,
		0x00,/* 13th byte */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00/* 20th byte*/
	};

	int size = 0;
	uint8_t vsd[21];
	uint8_t not_removed_vic[2] = {93, 95};

	memcpy((void *)vsd, (const void *)vsd_body, sizeof(vsd));

	mhl_lib_edid_replace_hdmi_vic_and_remove_3d(vsd, 20, not_removed_vic, sizeof(not_removed_vic));

	LONGS_EQUAL(0,
				memcmp((const void *)vsd,
						(const void *)expected_vsd_body,
						 21));

}


TEST(MhlTxEDID, squash) {
	uint8_t test_list[10] = {0x11, 0x22, 0x33, 0x00, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa};
	uint8_t expected[10] = {0x11, 0x22, 0x33, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0x00};
	int hit_cnt = 0;
	hit_cnt = squash_data_with_prune_0(test_list, sizeof(test_list));

	LONGS_EQUAL(0, memcmp((const void *)(test_list),
				(const void *)expected,
				10));

	LONGS_EQUAL(1, hit_cnt);

}

TEST(MhlTxEDID, squash_2) {
	uint8_t test_list[10] = {0x00, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa};
	uint8_t expected[10] = {0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0x00};
	int hit_cnt = 0;
	hit_cnt = squash_data_with_prune_0(test_list, sizeof(test_list));

	LONGS_EQUAL(0, memcmp((const void *)(test_list),
				(const void *)expected,
				10));

	LONGS_EQUAL(1, hit_cnt);

}


TEST(MhlTxEDID, squash_3) {
	uint8_t test_list[10] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00};
	uint8_t expected[10] =  {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00};
	int hit_cnt = 0;
	hit_cnt = squash_data_with_prune_0(test_list, sizeof(test_list));

	LONGS_EQUAL(0, memcmp((const void *)(test_list),
				(const void *)expected,
				10));

	LONGS_EQUAL(1, hit_cnt);

}

TEST(MhlTxEDID, squash_4_memory_break) {
	uint8_t test_list[11] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0xFF};
	uint8_t expected[11] =  {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0xFF};
	int hit_cnt = 0;

	hit_cnt = squash_data_with_prune_0(test_list, 10);

	LONGS_EQUAL(0, memcmp((const void *)(test_list),
				(const void *)expected,
				11));
	LONGS_EQUAL(1, hit_cnt);

	/* check memory integrity */
	LONGS_EQUAL(0xFF, test_list[10]);
	LONGS_EQUAL(0xFF, expected[10]);

}

TEST(MhlTxEDID, squash_several_0) {
	uint8_t test_list[10] = {0x11, 0x00, 0x33, 0x44, 0x00, 0x66, 0x77, 0x88, 0x99, 0xaa};
	uint8_t expected[10] =  {0x11, 0x33, 0x44, 0x66, 0x77, 0x88, 0x99, 0xaa, 0x00, 0x00};
	int hit_cnt = 0;

	hit_cnt = squash_data_with_prune_0(test_list, sizeof(test_list));

PRINT_DATA(test_list, 10)

	LONGS_EQUAL(0, memcmp((const void *)(test_list),
				(const void *)expected,
				10));

	LONGS_EQUAL(2, hit_cnt);

}

TEST(MhlTxEDID, squash_last_0) {
	uint8_t test_list[10] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x00, 0x77, 0x88, 0x99, 0x00};
	uint8_t expected[10] =  {0x11, 0x22, 0x33, 0x44, 0x55, 0x77, 0x88, 0x99, 0x00, 0x00};
	int hit_cnt = 0;

	hit_cnt = squash_data_with_prune_0(test_list, sizeof(test_list));

PRINT_DATA(test_list, 10)

	LONGS_EQUAL(0, memcmp((const void *)(test_list),
				(const void *)expected,
				10));

	LONGS_EQUAL(2, hit_cnt);

}


TEST(MhlTxEDID, squash_last_several_0) {
	uint8_t test_list[11] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x00, 0x00, 0xFF};
	uint8_t expected[11] =  {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x00, 0x00, 0xFF};
	int hit_cnt = 0;

	hit_cnt = squash_data_with_prune_0(test_list, 10);

PRINT_DATA(test_list, 10)

	LONGS_EQUAL(0, memcmp((const void *)(test_list),
				(const void *)expected,
				10));

	LONGS_EQUAL(2, hit_cnt);

	/* check memory break */
	LONGS_EQUAL(0xFF, test_list[10]);
	LONGS_EQUAL(0xFF, expected[10]);
}

TEST(MhlTxEDID, squash_last_several_v2) {
	uint8_t test_list[11] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x00, 0x00, 0x00, 0xFF};
	uint8_t expected[11] =  {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x00, 0x00, 0x00, 0xAA};
	int hit_cnt = 0;
	size_t len_10 = 10;

	hit_cnt = squash_data_with_prune_0(test_list, len_10);

	LONGS_EQUAL(0, memcmp((const void *)(test_list),
				(const void *)expected,
				len_10));

	LONGS_EQUAL(3, hit_cnt);

	/* check memory break */
	LONGS_EQUAL(0xFF, test_list[len_10]);
	LONGS_EQUAL(0xAA, expected[len_10]);
}


TEST(MhlTxEDID, squash_last_several_v3) {
	uint8_t test_list[11] = {0x00, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x00, 0x00, 0x00, 0xFF};
	uint8_t expected[11] =  {0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x00, 0x00, 0x00, 0x00, 0xAA};
	int hit_cnt = 0;
	size_t len_10 = 10;

	hit_cnt = squash_data_with_prune_0(test_list, len_10);

	LONGS_EQUAL(0, memcmp((const void *)(test_list),
				(const void *)expected,
				len_10));

	LONGS_EQUAL(4, hit_cnt);

	/* check memory break */
	LONGS_EQUAL(0xFF, test_list[len_10]);
	LONGS_EQUAL(0xAA, expected[len_10]);
}


TEST(MhlTxEDID, squash_several_0_v2) {
	uint8_t test_list[11] = {0x00, 0x22, 0x00, 0x44, 0x55, 0x66, 0x00, 0x88, 0x99, 0x00, 0xFF};
	uint8_t expected[11] =  {0x22, 0x44, 0x55, 0x66, 0x88, 0x99, 0x00, 0x00, 0x00, 0x00, 0xAA};
	int hit_cnt = 0;
	size_t len_10 = 10;

	hit_cnt = squash_data_with_prune_0(test_list, len_10);

	LONGS_EQUAL(0, memcmp((const void *)(test_list),
				(const void *)expected,
				len_10));

	LONGS_EQUAL(4, hit_cnt);

	/* check memory break */
	LONGS_EQUAL(0xFF, test_list[len_10]);
	LONGS_EQUAL(0xAA, expected[len_10]);
}


TEST(MhlTxEDID, short_squash_several_0_v2) {
	uint8_t test_list[11] = {0x01, 0x00, 0x02, 0x04, 0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA, 0x99};
	uint8_t expected[11] =  {0x01, 0x02, 0x04, 0x00, 0xAA, 0x99, 0x88, 0x77, 0x66, 0x55, 0x44};
	int hit_cnt = 0;
	size_t len_4 = 4;

	hit_cnt = squash_data_with_prune_0(test_list, len_4);

	LONGS_EQUAL(0, memcmp((const void *)(test_list),
				(const void *)expected,
				len_4));

	LONGS_EQUAL(1, hit_cnt);

	/* check memory break */
	LONGS_EQUAL(0xFF, test_list[len_4]);
	LONGS_EQUAL(0xEE, test_list[len_4 + 1]);
	LONGS_EQUAL(0xDD, test_list[len_4 + 2]);

	LONGS_EQUAL(0xAA, expected[len_4]);
	LONGS_EQUAL(0x99, expected[len_4 + 1]);
	LONGS_EQUAL(0x88, expected[len_4 + 2]);
}


TEST(MhlTxEDID, short_squash_several_0_v3) {
	uint8_t test_list[11] = {0x00, 0x01, 0x02, 0x04, 0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA, 0x99};
	uint8_t expected[11] =  {0x01, 0x02, 0x04, 0x00, 0xAA, 0x99, 0x88, 0x77, 0x66, 0x55, 0x44};
	int hit_cnt = 0;
	size_t len_4 = 4;

	hit_cnt = squash_data_with_prune_0(test_list, len_4);

	LONGS_EQUAL(0, memcmp((const void *)(test_list),
				(const void *)expected,
				len_4));

	LONGS_EQUAL(1, hit_cnt);

	/* check memory break */
	LONGS_EQUAL(0xFF, test_list[len_4]);
	LONGS_EQUAL(0xEE, test_list[len_4 + 1]);
	LONGS_EQUAL(0xDD, test_list[len_4 + 2]);

	LONGS_EQUAL(0xAA, expected[len_4]);
	LONGS_EQUAL(0x99, expected[len_4 + 1]);
	LONGS_EQUAL(0x88, expected[len_4 + 2]);
}

TEST(MhlTxEDID, short_squash_several_0_v4) {
	uint8_t test_list[11] = {0x00, 0x00, 0x00, 0x00, 0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA, 0x99};
	uint8_t expected[11] =  {0x00, 0x00, 0x00, 0x00, 0xAA, 0x99, 0x88, 0x77, 0x66, 0x55, 0x44};
	int hit_cnt = 0;
	size_t len_4 = 4;

	hit_cnt = squash_data_with_prune_0(test_list, len_4);

	LONGS_EQUAL(0, memcmp((const void *)(test_list),
				(const void *)expected,
				len_4));

	LONGS_EQUAL(4, hit_cnt);

	/* check memory break */
	LONGS_EQUAL(0xFF, test_list[len_4]);
	LONGS_EQUAL(0xEE, test_list[len_4 + 1]);
	LONGS_EQUAL(0xDD, test_list[len_4 + 2]);

	LONGS_EQUAL(0xAA, expected[len_4]);
	LONGS_EQUAL(0x99, expected[len_4 + 1]);
	LONGS_EQUAL(0x88, expected[len_4 + 2]);
}

TEST(MhlTxEDID, short_squash_several_0_v5) {
	uint8_t test_list[11] = {0x00, 0x01, 0x00, 0x00, 0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA, 0x99};
	uint8_t expected[11] =  {0x01, 0x00, 0x00, 0x00, 0xAA, 0x99, 0x88, 0x77, 0x66, 0x55, 0x44};
	int hit_cnt = 0;
	size_t len_4 = 4;

	hit_cnt = squash_data_with_prune_0(test_list, len_4);

	LONGS_EQUAL(0, memcmp((const void *)(test_list),
				(const void *)expected,
				len_4));

	LONGS_EQUAL(3, hit_cnt);

	/* check memory break */
	LONGS_EQUAL(0xFF, test_list[len_4]);
	LONGS_EQUAL(0xEE, test_list[len_4 + 1]);
	LONGS_EQUAL(0xDD, test_list[len_4 + 2]);

	LONGS_EQUAL(0xAA, expected[len_4]);
	LONGS_EQUAL(0x99, expected[len_4 + 1]);
	LONGS_EQUAL(0x88, expected[len_4 + 2]);
}



TEST(MhlTxEDID, ASUS_PROBLEM) {
	uint8_t ext_edid[EDID_BLOCK_SIZE];

	memcpy((void *)ext_edid, (const void *)Asus_VX239_EDID_EXT_BLOCK, EDID_BLOCK_SIZE);

PRINT_DATA(ext_edid, 128)

	//input edid
	mhl_edid_parser_remove_vic16_1080p60fps(ext_edid);

PRINT_DATA(ext_edid, 128)

	//check the vic is removed from edid or not
	LONGS_EQUAL(0, memcmp((const void *)(Asus_VX239_EDID_EXT_BLOCK_Remove_vic_16_from_svd),
				(const void *)ext_edid,
				EDID_BLOCK_SIZE));

}


TEST(MhlTxEDID, edid_lib__remove_hdmi_vic_and_3d_v2_less_than_8) {
	uint8_t vsd_body[21] = {
		0x74,
		0x03, 0x0c, 0x00, /* HDMI OUI , 1~3th byte*/\
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0x0f,/* 8th byte (no hdmi video present) */
		0xd0,
		0x0a, /* 10th byte */
		0x01,
		0x40,
		0xF8,/* 13th byte */
		0x7f,/* 14th byte */
		0x01, 0x02, 0x03, 0x80, 0x90, 0x76 /* 20th byte*/
	};
	int size = 0;
	uint8_t vsd[21];
	uint8_t not_removed_vic[2] = {93, 95};
	memcpy((void *)vsd, (const void *)vsd_body, sizeof(vsd));

	mhl_lib_edid_replace_hdmi_vic_and_remove_3d(vsd, 7, not_removed_vic, sizeof(not_removed_vic));

	LONGS_EQUAL(0,
				memcmp((const void *)vsd,
						(const void *)vsd_body,
						 20));
}

TEST(MhlTxEDID, edid_lib__remove_hdmi_vic_and_3d_v2_no_hdmi_video_present) {
	uint8_t vsd_body[21] = {
		0x74,
		0x03, 0x0c, 0x00, /* HDMI OUI , 1~3th byte*/\
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0x0f,/* 8th byte (no hdmi video present) */
		0xd0,
		0x0a, /* 10th byte */
		0x01,
		0x40,
		0xF8,/* 13th byte */
		0x7f,/* 14th byte */
		0x01, 0x02, 0x03, 0x80, 0x90, 0x76 /* 20th byte*/
	};
	int size = 0;
	uint8_t vsd[21];
	uint8_t not_removed_vic[2] = {93, 95};

	memcpy((void *)vsd, (const void *)vsd_body, sizeof(vsd));
	mhl_lib_edid_replace_hdmi_vic_and_remove_3d(vsd, 20, not_removed_vic, sizeof(not_removed_vic));

	LONGS_EQUAL(0,
				memcmp((const void *)vsd,
						(const void *)vsd_body,
						 sizeof(vsd_body)));
}



//---- fail!!
TEST(MhlTxEDID, edid_lib__remove_hdmi_vic_and_3d_v3) {
	uint8_t vsd_body[21] = {
		0x74,
		0x03, 0x0c, 0x00, /* HDMI OUI , 1~3th byte*/\
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0x2f,
		0xd0,
		0x67, /* HDMI VIC LEN : 3   10th byte */
		0x02,
		0x01,
		0x03,/* 13th byte */
		0x7f,/* 14th byte */
		0x02, 0x01, 0x03, 0x80, 0x90, 0x76 /* 20th byte*/
	};
	const uint8_t expected_vsd_body[21] = {
		0x6c,
		0x03, 0x0c, 0x00, /* 1~3th byte  HDMI OUI */
		0x20,
		0x00,
		0xb8, /* 6th byte */
		0x2d,
		0x2f, /* 8th byte*/
		0x10,
		0x40, /* HDMI VIC LEN : 2  10th byte */
		0x01,
		0x03,
		0x00,/* 13th byte */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00/* 20th byte*/
	};

	int size = 0;
	uint8_t vsd[21];
	uint8_t not_removed_vic[2] = {93/* hdmi vic 3 */, 95/* hdmi vic 1 */};

	memcpy((void *)vsd, (const void *)vsd_body, sizeof(vsd));

	mhl_lib_edid_replace_hdmi_vic_and_remove_3d(vsd, 20, not_removed_vic, sizeof(not_removed_vic));

	LONGS_EQUAL(0,
				memcmp((const void *)vsd,
						(const void *)expected_vsd_body,
						 20));

}

/*
 * DVI test
 */
#define EDID_IIYAMA_E1706S_DVI_BLOCK_0 \
0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x26, 0xCD, 0xE9, 0x46, 0x7E, 0x06, 0x00, 0x00,\
0x31, 0x15, 0x01, 0x03, 0x80, 0x22, 0x1B, 0x78, 0x2A, 0xF7, 0xC1, 0xA3, 0x55, 0x4C, 0x9B, 0x25,\
0x12, 0x50, 0x54, 0xBF, 0xEF, 0x00, 0x71, 0x4F, 0x81, 0x40, 0x81, 0x80, 0x01, 0x01, 0x01, 0x01,\
0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x30, 0x2A, 0x00, 0x98, 0x51, 0x00, 0x2A, 0x40, 0x30, 0x70,\
0x13, 0x00, 0x51, 0x0E, 0x11, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x31, 0x31, 0x30,\
0x31, 0x39, 0x43, 0x31, 0x43, 0x30, 0x31, 0x36, 0x36, 0x32, 0x00, 0x00, 0x00, 0xFD, 0x00, 0x37,\
0x4C, 0x1E, 0x53, 0x0E, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFC,\
0x00, 0x50, 0x4C, 0x31, 0x37, 0x30, 0x36, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x73,\

static uint8_t edid_iiyama_e1706s_dvi [EDID_BLOCK_SIZE] = {
	EDID_IIYAMA_E1706S_DVI_BLOCK_0
};

TEST(MhlTxEDID, edid_parser__dvi_when_cea_extention_number_is_1) {
	const uint8_t *dvi_edid_1_block = edid_iiyama_e1706s_dvi;

	mhl_lib_edid_set_edid(dvi_edid_1_block);
	LONGS_EQUAL(false, mhl_lib_edid_is_hdmi());
}

TEST(MhlTxEDID, edid_parser__not_dvi_without_initialize_lib_even_if_cea_extention_number_is_1) {
	const uint8_t *dvi_edid_1_block = edid_iiyama_e1706s_dvi;

	/* mhl_lib_edid_set_edid(dvi_edid_1_block); */
	LONGS_EQUAL(false, mhl_lib_edid_is_hdmi());
}


/* this block must be used by only ieee extraction test  */
uint8_t edid_no_ieee_oui[256] = {
0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x44, 0x89, 0x72, 0x03, 0x05, 0x00, 0x00, 0x00,
0x2D, 0x0E, 0x01, 0x03, 0x80, 0x50, 0x2D, 0x78, 0x0A, 0x0D, 0xC9, 0xA0, 0x57, 0x47, 0x98, 0x27,
0x12, 0x48, 0x4C, 0x20, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xD6, 0x09, 0x80, 0xA0, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x60,
0xA2, 0x00, 0x80, 0xE0, 0x21, 0x00, 0x00, 0x18, 0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10,
0x10, 0x3E, 0x96, 0x00, 0x13, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x48,
0x44, 0x4D, 0x49, 0x20, 0x54, 0x56, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFD,
0x00, 0x3B, 0x3D, 0x0F, 0x2E, 0x08, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x6F,
0x02, 0x01, 0x04, 0x00, 0x8C, 0x0A, 0xD0, 0x90, 0x20, 0x40, 0x31, 0x20, 0x0C, 0x40, 0x55, 0x00,
0x13, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x3E,
0x96, 0x00, 0xC4, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x8C, 0x0A, 0xD0, 0x90, 0x20, 0x40, 0x31, 0x20,
0x0C, 0x40, 0x55, 0x00, 0xC4, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68};

/* this block must be used by only ieee extraction test  */
uint8_t edid_no_ieee_oui_4_block_not_include_ieee[512] = {
0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x44, 0x89, 0x72, 0x03, 0x05, 0x00, 0x00, 0x00,
0x2D, 0x0E, 0x01, 0x03, 0x80, 0x50, 0x2D, 0x78, 0x0A, 0x0D, 0xC9, 0xA0, 0x57, 0x47, 0x98, 0x27,
0x12, 0x48, 0x4C, 0x20, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xD6, 0x09, 0x80, 0xA0, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x60,
0xA2, 0x00, 0x80, 0xE0, 0x21, 0x00, 0x00, 0x18, 0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10,
0x10, 0x3E, 0x96, 0x00, 0x13, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x48,
0x44, 0x4D, 0x49, 0x20, 0x54, 0x56, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFD,
0x00, 0x3B, 0x3D, 0x0F, 0x2E, 0x08, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x03, 0x6D,

0xF0, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C,

0x02, 0x01, 0x04, 0x00, 0x8C, 0x0A, 0xD0, 0x90, 0x20, 0x40, 0x31, 0x20, 0x0C, 0x40, 0x55, 0x00,
0x13, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x3E,
0x96, 0x00, 0xC4, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x8C, 0x0A, 0xD0, 0x90, 0x20, 0x40, 0x31, 0x20,
0x0C, 0x40, 0x55, 0x00, 0xC4, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68,

0x02, 0x01, 0x04, 0x00, 0x8C, 0x0A, 0xD0, 0x90, 0x20, 0x40, 0x31, 0x20, 0x0C, 0x40, 0x55, 0x00,
0x13, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x3E,
0x96, 0x00, 0xC4, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x8C, 0x0A, 0xD0, 0x90, 0x20, 0x40, 0x31, 0x20,
0x0C, 0x40, 0x55, 0x00, 0xC4, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68};


/* this block must be used by only ieee extraction test  */
uint8_t edid_no_ieee_oui_4_block_including_ieee[512] = {
0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x44, 0x89, 0x72, 0x03, 0x05, 0x00, 0x00, 0x00,
0x2D, 0x0E, 0x01, 0x03, 0x80, 0x50, 0x2D, 0x78, 0x0A, 0x0D, 0xC9, 0xA0, 0x57, 0x47, 0x98, 0x27,
0x12, 0x48, 0x4C, 0x20, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xD6, 0x09, 0x80, 0xA0, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x60,
0xA2, 0x00, 0x80, 0xE0, 0x21, 0x00, 0x00, 0x18, 0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10,
0x10, 0x3E, 0x96, 0x00, 0x13, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x48,
0x44, 0x4D, 0x49, 0x20, 0x54, 0x56, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFD,
0x00, 0x3B, 0x3D, 0x0F, 0x2E, 0x08, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x03, 0x6D,

0xF0, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C,

0x02, 0x01, 0x04, 0x00, 0x8C, 0x0A, 0xD0, 0x90, 0x20, 0x40, 0x31, 0x20, 0x0C, 0x40, 0x55, 0x00,
0x13, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x3E,
0x96, 0x00, 0xC4, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x8C, 0x0A, 0xD0, 0x90, 0x20, 0x40, 0x31, 0x20,
0x0C, 0x40, 0x55, 0x00, 0xC4, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68,

EDID_SONY_KD_65X9200A_BLOCK_EXT_1};




//1 vendor specific data block & block0 & 1
TEST(MhlTxEDID, edid_parser__dvi_when_there_is_not_ieee_oui) {
	const uint8_t *dvi_edid_block = edid_no_ieee_oui;
	pr_info("--------\n"); 
	mhl_lib_edid_set_edid(dvi_edid_block);
	LONGS_EQUAL(false, mhl_lib_edid_is_hdmi());
	pr_info("--------\n"); 
}

//1 vendor specific data block & block0, 1, 2
TEST(MhlTxEDID, edid_parser__dvi_when_there_is_not_ieee_oui_4_block_including_ieee) {
	const uint8_t *dvi_edid_block = edid_no_ieee_oui_4_block_including_ieee;

	mhl_lib_edid_set_edid(dvi_edid_block);
	LONGS_EQUAL(true, mhl_lib_edid_is_hdmi());

}

//1 vendor specific data block & block0, 1, 2, 3
TEST(MhlTxEDID, edid_parser__dvi_when_there_is_not_ieee_oui_4_block_not_including_ieee) {
	const uint8_t *dvi_edid_block = edid_no_ieee_oui_4_block_not_include_ieee;

	mhl_lib_edid_set_edid(dvi_edid_block);
	LONGS_EQUAL(false, mhl_lib_edid_is_hdmi());
/* pr_info("--------\n"); */
}


#define EDID_SONY_KD_65X9200A_BLOCK_EXT_1_2VSD \
0x02, 0x03, \
0x3e, /* offset d is 62 */\
0xf0, \
/* video data block */ \
0x4c, \
0x10, 0x05, 0x04, 0x20, 0x22, 0x3c, 0x3e, 0x03, 0x07, 0x02, 0x06, 0x01, /* <offset 16> */\
/* */ \
0x29, 0x09, 0x07, 0x07, 0x15, 0x07, 0x50, 0x35, 0x05, 0x50, 0x83, 0x01, 0x00, 0x00, /* <offset 30> */\
/* vendor specific data 1 */ \
0x65, \
0x04, 0x0c, 0x00, /* 1 ~ 3 byte */\
0x20, 0x00, /* phisical address */ \
/* vendor specific data 2 */ \
0x77, /* length : 23*/\
0x03, 0x0c, 0x00, /* 1 ~ 3 byte */\
0x20, 0x00, 0xb8, 0x3c, 0x2f, /* hdmi video present */\
/* there is no latency info */\
0xd0, /* 3D_present 1, 3D_Multi_present 10, Image_Size 10, Rsvd 0, Rsvd 0, Rsvd 0 */\
0x89, /* HDMI VIC LEN : 4 / HDMI 3D LEN : 9 */\
0x01, 0x02, 0x03, 0x04, /* hdmi vic */ \
0x01, 0x40, 0x00, 0x0f, 0x10, 0x40, 0x50, 0x60, 0x46, /* 3d */\
/* Colorimetry Data Block  */\
0xe2, 0x00, 0xfb, 0xe3, 0x05, 0x1f, 0x01, \
/* Detail Timing Discriptions <offset 62 ~ > */ \
0x8c, 0x0a, 0xd0, 0x8a, 0x20, 0xe0, 0x2d, 0x10, 0x10, 0x3e, 0x96, 0x00, 0xc2, 0xad, 0x42, 0x00, 0x00, 0x18,\
0x01, 0x1d, 0x80, 0x18, 0x71, 0x1c, 0x16, 0x20, 0x58, 0x2c, 0x25, 0x00, 0xc2, 0xad, 0x42, 0x00, 0x00, 0x9e,\
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5a\


uint8_t edid_no_ieee_oui_2_block_including_ieee_in_several_vsd[EDID_BLOCK_SIZE*2] = {
0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x44, 0x89, 0x72, 0x03, 0x05, 0x00, 0x00, 0x00,
0x2D, 0x0E, 0x01, 0x03, 0x80, 0x50, 0x2D, 0x78, 0x0A, 0x0D, 0xC9, 0xA0, 0x57, 0x47, 0x98, 0x27,
0x12, 0x48, 0x4C, 0x20, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xD6, 0x09, 0x80, 0xA0, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x60,
0xA2, 0x00, 0x80, 0xE0, 0x21, 0x00, 0x00, 0x18, 0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10,
0x10, 0x3E, 0x96, 0x00, 0x13, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x48,
0x44, 0x4D, 0x49, 0x20, 0x54, 0x56, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFD,
0x00, 0x3B, 0x3D, 0x0F, 0x2E, 0x08, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x6F,

EDID_SONY_KD_65X9200A_BLOCK_EXT_1_2VSD
};


uint8_t edid_no_ieee_oui_block_including_ieee_in_several_vsd[EDID_BLOCK_SIZE] = {
EDID_SONY_KD_65X9200A_BLOCK_EXT_1_2VSD
};


//2 vendor specific data block
TEST(MhlTxEDID, edid_parser__hdmi_when_ieee_is_in_2nd_vsd) {
	const uint8_t *ext_block = edid_no_ieee_oui_block_including_ieee_in_several_vsd;
	LONGS_EQUAL(true, mhl_lib_is_ieee_reg_id(ext_block));
/* pr_info("--------\n"); */
}

TEST(MhlTxEDID, edid_no_ieee_oui_2_block_including_ieee_in_several_vsd) {

	const uint8_t *dvi_edid_block = edid_no_ieee_oui_2_block_including_ieee_in_several_vsd;
	mhl_lib_edid_set_edid(dvi_edid_block);

	LONGS_EQUAL(true, mhl_lib_edid_is_hdmi());

/* pr_info("--------\n"); */
}

//2 vendor specific data block
uint8_t edid_no_ieee_oui_4_blocks_including_ieee_in_several_vsd[EDID_BLOCK_SIZE*4] = {

0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x44, 0x89, 0x72, 0x03, 0x05, 0x00, 0x00, 0x00,
0x2D, 0x0E, 0x01, 0x03, 0x80, 0x50, 0x2D, 0x78, 0x0A, 0x0D, 0xC9, 0xA0, 0x57, 0x47, 0x98, 0x27,
0x12, 0x48, 0x4C, 0x20, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xD6, 0x09, 0x80, 0xA0, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x60,
0xA2, 0x00, 0x80, 0xE0, 0x21, 0x00, 0x00, 0x18, 0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10,
0x10, 0x3E, 0x96, 0x00, 0x13, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x48,
0x44, 0x4D, 0x49, 0x20, 0x54, 0x56, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFD,
0x00, 0x3B, 0x3D, 0x0F, 0x2E, 0x08, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x03, 0x6D,

0xF0, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C,

EDID_SONY_KD_65X9200A_BLOCK_EXT_1_2VSD,

0x02, 0x01, 0x04, 0x00, 0x8C, 0x0A, 0xD0, 0x90, 0x20, 0x40, 0x31, 0x20, 0x0C, 0x40, 0x55, 0x00,
0x13, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x3E,
0x96, 0x00, 0xC4, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x8C, 0x0A, 0xD0, 0x90, 0x20, 0x40, 0x31, 0x20,
0x0C, 0x40, 0x55, 0x00, 0xC4, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68

};
TEST(MhlTxEDID, edid_no_ieee_oui_4_block_including_ieee_in_several_vsd) {

	const uint8_t *dvi_edid_block = edid_no_ieee_oui_4_blocks_including_ieee_in_several_vsd;
	mhl_lib_edid_set_edid(dvi_edid_block);

	LONGS_EQUAL(true, mhl_lib_edid_is_hdmi());



/* pr_info("--------\n"); */
}


static uint8_t pull_up_list[EDID_BLOCK_SIZE] = {

0x02, 0x03, 0x29, 0x41, 0x83, 0x4F, 0x00, 0x00, 0x67, 0x03, 0x0C, 0x00, 0x10, 0x00, 0x00, 0x21,
0x32, 0x0F, 0x7F, 0x07, 0x17, 0x7F, 0xFF, 0x3F, 0x7F, 0xFF, 0x57, 0x7F, 0x00, 0x5F, 0x7F, 0x01,
0x67, 0x7F, 0x00, 0xE2, 0x00, 0x0F, 0x42, 0x82, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAC };

static uint8_t expect_pull_up_list[EDID_BLOCK_SIZE] = {
0x02, 0x03, 0x28, 0x41, 0x83, 0x4F, 0x00, 0x00, 0x67, 0x03, 0x0C, 0x00, 0x10, 0x00, 0x00, 0x21,
0x32, 0x0F, 0x7F, 0x07, 0x17, 0x7F, 0xFF, 0x3F, 0x7F, 0xFF, 0x57, 0x7F, 0x00, 0x5F, 0x7F, 0x01,
0x67, 0x7F, 0x00, 0xE2, 0x00, 0x0F, 0x41, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30 };


TEST(MhlTxEDID, edid_lib__pull_up_cts3_2_2_x) {
	uint8_t ext_edid[EDID_BLOCK_SIZE];
	memcpy((void *)ext_edid, (const void *)pull_up_list, EDID_BLOCK_SIZE);

	uint8_t not_removed_vic[6] = {
		0x01,
		0x03,
		0x04,
		0x10,
		0x20,
		0x22
	};

#if 0
PRINT_DATA(ext_edid, EDID_BLOCK_SIZE)
#endif

	mhl_lib_edid_remove_vic_from_svd(ext_edid, not_removed_vic,sizeof(not_removed_vic));

#if 1
PRINT_DATA(ext_edid, EDID_BLOCK_SIZE)
#endif

	LONGS_EQUAL(0,
				memcmp((const void *)ext_edid,
						(const void *)expect_pull_up_list,
				 EDID_BLOCK_SIZE));

}

//Only Latency_Fields_Present contains 1.
//Latnecy fields (2 bytes) exist.

//I_Latency_Fields_Present is 0 -> Latency must be 0.
//I_Latency_Fields_Present is 1 -> 4 bytes exist.






TEST(MhlTxEDID, no_action_when_supported_preferred_disp_info_in_edid_block_0) {
	uint8_t *edid_block = edid_block_0_DEL_1908;
	uint8_t copied_block_0[EDID_BLOCK_SIZE];
	uint8_t index = 0x36;

	/*mhl_lib_edid_replace_dtd_preferred_disp_info(copied_block_0);*/
	memcpy((void *)copied_block_0, (const void *)edid_block_0_DEL_1908, sizeof(copied_block_0));


	/* HDMI_VFRMT_1280x1024p60_5_4 is supposed to be found */
	LONGS_EQUAL(true,
			mhl_lib_edid_is_supp_disp_info_in_one_dtd_blk(
					(const uint8_t *)(copied_block_0 + index),
					(const hdmi_edid_video_mode_property_type *)preferred_disp_info_pattern_2,
					sizeof(preferred_disp_info_pattern_2)/sizeof(*preferred_disp_info_pattern_2),
					NULL));
	index += 0x12;

	/* 2nd block is not for disp info. false is supposed to be */
	LONGS_EQUAL(false,
			mhl_lib_edid_is_supp_disp_info_in_one_dtd_blk(
					(const uint8_t *)(copied_block_0 + index),
					(const hdmi_edid_video_mode_property_type *)preferred_disp_info_pattern_2,
					sizeof(preferred_disp_info_pattern_2)/sizeof(*preferred_disp_info_pattern_2),
					NULL));
	index += 0x12;

	/* 3rd block is not for disp info. false is supposed to be */
	LONGS_EQUAL(false,
			mhl_lib_edid_is_supp_disp_info_in_one_dtd_blk(
					(const uint8_t *)(copied_block_0 + index),
					(const hdmi_edid_video_mode_property_type *)preferred_disp_info_pattern_2,
					sizeof(preferred_disp_info_pattern_2)/sizeof(*preferred_disp_info_pattern_2),
					NULL));
	index += 0x12;

	/* 4th block is not for disp info. false is supposed to be */
	LONGS_EQUAL(false,
			mhl_lib_edid_is_supp_disp_info_in_one_dtd_blk(
					(const uint8_t *)(copied_block_0 + index),
					(const hdmi_edid_video_mode_property_type *)preferred_disp_info_pattern_2,
					sizeof(preferred_disp_info_pattern_2)/sizeof(*preferred_disp_info_pattern_2),
					NULL));

}



TEST(MhlTxEDID, replace_one_preferred_disp_info) {
	uint8_t *edid_block = edid_block_0_DEL_1908;
	uint8_t copied_block_0[EDID_BLOCK_SIZE];

	memcpy((void *)copied_block_0, (const void *)edid_block_0_DEL_1908, sizeof(copied_block_0));

	mhl_lib_edid_replace_dtd_preferred_disp_info(copied_block_0, &vga_preferred_disp_info);

	/* pattern_1 input should result in true */

	LONGS_EQUAL(true,
			mhl_lib_edid_is_supp_disp_info_in_one_dtd_blk(
					(const uint8_t *)(copied_block_0 + 0x36),
					(const hdmi_edid_video_mode_property_type *)preferred_disp_info_pattern_1,
					sizeof(preferred_disp_info_pattern_1)/sizeof(*preferred_disp_info_pattern_1),
					NULL));

}

TEST(MhlTxEDID, remove_display_info_and_set_it_as_DummyDescriptorDefinition) {

	uint8_t *edid_block = edid_block_0_DEL_1908;
	uint8_t copied_block_0[EDID_BLOCK_SIZE];

	memcpy((void *)copied_block_0, (const void *)edid_block_0_DEL_1908, sizeof(copied_block_0));

	LONGS_EQUAL(false,
			mhl_lib_edid_is_supp_disp_info_in_one_dtd_blk(
						(const uint8_t *)(copied_block_0 + 0x36),
						(const hdmi_edid_video_mode_property_type *)preferred_disp_info_pattern_1,
						sizeof(preferred_disp_info_pattern_1)/sizeof(*preferred_disp_info_pattern_1),
						NULL));

}


TEST(MhlTxEDID, replace_unsupport_disp_info_to_vga) {

	uint8_t copied_block_0[EDID_BLOCK_SIZE];

	memcpy((void *)copied_block_0, (const void *)edid_block_0_DEL_1908, sizeof(copied_block_0));

	PRINT_DATA(copied_block_0, EDID_BLOCK_SIZE)

	LONGS_EQUAL(false,
			mhl_lib_edid_is_supp_disp_info_in_one_dtd_blk(
							(const uint8_t *)(copied_block_0 + 0x36),
							(const hdmi_edid_video_mode_property_type *)preferred_disp_info_pattern_3,
							sizeof(preferred_disp_info_pattern_3)/sizeof(*preferred_disp_info_pattern_3),
							NULL));

	mhl_lib_edid_replace_unsupport_descriptor_with_dummy(
			copied_block_0,
			(const hdmi_edid_video_mode_property_type *)preferred_disp_info_pattern_3,
			sizeof(preferred_disp_info_pattern_3)/sizeof(*preferred_disp_info_pattern_3));


	PRINT_DATA(copied_block_0, EDID_BLOCK_SIZE)


	LONGS_EQUAL(true,
			mhl_lib_edid_is_supp_disp_info_in_one_dtd_blk(
							(const uint8_t *)(copied_block_0 + 0x36),
							(const hdmi_edid_video_mode_property_type *)preferred_disp_info_pattern_3,
							sizeof(preferred_disp_info_pattern_3)/sizeof(*preferred_disp_info_pattern_3),
							NULL));

}

TEST(MhlTxEDID, replace_a_descriptor_with_dummy) {
	uint8_t edid_block_0_DEL_1908_test_for_dtd_dummy_block_replacement[EDID_BLOCK_SIZE] = {
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x10, 0xAC, 0x26, 0x40, 0x32, 0x30, 0x38, 0x41,
	0x17, 0x11, 0x01, 0x03, 0x80, 0x26, 0x1E, 0x78, 0xEE, 0xDE, 0x95, 0xA3, 0x54, 0x4C, 0x99, 0x26,
	0x0F, 0x50, 0x54, 0xA5, 0x4B, 0x00, 0x71, 0x4F, 0x81, 0x80, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x30, 0x2A, 0x00, 0x98, 0x51, 0x00, 0x2A, 0x40, 0x30, 0x70,
	0x13, 0x00, 0x78, 0x2D, 0x11, 0x00, 0x00, 0x1E, /*0x00*/0x05, 0x00, 0x00, 0xFF, 0x00, 0x4B, 0x55, 0x37,
	0x39, 0x30, 0x37, 0x36, 0x37, 0x41, 0x38, 0x30, 0x32, 0x0A, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x44,
	0x45, 0x4C, 0x4C, 0x20, 0x31, 0x39, 0x30, 0x38, 0x46, 0x50, 0x0A, 0x20, 0x00, 0x00, 0x00, 0xFD,
	0x00, 0x38, 0x4C, 0x1E, 0x51, 0x0E, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x3B
	};

	uint8_t expected_2nd_description_in_dtd[18] = {

	/* 2nd description in dtd */
	0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00,	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	};

	uint8_t *edid_block = edid_block_0_DEL_1908_test_for_dtd_dummy_block_replacement;
	uint8_t copied_block_0[EDID_BLOCK_SIZE];

	memcpy((void *)copied_block_0,
			(const void *)edid_block_0_DEL_1908_test_for_dtd_dummy_block_replacement,
			sizeof(edid_block_0_DEL_1908_test_for_dtd_dummy_block_replacement));

	PRINT_DATA(copied_block_0, EDID_BLOCK_SIZE)

	mhl_lib_edid_replace_unsupport_descriptor_with_dummy(
				copied_block_0,
				(const hdmi_edid_video_mode_property_type *)preferred_disp_info_pattern_3,
				sizeof(preferred_disp_info_pattern_3)/sizeof(*preferred_disp_info_pattern_3));

	PRINT_DATA(copied_block_0, EDID_BLOCK_SIZE)

	LONGS_EQUAL(0, memcmp((const void *)(copied_block_0 + 0x36 + 18),
				(const void *)expected_2nd_description_in_dtd,
				18));

}

TEST(MhlTxEDID, replace_a_descriptor_with_dummy__only_18_byte_is_replaced) {
	uint8_t sample_block_0[EDID_BLOCK_SIZE] = {
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x10, 0xAC, 0x26, 0x40, 0x32, 0x30, 0x38, 0x41,
	0x17, 0x11, 0x01, 0x03, 0x80, 0x26, 0x1E, 0x78, 0xEE, 0xDE, 0x95, 0xA3, 0x54, 0x4C, 0x99, 0x26,
	0x0F, 0x50, 0x54, 0xA5, 0x4B, 0x00, 0x71, 0x4F, 0x81, 0x80, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x30, 0x2A, 0x00, 0x98, 0x51, 0x00, 0x2A, 0x40, 0x30, 0x70,
	0x13, 0x00, 0x78, 0x2D, 0x11, 0x00, 0x00, 0x1E, /*0x00*/0x05, 0x00, 0x00, 0xFF, 0x00, 0x4B, 0x55, 0x37,
	0x39, 0x30, 0x37, 0x36, 0x37, 0x41, 0x38, 0x30, 0x32, 0x0A, 0x70, 0x62, 0x80, 0xa0, 0x20, 0xe0, 0x2d, 0x10, 0x30, 0x70,
	0x13, 0x00, 0x78, 0x2d, 0x11, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0xFD,
	0x00, 0x38, 0x4C, 0x1E, 0x51, 0x0E, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x3B
	};

	uint8_t expected_2nd_description_in_dtd[18] = {

	/* 2nd description in dtd */
	0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00,	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	};

	uint8_t copied_block_0[EDID_BLOCK_SIZE];

	memcpy((void *)copied_block_0,
			(const void *)sample_block_0,
			sizeof(sample_block_0));

	PRINT_DATA(copied_block_0, EDID_BLOCK_SIZE)

	mhl_lib_edid_replace_unsupport_descriptor_with_dummy(
				copied_block_0,
				(const hdmi_edid_video_mode_property_type *)preferred_disp_info_pattern_3,
				sizeof(preferred_disp_info_pattern_3)/sizeof(*preferred_disp_info_pattern_3));

	PRINT_DATA(copied_block_0, EDID_BLOCK_SIZE)

	{
		/* check after dummy block is not changed */
		uint8_t offset_to_3rd_block = 0x36 + 18 + 18;
		LONGS_EQUAL(0, memcmp((const void *)(copied_block_0 + offset_to_3rd_block),
					(const void *)(sample_block_0 + offset_to_3rd_block),
					18));
	}

}
#endif // MHL_DEVICE_TEST_ALL

TEST(MhlTxEDID, DEL_1908_unsupported_prefferred_disp_is_replaced_with_vga) {
	uint8_t copied_block_0[EDID_BLOCK_SIZE];
	uint8_t index = 0x36;

	/*mhl_lib_edid_replace_dtd_preferred_disp_info(copied_block_0);*/
	memcpy((void *)copied_block_0, (const void *)edid_block_0_DEL_1908, sizeof(copied_block_0));

	PRINT_DATA(copied_block_0, EDID_BLOCK_SIZE)

	/* HDMI_VFRMT_1280x1024p60_5_4 is NOT supposed to be found */
	LONGS_EQUAL(false,
			mhl_lib_edid_is_supp_disp_info_in_one_dtd_blk(
					(const uint8_t *)(copied_block_0 + index),
					preferred_disp_info,
					sizeof(preferred_disp_info)/sizeof(*preferred_disp_info),
					NULL));


	mhl_lib_edid_replace_unsupport_descriptor_with_dummy(
					copied_block_0,
					preferred_disp_info,
					sizeof(preferred_disp_info)/sizeof(*preferred_disp_info));

	PRINT_DATA(copied_block_0, EDID_BLOCK_SIZE)

	LONGS_EQUAL(true,
				mhl_lib_edid_is_supp_disp_info_in_one_dtd_blk(
						(const uint8_t *)(copied_block_0 + index),
						preferred_disp_info,
						sizeof(preferred_disp_info)/sizeof(*preferred_disp_info),
						NULL));

	{	/* aspect ratio must be 4:3 */
		uint8_t *data_buf = &copied_block_0[0x36];
		uint32_t img_size_h = ((((u32)data_buf[0xE] >> 0x4) & 0xF) << 8) | data_buf[0xC];
		uint32_t img_size_v = (((u32)data_buf[0xE] & 0xF) << 8) | data_buf[0xD];
		bool aspect_ratio_4_3 = (abs((int)img_size_h * 3 - (int)img_size_v * 4) < 5) ? 1 : 0;

		LONGS_EQUAL(true, aspect_ratio_4_3);

	}
}

