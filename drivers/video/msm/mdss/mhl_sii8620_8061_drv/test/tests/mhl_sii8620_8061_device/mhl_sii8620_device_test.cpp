#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

extern "C" {
#include "MockRegAccess.h"
#include "si_8620_regs.h"

extern void mhl_sii8620_clear_and_mask_interrupts(bool is_exclude_rgnd);
extern int int_4_isr(uint8_t int_4_status);
void mhl_device_isr (void);

}

TEST_GROUP(MhlDevice) {
	void setup() {
		mock("IO").strictOrder();
		MOCK_mhl_pf_i2c_print(false);
	}
	void teardown() {
		mock("IO").checkExpectations();
		mock().clear();
	}

	void setup_clear_and_mask_mock(bool is_exclude_rgnd)
	{
		uint8_t bit_disc_int_reg = 0xFF;
		uint8_t bit_disc_int_mask_reg = 0x00;

		if (is_exclude_rgnd == true) {
			bit_disc_int_reg = (uint8_t)~BIT_PAGE_5_CBUS_DISC_INTR0_CBUS_DISC_INTR0_STAT6;
			bit_disc_int_mask_reg = BIT_PAGE_5_CBUS_DISC_INTR0_MASK_CBUS_DISC_INTR0_MASK6;
		}

	/*** DISC ***/
	/* status (Clear statis with 1 except for RGND (6th bit)) */
	mock("IO")
		.expectOneCall("mhl_pf_write_reg")
		.withParameter("page", TX_PAGE_5)
		.withParameter("offset", 0xED)
		.withParameter("value", bit_disc_int_reg);
	/* mask (disable mask with 0, only RGND is enabled) */
	mock("IO")
		.expectOneCall("mhl_pf_write_reg")
		.withParameter("page", TX_PAGE_5)
		.withParameter("offset", 0xEE)
		.withParameter("value", bit_disc_int_mask_reg);

	/*** G2WB ***/
	/* status (Clear statis with 1) */
	mock("IO")
		.expectOneCall("mhl_pf_write_reg")
		.withParameter("page", TX_PAGE_5)
		.withParameter("offset", 0x8C)
		.withParameter("value", 0xFF);
	/* mask (disable mask with 0) */
	mock("IO")
		.expectOneCall("mhl_pf_write_reg")
		.withParameter("page", TX_PAGE_5)
		.withParameter("offset", 0x8D)
		.withParameter("value", 0x00);

	/*** MSC ***/
	/* status (Clear statis with 1) */
	mock("IO")
		.expectOneCall("mhl_pf_write_reg")
		.withParameter("page", TX_PAGE_5)
		.withParameter("offset", 0x92)
		.withParameter("value", 0xFF);
	/* mask (disable mask with 0) */
	mock("IO")
		.expectOneCall("mhl_pf_write_reg")
		.withParameter("page", TX_PAGE_5)
		.withParameter("offset", 0x93)
		.withParameter("value", 0x00);

	/*** MERR ***/
	/* status (Clear statis with 1) */
	mock("IO")
		.expectOneCall("mhl_pf_write_reg")
		.withParameter("page", TX_PAGE_5)
		.withParameter("offset", 0x94)
		.withParameter("value", 0xFF);
	/* mask (disable mask with 0) */
	mock("IO")
		.expectOneCall("mhl_pf_write_reg")
		.withParameter("page", TX_PAGE_5)
		.withParameter("offset", 0x95)
		.withParameter("value", 0x00);

	/*** INFR ***/
	/* status (Clear statis with 1) */
	mock("IO")
		.expectOneCall("mhl_pf_write_reg")
		.withParameter("page", TX_PAGE_0)
		.withParameter("offset", 0x7C)
		.withParameter("value", 0xFF);
	/* mask (disable mask with 0) */
	mock("IO")
		.expectOneCall("mhl_pf_write_reg")
		.withParameter("page", TX_PAGE_0)
		.withParameter("offset", 0x7E)
		.withParameter("value", 0x00);

	/*** HDCP ***/
	/* status (Clear statis with 1) */
	mock("IO")
		.expectOneCall("mhl_pf_write_reg")
		.withParameter("page", TX_PAGE_6)
		.withParameter("offset", 0x3D)
		.withParameter("value", 0xFF);
	/* mask (disable mask with 0) */
	mock("IO")
		.expectOneCall("mhl_pf_write_reg")
		.withParameter("page", TX_PAGE_6)
		.withParameter("offset", 0x3C)
		.withParameter("value", 0x00);

	/*** HDCP2 ***/
	/* status (Clear statis with 1) */
	mock("IO")
		.expectOneCall("mhl_pf_write_reg")
		.withParameter("page", TX_PAGE_3)
		.withParameter("offset", 0x98)
		.withParameter("value", 0xFF);
	/* mask (disable mask with 0) */
	mock("IO")
		.expectOneCall("mhl_pf_write_reg")
		.withParameter("page", TX_PAGE_3)
		.withParameter("offset", 0x99)
		.withParameter("value", 0x00);

	/*** EDID ***/
	/* status (Clear statis with 1) */
	mock("IO")
		.expectOneCall("mhl_pf_write_reg")
		.withParameter("page", TX_PAGE_2)
		.withParameter("offset", 0xE0)
		.withParameter("value", 0xFF);
	/* mask (disable mask with 0) */
	mock("IO")
		.expectOneCall("mhl_pf_write_reg")
		.withParameter("page", TX_PAGE_2)
		.withParameter("offset", 0xE1)
		.withParameter("value", 0x00);

	/*** SCDT ***/
	/* status (Clear statis with 1) */
	mock("IO")
		.expectOneCall("mhl_pf_write_reg")
		.withParameter("page", TX_PAGE_0)
		.withParameter("offset", 0x74)
		.withParameter("value", 0xFF);
	/* mask (disable mask with 0) */
	mock("IO")
		.expectOneCall("mhl_pf_write_reg")
		.withParameter("page", TX_PAGE_0)
		.withParameter("offset", 0x78)
		.withParameter("value", 0x00);

	/*** COC ***/
	/* status (Clear statis with 1) */
	mock("IO")
		.expectOneCall("mhl_pf_write_reg")
		.withParameter("page", TX_PAGE_7)
		.withParameter("offset", 0x26)
		.withParameter("value", 0xFF);
	/* mask (disable mask with 0) */
	mock("IO")
		.expectOneCall("mhl_pf_write_reg")
		.withParameter("page", TX_PAGE_7)
		.withParameter("offset", 0x27)
		.withParameter("value", 0x00);

	/*** TDM ***/
	/* status (Clear statis with 1) */
	mock("IO")
		.expectOneCall("mhl_pf_write_reg")
		.withParameter("page", TX_PAGE_1)
		.withParameter("offset", 0x64)
		.withParameter("value", 0xFF);
	/* mask (disable mask with 0) */
	mock("IO")
		.expectOneCall("mhl_pf_write_reg")
		.withParameter("page", TX_PAGE_1)
		.withParameter("offset", 0x66)
		.withParameter("value", 0x00);

	/*** TIMR ***/
	/* status (Clear statis with 1) */
	mock("IO")
		.expectOneCall("mhl_pf_write_reg")
		.withParameter("page", TX_PAGE_0)
		.withParameter("offset", 0x71)
		.withParameter("value", 0xFF);
	/* mask (disable mask with 0) */
	mock("IO")
		.expectOneCall("mhl_pf_write_reg")
		.withParameter("page", TX_PAGE_0)
		.withParameter("offset", 0x75)
		.withParameter("value", 0x00);
	}
};

/*
TEST(MhlDevice, clear_and_mask_all_interrupts) {
	bool is_exclude_rgnd = false;

	setup_clear_and_mask_mock(is_exclude_rgnd);

	mhl_sii8620_clear_and_mask_interrupts(is_exclude_rgnd);
}

TEST(MhlDevice, clear_and_mask_all_interrupts_except_for_rgnd) {
	bool is_exclude_rgnd = true;

	setup_clear_and_mask_mock(is_exclude_rgnd);

	mhl_sii8620_clear_and_mask_interrupts(is_exclude_rgnd);

}
*/

