#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

extern "C" {
#include "util.h"
#include "mhl_cbus_control.h"
#include "sii8620_dev_edid_spy.h"

extern uint8_t *cbus_wb_get_stored_scrpad(uint8_t *size);
extern uint8_t cbus_wb_get_vic(uint8_t *in_hev_vic, uint8_t *out_hev_vic);

extern uint8_t calculate_generic_checksum(
	uint8_t *info_frame_data,
	uint8_t checksum,
	uint8_t length);
extern void cbus_wb_handle_scr_data (uint8_t *scrpad);
extern uint8_t *get_stored_support_hev_vic(void);
}

TEST_GROUP(MhlCBUS) {
	void setup() {
		mock().ignoreOtherCalls();
		mock("CBUS_CONTROL").strictOrder();
		mock("IO").strictOrder();
		mock("DEV_EDID").strictOrder();

		mhl_sii8620_device_edid_init();
	}
	void teardown() {
		mock("CBUS_CONTROL").checkExpectations();
		mock("IO").checkExpectations();
		mock("DEV_EDID").checkExpectations();
		mock().clear();
	}

	void scratpad_reg_setup(uint8_t *in_scrpad)
	{
		mock("IO").expectOneCall("mhl_pf_read_reg_block")
			.withParameter("page",(int)TX_PAGE_4)
			.withParameter("offset",(int)0x40)
                        .withParameter("count",(int)16)
			.andReturnValue(in_scrpad);
	}
};

uint8_t video_3d_data_1[]	= { 0x00, 0x10, 0xD8, 0x10, 0x03, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t video_3d_data_2[]	= { 0x00, 0x10, 0xDB, 0x10, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t hev_vic_1[]		= { 0x00, 0x20, 0x56, 0x08, 0x01, 0x05, 0x00, 0x5D, 0x00, 0x5E, 0x00, 0x5F, 0x00, 0x62, 0x00, 0x00};
uint8_t hev_vic_2[]		= { 0x00, 0x20, 0xD3, 0x08, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t hev_dtda[]		= { 0x00, 0x21, 0xDD, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t hev_dtdb[]		= { 0x00, 0x22, 0xDC, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t adt_burst_id[]		= { 0x00, 0x41, 0xBB, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t emsc_support[]		= { 0x00, 0x61, 0x6F, 0x03, 0x01, 0x03, 0x00, 0x63, 0x00, 0x64, 0x00, 0x62, 0x00, 0x00, 0x00, 0x00};

uint8_t hev_vic_1_bad_num_ent[] = {
	0x00, 0x20, /* burst id */
	0xf8, /* check sum*/
	0x05, /* tot ent */
	0x01, /* seq */
	0x06, /* num ent */
	0x00, 0x5D, 0x00, 0x5E, 0x00, 0x5F, 0x00, 0x62, 0x00, 0x60 /* data */
};

uint8_t hev_vic_1_chksum_fail[] = {
	0x00, 0x20, /* burst id */
	0x56, /* check sum*/
	0x05, /* tot ent */
	0x01, /* seq */
	0x05, /* num ent */
	0x00, 0x5D, 0x00, 0x5E, 0x00, 0x5F, 0x00, 0x62, 0x00, 0x00 /* data */
};

/*FIXME
{
	uint8_t i;
	uint8_t length_ = 16;
	uint8_t checksum = 0;

	for (i = 0; i < length_; i++){
		pr_info("---------------0x%x\n",checksum);
		checksum += hev_vic_1_bad_num_ent[i];
	}

	checksum = 0x100 - checksum;

}
*/

#define ALL_TEST
#ifdef ALL_TEST

TEST(MhlCBUS, send_GNT_with_Request_Write_event) {
	uint8_t intr_0;

	//setup
	mock("CBUS_CONTROL").expectOneCall("set_cbus_command")
			.withParameter("command",(int)MHL_SET_INT)
			.withParameter("reg",(int)MHL_RCHANGE_INT)
                        .withParameter("reg_data",(int)MHL_INT_GRT_WRT)
			.andReturnValue(1);

	intr_0 = MHL_INT_REQ_WRT;
	cbus_wb_event_handler(intr_0);
}

TEST(MhlCBUS, Read_video_3d_data_1_from_scr_reg_with_DSCR_CHG) {
	uint8_t intr_0;
	uint8_t scrpad_size = 0;
	uint8_t *scrpad = NULL;
	uint8_t SCR_SIZE = 16;

	//setup
	mock("IO").expectOneCall("mhl_pf_read_reg_block")
			.withParameter("page",(int)TX_PAGE_4)
			.withParameter("offset",(int)0x40)
                        .withParameter("count",(int)16)
			.andReturnValue(video_3d_data_1);

	intr_0 =  MHL_INT_DSCR_CHG;
	cbus_wb_event_handler(intr_0);

	scrpad = cbus_wb_get_stored_scrpad(&scrpad_size);

	LONGS_EQUAL(SCR_SIZE, scrpad_size);
	LONGS_EQUAL(0, memcmp((const void *)scrpad, (const void *)(video_3d_data_1), SCR_SIZE));
}

TEST(MhlCBUS, get_hev_vic) {
	uint8_t vics[10];
	uint8_t length;
	uint8_t answer[4] = {93, 94, 95, 98};
	uint8_t answer_length = 4;

	length = cbus_wb_get_vic(hev_vic_1, vics);

	LONGS_EQUAL(answer_length, length);
	LONGS_EQUAL(0, memcmp((const void *)answer, (const void *)(vics), answer_length));
}

TEST(MhlCBUS, get_hev_vic_bad_num_ent) {
	uint8_t vics[10];
	uint8_t length;
	uint8_t answer[5] = {93, 94, 95, 98, 96};
	uint8_t answer_length = 5;

	length = cbus_wb_get_vic(hev_vic_1_bad_num_ent, vics);

	LONGS_EQUAL(answer_length, length);
	LONGS_EQUAL(0, memcmp((const void *)answer, (const void *)(vics), answer_length));
}

TEST(MhlCBUS, nothing_todo_when_check_sum_fails) {
	uint8_t vics[10] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t length;
	uint8_t answer[10] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t answer_length = 0;

	length = cbus_wb_get_vic(hev_vic_1_chksum_fail, vics);

	LONGS_EQUAL(answer_length, length);
	LONGS_EQUAL(0, memcmp((const void *)answer, (const void *)(vics), 10));
}

TEST(MhlCBUS, set_vics_when_burst_id_is_hev) {
	uint8_t * result;
	uint8_t answer[4] = {93, 94, 95, 98};

	cbus_wb_handle_scr_data(hev_vic_1);

        result = get_stored_support_hev_vic();

	LONGS_EQUAL(0, memcmp((const void *)answer, (const void *)(result), 4));
}

TEST(MhlCBUS, read_1_block) {
	uint8_t intr_0;
	uint8_t * result;
	uint8_t answer[4] = {93, 94, 95, 98};

	/*
	 * setup
	 */
	/* Step1. When receveis request write, then returns GRT WRITE */
	mock("CBUS_CONTROL").expectOneCall("set_cbus_command")
			.withParameter("command",(int)MHL_SET_INT)
			.withParameter("reg",(int)MHL_RCHANGE_INT)
                        .withParameter("reg_data",(int)MHL_INT_GRT_WRT)
			.andReturnValue(1);
	intr_0 = MHL_INT_REQ_WRT;
	cbus_wb_event_handler(intr_0);

	/* Step2. Recevies DSCR_CHG */
	intr_0 =  MHL_INT_DSCR_CHG;
	mock("IO").expectOneCall("mhl_pf_read_reg_block")
			.withParameter("page",(int)TX_PAGE_4)
			.withParameter("offset",(int)0x40)
                        .withParameter("count",(int)16)
			.andReturnValue(hev_vic_1);
	/*
	 * execute
	 */
	cbus_wb_event_handler(intr_0);

	/*
	 * verify
	 */
        result = get_stored_support_hev_vic();
	LONGS_EQUAL(0, memcmp((const void *)answer, (const void *)(result), 4));

}

TEST(MhlCBUS, read_8_block) {
	uint8_t intr_0;
	uint8_t * result;
	uint8_t answer[4] = {93, 94, 95, 98};
	uint8_t answer2[20] = {93, 94, 95, 98, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	/*
	 * setup
	 */
	/* Step1. When receveis request write, then returns GRT WRITE */
	mock("CBUS_CONTROL").expectOneCall("set_cbus_command")
			.withParameter("command",(int)MHL_SET_INT)
			.withParameter("reg",(int)MHL_RCHANGE_INT)
                        .withParameter("reg_data",(int)MHL_INT_GRT_WRT)
			.andReturnValue(1);
	intr_0 = MHL_INT_REQ_WRT;
	cbus_wb_event_handler(intr_0);

	/* Step2. Recevies DSCR_CHG */
	intr_0 =  MHL_INT_DSCR_CHG;
	scratpad_reg_setup(video_3d_data_1);
	scratpad_reg_setup(video_3d_data_2);
	scratpad_reg_setup(hev_vic_1);
	scratpad_reg_setup(hev_vic_2);
	scratpad_reg_setup(hev_dtda);
	scratpad_reg_setup(hev_dtdb);
	scratpad_reg_setup(adt_burst_id);
	scratpad_reg_setup(emsc_support);

	/*
	 * execute
	 */
	cbus_wb_event_handler(intr_0);
	cbus_wb_event_handler(intr_0);
	cbus_wb_event_handler(intr_0);
	cbus_wb_event_handler(intr_0);
	cbus_wb_event_handler(intr_0);
	cbus_wb_event_handler(intr_0);
	cbus_wb_event_handler(intr_0);
	cbus_wb_event_handler(intr_0);

	/*
	 * verify
	 */
        result = get_stored_support_hev_vic();
	LONGS_EQUAL(0, memcmp((const void *)answer, (const void *)(result), 4));
	LONGS_EQUAL(0, memcmp((const void *)answer2, (const void *)(result), 20));

}

#endif // ALL_TEST
TEST(MhlCBUS, read_8_block_with_feature_complete) {
	uint8_t intr_0;
	uint8_t * result;
	uint8_t answer[4] = {93, 94, 95, 98};
	uint8_t answer2[20] = {93, 94, 95, 98, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	/*
	 * setup
	 */
	/* Step1. When receveis request write, then returns GRT WRITE */
	mock("CBUS_CONTROL").expectOneCall("set_cbus_command")
			.withParameter("command",(int)MHL_SET_INT)
			.withParameter("reg",(int)MHL_RCHANGE_INT)
                        .withParameter("reg_data",(int)MHL_INT_GRT_WRT)
			.andReturnValue(1);

	/* Step2. Recevies DSCR_CHG */
	scratpad_reg_setup(video_3d_data_1);
	scratpad_reg_setup(video_3d_data_2);
	scratpad_reg_setup(hev_vic_1);
	scratpad_reg_setup(hev_vic_2);
	scratpad_reg_setup(hev_dtda);
	scratpad_reg_setup(hev_dtdb);
	scratpad_reg_setup(adt_burst_id);
	scratpad_reg_setup(emsc_support);

	mock("DEV_EDID").expectOneCall("mhl_device_edid_set_upstream_edid");

	/*
	 * execute
	 */
	/* req write */
	intr_0 = MHL_INT_REQ_WRT;
	cbus_wb_event_handler(intr_0);

	/* scrpad */
	intr_0 =  MHL_INT_DSCR_CHG;
	cbus_wb_event_handler(intr_0);
	cbus_wb_event_handler(intr_0);
	cbus_wb_event_handler(intr_0);
	cbus_wb_event_handler(intr_0);
	cbus_wb_event_handler(intr_0);
	cbus_wb_event_handler(intr_0);
	cbus_wb_event_handler(intr_0);

	/* complete */
	intr_0 =  MHL_INT_DSCR_CHG | MHL3_INT_FEAT_COMPLETE;
	cbus_wb_event_handler(intr_0);

	/*
	 * verify
	 */
        result = get_stored_support_hev_vic();
	LONGS_EQUAL(0, memcmp((const void *)answer, (const void *)(result), 4));
	LONGS_EQUAL(0, memcmp((const void *)answer2, (const void *)(result), 20));

}
