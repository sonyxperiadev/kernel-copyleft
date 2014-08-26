#include "CppUTestExt/MockSupport.h"

extern "C" {
#include "util.h"
#include "sii8620_dev_edid_spy.h"

static uint8_t hev_vics[100];
static int sup_vic_len;
uint8_t *get_stored_support_hev_vic(void)
{
	return hev_vics;
}

}


void mhl_device_edid_set_upstream_edid(void)
{
	mock("DEV_EDID")
		.actualCall("mhl_device_edid_set_upstream_edid");
}

void mhl_device_edid_setup_sink_support_hev_vic(uint8_t *vics, uint8_t length)
{
	int in_index = 0;
	int cur_len = sup_vic_len;

	for (; sup_vic_len < cur_len + length; sup_vic_len++) {
		hev_vics[sup_vic_len] = *(vics + in_index++);
	}
}

void dev_edid_spy_clean(void)
{
	common_spy_DisableMockMode();
}

int mhl_sii8620_device_edid_read_request(uint8_t block_number)
{
	int res = 0;

	if (!common_spy_get_MockMode()) {
		/* spy */
	} else {
		/* mock */
		res = mock("DEV_EDID")
			.actualCall("mhl_sii8620_device_edid_read_request")
			.withParameter("block_number", (u8)block_number)
			.returnValue().getIntValue();
	}

	return res;
}

void mhl_sii8620_device_edid_init(void)
{
	memset(hev_vics, 0x00, 100);
	sup_vic_len = 0;

	if (!common_spy_get_MockMode()) {
		/* spy */
	} else {
		/* mock */
		mock("DEV_EDID")
			.actualCall("mhl_sii8620_device_edid_init");
	}
}

int drive_hpd_low(void)
{
	if (!common_spy_get_MockMode()) {
		/* spy */
	} else {
		/* mock */
		mock("DEV_HPD")
			.actualCall("drive_hpd_low");
	}
}
