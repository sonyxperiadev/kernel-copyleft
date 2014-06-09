#include "CppUTestExt/MockSupport.h"

extern "C" {
#include "mhl_cbus_control.h"
#include "mhl_sii8620_8061_device.h"
}

void mhl_msc_cbus_communication_start(void)
{

}

void mhl_msc_hpd_receive(void)
{

}
void mhl_msc_command_done(bool is_Msc_Msg_Ack)
{

}

void mhl_msc_write_stat_receive(void)
{

}
void mhl_msc_msg_receive(void)
{

}
void mhl_msc_set_int_receive(void)
{

}
void mhl_upstream_tmds_change(void)
{

}
void mhl_upstream_infoframe_change(uint8_t int_8_status)
{

}
void mhl_hdcp_status_change(uint8_t tpi_int_status)
{

}

void exe_cbus_command(void)
{

}

void init_devcap(void)
{

}

void mhl_msc_init()
{
	mock("CBUS_CONTROL")
		.actualCall("mhl_msc_init");
}


int mhl_cbus_control_initialize(void)
{
	return 0;
}

void mhl_cbus_control_release(void)
{

}

void mhl_cbus_abort(uint8_t cbus_err_int)
{

}

bool set_cbus_command(uint8_t command, uint8_t reg, uint8_t reg_data)
{
	bool res = (bool)mock("CBUS_CONTROL")
		.actualCall("set_cbus_command")
		.withParameter("command", (u8)command)
		.withParameter("reg", (u8)reg)
		.withParameter("reg_data", (u16)reg_data)
                .returnValue().getIntValue();

	return res;
}

void mhl_cbus_communication_eCBUS_start(void)
{
	return;
}

uint8_t mhl_get_tdm_virt_chan_slot_counts(enum tdm_vc_assignments vc)
{
	return 0;
}

bool mhl_cbus_packed_pixel_available(void)
{
	bool res = (bool)mock("CBUS_CONTROL")
		.actualCall("packed_pixel_available")
                .returnValue().getIntValue();
	return res;
}

void set_cond_in_cbus_control(CBUS_CONTROL_COND cond)
{
	return;
}

void clear_cond_in_cbus_control(CBUS_CONTROL_COND cond)
{
	return;
}

bool get_cond_in_cbus_control(CBUS_CONTROL_COND cond)
{
	return true;
}

void send_link_mode_to_sink(void)
{
	return;
}


void set_link_mode(u8 link_mode)
{

}

u8 get_link_mode(void)
{
	return 0;
}

bool mhl_cbus_is_sink_support_ppixel(void)
{
	return true;
}


bool mhl_cbus_16bpp_available(void)
{
	return true;
}
