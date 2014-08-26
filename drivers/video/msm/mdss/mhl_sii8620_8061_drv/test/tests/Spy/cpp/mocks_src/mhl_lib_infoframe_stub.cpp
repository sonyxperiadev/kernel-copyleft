
extern "C" {
#include "mhl_lib_infoframe.h"
}



int is_valid_vsif(vendor_specific_info_frame_t *vsif)
{
	return 0;
}

int is_valid_avif(avi_info_frame_t *avif)
{
	return 0;
}

void print_vic_modes(uint8_t vic)
{

}

uint32_t find_pixel_clock_from_AVI_VIC(uint8_t vic)
{
	return 0;
}

uint32_t find_pixel_clock_from_HDMI_VIC(uint8_t vic)
{

}

uint8_t hdmi_vic_to_mhl3_vic(uint8_t vic)
{

}

uint32_t find_pixel_clock_from_totals(uint16_t h_total, uint16_t v_total)
{
	return 0;
}

uint8_t qualify_pixel_clock_for_mhl(uint32_t pixel_clock_frequency,
				    uint8_t bits_per_pixel,
				    bool is_mode_mhl3,
				    bool is_peer_support_ppixel)
{
	return 0;
}

uint8_t calculate_avi_info_frame_checksum(hw_avi_payload_t *payload)
{
	return 0;
}