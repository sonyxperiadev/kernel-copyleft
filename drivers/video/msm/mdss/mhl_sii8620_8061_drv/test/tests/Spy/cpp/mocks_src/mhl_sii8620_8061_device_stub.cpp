
extern "C" {
#include "mhl_sii8620_8061_device.h"
}

CBUS_MODE_TYPE cbus_mode;
int cbusp_processing_cond;

CBUS_MODE_TYPE mhl_device_get_cbus_mode(void) {
	return cbus_mode;
}

void mhl_dev_set_cbusp_cond_processing(MHL_DEV_CBUS_P cbusp) {
	return;
}

int mhl_device_get_cbusp_cond(void) {
	return cbusp_processing_cond;
}

bool mhl_device_is_cbusb_executing(void) {
	if (cbusp_processing_cond) {
		return true;
	} else {
		return false;
	}
}

void mhl_dev_clear_cbusp_cond_processing(MHL_DEV_CBUS_P cbusp) {
	cbusp_processing_cond &= (int)~cbusp;
}

bool mhl_dev_is_set_hpd(void) {
	return true;
}

int stop_video(void) {
	return 0;
}

int drive_hpd_low(void) {
	return 0;
}

void enable_heartbeat(void) {
	return;
}


bool mhl_device_is_peer_device_mhl1_2(void) {
	bool ret = false;

		switch (cbus_mode) {
		case CBUS_oCBUS_PEER_IS_MHL1_2:
			ret = true;
			break;
		case CBUS_NO_CONNECTION:
		case CBUS_oCBUS_PEER_VERSION_PENDING:
		case CBUS_oCBUS_PEER_IS_MHL3:
		case CBUS_bCBUS:
		case CBUS_TRANSITIONAL_TO_eCBUS_S:
		case CBUS_TRANSITIONAL_TO_eCBUS_S_CALIBRATED:
		case CBUS_TRANSITIONAL_TO_eCBUS_D:
		case CBUS_TRANSITIONAL_TO_eCBUS_D_CALIBRATED:
		case CBUS_eCBUS_S:
		case CBUS_eCBUS_D:
		default:
			break;
		}

		return ret;
}

bool mhl_device_is_peer_device_mhl3(void) {
	bool ret = false;

		switch (cbus_mode) {
		case CBUS_oCBUS_PEER_IS_MHL3:
		case CBUS_bCBUS:
		case CBUS_TRANSITIONAL_TO_eCBUS_S:
		case CBUS_TRANSITIONAL_TO_eCBUS_S_CALIBRATED:
		case CBUS_TRANSITIONAL_TO_eCBUS_D:
		case CBUS_TRANSITIONAL_TO_eCBUS_D_CALIBRATED:
		case CBUS_eCBUS_S:
		case CBUS_eCBUS_D:
			ret = true;
			break;
		case CBUS_NO_CONNECTION:
		case CBUS_oCBUS_PEER_VERSION_PENDING:
		case CBUS_oCBUS_PEER_IS_MHL1_2:
		default:
			break;
		}

		return ret;
}

int drive_hpd_high(void) {
	return 0;
}
