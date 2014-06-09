extern "C" {
#include "mhl_platform.h"
}

int mhl_pf_switch_to_usb(void)
{
	return 0;
}

int mhl_pf_switch_to_mhl(void)
{
	return 0;
}

void mhl_pf_switch_register_cb(int (*device_discovery)(void *context),
								void *context)
{

}

void mhl_pf_switch_unregister_cb(void)
{

}