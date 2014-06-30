#include "CppUTestExt/MockSupport.h"

extern "C" {
#include "mhl_platform.h"
}


void mhl_pf_chip_power_on(void)
{

}

void mhl_pf_chip_power_off(void)
{

}

bool mhl_pf_is_chip_power_on(void)
{
	int res =  mock("MHL_PLATFORM_CHIP")
	.actualCall("mhl_pf_is_chip_power_on")
	.returnValue().getIntValue();

	if (res == 1)
		return true;
	else
		return false;
}


hpd_control_mode platform_get_hpd_control_mode(void)
{
	return HPD_CTRL_PUSH_PULL;
}
