

#include "CppUTestExt/MockSupport.h"

extern "C"{
#include "mhl_sii8620_8061_device.h"
#include "MockRegAccess.h"
#include "string.h"
}

bool mhl_sii8620_dev_cbus_is_support_ppixel(void)
{
	return (bool)mock("CBUS_CONTROL")
	.actualCall("mhl_sii8620_dev_cbus_is_support_ppixel")
	.returnValue().getIntValue();
}
