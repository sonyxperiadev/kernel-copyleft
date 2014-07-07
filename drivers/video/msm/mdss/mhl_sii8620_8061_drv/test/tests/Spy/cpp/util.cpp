#include "CppUTestExt/MockSupport.h"

extern "C" {
#include "util.h"
#include "linux/delay.h"
}

static bool isMockMode;

void common_spy_MockMode() {
	isMockMode = true;
}

void common_spy_DisableMockMode() {
	isMockMode = false;
}

bool common_spy_get_MockMode() {
	return isMockMode;
}

void common_spy_clean(void) {
	isMockMode = false;
}

void msleep(int ms)
{
	if (!isMockMode) {

	} else {
		mock("DELAY")
			.actualCall("msleep")
			.withParameter("ms", (u8)ms);
	}
}

int i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	int res = 1;
	if (!isMockMode) {

	} else {
		res = mock("I2C")
			.actualCall("i2c_transfer")
			.returnValue().getIntValue();
	}

	return res;
}

void init_timer(struct timer_list *timer)
{

}

int del_timer(struct timer_list *timer)
{

}

int mod_timer(struct timer_list *timer, unsigned long expires)
{

	return 0;
}
