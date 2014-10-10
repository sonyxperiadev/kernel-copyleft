#include "CppUTestExt/MockSupport_c.h"

#include "util.h"
#include "linux/gpio.h"
#include "GpioSpy.h"
#include "linux/dummy_util.h"
#include "linux/of_gpio.h"

#define GPIO_LIST_SIZE 100
int gpios[GPIO_LIST_SIZE];

void gpio_spy_clean(void)
{
	common_spy_DisableMockMode();
	unsigned i = 0;
	for (i = 0; i<sizeof(gpios); i++)
		gpios[i]=0;
}

void gpio_set_value(unsigned gpio, int value)
{
	if (!common_spy_get_MockMode()) {
		if (gpio >= GPIO_LIST_SIZE)
			printf("invalid value!!");
		gpios[gpio] = value;
	} else {
		mock_scope_c("GPIO")->
		actualCall("gpio_set_value")->
		withIntParameters("gpio", (int)gpio)->
		withIntParameters("value", (int)value);
	}
}

int gpio_get_value(unsigned gpio)
{
	if (!common_spy_get_MockMode()) {
		if (gpio >= GPIO_LIST_SIZE)
			return -1;
		return gpios[gpio];
	} else {
		int res = mock_scope_c("GPIO")
			->actualCall("gpio_get_value")
			->withIntParameters("gpio", (int)gpio)
			->returnValue().value.intValue;
	}
}

int of_get_named_gpio(struct device_node *np,const char *propname, int index)
{
	return 0;
}
