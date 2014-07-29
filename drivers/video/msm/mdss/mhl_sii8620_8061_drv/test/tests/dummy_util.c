
#include <linux/dummy_util.h>

void kfree(const void *objp)
{
	free((void*)objp);
}

void * kzalloc(size_t size, gfp_t flags)
{
	return (void*)malloc(size);
}


bool gpio_is_valid(int number)
{
	return true;
}


int gpio_request(unsigned int gpio, const char *tag)
{
	return 0;
}

int gpio_direction_output(unsigned gpio, int level)
{
	return 0;
}

int gpio_direction_input(unsigned gpio)
{
	return 0;
}

void gpio_free(unsigned gpio)
{

}

int gpio_to_irq(int gpio)
{
	return 50;
}
#if 0
int gpio_get_value(unsigned int gpio)
{
	return 1;
}

#endif
/*
void msleep(int ms)
{

}
*/


#if 0

#endif

/*
void mhl_pf_i2c_init(struct i2c_adapter *adapter)
{

}

void i2c_set_clientdata(struct i2c_client *dev, void *data)
{

}
*/


int mhl_tx_initialize(void)
{

}


/*
void mhl_device_initialize(struct device *dev)
{

}
void mhl_device_release(struct device *dev)
{

}
*/
void mhl_tx_release(void)
{

}












