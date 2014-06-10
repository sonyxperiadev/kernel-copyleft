#include <linux/module.h>
#include <linux/clk.h>
#include <linux/of_platform.h>
#include <linux/err.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include "mhl_platform.h"
#include "mhl_sii8620_8061_device.h"
#include "mhl_common.h"
#include "si_8620_regs.h"
#include "mhl_tx.h"
#include "mhl_lib_timer.h"

#define MHL_DRIVER_NAME "sii8620"
#define COMPATIBLE_NAME "qcom,mhl-sii8620"

struct clk *mhl_clk;

static enum {
	CHIP_PWR_ON,
	CHIP_PWR_OFF
} chip_pwr_state;

/*gpio*/
static int int_gpio;
static int pwr_gpio;
static int rst_gpio;
static int switch_sel_1_gpio;
static int switch_sel_2_gpio;
static int fw_wake_gpio;

/*irq*/
static int irq_number;

/*device name*/
static const char *device_name;

/*device*/
static struct device *pdev;

static int mhl_pf_clock_enable(void);
static void mhl_pf_clock_disable(void);

const char *mhl_pf_get_device_name(void)
{
	return device_name;
}

int mhl_pf_get_irq_number(void)
{
	pr_debug("%s:irq_number:%d\n", __func__, irq_number);
	return irq_number;
}


/*
 * Return a value indicating how upstream HPD is
 * implemented on this platform.
 */
hpd_control_mode platform_get_hpd_control_mode(void)
{
	return HPD_CTRL_PUSH_PULL;
}

enum gpio_direction_types {
	GPIO_OUTPUT,
	GPIO_INPUT
};

static int mhl_set_gpio(const char *gpio_name, int gpio_number,
		enum gpio_direction_types direction, int out_val)
{
	int ret = -EBUSY;
	pr_debug("%s()\n", __func__);
	pr_debug("%s:%s=[%d]\n", __func__, gpio_name, gpio_number);

	if (gpio_number < 0)
		return -EINVAL;


	if (gpio_is_valid(gpio_number)) {
		ret = gpio_request((unsigned int)gpio_number, gpio_name);
		if (ret < 0) {
			pr_err("%s:%s=[%d] req failed:%d\n",
				__func__, gpio_name, gpio_number, ret);
			return -EBUSY;
		}
		if (direction == GPIO_OUTPUT) {
			pr_debug("%s:gpio output\n", __func__);
			ret = gpio_direction_output(
				(unsigned int)gpio_number, out_val);
		} else if (direction == GPIO_INPUT) {
			pr_debug("%s:gpio input\n", __func__);
			ret = gpio_direction_input((unsigned int)gpio_number);
		} else {
			pr_err("%s:%s=[%d] invalid direction type :%d\n",
				__func__, gpio_name, gpio_number, ret);
			return -EINVAL;
		}
		if (ret < 0) {
			pr_err("%s: set dirn %s failed: %d\n",
				__func__, gpio_name, ret);
			return -EBUSY;
		}
	}

	return 0;
}

void mhl_pf_gpio_config_release(void)
{
	if (int_gpio >= 0)
		gpio_free((unsigned int)int_gpio);
	if (pwr_gpio >= 0)
		gpio_free((unsigned int)pwr_gpio);
	if (rst_gpio >= 0)
		gpio_free((unsigned int)rst_gpio);
	if (switch_sel_1_gpio >= 0)
		gpio_free((unsigned int)switch_sel_1_gpio);
	if (switch_sel_2_gpio >= 0)
		gpio_free((unsigned int)switch_sel_2_gpio);
	if (fw_wake_gpio >= 0)
		gpio_free((unsigned int)fw_wake_gpio);
}

static int mhl_pf_gpio_config_init(void)
{
	int res = -1;
	pr_debug("%s()\n", __func__);

	/* reset */
	res = mhl_set_gpio("mhl-rst-gpio", rst_gpio, GPIO_OUTPUT, 0);
	if (res)
		goto error;

	/* interrupt */
	res = mhl_set_gpio("mhl-intr-gpio", int_gpio, GPIO_INPUT, 0);
	if (res)
		goto error;
	irq_number = gpio_to_irq(int_gpio);
	pr_debug("%s:irq_number:%d\n", __func__, irq_number);

	/* fw wake */
	res = mhl_set_gpio("mhl-fw-wake-gpio", fw_wake_gpio, GPIO_OUTPUT, 0);
	if (res)
		goto error;

	/* power */
	res = mhl_set_gpio("mhl-pwr-gpio", pwr_gpio, GPIO_OUTPUT, 0);
	if (res)
		goto error;

	/* switch sel 1 */
	res = mhl_set_gpio("mhl-switch-sel-1-gpio",
						switch_sel_1_gpio,
						GPIO_OUTPUT, 0);
	if (res)
		goto error;

	/* switch sel 2 */
	res = mhl_set_gpio("mhl-switch-sel-2-gpio",
			switch_sel_2_gpio, GPIO_OUTPUT, 0);
	if (res)
		goto error;

	return 0;

error:
	mhl_pf_gpio_config_release();
	return res;
}

bool mhl_pf_is_chip_power_on(void)
{
	if (chip_pwr_state == CHIP_PWR_ON)
		return true;
	else
		return false;
}

void mhl_pf_chip_power_on(void)
{
	pr_debug("%s()\n", __func__);

	/* the state is referred by i2c function.
	 * So the state must be changed at first.
	 * If it is PWR_OFF, all i2c access fails.
	 */
	chip_pwr_state = CHIP_PWR_ON;

	/* enable the clock supply */
	mhl_pf_clock_enable();

	/* 20 msec is enough to make the clock stable
	 * udelay might be better for less than 10 msec,
	 * however it blocks process, so avoiding it.
	 * (250 micro sec wait is the spec to make stable clock)
	 */
	msleep(20);

	/* turn on main power for the MHL chip */
	gpio_set_value(pwr_gpio, 1);

	/* Assert FW_WAKE */
	gpio_set_value(fw_wake_gpio, 1);

	/* Assert RESET */
	/*
	 * probably, the 0 might not be needed since it is already low at probe.
	 * However, just incase, 0 has been set here.
	 */
	gpio_set_value(rst_gpio, 0);
	msleep(20);
	gpio_set_value(rst_gpio, 1);

	/* Active Power Control, then go into D2 mode */
	mhl_pf_write_reg(REG_PAGE_0_DPD, 0xFE);

	/* NOTE : following power ctrl is not enough to access i2c. */
	/* mhl_pf_write_reg(REG_PAGE_0_DPD, 0x10); */


}



void mhl_pf_chip_power_off(void)
{
	pr_debug("%s()\n", __func__);

	/* device goes into low power mode */
	mhl_pf_write_reg(REG_PAGE_0_DPD, 0x10);

	/* de-assert FW_WAKE */
	gpio_set_value(fw_wake_gpio, 0);

	/* turn off main power for the MHL chip */
	gpio_set_value(pwr_gpio, 0);

	/* de-assert RESET */
	gpio_set_value(rst_gpio, 0);

	/* disable the clock supply */
	mhl_pf_clock_disable();

	/* the state is referred by i2c function.
	 * So the state must be changed at the end.
	 */
	chip_pwr_state = CHIP_PWR_OFF;

}


static int get_gpios_from_device_tree(struct device_node *of_node)
{
	pr_debug("%s()\n", __func__);

	/* INTERRUPT */
	int_gpio = of_get_named_gpio(of_node, "mhl-intr-gpio", 0);
	if (int_gpio < 0) {
		pr_err("%s: Can't get mhl-intr-gpio\n", __func__);
		goto error;
	}
	pr_debug("%s():int_gpio:%d\n", __func__, int_gpio);

	/* 1.0V Power */
	pwr_gpio = of_get_named_gpio(of_node, "mhl-pwr-gpio", 0);
	if (pwr_gpio < 0) {
		pr_err("%s: Can't get mhl-pwr-gpio\n", __func__);
		goto error;
	}
	pr_debug("%s():pwr_gpio:%d\n", __func__, pwr_gpio);

	/* RESET */
	rst_gpio = of_get_named_gpio(of_node, "mhl-rst-gpio", 0);
	if (rst_gpio < 0) {
		pr_err("%s: Can't get mhl-rst-gpio\n", __func__);
		goto error;
	}
	pr_debug("%s():rst_gpio:%d\n", __func__, rst_gpio);

	/* USB/MHL switch (SEL1) */
	switch_sel_1_gpio = of_get_named_gpio(of_node,
				 "mhl-switch-sel-1-gpio", 0);
	if (switch_sel_1_gpio < 0) {
		pr_err("%s: Can't get mhl-switch-sel-1-gpio\n", __func__);
		goto error;
	}
	pr_debug("%s():switch_sel_1_gpio:%d\n", __func__, switch_sel_1_gpio);

	/* USB/MHL switch (SEL2) */
	switch_sel_2_gpio = of_get_named_gpio(of_node,
				 "mhl-switch-sel-2-gpio", 0);
	if (switch_sel_2_gpio < 0) {
		pr_err("%s: Can't get mhl-switch-sel-2-gpio\n", __func__);
		goto error;
	}
	pr_debug("%s():switch_sel_2_gpio:%d\n", __func__, switch_sel_2_gpio);

	/* FW WAKE */
	fw_wake_gpio = of_get_named_gpio(of_node, "mhl-fw-wake-gpio", 0);
	if (fw_wake_gpio < 0) {
		pr_err("%s: Can't get mhl-fw-wake-gpio\n", __func__);
		goto error;
	}
	pr_debug("%s():fw_wake_gpio:%d\n", __func__, fw_wake_gpio);
	return 0;

error:
	pr_err("%s: ret due to err\n", __func__);
	return -EBUSY;
}

static int mhl_tx_get_dt_data(struct device *dev_)
{
	int rc = 0;
	struct device_node *of_node = NULL;
	struct platform_device *op = NULL;

	mhl_clk = NULL;

	of_node = dev_->of_node;
	if (!of_node) {
		pr_err("%s: invalid of_node\n", __func__);
		goto error;
	}

	op = of_find_device_by_node(of_node->parent);
	if (!op) {
		pr_err("%s: invalid op\n", __func__);
		goto error;
	}

	mhl_clk = clk_get(&op->dev, "");
	/* mhl_clk = clk_get(dev,""); */
	if (!mhl_clk) {
		pr_err("%s: invalid clk\n", __func__);
		goto error;
	}

	rc = get_gpios_from_device_tree(of_node);
	if (rc) {
		pr_err("%s: error gpio init\n", __func__);
		goto error;
	}

	return 0;
error:
	pr_err("%s: ret due to err\n", __func__);
	return rc;
} /* mhl_tx_get_dt_data */



static int mhl_pf_clock_enable(void)
{
	int rc = -1;

	pr_debug("%s()\n", __func__);


	if (!pdev)
		return -EBUSY;

	if (!mhl_clk)
		mhl_clk = clk_get(pdev, "");

	if (!mhl_clk) {
		pr_err("%s: mhl clk is null\n", __func__);
		return -EBUSY;
	}

	rc = clk_prepare(mhl_clk);
	if (rc) {
		pr_err("%s: invalid clk prepare, rc : %d\n", __func__, rc);
		return -EBUSY;
	}

	clk_enable(mhl_clk);
	pr_debug("%s:clk is enabled\n", __func__);

	return 0;
}

static void mhl_pf_clock_disable(void)
{
	pr_debug("%s()\n", __func__);
	if (mhl_clk) {
		clk_disable_unprepare(mhl_clk);
		pr_debug("%s:clk is disabled\n", __func__);
	}
}

static int mhl_i2c_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct mhl_tx_ctrl *mhl_ctrl;
	int rc = -1;
	pr_info("%s:\n", __func__);

	chip_pwr_state = CHIP_PWR_OFF;

	mhl_ctrl = devm_kzalloc(&client->dev, sizeof(*mhl_ctrl), GFP_KERNEL);

	if (!mhl_ctrl) {
		pr_err("%s: FAILED: cannot alloc hdmi tx ctrl\n", __func__);
		rc = -ENOMEM;
		goto failed_no_mem;
	}

	pdev = &client->dev;

	if (client->dev.of_node) {
		rc = mhl_tx_get_dt_data(&client->dev);
		if (rc) {
			pr_err("%s:device tree error\n", __func__);
			goto failed_error;
		}
		rc = mhl_pf_gpio_config_init();
		if (rc) {
			pr_err("%s:device tree error\n", __func__);
			goto failed_error;
		}
	}

	pdev->class = class_create(THIS_MODULE, "mhl");
	if (IS_ERR(pdev->class)) {
		pr_err("%s:fail class creation\n", __func__);
		rc = PTR_ERR(pdev->class);
		goto failed_error;
	}
	pr_info("%s:class name : %s\n", __func__, pdev->class->name);
	i2c_set_clientdata(client, mhl_ctrl);

	mhl_pf_i2c_init(client->adapter);
	device_name = client->dev.driver->name;
	pr_debug("%s:device name : %s\n", __func__, device_name);

	if (!rc) {
		/*
		 * libs should be initizlized first
		 * since there could be used by other module
		 */
		rc = mhl_lib_timer_init();
		if(rc < 0)
			goto failed_error;

		mhl_device_initialize(&client->dev);
		mhl_tx_rcp_init(pdev);
		rc = mhl_tx_initialize();
		if (rc < 0)
			goto failed_error;
	}

	return 0;

failed_error:

failed_no_mem:
	if (mhl_ctrl)
		devm_kfree(&client->dev, mhl_ctrl);
	return rc;
}

static int mhl_i2c_remove(struct i2c_client *client)
{
	struct mhl_tx_ctrl *mhl_ctrl = i2c_get_clientdata(client);
	pr_info("%s:\n", __func__);

	/*
	 * MHL driver sw module release
	 */
	/* release clock */
	if (mhl_clk) {
		pr_info("%s: disable mhl clk\n", __func__);
		clk_disable_unprepare(mhl_clk);
		clk_put(mhl_clk);
	}
	mhl_tx_rcp_release();
	mhl_pf_switch_to_usb();
	mhl_device_release(&client->dev);
	mhl_tx_release();
	/* libs should be release at the end
	 * since there could be used by other sw module */
	mhl_lib_timer_release();


	/*
	 * All gpio will be released. All release needing gpio
	 * must be released before this API call
	 */
	mhl_pf_gpio_config_release();


	/* mhl device class is released */
	class_destroy(client->dev.class);

	if (!mhl_ctrl)
		pr_warn("%s: i2c get client data failed\n", __func__);
	else
		devm_kfree(&client->dev, mhl_ctrl);

	return 0;
}

#ifndef UNIT_TEST
static struct i2c_device_id mhl_sii_i2c_id[] = {
	{ MHL_DRIVER_NAME, 0 },
	{}
};

static const struct of_device_id mhl_match_table[] = {
	{.compatible = COMPATIBLE_NAME},
	{}
};

static struct i2c_driver mhl_sii_i2c_driver = {
	.driver = {
		.name = MHL_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mhl_match_table,
	},
	.probe = mhl_i2c_probe,
	.remove =  mhl_i2c_remove,
	.id_table = mhl_sii_i2c_id,
};

module_i2c_driver(mhl_sii_i2c_driver);

MODULE_LICENSE("GPL");
#endif
