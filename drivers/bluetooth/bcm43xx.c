/*
 * Bluetooth Broadcomm  and low power control via GPIO
 *
 *  Copyright (C) 2011 Google, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Copyright (C) 2014 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/rfkill.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/termios.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>

#include <linux/bcm43xx_bt_lpm.h>
#include <linux/platform_data/msm_serial_hs.h>

#define D_BCM_BLUETOOTH_CONFIG_MATCH_TABLE   "bcm,bcm43xx"

enum gpio_id {
	BT_REG_ON_PIN = 0,
	BT_DEV_WAKE_PIN,
};

static char const * const gpio_rsrcs[] = {
	"bt-reg-on-gpio",
	"bt-dev-wake-gpio",
};

struct bcm43xx_data {
	struct device *dev;
	struct platform_device *pdev;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
	struct pinctrl_state *gpio_state_hostwake_active;
	struct pinctrl_state *gpio_state_hostwake_suspend;
	unsigned int gpios[ARRAY_SIZE(gpio_rsrcs)];
	bool is_enable;
};

static enum gpio_id req_ids[] = {
	BT_REG_ON_PIN,
	BT_DEV_WAKE_PIN,
};

static struct bcm43xx_data *bcm43xx_my_data;
static struct rfkill *bt_rfkill;
static bool bt_enabled;

struct bcm_bt_lpm {
	int wake;

	struct hrtimer enter_lpm_timer;
	ktime_t enter_lpm_delay;

	struct uart_port *uport;

	struct mutex mutex;
	struct work_struct enter_lpm_work;
} bt_lpm;

static int bcm43xx_bt_rfkill_set_power(void *data, bool blocked)
{
	int regOnGpio;
	int ret;

	pr_debug("Bluetooth device set power\n");

	regOnGpio = gpio_get_value(bcm43xx_my_data->gpios[BT_REG_ON_PIN]);

	/* rfkill_ops callback. Turn transmitter on when blocked is false */
	if (!blocked) {
		if (regOnGpio) {
			pr_debug("Bluetooth device is already power on:%d\n",
				regOnGpio);
			return 0;
		}
		ret = pinctrl_select_state(bcm43xx_my_data->pinctrl,
			bcm43xx_my_data->gpio_state_hostwake_active);
		if (ret)
			pr_err("%s(): Failed to select active state\n", __func__);
		gpio_set_value(bcm43xx_my_data->gpios[BT_DEV_WAKE_PIN], 1);
		gpio_set_value(bcm43xx_my_data->gpios[BT_REG_ON_PIN], 1);
	} else {
		if (!regOnGpio) {
			pr_debug("Bluetooth device is already power off:%d\n",
				regOnGpio);
			return 0;
		}
		gpio_set_value(bcm43xx_my_data->gpios[BT_REG_ON_PIN], 0);
		ret = pinctrl_select_state(bcm43xx_my_data->pinctrl,
			bcm43xx_my_data->gpio_state_hostwake_suspend);
		if (ret)
			pr_err("%s(): Failed to select suspend state\n", __func__);
	}
	bt_enabled = !blocked;

	return 0;
}

static const struct rfkill_ops bcm43xx_bt_rfkill_ops = {
	.set_block = bcm43xx_bt_rfkill_set_power,
};

static enum hrtimer_restart enter_lpm(struct hrtimer *timer)
{
	schedule_work(&bt_lpm.enter_lpm_work);
	return HRTIMER_NORESTART;
}

static void enter_lpm_work(struct work_struct *data)
{
	pr_debug("Bluetooth device sleep\n");

	mutex_lock(&bt_lpm.mutex);
	if (hrtimer_active(&bt_lpm.enter_lpm_timer) == 0) {
		bt_lpm.wake = 0;
		gpio_set_value(bcm43xx_my_data->gpios[BT_DEV_WAKE_PIN], 0);
	}
	mutex_unlock(&bt_lpm.mutex);
}

void bcm_bt_lpm_exit_lpm_locked(struct uart_port *uport)
{
	pr_debug("Bluetooth device exit lpm\n");

	mutex_lock(&bt_lpm.mutex);

	bt_lpm.uport = uport;

	hrtimer_try_to_cancel(&bt_lpm.enter_lpm_timer);

	bt_lpm.wake = 1;
	gpio_set_value(bcm43xx_my_data->gpios[BT_DEV_WAKE_PIN], 1);

	hrtimer_start(&bt_lpm.enter_lpm_timer, bt_lpm.enter_lpm_delay,
		HRTIMER_MODE_REL);

	mutex_unlock(&bt_lpm.mutex);
}
EXPORT_SYMBOL(bcm_bt_lpm_exit_lpm_locked);

static int bcm_bt_lpm_init(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "bcm_bt_lpm_init\n");

	hrtimer_init(&bt_lpm.enter_lpm_timer, CLOCK_MONOTONIC,
		HRTIMER_MODE_REL);
	bt_lpm.enter_lpm_delay = ktime_set(1, 0);  /* 1 sec */
	bt_lpm.enter_lpm_timer.function = enter_lpm;

	mutex_init(&bt_lpm.mutex);

	INIT_WORK(&bt_lpm.enter_lpm_work, enter_lpm_work);

	return 0;
}

static void bcm43xx_bluetooth_free_gpio(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(req_ids); i++)
		gpio_free(bcm43xx_my_data->gpios[req_ids[i]]);
}

static int bcm43xx_bluetooth_dev_init(struct platform_device *pdev,
				struct bcm43xx_data *my_data)
{
	int i, ret, ret_sus, gpio;
	unsigned int flags;
	struct pinctrl *pinctrl;
	struct pinctrl_state *lookup_state;
	struct device_node *of_node = pdev->dev.of_node;

	my_data->is_enable = false;

	pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&pdev->dev, "%s: pinctrl not defined\n",
			__func__);
		ret = PTR_ERR(pinctrl);
		goto error_pinctrl;
	}
	my_data->pinctrl = pinctrl;

	lookup_state = pinctrl_lookup_state(pinctrl, PINCTRL_STATE_DEFAULT);
	if (IS_ERR_OR_NULL(lookup_state)) {
		dev_err(&pdev->dev, "%s(): pinctrl lookup failed for default\n",
			__func__);
		ret = PTR_ERR(lookup_state);
		goto error_pinctrl;
	}
	my_data->gpio_state_active = lookup_state;

	lookup_state = pinctrl_lookup_state(pinctrl, PINCTRL_STATE_SLEEP);
	if (IS_ERR_OR_NULL(lookup_state)) {
		dev_err(&pdev->dev, "%s(): pinctrl lookup failed for sleep\n",
			__func__);
		ret = PTR_ERR(lookup_state);
		goto error_pinctrl;
	}
	my_data->gpio_state_suspend = lookup_state;

	lookup_state = pinctrl_lookup_state(pinctrl, "wake_irq_active");
	if (IS_ERR_OR_NULL(lookup_state)) {
		dev_err(&pdev->dev, "%s(): pinctrl lookup failed for hostwake active\n",
			__func__);
		ret = PTR_ERR(lookup_state);
		goto error_pinctrl;
	}
	my_data->gpio_state_hostwake_active = lookup_state;

	lookup_state = pinctrl_lookup_state(pinctrl, "wake_irq_suspend");
	if (IS_ERR_OR_NULL(lookup_state)) {
		dev_err(&pdev->dev, "%s(): pinctrl lookup failed for hostwake sleep\n",
			__func__);
		ret = PTR_ERR(lookup_state);
		goto error_pinctrl;
	}
	my_data->gpio_state_hostwake_suspend = lookup_state;

	ret = pinctrl_select_state(bcm43xx_my_data->pinctrl,
		bcm43xx_my_data->gpio_state_active);
	if (ret) {
		dev_err(&pdev->dev, "%s(): Failed to select active state\n",
			__func__);
		goto error_pinctrl;
	}

	for (i = 0; i < ARRAY_SIZE(gpio_rsrcs); i++) {
		gpio = of_get_gpio_flags(of_node, i, &flags);
		if (!gpio_is_valid(gpio)) {
			dev_err(&pdev->dev, "%s(): invalid gpio #%s: %d\n",
				__func__, gpio_rsrcs[i], gpio);
			ret = -ENXIO;
			goto error_gpio;
		}
		my_data->gpios[i] = gpio;
	}

	for (i = 0; i < ARRAY_SIZE(req_ids); i++) {
		ret = gpio_request(my_data->gpios[req_ids[i]],
				gpio_rsrcs[req_ids[i]]);
		if (ret) {
			dev_err(&pdev->dev, "%s(): request err %s: %d\n",
				__func__, gpio_rsrcs[req_ids[i]], ret);
			goto error_gpio_request;
		}
	}

	return 0;

error_gpio_request:
	for (; i >= 0; --i)
			gpio_free(my_data->gpios[req_ids[i]]);
error_gpio:
	ret_sus = pinctrl_select_state(bcm43xx_my_data->pinctrl,
		bcm43xx_my_data->gpio_state_suspend);
	if (ret_sus)
		dev_warn(&pdev->dev, "%s(): Failed to select suspend state\n",
			__func__);
error_pinctrl:
	return ret;

}

static int bcm43xx_bluetooth_probe(struct platform_device *pdev)
{
	int rc, ret, ret_sus;

	struct device_node *of_node = pdev->dev.of_node;
	dev_dbg(&pdev->dev, "bcm43xx bluetooth driver being loaded\n");

	if (!of_node) {
		dev_err(&pdev->dev, "%s(): of_node is null\n", __func__);
		ret = -EPERM;
		goto error_of_node;
	}

	bcm43xx_my_data = kzalloc(sizeof(*bcm43xx_my_data), GFP_KERNEL);
	if (!bcm43xx_my_data) {
		dev_err(&pdev->dev, "%s(): no memory\n", __func__);
		ret = -ENOMEM;
		goto error_alloc_mydata;
	}
	bcm43xx_my_data->pdev = pdev;

	ret = bcm43xx_bluetooth_dev_init(pdev, bcm43xx_my_data);
	if (ret) {
		dev_err(&pdev->dev, "%s(): dev init failed\n", __func__);
		goto error_dev_init;
	}

	bt_rfkill = rfkill_alloc("bcm43xx Bluetooth", &pdev->dev,
				RFKILL_TYPE_BLUETOOTH, &bcm43xx_bt_rfkill_ops,
				NULL);

	if (unlikely(!bt_rfkill)) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "%s(): rfkill_alloc fail\n",  __func__);
		goto  error_free_gpio;
	}

	rfkill_set_states(bt_rfkill, true, false);
	rc = rfkill_register(bt_rfkill);

	if (unlikely(rc)) {
		rfkill_destroy(bt_rfkill);
		ret = -ENOMEM;
		dev_err(&pdev->dev, "%s(): rfkill_register fail\n", __func__);
		goto  error_free_gpio;
	}

	ret = bcm_bt_lpm_init(pdev);
	if (ret) {
		rfkill_unregister(bt_rfkill);
		rfkill_destroy(bt_rfkill);
		dev_err(&pdev->dev, "%s() bcm_bt_lpm_init fail\n", __func__);
		goto  error_free_gpio;
	}
	return 0;

error_free_gpio:
	bcm43xx_bluetooth_free_gpio();
	ret_sus = pinctrl_select_state(bcm43xx_my_data->pinctrl,
		bcm43xx_my_data->gpio_state_suspend);
	if (ret_sus)
		dev_warn(&pdev->dev, "%s(): Failed to select suspend state\n",
			__func__);
error_dev_init:
	kzfree(bcm43xx_my_data);
error_alloc_mydata:
error_of_node:
	return ret;
}

static int bcm43xx_bluetooth_remove(struct platform_device *pdev)
{
	int ret;

	dev_dbg(&pdev->dev, "bcm43xx_bluetooth_remove\n");
	rfkill_unregister(bt_rfkill);
	rfkill_destroy(bt_rfkill);

	if (!IS_ERR_OR_NULL(bcm43xx_my_data) &&
			!IS_ERR_OR_NULL(bcm43xx_my_data->pinctrl) &&
			!IS_ERR_OR_NULL(bcm43xx_my_data->gpio_state_suspend)) {
		ret = pinctrl_select_state(bcm43xx_my_data->pinctrl,
			bcm43xx_my_data->gpio_state_suspend);
		if (ret)
			dev_warn(&pdev->dev, "%s(): Failed to select suspend state",
				__func__);
	}

	kzfree(bcm43xx_my_data);

	return 0;
}

int bcm43xx_bluetooth_suspend(struct platform_device *pdev, pm_message_t state)
{
	dev_dbg(&pdev->dev, "bcm43xx_bluetooth_suspend\n");
	return 0;
}

int bcm43xx_bluetooth_resume(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "bcm43xx_bluetooth_resume\n");
	return 0;
}

static struct of_device_id bcm43xx_match_table[] = {
	{.compatible = D_BCM_BLUETOOTH_CONFIG_MATCH_TABLE },
	{}
};

static struct platform_driver bcm43xx_bluetooth_platform_driver = {
	.probe = bcm43xx_bluetooth_probe,
	.remove = bcm43xx_bluetooth_remove,
	.suspend = bcm43xx_bluetooth_suspend,
	.resume = bcm43xx_bluetooth_resume,
	.driver = {
		.name = "bcm43xx_bluetooth",
		.owner = THIS_MODULE,
		.of_match_table = bcm43xx_match_table,
	},
};

static int __init bcm43xx_bluetooth_init(void)
{
	bt_enabled = false;
	return platform_driver_register(&bcm43xx_bluetooth_platform_driver);
}

static void __exit bcm43xx_bluetooth_exit(void)
{
	platform_driver_unregister(&bcm43xx_bluetooth_platform_driver);
}


module_init(bcm43xx_bluetooth_init);
module_exit(bcm43xx_bluetooth_exit);

MODULE_ALIAS("platform:bcm43xx");
MODULE_DESCRIPTION("bcm43xx_bluetooth");
MODULE_AUTHOR("Jaikumar Ganesh <jaikumar@google.com>");
MODULE_LICENSE("GPL v2");
