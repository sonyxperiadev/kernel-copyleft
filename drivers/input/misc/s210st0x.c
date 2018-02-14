/* drivers/input/misc/s210st0x.c
 *
 * Author: Atsushi Iyogi <atsushi2.X.iyogi@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
/*
 * Copyright (C) 2016 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/input/s210st0x.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/types.h>

#define S210ST0X_DEV_NAME "s210st0x"

struct s210st0x_event_data {
	const struct s210st0x_gpio_event *event;
	struct timer_list det_timer;
	struct work_struct det_work;
	unsigned int timer_debounce;
	unsigned int irq;
};

struct s210st0x_drvdata {
	struct device *dev;
	struct pinctrl *key_pinctrl;
	struct input_dev *input_dev;
	struct mutex lock;
	atomic_t detection_in_progress;
	unsigned int n_events;
	unsigned int current_state;
	struct s210st0x_event_data data[0];
};

enum s210st0x_input_state {
	NOT_DETECTED,
	DETECTED,
};

static int s210st0x_disable;

static int s210st0x_get_state(const struct s210st0x_gpio_event *event)
{
	return gpio_get_value_cansleep(event->gpio)
				       ^ event->active_low ?
				       DETECTED : NOT_DETECTED;
}

static void s210st0x_input_report(struct input_dev *idev,
				  const struct s210st0x_gpio_event *event)
{
	int gpio_state;
	struct s210st0x_drvdata *ddata = input_get_drvdata(idev);

	gpio_state = s210st0x_get_state(event);
	dev_dbg(&idev->dev, "%s: value(%d)\n", __func__, gpio_state);
	if (gpio_state == DETECTED) {
		input_report_key(idev, KEY_WAKEUP, 1);
		input_sync(idev);
		input_report_key(idev, KEY_WAKEUP, 0);
		input_sync(idev);
	}
	ddata->current_state = gpio_state;
}

static int s210st0x_get_devtree(struct device *dev,
				  struct s210st0x_platform_data *pdata)
{
	struct device_node *node, *pp = NULL;
	int i = 0;
	struct s210st0x_gpio_event *events;
	u32 reg;
	int gpio;
	int ret = -ENODEV;
	enum of_gpio_flags flags;

	node = dev->of_node;
	if (node == NULL)
		goto fail;

	memset(pdata, 0, sizeof(*pdata));

	pdata->n_events = 0;
	pp = NULL;
	while ((pp = of_get_next_child(node, pp)))
		pdata->n_events++;

	if (pdata->n_events == 0)
		goto fail;

	events = kzalloc(pdata->n_events * sizeof(*events), GFP_KERNEL);
	if (!events) {
		ret = -ENOMEM;
		goto fail;
	}

	while ((pp = of_get_next_child(node, pp))) {

		if (!of_find_property(pp, "gpios", NULL)) {
			pdata->n_events--;
			dev_warn(dev, "Found button without gpios\n");
			continue;
		}

		gpio = of_get_gpio_flags(pp, 0, &flags);
		if (!gpio_is_valid(gpio)) {
			dev_err(dev, "%s: invalid gpio %d\n", __func__, gpio);
			goto out_fail;
		}
		events[i].index = i;
		events[i].gpio = gpio;
		events[i].active_low = flags & OF_GPIO_ACTIVE_LOW;

		events[i].desc = of_get_property(pp, "label", NULL);

		if (of_property_read_u32(pp, "debounce-interval", &reg) == 0)
			events[i].debounce_interval = reg;
		i++;
	}
	pdata->events = events;

	return 0;

out_fail:
	kfree(events);
fail:
	return ret;
}

static irqreturn_t s210st0x_isr(int irq, void *data)
{
	struct s210st0x_event_data *edata =
			(struct s210st0x_event_data *)data;
	const struct s210st0x_gpio_event *event = edata->event;
	struct s210st0x_drvdata *ddata = container_of(edata,
						struct s210st0x_drvdata,
						data[event->index]);

	pm_stay_awake(ddata->dev);

	if (edata->timer_debounce)
		mod_timer(&edata->det_timer,
			jiffies + msecs_to_jiffies(edata->timer_debounce));
	else
		schedule_work(&edata->det_work);

	atomic_set(&ddata->detection_in_progress, 1);

	return IRQ_HANDLED;
}

static void s210st0x_det_tmr_func(unsigned long func_data)
{
	struct s210st0x_event_data *edata =
		(struct s210st0x_event_data *)func_data;

	schedule_work(&edata->det_work);
}

static void s210st0x_det_work(struct work_struct *work)
{
	struct s210st0x_event_data *edata =
		container_of(work, struct s210st0x_event_data, det_work);
	const struct s210st0x_gpio_event *event = edata->event;
	struct s210st0x_drvdata *ddata = container_of(edata,
						struct s210st0x_drvdata,
						data[event->index]);

	s210st0x_input_report(ddata->input_dev, event);

	pm_relax(ddata->dev);

	atomic_set(&ddata->detection_in_progress, 0);
}

static int s210st0x_pinctrl_configure(struct s210st0x_drvdata *ddata,
							bool active)
{
	struct pinctrl_state *set_state;
	int retval;

	if (active) {
		set_state =
			pinctrl_lookup_state(ddata->key_pinctrl,
						"tlmm_s210st0x_active");
		if (IS_ERR(set_state)) {
			dev_err(ddata->dev,
				"cannot get ts pinctrl active state\n");
			goto lookup_err;
		}
	} else {
		set_state =
			pinctrl_lookup_state(ddata->key_pinctrl,
						"tlmm_s210st0x_suspend");
		if (IS_ERR(set_state)) {
			dev_err(ddata->dev,
				"cannot get gpiokey pinctrl sleep state\n");
			goto lookup_err;
		}
	}
	retval = pinctrl_select_state(ddata->key_pinctrl, set_state);
	if (retval) {
		dev_err(ddata->dev,
				"cannot set ts pinctrl active state\n");
		goto select_err;
	}

	return 0;

lookup_err:
	return PTR_ERR(set_state);
select_err:
	return retval;
}

static int s210st0x_open(struct input_dev *idev)
{
	struct s210st0x_drvdata *ddata = input_get_drvdata(idev);
	struct s210st0x_event_data *edata;
	const struct s210st0x_gpio_event *event;
	int i;

	mutex_lock(&ddata->lock);
	for (i = 0; i < ddata->n_events; i++) {
		edata = &ddata->data[i];
		event = edata->event;
		s210st0x_input_report(ddata->input_dev, event);
	}
	mutex_unlock(&ddata->lock);

	return 0;
}

static int s210st0x_setup_event(struct platform_device *pdev,
				  struct s210st0x_event_data *edata,
				  const struct s210st0x_gpio_event *event)
{
	const char *desc = event->desc ? event->desc : S210ST0X_DEV_NAME;
	struct device *dev = &pdev->dev;
	irq_handler_t isr;
	unsigned long irqflags;
	int irq, error;

	edata->event = event;

	error = gpio_request(event->gpio, desc);
	if (error < 0)
		dev_warn(dev, "Failed to request GPIO %d, error %d\n",
			event->gpio, error);

	error = gpio_direction_input(event->gpio);
	if (error < 0) {
		dev_err(dev,
			"Failed to configure direction for GPIO %d, error %d\n",
			event->gpio, error);
		goto fail;
	}

	edata->timer_debounce = event->debounce_interval;

	irq = gpio_to_irq(event->gpio);
	if (irq < 0) {
		error = irq;
		dev_err(dev,
			"Unable to get irq number for GPIO %d, error %d\n",
			event->gpio, error);
		goto fail;
	}
	edata->irq = irq;

	isr = s210st0x_isr;
	irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;

	INIT_WORK(&edata->det_work, s210st0x_det_work);

	setup_timer(&edata->det_timer,
		    s210st0x_det_tmr_func, (unsigned long)edata);

	error = request_any_context_irq(edata->irq, isr, irqflags, desc, edata);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			event->irq, error);
		goto fail;
	}
	enable_irq_wake(edata->irq);

	return 0;

fail:
	gpio_free(event->gpio);
	return error;
}

static void s210st0x_remove_event(struct s210st0x_event_data *edata)
{
	free_irq(edata->irq, edata);
	del_timer_sync(&edata->det_timer);
	cancel_work_sync(&edata->det_work);
	if (gpio_is_valid(edata->event->gpio))
		gpio_free(edata->event->gpio);
}

static ssize_t s210st0x_attrs_current_state_read(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct s210st0x_drvdata *ddata = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", ddata->current_state);
}

static ssize_t s210st0x_attrs_current_disable_read(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", s210st0x_disable);
}

static ssize_t s210st0x_attrs_current_disable_write(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct s210st0x_drvdata *ddata = dev_get_drvdata(dev);
	struct s210st0x_event_data *edata = &ddata->data[0];
	unsigned long state;

	if (kstrtoul(buf, 10, &state))
		return -EINVAL;

	if (s210st0x_disable != state) {
		dev_dbg(ddata->dev, "%s change irq state=%lu \n", __func__, state);
		s210st0x_disable = state;
		if (state) {
			disable_irq(edata->irq);
			disable_irq_wake(edata->irq);
		} else {
			enable_irq(edata->irq);
			enable_irq_wake(edata->irq);
		}
	}
	return size;
}

static struct device_attribute s210st0x_state_attr[] = {
	__ATTR(s210st0x_state,
	       S_IRUGO, s210st0x_attrs_current_state_read, NULL),
};

static struct device_attribute s210st0x_disable_attr[] = {
	__ATTR(s210st0x_disable,
			S_IRUSR | S_IWUSR, s210st0x_attrs_current_disable_read,
			s210st0x_attrs_current_disable_write),
};

static int s210st0x_set_input_device(struct s210st0x_drvdata *ddata)
{
	struct input_dev *input;
	int error = 0;

	input = input_allocate_device();
	if (!input) {
		dev_err(ddata->dev, "failed to allocate input_dev (%s)\n",
			__func__);
		error = -ENOMEM;
		goto out;
	}
	ddata->input_dev = input;

	input_set_drvdata(ddata->input_dev, ddata);

	ddata->input_dev->name = S210ST0X_DEV_NAME;
	ddata->input_dev->open = s210st0x_open;

	error = input_register_device(ddata->input_dev);
	if (error) {
		dev_err(ddata->dev, "failed to register device (%s)\n",
			__func__);
		goto fail;
	}
	input_set_capability(ddata->input_dev, EV_KEY, KEY_WAKEUP);

	goto out;

fail:
	input_unregister_device(ddata->input_dev);
out:
	return error;

}

static int s210st0x_probe(struct platform_device *pdev)
{
	struct s210st0x_platform_data *pdata = pdev->dev.platform_data;
	struct s210st0x_platform_data alt_pdata;
	const struct s210st0x_gpio_event *event;
	struct s210st0x_event_data *edata;
	struct s210st0x_drvdata *ddata;
	int i = 0;
	int error = 0;
	struct pinctrl_state *set_state;

	if (!pdata) {
		error = s210st0x_get_devtree(&pdev->dev, &alt_pdata);
		if (error)
			goto fail;
		pdata = &alt_pdata;
	}

	ddata = kzalloc(sizeof(struct s210st0x_drvdata) +
			pdata->n_events * sizeof(struct s210st0x_event_data),
			GFP_KERNEL);
	if (!ddata) {
		dev_err(&pdev->dev, "failed to allocate drvdata in probe\n");
		error = -ENOMEM;
		goto fail;
	}

	ddata->dev = &pdev->dev;
	ddata->n_events = pdata->n_events;
	ddata->key_pinctrl = devm_pinctrl_get(ddata->dev);
	mutex_init(&ddata->lock);

	platform_set_drvdata(pdev, ddata);
	dev_set_drvdata(ddata->dev, ddata);

	if (IS_ERR(ddata->key_pinctrl)) {
		if (PTR_ERR(ddata->key_pinctrl) == -EPROBE_DEFER)
			goto fail_pinctrl;
		pr_debug("Target does not use pinctrl\n");
		ddata->key_pinctrl = NULL;
	}

	if (ddata->key_pinctrl) {
		error = s210st0x_pinctrl_configure(ddata, true);
		if (error) {
			dev_err(ddata->dev,
				"cannot set ts pinctrl active state\n");
			goto fail_pinctrl;
		}
	}

	error = s210st0x_set_input_device(ddata);
	if (error) {
		dev_err(ddata->dev, "%s cannot set input dev(%d)\n",
			__func__, error);
		goto fail_set_input;
	}

	error = device_create_file(ddata->dev, s210st0x_state_attr);
	if (error) {
		dev_err(ddata->dev, "%s: create_file failed %d\n",
			__func__, error);
		goto fail_device_create_file;
	}

	error = device_create_file(ddata->dev, s210st0x_disable_attr);
	if (error) {
		dev_err(ddata->dev, "%s: create_file failed %d\n",
			__func__, error);
		goto fail_device_create_file;
	}

	for (i = 0; i < pdata->n_events; i++) {
		event = &pdata->events[i];
		edata = &ddata->data[i];

		error = s210st0x_setup_event(pdev, edata, event);
		if (error) {
			dev_err(ddata->dev, "%s cannot set event error(%d)\n",
				__func__, error);
			goto fail_setup_event;
		}
	}
	ddata->current_state = s210st0x_get_state(event);

	dev_warn(ddata->dev, "s210st0x driver was successful.\n");
	return 0;

fail_setup_event:
	device_remove_file(ddata->dev, s210st0x_state_attr);
	device_remove_file(ddata->dev, s210st0x_disable_attr);
fail_device_create_file:
	input_unregister_device(ddata->input_dev);
	input_free_device(ddata->input_dev);
fail_set_input:
	if (ddata->key_pinctrl) {
		set_state =
		pinctrl_lookup_state(ddata->key_pinctrl,
				     "tlmm_s210st0x_suspend");
		if (IS_ERR(set_state))
			dev_err(ddata->dev, "cannot get pinctrl sleep state\n");
		else
			pinctrl_select_state(ddata->key_pinctrl, set_state);
	}
	while (--i >= 0)
		s210st0x_remove_event(&ddata->data[i]);
	platform_set_drvdata(pdev, NULL);
fail_pinctrl:
	mutex_destroy(&ddata->lock);
	kzfree(ddata);
fail:
	return error;
}

static int s210st0x_remove(struct platform_device *pdev)
{
	int i;
	struct s210st0x_drvdata *ddata = platform_get_drvdata(pdev);

	device_remove_file(ddata->dev, s210st0x_state_attr);
	device_remove_file(ddata->dev, s210st0x_disable_attr);
	input_unregister_device(ddata->input_dev);
	mutex_destroy(&ddata->lock);
	for (i = 0; i < ddata->n_events; i++)
		s210st0x_remove_event(&ddata->data[i]);
	if (!pdev->dev.platform_data)
		kfree(ddata->data[0].event);
	kzfree(ddata);

	return 0;
}

static int s210st0x_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct s210st0x_drvdata *ddata = platform_get_drvdata(pdev);
	int ret = 0;

	if (atomic_read(&ddata->detection_in_progress)) {
		dev_dbg(&pdev->dev, "detection in progress. (%s)\n",
			__func__);
		ret = -EAGAIN;
		goto out;
	}

	if (ddata->key_pinctrl) {
		ret = s210st0x_pinctrl_configure(ddata, false);
		if (ret)
			dev_err(&pdev->dev, "failed to put the pin\n");
	}

out:
	return ret;
}

static int s210st0x_resume(struct platform_device *pdev)
{
	struct s210st0x_drvdata *ddata = platform_get_drvdata(pdev);
	int ret = 0;

	if (ddata->key_pinctrl) {
		ret = s210st0x_pinctrl_configure(ddata, true);
		if (ret)
			dev_err(&pdev->dev, "failed to put the pin\n");
	}

	return ret;
}

static struct of_device_id s210st0x_match_table[] = {
	{	.compatible = "murata,s210st0x",
	},
	{}
};

static struct platform_driver s210st0x_driver = {
	.probe = s210st0x_probe,
	.remove = s210st0x_remove,
	.suspend = s210st0x_suspend,
	.resume = s210st0x_resume,
	.driver = {
		.name = S210ST0X_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = s210st0x_match_table,
	},
};

static int __init s210st0x_init(void)
{
	return platform_driver_register(&s210st0x_driver);
}

static void __exit s210st0x_exit(void)
{
	platform_driver_unregister(&s210st0x_driver);
}

module_init(s210st0x_init);
module_exit(s210st0x_exit);

MODULE_LICENSE("GPLv2");

