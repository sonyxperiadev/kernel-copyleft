/* drivers/regulator/fpf2495.c
 *
 * based on fixed.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
/*
 * Copyright (C) 2014 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/fpf2495.h>

#define FPF2495_DEFAULT_MICROVOLTS 5000000
#define FPF2495_OC_DELAY 1
#define FPF2495_FLAG_READY_WAIT 100

/* Phase of over current detection */
enum fpf2495_oc_det_state {
	OC_DET_STOP,
	OC_DET_READY,
	OC_DET_START,
	OC_DET_SETTLE,
};

struct fpf2495_data {
	struct device *dev;
	struct regulator_desc desc;
	struct regulator_dev *rdev;
	int microvolts;
	struct {
		int gpio;
		unsigned int active_high:1;
		int status;
		unsigned long flags;
	} gpio_en, gpio_flag;
	int vout_is_on;
	struct mutex mutex;
	struct regulator *flag_pull;
	int flag_irq;
	atomic_t oc_det_state;
	atomic_t oc_delay;
	struct delayed_work oc_delay_work;
	struct regulator_ocp_notification ocp_notification;
	spinlock_t ocp_lock;
};

#define ktime_set_from_us(us) \
	ktime_set((us) / USEC_PER_SEC, ((us) % USEC_PER_SEC) * NSEC_PER_USEC)

static const char *rdev_get_name(struct regulator_dev *rdev)
{
	if (rdev->constraints && rdev->constraints->name)
		return rdev->constraints->name;
	else if (rdev->desc->name)
		return rdev->desc->name;
	else
		return "";
}

/**
 * of_get_fpf2495_config - extract fpf2495_config structure info
 * @dev: device requesting for fpf2495_config
 *
 * Populates fpf2495_config structure by extracting data from device
 * tree node, returns a pointer to the populated structure of NULL if memory
 * alloc fails.
 */
static struct fpf2495_config *of_get_fpf2495_config(struct device *dev)
{
	struct fpf2495_config *config;
	struct device_node *np = dev->of_node;
	const __be32 *delay;
	struct regulator_init_data *init_data;

	config = devm_kzalloc(dev, sizeof(struct fpf2495_config), GFP_KERNEL);
	if (!config)
		return ERR_PTR(-ENOMEM);

	config->init_data = of_get_regulator_init_data(dev, dev->of_node);
	if (!config->init_data)
		return ERR_PTR(-EINVAL);

	init_data = config->init_data;
	init_data->constraints.apply_uV = 0;

	config->supply_name = init_data->constraints.name;
	if (init_data->constraints.min_uV == init_data->constraints.max_uV) {
		config->microvolts = init_data->constraints.min_uV;
	} else {
		dev_err(dev,
			 "Fixed regulator specified with variable voltages\n");
		return ERR_PTR(-EINVAL);
	}

	config->gpio_en.gpio = of_get_named_gpio(np, "gpio_en", 0);
	/*
	 * of_get_named_gpio() currently returns ENODEV rather than
	 * EPROBE_DEFER. This code attempts to be compatible with both
	 * for now; the ENODEV check can be removed once the API is fixed.
	 * of_get_named_gpio() doesn't differentiate between a missing
	 * property (which would be fine here, since the GPIO is optional)
	 * and some other error. Patches have been posted for both issues.
	 * Once they are check in, we should replace this with:
	 * if (config->gpio_en < 0 && config->gpio_en != -ENOENT)
	 */
	if ((config->gpio_en.gpio == -ENODEV) ||
			(config->gpio_en.gpio == -EPROBE_DEFER))
		return ERR_PTR(-EPROBE_DEFER);

	delay = of_get_property(np, "startup-delay-us", NULL);
	if (delay)
		config->startup_delay = be32_to_cpu(*delay);

	config->gpio_en.active_high = 1;
	if (of_find_property(np, "gpio_en-active_low", NULL))
		config->gpio_en.active_high = 0;

	if (of_find_property(np, "vin-supply", NULL))
		config->input_supply = "vin";

	if (of_find_property(np, "flag_pull-supply", NULL))
		config->flag_pull_supply = "flag_pull";

	config->gpio_flag.gpio = of_get_named_gpio(np, "gpio_flag", 0);
	if ((config->gpio_flag.gpio == -ENODEV) ||
			(config->gpio_flag.gpio == -EPROBE_DEFER))
		return ERR_PTR(-EPROBE_DEFER);

	if (of_find_property(np, "gpio_flag-active_high", NULL))
		config->gpio_flag.active_high = 1;

	delay = of_get_property(np, "oc-delay-ms", NULL);
	if (delay)
		config->oc_delay = be32_to_cpu(*delay);

	return config;
}

static int fpf2495_set_voltage_sel(struct regulator_dev *dev,
				      unsigned selector)
{
	struct fpf2495_data *data = rdev_get_drvdata(dev);

	if (!(data->desc.n_voltages > selector))
		return -EINVAL;

	return 0;
}

static int fpf2495_get_voltage(struct regulator_dev *dev)
{
	struct fpf2495_data *data = rdev_get_drvdata(dev);

	if (data->microvolts)
		return data->microvolts;
	else
		return -EINVAL;
}

static int fpf2495_list_voltage(struct regulator_dev *dev,
				      unsigned selector)
{
	struct fpf2495_data *data = rdev_get_drvdata(dev);

	if (!(data->desc.n_voltages > selector))
		return -EINVAL;

	return data->microvolts;
}

static const char *fpf2495_oc_det_state_to_string(int status)
{
	const char *ret;

	switch (status) {
	case OC_DET_STOP:
		ret = "STOP  ";
		break;
	case OC_DET_READY:
		ret = "READY ";
		break;
	case OC_DET_START:
		ret = "START ";
		break;
	case OC_DET_SETTLE:
		ret = "SETTLE";
		break;
	default:
		ret = "UNKNOWN";
		break;
	}

	return ret;
}

static void fpf2495_oc_notify(struct fpf2495_data *data)
{
	unsigned long flags;

	if (unlikely(OC_DET_START != atomic_cmpxchg(&data->oc_det_state,
				OC_DET_START, OC_DET_SETTLE))) {
		dev_warn(data->dev,
				"%s: strange status expected=%s, but now=%s\n",
				__func__,
				fpf2495_oc_det_state_to_string(OC_DET_START),
				fpf2495_oc_det_state_to_string(
					atomic_read(&data->oc_det_state)));
		return;
	}

	spin_lock_irqsave(&data->ocp_lock, flags);
	if (data->ocp_notification.notify)
		data->ocp_notification.notify(data->ocp_notification.ctxt);
	spin_unlock_irqrestore(&data->ocp_lock, flags);

	return;
}

static void fpf2495_oc_delay_work(struct work_struct *work)
{
	struct fpf2495_data *data = container_of(work, struct fpf2495_data,
							oc_delay_work.work);
	int ret;

	if (unlikely(OC_DET_START != atomic_read(&data->oc_det_state))) {
		dev_warn(data->dev,
				"%s: strange status expected=%s, but now=%s\n",
				__func__,
				fpf2495_oc_det_state_to_string(OC_DET_START),
				fpf2495_oc_det_state_to_string(
					atomic_read(&data->oc_det_state)));
		return;
	}

	/* check whether FLAG pin return to inactive */
	ret = gpio_get_value_cansleep(data->gpio_flag.gpio);
	if (IS_ERR_VALUE(ret)) {
		dev_err(data->dev,
			"%s: failed to read FLAG pin ret=%d, assume OC\n",
								__func__, ret);
		fpf2495_oc_notify(data);
	} else if (data->gpio_flag.active_high ? ret : !ret) {
		dev_info(data->dev, "%s: FLAG active, settle OC\n",
								__func__);
		fpf2495_oc_notify(data);
	} else {
		dev_info(data->dev, "%s: FLAG inactive, return from OC\n",
								__func__);
		if (unlikely(OC_DET_START != atomic_cmpxchg(&data->oc_det_state,
						OC_DET_START, OC_DET_READY)))
			dev_warn(data->dev,
				"%s: strange status expected=%s, but now=%s\n",
				__func__,
				fpf2495_oc_det_state_to_string(OC_DET_START),
				fpf2495_oc_det_state_to_string(
					atomic_read(&data->oc_det_state)));
		else
			enable_irq(data->flag_irq);
	}

	return;
}

static irqreturn_t fpf2495_flag_irq(int irq, void *val)
{
	struct fpf2495_data *data = (struct fpf2495_data *)val;
	int ret;

	dev_info(data->dev, "%s: irq=%d received FLAG\n", __func__, irq);

	disable_irq_nosync(irq);

	if (unlikely(OC_DET_READY != atomic_cmpxchg(&data->oc_det_state,
						OC_DET_READY, OC_DET_START))) {
		dev_warn(data->dev,
				"%s: strange status expected=%s, but now=%s\n",
				__func__,
				fpf2495_oc_det_state_to_string(OC_DET_READY),
				fpf2495_oc_det_state_to_string(
					atomic_read(&data->oc_det_state)));
		return IRQ_HANDLED;
	}

	ret = schedule_delayed_work(&data->oc_delay_work,
			msecs_to_jiffies(atomic_read(&data->oc_delay)));
	if (unlikely(ret < 0)) {
		dev_err(data->dev, "%s: failed to start the timer\n", __func__);
		fpf2495_oc_notify(data);
	}

	return IRQ_HANDLED;
}

static int fpf2495_ctrl(struct fpf2495_data *data, int enable)
{
	/* regulator vin is controled by regulator core. */

	if (!data->gpio_en.gpio) {
		dev_err(data->dev, "%s: en is not defined\n", __func__);
		return -EINVAL;
	}

	if (data->gpio_en.status == !!enable)
		return 0;

	gpio_set_value_cansleep(data->gpio_en.gpio, enable ?
			data->gpio_en.active_high : !data->gpio_en.active_high);

	data->gpio_en.status = !!enable;

	return 0;
}

static int fpf2495_enable(struct regulator_dev *dev)
{
	struct fpf2495_data *data = rdev_get_drvdata(dev);
	int ret;
	int cnt;

	mutex_lock(&data->mutex);

	/* if already output the vout, do nothing. */
	if (data->vout_is_on) {
		ret = 0;
		dev_dbg(data->dev, "%s: to output the vout, but already on\n",
								__func__);
		goto op_done;
	}

	/* pull-up ocpflag pin */
	if (data->flag_pull) {
		ret = regulator_enable(data->flag_pull);
		if (ret)
			dev_warn(data->dev, "%s: faild flag pull up\n",
								__func__);
	}

	/* wait until pulled-up */
	cnt = FPF2495_FLAG_READY_WAIT;
	do {
		usleep(1000);
		ret = gpio_get_value_cansleep(data->gpio_flag.gpio);
		if (IS_ERR_VALUE(ret)) {
			dev_err(data->dev, "%s: faild flag read\n", __func__);
			goto op_flagpullerr;
		}
	} while (--cnt && (data->gpio_flag.active_high ? ret : !ret));

	/* it thought something trouble if ocpflag is not pulled-up */
	if (data->gpio_flag.active_high ? ret : !ret) {
		ret = -EIO;
		dev_err(data->dev, "%s: flag indicates OCP.\n", __func__);
		goto op_flagpullerr;
	}

	atomic_set(&data->oc_det_state, OC_DET_READY);
	enable_irq(data->flag_irq);

	/* output the vout */
	ret = fpf2495_ctrl(data, 1);
	if (ret) {
		dev_err(data->dev, "%s: fail output the vout\n", __func__);
		goto op_ctrlerr;
	}

	/* all the process to output the vout was completed. */
	data->vout_is_on = 1;

	mutex_unlock(&data->mutex);
	return 0;

op_ctrlerr:
	fpf2495_ctrl(data, 0);
op_flagpullerr:
	if (data->flag_pull)
		regulator_disable(data->flag_pull);
op_done:
	mutex_unlock(&data->mutex);
	return ret;
}

static int fpf2495_disable(struct regulator_dev *dev)
{
	struct fpf2495_data *data = rdev_get_drvdata(dev);
	int ret;

	mutex_lock(&data->mutex);
	/* if already disabled, do nothing. */
	if (!data->vout_is_on) {
		dev_dbg(data->dev, "%s: to disable the vout, but already off\n",
								__func__);
		goto op_done;
	}
	if (OC_DET_READY == atomic_xchg(&data->oc_det_state, OC_DET_STOP))
		disable_irq(data->flag_irq);
	cancel_delayed_work_sync(&data->oc_delay_work);
	ret = fpf2495_ctrl(data, 0);
	data->vout_is_on = 0;
	if (data->flag_pull)
		regulator_disable(data->flag_pull);
op_done:
	mutex_unlock(&data->mutex);

	return ret;
}

static int fpf2495_is_enabled(struct regulator_dev *dev)
{
	struct fpf2495_data *data = rdev_get_drvdata(dev);
	return data->vout_is_on;
}

static int fpf2495_register_ocp_notification(
				struct regulator_dev *rdev,
				struct regulator_ocp_notification *notification)
{
	struct fpf2495_data *data = rdev_get_drvdata(rdev);
	unsigned long flags;

	spin_lock_irqsave(&data->ocp_lock, flags);
	if (notification)
		/* register ocp notification */
		data->ocp_notification = *notification;
	else
		/* unregister ocp notification */
		memset(&data->ocp_notification, 0,
						sizeof(data->ocp_notification));
	spin_unlock_irqrestore(&data->ocp_lock, flags);

	dev_dbg(data->dev,
		"%s: registered ocp notification(notify=%p, ctxt=%p)\n",
				rdev_get_name(data->rdev),
				data->ocp_notification.notify,
				data->ocp_notification.ctxt);

	return 0;
}

static struct regulator_ops fpf2495_ops = {
	.set_voltage_sel	= fpf2495_set_voltage_sel,
	.get_voltage		= fpf2495_get_voltage,
	.list_voltage		= fpf2495_list_voltage,
	.enable		= fpf2495_enable,
	.disable		= fpf2495_disable,
	.is_enabled		= fpf2495_is_enabled,
	.register_ocp_notification = fpf2495_register_ocp_notification,
};

static int fpf2495_probe(struct platform_device *pdev)
{
	struct fpf2495_config *config;
	struct fpf2495_data *drvdata;
	struct regulator_config cfg = { };
	int ret;
	unsigned long flags;

	if (pdev->dev.of_node) {
		config = of_get_fpf2495_config(&pdev->dev);
		if (IS_ERR(config))
			return PTR_ERR(config);
	} else {
		config = pdev->dev.platform_data;
	}

	if (!config)
		return -ENOMEM;

	drvdata = devm_kzalloc(&pdev->dev, sizeof(struct fpf2495_data),
			       GFP_KERNEL);
	if (drvdata == NULL) {
		dev_err(&pdev->dev, "Failed to allocate device data\n");
		return -ENOMEM;
	}

	drvdata->vout_is_on = 0;
	mutex_init(&drvdata->mutex);
	INIT_DELAYED_WORK(&drvdata->oc_delay_work, fpf2495_oc_delay_work);

	drvdata->desc.name = kstrdup(config->supply_name, GFP_KERNEL);
	if (drvdata->desc.name == NULL) {
		dev_err(&pdev->dev, "Failed to allocate supply name\n");
		ret = -ENOMEM;
		goto err_supply;
	}
	drvdata->desc.type = REGULATOR_VOLTAGE;
	drvdata->desc.owner = THIS_MODULE;
	drvdata->desc.ops = &fpf2495_ops;

	drvdata->desc.enable_time = config->startup_delay;

	if (config->input_supply) {
		drvdata->desc.supply_name = kstrdup(config->input_supply,
							GFP_KERNEL);
		if (!drvdata->desc.supply_name) {
			dev_err(&pdev->dev,
				"Failed to allocate input supply\n");
			ret = -ENOMEM;
			goto err_input_supply;
		}
	}

	drvdata->desc.n_voltages = 1;

	drvdata->microvolts = FPF2495_DEFAULT_MICROVOLTS;
	if (config->microvolts)
		drvdata->microvolts = config->microvolts;

	cfg.dev = &pdev->dev;
	cfg.init_data = config->init_data;
	cfg.driver_data = drvdata;
	cfg.of_node = pdev->dev.of_node;

	config->init_data->constraints.valid_ops_mask
						|= REGULATOR_CHANGE_STATUS;

	drvdata->rdev = regulator_register(&drvdata->desc, &cfg);
	if (IS_ERR(drvdata->rdev)) {
		ret = PTR_ERR(drvdata->rdev);
		dev_err(&pdev->dev, "Failed to register regulator: %d\n", ret);
		goto err_reg_regi;
	}

	/* register gpio for en */
	if (config->gpio_en.gpio < 0) {
		ret = -ENODEV;
		dev_err(&pdev->dev, "gpio for detet flag is undefined\n");
		goto err_gpio_en;
	}

	if (config->gpio_en.active_high)
		flags = GPIOF_OUT_INIT_LOW;
	else
		flags = GPIOF_OUT_INIT_HIGH;

	ret = devm_gpio_request_one(&pdev->dev, config->gpio_en.gpio, flags,
						rdev_get_name(drvdata->rdev));
	if (ret) {
		dev_err(&pdev->dev, "unable to get gpio for en\n");
		goto err_gpio_en;
	}

	drvdata->gpio_en.gpio = config->gpio_en.gpio;
	drvdata->gpio_en.status = 0;
	drvdata->gpio_en.flags = flags;
	drvdata->gpio_en.active_high = config->gpio_en.active_high;

	/* register gpio for ocpflag */
	if (config->gpio_flag.gpio < 0) {
		ret = -ENODEV;
		dev_err(&pdev->dev, "gpio for detet flag is undefined\n");
		goto err_gpio_flag;
	}

	flags = GPIOF_IN;
	ret = devm_gpio_request_one(&pdev->dev, config->gpio_flag.gpio, flags,
						rdev_get_name(drvdata->rdev));
	if (ret) {
		dev_err(&pdev->dev, "unable to get gpio for detet flag\n");
		goto err_gpio_flag;
	}

	drvdata->gpio_flag.gpio = config->gpio_flag.gpio;
	drvdata->gpio_flag.status = 0;
	drvdata->gpio_flag.flags = flags;
	drvdata->gpio_flag.active_high = config->gpio_flag.active_high;

	/* get regulator for ocpflag pull-up*/
	if (config->flag_pull_supply) {
		drvdata->flag_pull = devm_regulator_get(&pdev->dev,
						config->flag_pull_supply);
		if (IS_ERR(drvdata->flag_pull))
			dev_err(&pdev->dev,
				"unable to get regulator flag pull-up\n");
	}

	atomic_set(&drvdata->oc_delay, FPF2495_OC_DELAY);
	if (config->oc_delay)
		atomic_set(&drvdata->oc_delay, config->oc_delay);
	atomic_set(&drvdata->oc_det_state, OC_DET_STOP);
	memset(&drvdata->ocp_notification, 0,
					sizeof(drvdata->ocp_notification));
	spin_lock_init(&drvdata->ocp_lock);

	drvdata->flag_irq = gpio_to_irq(config->gpio_flag.gpio);
	if (IS_ERR_VALUE(drvdata->flag_irq)) {
		dev_err(&pdev->dev, "flag_irq is not defined\n");
		goto err_flag_irq;
	}

	ret = devm_request_irq(&pdev->dev, drvdata->flag_irq, fpf2495_flag_irq,
						drvdata->gpio_flag.active_high ?
							IRQF_TRIGGER_HIGH :
							IRQF_TRIGGER_LOW,
						"flag_irq", drvdata);
	if (ret) {
		dev_err(&pdev->dev, "request flag_irq=%d failed\n",
							drvdata->flag_irq);
		goto err_flag_irq;
	}
	disable_irq(drvdata->flag_irq);

	platform_set_drvdata(pdev, drvdata);
	drvdata->dev = &pdev->dev;

	dev_dbg(&pdev->dev, "%s supplying %duV\n", drvdata->desc.name,
							drvdata->microvolts);

	return 0;

err_flag_irq:
	if (drvdata->flag_pull)
		devm_regulator_put(drvdata->flag_pull);
	devm_gpio_free(&pdev->dev, drvdata->gpio_flag.gpio);
err_gpio_flag:
	devm_gpio_free(&pdev->dev, drvdata->gpio_en.gpio);
err_gpio_en:
	regulator_unregister(drvdata->rdev);
err_reg_regi:
	kfree(drvdata->desc.supply_name);
err_input_supply:
	kfree(drvdata->desc.name);
err_supply:
	mutex_destroy(&drvdata->mutex);
	return ret;
}

static int fpf2495_remove(struct platform_device *pdev)
{
	struct fpf2495_data *drvdata = platform_get_drvdata(pdev);

	fpf2495_disable(drvdata->rdev);
	devm_free_irq(&pdev->dev, drvdata->flag_irq, drvdata);
	if (drvdata->flag_pull)
		devm_regulator_put(drvdata->flag_pull);
	devm_gpio_free(&pdev->dev, drvdata->gpio_flag.gpio);
	devm_gpio_free(&pdev->dev, drvdata->gpio_en.gpio);
	regulator_unregister(drvdata->rdev);
	kfree(drvdata->desc.supply_name);
	kfree(drvdata->desc.name);
	mutex_destroy(&drvdata->mutex);

	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id fpf2495_of_match[] = {
	{ .compatible = "fairchild,fpf2495-regulator", },
	{},
};
MODULE_DEVICE_TABLE(of, fpf2495_of_match);
#endif

static struct platform_driver fpf2495_driver = {
	.probe		= fpf2495_probe,
	.remove		= fpf2495_remove,
	.driver		= {
		.name		= "fpf2495",
		.owner		= THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(fpf2495_of_match),
#endif
	},
};

static int __init fpf2495_init(void)
{
	return platform_driver_register(&fpf2495_driver);
}
subsys_initcall(fpf2495_init);

static void __exit fpf2495_exit(void)
{
	platform_driver_unregister(&fpf2495_driver);
}
module_exit(fpf2495_exit);

MODULE_DESCRIPTION("regulator driver for fairchild fpf2495");
MODULE_LICENSE("GPLV2");
