// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2013-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/of.h>
#include <linux/of_platform.h>

#include "adreno.h"

#define TO_ADRENO_CORESIGHT_ATTR(_attr) \
	container_of(_attr, struct adreno_coresight_attr, attr)

ssize_t adreno_coresight_show_register(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct adreno_coresight_device *adreno_csdev = dev_get_drvdata(dev);
	struct adreno_coresight_attr *cattr = TO_ADRENO_CORESIGHT_ATTR(attr);
	struct kgsl_device *device = adreno_csdev->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	unsigned int val = 0;

	mutex_lock(&device->mutex);
	/*
	 * Return the current value of the register if coresight is enabled,
	 * otherwise report 0
	 */

	if (!adreno_csdev->enabled)
		goto out;

	if (!adreno_active_count_get(adreno_dev)) {
		kgsl_regread(device, cattr->reg->offset, &cattr->reg->value);
		adreno_active_count_put(adreno_dev);
	}
	val = cattr->reg->value;

out:
	mutex_unlock(&device->mutex);
	return scnprintf(buf, PAGE_SIZE, "0x%X\n", val);
}

ssize_t adreno_coresight_store_register(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{

	struct adreno_coresight_attr *cattr = TO_ADRENO_CORESIGHT_ATTR(attr);
	struct adreno_coresight_device *adreno_csdev = dev_get_drvdata(dev);
	struct kgsl_device *device = adreno_csdev->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return ret;

	mutex_lock(&device->mutex);

	/* Ignore writes while coresight is off */
	if (!adreno_csdev->enabled)
		goto out;

	cattr->reg->value = val;
	if (!adreno_active_count_get(adreno_dev)) {
		kgsl_regwrite(device, cattr->reg->offset, cattr->reg->value);
		adreno_active_count_put(adreno_dev);
	}

out:
	mutex_unlock(&device->mutex);
	return size;
}

/*
 * This is a generic function to disable coresight debug bus on Adreno
 * devices. This function in turn calls the device specific function
 * through the gpudev hook.
 */
static void adreno_coresight_disable(struct coresight_device *csdev,
					struct perf_event *event)
{
	struct adreno_coresight_device *adreno_csdev = dev_get_drvdata(&csdev->dev);
	struct kgsl_device *device = adreno_csdev->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	const struct adreno_coresight *coresight = adreno_csdev->coresight;
	int i;

	mutex_lock(&device->mutex);

	if (!adreno_csdev->enabled) {
		mutex_unlock(&device->mutex);
		return;
	}

	if (!adreno_active_count_get(adreno_dev)) {
		for (i = 0; i < coresight->count; i++)
			kgsl_regwrite(device,
				coresight->registers[i].offset, 0);
		adreno_active_count_put(adreno_dev);
	}

	adreno_csdev->enabled = false;

	mutex_unlock(&device->mutex);
}

static void _adreno_coresight_get_and_clear(struct adreno_device *adreno_dev,
		struct adreno_coresight_device *adreno_csdev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	const struct adreno_coresight *coresight = adreno_csdev->coresight;
	int i;

	if (IS_ERR_OR_NULL(adreno_csdev->dev) || !adreno_csdev->enabled)
		return;

	kgsl_pre_hwaccess(device);
	/*
	 * Save the current value of each coresight register
	 * and then clear each register
	 */
	for (i = 0; i < coresight->count; i++) {
		kgsl_regread(device, coresight->registers[i].offset,
			&coresight->registers[i].value);
		kgsl_regwrite(device, coresight->registers[i].offset, 0);
	}
}

static void _adreno_coresight_set(struct adreno_device *adreno_dev,
		struct adreno_coresight_device *adreno_csdev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	const struct adreno_coresight *coresight = adreno_csdev->coresight;
	int i;

	if (IS_ERR_OR_NULL(adreno_csdev->dev) || !adreno_csdev->enabled)
		return;

	for (i = 0; i < coresight->count; i++)
		kgsl_regwrite(device, coresight->registers[i].offset,
			coresight->registers[i].value);
}

/* Generic function to enable coresight debug bus on adreno devices */
static int adreno_coresight_enable(struct coresight_device *csdev,
				struct perf_event *event, u32 mode)
{
	struct adreno_coresight_device *adreno_csdev = dev_get_drvdata(&csdev->dev);
	const struct adreno_coresight *coresight = adreno_csdev->coresight;
	struct kgsl_device *device = adreno_csdev->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	int ret = 0;

	mutex_lock(&device->mutex);
	if (!adreno_csdev->enabled) {
		int i;

		adreno_csdev->enabled = true;

		/* Reset all the debug registers to their default values */
		for (i = 0; i < coresight->count; i++)
			coresight->registers[i].value =
				coresight->registers[i].initial;

		ret = adreno_active_count_get(adreno_dev);
		if (!ret) {
			_adreno_coresight_set(adreno_dev, adreno_csdev);
			adreno_active_count_put(adreno_dev);
		}

	}
	mutex_unlock(&device->mutex);
	return ret;
}

void adreno_coresight_stop(struct adreno_device *adreno_dev)
{
	_adreno_coresight_get_and_clear(adreno_dev, &adreno_dev->gx_coresight);
	_adreno_coresight_get_and_clear(adreno_dev, &adreno_dev->cx_coresight);
}

void adreno_coresight_start(struct adreno_device *adreno_dev)
{
	_adreno_coresight_set(adreno_dev, &adreno_dev->gx_coresight);
	_adreno_coresight_set(adreno_dev, &adreno_dev->cx_coresight);
}

static int adreno_coresight_trace_id(struct coresight_device *csdev)
{
	struct adreno_coresight_device *adreno_csdev = dev_get_drvdata(&csdev->dev);

	return adreno_csdev->atid;
}

static const struct coresight_ops_source adreno_coresight_source_ops = {
	.trace_id = adreno_coresight_trace_id,
	.enable = adreno_coresight_enable,
	.disable = adreno_coresight_disable,
};

static const struct coresight_ops adreno_coresight_ops = {
	.source_ops = &adreno_coresight_source_ops,
};

void adreno_coresight_remove(struct adreno_device *adreno_dev)
{
	if (!IS_ERR_OR_NULL(adreno_dev->gx_coresight.dev))
		coresight_unregister(adreno_dev->gx_coresight.dev);

	if (!IS_ERR_OR_NULL(adreno_dev->cx_coresight.dev))
		coresight_unregister(adreno_dev->cx_coresight.dev);
}

static int funnel_gfx_enable(struct coresight_device *csdev, int inport,
			 int outport)
{
	struct kgsl_device *device = kgsl_get_device(0);
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	int ret;

	if (!device)
		return -ENODEV;

	mutex_lock(&device->mutex);

	ret = adreno_active_count_get(adreno_dev);
	if (ret)
		goto err;

	/* Now that GPU is up, Call into coresight driver to enable funnel */
	ret = adreno_dev->funnel_gfx.funnel_ops->link_ops->enable(csdev, inport, outport);

	adreno_active_count_put(adreno_dev);
err:
	mutex_unlock(&device->mutex);
	return ret;
}

static void funnel_gfx_disable(struct coresight_device *csdev, int inport,
			   int outport)
{
	struct kgsl_device *device = kgsl_get_device(0);
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	int ret;

	if (!device)
		return;

	mutex_lock(&device->mutex);

	ret = adreno_active_count_get(adreno_dev);
	if (ret)
		goto err;

	/* Now that GPU is up, Call into coresight driver to disable funnel */
	adreno_dev->funnel_gfx.funnel_ops->link_ops->disable(csdev, inport, outport);

	adreno_active_count_put(adreno_dev);
err:
	mutex_unlock(&device->mutex);
	return;
}

struct coresight_ops_link funnel_link_gfx_ops = {
	.enable = funnel_gfx_enable,
	.disable = funnel_gfx_disable,
};

struct coresight_ops funnel_gfx_ops = {
	.link_ops = &funnel_link_gfx_ops,
};

static void adreno_coresight_dev_probe(struct kgsl_device *device,
		const struct adreno_coresight *coresight,
		struct adreno_coresight_device *adreno_csdev,
		struct device_node *node)
{
	struct platform_device *pdev = of_find_device_by_node(node);
	struct coresight_desc desc;
	u32 atid;

	if (!pdev)
		return;

	if (of_property_read_u32(node, "coresight-atid", &atid))
		return;

	if (of_property_read_string(node, "coresight-name", &desc.name))
		return;

	desc.pdata = coresight_get_platform_data(&pdev->dev);
	platform_device_put(pdev);

	if (IS_ERR(desc.pdata))
		return;

	desc.type = CORESIGHT_DEV_TYPE_SOURCE;
	desc.subtype.source_subtype = CORESIGHT_DEV_SUBTYPE_SOURCE_SOFTWARE;
	desc.ops = &adreno_coresight_ops;
	desc.dev = &device->pdev->dev;
	desc.groups = coresight->groups;

	adreno_csdev->atid = atid;
	adreno_csdev->device = device;
	adreno_csdev->dev = coresight_register(&desc);

	adreno_csdev->coresight = coresight;

	if (!IS_ERR_OR_NULL(adreno_csdev->dev))
		dev_set_drvdata(&adreno_csdev->dev->dev, adreno_csdev);
}

void adreno_coresight_add_device(struct adreno_device *adreno_dev, const char *name,
		const struct adreno_coresight *coresight,
		struct adreno_coresight_device *adreno_csdev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct device_node *node = of_find_compatible_node(device->pdev->dev.of_node, NULL, name);
	struct adreno_funnel_device *funnel_gfx = &adreno_dev->funnel_gfx;

	if (!node)
		return;

	/* Set the funnel ops as graphics ops to bring GPU up before enabling funnel */
	if (funnel_gfx !=NULL && funnel_gfx->funnel_csdev != NULL
						&& funnel_gfx->funnel_csdev->ops == NULL)
		funnel_gfx->funnel_csdev->ops = &funnel_gfx_ops;

	adreno_coresight_dev_probe(device, coresight, adreno_csdev, node);

	of_node_put(node);
}
