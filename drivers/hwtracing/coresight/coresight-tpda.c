// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/amba/bus.h>
#include <linux/bitfield.h>
#include <linux/coresight.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "coresight-priv.h"
#include "coresight-tpda.h"
#include "coresight-trace-id.h"
#include "coresight-tpdm.h"

DEFINE_CORESIGHT_DEVLIST(tpda_devs, "tpda");

static bool coresight_device_is_tpdm(struct coresight_device *csdev)
{
	return (coresight_is_device_source(csdev)) &&
	       (csdev->subtype.source_subtype ==
			CORESIGHT_DEV_SUBTYPE_SOURCE_TPDM);
}

static bool is_static_tpdm(struct coresight_device *csdev)
{
	const char *compatible;
	bool ret = false;

	if (!fwnode_property_read_string(dev_fwnode(csdev->dev.parent),
		"compatible", &compatible)) {
		if (!strcmp(compatible, "qcom,coresight-static-tpdm"))
			ret = true;
	}
	return ret;
}

static void tpda_clear_element_size(struct coresight_device *csdev)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(csdev->dev.parent);

	drvdata->dsb_esize = 0;
	drvdata->cmb_esize = 0;
}

static void tpda_set_element_size(struct tpda_drvdata *drvdata, u32 *val)
{
	/* Clear all relevant fields */
	*val &= ~(TPDA_Pn_CR_DSBSIZE | TPDA_Pn_CR_CMBSIZE);

	if (drvdata->dsb_esize == 64)
		*val |= TPDA_Pn_CR_DSBSIZE;
	else if (drvdata->dsb_esize == 32)
		*val &= ~TPDA_Pn_CR_DSBSIZE;

	if (drvdata->cmb_esize == 64)
		*val |= FIELD_PREP(TPDA_Pn_CR_CMBSIZE, 0x2);
	else if (drvdata->cmb_esize == 32)
		*val |= FIELD_PREP(TPDA_Pn_CR_CMBSIZE, 0x1);
	else if (drvdata->cmb_esize == 8)
		*val &= ~TPDA_Pn_CR_CMBSIZE;
}

/*
 * Read the element size from the TPDM device. One TPDM must have at least one of the
 * element size property.
 * Returns
 *    0 - The element size property is read
 *    Others - Cannot read the property of the element size
 */
static int tpdm_read_element_size(struct tpda_drvdata *drvdata,
				  struct coresight_device *csdev)
{
	int rc = -EINVAL;
	struct tpdm_drvdata *tpdm_data = dev_get_drvdata(csdev->dev.parent);

	if (tpdm_has_dsb_dataset(tpdm_data)) {
		rc = fwnode_property_read_u32(dev_fwnode(csdev->dev.parent),
				"qcom,dsb-element-bits", &drvdata->dsb_esize);
	}

	if (tpdm_has_cmb_dataset(tpdm_data) || tpdm_has_mcmb_dataset(tpdm_data)) {
		rc = fwnode_property_read_u32(dev_fwnode(csdev->dev.parent),
				"qcom,cmb-element-bits", &drvdata->cmb_esize);
	}

	if (is_static_tpdm(csdev)) {
		fwnode_property_read_u32(dev_fwnode(csdev->dev.parent),
				"qcom,dsb-element-bits", &drvdata->dsb_esize);
		fwnode_property_read_u32(dev_fwnode(csdev->dev.parent),
				"qcom,cmb-element-bits", &drvdata->cmb_esize);
		if (drvdata->dsb_esize || drvdata->cmb_esize)
			rc = 0;
	}

	if (rc)
		dev_warn_once(&csdev->dev,
			"Failed to read TPDM Element size: %d\n", rc);

	return rc;
}

/*
 * Search and read element data size from the TPDM node in
 * the devicetree. Each input port of TPDA is connected to
 * a TPDM. Different TPDM supports different types of dataset,
 * and some may support more than one type of dataset.
 * Parameter "inport" is used to pass in the input port number
 * of TPDA, and it is set to -1 in the recursize call.
 */
static int tpda_get_element_size(struct tpda_drvdata *drvdata,
				 struct coresight_device *csdev,
				 int inport)
{
	int rc = 0;
	int i;
	struct coresight_device *in;

	for (i = 0; i < csdev->pdata->nr_inconns; i++) {
		in = csdev->pdata->in_conns[i]->src_dev;
		if (!in)
			continue;

		/* Ignore the paths that do not match port */
		if (inport >= 0 &&
		    csdev->pdata->in_conns[i]->dest_port != inport)
			continue;

		/*
		 * If this port has a hardcoded filter, use the source
		 * device directly.
		 */
		if (csdev->pdata->in_conns[i]->filter_src_fwnode) {
			in = csdev->pdata->in_conns[i]->filter_src_dev;
			if (!in)
				continue;
		}

		if (coresight_device_is_tpdm(in)) {
			if (drvdata->dsb_esize || drvdata->cmb_esize)
				return -EEXIST;
			rc = tpdm_read_element_size(drvdata, in);
			if (rc)
				return rc;
		} else {
			/* Recurse down the path */
			rc = tpda_get_element_size(drvdata, in, -1);
			if (rc)
				return rc;
		}
	}

	return rc;
}

/* Settings pre enabling port control register */
static void tpda_enable_pre_port(struct tpda_drvdata *drvdata)
{
	u32 val;

	val = readl_relaxed(drvdata->base + TPDA_CR);
	val &= ~TPDA_CR_MID;
	val &= ~TPDA_CR_ATID;
	val |= FIELD_PREP(TPDA_CR_ATID, drvdata->atid);
	if (drvdata->trig_async)
		val = val | TPDA_CR_SRIE;
	else
		val = val & ~TPDA_CR_SRIE;
	if (drvdata->trig_flag_ts)
		val = val | TPDA_CR_FLRIE;
	else
		val = val & ~TPDA_CR_FLRIE;
	if (drvdata->trig_freq)
		val = val | TPDA_CR_FRIE;
	else
		val = val & ~TPDA_CR_FRIE;
	if (drvdata->freq_ts)
		val = val | TPDA_CR_FREQTS;
	else
		val = val & ~TPDA_CR_FREQTS;
	if (drvdata->cmbchan_mode)
		val = val | TPDA_CR_CMBCHANMODE;
	else
		val = val & ~TPDA_CR_CMBCHANMODE;
	writel_relaxed(val, drvdata->base + TPDA_CR);

	/*
	 * If FLRIE bit is set, set the master and channel
	 * id as zero
	 */
	if (drvdata->trig_flag_ts)
		writel_relaxed(0x0, drvdata->base + TPDA_FPID_CR);
}

static int tpda_enable_port(struct tpda_drvdata *drvdata, int port)
{
	u32 val;
	int rc;

	val = readl_relaxed(drvdata->base + TPDA_Pn_CR(port));
	tpda_clear_element_size(drvdata->csdev);
	rc = tpda_get_element_size(drvdata, drvdata->csdev, port);
	if ((!rc && (drvdata->dsb_esize || drvdata->cmb_esize))) {
		tpda_set_element_size(drvdata, &val);
		/* Enable the port */
		val |= TPDA_Pn_CR_ENA;
		writel_relaxed(val, drvdata->base + TPDA_Pn_CR(port));
	} else if (rc == -EEXIST)
		dev_warn_once(&drvdata->csdev->dev,
			      "Detected multiple TPDMs on port %d", port);
	else
		dev_warn_once(&drvdata->csdev->dev,
			      "Didn't find TPDM element size");

	return rc;
}

static void tpda_enable_post_port(struct tpda_drvdata *drvdata)
{
	uint32_t val;

	val = readl_relaxed(drvdata->base + TPDA_SYNCR);
	/* Clear the mode */
	val = val & ~TPDA_MODE_CTRL;
	/* Program the counter value */
	val = val | 0xFFF;
	writel_relaxed(val, drvdata->base + TPDA_SYNCR);
}

static int __tpda_enable(struct tpda_drvdata *drvdata, int port)
{
	int ret;

	CS_UNLOCK(drvdata->base);

	/*
	 * Only do pre-port enable for first port that calls enable when the
	 * device's main refcount is still 0
	 */
	lockdep_assert_held(&drvdata->spinlock);
	if (!drvdata->csdev->refcnt)
		tpda_enable_pre_port(drvdata);

	ret = tpda_enable_port(drvdata, port);
	CS_LOCK(drvdata->base);

	if (!drvdata->csdev->refcnt)
		tpda_enable_post_port(drvdata);

	return ret;
}

static int tpda_alloc_trace_id(struct coresight_device *csdev)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(csdev->dev.parent);
	int trace_id;
	int i, nr_conns;

	nr_conns = csdev->pdata->nr_inconns;

	for (i = 0; i < nr_conns; i++)
		if (atomic_read(&csdev->pdata->in_conns[i]->dest_refcnt) != 0)
			return 0;

	trace_id = coresight_trace_id_get_system_id();
	if (trace_id < 0)
		return trace_id;

	drvdata->atid = trace_id;

	return 0;
}

static void tpda_release_trace_id(struct coresight_device *csdev)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(csdev->dev.parent);
	int i, nr_conns;

	nr_conns = csdev->pdata->nr_inconns;

	for (i = 0; i < nr_conns; i++)
		if (atomic_read(&csdev->pdata->in_conns[i]->dest_refcnt) != 0)
			return;

	coresight_trace_id_put_system_id(drvdata->atid);

	drvdata->atid = 0;
}

static int tpda_enable(struct coresight_device *csdev,
		       struct coresight_connection *in,
		       struct coresight_connection *out)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(csdev->dev.parent);
	int ret = 0;

	spin_lock(&drvdata->spinlock);

	ret = tpda_alloc_trace_id(csdev);
	if (ret < 0) {
		spin_unlock(&drvdata->spinlock);
		return ret;
	}

	if (atomic_read(&in->dest_refcnt) == 0) {
		ret = __tpda_enable(drvdata, in->dest_port);
		if (!ret) {
			atomic_inc(&in->dest_refcnt);
			csdev->refcnt++;
			dev_dbg(drvdata->dev, "TPDA inport %d enabled.\n", in->dest_port);
		}
	}

	spin_unlock(&drvdata->spinlock);
	return ret;
}

static void __tpda_disable(struct tpda_drvdata *drvdata, int port)
{
	u32 val;

	CS_UNLOCK(drvdata->base);

	val = readl_relaxed(drvdata->base + TPDA_Pn_CR(port));
	val &= ~TPDA_Pn_CR_ENA;
	writel_relaxed(val, drvdata->base + TPDA_Pn_CR(port));

	CS_LOCK(drvdata->base);
}

static void tpda_disable(struct coresight_device *csdev,
			 struct coresight_connection *in,
			 struct coresight_connection *out)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(csdev->dev.parent);

	spin_lock(&drvdata->spinlock);
	if (atomic_dec_return(&in->dest_refcnt) == 0) {
		__tpda_disable(drvdata, in->dest_port);
		csdev->refcnt--;
	}
	tpda_release_trace_id(csdev);
	spin_unlock(&drvdata->spinlock);

	dev_dbg(drvdata->dev, "TPDA inport %d disabled\n", in->dest_port);
}

static const struct coresight_ops_link tpda_link_ops = {
	.enable		= tpda_enable,
	.disable	= tpda_disable,
};

static const struct coresight_ops tpda_cs_ops = {
	.link_ops	= &tpda_link_ops,
};

static ssize_t trig_async_enable_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(dev->parent);

	return sysfs_emit(buf, "%u\n", (unsigned int)drvdata->trig_async);
}

static ssize_t trig_async_enable_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf,
				       size_t size)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	guard(spinlock)(&drvdata->spinlock);
	if (val)
		drvdata->trig_async = true;
	else
		drvdata->trig_async = false;

	return size;
}
static DEVICE_ATTR_RW(trig_async_enable);

static ssize_t trig_flag_ts_enable_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(dev->parent);

	return sysfs_emit(buf, "%u\n", (unsigned int)drvdata->trig_flag_ts);
}

static ssize_t trig_flag_ts_enable_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t size)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	guard(spinlock)(&drvdata->spinlock);
	if (val)
		drvdata->trig_flag_ts = true;
	else
		drvdata->trig_flag_ts = false;

	return size;
}
static DEVICE_ATTR_RW(trig_flag_ts_enable);

static ssize_t trig_freq_enable_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(dev->parent);

	return sysfs_emit(buf, "%u\n", (unsigned int)drvdata->trig_freq);
}

static ssize_t trig_freq_enable_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t size)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	guard(spinlock)(&drvdata->spinlock);
	if (val)
		drvdata->trig_freq = true;
	else
		drvdata->trig_freq = false;

	return size;
}
static DEVICE_ATTR_RW(trig_freq_enable);

static ssize_t freq_ts_enable_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(dev->parent);

	return sysfs_emit(buf, "%u\n", (unsigned int)drvdata->freq_ts);
}

static ssize_t freq_ts_enable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t size)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	guard(spinlock)(&drvdata->spinlock);
	if (val)
		drvdata->freq_ts = true;
	else
		drvdata->freq_ts = false;

	return size;
}
static DEVICE_ATTR_RW(freq_ts_enable);

static ssize_t freq_req_val_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val = drvdata->freq_req_val;

	return sysfs_emit(buf, "%#lx\n", val);
}

static ssize_t freq_req_val_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t size)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	guard(spinlock)(&drvdata->spinlock);
	drvdata->freq_req_val = val;

	return size;
}
static DEVICE_ATTR_RW(freq_req_val);

static ssize_t freq_req_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(dev->parent);

	return sysfs_emit(buf, "%u\n", (unsigned int)drvdata->freq_req);
}

static ssize_t freq_req_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t size)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	guard(spinlock)(&drvdata->spinlock);
	if (val)
		drvdata->freq_req = true;
	else
		drvdata->freq_req = false;

	return size;
}
static DEVICE_ATTR_RW(freq_req);

static ssize_t global_flush_req_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val;

	guard(spinlock)(&drvdata->spinlock);
	if (!drvdata->csdev->refcnt)
		return -EPERM;

	val = readl_relaxed(drvdata->base + TPDA_CR);
	return sysfs_emit(buf, "%lx\n", val);
}

static ssize_t global_flush_req_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t size)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	guard(spinlock)(&drvdata->spinlock);
	if (!drvdata->csdev->refcnt)
		return -EPERM;

	if (val) {
		CS_UNLOCK(drvdata->base);
		val = readl_relaxed(drvdata->base + TPDA_CR);
		val = val | BIT(0);
		writel_relaxed(val, drvdata->base + TPDA_CR);
		CS_LOCK(drvdata->base);
	}

	return size;
}
static DEVICE_ATTR_RW(global_flush_req);

static ssize_t port_flush_req_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val;

	guard(spinlock)(&drvdata->spinlock);
	if (!drvdata->csdev->refcnt)
		return -EPERM;

	val = readl_relaxed(drvdata->base + TPDA_FLUSH_CR);
	return sysfs_emit(buf, "%lx\n", val);
}

static ssize_t port_flush_req_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t size)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	guard(spinlock)(&drvdata->spinlock);
	if (!drvdata->csdev->refcnt)
		return -EPERM;

	if (val) {
		CS_UNLOCK(drvdata->base);
		writel_relaxed(val, drvdata->base + TPDA_FLUSH_CR);
		CS_LOCK(drvdata->base);
	}

	return size;
}
static DEVICE_ATTR_RW(port_flush_req);

static ssize_t cmbchan_mode_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(dev->parent);

	return sysfs_emit(buf, "%u\n", (unsigned int)drvdata->cmbchan_mode);
}

static ssize_t cmbchan_mode_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t size)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(dev->parent);
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	guard(spinlock)(&drvdata->spinlock);
	if (val)
		drvdata->cmbchan_mode = true;
	else
		drvdata->cmbchan_mode = false;

	return size;
}
static DEVICE_ATTR_RW(cmbchan_mode);

static struct attribute *tpda_attrs[] = {
	&dev_attr_trig_async_enable.attr,
	&dev_attr_trig_flag_ts_enable.attr,
	&dev_attr_trig_freq_enable.attr,
	&dev_attr_freq_ts_enable.attr,
	&dev_attr_freq_req_val.attr,
	&dev_attr_freq_req.attr,
	&dev_attr_global_flush_req.attr,
	&dev_attr_port_flush_req.attr,
	&dev_attr_cmbchan_mode.attr,
	NULL,
};

static struct attribute_group tpda_attr_grp = {
	.attrs = tpda_attrs,
};

static const struct attribute_group *tpda_attr_grps[] = {
	&tpda_attr_grp,
	NULL,
};

static void tpda_init_default_data(struct tpda_drvdata *drvdata)
{
	drvdata->freq_ts = true;
}

static int tpda_probe(struct amba_device *adev, const struct amba_id *id)
{
	int ret;
	struct device *dev = &adev->dev;
	struct coresight_platform_data *pdata;
	struct tpda_drvdata *drvdata;
	struct coresight_desc desc = { 0 };
	void __iomem *base;

	pdata = coresight_get_platform_data(dev);
	if (IS_ERR(pdata))
		return PTR_ERR(pdata);
	adev->dev.platform_data = pdata;

	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->dev = &adev->dev;
	dev_set_drvdata(dev, drvdata);

	drvdata->atclk = devm_clk_get_optional_enabled(dev, "atclk"); /* optional */
	if (IS_ERR(drvdata->atclk)) {
		ret = PTR_ERR(drvdata->atclk);
		dev_err(dev, "enable/get atclk fail, ret = %d\n", ret);
		return  ret == -ETIMEDOUT ? -EPROBE_DEFER : ret;
	}

	base = devm_ioremap_resource(dev, &adev->res);
	if (IS_ERR(base))
		return PTR_ERR(base);
	drvdata->base = base;

	spin_lock_init(&drvdata->spinlock);

	tpda_init_default_data(drvdata);

	desc.name = coresight_alloc_device_name(&tpda_devs, dev);
	if (!desc.name)
		return -ENOMEM;
	desc.type = CORESIGHT_DEV_TYPE_LINK;
	desc.subtype.link_subtype = CORESIGHT_DEV_SUBTYPE_LINK_MERG;
	desc.ops = &tpda_cs_ops;
	desc.pdata = adev->dev.platform_data;
	desc.dev = &adev->dev;
	desc.groups = tpda_attr_grps;
	desc.access = CSDEV_ACCESS_IOMEM(base);
	drvdata->csdev = coresight_register(&desc);
	if (IS_ERR(drvdata->csdev))
		return PTR_ERR(drvdata->csdev);

	pm_runtime_put_sync(&adev->dev);
	dev_dbg(drvdata->dev, "TPDA initialized\n");
	return 0;
}

#ifdef CONFIG_PM
static int tpda_runtime_suspend(struct device *dev)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(dev);

	if (drvdata && !IS_ERR(drvdata->atclk))
		clk_disable_unprepare(drvdata->atclk);

	return 0;
}

static int tpda_runtime_resume(struct device *dev)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(dev);

	if (drvdata && !IS_ERR(drvdata->atclk))
		clk_prepare_enable(drvdata->atclk);

	return 0;
}
#endif

static const struct dev_pm_ops tpda_dev_pm_ops = {
	SET_RUNTIME_PM_OPS(tpda_runtime_suspend,
			   tpda_runtime_resume, NULL)
};

static void tpda_remove(struct amba_device *adev)
{
	struct tpda_drvdata *drvdata = dev_get_drvdata(&adev->dev);

	coresight_trace_id_put_system_id(drvdata->atid);
	coresight_unregister(drvdata->csdev);
}

/*
 * Different TPDA has different periph id.
 * The difference is 0-7 bits' value. So ignore 0-7 bits.
 */
static struct amba_id tpda_ids[] = {
	{
		.id     = 0x000f0f00,
		.mask   = 0x000fff00,
	},
	{ 0, 0, NULL },
};

static struct amba_driver tpda_driver = {
	.drv = {
		.name   = "coresight-tpda",
		.pm = &tpda_dev_pm_ops,
		.suppress_bind_attrs = true,
	},
	.probe          = tpda_probe,
	.remove		= tpda_remove,
	.id_table	= tpda_ids,
};

module_amba_driver(tpda_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Trace, Profiling & Diagnostic Aggregator driver");
