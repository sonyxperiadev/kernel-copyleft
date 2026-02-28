// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/iopoll.h>

#include "adreno.h"
#include "adreno_trace.h"
#include "kgsl_device.h"
#include "kgsl_gmu_core.h"
#include "kgsl_trace.h"

static const struct of_device_id gmu_match_table[] = {
	{ .compatible = "qcom,gpu-gmu", .data = &a6xx_gmu_driver },
	{ .compatible = "qcom,gpu-rgmu", .data = &a6xx_rgmu_driver },
	{ .compatible = "qcom,gen7-gmu", .data = &gen7_gmu_driver },
	{ .compatible = "qcom,gen8-gmu", .data = &gen8_gmu_driver },
	{},
};

void __init gmu_core_register(void)
{
	const struct of_device_id *match;
	struct device_node *node;

	node = of_find_matching_node_and_match(NULL, gmu_match_table,
		&match);
	if (!node)
		return;

	platform_driver_register((struct platform_driver *) match->data);
	of_node_put(node);
}

void gmu_core_unregister(void)
{
	const struct of_device_id *match;
	struct device_node *node;

	node = of_find_matching_node_and_match(NULL, gmu_match_table,
		&match);
	if (!node)
		return;

	platform_driver_unregister((struct platform_driver *) match->data);
	of_node_put(node);
}

bool gmu_core_isenabled(struct kgsl_device *device)
{
	return test_bit(GMU_ENABLED, &device->gmu_core.flags);
}

bool gmu_core_gpmu_isenabled(struct kgsl_device *device)
{
	return (device->gmu_core.dev_ops != NULL);
}

bool gmu_core_scales_bandwidth(struct kgsl_device *device)
{
	const struct gmu_dev_ops *ops = GMU_DEVICE_OPS(device);

	if (ops && ops->scales_bandwidth)
		return ops->scales_bandwidth(device);

	return false;
}

int gmu_core_dev_acd_set(struct kgsl_device *device, bool val)
{
	const struct gmu_dev_ops *ops = GMU_DEVICE_OPS(device);

	if (ops && ops->acd_set)
		return ops->acd_set(device, val);

	return -EINVAL;
}

void gmu_core_regread(struct kgsl_device *device, unsigned int offsetwords,
		unsigned int *value)
{
	u32 val = kgsl_regmap_read(&device->regmap, offsetwords);
	*value  = val;
}

void gmu_core_regwrite(struct kgsl_device *device, unsigned int offsetwords,
		unsigned int value)
{
	kgsl_regmap_write(&device->regmap, value, offsetwords);
}

void gmu_core_blkwrite(struct kgsl_device *device, unsigned int offsetwords,
		const void *buffer, size_t size)
{
	kgsl_regmap_bulk_write(&device->regmap, offsetwords,
		buffer, size >> 2);
}

void gmu_core_regrmw(struct kgsl_device *device,
		unsigned int offsetwords,
		unsigned int mask, unsigned int bits)
{
	kgsl_regmap_rmw(&device->regmap, offsetwords, mask, bits);
}

int gmu_core_dev_oob_set(struct kgsl_device *device, enum oob_request req)
{
	const struct gmu_dev_ops *ops = GMU_DEVICE_OPS(device);

	if (ops && ops->oob_set)
		return ops->oob_set(device, req);

	return 0;
}

void gmu_core_dev_oob_clear(struct kgsl_device *device, enum oob_request req)
{
	const struct gmu_dev_ops *ops = GMU_DEVICE_OPS(device);

	if (ops && ops->oob_clear)
		ops->oob_clear(device, req);
}

void gmu_core_dev_cooperative_reset(struct kgsl_device *device)
{

	const struct gmu_dev_ops *ops = GMU_DEVICE_OPS(device);

	if (ops && ops->cooperative_reset)
		ops->cooperative_reset(device);
}

int gmu_core_dev_ifpc_isenabled(struct kgsl_device *device)
{
	const struct gmu_dev_ops *ops = GMU_DEVICE_OPS(device);

	if (ops && ops->ifpc_isenabled)
		return ops->ifpc_isenabled(device);

	return 0;
}

int gmu_core_dev_ifpc_store(struct kgsl_device *device, unsigned int val)
{
	const struct gmu_dev_ops *ops = GMU_DEVICE_OPS(device);

	if (ops && ops->ifpc_store)
		return ops->ifpc_store(device, val);

	return -EINVAL;
}

int gmu_core_dev_wait_for_active_transition(struct kgsl_device *device)
{
	const struct gmu_dev_ops *ops = GMU_DEVICE_OPS(device);

	if (ops && ops->wait_for_active_transition)
		return ops->wait_for_active_transition(device);

	return 0;
}

void gmu_core_fault_snapshot(struct kgsl_device *device,
			enum gmu_fault_panic_policy gf_policy)
{
	const struct gmu_dev_ops *ops = GMU_DEVICE_OPS(device);

	/* Send NMI first to halt GMU and capture the state close to the point of failure */
	if (ops && ops->send_nmi)
		ops->send_nmi(device, false, gf_policy);

	kgsl_device_snapshot(device, NULL, NULL, true);
}

int gmu_core_timed_poll_check(struct kgsl_device *device,
		unsigned int offset, unsigned int expected_ret,
		unsigned int timeout_ms, unsigned int mask)
{
	u32 val;

	return kgsl_regmap_read_poll_timeout(&device->regmap, offset,
		val, (val & mask) == expected_ret, 100, timeout_ms * 1000);
}

int gmu_core_map_memdesc(struct iommu_domain *domain, struct kgsl_memdesc *memdesc,
		u64 gmuaddr, int attrs)
{
	size_t mapped;

	if (!memdesc->pages) {
		mapped = kgsl_mmu_map_sg(domain, gmuaddr, memdesc->sgt->sgl,
			memdesc->sgt->nents, attrs);
	} else {
		struct sg_table sgt = { 0 };
		int ret;

		ret = sg_alloc_table_from_pages(&sgt, memdesc->pages,
			memdesc->page_count, 0, memdesc->size, GFP_KERNEL);

		if (ret)
			return ret;

		mapped = kgsl_mmu_map_sg(domain, gmuaddr, sgt.sgl, sgt.nents, attrs);
		sg_free_table(&sgt);
	}

	return mapped == 0 ? -ENOMEM : 0;
}

static int gmu_core_iommu_fault_handler(struct iommu_domain *domain,
		struct device *dev, unsigned long addr, int flags, void *token)
{
	char *fault_type = "unknown";

	if (flags & IOMMU_FAULT_TRANSLATION)
		fault_type = "translation";
	else if (flags & IOMMU_FAULT_PERMISSION)
		fault_type = "permission";
	else if (flags & IOMMU_FAULT_EXTERNAL)
		fault_type = "external";
	else if (flags & IOMMU_FAULT_TRANSACTION_STALLED)
		fault_type = "transaction stalled";

	dev_err(dev, "GMU fault addr = %lX, context=kernel (%s %s fault)\n",
			addr, (flags & IOMMU_FAULT_WRITE) ? "write" : "read", fault_type);

	return 0;
}

int gmu_core_iommu_init(struct kgsl_device *device)
{
	struct device *gmu_pdev_dev = GMU_PDEV_DEV(device);
	int ret;

	device->gmu_core.domain = iommu_domain_alloc(&platform_bus_type);
	if (!device->gmu_core.domain) {
		dev_err(gmu_pdev_dev, "Unable to allocate GMU IOMMU domain\n");
		return -ENODEV;
	}

	/*
	 * Disable stall on fault for the GMU context bank.
	 * This sets SCTLR.CFCFG = 0.
	 * Also note that, the smmu driver sets SCTLR.HUPCF = 0 by default.
	 */
	qcom_iommu_set_fault_model(device->gmu_core.domain,
		QCOM_IOMMU_FAULT_MODEL_NO_STALL);

	ret = iommu_attach_device(device->gmu_core.domain, gmu_pdev_dev);
	if (!ret) {
		iommu_set_fault_handler(device->gmu_core.domain,
			gmu_core_iommu_fault_handler, device);
		return 0;
	}

	dev_err(gmu_pdev_dev, "Unable to attach GMU IOMMU domain: %d\n", ret);
	iommu_domain_free(device->gmu_core.domain);
	device->gmu_core.domain = NULL;

	return ret;
}

void gmu_core_dev_force_first_boot(struct kgsl_device *device)
{
	const struct gmu_dev_ops *ops = GMU_DEVICE_OPS(device);

	if (ops && ops->force_first_boot)
		return ops->force_first_boot(device);
}

int gmu_core_set_vrb_register(struct kgsl_memdesc *vrb, u32 index, u32 val)
{
	u32 *vrb_buf;

	if (WARN_ON(IS_ERR_OR_NULL(vrb)))
		return -ENODEV;

	if (WARN_ON(index >= (vrb->size >> 2))) {
		pr_err("kgsl: Unable to set VRB register for index %u\n", index);
		return -EINVAL;
	}

	vrb_buf = vrb->hostptr;
	vrb_buf[index] = val;

	/* Make sure the vrb write is posted before moving ahead */
	wmb();

	return 0;
}

int gmu_core_get_vrb_register(struct kgsl_memdesc *vrb, u32 index, u32 *val)
{
	u32 *vrb_buf;

	if (IS_ERR_OR_NULL(vrb))
		return -ENODEV;

	if (WARN_ON(index >= (vrb->size >> 2))) {
		pr_err("kgsl: Unable to get VRB register for index %u\n", index);
		return -EINVAL;
	}

	vrb_buf = vrb->hostptr;
	*val = vrb_buf[index];

	return 0;
}

static void stream_trace_data(struct gmu_trace_packet *pkt)
{
	switch (pkt->trace_id) {
	case GMU_TRACE_PREEMPT_TRIGGER: {
		struct trace_preempt_trigger *data =
				(struct trace_preempt_trigger *)pkt->payload;

		trace_adreno_preempt_trigger(data->cur_rb, data->next_rb,
			data->ctx_switch_cntl, pkt->ticks);
		break;
		}
	case GMU_TRACE_PREEMPT_DONE: {
		struct trace_preempt_done *data =
				(struct trace_preempt_done *)pkt->payload;

		trace_adreno_preempt_done(data->prev_rb, data->next_rb,
			data->ctx_switch_cntl, pkt->ticks);
		break;
		}
	case GMU_TRACE_EXTERNAL_HW_FENCE_SIGNAL: {
		struct trace_ext_hw_fence_signal *data =
				(struct trace_ext_hw_fence_signal *)pkt->payload;

		trace_adreno_ext_hw_fence_signal(data->context, data->seq_no,
			data->flags, pkt->ticks);
		break;
		}
	case GMU_TRACE_SYNCOBJ_RETIRE: {
		struct trace_syncobj_retire *data =
				(struct trace_syncobj_retire *)pkt->payload;

		trace_adreno_syncobj_retired(data->gmu_ctxt_id, data->timestamp, pkt->ticks);
		break;
		}
	default: {
		char str[64];

		snprintf(str, sizeof(str),
			 "Unsupported GMU trace id %d\n", pkt->trace_id);
		trace_kgsl_msg(str);
		}
	}
}

void gmu_core_process_trace_data(struct kgsl_device *device,
	struct device *dev, struct kgsl_gmu_trace *trace)
{
	struct gmu_trace_header *trace_hdr = trace->md->hostptr;
	u32 size, *buffer = trace->md->hostptr;
	struct gmu_trace_packet *pkt;
	u16 seq_num, num_pkts = 0;
	u32 ridx = readl(&trace_hdr->read_index);
	u32 widx = readl(&trace_hdr->write_index);

	if (ridx == widx)
		return;

	/*
	 * Don't process any traces and force set read_index to write_index if
	 * previously encountered invalid trace packet
	 */
	if (trace->reset_hdr) {
		/* update read index to let f2h daemon to go to sleep */
		writel(trace_hdr->write_index, &trace_hdr->read_index);
		return;
	}

	/* start reading trace buffer data */
	pkt = (struct gmu_trace_packet *)&buffer[trace_hdr->payload_offset + ridx];

	/* Validate packet header */
	if (TRACE_PKT_GET_VALID_FIELD(pkt->hdr) != TRACE_PKT_VALID) {
		char str[128];

		snprintf(str, sizeof(str),
			"Invalid trace packet found at read index: %d resetting trace header\n",
			trace_hdr->read_index);
		/*
		 * GMU is not expected to write an invalid trace packet. This
		 * condition can be true in case there is memory corruption. In
		 * such scenario fastforward readindex to writeindex so the we
		 * don't process any trace packets until we reset the trace
		 * header in next slumber exit.
		 */
		dev_err_ratelimited(device->dev, "%s\n", str);
		trace_kgsl_msg(str);
		writel(trace_hdr->write_index, &trace_hdr->read_index);
		trace->reset_hdr = true;
		return;
	}

	size = TRACE_PKT_GET_SIZE(pkt->hdr);

	if (TRACE_PKT_GET_SKIP_FIELD(pkt->hdr))
		goto done;

	seq_num = TRACE_PKT_GET_SEQNUM(pkt->hdr);
	num_pkts = seq_num - trace->seq_num;

	/* Detect trace packet loss by tracking any gaps in the sequence number */
	if (num_pkts > 1) {
		char str[128];

		snprintf(str, sizeof(str),
			"%d GMU trace packets dropped from sequence number: %d\n",
			num_pkts - 1, trace->seq_num);
		trace_kgsl_msg(str);
	}

	trace->seq_num = seq_num;
	stream_trace_data(pkt);
done:
	ridx = (ridx + size) % trace_hdr->payload_size;
	writel(ridx, &trace_hdr->read_index);
}

bool gmu_core_is_trace_empty(struct gmu_trace_header *hdr)
{
	return (readl(&hdr->read_index) == readl(&hdr->write_index)) ? true : false;
}

void gmu_core_trace_header_init(struct kgsl_gmu_trace *trace)
{
	struct gmu_trace_header *hdr = trace->md->hostptr;

	hdr->threshold = TRACE_BUFFER_THRESHOLD;
	hdr->timeout = TRACE_TIMEOUT_MSEC;
	hdr->metadata = FIELD_PREP(GENMASK(31, 30), TRACE_MODE_DROP) |
			FIELD_PREP(GENMASK(3, 0), TRACE_HEADER_VERSION_1);
	hdr->cookie = trace->md->gmuaddr;
	hdr->size = trace->md->size;
	hdr->log_type = TRACE_LOGTYPE_HWSCHED;
}

void gmu_core_reset_trace_header(struct kgsl_gmu_trace *trace)
{
	struct gmu_trace_header *hdr = trace->md->hostptr;

	if (!trace->reset_hdr)
		return;

	memset(hdr, 0, sizeof(struct gmu_trace_header));
	/* Reset sequence number to detect trace packet loss */
	trace->seq_num = 0;
	gmu_core_trace_header_init(trace);
	trace->reset_hdr = false;
}

#if (KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE)
struct rproc *gmu_core_soccp_vote_init(struct device *dev)
{
	u32 soccp_handle;
	struct rproc *soccp_rproc;

	if (of_property_read_u32(dev->of_node, "qcom,soccp-controller", &soccp_handle))
		return NULL;

	soccp_rproc = rproc_get_by_phandle(soccp_handle);
	if (!IS_ERR_OR_NULL(soccp_rproc))
		return soccp_rproc;

	dev_err(dev, "Failed to get rproc for phandle:%u ret:%ld Disabling hw fences\n",
		soccp_handle, soccp_rproc ? PTR_ERR(soccp_rproc) : -ENOENT);

	return soccp_rproc ? soccp_rproc : ERR_PTR(-ENOENT);
}

int gmu_core_soccp_vote(struct device *dev, unsigned long *gmu_flags, struct rproc *soccp_rproc,
	bool pwr_on)
{
	int ret;

	if (!soccp_rproc)
		return 0;

	if (!(test_bit(GMU_PRIV_SOCCP_VOTE_ON, gmu_flags) ^ pwr_on))
		return 0;

	ret = rproc_set_state(soccp_rproc, pwr_on);
	if (!ret) {
		change_bit(GMU_PRIV_SOCCP_VOTE_ON, gmu_flags);
		return 0;
	}

	dev_err(dev, "soccp power %s failed: %d. Disabling hw fences\n",
		pwr_on ? "on" : "off", ret);

	return ret;
}

#else

struct rproc *gmu_core_soccp_vote_init(struct device *dev)
{
	return ERR_PTR(-ENOENT);
}

int gmu_core_soccp_vote(struct device *dev, unsigned long *gmu_flags, struct rproc *soccp_rproc,
	bool pwr_on)
{
	return -EINVAL;
}

#endif
