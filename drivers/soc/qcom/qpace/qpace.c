// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/pm_qos.h>
#include <linux/cpu.h>
#include <linux/interconnect.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/iopoll.h>
#include <linux/suspend.h>
#include <dt-bindings/interconnect/qcom,icc.h>

#include "qpace-constants.h"
#include "qpace-reg-accessors.h"
#include "qpace.h"

#define CREATE_TRACE_POINTS
#include <trace/events/qpace.h>

#define QPACE_REG_PAGE_SIZE (1 << 12)

/*
 * General config, statistic and debug registers for QPaCE
 */
static void __iomem *qpace_gen_regs;
#define QPACE_READ_GEN_REG(offset) readl(qpace_gen_regs + offset)
#define QPACE_WRITE_GEN_REG(offset, val) writel(val, qpace_gen_regs + offset)

/*
 * Register interface for controlling behavior of QPaCE's decomp, comp and copy
 * cores
 */
static void __iomem *qpace_gen_core_regs;
#define QPACE_READ_GEN_CORE_REG(offset) readl(qpace_gen_core_regs + offset)
#define QPACE_WRITE_GEN_CORE_REG(offset, val) writel(val, qpace_gen_core_regs + offset)

static void __iomem *qpace_comp_core_regs;
#define QPACE_READ_COMP_CORE_REG(offset) readl(qpace_comp_core_regs + offset)
#define QPACE_WRITE_COMP_CORE_REG(offset, val) writel(val, qpace_comp_core_regs + offset)

static void __iomem *qpace_decomp_core_regs;
static void __iomem *qpace_copy_core_regs;

/*
 * Register interface for submitting requests to QPaCE and for getting the
 * output of the commands
 */
static void __iomem *qpace_urg_regs;
#define QPACE_READ_URG_CMD_REG(idx, offset)				\
({									\
	readl(qpace_urg_regs + (idx * QPACE_REG_PAGE_SIZE) + offset);	\
})									\

#define QPACE_WRITE_URG_CMD_REG(idx, offset, val)				\
{										\
	writel(val, qpace_urg_regs + (idx * QPACE_REG_PAGE_SIZE) + offset);	\
}										\

#define QPACE_WRITE_URG_CMD_CTX_REG(offset, urg_reg_num, ctx_num, val)		\
{										\
	writel(val, qpace_urg_regs + (urg_reg_num * QPACE_REG_PAGE_SIZE) +	\
	       offset + (ctx_num * URG_CMD_CONTEXT_SPACING));			\
}										\

static void __iomem *qpace_tr_regs;
#define QPACE_READ_TR_REG(idx, offset) readl(qpace_tr_regs + (idx * QPACE_REG_PAGE_SIZE) + offset)

#define QPACE_WRITE_TR_REG(idx, offset, val)					\
{										\
	writel(val, qpace_tr_regs + (idx * QPACE_REG_PAGE_SIZE) + offset);	\
}										\

static void __iomem *qpace_er_regs;
#define QPACE_READ_ER_REG(idx, offset) readl(qpace_er_regs + (idx * QPACE_REG_PAGE_SIZE) + offset)

#define QPACE_WRITE_ER_REG(idx, offset, val)					\
{										\
	writel(val, qpace_er_regs + (idx * QPACE_REG_PAGE_SIZE) + offset);	\
}										\

static void __iomem *qpace_gen_cmd_regs;
#define QPACE_READ_GEN_CMD_REG(offset) readl(qpace_gen_cmd_regs + offset)
#define QPACE_WRITE_GEN_CMD_REG(offset, val) writel(val, qpace_gen_cmd_regs + offset)

static bool qpace_suspended;
static struct icc_path *qpace_interconnect;
static struct device *qpace_dev;
static struct dev_pm_qos_request qos_req;

/*
 * =============================================================================
 * Urgent init code
 * =============================================================================
 */

/* Urgent command registers */
enum urg_reg_cxts {
	URG_COMP_CNTXT,
	URG_DECOMP_CNTXT,
	URG_COPY_CNTXT
};

static inline void program_urg_comp_context(int urg_reg_num)
{
	u32 urg_cmd_settings = 0;

	/* Set the limit of the compression output size */
	urg_cmd_settings = FIELD_PREP(URG_CMD_0_CFG_CNTXT_SIZE_SIZE, PAGE_SIZE - 1);
	QPACE_WRITE_URG_CMD_CTX_REG(QPACE_URG_CMD_0_CFG_CNTXT_SIZE_n_OFFSET,
				    urg_reg_num, URG_COMP_CNTXT,
				    urg_cmd_settings);

	urg_cmd_settings = FIELD_PREP(URG_CMD_0_CFG_CNTXT_MISC_OPER, COMP);

	/* Indicate that the compression imput size is an order 0 page */
	urg_cmd_settings |= FIELD_PREP(URG_CMD_0_CFG_CNTXT_MISC_PAGE_CNT, 0);

	/* Program part of the SMMU input and output SIDs */
	urg_cmd_settings |= FIELD_PREP(URG_CMD_0_CFG_CNTXT_MISC_SCTX,
				       DEFAULT_SMMU_CONTEXT);
	urg_cmd_settings |= FIELD_PREP(URG_CMD_0_CFG_CNTXT_MISC_DCTX,
				       DEFAULT_SMMU_CONTEXT);

	/* Cache accesses in the system cache during compression */
	urg_cmd_settings |= FIELD_PREP(URG_CMD_0_CFG_CNTXT_MISC_WA, 1);

	QPACE_WRITE_URG_CMD_CTX_REG(QPACE_URG_CMD_0_CFG_CNTXT_MISC_n_OFFSET,
				    urg_reg_num, URG_COMP_CNTXT,
				    urg_cmd_settings);
}

static inline void program_urg_decomp_context(int urg_reg_num)
{
	u32 urg_cmd_settings = 0;

	/* The input size will be programmed by a requestor */
	urg_cmd_settings = FIELD_PREP(URG_CMD_0_CFG_CNTXT_SIZE_SIZE, 0);
	QPACE_WRITE_URG_CMD_CTX_REG(QPACE_URG_CMD_0_CFG_CNTXT_SIZE_n_OFFSET,
				    urg_reg_num, URG_DECOMP_CNTXT,
				    urg_cmd_settings);

	urg_cmd_settings = FIELD_PREP(URG_CMD_0_CFG_CNTXT_MISC_OPER, DECOMP);

	/* Program part of the SMMU input and output SIDs */
	urg_cmd_settings |= FIELD_PREP(URG_CMD_0_CFG_CNTXT_MISC_SCTX,
				       DEFAULT_SMMU_CONTEXT);
	urg_cmd_settings |= FIELD_PREP(URG_CMD_0_CFG_CNTXT_MISC_DCTX,
				       DEFAULT_SMMU_CONTEXT);

	/* Cache accesses in the system cache during decompression */
	urg_cmd_settings |= FIELD_PREP(URG_CMD_0_CFG_CNTXT_MISC_WA, 1);

	QPACE_WRITE_URG_CMD_CTX_REG(QPACE_URG_CMD_0_CFG_CNTXT_MISC_n_OFFSET,
				    urg_reg_num, URG_DECOMP_CNTXT,
				    urg_cmd_settings);
}

static void program_urg_command_contexts(void)
{
	int urg_reg_num;

	/* Configure the needed contexts for each of the urgent command registers */
	for (urg_reg_num = 0; urg_reg_num < NUM_TRS_ERS_URG_CMD_REGS; urg_reg_num++) {
		program_urg_comp_context(urg_reg_num);
		program_urg_decomp_context(urg_reg_num);
	}
}

/*
 * =============================================================================
 * Urgent operation code
 * =============================================================================
 */

static inline u32 qpace_urgent_command_trigger(phys_addr_t input_addr,
					       phys_addr_t output_addr,
					       int urg_reg_num,
					       enum urg_reg_cxts command)
{
	void *td_dst_src_reg = qpace_urg_regs + (urg_reg_num * QPACE_REG_PAGE_SIZE) +
			       QPACE_URG_CMD_0_TD_DST_ADDR_L_CFG_CNTXT_OFFSET;
	u64 urg_addr_field_lower, urg_addr_field_upper;
	u32 stat_reg;

	urg_addr_field_lower = FIELD_PREP(URG_CMD_0_TD_DST_ADDR_L__CMD_CFG_CNTXT,
					command);
	urg_addr_field_lower |= GENMASK(63, 8) & output_addr;

	urg_addr_field_upper = input_addr;

	/* Ensure that preceding stores that QPaCE will depend on are done executing */
	dma_wmb();

	asm volatile(
	"stp %0, %1, [%2]\n"
	: : "r" (urg_addr_field_lower), "r" (urg_addr_field_upper), "r" (td_dst_src_reg)
	: "memory");


	stat_reg = QPACE_READ_URG_CMD_REG(urg_reg_num,
					  QPACE_URG_CMD_0_ED_STAT_OFFSET);

	/* Wait for operation to finish */
	while (FIELD_GET(URG_CMD_0_ED_STAT_COMP_CODE, stat_reg) == OP_URG_ONGOING)
		stat_reg = QPACE_READ_URG_CMD_REG(urg_reg_num,
						  QPACE_URG_CMD_0_ED_STAT_OFFSET);

	return stat_reg;
}

/*
 * qpace_urgent_compress() - compress a page
 * @input_addr: Input address of the page to be compressed
 * @output_addr: Address to output the compressed page to
 *
 * Compress a page of PAGE_SIZE bytes and output it to @output_addr.
 *
 * Returns the size of the compressed page in bytes if successful and returns
 * -EINVAL if compression failed.
 */
int qpace_urgent_compress(phys_addr_t input_addr, phys_addr_t output_addr)
{
	/* We have 8 cores and 8 urgent command registers */
	int urg_reg_num;
	u32 stat_reg, stat_reg_val;

	urg_reg_num = get_cpu();
	stat_reg = qpace_urgent_command_trigger(input_addr, output_addr, urg_reg_num,
						URG_COMP_CNTXT);
	put_cpu();

	stat_reg_val = FIELD_GET(URG_CMD_0_ED_STAT_COMP_CODE, stat_reg);
	if (stat_reg_val != OP_OK) {
		pr_err("%s: register %d failed with %u\n",
		       __func__, urg_reg_num, stat_reg_val);
		return -EINVAL;
	}

	return FIELD_GET(URG_CMD_0_ED_STAT_SIZE, stat_reg);
}
EXPORT_SYMBOL_GPL(qpace_urgent_compress);

/*
 * qpace_urgent_decompress() - decompress a page that was compressed by QPaCE
 * @input_addr: Input address of the compressed page
 * @output_addr: Address to output the decompressed page to
 * @input_size: size of the compressed input-page
 *
 * Decompressed a compressed page previously compressed by QPaCE. The output is
 * expected to be PAGE_SIZE bytes.
 *
 * Returns the size of the decompressed page in bytes if successful (though the
 * caller should know its page sized), and returns -EINVAL if decompression
 * failed.
 */
int qpace_urgent_decompress(phys_addr_t input_addr,
			    phys_addr_t output_addr,
			    size_t input_size)
{
	/* We have 8 cores and 8 urgent command registers */
	int urg_reg_num;
	u32 stat_reg, stat_reg_val;

	trace_start_qpace_urgent_decompress((void *) input_addr,
		(void *) output_addr, input_size);

	urg_reg_num = get_cpu();
	stat_reg = qpace_urgent_command_trigger(input_addr, output_addr, urg_reg_num,
						URG_DECOMP_CNTXT);
	put_cpu();

	stat_reg_val = FIELD_GET(URG_CMD_0_ED_STAT_COMP_CODE, stat_reg);
	if (stat_reg_val != OP_OK) {
		pr_err("%s: register %d failed with %u\n",
		       __func__, urg_reg_num, stat_reg_val);

		trace_end_qpace_urgent_decompress((void *) input_addr,
			(void *) output_addr, input_size, stat_reg_val);

		return -EINVAL;
	}

	trace_end_qpace_urgent_decompress((void *) input_addr,
		(void *) output_addr, input_size, stat_reg_val);

	return FIELD_GET(URG_CMD_0_ED_STAT_SIZE, stat_reg);
}
EXPORT_SYMBOL_GPL(qpace_urgent_decompress);

/*
 * =============================================================================
 * Ring init code
 * =============================================================================
 */

struct transfer_ring {
	struct qpace_transfer_descriptor *ring_buffer_start;

	struct qpace_transfer_descriptor *hw_read_ptr;
	struct qpace_transfer_descriptor *hw_write_ptr;

	size_t item_count;
};

struct event_ring {
	struct completion ring_has_events;
	struct qpace_event_descriptor *ring_buffer_start;

	struct qpace_event_descriptor *hw_read_ptr;
	struct qpace_event_descriptor *hw_write_ptr;
	bool cycle_bit;
};

static struct transfer_ring *tr_rings;
static struct event_ring *ev_rings;


static inline void deinit_transfer_ring(int tr_num)
{
	__free_pages(virt_to_page(tr_rings[tr_num].ring_buffer_start), RING_SIZE_ORDER);
}

static inline int init_transfer_ring(int tr_num, bool reinit_ring)
{
	struct page *page_ptr;
	u32 tr_ring_reg_val;
	phys_addr_t ring_start_phys, ring_end_phys;

	BUILD_BUG_ON(sizeof(struct qpace_transfer_descriptor) != DESCRIPTOR_SIZE);

	if (!reinit_ring) {
		/* Allocate TR buffer */
		page_ptr = alloc_pages(GFP_KERNEL | __GFP_ZERO,
					RING_SIZE_ORDER);
		if (!page_ptr)
			return -ENOMEM;

		tr_rings[tr_num].ring_buffer_start = page_to_virt(page_ptr);
	}

	/*
	 * Set block-event-interrupt on all TDs by default. We will set bei = 0
	 * only one TD per batch, in order to have a single interrupt for a
	 * batch.
	 */
	for (int td_id = 0; td_id < DESCRIPTORS_PER_RING; td_id++)
		tr_rings[tr_num].ring_buffer_start[td_id].bei = 1;

	/* Configure the ring registers */
	tr_ring_reg_val = FIELD_PREP(DMA_TR_MGR_0_CFG_CACHEALLOC, LLC_NO_CACHE_OP);
	tr_ring_reg_val |= FIELD_PREP(DMA_TR_MGR_0_CFG_CTX,
				  DEFAULT_SMMU_CONTEXT);
	/*
	 * Leave COMP_SIZE_OVER_LIMIT_FORCE_EVT as is and let interrupt
	 * generation settings be done on a per-TD basis.
	 */
	QPACE_WRITE_TR_REG(tr_num, QPACE_DMA_TR_MGR_0_CFG_OFFSET,
			   tr_ring_reg_val);

	/* Initialize HW and SW ring pointers */
	tr_rings[tr_num].hw_write_ptr = tr_rings[tr_num].ring_buffer_start;
	tr_rings[tr_num].hw_read_ptr = tr_rings[tr_num].ring_buffer_start +
				       DESCRIPTORS_PER_RING - 1;

	ring_start_phys = virt_to_phys(tr_rings[tr_num].hw_write_ptr);
	ring_end_phys = virt_to_phys(tr_rings[tr_num].hw_read_ptr + 1);

	tr_ring_reg_val = FIELD_GET(GENMASK(63, 32), ring_start_phys);
	QPACE_WRITE_TR_REG(tr_num, QPACE_DMA_TR_MGR_0_START_H_OFFSET,
			   tr_ring_reg_val);
	QPACE_WRITE_TR_REG(tr_num, QPACE_DMA_TR_MGR_0_START_L_OFFSET,
			   FIELD_GET(GENMASK(31, 0), ring_start_phys));

	tr_ring_reg_val = FIELD_GET(GENMASK(63, 32), ring_end_phys);
	QPACE_WRITE_TR_REG(tr_num, QPACE_DMA_TR_MGR_0_END_H_OFFSET,
			   tr_ring_reg_val);
	QPACE_WRITE_TR_REG(tr_num, QPACE_DMA_TR_MGR_0_END_L_OFFSET,
			   FIELD_GET(GENMASK(31, 0), ring_end_phys));

	return 0;
}

static void deinit_transfer_rings(int num_rings_to_deinit)
{
	for (int tr_num = 0; tr_num < num_rings_to_deinit; tr_num++)
		deinit_transfer_ring(tr_num);

	kfree(tr_rings);
}

static int init_transfer_rings(void)
{
	int ret;

	tr_rings = kmalloc_array(NUM_RINGS, sizeof(struct transfer_ring),
				 GFP_KERNEL | __GFP_ZERO);
	if (!tr_rings)
		return -ENOMEM;
pr_err("transfer rings initialized\n");
	for (int tr_num = 0; tr_num < NUM_RINGS; tr_num++) {
		ret = init_transfer_ring(tr_num, false);
		if (ret) {
			deinit_transfer_rings(tr_num);
			return ret;
		}
	}

	return 0;
}

static inline void deinit_event_ring(int er_num)
{
	__free_pages(virt_to_page(ev_rings[er_num].ring_buffer_start), RING_SIZE_ORDER);
}

static inline int init_event_ring(int er_num, bool reinit_ring)
{
	struct page *page_ptr;
	u32 ev_ring_reg_val;
	phys_addr_t ring_start_phys, ring_end_phys;

	BUILD_BUG_ON(sizeof(struct qpace_event_descriptor) != DESCRIPTOR_SIZE);

	init_completion(&ev_rings[er_num].ring_has_events);

	if (!reinit_ring) {
		/* Allocate EV buffer */
		page_ptr = alloc_pages(GFP_KERNEL | __GFP_ZERO,
					RING_SIZE_ORDER);
		if (!page_ptr)
			return -ENOMEM;

		ev_rings[er_num].ring_buffer_start = page_to_virt(page_ptr);
	}

	/* Configure the ring registers */
	ev_ring_reg_val = FIELD_PREP(DMA_ER_MGR_0_CFG_CACHEALLOC, LLC_ALLOC);
	ev_ring_reg_val |= FIELD_PREP(DMA_ER_MGR_0_CFG_CTX,
				  DEFAULT_SMMU_CONTEXT);
	QPACE_WRITE_ER_REG(er_num, QPACE_DMA_ER_MGR_0_CFG_OFFSET,
			   ev_ring_reg_val);

	/* Initialize HW and SW ring pointers */
	ev_rings[er_num].hw_write_ptr = ev_rings[er_num].ring_buffer_start;
	ev_rings[er_num].hw_read_ptr = ev_rings[er_num].ring_buffer_start +
				       DESCRIPTORS_PER_RING - 1;

	ring_start_phys = virt_to_phys(ev_rings[er_num].hw_write_ptr);
	ring_end_phys = virt_to_phys(ev_rings[er_num].hw_read_ptr + 1);

	QPACE_WRITE_ER_REG(er_num, QPACE_DMA_ER_MGR_0_START_H_OFFSET,
			   FIELD_GET(GENMASK(63, 32), ring_start_phys));
	QPACE_WRITE_ER_REG(er_num, QPACE_DMA_ER_MGR_0_START_L_OFFSET,
			   FIELD_GET(GENMASK(31, 0), ring_start_phys));

	QPACE_WRITE_ER_REG(er_num, QPACE_DMA_ER_MGR_0_END_L_OFFSET,
			   FIELD_GET(GENMASK(31, 0), ring_end_phys));
	QPACE_WRITE_ER_REG(er_num, QPACE_DMA_ER_MGR_0_END_H_OFFSET,
			   FIELD_GET(GENMASK(63, 32), ring_end_phys));

	QPACE_WRITE_ER_REG(er_num, QPACE_DMA_ER_MGR_0_RD_PTR_H_OFFSET,
			   FIELD_GET(GENMASK(63, 32), ring_end_phys));
	QPACE_WRITE_ER_REG(er_num, QPACE_DMA_ER_MGR_0_RD_PTR_L_OFFSET,
			   FIELD_GET(GENMASK(31, 0), ring_end_phys) - DESCRIPTOR_SIZE);

	ev_rings[er_num].cycle_bit = false;

	/*
	 * Since struct event_ring->cycle_bit is zero initialized, for the initial pass
	 * by HW, an invalid ED will need to have a cycle bit of 1 / true, which will be
	 * cleared once a valid event has been placed in an ED.
	 */
	for (struct qpace_event_descriptor *ed = ev_rings[er_num].ring_buffer_start;
	     ed < ev_rings[er_num].ring_buffer_start + DESCRIPTORS_PER_RING;
	     ed++)
		ed->cycle_bit = true;

	return 0;
}

static void deinit_event_rings(int num_rings_to_deinit)
{
	for (int er_num = 0; er_num < num_rings_to_deinit; er_num++)
		deinit_event_ring(er_num);

	kfree(ev_rings);
}

static int init_event_rings(void)
{
	int ret;

	ev_rings = kmalloc_array(NUM_RINGS, sizeof(struct event_ring),
				 GFP_KERNEL | __GFP_ZERO);
	if (!ev_rings)
		return -ENOMEM;

	for (int er_num = 0; er_num < NUM_RINGS; er_num++) {
		ret = init_event_ring(er_num, false);
		if (ret) {
			deinit_event_rings(er_num);
			return ret;
		}
	}

	/* Enable interrupt generation for all in-use rings */
	QPACE_WRITE_GEN_CMD_REG(QPACE_COMMON_EE_DMA_ER_COMPL_INTR_EN_OFFSET,
				(1 << NUM_RINGS) - 1);

	return 0;
}

static DEFINE_MUTEX(qpace_ref_lock);
static int active_rings;

static bool rings_inited_since_activation[NUM_RINGS];

static void _get_qpace(void)
{
	lockdep_assert_held(&qpace_ref_lock);
	if (!active_rings) {
		pm_stay_awake(qpace_dev);
		dev_pm_qos_update_request(&qos_req, 300);
	}
	active_rings++;
}

static void _put_qpace(void)
{
	lockdep_assert_held(&qpace_ref_lock);
	active_rings--;
	if (!active_rings) {
		dev_pm_qos_update_request(&qos_req, PM_QOS_RESUME_LATENCY_DEFAULT_VALUE);
		pm_relax(qpace_dev);
		for (int i = 0; i < ARRAY_SIZE(rings_inited_since_activation); i++)
			rings_inited_since_activation[i] = false;
	}
}

void get_qpace(void)
{
	mutex_lock(&qpace_ref_lock);
	_get_qpace();
	mutex_unlock(&qpace_ref_lock);
}
EXPORT_SYMBOL_GPL(get_qpace);

void put_qpace(void)
{
	mutex_lock(&qpace_ref_lock);
	_put_qpace();
	mutex_unlock(&qpace_ref_lock);
}
EXPORT_SYMBOL_GPL(put_qpace);

/*
 * get_ring() - prepare a given ring for usage and ref count its usage
 * @ring_num: the ring we want to use
 *
 * Call the necessary PM callbacks on the first get_ring() call. Initialize
 * @ring_num if it has not been initialized already for the current usage
 * period. This function implicitly increments a reference counter that
 * tracks the number of items in the ring.
 *
 * Returns 0 if successful, -EINVAL if qpace is suspended.
 */
static int get_ring(int ring_num)
{
	struct transfer_ring *tr = &tr_rings[ring_num];

	/* protect active_rings and qpace_suspended */
	mutex_lock(&qpace_ref_lock);

	if (qpace_suspended) {
		mutex_unlock(&qpace_ref_lock);
		return -EBUSY;
	}

	if (!tr->item_count) {
		_get_qpace();

		if (!rings_inited_since_activation[ring_num]) {
			rings_inited_since_activation[ring_num] = true;
			init_event_ring(ring_num, true);
			init_transfer_ring(ring_num, true);
		}
	}
	tr->item_count++;

	mutex_unlock(&qpace_ref_lock);
	return 0;
}

/*
 * put_ring() - Reduce the number of items tracked by a ring
 * @ring_num: The ring we're modifying usage stats for
 * @n_consumed_entries: The number of items we want to mark as unused
 *
 * Reduces the number of items tracked by a ring. If a ring has no more
 * queued items, it becomes inactive. If all rings become inactive, then
 * we call the necessary PM callbacks to allow QPaCE's resources to be
 * collapsed.
 */
static void put_ring(int ring_num, int  n_consumed_entries)
{
	struct transfer_ring *tr = &tr_rings[ring_num];

	mutex_lock(&qpace_ref_lock);
	tr->item_count -= n_consumed_entries;

	if (!tr->item_count)
		_put_qpace();

	mutex_unlock(&qpace_ref_lock);
}

/*
 * =============================================================================
 * Ring operation code
 * =============================================================================
 */

static inline void transfer_ring_increment(struct transfer_ring *ring)
{
	if (ring->hw_write_ptr == ring->ring_buffer_start + DESCRIPTORS_PER_RING - 1)
		ring->hw_write_ptr = ring->ring_buffer_start;
	else
		ring->hw_write_ptr++;
}

/*
 * qpace_queue_copy() - queue a copy operation
 * @tr_num: The transfer ring to queue the request to
 * @src_addr: Input address of the object to copy
 * @dst_addr: Address to copy the object to
 * @copy_size: Number of bytes to copy
 *
 * Queue a copy request to a chosen transfer ring
 */
int qpace_queue_copy(int tr_num, phys_addr_t src_addr, phys_addr_t dst_addr, size_t copy_size)
{
	struct transfer_ring *ring = &tr_rings[tr_num];
	struct qpace_transfer_descriptor *td;
	int ret;

	ret = get_ring(tr_num);
	if (ret)
		return ret;

	td = ring->hw_write_ptr;

	td->src_addr = src_addr;
	td->dst_addr = dst_addr;

	td->size = copy_size;
	td->operation = COPY;

	/* Deallocate cache line from system cache during copy */
	td->rf = 1;

	/* Page count not needed for copy */

	/* Use default SIDs */

	/* We pair each transfer ring with a unique event ring */
	td->event_ring_id = tr_num;
	ret = ring->hw_write_ptr - ring->ring_buffer_start;
	transfer_ring_increment(ring);

	return ret;
}
EXPORT_SYMBOL_GPL(qpace_queue_copy);

/*
 * qpace_queue_compress() - queue a compression operation
 * @tr_num: The transfer ring to queue the request to
 * @src_addr: Input address of the object to copy
 * @dst_addr: Address to copy the object to
 *
 * Queue a compression request to a chosen transfer ring
 */
int qpace_queue_compress(int tr_num, phys_addr_t src_addr, phys_addr_t dst_addr)
{
	struct transfer_ring *ring = &tr_rings[tr_num];
	struct qpace_transfer_descriptor *td;
	int ret;

	ret = get_ring(tr_num);
	if (ret)
		return ret;

	td = ring->hw_write_ptr;

	td->src_addr = src_addr;
	td->dst_addr = dst_addr;

	td->size = (PAGE_SIZE) - 1;
	td->operation = COMP;

	/* Cache accesses in the system cache during compression */
	td->wa = 1;

	/* Leave page count as order 0 */

	/* Use default SIDs */

	/* We pair each transfer ring with a single event ring */
	td->event_ring_id = tr_num;

	ret = ring->hw_write_ptr - ring->ring_buffer_start;

	transfer_ring_increment(ring);

	return ret;
}
EXPORT_SYMBOL_GPL(qpace_queue_compress);

/*
 * OP_BUS_ERROR and OP_TIMED_OUT should be critial error.
 * OP_COMP_TOO_BIG is not an error. We're settign threshold size to (PAGE_SIZE-1).
 * This typically occurs if the page is completely uncompressible. The logic then
 * goes to huge_class_size case, which is fine.
 */
bool qpace_check_compress_err(struct qpace_event_descriptor *ed)
{
	if (ed->completion_code & (OP_BUS_ERROR | OP_TIMED_OUT))
		return true;
	else
		return false;
}
EXPORT_SYMBOL_GPL(qpace_check_compress_err);

static inline void qpace_free_tr_entries(int tr_num, int n_consumed_entries)
{
	struct transfer_ring *ring = &tr_rings[tr_num];

	/* Update hw_read_ptr */
	int offset = (ring->hw_read_ptr - ring->ring_buffer_start);
	int new_offset = (offset + n_consumed_entries) % DESCRIPTORS_PER_RING;

	ring->hw_read_ptr = ring->ring_buffer_start + new_offset;

	/* Reset BEI */
	ring->hw_read_ptr->bei = 1;
}

static inline bool transfer_ring_is_empty(struct transfer_ring *ring)
{
	return ring->item_count == 0;
}

/*
 * qpace_trigger_tr() - Have QPaCE process outstanding TD in the transfer ring
 * @tr_num: Transfer ring to have qpace take submissions from for processing
 *
 * Have QPaCE process outstanding transfer descriptors / TDs in the transfer ring,
 * i.e. QPaCE will process the requests submitted from the qpace_queue_* operations.
 * This boils down to updating a HW register that points to the end of the valid
 * in the ring buffer.
 *
 * Return: positive for the requests number that submitted to HW, 0 if the ring had
 * no items to submit.
 */
size_t qpace_trigger_tr(int tr_num)
{
	struct transfer_ring *ring = &tr_rings[tr_num];
	struct qpace_transfer_descriptor *last_processed_td;

	phys_addr_t write_ptr_addr;

	/* bail if ring is empty */
	if (transfer_ring_is_empty(ring))
		return 0;

	if (ring->hw_write_ptr == ring->ring_buffer_start)
		last_processed_td = ring->ring_buffer_start + DESCRIPTORS_PER_RING - 1;
	else
		last_processed_td = ring->hw_write_ptr - 1;

	last_processed_td->bei = 0;

	write_ptr_addr = virt_to_phys(ring->hw_write_ptr);

	QPACE_WRITE_TR_REG(tr_num, QPACE_DMA_TR_MGR_0_WR_PTR_H_OFFSET,
			   FIELD_GET(GENMASK(63, 32), write_ptr_addr));

	/* Below write triggers processing */
	QPACE_WRITE_TR_REG(tr_num, QPACE_DMA_TR_MGR_0_WR_PTR_L_OFFSET,
			   FIELD_GET(GENMASK(31, 0), write_ptr_addr));

	return ring->item_count;
}
EXPORT_SYMBOL_GPL(qpace_trigger_tr);

static inline bool read_last_cycle_bit(struct qpace_event_descriptor *ed)
{
	return FIELD_GET(CYCLE_BIT_MASK, READ_ONCE(ed->flags));
}
/*
 * qpace_wait_for_tr_consumption() - Wait for event completion interrupt for @tr_num
 * @tr_num: Transfer ring to have qpace take submissions from for processing
 * @nr_trigger: requests triggered in one batch
 * @no_sleep: If true, poll on the relevant completion event, go to sleep otherwise.
 *
 * Wait for there to be a completion event for the event ring corresponding to the
 * transfer ring @tr_num. Can be sleeping or non-sleeping based on @no_sleep.
 */
void qpace_wait_for_tr_consumption(int tr_num, size_t nr_trigger, bool no_sleep)
{
	struct event_ring *ev_ring = &ev_rings[tr_num];
	struct qpace_event_descriptor *last_ed_to_consume;
	unsigned long ret;
	bool last_cycle_bit, val;
	/* single core: 0.52 GB/s @ SVS -> 7.69us/page */
	unsigned long timeout_us = 100 * USEC_PER_MSEC;
	size_t offset;

	/* Validate nr_trigger to prevent underflow */
	if (WARN_ON_ONCE(nr_trigger == 0 || nr_trigger > DESCRIPTORS_PER_RING)) {
		pr_err("QPACE: Invalid nr_trigger=%zx\n", nr_trigger);
		return;
	}

	if (!no_sleep) {
		/* Returns 0 on timeout */
		ret = wait_for_completion_timeout(&ev_rings[tr_num].ring_has_events,
						usecs_to_jiffies(timeout_us));
		if (!ret)
			pr_err("QPACE: Timeout, fallback to poll\n");
	} else {
		/* Return -ETIMEDOUT on timeout */
		ret = read_poll_timeout_atomic(try_wait_for_completion,
				val, val, 1, timeout_us, false,
				&ev_rings[tr_num].ring_has_events);
		if (ret)
			pr_err("QPACE: Timeout, fallback to poll\n");
	}

	/*
	 * calculate last ed to consume from triggered requests from
	 * last time qpace_trigger_tr
	 */
	offset = ev_ring->hw_write_ptr - ev_ring->ring_buffer_start;
	offset += nr_trigger - 1;

	if (offset > DESCRIPTORS_PER_RING - 1) {
		last_cycle_bit = !ev_ring->cycle_bit;
		last_ed_to_consume = ev_ring->ring_buffer_start + offset -
					DESCRIPTORS_PER_RING;
	} else {
		last_cycle_bit = ev_ring->cycle_bit;
		last_ed_to_consume = ev_ring->ring_buffer_start + offset;
	}
	pr_debug("nr_trigger: %zx last_offset: %zx  cycle_bit: %x\n",
			nr_trigger, offset, last_cycle_bit);

	/*
	 * Check the last_ed_to_consume to see if it has been created yet. If not,
	 * treat the interrupt as spurious and wait for another one.
	 */
	ret = read_poll_timeout(read_last_cycle_bit, val, val == last_cycle_bit,
			8, timeout_us, false, last_ed_to_consume);
	if (ret)
		panic("QPACE: Timeout of polling ed cycle_bit, panic\n");
}
EXPORT_SYMBOL_GPL(qpace_wait_for_tr_consumption);

static inline void qpace_free_er_entries(int er_num, int n_consumed_entries)
{
	struct event_ring *ring = &ev_rings[er_num];
	phys_addr_t last_processed_ed_phys_addr;

	/* Update RD_PTR to last-processed ED */
	if (ring->hw_write_ptr == ring->ring_buffer_start)
		ring->hw_read_ptr = ring->ring_buffer_start + DESCRIPTORS_PER_RING - 1;
	else
		ring->hw_read_ptr = ring->hw_write_ptr - 1;

	last_processed_ed_phys_addr = virt_to_phys(ring->hw_read_ptr);

	QPACE_WRITE_ER_REG(er_num, QPACE_DMA_ER_MGR_0_RD_PTR_H_OFFSET,
			   FIELD_GET(GENMASK(63, 32), last_processed_ed_phys_addr));
	QPACE_WRITE_ER_REG(er_num, QPACE_DMA_ER_MGR_0_RD_PTR_L_OFFSET,
			   FIELD_GET(GENMASK(31, 0), last_processed_ed_phys_addr));

	put_ring(er_num, n_consumed_entries);
}

static inline void consume_ed(struct qpace_event_descriptor *ed, int ed_index,
			      process_ed_fn success_handler,
			      process_ed_fn fail_handler)
{
	if (ed->completion_code == OP_OK)
		success_handler(ed, ed_index);
	else
		fail_handler(ed, ed_index);
}

static inline struct qpace_event_descriptor *event_ring_increment(struct event_ring *ring)
{
	struct qpace_event_descriptor *prev_rd_ptr = ring->hw_read_ptr;

	if (ring->hw_read_ptr == ring->ring_buffer_start + DESCRIPTORS_PER_RING - 1)
		ring->hw_read_ptr = ring->ring_buffer_start;
	else
		ring->hw_read_ptr++;

	return prev_rd_ptr;
}

int qpace_consume_er(int er_num,
		     process_ed_fn success_handler,
		     process_ed_fn fail_handler,
			 bool consume)
{
	struct event_ring *ring = &ev_rings[er_num];
	struct qpace_event_descriptor *ed, *old_rd_ptr;
	int ed_offset;
	int n_consumed_entries = 0;
	bool orig_cycle_bit = ring->cycle_bit;

	/*
	 * The read pointer indicates the last ED processed by SW, so we
	 * increment the read pointer by one to get the first new ED.
	 */
	old_rd_ptr = event_ring_increment(ring);

	/* Initialize loop iterator */
	ed = ring->hw_read_ptr;

loop_again:
	/* Unrolled ring buffer loop */
	for (; ed < ring->ring_buffer_start + DESCRIPTORS_PER_RING &&
	     ed->cycle_bit == ring->cycle_bit;
	     ed++, n_consumed_entries++) {
		ed_offset = ed - ring->ring_buffer_start;

		/* Process the ED */
		consume_ed(ed, ed_offset, success_handler, fail_handler);
	}

	if (ed == ring->ring_buffer_start + DESCRIPTORS_PER_RING) {
		/* Set ed to point to start,  */
		ed = ring->ring_buffer_start;
		ring->cycle_bit = !ring->cycle_bit;
		goto loop_again;
	}

	if (consume) {
		ring->hw_write_ptr = ed;
		qpace_free_er_entries(er_num, n_consumed_entries);
		qpace_free_tr_entries(er_num, n_consumed_entries);
	} else {
		ring->hw_read_ptr = old_rd_ptr;
		ring->cycle_bit = orig_cycle_bit;
	}

	return n_consumed_entries;
}
EXPORT_SYMBOL_GPL(qpace_consume_er);

/*
 * =============================================================================
 * Interrupt handlers
 * =============================================================================
 */

static irqreturn_t urgent_interrupt_handler(int irq, void *unused)
{
	pr_debug("Urgent interrupt handled\n");
	return IRQ_HANDLED;
}

static irqreturn_t event_or_word_error_interrupt_handler(int irq, void *unused)
{
	u32 status_reg = QPACE_READ_GEN_CMD_REG(QPACE_COMMON_EE_DMA_ER_COMPL_STAT_OFFSET);

	for (int er_num = 0; er_num < NUM_RINGS; er_num++)
		if (status_reg & (1 << er_num)) {

			/* re-enable interrupts for this event ring */
			QPACE_WRITE_GEN_CMD_REG(QPACE_COMMON_EE_DMA_ER_COMPL_STAT_CLR_OFFSET,
						1 << er_num);

			complete(&ev_rings[er_num].ring_has_events);

			goto exit;
		}

	if (status_reg & (COMMON_EE_DMA_ER_COMPL_S_DMA_ER_BUS_ERR_STAT |
			  COMMON_EE_DMA_ER_COMPL_S_DMA_TR_BUS_ERR_STAT))
		panic("%s: word error detected, unrecoverable\n", __func__);

exit:
	return IRQ_HANDLED;
}

/*
 * =============================================================================
 * Power management
 * =============================================================================
 */

static inline bool _qpace_power_on(void)
{
	unsigned long ready_status;
	int remaining_attempts = 5;

	QPACE_WRITE_GEN_CORE_REG(QPACE_CORE_OPER_CORE_RUN_STOP_OFFSET,
				 QPACE_RUN);
	ready_status = QPACE_READ_GEN_CORE_REG(QPACE_CORE_OPER_CORE_READY_OFFSET);
	while (!ready_status) {

		remaining_attempts--;
		if (!remaining_attempts) {
			pr_err("Timeout in waiting for QPaCE to turn off\n");
			return false;
		}

		udelay(QPACE_STATE_CHANGE_TIMEOUT_US);

		ready_status = QPACE_READ_GEN_CORE_REG(QPACE_CORE_OPER_CORE_READY_OFFSET);
	}

	return true;
}

static int qpace_power_on(struct device *dev)
{
	int ret, ret2;

	qpace_interconnect = devm_of_icc_get(dev, NULL);
	if (IS_ERR_OR_NULL(qpace_interconnect)) {
		ret = PTR_ERR(qpace_interconnect);

		pr_err("%s: devm_of_icc_get() failed with %d\n", __func__, ret);
		return qpace_interconnect ? ret : -EINVAL;
	}

	ret = device_init_wakeup(dev, true);
	if (ret) {
		pr_err("%s: device_init_wakeup() failed with %d\n", __func__, ret);
		return ret;
	}

	ret = dev_pm_qos_add_request(get_cpu_device(0), &qos_req,
				     DEV_PM_QOS_RESUME_LATENCY,
				     PM_QOS_RESUME_LATENCY_DEFAULT_VALUE);
	if (ret != 0 && ret != 1) {
		pr_err("%s: dev_pm_qos_add_request() failed with %d\n", __func__, ret);
		goto rm_wakeup_capable;
	}

	icc_set_tag(qpace_interconnect, QCOM_ICC_TAG_ACTIVE_ONLY);
	ret = icc_set_bw(qpace_interconnect, 0, 1);
	if (ret) {
		pr_err("Failed to turn on QPaCE VCD: %d\n", ret);
		goto rm_qos;
	}

	if (!_qpace_power_on()) {
		pr_err("Failed to start QPaCE\n");
		ret = -EINVAL;
		goto rm_bw;
	}

	return 0;

rm_bw:
	ret2 = icc_set_bw(qpace_interconnect, 0, 0);
	if (ret2)
		pr_err("Failed to remove QPaCE VCD vote: %d\n", ret2);
rm_qos:
	ret2 = dev_pm_qos_remove_request(&qos_req);
	if (ret2 != 0 && ret2 != 1)
		pr_err("Failed change QoS vote for CPUs: %d\n", ret2);
rm_wakeup_capable:
	device_init_wakeup(dev, false);

	return ret;
}

static inline bool _qpace_power_off(void)
{
	unsigned long ready_status;
	int remaining_attempts = 5;

	QPACE_WRITE_GEN_CORE_REG(QPACE_CORE_OPER_CORE_RUN_STOP_OFFSET,
				 QPACE_STOP);
	ready_status = QPACE_READ_GEN_CORE_REG(QPACE_CORE_OPER_CORE_READY_OFFSET);
	while (ready_status) {

		remaining_attempts--;
		if (!remaining_attempts) {
			pr_err("Timeout in waiting for QPaCE to turn off\n");
			return false;
		}

		udelay(QPACE_STATE_CHANGE_TIMEOUT_US);

		ready_status = QPACE_READ_GEN_CORE_REG(QPACE_CORE_OPER_CORE_READY_OFFSET);
	}

	return true;
}

static void qpace_power_off(struct device *dev)
{
	int ret;

	/* If this fails we can still remove our vote for the VCD to turn QPaCE off */
	if (!_qpace_power_off())
		pr_err("Failed to stop QPaCE\n");

	ret = icc_set_bw(qpace_interconnect, 0, 0);
	if (ret)
		pr_err("Failed to turn off QPaCE VCD: %d\n", ret);

	ret = dev_pm_qos_remove_request(&qos_req);
	if (ret != 0 && ret != 1)
		pr_err("Failed change QoS vote for CPUs: %d\n", ret);

	device_init_wakeup(dev, false);
}

/*
 * =============================================================================
 * Init code
 * =============================================================================
 */

static inline int qpace_register_ioremap(struct device *dev)
{
	qpace_gen_regs = devm_ioremap(dev, QPACE_GEN_REGS_BASE_ADDR,
				      PAGE_SIZE);
	if (!qpace_gen_regs)
		return -EINVAL;

	qpace_gen_core_regs = devm_ioremap(dev, QPACE_GEN_CORE_REGS_BASE_ADDR,
					   PAGE_SIZE);
	if (!qpace_gen_core_regs)
		return -EINVAL;

	qpace_comp_core_regs = devm_ioremap(dev, QPACE_COMP_CORE_REGS_BASE_ADDR,
					    PAGE_SIZE);
	if (!qpace_comp_core_regs)
		return -EINVAL;

	qpace_decomp_core_regs = devm_ioremap(dev, QPACE_DECOMP_CORE_REGS_BASE_ADDR,
					      PAGE_SIZE);
	if (!qpace_decomp_core_regs)
		return -EINVAL;

	qpace_copy_core_regs = devm_ioremap(dev, QPACE_COPY_CORE_REGS_BASE_ADDR,
					    PAGE_SIZE);
	if (!qpace_copy_core_regs)
		return -EINVAL;

	qpace_urg_regs = devm_ioremap(dev, QPACE_URG_REGS_BASE_ADDR,
				      PAGE_SIZE * NUM_TRS_ERS_URG_CMD_REGS);
	if (!qpace_urg_regs)
		return -EINVAL;

	qpace_tr_regs = devm_ioremap(dev, QPACE_TR_REGS_BASE_ADDR,
				     PAGE_SIZE * NUM_TRS_ERS_URG_CMD_REGS);
	if (!qpace_tr_regs)
		return -EINVAL;

	qpace_er_regs = devm_ioremap(dev, QPACE_ER_REGS_BASE_ADDR,
				     PAGE_SIZE * NUM_TRS_ERS_URG_CMD_REGS);
	if (!qpace_er_regs)
		return -EINVAL;

	qpace_gen_cmd_regs = devm_ioremap(dev, QPACE_EE_REGS_BASE_ADDR,
					  PAGE_SIZE);
	if (!qpace_gen_cmd_regs)
		return -EINVAL;

	return 0;
}

static int qpace_init(void)
{
	int ret;
	u32 reg_val;

	/* Select CPU SCID for our system cache slice */
	reg_val = QPACE_READ_GEN_REG(QPACE_CORE_QNS4_CFG_OFFSET);
	reg_val &= ~CORE_QNS4_CFG_CACHEINDEX;
	reg_val |= FIELD_PREP(CORE_QNS4_CFG_CACHEINDEX, 0x1);
	QPACE_WRITE_GEN_REG(QPACE_CORE_QNS4_CFG_OFFSET, reg_val);

	/*
	 * Make all compression engiens as bulk, to trade start-up latency with
	 * power savings.
	 */
	reg_val = QPACE_READ_COMP_CORE_REG(QPACE_COMP_CORE_BULK_MODE_OFFSET);
	reg_val |= COMP_CORE_BULK_MODE_CORE_0 | COMP_CORE_BULK_MODE_CORE_1 |
		   COMP_CORE_BULK_MODE_CORE_2 | COMP_CORE_BULK_MODE_CORE_3 |
		   COMP_CORE_BULK_MODE_CORE_4;
	QPACE_WRITE_COMP_CORE_REG(QPACE_COMP_CORE_BULK_MODE_OFFSET, reg_val);

	reg_val = QPACE_READ_GEN_CORE_REG(QPACE_CORE_OPER_CFG_OFFSET);
	reg_val |= CORE_OPER_CFG_COMP_MEM_PWR_DWN_1;
	QPACE_WRITE_GEN_CORE_REG(QPACE_CORE_OPER_CFG_OFFSET,
				 reg_val);

	program_urg_command_contexts();

	ret = init_transfer_rings();
	if (ret)
		return ret;

	ret = init_event_rings();
	if (ret)
		goto deinit_transfer_rings;

	return 0;

deinit_transfer_rings:
	deinit_transfer_rings(NUM_RINGS);

	return ret;
}

enum QPACE_INTERRUPTS {
	QPACE_IRQ_RING_AND_BUS_ERR,
	QPACE_IRQ_URGENT
};

static int qpace_register_interrupts(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int irq, ret;

	/* Ring and bus error interrupt */
	irq = platform_get_irq(pdev, QPACE_IRQ_RING_AND_BUS_ERR);
	if (irq < 0) {
		dev_err(dev, "Failed to find urgent interrupt\n");
		return irq;
	}

	ret = devm_request_irq(dev, irq, event_or_word_error_interrupt_handler,
			       0, "qpace-event-or-word-err-irq", NULL);
	if (ret) {
		dev_err(dev, "failed to request urgent interrupt\n");
		return ret;
	}

	/* Urgent interrupt */
	irq = platform_get_irq(pdev, QPACE_IRQ_URGENT);
	if (irq < 0) {
		dev_err(dev, "Failed to find urgent interrupt\n");
		return irq;
	}

	ret = devm_request_irq(dev, irq, urgent_interrupt_handler,
			       0, "qpace-urgent-irq", NULL);
	if (ret) {
		dev_err(dev, "failed to request urgent interrupt\n");
		return ret;
	}

	return 0;
}

bool is_qpace_dev_available(void)
{
	struct device_node *node;

	node = of_find_node_by_name(NULL, "qcom,qpace");
	if (!node) {
		pr_warn("qpace: qpace device node not found\n");
		return false;
	}
	if (!of_device_is_available(node)) {
		pr_warn("qpace: qpace device node not found\n");
		of_node_put(node);
		return false;
	}
	return true;
}
EXPORT_SYMBOL_GPL(is_qpace_dev_available);

static int qpace_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	ret = qpace_register_interrupts(pdev);
	if (ret)
		return ret;

	ret = qpace_register_ioremap(dev);
	if (ret)
		return ret;

	ret = qpace_power_on(dev);
	if (ret)
		return ret;

	ret = qpace_init();
	if (ret)
		goto power_off;

	qpace_dev = dev;

	return ret;

power_off:
	qpace_power_off(dev);

	return ret;
}

static void qpace_remove(struct platform_device *pdev)
{
	deinit_transfer_rings(NUM_RINGS);
	deinit_event_rings(NUM_RINGS);
	qpace_power_off(&pdev->dev);
}

static const struct of_device_id qpace_match_table[] = {
	{.compatible = "qcom,qpace"},
	{},
};


static int qpace_suspend(struct device *dev)
{
	mutex_lock(&qpace_ref_lock);

	/* Set suspended flag first to prevent new submissions */
	qpace_suspended = true;
	/* Ensure flag is visible before checking queues */
	smp_wmb();

	if (active_rings) {
		/* new requests queued during suspend */
		dev_err(dev, "active_rings not 0, abort suspend\n");
		qpace_suspended = false;
		mutex_unlock(&qpace_ref_lock);
		return -EBUSY;
	}

	mutex_unlock(&qpace_ref_lock);
	return 0;
}

static int qpace_resume(struct device *dev)
{
	mutex_lock(&qpace_ref_lock);

	if (active_rings)
		dev_err(dev, "active_rings not 0, unexpected case\n");

	qpace_suspended = false;
	mutex_unlock(&qpace_ref_lock);
	return 0;
}

const struct dev_pm_ops qpace_pm_ops = {
	.suspend = qpace_suspend,
	.resume = qpace_resume,
};

static struct platform_driver qpace_driver = {
	.probe = qpace_probe,
	.remove = qpace_remove,
	.driver = {
		.name = "qcom-qpace",
		.of_match_table = qpace_match_table,
		.pm = &qpace_pm_ops,
	},
};

static int __init qpace_module_init(void)
{
	return platform_driver_register(&qpace_driver);
}

static void __exit qpace_module_exit(void)
{
	platform_driver_unregister(&qpace_driver);
}

module_init(qpace_module_init);
module_exit(qpace_module_exit);

MODULE_LICENSE("GPL");

