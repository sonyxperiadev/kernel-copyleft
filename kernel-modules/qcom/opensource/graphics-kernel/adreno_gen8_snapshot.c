// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2025, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "adreno.h"
#include "adreno_gen8_0_0_snapshot.h"
#include "adreno_gen8_3_0_snapshot.h"
#include "adreno_gen8_6_0_snapshot.h"
#include "adreno_snapshot.h"

static struct kgsl_memdesc *gen8_capturescript;
static struct kgsl_memdesc *gen8_crashdump_registers;
static u32 *gen8_cd_reg_end;
static const struct gen8_snapshot_block_list *gen8_snapshot_block_list;
static bool gen8_crashdump_timedout;

/* Starting kernel virtual address for QDSS TMC register block */
static void __iomem *tmc_virt;

const struct gen8_snapshot_block_list gen8_0_0_snapshot_block_list = {
	.pre_crashdumper_regs = gen8_0_0_ahb_registers,
	.num_pre_crashdumper_regs = ARRAY_SIZE(gen8_0_0_ahb_registers),
	.debugbus_blocks = gen8_debugbus_blocks,
	.debugbus_blocks_len = ARRAY_SIZE(gen8_debugbus_blocks),
	.gbif_debugbus_blocks = gen8_gbif_debugbus_blocks,
	.gbif_debugbus_blocks_len = ARRAY_SIZE(gen8_gbif_debugbus_blocks),
	.cx_debugbus_blocks = gen8_cx_debugbus_blocks,
	.cx_debugbus_blocks_len = ARRAY_SIZE(gen8_cx_debugbus_blocks),
	.external_core_regs = gen8_0_0_external_core_regs,
	.num_external_core_regs = ARRAY_SIZE(gen8_0_0_external_core_regs),
	.gmu_cx_unsliced_regs = gen8_0_0_gmu_registers,
	.gmu_gx_regs = gen8_gmu_gx_registers,
	.num_gmu_gx_regs = ARRAY_SIZE(gen8_gmu_gx_registers),
	.rscc_regs = gen8_0_0_rscc_rsc_registers,
	.reg_list = gen8_0_0_reg_list,
	.cx_misc_regs = gen8_0_0_cx_misc_registers,
	.shader_blocks = gen8_0_0_shader_blocks,
	.num_shader_blocks = ARRAY_SIZE(gen8_0_0_shader_blocks),
	.cp_clusters = gen8_0_0_cp_clusters,
	.num_cp_clusters = ARRAY_SIZE(gen8_0_0_cp_clusters),
	.clusters = gen8_0_0_mvc_clusters,
	.num_clusters = ARRAY_SIZE(gen8_0_0_mvc_clusters),
	.sptp_clusters = gen8_0_0_sptp_clusters,
	.num_sptp_clusters = ARRAY_SIZE(gen8_0_0_sptp_clusters),
	.index_registers = gen8_0_0_cp_indexed_reg_list,
	.index_registers_len = ARRAY_SIZE(gen8_0_0_cp_indexed_reg_list),
	.mempool_index_registers = gen8_0_0_cp_mempool_reg_list,
	.mempool_index_registers_len = ARRAY_SIZE(gen8_0_0_cp_mempool_reg_list),
};

const struct gen8_snapshot_block_list gen8_3_0_snapshot_block_list = {
	.pre_crashdumper_regs = gen8_0_0_ahb_registers,
	.num_pre_crashdumper_regs = ARRAY_SIZE(gen8_0_0_ahb_registers),
	.debugbus_blocks = gen8_3_0_debugbus_blocks,
	.debugbus_blocks_len = ARRAY_SIZE(gen8_3_0_debugbus_blocks),
	.gbif_debugbus_blocks = gen8_gbif_debugbus_blocks,
	.gbif_debugbus_blocks_len = ARRAY_SIZE(gen8_gbif_debugbus_blocks),
	.cx_debugbus_blocks = gen8_cx_debugbus_blocks,
	.cx_debugbus_blocks_len = ARRAY_SIZE(gen8_cx_debugbus_blocks),
	.external_core_regs = gen8_3_0_external_core_regs,
	.num_external_core_regs = ARRAY_SIZE(gen8_3_0_external_core_regs),
	.gmu_cx_unsliced_regs = gen8_3_0_gmu_registers,
	.gmu_gx_regs = gen8_3_0_gmu_gx_regs,
	.num_gmu_gx_regs = ARRAY_SIZE(gen8_3_0_gmu_gx_regs),
	.rscc_regs = gen8_3_0_rscc_rsc_registers,
	.reg_list = gen8_0_0_reg_list,
	.cx_misc_regs = gen8_0_0_cx_misc_registers,
	.shader_blocks = gen8_3_0_shader_blocks,
	.num_shader_blocks = ARRAY_SIZE(gen8_3_0_shader_blocks),
	.cp_clusters = gen8_3_0_cp_clusters,
	.num_cp_clusters = ARRAY_SIZE(gen8_3_0_cp_clusters),
	.clusters = gen8_3_0_mvc_clusters,
	.num_clusters = ARRAY_SIZE(gen8_3_0_mvc_clusters),
	.sptp_clusters = gen8_3_0_sptp_clusters,
	.num_sptp_clusters = ARRAY_SIZE(gen8_3_0_sptp_clusters),
	.index_registers = gen8_3_0_cp_indexed_reg_list,
	.index_registers_len = ARRAY_SIZE(gen8_3_0_cp_indexed_reg_list),
	.mempool_index_registers = gen8_0_0_cp_mempool_reg_list,
	.mempool_index_registers_len = ARRAY_SIZE(gen8_0_0_cp_mempool_reg_list),
};

const struct gen8_snapshot_block_list gen8_6_0_snapshot_block_list = {
	.pre_crashdumper_regs = gen8_0_0_ahb_registers,
	.num_pre_crashdumper_regs = ARRAY_SIZE(gen8_0_0_ahb_registers),
	.debugbus_blocks = gen8_6_0_debugbus_blocks,
	.debugbus_blocks_len = ARRAY_SIZE(gen8_6_0_debugbus_blocks),
	.gbif_debugbus_blocks = gen8_gbif_debugbus_blocks,
	.gbif_debugbus_blocks_len = ARRAY_SIZE(gen8_gbif_debugbus_blocks),
	.cx_debugbus_blocks = gen8_cx_debugbus_blocks,
	.cx_debugbus_blocks_len = ARRAY_SIZE(gen8_cx_debugbus_blocks),
	.external_core_regs = gen8_6_0_external_core_regs,
	.num_external_core_regs = ARRAY_SIZE(gen8_6_0_external_core_regs),
	.gmu_cx_unsliced_regs = gen8_6_0_gmu_registers,
	.gmu_gx_regs = gen8_6_0_gmu_gx_registers,
	.num_gmu_gx_regs = ARRAY_SIZE(gen8_6_0_gmu_gx_registers),
	.rscc_regs = gen8_0_0_rscc_rsc_registers,
	.reg_list = gen8_6_0_reg_list,
	.cx_misc_regs = gen8_0_0_cx_misc_registers,
	.shader_blocks = gen8_6_0_shader_blocks,
	.num_shader_blocks = ARRAY_SIZE(gen8_0_0_shader_blocks),
	.cp_clusters = gen8_0_0_cp_clusters,
	.num_cp_clusters = ARRAY_SIZE(gen8_0_0_cp_clusters),
	.clusters = gen8_6_0_mvc_clusters,
	.num_clusters = ARRAY_SIZE(gen8_6_0_mvc_clusters),
	.sptp_clusters = gen8_0_0_sptp_clusters,
	.num_sptp_clusters = ARRAY_SIZE(gen8_0_0_sptp_clusters),
	.index_registers = gen8_0_0_cp_indexed_reg_list,
	.index_registers_len = ARRAY_SIZE(gen8_0_0_cp_indexed_reg_list),
	.mempool_index_registers = gen8_0_0_cp_mempool_reg_list,
	.mempool_index_registers_len = ARRAY_SIZE(gen8_0_0_cp_mempool_reg_list),
};

#define GEN8_SP_READ_SEL_VAL(_sliceid, _location, _pipe, _statetype, _usptp, _sptp) \
				(FIELD_PREP(GENMASK(25, 21), _sliceid) | \
				 FIELD_PREP(GENMASK(20, 18), _location) | \
				 FIELD_PREP(GENMASK(17, 16), _pipe) | \
				 FIELD_PREP(GENMASK(15, 8), _statetype) | \
				 FIELD_PREP(GENMASK(7, 4), _usptp) | \
				 FIELD_PREP(GENMASK(3, 0), _sptp))

#define GEN8_CP_APERTURE_REG_VAL(_sliceid, _pipe, _cluster, _context) \
			(FIELD_PREP(GENMASK(23, 23), 1) | \
			 FIELD_PREP(GENMASK(18, 16), _sliceid) | \
			 FIELD_PREP(GENMASK(15, 12), _pipe) | \
			 FIELD_PREP(GENMASK(11, 8), _cluster) | \
			 FIELD_PREP(GENMASK(5, 4), _context))

#define GEN8_DEBUGBUS_SECTION_SIZE (sizeof(struct kgsl_snapshot_debugbus) \
			+ (GEN8_DEBUGBUS_BLOCK_SIZE << 3))

#define CD_REG_END 0xaaaaaaaa

static u32 CD_WRITE(u64 *ptr, u32 offset, u64 val)
{
	ptr[0] = val;
	ptr[1] = FIELD_PREP(GENMASK(63, 44), offset) | BIT(21) | BIT(0);

	return 2;
}

static u32 CD_READ(u64 *ptr, u32 offset, u32 size, u64 target)
{
	ptr[0] = target;
	ptr[1] = FIELD_PREP(GENMASK(63, 44), offset) | size;

	return 2;
}

static void CD_FINISH(u64 *ptr, u32 offset)
{
	gen8_cd_reg_end = gen8_crashdump_registers->hostptr + offset;
	*gen8_cd_reg_end = CD_REG_END;
	ptr[0] = gen8_crashdump_registers->gpuaddr + offset;
	ptr[1] = FIELD_PREP(GENMASK(63, 44), GEN8_CP_CRASH_DUMP_STATUS) | BIT(0);
	ptr[2] = 0;
	ptr[3] = 0;
}

static bool CD_SCRIPT_CHECK(struct kgsl_device *device)
{
	return (adreno_smmu_is_stalled(ADRENO_DEVICE(device)) ||
		(kgsl_mmu_ctx_terminated_on_fault(&device->mmu)) ||
		(!device->snapshot_crashdumper) ||
		IS_ERR_OR_NULL(gen8_capturescript) ||
		IS_ERR_OR_NULL(gen8_crashdump_registers) ||
		gen8_crashdump_timedout);
}

static bool _gen8_do_crashdump(struct kgsl_device *device)
{
	u32 reg = 0;
	ktime_t timeout;

	if (CD_SCRIPT_CHECK(device))
		return false;

	kgsl_regwrite(device, GEN8_CP_CRASH_DUMP_SCRIPT_BASE_LO,
			lower_32_bits(gen8_capturescript->gpuaddr));
	kgsl_regwrite(device, GEN8_CP_CRASH_DUMP_SCRIPT_BASE_HI,
			upper_32_bits(gen8_capturescript->gpuaddr));
	kgsl_regwrite(device, GEN8_CP_CRASH_DUMP_CNTL, 1);

	timeout = ktime_add_ms(ktime_get(), CP_CRASH_DUMPER_TIMEOUT);

	if (!device->snapshot_atomic)
		might_sleep();
	for (;;) {
		/* make sure we're reading the latest value */
		rmb();
		if ((*gen8_cd_reg_end) != CD_REG_END)
			break;
		if (ktime_compare(ktime_get(), timeout) > 0)
			break;
		/* Wait 1msec to avoid unnecessary looping */
		if (!device->snapshot_atomic)
			usleep_range(100, 1000);
	}

	kgsl_regread(device, GEN8_CP_CRASH_DUMP_STATUS, &reg);

	/*
	 * Writing to the GEN8_CP_CRASH_DUMP_CNTL also resets the
	 * GEN8_CP_CRASH_DUMP_STATUS. Make sure the read above is
	 * complete before we change the value
	 */
	rmb();

	kgsl_regwrite(device, GEN8_CP_CRASH_DUMP_CNTL, 0);

	if (WARN(!(reg & 0x2), "Crashdumper timed out\n")) {
		/*
		 * Gen7 crash dumper script is broken down into multiple chunks
		 * and script will be invoked multiple times to capture snapshot
		 * of different sections of GPU. If crashdumper fails once, it is
		 * highly likely it will fail subsequently as well. Hence update
		 * gen8_crashdump_timedout variable to avoid running crashdumper
		 * after it fails once.
		 */
		gen8_crashdump_timedout = true;
		return false;
	}

	return true;
}

size_t gen8_legacy_snapshot_registers(struct kgsl_device *device,
		 u8 *buf, size_t remain, void *priv)
{
	struct gen8_reg_list_info *info = (struct gen8_reg_list_info *)priv;
	const u32 *ptr = info->regs->regs;
	struct kgsl_snapshot_mvc_regs_v3 *header =
			(struct kgsl_snapshot_mvc_regs_v3 *)buf;
	u32 *data = (u32 *)(buf + sizeof(*header));
	u32 size = (adreno_snapshot_regs_count(ptr) * sizeof(*data)) + sizeof(*header);
	u32 count, k;

	if (remain < size) {
		SNAPSHOT_ERR_NOMEM(device, "REGISTERS");
		return 0;
	}

	header->ctxt_id = 0;
	header->cluster_id = CLUSTER_NONE;
	header->pipe_id = PIPE_NONE;
	header->location_id = UINT_MAX;
	header->sp_id = UINT_MAX;
	header->usptp_id = UINT_MAX;
	header->slice_id = info->regs->slice_region ? info->slice_id : UINT_MAX;

	if (info->regs->sel)
		kgsl_regwrite(device, info->regs->sel->host_reg, info->regs->sel->val);

	if (info->regs->slice_region)
		kgsl_regwrite(device, GEN8_CP_APERTURE_CNTL_HOST, GEN8_CP_APERTURE_REG_VAL
				(info->slice_id, 0, 0, 0));

	/* Make sure the previous writes are posted before reading */
	mb();

	for (ptr = info->regs->regs; ptr[0] != UINT_MAX; ptr += 2) {
		count = REG_COUNT(ptr);

		if (count == 1)
			*data++ = ptr[0];
		else {
			*data++ = ptr[0] | (1 << 31);
			*data++ = ptr[1];
		}
		for (k = ptr[0]; k <= ptr[1]; k++)
			kgsl_regread(device, k, data++);
	}

	return size;
}

static size_t gen8_snapshot_registers(struct kgsl_device *device, u8 *buf,
		size_t remain, void *priv)
{
	struct gen8_reg_list_info *info = (struct gen8_reg_list_info *)priv;
	const u32 *ptr = info->regs->regs;
	struct kgsl_snapshot_mvc_regs_v3 *header =
			(struct kgsl_snapshot_mvc_regs_v3 *)buf;
	u32 *data = (u32 *)(buf + sizeof(*header));
	u32 *src;
	u32 cnt;
	u32 size = (adreno_snapshot_regs_count(ptr) * sizeof(*data)) + sizeof(*header);

	if (remain < size) {
		SNAPSHOT_ERR_NOMEM(device, "REGISTERS");
		return 0;
	}

	header->ctxt_id = 0;
	header->cluster_id = CLUSTER_NONE;
	header->pipe_id = PIPE_NONE;
	header->location_id = UINT_MAX;
	header->sp_id = UINT_MAX;
	header->usptp_id = UINT_MAX;
	header->slice_id = info->regs->slice_region ? info->slice_id : UINT_MAX;

	src = gen8_crashdump_registers->hostptr + info->offset;

	for (ptr = info->regs->regs; ptr[0] != UINT_MAX; ptr += 2) {
		cnt = REG_COUNT(ptr);

		if (cnt == 1)
			*data++ = ptr[0];
		else {
			*data++ = BIT(31) | ptr[0];
			*data++ = ptr[1];
		}
		memcpy(data, src, cnt << 2);
		data += cnt;
		src += cnt;
	}

	/* Return the size of the section */
	return size;
}

static size_t gen8_legacy_snapshot_shader(struct kgsl_device *device,
				u8 *buf, size_t remain, void *priv)
{
	struct kgsl_snapshot_shader_v3 *header =
		(struct kgsl_snapshot_shader_v3 *) buf;
	struct gen8_shader_block_info *info = (struct gen8_shader_block_info *) priv;
	struct gen8_shader_block *block = info->block;
	u32 *data = (u32 *)(buf + sizeof(*header));
	u32 read_sel, i;

	if (remain < (sizeof(*header) + (block->size << 2))) {
		SNAPSHOT_ERR_NOMEM(device, "SHADER MEMORY");
		return 0;
	}

	header->type = block->statetype;
	header->slice_id = block->slice_region ? info->slice_id : UINT_MAX;
	header->sp_index = info->sp_id;
	header->usptp = info->usptp;
	header->pipe_id = block->pipeid;
	header->location = block->location;
	header->ctxt_id = 1;
	header->size = block->size;

	read_sel = GEN8_SP_READ_SEL_VAL(info->slice_id, block->location, block->pipeid,
				block->statetype, info->usptp, info->sp_id);

	kgsl_regwrite(device, GEN8_SP_READ_SEL, read_sel);

	/*
	 * An explicit barrier is needed so that reads do not happen before
	 * the register write.
	 */
	mb();

	for (i = 0; i < block->size; i++)
		data[i] = kgsl_regmap_read(&device->regmap, GEN8_SP_AHB_READ_APERTURE + i);

	return (sizeof(*header) + (block->size << 2));
}

static size_t gen8_snapshot_shader_memory(struct kgsl_device *device,
		u8 *buf, size_t remain, void *priv)
{
	struct kgsl_snapshot_shader_v3 *header =
		(struct kgsl_snapshot_shader_v3 *) buf;
	struct gen8_shader_block_info *info = (struct gen8_shader_block_info *) priv;
	struct gen8_shader_block *block = info->block;
	u32 *data = (u32 *) (buf + sizeof(*header));

	if (remain < (sizeof(*header) + (block->size << 2))) {
		SNAPSHOT_ERR_NOMEM(device, "SHADER MEMORY");
		return 0;
	}

	header->type = block->statetype;
	header->slice_id = block->slice_region ? info->slice_id : UINT_MAX;
	header->sp_index = info->sp_id;
	header->usptp = info->usptp;
	header->pipe_id = block->pipeid;
	header->location = block->location;
	header->ctxt_id = 1;
	header->size = block->size;

	memcpy(data, gen8_crashdump_registers->hostptr + info->offset,
			(block->size << 2));

	return (sizeof(*header) + (block->size << 2));
}

static void qdss_regwrite(void __iomem *regbase, u32 offsetbytes, u32 value)
{
	void __iomem *reg;

	reg = regbase + offsetbytes;

	 /* Ensure previous write is committed */
	wmb();
	__raw_writel(value, reg);
}

static u32 qdss_regread(void __iomem *regbase, u32 offsetbytes)
{
	void __iomem *reg;
	u32 val;

	reg = regbase + offsetbytes;
	val = __raw_readl(reg);

	/* Make sure memory is updated before next access */
	rmb();
	return val;
}

static size_t gen8_snapshot_trace_buffer_gfx_trace(struct kgsl_device *device,
		u8 *buf, size_t remain, void *priv)
{
	u32 start_idx = 0, status = 0, count = 0, wrap_count = 0, write_ptr = 0;
	struct kgsl_snapshot_trace_buffer *header =
			(struct kgsl_snapshot_trace_buffer *) buf;
	u32 *data = (u32 *)(buf + sizeof(*header));
	struct gen8_trace_buffer_info *info =
				(struct gen8_trace_buffer_info *) priv;

	if (remain < SZ_2K + sizeof(*header)) {
		SNAPSHOT_ERR_NOMEM(device, "TRACE 2K BUFFER");
		return 0;
	}

	memcpy(header->ping_blk, info->ping_blk, sizeof(header->ping_blk));
	memcpy(header->ping_idx, info->ping_idx, sizeof(header->ping_idx));
	header->granularity = info->granularity;
	header->segment = info->segment;
	header->dbgc_ctrl = info->dbgc_ctrl;

	/* Read the status of trace buffer to determine if it's full or empty */
	kgsl_regread(device, GEN8_DBGC_TRACE_BUFFER_STATUS, &status);

	/*
	 * wrap_count and write ptr are part of status.
	 * if status is 0 => wrap_count = 0 and write ptr = 0 buffer is empty.
	 * if status is non zero and wrap count is 0 read partial buffer.
	 * if wrap count in non zero read entier 2k buffer.
	 * Always read the oldest data available.
	 */

	/* if status is 0 then buffer is empty */
	if (!status) {
		header->size = 0;
		return sizeof(*header);
	}

	/* Number of times the circular buffer has wrapped around */
	wrap_count = FIELD_GET(GENMASK(31, 12), status);
	write_ptr = FIELD_GET(GENMASK(8, 0), status);

	/* Read partial buffer starting from 0 */
	if (!wrap_count) {
		/* No of dwords to read : (write ptr - 0) of indexed register */
		count = write_ptr;
		header->size = count << 2;
		start_idx = 0;
	} else {
		/* Read entire 2k buffer starting from write ptr */
		start_idx = write_ptr + 1;
		count = SZ_512;
		header->size = SZ_2K;
	}

	kgsl_regmap_read_indexed_interleaved(&device->regmap,
		GEN8_DBGC_DBG_TRACE_BUFFER_RD_ADDR, GEN8_DBGC_DBG_TRACE_BUFFER_RD_DATA, data,
			start_idx, count);

	return (sizeof(*header) + header->size);
}

static size_t gen8_snapshot_trace_buffer_etb(struct kgsl_device *device,
		u8 *buf, size_t remain, void *priv)
{
	u32 read_ptr, count, write_ptr, val, idx = 0;
	struct kgsl_snapshot_trace_buffer *header = (struct kgsl_snapshot_trace_buffer *) buf;
	u32 *data = (u32 *)(buf + sizeof(*header));
	struct gen8_trace_buffer_info *info = (struct gen8_trace_buffer_info *) priv;

	/* Unlock ETB buffer */
	qdss_regwrite(tmc_virt, QDSS_AOSS_APB_TMC_LAR, 0xC5ACCE55);

	/* Make sure unlock goes through before proceeding further */
	mb();

	/* Flush the QDSS pipeline to ensure completion of pending write to buffer */
	val = qdss_regread(tmc_virt, QDSS_AOSS_APB_TMC_FFCR);
	qdss_regwrite(tmc_virt, QDSS_AOSS_APB_TMC_FFCR, val | 0x40);

	/* Make sure pipeline is flushed before we get read and write pointers */
	mb();

	/* Disable ETB */
	qdss_regwrite(tmc_virt, QDSS_AOSS_APB_TMC_CTRL, 0);

	/* Set to circular mode */
	qdss_regwrite(tmc_virt, QDSS_AOSS_APB_TMC_MODE, 0);

	/* Ensure buffer is set to circular mode before accessing it */
	mb();

	/* Size of buffer is specified in register TMC_RSZ */
	count = qdss_regread(tmc_virt, QDSS_AOSS_APB_TMC_RSZ) << 2;
	read_ptr = qdss_regread(tmc_virt, QDSS_AOSS_APB_TMC_RRP);
	write_ptr = qdss_regread(tmc_virt, QDSS_AOSS_APB_TMC_RWP);

	/* ETB buffer if full read_ptr will be equal to write_ptr else write_ptr leads read_ptr */
	count = (read_ptr == write_ptr) ? count : (write_ptr - read_ptr);

	if (remain < count + sizeof(*header)) {
		SNAPSHOT_ERR_NOMEM(device, "ETB BUFFER");
		return 0;
	}

	/*
	 * Read pointer is 4 byte aligned and write pointer is 2 byte aligned
	 * We read 4 bytes of data in one iteration below so aligin it down
	 * to 4 bytes.
	 */
	count = ALIGN_DOWN(count, 4);

	header->size = count;
	header->dbgc_ctrl = info->dbgc_ctrl;
	memcpy(header->ping_blk, info->ping_blk, sizeof(header->ping_blk));
	memcpy(header->ping_idx, info->ping_idx, sizeof(header->ping_idx));
	header->granularity = info->granularity;
	header->segment = info->segment;

	while (count != 0) {
		/* This indexed register auto increments index as we read */
		data[idx++] = qdss_regread(tmc_virt, QDSS_AOSS_APB_TMC_RRD);
		count = count - 4;
	}

	return (sizeof(*header) + header->size);
}

static void gen8_snapshot_trace_buffer(struct kgsl_device *device,
				struct kgsl_snapshot *snapshot)
{
	u32 val_tmc_ctrl = 0, val_etr_ctrl = 0, val_etr1_ctrl = 0;
	u32 i = 0, sel_gx = 0, sel_cx = 0, val_gx = 0, val_cx = 0, val = 0;
	struct gen8_trace_buffer_info info;
	struct resource *res1, *res2;
	struct clk *clk;
	int ret;
	void __iomem *etr_virt;

	/*
	 * Data can be collected from CX_DBGC or DBGC and it's mutually exclusive.
	 * Read the necessary select registers and determine the source of data.
	 * This loop reads SEL_A to SEL_D of both CX_DBGC and DBGC and accordingly
	 * updates the header information of trace buffer section.
	 */
	for (i = 0; i < TRACE_BUF_NUM_SIG; i++) {
		kgsl_regread(device, GEN8_DBGC_CFG_DBGBUS_SEL_A + i, &sel_gx);
		kgsl_regread(device, GEN8_CX_DBGC_CFG_DBGBUS_SEL_A + i, &sel_cx);
		val_gx |= sel_gx;
		val_cx |= sel_cx;
		info.ping_idx[i] = FIELD_GET(GENMASK(7, 0), (sel_gx | sel_cx));
		info.ping_blk[i] = FIELD_GET(GENMASK(24, 16), (sel_gx | sel_cx));
	}

	/* Zero the header if not programmed to export any buffer */
	if (!val_gx && !val_cx) {
		kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_TRACE_BUFFER,
			snapshot, NULL, &info);
		return;
	}

	/* Enable APB clock to read data from trace buffer */
	clk = clk_get(&device->pdev->dev, "apb_pclk");

	if (IS_ERR(clk)) {
		dev_err(device->dev, "Unable to get QDSS clock\n");
		return;
	}

	ret = clk_prepare_enable(clk);

	if (ret) {
		dev_err(device->dev, "QDSS Clock enable error: %d\n", ret);
		clk_put(clk);
		return;
	}

	res1 = platform_get_resource_byname(device->pdev, IORESOURCE_MEM, "qdss_etr");
	res2 = platform_get_resource_byname(device->pdev, IORESOURCE_MEM, "qdss_tmc");

	if (!res1 || !res2)
		goto err_clk_put;

	etr_virt = ioremap(res1->start, resource_size(res1));
	tmc_virt = ioremap(res2->start, resource_size(res2));

	if (!etr_virt || !tmc_virt)
		goto err_unmap;

	/*
	 * Update header information based on source of data, read necessary CNTLT registers
	 * for granularity and segment information.
	 */
	if (val_gx) {
		info.dbgc_ctrl = GX_DBGC;
		kgsl_regread(device, GEN8_DBGC_CFG_DBGBUS_CNTLT, &val);
	} else {
		info.dbgc_ctrl = CX_DBGC;
		kgsl_regread(device, GEN8_CX_DBGC_CFG_DBGBUS_CNTLT, &val);
	}

	info.granularity = FIELD_GET(GENMASK(14, 12), val);
	info.segment = FIELD_GET(GENMASK(31, 28), val);

	val_tmc_ctrl = qdss_regread(tmc_virt, QDSS_AOSS_APB_TMC_CTRL);

	/*
	 * Incase TMC CTRL is 0 and val_cx is non zero dump empty buffer.
	 * Incase TMC CTRL is 0 and val_gx is non zero dump 2k gfx buffer.
	 * 2k buffer is not present for CX blocks.
	 * Incase both ETR's CTRL is 0 Dump ETB QDSS buffer and disable QDSS.
	 * Incase either ETR's CTRL is 1 Disable QDSS dumping ETB buffer to DDR.
	 */
	if (!val_tmc_ctrl) {
		if (val_gx)
			kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_TRACE_BUFFER,
				snapshot, gen8_snapshot_trace_buffer_gfx_trace, &info);
		else
			kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_TRACE_BUFFER,
					snapshot, NULL, &info);
	} else {
		val_etr_ctrl = qdss_regread(etr_virt, QDSS_AOSS_APB_ETR_CTRL);
		val_etr1_ctrl = qdss_regread(etr_virt, QDSS_AOSS_APB_ETR1_CTRL);
		if (!val_etr_ctrl && !val_etr1_ctrl)
			kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_TRACE_BUFFER,
				snapshot, gen8_snapshot_trace_buffer_etb, &info);
		qdss_regwrite(tmc_virt, QDSS_AOSS_APB_TMC_CTRL, 0);
	}

err_unmap:
	iounmap(tmc_virt);
	iounmap(etr_virt);

err_clk_put:
	clk_disable_unprepare(clk);
	clk_put(clk);
}

static bool gen8_snapshot_shader(struct kgsl_device *device,
				struct kgsl_snapshot *snapshot)
{
	struct gen8_shader_block_info info = {0};
	u64 *ptr;
	u32 offset = 0;
	struct gen8_shader_block *shader_blocks = gen8_snapshot_block_list->shader_blocks;
	size_t num_shader_blocks = gen8_snapshot_block_list->num_shader_blocks;
	u32 i, sp, usptp, slice;

	if (CD_SCRIPT_CHECK(device)) {
		for (i = 0; i < num_shader_blocks; i++) {
			struct gen8_shader_block *block = &shader_blocks[i];
			u32 slices = NUMBER_OF_SLICES(block->slice_region, ADRENO_DEVICE(device));

			for (slice = 0; slice < slices; slice++) {
				for (sp = 0; sp < block->num_sps; sp++) {
					for (usptp = 0; usptp < block->num_usptps; usptp++) {
						info.block = block;
						info.sp_id = sp;
						info.usptp = usptp;
						info.slice_id = slice;
						info.offset = offset;
						offset += block->size << 2;

						/* Shader working/shadow memory */
						kgsl_snapshot_add_section(device,
							KGSL_SNAPSHOT_SECTION_SHADER_V3,
							snapshot, gen8_legacy_snapshot_shader,
							&info);
					}
				}
			}
		}

		return true;
	}

	for (i = 0; i < num_shader_blocks; i++) {
		struct gen8_shader_block *block = &shader_blocks[i];
		u32 slices = NUMBER_OF_SLICES(block->slice_region, ADRENO_DEVICE(device));

		/* Build the crash script */
		ptr = gen8_capturescript->hostptr;
		offset = 0;

		for (slice = 0; slice < slices; slice++) {
			for (sp = 0; sp < block->num_sps; sp++) {
				for (usptp = 0; usptp < block->num_usptps; usptp++) {
					/* Program the aperture */
					ptr += CD_WRITE(ptr, GEN8_SP_READ_SEL,
						GEN8_SP_READ_SEL_VAL(slice, block->location,
						block->pipeid, block->statetype, usptp, sp));

					/* Read all the data in one chunk */
					ptr += CD_READ(ptr, GEN8_SP_AHB_READ_APERTURE, block->size,
						gen8_crashdump_registers->gpuaddr + offset);
					offset += block->size << 2;
				}
			}
		}
		/* Marker for end of script */
		CD_FINISH(ptr, offset);

		/* Try to run the crash dumper and bail if it times out */
		if (!_gen8_do_crashdump(device))
			return false;

		offset = 0;
		for (slice = 0; slice < slices; slice++) {
			for (sp = 0; sp < block->num_sps; sp++) {
				for (usptp = 0; usptp < block->num_usptps; usptp++) {
					info.block = block;
					info.sp_id = sp;
					info.usptp = usptp;
					info.slice_id = slice;
					info.offset = offset;
					offset += block->size << 2;

					/* Shader working/shadow memory */
					kgsl_snapshot_add_section(device,
					KGSL_SNAPSHOT_SECTION_SHADER_V3, snapshot,
					gen8_snapshot_shader_memory, &info);
				}
			}
		}
	}
	return true;
}

static void gen8_rmw_aperture(struct kgsl_device *device,
	u32 offsetwords, u32 mask, u32 val, u32 pipe, u32 slice_id, u32 use_slice_id)
{
	gen8_host_aperture_set(ADRENO_DEVICE(device), pipe, slice_id, use_slice_id);

	kgsl_regmap_rmw(&device->regmap, offsetwords, mask, val);
}

static void gen8_snapshot_mempool(struct kgsl_device *device,
				struct kgsl_snapshot *snapshot)
{
	struct gen8_cp_indexed_reg *cp_indexed_reg;
	size_t mempool_index_registers_len  = gen8_snapshot_block_list->mempool_index_registers_len;
	u32 i, j, slice;

	for (i = 0; i < mempool_index_registers_len; i++) {
		cp_indexed_reg = &gen8_snapshot_block_list->mempool_index_registers[i];
		slice = NUMBER_OF_SLICES(cp_indexed_reg->slice_region, ADRENO_DEVICE(device));

		for (j = 0; j < slice; j++) {

			/* set CP_CHICKEN_DBG[StabilizeMVC] to stabilize it while dumping */
			gen8_rmw_aperture(device, GEN8_CP_CHICKEN_DBG_PIPE, 0x4, 0x4,
				cp_indexed_reg->pipe_id, 0, 0);

			gen8_rmw_aperture(device, GEN8_CP_SLICE_CHICKEN_DBG_PIPE, 0x4, 0x4,
				cp_indexed_reg->pipe_id, j, 1);

			kgsl_snapshot_indexed_registers_v2(device, snapshot,
				cp_indexed_reg->addr, cp_indexed_reg->data,
				0, cp_indexed_reg->size, cp_indexed_reg->pipe_id,
				SLICE_ID(cp_indexed_reg->slice_region, j));

			/* Reset CP_CHICKEN_DBG[StabilizeMVC] once we are done */
			gen8_rmw_aperture(device, GEN8_CP_CHICKEN_DBG_PIPE, 0x4, 0x0,
				cp_indexed_reg->pipe_id, 0, 0);

			gen8_rmw_aperture(device, GEN8_CP_SLICE_CHICKEN_DBG_PIPE, 0x4, 0x0,
				cp_indexed_reg->pipe_id, j, 1);
		}
	}

	/* Clear aperture register */
	gen8_host_aperture_set(ADRENO_DEVICE(device), 0, 0, 0);
}

static u32 gen8_read_dbgahb(struct kgsl_device *device,
				u32 regbase, u32 reg)
{
	u32 val;

	kgsl_regread(device, (GEN8_SP_AHB_READ_APERTURE + reg - regbase), &val);
	return val;
}

static size_t gen8_legacy_snapshot_cluster_dbgahb(struct kgsl_device *device,
				u8 *buf, size_t remain, void *priv)
{
	struct kgsl_snapshot_mvc_regs_v3 *header =
				(struct kgsl_snapshot_mvc_regs_v3 *)buf;
	struct gen8_sptp_cluster_registers_info *info =
			(struct gen8_sptp_cluster_registers_info *)priv;
	const u32 *ptr = info->cluster->regs;
	u32 *data = (u32 *)(buf + sizeof(*header));
	u32 read_sel, j;
	u32 size = adreno_snapshot_regs_count(ptr) * sizeof(*data);

	if (remain < (sizeof(*header) + size)) {
		SNAPSHOT_ERR_NOMEM(device, "MVC REGISTERS");
		return 0;
	}

	header->ctxt_id = info->context_id;
	header->cluster_id = info->cluster_id;
	header->pipe_id = info->pipe_id;
	header->location_id = info->location_id;
	header->sp_id = info->sp_id;
	header->usptp_id = info->usptp_id;
	header->slice_id = info->cluster->slice_region ? info->slice_id : UINT_MAX;

	read_sel = GEN8_SP_READ_SEL_VAL(info->slice_id, info->location_id,
			info->pipe_id, info->statetype_id, info->usptp_id, info->sp_id);

	kgsl_regwrite(device, GEN8_SP_READ_SEL, read_sel);

	/*
	 * An explicit barrier is needed so that reads do not happen before
	 * the register write.
	 */
	mb();

	for (; ptr[0] != UINT_MAX; ptr += 2) {
		u32 count = REG_COUNT(ptr);

		if (count == 1)
			*data++ = ptr[0];
		else {
			*data++ = ptr[0] | (1 << 31);
			*data++ = ptr[1];
		}
		for (j = ptr[0]; j <= ptr[1]; j++)
			*data++ = gen8_read_dbgahb(device, info->cluster->regbase, j);
	}

	return (size + sizeof(*header));
}

static size_t gen8_snapshot_cluster_dbgahb(struct kgsl_device *device, u8 *buf,
				size_t remain, void *priv)
{
	struct kgsl_snapshot_mvc_regs_v3 *header =
				(struct kgsl_snapshot_mvc_regs_v3 *)buf;
	struct gen8_sptp_cluster_registers_info *info =
				(struct gen8_sptp_cluster_registers_info *)priv;
	const u32 *ptr = info->cluster->regs;
	u32 *data = (u32 *)(buf + sizeof(*header));
	u32 *src;
	u32 size = adreno_snapshot_regs_count(ptr) * sizeof(*data);

	if (remain < (sizeof(*header) + size)) {
		SNAPSHOT_ERR_NOMEM(device, "REGISTERS");
		return 0;
	}

	header->ctxt_id = info->context_id;
	header->cluster_id = info->cluster_id;
	header->pipe_id = info->pipe_id;
	header->location_id = info->location_id;
	header->sp_id = info->sp_id;
	header->usptp_id = info->usptp_id;
	header->slice_id = info->cluster->slice_region ? info->slice_id : UINT_MAX;

	src = gen8_crashdump_registers->hostptr + info->offset;

	for (ptr = info->cluster->regs; ptr[0] != UINT_MAX; ptr += 2) {
		u32 cnt = REG_COUNT(ptr);

		if (cnt == 1)
			*data++ = ptr[0];
		else {
			*data++ = ptr[0] | (1 << 31);
			*data++ = ptr[1];
		}
		memcpy(data, src, cnt << 2);
		data += cnt;
		src += cnt;
	}

	return (size + sizeof(*header));
}

static bool gen8_snapshot_dbgahb_regs(struct kgsl_device *device,
			struct kgsl_snapshot *snapshot)
{
	u32 i, j, sp, usptp, count, slice;
	u64 *ptr, offset = 0;
	struct gen8_sptp_cluster_registers_info info = {0};
	struct gen8_sptp_cluster_registers *sptp_clusters = gen8_snapshot_block_list->sptp_clusters;
	size_t num_sptp_clusters = gen8_snapshot_block_list->num_sptp_clusters;

	if (CD_SCRIPT_CHECK(device)) {
		for (i = 0; i < num_sptp_clusters; i++) {
			struct gen8_sptp_cluster_registers *cluster = &sptp_clusters[i];

			slice = NUMBER_OF_SLICES(cluster->slice_region, ADRENO_DEVICE(device));
			for (sp = 0; sp < cluster->num_sps; sp++) {
				for (usptp = 0; usptp < cluster->num_usptps; usptp++) {
					for (j = 0; j < slice; j++) {
						info.cluster = cluster;
						info.location_id = cluster->location_id;
						info.pipe_id = cluster->pipe_id;
						info.usptp_id = usptp;
						info.sp_id = sp;
						info.slice_id = SLICE_ID(cluster->slice_region, j);
						info.statetype_id = cluster->statetype;
						info.cluster_id = cluster->cluster_id;
						info.context_id = cluster->context_id;
						kgsl_snapshot_add_section(device,
							KGSL_SNAPSHOT_SECTION_MVC_V3, snapshot,
							gen8_legacy_snapshot_cluster_dbgahb, &info);
					}
				}
			}
		}
		return true;
	}

	for (i = 0; i < num_sptp_clusters; i++) {
		struct gen8_sptp_cluster_registers *cluster = &sptp_clusters[i];

		slice = NUMBER_OF_SLICES(cluster->slice_region, ADRENO_DEVICE(device));

		cluster->offset = offset;

		for (sp = 0; sp < cluster->num_sps; sp++) {
			for (usptp = 0; usptp < cluster->num_usptps; usptp++) {
				for (j = 0; j < slice; j++) {
					const u32 *regs = cluster->regs;

					info.cluster = cluster;
					info.location_id = cluster->location_id;
					info.pipe_id = cluster->pipe_id;
					info.usptp_id = usptp;
					info.sp_id = sp;
					info.slice_id = SLICE_ID(cluster->slice_region, j);
					info.statetype_id = cluster->statetype;
					info.cluster_id = cluster->cluster_id;
					info.context_id = cluster->context_id;
					info.offset = offset;

					/* Build the crash script */
					ptr = gen8_capturescript->hostptr;

					/* Program the aperture */
					ptr += CD_WRITE(ptr, GEN8_SP_READ_SEL, GEN8_SP_READ_SEL_VAL
						(j, cluster->location_id, cluster->pipe_id,
						cluster->statetype, usptp, sp));

					for (; regs[0] != UINT_MAX; regs += 2) {
						count = REG_COUNT(regs);
						ptr += CD_READ(ptr, (GEN8_SP_AHB_READ_APERTURE +
							regs[0] - cluster->regbase), count,
							(gen8_crashdump_registers->gpuaddr +
								offset));

						offset += count * sizeof(u32);
					}
					/* Marker for end of script */
					CD_FINISH(ptr, offset);

					/* Try to run the crash dumper and bail if it times out */
					if (!_gen8_do_crashdump(device))
						return false;

					kgsl_snapshot_add_section(device,
						KGSL_SNAPSHOT_SECTION_MVC_V3, snapshot,
						gen8_snapshot_cluster_dbgahb, &info);
				}
			}
		}
	}
	return true;
}

static size_t gen8_legacy_snapshot_mvc(struct kgsl_device *device, u8 *buf,
				size_t remain, void *priv)
{
	struct kgsl_snapshot_mvc_regs_v3 *header =
					(struct kgsl_snapshot_mvc_regs_v3 *)buf;
	u32 *data = (u32 *)(buf + sizeof(*header));
	struct gen8_cluster_registers_info *info =
			(struct gen8_cluster_registers_info *)priv;
	const u32 *ptr = info->cluster->regs;
	u32 size = adreno_snapshot_regs_count(ptr) * sizeof(*data);
	u32 j;

	if (remain < (sizeof(*header) + size)) {
		SNAPSHOT_ERR_NOMEM(device, "MVC REGISTERS");
		return 0;
	}

	header->ctxt_id = (info->context_id == STATE_FORCE_CTXT_1) ? 1 : 0;
	header->cluster_id = info->cluster_id;
	header->pipe_id = info->pipe_id;
	header->location_id = UINT_MAX;
	header->sp_id = UINT_MAX;
	header->usptp_id = UINT_MAX;
	header->slice_id = info->cluster->slice_region ? info->slice_id : UINT_MAX;

	/*
	 * Set the AHB control for the Host to read from the
	 * cluster/context for this iteration.
	 */
	kgsl_regwrite(device, GEN8_CP_APERTURE_CNTL_HOST, GEN8_CP_APERTURE_REG_VAL
			(info->slice_id, info->pipe_id, info->cluster_id, info->context_id));

	if (info->cluster->sel)
		kgsl_regwrite(device, info->cluster->sel->host_reg, info->cluster->sel->val);

	/* Make sure the previous writes are posted before reading */
	mb();

	for (; ptr[0] != UINT_MAX; ptr += 2) {
		u32 count = REG_COUNT(ptr);

		if (count == 1)
			*data++ = ptr[0];
		else {
			*data++ = ptr[0] | (1 << 31);
			*data++ = ptr[1];
		}
		for (j = ptr[0]; j <= ptr[1]; j++)
			kgsl_regread(device, j, data++);
	}

	return (size + sizeof(*header));
}

static size_t gen8_snapshot_mvc(struct kgsl_device *device, u8 *buf,
				size_t remain, void *priv)
{
	struct kgsl_snapshot_mvc_regs_v3 *header =
				(struct kgsl_snapshot_mvc_regs_v3 *)buf;
	struct gen8_cluster_registers_info *info =
			(struct gen8_cluster_registers_info *)priv;
	const u32 *ptr = info->cluster->regs;
	u32 *data = (u32 *)(buf + sizeof(*header));
	u32 *src;
	u32 cnt;
	u32 size = adreno_snapshot_regs_count(ptr) * sizeof(*data);

	if (remain < (sizeof(*header) + size)) {
		SNAPSHOT_ERR_NOMEM(device, "MVC REGISTERS");
		return 0;
	}

	header->ctxt_id = (info->context_id == STATE_FORCE_CTXT_1) ? 1 : 0;
	header->cluster_id = info->cluster_id;
	header->pipe_id = info->pipe_id;
	header->location_id = UINT_MAX;
	header->sp_id = UINT_MAX;
	header->usptp_id = UINT_MAX;
	header->slice_id = info->cluster->slice_region ? info->slice_id : UINT_MAX;

	src = gen8_crashdump_registers->hostptr + info->offset;

	for (; ptr[0] != UINT_MAX; ptr += 2) {
		cnt = REG_COUNT(ptr);

		if (cnt == 1)
			*data++ = ptr[0];
		else {
			*data++ = ptr[0] | (1 << 31);
			*data++ = ptr[1];
		}
		memcpy(data, src, cnt << 2);
		src += cnt;
		data += cnt;
	}

	return (size + sizeof(*header));
}

static bool gen8_snapshot_mvc_regs(struct kgsl_device *device,
				struct kgsl_snapshot *snapshot,
				struct gen8_cluster_registers *clusters,
				size_t num_cluster)
{
	u32 i, j;
	u64 *ptr, offset = 0;
	u32 count, slice;
	struct gen8_cluster_registers_info info = {0};

	if (CD_SCRIPT_CHECK(device)) {
		for (i = 0; i < num_cluster; i++) {
			struct gen8_cluster_registers *cluster = &clusters[i];

			slice = NUMBER_OF_SLICES(cluster->slice_region, ADRENO_DEVICE(device));
			for (j = 0; j < slice; j++) {
				info.cluster = cluster;
				info.pipe_id = cluster->pipe_id;
				info.cluster_id = cluster->cluster_id;
				info.context_id = cluster->context_id;
				info.slice_id = SLICE_ID(cluster->slice_region, j);
				kgsl_snapshot_add_section(device,
					KGSL_SNAPSHOT_SECTION_MVC_V3, snapshot,
					gen8_legacy_snapshot_mvc, &info);
			}
		}
		return true;
	}

	for (i = 0; i < num_cluster; i++) {
		struct gen8_cluster_registers *cluster = &clusters[i];

		slice = NUMBER_OF_SLICES(cluster->slice_region, ADRENO_DEVICE(device));
		cluster->offset = offset;

		for (j = 0; j < slice; j++) {
			const u32 *regs = cluster->regs;

			info.cluster = cluster;
			info.pipe_id = cluster->pipe_id;
			info.cluster_id = cluster->cluster_id;
			info.context_id = cluster->context_id;
			info.slice_id = SLICE_ID(cluster->slice_region, j);
			info.offset = offset;

			/* Build the crash script */
			ptr = gen8_capturescript->hostptr;

			ptr += CD_WRITE(ptr, GEN8_CP_APERTURE_CNTL_CD, GEN8_CP_APERTURE_REG_VAL
				(j, cluster->pipe_id, cluster->cluster_id, cluster->context_id));

			if (cluster->sel)
				ptr += CD_WRITE(ptr, cluster->sel->cd_reg, cluster->sel->val);

			for (; regs[0] != UINT_MAX; regs += 2) {
				count = REG_COUNT(regs);

				ptr += CD_READ(ptr, regs[0],
					count, (gen8_crashdump_registers->gpuaddr + offset));

				offset += count * sizeof(u32);
			}

			/* Marker for end of script */
			CD_FINISH(ptr, offset);

			/* Try to run the crash dumper and bail if it times out */
			if (!_gen8_do_crashdump(device))
				return false;

			kgsl_snapshot_add_section(device,
				KGSL_SNAPSHOT_SECTION_MVC_V3, snapshot, gen8_snapshot_mvc, &info);
		}
	}
	return true;
}

/* gen8_dbgc_debug_bus_read() - Read data from trace bus */
static void gen8_dbgc_debug_bus_read(struct kgsl_device *device,
	u32 block_id, u32 index, u32 *val)
{
	u32 reg;

	reg = FIELD_PREP(GENMASK(7, 0), index) |
		FIELD_PREP(GENMASK(24, 16), block_id);

	kgsl_regwrite(device, GEN8_DBGC_CFG_DBGBUS_SEL_A, reg);
	kgsl_regwrite(device, GEN8_DBGC_CFG_DBGBUS_SEL_B, reg);
	kgsl_regwrite(device, GEN8_DBGC_CFG_DBGBUS_SEL_C, reg);
	kgsl_regwrite(device, GEN8_DBGC_CFG_DBGBUS_SEL_D, reg);

	/*
	 * There needs to be a delay of 1 us to ensure enough time for correct
	 * data is funneled into the trace buffer
	 */
	udelay(1);

	kgsl_regread(device, GEN8_DBGC_CFG_DBGBUS_TRACE_BUF2, val);
	val++;
	kgsl_regread(device, GEN8_DBGC_CFG_DBGBUS_TRACE_BUF1, val);
}

/* gen8_snapshot_dbgc_debugbus_block() - Capture debug data for a gpu block */
static size_t gen8_snapshot_dbgc_debugbus_block(struct kgsl_device *device,
	u8 *buf, size_t remain, void *priv)
{
	struct kgsl_snapshot_debugbus *header =
		(struct kgsl_snapshot_debugbus *)buf;
	const u32 *block = priv;
	u32 i;
	u32 *data = (u32 *)(buf + sizeof(*header));

	if (remain < GEN8_DEBUGBUS_SECTION_SIZE) {
		SNAPSHOT_ERR_NOMEM(device, "DEBUGBUS");
		return 0;
	}

	header->id = *block;
	header->count = GEN8_DEBUGBUS_BLOCK_SIZE * 2;

	for (i = 0; i < GEN8_DEBUGBUS_BLOCK_SIZE; i++)
		gen8_dbgc_debug_bus_read(device, *block, i, &data[i*2]);

	return GEN8_DEBUGBUS_SECTION_SIZE;
}

static void gen8_dbgc_side_debug_bus_read(struct kgsl_device *device,
	u32 block_id, u32 index, u32 *val)
{
	u32 reg = FIELD_PREP(GENMASK(7, 0), index) |
			FIELD_PREP(GENMASK(24, 16), block_id);

	kgsl_regwrite(device, GEN8_DBGC_CFG_DBGBUS_SEL_A, reg);
	kgsl_regwrite(device, GEN8_DBGC_CFG_DBGBUS_SEL_B, reg);
	kgsl_regwrite(device, GEN8_DBGC_CFG_DBGBUS_SEL_C, reg);
	kgsl_regwrite(device, GEN8_DBGC_CFG_DBGBUS_SEL_D, reg);

	/*
	 * There needs to be a delay of 1 us to ensure enough time for correct
	 * data is funneled into the trace buffer
	 */
	udelay(1);

	reg = kgsl_regmap_read(&device->regmap, GEN8_DBGC_CFG_DBGBUS_OVER);

	*val = FIELD_GET(GENMASK(27, 24), reg);
}

static size_t gen8_snapshot_dbgc_side_debugbus_block(struct kgsl_device *device,
	u8 *buf, size_t remain, void *priv)
{
	struct kgsl_snapshot_side_debugbus *header =
		(struct kgsl_snapshot_side_debugbus *)buf;
	const u32 *block = priv;
	int i;
	u32 *data = (u32 *)(buf + sizeof(*header));
	size_t size = (GEN8_DEBUGBUS_BLOCK_SIZE * sizeof(u32)) + sizeof(*header);

	if (remain < size) {
		SNAPSHOT_ERR_NOMEM(device, "DEBUGBUS");
		return 0;
	}

	header->id = *block;
	header->size = GEN8_DEBUGBUS_BLOCK_SIZE;
	header->valid_data = 0x4;

	for (i = 0; i < GEN8_DEBUGBUS_BLOCK_SIZE; i++)
		gen8_dbgc_side_debug_bus_read(device, *block, i, &data[i]);

	return size;
}

/* gen8_cx_dbgc_debug_bus_read() - Read data from trace bus */
static void gen8_cx_debug_bus_read(struct kgsl_device *device,
	u32 block_id, u32 index, u32 *val)
{
	u32 reg = FIELD_PREP(GENMASK(7, 0), index) |
		FIELD_PREP(GENMASK(24, 16), block_id);

	kgsl_regwrite(device, GEN8_CX_DBGC_CFG_DBGBUS_SEL_A, reg);
	kgsl_regwrite(device, GEN8_CX_DBGC_CFG_DBGBUS_SEL_B, reg);
	kgsl_regwrite(device, GEN8_CX_DBGC_CFG_DBGBUS_SEL_C, reg);
	kgsl_regwrite(device, GEN8_CX_DBGC_CFG_DBGBUS_SEL_D, reg);

	/*
	 * There needs to be a delay of 1 us to ensure enough time for correct
	 * data is funneled into the trace buffer
	 */
	udelay(1);

	kgsl_regread(device, GEN8_CX_DBGC_CFG_DBGBUS_TRACE_BUF2, val);
	val++;
	kgsl_regread(device, GEN8_CX_DBGC_CFG_DBGBUS_TRACE_BUF1, val);
}

/*
 * gen8_snapshot_cx_dbgc_debugbus_block() - Capture debug data for a gpu
 * block from the CX DBGC block
 */
static size_t gen8_snapshot_cx_dbgc_debugbus_block(struct kgsl_device *device,
	u8 *buf, size_t remain, void *priv)
{
	struct kgsl_snapshot_debugbus *header =
		(struct kgsl_snapshot_debugbus *)buf;
	const u32 *block = priv;
	int i;
	u32 *data = (u32 *)(buf + sizeof(*header));

	if (remain < GEN8_DEBUGBUS_SECTION_SIZE) {
		SNAPSHOT_ERR_NOMEM(device, "DEBUGBUS");
		return 0;
	}

	header->id = *block;
	header->count = GEN8_DEBUGBUS_BLOCK_SIZE * 2;

	for (i = 0; i < GEN8_DEBUGBUS_BLOCK_SIZE; i++)
		gen8_cx_debug_bus_read(device, *block, i, &data[i*2]);

	return GEN8_DEBUGBUS_SECTION_SIZE;
}

/* gen8_cx_side_dbgc_debug_bus_read() - Read data from trace bus */
static void gen8_cx_side_debug_bus_read(struct kgsl_device *device,
	u32 block_id, u32 index, u32 *val)
{
	u32 reg = FIELD_PREP(GENMASK(7, 0), index) |
			FIELD_PREP(GENMASK(24, 16), block_id);

	kgsl_regwrite(device, GEN8_CX_DBGC_CFG_DBGBUS_SEL_A, reg);
	kgsl_regwrite(device, GEN8_CX_DBGC_CFG_DBGBUS_SEL_B, reg);
	kgsl_regwrite(device, GEN8_CX_DBGC_CFG_DBGBUS_SEL_C, reg);
	kgsl_regwrite(device, GEN8_CX_DBGC_CFG_DBGBUS_SEL_D, reg);

	/*
	 * There needs to be a delay of 1 us to ensure enough time for correct
	 * data is funneled into the trace buffer
	 */
	udelay(1);

	kgsl_regread(device, GEN8_CX_DBGC_CFG_DBGBUS_OVER, &reg);
	*val = FIELD_GET(GENMASK(27, 24), reg);
}

/*
 * gen8_snapshot_cx_dbgc_debugbus_block() - Capture debug data for a gpu
 * block from the CX DBGC block
 */
static size_t gen8_snapshot_cx_side_dbgc_debugbus_block(struct kgsl_device *device,
	u8 *buf, size_t remain, void *priv)
{
	struct kgsl_snapshot_side_debugbus *header =
		(struct kgsl_snapshot_side_debugbus *)buf;
	const u32 *block = priv;
	int i;
	u32 *data = (u32 *)(buf + sizeof(*header));
	size_t size = (GEN8_DEBUGBUS_BLOCK_SIZE * sizeof(u32)) + sizeof(*header);

	if (remain < size) {
		SNAPSHOT_ERR_NOMEM(device, "DEBUGBUS");
		return 0;
	}

	header->id = *block;
	header->size = GEN8_DEBUGBUS_BLOCK_SIZE;
	header->valid_data = 0x4;

	for (i = 0; i < GEN8_DEBUGBUS_BLOCK_SIZE; i++)
		gen8_cx_side_debug_bus_read(device, *block, i, &data[i]);

	return size;
}

static void gen8_snapshot_cx_debugbus(struct adreno_device *adreno_dev,
		struct kgsl_snapshot *snapshot)
{
	u32 i;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	kgsl_regwrite(device, GEN8_CX_DBGC_CFG_DBGBUS_CNTLT,
			FIELD_PREP(GENMASK(31, 28), 0xf));

	kgsl_regwrite(device, GEN8_CX_DBGC_CFG_DBGBUS_CNTLM,
			FIELD_PREP(GENMASK(27, 24), 0xf));

	kgsl_regwrite(device, GEN8_CX_DBGC_CFG_DBGBUS_IVTL_0, 0);
	kgsl_regwrite(device, GEN8_CX_DBGC_CFG_DBGBUS_IVTL_1, 0);
	kgsl_regwrite(device, GEN8_CX_DBGC_CFG_DBGBUS_IVTL_2, 0);
	kgsl_regwrite(device, GEN8_CX_DBGC_CFG_DBGBUS_IVTL_3, 0);

	kgsl_regwrite(device, GEN8_CX_DBGC_CFG_DBGBUS_BYTEL_0,
			FIELD_PREP(GENMASK(3, 0), 0x0) |
			FIELD_PREP(GENMASK(7, 4), 0x1) |
			FIELD_PREP(GENMASK(11, 8), 0x2) |
			FIELD_PREP(GENMASK(15, 12), 0x3) |
			FIELD_PREP(GENMASK(19, 16), 0x4) |
			FIELD_PREP(GENMASK(23, 20), 0x5) |
			FIELD_PREP(GENMASK(27, 24), 0x6) |
			FIELD_PREP(GENMASK(31, 28), 0x7));
	kgsl_regwrite(device, GEN8_CX_DBGC_CFG_DBGBUS_BYTEL_1,
			FIELD_PREP(GENMASK(3, 0), 0x8) |
			FIELD_PREP(GENMASK(7, 4), 0x9) |
			FIELD_PREP(GENMASK(11, 8), 0xa) |
			FIELD_PREP(GENMASK(15, 12), 0xb) |
			FIELD_PREP(GENMASK(19, 16), 0xc) |
			FIELD_PREP(GENMASK(23, 20), 0xd) |
			FIELD_PREP(GENMASK(27, 24), 0xe) |
			FIELD_PREP(GENMASK(31, 28), 0xf));

	kgsl_regwrite(device, GEN8_CX_DBGC_CFG_DBGBUS_MASKL_0, 0);
	kgsl_regwrite(device, GEN8_CX_DBGC_CFG_DBGBUS_MASKL_1, 0);

	/* Dump the CX debugbus data if the block exists */
	if (!kgsl_regmap_valid_offset(&device->regmap, GEN8_CX_DBGC_CFG_DBGBUS_SEL_A))
		return;

	for (i = 0; i < gen8_snapshot_block_list->cx_debugbus_blocks_len; i++) {
		kgsl_snapshot_add_section(device,
			KGSL_SNAPSHOT_SECTION_DEBUGBUS,
			snapshot, gen8_snapshot_cx_dbgc_debugbus_block,
			(void *) &gen8_snapshot_block_list->cx_debugbus_blocks[i]);
		kgsl_snapshot_add_section(device,
			KGSL_SNAPSHOT_SECTION_SIDE_DEBUGBUS,
			snapshot, gen8_snapshot_cx_side_dbgc_debugbus_block,
			(void *) &gen8_snapshot_block_list->cx_debugbus_blocks[i]);
	}
}

/* gen8_snapshot_debugbus() - Capture debug bus data */
static void gen8_snapshot_debugbus(struct adreno_device *adreno_dev,
		struct kgsl_snapshot *snapshot)
{
	u32 i;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	kgsl_regwrite(device, GEN8_DBGC_CFG_DBGBUS_CNTLT,
			FIELD_PREP(GENMASK(31, 28), 0xf));

	kgsl_regwrite(device, GEN8_DBGC_CFG_DBGBUS_CNTLM,
			FIELD_PREP(GENMASK(27, 24), 0xf));

	kgsl_regwrite(device, GEN8_DBGC_CFG_DBGBUS_IVTL_0, 0);
	kgsl_regwrite(device, GEN8_DBGC_CFG_DBGBUS_IVTL_1, 0);
	kgsl_regwrite(device, GEN8_DBGC_CFG_DBGBUS_IVTL_2, 0);
	kgsl_regwrite(device, GEN8_DBGC_CFG_DBGBUS_IVTL_3, 0);

	kgsl_regwrite(device, GEN8_DBGC_CFG_DBGBUS_BYTEL_0,
			FIELD_PREP(GENMASK(3, 0), 0x0) |
			FIELD_PREP(GENMASK(7, 4), 0x1) |
			FIELD_PREP(GENMASK(11, 8), 0x2) |
			FIELD_PREP(GENMASK(15, 12), 0x3) |
			FIELD_PREP(GENMASK(19, 16), 0x4) |
			FIELD_PREP(GENMASK(23, 20), 0x5) |
			FIELD_PREP(GENMASK(27, 24), 0x6) |
			FIELD_PREP(GENMASK(31, 28), 0x7));
	kgsl_regwrite(device, GEN8_DBGC_CFG_DBGBUS_BYTEL_1,
			FIELD_PREP(GENMASK(3, 0), 0x8) |
			FIELD_PREP(GENMASK(7, 4), 0x9) |
			FIELD_PREP(GENMASK(11, 8), 0xa) |
			FIELD_PREP(GENMASK(15, 12), 0xb) |
			FIELD_PREP(GENMASK(19, 16), 0xc) |
			FIELD_PREP(GENMASK(23, 20), 0xd) |
			FIELD_PREP(GENMASK(27, 24), 0xe) |
			FIELD_PREP(GENMASK(31, 28), 0xf));

	kgsl_regwrite(device, GEN8_DBGC_CFG_DBGBUS_MASKL_0, 0);
	kgsl_regwrite(device, GEN8_DBGC_CFG_DBGBUS_MASKL_1, 0);
	kgsl_regwrite(device, GEN8_DBGC_CFG_DBGBUS_MASKL_2, 0);
	kgsl_regwrite(device, GEN8_DBGC_CFG_DBGBUS_MASKL_3, 0);

	for (i = 0; i < gen8_snapshot_block_list->debugbus_blocks_len; i++) {
		kgsl_snapshot_add_section(device,
			KGSL_SNAPSHOT_SECTION_DEBUGBUS,
			snapshot, gen8_snapshot_dbgc_debugbus_block,
			(void *) &gen8_snapshot_block_list->debugbus_blocks[i]);
		kgsl_snapshot_add_section(device,
			KGSL_SNAPSHOT_SECTION_SIDE_DEBUGBUS,
			snapshot, gen8_snapshot_dbgc_side_debugbus_block,
			(void *) &gen8_snapshot_block_list->debugbus_blocks[i]);
	}

	for (i = 0; i < gen8_snapshot_block_list->gbif_debugbus_blocks_len; i++) {
		kgsl_snapshot_add_section(device,
			KGSL_SNAPSHOT_SECTION_DEBUGBUS,
			snapshot, gen8_snapshot_dbgc_debugbus_block,
			(void *) &gen8_snapshot_block_list->gbif_debugbus_blocks[i]);
		kgsl_snapshot_add_section(device,
			KGSL_SNAPSHOT_SECTION_SIDE_DEBUGBUS,
			snapshot, gen8_snapshot_dbgc_side_debugbus_block,
			(void *) &gen8_snapshot_block_list->gbif_debugbus_blocks[i]);
	}
}

static bool gen8_reglist_snapshot(struct kgsl_device *device,
					struct kgsl_snapshot *snapshot)
{
	u64 *ptr, offset = 0;
	u32 i, j, r, slices;
	struct gen8_reg_list *reg_list = gen8_snapshot_block_list->reg_list;
	struct gen8_reg_list_info info = {0};

	if (CD_SCRIPT_CHECK(device)) {
		for (i = 0; reg_list[i].regs; i++) {
			struct gen8_reg_list *regs = &reg_list[i];

			slices = NUMBER_OF_SLICES(regs->slice_region, ADRENO_DEVICE(device));
			for (j = 0; j < slices; j++) {
				info.regs = regs;
				info.slice_id = SLICE_ID(regs->slice_region, j);
				kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_MVC_V3,
					snapshot, gen8_legacy_snapshot_registers, &info);
			}
		}
		return true;
	}

	for (i = 0; reg_list[i].regs; i++) {
		struct gen8_reg_list *regs = &reg_list[i];

		slices = NUMBER_OF_SLICES(regs->slice_region, ADRENO_DEVICE(device));
		regs->offset = offset;

		for (j = 0; j < slices; j++) {
			const u32 *regs_ptr = regs->regs;

			/* Build the crash script */
			ptr = gen8_capturescript->hostptr;

			ptr += CD_WRITE(ptr, GEN8_CP_APERTURE_CNTL_CD, GEN8_CP_APERTURE_REG_VAL
					(j, 0, 0, 0));
			/* Program the SEL_CNTL_CD register appropriately */
			if (regs->sel)
				ptr += CD_WRITE(ptr, regs->sel->cd_reg, regs->sel->val);
			info.regs = regs;
			info.slice_id = SLICE_ID(regs->slice_region, j);
			info.offset = offset;

			for (; regs_ptr[0] != UINT_MAX; regs_ptr += 2) {
				r = REG_COUNT(regs_ptr);
				ptr += CD_READ(ptr, regs_ptr[0], r,
					(gen8_crashdump_registers->gpuaddr + offset));
				offset += r * sizeof(u32);
			}

			/* Marker for end of script */
			CD_FINISH(ptr, offset);

			/* Try to run the crash dumper and bail if it times out */
			if (!_gen8_do_crashdump(device))
				return false;

			kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_MVC_V3,
				snapshot, gen8_snapshot_registers, &info);
		}
	}

	return true;
}

static size_t gen8_snapshot_cx_misc_registers(struct kgsl_device *device, u8 *buf,
		size_t remain, void *priv)
{
	const u32 *ptr = (u32 *)priv;
	u32 *src, *data = (unsigned int *)buf;
	size_t size = adreno_snapshot_regs_count(ptr) * sizeof(u32);

	if (remain < size) {
		SNAPSHOT_ERR_NOMEM(device, "CX_MISC REGISTERS");
		return 0;
	}

	src = gen8_crashdump_registers->hostptr;

	for (; ptr[0] != UINT_MAX; ptr += 2) {
		u32 cnt = REG_COUNT(ptr);

		if (cnt == 1)
			*data++ = BIT(31) | ptr[0];
		else {
			*data++ = ptr[0];
			*data++ = cnt;
		}
		memcpy(data, src, cnt << 2);
		data += cnt;
		src += cnt;
	}

	/* Return the size of the section */
	return size;
}

static bool gen8_cx_misc_regs_snapshot(struct kgsl_device *device,
					struct kgsl_snapshot *snapshot)
{
	u64 *ptr, offset = 0;
	const u32 *regs_ptr = (const u32 *)gen8_snapshot_block_list->cx_misc_regs;

	if (CD_SCRIPT_CHECK(device) || !gen8_gmu_rpmh_pwr_state_is_active(device)
		|| !gen8_gmu_gx_is_on(ADRENO_DEVICE(device)))
		goto legacy_snapshot;

	/* Build the crash script */
	ptr = (u64 *)gen8_capturescript->hostptr;

	for (; regs_ptr[0] != UINT_MAX; regs_ptr += 2) {
		u32 r = REG_COUNT(regs_ptr);

		ptr += CD_READ(ptr, regs_ptr[0], r,
			(gen8_crashdump_registers->gpuaddr + offset));
		offset += r * sizeof(u32);
	}

	/* Marker for end of script */
	CD_FINISH(ptr, offset);

	/* Try to run the crash dumper if it fails return */
	if (_gen8_do_crashdump(device)) {
		kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_REGS_V2,
			snapshot, gen8_snapshot_cx_misc_registers,
			(void *)gen8_snapshot_block_list->cx_misc_regs);
		return true;
	} else
		return false;

legacy_snapshot:
	regs_ptr = (const u32 *)gen8_snapshot_block_list->cx_misc_regs;

	if (!kgsl_regmap_valid_offset(&device->regmap, regs_ptr[0])) {
		WARN_ONCE(1, "cx_misc registers are not defined in device tree");
		return true;
	}

	kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_REGS_V2,
		snapshot, adreno_snapshot_registers_v2, (void *)regs_ptr);

	return true;
}

void gen8_snapshot_external_core_regs(struct kgsl_device *device,
			struct kgsl_snapshot *snapshot)
{
	const u32 **external_core_regs;
	u32 i, num_external_core_regs;
	const struct adreno_gen8_core *gpucore = to_gen8_core(ADRENO_DEVICE(device));

	gen8_snapshot_block_list = gpucore->gen8_snapshot_block_list;
	external_core_regs = gen8_snapshot_block_list->external_core_regs;
	num_external_core_regs = gen8_snapshot_block_list->num_external_core_regs;

	for (i = 0; i < num_external_core_regs; i++)
		kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_REGS_V2,
			snapshot, adreno_snapshot_registers_v2,
			(void *) external_core_regs[i]);
}

/*
 * gen8_snapshot() - GEN8 GPU snapshot function
 * @adreno_dev: Device being snapshotted
 * @snapshot: Pointer to the snapshot instance
 *
 * This is where all of the GEN8 specific bits and pieces are grabbed
 * into the snapshot memory
 */
void gen8_snapshot(struct adreno_device *adreno_dev,
		struct kgsl_snapshot *snapshot)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	u32 i, slice_mask;
	const struct adreno_gen8_core *gpucore = to_gen8_core(ADRENO_DEVICE(device));
	int is_current_rt;
	gen8_crashdump_timedout = false;
	gen8_snapshot_block_list = gpucore->gen8_snapshot_block_list;

	/* External core and CX MISC regs are dumped in the beginning of gmu snapshot */
	if (!gmu_core_isenabled(device)) {
		gen8_snapshot_external_core_regs(device, snapshot);

		/*
		 * If crashdumper timed out while dumping this section skip everything
		 * since even AHB accesses to the GPU might cause NoC errors.
		 */
		if (!gen8_cx_misc_regs_snapshot(device, snapshot))
			return;
	}

	gen8_snapshot_cx_debugbus(adreno_dev, snapshot);

	if (!gen8_gmu_rpmh_pwr_state_is_active(device) ||
		!gen8_gmu_gx_is_on(adreno_dev))
		return;

	gen8_snapshot_trace_buffer(device, snapshot);

	gen8_snapshot_debugbus(adreno_dev, snapshot);

	is_current_rt = rt_task(current);

	if (is_current_rt)
		sched_set_normal(current, 0);

	gen8_regread64_aperture(device, GEN8_CP_IB1_BASE_LO_PIPE,
		GEN8_CP_IB1_BASE_HI_PIPE, &snapshot->ib1base, PIPE_BR, 0, 0);

	gen8_regread64_aperture(device, GEN8_CP_IB2_BASE_LO_PIPE,
		GEN8_CP_IB2_BASE_HI_PIPE, &snapshot->ib2base, PIPE_BR, 0, 0);

	gen8_regread64_aperture(device, GEN8_CP_IB3_BASE_LO_PIPE,
		GEN8_CP_IB3_BASE_HI_PIPE, &snapshot->ib3base, PIPE_BR, 0, 0);

	gen8_regread_aperture(device, GEN8_CP_IB1_REM_SIZE_PIPE,
			&snapshot->ib1size, PIPE_BR, 0, 0);
	gen8_regread_aperture(device, GEN8_CP_IB2_REM_SIZE_PIPE,
			&snapshot->ib2size, PIPE_BR, 0, 0);
	gen8_regread_aperture(device, GEN8_CP_IB3_REM_SIZE_PIPE,
		&snapshot->ib3size, PIPE_BR, 0, 0);

	if (ADRENO_FEATURE(adreno_dev, ADRENO_LPAC)) {
		gen8_regread64_aperture(device, GEN8_CP_IB1_BASE_LO_PIPE,
			GEN8_CP_IB1_BASE_HI_PIPE, &snapshot->ib1base_lpac, PIPE_LPAC, 0, 0);

		gen8_regread64_aperture(device, GEN8_CP_IB2_BASE_LO_PIPE,
			GEN8_CP_IB2_BASE_HI_PIPE, &snapshot->ib2base_lpac, PIPE_LPAC, 0, 0);

		gen8_regread_aperture(device, GEN8_CP_IB1_REM_SIZE_PIPE,
			&snapshot->ib1size_lpac, PIPE_LPAC, 0, 0);
		gen8_regread_aperture(device, GEN8_CP_IB2_REM_SIZE_PIPE,
			&snapshot->ib2size_lpac, PIPE_LPAC, 0, 0);
	}

	/* Clear aperture register */
	gen8_host_aperture_set(adreno_dev, 0, 0, 0);

	/*
	 * Assert the isStatic bit before triggering snapshot.
	 * BIT(0): GPU activity during snapshot dump
	 * BIT(1): GPU UNSLICE activity during snapshot dump
	 * Similarly, BIT(4) for slice-0, BIT(5) for slice-1 and so on.
	 */
	slice_mask = ((1 << gen8_get_num_slices(adreno_dev)) - 1) << 4;
	kgsl_regwrite(device, GEN8_RBBM_SNAPSHOT_STATUS, BIT(0) | BIT(1) | slice_mask);

	/* Dump the registers which get affected by crash dumper trigger */
	for (i = 0; i < gen8_snapshot_block_list->num_pre_crashdumper_regs; i++) {
		struct gen8_reg_list *regs = &gen8_snapshot_block_list->pre_crashdumper_regs[i];
		struct gen8_reg_list_info info = {0};
		u32 j, slices;

		slices = NUMBER_OF_SLICES(regs->slice_region, adreno_dev);

		for (j = 0; j < slices; j++) {
			info.regs = regs;
			info.slice_id = SLICE_ID(regs->slice_region, j);
			kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_MVC_V3,
				snapshot, gen8_legacy_snapshot_registers, &info);
		}
	}

	/*
	 * If crashdumper timed out while dumping this section skip everything
	 * since even AHB acceses to the GPU might cause NoC errors.
	 */
	if (!gen8_reglist_snapshot(device, snapshot))
		goto err;

	for (i = 0; i < gen8_snapshot_block_list->index_registers_len; i++) {
		kgsl_regwrite(device, GEN8_CP_APERTURE_CNTL_HOST, GEN8_CP_APERTURE_REG_VAL
				(0,  gen8_snapshot_block_list->index_registers[i].pipe_id, 0, 0));

		kgsl_snapshot_indexed_registers_v2(device, snapshot,
			gen8_snapshot_block_list->index_registers[i].addr,
			gen8_snapshot_block_list->index_registers[i].data, 0,
			gen8_snapshot_block_list->index_registers[i].size,
			gen8_snapshot_block_list->index_registers[i].pipe_id, UINT_MAX);
	}

	/* Mempool debug data */
	gen8_snapshot_mempool(device, snapshot);

	/*
	 * CP MVC register section
	 * If crashdumper timed out while dumping any section below skip everything
	 * since even AHB acceses to the GPU might cause NoC errors.
	 */
	if (!gen8_snapshot_mvc_regs(device, snapshot,
		gen8_snapshot_block_list->cp_clusters, gen8_snapshot_block_list->num_cp_clusters))
		goto err;

	/* MVC register section */
	if (!gen8_snapshot_mvc_regs(device, snapshot,
		gen8_snapshot_block_list->clusters, gen8_snapshot_block_list->num_clusters))
		goto err;

	/* registers dumped through DBG AHB */
	if (!gen8_snapshot_dbgahb_regs(device, snapshot))
		goto err;

	/* Shader memory */
	if (!gen8_snapshot_shader(device, snapshot))
		goto err;

	kgsl_regwrite(device, GEN8_RBBM_SNAPSHOT_STATUS, 0x0);

err:
	/* Preemption record */
	adreno_snapshot_preemption_record(device, snapshot);

	if (is_current_rt)
		sched_set_fifo(current);
}

void gen8_crashdump_init(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	int ret;

	ret = adreno_allocate_global(device, &gen8_capturescript,
		50 * PAGE_SIZE, 0, KGSL_MEMFLAGS_GPUREADONLY,
		KGSL_MEMDESC_PRIVILEGED, "capturescript");

	if (!ret)
		ret = adreno_allocate_global(device, &gen8_crashdump_registers,
			200 * PAGE_SIZE, 0, 0,
			KGSL_MEMDESC_PRIVILEGED, "capturescript_regs");

	if (ret)
		dev_err(device->dev, "Failed to init crashdumper err = %d\n", ret);
}
