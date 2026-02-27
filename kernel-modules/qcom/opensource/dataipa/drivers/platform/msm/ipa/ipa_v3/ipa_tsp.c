// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/bits.h>
#include "ipa_i.h"
#include "ipahal.h"
#include "ipahal_reg.h"
#include "ipahal_tsp.h"

/*
 * Every Producer can hold up to 16 K Queue elements and 8 Queues.
 * Thus, software allocates up to 512 Kbytes (32 bytes *16 K QEs) contiguously in memory.
 */
#define IPA_TSP_QM_DRAM_BYTESIZE (16384 * 32)

static void __ipa_tsp_get_supported_constrains(void)
{
	if (ipa3_ctx->ipa_hw_type >= IPA_HW_v5_5) {
		struct ipahal_ipa_flavor_9 ipa_flavor_9;

		ipahal_read_reg_fields(IPA_FLAVOR_9, &ipa_flavor_9);
		ipa3_ctx->tsp.ingr_tc_max = ipa_flavor_9.ipa_tsp_max_ingr_tc;
		ipa3_ctx->tsp.egr_ep_max = ipa_flavor_9.ipa_tsp_max_prod;
		ipa3_ctx->tsp.egr_tc_max = ipa_flavor_9.ipa_tsp_max_egr_tc;
	} else {
		ipa3_ctx->tsp.ingr_tc_max = 0;
		ipa3_ctx->tsp.egr_ep_max = 0;
		ipa3_ctx->tsp.egr_tc_max = 0;

		IPAERR("TSP not supported. ingr_tc_max=%d egr_ep_max=%d egr_tc_max=%d\n",
		       ipa3_ctx->tsp.ingr_tc_max, ipa3_ctx->tsp.egr_ep_max, ipa3_ctx->tsp.egr_tc_max);
	}
}

int ipa_tsp_init(void)
{
	int i, ret = 0;
	gfp_t flag = GFP_KERNEL;
	dma_addr_t qm_tlv_base;
	u32 qm_tlv_size;

	/*
	 * Cache the TSP table size constrains
	 */
	__ipa_tsp_get_supported_constrains();
	if (!ipa3_ctx->tsp.ingr_tc_max || !ipa3_ctx->tsp.egr_ep_max || !ipa3_ctx->tsp.egr_tc_max) {
		IPAERR("TSP not supported. ingr_tc_max=%d egr_ep_max=%d egr_tc_max=%d\n",
		       ipa3_ctx->tsp.ingr_tc_max, ipa3_ctx->tsp.egr_ep_max, ipa3_ctx->tsp.egr_tc_max);
		return -EFAULT;
	}

	/*
	 * Allocate DMA accessible memory for HW commitable ipa3_ctx->tsp.ingr_tc_tbl,
	 * ipa3_ctx->tsp.egr_ep_tbl and ipa3_ctx->tsp.egr_tc_tbl
	 */
	ipa3_ctx->tsp.ingr_tc_tbl.size = ipa3_ctx->tsp.ingr_tc_max * IPA_TSP_INGR_TC_SIZE;
	ipa3_ctx->tsp.egr_ep_tbl.size = ipa3_ctx->tsp.egr_ep_max * IPA_TSP_EGR_EP_SIZE;
	ipa3_ctx->tsp.egr_tc_tbl.size = ipa3_ctx->tsp.egr_tc_max * IPA_TSP_EGR_TC_SIZE;

	if (!ipa3_ctx->tsp.ingr_tc_tbl.size || !ipa3_ctx->tsp.egr_ep_tbl.size || !ipa3_ctx->tsp.egr_tc_tbl.size) {
		IPAERR("ingr_tc_tbl.size=%d egr_ep_tbl.size=%d egr_tc_tbl.size=%d\n",
		       ipa3_ctx->tsp.ingr_tc_tbl.size, ipa3_ctx->tsp.egr_ep_tbl.size, ipa3_ctx->tsp.egr_tc_tbl.size);
		return -EFAULT;
	}

	if (!ipa3_ctx->pdev || !&(ipa3_ctx->tsp.ingr_tc_tbl.phys_base)) {
		IPAERR("ipa3_ctx->pdev=%16X &(ipa3_ctx->tsp.ingr_tc_tbl.phys_base)=%16X\n",
		       ipa3_ctx->pdev, &(ipa3_ctx->tsp.ingr_tc_tbl.phys_base));
		return -EFAULT;
	}
	IPAERR("ipa3_ctx->pdev=%16X &(ipa3_ctx->tsp.ingr_tc_tbl.phys_base)=%16X\n",
	       ipa3_ctx->pdev, &(ipa3_ctx->tsp.ingr_tc_tbl.phys_base));

	ipa3_ctx->tsp.ingr_tc_tbl.base = dma_alloc_coherent(ipa3_ctx->pdev,
		ipa3_ctx->tsp.ingr_tc_tbl.size, &(ipa3_ctx->tsp.ingr_tc_tbl.phys_base), GFP_KERNEL);
	if (!ipa3_ctx->tsp.ingr_tc_tbl.base) {
		IPAERR("Failed to allocate cache memory for ingress TC TSP table.\n");
		return -ENOMEM;
	}

	ipa3_ctx->tsp.egr_ep_tbl.base = dma_alloc_coherent(ipa3_ctx->pdev,
		ipa3_ctx->tsp.egr_ep_tbl.size, &(ipa3_ctx->tsp.egr_ep_tbl.phys_base), GFP_KERNEL);
	if (!ipa3_ctx->tsp.egr_ep_tbl.base) {
		IPAERR("Failed to allocate cache memory for egress producer TSP table.\n");
		ret = -ENOMEM;
		goto free_ingr;
	}

	ipa3_ctx->tsp.egr_tc_tbl.base = dma_alloc_coherent(ipa3_ctx->pdev,
		ipa3_ctx->tsp.egr_tc_tbl.size, &(ipa3_ctx->tsp.egr_tc_tbl.phys_base), GFP_KERNEL);
	if (!ipa3_ctx->tsp.egr_tc_tbl.base) {
		IPAERR("Failed to allocate cache memory for egress TC TSP table.\n");
		ret = -ENOMEM;
		goto free_ep;
	}

	/*
	 * Allocate configured producer array
	 */
	ipa3_ctx->tsp.egr_ep_config =
		(enum ipa_client_type *)
		kzalloc(ipa3_ctx->tsp.egr_ep_max * sizeof(enum ipa_client_type), GFP_KERNEL);

	if (ipa3_ctx->tsp.egr_ep_config == NULL) {
		IPAERR("Failed to allocate cache memory for egress producer config.\n");
		ret = -ENOMEM;
		goto free_egr;
	}

	/*
	 * Init configured producer array
	 */
	for (i = 0; i < ipa3_ctx->tsp.egr_ep_max; i++)
		ipa3_ctx->tsp.egr_ep_config[i] = IPA_CLIENT_MAX;
	ipa3_ctx->tsp.egr_tc_range_mask = 0;

	/*
	 * Allocate memory for TLV-IN queues:
	 * Every Producer can hold up to 16 K Queue elements and 8 Queues.
	 * Thus, software allocates up to 512 Kbytes (32 bytes *16 K QEs) contiguously in memory.
	 * Note: All queues of all PRODs have to be together in contiguous memory!
	 * Must be 128B aligned.
	 */
	qm_tlv_size = IPA_TSP_QM_DRAM_BYTESIZE;
	ipa3_ctx->tsp.qm_tlv_mem.size = qm_tlv_size + 128; // + 128 bytes to fit alignment
alloc:
	ipa3_ctx->tsp.qm_tlv_mem.base = dma_alloc_coherent(ipa3_ctx->pdev,
		ipa3_ctx->tsp.qm_tlv_mem.size, &(ipa3_ctx->tsp.qm_tlv_mem.phys_base), flag);
	if (!ipa3_ctx->tsp.qm_tlv_mem.base) {
		if (flag == GFP_KERNEL) {
			flag = GFP_ATOMIC;
			goto alloc;
		}
		IPAERR("fail to alloc DMA buff of size %d\n", ipa3_ctx->tsp.qm_tlv_mem.size);
		ret = -ENOMEM;
		goto free_ep_conf;
	}

	/* Get 128B aligned address inside the buffer */
	qm_tlv_base = ipa3_ctx->tsp.qm_tlv_mem.phys_base;
	if (qm_tlv_base & 0x7F) {
		qm_tlv_base += 0x80;
		qm_tlv_base &= !0x7F;
	}

	ipahal_write_reg(IPA_TSP_QM_EXTERNAL_BADDR_LSB, qm_tlv_base & 0xFFFFFFFF);
	ipahal_write_reg(IPA_TSP_QM_EXTERNAL_BADDR_MSB, qm_tlv_base >> 32);
	ipahal_write_reg(IPA_TSP_QM_EXTERNAL_SIZE, qm_tlv_size >> 12); //Size in 4kB resolution

	goto done;

free_ep_conf:
	kfree(ipa3_ctx->tsp.egr_ep_config);
	ipa3_ctx->tsp.egr_ep_config = NULL;
free_egr:
	dma_free_coherent(ipa3_ctx->pdev, ipa3_ctx->tsp.egr_tc_tbl.size,
		ipa3_ctx->tsp.egr_tc_tbl.base, ipa3_ctx->tsp.egr_tc_tbl.phys_base);
	ipa3_ctx->tsp.egr_tc_tbl.base = NULL;
free_ep:
	dma_free_coherent(ipa3_ctx->pdev, ipa3_ctx->tsp.egr_ep_tbl.size,
		ipa3_ctx->tsp.egr_ep_tbl.base, ipa3_ctx->tsp.egr_ep_tbl.phys_base);
	ipa3_ctx->tsp.egr_ep_tbl.base = NULL;
free_ingr:
	dma_free_coherent(ipa3_ctx->pdev, ipa3_ctx->tsp.ingr_tc_tbl.size,
		ipa3_ctx->tsp.ingr_tc_tbl.base, ipa3_ctx->tsp.ingr_tc_tbl.phys_base);
	ipa3_ctx->tsp.ingr_tc_tbl.base = NULL;
done:
	return ret;
}

int ipa_tsp_get_ingr_tc(u8 index, struct ipa_ioc_tsp_ingress_class_params *output)
{
	/* The function is internal only, assuming valid params */

	ipahal_tsp_parse_hw_ingr_tc(ipa3_ctx->tsp.ingr_tc_tbl.base, index, output);
	output->include_l2_len = !!(ipahal_read_reg(IPA_TSP_INGRESS_POLICING_CFG) & (0x1 << index));

	return 0;
}

int ipa_tsp_get_egr_ep(u8 index, struct ipa_ioc_tsp_egress_prod_params *output)
{
	u32 regval;
	struct ipa_ep_cfg_prod_cfg prod_cfg;

	/* The function is internal only, assuming valid params */

	ipahal_tsp_parse_hw_egr_ep(ipa3_ctx->tsp.egr_ep_tbl.base, index, output);

	output->client = ipa3_ctx->tsp.egr_ep_config[index];

	regval = ipahal_read_reg_n_fields(IPA_ENDP_INIT_PROD_CFG_n,
		ipa_get_ep_mapping(output->client), (void *)&prod_cfg);

	output->max_out_bytes = prod_cfg.max_output_size << 6; // max_output_size*64
	output->policing_by_max_out = prod_cfg.max_output_size_drop_enable;
	output->tc_lo = prod_cfg.egress_tc_lowest;
	output->tc_hi = prod_cfg.egress_tc_highest;

	return 0;
}

int ipa_tsp_get_egr_tc(u8 index, struct ipa_ioc_tsp_egress_class_params *output)
{
	/* The function is internal only, assuming valid params */

	ipahal_tsp_parse_hw_egr_tc(ipa3_ctx->tsp.egr_tc_tbl.base, index, output);

	return 0;
}

int ipa_tsp_set_ingr_tc(u8 index, const struct ipa_ioc_tsp_ingress_class_params *input)
{
	/* The function is internal only, assuming valid params */

	ipahal_tsp_fill_hw_ingr_tc(input, ipa3_ctx->tsp.ingr_tc_tbl.base, index);
	ipahal_write_reg_mask(IPA_TSP_INGRESS_POLICING_CFG,
		0x1 << input->include_l2_len, 0x1 << index);

	return 0;
}

int ipa_tsp_set_egr_ep(u8 index, const struct ipa_ioc_tsp_egress_prod_params *input)
{
	u32 regval, ep_index, ep_tc_mask, new_tc_range_mask;
	struct ipa_ep_cfg_prod_cfg prod_cfg;
	bool cleanup = false;

	ep_tc_mask = GENMASK(input->tc_hi, input->tc_lo);
	new_tc_range_mask = ipa3_ctx->tsp.egr_tc_range_mask;

	ep_index = ipa_get_ep_mapping(ipa3_ctx->tsp.egr_ep_config[index]);
	regval = ipahal_read_reg_n_fields(
		IPA_ENDP_INIT_PROD_CFG_n, ep_index, (void *)&prod_cfg);

	if (ipa3_ctx->tsp.egr_ep_config[index] != IPA_CLIENT_MAX &&
	    ipa3_ctx->tsp.egr_ep_config[index] != input->client) {
		cleanup = true;
		new_tc_range_mask &= !GENMASK(prod_cfg.egress_tc_highest,prod_cfg.egress_tc_lowest);
	}

	if (ep_tc_mask & new_tc_range_mask) {
		IPAERR("New egress TC range overlaps existing.\n");
		return -EINVAL;
	}

	if (cleanup) {
		/* Cleanup old producer config */
		prod_cfg.tsp_enable = false;
		prod_cfg.egress_tc_lowest = 0;
		prod_cfg.egress_tc_highest = 0;
		if (ipa3_cfg_ep_prod_cfg(ep_index, &prod_cfg) != 0) {
			IPAERR("Failed configuring the producer EP.\n");
			return -EFAULT;
		}
	}

	prod_cfg.tsp_enable = true;
	prod_cfg.tsp_idx = index;
	prod_cfg.max_output_size = input->max_out_bytes >> 6; // max_out_bytes/64
	prod_cfg.max_output_size_drop_enable = input->policing_by_max_out;
	prod_cfg.egress_tc_lowest = input->tc_lo;
	prod_cfg.egress_tc_highest = input->tc_hi;
	if (ipa3_cfg_ep_prod_cfg(ipa_get_ep_mapping(input->client), &prod_cfg) != 0) {
		IPAERR("Failed configuring the producer EP.\n");
		return -EFAULT;
	}

	ipa3_ctx->tsp.egr_ep_config[index] = input->client;
	ipa3_ctx->tsp.egr_tc_range_mask = new_tc_range_mask | ep_tc_mask;
	ipahal_tsp_fill_hw_egr_ep(input, ipa3_ctx->tsp.egr_ep_tbl.base, index);

	return 0;
}

int ipa_tsp_set_egr_tc(u8 index, const struct ipa_ioc_tsp_egress_class_params *input)
{
	/* The function is internal only, assuming valid params */

	ipahal_tsp_fill_hw_egr_tc(input, ipa3_ctx->tsp.egr_tc_tbl.base, index);

	/*
	 * If guaranteed_rate and guaranteed_burst are set to 0,
	 * the guaranteed bandwidth rate will be disabled,
	 * and only maximal bandwidth rate will be considered.
	 */
	if (input->guaranteed_rate || input->guaranteed_burst)
		ipahal_write_reg_mask(IPA_TSP_EGRESS_POLICING_CFG, 0x1 << index, 0x1 << index);
	else
		ipahal_write_reg_mask(IPA_TSP_EGRESS_POLICING_CFG, 0x0, 0x1 << index);

	return 0;
}

int ipa_tsp_commit(void)
{
	int ret = 0;
	u32 ingr_tc_base, egr_tc_base, prod_base;
	void *ingr_tc_mmio, *egr_tc_mmio, *prod_mmio;

	ingr_tc_base = ipahal_read_reg(IPA_RAM_INGRESS_POLICER_DB_BASE_ADDR);
	egr_tc_base = ipahal_read_reg(IPA_RAM_EGRESS_SHAPING_TC_DB_BASE_ADDR);
	prod_base = ipahal_read_reg(IPA_RAM_EGRESS_SHAPING_PROD_DB_BASE_ADDR);

	/* map IPA SRAM */
	ingr_tc_mmio = ioremap(ipa3_ctx->ipa_wrapper_base + ingr_tc_base,
		ipa3_ctx->tsp.ingr_tc_tbl.size);
	egr_tc_mmio = ioremap(ipa3_ctx->ipa_wrapper_base + egr_tc_base,
		ipa3_ctx->tsp.egr_tc_tbl.size);
	prod_mmio = ioremap(ipa3_ctx->ipa_wrapper_base + prod_base,
		ipa3_ctx->tsp.egr_ep_tbl.size);
	if (!ingr_tc_mmio || !egr_tc_mmio || !prod_mmio) {
		IPAERR("Failed to ioremap TSP SRAM\n");
		ret = -ENOMEM;
		goto end;
	}

	/*
	 * The tables are located in the HW SRAM area, and we can't issue the DMA IMM, because
	 * the offset field is restricted to 16 addresses in it.
	 * Therefore, we do memcopy instead, but the infrastructure will be ready for DMA
	 * in future IPA versions.
	 */
	memcpy_toio(ingr_tc_mmio, ipa3_ctx->tsp.ingr_tc_tbl.base, ipa3_ctx->tsp.ingr_tc_tbl.size);
	memcpy_toio(egr_tc_mmio, ipa3_ctx->tsp.egr_tc_tbl.base, ipa3_ctx->tsp.egr_tc_tbl.size);
	memcpy_toio(prod_mmio, ipa3_ctx->tsp.egr_ep_tbl.base, ipa3_ctx->tsp.egr_ep_tbl.size);

end:
	if (ingr_tc_mmio)
		iounmap(ingr_tc_mmio);
	if (egr_tc_mmio)
		iounmap(egr_tc_mmio);
	if (prod_mmio)
		iounmap(prod_mmio);
	return ret;
}

int ipa_tsp_reset(void)
{
	int i;
	struct ipa_ep_cfg_prod_cfg prod_cfg = {0};

	for (i = 0;
	      i < ipa3_ctx->tsp.egr_ep_max && ipa3_ctx->tsp.egr_ep_config[i] < IPA_CLIENT_MAX; i++)
		ipa3_cfg_ep_prod_cfg(ipa_get_ep_mapping(ipa3_ctx->tsp.egr_ep_config[i]),
		      &prod_cfg);

	if (ipa3_ctx->tsp.ingr_tc_tbl.base)
		memset(ipa3_ctx->tsp.ingr_tc_tbl.base, 0, ipa3_ctx->tsp.ingr_tc_tbl.size);
	if (ipa3_ctx->tsp.egr_tc_tbl.base)
		memset(ipa3_ctx->tsp.egr_tc_tbl.base, 0, ipa3_ctx->tsp.egr_tc_tbl.size);
	if (ipa3_ctx->tsp.egr_ep_tbl.base)
		memset(ipa3_ctx->tsp.egr_ep_tbl.base, 0, ipa3_ctx->tsp.egr_ep_tbl.size);

	/* Reinit configured producer array */
	for (i = 0; i < ipa3_ctx->tsp.egr_ep_max; i++)
		ipa3_ctx->tsp.egr_ep_config[i] = IPA_CLIENT_MAX;

	ipa3_ctx->tsp.egr_tc_range_mask = 0;

	return ipa_tsp_commit();
}

