/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Synopsys DesignWare XPCS platform device driver
 *
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __LINUX_PCS_XPCS_QCOM_H
#define __LINUX_PCS_XPCS_QCOM_H

#include <linux/phy.h>
#include <linux/phylink.h>

/* AN mode */
#define DW_AN_C37_USXGMII		1
#define DW_10GBASER			5

struct xpcs_id;

struct dw_xpcs_qcom {
	const struct xpcs_id *id;
	struct phylink_pcs pcs;
	void __iomem *addr;
	int pcs_intr;
	int pcs_fusa_intr;
	int pcs_fusa_error_count;
	bool intr_en;
	bool needs_aneg;
	int phy_interface;
};

struct phylink_pcs *qcom_xpcs_create(struct device_node *np,
				      phy_interface_t interface);
void qcom_xpcs_link_up(struct phylink_pcs *pcs, unsigned int mode,
		  phy_interface_t interface, int speed, int duplex);
void qcom_xpcs_destroy(struct phylink_pcs *pcs);

void qcom_xpcs_get_err_stats(struct phylink_pcs *pcs, unsigned long *ptr);

#endif /* __LINUX_PCS_XPCS_QCOM_H */
