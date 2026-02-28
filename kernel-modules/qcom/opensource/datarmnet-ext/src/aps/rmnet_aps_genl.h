/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _RMNET_APS_GENL_H_
#define _RMNET_APS_GENL_H_

#include <net/genetlink.h>

/* Generic Netlink Definitions */
#define RMNET_APS_GENL_VERSION 1
#define RMNET_APS_GENL_FAMILY_NAME "RMNET_APS"

enum { RMNET_APS_GENL_CMD_UNSPEC,
       RMNET_APS_GENL_CMD_FLOW,
       RMNET_APS_GENL_CMD_PDN_CONFIG,
       RMNET_APS_GENL_CMD_FILTER,
       RMNET_APS_GENL_CMD_DATA_REPORT,
       __RMNET_APS_GENL_CMD_MAX,
};

enum { RMNET_APS_GENL_ATTR_UNSPEC,
       RMNET_APS_GENL_ATTR_FLOW_REQ,
       RMNET_APS_GENL_ATTR_FLOW_RESP,
       RMNET_APS_GENL_ATTR_PDN_CONFIG_REQ,
       RMNET_APS_GENL_ATTR_PDN_CONFIG_RESP,
       RMNET_APS_GENL_ATTR_FILTER_REQ,
       RMNET_APS_GENL_ATTR_FILTER_RESP,
       RMNET_APS_GENL_ATTR_DATA_REPORT,
       __RMNET_APS_GENL_ATTR_MAX,
};
#define RMNET_APS_GENL_ATTR_MAX (__RMNET_APS_GENL_ATTR_MAX - 1)

int rmnet_aps_genl_init(void);

void rmnet_aps_genl_deinit(void);

#endif /*_RMNET_APS_GENL_H_*/
