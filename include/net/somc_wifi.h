/*
 * Copyright (C) 2015 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */
#ifndef __SOMC_WIFI_H__
#define __SOMC_WIFI_H__

#include <linux/mmc/host.h>

extern int somc_wifi_init(struct platform_device *pdev);
extern void somc_wifi_deinit(struct platform_device *pdev);
extern struct wifi_platform_data somc_wifi_control;
extern void somc_wifi_mmc_host_register(struct mmc_host *host);

#endif /* __SOMC_WIFI_H__ */
