#ifndef __SOMC_WIFI_H__
#define __SOMC_WIFI_H__

#include <linux/mmc/host.h>

extern int somc_wifi_init(struct platform_device *pdev);
extern void somc_wifi_deinit(struct platform_device *pdev);
extern struct wifi_platform_data somc_wifi_control;
extern void somc_wifi_mmc_host_register(struct mmc_host *host);

#endif /* __SOMC_WIFI_H__ */
