/* main_module.c
 *
 * Copyright (C) 2017 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>

extern int bd7602_dev_init(void);
extern void bd7602_dev_exit(void);
extern int cxd224x_dev_init(void);
extern void cxd224x_dev_exit(void);

static int __init nfc_carillon_init(void)
{
	int ret;

	pr_info("%s\n", __func__);

	ret = bd7602_dev_init();
	if (ret)
		goto exit;

	ret = cxd224x_dev_init();
	if (ret)
		goto exit_del_bd7602_driver;

	return 0;

exit_del_bd7602_driver:
	bd7602_dev_exit();

exit:
	return ret;
}
module_init(nfc_carillon_init);

static void __exit nfc_carillon_exit(void)
{
	pr_info("%s\n", __func__);

	cxd224x_dev_exit();
	bd7602_dev_exit();
}
module_exit(nfc_carillon_exit);

MODULE_AUTHOR("Sony Mobile Communications Inc.");
MODULE_DESCRIPTION("NFC Carillon's component driver");
MODULE_LICENSE("GPL v2");
