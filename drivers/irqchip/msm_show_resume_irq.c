/* Copyright (c) 2011, 2014, The Linux Foundation. All rights reserved.
 * Copyright (c) 2011-2013 Foxconn International Holdings, Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

//CORE-PK-SuspendLog-00+[
#ifdef CONFIG_FIH_SUSPEND_RESUME_LOG
int msm_show_resume_irq_mask = 0x1;
#else
int msm_show_resume_irq_mask;
#endif
//CORE-PK-SuspendLog-00+]

module_param_named(
	debug_mask, msm_show_resume_irq_mask, int, S_IRUGO | S_IWUSR | S_IWGRP
);
