/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */
#include <asm/memory.h>
#include <linux/coresight-stm.h>
#include <linux/delay.h>
#include <linux/devfreq.h>
#include <linux/hash.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/pm_qos.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/soc/qcom/llcc-qcom.h>
#include <linux/qcom_scm.h>
#include <linux/soc/qcom/smem.h>
#include <linux/dma-mapping.h>
#include <linux/reset.h>
#include <linux/pm_wakeup.h>
#include "hfi_packetization.h"
#include "msm_cvp_debug.h"
#include "cvp_core_hfi.h"
#include "cvp_hfi_helper.h"
#include "cvp_hfi_io.h"
#include "msm_cvp_dsp.h"
#include "msm_cvp_clocks.h"
#include "cvp_dump.h"
#include "cvp_vm.h"

#define FIRMWARE_SIZE			0X00A00000

static int msm_cvp_vm_start(struct msm_cvp_core *core);
static int msm_cvp_vm_init_reg_and_irq(struct iris_hfi_device *device,
		struct msm_cvp_platform_resources *res);

static struct msm_cvp_vm_ops vm_ops = {
	.vm_start = msm_cvp_vm_start,
	.vm_init_reg_and_irq = msm_cvp_vm_init_reg_and_irq,
};

struct msm_cvp_vm_manager vm_manager = {
	.msgq_drv = &cvp_ipc_msgq,
	.vm_rm = &cvp_vm_rm,
	.vm_ops = &vm_ops,
};

static int msm_cvp_vm_start(struct msm_cvp_core *core)
{
	if (!core || !core->platform_data) {
		dprintk(CVP_ERR, "%s: Invalid params %pK %pK\n",
			__func__, core,
				(core == NULL)? NULL: core->platform_data);
		return -EINVAL;
	}

	vm_manager.vm_id = core->platform_data->vm_id;
	return 0;
}

static int __check_core_registered(struct iris_hfi_device *device,
		phys_addr_t fw_addr, u8 *reg_addr, u32 reg_size,
		phys_addr_t irq)
{
	struct cvp_hal_data *cvp_hal_data;

	if (!device) {
		dprintk(CVP_INFO, "no device Registered\n");
		return -EINVAL;
	}

	cvp_hal_data = device->cvp_hal_data;
	if (!cvp_hal_data)
		return -EINVAL;

	if (cvp_hal_data->irq == irq &&
		(CONTAINS(cvp_hal_data->firmware_base,
				FIRMWARE_SIZE, fw_addr) ||
		CONTAINS(fw_addr, FIRMWARE_SIZE,
				cvp_hal_data->firmware_base) ||
		CONTAINS(cvp_hal_data->register_base,
				reg_size, reg_addr) ||
		CONTAINS(reg_addr, reg_size,
				cvp_hal_data->register_base) ||
		OVERLAPS(cvp_hal_data->register_base,
				reg_size, reg_addr, reg_size) ||
		OVERLAPS(reg_addr, reg_size,
				cvp_hal_data->register_base,
				reg_size) ||
		OVERLAPS(cvp_hal_data->firmware_base,
				FIRMWARE_SIZE, fw_addr,
				FIRMWARE_SIZE) ||
		OVERLAPS(fw_addr, FIRMWARE_SIZE,
				cvp_hal_data->firmware_base,
				FIRMWARE_SIZE))) {
		return 0;
	}

	dprintk(CVP_INFO, "Device not registered\n");
	return -EINVAL;
}

static int msm_cvp_vm_init_reg_and_irq(struct iris_hfi_device *device,
		struct msm_cvp_platform_resources *res)
{
	struct cvp_hal_data *hal = NULL;
	int rc = 0;

	if (vm_manager.vm_id == VM_TRUSTED)
		return 0;

	rc = __check_core_registered(device, res->firmware_base,
			(u8 *)(uintptr_t)res->register_base,
			res->register_size, res->irq);
	if (!rc) {
		dprintk(CVP_ERR, "Core present/Already added\n");
		rc = -EEXIST;
		goto err_core_init;
	}

	hal = kzalloc(sizeof(*hal), GFP_KERNEL);
	if (!hal) {
		dprintk(CVP_ERR, "Failed to alloc\n");
		rc = -ENOMEM;
		goto err_core_init;
	}

	hal->irq = res->irq;
	hal->irq_wd = res->irq_wd;
	hal->firmware_base = res->firmware_base;
	hal->register_base = devm_ioremap(&res->pdev->dev,
			res->register_base, res->register_size);
	hal->register_size = res->register_size;
	if (!hal->register_base) {
		dprintk(CVP_ERR,
			"could not map reg addr %pa of size %d\n",
			&res->register_base, res->register_size);
		goto error_irq_fail;
	}

	if (res->gcc_reg_base) {
		hal->gcc_reg_base = devm_ioremap(&res->pdev->dev,
				res->gcc_reg_base, res->gcc_reg_size);
		hal->gcc_reg_size = res->gcc_reg_size;
		if (!hal->gcc_reg_base)
			dprintk(CVP_ERR,
				"could not map gcc reg addr %pa of size %d\n",
				&res->gcc_reg_base, res->gcc_reg_size);
	}

	device->cvp_hal_data = hal;
	rc = request_threaded_irq(res->irq, cvp_hfi_isr, iris_hfi_core_work_handler,
			IRQF_TRIGGER_HIGH, "msm_cvp", device);
	if (unlikely(rc)) {
		dprintk(CVP_ERR, "%s: request_irq failed rc: %d\n", __func__, rc);
		goto error_irq_fail;
	}

	rc = request_irq(res->irq_wd, iris_hfi_isr_wd, IRQF_TRIGGER_HIGH,
			"msm_cvp", device);
	if (unlikely(rc)) {
		dprintk(CVP_ERR, "() :request_irq for WD failed\n");
		goto error_irq_fail;
	}

	disable_irq_nosync(res->irq);
	dprintk(CVP_INFO,
		"firmware_base = %pa, register_base = %pa, register_size = %d\n",
		&res->firmware_base, &res->register_base,
		res->register_size);
	return rc;

error_irq_fail:
	kfree(hal);
err_core_init:
	return rc;
}
