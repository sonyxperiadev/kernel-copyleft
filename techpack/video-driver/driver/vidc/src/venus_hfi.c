// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/iommu.h>
#include <linux/qcom_scm.h>
#include <linux/soc/qcom/smem.h>
#include <linux/irqreturn.h>
#include <linux/of_address.h>
#include <linux/firmware.h>
#include <linux/qcom_scm.h>
#include <linux/soc/qcom/mdt_loader.h>
#include <linux/soc/qcom/llcc-qcom.h>
#include <linux/iopoll.h>

#include "venus_hfi.h"
#include "msm_vidc_core.h"
#include "msm_vidc_control.h"
#include "msm_vidc_power.h"
#include "msm_vidc_platform.h"
#include "msm_vidc_memory.h"
#include "msm_vidc_debug.h"
#include "msm_vidc_driver.h"
#include "hfi_packet.h"
#include "venus_hfi_response.h"
#include "venus_hfi_queue.h"
#include "msm_vidc_events.h"
#include "msm_vidc_state.h"
#include "firmware.h"

#define update_offset(offset, val)		((offset) += (val))
#define update_timestamp(ts, val) \
	do { \
		do_div((ts), NSEC_PER_USEC); \
		(ts) += (val); \
		(ts) *= NSEC_PER_USEC; \
	} while (0)

extern struct msm_vidc_core *g_core;

static int __resume(struct msm_vidc_core *core);
static int __suspend(struct msm_vidc_core *core);

static void __fatal_error(bool fatal)
{
	WARN_ON(fatal);
}

int __strict_check(struct msm_vidc_core *core, const char *function)
{
	bool fatal = !mutex_is_locked(&core->lock);

	__fatal_error(fatal);

	if (fatal)
		d_vpr_e("%s: strict check failed\n", function);

	return fatal ? -EINVAL : 0;
}

static bool __valdiate_session(struct msm_vidc_core *core,
		struct msm_vidc_inst *inst, const char *func)
{
	bool valid = false;
	struct msm_vidc_inst *temp;
	int rc = 0;

	rc = __strict_check(core, __func__);
	if (rc)
		return false;

	list_for_each_entry(temp, &core->instances, list) {
		if (temp == inst) {
			valid = true;
			break;
		}
	}
	if (!valid)
		i_vpr_e(inst, "%s: invalid session\n", func);

	return valid;
}

static void __schedule_power_collapse_work(struct msm_vidc_core *core)
{
	if (!core->capabilities[SW_PC].value) {
		d_vpr_l("software power collapse not enabled\n");
		return;
	}

	if (!mod_delayed_work(core->pm_workq, &core->pm_work,
			msecs_to_jiffies(core->capabilities[SW_PC_DELAY].value))) {
		d_vpr_h("power collapse already scheduled\n");
	} else {
		d_vpr_l("power collapse scheduled for %d ms\n",
			core->capabilities[SW_PC_DELAY].value);
	}
}

static void __cancel_power_collapse_work(struct msm_vidc_core *core)
{
	if (!core->capabilities[SW_PC].value)
		return;

	cancel_delayed_work(&core->pm_work);
}

static void __flush_debug_queue(struct msm_vidc_core *core,
	u8 *packet, u32 packet_size)
{
	u8 *log;
	struct hfi_debug_header *pkt;
	bool local_packet = false;
	enum vidc_msg_prio_fw log_level_fw = msm_fw_debug;

	if (!packet || !packet_size) {
		packet = vzalloc(VIDC_IFACEQ_VAR_HUGE_PKT_SIZE);
		if (!packet) {
			d_vpr_e("%s: allocation failed\n", __func__);
			return;
		}
		packet_size = VIDC_IFACEQ_VAR_HUGE_PKT_SIZE;

		local_packet = true;

		/*
		 * Local packet is used when error occurred.
		 * It is good to print these logs to printk as well.
		 */
		log_level_fw |= FW_PRINTK;
	}

	while (!venus_hfi_queue_dbg_read(core, packet)) {
		pkt = (struct hfi_debug_header *)packet;

		if (pkt->size < sizeof(struct hfi_debug_header)) {
			d_vpr_e("%s: invalid pkt size %d\n",
				__func__, pkt->size);
			continue;
		}
		if (pkt->size >= packet_size) {
			d_vpr_e("%s: pkt size[%d] >= packet_size[%d]\n",
				__func__, pkt->size, packet_size);
			continue;
		}

		packet[pkt->size] = '\0';
		/*
		 * All fw messages starts with new line character. This
		 * causes dprintk to print this message in two lines
		 * in the kernel log. Ignoring the first character
		 * from the message fixes this to print it in a single
		 * line.
		 */
		log = (u8 *)packet + sizeof(struct hfi_debug_header) + 1;
		dprintk_firmware(log_level_fw, "%s", log);
	}

	if (local_packet)
		vfree(packet);
}

static int __cmdq_write(struct msm_vidc_core *core, void *pkt)
{
	int rc;

	rc = __resume(core);
	if (rc)
		return rc;

	rc = venus_hfi_queue_cmd_write(core, pkt);
	if (!rc)
		__schedule_power_collapse_work(core);

	return rc;
}

static int __cmdq_write_intr(struct msm_vidc_core *core, void *pkt, bool allow_intr)
{
	int rc;

	rc = __resume(core);
	if (rc)
		return rc;

	rc = venus_hfi_queue_cmd_write_intr(core, pkt, allow_intr);
	if (!rc)
		__schedule_power_collapse_work(core);

	return rc;
}

static int __sys_set_debug(struct msm_vidc_core *core, u32 debug)
{
	int rc = 0;

	rc = hfi_packet_sys_debug_config(core, core->packet,
			core->packet_size, debug);
	if (rc)
		goto exit;

	rc = __cmdq_write(core, core->packet);
	if (rc)
		goto exit;

exit:
	if (rc)
		d_vpr_e("Debug mode setting to FW failed\n");

	return rc;
}

static int __sys_set_power_control(struct msm_vidc_core *core, bool enable)
{
	int rc = 0;

	if (!is_core_sub_state(core, CORE_SUBSTATE_GDSC_HANDOFF)) {
		d_vpr_e("%s: skipping as power control hanfoff was not done\n",
			__func__);
		return rc;
	}

	if (!core_in_valid_state(core)) {
		d_vpr_e("%s: invalid core state %s\n",
			__func__, core_state_name(core->state));
		return rc;
	}

	rc = hfi_packet_sys_intraframe_powercollapse(core,
		core->packet, core->packet_size, enable);
	if (rc)
		return rc;

	rc = __cmdq_write(core, core->packet);
	if (rc)
		return rc;

	rc = msm_vidc_change_core_sub_state(core, 0, CORE_SUBSTATE_FW_PWR_CTRL, __func__);
	if (rc)
		return rc;

	d_vpr_h("%s: set hardware power control successful\n", __func__);

	return rc;
}

int __prepare_pc(struct msm_vidc_core *core)
{
	int rc = 0;

	rc = hfi_packet_sys_pc_prep(core, core->packet, core->packet_size);
	if (rc) {
		d_vpr_e("Failed to create sys pc prep pkt\n");
		goto err_pc_prep;
	}

	if (__cmdq_write(core, core->packet))
		rc = -ENOTEMPTY;
	if (rc)
		d_vpr_e("Failed to prepare venus for power off");
err_pc_prep:
	return rc;
}

static int __power_collapse(struct msm_vidc_core *core, bool force)
{
	int rc = 0;

	if (!is_core_sub_state(core, CORE_SUBSTATE_POWER_ENABLE)) {
		d_vpr_h("%s: Power already disabled\n", __func__);
		goto exit;
	}

	if (!core_in_valid_state(core)) {
		d_vpr_e("%s: Core not in init state\n", __func__);
		return -EINVAL;
	}

	__flush_debug_queue(core, (!force ? core->packet : NULL), core->packet_size);

	rc = call_venus_op(core, prepare_pc, core);
	if (rc)
		goto skip_power_off;

	rc = __suspend(core);
	if (rc)
		d_vpr_e("Failed __suspend\n");

exit:
	return rc;

skip_power_off:
	d_vpr_e("%s: skipped\n", __func__);
	return -EAGAIN;
}

static int __release_subcaches(struct msm_vidc_core *core)
{
	int rc = 0;
	struct subcache_info *sinfo;
	struct hfi_buffer buf;

	if (msm_vidc_syscache_disable || !is_sys_cache_present(core))
		return 0;

	if (!core->resource->subcache_set.set_to_fw) {
		d_vpr_h("Subcaches not set to Venus\n");
		return 0;
	}

	rc = hfi_create_header(core->packet, core->packet_size,
		0, core->header_id++);
	if (rc)
		return rc;

	memset(&buf, 0, sizeof(struct hfi_buffer));
	buf.type = HFI_BUFFER_SUBCACHE;
	buf.flags = HFI_BUF_HOST_FLAG_RELEASE;

	venus_hfi_for_each_subcache_reverse(core, sinfo) {
		if (!sinfo->isactive)
			continue;

		buf.index = sinfo->subcache->slice_id;
		buf.buffer_size = sinfo->subcache->slice_size;

		rc = hfi_create_packet(core->packet,
			core->packet_size,
			HFI_CMD_BUFFER,
			HFI_BUF_HOST_FLAG_NONE,
			HFI_PAYLOAD_STRUCTURE,
			HFI_PORT_NONE,
			core->packet_id++,
			&buf,
			sizeof(buf));
		if (rc)
			return rc;
	}

	/* Set resource to Venus for activated subcaches */
	rc = __cmdq_write(core, core->packet);
	if (rc)
		return rc;

	venus_hfi_for_each_subcache_reverse(core, sinfo) {
		if (!sinfo->isactive)
			continue;

		d_vpr_h("%s: release Subcache id %d size %lu done\n",
			__func__, sinfo->subcache->slice_id,
			sinfo->subcache->slice_size);
	}
	core->resource->subcache_set.set_to_fw = false;

	return 0;
}

static int __set_subcaches(struct msm_vidc_core *core)
{
	int rc = 0;
	struct subcache_info *sinfo;
	struct hfi_buffer buf;

	if (msm_vidc_syscache_disable ||
		!is_sys_cache_present(core)) {
		return 0;
	}

	if (core->resource->subcache_set.set_to_fw) {
		d_vpr_h("Subcaches already set to Venus\n");
		return 0;
	}

	rc = hfi_create_header(core->packet, core->packet_size,
		0, core->header_id++);
	if (rc)
		goto err_fail_set_subacaches;

	memset(&buf, 0, sizeof(struct hfi_buffer));
	buf.type = HFI_BUFFER_SUBCACHE;
	buf.flags = HFI_BUF_HOST_FLAG_NONE;

	venus_hfi_for_each_subcache(core, sinfo) {
		if (!sinfo->isactive)
			continue;
		buf.index = sinfo->subcache->slice_id;
		buf.buffer_size = sinfo->subcache->slice_size;

		rc = hfi_create_packet(core->packet,
			core->packet_size,
			HFI_CMD_BUFFER,
			HFI_BUF_HOST_FLAG_NONE,
			HFI_PAYLOAD_STRUCTURE,
			HFI_PORT_NONE,
			core->packet_id++,
			&buf,
			sizeof(buf));
		if (rc)
			goto err_fail_set_subacaches;
	}

	/* Set resource to Venus for activated subcaches */
	rc = __cmdq_write(core, core->packet);
	if (rc)
		goto err_fail_set_subacaches;

	venus_hfi_for_each_subcache(core, sinfo) {
		if (!sinfo->isactive)
			continue;
		d_vpr_h("%s: set Subcache id %d size %lu done\n",
			__func__, sinfo->subcache->slice_id,
			sinfo->subcache->slice_size);
	}
	core->resource->subcache_set.set_to_fw = true;

	return 0;

err_fail_set_subacaches:
	call_res_op(core, llcc, core, false);
	return rc;
}

static int __venus_power_off(struct msm_vidc_core *core)
{
	int rc = 0;

	if (!is_core_sub_state(core, CORE_SUBSTATE_POWER_ENABLE))
		return 0;

	rc = call_venus_op(core, power_off, core);
	if (rc) {
		d_vpr_e("Failed to power off, err: %d\n", rc);
		return rc;
	}
	msm_vidc_change_core_sub_state(core, CORE_SUBSTATE_POWER_ENABLE, 0, __func__);

	return rc;
}

static int __venus_power_on(struct msm_vidc_core *core)
{
	int rc = 0;

	if (is_core_sub_state(core, CORE_SUBSTATE_POWER_ENABLE))
		return 0;

	rc = call_venus_op(core, power_on, core);
	if (rc) {
		d_vpr_e("Failed to power on, err: %d\n", rc);
		return rc;
	}

	rc = msm_vidc_change_core_sub_state(core, 0, CORE_SUBSTATE_POWER_ENABLE, __func__);
	if (rc)
		return rc;

	return rc;
}

static int __suspend(struct msm_vidc_core *core)
{
	int rc = 0;

	if (!is_core_sub_state(core, CORE_SUBSTATE_POWER_ENABLE)) {
		d_vpr_h("Power already disabled\n");
		return 0;
	}

	rc = __strict_check(core, __func__);
	if (rc)
		return rc;

	d_vpr_h("Entering suspend\n");

	rc = fw_suspend(core);
	if (rc) {
		d_vpr_e("Failed to suspend video core %d\n", rc);
		goto err_tzbsp_suspend;
	}

	call_res_op(core, llcc, core, false);

	__venus_power_off(core);
	d_vpr_h("Venus power off\n");
	return rc;

err_tzbsp_suspend:
	return rc;
}

static int __resume(struct msm_vidc_core *core)
{
	int rc = 0;

	if (is_core_sub_state(core, CORE_SUBSTATE_POWER_ENABLE)) {
		goto exit;
	} else if (!core_in_valid_state(core)) {
		d_vpr_e("%s: core not in valid state\n", __func__);
		return -EINVAL;
	}

	rc = __strict_check(core, __func__);
	if (rc)
		return rc;

	d_vpr_h("Resuming from power collapse\n");
	/* reset handoff done from core sub_state */
	rc = msm_vidc_change_core_sub_state(core, CORE_SUBSTATE_GDSC_HANDOFF, 0, __func__);
	if (rc)
		return rc;
	/* reset hw pwr ctrl from core sub_state */
	rc = msm_vidc_change_core_sub_state(core, CORE_SUBSTATE_FW_PWR_CTRL, 0, __func__);
	if (rc)
		return rc;

	rc = __venus_power_on(core);
	if (rc) {
		d_vpr_e("Failed to power on venus\n");
		goto err_venus_power_on;
	}

	/* Reboot the firmware */
	rc = fw_resume(core);
	if (rc) {
		d_vpr_e("Failed to resume video core %d\n", rc);
		goto err_set_video_state;
	}

	/*
	 * Hand off control of regulators to h/w _after_ loading fw.
	 * Note that the GDSC will turn off when switching from normal
	 * (s/w triggered) to fast (HW triggered) unless the h/w vote is
	 * present.
	 */
	call_res_op(core, gdsc_hw_ctrl, core);

	/* Wait for boot completion */
	rc = call_venus_op(core, boot_firmware, core);
	if (rc) {
		d_vpr_e("Failed to reset venus core\n");
		goto err_reset_core;
	}

	__sys_set_debug(core, (msm_fw_debug & FW_LOGMASK) >> FW_LOGSHIFT);

	rc = call_res_op(core, llcc, core, true);
	if (rc) {
		d_vpr_e("Failed to activate subcache\n");
		goto err_reset_core;
	}
	__set_subcaches(core);

	rc = __sys_set_power_control(core, true);
	if (rc) {
		d_vpr_e("%s: set power control failed\n", __func__);
		call_res_op(core, gdsc_sw_ctrl, core);
		rc = 0;
	}

	d_vpr_h("Resumed from power collapse\n");
exit:
	/* Don't reset skip_pc_count for SYS_PC_PREP cmd */
	//if (core->last_packet_type != HFI_CMD_SYS_PC_PREP)
	//	core->skip_pc_count = 0;
	return rc;
err_reset_core:
	fw_suspend(core);
err_set_video_state:
	__venus_power_off(core);
err_venus_power_on:
	d_vpr_e("Failed to resume from power collapse\n");
	return rc;
}

int __load_fw(struct msm_vidc_core *core)
{
	int rc = 0;

	d_vpr_h("%s: loading video firmware\n", __func__);

	/* clear all substates */
	msm_vidc_change_core_sub_state(core, CORE_SUBSTATE_MAX - 1, 0, __func__);

	trace_msm_v4l2_vidc_fw_load("START");
	rc = __venus_power_on(core);
	if (rc) {
		d_vpr_e("%s: power on failed\n", __func__);
		goto fail_power;
	}

	rc = fw_load(core);
	if (rc)
		goto fail_load_fw;

	/*
	 * Hand off control of regulators to h/w _after_ loading fw.
	 * Note that the GDSC will turn off when switching from normal
	 * (s/w triggered) to fast (HW triggered) unless the h/w vote is
	 * present.
	 */
	call_res_op(core, gdsc_hw_ctrl, core);
	trace_msm_v4l2_vidc_fw_load("END");

	return rc;
fail_load_fw:
	__venus_power_off(core);
fail_power:
	trace_msm_v4l2_vidc_fw_load("END");
	return rc;
}

void __unload_fw(struct msm_vidc_core *core)
{
	if (!core->resource->fw_cookie)
		return;

	cancel_delayed_work(&core->pm_work);
	fw_unload(core);
	__venus_power_off(core);

	/* clear all substates */
	msm_vidc_change_core_sub_state(core, CORE_SUBSTATE_MAX - 1, 0, __func__);

	d_vpr_h("%s unloaded video firmware\n", __func__);
}

static int __response_handler(struct msm_vidc_core *core)
{
	int rc = 0;

	if (call_venus_op(core, watchdog, core, core->intr_status)) {
		struct hfi_packet pkt = {.type = HFI_SYS_ERROR_WD_TIMEOUT};

		core_lock(core, __func__);
		msm_vidc_change_core_state(core, MSM_VIDC_CORE_ERROR, __func__);
		/* mark cpu watchdog error */
		msm_vidc_change_core_sub_state(core,
			0, CORE_SUBSTATE_CPU_WATCHDOG, __func__);
		d_vpr_e("%s: CPU WD error received\n", __func__);
		core_unlock(core, __func__);

		return handle_system_error(core, &pkt);
	}

	memset(core->response_packet, 0, core->packet_size);
	while (!venus_hfi_queue_msg_read(core, core->response_packet)) {
		rc = handle_response(core, core->response_packet);
		if (rc)
			continue;
		/* check for system error */
		if (core->state != MSM_VIDC_CORE_INIT)
			break;
		memset(core->response_packet, 0, core->packet_size);
	}

	__schedule_power_collapse_work(core);
	__flush_debug_queue(core, core->response_packet, core->packet_size);

	return rc;
}

irqreturn_t venus_hfi_isr(int irq, void *data)
{
	disable_irq_nosync(irq);
	return IRQ_WAKE_THREAD;
}

irqreturn_t venus_hfi_isr_handler(int irq, void *data)
{
	struct msm_vidc_core *core = data;
	int num_responses = 0, rc = 0;

	d_vpr_l("%s: received interrupt from hardware\n", __func__);

	if (!core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return IRQ_NONE;
	}

	core_lock(core, __func__);
	rc = __resume(core);
	if (rc) {
		d_vpr_e("%s: Power on failed\n", __func__);
		core_unlock(core, __func__);
		goto exit;
	}
	call_venus_op(core, clear_interrupt, core);
	core_unlock(core, __func__);

	num_responses = __response_handler(core);

exit:
	if (!call_venus_op(core, watchdog, core, core->intr_status))
		enable_irq(irq);

	return IRQ_HANDLED;
}

void venus_hfi_pm_work_handler(struct work_struct *work)
{
	int rc = 0;
	struct msm_vidc_core *core;

	core = container_of(work, struct msm_vidc_core, pm_work.work);

	core_lock(core, __func__);
	d_vpr_h("%s: try power collapse\n", __func__);
	/*
	 * It is ok to check this variable outside the lock since
	 * it is being updated in this context only
	 */
	if (core->skip_pc_count >= VIDC_MAX_PC_SKIP_COUNT) {
		d_vpr_e("Failed to PC for %d times\n",
				core->skip_pc_count);
		core->skip_pc_count = 0;
		msm_vidc_change_core_state(core, MSM_VIDC_CORE_ERROR, __func__);
		/* mark video hw unresponsive */
		msm_vidc_change_core_sub_state(core,
			0, CORE_SUBSTATE_VIDEO_UNRESPONSIVE, __func__);
		/* do core deinit to handle error */
		msm_vidc_core_deinit_locked(core, true);
		goto unlock;
	}

	/* core already deinited - skip power collapse */
	if (is_core_state(core, MSM_VIDC_CORE_DEINIT)) {
		d_vpr_e("%s: invalid core state %s\n",
			__func__, core_state_name(core->state));
		goto unlock;
	}

	rc = __power_collapse(core, false);
	switch (rc) {
	case 0:
		core->skip_pc_count = 0;
		/* Cancel pending delayed works if any */
		__cancel_power_collapse_work(core);
		d_vpr_h("%s: power collapse successful!\n", __func__);
		break;
	case -EBUSY:
		core->skip_pc_count = 0;
		d_vpr_h("%s: retry PC as dsp is busy\n", __func__);
		__schedule_power_collapse_work(core);
		break;
	case -EAGAIN:
		core->skip_pc_count++;
		d_vpr_e("%s: retry power collapse (count %d)\n",
			__func__, core->skip_pc_count);
		__schedule_power_collapse_work(core);
		break;
	default:
		d_vpr_e("%s: power collapse failed\n", __func__);
		break;
	}
unlock:
	core_unlock(core, __func__);
}

static int __sys_init(struct msm_vidc_core *core)
{
	int rc = 0;

	rc =  hfi_packet_sys_init(core, core->packet, core->packet_size);
	if (rc)
		return rc;

	rc = __cmdq_write(core, core->packet);
	if (rc)
		return rc;

	return 0;
}

static int __sys_image_version(struct msm_vidc_core *core)
{
	int rc = 0;

	rc = hfi_packet_image_version(core, core->packet, core->packet_size);
	if (rc)
		return rc;

	rc = __cmdq_write(core, core->packet);
	if (rc)
		return rc;

	return 0;
}

int venus_hfi_core_init(struct msm_vidc_core *core)
{
	int rc = 0;

	d_vpr_h("%s(): core %pK\n", __func__, core);

	rc = __strict_check(core, __func__);
	if (rc)
		return rc;

	rc = venus_hfi_queue_init(core);
	if (rc)
		goto error;

	rc = __load_fw(core);
	if (rc)
		goto error;

	rc = call_venus_op(core, boot_firmware, core);
	if (rc)
		goto error;

	rc = call_res_op(core, llcc, core, true);
	if (rc)
		goto error;

	rc = __sys_init(core);
	if (rc)
		goto error;

	rc = __sys_image_version(core);
	if (rc)
		goto error;

	rc = __sys_set_debug(core, (msm_fw_debug & FW_LOGMASK) >> FW_LOGSHIFT);
	if (rc)
		goto error;

	rc = __set_subcaches(core);
	if (rc)
		goto error;

	rc = __sys_set_power_control(core, true);
	if (rc) {
		d_vpr_e("%s: set power control failed\n", __func__);
		call_res_op(core, gdsc_sw_ctrl, core);
		rc = 0;
	}

	d_vpr_h("%s(): successful\n", __func__);
	return 0;

error:
	d_vpr_e("%s(): failed\n", __func__);
	return rc;
}

int venus_hfi_core_deinit(struct msm_vidc_core *core, bool force)
{
	int rc = 0;

	d_vpr_h("%s(): core %pK\n", __func__, core);
	rc = __strict_check(core, __func__);
	if (rc)
		return rc;

	if (is_core_state(core, MSM_VIDC_CORE_DEINIT))
		return 0;
	__resume(core);
	__flush_debug_queue(core, (!force ? core->packet : NULL), core->packet_size);
	__release_subcaches(core);
	call_res_op(core, llcc, core, false);
	__unload_fw(core);
	/**
	 * coredump need to be called after firmware unload, coredump also
	 * copying queues memory. So need to be called before queues deinit.
	 */
	if (msm_vidc_fw_dump)
		fw_coredump(core);

	return 0;
}

int venus_hfi_noc_error_info(struct msm_vidc_core *core)
{
	int rc = 0;

	if (!core->capabilities[NON_FATAL_FAULTS].value)
		return 0;

	core_lock(core, __func__);
	if (is_core_state(core, MSM_VIDC_CORE_DEINIT))
		goto unlock;

	/* resume venus before accessing noc registers */
	rc = __resume(core);
	if (rc) {
		d_vpr_e("%s: Power on failed\n", __func__);
		goto unlock;
	}

	call_venus_op(core, noc_error_info, core);

unlock:
	core_unlock(core, __func__);
	return rc;
}

int venus_hfi_suspend(struct msm_vidc_core *core)
{
	int rc = 0;

	rc = __strict_check(core, __func__);
	if (rc)
		return rc;

	if (!core->capabilities[SW_PC].value) {
		d_vpr_h("Skip suspending venus\n");
		return 0;
	}

	d_vpr_h("Suspending Venus\n");
	rc = __power_collapse(core, true);
	if (!rc) {
		/* Cancel pending delayed works if any */
		__cancel_power_collapse_work(core);
	} else {
		d_vpr_e("%s: Venus is busy\n", __func__);
		rc = -EBUSY;
	}

	return rc;
}

int venus_hfi_trigger_ssr(struct msm_vidc_core *core, u32 type,
	u32 client_id, u32 addr)
{
	int rc = 0;
	u32 payload[2];

	// WA for CR: 3584248
	if (type != HFI_SSR_TYPE_SW_ERR_FATAL) {
		d_vpr_h("SSR Type 0x1 is only allowed for pineapple\n");
		return rc;
	}

	/*
	 * call resume before preparing ssr hfi packet in core->packet
	 * otherwise ssr hfi packet in core->packet will be overwritten
	 * by __resume() call (inside __cmdq_write) which is preparing
	 * ifpc hfi packets in core->packet
	 */
	rc = __resume(core);
	if (rc)
		return rc;

	payload[0] = client_id << 4 | type;
	payload[1] = addr;

	rc = hfi_create_header(core->packet, core->packet_size,
			   0 /*session_id*/,
			   core->header_id++);
	if (rc)
		goto exit;

	/* HFI_CMD_SSR */
	rc = hfi_create_packet(core->packet, core->packet_size,
				   HFI_CMD_SSR,
				   HFI_HOST_FLAGS_RESPONSE_REQUIRED |
				   HFI_HOST_FLAGS_INTR_REQUIRED,
				   HFI_PAYLOAD_U64,
				   HFI_PORT_NONE,
				   core->packet_id++,
				   &payload, sizeof(u64));
	if (rc)
		goto exit;

	rc = __cmdq_write(core, core->packet);
	if (rc)
		goto exit;

exit:
	if (rc)
		d_vpr_e("%s(): failed\n", __func__);

	return rc;
}

int venus_hfi_trigger_stability(struct msm_vidc_inst *inst, u32 type,
	u32 client_id, u32 val)
{
	struct msm_vidc_core *core;
	u32 payload[2];
	int rc = 0;

	if (!inst->packet) {
		d_vpr_e("%s: Invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;
	core_lock(core, __func__);

	if (!__valdiate_session(core, inst, __func__)) {
		rc = -EINVAL;
		goto unlock;
	}

	payload[0] = client_id << 4 | type;
	payload[1] = val;
	rc = hfi_create_header(inst->packet, inst->packet_size,
			   inst->session_id, core->header_id++);
	if (rc)
		goto unlock;

	/* HFI_CMD_STABILITY */
	rc = hfi_create_packet(inst->packet, inst->packet_size,
				   HFI_CMD_STABILITY,
				   HFI_HOST_FLAGS_RESPONSE_REQUIRED |
				   HFI_HOST_FLAGS_INTR_REQUIRED,
				   HFI_PAYLOAD_U64,
				   HFI_PORT_NONE,
				   core->packet_id++,
				   &payload, sizeof(u64));
	if (rc)
		goto unlock;

	rc = __cmdq_write(core, inst->packet);
	if (rc)
		goto unlock;

unlock:
	core_unlock(core, __func__);
	return rc;
}

int venus_hfi_reserve_hardware(struct msm_vidc_inst *inst, u32 duration)
{
	struct msm_vidc_core *core;
	enum hfi_reserve_type payload;
	int rc = 0;

	if (!inst->packet) {
		d_vpr_e("%s: Invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;
	core_lock(core, __func__);

	if (!__valdiate_session(core, inst, __func__)) {
		rc = -EINVAL;
		goto unlock;
	}

	if (duration)
		payload = HFI_RESERVE_START;
	else
		payload = HFI_RESERVE_STOP;

	rc = hfi_create_header(inst->packet, inst->packet_size,
		inst->session_id, core->header_id++);
	if (rc)
		goto unlock;

	rc = hfi_create_packet(inst->packet, inst->packet_size,
		HFI_CMD_RESERVE,
		HFI_HOST_FLAGS_NONE,
		HFI_PAYLOAD_U32_ENUM,
		HFI_PORT_NONE,
		core->packet_id++,
		&payload, sizeof(u32));
	if (rc)
		goto unlock;

	rc = __cmdq_write(core, inst->packet);
	if (rc)
		goto unlock;

unlock:
	core_unlock(core, __func__);
	return rc;
}

int venus_hfi_session_open(struct msm_vidc_inst *inst)
{
	int rc = 0;
	struct msm_vidc_core *core;

	if (!inst->packet) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;
	core_lock(core, __func__);

	if (!__valdiate_session(core, inst, __func__)) {
		rc = -EINVAL;
		goto unlock;
	}

	__sys_set_debug(core, (msm_fw_debug & FW_LOGMASK) >> FW_LOGSHIFT);

	rc = hfi_packet_session_command(inst,
				HFI_CMD_OPEN,
				(HFI_HOST_FLAGS_RESPONSE_REQUIRED |
				HFI_HOST_FLAGS_INTR_REQUIRED),
				HFI_PORT_NONE,
				0, /* session_id */
				HFI_PAYLOAD_U32,
				&inst->session_id, /* payload */
				sizeof(u32));
	if (rc)
		goto unlock;

	rc = __cmdq_write(inst->core, inst->packet);
	if (rc)
		goto unlock;

unlock:
	core_unlock(core, __func__);
	return rc;
}

int venus_hfi_session_set_codec(struct msm_vidc_inst *inst)
{
	int rc = 0;
	struct msm_vidc_core *core;
	u32 codec;

	if (!inst->packet) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;
	core_lock(core, __func__);

	if (!__valdiate_session(core, inst, __func__)) {
		rc = -EINVAL;
		goto unlock;
	}

	rc = hfi_create_header(inst->packet, inst->packet_size,
			inst->session_id, core->header_id++);
	if (rc)
		goto unlock;

	codec = get_hfi_codec(inst);
	rc = hfi_create_packet(inst->packet, inst->packet_size,
			HFI_PROP_CODEC,
			HFI_HOST_FLAGS_NONE,
			HFI_PAYLOAD_U32_ENUM,
			HFI_PORT_NONE,
			core->packet_id++,
			&codec,
			sizeof(u32));
	if (rc)
		goto unlock;

	rc = __cmdq_write(inst->core, inst->packet);
	if (rc)
		goto unlock;

unlock:
	core_unlock(core, __func__);
	return rc;
}

int venus_hfi_session_set_secure_mode(struct msm_vidc_inst *inst)
{
	int rc = 0;
	struct msm_vidc_core *core;
	u32 secure_mode;

	if (!inst->packet) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;
	core_lock(core, __func__);

	if (!__valdiate_session(core, inst, __func__)) {
		rc = -EINVAL;
		goto unlock;
	}

	rc = hfi_create_header(inst->packet, inst->packet_size,
			inst->session_id, core->header_id++);
	if (rc)
		goto unlock;

	secure_mode = inst->capabilities[SECURE_MODE].value;
	rc = hfi_create_packet(inst->packet, inst->packet_size,
			HFI_PROP_SECURE,
			HFI_HOST_FLAGS_NONE,
			HFI_PAYLOAD_U32,
			HFI_PORT_NONE,
			core->packet_id++,
			&secure_mode,
			sizeof(u32));
	if (rc)
		goto unlock;

	rc = __cmdq_write(inst->core, inst->packet);
	if (rc)
		goto unlock;

unlock:
	core_unlock(core, __func__);
	return rc;
}

static int venus_hfi_cache_packet(struct msm_vidc_inst *inst)
{
	int rc = 0;
	struct hfi_header *hdr;
	struct hfi_pending_packet *packet;
	struct msm_vidc_core *core;

	if (!inst->packet) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;

	hdr = (struct hfi_header *)inst->packet;
	if (hdr->size < sizeof(struct hfi_header)) {
		d_vpr_e("%s: invalid hdr size %d\n", __func__, hdr->size);
		return -EINVAL;
	}

	packet = msm_vidc_pool_alloc(inst, MSM_MEM_POOL_PACKET);
	if (!packet) {
		i_vpr_e(inst, "%s: failed to allocate pending packet\n", __func__);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&packet->list);
	list_add_tail(&packet->list, &inst->pending_pkts);
	packet->data = (u8 *)packet + sizeof(struct hfi_pending_packet);

	if (hdr->size > MSM_MEM_POOL_PACKET_SIZE) {
		i_vpr_e(inst, "%s: header size %d exceeds pool packet size %d\n",
			__func__, hdr->size, MSM_MEM_POOL_PACKET_SIZE);
		return -EINVAL;
	}
	memcpy(packet->data, inst->packet, hdr->size);

	return rc;
}

int venus_hfi_session_property(struct msm_vidc_inst *inst,
	u32 pkt_type, u32 flags, u32 port, u32 payload_type,
	void *payload, u32 payload_size)
{
	int rc = 0;
	struct msm_vidc_core *core;

	if (!inst->packet) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;
	core_lock(core, __func__);

	if (!__valdiate_session(core, inst, __func__)) {
		rc = -EINVAL;
		goto unlock;
	}

	rc = hfi_create_header(inst->packet, inst->packet_size,
				inst->session_id, core->header_id++);
	if (rc)
		goto unlock;
	rc = hfi_create_packet(inst->packet, inst->packet_size,
				pkt_type,
				flags,
				payload_type,
				port,
				core->packet_id++,
				payload,
				payload_size);
	if (rc)
		goto unlock;

	/* skip sending packet to firmware */
	if (inst->request) {
		rc = venus_hfi_cache_packet(inst);
		goto unlock;
	}

	rc = __cmdq_write(inst->core, inst->packet);
	if (rc)
		goto unlock;

unlock:
	core_unlock(core, __func__);
	return rc;
}

int venus_hfi_session_close(struct msm_vidc_inst *inst)
{
	int rc = 0;
	struct msm_vidc_core *core;

	if (!inst->packet) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;
	core_lock(core, __func__);

	if (!__valdiate_session(core, inst, __func__)) {
		rc = -EINVAL;
		goto unlock;
	}

	rc = hfi_packet_session_command(inst,
				HFI_CMD_CLOSE,
				(HFI_HOST_FLAGS_RESPONSE_REQUIRED |
				HFI_HOST_FLAGS_INTR_REQUIRED |
				HFI_HOST_FLAGS_NON_DISCARDABLE),
				HFI_PORT_NONE,
				inst->session_id,
				HFI_PAYLOAD_NONE,
				NULL,
				0);
	if (rc)
		goto unlock;

	rc = __cmdq_write(inst->core, inst->packet);
	if (rc)
		goto unlock;

unlock:
	core_unlock(core, __func__);
	return rc;
}

int venus_hfi_start(struct msm_vidc_inst *inst, enum msm_vidc_port_type port)
{
	int rc = 0;
	struct msm_vidc_core *core;

	if (!inst->packet) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;
	core_lock(core, __func__);

	if (!__valdiate_session(core, inst, __func__)) {
		rc = -EINVAL;
		goto unlock;
	}

	if (port != INPUT_PORT && port != OUTPUT_PORT) {
		i_vpr_e(inst, "%s: invalid port %d\n", __func__, port);
		goto unlock;
	}

	rc = hfi_packet_session_command(inst,
				HFI_CMD_START,
				(HFI_HOST_FLAGS_RESPONSE_REQUIRED |
				HFI_HOST_FLAGS_INTR_REQUIRED),
				get_hfi_port(inst, port),
				inst->session_id,
				HFI_PAYLOAD_NONE,
				NULL,
				0);
	if (rc)
		goto unlock;

	rc = __cmdq_write(inst->core, inst->packet);
	if (rc)
		goto unlock;

unlock:
	core_unlock(core, __func__);
	return rc;
}

int venus_hfi_stop(struct msm_vidc_inst *inst, enum msm_vidc_port_type port)
{
	int rc = 0;
	struct msm_vidc_core *core;

	if (!inst->packet) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;
	core_lock(core, __func__);

	if (!__valdiate_session(core, inst, __func__)) {
		rc = -EINVAL;
		goto unlock;
	}

	if (port != INPUT_PORT && port != OUTPUT_PORT) {
		i_vpr_e(inst, "%s: invalid port %d\n", __func__, port);
		goto unlock;
	}

	rc = hfi_packet_session_command(inst,
				HFI_CMD_STOP,
				(HFI_HOST_FLAGS_RESPONSE_REQUIRED |
				HFI_HOST_FLAGS_INTR_REQUIRED |
				HFI_HOST_FLAGS_NON_DISCARDABLE),
				get_hfi_port(inst, port),
				inst->session_id,
				HFI_PAYLOAD_NONE,
				NULL,
				0);
	if (rc)
		goto unlock;

	rc = __cmdq_write(inst->core, inst->packet);
	if (rc)
		goto unlock;

unlock:
	core_unlock(core, __func__);
	return rc;
}

int venus_hfi_session_pause(struct msm_vidc_inst *inst, enum msm_vidc_port_type port)
{
	int rc = 0;
	struct msm_vidc_core *core;

	if (!inst->packet) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;
	core_lock(core, __func__);

	if (!__valdiate_session(core, inst, __func__)) {
		rc = -EINVAL;
		goto unlock;
	}

	if (port != INPUT_PORT && port != OUTPUT_PORT) {
		i_vpr_e(inst, "%s: invalid port %d\n", __func__, port);
		goto unlock;
	}

	rc = hfi_packet_session_command(inst,
				HFI_CMD_PAUSE,
				(HFI_HOST_FLAGS_RESPONSE_REQUIRED |
				HFI_HOST_FLAGS_INTR_REQUIRED),
				get_hfi_port(inst, port),
				inst->session_id,
				HFI_PAYLOAD_NONE,
				NULL,
				0);
	if (rc)
		goto unlock;

	rc = __cmdq_write(inst->core, inst->packet);
	if (rc)
		goto unlock;

unlock:
	core_unlock(core, __func__);
	return rc;
}

int venus_hfi_session_resume(struct msm_vidc_inst *inst,
	enum msm_vidc_port_type port, u32 payload)
{
	int rc = 0;
	struct msm_vidc_core *core;

	if (!inst->packet) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;
	core_lock(core, __func__);

	if (!__valdiate_session(core, inst, __func__)) {
		rc = -EINVAL;
		goto unlock;
	}

	if (port != INPUT_PORT && port != OUTPUT_PORT) {
		i_vpr_e(inst, "%s: invalid port %d\n", __func__, port);
		goto unlock;
	}

	rc = hfi_packet_session_command(inst,
				HFI_CMD_RESUME,
				(HFI_HOST_FLAGS_RESPONSE_REQUIRED |
				HFI_HOST_FLAGS_INTR_REQUIRED),
				get_hfi_port(inst, port),
				inst->session_id,
				HFI_PAYLOAD_U32,
				&payload,
				sizeof(u32));
	if (rc)
		goto unlock;

	rc = __cmdq_write(inst->core, inst->packet);
	if (rc)
		goto unlock;

unlock:
	core_unlock(core, __func__);
	return rc;
}

int venus_hfi_session_drain(struct msm_vidc_inst *inst, enum msm_vidc_port_type port)
{
	int rc = 0;
	struct msm_vidc_core *core;

	if (!inst->packet) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;
	core_lock(core, __func__);

	if (!__valdiate_session(core, inst, __func__)) {
		rc = -EINVAL;
		goto unlock;
	}

	if (port != INPUT_PORT) {
		i_vpr_e(inst, "%s: invalid port %d\n", __func__, port);
		goto unlock;
	}

	rc = hfi_packet_session_command(inst,
				HFI_CMD_DRAIN,
				(HFI_HOST_FLAGS_RESPONSE_REQUIRED |
				HFI_HOST_FLAGS_INTR_REQUIRED |
				HFI_HOST_FLAGS_NON_DISCARDABLE),
				get_hfi_port(inst, port),
				inst->session_id,
				HFI_PAYLOAD_NONE,
				NULL,
				0);
	if (rc)
		goto unlock;

	rc = __cmdq_write(inst->core, inst->packet);
	if (rc)
		goto unlock;

unlock:
	core_unlock(core, __func__);
	return rc;
}

int venus_hfi_session_command(struct msm_vidc_inst *inst,
	u32 cmd, enum msm_vidc_port_type port, u32 payload_type,
	void *payload, u32 payload_size)
{
	int rc = 0;
	struct msm_vidc_core *core;

	if (!inst->packet) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;
	core_lock(core, __func__);

	if (!__valdiate_session(core, inst, __func__)) {
		rc = -EINVAL;
		goto unlock;
	}

	rc = hfi_create_header(inst->packet, inst->packet_size,
			inst->session_id,
			core->header_id++);
	if (rc)
		goto unlock;

	rc = hfi_create_packet(inst->packet, inst->packet_size,
			cmd,
			(HFI_HOST_FLAGS_RESPONSE_REQUIRED |
			HFI_HOST_FLAGS_INTR_REQUIRED),
			payload_type,
			get_hfi_port(inst, port),
			core->packet_id++,
			payload,
			payload_size);
	if (rc)
		goto unlock;

	rc = __cmdq_write(inst->core, inst->packet);
	if (rc)
		goto unlock;

unlock:
	core_unlock(core, __func__);
	return rc;
}

int venus_hfi_queue_super_buffer(struct msm_vidc_inst *inst,
	struct msm_vidc_buffer *buffer, struct msm_vidc_buffer *metabuf)
{
	int rc = 0;
	struct msm_vidc_core *core;
	struct hfi_buffer hfi_buffer;
	struct hfi_buffer hfi_meta_buffer;
	u32 frame_size, meta_size, batch_size, cnt = 0;
	u64 ts_delta_us;

	if (!inst->packet) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;
	core_lock(core, __func__);

	if (!__valdiate_session(core, inst, __func__)) {
		rc = -EINVAL;
		goto unlock;
	}

	/* Get super yuv buffer */
	rc = get_hfi_buffer(inst, buffer, &hfi_buffer);
	if (rc)
		goto unlock;

	/* Get super meta buffer */
	if (metabuf) {
		rc = get_hfi_buffer(inst, metabuf, &hfi_meta_buffer);
		if (rc)
			goto unlock;
	}

	batch_size = inst->capabilities[SUPER_FRAME].value;
	frame_size = call_session_op(core, buffer_size, inst, MSM_VIDC_BUF_INPUT);
	meta_size = call_session_op(core, buffer_size, inst, MSM_VIDC_BUF_INPUT_META);
	ts_delta_us = 1000000 / (inst->capabilities[FRAME_RATE].value >> 16);

	/* Sanitize super yuv buffer */
	if (frame_size * batch_size != buffer->buffer_size) {
		i_vpr_e(inst, "%s: invalid super yuv buffer. frame %u, batch %u, buffer size %u\n",
			__func__, frame_size, batch_size, buffer->buffer_size);
		goto unlock;
	}

	/* Sanitize super meta buffer */
	if (metabuf && meta_size * batch_size > metabuf->buffer_size) {
		i_vpr_e(inst, "%s: invalid super meta buffer. meta %u, batch %u, buffer size %u\n",
			__func__, meta_size, batch_size, metabuf->buffer_size);
		goto unlock;
	}

	/* Initialize yuv buffer */
	hfi_buffer.data_size = frame_size;
	hfi_buffer.addr_offset = 0;

	/* Initialize meta buffer */
	if (metabuf) {
		hfi_meta_buffer.data_size = meta_size;
		hfi_meta_buffer.addr_offset = 0;
	}

	while (cnt < batch_size) {
		/* Create header */
		rc = hfi_create_header(inst->packet, inst->packet_size,
				inst->session_id, core->header_id++);
		if (rc)
			goto unlock;

		/* Create yuv packet */
		update_offset(hfi_buffer.addr_offset, (cnt ? frame_size : 0u));
		update_timestamp(hfi_buffer.timestamp, (cnt ? ts_delta_us : 0u));
		rc = hfi_create_packet(inst->packet,
				inst->packet_size,
				HFI_CMD_BUFFER,
				HFI_HOST_FLAGS_INTR_REQUIRED,
				HFI_PAYLOAD_STRUCTURE,
				get_hfi_port_from_buffer_type(inst, buffer->type),
				core->packet_id++,
				&hfi_buffer,
				sizeof(hfi_buffer));
		if (rc)
			goto unlock;

		/* Create meta packet */
		if (metabuf) {
			update_offset(hfi_meta_buffer.addr_offset, (cnt ? meta_size : 0u));
			update_timestamp(hfi_meta_buffer.timestamp, (cnt ? ts_delta_us : 0u));
			rc = hfi_create_packet(inst->packet,
				inst->packet_size,
				HFI_CMD_BUFFER,
				HFI_HOST_FLAGS_INTR_REQUIRED,
				HFI_PAYLOAD_STRUCTURE,
				get_hfi_port_from_buffer_type(inst, metabuf->type),
				core->packet_id++,
				&hfi_meta_buffer,
				sizeof(hfi_meta_buffer));
			if (rc)
				goto unlock;
		}

		/* Raise interrupt only for last pkt in the batch */
		rc = __cmdq_write_intr(inst->core, inst->packet,
				       (cnt == batch_size - 1));
		if (rc)
			goto unlock;

		/* update start timestamp */
		msm_vidc_add_buffer_stats(inst, buffer, hfi_buffer.timestamp);

		cnt++;
	}
unlock:
	core_unlock(core, __func__);
	if (rc)
		i_vpr_e(inst, "%s: queue super buffer failed: %d\n", __func__, rc);

	return rc;
}

static int venus_hfi_add_pending_packets(struct msm_vidc_inst *inst)
{
	int rc = 0;
	int num_packets = 0;
	struct hfi_pending_packet *pkt_info, *dummy;
	struct hfi_header *hdr, *src_hdr;
	struct hfi_packet *src_pkt;
	struct msm_vidc_core *core;

	if (!inst->packet) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;

	hdr = (struct hfi_header *)inst->packet;
	if (hdr->size < sizeof(struct hfi_header)) {
		i_vpr_e(inst, "%s: invalid hdr size %d\n",
			__func__, hdr->size);
		return -EINVAL;
	}

	list_for_each_entry_safe(pkt_info, dummy, &inst->pending_pkts, list) {
		src_hdr = (struct hfi_header *)(pkt_info->data);
		num_packets = src_hdr->num_packets;
		src_pkt = (struct hfi_packet *)((u8 *)src_hdr + sizeof(struct hfi_header));
		while (num_packets > 0) {
			memcpy((u8 *)hdr + hdr->size, (void *)src_pkt, src_pkt->size);
			hdr->num_packets++;
			hdr->size += src_pkt->size;
			num_packets--;
			src_pkt = (struct hfi_packet *)((u8 *)src_pkt + src_pkt->size);
			if ((u8 *)src_pkt < (u8 *)src_hdr ||
					(u8 *)src_pkt > (u8 *)src_hdr + hdr->size) {
				i_vpr_e(inst, "%s: invalid packet address\n", __func__);
				return -EINVAL;
			}
		}
		list_del(&pkt_info->list);
		msm_vidc_pool_free(inst, pkt_info);
	}
	return rc;
}

int venus_hfi_queue_buffer(struct msm_vidc_inst *inst,
	struct msm_vidc_buffer *buffer, struct msm_vidc_buffer *metabuf)
{
	int rc = 0;
	struct msm_vidc_core *core;
	struct hfi_buffer hfi_buffer, hfi_meta_buffer;

	if (!inst->packet) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;
	core_lock(core, __func__);

	if (!__valdiate_session(core, inst, __func__)) {
		rc = -EINVAL;
		goto unlock;
	}

	rc = get_hfi_buffer(inst, buffer, &hfi_buffer);
	if (rc)
		goto unlock;

	rc = hfi_create_header(inst->packet, inst->packet_size,
			   inst->session_id, core->header_id++);
	if (rc)
		goto unlock;

	rc = hfi_create_packet(inst->packet,
			inst->packet_size,
			HFI_CMD_BUFFER,
			HFI_HOST_FLAGS_INTR_REQUIRED,
			HFI_PAYLOAD_STRUCTURE,
			get_hfi_port_from_buffer_type(inst, buffer->type),
			core->packet_id++,
			&hfi_buffer,
			sizeof(hfi_buffer));
	if (rc)
		goto unlock;

	if (metabuf) {
		rc = get_hfi_buffer(inst, metabuf, &hfi_meta_buffer);
		if (rc)
			goto unlock;
		rc = hfi_create_packet(inst->packet,
			inst->packet_size,
			HFI_CMD_BUFFER,
			HFI_HOST_FLAGS_INTR_REQUIRED,
			HFI_PAYLOAD_STRUCTURE,
			get_hfi_port_from_buffer_type(inst, metabuf->type),
			core->packet_id++,
			&hfi_meta_buffer,
			sizeof(hfi_meta_buffer));
		if (rc)
			goto unlock;
	}

	if (is_meta_rx_inp_enabled(inst, META_OUTBUF_FENCE) &&
		is_output_buffer(buffer->type)) {
		if (!buffer->fence_id) {
			i_vpr_e(inst, "%s: fence id cannot be 0\n", __func__);
			rc = -EINVAL;
			goto unlock;
		}
		rc = hfi_create_packet(inst->packet,
			inst->packet_size,
			HFI_PROP_FENCE,
			0,
			HFI_PAYLOAD_U64,
			HFI_PORT_RAW,
			core->packet_id++,
			&buffer->fence_id,
			sizeof(u64));
		if (rc)
			goto unlock;
	}

	rc = venus_hfi_add_pending_packets(inst);
	if (rc)
		goto unlock;

	rc = __cmdq_write(inst->core, inst->packet);
	if (rc)
		goto unlock;

	/* update start timestamp */
	msm_vidc_add_buffer_stats(inst, buffer, hfi_buffer.timestamp);

unlock:
	core_unlock(core, __func__);
	return rc;
}

int venus_hfi_release_buffer(struct msm_vidc_inst *inst,
	struct msm_vidc_buffer *buffer)
{
	int rc = 0;
	struct msm_vidc_core *core;
	struct hfi_buffer hfi_buffer;

	if (!inst->packet || !buffer) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;
	core_lock(core, __func__);

	if (!__valdiate_session(core, inst, __func__)) {
		rc = -EINVAL;
		goto unlock;
	}

	rc = get_hfi_buffer(inst, buffer, &hfi_buffer);
	if (rc)
		goto unlock;

	/* add release flag */
	hfi_buffer.flags |= HFI_BUF_HOST_FLAG_RELEASE;

	rc = hfi_create_header(inst->packet, inst->packet_size,
			   inst->session_id, core->header_id++);
	if (rc)
		goto unlock;

	rc = hfi_create_packet(inst->packet,
			inst->packet_size,
			HFI_CMD_BUFFER,
			(HFI_HOST_FLAGS_RESPONSE_REQUIRED |
			HFI_HOST_FLAGS_INTR_REQUIRED),
			HFI_PAYLOAD_STRUCTURE,
			get_hfi_port_from_buffer_type(inst, buffer->type),
			core->packet_id++,
			&hfi_buffer,
			sizeof(hfi_buffer));
	if (rc)
		goto unlock;

	rc = __cmdq_write(inst->core, inst->packet);
	if (rc)
		goto unlock;

unlock:
	core_unlock(core, __func__);
	return rc;
}

int venus_hfi_scale_clocks(struct msm_vidc_inst *inst, u64 freq)
{
	int rc = 0;
	struct msm_vidc_core *core;

	core = inst->core;

	core_lock(core, __func__);
	rc = __resume(core);
	if (rc) {
		i_vpr_e(inst, "%s: Resume from power collapse failed\n", __func__);
		goto exit;
	}
	rc = call_res_op(core, set_clks, core, freq);
	if (rc)
		goto exit;

exit:
	core_unlock(core, __func__);

	return rc;
}

int venus_hfi_scale_buses(struct msm_vidc_inst *inst, u64 bw_ddr, u64 bw_llcc)
{
	int rc = 0;
	struct msm_vidc_core *core;

	core = inst->core;

	core_lock(core, __func__);
	rc = __resume(core);
	if (rc) {
		i_vpr_e(inst, "%s: Resume from power collapse failed\n", __func__);
		goto exit;
	}
	rc = call_res_op(core, set_bw, core, bw_ddr, bw_llcc);
	if (rc)
		goto exit;

exit:
	core_unlock(core, __func__);

	return rc;
}

int venus_hfi_set_ir_period(struct msm_vidc_inst *inst, u32 ir_type,
	enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_core *core;
	u32 ir_period, sync_frame_req = 0;

	core = inst->core;

	core_lock(core, __func__);

	ir_period = inst->capabilities[cap_id].value;

	rc = hfi_create_header(inst->packet, inst->packet_size,
			       inst->session_id, core->header_id++);
	if (rc)
		goto exit;

	/* Request sync frame if ir period enabled dynamically */
	if (!inst->ir_enabled) {
		inst->ir_enabled = ((ir_period > 0) ? true : false);
		if (inst->ir_enabled && inst->bufq[OUTPUT_PORT].vb2q->streaming) {
			sync_frame_req = HFI_SYNC_FRAME_REQUEST_WITH_PREFIX_SEQ_HDR;
			rc = hfi_create_packet(inst->packet, inst->packet_size,
					       HFI_PROP_REQUEST_SYNC_FRAME,
					       HFI_HOST_FLAGS_NONE,
					       HFI_PAYLOAD_U32_ENUM,
					       msm_vidc_get_port_info(inst, REQUEST_I_FRAME),
					       core->packet_id++,
					       &sync_frame_req,
					       sizeof(u32));
			if (rc)
				goto exit;
		}
	}

	rc = hfi_create_packet(inst->packet, inst->packet_size,
			       ir_type,
			       HFI_HOST_FLAGS_NONE,
			       HFI_PAYLOAD_U32,
			       msm_vidc_get_port_info(inst, cap_id),
			       core->packet_id++,
			       &ir_period,
			       sizeof(u32));
	if (rc)
		goto exit;

	rc = __cmdq_write(inst->core, inst->packet);
	if (rc) {
		i_vpr_e(inst, "%s: failed to set inst->capabilities[%d] %s to fw\n",
			__func__, cap_id, cap_name(cap_id));
		goto exit;
	}

exit:
	core_unlock(core, __func__);

	return rc;
}

struct device_region_info *venus_hfi_get_device_region_info(
	struct msm_vidc_core *core, enum msm_vidc_device_region region)
{
	struct device_region_info *dev_reg = NULL, *match = NULL;

	if (!region || region >= MSM_VIDC_DEVICE_REGION_MAX) {
		d_vpr_e("%s: invalid region %#x\n", __func__, region);
		return NULL;
	}

	venus_hfi_for_each_device_region(core, dev_reg) {
		if (dev_reg->region == region) {
			match = dev_reg;
			break;
		}
	}
	if (!match)
		d_vpr_e("%s: device region %d not found\n", __func__, region);

	return match;
}
