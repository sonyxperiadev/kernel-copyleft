// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/gunyah/gh_rm_drv.h>
#include <linux/gunyah/gh_dbl.h>
#include <soc/qcom/secure_buffer.h>

#include "hw_fence_drv_priv.h"
#include "hw_fence_drv_utils.h"
#include "hw_fence_drv_ipc.h"
#include "hw_fence_drv_debug.h"

static void _lock(uint64_t *wait)
{
#if defined(__aarch64__)
	__asm__(
		// Sequence to wait for lock to be free (i.e. zero)
		"PRFM PSTL1KEEP, [%x[i_lock]]\n\t"
		"1:\n\t"
		"LDAXR W5, [%x[i_lock]]\n\t"
		"CBNZ W5, 1b\n\t"
		// Sequence to set PVM BIT0
		"LDR W7, =0x1\n\t"              // Load BIT0 (0x1) into W7
		"STXR W5, W7, [%x[i_lock]]\n\t" // Atomic Store exclusive BIT0 (lock = 0x1)
		"CBNZ W5, 1b\n\t"               // If cannot set it, goto 1
		:
		: [i_lock] "r" (wait)
		: "memory");
#endif
}

static void _unlock(struct hw_fence_driver_data *drv_data, uint64_t *lock)
{
	uint64_t lock_val;

#if defined(__aarch64__)
	__asm__(
		// Sequence to clear PVM BIT0
		"2:\n\t"
		"LDAXR W5, [%x[i_out]]\n\t"             // Atomic Fetch Lock
		"AND W6, W5, #0xFFFFFFFFFFFFFFFE\n\t"   // AND to clear BIT0 (lock &= ~0x1))
		"STXR W5, W6, [%x[i_out]]\n\t"          // Store exclusive result
		"CBNZ W5, 2b\n\t"                       // If cannot store exclusive, goto 2
		:
		: [i_out] "r" (lock)
		: "memory");
#endif
	mb(); /* Make sure the memory is updated */

	lock_val = *lock; /* Read the lock value */
	HWFNC_DBG_LOCK("unlock: lock_val after:0x%llx\n", lock_val);
	if (lock_val & 0x2) { /* check if SVM BIT1 is set*/
		/*
		 * SVM is in WFI state, since SVM acquire bit is set
		 * Trigger IRQ to Wake-Up SVM Client
		 */
#if IS_ENABLED(CONFIG_DEBUG_FS)
		drv_data->debugfs_data.lock_wake_cnt++;
		HWFNC_DBG_LOCK("triggering ipc to unblock SVM lock_val:%d cnt:%llu\n", lock_val,
			drv_data->debugfs_data.lock_wake_cnt);
#endif
		hw_fence_ipcc_trigger_signal(drv_data,
			drv_data->ipcc_client_pid,
			drv_data->ipcc_client_vid, 30); /* Trigger APPS Signal 30 */
	}
}

void global_atomic_store(struct hw_fence_driver_data *drv_data, uint64_t *lock, bool val)
{
	if (val) {
		preempt_disable();
		_lock(lock);
	} else {
		_unlock(drv_data, lock);
		preempt_enable();
	}
}

/*
 * Each bit in this mask represents each of the loopback clients supported in
 * the enum hw_fence_loopback_id
 */
#define HW_FENCE_LOOPBACK_CLIENTS_MASK 0x7f

static inline int _process_dpu_client_loopback(struct hw_fence_driver_data *drv_data,
	int client_id)
{
	int ctl_id = client_id; /* dpu ctl path id is mapped to client id used for the loopback */
	void *ctl_start_reg;
	u32 val;

	if (ctl_id > HW_FENCE_LOOPBACK_DPU_CTL_5) {
		HWFNC_ERR("invalid ctl_id:%d\n", ctl_id);
		return -EINVAL;
	}

	ctl_start_reg = drv_data->ctl_start_ptr[ctl_id];
	if (!ctl_start_reg) {
		HWFNC_ERR("ctl_start reg not valid for ctl_id:%d\n", ctl_id);
		return -EINVAL;
	}

	HWFNC_DBG_H("Processing DPU loopback ctl_id:%d\n", ctl_id);

	val = 0x1; /* ctl_start trigger */
#ifdef CTL_START_SIM
	HWFNC_DBG_IRQ("ctl_id:%d Write: to RegOffset:0x%pK val:0x%x\n", ctl_start_reg, val, ctl_id);
	writel_relaxed(val, ctl_start_reg);
#else
	HWFNC_DBG_IRQ("ctl_id:%d Write: to RegOffset:0x%pK val:0x%x (COMMENTED)\n", ctl_id,
		ctl_start_reg, val);
#endif

	return 0;
}

static inline int _process_gfx_client_loopback(struct hw_fence_driver_data *drv_data,
	int client_id)
{
	int queue_type = HW_FENCE_RX_QUEUE - 1; /* rx queue index */
	struct msm_hw_fence_queue_payload payload;
	int read = 1;

	HWFNC_DBG_IRQ("Processing GFX loopback client_id:%d\n", client_id);
	while (read) {
		/*
		 * 'client_id' is the loopback-client-id, not the hw-fence client_id,
		 * so use GFX hw-fence client id, to get the client data
		 */
		read = hw_fence_read_queue(drv_data->clients[HW_FENCE_CLIENT_ID_CTX0], &payload,
			queue_type);
		if (read < 0) {
			HWFNC_ERR("unable to read gfx rxq\n");
			break;
		}
		HWFNC_DBG_L("GFX loopback rxq read: hash:%llu ctx:%llu seq:%llu f:%llu e:%lu\n",
			payload.hash, payload.ctxt_id, payload.seqno, payload.flags, payload.error);
	}

	return read;
}

static int _process_doorbell_client(struct hw_fence_driver_data *drv_data, int client_id)
{
	int ret;

	HWFNC_DBG_H("Processing loopback client_id:%d\n", client_id);
	switch (client_id) {
	case HW_FENCE_LOOPBACK_DPU_CTL_0:
	case HW_FENCE_LOOPBACK_DPU_CTL_1:
	case HW_FENCE_LOOPBACK_DPU_CTL_2:
	case HW_FENCE_LOOPBACK_DPU_CTL_3:
	case HW_FENCE_LOOPBACK_DPU_CTL_4:
	case HW_FENCE_LOOPBACK_DPU_CTL_5:
		ret = _process_dpu_client_loopback(drv_data, client_id);
		break;
	case HW_FENCE_LOOPBACK_GFX_CTX_0:
		ret = _process_gfx_client_loopback(drv_data, client_id);
		break;
#if IS_ENABLED(CONFIG_DEBUG_FS)
	case HW_FENCE_LOOPBACK_VAL_0:
	case HW_FENCE_LOOPBACK_VAL_1:
	case HW_FENCE_LOOPBACK_VAL_2:
	case HW_FENCE_LOOPBACK_VAL_3:
	case HW_FENCE_LOOPBACK_VAL_4:
	case HW_FENCE_LOOPBACK_VAL_5:
	case HW_FENCE_LOOPBACK_VAL_6:
		ret = process_validation_client_loopback(drv_data, client_id);
		break;
#endif /* CONFIG_DEBUG_FS */
	default:
		HWFNC_ERR("unknown client:%d\n", client_id);
		ret = -EINVAL;
	}

	return ret;
}

void hw_fence_utils_process_doorbell_mask(struct hw_fence_driver_data *drv_data, u64 db_flags)
{
	int client_id = HW_FENCE_LOOPBACK_DPU_CTL_0;
	u64 mask;

	for (; client_id < HW_FENCE_LOOPBACK_MAX; client_id++) {
		mask = 1 << client_id;
		if (mask & db_flags) {
			HWFNC_DBG_H("client_id:%d signaled! flags:0x%llx\n", client_id, db_flags);

			/* process client */
			if (_process_doorbell_client(drv_data, client_id))
				HWFNC_ERR("Failed to process client:%d\n", client_id);

			/* clear mask for this client and if nothing else pending finish */
			db_flags = db_flags & ~(mask);
			HWFNC_DBG_H("client_id:%d cleared flags:0x%llx mask:0x%llx ~mask:0x%llx\n",
				client_id, db_flags, mask, ~(mask));
			if (!db_flags)
				break;
		}
	}
}

/* doorbell callback */
static void _hw_fence_cb(int irq, void *data)
{
	struct hw_fence_driver_data *drv_data = (struct hw_fence_driver_data *)data;
	gh_dbl_flags_t clear_flags = HW_FENCE_LOOPBACK_CLIENTS_MASK;
	int ret;

	if (!drv_data)
		return;

	ret = gh_dbl_read_and_clean(drv_data->rx_dbl, &clear_flags, 0);
	if (ret) {
		HWFNC_ERR("hw_fence db callback, retrieve flags fail ret:%d\n", ret);
		return;
	}

	HWFNC_DBG_IRQ("db callback label:%d irq:%d flags:0x%llx qtime:%llu\n", drv_data->db_label,
		irq, clear_flags, hw_fence_get_qtime(drv_data));

	hw_fence_utils_process_doorbell_mask(drv_data, clear_flags);
}

int hw_fence_utils_init_virq(struct hw_fence_driver_data *drv_data)
{
	struct device_node *node = drv_data->dev->of_node;
	struct device_node *node_compat;
	const char *compat = "qcom,msm-hw-fence-db";
	int ret;

	node_compat = of_find_compatible_node(node, NULL, compat);
	if (!node_compat) {
		HWFNC_ERR("Failed to find dev node with compat:%s\n", compat);
		return -EINVAL;
	}

	ret = of_property_read_u32(node_compat, "gunyah-label", &drv_data->db_label);
	if (ret) {
		HWFNC_ERR("failed to find label info %d\n", ret);
		return ret;
	}

	HWFNC_DBG_IRQ("registering doorbell db_label:%d\n", drv_data->db_label);
	drv_data->rx_dbl = gh_dbl_rx_register(drv_data->db_label, _hw_fence_cb, drv_data);
	if (IS_ERR_OR_NULL(drv_data->rx_dbl)) {
		ret = PTR_ERR(drv_data->rx_dbl);
		HWFNC_ERR("Failed to register doorbell\n");
		return ret;
	}

	return 0;
}

static int hw_fence_gunyah_share_mem(struct hw_fence_driver_data *drv_data,
				gh_vmid_t self, gh_vmid_t peer)
{
	u32 src_vmlist[1] = {self};
	int src_perms[2] = {PERM_READ | PERM_WRITE | PERM_EXEC};
	int dst_vmlist[2] = {self, peer};
	int dst_perms[2] = {PERM_READ | PERM_WRITE, PERM_READ | PERM_WRITE};
	struct gh_acl_desc *acl;
	struct gh_sgl_desc *sgl;
	int ret;

	ret = hyp_assign_phys(drv_data->res.start, resource_size(&drv_data->res),
			src_vmlist, 1, dst_vmlist, dst_perms, 2);
	if (ret) {
		HWFNC_ERR("%s: hyp_assign_phys failed addr=%x size=%u err=%d\n",
			__func__, drv_data->res.start, drv_data->size, ret);
		return ret;
	}

	acl = kzalloc(offsetof(struct gh_acl_desc, acl_entries[2]), GFP_KERNEL);
	if (!acl)
		return -ENOMEM;
	sgl = kzalloc(offsetof(struct gh_sgl_desc, sgl_entries[1]), GFP_KERNEL);
	if (!sgl) {
		kfree(acl);
		return -ENOMEM;
	}
	acl->n_acl_entries = 2;
	acl->acl_entries[0].vmid = (u16)self;
	acl->acl_entries[0].perms = GH_RM_ACL_R | GH_RM_ACL_W;
	acl->acl_entries[1].vmid = (u16)peer;
	acl->acl_entries[1].perms = GH_RM_ACL_R | GH_RM_ACL_W;

	sgl->n_sgl_entries = 1;
	sgl->sgl_entries[0].ipa_base = drv_data->res.start;
	sgl->sgl_entries[0].size = resource_size(&drv_data->res);

	ret = gh_rm_mem_share(GH_RM_MEM_TYPE_NORMAL, 0, drv_data->label,
			acl, sgl, NULL, &drv_data->memparcel);
	if (ret) {
		HWFNC_ERR("%s: gh_rm_mem_share failed addr=%x size=%u err=%d\n",
			__func__, drv_data->res.start, drv_data->size, ret);
		/* Attempt to give resource back to HLOS */
		hyp_assign_phys(drv_data->res.start, resource_size(&drv_data->res),
				dst_vmlist, 2,
				src_vmlist, src_perms, 1);
		ret = -EPROBE_DEFER;
	}

	kfree(acl);
	kfree(sgl);

	return ret;
}

static int hw_fence_rm_cb(struct notifier_block *nb, unsigned long cmd, void *data)
{
	struct gh_rm_notif_vm_status_payload *vm_status_payload;
	struct hw_fence_driver_data *drv_data;
	gh_vmid_t peer_vmid;
	gh_vmid_t self_vmid;

	drv_data = container_of(nb, struct hw_fence_driver_data, rm_nb);

	HWFNC_DBG_INIT("cmd:0x%lx ++\n", cmd);
	if (cmd != GH_RM_NOTIF_VM_STATUS)
		goto end;

	vm_status_payload = data;
	HWFNC_DBG_INIT("payload vm_status:%d\n", vm_status_payload->vm_status);
	if (vm_status_payload->vm_status != GH_RM_VM_STATUS_READY &&
	    vm_status_payload->vm_status != GH_RM_VM_STATUS_RESET)
		goto end;

	if (gh_rm_get_vmid(drv_data->peer_name, &peer_vmid))
		goto end;

	if (gh_rm_get_vmid(GH_PRIMARY_VM, &self_vmid))
		goto end;

	if (peer_vmid != vm_status_payload->vmid)
		goto end;

	switch (vm_status_payload->vm_status) {
	case GH_RM_VM_STATUS_READY:
		HWFNC_DBG_INIT("init mem\n");
		if (hw_fence_gunyah_share_mem(drv_data, self_vmid, peer_vmid))
			HWFNC_ERR("failed to share memory\n");
		else
			drv_data->vm_ready = true;
		break;
	case GH_RM_VM_STATUS_RESET:
		HWFNC_DBG_INIT("reset\n");
		break;
	}

end:
	return NOTIFY_DONE;
}

/* Allocates carved-out mapped memory */
int hw_fence_utils_alloc_mem(struct hw_fence_driver_data *drv_data)
{
	struct device_node *node = drv_data->dev->of_node;
	struct device_node *node_compat;
	const char *compat = "qcom,msm-hw-fence-mem";
	struct device *dev = drv_data->dev;
	struct device_node *np;
	int notifier_ret, ret;

	node_compat = of_find_compatible_node(node, NULL, compat);
	if (!node_compat) {
		HWFNC_ERR("Failed to find dev node with compat:%s\n", compat);
		return -EINVAL;
	}

	ret = of_property_read_u32(node_compat, "gunyah-label", &drv_data->label);
	if (ret) {
		HWFNC_ERR("failed to find label info %d\n", ret);
		return ret;
	}

	np = of_parse_phandle(node_compat, "shared-buffer", 0);
	if (!np) {
		HWFNC_ERR("failed to read shared-buffer info\n");
		return -ENOMEM;
	}

	ret = of_address_to_resource(np, 0, &drv_data->res);
	of_node_put(np);
	if (ret) {
		HWFNC_ERR("of_address_to_resource failed %d\n", ret);
		return -EINVAL;
	}

	drv_data->io_mem_base = devm_ioremap(dev, drv_data->res.start,
		resource_size(&drv_data->res));
	if (!drv_data->io_mem_base) {
		HWFNC_ERR("ioremap failed!\n");
		return -ENXIO;
	}
	drv_data->size = resource_size(&drv_data->res);

	HWFNC_DBG_INIT("io_mem_base:0x%x start:0x%x end:0x%x size:0x%x name:%s\n",
		drv_data->io_mem_base, drv_data->res.start,
		drv_data->res.end, drv_data->size, drv_data->res.name);

	memset_io(drv_data->io_mem_base, 0x0, drv_data->size);

	/* Register memory with HYP */
	ret = of_property_read_u32(node_compat, "peer-name", &drv_data->peer_name);
	if (ret)
		drv_data->peer_name = GH_SELF_VM;

	drv_data->rm_nb.notifier_call = hw_fence_rm_cb;
	drv_data->rm_nb.priority = INT_MAX;
	notifier_ret = gh_rm_register_notifier(&drv_data->rm_nb);
	HWFNC_DBG_INIT("notifier: ret:%d peer_name:%d notifier_ret:%d\n", ret,
		drv_data->peer_name, notifier_ret);
	if (notifier_ret) {
		HWFNC_ERR("fail to register notifier ret:%d\n", notifier_ret);
		return -EPROBE_DEFER;
	}

	return 0;
}

char *_get_mem_reserve_type(enum hw_fence_mem_reserve type)
{
	switch (type) {
	case HW_FENCE_MEM_RESERVE_CTRL_QUEUE:
		return "HW_FENCE_MEM_RESERVE_CTRL_QUEUE";
	case HW_FENCE_MEM_RESERVE_LOCKS_REGION:
		return "HW_FENCE_MEM_RESERVE_LOCKS_REGION";
	case HW_FENCE_MEM_RESERVE_TABLE:
		return "HW_FENCE_MEM_RESERVE_TABLE";
	case HW_FENCE_MEM_RESERVE_CLIENT_QUEUE:
		return "HW_FENCE_MEM_RESERVE_CLIENT_QUEUE";
	}

	return "Unknown";
}

/* Calculates the memory range for each of the elements in the carved-out memory */
int hw_fence_utils_reserve_mem(struct hw_fence_driver_data *drv_data,
	enum hw_fence_mem_reserve type, phys_addr_t *phys, void **pa, u32 *size, int client_id)
{
	int ret = 0;
	u32 start_offset = 0;

	switch (type) {
	case HW_FENCE_MEM_RESERVE_CTRL_QUEUE:
		start_offset = 0;
		*size = drv_data->hw_fence_mem_ctrl_queues_size;
		break;
	case HW_FENCE_MEM_RESERVE_LOCKS_REGION:
		/* Locks region starts at the end of the ctrl queues */
		start_offset = drv_data->hw_fence_mem_ctrl_queues_size;
		*size = HW_FENCE_MEM_LOCKS_SIZE;
		break;
	case HW_FENCE_MEM_RESERVE_TABLE:
		/* HW Fence table starts at the end of the Locks region */
		start_offset = drv_data->hw_fence_mem_ctrl_queues_size + HW_FENCE_MEM_LOCKS_SIZE;
		*size = drv_data->hw_fence_mem_fences_table_size;
		break;
	case HW_FENCE_MEM_RESERVE_CLIENT_QUEUE:
		if (client_id >= HW_FENCE_CLIENT_MAX) {
			HWFNC_ERR("unexpected client_id:%d\n", client_id);
			ret = -EINVAL;
			goto exit;
		}

		start_offset = PAGE_ALIGN(drv_data->hw_fence_mem_ctrl_queues_size +
			HW_FENCE_MEM_LOCKS_SIZE +
			drv_data->hw_fence_mem_fences_table_size) +
			((client_id - 1) * drv_data->hw_fence_mem_clients_queues_size);
		*size = drv_data->hw_fence_mem_clients_queues_size;

		break;
	default:
		HWFNC_ERR("Invalid mem reserve type:%d\n", type);
		ret = -EINVAL;
		break;
	}

	if (start_offset + *size > drv_data->size) {
		HWFNC_ERR("reservation request:%lu exceeds total size:%d\n",
			start_offset + *size, drv_data->size);
		return -ENOMEM;
	}

	HWFNC_DBG_INIT("type:%s (%d) io_mem_base:0x%x start:0x%x start_offset:%lu size:0x%x\n",
		_get_mem_reserve_type(type), type, drv_data->io_mem_base, drv_data->res.start,
		start_offset, *size);


	*phys = drv_data->res.start + (phys_addr_t)start_offset;
	*pa = (drv_data->io_mem_base + start_offset); /* offset is in bytes */
	HWFNC_DBG_H("phys:0x%x pa:0x%pK\n", *phys, *pa);

exit:
	return ret;
}

int hw_fence_utils_parse_dt_props(struct hw_fence_driver_data *drv_data)
{
	int ret;
	u32 val = 0;

	ret = of_property_read_u32(drv_data->dev->of_node, "qcom,hw-fence-table-entries", &val);
	if (ret || !val) {
		HWFNC_ERR("missing hw fences table entry or invalid ret:%d val:%d\n", ret, val);
		return ret;
	}
	drv_data->hw_fence_table_entries = val;

	if (drv_data->hw_fence_table_entries >= U32_MAX / sizeof(struct msm_hw_fence)) {
		HWFNC_ERR("table entries:%lu will overflow table size\n",
			drv_data->hw_fence_table_entries);
		return -EINVAL;
	}
	drv_data->hw_fence_mem_fences_table_size = (sizeof(struct msm_hw_fence) *
		drv_data->hw_fence_table_entries);

	ret = of_property_read_u32(drv_data->dev->of_node, "qcom,hw-fence-queue-entries", &val);
	if (ret || !val) {
		HWFNC_ERR("missing queue entries table entry or invalid ret:%d val:%d\n", ret, val);
		return ret;
	}
	drv_data->hw_fence_queue_entries = val;

	/* ctrl queues init */

	if (drv_data->hw_fence_queue_entries >= U32_MAX / HW_FENCE_CTRL_QUEUE_PAYLOAD) {
		HWFNC_ERR("queue entries:%lu will overflow ctrl queue size\n",
			drv_data->hw_fence_queue_entries);
		return -EINVAL;
	}
	drv_data->hw_fence_ctrl_queue_size = HW_FENCE_CTRL_QUEUE_PAYLOAD *
		drv_data->hw_fence_queue_entries;

	if (drv_data->hw_fence_ctrl_queue_size >= (U32_MAX - HW_FENCE_HFI_CTRL_HEADERS_SIZE) /
			HW_FENCE_CTRL_QUEUES) {
		HWFNC_ERR("queue size:%lu will overflow ctrl queue mem size\n",
			drv_data->hw_fence_ctrl_queue_size);
		return -EINVAL;
	}
	drv_data->hw_fence_mem_ctrl_queues_size = HW_FENCE_HFI_CTRL_HEADERS_SIZE +
		(HW_FENCE_CTRL_QUEUES * drv_data->hw_fence_ctrl_queue_size);

	/* clients queues init */

	if (drv_data->hw_fence_queue_entries >= U32_MAX / HW_FENCE_CLIENT_QUEUE_PAYLOAD) {
		HWFNC_ERR("queue entries:%lu will overflow client queue size\n",
			drv_data->hw_fence_queue_entries);
		return -EINVAL;
	}
	drv_data->hw_fence_client_queue_size = HW_FENCE_CLIENT_QUEUE_PAYLOAD *
		drv_data->hw_fence_queue_entries;

	if (drv_data->hw_fence_client_queue_size >= ((U32_MAX & PAGE_MASK) -
			HW_FENCE_HFI_CLIENT_HEADERS_SIZE) / HW_FENCE_CLIENT_QUEUES) {
		HWFNC_ERR("queue size:%lu will overflow client queue mem size\n",
			drv_data->hw_fence_client_queue_size);
		return -EINVAL;
	}
	drv_data->hw_fence_mem_clients_queues_size = PAGE_ALIGN(HW_FENCE_HFI_CLIENT_HEADERS_SIZE +
		(HW_FENCE_CLIENT_QUEUES * drv_data->hw_fence_client_queue_size));

	HWFNC_DBG_INIT("table: entries=%lu mem_size=%lu queue: entries=%lu\b",
		drv_data->hw_fence_table_entries, drv_data->hw_fence_mem_fences_table_size,
		drv_data->hw_fence_queue_entries);
	HWFNC_DBG_INIT("ctrl queue: size=%lu mem_size=%lu clients queues: size=%lu mem_size=%lu\b",
		drv_data->hw_fence_ctrl_queue_size, drv_data->hw_fence_mem_ctrl_queues_size,
		drv_data->hw_fence_client_queue_size, drv_data->hw_fence_mem_clients_queues_size);

	return 0;
}

int hw_fence_utils_map_ipcc(struct hw_fence_driver_data *drv_data)
{
	int ret;
	u32 reg_config[2];
	void __iomem *ptr;

	/* Get ipcc memory range */
	ret = of_property_read_u32_array(drv_data->dev->of_node, "qcom,ipcc-reg",
				reg_config, 2);
	if (ret) {
		HWFNC_ERR("failed to read ipcc reg: %d\n", ret);
		return ret;
	}
	drv_data->ipcc_reg_base = reg_config[0];
	drv_data->ipcc_size = reg_config[1];

	/* Mmap ipcc registers */
	ptr = devm_ioremap(drv_data->dev, drv_data->ipcc_reg_base, drv_data->ipcc_size);
	if (!ptr) {
		HWFNC_ERR("failed to ioremap ipcc regs\n");
		return -ENOMEM;
	}
	drv_data->ipcc_io_mem = ptr;

	HWFNC_DBG_H("mapped address:0x%x size:0x%x io_mem:0x%pK\n",
		drv_data->ipcc_reg_base, drv_data->ipcc_size,
		drv_data->ipcc_io_mem);

	hw_fence_ipcc_enable_signaling(drv_data);

	return ret;
}

int hw_fence_utils_map_qtime(struct hw_fence_driver_data *drv_data)
{
	int ret = 0;
	unsigned int reg_config[2];
	void __iomem *ptr;

	ret = of_property_read_u32_array(drv_data->dev->of_node, "qcom,qtime-reg",
			reg_config, 2);
	if (ret) {
		HWFNC_ERR("failed to read qtimer reg: %d\n", ret);
		return ret;
	}

	drv_data->qtime_reg_base = reg_config[0];
	drv_data->qtime_size = reg_config[1];

	ptr = devm_ioremap(drv_data->dev, drv_data->qtime_reg_base, drv_data->qtime_size);
	if (!ptr) {
		HWFNC_ERR("failed to ioremap qtime regs\n");
		return -ENOMEM;
	}

	drv_data->qtime_io_mem = ptr;

	return ret;
}

static int _map_ctl_start(struct hw_fence_driver_data *drv_data, u32 ctl_id,
	void **iomem_ptr, uint32_t *iomem_size)
{
	u32 reg_config[2];
	void __iomem *ptr;
	char name[30] = {0};
	int ret;

	snprintf(name, sizeof(name), "qcom,dpu-ctl-start-%d-reg", ctl_id);
	ret = of_property_read_u32_array(drv_data->dev->of_node, name, reg_config, 2);
	if (ret)
		return 0; /* this is an optional property */

	/* Mmap registers */
	ptr = devm_ioremap(drv_data->dev, reg_config[0], reg_config[1]);
	if (!ptr) {
		HWFNC_ERR("failed to ioremap %s reg\n", name);
		return -ENOMEM;
	}

	*iomem_ptr = ptr;
	*iomem_size = reg_config[1];

	HWFNC_DBG_INIT("mapped ctl_start ctl_id:%d name:%s address:0x%x size:0x%x io_mem:0x%pK\n",
		ctl_id, name, reg_config[0], reg_config[1], ptr);

	return 0;
}

int hw_fence_utils_map_ctl_start(struct hw_fence_driver_data *drv_data)
{
	u32 ctl_id = HW_FENCE_LOOPBACK_DPU_CTL_0;

	for (; ctl_id <= HW_FENCE_LOOPBACK_DPU_CTL_5; ctl_id++) {
		if (_map_ctl_start(drv_data, ctl_id, &drv_data->ctl_start_ptr[ctl_id],
			&drv_data->ctl_start_size[ctl_id])) {
			HWFNC_ERR("cannot map ctl_start ctl_id:%d\n", ctl_id);
		} else {
			if (drv_data->ctl_start_ptr[ctl_id])
				HWFNC_DBG_INIT("mapped ctl_id:%d ctl_start_ptr:0x%pK size:%u\n",
					ctl_id, drv_data->ctl_start_ptr[ctl_id],
					drv_data->ctl_start_size[ctl_id]);
		}
	}

	return 0;
}
