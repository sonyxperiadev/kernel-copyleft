// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/gunyah/gh_rm_drv.h>
#include <linux/gunyah/gh_dbl.h>
#include <linux/qcom_scm.h>
#include <linux/version.h>
#if (KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE)
#include <linux/gh_cpusys_vm_mem_access.h>
#endif
#include <soc/qcom/secure_buffer.h>

#include "hw_fence_drv_priv.h"
#include "hw_fence_drv_utils.h"
#include "hw_fence_drv_ipc.h"
#include "hw_fence_drv_debug.h"

/**
 * MAX_CLIENT_QUEUE_MEM_SIZE:
 * Maximum memory size for client queues of a hw fence client.
 */
#define MAX_CLIENT_QUEUE_MEM_SIZE 0x100000

/**
 * HW_FENCE_MAX_CLIENT_TYPE:
 * Total number of client types with and without configurable number of sub-clients
 */
#define HW_FENCE_MAX_CLIENT_TYPE (HW_FENCE_MAX_CLIENT_TYPE_STATIC + \
	HW_FENCE_MAX_CLIENT_TYPE_CONFIGURABLE)

/**
 * HW_FENCE_MIN_RXQ_CLIENTS:
 * Minimum number of static hw fence clients with rxq
 */
#define HW_FENCE_MIN_RXQ_CLIENTS HW_FENCE_CLIENT_ID_VAL6

/**
 * HW_FENCE_MIN_RXQ_CLIENT_TYPE:
 * Minimum number of static hw fence client types with rxq (GFX, DPU, VAL)
 */
#define HW_FENCE_MIN_RXQ_CLIENT_TYPE 3

/* Maximum number of clients for each client type */
#define HW_FENCE_CLIENT_TYPE_MAX_GPU 1
#define HW_FENCE_CLIENT_TYPE_MAX_DPU 6
#define HW_FENCE_CLIENT_TYPE_MAX_VAL 7
#define HW_FENCE_CLIENT_TYPE_MAX_IPE 32
#define HW_FENCE_CLIENT_TYPE_MAX_VPU 32
#define HW_FENCE_CLIENT_TYPE_MAX_IFE 32

/**
 * HW_FENCE_CTRL_QUEUE_DOORBELL:
 * Bit set in doorbell flags mask if hw fence driver should read ctrl rx queue
 */
#define HW_FENCE_CTRL_QUEUE_DOORBELL 0

/**
 * HW_FENCE_DOORBELL_FLAGS_ID_LAST:
 * Last doorbell flags id for which HW Fence Driver can receive doorbell
 */
#if IS_ENABLED(CONFIG_DEBUG_FS)
#define HW_FENCE_DOORBELL_FLAGS_ID_LAST HW_FENCE_CLIENT_ID_VAL6
#else
#define HW_FENCE_DOORBELL_FLAGS_ID_LAST HW_FENCE_CTRL_QUEUE_DOORBELL
#endif /* CONFIG_DEBUG_FS */

/**
 * HW_FENCE_DOORBELL_MASK:
 * Each bit in this mask represents possible doorbell flag ids for which hw fence driver can receive
 */
#define HW_FENCE_DOORBELL_MASK \
	GENMASK(HW_FENCE_DOORBELL_FLAGS_ID_LAST, HW_FENCE_CTRL_QUEUE_DOORBELL)

/**
 * HW_FENCE_MAX_ITER_READ:
 * Maximum number of iterations when reading queue
 */
#define HW_FENCE_MAX_ITER_READ 100

/**
 * HW_FENCE_MAX_EVENTS:
 * Maximum number of HW Fence debug events
 */
#define HW_FENCE_MAX_EVENTS 1000

/**
 * struct hw_fence_client_types - Table describing all supported client types, used to parse
 *                                device-tree properties related to client queue size.
 *
 * The fields name, init_id, and max_clients_num are constants. Default values for clients_num,
 * queues_num, and skip_txq_wr_idx are provided in this table, and clients_num, queues_num,
 * queue_entries, and skip_txq_wr_idx can be read from device-tree.
 *
 * If a value for queue entries is not parsed for the client type, then the default number of client
 * queue entries (parsed from device-tree) is used.
 *
 * Notes:
 * 1. Client types must be in the same order as client_ids within the enum 'hw_fence_client_id'.
 * 2. Each HW Fence client ID must be described by one of the client types in this table.
 * 3. A new client type must set: name, init_id, max_clients_num, clients_num, queues_num, and
 *    skip_txq_wr_idx.
 * 4. Either constant HW_FENCE_MAX_CLIENT_TYPE_CONFIGURABLE or HW_FENCE_MAX_CLIENT_TYPE_STATIC must
 *    be incremented as appropriate for new client types.
 */
struct hw_fence_client_type_desc hw_fence_client_types[HW_FENCE_MAX_CLIENT_TYPE] = {
	{"gpu", HW_FENCE_CLIENT_ID_CTX0, HW_FENCE_CLIENT_TYPE_MAX_GPU, HW_FENCE_CLIENT_TYPE_MAX_GPU,
		HW_FENCE_CLIENT_QUEUES, 0, 0, 0, 0, 0, 0, false},
	{"dpu", HW_FENCE_CLIENT_ID_CTL0, HW_FENCE_CLIENT_TYPE_MAX_DPU, HW_FENCE_CLIENT_TYPE_MAX_DPU,
		HW_FENCE_CLIENT_QUEUES, 0, 0, 0, 0, 0, 0, false},
	{"val", HW_FENCE_CLIENT_ID_VAL0, HW_FENCE_CLIENT_TYPE_MAX_VAL, HW_FENCE_CLIENT_TYPE_MAX_VAL,
		HW_FENCE_CLIENT_QUEUES, 0, 0, 0, 0, 0, 0, false},
	{"ipe", HW_FENCE_CLIENT_ID_IPE, HW_FENCE_CLIENT_TYPE_MAX_IPE, 0, HW_FENCE_CLIENT_QUEUES,
		0, 0, 0, 0, 0, 0, false},
	{"vpu", HW_FENCE_CLIENT_ID_VPU, HW_FENCE_CLIENT_TYPE_MAX_VPU, 0, HW_FENCE_CLIENT_QUEUES,
		0, 0, 0, 0, 0, 0, false},
	{"ife0", HW_FENCE_CLIENT_ID_IFE0, HW_FENCE_CLIENT_TYPE_MAX_IFE, 0, 1, 0, 0, 0, 0, 0, 0,
		true},
	{"ife1", HW_FENCE_CLIENT_ID_IFE1, HW_FENCE_CLIENT_TYPE_MAX_IFE, 0, 1, 0, 0, 0, 0, 0, 0,
		true},
	{"ife2", HW_FENCE_CLIENT_ID_IFE2, HW_FENCE_CLIENT_TYPE_MAX_IFE, 0, 1, 0, 0, 0, 0, 0, 0,
		true},
	{"ife3", HW_FENCE_CLIENT_ID_IFE3, HW_FENCE_CLIENT_TYPE_MAX_IFE, 0, 1, 0, 0, 0, 0, 0, 0,
		true},
	{"ife4", HW_FENCE_CLIENT_ID_IFE4, HW_FENCE_CLIENT_TYPE_MAX_IFE, 0, 1, 0, 0, 0, 0, 0, 0,
		true},
	{"ife5", HW_FENCE_CLIENT_ID_IFE5, HW_FENCE_CLIENT_TYPE_MAX_IFE, 0, 1, 0, 0, 0, 0, 0, 0,
		true},
	{"ife6", HW_FENCE_CLIENT_ID_IFE6, HW_FENCE_CLIENT_TYPE_MAX_IFE, 0, 1, 0, 0, 0, 0, 0, 0,
		true},
	{"ife7", HW_FENCE_CLIENT_ID_IFE7, HW_FENCE_CLIENT_TYPE_MAX_IFE, 0, 1, 0, 0, 0, 0, 0, 0,
		true},
};

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

int hw_fence_utils_fence_error_cb(struct msm_hw_fence_client *hw_fence_client, u64 ctxt_id,
	u64 seqno, u64 hash, u64 flags, u32 error)
{
	struct msm_hw_fence_cb_data cb_data;
	struct dma_fence fence;
	int ret = 0;

	if (IS_ERR_OR_NULL(hw_fence_client)) {
		HWFNC_ERR("Invalid client:0x%pK\n", hw_fence_client);
		return -EINVAL;
	}

	mutex_lock(&hw_fence_client->error_cb_lock);
	if (!error || !hw_fence_client->fence_error_cb) {
		HWFNC_ERR("Invalid error:%d fence_error_cb:0x%pK\n", error,
			hw_fence_client->fence_error_cb);
		ret = -EINVAL;
		goto exit;
	}

	/* initialize cb_data info */
	fence.context = ctxt_id;
	fence.seqno = seqno;
	fence.flags = flags;
	fence.error = error;
	cb_data.fence = &fence;
	cb_data.data = hw_fence_client->fence_error_cb_userdata;

	HWFNC_DBG_L("invoking cb for client:%d ctx:%llu seq:%llu flags:%llu e:%u data:0x%pK\n",
		hw_fence_client->client_id, ctxt_id, seqno, flags, error,
		hw_fence_client->fence_error_cb_userdata);

	hw_fence_client->fence_error_cb(hash, error, &cb_data);

exit:
	mutex_unlock(&hw_fence_client->error_cb_lock);

	return ret;
}

static int _process_fence_error_client_loopback(struct hw_fence_driver_data *drv_data,
	int db_flag_id)
{
	struct msm_hw_fence_client *hw_fence_client;
	struct msm_hw_fence_queue_payload payload;
	int i, cb_ret, ret = 0, read = 1;
	u32 client_id;

	for (i = 0; read && i < HW_FENCE_MAX_ITER_READ; i++) {
		read = hw_fence_read_queue_helper(&drv_data->ctrl_queues[HW_FENCE_RX_QUEUE - 1],
			&payload);
		if (read < 0) {
			HWFNC_DBG_Q("unable to read ctrl rxq for db_flag_id:%d\n", db_flag_id);
			return read;
		}
		if (payload.type != HW_FENCE_PAYLOAD_TYPE_2) {
			HWFNC_ERR("unsupported payload type in ctrl rxq received:%u expected:%u\n",
				payload.type, HW_FENCE_PAYLOAD_TYPE_2);
			ret = -EINVAL;
			continue;
		}
		if (payload.client_data < HW_FENCE_CLIENT_ID_CTX0 ||
				payload.client_data >= drv_data->clients_num) {
			HWFNC_ERR("read invalid client_id:%llu from ctrl rxq min:%u max:%u\n",
				payload.client_data, HW_FENCE_CLIENT_ID_CTX0,
				drv_data->clients_num);
			ret = -EINVAL;
			continue;
		}

		client_id = payload.client_data;
		HWFNC_DBG_Q("ctrl rxq rd: it:%d h:%llu ctx:%llu seq:%llu f:%llu e:%u client:%u\n",
			i, payload.hash, payload.ctxt_id, payload.seqno, payload.flags,
			payload.error, client_id);

		hw_fence_client = drv_data->clients[client_id];
		if (!hw_fence_client) {
			HWFNC_ERR("processing fence error cb for unregistered client_id:%u\n",
				client_id);
			ret = -EINVAL;
			continue;
		}

		cb_ret = hw_fence_utils_fence_error_cb(hw_fence_client, payload.ctxt_id,
			payload.seqno, payload.hash, payload.flags, payload.error);
		if (cb_ret) {
			HWFNC_ERR("fence_error_cb failed for client:%u ctx:%llu seq:%llu err:%u\n",
				client_id, payload.ctxt_id, payload.seqno, payload.error);
			ret = cb_ret;
		}
	}

	return ret;
}

static int _process_doorbell_id(struct hw_fence_driver_data *drv_data, int db_flag_id)
{
	int ret;

	HWFNC_DBG_H("Processing doorbell mask id:%d\n", db_flag_id);
	switch (db_flag_id) {
	case HW_FENCE_CTRL_QUEUE_DOORBELL:
		ret = _process_fence_error_client_loopback(drv_data, db_flag_id);
		break;
#if IS_ENABLED(CONFIG_DEBUG_FS)
	case HW_FENCE_CLIENT_ID_VAL0:
	case HW_FENCE_CLIENT_ID_VAL1:
	case HW_FENCE_CLIENT_ID_VAL2:
	case HW_FENCE_CLIENT_ID_VAL3:
	case HW_FENCE_CLIENT_ID_VAL4:
	case HW_FENCE_CLIENT_ID_VAL5:
	case HW_FENCE_CLIENT_ID_VAL6:
		ret = process_validation_client_loopback(drv_data, db_flag_id);
		break;
#endif /* CONFIG_DEBUG_FS */
	default:
		HWFNC_ERR("unknown mask id:%d\n", db_flag_id);
		ret = -EINVAL;
	}

	return ret;
}

void hw_fence_utils_process_doorbell_mask(struct hw_fence_driver_data *drv_data, u64 db_flags)
{
	int db_flag_id = HW_FENCE_CTRL_QUEUE_DOORBELL;
	u64 mask;

	for (; db_flag_id <= HW_FENCE_DOORBELL_FLAGS_ID_LAST; db_flag_id++) {
		mask = 1 << db_flag_id;
		if (mask & db_flags) {
			HWFNC_DBG_H("db_flag:%d signaled! flags:0x%llx\n", db_flag_id, db_flags);

			if (_process_doorbell_id(drv_data, db_flag_id))
				HWFNC_ERR("Failed to process db_flag_id:%d\n", db_flag_id);

			/* clear mask for this flag id if nothing else pending finish */
			db_flags = db_flags & ~(mask);
			HWFNC_DBG_H("db_flag_id:%d cleared flags:0x%llx mask:0x%llx ~mask:0x%llx\n",
				db_flag_id, db_flags, mask, ~(mask));
			if (!db_flags)
				break;
		}
	}
}

/* doorbell callback */
static void _hw_fence_cb(int irq, void *data)
{
	struct hw_fence_driver_data *drv_data = (struct hw_fence_driver_data *)data;
	gh_dbl_flags_t clear_flags = HW_FENCE_DOORBELL_MASK;
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
	struct qcom_scm_vmperm src_vmlist[] = {{self, PERM_READ | PERM_WRITE | PERM_EXEC}};
	struct qcom_scm_vmperm dst_vmlist[] = {{self, PERM_READ | PERM_WRITE},
					       {peer, PERM_READ | PERM_WRITE}};
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
	u64 srcvmids, dstvmids;
#else
	unsigned int srcvmids, dstvmids;
#endif
	struct gh_acl_desc *acl;
	struct gh_sgl_desc *sgl;
	int ret;

	srcvmids = BIT(src_vmlist[0].vmid);
	dstvmids = BIT(dst_vmlist[0].vmid) | BIT(dst_vmlist[1].vmid);
	ret = qcom_scm_assign_mem(drv_data->res.start, resource_size(&drv_data->res), &srcvmids,
			dst_vmlist, ARRAY_SIZE(dst_vmlist));
	if (ret) {
		HWFNC_ERR("%s: qcom_scm_assign_mem failed addr=%x size=%u err=%d\n",
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

#if (KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE)
	ret = ghd_rm_mem_share(GH_RM_MEM_TYPE_NORMAL, 0, drv_data->label,
			acl, sgl, NULL, &drv_data->memparcel);
#else
	ret = gh_rm_mem_share(GH_RM_MEM_TYPE_NORMAL, 0, drv_data->label,
			acl, sgl, NULL, &drv_data->memparcel);
#endif
	if (ret) {
		HWFNC_ERR("%s: gh_rm_mem_share failed addr=%x size=%u err=%d\n",
			__func__, drv_data->res.start, drv_data->size, ret);
		/* Attempt to give resource back to HLOS */
		qcom_scm_assign_mem(drv_data->res.start, resource_size(&drv_data->res),
				&dstvmids, src_vmlist, ARRAY_SIZE(src_vmlist));
		ret = -EPROBE_DEFER;
	}

	kfree(acl);
	kfree(sgl);

	return ret;
}

static int _is_mem_shared(struct resource *res)
{
#if (KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE)
	return gh_cpusys_vm_get_share_mem_info(res);
#else
	return -EINVAL;
#endif
}

static int hw_fence_rm_cb(struct notifier_block *nb, unsigned long cmd, void *data)
{
	struct gh_rm_notif_vm_status_payload *vm_status_payload;
	struct hw_fence_driver_data *drv_data;
	struct resource res;
	gh_vmid_t peer_vmid;
	gh_vmid_t self_vmid;
	int ret;

	drv_data = container_of(nb, struct hw_fence_driver_data, rm_nb);

	HWFNC_DBG_INIT("cmd:0x%lx ++\n", cmd);
	if (cmd != GH_RM_NOTIF_VM_STATUS)
		goto end;

	vm_status_payload = data;
	HWFNC_DBG_INIT("payload vm_status:%d\n", vm_status_payload->vm_status);
	if (vm_status_payload->vm_status != GH_RM_VM_STATUS_READY &&
	    vm_status_payload->vm_status != GH_RM_VM_STATUS_RESET)
		goto end;

#if (KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE)
	if (ghd_rm_get_vmid(drv_data->peer_name, &peer_vmid))
		goto end;

	if (ghd_rm_get_vmid(GH_PRIMARY_VM, &self_vmid))
		goto end;
#else
	if (gh_rm_get_vmid(drv_data->peer_name, &peer_vmid))
		goto end;

	if (gh_rm_get_vmid(GH_PRIMARY_VM, &self_vmid))
		goto end;
#endif

	if (peer_vmid != vm_status_payload->vmid)
		goto end;

	switch (vm_status_payload->vm_status) {
	case GH_RM_VM_STATUS_READY:
		ret = _is_mem_shared(&res);
		if (ret) {
			HWFNC_DBG_INIT("mem not shared ret:%d, attempt share\n", ret);
			if (hw_fence_gunyah_share_mem(drv_data, self_vmid, peer_vmid))
				HWFNC_ERR("failed to share memory\n");
			else
				drv_data->vm_ready = true;
		} else {
			if (drv_data->res.start == res.start &&
					resource_size(&drv_data->res) == resource_size(&res)) {
				drv_data->vm_ready = true;
				HWFNC_DBG_INIT("mem_ready: add:0x%x size:%d ret:%d\n", res.start,
					resource_size(&res), ret);
			} else {
				HWFNC_ERR("mem-shared mismatch:[0x%x,%d] expected:[0x%x,%d]\n",
					res.start, resource_size(&res), drv_data->res.start,
					resource_size(&drv_data->res));
			}
		}
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

	drv_data->io_mem_base = devm_ioremap_wc(dev, drv_data->res.start,
		resource_size(&drv_data->res));
	if (!drv_data->io_mem_base) {
		HWFNC_ERR("ioremap failed!\n");
		return -ENXIO;
	}
	drv_data->size = resource_size(&drv_data->res);
	if (drv_data->size < drv_data->used_mem_size) {
		HWFNC_ERR("0x%x size of carved-out memory region is less than required size:0x%x\n",
			drv_data->size, drv_data->used_mem_size);
		return -ENOMEM;
	}

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
	case HW_FENCE_MEM_RESERVE_EVENTS_BUFF:
		return "HW_FENCE_MEM_RESERVE_EVENTS_BUFF";
	}

	return "Unknown";
}

/* Calculates the memory range for each of the elements in the carved-out memory */
int hw_fence_utils_reserve_mem(struct hw_fence_driver_data *drv_data,
	enum hw_fence_mem_reserve type, phys_addr_t *phys, void **pa, u32 *size, int client_id)
{
	int ret = 0;
	u32 start_offset = 0;
	u32 remaining_size_bytes;
	u32 total_events;

	switch (type) {
	case HW_FENCE_MEM_RESERVE_CTRL_QUEUE:
		start_offset = 0;
		*size = drv_data->hw_fence_mem_ctrl_queues_size;
		break;
	case HW_FENCE_MEM_RESERVE_LOCKS_REGION:
		/* Locks region starts at the end of the ctrl queues */
		start_offset = drv_data->hw_fence_mem_ctrl_queues_size;
		*size = HW_FENCE_MEM_LOCKS_SIZE(drv_data->rxq_clients_num);
		break;
	case HW_FENCE_MEM_RESERVE_TABLE:
		/* HW Fence table starts at the end of the Locks region */
		start_offset = drv_data->hw_fence_mem_ctrl_queues_size +
			HW_FENCE_MEM_LOCKS_SIZE(drv_data->rxq_clients_num);
		*size = drv_data->hw_fence_mem_fences_table_size;
		break;
	case HW_FENCE_MEM_RESERVE_CLIENT_QUEUE:
		if (client_id >= drv_data->clients_num ||
				!drv_data->hw_fence_client_queue_size[client_id].type) {
			HWFNC_ERR("unexpected client_id:%d for clients_num:%lu\n", client_id,
				drv_data->clients_num);
			ret = -EINVAL;
			goto exit;
		}

		start_offset = drv_data->hw_fence_client_queue_size[client_id].start_offset;
		*size = drv_data->hw_fence_client_queue_size[client_id].type->mem_size;
		break;
	case HW_FENCE_MEM_RESERVE_EVENTS_BUFF:
		start_offset = drv_data->used_mem_size;
		remaining_size_bytes = drv_data->size - start_offset;
		if (start_offset >= drv_data->size ||
				remaining_size_bytes < sizeof(struct msm_hw_fence_event)) {
			HWFNC_DBG_INFO("no space for events total_sz:%lu offset:%lu evt_sz:%lu\n",
				drv_data->size, start_offset, sizeof(struct msm_hw_fence_event));
			ret = -ENOMEM;
			goto exit;
		}

		total_events = remaining_size_bytes / sizeof(struct msm_hw_fence_event);
		if (total_events > HW_FENCE_MAX_EVENTS)
			total_events = HW_FENCE_MAX_EVENTS;
		*size = total_events * sizeof(struct msm_hw_fence_event);
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

static int _parse_client_queue_dt_props_extra(struct hw_fence_driver_data *drv_data,
	struct hw_fence_client_type_desc *desc)
{
	u32 max_idx_from_zero, payload_size_u32 = HW_FENCE_CLIENT_QUEUE_PAYLOAD / sizeof(u32);
	char name[40];
	u32 tmp[4];
	bool idx_by_payload = false;
	int count, ret;

	snprintf(name, sizeof(name), "qcom,hw-fence-client-type-%s-extra", desc->name);

	/* check if property is present */
	ret = of_property_read_bool(drv_data->dev->of_node, name);
	if (!ret)
		return 0;

	count = of_property_count_u32_elems(drv_data->dev->of_node, name);
	if (count <= 0 || count > 4) {
		HWFNC_ERR("invalid %s extra dt props count:%d\n", desc->name, count);
		return -EINVAL;
	}

	ret = of_property_read_u32_array(drv_data->dev->of_node, name, tmp, count);
	if (ret) {
		HWFNC_ERR("Failed to read %s extra dt properties ret=%d count=%d\n", desc->name,
			ret, count);
		ret = -EINVAL;
		goto exit;
	}

	desc->start_padding = tmp[0];
	if (count >= 2)
		desc->end_padding = tmp[1];
	if (count >= 3)
		desc->txq_idx_start = tmp[2];
	if (count >= 4) {
		if (tmp[3] > 1) {
			HWFNC_ERR("%s invalid txq_idx_by_payload prop:%lu\n", desc->name, tmp[3]);
			ret = -EINVAL;
			goto exit;
		}
		idx_by_payload = tmp[3];
		desc->txq_idx_factor = idx_by_payload ? payload_size_u32 : 1;
	}

	if (desc->start_padding % sizeof(u32) || desc->end_padding % sizeof(u32) ||
			(desc->start_padding + desc->end_padding) % sizeof(u64)) {
		HWFNC_ERR("%s start_padding:%lu end_padding:%lu violates mem alignment\n",
			desc->name, desc->start_padding, desc->end_padding);
		ret = -EINVAL;
		goto exit;
	}

	if (desc->start_padding >= U32_MAX - HW_FENCE_HFI_CLIENT_HEADERS_SIZE(desc->queues_num)) {
		HWFNC_ERR("%s client queues_num:%lu start_padding:%lu will overflow mem_size\n",
			desc->name, desc->queues_num, desc->start_padding);
		ret = -EINVAL;
		goto exit;
	}

	if (desc->end_padding >= U32_MAX - HW_FENCE_HFI_CLIENT_HEADERS_SIZE(desc->queues_num) -
			desc->start_padding) {
		HWFNC_ERR("%s client q_num:%lu start_p:%lu end_p:%lu will overflow mem_size\n",
			desc->name, desc->queues_num, desc->start_padding, desc->end_padding);
		ret = -EINVAL;
		goto exit;
	}

	max_idx_from_zero = idx_by_payload ? desc->queue_entries :
		desc->queue_entries * payload_size_u32;
	if (desc->txq_idx_start >= U32_MAX - max_idx_from_zero) {
		HWFNC_ERR("%s txq_idx start:%lu by_payload:%s q_entries:%d will overflow txq_idx\n",
			desc->name, desc->txq_idx_start, idx_by_payload ? "true" : "false",
			desc->queue_entries);
		ret = -EINVAL;
		goto exit;
	}

	HWFNC_DBG_INIT("%s: start_p=%lu end_p=%lu txq_idx_start:%lu txq_idx_by_payload:%s\n",
		desc->name, desc->start_padding, desc->end_padding, desc->txq_idx_start,
		idx_by_payload ? "true" : "false");

exit:
	return ret;
}

static int _parse_client_queue_dt_props_indv(struct hw_fence_driver_data *drv_data,
	struct hw_fence_client_type_desc *desc)
{
	char name[31];
	u32 tmp[4];
	u32 queue_size;
	int ret;

	/* parse client queue properties from device-tree */
	snprintf(name, sizeof(name), "qcom,hw-fence-client-type-%s", desc->name);
	ret = of_property_read_u32_array(drv_data->dev->of_node, name, tmp, 4);
	if (ret) {
		HWFNC_DBG_INIT("missing %s client queue entry or invalid ret:%d\n", desc->name,
			ret);
		desc->queue_entries = drv_data->hw_fence_queue_entries;
	} else {
		desc->clients_num = tmp[0];
		desc->queues_num = tmp[1];
		desc->queue_entries = tmp[2];

		if (tmp[3] > 1) {
			HWFNC_ERR("%s invalid skip_txq_wr_idx prop:%lu\n", desc->name, tmp[3]);
			return -EINVAL;
		}
		desc->skip_txq_wr_idx = tmp[3];
	}

	if (desc->clients_num > desc->max_clients_num || !desc->queues_num ||
			desc->queues_num > HW_FENCE_CLIENT_QUEUES || !desc->queue_entries) {
		HWFNC_ERR("%s invalid dt: clients_num:%lu queues_num:%lu, queue_entries:%lu\n",
			desc->name, desc->clients_num, desc->queues_num, desc->queue_entries);
		return -EINVAL;
	}

	/* parse extra client queue properties from device-tree */
	ret = _parse_client_queue_dt_props_extra(drv_data, desc);
	if (ret) {
		HWFNC_ERR("%s failed to parse extra dt props\n", desc->name);
		return -EINVAL;
	}

	/* compute mem_size */
	if (desc->queue_entries >= U32_MAX / HW_FENCE_CLIENT_QUEUE_PAYLOAD) {
		HWFNC_ERR("%s client queue entries:%lu will overflow client queue size\n",
			desc->name, desc->queue_entries);
		return -EINVAL;
	}

	queue_size = HW_FENCE_CLIENT_QUEUE_PAYLOAD * desc->queue_entries;
	if (queue_size >= ((U32_MAX & PAGE_MASK) -
			(HW_FENCE_HFI_CLIENT_HEADERS_SIZE(desc->queues_num) +
			desc->start_padding + desc->end_padding)) / desc->queues_num) {
		HWFNC_ERR("%s client queue_sz:%lu start_p:%lu end_p:%lu will overflow mem size\n",
			desc->name, queue_size, desc->start_padding, desc->end_padding);
		return -EINVAL;
	}

	desc->mem_size = PAGE_ALIGN(HW_FENCE_HFI_CLIENT_HEADERS_SIZE(desc->queues_num) +
		(queue_size * desc->queues_num) + desc->start_padding + desc->end_padding);

	if (desc->mem_size > MAX_CLIENT_QUEUE_MEM_SIZE) {
		HWFNC_ERR("%s client queue mem_size:%lu greater than max mem size:%lu\n",
			desc->name, desc->mem_size, MAX_CLIENT_QUEUE_MEM_SIZE);
		return -EINVAL;
	}

	HWFNC_DBG_INIT("%s: clients=%lu q_num=%lu q_entries=%lu mem_sz=%lu skips_wr_ptr:%s\n",
		desc->name, desc->clients_num, desc->queues_num, desc->queue_entries,
		desc->mem_size, desc->skip_txq_wr_idx ? "true" : "false");

	return 0;
}

static int _parse_client_queue_dt_props(struct hw_fence_driver_data *drv_data)
{
	struct hw_fence_client_type_desc *desc;
	int i, j, ret;
	u32 start_offset;
	size_t size;
	int configurable_clients_num = 0;

	drv_data->rxq_clients_num = HW_FENCE_MIN_RXQ_CLIENTS;
	for (i = 0; i < HW_FENCE_MAX_CLIENT_TYPE; i++) {
		desc = &hw_fence_client_types[i];
		ret = _parse_client_queue_dt_props_indv(drv_data, desc);
		if (ret) {
			HWFNC_ERR("failed to initialize %s client queue size properties\n",
				desc->name);
			return ret;
		}

		if (i >= HW_FENCE_MIN_RXQ_CLIENT_TYPE &&
				desc->queues_num == HW_FENCE_CLIENT_QUEUES)
			drv_data->rxq_clients_num += desc->clients_num;

		if (i >= HW_FENCE_MAX_CLIENT_TYPE_STATIC)
			configurable_clients_num += desc->clients_num;
	}

	/* store client type descriptors for configurable client indexing logic */
	drv_data->hw_fence_client_types = hw_fence_client_types;

	/* clients and size desc are allocated for all static clients regardless of device-tree */
	drv_data->clients_num = HW_FENCE_MAX_STATIC_CLIENTS_INDEX + configurable_clients_num;

	/* allocate memory for client queue size descriptors */
	size = drv_data->clients_num * sizeof(struct hw_fence_client_queue_desc);
	drv_data->hw_fence_client_queue_size = kzalloc(size, GFP_KERNEL);
	if (!drv_data->hw_fence_client_queue_size)
		return -ENOMEM;

	/* initialize client queue size desc for each client */
	start_offset = PAGE_ALIGN(drv_data->hw_fence_mem_ctrl_queues_size +
		HW_FENCE_MEM_LOCKS_SIZE(drv_data->rxq_clients_num) +
		drv_data->hw_fence_mem_fences_table_size);
	for (i = 0; i < HW_FENCE_MAX_CLIENT_TYPE; i++) {
		desc = &hw_fence_client_types[i];
		for (j = 0; j < desc->clients_num; j++) {
			enum hw_fence_client_id client_id_ext = desc->init_id + j;
			enum hw_fence_client_id client_id =
				hw_fence_utils_get_client_id_priv(drv_data, client_id_ext);

			drv_data->hw_fence_client_queue_size[client_id] =
				(struct hw_fence_client_queue_desc){desc, start_offset};
			HWFNC_DBG_INIT("%s client_id_ext:%lu client_id:%lu start_offset:%lu\n",
				desc->name, client_id_ext, client_id, start_offset);
			start_offset += desc->mem_size;
		}
	}
	drv_data->used_mem_size = start_offset;

	return 0;
}

int hw_fence_utils_parse_dt_props(struct hw_fence_driver_data *drv_data)
{
	int ret;
	size_t size;
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

	ret = _parse_client_queue_dt_props(drv_data);
	if (ret) {
		HWFNC_ERR("failed to parse client queue properties\n");
		return -EINVAL;
	}

	/* allocate clients */

	size = drv_data->clients_num * sizeof(struct msm_hw_fence_client *);
	drv_data->clients = kzalloc(size, GFP_KERNEL);
	if (!drv_data->clients)
		return -ENOMEM;

	HWFNC_DBG_INIT("table: entries=%lu mem_size=%lu queue: entries=%lu\b",
		drv_data->hw_fence_table_entries, drv_data->hw_fence_mem_fences_table_size,
		drv_data->hw_fence_queue_entries);
	HWFNC_DBG_INIT("ctrl queue: size=%lu mem_size=%lu\b",
		drv_data->hw_fence_ctrl_queue_size, drv_data->hw_fence_mem_ctrl_queues_size);
	HWFNC_DBG_INIT("clients_num: %lu, total_mem_size:%lu\n", drv_data->clients_num,
		drv_data->used_mem_size);

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

enum hw_fence_client_id hw_fence_utils_get_client_id_priv(struct hw_fence_driver_data *drv_data,
	enum hw_fence_client_id client_id)
{
	int i, client_type, offset;
	enum hw_fence_client_id client_id_priv;

	if (client_id < HW_FENCE_MAX_STATIC_CLIENTS_INDEX)
		return client_id;

	/* consolidate external 'hw_fence_client_id' enum into consecutive internal client IDs */
	client_type = HW_FENCE_MAX_CLIENT_TYPE_STATIC +
		(client_id - HW_FENCE_MAX_STATIC_CLIENTS_INDEX) /
		MSM_HW_FENCE_MAX_SIGNAL_PER_CLIENT;
	offset = (client_id - HW_FENCE_MAX_STATIC_CLIENTS_INDEX) %
		MSM_HW_FENCE_MAX_SIGNAL_PER_CLIENT;

	/* invalid client id out of range of supported configurable sub-clients */
	if (offset >= drv_data->hw_fence_client_types[client_type].clients_num)
		return HW_FENCE_CLIENT_MAX;

	client_id_priv = HW_FENCE_MAX_STATIC_CLIENTS_INDEX + offset;

	for (i = HW_FENCE_MAX_CLIENT_TYPE_STATIC; i < client_type; i++)
		client_id_priv += drv_data->hw_fence_client_types[i].clients_num;

	return client_id_priv;
}

int hw_fence_utils_get_queues_num(struct hw_fence_driver_data *drv_data, int client_id)
{
	if (!drv_data || client_id >= drv_data->clients_num ||
			!drv_data->hw_fence_client_queue_size[client_id].type) {
		HWFNC_ERR("invalid access to client:%d queues_num\n", client_id);
		return 0;
	}

	return drv_data->hw_fence_client_queue_size[client_id].type->queues_num;
}
