/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022, Qualcomm Innovation Center, Inc. All rights reserved.
 */
#include <linux/soc/qcom/panel_event_notifier.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/firmware.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include "linux/gunyah/gh_msgq.h"
#include "linux/gunyah/gh_rm_drv.h"
#include <linux/gunyah/gh_irq_lend.h>
#include <linux/gunyah/gh_mem_notifier.h>
#include "qts_core_common.h"
#include <linux/clk.h>
#include <linux/kobject.h>

#define QTS_NAME "qti-ts"

enum trusted_touch_mode_config {
	TRUSTED_TOUCH_VM_MODE,
	TRUSTED_TOUCH_MODE_NONE
};

enum trusted_touch_pvm_states {
	TRUSTED_TOUCH_PVM_INIT,
	PVM_I2C_RESOURCE_ACQUIRED,
	PVM_INTERRUPT_DISABLED,
	PVM_IOMEM_LENT,
	PVM_IOMEM_LENT_NOTIFIED,
	PVM_IRQ_LENT,
	PVM_IRQ_LENT_NOTIFIED,
	PVM_IOMEM_RELEASE_NOTIFIED,
	PVM_IRQ_RELEASE_NOTIFIED,
	PVM_ALL_RESOURCES_RELEASE_NOTIFIED,
	PVM_IRQ_RECLAIMED,
	PVM_IOMEM_RECLAIMED,
	PVM_INTERRUPT_ENABLED,
	PVM_I2C_RESOURCE_RELEASED,
	TRUSTED_TOUCH_PVM_STATE_MAX
};

enum trusted_touch_tvm_states {
	TRUSTED_TOUCH_TVM_INIT,
	TVM_IOMEM_LENT_NOTIFIED,
	TVM_IRQ_LENT_NOTIFIED,
	TVM_ALL_RESOURCES_LENT_NOTIFIED,
	TVM_IOMEM_ACCEPTED,
	TVM_I2C_SESSION_ACQUIRED,
	TVM_IRQ_ACCEPTED,
	TVM_INTERRUPT_ENABLED,
	TVM_INTERRUPT_DISABLED,
	TVM_IRQ_RELEASED,
	TVM_I2C_SESSION_RELEASED,
	TVM_IOMEM_RELEASED,
	TRUSTED_TOUCH_TVM_STATE_MAX
};

#define TRUSTED_TOUCH_MEM_LABEL 0x7

#define TOUCH_RESET_GPIO_BASE 0xf118000
#define TOUCH_RESET_GPIO_SIZE 0x1000
#define TOUCH_RESET_GPIO_OFFSET 0x4
#define TOUCH_INTR_GPIO_BASE 0xf119000
#define TOUCH_INTR_GPIO_SIZE 0x1000
#define TOUCH_INTR_GPIO_OFFSET 0x8

#define TRUSTED_TOUCH_EVENT_LEND_FAILURE -1
#define TRUSTED_TOUCH_EVENT_LEND_NOTIFICATION_FAILURE -2
#define TRUSTED_TOUCH_EVENT_ACCEPT_FAILURE -3
#define TRUSTED_TOUCH_EVENT_FUNCTIONAL_FAILURE -4
#define TRUSTED_TOUCH_EVENT_RELEASE_FAILURE -5
#define TRUSTED_TOUCH_EVENT_RECLAIM_FAILURE -6
#define TRUSTED_TOUCH_EVENT_I2C_FAILURE -7
#define TRUSTED_TOUCH_EVENT_NOTIFICATIONS_PENDING 5

struct trusted_touch_vm_info {
	enum gh_irq_label irq_label;
	enum gh_mem_notifier_tag mem_tag;
	enum gh_vm_names vm_name;
	const char *trusted_touch_type;
	u32 hw_irq;
	gh_memparcel_handle_t vm_mem_handle;
	u32 *iomem_bases;
	u32 *iomem_sizes;
	u32 iomem_list_size;
	void *mem_cookie;
	atomic_t vm_state;
};

struct qts_data;

struct qts_data {
	struct i2c_client *client;
	struct spi_device *spi;
	struct device *dev;
	struct device_node *dp;
	void *vendor_data; /* vendor touch driver data */
	u32 bus_type; /*i2c or spi*/
	u32 client_type; /* primary or secondary */
	struct drm_panel *panel;
	struct workqueue_struct *ts_workqueue;
	struct work_struct resume_work;
	struct work_struct suspend_work;
	bool schedule_suspend;
	bool schedule_resume;
	void *notifier_cookie;

	/* Resources */
	struct regulator *vdd;
	struct regulator *avdd;
	u32 irq_gpio;
	u32 irq_gpio_flags;
	u32 reset_gpio;
	u32 reset_gpio_flags;
	int irq;
	bool irq_disabled;
	bool power_disabled;

	struct mutex transition_lock;
	bool suspended;

	/* vendor callback ops */
	struct qts_vendor_callback_ops vendor_ops;

	/* TUI */
	bool tui_supported;
	struct trusted_touch_vm_info *vm_info;
	struct mutex qts_clk_io_ctrl_mutex;
	const char *touch_environment;
	struct completion trusted_touch_powerdown;
	struct clk *core_clk;
	struct clk *iface_clk;
	atomic_t trusted_touch_initialized;
	atomic_t trusted_touch_enabled;
	atomic_t trusted_touch_transition;
	atomic_t trusted_touch_event;
	atomic_t trusted_touch_abort_status;
	atomic_t delayed_tvm_probe_pending;
	atomic_t delayed_pvm_probe_pending;
	atomic_t trusted_touch_mode;
};

struct qts_data_entries;

struct qts_data_entries {
	struct kset *qts_kset;
	struct qts_data info[QTS_CLIENT_MAX];
	struct mutex qts_data_entries_lock;
};

#ifdef CONFIG_ARCH_QTI_VM
void qts_trusted_touch_tvm_i2c_failure_report(struct qts_data *qts_data);
#endif
