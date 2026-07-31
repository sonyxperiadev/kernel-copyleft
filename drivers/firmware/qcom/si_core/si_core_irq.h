/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __SI_CORE_IRQ_H__
#define __SI_CORE_IRQ_H__

#define NUM_PAGES                      1
#define IRQ_KT_SLEEP                   0
#define IRQ_KT_WAKE                    1
#define SI_OBJECT_OP_CALLRUNNABLE      0
#define SI_DOORBELL_UID                439
#define SI_REGISTER_TASK_RUNNER_UID    437

#define to_task_runner_cbo(o) container_of((o), struct task_runner_cbo, object)

struct doorbell_mo_ctx {
	struct si_object *object;
	void *vaddr;
	size_t size;
	struct sg_table *sgt;
};

struct task_runner_cbo {
	struct si_object object;
};

struct worker_kthread {
	atomic_t counter_wq;
	wait_queue_head_t kthread_wq;
	struct task_struct *kthread_task;
	bool alive;
};

struct doorbell_msg {
	uint32_t tag;
	uint32_t len;
	uint8_t buff[];
};

#ifdef CONFIG_QCOM_SI_CORE_DOORBELL

int si_core_doorbell_setup(struct platform_device *pdev);
void si_core_doorbell_cleanup(struct platform_device *pdev);

#else

static inline int si_core_doorbell_setup(struct platform_device *pdev)
{
	return 0;
}

static inline void si_core_doorbell_cleanup(struct platform_device *pdev)
{
}

#endif /* CONFIG_QCOM_SI_CORE_DOORBELL */

#endif /* __SI_CORE_IRQ_H__ */
