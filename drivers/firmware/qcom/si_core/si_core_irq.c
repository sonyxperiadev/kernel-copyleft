// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#define pr_fmt(fmt) "si-irq: %s: " fmt, __func__

#include <linux/atomic.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/dma-buf.h>
#include <linux/qtee_shmbridge.h>
#include <linux/firmware/qcom/si_core_xts.h>
#include "si_core.h"
#include "si_core_irq.h"

static struct worker_kthread irq_worker;
static struct doorbell_mo_ctx *mo_obj_ctx;
static struct task_runner_cbo *task_cbo_obj;
static struct si_object *doorbell_service_obj;
static struct si_object *task_runner_service_obj;
static int si_core_irq = -1;

static int op_callrunnable(struct si_arg args[])
{
	if (size_of_arg(args) != 1 || args[0].type != SI_AT_IO)
		return -EINVAL;

	int ret, result = 0;
	static struct si_object_invoke_ctx oic;
	struct si_arg in_args[1] = { 0 };

	/* invocation on input object of type IRunnable, IRunnable_OP_run is 0 */
	ret = si_object_do_invoke(&oic, args[0].o, 0, in_args, &result);
	if (ret || result) {
		pr_err("%s failed with result %d(ret = %d).\n", __func__,
		       result, ret);
		return -EINVAL;
	}

	return 0;
}

static void task_runner_cbo_notify(unsigned int context_id,
				   struct si_object *object, int status)
{
	/* Notification handling not required for this CBO */
}

static void task_runner_cbo_release(struct si_object *object)
{
	struct task_runner_cbo *cbo_obj = to_task_runner_cbo(object);

	if (cbo_obj != task_cbo_obj)
		return;

	kfree(task_cbo_obj);
	task_cbo_obj = NULL;
}

static int task_runner_cbo_dispatch(unsigned int context_id,
	struct si_object *object, unsigned long op, struct si_arg args[])
{
	int ret = 0;

	switch (SI_OBJECT_OP_METHOD_ID(op)) {
	case SI_OBJECT_OP_CALLRUNNABLE: {
		ret = op_callrunnable(args);
	}
		break;
	default:
		/* The operation is not supported! */
		ret = -EINVAL;
		break;
	}

	return ret;
}

static struct si_object_operations task_runner_cbo_ops = {
	.notify = task_runner_cbo_notify,
	.release = task_runner_cbo_release,
	.dispatch = task_runner_cbo_dispatch
};

static int irq_kthread_func(void *data)
{
	u32 doorbell_id = 0;
	u32 msg_id = 0;
	struct doorbell_msg *msg;
	void *buff;
	int ret = 0;

	while (!kthread_should_stop()) {
		wait_event_interruptible(
			irq_worker.kthread_wq,
			kthread_should_stop() ||
			(atomic_read(&irq_worker.counter_wq) > 0));

		if (atomic_read(&irq_worker.counter_wq) > 0) {
			pr_debug("reading the contents of doorbell msg buffer\n");
			if (!mo_obj_ctx || !mo_obj_ctx->vaddr) {
				/* This is a fatal condition, if mo is not available then
				 * something went seriously wrong, and the thread shouldn't
				 * continue.
				 */
				atomic_sub(1, &irq_worker.counter_wq);
				pr_err("mo_obj_ctx is null!\n");
				irq_worker.alive = false;
				return -1;
			}

			/* This thread is holding a reference to this object, do not
			 * release it yet
			 */
			get_si_object(mo_obj_ctx->object);

			msg = (struct doorbell_msg *)mo_obj_ctx->vaddr;
			buff = msg->buff;

			ret = process_doorbell_msg(buff);
			if (ret)
				pr_err("process_doorbell_msg failed with %d!\n", ret);

			put_si_object(mo_obj_ctx->object);

			/* Send ACK to notify QTEE doorbell msg has been consumed */
			ret = qcom_scm_invoke_ack_doorbell(doorbell_id, msg_id);
			if (ret) {
				/* QTEE won't raise another Doorbell interrupt if Doorbell ACK
				 * failed. So there's no reason to keep this thread going.
				 */
				pr_err("doorbell_ack scm call failed with %d!\n", ret);
				atomic_sub(1, &irq_worker.counter_wq);
				irq_worker.alive = false;
				return -1;
			}

			atomic_sub(1, &irq_worker.counter_wq);
		}
	}

	irq_worker.alive = false;
	pr_warn("irq_worker thread exiting.\n");
	return 0;
}

static int irq_kthread_create(void)
{
	int ret;

	init_waitqueue_head(&irq_worker.kthread_wq);
	atomic_set(&irq_worker.counter_wq, 0);
	irq_worker.alive = false;

	irq_worker.kthread_task = kthread_run(irq_kthread_func, NULL,
					      "irq_worker_thread");
	if (IS_ERR(irq_worker.kthread_task)) {
		ret = PTR_ERR(irq_worker.kthread_task);
		pr_err("fail to create irq worker kthread, ret = %x\n", ret);
		return ret;
	}

	irq_worker.alive = true;
	return 0;
}

static void irq_kthread_destroy(void)
{
	kthread_stop(irq_worker.kthread_task);
}

static irqreturn_t qtee_irq_handler(int irq, void *data)
{
	if (!irq_worker.alive) {
		/* Better explain why we are refusing to handle the Doorbell
		 * interrupt raised by QTEE.
		 */
		pr_warn_ratelimited("irq_worker is not available\n");
		return IRQ_NONE;
	}

	atomic_add(1, &irq_worker.counter_wq);
	wake_up_interruptible(&irq_worker.kthread_wq);

	return IRQ_HANDLED;
}

static void doorbell_mo_release(void *private)
{
	if ((struct doorbell_mo_ctx *)private != mo_obj_ctx)
		return;

	if (mo_obj_ctx) {
		__free_page(sg_page(mo_obj_ctx->sgt->sgl));
		sg_free_table(mo_obj_ctx->sgt);
		kfree(mo_obj_ctx->sgt);
		kfree(mo_obj_ctx);
		mo_obj_ctx = NULL;
	}
}

static int doorbell_mo_init(void)
{
	int ret;
	struct sg_table *sgt;
	struct page *page;

	mo_obj_ctx = kzalloc(sizeof(*mo_obj_ctx), GFP_KERNEL);
	if (!mo_obj_ctx)
		return -ENOMEM;

	mo_obj_ctx->size = NUM_PAGES * PAGE_SIZE;
	mo_obj_ctx->sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!mo_obj_ctx->sgt) {
		ret = -ENOMEM;
		goto err_release_sgt;
	}

	/* alloc_pages() allocates contiguous memory so we need an sgt with
	 * just 1 entry.
	 */
	ret = sg_alloc_table(mo_obj_ctx->sgt, 1, GFP_KERNEL);
	if (ret)
		goto err_sg_alloc_table;

	page = alloc_pages(GFP_KERNEL, get_order(mo_obj_ctx->size));
	if (!page) {
		ret = -ENOMEM;
		goto err_page_alloc;
	}

	sg_set_page(mo_obj_ctx->sgt->sgl, page, mo_obj_ctx->size, 0);
	mo_obj_ctx->vaddr = sg_virt(mo_obj_ctx->sgt->sgl);

	mo_obj_ctx->object = init_si_mem_object_sg(mo_obj_ctx->sgt, 0, 0,
						doorbell_mo_release,
						mo_obj_ctx);

	if (!mo_obj_ctx->object) {
		pr_err("init_si_mem_object_sg failed.\n");
		ret = -EINVAL;
		goto err_init_mo;
	}

	return 0;

err_init_mo:
	__free_page(page);
err_page_alloc:
	sg_free_table(mo_obj_ctx->sgt);
err_sg_alloc_table:
	kfree(mo_obj_ctx->sgt);
err_release_sgt:
	kfree(mo_obj_ctx);
	mo_obj_ctx = NULL;
	return ret;
}

static int qtee_task_runner_cbo_register(void)
{
	int ret, result;
	uint32_t flags = 0;
	static struct si_object_invoke_ctx oic;
	struct si_object *client_env;
	struct si_arg args[3] = { 0 };

	ret = si_core_get_client_env(&oic, &client_env);
	if (ret) {
		pr_err("si_core_get_client_env failed (ret = %d).\n", ret);
		return ret;
	}

	ret = si_core_client_env_open(&oic, client_env, SI_REGISTER_TASK_RUNNER_UID,
				      &task_runner_service_obj);
	if (ret) {
		pr_err("si_core_client_env_open failed (ret = %d).\n", ret);
		goto out_client;
	}

	task_cbo_obj = kzalloc(sizeof(*task_cbo_obj), GFP_KERNEL);
	if (!task_cbo_obj) {
		ret = -ENOMEM;
		goto out_service;
	}

	ret = init_si_object_user(&task_cbo_obj->object, SI_OT_CB_OBJECT,
			    &task_runner_cbo_ops, "task_runner_cbo_obj");
	if (ret)
		goto out_object_init;

	args[0].b = (struct si_buffer) { {&flags}, sizeof(flags) };
	args[0].type = SI_AT_IB;
	args[1].o = &task_cbo_obj->object;
	args[1].type = SI_AT_IO;
	args[2].type = SI_AT_END;

	get_si_object(&task_cbo_obj->object);

	/* IRegisterTaskRunner_OP_registerRunner is 0. */
	ret =  si_object_do_invoke(&oic, task_runner_service_obj, 0, args, &result);
	if (ret) {
		pr_err("Unable to register task_runner_cbo with QTEE (ret = %d).\n", ret);
		put_si_object(&task_cbo_obj->object);
		goto out_cbo;
	}

	put_si_object(client_env);

	return 0;

out_cbo:
	put_si_object(&task_cbo_obj->object);
out_object_init:
	kfree(task_cbo_obj);
out_service:
	put_si_object(task_runner_service_obj);
out_client:
	put_si_object(client_env);

	return ret;
}

static void qtee_task_runner_cbo_unregister(void)
{
	put_si_object(task_runner_service_obj);
}

static int qtee_doorbell_register(uint32_t irq)
{
	int ret, result;
	static struct si_object_invoke_ctx oic;
	struct si_object *client_env;
	struct si_arg args[3] = { 0 };

	ret = si_core_get_client_env(&oic, &client_env);
	if (ret) {
		pr_err("si_core_get_client_env failed (ret = %d).\n", ret);
		return ret;
	}

	ret = si_core_client_env_open(&oic, client_env, SI_DOORBELL_UID,
				      &doorbell_service_obj);
	if (ret) {
		pr_err("si_core_client_env_open failed (ret = %d).\n", ret);
		goto out_client;
	}

	ret = doorbell_mo_init();
	if (ret) {
		pr_err("failed to create memobj (ret = %d).\n", ret);
		goto out_service;
	}

	args[0].b = (struct si_buffer) { {&irq}, sizeof(irq) };
	args[0].type = SI_AT_IB;
	args[1].o = mo_obj_ctx->object;
	args[1].type = SI_AT_IO;
	args[2].type = SI_AT_END;

	get_si_object(mo_obj_ctx->object);

	/* IDoorbell_OP_registerInterrupt is 0. */
	ret =  si_object_do_invoke(&oic, doorbell_service_obj, 0, args, &result);
	if (ret) {
		pr_err("unable to register doorbell with QTEE (ret = %d).\n", ret);
		put_si_object(mo_obj_ctx->object);
		goto out_mo;
	}

	put_si_object(client_env);

	return 0;

out_mo:
	put_si_object(mo_obj_ctx->object);
out_service:
	put_si_object(doorbell_service_obj);
out_client:
	put_si_object(client_env);

	return ret;
}

static void qtee_doorbell_unregister(void)
{
	put_si_object(doorbell_service_obj);
}

int si_core_doorbell_setup(struct platform_device *pdev)
{
	int ret = -1;
	struct device *dev = &pdev->dev;

	si_core_irq = platform_get_irq_optional(pdev, 0);
	if (si_core_irq < 0) {
		ret = si_core_irq;
		return ret;
	}

	ret = irq_kthread_create();
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, si_core_irq, NULL, qtee_irq_handler,
					IRQF_ONESHOT, "qcom,mem-object", dev);
	if (ret < 0) {
		pr_err("failed to request qcom-scm irq (ret = %d).\n", ret);
		goto err_devm_req_threaded_irq;
	}


	/* platform_get_irq_optional returns linux virtual irq, send hw_irq to QTEE */
	struct irq_data *irqd = irq_get_irq_data(si_core_irq);
	uint32_t hw_irq = (uint32_t)irqd->hwirq;

	ret = qtee_doorbell_register(hw_irq);
	if (ret)
		goto err_doorbell_register;

	ret = qtee_task_runner_cbo_register();
	if (ret)
		goto err_task_runner_register;

	return 0;

err_task_runner_register:
	qtee_doorbell_unregister();
err_doorbell_register:
	devm_free_irq(dev, si_core_irq, dev);
err_devm_req_threaded_irq:
	irq_kthread_destroy();

	pr_err("Doorbell setup with QTEE failed %d\n", ret);
	return ret;
}

void si_core_doorbell_cleanup(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	put_si_object(&task_cbo_obj->object);
	qtee_task_runner_cbo_unregister();

	put_si_object(mo_obj_ctx->object);
	qtee_doorbell_unregister();

	devm_free_irq(dev, si_core_irq, dev);
	irq_kthread_destroy();
}
