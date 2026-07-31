// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/random.h>
#include <linux/remoteproc/qcom_rproc.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <linux/soc/qcom/qcom_dbl.h>

#define DBL_WAIT_DEFAULT_TIMEOUT_MS 1000
#define DBL_REG_VOTE 0x1
#define DBL_REG_UNVOTE 0x0

enum register_state {
	DBL_INIT = 0,
	DBL_VOTE,
	DBL_ACKED
};

struct client_handle {
	struct dbl_driver *lpi_dbl;
	char client_name[32];
	u8 flags;
	bool unregistered;
	void (*cb)(void);
	atomic_t client_vote_count;
	struct device *client_dev;
	struct list_head node;
	struct list_head async_node;
	struct work_struct cb_work;
};

struct dbl_driver {
	void __iomem *dbl_reg;
	int irq;
	enum register_state dbl_reg_state;
	atomic_t vote_count;
	atomic_t lpai_reset;
	spinlock_t lock;
	struct mutex client_list_lock;
	struct device *dev;
	struct list_head active_clients;
	struct list_head async_clients;
	struct notifier_block ssr_nb;
	void *ssr_notifier_handle;
	wait_queue_head_t wait_queue;
	struct workqueue_struct *wq;
};

static void dbl_kick(struct dbl_driver *dbl, u32 value)
{
	writel(value, dbl->dbl_reg);
}

static int qcom_dbl_ssr_notifier_cb(struct notifier_block *nb,
			    unsigned long event, void *data)
{
	struct dbl_driver *dbl = container_of(nb, struct dbl_driver, ssr_nb);
	unsigned long flags;

	switch (event) {
	case QCOM_SSR_BEFORE_SHUTDOWN:
		spin_lock_irqsave(&dbl->lock, flags);
		atomic_set(&dbl->lpai_reset, 1);
		wake_up_all(&dbl->wait_queue);
		spin_unlock_irqrestore(&dbl->lock, flags);
		break;
	case QCOM_SSR_AFTER_POWERUP:
		spin_lock_irqsave(&dbl->lock, flags);
		atomic_set(&dbl->lpai_reset, 0);
		dbl->dbl_reg_state = DBL_INIT;
		spin_unlock_irqrestore(&dbl->lock, flags);
		break;
	default:
		break;
	}

	return NOTIFY_DONE;
}

static void dbl_client_cb_work(struct work_struct *work)
{
	struct client_handle *client =
		container_of(work, struct client_handle, cb_work);

	if (client->cb)
		client->cb();
}

static struct dbl_driver *lpi_dbl_get(void)
{
	struct device_node *np;
	struct platform_device *pdev;

	np = of_find_compatible_node(NULL, NULL, "qcom,lpi-dbl");
	if (!np)
		return ERR_PTR(-ENODEV);

	pdev = of_find_device_by_node(np);
	if (!pdev)
		return ERR_PTR(-ENODEV);

	return platform_get_drvdata(pdev);
}

static struct client_handle *find_client_by_handle(struct dbl_client *handle)
{
	struct dbl_driver *dbl;
	struct client_handle *tmp;
	struct client_handle *client = (struct client_handle *)handle;

	dbl = lpi_dbl_get();
	if (IS_ERR(dbl)) {
		pr_err("Failed to get dbl driver: %ld\n", PTR_ERR(dbl));
		return NULL;
	}

	if (!dbl)
		return ERR_PTR(-ENODEV);

	mutex_lock(&dbl->client_list_lock);
	list_for_each_entry(tmp, &dbl->active_clients, node) {
		if (tmp == client) {
			mutex_unlock(&dbl->client_list_lock);
			return client;
		}
	}
	mutex_unlock(&dbl->client_list_lock);
	pr_err("DBL: Client handle %pK not found (process: %s, pid: %d)\n",
			handle, current->comm, task_pid_nr(current));

	return NULL;
}

static int dbl_wait_for_state_change(struct dbl_driver *dbl, enum register_state expected,
				     long timeout)
{
	int ret;

	ret = wait_event_interruptible_lock_irq_timeout(dbl->wait_queue,
			(dbl->dbl_reg_state == expected) ||
			atomic_read(&dbl->lpai_reset),
			dbl->lock, timeout);

	if (atomic_read(&dbl->lpai_reset))
		ret = -ERESTARTSYS;
	else if (ret == 0 && dbl->dbl_reg_state != expected)
		ret = -ETIMEDOUT;
	else if (ret > 0)
		ret = 0;
	return ret;
}

struct dbl_client *dbl_register_client(const char *name)
{
	struct dbl_driver *dbl;
	struct client_handle *client = NULL;

	dbl = lpi_dbl_get();
	if (IS_ERR(dbl)) {
		long err = PTR_ERR(dbl);

		if (err != -EPROBE_DEFER)
			pr_err("Failed to get dbl driver: %ld\n", err);
		return ERR_PTR(err);
	}

	if (!dbl)
		return ERR_PTR(-ENODEV);

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return ERR_PTR(-ENOMEM);

	client->unregistered = false;
	strscpy(client->client_name, name, sizeof(client->client_name));
	atomic_set(&client->client_vote_count, 0);
	INIT_LIST_HEAD(&client->node);
	INIT_LIST_HEAD(&client->async_node);
	INIT_WORK(&client->cb_work, dbl_client_cb_work);

	client->lpi_dbl = dbl;

	mutex_lock(&dbl->client_list_lock);
	list_add_tail(&client->node, &dbl->active_clients);
	mutex_unlock(&dbl->client_list_lock);

	return (struct dbl_client *)client;
}
EXPORT_SYMBOL_GPL(dbl_register_client);

void dbl_unregister_client(struct dbl_client *client_handle)
{
	struct dbl_driver *dbl;
	struct client_handle *client;
	int remaining_votes;
	unsigned long flags;

	client = find_client_by_handle(client_handle);
	if (!client)
		return;

	dbl = client->lpi_dbl;
	if (IS_ERR(dbl)) {
		pr_err("Failed to get dbl driver: %ld\n", PTR_ERR(dbl));
		return;
	}
	client->unregistered = true;
	remaining_votes = atomic_read(&client->client_vote_count);
	if (remaining_votes > 0) {
		spin_lock_irqsave(&dbl->lock, flags);
		if (atomic_sub_return(remaining_votes, &dbl->vote_count) == 0) {
			if (dbl->dbl_reg_state == DBL_ACKED) {
				dbl_kick(dbl, DBL_REG_UNVOTE);
				dbl->dbl_reg_state = DBL_INIT;
			} else if (dbl->dbl_reg_state == DBL_VOTE) {
				dbl_wait_for_state_change(dbl, DBL_ACKED, 1000);
				dbl_kick(dbl, DBL_REG_UNVOTE);
				dbl->dbl_reg_state = DBL_INIT;
			}
		}
		spin_unlock_irqrestore(&dbl->lock, flags);
	}
	mutex_lock(&dbl->client_list_lock);
	list_del(&client->node);
	mutex_unlock(&dbl->client_list_lock);
	kfree(client);
}
EXPORT_SYMBOL_GPL(dbl_unregister_client);

int lpi_dbl_unvote(struct dbl_client *client_handle)
{
	struct dbl_driver *dbl;
	struct client_handle *client;
	unsigned long flags;

	client = find_client_by_handle(client_handle);
	if (!client)
		return -EINVAL;

	dbl = client->lpi_dbl;
	if (IS_ERR(dbl)) {
		pr_err("Failed to get dbl driver: %ld\n", PTR_ERR(dbl));
		return PTR_ERR(dbl);
	}
	atomic_dec(&client->client_vote_count);
	spin_lock_irqsave(&dbl->lock, flags);
	if (atomic_read(&dbl->vote_count) > 0 &&
		atomic_dec_and_test(&dbl->vote_count)) {
		dbl_kick(dbl, DBL_REG_UNVOTE);
		dbl->dbl_reg_state = DBL_INIT;
	}
	spin_unlock_irqrestore(&dbl->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(lpi_dbl_unvote);

int lpi_dbl_vote(struct dbl_client *client_handle, void *async_cb, long timeout_ms)
{
	struct dbl_driver *dbl;
	struct client_handle *client;
	unsigned long flags;
	long timeout;
	int ret = 0;
	enum register_state current_state;

	client = find_client_by_handle(client_handle);
	if (!client)
		return -EINVAL;

	dbl = client->lpi_dbl;
	if (IS_ERR(dbl)) {
		pr_err("Failed to get dbl driver: %ld\n", PTR_ERR(dbl));
		return PTR_ERR(dbl);
	}

	timeout = msecs_to_jiffies((timeout_ms <= 0) ? DBL_WAIT_DEFAULT_TIMEOUT_MS : timeout_ms);

	if (atomic_read(&dbl->lpai_reset))
		return -ECONNRESET;

	spin_lock_irqsave(&dbl->lock, flags);

	current_state = dbl->dbl_reg_state;
	switch (current_state) {
	case DBL_INIT:
		dbl->dbl_reg_state = DBL_VOTE;
		dbl_kick(dbl, DBL_REG_VOTE);
		if (async_cb) {
			client->cb = (void (*)(void))async_cb;
			list_add(&client->async_node, &dbl->async_clients);
			spin_unlock_irqrestore(&dbl->lock, flags);
			return 0;
		}
		ret = dbl_wait_for_state_change(dbl, DBL_ACKED, timeout);
		if (ret < 0) {
			spin_lock_irqsave(&dbl->lock, flags);
			dbl->dbl_reg_state = DBL_INIT;
			spin_unlock_irqrestore(&dbl->lock, flags);
		} else {
			atomic_inc(&client->client_vote_count);
		}
		break;
	case DBL_VOTE:
		if (async_cb) {
			client->cb = (void (*)(void))async_cb;
			list_add(&client->async_node, &dbl->async_clients);
			spin_unlock_irqrestore(&dbl->lock, flags);
			return 0;
		}
		ret = dbl_wait_for_state_change(dbl, DBL_ACKED, timeout);
		break;
	case DBL_ACKED:
		atomic_inc(&dbl->vote_count);
		atomic_inc(&client->client_vote_count);
		if (async_cb) {
			client->cb = (void (*)(void))async_cb;
			queue_work(dbl->wq, &client->cb_work);
			spin_unlock_irqrestore(&dbl->lock, flags);
			return 0;
		}
		break;
	default:
		dev_err(dbl->dev, "Unexpected state %d\n", current_state);
		ret = -EINVAL;
		break;
	}
	spin_unlock_irqrestore(&dbl->lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(lpi_dbl_vote);

static irqreturn_t lpi_dbl_reg_intr(int irq, void *data)
{
	struct dbl_driver *dbl = data;
	struct client_handle *client, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&dbl->lock, flags);
	if (dbl->dbl_reg_state == DBL_VOTE) {
		dbl->dbl_reg_state = DBL_ACKED;
		atomic_inc(&dbl->vote_count);
		wake_up_all(&dbl->wait_queue);
		list_for_each_entry_safe(client, tmp, &dbl->async_clients, async_node) {
			if (client->unregistered)
				continue;
			atomic_inc(&client->client_vote_count);
			queue_work(dbl->wq, &client->cb_work);
			list_del(&client->async_node);
		}
	}
	spin_unlock_irqrestore(&dbl->lock, flags);
	return IRQ_HANDLED;
}

static int lpi_dbl_probe(struct platform_device *pdev)
{
	int ret;
	int irq;
	struct dbl_driver *dbl;
	struct resource *res;

	dbl = devm_kzalloc(&pdev->dev, sizeof(*dbl), GFP_KERNEL);
	if (!dbl)
		return -ENOMEM;

	dbl->dev = &pdev->dev;

	atomic_set(&dbl->vote_count, 0);
	atomic_set(&dbl->lpai_reset, 0);
	dbl->dbl_reg_state = DBL_INIT;
	dbl->wq = create_workqueue("dbl_wq");
	if (!dbl->wq)
		return -ENOMEM;

	init_waitqueue_head(&dbl->wait_queue);
	spin_lock_init(&dbl->lock);
	mutex_init(&dbl->client_list_lock);
	INIT_LIST_HEAD(&dbl->async_clients);
	INIT_LIST_HEAD(&dbl->active_clients);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	dbl->dbl_reg = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dbl->dbl_reg))
		return PTR_ERR(dbl->dbl_reg);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to get IRQ: %d\n", irq);
		return irq;
	}

	ret = devm_request_irq(&pdev->dev, irq, lpi_dbl_reg_intr, 0, "lpi_dbl_reg", dbl);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request interrupt: %d\n", ret);
		return ret;
	}
	enable_irq(irq);

	dbl->ssr_nb.notifier_call = qcom_dbl_ssr_notifier_cb;
	dbl->ssr_notifier_handle = qcom_register_ssr_notifier("lpai", &dbl->ssr_nb);
	if (IS_ERR(dbl->ssr_notifier_handle)) {
		ret = PTR_ERR(dbl->ssr_notifier_handle);
		dev_err(&pdev->dev, "Failed to register SSR notifier: %ld\n",
				PTR_ERR(dbl->ssr_notifier_handle));
		return ret;
	}

	platform_set_drvdata(pdev, dbl);
	return 0;
}

static void lpi_dbl_remove(struct platform_device *pdev)
{
	struct dbl_driver *dbl = platform_get_drvdata(pdev);

	destroy_workqueue(dbl->wq);
	qcom_unregister_ssr_notifier(dbl->ssr_notifier_handle, &dbl->ssr_nb);

}

static const struct of_device_id dbl_dt_match[] = {
	{ .compatible = "qcom,lpi-dbl", },
	{}
};
MODULE_DEVICE_TABLE(of, dbl_dt_match);

static struct platform_driver dbl_driver = {
	.driver = {
		.name = "qcom_lpi_doorbell",
		.of_match_table = dbl_dt_match,
		.suppress_bind_attrs = true,
	},
	.probe = lpi_dbl_probe,
	.remove = lpi_dbl_remove,
};
module_platform_driver(dbl_driver);

MODULE_DESCRIPTION("Qualcomm LPI Doorbell Driver");
MODULE_LICENSE("GPL");
