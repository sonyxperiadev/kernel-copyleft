// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */
#include <linux/circ_buf.h>
#include <linux/delay.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/rpmsg.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/mailbox_client.h>
#include <linux/rpmsg/qcom_glink.h>
#include <linux/ipc_logging.h>

#include "qcom_glink_native.h"
#include "qcom_glink_cma.h"

#define GLINK_CMA_DEBUG_LOG(ctxt, fmt, ...)	\
		ipc_log_string(ctxt, "%s: %d" fmt "\n", __func__, __LINE__, ##__VA_ARGS__)

#define FIFO_FULL_RESERVE	8
#define FIFO_ALIGNMENT		8
#define TX_BLOCKED_CMD_RESERVE	8 /* size of struct read_notif_request */

#define FIFO_SIZE		0x4000

#define HDR_KEY_VALUE		0xdead

#define MAGIC_KEY_VALUE		0x24495043 /* "$IPC" */
#define MAGIC_KEY		0x0
#define BUFFER_SIZE		0x4

#define FIFO_0_START_OFFSET	0x1000
#define FIFO_0_BASE		0x8
#define FIFO_0_SIZE		0xc
#define FIFO_0_TAIL		0x10
#define FIFO_0_HEAD		0x14
#define FIFO_0_NOTIFY		0x18

#define FIFO_1_START_OFFSET	(FIFO_0_START_OFFSET + FIFO_SIZE)
#define FIFO_1_BASE		0x1c
#define FIFO_1_SIZE		0x20
#define FIFO_1_TAIL		0x24
#define FIFO_1_HEAD		0x28
#define FIFO_1_NOTIFY		0x2c

struct glink_cma_hdr {
	__le16 len;
	__le16 magic;
};

struct glink_cma_pipe {
	struct qcom_glink_pipe native;

	__le32 *tail;
	__le32 *head;

	void *fifo;
};

/**
 * glink_cma_dev - GLINK CMA fifo transport structure
 * @dev: device for this node.
 * @config: configuration setting for this transport.
 * @rx_pipe: RX CMA GLINK fifo specific info.
 * @tx_pipe: TX CMA GLINK fifo specific info.
 * @glink_cma_ilc: IPC logging context reference.
 * @name: Name of the label.
 * @irq: IRQ for signaling incoming events.
 * @irqname: Name associated with irq.
 * @mbox_client: mailbox client.
 * @mbox_chan: mailbox channel.
 * @glink: glink struct associated with this glink cma dev.
 */
struct glink_cma_dev {
	struct device dev;
	struct glink_cma_config *config;
	struct glink_cma_pipe rx_pipe;
	struct glink_cma_pipe tx_pipe;
	void *glink_cma_ilc;
	const char *name;
	int irq;
	char irqname[32];
	struct mbox_client mbox_client;
	struct mbox_chan *mbox_chan;
	struct qcom_glink *glink;
};

#define to_glink_cma_pipe(p) container_of(p, struct glink_cma_pipe, native)

static size_t glink_cma_rx_avail(struct qcom_glink_pipe *np)
{
	struct glink_cma_pipe *pipe = to_glink_cma_pipe(np);
	size_t len;
	u32 head;
	u32 tail;

	head = le32_to_cpu(*pipe->head);
	tail = le32_to_cpu(*pipe->tail);

	if (head < tail)
		len = pipe->native.length - tail + head;
	else
		len = head - tail;

	if (WARN_ON_ONCE(len > pipe->native.length))
		len = 0;

	return len;
}

static void glink_cma_rx_peek(struct qcom_glink_pipe *np, void *data,
				unsigned int offset, size_t count)
{
	struct glink_cma_pipe *pipe = to_glink_cma_pipe(np);
	size_t len;
	u32 tail;

	tail = le32_to_cpu(*pipe->tail);

	if (WARN_ON_ONCE(tail > pipe->native.length))
		return;

	tail += offset;
	if (tail >= pipe->native.length)
		tail %= pipe->native.length;

	len = min_t(size_t, count, pipe->native.length - tail);
	if (len)
		memcpy_fromio(data, pipe->fifo + tail, len);

	if (len != count)
		memcpy_fromio(data + len, pipe->fifo, (count - len));
}

static void glink_cma_rx_advance(struct qcom_glink_pipe *np, size_t count)
{
	struct glink_cma_pipe *pipe = to_glink_cma_pipe(np);
	u32 tail;

	tail = le32_to_cpu(*pipe->tail);

	tail += count;
	if (tail >= pipe->native.length)
		tail %= pipe->native.length;

	*pipe->tail = cpu_to_le32(tail);
}

static size_t glink_cma_tx_avail(struct qcom_glink_pipe *np)
{
	struct glink_cma_pipe *pipe = to_glink_cma_pipe(np);
	u32 avail;
	u32 head;
	u32 tail;

	head = le32_to_cpu(*pipe->head);
	tail = le32_to_cpu(*pipe->tail);

	if (tail <= head)
		avail = pipe->native.length - head + tail;
	else
		avail = tail - head;

	if (WARN_ON_ONCE(avail > pipe->native.length))
		return 0;

	if (avail < (FIFO_FULL_RESERVE + TX_BLOCKED_CMD_RESERVE))
		return 0;

	return avail - (FIFO_FULL_RESERVE + TX_BLOCKED_CMD_RESERVE);
}

static unsigned int glink_cma_tx_write_one(struct glink_cma_pipe *pipe, unsigned int head,
					    const void *data, size_t count)
{
	size_t len;

	if (WARN_ON_ONCE(head > pipe->native.length))
		return head;

	len = min_t(size_t, count, pipe->native.length - head);
	if (len)
		memcpy(pipe->fifo + head, data, len);

	if (len != count)
		memcpy(pipe->fifo, data + len, count - len);

	head += count;
	if (head >= pipe->native.length)
		head %= pipe->native.length;

	return head;
}

static void glink_cma_tx_write(struct qcom_glink_pipe *glink_pipe,
				const void *hdr, size_t hlen,
				const void *data, size_t dlen)
{
	struct glink_cma_pipe *pipe = to_glink_cma_pipe(glink_pipe);
	unsigned int head;

	head = le32_to_cpu(*pipe->head);

	head = glink_cma_tx_write_one(pipe, head, hdr, hlen);
	head = glink_cma_tx_write_one(pipe, head, data, dlen);

	/* Ensure head is always aligned to 8 bytes */
	head = ALIGN(head, 8);
	if (head >= pipe->native.length)
		head %= pipe->native.length;

	/* Ensure ordering of fifo and head update */
	wmb();

	*pipe->head = cpu_to_le32(head);
}

static void glink_cma_tx_kick(struct qcom_glink_pipe *glink_pipe)
{
	struct glink_cma_pipe *pipe = to_glink_cma_pipe(glink_pipe);
	struct glink_cma_dev *cma = container_of(pipe, struct glink_cma_dev, tx_pipe);

	mbox_send_message(cma->mbox_chan, NULL);
	mbox_client_txdone(cma->mbox_chan, 0);
}
static void glink_cma_native_init(struct glink_cma_dev *gdev)
{
	struct qcom_glink_pipe *tx_native = &gdev->tx_pipe.native;
	struct qcom_glink_pipe *rx_native = &gdev->rx_pipe.native;

	tx_native->length = FIFO_SIZE;
	tx_native->avail = glink_cma_tx_avail;
	tx_native->write = glink_cma_tx_write;
	tx_native->kick = glink_cma_tx_kick;

	rx_native->length = FIFO_SIZE;
	rx_native->avail = glink_cma_rx_avail;
	rx_native->peek = glink_cma_rx_peek;
	rx_native->advance = glink_cma_rx_advance;
	GLINK_CMA_DEBUG_LOG(gdev->glink_cma_ilc, "success");
}

static int glink_cma_fifo_init(struct glink_cma_dev *gdev)
{
	struct glink_cma_pipe *rx_pipe = &gdev->rx_pipe;
	struct glink_cma_pipe *tx_pipe = &gdev->tx_pipe;
	struct glink_cma_config *config = gdev->config;
	u8 *descs = config->base;

	if (!descs)
		return -EINVAL;

	memset(descs, 0, FIFO_0_START_OFFSET);

	*(u32 *)(descs + MAGIC_KEY) = MAGIC_KEY_VALUE;
	*(u32 *)(descs + BUFFER_SIZE) = config->size;

	*(u32 *)(descs + FIFO_0_BASE) = FIFO_0_START_OFFSET;
	*(u32 *)(descs + FIFO_0_SIZE) = FIFO_SIZE;
	tx_pipe->fifo = (u32 *)(descs + FIFO_0_START_OFFSET);
	tx_pipe->tail = (u32 *)(descs + FIFO_0_TAIL);
	tx_pipe->head = (u32 *)(descs + FIFO_0_HEAD);

	*(u32 *)(descs + FIFO_1_BASE) = FIFO_1_START_OFFSET;
	*(u32 *)(descs + FIFO_1_SIZE) = FIFO_SIZE;
	rx_pipe->fifo = (u32 *)(descs + FIFO_1_START_OFFSET);
	rx_pipe->tail = (u32 *)(descs + FIFO_1_TAIL);
	rx_pipe->head = (u32 *)(descs + FIFO_1_HEAD);

	/* Reset respective index */
	*tx_pipe->head = 0;
	*rx_pipe->tail = 0;

	GLINK_CMA_DEBUG_LOG(gdev->glink_cma_ilc, "success");
	return 0;
}

static void qcom_glink_cma_release(struct device *dev)
{
	struct glink_cma_dev *gdev = dev_get_drvdata(dev);
	GLINK_CMA_DEBUG_LOG(gdev->glink_cma_ilc, "");
	kfree(gdev);
}

static irqreturn_t qcom_glink_cma_intr(int irq, void *data)
{
	struct glink_cma_dev *gdev = data;

	qcom_glink_native_rx(gdev->glink);

	return IRQ_HANDLED;
}
struct glink_cma_dev *qcom_glink_cma_register(struct device *parent, struct device_node *node,
					struct glink_cma_config *config)
{
	struct glink_cma_dev *gdev;
	struct qcom_glink *glink;
	struct device *dev;
	int rc, ret;

	if (!parent || !node || !config)
		return ERR_PTR(-EINVAL);

	gdev = kzalloc(sizeof(*gdev), GFP_KERNEL);
	if (!gdev)
		return ERR_PTR(-ENOMEM);

	dev = &gdev->dev;
	dev->parent = parent;
	dev->of_node = node;
	dev->release = qcom_glink_cma_release;
	dev_set_name(dev, "%s:%pOFn", dev_name(parent->parent), node);
	rc = device_register(dev);
	if (rc) {
		pr_err("failed to register glink edge\n");
		put_device(dev);
		kfree(gdev);
		return ERR_PTR(rc);
	}

	gdev->glink_cma_ilc = ipc_log_context_create(2, "glink_cma", 0);
	dev_set_drvdata(dev, gdev);
	gdev->config = config;

	rc = glink_cma_fifo_init(gdev);
	if (rc) {
		kfree(gdev);
		return ERR_PTR(rc);
	}

	ret = of_property_read_string(dev->of_node, "label", &gdev->name);

	scnprintf(gdev->irqname, 32, "glink-native-%s", gdev->name);

	gdev->irq = of_irq_get(gdev->dev.of_node, 0);
	ret = devm_request_irq(&gdev->dev, gdev->irq, qcom_glink_cma_intr,
							IRQF_NO_SUSPEND,
							gdev->irqname, gdev);
	if (ret) {
		pr_err("%s: failed to request irq\n", __func__);
		goto err_put_dev;
	}

	gdev->mbox_client.dev = &gdev->dev;
	gdev->mbox_client.knows_txdone = true;
	gdev->mbox_chan = mbox_request_channel(&gdev->mbox_client, 0);
	if (IS_ERR(gdev->mbox_chan)) {
		pr_err("%s: failed to get mbox channel\n", __func__);
		goto err_put_dev;
	}

	glink_cma_native_init(gdev);

	glink = qcom_glink_native_probe(dev, GLINK_FEATURE_INTENT_REUSE,
					&gdev->rx_pipe.native, &gdev->tx_pipe.native, false);
	if (IS_ERR(glink)) {
		rc = PTR_ERR(glink);
		goto err_free_mbox;
	}

	gdev->glink = glink;

	GLINK_CMA_DEBUG_LOG(gdev->glink_cma_ilc, "success");
	return gdev;

err_free_mbox:
	mbox_free_channel(gdev->mbox_chan);

err_put_dev:
	GLINK_CMA_DEBUG_LOG(gdev->glink_cma_ilc, "Exit error %d", rc);
	device_unregister(dev);
	kfree(gdev);

	return ERR_PTR(rc);
}
EXPORT_SYMBOL_GPL(qcom_glink_cma_register);

int qcom_glink_cma_start(struct qcom_glink *glink)
{
	return qcom_glink_native_start(glink);
}
EXPORT_SYMBOL_GPL(qcom_glink_cma_start);

void qcom_glink_cma_unregister(struct glink_cma_dev *gdev)
{

	if (!gdev)
		return;

	disable_irq(gdev->irq);

	qcom_glink_native_remove(gdev->glink);

	mbox_free_channel(gdev->mbox_chan);
	device_unregister(&gdev->dev);
}
EXPORT_SYMBOL_GPL(qcom_glink_cma_unregister);
