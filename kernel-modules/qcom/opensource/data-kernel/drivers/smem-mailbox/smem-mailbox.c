// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/soc/qcom/smem.h>
#include <linux/soc/qcom/smem_state.h>
#include <linux/types.h>
#include <linux/version.h>
#include <net/genetlink.h>

#include "smem-mailbox.h"

#define SMEM_VERSION 0x00000001 // v0.1
#define START_COOKIE 0xbeefcafe
#define END_COOKIE 0x0fed0fed

#define DEVICE_NAME "smem_mailbox"

#define HEAD_PTR_SIZE 4
#define TAIL_PTR_SIZE 4
#define BUF_IO_PTR_SIZE HEAD_PTR_SIZE + TAIL_PTR_SIZE

#define GET_READ_HEAD(smem, entry) *(u32 *)(smem + entry->start_loc)
#define GET_READ_TAIL(smem, entry) *(u32 *)(smem + entry->start_loc + HEAD_PTR_SIZE)
#define GET_READ_BUF(smem, entry) smem + entry->start_loc + BUF_IO_PTR_SIZE

#define GET_WRITE_HEAD(smem, entry) \
	*(u32 *)(smem + entry->start_loc + BUF_IO_PTR_SIZE + entry->read_size)
#define GET_WRITE_TAIL(smem, entry) \
	*(u32 *)(smem + entry->start_loc + BUF_IO_PTR_SIZE + entry->read_size + HEAD_PTR_SIZE)
#define GET_WRITE_BUF(smem, entry) \
	smem + entry->start_loc + BUF_IO_PTR_SIZE + entry->read_size + BUF_IO_PTR_SIZE

#define UPDATE_WRITE_HEAD(smem, value) \
	memcpy_toio(smem + entry->start_loc + BUF_IO_PTR_SIZE + entry->read_size, &value, \
		sizeof(value));
#define UPDATE_READ_TAIL(smem, value) \
	memcpy_toio(smem + entry->start_loc + HEAD_PTR_SIZE, &value, sizeof(value));

// This list must match the configured names in the device tree for the smp2p in (IRQ).
#define IRQ_NUM_NAMES 4
static char *IRQ_NAMES[] = {"smem-mailbox-smp2p-1-in", "smem-mailbox-smp2p-2-in",
			"smem-mailbox-smp2p-3-in", "smem-mailbox-smp2p-4-in"};

#define SMEM_DESCRIPTOR 655
/* SMEM host id representing the modem. */
#define QCOM_SMEM_HOST_MODEM 1

struct smem_client_channel {
	struct list_head list;
	u32 id;
	u32 start_loc;
	u32 read_size;
	u32 write_size;
	bool client_connected;
	char irq_name[24];
	smem_mailbox_urgent_cb urgent_cb;
};
static LIST_HEAD(smem_client_channel_list_head);

struct mailbox_smp2p {
	struct qcom_smem_state *smem_state;
	u32 smem_bit;
};
struct mailbox_smp2p *mailbox;
struct device *dev;

struct client_info {
	u32 id;
	u32 start_loc;
	u32 read_size;
	u32 write_size;
};

struct smem_info {
	u32 start_cookie;
	u32 version;
	u32 num_of_clients;
	struct client_info clients[4];
	u32 end_cookie;
};
bool info_validated;

#define TLV_TYPE_TIME 0
#define TLV_TYPE_DATA 1
#define TLV_TYPE_MODULE_MAX 32767
// TLV range of 32,768 - 65,535 is reserved for client use.

struct tlv {
	u16 type;
	u16 length;
};

int free_bytes(u32 head, u32 tail, u32 max) {
	int free_bytes = 0;

	if (head >= tail) {
		free_bytes = max - head + tail;
	} else {
		free_bytes = tail - head;
	}
	return free_bytes;
}

static irqreturn_t smem_read_ready(int irq, void *channel)
{
	int id = 0;
	int pending_bytes = 0;
	void *smem;
	size_t size;
	struct smem_client_channel *entry = NULL;

	id = ((struct smem_client_channel *)channel)->id;

	list_for_each_entry(entry, &smem_client_channel_list_head, list) {
		if (entry->id == id) break;
	}

	if (entry == NULL || entry->id != id) {
		pr_err(KERN_ALERT "smem: ID does not match any smem sub allocation %d\n", id);
		return IRQ_HANDLED;
	}

	if (!entry->client_connected) {
		pr_err(KERN_ALERT "smem: there is no connected client for ID %d, ignore\n", id);
		return IRQ_HANDLED;
	}

	smem = qcom_smem_get(QCOM_SMEM_HOST_MODEM, SMEM_DESCRIPTOR, &size);
	if (IS_ERR(smem)) {
		pr_err("%s: smem qcom_smem_get fail.\n", __func__);
		return IRQ_HANDLED;
	}

	pending_bytes = entry->read_size -
			free_bytes(GET_READ_HEAD(smem, entry), GET_READ_TAIL(smem, entry), entry->read_size);
	entry->urgent_cb(pending_bytes);

	return IRQ_HANDLED;
}

static bool read_client_info_map(void) {
	int ret = 0;
	int irq;
	size_t info_size;
	struct smem_info smem_info;
	struct device_node *node = dev->of_node;
	struct smem_client_channel *entry = NULL;

	void *smem = qcom_smem_get(QCOM_SMEM_HOST_MODEM, SMEM_DESCRIPTOR, &info_size);
	if (IS_ERR(smem)) {
		pr_err("%s: smem qcom_smem_get fail.\n", __func__);
		return -1;
	}

	memcpy_fromio(&smem_info, smem, sizeof(struct smem_info));
	info_validated = false;

	if (smem_info.start_cookie != START_COOKIE) {
		pr_err(KERN_ALERT "smem: start cookie does not match %d\n", smem_info.start_cookie);
		return false;
	}

	if (smem_info.version != SMEM_VERSION) {
		pr_err(KERN_ALERT "smem: version does not match %d\n", smem_info.version);
		return false;
	}

	for (u32 i = 0; i < smem_info.num_of_clients && i < IRQ_NUM_NAMES; i++) {
		struct client_info client = smem_info.clients[i];
		u32 id = client.id;
		u32 start_loc = client.start_loc;
		u32 read_size = client.read_size;
		u32 write_size = client.write_size;

		struct smem_client_channel *new_entry;
		new_entry = kzalloc(sizeof(*new_entry), GFP_KERNEL);
		new_entry->id = id;
		new_entry->start_loc = start_loc;
		new_entry->read_size = read_size;
		new_entry->write_size = write_size;

		// IRQ name is hardcoded
		strscpy(new_entry->irq_name, IRQ_NAMES[i], sizeof(new_entry->irq_name));

		INIT_LIST_HEAD(&new_entry->list);
		list_add_tail(&new_entry->list, &smem_client_channel_list_head);
	}

	if (smem_info.end_cookie != END_COOKIE) {
		pr_err(KERN_ALERT "smem: end cookie does not match %d\n", smem_info.end_cookie);
		return false;
	}
	info_validated = true;

	list_for_each_entry(entry, &smem_client_channel_list_head, list) {
		irq = ret = of_irq_get_byname(node, entry->irq_name);
		if (ret < 0) {
			pr_err("%s: smem platform_get_irq_byname fail. %d %s\n", __func__, ret,
				   entry->irq_name);
			return 1;
		}

		ret = devm_request_threaded_irq(dev, irq, NULL, smem_read_ready,
										IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, entry->irq_name,
										entry);
		if (ret < 0) {
			pr_err("%s: smem request_threaded_irq fail. %d\n", __func__, ret);
			return 1;
		}
	}

	return true;
}

static int smem_probe(struct platform_device *pdev) {
	dev = &pdev->dev;

	if (mailbox == NULL) {
		mailbox = kzalloc(sizeof(struct mailbox_smp2p), GFP_KERNEL);
	}

	mailbox->smem_state = qcom_smem_state_get(dev, "smem-mailbox-smp2p-out", &mailbox->smem_bit);
	if (IS_ERR(mailbox->smem_state)) {
		pr_err("%s: smem fail to get smp2p clk resp bit %ld\n", __func__,
			   PTR_ERR(mailbox->smem_state));
		return 1;
	}

	// Attempt to read client map. This should fail at boot since modem is not up. Retry at first
	// client connection.
	read_client_info_map();
	return 0;
}

static int smem_remove(struct platform_device *pdev) {
	return 0;
}

int smem_mailbox_start(int id, smem_mailbox_urgent_cb urgent_cb) {
	struct smem_client_channel *entry = NULL;

	if (!info_validated && !read_client_info_map()) {
		pr_err(KERN_ALERT "smem: client info map has not been setup by modem.\n");
		return -EAGAIN;
	}

	list_for_each_entry(entry, &smem_client_channel_list_head, list) {
		if (entry->id == id) break;
	}

	if (entry == NULL || entry->id != id) {
		pr_err(KERN_ALERT "smem: ID does not match any smem sub allocation %d\n", id);
		return -EINVAL;
	}

	if (entry->client_connected) {
		pr_err(KERN_ALERT "smem: there is already a client connected to this ID %d\n", id);
		return -EPERM;
	}

	entry->urgent_cb = urgent_cb;
	entry->client_connected = true;
	return 1;
}
EXPORT_SYMBOL(smem_mailbox_start);

int smem_mailbox_stop(int id) {
	void *smem;
	size_t size;
	u32 read_head_index;
	u32 read_tail_index;
	u32 write_head_index;
	u32 write_tail_index;
	struct smem_client_channel *entry = NULL;

	list_for_each_entry(entry, &smem_client_channel_list_head, list) {
		if (entry->id == id) break;
	}

	if (entry == NULL || entry->id != id) {
		pr_err(KERN_ALERT "smem: ID does not match any smem sub allocation %d\n", id);
		return -EINVAL;
	}

	if (!entry->client_connected) {
		pr_err(KERN_ALERT "smem: client never started this ID %d\n", id);
		return -EPERM;
	}

	smem = qcom_smem_get(QCOM_SMEM_HOST_MODEM, SMEM_DESCRIPTOR, &size);
	if (IS_ERR(smem)) {
		pr_err("%s: smem qcom_smem_get fail.\n", __func__);
		return -ENOMEM;
	}

	read_head_index = GET_READ_HEAD(smem, entry);
	read_tail_index = GET_READ_TAIL(smem, entry);
	if (read_head_index != read_tail_index) {
		pr_err("%s: smem read_head_index %d does not equal read_tail_index %d.\n", __func__,
			   read_head_index, read_tail_index);
		UPDATE_READ_TAIL(smem, read_head_index);
	}

	write_head_index = GET_WRITE_HEAD(smem, entry);
	write_tail_index = GET_WRITE_TAIL(smem, entry);
	if (write_head_index != write_tail_index) {
		pr_err("%s: smem write_head_index %d does not equal write_tail_index %d.\n", __func__,
			   write_head_index, write_tail_index);
		UPDATE_WRITE_HEAD(smem, write_tail_index);
	}

	entry->client_connected = false;
	return 1;
}
EXPORT_SYMBOL(smem_mailbox_stop);

bool read_item(struct smem_client_channel *entry, void *item, u16 item_size) {
	int offset;
	void *smem;
	void *read_buf;
	size_t size;
	u32 head_index;
	u32 tail_index;

	smem = qcom_smem_get(QCOM_SMEM_HOST_MODEM, SMEM_DESCRIPTOR, &size);
	if (IS_ERR(smem)) {
		pr_err("%s: smem qcom_smem_get fail.\n", __func__);
		return false;
	}

	head_index = GET_READ_HEAD(smem, entry);
	tail_index = GET_READ_TAIL(smem, entry);
	offset = tail_index + item_size - entry->read_size;
	read_buf = GET_READ_BUF(smem, entry);

	if (offset > 0) {
		if (offset > head_index) {
			pr_err("%s: smem item size is greater than remaining buffer. head %d\n", __func__,
				   head_index);
			tail_index = head_index;
			UPDATE_READ_TAIL(smem, tail_index);
			return false;
		}

		memcpy_fromio(item, read_buf + tail_index, item_size - offset);
		memcpy_fromio(item + (item_size - offset), read_buf, offset);
		UPDATE_READ_TAIL(smem, offset);
	} else {
		if ((tail_index + item_size) > head_index) {
			pr_err("%s: smem item size is greater than remaining buffer. head %d\n", __func__,
				   head_index);
			tail_index = head_index;
			UPDATE_READ_TAIL(smem, tail_index);
			return false;
		}

		memcpy_fromio(item, read_buf + tail_index, item_size);
		tail_index = tail_index + item_size;
		UPDATE_READ_TAIL(smem, tail_index);
	}
	return true;
}

int smem_mailbox_read(int id, u8 **data, u16 *data_length, unsigned long long *xo_time) {
	bool ret;
	size_t size;
	void *smem;
	struct smem_client_channel *entry = NULL;
	u32 head_index;
	u32 tail_index;
	int word_offset;

	list_for_each_entry(entry, &smem_client_channel_list_head, list) {
		if (entry->id == id) break;
	}

	if (entry == NULL || entry->id != id) {
		pr_err(KERN_ALERT "smem: incorrect ID %d\n", id);
		return -EINVAL;
	}

	if (!entry->client_connected) {
		pr_err(KERN_ALERT "smem: client never started this ID %d\n", id);
		return -EPERM;
	}

	smem = qcom_smem_get(QCOM_SMEM_HOST_MODEM, SMEM_DESCRIPTOR, &size);
	if (IS_ERR(smem)) {
		pr_err("%s: smem qcom_smem_get fail.\n", __func__);
		return -ENOMEM;
	}

	head_index = GET_READ_HEAD(smem, entry);
	tail_index = GET_READ_TAIL(smem, entry);

	if (head_index == tail_index) {
		pr_err("%s: smem qcom_smem_get nothing to read for this client %d. %d %d \n", __func__, id,
			   head_index, tail_index);
		return -ENOMSG;
	}

	while (true) {
		struct tlv temp_tlv;
		ret = read_item(entry, &temp_tlv, sizeof(struct tlv));
		if (!ret) goto exit_loop;

		switch (temp_tlv.type) {
			case TLV_TYPE_TIME:
				ret = read_item(entry, xo_time, sizeof(unsigned long long));
				if (!ret) goto exit_loop;
				break;
			case TLV_TYPE_DATA:
				*data_length = temp_tlv.length - sizeof(struct tlv);
				*data = kmalloc(*data_length, GFP_KERNEL);
				ret = read_item(entry, *data, *data_length);
				if (!ret) goto exit_loop;

				// Round to next word.
				word_offset = 4 - (*data_length % 4);
				if (word_offset > 0) {
					tail_index = GET_READ_TAIL(smem, entry) + word_offset;
					UPDATE_READ_TAIL(smem, tail_index);
				}
				goto exit_loop;
				break;
			default:
				pr_err(KERN_ALERT "smem: invalid TLV type %d.\n", temp_tlv.type);
				return -ENOMSG;
				break;
		}
	}
exit_loop:
	return (entry->read_size -
			free_bytes(GET_READ_HEAD(smem, entry), GET_READ_TAIL(smem, entry), entry->read_size));
}
EXPORT_SYMBOL(smem_mailbox_read);

int smem_mailbox_write(int id, int flags, __u8 *data, u16 data_length) {
	size_t size;
	void *smem;
	void *write_buf;
	u32 head_index;
	u32 tail_index;
	u8 *buf;
	struct tlv time_tlv;
	struct tlv data_tlv;
	struct smem_client_channel *entry = NULL;
	unsigned long long xo_time = 0;
	int overflow_bytes;
	int buf_length;
	int word_offset;

	list_for_each_entry(entry, &smem_client_channel_list_head, list) {
		if (entry->id == id) break;
	}

	if (entry == NULL || entry->id != id) {
		pr_err(KERN_ALERT "smem: incorrect ID %d\n", id);
		return -EINVAL;
	}

	if (!entry->client_connected) {
		pr_err(KERN_ALERT "smem: client never started this ID %d\n", id);
		return -EPERM;
	}

	// Size of u32 in subtracted from max size because a full buffer would be the same as an emtpy
	// buffer.
	if (data_length > (entry->write_size - sizeof(u32))) {
		pr_err(KERN_ALERT
			   "smem: data size is larger than write buffer size %d, smem allocation %d\n",
			   data_length, entry->write_size);
		return -ENOSPC;
	}

	smem = qcom_smem_get(QCOM_SMEM_HOST_MODEM, SMEM_DESCRIPTOR, &size);
	if (IS_ERR(smem)) {
		pr_err("%s: smem qcom_smem_get fail.\n", __func__);
		return -ENOMEM;
	}

	head_index = GET_WRITE_HEAD(smem, entry);
	tail_index = GET_WRITE_TAIL(smem, entry);
	buf_length = sizeof(struct tlv) + sizeof(xo_time) + sizeof(struct tlv) + data_length;

	// Not enough space in circular buffer, exit.
	// Subtract u32 from max size so a full buffer does not have head == tail.
	if (head_index > tail_index) {
		if ((tail_index + (entry->write_size - head_index)) < (buf_length - sizeof(u32))) {
			pr_err("%s: smem qcom_smem_get fail, not enough room in write buffer. %d %d\n",
				   __func__, (tail_index - head_index), buf_length);
			return -ENOSPC;
		}
	} else if (head_index != tail_index) {
		if ((tail_index - head_index) < (buf_length - sizeof(u32))) {
			pr_err("%s: smem qcom_smem_get2 fail, not enough room in write buffer. %d %d\n",
				   __func__, (tail_index - head_index), buf_length);
			return -ENOSPC;
		}
	}

	buf = kmalloc(buf_length, GFP_KERNEL);
	if (!buf) {
		pr_err("%s: smem kmalloc fail.\n", __func__);
		return -ENOMEM;
	}

	xo_time = arch_timer_read_cntvct_el0();
	time_tlv.type = TLV_TYPE_TIME;
	time_tlv.length = sizeof(xo_time) + sizeof(struct tlv);
	memcpy(buf, &time_tlv, sizeof(time_tlv));
	memcpy(buf + sizeof(time_tlv), &xo_time, sizeof(xo_time));

	data_tlv.type = TLV_TYPE_DATA;
	data_tlv.length = data_length + sizeof(struct tlv);
	memcpy(buf + sizeof(time_tlv) + sizeof(xo_time), &data_tlv, sizeof(data_tlv));
	memcpy(buf + sizeof(time_tlv) + sizeof(xo_time) + sizeof(data_tlv), data, data_length);

	// Read/Write must always be word aligned for modem.
	word_offset = (~4 & (4 - (buf_length % 4)));
	write_buf = GET_WRITE_BUF(smem, entry);

	overflow_bytes = head_index + buf_length - entry->write_size;
	if (overflow_bytes < 0) {
		memcpy_toio(write_buf + head_index, buf, buf_length);
		head_index = head_index + buf_length + word_offset;
		UPDATE_WRITE_HEAD(smem, head_index);
	} else {
		memcpy_toio(write_buf + head_index, buf, buf_length - overflow_bytes);
		memcpy_toio(write_buf, buf + buf_length - overflow_bytes, overflow_bytes);
		head_index = overflow_bytes + word_offset;
		UPDATE_WRITE_HEAD(smem, head_index);
	}

	if ((flags & FLAG_URGENT) == FLAG_URGENT) {
		if (mailbox->smem_state) {
			// FIFO 1 client ID range starts at 16 (0xF).
			mailbox->smem_bit = mailbox->smem_bit ^ (1 << id);
			qcom_smem_state_update_bits(mailbox->smem_state, 0xff, mailbox->smem_bit);
		} else {
			pr_err("%s: smem smem_state is not setup.\n", __func__);
		}
	}

	return (free_bytes(GET_WRITE_HEAD(smem, entry), GET_WRITE_TAIL(smem, entry), entry->read_size));
}
EXPORT_SYMBOL(smem_mailbox_write);

static const struct of_device_id qcm_smem_match[] = {
	{ .compatible = "qcom,smem_mailbox", },
	{},
};
MODULE_DEVICE_TABLE(of, qcm_smem_match);

static struct platform_driver qcom_smem_driver = {
	.probe = smem_probe,
	.remove = smem_remove,
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = qcm_smem_match,
	},
};
module_platform_driver(qcom_smem_driver);

MODULE_DESCRIPTION("QCOM SMEM Mailbox Driver");
MODULE_LICENSE("GPL v2");
