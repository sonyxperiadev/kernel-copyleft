// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */
#define pr_fmt(fmt) "%s:%s: " fmt, KBUILD_MODNAME, __func__

#include <linux/module.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/platform_device.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <asm/memory.h>
#include <linux/sizes.h>

#include <linux/hwspinlock.h>

#include <linux/sysfs.h>

#include "ipclite_client.h"
#include "ipclite.h"

static struct ipclite_info *ipclite;
static struct ipclite_client synx_client;
static struct ipclite_client test_client;
static struct ipclite_debug_info *ipclite_dbg_info;
static struct ipclite_debug_struct *ipclite_dbg_struct;
static struct ipclite_debug_inmem_buf *ipclite_dbg_inmem;
static struct mutex ssr_mutex;
static struct kobject *sysfs_kobj;

static uint32_t ipclite_debug_level = IPCLITE_ERR | IPCLITE_WARN | IPCLITE_INFO;
static uint32_t ipclite_debug_control = IPCLITE_DMESG_LOG, ipclite_debug_dump;
static uint32_t enabled_hosts, partitions, major_ver, minor_ver;
static uint64_t feature_mask;

static inline bool is_host_enabled(uint32_t host)
{
	return (1U & (enabled_hosts >> host));
}

static inline bool is_loopback_except_apps(uint32_t h0, uint32_t h1)
{
	return (h0 == h1 && h0 != IPCMEM_APPS);
}

static void ipclite_inmem_log(const char *psztStr, ...)
{
	uint32_t local_index = 0;
	va_list pArgs;

	va_start(pArgs, psztStr);

	/* Incrementing the index atomically and storing the index in local variable */
	local_index = ipclite_global_atomic_inc((ipclite_atomic_int32_t *)
							&ipclite_dbg_info->debug_log_index);
	local_index %= IPCLITE_LOG_BUF_SIZE;

	/* Writes data on the index location */
	vsnprintf(ipclite_dbg_inmem->IPCLITELog[local_index], IPCLITE_LOG_MSG_SIZE, psztStr, pArgs);

	va_end(pArgs);
}

static void ipclite_dump_debug_struct(void)
{
	int i = 0, host = 0;
	struct ipclite_debug_struct *temp_dbg_struct;

	/* Check if debug structures are initialized */
	if (!ipclite_dbg_info || !ipclite_dbg_struct) {
		pr_err("Debug Structures not initialized\n");
		return;
	}

	/* Check if debug structures are enabled before printing */
	if (!(IS_DEBUG_CONFIG(IPCLITE_DBG_STRUCT))) {
		pr_err("Debug Structures not enabled\n");
		return;
	}

	/* Dumping the debug structures */
	pr_info("------------------- Dumping IPCLite Debug Structure -------------------\n");

	for (host = 0; host < IPCMEM_NUM_HOSTS; host++) {
		if (!is_host_enabled(host))
			continue;
		temp_dbg_struct = (struct ipclite_debug_struct *)
					(((char *)ipclite_dbg_struct) +
					(sizeof(*temp_dbg_struct) * host));

		pr_info("---------- Host ID: %d dbg_mem:%p ----------\n",
				host, temp_dbg_struct);
		pr_info("Total Signals Sent : %d Total Signals Received : %d\n",
				temp_dbg_struct->dbg_info_overall.total_numsig_sent,
				temp_dbg_struct->dbg_info_overall.total_numsig_recv);
		pr_info("Last Signal Sent to Host ID : %d Last Signal Received from Host ID : %d\n",
				temp_dbg_struct->dbg_info_overall.last_sent_host_id,
				temp_dbg_struct->dbg_info_overall.last_recv_host_id);
		pr_info("Last Signal ID Sent : %d Last Signal ID Received : %d\n",
				temp_dbg_struct->dbg_info_overall.last_sigid_sent,
				temp_dbg_struct->dbg_info_overall.last_sigid_recv);

		for (i = 0; i < IPCMEM_NUM_HOSTS; i++) {
			if (!is_host_enabled(i))
				continue;
			pr_info("----------> Host ID : %d Host ID : %d\n", host, i);
			pr_info("No. of Messages Sent : %d No. of Messages Received : %d\n",
			temp_dbg_struct->dbg_info_host[i].numsig_sent,
			temp_dbg_struct->dbg_info_host[i].numsig_recv);
			pr_info("No. of Interrupts Received : %d\n",
			temp_dbg_struct->dbg_info_host[i].num_intr);
			pr_info("TX Write Index : %d TX Read Index : %d\n",
			temp_dbg_struct->dbg_info_host[i].tx_wr_index,
			temp_dbg_struct->dbg_info_host[i].tx_rd_index);
			pr_info("TX Write Index[0] : %d TX Read Index[0] : %d\n",
			temp_dbg_struct->dbg_info_host[i].prev_tx_wr_index[0],
			temp_dbg_struct->dbg_info_host[i].prev_tx_rd_index[0]);
			pr_info("TX Write Index[1] : %d TX Read Index[1] : %d\n",
			temp_dbg_struct->dbg_info_host[i].prev_tx_wr_index[1],
			temp_dbg_struct->dbg_info_host[i].prev_tx_rd_index[1]);
			pr_info("RX Write Index : %d RX Read Index : %d\n",
			temp_dbg_struct->dbg_info_host[i].rx_wr_index,
			temp_dbg_struct->dbg_info_host[i].rx_rd_index);
			pr_info("RX Write Index[0] : %d RX Read Index[0] : %d\n",
			temp_dbg_struct->dbg_info_host[i].prev_rx_wr_index[0],
			temp_dbg_struct->dbg_info_host[i].prev_rx_rd_index[0]);
			pr_info("RX Write Index[1] : %d RX Read Index[1] : %d\n",
			temp_dbg_struct->dbg_info_host[i].prev_rx_wr_index[1],
			temp_dbg_struct->dbg_info_host[i].prev_rx_rd_index[1]);
		}
	}
	return;
}

static void ipclite_dump_inmem_logs(void)
{
	int i = 0;
	uint32_t local_index = 0;

	/* Check if debug and inmem structures are initialized */
	if (!ipclite_dbg_info || !ipclite_dbg_inmem) {
		pr_err("Debug structures not initialized\n");
		return;
	}

	/* Check if debug structures are enabled before printing */
	if (!(IS_DEBUG_CONFIG(IPCLITE_INMEM_LOG))) {
		pr_err("In-Memory Logs not enabled\n");
		return;
	}

	/* Dumping the debug in-memory logs */
	pr_info("------------------- Dumping In-Memory Logs -------------------\n");

	/* Storing the index atomically in local variable */
	local_index = ipclite_global_atomic_load_u32((ipclite_atomic_uint32_t *)
							&ipclite_dbg_info->debug_log_index);

	/* Printing from current index till the end of buffer */
	for (i = local_index % IPCLITE_LOG_BUF_SIZE; i < IPCLITE_LOG_BUF_SIZE; i++) {
		if (ipclite_dbg_inmem->IPCLITELog[i][0])
			pr_info("%s\n", ipclite_dbg_inmem->IPCLITELog[i]);
	}

	/* Printing from 0th index to current-1 index */
	for (i = 0; i < local_index % IPCLITE_LOG_BUF_SIZE; i++) {
		if (ipclite_dbg_inmem->IPCLITELog[i][0])
			pr_info("%s\n", ipclite_dbg_inmem->IPCLITELog[i]);
	}

	return;
}

int ipclite_hw_mutex_acquire(void)
{
	int ret;

	if (unlikely(!ipclite)) {
		pr_err("IPCLite not initialized\n");
		return -ENOMEM;
	}
	ret = hwspin_lock_timeout_irqsave(ipclite->hwlock,
					HWSPINLOCK_TIMEOUT, &ipclite->hw_mutex_flags);
	if (ret) {
		pr_err("Hw mutex lock acquire failed\n");
		return ret;
	}
	ipclite->ipcmem.toc_data.host_info->hwlock_owner = IPCMEM_APPS;
	return ret;
}
EXPORT_SYMBOL(ipclite_hw_mutex_acquire);

int ipclite_hw_mutex_release(void)
{
	if (unlikely(!ipclite)) {
		pr_err("IPCLite not initialized\n");
		return -ENOMEM;
	}
	if (ipclite->ipcmem.toc_data.host_info->hwlock_owner != IPCMEM_APPS)
		return -EINVAL;

	ipclite->ipcmem.toc_data.host_info->hwlock_owner = IPCMEM_INVALID_HOST;
	hwspin_unlock_irqrestore(ipclite->hwlock, &ipclite->hw_mutex_flags);
	return 0;
}
EXPORT_SYMBOL(ipclite_hw_mutex_release);

/* Atomic Functions Start */
void ipclite_atomic_init_u32(ipclite_atomic_uint32_t *addr, uint32_t data)
{
	BUG_ON(addr == NULL);

	atomic_set(addr, data);
}
EXPORT_SYMBOL(ipclite_atomic_init_u32);

void ipclite_atomic_init_i32(ipclite_atomic_int32_t *addr, int32_t data)
{
	BUG_ON(addr == NULL);

	atomic_set(addr, data);
}
EXPORT_SYMBOL(ipclite_atomic_init_i32);

void ipclite_global_atomic_store_u32(ipclite_atomic_uint32_t *addr, uint32_t data)
{
	BUG_ON(addr == NULL);

	ATOMIC_HW_MUTEX_ACQUIRE;

	atomic_set(addr, data);

	ATOMIC_HW_MUTEX_RELEASE;
}
EXPORT_SYMBOL(ipclite_global_atomic_store_u32);

void ipclite_global_atomic_store_i32(ipclite_atomic_int32_t *addr, int32_t data)
{
	BUG_ON(addr == NULL);

	ATOMIC_HW_MUTEX_ACQUIRE;

	atomic_set(addr, data);

	ATOMIC_HW_MUTEX_RELEASE;
}
EXPORT_SYMBOL(ipclite_global_atomic_store_i32);

uint32_t ipclite_global_atomic_load_u32(ipclite_atomic_uint32_t *addr)
{
	uint32_t ret = 0;

	BUG_ON(addr == NULL);

	ATOMIC_HW_MUTEX_ACQUIRE;

	ret = atomic_read(addr);

	ATOMIC_HW_MUTEX_RELEASE;

	return ret;
}
EXPORT_SYMBOL(ipclite_global_atomic_load_u32);

int32_t ipclite_global_atomic_load_i32(ipclite_atomic_int32_t *addr)
{
	int32_t ret = 0;

	BUG_ON(addr == NULL);

	ATOMIC_HW_MUTEX_ACQUIRE;

	ret = atomic_read(addr);

	ATOMIC_HW_MUTEX_RELEASE;

	return ret;
}
EXPORT_SYMBOL(ipclite_global_atomic_load_i32);

uint32_t ipclite_global_test_and_set_bit(uint32_t nr, ipclite_atomic_uint32_t *addr)
{
	uint32_t ret = 0, mask = (1 << nr);

	BUG_ON(addr == NULL);

	ATOMIC_HW_MUTEX_ACQUIRE;

	ret = atomic_fetch_or(mask, addr);

	ATOMIC_HW_MUTEX_RELEASE;

	return ret;
}
EXPORT_SYMBOL(ipclite_global_test_and_set_bit);

uint32_t ipclite_global_test_and_clear_bit(uint32_t nr, ipclite_atomic_uint32_t *addr)
{
	uint32_t ret = 0, mask = (1 << nr);

	BUG_ON(addr == NULL);

	ATOMIC_HW_MUTEX_ACQUIRE;

	ret = atomic_fetch_and(~mask, addr);

	ATOMIC_HW_MUTEX_RELEASE;

	return ret;
}
EXPORT_SYMBOL(ipclite_global_test_and_clear_bit);

int32_t ipclite_global_atomic_inc(ipclite_atomic_int32_t *addr)
{
	int32_t ret = 0;

	BUG_ON(addr == NULL);

	ATOMIC_HW_MUTEX_ACQUIRE;

	ret = atomic_fetch_add(1, addr);

	ATOMIC_HW_MUTEX_RELEASE;

	return ret;
}
EXPORT_SYMBOL(ipclite_global_atomic_inc);

int32_t ipclite_global_atomic_dec(ipclite_atomic_int32_t *addr)
{
	int32_t ret = 0;

	BUG_ON(addr == NULL);

	ATOMIC_HW_MUTEX_ACQUIRE;

	ret = atomic_fetch_sub(1, addr);

	ATOMIC_HW_MUTEX_RELEASE;

	return ret;
}
EXPORT_SYMBOL(ipclite_global_atomic_dec);
/* Atomic Functions End */

static size_t ipcmem_rx_avail(struct ipclite_fifo *rx_fifo)
{
	size_t len = 0;
	u32 head = 0, tail = 0;

	head = le32_to_cpu(*rx_fifo->head);
	tail = le32_to_cpu(*rx_fifo->tail);

	IPCLITE_OS_LOG(IPCLITE_DBG, "head=%d, tail=%d\n", head, tail);

	if (head < tail)
		len = rx_fifo->length - tail + head;
	else
		len = head - tail;

	if (WARN_ON_ONCE(len > rx_fifo->length))
		len = 0;

	IPCLITE_OS_LOG(IPCLITE_DBG, "len=%d\n", len);

	return len;
}

static void ipcmem_rx_peak(struct ipclite_fifo *rx_fifo,
			       void *data, size_t count)
{
	size_t len = 0;
	u32 tail = 0;

	tail = le32_to_cpu(*rx_fifo->tail);

	if (WARN_ON_ONCE(tail > rx_fifo->length))
		return;

	if (tail >= rx_fifo->length)
		tail -= rx_fifo->length;

	len = min_t(size_t, count, rx_fifo->length - tail);
	if (len)
		memcpy_fromio(data, rx_fifo->fifo + tail, len);

	if (len != count)
		memcpy_fromio(data + len, rx_fifo->fifo, (count - len));
}

static void ipcmem_rx_advance(struct ipclite_fifo *rx_fifo,
				  size_t count, uint32_t core_id)
{
	u32 tail = 0;

	tail = le32_to_cpu(*rx_fifo->tail);

	tail += count;
	if (tail >= rx_fifo->length)
		tail %= rx_fifo->length;

	*rx_fifo->tail = cpu_to_le32(tail);

	/* Storing the debug data in debug structures */
	if (IS_DEBUG_CONFIG(IPCLITE_DBG_STRUCT)) {
		ipclite_dbg_struct->dbg_info_host[core_id].prev_rx_wr_index[1] =
				ipclite_dbg_struct->dbg_info_host[core_id].prev_rx_wr_index[0];
		ipclite_dbg_struct->dbg_info_host[core_id].prev_rx_wr_index[0] =
				ipclite_dbg_struct->dbg_info_host[core_id].rx_wr_index;
		ipclite_dbg_struct->dbg_info_host[core_id].rx_wr_index = *rx_fifo->head;

		ipclite_dbg_struct->dbg_info_host[core_id].prev_rx_rd_index[1] =
				ipclite_dbg_struct->dbg_info_host[core_id].prev_rx_rd_index[0];
		ipclite_dbg_struct->dbg_info_host[core_id].prev_rx_rd_index[0] =
				ipclite_dbg_struct->dbg_info_host[core_id].rx_rd_index;
		ipclite_dbg_struct->dbg_info_host[core_id].rx_rd_index = *rx_fifo->tail;

		ipclite_dbg_struct->dbg_info_overall.total_numsig_recv++;
		ipclite_dbg_struct->dbg_info_host[core_id].numsig_recv++;
	}
}

static size_t ipcmem_tx_avail(struct ipclite_fifo *tx_fifo)
{
	u32 head = 0, tail = 0, avail = 0;

	head = le32_to_cpu(*tx_fifo->head);
	tail = le32_to_cpu(*tx_fifo->tail);

	if (tail <= head)
		avail = tx_fifo->length - head + tail;
	else
		avail = tail - head;

	if (avail < FIFO_FULL_RESERVE)
		avail = 0;
	else
		avail -= FIFO_FULL_RESERVE;

	if (WARN_ON_ONCE(avail > tx_fifo->length))
		avail = 0;

	return avail;
}

static unsigned int ipcmem_tx_write_one(struct ipclite_fifo *tx_fifo,
					    unsigned int head,
					    const void *data, size_t count)
{
	size_t len = 0;

	if (WARN_ON_ONCE(head > tx_fifo->length))
		return head;

	len = min_t(size_t, count, tx_fifo->length - head);
	if (len)
		memcpy(tx_fifo->fifo + head, data, len);

	if (len != count)
		memcpy(tx_fifo->fifo, data + len, count - len);

	head += count;
	if (head >= tx_fifo->length)
		head -= tx_fifo->length;

	return head;
}

static void ipcmem_tx_write(struct ipclite_fifo *tx_fifo,
			const void *data, size_t dlen, uint32_t core_id, uint32_t signal_id)
{
	unsigned int head = 0;

	head = le32_to_cpu(*tx_fifo->head);
	head = ipcmem_tx_write_one(tx_fifo, head, data, dlen);

	head = ALIGN(head, 8);
	if (head >= tx_fifo->length)
		head -= tx_fifo->length;

	/* Ensure ordering of fifo and head update */
	wmb();

	*tx_fifo->head = cpu_to_le32(head);

	IPCLITE_OS_LOG(IPCLITE_DBG, "head : %d core_id : %d signal_id : %d\n",
						*tx_fifo->head, core_id, signal_id);

	/* Storing the debug data in debug structures */
	if (IS_DEBUG_CONFIG(IPCLITE_DBG_STRUCT)) {
		ipclite_dbg_struct->dbg_info_host[core_id].prev_tx_wr_index[1] =
				ipclite_dbg_struct->dbg_info_host[core_id].prev_tx_wr_index[0];
		ipclite_dbg_struct->dbg_info_host[core_id].prev_tx_wr_index[0] =
				ipclite_dbg_struct->dbg_info_host[core_id].tx_wr_index;
		ipclite_dbg_struct->dbg_info_host[core_id].tx_wr_index = *tx_fifo->head;

		ipclite_dbg_struct->dbg_info_host[core_id].prev_tx_rd_index[1] =
				ipclite_dbg_struct->dbg_info_host[core_id].prev_tx_rd_index[0];
		ipclite_dbg_struct->dbg_info_host[core_id].prev_tx_rd_index[0] =
				ipclite_dbg_struct->dbg_info_host[core_id].tx_rd_index;
		ipclite_dbg_struct->dbg_info_host[core_id].tx_rd_index = *tx_fifo->tail;

		ipclite_dbg_struct->dbg_info_overall.total_numsig_sent++;
		ipclite_dbg_struct->dbg_info_host[core_id].numsig_sent++;
		ipclite_dbg_struct->dbg_info_overall.last_sent_host_id = core_id;
		ipclite_dbg_struct->dbg_info_overall.last_sigid_sent = signal_id;
	}
}

static size_t ipclite_rx_avail(struct ipclite_channel *channel)
{
	return channel->rx_fifo->avail(channel->rx_fifo);
}

static void ipclite_rx_peak(struct ipclite_channel *channel,
			       void *data, size_t count)
{
	channel->rx_fifo->peak(channel->rx_fifo, data, count);
}

static void ipclite_rx_advance(struct ipclite_channel *channel,
					size_t count)
{
	channel->rx_fifo->advance(channel->rx_fifo, count, channel->remote_pid);
}

static size_t ipclite_tx_avail(struct ipclite_channel *channel)
{
	return channel->tx_fifo->avail(channel->tx_fifo);
}

static void ipclite_tx_write(struct ipclite_channel *channel,
				const void *data, size_t dlen)
{
	channel->tx_fifo->write(channel->tx_fifo, data, dlen, channel->remote_pid,
								channel->irq_info->signal_id);
}

static int ipclite_rx_data(struct ipclite_channel *channel, size_t avail)
{
	int ret = 0;
	uint64_t data = 0;

	if (avail < sizeof(data)) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Not enough data in fifo, Core : %d Signal : %d\n",
						channel->remote_pid, channel->irq_info->signal_id);
		return -EAGAIN;
	}

	ipclite_rx_peak(channel, &data, sizeof(data));

	if (synx_client.reg_complete == 1) {
		if (synx_client.callback)
			synx_client.callback(channel->remote_pid, data,
								synx_client.priv_data);
	}
	ipclite_rx_advance(channel, ALIGN(sizeof(data), 8));
	return ret;
}

static int ipclite_rx_test_data(struct ipclite_channel *channel, size_t avail)
{
	int ret = 0;
	uint64_t data = 0;

	if (avail < sizeof(data)) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Not enough data in fifo, Core : %d Signal : %d\n",
						channel->remote_pid, channel->irq_info->signal_id);
		return -EAGAIN;
	}

	ipclite_rx_peak(channel, &data, sizeof(data));

	if (test_client.reg_complete == 1) {
		if (test_client.callback)
			test_client.callback(channel->remote_pid, data,
								test_client.priv_data);
	}
	ipclite_rx_advance(channel, ALIGN(sizeof(data), 8));
	return ret;
}

static irqreturn_t ipclite_intr(int irq, void *data)
{
	int ret = 0;
	unsigned int avail = 0;
	uint64_t msg = 0;
	struct ipclite_channel *channel;
	struct ipclite_irq_info *irq_info;

	irq_info = (struct ipclite_irq_info *)data;
	channel = container_of(irq_info, struct ipclite_channel, irq_info[irq_info->signal_id]);

	IPCLITE_OS_LOG(IPCLITE_DBG, "Interrupt received from Core : %d Signal : %d\n",
							channel->remote_pid, irq_info->signal_id);

	/* Storing the debug data in debug structures */
	if (IS_DEBUG_CONFIG(IPCLITE_DBG_STRUCT)) {
		ipclite_dbg_struct->dbg_info_host[channel->remote_pid].num_intr++;
		ipclite_dbg_struct->dbg_info_overall.last_recv_host_id = channel->remote_pid;
		ipclite_dbg_struct->dbg_info_overall.last_sigid_recv = irq_info->signal_id;
	}

	if (irq_info->signal_id == IPCLITE_MSG_SIGNAL) {
		for (;;) {
			avail = ipclite_rx_avail(channel);
			if (avail < sizeof(msg))
				break;

			ret = ipclite_rx_data(channel, avail);
		}
		IPCLITE_OS_LOG(IPCLITE_DBG, "checking messages in rx_fifo done\n");
	} else if (irq_info->signal_id == IPCLITE_VERSION_SIGNAL) {
		IPCLITE_OS_LOG(IPCLITE_DBG, "Versioning is not enabled using IPCC signals\n");
	} else if (irq_info->signal_id == IPCLITE_TEST_SIGNAL) {
		for (;;) {
			avail = ipclite_rx_avail(channel);
			if (avail < sizeof(msg))
				break;

			ret = ipclite_rx_test_data(channel, avail);
		}
		IPCLITE_OS_LOG(IPCLITE_DBG, "checking messages in rx_fifo done\n");
	} else {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Wrong Interrupt Signal from core : %d signal : %d\n",
							channel->remote_pid, irq_info->signal_id);
	}

	return IRQ_HANDLED;
}

static int ipclite_tx(struct ipclite_channel *channel,
			uint64_t data, size_t dlen, uint32_t ipclite_signal)
{
	int ret = 0;
	unsigned long flags;

	if (channel->status != ACTIVE) {
		if (channel->status == IN_PROGRESS && *channel->gstatus_ptr == ACTIVE) {
			channel->status = ACTIVE;
		} else {
			IPCLITE_OS_LOG(IPCLITE_ERR, "Cannot Send, Channel not active\n");
			return -EOPNOTSUPP;
		}
	}

	spin_lock_irqsave(&channel->tx_lock, flags);
	if (ipclite_tx_avail(channel) < dlen) {
		spin_unlock_irqrestore(&channel->tx_lock, flags);
		ret = -EAGAIN;
		return ret;
	}

	ipclite_tx_write(channel, &data, dlen);

	mbox_send_message(channel->irq_info[ipclite_signal].mbox_chan, NULL);
	mbox_client_txdone(channel->irq_info[ipclite_signal].mbox_chan, 0);

	spin_unlock_irqrestore(&channel->tx_lock, flags);

	return ret;
}

static int ipclite_notify_core(int32_t proc_id, int32_t signal_id)
{
	int ret = 0;
	struct ipclite_channel *channel;

	if (proc_id < 0 || proc_id >= IPCMEM_NUM_HOSTS) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Invalid proc_id : %d\n", proc_id);
		return -EINVAL;
	}
	channel = &ipclite->channel[proc_id];

	if (channel->status != ACTIVE) {
		if (channel->status == IN_PROGRESS && *channel->gstatus_ptr == ACTIVE) {
			channel->status = ACTIVE;
		} else {
			IPCLITE_OS_LOG(IPCLITE_ERR, "Cannot Send, Core %d is Inactive\n", proc_id);
			return -EOPNOTSUPP;
		}
	}

	ret = mbox_send_message(channel->irq_info[signal_id].mbox_chan, NULL);
	if (ret < 0) {
		IPCLITE_OS_LOG(IPCLITE_ERR,
				"Signal sending failed to Core : %d Signal : %d ret : %d\n",
									proc_id, signal_id, ret);
		return ret;
	}

	IPCLITE_OS_LOG(IPCLITE_DBG,
			"Signal send completed to core : %d signal : %d ret : %d\n",
									proc_id, signal_id, ret);
	return 0;
}

static int map_ipcmem(struct ipclite_info *ipclite, const char *name)
{
	int ret = 0;
	struct device *dev;
	struct device_node *np;
	struct resource r;

	dev = ipclite->dev;

	np = of_parse_phandle(dev->of_node, name, 0);
	if (!np) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "No %s specified\n", name);
		return -EINVAL;
	}

	ret = of_address_to_resource(np, 0, &r);
	of_node_put(np);
	if (ret)
		return ret;

	ipclite->ipcmem.mem.aux_base = (u64)r.start;
	ipclite->ipcmem.mem.size = resource_size(&r);
	ipclite->ipcmem.mem.virt_base = devm_ioremap_wc(dev, r.start,
					resource_size(&r));
	if (!ipclite->ipcmem.mem.virt_base)
		return -ENOMEM;

	IPCLITE_OS_LOG(IPCLITE_DBG, "aux_base = %lx, size=%d,virt_base=%p\n",
			ipclite->ipcmem.mem.aux_base, ipclite->ipcmem.mem.size,
			ipclite->ipcmem.mem.virt_base);

	return ret;
}

/**
 * insert_magic_number() - Inserts the magic number in toc header
 *
 * Function computes a simple checksum of the contents in toc header
 * and stores the result in magic_number field in the toc header
 */
static void insert_magic_number(void)
{
	uint32_t *block = ipclite->ipcmem.mem.virt_base;
	size_t size = sizeof(struct ipcmem_toc_header) / sizeof(uint32_t);

	for (int i = 1; i < size; i++)
		block[0] ^= block[i];

	block[0] = ~block[0];
}

static int32_t setup_toc(struct ipclite_mem *ipcmem)
{
	size_t offset = 0;
	void *virt_base = ipcmem->mem.virt_base;
	struct ipcmem_offsets *offsets = &ipcmem->toc->offsets;
	struct ipcmem_toc_data *toc_data = &ipcmem->toc_data;

	/* Setup Offsets */
	offsets->host_info		= offset += IPCMEM_TOC_VAR_OFFSET;
	offsets->global_entry		= offset += sizeof(struct ipcmem_host_info);
	offsets->partition_info		= offset += sizeof(struct ipcmem_partition_entry);
	offsets->partition_entry	= offset += sizeof(struct ipcmem_partition_info);
	// offsets->debug		= virt_base + size - 64K;
	/* Offset to be used for any new structure added in toc (after partition_entry) */
	// offsets->new_struct	= offset += sizeof(struct ipcmem_partition_entry)*IPCMEM_NUM_HOSTS;

	IPCLITE_OS_LOG(IPCLITE_DBG, "toc_data offsets:");
	IPCLITE_OS_LOG(IPCLITE_DBG, "host_info = 0x%X", offsets->host_info);
	IPCLITE_OS_LOG(IPCLITE_DBG, "global_entry = 0x%X", offsets->global_entry);
	IPCLITE_OS_LOG(IPCLITE_DBG, "partition_info = 0x%X", offsets->partition_info);
	IPCLITE_OS_LOG(IPCLITE_DBG, "partition_entry = 0x%X", offsets->partition_entry);

	/* Point structures to the appropriate offset in TOC */
	toc_data->host_info		= ADD_OFFSET(virt_base, offsets->host_info);
	toc_data->global_entry		= ADD_OFFSET(virt_base, offsets->global_entry);
	toc_data->partition_info	= ADD_OFFSET(virt_base, offsets->partition_info);
	toc_data->partition_entry	= ADD_OFFSET(virt_base, offsets->partition_entry);

	return 0;
}

static void setup_global_partition(struct ipclite_mem *ipcmem, uint32_t base_offset)
{
	/*Fill in global partition details*/
	ipcmem->toc_data.global_entry->base_offset = base_offset;
	ipcmem->toc_data.global_entry->size = GLOBAL_PARTITION_SIZE;
	ipcmem->toc_data.global_entry->flags = GLOBAL_PARTITION_FLAGS;
	ipcmem->toc_data.global_entry->host0 = IPCMEM_GLOBAL_HOST;
	ipcmem->toc_data.global_entry->host1 = IPCMEM_GLOBAL_HOST;

	ipcmem->global_partition = ADD_OFFSET(ipcmem->mem.virt_base, base_offset);

	IPCLITE_OS_LOG(IPCLITE_DBG, "base_offset =%x,ipcmem->global_partition = %p\n",
				base_offset,
				ipcmem->global_partition);

	ipcmem->global_partition->hdr = global_partition_hdr;

	IPCLITE_OS_LOG(IPCLITE_DBG, "hdr.type = %x,hdr.offset = %x,hdr.size = %d\n",
				ipcmem->global_partition->hdr.partition_type,
				ipcmem->global_partition->hdr.region_offset,
				ipcmem->global_partition->hdr.region_size);
}

static void update_partition(struct ipclite_mem *ipcmem, uint32_t p)
{
	int host0 = ipcmem->toc_data.partition_entry[p].host0;
	int host1 = ipcmem->toc_data.partition_entry[p].host1;

	IPCLITE_OS_LOG(IPCLITE_DBG, "host0 = %d, host1=%d\n", host0, host1);

	ipcmem->partition[p] = ADD_OFFSET(ipcmem->mem.virt_base,
					ipcmem->toc_data.partition_entry[p].base_offset);

	IPCLITE_OS_LOG(IPCLITE_DBG, "partition[%d] = %p,partition_base_offset[%d]=%lx",
				p, ipcmem->partition[p],
				p, ipcmem->toc_data.partition_entry[p].base_offset);

	if (host0 == host1)
		ipcmem->partition[p]->hdr = loopback_partition_hdr;
	else
		ipcmem->partition[p]->hdr = default_partition_hdr;

	IPCLITE_OS_LOG(IPCLITE_DBG, "hdr.type = %x,hdr.offset = %x,hdr.size = %d",
				ipcmem->partition[p]->hdr.type,
				ipcmem->partition[p]->hdr.desc_offset,
				ipcmem->partition[p]->hdr.desc_size);
}

static int32_t setup_partitions(struct ipclite_mem *ipcmem, uint32_t base_offset)
{
	uint32_t p, host0, host1;
	uint32_t num_entry = 0;

	/*Fill in each valid ipcmem partition table entry*/
	for (host0 = 0; host0 < IPCMEM_NUM_HOSTS; host0++) {
		if (!is_host_enabled(host0))
			continue;
		for (host1 = host0; host1 < IPCMEM_NUM_HOSTS; host1++) {
			if (!is_host_enabled(host1) || is_loopback_except_apps(host0, host1))
				continue;
			ipcmem->toc_data.partition_entry[num_entry].base_offset = base_offset;
			ipcmem->toc_data.partition_entry[num_entry].size = DEFAULT_PARTITION_SIZE;
			ipcmem->toc_data.partition_entry[num_entry].flags = DEFAULT_PARTITION_FLAGS;
			ipcmem->toc_data.partition_entry[num_entry].host0 = host0;
			ipcmem->toc_data.partition_entry[num_entry].host1 = host1;

			base_offset += DEFAULT_PARTITION_SIZE;
			num_entry++;
		}
	}
	IPCLITE_OS_LOG(IPCLITE_DBG, "total partitions = %u", num_entry);

	ipcmem->partition = kcalloc(num_entry, sizeof(*ipcmem->partition), GFP_KERNEL);
	if (!ipcmem->partition) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Partition Allocation failed");
		return -ENOMEM;
	}

	/*Update appropriate partition based on partition entries*/
	for (p = 0; p < num_entry; p++)
		update_partition(ipcmem, p);

	/*Set up info to parse partition entries*/
	ipcmem->toc_data.partition_info->num_entries = partitions = num_entry;
	ipcmem->toc_data.partition_info->entry_size = sizeof(struct ipcmem_partition_entry);

	return 0;
}

static int32_t ipcmem_init(struct ipclite_mem *ipcmem, struct device_node *pn)
{
	int ret = 0;
	uint32_t remote_pid = 0, host_count = 0, gmem_offset = 0;
	struct device_node *cn;

	for_each_available_child_of_node(pn, cn) {
		of_property_read_u32(cn, "qcom,remote-pid", &remote_pid);
		if (remote_pid < IPCMEM_NUM_HOSTS) {
			enabled_hosts |= BIT_MASK(remote_pid);
			host_count++;
		}
	}
	IPCLITE_OS_LOG(IPCLITE_DBG, "enabled_hosts = 0x%X", enabled_hosts);
	IPCLITE_OS_LOG(IPCLITE_DBG, "host_count = %u", host_count);

	ipcmem->toc = ipcmem->mem.virt_base;
	IPCLITE_OS_LOG(IPCLITE_DBG, "toc_base = %p\n", ipcmem->toc);

	ret = setup_toc(ipcmem);
	if (ret) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Failed to set up toc");
		return ret;
	}

	/*Set up host related info*/
	ipcmem->toc_data.host_info->hwlock_owner = IPCMEM_INVALID_HOST;
	ipcmem->toc_data.host_info->configured_host = enabled_hosts;

	gmem_offset += IPCMEM_TOC_SIZE;
	setup_global_partition(ipcmem, gmem_offset);

	gmem_offset += GLOBAL_PARTITION_SIZE;
	ret = setup_partitions(ipcmem, gmem_offset);
	if (ret) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Failed to set up partitions");
		return ret;
	}

	/*Making sure all writes for ipcmem configurations are completed*/
	wmb();

	ipcmem->toc->hdr.init_done = IPCMEM_INIT_COMPLETED;
	IPCLITE_OS_LOG(IPCLITE_DBG, "Ipcmem init completed\n");

	return 0;
}

static int ipclite_channel_irq_init(struct device *parent, struct device_node *node,
								struct ipclite_channel *channel)
{
	int ret = 0;
	u32 index = 0;
	struct ipclite_irq_info *irq_info;
	struct device *dev;
	char strs[MAX_CHANNEL_SIGNALS][IPCLITE_SIGNAL_LABEL_SIZE] = {
			"msg", "mem-init", "version", "test", "ssr", "debug"};

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->parent = parent;
	dev->of_node = node;
	dev_set_name(dev, "%s:%pOFn", dev_name(parent->parent), node);
	IPCLITE_OS_LOG(IPCLITE_DBG, "Registering %s device\n", dev_name(parent->parent));
	ret = device_register(dev);
	if (ret) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "failed to register ipclite child node\n");
		put_device(dev);
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "index", &index);
	if (ret) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "failed to parse index\n");
		goto err_dev;
	}

	irq_info = &channel->irq_info[index];
	IPCLITE_OS_LOG(IPCLITE_DBG, "irq_info[%d]=%p\n", index, irq_info);

	irq_info->mbox_client.dev = dev;
	irq_info->mbox_client.knows_txdone = true;
	irq_info->mbox_chan = mbox_request_channel(&irq_info->mbox_client, 0);
	IPCLITE_OS_LOG(IPCLITE_DBG, "irq_info[%d].mbox_chan=%p\n", index, irq_info->mbox_chan);
	if (IS_ERR(irq_info->mbox_chan)) {
		if (PTR_ERR(irq_info->mbox_chan) != -EPROBE_DEFER)
			IPCLITE_OS_LOG(IPCLITE_ERR, "failed to acquire IPC channel\n");
		goto err_dev;
	}

	snprintf(irq_info->irqname, 32, "ipclite-signal-%s", strs[index]);
	irq_info->irq = of_irq_get(dev->of_node, 0);
	IPCLITE_OS_LOG(IPCLITE_DBG, "irq[%d] = %d\n", index, irq_info->irq);
	irq_info->signal_id = index;
	ret = devm_request_irq(dev, irq_info->irq,
			       ipclite_intr,
			       IRQF_NO_SUSPEND | IRQF_SHARED,
			       irq_info->irqname, irq_info);
	if (ret) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "failed to request IRQ\n");
		goto err_dev;
	}
	IPCLITE_OS_LOG(IPCLITE_DBG, "Interrupt init completed, ret = %d\n", ret);

	return ret;

err_dev:
	device_unregister(dev);
	kfree(dev);
	return ret;
}

static struct ipcmem_partition_header *get_ipcmem_partition_hdr(struct ipclite_mem ipcmem, int local_pid,
								int remote_pid)
{
	uint32_t p = 0, found = -1;

	for (p = 0; p < partitions; p++) {
		if (ipcmem.toc_data.partition_entry[p].host0 == local_pid
			&& ipcmem.toc_data.partition_entry[p].host1 == remote_pid) {
			found = p;
			break;
		}
	}

	if (found < partitions)
		return (struct ipcmem_partition_header *)((char *)ipcmem.mem.virt_base +
					ipcmem.toc_data.partition_entry[found].base_offset);
	else
		return NULL;
}

static void ipclite_channel_release(struct device *dev)
{
	IPCLITE_OS_LOG(IPCLITE_INFO, "Releasing ipclite channel\n");
	kfree(dev);
}

/* Sets up following fields of IPCLite channel structure:
 *	remote_pid,tx_fifo, rx_fifo
 */
static int ipclite_channel_init(struct device *parent,
								struct device_node *node)
{
	int ret = 0;
	u32 local_pid = 0, remote_pid = 0;
	u32 *descs = NULL;
	struct ipclite_fifo *rx_fifo;
	struct ipclite_fifo *tx_fifo;
	struct device *dev;
	struct device_node *child;
	struct ipcmem_partition_header *partition_hdr;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->parent = parent;
	dev->of_node = node;
	dev->release = ipclite_channel_release;
	dev_set_name(dev, "%s:%pOFn", dev_name(parent->parent), node);
	IPCLITE_OS_LOG(IPCLITE_DBG, "Registering %s device\n", dev_name(parent->parent));
	ret = device_register(dev);
	if (ret) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "failed to register ipclite device\n");
		put_device(dev);
		kfree(dev);
		return ret;
	}

	local_pid = LOCAL_HOST;

	ret = of_property_read_u32(dev->of_node, "qcom,remote-pid",
				   &remote_pid);
	if (ret) {
		dev_err(dev, "failed to parse qcom,remote-pid\n");
		goto err_put_dev;
	}
	IPCLITE_OS_LOG(IPCLITE_DBG, "remote_pid = %d, local_pid=%d\n", remote_pid, local_pid);

	rx_fifo = devm_kzalloc(dev, sizeof(*rx_fifo), GFP_KERNEL);
	tx_fifo = devm_kzalloc(dev, sizeof(*tx_fifo), GFP_KERNEL);
	if (!rx_fifo || !tx_fifo) {
		ret = -ENOMEM;
		goto err_put_dev;
	}
	IPCLITE_OS_LOG(IPCLITE_DBG, "rx_fifo = %p, tx_fifo=%p\n", rx_fifo, tx_fifo);

	partition_hdr = get_ipcmem_partition_hdr(ipclite->ipcmem, local_pid, remote_pid);
	IPCLITE_OS_LOG(IPCLITE_DBG, "partition_hdr = %p\n", partition_hdr);
	if (!partition_hdr) {
		ret = -ENOMEM;
		goto err_put_dev;
	}

	descs = (u32 *)((char *)partition_hdr + partition_hdr->desc_offset);
	IPCLITE_OS_LOG(IPCLITE_DBG, "descs = %p\n", descs);

	if (local_pid < remote_pid) {
		tx_fifo->fifo = (char *)partition_hdr + partition_hdr->fifo0_offset;
		tx_fifo->length = partition_hdr->fifo0_size;
		rx_fifo->fifo = (char *)partition_hdr + partition_hdr->fifo1_offset;
		rx_fifo->length = partition_hdr->fifo1_size;

		tx_fifo->tail = &descs[0];
		tx_fifo->head = &descs[1];
		rx_fifo->tail = &descs[2];
		rx_fifo->head = &descs[3];

	} else {
		tx_fifo->fifo = (char *)partition_hdr + partition_hdr->fifo1_offset;
		tx_fifo->length = partition_hdr->fifo1_size;
		rx_fifo->fifo = (char *)partition_hdr + partition_hdr->fifo0_offset;
		rx_fifo->length = partition_hdr->fifo0_size;

		rx_fifo->tail = &descs[0];
		rx_fifo->head = &descs[1];
		tx_fifo->tail = &descs[2];
		tx_fifo->head = &descs[3];
	}

	if (partition_hdr->type == LOOPBACK_PARTITION_TYPE) {
		rx_fifo->tail = tx_fifo->tail;
		rx_fifo->head = tx_fifo->head;
	}

	/* rx_fifo->reset = ipcmem_rx_reset;*/
	rx_fifo->avail = ipcmem_rx_avail;
	rx_fifo->peak = ipcmem_rx_peak;
	rx_fifo->advance = ipcmem_rx_advance;

	/* tx_fifo->reset = ipcmem_tx_reset;*/
	tx_fifo->avail = ipcmem_tx_avail;
	tx_fifo->write = ipcmem_tx_write;

	*rx_fifo->tail = 0;
	*tx_fifo->head = 0;

	/*Store Channel Information*/
	ipclite->channel[remote_pid].remote_pid = remote_pid;
	ipclite->channel[remote_pid].tx_fifo = tx_fifo;
	ipclite->channel[remote_pid].rx_fifo = rx_fifo;
	ipclite->channel[remote_pid].gstatus_ptr = &partition_hdr->status;

	spin_lock_init(&ipclite->channel[remote_pid].tx_lock);

	for_each_available_child_of_node(dev->of_node, child) {
		ret = ipclite_channel_irq_init(dev, child,
				&ipclite->channel[remote_pid]);
		if (ret) {
			IPCLITE_OS_LOG(IPCLITE_ERR, "irq setup for ipclite channel failed\n");
			goto err_put_dev;
		}
	}

	/* Updating Local & Global Channel Status */
	if (remote_pid == IPCMEM_APPS) {
		*ipclite->channel[remote_pid].gstatus_ptr = ACTIVE;
		ipclite->channel[remote_pid].status = ACTIVE;
	} else {
		*ipclite->channel[remote_pid].gstatus_ptr = IN_PROGRESS;
		ipclite->channel[remote_pid].status = IN_PROGRESS;
	}
	IPCLITE_OS_LOG(IPCLITE_DBG, "Channel init completed, ret = %d\n", ret);
	return ret;

err_put_dev:
	ipclite->channel[remote_pid].status = INACTIVE;
	device_unregister(dev);
	kfree(dev);
	return ret;
}

static void probe_subsystem(struct device *dev, struct device_node *np)
{
	int ret = 0;

	ret = ipclite_channel_init(dev, np);
	if (ret)
		IPCLITE_OS_LOG(IPCLITE_ERR, "IPCLite Channel init failed\n");
}

/* IPCLite Debug related functions start */
static ssize_t ipclite_dbg_lvl_write(struct kobject *kobj,
				struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = 0, host = 0;

	/* Parse the string from Sysfs Interface */
	ret = kstrtoint(buf, 0, &ipclite_debug_level);
	if (ret < 0) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Error parsing the sysfs value");
		return ret;
	}

	/* Check if debug structure is initialized */
	if (!ipclite_dbg_info) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Debug structures not initialized\n");
		return -ENOMEM;
	}

	/* Update the Global Debug variable for FW cores */
	ipclite_dbg_info->debug_level = ipclite_debug_level;

	/* Memory Barrier to make sure all writes are completed */
	wmb();

	/* Signal other cores for updating the debug information */
	for (host = 1; host < IPCMEM_NUM_HOSTS; host++) {
		if (!is_host_enabled(host))
			continue;
		ret = ipclite_notify_core(host, IPCLITE_DEBUG_SIGNAL);
		if (ret < 0)
			IPCLITE_OS_LOG(IPCLITE_ERR, "Failed to send the debug info %d\n", host);
		else
			IPCLITE_OS_LOG(IPCLITE_DBG, "Debug info sent to host %d\n", host);
	}

	return count;
}

static ssize_t ipclite_dbg_ctrl_write(struct kobject *kobj,
				struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = 0, host = 0;

	/* Parse the string from Sysfs Interface */
	ret = kstrtoint(buf, 0, &ipclite_debug_control);
	if (ret < 0) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Error parsing the sysfs value");
		return ret;
	}

	/* Check if debug structures are initialized */
	if (!ipclite_dbg_info || !ipclite_dbg_struct || !ipclite_dbg_inmem) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Debug structures not initialized\n");
		return -ENOMEM;
	}

	/* Update the Global Debug variable for FW cores */
	ipclite_dbg_info->debug_control = ipclite_debug_control;

	/* Memory Barrier to make sure all writes are completed */
	wmb();

	/* Signal other cores for updating the debug information */
	for (host = 1; host < IPCMEM_NUM_HOSTS; host++) {
		if (!is_host_enabled(host))
			continue;
		ret = ipclite_notify_core(host, IPCLITE_DEBUG_SIGNAL);
		if (ret < 0)
			IPCLITE_OS_LOG(IPCLITE_ERR, "Failed to send the debug info %d\n", host);
		else
			IPCLITE_OS_LOG(IPCLITE_DBG, "Debug info sent to host %d\n", host);
	}

	return count;
}

static ssize_t ipclite_dbg_dump_write(struct kobject *kobj,
				struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;

	/* Parse the string from Sysfs Interface */
	ret = kstrtoint(buf, 0, &ipclite_debug_dump);
	if (ret < 0) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Error parsing the sysfs value");
		return ret;
	}

	/* Check if debug structures are initialized */
	if (!ipclite_dbg_info || !ipclite_dbg_struct || !ipclite_dbg_inmem) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Debug structures not initialized\n");
		return -ENOMEM;
	}

	/* Dump the debug information */
	if (ipclite_debug_dump & IPCLITE_DUMP_DBG_STRUCT)
		ipclite_dump_debug_struct();

	if (ipclite_debug_dump & IPCLITE_DUMP_INMEM_LOG)
		ipclite_dump_inmem_logs();

	return count;
}

struct kobj_attribute sysfs_dbg_lvl = __ATTR(ipclite_debug_level, 0660,
					NULL, ipclite_dbg_lvl_write);
struct kobj_attribute sysfs_dbg_ctrl = __ATTR(ipclite_debug_control, 0660,
					NULL, ipclite_dbg_ctrl_write);
struct kobj_attribute sysfs_dbg_dump = __ATTR(ipclite_debug_dump, 0660,
					NULL, ipclite_dbg_dump_write);

static int ipclite_debug_sysfs_setup(void)
{
	int ret = 0;

	/* Creating a directory in /sys/kernel/ */
	sysfs_kobj = kobject_create_and_add("ipclite", kernel_kobj);
	if (!sysfs_kobj) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Cannot create and add sysfs directory\n");
		return -ENOMEM;
	}

	/* Creating sysfs files/interfaces for debug */
	ret = sysfs_create_file(sysfs_kobj, &sysfs_dbg_lvl.attr);
	if (ret) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Cannot create sysfs debug level file\n");
		return ret;
	}

	ret = sysfs_create_file(sysfs_kobj, &sysfs_dbg_ctrl.attr);
	if (ret) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Cannot create sysfs debug control file\n");
		return ret;
	}

	ret = sysfs_create_file(sysfs_kobj, &sysfs_dbg_dump.attr);
	if (ret) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Cannot create sysfs debug dump file\n");
		return ret;
	}

	return ret;
}

static int ipclite_debug_mem_setup(void)
{
	/* Setting up the Debug Structures */
	ipclite_dbg_info = (struct ipclite_debug_info *)(((char *)ipclite->ipcmem.mem.virt_base +
						ipclite->ipcmem.mem.size) - DEBUG_PARTITION_SIZE);
	if (!ipclite_dbg_info)
		return -EADDRNOTAVAIL;

	ipclite_dbg_struct = (struct ipclite_debug_struct *)
					(((char *)ipclite_dbg_info + IPCLITE_DEBUG_INFO_SIZE) +
					(sizeof(*ipclite_dbg_struct) * IPCMEM_APPS));
	if (!ipclite_dbg_struct)
		return -EADDRNOTAVAIL;

	ipclite_dbg_inmem = (struct ipclite_debug_inmem_buf *)
					(((char *)ipclite_dbg_info + IPCLITE_DEBUG_INFO_SIZE) +
					(sizeof(*ipclite_dbg_struct) * IPCMEM_NUM_HOSTS));

	if (!ipclite_dbg_inmem)
		return -EADDRNOTAVAIL;

	IPCLITE_OS_LOG(IPCLITE_DBG, "virtual_base_ptr = %p total_size : %d debug_size : %d\n",
		ipclite->ipcmem.mem.virt_base, ipclite->ipcmem.mem.size, DEBUG_PARTITION_SIZE);
	IPCLITE_OS_LOG(IPCLITE_DBG, "dbg_info : %p dbg_struct : %p dbg_inmem : %p\n",
					ipclite_dbg_info, ipclite_dbg_struct, ipclite_dbg_inmem);

	return 0;
}

static int ipclite_debug_setup(void)
{
	int ret = 0;

	/* Set up sysfs for debug */
	ret = ipclite_debug_sysfs_setup();
	if (ret != 0) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Failed to Set up IPCLite Debug Sysfs\n");
		return ret;
	}

	/* Mapping Debug Memory */
	ret = ipclite_debug_mem_setup();
	if (ret != 0) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Failed to Set up IPCLite Debug Structures\n");
		return ret;
	}

	/* Update the Global Debug variable for FW cores */
	ipclite_dbg_info->debug_level = ipclite_debug_level;
	ipclite_dbg_info->debug_control = ipclite_debug_control;

	return ret;
}
/* IPCLite Debug related functions end */

/* IPCLite Features setup related functions start */
static int ipclite_feature_setup(struct device_node *pn)
{
	int ret = 0;
	uint32_t feature_mask_l = 0, feature_mask_h = 0;

	/* Parse the feature related DT entries and store the values locally */
	ret = of_property_read_u32(pn, "feature_mask_low", &feature_mask_l);
	if (ret != 0) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "failed to parse feature_mask_low\n");
		return ret;
	}

	ret = of_property_read_u32(pn, "feature_mask_high", &feature_mask_h);
	if (ret != 0) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "failed to parse feature_mask_high\n");
		return ret;
	}

	/* Combine feature_mask_low and feature_mask_high into 64-bit feature_mask */
	feature_mask = (uint64_t) feature_mask_h << 32 | feature_mask_l;

	/* Update the feature mask to TOC for FW */
	ipclite->ipcmem.toc->hdr.feature_mask = feature_mask;

	/* Set up Global Atomics Feature*/
	if (!(IS_FEATURE_CONFIG(IPCLITE_GLOBAL_ATOMIC)))
		IPCLITE_OS_LOG(IPCLITE_INFO, "IPCLite Global Atomic Support Disabled\n");

	/* Set up Test Suite Feature*/
	if (!(IS_FEATURE_CONFIG(IPCLITE_TEST_SUITE)))
		IPCLITE_OS_LOG(IPCLITE_INFO, "IPCLite Test Suite Disabled\n");

	return ret;
}
/* IPCLite Features setup related functions end */

/* API Definition Start - Minor Version 0*/
static int ipclite_init_v0(struct platform_device *pdev)
{
	int ret = 0, hwlock_id = 0;
	struct ipcmem_region *mem;
	struct device_node *cn;
	struct device_node *pn = pdev->dev.of_node;
	struct ipclite_channel broadcast;

	/* Allocate memory for IPCLite */
	ipclite = kzalloc(sizeof(*ipclite), GFP_KERNEL);
	if (!ipclite) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "IPCLite Memory Allocation Failed\n");
		ret = -ENOMEM;
		goto error;
	}

	ipclite->dev = &pdev->dev;

	/* Parse HW Lock from DT */
	hwlock_id = of_hwspin_lock_get_id(pn, 0);
	if (hwlock_id < 0) {
		if (hwlock_id != -EPROBE_DEFER)
			dev_err(&pdev->dev, "failed to retrieve hwlock\n");
		ret = hwlock_id;
		goto release;
	}
	IPCLITE_OS_LOG(IPCLITE_DBG, "Hwlock id retrieved, hwlock_id=%d\n", hwlock_id);

	/* Reserve a HWSpinLock for later use */
	ipclite->hwlock = hwspin_lock_request_specific(hwlock_id);
	if (!ipclite->hwlock) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Failed to assign hwlock_id\n");
		ret = -ENXIO;
		goto release;
	}
	IPCLITE_OS_LOG(IPCLITE_DBG, "Hwlock id assigned successfully, hwlock=%p\n",
									ipclite->hwlock);

	/* Initializing Local Mutex Lock for SSR functionality */
	mutex_init(&ssr_mutex);

	/* Map to IPCLite Memory */
	ret = map_ipcmem(ipclite, "memory-region");
	if (ret) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "failed to map ipcmem\n");
		goto release;
	}
	mem = &(ipclite->ipcmem.mem);
	memset(mem->virt_base, 0, mem->size);

	ret = ipcmem_init(&ipclite->ipcmem, pn);
	if (ret) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Failed to set up IPCMEM");
		goto release;
	}

	/* Setup Channel for each Remote Subsystem */
	for_each_available_child_of_node(pn, cn)
		probe_subsystem(&pdev->dev, cn);

	/* Broadcast init_done signal to all subsystems once mbox channels are set up */
	broadcast = ipclite->channel[IPCMEM_APPS];
	ret = mbox_send_message(broadcast.irq_info[IPCLITE_MEM_INIT_SIGNAL].mbox_chan, NULL);
	if (ret < 0)
		goto mem_release;

	mbox_client_txdone(broadcast.irq_info[IPCLITE_MEM_INIT_SIGNAL].mbox_chan, 0);

	/* Debug Setup */
	ret = ipclite_debug_setup();
	if (ret != 0) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "IPCLite Debug Setup Failed\n");
		goto release;
	}

	/* Features Setup */
	ret = ipclite_feature_setup(pn);
	if (ret != 0) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "IPCLite Features Setup Failed\n");
		goto release;
	}

	/* Update TOC with version entries for FW */
	ipclite->ipcmem.toc->hdr.major_version = major_ver;
	ipclite->ipcmem.toc->hdr.minor_version = minor_ver;

	/* Should be called after all Global TOC related init is done */
	insert_magic_number();

	IPCLITE_OS_LOG(IPCLITE_INFO, "IPCLite Version : %d.%d Feature Mask : 0x%llx\n",
						major_ver, minor_ver, feature_mask);

	IPCLITE_OS_LOG(IPCLITE_INFO, "IPCLite Probe Completed Successfully\n");

	return ret;

mem_release:
	/* If the remote subsystem has already completed the init and actively
	 * using IPCMEM, re-assigning IPCMEM memory back to HLOS can lead to crash
	 * Solution: Either we don't take back the memory or make sure APPS completes
	 * init before any other subsystem initializes IPCLite (we won't have to send
	 * braodcast)
	 */
release:
	kfree(ipclite);
	ipclite = NULL;
error:
	return ret;
}

static int ipclite_register_client_v0(IPCLite_Client cb_func_ptr, void *priv)
{
	if (!cb_func_ptr) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Invalid callback pointer\n");
		return -EINVAL;
	}

	synx_client.callback = cb_func_ptr;
	synx_client.priv_data = priv;
	synx_client.reg_complete = 1;

	IPCLITE_OS_LOG(IPCLITE_DBG, "Client Registration completed\n");

	return 0;
}

static int ipclite_register_test_client_v0(IPCLite_Client cb_func_ptr, void *priv)
{
	if (!cb_func_ptr) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Invalid callback pointer\n");
		return -EINVAL;
	}

	test_client.callback = cb_func_ptr;
	test_client.priv_data = priv;
	test_client.reg_complete = 1;

	IPCLITE_OS_LOG(IPCLITE_DBG, "Test Client Registration Completed\n");

	return 0;
}

static int ipclite_msg_send_v0(int32_t proc_id, uint64_t data)
{
	int ret = 0;

	/* Check for valid core id */
	if (proc_id < 0 || proc_id >= IPCMEM_NUM_HOSTS) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Invalid proc_id : %d\n", proc_id);
		return -EINVAL;
	}

	/* Send the data to the core */
	ret = ipclite_tx(&ipclite->channel[proc_id], data, sizeof(data), IPCLITE_MSG_SIGNAL);
	if (ret < 0) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Message send failed to core : %d signal:%d ret:%d\n",
								proc_id, IPCLITE_MSG_SIGNAL, ret);
		return ret;
	}

	IPCLITE_OS_LOG(IPCLITE_DBG, "Message send complete to core : %d signal : %d ret : %d\n",
								proc_id, IPCLITE_MSG_SIGNAL, ret);
	return ret;
}

static int ipclite_test_msg_send_v0(int32_t proc_id, uint64_t data)
{
	int ret = 0;

	/* Check for valid core id */
	if (proc_id < 0 || proc_id >= IPCMEM_NUM_HOSTS) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Invalid proc_id : %d\n", proc_id);
		return -EINVAL;
	}

	/* Send the data to the core */
	ret = ipclite_tx(&ipclite->channel[proc_id], data, sizeof(data), IPCLITE_TEST_SIGNAL);
	if (ret < 0) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Message send failed to core : %d signal:%d ret:%d\n",
								proc_id, IPCLITE_TEST_SIGNAL, ret);
		return ret;
	}

	IPCLITE_OS_LOG(IPCLITE_DBG, "Test Msg send complete to core : %d signal : %d ret : %d\n",
								proc_id, IPCLITE_TEST_SIGNAL, ret);
	return ret;
}

static int32_t get_global_partition_info_v0(struct global_region_info *global_ipcmem)
{
	struct ipcmem_global_partition *global_partition;

	if (!ipclite) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "IPCLite not initialized\n");
		return -ENOMEM;
	}

	if (!global_ipcmem)
		return -EINVAL;

	global_partition = ipclite->ipcmem.global_partition;
	global_ipcmem->virt_base = (void *)((char *)global_partition +
							global_partition->hdr.region_offset);
	global_ipcmem->size = (size_t)(global_partition->hdr.region_size);

	IPCLITE_OS_LOG(IPCLITE_DBG, "base = %p, size=%lx\n", global_ipcmem->virt_base,
									global_ipcmem->size);
	return 0;
}

static void ipclite_recover_v0(enum ipcmem_host_type core_id)
{
	int ret = 0, host = 0, host0 = 0, host1 = 0;
	uint32_t p = 0;

	IPCLITE_OS_LOG(IPCLITE_DBG, "IPCLite Recover - Crashed Core : %d\n", core_id);

	/* verify and reset the hw mutex lock */
	if (core_id == ipclite->ipcmem.toc_data.host_info->hwlock_owner) {
		ipclite->ipcmem.toc_data.host_info->hwlock_owner = IPCMEM_INVALID_HOST;
		hwspin_unlock_raw(ipclite->hwlock);
		IPCLITE_OS_LOG(IPCLITE_DBG, "HW Lock Reset\n");
	}

	mutex_lock(&ssr_mutex);

	/* Set the Global Channel Status to 0 to avoid Race condition */
	for (p = 0; p < partitions; p++) {
		host0 = ipclite->ipcmem.toc_data.partition_entry[p].host0;
		host1 = ipclite->ipcmem.toc_data.partition_entry[p].host1;
		if (host0 != core_id && host1 != core_id)
			continue;

		ipclite_global_atomic_store_i32((ipclite_atomic_int32_t *)
			(&(ipclite->ipcmem.partition[p]->hdr.status)), 0);

		IPCLITE_OS_LOG(IPCLITE_DBG, "Global Channel Status : [%d][%d] : %d\n",
					host0, host1, ipclite->ipcmem.partition[p]->hdr.status);
	}

	/* Resets the TX/RX queue */
	*(ipclite->channel[core_id].tx_fifo->head) = 0;
	*(ipclite->channel[core_id].rx_fifo->tail) = 0;

	IPCLITE_OS_LOG(IPCLITE_DBG, "TX Fifo Reset : %d\n",
						*(ipclite->channel[core_id].tx_fifo->head));
	IPCLITE_OS_LOG(IPCLITE_DBG, "RX Fifo Reset : %d\n",
						*(ipclite->channel[core_id].rx_fifo->tail));

	/* Increment the Global Channel Status for APPS and crashed core*/
	ipclite_global_atomic_inc((ipclite_atomic_int32_t *)
					ipclite->channel[core_id].gstatus_ptr);

	ipclite->channel[core_id].status = *ipclite->channel[core_id].gstatus_ptr;

	/* Update other cores about SSR */
	for (host = 1; host < IPCMEM_NUM_HOSTS; host++) {
		if (!is_host_enabled(host) || host == core_id)
			continue;
		ret = ipclite_notify_core(host, IPCLITE_SSR_SIGNAL);
		if (ret < 0)
			IPCLITE_OS_LOG(IPCLITE_ERR, "Failed to send SSR update to core %d\n", host);
		else
			IPCLITE_OS_LOG(IPCLITE_DBG, "SSR update sent to core %d\n", host);
	}

	mutex_unlock(&ssr_mutex);

	/* Dump the debug information */
	if (ipclite_debug_dump & IPCLITE_DUMP_SSR) {
		ipclite_dump_debug_struct();
		ipclite_dump_inmem_logs();
	}
}
/* API Definition End - Minor Version 0*/

/* Versioned Functions Start */
int ipclite_init(struct platform_device *pdev)
{
	if (api_list_t.init == NULL) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Unassigned function : %s", __func__);
		return -EINVAL;
	}

	return api_list_t.init(pdev);
}

int ipclite_register_client(IPCLite_Client cb_func_ptr, void *priv)
{
	if (api_list_t.register_client == NULL) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Unassigned function : %s", __func__);
		return -EINVAL;
	}

	return api_list_t.register_client(cb_func_ptr, priv);
}
EXPORT_SYMBOL(ipclite_register_client);

int ipclite_register_test_client(IPCLite_Client cb_func_ptr, void *priv)
{
	if (api_list_t.register_test_client == NULL) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Unassigned function : %s", __func__);
		return -EINVAL;
	}

	return api_list_t.register_test_client(cb_func_ptr, priv);
}
EXPORT_SYMBOL(ipclite_register_test_client);

int ipclite_msg_send(int32_t proc_id, uint64_t data)
{
	if (api_list_t.msg_send == NULL) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Unassigned function : %s", __func__);
		return -EINVAL;
	}

	return api_list_t.msg_send(proc_id, data);
}
EXPORT_SYMBOL(ipclite_msg_send);

int ipclite_test_msg_send(int32_t proc_id, uint64_t data)
{
	if (api_list_t.test_msg_send == NULL) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Unassigned function : %s", __func__);
		return -EINVAL;
	}

	return api_list_t.test_msg_send(proc_id, data);
}
EXPORT_SYMBOL(ipclite_test_msg_send);

void ipclite_recover(enum ipcmem_host_type core_id)
{
	if (api_list_t.recover == NULL) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Unassigned function : %s", __func__);
		return;
	}

	api_list_t.recover(core_id);
}
EXPORT_SYMBOL(ipclite_recover);

int32_t get_global_partition_info(struct global_region_info *global_ipcmem)
{
	if (api_list_t.partition_info == NULL) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "Unassigned function : %s", __func__);
		return -EINVAL;
	}

	return api_list_t.partition_info(global_ipcmem);
}
EXPORT_SYMBOL(get_global_partition_info);
/* Versioned Functions End */

/* List of APIs  based on the version */
struct ipclite_api_list api_list_version[] = {
	/* Minor Version 0 */
	{
		.init = ipclite_init_v0,
		.register_client = ipclite_register_client_v0,
		.register_test_client = ipclite_register_test_client_v0,
		.msg_send = ipclite_msg_send_v0,
		.test_msg_send = ipclite_test_msg_send_v0,
		.partition_info = get_global_partition_info_v0,
		.recover = ipclite_recover_v0,
	},
};

/* IPCLite Version setup related functions start */
static int ipclite_update_version_api(struct ipclite_api_list *res_str,
						struct ipclite_api_list *ver_str)
{
	if (res_str == NULL || ver_str == NULL)
		return -EINVAL;

	/* Register APIs based on the version */
	res_str->init = (ver_str->init != NULL) ?
		ver_str->init : res_str->init;

	res_str->register_client = (ver_str->register_client != NULL) ?
		ver_str->register_client : res_str->register_client;
	res_str->register_test_client = (ver_str->register_test_client != NULL) ?
		ver_str->register_test_client : res_str->register_test_client;

	res_str->msg_send = (ver_str->msg_send != NULL) ?
		ver_str->msg_send : res_str->msg_send;
	res_str->test_msg_send = (ver_str->test_msg_send != NULL) ?
		ver_str->test_msg_send : res_str->test_msg_send;

	res_str->partition_info = (ver_str->partition_info != NULL) ?
		ver_str->partition_info : res_str->partition_info;
	res_str->recover = (ver_str->recover != NULL) ?
		ver_str->recover : res_str->recover;

	return 0;
}

static int ipclite_register_api(void)
{
	int ret = 0, ver_itr = 0;

	/* Register APIs based on the version */
	for (ver_itr = 0; ver_itr <= minor_ver; ver_itr++) {
		ret = ipclite_update_version_api(&api_list_t, &api_list_version[ver_itr]);
		if (ret != 0)
			return ret;
	}

	return ret;
}

static int ipclite_version_setup(struct device_node *pn)
{
	int ret = 0;

	/* Parse the version related DT entries and store the values locally */
	ret = of_property_read_u32(pn, "major_version", &major_ver);
	if (ret != 0) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "failed to parse major_vesion\n");
		return ret;
	}

	ret = of_property_read_u32(pn, "minor_version", &minor_ver);
	if (ret != 0) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "failed to parse minor_vesion\n");
		return ret;
	}

	/* Verify IPCLite Version - if version does not match crash the system */
	BUG_ON(major_ver != MAJOR_VERSION || minor_ver > MINOR_VERSION);

	return ret;
}
/* IPCLite Version setup related functions end */

/* Start of IPCLite Init*/
static int ipclite_probe(struct platform_device *pdev)
{
	int ret = 0;

	/* Version Setup */
	ret = ipclite_version_setup(pdev->dev.of_node);
	if (ret != 0) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "IPCLite Version Setup Failed\n");
		goto error;
	}

	/* Register API Setup */
	ret = ipclite_register_api();
	if (ret != 0) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "IPCLite API Registration Failed\n");
		goto error;
	}

	/* IPCLite Init */
	ret = ipclite_init(pdev);
	if (ret != 0) {
		IPCLITE_OS_LOG(IPCLITE_ERR, "IPCLite Init Failed\n");
		goto error;
	}

	return ret;

error:
	IPCLITE_OS_LOG(IPCLITE_ERR, "IPCLite probe failed\n");
	return ret;
}

static const struct of_device_id ipclite_of_match[] = {
	{ .compatible = "qcom,ipclite"},
	{}
};
MODULE_DEVICE_TABLE(of, ipclite_of_match);

static struct platform_driver ipclite_driver = {
	.probe = ipclite_probe,
	.driver = {
		.name = "ipclite",
		.of_match_table = ipclite_of_match,
	},
};

module_platform_driver(ipclite_driver);

MODULE_DESCRIPTION("IPCLite Driver");
MODULE_LICENSE("GPL v2");
MODULE_SOFTDEP("pre: qcom_hwspinlock");
