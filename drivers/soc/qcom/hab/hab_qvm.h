/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023, 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#ifndef __HAB_QNX_H
#define __HAB_QNX_H
#include "hab.h"
#include "hab_pipe.h"

#include "hab_qvm_os.h"

struct qvm_channel {
	int be;

	struct hab_pipe *pipe;
	struct hab_pipe_endpoint *pipe_ep;
	struct hab_shared_buf *tx_buf;
	struct hab_shared_buf *rx_buf;
	struct dbg_items *dbg_itms;
	spinlock_t io_lock;

	/* common but only for guest */
	struct guest_shm_factory *guest_factory;
	struct guest_shm_control *guest_ctrl;

	/* cached guest ctrl idx value to prevent trap when accessed */
	uint32_t idx;

	/* Guest VM */
	unsigned int guest_intr;
	unsigned int guest_iid;
	unsigned int factory_addr;
	unsigned int irq;

	/* os-specific part */
	struct qvm_channel_os *os_data;

	/* debug only */
	struct workqueue_struct *wq;
	struct work_data {
		struct work_struct work;
		int data; /* free to modify */
	} wdata;
	char *side_buf; /* to store the contents from hab-pipe */
};

/* This is common but only for guest in HQX */
struct shmem_irq_config {
	unsigned long factory_addr; /* from gvm settings when provided */
	int irq; /* from gvm settings when provided */
};

struct qvm_plugin_info {
	struct shmem_irq_config *pchan_settings;
	int setting_size;
	int curr;
	int probe_cnt;
};

extern struct qvm_plugin_info qvm_priv_info;

/* Shared mem size in each direction for communication pipe */
#define PIPE_SHMEM_SIZE (512 * 1024)

void hab_pipe_reset(struct physical_channel *pchan);
void habhyp_notify(void *commdev);
unsigned long hab_shmem_factory_va(unsigned long factory_addr);
char *hab_shmem_attach(struct qvm_channel *dev, const char *name,
	uint32_t pages);
uint64_t get_guest_ctrl_paddr(struct qvm_channel *dev,
	unsigned long factory_addr, int irq, const char *name, uint32_t pages);
int shmem_habhyp_commdev_alloc(void **commdev, int is_be, char *name,
			int vmid_remote,
			struct hab_device *mmid_device);
int shmem_habhyp_commdev_dealloc(void *commdev);
void shmem_habhyp_commdev_dealloc_os(void *commdev);
int shmem_physical_channel_send(struct physical_channel *pchan,
			struct hab_header *header,
			void *payload,
			unsigned int flags);
int shmem_physical_channel_read(struct physical_channel *pchan,
			void *payload,
			size_t read_size);
void shmem_physical_channel_rx_dispatch(unsigned long data);
int shmem_habhyp_commdev_create_dispatcher(struct physical_channel *pchan);
void shmem_hab_pipe_read_dump(struct physical_channel *pchan);
void shmem_dump_hab_wq(struct physical_channel *pchan);
int shmem_hab_hypervisor_register(void);
int shmem_hab_hypervisor_register_os(void);
void shmem_hab_hypervisor_unregister(void);
int shmem_hab_stat_log(struct physical_channel **pchans,
			int pchan_cnt,
			char *dest,
			int dest_siz);
int shmem_hab_hypervisor_unregister_os(void);

#endif /* __HAB_QNX_H */
