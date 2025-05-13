/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _HFI_INTF_H_
#define _HFI_INTF_H_

#include <linux/types.h>

#define HFI_CMD_Q_MINI_DUMP_SIZE_IN_BYTES      4096
#define HFI_MSG_Q_MINI_DUMP_SIZE_IN_BYTES      4096

#define HFI_NUM_MAX                            2
#define HFI_HANDLE_INIT_VALUE                  HFI_NUM_MAX

#define HFI_CLIENT_NAME_LEN                    32

/**
 * struct hfi_mem
 * @len: length of memory
 * @kva: kernel virtual address
 * @iova: IO virtual address
 * @reserved: reserved field
 */
struct hfi_mem {
	uint64_t  len;
	uintptr_t kva;
	uint32_t  iova;
	uint32_t  reserved;
};

/**
 * struct hfi_mem_info
 * @qtbl: qtable hfi memory
 * @cmd_q: command queue hfi memory for host to firmware communication
 * @msg_q: message queue hfi memory for firmware to host communication
 * @dbg_q: debug queue hfi memory for firmware debug information
 * @sfr_buf: buffer for subsystem failure reason[SFR]
 * @sec_heap: secondary heap hfi memory for firmware
 * @qdss: qdss mapped memory for fw
 * @io_mem: io memory info
 * @io_mem2: 2nd io memory info
 * @fw_uncached: FW uncached region info
 * @global_sync: Global sync mem for IPC
 * @device_mem: device memory
 */
struct hfi_mem_info {
	struct hfi_mem qtbl;
	struct hfi_mem cmd_q;
	struct hfi_mem msg_q;
	struct hfi_mem dbg_q;
	struct hfi_mem sfr_buf;
	struct hfi_mem sec_heap;
	struct hfi_mem shmem;
	struct hfi_mem qdss;
	struct hfi_mem io_mem;
	struct hfi_mem io_mem2;
	struct hfi_mem fw_uncached;
	struct hfi_mem device_mem;
};

/**
 * struct hfi_ops
 * @irq_raise: called to raise H2ICP interrupt
 * @irq_enable: called to enable interrupts from ICP
 * @iface_addr: called to get interface registers base address
 */
struct hfi_ops {
	void (*irq_raise)(void *data);
	void (*irq_enable)(void *data);
	void __iomem *(*iface_addr)(void *data);
};

/**
 * struct hfi_mini_dump_info
 * @cmd_q: cmd queue
 * @msg_q: msg queue
 * @msg_q_state: msg queue state
 * @cmd_q_state: cmd queue state
 * @dbg_q_state: dbg queue state
 */
struct hfi_mini_dump_info {
	uint32_t       cmd_q[HFI_CMD_Q_MINI_DUMP_SIZE_IN_BYTES];
	uint32_t       msg_q[HFI_MSG_Q_MINI_DUMP_SIZE_IN_BYTES];
	bool           msg_q_state;
	bool           cmd_q_state;
	bool           dbg_q_state;
};
/**
 * hfi_write_cmd() - function for hfi write
 * @client_handle: client handle
 * @cmd_ptr: pointer to command data for hfi write
 *
 * Returns success(zero)/failure(non zero)
 */
int hfi_write_cmd(int client_handle, void *cmd_ptr);

/**
 * hfi_read_message() - function for hfi read
 * @client_handle: client handle
 * @pmsg: buffer to place read message for hfi queue
 * @q_id: Queue to read from
 * @buf_words_size: size in words of the input buffer
 * @words_read: total number of words read from the queue
 *              returned as output to the caller
 *
 * Returns success(zero)/failure(non zero)
 */
int hfi_read_message(int client_handle, uint32_t *pmsg, uint8_t q_id,
	size_t buf_words_size, uint32_t *words_read);

/**
 * hfi_init() - function initialize hfi after firmware download
 * @client_handle: client handle
 * @hfi_mem: hfi memory info
 * @hfi_ops: processor-specific hfi ops
 * @priv: device private data
 * @event_driven_mode: event mode
 *
 * Returns success(zero)/failure(non zero)
 */
int cam_hfi_init(int client_handle, struct hfi_mem_info *hfi_mem,
	const struct hfi_ops *hfi_ops, void *priv, uint8_t event_driven_modem);

/**
 * cam_hfi_register() - function to register user as hfi client and retrieve handle
 * @client_handle: client handle to be retrieved
 * @client_name: Name of the client to be registered
 *
 * Returns success(zero)/failure(non zero)
 */
int cam_hfi_register(int *client_handle, const char *client_name);

/**
 * cam_hfi_unregister() - function to unregister hfi client
 * @client_handle: client handle
 *
 * Returns success(zero)/failure(non zero)
 */
int cam_hfi_unregister(int *client_handle);

/**
 * hfi_get_hw_caps() - hardware capabilities from firmware
 * @query_caps: holds query information from hfi
 *
 * Returns success(zero)/failure(non zero)
 */
int hfi_get_hw_caps(void *query_caps);

/**
 * hfi_get_hw_caps_v2() - hardware capabilities from firmware, used for v2 query cap
 * @client_handle: client handle
 * @query_caps: holds query information from hfi
 *
 * Returns success(zero)/failure(non zero)
 */
int hfi_get_hw_caps_v2(int client_handle, void *query_caps);

/**
 * hfi_send_system_cmd() - send hfi system command to firmware
 * @client_handle: client handle
 * @type: type of system command
 * @data: command data
 * @size: size of command data
 *
 * Returns success(zero)/failure(non zero)
 */
int hfi_send_system_cmd(int client_handle, uint32_t type, uint64_t data, uint32_t size);

/**
 * cam_hfi_deinit() - cleanup HFI
 * @client_handle: client handle to be retrieved
 */
void cam_hfi_deinit(int client_handle);
/**
 * hfi_set_debug_level() - set debug level
 * @client_handle: client handle to be retrieved
 * @icp_dbg_type: 1 for debug_q & 2 for qdss
 * @lvl: FW debug message level
 *
 * Returns success(zero)/failure(non zero)
 */
int hfi_set_debug_level(int client_handle, u64 icp_dbg_type, uint32_t lvl);

/**
 * hfi_set_fw_dump_levels() - set firmware hang dump/ramdump levels
 * @client_handle: client handle to be retrieved
 * @hang_dump_lvl : level of firmware hang dump
 * @ram_dump_lvl  : level of firmware ram dump
 *
 * Returns success(zero)/failure(non zero)
 */
int hfi_set_fw_dump_levels(int client_handle,
	uint32_t hang_dump_lvl, uint32_t ram_dump_lvl);

/**
 * hfi_send_freq_info() - set firmware dump level
 * @client_handle: client handle to be retrieved
 * @freq: icp freq
 *
 * Returns success(zero)/failure(non zero)
 */
int hfi_send_freq_info(int client_handle, int32_t freq);

/**
 * hfi_cmd_ubwc_config_ext() - UBWC configuration to firmware
 * @client_handle: client handle to be retrieved
 * @ubwc_ipe_cfg: UBWC ipe fetch/write configuration params
 * @ubwc_bps_cfg: UBWC bps fetch/write configuration params
 * @ubwc_ofe_cfg: UBWC ofe fetch/write configuration params
 *
 * Returns success(zero)/failure(non zero)
 */
int hfi_cmd_ubwc_config_ext(int client_handle, uint32_t *ubwc_ipe_cfg,
	uint32_t *ubwc_bps_cfg, uint32_t *ubwc_ofe_cfg);

/**
 * hfi_cmd_ubwc_config() - UBWC configuration to firmware
 *                         for older targets
 * @client_handle: client handle to be retrieved
 * @ubwc_cfg: UBWC configuration parameters
 *
 * Returns success(zero)/failure(non zero)
 */
int hfi_cmd_ubwc_config(int client_handle, uint32_t *ubwc_cfg);

/**
 * cam_hfi_resume() - function to resume
 * @client_handle: client handle to be retrieved
 *
 * Returns success(zero)/failure(non zero)
 */
int cam_hfi_resume(int client_handle);

/**
 * cam_hfi_queue_dump() - utility function to dump hfi queues
 * @client_handle: client handle to be retrieved
 * @dump_queue_data: if set dumps queue contents
 */
void cam_hfi_queue_dump(int client_handle, bool dump_queue_data);

/**
 * cam_hfi_mini_dump() - utility function for mini dump
 * @client_handle: client handle to be retrieved
 * @dst: memory destination
 */
void cam_hfi_mini_dump(int client_handle, struct hfi_mini_dump_info *dst);
#endif /* _HFI_INTF_H_ */
