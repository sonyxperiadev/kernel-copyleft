/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2019, 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __SYNX_API_H__
#define __SYNX_API_H__

#include <linux/list.h>
#include <synx_header.h>

#include "synx_err.h"

#define SYNX_NO_TIMEOUT        ((u64)-1)

/**
 * SYNX_INVALID_HANDLE      : client can assign the synx handle variable with this value
 *                            when it doesn't hold a valid synx handle
 */
#define SYNX_INVALID_HANDLE 0

/* synx object states */
#define SYNX_STATE_INVALID             0    // Invalid synx object
#define SYNX_STATE_ACTIVE              1    // Synx object has not been signaled
#define SYNX_STATE_SIGNALED_ERROR      3    // Synx object signaled with error
#define SYNX_STATE_SIGNALED_EXTERNAL   5    // Synx object was signaled by external dma client.
#define SYNX_STATE_SIGNALED_SSR        6    // Synx object signaled with SSR
#define SYNX_STATE_TIMEOUT             7    // Callback status for synx object in case of timeout

/**
 * enum synx_create_flags - Flags passed during synx_create call.
 *
 * SYNX_CREATE_LOCAL_FENCE  : Instructs the framework to create local synx object,
 *                            for local synchronization i.e. within same core.
 * SYNX_CREATE_GLOBAL_FENCE : Instructs the framework to create global synx object
 *                            for global synchronization i.e. across supported core.
 * SYNX_CREATE_DMA_FENCE    : Create a synx object by wrapping the provided dma fence.
 *                            Need to pass the dma_fence ptr through fence variable
 *                            if this flag is set. (NOT SUPPORTED)
 * SYNX_CREATE_CSL_FENCE    : Create a synx object with provided csl fence.
 *                            Establishes interop with the csl fence through
 *                            bind operations. (NOT SUPPORTED)
 */
enum synx_create_flags {
	SYNX_CREATE_LOCAL_FENCE  = 0x01,
	SYNX_CREATE_GLOBAL_FENCE = 0x02,
	SYNX_CREATE_DMA_FENCE    = 0x04,
	SYNX_CREATE_CSL_FENCE    = 0x08,
	SYNX_CREATE_MAX_FLAGS    = 0x10,
};

/**
 * enum synx_init_flags - Session initialization flag
 * SYNX_INIT_DEFAULT   : Initialization flag to be passed
 *                       when initializing session
 * SYNX_INIT_MAX       : Used for internal checks
 */
enum synx_init_flags {
	SYNX_INIT_DEFAULT = 0x00,
	SYNX_INIT_MAX     = 0x01,
};

/**
 * enum synx_import_flags - Import flags
 *
 * SYNX_IMPORT_LOCAL_FENCE  : Instructs the framework to create local synx object,
 *                            for local synchronization i.e. within same core.
 * SYNX_IMPORT_GLOBAL_FENCE : Instructs the framework to create global synx object,
 *                            for global synchronization i.e. across supported core.
 * SYNX_IMPORT_SYNX_FENCE   : Import native Synx handle for synchronization.
 *                            Need to pass the Synx handle ptr through fence variable
 *                            if this flag is set. Client must pass:
 *                            a. SYNX_IMPORT_SYNX_FENCE|SYNX_IMPORT_LOCAL_FENCE
 *                               to import a synx handle as local synx handle.
 *                            b. SYNX_IMPORT_SYNX_FENCE|SYNX_IMPORT_GLOBAL_FENCE
 *                               to import a synx handle as global synx handle.
 * SYNX_IMPORT_DMA_FENCE    : Import dma fence and create Synx handle for interop.
 *                            Need to pass the dma_fence ptr through fence variable
 *                            if this flag is set. Client must pass:
 *                            a. SYNX_IMPORT_DMA_FENCE|SYNX_IMPORT_LOCAL_FENCE
 *                               to import a dma fence and create local synx handle
 *                               for interop.
 *                            b. SYNX_IMPORT_DMA_FENCE|SYNX_IMPORT_GLOBAL_FENCE
 *                               to import a dma fence and create global synx handle
 *                               for interop.
 * SYNX_IMPORT_EX_RELEASE   : Flag to inform relaxed invocation where release call
 *                            need not be called by client on this handle after import.
 *                            (NOT SUPPORTED)
 */
enum synx_import_flags {
	SYNX_IMPORT_LOCAL_FENCE  = 0x01,
	SYNX_IMPORT_GLOBAL_FENCE = 0x02,
	SYNX_IMPORT_SYNX_FENCE   = 0x04,
	SYNX_IMPORT_DMA_FENCE    = 0x08,
	SYNX_IMPORT_EX_RELEASE   = 0x10,
};

/**
 * enum synx_signal_status - Signal status
 *
 * SYNX_STATE_SIGNALED_SUCCESS : Signal success
 * SYNX_STATE_SIGNALED_CANCEL  : Signal cancellation
 * SYNX_STATE_SIGNALED_MAX     : Clients can send custom notification
 *                               beyond the max value (only positive)
 */
enum synx_signal_status {
	SYNX_STATE_SIGNALED_SUCCESS = 2,
	SYNX_STATE_SIGNALED_CANCEL  = 4,
	SYNX_STATE_SIGNALED_MAX     = 64,
};

/**
 * synx_callback - Callback invoked by external fence
 *
 * External fence dispatch the registered callback to notify
 * signal to synx framework.
 */
typedef void (*synx_callback)(s32 sync_obj, int status, void *data);

/**
 * synx_user_callback - Callback function registered by clients
 *
 * User callback registered for non-blocking wait. Dispatched when
 * synx object is signaled or timed-out with status of synx object.
 */
typedef void (*synx_user_callback_t)(u32 h_synx, int status, void *data);

/**
 * struct bind_operations - Function pointers that need to be defined
 *    to achieve bind functionality for external fence with synx obj
 *
 * @register_callback   : Function to register with external sync object
 * @deregister_callback : Function to deregister with external sync object
 * @enable_signaling    : Function to enable the signaling on the external
 *                        sync object (optional)
 * @signal              : Function to signal the external sync object
 */
struct bind_operations {
	int (*register_callback)(synx_callback cb_func,
		void *userdata, s32 sync_obj);
	int (*deregister_callback)(synx_callback cb_func,
		void *userdata, s32 sync_obj);
	int (*enable_signaling)(s32 sync_obj);
	int (*signal)(s32 sync_obj, u32 status);
};

/**
 * synx_bind_client_type : External fence supported for bind (NOT SUPPORTED)
 *
 * SYNX_TYPE_CSL : Camera CSL fence
 * SYNX_MAX_BIND_TYPES : Used for internal checks
 */
enum synx_bind_client_type {
	SYNX_TYPE_CSL = 0,
	SYNX_MAX_BIND_TYPES,
};

/**
 * struct synx_register_params - External registration parameters  (NOT SUPPORTED)
 *
 * @ops  : Bind operations struct
 * @name : External client name
 *         Only first 64 bytes are accepted, rest will be ignored
 * @type : Synx bind client type
 */
struct synx_register_params {
	struct bind_operations ops;
	char *name;
	enum synx_bind_client_type type;
};

/**
 * struct synx_queue_desc - Memory descriptor of the queue allocated by
 *                          the fence driver for each client during
 *                          register. (Clients need not pass any pointer
 *                          in synx_initialize_params. It is for future
 *                          use).
 *
 * @vaddr    : CPU virtual address of the queue.
 * @dev_addr : Physical address of the memory object.
 * @size     : Size of the memory.
 * @mem_data : Internal pointer with the attributes of the allocation.
 */
struct synx_queue_desc {
	void *vaddr;
	u64 dev_addr;
	u64 size;
	void *mem_data;
};

/**
 * enum synx_client_id : Unique identifier of the supported clients
 *
 * @SYNX_CLIENT_NATIVE   : Native Client
 * @SYNX_CLIENT_GFX_CTX0 : GFX Client 0
 * @SYNX_CLIENT_DPU_CTL0 : DPU Client 0
 * @SYNX_CLIENT_DPU_CTL1 : DPU Client 1
 * @SYNX_CLIENT_DPU_CTL2 : DPU Client 2
 * @SYNX_CLIENT_DPU_CTL3 : DPU Client 3
 * @SYNX_CLIENT_DPU_CTL4 : DPU Client 4
 * @SYNX_CLIENT_DPU_CTL5 : DPU Client 5
 * @SYNX_CLIENT_EVA_CTX0 : EVA Client 0
 * @SYNX_CLIENT_VID_CTX0 : Video Client 0
 * @SYNX_CLIENT_NSP_CTX0 : NSP Client 0
 * @SYNX_CLIENT_IFE_CTX0 : IFE Client 0
 * @SYNX_CLIENT_ICP_CTX0 : ICP Client 0
 */
enum synx_client_id {
	SYNX_CLIENT_NATIVE = 0,
	SYNX_CLIENT_GFX_CTX0,
	SYNX_CLIENT_DPU_CTL0,
	SYNX_CLIENT_DPU_CTL1,
	SYNX_CLIENT_DPU_CTL2,
	SYNX_CLIENT_DPU_CTL3,
	SYNX_CLIENT_DPU_CTL4,
	SYNX_CLIENT_DPU_CTL5,
	SYNX_CLIENT_EVA_CTX0,
	SYNX_CLIENT_VID_CTX0,
	SYNX_CLIENT_NSP_CTX0,
	SYNX_CLIENT_IFE_CTX0,
	SYNX_CLIENT_ICP_CTX0,
	SYNX_CLIENT_MAX,
};

/**
 * struct synx_session - Client session identifier
 *
 * @type   : Session type.
 *           Internal Member. (Do not access/modify)
 * @client : Pointer to client session
 *           Internal Member. (Do not access/modify)
 */
struct synx_session {
	u32 type;
	void *client;
};

/**
 * struct synx_initialization_params - Session params
 *
 * @name  : Client session name
 *          Only first 64 bytes are accepted, rest will be ignored
 * @ptr   : Memory descriptor of queue allocated by fence during
 *          device register. (filled by function)
 * @id    : Client identifier
 * @flags : Synx initialization flags
 */
struct synx_initialization_params {
	const char *name;
	struct synx_queue_desc *ptr;
	enum synx_client_id id;
	enum synx_init_flags flags;
};

/**
 * struct synx_create_params - Synx creation parameters
 *
 * @name     : Optional parameter associating a name with the synx
 *             object for debug purposes
 *             Only first 64 bytes are accepted,
 *             rest will be ignored
 * @h_synx   : Pointer to synx object handle (filled by function)
 * @fence    : Pointer to external dma fence or csl fence. (NOT SUPPORTED)
 * @flags    : Synx flags for customization
 */

struct synx_create_params {
	const char *name;
	u32 *h_synx;
	void *fence;
	enum synx_create_flags flags;
};

/**
 * enum synx_merge_flags - Handle merge flags
 *
 * SYNX_MERGE_LOCAL_FENCE   : Create local composite synx object. To be passed along
 *                            with SYNX_MERGE_NOTIFY_ON_ALL.
 * SYNX_MERGE_GLOBAL_FENCE  : Create global composite synx object. To be passed along
 *                            with SYNX_MERGE_NOTIFY_ON_ALL.
 * SYNX_MERGE_NOTIFY_ON_ALL : Notify on signaling of ALL objects.
 *                            Clients must pass:
 *                            a. SYNX_MERGE_LOCAL_FENCE|SYNX_MERGE_NOTIFY_ON_ALL
 *                               to create local composite synx object and notify
 *                               it when all child synx objects are signaled.
 *                            b. SYNX_MERGE_GLOBAL_FENCE|SYNX_MERGE_NOTIFY_ON_ALL
 *                               to create global composite synx object and notify
 *                               it when all child synx objects are signaled.
 * SYNX_MERGE_NOTIFY_ON_ANY : Notify on signaling of ANY object. (NOT SUPPORTED)
 */
enum synx_merge_flags {
	SYNX_MERGE_LOCAL_FENCE   = 0x01,
	SYNX_MERGE_GLOBAL_FENCE  = 0x02,
	SYNX_MERGE_NOTIFY_ON_ALL = 0x04,
	SYNX_MERGE_NOTIFY_ON_ANY = 0x08,
};

/*
 * struct synx_merge_params - Synx merge parameters
 *
 * @h_synxs      : Pointer to a array of synx handles to be merged
 * @flags        : Merge flags
 * @num_objs     : Number of synx handles to be merged (in array h_synxs).
 * @h_merged_obj : Merged synx handle (filled by function)
 */
struct synx_merge_params {
	u32 *h_synxs;
	enum synx_merge_flags flags;
	u32 num_objs;
	u32 *h_merged_obj;
};

/**
 * enum synx_import_type - Import type
 *
 * SYNX_IMPORT_INDV_PARAMS : Import filled with synx_import_indv_params struct
 * SYNX_IMPORT_ARR_PARAMS  : Import filled with synx_import_arr_params struct
 */
enum synx_import_type {
	SYNX_IMPORT_INDV_PARAMS = 0x01,
	SYNX_IMPORT_ARR_PARAMS  = 0x02,
};

/**
 * struct synx_import_indv_params - Synx import indv parameters
 *
 * @new_h_synxs : Pointer to new synx object
 *                (filled by the function)
 *                The new handle/s should be used by importing
 *                process for all synx api operations and
 *                for sharing with FW cores.
 * @flags       : Synx import flags
 * @fence       : Pointer to DMA fence fd or synx handle.
 */
struct synx_import_indv_params {
	u32 *new_h_synx;
	enum synx_import_flags flags;
	void *fence;
};

/**
 * struct synx_import_arr_params - Synx import arr parameters
 *
 * @list        : List of synx_import_indv_params
 * @num_fences  : Number of fences or synx handles to be imported
 */
struct synx_import_arr_params {
	struct synx_import_indv_params *list;
	u32 num_fences;
};

/**
 * struct synx_import_params - Synx import parameters
 *
 * @type : Import params type filled by client
 * @indv : Params to import an individual handle or fence
 * @arr  : Params to import an array of handles or fences
 */
struct synx_import_params {
	enum synx_import_type type;
	union {
		struct synx_import_indv_params indv;
		struct synx_import_arr_params  arr;
	};
};

/**
 * struct synx_callback_params - Synx callback parameters
 *
 * @h_synx         : Synx object handle
 * @cb_func        : Pointer to callback func to be invoked.
 * @userdata       : Opaque pointer passed back with callback as data
 * @cancel_cb_func : Pointer to callback to ack cancellation
 * @timeout_ms     : Timeout in ms. SYNX_NO_TIMEOUT if no timeout.
 */
struct synx_callback_params {
	u32 h_synx;
	synx_user_callback_t cb_func;
	void *userdata;
	synx_user_callback_t cancel_cb_func;
	u64 timeout_ms;
};

/* Kernel APIs */

/* synx_register_ops - Register operations for external synchronization  (NOT SUPPORTED)
 *
 * Register with synx for enabling external synchronization through bind
 *
 * @param params : Pointer to register params
 *
 * @return Status of operation. SYNX_SUCCESS in case of success.
 * -SYNX_INVALID will be returned if params are invalid.
 * -SYNX_NOMEM will be returned if bind ops cannot be registered due to
 * insufficient memory.
 * -SYNX_ALREADY will be returned if type already in use.
 */
int synx_register_ops(const struct synx_register_params *params);

/**
 * synx_deregister_ops - De-register external synchronization operations  (NOT SUPPORTED)
 *
 * @param params : Pointer to register params
 *
 * @return Status of operation. SYNX_SUCCESS in case of success.
 * -SYNX_INVALID will be returned if record not found.
 */
int synx_deregister_ops(const struct synx_register_params *params);

/**
 * synx_initialize - Initializes a new client session
 *
 * @param params : Pointer to session init params
 *
 * @return Client session pointer on success. NULL or error in case of failure.
 */
struct synx_session *synx_initialize(struct synx_initialization_params *params);

/**
 * synx_uninitialize - Destroys the client session
 *
 * @param session : Session ptr (returned from synx_initialize)
 *
 * @return Status of operation. Negative in case of error, SYNX_SUCCESS otherwise.
 */
int synx_uninitialize(struct synx_session *session);

/**
 * synx_create - Creates a synx object
 *
 * Creates a new synx obj and returns the handle to client. There can be
 * maximum of 4095 global synx handles or local synx handles across
 * sessions.
 *
 * @param session : Session ptr (returned from synx_initialize)
 * @param params  : Pointer to create params
 *
 * @return Status of operation. Negative in case of error, SYNX_SUCCESS otherwise.
 */
int synx_create(struct synx_session *session, struct synx_create_params *params);

/**
 * synx_async_wait - Registers a callback with a synx object
 *
 * Clients can register maximum of 64 callbacks functions per
 * synx session. Clients should register callback functions with minimal computation.
 *
 * @param session : Session ptr (returned from synx_initialize)
 * @param params  : Callback params.
 *                  cancel_cb_func in callback params is optional with this API.
 *
 * @return Status of operation. Negative in case of error, SYNX_SUCCESS otherwise.
 */
int synx_async_wait(struct synx_session *session, struct synx_callback_params *params);

/**
 * synx_cancel_async_wait - De-registers a callback with a synx object
 *
 * This API will cancel one instance of callback function (mapped
 * with userdata and h_synx) provided in cb_func of callback params.
 *
 * @param session : Session ptr (returned from synx_initialize)
 * @param params  : Callback params
 *
 * @return Status of operation.Negative in case of error, SYNX_SUCCESS otherwise.
 */
int synx_cancel_async_wait(struct synx_session *session,
	struct synx_callback_params *params);

/**
 * synx_signal - Signals a synx object with the status argument.
 *
 * This function will signal the synx object referenced by h_synx
 * and invoke any external binding synx objs.
 * The status parameter will indicate whether the entity
 * performing the signaling wants to convey an error case or a success case.
 *
 * @param session : Session ptr (returned from synx_initialize)
 * @param h_synx  : Synx object handle
 * @param status  : Status of signaling.
 *                  Clients can send custom signaling status
 *                  beyond SYNX_STATE_SIGNALED_MAX.
 *
 * @return Status of operation. Negative in case of error. SYNX_SUCCESS otherwise.
 */
int synx_signal(struct synx_session *session, u32 h_synx,
	enum synx_signal_status status);

/**
 * synx_merge - Merges multiple synx objects
 *
 * This function will merge multiple synx objects into a synx group.
 *
 * @param session : Session ptr (returned from synx_initialize)
 * @param params  : Merge params
 *
 * @return Status of operation. Negative in case of error. SYNX_SUCCESS otherwise.
 */
int synx_merge(struct synx_session *session, struct synx_merge_params *params);

/**
 * synx_wait - Waits for a synx object synchronously
 *
 * Does a wait on the synx object identified by h_synx for a maximum
 * of timeout_ms milliseconds. Must not be called from interrupt context as
 * this API can sleep.
 *
 * @param session    : Session ptr (returned from synx_initialize)
 * @param h_synx     : Synx object handle to be waited upon
 * @param timeout_ms : Timeout in ms
 *
 * @return Status of synx object if handle is signaled. -SYNX_INVAL if synx object
 * is in bad state or arguments are invalid, -SYNX_TIMEOUT if wait times out.
 */
int synx_wait(struct synx_session *session, u32 h_synx, u64 timeout_ms);

/**
 * synx_get_status - Returns the status of the synx object.
 *
 * This API should not be used in polling mode to know if the handle
 * is signaled or not.
 * Clients need to explicitly wait using synx_wait() or synx_async_wait()
 *
 * @param session : Session ptr (returned from synx_initialize)
 * @param h_synx  : Synx object handle
 *
 * @return Status of the synx object
 */
int synx_get_status(struct synx_session *session, u32 h_synx);

/**
 * synx_import - Imports (looks up) synx object from given handle or fence
 * *
 * @param session : Session ptr (returned from synx_initialize)
 * @param params  : Pointer to import params
 *
 * @return Status of operation. Negative in case of failure, SYNX_SUCCESS otherwise.
 */
int synx_import(struct synx_session *session, struct synx_import_params *params);

/**
 * synx_get_fence - Get the native fence backing the synx object
 *
 * Synx framework will take additional reference on dma fence and returns the native
 * fence. Clients need to release additional reference explicitly by calling kref_put.
 *
 * @param session : Session ptr (returned from synx_initialize)
 * @param h_synx  : Synx object handle
 *
 * @return Fence pointer in case of success and NULL in case of failure.
 */
void *synx_get_fence(struct synx_session *session, u32 h_synx);

/**
 * synx_release - Release the synx object.
 *
 * Every created, imported or merged synx object should be released.
 *
 * @param session : Session ptr (returned from synx_initialize)
 * @param h_synx  : Synx object handle to be destroyed
 *
 * @return Status of operation. Negative in case of error. SYNX_SUCCESS otherwise.
 */
int synx_release(struct synx_session *session, u32 h_synx);

/**
 * synx_recover - Recover any possible handle leaks
 *
 * Function should be called on HW hang/reset to
 * recover the Synx handles shared. This cleans up
 * synx handles owned by subsystem under hang/reset, and avoids
 * potential resource leaks.
 *
 * Function does not destroy the session, but only
 * recover synx handles belonging to the session.
 * Synx session would still be active and clients
 * need to destroy the session explicitly through
 * synx_uninitialize API.
 *
 * All the unsignaled handles owned/imported by the core at the time of reset
 * will be signaled by synx framework on behalf of hung core with SYNX_STATE_SIGNALED_SSR.
 *
 * @param id : Client ID of core to recover
 *
 * @return Status of operation. Negative in case of error. SYNX_SUCCESS otherwise.
 */
int synx_recover(enum synx_client_id id);

#endif /* __SYNX_API_H__ */
