/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

struct qpace_transfer_descriptor {
	uint64_t dst_addr;
	uint64_t src_addr;
	uint64_t size:20;

	uint64_t ed_reserved:44;

	uint32_t crc;
	uint32_t crc_enable:1;
	uint32_t crc_mode:1;
	uint32_t crc_keep:1;

	uint32_t pcm:1;

	uint32_t operation:3;

	uint32_t unused_1:1;

	uint32_t page_count:6;

	uint32_t sctx:2;
	uint32_t dctx:2;

	uint32_t beosc:1;
	uint32_t bei:1;

	uint32_t rf:1;
	uint32_t wa:1;

	uint32_t event_ring_id:4;

	uint32_t unused_2:4;

	uint32_t pcd:1;

	uint32_t noop:1;
};

#define CYCLE_BIT_MASK BIT(30)
struct qpace_event_descriptor {
	uint64_t td_addr;
	uint64_t out_addr;
	union {
		uint64_t flags;

		struct {
			uint64_t size:20;
			uint64_t completion_code:4;
			uint64_t tr_ring_id:4;
			uint64_t unused:2;
			uint64_t cycle_bit:1;
			uint64_t replication_found:1;
			uint64_t rep_word:32;
		};
	};

	uint32_t crc;

	uint32_t reserved;
};

enum ring_vals {
	COMPRESS_RING,
	COPY_RING,
	NUM_RINGS,
};

#define RING_SIZE_ORDER 0
#define DESCRIPTOR_SIZE 32
#define DESCRIPTORS_PER_RING ((PAGE_SIZE << RING_SIZE_ORDER) / DESCRIPTOR_SIZE)

typedef void (*process_ed_fn)(struct qpace_event_descriptor *ed, int ed_index);

#if IS_ENABLED(CONFIG_QTI_PAGE_COMPRESSION_ENGINE)

/*
 * qpace_urgent_compress() - compress a page
 * @input_addr: Input address of the page to be compressed
 * @output_addr: Address to output the compressed page to
 *
 * Compress a page of PAGE_SIZE bytes and output it to @output_addr.
 *
 * Returns the size of the compressed page in bytes if successful and returns
 * -EINVAL if compression failed.
 */
int qpace_urgent_compress(phys_addr_t input_addr,
			  phys_addr_t output_addr);


/*
 * qpace_urgent_decompress() - decompress a page that was compressed by QPaCE
 * @input_addr: Input address of the compressed page
 * @output_addr: Address to output the decompressed page to
 * @input_size: size of the compressed input-page
 *
 * Decompressed a compressed page previously compressed by QPaCE. The output is
 * expected to be PAGE_SIZE bytes.
 *
 * Returns the size of the decompressed page in bytes if successful (though the
 * caller should know its page sized), and returns -EINVAL if decompression
 * failed.
 */
int qpace_urgent_decompress(phys_addr_t input_addr,
			    phys_addr_t output_addr,
			    size_t input_size);

/*
 * qpace_queue_copy() - queue a copy operation
 * @tr_num: The transfer ring to queue the request to
 * @src_addr: Input address of the object to copy
 * @dst_addr: Address to copy the object to
 * @copy_size: Number of bytes to copy
 *
 * Queue a copy request to a chosen transfer ring
 */
int qpace_queue_copy(int tr_num, phys_addr_t src_addr,
		      phys_addr_t dst_addr, size_t copy_size);

/*
 * qpace_queue_compress() - queue a compression operation
 * @tr_num: The transfer ring to queue the request to
 * @src_addr: Input address of the object to copy
 * @dst_addr: Address to copy the object to
 *
 * Queue a compression request to a chosen transfer ring
 */
int qpace_queue_compress(int tr_num, phys_addr_t src_addr,
			  phys_addr_t dst_addr);

/*
 * qpace_check_compress_err() - check a compression result
 * @ed: The event descriptor of the compression request
 *
 * Return: true if there is error, false otherwise.
 */
bool qpace_check_compress_err(struct qpace_event_descriptor *ed);
/*
 * qpace_trigger_tr() - Have QPaCE process outstanding TD in the transfer ring
 * @tr_num: Transfer ring to have qpace take submissions from for processing
 *
 * Have QPaCE process outstanding transfer descriptors / TDs in the transfer ring,
 * i.e. QPaCE will process the requests submitted from the qpace_queue_* operations.
 * This boils down to updating a HW register that points to the end of the valid
 * in the ring buffer.
 *
 * Return: positive for the requests number that submitted to HW, 0 if the ring had
 * no items to submit.
 */
size_t qpace_trigger_tr(int tr_num);

/*
 * qpace_wait_for_tr_consumption() - Wait for event completion interrupt for @tr_num
 * @tr_num: Transfer ring to have qpace take submissions from for processing
 * @nr_trigger: requests triggered in one batch
 * @no_sleep: If true, poll on the relevant completion event, go to sleep otherwise.
 *
 * Wait for there to be a completion event for the event ring corresponding to the
 * transfer ring @tr_num. Can be sleeping or non-sleeping based on @no_sleep.
 */
void qpace_wait_for_tr_consumption(int tr_num, size_t nr_trigger, bool no_sleep);

/*
 * qpace_consume_er() - process completed event descriptors in the event ring @er_num
 * @er_num: Event ring for which we want process completed event descriptors
 * @success_handler: Callback for processing a successful event
 * @fail_handler: Callback for processing a failed event
 * @consume: If true, consume the event descriptors, otherwise just iterate over them.
 *
 * Process the completed events in the event ring corresponding to @er_num
 *
 * Return: the number of consumed entries in the ring correspond to @er_num
 */
int qpace_consume_er(int er_num,
		     process_ed_fn success_handler,
		     process_ed_fn fail_handler, bool consume);

/*
 *  is_qpace_enabled() - return true if qpace is enabled
 */
bool is_qpace_dev_available(void);

/*
 * get_qpace() - grab an additional reference to QPaCE
 *
 * Grabs an additional reference to QPaCE to stop it from entering
 * power collapse, even if all of its rings are emptied. This can be used
 * to prevent uncessarily re-initialization of QPaCE rings, and to avoid
 * the overhead of turning on the clocks again.
 */
void get_qpace(void);

/*
 * put_qpace() - Drop the reference obtained by get_qpace().
 */
void put_qpace(void);

#else /* CONFIG_QTI_PAGE_COMPRESSION_ENGINE */


static inline bool is_qpace_dev_available(void)
{
	return false;
}

static inline int qpace_urgent_compress(phys_addr_t input_addr,
					phys_addr_t output_addr)
{
	return -EINVAL;
}

static inline int qpace_urgent_decompress(phys_addr_t input_addr,
					  phys_addr_t output_addr,
					  size_t input_size)
{
	return -EINVAL;
}

static inline int qpace_queue_copy(int tr_num, phys_addr_t src_addr,
				    phys_addr_t dst_addr, size_t copy_size)
{
	return -EINVAL;
}

static inline int qpace_queue_compress(int tr_num, phys_addr_t src_addr,
					phys_addr_t dst_addr)
{
	return -EINVAL;
}

static inline bool qpace_check_compress_err(struct qpace_event_descriptor *ed)
{
	return false;
}

static inline size_t qpace_trigger_tr(int tr_num)
{
	return 0;
}

static inline void qpace_wait_for_tr_consumption(int tr_num, size_t nr_trigger, bool no_sleep)
{

}

static inline int qpace_consume_er(int er_num,
				   process_ed_fn success_handler,
				   process_ed_fn fail_handler, bool consume)
{
	return 0;
}

void get_qpace(void)
{

}

void put_qpace(void)
{

}

#endif
