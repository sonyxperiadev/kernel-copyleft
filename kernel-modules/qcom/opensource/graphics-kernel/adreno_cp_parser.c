// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2013-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/slab.h>

#include "adreno.h"
#include "adreno_cp_parser.h"
#include "adreno_pm4types.h"
#include "adreno_snapshot.h"

#define MAX_IB_OBJS 1000
#define NUM_SET_DRAW_GROUPS 32
#define IB_LEVEL_1 1
#define IB_LEVEL_2 2
#define IB_LEVEL_3 3

struct snapshot_ib_meta metadata;

struct set_draw_state {
	uint64_t cmd_stream_addr;
	uint64_t cmd_stream_dwords;
};

/* List of variables used when parsing an IB */
struct ib_parser_variables {
	/* List of registers containing addresses and their sizes */
	unsigned int cp_addr_regs[ADRENO_CP_ADDR_MAX];
	/* 32 groups of command streams in set draw state packets */
	struct set_draw_state set_draw_groups[NUM_SET_DRAW_GROUPS];
};

static int adreno_ib_find_objs(struct kgsl_device *device,
				struct kgsl_process_private *process,
				uint64_t gpuaddr, uint64_t dwords,
				uint64_t ib2base, int obj_type,
				struct adreno_ib_object_list *ib_obj_list,
				int ib_level);

static int ib_parse_type7_set_draw_state(struct kgsl_device *device,
	unsigned int *ptr,
	struct kgsl_process_private *process,
	struct adreno_ib_object_list *ib_obj_list);

/*
 * adreno_ib_merge_range() - Increases the address range tracked by an ib
 * object
 * @ib_obj: The ib object
 * @gpuaddr: The start address which is to be merged
 * @size: Size of the merging address
 */
static void adreno_ib_merge_range(struct adreno_ib_object *ib_obj,
		uint64_t gpuaddr, uint64_t size)
{
	uint64_t addr_end1 = ib_obj->gpuaddr + ib_obj->size;
	uint64_t addr_end2 = gpuaddr + size;

	if (gpuaddr < ib_obj->gpuaddr)
		ib_obj->gpuaddr = gpuaddr;
	if (addr_end2 > addr_end1)
		ib_obj->size = addr_end2 - ib_obj->gpuaddr;
	else
		ib_obj->size = addr_end1 - ib_obj->gpuaddr;
}

/*
 * adreno_ib_check_overlap() - Checks if an address range overlap
 * @gpuaddr: The start address range to check for overlap
 * @size: Size of the address range
 * @type: The type of address range
 * @ib_obj_list: The list of address ranges to check for overlap
 *
 * Checks if an address range overlaps with a list of address ranges
 * Returns the entry from list which overlaps else NULL
 */
static struct adreno_ib_object *adreno_ib_check_overlap(uint64_t gpuaddr,
		uint64_t size, int type,
		struct adreno_ib_object_list *ib_obj_list)
{
	struct adreno_ib_object *ib_obj;
	int i;

	for (i = 0; i < ib_obj_list->num_objs; i++) {
		ib_obj = &(ib_obj_list->obj_list[i]);
		if ((type == ib_obj->snapshot_obj_type) &&
			kgsl_addr_range_overlap(ib_obj->gpuaddr, ib_obj->size,
			gpuaddr, size))
			/* regions overlap */
			return ib_obj;
	}
	return NULL;
}

/*
 * adreno_ib_add() - Add a gpuaddress range to list
 * @process: Process in which the gpuaddress is mapped
 * @type: The type of address range
 * @ib_obj_list: List of the address ranges in which the given range is to be
 * added
 *
 * Add a gpuaddress range as an ib object to a given list after checking if it
 * overlaps with another entry on the list. If it conflicts then change the
 * existing entry to incorporate this range
 *
 * Returns 0 on success else error code
 */
static int adreno_ib_add(struct kgsl_process_private *process,
				uint64_t gpuaddr, int type,
				struct adreno_ib_object_list *ib_obj_list)
{
	uint64_t size;
	struct adreno_ib_object *ib_obj;
	struct kgsl_mem_entry *entry;

	if (ib_obj_list->num_objs >= MAX_IB_OBJS)
		return -E2BIG;

	entry = kgsl_sharedmem_find(process, gpuaddr);
	if (!entry)
		/*
		 * Do not fail if gpuaddr not found, we can continue
		 * to search for other objects even if few objects are
		 * not found
		 */
		return 0;

	size = entry->memdesc.size;
	gpuaddr = entry->memdesc.gpuaddr;

	ib_obj = adreno_ib_check_overlap(gpuaddr, size, type, ib_obj_list);
	if (ib_obj) {
		adreno_ib_merge_range(ib_obj, gpuaddr, size);
		kgsl_mem_entry_put(entry);
	} else {
		adreno_ib_init_ib_obj(gpuaddr, size, type, entry,
			&(ib_obj_list->obj_list[ib_obj_list->num_objs]));
		ib_obj_list->num_objs++;
		/* Skip reclaim for the memdesc until it is dumped */
		entry->memdesc.priv |= KGSL_MEMDESC_SKIP_RECLAIM;
	}
	return 0;
}

/*
 * ib_add_type0_entries() - Add memory objects to list
 * @device: The device on which the IB will execute
 * @process: The process in which IB is mapped
 * @ib_obj_list: The list of gpu objects
 * @ib_parse_vars: addresses ranges found in type0 packets
 *
 * Add memory objects to given list that are found in type0 packets
 * Returns 0 on success else 0
 */
static int ib_add_type0_entries(struct kgsl_device *device,
	struct kgsl_process_private *process,
	struct adreno_ib_object_list *ib_obj_list,
	struct ib_parser_variables *ib_parse_vars)
{
	int ret = 0;
	int i;
	int vfd_end;
	unsigned int mask;
	/* First up the visiblity stream buffer */
	mask = 0xFFFFFFFF;
	for (i = ADRENO_CP_ADDR_VSC_PIPE_DATA_ADDRESS_0;
		i < ADRENO_CP_ADDR_VSC_PIPE_DATA_LENGTH_7; i++) {
		if (ib_parse_vars->cp_addr_regs[i]) {
			ret = adreno_ib_add(process,
				ib_parse_vars->cp_addr_regs[i] & mask,
				SNAPSHOT_GPU_OBJECT_GENERIC,
				ib_obj_list);
			if (ret)
				return ret;
			ib_parse_vars->cp_addr_regs[i] = 0;
			ib_parse_vars->cp_addr_regs[i + 1] = 0;
			i++;
		}
	}

	vfd_end = ADRENO_CP_ADDR_VFD_FETCH_INSTR_1_15;
	for (i = ADRENO_CP_ADDR_VFD_FETCH_INSTR_1_0;
		i <= vfd_end; i++) {
		if (ib_parse_vars->cp_addr_regs[i]) {
			ret = adreno_ib_add(process,
				ib_parse_vars->cp_addr_regs[i],
				SNAPSHOT_GPU_OBJECT_GENERIC,
				ib_obj_list);
			if (ret)
				return ret;
			ib_parse_vars->cp_addr_regs[i] = 0;
		}
	}

	if (ib_parse_vars->cp_addr_regs[ADRENO_CP_ADDR_VSC_SIZE_ADDRESS]) {
		ret = adreno_ib_add(process,
			ib_parse_vars->cp_addr_regs[
				ADRENO_CP_ADDR_VSC_SIZE_ADDRESS] & mask,
			SNAPSHOT_GPU_OBJECT_GENERIC, ib_obj_list);
		if (ret)
			return ret;
		ib_parse_vars->cp_addr_regs[
			ADRENO_CP_ADDR_VSC_SIZE_ADDRESS] = 0;
	}
	mask = 0xFFFFFFE0;
	for (i = ADRENO_CP_ADDR_SP_VS_PVT_MEM_ADDR;
		i <= ADRENO_CP_ADDR_SP_FS_OBJ_START_REG; i++) {
		ret = adreno_ib_add(process,
			ib_parse_vars->cp_addr_regs[i] & mask,
			SNAPSHOT_GPU_OBJECT_GENERIC, ib_obj_list);
		if (ret)
			return ret;
		ib_parse_vars->cp_addr_regs[i] = 0;
	}
	return ret;
}

/*
 * Parse all the type7 opcode packets that may contain important information,
 * such as additional GPU buffers to grab or a draw initator
 */

static int ib_parse_type7(struct kgsl_device *device, unsigned int *ptr,
	struct kgsl_process_private *process,
	struct adreno_ib_object_list *ib_obj_list,
	struct ib_parser_variables *ib_parse_vars)
{
	int opcode = cp_type7_opcode(*ptr);

	switch (opcode) {
	case CP_SET_DRAW_STATE:
		return ib_parse_type7_set_draw_state(device, ptr, process,
					ib_obj_list);
	}

	return 0;
}

static int ib_parse_type7_set_draw_state(struct kgsl_device *device,
	unsigned int *ptr,
	struct kgsl_process_private *process,
	struct adreno_ib_object_list *ib_obj_list)
{
	int size = type7_pkt_size(*ptr);
	int i;
	int ret = 0;
	int flags;
	uint64_t cmd_stream_dwords;
	uint64_t cmd_stream_addr;

	/*
	 * size is the size of the packet that does not include the DWORD
	 * for the packet header, we only want to loop here through the
	 * packet parameters from ptr[1] till ptr[size] where ptr[0] is the
	 * packet header. In each loop we look at 3 DWORDS hence increment
	 * loop counter by 3 always
	 */
	for (i = 1; i <= size; i += 3) {
		/* take action based on flags */
		flags = (ptr[i] & 0x000F0000) >> 16;

		/*
		 * dirty flag or no flags both mean we need to load it for
		 * next draw. No flags is used when the group is activated
		 * or initialized for the first time in the IB
		 */
		if (flags & 0x1 || !flags) {
			cmd_stream_dwords = ptr[i] & 0x0000FFFF;
			cmd_stream_addr = ptr[i + 2];
			cmd_stream_addr = cmd_stream_addr << 32 | ptr[i + 1];
			if (cmd_stream_dwords)
				ret = adreno_ib_find_objs(device, process,
					cmd_stream_addr, cmd_stream_dwords,
					0, SNAPSHOT_GPU_OBJECT_DRAW,
					ib_obj_list, IB_LEVEL_2);
			if (ret)
				break;
			continue;
		}
		/* load immediate */
		if (flags & 0x8) {
			uint64_t gpuaddr = ptr[i + 2];

			gpuaddr = gpuaddr << 32 | ptr[i + 1];
			ret = adreno_ib_find_objs(device, process,
				gpuaddr, (ptr[i] & 0x0000FFFF),
				0, SNAPSHOT_GPU_OBJECT_IB,
				ib_obj_list, IB_LEVEL_2);
			if (ret)
				break;
		}
	}
	return ret;
}

/*
 * adreno_cp_parse_ibn() - Wrapper function around IBn parsing
 * @device: Device pointer
 * @process: Process in which the IB is allocated
 * @gpuaddr: IB gpuaddr
 * @dwords: IB size in dwords
 * @ibbase: Base address of active IB
 * @ib_obj_list: List of objects found in IB
 * @ib_level: The level from which function is called from IBn-1
 *
 * Function does some checks to ensure that IBn parsing is called from IBn-1
 * and then calls the function to find objects in IBn.
 */
static int adreno_cp_parse_ibn(struct kgsl_device *device,
			struct kgsl_process_private *process,
			u64 gpuaddr, u64 dwords, u64 ibbase,
			struct adreno_ib_object_list *ib_obj_list,
			int ib_level)
{
	int i;

	/*
	 * We can only expect an IBn in IBn-1, if we are
	 * already processing an IBn then return error
	 */
	if (ib_level >= IB_LEVEL_3)
		return -EINVAL;

	/* Save current IBn statically */
	if (ibbase == gpuaddr)
		kgsl_snapshot_push_object(device, process, gpuaddr, dwords);
	/*
	 * Only try to find sub objects if this IBn has
	 * not been processed already
	 */
	for (i = 0; i < ib_obj_list->num_objs; i++) {
		struct adreno_ib_object *ib_obj = &(ib_obj_list->obj_list[i]);

		if ((ib_obj->snapshot_obj_type == SNAPSHOT_GPU_OBJECT_IB) &&
			(gpuaddr >= ib_obj->gpuaddr) &&
			(gpuaddr + dwords * sizeof(unsigned int) <=
			ib_obj->gpuaddr + ib_obj->size))
			return 0;
	}

	return adreno_ib_find_objs(device, process, gpuaddr, dwords, ibbase,
		SNAPSHOT_GPU_OBJECT_IB, ib_obj_list, ib_level + 1);
}

static s64 get_ib_base(struct adreno_device *adreno_dev, int ib_level)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	s64 ibbase = 0;

	switch (ib_level) {
	case IB_LEVEL_2:
		ibbase = metadata.ib2base;
		break;
	case IB_LEVEL_3:
		if (!adreno_is_a5xx(adreno_dev) && !adreno_is_a6xx(adreno_dev))
			ibbase = metadata.ib3base;
		break;
	default:
		/* Invalid IB level */
		dev_err(device->dev, "Invalid IB level %d\n", ib_level);
		return -EINVAL;
	}

	return ibbase;
}

/*
 * adreno_ib_find_objs() - Find all IB objects in a given IB
 * @device: The device pointer on which the IB executes
 * @process: The process in which the IB and all contained objects are mapped.
 * @gpuaddr: The gpu address of the IB
 * @ib2base: IB2 base address
 * @dwords: Size of ib in dwords
 * @obj_type: The object type can be either an IB or a draw state sequence
 * @ib_obj_list: The list in which the IB and the objects in it are added.
 * @ib_level: Indicates if IB1 or IB2 is being processed
 *
 * Finds all IB objects in a given IB and puts then in a list. Can be called
 * recursively for the IBn in the IBn-1
 * Returns 0 on success else error code
 */
static int adreno_ib_find_objs(struct kgsl_device *device,
				struct kgsl_process_private *process,
				uint64_t gpuaddr, uint64_t dwords,
				uint64_t ib2base, int obj_type,
				struct adreno_ib_object_list *ib_obj_list,
				int ib_level)
{
	int ret = 0;
	uint64_t rem = dwords;
	int i;
	struct ib_parser_variables ib_parse_vars;
	unsigned int *src;
	struct adreno_ib_object *ib_obj;
	struct kgsl_mem_entry *entry;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	/* check that this IB is not already on list */
	for (i = 0; i < ib_obj_list->num_objs; i++) {
		ib_obj = &(ib_obj_list->obj_list[i]);
		if ((obj_type == ib_obj->snapshot_obj_type) &&
			(ib_obj->gpuaddr <= gpuaddr) &&
			((ib_obj->gpuaddr + ib_obj->size) >=
			(gpuaddr + (dwords << 2))))
			return 0;
	}

	entry = kgsl_sharedmem_find(process, gpuaddr);
	if (!entry)
		return -EINVAL;

	if (!kgsl_gpuaddr_in_memdesc(&entry->memdesc, gpuaddr, (dwords << 2))) {
		kgsl_mem_entry_put(entry);
		return -EINVAL;
	}

	src = kgsl_gpuaddr_to_vaddr(&entry->memdesc, gpuaddr);
	if (!src) {
		kgsl_mem_entry_put(entry);
		return -EINVAL;
	}

	memset(&ib_parse_vars, 0, sizeof(struct ib_parser_variables));

	ret = adreno_ib_add(process, gpuaddr, obj_type, ib_obj_list);
	if (ret)
		goto done;

	for (i = 0; rem > 0; rem--, i++) {
		int pktsize;

		if (pkt_is_type4(src[i]))
			pktsize = type4_pkt_size(src[i]);
		else if (pkt_is_type7(src[i]))
			pktsize = type7_pkt_size(src[i]);

		/*
		 * If the packet isn't a type 1, type 3, type 4 or type 7 then
		 * don't bother parsing it - it is likely corrupted
		 */
		else
			break;

		/* Check if rem is less than pktsize before decrementing */
		if (rem < pktsize)
			break;

		if (pkt_is_type7(src[i])) {
			if (adreno_cmd_is_ib(adreno_dev, src[i])) {
				u64 size = src[i + 3];
				u64 gpuaddribn = ((u64)(src[i + 2]) << 32) | src[i + 1];
				s64 next_ibbase = get_ib_base(adreno_dev, ib_level + IB_LEVEL_1);

				if (next_ibbase == -EINVAL)
					goto done;
				ret = adreno_cp_parse_ibn(device, process,
					gpuaddribn, size, (u64)next_ibbase, ib_obj_list, ib_level);
				if (ret)
					goto done;
			} else {
				ret = ib_parse_type7(device, &src[i], process,
						ib_obj_list,
						&ib_parse_vars);
				/*
				 * If the parse function failed (probably
				 * because of a bad decode) then bail out and
				 * just capture the binary IB data
				 */

				if (ret)
					goto done;
			}
		}

		i += pktsize;
		rem -= pktsize;
	}

done:
	/*
	 * For set draw objects there may not be a draw_indx packet at its end
	 * to signal that we need to save the found objects in it, so just save
	 * it here.
	 */
	if (!ret && SNAPSHOT_GPU_OBJECT_DRAW == obj_type)
		ret = ib_add_type0_entries(device, process, ib_obj_list,
			&ib_parse_vars);

	kgsl_memdesc_unmap(&entry->memdesc);
	kgsl_mem_entry_put(entry);
	return ret;
}


/*
 * adreno_ib_create_object_list() - Find all the memory objects in IB
 * @device: The device pointer on which the IB executes
 * @process: The process in which the IB and all contained objects are mapped
 * @gpuaddr: The gpu address of the IB
 * @dwords: Size of ib in dwords
 * @ib2base: Base address of active IB2
 * @ib_obj_list: The list in which the IB and the objects in it are added.
 *
 * Find all the memory objects that an IB needs for execution and place
 * them in a list including the IB.
 * Returns the ib object list. On success 0 is returned, on failure error
 * code is returned along with number of objects that was saved before
 * error occurred. If no objects found then the list pointer is set to
 * NULL.
 */
int adreno_ib_create_object_list(struct kgsl_device *device,
		struct kgsl_process_private *process,
		uint64_t gpuaddr, uint64_t dwords, uint64_t ib2base,
		struct adreno_ib_object_list **out_ib_obj_list)
{
	int ret = 0;
	struct adreno_ib_object_list *ib_obj_list;

	if (!out_ib_obj_list)
		return -EINVAL;

	*out_ib_obj_list = NULL;

	ib_obj_list = kzalloc(sizeof(*ib_obj_list), GFP_KERNEL);
	if (!ib_obj_list)
		return -ENOMEM;

	ib_obj_list->obj_list = vmalloc(MAX_IB_OBJS *
					sizeof(struct adreno_ib_object));

	if (!ib_obj_list->obj_list) {
		kfree(ib_obj_list);
		return -ENOMEM;
	}

	ret = adreno_ib_find_objs(device, process, gpuaddr, dwords, ib2base,
		SNAPSHOT_GPU_OBJECT_IB, ib_obj_list, IB_LEVEL_1);

	/* Even if there was an error return the remaining objects found */
	if (ib_obj_list->num_objs)
		*out_ib_obj_list = ib_obj_list;

	return ret;
}

/*
 * adreno_ib_destroy_obj_list() - Destroy an ib object list
 * @ib_obj_list: List to destroy
 *
 * Free up all resources used by an ib_obj_list
 */
void adreno_ib_destroy_obj_list(struct adreno_ib_object_list *ib_obj_list)
{
	int i;

	if (!ib_obj_list)
		return;

	for (i = 0; i < ib_obj_list->num_objs; i++) {
		if (ib_obj_list->obj_list[i].entry)
			kgsl_mem_entry_put(ib_obj_list->obj_list[i].entry);
	}
	vfree(ib_obj_list->obj_list);
	kfree(ib_obj_list);
}
