// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/slab.h>
#include <linux/random.h>
#include <linux/vmalloc.h>

#include "synx_debugfs.h"
#include "synx_debugfs_util.h"
#include "synx_util.h"
#include "synx_private.h"
#include "synx_global.h"

#define MAX_CUSTOM_STATUS ((1UL << 32) - 1)

char *synx_debugfs_util_get_state_name(u32 status)
{
	char *state;

	if (status == 0)
		state = "INV";
	else if (status == 1)
		state = "ACT";
	else if (status == 2)
		state = "SUC";
	else if (status == 3)
		state = "ERR";
	else if (status == 4)
		state = "CAN";
	else if (status == 5)
		state = "EXT";
	else if (status == 6)
		state = "SSR";
	else if (status > 64 && status <= MAX_CUSTOM_STATUS)
		state = "CUS";
	else
		state = "???";

	return state;
}

static int synx_debugfs_util_get_client_data(struct synx_client *client)
{
	if (IS_ERR_OR_NULL(client))
		return -SYNX_NOENT;
	kref_get(&client->refcount);
	return SYNX_SUCCESS;
}

static void synx_debugfs_util_put_client_data(struct synx_client *client)
{
	if (!IS_ERR_OR_NULL(client))
		kref_put(&client->refcount, synx_client_destroy);
}

static int synx_debugfs_util_get_handle(struct synx_handle_coredata *handle_coredata)
{
	if (IS_ERR_OR_NULL(handle_coredata))
		return -SYNX_NOENT;
	kref_get(&handle_coredata->refcount);
	return SYNX_SUCCESS;
}

static void synx_debugfs_util_put_handle(struct synx_handle_coredata *handle_coredata)
{
	if (!IS_ERR_OR_NULL(handle_coredata))
		kref_put(&handle_coredata->refcount, synx_util_destroy_handle);
}

static int synx_debugfs_util_get_CSL_fence_entry(struct synx_entry_64 *entry)
{
	if (IS_ERR_OR_NULL(entry))
		return -SYNX_NOENT;
	kref_get(&entry->refcount);
	return SYNX_SUCCESS;
}

static void synx_debugfs_util_put_CSL_fence_entry(struct synx_entry_64 *entry)
{
	if (!IS_ERR_OR_NULL(entry))
		kref_put(&entry->refcount, synx_util_destroy_data);
}

bool synx_debugfs_util_is_valid_global_shared_memory_entry(struct synx_global_coredata *entry,
	u32 idx)
{
	int i;

	if (!entry || entry->handle != idx)
		return false;
	if (entry->status || entry->handle || entry->refcount ||
	 entry->num_child || entry->subscribers || entry->waiters)
		return true;

	for (i = 0; i < SYNX_GLOBAL_MAX_PARENTS; i++) {
		if (entry->parents[i])
			return true;
	}
	return false;
}

static bool synx_debugfs_util_is_valid_dma_handle_range(struct synx_fence_entry *fence_entry)
{
	if ((fence_entry->g_handle >= lower_handle_id &&
		fence_entry->g_handle <= upper_handle_id) ||
		(fence_entry->l_handle >= lower_handle_id &&
		fence_entry->l_handle <= upper_handle_id))
		return true;
	return false;
}

static void synx_debugfs_util_print_map_column_values(char **cur,
	char **end,
	struct synx_map_entry *entry)
{
	if (synx_columns & STATUS_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "\t\t%s", synx_debugfs_util_get_state_name
		(synx_util_get_object_status(entry->synx_obj)));
	if (synx_columns & ID_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "\t\t  %x", entry->key);
	if (synx_columns & REF_CNT_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "\t  %d", kref_read(&entry->refcount));
	if (synx_columns & BOUND_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "\t  %d", entry->synx_obj->num_bound_synxs);
	if (synx_columns & GLOBAL_IDX_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "\t\t  %d", entry->synx_obj->global_idx);
	if (synx_columns & MAP_CNT_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "\t\t  %d", entry->synx_obj->map_count);
	SYNX_CONSOLE_LOG(*cur, *end, "\n\t-------------------------------------");
	SYNX_CONSOLE_LOG(*cur, *end, "-----------------------------------------");
	SYNX_CONSOLE_LOG(*cur, *end, "-----------\n");
}

void synx_debugfs_util_print_hash_table(char **cur,
	char **end,
	bool is_global)
{
	struct synx_map_entry *map_entry = NULL;
	struct synx_coredata *synx_obj = NULL;
	u32 key;

	if (is_global)
		SYNX_CONSOLE_LOG(*cur, *end,
			"\n\t-------------GLOBAL MAP TABLE------------\n");
	else
		SYNX_CONSOLE_LOG(*cur, *end,
			"\n\t-------------LOCAL MAP TABLE------------\n");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\t\t");

	if (synx_columns & STATUS_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "|  STATUS  |");
	if (synx_columns & ID_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "|  HANDLE  |");
	if (synx_columns & REF_CNT_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "|  REF CNT |");
	if (synx_columns & BOUND_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "| NUM BOUND |");
	if (synx_columns & GLOBAL_IDX_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "| GLOBAL INDEX |");
	if (synx_columns & MAP_CNT_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "|  MAP CNT |");
	SYNX_CONSOLE_LOG(*cur, *end, "\n");

	for (key = lower_handle_id; key <= upper_handle_id; key++) {
		map_entry = synx_util_get_map_entry(key);
		if (IS_ERR_OR_NULL(map_entry) || IS_ERR_OR_NULL(map_entry->synx_obj) ||
			(is_global ^ synx_util_is_global_handle(key))) {
			synx_util_release_map_entry(map_entry);
			continue;
		}
		synx_obj = map_entry->synx_obj;
		synx_util_get_object(synx_obj);
		mutex_lock(&synx_obj->obj_lock);
		synx_debugfs_util_print_map_column_values(cur, end, map_entry);
		mutex_unlock(&synx_obj->obj_lock);
		synx_util_put_object(synx_obj);
		synx_util_release_map_entry(map_entry);
	}
}

void synx_debugfs_util_print_dma_fence(char **cur,
	char **end)
{
	struct synx_fence_entry *curr = NULL;
	struct hlist_node *tmp;
	struct dma_fence *fence_entry = NULL;
	u32 map_itr;

	SYNX_CONSOLE_LOG(*cur, *end, "\n\t-------------DMA FENCE MAP TABLE------------\n");

	if (synx_columns & FENCE_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "|         DMA FENCE           |");
	if (synx_columns & STATUS_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "|   STATUS   |");
	if (synx_columns & ID_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "|         HANDLE          |");
	if (synx_columns & REF_CNT_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "|    REF CNT   |");
	SYNX_CONSOLE_LOG(*cur, *end, "\n");

	spin_lock_bh(&synx_dev->native->fence_map_lock);
	hash_for_each_safe(synx_dev->native->fence_map, map_itr, tmp, curr, node) {
		if (IS_ERR_OR_NULL(curr))
			continue;
		fence_entry = (struct dma_fence *)curr->key;
		dma_fence_get(fence_entry);
		if (synx_debugfs_util_is_valid_dma_handle_range(curr)) {
			if (synx_columns & FENCE_COLUMN)
				SYNX_CONSOLE_LOG(*cur, *end, "\t%p", fence_entry);
			if (synx_columns & STATUS_COLUMN)
				SYNX_CONSOLE_LOG(*cur, *end, "\t\t%s",
				synx_debugfs_util_get_state_name
				(__fence_state(fence_entry, false)));
			if (synx_columns & ID_COLUMN) {
				SYNX_CONSOLE_LOG(*cur, *end, "\t\t%d", curr->g_handle);
				SYNX_CONSOLE_LOG(*cur, *end, "\t%d", curr->l_handle);
			}
			if (synx_columns & REF_CNT_COLUMN)
				SYNX_CONSOLE_LOG(*cur, *end, "\t\t%d",
				kref_read(&(fence_entry)->refcount));
			SYNX_CONSOLE_LOG(*cur, *end,
				"\n\t-------------------------------------");
			SYNX_CONSOLE_LOG(*cur, *end,
				"-----------------------------------------");
			SYNX_CONSOLE_LOG(*cur, *end, "-----------\n");
		}
		dma_fence_put(fence_entry);
	}
	spin_unlock_bh(&synx_dev->native->fence_map_lock);
}

void synx_debugfs_util_print_csl_fence(char **cur,
	char **end)
{
	u32 itr, rc = SYNX_SUCCESS;
	struct synx_entry_64 *curr = NULL;
	struct hlist_node *tmp;
	struct synx_map_entry *map_entry = NULL;

	SYNX_CONSOLE_LOG(*cur, *end, "\n\t------------- CSL FENCE MAP TABLE------------\n");

	if (synx_columns & FENCE_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "|           CSL FENCE        |");
	if (synx_columns & STATUS_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "|   STATUS   |");
	if (synx_columns & ID_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "|    HANDLE   |");
	if (synx_columns & REF_CNT_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "|    REF CNT   |");
	SYNX_CONSOLE_LOG(*cur, *end, "\n");

	spin_lock_bh(&synx_dev->native->csl_map_lock);
	hash_for_each_safe(synx_dev->native->csl_fence_map, itr, tmp, curr, node) {
		rc = synx_debugfs_util_get_CSL_fence_entry(curr);
		if (rc) {
			spin_unlock_bh(&synx_dev->native->csl_map_lock);
			return;
		}
		if (curr->data[0] >= lower_handle_id && curr->data[0] <= upper_handle_id) {
			if (synx_columns & FENCE_COLUMN)
				SYNX_CONSOLE_LOG(*cur, *end, "%p", curr->key);
			if (synx_columns & STATUS_COLUMN) {
				map_entry = synx_util_get_map_entry(curr->data[0]);
				if (!IS_ERR_OR_NULL(map_entry) &&
					!IS_ERR_OR_NULL(map_entry->synx_obj)) {
					SYNX_CONSOLE_LOG(*cur, *end, "\t%s",
					synx_debugfs_util_get_state_name
					(synx_util_get_object_status(map_entry->synx_obj)));
					synx_util_release_map_entry(map_entry);
				}
			} //TODO : Update status field of CSL Fence with updated structure
			if (synx_columns & ID_COLUMN)
				SYNX_CONSOLE_LOG(*cur, *end, "\t\t%d", curr->data[0]);
			if (synx_columns & REF_CNT_COLUMN)
				SYNX_CONSOLE_LOG(*cur, *end, "\t%d", kref_read(&curr->refcount));
			SYNX_CONSOLE_LOG(*cur, *end, "\n\t-------------------------------------");
			SYNX_CONSOLE_LOG(*cur, *end, "-----------------------------------------");
			SYNX_CONSOLE_LOG(*cur, *end, "-----------\n");
		}
		synx_debugfs_util_put_CSL_fence_entry(curr);
	}
	spin_unlock_bh(&synx_dev->native->csl_map_lock);
}

void synx_debugfs_util_print_global_shared_memory(char **cur,
	char **end)
{
	struct synx_global_coredata synx_global_entry;
	u32 i, idx;

	/* Column heading set up*/
	SYNX_CONSOLE_LOG(*cur, *end,
				"\n\t  ------------- GLOBAL SHARED MEMORY ------------\n\t");

	if (synx_columns & STATUS_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "|  STATUS  |");
	if (synx_columns & ID_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "|  HANDLE  |");
	if (synx_columns & REF_CNT_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "|  REF CNT |");
	if (synx_columns & NUM_CHILD_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "| NUM CHILD |");
	if (synx_columns & SUBSCRIBERS_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "| SUBSCRIBERS |");
	if (synx_columns & WAITERS_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "| WAITERS |");
	if (synx_columns & PARENTS_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "|    PARENTS    |");
	SYNX_CONSOLE_LOG(*cur, *end, "\n");

	for (idx = lower_handle_id ; idx <= upper_handle_id ; idx++) {
		if (!synx_fetch_global_shared_memory_handle_details(idx, &synx_global_entry) ||
		!synx_debugfs_util_is_valid_global_shared_memory_entry(&synx_global_entry, idx))
			continue;
		if (synx_columns & STATUS_COLUMN)
			SYNX_CONSOLE_LOG(*cur, *end, "\t   %s",
			synx_debugfs_util_get_state_name(synx_global_entry.status));
		if (synx_columns & ID_COLUMN)
			SYNX_CONSOLE_LOG(*cur, *end, "\t\t%x", synx_global_entry.handle);
		if (synx_columns & REF_CNT_COLUMN)
			SYNX_CONSOLE_LOG(*cur, *end, "\t\t%d", synx_global_entry.refcount);
		if (synx_columns & NUM_CHILD_COLUMN)
			SYNX_CONSOLE_LOG(*cur, *end, "\t\t%d", synx_global_entry.num_child);
		if (synx_columns & SUBSCRIBERS_COLUMN)
			SYNX_CONSOLE_LOG(*cur, *end, "\t%d", synx_global_entry.subscribers);
		if (synx_columns & WAITERS_COLUMN)
			SYNX_CONSOLE_LOG(*cur, *end, "\t\t%d", synx_global_entry.waiters);
		if (synx_columns & PARENTS_COLUMN) {
			for (i = 0; i < SYNX_GLOBAL_MAX_PARENTS; i++) {
				if (synx_global_entry.parents[i])
					SYNX_CONSOLE_LOG(*cur, *end, "   %2u",
					synx_global_entry.parents[i]);
			}
		}
		SYNX_CONSOLE_LOG(*cur, *end, "\n\t-------------------------------------");
		SYNX_CONSOLE_LOG(*cur, *end, "-----------------------------------------");
		SYNX_CONSOLE_LOG(*cur, *end, "-----------\n");
	}
}

void synx_debugfs_util_print_client_table(char **cur,
	char **end)
{
	u32 rc = SYNX_SUCCESS;
	struct synx_client *curr;
	struct hlist_node *tmp;
	struct hlist_node *tmp2;
	struct synx_handle_coredata *curr2 = NULL;
	u32 client_map_itr, handle_map_itr;

	SYNX_CONSOLE_LOG(*cur, *end, "\n\t ------------- CLIENT MAP TABLE------------\n");
	if (synx_columns & CLIENT_ID_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "| CLIENT ID |");
	if (synx_columns & REF_CNT_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "|CLIENT REF COUNT|");
	if (synx_columns & ID_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "| HANDLE ID |");
	if (synx_columns & REF_CNT_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "|REF COUNT|");
	if (synx_columns & REL_CNT_COLUMN)
		SYNX_CONSOLE_LOG(*cur, *end, "|REL COUNT|");
	SYNX_CONSOLE_LOG(*cur, *end, "\n");
	spin_lock_bh(&synx_dev->native->metadata_map_lock);
	hash_for_each_safe(synx_dev->native->client_metadata_map,
		client_map_itr, tmp, curr, node) {
		rc = synx_debugfs_util_get_client_data(curr);
		if (rc)
			goto bail;
		spin_lock_bh(&curr->handle_map_lock);
		hash_for_each_safe(curr->handle_map,
			handle_map_itr, tmp2, curr2, node) {
			rc = synx_debugfs_util_get_handle(curr2);
			if (rc) {
				spin_unlock_bh(&curr->handle_map_lock);
				synx_debugfs_util_put_client_data(curr);
				goto bail;
			}
			if (curr2->key >= lower_handle_id && curr2->key <= upper_handle_id) {
				if (synx_columns & CLIENT_ID_COLUMN)
					SYNX_CONSOLE_LOG(*cur, *end, "\t%u", curr->id);
				if (synx_columns & REF_CNT_COLUMN)
					SYNX_CONSOLE_LOG(*cur, *end, "\t\t%d",
					kref_read(&curr->refcount));
				if (synx_columns & ID_COLUMN)
					SYNX_CONSOLE_LOG(*cur, *end, "\t%d", curr2->key);
				if (synx_columns & REF_CNT_COLUMN)
					SYNX_CONSOLE_LOG(*cur, *end, "\t\t%d",
					kref_read(&curr2->refcount));
				if (synx_columns & REL_CNT_COLUMN)
					SYNX_CONSOLE_LOG(*cur, *end, "\t\t%d", curr2->rel_count);
				SYNX_CONSOLE_LOG(*cur, *end,
					"\n\t-------------------------------------");
				SYNX_CONSOLE_LOG(*cur, *end,
					"-----------------------------------------");
				SYNX_CONSOLE_LOG(*cur, *end, "-----------\n");
			}
			synx_debugfs_util_put_handle(curr2);
		}
		spin_unlock_bh(&curr->handle_map_lock);
		synx_debugfs_util_put_client_data(curr);
	}
bail:
	spin_unlock_bh(&synx_dev->native->metadata_map_lock);
}

void synx_debugfs_util_load_help_content(char **cur,
	char **end)
{
	SYNX_CONSOLE_LOG(*cur, *end,
		"\n\n\tSynx tables Supported for debugfs with the column names:");
	SYNX_CONSOLE_LOG(*cur, *end,
		"\n\n\tGLOBAL/LOCAL MAP COLUMNS : STATUS, ID, REF_CNT, BOUND,");
	SYNX_CONSOLE_LOG(*cur, *end, "\tGLOBAL INDEX, MAP CNT\n");
	SYNX_CONSOLE_LOG(*cur, *end,
		"\n\tGLOBAL SHARED MEMORY COLUMNS : STATUS, ID,");
	SYNX_CONSOLE_LOG(*cur, *end,
		"REF_CNT, NUM_CHILD, \tSUBSCRIBERS, WAITERS, PARENTS");
	SYNX_CONSOLE_LOG(*cur, *end,
		"\n\n\tCLIENT MAP COLUMNS : CLIENT_ID, REF_CNT, REL_CNT, ID");
	SYNX_CONSOLE_LOG(*cur, *end,
		"\n\n\tDMA FENCE COLUMNS: STATUS, ID, REF_CNT, DMA FENCE");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\n\tINSTRUCTIONS TO BE FOLLOWED:");
	SYNX_CONSOLE_LOG(*cur, *end,
		"\n\n\tTO PRINT CHOOSE THE COLUMNS ACCORDINGLY AND ADD UP THE");
	SYNX_CONSOLE_LOG(*cur, *end,
		"\tHEXADECIMAL VALUES & PASS THE ADDED UP VALUES FOR COLUMN ALONG");
	SYNX_CONSOLE_LOG(*cur, *end, "WITH TABLE SELECTION VALUE AS SHOWN BELOW:");
	SYNX_CONSOLE_LOG(*cur, *end,
		"\n\tSet Below Values for Column selection\n");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\n\tNAME_COLUMN       = 0x0001");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tID_COLUMN         = 0x0002");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tBOUND_COLUMN      = 0x0004");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tSTATUS_COLUMN     = 0x0008");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tFENCE_COLUMN      = 0x0010");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tCOREDATA_COLUMN   = 0x0020");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tGLOBAL_IDX_COLUMN = 0x0040");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tREL_CNT_COLUMN    = 0x0080");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tMAP_CNT_COLUMN    = 0x0100");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tREF_CNT_COLUMN    = 0x0200");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tNUM_CHILD_COLUMN  = 0x0400");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tSUBSCRIBERS_COLUMN= 0x0800");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tWAITERS_COLUMN    = 0x1000");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tPARENTS_COLUMN    = 0x2000");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tCLIENT_ID_COLUMN  = 0x4000");
	SYNX_CONSOLE_LOG(*cur, *end,
		"\n\n\tSet Below Values for Table selection\n");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tLOCAL_HASHTABLE   = 0x00010000");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tGLOBAL_HASHTABLE  = 0x00020000");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tCLIENT_HASHTABLE  = 0x00040000");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tGLOBAL_SHARED_MEM = 0x00080000");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tDMA_FENCE_MAP     = 0x00100000\n");
	SYNX_CONSOLE_LOG(*cur, *end,
		"\n\tExample : To select Global map & all its columns :");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\t echo 0x2034E>column_level");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\t Last four digits in hexadecimal flag");
	SYNX_CONSOLE_LOG(*cur, *end, " is dedicated for setting columns,");
	SYNX_CONSOLE_LOG(*cur, *end,
		"\tuser can even set \"FFFF\" to set all columns");
	SYNX_CONSOLE_LOG(*cur, *end,
		"\n\t Instead of passing 0x2034E, \tuser can even pass");
	SYNX_CONSOLE_LOG(*cur, *end, " 0x2FFFF to fetch the same");
	SYNX_CONSOLE_LOG(*cur, *end,
		"\n\n\tUser can set Handle Range with echo command as shown below\n");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\techo 1048577-1048580>synx_table");
	SYNX_CONSOLE_LOG(*cur, *end,
		"\n\tFor single handle : echo \"1048577\">synx_table");
	SYNX_CONSOLE_LOG(*cur, *end,
		"\n\tHandle range can be set in hexadecimal values as shown below:");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\techo 0x100001-10000f>synx_table");
	SYNX_CONSOLE_LOG(*cur, *end,
		"\n\tSingle handle : echo 0x100001>synx_table");
	SYNX_CONSOLE_LOG(*cur, *end,
		"\n\n\tTo print info on console : cat synx_table");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\n\tHandle states :");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tACT : SYNX_STATE_ACTIVE");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tINV : SYNX_STATE_INVALID");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tERR : SYNX_STATE_SIGNALED_ERROR");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tSUC : SYNX_STATE_SIGNALED_SUCCESS");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tCAN : SYNX_STATE_SIGNALED_CANCELLED");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tEXT : SYNX_STATE_SIGNALED_EXTERNAL");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tSSR : SYNX_STATE_SIGNALED_SSR\n");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tCUS : CUSTOM SIGNAL");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\t??? : UNKNOWN / UNDEFINED");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\n\tAdditional information:");
	SYNX_CONSOLE_LOG(*cur, *end,
		"\n\tNo need to set handle ID range and column or table selection");
	SYNX_CONSOLE_LOG(*cur, *end, "\tvalues again if once it is already set");
	SYNX_CONSOLE_LOG(*cur, *end,
		"\n\tSimply using cat synx_table command user can print the data");
	SYNX_CONSOLE_LOG(*cur, *end, "\tfor same table with same set of columns");
	SYNX_CONSOLE_LOG(*cur, *end, "\n\tTo print all tables and all");
	SYNX_CONSOLE_LOG(*cur, *end, "columns set column level value to 0x1fffff");
	SYNX_CONSOLE_LOG(*cur, *end,
		"\n\tCurrently we do not support CSL fence\n\n");
}
