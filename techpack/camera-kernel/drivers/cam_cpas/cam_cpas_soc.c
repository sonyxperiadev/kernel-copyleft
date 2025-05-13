// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/device.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <dt-bindings/msm-camera.h>

#include "cam_cpas_api.h"
#include "cam_cpas_hw_intf.h"
#include "cam_cpas_hw.h"
#include "cam_cpas_soc.h"
#include "cam_compat.h"

static uint cpas_dump;
module_param(cpas_dump, uint, 0644);

#define CAM_ICP_CLK_NAME "cam_icp_clk"

void cam_cpas_dump_tree_vote_info(struct cam_hw_info *cpas_hw,
	const struct cam_cpas_tree_node *tree_node,
	const char *identifier, int ddr_drv_idx, int cesta_drv_idx)
{
	struct cam_cpas_private_soc *soc_private =
		(struct cam_cpas_private_soc *) cpas_hw->soc_info.soc_private;

	if ((cpas_dump & BIT(0)) == 0)
		return;

	if (cesta_drv_idx > CAM_CPAS_PORT_HLOS_DRV)
		CAM_INFO(CAM_PERF,
			"%s node:%s lvl:%d cesta_drv_idx:%d DRV BW camnoc[%llu %llu]",
			identifier, tree_node->node_name, tree_node->level_idx, cesta_drv_idx,
			tree_node->bw_info[cesta_drv_idx].drv_vote.high.camnoc,
			tree_node->bw_info[cesta_drv_idx].drv_vote.low.camnoc);
	else
		CAM_INFO(CAM_PERF,
			"%s node:%s lvl:%d cesta_drv_idx:%d HLOS BW camnoc[%llu]",
			identifier, tree_node->node_name, tree_node->level_idx, cesta_drv_idx,
			tree_node->bw_info[cesta_drv_idx].hlos_vote.camnoc);

	if (ddr_drv_idx > CAM_CPAS_PORT_HLOS_DRV)
		CAM_INFO(CAM_PERF,
			"%s node:%s lvl:%d ddr_drv_idx:%d DRV BW ab[%llu %llu] ib[%llu %llu]",
			identifier, tree_node->node_name, tree_node->level_idx, ddr_drv_idx,
			tree_node->bw_info[ddr_drv_idx].drv_vote.high.ab,
			tree_node->bw_info[ddr_drv_idx].drv_vote.low.ab,
			tree_node->bw_info[ddr_drv_idx].drv_vote.high.ib,
			tree_node->bw_info[ddr_drv_idx].drv_vote.low.ib);
	else
		CAM_INFO(CAM_PERF,
			"%s node:%s lvl:%d ddr_drv_idx:%d HLOS BW ab[%llu] ib[%llu]",
			identifier, tree_node->node_name, tree_node->level_idx, ddr_drv_idx,
			tree_node->bw_info[ddr_drv_idx].hlos_vote.ab,
			tree_node->bw_info[ddr_drv_idx].hlos_vote.ib);

	if (soc_private->enable_cam_ddr_drv) {
		int i;

		CAM_INFO(CAM_PERF,
			"%s node:%s lvl:%d drv_idx:%d cesta_drv_idx:%d ==== printing full node state",
			identifier, tree_node->node_name, tree_node->level_idx,
			ddr_drv_idx, cesta_drv_idx);

		for (i = 0; i < CAM_CPAS_MAX_DRV_PORTS; i++) {

			if (i == CAM_CPAS_PORT_HLOS_DRV)
				CAM_INFO(CAM_PERF,
					"idx[%d] HLOS camnoc[%llu], DDR ab[%llu] ib[%llu]",
					i,
					tree_node->bw_info[i].hlos_vote.camnoc,
					tree_node->bw_info[i].hlos_vote.ab,
					tree_node->bw_info[i].hlos_vote.ib);
			else
				CAM_INFO(CAM_PERF,
					"idx[%d] DRV camnoc[%llu %llu], DDR ab[%llu %llu] ib[%llu %llu]",
					i,
					tree_node->bw_info[i].drv_vote.high.camnoc,
					tree_node->bw_info[i].drv_vote.low.camnoc,
					tree_node->bw_info[i].drv_vote.high.ab,
					tree_node->bw_info[i].drv_vote.low.ab,
					tree_node->bw_info[i].drv_vote.high.ib,
					tree_node->bw_info[i].drv_vote.low.ib);
		}
	}

}

void cam_cpas_dump_full_tree_state(struct cam_hw_info *cpas_hw, const char *identifier)
{
	int j;
	struct cam_cpas_private_soc *soc_private =
		(struct cam_cpas_private_soc *) cpas_hw->soc_info.soc_private;
	struct cam_cpas_tree_node *curr_node;

	if ((cpas_dump & BIT(1)) == 0)
		return;

	CAM_INFO(CAM_CPAS, "Dumping cpas tree full state start ============== %s", identifier);

	/* This will traverse through all nodes in the tree and print stats*/
	for (j = 0; j < CAM_CPAS_MAX_TREE_NODES; j++) {
		if (!soc_private->tree_node[j])
			continue;

		curr_node = soc_private->tree_node[j];

		if (soc_private->enable_cam_ddr_drv) {
			int i;

			CAM_INFO(CAM_PERF,
				"Identifier[%s] node:[%s] cell:%d lvl:%d PortIdx mnoc[%d %d %d %d] camnoc[%d] camnoc_max %d, bus_width:%d, drv_idx:%d",
				identifier, curr_node->node_name, curr_node->cell_idx,
				curr_node->level_idx,
				curr_node->axi_port_idx_arr[CAM_CPAS_PORT_HLOS_DRV],
				curr_node->axi_port_idx_arr[CAM_CPAS_PORT_DRV_0],
				curr_node->axi_port_idx_arr[CAM_CPAS_PORT_DRV_1],
				curr_node->axi_port_idx_arr[CAM_CPAS_PORT_DRV_2],
				curr_node->camnoc_axi_port_idx,
				curr_node->camnoc_max_needed,
				curr_node->bus_width_factor,
				curr_node->drv_voting_idx);

			for (i = 0; i < CAM_CPAS_MAX_DRV_PORTS; i++) {
				if (i == CAM_CPAS_PORT_HLOS_DRV)
					CAM_INFO(CAM_PERF,
						"    idx[%d] HLOS camnoc[%llu], DDR ab[%llu] ib[%llu]",
						i,
						curr_node->bw_info[i].hlos_vote.camnoc,
						curr_node->bw_info[i].hlos_vote.ab,
						curr_node->bw_info[i].hlos_vote.ib);
				else
					CAM_INFO(CAM_PERF,
						"    idx[%d] DRV camnoc[%llu %llu], DDR ab[%llu %llu] ib[%llu %llu]",
						i,
						curr_node->bw_info[i].drv_vote.high.camnoc,
						curr_node->bw_info[i].drv_vote.low.camnoc,
						curr_node->bw_info[i].drv_vote.high.ab,
						curr_node->bw_info[i].drv_vote.low.ab,
						curr_node->bw_info[i].drv_vote.high.ib,
						curr_node->bw_info[i].drv_vote.low.ib);
			}
		} else {
			CAM_INFO(CAM_CPAS,
				"[%s] Cell[%d] level[%d] PortIdx[%d][%d] camnoc_bw[%d %d %lld %lld] mnoc_bw[%lld %lld]",
				curr_node->node_name, curr_node->cell_idx,
				curr_node->level_idx,
				curr_node->axi_port_idx_arr[CAM_CPAS_PORT_HLOS_DRV],
				curr_node->camnoc_axi_port_idx,
				curr_node->camnoc_max_needed,
				curr_node->bus_width_factor,
				curr_node->bw_info[CAM_CPAS_PORT_HLOS_DRV].hlos_vote.camnoc,
				curr_node->bw_info[CAM_CPAS_PORT_HLOS_DRV].hlos_vote.camnoc *
				curr_node->bus_width_factor,
				curr_node->bw_info[CAM_CPAS_PORT_HLOS_DRV].hlos_vote.ab,
				curr_node->bw_info[CAM_CPAS_PORT_HLOS_DRV].hlos_vote.ib);
		}
	}

	CAM_INFO(CAM_CPAS, "Dumping cpas tree full state end ============== %s", identifier);
}

void cam_cpas_dump_axi_vote_info(
	const struct cam_cpas_client *cpas_client,
	const char *identifier,
	struct cam_axi_vote *axi_vote)
{
	int i;

	if ((cpas_dump & BIT(0)) == 0)
		return;

	if (!axi_vote || (axi_vote->num_paths >
		CAM_CPAS_MAX_PATHS_PER_CLIENT)) {
		CAM_ERR(CAM_PERF, "Invalid num_paths %d",
			axi_vote ? axi_vote->num_paths : -1);
		return;
	}

	for (i = 0; i < axi_vote->num_paths; i++) {
		CAM_INFO(CAM_PERF,
		"Client [%s][%d] : [%s], Path=[%d] [%d], [%s], camnoc[%llu], mnoc_ab[%llu], mnoc_ib[%llu]",
		cpas_client->data.identifier, cpas_client->data.cell_index,
		identifier,
		axi_vote->axi_path[i].path_data_type,
		axi_vote->axi_path[i].transac_type,
		cam_cpas_axi_util_drv_vote_lvl_to_string(axi_vote->axi_path[i].vote_level),
		axi_vote->axi_path[i].camnoc_bw,
		axi_vote->axi_path[i].mnoc_ab_bw,
		axi_vote->axi_path[i].mnoc_ib_bw);
	}

}

void cam_cpas_util_debug_parse_data(
	struct cam_cpas_private_soc *soc_private)
{
	int i, j;
	struct cam_cpas_tree_node *curr_node = NULL;

	if ((cpas_dump & BIT(0)) == 0)
		return;

	for (i = 0; i < CAM_CPAS_MAX_TREE_NODES; i++) {
		if (!soc_private->tree_node[i])
			break;

		curr_node = soc_private->tree_node[i];
		CAM_INFO(CAM_CPAS,
			"NODE cell_idx: %d, level: %d, name: %s, axi_port_idx: %d, merge_type: %d, parent_name: %s camnoc_max_needed: %d",
			curr_node->cell_idx, curr_node->level_idx,
			curr_node->node_name, curr_node->axi_port_idx_arr[CAM_CPAS_PORT_HLOS_DRV],
			curr_node->merge_type, curr_node->parent_node ?
			curr_node->parent_node->node_name : "no parent",
			curr_node->camnoc_max_needed);

		if (curr_node->level_idx)
			continue;

		CAM_INFO(CAM_CPAS, "path_type: %d, transac_type: %s drv_voting_idx:%d",
			curr_node->path_data_type,
			cam_cpas_axi_util_trans_type_to_string(
			curr_node->path_trans_type), curr_node->drv_voting_idx);

		for (j = 0; j < CAM_CPAS_PATH_DATA_MAX; j++) {
			if (curr_node->constituent_paths[j])
				CAM_INFO(CAM_CPAS, "Constituent path: %d", j);
		}
	}

	CAM_INFO(CAM_CPAS, "NUMBER OF NODES PARSED: %d", i);
}

int cam_cpas_node_tree_cleanup(struct cam_cpas *cpas_core,
	struct cam_cpas_private_soc *soc_private)
{
	int i = 0;

	for (i = 0; i < CAM_CPAS_MAX_TREE_NODES; i++) {
		if (soc_private->tree_node[i]) {
			kfree(soc_private->tree_node[i]->bw_info);
			kfree(soc_private->tree_node[i]->axi_port_idx_arr);
			soc_private->tree_node[i]->bw_info = NULL;
			soc_private->tree_node[i]->axi_port_idx_arr = NULL;
			of_node_put(soc_private->tree_node[i]->tree_dev_node);
			kfree(soc_private->tree_node[i]);
			soc_private->tree_node[i] = NULL;
		}
	}

	for (i = 0; i < CAM_CPAS_MAX_TREE_LEVELS; i++) {
		if (soc_private->level_node[i]) {
			of_node_put(soc_private->level_node[i]);
			soc_private->level_node[i] = NULL;
		}
	}

	if (soc_private->camera_bus_node) {
		of_node_put(soc_private->camera_bus_node);
		soc_private->camera_bus_node = NULL;
	}

	mutex_destroy(&cpas_core->tree_lock);

	return 0;
}

static int cam_cpas_util_path_type_to_idx(uint32_t *path_data_type)
{
	if (*path_data_type >= CAM_CPAS_PATH_DATA_CONSO_OFFSET) {
		*path_data_type = CAM_CPAS_MAX_GRAN_PATHS_PER_CLIENT +
			(*path_data_type % CAM_CPAS_MAX_GRAN_PATHS_PER_CLIENT);
	}
	else {
		*path_data_type %= CAM_CPAS_MAX_GRAN_PATHS_PER_CLIENT;
	}

	if (*path_data_type >= CAM_CPAS_PATH_DATA_MAX) {
		CAM_ERR(CAM_CPAS, "index Invalid: %u", *path_data_type);
		return -EINVAL;
	}

	return 0;
}

static int cam_cpas_update_camnoc_node(struct cam_cpas *cpas_core,
	struct device_node *curr_node,
	struct cam_cpas_tree_node *cpas_node_ptr,
	int *camnoc_idx)

{
	struct device_node *camnoc_node;
	int rc;

	camnoc_node = of_find_node_by_name(curr_node,
			"qcom,axi-port-camnoc");
	if (camnoc_node) {

		if (*camnoc_idx >=
			CAM_CPAS_MAX_AXI_PORTS) {
			CAM_ERR(CAM_CPAS, "CAMNOC axi index overshoot %d",
				*camnoc_idx);
			return -EINVAL;
		}

		cpas_core->camnoc_axi_port[*camnoc_idx]
			.axi_port_node = camnoc_node;
		rc = of_property_read_string(
			curr_node,
			"qcom,axi-port-name",
			&cpas_core->camnoc_axi_port[*camnoc_idx]
			.axi_port_name);

		if (rc) {
			CAM_ERR(CAM_CPAS,
				"fail to read camnoc-port-name rc=%d",
				rc);
			return rc;
		}
		cpas_node_ptr->camnoc_axi_port_idx = *camnoc_idx;
		cpas_core->num_camnoc_axi_ports++;
		(*camnoc_idx)++;
	}
	return 0;
}

static int cam_cpas_parse_mnoc_node(struct cam_cpas *cpas_core,
	struct cam_cpas_private_soc *soc_private, struct cam_cpas_tree_node *curr_node_ptr,
	struct device_node *mnoc_node, int *mnoc_idx)
{
	int rc = 0, count, i;
	bool ib_voting_needed = false, is_rt_port = false;
	struct of_phandle_args src_args = {0}, dst_args = {0};

	ib_voting_needed = of_property_read_bool(curr_node_ptr->tree_dev_node,
		"ib-bw-voting-needed");
	is_rt_port = of_property_read_bool(curr_node_ptr->tree_dev_node, "rt-axi-port");

	if (soc_private->bus_icc_based) {
		count = of_property_count_strings(mnoc_node, "interconnect-names");
		if (count <= 0) {
			CAM_ERR(CAM_CPAS, "no interconnect-names found");
			return -EINVAL;
		} else if (count > CAM_CPAS_MAX_DRV_PORTS) {
			CAM_ERR(CAM_CPAS, "Number of interconnects %d greater than max ports %d",
				count, CAM_CPAS_MAX_DRV_PORTS);
			return -EINVAL;
		}

		for (i = 0; i < count; i++) {
			if ((i > 0) && !soc_private->enable_cam_ddr_drv)
				break;

			if (*mnoc_idx >= CAM_CPAS_MAX_AXI_PORTS) {
				CAM_ERR(CAM_CPAS, "Invalid mnoc index: %d", *mnoc_idx);
				return -EINVAL;
			}

			cpas_core->axi_port[*mnoc_idx].axi_port_node = mnoc_node;
			rc = of_property_read_string_index(mnoc_node, "interconnect-names", i,
				&cpas_core->axi_port[*mnoc_idx].bus_client.common_data.name);
			if (rc) {
				CAM_ERR(CAM_CPAS, "failed to read interconnect-names rc=%d", rc);
				return rc;
			}

			rc = of_parse_phandle_with_args(mnoc_node, "interconnects",
				"#interconnect-cells", (2 * i), &src_args);
			if (rc) {
				CAM_ERR(CAM_CPAS,
					"failed to read axi bus src info rc=%d",
					rc);
				return -EINVAL;
			}

			of_node_put(src_args.np);
			if (src_args.args_count != 1) {
				CAM_ERR(CAM_CPAS, "Invalid number of axi src args: %d",
					src_args.args_count);
				return -EINVAL;
			}

			cpas_core->axi_port[*mnoc_idx].bus_client.common_data.src_id =
				src_args.args[0];

			rc = of_parse_phandle_with_args(mnoc_node, "interconnects",
				"#interconnect-cells", ((2 * i) + 1), &dst_args);
			if (rc) {
				CAM_ERR(CAM_CPAS, "failed to read axi bus dst info rc=%d", rc);
				return -EINVAL;
			}

			of_node_put(dst_args.np);
			if (dst_args.args_count != 1) {
				CAM_ERR(CAM_CPAS, "Invalid number of axi dst args: %d",
					dst_args.args_count);
				return -EINVAL;
			}

			cpas_core->axi_port[*mnoc_idx].bus_client.common_data.dst_id =
				dst_args.args[0];
			cpas_core->axi_port[*mnoc_idx].bus_client.common_data.num_usecases = 2;
			cpas_core->axi_port[*mnoc_idx].axi_port_name =
				cpas_core->axi_port[*mnoc_idx].bus_client.common_data.name;
			cpas_core->axi_port[*mnoc_idx].drv_idx = i;

			if (i > CAM_CPAS_PORT_HLOS_DRV) {
				cpas_core->axi_port[*mnoc_idx].bus_client.common_data.is_drv_port =
					true;
				cpas_core->axi_port[*mnoc_idx].curr_bw.vote_type =
					CAM_CPAS_VOTE_TYPE_DRV;
				cpas_core->axi_port[*mnoc_idx].applied_bw.vote_type =
					CAM_CPAS_VOTE_TYPE_DRV;
				cpas_core->axi_port[*mnoc_idx].cam_rsc_dev =
					cam_cpas_get_rsc_dev_for_drv(i - CAM_CPAS_PORT_DRV_0);
				if (!cpas_core->axi_port[*mnoc_idx].cam_rsc_dev) {
					CAM_ERR(CAM_CPAS,
						"Port[%s][%d] Failed to get rsc device drv_idx:%d",
						cpas_core->axi_port[*mnoc_idx].axi_port_name,
						*mnoc_idx, i);
					rc = -ENODEV;
					goto err;
				}
			}

			/*
			 * The indexes of axi_port_idx_arr map to drv_voting_idx,
			 * with 0 pointing to hlos drv bus ID
			 */
			curr_node_ptr->axi_port_idx_arr[i] = *mnoc_idx;
			cpas_core->axi_port[*mnoc_idx].ib_bw_voting_needed = ib_voting_needed;
			cpas_core->axi_port[*mnoc_idx].is_rt = is_rt_port;
			CAM_DBG(CAM_PERF, "Adding Bus Client=[%s] : src=%d, dst=%d mnoc_idx:%d",
				cpas_core->axi_port[*mnoc_idx].bus_client.common_data.name,
				cpas_core->axi_port[*mnoc_idx].bus_client.common_data.src_id,
				cpas_core->axi_port[*mnoc_idx].bus_client.common_data.dst_id,
				*mnoc_idx);
			(*mnoc_idx)++;
			cpas_core->num_axi_ports++;
		}
	} else {
		if (soc_private->enable_cam_ddr_drv) {
			CAM_ERR(CAM_CPAS, "DRV not supported for old bus scaling clients");
			return -EPERM;
		}

		cpas_core->axi_port[*mnoc_idx].axi_port_node = mnoc_node;
		rc =  of_property_read_string(curr_node_ptr->tree_dev_node, "qcom,axi-port-name",
			&cpas_core->axi_port[*mnoc_idx].bus_client.common_data.name);
		if (rc) {
			CAM_ERR(CAM_CPAS,
				"failed to read mnoc-port-name rc=%d",
				rc);
			return rc;
		}

		cpas_core->axi_port[*mnoc_idx].axi_port_name =
			cpas_core->axi_port[*mnoc_idx].bus_client.common_data.name;
		curr_node_ptr->axi_port_idx_arr[0] = *mnoc_idx;
		cpas_core->axi_port[*mnoc_idx].ib_bw_voting_needed = ib_voting_needed;
		cpas_core->axi_port[*mnoc_idx].is_rt = is_rt_port;
		(*mnoc_idx)++;
		cpas_core->num_axi_ports++;
	}

err:
	return rc;
}

static int cam_cpas_parse_node_tree(struct cam_cpas *cpas_core,
	struct device_node *of_node, struct cam_cpas_private_soc *soc_private)
{
	struct device_node *camera_bus_node;
	struct device_node *level_node;
	struct device_node *curr_node;
	struct device_node *parent_node;
	struct device_node *mnoc_node;
	int mnoc_idx = 0, camnoc_idx = 0, level_idx = 0;
	uint32_t path_idx;
	bool camnoc_max_needed = false;
	struct cam_cpas_tree_node *curr_node_ptr = NULL;
	struct cam_cpas_client *curr_client = NULL;
	const char *client_name = NULL;
	uint32_t client_idx = 0, cell_idx = 0;
	uint8_t niu_idx = 0;
	int rc = 0, count = 0, i, j, num_drv_ports;

	camera_bus_node = of_get_child_by_name(of_node, "camera-bus-nodes");
	if (!camera_bus_node) {
		CAM_ERR(CAM_CPAS, "Camera Bus node not found in cpas DT node");
		return -EINVAL;
	}

	soc_private->camera_bus_node = camera_bus_node;

	for_each_available_child_of_node(camera_bus_node, level_node) {
		rc = of_property_read_u32(level_node, "level-index", &level_idx);
		if (rc) {
			CAM_ERR(CAM_CPAS, "Error reading level idx rc: %d", rc);
			return rc;
		}
		if (level_idx >= CAM_CPAS_MAX_TREE_LEVELS) {
			CAM_ERR(CAM_CPAS, "Invalid level idx: %d", level_idx);
			return -EINVAL;
		}

		soc_private->level_node[level_idx] = level_node;
	}

	if (soc_private->enable_smart_qos)
		soc_private->smart_qos_info->num_rt_wr_nius = 0;

	if (soc_private->enable_cam_ddr_drv)
		num_drv_ports = CAM_CPAS_MAX_DRV_PORTS;
	else
		num_drv_ports = 1;

	for (level_idx = (CAM_CPAS_MAX_TREE_LEVELS - 1); level_idx >= 0;
		level_idx--) {
		level_node = soc_private->level_node[level_idx];
		if (!level_node)
			continue;

		CAM_DBG(CAM_CPAS, "Parsing level %d nodes", level_idx);

		camnoc_max_needed = of_property_read_bool(level_node, "camnoc-max-needed");
		for_each_available_child_of_node(level_node, curr_node) {
			curr_node_ptr = kzalloc(sizeof(struct cam_cpas_tree_node), GFP_KERNEL);
			if (!curr_node_ptr)
				return -ENOMEM;

			curr_node_ptr->tree_dev_node = curr_node;
			rc = of_property_read_u32(curr_node, "cell-index",
				&curr_node_ptr->cell_idx);
			if (rc) {
				CAM_ERR(CAM_CPAS, "Node index not found");
				return rc;
			}

			CAM_DBG(CAM_CPAS, "Parsing Node with cell index %d",
					curr_node_ptr->cell_idx);

			if (curr_node_ptr->cell_idx >=
				CAM_CPAS_MAX_TREE_NODES) {
				CAM_ERR(CAM_CPAS, "Invalid cell idx: %d", curr_node_ptr->cell_idx);
				return -EINVAL;
			}

			soc_private->tree_node[curr_node_ptr->cell_idx] = curr_node_ptr;
			curr_node_ptr->level_idx = level_idx;

			rc = of_property_read_string(curr_node, "node-name",
				&curr_node_ptr->node_name);
			if (rc) {
				CAM_ERR(CAM_CPAS, "failed to read node-name rc=%d", rc);
				return rc;
			}

			curr_node_ptr->bw_info = kzalloc((sizeof(struct cam_cpas_axi_bw_info) *
				num_drv_ports), GFP_KERNEL);
			if (!curr_node_ptr->bw_info) {
				CAM_ERR(CAM_CPAS, "Failed in allocating memory for bw info");
				return -ENOMEM;
			}

			curr_node_ptr->axi_port_idx_arr = kzalloc((sizeof(int) * num_drv_ports),
				GFP_KERNEL);
			if (!curr_node_ptr->axi_port_idx_arr) {
				CAM_ERR(CAM_CPAS, "Failed in allocating memory for port indices");
				return -ENOMEM;
			}

			if (soc_private->enable_smart_qos && (level_idx == 1) &&
				of_property_read_bool(curr_node, "rt-wr-niu")) {

				rc = of_property_read_u32(curr_node, "priority-lut-low-offset",
					&curr_node_ptr->pri_lut_low_offset);
				if (rc) {
					CAM_ERR(CAM_CPAS, "Invalid priority low offset rc %d", rc);
					return rc;
				}

				rc = of_property_read_u32(curr_node, "priority-lut-high-offset",
					&curr_node_ptr->pri_lut_high_offset);
				if (rc) {
					CAM_ERR(CAM_CPAS, "Invalid priority high offset rc %d", rc);
					return rc;
				}

				rc = of_property_read_u32(curr_node, "niu-size",
					&curr_node_ptr->niu_size);
				if (rc || !curr_node_ptr->niu_size) {
					CAM_ERR(CAM_CPAS, "Invalid niu size rc %d", rc);
					return rc;
				}

				niu_idx = soc_private->smart_qos_info->num_rt_wr_nius;
				if (niu_idx >= CAM_CPAS_MAX_RT_WR_NIU_NODES) {
					CAM_ERR(CAM_CPAS, "Invalid number of level1 nodes %d",
						soc_private->smart_qos_info->num_rt_wr_nius);
					return -EINVAL;
				}

				soc_private->smart_qos_info->rt_wr_niu_node[niu_idx] =
					curr_node_ptr;
				soc_private->smart_qos_info->num_rt_wr_nius++;

				CAM_DBG(CAM_CPAS,
					"level1[%d] : Node %s idx %d priority offset 0x%x, NIU size %dKB",
					niu_idx, curr_node_ptr->node_name, curr_node_ptr->cell_idx,
					curr_node_ptr->pri_lut_low_offset, curr_node_ptr->niu_size);
			}

			curr_node_ptr->camnoc_max_needed = camnoc_max_needed;
			rc = of_property_read_u32(curr_node, "bus-width-factor",
				&curr_node_ptr->bus_width_factor);
			if (rc)
				curr_node_ptr->bus_width_factor = 1;

			rc = of_property_read_u32(curr_node, "traffic-merge-type",
				&curr_node_ptr->merge_type);
			if (rc)
				curr_node_ptr->merge_type = CAM_CPAS_TRAFFIC_MERGE_SUM;

			for (j = 0; j < num_drv_ports; j++)
				curr_node_ptr->axi_port_idx_arr[j] = -1;

			mnoc_node = of_get_child_by_name(curr_node, "qcom,axi-port-mnoc");
			if (mnoc_node) {
				rc = cam_cpas_parse_mnoc_node(cpas_core, soc_private, curr_node_ptr,
					mnoc_node, &mnoc_idx);
				if (rc) {
					CAM_ERR(CAM_CPAS, "failed to parse mnoc node info rc=%d",
						rc);
					return rc;
				}
			}

			if (!soc_private->control_camnoc_axi_clk) {
				rc = cam_cpas_update_camnoc_node(cpas_core, curr_node,
					curr_node_ptr, &camnoc_idx);
				if (rc) {
					CAM_ERR(CAM_CPAS, "failed to parse camnoc node info rc=%d",
						rc);
					return rc;
				}
			}

			rc = of_property_read_string(curr_node, "client-name", &client_name);
			if (!rc) {
				rc = of_property_read_u32(curr_node, "traffic-data",
					&curr_node_ptr->path_data_type);
				if (rc) {
					CAM_ERR(CAM_CPAS,
						"Path Data type not found");
					return rc;
				}

				rc = cam_cpas_util_path_type_to_idx(&curr_node_ptr->path_data_type);
				if (rc) {
					CAM_ERR(CAM_CPAS, "Incorrect path type for client: %s",
						client_name);
					return rc;
				}

				rc = of_property_read_u32(curr_node, "traffic-transaction-type",
					&curr_node_ptr->path_trans_type);
				if (rc) {
					CAM_ERR(CAM_CPAS, "Path Transac type not found");
					return rc;
				}

				if (curr_node_ptr->path_trans_type >= CAM_CPAS_TRANSACTION_MAX) {
					CAM_ERR(CAM_CPAS, "Invalid transac type: %d",
						curr_node_ptr->path_trans_type);
					return -EINVAL;
				}

				count = of_property_count_u32_elems(curr_node, "constituent-paths");
				for (i = 0; i < count; i++) {
					rc = of_property_read_u32_index(curr_node,
						"constituent-paths", i, &path_idx);
					if (rc) {
						CAM_ERR(CAM_CPAS, "No constituent path at %d", i);
						return rc;
					}

					rc = cam_cpas_util_path_type_to_idx(&path_idx);
					if (rc)
						return rc;

					curr_node_ptr->constituent_paths[path_idx] = true;
				}

				rc = cam_common_util_get_string_index(soc_private->client_name,
					soc_private->num_clients, client_name, &client_idx);
				if (rc) {
					CAM_ERR(CAM_CPAS, "client name not found in list: %s",
						client_name);
					return rc;
				}

				if (client_idx >= CAM_CPAS_MAX_CLIENTS)
					return -EINVAL;

				curr_client = cpas_core->cpas_client[client_idx];
				curr_client->tree_node_valid = true;
				curr_client->tree_node[curr_node_ptr->path_data_type]
					[curr_node_ptr->path_trans_type] = curr_node_ptr;

				if (soc_private->enable_cam_ddr_drv) {
					rc = of_property_read_u32(curr_node, "drv-voting-index",
						&curr_node_ptr->drv_voting_idx);
					if (rc)
						curr_node_ptr->merge_type = CAM_CPAS_PORT_HLOS_DRV;

					if (curr_node_ptr->drv_voting_idx == CAM_CPAS_PORT_DRV_DYN)
						curr_client->is_drv_dyn = true;

					if (curr_client->is_drv_dyn &&
						(curr_node_ptr->drv_voting_idx !=
						CAM_CPAS_PORT_DRV_DYN))
						CAM_ERR(CAM_CPAS,
							"Invalid config for drv dyn client: %s drv_idx: %d",
							client_name, curr_node_ptr->drv_voting_idx);
				}

				CAM_DBG(CAM_CPAS,
					"Node Added: Client[%s] DataType[%d] TransType[%d] DRV idx[%d]",
					client_name,
					curr_node_ptr->path_data_type,
					curr_node_ptr->path_trans_type,
					curr_node_ptr->drv_voting_idx);
			}

			if (soc_private->enable_cam_ddr_drv)
				for (j = CAM_CPAS_PORT_DRV_0; j < num_drv_ports; j++)
					curr_node_ptr->bw_info[j].vote_type =
						CAM_CPAS_VOTE_TYPE_DRV;

			parent_node = of_parse_phandle(curr_node, "parent-node", 0);
			if (parent_node) {
				of_property_read_u32(parent_node, "cell-index", &cell_idx);
				curr_node_ptr->parent_node = soc_private->tree_node[cell_idx];
			} else {
				CAM_DBG(CAM_CPAS, "no parent node at this level");
			}
		}
	}

	mutex_init(&cpas_core->tree_lock);
	cam_cpas_util_debug_parse_data(soc_private);

	return 0;
}

int cam_cpas_get_hw_features(struct platform_device *pdev,
	struct cam_cpas_private_soc *soc_private)
{
	struct device_node *of_node;
	void *fuse;
	uint32_t fuse_addr, fuse_mask, fuse_shift;
	uint32_t val = 0, fuse_val = 0, feature;
	uint32_t enable_type = 0, hw_map = 0;
	int count = 0, i = 0, j = 0,  num_feature = 0, num_fuse = 0;
	struct cam_cpas_feature_info *feature_info;

	of_node = pdev->dev.of_node;
	count = of_property_count_u32_elems(of_node, "cam_hw_fuse");

	CAM_DBG(CAM_CPAS, "fuse info elements count %d", count);

	if (count <= 0) {
		goto end;
	} else if (count%5 != 0) {
		CAM_INFO(CAM_CPAS, "fuse entries should be multiple of 5 %d",
			count);
		goto end;
	}

	for (i = 0; (i + 5) <= count; i = i + 5) {
		of_property_read_u32_index(of_node, "cam_hw_fuse", i,
				&feature);
		of_property_read_u32_index(of_node, "cam_hw_fuse", i + 1,
				&fuse_addr);
		of_property_read_u32_index(of_node, "cam_hw_fuse", i + 2,
				&fuse_mask);
		of_property_read_u32_index(of_node, "cam_hw_fuse", i + 3,
				&enable_type);
		of_property_read_u32_index(of_node, "cam_hw_fuse", i + 4,
				&hw_map);
		val = ffs(fuse_mask);
		if (val == 0) {
			CAM_ERR(CAM_CPAS, "fuse_mask not valid 0x%x",
				fuse_mask);
			fuse_shift = 0;
		} else {
			fuse_shift = val - 1;
		}
		CAM_INFO(CAM_CPAS,
			"feature 0x%x addr 0x%x, mask 0x%x, shift 0x%x type 0x%x hw_map 0x%x",
			feature, fuse_addr, fuse_mask, fuse_shift, enable_type,
			hw_map);

		fuse = ioremap(fuse_addr, 4);
		if (fuse) {
			fuse_val = cam_io_r(fuse);
			for (j = 0; (j < num_fuse) && (j < CAM_CPAS_FUSES_MAX);
				j++) {
				if (soc_private->fuse_info.fuse_val[j].fuse_id
					== fuse_addr)
					break;
			}
			if (j >= CAM_CPAS_FUSES_MAX) {
				CAM_ERR(CAM_CPAS,
					"fuse_info array overflow! %d", j);
				goto end;
			}
			if (j == num_fuse) {
				soc_private->fuse_info.fuse_val[j].fuse_id =
					fuse_addr;
				soc_private->fuse_info.fuse_val[j].fuse_val =
					fuse_val;
				CAM_INFO(CAM_CPAS,
					"fuse_addr 0x%x, fuse_val %x",
					fuse_addr, fuse_val);
				num_fuse++;
			}
		} else {
			/* if fuse ioremap is failed, disable the feature */
			CAM_ERR(CAM_CPAS,
				"fuse register io remap failed fuse_addr:0x%x feature0x%x ",
				fuse_addr, feature);

			if (enable_type == CAM_CPAS_FEATURE_TYPE_ENABLE ||
				enable_type == CAM_CPAS_FEATURE_TYPE_DISABLE)
				fuse_val = (enable_type) ? ~fuse_mask :
					fuse_mask;
			else
				fuse_val = 0;
		}

		if (num_feature >= CAM_CPAS_MAX_FUSE_FEATURE) {
			CAM_ERR(CAM_CPAS, "feature_info array overflow %d",
				num_feature);
			goto end;
		}

		soc_private->feature_info[num_feature].feature =
			feature;
		soc_private->feature_info[num_feature].hw_map = hw_map;
		soc_private->feature_info[num_feature].type = enable_type;
		feature_info = &soc_private->feature_info[num_feature];

		if (enable_type != CAM_CPAS_FEATURE_TYPE_VALUE) {
			if (enable_type == CAM_CPAS_FEATURE_TYPE_ENABLE) {
				/*
				 * fuse is for enable feature
				 * if fust bit is set means feature is enabled
				 * or HW is enabled
				 */
				if (fuse_val & fuse_mask)
					feature_info->enable = true;
				else
					feature_info->enable = false;
			} else if (enable_type ==
				CAM_CPAS_FEATURE_TYPE_DISABLE){
				/*
				 * fuse is for disable feature
				 * if fust bit is set means feature is disabled
				 * or HW is disabled
				 */
				if (fuse_val & fuse_mask)
					feature_info->enable = false;
				else
					feature_info->enable = true;
			} else {
				CAM_ERR(CAM_CPAS,
					"Feature type not valid, type: %d",
					enable_type);
				goto end;
			}
			CAM_INFO(CAM_CPAS,
				"feature 0x%x enable=%d hw_map=0x%x",
				feature_info->feature, feature_info->enable,
				feature_info->hw_map);
		} else {
			feature_info->value =
				(fuse_val & fuse_mask) >> fuse_shift;
			CAM_INFO(CAM_CPAS,
				"feature 0x%x value=0x%x hw_map=0x%x",
				feature_info->feature, feature_info->value,
				feature_info->hw_map);
		}
		num_feature++;
		iounmap(fuse);
	}

end:
	soc_private->fuse_info.num_fuses = num_fuse;
	soc_private->num_feature_info = num_feature;
	return 0;
}

static inline enum cam_sys_cache_config_types cam_cpas_find_type_from_string(
	const char *cache_name)
{
	if (strcmp(cache_name, "small-1") == 0)
		return CAM_LLCC_SMALL_1;
	else if (strcmp(cache_name, "small-2") == 0)
		return CAM_LLCC_SMALL_2;
	else if (strcmp(cache_name, "large-1") == 0)
		return CAM_LLCC_LARGE_1;
	else if (strcmp(cache_name, "large-2") == 0)
		return CAM_LLCC_LARGE_2;
	else if (strcmp(cache_name, "large-3") == 0)
		return CAM_LLCC_LARGE_3;
	else if (strcmp(cache_name, "large-4") == 0)
		return CAM_LLCC_LARGE_4;
	else
		return CAM_LLCC_MAX;
}

static int cam_cpas_parse_sys_cache_uids(
	struct device_node          *of_node,
	struct cam_cpas_private_soc *soc_private)
{
	enum cam_sys_cache_config_types type = CAM_LLCC_MAX;
	int num_caches, i, rc;
	uint32_t scid;

	soc_private->llcc_info = NULL;
	soc_private->num_caches = 0;

	num_caches = of_property_count_strings(of_node, "sys-cache-names");
	if (num_caches <= 0) {
		CAM_DBG(CAM_CPAS, "no cache-names found");
		return 0;
	}

	if (num_caches > CAM_LLCC_MAX) {
		CAM_ERR(CAM_CPAS,
			"invalid number of cache-names found: 0x%x",
			num_caches);
		return -EINVAL;
	}

	soc_private->llcc_info = kcalloc(num_caches,
		sizeof(struct cam_sys_cache_info), GFP_KERNEL);
	if (!soc_private->llcc_info)
		return -ENOMEM;

	for (i = 0; i < num_caches; i++) {
		rc = of_property_read_string_index(of_node, "sys-cache-names", i,
			&soc_private->llcc_info[i].name);
		if (rc) {
			CAM_ERR(CAM_CPAS, "failed to read cache-names at %d", i);
			goto end;
		}

		type = cam_cpas_find_type_from_string(
			soc_private->llcc_info[i].name);
		if (type == CAM_LLCC_MAX) {
			CAM_ERR(CAM_CPAS, "Unsupported cache found: %s",
				soc_private->llcc_info[i].name);
			rc = -EINVAL;
			goto end;
		}

		soc_private->llcc_info[i].type = type;
		rc = of_property_read_u32_index(of_node,
				"sys-cache-uids", i,
				&soc_private->llcc_info[i].uid);
		if (rc < 0) {
			CAM_ERR(CAM_CPAS,
				"unable to read sys cache uid at index %d", i);
			goto end;
		}

		soc_private->llcc_info[i].slic_desc =
			llcc_slice_getd(soc_private->llcc_info[i].uid);

		if (IS_ERR_OR_NULL(soc_private->llcc_info[i].slic_desc)) {
			CAM_ERR(CAM_CPAS,
				"Failed to get slice desc for uid %u",
				soc_private->llcc_info[i].uid);
			rc = -EINVAL;
			goto end;
		}

		scid = llcc_get_slice_id(soc_private->llcc_info[i].slic_desc);
		soc_private->llcc_info[i].scid = scid;
		soc_private->llcc_info[i].size =
			llcc_get_slice_size(soc_private->llcc_info[i].slic_desc);
		soc_private->llcc_info[i].staling_distance = 0;
		soc_private->llcc_info[i].mode = CAM_LLCC_STALING_MODE_CAPACITY;
		soc_private->llcc_info[i].op_type = CAM_LLCC_NOTIFY_STALING_EVICT;
		soc_private->num_caches++;

		CAM_DBG(CAM_CPAS,
			"Cache: %s uid: %u scid: %d size: %zukb",
			soc_private->llcc_info[i].name,
			soc_private->llcc_info[i].uid, scid,
			soc_private->llcc_info[i].size);
	}

	return 0;

end:
	kfree(soc_private->llcc_info);
	soc_private->llcc_info = NULL;
	return rc;
}

#ifdef CONFIG_DOMAIN_ID_SECURE_CAMERA
static int cam_cpas_parse_domain_id_mapping(struct device_node *of_node,
	struct cam_cpas_private_soc *soc_private)
{
	int count, i, rc = 0;

	soc_private->domain_id_info.num_domain_ids = 0;
	soc_private->domain_id_info.domain_id_supported = false;
	soc_private->domain_id_info.domain_id_entries = NULL;
	count = of_property_count_u32_elems(of_node, "domain-id");
	if (count <= 0) {
		CAM_DBG(CAM_CPAS, "No domain-id mapping found, count: %d", count);
		return rc;
	}

	/* This property will always be specified in pairs  */
	if (count % 2) {
		CAM_ERR(CAM_CPAS,
			"Mismatch in domain-id mapping, found %d number of entries", count);
		rc = -EINVAL;
		goto err;
	}

	soc_private->domain_id_info.num_domain_ids = count / 2;

	if (soc_private->domain_id_info.num_domain_ids > CAM_CPAS_DOMAIN_ID_MAX) {
		CAM_ERR(CAM_CPAS,
			"Number of domain id types: %u more than supported: %d",
			soc_private->domain_id_info.num_domain_ids, CAM_CPAS_DOMAIN_ID_MAX);
		rc = -EINVAL;
		goto err;
	}

	soc_private->domain_id_info.domain_id_entries =
		kcalloc(soc_private->domain_id_info.num_domain_ids,
			sizeof(struct cam_cpas_domain_id_mapping), GFP_KERNEL);
	if (!soc_private->domain_id_info.domain_id_entries) {
		CAM_ERR(CAM_CPAS,
			"Error allocating memory for %u domain-id mapping(s)",
			soc_private->domain_id_info.num_domain_ids);
		rc = -ENOMEM;
		goto err;
	}

	for (i = 0; i < soc_private->domain_id_info.num_domain_ids; i++) {
		struct cam_cpas_domain_id_mapping *domain_id_entry =
			&soc_private->domain_id_info.domain_id_entries[i];

		rc = of_property_read_u32_index(of_node, "domain-id",
		(i * 2), &domain_id_entry->domain_type);
		if (rc) {
			CAM_ERR(CAM_CPAS, "Error reading domain-id type entry at pos %d", i);
			rc = -EINVAL;
			goto err;
		}

		if (domain_id_entry->domain_type > CAM_CPAS_SECURE_DOMAIN) {
			CAM_ERR(CAM_CPAS, "Unexpected domain id type: %u",
				domain_id_entry->domain_type);
			rc =  -EINVAL;
			goto err;
		}

		rc = of_property_read_u32_index(of_node, "domain-id",
		((i * 2) + 1), &domain_id_entry->mapping_id);
		if (rc) {
			CAM_ERR(CAM_CPAS, "Error reading domain-id mapping id at pos %d", i);
			rc = -EINVAL;
			goto err;
		}

		CAM_DBG(CAM_CPAS, "Domain-id type: %u, mapping: %u at pos: %d",
			domain_id_entry->domain_type, domain_id_entry->mapping_id, i);
	}

	soc_private->domain_id_info.domain_id_supported = true;

	return rc;

err:
	soc_private->domain_id_info.num_domain_ids = 0;
	kfree(soc_private->domain_id_info.domain_id_entries);
	soc_private->domain_id_info.domain_id_entries = NULL;
	return rc;
}
#endif

static int cam_cpas_get_domain_id_support_clks(struct device_node *of_node,
	struct cam_hw_soc_info *soc_info, struct cam_cpas_private_soc *soc_private)
{
	int rc = 0, count, i;
	struct cam_cpas_domain_id_support_clks *domain_id_clks;

	soc_private->domain_id_clks = kzalloc(sizeof(struct cam_cpas_domain_id_support_clks),
		GFP_KERNEL);
	if (!soc_private->domain_id_clks) {
		CAM_ERR(CAM_CPAS, "Failed in allocating memory for domain_id_clk");
		return -ENOMEM;
	}

	domain_id_clks = soc_private->domain_id_clks;

	count = of_property_count_strings(of_node, "domain-id-support-clks");
	CAM_DBG(CAM_CPAS, "Domain-id clk count: %d", count);
	if (count > CAM_SOC_MAX_OPT_CLK) {
		CAM_ERR(CAM_CPAS, "Invalid cnt of clocks, count: %d", count);
		rc  = -EINVAL;
		goto err;
	}
	if (count <= 0) {
		CAM_ERR(CAM_CPAS, "No domain-id clk found");
		rc = -EINVAL;
		goto err;
	}

	domain_id_clks->number_clks = count;

	for (i = 0; i < count; i++) {
		rc = of_property_read_string_index(of_node, "domain-id-support-clks",
			i, &domain_id_clks->clk_names[i]);
		if (rc) {
			CAM_ERR(CAM_CPAS,
				"Failed reading domain-id clk name at i: %d, total clks: %d",
				i, count);
			rc = -EINVAL;
			goto err;
		}

		rc = cam_soc_util_get_option_clk_by_name(soc_info, domain_id_clks->clk_names[i],
			&domain_id_clks->clk_idx[i]);
		if (rc) {
			CAM_ERR(CAM_CPAS,
				"Failed reading domain-id clk %s at i: %d, total clks; %d",
					domain_id_clks->clk_names[i], i, count);
			rc = -EINVAL;
			goto err;
		}

	CAM_DBG(CAM_CPAS, "Domain-id-clk %s with clk index %d",
		domain_id_clks->clk_names[i], domain_id_clks->clk_idx[i]);

	}

	return rc;

err:
	kfree(domain_id_clks);
	return rc;
}

int cam_cpas_get_custom_dt_info(struct cam_hw_info *cpas_hw,
	struct platform_device *pdev, struct cam_cpas_private_soc *soc_private)
{
	struct device_node *of_node;
	struct of_phandle_args src_args = {0}, dst_args = {0};
	int count = 0, i = 0, rc = 0, num_bw_values = 0, num_levels = 0;
	uint32_t cam_drv_en_mask_val = 0;
	struct cam_cpas *cpas_core = (struct cam_cpas *) cpas_hw->core_info;
	uint32_t ahb_bus_client_ab = 0, ahb_bus_client_ib = 0;

	if (!soc_private || !pdev) {
		CAM_ERR(CAM_CPAS, "invalid input arg %pK %pK",
			soc_private, pdev);
		return -EINVAL;
	}

	of_node = pdev->dev.of_node;

	rc = of_property_read_string(of_node, "arch-compat",
		&soc_private->arch_compat);
	if (rc) {
		CAM_ERR(CAM_CPAS, "device %s failed to read arch-compat",
			pdev->name);
		return rc;
	}

	cam_cpas_get_hw_features(pdev, soc_private);

#ifdef CONFIG_DOMAIN_ID_SECURE_CAMERA
	/* get domain id mapping info */
	rc = cam_cpas_parse_domain_id_mapping(of_node, soc_private);
	if (rc)
		return rc;
	/* check if the domain ID configuration is available in the DTSI */
	if (soc_private->domain_id_info.domain_id_supported == false) {
		CAM_ERR(CAM_CPAS, "Domain ID configuration is expected for this target");
		return -EINVAL;
	}
#endif

	soc_private->camnoc_axi_min_ib_bw = 0;
	rc = of_property_read_u64(of_node,
		"camnoc-axi-min-ib-bw",
		&soc_private->camnoc_axi_min_ib_bw);
	if (rc == -EOVERFLOW) {
		soc_private->camnoc_axi_min_ib_bw = 0;
		rc = of_property_read_u32(of_node,
			"camnoc-axi-min-ib-bw",
			(u32 *)&soc_private->camnoc_axi_min_ib_bw);
	}

	if (rc) {
		CAM_DBG(CAM_CPAS,
			"failed to read camnoc-axi-min-ib-bw rc:%d", rc);
		soc_private->camnoc_axi_min_ib_bw =
			CAM_CPAS_AXI_MIN_CAMNOC_IB_BW;
	}

	CAM_DBG(CAM_CPAS, "camnoc-axi-min-ib-bw = %llu",
		soc_private->camnoc_axi_min_ib_bw);

	soc_private->client_id_based = of_property_read_bool(of_node,
		"client-id-based");
	soc_private->bus_icc_based = of_property_read_bool(of_node,
		"interconnects");

	if (soc_private->bus_icc_based) {
		rc = of_property_read_string(of_node, "interconnect-names",
			&cpas_core->ahb_bus_client.common_data.name);
		if (rc) {
			CAM_ERR(CAM_CPAS,
				"device %s failed to read interconnect-names",
				pdev->name);
			return rc;
		}

		rc = of_parse_phandle_with_args(of_node, "interconnects",
			"#interconnect-cells", 0, &src_args);
		if (rc) {
			CAM_ERR(CAM_CPAS,
				"device %s failed to read ahb bus src info",
				pdev->name);
			return rc;
		}

		of_node_put(src_args.np);
		if (src_args.args_count != 1) {
			CAM_ERR(CAM_CPAS,
				"Invalid number of ahb src args: %d",
				src_args.args_count);
			return -EINVAL;
		}

		cpas_core->ahb_bus_client.common_data.src_id = src_args.args[0];

		rc = of_parse_phandle_with_args(of_node, "interconnects",
			"#interconnect-cells", 1, &dst_args);
		if (rc) {
			CAM_ERR(CAM_CPAS,
				"device %s failed to read ahb bus dst info",
				pdev->name);
			return rc;
		}

		of_node_put(dst_args.np);
		if (dst_args.args_count != 1) {
			CAM_ERR(CAM_CPAS,
				"Invalid number of ahb dst args: %d",
				dst_args.args_count);
			return -EINVAL;
		}

		cpas_core->ahb_bus_client.common_data.dst_id = dst_args.args[0];

		rc = of_property_read_u32(of_node, "cam-ahb-num-cases",
			&cpas_core->ahb_bus_client.common_data.num_usecases);
		if (rc) {
			CAM_ERR(CAM_CPAS,
				"device %s failed to read ahb num usecases",
				pdev->name);
			return rc;
		}

		if (cpas_core->ahb_bus_client.common_data.num_usecases >
			CAM_SOC_BUS_MAX_NUM_USECASES) {
			CAM_ERR(CAM_UTIL, "Invalid number of usecases: %d",
				cpas_core->ahb_bus_client.common_data
				.num_usecases);
			return -EINVAL;
		}

		num_bw_values = of_property_count_u32_elems(of_node,
			"cam-ahb-bw-KBps");
		if (num_bw_values <= 0) {
			CAM_ERR(CAM_UTIL, "Error counting ahb bw values");
			return -EINVAL;
		}

		CAM_DBG(CAM_CPAS, "AHB: num bw values %d", num_bw_values);
		num_levels = (num_bw_values / 2);

		if (num_levels !=
			cpas_core->ahb_bus_client.common_data.num_usecases) {
			CAM_ERR(CAM_UTIL, "Invalid number of levels: %d",
				num_bw_values/2);
			return -EINVAL;
		}

		for (i = 0; i < num_levels; i++) {
			rc = of_property_read_u32_index(of_node,
				"cam-ahb-bw-KBps",
				(i * 2),
				&ahb_bus_client_ab);
			if (rc) {
				CAM_ERR(CAM_UTIL,
					"Error reading ab bw value, rc=%d",
					rc);
				return rc;
			}
			cpas_core->ahb_bus_client.common_data.bw_pair[i].ab = ahb_bus_client_ab;

			rc = of_property_read_u32_index(of_node,
				"cam-ahb-bw-KBps",
				((i * 2) + 1),
				&ahb_bus_client_ib);
			if (rc) {
				CAM_ERR(CAM_UTIL,
					"Error reading ib bw value, rc=%d",
					rc);
				return rc;
			}
			cpas_core->ahb_bus_client.common_data.bw_pair[i].ib = ahb_bus_client_ib;

			CAM_DBG(CAM_CPAS,
				"AHB: Level: %d, ab_value %llu, ib_value: %llu",
				i, cpas_core->ahb_bus_client.common_data
				.bw_pair[i].ab, cpas_core->ahb_bus_client
				.common_data.bw_pair[i].ib);
		}
	}

	count = of_property_count_strings(of_node, "client-names");
	if (count <= 0) {
		CAM_ERR(CAM_CPAS, "no client-names found");
		return -EINVAL;
	} else if (count > CAM_CPAS_MAX_CLIENTS) {
		CAM_ERR(CAM_CPAS, "Number of clients %d greater than max %d",
			count, CAM_CPAS_MAX_CLIENTS);
		return -EINVAL;
	}

	soc_private->num_clients = count;
	CAM_DBG(CAM_CPAS,
		"arch-compat=%s, client_id_based = %d, num_clients=%d",
		soc_private->arch_compat, soc_private->client_id_based,
		soc_private->num_clients);

	for (i = 0; i < soc_private->num_clients; i++) {
		rc = of_property_read_string_index(of_node,
			"client-names", i, &soc_private->client_name[i]);
		if (rc) {
			CAM_ERR(CAM_CPAS, "no client-name at cnt=%d", i);
			return -EINVAL;
		}

		cpas_core->cpas_client[i] =
			kzalloc(sizeof(struct cam_cpas_client), GFP_KERNEL);
		if (!cpas_core->cpas_client[i]) {
			rc = -ENOMEM;
			goto cleanup_clients;
		}

		CAM_DBG(CAM_CPAS, "Client[%d] : %s", i,
			soc_private->client_name[i]);
	}

	soc_private->control_camnoc_axi_clk = of_property_read_bool(of_node,
		"control-camnoc-axi-clk");

	if (soc_private->control_camnoc_axi_clk == true) {
		rc = of_property_read_u32(of_node, "camnoc-bus-width",
			&soc_private->camnoc_bus_width);
		if (rc || (soc_private->camnoc_bus_width == 0)) {
			CAM_ERR(CAM_CPAS, "Bus width not found rc=%d, %d",
				rc, soc_private->camnoc_bus_width);
			goto cleanup_clients;
		}

		if (of_property_read_u32(of_node,
			"camnoc-axi-clk-bw-margin-perc",
			&soc_private->camnoc_axi_clk_bw_margin)) {

			/* this is not fatal, overwrite to 0 */
			soc_private->camnoc_axi_clk_bw_margin = 0;
		}
	}

	CAM_DBG(CAM_CPAS,
		"control_camnoc_axi_clk=%d, width=%d, margin=%d",
		soc_private->control_camnoc_axi_clk,
		soc_private->camnoc_bus_width,
		soc_private->camnoc_axi_clk_bw_margin);

	count = of_property_count_u32_elems(of_node, "vdd-corners");
	if ((count > 0) && (count <= CAM_REGULATOR_LEVEL_MAX) &&
		(of_property_count_strings(of_node, "vdd-corner-ahb-mapping") ==
		count)) {
		const char *ahb_string;

		for (i = 0; i < count; i++) {
			rc = of_property_read_u32_index(of_node, "vdd-corners",
				i, &soc_private->vdd_ahb[i].vdd_corner);
			if (rc) {
				CAM_ERR(CAM_CPAS,
					"vdd-corners failed at index=%d", i);
				rc = -ENODEV;
				goto cleanup_clients;
			}

			rc = of_property_read_string_index(of_node,
				"vdd-corner-ahb-mapping", i, &ahb_string);
			if (rc) {
				CAM_ERR(CAM_CPAS,
					"no ahb-mapping at index=%d", i);
				rc = -ENODEV;
				goto cleanup_clients;
			}

			rc = cam_soc_util_get_level_from_string(ahb_string,
				&soc_private->vdd_ahb[i].ahb_level);
			if (rc) {
				CAM_ERR(CAM_CPAS,
					"invalid ahb-string at index=%d", i);
				rc = -EINVAL;
				goto cleanup_clients;
			}

			CAM_DBG(CAM_CPAS,
				"Vdd-AHB mapping [%d] : [%d] [%s] [%d]", i,
				soc_private->vdd_ahb[i].vdd_corner,
				ahb_string, soc_private->vdd_ahb[i].ahb_level);
		}

		soc_private->num_vdd_ahb_mapping = count;
	}

	soc_private->enable_smart_qos = of_property_read_bool(of_node,
			"enable-smart-qos");

	if (soc_private->enable_smart_qos) {
		uint32_t value;

		soc_private->smart_qos_info = kzalloc(sizeof(struct cam_cpas_smart_qos_info),
			GFP_KERNEL);
		if (!soc_private->smart_qos_info) {
			rc = -ENOMEM;
			goto cleanup_clients;
		}

		/*
		 * If enabled, we expect min and max priority values,
		 * clamp priority value, slope factor, least and most
		 * stressed clamp threshold values, high and low stress
		 * indicator threshold values, bw ratio scale factor value,
		 * so treat as fatal error if not available.
		 */
		rc = of_property_read_u32(of_node, "rt-wr-priority-min",
			&value);
		if (rc) {
			CAM_ERR(CAM_CPAS, "Invalid min Qos priority rc %d", rc);
			goto cleanup_clients;
		}
		soc_private->smart_qos_info->rt_wr_priority_min = (uint8_t)value;

		rc = of_property_read_u32(of_node, "rt-wr-priority-max",
			&value);
		if (rc) {
			CAM_ERR(CAM_CPAS, "Invalid max Qos priority rc %d", rc);
			goto cleanup_clients;
		}
		soc_private->smart_qos_info->rt_wr_priority_max = (uint8_t)value;

		rc = of_property_read_u32(of_node, "rt-wr-priority-clamp",
			&value);
		if (rc) {
			CAM_ERR(CAM_CPAS, "Invalid clamp Qos priority rc %d", rc);
			goto cleanup_clients;
		}
		soc_private->smart_qos_info->rt_wr_priority_clamp = (uint8_t)value;

		rc = of_property_read_u32(of_node, "rt-wr-slope-factor",
			&value);
		if (rc) {
			CAM_ERR(CAM_CPAS, "Invalid slope factor rc %d", rc);
			goto cleanup_clients;
		}

		if (value > CAM_CPAS_MAX_SLOPE_FACTOR) {
			CAM_ERR(CAM_UTIL, "Invalid slope factor value %d", value);
			rc = -EINVAL;
			goto cleanup_clients;
		} else
			soc_private->smart_qos_info->rt_wr_slope_factor = (uint8_t)value;

		CAM_DBG(CAM_CPAS,
			"SmartQoS enabled, priority min=%u, max=%u, clamp=%u, slope factor=%u",
			(uint32_t)soc_private->smart_qos_info->rt_wr_priority_min,
			(uint32_t)soc_private->smart_qos_info->rt_wr_priority_max,
			(uint32_t)soc_private->smart_qos_info->rt_wr_priority_clamp,
			(uint32_t)soc_private->smart_qos_info->rt_wr_slope_factor);

		rc = of_property_read_u32(of_node, "rt-wr-leaststressed-clamp-threshold",
			&value);
		if (rc) {
			CAM_ERR(CAM_CPAS, "Invalid leaststressed clamp threshold rc %d", rc);
			goto cleanup_clients;
		}
		soc_private->smart_qos_info->leaststressed_clamp_th = (uint8_t)value;

		rc = of_property_read_u32(of_node, "rt-wr-moststressed-clamp-threshold",
			&value);
		if (rc) {
			CAM_ERR(CAM_CPAS, "Invalid moststressed clamp threshold rc %d", rc);
			goto cleanup_clients;
		}
		soc_private->smart_qos_info->moststressed_clamp_th = (uint8_t)value;

		CAM_DBG(CAM_CPAS,
			"leaststressed_clamp_th=%u, moststressed_clamp_th=%u",
			(uint32_t)soc_private->smart_qos_info->leaststressed_clamp_th,
			(uint32_t)soc_private->smart_qos_info->moststressed_clamp_th);

		rc = of_property_read_u32(of_node, "rt-wr-highstress-indicator-threshold",
			&value);
		if (rc) {
			CAM_ERR(CAM_CPAS, "Invalid highstress indicator threshold rc %d", rc);
			goto cleanup_clients;
		}

		if (value > CAM_CPAS_MAX_STRESS_INDICATOR) {
			CAM_ERR(CAM_UTIL, "Invalid highstress indicator threshold value %d", value);
			rc = -EINVAL;
			goto cleanup_clients;
		} else
			soc_private->smart_qos_info->highstress_indicator_th = (uint8_t)value;

		rc = of_property_read_u32(of_node, "rt-wr-lowstress-indicator-threshold",
			&value);
		if (rc) {
			CAM_ERR(CAM_CPAS, "Invalid lowstress indicator threshold rc %d", rc);
			goto cleanup_clients;
		}

		if (value > CAM_CPAS_MAX_STRESS_INDICATOR) {
			CAM_ERR(CAM_UTIL, "Invalid lowstress indicator threshold value %d", value);
			rc = -EINVAL;
			goto cleanup_clients;
		} else
			soc_private->smart_qos_info->lowstress_indicator_th = (uint8_t)value;

		rc = of_property_read_u32(of_node, "rt-wr-bw-ratio-scale-factor",
			&value);
		if (rc) {
			CAM_ERR(CAM_CPAS, "Invalid bw ratio scale factor rc %d", rc);
			goto cleanup_clients;
		}
		soc_private->smart_qos_info->bw_ratio_scale_factor = (uint8_t)value;

		CAM_DBG(CAM_CPAS,
			"highstress_indicator_th=%u, lowstress_indicator_th=%u, scale factor=%u",
			(uint32_t)soc_private->smart_qos_info->highstress_indicator_th,
			(uint32_t)soc_private->smart_qos_info->lowstress_indicator_th,
			(uint32_t)soc_private->smart_qos_info->bw_ratio_scale_factor);
	} else {
		CAM_DBG(CAM_CPAS, "SmartQoS not enabled, use static settings");
		soc_private->smart_qos_info = NULL;
	}

	rc = of_property_read_u32(of_node, "enable-cam-drv", &cam_drv_en_mask_val);

	if (!rc) {
		if (cam_drv_en_mask_val & CAM_DDR_DRV)
			soc_private->enable_cam_ddr_drv = true;

		if (cam_drv_en_mask_val & CAM_CLK_DRV) {
			if (!soc_private->enable_cam_ddr_drv) {
				CAM_ERR(CAM_CPAS, "DDR DRV needs to be enabled for Clock DRV");
				rc = -EPERM;
				goto cleanup_clients;
			}

			soc_private->enable_cam_clk_drv = true;
			rc = cam_soc_util_cesta_populate_crm_device();
			if (rc) {
				CAM_ERR(CAM_CPAS, "Failed to populate cam cesta crm device rc %d",
					rc);
				goto cleanup_clients;
			}
		}
	}

	CAM_DBG(CAM_CPAS, "enable_cam_ddr_drv %d enable_cam_clk_drv %d cam_drv_en_mask_val %d",
		soc_private->enable_cam_ddr_drv, soc_private->enable_cam_clk_drv,
		cam_drv_en_mask_val);

	rc = cam_cpas_parse_node_tree(cpas_core, of_node, soc_private);
	if (rc) {
		CAM_ERR(CAM_CPAS, "Node tree parsing failed rc: %d", rc);
		goto cleanup_tree;
	}

	/* If SmartQoS is enabled, we expect few tags in dtsi, validate */
	if (soc_private->enable_smart_qos) {
		int port_idx;
		bool rt_port_exists = false;

		if ((soc_private->smart_qos_info->num_rt_wr_nius == 0) ||
			(soc_private->smart_qos_info->num_rt_wr_nius >
			CAM_CPAS_MAX_RT_WR_NIU_NODES)) {
			CAM_ERR(CAM_CPAS, "Invalid number of level1 nodes %d",
				soc_private->smart_qos_info->num_rt_wr_nius);
			rc = -EINVAL;
			goto cleanup_tree;
		}

		for (port_idx = 0; port_idx < cpas_core->num_axi_ports;
			port_idx++) {
			CAM_DBG(CAM_CPAS, "[%d] : Port[%s] is_rt=%d",
				port_idx, cpas_core->axi_port[port_idx].axi_port_name,
				cpas_core->axi_port[port_idx].is_rt);
			if (cpas_core->axi_port[port_idx].is_rt) {
				rt_port_exists = true;
				break;
			}
		}

		if (!rt_port_exists) {
			CAM_ERR(CAM_CPAS,
				"RT AXI port not tagged, num ports %d",
				cpas_core->num_axi_ports);
			rc = -EINVAL;
			goto cleanup_tree;
		}
	}

	/* Optional rpmh bcm info */
	count = of_property_count_u32_elems(of_node, "rpmh-bcm-info");
	/*
	 * We expect count=5(CAM_RPMH_BCM_INFO_MAX) if valid rpmh bcm info
	 * is available.
	 * 0 - Total number of BCMs
	 * 1 - First BCM FE (front-end) register offset.
	 *     These represent requested clk plan by sw
	 * 2 - First BCM BE (back-end) register offset.
	 *     These represent actual clk plan at hw
	 * 3 - DDR BCM index
	 * 4 - MMNOC BCM index
	 */
	if (count == CAM_RPMH_BCM_INFO_MAX) {
		for (i = 0; i < count; i++) {
			rc = of_property_read_u32_index(of_node,
				"rpmh-bcm-info", i, &soc_private->rpmh_info[i]);
			if (rc) {
				CAM_ERR(CAM_CPAS,
					"Incorrect rpmh info at %d, count=%d",
					i, count);
				break;
			}
			CAM_DBG(CAM_CPAS, "RPMH BCM Info [%d]=0x%x",
				i, soc_private->rpmh_info[i]);
		}

		if (rc)
			soc_private->rpmh_info[CAM_RPMH_NUMBER_OF_BCMS] = 0;
	} else {
		CAM_DBG(CAM_CPAS, "RPMH BCM info not available in DT, count=%d",
			count);
	}

	/* check cache info */
	rc = cam_cpas_parse_sys_cache_uids(of_node, soc_private);
	if (rc)
		goto cache_parse_fail;

	return 0;

cache_parse_fail:
	soc_private->rpmh_info[CAM_RPMH_NUMBER_OF_BCMS] = 0;
cleanup_tree:
	cam_cpas_node_tree_cleanup(cpas_core, soc_private);
cleanup_clients:
	cam_cpas_util_client_cleanup(cpas_hw);
	return rc;
}

static int cam_cpas_soc_fill_irq_data(struct cam_hw_info *cpas_hw,
	struct cam_hw_soc_info *soc_info, void **irq_data)
{
	struct cam_cpas_private_soc *soc_private = soc_info->soc_private;
	int i;

	for (i = 0; i < soc_info->irq_count; i++) {
		soc_private->irq_data[i].cpas_hw = cpas_hw;

		if (!strcmp(soc_info->irq_name[i], "cpas_camnoc"))
			soc_private->irq_data[i].camnoc_type = CAM_CAMNOC_HW_COMBINED;
		else if (!strcmp(soc_info->irq_name[i], "cpas_camnoc_rt"))
			soc_private->irq_data[i].camnoc_type = CAM_CAMNOC_HW_RT;
		else if (!strcmp(soc_info->irq_name[i], "cpas_camnoc_nrt"))
			soc_private->irq_data[i].camnoc_type = CAM_CAMNOC_HW_NRT;
		else {
			CAM_ERR(CAM_CPAS, "Unable to identify interrupt name: %s",
				soc_info->irq_name[i]);
			return -EINVAL;
		}

		irq_data[i] = &soc_private->irq_data[i];
	}

	return 0;
}

int cam_cpas_soc_init_resources(struct cam_hw_soc_info *soc_info,
	irq_handler_t irq_handler, struct cam_hw_info *cpas_hw)
{
	int rc = 0;
	struct cam_cpas_private_soc *soc_private;
	void *irq_data[CAM_SOC_MAX_IRQ_LINES_PER_DEV] = {0};

	rc = cam_soc_util_get_dt_properties(soc_info);
	if (rc) {
		CAM_ERR(CAM_CPAS, "failed in get_dt_properties, rc=%d", rc);
		return rc;
	}

	if (soc_info->irq_count > 0 && !irq_handler) {
		CAM_ERR(CAM_CPAS, "Invalid IRQ handler");
		return -EINVAL;
	}

	soc_info->soc_private = kzalloc(sizeof(struct cam_cpas_private_soc),
		GFP_KERNEL);
	if (!soc_info->soc_private) {
		CAM_ERR(CAM_CPAS, "Failed to allocate soc private");
		return -ENOMEM;
	}
	soc_private = (struct cam_cpas_private_soc *)soc_info->soc_private;

	rc = cam_cpas_get_custom_dt_info(cpas_hw, soc_info->pdev, soc_private);
	if (rc) {
		CAM_ERR(CAM_CPAS, "failed in get_custom_info, rc=%d", rc);
		goto free_soc_private;
	}

	soc_private->irq_data = kcalloc(soc_info->irq_count, sizeof(struct cam_cpas_soc_irq_data),
		GFP_KERNEL);
	if (!soc_private->irq_data) {
		CAM_ERR(CAM_CPAS, "Failed to allocate irq data");
		rc = -ENOMEM;
		goto free_soc_private;
	}

	rc = cam_cpas_soc_fill_irq_data(cpas_hw, soc_info, &(irq_data[0]));
	if (rc) {
		CAM_ERR(CAM_CPAS, "Failed to fill irq data rc=%d", rc);
		goto free_irq_data;
	}

	soc_info->is_clk_drv_en = soc_private->enable_cam_clk_drv;

	rc = cam_soc_util_request_platform_resource(soc_info, irq_handler, &(irq_data[0]));
	if (rc) {
		CAM_ERR(CAM_CPAS, "failed in request_platform_resource, rc=%d", rc);
		goto free_irq_data;
	}

	rc = cam_soc_util_get_option_clk_by_name(soc_info, CAM_ICP_CLK_NAME,
		&soc_private->icp_clk_index);
	if (rc) {
		CAM_DBG(CAM_CPAS, "ICP option clk get failed with rc %d", rc);
		soc_private->icp_clk_index = -1;
		rc = 0;
	} else {
		CAM_DBG(CAM_CPAS, "ICP option clk get success index %d",
			soc_private->icp_clk_index);
	}

	if (soc_private->domain_id_info.domain_id_supported) {
		rc = cam_cpas_get_domain_id_support_clks(soc_info->pdev->dev.of_node,
			soc_info, soc_private);
		if (rc)
			goto release_res;
	}

	return rc;

release_res:
	cam_soc_util_release_platform_resource(soc_info);
free_irq_data:
	kfree(soc_private->irq_data);
free_soc_private:
	kfree(soc_private->llcc_info);
	kfree(soc_private->smart_qos_info);
	kfree(soc_info->soc_private);
	soc_info->soc_private = NULL;
	return rc;
}

int cam_cpas_soc_deinit_resources(struct cam_hw_soc_info *soc_info)
{
	int rc, i;
	struct cam_cpas_private_soc *soc_private = soc_info->soc_private;

	for (i = 0; i < soc_private->num_caches; i++)
		llcc_slice_putd(soc_private->llcc_info[i].slic_desc);

	if (soc_private->icp_clk_index != -1) {
		rc = cam_soc_util_put_optional_clk(soc_info, soc_private->icp_clk_index);
		if (rc)
			CAM_ERR(CAM_CPAS, "Error Put optional clk failed rc=%d", rc);
	}

	kfree(soc_private->domain_id_info.domain_id_entries);

	kfree(soc_private->domain_id_clks);

	rc = cam_soc_util_release_platform_resource(soc_info);
	if (rc)
		CAM_ERR(CAM_CPAS, "release platform failed, rc=%d", rc);

	kfree(soc_private->irq_data);
	kfree(soc_private->llcc_info);
	kfree(soc_private->smart_qos_info);
	kfree(soc_info->soc_private);
	soc_info->soc_private = NULL;

	return rc;
}

int cam_cpas_soc_enable_resources(struct cam_hw_soc_info *soc_info,
	enum cam_vote_level default_level)
{
	struct cam_cpas_private_soc *soc_private = soc_info->soc_private;
	int rc = 0;

	/* set this everytime in order to support debugfs to disable clk drv between runs */
	soc_info->is_clk_drv_en = soc_private->enable_cam_clk_drv;

	rc = cam_soc_util_enable_platform_resource(soc_info, CAM_CLK_SW_CLIENT_IDX, true,
		default_level, true);
	if (rc)
		CAM_ERR(CAM_CPAS, "enable platform resource failed, rc=%d", rc);

	return rc;
}

int cam_cpas_soc_disable_resources(struct cam_hw_soc_info *soc_info,
	bool disable_clocks, bool disable_irq)
{
	int rc = 0;

	rc = cam_soc_util_disable_platform_resource(soc_info, CAM_CLK_SW_CLIENT_IDX,
		disable_clocks, disable_irq);
	if (rc)
		CAM_ERR(CAM_CPAS, "disable platform failed, rc=%d", rc);

	return rc;
}

int cam_cpas_soc_disable_irq(struct cam_hw_soc_info *soc_info)
{
	int rc = 0;

	rc = cam_soc_util_irq_disable(soc_info);
	if (rc)
		CAM_ERR(CAM_CPAS, "disable irq failed, rc=%d", rc);

	return rc;
}
