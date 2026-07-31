// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/regmap.h>
#include <linux/scmi_protocol.h>
#include <linux/qcom_scmi_vendor.h>
#include <soc/qcom/slc_perfmon_scmi.h>

#define SLC_PERFMON_ALGO_STR		0x534c432d50455246
#define MAX_SHARED_MEM_RETRY_CNT		5000
#define MAX_SHARED_MATCH_SEQ_CNT		10

#define DELIM_CHAR			" "
#define UNKNOWN_EVENT			255

/* dummy value to assign for cycle count */
#define CLOCK_PORT			0xff
#define CLOCK_EVENT			0xffff

/* Payload sizing */
#define SCMI_PAYLOAD_SIZE		(5 * sizeof(struct filter_config))

/* TRP offsets */
#define TRP_CAP_COUNTERS_DUMP_CFG	(0x48100)
#define TRP_SCID_n_STATUS(n)		(0x000004 + (0x1000 * (n)))
#define TRP_SCID_STATUS_CURRENT_CAP_SHIFT	(16)
#define TRP_SCID_STATUS_CURRENT_CAP_MASK	GENMASK(TRP_SCID_STATUS_CURRENT_CAP_SHIFT + 15,\
						TRP_SCID_STATUS_CURRENT_CAP_SHIFT)
#define TRP_SCID_STATUS_ACTIVE_SHIFT	(0)
#define TRP_SCID_STATUS_ACTIVE_MASK	GENMASK(TRP_SCID_STATUS_ACTIVE_SHIFT + 0,\
						TRP_SCID_STATUS_ACTIVE_SHIFT)

#define VERSION_6			6
#define FLTR_TYPE_LEN			70

/**
 * struct filter_attribute	filter attribute
 *
 * @is_filter_configured:	Flag for filter configuration applied.
 * @is_fltr_sent:		Flag for filter configuration payload sent using SCMI.
 * @fltr_cfg_payload_size:	Filter configuration payload size.
 *
 */
struct filter_attribute {
	bool is_filter_configured;
	bool is_fltr_sent;
	u8 fltr_cfg_payload_size;
};

/**
 * struct counter_attribute	counter attribute
 *
 * @is_perfmon_running:		Flag for whether perfmon is running.
 * @is_perfmon_configured:	Flag for whether perfmon is configured.
 * @configured_cntrs:		Count of configured counters.
 * @cntr_cfg_payload_size:	Counter configuration payload size.
 */
struct counter_attribute {
	bool is_perfmon_running;
	bool is_perfmon_configured;
	u8 configured_cntrs;
	u8 cntr_cfg_payload_size;
};

/**
 * struct filtered_port	set of filtered ports.
 *
 * @port:		bit field for ports on which filters applied.
 */
struct filtered_port {
	u16 port;
};

/**
 * struct slc_perfmon_private	slc perfmon private
 *
 * @clock_enabled:		Flag for clock enablement.
 * @filter_config_payload:	Filter payload.
 * @curr_fltr_payload_ptr:	Current position in filter payload
 * @counter_config_payload:	Counter payload.
 * @curr_cntr_payload_ptr:	Current position in counter payload
 * @timer:			Timer functionality.
 * @filter_payload_capacity:	Total filter payload carrying capacity.
 * @counter_payload_capacity:	Total counter payload carrying capacity.
 * @perfmon_offset:		PERFMON memory offset.
 * @dump:			Array of counter dumps.
 * @fltr_port:			Port filter enabled for FILTER0 and FILTER1
 * @prt_evnt_map:		Counter configuration for dump rendering
 * @sdev:			SCMI device struct
 * @ph:				SCMI Protocol Handle
 * @ops:			SCMI Ops
 * @clock:			clock node to enable qdss.
 * @info_attr			SLC Information attributes.
 * @perf_info_attr		PERFMON Information attributes.
 * @fltr_attr:			Filter attribute
 * @cntr_attr:			Counter attribute
 * @mutex:			To protect private structure
 * @res:			PERFMON resource
 * @perfmon_base:		PERFMON memory base.
 * @slc_maps:			SLC register address space mappings
 * @slc_bcast_map:		SLC broadcast register address space map
 * @scid_status_trigger:		Flag for trigger based scid_status dump
 * @num_ports:			Number of ports register.
 */
struct slc_perfmon_private {
	bool clock_enabled;
	u8 *filter_config_payload;
	u8 *curr_fltr_payload_ptr;
	u8 *counter_config_payload;
	u8 *curr_cntr_payload_ptr;
	u32 timer;
	u32 filter_payload_capacity;
	u32 counter_payload_capacity;
	u32 perfmon_offset;
	u64 *dump;
	struct filtered_port *fltr_port;
	struct port_event_config *prt_evnt_map;
	struct scmi_device *sdev;
	struct scmi_protocol_handle *ph;
	const struct qcom_scmi_vendor_ops *ops;
	struct clk *clock;
	struct slc_info_attr *info_attr;
	struct slc_perfmon_info_attr *perf_info_attr;
	struct filter_attribute fltr_attr;
	struct counter_attribute cntr_attr;
	struct mutex mutex;
	struct resource *res;
	struct qcom_slc_perfmon_mem __iomem *perfmon_base;
	struct regmap **slc_maps;
	struct regmap *slc_bcast_map;
	bool scid_status_trigger;
	u8 num_ports;
};

static char *ports_v5[] = {
	"FEAC",
	"FERC",
	"FEWC",
	"BEAC",
	"BERC",
	"TRP",
	"DRP",
};

static char *ports_v6[] = {
	"FEAC",
	"FERC",
	"FEWC",
	"EWB",
	"BERC",
	"TRP",
	"DRP",
};

static char *filters[] = {
	" SCID MULTISCID",
	" MID",
	" PROFILING_TAG",
	" OPCODE",
	" CACHEALLOC",
	" MEMTAGOPS"
};

/* filter index from input */
static bool find_filter_index(const char *token, u8 *filter_idx)
{
	if (sysfs_streq(token, "FILTER0")) {
		*filter_idx = FILTER_0;
		return true;
	} else if (sysfs_streq(token, "FILTER1")) {
		*filter_idx = FILTER_1;
		return true;
	} else {
		return false;
	}
}

/* Find filter type from input */
static enum filter_type find_filter_type(char *filter)
{
	enum filter_type ret = UNKNOWN_FILTER;

	if (!strcmp(filter, "SCID"))
		ret = SCID;
	else if (!strcmp(filter, "MULTISCID"))
		ret = MULTISCID;
	else if (!strcmp(filter, "MID"))
		ret = MID;
	else if (!strcmp(filter, "PROFILING_TAG"))
		ret = PROFTAG;
	else if (!strcmp(filter, "OPCODE"))
		ret = OPCODE;
	else if (!strcmp(filter, "CACHEALLOC"))
		ret = CACHEALLOC;
	else if (!strcmp(filter, "MEMTAGOPS"))
		ret = MEMTAGOPS;

	return ret;
}

/* Filter Payload clean-up and resetting the related attributes */
static void fltr_payload_cleanup(struct slc_perfmon_private *slc_priv)
{
	memset(slc_priv->filter_config_payload, 0, slc_priv->filter_payload_capacity);
	slc_priv->curr_fltr_payload_ptr = slc_priv->filter_config_payload;
	memset(slc_priv->fltr_port, 0,
		sizeof(struct filtered_port) * slc_priv->info_attr->max_fltr_idx);
	memset(&slc_priv->fltr_attr, 0, sizeof(struct filter_attribute));
	slc_priv->fltr_attr.fltr_cfg_payload_size = 0;
}

/* Counter Payload clean-up and resetting the related attributes */
static void cntr_payload_cleanup(struct slc_perfmon_private *slc_priv)
{
	memset(slc_priv->counter_config_payload, 0, slc_priv->counter_payload_capacity);
	slc_priv->curr_cntr_payload_ptr = slc_priv->counter_config_payload;
	memset(slc_priv->fltr_port, 0,
		sizeof(struct port_event_config) * slc_priv->info_attr->max_cntr);
	memset(&slc_priv->cntr_attr, 0, sizeof(struct counter_attribute));
	memset(slc_priv->prt_evnt_map, 0, sizeof(struct port_event_config));
	slc_priv->cntr_attr.cntr_cfg_payload_size = 0;
	slc_priv->cntr_attr.is_perfmon_configured = false;
	slc_priv->timer = 0;
}

/* Send SCMI set command */
static int send_set_scmi(struct slc_perfmon_private *slc_priv,
				int attr, void *payload, int size)
{
	return slc_priv->ops->set_param(slc_priv->ph, payload, SLC_PERFMON_ALGO_STR, attr, size);
}

/* Receive SCMI get command */
static int receive_get_scmi(struct slc_perfmon_private *slc_priv, int attr)
{
	int ret = 0;
	u8 size = 0;

	switch (attr) {
	case PERFMON_SLC_COMMON_INFO:
		size = sizeof(struct slc_info_attr);
		ret = slc_priv->ops->get_param(slc_priv->ph, (void *)slc_priv->info_attr,
			SLC_PERFMON_ALGO_STR, attr, size, size);

		break;

	case PERFMON_CONFIG_INFO:
		size = sizeof(struct slc_perfmon_info_attr) + (slc_priv->num_ports * sizeof(u8));
		ret = slc_priv->ops->get_param(slc_priv->ph, (void *)slc_priv->perf_info_attr,
			SLC_PERFMON_ALGO_STR, attr, size, size);

		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

/* Check for the match and mask value for respective filter type */
static int check_filter_match(struct slc_perfmon_info_attr *perf_info_attr,
				enum filter_type fltr_type, u64 fltr_match, u64 fltr_mask)
{
	int ret = 0;
	u16 *filter_max_match = perf_info_attr->filter_max_match;
	u64 mch = 0;

	switch (fltr_type) {
	case SCID:
		if (fltr_match >= filter_max_match[SCID] || fltr_mask >= filter_max_match[SCID])
			ret = -EINVAL;
		break;

	case MULTISCID:
		mch = ((1ull << filter_max_match[SCID]) - 1);
		if (fltr_match > mch || fltr_mask > mch)
			ret = -EINVAL;
		break;

	case MID:
		if (fltr_match > filter_max_match[MID] || fltr_mask > filter_max_match[MID])
			ret = -EINVAL;
		break;

	case PROFTAG:
		if (fltr_match > filter_max_match[PROFTAG] ||
			fltr_mask > filter_max_match[PROFTAG])
			ret = -EINVAL;
		break;

	case OPCODE:
		if (fltr_match > filter_max_match[OPCODE] || fltr_mask > filter_max_match[OPCODE])
			ret = -EINVAL;
		break;

	case CACHEALLOC:
		if (fltr_match > filter_max_match[CACHEALLOC] ||
			fltr_mask > filter_max_match[CACHEALLOC])
			ret = -EINVAL;
		break;

	case MEMTAGOPS:
		if (fltr_match > filter_max_match[MEMTAGOPS] ||
			fltr_mask > filter_max_match[MEMTAGOPS])
			ret = -EINVAL;
		break;

	default:
		break;
	}

	return ret;
}

/* Filter configuration interface */
static ssize_t perfmon_filter_configure_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	char *token, *delim = DELIM_CHAR, fltr_str[15];
	u8 port, fltr_idx;
	enum filter_type fltr_type = UNKNOWN_FILTER;
	u16 fltr_ports = 0;
	u64 fltr_mask = 0, fltr_match = 0;
	struct filter_config *fltr_cfg;
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct slc_perfmon_private *slc_priv = platform_get_drvdata(pdev);

	mutex_lock(&slc_priv->mutex);
	fltr_idx = slc_priv->info_attr->max_fltr_idx;

	/* Check if PERFMON is already configured. */
	if (slc_priv->cntr_attr.is_perfmon_configured) {
		dev_err(dev, "FLTR_CFG_ERR: Perfmon is already configured, try again.\n");
		mutex_unlock(&slc_priv->mutex);
		return -EINVAL;
	}

	token = strsep((char **)&buf, delim);
	if (token) {
		fltr_type = find_filter_type(token);
		strscpy(fltr_str, token, sizeof(fltr_str));
	}

	if (fltr_type == UNKNOWN_FILTER) {
		dev_err(dev, "FLTR_CFG_ERR: Unsupported filter %s, try again.\n", token);
		goto filter_configure_err;
	}

	token = strsep((char **)&buf, delim);
	if (!token) {
		dev_err(dev, "FLTR_CFG_ERR: Wrong input, try again.\n");
		goto filter_configure_err;
	}

	if (kstrtou64(token, 0, &fltr_match)) {
		dev_err(dev, "FLTR_CFG_ERR: Wrong match format, try again.\n");
		goto filter_configure_err;
	}

	token = strsep((char **)&buf, delim);
	if (!token) {
		dev_err(dev, "FLTR_CFG_ERR: Wrong input, try again.\n");
		goto filter_configure_err;
	}

	if (kstrtou64(token, 0, &fltr_mask)) {
		dev_err(dev, "FLTR_CFG_ERR: Wrong mask input, try again.\n");
		goto filter_configure_err;
	}

	/* check if match value not exceeding respective filter tpye max match value */
	if (check_filter_match(slc_priv->perf_info_attr, fltr_type, fltr_match, fltr_mask)) {
		dev_err(dev, "FLTR_CFG_ERR: %s match/mask above MAX value, try again.\n",
			fltr_str);
		goto filter_configure_err;
	}

	if (fltr_type == SCID) {
		fltr_match = 1ull << fltr_match;
		fltr_mask = 1ull << fltr_mask;
	}

	if (fltr_type == MULTISCID)
		fltr_type = SCID;

	while (token) {
		token = strsep((char **)&buf, delim);
		if (!token)
			break;

		if (find_filter_index(token, &fltr_idx))
			break;

		if (kstrtou8(token, 0, &port)) {
			dev_err(dev, "FLTR_CFG_ERR: Wrong port input, try again.\n");
			goto filter_configure_err;
		}

		if (!((1 << port) & slc_priv->info_attr->port_registered)) {
			dev_err(dev, "FLTR_CFG_ERR: port doesn't exist, try again.\n");
			goto filter_configure_err;
		}

		/* check for the ports compatible with respective filters */
		if (!((1 << fltr_type) & slc_priv->perf_info_attr->filter_port_support[port])) {
			dev_err(dev, "FLTR_CFG_ERR: port %u don't support %s filter try again.\n",
			port, fltr_str);
			goto filter_configure_err;
		}

		fltr_ports |= (1 << port);
	}

	if (!fltr_ports) {
		dev_err(dev, "FLTR_CFG_ERR: no ports provided, try again.\n");
		goto filter_configure_err;
	}

	if (fltr_idx >= slc_priv->info_attr->max_fltr_idx) {
		dev_err(dev, "FLTR_CFG_ERR: Filter idx is not specified, try again.\n");
		goto filter_configure_err;
	}

	slc_priv->fltr_port[fltr_idx].port |= fltr_ports;

	if ((slc_priv->fltr_attr.fltr_cfg_payload_size + sizeof(struct filter_config))
		> slc_priv->filter_payload_capacity) {
		dev_err(dev, "FLTR_CFG_ERR: Configuration limit exceeds, try again.\n");
		goto filter_configure_err;
	}

	fltr_cfg = (struct filter_config *)slc_priv->curr_fltr_payload_ptr;
	fltr_cfg->fltr_ports = fltr_ports;
	/* Casting down to u8 for payload compatibility */
	fltr_cfg->fltr_type = (uint8_t)fltr_type;
	fltr_cfg->fltr_idx = fltr_idx;
	fltr_cfg->fltr_match = fltr_match;
	fltr_cfg->fltr_mask = fltr_mask;

	slc_priv->curr_fltr_payload_ptr += sizeof(struct filter_config);
	slc_priv->fltr_attr.fltr_cfg_payload_size += sizeof(struct filter_config);

	slc_priv->fltr_attr.is_filter_configured = true;
	dev_info(dev, "%s filter applied on FILTER%u.\n", fltr_str, fltr_idx);
	mutex_unlock(&slc_priv->mutex);
	return count;

filter_configure_err:
	fltr_payload_cleanup(slc_priv);
	mutex_unlock(&slc_priv->mutex);
	return -EINVAL;
}

/* Counter configuration interface */
static ssize_t perfmon_counter_configure_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	char *token, *delim = DELIM_CHAR;
	u8 fltr_idx, port, fltr_en = 0, j = 0, payload_size, size, curr_cntr_idx;
	u8 *cntr_payload = NULL;
	u16 event;
	int ret;
	struct counter_config *cntr_cfg;
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct slc_perfmon_private *slc_priv = platform_get_drvdata(pdev);

	mutex_lock(&slc_priv->mutex);
	fltr_idx = slc_priv->info_attr->max_fltr_idx;

	/* check if PERFMON is running */
	if (slc_priv->cntr_attr.is_perfmon_running) {
		dev_err(dev, "CNTR_CFG_ERR: Can't configure counters while PERFMON is running.\n");
		goto counter_configure_out;
	}

	/* check if configured counters are less than max_cntr */
	if (slc_priv->cntr_attr.configured_cntrs >= slc_priv->info_attr->max_cntr) {
		dev_err(dev, "CNTR_CFG_ERR: No. of configured counters exceeds maximum limit.\n");
		goto counter_configure_out;
	}

	if (slc_priv->cntr_attr.configured_cntrs) {
		j = slc_priv->cntr_attr.configured_cntrs;
		curr_cntr_idx = slc_priv->cntr_attr.configured_cntrs;
	}

	/* Check for the FILTER index */
	token = strsep((char **)&buf, delim);
	if (token && strlen(token) >= strlen("FILTERX")) {
		if (token && strlen(token) != strlen("FILTERX")) {
			dev_err(dev, "CNTR_CFG_ERR: Invalid filter, %s\n",
				"remove and try again.");
			goto counter_configure_err;
		}

		if (!find_filter_index(token, &fltr_idx)) {
			dev_err(dev, "CNTR_CFG_ERR: Wrong filter %s, %s\n",
				token, "remove and try again.");
			goto counter_configure_err;
		}

		if (!slc_priv->fltr_port[fltr_idx].port) {
			dev_err(dev, "CNTR_CFG_ERR: FILTER%u is not configured try again\n",
				fltr_idx);
			goto counter_configure_err;
		}

		fltr_en = 1;
		token = strsep((char **)&buf, delim);
	}

	/* Check for the port event pair */
	while (token) {
		if (j == slc_priv->info_attr->max_cntr) {
			dev_err(dev, "No. of cntrs exceeds max limit, skipping next config.\n");
			break;
		}

		if (kstrtou8(token, 0, &port)) {
			dev_err(dev, "CNTR_CFG_ERR: Invalid port %s, try again.\n", token);
			goto counter_configure_err;
		}

		if (!((1 << port) & slc_priv->info_attr->port_registered)) {
			dev_err(dev, "CNTR_CFG_ERR: port %u not registered, try again.\n", port);
			goto counter_configure_err;
		}

		token = strsep((char **)&buf, delim);
		if (!token)
			break;

		if (kstrtou16(token, 0, &event)) {
			dev_err(dev, "CNTR_CFG_ERR: Invalid event %s, try again.\n", token);
			goto counter_configure_err;
		}

		if (event >= UNKNOWN_EVENT) {
			dev_err(dev, "CNTR_CFG_ERR: Unsupported event %u, try again.\n", event);
			goto counter_configure_err;
		}

		token = strsep((char **)&buf, delim);

		/* Updating port-event map */
		slc_priv->prt_evnt_map[j].port = port;
		slc_priv->prt_evnt_map[j].event = event;

		/* Populating data in the counter payload */
		cntr_cfg = (struct counter_config *)slc_priv->curr_cntr_payload_ptr;
		cntr_cfg->prt_evnt_map.port = port;
		cntr_cfg->prt_evnt_map.event = event;
		cntr_cfg->fltr.fltr_en = 0;
		cntr_cfg->fltr.fltr_idx = 0;

		if (fltr_en && (slc_priv->fltr_port[fltr_idx].port & (1 << port))) {
			cntr_cfg->fltr.fltr_en = fltr_en;
			cntr_cfg->fltr.fltr_idx = fltr_idx;
			dev_info(dev, "Cntr %02u: port %u, event %03u FILTER%u configured.\n",
				j, cntr_cfg->prt_evnt_map.port, cntr_cfg->prt_evnt_map.event,
				cntr_cfg->fltr.fltr_idx);
		} else {
			dev_info(dev, "Cntr %02u: port %u event %03u configured.\n",
				j, cntr_cfg->prt_evnt_map.port, cntr_cfg->prt_evnt_map.event);
		}

		slc_priv->curr_cntr_payload_ptr += sizeof(struct counter_config);
		slc_priv->cntr_attr.cntr_cfg_payload_size += sizeof(struct counter_config);
		j++;
	}

	if (!j) {
		dev_err(dev, "CNTR_CFG_ERR: Port/Event not provided.\n");
		goto counter_configure_err;
	}

	slc_priv->cntr_attr.configured_cntrs = j;

	if (slc_priv->cntr_attr.configured_cntrs < slc_priv->info_attr->max_cntr) {
		slc_priv->prt_evnt_map[slc_priv->cntr_attr.configured_cntrs].port = CLOCK_PORT;
		slc_priv->prt_evnt_map[slc_priv->cntr_attr.configured_cntrs].event = CLOCK_EVENT;
	}

	/* After accumulating the counter configuration, send the filter payload (if configured)
	 * and counter payload. filter payload is sent only once even though there multiple counter
	 * configuration.
	 */
	/* Sending Filter Payload if filters are configured */
	if (slc_priv->fltr_attr.is_filter_configured && !slc_priv->fltr_attr.is_fltr_sent) {
		payload_size = slc_priv->fltr_attr.fltr_cfg_payload_size;
		slc_priv->curr_fltr_payload_ptr = slc_priv->filter_config_payload;

		do {
			size = payload_size > SCMI_PAYLOAD_SIZE ? SCMI_PAYLOAD_SIZE : payload_size;

			ret = send_set_scmi(slc_priv, PERFMON_FILTER_CONFIG,
				(void *)slc_priv->curr_fltr_payload_ptr, size);
			if (ret) {
				dev_err(dev, "Send SCMI: attr %d err: %d, %s\n",
					PERFMON_FILTER_CONFIG, ret, "remove and try again.");
				goto counter_configure_out;
			}

			payload_size -= size;
			slc_priv->curr_fltr_payload_ptr += size;
		} while (payload_size > 0);

		slc_priv->fltr_attr.is_fltr_sent = true;
	}

	/* While sending counter payload, in case of multiple sets of counter configuration,
	 * only the payload with the latest set of counters is sent,
	 * not the whole accumulated counter payload.
	 */
	/* Sending Counter Payload */
	if (curr_cntr_idx > 0) {
		cntr_payload = slc_priv->counter_config_payload + (curr_cntr_idx *
				sizeof(struct counter_config));
		payload_size = (slc_priv->cntr_attr.configured_cntrs - curr_cntr_idx) *
				sizeof(struct counter_config);
	} else {
		cntr_payload = slc_priv->counter_config_payload;
		payload_size = slc_priv->cntr_attr.cntr_cfg_payload_size;
	}

	if (payload_size > 0) {
		ret = send_set_scmi(slc_priv, PERFMON_COUNTER_CONFIG, (void *)cntr_payload,
			payload_size);
		if (ret) {
			dev_err(dev, "Send SCMI: attr %d err: %d, %s\n",
				PERFMON_COUNTER_CONFIG, ret, "remove and try again.");
			goto counter_configure_out;
		}
	}

	slc_priv->cntr_attr.is_perfmon_configured = true;
	mutex_unlock(&slc_priv->mutex);
	return count;

counter_configure_out:
	mutex_unlock(&slc_priv->mutex);
	return -EINVAL;

counter_configure_err:
	cntr_payload_cleanup(slc_priv);
	if (slc_priv->fltr_attr.is_filter_configured)
		fltr_payload_cleanup(slc_priv);
	mutex_unlock(&slc_priv->mutex);
	return -EINVAL;
}

static int perf_remove(struct device *dev, struct slc_perfmon_private *slc_priv)
{
	u8 payload_size = 0, size = 0;
	int ret = 0;

	/* Sending Counter Payload */
	if (slc_priv->cntr_attr.configured_cntrs) {
		payload_size = slc_priv->cntr_attr.cntr_cfg_payload_size;
		if (payload_size <= SCMI_PAYLOAD_SIZE) {
			ret = send_set_scmi(slc_priv, PERFMON_COUNTER_REMOVE,
				(void *)slc_priv->counter_config_payload, payload_size);
			if (ret) {
				dev_err(dev, "Send SCMI: attr %d err: %d, %s\n",
					PERFMON_COUNTER_REMOVE, ret, "remove and try again.");
				return -EINVAL;
			}

		} else {
			while (payload_size > 0) {
				size = payload_size > SCMI_PAYLOAD_SIZE ?
					SCMI_PAYLOAD_SIZE : payload_size;

				ret = send_set_scmi(slc_priv, PERFMON_COUNTER_REMOVE,
					(void *)slc_priv->curr_cntr_payload_ptr, size);
				if (ret) {
					dev_err(dev, "Send SCMI: attr %d err: %d, %s\n",
						PERFMON_COUNTER_REMOVE, ret,
						"remove and try again.");
					return -EINVAL;
				}

				payload_size -= size;
				slc_priv->curr_cntr_payload_ptr += size;
			}
		}
	}

	/* Sending Filter Payload if filter are configured */
	if (slc_priv->fltr_attr.is_filter_configured) {
		payload_size = slc_priv->fltr_attr.fltr_cfg_payload_size;
		slc_priv->curr_fltr_payload_ptr = slc_priv->filter_config_payload;

		do {
			size = payload_size > SCMI_PAYLOAD_SIZE ? SCMI_PAYLOAD_SIZE : payload_size;

			ret = send_set_scmi(slc_priv, PERFMON_FILTER_REMOVE,
				(void *)slc_priv->curr_fltr_payload_ptr, size);
			if (ret) {
				dev_err(dev, "Send SCMI: attr %d err: %d, %s\n",
					PERFMON_FILTER_REMOVE, ret, "remove and try again.");
				return -EINVAL;
			}

			payload_size -= size;
			slc_priv->curr_fltr_payload_ptr += size;
		} while (payload_size > 0);
	}

	cntr_payload_cleanup(slc_priv);
	fltr_payload_cleanup(slc_priv);
	return ret;
}

/* PERFMON configuration remove interface */
static ssize_t perfmon_remove_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	char *token, *delim = DELIM_CHAR;
	bool remove = false;
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct slc_perfmon_private *slc_priv = platform_get_drvdata(pdev);

	mutex_lock(&slc_priv->mutex);

	if (slc_priv->cntr_attr.is_perfmon_running) {
		dev_err(dev, "PERF_RM_ERR: PERFMON is running, can't remove the configuration.\n");
		goto remove_err;
	}

	token = strsep((char **)&buf, delim);
	if (token && sysfs_streq(token, "REMOVE")) {
		remove = true;
	} else {
		dev_err(dev, "PERF_RM_ERR: Invalid configuration, try again.\n");
		goto remove_err;
	}

	if (!(slc_priv->fltr_attr.is_filter_configured || slc_priv->cntr_attr.configured_cntrs)) {
		dev_err(dev, "PERF_RM_ERR: PERFMON is not configured.\n");
		goto remove_err;
	}

	if (perf_remove(dev, slc_priv)) {
		dev_err(dev, "PERF_RM_ERR: PERFMON Config remove failed.\n");
		goto remove_err;
	}

	dev_info(dev, "PERFMON is removed.\n");
	slc_priv->cntr_attr.is_perfmon_running = false;
	mutex_unlock(&slc_priv->mutex);
	return count;

remove_err:
	mutex_unlock(&slc_priv->mutex);
	return -EINVAL;
}

/* Timer configuration interface */
static ssize_t perfmon_timer_configure_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	int ret = 0;
	struct timer_config tm_cfg;
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct slc_perfmon_private *slc_priv = platform_get_drvdata(pdev);

	mutex_lock(&slc_priv->mutex);
	if (!slc_priv->cntr_attr.is_perfmon_configured) {
		dev_err(dev, "PERF_TMR_ERR: PERFMON is not configured.\n");
		goto timer_err;
	}

	if (slc_priv->cntr_attr.is_perfmon_running) {
		dev_err(dev, "PERF_TMR_ERR: Can't configure timer while PERFMON is running.\n");
		goto timer_err;
	}

	if (kstrtou32(buf, 0, &slc_priv->timer)) {
		dev_err(dev, "PERF_TMR_ERR: Invalid timer configuration.\n");
		goto timer_err;
	}

	if (slc_priv->timer == 0) {
		dev_err(dev, "PERF_TMR_ERR: Timer is not configured.\n");
		goto timer_err;
	}

	tm_cfg.timer = slc_priv->timer;

	/* Sending SCMI Payload */
	ret = send_set_scmi(slc_priv, PERFMON_TIMER_CONFIG, (void *)&tm_cfg,
		sizeof(struct timer_config));
	if (ret) {
		dev_err(dev, "Send SCMI: attr %d err: %d, %s\n",
			PERFMON_TIMER_CONFIG, ret, "remove and try again.");
		goto timer_err;
	} else {
		dev_info(dev, "Timer configured: %dms.\n", slc_priv->timer);
	}

	mutex_unlock(&slc_priv->mutex);
	return count;

timer_err:
	mutex_unlock(&slc_priv->mutex);
	return -EINVAL;
}

/* PERFMON Start/Stop interface */
static ssize_t perfmon_start_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	u8 start;
	int ret;
	struct perfmon_start_config start_cfg;
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct slc_perfmon_private *slc_priv = platform_get_drvdata(pdev);

	mutex_lock(&slc_priv->mutex);

	if (!slc_priv->cntr_attr.is_perfmon_configured) {
		dev_err(dev, "PERF_STRT_ERR: PERFMON is not configured.\n");
		goto start_err;
	}

	if (slc_priv->timer == 0) {
		dev_err(dev, "PERF_STRT_ERR: Timer is not configured.\n");
		goto start_err;
	}

	if (kstrtou8(buf, 0, &start)) {
		dev_err(dev, "PERF_STRT_ERR: Invalid start configuration, try again.\n");
		goto start_err;
	}

	if (!(start == 0 || start == 1)) {
		dev_err(dev, "PERF_STRT_ERR: Incorrect start configuration, try again.\n");
		goto start_err;
	}

	if (start) {
		if (slc_priv->clock && !slc_priv->clock_enabled) {
			ret = clk_prepare_enable(slc_priv->clock);
			if (ret) {
				dev_err(dev, "PERF_STRT_ERR: clock not enabled.\n");
				goto start_err;
			}
			slc_priv->clock_enabled = true;
		}
		dev_info(dev, "%u Counters are configured, PERFMON started.\n",
			slc_priv->cntr_attr.configured_cntrs);
	}

	/* Sending SCMI Payload */
	start_cfg.start = start;
	ret = send_set_scmi(slc_priv, PERFMON_START, (void *)&start_cfg,
		sizeof(struct perfmon_start_config));
	if (ret) {
		dev_err(dev, "Send SCMI: attr %d err: %d, %s\n",
			PERFMON_START, ret, "remove and try again.");
		goto start_err;
	}

	slc_priv->cntr_attr.is_perfmon_running = start;

	if (!start) {
		if (slc_priv->clock && slc_priv->clock_enabled) {
			clk_disable_unprepare(slc_priv->clock);
			slc_priv->clock_enabled = false;
		}
	}

	mutex_unlock(&slc_priv->mutex);
	return count;

start_err:
	mutex_unlock(&slc_priv->mutex);
	return -EINVAL;
}

/* PERFMON Counter dump interface */
static ssize_t perfmon_counter_dump_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	ssize_t cnt = 0;
	int retry_cnt = 0, match_seq_cnt = 0;
	u32 port, event, match_seq, num_cntr;
	u64 current_time, dump;
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct slc_perfmon_private *slc_priv = platform_get_drvdata(pdev);

	/* Checking for the CYCLE COUNT */
	if (slc_priv->cntr_attr.configured_cntrs == 0)
		num_cntr = 0;
	else if (slc_priv->cntr_attr.configured_cntrs == slc_priv->info_attr->max_cntr)
		num_cntr = slc_priv->cntr_attr.configured_cntrs;
	else
		num_cntr = slc_priv->cntr_attr.configured_cntrs + 1;

	do {
		while (unlikely((match_seq = slc_priv->perfmon_base->match_seq) % 2) &&
			(retry_cnt++ < MAX_SHARED_MEM_RETRY_CNT))
			;

		if (retry_cnt >= MAX_SHARED_MEM_RETRY_CNT)
			return -EINVAL;

		if (current_time == slc_priv->perfmon_base->last_captured_time)
			continue;

		memcpy(slc_priv->dump, slc_priv->perfmon_base->dump, sizeof(u64) * num_cntr);
		current_time = slc_priv->perfmon_base->last_captured_time;

	} while (match_seq != slc_priv->perfmon_base->match_seq &&
		match_seq_cnt++ < MAX_SHARED_MATCH_SEQ_CNT);

	if (match_seq_cnt == MAX_SHARED_MATCH_SEQ_CNT)
		return -EINVAL;

	for (int i = 0; i < num_cntr; i++) {
		port = slc_priv->prt_evnt_map[i].port;
		event = slc_priv->prt_evnt_map[i].event;

		if (port == CLOCK_PORT) {
			cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "CYCLE COUNT,,");
			dump = slc_priv->dump[i] / slc_priv->info_attr->mc_ch.channels;
		} else {
			cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt,
				"PORT %02u,EVENT %03u,", port, event);
			dump = slc_priv->dump[i];
		}

		cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "0x%016llx\n", dump);
	}

	cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "LAST CAPTURED TIME,,");
	cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "0x%016llx\n", current_time);
	slc_priv->perfmon_base->last_captured_time = current_time;
	return cnt;
}

/* SCID capacity status */
static ssize_t perfmon_scid_status_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	u8 max_scid;
	u32 val, snap_reg_val;
	unsigned int i, j, offset;
	ssize_t cnt = 0;
	unsigned long total;
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct slc_perfmon_private *slc_priv = platform_get_drvdata(pdev);

	max_scid = slc_priv->perf_info_attr->filter_max_match[SCID];
	if (slc_priv->scid_status_trigger) {
		regmap_read(slc_priv->slc_bcast_map, TRP_CAP_COUNTERS_DUMP_CFG, &snap_reg_val);
		regmap_write(slc_priv->slc_bcast_map, TRP_CAP_COUNTERS_DUMP_CFG, 0);
		regmap_write(slc_priv->slc_bcast_map, TRP_CAP_COUNTERS_DUMP_CFG, 1);
		regmap_write(slc_priv->slc_bcast_map, TRP_CAP_COUNTERS_DUMP_CFG, 0);
	}

	for (i = 0; i < max_scid; i++) {
		total = 0;
		offset = TRP_SCID_n_STATUS(i);
		for (j = 0; j < slc_priv->info_attr->mc_ch.channels; j++) {
			regmap_read(slc_priv->slc_maps[j], offset, &val);
			val = (val & TRP_SCID_STATUS_CURRENT_CAP_MASK) >>
				TRP_SCID_STATUS_CURRENT_CAP_SHIFT;
			total += val;
		}

		regmap_read(slc_priv->slc_bcast_map, offset, &val);
		if (val & TRP_SCID_STATUS_ACTIVE_MASK)
			cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt,
					"SCID %02d %10s", i, "ACTIVE");
		else
			cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt,
					"SCID %02d %10s", i, "DE-ACTIVE");

		cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, ",0x%08lx\n", total);
	}

	if (slc_priv->scid_status_trigger)
		regmap_write(slc_priv->slc_bcast_map, TRP_CAP_COUNTERS_DUMP_CFG, snap_reg_val);

	return cnt;
}

/* SLC PERFMON Information */
static ssize_t perfmon_info_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	ssize_t cnt = 0;
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct slc_perfmon_private *slc_priv = platform_get_drvdata(pdev);
	struct slc_version version = slc_priv->info_attr->version;
	u8 port_fltr;

	cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "\nSLC Version: %u.%u.%u ",
			version.major, version.branch, version.minor);
	cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "No. of Memory Controller: %u ",
			slc_priv->info_attr->mc_ch.num_mc);
	cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "No. of Channels: %u.\n",
			slc_priv->info_attr->mc_ch.channels);
	cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "Max no. of counters supported: %u ",
			slc_priv->info_attr->max_cntr);
	cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "Max no. of filters supported: %u\n\n",
			slc_priv->info_attr->max_fltr_idx);
	cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "SLC port filter support:\n");
	cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "|--------+---------------------");
	cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "----------------------------|\n");
	cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "| PORT\t | FILTER SUPPORT      ");
	cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "                            |\n");
	cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "|--------+---------------------");
	cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "----------------------------|\n");

	for (int i = 0; i < slc_priv->num_ports; i++) {
		char fltr_types[FLTR_TYPE_LEN] = "";
		const char *port_name = (slc_priv->info_attr->version.major < VERSION_6) ?
					ports_v5[i] : ports_v6[i];

		port_fltr = slc_priv->perf_info_attr->filter_port_support[i];
		cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "| %s\t |", port_name);


		for (int j = 0; j < MAX_FILTER_TYPE; j++) {
			if (port_fltr & (1 << j))
				strlcat(fltr_types, filters[j], FLTR_TYPE_LEN);
		}

		strlcat(fltr_types, "\0", FLTR_TYPE_LEN);
		cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "%-48s |\n", fltr_types);
	}
	cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "|--------+---------------------");
	cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "----------------------------|\n");

	return cnt;
}

static DEVICE_ATTR_WO(perfmon_filter_configure);
static DEVICE_ATTR_WO(perfmon_counter_configure);
static DEVICE_ATTR_WO(perfmon_start);
static DEVICE_ATTR_WO(perfmon_timer_configure);
static DEVICE_ATTR_WO(perfmon_remove);
static DEVICE_ATTR_RO(perfmon_counter_dump);
static DEVICE_ATTR_RO(perfmon_scid_status);
static DEVICE_ATTR_RO(perfmon_info);

static struct attribute *slc_perfmon_attrs[] = {
	&dev_attr_perfmon_filter_configure.attr,
	&dev_attr_perfmon_counter_configure.attr,
	&dev_attr_perfmon_timer_configure.attr,
	&dev_attr_perfmon_start.attr,
	&dev_attr_perfmon_remove.attr,
	&dev_attr_perfmon_counter_dump.attr,
	&dev_attr_perfmon_scid_status.attr,
	&dev_attr_perfmon_info.attr,
	NULL,
};

static struct attribute_group slc_perfmon_group = {
	.attrs	= slc_perfmon_attrs,
};

/* Initialize regmap */
static struct regmap *qcom_slc_init_mmio(struct platform_device *pdev, const char *name)
{
	void __iomem *base;
	struct resource *res;

	const struct regmap_config slc_regmap_config = {
		.reg_bits = 32,
		.reg_stride = 4,
		.val_bits = 32,
		.fast_io = true,
		.name = name,
	};

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (IS_ERR_OR_NULL(res))
		return NULL;

	base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (IS_ERR_OR_NULL(base))
		return NULL;

	return devm_regmap_init_mmio(&pdev->dev, base, &slc_regmap_config);
}

static int slc_perfmon_probe(struct platform_device *pdev)
{
	struct slc_perfmon_private *slc_priv;
	struct scmi_device *sdev;
	struct clk *clock;
	struct slc_version version;
	u16 ports = 0;
	int ret, reg_count;

	/* Initializing SCMI Device */
	sdev = get_qcom_scmi_device();
	if (IS_ERR(sdev)) {
		ret = PTR_ERR(sdev);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Error getting scmi_dev ret=%d.\n", ret);
		return ret;
	}

	/* Initializing QDSS clock */
	clock = devm_clk_get(&pdev->dev, "qdss_clk");
	if (IS_ERR_OR_NULL(clock)) {
		dev_warn(&pdev->dev, "failed to get qdss clock node.\n");
		clock = NULL;
	}

	/* private struct initialization */
	slc_priv = devm_kzalloc(&pdev->dev, sizeof(struct slc_perfmon_private), GFP_KERNEL);
	if (IS_ERR_OR_NULL(slc_priv))
		return -ENOMEM;

	/* Mutex initialization */
	mutex_init(&slc_priv->mutex);

	slc_priv->sdev = sdev;
	slc_priv->clock = clock;

	/* Getting SCMI protocol handle and getting SCMI Ops */
	slc_priv->ops = slc_priv->sdev->handle->devm_protocol_get(slc_priv->sdev,
			QCOM_SCMI_VENDOR_PROTOCOL, &slc_priv->ph);
	if (IS_ERR(slc_priv->ops)) {
		ret = PTR_ERR(slc_priv->ops);
		slc_priv->ops = NULL;
		dev_err(&pdev->dev, "Error getting vendor protocol ops: %d.\n", ret);
		return ret;
	}

	/* Initializing shared memory using perfmon offset property */
	ret = of_property_read_u32(pdev->dev.of_node, "perfmon_offset", &slc_priv->perfmon_offset);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get perfmon offset.\n");
		return -ENODEV;
	}

	slc_priv->res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "perfmon_base");
	if (IS_ERR_OR_NULL(slc_priv->res)) {
		dev_err(&pdev->dev, "Failed to get memory resource.\n");
		return -ENODEV;
	}

	slc_priv->perfmon_base = (struct qcom_slc_perfmon_mem *)(devm_ioremap(&pdev->dev,
		slc_priv->res->start, resource_size(slc_priv->res)) + slc_priv->perfmon_offset);
	if (IS_ERR_OR_NULL(slc_priv->perfmon_base)) {
		dev_err(&pdev->dev, "Error ioremap perfmon_base.\n");
		return -ENOMEM;
	}

	/* Initializing info attribute */
	slc_priv->info_attr = devm_kzalloc(&pdev->dev, sizeof(struct slc_info_attr),
						GFP_KERNEL);
	if (IS_ERR_OR_NULL(slc_priv->info_attr))
		return -ENOMEM;

	/* PERFMON common info should be fetched using SCMI */
	ret = receive_get_scmi(slc_priv, PERFMON_SLC_COMMON_INFO);
	if (ret) {
		dev_err(&pdev->dev, "Receive SCMI: attr %d err: %d, %s\n",
			PERFMON_SLC_COMMON_INFO, ret, "remove and try again.");
		return -EINVAL;
	}

	/* Initializing perfmon info attribute */
	ports = slc_priv->info_attr->port_registered;
	while (ports) {
		ports &= (ports - 1);
		slc_priv->num_ports++;
	}

	slc_priv->perf_info_attr = devm_kzalloc(&pdev->dev,
			sizeof(struct slc_perfmon_info_attr) + (slc_priv->num_ports * sizeof(u8)),
			GFP_KERNEL);
	if (IS_ERR_OR_NULL(slc_priv->perf_info_attr))
		return -ENOMEM;

	ret = receive_get_scmi(slc_priv, PERFMON_CONFIG_INFO);
	if (ret) {
		dev_err(&pdev->dev, "Receive SCMI: attr %d err: %d, %s\n",
			PERFMON_CONFIG_INFO, ret, "remove and try again.");
		return -EINVAL;
	}

	reg_count = of_property_count_elems_of_size(pdev->dev.of_node, "reg", sizeof(u32));
	/* regmaps should include shm base, broadcast or base and 4 channel bases */
	if (reg_count < slc_priv->info_attr->mc_ch.channels + 2) {
		dev_err(&pdev->dev, "Insufficient no. of regmaps\n");
		return -EINVAL;
	}

	/* Initialize all TRP channels regmap */
	slc_priv->slc_maps = devm_kcalloc(&pdev->dev, slc_priv->info_attr->mc_ch.channels,
					sizeof(*slc_priv->slc_maps), GFP_KERNEL);
	if (IS_ERR_OR_NULL(slc_priv->slc_maps)) {
		dev_err(&pdev->dev, "Error: trp regmap allocation failedn %ld\n",
			PTR_ERR(slc_priv->slc_maps));
		return -ENOMEM;
	}

	for (int i = 0; i < slc_priv->info_attr->mc_ch.channels; i++) {
		char *base __free(kfree) = kasprintf(GFP_KERNEL, "trp_%d_base", i);
		slc_priv->slc_maps[i] = qcom_slc_init_mmio(pdev, base);
	}

	/* Initialize TRP broadcast or regmap */
	slc_priv->slc_bcast_map = qcom_slc_init_mmio(pdev, "trp_or_bcast_base");
	if (IS_ERR_OR_NULL(slc_priv->slc_bcast_map)) {
		dev_err(&pdev->dev, "Error: bcast trp regmap allocation failed, %ld\n",
			PTR_ERR(slc_priv->slc_bcast_map));
		return -ENOMEM;
	}

	/* Set scid_status_trigger flag based on DT entry for supported platforms */
	slc_priv->scid_status_trigger = of_property_read_bool(pdev->dev.of_node,
							"slc-scid-status-snapshot");

	/* Initializing payload capacities */
	slc_priv->filter_payload_capacity = (slc_priv->info_attr->max_fltr_idx *
					MAX_FILTER_TYPE) * sizeof(struct filter_config);
	slc_priv->counter_payload_capacity = (slc_priv->info_attr->max_cntr)
					* sizeof(struct counter_config);

	/* Initializing perfmon attributes based on SCMI result */
	slc_priv->fltr_port = devm_kzalloc(&pdev->dev, (sizeof(struct filtered_port) *
				slc_priv->info_attr->max_fltr_idx), GFP_KERNEL);
	slc_priv->prt_evnt_map = devm_kzalloc(&pdev->dev, (sizeof(struct port_event_config) *
				slc_priv->info_attr->max_cntr), GFP_KERNEL);
	slc_priv->dump = devm_kzalloc(&pdev->dev, (sizeof(u64) *
				slc_priv->info_attr->max_cntr), GFP_KERNEL);

	/* Initializing Perfmon filter payload */
	slc_priv->filter_config_payload = devm_kzalloc(&pdev->dev,
						slc_priv->filter_payload_capacity, GFP_KERNEL);

	/* Initializing Perfmon counter payload */
	slc_priv->counter_config_payload = devm_kzalloc(&pdev->dev,
						slc_priv->counter_payload_capacity, GFP_KERNEL);

	if (!slc_priv->fltr_port || !slc_priv->prt_evnt_map || !slc_priv->filter_config_payload ||
		!slc_priv->counter_config_payload)
		return -ENOMEM;

	slc_priv->curr_fltr_payload_ptr = slc_priv->filter_config_payload;
	slc_priv->curr_cntr_payload_ptr = slc_priv->counter_config_payload;

	/* SLC Version */
	version = slc_priv->info_attr->version;

	/* Creating SYSFS nodes group */
	if (sysfs_create_group(&pdev->dev.kobj, &slc_perfmon_group)) {
		dev_err(&pdev->dev, "Unable to create sysfs group.\n");
		return -EINVAL;
	}

	platform_set_drvdata(pdev, slc_priv);

	dev_dbg(&pdev->dev, "SLC Version: %u.%u.%u.\n", version.major, version.branch,
		version.minor);
	dev_dbg(&pdev->dev, "No. of Memory Controller: %u.\n", slc_priv->info_attr->mc_ch.num_mc);
	dev_dbg(&pdev->dev, "Max. Counters: %u.\n", slc_priv->info_attr->max_cntr);
	dev_dbg(&pdev->dev, "Max. Filters: %u.\n", slc_priv->info_attr->max_fltr_idx);
	dev_info(&pdev->dev, "Module inserted successfully.\n");
	return 0;
}

static void slc_perfmon_remove(struct platform_device *pdev)
{
	struct slc_perfmon_private *slc_priv = platform_get_drvdata(pdev);

	mutex_lock(&slc_priv->mutex);
	/* Checking if PERFMON is configured, if so then remove the configurations
	 * and then module is removed.
	 */
	if (slc_priv->fltr_attr.is_filter_configured || slc_priv->cntr_attr.configured_cntrs) {
		dev_dbg(&pdev->dev, "PERFMON is still configured, removing configuration.\n");
		if (perf_remove(&pdev->dev, slc_priv)) {
			dev_err(&pdev->dev, "Can't remove config, module can't be removed.\n");
			mutex_unlock(&slc_priv->mutex);
			return;
		}
	}

	mutex_unlock(&slc_priv->mutex);

	/* clean up */
	mutex_destroy(&slc_priv->mutex);
	sysfs_remove_group(&pdev->dev.kobj, &slc_perfmon_group);
	platform_set_drvdata(pdev, NULL);

	dev_info(&pdev->dev, "Module removed successfully.\n");
}

static const struct of_device_id of_match_slc_perfmon[] = {
	{
		.compatible = "qcom,slc-perfmon",
	},
	{},
};
MODULE_DEVICE_TABLE(of, of_match_slc_perfmon);

static struct platform_driver slc_perfmon_driver = {
	.driver	= {
		.name = "qcom-slc-perfmon",
		.of_match_table = of_match_slc_perfmon,
	},
	.probe = slc_perfmon_probe,
	.remove	= slc_perfmon_remove,
};
module_platform_driver(slc_perfmon_driver);

MODULE_DESCRIPTION("QCOM SLC PERFMON");
MODULE_LICENSE("GPL");
