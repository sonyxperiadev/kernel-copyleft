// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2016-2019, STMicroelectronics Limited.
 * Authors: AMG(Analog Mems Group) <marco.cali@st.com>
 *
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/*
  *
  **************************************************************************
  **                        STMicroelectronics				**
  **************************************************************************
  **                        marco.cali@st.com				 **
  **************************************************************************
  *                                                                        *
  *                  FTS functions for getting frames			  *
  *                                                                        *
  **************************************************************************
  **************************************************************************
  *
  */

/*!
  * \file ftsFrame.c
  * \brief Contains all the functions to work with frames
  */

#include "ftsCompensation.h"
#include "ftsCore.h"
#include "ftsError.h"
#include "ftsFrame.h"
#include "ftsHardware.h"
#include "ftsIO.h"
#include "ftsSoftware.h"
#include "ftsTool.h"
#include "ftsTime.h"


#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/version.h>
#if (KERNEL_VERSION(5, 15, 0) <= LINUX_VERSION_CODE)
#include <linux/stdarg.h>
#else
#include <stdarg.h>
#endif
#include <linux/serio.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/ctype.h>



/**
  * Read the channels lengths from the config memory
  * @return OK if success or an error code which specify the type of error
  */
int getChannelsLength(struct fts_ts_info *info)
{
	int ret;
	u8 data[2];

	ret = readConfig(info, ADDR_CONFIG_SENSE_LEN, data, 2);
	if (ret < OK) {
		logError(1, "%s getChannelsLength: ERROR %08X\n", tag, ret);

		return ret;
	}

	info->systemInfo.u8_scrRxLen = (int)data[0];
	info->systemInfo.u8_scrTxLen = (int)data[1];

	logError(0, "%s Force_len = %d   Sense_Len = %d\n", tag,
		 info->systemInfo.u8_scrTxLen, info->systemInfo.u8_scrRxLen);

	return OK;
}



/**
  * Read and pack the frame data related to the nodes
  * @param address address in memory when the frame data node start
  * @param size amount of data to read
  * @param frame pointer to an array of bytes which will contain the frame node
  *data
  * @return OK if success or an error code which specify the type of error
  */
int getFrameData(struct fts_ts_info *info, u16 address, int size, short *frame)
{
	int i, j, ret;
	u8 *data = (u8 *)kmalloc(size * sizeof(u8), GFP_KERNEL);

	if (data == NULL) {
		logError(1, "%s getFrameData: ERROR %08X\n", tag, ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	ret = fts_writeReadU8UX(info, FTS_CMD_FRAMEBUFFER_R, BITS_16, address, data,
				size, DUMMY_FRAMEBUFFER);
	if (ret < OK) {
		logError(1, "%s getFrameData: ERROR %08X\n", tag, ERROR_BUS_R);
		kfree(data);
		return ERROR_BUS_R;
	}
	j = 0;
	for (i = 0; i < size; i += 2) {
		frame[j] = (short)((data[i + 1] << 8) + data[i]);
		j++;
	}
	kfree(data);
	return OK;
}


/**
  * Return the number of Sense Channels (Rx)
  * @return number of Rx channels
  */
int getSenseLen(struct fts_ts_info *info)
{
	if (info->systemInfo.u8_scrRxLen == 0)
		getChannelsLength(info);
	return info->systemInfo.u8_scrRxLen;
}

/**
  * Return the number of Force Channels (Tx)
  * @return number of Tx channels
  */
int getForceLen(struct fts_ts_info *info)
{
	if (info->systemInfo.u8_scrTxLen == 0)
		getChannelsLength(info);
	return info->systemInfo.u8_scrTxLen;
}


/********************    New API     **************************/

/**
  * Read a MS Frame from frame buffer memory
  * @param type type of MS frame to read
  * @param frame pointer to MutualSenseFrame variable which will contain the
  * data
  * @return > 0 if success specifying the number of node into the frame or
  * an error code which specify the type of error
  */
int getMSFrame3(struct fts_ts_info *info, MSFrameType type, MutualSenseFrame *frame)
{
	u16 offset;
	int ret, force_len, sense_len;

	force_len = getForceLen(info);
	sense_len = getSenseLen(info);

	frame->node_data = NULL;

	logError(0, "%s %s: Starting to get frame %02X\n", tag, __func__,
		 type);
	switch (type) {
	case MS_RAW:
		offset = info->systemInfo.u16_msTchRawAddr;
		goto LOAD_NORM;
	case MS_FILTER:
		offset = info->systemInfo.u16_msTchFilterAddr;

		goto LOAD_NORM;
	case MS_STRENGTH:
		offset = info->systemInfo.u16_msTchStrenAddr;
		goto LOAD_NORM;
	case MS_BASELINE:
		offset = info->systemInfo.u16_msTchBaselineAddr;
LOAD_NORM:
		if (force_len == 0 || sense_len == 0) {
			logError(1,
				 "%s %s: number of channels not initialized ERROR %08X\n",
				 tag, __func__, ERROR_CH_LEN);
			return ERROR_CH_LEN | ERROR_GET_FRAME;
		}

		break;

	case MS_KEY_RAW:
		offset = info->systemInfo.u16_keyRawAddr;
		goto LOAD_KEY;
	case MS_KEY_FILTER:
		offset = info->systemInfo.u16_keyFilterAddr;
		goto LOAD_KEY;
	case MS_KEY_STRENGTH:
		offset = info->systemInfo.u16_keyStrenAddr;
		goto LOAD_KEY;
	case MS_KEY_BASELINE:
		offset = info->systemInfo.u16_keyBaselineAddr;
LOAD_KEY:
		if (info->systemInfo.u8_keyLen == 0) {
			logError(1,
				 "%s %s: number of channels not initialized ERROR %08X\n",
				 tag, __func__, ERROR_CH_LEN);
			return ERROR_CH_LEN | ERROR_GET_FRAME;
		}
		force_len = 1;
		sense_len = info->systemInfo.u8_keyLen;
		break;

	case FRC_RAW:
		offset = info->systemInfo.u16_frcRawAddr;
		goto LOAD_FRC;
	case FRC_FILTER:
		offset = info->systemInfo.u16_frcFilterAddr;
		goto LOAD_FRC;
	case FRC_STRENGTH:
		offset = info->systemInfo.u16_frcStrenAddr;
		goto LOAD_FRC;
	case FRC_BASELINE:
		offset = info->systemInfo.u16_frcBaselineAddr;
LOAD_FRC:
		if (force_len == 0) {
			logError(1,
				 "%s %s: number of channels not initialized ERROR %08X\n",
				 tag, __func__, ERROR_CH_LEN);
			return ERROR_CH_LEN | ERROR_GET_FRAME;
		}
		sense_len = 1;
		break;
	default:
		logError(1, "%s %s: Invalid type ERROR %08X\n", tag, __func__,
			 ERROR_OP_NOT_ALLOW | ERROR_GET_FRAME);
		return ERROR_OP_NOT_ALLOW | ERROR_GET_FRAME;
	}

	frame->node_data_size = ((force_len) * sense_len);
	frame->header.force_node = force_len;
	frame->header.sense_node = sense_len;
	frame->header.type = type;

	logError(0, "%s %s: Force_len = %d Sense_len = %d Offset = %04X\n",
		 tag, __func__, force_len, sense_len, offset);

	frame->node_data = (short *)kmalloc(frame->node_data_size *
					    sizeof(short), GFP_KERNEL);
	if (frame->node_data == NULL) {
		logError(1, "%s %s: ERROR %08X\n", tag, __func__, ERROR_ALLOC |
			 ERROR_GET_FRAME);
		return ERROR_ALLOC | ERROR_GET_FRAME;
	}

	ret = getFrameData(info, offset, frame->node_data_size * BYTES_PER_NODE,
			   (frame->node_data));
	if (ret < OK) {
		logError(1, "%s %s: ERROR %08X\n", tag, __func__,
			 ERROR_GET_FRAME_DATA);
		kfree(frame->node_data);
		frame->node_data = NULL;
		return ret | ERROR_GET_FRAME_DATA | ERROR_GET_FRAME;
	}
	/* if you want to access one node i,j,
	  * compute the offset like: offset = i*columns + j = > frame[i, j] */

	logError(0, "%s Frame acquired!\n", tag);
	return frame->node_data_size;
	/* return the number of data put inside frame */
}

/**
  * Read a SS Frame from frame buffer
  * @param type type of SS frame to read
  * @param frame pointer to SelfSenseFrame variable which will contain the data
  * @return > 0 if success specifying the number of node into frame or an
  *  error code which specify the type of error
  */
int getSSFrame3(struct fts_ts_info *info, SSFrameType type, SelfSenseFrame *frame)
{
	u16 offset_force, offset_sense;
	int ret;

	frame->force_data = NULL;
	frame->sense_data = NULL;

	frame->header.force_node = getForceLen(info);	/* use getForce/SenseLen
							 * because introduce a
							 * recover mechanism in
							 * case of len =0 */
	frame->header.sense_node = getSenseLen(info);

	if (frame->header.force_node == 0 || frame->header.sense_node == 0) {
		logError(1,
			 "%s %s: number of channels not initialized ERROR %08X\n",
			 tag,
			 __func__, ERROR_CH_LEN);
		return ERROR_CH_LEN | ERROR_GET_FRAME;
	}


	logError(0, "%s %s: Starting to get frame %02X\n", tag, __func__,
		 type);
	switch (type) {
	case SS_RAW:
		offset_force = info->systemInfo.u16_ssTchTxRawAddr;
		offset_sense = info->systemInfo.u16_ssTchRxRawAddr;
		break;
	case SS_FILTER:
		offset_force = info->systemInfo.u16_ssTchTxFilterAddr;
		offset_sense = info->systemInfo.u16_ssTchRxFilterAddr;
		break;
	case SS_STRENGTH:
		offset_force = info->systemInfo.u16_ssTchTxStrenAddr;
		offset_sense = info->systemInfo.u16_ssTchRxStrenAddr;
		break;
	case SS_BASELINE:
		offset_force = info->systemInfo.u16_ssTchTxBaselineAddr;
		offset_sense = info->systemInfo.u16_ssTchRxBaselineAddr;
		break;

	case SS_HVR_RAW:
		offset_force = info->systemInfo.u16_ssHvrTxRawAddr;
		offset_sense = info->systemInfo.u16_ssHvrRxRawAddr;
		break;
	case SS_HVR_FILTER:
		offset_force = info->systemInfo.u16_ssHvrTxFilterAddr;
		offset_sense = info->systemInfo.u16_ssHvrRxFilterAddr;
		break;
	case SS_HVR_STRENGTH:
		offset_force = info->systemInfo.u16_ssHvrTxStrenAddr;
		offset_sense = info->systemInfo.u16_ssHvrRxStrenAddr;
		break;
	case SS_HVR_BASELINE:
		offset_force = info->systemInfo.u16_ssHvrTxBaselineAddr;
		offset_sense = info->systemInfo.u16_ssHvrRxBaselineAddr;
		break;

	case SS_PRX_RAW:
		offset_force = info->systemInfo.u16_ssPrxTxRawAddr;
		offset_sense = info->systemInfo.u16_ssPrxRxRawAddr;
		break;
	case SS_PRX_FILTER:
		offset_force = info->systemInfo.u16_ssPrxTxFilterAddr;
		offset_sense = info->systemInfo.u16_ssPrxRxFilterAddr;
		break;
	case SS_PRX_STRENGTH:
		offset_force = info->systemInfo.u16_ssPrxTxStrenAddr;
		offset_sense = info->systemInfo.u16_ssPrxRxStrenAddr;
		break;
	case SS_PRX_BASELINE:
		offset_force = info->systemInfo.u16_ssPrxTxBaselineAddr;
		offset_sense = info->systemInfo.u16_ssPrxRxBaselineAddr;
		break;
	case SS_DETECT_RAW:
		if (info->systemInfo.u8_ssDetScanSet == 0) {
			offset_force = info->systemInfo.u16_ssDetRawAddr;
			offset_sense = 0;
			frame->header.sense_node = 0;
		} else {
			offset_sense = info->systemInfo.u16_ssDetRawAddr;
			offset_force = 0;
			frame->header.force_node = 0;
		}
		break;

	case SS_DETECT_FILTER:
		if (info->systemInfo.u8_ssDetScanSet == 0) {
			offset_force = info->systemInfo.u16_ssDetFilterAddr;
			offset_sense = 0;
			frame->header.sense_node = 0;
		} else {
			offset_sense = info->systemInfo.u16_ssDetFilterAddr;
			offset_force = 0;
			frame->header.force_node = 0;
		}
		break;

	case SS_DETECT_BASELINE:
		if (info->systemInfo.u8_ssDetScanSet == 0) {
			offset_force = info->systemInfo.u16_ssDetBaselineAddr;
			offset_sense = 0;
			frame->header.sense_node = 0;
		} else {
			offset_sense = info->systemInfo.u16_ssDetBaselineAddr;
			offset_force = 0;
			frame->header.force_node = 0;
		}
		break;

	case SS_DETECT_STRENGTH:
		if (info->systemInfo.u8_ssDetScanSet == 0) {
			offset_force = info->systemInfo.u16_ssDetStrenAddr;
			offset_sense = 0;
			frame->header.sense_node = 0;
		} else {
			offset_sense = info->systemInfo.u16_ssDetStrenAddr;
			offset_force = 0;
			frame->header.force_node = 0;
		}
		break;

	default:
		logError(1, "%s %s: Invalid type ERROR %08X\n", tag, __func__,
			 ERROR_OP_NOT_ALLOW | ERROR_GET_FRAME);
		return ERROR_OP_NOT_ALLOW | ERROR_GET_FRAME;
	}

	frame->header.type = type;

	logError(0,
		 "%s %s: Force_len = %d Sense_len = %d Offset_force = %04X Offset_sense = %04X\n",
		 tag, __func__, frame->header.force_node,
		 frame->header.sense_node,
		 offset_force, offset_sense);

	frame->force_data = (short *)kmalloc(frame->header.force_node *
					     sizeof(short), GFP_KERNEL);
	if (frame->force_data == NULL) {
		logError(1, "%s %s: can not allocate force_data ERROR %08X\n",
			 tag, __func__, ERROR_ALLOC | ERROR_GET_FRAME);
		return ERROR_ALLOC | ERROR_GET_FRAME;
	}

	frame->sense_data = (short *)kmalloc(frame->header.sense_node *
					     sizeof(short), GFP_KERNEL);
	if (frame->sense_data == NULL) {
		kfree(frame->force_data);
		frame->force_data = NULL;
		logError(1, "%s %s: can not allocate sense_data ERROR %08X\n",
			 tag, __func__, ERROR_ALLOC | ERROR_GET_FRAME);
		return ERROR_ALLOC | ERROR_GET_FRAME;
	}

	ret = getFrameData(info, offset_force, frame->header.force_node *
			   BYTES_PER_NODE, (frame->force_data));
	if (ret < OK) {
		logError(1,
			 "%s %s: error while reading force data ERROR %08X\n",
			 tag,
			 __func__, ERROR_GET_FRAME_DATA);
		kfree(frame->force_data);
		frame->force_data = NULL;
		kfree(frame->sense_data);
		frame->sense_data = NULL;
		return ret | ERROR_GET_FRAME_DATA | ERROR_GET_FRAME;
	}

	ret = getFrameData(info, offset_sense, frame->header.sense_node *
			   BYTES_PER_NODE, (frame->sense_data));
	if (ret < OK) {
		logError(1,
			 "%s %s: error while reading sense data ERROR %08X\n",
			 tag,
			 __func__, ERROR_GET_FRAME_DATA);
		kfree(frame->force_data);
		frame->force_data = NULL;
		kfree(frame->sense_data);
		frame->sense_data = NULL;
		return ret | ERROR_GET_FRAME_DATA | ERROR_GET_FRAME;
	}
	/* if you want to access one node i,j, you should compute the offset
	 * like: offset = i*columns + j = > frame[i, j] */

	logError(0, "%s Frame acquired!\n", tag);
	return frame->header.force_node + frame->header.sense_node;
	/* return the number of data put inside frame */
}


/**
  * Read Initialization Data Header and check that the type loaded match with
  * the one previously requested
  * @param type type of Initialization data requested @link load_opt Load Host
  * Data Option @endlink
  * @param msHeader pointer to DataHeader variable which will contain the header
  * info for the MS frame
  * @param ssHeader pointer to DataHeader variable which will contain the header
  * info for the SS frame
  * @param address pointer to a variable which will contain the updated address
  * to the next data
  * @return OK if success or an error code which specify the type of error
  */
int readSyncDataHeader(struct fts_ts_info *info, u8 type, DataHeader *msHeader,
	DataHeader *ssHeader, u64 *address)
{
	u64 offset = ADDR_FRAMEBUFFER;
	u8 data[SYNCFRAME_DATA_HEADER];
	int ret;

	ret = fts_writeReadU8UX(info, FTS_CMD_FRAMEBUFFER_R, BITS_16, offset, data,
				SYNCFRAME_DATA_HEADER, DUMMY_FRAMEBUFFER);
	if (ret < OK) {	/* i2c function have already a retry mechanism */
		logError(1,
			 "%s %s: error while reading data header ERROR %08X\n",
			 tag,
			 __func__, ret);
		return ret;
	}

	logError(0, "%s Read Data Header done!\n", tag);

	if (data[0] != HEADER_SIGNATURE) {
		logError(1,
			 "%s %s: The Header Signature was wrong! %02X != %02X ERROR %08X\n",
			 tag, __func__, data[0], HEADER_SIGNATURE,
			 ERROR_WRONG_DATA_SIGN);
		return ERROR_WRONG_DATA_SIGN;
	}


	if (data[1] != type) {
		logError(1, "%s %s: Wrong type found! %02X!=%02X ERROR %08X\n",
			 tag, __func__, data[1], type, ERROR_DIFF_DATA_TYPE);
		return ERROR_DIFF_DATA_TYPE;
	}

	logError(0, "%s Type = %02X of SyncFrame data OK!\n", tag, type);

	msHeader->force_node = data[5];
	msHeader->sense_node = data[6];
	logError(0, "%s MS Frame force_node = %d, sense_node = %d\n", tag,
		 msHeader->force_node, msHeader->sense_node);

	ssHeader->force_node = data[7];
	ssHeader->sense_node = data[8];
	logError(0, "%s SS Frame force_node = %d, sense_node = %d\n", tag,
		 ssHeader->force_node, ssHeader->sense_node);

	*address = offset + SYNCFRAME_DATA_HEADER + data[4];

	return OK;
}

/**
  * Read a Sync Frame from frame buffer which contain MS and SS data collected
  *for the same scan
  * @param type type of Sync frame to read, possible values:
  *  LOAD_SYNC_FRAME_RAW, LOAD_SYNC_FRAME_FILTER, LOAD_SYNC_FRAME_BASELINE,
  * LOAD_SYNC_FRAME_STRENGTH
  * @param msFrame pointer to MutualSenseFrame variable which will contain the
  *MS data
  * @param ssFrame pointer to SelfSenseFrame variable which will contain the SS
  *data
  * @return >0 if success specifying the total number of nodes copied into
  * msFrame and ssFrame or an error code which specify the type of error
  */
int getSyncFrame(struct fts_ts_info *info, u8 type, MutualSenseFrame *msFrame,
	SelfSenseFrame *ssFrame)
{
	int res;
	u64 address;

	msFrame->node_data = NULL;
	ssFrame->force_data = NULL;
	ssFrame->sense_data = NULL;

	logError(0, "%s %s: Starting to get Sync Frame %02X...\n", tag,
		 __func__, type);
	switch (type) {
	case LOAD_SYNC_FRAME_RAW:
		msFrame->header.type = MS_RAW;
		ssFrame->header.type = SS_RAW;
		break;

	case LOAD_SYNC_FRAME_FILTER:
		msFrame->header.type = MS_FILTER;
		ssFrame->header.type = SS_FILTER;
		break;

	case LOAD_SYNC_FRAME_BASELINE:
		msFrame->header.type = MS_BASELINE;
		ssFrame->header.type = SS_BASELINE;
		break;

	case LOAD_SYNC_FRAME_STRENGTH:
		msFrame->header.type = MS_STRENGTH;
		ssFrame->header.type = SS_STRENGTH;
		break;

	default:
		return ERROR_OP_NOT_ALLOW | ERROR_GET_FRAME;
	}

	logError(0, "%s %s: Requesting Sync Frame %02X...\n", tag, __func__,
		 type);
	res = requestSyncFrame(info, type);
	if (res < OK) {
		logError(1,
			 "%s %s: error while requesting Sync Frame ERROR %08X\n",
			 tag,
			 __func__, res | ERROR_GET_FRAME_DATA);
		return res | ERROR_GET_FRAME_DATA;
	}

	res = readSyncDataHeader(info, type, &(msFrame->header), &(ssFrame->header),
				 &address);
	if (res < OK) {
		logError(1,
			 "%s %s: error while reading Sync Frame header... ERROR %08X\n",
			 tag, __func__, res | ERROR_GET_FRAME_DATA);
		return res | ERROR_GET_FRAME_DATA;
	}

	msFrame->node_data_size = msFrame->header.force_node *
				  msFrame->header.sense_node;

	msFrame->node_data = (short *)kmalloc(msFrame->node_data_size *
					      sizeof(short), GFP_KERNEL);
	if (msFrame->node_data == NULL) {
		logError(1,
			 "%s %s: impossible allocate memory for MS frame... ERROR %08X\n",
			 tag, __func__, ERROR_ALLOC | ERROR_GET_FRAME);
		return ERROR_ALLOC | ERROR_GET_FRAME;
	}

	logError(0, "%s %s: Getting MS frame at %04llX...\n", tag, __func__,
		 address);
	res = getFrameData(info, address, (msFrame->node_data_size) * BYTES_PER_NODE,
			   (msFrame->node_data));
	if (res < OK) {
		logError(1, "%s %s: error while getting MS data...ERROR %08X\n",
			 tag, __func__, res);
		res |= ERROR_GET_FRAME_DATA | ERROR_GET_FRAME;
		goto ERROR;
	}

	/* move the offset */
	address += (msFrame->node_data_size) * BYTES_PER_NODE;

	ssFrame->force_data = (short *)kmalloc(ssFrame->header.force_node *
					       sizeof(short), GFP_KERNEL);
	if (ssFrame->force_data == NULL) {
		logError(1,
			 "%s %s: impossible allocate memory for SS force frame...ERROR %08X\n",
			 tag, __func__, ERROR_ALLOC | ERROR_GET_FRAME);
		res = ERROR_ALLOC | ERROR_GET_FRAME;
		goto ERROR;
	}

	logError(0, "%s %s: Getting SS force frame at %04llX...\n", tag, __func__,
		 address);
	res = getFrameData(info, address, (ssFrame->header.force_node) *
			   BYTES_PER_NODE, (ssFrame->force_data));
	if (res < OK) {
		logError(1,
			 "%s %s: error while getting SS force data...ERROR %08X\n",
			 tag,
			 __func__, res);
		res |= ERROR_GET_FRAME_DATA | ERROR_GET_FRAME;
		goto ERROR;
	}

	/* move the offset */
	address += (ssFrame->header.force_node) * BYTES_PER_NODE;

	ssFrame->sense_data = (short *)kmalloc(ssFrame->header.sense_node *
					       sizeof(short), GFP_KERNEL);
	if (ssFrame->sense_data == NULL) {
		logError(1,
			 "%s %s: impossible allocate memory for SS sense frame...ERROR %08X\n",
			 tag, __func__, ERROR_ALLOC | ERROR_GET_FRAME);
		res = ERROR_ALLOC | ERROR_GET_FRAME;
		goto ERROR;
	}

	logError(0, "%s %s: Getting SS sense frame at %04llX...\n", tag, __func__,
		 address);
	res = getFrameData(info, address, (ssFrame->header.sense_node) *
			   BYTES_PER_NODE, (ssFrame->sense_data));
	if (res < OK) {
		logError(1,
			 "%s %s: error while getting SS sense data...ERROR %08X\n",
			 tag,
			 __func__, res);
		res |= ERROR_GET_FRAME_DATA | ERROR_GET_FRAME;
		goto ERROR;
	}

ERROR:
	if (res < OK) {
		if (msFrame->node_data != NULL) {
			kfree(msFrame->node_data);
			msFrame->node_data = NULL;
		}

		if (ssFrame->force_data != NULL) {
			kfree(ssFrame->force_data);
			ssFrame->force_data = NULL;
		}

		if (ssFrame->sense_data != NULL) {
			kfree(ssFrame->sense_data);
			ssFrame->sense_data = NULL;
		}
		logError(0, "Getting Sync Frame FAILED! ERROR %08X!\n", res);
	} else {
		logError(0, "Getting Sync Frame FINISHED!\n");
		res = msFrame->node_data_size + ssFrame->header.force_node +
		      ssFrame->header.sense_node;
	}
	return res;
}
