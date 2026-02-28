/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2016-2019, STMicroelectronics Limited.
 * Authors: AMG(Analog Mems Group) <marco.cali@st.com>
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/*
  *
  **************************************************************************
  **                        STMicroelectronics				  **
  **************************************************************************
  **                        marco.cali@st.com				**
  **************************************************************************
  *                                                                        *
  *               FTS functions for getting Initialization Data		  **
  *                                                                        *
  **************************************************************************
  **************************************************************************
  *
  */

/*!
  * \file ftsCompensation.h
  * \brief Contains all the definitions and structs to work with Initialization
  * Data
  */

#ifndef FTS_COMPENSATION_H
#define FTS_COMPENSATION_H

#include "ftsCore.h"
#include "ftsSoftware.h"



#define RETRY_COMP_DATA_READ 2	/* /< max number of attempts to read
				 * initialization data */


/* Bytes dimension of Compensation Data Format */

#define COMP_DATA_HEADER	DATA_HEADER	/* /< size in bytes of
						 * initialization data header */
#define COMP_DATA_GLOBAL	(16 - COMP_DATA_HEADER)	/* /< size in bytes
							  * of initialization
							  *data general info */


#define HEADER_SIGNATURE	0xA5	/* /< signature used as starting byte of
					 * data loaded in memory */



/**
  * Struct which contains the general info about Frames and Initialization Data
  */
typedef struct {
	int force_node;	/* /< Number of Force Channels in the
			 * frame/Initialization data */
	int sense_node;	/* /< Number of Sense Channels in the
			 * frame/Initialization data */
	int type;	/* /< Type of frame/Initialization data */
} DataHeader;

/**
  * Struct which contains the MS Initialization data
  */
typedef struct {
	DataHeader header;	/* /< Header */
	i8 cx1;	/* /< Cx1 value (can be negative)) */
	i8 *node_data;	/* /< Pointer to an array of bytes which contains the
			 * CX2 data (can be negative) */
	int node_data_size;	/* /< size of the data */
} MutualSenseData;


/**
  * Struct which contains the SS Initialization data
  */
typedef struct {
	DataHeader header;	/* /< Header */
	u8 f_ix1;	/* /< IX1 Force */
	u8 s_ix1;	/* /< IX1 Sense */
	i8 f_cx1;	/* /< CX1 Force (can be negative) */
	i8 s_cx1;	/* /< CX1 Sense (can be negative) */
	u8 f_max_n;	/* /< Force MaxN */
	u8 s_max_n;	/* /< Sense MaxN */
	u8 f_ix0;	/* /< IX0 Force */
	u8 s_ix0;	/* /< IX0 Sense */
	u8 *ix2_fm;	/* /< pointer to an array of bytes which contains Force
			 * Ix2 data node */
	u8 *ix2_sn;	/* /< pointer to an array of bytes which contains Sense
			 * Ix2 data node */
	i8 *cx2_fm;	/* /< pointer to an array of bytes which contains Force
			 * Cx2 data node (can be negative) */
	i8 *cx2_sn;	/* /< pointer to an array of bytes which contains Sense
			 * Cx2 data node (can be negative)) */
} SelfSenseData;

/**
  * Struct which contains the TOT MS Initialization data
  */
typedef struct {
	DataHeader header;	/* /< Header */
	short *node_data;	/* /< pointer to an array of ushort which
				 * contains TOT MS Initialization data */
	int node_data_size;	/* /< size of data */
} TotMutualSenseData;

/**
 * Struct which contains the TOT SS Initialization data
 */
typedef struct {
	DataHeader header;	/* /< Header */

	u16 *ix_fm;	/* /< pointer to an array of ushort which contains TOT
			 * SS IX Force data
			 */
	u16 *ix_sn;	/* /< pointer to an array of ushort which contains TOT
			 * SS IX Sense data
			 */
	short *cx_fm;	/* /< pointer to an array of ushort which contains TOT
			 * SS CX Force data (can be negative)
			 */
	short *cx_sn;	/* /< pointer to an array of ushort which contains TOT
			 * SS CX Sense data (can be negative)
			 */
} TotSelfSenseData;



int requestCompensationData(struct fts_ts_info *info, u8 type);
int readCompensationDataHeader(struct fts_ts_info *info, u8 type, DataHeader *header,
	u64 *address);
int readMutualSenseGlobalData(struct fts_ts_info *info, u64 *address, MutualSenseData *global);
int readMutualSenseNodeData(struct fts_ts_info *info, u64 address, MutualSenseData *node);
int readMutualSenseCompensationData(struct fts_ts_info *info, u8 type, MutualSenseData *data);
int readSelfSenseGlobalData(struct fts_ts_info *info, u64 *address, SelfSenseData *global);
int readSelfSenseNodeData(struct fts_ts_info *info, u64 address, SelfSenseData *node);
int readSelfSenseCompensationData(struct fts_ts_info *info, u8 type, SelfSenseData *data);
int readToTMutualSenseGlobalData(struct fts_ts_info *info, u64 *address,
	TotMutualSenseData *global);
int readToTMutualSenseNodeData(struct fts_ts_info *info, u64 address, TotMutualSenseData *node);
int readTotMutualSenseCompensationData(struct fts_ts_info *info, u8 type,
	TotMutualSenseData *data);
int readTotSelfSenseGlobalData(struct fts_ts_info *info, u64 *address,
	TotSelfSenseData *global);
int readTotSelfSenseNodeData(struct fts_ts_info *info, u64 address, TotSelfSenseData *node);
int readTotSelfSenseCompensationData(struct fts_ts_info *info, u8 type,
	TotSelfSenseData *data);
#endif
