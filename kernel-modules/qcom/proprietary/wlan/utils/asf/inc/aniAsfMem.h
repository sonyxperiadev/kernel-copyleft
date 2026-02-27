/*
 * Copyright (c) 2013 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


/*
 * This file aniAsfMem.h is for the Memory Manager (Function declarations)
 * Author:  U. Loganathan
 * Date:    May 28th 2002
 * History:-
 * Date     Modified by Modification Information
 *
 */

#ifndef _ANI_ASF_MEM_H_
#define _ANI_ASF_MEM_H_

#include <malloc.h>

#ifdef ANI_DMALLOC
#include "dmalloc.h"
#endif

#define aniMalloc   malloc
#define aniFree     free
#define aniCalloc   calloc

#endif /* _ANI_ASF_MEM_H_ */
