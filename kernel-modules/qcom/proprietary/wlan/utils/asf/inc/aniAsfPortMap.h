/*
 * Copyright (c) 2013 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


/*
 * This file aniAsfPortMap.h is for the Port Map (Function declarations) 
 * Author:  U. Loganathan
 * Date:    May 28th 2002
 * History:-
 * Date     Modified by Modification Information
 *
 */

#ifndef _ANI_ASF_PORT_MAP_H_
#define _ANI_ASF_PORT_MAP_H_

extern int aniAsfPmSet(int, int, int, int);
extern int aniAsfPmUnSet(int, int, int);
extern int aniAsfPmGetPort(char *, int, int, int);
extern int aniAsfPmDump(void);

#endif /* _ANI_ASF_PORT_MAP_H_ */
