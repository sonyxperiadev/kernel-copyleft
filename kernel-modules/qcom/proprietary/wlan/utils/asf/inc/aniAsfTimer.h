/*
 * Copyright (c) 2013 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


/*
 * This file timer.h is for the Timer Manager (Function declarations)
 * Author:  U. Loganathan
 * Date:    May 16th 2002
 * History:-
 * Date     Modified by Modification Information
 *
 */

#ifndef _ANI_ASF_TIMER_H_
#define _ANI_ASF_TIMER_H_

#include <sys/time.h>

typedef struct timer tAniTimer;

// Structure defined for use by the Applications
// *** This is not the definition for AniTimer
typedef struct sAppsTimers {
	// Function name
	void	(*func)(void *);

	// Timeout Value
	long	timeout;

	// Timer Name
	char	*name;
} tAppsTimers;

extern void aniAsfTimerCommonInit(int);
extern void aniAsfTimerInit(int);
extern tAniTimer *aniAsfTimerCreate(void (*)(void *), void *);
extern int aniAsfTimerFree(tAniTimer *);
extern void *aniAsfTimerArgs(tAniTimer *);
extern int aniAsfTimerStart(tAniTimer *);
extern int aniAsfTimerStop(tAniTimer *);
extern void aniAsfTimerSet(tAniTimer *, unsigned long);
extern unsigned long aniAsfTimerGet(tAniTimer *);
extern void aniAsfTimerProcess(int);

extern void aniAsfTimerThreadInit(void);
extern void aniAsfTimerThreadStop(void);

extern void aniAsfGetUpTime(struct timeval *);

#endif /* _ANI_ASF_TIMER_H_ */
