/*
 * Copyright (c) 2013, 2016 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


/*
 * This file timer.c has the Event Manager library code
 * Author:	U. Loganathan
 * Date:	May 16th 2002
 * History:-
 * Date		Modified by	Modification Information
 *
 */

#include <stdio.h>
#include <sys/time.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/times.h>
#include <unistd.h>

#include "aniAsfMem.h"
#include "aniAsfHdr.h"
#include "aniAsfTimer.h"
#include "aniAsfLog.h"

#define MAX_TIMERS	2000

#define SEC_IN_USECS	1000000

typedef enum states {
	TIMER_NEW,
	TIMER_STOPPED,
	TIMER_RUNNING,
	TIMER_EXPIRED
}tState;

struct timer {

	// Duration of timer
	struct timeval	duration;

	// Time at expiration
	struct timeval	expiration;

	// Function to call at expiration
	void		(*func) (void *);

	// Arguments to be passed for call-back function
	void		*args;

	// Current Timer state
	tState		state;

	// Linked-list pointer
	struct timer	*next;

};

// List which holds all the active timers
static tAniTimer *timerlist = NULL;

// To hold the maximum active timers
static int maxtimers = 0;

// To hold the timer signal off flag
static int signaloff = 0;

// To hold ticks per sec value
static long ticksec = 1;

// To hold the wraptime in seconds
static clock_t wraptime = 0;

// To hold the previous clock tick
static clock_t prevjiffies = 0;

static void aniAsfSetTimer(void);

struct timerstats {

	// Number of timerProcess 
	// invocations
	int process;

	// Number of timerProcess 
	// invocations when no action
	// was taken
	int noprocess;

	// Number of time default timeout
	// is set
	int deftmos;

	// Number of wrap around
	int wrap;

	// Number of secs wrap around
	int secwrap;

	// Number of secs wrap under
	int secunder;

};

// Timer Statistics
static struct timerstats tmrstats;

// To handle the mutex lock for the multi threaded applications
pthread_mutex_t timermutex;

static int threadFlag = 0;


// General Macros
#define ANI_BLOCK_SIGALRM(nmask, omask) \
	sigemptyset(&nmask); \
	sigaddset(&nmask, SIGALRM); \
	sigprocmask(SIG_BLOCK, &nmask, &omask); 

#define ANI_UNBLOCK_SIGALRM(omask) \
	sigprocmask(SIG_SETMASK, &omask, NULL); 

/**
 * 4 bytes unsigned integer can hold
 *
 * 	Secs 	-	4294967295
 * 	Mins	-	  71582788.25
 * 	Hours	-	   1193046.47
 * 	Days	-	     49710.26
 * 	Years	-	       136.19
 *
 */

// Compare two timer values
static int aniAsfCmpTimer(struct timeval *t1,
			struct timeval *t2,
			struct timeval *diff)
{
	// Get the seconds and micro seconds diff
	diff->tv_sec = t1->tv_sec - t2->tv_sec;
	diff->tv_usec = t1->tv_usec - t2->tv_usec;

	// Check for the wrap under and adjust accordingly
	if (diff->tv_usec < 0) {
		diff->tv_sec--;
		diff->tv_usec = diff->tv_usec +
					SEC_IN_USECS;
		tmrstats.secunder++;
	}

	if (diff->tv_sec >= 0) {
		return(1);
	}

	return(0);
}

/**
 * aniAsfGetUpTime
 *
 * FUNCTION:
 * 	Internal Function to get the time in timeval struct 
 * 		from the clock tick 
 *
 * LOGIC:
 *
 * @param struct timeval
 *
 * @param return - None
 *
 */

void aniAsfGetUpTime(struct timeval *tval)
{
	clock_t jiffies;
	clock_t clksecs;
	clock_t tmp = 0;

	// Get the clock ticks
	jiffies = times(NULL);

	// Adjust for the time wrap 
	if (jiffies < prevjiffies) {
		wraptime += (~tmp / ticksec);
		tmrstats.wrap++;
	}

	prevjiffies = jiffies;

	// Convert to seconds
	clksecs = jiffies / ticksec;

	tval->tv_sec = clksecs + wraptime;
	tval->tv_usec = (jiffies % ticksec) * 10000;
}

/**
 * aniAsfTimerCommonInit
 *
 * FUNCTION:
 * 	Common Init Function Threaded and non
 * 		Threaded inits
 *
 * LOGIC:
 *
 * @param Thread flag 
 *
 * @param return - None
 *
 */

void aniAsfTimerCommonInit(int thFlag)
{
	// mutex init always returns 0
	pthread_mutex_init(&timermutex, NULL);

	// Set the Thread Flag
	threadFlag = thFlag;

	// Initialize the timer stats
	memset(&tmrstats, 0, sizeof(struct timerstats));

	// Get the clock ticks per second
	ticksec = sysconf(_SC_CLK_TCK);
}

/**
 * aniAsfTimerInit
 *
 * FUNCTION:
 * 	Function to initialize the Timer Manager
 *
 * LOGIC:
 * 	The function aniAsfTimerProcess will be called 
 * 		when the next timer expires. 
 * 	If turnsignaloff is not set, main thread should
 * 		invoke the function aniAsfTimerProcess 
 * 		periodically to handle timer expirations.  
 * 	It uses system calls sigaction to register
 * 		aniAsfTimerProcess as SIGALRM call back function.
 *
 * @param turnsignaloff - If this flag is set then the 
 * 		system calls signal and setitimer is not used.
 *
 * @param return - None
 *
 */

void aniAsfTimerInit(int turnsignaloff)
{
	struct sigaction act;
	
	// Call the common Init function
	aniAsfTimerCommonInit(0);

	if (turnsignaloff == 1) {
		signaloff = turnsignaloff;
		return;
	}

	// Initialize the sigaction structure
	memset(&act, 0, sizeof(struct sigaction));
	act.sa_handler = &aniAsfTimerProcess;
	sigemptyset(&act.sa_mask);
	act.sa_flags = SA_RESTART;

	// Register the call back function for the SIGALRM signal
	sigaction(SIGALRM, &act, NULL);
}

/**
 * aniAsfTimerCreate
 *
 * FUNCTION:
 * 	Function to create a timer
 *
 * LOGIC:
 * 	Initialize an instance of the timer structure
 * 	Fills the timer structure values - call back function 
 * 		and its arguments
 *
 * @param (*func)(void *) - Call back function associated 
 * 		with this timer
 * @param *args - argument context to be supplied at call back
 *
 * @param return - NULL on error or new timer pointer on success
 *
 */

tAniTimer *aniAsfTimerCreate(void(*func)(void *), void *args)
{
	tAniTimer *tmr;

	// allocate memory return NULL on error
	if ((tmr = (tAniTimer *)aniMalloc(
			sizeof(struct timer))) == NULL) {

		aniAsfLogMsg(ANI_MALLOC_ERR);
		return(NULL);

	}

	// Initialize timer pointer
	memset(tmr, 0, sizeof(struct timer));

	// save the call back function pointer and 
	// context information
	tmr->func = func;
	tmr->args = args;

	tmr->state = TIMER_NEW;

	return(tmr);
}

/**
 * aniAsfTimerFree
 *
 * FUNCTION:
 * 	Function to free the timer
 *
 * LOGIC:
 * 	Free an instance of the timer structure.
 *
 * @param *tmr - The timer pointer
 *
 * @param return - None
 *
 */

int aniAsfTimerFree(tAniTimer *tmr)
{
	//return if NULL
	if (tmr == NULL) {

		aniAsfLogMsg(ANI_NULL_TIMER);
		return -1;

	}

	// Stop the timer if running
	if (tmr->state == TIMER_RUNNING) {

		aniAsfTimerStop(tmr);

	}

	aniFree(tmr);
	return 0;
}

/**
 * aniAsfTimerArgs
 *
 * FUNCTION:
 * 	Function to return the args pointer
 *
 * LOGIC:
 * 	Return the args pointer
 *
 * @param *tmr - The timer pointer
 *
 * @param return - args pointer
 *
 */

void *aniAsfTimerArgs(tAniTimer *tmr)
{	
	//return if NULL
	if (tmr == NULL) {

		aniAsfLogMsg(ANI_NULL_TIMER);
		return NULL;

	}

	return(tmr->args);
}

/**
 * aniAsfTimerStart
 *
 * FUNCTION:
 * 	Function to start a timer
 *
 * LOGIC:
 * 	If the current timer is already running stop it to 
 * 	restart.  Computes the time in the future, when 
 * 	the current timer will expire.  Inserts the current 
 * 	timer into a linked list of active timers in the 
 * 	ascending order of timer expiration.  In other words the 
 * 	timer that will expire first will be on the top of 
 * 	this list.
 *
 * @param *tmr - The timer pointer
 *
 * @param return - None
 *
 */
int aniAsfTimerStart(tAniTimer *tmr)
{
	sigset_t nmask, omask;
	tAniTimer *tnext = NULL, *tprev = NULL;
	struct timeval tv;
	struct timeval diff;

	// If NULL pointer, then return
	if (tmr == NULL) {

		aniAsfLogMsg(ANI_NULL_TIMER);
		return -1;

	}

	// If duration is zero, then return
	if ((tmr->duration.tv_sec == 0) &&
			(tmr->duration.tv_usec == 0)) {

		aniAsfLogMsg(ANI_NULL_DURATION);
		return -1;

	}

	// If timer is already running stop and then start
	if (tmr->state == TIMER_RUNNING) {

		aniAsfTimerStop(tmr);

	}

	// Check for the maximum active timers
	if (maxtimers == MAX_TIMERS) {

		aniAsfLogMsg(ANI_MAX_TMRS, maxtimers);
        	return -1;

	}

	if ((threadFlag == 0) && (signaloff == 0)) {

		// Block Signals
		ANI_BLOCK_SIGALRM(nmask, omask);

	}

	// lock the critical section
	if ((errno = pthread_mutex_lock(&timermutex)) > 0) {

		aniAsfLogMsg(ANI_MUTEX_ERR);

		if ((threadFlag == 0) && (signaloff == 0)) {

			// Unblock Signals
			ANI_UNBLOCK_SIGALRM(omask);

		}

		return -1;

	}

	// increment the maxtimers
	maxtimers++;

	// Get the Uptime
	aniAsfGetUpTime(&tv);

	// Set the expiration time 
	tmr->expiration.tv_sec = tv.tv_sec + 
				tmr->duration.tv_sec;
	tmr->expiration.tv_usec = tv.tv_usec + 
				tmr->duration.tv_usec;

	// Take care of usecs wrap around
	if (tmr->expiration.tv_usec >= SEC_IN_USECS) {

		// increment the secs and adjsut usecs.
		tmr->expiration.tv_sec++;
		tmr->expiration.tv_usec = tmr->expiration.
				tv_usec - SEC_IN_USECS;

		tmrstats.secwrap++;

	}

	// Change state to RUNNING
	tmr->state = TIMER_RUNNING;

	// Insert this timer to the list sorted so that 
	// first timer to expire is always at the top of the list
	for (tnext = timerlist; tnext != NULL; 
			tnext = tnext->next) {

		if (aniAsfCmpTimer(&tnext->expiration,
				&tmr->expiration, &diff)) {

			break;

		}

		tprev = tnext;

	}

	if (tprev == NULL) {

		timerlist = tmr;

	} else {

		tprev->next = tmr;

	}

	tmr->next = tnext;

	// If SIGALRM signal is not turned off 
	if (signaloff == 0) {

		// Set the new expiration time 
		aniAsfSetTimer();

	}

	// unlock the critical section
	if ((errno = pthread_mutex_unlock(&timermutex)) > 0) {

		aniAsfLogMsg(ANI_MUTEX_ERR);

	}

	if ((threadFlag == 0) && (signaloff == 0)) {

		// Unblock Signals
		ANI_UNBLOCK_SIGALRM(omask);

	}

	return 0;
}

/**
 * aniAsfTimerStop
 *
 * FUNCTION:
 * 	Function to stop the currently active timer
 *
 * LOGIC:
 * 	Walks through the list of active timers and 
 * 	matches the current timer in the list and removes 
 * 	it from the list.
 *
 * @param *tmr - The timer pointer
 *
 * @param return - None
 *
 */

int aniAsfTimerStop(tAniTimer *tmr)
{
	sigset_t nmask, omask;
	tAniTimer *tcur = NULL, *tlast = NULL;

	// If NULL pointer, then return
	if (tmr == NULL) {

		aniAsfLogMsg(ANI_NULL_TIMER);
		return -1;

	}

	// If timer is not in the running state, then return
	if (tmr->state != TIMER_RUNNING) {

		aniAsfLogMsg(ANI_TIMER_NOT_RUNNING);
		return -1;

	}

	if ((threadFlag == 0) && (signaloff == 0)) {

		// Block Signals
		ANI_BLOCK_SIGALRM(nmask, omask);

	}

	// lock the critical section
	if ((errno = pthread_mutex_lock(&timermutex)) > 0) {

		aniAsfLogMsg(ANI_MUTEX_ERR);

		if ((threadFlag == 0) && (signaloff == 0)) {

			// Unblock Signals
			ANI_UNBLOCK_SIGALRM(omask);

		}

		return -1;

	}

	// Verify that timer is really on list
	for (tcur = timerlist; tcur != NULL; tcur = tcur->next) {

		if (tcur == tmr) {

			break;

		}

		tlast = tcur;

	}

	if (tcur != NULL) {

		if (tlast != NULL) {

			tlast->next = tcur->next;

		} else {

			timerlist = tcur->next;

		}

		tcur->state = TIMER_STOPPED;

        	// decrement the maxtimers
        	maxtimers--;

	}

	// If SIGALRM signal is not turned off 
	if (signaloff == 0) {

		// Set the new expiration time 
		aniAsfSetTimer();

	}

	// unlock the critical section
	if ((errno = pthread_mutex_unlock(&timermutex)) > 0) {

		aniAsfLogMsg(ANI_MUTEX_ERR);

	}

	if ((threadFlag == 0) && (signaloff == 0)) {

		// Unblock Signals
		ANI_UNBLOCK_SIGALRM(omask);

	}

	return 0;
}

/**
 * aniAsfTimerSet
 *
 * FUNCTION:
 * 	Function to set the timeout value in msecs for 
 * 	the current timer
 *
 * LOGIC:
 * 	Changing the time out value on a running timer 
 * 	will on affect the current time.
 * 	The next aniAsfTimerStart will use this changed tmo value
 *
 * @param *tmr - The timer pointer
 * @param tmo - The timeout value
 *
 * @param return - None
 *
 */

void aniAsfTimerSet(tAniTimer *tmr, unsigned long tmo)
{
	// If NULL pointer, then return
	if (tmr == NULL) {

		aniAsfLogMsg(ANI_NULL_TIMER);
		return;

	}

	tmr->duration.tv_sec = tmo / 1000;
	tmr->duration.tv_usec = (tmo % 1000) * 1000;
}

/**
 * aniAsfTimerGet
 *
 * FUNCTION:
 * 	Function to get the remaining timeout value for 
 * 	the current timer
 *
 * LOGIC:
 *	Computes the timeout value and returns the timeout 
 *	value in msecs
 *
 * @param *tmr - The timer pointer
 *
 * @param return - Remaining timeout value in msecs
 *
 */

unsigned long aniAsfTimerGet(tAniTimer *tmr)
{
	unsigned long tleft;
	struct timeval tv;
	struct timeval diff;

	// If NULL pointer, then return
	if (tmr == NULL) {

		aniAsfLogMsg(ANI_NULL_TIMER);
		return(0);

	}

	// If timer is not in the run state, then return
	if (tmr->state != TIMER_RUNNING) {

		return(0);

	}

	// Get the Uptime
	aniAsfGetUpTime(&tv);

	// Compute the time remaining
	if (aniAsfCmpTimer(&tmr->expiration, &tv, &diff)) {

		tleft = (diff.tv_sec * 1000) +
				(diff.tv_usec / 1000);
		return (tleft);
	
	}

	return(0);
}

/**
 * aniAsfTimerProcess
 *
 * FUNCTION:
 * 	Function to handle all expired timers.
 *
 * LOGIC:
 * 	Walks through the linked list and gathers all the 
 * 		timers that have expired into an expired list. 
 * 	While walking through the list if it finds a timer 
 * 		that has not expired then it stops looking 
 * 		further, because the timers are stored in 
 * 		an ascending order of expiration
 * 	Then it invokes the call back function of the 
 * 		expired timers one by one. 
 * 	The need for an expired list is to allow the 
 * 		call back function to start and stop the 
 * 		timers (which internally changes the timer
 * 		linked list).  This is very critical in 
 * 		multithreaded applications.
 *
 * @param arg - argument context from signal handler
 *
 * @param return - None
 *
 */

void aniAsfTimerProcess(int arg)
{
	UNUSED(arg);
	tAniTimer *tmr, *expired = NULL;
	struct timeval tv;
	struct timeval diff;
	int flag = 0;

	// count the timer process entries
	tmrstats.process++;

	// If there are no active timers, return
	if (timerlist == NULL) {

		return;

	}

	// lock the critical section
	if ((errno = pthread_mutex_lock(&timermutex)) > 0) {

		aniAsfLogMsg(ANI_MUTEX_ERR);
		return;

	}

	// Get the Uptime
	aniAsfGetUpTime(&tv);

	// handle expired timers
	while ((timerlist != NULL) &&
		(aniAsfCmpTimer(&tv, &timerlist->expiration, 
					&diff))) {
		
		flag = 1;
		
		// Save the current timer 
		tmr = timerlist;

		// Change the current expired timer state 
		// and move to the next timer
		timerlist->state = TIMER_EXPIRED;
		timerlist = timerlist->next;

        	// decrement the maxtimers
        	maxtimers--;

		if (tmr->func) {

			tmr->next = expired;
			expired = tmr;

		}
	}

	// unlock the critical section
	if ((errno = pthread_mutex_unlock(&timermutex)) > 0) {

		aniAsfLogMsg(ANI_MUTEX_ERR);
		return;

	}

	if (flag == 0) {
		tmrstats.noprocess++;
	}

	while ((tmr = expired) != NULL) {

		expired = tmr->next;

		// Execute the call back function
		(*tmr->func)(tmr->args);

	}

	// If SIGALRM signal is not turned off 
	if (signaloff == 0) {

		// Set the new expiration time 
		aniAsfSetTimer();

	}
}

/**
 * aniAsfSetTimer
 *
 * FUNCTION:
 * 	Function to initialize the next timer Tick
 *
 * LOGIC:
 *
 * @param void
 *
 * @param return - None
 *
 */

static void aniAsfSetTimer(void)
{
	struct itimerval tval;
	struct timeval tv;
	struct timeval diff;
	long secs = 0;
	long usecs = 0;

	if (timerlist != NULL) {

		// Get the Uptime
		aniAsfGetUpTime(&tv);

		// Compare the Uptime with the top of
		// the Timer list to get the timeout for the
		// next timer tick
		if (aniAsfCmpTimer(&timerlist->expiration, &tv,
					&diff)) {

			secs = diff.tv_sec;
			usecs = diff.tv_usec;

		} else {
		
			// already expired (10 msecs)
			usecs = 10 * 1000;

			tmrstats.deftmos++;

		}

	}

	// Initialize for the timer tick
	tval.it_interval.tv_sec = 0;
	tval.it_interval.tv_usec = 0; 
	tval.it_value.tv_sec = secs;
	tval.it_value.tv_usec = usecs;

	// Start the timer tick
	setitimer(ITIMER_REAL, &tval, 0);
}

