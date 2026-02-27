/*
 * Copyright (c) 2013 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


/*
 * This file asfevent.c has the Event Manager library code
 * Author:	U. Loganathan
 * Date:	May 16th 2002
 * History:-
 * Date		Modified by	Modification Information
 *
 */

#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#include "aniAsfMem.h"
#include "aniAsfHdr.h"
#include "aniAsfEvent.h"
#include "aniAsfLog.h"

#define MAX_EVENT_ENTRIES	1000

// Internal function to create an instance of event
static  tAniEvent *eventCreate(long, void *);

// Internal function to free an instance of event
static  void eventFree(tAniEvent *);

struct event {

	// Event ID
	long		eventId;
	
	// Data Associated with this eventId
	void		*data;

	// Linked-list next pointer
	struct event	*next;

};

struct eventq {

	// Function to call back for this Event Queue 
	void		(*func) (long, void *);

	// Total number of events
	long		count;

	// Total number process entered
	long		entry;

	// Total number procossed
	long		process;

	// Total number producer
	long		prod;

	// Total number consumer
	long		cons;

	// Total number retries
	long		retry;

	// Total number recovered
	long		fail;

	// Thread Id if the events are
	// handled in a seperate thread
	pthread_t	threadId;

	// PThread Condition is used for
	// communication between the Event
	// Send and the process function
	pthread_cond_t	cond;

	// Mutex lock to control critical section
	pthread_mutex_t	mutex;

	// Head of the Pending Events 
	struct event	*head;

	// Pending Events in a linked list 
	struct event	*events;

};


/**
 * aniAsfEventQCreate
 *
 * FUNCTION:
 * 	Function to create and initialize an instance of the event queue
 *
 * LOGIC:
 * 	Each event queue is associated with a call back function
 *
 * @param (*func)(long, void *) - Call back function for this event queue
 *
 * @param return - NULL on error; or New Event Queue pointer on success
 *
 */

tAniEventQ *aniAsfEventQCreate(void (*func)(long, void *))
{
	tAniEventQ *eventq;

	// allocate memory return NULL on error
	if ((eventq = (tAniEventQ *)
			aniMalloc(sizeof(struct eventq))) == NULL) {
		aniAsfLogMsg(ANI_MALLOC_ERR);
		return(NULL);
	}

	memset(eventq, 0, sizeof(struct eventq));

	// save the function pointer 
	eventq->func = func;

	// Init pthread mutex to protect the critical
	// section for multiple threads
	// pthread_mutex always returns zero
	pthread_mutex_init(&eventq->mutex, NULL);

	return(eventq);
}

/**
 * aniAsfEventCleanUp
 *
 * FUNCTION:
 * 	Function to free the event queue pointer
 *
 * LOGIC:
 * 	Frees the event queue pointer
 *
 * @param *eventq - Event queue pointer to be freed
 *
 * @param return - None
 *
 */

void aniAsfEventCleanUp(tAniEventQ *eventq)
{
	// return if NULL
	if (eventq == NULL) {
		return;
	}

	if (eventq->threadId) {

		// Lock the critical section
		if ((errno = pthread_mutex_lock(&eventq->
						mutex)) > 0) {
			aniAsfLogMsg(ANI_MUTEX_ERR);
		}

		pthread_cond_signal(&eventq->cond);

		// UnLock the critical section
		if ((errno = pthread_mutex_unlock(&eventq->
						mutex)) > 0) {
			aniAsfLogMsg(ANI_MUTEX_ERR);
		}

	}

}

/**
 * aniAsfEventQFree
 *
 * FUNCTION:
 * 	Function to free the event queue pointer
 *
 * LOGIC:
 * 	Frees the event queue pointer
 *
 * @param *eventq - Event queue pointer to be freed
 *
 * @param return - None
 *
 */

void aniAsfEventQFree(tAniEventQ *eventq)
{
	// return if NULL
	if (eventq == NULL) {
		return;
	}

	if (eventq->threadId) {
		// Clean up condition
		if ((errno = pthread_cond_destroy(&eventq->
						cond)) > 0) {
			aniAsfLogMsg(ANI_COND_ERR);
		}
	}

	// Clean up Mutex
	if ((errno = pthread_mutex_destroy(&eventq->mutex)) > 0) {
		aniAsfLogMsg(ANI_MUTEX_ERR);
	}

	aniFree(eventq);
}

/**
 * aniAsfEventSend
 *
 * FUNCTION:
 * 	Function to send a event
 *
 * LOGIC:
 * 	Adds the event to the event queue
 *
 * @param *eventq - Event queue to add the event
 * @param eventId - Unique event identification number
 * @param void *data - data associated with this event
 *
 * @param return - None
 *
 */

int aniAsfEventSend(tAniEventQ *eventq, long eventId, void *data)
{
	tAniEvent *eve = NULL;

	// If eventQ is NULL, then return
	if (eventq == NULL) {
		return(-1);
	}

	errno = 0;

	// Lock the critical section
	if ((errno = pthread_mutex_lock(&eventq->mutex)) != 0) {
		aniAsfLogMsg(ANI_MUTEX_ERR);
		return(-1);
	}

	if ((eve = eventCreate(eventId, data)) == NULL) {
		// UnLock the critical section
		if ((errno = pthread_mutex_unlock(&eventq->
						mutex)) > 0) {
			aniAsfLogMsg(ANI_MUTEX_ERR);
		}
		return(-1);
	}

	if (eventq->count == MAX_EVENT_ENTRIES) {
		aniAsfLogMsg(LOG_ERR, ANI_WHERE,
"EventQ:0x%x, Max Events(%d) Reached,  Produced %d, Consumed %d, Retry %d, Entered %d, Processed %d, Failed %d",
			eventq, eventq->count, eventq->prod, 
			eventq->cons, eventq->retry, eventq->entry,
			eventq->process, eventq->fail);

		eventFree(eve);

		// UnLock the critical section
		if ((errno = pthread_mutex_unlock(&eventq->
						mutex)) > 0) {
			aniAsfLogMsg(ANI_MUTEX_ERR);
		}
		return(-1);
	}

	// Insert into the list
	if (eventq->events == NULL) {
		eventq->events = eve;
		eventq->head = eve;
		eventq->count = 0;
	} else {
		eventq->events = eventq->events->next = eve;
	}

	eve->next = NULL;

	eventq->count++;
	eventq->prod++;

	/*
	 * If the Event Process is run seperately without
	 * ipc process and when there are no events in the eventq,
	 * eventProcess could run a tight spin loop 
	 * consuming CPU cycles. To avoid this, problem 
	 * on an optional basis, could release eventProcess
	 * from block using pthread conditions
	 *
	 */
	if (eventq->threadId) {
		pthread_cond_signal(&eventq->cond);
	}

	// UnLock the critical section
	if ((errno = pthread_mutex_unlock(&eventq->mutex)) > 0) {
		aniAsfLogMsg(ANI_MUTEX_ERR);
	}

	return(0);
}

/**
 * aniAsfEventProcess
 *
 * FUNCTION:
 * 	Driver to handle the events in the event queue
 *
 * LOGIC:
 * 	Walks through the event queue list.
 * 	Invokes the function call back function for each event queue 
 * 		in the list
 * 	This function should be called from main thread for single 
 * 		threaded application and directly from other corresponding 
 * 		threads in case of multithreaded multi event queues.
 *
 * @param *teventq - Event queue pointer
 *
 * @param return - -1 on error; or 0 on success
 *
 */

int aniAsfEventProcess(tAniEventQ *teventq)
{
	tAniEventQ *eventq = teventq;
	tAniEvent *events, *tevent;

start:

	// If eventq is NULL return
	if (eventq == NULL) {
		return -1;
	}

	eventq->entry++;

	// Lock the critical section
	if ((errno = pthread_mutex_lock(&eventq->mutex)) != 0) {
		aniAsfLogMsg(ANI_MUTEX_ERR);
		return -1;
	}

	/*
	 * If the Event Process is run seperately without 
	 * ipc process and when there are no events in
	 * the eventq, eventProcess could run a tight spin 
	 * loop consuming CPU cycles. To avoid this, problem 
	 * on an optional basis this function could block 
	 * till it recevies notification from the Event 
	 * send funcion thru mutex locks
	 *
	 */
	if ((eventq->threadId) && (eventq->head == NULL)) {
		// Mutex Lock gets unlocked in the cond_wait
		if ((errno = pthread_cond_wait(&eventq->cond,
				&eventq->mutex)) > 0) {
			aniAsfLogMsg(ANI_COND_ERR);

			eventq->fail++;

			// UnLock the critical section
			if ((errno = pthread_mutex_unlock(
					&eventq->mutex)) > 0) {
				aniAsfLogMsg(ANI_MUTEX_ERR);
			}
			return -1;
		}
	}

	eventq->process++;

	// Start from the head
	events = eventq->head;

	// Allow events to be posted to the eventq while other 
	// pending events are still there to be handled
	eventq->events = NULL;
	eventq->head = NULL;
	eventq->count = 0;

	// UnLock the critical section
	if ((errno = pthread_mutex_unlock(&eventq->mutex)) > 0) {
		aniAsfLogMsg(ANI_MUTEX_ERR);
		return -1;
	}

 	// Invoke the call back function for each
 	// event in the event queue
	while (events != NULL) {

		eventq->cons++;

		// Invoke the call back function
		(*eventq->func)(events->eventId, events->data);

		// Save the current event
		tevent = events;

		// Move on to the next event
		events = events->next;

		// Free the current event
		eventFree(tevent);
	}

	if (eventq->prod != eventq->cons) {
		eventq->retry++;
		goto start;
	}

	return(0);
}

/**
 * aniAsfEventStats
 *
 * FUNCTION:
 * 	Debugging function to print the Event Stats 
 *
 * LOGIC:
 * 	Frees the event pointer
 *
 * @param *event - Event pointer to be freed
 * @param *name - Event Name to track the stats by name
 *
 * @param return - None
 *
 */

void aniAsfEventStats(tAniEventQ *eventq, char *name)
{
	// return if NULL
	if (eventq == NULL) {
		return;
	}

	aniAsfLogMsg(LOG_NOTICE, ANI_WHERE, 
"%15s:  prod %8d, cons %8d, retry %8d, count %4d, entry %8d, process %8d, fail %4d", 
		name, eventq->prod, eventq->cons, eventq->retry,
		eventq->count, eventq->entry, 
		eventq->process, eventq->fail);

}

/**
 * eventCreate
 *
 * FUNCTION:
 * 	Function to initialize and create an instance of an event
 *
 * LOGIC:
 * 	Copies the eventid and eventdata into the event
 *
 * @param eventid - unique eventid
 * @param *data - data associated to this event
 *
 * @param return - NULL on error; or New Event Pointer on success
 *
 */

static tAniEvent *eventCreate(long eventId, void *data)
{
	tAniEvent *event;

	// allocate memory return NULL on error
	if ((event = (tAniEvent *)
			aniMalloc(sizeof(struct event))) == NULL) {
		aniAsfLogMsg(ANI_MALLOC_ERR);
		return(NULL);
	}

	// save the eventId and the data
	event->eventId = eventId;
	event->data = data;

	return(event);
}

/**
 * eventFree
 *
 * FUNCTION:
 * 	Function to free the event pointer
 *
 * LOGIC:
 * 	Frees the event pointer
 *
 * @param *event - Event pointer to be freed
 *
 * @param return - None
 *
 */

static void eventFree(tAniEvent *event)
{
	// return if NULL
	if (event == NULL) {
		return;
	}

	aniFree(event);
}

