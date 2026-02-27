/*
 * Copyright (c) 2013 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


/*
 * This file asflog.c has the Log Manager library code
 * Author:  U. Loganathan
 * Date:    May 16th 2002
 * History:-
 * Date     Modified by      Modification Information
 * 10/1/02  Mayank Upadhyay  Added support for func name
 * 11/12/02  Andy Chinmulgund Removed syslog Facility calls 
 *
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <syslog.h>
#include <sys/types.h>
#include <unistd.h>

#include "aniAsfMem.h"
#include "aniAsfHdr.h"
#include "aniAsfLog.h"
#include "aniAsfIpc.h"
#include "aniErrors.h"

#define FILE_ONLY "%20s:%3ld %s\n"
#define FUNC_ONLY "%s: %s\n"
#define SIMPLE    "%s\n"

// Maximum allowed log level
static int  mlevel = 0;

// Flag for log Init
static int  loginit = 0;

// Flag for console log
static int  conslog = 0;

// appName for dumping with function name
static char *appName = NULL;

/**
 * aniAsfLogInit
 *
 * FUNCTION:
 *  Function to initialize logging for this process.
 *
 * @param *appname - Name of the application
 * @param *maxlevel - Maxmimum log level 
 *          LOG_EMERG - system is unusable
 *          LOG_ALERT - action must be taken immediately
 *          LOG_CRIT - critical conditions
 *          LOG_ERR - error conditions
 *          LOG_WARNING - warning conditions
 *          LOG_NOTICE - normal, but significant, condition
 *          LOG_INFO - informational message
 *          LOG_DEBUG - debug-level message
 * @param consflag - If TRUE prints log parallely to console
 *
 * @param return - None
 *
 */

void aniAsfLogInit(const char *appname, int maxlevel, int consflag)
{
	// Open syslog for appname, don't wait, User log
	openlog(appname, LOG_NOWAIT, LOG_USER);

	// Set Log level
	aniAsfLogSetLevel(maxlevel);

	// Store the console log flag
	conslog = consflag;

	// Store the app name to print with the function name
	appName = strdup(appname);

	// Set the flag for log init
	loginit = 1;

	// Log the Pid entry
	aniAsfLogMsg(ANI_LOG_INIT, appname, getpid());
}

/**
 * aniAsfLogConsole
 *
 * FUNCTION:
 *  Function to Change the console Flag
 *
 * LOGIC:
 *  Sets the console flag to true or flase
 *
 * @param None
 *
 * @param return - None
 *
 */

void aniAsfLogConsole(int consFlag)
{
	// Change the conslog Flag
	conslog = consFlag;
}


/**
 * aniAsfLogClose
 *
 * FUNCTION:
 *  Function to Close the syslog communication
 *
 * LOGIC:
 *  Uses the system call closelog
 *
 * @param None
 *
 * @param return - None
 *
 */

void aniAsfLogClose()
{
	aniAsfLogMsg(ANI_LOG_CLOSE, getpid());

	closelog();

	if (appName != NULL) {
		aniFree(appName);
	}
}

/**
 * aniAsfLogSetLevel
 *
 * FUNCTION:
 *  Function to change the maximum log level for this process.
 *
 * LOGIC:
 *  Uses the system call setlogmask to change the log level
 *
 * @param maxlevel - The new maximum log level.
 *
 * @param return - None
 *
 */

void aniAsfLogSetLevel(int maxlevel)
{
	// Set Log mask to the maxlevel as set my user
	setlogmask(LOG_UPTO(maxlevel));

	// Save max log level
	mlevel = maxlevel;
}

#ifndef ANI_ENTRY_LEVEL_AP
/**
 * aniAsfLogMsg
 *
 * FUNCTION:
 *  Function to log a message to syslog with a filename and
 *  line number, or an application and function name.
 *
 * LOGIC:
 *  Uses system call syslog
 *  If the log level of the current message is greater than
 *  the max level no action is taken
 *
 * @param level - Current log messages log level
 * @param *filename - File name from where aniAsfLogMsg was called. If
 * this is NULL, then the filename and line number are not logged.
 * @param line - line number from where aniAsfLogMsg was called.
 * @param func the function name to log. If this is NULL, then it is 
 * not logged.
 * @param *fmt - format of the message string
 * @param ... - va_list argument list
 *
 *
 * @return ANI_OK. errno is set if the operation failed.
 *
 */

int aniAsfLogMsg(int level, 
                 const char *filename, 
                 long line, 
                 const char *func,
                 const char *fmt, ...)
{
	va_list ap;
	char *cstring;
	const char *fname;
	int lerrno = errno;

	// If the log is not inited then return
	if (!loginit) {
		return(-1);
	}

	// if level is higher than maxlevel then return
	if (level > mlevel) {
		return(-1);
	}

	if ((cstring = aniMalloc(ANI_MAX_BUF_SIZE + 1)) == NULL) {

		if (conslog) {
			fprintf(stderr, SIMPLE, "Malloc Error");
			fflush(stderr);
		}
		syslog(level, SIMPLE, "Malloc Error");

		// Restore errno
		errno = lerrno;

		return(-1);
	}

	// extract the parameters to cstring
	va_start(ap, fmt);
	if (vsnprintf(cstring, ANI_MAX_BUF_SIZE, fmt, ap) < 0) {
		if (conslog) {
			fprintf(stderr, SIMPLE, "Length truncated");
			fflush(stderr);
		}
		syslog(level, SIMPLE, "Length truncated");

	}
	va_end(ap);

	// Remove the file path and get the actual filename
	if (filename != NULL) {
		if ((fname = strrchr(filename, '/')) == NULL) {
			fname = filename;
		} else {
			fname++;
		}
	} else {
		fname = NULL;
	}

	if (fname != NULL) {
		if (conslog) {
			fprintf(stderr, FILE_ONLY, fname, 
					line,  cstring);
			fflush(stderr);
		}
		syslog(level, FILE_ONLY, fname, line, cstring);
	} else if (func != NULL) {
		if (conslog) {
			fprintf(stderr, FUNC_ONLY, func,  cstring);
			fflush(stderr);
		}
		syslog(level, FUNC_ONLY, func, cstring);
	} else {
		if (conslog) {
			fprintf(stderr, SIMPLE, cstring);
			fflush(stderr);
		}
		syslog(level, SIMPLE, cstring);
	}

	aniFree(cstring);

	// Restore errno
	errno = lerrno;

	return ANI_OK;
}
#endif /* ANI_ENTRY_LEVEL_AP */
