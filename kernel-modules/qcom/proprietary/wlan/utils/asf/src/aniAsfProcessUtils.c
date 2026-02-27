/*
 * Copyright (c) 2013 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


/*
 * This file timer.c has the Event Manager library code
 * Author:	U. Loganathan
 * Date:	May 24th 2002
 * History:-
 * Date		Modified by	Modification Information
 * 12/17/03  yauchu      Rewrote aniAsfDaemonize().
 * 04/22/04  gilbert     Make SDK use previous code.
 *
 */

#include <unistd.h>

#ifdef ANI_AP_SDK
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#endif /* ANI_AP_SDK */

/**
 * aniAsfDaemonize
 *
 * FUNCTION:
 * 	Function to Daemonize a process
 *
 * LOGIC:
 * 	Uses system call fork to create a child process
 * 	Close the file
 * 	Closes the open file descriptors, 0, 1 & 2
 * 	Sets a new sesion id to complete the daemonization
 *
 * @param None
 *
 * @param return - None
 *
 */

void aniAsfDaemonize(void)
{
#ifdef ANI_AP_SDK
	int pid;

	// Create a child process 
	pid = fork();

	// Exit the parent process
	if (pid > 0) {
		exit(0);
	}

	// Fork error
	if (pid == -1) {
		exit(1);
	}

	// Create a new session and set the process group ID 
	setsid();

	// Close standard input
	(void)close(STDIN_FILENO);

	// Close standard output
	(void)close(STDOUT_FILENO);

	// Close standrad error
	(void)close(STDERR_FILENO);

	// Open the file /dev/null for reading 
	// and STDIN_FILENO gets the return file descriptor
	(void)open("/dev/null", O_RDWR);

	// Dup STDIN_FILENO to STDOUT_FILENO
	(void)dup2(STDIN_FILENO, STDOUT_FILENO);

	// Dup STDIN_FILENO to STDERR_FILENO
	(void)dup2(STDIN_FILENO, STDERR_FILENO);
#else /* ANI_AP_SDK */
    /*
     * Create a new session and make ourself the session group and process
     * group leader.  This is necessary in order to prevent the kernel from
     * sending a SIGHUP to this process and the child process in case this
     * process was started from the shell and the shell dies while this
     * process and/or the child process belong to the same process group.  If
     * that happens, the process(es) which do not handle  SIGHUP will 
     * terminate since that is the default action for SIGHUP.
     */

    setsid();


    /*
     * Make ourself into a daemon.
     */

    daemon(0, 0);
#endif /* ANI_AP_SDK */
}
