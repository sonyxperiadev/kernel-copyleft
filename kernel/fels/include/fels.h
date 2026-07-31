/*
 * Copyright 2023 Sony Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef FELS_H
#define FELS_H

#define FELS_MAX_DEBUG_MSG_LEN	1024
#define FELS_MAX_LOG_COUNT	128

/*
 * fels logs return codes
 * Return FELS_SUCCESS in success case with no errors
 * Return FELS_ERROR for any generic error
 * Return FELS_ILLEGAL ARGUMENT, when unexpected arugment is passed.
 * Below are conditions when FELS_ILLEGAL ARGUMENT will be returned.
 *      debug_msg must not be NULL.
 *      debug_msg length must be between 1 to 1024.
 *      category must be within the range of values defined in fels_category_t.
 *      log_level must be within the range of values defined in fels_log_level_t.
 *      error_code must be within the range of values defined in fels_error_code_t.
 * Return FELS_NO_MEM, when the memory allocation fails.
 * Return FELS_FULL_OF_LOGS, Max number of cached logs is 128 for fels_logs and fels_debug_logs.
 */

typedef enum {
	FELS_SUCCESS = 0,            /* No error */
	FELS_ERROR = -1,             /* General error */
	FELS_ILLEGAL_ARGUMENT = -2,  /* Illegal parameters */
	FELS_NOMEM = -3,             /* Out of memory */
	FELS_FULL_OF_LOGS = -4,      /* Number of cached logs croosed more than 128 */
} fels_return_t;

/**
 * logging function to drivers with minimal information
 * @param category fels_category_t enum
 * to determine the log from which driver
 * @param log_level fels_log_level_t enum
 * to know the type of log_level
 * @param error_code fels_error_code_t enum
 * to determine the which error from corresponding driver
 * @param parm1,parm2,parm3,parm4
 * extra int parameters along with error code
 * @return 0 on success, !=0 on failure.
 */

extern int fels_log(fels_category_t category, fels_log_level_t log_level,
		fels_error_code_t error_code, int param1, int param2, int param3, int param4);

/**
 * logging function to drivers to have full debug information
 * @param category fels_category_t enum
 * to determine the log from which driver
 * @param log_level fels_log_level_t enum
 * to know the type of log_level
 * @param debug_msg debug message string to be logged
 * @return 0 on success, !=0 on failure.
 */
extern int fels_debug_log(fels_category_t category, fels_log_level_t log_level,
		const char *debug_msg);

static inline bool fels_category_validate(fels_category_t category)
{
	if (category > FELS_CATEGORY_MIN && category < FELS_CATEGORY_MAX)
		return true;
	else
		return false;
}

static inline bool fels_log_level_validate(fels_log_level_t log_level)
{
	switch (log_level) {
	case FELS_LOG_LEVEL_INFO:
	case FELS_LOG_LEVEL_WARNING:
	case FELS_LOG_LEVEL_ERROR:
	return true;
	}
	return false;
}

static inline bool fels_error_code_validate(fels_error_code_t error_code)
{
	if (error_code >= FELS_ERROR_CODE_MIN &&
			error_code < FELS_ERROR_CODE_MAX)
		return true;
	return false;
}
#endif /* FELS_H */
