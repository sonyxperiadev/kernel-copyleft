#ifndef __MHL_COMMON_H
#define __MHL_COMMON_H

#include <linux/types.h>

/*
 * Used in the unit test build
 */
#ifdef UNIT_TEST
#define static
#endif


/*
 * Debug print
 */
/* print all debug info */
/* #define DEBUG_PRINT */

#ifdef DEBUG_PRINT
#undef pr_debug
#define pr_debug pr_info
#endif

/* edid read printing with pr_info level*/
#define EDID_DATA_DEBUG_PRINT

/*
 * FIXME
 */
#define ROGUE_WORKAROUND
#define ROGUE_WORKAROUND2
#define EMSC_WORKAROUND
/*
 * other
 */
#define MHL_UCHAR_MAX 255

#define SI_PACK_THIS_STRUCT __attribute__((__packed__))

enum {
	MHL_SUCCESS = 0,
	MHL_FAIL = -1
};

#endif
