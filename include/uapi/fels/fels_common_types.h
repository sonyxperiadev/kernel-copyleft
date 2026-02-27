/*
Copyright 2023 Sony Corporation
*/

#ifndef FELS_COMMON_TYPES_H
#define FELS_COMMON_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FELS_CATEGORY_MIN = -1,
    FELS_CATEGORY_AUDIO = 0,
    FELS_CATEGORY_CAMERA = 1,
    FELS_CATEGORY_LINUX_OS = 2,
    FELS_CATEGORY_DISPLAY = 3,
    FELS_CATEGORY_MAX,
} fels_category_t;

typedef enum {
    FELS_ERROR_CODE_MIN = 1000,
    /* [1000-2000) is assigned to LINUX_OS */
    FELS_ERROR_CODE_LINUX_OS_BAD_SECTOR = 1000,
    FELS_ERROR_CODE_LINUX_OS_MMC_RECOVERY_FAILURE = 1001,
    /* [2000-3000) is assigned to DISPLAY */
    FELS_ERROR_CODE_DISPLAY_SHORT = 2000,
    FELS_ERROR_CODE_MAX,
} fels_error_code_t;

typedef enum {
    FELS_LOG_LEVEL_INFO = 0,
    FELS_LOG_LEVEL_WARNING = 1,
    FELS_LOG_LEVEL_ERROR = 2,
} fels_log_level_t;

#ifdef __cplusplus
}
#endif

#endif /* FELS_COMMON_TYPES_H */
