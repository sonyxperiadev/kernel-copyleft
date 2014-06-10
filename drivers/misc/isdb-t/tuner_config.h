/* linux/drivers/misc/isdb-t/tuner_config.h
 *
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * Author: Tatsuya Ooka <Tatsuya.Ooka@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#ifndef _TUNER_CONFIG_H
#define _TUNER_CONFIG_H

#define TUNER_SET_ON                        1    /* Enable setting          */
#define TUNER_SET_OFF                       0    /* Disable setting         */

/* chfile setting 551chfile_xxx.csv */
#define TUNER_CONFIG_PARAM_CH_CHFILE        TUNER_SET_OFF
#define TUNER_CONFIG_PARAM_CH_CHFILE_TMM    TUNER_SET_OFF
#define TUNER_CONFIG_PARAM_CH_CHFILE_TMM_C  TUNER_SET_ON

#define TUNER_CONFIG_TNEND_TIME       3000000    /* Tuning check time [3s]    */
#define TUNER_CONFIG_CHSRCH_TIME      5000000    /* CH search check time [5s] */

/* BER/PER calculation precision */
#define TUNER_CONFIG_PRECISION         100000    /* Range : 100 ~ 100000  */

#define TUNER_CONFIG_FLG_NUM                7    /* Number of error flags */

#define TUNER_CONFIG_DEVFILE   "/dev/mmtuner"    /* Device file */

#define TUNER_CONFIG_INI_SET_D_LINES_PSEQ  2000  /* PSEQ data max line number */
#define TUNER_CONFIG_INI_SET_D_LINES_TNCTL 6000  /* TNCTL data max line number*/

#endif/* _TUNER_CONFIG_H */
