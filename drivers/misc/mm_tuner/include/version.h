/***************************************************************************//**
 *
 *  @file		version.h
 *
 *  @brief		define version of mm_tuner driver
 *
 ****************************************************************************//*
 * Copyright (c) 2015 Socionext Inc.
 *******************************************************************************
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 ******************************************************************************/
/*
 * Copyright (C) 2019 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */
/*..+....1....+....2....+....3....+....4....+....5....+....6....+....7....+...*/
#ifndef _MMTUNER_VERSION_H
#define _MMTUNER_VERSION_H

#ifndef MMTUENR_DEVICE
#define MMTUNER_DEVICE	(0x53)	/*!< Target device id. (MN885xx) */
#endif
#ifndef MMTUNER_MAJOR
#define MMTUNER_MAJOR	(0)
#endif
#ifndef MMTUNER_MINOR
#define MMTUNER_MINOR	(4)
#endif
#ifndef MMTUNER_HOTFIX
#define MMTUNER_HOTFIX	(1)
#endif
/*
 * MMTUNER_RC is the release candidate suffix.
 * Should normally be empty.
 */
#ifndef MMTUNER_RC
#define MMTUNER_RC		""
#endif

#ifndef MMTUNER_DESC
#define MMTUNER_DESC		""
#endif

#endif
/*******************************************************************************
 * Copyright (c) 2015 Socionext Inc.
 ******************************************************************************/
