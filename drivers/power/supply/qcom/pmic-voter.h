/* Copyright (c) 2015-2016 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
/*
 * NOTE: This file has been modified by Sony Mobile Communications Inc.
 * Modifications are Copyright (c) 2017 Sony Mobile Communications Inc,
 * and licensed under the license of the file.
 */

#ifndef __PMIC_VOTER_H
#define __PMIC_VOTER_H

#include <linux/mutex.h>

#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
#define NUM_MAX_CLIENTS	20
#endif
struct votable;

enum votable_type {
	VOTE_MIN,
	VOTE_MAX,
	VOTE_SET_ANY,
	NUM_VOTABLE_TYPES,
};

int get_client_vote(struct votable *votable, const char *client_str);
int get_client_vote_locked(struct votable *votable, const char *client_str);
int get_effective_result(struct votable *votable);
int get_effective_result_locked(struct votable *votable);
const char *get_effective_client(struct votable *votable);
const char *get_effective_client_locked(struct votable *votable);
int vote(struct votable *votable, const char *client_str, bool state, int val);
int rerun_election(struct votable *votable);
struct votable *find_votable(const char *name);
struct votable *create_votable(const char *name,
				int votable_type,
				int (*callback)(struct votable *votable,
						void *data,
						int effective_result,
						const char *effective_client),
				void *data);
void destroy_votable(struct votable *votable);
void lock_votable(struct votable *votable);
void unlock_votable(struct votable *votable);
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
ssize_t somc_output_voter_param(struct votable *votable,
						char *buf, size_t size);
int somc_get_vote_clients(struct votable *votable, char *clients[]);
#endif
#endif /* __PMIC_VOTER_H */
