/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
/*
 * Copyright (C) 2014 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#define DEBUG

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/limits.h>
#include <linux/fs.h>
#include <asm/siginfo.h>
#include <linux/highmem.h>
#include <linux/pagemap.h>
#include "avc_ss.h"
#include "trap.h"

extern void trapped_node_entry(pid_t, char *);

#define	STRING_ALL_LEN_MAX	PATH_MAX
#define	STRING_PART_LEN_MAX	PATH_MAX

enum string_lsm_audit_data {
	STRING_LSM_AUDIT_DATA_TERM = 0,
	STRING_LSM_AUDIT_DATA_ACTION,
	STRING_LSM_AUDIT_DATA_PID,
	STRING_LSM_AUDIT_DATA_NONE,
	STRING_LSM_AUDIT_DATA_IPC,
	STRING_LSM_AUDIT_DATA_CAP,
	STRING_LSM_AUDIT_DATA_PATH,
	STRING_LSM_AUDIT_DATA_DENTRY,
	STRING_LSM_AUDIT_DATA_INODE,
	STRING_LSM_AUDIT_DATA_TASK,
	STRING_LSM_AUDIT_DATA_NET,
	STRING_LSM_AUDIT_DATA_KEY,
	STRING_LSM_AUDIT_DATA_KMOD,
	STRING_LSM_AUDIT_DATA_TCONTEXT,
	STRING_LSM_AUDIT_DATA_SCONTEXT,
	STRING_LSM_AUDIT_DATA_TCLASS,
	STRING_LSM_AUDIT_DATA_PERMISSIVE,
	STRING_LSM_AUDIT_DATA_MAX
};

int selinux_trap_enable;
int selinux_trap_debug;
struct selinux_trap_list selinux_trap_list_head;
struct selinux_trap_process_list selinux_trap_process_list_head;
struct semaphore selinux_trap_list_sem;
static char string_work[STRING_PART_LEN_MAX];
static const char copy_table[] = {
	STRING_LSM_AUDIT_DATA_ACTION,	/* action */
	STRING_LSM_AUDIT_DATA_PID,		/* pid comm */
	STRING_LSM_AUDIT_DATA_INODE,	/* name dev ino */
	STRING_LSM_AUDIT_DATA_TCONTEXT,	/* scontext */
	STRING_LSM_AUDIT_DATA_SCONTEXT,	/* tcontext */
	STRING_LSM_AUDIT_DATA_TCLASS,	/* tclass */
	STRING_LSM_AUDIT_DATA_PERMISSIVE,	/* permissive ppid pgid pgcomm */
	STRING_LSM_AUDIT_DATA_TERM		/* TERM */
};

static int cmp_string(char *rule_str, const char *cmp_str, size_t len)
{
	size_t rule_str_len = strlen(rule_str);
	if (rule_str[rule_str_len-1] == '*')
		return strncmp(rule_str, cmp_str, rule_str_len-1);	/* Wild card match ex) cmpStr = "*" , "xxxxx*" or etc... */
	return strncmp(rule_str, cmp_str, len);			/* Exact match ex) ruleStr = "xxxxx" cmpStr ="xxxxx" */
}

/* return 0 = identical / not 0 = different */
static int cmp_polarity(char *rule, int c)
{
	int rc;

	if (!rule) {
		trap_devel_log("SELinux: trap: no rule for name\n");
		return 1;	/* Only + or - string / not NULL */
	}

	trap_devel_log("SELinux: trap: compare rule '%s' with polarity '%c'\n",
		rule, c);

	if (rule[0] == c)
		rc = 0;		/* Matched */
	else
		rc = 1;		/* Unmatched */

	if (rc)
		trap_devel_log("SELinux: trap: different\n");
	else
		trap_devel_log("SELinux: trap: identical\n");

	return rc;
}

/* return 0 = identical / not 0 = different */
static int cmp_scontext(char *rule, struct common_audit_data *ad)
{
	int rc;
	char *scontext;
	u32 scontext_len;
	u32 ssid = ad->selinux_audit_data->ssid;

	if (!rule) {
		trap_devel_log("SELinux: trap: no rule for scontext\n");
		return 0;
	}
	rc = security_sid_to_context(ssid, &scontext, &scontext_len);
	if (rc) {
		pr_err("SELinux: trap: error context for ssid:%d\n",
			ssid);
		return 0;
	}

	trap_devel_log("SELinux: trap: compare rule '%s' with scontext '%s'\n",
		rule, scontext);

	rc = cmp_string(rule, scontext, scontext_len);
	if (rc)
		trap_devel_log("SELinux: trap: different\n");
	else
		trap_devel_log("SELinux: trap: identical\n");

	kfree(scontext);
	return rc;
}

/* return 0 = identical / not 0 = different */
static int cmp_tcontext(char *rule, struct common_audit_data *ad)
{
	int rc;
	char *tcontext;
	u32 tcontext_len;
	u32 tsid = ad->selinux_audit_data->tsid;

	if (!rule) {
		trap_devel_log("SELinux: trap: no rule for tcontext\n");
		return 0;
	}

	rc = security_sid_to_context(tsid, &tcontext, &tcontext_len);
	if (rc) {
		pr_err("SELinux: trap: error context for tsid:%d\n",
			tsid);
		return 0;
	}

	trap_devel_log("SELinux: trap: compare rule '%s' with tcontext '%s'\n",
		rule, tcontext);

	rc = cmp_string(rule, tcontext, tcontext_len);
	if (rc)
		trap_devel_log("SELinux: trap: different\n");
	else
		trap_devel_log("SELinux: trap: identical\n");

	kfree(tcontext);
	return rc;
}

/* return 0 = identical / not 0 = different */
static int cmp_tclass(char *rule, struct common_audit_data *ad)
{
	int rc;
	u16 tclass = ad->selinux_audit_data->tclass;

	if (!rule) {
		trap_devel_log("SELinux: trap: no rule for tclass\n");
		return 0;
	}

	BUG_ON(tclass >= secclass_map_size);

	trap_devel_log("SELinux: trap: compare rule '%s' with tclass '%s'\n",
		rule, secclass_map[tclass-1].name);

	rc = cmp_string(rule, secclass_map[tclass-1].name,
		strlen(secclass_map[tclass-1].name));

	if (rc)
		trap_devel_log("SELinux: trap: different\n");
	else
		trap_devel_log("SELinux: trap: identical\n");

	return rc;
}

/* return 0 = identical / not 0 = different */
static int get_pname(struct task_struct *task, char *zeroed_page)
{
	unsigned int arg_len;
	struct mm_struct *mm = get_task_mm(task);
	struct vm_area_struct *vma;
	int offset;
	void *maddr;
	struct page *page = NULL;

	if (!mm)
		return 0;
	if (!mm->arg_end) {
		mmput(mm);
		return 0;
	}
	arg_len = mm->arg_end - mm->arg_start;

	if (get_user_pages(task, mm, mm->arg_start, 1, 0, 1, &page, &vma) != 1) {
		mmput(mm);
		return 0;
	}

	offset = mm->arg_start & (PAGE_SIZE-1);
	if (arg_len > PAGE_SIZE-offset)
		arg_len = PAGE_SIZE-offset;
	maddr = kmap(page);
	copy_from_user_page(vma, page, mm->arg_start, zeroed_page,
		maddr + offset, arg_len);
	kunmap(page);
	page_cache_release(page);

	mmput(mm);
	return 1;
}

/* return 0 = identical / not 0 = different */
static int cmp_pname(char *rule, struct common_audit_data *ad)
{
	int rc;
	char *pname = NULL;

	if (!rule) {
		trap_devel_log("SELinux: trap: no rule for pname\n");
		return 0;
	}

	pname = (char *)get_zeroed_page(GFP_KERNEL);
	if (!pname) {
		pr_err("SELinux: trap: get_zeroed_page failed\n");
		return 0;
	}

	rc = get_pname(current, pname);

	if (rc <= 0) {
		pr_err("SELinux: trap: get_pname failed\n");
		free_page((unsigned long) pname);
		return 0;
	}

	trap_devel_log("SELinux: trap: compare rule '%s' with pname '%s'\n",
		rule, pname);

	rc = cmp_string(rule, pname, strlen(pname));
	if (rc)
		trap_devel_log("SELinux: trap: different\n");
	else
		trap_devel_log("SELinux: trap: identical\n");

	free_page((unsigned long) pname);
	return rc;
}

/* return 0 = identical / not 0 = different */
static int cmp_pname_parent(char *rule, struct common_audit_data *ad)
{
	int rc;
	char *pname = NULL;

	if (!rule) {
		trap_devel_log("SELinux: trap: no rule for pname_parent\n");
		return 0;
	}

	pname = (char *)get_zeroed_page(GFP_KERNEL);
	if (!pname) {
		pr_err("SELinux: trap: get_zeroed_page failed\n");
		return 0;
	}

	rc = get_pname(current->parent, pname);

	if (rc <= 0) {
		pr_err("SELinux: trap: get_pname(parent) failed\n");
		free_page((unsigned long) pname);
		return 0;
	}

	trap_devel_log("SELinux: trap: compare rule '%s' with pname_parent '%s'\n",
		rule, pname);

	rc = cmp_string(rule, pname, strlen(pname));
	if (rc)
		trap_devel_log("SELinux: trap: different\n");
	else
		trap_devel_log("SELinux: trap: identical\n");

	free_page((unsigned long) pname);
	return rc;
}

# if 0 /* XXX: temporarily disabled to check process group leader */
/* return 0 = identical / not 0 = different */
static int cmp_pname_pgl(char *rule, struct common_audit_data *ad)
{
	int rc;
	char *pname = NULL;

	if (!rule) {
		trap_devel_log("SELinux: trap: no rule for pname_pgl\n");
		return 0;
	}

	pname = (char *)get_zeroed_page(GFP_KERNEL);
	if (!pname) {
		pr_err("SELinux: trap: get_zeroed_page failed\n");
		return 0;
	}

	if (current->group_leader->pid != current->pid) {
		rc = get_pname(current->group_leader, pname);
	} else {
		rc = get_pname(current->parent->group_leader, pname);
	}

	if (rc <= 0) {
		pr_err("SELinux: trap: get_pname(pgl) failed\n");
		free_page((unsigned long) pname);
		return 0;
	}

	trap_devel_log("SELinux: trap: compare rule '%s' with pname_pgl '%s'\n",
		rule, pname);

	rc = cmp_string(rule, pname, strlen(pname));
	if (rc)
		trap_devel_log("SELinux: trap: different\n");
	else
		trap_devel_log("SELinux: trap: identical\n");

	free_page((unsigned long) pname);
	return rc;
}
#endif

/* return 0 = identical / not 0 = different */
static int cmp_path(char *rule, struct common_audit_data *ad)
{
	int rc;
	struct path *path = &ad->u.path;
	char *path_str, *pathname;

	if (!rule) {
		trap_devel_log("SELinux: trap: no rule for path\n");
		return 0;
	}

	if (ad->type != LSM_AUDIT_DATA_PATH) {
		trap_devel_log("SELinux: trap: not support for path\n");
		return 0;
	}

	pathname = kzalloc(PATH_MAX+11, GFP_KERNEL);
	if (!pathname) {
		pr_err("SELinux: trap: kzalloc failed\n");
		return 0;
	}

	path_str = d_path(path, pathname, PATH_MAX+11);
	if (IS_ERR(path_str)) {
		kfree(pathname);
		pathname = NULL;
		pr_err("SELinux: trap: too long path name\n");
		return 0;
	}

	trap_devel_log("SELinux: trap: compare rule '%s' with path '%s'\n",
		rule, path_str);

	rc = cmp_string(rule, path_str, strlen(path_str));

	if (rc)
		trap_devel_log("SELinux: trap: different\n");
	else
		trap_devel_log("SELinux: trap: identical\n");

	kfree(pathname);
	pathname = NULL;
	return rc;
}

/* return 0 = identical / not 0 = different */
static int cmp_name(char *rule, struct common_audit_data *ad)
{
	int rc;
	const char *name;

	if (!rule) {
		trap_devel_log("SELinux: trap: no rule for name\n");
		return 0;
	}

	if (ad->type != LSM_AUDIT_DATA_DENTRY) {
		trap_devel_log("SELinux: trap: not support for name\n");
		return 0;
	}

	name = ad->u.dentry->d_name.name;

	trap_devel_log("SELinux: trap: compare rule '%s' with name '%s'\n",
		rule, name);

	rc = cmp_string(rule, name, strlen(name));

	if (rc)
		trap_devel_log("SELinux: trap: different\n");
	else
		trap_devel_log("SELinux: trap: identical\n");

	return rc;
}

/* return 0 = identical / not 0 = different */
static int cmp_action(char *rule, struct common_audit_data *ad)
{
	int rc;
	const char **perms;
	int i, perm;
	u16 tclass = ad->selinux_audit_data->tclass;
	u32 av = ad->selinux_audit_data->audited;
	u32 av_rule = 0;
	char *temp;
	char *token;

	if (!rule) {
		trap_devel_log("SELinux: trap: no rule for action\n");
		return 0;
	}

	if (av == 0)
		return 0;

	temp = kmalloc(strlen(rule)+1, GFP_KERNEL);
	if (!temp) {
		pr_err("SELinux: trap: kzalloc failed\n");
		return 0;
	}
	strlcpy(temp, rule, strlen(rule)+1);

	trap_devel_log("SELinux: trap: compare rule '%s' with action [",
		rule);

	perms = secclass_map[tclass-1].perms;

	i = 0;
	perm = 1;
	while (i < (sizeof(av) * 8)) {
		if ((perm & av) && perms[i])
			trap_devel_log("%s", perms[i]);
		i++;
		perm <<= 1;
	}
	trap_devel_log("]\n");

	while ((token = strsep(&temp, " ")) != NULL) {
		int len = strlen(token);
		if (len > 0) {
			i = 0;
			while (perms[i]) {
				trap_devel_log("compare %s - %s\n", token, perms[i]);
				if (!strncmp(token, perms[i], len)) {
					trap_devel_log("match\n");
					av_rule |= (1 << i);
					break;
				}
				i++;
			}
		}
	}
	kfree(temp);

	trap_devel_log("av 0x%x\n", av);		/* audited value */
	trap_devel_log("av_rule 0x%x\n", av_rule);	/* result: string matched position */

	rc = av & ~av_rule;
	if (rc)
		trap_devel_log("SELinux: trap: different\n");
	else
		trap_devel_log("SELinux: trap: identical\n");

	return rc;
}

/* param in ad ... check item */
/* return 0 = blacklist matched or alllist unmatched / 1 = whitelist matched  */
static int mask_trap(struct common_audit_data *ad)
{
	int ret = 0;
	struct selinux_trap_list *entry;
	rcu_read_lock();
	/* Blacklist matching */
	list_for_each_entry_rcu(entry, &selinux_trap_list_head.list, list) {
		if (cmp_polarity(
				entry->item_array[TRAP_MASK_TYPE_POLARITY],
				'-'))
			continue;	/* different to next rule */
		else if (cmp_scontext(
				entry->item_array[TRAP_MASK_TYPE_SCONTEXT],
				ad))
			continue;	/* different to next rule */
		else if (cmp_tcontext(
				entry->item_array[TRAP_MASK_TYPE_TCONTEXT],
				ad))
			continue;	/* different to next rule */
		else if (cmp_tclass(
				entry->item_array[TRAP_MASK_TYPE_TCLASS],
				ad))
			continue;	/* different to next rule */
		else if (cmp_pname(entry->item_array[TRAP_MASK_TYPE_PNAME], ad))
			continue;	/* different to next rule */
		else if (cmp_pname_parent(
				entry->item_array[TRAP_MASK_TYPE_PNAME_PARENT],
				ad))
			continue;	/* different to next rule */
#if 0 /* XXX: temporarily disabled to check process group leader */
		else if (cmp_pname_pgl(
				entry->item_array[TRAP_MASK_TYPE_PNAME_PGL],
				ad))
			continue;	/* different to next rule */
#endif
		else if (cmp_path(
				entry->item_array[TRAP_MASK_TYPE_PATH],
				ad))
			continue;	/* different to next rule */
		else if (cmp_name(
				entry->item_array[TRAP_MASK_TYPE_NAME],
				ad))
			continue;	/* different to next rule */
		else if (cmp_action(
				entry->item_array[TRAP_MASK_TYPE_ACTION],
				ad))
			continue;	/* different to next rule */
		/* Rule matched. It requires forcible crash. */
		ret = 0;
		goto out;
	}
	/* Whitelist matching */
	list_for_each_entry_rcu(entry, &selinux_trap_list_head.list, list) {
		if (cmp_polarity(
				entry->item_array[TRAP_MASK_TYPE_POLARITY],
				'+'))
			continue;	/* different to next rule */
		else if (cmp_scontext(
				entry->item_array[TRAP_MASK_TYPE_SCONTEXT],
				ad))
			continue;	/* different to next rule */
		else if (cmp_tcontext(
				entry->item_array[TRAP_MASK_TYPE_TCONTEXT],
				ad))
			continue;	/* different to next rule */
		else if (cmp_tclass(
				entry->item_array[TRAP_MASK_TYPE_TCLASS],
				ad))
			continue;	/* different to next rule */
		else if (cmp_pname(entry->item_array[TRAP_MASK_TYPE_PNAME], ad))
			continue;	/* different to next rule */
		else if (cmp_pname_parent(
				entry->item_array[TRAP_MASK_TYPE_PNAME_PARENT],
				ad))
			continue;	/* different to next rule */
#if 0 /* XXX: temporarily disabled to check process group leader */
		else if (cmp_pname_pgl(
				entry->item_array[TRAP_MASK_TYPE_PNAME_PGL],
				ad))
			continue;	/* different to next rule */
#endif
		else if (cmp_path(
				entry->item_array[TRAP_MASK_TYPE_PATH],
				ad))
			continue;	/* different to next rule */
		else if (cmp_name(
				entry->item_array[TRAP_MASK_TYPE_NAME],
				ad))
			continue;	/* different to next rule */
		else if (cmp_action(
				entry->item_array[TRAP_MASK_TYPE_ACTION],
				ad))
			continue;	/* different to next rule */
		/* Rule matched. It's exception. */
		ret = 1;
		goto out;
	}
	/* All list not matched ..."ret = 0" */
out:
	rcu_read_unlock();
	return ret;
}

static void task_killer(struct task_struct *task)
{
	int ret;
	struct siginfo info;

	memset(&info, 0, sizeof(struct siginfo));
	info.si_signo = SIGABRT;
	info.si_code = SI_KERNEL;
	pr_info("SELinux: trap: send signal to pid:%d.\n", task->pid);
	ret = send_sig_info(SIGABRT, &info, task);
	if (ret < 0)
		pr_err("SELinux: trap: send_sig_info failed\n");
}

static void dump_common_audit_data_part(struct common_audit_data *ad, char type, char string[], int len, int maxlen)
{
	struct task_struct *tsk = current;

	/* string work Initialize */
	string_work[0] = '\0';

	switch (type) {
	case STRING_LSM_AUDIT_DATA_ACTION: {
		const char **perms;
		int i, perm;
		u16 tclass = ad->selinux_audit_data->tclass;
		u32 av = ad->selinux_audit_data->audited;

		if (av == 0)
			break;
		perms = secclass_map[tclass-1].perms;
		i = 0;
		perm = 1;
		while (i < (sizeof(av) * 8)) {
			if ((perm & av) && perms[i])
				snprintf(&string_work[strlen(string_work)], STRING_PART_LEN_MAX-strlen(string_work), " { %s }", perms[i]);
			i++;
			perm <<= 1;
		}
		break;
	}
	case STRING_LSM_AUDIT_DATA_PID:
		if (tsk && tsk->pid) {
			snprintf(&string_work[strlen(string_work)], STRING_PART_LEN_MAX-strlen(string_work), " pid=%d comm=\"%s\"", tsk->pid, tsk->comm);
		}
		break;
	case STRING_LSM_AUDIT_DATA_NONE:
		break;
	case STRING_LSM_AUDIT_DATA_IPC:
		snprintf(&string_work[strlen(string_work)], STRING_PART_LEN_MAX-strlen(string_work), " key=%d ", ad->u.ipc_id);
		break;
	case STRING_LSM_AUDIT_DATA_CAP:
		snprintf(&string_work[strlen(string_work)], STRING_PART_LEN_MAX-strlen(string_work), " capability=%d ", ad->u.cap);
		break;
	case STRING_LSM_AUDIT_DATA_PATH: {
		struct inode *inode;
		if (ad->type != LSM_AUDIT_DATA_PATH)
			break;
		snprintf(&string_work[strlen(string_work)], STRING_PART_LEN_MAX-strlen(string_work), " path=%s", ad->u.path.dentry->d_iname);
		inode = ad->u.path.dentry->d_inode;
		if (inode) {
			snprintf(&string_work[strlen(string_work)], STRING_PART_LEN_MAX-strlen(string_work), " dev=\"%s\"", inode->i_sb->s_id);
			snprintf(&string_work[strlen(string_work)], STRING_PART_LEN_MAX-strlen(string_work), " ino=%lu", inode->i_ino);
		}
		break;
	}
	case STRING_LSM_AUDIT_DATA_DENTRY: {
		struct inode *inode;
		if (ad->type != LSM_AUDIT_DATA_DENTRY)
			break;
		snprintf(string_work, STRING_PART_LEN_MAX, " name=%s", ad->u.dentry->d_name.name);
		inode = ad->u.dentry->d_inode;
		if (inode) {
			snprintf(&string_work[strlen(string_work)], STRING_PART_LEN_MAX-strlen(string_work), " dev=\"%s\"", inode->i_sb->s_id);
			snprintf(&string_work[strlen(string_work)], STRING_PART_LEN_MAX-strlen(string_work), " ino=%lu", inode->i_ino);
		}
		break;
	}
	case STRING_LSM_AUDIT_DATA_INODE:
		break;

	case STRING_LSM_AUDIT_DATA_TASK:
		tsk = ad->u.tsk;
		if (tsk && tsk->pid) {
			snprintf(&string_work[strlen(string_work)], STRING_PART_LEN_MAX-strlen(string_work), " pid=%d comm=\"%s\"", tsk->pid, tsk->comm);
		}
		break;
	case STRING_LSM_AUDIT_DATA_NET:
		break;
#ifdef CONFIG_KEYS
	case STRING_LSM_AUDIT_DATA_KEY:
		snprintf(&string_work[strlen(string_work)], STRING_PART_LEN_MAX-strlen(string_work), " key_serial=%u", ad->u.key_struct.key);
		if (ad->u.key_struct.key_desc) {
			snprintf(&string_work[strlen(string_work)], STRING_PART_LEN_MAX-strlen(string_work), " key_desc=%s", ad->u.key_struct.key_desc);
		}
		break;
#endif
	case STRING_LSM_AUDIT_DATA_KMOD:
		snprintf(&string_work[strlen(string_work)], STRING_PART_LEN_MAX-strlen(string_work), " kmod=%s", ad->u.kmod_name);
		break;

	case STRING_LSM_AUDIT_DATA_SCONTEXT: {
		int rc;
		char *scontext;
		u32 scontext_len;
		u32 ssid = ad->selinux_audit_data->ssid;
		rc = security_sid_to_context(ssid, &scontext, &scontext_len);
		if (rc) {
			break;
		}
		snprintf(&string_work[strlen(string_work)], STRING_PART_LEN_MAX-strlen(string_work), " scontext=%s", scontext);
		kfree(scontext);
		break;
	}
	case STRING_LSM_AUDIT_DATA_TCONTEXT: {
		int rc;
		char *tcontext;
		u32 tcontext_len;
		u32 tsid = ad->selinux_audit_data->tsid;
		rc = security_sid_to_context(tsid, &tcontext, &tcontext_len);
		if (rc) {
			break;
		}
		snprintf(&string_work[strlen(string_work)], STRING_PART_LEN_MAX-strlen(string_work), " tcontext=%s", tcontext);
		kfree(tcontext);
		break;
	}
	case STRING_LSM_AUDIT_DATA_TCLASS: {
		u16 tclass = ad->selinux_audit_data->tclass;

		BUG_ON(tclass >= secclass_map_size);

		snprintf(&string_work[strlen(string_work)], STRING_PART_LEN_MAX-strlen(string_work), " tclass=%s", secclass_map[tclass-1].name);
		break;
	}
	case STRING_LSM_AUDIT_DATA_PERMISSIVE:
		snprintf(&string_work[strlen(string_work)], STRING_PART_LEN_MAX-strlen(string_work), " permissive=%u", ad->selinux_audit_data->result ? 0 : 1);
		if (tsk && tsk->pid) {
			snprintf(&string_work[strlen(string_work)], STRING_PART_LEN_MAX-strlen(string_work), " ppid=%d pcomm=\"%s\"", tsk->parent->pid, tsk->parent->comm);
			if (tsk->group_leader->pid != tsk->pid) {
				snprintf(&string_work[strlen(string_work)], STRING_PART_LEN_MAX-strlen(string_work), " pgid=%d pgcomm=\"%s\"", tsk->group_leader->pid, tsk->group_leader->comm);
			} else if (tsk->parent->group_leader->pid) {
				snprintf(&string_work[strlen(string_work)], STRING_PART_LEN_MAX-strlen(string_work), " pgid=%d pgcomm=\"%s\"", tsk->parent->group_leader->pid, tsk->parent->group_leader->comm);
			}
		}
		break;
	} /* switch (ad->type) */
	/* parent buff size check */
	if (strlen(string_work) < maxlen - len) {
		/* parent buff copy */
		strlcpy(&string[len], string_work, maxlen - len);
	}
}

static char *dump_audit_data(struct common_audit_data *ad)
{
	char *string = NULL;
	int i;

	string = kmalloc(STRING_ALL_LEN_MAX, GFP_KERNEL);
	if (!string) {
		/* error */
		return NULL;
	}

	/* string work Initialize */
	string[0] = '\0';

	for (i = 0; STRING_LSM_AUDIT_DATA_TERM != copy_table[i]; i++) {
		dump_common_audit_data_part(ad, copy_table[i], string, strlen(string), STRING_ALL_LEN_MAX);
	}
	return string;
}

static void trapped_node_update(struct common_audit_data *ad)
{
	char *string = NULL;
	struct task_struct *tsk = current;

	rcu_read_lock();

	string = dump_audit_data(ad);
	if (string) {
		/* kmsg output */
		pr_info("SELinux: trap: caught%s", string);
		/* Node Entry */
		trapped_node_entry(tsk->pid, string);
	}

	rcu_read_unlock();
}

void trap_selinux_error(struct common_audit_data *ad)
{
	if (!selinux_trap_enable)
		return;

	/* black list and white list check */
	if (mask_trap(ad)) {
		/* Ignore error type( No problem ) */
		if (selinux_trap_debug >= TRAP_LOGLEVEL_NORMAL) {
			pr_info("SELinux: trap: Ignore SELinux violation(pid = %d).\n", ((struct task_struct *)current)->pid);
		}
		return;
	}

	/* trapped Update & kmeg */
	trapped_node_update(ad);

	/* show current call stack of kernel layer */
	pr_info("SELinux: trap: show stack.\n");
	show_stack(NULL, NULL);

	/* sending SIGABRT to myself */
	task_killer(current);
}

static int selinux_trap_init(void)
{
	int ret = 0;
	selinux_trap_debug = TRAP_LOGLEVEL_NORMAL;
	trap_devel_log("SELinux: trap: Init module\n");
	sema_init(&selinux_trap_list_sem, 1);
	INIT_LIST_HEAD(&selinux_trap_list_head.list);
	INIT_LIST_HEAD(&selinux_trap_process_list_head.list);
	return ret;
}

static void selinux_trap_exit(void)
{
	trap_devel_log("SELinux: trap: Exit module\n");
}

module_init(selinux_trap_init);
module_exit(selinux_trap_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Naoya Hirota <naoya.hirota@sonymobile.com>");
MODULE_AUTHOR("Naoya Inoue <naoya.xa.inoue@sonymobile.com>");
MODULE_DESCRIPTION("SELinux error trap extention");
