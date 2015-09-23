#ifndef __INCLUDE_LINUX_OOM_H
#define __INCLUDE_LINUX_OOM_H

/*
 * /proc/<pid>/oom_adj is deprecated, see
 * Documentation/feature-removal-schedule.txt.
 *
 * /proc/<pid>/oom_adj set to -17 protects from the oom-killer
 */
#define OOM_DISABLE (-17)
/* inclusive */
#define OOM_ADJUST_MIN (-16)
#define OOM_ADJUST_MAX 15

/*
 * /proc/<pid>/oom_score_adj set to OOM_SCORE_ADJ_MIN disables oom killing for
 * pid.
 */
#define OOM_SCORE_ADJ_MIN	(-1000)
#define OOM_SCORE_ADJ_MAX	1000

#ifdef __KERNEL__

#include <linux/sched.h>
#include <linux/types.h>
#include <linux/nodemask.h>

struct zonelist;
struct notifier_block;
struct mem_cgroup;
struct task_struct;

/*
 * Types of limitations to the nodes from which allocations may occur
 */
enum oom_constraint {
	CONSTRAINT_NONE,
	CONSTRAINT_CPUSET,
	CONSTRAINT_MEMORY_POLICY,
	CONSTRAINT_MEMCG,
};

/* Thread is the potential origin of an oom condition; kill first on oom */
#define OOM_FLAG_ORIGIN		((__force oom_flags_t)0x1)

static inline void set_current_oom_origin(void)
{
	current->signal->oom_flags |= OOM_FLAG_ORIGIN;
}

static inline void clear_current_oom_origin(void)
{
	current->signal->oom_flags &= ~OOM_FLAG_ORIGIN;
}

static inline bool oom_task_origin(const struct task_struct *p)
{
	return !!(p->signal->oom_flags & OOM_FLAG_ORIGIN);
}

extern unsigned int oom_badness(struct task_struct *p, struct mem_cgroup *memcg,
			const nodemask_t *nodemask, unsigned long totalpages);
extern int try_set_zonelist_oom(struct zonelist *zonelist, gfp_t gfp_flags);
extern void clear_zonelist_oom(struct zonelist *zonelist, gfp_t gfp_flags);

extern void out_of_memory(struct zonelist *zonelist, gfp_t gfp_mask,
		int order, nodemask_t *mask, bool force_kill);
extern int register_oom_notifier(struct notifier_block *nb);
extern int unregister_oom_notifier(struct notifier_block *nb);

extern bool oom_killer_disabled;

static inline void oom_killer_disable(void)
{
	oom_killer_disabled = true;
}

static inline void oom_killer_enable(void)
{
	oom_killer_disabled = false;
}

extern struct task_struct *find_lock_task_mm(struct task_struct *p);

/* sysctls */
extern int sysctl_oom_dump_tasks;
extern int sysctl_oom_kill_allocating_task;
extern int sysctl_panic_on_oom;
#endif /* __KERNEL__*/
#endif /* _INCLUDE_LINUX_OOM_H */
