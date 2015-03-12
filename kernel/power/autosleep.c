/*
 * kernel/power/autosleep.c
 *
 * Opportunistic sleep support.
 *
 * Copyright (C) 2012 Rafael J. Wysocki <rjw@sisk.pl>
 * Copyright (c) 2014 Sony Mobile Communications Inc.
 */

#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/pm_wakeup.h>
#include <linux/wakelock.h>

#include "power.h"

/*
 * suspend back-off default values
 */
#define SBO_SLEEP_MSEC 1100
#define SBO_TIME 10000
#define SBO_CNT 10

static unsigned suspend_short_count;
static suspend_state_t autosleep_state;
static struct workqueue_struct *autosleep_wq;
static struct wake_lock suspend_backoff_lock;

/*
 * Note: it is only safe to mutex_lock(&autosleep_lock) if a wakeup_source
 * is active, otherwise a deadlock with try_to_suspend() is possible.
 * Alternatively mutex_lock_interruptible() can be used.  This will then fail
 * if an auto_sleep cycle tries to freeze processes.
 */
static DEFINE_MUTEX(autosleep_lock);
static struct wakeup_source *autosleep_ws;

static void
suspend_backoff(void)
{
	pr_info("suspend: too many immediate wakeups, back off\n");

	wake_lock_timeout(&suspend_backoff_lock,
		msecs_to_jiffies(SBO_TIME));
}

static void try_to_suspend(struct work_struct *work)
{
	unsigned int initial_count, final_count;
	struct timespec ts_entry, ts_exit;
	u64 elapsed_msecs64;
	u32 elapsed_msecs32;

	if (!pm_get_wakeup_count(&initial_count, true))
		goto out;

	mutex_lock(&autosleep_lock);

	if (!pm_save_wakeup_count(initial_count)) {
		mutex_unlock(&autosleep_lock);
		goto out;
	}

	if (autosleep_state == PM_SUSPEND_ON) {
		mutex_unlock(&autosleep_lock);
		return;
	}
	if (autosleep_state >= PM_SUSPEND_MAX)
		hibernate();
	else {
		getnstimeofday(&ts_entry);
		pm_suspend(autosleep_state);
		getnstimeofday(&ts_exit);

		/*
		 * We want to prevent system from frequent periodic wake-ups
		 * when sleeping time is less or equival certain interval.
		 * It's done in order to save power in certain cases, one of
		 * the examples is GPS tracking, but not only.
		 */
		elapsed_msecs64 = timespec_to_ns(&ts_exit) -
			timespec_to_ns(&ts_entry);
		do_div(elapsed_msecs64, NSEC_PER_MSEC);
		elapsed_msecs32 = elapsed_msecs64;

		if (elapsed_msecs32 <= SBO_SLEEP_MSEC) {
			if (suspend_short_count == SBO_CNT)
				suspend_backoff();
			else
				suspend_short_count++;
		} else {
			suspend_short_count = 0;
		}
	}

	mutex_unlock(&autosleep_lock);

	if (!pm_get_wakeup_count(&final_count, false))
		goto out;

	/*
	 * If the wakeup occured for an unknown reason, wait to prevent the
	 * system from trying to suspend and waking up in a tight loop.
	 */
	if (final_count == initial_count)
		schedule_timeout_uninterruptible(HZ / 2);

 out:
	queue_up_suspend_work();
}

static DECLARE_WORK(suspend_work, try_to_suspend);

void queue_up_suspend_work(void)
{
	if (!work_pending(&suspend_work) && autosleep_state > PM_SUSPEND_ON)
		queue_work(autosleep_wq, &suspend_work);
}

suspend_state_t pm_autosleep_state(void)
{
	return autosleep_state;
}

int pm_autosleep_lock(void)
{
	return mutex_lock_interruptible(&autosleep_lock);
}

void pm_autosleep_unlock(void)
{
	mutex_unlock(&autosleep_lock);
}

int pm_autosleep_set_state(suspend_state_t state)
{

#ifndef CONFIG_HIBERNATION
	if (state >= PM_SUSPEND_MAX)
		return -EINVAL;
#endif

	__pm_stay_awake(autosleep_ws);

	mutex_lock(&autosleep_lock);

	autosleep_state = state;

	__pm_relax(autosleep_ws);

	if (state > PM_SUSPEND_ON) {
		pm_wakep_autosleep_enabled(true);
		queue_up_suspend_work();
	} else {
		pm_wakep_autosleep_enabled(false);
	}

	mutex_unlock(&autosleep_lock);
	return 0;
}

int __init pm_autosleep_init(void)
{
	autosleep_ws = wakeup_source_register("autosleep");
	if (!autosleep_ws)
		return -ENOMEM;

	wake_lock_init(&suspend_backoff_lock, WAKE_LOCK_SUSPEND,
		"suspend_backoff");

	autosleep_wq = alloc_ordered_workqueue("autosleep", 0);
	if (autosleep_wq)
		return 0;

	wake_lock_destroy(&suspend_backoff_lock);
	wakeup_source_unregister(autosleep_ws);
	return -ENOMEM;
}
