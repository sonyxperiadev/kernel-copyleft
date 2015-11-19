/*
 * kernel/power/autosleep.c
 *
 * Opportunistic sleep support.
 *
 * Copyright (C) 2012 Rafael J. Wysocki <rjw@sisk.pl>
 */

#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/pm_wakeup.h>

#include "power.h"

//CORE-EL-add_auto_sleep_mesg-00+[
#include <linux/module.h>
#include <linux/ktime.h>
#include <linux/rtc.h>

enum {
	DEBUG_USER_STATE = 1U << 0,
	DEBUG_SUSPEND = 1U << 2,
	DEBUG_VERBOSE = 1U << 3,
};
static int debug_mask = DEBUG_USER_STATE;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

void pm_track_msg(bool show_detail_time, char *msg){

	if (show_detail_time) {
		struct timespec ts;
		struct rtc_time tm;
		getnstimeofday(&ts);
		rtc_time_to_tm(ts.tv_sec, &tm);
		printk("[PM]%s %lld"
			"(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n",
			msg, ktime_to_ns(ktime_get()),
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
	}
	else {
		printk("[PM]%s %lld\n",
			msg, ktime_to_ns(ktime_get()));
	}
}

#define PM_TRACK_MSG(show_detail_time, msg, ...)\
do {\
	if (debug_mask & DEBUG_USER_STATE) {\
		char   buf[60];\
		snprintf(buf, sizeof(buf), msg, ##__VA_ARGS__);\
		pm_track_msg(show_detail_time, buf);\
	}\
}while(0)

//CORE-EL-add_auto_sleep_mesg-00+]

static suspend_state_t autosleep_state;
static struct workqueue_struct *autosleep_wq;
/*
 * Note: it is only safe to mutex_lock(&autosleep_lock) if a wakeup_source
 * is active, otherwise a deadlock with try_to_suspend() is possible.
 * Alternatively mutex_lock_interruptible() can be used.  This will then fail
 * if an auto_sleep cycle tries to freeze processes.
 */
static DEFINE_MUTEX(autosleep_lock);
static struct wakeup_source *autosleep_ws;

//CORE-EL-add_auto_sleep_mesg-00+[
static void try_to_suspend(struct work_struct *work)
{
	unsigned int initial_count, final_count;

	PM_TRACK_MSG(true, "%s 0:enter", __func__);

	if (!pm_get_wakeup_count(&initial_count, true)) {
		PM_TRACK_MSG(false, "%s 1:pm_get_wakeup_count", __func__);
		goto out;
	}

	mutex_lock(&autosleep_lock);

	if (!pm_save_wakeup_count(initial_count) ||
		system_state != SYSTEM_RUNNING) {
		mutex_unlock(&autosleep_lock);
		PM_TRACK_MSG(false, "%s 2:pm_save_wakeup_count", __func__);
		goto out;
	}

	if (autosleep_state == PM_SUSPEND_ON) {
		mutex_unlock(&autosleep_lock);

		PM_TRACK_MSG(false, "%s 3:quit autosleep_state == PM_SUSPEND_ON", __func__);
		return;
	}
	if (autosleep_state >= PM_SUSPEND_MAX)
		hibernate();
	else {
		PM_TRACK_MSG(true, "%s 4:calling pm_suspend", __func__);
		pm_suspend(autosleep_state);
	}

	mutex_unlock(&autosleep_lock);

	if (!pm_get_wakeup_count(&final_count, false)) {
		PM_TRACK_MSG(false, "%s 5:pm_get_wakeup_count", __func__);
		goto out;
	}

	/*
	 * If the wakeup occured for an unknown reason, wait to prevent the
	 * system from trying to suspend and waking up in a tight loop.
	 */
	if (final_count == initial_count) {

		PM_TRACK_MSG(false, "%s 6:final_count == initial_count:%d", __func__, final_count);
		schedule_timeout_uninterruptible(HZ / 2);
	}

 out:
	queue_up_suspend_work();
}
//CORE-EL-add_auto_sleep_mesg-00+]

static DECLARE_WORK(suspend_work, try_to_suspend);

void queue_up_suspend_work(void)
{
	if (autosleep_state > PM_SUSPEND_ON)
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
	
	//CORE-EL-add_auto_sleep_mesg-00+
	PM_TRACK_MSG(true, "%s: %s (%d->%d)",  __func__,
			state != PM_SUSPEND_ON ? "sleep" : "wakeup",
			autosleep_state, state);

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

	autosleep_wq = alloc_ordered_workqueue("autosleep", 0);
	if (autosleep_wq)
		return 0;

	wakeup_source_unregister(autosleep_ws);
	return -ENOMEM;
}
