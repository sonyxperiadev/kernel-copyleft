/*
 * NOTE: This file has been modified by Sony Mobile Communications Inc.
 * Modifications are Copyright (c) 2016 Sony Mobile Communications Inc,
 * and licensed under the license of the file.
 */
// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2010-2019, The Linux Foundation. All rights reserved.
 */
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/devfreq.h>
#include <linux/math64.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/ftrace.h>
#include <linux/mm.h>
#include <linux/msm_adreno_devfreq.h>
#include <asm/cacheflush.h>
#include <soc/qcom/scm.h>
#include <soc/qcom/qtee_shmbridge.h>
#include <linux/of_platform.h>
#include "governor.h"

static DEFINE_SPINLOCK(tz_lock);
static DEFINE_SPINLOCK(sample_lock);
static DEFINE_SPINLOCK(sample_load_lock);
static DEFINE_SPINLOCK(suspend_lock);
/*
 * FLOOR is 5msec to capture up to 3 re-draws
 * per frame for 60fps content.
 */
#define FLOOR		        5000
/*
 * MIN_BUSY is 1 msec for the sample to be sent
 */
#define MIN_BUSY		1000
#define MAX_TZ_VERSION		0

/*
 * CEILING is 50msec, larger than any standard
 * frame length, but less than the idle timer.
 */
#define CEILING			50000
#define TZ_RESET_ID		0x3
#define TZ_UPDATE_ID		0x4
#define TZ_INIT_ID		0x6

#define TZ_RESET_ID_64          0x7
#define TZ_UPDATE_ID_64         0x8
#define TZ_INIT_ID_64           0x9

#define TZ_V2_UPDATE_ID_64         0xA
#define TZ_V2_INIT_ID_64           0xB
#define TZ_V2_INIT_CA_ID_64        0xC
#define TZ_V2_UPDATE_WITH_CA_ID_64 0xD

#define TAG "msm_adreno_tz: "

#define USEC_PER_MINUTE (1*60*1000*1000)
#define NMAX (15*60*60+1)

struct gpu_load_data {
	unsigned long total_time;
	unsigned long busy_time;
	u64 update_time;
};

struct gpu_load_queue {
	struct gpu_load_data *gpu_load;
	int head;
	int tail;
};

static u64 suspend_time, suspend_time_idd;
static u64 suspend_start, suspend_start_idd;
static unsigned long acc_total, acc_relative_busy;
static unsigned long gpu_load_total, gpu_load_rel_busy;
static struct gpu_load_queue *gpu_load_infos;

static struct msm_adreno_extended_profile *partner_gpu_profile;
static void do_partner_start_event(struct work_struct *work);
static void do_partner_stop_event(struct work_struct *work);
static void do_partner_suspend_event(struct work_struct *work);
static void do_partner_resume_event(struct work_struct *work);

static struct workqueue_struct *workqueue;

/*
 * Returns GPU suspend time in millisecond.
 */
u64 suspend_time_ms(void)
{
	u64 suspend_sampling_time;
	u64 time_diff = 0;

	if (suspend_start == 0)
		return 0;

	suspend_sampling_time = (u64)ktime_to_ms(ktime_get());
	time_diff = suspend_sampling_time - suspend_start;
	/* Update the suspend_start sample again */
	suspend_start = suspend_sampling_time;
	return time_diff;
}

u64 suspend_time_ms_idd(void)
{
	u64 suspend_sampling_time;
	u64 time_diff = 0;

	if (suspend_start_idd == 0)
		return 0;

	suspend_sampling_time = (u64)ktime_to_ms(ktime_get());
	time_diff = suspend_sampling_time - suspend_start_idd;
	/* Update the suspend_start sample again */
	suspend_start_idd = suspend_sampling_time;
	return time_diff;
}

static ssize_t gpu_load_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	unsigned long sysfs_busy_perc = 0;
	/*
	 * Average out the samples taken since last read
	 * This will keep the average value in sync with
	 * with the client sampling duration.
	 */
	spin_lock(&sample_lock);
	if (acc_total)
		sysfs_busy_perc = (acc_relative_busy * 100) / acc_total;

	/* Reset the parameters */
	acc_total = 0;
	acc_relative_busy = 0;
	spin_unlock(&sample_lock);
	return snprintf(buf, PAGE_SIZE, "%lu\n", sysfs_busy_perc);
}

int findItem(int head, int tail, u64 refTime)
{
	int low = head, high = tail, middle;
	u64 update_time;

	while (low < high) {
		middle = (low + high) / 2;
		update_time =
			gpu_load_infos->gpu_load[middle % NMAX].update_time;

		if (update_time <= refTime)
			low = middle + 1;
		else if (update_time > refTime)
			high = middle - 1;
	}

	update_time = gpu_load_infos->gpu_load[low % NMAX].update_time;
	if (update_time > refTime)
		low = low - 1;

	if (low < tail) {
		u64 l = gpu_load_infos->gpu_load[low % NMAX].update_time;
		u64 r = gpu_load_infos->gpu_load[(low+1) % NMAX].update_time;

		if (r - refTime < refTime - l)
			low = low + 1;
	}
	return low % NMAX;
}

unsigned long cal_gpu_load(u64 current_time, u64 update_time,
				u64 begin_time, int minutes)
{
	unsigned long sysfs_busy_perc;
	unsigned long tmp_act_relative_busy, tmp_act_total;
	int tail = gpu_load_infos->tail, head = gpu_load_infos->head;

	if (current_time - update_time > minutes * USEC_PER_MINUTE) {
		sysfs_busy_perc = 0;
	} else {
		int tmpIndex = head;

		if (current_time - begin_time <= minutes * USEC_PER_MINUTE) {
			tmpIndex = head;
		} else {
			u64 begin = current_time - minutes * USEC_PER_MINUTE;
			int tmpTail = (tail < head) ? tail + NMAX : tail;

			tmpIndex = findItem(head, tmpTail-1, begin);
		}

		tmp_act_relative_busy =
			gpu_load_infos->gpu_load[tail-1].busy_time -
			gpu_load_infos->gpu_load[tmpIndex].busy_time;

		tmp_act_total = current_time -
			gpu_load_infos->gpu_load[tmpIndex].update_time;

		sysfs_busy_perc = (tmp_act_relative_busy * 100 * 10) /
			tmp_act_total;
	}
	return sysfs_busy_perc;
}

void convert_int_to_string(unsigned long perc, char *str, unsigned int size)
{
	char low[6] = "0";
	char mid[] = ".";

	snprintf(str, sizeof(low), "%lu", (perc / 10));
	snprintf(low, sizeof(low), "%lu", (perc % 10));
	strlcat(str, mid, size);
	strlcat(str, low, size);
}

static ssize_t gpu_period_load_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	unsigned long sysfs_busy_perc[3];
	int tail, head;
	char load[3][6] = {"0", "0", "0"};
	int i;
	unsigned int size;
	spin_lock(&sample_load_lock);
	tail = gpu_load_infos->tail;
	head = gpu_load_infos->head;

	if (tail == head) {
		sysfs_busy_perc[0] = 0;
		sysfs_busy_perc[1] = 0;
		sysfs_busy_perc[2] = 0;
	} else {
		u64 current_time = (u64)ktime_to_us(ktime_get());
		u64 update_time = gpu_load_infos->gpu_load[tail-1].update_time;
		u64 begin_time = gpu_load_infos->gpu_load[head].update_time;

		sysfs_busy_perc[0] = cal_gpu_load(current_time,
					update_time, begin_time, 1);
		sysfs_busy_perc[1] = cal_gpu_load(current_time,
					update_time, begin_time, 5);
		sysfs_busy_perc[2] = cal_gpu_load(current_time,
					update_time, begin_time, 15);
	}
	for (i = 0; i < 3; i++) {
		size = sizeof(load[i]);
		convert_int_to_string(sysfs_busy_perc[i], load[i], size);
	}

	spin_unlock(&sample_load_lock);
	return snprintf(buf, PAGE_SIZE, "%s  %s  %s\n",
		load[0],
		load[1],
		load[2]
		);
}

static ssize_t gpu_load_idd_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	unsigned long busy, total;
	spin_lock(&sample_load_lock);
	busy = gpu_load_rel_busy;
	total = gpu_load_total;
	spin_unlock(&sample_load_lock);
	return snprintf(buf, PAGE_SIZE, "%lu %lu\n", busy, total);
}

/*
 * Returns the time in ms for which gpu was in suspend state
 * since last time the entry is read.
 */
static ssize_t suspend_time_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	u64 time_diff = 0;

	spin_lock(&suspend_lock);
	time_diff = suspend_time_ms();
	/*
	 * Adding the previous suspend time also as the gpu
	 * can go and come out of suspend states in between
	 * reads also and we should have the total suspend
	 * since last read.
	 */
	time_diff += suspend_time;
	suspend_time = 0;
	spin_unlock(&suspend_lock);

	return snprintf(buf, PAGE_SIZE, "%llu\n", time_diff);
}

static ssize_t suspend_time_idd_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	u64 time_diff = 0;

	spin_lock(&suspend_lock);
	time_diff = suspend_time_ms_idd();
	/*
	 * Adding the previous suspend time also as the gpu
	 * can go and come out of suspend states in between
	 * reads also and we should have the total suspend
	 * since last read.
	 */
	suspend_time_idd += time_diff;
	spin_unlock(&suspend_lock);

	return snprintf(buf, PAGE_SIZE, "%llu\n", suspend_time_idd);
}
static DEVICE_ATTR_RO(gpu_load);

static DEVICE_ATTR_RO(suspend_time);
static DEVICE_ATTR(gpu_period_load, 0444, gpu_period_load_show, NULL);
static DEVICE_ATTR(gpu_load_idd, 0444, gpu_load_idd_show, NULL);
static DEVICE_ATTR(gpu_suspend_idd, 0444, suspend_time_idd_show, NULL);

static const struct device_attribute *adreno_tz_attr_list[] = {
		&dev_attr_gpu_load,
		&dev_attr_gpu_period_load,
		&dev_attr_gpu_load_idd,
		&dev_attr_gpu_suspend_idd,
		&dev_attr_suspend_time,
		NULL
};

void store_work_load(unsigned long gpu_load_total, unsigned long gpu_load_busy)
{
	/* Queue item to gpu_load_queue */
	int index = gpu_load_infos->tail;
	int head = gpu_load_infos->head;

	gpu_load_infos->gpu_load[index].total_time = gpu_load_total;
	gpu_load_infos->gpu_load[index].busy_time = gpu_load_busy;
	gpu_load_infos->gpu_load[index].update_time =
				(u64)ktime_to_us(ktime_get());

	if ((index + 1) % NMAX == head)
		gpu_load_infos->head = (head + 1) % NMAX;

	gpu_load_infos->tail = (index + 1) % NMAX;
}

void compute_work_load(struct devfreq_dev_status *stats,
		struct devfreq_msm_adreno_tz_data *priv,
		struct devfreq *devfreq)
{
	u64 busy;

	spin_lock(&sample_lock);
	/*
	 * Keep collecting the stats till the client
	 * reads it. Average of all samples and reset
	 * is done when the entry is read
	 */
	acc_total += stats->total_time;
	busy = (u64)stats->busy_time * stats->current_frequency;
	do_div(busy, devfreq->profile->freq_table[0]);
	acc_relative_busy += busy;

	spin_unlock(&sample_lock);

	spin_lock(&sample_load_lock);
	gpu_load_total += stats->total_time;
	gpu_load_rel_busy += (stats->busy_time * stats->current_frequency) /
			devfreq->profile->freq_table[0];

	store_work_load(gpu_load_total, gpu_load_rel_busy);

	spin_unlock(&sample_load_lock);
}

/* Trap into the TrustZone, and call funcs there. */
static int __secure_tz_reset_entry2(unsigned int *scm_data, u32 size_scm_data,
					bool is_64)
{
	int ret;
	/* sync memory before sending the commands to tz */
	__iowmb();

	if (!is_64) {
		struct scm_desc desc = {
			.args[0] = scm_data[0],
			.args[1] = scm_data[1],
			.arginfo = SCM_ARGS(2),
		};
		spin_lock(&tz_lock);
		ret = scm_call2_atomic(SCM_SIP_FNID(SCM_SVC_IO, TZ_RESET_ID),
			&desc);
		spin_unlock(&tz_lock);
	} else {
		struct scm_desc desc = {0};

		desc.arginfo = 0;
		ret = scm_call2(SCM_SIP_FNID(SCM_SVC_DCVS,
				 TZ_RESET_ID_64), &desc);
	}
	return ret;
}

static int __secure_tz_update_entry3(unsigned int *scm_data, u32 size_scm_data,
		int *val, u32 size_val, struct devfreq_msm_adreno_tz_data *priv)
{
	int ret;
	/* sync memory before sending the commands to tz */
	__iowmb();

	if (!priv->is_64) {
		struct scm_desc desc = {
			.args[0] = scm_data[0],
			.args[1] = scm_data[1],
			.args[2] = scm_data[2],
			.arginfo = SCM_ARGS(3),
		};
		spin_lock(&tz_lock);
		ret = scm_call2_atomic(SCM_SIP_FNID(SCM_SVC_IO, TZ_UPDATE_ID),
			&desc);
		spin_unlock(&tz_lock);
		*val = ret;
	} else {
		unsigned int cmd_id;
		struct scm_desc desc = {0};

		desc.args[0] = scm_data[0];
		desc.args[1] = scm_data[1];
		desc.args[2] = scm_data[2];

		if (!priv->ctxt_aware_enable) {
			desc.arginfo = SCM_ARGS(3);
			cmd_id =  TZ_V2_UPDATE_ID_64;
		} else {
			/* Add context count infomration to update*/
			desc.args[3] = scm_data[3];
			desc.arginfo = SCM_ARGS(4);
			cmd_id =  TZ_V2_UPDATE_WITH_CA_ID_64;
		}
			ret = scm_call2(SCM_SIP_FNID(SCM_SVC_DCVS, cmd_id),
						&desc);
			*val = desc.ret[0];
	}
	return ret;
}

static void tz_init_gpuloadinfos(void)
{
	gpu_load_infos = kzalloc(sizeof(struct gpu_load_queue), GFP_KERNEL);
	gpu_load_infos->head = gpu_load_infos->tail = 0;
	gpu_load_infos->gpu_load =
		kcalloc(NMAX, sizeof(struct gpu_load_data), GFP_KERNEL);
}

static int tz_init_ca(struct devfreq_msm_adreno_tz_data *priv)
{
	unsigned int tz_ca_data[2];
	struct scm_desc desc = {0};
	u8 *tz_buf;
	int ret;
	struct qtee_shm shm;

	/* Set data for TZ */
	tz_ca_data[0] = priv->bin.ctxt_aware_target_pwrlevel;
	tz_ca_data[1] = priv->bin.ctxt_aware_busy_penalty;

	if (!qtee_shmbridge_is_enabled()) {
		tz_buf = kzalloc(PAGE_ALIGN(sizeof(tz_ca_data)), GFP_KERNEL);
		if (!tz_buf)
			return -ENOMEM;
		desc.args[0] = virt_to_phys(tz_buf);
	} else {
		ret = qtee_shmbridge_allocate_shm(
				PAGE_ALIGN(sizeof(tz_ca_data)), &shm);
		if (ret)
			return -ENOMEM;
		tz_buf = shm.vaddr;
		desc.args[0] = shm.paddr;
	}

	memcpy(tz_buf, tz_ca_data, sizeof(tz_ca_data));
	/* Ensure memcpy completes execution */
	mb();
	dmac_flush_range(tz_buf,
		tz_buf + PAGE_ALIGN(sizeof(tz_ca_data)));

	desc.args[1] = sizeof(tz_ca_data);
	desc.arginfo = SCM_ARGS(2, SCM_RW, SCM_VAL);

	ret = scm_call2(SCM_SIP_FNID(SCM_SVC_DCVS,
			TZ_V2_INIT_CA_ID_64),
			&desc);
	if (!qtee_shmbridge_is_enabled())
		kzfree(tz_buf);
	else
		qtee_shmbridge_free_shm(&shm);

	return ret;
}

static int tz_init(struct devfreq_msm_adreno_tz_data *priv,
			unsigned int *tz_pwrlevels, u32 size_pwrlevels,
			unsigned int *version, u32 size_version)
{
	int ret;

	tz_init_gpuloadinfos();
	/* Make sure all CMD IDs are avaialble */
	if (scm_is_call_available(SCM_SVC_DCVS, TZ_INIT_ID_64) &&
			scm_is_call_available(SCM_SVC_DCVS, TZ_UPDATE_ID_64) &&
			scm_is_call_available(SCM_SVC_DCVS, TZ_RESET_ID_64)) {
		struct scm_desc desc = {0};
		u8 *tz_buf;
		struct qtee_shm shm;

		if (!qtee_shmbridge_is_enabled()) {
			tz_buf = kzalloc(PAGE_ALIGN(size_pwrlevels),
						GFP_KERNEL);
			if (!tz_buf)
				return -ENOMEM;
			desc.args[0] = virt_to_phys(tz_buf);
		} else {
			ret = qtee_shmbridge_allocate_shm(
					PAGE_ALIGN(size_pwrlevels), &shm);
			if (ret)
				return -ENOMEM;
			tz_buf = shm.vaddr;
			desc.args[0] = shm.paddr;
		}

		memcpy(tz_buf, tz_pwrlevels, size_pwrlevels);
		/* Ensure memcpy completes execution */
		mb();
		dmac_flush_range(tz_buf, tz_buf + PAGE_ALIGN(size_pwrlevels));

		desc.args[1] = size_pwrlevels;
		desc.arginfo = SCM_ARGS(2, SCM_RW, SCM_VAL);

		ret = scm_call2(SCM_SIP_FNID(SCM_SVC_DCVS, TZ_V2_INIT_ID_64),
				&desc);
		*version = desc.ret[0];
		if (!ret)
			priv->is_64 = true;
		if (!qtee_shmbridge_is_enabled())
			kzfree(tz_buf);
		else
			qtee_shmbridge_free_shm(&shm);
	} else
		ret = -EINVAL;

	 /* Initialize context aware feature, if enabled. */
	if (!ret && priv->ctxt_aware_enable) {
		if (priv->is_64 &&
			(scm_is_call_available(SCM_SVC_DCVS,
				TZ_V2_INIT_CA_ID_64)) &&
			(scm_is_call_available(SCM_SVC_DCVS,
				TZ_V2_UPDATE_WITH_CA_ID_64))) {
			ret = tz_init_ca(priv);
			/*
			 * If context aware feature initialization fails,
			 * just print an error message and return
			 * success as normal DCVS will still work.
			 */
			if (ret) {
				pr_err(TAG "tz: context aware DCVS init failed\n");
				priv->ctxt_aware_enable = false;
				return 0;
			}
		} else {
			pr_warn(TAG "tz: context aware DCVS not supported\n");
			priv->ctxt_aware_enable = false;
		}
	}

	return ret;
}

static inline int devfreq_get_freq_level(struct devfreq *devfreq,
	unsigned long freq)
{
	int lev;

	for (lev = 0; lev < devfreq->profile->max_state; lev++)
		if (freq == devfreq->profile->freq_table[lev])
			return lev;

	return -EINVAL;
}

static int tz_get_target_freq(struct devfreq *devfreq, unsigned long *freq)
{
	int result = 0;
	struct devfreq_msm_adreno_tz_data *priv = devfreq->data;
	struct devfreq_dev_status *stats = &devfreq->last_status;
	int val, level = 0;
	unsigned int scm_data[4];
	int context_count = 0;

	/* keeps stats.private_data == NULL   */
	result = devfreq_update_stats(devfreq);
	if (result) {
		pr_err(TAG "get_status failed %d\n", result);
		return result;
	}

	*freq = stats->current_frequency;
	priv->bin.total_time += stats->total_time;
	priv->bin.busy_time += stats->busy_time;

	if (stats->private_data)
		context_count =  *((int *)stats->private_data);

	/* Update the GPU load statistics */
	compute_work_load(stats, priv, devfreq);
	/*
	 * Do not waste CPU cycles running this algorithm if
	 * the GPU just started, or if less than FLOOR time
	 * has passed since the last run or the gpu hasn't been
	 * busier than MIN_BUSY.
	 */
	if ((stats->total_time == 0) ||
		(priv->bin.total_time < FLOOR) ||
		(unsigned int) priv->bin.busy_time < MIN_BUSY) {
		return 0;
	}

	level = devfreq_get_freq_level(devfreq, stats->current_frequency);
	if (level < 0) {
		pr_err(TAG "bad freq %ld\n", stats->current_frequency);
		return level;
	}

	/*
	 * If there is an extended block of busy processing,
	 * increase frequency.  Otherwise run the normal algorithm.
	 */
	if (!priv->disable_busy_time_burst &&
			priv->bin.busy_time > CEILING) {
		val = -1 * level;
	} else {

		scm_data[0] = level;
		scm_data[1] = priv->bin.total_time;
		scm_data[2] = priv->bin.busy_time;
		scm_data[3] = context_count;
		__secure_tz_update_entry3(scm_data, sizeof(scm_data),
					&val, sizeof(val), priv);
	}
	priv->bin.total_time = 0;
	priv->bin.busy_time = 0;

	/*
	 * If the decision is to move to a different level, make sure the GPU
	 * frequency changes.
	 */
	if (val) {
		level += val;
		level = max(level, 0);
		level = min_t(int, level, devfreq->profile->max_state - 1);
	}

	*freq = devfreq->profile->freq_table[level];
	return 0;
}

static int tz_notify(struct notifier_block *nb, unsigned long type, void *devp)
{
	int result = 0;
	struct devfreq *devfreq = devp;

	switch (type) {
	case ADRENO_DEVFREQ_NOTIFY_IDLE:
	case ADRENO_DEVFREQ_NOTIFY_RETIRE:
		mutex_lock(&devfreq->lock);
		result = update_devfreq(devfreq);
		mutex_unlock(&devfreq->lock);
		/* Nofifying partner bus governor if any */
		if (partner_gpu_profile && partner_gpu_profile->bus_devfreq) {
			mutex_lock(&partner_gpu_profile->bus_devfreq->lock);
			update_devfreq(partner_gpu_profile->bus_devfreq);
			mutex_unlock(&partner_gpu_profile->bus_devfreq->lock);
		}
		break;
	/* ignored by this governor */
	case ADRENO_DEVFREQ_NOTIFY_SUBMIT:
	default:
		break;
	}
	return notifier_from_errno(result);
}

static int tz_start(struct devfreq *devfreq)
{
	struct devfreq_msm_adreno_tz_data *priv;
	unsigned int tz_pwrlevels[MSM_ADRENO_MAX_PWRLEVELS + 1];
	int i, out, ret;
	unsigned int version;

	struct msm_adreno_extended_profile *gpu_profile = container_of(
					(devfreq->profile),
					struct msm_adreno_extended_profile,
					profile);

	/*
	 * Assuming that we have only one instance of the adreno device
	 * connected to this governor,
	 * can safely restore the pointer to the governor private data
	 * from the container of the device profile
	 */
	devfreq->data = gpu_profile->private_data;
	partner_gpu_profile = gpu_profile;

	priv = devfreq->data;
	priv->nb.notifier_call = tz_notify;

	out = 1;
	if (devfreq->profile->max_state < MSM_ADRENO_MAX_PWRLEVELS) {
		for (i = 0; i < devfreq->profile->max_state; i++)
			tz_pwrlevels[out++] = devfreq->profile->freq_table[i];
		tz_pwrlevels[0] = i;
	} else {
		pr_err(TAG "tz_pwrlevels[] is too short\n");
		return -EINVAL;
	}

	INIT_WORK(&gpu_profile->partner_start_event_ws,
					do_partner_start_event);
	INIT_WORK(&gpu_profile->partner_stop_event_ws,
					do_partner_stop_event);
	INIT_WORK(&gpu_profile->partner_suspend_event_ws,
					do_partner_suspend_event);
	INIT_WORK(&gpu_profile->partner_resume_event_ws,
					do_partner_resume_event);

	ret = tz_init(priv, tz_pwrlevels, sizeof(tz_pwrlevels), &version,
				sizeof(version));
	if (ret != 0 || version > MAX_TZ_VERSION) {
		pr_err(TAG "tz_init failed\n");
		return ret;
	}

	for (i = 0; adreno_tz_attr_list[i] != NULL; i++)
		device_create_file(&devfreq->dev, adreno_tz_attr_list[i]);

	return kgsl_devfreq_add_notifier(devfreq->dev.parent, &priv->nb);
}

static int tz_stop(struct devfreq *devfreq)
{
	int i;
	struct devfreq_msm_adreno_tz_data *priv = devfreq->data;

	kgsl_devfreq_del_notifier(devfreq->dev.parent, &priv->nb);

	for (i = 0; adreno_tz_attr_list[i] != NULL; i++)
		device_remove_file(&devfreq->dev, adreno_tz_attr_list[i]);

	flush_workqueue(workqueue);

	/* leaving the governor and cleaning the pointer to private data */
	devfreq->data = NULL;
	partner_gpu_profile = NULL;
	return 0;
}

static int tz_suspend(struct devfreq *devfreq)
{
	struct devfreq_msm_adreno_tz_data *priv = devfreq->data;
	unsigned int scm_data[2] = {0, 0};

	__secure_tz_reset_entry2(scm_data, sizeof(scm_data), priv->is_64);

	priv->bin.total_time = 0;
	priv->bin.busy_time = 0;
	return 0;
}

static int tz_handler(struct devfreq *devfreq, unsigned int event, void *data)
{
	int result;
	struct msm_adreno_extended_profile *gpu_profile;
	struct device_node *node = devfreq->dev.parent->of_node;

	/*
	 * We want to restrict this governor be set only for
	 * gpu devfreq devices.
	 */
	if (!of_device_is_compatible(node, "qcom,kgsl-3d0"))
		return -EINVAL;

	gpu_profile = container_of((devfreq->profile),
		struct msm_adreno_extended_profile, profile);

	switch (event) {
	case DEVFREQ_GOV_START:
		result = tz_start(devfreq);
		break;

	case DEVFREQ_GOV_STOP:
		/* Queue the stop work before the TZ is stopped */
		if (partner_gpu_profile && partner_gpu_profile->bus_devfreq)
			queue_work(workqueue,
				&gpu_profile->partner_stop_event_ws);
		spin_lock(&suspend_lock);
		suspend_start = 0;
		spin_unlock(&suspend_lock);
		result = tz_stop(devfreq);
		break;

	case DEVFREQ_GOV_SUSPEND:
		result = tz_suspend(devfreq);
		if (!result) {
			spin_lock(&suspend_lock);
			/* Collect the start sample for suspend time */
			suspend_start = (u64)ktime_to_ms(ktime_get());
			suspend_start_idd = suspend_start;
			spin_unlock(&suspend_lock);
		}
		break;

	case DEVFREQ_GOV_RESUME:
		spin_lock(&suspend_lock);
		suspend_time += suspend_time_ms();
		suspend_time_idd += suspend_time_ms_idd();
		/* Reset the suspend_start when gpu resumes */
		suspend_start = 0;
		suspend_start_idd = 0;
		spin_unlock(&suspend_lock);
		/* fallthrough */
	case DEVFREQ_GOV_INTERVAL:
		/* fallthrough, this governor doesn't use polling */
	default:
		result = 0;
		break;
	}

	if (partner_gpu_profile && partner_gpu_profile->bus_devfreq)
		switch (event) {
		case DEVFREQ_GOV_START:
			queue_work(workqueue,
					&gpu_profile->partner_start_event_ws);
			break;
		case DEVFREQ_GOV_SUSPEND:
			queue_work(workqueue,
					&gpu_profile->partner_suspend_event_ws);
			break;
		case DEVFREQ_GOV_RESUME:
			queue_work(workqueue,
					&gpu_profile->partner_resume_event_ws);
			break;
		}

	return result;
}

static void _do_partner_event(struct work_struct *work, unsigned int event)
{
	struct devfreq *bus_devfreq;

	if (partner_gpu_profile == NULL)
		return;

	bus_devfreq = partner_gpu_profile->bus_devfreq;

	if (bus_devfreq != NULL &&
		bus_devfreq->governor &&
		bus_devfreq->governor->event_handler)
		bus_devfreq->governor->event_handler(bus_devfreq, event, NULL);
}

static void do_partner_start_event(struct work_struct *work)
{
	_do_partner_event(work, DEVFREQ_GOV_START);
}

static void do_partner_stop_event(struct work_struct *work)
{
	_do_partner_event(work, DEVFREQ_GOV_STOP);
}

static void do_partner_suspend_event(struct work_struct *work)
{
	_do_partner_event(work, DEVFREQ_GOV_SUSPEND);
}

static void do_partner_resume_event(struct work_struct *work)
{
	_do_partner_event(work, DEVFREQ_GOV_RESUME);
}


static struct devfreq_governor msm_adreno_tz = {
	.name = "msm-adreno-tz",
	.get_target_freq = tz_get_target_freq,
	.event_handler = tz_handler,
};

static int __init msm_adreno_tz_init(void)
{
	workqueue = create_freezable_workqueue("governor_msm_adreno_tz_wq");

	if (workqueue == NULL)
		return -ENOMEM;

	return devfreq_add_governor(&msm_adreno_tz);
}
subsys_initcall(msm_adreno_tz_init);

static void __exit msm_adreno_tz_exit(void)
{
	int ret = devfreq_remove_governor(&msm_adreno_tz);

	if (ret)
		pr_err(TAG "failed to remove governor %d\n", ret);

	if (workqueue != NULL)
		destroy_workqueue(workqueue);
}

module_exit(msm_adreno_tz_exit);

MODULE_LICENSE("GPL v2");
