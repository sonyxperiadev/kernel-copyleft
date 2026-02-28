// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021-2022, 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/skbuff.h>
#include <linux/rtnetlink.h>
#include <net/pkt_sched.h>

/* Insert newest first, last 4 bytes of the change id */
static char *verinfo[] = { "b10f2ea2", "e6371d40", "7415921c", "ae244a9d" };
module_param_array(verinfo, charp, NULL, 0444);
MODULE_PARM_DESC(verinfo, "Version of the driver");

static const char *rmnet_sch_version = "1.2";

/* queue 0 has highest priority */
#define RMNET_SCH_MAX_QUEUE 4

/* Linux priority 6, 7, 8, 9 maps to queue 0, 1, 2, 3.
 * All other priorities use queue 3 */
static const u8 prio2queue[TC_PRIO_MAX + 1] = { 3, 3, 3, 3, 3, 3, 0, 1,
						2, 3, 3, 3, 3, 3, 3, 3 };

/* Bytes to dequeue before switching to lower priority queue */
static const int bytes_limit[RMNET_SCH_MAX_QUEUE] = { 256 * 1024, 128 * 1024,
						      64 * 1024, 32 * 1024 };

/* Packets to dequeue before switching to lower priority queue */
static const int pkts_limit[RMNET_SCH_MAX_QUEUE] = { 8, 6, 4, 2 };

/* Queue len ratio (total 10) for each queue */
static const int qlen_ratio[RMNET_SCH_MAX_QUEUE] = { 4, 3, 2, 1 };

struct rmnet_sch_queue {
	struct qdisc_skb_head q;
	int pkts_quota;
	int bytes_quota;
	unsigned int qlen_thresh;
	unsigned int qlen_thresh2;
};

struct rmnet_sch_priv {
	struct rmnet_sch_queue queue[RMNET_SCH_MAX_QUEUE];
};

/*
 * Choose a queue that exceeds qlen threshold to drop.
 * return RMNET_SCH_MAX_QUEUE if no such queue.
 */
static int rmnet_sch_next_to_drop(struct rmnet_sch_priv *priv)
{
	int candidate = RMNET_SCH_MAX_QUEUE;
	int candidate2 = RMNET_SCH_MAX_QUEUE;
	int qn, diff, max = -1;

	/* candidate is the queue that exceeds thresh2 the most.
	 * candidate2 is the lowest priority queue that exceeds thresh.
	 */
	for (qn = 0; qn < RMNET_SCH_MAX_QUEUE; qn++) {
		if (priv->queue[qn].q.qlen > priv->queue[qn].qlen_thresh2) {
			diff = priv->queue[qn].q.qlen -
			       priv->queue[qn].qlen_thresh2;
			if (diff >= max) {
				max = diff;
				candidate = qn;
			}
		}
		if (priv->queue[qn].q.qlen > priv->queue[qn].qlen_thresh)
			candidate2 = qn;
	}

	if (candidate < RMNET_SCH_MAX_QUEUE)
		return candidate;

	return candidate2;
}

static inline void rmnet_sch_set_quota(struct rmnet_sch_priv *priv, int qn)
{
	priv->queue[qn].pkts_quota = pkts_limit[qn];
	priv->queue[qn].bytes_quota = bytes_limit[qn];
}

static inline void rmnet_sch_set_qlen(struct rmnet_sch_priv *priv, int qn,
				      unsigned int tx_qlen)
{
	priv->queue[qn].qlen_thresh = tx_qlen / 10 * qlen_ratio[qn];
	priv->queue[qn].qlen_thresh2 = priv->queue[qn].qlen_thresh << 1;
}

static int rmnet_sch_enqueue(struct sk_buff *skb, struct Qdisc *sch,
			     struct sk_buff **to_free)
{
	struct rmnet_sch_priv *priv = qdisc_priv(sch);
	unsigned int pkt_len = qdisc_pkt_len(skb);
	int qn_to_enq;
	int qn_to_drop;
	struct sk_buff *skb_to_drop;

	qn_to_enq = prio2queue[skb->priority & TC_PRIO_MAX];

	/* If qlen is full, try to drop one packet from the queue that
	 * exceeds qlen threshold
	 */
	if (unlikely(sch->q.qlen >= qdisc_dev(sch)->tx_queue_len)) {
		qn_to_drop = rmnet_sch_next_to_drop(priv);
		if (qn_to_drop < RMNET_SCH_MAX_QUEUE &&
		    qn_to_drop != qn_to_enq) {
			skb_to_drop = __qdisc_dequeue_head(
				&priv->queue[qn_to_drop].q);
			if (likely(skb_to_drop)) {
				sch->qstats.backlog -=
					qdisc_pkt_len(skb_to_drop);
				sch->q.qlen--;
				qdisc_drop(skb_to_drop, sch, to_free);
			}
		} else {
			return qdisc_drop(skb, sch, to_free);
		}
	}

	__qdisc_enqueue_tail(skb, &priv->queue[qn_to_enq].q);
	qdisc_update_stats_at_enqueue(sch, pkt_len);
	return NET_XMIT_SUCCESS;
}

/*
 * Next queue to dequeue. RMNET_SCH_MAX_QUEUE no data available.
 */
static u8 rmnet_sch_next_to_dequeue(struct rmnet_sch_priv *priv)
{
	int qn, candidate = RMNET_SCH_MAX_QUEUE;

	for (qn = 0; qn < RMNET_SCH_MAX_QUEUE; qn++) {
		if (!priv->queue[qn].q.qlen)
			continue;
		if (priv->queue[qn].pkts_quota <= 0 ||
		    priv->queue[qn].bytes_quota <= 0) {
			if (qn < candidate)
				candidate = qn;
			continue;
		}
		return qn;
	}

	/* Either no packet, or all queues with packets have quota consumed,
	 * reset quota */
	for (qn = 0; qn < RMNET_SCH_MAX_QUEUE; qn++)
		rmnet_sch_set_quota(priv, qn);

	return candidate;
}

static struct sk_buff *rmnet_sch_dequeue(struct Qdisc *sch)
{
	struct rmnet_sch_priv *priv = qdisc_priv(sch);
	struct sk_buff *skb = NULL;
	u8 qn;

	qn = rmnet_sch_next_to_dequeue(priv);

	if (qn < RMNET_SCH_MAX_QUEUE) {
		skb = __qdisc_dequeue_head(&priv->queue[qn].q);
		if (likely(skb)) {
			priv->queue[qn].pkts_quota--;
			priv->queue[qn].bytes_quota -= qdisc_pkt_len(skb);
			qdisc_update_stats_at_dequeue(sch, skb);
		}
	}

	return skb;
}

static struct sk_buff *rmnet_sch_peek(struct Qdisc *sch)
{
	struct rmnet_sch_priv *priv = qdisc_priv(sch);
	struct sk_buff *skb = NULL;
	u8 qn;

	qn = rmnet_sch_next_to_dequeue(priv);

	if (qn < RMNET_SCH_MAX_QUEUE)
		skb = priv->queue[qn].q.head;

	return skb;
}

static int rmnet_sch_init(struct Qdisc *sch, struct nlattr *arg,
			  struct netlink_ext_ack *extack)
{
	struct rmnet_sch_priv *priv = qdisc_priv(sch);
	int qn;

	for (qn = 0; qn < RMNET_SCH_MAX_QUEUE; qn++) {
		rmnet_sch_set_quota(priv, qn);
		rmnet_sch_set_qlen(priv, qn, qdisc_dev(sch)->tx_queue_len);
	}

	sch->flags |= TCQ_F_CAN_BYPASS;

	return 0;
}

static void rmnet_sch_reset(struct Qdisc *sch)
{
	struct rmnet_sch_priv *priv = qdisc_priv(sch);
	int qn;

	for (qn = 0; qn < RMNET_SCH_MAX_QUEUE; qn++) {
		kfree_skb_list(priv->queue[qn].q.head);
		priv->queue[qn].q.head = NULL;
		priv->queue[qn].q.tail = NULL;
		priv->queue[qn].q.qlen = 0;
		rmnet_sch_set_quota(priv, qn);
		rmnet_sch_set_qlen(priv, qn, qdisc_dev(sch)->tx_queue_len);
	}

	/* stats will be reset by qdisc_reset */
}

static int rmnet_sch_change_tx_queue_len(struct Qdisc *sch, unsigned int qlen)
{
	struct rmnet_sch_priv *priv = qdisc_priv(sch);
	int qn;

	for (qn = 0; qn < RMNET_SCH_MAX_QUEUE; qn++)
		rmnet_sch_set_qlen(priv, qn, qlen);

	return 0;
}

static struct Qdisc_ops rmnet_sch_qdisc_ops __read_mostly = {
	.id = "rmnet_sch",
	.priv_size = sizeof(struct rmnet_sch_priv),
	.enqueue = rmnet_sch_enqueue,
	.dequeue = rmnet_sch_dequeue,
	.peek = rmnet_sch_peek,
	.init = rmnet_sch_init,
	.reset = rmnet_sch_reset,
	.change_tx_queue_len = rmnet_sch_change_tx_queue_len,
	.owner = THIS_MODULE,
};

static int __init rmnet_sch_module_init(void)
{
	pr_info("sch: init (%s)\n", rmnet_sch_version);
	return register_qdisc(&rmnet_sch_qdisc_ops);
}

static void __exit rmnet_sch_module_exit(void)
{
	unregister_qdisc(&rmnet_sch_qdisc_ops);
}

MODULE_LICENSE("GPL v2");
module_init(rmnet_sch_module_init);
module_exit(rmnet_sch_module_exit);
