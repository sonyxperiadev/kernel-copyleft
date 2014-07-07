/* driver/net/pppocpw.c
 *
 * Driver for CDMA plus CDMA tunnel/ PPPoCPW Socket
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright 1999 Paul Mackerras.
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This driver handles C+W data packets between a UDP socket and a PPP channel.
 * The socket must keep connected, and only one session per socket is permitted.
 * A CT tunnnel ppp packet Contains follow fields:
 *
 * Tunnel header(8 bytes)
 * Flag(1 byte)
 * Address(1 byte) (O)
 * Control(1 byte) (O)
 * Protocol(1-2 bytes)
 * Payload(1500 bytes)
 * FCS (2-4 bytes)
 * Flag(1 byte)
 *
 * "Tunnel header" include 4 constant bytes:"0xc5, 0x01, 0x00, 0x00" and
 * 4 bytes length integer in big-endian format.
 * The 4 bytes length integer is named as "stream".
 * The process "ctcwd" (a C application to handle special requirement from
 * China Telecom) should write the value of "stream" in "sockaddr *" parameter
 * when it call "connect()"  function of "PX_PROTO_OCPW" socket.
 * This driver should add "Tunnel header" and convert standard PPP packet to
 * CT PPP packet before a packet is send.
 * This driver should delete "Tunnel header" and convert CT PPP packet
 * to standard PPP packet after a packet is received.
 * The format of CT PPP packet is similar as "PPP on Asynchronous Links".
 * This driver only works on IPv4 due to the lack of UDP
 * encapsulation support in IPv6.
 */

#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/skbuff.h>
#include <linux/file.h>
#include <linux/netdevice.h>
#include <linux/net.h>
#include <linux/udp.h>
#include <linux/ppp_defs.h>
#include <linux/if_ppp.h>
#include <linux/if_pppox.h>
#include <linux/ppp_channel.h>
#include <net/tcp_states.h>
#include <linux/uaccess.h>
#include <linux/bug.h>

static  void print_uint8_buffer(unsigned char *data , int len, char *note);

/* for enable/disable debug log */
static uint32_t ctdbg;
module_param(ctdbg, uint, S_IWUSR | S_IRUGO);

#define L2TP_CONTROL_BIT	0x80
#define L2TP_LENGTH_BIT		0x40
#define L2TP_SEQUENCE_BIT	0x08
#define L2TP_OFFSET_BIT		0x02
#define L2TP_VERSION		0x02
#define L2TP_VERSION_MASK	0x0F

#define PPP_ADDR	0xFF
#define PPP_CTRL	0x03

#define PPP_FLAG_LENGTH	1
#define PPP_MAX_FCS_LEN	4
#define MAX_CONVERT_LENGTH	\
	(PPP_HDRLEN+PPP_MRU+PPP_MAX_FCS_LEN+PPP_FLAG_LENGTH*2)

/* DEBUG marco define START! */
#define DEBUG_MSG_LEN		128
#define DEBUG_HEX_P_CHAR	4
#define PPP_CT_DEBUG_LOG	(1u << 0)
#define PPP_CT_SMSG_LOG	(1u << 1)
#define PPP_CT_DMSG_LOG	(1u << 2)
#define PPP_CT_STACK_LOG	(1u << 3)

/*
 * Some of logs should be controlled by module parameter : "ctdbg" .
 * When we want to view these logs, we should just set "ctdbg" to
 * the corresponding value.
 */

/* info */
#define DEBUG_PRINTI(x...) \
do { \
	if (PPP_CT_DEBUG_LOG & ctdbg) \
		printk(KERN_INFO x); \
} while (0)

/* Print message from network : */
#define DEBUG_PRINTM(data, len, note, ism) \
do { \
	if (((PPP_CT_SMSG_LOG & ctdbg) && ism) || \
		(PPP_CT_DMSG_LOG & ctdbg)) \
		print_uint8_buffer(data , len, note); \
} while (0)

/* Print "call stack" if we need: */
#define DEBUG_STACK(c, f...) \
do { \
	if (PPP_CT_STACK_LOG & ctdbg) \
		WARN(c, f); \
} while (0)

/* DEBUG marco define END! */

#define PUT_BYTE(cvt_inc, buf, c, islcp) \
do { \
	if ((islcp && c < 0x20) ||  c == PPP_FLAG ||  c == PPP_ESCAPE) { \
		*buf++ = PPP_ESCAPE; \
		*buf++ = c ^ 0x20; \
		cvt_inc += 2; \
	} \
	else{ \
		*buf++ = c; \
		cvt_inc++; \
	} \
} while (0)

static struct sk_buff_head delivery_queue;

/*
 * Print every byte of the given buffer in HEX format like "XX, ".
 * This function is used to view Rx and Tx message in CT interface.
 */
static void print_uint8_buffer(unsigned char *data , int len, char *note)
{
	int ic;
	char dispbuffer[DEBUG_MSG_LEN*DEBUG_HEX_P_CHAR+2] = {0};
	char *start_p = dispbuffer;
	for (ic = 0; ic < len; ic++) {
		if (ic >= DEBUG_MSG_LEN)
			break;
		else if (ic == len-1) {
			snprintf(start_p, DEBUG_HEX_P_CHAR+1,
				"%02X  ", data[ic]);
		} else {
			snprintf(start_p, DEBUG_HEX_P_CHAR+1,
				"%02X, ", data[ic]);
		}
		start_p += DEBUG_HEX_P_CHAR;
	}
	*start_p++ = '\0';
	printk(KERN_DEBUG "%s : %s\n", note, dispbuffer);
}

static int pppocpw_recv_core(struct sock *sk_udp, struct sk_buff *skb)
{
	struct sock *sk = (struct sock *)sk_udp->sk_user_data;
	uint offset = 0, cvt_inc = 0;
	int fcs = 0;
	unsigned int count = 0;
	int proto = 0,  is_sig_msg = 0;
	unsigned char *data = NULL;
	unsigned char *cvt_buff = NULL;
	DEBUG_PRINTI(KERN_DEBUG "pppocpw_recv_core() worked!\n");

	cvt_buff = kmalloc(sizeof(unsigned char) * MAX_CONVERT_LENGTH,
		GFP_KERNEL);
	if (unlikely(!cvt_buff))
		panic("Fatal Error in pppocpw_init(): Out of memory!\n");

	/* Drop the packet if it is too short. */
	if (skb->len < sizeof(struct udphdr) + 8) {
		printk(KERN_DEBUG "CTTunnel drop  package:  it is too short !\n");
		goto drop;
	}

	/* extract CTTunnel header */

	/* Skip UDP header. */
	skb_pull(skb, sizeof(struct udphdr));

	/* Skip CTTunnel header. */
	skb_pull(skb, 8);

	count = skb->len;
	data = skb->data;

	/* input buffer at least has 7e, 7e for flag */
	if (count <= 2 || data[offset++] != PPP_FLAG) {
		printk(KERN_DEBUG "pppocpw_recv_core: drop %x %x\n",
			data[offset - 1], data[offset]);
		goto drop;
	}

	/*
	 * The highest bit of protocol is 1 when
	 * PPP transmit signaling message
	 */
	is_sig_msg = data[1] & 0x80;
	/* we just need print signaling message */
	DEBUG_PRINTM(skb->data, skb->len, "PPP CT RX: ", is_sig_msg);
	while (offset < count) {
		if (data[offset] == PPP_FLAG)
			break;

		if (data[offset] == PPP_ESCAPE)
			cvt_buff[cvt_inc++] = data[++offset] ^ 0x20;
		else
			cvt_buff[cvt_inc++] = data[offset];
		offset++;
	}
	DEBUG_PRINTI(KERN_DEBUG "cvt_inc=%d, len=%d\n",
		cvt_inc,
		skb->len);

	skb_trim(skb, cvt_inc);
	memcpy(skb->data, cvt_buff, cvt_inc);

	/* check the FCS */
	data = skb->data;
	count = skb->len;
	if (count < 3) {
		printk(KERN_ERR "Cvt PPP buffer is less than 3 bytes !\n");
		/*
		 * "skb->data" should at least contain:
		 * protocol(>=1 byte) and FCS(>=2 bytes)
		 */
		goto drop;
	}
	fcs = PPP_INITFCS;
	for (; count > 0; --count)
		fcs = PPP_FCS(fcs, *data++);
	if (fcs != PPP_GOODFCS) {
		printk(KERN_DEBUG "CTTunnel drop  package: Bad FCS!\n");
		goto drop; /* bad FCS */
	}
	skb_trim(skb, skb->len - 2);

	/* check for address/control and protocol compression */
	data = skb->data;
	if (data[0] == PPP_ALLSTATIONS) {
		/* chop off address/control */
		if (data[1] != PPP_UI || skb->len < 3) {
			printk(KERN_DEBUG "CTTunnel drop  package: chop off address/control !\n");
			/*
			 * "skb->data" should at least contain:
			 * address 0xFF , control 0x03,
			 * protocol (>=1bytes) if the first byte is 0xFF.
			 */
			goto drop;
		}
		data = skb_pull(skb, 2);
	}
	proto = data[0];
	if (proto & 1) {
		/* protocol is compressed */
		skb_push(skb, 1)[0] = 0;
	}

	kfree(cvt_buff);

	/* Skip PPP address and control if they are present. */
	/* Fix PPP protocol if it is compressed. */
	/* Finally, deliver the packet to PPP channel. */
	skb_orphan(skb);
	DEBUG_PRINTM(skb->data, skb->len, "PPP Dev RX: ", is_sig_msg);
	/*
	 * Route table should be set by IP stack later,
	 * to do that, we must clean it here.
	 * Please check "ip_rcv_finish()" in ip_input.c.
	 * The old value is the route table for "sk_udp",if
	 * we remain it, IP stack will not compute correct
	 * value and it will bring some issues in higher
	 * layer stack such as UDP.
	 */
	if (!is_sig_msg)
		skb_dst_set(skb, NULL);
	ppp_input(&pppox_sk(sk)->chan, skb);
	return NET_RX_SUCCESS;
drop:
	kfree(cvt_buff);
	kfree_skb(skb);
	return NET_RX_DROP;
}

static int pppocpw_recv(struct sock *sk_udp, struct sk_buff *skb)
{
	sock_hold(sk_udp);
	sk_receive_skb(sk_udp, skb, 0);
	return 0;
}

static void pppocpw_xmit_core(struct work_struct *delivery_work)
{
	mm_segment_t old_fs = get_fs();
	struct sk_buff *skb;

	set_fs(KERNEL_DS);
	while ((skb = skb_dequeue(&delivery_queue))) {
		struct sock *sk_udp = skb->sk;
		struct kvec iov = {.iov_base = skb->data, .iov_len = skb->len};
		struct msghdr msg = {
			.msg_iov = (struct iovec *)&iov,
			.msg_iovlen = 1,
			.msg_flags = MSG_NOSIGNAL | MSG_DONTWAIT,
		};
		sk_udp->sk_prot->sendmsg(NULL, sk_udp, &msg, skb->len);
		kfree_skb(skb);
	}
	set_fs(old_fs);
}

static DECLARE_WORK(delivery_work, pppocpw_xmit_core);

static int pppocpw_xmit(struct ppp_channel *chan, struct sk_buff *skb)
{
	struct sock *sk_udp = (struct sock *)chan->private;
	struct pppocpw_opt *opt = NULL;
	uint stream = 0;
	uint offset = 0, cvt_inc = 0;
	int fcs = 0;
	unsigned short proto = 0, is_sig_msg = 0;
	unsigned char *buf = NULL;
	unsigned char c = 0;
	int islcp = 0;
	struct sk_buff *skb_new = NULL;
	struct sk_buff *skb_send = NULL;
	int skb_buf_len = 0;
	unsigned char *cvt_buff = NULL;
	DEBUG_STACK(1, "In pppocpw_xmit()\n");

	cvt_buff = kmalloc(sizeof(unsigned char) * MAX_CONVERT_LENGTH,
		GFP_KERNEL);
	if (unlikely(!cvt_buff))
		panic("Fatal Error in pppocpw_init(): Out of memory!\n");
	opt = &pppox_sk(sk_udp->sk_user_data)->proto.cpw;

	proto = (skb->data[0] << 8) + skb->data[1];
	buf = cvt_buff;

	/*
	 * LCP packets with code values between 1 (configure-request)
	 * and 7 (code-reject) must be sent as though no options
	 * had been negotiated.
	 */
	islcp = proto == PPP_LCP && 1 <= skb->data[2] && skb->data[2] <= 7;
	/*
	 * Start of a new packet - insert the leading FLAG
	 * character if necessary.
	 */
	is_sig_msg = proto & 0x8000;
	DEBUG_PRINTM(skb->data, skb->len, "PPP Dev TX: ", is_sig_msg);
	fcs = PPP_INITFCS;
	if (islcp || !is_sig_msg) {
		*buf++ = PPP_FLAG;
		cvt_inc++;
		/*
		 * Put in the address/control bytes if necessary
		 */
		PUT_BYTE(cvt_inc, buf, 0xff, islcp);
		fcs = PPP_FCS(fcs, 0xff);
		PUT_BYTE(cvt_inc, buf, 0x03, islcp);
		fcs = PPP_FCS(fcs, 0x03);
	}
	/*
	 * Once we put in the last byte, we need to put in the FCS
	 * and closing flag, so make sure there is at least 7 bytes
	 * of free space in the output buffer.
	 */
	while (offset < skb->len) {
		c = skb->data[offset++];
		/* compress protocol field */
		fcs = PPP_FCS(fcs, c);
		PUT_BYTE(cvt_inc, buf, c, islcp);
	}
	/*
	 * We have finished the packet.  Add the FCS and flag.
	 */
	fcs = ~fcs;
	c = fcs & 0xff;
	PUT_BYTE(cvt_inc, buf, c, islcp);
	c = (fcs >> 8) & 0xff;
	PUT_BYTE(cvt_inc, buf, c, islcp);
	*buf++ = PPP_FLAG;
	cvt_inc++;
	DEBUG_PRINTI(KERN_DEBUG "pppocpw_xmit: convert OK cvt_inc=%d, len=%d\n",
		cvt_inc, skb->len);
	if (unlikely(cvt_inc >= MAX_CONVERT_LENGTH)) {
		printk(KERN_ERR "Fatal Error in pppocpw_xmit()\n");
		printk(KERN_ERR "durring convertion!Drop!\n");
		printk(KERN_ERR "cvt_inc is %d, too long!\n",
			cvt_inc);
		goto drop;
	}
	skb_buf_len = skb->len + (skb->data - skb->head);
	DEBUG_PRINTI(KERN_DEBUG "skb_buf_len = %d\n", skb_buf_len);

	if (cvt_inc + 8 > skb_buf_len) {
		/* We nee a bigger sk_buffer to send */
		DEBUG_PRINTI(KERN_DEBUG "we need new skbuffer to Tx because\n");
		DEBUG_PRINTI(KERN_DEBUG "CT ppp data is too long !\n");
		DEBUG_PRINTI(KERN_DEBUG " cvt_inc=%d,skb->len=%d, max = %d\n",
			cvt_inc, skb->len, skb_buf_len);
		skb_new = dev_alloc_skb(MAX_CONVERT_LENGTH+8);
		skb_put(skb_new, cvt_inc);
		skb_send = skb_new;
		kfree_skb(skb);
	} else {
		DEBUG_PRINTI(KERN_DEBUG "skb->data is enough long .\n");
		skb_push(skb, cvt_inc - skb->len);
		skb_send = skb;
	}
	memcpy(skb_send->data, cvt_buff, cvt_inc);
	/* add cttunnel header */
	skb_push(skb_send, 8);
	skb_send->data[0] = 0xC5;
	skb_send->data[1] = 0x01;
	skb_send->data[2] = 0;
	skb_send->data[3] = 0;
	stream = opt->stream_id;
	skb_send->data[4] = (stream >> 24) & 0xFF;
	skb_send->data[5] = (stream >> 16) & 0xFF;
	skb_send->data[6] = (stream >> 8) & 0xFF;
	skb_send->data[7] = (stream) & 0xFF;
	/* Now send the packet via the delivery queue. */
	DEBUG_PRINTM(skb_send->data, skb_send->len,
		"PPP CT TX: ", is_sig_msg);
	skb_set_owner_w(skb_send, sk_udp);
	skb_queue_tail(&delivery_queue, skb_send);
	schedule_work(&delivery_work);
drop:
	kfree(cvt_buff);
	return 1;
}

static struct ppp_channel_ops pppocpw_channel_ops = {
	.start_xmit = pppocpw_xmit,
};

static int pppocpw_connect(struct socket *sock, struct sockaddr *useraddr,
	int addrlen, int flags)
{
	struct sock *sk = sock->sk;
	struct pppox_sock *po = pppox_sk(sk);
	struct sockaddr_pppocpw *addr = (struct sockaddr_pppocpw *) useraddr;
	struct socket *sock_udp = NULL;
	struct sock *sk_udp;
	int error;

	DEBUG_PRINTI(KERN_DEBUG "pppocpw_connect = %d\n", addr->stream_id);

	lock_sock(sk);
	error = -EALREADY;
	if (sk->sk_state != PPPOX_NONE)
		goto out;

	sock_udp = sockfd_lookup(addr->udp_socket, &error);
	if (!sock_udp)
		goto out;
	sk_udp = sock_udp->sk;
	lock_sock(sk_udp);

	/* Remove this check when IPv6 supports UDP encapsulation. */
	error = -EAFNOSUPPORT;
	if (sk_udp->sk_family != AF_INET)
		goto out;
	error = -EPROTONOSUPPORT;
	if (sk_udp->sk_protocol != IPPROTO_UDP)
		goto out;
	error = -EDESTADDRREQ;
	if (sk_udp->sk_state != TCP_ESTABLISHED)
		goto out;
	error = -EBUSY;
	if (udp_sk(sk_udp)->encap_type || sk_udp->sk_user_data)
		goto out;
	if (!sk_udp->sk_bound_dev_if) {
		struct dst_entry *dst = sk_dst_get(sk_udp);
		error = -ENODEV;
		if (!dst)
			goto out;
		sk_udp->sk_bound_dev_if = dst->dev->ifindex;
		dst_release(dst);
	}

	/*
	 * CT tunnel head length:
	 * lcp: 7e, ff, 7d, 03, c5, 01, 00, 00, streamId(4 bytes)
	 * data: 7e, protocol(2 bytes), c5, 01, 00, 00, streamId(4 bytes)
	 */
	po->chan.hdrlen = 12;
	po->chan.private = sk_udp;
	po->chan.ops = &pppocpw_channel_ops;
	po->chan.mtu = PPP_MRU - 80;
	po->proto.cpw.stream_id = addr->stream_id;
	po->proto.cpw.backlog_rcv = sk_udp->sk_backlog_rcv;

	error = ppp_register_channel(&po->chan);
	if (error) {
		printk(KERN_ERR "pppocpw register ppp channel failed!\n");
		goto out;
	}

	sk->sk_state = PPPOX_CONNECTED;
	udp_sk(sk_udp)->encap_type = UDP_ENCAP_L2TPINUDP;
	udp_sk(sk_udp)->encap_rcv = pppocpw_recv;
	sk_udp->sk_backlog_rcv = pppocpw_recv_core;
	sk_udp->sk_user_data = sk;
out:
	if (sock_udp) {
		release_sock(sk_udp);
		if (error)
			sockfd_put(sock_udp);
	}
	release_sock(sk);
	DEBUG_PRINTI(KERN_DEBUG "pppocpw_connect oK\n");
	return error;
}

static int pppocpw_release(struct socket *sock)
{
	struct sock *sk = sock->sk;

	DEBUG_PRINTI(KERN_DEBUG "pppocpw_release()\n");

	if (!sk)
		return 0;

	lock_sock(sk);
	if (sock_flag(sk, SOCK_DEAD)) {
		release_sock(sk);
		return -EBADF;
	}

	if (sk->sk_state != PPPOX_NONE) {
		struct sock *sk_udp = (struct sock *)pppox_sk(sk)->chan.private;
		lock_sock(sk_udp);
		pppox_unbind_sock(sk);
		udp_sk(sk_udp)->encap_type = 0;
		udp_sk(sk_udp)->encap_rcv = NULL;
		sk_udp->sk_backlog_rcv = pppox_sk(sk)->proto.cpw.backlog_rcv;
		sk_udp->sk_user_data = NULL;
		release_sock(sk_udp);
		sockfd_put(sk_udp->sk_socket);
	}

	sock_orphan(sk);
	sock->sk = NULL;
	release_sock(sk);
	sock_put(sk);
	return 0;
}

static struct proto pppocpw_proto = {
	.name = "PPPOCPW",
	.owner = THIS_MODULE,
	.obj_size = sizeof(struct pppox_sock),
};

static const struct proto_ops pppocpw_proto_ops = {
	.family = PF_PPPOX,
	.owner = THIS_MODULE,
	.release = pppocpw_release,
	.bind = sock_no_bind,
	.connect = pppocpw_connect,
	.socketpair = sock_no_socketpair,
	.accept = sock_no_accept,
	.getname = sock_no_getname,
	.poll = sock_no_poll,
	.ioctl = pppox_ioctl,
	.listen = sock_no_listen,
	.shutdown = sock_no_shutdown,
	.setsockopt = sock_no_setsockopt,
	.getsockopt = sock_no_getsockopt,
	.sendmsg = sock_no_sendmsg,
	.recvmsg = sock_no_recvmsg,
	.mmap = sock_no_mmap,
};

static int pppocpw_create(struct net *net, struct socket *sock)
{
	struct sock *sk;

	printk(KERN_DEBUG "pppocpw_create\n");
	sk = sk_alloc(net, PF_PPPOX, GFP_KERNEL, &pppocpw_proto);
	if (!sk)
		return -ENOMEM;

	sock_init_data(sock, sk);
	sock->state = SS_UNCONNECTED;
	sock->ops = &pppocpw_proto_ops;
	sk->sk_protocol = PX_PROTO_OCPW;
	sk->sk_state = PPPOX_NONE;
	return 0;
}

static struct pppox_proto pppocpw_pppox_proto = {
	.create = pppocpw_create,
	.owner = THIS_MODULE,
};

static int __init pppocpw_init(void)
{
	int error;

	DEBUG_PRINTI(KERN_DEBUG "pppocpw_init\n");

	error = proto_register(&pppocpw_proto, 0);
	if (error)
		return error;

	error = register_pppox_proto(PX_PROTO_OCPW, &pppocpw_pppox_proto);
	if (error)
		proto_unregister(&pppocpw_proto);
	else
		skb_queue_head_init(&delivery_queue);
	return error;
}

static void __exit pppocpw_exit(void)
{
	DEBUG_PRINTI(KERN_DEBUG "pppocpw_exit\n");
	unregister_pppox_proto(PX_PROTO_OCPW);
	proto_unregister(&pppocpw_proto);
}

module_init(pppocpw_init);
module_exit(pppocpw_exit);

MODULE_DESCRIPTION("PPP on China Telecom Tunnel (PPPoCPW)");
MODULE_AUTHOR("Kaiyan.Liu <kaiyan.liu@sonymobile.com>");
MODULE_LICENSE("GPL");
