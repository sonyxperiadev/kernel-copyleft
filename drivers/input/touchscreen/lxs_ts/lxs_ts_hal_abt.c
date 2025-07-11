/*
 * Copyright 2024 Sony Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */
// SPDX-License-Identifier: GPL-2.0
/*
 * LXS touch raw-data debugging via network
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "lxs_ts_hal_abt.h"

#if defined(__LXS_SUPPORT_ABT)
static u32 t_abt_dbg_flag;

/* usage
 * (1) echo <value> > /sys/module/{Touch Module Name}/parameters/s_abt_dbg_flag
 * (2) insmod {Touch Module Name}.ko s_abt_dbg_flag=<value>
 */
module_param_named(s_abt_dbg_flag, t_abt_dbg_flag, int, 0664);

static u32 t_abt_dbg_mask;

/* usage
 * (1) echo <value> > /sys/module/{Touch Module Name}/parameters/s_abt_dbg_mask
 * (2) insmod {Touch Module Name}.ko s_abt_dbg_mask=<value>
 */
module_param_named(s_abt_dbg_mask, t_abt_dbg_mask, int, 0664);

static void abt_report_enable(struct abt_data *abt, u8 mode);

static struct abt_data *__dev_get_abt(struct device *dev)
{
	return ((struct lxs_ts *)dev_get_drvdata(dev))->abt;
}

static int abt_check_ipv4_num(char *str)
{
	while (*str) {
		if (!isdigit(*str))
			return 0;
		str++; //point to next character
	}
	return 1;
}

static int abt_check_ipv4(struct abt_data *abt, char *ip)
{
	char *ip_tmp, *ip_dup, *ptr;
	int num, dots = 0;
	int chk = 0;
	int ret = 0;

	if (ip == NULL)
		return 0;

	ip_dup = kstrdup(ip, GFP_KERNEL);
	if (!ip_dup) {
		t_abt_err(abt, "%s\n", "failed to allocate copy of ip string");
		return 0;
	}

	ip_tmp = ip_dup;

	//cut the string using dor delimiter
	ptr = strsep(&ip_tmp, ".");
	if (ptr == NULL)
		goto out;

	while (ptr) {
		//check whether the sub string is holding only number or not
		if (!abt_check_ipv4_num(ptr))
			goto out;

		//convert substring to number
		ret = kstrtoint(ptr, 10, &num);
		if (ret < 0)
			goto out;

		if ((num < 0) || (num > 255))
			goto out;

		//cut the next part of the string
		ptr = strsep(&ip_tmp, ".");
		if (ptr != NULL)
			dots++;	//increase the dot count
	}

	//if the number of dots are not 3, return false
	chk = (dots == 3);

out:
	kfree(ip_dup);

	if (!chk)
		t_abt_err(abt, "invalid ip, %s\n", ip);

	return chk;
}

static int abt_access_not_allowed(struct abt_data *abt)
{
	struct lxs_ts *ts = abt->ts;

	if (lxs_ts_is_power_off(ts))
		return 1;

	if (ts->chip.reset_is_on_going)
		return 1;

	if (ts->state_core != TS_CORE_NORMAL)
		return 1;

	return 0;
}

/*
 * Ref. from 'include/uapi/asm-generic/errno.h'
 */
static const char *abt_sock_err_str(int code)
{
	struct code_str {
		int code;
		const char *str;
	} code_list[] = {
		{ -EPIPE,			"broken pipe" },
		{ -ESTRPIPE,		"streams pipe error" },
		{ -ENOTSOCK,		"socket operation on non-socket" },
		{ -EMSGSIZE,		"message too long" },
		{ -EPFNOSUPPORT,	"protocol family not supported" },
		{ -EAFNOSUPPORT,	"address family not supported" },
		{ -EADDRINUSE,		"address already in use" },
		{ -EADDRNOTAVAIL,	"can't assign requested address" },
		{ -ENETDOWN,		"network is down" },
		{ -ENETUNREACH,		"network is unreachable" },
		{ -ENETRESET,		"network dropped connection because of reset" },
		{ -ECONNABORTED,	"sw caused connection abort" },
		{ -ECONNRESET,		"connection reset by peer" },
		{ -ETIMEDOUT,		"connection timed out" },
		{ -ECONNREFUSED,	"connection refused" },
		{ -EHOSTDOWN,		"host is down" },
		{ -EHOSTUNREACH,	"no route to host" },
		{ -EALREADY,		"already in progress" },
		{ -EINPROGRESS,		"request sent but timed out" },
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(code_list); i++)
		if (code == code_list[i].code)
			return code_list[i].str;

	return "-";
}

static int __used abt_sock_msg_chk_size(struct abt_data *abt,
				int size, int boundary, char *name)
{
	if (size >= boundary)
		return 0;

	/*
	 * EAGAIN is not real error
	 * becase it happens when the server doesn't send any packet
	 */
	if (size == -EAGAIN)
		return 0;

	/*
	 * 0 means normal disconnection
	 */
	if (size)
		t_abt_err(abt, "%s failed, %d\n", name, size);

	return -1;
}

static int abt_sock_recvmsg(struct abt_data *abt,
			struct socket *sock, struct sockaddr_in *addr_in,
			unsigned char *buf, int len, int num)
{
	struct msghdr msg;
	struct kvec iov = {
		.iov_base = buf,
		.iov_len = len
	};
	int ret = 0;

	memset(&msg, 0, sizeof(struct msghdr));

	msg.msg_name = addr_in;
	msg.msg_namelen  = sizeof(struct sockaddr_in);

	ret = kernel_recvmsg(sock, &msg, &iov, num, len, 0);
	(void)abt_sock_msg_chk_size(abt, ret, 0, "kernel_recvmsg");

	return ret;
}

static int abt_sock_sendmsg(struct abt_data *abt,
			struct socket *sock, struct sockaddr_in *addr_in,
			unsigned char *buf, int len, int num)
{
	struct msghdr msg;
	struct kvec iov = {
		.iov_base = buf,
		.iov_len = len
	};
	int ret = 0;

	memset(&msg, 0, sizeof(struct msghdr));

	msg.msg_name = addr_in;
	msg.msg_namelen  = sizeof(struct sockaddr_in);

	ret = kernel_sendmsg(sock, &msg, &iov, 1, len);
	(void)abt_sock_msg_chk_size(abt, ret, 0, "kernel_sendmsg");

	return ret;
}

static int sock_create_kern_ctrl(int family, int type, int protocol, struct socket **res)
{
#if (KERNEL_VERSION(4, 2, 0) <= LINUX_VERSION_CODE)
	return sock_create_kern(&init_net, family, type, protocol, res);
#else
	return sock_create_kern(family, type, protocol, res);
#endif
}

static int __used abt_sock_create_stream(struct abt_data *abt,
				struct socket **res)
{
	int ret = 0;

	ret = sock_create_kern_ctrl(PF_INET, SOCK_STREAM, IPPROTO_TCP, res);
	if (ret < 0)
		t_abt_err(abt, "failed to create a stream socket, %d\n", ret);
	else
		t_abt_info(abt, "%s\n", "stream socket created");

	return ret;
}

static int __used abt_sock_listen(struct abt_data *abt,
				struct socket *sock, struct sockaddr_in *addr_in,
				int backlog)
{
	int ret = 0;

#if defined(__LXS_CUSTOM_BUILD_WA_FOR_KERNEL_LISTEN)
	ret = sock->ops->listen(sock, backlog);
#else
	ret = kernel_listen(sock, backlog);
#endif
	if (ret < 0) {
		t_abt_err(abt, "kernel_listen[%pI4:%d, %d](%d) failed, %d(%s)\n",
			&addr_in->sin_addr,
			__be16_to_cpu(addr_in->sin_port),
			addr_in->sin_family,
			backlog, ret, abt_sock_err_str(ret));
		return ret;
	}

	t_abt_info(abt, "kernel_listen[%pI4:%d, %d](%d) done\n",
		&addr_in->sin_addr,
		__be16_to_cpu(addr_in->sin_port),
		addr_in->sin_family,
		backlog);

	return 0;
}

static int __used abt_sock_bind(struct abt_data *abt,
				struct socket *sock, struct sockaddr_in *addr_in,
				int addrlen)
{
	int ret = 0;

	ret = kernel_bind(sock, (struct sockaddr *)addr_in, addrlen);
	if (ret < 0) {
		t_abt_err(abt, "kernel_bind[%pI4:%d, %d] failed, %d(%s)\n",
			&addr_in->sin_addr,
			__be16_to_cpu(addr_in->sin_port),
			addr_in->sin_family, ret, abt_sock_err_str(ret));
		return ret;
	}

	t_abt_info(abt, "kernel_bind[%pI4:%d, %d] done\n",
		&addr_in->sin_addr,
		__be16_to_cpu(addr_in->sin_port),
		addr_in->sin_family);

	return 0;
}

static int __used abt_sock_connect(struct abt_data *abt,
				struct socket *sock, struct sockaddr_in *addr_in,
				int addrlen, int flags)
{
	int ret = 0;

	ret = kernel_connect(sock, (struct sockaddr *)addr_in, addrlen, flags);
	if (ret < 0) {
		t_abt_err(abt, "kernel_connect[%pI4:%d, %d] failed, %d(%s)\n",
			&addr_in->sin_addr,
			__be16_to_cpu(addr_in->sin_port),
			addr_in->sin_family, ret, abt_sock_err_str(ret));
		return ret;
	}

	t_abt_info(abt, "kernel_connect[%pI4:%d, %d] done\n",
		&addr_in->sin_addr,
		__be16_to_cpu(addr_in->sin_port),
		addr_in->sin_family);

	return 0;
}

static int __used abt_sock_shutdown(struct abt_data *abt,
				struct socket *sock, struct sockaddr_in *addr_in)
{
	int ret = 0;

	ret = kernel_sock_shutdown(sock, SHUT_RDWR);
	if (ret != -ENOTCONN)
		ret = 0;

	if (ret < 0) {
		t_abt_err(abt, "kernel_sock_shutdown[%pI4:%d, %d] failed, %d\n",
			&addr_in->sin_addr,
			__be16_to_cpu(addr_in->sin_port),
			addr_in->sin_family, ret);
		return ret;
	}

	t_abt_info(abt, "kernel_sock_shutdown[%pI4:%d, %d] done\n",
		&addr_in->sin_addr,
		__be16_to_cpu(addr_in->sin_port),
		addr_in->sin_family);

	return 0;
}

static void __used abt_sock_set_timeout(struct abt_data *abt,
				struct socket *sock, long new_time_out)
{
	long old_sndtimeo, old_rcvtimeo;

	spin_lock_bh(&sock->sk->sk_lock.slock);

	if (new_time_out) {
		old_sndtimeo = sock->sk->sk_sndtimeo/HZ;
		sock->sk->sk_sndtimeo = new_time_out * HZ;

		old_rcvtimeo = sock->sk->sk_rcvtimeo/HZ;
		sock->sk->sk_rcvtimeo = new_time_out * HZ;

		t_abt_info(abt, "sock time out: %ld secs (%ld, %ld)\n",
			new_time_out, old_sndtimeo, old_rcvtimeo);
	}

	spin_unlock_bh(&sock->sk->sk_lock.slock);
}

static int __used abt_conn_is_invalid(struct abt_data *abt)
{
	int inval = !!(!abt->abt_conn_tool || (abt->abt_conn_tool >= ABT_CONN_MAX));

	if (inval)
		t_abt_warn(abt, "Invalid Tool Select, %d\n", abt->abt_conn_tool);

	return inval;
}

static int abt_ksocket_get_status(struct abt_data *abt)
{
	return atomic_read(&abt->socket_status);
}

static void abt_ksocket_set_status(struct abt_data *abt, int status)
{
	const char *str = abt_sock_status_str(status);

	t_abt_info(abt, "socket status %d(%s)\n", status, str);

	atomic_set(&abt->socket_status, status);
}

static int abt_ksocket_do_send(struct abt_data *abt,
				struct socket *sock, struct sockaddr_in *addr,
				unsigned char *buf, int len)
{
	int ret = 0;

	if (!sock) {
		t_abt_err(abt, "%s\n", "socket send err : NULL sock");
		return 0;
	}

	ret = abt_sock_sendmsg(abt, sock, addr, buf, len, 1);

	return ret;
}

static int abt_ksocket_send(struct abt_data *abt,
				struct socket *sock, struct sockaddr_in *addr,
				unsigned char *buf, int len)
{
	if (abt_conn_is_invalid(abt))
		return 0;

	return abt_ksocket_do_send(abt, sock, addr, buf, len);
}

static void __used abt_ksocket_send_disconn(struct abt_data *abt,
			struct socket *sock, struct sockaddr_in *addr_in)
{
	struct abt_comm *comm = &abt->comm;
	struct abt_comm_packet *send_pkt = comm->send_packet;

	t_abt_info(abt, "send TCP_DISCONNECT_CMD(%d)\n", TCP_DISCONNECT_CMD);

	/* send disconnect command to server */
	send_pkt->hdr.cmd = TCP_DISCONNECT_CMD;
	send_pkt->hdr.size = 4;
	abt_ksocket_send(abt, sock, addr_in, (u8 *)send_pkt,
		sizeof(struct abt_packet_hdr) + send_pkt->hdr.size);
}

static void abt_ksocket_exit_disconn(struct abt_data *abt, int thread)
{
	struct abt_comm *comm = &abt->comm;
	struct socket *sock = NULL;
	struct sockaddr_in *addr_in = NULL;
	int client_connected = abt->client_connected;
	int client_connect_trying = abt->client_connect_trying;

	if (atomic_read(&comm->running) == ABT_RUNNING_OFF)
		return;

	atomic_set(&comm->running, ABT_RUNNING_EXIT);

	switch (abt->abt_conn_tool) {
	case ABT_CONN_TOUCH:
		sock = comm->ts_sock;
		addr_in = &comm->ts_addr;
		if (!sock)
			return;

		if (client_connected || client_connect_trying)
			abt_sock_shutdown(abt, sock, addr_in);

		if (client_connect_trying)
			lxs_ts_delay(100);
		break;
	}
}

static void abt_ksocket_exit_kill(struct abt_data *abt)
{
	struct abt_comm *comm = &abt->comm;
	int cnt = 500;
	int ret = 0;

	if (comm->thread != NULL) {
		ret = kthread_stop(comm->thread);
		if (ret < 0) {
			t_abt_err(abt, "error %d while trying to terminate socket thread\n", ret);
		} else {
			u32 running = 0;

			while (cnt--) {
				running = atomic_read(&comm->running);
				if (running == ABT_RUNNING_OFF)
					break;

				t_abt_dbg_base(abt, "waiting for killing socket thread. running = %d\n", running);
				lxs_ts_delay(10);
			} while (1);
			if (cnt)
				t_abt_info(abt, "%s\n", "socket thread terminated");
			else
				t_abt_warn(abt, "%s\n", "socket thread not terminated");
		}
		comm->thread = NULL;
	}

	if (comm->sock != NULL) {
		sock_release(comm->sock);
		comm->sock = NULL;
		t_abt_info(abt, "%s\n", "sock released");
	}

	if (comm->ts_sock != NULL) {
		sock_release(comm->ts_sock);
		comm->ts_sock = NULL;
		t_abt_info(abt, "%s\n", "ts_sock released");
	}

	comm->curr_sock = NULL;
	comm->curr_addr = NULL;

	abt->abt_conn_tool = ABT_CONN_NOTHING;
}

static void abt_ksocket_exit(struct abt_data *abt, int thread)
{
	const char *str = (thread) ? "thread" : "command";

	t_abt_info(abt, "sock exit begins (%s)\n", str);

	mutex_lock(&abt->comm_lock);
	abt_ksocket_exit_disconn(abt, thread);
	mutex_unlock(&abt->comm_lock);

	if (!thread)
		t_abt_info(abt, "waiting for killing thread (%s)\n", str);

	if (!thread)
		mutex_lock(&abt->comm_lock);
	abt_ksocket_exit_kill(abt);
	if (!thread)
		mutex_unlock(&abt->comm_lock);

	t_abt_info(abt, "socket exit done (%s)\n", str);
}

static int abt_ksocket_exit_cond(struct abt_data *abt)
{
	struct abt_comm *comm = &abt->comm;

	if (atomic_read(&comm->running) != ABT_RUNNING_OFF) {
		t_abt_dbg_base(abt, "%s: running\n", __func__);
		return 1;
	}

	if (comm->thread != NULL) {
		t_abt_dbg_base(abt, "%s: thread\n", __func__);
		return 1;
	}

	if (comm->sock != NULL) {
		t_abt_dbg_base(abt, "%s: sock\n", __func__);
		return 1;
	}

	if (comm->ts_sock != NULL) {
		t_abt_dbg_base(abt, "%s: ts_sock\n", __func__);
		return 1;
	}

	if (comm->sock_send != NULL) {
		t_abt_dbg_base(abt, "%s: sock_send\n", __func__);
		return 1;
	}

	if (comm->send_connected) {
		t_abt_dbg_base(abt, "%s: send_connected\n", __func__);
		return 1;
	}

	if (comm->curr_sock != NULL) {
		t_abt_dbg_base(abt, "%s: curr_sock\n", __func__);
		return 1;
	}

	return 0;
}

static void abt_ksocket_close(struct abt_data *abt, int thread)
{
	int chk;

	if (abt->close)
		return;

	if (abt_ksocket_get_status(abt) == ABT_SOCK_STS_NONE)
		return;

	abt->close = 1;

	mutex_lock(&abt->comm_lock);
	chk = abt_ksocket_exit_cond(abt);
	mutex_unlock(&abt->comm_lock);
	if (!chk)
		return;

	abt_report_enable(abt, 0);
	abt_ksocket_exit(abt, thread);

	abt_ksocket_set_status(abt, ABT_SOCK_STS_NONE);

	abt->close = 0;
}

static int abt_ksocket_listener_touch_err(struct abt_data *abt,
				struct abt_comm_packet *recv_pkt, u32 len)
{
	const char *msg = NULL;
	int ret = 0;

	if (len < sizeof(struct abt_packet_hdr)) {
		msg = "abnormal";
		ret = -EPROTONOSUPPORT;
	} else if (len < (recv_pkt->hdr.size + sizeof(struct abt_packet_hdr))) {
		msg = "slicing";
		ret = -EPROTONOSUPPORT;
	} else if (recv_pkt->hdr.size > sizeof(recv_pkt->data.frame)) {
		msg = "overflow";
		ret = -EOVERFLOW;
	}
	if (ret < 0)
		t_abt_err(abt, "%s: %s: len %d, size %d, cmd 0x%02X, addr 0x%04X\n",
			__func__, msg, len, recv_pkt->hdr.size,
			recv_pkt->hdr.cmd, recv_pkt->hdr.addr);

	return ret;
}

static int abt_ksocket_listener_touch(void *data, u8 *buf, u32 len)
{
	struct abt_data *abt = data;
	struct lxs_ts *ts = abt->ts;
	struct abt_comm *comm = &abt->comm;
	struct abt_comm_packet *send_pkt = comm->send_packet;
	struct abt_comm_packet *recv_pkt = NULL;
	u32 buf_size = sizeof(send_pkt->data.frame);
	int use_mutex = (abt->dbg_mask & BIT(8));
	u32 reg_addr = 0;
	void *reg_data = NULL;
	u32 reg_size = 0;
	int ret = 0;

	buf_size = min_t(u32, buf_size, lxs_ts_act_buf_size(ts));

	recv_pkt = (struct abt_comm_packet *)buf;
	comm->recv_packet = recv_pkt;

	send_pkt->hdr.cmd = recv_pkt->hdr.cmd;
	send_pkt->hdr.addr = recv_pkt->hdr.addr;
	send_pkt->hdr.size = sizeof(send_pkt->data.value);
	send_pkt->data.value = COMM_RES_SUCCESS;

	ret = abt_ksocket_listener_touch_err(abt, recv_pkt, len);
	if (ret < 0) {
		send_pkt->data.value = COMM_RES_WR_FAILED;
		goto send_resp;
	}

	t_abt_dbg_trace(abt, "%s: cmd 0x%02X, addr 0x%04X, size %d\n", __func__,
		recv_pkt->hdr.cmd, recv_pkt->hdr.addr, recv_pkt->hdr.size);

	switch (recv_pkt->hdr.cmd) {
	case TCP_REG_READ:
		reg_addr = recv_pkt->hdr.addr;
		reg_data = (void *)send_pkt->data.frame;
		reg_size = recv_pkt->data.value;

		if (reg_size > buf_size) {
			t_abt_err(abt, "%s: tcp read: overflow, 0x%X > 0x%X\n",
				__func__, reg_size, buf_size);

			reg_size = min_t(u32, reg_size, sizeof(send_pkt->data.frame));

			memset(reg_data, 0, reg_size);
			send_pkt->hdr.size = reg_size;
			break;
		}

		if (abt_access_not_allowed(abt)) {
			memset(reg_data, 0, reg_size);
			send_pkt->hdr.size = reg_size;
			break;
		}

		if (use_mutex)
			mutex_lock(&ts->lock);
		ret = lxs_hal_reg_read(ts, reg_addr, reg_data, reg_size);
		if (use_mutex)
			mutex_unlock(&ts->lock);
		if (ret < 0)
			t_abt_err(abt, "%s: tcp read: reg error, %d\n", __func__, ret);

		send_pkt->hdr.size = reg_size;
		break;
	case TCP_REG_WRITE:
		reg_addr = recv_pkt->hdr.addr;
		reg_data = (void *)recv_pkt->data.frame;
		reg_size = recv_pkt->hdr.size;

		if (reg_size > buf_size) {
			t_abt_err(abt, "%s: tcp write: overflow, 0x%X > 0x%X\n",
				__func__, reg_size, buf_size);

			send_pkt->data.value = COMM_RES_WR_FAILED;
			break;
		}

		if (abt_access_not_allowed(abt)) {
			send_pkt->data.value = COMM_RES_WR_FAILED;
			break;
		}

		if (use_mutex)
			mutex_lock(&ts->lock);
		ret = lxs_hal_reg_write(ts, reg_addr, reg_data, reg_size);
		if (use_mutex)
			mutex_unlock(&ts->lock);
		if (ret < 0) {
			t_abt_err(abt, "%s: tcp write: reg error, %d\n", __func__, ret);
			send_pkt->data.value = COMM_RES_WR_FAILED;
		}
		break;
	default:
		send_pkt->data.value = COMM_RES_WR_FAILED;
		break;
	}

send_resp:
	abt_ksocket_send(abt, comm->ts_sock, &comm->ts_addr,
		(u8 *)send_pkt, sizeof(struct abt_packet_hdr) + send_pkt->hdr.size);

	return ret;
}

static int abt_ksocket_do_recv(struct abt_data *abt,
				unsigned char *buf, int len)
{
//	struct device *dev = abt->dev;
	struct abt_comm *comm = &abt->comm;
	struct socket *sock = NULL;
	struct sockaddr_in *addr = NULL;
	int size = 0;

	WARN_ON(comm->sock_listener == NULL);

	sock = comm->curr_sock;
	addr = comm->curr_addr;
	if (!sock) {
		t_abt_err(abt, "%s\n", "socket recv err : NULL sock");
		return 0;
	}

	size = abt_sock_recvmsg(abt, sock, addr, buf, len, 1);
	if (size <= 0)
		goto out;

	if (comm->sock_listener)
		comm->sock_listener((void *)abt, buf, size);

out:
	return size;
}

static int abt_ksocket_recv(struct abt_data *abt,
				unsigned char *buf, int len)
{
	if (abt_conn_is_invalid(abt))
		return -EINVAL;

	return abt_ksocket_do_recv(abt, buf, len);
}

static int abt_ksocket_thread_init(struct abt_data *abt,
				int tool, int opt)
{
	struct abt_comm *comm = &abt->comm;
	struct sockaddr_in *addr_in = NULL;
	struct socket *sock = NULL;
	__be32 s_addr = 0;
	u32 s_port = 0;
	int ret = 0;

	switch (tool) {
	case ABT_CONN_TOUCH:
		ret = abt_sock_create_stream(abt, &sock);
		if (ret < 0)
			break;

		addr_in = &comm->ts_addr;
		s_addr = in_aton(comm->send_ip);
		s_port = htons(abt->port_tcp);

		comm->ts_sock = sock;
		comm->curr_addr = &comm->ts_addr;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	if (ret < 0)
		goto out;

	comm->curr_sock = sock;

	addr_in = comm->curr_addr;

	memset(addr_in, 0, sizeof(struct sockaddr_in));
	addr_in->sin_family = PF_INET;
	addr_in->sin_addr.s_addr = s_addr;
	addr_in->sin_port = s_port;

	t_abt_info(abt, "sock init(%d) addr config: %pI4:%d, %d\n",
		opt, &addr_in->sin_addr,
		__be16_to_cpu(s_port),
		addr_in->sin_family);

	abt_sock_set_timeout(abt, sock, abt->sock_time_out);

	if (opt & ABT_SOCK_USE_BIND) {
		ret = abt_sock_bind(abt, sock,
						addr_in,
						sizeof(struct sockaddr));
		if (ret < 0)
			goto out;
	}

	if (opt & ABT_SOCK_USE_LISTEN) {
		ret = abt_sock_listen(abt, sock, addr_in, 5);
		if (ret < 0)
			goto out;
	}

	if (opt & ABT_SOCK_USE_CONN) {
		abt->client_connect_trying = 1;
		ret = abt_sock_connect(abt, sock, addr_in,
				sizeof(struct sockaddr), !O_NONBLOCK);
		abt->client_connect_trying = 0;
		if (ret < 0)
			goto out;
	}

	t_abt_info(abt, "sock init(%d) done: %pI4:%d, %d\n",
		opt, &addr_in->sin_addr,
		__be16_to_cpu(s_port),
		addr_in->sin_family);

	return 0;

out:
	t_abt_err(abt, "sock init(%d) failed, %d\n", opt, ret);
	return ret;
}

static void abt_ksocket_thread_loop(struct abt_data *abt,
				unsigned char *buf, int bufsize,
				int boundary, int delay)
{
	struct abt_comm *comm = &abt->comm;
	int size;
	int ret;

	abt_ksocket_set_status(abt, ABT_SOCK_STS_RUN);

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);

		memset(buf, 0, bufsize);

		if (atomic_read(&comm->running) != ABT_RUNNING_ON) {
			t_abt_info(abt, "%s\n", "running off");
			break;
		}

	//	mutex_lock(&abt->comm_lock);
		size = abt_ksocket_recv(abt, buf, bufsize);
	//	mutex_unlock(&abt->comm_lock);

		ret = abt_sock_msg_chk_size(abt, size, boundary,
					"abt_ksocket_recv");
		if (ret < 0)
			break;

		if (kthread_should_stop()) {
			t_abt_info(abt, "%s\n", "kthread_should_stop");
			break;
		}

		lxs_ts_delay(delay);
	}
}

static void abt_ksocket_thread_exit(struct abt_data *abt,
				struct socket **res, struct sockaddr_in *addr)
{
	struct abt_comm *comm = &abt->comm;
	struct socket *sock = *res;
	int tool = abt->abt_conn_tool;

	if (*res != NULL) {
		if (atomic_read(&comm->running) != ABT_RUNNING_EXIT)
			abt_sock_shutdown(abt, sock, addr);

		sock_release(sock);
		*res = NULL;
	}

	comm->thread = NULL;
	atomic_set(&comm->running, ABT_RUNNING_OFF);

	abt->abt_conn_tool = ABT_CONN_NOTHING;

	t_abt_info(abt, "thread exit[%d]\n", tool);
}

static int abt_ksocket_thread_for_touch(void *data)
{
	struct abt_data *abt = data;
	struct abt_comm *comm = &abt->comm;
//	struct socket *sock = NULL;
	unsigned char *buf = NULL;
	int bufsize = 0;
	int tool = abt->abt_conn_tool;
	int ret = 0;

	/* kernel thread initialization */
	atomic_set(&comm->running, ABT_RUNNING_ON);
//	comm->dev = dev;

	abt_ksocket_set_status(abt, ABT_SOCK_STS_INIT);

	ret = abt_ksocket_thread_init(abt, tool, ABT_SOCK_TOOL_OPT);
	if (ret) {
		abt_ksocket_set_status(abt, ABT_SOCK_STS_NG);
		atomic_set(&comm->running, ABT_RUNNING_EXIT);
		goto out;
	}

	abt->client_connected = 1;
	t_abt_info(abt, "TCP connected with TS (ip %s, port %d)\n",
		comm->send_ip, abt->port_tcp);

	buf = kzalloc(sizeof(struct abt_comm_packet), GFP_KERNEL);
	bufsize = sizeof(struct abt_comm_packet);

	abt_ksocket_thread_loop(abt, buf, bufsize, 1, 0);

	kfree(buf);

out:
	__set_current_state(TASK_RUNNING);

	mutex_lock(&abt->comm_lock);
	abt_ksocket_thread_exit(abt, &comm->ts_sock, &comm->ts_addr);
	mutex_unlock(&abt->comm_lock);

	abt_ksocket_close(abt, 1);

	abt->client_connected = 0;

	t_abt_info(abt, "%s terminated\n", abt_conn_name(tool));

	return ret;
}


#define abt_kthread_run(_abt, _func)	\
	({	\
		struct task_struct *_thread;	\
		t_abt_info(_abt, "thread beings: %s\n", #_func);	\
		_thread = kthread_run(_func, _abt, "abt-%s", dev_name(_abt->dev));	\
		_thread;	\
	})

static int abt_ksocket_init(struct abt_data *abt,
			char *ip, int tool, abt_sock_listener_t listener)
{
//	struct device *dev = abt->dev;
	struct abt_comm *comm = &abt->comm;
	struct task_struct *thread = NULL;

	abt->abt_conn_tool = tool;

	if (abt_conn_is_invalid(abt))
		return -EINVAL;

	t_abt_info(abt, "%s\n", "tool begins");

	memcpy(comm->send_ip, ip, ABT_SEND_IP_SIZE);

	switch (abt->abt_conn_tool) {
	case ABT_CONN_TOUCH:
		thread = abt_kthread_run(abt, abt_ksocket_thread_for_touch);
		break;
	}
	if (IS_ERR(thread)) {
		t_abt_err(abt, "%s\n", "unable to start kernel thread");
		return -ENOMEM;
	}

	comm->thread = thread;
	comm->sock_listener = listener;

	return 0;
}

static void abt_report_enable(struct abt_data *abt, u8 mode)
{
	/* TBD */
}

static int abt_tool_do_start(struct abt_data *abt,
			char *ip, int tool, abt_sock_listener_t listener)
{
	int ret = 0;

	abt->abt_report_point = 0;

	ret = abt_ksocket_init(abt, ip, tool, listener);
	if (ret) {
		t_abt_err(abt, "ksocket init[%d] failed, %d\n",
			abt->abt_conn_tool, ret);
		return ret;
	}
	abt_report_enable(abt, 1);
	return 0;
}

static int abt_tool_start(struct abt_data *abt,
			char *ip, int tool, abt_sock_listener_t listener)
{
	struct abt_comm *comm = &abt->comm;

	if (comm->thread != NULL) {
		if (!memcmp((u8 *)comm->send_ip, (u8 *)ip, ABT_SEND_IP_SIZE))
			return 0;

		t_abt_info(abt, "%s\n", "IP changed : ksocket exit, then restart");

		abt_ksocket_exit(abt, 0);
	}

	return abt_tool_do_start(abt, ip, tool, listener);
}

static int abt_store_report_touch(struct abt_data *abt, char *ip)
{
	int ret = 0;

	if (abt_access_not_allowed(abt)) {
		t_abt_err(abt, "%s: not allowed\n", __func__);
		return -EPERM;
	}

	ret = abt_tool_start(abt, ip, ABT_CONN_TOUCH,
			abt_ksocket_listener_touch);

	return ret;
}

static int abt_store_report_exit(struct abt_data *abt, char *ip)
{
	int delay = 10;
	int cnt = (abt->sock_time_out * 1000L)/delay;

	while (cnt--) {
		if (abt_ksocket_get_status(abt) != ABT_SOCK_STS_INIT)
			break;
		lxs_ts_delay(delay);
	}

	abt_ksocket_close(abt, 0);

	return 0;
}

static ssize_t abt_store_report_ctrl(struct abt_data *abt,
				const char *buf, size_t count, int opt)
{
	int mode = 0;
	char __ip[ABT_SEND_IP_SIZE+4] = { 0, };
	char *ip = NULL;
//	bool setFlag = false;
	int ret;

	if (abt_ksocket_get_status(abt) == ABT_SOCK_STS_INIT)
		return count;

	if (opt) {
		ip = __ip;
		if (sscanf(buf, "%d %15s", &mode, ip) <= 0) {
			t_abt_err(abt, "%s\n", "Invalid param");
			return count;
		}
	} else {
		mode = buf[0];
		if (mode >= '0')
			mode -= '0';

		ip = (char *)&buf[1];
	}

	if (mode && mode < ABT_CONN_MAX) {
		if (!abt_check_ipv4(abt, ip))
			return count;
	}

	t_abt_info(abt, "set abt conn mode - mode:%d, IP:%s (%d)\n", mode, ip, opt);

	switch (mode) {
	case ABT_CONN_TOUCH:
		ret = abt_store_report_touch(abt, ip);
		break;

	default:
		ret = abt_store_report_exit(abt, ip);
		break;
	}

	return count;
}

/*
 * text (echo)
 */
static ssize_t abt_show_report(struct device *dev, char *buf)
{
	struct abt_data *abt = __dev_get_abt(dev);
	struct abt_comm *comm = &abt->comm;
	int tool = abt->abt_conn_tool;
	int is_off = (atomic_read(&comm->running) == ABT_RUNNING_OFF);
	int is_run = (abt_ksocket_get_status(abt) == ABT_SOCK_STS_RUN);
	char rstr[64] = {0, };
	int size = 0;

	snprintf(rstr, sizeof(rstr) - 1, "tool:%d ip:%s%s", tool,
		(is_off) ? NONE_IP : comm->send_ip,
		(is_run) ? " (running)" : "");

	t_abt_info(abt, "%s: %s\n", __func__, rstr);

	size += lxs_snprintf(buf, size, "%s\n", rstr);

	return (ssize_t)size;
}

static ssize_t abt_store_report(struct device *dev,
				const char *buf, size_t count)
{
	struct abt_data *abt = __dev_get_abt(dev);

	t_abt_info(abt, "%s\n", __func__);

	return abt_store_report_ctrl(abt, buf, count, 1);
}
static TS_ATTR_ABT(abt_report, abt_show_report, abt_store_report);

static ssize_t abt_show_dbg_mask(struct device *dev, char *buf)
{
	struct abt_data *abt = __dev_get_abt(dev);
	int size = 0;

	size += lxs_snprintf(buf, size,
				"abt->dbg_mask  %08Xh\n",
				abt->dbg_mask);
	size += lxs_snprintf(buf, size,
				"t_abt_dbg_flag %08Xh\n",
				t_abt_dbg_flag);
	size += lxs_snprintf(buf, size,
				"t_abt_dbg_mask %08Xh\n",
				t_abt_dbg_mask);

	size += lxs_snprintf(buf, size,
				"\nUsage:\n");
	size += lxs_snprintf(buf, size,
				" abt->dbg_mask  : echo 0 {mask_value} > abt_dbg_mask\n");
	size += lxs_snprintf(buf, size,
				" t_abt_dbg_flag : echo 8 {mask_value} > abt_dbg_mask\n");
	size += lxs_snprintf(buf, size,
				" t_abt_dbg_mask : echo 9 {mask_value} > abt_dbg_mask\n");

	return (ssize_t)size;
}

static void abt_store_dbg_mask_usage(struct abt_data *abt)
{
	t_abt_info(abt, "%s\n", "Usage:");
	t_abt_info(abt, "%s\n", " abt->dbg_mask  : echo 0 {mask_value(hex)} > abt_dbg_mask");
	t_abt_info(abt, "%s\n", " t_abt_dbg_flag : echo 8 {mask_value(hex)} > abt_dbg_mask");
	t_abt_info(abt, "%s\n", " t_abt_dbg_mask : echo 9 {mask_value(hex)} > abt_dbg_mask");
}

static ssize_t abt_store_dbg_mask(struct device *dev,
				const char *buf, size_t count)
{
	struct abt_data *abt = __dev_get_abt(dev);
	int type = 0;
	u32 old_value, new_value = 0;

	if (sscanf(buf, "%d %X", &type, &new_value) <= 0) {
		t_abt_err(abt, "%s\n", "Invalid param");
		abt_store_dbg_mask_usage(abt);
		return count;
	}

	switch (type) {
	case 0:
		old_value = abt->dbg_mask;
		abt->dbg_mask = new_value;
		t_abt_info(abt, "abt->dbg_mask changed : %08Xh -> %08xh\n",
			old_value, new_value);
		break;
	case 8:
		old_value = t_abt_dbg_flag;
		t_abt_dbg_flag = new_value;
		t_abt_info(abt, "t_abt_dbg_flag changed : %08Xh -> %08xh\n",
			old_value, new_value);
		break;
	case 9:
		old_value = t_abt_dbg_mask;
		t_abt_dbg_mask = new_value;
		t_abt_info(abt, "t_abt_dbg_mask changed : %08Xh -> %08xh\n",
			old_value, new_value);
		break;
	default:
		abt_store_dbg_mask_usage(abt);
		break;
	}

	return count;
}
static TS_ATTR_ABT(abt_dbg_mask, abt_show_dbg_mask, abt_store_dbg_mask);

static struct attribute *abt_attributes[] = {
	__ts_attr_def(abt_report)
	__ts_attr_def(abt_dbg_mask)
	NULL,
};

static struct attribute_group abt_attr_group = {
	.attrs = abt_attributes,
};

static int abt_create_group(struct lxs_ts *ts)
{
	struct abt_data *abt = ts->abt;
	int ret = 0;

	ret = sysfs_create_group(&ts->kobj, &abt_attr_group);
	if (ret < 0) {
		t_abt_err(abt, "sysfs_create_group(abt) failed, %d\n", ret);
		goto out;
	}

out:
	return ret;
}

static void abt_remove_group(struct lxs_ts *ts)
{
	sysfs_remove_group(&ts->kobj, &abt_attr_group);
}

static struct abt_data *abt_alloc(struct lxs_ts *ts)
{
	struct abt_data *abt = NULL;
	struct abt_send_data *data_send = NULL;
	struct abt_comm_packet *send_packet = NULL;

	abt = kzalloc(sizeof(*abt), GFP_KERNEL);
	if (!abt) {
		t_dev_err(ts->dev, "%s: failed to allocate memory for abt\n", __func__);
		goto out;
	}

	data_send = kzalloc(sizeof(struct abt_send_data), GFP_KERNEL);
	if (!data_send) {
		t_dev_err(ts->dev, "%s: failed to allocate data_send\n", __func__);
		goto out_data_send;
	}
	abt->comm.data_send = data_send;

	send_packet = kzalloc(sizeof(struct abt_comm_packet), GFP_KERNEL);
	if (!send_packet) {
		t_dev_err(ts->dev, "%s: failed to allocate send_packet\n", __func__);
		goto out_send_packet;
	}
	abt->comm.send_packet = send_packet;

	abt->ts = ts;

	abt->dev = ts->dev;

	mutex_init(&abt->comm_lock);

	abt->abt_conn_tool = ABT_CONN_NOTHING;

	abt->sock_time_out = ABT_SOCK_TIME_OUT;

	abt->port_default = DEFAULT_PORT;
	abt->port_tcp = TS_TCP_PORT;
	abt->port_send = SEND_PORT;
	abt->port_buf_omk = OMK_BUF;

	ts->abt = abt;

	return abt;

out_send_packet:
	kfree(send_packet);

out_data_send:
	kfree(abt);

out:
	return NULL;
}

static void abt_free(struct lxs_ts *ts)
{
	struct abt_data *abt = ts->abt;

	if (abt) {
		abt_store_report_exit(abt, NULL);

		mutex_destroy(&abt->comm_lock);

		kfree(abt->comm.send_packet);
		kfree(abt->comm.data_send);
		kfree(abt);

		ts->abt = NULL;
	}
}

static void abt_create(struct lxs_ts *ts)
{
	struct abt_data *abt = ts->abt;
	int ret = 0;

	if (abt != NULL)
		return;

	abt = abt_alloc(ts);
	if (!abt)
		return;

	ret = abt_create_group(ts);
	if (ret < 0) {
		t_dev_err(ts->dev, "%s abt sysfs register failed, %d\n",
			ts->chip_name, ret);
		goto out_sysfs;
	}

	t_dev_dbg_base(ts->dev, "%s abt sysfs registered\n", ts->chip_name);

	abt->sysfs_done = 1;

	return;

out_sysfs:
	abt_free(ts);
}

static void abt_remove(struct lxs_ts *ts)
{
	struct abt_data *abt = ts->abt;

	if (abt == NULL)
		return;

	if (abt->sysfs_done)
		abt_remove_group(ts);

	abt_free(ts);

	t_dev_dbg_base(ts->dev, "%s abt sysfs unregistered\n", ts->chip_name);
}

int lxs_hal_abt_sysfs(struct lxs_ts *ts, int on_off)
{
	if (on_off)
		abt_create(ts);
	else
		abt_remove(ts);
	return 0;
}

void lxs_ts_abt_report_event(struct lxs_ts *ts, int release_all)
{
	struct abt_data *abt = ts->abt;
	int do_send = 0;

	if (abt == NULL)
		return;

	if (release_all)
		do_send |= !!(ts->old_mask);
	else
		do_send |=  !!(ts->intr_status & TS_IRQ_FINGER);

	if (!do_send)
		return;

	if (abt->report_handler)
		abt->report_handler(ts, release_all);
}
#endif	/* __LXS_SUPPORT_ABT */


