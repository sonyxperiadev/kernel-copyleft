// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* For debug only. Uncomment these blocks to log on every VERIFY statement */

/*
 * #ifndef VERIFY_PRINT_ERROR
 * #define VERIFY_PRINT_ERROR
 * #endif
 */

/*
 * #ifndef VERIFY_PRINT_INFO
 * #define VERIFY_PRINT_INFO
 * #endif
 * #define VERIFY_IPRINTF(format, ...) pr_info(format, ##__VA_ARGS__)
 */

#include <linux/uaccess.h>
#include <linux/qrtr.h>
#include <linux/mutex.h>
#include <net/sock.h>
#include "fastrpc_trace.h"
#include <linux/workqueue.h>
#include "adsprpc_shared.h"

// Registered QRTR service ID
#define FASTRPC_REMOTE_SERVER_SERVICE_ID 5012

/*
 * Fastrpc remote server instance ID bit-map:
 *
 * bits 0-1   : channel ID
 * bits 2-7   : reserved
 * bits 8-9   : remote domains (SECURE_PD, GUEST_OS)
 * bits 10-31 : reserved
 */
#define REMOTE_DOMAIN_INSTANCE_INDEX (8)
#define GET_SERVER_INSTANCE(remote_domain, cid) \
	((remote_domain << REMOTE_DOMAIN_INSTANCE_INDEX) | cid)
#define GET_CID_FROM_SERVER_INSTANCE(remote_server_instance) \
	(remote_server_instance & 0x3)

// Maximun received fastprc packet size
#define FASTRPC_SOCKET_RECV_SIZE sizeof(union rsp)

union rsp {
	struct smq_invoke_rsp rsp;
	struct smq_invoke_rspv2 rsp2;
	struct smq_notif_rspv3 rsp3;
};

enum fastrpc_remote_domains_id {
	SECURE_PD = 0,
	GUEST_OS = 1,
	MAX_REMOTE_ID = SECURE_PD + 1,
};

struct fastrpc_socket {
	struct socket *sock;                   // Socket used to communicate with remote domain
	struct sockaddr_qrtr local_sock_addr;  // Local socket address on kernel side
	struct sockaddr_qrtr remote_sock_addr; // Remote socket address on remote domain side
	struct mutex socket_mutex;             // Mutex for socket synchronization
	void *recv_buf;                        // Received packet buffer
};

struct frpc_transport_session_control {
	struct fastrpc_socket frpc_socket;     // Fastrpc socket data structure
	uint32_t remote_server_instance;       // Unique remote server instance ID
	bool remote_server_online;             // Flag to indicate remote server status
	struct work_struct work;               // work for handling incoming messages
	struct workqueue_struct *wq;           // workqueue to post @work on
};

struct remote_domain_configuration {
	int channel_id;
	int remote_domain;
};

/**
 * glist_session_ctrl
 * Static list containing socket session information for all remote domains.
 */
static struct frpc_transport_session_control *glist_session_ctrl[NUM_CHANNELS][MAX_REMOTE_ID];


static const struct remote_domain_configuration configurations[] = {
	{
		.channel_id = CDSP_DOMAIN_ID,
		.remote_domain = SECURE_PD,
	},
};

int fastrpc_set_tvm_remote_domain(struct fastrpc_file *fl, struct fastrpc_ioctl_init *init)
{
	int err = 0;
	char *proc_name = NULL;

	fl->tvm_remote_domain = GUEST_OS;
	if (init->file) {
		if (!init->filelen)
			goto bail;

		proc_name = kzalloc(init->filelen + 1, GFP_KERNEL);
		VERIFY(err, !IS_ERR_OR_NULL(proc_name));
		if (err) {
			err = -ENOMEM;
			goto bail;
		}
		err = copy_from_user((void *)proc_name,
			(void __user *)init->file, init->filelen);
		if (err) {
			err = -EFAULT;
			goto bail;
		}
		if (!strcmp(proc_name, "securepd"))
			fl->tvm_remote_domain = SECURE_PD;
	}
bail:
	kfree(proc_name);
	return err;
}

/**
 * verify_transport_device()
 * @cid: Channel ID.
 * @tvm_remote_domain: Remote domain on TVM.
 *
 * Obtain remote session information given channel ID and tvm_remote_domain
 * and verify that socket has been created and remote server is up.
 *
 * Return: 0 on success or negative errno value on failure.
 */
inline int verify_transport_device(int cid, int tvm_remote_domain)
{
	int remote_domain, err = 0;
	struct frpc_transport_session_control *session_control = NULL;

	remote_domain = tvm_remote_domain;
	VERIFY(err, remote_domain < MAX_REMOTE_ID);
	if (err) {
		err = -ECHRNG;
		goto bail;
	}

	session_control = glist_session_ctrl[cid][remote_domain];
	VERIFY(err, session_control);
	if (err) {
		err = -ECHRNG;
		goto bail;
	}

	mutex_lock(&session_control->frpc_socket.socket_mutex);
	VERIFY(err, session_control->frpc_socket.sock);
	VERIFY(err, session_control->remote_server_online);
	if (err) {
		err = -EPIPE;
		mutex_unlock(&session_control->frpc_socket.socket_mutex);
		goto bail;
	}
	mutex_unlock(&session_control->frpc_socket.socket_mutex);

bail:
	return err;
}

static void fastrpc_recv_new_server(struct frpc_transport_session_control *session_control,
				unsigned int service, unsigned int instance,
				unsigned int node, unsigned int port)
{
	uint32_t remote_server_instance = session_control->remote_server_instance;
	int32_t err = 0;

	/* Ignore EOF marker */
	if (!node && !port) {
		err = -EINVAL;
		goto bail;
	}

	if (service != FASTRPC_REMOTE_SERVER_SERVICE_ID ||
		instance != remote_server_instance) {
		err = -ENOMSG;
		goto bail;
	}

	mutex_lock(&session_control->frpc_socket.socket_mutex);
	session_control->frpc_socket.remote_sock_addr.sq_family = AF_QIPCRTR;
	session_control->frpc_socket.remote_sock_addr.sq_node = node;
	session_control->frpc_socket.remote_sock_addr.sq_port = port;
	session_control->remote_server_online = true;
	mutex_unlock(&session_control->frpc_socket.socket_mutex);
	ADSPRPC_INFO("Remote server is up: remote ID (0x%x), node %u, port %u",
				remote_server_instance, node, port);
bail:
	if (err != -EINVAL && err) {
		ADSPRPC_WARN("Ignoring ctrl packet: service id %u, instance id %u, err %d",
					service, instance, err);
	}
}

static void fastrpc_recv_del_server(struct frpc_transport_session_control *session_control,
				unsigned int node, unsigned int port)
{
	uint32_t remote_server_instance = session_control->remote_server_instance;
	int32_t err = 0;
	int32_t cid = 0;

	/* Ignore EOF marker */
	if (!node && !port) {
		err = -EINVAL;
		goto bail;
	}

	if (node != session_control->frpc_socket.remote_sock_addr.sq_node ||
		port != session_control->frpc_socket.remote_sock_addr.sq_port) {
		err = -ENOMSG;
		goto bail;
	}

	mutex_lock(&session_control->frpc_socket.socket_mutex);
	session_control->frpc_socket.remote_sock_addr.sq_node = 0;
	session_control->frpc_socket.remote_sock_addr.sq_port = 0;
	session_control->remote_server_online = false;
	mutex_unlock(&session_control->frpc_socket.socket_mutex);
	cid = GET_CID_FROM_SERVER_INSTANCE(remote_server_instance);
	ADSPRPC_INFO("Remote server is down: remote ID (0x%x)", remote_server_instance);
	fastrpc_restart_drivers(cid);
bail:
	if (err != -EINVAL && err)
		ADSPRPC_WARN("Ignoring ctrl packet: node %u, port %u, err %d", node, port, err);
}

/**
 * fastrpc_recv_ctrl_pkt()
 * @session_control: Data structure that contains information related to socket and
 *                   remote server availability.
 * @buf: Control packet.
 * @len: Control packet length.
 *
 * Handle control packet status notifications from remote domain.
 */
static void fastrpc_recv_ctrl_pkt(struct frpc_transport_session_control *session_control,
					const void *buf, size_t len)
{
	const struct qrtr_ctrl_pkt *pkt = buf;

	if (len < sizeof(struct qrtr_ctrl_pkt)) {
		ADSPRPC_WARN("Ignoring short control packet (%d bytes)", len);
		return;
	}

	switch (le32_to_cpu(pkt->cmd)) {
	case QRTR_TYPE_NEW_SERVER:
		fastrpc_recv_new_server(session_control,
				    le32_to_cpu(pkt->server.service),
				    le32_to_cpu(pkt->server.instance),
				    le32_to_cpu(pkt->server.node),
				    le32_to_cpu(pkt->server.port));
		break;
	case QRTR_TYPE_DEL_SERVER:
		fastrpc_recv_del_server(session_control,
				    le32_to_cpu(pkt->server.node),
				    le32_to_cpu(pkt->server.port));
		break;
	default:
		ADSPRPC_WARN("Ignoring unknown ctrl packet with size %zu", len);
	}
}

/**
 * fastrpc_socket_callback_wq()
 * @work: workqueue structure for incoming socket packets
 *
 * Callback function to receive responses that were posted on workqueue.
 * We expect to receive control packets with remote domain status notifications or
 * RPC data packets from remote domain.
 */
static void fastrpc_socket_callback_wq(struct work_struct *work)
{
	int32_t err = 0, cid = -1, bytes_rx = 0;
	uint32_t remote_server_instance = (uint32_t)-1;
	bool ignore_err = false;
	struct kvec msg = {0};
	struct sockaddr_qrtr remote_sock_addr = {0};
	struct msghdr remote_server = {0};
	struct frpc_transport_session_control *session_control = NULL;
	__u32 sq_node = 0, sq_port = 0;

	session_control = container_of(work, struct frpc_transport_session_control, work);
	VERIFY(err, session_control);
	if (err) {
		err = -EFAULT;
		goto bail;
	}

	remote_server.msg_name = &remote_sock_addr;
	remote_server.msg_namelen = sizeof(remote_sock_addr);
	msg.iov_base = session_control->frpc_socket.recv_buf;
	msg.iov_len = FASTRPC_SOCKET_RECV_SIZE;
	remote_server_instance = session_control->remote_server_instance;
	for (;;) {
		trace_fastrpc_msg("socket_callback_ready: begin");
		err = kernel_recvmsg(session_control->frpc_socket.sock, &remote_server, &msg, 1,
					msg.iov_len, MSG_DONTWAIT);
		if (err == -EAGAIN) {
			ignore_err = true;
			goto bail;
		}
		if (err < 0)
			goto bail;

		bytes_rx = err;
		err = 0;

		sq_node = remote_sock_addr.sq_node;
		sq_port = remote_sock_addr.sq_port;
		if (sq_node == session_control->frpc_socket.local_sock_addr.sq_node &&
			sq_port == QRTR_PORT_CTRL) {
			fastrpc_recv_ctrl_pkt(session_control,
							session_control->frpc_socket.recv_buf,
							bytes_rx);
		} else {
			cid = GET_CID_FROM_SERVER_INSTANCE(remote_server_instance);
			VERIFY(err, VALID_FASTRPC_CID(cid));
			if (err) {
				err = -ECHRNG;
				goto bail;
			}
			fastrpc_handle_rpc_response(msg.iov_base, msg.iov_len, cid);
		}
		trace_fastrpc_msg("socket_callback_ready: end");
	}
bail:
	if (!ignore_err && err < 0) {
		ADSPRPC_ERR(
			"invalid response data %pK (rx %d bytes), buffer len %d from remote ID (0x%x) err %d\n",
			msg.iov_base, bytes_rx, msg.iov_len, remote_server_instance, err);
	}
}

/**
 * fastrpc_socket_callback()
 * @sk: Sock data structure with information related to the callback response.
 *
 * Callback function to receive responses from socket layer.
 * Responses are posted on workqueue to be process.
 */
static void fastrpc_socket_callback(struct sock *sk)
{
	int32_t err = 0;
	struct frpc_transport_session_control *session_control = NULL;

	trace_fastrpc_msg("socket_callback: begin");
	VERIFY(err, sk);
	if (err) {
		err = -EFAULT;
		goto bail;
	}

	rcu_read_lock();
	session_control = rcu_dereference_sk_user_data(sk);
	if (session_control)
		queue_work(session_control->wq, &session_control->work);
	rcu_read_unlock();
bail:
	if (err < 0)
		ADSPRPC_ERR("invalid sock received, err %d", err);
	trace_fastrpc_msg("socket_callback: end");
}

/**
 * fastrpc_transport_send()
 * @cid: Channel ID.
 * @rpc_msg: RPC message to send to remote domain.
 * @rpc_msg_size: RPC message size.
 * @tvm_remote_domain: Remote domain on TVM.
 *
 * Send RPC message to remote domain. Depending on tvm_remote_domain flag message will be
 * sent to one of the remote domains on remote subsystem.
 * Depending on the channel ID and remote domain, a corresponding socket is retrieved
 * from glist_session_ctrl and is use to send RPC message.
 *
 * Return: 0 on success or negative errno value on failure.
 */
int fastrpc_transport_send(int cid, void *rpc_msg, uint32_t rpc_msg_size, int tvm_remote_domain)
{
	int err = 0, remote_domain;
	struct fastrpc_socket *frpc_socket = NULL;
	struct frpc_transport_session_control *session_control = NULL;
	struct msghdr remote_server = {0};
	struct kvec msg = {0};

	remote_domain = tvm_remote_domain;
	VERIFY(err, remote_domain < MAX_REMOTE_ID);
	if (err) {
		err = -ECHRNG;
		goto bail;
	}
	session_control = glist_session_ctrl[cid][remote_domain];
	VERIFY(err, session_control);
	if (err) {
		err = -ECHRNG;
		goto bail;
	}
	frpc_socket = &session_control->frpc_socket;
	remote_server.msg_name = &frpc_socket->remote_sock_addr;
	remote_server.msg_namelen = sizeof(frpc_socket->remote_sock_addr);

	msg.iov_base = rpc_msg;
	msg.iov_len = rpc_msg_size;

	mutex_lock(&frpc_socket->socket_mutex);
	VERIFY(err, frpc_socket->sock);
	VERIFY(err, session_control->remote_server_online);
	if (err) {
		err = -EPIPE;
		mutex_unlock(&frpc_socket->socket_mutex);
		goto bail;
	}

	err = kernel_sendmsg(frpc_socket->sock, &remote_server, &msg, 1, msg.iov_len);
	if (err > 0)
		err = 0;

	mutex_unlock(&frpc_socket->socket_mutex);
bail:
	return err;
}

/**
 * create_socket()
 * @session_control: Data structure that contains information related to socket and
 *                   remote server availability.
 *
 * Initializes and creates a kernel socket.
 *
 * Return: pointer to a socket on success or negative errno value on failure.
 */
static struct socket *create_socket(struct frpc_transport_session_control *session_control)
{
	int err = 0;
	struct socket *sock = NULL;
	struct fastrpc_socket *frpc_socket = NULL;

	err = sock_create_kern(&init_net, AF_QIPCRTR, SOCK_DGRAM,
				   PF_QIPCRTR, &sock);
	if (err < 0) {
		ADSPRPC_ERR("sock_create_kern failed with err %d\n", err);
		goto bail;
	}
	frpc_socket = &session_control->frpc_socket;
	err = kernel_getsockname(sock, (struct sockaddr *)&frpc_socket->local_sock_addr);
	if (err < 0) {
		sock_release(sock);
		ADSPRPC_ERR("kernel_getsockname failed with err %d\n", err);
		goto bail;
	}

	rcu_assign_sk_user_data(sock->sk, session_control);
	sock->sk->sk_data_ready = fastrpc_socket_callback;
	sock->sk->sk_error_report = fastrpc_socket_callback;
bail:
	if (err < 0)
		return ERR_PTR(err);
	else
		return sock;
}

/**
 * register_remote_server_notifications()
 * @frpc_socket: Socket to send message to register for remote service notifications.
 * @remote_server_instance: ID to uniquely identify remote server
 *
 * Register socket to receive status notifications from remote service
 * using remote service ID FASTRPC_REMOTE_SERVER_SERVICE_ID and instance ID.
 *
 * Return: 0 on success or negative errno value on failure.
 */
static int register_remote_server_notifications(struct fastrpc_socket *frpc_socket,
				uint32_t remote_server_instance)
{
	struct qrtr_ctrl_pkt pkt = {0};
	struct sockaddr_qrtr sq = {0};
	struct msghdr remote_server = {0};
	struct kvec msg = { &pkt, sizeof(pkt) };
	int err = 0;

	memset(&pkt, 0, sizeof(pkt));
	pkt.cmd = cpu_to_le32(QRTR_TYPE_NEW_LOOKUP);
	pkt.server.service = cpu_to_le32(FASTRPC_REMOTE_SERVER_SERVICE_ID);
	pkt.server.instance = cpu_to_le32(remote_server_instance);

	sq.sq_family = frpc_socket->local_sock_addr.sq_family;
	sq.sq_node = frpc_socket->local_sock_addr.sq_node;
	sq.sq_port = QRTR_PORT_CTRL;

	remote_server.msg_name = &sq;
	remote_server.msg_namelen = sizeof(sq);

	err = kernel_sendmsg(frpc_socket->sock, &remote_server, &msg, 1, sizeof(pkt));
	if (err < 0)
		goto bail;

bail:
	if (err < 0)
		ADSPRPC_ERR("failed to send lookup registration: %d\n", err);

	return err;
}

inline void fastrpc_transport_session_init(int cid, char *subsys)
{
}

inline void fastrpc_transport_session_deinit(int cid)
{
}

int fastrpc_wait_for_transport_interrupt(int cid, unsigned int flags)
{
	return 0;
}

void fastrpc_rproc_trace_events(const char *name, const char *event,
				const char *subevent)
{
}

/**
 * fastrpc_transport_init() - Initialize sockets for fastrpc driver.
 *
 * Initialize and create all sockets that are enabled from all channels
 * and remote domains.
 * Traverse array configurations and initialize session on glist_session_ctrl if remote
 * domain is enabled.
 *
 * Return: 0 on success or negative errno value on failure.
 */
int fastrpc_transport_init(void)
{
	int err = 0, cid = -1, ii = 0, remote_domain = -1;
	struct socket *sock = NULL;
	struct fastrpc_socket *frpc_socket = NULL;
	struct frpc_transport_session_control *session_control = NULL;
	struct workqueue_struct *wq = NULL;

	for (ii = 0; ii < ARRAY_SIZE(configurations); ii++) {
		session_control = kzalloc(sizeof(*session_control), GFP_KERNEL);
		VERIFY(err, NULL != session_control);
		if (err) {
			err = -ENOMEM;
			goto bail;
		}
		cid = configurations[ii].channel_id;
		remote_domain = configurations[ii].remote_domain;
		VERIFY(err, remote_domain < MAX_REMOTE_ID);
		if (err) {
			err = -ECHRNG;
			goto bail;
		}
		VERIFY(err, VALID_FASTRPC_CID(cid));
		if (err) {
			err = -ECHRNG;
			goto bail;
		}

		session_control->remote_server_online = false;
		frpc_socket = &session_control->frpc_socket;
		mutex_init(&frpc_socket->socket_mutex);

		sock = create_socket(session_control);
		if (IS_ERR_OR_NULL(sock)) {
			err = PTR_ERR(sock);
			goto bail;
		}

		frpc_socket->sock = sock;
		frpc_socket->recv_buf = kzalloc(FASTRPC_SOCKET_RECV_SIZE, GFP_KERNEL);
		if (!frpc_socket->recv_buf) {
			err = -ENOMEM;
			goto bail;
		}

		INIT_WORK(&session_control->work, fastrpc_socket_callback_wq);
		wq = alloc_workqueue("fastrpc_msg_handler", WQ_UNBOUND|WQ_HIGHPRI, 0);
		if (!wq) {
			err = -ENOMEM;
			goto bail;
		}
		session_control->wq = wq;

		session_control->remote_server_instance = GET_SERVER_INSTANCE(remote_domain, cid);
		err = register_remote_server_notifications(frpc_socket,
						session_control->remote_server_instance);
		if (err < 0)
			goto bail;

		glist_session_ctrl[cid][remote_domain] = session_control;
		ADSPRPC_INFO("Created and registered socket for remote server (service ID %u, instance ID 0x%x)\n",
			FASTRPC_REMOTE_SERVER_SERVICE_ID, session_control->remote_server_instance);

	}

	err = 0;
bail:
	if (err)
		ADSPRPC_ERR("fastrpc_socket_init failed with err %d\n", err);
	return err;
}

/**
 * fastrpc_transport_deinit() - Deinitialize sockets for fastrpc driver.
 *
 * Deinitialize and release all sockets that are enabled from all channels
 * and remote domains.
 * Traverse array configurations and deinitialize corresponding session from
 * glist_session_ctrl.
 */
void fastrpc_transport_deinit(void)
{
	int ii = 0, cid = -1, remote_domain = -1, err = 0;
	struct fastrpc_socket *frpc_socket = NULL;
	struct frpc_transport_session_control *session_control = NULL;

	for (ii = 0; ii < ARRAY_SIZE(configurations); ii++) {
		cid = configurations[ii].channel_id;
		remote_domain = configurations[ii].remote_domain;
		VERIFY(err, remote_domain < MAX_REMOTE_ID);
		if (err) {
			err = -ECHRNG;
			goto bail;
		}
		VERIFY(err, VALID_FASTRPC_CID(cid));
		if (err) {
			err = -ECHRNG;
			goto bail;
		}

		session_control = glist_session_ctrl[cid][remote_domain];
		if (!session_control)
			continue;

		frpc_socket = &session_control->frpc_socket;

		if (frpc_socket->sock)
			sock_release(frpc_socket->sock);

		if (session_control->wq)
			destroy_workqueue(session_control->wq);

		kfree(frpc_socket->recv_buf);
		frpc_socket->recv_buf = NULL;
		frpc_socket->sock = NULL;
		mutex_destroy(&frpc_socket->socket_mutex);
		kfree(session_control);
		glist_session_ctrl[cid][remote_domain] = NULL;
	}
bail:
	if (err)
		ADSPRPC_ERR("fastrpc_socket_deinit failed with err %d\n", err);
}
