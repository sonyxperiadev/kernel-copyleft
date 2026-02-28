// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#define pr_fmt(fmt) "smcinvoke: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/file.h>
#include <linux/dma-buf.h>
#include <linux/mem-buf.h>
#include <linux/freezer.h>
#include <linux/cdev.h>
#include <linux/version.h>
#include <linux/anon_inodes.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>

#if IS_ENABLED(CONFIG_QSEECOM_PROXY)
#include <linux/qseecom_kernel.h>
#include "misc/qseecom_priv.h"
#else
#include "misc/qseecom_kernel.h"
#endif /* CONFIG_QSEECOM_PROXY */

#include "IClientEnv.h"
# undef OBJECT_OP_METHOD_MASK

#include "smci_impl.h"

#include <linux/firmware/qcom/si_core_xts.h>

#define SMCI_DEVICE_NAME "smcinvoke"
#define SMCI_CLASS "smcinvoke-class"

static DEFINE_MUTEX(si_mutex);

static const struct file_operations qtee_fops;
static const struct file_operations server_fops;

static struct si_object_operations cbo_sio_ops;

/* 'struct cb_object' is a userspace callback object. */

struct cb_object {
	struct si_object object;

	/* If set, we will notify the userspace server about the release of object. */
	int notify_on_release;

	/* 'u_handle' + 'server_info' combo that represents this object.*/

	/* 'si' keeps a refcounted copy of server_info. If userspace server dies,
	 * we keep 'si' around until QTEE releases all instances of cb_objects.
	 */

	struct {
		s64 u_handle;

		struct server_info *si;
	};
};

#define to_cb_object(o) container_of((o), struct cb_object, object)

static int is_cb_object(struct si_object *object)
{
	/* Check 'typeof_si_object' to make sure 'object''s 'ops' has been
	 * initialized before checking it.
	 */

	return (typeof_si_object(object) == SI_OT_CB_OBJECT) &&
		(object->ops == &cbo_sio_ops);
}

/* U_HANDLE allocation and mapping to instance of 'file'. */

static int u_handle_alloc(const char *name, const struct file_operations *fops,
	void *private, s64 *u_handle)
{
	int fd;

	struct file *file;

	fd = get_unused_fd_flags(O_RDWR);
	if (fd < 0)
		return fd;

	file = anon_inode_getfile(name, fops, private, O_RDWR);
	if (IS_ERR(file)) {
		put_unused_fd(fd);

		return PTR_ERR(file);
	}

	fd_install(fd, file);
	*u_handle = fd;

	pr_debug("fd %d assigned for %s.\n", fd, name);

	return 0;
}

static struct file *get_file_of_type(int fd, const struct file_operations *fop)
{
	struct file *filp = fget(fd);

	if (!filp)
		return NULL;

	if (filp->f_op != fop) {
		fput(filp);

		return NULL;
	}

	return filp;
}

#define get_qtee_file(uh)		get_file_of_type((uh), &qtee_fops)
#define get_cb_server_file(uh)	get_file_of_type((uh), &server_fops)

/* That's how userspace distinguish object domain. But why!? */
/* 'U_HANDLE_IS_OBJECT' represents HLOS and QTEE domains. */
#define U_HANDLE_IS_OBJECT(uh)		((uh) >= 0)
#define U_HANDLE_IS_NULL(uh)		((uh) == SMCINVOKE_USERSPACE_OBJ_NULL)
#define U_HANDLE_IS_CB_OBJECT(uh)	((uh) < SMCINVOKE_USERSPACE_OBJ_NULL)

/* SI_OBJECT to/from U_HANDLE. */

/* 'get_si_object_from_u_handle': converts a u_handle to instance of si_object.
 * It calls 'get_si_object' before returning (i.e. ref == 2) for all objects
 * except 'SI_OT_USER'; One reference for QTEE and one for driver itself.
 */

static struct cb_object *cb_object_alloc(s64 server_id)
{
	struct file *filp;
	struct cb_object *cb_object;

	filp = get_cb_server_file(server_id);
	if (!filp)
		return ERR_PTR(-EBADF);

	cb_object = kzalloc(sizeof(*cb_object), GFP_KERNEL);
	if (cb_object) {

		/* Default is to always notify the callback server. */
		cb_object->notify_on_release = 1;
		cb_object->si = filp->private_data;

		/* We put 'filp' (below) while keeping the instance of 'server_info'.
		 * We want to receive the file's release callback for server if server dies.
		 * This will keep the 'server_info' instance alive as long as the cb_object exists.
		 */

		kref_get(&cb_object->si->refcount);

	} else
		cb_object = ERR_PTR(-ENOMEM);

	fput(filp);

	return cb_object;
}

static void mem_object_release(void *private);
static int get_si_object_from_u_handle(struct smcinvoke_obj *o, struct si_arg *arg)
{
	int ret = 0;

	struct si_object *object;

	/* The 'U_HANDLE' instance mapped to 'si_object'. */
	s64 u_handle = o->fd;

	if (U_HANDLE_IS_NULL(u_handle)) {
		object = NULL_SI_OBJECT;

	} else if (U_HANDLE_IS_CB_OBJECT(u_handle)) {
		struct cb_object *cb_object;

		/* We always allocate a 'cb_object' instance. 'u_handle' has meaning only
		 * in the context of callback server. Having a same 'u_handle' does not
		 * necessarily means objects are the same!
		 */

		cb_object = cb_object_alloc(o->cb_server_fd);
		if (!IS_ERR(cb_object)) {

			/* 'cb_object' represent an object in 'si'. */

			cb_object->u_handle = u_handle;
			init_si_object_user(&cb_object->object, SI_OT_CB_OBJECT, &cbo_sio_ops,
				"cbo-%s%d", cb_object->si->comm, u_handle);

			object = &cb_object->object;

			get_si_object(object);
		} else
			ret = PTR_ERR(cb_object);

	} else { /* 'U_HANDLE_IS_OBJECT'. */
		struct dma_buf *dma_buf;

		dma_buf = dma_buf_get(u_handle);
		if (!IS_ERR(dma_buf)) {

			struct cb_object *x_cb;

			/* This is an effort to simulate the invalid fix propagted deep
			 * in userspace. See: 'mem_object_release'. If IS_ERR(x_cb), we will
			 * proceed without the mem-object RELEASE but will print a warning
			 * for leak in 'mem_object_release'.
			 */

			/* BUG!!! This assumes only callback server is sending a memory object;
			 * It assumes memory object ALWAYS should belong to a server, i.e.
			 * 'cb_server_fd' MUST be valid. I can not fix it.
			 */

			x_cb = cb_object_alloc(o->cb_server_fd);
			if (!IS_ERR(x_cb))
				x_cb->u_handle = u_handle;

			object = init_si_mem_object_user(dma_buf, mem_object_release,
				IS_ERR(x_cb) ? NULL : x_cb);

			if (!object)
				ret = -EINVAL;

			get_si_object(object);

			/* 'init_si_mem_object_user' calls 'dma_buf_get' internally. */
			dma_buf_put(dma_buf);
		} else {
			struct file *filp;

			filp = get_qtee_file(u_handle);
			if (filp) {
				object = filp->private_data;

				/* We put 'filp' while keeping the instance of object. */
				get_si_object(object);

				fput(filp);
			} else
				ret = -EINVAL;
		}
	}

	if (ret) {

		/* Return 'NULL' as default. */

		object = NULL_SI_OBJECT;
	}

	arg->o = object;

	return ret;
}

/* 'get_u_handle_from_si_object': converts 'object' to 'u_handle'.
 * On SUCCESS, it calls 'put_si_object' before returning for all objects except
 * 'SI_OT_USER'.
 */

static int get_u_handle_from_si_object(struct si_object *object,
	s64 *u_handle, struct server_info **si)
{
	int ret = 0;

	if (si)
		*si = NULL;

	switch (typeof_si_object(object)) {
	case SI_OT_NULL:
		*u_handle = SMCINVOKE_USERSPACE_OBJ_NULL;

		break;
	case SI_OT_CB_OBJECT: {

		/* Check for memory object or an object in a callback server!? */

		if (is_mem_object(object)) {
			struct dma_buf *dma_buf = mem_object_to_dma_buf(object);

			get_dma_buf(dma_buf);
			*u_handle = dma_buf_fd(dma_buf, O_CLOEXEC);
			if (*u_handle < 0) {
				dma_buf_put(dma_buf);

				ret = -EBADF;
			} else
				put_si_object(object);

		} else {
			struct cb_object *cb_object = to_cb_object(object);

			*u_handle = cb_object->u_handle;

			if (si)
				*si = cb_object->si;

			/* We put 'object'. 'u_handle' has meaning only in the context
			 * of callback server. QTEE will not release it as long as a request
			 * is in progress in a callback server.
			 */

			put_si_object(object);
		}
	}

		break;
	case SI_OT_USER:

		/* On SUCCESS, we do not put 'object' unlike object of 'SI_OT_CB_OBJECT'
		 * type. Because, it is used by file's 'private_data'.
		 */

		if (u_handle_alloc(si_object_name(object), &qtee_fops, object, u_handle))
			ret = -EBADF;

		break;
	case SI_OT_ROOT:
	default:
		ret = -EBADF;

		break;
	}

	if (ret)
		*u_handle = SMCINVOKE_USERSPACE_OBJ_NULL;

	return ret;
}

/* Marshaling API. */
/* 'marshal_in_req' Prepare input buffer for sending to QTEE.
 * 'marshal_out_req' Parse QTEE response in input buffer.
 * 'marshal_in_cb_req' Parse QTEE request from output buffer.
 * 'marshal_out_cb_req' Update output buffer with response for QTEE request.
 *
 * 'marshal_in_req' and 'marshal_out_req' are used in direct invocation path.
 * 'marshal_in_cb_req' and 'marshal_out_cb_req' are used for QTEE request.
 */

static void marshal_in_req_cleanup(struct si_arg u[], int notify)
{
	int i;
	struct si_object *object;

	for (i = 0; u[i].type; i++) {
		switch (u[i].type) {
		case SI_AT_IO:

			object = u[i].o;

			/* For cb_objects, we will notify userspace of its release.
			 * On failure, we should not do that.
			 */

			if (is_cb_object(object))
				to_cb_object(object)->notify_on_release = notify;

			/* For object of type SI_OT_USER, 'get_si_object_from_u_handle' does
			 * not call 'get_si_object' before returning (i.e. ref == 1). Replace
			 * it with NULL_SI_OBJECT as after 'put_si_object', u[i].o is invalid.
			 */

			else if (typeof_si_object(object) == SI_OT_USER)
				u[i].o = NULL_SI_OBJECT;

			put_si_object(object);

			break;
		case SI_AT_IB:
		case SI_AT_OB:
		case SI_AT_OO:
		default:

			break;
		}
	}
}

static int marshal_in_req(struct si_arg u[], union smcinvoke_arg args[], u32 counts)
{
	int i, err = 0;

	FOR_ARGS(i, counts, BI) {
		u[i].type = SI_AT_IB;
		u[i].flags = SI_ARG_FLAGS_UADDR;
		u[i].b.uaddr = u64_to_user_ptr(args[i].b.addr);
		u[i].b.size = args[i].b.size;
	}

	FOR_ARGS(i, counts, BO) {
		u[i].type = SI_AT_OB;
		u[i].flags = SI_ARG_FLAGS_UADDR;
		u[i].b.uaddr = u64_to_user_ptr(args[i].b.addr);
		u[i].b.size = args[i].b.size;
	}

	FOR_ARGS(i, counts, OI) {
		u[i].type = SI_AT_IO;

		if (!err)
			err = get_si_object_from_u_handle(&args[i].o, &u[i]);
		else
			u[i].o = NULL_SI_OBJECT;
	}

	FOR_ARGS(i, counts, OO)
		u[i].type = SI_AT_OO;

	u[i].type = SI_AT_END;

	if (!err)
		return 0;

	/* Release whatever resources we got in 'u'. */
	/* Return with clean slate. */

	marshal_in_req_cleanup(u, 0);

	/* Here, drop QTEE istances; on Success QTEE does that. */

	for (i = 0; u[i].type; i++) {
		if (u[i].type == SI_AT_IO &&
			typeof_si_object(u[i].o) != SI_OT_USER)
			put_si_object(u[i].o);
	}

	return -1;
}

/* 'marshal_out_req' consumes 'u' as initialized by 'marshal_in_req' to fill 'args'. */

static int marshal_out_req(union smcinvoke_arg args[], struct si_arg u[])
{
	int i = 0, err = 0;

	while (u[i].type == SI_AT_IB)
		i++;

	while (u[i].type == SI_AT_OB) {
		args[i].b.size = u[i].b.size;

		i++;
	}

	while (u[i].type == SI_AT_IO) {

		/* 'get_si_object_from_u_handle' increases the refcount of all si_object
		 * instances. We drop the temporary reference.
		 */

		put_si_object(u[i].o);

		i++;
	}

	while (u[i].type == SI_AT_OO) {

		/* 'get_u_handle_from_si_object' convert the si_object instances to a
		 * u_handle and put the reference counter. On failure, we do not need to
		 * continue with heavy work anymore.
		 */

		if (err) {

			/* Last iteration FAILED. Use an easy way out. */

			args[i].o.fd = SMCINVOKE_USERSPACE_OBJ_NULL;

			put_si_object(u[i].o);
		} else if (get_u_handle_from_si_object(u[i].o, &args[i].o.fd, NULL)) {
			put_si_object(u[i].o);

			err = -1;
		}

		i++;
	}

	if (!err)
		return 0;

	/* Release whatever resources we got in 'args'! */
	/* Return with clean slate. */

	for (i = 0; u[i].type; i++) {
		if (u[i].type == SI_AT_OO) {
			if (U_HANDLE_IS_OBJECT(args[i].o.fd)) {

				/* This is a file descriptor. Need cleanup. */
				/* TODO. Cleanup exported object. */

			}
		}
	}

	return -1;
}

/* We assume 'u' is sorted so 'SI_AT_IB', 'SI_AT_OB', following by 'SI_AT_IO', and 'SI_AT_OO'. */

static int marshal_in_cb_req(union smcinvoke_arg args[], void __user *u_args,
	u32 *counts, struct si_arg u[], struct server_info *target_si)
{
	int i = 0, err = 0;
	int ib = 0, ob = 0, io = 0, oo = 0;

	size_t offset = 0;

	for (; u[i].type == SI_AT_IB; i++) {
		void __user *u_addr;

		if (!err) {
			args[i].b.addr = (u64) (u_args + offset);
			args[i].b.size = u[i].b.size;

			u_addr = u64_to_user_ptr(args[i].b.addr);

			if (copy_to_user(u_addr, u[i].b.addr, u[i].b.size))
				err = -1;

			offset = ALIGN(offset + u[i].b.size, 8);
		}
	}

	ib = i;

	for (; u[i].type == SI_AT_OB; i++) {
		if (!err) {
			args[i].b.addr = (u64) (u_args + offset);
			args[i].b.size = u[i].b.size;

			offset = ALIGN(offset + u[i].b.size, 8);
		}
	}

	ob = i;

	for (; u[i].type == SI_AT_IO; i++) {
		struct server_info *si;

		if (!err) {
			if (get_u_handle_from_si_object(u[i].o, &args[i].o.fd, &si)) {
				put_si_object(u[i].o);

				err = -1;
			}

			if (target_si && si && si != target_si)
				err = -1;
		} else {

			/* Last iteration FAILED. Use an easy way out. */

			args[i].o.fd = SMCINVOKE_USERSPACE_OBJ_NULL;

			put_si_object(u[i].o);
		}
	}

	io = i;

	for (; u[i].type == SI_AT_OO; i++)
		;

	oo = i;

	if (!err) {
		*counts = 0;
		*counts |= ((oo - io) & 0xFU) << 12;	/* No. Output Objects. */
		*counts |= ((io - ob) & 0xFU) << 8;		/* No. Input Objects. */
		*counts |= ((ob - ib) & 0xFU) << 4;		/* No. Output Buffer. */
		*counts |= (ib & 0xFU);					/* No. Input Buffer. */

		return 0;
	}

	/* Release whatever resources we got in 'args'! */
	/* Return with clean slate. */

	for (i = 0; u[i].type; i++) {
		if (u[i].type == SI_AT_IO) {
			if (U_HANDLE_IS_OBJECT(args[i].o.fd)) {

				/* This is a file descriptor. Need cleanup. */
				/* TODO. Cleanup exported object. */

			}
		}
	}

	return -1;
}

static int marshal_out_cb_req(struct si_arg u[], union smcinvoke_arg args[])
{
	int i = 0, err = 0;

	while (u[i].type == SI_AT_IB)
		i++;

	while (u[i].type ==  SI_AT_OB) {
		void __user *u_addr;

		if (args[i].b.size <= u[i].b.size) {
			u[i].b.size = args[i].b.size;
			u_addr = u64_to_user_ptr(args[i].b.addr);

			if (copy_from_user(u[i].b.addr, u_addr, args[i].b.size))
				return -1;
		} else
			return -1;

		i++;
	}

	while (u[i].type == SI_AT_IO)
		i++;

	while (u[i].type == SI_AT_OO) {

		/* Client is not aware if a callback has been terminated.
		 * We temporarily hold a reference for objects as the callback is
		 * in progress. We drop these references in the objects' notification.
		 */

		if (err) {

			/* On failure, we do not need to continue with heavy work anymore. */

			u[i].o = NULL_SI_OBJECT;
		} else
			err = get_si_object_from_u_handle(&args[i].o, &u[i]);

		i++;
	}

	if (!err)
		return 0;

	/* Release whatever resources we got in 'u'. */
	/* Return with clean slate. */

	for (i = 0; u[i].type; i++) {
		switch (u[i].type) {
		case SI_AT_OO:

			/* For cb_objects, we will notify userspace of its release.
			 * On failure, we should not do that.
			 */

			if (is_cb_object(u[i].o))
				to_cb_object(u[i].o)->notify_on_release = 0;

			/* 'get_si_object_from_u_handle' calls 'get_si_object' before
			 * returning (i.e. ref == 2) for all objects except SI_OT_USER.
			 * One reference for QTEE and one for driver itself.
			 */

			if (typeof_si_object(u[i].o) != SI_OT_USER)
				put_si_object(u[i].o);

			put_si_object(u[i].o);

			break;
		case SI_AT_IB:
		case SI_AT_OB:
		case SI_AT_IO:
		default:

			break;
		}
	}

	return -1;
}

/* Transaction management. */

static struct cb_txn *txn_alloc(int u_args_n)
{
	struct cb_txn *t;

	t = kzalloc(sizeof(*t), GFP_KERNEL);
	if (!t)
		return ERR_PTR(-ENOMEM);

	/* 'u_args' is a memory used by 'process_accept_req'. Allocate it here. */

	t->u_args_sz = u_args_n * sizeof(t->u_args[0]);
	if (t->u_args_sz) {
		t->u_args = kzalloc(t->u_args_sz, GFP_KERNEL);
		if (!t->u_args) {
			kfree(t);

			return ERR_PTR(-ENOMEM);
		}
	}

	t->processing = XST_NEW;
	kref_init(&t->refcount);
	init_completion(&t->completion);
	INIT_LIST_HEAD(&t->node);

	return t;
}

static void txn_free(struct cb_txn *cb_txn)
{
	int i;
	struct si_arg *args = cb_txn->args;

	if (args) {
		for (i = 0; i < size_of_arg(args); i++) {
			if (args[i].type == SI_AT_IB ||
				args[i].type == SI_AT_OB)
				kfree(args[i].b.addr);
		}

		kfree(args);
	}

	kfree(cb_txn->u_args);
	kfree(cb_txn);
}

static int queue_txn(struct server_info *si, struct cb_txn *cb_txn)
{
	int dead;

	mutex_lock(&si_mutex);
	dead = si->dead;
	if (!dead) {
		list_add(&cb_txn->node, &si->cb_tx_list);
		cb_txn->processing = XST_PENDING;
	}
	mutex_unlock(&si_mutex);

	return dead;
}

static void dequeue_txn_locked(struct cb_txn *cb_txn)
{
	list_del_init(&cb_txn->node);
}

static struct cb_txn *dequeue_txn_by_id(struct server_info *si, unsigned int id)
{
	struct cb_txn *t;

	mutex_lock(&si_mutex);
	list_for_each_entry(t, &si->cb_tx_list, node)
		if (t->context_id == id) {
			dequeue_txn_locked(t);

			goto found;
		}

	t = NULL;

found:
	mutex_unlock(&si_mutex);

	return t;
}

static int set_txn_state_locked(struct cb_txn *cb_txn, enum state state)
{
	/* Supported state transitions:
	 * PENDING    -> PROCESSING, XST_TIMEDOUT.
	 * PROCESSING -> PROCESSED, XST_TIMEDOUT.
	 */

	/* Moving to PROCESSING state; we should be in PENDING state. */
	if (state == XST_PROCESSING) {
		if (cb_txn->processing != XST_PENDING)
			return -1;

	/* Moving to PROCESSED state; we should be in PROCESSING state. */
	} else if (state == XST_PROCESSED) {
		if (cb_txn->processing != XST_PROCESSING)
			return -1;

	/* Moving to TIMEDOUT state; we should not be in PROCESSED state. */
	} else if (state == XST_TIMEDOUT) {
		if (cb_txn->processing != XST_PENDING &&
			cb_txn->processing != XST_PROCESSING)
			return -1;

	} else
		return -1;

	cb_txn->processing = state;

	return 0;
}

static struct cb_txn *get_txn_for_state_transition_locked(struct server_info *si,
	unsigned int id, enum state state)
{
	struct cb_txn *t, *cb_txn = NULL;

	/* Supported state transitions:
	 * PENDING    -> PROCESSING.
	 * PROCESSING -> PROCESSED.
	 *
	 * We only search for these possible transtitions; other state, i.e.
	 * TIMEDOUT is insignificant.
	 */

	list_for_each_entry(t, &si->cb_tx_list, node) {

		/* Search for a specific transaction with a particular state?! */
		if ((id != CONTEXT_ID_ANY) && (t->context_id != id))
			continue;

		/* Moving to PROCESSING state; we should be in PENDING state. */
		if (state == XST_PROCESSING) {
			if (t->processing == XST_PENDING) {
				cb_txn = t;
				break;
			}

		/* Moving to PROCESSED state; we should be in PROCESSING state. */
		} else if (state == XST_PROCESSED) {
			if (t->processing == XST_PROCESSING) {
				cb_txn = t;
				break;
			}

		} else {
			pr_err("invalid state transition %d\n", state);

			break;
		}
	}

	if (cb_txn && !kref_get_unless_zero(&cb_txn->refcount))
		cb_txn = NULL;

	return cb_txn;
}

static struct cb_txn *get_txn_for_state_transition(struct server_info *si,
	unsigned int context_id, enum state state)
{
	struct cb_txn *cb_txn;

	mutex_lock(&si_mutex);
	cb_txn = get_txn_for_state_transition_locked(si, context_id, state);
	mutex_unlock(&si_mutex);

	return cb_txn;
}

static int set_txn_state(struct cb_txn *cb_txn, enum state state)
{
	int ret;

	mutex_lock(&si_mutex);
	ret = set_txn_state_locked(cb_txn, state);
	mutex_unlock(&si_mutex);

	return ret;
}

static void __release_txn(struct kref *refcount)
{
	struct cb_txn *cb_txn = container_of(refcount, struct cb_txn, refcount);

	txn_free(cb_txn);
}

static void put_txn(struct cb_txn *cb_txn)
{
	kref_put(&cb_txn->refcount, __release_txn);
}

static void dequeue_and_put_txn(struct cb_txn *cb_txn)
{
	mutex_lock(&si_mutex);
	/* Only if it is queued. */
	if (cb_txn->processing != XST_NEW)
		dequeue_txn_locked(cb_txn);
	mutex_unlock(&si_mutex);

	put_txn(cb_txn);
}

/* 'wait_for_pending_txn' picks the next available pending transaction or sleep. */

static int wait_for_pending_txn(struct server_info *si, struct cb_txn **cb_txn)
{
	int ret = 0;
	struct cb_txn *t = NULL;

	DEFINE_WAIT_FUNC(wait, woken_wake_function);

	add_wait_queue(&si->Q, &wait);
	while (1) {
		if (signal_pending(current)) {
			ret = -ERESTARTSYS;

			break;
		}

		mutex_lock(&si_mutex);
		t = get_txn_for_state_transition_locked(si, CONTEXT_ID_ANY, XST_PROCESSING);
		if (t) {
			/* ''PENDING -> PROCESSING'' is atomic so it is not picked up again. */
			set_txn_state_locked(t, XST_PROCESSING);
			mutex_unlock(&si_mutex);

			break;
		}
		mutex_unlock(&si_mutex);

		wait_woken(&wait, TASK_INTERRUPTIBLE, MAX_SCHEDULE_TIMEOUT);
	}

	remove_wait_queue(&si->Q, &wait);
	*cb_txn = t;

	return ret;
}

/* Callback object's operations. */

static void ____destroy_server_info(struct kref *kref);

/* 'dispatcher_marshal_in' and 'dispatcher_marshal_out'. */

/* The only reason they are here are due to UAPI design. */
/* The marshaling with 'marshal_in_cb_req' and 'marshal_out_cb_req' should happen
 * in dispatcher (1) as it's overhead should be attributed to the invoke thread,
 * (2) marshaling form args to transaction is relevant here. However, the UAPI,
 * requires addrss (not the offset) which is anly available in 'process_accept_req'.
 */

static struct si_arg *dispatcher_marshal_in(struct si_arg args[])
{
	struct si_arg *cb_txn_args;
	int i, nargs = size_of_arg(args);

	/* Plus one for 'SI_AT_END'. */
	cb_txn_args = kcalloc(nargs + 1, sizeof(struct si_arg), GFP_KERNEL);
	if (!cb_txn_args)
		return NULL;

	for (i = 0; i < nargs; i++) {
		cb_txn_args[i].type = args[i].type;

		if (args[i].type == SI_AT_IB) {
			cb_txn_args[i].b.addr = kzalloc(args[i].b.size, GFP_KERNEL);
			if (!cb_txn_args[i].b.addr)
				goto failed;

			cb_txn_args[i].b.size = args[i].b.size;

			memcpy(cb_txn_args[i].b.addr, args[i].b.addr, args[i].b.size);
		} else if (args[i].type == SI_AT_OB) {
			cb_txn_args[i].b.addr = kzalloc(args[i].b.size, GFP_KERNEL);
			if (!cb_txn_args[i].b.addr)
				goto failed;

			cb_txn_args[i].b.size = args[i].b.size;
		} else /* 'SI_AT_IO' || 'SI_AT_OO'. */
			cb_txn_args[i].o = args[i].o;

	}

	return cb_txn_args;

failed:

	for (i = 0; i < nargs; i++) {
		if (cb_txn_args[i].type == SI_AT_IB ||
			cb_txn_args[i].type == SI_AT_OB)
			kfree(cb_txn_args[i].b.addr);
	}

	kfree(cb_txn_args);

	return NULL;
}

static void dispatcher_marshal_out(struct si_arg cb_txn_args[], struct si_arg args[])
{
	int i, nargs = size_of_arg(args);

	/* See 'marshal_out_cb_req'. */
	for (i = 0; i < nargs; i++) {
		if (args[i].type == SI_AT_OB) {
			args[i].b.size = cb_txn_args[i].b.size;

			memcpy(args[i].b.addr, cb_txn_args[i].b.addr, cb_txn_args[i].b.size);
		} else if (args[i].type == SI_AT_OO)
			args[i].o = cb_txn_args[i].o;
	}
}

static int cbo_dispatch(unsigned int context_id,
	struct si_object *object, unsigned long op, struct si_arg args[])
{
	struct cb_txn *cb_txn;
	struct cb_object *cb_object = to_cb_object(object);

	int errno, nargs = size_of_arg(args);

	cb_txn = txn_alloc(nargs);
	if (IS_ERR(cb_txn))
		return PTR_ERR(cb_txn);

	/* 'context_id' is unique. Let's use it fot transaction ID. */

	cb_txn->context_id = context_id;
	cb_txn->op = op;
	cb_txn->u_handle = cb_object->u_handle;

	/* TODO. Move this to 'txn_alloc'. */
	cb_txn->args = dispatcher_marshal_in(args);
	if (!cb_txn->args) {
		put_txn(cb_txn);

		return -EINVAL;
	}

	/* START a Transaction. */

	pr_debug("%s invocation with %d arguments and op %lu (context_id %u).\n",
		si_object_name(object), nargs, cb_txn->op, context_id);

	if (queue_txn(cb_object->si, cb_txn)) {
		put_txn(cb_txn);

		return -EINVAL;
	}

	wake_up_interruptible_all(&cb_object->si->Q);

	if (context_id == CONTEXT_ID_ANY) {

		/* We submitted a transaction with 'CONTEXT_ID_ANY' context ID.
		 * This means no one is waiting for this callback request; just return.
		 * 'process_accept_req' does the cleanup after processing this request.
		 */

		return 0;
	}

	/* Wait in FREEZABLE state in case we should suspend, here. */
	wait_for_completion_state(&cb_txn->completion, TASK_KILLABLE | TASK_FREEZABLE);

	/* We do not care why 'wait_for_completion_state' returend. The fastest way
	 * to exit the dispatcher is to TIMEOUT the transaction. However, if 'set_txn_state'
	 * failed, then transaction has already been PROCESSED.
	 */

	errno = set_txn_state(cb_txn, XST_TIMEDOUT) ? cb_txn->errno : -EINVAL;
	pr_debug("%s invocation returned with %d (context_id %u).\n",
		si_object_name(object), errno, context_id);
	if (!errno)
		dispatcher_marshal_out(cb_txn->args, args);
	else
		dequeue_and_put_txn(cb_txn);

	return errno;
}

static void cbo_notify(unsigned int context_id, struct si_object *object, int status)
{
	struct cb_txn *cb_txn;

	cb_txn = dequeue_txn_by_id(to_cb_object(object)->si, context_id);
	if (cb_txn) {
		int i;
		struct si_arg *u = cb_txn->args;

		for (i = 0; u[i].type; i++) {
			switch (u[i].type) {
			case SI_AT_OO:

				/* See 'marshal_out_cb_req'. */

				/* 'get_si_object_from_u_handle' calls 'get_si_object'
				 * before returning (i.e. ref == 2) for all objects
				 * except SI_OT_USER. One reference for QTEE and one
				 * for driver itself. If !status, then QTEE owns an
				 * instance of the object so we call 'put_si_object'
				 * once.
				 */

				if (status && (typeof_si_object(u[i].o) != SI_OT_USER))
					put_si_object(u[i].o);

				put_si_object(u[i].o);

				break;
			case SI_AT_IB:
			case SI_AT_OB:
			case SI_AT_IO:
			default:

				break;
			}
		}

		put_txn(cb_txn);
	}
}

static void cbo_release(struct si_object *object)
{
	struct cb_object *cb_object = to_cb_object(object);

	if (cb_object->notify_on_release) {
		static struct si_arg args[] = { { .type = 0 } };

		/* Use 'CONTEXT_ID_ANY' as context ID; as we do not care about the results. */
		cbo_dispatch(CONTEXT_ID_ANY, object, SI_OBJECT_OP_RELEASE, args);
	}

	/* The matching 'kref_get' is in 'cb_object_alloc'. */
	kref_put(&cb_object->si->refcount, ____destroy_server_info);
	kfree(cb_object);
}


static void mem_object_release(void *private)
{
	/* THIS IS A STUPIDEST IMPLEMENTATION EVER! */

	/* Old smcinvoke driver had a memory leak but rather than fixing it by
	 * figuring out the root cause they decided it is COOL to send irrelevant
	 * memory release to userspace to compensate for the leaked memory. Interestingly,
	 * it has a made up story behind to justify the wrong change, see comments in
	 * 'process_tzcb_req' this patch:
	 * https://review-android.quicinc.com/c/platform/vendor/qcom/opensource/securemsm-kernel/+/4382238.
	 * I WISH I COULD HAVE REVERTED THIS.
	 */

	struct cb_object *cb_x = private;

	if (cb_x) {

		/* Note 'cb_x->object' has not been isinialized. Do not use it! */
		pr_debug("dma_buf released i.e. cbo-%s%lld\n", cb_x->si->comm, cb_x->u_handle);

		cbo_release(&cb_x->object);
	} else
		pr_err("memory leak detected!\n");
}

static struct si_object_operations cbo_sio_ops = {
	.release = cbo_release,
	.notify = cbo_notify,
	.dispatch = cbo_dispatch,
};

/* */
/* We support three types of files:
 *   - 'qtee_fops'
 *   - 'server_fops'
 *   - 'root_fops'.
 */

/* User Callback server */

static long process_accept_req(struct server_info *si, struct smcinvoke_accept *accept)
{
	struct cb_txn *cb_txn;

	if (accept->argsize != sizeof(union smcinvoke_arg))
		return -EINVAL;

	/* Processing response ... */
	if (accept->has_resp) {
		int i, errno = 0;

		pr_debug("%s submit response (context_id %llu)\n", si->comm, accept->txn_id);

		/* 'CONTEXT_ID_ANY' context ID?! Ignore. */
		if (!accept->txn_id)
			goto wait_on_request;

		cb_txn = get_txn_for_state_transition(si, accept->txn_id, XST_PROCESSED);
		if (!cb_txn) {

			/* We get here, if the invoke thread goes away, e.g. timed out or killed. */
			/* In correct implementation we should return to userspace for the callback
			 * server to cleanup.
			 */

			goto wait_on_request;
		}

		errno = accept->result;
		if (!errno) {
			/* Only parse arguments on SUCCESS. */

			/* Only copy arguments that we expect, i.e. 'u_args_sz'. */
			if (copy_from_user(cb_txn->u_args,
				(void __user *)accept->buf_addr, cb_txn->u_args_sz)) {

				errno = -EFAULT;
			} else {
				if (marshal_out_cb_req(cb_txn->args, cb_txn->u_args))
					errno = -EINVAL;
			}
		}

		cb_txn->errno = errno;

		/* Try to notify the invoke thread. */
		if (set_txn_state(cb_txn, XST_PROCESSED)) {

			/* If 'set_txn_state' fails, e.g. invoke thread TIMEDOUT
			 * undo 'marshal_out_cb_req' only on SUCCESS.
			 */
			if (!errno) {
				struct si_arg *u = cb_txn->args;

				/* See comments in 'marshal_out_cb_req'. */

				for (i = 0; u[i].type; i++) {
					if (u[i].type != SI_AT_OO)
						continue;

					/* u[i].type == SI_AT_OO. */

					if (is_cb_object(u[i].o))
						to_cb_object(u[i].o)->notify_on_release = 0;

					if (typeof_si_object(u[i].o) != SI_OT_USER)
						put_si_object(u[i].o);

					put_si_object(u[i].o);
				}
			}

		} else
			complete(&cb_txn->completion);

		put_txn(cb_txn);

		if (errno && !accept->result)
			goto wait_on_request;


		/* SUCCESS submitting the response. */
	}

	/* Processing request ... */
wait_on_request:

	do {
		if (wait_for_pending_txn(si, &cb_txn)) {
			pr_debug("%s received a signal.\n", si->comm);

			return -ERESTARTSYS;
		}

		/* CONSTRUCT a request. */

		accept->cbobj_id = cb_txn->u_handle;
		accept->op = cb_txn->op;
		accept->txn_id = cb_txn->context_id;

		if (marshal_in_cb_req(cb_txn->u_args,
			(void __user *)ALIGN(accept->buf_addr + cb_txn->u_args_sz, 8),
				&accept->counts, cb_txn->args, si)) {

			goto out_failed;
		}

		pr_debug("%s pick request with arguments (%04x) and op %d (context_id %llu)\n",
			si->comm, accept->counts, accept->op, accept->txn_id);

		if (copy_to_user((void __user *)accept->buf_addr,
			cb_txn->u_args, cb_txn->u_args_sz)) {

			/* TODO. We need to do some cleanup for 'marshal_in_cb_req'. */

			goto out_failed;
		}

		/* SUCCESS picking a request; return. */
		break;

out_failed:

		/* FAILED parsing a request; notify QTEE (if necessary) and try another one. */
		if (!accept->txn_id)
			dequeue_and_put_txn(cb_txn);
		else
			complete(&cb_txn->completion);

		put_txn(cb_txn);

	} while (1);

	if (!accept->txn_id)
		dequeue_and_put_txn(cb_txn);

	put_txn(cb_txn);

	return 0;
}

static long server_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret;
	struct server_info *si = filp->private_data;

	switch (cmd) {
	case SMCINVOKE_IOCTL_ACCEPT_REQ: {
		struct smcinvoke_accept accept;

		if (_IOC_SIZE(cmd) != sizeof(accept))
			return -EINVAL;

		if (copy_from_user(&accept, (void __user *)arg, sizeof(accept)))
			return -EFAULT;

		ret = process_accept_req(si, &accept);
		if (ret == -ERESTARTSYS) {
			struct smcinvoke_accept __user *a = (struct smcinvoke_accept __user *)arg;

			/* BAD IOCTL UAPI DESIGN! */

			/* We do this because same IOCTL command has been used for two different
			 * purposes (submit response + pick request). 'ERESTARTSYS' means we were
			 * handling second part of the IOCTL when signal arrived.
			 */

			/* We need to reset 'has_resp' so if the IOCTL call restarted we
			 * resume from second half of the IOCTL. I did not use an state for 'si'
			 * as restart is not guaranteed.
			 */

			if (put_user(0, &a->has_resp))
				return -EFAULT;

		} else if (!ret) {

			/* We picked a request; and submitted any pending response.*/
			accept.has_resp = 0;

			if (copy_to_user((void __user *)arg, &accept, sizeof(accept))) {

				/* TODO. We need to do some cleanup for 'process_accept_req'. */

				return -EFAULT;
			}
		}

		break;
	}
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

static int server_release(struct inode *nodp, struct file *filp)
{
	struct cb_txn *cb_txn, *t;

	struct server_info *si = filp->private_data;

	/* We get here when there is no more server thread running in kernel. */

	mutex_lock(&si_mutex);
	si->dead = 1;

	/* Terminate any PENDING or PROCESSING transaction. */
	list_for_each_entry_safe(cb_txn, t, &si->cb_tx_list, node) {
		if (cb_txn->processing == XST_PENDING ||
			cb_txn->processing == XST_PROCESSING) {
			if (!cb_txn->context_id) {

				/* For PROCESSING transaction,
				 * 'process_accept_req' already dequeued.
				 */

				if (cb_txn->processing == XST_PENDING) {
					dequeue_txn_locked(cb_txn);

					txn_free(cb_txn);
				}
			} else
				complete(&cb_txn->completion);
		}
	}

	mutex_unlock(&si_mutex);

	/* We can not notify QTEE of the server demise; we keep objects around to
	 * notify QTEE the next time it invokes an object with a dead server.
	 */

	pr_info("%s server terminated (refs: %u).\n", si->comm, kref_read(&si->refcount));

	kref_put(&si->refcount, ____destroy_server_info);

	return 0;
}

static const struct file_operations server_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = server_ioctl,
	.compat_ioctl = server_ioctl,
	.release = server_release,
};

/* QTEE object invocation. */

static long process_invoke_req(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int i, ret;

	struct si_object *object = filp->private_data;

	struct smcinvoke_cmd_req u_req;	/* User request. */
	union smcinvoke_arg *u_args;	/* Array of arguments as passed from userspace. */
	int u_args_nr;					/* Total number of arguments in 'u_args'. */

	struct si_arg *u = NULL;		/* Array of arguments passed to the QTEE. */

	struct si_object_invoke_ctx *oic;

	/* Do some sanity check! */

	if (_IOC_SIZE(cmd) != sizeof(u_req))
		return -EINVAL;

	if (copy_from_user(&u_req, (void __user *)arg, sizeof(u_req)))
		return -EFAULT;

	if (typeof_si_object(object) == SI_OT_ROOT) {
		if ((u_req.op == IClientEnv_OP_notifyDomainChange) ||
			(u_req.op == IClientEnv_OP_adciAccept) ||
			(u_req.op == IClientEnv_OP_adciShutdown)) {
			pr_err("invalid rootenv op\n");

			return -EINVAL;
		}

		if (u_req.op == IClientEnv_OP_registerWithCredentials) {
			if (u_req.counts != OBJECT_COUNTS_PACK(0, 0, 1, 1)) {
				pr_err("IClientEnv_OP_registerWithCredentials: incorrect number of arguments.\n");

				return -EINVAL;
			}
		}
	}

	if (u_req.argsize != sizeof(union smcinvoke_arg))
		return -EINVAL;

	/* Allocate and initialize resources. */

	oic = kzalloc(sizeof(*oic), GFP_KERNEL);
	if (!oic)
		return -ENOMEM;

	u_args_nr = OBJECT_COUNTS_TOTAL(u_req.counts);
	u_args = kcalloc(u_args_nr, u_req.argsize, GFP_KERNEL);
	if (!u_args) {
		ret = -ENOMEM;

		goto out_failed;
	}

	/* Plus one for 'SI_AT_END'. */
	u = kcalloc(u_args_nr + 1, sizeof(struct si_arg), GFP_KERNEL);
	if (!u) {
		ret = -ENOMEM;

		goto out_failed;
	}

	/* Copy argument array from userspace. */
	if (copy_from_user(u_args, (void __user *)u_req.args, u_args_nr * u_req.argsize)) {
		ret = -EFAULT;

		goto out_failed;
	}

	if (typeof_si_object(object) == SI_OT_ROOT) {
		if (u_req.op == IClientEnv_OP_registerWithCredentials) {
			if (U_HANDLE_IS_NULL(u_args[0].o.fd)) {
				pr_err("IClientEnv_OP_registerWithCredentials: privileged credential.\n");

				ret = -EINVAL;
				goto out_failed;
			}
		}
	}

	pr_debug("%s object invocation with %d arguments (%04x) and op %d.\n",
		si_object_name(object), u_args_nr, u_req.counts, u_req.op);

	/* + INITIATE an invocation. */

	if (marshal_in_req(u, u_args, u_req.counts)) {
		pr_err("marshal_in_req failed.\n");
		ret = -EINVAL;

		goto out_failed;
	}

	/* TODO. Move this initialization to SI-CORE. */
	u_req.result = OBJECT_ERROR_INVALID;

	ret = si_object_do_invoke(oic, object, u_req.op, u, &u_req.result);
	if (ret) {
		pr_err("si_object_do_invoke failed %d, %d.\n", ret, u_req.result);

		if (u_req.result) {
			if (ret == -EINVAL || ret == -ENOMEM ||	ret == -ENOSPC) {

				/* SI-CORE did not even starts the invocation. */

				marshal_in_req_cleanup(u, 0);

				for (i = 0; u[i].type; i++) {
					if (u[i].type == SI_AT_IO &&
						typeof_si_object(u[i].o) != SI_OT_USER)
						put_si_object(u[i].o);
				}

				goto out_failed;
			}
		}

		/* SI-CORE made an unsuccessful invocation. */
		/* ret == -EINVAL || ret == -ENOMEM && !u_req.result: Marshal out failed.
		 * ret == -EAGAIN || ret == -ENODEV: QTEE communication failed.
		 */

		marshal_in_req_cleanup(u, 0);

		goto out_failed;
	}

	if (!u_req.result) {
		if (marshal_out_req(u_args, u)) {
			pr_err("marshal_out_req failed.\n");
			ret = -EINVAL;

			goto out_failed;
		}

		if (copy_to_user((void __user *)u_req.args, u_args, u_args_nr * u_req.argsize)) {
			ret = -EFAULT;

			goto out_failed;
		}
	} else {

		/* SI-CORE made a successful invocation but QTEE failed.
		 * We still need to put temporary references we hold from 'marshal_in_req';
		 * QTEE will release it's own. 'notify == 1' as we return success to user.
		 */

		marshal_in_req_cleanup(u, 1);
	}

	/* Copy u_req.result back! */
	if (copy_to_user((void __user *)arg, &u_req, sizeof(u_req))) {
		ret = -EFAULT;

		goto out_failed;
	}

	pr_debug("%s object invocation returned with %d.\n",
		si_object_name(object), u_req.result);

	ret = 0;

out_failed:

	kfree(u);
	kfree(u_args);
	kfree(oic);

	if (ret)
		pr_err("failed with %d\n", ret);

	return ret;
}

static long qtee_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret;

	switch (cmd) {
	case SMCINVOKE_IOCTL_INVOKE_REQ:
		ret = process_invoke_req(filp, cmd, arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

static int qtee_release(struct inode *nodp, struct file *filp)
{
	struct si_object *object = filp->private_data;

	/* The matching 'get_si_object' is in 'get_u_handle_from_si_object'. */

	pr_debug("%s released.\n", si_object_name(object));

	put_si_object(object);

	return 0;
}

static const struct file_operations qtee_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = qtee_ioctl,
	.compat_ioctl = qtee_ioctl,
	.release = qtee_release,
};

/* ''ROOT Object'' */

static void ____destroy_server_info(struct kref *kref)
{
	struct server_info *si = container_of(kref, struct server_info, refcount);

	pr_info("%s server destroyed.\n", si->comm);

	kfree(si);
}

static long process_server_req(struct file *filp, unsigned int cmd,	unsigned long arg)
{
	s64 u_handle;
	struct server_info *si;

	/* Use may pass an instance of 'smcinvoke_server' to change the output buffer
	 * size. We do not care for that value as si_core always use a constant.
	 */

	si = kzalloc(sizeof(*si), GFP_KERNEL);
	if (!si)
		return -ENOMEM;

	kref_init(&si->refcount);
	INIT_LIST_HEAD(&si->cb_tx_list);
	init_waitqueue_head(&si->Q);
	get_task_comm(si->comm, current);

	/* Assign a u_handle for the server. */
	if (!u_handle_alloc(si->comm, &server_fops, si, &u_handle))
		return u_handle;

	kfree(si);

	return -EBADF;
}

static long root_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret;

	switch (cmd) {
	case SMCINVOKE_IOCTL_INVOKE_REQ:
		ret = process_invoke_req(filp, cmd, arg);
		break;
	case SMCINVOKE_IOCTL_SERVER_REQ:
		ret = process_server_req(filp, cmd, arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

static int root_open(struct inode *nodp, struct file *filp)
{
	/* Always return the same instance of root si_object. */

	filp->private_data = ROOT_SI_OBJECT;

	return 0;
}

static const struct file_operations root_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = root_ioctl,
	.compat_ioctl = root_ioctl,
	.open = root_open,
};

static dev_t smcinvoke_device_no;
static struct class *driver_class;
struct device *smci_dev;
static struct cdev smcinvoke_cdev;

static int smcinvoke_probe(struct platform_device *pdev)
{
	int ret;

	ret = alloc_chrdev_region(&smcinvoke_device_no, 0, 1, SMCI_DEVICE_NAME);
	if (ret < 0) {
		pr_err("alloc_chrdev_region failed %d.\n", ret);

		return ret;
	}

#if  (KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE)
	driver_class = class_create(SMCI_DEVICE_NAME);
#else
	driver_class = class_create(THIS_MODULE, SMCI_DEVICE_NAME);
#endif
	if (IS_ERR(driver_class)) {
		ret = PTR_ERR(driver_class);
		pr_err("class_create failed %d.\n", ret);

		goto exit_unreg_chrdev_region;
	}

	cdev_init(&smcinvoke_cdev, &root_fops);
	smcinvoke_cdev.owner = THIS_MODULE;

	ret = cdev_add(&smcinvoke_cdev, MKDEV(MAJOR(smcinvoke_device_no), 0), 1);
	if (ret < 0) {
		pr_err("cdev_add failed %d.\n", ret);

		goto exit_destroy_class;
	}

	/* Create '/dev/smcinvoke'. */
	smci_dev = device_create(driver_class, NULL, smcinvoke_device_no, NULL, SMCI_DEVICE_NAME);
	if (IS_ERR(smci_dev)) {
		ret = PTR_ERR(smci_dev);
		pr_err("device_create failed %d.\n", ret);

		goto exit_destroy_cdev;
	}

#if IS_ENABLED(CONFIG_QSEECOM_COMPAT) && IS_ENABLED(CONFIG_QSEECOM_PROXY)
	ret = get_qseecom_kernel_fun_ops();
	if (ret)
		pr_err("failed to get qseecom kernel func ops %d\n", ret);
#endif

	return 0;

exit_destroy_cdev:
	cdev_del(&smcinvoke_cdev);
exit_destroy_class:
	class_destroy(driver_class);
exit_unreg_chrdev_region:
	unregister_chrdev_region(smcinvoke_device_no, 1);

	return ret;
}

static const struct of_device_id smcinvoke_match[] = {
	{ .compatible = "qcom,smcinvoke", }, {},
};

static struct platform_driver smcinvoke_plat_driver = {
	.probe = smcinvoke_probe,
	.driver = {
		.name = "smcinvoke",
		.of_match_table = smcinvoke_match,
	},
};

module_platform_driver(smcinvoke_plat_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("smcinvoke driver");
MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);
MODULE_IMPORT_NS(DMA_BUF);
