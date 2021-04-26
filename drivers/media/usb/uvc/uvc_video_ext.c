static int uvc_init_video_isoc_ext(struct uvc_streaming *stream,
	struct usb_host_endpoint *ep, gfp_t gfp_flags)
{
	struct urb_ext *urb_ext;
	struct urb *urb;
	unsigned int npackets, i, j;
	u16 psize;
	u32 size;

	stream->uvc_wq = alloc_workqueue("uvc_wq",
					__WQ_ORDERED | __WQ_ORDERED_EXPLICIT |
					WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);
	if (!stream->uvc_wq) {
		uvc_printk(KERN_ERR, "Failed to alloc_workqueue.\n");
		return -ENOMEM;
	}

	stream->urbs_num = uvc_urbs_isoc_param;

	psize = uvc_endpoint_max_bpi(stream->dev->udev, ep);
	size = stream->ctrl.dwMaxVideoFrameSize;

	npackets = uvc_alloc_urb_buffers(stream, size, psize, gfp_flags);
	if (npackets == 0)
		return -ENOMEM;

	/* don't use stream->urb */
	stream->urb = NULL;

	stream->urb_ext = kmalloc(stream->urbs_num * sizeof(struct urb_ext *),
				GFP_KERNEL);
	if (stream->urb_ext == NULL) {
		uvc_free_urb_buffers(stream);
		return -ENOMEM;
	}

	size = npackets * psize;

	for (i = 0; i < stream->urbs_num; ++i) {
		urb_ext = kmalloc(sizeof(struct urb_ext), GFP_KERNEL);
		if (urb_ext == NULL) {
			uvc_uninit_video(stream, 1);
			return -ENOMEM;
		}
		urb = usb_alloc_urb(npackets, gfp_flags);
		if (urb == NULL) {
			uvc_uninit_video(stream, 1);
			return -ENOMEM;
		}

		stream->urb_ext[i] = urb_ext;
		urb->dev = stream->dev->udev;
		urb->context = stream->urb_ext[i];
		urb->pipe = usb_rcvisocpipe(stream->dev->udev,
				ep->desc.bEndpointAddress);
#ifndef CONFIG_DMA_NONCOHERENT
		urb->transfer_flags = URB_ISO_ASAP | URB_NO_TRANSFER_DMA_MAP;
		urb->transfer_dma = stream->urb_dma[i];
#else
		urb->transfer_flags = URB_ISO_ASAP;
#endif
		urb->interval = ep->desc.bInterval;
		urb->transfer_buffer = stream->urb_buffer[i];
		urb->complete = uvc_video_complete_ext;
		urb->number_of_packets = npackets;
		urb->transfer_buffer_length = size;

		for (j = 0; j < npackets; ++j) {
			urb->iso_frame_desc[j].offset = j * psize;
			urb->iso_frame_desc[j].length = psize;
		}

		INIT_WORK(&stream->urb_ext[i]->complete_work,
				uvc_video_complete_ext_work);
		stream->urb_ext[i]->context = stream;
		stream->urb_ext[i]->urb_id = i;
		stream->urb_ext[i]->urb = urb;
	}

	/* set active urb extension isoc=1 */
	atomic_set(&stream->urb_ext_active, 1);

	return 0;
}

static int uvc_init_video_bulk_ext(struct uvc_streaming *stream,
	struct usb_host_endpoint *ep, gfp_t gfp_flags)
{
	struct urb_ext *urb_ext;
	struct urb *urb;
	unsigned int npackets, pipe, i;
	u16 psize;
	u32 size;

	stream->uvc_wq = alloc_workqueue("uvc_wq",
					__WQ_ORDERED | __WQ_ORDERED_EXPLICIT |
					WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);
	if (!stream->uvc_wq) {
		uvc_printk(KERN_ERR, "Failed to alloc_workqueue.\n");
		return -ENOMEM;
	}

	psize = usb_endpoint_maxp(&ep->desc);
	size = stream->ctrl.dwMaxPayloadTransferSize;
	stream->bulk.max_payload_size = size;

	npackets = uvc_alloc_urb_buffers(stream, size, psize, gfp_flags);
	if (npackets == 0)
		return -ENOMEM;

	/* don't use stream->urb */
	stream->urb = NULL;

	stream->urb_ext = kmalloc(stream->urbs_num * sizeof(struct urb_ext *),
				GFP_KERNEL);
	if (stream->urb_ext == NULL) {
		uvc_free_urb_buffers(stream);
		return -ENOMEM;
	}

	size = npackets * psize;

	if (usb_endpoint_dir_in(&ep->desc))
		pipe = usb_rcvbulkpipe(stream->dev->udev,
				       ep->desc.bEndpointAddress);
	else
		pipe = usb_sndbulkpipe(stream->dev->udev,
				       ep->desc.bEndpointAddress);

	if (stream->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		size = 0;

	for (i = 0; i < stream->urbs_num; ++i) {
		urb_ext = kmalloc(sizeof(struct urb_ext), GFP_KERNEL);
		if (urb_ext == NULL) {
			uvc_uninit_video(stream, 1);
			return -ENOMEM;
		}
		urb = usb_alloc_urb(0, gfp_flags);
		if (urb == NULL) {
			uvc_uninit_video(stream, 1);
			return -ENOMEM;
		}

		stream->urb_ext[i] = urb_ext;
		usb_fill_bulk_urb(urb, stream->dev->udev, pipe,
			stream->urb_buffer[i], size, uvc_video_complete_ext,
			stream->urb_ext[i]);
#ifndef CONFIG_DMA_NONCOHERENT
		urb->transfer_flags = URB_NO_TRANSFER_DMA_MAP;
		urb->transfer_dma = stream->urb_dma[i];
#endif

		INIT_WORK(&stream->urb_ext[i]->complete_work,
				uvc_video_complete_ext_work);
		stream->urb_ext[i]->context = stream;
		stream->urb_ext[i]->urb_id = i;
		stream->urb_ext[i]->urb = urb;
	}

	/* set active urb extension bulk=2 */
	atomic_set(&stream->urb_ext_active, 2);

	return 0;
}

static void uvc_uninit_video_ext(struct uvc_streaming *stream)
{
	struct urb *urb;
	unsigned int i;

	atomic_set(&stream->urb_ext_active, 0);

	if (stream->uvc_wq)
		flush_workqueue(stream->uvc_wq);

	if (!stream->urb_ext)
		goto destroy_wq;

	for (i = 0; i < stream->urbs_num; ++i) {
		urb = stream->urb_ext[i]->urb;
		if (urb == NULL)
			continue;

		cancel_work_sync(&stream->urb_ext[i]->complete_work);
		usb_kill_urb(urb);
		usb_free_urb(urb);
		stream->urb_ext[i]->urb = NULL;
		kfree(stream->urb_ext[i]);
		stream->urb_ext[i] = NULL;
	}
	kfree(stream->urb_ext);
	stream->urb_ext = NULL;

destroy_wq:
	if (stream->uvc_wq) {
		destroy_workqueue(stream->uvc_wq);
		stream->uvc_wq = NULL;
	}
}

static void uvc_video_complete_ext(struct urb *urb)
{
	unsigned long flags;
	struct urb_ext *urb_ext = urb->context;
	struct uvc_streaming *stream = urb_ext->context;
	struct uvc_video_queue *queue = &stream->queue;

	spin_lock_irqsave(&queue->irqlock, flags);
	if (!atomic_read(&stream->urb_ext_active)) {
		spin_unlock_irqrestore(&queue->irqlock, flags);
		return;
	}
	spin_unlock_irqrestore(&queue->irqlock, flags);
	urb_ext->urb_ts = uvc_video_get_time();
	queue_work_on(stream->specify_cpu,
		stream->uvc_wq, &urb_ext->complete_work);
}

static void uvc_video_complete_ext_work(struct work_struct *work)
{
	struct urb_ext *urb_ext = container_of(work, struct urb_ext,
						complete_work);
	struct urb *urb = urb_ext->urb;
	struct uvc_streaming *stream = urb_ext->context;
	struct uvc_video_queue *queue = &stream->queue;
	struct uvc_video_queue *qmeta = &stream->meta.queue;
	struct vb2_queue *vb2_qmeta = stream->meta.vdev.queue;
	struct uvc_buffer *buf = NULL;
	struct uvc_buffer *buf_meta = NULL;
	unsigned long flags;
	int ret;

	switch (urb->status) {
	case 0:
		break;

	default:
		uvc_printk(KERN_WARNING, "Non-zero status (%d) in video "
			"completion handler.\n", urb->status);
		/* fall through */
	case -ENOENT:		/* usb_kill_urb() called. */
		if (stream->frozen)
			return;
		/* fall through */
	case -ECONNRESET:	/* usb_unlink_urb() called. */
	case -ESHUTDOWN:	/* The endpoint is being disabled. */
		uvc_queue_cancel(queue, urb->status == -ESHUTDOWN);
		if (vb2_qmeta)
			uvc_queue_cancel(qmeta, urb->status == -ESHUTDOWN);
		return;
	}

	stream->stats.stream.urb_act_len = urb->actual_length;

	spin_lock_irqsave(&queue->irqlock, flags);
	if (!list_empty(&queue->irqqueue))
		buf = list_first_entry(&queue->irqqueue, struct uvc_buffer,
				       queue);
	spin_unlock_irqrestore(&queue->irqlock, flags);

	if (vb2_qmeta) {
		spin_lock_irqsave(&qmeta->irqlock, flags);
		if (!list_empty(&qmeta->irqqueue))
			buf_meta = list_first_entry(&qmeta->irqqueue,
						    struct uvc_buffer, queue);
		spin_unlock_irqrestore(&qmeta->irqlock, flags);
	}

	stream->decode(urb, stream, buf, buf_meta);

	ret = usb_submit_urb(urb, GFP_ATOMIC);
	if (ret < 0) {
		uvc_printk(KERN_ERR, "Failed to resubmit video URB (%d).\n",
			ret);
	}
	uvc_trace(UVC_TRACE_VIDEO, "%s: done(cpu:%u,pid:%d).\n",
		__func__, current->cpu, current->pid);
}

