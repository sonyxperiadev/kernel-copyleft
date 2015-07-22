/*
 * f_gg.c -- USB gadget gordons gate driver
 * Based on f_serial.c
 *
 *
 * Copyright (C) 2003 Al Borchers (alborchers@steinerpoint.com)
 * Copyright (C) 2008 by David Brownell
 * Copyright (C) 2008 by Nokia Corporation
 * Copyright (C) 2011 by Sony Ericsson Mobile Communications AB
 * Copyright (C) 2013 by Sony Mobile Communications AB
 *
 * This software is distributed under the terms of the GNU General
 * Public License ("GPL") as published by the Free Software Foundation,
 * either version 2 of that License or (at your option) any later version.
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <mach/usb_gadget_xport.h>

#include "u_serial.h"
#include "gadget_chips.h"

#define GG_GSERIAL_NO_PORTS 1
/*
 * This function packages a simple "generic serial" port with no real
 * control mechanisms, just raw data transfer over two bulk endpoints.
 */

struct ggor_descs {
	struct usb_endpoint_descriptor *in;
	struct usb_endpoint_descriptor *out;
};

struct f_ggor {
	struct gserial port;
	u8 data_id;
	u8 port_num;
	struct ggor_descs fs;
	struct ggor_descs hs;
	u8 online;
	enum transport_type transport;
};

static unsigned int no_tty_ports;
static unsigned int nr_ports;

static struct gordon_port_info {
	enum transport_type transport;
	unsigned port_num;
	unsigned client_port_num;
} ggordon_ports[GG_GSERIAL_NO_PORTS];

static inline struct f_ggor *func_to_ggor(struct usb_function *f)
{
	return container_of(f, struct f_ggor, port.func);
}

/*-------------------------------------------------------------------------*/

/* interface descriptor: */

static struct usb_interface_descriptor ggor_interface_desc = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	/* .bInterfaceNumber = DYNAMIC */
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	/* .iInterface = DYNAMIC */
};
/* full speed support: */

static struct usb_endpoint_descriptor ggor_fs_in_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor ggor_fs_out_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_OUT,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *ggor_fs_function[] = {
	(struct usb_descriptor_header *) &ggor_interface_desc,
	(struct usb_descriptor_header *) &ggor_fs_in_desc,
	(struct usb_descriptor_header *) &ggor_fs_out_desc,
	NULL,
};

/* high speed support: */
static struct usb_endpoint_descriptor ggor_hs_in_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor ggor_hs_out_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = __constant_cpu_to_le16(512),
};

static struct usb_descriptor_header *ggor_hs_function[] = {
	(struct usb_descriptor_header *) &ggor_interface_desc,
	(struct usb_descriptor_header *) &ggor_hs_in_desc,
	(struct usb_descriptor_header *) &ggor_hs_out_desc,
	NULL,
};

/* string descriptors: */

static struct usb_string ggor_string_defs[] = {
	[0].s = "Gordons Gate",
	{  } /* end of list */
};

static struct usb_gadget_strings ggor_string_table = {
	.language = 0x0409, /* en-us */
	.strings = ggor_string_defs,
};

static struct usb_gadget_strings *ggor_strings[] = {
	&ggor_string_table,
	NULL,
};

static int ggate_setup(struct usb_configuration *c)
{
	int ret = 0;

	if (no_tty_ports)
		ret = gserial_setup(c->cdev->gadget, no_tty_ports);

	return ret;
}

static int ggate_connect(struct f_ggor *gser)
{
	unsigned port_num;

	pr_debug("%s: transport: %s f_ggor: %p ggor: %p port_num: %d\n",
			__func__, xport_to_str(gser->transport),
			gser, &gser->port, gser->port_num);
	port_num = ggordon_ports[gser->port_num].client_port_num;

	switch (gser->transport) {
	case USB_GADGET_XPORT_TTY:
		gserial_connect(&gser->port, port_num);
		break;

	default:
		pr_err("%s: Un-supported transport: %s\n", __func__,
				xport_to_str(gser->transport));
		return -ENODEV;
	}

	return 0;
}

static int ggate_disconnect(struct f_ggor *gser)
{
	unsigned port_num;

	pr_debug("%s: transport: %s f_ggor: %p ggor: %p port_num: %d\n",
			__func__, xport_to_str(gser->transport),
			gser, &gser->port, gser->port_num);

	port_num = ggordon_ports[gser->port_num].client_port_num;

	switch (gser->transport) {
	case USB_GADGET_XPORT_TTY:
		gserial_disconnect(&gser->port);
		break;

	default:
		pr_err("%s: Un-supported transport:%s\n", __func__,
				xport_to_str(gser->transport));
		return -ENODEV;
	}

	return 0;
}

static int ggor_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_ggor *gser = func_to_ggor(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int rc = 0;

	/* we know alt == 0, so this is an activation or a reset */

	if (gser->port.in->driver_data) {
		pr_err("%s(): reset generic data ttyGS%d\n",
				__func__, gser->port_num);
		ggate_disconnect(gser);
	} else {
		pr_info("%s(): activate generic data ttyGS%d\n",
				__func__, gser->port_num);
	}

	if (!gser->port.in->desc || !gser->port.out->desc) {
		pr_debug("%s(): activate generic ttyGS%d\n",
				__func__, gser->port_num);
		if (config_ep_by_speed(cdev->gadget, f, gser->port.in) ||
			config_ep_by_speed(cdev->gadget, f, gser->port.out)) {
			gser->port.in->desc = NULL;
			gser->port.out->desc = NULL;
			return -EINVAL;
		}
	}

	ggate_connect(gser);

	gser->online = 1;
	return rc;
}

static void ggor_disable(struct usb_function *f)
{
	struct f_ggor *gser = func_to_ggor(f);
	ggate_disconnect(gser);

	gser->online = 0;
}
/*-------------------------------------------------------------------------*/

/* serial function driver setup/binding */

static int
ggor_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_ggor *gser = func_to_ggor(f);
	struct usb_ep *ep;
	int status;

	/* allocate instance-specific interface IDs */
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	gser->data_id = status;
	ggor_interface_desc.bInterfaceNumber = status;

	status = -ENODEV;

	/* allocate instance-specific endpoints */
	ep = usb_ep_autoconfig(cdev->gadget, &ggor_fs_in_desc);
	if (!ep)
		goto fail;
	gser->port.in = ep;
	ep->driver_data = cdev; /* claim */

	ep = usb_ep_autoconfig(cdev->gadget, &ggor_fs_out_desc);
	if (!ep)
		goto fail;
	gser->port.out = ep;
	ep->driver_data = cdev; /* claim */

	/* copy descriptors, and track endpoint copies */
	f->descriptors = usb_copy_descriptors(ggor_fs_function);

	if (!f->descriptors)
		goto fail;

	/* support all relevant hardware speeds... we expect that when
	 * hardware is dual speed, all bulk-capable endpoints work at
	 * both speeds
	 */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		ggor_hs_in_desc.bEndpointAddress =
				ggor_fs_in_desc.bEndpointAddress;
		ggor_hs_out_desc.bEndpointAddress =
				ggor_fs_out_desc.bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
		f->hs_descriptors = usb_copy_descriptors(ggor_hs_function);

		if (!f->hs_descriptors)
			goto fail;

	}

	pr_info("%s(): generic ttyGS%d: %s speed IN/%s OUT/%s\n",
			__func__,
			gser->port_num,
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			gser->port.in->name, gser->port.out->name);
	return 0;

fail:
	if (f->descriptors)
		usb_free_descriptors(f->descriptors);
	/* we might as well release our claims on endpoints */
	if (gser->port.out)
		gser->port.out->driver_data = NULL;
	if (gser->port.in)
		gser->port.in->driver_data = NULL;

	return status;
}

static void
ggor_unbind(struct usb_configuration *c, struct usb_function *f)
{
	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);
	usb_free_descriptors(f->descriptors);
	kfree(func_to_ggor(f));
}

/**
 * ggor_bind_config - add a generic serial function to a configuration
 * @c: the configuration to support the serial instance
 * @port_num: /dev/ttyGS* port this interface will use
 * Context: single threaded during gadget setup
 *
 * Returns zero on success, else negative errno.
 *
 * Caller must have called @ggordon_setup() with enough ports to
 * handle all the ones it binds.  Caller is also responsible
 * for calling @ggordon_cleanup() before module unload.
 */
int ggor_bind_config(struct usb_configuration *c, u8 port_num)
{
	struct f_ggor *gser;
	int status;

	/* REVISIT might want instance-specific strings to help
	 * distinguish instances ...
	 */

	/* maybe allocate device-global string ID */
	if (ggor_string_defs[0].id == 0) {
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		ggor_string_defs[0].id = status;
	}

	/* allocate and initialize one new instance */
	gser = kzalloc(sizeof *gser, GFP_KERNEL);
	if (!gser)
		return -ENOMEM;

	gser->port_num = port_num;

	gser->port.func.name = "ggor";
	gser->port.func.strings = ggor_strings;
	gser->port.func.bind = ggor_bind;
	gser->port.func.unbind = ggor_unbind;
	gser->port.func.set_alt = ggor_set_alt;
	gser->port.func.disable = ggor_disable;
	gser->transport = ggordon_ports[port_num].transport;

	status = usb_add_function(c, &gser->port.func);
	if (status)
		kfree(gser);
	return status;
}

/**
 * ggordon_init_port - bind a ggordon_port to its transport
 */
static int ggordon_init_port(int port_num, const char *name)
{
	enum transport_type transport;

	if (port_num >= GG_GSERIAL_NO_PORTS)
		return -ENODEV;

	transport = str_to_xport(name);
	pr_debug("%s(): port:%d, transport:%s\n", __func__,
			port_num, xport_to_str(transport));

	ggordon_ports[port_num].transport = transport;
	ggordon_ports[port_num].port_num = port_num;


	switch (transport) {
	case USB_GADGET_XPORT_TTY:
		ggordon_ports[port_num].client_port_num = no_tty_ports;
		no_tty_ports++;
		break;

	default:
		pr_err("%s(): Un-supported transport transport: %u\n",
				__func__, ggordon_ports[port_num].transport);
		return -ENODEV;
	}

	nr_ports++;

	return 0;
}
