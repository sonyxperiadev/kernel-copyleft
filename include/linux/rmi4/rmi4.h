/*
 * RMI4 bus driver.
 * include/linux/rmi4/rmi4.h
 *
 * Copyright (C) 2011 Sony Ericsson mobile communications AB
 * Copyright (C) 2012 Sony Mobile Communications AB
 *
 * Author: Joachim Holst <joachim.holst@sonyericsson.com>
 * Author: Joachim Holst <joachim.holst@sonymobile.com>
 *
 * Based on rmi_bus by Synaptics and Unixphere.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __RMI4_H__
#define __RMI4_H__

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/fs.h>

#define RMI4_CORE_DRIVER_NAME	"rmi4-core-driver"
#define RMI4_CHAR_DEV_NAME	"rmi4-char-dev"

#define RMI4_NAME_MAXLEN 32

#define PDT_START_SCAN_LOCATION 0x00E9

enum rmi4_data_command {
	QUERY,
	COMMAND,
	CONTROL,
	DATA,
	NUM_COMMANDS,
};

/* All notification events should be bitmasks */
enum rmi4_notification_event {
	RMI4_DRIVER_RESET		= (1 << 0),
	RMI4_CHIP_SUSPEND		= (1 << 1),
	RMI4_CHIP_WAKEUP		= (1 << 2),
	RMI4_CHIP_BOOTLOADER		= (1 << 3),
};

struct cdev;
struct rmi4_device;
struct rmi4_function;
struct rmi4_core_device;
struct rmi4_function_device;

struct rmi4_comm_ops {
	int (*chip_read)(struct rmi4_core_device *cdev, u16 addr, u8 *buf,
			 int len);
	int (*chip_write)(struct rmi4_core_device *cdev, u16 addr, u8 *buf,
			  int len);
};

struct rmi4_core_device {
	char name[RMI4_NAME_MAXLEN];
	struct device dev;

	int (*read)(struct rmi4_core_device *cdev, u16 addr, u8 *buf,
		    int len);
	int (*write)(struct rmi4_core_device *cdev, u16 addr, u8 *buf,
		     int len);

};
#define to_rmi4_core_device(d) container_of(d, struct rmi4_core_device, dev)

/* Core Driver should take care of PDT reading and low-level IRQ
 * handling (requesting the IRQ). Function drivers can subscribe
 * to low-level interrupts, but need to handle them by themselves.
 * Core driver will create function core devices based on PDT scanning
 * and platform data */
struct rmi4_core_driver {
	struct device_driver drv;

	int (*probe)(struct rmi4_core_device *cdev);
	int (*remove)(struct rmi4_core_device *cdev);

	/* This function will fore a re-read of the PDT. It's mostly F34
	 * that will have eny use of this function */
	int (*read_pdt)(struct rmi4_function_device *fdev);

	/* These functions abstract low level communication with the chip.
	 * No function should know anything about addresses. Only the address
	 * ofsets withing it own PDTshould be known. */
	int (*read)(struct rmi4_function_device *fdev,
		    enum rmi4_data_command cmd, int addr_offset,
		    u8 *data, int data_len);

	int (*write)(struct rmi4_function_device *fdev,
		     enum rmi4_data_command cmd, int addr_offset,
		     u8 *data, int data_len);

	int (*request_irq)(struct rmi4_function_device *fdev, void *data,
			   void(*fn)(int, void *), u8 mask);

	int (*free_irq)(struct rmi4_function_device *fdev, void *data);

	/* This function will disable all non essential interrupts. Main use
	 * is for funtion 54 which needs exclusive access to IRQ's in order
	 * to read out correct data from the chip. */
	int (*disable_non_essential_irqs)(struct rmi4_function_device *fdev,
					  bool disable);
	int (*request_notification)(
		struct rmi4_function_device *fdev,
		enum rmi4_notification_event events,
		void (*callback)(enum rmi4_notification_event event,
				 void *data),
		void *data);

	void (*release_notification)(struct rmi4_function_device *fdev,
				     void *data);

	void (*notify)(struct rmi4_function_device *fdev,
		       enum rmi4_notification_event event);
};
#define to_rmi4_core_driver(d) container_of(d, struct rmi4_core_driver, drv)

struct rmi4_function_device {
	char name[RMI4_NAME_MAXLEN];
	unsigned func_id;
	struct device dev;
};
#define to_rmi4_func_core(d) container_of(d, struct rmi4_function_device, dev)

struct rmi4_function_driver {
	struct device_driver drv;

	int (*probe)(struct rmi4_function_device *fdev);
	int (*remove)(struct rmi4_function_device *fdev);
};
#define to_rmi4_func_driver(d) container_of(d, struct rmi4_function_driver, drv)

/* This data will be passed to the char_dev driver. This driver needs
 * direct access to register read/write, so we need to supply data to those
 * functions. It should normally not be enabled since it's mainly used
 * for debug/parameter trimming  purposes.
 * This data will be created and registered by the core driver */
struct rmi4_char_dev_func_data {
	char sensor_name[RMI4_NAME_MAXLEN];
	int gpio;
	int (*read)(struct rmi4_function_device *fdev, u16 addr, u8 *buf,
		    int len);
	int (*write)(struct rmi4_function_device *fdev, u16 addr, u8 *buf,
		     int len);
};

/* Data that will be passed to the function drivers */
struct rmi4_function_data {
	const char *func_name;
	int func_id;
	void *func_data;
};

/* Data required by the core of the RMI4 bus */
struct rmi4_core_device_data {
	const char *core_name;
	int attn_gpio;
	int irq_polarity;
	bool irq_is_shared;
	unsigned long num_functions;
	struct rmi4_function_data *func_data;
};

/* Function core related functions */
int rmi4_bus_register_function_core(struct device *parent,
				    struct rmi4_function_data *fdata);

int rmi4_bus_unregister_function_core(struct device *parent,
				      struct rmi4_function_data *fdata);

/* Adapter related functions */
int rmi4_bus_register_adapter(struct device *parent, struct rmi4_comm_ops *ops,
			      struct rmi4_core_device_data *ddata);

int rmi4_bus_unregister_adapter(struct device *parent);

/* Core driver related functions */
int rmi4_bus_register_core_driver(struct rmi4_core_driver *cdrv);
void rmi4_bus_unregister_core_driver(struct rmi4_core_driver *cdrv);

/* Function driver related functions */
int rmi4_bus_register_function_driver(struct rmi4_function_driver *fdrv);
void rmi4_bus_unregister_function_driver(struct rmi4_function_driver *fdrv);


int rmi4_bus_read(struct rmi4_function_device *fdev,
		  enum rmi4_data_command func, int addr_offset,
		  u8 *data, int data_len);

int rmi4_bus_write(struct rmi4_function_device *fdev,
		   enum rmi4_data_command func, int addr_offset,
		   u8 *data, int data_len);

int rmi4_bus_update_pdt(struct rmi4_function_device *fdev);

int rmi4_bus_request_irq(struct rmi4_function_device *fdev, void *data,
			 void (*fn)(int, void *), u8 mask);

int rmi4_bus_free_irq(struct rmi4_function_device *fdev, void *data);

int rmi4_bus_set_non_essential_irq_status(struct rmi4_function_device *fdev,
					  bool disabled);

/* Register for notification events.
 * Since all events are bitmasks, it is possible to register for several
 * different events in one go. It is also possible to call this several
 * times in order to register for different events. When the callback is
 * called, several events can be combined in the event input and all need
 * to be handled */
int rmi4_bus_request_notification(
	struct rmi4_function_device *fdev, enum rmi4_notification_event events,
	void (*callback)(enum rmi4_notification_event event, void *data),
	void *data);

/* Releases a notifocation request. This will unsubscribe all notifications
 * associated with the supplied data pinter */
void rmi4_bus_release_notification(struct rmi4_function_device *fdev,
				  void *data);

/* Notify all subscribers about function initiated event.
 * Any function driver can send notifications without being
 * registered for listen events. */
void rmi4_bus_notify(struct rmi4_function_device *fdev,
			     enum rmi4_notification_event event);

struct device * __must_check rmi4_bus_create_char_dev(
	const char *name, const struct file_operations *fops, struct cdev *cdev,
	struct device *parent, void *drvdata);


void rmi4_bus_destroy_char_dev(struct device *parent, struct cdev *cdev);

#endif /* __RMI4_H__ */
