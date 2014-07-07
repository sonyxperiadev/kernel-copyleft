/*
 * RMI4 bus core driver.
 * drivers/rmi4/rmi4_driver.c
 *
 * Copyright (C) 2011 Sony Ericsson mobile communications AB
 * Copyright (C) 2012 Sony Mobile communications
 *
 * Author: Joachim Holst <joachim.holst@sonymobile.com>
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

#include <linux/module.h>
#include <linux/rmi4/rmi4.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/atomic.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <asm/atomic.h>

#define RMI4_END_OF_PDT(id) ((id) == 0x00 || (id) == 0xff)

#define PDT_END_SCAN_LOCATION		0x0005
#define RMI4_MAX_PAGE			0xFF
#define RMI4_PAGE_SIZE			0x100
#define RMI4_PDT_PROPERTIES_ADDR	0x00FE

#define F01_FUNCTION_ID		0x01
#define F01_IRQ_ADDR_OFFSET		0x01

#define F01_IRQ_CONTROL_OFFSET		0x01

#define RMI4_HAS_BSR_MASK		0x20

#define RMI4_CORE_RESET_DELAY_MS	50

#define LOCK_PDT(drvdata)						\
	do {								\
		dev_vdbg(&drvdata->cdev->dev, "%s - %d : Locking PDT\n", \
			 __func__, __LINE__);				\
		mutex_lock(&drvdata->pdt_lock);				\
		dev_vdbg(&drvdata->cdev->dev, "%s - PDT Locked\n", __func__); \
	} while (0)

#define UNLOCK_PDT(drvdata)						\
	do {								\
		dev_vdbg(&drvdata->cdev->dev, "%s - %d : Unlocking PDT\n", \
			 __func__, __LINE__);				\
		mutex_unlock(&drvdata->pdt_lock);			\
		dev_vdbg(&drvdata->cdev->dev, "%s - PDT Unlocked\n",	\
			 __func__);				    \
	} while (0)

struct rmi4_pdt_properties {
	union {
		struct {
			u8 unused1:4;
			u8 has_bsr:1;
			u8 non_standard:1;
			u8 unused2:1;
		};
		u8 reg;
	};
};

struct rmi4_pdt_entry {
	u8 query_base_addr:8;
	u8 command_base_addr:8;
	u8 control_base_addr:8;
	u8 data_base_addr:8;
	u8 interrupt_source_count:3;
	u8 bits3and4:2;
	u8 function_version:2;
	u8 bit7:1;
	u8 function_number:8;
};

struct rmi4_notify_client {
	struct list_head entry;
	void *event_data;
	enum rmi4_notification_event event;
	void (*callback)(enum rmi4_notification_event event, void *data);
};
#define to_rmi4_notify_client(l)				\
	container_of(l, struct rmi4_notify_client, entry);

struct rmi4_irq_client {
	struct list_head entry;
	void (*func)(int fn_irq, void *data);
	void *func_data;
	u8 irq_mask;
};
#define to_rmi4_irq_client(l) container_of(l, struct rmi4_irq_client, entry)

struct rmi4_function_data_container {
	struct list_head entry;
	bool is_local;
	struct rmi4_function_data *data;
};
#define to_rmi4_fdata_container(l)					\
	container_of(l, struct rmi4_function_data_container, entry)

struct rmi4_function_container {
	u8 function_id;
	struct mutex data_mutex;
	struct list_head entry;

	/* One or more function data pointers can be stored here. */
	struct list_head function_data;
	bool is_registered;

	/* Contains list of clients requesting IRQ for this function */
	struct mutex irq_client_lock;
	struct list_head irq_client_list;

	u8 num_irqs;
	u32 irq_start_pos;

	atomic_t force_irq_disabled;
};
#define to_rmi4_function_container(l)					\
	container_of(l, struct rmi4_function_container, entry)

struct rmi4_pdt_container {
	u16 bank;
	struct list_head entry;
	struct rmi4_pdt_entry pdt;
};
#define to_rmi4_pdt_container(l)				\
	container_of(l, struct rmi4_pdt_container, entry)

struct rmi4_core_drv_data {
	/* Keeps a list of all PDT entries */
	struct mutex pdt_lock;
	struct list_head pdt_list;

	/* Keeps track of which functions are already registered. Function data
	 * required by these functions is iether supplied via platform data or
	 * created empty here. */
	struct mutex function_lock;
	struct list_head function_list;

	/* Keeps track of IRQ requests */
	struct mutex irq_lock;
	struct list_head interrupt_list;

	struct list_head notify_list;

	/* Keeps track of which functions are not defined via platform
	 * data. This is required since not all functions actually require
	 * platform data, but we should support them anyhow, and also
	 * if a function exists, it should be supported if correct
	 * function driver is loaded. F54 is a good example of this. It should
	 * usually not be included, but if we need it for debugging, then
	 * we should be able to build and load the driver */
	struct mutex reg_func_lock;
	struct list_head local_registered_functions;

	int num_supported_irqs;
	u8 *irq_bits;
	int irq_read_len;

	int irq;

	struct mutex data_lock;
	bool enabled;

	struct mutex pdt_prop_lock;
	struct rmi4_pdt_properties pdt_properties;

	struct rmi4_core_device *cdev;

	struct rmi4_function_data *char_dev_data;

	wait_queue_head_t wq;
	atomic_t suspended;
};

static irqreturn_t rmi4_core_drv_irq_handler(int irq, void *data);
static int _rmi4_core_driver_request_notification(
	struct rmi4_core_device *cdev, enum rmi4_notification_event events,
	void (*callback)(enum rmi4_notification_event event, void *data),
	void *data);
static void _rmi4_core_driver_release_notification(
	struct rmi4_core_device *cdev, void *data);

static u8 rmi4_core_driver_get_irq_mask(struct rmi4_core_device *cdev,
					struct rmi4_function_container *fc)
{
	u8 mask = 0;
	struct list_head *n;
	struct list_head *list;
	struct rmi4_irq_client *c;

	mutex_lock(&fc->irq_client_lock);
	list_for_each_safe(list, n, &fc->irq_client_list) {
		c = to_rmi4_irq_client(list);
		mask |= c->irq_mask;
	}
	mutex_unlock(&fc->irq_client_lock);

	return mask;
}

static struct rmi4_function_container
*rmi4_core_driver_get_function_container(struct rmi4_core_device *cdev,
					 u8 func_id)
{
	struct list_head *n;
	struct list_head *list;
	struct rmi4_function_container *f;
	struct rmi4_core_drv_data *d = dev_get_drvdata(&cdev->dev);

	if (list_empty(&d->function_list)) {
		dev_dbg(&cdev->dev, "%s - No functions found\n",
			__func__);
		return NULL;
	}

	list_for_each_safe(list, n, &d->function_list) {
		f = to_rmi4_function_container(list);
		if (f->function_id == func_id)
			return f;
	}

	return NULL;
}

static struct rmi4_function_container
*rmi4_core_driver_new_function_container(struct rmi4_core_device *cdev)
{
	struct rmi4_function_container *c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c) {
		dev_err(&cdev->dev,
			"%s - Failed to create function container\n",
			__func__);
		return NULL;
	}
	INIT_LIST_HEAD(&c->entry);
	INIT_LIST_HEAD(&c->irq_client_list);
	INIT_LIST_HEAD(&c->function_data);
	mutex_init(&c->irq_client_lock);
	mutex_init(&c->data_mutex);
	return c;
}

static struct rmi4_function_data_container *rmi4_new_fdata_container(
	struct rmi4_core_device *cdev)
{
	struct rmi4_function_data_container *c =
		kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c) {
		dev_err(&cdev->dev,
			"%s - Failed to allocate functon data container\n",
			__func__);
		return ERR_PTR(-ENOMEM);
	}

	INIT_LIST_HEAD(&c->entry);

	return c;
}

/* This function should add all function drivers connected with this function
 * to a list so that we can register them later on. */
static int rmi4_core_driver_add_plat_data(struct rmi4_core_device *cdev,
			  struct rmi4_function_container *c,
			  struct rmi4_function_data_container *initial)
{
	int i;
	struct rmi4_function_data_container *fdc;
	struct rmi4_core_device_data *cd = dev_get_platdata(&cdev->dev);
	struct rmi4_function_data *fd = cd->func_data;

	for (i = 0; i < cd->num_functions; i++, fd++) {
		if (fd->func_id == c->function_id) {
			dev_dbg(&cdev->dev,
				"%s - Platform data found for func 0x%02X\n",
				__func__, fd->func_id);
			dev_dbg(&cdev->dev, "Function name: %s, ID: %d\n",
				 fd->func_name, fd->func_id);
			if (!strncmp(fd->func_name, initial->data->func_name,
				    RMI4_NAME_MAXLEN)) {
				kfree(initial->data);
				initial->data = fd;
				initial->is_local = false;
			} else {
				fdc = rmi4_new_fdata_container(cdev);
				if (!fdc)
					return PTR_ERR(fdc);
				fdc->data = fd;
				list_add_tail(&fdc->entry, &c->function_data);
			}
		}
	}

	return 0;
}

static void rmi4_core_remove_plat_data(struct rmi4_core_device *cdev,
				       struct rmi4_function_container *c)
{
	struct list_head *n;
	struct list_head *list;
	struct rmi4_function_data_container *fdc;

	list_for_each_safe(list, n, &c->function_data) {
		fdc = to_rmi4_fdata_container(list);
		if (fdc->is_local) {
			kfree(fdc->data->func_name);
			kfree(fdc);
		}
		list_del(list);
	}
}

static void rmi4_core_driver_clear_functiondata(struct rmi4_core_device *cdev)
{
	struct list_head *n;
	struct list_head *list;
	struct rmi4_function_container *fc;
	struct rmi4_core_drv_data *d = dev_get_drvdata(&cdev->dev);

	if (list_empty(&d->function_list)) {
		dev_dbg(&cdev->dev, "%s - No functions stored\n",
			__func__);
		return;
	}

	list_for_each_safe(list, n, &d->function_list) {
		fc = to_rmi4_function_container(list);
		rmi4_core_remove_plat_data(cdev, fc);
		list_del(list);
		WARN(!list_empty(&fc->irq_client_list),
		     "All interrupts have not been free'd. Leaking memory. "
		     "Function 0x%02X still has registered IRQ's\n",
		     fc->function_id);

		rmi4_core_remove_plat_data(cdev, fc);
		kfree(fc);
	}
}

static int rmi4_core_driver_populate_functiondata(struct rmi4_core_device *cdev)
{
	int irq = 0;
	char buf[10];
	struct list_head *n;
	struct list_head *list;
	struct rmi4_pdt_container *pdt;
	struct rmi4_function_data *fdata;
	struct rmi4_function_container *fc;
	struct rmi4_function_data_container *fdc;
	struct rmi4_core_drv_data *cdata = dev_get_drvdata(&cdev->dev);

	list_for_each_safe(list, n, &cdata->pdt_list) {
		pdt = to_rmi4_pdt_container(list);
		fc = rmi4_core_driver_get_function_container(
			cdev, pdt->pdt.function_number);
		if (fc) {
			dev_dbg(&cdev->dev,
				"f%02X already available. Fixing IRQ mask\n",
				pdt->pdt.function_number);
			goto create_mask;
		}

		fc = rmi4_core_driver_new_function_container(cdev);
		if (!fc)
			goto fail;

		fc->function_id = pdt->pdt.function_number;

		dev_dbg(&cdev->dev,
			"%s - Creating platform data for func 0x%02X\n",
			__func__, pdt->pdt.function_number);

		fdc = rmi4_new_fdata_container(cdev);
		if (!fdc) {
			dev_err(&cdev->dev,
				"%s - Failed to create function data\n",
				__func__);
			goto fail;
		}
		fdc->is_local = true;

		fdc->data = kzalloc(sizeof(*fdata), GFP_KERNEL);
		if (!fdc->data) {
			dev_err(&cdev->dev,
				"%s - Failed to create initial function data\n",
				__func__);
			goto fail_1;
		}

		snprintf(buf, sizeof(buf), "f%02X",
			 pdt->pdt.function_number);
		fdc->data->func_name = kstrndup(buf,
						strnlen(buf, RMI4_NAME_MAXLEN),
						GFP_KERNEL);
		if (!fdc->data->func_name) {
			dev_err(&cdev->dev,
				"%s - Failed to create function name\n",
				__func__);
			goto fail_2;
		}
		fdc->data->func_id = pdt->pdt.function_number;
		list_add_tail(&fdc->entry, &fc->function_data);

		list_add_tail(&fc->entry, &cdata->function_list);

		if (rmi4_core_driver_add_plat_data(cdev, fc, fdc)) {
			dev_err(&cdev->dev,
				"%s - Failed to add platform data\n",
				__func__);
			goto fail;
		}

create_mask:
		fc->num_irqs = pdt->pdt.interrupt_source_count;
		fc->irq_start_pos = irq;
		irq += pdt->pdt.interrupt_source_count;
		cdata->num_supported_irqs += pdt->pdt.interrupt_source_count;

		dev_dbg(&cdev->dev, "0x%02X supports %d IRQs. Offset = %d\n",
			 fc->function_id, fc->num_irqs, fc->irq_start_pos);
		dev_dbg(&cdev->dev, "Total number of IRQ's = %d\n",
			 cdata->num_supported_irqs);
	}

	return 0;

fail_2:
	kfree(fdc->data);
fail_1:
	kfree(fdc);
fail:
	rmi4_core_driver_clear_functiondata(cdev);
	return -ENOMEM;


}

/* Note: This function must and is always called with PDT (function_list_lock)
 * mutex held. If not, the PDT may be updated while this is being accessed,
 * and that _really_ not good */
static struct rmi4_pdt_container
*rmi4_core_driver_get_pdt_container(struct rmi4_core_device *cdev,
				    int function_id)
{
	struct list_head *n;
	struct list_head *list;
	struct rmi4_pdt_container *entry;
	struct rmi4_core_drv_data *data = dev_get_drvdata(&cdev->dev);

	if (list_empty(&data->pdt_list)) {
		dev_dbg(&cdev->dev, "%s list is empty\n", __func__);
		goto empty;
	}

	list_for_each_safe(list, n, &data->pdt_list) {
		entry = to_rmi4_pdt_container(list);
		dev_dbg(&cdev->dev,
			"%s - Found ID = 0x%02X. Requested = 0x%02X\n",
			__func__, entry->pdt.function_number,
			function_id);
		if (entry->pdt.function_number == function_id)
			return entry;
	}

	dev_dbg(&cdev->dev, "%s - Entry not found\n", __func__);

empty:

	return NULL;
}

static int rmi4_core_driver_set_irq_state(struct rmi4_core_device *cdev,
					  struct rmi4_function_container *fc,
					  u8 mask);

static int rmi4_core_disable_all_irqs(struct rmi4_core_device *cdev)
{
	struct list_head *list;
	struct list_head *n;
	struct rmi4_function_container *fc;
	struct rmi4_core_drv_data *data = dev_get_drvdata(&cdev->dev);

	list_for_each_safe(list, n, &data->function_list) {
		fc = to_rmi4_function_container(list);
		rmi4_core_driver_set_irq_state(cdev, fc, 0);
	}

	dev_dbg(&cdev->dev, "%s - Done\n", __func__);

	return 0;
}

static int rmi4_core_driver_set_irq_state(struct rmi4_core_device *cdev,
					  struct rmi4_function_container *fc,
					  u8 irq_mask)
{
	int i;
	int err;
	struct rmi4_pdt_container *pdt_entry;
	struct rmi4_core_drv_data *ddata = dev_get_drvdata(&cdev->dev);
	u8 data[ddata->irq_read_len];
	unsigned long l_mask = irq_mask;

	pdt_entry = rmi4_core_driver_get_pdt_container(cdev, F01_FUNCTION_ID);
	if (!pdt_entry) {
		dev_err(&cdev->dev,
			"%s - Cant read from non existing function\n",
			__func__);
		err = -ENODEV;
		goto exit;
	}

	dev_dbg(&cdev->dev, "%s - Reading %d bytes from address 0x%X\n",
		__func__, sizeof(data), pdt_entry->pdt.control_base_addr +
		pdt_entry->bank + F01_IRQ_CONTROL_OFFSET);

	err = cdev->read(cdev, pdt_entry->pdt.control_base_addr +
			 pdt_entry->bank + F01_IRQ_CONTROL_OFFSET,
			 data, sizeof(data));

	if (0 > err) {
		dev_err(&cdev->dev, "%s - Failed to read from chip\n",
			__func__);
		goto exit;
	}

	for (i = 0; i < fc->num_irqs; i++) {
		if (test_bit(i, &l_mask)) {
			dev_dbg(&cdev->dev, "%s; Setting bit %d\n",
				 __func__, i + fc->irq_start_pos);
			set_bit(i + fc->irq_start_pos,
				(unsigned long *)&data);
		} else {
			dev_dbg(&cdev->dev, "%s - Clearing bit %d\n",
				 __func__, i + fc->irq_start_pos);
			clear_bit(i + fc->irq_start_pos,
				  (unsigned long *)&data);
		}
	}

	dev_dbg(&cdev->dev, "%s - Writing to address 0x%X\n", __func__,
		pdt_entry->pdt.control_base_addr + pdt_entry->bank +
		F01_IRQ_CONTROL_OFFSET);

	err = cdev->write(cdev, pdt_entry->pdt.control_base_addr +
			  pdt_entry->bank + F01_IRQ_CONTROL_OFFSET,
			  data, sizeof(data));
	if (0 > err)
		dev_err(&cdev->dev, "%s - Failed to write to chip\n",
			__func__);
	else
		dev_dbg(&cdev->dev, "%s - Wrote %d bytes to chip\n",
			 __func__, err);

exit:
	return (0 > err) ? err : 0;
}

static int rmi4_core_driver_register_function_list(
	struct rmi4_core_device *cdev, struct rmi4_function_container *c,
	bool register_function)
{
	int err = 0;
	struct list_head *n;
	struct list_head *list;
	struct rmi4_function_data_container *fdc;

	list_for_each_safe(list, n, &c->function_data) {
		fdc = to_rmi4_fdata_container(list);
		dev_dbg(&cdev->dev,
			"%s - Registerering function %s\n",
			__func__, fdc->data->func_name);
		if (register_function)
			err = rmi4_bus_register_function_core(&cdev->dev,
							      fdc->data);
		else
			rmi4_bus_unregister_function_core(&cdev->dev,
							  fdc->data);
		if (err)
			return err;

		dev_dbg(&cdev->dev,
			"%s registered function %s\n", __func__,
			fdc->data->func_name);
	};

	return 0;
}

static int rmi4_core_driver_cdev_read(struct rmi4_function_device *fdev,
				      u16 addr, u8 *buf, int len)
{
	struct rmi4_core_device *cdev = to_rmi4_core_device(fdev->dev.parent);
	return cdev->read(cdev, addr, buf, len);
}

static int rmi4_core_driver_cdev_write(struct rmi4_function_device *fdev,
				       u16 addr, u8 *buf, int len)
{
	struct rmi4_core_device *cdev = to_rmi4_core_device(fdev->dev.parent);
	return cdev->write(cdev, addr, buf, len);
}

static int rmi4_core_driver_register_char_func(struct rmi4_core_device *cdev)
{
	int err;
	struct rmi4_char_dev_func_data *cfdata;
	struct rmi4_function_data *fdata = kzalloc(sizeof(*fdata), GFP_KERNEL);
	struct rmi4_core_drv_data *ddata = dev_get_drvdata(&cdev->dev);
	struct rmi4_core_device_data *pdata = dev_get_platdata(&cdev->dev);

	if (!fdata) {
		dev_err(&cdev->dev, "%s - Failed to allocate char_dev data\n",
			__func__);
		return -ENOMEM;
	}

	cfdata = kzalloc(sizeof(*cfdata), GFP_KERNEL);
	if (!cfdata) {
		dev_err(&cdev->dev,
			"%s - Failed to alloc char_dev specific data\n",
			__func__);
		err = -ENOMEM;
		goto error_dev_data;
	}

	ddata->char_dev_data = fdata;
	fdata->func_data = cfdata;

	fdata->func_name = kstrndup(RMI4_CHAR_DEV_NAME,
				    sizeof(RMI4_CHAR_DEV_NAME) + 1,
				    GFP_KERNEL);
	strncpy(cfdata->sensor_name, dev_name(&cdev->dev),
		sizeof(cfdata->sensor_name));
	cfdata->sensor_name[RMI4_NAME_MAXLEN - 1] = 0;

	cfdata->read = rmi4_core_driver_cdev_read;
	cfdata->write = rmi4_core_driver_cdev_write;
	cfdata->gpio = pdata->attn_gpio;

	err = rmi4_bus_register_function_core(&cdev->dev, fdata);
	if (err) {
		dev_err(&cdev->dev,
			"%s - Failed to register char dev function\n",
			__func__);
		goto error_reg_func;
	}

	return err;

error_reg_func:
	kfree(fdata->func_name);
	kfree(cfdata);

error_dev_data:
	ddata->char_dev_data = NULL;
	kfree(fdata);
	return err;
}

static void rmi4_core_driver_unregister_char_dev(struct rmi4_core_device *cdev)
{
	struct rmi4_function_data *fdata;
	struct rmi4_core_drv_data *ddata = dev_get_drvdata(&cdev->dev);

	if (NULL == ddata->char_dev_data) {
		dev_info(&cdev->dev, "Char dev not registered\n");
		return;
	}

	fdata = ddata->char_dev_data;
	ddata->char_dev_data = NULL;

	rmi4_bus_unregister_function_core(&cdev->dev, fdata);

	kfree(fdata->func_data);
	kfree(fdata->func_name);
	kfree(fdata);
}

static int rmi4_core_driver_register_functions(struct rmi4_core_device *cdev)
{
	int err = 0;
	struct list_head *n;
	struct list_head *list;
	struct rmi4_function_container *fdata_container;
	struct rmi4_core_drv_data *ddata = dev_get_drvdata(&cdev->dev);

	if (list_empty(&ddata->function_list)) {
		dev_dbg(&cdev->dev, "%s - No functions available\n", __func__);
		return -ENODEV;
	}

	mutex_lock(&ddata->function_lock);
	list_for_each_safe(list, n, &ddata->function_list) {
		fdata_container = to_rmi4_function_container(list);
		if (true == fdata_container->is_registered) {
			dev_dbg(&cdev->dev,
				"%s - Function f%02X already registered\n",
				__func__, fdata_container->function_id);
			continue;
		}
		/* If we fail to register function devices, we shouldn't
		 * abort. Some functions may still be registered and work */
		err = rmi4_core_driver_register_function_list(cdev,
							      fdata_container,
							      true);
		if (err)
			dev_warn(&cdev->dev,
				 "%s - Failed to register all functions\n",
				 __func__);

		fdata_container->is_registered = true;
	}

	mutex_unlock(&ddata->function_lock);
	return err;
}

static int rmi4_core_driver_unregister_functions(struct rmi4_core_device *cdev)
{
	int err = -ENODEV;
	struct list_head *list;
	struct list_head *n;
	struct rmi4_function_container *fdata_container;
	struct rmi4_core_drv_data *ddata = dev_get_drvdata(&cdev->dev);

	mutex_unlock(&ddata->function_lock);
	list_for_each_safe(list, n, &ddata->function_list) {
		fdata_container = to_rmi4_function_container(list);
		dev_dbg(&cdev->dev, "%s - Unregistering function f%02X\n",
			 __func__, fdata_container->function_id);
		err = rmi4_core_driver_register_function_list(cdev,
							      fdata_container,
							      false);
		if (err)
			goto done;

		dev_dbg(&cdev->dev,
			"%s - Unregistered function F%02X\n", __func__,
			fdata_container->function_id);
	}

	rmi4_core_driver_clear_functiondata(cdev);
done:
	mutex_unlock(&ddata->function_lock);
	return err;
}

/* Must always be called with mutex list_lock held */
static void rmi4_core_driver_clear_pdt_list(struct rmi4_core_device *cdev)
{
	struct list_head *list;
	struct list_head *n;
	struct rmi4_pdt_container *entry;
	struct rmi4_core_drv_data *data = dev_get_drvdata(&cdev->dev);

	if (list_empty(&data->pdt_list))
		return;

	list_for_each_safe(list, n, &data->pdt_list) {
		entry = to_rmi4_pdt_container(list);
		dev_dbg(&cdev->dev, "%s - Removing PDT for func 0x%02X\n",
			__func__, entry->pdt.function_number);
		list_del(list);
		kfree(entry);
	}
}

static struct rmi4_pdt_container
*rmi4_core_driver_new_pdt_entry(struct rmi4_core_device *cdev)
{
	struct rmi4_pdt_container *e = kzalloc(sizeof(*e), GFP_KERNEL);

	if (!e) {
		dev_err(&cdev->dev,
			"%s - Failed to create RMI4 function entry\n",
			__func__);
		return NULL;
	}

	INIT_LIST_HEAD(&e->entry);

	return e;
}

static void rmi4_core_driver_dump_pdt_entry(struct rmi4_core_device *cdev,
					    struct rmi4_pdt_entry *entry)
{
	dev_dbg(&cdev->dev, "\n--- PTD DUMP START ---\n");
	dev_dbg(&cdev->dev, "Function number = 0x%02X\n",
		 entry->function_number);
	dev_dbg(&cdev->dev, "Query base addr = 0x%02X\n",
		 entry->query_base_addr);
	dev_dbg(&cdev->dev, "Command base addr = 0x%02X\n",
		 entry->command_base_addr);
	dev_dbg(&cdev->dev, "Control base addr = 0x%02X\n",
		 entry->control_base_addr);
	dev_dbg(&cdev->dev, "Data base addr = 0x%02X\n",
		 entry->data_base_addr);
	dev_dbg(&cdev->dev, "Interrupt source count = 0x%02X\n",
		 entry->interrupt_source_count);
	dev_dbg(&cdev->dev, "Bits 3 & 4 = 0x%02X\n", entry->bits3and4);
	dev_dbg(&cdev->dev, "Function version = 0x%02X\n",
		 entry->function_version);
	dev_dbg(&cdev->dev, "Bit 7 = 0x%02X\n", entry->bit7);
	dev_dbg(&cdev->dev, "\n--- PTD DUMP END ---\n");
}

static int rmi4_core_driver_add_pdt_entry(struct rmi4_core_device *cdev,
					  struct rmi4_pdt_entry *pdt,
					  u8 bank)
{
	struct rmi4_core_drv_data *data = dev_get_drvdata(&cdev->dev);
	struct rmi4_pdt_container *entry =
		rmi4_core_driver_new_pdt_entry(cdev);

	if (!entry) {
		dev_err(&cdev->dev, "%s - Failed to create PDT container\n",
			__func__);
		return -ENOMEM;
	}

	memcpy(&entry->pdt, pdt, sizeof(entry->pdt));
	/* bank num is upper byte of u16. Bit shift it here so it
	 * will be usable later on */
	entry->bank = bank << 8;

	list_add_tail(&entry->entry, &data->pdt_list);

	dev_dbg(&cdev->dev, "%s - Added PDT to list\n", __func__);

	return 0;
}

static int rmi4_read_pdt(struct rmi4_core_device *cdev)
{
	int retval;
	int page;
	struct rmi4_pdt_entry pdt;
	struct rmi4_core_drv_data *data = dev_get_drvdata(&cdev->dev);

	dev_dbg(&cdev->dev, "%s - Called\n", __func__);

	if (!cdev->read) {
		dev_err(&cdev->dev,
			"%s - Core device not initialized. No read func\n",
			__func__);
		return -EINVAL;
	}

	rmi4_core_driver_clear_pdt_list(cdev);

	for (page = 0; page <= RMI4_MAX_PAGE; page++) {
		int i;
		int index;
		int err;
		u16 page_start = RMI4_PAGE_SIZE * page;
		u16 pdt_start = page_start + PDT_START_SCAN_LOCATION;
		u16 pdt_end = page_start + PDT_END_SCAN_LOCATION;

		dev_dbg(&cdev->dev, "%s - Checking page 0x%4X\n", __func__,
			page_start);

		for (index = 0, i = pdt_start; i >= pdt_end;
		     i -= sizeof(pdt), index++) {
			retval = cdev->read(cdev, i, (u8 *)&pdt,
					    sizeof(pdt));
			if (retval != sizeof(pdt)) {
				dev_err(&cdev->dev,
					"%s - Read PDT entry at 0x%X failed.\n",
					__func__, i);
				retval = -EINVAL;
				goto error;
			}

			if (RMI4_END_OF_PDT(pdt.function_number)) {
				dev_dbg(&cdev->dev,
					"%s - End of PDT\n", __func__);
				break;
			}

			if (rmi4_core_driver_get_pdt_container(
				    cdev, pdt.function_number)) {
				dev_dbg(&cdev->dev,
					 "%s - F0x%02X already registered\n",
					 __func__, pdt.function_number);
				continue;
			}
			dev_dbg(&cdev->dev,
				"%s - Adding F%02X at bank 0x%02X\n",
				__func__, pdt.function_number, page);
			rmi4_core_driver_dump_pdt_entry(cdev, &pdt);

			err = rmi4_core_driver_add_pdt_entry(cdev, &pdt, page);
			if (err)
				goto error;

			dev_info(&cdev->dev,
				 "Chip supports function F%02X\n",
				 pdt.function_number);
			dev_dbg(&cdev->dev, "%s\n", "\n\n");
		}
	}

	dev_dbg(&cdev->dev, "%s - PDT scan successfull!\n", __func__);

	if (list_empty(&data->pdt_list)) {
		dev_err(&cdev->dev, "%s - No functions found. Aborting!\n",
			__func__);
		retval = -ENOENT;
		goto error;
	}

	return 0;

error:
	dev_err(&cdev->dev, "PDT scan failed\n");
	rmi4_core_driver_clear_pdt_list(cdev);

	return retval;
}

static int rmi4_core_driver_read_pdt(struct rmi4_function_device *fdev)
{
	int err;
	struct rmi4_core_drv_data *dd = dev_get_drvdata(fdev->dev.parent);

	LOCK_PDT(dd);
	err = rmi4_read_pdt(to_rmi4_core_device(fdev->dev.parent));
	UNLOCK_PDT(dd);

	return err;
}

static void rmi4_core_driver_reset(struct rmi4_core_device *cdev)
{
	u8 mask;
	int err;
	struct list_head *n;
	struct list_head *list;
	struct rmi4_function_container *fc;
	struct rmi4_core_drv_data *dd = dev_get_drvdata(&cdev->dev);

	LOCK_PDT(dd);
	err = rmi4_read_pdt(dd->cdev);
	UNLOCK_PDT(dd);
	if (err) {
		dev_err(&cdev->dev,
			"%s - Failed to rescan PDT\n", __func__);
		return;
	}

	err = rmi4_core_driver_populate_functiondata(cdev);
	if (err) {
		dev_err(&cdev->dev, "%s - Failed to populate function data\n",
			 __func__);
		return;
	}

	rmi4_core_disable_all_irqs(cdev);

	/* Re-enabling all subscribed IRQ's */
	list_for_each_safe(list, n, &dd->function_list) {
		fc = to_rmi4_function_container(list);
		if (!list_empty(&fc->irq_client_list)) {
			dev_dbg(&cdev->dev, "%s - Re-enabling IRQs f%02X\n",
				 __func__, fc->function_id);
			mask = rmi4_core_driver_get_irq_mask(cdev, fc);
			rmi4_core_driver_set_irq_state(cdev, fc, mask);
		}
	}

	err = rmi4_core_driver_register_functions(cdev);
	if (err) {
		dev_err(&dd->cdev->dev, "%s - Failed to register functions\n",
			__func__);
		return;
	}

	dev_dbg(&dd->cdev->dev, "Successfully reset rmi4 driver\n");
}

int rmi4_core_driver_set_non_essential_irq_status(
	struct rmi4_function_device *fdev, bool disable)
{
	u8 mask;
	int count = 0;
	int err = 0;
	struct list_head *n;
	struct list_head *list;
	struct rmi4_function_container *fc;
	struct rmi4_core_drv_data *ddata = dev_get_drvdata(fdev->dev.parent);
	struct rmi4_core_device *cdev = to_rmi4_core_device(fdev->dev.parent);

	mutex_lock(&ddata->function_lock);

	list_for_each_safe(list, n, &ddata->function_list) {
		fc = to_rmi4_function_container(list);
		count = atomic_read(&fc->force_irq_disabled);
		if (0x01 == fc->function_id ||
		    0x34 == fc->function_id ||
		    0x54 == fc->function_id) {
			dev_dbg(&fdev->dev, "%s - Essential IRQ found\n",
				__func__);
			continue;
		}
		if (disable) {
			if (0 == count && !list_empty(&fc->irq_client_list)) {
				dev_dbg(&fdev->dev,
					"%s - Disabling IRQ for F%02X\n",
					__func__, fc->function_id);
				err = rmi4_core_driver_set_irq_state(
					to_rmi4_core_device(fdev->dev.parent),
					fc, 0);
			}
			dev_dbg(&fdev->dev, "%s - disable: count = %d\n",
				__func__, count);
			atomic_inc(&fc->force_irq_disabled);
		} else {
			if (0 == count)
				continue;

			if (1 == count && !list_empty(&fc->irq_client_list)) {
				dev_dbg(&fdev->dev,
					 "%s - Enabling IRQ for F%02X\n",
					 __func__, fc->function_id);
				mask = rmi4_core_driver_get_irq_mask(cdev, fc);
				err = rmi4_core_driver_set_irq_state(
					to_rmi4_core_device(fdev->dev.parent),
					fc, mask);
			}
			dev_dbg(&fdev->dev, "%s - enable: count = %d\n",
				__func__, count);
			atomic_dec(&fc->force_irq_disabled);
		}
		dev_dbg(&fdev->dev, "%s - done: count = %d\n",
			__func__, atomic_read(&fc->force_irq_disabled));
	}

	mutex_unlock(&ddata->function_lock);
	return err;
}

static void rmi4_core_drv_process_irq(struct rmi4_core_device *cdev,
				      struct rmi4_function_container *fc,
				      int bit_id)
{
	struct list_head *n;
	struct list_head *list;
	struct rmi4_irq_client *client;
	unsigned long l_mask;

	list_for_each_safe(list, n, &fc->irq_client_list) {
		client = to_rmi4_irq_client(list);
		l_mask = client->irq_mask;
		if (test_bit(bit_id, &l_mask) && client->func)
			client->func(bit_id, client->func_data);
	}

}

static void rmi4_dump_irq_register(struct rmi4_core_device *cdev)
{
#ifdef DEBUG
	int i;
	struct rmi4_core_drv_data *ddata = dev_get_drvdata(&cdev->dev);
	char buf[ddata->num_supported_irqs * 8 + 1];

	for (i = 0; i < ddata->num_supported_irqs; i++)
		snprintf(buf + i, sizeof(buf) - i, "%u",
			 test_bit(i, (unsigned long *)ddata->irq_bits));
	buf[i] = '\n';

	dev_info(&cdev->dev, "irq register dump (LSB first): %s", buf);
#endif
}

static irqreturn_t rmi4_core_drv_irq_handler(int irq, void *data)
{
	int i;
	int err;
	struct list_head *n;
	struct list_head *list;
	struct rmi4_pdt_container *pdt;
	struct rmi4_function_container *fc;
	struct rmi4_core_device *cdev = data;
	struct rmi4_core_drv_data *ddata = dev_get_drvdata(&cdev->dev);

	dev_dbg(&cdev->dev, "%s - Called\n", __func__);

	/* If we're asleep, wait until we are awake */
	wait_event(ddata->wq, !!(0 == atomic_cmpxchg(&ddata->suspended, 0 ,1)));

	dev_dbg(&cdev->dev, "System no longer suspended. Reading stuff\n");

	pdt = rmi4_core_driver_get_pdt_container(cdev, F01_FUNCTION_ID);
	if (!pdt) {
		dev_err(&cdev->dev, "%s - F01 not supported. FATAL ERROR!\n",
			__func__);

		return IRQ_HANDLED;
	}

	dev_dbg(&cdev->dev, "%s- Reading from addr 0x%x\n", __func__,
		pdt->pdt.data_base_addr + F01_IRQ_ADDR_OFFSET +
		pdt->bank);

	err = cdev->read(cdev, pdt->pdt.data_base_addr + F01_IRQ_ADDR_OFFSET +
			 pdt->bank, ddata->irq_bits, ddata->irq_read_len);
	if (err != ddata->irq_read_len) {
		dev_err(&cdev->dev, "%s - Failed to read IRQ data\n",
			__func__);
		goto done;
	}

	dev_dbg(&cdev->dev, "%s - Done reading IRQ state\n", __func__);
	rmi4_dump_irq_register(cdev);

	if (list_empty(&ddata->function_list))
		goto done;

	list_for_each_safe(list, n, &ddata->function_list) {
		fc = to_rmi4_function_container(list);
		if (list_empty(&fc->irq_client_list)) {
			dev_dbg(&cdev->dev, "%s - f%02X - NO IRQ's stored\n",
				__func__, fc->function_id);
			continue;
		}
		for (i = 0; i < fc->num_irqs; i++)
			if (test_bit(fc->irq_start_pos + i,
				     (unsigned long *)ddata->irq_bits) &&
			    !atomic_read(&fc->force_irq_disabled))
				rmi4_core_drv_process_irq(cdev, fc, i);
	}

	dev_dbg(&cdev->dev, "%s - IRQ Handled\n", __func__);

done:
	atomic_set(&ddata->suspended, 0);
	wake_up(&ddata->wq);
	dev_dbg(&cdev->dev, "Released lock\n");
	return IRQ_HANDLED;
}

static int rmi4_core_update_pdt_properties(struct rmi4_core_device *cdev,
					   bool read)
{
	int err;
	struct rmi4_core_drv_data *ddata = dev_get_drvdata(&cdev->dev);

	if (read)
		err = cdev->read(cdev, RMI4_PDT_PROPERTIES_ADDR,
				 (u8 *)&ddata->pdt_properties.reg,
				 sizeof(ddata->pdt_properties.reg));
	else
		err = cdev->write(cdev, RMI4_PDT_PROPERTIES_ADDR,
				 (u8 *)&ddata->pdt_properties.reg,
				 sizeof(ddata->pdt_properties.reg));

	if (0 > err) {
		dev_err(&cdev->dev, "%s - Failed to %s PDT properties\n",
			__func__, read ? "read" : "write");
		return err;
	}

	return 0;
}

static ssize_t rmi4_core_bsr_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int err;
	struct rmi4_core_device *cdev = to_rmi4_core_device(dev);
	struct rmi4_core_drv_data *ddata = dev_get_drvdata(dev);

	mutex_lock(&ddata->pdt_prop_lock);
	err = rmi4_core_update_pdt_properties(cdev, true);
	if (err)
		goto exit;

	err = snprintf(buf, PAGE_SIZE, "%u\n", ddata->pdt_properties.has_bsr);

exit:
	mutex_unlock(&ddata->pdt_prop_lock);

	return err;
}

static ssize_t rmi4_core_bsr_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int err;
	unsigned long val;
	struct rmi4_core_device *cdev = to_rmi4_core_device(dev);
	struct rmi4_core_drv_data *ddata = dev_get_drvdata(dev);

	mutex_lock(&ddata->pdt_prop_lock);
	err = strict_strtoul(buf, 10, &val);
	if (0 > err) {
		dev_err(dev, "Invalid value '%s' written to BSR.\n", buf);
		goto exit;
	}

	if (1 < val) {
		dev_err(dev, "Invalid value %lu. Accepted are 0 and 1\n", val);
		err = -EINVAL;
		goto exit;
	}

	ddata->pdt_properties.has_bsr = val;

	err = rmi4_core_update_pdt_properties(cdev, false);

exit:
	mutex_unlock(&ddata->pdt_prop_lock);
	return err ? err : count;
}

static ssize_t rmi4_core_enable_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int err;
	struct rmi4_core_drv_data *ddata = dev_get_drvdata(dev);

	mutex_lock(&ddata->data_lock);
	err = snprintf(buf, PAGE_SIZE, "%d\n", ddata->enabled);
	mutex_unlock(&ddata->data_lock);
	return err;

}

static ssize_t  rmi4_core_enable_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	int err;
	bool val;
	struct rmi4_core_device *cdev = to_rmi4_core_device(dev);
	struct rmi4_core_drv_data *ddata = dev_get_drvdata(dev);
	struct rmi4_core_device_data *cdata = dev_get_platdata(dev);

	val = sysfs_streq(buf, "1");

	if (val == ddata->enabled) {
		dev_info(dev, "State already: %d\n", val);
		return count;
	}

	mutex_lock(&ddata->data_lock);
	if (val) {
		err = request_threaded_irq(ddata->irq, NULL,
					   rmi4_core_drv_irq_handler,
					   cdata->irq_polarity,
					   dev_name(&cdev->dev), cdev);
		if (err)
			dev_err(dev,
				"Failed to restart IRQ %d\n",
				cdata->attn_gpio);
	} else {
		free_irq(ddata->irq, cdev);
	}
	ddata->enabled = val;
	mutex_unlock(&ddata->data_lock);

	return count;
}

static struct device_attribute rmi4_core_attrs[] = {
	__ATTR(bsr, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
	       rmi4_core_bsr_show, rmi4_core_bsr_store),
	__ATTR(enable, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
	       rmi4_core_enable_show, rmi4_core_enable_store),
};

static int rmi4_core_create_sysfs_files(struct rmi4_core_device *cdev,
					bool create)
{
	int i;
	int err = 0;

	for (i = 0; i < ARRAY_SIZE(rmi4_core_attrs); i++) {
		if (create) {
			err = sysfs_create_file(&cdev->dev.kobj,
						&rmi4_core_attrs[i].attr);
			if (err) {
				dev_err(&cdev->dev,
					"%s - Failed to create sysfs files\n",
					__func__);
				goto fail;
			}
		} else {
			sysfs_remove_file(&cdev->dev.kobj,
					  &rmi4_core_attrs[i].attr);
		}
	}

	return err;

fail:
	for (; i >= 0; i--)
		sysfs_remove_file(&cdev->dev.kobj, &rmi4_core_attrs[i].attr);

	return err;
}

static void rmi4_core_driver_notified(enum rmi4_notification_event event,
				      void *data)
{
	struct rmi4_core_device *cdev = data;

	dev_dbg(&cdev->dev, "%s - Called\n", __func__);
	if (RMI4_DRIVER_RESET & event) {
		dev_dbg(&cdev->dev, "%s - Resetting driver\n", __func__);
		rmi4_core_driver_reset(cdev);
	} else {
		dev_warn(&cdev->dev, "%s - Registered event %d not handled\n",
			 __func__, event);
	}
}

static int rmi4_core_driver_probe(struct rmi4_core_device *cdev)
{
	int err;
	int irq;
	struct rmi4_core_device_data *pdata = dev_get_platdata(&cdev->dev);
	struct rmi4_core_drv_data *data = kzalloc(sizeof(*data), GFP_KERNEL);

	dev_dbg(&cdev->dev, "%s - Called\n", __func__);

	if (!data) {
		dev_err(&cdev->dev,
			"%s - Failed to allocate driver data\n", __func__);
		return -ENOMEM;
	}

	mutex_init(&data->pdt_lock);
	mutex_init(&data->function_lock);
	mutex_init(&data->pdt_prop_lock);
	mutex_init(&data->data_lock);

	INIT_LIST_HEAD(&data->pdt_list);
	INIT_LIST_HEAD(&data->function_list);
	INIT_LIST_HEAD(&data->notify_list);
	data->cdev = cdev;

	init_waitqueue_head(&data->wq);

	dev_set_drvdata(&cdev->dev, data);

	err = rmi4_read_pdt(cdev);
	if (err) {
		dev_warn(&cdev->dev,
			 "%s - Failed to scan PDT. Touch may not work\n",
			 __func__);
		goto fail;
	}

	err = rmi4_core_driver_populate_functiondata(cdev);
	if (err)
		goto fail_pdt;

	/* num_supported_irqs is updated in the previous function call. */
	data->irq_read_len = (data->num_supported_irqs / 8) + 1;
	dev_dbg(&cdev->dev, "Num bytes required: %d\n",
		 data->irq_read_len);

	data->irq_bits = kzalloc(data->irq_read_len * sizeof(u8), GFP_KERNEL);
	if (!data->irq_bits) {
		dev_err(&cdev->dev, "Failed to allocate IRQ buffer\n");
		goto fail_pdt;
	}

	err = rmi4_core_disable_all_irqs(cdev);
	if (err)
		goto fail_populate;

	dev_dbg(&cdev->dev, "%s - All IRQ's are disabled\n", __func__);

	if (0 >= pdata->attn_gpio) {
		dev_err(&cdev->dev, "%s - Attention GPIO is required\n",
			__func__);
		err = -EINVAL;
		goto fail_populate;
	}
	dev_dbg(&cdev->dev, "%s - All pending ISR's cleared\n", __func__);

	if (!pdata->irq_is_shared) {
		err = gpio_request(pdata->attn_gpio, "RMI4_core_IRQ");
		if (err) {
			dev_err(&cdev->dev,
				"%s -Failed to request gpio. Err: %d\n",
				__func__, err);
			goto fail_populate;
		}
	} else {
		pdata->irq_polarity |= IRQF_SHARED;
	}

	irq = gpio_to_irq(pdata->attn_gpio);
	if (0 > irq) {
		dev_err(&cdev->dev,
			"%s - Failed to retrieve correct IRQ\n",
			__func__);
		err = -ENODEV;
		goto fail_gpio_req;
	}

	err = request_threaded_irq(irq, NULL,
				   rmi4_core_drv_irq_handler,
				   pdata->irq_polarity,
				   dev_name(&cdev->dev), cdev);
	if (err) {
		dev_err(&cdev->dev,
			"request_threaded_irq failed %d\n",
			pdata->attn_gpio);
		goto fail_gpio_req;
	}
	data->irq = irq;
	data->enabled = true;
	enable_irq_wake(data->irq);

	err = rmi4_core_create_sysfs_files(cdev, true);
	if (err) {
		dev_err(&cdev->dev, "%s - Failed to create sysfs files\n",
			__func__);
		goto fail_irq;
	}

	err = _rmi4_core_driver_request_notification(cdev, RMI4_DRIVER_RESET,
						     rmi4_core_driver_notified,
						     cdev);
	if (err) {
		dev_err(&cdev->dev,
			"%s - Failed to register for notification events\n",
			__func__);
		goto fail_sysfs;
	}

	err = rmi4_core_driver_register_functions(cdev);
	if (err) {
		dev_err(&cdev->dev, "%s - Failed to register functions\n",
			__func__);
		goto fail_notify;
	}
	dev_info(&cdev->dev, "Successfully probed rmi4_core-driver\n");

	/* If this fails, it's not really an error */
	err = rmi4_core_driver_register_char_func(cdev);
	if (err)
		dev_warn(&cdev->dev, "Failed to register char dev function.\n");

	goto done;

fail_notify:
	_rmi4_core_driver_release_notification(cdev, cdev);
fail_sysfs:
	rmi4_core_create_sysfs_files(cdev, false);
fail_irq:
	free_irq(data->irq, cdev);
fail_gpio_req:
	if (!pdata->irq_is_shared)
		gpio_free(pdata->attn_gpio);
fail_populate:
	kfree(data->irq_bits);
	rmi4_core_driver_clear_functiondata(cdev);
fail_pdt:
	rmi4_core_driver_clear_pdt_list(cdev);
fail:

	mutex_destroy(&data->pdt_lock);
	mutex_destroy(&data->function_lock);
	mutex_destroy(&data->irq_lock);
	mutex_destroy(&data->reg_func_lock);
	mutex_destroy(&data->data_lock);

	dev_set_drvdata(&cdev->dev, NULL);
	kfree(data);
	dev_err(&cdev->dev, "Failed to probe core driver\n");

done:
	return err;
}

static int rmi4_core_driver_remove(struct rmi4_core_device *cdev)
{
	struct rmi4_core_drv_data *data = dev_get_drvdata(&cdev->dev);
	struct rmi4_core_device_data *pdata = dev_get_platdata(&cdev->dev);

	dev_dbg(&cdev->dev, "%s - Called\n", __func__);

	rmi4_core_driver_unregister_char_dev(cdev);

	rmi4_core_driver_unregister_functions(cdev);
	_rmi4_core_driver_release_notification(cdev, cdev);
	rmi4_core_create_sysfs_files(cdev, false);
	free_irq(data->irq, cdev);

	if (!pdata->irq_is_shared)
		gpio_free(pdata->attn_gpio);

	rmi4_core_disable_all_irqs(cdev);
	rmi4_core_driver_clear_functiondata(cdev);
	rmi4_core_driver_clear_pdt_list(cdev);

	mutex_destroy(&data->pdt_lock);
	mutex_destroy(&data->function_lock);
	mutex_destroy(&data->irq_lock);
	mutex_destroy(&data->reg_func_lock);
	mutex_destroy(&data->data_lock);

	kfree(data->irq_bits);
	dev_set_drvdata(&cdev->dev, NULL);
	kfree(data);

	return 0;
}

static int rmi4_core_driver_read_data(struct rmi4_function_device *fdev,
				      enum rmi4_data_command cmd,
				      int addr_offset, u8 *data, int data_len)
{
	int err;
	u16 addr;
	struct rmi4_core_device *cdev = to_rmi4_core_device(fdev->dev.parent);
	struct rmi4_core_drv_data *ddata = dev_get_drvdata(&cdev->dev);
	struct rmi4_pdt_container *fentry;

	dev_dbg(&fdev->dev, "%s - Called\n", __func__);

	LOCK_PDT(ddata);
	fentry = rmi4_core_driver_get_pdt_container(cdev, fdev->func_id);
	if (!fentry) {
		err = -ENODEV;
		goto exit;
	}

	switch (cmd) {
	case QUERY:
		addr = fentry->pdt.query_base_addr + addr_offset + fentry->bank;
		break;
	case COMMAND:
		addr = fentry->pdt.command_base_addr + addr_offset +
			fentry->bank;
		break;
	case CONTROL:
		addr = fentry->pdt.control_base_addr + addr_offset +
			fentry->bank;
		break;
	case DATA:
		addr = fentry->pdt.data_base_addr + addr_offset + fentry->bank;
		break;
	default:
		dev_err(&fdev->dev, "%s - Tried to execute invalid command\n",
			__func__);
		err = -EINVAL;
		goto exit;
	}

	if (cdev && cdev->read) {
		err = cdev->read(cdev, addr, data, data_len);
	} else {
		dev_err(&fdev->dev,
			"%s - Major error: Read function not initialized!\n",
			__func__);
		err = -ENODEV;
	}

exit:
	UNLOCK_PDT(ddata);
	return err;
}

static int rmi4_core_driver_write_data(struct rmi4_function_device *fdev,
				       enum rmi4_data_command cmd,
				       int addr_offset, u8 *data, int data_len)
{
	int err;
	u16 addr;
	struct rmi4_core_device *cdev = to_rmi4_core_device(fdev->dev.parent);
	struct rmi4_core_drv_data *ddata = dev_get_drvdata(&cdev->dev);
	struct rmi4_pdt_container *fentry;

	dev_dbg(&fdev->dev, "%s - Called. Func = 0x%02X\n", __func__,
		fdev->func_id);

	LOCK_PDT(ddata);
	fentry = rmi4_core_driver_get_pdt_container(cdev, fdev->func_id);
	if (!fentry) {
		dev_err(&fdev->dev,
			"%s - Cant read from non existing function\n",
			__func__);
		err = -ENODEV;
		goto exit;
	}

	switch (cmd) {
	case QUERY:
		addr = fentry->pdt.query_base_addr + addr_offset + fentry->bank;
		break;
	case COMMAND:
		addr = fentry->pdt.command_base_addr + addr_offset +
			fentry->bank;
		break;
	case CONTROL:
		addr = fentry->pdt.control_base_addr + addr_offset +
			fentry->bank;
		break;
	case DATA:
		addr = fentry->pdt.data_base_addr + addr_offset + fentry->bank;
		break;
	default:
		dev_err(&fdev->dev, "%s - Tried to execute invalid command\n",
			__func__);
		err = -EINVAL;
		goto exit;
	}

	if (cdev && cdev->write) {
		err = cdev->write(cdev, addr, data, data_len);
	} else {
		dev_err(&fdev->dev,
			"%s - Major error: Read function not initialized!\n",
			__func__);
		err = -ENODEV;
	}

exit:
	mutex_unlock(&ddata->pdt_lock);
	return err;
}

static int rmi4_core_driver_request_irq(struct rmi4_function_device *fdev,
					void *data, void (*fn)(int, void *),
					u8 irq_mask)
{
	int err = 0;
	struct rmi4_irq_client *irq_client;
	struct rmi4_function_container *fc;
	struct rmi4_core_device *cdev = to_rmi4_core_device(fdev->dev.parent);
	struct rmi4_core_drv_data *ddata = dev_get_drvdata(&cdev->dev);


	dev_dbg(&fdev->dev, "%s - Called\n", __func__);

	if (0x7F < irq_mask) {
		dev_err(&fdev->dev, "%s - Max 7 IRQ's are supported\n",
			__func__);
		return -EINVAL;
	}

	LOCK_PDT(ddata);

	fc = rmi4_core_driver_get_function_container(cdev, fdev->func_id);
	if (!fc) {
		dev_err(&cdev->dev,
			"%s - IRQ for function F%02X is not supported\n",
			__func__, fdev->func_id);
		err = -ENODEV;
		goto exit;
	}

	irq_client = kzalloc(sizeof(*irq_client), GFP_KERNEL);
	if (!irq_client) {
		dev_err(&cdev->dev,
			"%s - Failed to create an IRQ_Client. No memory\n",
			__func__);
		err = -ENOMEM;
		goto exit;
	}
	INIT_LIST_HEAD(&irq_client->entry);

	err = rmi4_core_driver_set_irq_state(cdev, fc, irq_mask);
	if (err) {
		dev_err(&cdev->dev,
			"%s - Failed to enable IRQ for func F%02X\n",
			__func__, fdev->func_id);
		kfree(irq_client);
		goto exit;
	}

	irq_client->func = fn;
	irq_client->func_data = data;
	irq_client->irq_mask |= irq_mask;

	mutex_lock(&fc->irq_client_lock);
	list_add_tail(&irq_client->entry, &fc->irq_client_list);
	mutex_unlock(&fc->irq_client_lock);

	dev_dbg(&fdev->dev, "%s - Added IRQ for function f%02X\n", __func__,
		 fdev->func_id);

exit:
	UNLOCK_PDT(ddata);
	return err;
}

static int rmi4_core_driver_free_irq(struct rmi4_function_device *fdev,
				     void *data)
{
	u8 mask = 0;
	int err = 0;
	struct list_head *n;
	struct list_head *list;
	struct rmi4_irq_client *irq_client = NULL;
	struct rmi4_function_container *fc;
	struct rmi4_core_device *cdev = to_rmi4_core_device(fdev->dev.parent);
	struct rmi4_core_drv_data *ddata = dev_get_drvdata(&cdev->dev);

	dev_dbg(&fdev->dev, "%s - Called\n", __func__);

	LOCK_PDT(ddata);

	fc = rmi4_core_driver_get_function_container(cdev, fdev->func_id);
	if (!fc) {
		dev_err(&cdev->dev,
			"%s - Function F%02X is not supported\n",
			__func__, fdev->func_id);
		err = -ENODEV;
		goto exit;
	}
	dev_dbg(&fdev->dev, "%s - Located function\n", __func__);

	if (list_empty(&fc->irq_client_list)) {
		dev_err(&cdev->dev, "%s - Tried to free unsibscribed IRQ\n",
			__func__);
		err = -EINVAL;
		goto exit;
	}
	dev_dbg(&fdev->dev,
		 "%s - Someone has subscribed for this IRQ (f%02X)\n",
		 __func__, fdev->func_id);

	mutex_lock(&fc->irq_client_lock);
	list_for_each_safe(list, n, &fc->irq_client_list) {
		irq_client = to_rmi4_irq_client(list);
		if (irq_client->func_data == data) {
			dev_dbg(&cdev->dev, "%s - f%02X Match found\n",
				__func__, fdev->func_id);
			list_del(list);
			kfree(irq_client);
		} else {
			dev_dbg(&fdev->dev, "%s - f%02X Match not found\n",
				__func__, fdev->func_id);
		}
	}
	mutex_unlock(&fc->irq_client_lock);

	if (!list_empty(&fc->irq_client_list))
		mask = rmi4_core_driver_get_irq_mask(cdev, fc);

	err = rmi4_core_driver_set_irq_state(cdev, fc, mask);
	if (err)
		dev_err(&cdev->dev,
			"%s - Failed to disable IRQ for func F%02X\n",
			__func__, fdev->func_id);

exit:
	UNLOCK_PDT(ddata);
	return err;
}

static int _rmi4_core_driver_request_notification(
	struct rmi4_core_device *cdev, enum rmi4_notification_event events,
	void (*callback)(enum rmi4_notification_event event, void *data),
	void *data)
{
	struct rmi4_core_drv_data *ddata = dev_get_drvdata(&cdev->dev);
	struct rmi4_notify_client *c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c) {
		dev_err(&cdev->dev,
			"%s - Failed to allocate notification data\n",
			__func__);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&c->entry);
	c->event_data = data;
	c->event = events;
	c->callback = callback;

	list_add_tail(&c->entry, &ddata->notify_list);

	return 0;
}

static int rmi4_core_driver_request_notification(
	struct rmi4_function_device *fdev, enum rmi4_notification_event events,
	void (*callback)(enum rmi4_notification_event event, void *data),
	void *data)
{
	struct rmi4_core_device *cdev = to_rmi4_core_device(fdev->dev.parent);
	return _rmi4_core_driver_request_notification(cdev, events, callback,
						      data);
}

static void _rmi4_core_driver_release_notification(
	struct rmi4_core_device *cdev, void *data)
{
	struct list_head *n;
	struct list_head *list;
	struct rmi4_notify_client *c;
	struct rmi4_core_drv_data *ddata = dev_get_drvdata(&cdev->dev);

	if (list_empty(&ddata->notify_list))
		return;

	list_for_each_safe(list, n, &ddata->notify_list) {
		c = to_rmi4_notify_client(list);
		if (c->event_data == data) {
			list_del(list);
			kfree(c);
		}
	}
}

static void rmi4_core_driver_release_notification(
	struct rmi4_function_device *fdev, void *data)
{
	struct rmi4_core_device *cdev = to_rmi4_core_device(fdev->dev.parent);
	_rmi4_core_driver_release_notification(cdev, data);
}

static void _rmi4_core_driver_notify(struct rmi4_core_device *cdev,
				     enum rmi4_notification_event event)
{
	struct list_head *n;
	struct list_head *list;
	struct rmi4_notify_client *c;
	struct rmi4_core_drv_data *ddata = dev_get_drvdata(&cdev->dev);

	if (list_empty(&ddata->notify_list))
		return;

	list_for_each_safe(list, n, &ddata->notify_list) {
		c = to_rmi4_notify_client(list);
		if ((c->event & event) && c->callback)
			c->callback(event, c->event_data);
	}
}

static void rmi4_core_driver_notify(struct rmi4_function_device *fdev,
				    enum rmi4_notification_event event)
{
	_rmi4_core_driver_notify(to_rmi4_core_device(fdev->dev.parent), event);
}

#ifdef CONFIG_PM
static int rmi4_core_driver_suspend(struct device *dev)
{
	struct rmi4_core_drv_data *ddata = dev_get_drvdata(dev);

	dev_dbg(dev, "Suspend called\n");
	wait_event(ddata->wq, atomic_cmpxchg(&ddata->suspended, 0, 1));
	dev_vdbg(dev, "Suspended\n");

	return 0;
}

static int rmi4_core_driver_resume(struct device *dev)
{
	struct rmi4_core_drv_data *ddata = dev_get_drvdata(dev);

	dev_dbg(dev, "Resume called\n");
	atomic_set(&ddata->suspended, 0);
	wake_up(&ddata->wq);
	dev_vdbg(dev, "Resumed\n");

	return 0;
}
#else
#define rmi4_core_driver_suspend NULL
#define rmi4_core_driver_resume NULL
#endif

static SIMPLE_DEV_PM_OPS(rmi4_core_driver_pm_ops, rmi4_core_driver_suspend,
			 rmi4_core_driver_resume);

static struct rmi4_core_driver driver = {
	.probe = rmi4_core_driver_probe,
	.remove = rmi4_core_driver_remove,

	.read_pdt = rmi4_core_driver_read_pdt,
	.read = rmi4_core_driver_read_data,
	.write = rmi4_core_driver_write_data,
	.request_irq = rmi4_core_driver_request_irq,
	.free_irq = rmi4_core_driver_free_irq,
	.disable_non_essential_irqs =
		rmi4_core_driver_set_non_essential_irq_status,
	.request_notification = rmi4_core_driver_request_notification,
	.release_notification = rmi4_core_driver_release_notification,
	.notify = rmi4_core_driver_notify,

	.drv = {
		.name = RMI4_CORE_DRIVER_NAME,
		.pm = &rmi4_core_driver_pm_ops,
	},
};

static int rmi4_core_driver_init(void)
{
	pr_info("Registering %s on RMI4 bus\n", driver.drv.name);
	return rmi4_bus_register_core_driver(&driver);
}

static void rmi4_core_driver_exit(void)
{
	pr_info("Unregistering %s from RMI4 bus\n", driver.drv.name);
	rmi4_bus_unregister_core_driver(&driver);
}

module_init(rmi4_core_driver_init);
module_exit(rmi4_core_driver_exit);

MODULE_AUTHOR("Joachim Holst <joachim.holst@sonyerisson.com>");
MODULE_DESCRIPTION("RMI4 core driver");
MODULE_LICENSE("GPL");
