/*
 * Last modified: Jul 27th, 2012
 * Revision: V1.4
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2012 Bosch Sensortec GmbH
 * Copyright(C) 2013 Foxconn International Holdings, Ltd. All rights reserved.
 * All Rights Reserved
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/uaccess.h>
 #include <linux/fih_hw_info.h>
#include "tfa9887.h"

/* sensor specific */
#define SENSOR_NAME "tfa9887"

#define SENSOR_CHIP_ID_tfa9887 (0x12)

#define tfa9887_REG_NAME(name) tfa9887_##name
#define tfa9887_CALL_API(name) tfa9887_##name

#define tfa9887_I2C_WRITE_DELAY_TIME 1

/* generic */
#define tfa9887_MAX_RETRY_I2C_XFER (10)//(100)

#define BYTES_PER_LINE (16)

#define tfa9887_SELF_TEST 0

#define TFA9887_CHIP_ID_ADDR	0x03

/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO fmt, ##args)                 
#define FTM_CUST_ALSPS "/data/tfa9887"

/*----------------------------------------------------------------------------*/
u8 read_size;
u8 ReadReg;
u8 PreReadReg = 0xFF;
u32 repeat = 0;

struct op_mode_map {
	char *op_mode_name;
	long op_mode;
};

struct tfa9887_client_data {
	struct tfa9887_t device;
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;

	atomic_t delay;
	u8 enable:1;

	int layout;
};

static struct i2c_client *tfa9887_client;
/* i2c operation for API */
static void tfa9887_i2c_delay(tfa9887_S32 msec);
static char tfa9887_i2c_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len);
static char tfa9887_i2c_write(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len);

static int tfa9887_check_chip_id(struct i2c_client *client);

static void tfa9887_i2c_delay(tfa9887_S32 msec)
{
	mdelay(msec);
}

static int tfa9887_check_chip_id(struct i2c_client *client)
{
       int err = 0;
       int rt = 0;
       u8 chip_id[64];

	rt = tfa9887_i2c_read(client, TFA9887_CHIP_ID_ADDR, chip_id, 2);
       if(rt < 0)
           APS_ERR("tfa9887_i2c_read fail !!");
        else
	    APS_ERR("read chip id result: %#x\n", chip_id[1]);

	if ((chip_id[1] & 0xff) != SENSOR_CHIP_ID_tfa9887)
		err = -1;

	return err;
}

/*	i2c read routine for API*/
static char tfa9887_i2c_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
       u8 *vData;
       //int i;
       
	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = &reg_addr,
		},

		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = len,
		 .buf = data,
		 },
	};

       vData = msg[1].buf;

       if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
       {   
            //for(i = 0; i < len; i++)
                //APS_ERR("tfa9887_i2c_read: Data[%d] of Reg(0x%x) = 0x%x \n", i , reg_addr, *(vData + i ));
            return 0;
       }
       else
       {
            APS_ERR("Read data fail for reg 0x%x !! \n", reg_addr);
            return -1;
       }
}

/*	i2c write routine for */
static char tfa9887_i2c_write(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
	u8 buffer[len+1];
       int i = 0;

        struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = len + 1,
		 .buf = buffer,
		 },
	};

        buffer[0] = reg_addr;
        
	 for (i = 0; i < len; i++){
		buffer[i+1] = *(data + i);
              //APS_ERR("tfa9887_i2c_write: buffer[%d] = 0x%x \n", i+1, buffer[i+1] );
	};

        if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0) {
            //APS_ERR("tfa9887_i2c_write: I2C xfer success, reg_addr = 0x%x, len =%d\n", reg_addr, len);
        } else {
            APS_ERR("tfa9887_i2c_write: I2C xfer error, len =%d\n",len);
        }

       //APS_ERR("tfa9887_i2c_write X : \n");  
	return 0;

}

static char tfa9887_i2c_read_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	char err = 0;
	err = tfa9887_i2c_read(tfa9887_client, reg_addr, data, len);
	return err;
}

static char tfa9887_i2c_write_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	char err = 0;
	err = tfa9887_i2c_write(tfa9887_client, reg_addr, data, len);
	return err;
}

int tfa9887_set_power(int Mode)
{
    int rc = 0;
    //struct tfa9887_client_data *client_data = i2c_get_clientdata(tfa9887_client);

    return rc;
}

static ssize_t hal_i2c_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = 0;
    struct input_dev *input = to_input_dev(dev);
    struct tfa9887_client_data *client_data = input_get_drvdata(input);
    char *read_data;
    int i =0;

    read_data = kmalloc(read_size, GFP_KERNEL);

    ret = tfa9887_i2c_read(client_data->client, ReadReg, read_data, read_size); //read length be (read_size - 1)
    if(ret == 0){
        //APS_DBG("Reg(0x%x) read success \n", ReadReg);
    }
    else
        APS_ERR("Reg(0x%x) read fail \n", ReadReg);

    for (i = 0; i < read_size; i++) {
        sprintf(buf + i * 3, "%03d", *(read_data + i));
    }
    //APS_ERR("hal_i2c_read: ReadReg =ox%x, read_size = %d, buf = %s \n", ReadReg, read_size, buf);

    if(PreReadReg == ReadReg){
        repeat++;
    }
    else{
        if(repeat > 3)
        APS_DBG("Read Reg(0x%x) %d times repeatly ! \n", PreReadReg, repeat);
        repeat = 0;
    }

    PreReadReg = ReadReg;
    
    return read_size *3;
}

static ssize_t hal_i2c_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret = 0;
    struct input_dev *input = to_input_dev(dev);
    struct tfa9887_client_data *client_data = input_get_drvdata(input);
    int i = 0;
    u8 Reg[count];

    for(i = 0; i < count; i++){
        
        Reg[i] = *(buf + i);
        //APS_ERR("hal_i2c_write: Reg[%d] = 0x%x \n", i, Reg[i]);
    }
    ReadReg = Reg[0];

    if(count == 2)
        read_size = *(buf + 1);
    
    //APS_ERR("hal_i2c_write: count = %d, Reg = 0x%x,read_size = %d\n",count, Reg[0], read_size);

    if(count >= 3 )
        ret = tfa9887_i2c_write(client_data->client, Reg[0], &Reg[1], count - 1);
    
    return ret;
}

static DEVICE_ATTR(tfa9887_i2c_op, 0666, hal_i2c_read, hal_i2c_write);

static int tfa9887_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err = 0;
    struct tfa9887_client_data *client_data = NULL;

    APS_ERR("function entrance");

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        APS_ERR("i2c_check_functionality error!");
        err = -EIO;
        goto exit_err_clean;
    }

    if (NULL == tfa9887_client) {
        tfa9887_client = client;
    } else {
        APS_ERR("this driver does not support multiple clients");
        err = -EINVAL;
        goto exit_err_clean;
    }

    client_data = kzalloc(sizeof(struct tfa9887_client_data), GFP_KERNEL);
    if (NULL == client_data) {
        APS_ERR("no memory available");
        err = -ENOMEM;
        goto exit_err_clean;
    }

    i2c_set_clientdata(client, client_data);
    client_data->client = client;
    
    /* check chip id */
    err = tfa9887_check_chip_id(client);
    if (!err) {
        APS_ERR("NXP smart AMP %s detected\n", SENSOR_NAME);
    } else {
        APS_ERR("NXP smart AMP not found, chip id mismatch\n");
        err = -1;
        goto exit_err_clean;
    }

    /* input device init */

    /* sysfs node creation */
    //err = sysfs_create_group(&client_data->input->dev.kobj, &tfa9887_attribute_group);

    err = device_create_file(&client->dev, &dev_attr_tfa9887_i2c_op);

    /* h/w init */
    client_data->device.bus_read = tfa9887_i2c_read_wrapper;
    client_data->device.bus_write = tfa9887_i2c_write_wrapper;
    client_data->device.delay_msec = tfa9887_i2c_delay;
    //tfa9887_CALL_API(init)(&client_data->device);

    //tfa9887_dump_reg(client);

    APS_ERR("sensor %s probed successfully\n", SENSOR_NAME);

    /*APS_ERR("i2c_client: %p client_data: %p i2c_device: %p input: %p\n",
    		client, client_data, &client->dev, client_data->input);*/

    return 0;

exit_err_clean:
    if (err) {
        if (client_data != NULL) {
            kfree(client_data);
            client_data = NULL;
        }

        tfa9887_client = NULL;
    }

    return err;
}


static int tfa9887_remove(struct i2c_client *client)
{
	int err = 0;
	//u8 op_mode;

	struct tfa9887_client_data *client_data =
		(struct tfa9887_client_data *)i2c_get_clientdata(client);

	if (NULL != client_data) {
#if 0 //CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&client_data->early_suspend_handler);
#endif
		kfree(client_data);

		tfa9887_client = NULL;
	}

	return err;
}

static const struct i2c_device_id tfa9887_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tfa9887_id);

static struct of_device_id tfa9887_match_table[] = {
	{ .compatible = "nxp,tfa9887",},
	{ },
};

static struct i2c_driver tfa9887_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SENSOR_NAME,
		.of_match_table = tfa9887_match_table,
	},
	.class = I2C_CLASS_HWMON,
	.id_table = tfa9887_id,
	.probe = tfa9887_probe,
	.remove = tfa9887_remove,
};

static int __init tfa9887_init(void)
{
       unsigned band_id = 0;
       band_id = fih_get_band_id();
       APS_ERR("tfa9887_init: band_id =%d\n", band_id);
       
       if(band_id == BAND_VIV)
            return i2c_add_driver(&tfa9887_driver);
       else 
            return 0;
}

static void __exit tfa9887_exit(void)
{
	i2c_del_driver(&tfa9887_driver);
}

MODULE_DESCRIPTION("driver for " SENSOR_NAME);
MODULE_LICENSE("GPL");

module_init(tfa9887_init);
module_exit(tfa9887_exit);

