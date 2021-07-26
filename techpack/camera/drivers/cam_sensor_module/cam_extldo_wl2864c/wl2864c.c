/*
 * Copyright (C) 2015 HUAQIN Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

#include "wl2864c.h"

/*****************************************************************************
 * GLobal Variable
 *****************************************************************************/

static struct i2c_client *wl2864c_i2c_client;
static struct pinctrl *wl2864c_pctrl; /* static pinctrl instance */
static unsigned short wl2864c_en_refcnt[WL2864C_MAX];
static DEFINE_MUTEX(ldo_en_lock);

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int wl2864c_dts_probe(struct platform_device *pdev);
static int wl2864c_dts_remove(struct platform_device *pdev);
static int wl2864c_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int wl2864c_i2c_remove(struct i2c_client *client);

/*****************************************************************************
 * Extern Area
 *****************************************************************************/
void wl2864c_set_en_ldo(WL2864C_SELECT ldonum,unsigned int en)
{
	s32 ret=0;
	unsigned int value =0;

	 if (NULL == wl2864c_i2c_client) {
	        WL2864C_PRINT("[wl2864c] wl2864c_i2c_client is null!!\n");
	        return ;
        }
	ret= i2c_smbus_read_byte_data(wl2864c_i2c_client, WL2864C_LDO_EN_ADDR);

	if(ret <0)
	{
		WL2864C_PRINT("[wl2864c] wl2864c_set_en_ldo read error!\n");
		return;
	}

	if(en == 0)
	{
		value = ret & (~(0x01<<ldonum));
	}
	else
	{
		value = ret|(0x01<<ldonum);
	}

	i2c_smbus_write_byte_data(wl2864c_i2c_client,WL2864C_LDO_EN_ADDR,value);
	WL2864C_PRINT("[wl2864c] wl2864c_set_en_ldo enable before:%x after set :%x  \n",ret,value);
       return;

}


//Voutx=0.6v+LDOX_OUT[7:0]*0.0125V   LDO1/LDO2
//Voutx=1.2v+LDOX_OUT[7:0]*0.0125V   LDO3~LDO7
void wl2864c_set_ldo_value(WL2864C_SELECT ldonum,unsigned int value)
{
	unsigned int  Ldo_out =0;
       unsigned char regaddr =0;
       s32 ret =0;
	WL2864C_PRINT("[wl2864c] %s enter!!!\n",__FUNCTION__);

	 if (NULL == wl2864c_i2c_client) {
	        WL2864C_PRINT("[wl2864c] wl2864c_i2c_client is null!!\n");
	        return ;
        }
	if(ldonum >= WL2864C_MAX)
	{
		WL2864C_PRINT("[wl2864c] error ldonum not support!!!\n");
		return;
	}

	switch(ldonum)
	{
		case WL2864C_DVDD1:
		case WL2864C_DVDD2:
			if(value<600)
			{
				WL2864C_PRINT("[wl2864c] error vol!!!\n");
				goto exit;
			}
			else
			{
			 	Ldo_out = (value-600)*80/1000;
			}
	       break;
		case WL2864C_AVDD1:
		case WL2864C_AVDD2:
		case WL2864C_VDDAF:
		case WL2864C_VDDOIS:
		case WL2864C_VDDIO:
			if(value<1200)
			{
				WL2864C_PRINT("[wl2864c] error vol!!!\n");
				goto exit;

			}
			else
			{
				Ldo_out = (value-1200)*80/1000;
			}
			break;
		default:
			goto exit;
			break;
	}
	regaddr = ldonum+WL2864C_LDO1_OUT_ADDR;

	WL2864C_PRINT("[wl2864c] ldo=%d,value=%d,Ldo_out:%d,regaddr=0x%x \n",ldonum,value,Ldo_out,regaddr);
	i2c_smbus_write_byte_data(wl2864c_i2c_client,regaddr,Ldo_out);
	ret=i2c_smbus_read_byte_data(wl2864c_i2c_client,regaddr);
	WL2864C_PRINT("[wl2864c] after write ret=0x%x\n",ret);

exit:
	WL2864C_PRINT("[wl2864c] %s exit!!!\n",__FUNCTION__);

}

static struct wl2864c_ldomap ldolist[] = {
	{CAMERA_INDEX_MAIN, SENSOR_WL2864C_VANA, WL2864C_AVDD2}, //BackMain AVDD
	{CAMERA_INDEX_MAIN, SENSOR_WL2864C_VDIG, WL2864C_DVDD2}, //BackMain DVDD
	{CAMERA_INDEX_MAIN, SENSOR_WL2864C_VIO, WL2864C_VDDIO}, //BackMain IOVDD
	{CAMERA_INDEX_MAIN, SENSOR_WL2864C_VAF, WL2864C_VDDAF}, //BackMain AFVDD

	{CAMERA_INDEX_FRONT, SENSOR_WL2864C_VANA, WL2864C_AVDD1}, //FrontMain AVDD
	{CAMERA_INDEX_FRONT, SENSOR_WL2864C_VDIG, WL2864C_DVDD1}, //FrontMain DVDD
	{CAMERA_INDEX_FRONT, SENSOR_WL2864C_VIO, WL2864C_VDDIO}, //FrontMain IOVDD

	{CAMERA_INDEX_MAIN_WIDE, SENSOR_WL2864C_VIO, WL2864C_VDDIO}, //BackWide IOVDD
	{CAMERA_INDEX_MAIN_WIDE, SENSOR_WL2864C_VANA, WL2864C_AVDD1}, //BackWide AVDD
	{CAMERA_INDEX_MAIN_WIDE, SENSOR_WL2864C_VDIG, WL2864C_DVDD1}, //BackWide DVDD

	{CAMERA_INDEX_MAIN_TELE, SENSOR_WL2864C_VIO, WL2864C_VDDIO}, //BackTele IOVDD
	{CAMERA_INDEX_MAIN_TELE, SENSOR_WL2864C_VAF, WL2864C_VDDAF}, //BackTele AFVDD
	{CAMERA_INDEX_MAIN_TELE, SENSOR_WL2864C_VANA, WL2864C_VDDOIS}, //BackTele AVDD
	{CAMERA_INDEX_MAIN_TELE, SENSOR_WL2864C_VDIG, WL2864C_DVDD1}, //BackTele DVDD
};

int wl2864c_set_ldo_enable(uint32_t sensor_index, enum msm_camera_power_seq_type seq_type, uint32_t min_voltage, uint32_t max_voltage)
{
	WL2864C_SELECT ldonum = WL2864C_NONE;
	unsigned int ldo_vol_value = 0;
	unsigned int i = 0;

	if(sensor_index >= CAMERA_INDEX_MAX_NUM ||
		seq_type >= SENSOR_SEQ_TYPE_MAX ||
		min_voltage < 600000 ||
		max_voltage < 600000 ||
		min_voltage > max_voltage){
		WL2864C_PRINT("[wl2864c] %s invalid parameters!!!\n",__FUNCTION__);
		return -1;
	}

	for(i;i < (sizeof(ldolist) / sizeof(ldolist[0]));i++) {
		if(sensor_index == ldolist[i].sensor_index && seq_type == ldolist[i].seq_type) {
			ldonum = ldolist[i].ldo_selected;
			break;
		}
	}

	if(ldonum == WL2864C_NONE) {
		WL2864C_PRINT("[wl2864c] %s ldo setting not found in ldolist!!!\n",__FUNCTION__);
		return -2;
	}

	ldo_vol_value = min_voltage / 1000;

	mutex_lock(&ldo_en_lock);
	if(!wl2864c_en_refcnt[ldonum]) {
		wl2864c_set_ldo_value(ldonum, ldo_vol_value);
		wl2864c_set_en_ldo(ldonum, 1);
	}
	wl2864c_en_refcnt[ldonum]++;
	mutex_unlock(&ldo_en_lock);
	WL2864C_PRINT("[wl2864c] ldo[%d] refcnt=%d  \n",ldonum,wl2864c_en_refcnt[ldonum]);

	return 0;
}

int wl2864c_set_ldo_disable(uint32_t sensor_index, enum msm_camera_power_seq_type seq_type)
{
	WL2864C_SELECT ldonum = WL2864C_NONE;
	unsigned int i = 0;

	if(sensor_index >= CAMERA_INDEX_MAX_NUM ||
					seq_type >= SENSOR_SEQ_TYPE_MAX) {
		WL2864C_PRINT("[wl2864c] %s invalid parameters!!!\n",__FUNCTION__);
		return -1;
	}

	for(i;i < (sizeof(ldolist) / sizeof(ldolist[0]));i++) {
		if(sensor_index == ldolist[i].sensor_index && seq_type == ldolist[i].seq_type) {
			ldonum = ldolist[i].ldo_selected;
			break;
		}
	}

	if(ldonum == WL2864C_NONE) {
		WL2864C_PRINT("[wl2864c] %s ldo setting not found in ldolist!!!\n",__FUNCTION__);
		return -2;
	}

	mutex_lock(&ldo_en_lock);
	wl2864c_en_refcnt[ldonum]--;
	if(!wl2864c_en_refcnt[ldonum]) {
		wl2864c_set_en_ldo(ldonum, 0);
	}
	mutex_unlock(&ldo_en_lock);
	WL2864C_PRINT("[wl2864c] ldo[%d] refcnt=%d  \n",ldonum,wl2864c_en_refcnt[ldonum]);

	return 0;
}

/*****************************************************************************
 * Data Structure
 *****************************************************************************/
static const char *wl2864c_state_name[WL2864C_GPIO_STATE_MAX] = {
    "wl2864c_gpio_enp0",
    "wl2864c_gpio_enp1"
};/* DTS state mapping name */

static const struct of_device_id gpio_of_match[] = {
    { .compatible = "qualcomm,gpio_wl2864c", },
    {},
};

static const struct of_device_id i2c_of_match[] = {
    { .compatible = "qualcomm,i2c_wl2864c", },
    {},
};

static const struct i2c_device_id wl2864c_i2c_id[] = {
    {"WL2864C_I2C", 0},
    {},
};

static struct platform_driver wl2864c_platform_driver = {
    .probe = wl2864c_dts_probe,
    .remove = wl2864c_dts_remove,
    .driver = {
        .name = "WL2864C_DTS",
        .of_match_table = gpio_of_match,
    },
};

static struct i2c_driver wl2864c_i2c_driver = {
/************************************************************
Attention:
Althouh i2c_bus do not use .id_table to match, but it must be defined,
otherwise the probe function will not be executed!
************************************************************/
    .id_table = wl2864c_i2c_id,
    .probe = wl2864c_i2c_probe,
    .remove = wl2864c_i2c_remove,
    .driver = {
        .name = "WL2864C_I2C",
        .of_match_table = i2c_of_match,
    },
};

/*****************************************************************************
 * Function
 *****************************************************************************/
static long wl2864c_set_state(const char *name)
{
    int ret = 0;
    struct pinctrl_state *pState = 0;

    BUG_ON(!wl2864c_pctrl);

    pState = pinctrl_lookup_state(wl2864c_pctrl, name);
    if (IS_ERR(pState)) {
        pr_err("set state '%s' failed\n", name);
        ret = PTR_ERR(pState);
        goto exit;
    }

    /* select state! */
    pinctrl_select_state(wl2864c_pctrl, pState);

exit:
    return ret; /* Good! */
}

void wl2864c_gpio_select_state(WL2864C_GPIO_STATE s)
{
    WL2864C_PRINT("[wl2864c]%s,%d\n",__FUNCTION__,s);

    BUG_ON(!((unsigned int)(s) < (unsigned int)(WL2864C_GPIO_STATE_MAX)));
    wl2864c_set_state(wl2864c_state_name[s]);
}

static long wl2864c_dts_init(struct platform_device *pdev)
{
    int ret = 0;
    struct pinctrl *pctrl;

    /* retrieve */
    pctrl = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR(pctrl)) {
        dev_err(&pdev->dev, "Cannot find disp pinctrl!");
        ret = PTR_ERR(pctrl);
        goto exit;
    }

    wl2864c_pctrl = pctrl;

exit:
    return ret;
}

static int wl2864c_dts_probe(struct platform_device *pdev)
{
    int ret = 0;

    ret = wl2864c_dts_init(pdev);
    if (ret) {
        WL2864C_PRINT("[wl2864c]wl2864c_dts_probe failed\n");
        return ret;
    }

    WL2864C_PRINT("[wl2864c] wl2864c_dts_probe success\n");

    return 0;
}

static int wl2864c_dts_remove(struct platform_device *pdev)
{
    platform_driver_unregister(&wl2864c_platform_driver);

    return 0;
}

static int wl2864c_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    if (NULL == client) {
        WL2864C_PRINT("[wl2864c] i2c_client is NULL\n");
        return -1;
    }
    wl2864c_i2c_client = client;
    wl2864c_gpio_select_state(WL2864C_GPIO_STATE_ENP0);

    WL2864C_PRINT("[wl2864c]wl2864c_i2c_probe success addr = 0x%x\n", client->addr);
    return 0;
}

static int wl2864c_i2c_remove(struct i2c_client *client)
{
    wl2864c_i2c_client = NULL;
    i2c_unregister_device(client);

    return 0;
}

static int __init wl2864c_init(void)
{
		if (platform_driver_register(&wl2864c_platform_driver)) {
				WL2864C_PRINT("[wl2864c]Failed to register wl2864c_platform_driver!\n");
				i2c_del_driver(&wl2864c_i2c_driver);
				return -1;
		}

    if (i2c_add_driver(&wl2864c_i2c_driver)) {
        WL2864C_PRINT("[wl2864c]Failed to register wl2864c_i2c_driver!\n");
        return -1;
    }

    return 0;
}

static void __exit wl2864c_exit(void)
{
    platform_driver_unregister(&wl2864c_platform_driver);
    i2c_del_driver(&wl2864c_i2c_driver);
}

module_init(wl2864c_init);
module_exit(wl2864c_exit);

MODULE_AUTHOR("AmyGuo <guohuiqing@huaqin.com>");
MODULE_DESCRIPTION("EXT CAMERA LDO Driver");
MODULE_LICENSE("GPL");
