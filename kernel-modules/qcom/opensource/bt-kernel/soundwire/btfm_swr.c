// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */


#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/ratelimit.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include "btpower.h"
#include "btfm_swr.h"
#include "btfm_swr_hw_interface.h"
#include "btfm_swr_slave.h"

struct class *btfm_swr_class;
static int btfm_swr_major;
struct btfmswr *pbtfmswr;
static int btfm_num_ports_open;

#define BT_CMD_SWR_TEST	0xbfac

static int btfm_swr_probe(struct swr_device *pdev);

int btfm_get_bt_soc_index(int chipset_ver)
{
	switch (chipset_ver) {
	case QCA_GANGES_SOC_ID_0100:
	case QCA_GANGES_SOC_ID_0200:
		return GANGES;
	case QCA_EVROS_SOC_ID_0100:
	case QCA_EVROS_SOC_ID_0200:
		return EVROS;
	default:
		BTFMSWR_ERR("no BT SOC id defined, returning EVROS");
		return EVROS;
	}
}

int btfm_swr_hw_init(void)
{
	uint8_t dev_num = 0;
	int ret = 0;
	int chipset_ver;

	BTFMSWR_DBG("");

	if (pbtfmswr->initialized)
		BTFMSWR_INFO("Already initialized");

	// get BT chipset version
	chipset_ver = btpower_get_chipset_version();

	// get BT/FM SOC slave port details
	pbtfmswr->soc_index = btfm_get_bt_soc_index(chipset_ver);

	BTFMSWR_INFO("chipset soc version:%x, soc index: %x", chipset_ver,
							pbtfmswr->soc_index);

	pbtfmswr->p_dai_port = &slave_port[pbtfmswr->soc_index];

	// get logical address
	/*
	 * Add 5msec delay to provide sufficient time for
	 * soundwire auto enumeration of slave devices as
	 * per HW requirement.
	 */
	usleep_range(5000, 5010);
	ret = swr_get_logical_dev_num(pbtfmswr->swr_slave, pbtfmswr->p_dai_port->ea,
									&dev_num);
	if (ret) {
		BTFMSWR_ERR("error while getting logical device number");
		goto err;
	}

	pbtfmswr->swr_slave->dev_num = dev_num;
	pbtfmswr->initialized = true;

err:
	return ret;
}

int btfm_swr_enable_port(u8 port_num, u8 ch_count, u32 sample_rate, u8 usecase)
{
	int ret = 0;
	u8 port_id[MAX_BT_PORTS];
	u8 num_ch[MAX_BT_PORTS];
	u8 ch_mask[MAX_BT_PORTS];
	u32 ch_rate[MAX_BT_PORTS];
	u8 port_type[MAX_BT_PORTS];
	u8 num_port = 1;

	// master expects port num -1 to be sent
	port_id[0] = port_num-1;
	num_ch[0] = ch_count;
	ch_mask[0] = ch_count == 2 ? TWO_CHANNEL_MASK :	ONE_CHANNEL_MASK;
	ch_rate[0] = sample_rate;
	port_type[0] = usecase;

	BTFMSWR_INFO("enabling port : %d\n", port_num);
	ret = swr_connect_port(pbtfmswr->swr_slave, &port_id[0], num_port,
							&ch_mask[0], &ch_rate[0], &num_ch[0],
							&port_type[0]);

	if (ret < 0) {
		BTFMSWR_ERR("swr_connect_port failed, error %d", ret);
		return ret;
	}

	BTFMSWR_INFO("calling swr_slvdev_datapath_control\n");
	ret = swr_slvdev_datapath_control(pbtfmswr->swr_slave,
							pbtfmswr->swr_slave->dev_num,
							true);
	if (ret < 0)
		BTFMSWR_ERR("swr_slvdev_datapath_control failed");

	if (ret == 0)
		btfm_num_ports_open++;

	BTFMSWR_INFO("btfm_num_ports_open: %d", btfm_num_ports_open);

	return ret;
}

int btfm_swr_disable_port(u8 port_num, u8 ch_count, u8 usecase)
{
	int ret = 0;
	u8 port_id[MAX_BT_PORTS];
	u8 ch_mask[MAX_BT_PORTS];
	u8 port_type[MAX_BT_PORTS];
	u8 num_port = 1;

	// master expects port num -1 to be sent
	port_id[0] = port_num-1;
	ch_mask[0] = ch_count == 2 ? TWO_CHANNEL_MASK :	ONE_CHANNEL_MASK;
	port_type[0] = usecase;

	BTFMSWR_INFO("disabling port : %d\n", port_num);
	ret = swr_disconnect_port(pbtfmswr->swr_slave, &port_id[0], num_port,
							&ch_mask[0], &port_type[0]);

	if (ret < 0)
		BTFMSWR_ERR("swr_disconnect_port port failed, error %d", ret);

	BTFMSWR_INFO("calling swr_slvdev_datapath_control\n");
	ret = swr_slvdev_datapath_control(pbtfmswr->swr_slave,
							pbtfmswr->swr_slave->dev_num,
							false);
	if (ret < 0)
		BTFMSWR_ERR("swr_slvdev_datapath_control failed");

	if (btfm_num_ports_open > 0)
		btfm_num_ports_open--;

	BTFMSWR_INFO("btfm_num_ports_open: %d", btfm_num_ports_open);

	return ret;
}

static long btfm_swr_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	BTFMSWR_INFO("");
	switch (cmd) {
	case BT_CMD_SWR_TEST:
		BTFMSWR_INFO("cmd BT_CMD_SLIM_TEST, call btfm_swr_hw_init");
		ret = btfm_swr_hw_init();
		break;
	}
	return ret;
}

static const struct file_operations bt_dev_fops = {
	.unlocked_ioctl = btfm_swr_ioctl,
	.compat_ioctl = btfm_swr_ioctl,
};

static int btfm_swr_probe(struct swr_device *pdev)
{
	int ret = 0;

	BTFMSWR_INFO("");

	pbtfmswr = devm_kzalloc(&pdev->dev,
					sizeof(struct btfmswr), GFP_KERNEL);
	if (!pbtfmswr) {
		BTFMSWR_ERR("memory allocation to driver failed");
		return -ENOMEM;
	}

	swr_set_dev_data(pdev, pbtfmswr);
	pbtfmswr->swr_slave = pdev;
	pbtfmswr->dev = &pdev->dev;
	pbtfmswr->initialized = false;

	// register with ALSA
	ret = btfm_swr_register_hw_ep(pbtfmswr);
	if (ret) {
		BTFMSWR_ERR("registration with ALSA failed, returning");
		goto dealloc;
	}

	btfm_swr_major = register_chrdev(0, "btfm_swr", &bt_dev_fops);
	if (btfm_swr_major < 0) {
		BTFMSWR_ERR("%s: failed to allocate char dev\n", __func__);
		ret = -1;
		goto register_err;
	}

	btfm_swr_class = class_create(THIS_MODULE, "btfmswr-dev");
	if (IS_ERR(btfm_swr_class)) {
		BTFMSWR_ERR("%s: coudn't create class\n", __func__);
		ret = -1;
		goto class_err;
	}

	if (device_create(btfm_swr_class, NULL, MKDEV(btfm_swr_major, 0),
		NULL, "btfmswr") == NULL) {
		BTFMSWR_ERR("%s: failed to allocate char dev\n", __func__);
		ret = -1;
		goto device_err;
	}
	return ret;

device_err:
	class_destroy(btfm_swr_class);
class_err:
	unregister_chrdev(btfm_swr_major, "btfm_swr");

register_err:
	btfm_swr_unregister_hwep();
dealloc:
	kfree(pbtfmswr);
	return ret;
}

static const struct swr_device_id btfm_swr_id[] = {
	{SWR_SLAVE_COMPATIBLE_STR, 0},
	{}
};

static const struct of_device_id btfm_swr_dt_match[] = {
	{
		.compatible = "qcom,btfmswr_slave",
	},
	{}
};

static struct swr_driver btfm_swr_driver = {
	.driver = {
		.name = "btfmswr-driver",
		.owner = THIS_MODULE,
		.of_match_table = btfm_swr_dt_match,
	},
	.probe = btfm_swr_probe,
	.id_table = btfm_swr_id,
};

static int __init btfm_swr_init(void)
{
	BTFMSWR_INFO("");
	return swr_driver_register(&btfm_swr_driver);
}

static void __exit btfm_swr_exit(void)
{
	BTFMSWR_INFO("");
	swr_driver_unregister(&btfm_swr_driver);
}

module_init(btfm_swr_init);
module_exit(btfm_swr_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("BTFM SoundWire Slave driver");

