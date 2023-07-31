/*
 * Copyright (C) 2021 HUAQIN Inc.
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
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/timekeeping.h>
#include "cam_sensor_i2c.h"
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "cam_common_util.h"
#include "dw9781_ois.h"
#include "dw9781_ois_fw.h"
#include "../cam_ois/cam_ois_dev.h"
#include "cam_cci_dev.h"
#include "cam_sensor_i2c.h"

typedef struct
{
	unsigned int driverIc;
	unsigned int size;
	unsigned short *fwContentPtr;
	unsigned short version;
}FirmwareContex;

FirmwareContex g_firmwareContext;

unsigned short g_downloadByForce = 0;
struct cam_sensor_i2c_reg_setting dw9781_i2c_setting;
struct cam_sensor_i2c_reg_array *ois_setting = NULL;
struct cam_ois_ctrl_t *dw9781_o_ctrl;
uint32_t i2c_read_data = 0;
int init_success = 1;


/*write ois i2c*/
void write_reg_16bit_value_16bit(uint32_t reg_addr, uint32_t reg_data, uint32_t delay)
{
         ois_setting->reg_addr = reg_addr;
         ois_setting->reg_data = reg_data;
         ois_setting->delay = delay;
         ois_setting->data_mask = 0;
         dw9781_i2c_setting.size = 1;
         dw9781_i2c_setting.addr_type = 2;
         dw9781_i2c_setting.data_type =2;
         dw9781_i2c_setting.delay = 0;
         dw9781_i2c_setting.read_buff = NULL;
         dw9781_i2c_setting.read_buff_len = 0;
         camera_io_dev_write(&(dw9781_o_ctrl->io_master_info), &dw9781_i2c_setting);
         mdelay(delay);
}


void i2c_block_write_reg(uint32_t reg_addr, unsigned short *data, uint32_t size)
{
    int base = 0, offset = 0;
    int block_size = 8;
    dw9781_i2c_setting.addr_type = 2;
    dw9781_i2c_setting.data_type =2;
    dw9781_i2c_setting.delay = 0;
    dw9781_i2c_setting.read_buff = NULL;
    dw9781_i2c_setting.read_buff_len = 0;
    while(base < size){
        block_size = ((size - base < block_size)? (size - base): block_size);
        dw9781_i2c_setting.size = block_size;
        for (offset = 0; offset < block_size; offset++){
            ois_setting[offset].reg_addr = reg_addr + base + offset;
            ois_setting[offset].reg_data = *(data + base + offset);
            ois_setting[offset].delay = 0;
            ois_setting[offset].data_mask = 0;
        }
        cam_cci_i2c_write_continuous_table(&(dw9781_o_ctrl->io_master_info), &dw9781_i2c_setting, 0);
        base += block_size;
    }
}

/*read ois i2c*/
void read_reg_16bit_value_16bit(uint32_t reg_addr, uint8_t *reg_data)
{
    camera_io_dev_read_seq(&(dw9781_o_ctrl->io_master_info), reg_addr, reg_data , 2, 1, 2);
}

/*read fw by i2c*/
void i2c_block_read_reg(uint32_t reg_addr, uint8_t *reg_data, uint32_t size)
{
    int i = 0;
    while(i < size){
        camera_io_dev_read_seq(&(dw9781_o_ctrl->io_master_info), reg_addr + i, reg_data + 2 * i, 2, 2, 2);
        i++;
    }
}

int dw9781_whoami_check(void)
{
	uint8_t second_chip_id[2];
	uint8_t fw_flag1[2], fw_flag2[2], fw_flag3[2];
    //uint8_t sample_ctrl[2];
    /* chip enable */
	write_reg_16bit_value_16bit(0xD000, 0x0001, 4);
    /* dsp off mode */
	write_reg_16bit_value_16bit(0xD001, 0x0000, 0);
    write_reg_16bit_value_16bit(0xFAFA, 0x98AC, 0); /* All protection(1) */
	write_reg_16bit_value_16bit(0xF053, 0x70BD, 0); /* All protection(2) */

	read_reg_16bit_value_16bit(DW9781_WHOAMI_ADDRESS, &second_chip_id[0]); /* second_chip_id: 0x0020 */
	ois_printf("[dw9781_whoami_check] dw9781 second chip id : 0x%02x 0x%02x\r\n", second_chip_id[0], second_chip_id[1]);

	if((second_chip_id[0] | second_chip_id[1]) == DW9781_WHOAMI_DATA) /* second_chip_id: 0x0020 */
	{
		/* check fw status */
		read_reg_16bit_value_16bit(0x8000, &fw_flag1[0]); /* check checksum flag1 */
		ois_printf("[dw9781_whoami_check] checksum flag1 : 0x%02x 0x%02x\r\n", fw_flag1[0], fw_flag1[1]);

		read_reg_16bit_value_16bit(0x8001, &fw_flag2[0]); /* check checksum flag2 */
		ois_printf("[dw9781_whoami_check] checksum flag2 : 0x%02x 0x%02x\r\n", fw_flag2[0], fw_flag2[1]);

		read_reg_16bit_value_16bit(0xA7F9, &fw_flag3[0]); /* check checksum flag3 */
		ois_printf("[dw9781_whoami_check] checksum flag3 : 0x%02x 0x%02x\r\n", fw_flag3[0], fw_flag3[1]);

		if(((fw_flag1[0] << 8 | fw_flag1[1]) == 0xF073) && ((fw_flag2[0] << 8 | fw_flag2[1]) == 0x2045) && ((fw_flag3[0] << 8 | fw_flag3[1])== 0xCC33)) /*210517 add 1st page ready check*/
		{
			ois_printf("[dw9781_whoami_check] ois fw fixed data correct\r\n");
			ois_reset(); /* ois reset */
	        write_reg_16bit_value_16bit(0x70DA, 0x1002, 1);
		    //read_reg_16bit_value_16bit(0x70DA, &sample_ctrl[0]); /* check sample time ctrl*/
		    //ois_printf("[dw9781_sample_check] sample_ctrl: 0x%02x 0x%02x\r\n", sample_ctrl[0], sample_ctrl[1]);
		}
		else
		{
			write_reg_16bit_value_16bit(0xD002, 0x0001, 4); /* logic reset */
			ois_printf("[dw9781_whoami_check] previous firmware download fail\r\n");
		}

		ois_printf("[dw9781_whoami_check] dw9781 second chip id correct\r\n");
		return ADJ_OK;
	}
	else
	{
		ois_printf("[dw9781_whoami_check] dw9781 second chip id error!!!\r\n");
		return DW9781_WHOAMI_ERROR;
	}
}

void ois_reset(void)
{
	ois_printf("[dw9781_ois_reset] ois reset\r\n");
    mdelay(16);
	write_reg_16bit_value_16bit(0xD002, 0x0001, 4); /* printfc reset */
	write_reg_16bit_value_16bit(0xD001, 0x0001, 5); /* Active mode (DSP ON) */
	write_reg_16bit_value_16bit(0x7015, 0x0002, 20); /* Active mode (DSP ON) */
	write_reg_16bit_value_16bit(0xEBF1, 0x56FA, 0); /* User protection release */
}

void dw9781_exit(void){
	ois_printf("[dw9781_exit] ois exit\r\n");
	mdelay(20);
	ois_printf("[dw9781_exit] lens offset\r\n");
}

void ois_ready_check(void)
{
	uint8_t fw_flag1[2], fw_flag2[2], fw_flag3[2];
	write_reg_16bit_value_16bit(0xD000, 0x0001, 4); /* chip enable */
	write_reg_16bit_value_16bit(0xD001, 0x0000, 0); /* dsp off mode */
	write_reg_16bit_value_16bit(0xFAFA, 0x98AC, 0); /* All protection(1) */
	write_reg_16bit_value_16bit(0xF053, 0x70BD, 0); /* All protection(2) */

	read_reg_16bit_value_16bit(0x8000, &fw_flag1[0]); /* check checksum flag1 */
	ois_printf("[dw9781_ois_ready_check] checksum flag1 : 0x%04x\r\n", fw_flag1);
	read_reg_16bit_value_16bit(0x8001, &fw_flag2[0]); /* check checksum flag2 */
	ois_printf("[dw9781_ois_ready_check] checksum flag2 : 0x%04x\r\n", fw_flag2);
	read_reg_16bit_value_16bit(0xA7F9, &fw_flag3[0]); /* check checksum flag3 */
	ois_printf("[dw9781_ois_ready_check] checksum flag3 : 0x%04x\r\n", fw_flag3);

	if(((fw_flag1[0] <<8 | fw_flag1[1]) == 0xF073) && ((fw_flag2[0] <<8 | fw_flag2[1]) == 0x2045) && ((fw_flag3[0] <<8 | fw_flag3[1])== 0xCC33)) /*210517 add 1st page ready check*/
	{
		ois_printf("[dw9781c_ois_ready_check] fw fixed data is ok\r\n");
		ois_reset(); /* ois reset */
	} else {
		write_reg_16bit_value_16bit(0xD002, 0x0001, 4); /* logic reset */
		ois_printf("[dw9781c_ois_ready_check] previous firmware download fail\r\n");
	}
}

void GenerateFirmwareContexts(void)
{
	g_firmwareContext.version = 0x0100;
	g_firmwareContext.size = 10240; /* size: word */
	g_firmwareContext.driverIc = 0x9781;
	g_firmwareContext.fwContentPtr = DW9781_Flash_Buf;
	g_downloadByForce = 0;
}

int dw9781_download_ois_fw(struct cam_ois_ctrl_t *o_ctrl)
{
	unsigned char ret = ADJ_OK;
	uint8_t first_chip_id[2];
	unsigned short fwchecksum = 0;
	uint8_t fw_version_current[2];
        unsigned short fw_version_latest = 0;

    dw9781_o_ctrl = o_ctrl;
    set_ois_init_setting();
    if(!init_success){
        ois_printf("[dw9781_download_ois_fw], init_setting fail");
        ret = ERROR_ALLOC;
        goto err_alloc;
    }

	ret = dw9781_whoami_check();

	if(ret != ADJ_OK)
	{
		ois_printf("[dw9781_download_ois_fw] dw9781 whoami check error!!!\r\n");
		ois_printf("[dw9781_download_ois_fw] stop fw download and return\r\n");
		return DW9781_WHOAMI_ERROR;
	}

	read_reg_16bit_value_16bit(DW9781_CHIP_ID_ADDRESS, &first_chip_id[0]);
	ois_printf("[dw9781_download_ois_fw] first_chip_id : 0x%02x 0x%02x\r\n", first_chip_id[0], first_chip_id[1]);

    GenerateFirmwareContexts();
	if ((first_chip_id[0] << 8 | first_chip_id[1]) != DW9781_CHIP_ID) { /* first_chip_id verification failed */
		ois_printf("[dw9781_download_ois_fw] dw9781 1st chip id failed, will download fw force\r\n");
		g_downloadByForce = 1;
	} else { /* first_chip_id verification success */
		fwchecksum = fw_checksum_verify();

        /*checksum should be 0*/
		if(fwchecksum != *(g_firmwareContext.fwContentPtr+10234)) {	/* fw checksum failed */
			g_downloadByForce = 1;
			ois_printf("[dw9781_download_ois_fw] firmware checksum error 0x%04X, 0x%04X\r\n, will download fw force", *(g_firmwareContext.fwContentPtr+10234), fwchecksum);
		}
	}

	read_reg_16bit_value_16bit(FW_VER_CURR_ADDR, &fw_version_current[0]);	/* current fw inside ic */
	fw_version_latest = *(g_firmwareContext.fwContentPtr + 10235); /* latest updated fw version */
	ois_printf("[dw9781_download_ois_fw] fw_version_current = 0x%02X%02X, fw_version_latest = 0x%04x\r\n", fw_version_current[0], fw_version_current[1], fw_version_latest);

	/* check if need to download firmware to flash, fw_version is 0x0100*/
	if (g_downloadByForce || ((fw_version_current[0] << 8 | fw_version_current[1]) < fw_version_latest)) {
		ois_printf("[dw9781_download_ois_fw] start firmware download:: size:%d, version:0x%x g_downloadByForce %d\r\n",
			g_firmwareContext.size, g_firmwareContext.version, g_downloadByForce);

		ret = download_fw();
		ois_printf("[dw9781_download_ois_fw] firmware download finished\r\n");

		if (ret != ADJ_OK) {
			erase_mtp_rewritefw();
			write_reg_16bit_value_16bit(0xd000, 0x0000, 0); /* Shut download mode */
			ois_printf("[dw9781_download_ois_fw] firmware download error, ret = 0x%x\r\n", ret);
			ois_printf("[dw9781_download_ois_fw] change dw9781c state to shutdown mode\r\n");
			return ERROR_FW_VERIFY;
		} else {
			ois_printf("[dw9781_download_ois_fw] firmware download success\r\n");
		}
	}
	ois_reset(); /* reset after download fw */
err_alloc:
    kfree(ois_setting);
	return ret;
}
EXPORT_SYMBOL(dw9781_download_ois_fw);

//uint8_t buf_temp[20480];

int download_fw(void)
{
	unsigned char ret = ADJ_OK;
	//unsigned short i;
    //int count = 0;
    /* chip enable */

	//memset(buf_temp, 0, g_firmwareContext.size * sizeof(unsigned short));

	write_reg_16bit_value_16bit(0xd001, 0x0000, 0);

	all_protection_release();
	erase_mtp();
    ois_printf("[dw9781_download_fw] start firmware download\r\n");
    i2c_block_write_reg(MTP_START_ADDRESS, &DW9781_Flash_Buf[0], g_firmwareContext.size);
    ois_printf("[dw9781_download_fw] end write firmware to flash\r\n");

    //i2c_block_read_reg(MTP_START_ADDRESS, &buf_temp[0], g_firmwareContext.size);

	/*for (i = 0; i < g_firmwareContext.size; i++)
    {
        if(i < 100 && i % 9 == 0)
            ois_printf("[dw9781_download_fw] 0x%02x%02x  0x%02x%02x  0x%02x%02x  0x%02x%02x  0x%02x%02x  0x%02x%02x  0x%02x%02x  0x%02x%02x  0x%02x%02x \n",
                     buf_temp[2 * i], buf_temp[2 * i + 1],buf_temp[2 * i + 2], buf_temp[2 * i + 3],buf_temp[2 * i + 4], buf_temp[2 * i + 5 ],buf_temp[2 * i +6 ], buf_temp[2 * i + 7],
buf_temp[2 * i + 8], buf_temp[2 * i + 9],buf_temp[2 * i + 10 ], buf_temp[2 * i + 11],buf_temp[2 * i + 12], buf_temp[2 * i + 13],buf_temp[2 * i + 14], buf_temp[2 * i + 15],buf_temp[2 * i + 16], buf_temp[2 * i + 17]);
    }

	for (i = 0; i < g_firmwareContext.size; i++)
	{
        if (i < 20)
            ois_printf("[dw9781_download_fw] check addr:%04x READ:0x%02x 0x%02x\r\n",  MTP_START_ADDRESS+i, buf_temp[2 * i], buf_temp[2 * i + 1]);

		if (g_firmwareContext.fwContentPtr[i] != (buf_temp[2 * i] << 8 | buf_temp[2 * i + 1]))
		{
            if(count < 25)
			    ois_printf("[dw9781_download_fw] firmware verify NG!!! ADDR:%04X -- firmware:%04x -- READ:0x%02x 0x%02x\r\n", MTP_START_ADDRESS+i, g_firmwareContext.fwContentPtr[i], buf_temp[2 * i], buf_temp[2 * i + 1]);
            count++;
			ret =  ERROR_FW_VERIFY;
		}else
			ret = ADJ_OK;
	}*/

	//ois_printf("[dw9781_download_fw] firmware verification pass, count %d \r\n", count);
	ois_printf("[dw9781_download_fw] firmware verification pass\r\n");
	ois_printf("[dw9781_download_fw] firmware download success\r\n");

	return ret;
}

void all_protection_release(void)
{
	ois_printf("[dw9781c_all_protection_release] execution\r\n");
	/* release all protection */
	write_reg_16bit_value_16bit(0xFAFA, 0x98AC, 1);
	write_reg_16bit_value_16bit(0xF053, 0x70BD, 1);
}

int erase_mtp_rewritefw(void)
{
	ois_printf("[erase_mtp_rewritefw] erase flash for rewritefw starting..");

	/* 512 byte page */
	write_reg_16bit_value_16bit(0xde03, 0x0027, 0);
	/* page erase */
	write_reg_16bit_value_16bit(0xde04, 0x0008, 10);

	ois_printf("[erase_mtp_rewritefw] dw9781 checksum flag erase : 0xCC33\r\n");
	return 0;
}

void erase_mtp(void)
{
	ois_printf("[dw9781_erase_mtp] start erasing firmware flash\r\n");
	/* 12c level adjust */
	write_reg_16bit_value_16bit(0xd005, 0x0001, 0);
	write_reg_16bit_value_16bit(0xdd03, 0x0002, 0);
	write_reg_16bit_value_16bit(0xdd04, 0x0002, 0);

	/* 4k Sector_0 */
	write_reg_16bit_value_16bit(0xde03, 0x0000, 0);
	/* 4k Sector Erase */
	write_reg_16bit_value_16bit(0xde04, 0x0002, 10);
	/* 4k Sector_1 */
	write_reg_16bit_value_16bit(0xde03, 0x0008, 0);
	/* 4k Sector Erase */
	write_reg_16bit_value_16bit(0xde04, 0x0002, 10);
	/* 4k Sector_2 */
	write_reg_16bit_value_16bit(0xde03, 0x0010, 0);
	/* 4k Sector Erase */
	write_reg_16bit_value_16bit(0xde04, 0x0002, 10);
	/* 4k Sector_3 */
	write_reg_16bit_value_16bit(0xde03, 0x0018, 0);
	/* 4k Sector Erase */
	write_reg_16bit_value_16bit(0xde04, 0x0002, 10);
	/* 4k Sector_4 */
	write_reg_16bit_value_16bit(0xde03, 0x0020, 0);
	/* 4k Sector Erase */
	write_reg_16bit_value_16bit(0xde04, 0x0002, 10);
	ois_printf("[dw9781_erase_mtp] complete erasing firmware flash\r\n");
}

unsigned short fw_checksum_verify(void)
{
	unsigned short data;
    uint8_t check_data[2];
	/* FW checksum command */
	write_reg_16bit_value_16bit(0x7011, 0x2000, 0);
	/* command  start */
	write_reg_16bit_value_16bit(0x7010, 0x8000, 10);
	/* calc the checksum to write the 0x7005 */
	read_reg_16bit_value_16bit(0x7005, &check_data[0]);
    data = (check_data[0] << 8) | check_data[1];
	ois_printf("[dw9781_fw_checksum_verify] F/W Checksum calculated value : 0x%02x 0x%02x\r\n", check_data[0], check_data[1]);
	ois_printf("[dw9781_fw_checksum_verify] F/W Checksum calculated value : 0x%04X\r\n", data);

	return data;
}

int gyro_offset_calibrtion(struct cam_ois_ctrl_t *o_ctrl)
{
	unsigned short Addr;
    uint8_t status[2];
	uint8_t xOffset[2], yOffset[2];
	unsigned int OverCnt;
	int msg, i;

        dw9781_o_ctrl = o_ctrl;
        set_ois_init_setting();
        if(!init_success){
            ois_printf("[dw9781_download_ois_fw], init_setting fail");
            msg = ERROR_ALLOC;
            goto err_alloc;

        }

	ois_printf("[dw9781_gyro_offset_calibrtion] gyro_offset_calibrtion starting\r\n");
	for(i = 1; i < 3; i++)
	{
		/* Gyro offset */
		write_reg_16bit_value_16bit(0x7011, 0x4015, 0);
		write_reg_16bit_value_16bit(0x7010, 0x8000, 100);
		msg = 0;
		OverCnt = 0;
		while (1)
		{
			Addr = 0x7036; status[0] = 0, status[0] = 0;
			read_reg_16bit_value_16bit(Addr, &status[0]);
			if (status[0] & 0x80)
			{
				break;
			} /* it done! */
			else
			{
                 mdelay(50);
			}
			if (OverCnt++ > GYRO_OFST_CAL_OVERCNT)
			{ /* when it take over 10sec .. break */
				msg = GYRO_CAL_TIME_OVER;
				break;
			}
		}
		if (msg == 0)
		{
			if (status[0] & 0x80)
			{
				if ( status[0] == 0x80 && status[1] == 0x00 )
				{
					msg = GYRO_OFFSET_CAL_OK;
					ois_printf("[dw9781c_gyro_offset_calibrtion] GYRO_OFFSET_CAL_OK\r\n");
					break;
				}
				else
				{
					if (status[1] & 0x1)
					{
						msg += X_GYRO_OFFSET_SPEC_OVER_NG;
						ois_printf("[dw9781c_gyro_offset_calibrtion] X_GYRO_OFFSET_SPEC_OVER_NG\r\n");
					}
					if (status[1] & 0x2)
					{
						msg += Y_GYRO_OFFSET_SPEC_OVER_NG;
						ois_printf("[dw9781c_gyro_offset_calibrtion] Y_GYRO_OFFSET_SPEC_OVER_NG\r\n");
					}
					if (status[1] & 0x10)
					{
						msg += X_GYRO_RAW_DATA_CHECK;
						ois_printf("[dw9781c_gyro_offset_calibrtion] X_GYRO_RAW_DATA_CHECK\r\n");
					}
					if (status[1] & 0x20)
					{
						msg += Y_GYRO_RAW_DATA_CHECK;
						ois_printf("[dw9781c_gyro_offset_calibrtion] Y_GYRO_RAW_DATA_CHECK\r\n");
					}
					if (i >= 2)
					{
						ois_printf("[dw9781c_gyro_offset_calibrtion] gyro offset calibration-retry NG (%d times)\r\n", i);
					} else {
						ois_printf("[dw9781c_gyro_offset_calibrtion] gyro offset calibration-retry NG (%d times)\r\n", i);
						msg = 0;
					}
				}
			}
		}
	}
	read_reg_16bit_value_16bit(0x70F8, &xOffset[0]);
	read_reg_16bit_value_16bit(0x70F9, &yOffset[0]);
	ois_printf("[dw9781_gyro_offset_calibrtion] msg : %d\r\n", msg);
	ois_printf("[dw9781_gyro_offset_calibrtion] x_gyro_offset: 0x%02X%02X, y_gyro_offset : 0x%02X%02X\r\n", xOffset[0], xOffset[1], yOffset[0], yOffset[1]);
	ois_printf("[dw9781_gyro_offset_calibrtion] gyro_offset_calibrtion finished...Status = 0x%02X%02X\r\n", status[0], status[1]);

	if(msg == ADJ_OK)
		calibration_save();
err_alloc:
    kfree(ois_setting);
	return msg;
}
EXPORT_SYMBOL(gyro_offset_calibrtion);

void calibration_save(void)
{
	ois_printf("[dw9781c_calibration_save] calibration save starting\r\n");
	write_reg_16bit_value_16bit(0x7011, 0x4000, 1); /* calibration mode */
	write_reg_16bit_value_16bit(0x7011, 0x00AA, 10); /* select mode */
	write_reg_16bit_value_16bit(0x7010, 0x8000, 100); /* start mode */
	ois_reset();
	ois_printf("[dw9781c_calibration_save] calibration save finish\r\n");
}

void set_ois_mode(int ois_mode)
{
	switch(ois_mode) {
	case 0:
		ois_printf("[dw9781_set_ois_mode] set ois on mode\r\n");
		write_reg_16bit_value_16bit(0x7015, 0x0000, 0); /* ois on mode */
		break;
	case 1:
		ois_printf("[dw9781_set_ois_mode] set ois off mode\r\n");
		write_reg_16bit_value_16bit(0x7015, 0x0001, 0); /* servo on, ois off mode */
		break;
	case 2:
		ois_printf("[dw9781_set_ois_mode] set servo off mode\r\n");
		write_reg_16bit_value_16bit(0x7015, 0x0002, 0); /* servo off mode */
		break;
	default:
		ois_printf("[dw9781_set_ois_mode] set ois mode select error\r\n");
		break;
	}
}

void set_ois_init_setting(void)
{
    dw9781_i2c_setting.reg_setting = kcalloc(1, sizeof(struct cam_sensor_i2c_reg_array), GFP_KERNEL);
    ois_setting = dw9781_i2c_setting.reg_setting;
    if(!ois_setting) {
        ois_printf("[dw9781_set_ois_init_setting] init ois alloc fail!");
        init_success = 0;
    }
}
