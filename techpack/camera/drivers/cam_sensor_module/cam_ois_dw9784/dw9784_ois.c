/**
 ******************************************************************************
 * File Name		  : DW9784_SET_API.C
 * Description		: Main program c file
 * DongWoon Anatech   :
 * Version			: 1.0
 ******************************************************************************
 *
 * COPYRIGHT(c) 2022 DWANATECH
 * DW9784 Setup Program for SET
 * Revision History
 * 2022.06.14 by SJ cho
 ******************************************************************************
 **/
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/err.h>
#include <linux/timekeeping.h>
#include "cam_sensor_i2c.h"
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "cam_common_util.h"
#include "dw9784_ois.h"
#include "dw9784_ois_fw.h"
#include "cam_cci_dev.h"

#define logi pr_info

/*Global buffer for flash download*/
FirmwareContex g_firmwareContext;
unsigned short g_downloadByForce;
unsigned short g_updateFw;
struct cam_ois_ctrl_t dw9784_o_ctrl;
int init_success = 1;
unsigned short DW9784_FW_VERSION = 0;

/*write ois i2c*/
void write_reg_16bit_value_16bit(uint32_t reg_addr, uint32_t reg_data)
{
	struct cam_sensor_i2c_reg_setting dw9784_i2c_setting = {0};
	struct cam_sensor_i2c_reg_array ois_setting = {0};
	ois_setting.reg_addr = reg_addr;
	ois_setting.reg_data = reg_data;
	ois_setting.delay = 0;
	ois_setting.data_mask = 0;
	dw9784_i2c_setting.size = 1;
	dw9784_i2c_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	dw9784_i2c_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	dw9784_i2c_setting.delay = 0;
	dw9784_i2c_setting.read_buff = NULL;
	dw9784_i2c_setting.read_buff_len = 0;
	dw9784_i2c_setting.reg_setting = &ois_setting;
	camera_io_dev_write(&dw9784_o_ctrl.io_master_info, &dw9784_i2c_setting);
}


void i2c_block_write_reg(uint32_t reg_addr, unsigned short *data, uint32_t size)
{
	int base = 0, offset = 0;
	int block_size = 8;
	struct cam_sensor_i2c_reg_setting dw9784_i2c_setting = {0};
	struct cam_sensor_i2c_reg_array ois_setting[8] = {0};
	dw9784_i2c_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	dw9784_i2c_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	dw9784_i2c_setting.delay = 0;
	dw9784_i2c_setting.read_buff = NULL;
	dw9784_i2c_setting.read_buff_len = 0;
	dw9784_i2c_setting.reg_setting = ois_setting;
	while (base < size) {
		block_size = ((size - base < block_size)? (size - base): block_size);
		dw9784_i2c_setting.size = block_size;
		for (offset = 0; offset < block_size; offset++) {
			ois_setting[offset].reg_addr = reg_addr + base + offset;
			ois_setting[offset].reg_data = *(data + base + offset);
			ois_setting[offset].delay = 0;
			ois_setting[offset].data_mask = 0;
		}
		cam_cci_i2c_write_continuous_table(&dw9784_o_ctrl.io_master_info, &dw9784_i2c_setting, 0);
		base += block_size;
	}
}

/*read ois i2c*/
void read_reg_16bit_value_16bit(uint32_t reg_addr, unsigned short *reg_data)
{
	uint8_t *tmp = (uint8_t*)reg_data;
	camera_io_dev_read_seq(&dw9784_o_ctrl.io_master_info, reg_addr, &tmp[0] , 2, 1, 2);
	*(unsigned short *)reg_data = tmp[0] << 8 | tmp[1];
}

/*read fw by i2c*/
void i2c_block_read_reg(uint32_t reg_addr, uint8_t *reg_data, uint32_t size)
{
	int i = 0;
	while (i < size) {
		camera_io_dev_read_seq(&dw9784_o_ctrl.io_master_info, reg_addr + i, reg_data + 2 * i, 2, 2, 2);
		i++;
	}
}

void os_mdelay(int count)
{
	mdelay(count);
}

void GenerateFirmwareContexts(void)
{
	logi("[generateFirmwareContext] enter");
	g_firmwareContext.version = DW9784_FW_VERSION;
	g_firmwareContext.driverIc = 0x9784;
	g_firmwareContext.fwContentPtr = DW9784_FW;
	g_downloadByForce = 0;
	g_updateFw = 0;
}

int dw9784_download_open_camera(struct cam_ois_ctrl_t *o_ctrl)
{
	int ret = 0;
	int err_whoami = 0;
	unsigned int try = 0;
	unsigned short pre_module_state = 0; /* 0x0000: normal, 0xFFFF: abnormal */
	int store_flag = 0;
	memcpy(&dw9784_o_ctrl, o_ctrl, sizeof(struct cam_ois_ctrl_t));
	if (DW9784_FW_VERSION == 0)
		DW9784_FW_VERSION = DW9784_FW[10493];
	GenerateFirmwareContexts();

	err_whoami = dw9784_whoami_chk();
	if (err_whoami == ERROR_SECOND_ID) {
		logi("[dw9784_download_open_camera] failed to check the second_id(0x0020) inside the ic");
		logi("[dw9784_download_open_camera] stop the dw9784 ic boot operation");
		return ERROR_WHO_AMI;
	} else if (err_whoami == ERROR_FW_VALID) {
		/* it works when the function is enabled in dw9784_whoami_chk() */
		g_downloadByForce = 1;
		logi("[dw9784_download_open_camera] force to recovery firmware");
	}

	if (dw9784_chip_id_chk() == DW9784_CHIP_ID) {
		logi("[dw9784_download_open_camera] dw9784 chip_id check pass");
	} else {
		g_downloadByForce = 1;
		logi("[dw9784_download_open_camera] dw9784 chip_id check failed");
		logi("[dw9784_download_open_camera] force to recovery firmware");
	}

	if (dw9784_checksum_fw_chk() == ERROR_FW_CHECKSUM) {
		g_downloadByForce = 1;
		logi("[dw9784_download_open_camera] The firmware checksum check of the flash memory failed.");
		logi("[dw9784_download_open_camera] force to recovery firmware");
	}

	if (dw9784_fw_ver_chk() != DW9784_FW_VERSION ) {
		g_updateFw = 1;
		logi("[dw9784_download_open_camera] updating fw to the newer version firmware");
	} else {
		logi("[dw9784_download_open_camera] current ver: 0x%04X is latest", DW9784_FW_VERSION);
	}

	if (g_downloadByForce == 1) {
		pre_module_state = 0xFFFF; /* abnormal state */
	} else if (g_updateFw == 1) {
		pre_module_state = 0x0000; /* normal state */
	}

	if (g_downloadByForce || g_updateFw) {
try_firmware_update:
		logi("[dw9784_download_open_camera] start downloading firmware, ver: 0x%04X", DW9784_FW_VERSION);
		if (dw9784_download_fw(pre_module_state) == EOK) {
			/* fw download success */
			logi("[dw9784_download_open_camera] complete fw download");
			return EOK;
		} else {
			/* fw download failed */
			if (try++ < 3) {
				logi("[dw9784_download_open_camera] update firmware tried %d times", try);
				goto try_firmware_update;
			}
			dw9784_shutdown_mode();
			logi("[dw9784_download_open_camera] enter ic shutdown(sleep) mode");
			return ERROR_FW_DWON_FAIL;
		}
	}
	store_flag += dw9784_set_gyro_select(6); /*BOCSH  BMI260 */
	store_flag += dw9784_gyro_direction_setting(GYRO_BACK_LAYOUT, GYRO_DEGREE_0);

	if (store_flag)
		ret = dw9784_set_cal_store();

	return ret;
}

int dw9784_download_fw(int module_state)
{
	unsigned char ret = ERROR_FW_VERIFY;
	unsigned short i;
	unsigned short addr;
	unsigned short FMC;
	//unsigned short buf[g_firmwareContext.size];
	/* step 1: MTP Erase and DSP Disable for firmware 0x8000 write */
	write_reg_16bit_value_16bit(0xd001, 0x0000);

	/* step 2: MTP setup */
	dw9784_flash_acess();
	os_mdelay(1);

	/* step 3. FMC register check */
	write_reg_16bit_value_16bit(0xDE01, 0x0000); // FMC block FW select
	os_mdelay(1);
	read_reg_16bit_value_16bit(0xDE01, &FMC);
	if (FMC != 0) {
		logi("[dw9784_download_fw] FMC register value 1st warning : %04x", FMC);
		write_reg_16bit_value_16bit(0xDE01, 0x0000);
		os_mdelay(1);
		FMC = 0; // initialize FMC value

		read_reg_16bit_value_16bit(0xDE01, &FMC);
		if (FMC != 0) {
			logi("[dw9784_download_fw] 2nd FMC register value 2nd warning : %04x", FMC);
			logi("[dw9784_download_fw] stop f/w download");
			return ERROR_FW_DOWN_FMC;
		}
	}

	/* step 4. code protection off */
	dw9784_code_pt_off();

	/* step 5. erase flash fw data */
	dw9784_fw_eflash_erase();

	logi("[dw9784_download_fw] start firmware download");
	/* step 6. firmware sequential write to flash */
	/* updates the module status before firmware download */
	*(g_firmwareContext.fwContentPtr + MCS_SIZE_W -1) = module_state;
	for (i = 0; i < MCS_SIZE_W; i += DATPKT_SIZE) {
		addr = MCS_START_ADDRESS + i;
		i2c_block_write_reg(addr, g_firmwareContext.fwContentPtr + i, DATPKT_SIZE);
	}

	/* step 6. Writes 512Byte FW(PID) data to IF flash.	(FMC register check) */
	write_reg_16bit_value_16bit(0xDE01, 0x1000);
	os_mdelay(1);
	read_reg_16bit_value_16bit(0xDE01, &FMC);
	os_mdelay(1);
	if (FMC != 0x1000) {
		logi("[dw9784_download_fw] IF FMC register value 1st warning : %04x", FMC);
		write_reg_16bit_value_16bit(0xDE01, 0x1000);
		os_mdelay(1);
		FMC = 0; // initialize FMC value

		read_reg_16bit_value_16bit(0xDE01, &FMC);
		if (FMC != 0x1000) {
			logi("[dw9784_download_fw] 2nd IF FMC register value 2nd fail : %04x", FMC);
			logi("[dw9784_download_fw] stop firmware download");
			return ERROR_FW_DOWN_FMC;
		}
	}

	/* step 7. erease IF(FW/PID) eFLASH  */
	dw9784_pid_erase();

	logi("[dw9784_download_fw] start firmware/pid download");
	/* step 8. firmware sequential write to flash */
	for (i = 0; i < PID_SIZE_W; i += DATPKT_SIZE) {
		addr = IF_START_ADDRESS + i;
		i2c_block_write_reg(addr, g_firmwareContext.fwContentPtr + MCS_SIZE_W + i, DATPKT_SIZE);
	}
	logi("[dw9784_download_fw] write firmware/pid to flash");

	/* step 14. ic reboot */
	dw9784_ois_reset();

	/* step 15. check fw_checksum */
	if (dw9784_checksum_fw_chk() == 0) {
		logi("[dw9784_download_fw] fw download success.");
		logi("[dw9784_download_fw] finish");
		return EOK;
	} else {
		logi("[dw9784_download_fw] fw download cheksum fail.");
		logi("[dw9784_download_fw] finish");
		return ERROR_FW_CHECKSUM;
	}
	return ret;
}

int dw9784_whoami_chk(void)
{
	//unsigned short fw_flag1, fw_flag2, fw_flag3;
	unsigned short sec_chip_id;
	write_reg_16bit_value_16bit(0xD000, 0x0001); /* chip enable */
	os_mdelay(4);
	write_reg_16bit_value_16bit(0xD001, 0x0000); /* dsp off mode */
	os_mdelay(1);

	dw9784_flash_acess(); /* All protection */

	read_reg_16bit_value_16bit(0xD060, &sec_chip_id); /* 2nd chip id */
	logi("[dw9784_ois_ready_check] sec_chip_id : 0x%04x", sec_chip_id);
	write_reg_16bit_value_16bit(0x70DA, 0x1002);
	if (sec_chip_id != 0x0020) {
		logi("[dw9784] second_chip_id check fail : 0x%04X", sec_chip_id);
		logi("[dw9784] second_enter shutdown mode");
		logi("[dw9784] dw9784 cannot work");
		write_reg_16bit_value_16bit(0xD000, 0x0000); /* ic */
		return ERROR_SECOND_ID;
	}

	dw9784_ois_reset(); /* ois reset */
	return EOK;
}

unsigned short dw9784_chip_id_chk(void)
{
	unsigned short chip_id;
	read_reg_16bit_value_16bit(DW9784_CHIP_ID_ADDRESS, &chip_id);

	logi("[dw9784_chip_id] chip_id : 0x%04X", chip_id);
	return chip_id;
}

unsigned short dw9784_fw_ver_chk(void)
{
	unsigned short fw_ver;
	unsigned short fw_date;

	read_reg_16bit_value_16bit(0x7001, &fw_ver);
	read_reg_16bit_value_16bit(0x7002, &fw_date);

	logi("[dw9784_chip_id] fw version : 0x%04X", fw_ver);
	logi("[dw9784_chip_id] fw date : 0x%04X", fw_date);
	return fw_ver;
}

int dw9784_fw_type(void)
{
	unsigned short r_data;
	unsigned short eis; /* vsync(0) or qtime(1) */
	unsigned short fw_type; /* module(0) or set(1) */

	read_reg_16bit_value_16bit(0x7006, &r_data);
	logi("[dw9784_chip_id] fw type : 0x%04X", r_data);

	eis = (r_data >> 8) & 0x1;
	fw_type = r_data & 0x1;

	if (eis == EIS_VSYNC ) {
		logi("[dw9784_chip_id] eis mode : vsync");
	} else if (eis == EIS_QTIME ) {
		logi("[dw9784_chip_id] eis mode : qtime");
	}

	if (fw_type == MODULE_FW ) {
		logi("[dw9784_chip_id] fw type : module fw");
	} else if (fw_type == SET_FW ) {
		logi("[dw9784_chip_id] fw type : set fw");
	}
	return fw_type;
}

void dw9784_fw_info(void)
{
	unsigned short r_data;
	logi("[dw9784_fw_info] start");

	read_reg_16bit_value_16bit(0x7000, &r_data);
	logi("[dw9784_fw_info] chip_id : 0x%04X", r_data);
	read_reg_16bit_value_16bit(0x7001, &r_data);
	logi("[dw9784_fw_info] fw version : 0x%04X", r_data);
	read_reg_16bit_value_16bit(0x7002, &r_data);
	logi("[dw9784_fw_info] fw_date : 0x%04X", r_data);
	read_reg_16bit_value_16bit(0x7003, &r_data);
	logi("[dw9784_fw_info] set & project : 0x%04X", r_data);
	read_reg_16bit_value_16bit(0x7006, &r_data);
	logi("[dw9784_fw_info] vsync/qtime & set_module_fw info : 0x%04X", r_data);

	read_reg_16bit_value_16bit(0x71FC, &r_data);
	logi("[dw9784_fw_info] param_date: 0x%04X", r_data);
	read_reg_16bit_value_16bit(0x71FD, &r_data);
	logi("[dw9784_fw_info] module_id : 0x%04X", r_data);
	read_reg_16bit_value_16bit(0x71FE, &r_data);
	logi("[dw9784_fw_info] act_id : 0x%04X", r_data);

	read_reg_16bit_value_16bit(0x700A, &r_data);
	logi("[dw9784_fw_info] reg_module_cal_checksum : 0x%04X", r_data);
	read_reg_16bit_value_16bit(0x700B, &r_data);
	logi("[dw9784_fw_info] reg_set_cal_checksum : 0x%04X", r_data);
	read_reg_16bit_value_16bit(0x700C, &r_data);
	logi("[dw9784_fw_info] reg_fw_checksum : 0x%04X", r_data);
	read_reg_16bit_value_16bit(0x700D, &r_data);
	logi("[dw9784_fw_info] reg_checksum_status : 0x%04X", r_data);
}

int dw9784_erase_mtp_rewritefw(void)
{
	unsigned short FMC;
	logi("dw9784 erase for rewritefw starting..");
	/* dsp off */
	write_reg_16bit_value_16bit(0xd001, 0x0000);
	os_mdelay(1);
	/* all protection off */
	dw9784_flash_acess();

	/* FMC register check */
	write_reg_16bit_value_16bit(0xDE01, 0x0000); // FMC block FW select
	os_mdelay(1);
	read_reg_16bit_value_16bit(0xDE01, &FMC);
	if (FMC != 0) {
		logi("[dw9784_erase_mtp_rewritefw] FMC register value 1st warning : %04x", FMC);
		write_reg_16bit_value_16bit(0xDE01, 0x0000);
		os_mdelay(1);
		FMC = 0; // initialize FMC value

		read_reg_16bit_value_16bit(0xDE01, &FMC);
		if (FMC != 0) {
			logi("[dw9784_erase_mtp_rewritefw] 2nd FMC register value 2nd warning : %04x", FMC);
			logi("[dw9784_erase_mtp_rewritefw] stop f/w download");
			return ERROR_FW_DOWN_FMC;
		}
	}
	/* code protection off */
	dw9784_code_pt_off();

	/* 512 byte page */
	write_reg_16bit_value_16bit(0xde03, 0x0027);
	os_mdelay(1);
	/* page erase */
	write_reg_16bit_value_16bit(0xde04, 0x0008);
	os_mdelay(10);

	write_reg_16bit_value_16bit(0xD000, 0x0000); /* chip shutdown */
	logi("dw9784 enter shutdown mode");

	return EOK;
}

void dw9784_flash_acess(void)
{
	logi("[dw9784_flash_acess] start");
	/* release all protection */
	write_reg_16bit_value_16bit(0xFAFA, 0x98AC);
	os_mdelay(1);
	write_reg_16bit_value_16bit(0xF053, 0x70BD);
	os_mdelay(1);
	logi("[dw9784_flash_acess] finish");
}

void dw9784_code_pt_off(void)
{
	logi("[dw9784_code_pt_off] start");
	/* release all protection */
	write_reg_16bit_value_16bit(0xFD00, 0x5252);
	os_mdelay(1);
	logi("[dw9784_code_pt_off] finish");
}

void dw9784_pid_erase(void)
{
	logi("[dw9784_pid_erase] start pid flash(IF) erase");
	write_reg_16bit_value_16bit(0xde03, 0x0000);			// page 0
	os_mdelay(1);
	write_reg_16bit_value_16bit(0xde04, 0x0008);			// page erase
	os_mdelay(10);											// need to delay after erase
	logi("[dw9784_pid_erase] finish");
}

void dw9784_fw_eflash_erase(void)
{
	logi("[dw9784_fw_eflash_erase] start fw flash erase");
	write_reg_16bit_value_16bit(0xde03, 0x0000);			// 4k Sector_0
	os_mdelay(1);
	write_reg_16bit_value_16bit(0xde04, 0x0002);			// 4k Sector Erase
	os_mdelay(10);						// need to delay after erase

	write_reg_16bit_value_16bit(0xde03, 0x0008);			// 4k Sector_1
	os_mdelay(1);
	write_reg_16bit_value_16bit(0xde04, 0x0002);			// 4k Sector Erase
	os_mdelay(10);						// need to delay after erase

	write_reg_16bit_value_16bit(0xde03, 0x0010);			// 4k Sector_2
	os_mdelay(1);
	write_reg_16bit_value_16bit(0xde04, 0x0002);			// 4k Sector Erase
	os_mdelay(10);						// need to delay after erase

	write_reg_16bit_value_16bit(0xde03, 0x0018);			// 4k Sector_3
	os_mdelay(1);
	write_reg_16bit_value_16bit(0xde04, 0x0002);			// 4k Sector Erase
	os_mdelay(10);						// need to delay after erase

	write_reg_16bit_value_16bit(0xde03, 0x0020);			// 4k Sector_4
	os_mdelay(1);
	write_reg_16bit_value_16bit(0xde04, 0x0002);			// 4k Sector Erase
	os_mdelay(10);						// need to delay after erase
	logi("[dw9784_fw_eflash_erase] finish");
}

void dw9784_shutdown_mode(void)
{
	logi("[all_prot_off] enter ic shutdown mode");
	write_reg_16bit_value_16bit(0xD000, 0x0000);
	os_mdelay(1);
}

void dw9784_ois_reset(void)
{
	logi("[dw9784_ois_reset] start ois reset");
	write_reg_16bit_value_16bit(0xD002, 0x0001); /* logic reset */
	os_mdelay(4);
	write_reg_16bit_value_16bit(0xD001, 0x0001); /* Active mode (DSP ON) */
	os_mdelay(25);
	write_reg_16bit_value_16bit(0xEBF1, 0x56FA); /* User protection release */
	logi("[dw9784_ois_reset] finish");
}

int dw9784_ois_on(void)
{
	logi("[dw9784_ois_on] ois on mode change");
	write_reg_16bit_value_16bit(0x7012, 0x0001); /* set control mode */
	os_mdelay(1);
	write_reg_16bit_value_16bit(0x7011, 0x0000); /* ois on */
	os_mdelay(1);
	if (dw9784_wait_check_register(0x7010, 0x1000) == FUNC_PASS) { /* busy check */
		logi("[dw9784_ois_on] ois on success");
		return FUNC_PASS;
	}

	return FUNC_FAIL;
}

int dw9784_servo_on(void)
{
	logi("[dw9784_servo_on] servo on mode change");
	write_reg_16bit_value_16bit(0x7012, 0x0001); /* set control mode */
	os_mdelay(1);
	write_reg_16bit_value_16bit(0x7011, 0x0001); /* servo on */
	os_mdelay(1);
	if (dw9784_wait_check_register(0x7010, 0x1001) == FUNC_PASS) { /* busy check */
		logi("[dw9784_servo_on] servo on success");
		return FUNC_PASS;
	}

	return FUNC_FAIL;
}

int dw9784_servo_off(void)
{
	logi("[dw9784_servo_off] servo off mode change");
	write_reg_16bit_value_16bit(0x7012, 0x0001); /* Set control mode */
	os_mdelay(1);
	write_reg_16bit_value_16bit(0x7011, 0x0002); /* servo off */
	os_mdelay(1);

	if (dw9784_wait_check_register(0x7010, 0x1002) == FUNC_PASS) { /* busy check */
		logi("[dw9784_servo_off] servo off success");
		return FUNC_PASS;
	}

	return FUNC_FAIL;
}

int dw9784_ois_status(struct cam_ois_ctrl_t *o_ctrl){
	unsigned short ois_status;
	int ret = FUNC_PASS;
	memcpy(&dw9784_o_ctrl, o_ctrl, sizeof(struct cam_ois_ctrl_t));
	read_reg_16bit_value_16bit(0x7010, &ois_status);
	switch (ois_status) {
	case 0x1000:
		logi("[dw9784_ois_status] ois on");
		break;
	case 0x1001:
		logi("[dw9784_ois_status] ois off");
		break;
	case 0x1002:
		logi("[dw9784_ois_status] servo off");
		break;
	default:
		logi("[dw9784_ois_status] error status");
	}
	return ret;
}


int dw9784_checksum_all_chk(void)
{
	/*
	   Bit [0]: FW checksum error
	   Bit [1]: Module cal. checksum error
	   Bit [2]: Set cal. checksum error
	 */
	unsigned short reg_fw_checksum;			// 0x700C
	unsigned short reg_module_cal_checksum;	// 0x700A
	unsigned short reg_set_cal_checksum;		// 0x700B
	unsigned short reg_checksum_status;		// 0x700D

	read_reg_16bit_value_16bit(0x700C, &reg_fw_checksum);
	read_reg_16bit_value_16bit(0x700A, &reg_module_cal_checksum);
	read_reg_16bit_value_16bit(0x700B, &reg_set_cal_checksum);
	read_reg_16bit_value_16bit(0x700D, &reg_checksum_status);

	logi("[dw9784_checksum_all_chk] reg_checksum_status : 0x%04X", reg_checksum_status);
	//logi("[dw9784_checksum_all_chk] ref_fw_checksum : 0x%04X, reg_fw_checksum : 0x%04X", DW9784_FW_CHECKSUM, reg_fw_checksum);

	if ((reg_checksum_status & 0x0111) == 0) {
		logi("[dw9784_checksum_all_chk] fw checksum pass");
		return EOK;
	} else {
		if (reg_checksum_status & 0x0001)
			logi("[dw9784_checksum_all_chk] fw checksum error");

		if (reg_checksum_status & 0x0010)
			logi("[dw9784_checksum_all_chk] module cal. checksum error");

		if (reg_checksum_status & 0x0100)
			logi("[dw9784_checksum_all_chk] set cal. checksum error");

		logi("[dw9784_checksum_all_chk] module_cal_checksum : 0x%04X", reg_module_cal_checksum);
		logi("[dw9784_checksum_all_chk] set_cal_checksum : 0x%04X", reg_set_cal_checksum);

		return ERROR_ALL_CHECKUM;
	}
}

int dw9784_checksum_fw_chk(void)
{
	/*
	   Bit [0]: FW checksum error
	   Bit [1]: Module cal. checksum error
	   Bit [2]: Set cal. checksum error
	 */
	unsigned short reg_fw_checksum;			// 0x700C
	unsigned short reg_checksum_status;		// 0x700D

	read_reg_16bit_value_16bit(0x700C, &reg_fw_checksum);
	read_reg_16bit_value_16bit(0x700D, &reg_checksum_status);

	logi("[dw9784_checksum_fw_chk] reg_checksum_status : 0x%04X", reg_checksum_status);
	//logi("[dw9784_checksum_fw_chk] ref_fw_checksum : 0x%04X, reg_fw_checksum : 0x%04X", DW9784_FW_CHECKSUM, reg_fw_checksum);

	if ((reg_checksum_status & 0x0001) == 0) {
		logi("[dw9784_checksum_fw_chk] fw checksum pass");
		return EOK;
	} else {
		logi("[dw9784_checksum_fw_chk] fw checksum error");
		return ERROR_FW_CHECKSUM;
	}
}

int dw9784_set_gyro_select(unsigned short new_gyro_id)
{
	unsigned short old_gyro_id;
	int upd_flag = 0;

	read_reg_16bit_value_16bit(0x7194, &old_gyro_id); /* gyro sensor set in register */

	if (old_gyro_id != new_gyro_id) {
		write_reg_16bit_value_16bit(0x7194, new_gyro_id);
		upd_flag = 1;
		logi("[dw9784_set_gyro_select] gyro sensor information has been updated");
		logi("[dw9784_set_gyro_select] before set_gyro: %d, after set_gyro: %d",old_gyro_id, new_gyro_id);
	} else {
		logi("[dw9784_set_gyro_select] the gyro sensor does not change, 0x%04X", new_gyro_id);
	}

	if (new_gyro_id == 0)
		logi("[dw9784_set_gyro_select] ICM_20690");
	else if (new_gyro_id == 2)
		logi("[dw9784_set_gyro_select] ST_LSM6DSM");
	else if (new_gyro_id == 4)
		logi("[dw9784_set_gyro_select] ST_LSM6DSOQ");
	else if (new_gyro_id == 5)
		logi("[dw9784_set_gyro_select] ICM_42631");
	else if (new_gyro_id == 6)
		logi("[dw9784_set_gyro_select] BMI260");
	else if (new_gyro_id == 7)
		logi("[dw9784_set_gyro_select] ICM_42602");
	else {
		logi("[dw9784_set_gyro_select] gyro sensor selection failed");
		return FUNC_FAIL;
	}
	return upd_flag;
}

int dw9784_gyro_direction_setting(int gyro_arrangement, int gyro_degree)
{
	int upd_flag = 0;
	unsigned short gyro_id;
	/* set register new value */
	unsigned short x_gyro_gain_pol, y_gyro_gain_pol;
	unsigned short x_gyro_mat_cos, x_gyro_mat_sin;
	unsigned short y_gyro_mat_cos, y_gyro_mat_sin;
	/* read register value */
	unsigned short x_reg_gyro_gain_pol, y_reg_gyro_gain_pol;
	unsigned short x_reg_mat_cos, x_reg_mat_sin;
	unsigned short y_reg_mat_cos, y_reg_mat_sin;

	read_reg_16bit_value_16bit(0x7194, &gyro_id); /* gyro sensor set in register */

	read_reg_16bit_value_16bit(0x7184, &x_reg_gyro_gain_pol); /* x_gyro_gain_pol reg */
	read_reg_16bit_value_16bit(0x7185, &y_reg_gyro_gain_pol); /* y_gyro_gain_pol reg */

	read_reg_16bit_value_16bit(0x7186, &x_reg_mat_cos); /* x_gyro_mat_cos */
	read_reg_16bit_value_16bit(0x7188, &y_reg_mat_cos); /* y_gyro_mat_cos */
	read_reg_16bit_value_16bit(0x7187, &x_reg_mat_sin); /* x_gyromat_sin */
	read_reg_16bit_value_16bit(0x7189, &y_reg_mat_sin); /* y_gyromat_sin */

	if (gyro_id == 6) { /* Bosch Gryo */
		logi("[dw9784_gyro_direction_setting] dw9784 bosch gyro direction is setting ");
		switch (gyro_degree) {
		case GYRO_DEGREE_0:
			gyro_degree = GYRO_DEGREE_90;
			break;
		case GYRO_DEGREE_90:
			gyro_degree = GYRO_DEGREE_180;
			break;
		case GYRO_DEGREE_180:
			gyro_degree = GYRO_DEGREE_270;
			break;
		case GYRO_DEGREE_270:
			gyro_degree = GYRO_DEGREE_0;
			break;
		}
	}

	if (gyro_arrangement == GYRO_FRONT_LAYOUT) {
		/* Gyro Polarity - Front Side Layout */
		x_gyro_gain_pol = 0xFFFF; /* x_gyro_gain_pol reg: 0x7184 */
		y_gyro_gain_pol = 0xFFFF; /* y_gyro_gain_pol reg: 0x7185 */
	} else if (gyro_arrangement == GYRO_BACK_LAYOUT) {
		/* Gyro Polarity - Back Side Layout */
		x_gyro_gain_pol = 0x0001; /* x_gyro_gain_pol reg: 0x7184 */
		y_gyro_gain_pol = 0xFFFF; /* y_gyro_gain_pol reg: 0x7185 */
	} else {
		x_gyro_gain_pol = 0x0000; /* x_gyro_gain_pol reg: 0x7184 */
		y_gyro_gain_pol = 0x0000; /* y_gyro_gain_pol reg: 0x7185 */
		logi("[dw9784_gyro_direction_setting] the location information of the gyro sensor is incorrect");
	}

	switch (gyro_degree) {
	case GYRO_DEGREE_0:
		x_gyro_mat_cos = 0x7FFF; /* x_gyro_mat_cos reg: 0x7186 */
		y_gyro_mat_cos = 0x7FFF; /* y_gyro_mat_cos reg: 0x7188 */
		x_gyro_mat_sin = 0x0000; /* x_gyro_mat_sin reg: 0x7187 */
		y_gyro_mat_sin = 0x0000; /* y_gyro_mat_sin reg: 0x7189 */
		break;
	case GYRO_DEGREE_90:
		x_gyro_mat_cos = 0x0000;
		y_gyro_mat_cos = 0x0000;
		x_gyro_mat_sin = 0x8000;
		y_gyro_mat_sin = 0x7FFF;
		break;
	case GYRO_DEGREE_180:
		x_gyro_mat_cos = 0x8000;
		y_gyro_mat_cos = 0x8000;
		x_gyro_mat_sin = 0x0000;
		y_gyro_mat_sin = 0x0000;
		break;
	case GYRO_DEGREE_270:
		x_gyro_mat_cos = 0x0000;
		y_gyro_mat_cos = 0x0000;
		x_gyro_mat_sin = 0x7FFF;
		y_gyro_mat_sin = 0x8000;
		break;
	default:
		logi("[dw9784_gyro_direction_setting] the rotation information of the gyro sensor is incorrect");
		logi("[dw9784_gyro_direction_setting] it is set to 0 degree by default");
		x_gyro_mat_cos = 0x7FFF; /* x_gyro_mat_cos reg: 0x7186 */
		y_gyro_mat_cos = 0x7FFF; /* y_gyro_mat_cos reg: 0x7188 */
		x_gyro_mat_sin = 0x0000; /* x_gyro_mat_sin reg: 0x7187 */
		y_gyro_mat_sin = 0x0000; /* y_gyro_mat_sin reg: 0x7189 */
	}

	if (x_gyro_gain_pol != x_reg_gyro_gain_pol && y_gyro_gain_pol != y_reg_gyro_gain_pol) {
		logi("the gyro gain polarity value has been updated");
		logi("x gyro gain pol: 0x%04X, y gyro gain pol: 0x%04X", x_gyro_gain_pol, y_gyro_gain_pol);

		write_reg_16bit_value_16bit(0x7184, x_gyro_gain_pol);
		write_reg_16bit_value_16bit(0x7185, y_gyro_gain_pol);
		upd_flag = 1;
	}

	if ((x_gyro_mat_cos != x_reg_mat_cos) && (y_gyro_mat_cos != y_reg_mat_cos) &&
		(x_gyro_mat_sin != x_reg_mat_sin) && (y_gyro_mat_sin != y_reg_mat_sin)) {
		logi("[dw9784_gyro_direction_setting] the gyro matrix value has been updated");
		logi("[dw9784_gyro_direction_setting] x_gyro_mat_cos: 0x%04X", x_gyro_mat_cos);
		logi("[dw9784_gyro_direction_setting] y_gyro_mat_cos: 0x%04X", y_gyro_mat_cos);
		logi("[dw9784_gyro_direction_setting] x_gyro_mat_sin: 0x%04X", x_gyro_mat_sin);
		logi("[dw9784_gyro_direction_setting] y_gyro_mat_sin: 0x%04X", y_gyro_mat_sin);

		write_reg_16bit_value_16bit(0x7186, x_gyro_mat_cos);
		write_reg_16bit_value_16bit(0x7188, y_gyro_mat_cos);
		write_reg_16bit_value_16bit(0x7187, x_gyro_mat_sin);
		write_reg_16bit_value_16bit(0x7189, y_gyro_mat_sin);
		upd_flag = 1;
	}
	return upd_flag;
}

int dw9784_gyro_ofs_calibration(struct cam_ois_ctrl_t *o_ctrl)
{
	/*
	 * dw9784 gyro offset calibration
	 Error code definition
	 -1 : FUNC_FAIL
	  0 : No Error
	 */
	unsigned short x_ofs, y_ofs, gyro_status;
	int msg = 0;
	memcpy(&dw9784_o_ctrl, o_ctrl, sizeof(struct cam_ois_ctrl_t));
	logi("[dw9784_gyro_ofs_calibration] start");
	write_reg_16bit_value_16bit(0x7012, 0x0006); // gyro offset calibration
	os_mdelay(1);

	if (dw9784_wait_check_register(0x7010, 0x6000) == FUNC_PASS) {
		write_reg_16bit_value_16bit(0x7011, 0x0001); // gyro ofs calibration execute command
		os_mdelay(100);
	} else {
		logi("[dw9784_gyro_ofs_calibration] FUNC_FAIL");
		return FUNC_FAIL;
	}
	if (dw9784_wait_check_register(0x7010, 0x6001) == FUNC_PASS) { // when calibration is done, Status changes to 0x6001
		logi("[dw9784_gyro_ofs_calibration]calibration function finish");
	} else {
		logi("[dw9784_gyro_ofs_calibration]calibration function error");
		return FUNC_FAIL;
	}

	read_reg_16bit_value_16bit(0x7180, &x_ofs); /* x gyro offset */
	read_reg_16bit_value_16bit(0x7181, &y_ofs); /* y gyro offset */
	read_reg_16bit_value_16bit(0x7195, &gyro_status); /* gyro offset status */
	logi("[dw9784_gyro_ofs_calibration]x gyro offset: 0x%04X(%d)", x_ofs, (short)x_ofs);
	logi("[dw9784_gyro_ofs_calibration]y gyro offset: 0x%04X(%d)", y_ofs, (short)y_ofs);
	logi("[dw9784_gyro_ofs_calibration]gyro_status: 0x%04X", gyro_status);

	if ((gyro_status & 0x8000)== 0x8000) {	/* Read Gyro offset cailbration result status */
		if ((gyro_status & 0x1) == X_AXIS_GYRO_OFS_PASS) {
			msg = EOK;
			logi("[dw9784_gyro_ofs_calibration] x gyro ofs cal pass");
		} else {
			msg += X_AXIS_GYRO_OFS_FAIL;
			logi("[dw9784_gyro_ofs_calibration] x gyro ofs cal fail");
		}

		if ((gyro_status & 0x10) == X_AXIS_GYRO_OFS_OVER_MAX_LIMIT) {
			msg += X_AXIS_GYRO_OFS_OVER_MAX_LIMIT;
			logi("[dw9784_gyro_ofs_calibration] x gyro ofs over the max. limit");
		}

		if ((gyro_status & 0x2) == Y_AXIS_GYRO_OFS_PASS) {
			msg += EOK;
			logi("[dw9784_gyro_ofs_calibration] y gyro ofs cal pass");
		} else {
			msg += Y_AXIS_GYRO_OFS_FAIL;
			logi("[dw9784_gyro_ofs_calibration] y gyro ofs cal fail");
		}

		if ((gyro_status & 0x20) == Y_AXIS_GYRO_OFS_OVER_MAX_LIMIT) {
			msg += Y_AXIS_GYRO_OFS_OVER_MAX_LIMIT;
			logi("[dw9784_gyro_ofs_calibration] y gyro ofs over the max. limit");
		}

		if ((gyro_status & 0x800) == XY_AXIS_CHECK_GYRO_RAW_DATA) {
			msg += XY_AXIS_CHECK_GYRO_RAW_DATA;
			logi("[dw9784_gyro_ofs_calibration] check the x/y gyro raw data");
		}
		logi("[dw9784_gyro_ofs_calibration] x/y gyro ofs calibration finish");

		if (msg == EOK) {
			msg = dw9784_set_cal_store();
		}
		return msg;
	} else {
		logi("[dw9784_gyro_ofs_calibration] x/y gyro ofs calibration done fail");
		logi("[dw9784_gyro_ofs_calibration] x/y gyro ofs calibration finish");
		return GYRO_OFS_CAL_DONE_FAIL;
	}
	return GYRO_OFS_CAL_DONE_FAIL;
}

int dw9784_set_cal_store(void)
{
	/*
	   Error code definition
	 0 : No Error
	-1 : FUNC_FAIL
	 */
	logi("[dw9784_set_cal_store] start");
	write_reg_16bit_value_16bit(0x7012, 0x000A); //Set store mode

	//When store is done, status changes to 0xA000
	if (dw9784_wait_check_register(0x7010, 0xA000) == FUNC_PASS) {
		logi("[dw9784_set_cal_store] successful entry into store mode");
	} else {
		logi("[dw9784_set_cal_store] failed to enter store mode");
		return FUNC_FAIL;
	}

	dw9784_code_pt_off(); /* code protection off */
	write_reg_16bit_value_16bit(0x700F, 0x5959); //Set protect code
	os_mdelay(1);
	write_reg_16bit_value_16bit(0x7011, 0x0001); //Execute store
	os_mdelay(40);

	//When store is done, status changes to 0xA001
	if (dw9784_wait_check_register(0x7010, 0xA001) == FUNC_PASS) {
		dw9784_ois_reset();
		logi("[dw9784_set_cal_store] finish");
	} else {
		logi("[dw9784_set_cal_store] store function fail");
		return FUNC_FAIL;
	}
	return FUNC_PASS;
}

int dw9784_wait_check_register(unsigned short reg, unsigned short ref)
{
	/*
	reg : read target register
	ref : compare reference data
	 */
	unsigned short r_data;
	int i = 0;

	for (i = 0; i < LOOP_A; i++) {
		read_reg_16bit_value_16bit(reg, &r_data); //Read status
		if (r_data == ref) {
			break;
		} else {
			if (i >= LOOP_B) {
				logi("[dw9784_wait_check_register]fail: 0x%04X", r_data);
				return FUNC_FAIL;
			}
		}
		os_mdelay(WAIT_TIME);
	}
	return FUNC_PASS;
}

void dw9784_fw_read(void)
{

	/* Read the data of fw memory using register */
	unsigned short buf_R[10240];
	int i = 0;
	write_reg_16bit_value_16bit(0xD001, 0x0000); /* dsp mode */
	os_mdelay(1);
	dw9784_flash_acess();
	logi("dw9784_fw_read");
	/* FW Register Read */
	for (i = 0; i < 10240; i++) {
		read_reg_16bit_value_16bit(0x2000+i, buf_R+i);
	}

	for (i = 0; i < 10240; i+= 0x10) {
		/* log for debug */
		logi("[dw9784_fw_read] %04X = %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X", 
				0x2000 + i, buf_R[i + 0], buf_R[i + 1], buf_R[i + 2],buf_R[i + 3],buf_R[i + 4],buf_R[i + 5], buf_R[i + 6], buf_R[i + 7], 
				buf_R[i + 8], buf_R[i + 9], buf_R[i + 10], buf_R[i + 11], buf_R[i + 12], buf_R[i + 13], buf_R[i + 14], buf_R[i + 15] ); 
	}
	dw9784_ois_reset();
}

void dw9784_flash_if_ram_read(void)
{
	/* Read the data of IF memory using RAM register */
	unsigned short buf_R[640];
	int i = 0;
	memset(buf_R, 0, 640 * sizeof(unsigned short));

	for (i = 0; i < 640; i++) {
		read_reg_16bit_value_16bit(0x7180+i, buf_R+i);
	}
	logi("[dw9784_flash_if_ram_read] IF_Memory Data Log");
	for (i = 0; i < 640; i+= 0x10) {
		/* log for debug */
		logi("[dw9784_flash_if_ram_read] %04X = %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X", 
				0x7180 + i, buf_R[i + 0], buf_R[i + 1], buf_R[i + 2],buf_R[i + 3],buf_R[i + 4],buf_R[i + 5], buf_R[i + 6], buf_R[i + 7], 
				buf_R[i + 8], buf_R[i + 9], buf_R[i + 10], buf_R[i + 11], buf_R[i + 12], buf_R[i + 13], buf_R[i + 14], buf_R[i + 15] ); 
	}
}

void dw9784_flash_ldt_register_read(void)
{
	/* Read the data of LDT memory using register */
	unsigned short buf_R[20];
	int i = 0;
	memset(buf_R, 0, 20 * sizeof(unsigned short));
	write_reg_16bit_value_16bit(0xD001, 0x0000); /* dsp mode */
	os_mdelay(1);
	dw9784_flash_acess();
	/* LDT Register Read */
	for (i = 0; i < 20; i++) {
		read_reg_16bit_value_16bit(0xD060+i, (unsigned short*)buf_R+i);
		logi("[dw9784_flash_ldt_register_read] LDT : buf_R[%04X] = %04X", 0xD060 + i, buf_R[i]); /* log for debug */
	}
	dw9784_ois_reset();
}

