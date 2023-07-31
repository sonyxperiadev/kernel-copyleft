#ifndef _DW9781_OIS_H_
#define _DW9781_OIS_H_

#include <cam_sensor_cmn_header.h>
#include "../cam_ois/cam_ois_dev.h"
#define ois_printf pr_info

#define ADJ_OK					0
#define DW9781_WHOAMI_ERROR		-1
#define ERROR_FW_VERIFY			-2
#define ERROR_ALLOC             -3
#define ERROR_READ_FW           -4

#define DW9781_WHOAMI_ADDRESS		0xD060
#define DW9781_WHOAMI_DATA			0x0020

/* fw downlaod */
#define DW9781_CHIP_ID_ADDRESS		0x7000
#define DW9781_CHIP_ID				0x9781
#define FW_VER_CURR_ADDR			0x7001
#define MTP_START_ADDRESS			0x8000
#define DATPKT_SIZE					16	//256

/* gyro offset calibration */
#define GYRO_OFFSET_CAL_OK 0x00
#define GYRO_CAL_TIME_OVER 0xFF
#define X_GYRO_OFFSET_SPEC_OVER_NG 0x0001
#define X_GYRO_RAW_DATA_CHECK 0x0010
#define Y_GYRO_OFFSET_SPEC_OVER_NG 0x0002
#define Y_GYRO_RAW_DATA_CHECK 0x0020
#define GYRO_OFST_CAL_OVERCNT 100


void os_mdelay(unsigned int delay);
int dw9781_whoami_check(void);
void ois_reset(void);
void ois_ready_check(void);
void GenerateFirmwareContexts(void);
void all_protection_release(void);
int dw9781_download_ois_fw(struct cam_ois_ctrl_t *o_ctrl);
unsigned short fw_checksum_verify(void);
int erase_mtp_rewritefw(void);
void erase_mtp(void);
int download_fw(void);
int gyro_offset_calibrtion(struct cam_ois_ctrl_t *o_ctrl);
void calibration_save(void);
void dw9781_exit(void);
void set_ois_mode(int ois_mode);
void set_ois_init_setting(void);
#endif
