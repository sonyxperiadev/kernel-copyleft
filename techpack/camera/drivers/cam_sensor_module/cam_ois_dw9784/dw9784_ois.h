/**
  ******************************************************************************
  * File Name          : DW9784_SET_API.H
  * Description        : Main program c file
  * DongWoon Anatech   :
  * Version            : 1.0
  ******************************************************************************
  *
  * COPYRIGHT(c) 2022 DWANATECH
  * DW9784 Setup Program for SET
  * Revision History
  * 2022.06.14 by SJ cho
  ******************************************************************************
**/
#include <cam_sensor_cmn_header.h>
#include "../cam_ois/cam_ois_dev.h"

void os_mdelay(int count);
void GenerateFirmwareContexts(void);
int dw9784_download_open_camera(struct cam_ois_ctrl_t *o_ctrl);
int dw9784_whoami_chk(void);
unsigned short dw9784_chip_id_chk(void);
int dw9784_checksum_fw_chk(void);
unsigned short dw9784_fw_ver_chk(void);
int dw9784_fw_type(void);
void dw9784_fw_info(void);
int dw9784_erase_mtp_rewritefw(void);
int dw9784_download_fw(int module_state);
void dw9784_flash_acess(void);
void dw9784_code_pt_off(void);
void dw9784_pid_erase(void);
void dw9784_fw_eflash_erase(void);
void dw9784_shutdown_mode(void);
void dw9784_ois_reset(void);
int dw9784_ois_on(void);
int dw9784_servo_on(void);
int dw9784_servo_off(void);
int dw9784_checksum_all_chk(void);
int dw9784_checksum_fw_chk(void);
int dw9784_set_gyro_select(unsigned short new_gyro_id);
int dw9784_gyro_direction_setting(int gyro_arrangement, int gyro_degree);
int dw9784_gyro_ofs_calibration(struct cam_ois_ctrl_t *o_ctrl);
int dw9784_set_cal_store(void);
int dw9784_wait_check_register(unsigned short reg, unsigned short value);
void dw9784_fw_read(void);
void dw9784_flash_if_ram_read(void);
void dw9784_flash_ldt_register_read(void);
void set_ois_init_setting(void);
int dw9784_ois_status(struct cam_ois_ctrl_t *o_ctrl);

typedef struct
{
	unsigned short driverIc;
	unsigned short *fwContentPtr;
	unsigned short version;
} FirmwareContex;

#define LOOP_A							200
#define	LOOP_B							LOOP_A-1
#define WAIT_TIME						100

/* fw_update_item */
#define UPDATE_ITEM_LENGTH 3

/* fw downlaod */
#define DW9784_CHIP_ID_ADDRESS			0x7000
#define DW9784_CHIP_ID					0x9784
#define DW9784_FW_VER_ADDR				0x7001
#define DW9784_FW_DATE_ADDR				0x7002
#define DW9784_FW_PRJ_ADDR				0x7003
#define DW9784_FW_TYPE_ADDR				0x7006

#define MODULE_FW						0
#define SET_FW							1
#define EIS_VSYNC						0
#define EIS_QTIME						1

#define FUNC_PASS						0
#define FUNC_FAIL						-1
#define ERROR_ALLOC						-2

#define EOK 							0
#define ERROR_SECOND_ID 				1
#define ERROR_WHOAMI					2
#define ERROR_FW_VALID					3
#define ERROR_FW_VERIFY 				4
#define ERROR_FW_CHECKSUM				5
#define ERROR_FW_DOWN_FMC				6
#define ERROR_FW_DWON_FAIL				7
#define ERROR_ALL_CHECKUM				8
#define ERROR_WHO_AMI					9

#define MAX_FIRMWARE_NUMBER 			5
#define DATPKT_SIZE 					256

#define DW9784_CHIP_ID					0x9784
#define AUTORD_RV_OK					0x0000

#define MCS_START_ADDRESS				0x8000
#define IF_START_ADDRESS				0x8000
#define MCS_SIZE_W						10240	//20KB
#define PID_SIZE_W						256		//0.5KB

/* gyro offset calibration */
#define GYRO_OFS_CAL_DONE_FAIL			0xFF
#define X_AXIS_GYRO_OFS_PASS			0x1
#define X_AXIS_GYRO_OFS_FAIL			0x1
#define Y_AXIS_GYRO_OFS_PASS			0x2
#define Y_AXIS_GYRO_OFS_FAIL			0x2
#define X_AXIS_GYRO_OFS_OVER_MAX_LIMIT	0x10
#define Y_AXIS_GYRO_OFS_OVER_MAX_LIMIT	0x20
#define XY_AXIS_CHECK_GYRO_RAW_DATA		0x800

#define GYRO_FRONT_LAYOUT				0
#define GYRO_BACK_LAYOUT				1

#define GYRO_DEGREE_0					0
#define GYRO_DEGREE_90					90
#define GYRO_DEGREE_180					180
#define	GYRO_DEGREE_270					270
