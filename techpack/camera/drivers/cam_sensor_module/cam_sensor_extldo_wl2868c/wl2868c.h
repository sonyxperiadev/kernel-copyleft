#ifndef _WL2868C_H_
#define _WL2868C_H_

#include "../cam_sensor_utils/cam_sensor_cmn_header.h"

#define WL2868C_CHIP_REV_ADDR    0x00
#define ET5907_CHIP_REV_ADDR     0x01
#define FAN53870_CHIP_REV_ADDR   0x01

#define WL2868C_DIC_ADDR         0x02

#define WL2868C_LDO_I2C_ADDR     0x2F
#define ET5907_LDO_I2C_ADDR      0x35
#define FAN53870_LDO_I2C_ADDR    0x35

#define CAMERA_LDO_WL2868C  0x82 //VERID
#define CAMERA_LDO_ET5907   0x00
#define CAMERA_LDO_FAN53870 0x01

#define LDO1_OUT_ADDR    0x04 //DVDD1
//#define LDO2_OUT_ADDR    0x05 //DVDD2
//#define LDO3_OUT_ADDR    0x06 //AVDD1
//#define LDO4_OUT_ADDR    0x07 //AVDD2
//#define LDO5_OUT_ADDR    0x08 //VDDAF
//#define LDO6_OUT_ADDR    0x09 //VDDOIS
//#define LDO7_OUT_ADDR    0x0A //VDDIO

#define WL2868C_LDO1_OUT_ADDR    0x03 //DVDD1
//#define WL2868C_LDO2_OUT_ADDR    0x04 //DVDD2
//#define WL2868C_LDO3_OUT_ADDR    0x05 //AVDD1
//#define WL2868C_LDO4_OUT_ADDR    0x06 //AVDD2
//#define WL2868C_LDO5_OUT_ADDR    0x07 //VDDAF
//#define WL2868C_LDO6_OUT_ADDR    0x08 //VDDOIS
//#define WL2868C_LDO7_OUT_ADDR    0x09 //VDDIO

#define LDO21_SEQ_ADDR   0x0B
#define LDO43_SEQ_ADDR   0x0C
#define LDO65_SEQ_ADDR   0x0D
#define LDO07_SEQ_ADDR   0x0E

#define WL2868C_LDO21_SEQ_ADDR   0x0A
#define WL2868C_LDO43_SEQ_ADDR   0x0B
#define WL2868C_LDO65_SEQ_ADDR   0x0C
#define WL2868C_LDO07_SEQ_ADDR   0x0D

#define ET5907_LDO_EN_ADDR           0x03  //bit0:LDO1 ~ bit6:LDO7
#define FAN53870_LDO_EN_ADDR         0x03  //bit0:LDO1 ~ bit6:LDO7
#define WL2868C_LDO_EN_ADDR          0x0E  //bit0:LDO1 ~ bit6:LDO7
#define WL2868C_LDO_SEQ_CTRL_ADDR    0x0F

#define WL2868C_PRINT pr_info //printk


typedef enum {
	WL2868C_NONE=-1,
	WL2868C_DVDD1,
	WL2868C_DVDD2,
	WL2868C_AVDD1,
	WL2868C_AVDD2,
	WL2868C_VDDAF,
	WL2868C_VDDOIS,
	WL2868C_VDDIO,
	WL2868C_MAX
} WL2868C_SELECT;


/* DTS state */
typedef enum {
	WL2868C_GPIO_STATE_ENP0,
	WL2868C_GPIO_STATE_ENP1,
	WL2868C_GPIO_STATE_MAX,	/* for array size */
} WL2868C_GPIO_STATE;

enum CAM_SENSOR_INDEX {
	CAMERA_INDEX_MAIN = 0,    //BackMain
	//CAMERA_INDEX_MAIN_DEPTH,
	CAMERA_INDEX_FRONT,		  //FrontMain
	CAMERA_INDEX_MAIN_WIDE,   //BackWide
	CAMERA_INDEX_MAIN_TELE,   //Back tele
	CAMERA_INDEX_MAX_NUM,
};

struct wl2868c_ldomap {
	enum CAM_SENSOR_INDEX sensor_index;
	enum msm_camera_power_seq_type seq_type;
	WL2868C_SELECT ldo_selected;
};

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
//extern void wl2868c_gpio_select_state(WL2868C_GPIO_STATE s);
//extern void wl2868c_set_ldo_value(WL2868C_SELECT ldonum,unsigned int value);
//extern void wl2868c_set_en_ldo(WL2868C_SELECT ldonum,unsigned int en);
extern int wl2868c_set_ldo_enable(uint32_t sensor_index, enum msm_camera_power_seq_type seq_type, uint32_t min_voltage, uint32_t max_voltage);
extern int wl2868c_set_ldo_disable(uint32_t sensor_index, enum msm_camera_power_seq_type seq_type);
int wl2868c_init_module(void);
void wl2868c_exit_module(void);

#endif /* _WL2868C_H_*/
