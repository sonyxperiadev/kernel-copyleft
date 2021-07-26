#ifndef _WL2864C_H_
#define _WL2864C_H_

#include "../cam_sensor_utils/cam_sensor_cmn_header.h"

#define WL2864C_CHIP_REV_ADDR    0x00
#define WL2864C_CL_ADDR    		0x01
#define WL2864C_DIC_ADDR         0x02

#define WL2864C_LDO1_OUT_ADDR    0x03 //DVDD1
#define WL2864C_LDO2_OUT_ADDR    0x04 //DVDD2
#define WL2864C_LDO3_OUT_ADDR    0x05 //AVDD1
#define WL2864C_LDO4_OUT_ADDR    0x06 //AVDD2
#define WL2864C_LDO5_OUT_ADDR    0x07 //VDDAF
#define WL2864C_LDO6_OUT_ADDR    0x08 //VDDOIS
#define WL2864C_LDO7_OUT_ADDR    0x09 //VDDIO

#define WL2864C_LDO21_SEQ_ADDR   0x0A
#define WL2864C_LDO43_SEQ_ADDR   0x0B
#define WL2864C_LDO65_SEQ_ADDR   0x0C
#define WL2864C_LDO07_SEQ_ADDR   0x0D

#define WL2864C_LDO_EN_ADDR          0x0E  //bit0:LDO1 ~ bit6:LDO7
#define WL2864C_LDO_SEQ_CTRL_ADDR    0x0F

#define WL2864C_PRINT pr_info //printk


typedef enum {
	WL2864C_NONE=-1,
	WL2864C_DVDD1,
	WL2864C_DVDD2,
	WL2864C_AVDD1,
	WL2864C_AVDD2,
	WL2864C_VDDAF,
	WL2864C_VDDOIS,
	WL2864C_VDDIO,
	WL2864C_MAX
} WL2864C_SELECT;


/* DTS state */
typedef enum {
	WL2864C_GPIO_STATE_ENP0,
	WL2864C_GPIO_STATE_ENP1,
	WL2864C_GPIO_STATE_MAX,	/* for array size */
} WL2864C_GPIO_STATE;

enum CAM_SENSOR_INDEX {
	CAMERA_INDEX_MAIN = 0,  //BackMain
	CAMERA_INDEX_FRONT,			//FrontMain
//	CAMERA_INDEX_MAIN_MACRO, //BackMacro
	CAMERA_INDEX_MAIN_WIDE,  //BackWide
	CAMERA_INDEX_MAIN_TELE,  //BackWide
	CAMERA_INDEX_MAX_NUM,
};

struct wl2864c_ldomap {
	enum CAM_SENSOR_INDEX sensor_index;
	enum msm_camera_power_seq_type seq_type;
	WL2864C_SELECT ldo_selected;
};

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
extern void wl2864c_gpio_select_state(WL2864C_GPIO_STATE s);
extern void wl2864c_set_ldo_value(WL2864C_SELECT ldonum,unsigned int value);
extern void wl2864c_set_en_ldo(WL2864C_SELECT ldonum,unsigned int en);
extern int wl2864c_set_ldo_enable(uint32_t sensor_index, enum msm_camera_power_seq_type seq_type, uint32_t min_voltage, uint32_t max_voltage);
extern int wl2864c_set_ldo_disable(uint32_t sensor_index, enum msm_camera_power_seq_type seq_type);


#endif /* _WL2864C_H_*/
