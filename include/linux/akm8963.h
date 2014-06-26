/*
 * Definitions for akm8963 compass chip.
 */
#ifndef AKM8963_H
#define AKM8963_H

#define AKM8963_I2C_NAME "akm8963"

/* \name AK8963 operation mode
 \anchor AK8963_Mode
 Defines an operation mode of the AK8963.
*/
#define AK8963_MODE_SNG_MEASURE		0x01
#define AK8963_MODE_CNT_NORMAL		0x02
#define AK8963_MODE_CNT_FASTEST		0x06
#define	AK8963_MODE_SELF_TEST		0x08
#define	AK8963_MODE_FUSE_ACCESS		0x0F
#define	AK8963_MODE_POWERDOWN		0x00
#define AK8963_MODE_RESET			0x01
#define AK8963_MODE_14BIT			0x00
#define AK8963_MODE_16BIT			0x10

#define SENSOR_DATA_SIZE	8	/* Rx buffer size */
#define RWBUF_SIZE			16	/* Read/Write buffer size.*/
#define FUSEROM_SIZE		3   /* Read only fuse ROM area size. */

/* \name AK8963 register address
 \anchor AK8963_REG
 Defines a register address of the AK8963.
*/
#define AK8963_REG_WIA		0x00
#define AK8963_REG_INFO		0x01
#define AK8963_REG_ST1		0x02
#define AK8963_REG_HXL		0x03
#define AK8963_REG_HXH		0x04
#define AK8963_REG_HYL		0x05
#define AK8963_REG_HYH		0x06
#define AK8963_REG_HZL		0x07
#define AK8963_REG_HZH		0x08
#define AK8963_REG_ST2		0x09
#define AK8963_REG_CNTL1	0x0A
#define AK8963_REG_CNTL2	0x0B
#define AK8963_REG_ASTC		0x0C
#define AK8963_REG_TS1		0x0D
#define AK8963_REG_TS2		0x0E
#define AK8963_REG_I2CDIS	0x0F

#define AK8963_DEVICE_ID		0x48
#define AK8963_ST1_NORMAL		0x00
#define AK8963_ST1_DATA_READY	0x01

#define UCHAR_MIN	0
#define UCHAR_MAX	255

/* \name AK8963 fuse-rom address
 \anchor AK8963_FUSE
 Defines a read-only address of the fuse ROM of the AK8963.
*/
#define AK8963_FUSE_ASAX	0x10
#define AK8963_FUSE_ASAY	0x11
#define AK8963_FUSE_ASAZ	0x12

struct akm8963_platform_data {
	char layouts[3][3];
	int (*setup)(void);
	void (*shutdown)(void);
	void (*hw_config)(int enable);
	/*********/
	char layout;
	char outbit;
	int gpio_DRDY;
	int gpio_RST;
	int irq_gpio;
	u32 irq_gpio_flags;
};

#endif

