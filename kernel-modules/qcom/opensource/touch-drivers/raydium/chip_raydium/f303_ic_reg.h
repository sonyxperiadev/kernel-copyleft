/* f303_ic_reg.h
 *
 * Raydium TouchScreen driver.
 *
 * Copyright (c) 2021  Raydium tech Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

 // ['h5000_0900], [32'hFFFF_FFFF], PRAM_LOCK
#define I2CTB_LOCK                          (0x00000001<<6)      // [0] (R/W) I2C Table lock
#define BOTLR_LOCK                          (0x00000001<<5)      // [0] (R/W) Boot loader lock
#define USEFW_LOCK                          (0x00000001<<4)      // [0] (R/W) User fw lock
#define CONFIG_LOCK                         (0x00000001<<3)      // [0] (R/W) Configuration lock
#define COMP_LOCK                           (0x00000001<<2)      // [0] (R/W) compensation lock
#define BASEL_LOCK                          (0x00000001<<1)      // [0] (R/W) baseline lock
#define INICO_LOCK                          (0x00000001<<0)      // [0] (R/W) Initial code lock

// ['h5000_0904], [32'h0000_0000], Program RAM store type, PRAM_STORE_TYPE
#define BOTLR_AREA                          (0x00000001<<5)      // [0] (R/W) Boot loader  area
#define USEFW_AREA                          (0x00000001<<4)      // [0] (R/W) User fw area
#define CONFIG_AREA                         (0x00000001<<3)      // [0] (R/W) Configuration area
#define COMP_AREA                           (0x00000001<<2)      // [0] (R/W) compensation area
#define BASEL_AREA                          (0x00000001<<1)      // [0] (R/W) baseline area
#define INICO_AREA                          (0x00000001<<0)      // [0] (R/W) Initial code area

// ['h5000_0918], [32'h0000_0000],Flash state control register ,FLASH_STATE_REG
#define BLDR_FINISH							(0x00000001<<14)	// [0] (R) MCU_HOLD status
#define MCU_HOLD_STATUS						(0x00000001<<13)	// [0] (R) MCU_HOLD status
#define BOOT_REGION							(0x00000001<<12)	// [0] (R) boot region index
#define BL_CRC_CHK							(0x00000001<<11)	// [0] (R/W) boot-loader area CRC check
#define FW_CRC_CHK							(0x00000001<<10)	// [0] (R/W) user FW CRC check
#define PARA_CRC_CHK						(0x00000001<<9)		// [0] (R/W) parameter area CRC check
#define COMP_CRC_CHK						(0x00000001<<8)		// [0] (R/W) compensation area CRC check
#define BASELINE_CRC_CHK					(0x00000001<<7)		// [0] (R/W) baseline area CRC check
#define INITIAL_CRC_CHK						(0x00000001<<6)		// [0] (R/W) initial code CRC check
#define MCU_HOLD							(0x00000001<<5)		// [0] (R/W) MCU hold
#define SKIP_LOAD  							(0x00000001<<4)		// [0] (R/W) Skip all load flash action
#define FW_INICO_ERR						(0x00000001<<3)		// [0] (R) FW_INICO_ERR
#define FW_CG_ERR    						(0x00000001<<2)		// [0] (R) cc bl CRC error
#define FW_CRC_ERR							(0x00000001<<1)		// [0] (R) FW_CRC_ERR
#define BL0_CRC_ERR							(0x00000001<<0)		// [0] (R) BL0_CRC_ERR

// ['h5000_0934], [32'h0000_0000], FLASH Lock and Key Register (main) (FLKEY1)
#define FLKEY1_LOCK                     	(0x00000000<<0)      // [7:0] (R/W) FLASH Lock and Key1 Register
#define FLKEY1_KEY                      	(0x000000A5<<0)      // [7:0] (R/W) FLASH Unlock and Key1 Register:0xA5

// ['h5000_0938], [32'h0000_0000], FLASH Lock and Key Register (information) (FLKEY2)
#define FLKEY2_LOCK                     	(0x00000000<<0)      // [7:0] (R/W) FLASH Lock and Key3 Register
#define FLKEY2_KEY                      	(0x000000D7<<0)      // [7:0] (R/W) FLASH Unlock and Key3 Register:0xD7

//#define MAX_SENSING_PIN_NUM  				30

// ['h5000_0610], [32'h0106_0300], I2C eng Register (I2CENG)
#define REG_I2CENG_ADDR						0x50000610
#define I2CENG_AUTO_I2C_DGF_MODE2			(0x00000001<<26)					// [26:26]		(R/W)	auto_i2c_dgf_mode2,, 1: switch def_en after next posedge SCL_dgf, 0: switch def_en control auto_i2c_dgf_mode (bit[24]).
#define I2CENG_DGF_OSC_AUTOSEL				(0x00000001<<25)					// [25:25]		(R/W)	dgf_osc_autosel,, 0: TP OSC control, 1: TP OSC/DRIVER OSC control
#define I2CENG_AUTO_I2C_DGF_MODE				(0x00000001<<24)					// [24:24]		(R/W)	dgf_en signal delay timing select 0: switch def_en after STOP signal 1: switch def_en after next posedge SCL.
#define I2CENG_I2CS_DGFEN_DLY_NUM(u7x)		((u7x&0x0000007F)<<16)				// [22:16]		(R/W)	dgf_en signal delay number (clock by system clock ).
#define I2CENG_I2CS_DEGFIR_NUM(u7x)			((u7x&0x0000007F)<<8)				// [14:8]		(R/W)	I2C Pad (SCL/SDA) deglitch filter number.
#define I2CENG_RB_MANUAL_I2C_DGF				(0x00000001<<7)						// [7:7]		(R/W)	I2C Pad (SCL/SDA) deglitch filter manual mode control 1: Enable 0: Disable.
#define I2CENG_RB_I2C_DGF_EN					(0x00000001<<6)						// [6:6]		(R/W)	I2C Pad (SCL/SDA) deglitch filter enable control register when rb_manual_i2c_dgf =1, 1: Enable 0: Disable.
#define I2CENG_FIRST_DAT_SEL(u2x)				((u2x&0x00000003)<<0)				// [1:0]		(R/W)	First data request select for PDA2 read. 0: at 2nd SCL; 1: at 3th SCL; 2:at 4th SCL; 3:at 5th SCL.

#define REG_I2C_I2CFLASHPRO					0x50000624

//['h5000_0628], [32'h0000_0000], PDA2 Control Register (PDA2CTL)
#define REG_PDA2CTL_ADDR						0x50000628
#define PDA2CTL_PDA2_EN						(0x00000001UL<<2)					// [2:2]		(R/W)	PDA2 enable bit. 1: enable 0: disable.
#define PDA2CTL_SIE2							(0x00000001<<1)						// [1:1]		(R/W)	SIE2 enable register . 1: enable 0: mask.
#define PDA2CTL_SI2								(0x00000001<<0)						// [0:0]		(R/W)	SI2 interrupt flag (write 1 to clear).


//['h5000_0E1C], [32'h0000_0000], GPIO deglitch enable(GPIO_DEGLITCH)
#define REG_GPIO_DEGLITCH_ENABLE				0x50000E1C
#define GPIO_PULLH_EN(u2x)						((u2x&0x00000003)<<0)				// [3:2]		(R/W)	1: enable pull-high of GPIO, 0: disable pull-high of GPIO
#define GPIO_DEGLITCH_EN(u2x)					((u2x&0x00000003)<<0)				// [1:0]		(R/W)	1: enable deglitch function of GPIO, 0: disable deglitch function of GPIO

#define REG_SYSCON_BLKEN_ADDR				0x40000000
#define REG_SYSCON_BLKRST_ADDR				0x40000004
//#define REG_SYSCON_MISCIER_ADDR				0x40000014

#define REG_T2D_CONFIG_1					0x5000145c
#define REG_T2D_CONFIG_2					0x50001460
#define REG_T2D_CONFIG_3					0x50001464
#define REG_T2D_R_CONFIG_1					0x50001468
#define REG_T2D_R_CONFIG_2					0x5000146C

#define MCU_HOLD                    		(0x00000001<<5)		// [0] (R/W) MCU hold
#define SKIP_LOAD                   		(0x00000001<<4)		// [0] (R/W) Skip all load flash action
#define BLKRST_SW_RST               		(0x00000001<<0)      	// [0] (R/W) 1: Software reset, all digital block will be reset
#define MCU_HOLD_STATUS             		(0x00000001<<13)	// [0] (R) MCU_HOLD status
#define FLH_RELEASE_PD              		(0x00000001<<5)      	// [0] (R/W) Release from deep power down mode
#define BL_CRC_CHK                  		(0x00000001<<11)	// [0] (R/W) boot-loader area CRC check

/* Base addresses                                                             */
#define RM_PRAM_BASE                    	(0x00000000UL)             // Program, AHB
#define RM_RAM_BASE                     	(0x20000000UL)             // SRAM, AHB
#define RM_AHB_BASE                     	(0x40000000UL)             // Peripheral, AHB
#define RM_APB_BASE                     	(0x50000000UL)             // Peripheral, APB

//#define FW_SYS_CMD_ADDR             		0x20000288
#define FW_FT_CMD_ADDR              		0x20000289
#define FW_FT_ARG0_ADDR             		0x2000028A
#define FW_FT_ARG1_ADDR             		0x2000028C
//#define FW_FT_IMG_ADDR              		0x2000019C
//#define FW_TP_SEQ_NUM_ADDR          		0x20000290

#define SYS_CMD_FUNC_DIS_BS_UPDATE  		0x20
#define DIS_BASELINE_UPDATE         		0x00010000

#define SYS_CMD_DO_BL_CAL                   0x5A
#define SYS_CMD_READ_CAL_FLAG               0x5B
#define SYS_CMD_DO_CC_CAL                   0x5C
#define SYS_CMD_CAL_WAIT                    0x5D

#define SYS_CMD_WAKEUP_GESTURE_ENABLE       0x40
#define SYS_CMD_WAKEUP_GESTURE_DISABLE      0x41

#define SYS_CMD_FT_GET_DSP_NS_PARAM         0x60
#define SYS_CMD_FT_FUN_FLAG                 0x62
#define SYS_CMD_FT_DC_DISABLE          	 	0x010000
#define SYS_CMD_FT_DIG_GAIN_ENABLE      	0x020000
#define SYS_CMD_FT_TEST_LOG_EN          	0x800000

#define FW_FT_CHANNEL_X_ADDR        		(PRAM_PARA_START + 24)
#define FW_FT_CHANNEL_Y_ADDR        		(PRAM_PARA_START + 25)
#define FW_FT_PIN_ADDR              		(PRAM_PARA_START + 27)
#define FW_FT_PWR_MODE_ADDR         		(PRAM_PARA_START + 26)
#define FW_FT_FW_VERSION            		(PRAM_PARA_START + 4)
#define FW_FT_SRAM_FW_VERSION       		0x200006E0


#define BOOT_SYNC_DATA_ADDR					0x20000200
#define BOOT_MAIN_STATE_ADDR				0x20000204
#define BOOT_NORMAL_STATE_ADDR				0x20000208
#define BOOT_BURNING_STATE_ADDR				0x2000020C
#define BOOT_CMD_TYPE_ADDR					0x20000210
#define BOOT_RET_DATA_ADDR					0x20000214
#define BOOT_TEST_MODE_ADDR					0x20000218

#define FLASH_OFFSET                    	(0x7800)
#define PRAM_BASELINE_LENGTH            	(0x130)
#define PRAM_COMP_LENGTH                	(0x304)
#define PRAM_DIS_INIT_LENGTH            	(0x80)
#define PRAM_PARA_LENGTH                	(0x174)
#define PRAM_FW_LENGTH                  	(0x7300)
#define PRAM_BOOT_LENGTH                	(0x800)

#define PRAM_BOOT_START                 	(0x0000)
#define PRAM_DIS_INIT_START             	(0x7F80)
#define PRAM_COMP_START                 	(0x7C78)
#define PRAM_PARA_START	                	(0x7B00)
#define PRAM_FW_START                   	(0x0800)
#define PRAM_RESERVE_START              	(0x0800)
#define PRAM_PARA_DC_THD_ADDR           	(PRAM_PARA_START + 136)
#define PRAM_CC_TABLE_ADDR              	(0x7F78)

#define PRAM_BASEINE_START 					(0x6CCC)
#define PRAM_BOOT_CRC_LENGTH				(PRAM_BOOT_LENGTH - HEADER_LENGTH)
#define PRAM_FW_CRC_START					(0x6B5C)
#define PRAM_FW_CRC_LENGTH					(PRAM_FW_LENGTH + PRAM_PARA_LENGTH)	//0x7474
#define PRAM_CB_CRC_START					(0x6DFC)
#define PRAM_CB_CRC_LENGTH					(PRAM_COMP_LENGTH + PRAM_BASELINE_LENGTH)


#define FT_RAWDATA1_SHORT_BUF_ADDR  		0x200002E4                        //((at(0x200002E4)));
#define FT_RAWDATA2_OPEN_BUF_ADDR   		(FT_RAWDATA1_SHORT_BUF_ADDR+100)     //((at(0x20000348)));
#define FT_RAWDATA3_CC_BUF_ADDR     		(FT_RAWDATA2_OPEN_BUF_ADDR+100)      //((at(0x200003AC)));
#define FT_UC_BUF_ADDR              		(FT_RAWDATA3_CC_BUF_ADDR+100)        //((at(0x20000410)));

#define FT_OPEN_BL_BUF_ADDR         		(FT_UC_BUF_ADDR+100)                 //((at(0x20000474)));
#define FT_TEST_RESULT_BUF_ADDR     		(FT_OPEN_BL_BUF_ADDR+100)           //((at(0x200004D8)));
#define FT_TEST_ITEM_RESULT         		(FT_TEST_RESULT_BUF_ADDR+50+2)       //((at(0x2000050C)));
#define FT_TEST_INFO_ADDR           		0x20000674//FT_IMG2PIN_BUF_ADDR+72//((at(0x20000674)));
#define FT_TEST_THD_ADDR            		(FT_TEST_INFO_ADDR+16)              //((at(0x20000684)));
#define FT_TEST_PARA_ADDR           		(FT_TEST_THD_ADDR+36)               //((at(0x200006A8)));


#define SRAM_FT_RAWDATA_3_CC_ADDR       	(RM_RAM_BASE + 0x0000074C)	//0x2000074C
#define SRAM_FT_UC_CC_ADDR              	(RM_RAM_BASE + 0x000006EC)	//0x200006EC

#define FLASH_NORMAL_FW_FW_VERSION_ADDR     (FW_FT_FW_VERSION)
#define FLASH_NORMAL_FW_CUST_VERSION_ADDR   (PRAM_PARA_START + 10)
#define FLASH_TEST_FW_FW_VERSION_ADDR       (FW_FT_FW_VERSION + FLASH_OFFSET)
#define FLASH_NORMAL_FW_CC_TABLE_ADDR       0x9300

#define REG_FLASHCTL_FLASH_PRAM_LOCK		0X50000900
#define REG_FLASHCTL_FLASH_PRAM_STORE_TYPE 	0X50000904
#define REG_FLASHCTL_FLASH_PRAM_ADDR		0X50000908
#define REG_FLASHCTL_FLASH_PRAM_LENGTH 		0X5000090C
#define REG_FLASHCTL_FLASH_ADDR				0X50000910
#define REG_FLASHCTL_FLASH_ISPCTL			0X50000914
#define REG_FLASHCTL_FLASH_STATE_REG_ADDR	0x50000918
#define REG_FLASHCTL_FLASH_FLKEY1 			0x50000934
#define REG_FLASHCTL_FLASH_FLKEY2 			0x50000938
#define REG_FLASHCTL_FLASH_DATA				0x5000093C
#define REG_FLASHCTL_FLASH_ENG3				0x5000094C
#define REG_FLASHCTL_FLASH_PRGCHKSUM_ADDR 	0x50000974
#define REG_FLASHCTL_FLASH_PRGCHKSUM_RESULT 0x50000978
#define REG_FLASHCTL_DEVID_ADDR				0x500009BC

#define REG_SPI_SLAVE_SPIFLASHPRO			0x50000524

#define PRAM_ADDR_CC_INFO					0x00007F78

#define RAM_WRITE_TEST_ADDR1 				0x50000950
#define RAM_WRITE_TEST_ADDR2 				0x50000B10
#define RAM_WRITE_TEST_ADDR3 				0x50000B00
#define RAM_READ_TEST_ADDR1					0x50000954
#define RAM_READ_TEST_ADDR2					0x50000B04
#define RAM_READ_TEST_ADDR3					0x50000B08

#define FT_UPDATE                   0x01
#define FT_BASELINE_SF              0x02
#define FT_BASELINE_PS              0x42
#define FT_COMPENSATION_SF          0x04
#define FT_COMPENSATION_PS          0x44
#define FT_RAWDATA_W_BL_SF          0x08
#define FT_RAWDATA_W_BL_PS          0x48
#define FT_RAWDATA_WO_BL_SF         0x10
#define FT_RAWDATA_WO_BL_PS		    0x50
#define FT_ALG_RAWDATA              0x20
#define FT_PS_SEL          			0x40
#define FT_DEBUG_MESSAGE            0x80
#define FT_UPDATE_CASE              0xFE

#define FT_STATUS_PURE_RAW          0x01
#define FT_STATUS_PURE_AF_DC        0x02

typedef enum {
	CRC_CHECK_FAIL = 0x80,
	CRC_CHECK_PASS = 0x81,
	WAIT_TEST_MODE = 0x82,
	PARTITION_CRC = 0xA0,
	SET_ADDR_READY = 0xA1,
	SET_ADDR_FAIL = 0xA2,
	WRT_PRAM_DATA = 0xA3,
	WRT_PRAM_FAIL = 0xA4,
	WAIT_WRT_ACK = 0xA5,
	WAIT_ACK_FAIL = 0xA6,
	GET_WRT_ACK = 0xA7,
	GET_WRT_UNLOCK = 0xA8,
	GET_UNLOCK_FAIL = 0xA9,
} I2C_SYNC_CMD;

typedef enum {
	MAIN_STATE_NORMAL_MODE = 0,
	MAIN_STATE_BURNING_MODE,
	MAIN_STATE_FIRWARE_MODE,
} TCH_BOOTLOADER_STATE;

typedef enum {
	BURNING_STATE_INIT = 0,
	BURNING_CHECK_ADDR,
	BURNING_UNLOCK_PRAM,
	BURNING_WRT_PRAM,
	BURNING_WRT_FLASH_PREPARE,
	BURNING_WRT_FLASH_EXCUTE,
	BURNING_WRT_FLASH_FINISH,
	BURNING_STATE_HALT,
} BL_BURNING_STATE;

typedef enum {
	NORMAL_STATE_CHECK = 0,
	NORMAL_CRC_CALC,
	NORMAL_CRC_NOTIFY,
	NORMAL_FW_CRC,
	NORMAL_MODE_CHANGE,
	NORMAL_MODE_IDLE,
} BL_NORMAL_STATE;

