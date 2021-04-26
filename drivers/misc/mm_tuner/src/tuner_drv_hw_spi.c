/**************************************************************************//**
 *
 *  @file		tuner_drv_hw_spi.c
 *
 *  @brief		Implementation of the hardware control layer in SPI.
 *
 ****************************************************************************//*
 * Copyright (c) 2015 Socionext Inc.
 ******************************************************************************
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 ******************************************************************************/
/*
 * Copyright (C) 2019 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */
/*..+....1....+....2....+....3....+....4....+....5....+....6....+....7....+...*/
/******************************************************************************
 * include
 ******************************************************************************/
#include "tuner_drv_hw.h"

#if defined(SPI_GP_MESURE_DEBUG) || defined(DSPI_DR_MESURE_DEBUG)
#include <linux/time.h>
#endif

#include <linux/module.h>
#include <linux/spi/spi.h>

#ifdef TUNER_CONFIG_SPI_DIVMSG
#include <linux/mutex.h>
#endif

/******************************************************************************
 * function
 ******************************************************************************/
static int tuner_drv_spi_probe(struct spi_device *spidev);
static int tuner_drv_spi_remove(struct spi_device *spidev);

/* Private functions */
#ifdef TUNER_CONFIG_SPI_EDGE
static int tuner_drv_spi_edge(int ctrl);
static int tuner_drv_spi_calibration(void);
#endif

#if defined(TUNER_CONFIG_SPI_EXTREAD) || defined(TUNER_CONFIG_AES_ENABLE)
static int tuner_drv_hw_tsif_set_dmycnt(unsigned int cnt);
#endif

#define SPI_CMD_NUM 0xc
#define SPI_DATA_NUM 0xc
#define SPI_PRG_MAX_NUM 0x100
#define SPI_PSEQ_ADRS_INIT 0x00
#define SPI_BREAKCODE_PATTERN {0xff, 0xfe, 0x81, 0x00}

/******************************************************************************
 * Macro
 ******************************************************************************/
#define CALC_LENGTH(len) ((len >=  SPI_PRG_MAX_NUM) ? SPI_PRG_MAX_NUM : len)

#define SPI_TSREAD_DMYCNT 10

/* SPI configuration setting(SPI internal register address=0x08) for each mode
* SPI_DIVMSG  SPI_BREAKCODE  SPI_CONFIG_SET
* ON    OFF       0x00       : XCS_RESET=OFF , TRANSFER BREAK=DISABLE
* ON    ON        0x02       : XCS_RESET=OFF , TRANSFER BREAK=ENABLE
* OFF   OFF       0x03(0x01) : XCS_RESET=ON  , TRANSFER BREAK=DISABLE
* OFF   ON        0x03       : XCS_RESET=ON  , TRANSFER BREAK=ENABLE
*/

#if defined(TUNER_CONFIG_SPI_DIVMSG) && defined(TUNER_CONFIG_SPI_BREAKCODE)
#define SPI_CONFIG_SET (0x02)
#define CMDBUF_POS(x) (void *)(x)
#define CMDBUF_LEN(x) (x+4)
#elif defined(TUNER_CONFIG_SPI_DIVMSG) && !defined(TUNER_CONFIG_SPI_BREAKCODE)
#define SPI_CONFIG_SET (0x00)
#define CMDBUF_POS(x) (void *)(x+4)
#define CMDBUF_LEN(x) (x)
#elif !defined(TUNER_CONFIG_SPI_DIVMSG) && !defined(TUNER_CONFIG_SPI_BREAKCODE)
#define SPI_CONFIG_SET (0x03)
#define CMDBUF_POS(x) (void *)(x+4)
#define CMDBUF_LEN(x) (x)
#else
 /* elif !defined(TUNER_CONFIG_SPI_DIVMSG) &&
* defined(TUNER_CONFIG_SPI_BREAKCODE)
 */
#define SPI_CONFIG_SET (0x03)
#define CMDBUF_POS(x) (void *)(x)
#define CMDBUF_LEN(x) (x+4)
#endif

#if defined(TUNER_CONFIG_SPI_EXTREAD) || defined(TUNER_CONFIG_AES_ENABLE)
/* Extend Read command for AES */
#define SPI_READ_COMMAND  0x3b
#else
/* Normal Read command */
#define SPI_READ_COMMAND  0x0b
#endif

#if defined(TUNER_CONFIG_SPI_ALIGN)
#define BUFLEN_ALIGN(size) ((size + (TUNER_CONFIG_SPI_ALIGN - 1))&\
			(~(TUNER_CONFIG_SPI_ALIGN - 1)))
/* For example
*  TUNER_CONFIG_SPI_ALIGN = 1024
*  Expression : (size + 1023) & 0xffffffc00
*  size : 256*188=48128 :
*         (48128+1023)&0xfffffc00 = 0xbc00  = 48128 = 256*188  ( Aligned )
*  size : 344*188=64672 :
*         (64672+1023)&0xfffffc00 = 0x10000 = 65536 = 344*188 + 864
*/

#ifndef TUNER_CONFIG_SPI_DIVMSG
#error "TUNER_CONFIG_SPI_ALIGN requires TUNRE_CONFIG_SPI_DIVMSG"
#endif

#else
#define BUFLEN_ALIGN(size) (size)
#endif

#define D_TUNER_SPI_MATCH_TABLE          "socionext,mn553-spi"

/******************************************************************************
 * global
 ******************************************************************************/
#ifdef TUNER_CONFIG_SPI_DIVMSG
DEFINE_MUTEX(g_spi_mutex);
#define mtxLock() mutex_lock(&g_spi_mutex)
#define mtxUnlock() mutex_unlock(&g_spi_mutex)
#else
#define mtxLock()
#define mtxUnlock()
#endif

/******************************************************************************
 * data
 ******************************************************************************/
static const struct of_device_id spi_dt_match[] = {
	{
		.compatible = D_TUNER_SPI_MATCH_TABLE
	},
	{ },
};
MODULE_DEVICE_TABLE(of, spi_dt_match);

static struct spi_driver mn8855x_spi_driver = {
		.driver = {
				.name = "553spi",
				.owner = THIS_MODULE,
				.of_match_table = spi_dt_match,
		},
		.probe = tuner_drv_spi_probe,
		.remove = tuner_drv_spi_remove,
};

struct spi_drvdata {
	struct spi_device *spi;
	spinlock_t spi_lock;
};

static struct spi_drvdata *g_spi_drvdata;

/******************************************************************************
 * Variable
 ******************************************************************************/
#ifdef TUNER_CONFIG_SPI_EDGE
static int g_edge_mode;
#endif
/******************************************************************************
 * code area
 ******************************************************************************/
/**************************************************************************//**
 * Set TPM register
 *
 * The TPM (register of the tuner device) control the I/F port of
 * the tuner device.
 * TPM must be set to 0x02 when the Data-PATH (TS I/F) use SPI and
 * Control-PATH use I2C.
 *
 * @retval 0			Normal end
 * @retval <0			error
 ******************************************************************************/
int tuner_drv_hw_tsif_set_tpm(void)
{
	int ret = 0;
	uint8_t buf = 0x00;

	/* TPM[4:0] (PINCNT0[4:0]) */
	ret = tuner_drv_hw_read_reg(Main2, 0x00, 1, &buf);
	if (ret) {
		pr_err("Read PINCNT0, failed.\n");
		return ret;
	}
	if ((buf & 0x1F) != 0x0a) { /* NOT Diver Mode */
#ifdef CPATH_I2C
		buf = 0x02; /* CPATH:I2C, DPATH:SPI(slave-IF) */
#else
		buf = 0x00; /* CPATH:SPI, DPATH:SPI(slave-IF) */
#endif
		ret = tuner_drv_hw_write_reg(Main2, 0x00, 1, &buf);
		if (ret) {
			pr_err("write PINCNT0.TPM, failed.\n");
			return ret;
		}
	}
	return ret;
}

/**************************************************************************//**
 * address incremental read from some registers of the Tuner device.
 *
 * @retval 0			Normal end
 * @retval <0			error
 *
 * @param [in] bank	register bank enumerator
 * @param [in] adr	address of the register to read-out start
 * @param [in] len	continuous read length
 * @param [out] rd	pointer to the buffer
 ******************************************************************************/
#ifdef CPATH_SPI
int tuner_drv_hw_read_reg(enum _reg_bank bank, uint8_t adr, uint16_t len,
							uint8_t *rd)
{
	int ret = 0;
	struct spi_message msg;
	struct spi_transfer xfer;

	unsigned short loop_cnt;
	unsigned char read_data;

	/* breakcode 0xff, 0xfe, 0x81, 0x00 */
	uint8_t scmd[SPI_CMD_NUM] = { 0xff, 0xfe, 0x81, 0x00,
		0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	uint8_t sdata[SPI_DATA_NUM];
	uint8_t bank_set[3] = {
		0x90, /* Sub   0x80(I2C) + 0x10(R) + 0x00 */
		0x91, /* Main1 0x80(I2C) + 0x10(R) + 0x01 */
		0x92  /* Main2 0x80(I2C) + 0x10(R) + 0x02 */
	};
	uint8_t n = 0;

	if (g_spi_drvdata == NULL) {
		pr_debug("[%s](%d) ERR:can't spi read. g_spi_drvdata NULL",
				__func__, __LINE__);
		return -EINVAL;
	}

	if (bank > Main2) {
		pr_err("[%s](%d) Illegal bank %d\n", __func__, __LINE__, bank);
		return -EINVAL;
	}

	/* access loop */
	for (loop_cnt = 0; loop_cnt < len; loop_cnt++) {
		memset(sdata, 0x00, sizeof(unsigned char) * SPI_DATA_NUM);
		memset(&xfer, 0, sizeof(struct spi_transfer));

		spi_message_init(&msg);

		scmd[4 + 0] = 0x03;			/* command(read) */
		scmd[4 + 1] = bank_set[bank];			/* bank */
		scmd[4 + 2] = (uint8_t) (adr + n);		/* address */

		xfer.tx_buf = CMDBUF_POS(scmd);
		xfer.len = CMDBUF_LEN(8);
		xfer.bits_per_word = 8;
		xfer.rx_buf = CMDBUF_POS(sdata);

		spi_message_add_tail(&xfer, &msg);
		mtxLock();
		ret = spi_sync(g_spi_drvdata->spi, &msg);
		mtxUnlock();
		if (ret) {
			pr_debug("spi_sync() return with %d", ret);
			return ret;
		}

		/* read data */
		/* MOSI = {0x72, xx, xx, xx, RD, RD, RD, RD} */
		read_data = sdata[11];
		*(rd + n) = read_data;

		n++;
	}
#ifdef DEBUG
	{
		int i;
		char lbuf[128] = { 0 };
		char *plbuf = lbuf;

		pr_debug("SPI(R) bank:%d bankset:0x%02x offset:0x%02x len:%d\n",
				(int)bank, bank_set[(int)bank], adr, len);
		for (i = 0; i < len; i++) {
			if (0 == (i % 16)) {
				if (i)
					pr_debug("%s\n", lbuf);
				snprintf(lbuf, sizeof(lbuf)-1, "[%04x] %02x",
					i, rd[i]);
			} else {
				snprintf(lbuf, sizeof(lbuf)-1, "%s %02x",
					plbuf, rd[i]);
			}
		}
		pr_debug("%s\n", lbuf);
	}
#endif

	return ret;
}

/**************************************************************************//**
 * address incremental write to some registers of the Tuner device.
 *
 * @retval 0					Normal end
 * @retval <0					error (refer the errno)
 *
 * @param [in] bank	register bank enumerator
 * @param [in] adr	start address for continuous write
 * @param [in] len	continuous write length
 * @param [out] wd	pointer to the write data array
 ******************************************************************************/
int tuner_drv_hw_write_reg(enum _reg_bank bank, uint8_t adr, uint16_t len,
							uint8_t *wd)
{
	int ret = 0;
	struct spi_message msg;
	struct spi_transfer xfer;

	unsigned short loop_cnt;
	unsigned char write_data;

	/* breakcode 0xff, 0xfe, 0x81, 0x00 */
	uint8_t scmd[SPI_CMD_NUM] = { 0xff, 0xfe, 0x81, 0x00,
		0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	uint8_t bank_set[3] = {
			0x80, /* Sub    0x80(I2C) + 0x00(W) + 0x00 */
			0x81, /* Main1  0x80(I2C) + 0x00(W) + 0x01 */
			0x82, /* Main2  0x80(I2C) + 0x00(W) + 0x02 */
	};
	int i;
	uint8_t n = 0;

	if (g_spi_drvdata == NULL) {
		pr_debug("[%s](%d) ERR:can't spi read. g_spi_drvdata NULL",
			__func__, __LINE__);
		return -EINVAL;
	}

	if (bank > Main2) {
		pr_err("[%s](%d) Illegal bank %d\n", __func__, __LINE__, bank);
		return -EINVAL;
	}

	memset(&xfer, 0, sizeof(struct spi_transfer));

	/* access loop */
	for (loop_cnt = 0; loop_cnt < len; loop_cnt++) {
		memset(&xfer, 0, sizeof(struct spi_transfer));

		spi_message_init(&msg);

		scmd[4 + 0] = 0x03;		/* command(write) */
		scmd[4 + 1] = bank_set[bank];			/* bank */
		scmd[4 + 2] = (unsigned char) adr + n;	/* address */
		/* write data */
		write_data = *(wd + n);
		for (i = 3; i < 8; i++)
			scmd[4 + i] = write_data;

		xfer.tx_buf = CMDBUF_POS(scmd);
		xfer.len = CMDBUF_LEN(8);
		xfer.bits_per_word = 8;

		spi_message_add_tail(&xfer, &msg);
		mtxLock();
		ret = spi_sync(g_spi_drvdata->spi, &msg);
		mtxUnlock();
		if (ret) {
			pr_debug("spi_sync() return with %d", ret);
			return ret;
		}
		n++;
	}

#ifdef DEBUG
	{
		int i;
		char lbuf[128] = { 0 };
		char *plbuf = lbuf;

		pr_debug("SPI(W) bank:%d bankset:0x%02x adr:0x%02x len:%d",
				(int)bank, bank_set[(int)bank], adr, len);
		for (i = 0; i < len; i++) {
			if (0 == (i % 16)) {
				if (i)
					pr_debug("%s\n", lbuf);
				snprintf(lbuf, sizeof(lbuf)-1, "[%04x] %02x",
					i, wd[i]);
			} else {
				snprintf(lbuf, sizeof(lbuf)-1, "%s %02x",
					plbuf, wd[i]);
			}
		}
		pr_debug("%s\n", lbuf);
	}
#endif

	return ret;
}

/************************************************************************//**
 * address incremental write for PSEQ/TNCTL program download
 * to the Tuner device.
 * @retval 0					Normal end
 * @retval <0					error (refer the errno)
 *
 * @param [in] bank	register bank enumerator
 * @param [in] adr	start address for continuous write
 * @param [in] len	continuous write length
 * @param [out] wd	pointer to the write data array
 ****************************************************************************/
int tuner_drv_hw_write_prg(enum _reg_bank bank, uint8_t adr,
						uint16_t len, uint8_t *wd)
{
	int ret = 0;
	struct spi_message msg;
	struct spi_transfer xfer;
	uint8_t buf[3];
	uint8_t *p_scmd;
	uint8_t stadu = 0x00;
	uint8_t stadl = 0x00;

	int i, loop_cnt;
	uint16_t write_length = 0;
	int16_t remain_length = 0;
	uint16_t n = 0;

	const uint8_t stadu_mask[3] = {
			0x3F, /* TNCSTAD[12:8] is TNCSTADU[4:0] */
			0x00, /* (ignore) */
			0x1F  /* PSCSTAD[ */
	};
	const uint8_t dl_flg[3] = {
			0x01, /* Sub (TNCTL) */
			0xFF, /* (ignore) */
			0x00  /* Main2 (PSCTL) */
	};

	struct snglreg rmw[3] = {
		/**
		 * Upper bits of the offset address for up-loading firmware
		*/
			{ bank, 0xF4, stadu_mask[bank], 0x00, }, /* *STADU */
		/**
		 * Lower bits of the offset address for up-loading firmware
		*/
			{ bank, 0xF5, 0xFF, 0x00, }, /* *STADL */
		/**
		 * PSEQ Program Reset (*PGRST)
		*/
			{ bank, 0xF3, 0x02, 0x02, } /* *PGRST */
	};

	const uint8_t break_code[] = { 0xff, 0xfe, 0x81, 0x00, };

	if (g_spi_drvdata == NULL) {
		pr_debug("[%s](%d) ERR:can't spi read. g_spi_drvdata NULL",
			__func__, __LINE__);
		return -EINVAL;
	}

	if (bank != Main2 && bank != Sub) {
		pr_err("[%s](%d) Illegal bank %d\n", __func__, __LINE__, bank);
		return -EINVAL;
	}

	memset(&xfer, 0, sizeof(struct spi_transfer));

	/* PRTYMD setting */
	ret = tuner_drv_hw_rmw_reg(bank, 0xF2, 0x40, 0x40);	/* PRTYMD=1 */
	if (ret) {
		pr_err("Write PRTYMD of bank#%d, failed.\n", bank);
		return ret;
	}

	p_scmd = NULL;
	p_scmd = kmalloc((SPI_PRG_MAX_NUM + 4 + 4), GFP_KERNEL);
	if (p_scmd == NULL) {
		pr_err("memory allocation failed(write_prg).\n");
		return -ENOMEM;
	}
	memcpy(p_scmd, break_code, sizeof(break_code));

	/* access loop */
	loop_cnt = 0;
	remain_length = len;
	do {
		rmw[0].param = stadu + (loop_cnt & 0x00FF);
		rmw[1].param = stadl;

		for (i = 0; i < 3; i++) {
			ret = tuner_drv_hw_rmw_reg(rmw[i].bank, rmw[i].adr,
				rmw[i].enabit, rmw[i].param);
			if (ret) {
				kfree(p_scmd);
				p_scmd = NULL;
				return ret;
			}
		}

		write_length = CALC_LENGTH(remain_length);
		remain_length -= write_length;
		memset(p_scmd+8, 0x00, sizeof(unsigned char) * (write_length));
		memset(&xfer, 0, sizeof(struct spi_transfer));

		spi_message_init(&msg);

		p_scmd[4 + 0] = 0x02;				/* command */
		p_scmd[4 + 1] = dl_flg[bank];		/* 0: PSEQ / 1: TNCTL */
		/* size(upper) */
		p_scmd[4 + 2] = (write_length >> 8) & 0x00FF;
		p_scmd[4 + 3] = write_length & 0x00FF;	/* size(lower) */
		/* write data */
		for (i = 0; i < write_length; i++) {
			p_scmd[4 + i + 4] = *(wd + n);
			n++;
		}

		xfer.tx_buf = CMDBUF_POS(p_scmd);
		xfer.len = CMDBUF_LEN(write_length + 4);
		xfer.bits_per_word = 8;

		spi_message_add_tail(&xfer, &msg);
		mtxLock();
		ret = spi_sync(g_spi_drvdata->spi, &msg);
		mtxUnlock();
		if (ret) {
			pr_debug("spi_sync() return with %d", ret);
			kfree(p_scmd);
			p_scmd = NULL;
			return ret;
		}

		WARN_ON(remain_length < 0);
		if (remain_length == 0)
			break;

		loop_cnt++;
	} while (-1);

	kfree(p_scmd);
	p_scmd = NULL;

	/* Dummy read */
	ret = tuner_drv_hw_read_reg(bank, 0xF8, 1, buf);
	if (ret) {
		pr_err("RAM read mode setting fail.\n");
		return ret;
	}

	return ret;
}
#endif /* CPATH_SPI */

/**************************************************************************//**
 * Register the TS I/F driver
 *
 * @retval 0					Normal end
 * @retval <0					error
 ******************************************************************************/
int tuner_drv_hw_tsif_register(void)
{
	int ret = 0;

	pr_debug("%s\n", __func__);

	ret = spi_register_driver(&mn8855x_spi_driver);
	if (ret) {
		pr_err("spi_register_driver() failed.\n");
		return ret;
	}

	return ret;
}

/**************************************************************************//**
 * Configure the SPI-Slave I/F of the tuner device.
 *
 * @retval 0					Normal end
 * @retval <0					error
 *
 * @param [in]
 ******************************************************************************/
int tuner_drv_hw_tsif_config(struct _tsif_cntxt *tc)
{
	int ret = 0;
	struct _tuner_data_tsif *tsif;
	union _tuner_data_event iberint;
	int i;

	/* breakcode 0xff, 0xfe, 0x81, 0x00 */
	uint8_t tx[12] = { 0xff, 0xfe, 0x81, 0x00,
		0x03, 0x00, 0x08, 0x00, SPI_CONFIG_SET, 0x00, 0x00, 0x00 };
	struct spi_message msg;
	struct spi_transfer xfer;
	uint8_t pbuf_max_size = 0;
	uint8_t byte_order_set = 0;

	pr_debug("%s\n", __func__);

	ret = tuner_drv_hw_tsif_set_tpm();
	if (ret) {
		pr_err("tuner_drv_hw_tsif_set_tpm() failed.\n");
		return ret;
	}

	if (tc == NULL || tc->tsif == NULL) {
		pr_err("illegal arguments.\n");
		return -EINVAL;
	}

	tsif = (struct _tuner_data_tsif *) tc->tsif;

	/* configure the SPI(slave) I/F sub-system of Tuner device */
	memset(&xfer, 0, sizeof(struct spi_transfer));
	spi_message_init(&msg);

	xfer.tx_buf = (void *) tx;
	xfer.len = 4 + 8;
	xfer.bits_per_word = 8;
	spi_message_add_tail(&xfer, &msg);
	mtxLock();
	ret = spi_sync(g_spi_drvdata->spi, &msg);
	mtxUnlock();
	if (ret) {
		pr_err("spi_sync() failed. (return:%d).\n", ret);
		return ret;
	}

#if defined(TUNER_CONFIG_SPI_EXTREAD) || defined(TUNER_CONFIG_AES_ENABLE)
	tuner_drv_hw_tsif_set_dmycnt(SPI_TSREAD_DMYCNT);
#endif

#ifdef TUNER_CONFIG_SPI_EDGE
	tuner_drv_spi_edge(1);
	ret = tuner_drv_spi_calibration();
	if (ret) {
		pr_err("tuner_drv_spi_calibration() failed. (return:%d).\n",
				ret);
		return ret;
	}
#endif

	/* Water Line setting */
	slvif_cfgregs[SLVIF_CFG_WATERLINE].param =
	((uint8_t) (tsif->dwind[tc->bw]) << 3) | (tsif->thl[tc->bw]&0x7);

	/* Byte order configuration */
	if (tsif->spi_ts_bit_per_word == 32) {
#ifdef TUNER_CONFIG_SLV_MSBFIRST
		slvif_cfgregs[SLVIF_CFG_BYTEORDER].param = 0x00;
#else
		slvif_cfgregs[SLVIF_CFG_BYTEORDER].param = 0x40;
#endif
	} else {
		slvif_cfgregs[SLVIF_CFG_BYTEORDER].param = 0x00;
	}

	for (i = 0; slvif_cfgregs[i].bank != END_SLVCFG ; i++) {
		ret = tuner_drv_hw_rmw_reg(slvif_cfgregs[i].bank,
			slvif_cfgregs[i].adr, slvif_cfgregs[i].enabit,
			slvif_cfgregs[i].param);
		if (ret) {
			pr_err("TS slave-IF configuration, failed.\n");
			return ret;
		}
	}

	ret = tuner_drv_hw_read_reg(Main2, 0x62, 1, &pbuf_max_size);
	if (ret) {
		pr_err("Read PKTMSIZE register, failed.\n");
		return ret;
	}
	if ((pbuf_max_size & 0x0F) == 0)
		pr_debug("PKTMSIZE.MEMSIZE0[3:0] != 0x3.\n");

	/* Interrupt setting */
	/* IBERINT_F */
	iberint.pack = 0;
	iberint.set.mode = TUNER_EVENT_MODE_ADD;
	iberint.set.intdef1 = 0x80; /* IBERINT */
	iberint.set.intset1 = 0x09; /* NINTEN, INTMD = 1 */
	ret = tuner_drv_hw_setev(&iberint);
	if (ret) {
		pr_err("tuner_drv_setev(F) failed.\n");
		return ret;
	}

	/* Check byte order configuration. */
	ret = tuner_drv_hw_read_reg(Main2, 0x60, 1, &byte_order_set);
	if (ret) {
		pr_err("Main2 register read failed.\n");
		return ret;
	}
#ifdef TUNER_CONFIG_SPI_DIVMSG
	pr_debug("<SPI message division mode. [XCS_RESET: 0]>\n");
#else
	pr_debug("<SPI message stacking mode. [XCS_RESET: 1]>\n");
#endif
	pr_debug("<tsif-spi_ts_bit_per_word = %2d>\n",
		tsif->spi_ts_bit_per_word);
	pr_debug("<Hareware Byte order configuration = 0x%02x [%s First]>\n",
			 byte_order_set,
			 (byte_order_set & 0x40) ? "LSB" : "MSB");
	return 0;
}

/**************************************************************************//**
 * Unregister the TS I/F driver
 *
 ******************************************************************************/
void tuner_drv_hw_tsif_unregister(void)
{
	pr_debug("%s\n", __func__);

	spi_unregister_driver(&mn8855x_spi_driver);
}

/**************************************************************************//**
 * Get the DATAREADY flag
 *
 * This function return the DATAREADY flag of the Slave-I/F of
 * tuner device. DATAREADY flag contain OVER/UNER-Run indicator.
 * It is below there bit position.
 * OVER-Run is bit-2. UNDER-Run is bit-1, DATA-Ready is bit-0.
 *
 * @retval >=0		DATAREADY flag (casted from uint8_t)
 * @retval <0			error
 ******************************************************************************/
int tuner_drv_hw_tsif_get_dready(void)
{
	int ret;
	/* breakcode 0xff, 0xfe, 0x81, 0x00 */
	uint8_t tx[12] = { 0xff, 0xfe, 0x81, 0x00,
		0x03, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	uint8_t rx[12] = { 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	struct spi_message msg;
	struct spi_transfer xfer;

#ifdef DSPI_DR_MESURE_DEBUG
	struct timeval s1;
	struct timeval t1;
	unsigned long temp;
#endif

	memset(&xfer, 0, sizeof(struct spi_transfer));
	spi_message_init(&msg);

	xfer.tx_buf = CMDBUF_POS(tx);
	xfer.rx_buf = CMDBUF_POS(rx);
	xfer.len = CMDBUF_LEN(8);
	xfer.bits_per_word = 8;

	spi_message_add_tail(&xfer, &msg);
	mtxLock();

#ifdef DSPI_DR_MESURE_DEBUG
	do_gettimeofday(&s1);
#endif

	ret = spi_sync(g_spi_drvdata->spi, &msg);

#ifdef DSPI_DR_MESURE_DEBUG
	do_gettimeofday(&t1);
	temp = ((t1.tv_sec - s1.tv_sec) * 1000000 + (t1.tv_usec - s1.tv_usec));
	pr_err("dready() sa=%lu\n", temp);
#endif

	mtxUnlock();
	if (ret) {
		pr_err("spi_sync() failed (rc:%d)\n", ret);
		return ret;
	}

	return (int) rx[11];
}

#if defined(TUNER_CONFIG_SPI_EXTREAD) || defined(TUNER_CONFIG_AES_ENABLE)
/**************************************************************************//**
 * Set dummy cnt of SPI Extend Read
 *
 * This function set DMYCNT when use SPI EXTEND READ COMMAND for AES
 *
 * @retval >=0
 * @retval <0			error
 ******************************************************************************/
int tuner_drv_hw_tsif_set_dmycnt(unsigned int cnt)
{
	int ret;
	uint8_t tx[12] = {
		0xff, 0xfe, 0x81, 0x00, /* breakcode 0xff, 0xfe, 0x81, 0x00 */
		0x03, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00,
	};
	struct spi_message msg;
	struct spi_transfer xfer;

	memset(&xfer, 0, sizeof(struct spi_transfer));
	spi_message_init(&msg);

	xfer.tx_buf = CMDBUF_POS(tx);
	xfer.len = CMDBUF_LEN(8);
	xfer.bits_per_word = 8;

#ifdef TUNER_CONFIG_SPI_EDGE
	tx[8] = (cnt & 0xf) | 0x80;
#else
	tx[8] = cnt & 0xf;
#endif
	spi_message_add_tail(&xfer, &msg);
	mtxLock();
	ret = spi_sync(g_spi_drvdata->spi, &msg);
	mtxUnlock();
	if (ret) {
		pr_err("spi_sync() failed (rc:%d)\n", ret);
		return ret;
	}

	return ret;
}
#endif

/**************************************************************************//**
 * Send the transaction command to synchronize slave I/F of tuner.
 *
 * This function send the packet synchronization command.
 * It initialize the read pointer and clear the FIFO buffer.
 *
 * @retval 0		normal
 * @retval <0		error
 ******************************************************************************/
int tuner_drv_hw_tsif_sync_pkt(void)
{
	int ret = 0;
	/* breakcode 0xff, 0xfe, 0x81, 0x00 */
	uint8_t tx[8] = { 0xff, 0xfe, 0x81, 0x00, 0xd8, 0x00, 0x00, 0x00 };
	struct spi_message msg;
	struct spi_transfer xfer;

	pr_debug("%s\n", __func__);

	memset(&xfer, 0, sizeof(struct spi_transfer));
	spi_message_init(&msg);

	xfer.tx_buf = CMDBUF_POS(tx);
	xfer.len = CMDBUF_LEN(4);
	xfer.bits_per_word = 8;

	spi_message_add_tail(&xfer, &msg);
	mtxLock();
	ret = spi_sync(g_spi_drvdata->spi, &msg);
	mtxUnlock();
	if (ret) {
		pr_err("spi_sync() failed (rc:%d)\n", ret);
		return ret;
	}

	return 0;
}

/**************************************************************************//**
 * Get the TS packets of the appointed number.
 *
 * @retval >=0		Normal end (number of the get packet)
 * @retval <0			error (refer the errno)
 *
 * @param [in] num			num of packets
 * @param [out] pktbuf		packet storage
 * @param [in] pktsize		packet size enumerator
 ******************************************************************************/
int tuner_drv_hw_tsif_get_pkts(struct _tsif_cntxt *tc)
{
	int ret;
	struct spi_message msg;
#ifdef TUNER_CONFIG_SPI_DIVMSG
	struct spi_message tsmsg;
#endif
	struct spi_transfer xfer[2];
	int sum = 0;
	struct _tuner_data_tsif *tsif = tc->tsif;
	unsigned int ts_rdelay = 0;

	/* TS packet size: 188Byte, num of TS packets:256 */
	/* Break code0xff, 0xfe, 0x81, 0x00 + Packet Read commnad */
	uint8_t tx[9 + 17] = { 0xff, 0xfe, 0x81, 0x00, /* 4 byte length */
	/* 5 byte length command ex. 0x0b .. */
	SPI_READ_COMMAND, 0x00, 0x00, 0xFF, 0x00,
	/* This 16 bytes is dummy cycles */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* for extend read command */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* Last byte is for additional cycle in edge mode and
	 * extend read is enabled
	*/
	0x00 };
#ifdef SPI_GP_MESURE_DEBUG
	struct timeval s1;
	struct timeval t1;
	unsigned long temp;
#endif

#if defined(TUNER_CONFIG_SPI_EXTREAD) || defined(TUNER_CONFIG_AES_ENABLE)
	ts_rdelay = ts_rdelay + SPI_TSREAD_DMYCNT;
#endif  /* TUNER_CONFIG_SPI_EXTREAD */
#ifdef TUNER_CONFIG_SPI_EDGE
	ts_rdelay = ts_rdelay + 1;  /* EDGE==1 */
#endif  /* TUNER_CONFIG_SPI_EDGE */

	if (!g_spi_drvdata) {
		pr_err("SPI I/F not active.\n");
		return -ENXIO;
	}
	if (!tc->pktbuf) {
		pr_err("TS buffer not found.\n");
		return -EINVAL;
	}
	if (tc->tsif->ts_pkt_type == TUNER_DRV_TS_TSTAMP) {
		pr_err("not support the Time-Stamp TS");
		return -EINVAL;
	}
	if (!tc->spibuf) {
		pr_err("SPI buf not found.\n");
		return -EINVAL;
	}
	memset(tc->spibuf, 0, tc->ts_rx_size);

	memset(xfer, 0, sizeof(xfer));
	spi_message_init(&msg);

	tx[4 + 2] = (tc->ts_rxpkt_num - 1) >> 8;
	tx[4 + 3] = (tc->ts_rxpkt_num - 1) & 0xff;
	/* break | read mode | edge mode ||  ts_rdelay(=edge delay+DMYCNT delay)
	*       |   nomral  |    off    ||   0
	*       |   nomral  |    on     ||   1
	*  on   |   nomral  |    off    ||   0
	*  on   |   nomral  |    on     ||   1
	*       |   extend  |    off    ||   DMYCNT
	*       |   extned  |    on     ||   DMYCNT+1
	*  on   |   extend  |    off    ||   DMYCNT
	*  on   |   extend  |    on     ||   DMYCNT+1
	*/
	xfer[0].tx_buf = CMDBUF_POS(tx);
	xfer[0].len = CMDBUF_LEN(5) + ts_rdelay;
	xfer[0].bits_per_word = 8;

	spi_message_add_tail(&xfer[0], &msg);

	xfer[1].rx_buf = (void *) (tc->spibuf);
	xfer[1].bits_per_word = tsif->spi_ts_bit_per_word;
#ifdef TUNER_CONFIG_SPI_DIVMSG
	xfer[1].len = BUFLEN_ALIGN(tc->ts_rx_size);
	spi_message_init(&tsmsg);
	spi_message_add_tail(&xfer[1], &tsmsg);
	mtxLock();
	ret = spi_sync(g_spi_drvdata->spi, &msg);
	if (ret) {
		pr_err("spi_sync() failed (rc:%d)\n", ret);
		mtxUnlock();
		return ret;
	}

	ret = spi_sync(g_spi_drvdata->spi, &tsmsg);
	mtxUnlock();
	if (ret) {
		pr_err("spi_sync() failed (rc:%d)\n", ret);
		return ret;
	}
#else
	xfer[1].len = tc->ts_rx_size;
	spi_message_add_tail(&xfer[1], &msg);

	mtxLock();

#ifdef SPI_GP_MESURE_DEBUG
	do_gettimeofday(&s1);
#endif

	ret = spi_sync(g_spi_drvdata->spi, &msg);

#ifdef SPI_GP_MESURE_DEBUG
	do_gettimeofday(&t1);
	temp = ((t1.tv_sec - s1.tv_sec) * 1000000 + (t1.tv_usec - s1.tv_usec));
	pr_err("spi_sync() sa=%lu\n", temp);
#endif

	mtxUnlock();
	if (ret) {
		pr_err("spi_sync() failed (rc:%d)\n", ret);
		return ret;
	}
#endif

	/* Plase add the code to transform endian , if you need */
	/* ---------------------------------------------------- */

	/* ---------------------------------------------------- */

	memcpy((void *) (tc->pktbuf + tc->pwr),
	(void *) (tc->spibuf), tc->ts_rx_size);
	tc->pwr += tc->ts_rx_size;

	if (tc->pwr == tc->ts_pktbuf_size)
		tc->pwr = 0;

	return sum;
}

/**************************************************************************//**
 * probe function called by spi_register_driver()
 *
 * @retval 0			Normal end
 * @retval <0			error (refer the errno)
 *
 * @param [in] spidev	pointer to the "spi_device" structure
 ******************************************************************************/
static int tuner_drv_spi_probe(struct spi_device *spidev)
{
	int ret = 0;
	struct spi_drvdata *drvdata;

	pr_debug("%s\n", __func__);

	if (g_spi_drvdata != NULL) {
		pr_err("SPI I/F not active.\n");
		return -EBUSY;
	}
	if (spidev == NULL) {
		pr_err("illegal argument.\n");
		return -EINVAL;
	}

	drvdata = kzalloc(sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->spi = spidev;
	spin_lock_init(&drvdata->spi_lock);
	spi_set_drvdata(spidev, drvdata);
	g_spi_drvdata = drvdata;

	ret = spi_setup(spidev);
	if (ret) {
		pr_err("spi_setup() failed.\n");
		return ret;
	}
#ifdef DEBUG
	pr_info("max_speed_hz   :%d\n", spidev->max_speed_hz);
	pr_info("chip_select    :%d\n", spidev->chip_select);
	pr_info("mode           :%d\n", spidev->mode);
	pr_info("bits_per_word  :%d\n", spidev->bits_per_word);
	pr_info("irq            :%d\n", spidev->irq);
	pr_info("modalias       :%s\n", spidev->modalias);
#endif
	return ret;
}

/**************************************************************************//**
 * remove function called by spi_register_driver()
 *
 * @retval 0			Normal end
 * @retval <0			error (refer the errno)
 *
 * @param [in] spidev	pointer to the "spi_device" structure
 ******************************************************************************/
static int tuner_drv_spi_remove(struct spi_device *spidev)
{
	pr_debug("%s\n", __func__);

	spi_set_drvdata(spidev, NULL);
	kfree(g_spi_drvdata);
	g_spi_drvdata = NULL;

	return 0;
}

/**************************************************************************//**
 * TS read Calibration
 *
 * @retval 0		normal
 * @retval <0		error
 ******************************************************************************/
#ifdef TUNER_CONFIG_SPI_EDGE
int tuner_drv_spi_calibration(void)
{
	int ret = 0;
	/* breakcode 0xff, 0xfe, 0x81, 0x00 */
	uint8_t tx1[7 + 32] = { 0xff, 0xfe, 0x81, 0x00,
	0x4b, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00 }; /* Calibration command */
	uint8_t rx1[7 + 4 * 8] = { 0x00 }; /* Read calib. result */
	/* breakcode 0xff, 0xfe, 0x81, 0x00 */
	uint8_t tx2[12] = { 0xff, 0xfe, 0x81, 0x00,
	0x03, 0x00, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00 }; /* Delay set */
	struct spi_message msg;
	struct spi_transfer xfer[3];
	int i;
	int sp, ep, pos;

	pr_debug("%s\n", __func__);

	memset(xfer, 0, sizeof(xfer));
	spi_message_init(&msg);

	/* Calibration Command */
	xfer[0].tx_buf = CMDBUF_POS(tx1);
	xfer[0].rx_buf = CMDBUF_POS(rx1);
	xfer[0].len = CMDBUF_LEN(3+32);
	xfer[0].bits_per_word = 8;
	spi_message_add_tail(&xfer[0], &msg);

	mtxLock();
	ret = spi_sync(g_spi_drvdata->spi, &msg);
	mtxUnlock();
	if (ret) {
		pr_err("spi_sync() failed (rc:%d)\n", ret);
		return ret;
	}

	pr_debug("<SPI calibration>\n");
	for (i = 7; i < 39; i += 4) {
		pr_debug(
		" (%1d) rx[%2d]:%02X, rx[%2d]:%02X, rx[%2d]:%02X, rx[%2d]:%02X\n",
		(int)((i-7)/4), i+0, rx1[i+0], i+1, rx1[i+1], i+2, rx1[i+2],
		i+3, rx1[i+3]);
	}

	/* Calculation delay */
	sp = 0;
	ep = 0;
	for (i = 0; i < 8; i++) {
		if (rx1[4 + 5 + i * 4] == 0x72) {
			if (sp == 0)
				sp = i + 1;
			else
				ep = i + 1;
		}
	}

	/* Delay set */
	pos = (int) ((sp + ep) / 2);
	tx2[4 + 4] = pos;
	pr_debug("sp(%d),ep(%d)==>pos: %d (edge_mode:%d)\n",
			sp, ep, pos, g_edge_mode);

	memset(xfer, 0, sizeof(xfer));
	spi_message_init(&msg);
	xfer[2].tx_buf = CMDBUF_POS(tx2);
	xfer[2].len = CMDBUF_LEN(8);
	xfer[2].bits_per_word = 8;
	spi_message_add_tail(&xfer[2], &msg);
	mtxLock();
	ret = spi_sync(g_spi_drvdata->spi, &msg);
	mtxUnlock();
	if (ret) {
		pr_err("spi_sync() failed (rc:%d)\n", ret);
		return ret;
	}

	if (sp == 0) {
		pr_err("spi_calibration() failed. sp = %d", sp);
		return -EAGAIN;
	}
	return 0;
}
#endif

/**************************************************************************//**
 * EDGE mode setting
 *
 * @retval 0            normal
 * @retval <0           error
 ******************************************************************************/
#ifdef TUNER_CONFIG_SPI_EDGE
int tuner_drv_spi_edge(int ctrl)
{
	int ret = 0;
	/* breakcode 0xff, 0xfe, 0x81, 0x00 */
	uint8_t tx[12] = { 0xff, 0xfe, 0x81, 0x00, 0x03, 0x00, 0x0c, 0x00, 0x00,
			0x00, 0x00, 0x00 };
	struct spi_message msg;
	struct spi_transfer xfer;

	pr_debug("%s\n", __func__);

	memset(&xfer, 0, sizeof(struct spi_transfer));
	spi_message_init(&msg);

	tx[4 + 4] = (ctrl << 7) | SPI_TSREAD_DMYCNT;

	xfer.tx_buf = CMDBUF_POS(tx);
	xfer.len = CMDBUF_LEN(8);
	xfer.bits_per_word = 8;
	spi_message_add_tail(&xfer, &msg);
	mtxLock();
	ret = spi_sync(g_spi_drvdata->spi, &msg);
	mtxUnlock();
	if (ret) {
		pr_err("spi_sync() return with %d", ret);
		return ret;
	}
	g_edge_mode = ctrl;
	pr_debug("SPI EDGE mode (%d)", g_edge_mode);
	return 0;
}
#endif

/*******************************************************************************
 * Copyright (c) 2015 Socionext Inc.
 ******************************************************************************/
