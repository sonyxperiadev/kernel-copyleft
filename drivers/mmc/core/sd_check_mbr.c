/*********************************************************************
 *  ____                      _____      _                           *
 * / ___|  ___  _ __  _   _  | ____|_ __(_) ___ ___ ___  ___  _ __   *
 * \___ \ / _ \| '_ \| | | | |  _| | '__| |/ __/ __/ __|/ _ \| '_ \  *
 *  ___) | (_) | | | | |_| | | |___| |  | | (__\__ \__ \ (_) | | | | *
 * |____/ \___/|_| |_|\__, | |_____|_|  |_|\___|___/___/\___/|_| |_| *
 *                    |___/                                          *
 *                                                                   *
 *********************************************************************/
/* [linux/drivers/mmc/core/sd_check_mbr.c]
 *
 * Copyright (C) [2011] Sony Ericsson Mobile Communications AB.
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * Author: Daisuke Okitsu <daisuke.x.okitsu@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */


#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include "core.h"
#include "sd_check_mbr.h"

#define SD_BLOCK_SIZE 512

/*
 * Configure correct block size in card
 */
static int mmc_sd_set_blksize(struct mmc_card *card, unsigned size)
{
	int ret = 0;
	struct mmc_command cmd;

	if (!(mmc_card_blockaddr(card))) {
		cmd.opcode = MMC_SET_BLOCKLEN;
		cmd.arg = size;
		cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;

		ret = mmc_wait_for_cmd(card->host, &cmd, 0);
	}

	return ret;
}

/*
 * Fill in the mmc_request structure given a set of transfer parameters.
 */
static void mmc_sd_prepare_mrq(struct mmc_card *card,
	struct mmc_request *mrq, struct scatterlist *sg, unsigned sg_len,
	unsigned dev_addr, unsigned blocks, unsigned blksz)
{
	mrq->cmd->opcode = MMC_READ_SINGLE_BLOCK;

	mrq->cmd->arg = dev_addr;
	if (!mmc_card_blockaddr(card)) {
		/* MMC_STATE_BLOCKADDR */
		mrq->cmd->arg *= SD_BLOCK_SIZE;
	}

	mrq->cmd->flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	mrq->stop = NULL;

	mrq->data->blksz = blksz;
	mrq->data->blocks = blocks;
	mrq->data->flags = MMC_DATA_READ;
	mrq->data->sg = sg;
	mrq->data->sg_len = sg_len;

	mmc_set_data_timeout(mrq->data, card);
}

/*
 * Checks that a normal transfer didn't have any errors
 */
static int mmc_sd_check_result(struct mmc_card *card,
	struct mmc_request *mrq)
{
	int ret = 0;

	if ((mrq->cmd->error)  ||
	    (mrq->data->error) ||
	    (mrq->stop && mrq->stop->error) ||
	    (mrq->data->bytes_xfered !=
	     mrq->data->blocks * mrq->data->blksz)) {
		ret = -EIO;
	}

	return ret;
}

/*
 * transfer with certain parameters
 */
static int mmc_sd_transfer(struct mmc_card *card,
	struct scatterlist *sg, unsigned sg_len, unsigned dev_addr,
	unsigned blocks, unsigned blksz)
{
	struct mmc_request mrq;
	struct mmc_command cmd;
	struct mmc_command stop;
	struct mmc_data data;

	memset(&mrq, 0, sizeof(struct mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&data, 0, sizeof(struct mmc_data));
	memset(&stop, 0, sizeof(struct mmc_command));

	mrq.cmd = &cmd;
	mrq.data = &data;
	mrq.stop = &stop;

	mmc_sd_prepare_mrq(card, &mrq, sg, sg_len, dev_addr,
		blocks, blksz);

	mmc_wait_for_req(card->host, &mrq);

	return mmc_sd_check_result(card, &mrq);
}

static int mmc_sd_read_block(struct mmc_card *card,
	unsigned long addr, unsigned long len, unsigned char *buff)
{
	int ret = -EIO;
	struct scatterlist sg;

	if (len == 1) {
		ret = mmc_sd_set_blksize(card, SD_BLOCK_SIZE);
		if (!ret) {
			sg_init_one(&sg, buff, SD_BLOCK_SIZE);

			ret = mmc_sd_transfer(card, &sg, 1,
					addr,
					len, SD_BLOCK_SIZE);
		}
	}

	return ret;
}


int mmc_sd_check_boot_signature(struct mmc_card *card)
{
	int ret = 0;
	unsigned char *buff;

	buff = kmalloc(SD_BLOCK_SIZE, GFP_KERNEL);
	if (buff) {
		if (!(mmc_sd_read_block(card, 0, 1, buff))) {
			if ((buff[510] != 0x55) || (buff[511] != 0xAA)) {
				/* no boot signature */
				ret = -EINVAL;
			}
		} else {
			/* read error */
			ret = -EIO;
		}
		kfree(buff);
	}
	return ret;
}
