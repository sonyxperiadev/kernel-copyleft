/*
 * Copyright (c) 2013, 2014
 * Phillip Lougher <phillip@squashfs.org.uk>
 *
 * This work is licensed under the terms of the GNU GPL, version 2. See
 * the COPYING file in the top-level directory.
 */

#include <linux/buffer_head.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/lz4.h>

#include "squashfs_fs.h"
#include "squashfs_fs_sb.h"
#include "squashfs.h"
#include "decompressor.h"

struct squashfs_lz4 {
	void *input;
	void *output;
};

static void *lz4_init(struct squashfs_sb_info *msblk, void *buff, int len)
{
	int block_size = max_t(int, msblk->block_size, SQUASHFS_METADATA_SIZE);
	struct squashfs_lz4 *stream;

	stream = kzalloc(sizeof(*stream), GFP_KERNEL);
	if (stream == NULL)
		goto failed;
	stream->input = vmalloc(block_size);
	if (stream->input == NULL)
		goto failed2;
	stream->output = vmalloc(block_size);
	if (stream->output == NULL)
		goto failed3;

	return stream;

failed3:
	vfree(stream->input);
failed2:
	kfree(stream);
failed:
	ERROR("Failed to initialise LZ4 decompressor\n");
	return ERR_PTR(-ENOMEM);
}


static void lz4_free(void *strm)
{
	struct squashfs_lz4 *stream = strm;

	if (stream) {
		vfree(stream->input);
		vfree(stream->output);
	}
	kfree(stream);
}


static int lz4_uncompress(struct squashfs_sb_info *msblk, void **buffer,
	struct buffer_head **bh, int b, int offset, int length, int srclength,
	int pages)
{
	struct squashfs_lz4 *stream = msblk->stream;
	void *buff = stream->input;
	int avail, i, bytes = length, res;
	size_t out_len = srclength;

	mutex_lock(&msblk->read_data_mutex);

	for (i = 0; i < b; i++) {
		wait_on_buffer(bh[i]);
		if (!buffer_uptodate(bh[i]))
			goto block_release;

		avail = min(bytes, msblk->devblksize - offset);
		memcpy(buff, bh[i]->b_data + offset, avail);
		buff += avail;
		bytes -= avail;
		offset = 0;
		put_bh(bh[i]);
	}

	res = lz4_decompress_unknownoutputsize(stream->input, length,
					stream->output, &out_len);
	if (res)
		goto failed;

	res = bytes = (int)out_len;
	for (i = 0, buff = stream->output; bytes && i < pages; i++) {
		avail = min_t(int, bytes, PAGE_CACHE_SIZE);
		memcpy(buffer[i], buff, avail);
		buff += avail;
		bytes -= avail;
	}

	mutex_unlock(&msblk->read_data_mutex);
	return res;

block_release:
	for (; i < b; i++)
		put_bh(bh[i]);

failed:
	mutex_unlock(&msblk->read_data_mutex);

	ERROR("lz4 decompression failed, data probably corrupt\n");
	return -EIO;
}

const struct squashfs_decompressor squashfs_lz4_comp_ops = {
	.init = lz4_init,
	.free = lz4_free,
	.decompress = lz4_uncompress,
	.id = LZ4_COMPRESSION,
	.name = "lz4",
	.supported = 1
};
