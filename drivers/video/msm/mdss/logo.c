/*
 * Copyright (C) 2008 The Android Open Source Project
 * Copyright(C) 2013 Foxconn International Holdings, Ltd. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/fb.h>
#include <linux/vt_kern.h>
#include <linux/unistd.h>
#include <linux/syscalls.h>
#include <linux/irq.h>
#include <asm/system.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include "mdss_fb.h"

/*PERI-AC-LOGO-00+{*/
#define fb_width(fb)	((fb)->var.xres)
#define fb_linewidth(fb) \
	((fb)->fix.line_length / (fb_depth(fb) == 2 ? 2 : 4))
#define fb_height(fb)	((fb)->var.yres)
#define fb_depth(fb)	((fb)->var.bits_per_pixel >> 3)
#define fb_size(fb)	(fb_width(fb) * fb_height(fb) * fb_depth(fb))

static void memset16(void *_ptr, unsigned short val, unsigned count)
{
	unsigned short *ptr = _ptr;
	count >>= 1;
	while (count--)
		*ptr++ = val;
}

static void memset32(void *_ptr, unsigned int val, unsigned count)
{
	unsigned int *ptr = _ptr;
	count >>= 2;
	while (count--)
		*ptr++ = val;
}
/*PERI-AC-LOGO-00+}*/

int fih_load_rle_image(const char *filename)
{
/*PERI-AC-LOGO-00+{*/
	struct fb_info *info;
	int fd, err = 0;
	unsigned count, max, width, stride, line_pos = 0;
	unsigned short *data, *ptr;
	unsigned char *bits;
	mm_segment_t old_fs = get_fs();
	set_fs (get_ds());
	info = registered_fb[0];
	if (!info) {
		printk(KERN_WARNING "%s: Can not access framebuffer\n",
			__func__);
		set_fs(old_fs);
		return -ENODEV;
	}

	fd = sys_open(filename, O_RDONLY, 0);
	if (fd < 0) {
		printk(KERN_WARNING "%s: Can not open %s\n",
			__func__, filename);
		set_fs(old_fs);
		return -ENOENT;
	}
	count = sys_lseek(fd, (off_t)0, 2);
	if (count <= 0) {
		err = -EIO;
		goto err_logo_close_file;
	}
	sys_lseek(fd, (off_t)0, 0);
	data = kmalloc(count, GFP_KERNEL);
	if (!data) {
		printk(KERN_WARNING "%s: Can not alloc data\n", __func__);
		err = -ENOMEM;
		goto err_logo_close_file;
	}
	if (sys_read(fd, (char *)data, count) != count) {
		err = -EIO;
		goto err_logo_free_data;
	}
	width = fb_width(info);
	stride = fb_linewidth(info);
	max = width * fb_height(info);
	ptr = data;
	bits = (unsigned char *)(info->screen_base);

	while (count > 3) {
		int n = ptr[0];

		if (n > max)
			break;
		max -= n;
		while (n > 0) {
			unsigned int j =
				(line_pos + n > width ? width-line_pos : n);

			if (fb_depth(info) == 2)
				memset16(bits, swab16(ptr[1]), j << 1);
			else {
				unsigned int widepixel = ptr[1];
				/*
				 * Format is RGBA, but fb is big
				 * endian so we should make widepixel
				 * as ABGR.
				 */
				widepixel =
					/* red :   f800 -> 000000f8 */
					(widepixel & 0xf800) >> 8 |
					/* green : 07e0 -> 0000fc00 */
					(widepixel & 0x07e0) << 5 |
					/* blue :  001f -> 00f80000 */
					(widepixel & 0x001f) << 19;
				memset32(bits, widepixel, j << 2);
			}
			bits += j * fb_depth(info);
			line_pos += j;
			n -= j;
			if (line_pos == width) {
				bits += (stride-width) * fb_depth(info);
				line_pos = 0;
			}
		}
		ptr += 2;
		count -= 4;
	}

err_logo_free_data:
	kfree(data);
err_logo_close_file:
	sys_close(fd);
	set_fs(old_fs);
	return err;
/*PERI-AC-LOGO-00+}*/
}

int mdss_load_rle565_image(const char *filename,int clean)
{
	struct fb_info *info = NULL;
	struct file    *filp = NULL;
	unsigned short *ptr  = NULL;
	unsigned char  *bits = NULL;
	unsigned char  *data = NULL;
	unsigned max = 0;
	int bits_count = 0, count = 0, err = 0;

	mm_segment_t old_fs = get_fs();
	set_fs (get_ds());

	printk(KERN_INFO "[DISPLAY]%s\n", __func__);

	info = registered_fb[0];
	if (!info) {
		printk(KERN_WARNING "%s: Can not access fb0\n", __func__);
		return -ENODEV;
	}  

	filp = filp_open(filename, O_RDONLY, 0);
	if (IS_ERR(filp)) {
		printk(KERN_ERR "%s: Can not open %s\n", __func__, filename);
		err = -ENOENT;
		goto error2;
	}  

	count = filp->f_dentry->d_inode->i_size;
	data = kmalloc(count, GFP_KERNEL);
	if (!data) {
		printk(KERN_ERR "%s: Can not alloc data\n", __func__);
		err = -ENOMEM;
		goto error1;
	}  

	if (filp->f_op->read(filp, data, count, &filp->f_pos) < 0) {
		printk(KERN_ERR "%s: read file error?\n", __func__);
		err = -EIO;
		goto error1;
	}  

	max = info->var.xres * info->var.yres;
	ptr = (unsigned short *)data;
	bits = (unsigned char *)(info->screen_base);

	if (!bits) {
		printk(KERN_ERR "%s: screen_base = NULL\n", __func__);
		err = -ENOMEM;
		goto error1;
	}  

	while (count > 3) {
		unsigned n = ptr[0];
		if (n > max)
			break;
		bits_count = n;

		while (bits_count--) {
			if(clean)
				ptr[1] = 0x0;
			*bits++ = (ptr[1] & 0xF800) >> 8;
			*bits++ = (ptr[1] & 0x7E0) >> 3;
			*bits++ = (ptr[1] & 0x1F) << 3;
			*bits++ = 0xFF;
		}

		max -= n;
		ptr += 2;
		count -= 4;
	}  

	error1:
	filp_close(filp, NULL);
	kfree(data);
	error2:
	set_fs(old_fs);
	return err;

}

int fih_dump_framebuffer(char *filename)
{
	struct fb_info *info = NULL;
	struct file    *filp = NULL;
	unsigned char  *bits = NULL;
	int count = 0, err = 0;

	mm_segment_t old_fs = get_fs();
	set_fs (get_ds());

	printk(KERN_INFO "[DISPLAY]%s\n", __func__);

	info = registered_fb[0];
	if (!info) {
		printk(KERN_WARNING "%s: Can not access framebuffer\n",
			__func__);
		return -ENODEV;
	}

	bits = (unsigned char *)(info->screen_base);
	filp = filp_open(filename, O_RDWR | O_CREAT, 0644);
	if (IS_ERR(filp)) {
		printk(KERN_WARNING "%s: Can not open %s\n",
			__func__, filename);
		err = -ENOENT;
		goto error1;
	}
#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER
	count = 3 *(info->var.xres * info->var.yres * 4);
#else
	count = 2 *(info->var.xres * info->var.yres * 4);
#endif
	if (filp->f_op->write(filp, bits, count, &filp->f_pos) < 0) {
		printk(KERN_WARNING "%s: write file error?\n", __func__);
		err = -EIO;
	}

	filp_close(filp, NULL);
error1:
	set_fs(old_fs);
	return err;
}

void draw_logo(void) {
	struct fb_info *fb_info = registered_fb[0];
	int ret = 0;

	printk(KERN_INFO "[DISPLAY]%s\n", __func__);

	if (fb_info && fb_info->fbops->fb_open) {
		ret = fb_info->fbops->fb_open(fb_info, 0);
		if (ret) {
			printk(KERN_WARNING "[DISPLAY]%s: Can not open fb0, ret <%d>\n",
					__func__, ret);
		} else {
			ret = fih_load_rle_image(INIT_IMAGE_FILE);
			if (!ret) {
				ret = fb_info->fbops->fb_pan_display(&fb_info->var, fb_info);
				if (!ret) {
					printk(KERN_INFO "[DISPLAY]%s: Drawing logo\n", __func__);
				} else {
					printk(KERN_ERR "[DISPLAY]%s: Can not pan, ret <%d>\n",
							__func__, ret);
				}
			}
		}
	} else {
		printk("[DISPLAY]%s: fb_info/fb_info->fbops->fb_open is NULL\n", __func__);
	}
}

int __init logo_init(void)
{
	if(0)
		draw_logo();
	return 0;
}
module_init(logo_init);
