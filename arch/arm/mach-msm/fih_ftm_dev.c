/*
*Copyright(C) 2011-2013 Foxconn International Holdings, Ltd. All rights reserved
*/

#include <asm/uaccess.h>
#include <mach/gpio.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "../../../drivers/gpio/gpio-msm-common.h"

#include <linux/fih_ftm_dev.h>
#include <linux/fih_hw_info.h>

static int ftm_dev_open( struct inode * inode, struct file * file )
{
  pr_info("FIH FTM dev open\n");
  return 0;
}

static ssize_t ftm_dev_read( struct file * file, char __user * buffer, size_t len, loff_t * f_pos )
{
  pr_info("FIH FTM read, length=%d\n",len);
  return 0;
}

static int ftm_dev_write( struct file * filp, const char __user * buffer, size_t len, loff_t * offset )
{
  pr_info("FIH FTM write, length=%d, data=%s\r\n",len ,buffer);
  return len;
}

static long ftm_dev_ioctl(struct file * filp, unsigned int cmd, unsigned long arg )
{
  int rc = 0;
  unsigned int *gpio_cfg;
  unsigned gpio_num, gpio_outv;
  //===========================
  int *hwid_adc_volt;

  pr_info("FIH ftm ioctl, cmd = 0x%08x\r\n", cmd);

  switch(cmd)
  {
    case FIH_IOCTL_GPIO_CFG:
      gpio_cfg = kzalloc(sizeof(*gpio_cfg), GFP_KERNEL);
      if ( copy_from_user(gpio_cfg, (void *)arg, sizeof(*gpio_cfg)) )
      {
        rc = -EFAULT;
      }
      else
      {
        pr_info("FIH ftm ioctl, GPIO_CFG, arg = 0x%08x\r\n", *gpio_cfg);
        __gpio_tlmm_config(*gpio_cfg);
      }
      kfree(gpio_cfg);
     break;

    case FIH_IOCTL_GPIO_OUTV:
      gpio_cfg = kzalloc(sizeof(*gpio_cfg), GFP_KERNEL);
      if ( copy_from_user(gpio_cfg, (void *)arg, sizeof(*gpio_cfg)) )
      {
        rc = -EFAULT;
      }
      else
      {
        pr_info("FIH ftm ioctl, GPIO_OUTV, arg = 0x%08x\r\n", *gpio_cfg);
        gpio_num = GPIO_PIN(*gpio_cfg);
        /*
         notic that ftm use output value
         to replace the GPIO pull status
        */
        gpio_outv = GPIO_PULL(*gpio_cfg);
        __msm_gpio_set_inout(gpio_num,gpio_outv);
      }
      kfree(gpio_cfg);
     break;

    case FIH_IOCTL_GPIO_INV:
      gpio_cfg = kzalloc(sizeof(*gpio_cfg), GFP_KERNEL);
      if ( copy_from_user(gpio_cfg, (void *)arg, sizeof(*gpio_cfg)) )
      {
        rc = -EFAULT;
      }
      else
      {
        pr_info("FIH ftm ioctl, GPIO_INV, arg = 0x%08x\r\n", *gpio_cfg);
        gpio_num = GPIO_PIN(*gpio_cfg);
        //use return value to stand for high or low
        rc = __msm_gpio_get_inout(gpio_num);
      }
      kfree(gpio_cfg);
     break;

    case FIH_IOCTL_HWID_ADC:
      hwid_adc_volt = kzalloc(sizeof(*hwid_adc_volt), GFP_KERNEL);
      if ( copy_from_user(hwid_adc_volt, (void *)arg, sizeof(*hwid_adc_volt)) )
      {
        rc = -EFAULT;
      }
      else
      {
        pr_info("FIH ftm ioctl, HWID_ADC, arg = 0x%08x\r\n", *hwid_adc_volt);
        //use the same buffer to recieve and transmit
        *hwid_adc_volt=(int)fih_get_hwid_adc(*hwid_adc_volt);

        if ( copy_to_user( (void *)arg, hwid_adc_volt, sizeof(*hwid_adc_volt)) )
        {
          rc = -EFAULT;
        }
      }
      kfree(hwid_adc_volt);
     break;
    default:
      pr_err("Undefined FIH FTM ioctl cmd\r\n");
     return -EPERM;
  }
  return rc;
}

static const struct file_operations fih_ftm_dev_fops = {
  .open = ftm_dev_open,
  .read = ftm_dev_read,
  .write = ftm_dev_write,
  .unlocked_ioctl = ftm_dev_ioctl,
};

static struct miscdevice fih_ftm_dev = {
  MISC_DYNAMIC_MINOR,
  "fih_ftm_dev",
  &fih_ftm_dev_fops
};

static int __init fih_ftm_dev_init(void)
{
  int ret;

  pr_info("FIH FTM device init\r\n" );

  ret = misc_register(&fih_ftm_dev);
  if (ret)
  {
    pr_err("FIH FTM Unable to register misc device\r\n");
  }

  return ret;
}

static void __exit fih_ftm_dev_exit(void)
{
  pr_info("FIH FTM device exit\r\n" );
  misc_deregister(&fih_ftm_dev);
}

module_init(fih_ftm_dev_init);
module_exit(fih_ftm_dev_exit);

MODULE_DESCRIPTION( "FIH FTM driver" );
MODULE_LICENSE( "GPL" );

