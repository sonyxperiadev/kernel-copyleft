//
// Created by ubuntu on 2021/5/28.
//
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gfp.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
//#include <linux/hardware_info.h>

#define NXP_RESET_DEV_CNT 1 /* 设备号长度 */
#define NXP_RESET_DEV_NAME "nfcReset" /* 设备名字 */

#define IOC_MAGIC  'c'
#define IOCINIT    _IO(IOC_MAGIC, 0)
#define IOCGREG    _IOR(IOC_MAGIC, 1, int)

/* nfc_g7 设备结构体 */
struct nfc_g7_dev{
    char * val;
    dev_t devid; /* 设备号 */
    struct cdev cdev; /* cdev */
    struct class *class; /* 类 */
    struct device *device; /* 设备 */
    int major; /* 主设备号 */
    struct device_node *node; /* NFC 设备节点 */
    int reset0; /* NFC reset GPIO 标号 */
};

struct nfc_g7_dev *nfc_dev; /* nfc 设备 */


/*
 * @description : 打开设备
 * @param – inode : 传递给驱动的 inode
 * @param - filp : 设备文件，file 结构体有个叫做 private_data 的成员变量
 * 一般在 open 的时候将 private_data 指向设备结构体。
 * @return : 0 成功;其他 失败
 */
static int nfc_reset_open(struct inode *inode, struct file *filp)
{
    struct nfc_g7_dev * dev;
    dev = container_of(inode->i_cdev, struct nfc_g7_dev, cdev);
    filp->private_data = dev; /* 设置私有数据 */
    return 0;
}

static long nfc_reset_ioctl(struct file *filp, unsigned int cmd, unsigned long arg){

    struct nfc_g7_dev* dev = filp->private_data;
    long ret = 0;
    printk(KERN_ALERT" nfc_reset_ioctl entered \n");
    switch(cmd){
        case IOCGREG:
            printk(KERN_ALERT"%s: dev->reset0=%d\n",__func__, dev->reset0);
            gpio_set_value(dev->reset0, 1);
            printk(KERN_ALERT"nfc_reset_ioctl gpio_value1: %d\n", gpio_get_value(dev->reset0));
            usleep_range(10000, 10100);
            gpio_set_value(dev->reset0, 0);
            usleep_range(10000, 10100);
            printk(KERN_ALERT"nfc_reset_ioctl gpio_value2: %d\n", gpio_get_value(dev->reset0));
            break;
        default:
            return -EFAULT;

    }


    return ret;
}


/* 设备操作函数 */
static struct file_operations nfc_reset_fops = {
    .owner = THIS_MODULE,
    .open = nfc_reset_open,
    .unlocked_ioctl = nfc_reset_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = nfc_reset_ioctl,
#endif
};

/*
 * @description : flatform 驱动的 probe 函数，当驱动与
 * 设备匹配以后此函数就会执行
 * @param - dev : platform 设备
 * @return : 0，成功;其他负值,失败
 */
static int nfc_reset_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct device_node *np = dev->of_node;
    int tmp;

    pr_err("%s: entered!\n",__func__);

    nfc_dev = kmalloc(sizeof(struct nfc_g7_dev), GFP_KERNEL);
    if(!nfc_dev){
        printk(KERN_ALERT"Failed to alloc nfc g7 dev!!\n");
        return -ENOMEM;
    }

    /* 1、设置设备号 */
    printk("%s: nfc_dev->major1=%d\n",__func__, nfc_dev->major);
    //if (nfc_dev->major) {
    //    nfc_dev->devid = MKDEV(nfc_dev->major, 0);
    //    register_chrdev_region(nfc_dev->devid, NXP_RESET_DEV_CNT,
    //                           NXP_RESET_DEV_NAME);
    //} else {
        alloc_chrdev_region(&nfc_dev->devid, 0, NXP_RESET_DEV_CNT,
                            NXP_RESET_DEV_NAME);
        nfc_dev->major = MAJOR(nfc_dev->devid);
    //}
    printk("%s: nfc_dev->major2=%d\n",__func__, nfc_dev->major);
    /* 2、注册设备 */
    cdev_init(&nfc_dev->cdev, &nfc_reset_fops);

    tmp = cdev_add(&nfc_dev->cdev, nfc_dev->devid, NXP_RESET_DEV_CNT);
    printk("%s: tmp=%d\n",__func__, tmp);

    /* 3、创建类 */
    nfc_dev->class = class_create(NXP_RESET_DEV_NAME);
    if (IS_ERR(nfc_dev->class)) {
        return PTR_ERR(nfc_dev->class);
    }

    /* 4、创建设备 */
    nfc_dev->device = device_create(nfc_dev->class, NULL, nfc_dev->devid,
            NULL, NXP_RESET_DEV_NAME);
    if (IS_ERR(nfc_dev->device)) {
        return PTR_ERR(nfc_dev->device);
    }

    nfc_dev->reset0 = of_get_named_gpio(np, "nfcrst-gpio", 0);
    printk("%s: nfc_dev->reset0=%d\n",__func__, nfc_dev->reset0);
    if (!gpio_is_valid(nfc_dev->reset0)) {
	pr_err("%s: gpio_is_valid(nfc_dev->reset0)=%d: invalid\n",
		__func__, nfc_dev->reset0);
        return -EINVAL;
    }

    //gpio_request(nfc_dev->reset0, "reset0");
    //gpio_direction_output(nfc_dev->reset0, 0); /*设置为输出，默认低电平 */
    printk("nfc_reset_probe successfull !!!\n");

    return 0;
}

/*
 * @description : remove 函数，移除 platform 驱动的时候此函数会执行
 * @param - dev : platform 设备
 * @return : 0，成功;其他负值,失败
 */
static int nfc_reset_remove(struct platform_device *dev)
{
    gpio_set_value(nfc_dev->reset0, 0); /* 卸载驱动的时候关闭 nfcrst */

    cdev_del(&nfc_dev->cdev); /* 删除 cdev */
    unregister_chrdev_region(nfc_dev->devid, NXP_RESET_DEV_CNT);
    device_destroy(nfc_dev->class, nfc_dev->devid);
    class_destroy(nfc_dev->class);
    return 0;
}


static const struct of_device_id nfc_reset_of_match[] = {
	{.compatible = "sony,coldreset-nfcgpio",},
	{},
};

static struct platform_driver nfc_reset_driver = {
	.probe = nfc_reset_probe,
	.remove = nfc_reset_remove,
	.driver = {
		   .name = "nxp_reset",
		   .of_match_table = nfc_reset_of_match,
		   },
};

/*
 * @description : 驱动模块加载函数
 * @param : 无
 * @return : 无
 */
static int __init nfc_reset_test_init(void)
{
    return platform_driver_register(&nfc_reset_driver);
}

/*
 * @description : 驱动模块卸载函数
 * @param : 无
 * @return : 无
 */
static void __exit nfc_reset_test_exit(void)
{
    platform_driver_unregister(&nfc_reset_driver);
}

module_init(nfc_reset_test_init);
module_exit(nfc_reset_test_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("liuchuanfeng@aa.com");
