#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/stddef.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/acpi.h>
#include <linux/io.h>
#include <linux/dmi.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/err.h>

#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/pm_runtime.h>

#include <linux/gpio.h>
#include <linux/i2c-mux-gpio.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>

//#include<linux/i2c-dev.h>

#include "sha204.h"

#define ATSHA204A_DEVICE_ADDR         0xC8
#define ATSHA204A_DRIVER_NAME         "msic-atsha204a"
#define ATSHA204A_I2C_DEV_NAME        "msm_sha204" 

int i2c_fail;
#define I2C_ADAPTER    2
struct i2c_client           *gclient = NULL;
struct miscdevice	    	misc;
static struct i2c_adapter   *i2c_bus_adapter;

static struct i2c_board_info i2c_atsha204a ={ I2C_BOARD_INFO(ATSHA204A_I2C_DEV_NAME, (ATSHA204A_DEVICE_ADDR>>1)) };

struct atsha204_pack {
    uint8_t device_addr;
    uint8_t len;
    uint8_t data[128];
};


void printbuf(char *buf, int len)
{
    int i;
    for(i=0; i<len; i++) {
        if (i && i%16 == 0)
            printk("\n");
        printk("0x%02x ", buf[i]);
    }
    printk("\n");
}

static int result = 0;
static ssize_t serial_number_show(struct device *dev,struct device_attribute *attr,char* buf)
{
	if (result)
		return sprintf(buf,"%s\n", "success!");
	else
		return sprintf(buf,"%s\n", "fail!");
    return 0;
}

static ssize_t serial_number_store(struct device *dev, struct device_attribute *attr, const char* buf, size_t size)
{

	return size;
}
//static DEVICE_ATTR(SerialNumber, 0666, serial_number_show, serial_number_store);
static DEVICE_ATTR(atsha204a, S_IRUGO | S_IWUSR, serial_number_show, serial_number_store);

static struct attribute *mid_att_als[] = {
	&dev_attr_atsha204a.attr,
	//&dev_attr_lux0_input.attr,
	NULL
};

static struct attribute_group serialNum = {
	.name = "crypto-dev",
	.attrs = mid_att_als
};

static int sha204_receive(struct atsha204_pack *pack)
{
	uint8_t count = 0;
    struct i2c_client *client = gclient;
    int ret = 1;
    printk("sha204_receive enter!\n");

    pack->len = 0;
    client->addr = pack->device_addr;
    ret = i2c_master_recv(client, pack->data, 1);
    if (ret < 0)
		return -1;

    count = pack->data[0];
    printk("i2c recv frame len %d\n", count);
    if ((count < 4) || (count > sizeof(pack->data))) {
        return -1;
    }
    ret = i2c_master_recv(client, &pack->data[1], count-1);
    if (ret == count-1) {
        printk("i2c recv data success:\n");
        printbuf(pack->data, count);
        pack->len = count;
        return 0;	
    } else {
        printk("i2c recv data fail\n");
		return -1;
    }
    return 0;	
}

static int sha204_send(struct atsha204_pack *pack)
{
    struct i2c_client *client = gclient;
    int ret = 0;
    printk("sha204_send enter!\n");

    client->addr = pack->device_addr;
    ret = i2c_master_send(client, (const char *)pack->data, pack->len);
    printk("send cmd ret %d\n", ret);

    printk("send data dump:\n");
    printbuf(pack->data, pack->len);

    if (ret == pack->len)
        return 0;
    else
        return -1;
}

#define ATSHA204A_WRITE 0x00
#define ATSHA204A_READ  0x01
static long atsha204a_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct atsha204_pack pack;
    long ret;

	switch (cmd) {
    case ATSHA204A_READ:
        {
            if (copy_from_user(&pack, (void __user *)arg, sizeof(pack)))
                return -EFAULT;

            ret = sha204_receive(&pack);
            if (copy_to_user((void __user *)arg, &pack, sizeof(pack)))
                return -EFAULT;

			break;
        }
	case ATSHA204A_WRITE:
        {
            if (copy_from_user(&pack, (void __user *)arg, sizeof(pack)))
                return -EFAULT;
            printk("recv pack len = %d, addr 0x%02x\n", pack.len, pack.device_addr);
            printbuf(pack.data, pack.len);

            ret = sha204_send(&pack);
            break;
        }
	default:
		return -EINVAL;
	}

	return ret;
}


#ifdef CONFIG_COMPAT
static long atsha204a_compat_ioctl_process(struct file *filep,
				   unsigned int cmd, unsigned long arg)
{
	arg = (unsigned long)compat_ptr(arg);
	return atsha204a_ioctl(filep, cmd, arg);
}
#endif	/* CONFIG_COMPAT */

const struct file_operations atsha204a_fops = {
	.unlocked_ioctl = atsha204a_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = atsha204a_compat_ioctl_process,
#endif  /* CONFIG_COMPAT */
};

static int msm_sha204_i2c_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s i2c_check_functionality failed\n", __func__);
        return -1;
	}
    printk("client device addr = 0x%x\n", client->addr);

	misc.name = ATSHA204A_DRIVER_NAME;
	misc.minor = MISC_DYNAMIC_MINOR;
    misc.fops = &atsha204a_fops;
    if (misc_register(&misc)) {
        printk("Unable to register misc device!\n");
        return -EFAULT;
    }
    return 0;
}

static int msm_sha204_i2c_remove(struct i2c_client *client)
{
	return 0;
}

//match by msm_sha204_i2c_id or .name
static const struct i2c_device_id msm_sha204_i2c_id[] = {
	{ ATSHA204A_I2C_DEV_NAME, (kernel_ulong_t)NULL},
	{ }
};

static struct i2c_driver msm_sha204_i2c_driver = {
	.id_table = msm_sha204_i2c_id,
	.probe  = msm_sha204_i2c_probe,
	.remove = msm_sha204_i2c_remove,
	.driver = {
		.name = ATSHA204A_I2C_DEV_NAME,
	},
};


static int i2c_init(void)
{
	int ret = -ENODEV;

	i2c_bus_adapter = i2c_get_adapter(I2C_ADAPTER);
	if (i2c_bus_adapter == NULL)
        return ret;
    gclient = i2c_new_device(i2c_bus_adapter, &i2c_atsha204a);
    if (!gclient)
        return ret;
	ret = i2c_add_driver(&msm_sha204_i2c_driver);
	if (ret < 0) {
        printk("i2c_add_driver error!\n");
        i2c_unregister_device(gclient);
        return ret;
	}
	return ret;
}

static int i2c_release(void)
{
    printk("release atsha204 i2c driver!\n");
    i2c_del_driver(&msm_sha204_i2c_driver);
    if(gclient) {
        i2c_unregister_device(gclient);
        printk("release atsha204 i2c device!\n");
    }
    misc_deregister(&misc);
    return 0;
}
static int sha204_probe(struct platform_device *dev)
{
    int res;
	res = sysfs_create_group(&dev->dev.kobj, &serialNum);
	if (res) {
		printk("could not create file attrbute: serialNumber\n");
        return -1;
	}

    i2c_fail = i2c_init();
    printk("i2c init ret = %d\n", i2c_fail);

	return 0;
}

static void sha204_device_release(struct device *dev)
{
    printk("sha204: device release\n");
}

static int sha204_remove(struct platform_device *dev)
{
	//device_remove_file(&dev->dev,&dev_attr_SerialNumber);
	printk("remove sysfs devices ...\n");
    sysfs_remove_group(&dev->dev.kobj, &serialNum);
	return 0;
}

static struct platform_device sha204_device = {
	.name = "sha204",
	.id = 0,
	.dev = {
        .coherent_dma_mask = 0xffffffffUL,	
        .release = sha204_device_release
    },
};

static struct platform_driver sha204_driver = {
	.driver = {
		.name = "sha204",
		.owner = THIS_MODULE,
	},
	.probe	= sha204_probe,
	.remove = sha204_remove,
};


static int __init sha204_init(void)
{
	int ret = 0;

	printk("sha204 init entry ...\n");
	ret = platform_driver_register(&sha204_driver);
	if (!ret)
	{
		ret = platform_device_register(&sha204_device);
		if (ret)
			platform_driver_unregister(&sha204_driver);
	}
    return 0;
}

static void __exit sha204_exit(void)
{
	printk("sha204 exit entry ...\n");
	platform_device_unregister(&sha204_device);
	platform_driver_unregister(&sha204_driver);

    if(!i2c_fail)
        i2c_release();
}

module_init(sha204_init);
module_exit(sha204_exit);

MODULE_LICENSE("GPL");

