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
	int retval = 0;
	//int key_id = 2;

	result = 0;
	printk("get: %s\n", buf);

	//retval = atsha204_device_personalization();
	if (retval == 0)
		result = 1;

	return size;
}
//static DEVICE_ATTR(SerialNumber, 0666, serial_number_show, serial_number_store);
static DEVICE_ATTR(atsha204a, S_IRUGO | S_IWUSR, serial_number_show, serial_number_store);

static struct attribute *mid_att_als[] = {
	&dev_attr_atsha204a.attr,
	//&dev_attr_lux0_input.attr,
	NULL
};

static struct attribute_group m_als_gr = {
	.name = "crypto-dev",
	.attrs = mid_att_als
};

static int sha204_probe(struct platform_device *dev)
{
    int res;
#if 1
#if 0
	int retval;
	int key_id = 2;
	//we need use key_id 2, if not,reset!
	char key[] = {0x14, 0x15, 0x63, 0x37, 0x28, 0x45, 0x73, 0x94, 0x51, 0x34,
				 0x61, 0x92, 0x79, 0x3b, 0xec, 0xc4, 0x29, 0xfc, 0xdf, 0x7d,
				 0x6c, 0xaa, 0x76, 0x23, 0x85, 0x12, 0x1d, 0x4e, 0x53, 0x8e,
				 0xe1, 0xd3};
	
	char num_in[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10,
				     0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x20};

	char challenge[] = {0xff, 0xfe, 0xfd, 0xfc, 0xfb, 0xfa, 0xf9, 0xf8, 0xf7, 0xf6,
						0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10,
						0x6c, 0xaa, 0x76, 0x23, 0x85, 0x12, 0x1d, 0x4e, 0x23, 0x8e,
						0xe1, 0xd3};
						
	//retval = atsha204_mac(key_id, key, num_in, challenge);
	//printk("sha204_mac retval = %d\n", retval);
	if (retval != 0)
	{
		//reset  
		printk("sha204_module reset ...\n");
		//mub_update1(0x0101, 0x05);	
	}
	retval=device_create_file(&dev->dev, &dev_attr_SerialNumber);
	if (retval)
		printk("could not create file attrbute: SerialNumber\n");
#endif

	res = sysfs_create_group(&dev->dev.kobj, &m_als_gr);
	if (res) {
        return -1;
	}

#endif
	return 0;
}

static int sha204_remove(struct platform_device *dev)
{
	//device_remove_file(&dev->dev,&dev_attr_SerialNumber);
    sysfs_remove_group(&dev->dev.kobj, &m_als_gr);
	return 0;
}

static struct platform_device sha204_device = {
	.name = "sha204",
	.id = 0,
	.dev = {
		.coherent_dma_mask = 0xffffffffUL	
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

	return ret;
}

static void __exit sha204_exit(void)
{
	platform_device_unregister(&sha204_device);
	platform_driver_unregister(&sha204_driver);
}

module_init(sha204_init);
module_exit(sha204_exit);

MODULE_LICENSE("GPL");

