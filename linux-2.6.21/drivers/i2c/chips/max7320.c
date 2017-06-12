/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon-sysfs.h>
#include <linux/delay.h>

/* Addresses to scan */
static unsigned short normal_i2c[] = {0x50, I2C_CLIENT_END};
static struct i2c_client *max7320_client = NULL;

/* Insmod parameters */
I2C_CLIENT_INSMOD_1(max7320);

static int max7320_attach_adapter(struct i2c_adapter *adapter);
static int max7320_detect(struct i2c_adapter *adapter, int address, int kind);
static int max7320_detach_client(struct i2c_client *client);
int max7320_read(void);
int max7320_write(u8 value);

/* This is the driver that will be inserted */
static struct i2c_driver max7320_driver = {
	.driver = {
		.name	= "max7320",
	},
	.attach_adapter	= max7320_attach_adapter,
	.detach_client	= max7320_detach_client,
};

struct max7320_data {
	struct i2c_client client;
};

/* following are the sysfs callback functions */
static ssize_t max7320_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	return sprintf(buf, "%d\n", max7320_read());
}

static ssize_t max7320_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	unsigned long val = simple_strtoul(buf, NULL, 0);
	if (val > 0xff)
		return -EINVAL;

	max7320_write(val);
	return count;
}

/* Define the device attributes */
static SENSOR_DEVICE_ATTR(ro, S_IRUGO, max7320_show, NULL, 0);
static SENSOR_DEVICE_ATTR(rw, S_IRUGO | S_IWUSR, max7320_show, max7320_store, 1);

static struct attribute *max7320_attributes[] = {
	&sensor_dev_attr_ro.dev_attr.attr,
	&sensor_dev_attr_rw.dev_attr.attr,
	NULL
};

static struct attribute_group max7320_defattr_group = {
	.attrs = max7320_attributes,
};

static int max7320_attach_adapter(struct i2c_adapter *adapter)
{
	return i2c_probe(adapter, &addr_data, max7320_detect);
}

/* This function is called by i2c_probe */
static int max7320_detect(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client *new_client;
	struct max7320_data *data;
	int err = 0;
	int temp, tempvalue;
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		goto exit;

	/* OK. For now, we presume we have a valid client. We now create the
	   client structure, even though we cannot fill it completely yet. */
	if (!(data = kmalloc(sizeof(struct max7320_data), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}
	memset(data, 0, sizeof(struct max7320_data));

	new_client = &data->client;
	i2c_set_clientdata(new_client, data);
	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &max7320_driver;
	new_client->flags = 0;

	if (i2c_smbus_read_byte(new_client) < 0){
		goto exit_kfree;
	}else {
		printk("GPIO expander Max7320 detected!\n");
	}

	strlcpy(new_client->name, "max7320", I2C_NAME_SIZE);
	max7320_client = new_client;
		
	/* Tell the I2C layer a new client has arrived */
	if ((err = i2c_attach_client(new_client)))
		goto exit_kfree;

	/* Register sysfs hooks (don't care about failure) */
	err = sysfs_create_group(&new_client->dev.kobj, &max7320_defattr_group);
	if (err)
		goto exit_kfree;

	tempvalue = max7320_read();
	temp = max7320_write(tempvalue & 0xdf);
	if(temp == 0){
		printk("VGA camera power on ok!\n");
	}else{
		printk("VGA camera power on failed!\n");
	}
	
	return 0;

exit_kfree:
	kfree(data);
exit:
	return err;
}

static int max7320_detach_client(struct i2c_client *client)
{
	int err;

	if ((err = i2c_detach_client(client)))
		return err;

	max7320_client = NULL;

	kfree(i2c_get_clientdata(client));
	return 0;
}

int max7320_read(void)
{
	if( max7320_client == NULL )	/*	No global client pointer?	*/
		return -1;
		
	return i2c_smbus_read_byte(max7320_client);
}

int max7320_write(u8 value)
{
	if( max7320_client == NULL )	/*	No global client pointer?	*/
		return -1;
		
	return i2c_smbus_write_byte(max7320_client, value);
}

EXPORT_SYMBOL_GPL(max7320_read);
EXPORT_SYMBOL_GPL(max7320_write);

static int __init max7320_init(void)
{
	return i2c_add_driver(&max7320_driver);
}

static void __exit max7320_exit(void)
{
	i2c_del_driver(&max7320_driver);
}

MODULE_AUTHOR("Mingliang <mingliang.hu@marvell.com>");
MODULE_DESCRIPTION("MAX7320 driver");
MODULE_LICENSE("GPL");

module_init(max7320_init);
module_exit(max7320_exit);

