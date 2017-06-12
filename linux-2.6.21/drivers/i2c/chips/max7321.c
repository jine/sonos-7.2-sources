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
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/arch/littleton.h>
#include <asm/arch/mfp.h>
#include <asm/arch/gpio.h>
#include <asm/irq.h>

/* Addresses to scan (8 bit, not 7 bit !)*/
/*
Known Technology Boards                                           I2C address
802.11b Marvell 88W8385                                           0xC0
Camera Omnivision OV5630/OV3630                                   0xC8
UMTS                                                              0xD0
8688/Camera Board						  0xC6
Combined 802.11b Marvell 88W8385 Camera Omnivision OV5630/OV3630  0xD8
Combined 802.11b Marvell 88W8686 Camera Omnivision OV5630/OV3630  0xCC
Siemens Baseband Board						  0xC4

*/

#define SIEMENS_BOARD	0x00 /* MFB */

static u8 wlan_board_shadow_reg=0;
static u8 btwlancamera_board_shadow_reg=0;
static u8 camera_board_shadow_reg=0;
static u8 umts_board_shadow_reg=0;
static u8 wlan8385camera_board_shadow_reg=0;
static u8 wlan8385camera_board_shadow_mask=0xf;
static u8 wlan8686camera_board_shadow_reg=0;
static u8 wlan8686camera_board_shadow_mask=0xf;
  
 //7 bit addresses = 8 bit addresses >> 1
#define WLAN_BOARD_MAX7321_ADDR         0x60
#define btwlancamera_board_MAX7321_ADDR 0x63
#define CAMERA_BOARD_MAX7321_ADDR       0x65
#define UMTS_BOARD_MAX7321_ADDR         0x61
#define WLAN8385CAMERA_BOARD_MAX7321_ADDR   0x6C
#define WLAN8686CAMERA_BOARD_MAX7321_ADDR   0x66
/* MFB */
#define SIEMENS_BASEBAND_MAX7321_ADDR	0x62

static unsigned short normal_i2c[] = {
		WLAN_BOARD_MAX7321_ADDR,
		UMTS_BOARD_MAX7321_ADDR,
 		btwlancamera_board_MAX7321_ADDR,
		CAMERA_BOARD_MAX7321_ADDR,
 		WLAN8385CAMERA_BOARD_MAX7321_ADDR,
 		WLAN8686CAMERA_BOARD_MAX7321_ADDR,
		SIEMENS_BASEBAND_MAX7321_ADDR, /* MFB */
		I2C_CLIENT_END
};
static struct i2c_client *wlan_board_max7321_client = NULL;
static struct i2c_client *camera_board_max7321_client = NULL;
static struct i2c_client *umts_board_max7321_client = NULL;
static struct i2c_client *btwlancamera_board_max7321_client = NULL;
static struct i2c_client *wlan8385camera_board_max7321_client = NULL;
static struct i2c_client *wlan8686camera_board_max7321_client = NULL;
static struct i2c_client *siemens_baseband_max7321_client = NULL; /* MFB */
	
#define MFP_TECH_BOARD_0_INTERRUPT_GPIO  MFP_PIN_GPIO77
#define MFP_TECH_BOARD_1_INTERRUPT_GPIO  MFP_PIN_GPIO79
#define MFP_TECH_BOARD_2_INTERRUPT_GPIO  MFP_PIN_GPIO89
#define MFP_TECH_BOARD_3_INTERRUPT_GPIO  MFP_PIN_GPIO90

typedef enum expander_irq{
	wlan_board_irq,
	camera_board_irq,
	umts_board_irq,
	btwlancamera_board_irq,
	wlan8385camera_board_irq,
	wlan8686camera_board_irq,
	siemens_baseband_irq, /* MFB */
	none_irq,
} board_irq;

board_irq tech_board_irq = none_irq;
board_irq monahans_tech_0_irq = none_irq;
board_irq monahans_tech_1_irq = none_irq;
board_irq monahans_tech_2_irq = none_irq;
board_irq monahans_tech_3_irq = none_irq;

/* Insmod parameters */
I2C_CLIENT_INSMOD_1(max7321);

static int max7321_attach_adapter(struct i2c_adapter *adapter);
static int max7321_detect(struct i2c_adapter *adapter, int address, int kind);
static int max7321_detach_client(struct i2c_client *client);
int WLAN_board_read_max7321(void);
int WLAN_board_write_max7321(u8 value);
int Camera_board_read_max7321(void);
int Camera_board_write_max7321(u8 value);
int UMTS_board_read_max7321(void);
int UMTS_board_write_max7321(u8 value);
int btwlancamera_board_read_max7321(void);
int btwlancamera_board_write_max7321(u8 value);
int wlan8385camera_board_read_max7321(void);
int wlan8385camera_board_write_max7321(u8 value);
int wlan8686camera_board_read_max7321(void);
int wlan8686camera_board_write_max7321(u8 value);
int Siemens_baseband_read_max7321(void); /* MFB */
int Siemens_baseband_write_max7321(u8 value); /* MFB */

int Test_technology_board_interrupt(void);

/* This is the driver that will be inserted */
static struct i2c_driver max7321_driver = {
	.driver = {
		.name	= "max7321",
	},
	.attach_adapter	= max7321_attach_adapter,
	.detach_client	= max7321_detach_client,
};

struct max7321_data {
	struct i2c_client client;
};

static int max7321_attach_adapter(struct i2c_adapter *adapter)
{
	return i2c_probe(adapter, &addr_data, max7321_detect);
}

/* This function is called by i2c_probe */
static int max7321_detect(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client *new_client;
	struct max7321_data *data;
	int err = 0;
	int temp;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		goto exit;

	/* OK. For now, we presume we have a valid client. We now create the
	   client structure, even though we cannot fill it completely yet. */
	if (!(data = kmalloc(sizeof(struct max7321_data), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}
	memset(data, 0, sizeof(struct max7321_data));

	new_client = &data->client;
	i2c_set_clientdata(new_client, data);
	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &max7321_driver;
	new_client->flags = 0;

	if (i2c_smbus_read_byte(new_client) < 0){
		if(new_client->addr == WLAN_BOARD_MAX7321_ADDR){
			printk("WLAN technology board NOT detected!\n");
		}else if(new_client->addr == CAMERA_BOARD_MAX7321_ADDR){
			printk("Hi-Res Camera technology board NOT detected!\n");
		}else if(new_client->addr == UMTS_BOARD_MAX7321_ADDR){
			printk("UMTS technology board NOT detected!\n");
		}else if(new_client->addr == btwlancamera_board_MAX7321_ADDR){
			printk("8688/Camera technology board NOT detected!\n");
		}else if(new_client->addr == WLAN8385CAMERA_BOARD_MAX7321_ADDR){
			printk("Combined WLAN8385 and Hi-Res Camera technology"
					"board NOT detected!\n");
		}else if(new_client->addr == WLAN8686CAMERA_BOARD_MAX7321_ADDR){
			printk("Combined WLAN8686 and Hi-Res Camera technology"
					"board NOT detected!\n");
		}else if(new_client->addr == SIEMENS_BASEBAND_MAX7321_ADDR){ /* MFB */
			printk("Siemens Baseband technology"
					"board NOT detected!\n");
                }
		goto exit_kfree;
	}else {
		if(new_client->addr == WLAN_BOARD_MAX7321_ADDR){
			wlan_board_max7321_client = new_client;
			strlcpy(new_client->name, "max7321_WLAN", I2C_NAME_SIZE);
			printk("WLAN technology board detected!!!\n");

			WLAN_board_write_max7321(0xf1);
			mdelay(5);
			WLAN_board_write_max7321(0xf3);
			mdelay(5);
			temp = WLAN_board_write_max7321(0xf1);
			if(temp == 0){
				printk("WLAN board power on ok!\n");
			}else{
				printk("WLAN board power on failed!\n");
			}
		}else if(new_client->addr == CAMERA_BOARD_MAX7321_ADDR){
			camera_board_max7321_client = new_client;
			strlcpy(new_client->name, "max7321_Camera", I2C_NAME_SIZE);
			printk("Hi-Res Camera technology board detected!!!\n");
		}else if(new_client->addr == UMTS_BOARD_MAX7321_ADDR){
			umts_board_max7321_client = new_client;
			strlcpy(new_client->name, "max7321_UMTS", I2C_NAME_SIZE);
			printk("UMTS technology board detected!!!\n");
		}else if(new_client->addr == btwlancamera_board_MAX7321_ADDR){
			btwlancamera_board_max7321_client = new_client;
			strlcpy(new_client->name, "max7321_BTWLANCAMERA", I2C_NAME_SIZE);
			printk("8688/Camera technology board detected!!!\n");
			
			btwlancamera_board_write_max7321(0xfc);
			mdelay(5);
			temp = btwlancamera_board_write_max7321(0xfe);
			if(temp == 0){
				printk("8688/ CAMERA board power on ok!\n");
			}else{
				printk("8688/ CAMERA board power on failed!\n");
			}
		}else if(new_client->addr == WLAN8385CAMERA_BOARD_MAX7321_ADDR){
			wlan8385camera_board_max7321_client = new_client;
			strlcpy(new_client->name, "max7321_wlan8385camera", I2C_NAME_SIZE);
			printk("WLAN8385 and CAMERA technology board detected!!!\n");

			wlan8385camera_board_write_max7321(0xf5);
			mdelay(5);
			wlan8385camera_board_write_max7321(0xf7);
			mdelay(5);
			temp = wlan8385camera_board_write_max7321(0xf5);
			if(temp == 0){
				printk("WLAN8385 CAMERA board power on ok!\n");
			}else{
				printk("WLAN8385 CAMERA board power on failed!\n");
			}
		}else if(new_client->addr == WLAN8686CAMERA_BOARD_MAX7321_ADDR){
			wlan8686camera_board_max7321_client = new_client;
			strlcpy(new_client->name, "max7321_wlan8686camera", I2C_NAME_SIZE);
			printk("WLAN8686 and CAMERA technology board detected!!!\n");

			wlan8686camera_board_write_max7321(0xfc);
			mdelay(5);
			temp = wlan8686camera_board_write_max7321(0xfe);
			if(temp == 0){
				printk("WLAN8686 CAMERA board power on ok!\n");
			}else{
				printk("WLAN8686 CAMERA board power on failed!\n");
			}
                }else if(new_client->addr == SIEMENS_BASEBAND_MAX7321_ADDR){ /* MFB */
			siemens_baseband_max7321_client = new_client;
			strlcpy(new_client->name, "max7321_siemens_baseband", I2C_NAME_SIZE);
			printk("Siemens Baseband technology board detected!!!\n");

			printk("Siemens board power on\n");

			if (Siemens_baseband_write_max7321(0xE))
			{
				printk ("Error detected on i2c\n");
			}
			mdelay(450);

			printk("Siemens board power release on key\n");
			if (Siemens_baseband_write_max7321(0xF))
			{
				printk ("Error detected on i2c\n");
			}

			/* enable CLK_POUT if BB is detected */
			enable_oscc_pout();
		}
	}

	/* Tell the I2C layer a new client has arrived */
	if ((err = i2c_attach_client(new_client)))
		goto exit_kfree;

	return 0;

exit_kfree:
	kfree(data);
exit:
	return err;
}

static int max7321_detach_client(struct i2c_client *client)
{
	int err;

	if ((err = i2c_detach_client(client)))
		return err;

	if(client->addr == WLAN_BOARD_MAX7321_ADDR){
		wlan_board_max7321_client = NULL;
	}else if(client->addr == CAMERA_BOARD_MAX7321_ADDR){
		camera_board_max7321_client = NULL;
	}else if(client->addr == UMTS_BOARD_MAX7321_ADDR){
		umts_board_max7321_client = NULL;
	}else if(client->addr == btwlancamera_board_MAX7321_ADDR){
		btwlancamera_board_max7321_client = NULL;
	}else if(client->addr == WLAN8385CAMERA_BOARD_MAX7321_ADDR){
		wlan8385camera_board_max7321_client = NULL;
	}else if(client->addr == WLAN8686CAMERA_BOARD_MAX7321_ADDR){
		wlan8686camera_board_max7321_client = NULL;
	}else if(client->addr == SIEMENS_BASEBAND_MAX7321_ADDR){
		siemens_baseband_max7321_client = NULL;
	}

	kfree(i2c_get_clientdata(client));
	return 0;
}

int WLAN_board_read_max7321(void)
{
	if( wlan_board_max7321_client == NULL )	/*	No global client pointer?	*/
		return -1;
		
	return i2c_smbus_read_byte(wlan_board_max7321_client);
}

int WLAN_board_write_max7321(u8 value)
{
	if( wlan_board_max7321_client == NULL )	/*	No global client pointer?	*/
		return -1;
		
	return i2c_smbus_write_byte(wlan_board_max7321_client, value);
}

int Camera_board_read_max7321(void)
{
	if( camera_board_max7321_client == NULL )	/*	No global client pointer?	*/
		return -1;
		
	return i2c_smbus_read_byte(camera_board_max7321_client);
}

int Camera_board_write_max7321(u8 value)
{
	if( camera_board_max7321_client == NULL )	/*	No global client pointer?	*/
		return -1;
		
	return i2c_smbus_write_byte(camera_board_max7321_client, value);
}

int UMTS_board_read_max7321(void)
{
	if( umts_board_max7321_client == NULL )	/*	No global client pointer?	*/
		return -1;
		
	return i2c_smbus_read_byte(umts_board_max7321_client);
}

int UMTS_board_write_max7321(u8 value)
{
	if( umts_board_max7321_client == NULL )	/*	No global client pointer?	*/
		return -1;
		
	return i2c_smbus_write_byte(umts_board_max7321_client, value);
}

int btwlancamera_board_read_max7321(void)
{
	if( btwlancamera_board_max7321_client == NULL )	/*	No global client pointer?	*/
		return -1;
		
	return i2c_smbus_read_byte(btwlancamera_board_max7321_client);
}

int btwlancamera_board_write_max7321(u8 value)
{
	if( btwlancamera_board_max7321_client == NULL )	/*	No global client pointer?	*/
		return -1;
		
	return i2c_smbus_write_byte(btwlancamera_board_max7321_client, value);
}

int Siemens_baseband_read_max7321(void) /* MFB */
{
	if ( siemens_baseband_max7321_client == NULL )		/*	No global client pointer?	*/
		return -1;
	return i2c_smbus_read_byte(siemens_baseband_max7321_client);
}

int Siemens_baseband_write_max7321(u8 value) /* MFB */
{
	if ( siemens_baseband_max7321_client == NULL )		/*	No global client pointer?	*/
		return -1;
	return i2c_smbus_write_byte(siemens_baseband_max7321_client, value);
}

int WLAN_board_read_revision_id(void)
{
	int temp, revid = 0;
	temp = WLAN_board_read_max7321();

	if(temp == -1)
		return -1;

	revid = (temp >> 4) | 0x07;

	return revid;
}

int wlan8385camera_board_read_revision_id(void)
{
	int temp, revid = 0;
	temp = wlan8385camera_board_read_max7321();

	if(temp == -1)
		return -1;

	revid = (temp >> 4) | 0x07;

	return revid;
}
int wlan8686camera_board_read_revision_id(void)
{
	int temp, revid = 0;
	temp = wlan8686camera_board_read_max7321();

	if(temp == -1)
		return -1;

	revid = (temp >> 4) | 0x07;

	return revid;
}

int Camera_board_read_revision_id(void)
{
	int temp, revid = 0;
	temp = Camera_board_read_max7321();

	if(temp == -1)
		return -1;

	revid = (temp >> 4) | 0x07;

	return revid;
}


int UMTS_board_read_revision_id(void)
{
	int temp, revid = 0;
	temp = UMTS_board_read_max7321();

	if(temp == -1)
		return -1;

	revid = (temp >> 4) | 0x07;

	return revid;
}

int btwlancamera_board_read_revision_id(void)
{
	int temp, revid = 0;
	temp = btwlancamera_board_read_max7321();

	if(temp == -1)
		return -1;

	revid = (temp >> 4) | 0x07;

	return revid;
}

int Siemens_baseband_read_revision_id(void) /* MFB */
{
	int temp, revid = 0;
	temp = Siemens_baseband_read_max7321();

	if(temp == -1)
		return -1;

	revid = (temp >> 4) | 0x07;

	return revid;
}

/* The First 4 bits are write only so we are using a shadow register */
int wlan8385camera_board_read_max7321(void)
{
	int val;
	if( wlan8385camera_board_max7321_client == NULL )
		return -1;
		
	val = i2c_smbus_read_byte(wlan8385camera_board_max7321_client);
	val &= ~wlan8385camera_board_shadow_mask;
	val |= (wlan8385camera_board_shadow_reg & wlan8385camera_board_shadow_mask);
	return val;
}

/* The First 4 bits are write only so we are using a shadow register */
int wlan8385camera_board_write_max7321(u8 value)
{
	if( wlan8385camera_board_max7321_client == NULL )
		return -1;
	wlan8385camera_board_shadow_reg = value;	
	return i2c_smbus_write_byte(wlan8385camera_board_max7321_client, value);
}
int wlan8686camera_board_read_max7321(void)
{
	int val;
	if( wlan8686camera_board_max7321_client == NULL )
		return -1;
		
	val = i2c_smbus_read_byte(wlan8686camera_board_max7321_client);
	val &= ~wlan8686camera_board_shadow_mask;
	val |= (wlan8686camera_board_shadow_reg & wlan8385camera_board_shadow_mask);
	return val;
}

/* The First 4 bits are write only so we are using a shadow register */
int wlan8686camera_board_write_max7321(u8 value)
{
	if( wlan8686camera_board_max7321_client == NULL )
		return -1;
	wlan8686camera_board_shadow_reg = value;	
	return i2c_smbus_write_byte(wlan8686camera_board_max7321_client, value);
}

/*Technology_board_detect
This function will return which technology boards are on main board, 
indicated by bit 0 ~ 3 of return value.
See liitleton.h

Bit 0:
	1 (indicate WLAN technology board detected)
	0 (indicate WLAN technology board not detected)
Bit 1:
	1 (indicate camera technology board detected)
	0 (indicate camera technology board not detected)
Bit 2:
	1 (indicate UMTS technology board detected)
	0 (indicate UMTS technology board not detected)
Bit 3:
	1 (indicate BTWLANCamera technology board detected)
	0 (indicate BTWLANCamera technology board not detected)

Bit 4:
	1 (indicate wlancamera technology board detected)
	0 (indicate wlancamera technology board not detected)

Bit 5~7: ignored
*/
u8 Technology_board_detect(void)
{
	u8 temp = 0x00;
	
	if(wlan_board_max7321_client != NULL){
		temp = temp | WLAN_BOARD;
	}

	if(camera_board_max7321_client != NULL){
		temp = temp | CAMERA_BOARD;
	}	
	
	if(umts_board_max7321_client != NULL){
		temp = temp | UMTS_BOARD;
	}

	if(btwlancamera_board_max7321_client != NULL){
		temp = temp | BTWLANCAMERA_BOARD;
	}	
	if(wlan8385camera_board_max7321_client != NULL){
		temp = temp | WLAN8385CAMERA_BOARD;
	}	
	if(wlan8686camera_board_max7321_client != NULL){
		temp = temp | WLAN8686CAMERA_BOARD;
	}	

	if(siemens_baseband_max7321_client != NULL){ /* MFB */
		temp = temp | SIEMENS_BOARD;
	}	
 
	return temp;
}

EXPORT_SYMBOL_GPL(WLAN_board_read_max7321);
EXPORT_SYMBOL_GPL(WLAN_board_write_max7321);
EXPORT_SYMBOL_GPL(Camera_board_read_max7321);
EXPORT_SYMBOL_GPL(Camera_board_write_max7321);
EXPORT_SYMBOL_GPL(UMTS_board_read_max7321);
EXPORT_SYMBOL_GPL(UMTS_board_write_max7321);
EXPORT_SYMBOL_GPL(WLAN_board_read_revision_id);
EXPORT_SYMBOL_GPL(Camera_board_read_revision_id);
EXPORT_SYMBOL_GPL(UMTS_board_read_revision_id);
EXPORT_SYMBOL_GPL(wlan8385camera_board_read_max7321);
EXPORT_SYMBOL_GPL(wlan8385camera_board_write_max7321);
EXPORT_SYMBOL_GPL(wlan8385camera_board_read_revision_id);
EXPORT_SYMBOL_GPL(wlan8686camera_board_read_max7321);
EXPORT_SYMBOL_GPL(wlan8686camera_board_write_max7321);
EXPORT_SYMBOL_GPL(wlan8686camera_board_read_revision_id);
EXPORT_SYMBOL_GPL(Technology_board_detect);

EXPORT_SYMBOL_GPL(Siemens_baseband_read_revision_id); /* MFB */
EXPORT_SYMBOL_GPL(Siemens_baseband_read_max7321); /* MFB */
EXPORT_SYMBOL_GPL(Siemens_baseband_write_max7321); /* MFB */

static int __init max7321_init(void)
{
	return i2c_add_driver(&max7321_driver);
}

static void __exit max7321_exit(void)
{
	i2c_del_driver(&max7321_driver);
}

MODULE_AUTHOR("Mingliang <mingliang.hu@marvell.com>");
MODULE_DESCRIPTION("MAX7321 driver");
MODULE_LICENSE("GPL");

module_init(max7321_init);
module_exit(max7321_exit);

