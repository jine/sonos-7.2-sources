#ifndef _WOODSTOCK_IOCTL_H_
#define _WOODSTOCK_IOCTL_H_

// /dev/da9034_misc
#define DA9034_MISC_IOCTL_GET_VBAT	_IOR('d',0x01,int)
#define DA9034_MISC_IOCTL_GET_ICHG	_IOR('d',0x02,int)
#define DA9034_MISC_IOCTL_GET_VCHG	_IOR('d',0x03,int)
#define DA9034_MISC_IOCTL_GET_TBAT	_IOR('d',0x04,int)
#define DA9034_MISC_IOCTL_GET_LIGHT	_IOR('d',0x05,int)
#define DA9034_MISC_IOCTL_GET_TEMP	_IOR('d',0x06,int)
#define DA9034_MISC_IOCTL_SET_BUTTON_BACKLIGHT	_IO('d',0x07)
#define DA9034_MISC_IOCTL_GET_BUTTON_BACKLIGHT	_IOR('d',0x08,int)
#define DA9034_MISC_IOCTL_GET_STATE	_IOR('d',0x09,unsigned int)
#define DA9034_MISC_IOCTL_SET_CHARGER	_IO('d',0x0a)
#define DA9034_MISC_IOCTL_SET_CHARGE_LED	_IO('d',0x0b)
#define DA9034_MISC_IOCTL_SET_CHARGE_LED_EXTENDED	_IO('d',0x0c)

#define DA9034_STATE_DC_PRESENT 0x01
#define DA9034_STATE_CHARGING 0x02
#define DA9034_STATE_TBAT_FAULT	0x04
#define DA9034_STATE_HIGHDC_PRESENT 0x08
#define DA9034_STATE_CHARGER_DISABLED 0x10

#define DA9034_CHARGER_ENABLE 1
#define DA9034_CHARGER_DISABLE 2

#define DA9034_CHARGE_LED_ENABLE 1
#define DA9034_CHARGE_LED_DISABLE 2

// /dev/fb

#define PXAFB_DBFR_FLIP         _IOW('F', 0x85, unsigned int)

// /dev/pm

#define PMIOC_SLEEP _IO('p',0x01)
#define SF_MOTION                       0x00000001
#define SF_HIBERNATE                    0x00000002
#define SF_KEY                          0x00000004
// #define SF_WAKE_LIGHT                   0x00000008
// #define SF_WAKE_BEEP                    0x00000010
#define SF_TOUCH			0x00000020

#define PMIOC_SET_TIMEOUT _IO('p',0x02)
#define PMIOC_GET_WAKE_REASON _IOR('p',0x03,unsigned int)

#define WAKE_REASON_NONE        0
#define WAKE_REASON_KEY         1
#define WAKE_REASON_MOTION      2
#define WAKE_REASON_DOCK        3
#define WAKE_REASON_TIMEOUT     4
// #define WAKE_REASON_HOST        5
#define WAKE_REASON_BAT		6
#define WAKE_REASON_TOUCH	7


#endif
