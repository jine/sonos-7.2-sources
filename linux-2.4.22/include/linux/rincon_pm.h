#ifndef _LINUX_RINCON_PM_H
#define _LINUX_RINCON_PM_H

#define PMIOC_SLEEP 1

struct pmdevice {
	struct pmdevice *next;
	char name[16];
	void (*sleep) (struct pmdevice *);
	void (*wake) (struct pmdevice *);
	void *priv;
};

struct sleep_parms {
	unsigned int timeout; /*in seconds*/
	unsigned int flags;
};
#define SF_MOTION			0x00000001
#define SF_HIBERNATE			0x00000002
#define SF_KEY				0x00000004
#define SF_WAKE_LIGHT			0x00000008
#define SF_WAKE_BEEP			0x00000010

int rincon_pm_register(struct pmdevice *);
int rincon_pm_unregister(struct pmdevice *);

//int pm_register(struct pmdevice *);
//int pm_unregister(struct pmdevice *);

extern void (*m16_sleep) (struct sleep_parms *);
extern void (*m16_wake) (struct sleep_parms *);

#endif /*_LINUX_RINCON_PM_H*/
