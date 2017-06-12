/*
 * Copyright (c) 2015, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * common/include/sonos_lock.h
 */

#ifndef SONOS_LOCK_H
#define SONOS_LOCK_H

#define CONSTRAINED_MODE 0
#define NORMAL_MODE 1
#define UNCONSTRAINED_MODE 2

/*
 * watchdog states
 */
#define KERNEL_DEFAULT		0
#define USERSPACE_SERVICE	1
#define KERNEL_SERVICE		2

#define KERNEL_SERVICE_MAX	4

/*
 * How many times to pet the WDT per timeout.  This is currently
 * 4 rather than 2 to allow for inaccuracy of the timebase when the
 * 32kHz crystal is not populated, in which case the timebase frequency
 * may be anywhere in the range ~20 - 60kHz
 */
#define WDT_PETS_PER_TIMEOUT	4

#ifndef	SONOS_LOCK_C
// All of these functions are defined in sonos_lock.c - if that's
// not the current compiling file, extern them...
extern int sonos_forbid_execution(void);
extern int sonos_allow_execution(void);
extern int proc_noexec_set_constrained(void);
extern int sonos_forbid_insmod(void);
extern int proc_insmod_set_constrained(void);
extern int sonos_force_nodev(void);
extern int proc_nodev_set_constrained(void);
#endif
extern int is_mdp_authorized(int);

#endif
