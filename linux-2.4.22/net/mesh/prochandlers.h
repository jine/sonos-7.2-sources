/*
 *   Mesh network routing
 *
 *      $Id: prochandlers.h,v 1.2 2004/01/08 19:20:43 vangool Exp $
 *
 */
#ifndef PROCHANDLERS_H
#define PROCHANDLERS_H

#include <linux/kernel.h>
#include <linux/module.h>

/* Setup the proc file system */
void proc_init(void);

/* Get rid of all our proc files */
void proc_deinit(void);

#endif
