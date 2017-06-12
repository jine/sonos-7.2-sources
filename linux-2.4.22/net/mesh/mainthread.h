#ifndef MAINTHREAD_H
#define MAINTHREAD_H

#include <linux/kernel.h>
#include <linux/module.h>

void start_main_thread(void);
void stop_main_thread(void);
void wakeup_main_thread(void);

#endif
