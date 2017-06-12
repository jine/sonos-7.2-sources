/*
 * Copyright (c) 2015, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 */

#ifndef __SONOS_COMMON_H__
#define __SONOS_COMMON_H__
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/fs.h>
#include <syslog.h>

#include "virtual_block_ioctl.h"

#define PROGRAM_NAME "encryptfs"

#define _LOG_DEBUG 7
#define _LOG_INFO 6
#define _LOG_NOTICE 5
#define _LOG_WARN 4
#define _LOG_ERR 3
#define _LOG_FATAL 2

typedef struct _sonos_enfs_ {
	const char * block_dev;
	const char * partition;
	const char * rootfs_name;
	int rootfs_size;
	const char * virtual_dev;
} sonos_enfs;

#if 0
#define stack syslog(LOG_INFO, "<backtrace>")
#define goto_bad do { stack; goto bad; } while (0)
#define return_0 do { stack; return 0; } while (0)
#define return_NULL do { stack; return NULL; } while (0)
#define log_sys_error(x, y) \
		syslog(LOG_ERR, "%s: %s failed: %s", y, x, strerror(errno))
#define log_sys_debug(x, y) \
		syslog(LOG_ERR, "%s: %s failed: %s", y, x, strerror(errno))
#else
#define stack       do {} while (0)
#define goto_bad    do {goto bad;} while (0)
#define return_0    do {return 0;} while (0)
#define return_NULL do {return NULL;} while (0)
#define log_sys_error(x, y) do {} while (0)
#define log_sys_debug(x, y) do {} while (0)
#endif

int sonos_action_luksFormat(const sonos_enfs *config);
int sonos_action_open_luks(const sonos_enfs *config);
int ubi_update(const sonos_enfs *config);
int action_close(const sonos_enfs *config);

#define DM_LIB_VERSION "1.02.77 (2012-10-15)"
#define DEFAULT_DM_NAME_MANGLING DM_STRING_MANGLING_AUTO
#define DEFAULT_LUKS1_HASH "sha1"
#define DEFAULT_LUKS1_KEYBITS 256
#endif
