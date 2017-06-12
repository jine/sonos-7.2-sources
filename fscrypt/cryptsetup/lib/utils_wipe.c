/*
 * utils_wipe - wipe a device
 *
 * Copyright (C) 2004-2007, Clemens Fruhwirth <clemens@endorphin.org>
 * Copyright (C) 2011-2012, Red Hat, Inc. All rights reserved.
 * Copyright (C) 2009-2012, Milan Broz
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include "libcryptsetup.h"
#include "internal.h"
#include "sonos_common.h"

#define MAXIMUM_WIPE_BYTES	1024 * 1024 * 32 /* 32 MiB */

static ssize_t _crypt_wipe_zero(int fd, int bsize, char *buffer,
				uint64_t offset, uint64_t size)
{
	memset(buffer, 0, size);
	return write_lseek_blockwise(fd, bsize, buffer, size, offset);
}

int crypt_wipe(struct device *device, uint64_t offset, uint64_t size)
{
	struct stat st;
	char *buffer;
	int devfd, flags, bsize;
	ssize_t written;

	if (!size || size % SECTOR_SIZE || (size > MAXIMUM_WIPE_BYTES)) {
		syslog(LOG_ERR, "Unsuported wipe size for device %s: %ld.",
			device_path(device), (unsigned long)size);
		return -EINVAL;
	}

	if (stat(device_path(device), &st) < 0) {
		syslog(LOG_ERR, "Device %s not found.", device_path(device));
		return -EINVAL;
	}

	bsize = device_block_size(device);
	if (bsize <= 0)
		return -EINVAL;

	buffer = malloc(size);
	if (!buffer)
		return -ENOMEM;

	flags = O_RDWR;

	/* use O_EXCL only for block devices */
	if (S_ISBLK(st.st_mode))
		flags |= O_EXCL;

	/* coverity[toctou] */
	devfd = device_open(device, flags);
	if (devfd == -1) {
		free(buffer);
		return errno ? -errno : -EINVAL;
	}

	written = _crypt_wipe_zero(devfd, bsize, buffer, offset, size);

	close(devfd);
	free(buffer);

	if (written != (ssize_t)size || written < 0)
		return -EIO;

	return 0;
}
