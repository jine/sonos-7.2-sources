/*
 * Copyright (c) 2015-2016, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 */

#include "sonos_common.h"

#define DATA_BUFFER_LEN 4096

static int ubi_start(const sonos_enfs *config)
{
	int fd;
	int ret;
	struct virtual_block_ioctl msg;
	msg.rootfs_size = config->rootfs_size;
	msg.rootfs_name_len = strlen(config->partition);
	memcpy(msg.rootfs_ubi_name, config->partition,
		msg.rootfs_name_len);

	fd = open(config->block_dev, O_RDWR);
	if ( fd < 0 ) {
		syslog(LOG_ERR, "open %s fail\n", config->block_dev);
		return 0;
	}
	ret = ioctl(fd, VIRTUAL_BLOCK_SET_CONFIG, &msg);
	if ( ret < 0 ) {
		syslog(LOG_ERR, "VIRTUAL_BLOCK_SET_CONFIG error %d\n", ret);
	}
	close(fd);
	return ret;
}

static int ubi_commit(const sonos_enfs *config)
{
	int fd;
	int ret;

	fd = open(config->block_dev, O_RDWR);
	if ( fd < 0 ) {
		syslog(LOG_ERR, "open %s fail\n", config->block_dev);
		return 1;
	}

	ret = ioctl(fd, VIRTUAL_BLOCK_COMMIT, NULL);
	if ( ret < 0 ) {
		syslog(LOG_ERR, "VIRTUAL_BLOCK_COMMIT error %d\n", ret);
	}
	close(fd);
	return ret;
}

static int ubi_check(const sonos_enfs *config, unsigned long len)
{
	int fd;
	int ret = 0;
	unsigned long read_len = 0;
	int timeout_count = 0;

	if ( len == 0 ) {
		return 0;
	}

	fd = open(config->block_dev, O_RDWR);
	if ( fd < 0 ) {
		syslog(LOG_ERR, "open %s fail\n", config->block_dev);
		return 1;
	}
	while (1) {
		ret = ioctl(fd, VIRTUAL_BLOCK_CHECK, &read_len);
		if ( ret < 0 ) {
			syslog(LOG_ERR, "VIRTUAL_BLOCK_CHECK error %d\n", ret);
			break;
		}

		if ( read_len == len ) {
			ret = 0;
			break;
		}
		sleep(1);
		timeout_count ++;
		if ( timeout_count > 60 ) {
			ret = 1;
			syslog(LOG_ERR, "Can't complete rootfs write\n");
			break;
		}
	}
	close(fd);
	return ret;
}

static int ubi_stop(const sonos_enfs *config)
{
	int fd;
	int ret;

	fd = open(config->block_dev, O_RDWR);
	if ( fd < 0 ) {
		syslog(LOG_ERR, "open %s fail\n", config->block_dev);
		return 1;
	}
	ret = ioctl(fd, VIRTUAL_BLOCK_COMPLETE, NULL);
	if ( ret < 0 ) {
		syslog(LOG_ERR, "UBI_IOCVOLUP error %d\n", ret);
	}
	close(fd);
	return ret;
}

int ubi_update(const sonos_enfs *config)
{
	int fd, out_fd;
	int ret = 0, count = 0;
	int bytes_read = 0;
	char buffer[DATA_BUFFER_LEN];

	memset(buffer, 0, DATA_BUFFER_LEN);
	snprintf(buffer, DATA_BUFFER_LEN, "/dev/mapper/%s", config->virtual_dev);

	out_fd = open(buffer, O_RDWR | O_SYNC);
	if ( out_fd < 0 ) {
		syslog(LOG_ERR, "open %s fail\n", buffer);
		return 1;
	}
	if ( config->rootfs_name ) {
		fd = open(config->rootfs_name, O_RDWR);
		if ( fd < 0 ) {
			close(out_fd);
			syslog(LOG_ERR, "open %s fail\n", config->rootfs_name);
			return 1;
		}
		ubi_start(config);
		while ( 1 ) {
			bytes_read = read(fd, buffer, DATA_BUFFER_LEN);
			if ( bytes_read < 0 ) {
				syslog(LOG_ERR, "read %s fail\n", config->rootfs_name);
				count = 0;
				break;
			} else if ( bytes_read ) {
				ret = write(out_fd, buffer, bytes_read);
				if (ret == bytes_read) {
					count += bytes_read;
				} else {
					syslog(LOG_ERR, "write of %d bytes failed:  %d\n", bytes_read, ret);
					break;
				}
			} else {
				break; /* end of read */
			}
		}
		close(fd);
	} else {
		syslog(LOG_INFO, "Using stdin\n");
		while ( 1 ) {
			bytes_read = read(0, buffer, DATA_BUFFER_LEN);
			if ( bytes_read < 0 ) {
				syslog(LOG_ERR, "%d failure reading from stdin\n", bytes_read);
				count = 0;
				break;
			} else if ( bytes_read ) {
				ret = write(out_fd, buffer, bytes_read);
				if (ret == bytes_read) {
					count += bytes_read;
				} else {
					syslog(LOG_ERR, "write of %d bytes failed:  %d\n", bytes_read, ret);
					break;
				}
			} else {
				break; /* end of read */
			}
			if ( config->rootfs_size == count ) {
				ret = 0;
				break;
			}
		}
	}
	if ( count ) {
		ret = ubi_check(config, count);
	}
	ret |= ubi_commit(config);
	ret |= ubi_stop(config);
	close(out_fd);
	return ret;
}
