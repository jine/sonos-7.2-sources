/*
 * Copyright (c) 2015, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 */

#include "sonos_common.h"

static sonos_enfs enfs;

#define VIRTUAL_BLOCK_DEV	"/dev/virtualblock1";
#define MAPPER_DEV		"upgrade";

static const char usage[] =
"Usage: \n\t" PROGRAM_NAME " \t\t[-h]";

static const char optionsstr[] =
"-h, --help                               print help message\n";

static const struct option long_options[] = {
	{ .name = "help",           .has_arg = 0, .flag = NULL, .val = 'h' },
	{ NULL, 0, NULL, 0}
};

static int parse_opt(int argc, char * const argv[])
{
	while (1) {
		int key;
		key = getopt_long(argc, argv, "h", long_options, NULL);
		if ( key == -1 )
			break;
		switch (key) {
			case 'h':
				printf("%s\n\n", usage);
				printf("%s\n", optionsstr);
				exit(EXIT_SUCCESS);
			default:
				printf("Use %s -h for help\n", PROGRAM_NAME);
				return -1;
		}
	}
	return 0;
}

int main(int argc, char** argv)
{
	int err = -1;
	int ret = 0;

	memset(&enfs, 0, sizeof(enfs));
	enfs.block_dev = VIRTUAL_BLOCK_DEV;
	enfs.virtual_dev = MAPPER_DEV;

	setlogmask(LOG_UPTO(LOG_INFO));
	//setmaxlogsize(80 * 1024);
	//openlog(PROGRAM_NAME,  0, LOG_USER);
	syslog(LOG_INFO, "****************************");

	err = parse_opt(argc, argv);
	if (err) {
		ret = -1;
		goto end;
	}

	ret = sonos_action_luksFormat(&enfs);
	if (ret) {
		syslog(LOG_ERR, "Format error %d\n", ret);
		goto end;
	}
	ret = sonos_action_open_luks(&enfs);
	if (ret) {
		syslog(LOG_ERR, "Open error %d\n", ret);
		goto end;
	}
	ret = ubi_update(&enfs);
	if (ret) {
		syslog(LOG_ERR, "UBI update %d\n", ret);
	}
	ret |= action_close(&enfs);
	if (ret) {
		syslog(LOG_ERR, "Close %d\n", ret);
	}

end:
	closelog();
	return ret;
}
