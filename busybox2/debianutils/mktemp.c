/* vi: set sw=4 ts=4: */
/*
 * Mini mktemp implementation for busybox
 *
 *
 * Copyright (C) 2000 by Daniel Jacobowitz
 * Written by Daniel Jacobowitz <dan@debian.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include "busybox.h"

extern int mktemp_main(int argc, char **argv)
{
	unsigned char dir_flag = 0;
	int opt;

	while ((opt = getopt(argc, argv, "qd")) != -1) {
		if (opt == 'd') {
			dir_flag = 1;
		}
		else if (opt != 'q') {
			bb_show_usage();
		} 
	}

	if (optind + 1 != argc) {
		bb_show_usage();
	}

	if (dir_flag) {
		if (mkdtemp(argv[argc-1]) == NULL) {
			return EXIT_FAILURE;
		}
	} else {
		if (mkstemp(argv[argc-1]) < 0) {
			return EXIT_FAILURE;
		}
	}

	(void) puts(argv[argc-1]);

	return EXIT_SUCCESS;
}
