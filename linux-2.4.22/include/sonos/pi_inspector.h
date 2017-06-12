// SPDX-License-Identifier: GPL-2.0+

#ifndef PI_INSPECTOR_H
#define PI_INSPECTOR_H

//
// Function prototypes
// *******************
//

typedef void (*pi_hook_fn_t)(int);
extern pi_hook_fn_t PI_hook_fp;

#define PI_DISPATCH 1
#define PI_EXIT_NOTIFY 2

#endif
