#ifndef ASMARM_ARCH_PXA3xx_KEYPAD_H
#define ASMARM_ARCH_PXA3xx_KEYPAD_H

#include <linux/input.h>

#define MAX_MATRIX_KEY_ROWS	(8)
#define MAX_MATRIX_KEY_COLS	(8)
#define MAX_DIRECT_KEYS		(8)

struct pxa3xx_keypad_platform_data {
	int enable_repeat_key;
	/* pxa3xx keypad platform specific parameters */
	int enable_matrix_key;
	int enable_direct_key;
	int enable_rotary_key;

	int matrix_key_debounce;
	int direct_key_debounce;

	int matrix_key_rows;
	int matrix_key_cols;
	int direct_key_num;

	/* code map for the matrix keys */
	unsigned int *matrix_key_map;
	int matrix_key_map_size;

	/* code map for the direct keys */
	unsigned int *direct_key_map;
	int direct_key_map_size;

	/* key code for the rotary up/down/push keys */
	unsigned int rotary_up_key;
	unsigned int rotary_down_key;
};

#define KEY(row, col, val) (((row) << 28) | ((col) << 24) | (val))

extern void pxa_set_keypad_info(void *);

#endif
