/* linux/drivers/input/keyboard/s3c-keypad.h
 *
 * Driver header for Samsung SoC keypad.
 *
 * Kim Kyoungil, Copyright (c) 2006-2009 Samsung Electronics
 *      http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef _S3C_KEYPAD_H_
#define _S3C_KEYPAD_H_

static void __iomem *key_base;

#if defined(CONFIG_KEYPAD_S3C_MSM)
#define KEYPAD_COLUMNS 	8
#define KEYPAD_ROWS		14
#define MAX_KEYPAD_NR	112 /* 8*14 */
#define MAX_KEYMASK_NR	56
#else
/*
#define KEYPAD_COLUMNS  8
#define KEYPAD_ROWS 8
#define MAX_KEYPAD_NR   64 
#define MAX_KEYMASK_NR  32
*/
#define KEYPAD_COLUMNS  2
#define KEYPAD_ROWS 3
#define MAX_KEYPAD_NR   64  /* 8*8 */
#define MAX_KEYMASK_NR  32
#endif

int keypad_keycode[] = {
		1,2,3,4,5,6,7,8,
		9,10,11,12,13,14,15,16,
		17,18,19,20,21,22,23,24,
		25,26,27,28,29,30,31,32,
		33,34,35,36,37,38,39,40,
		41,42,43,44,45,46,47,48,
		49,50,51,52,53,54,55,56,
		57,58,59,60,61,62,63,64

};

#if defined(CONFIG_CPU_S3C6410)
#define KEYPAD_DELAY		(50)
#elif defined(CONFIG_CPU_S5PC100)
#define KEYPAD_DELAY		(600)
#elif defined(CONFIG_CPU_S5PV210)
#define KEYPAD_DELAY		(900)
#elif defined(CONFIG_CPU_S5P6442)
#define KEYPAD_DELAY		(50)
#endif

#define	KEYIFCOL_CLEAR		(readl(key_base+S3C_KEYIFCOL) & ~0xffff)
#define	KEYIFCON_CLEAR		(readl(key_base+S3C_KEYIFCON) & ~0x1f)
#define	KEYIFFC_DIV			(readl(key_base+S3C_KEYIFFC) | 0x1)

struct s3c_keypad {
	struct input_dev *dev;
	int nr_rows;
	int no_cols;
	int total_keys;
	int keycodes[MAX_KEYPAD_NR];
};

#if defined(CONFIG_KEYPAD_S3C_MSM)
extern void s3c_setup_keypad_cfg_gpio(void);
#else
extern void s3c_setup_keypad_cfg_gpio(int rows, int columns);
#endif
#endif				/* _S3C_KEYIF_H_ */
