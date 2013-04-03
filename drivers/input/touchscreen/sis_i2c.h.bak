/*
 * include/linux/sis_i2c.h - platform data structure for SiS 81x/9200 family
 *
 * Copyright (C) 2009 SiS, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_SIS_I2C_H
#define _LINUX_SIS_I2C_H

#define SIS_I2C_NAME "sis_i2c_ts"

struct sis_i2c_rmi_platform_data {
	uint32_t version;	/* Use this entry for panels with */
				/* (major << 8 | minor) version or above. */
				/* If non-zero another array entry follows */
	int (*power)(int on);	/* Only valid in first array entry */
	uint32_t flags;
	unsigned long irqflags;
	uint32_t inactive_left; /* 0x10000 = screen width */
	uint32_t inactive_right; /* 0x10000 = screen width */
	uint32_t inactive_top; /* 0x10000 = screen height */
	uint32_t inactive_bottom; /* 0x10000 = screen height */
	uint32_t snap_left_on; /* 0x10000 = screen width */
	uint32_t snap_left_off; /* 0x10000 = screen width */
	uint32_t snap_right_on; /* 0x10000 = screen width */
	uint32_t snap_right_off; /* 0x10000 = screen width */
	uint32_t snap_top_on; /* 0x10000 = screen height */
	uint32_t snap_top_off; /* 0x10000 = screen height */
	uint32_t snap_bottom_on; /* 0x10000 = screen height */
	uint32_t snap_bottom_off; /* 0x10000 = screen height */
	uint32_t fuzz_x; /* 0x10000 = screen width */
	uint32_t fuzz_y; /* 0x10000 = screen height */
	int fuzz_p;
	int fuzz_w;
	int8_t sensitivity_adjust;
};


// For SiS9200 i2c data format
// Define if for SMBus Tx/Rx in X86.
// Undef it for I2C Tx/Rx in Embedded System.

//#define _SMBUS_INTERFACE
//#define _FOR_LEGACY_SIS81X

#define _I2C_INT_ENABLE		1

#ifdef _FOR_LEGACY_SIS81X
#define MAX_FINGERS			2
#else   // SiS9200
#define MAX_FINGERS			10
#endif

#define X_SENSE_LINE		38 //8
#define Y_SENSE_LINE		22 //12

#ifdef _FOR_LEGACY_SIS81X
// Fixed point mode in tracking algorothm mapping to 0-4095
#define SIS_MAX_X	        4095
#define SIS_MAX_Y	        4095
#else
// SiS9200 Resolution mode
//#define SIS_MAX_X		4095
//#define SIS_MAX_Y		4095
// SiS9200 Fixed point mode
#define SIS_MAX_X           (X_SENSE_LINE * 128)
#define SIS_MAX_Y	        (Y_SENSE_LINE * 128)
#endif

#define SIS_CMD_NORMAL							   0x0
#define SIS_CMD_RECALIBRATE		                   0x87
#define MAX_READ_BYTE_COUNT						   16
#define MSK_TOUCHNUM							   0x0f
#define MSK_HAS_CRC								   0x10
#define MSK_DATAFMT								   0xe0
#define MSK_PSTATE								   0x0f
#define MSK_PID	                                   0xf0
#define TOUCHDOWN								   0x0
#define TOUCHUP									   0x1
#define RES_FMT									   0x00
#define FIX_FMT									   0x40

#ifdef _SMBUS_INTERFACE
#define CMD_BASE	0
#else
#define CMD_BASE	1	//i2c
#endif

#define PKTINFO									   CMD_BASE + 1

#define P1TSTATE								   2

#define NO_T                                       0x02
#define SINGLE_T                                   0x09
#define MULTI_T                                    0x0e
#define LAST_ONE                                   0x07
#define LAST_TWO                                   0x0c

#define CRCCNT(x) ((x + 0x1) & (~0x1))

#endif /* _LINUX_SIS_I2C_H */
