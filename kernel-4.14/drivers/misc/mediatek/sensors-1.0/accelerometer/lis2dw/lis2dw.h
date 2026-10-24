/* linux/drivers/hwmon/LIS2DW.c
 *
 * (C) Copyright 2008
 * MediaTek <www.mediatek.com>
 *
 * LIS2DW driver for MT6516
 *
 *
 * LIS2DW driver for MT8167
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.
 */
#ifndef LIS2DW_H
#define LIS2DW_H

#include <linux/ioctl.h>

extern struct acc_hw *lis2dw_get_cust_acc_hw(void);

#define LIS2DW_I2C_SLAVE_ADDR		0x30/* 0x30<-> SD0=GND;0x32<-> SD0=High */

/* LIS2DW Register Map  (Please refer to LIS2DW Specifications) */
#define LIS2DW_REG_CTL_REG1		0x20
#define LIS2DW_REG_CTL_REG2		0x21
#define LIS2DW_REG_CTL_REG3		0x22
#define LIS2DW_REG_CTL_REG4		0x23
#define LIS2DW_REG_CTL_REG6		0x25
#define LIS2DW_REG_REFERENCE		0x26
#define LIS2DW_REG_DATAX0		    0x28
#define LIS2DW_REG_OUT_X		    0x28
#define LIS2DW_REG_OUT_Y		    0x2A
#define LIS2DW_REG_OUT_Z		    0x2C

#define LIS2DW_FIXED_DEVID			0xE5

#define LIS2DW_BW_200HZ			0x60
#define LIS2DW_BW_100HZ			0x50 /* 400 or 100 on other choise //changed */
#define LIS2DW_BW_50HZ			0x40

#define	LIS2DW_FULLRANG_LSB		0XFF

#define LIS2DW_MEASURE_MODE		0x0C	/* changed */
#define LIS2DW_DATA_READY		0x07    /* changed */

#define LIS2DW_HIGH_RES			0x08
#define LIS2DW_RANGE_2G			0x00
#define LIS2DW_RANGE_4G			0x10
#define LIS2DW_RANGE_8G			0x20
#define LIS2DW_RANGE_16G		0x30

#define LIS2DW_SELF_TEST           0x10 /* changed */

#define LIS2DW_STREAM_MODE			0x80
#define LIS2DW_SAMPLES_15			0x0F

#define LIS2DW_FS_8G_LSB_G			0x20
#define LIS2DW_FS_4G_LSB_G			0x10
#define LIS2DW_FS_2G_LSB_G			0x00

#define LIS2DW_LEFT_JUSTIFY		0x04
#define LIS2DW_RIGHT_JUSTIFY		0x00


#define LIS2DW_SUCCESS						0
#define LIS2DW_ERR_I2C						-1
#define LIS2DW_ERR_STATUS					-3
#define LIS2DW_ERR_SETUP_FAILURE			-4
#define LIS2DW_ERR_GETGSENSORDATA			-5
#define LIS2DW_ERR_IDENTIFICATION			-6



#define LIS2DW_BUFSIZE				256

#endif

