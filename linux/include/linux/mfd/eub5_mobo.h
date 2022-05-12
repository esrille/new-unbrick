/*
 * Esrille New Unbrick Multi-Function Driver
 *
 * Copyright (C) 2018-2022 Esrille Inc.
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
 * along with this program; if not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __LINUX_MFD_EUB5_MOBO_H
#define __LINUX_MFD_EUB5_MOBO_H

/*
 * ----------------------------------------------------------------------------
 * Registers, all 8 bits
 * ----------------------------------------------------------------------------
 */

/* Register definitions */
#define EUB_MOBO_REG_VENDOR		0x00
#define EUB_MOBO_REG_ID			0x01
#define EUB_MOBO_REG_VERSION		0x02
#define EUB_MOBO_REG_REVISION		0x03
#define EUB_MOBO_REG_BRIGHTNESS		0x04
#define EUB_MOBO_REG_DISPLAY		0x05
#define EUB_MOBO_REG_BUTTONS		0x06
#define EUB_MOBO_REG_X_LOW		0x07
#define EUB_MOBO_REG_X_HIGH		0x08
#define EUB_MOBO_REG_Y_LOW		0x09
#define EUB_MOBO_REG_Y_HIGH		0x0A
#define EUB_MOBO_REG_Z_LOW		0x0B
#define EUB_MOBO_REG_Z_HIGH		0x0C
#define EUB_MOBO_REG_VREF		0x0D
#define EUB_MOBO_REG_CC1		0x0E
#define EUB_MOBO_REG_CC2		0x0F
#define EUB_MOBO_REG_SECONDS		0x10
#define EUB_MOBO_REG_MINUTES		0x11
#define EUB_MOBO_REG_HOURS		0x12
#define EUB_MOBO_REG_WEEKDAY		0x13
#define EUB_MOBO_REG_DAY		0x14
#define EUB_MOBO_REG_MONTH		0x15
#define EUB_MOBO_REG_YEAR		0x16
#define EUB_MOBO_REG_RTC_CONTROL	0x17
#define EUB_MOBO_REG_MAX		0x18

#define EUB_MOBO_I2C_RETRY		0x01
#define EUB_MOBO_I2C_CRC		0x02	/* use CRC-8 */
#define EUB_MOBO_I2C_CRC_JAM		0x80	/* for testing purpose */

struct eub5_mobo_dev {
	unsigned char version;
	struct device *dev;
	struct i2c_client *i2c_client;
	int (*read_dev)(struct eub5_mobo_dev *eub_mobo, char reg, int size, void *dest, u8 flags);
	int (*write_dev)(struct eub5_mobo_dev *eub_mobo, char reg, int size, void *src, u8 flags);
};

#endif /*  __LINUX_MFD_EUB5_MOBO_H */
