/*
 * Esrille New Unbrick Motherboard Multi-Function Driver
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

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/mfd/core.h>

#include <linux/mfd/eub5_mobo.h>

#define MAX_RETRY	10
#define POLY		7U	/* x^8 + x^2 + x + 1 */

#define CMD_STANDARD	0x00
#define CMD_WRITE	0x40
#define CMD_READ	0x80
#define CMD_MASK	0xc0
#define CMD_ADDRESS	0x3f

static const struct mfd_cell eub5_mobo_devs[] = {
	{
		.name = "eub5_backlight",
		.of_compatible = "esrille,eub5_backlight",
	},
	{
		.name = "eub5_battery",
		.of_compatible = "esrille,eub5_battery",
	},
	{
		.name = "eub5_rtc",
		.of_compatible = "esrille,eub5_rtc",
	},
	{
		.name = "eub5_touch",
		.of_compatible = "esrille,eub5_touch",
	},
};

static u8 crc8(u8 crc)
{
	int i;

	for (i = 0; i < 8; ++i) {
		if (crc & 0x80) {
			crc <<= 1;
			crc ^= POLY;
		} else {
			crc <<= 1;
		}
	}
	return crc;
}

static u8 i2c_crc8(u8 crc, const u8 *p, int count)
{
	while (0 < count--) {
		crc = crc8(crc ^ *p++);
	}
	return crc;
}

static int i2c_client_read_device(struct i2c_client *i2c, char reg, int bytes, void *dest, u8 flags)
{
	int ret;
	struct i2c_msg xfer[2];

	if (bytes <= 0 || EUB_MOBO_REG_MAX < bytes)
		return -EINVAL;
	if (flags & EUB_MOBO_I2C_CRC) {
		u8 msg_write[3];
		/* one more byte for CRC */
		u8 msg_read[EUB_MOBO_REG_MAX + 1];
		u8 crc;

		/* write register number, length, and CRC */
		xfer[0].addr = i2c->addr;
		xfer[0].flags = 0;
		xfer[0].len = 3;
		xfer[0].buf = msg_write;
		msg_write[0] = CMD_READ | reg;
		msg_write[1] = bytes;
		msg_write[2] = crc = i2c_crc8(0, msg_write, 2);

		/* read data and CRC */
		xfer[1].addr = i2c->addr;
		xfer[1].flags = I2C_M_RD;
		xfer[1].len = bytes + 1;
		xfer[1].buf = msg_read;

		ret = i2c_transfer(i2c->adapter, xfer, 2);
		if (ret < 0)
			return ret;
		if (ret != 2)
			return -EIO;

		/* check CRC */
		crc = i2c_crc8(crc, msg_read, bytes);
		if (crc != msg_read[bytes])
			return -EBADMSG;

		/* copy back */
		memcpy(dest, msg_read, bytes);
	} else {
		dev_info(&i2c->dev, "i2c_client_read_device: standard\n");

		/* write register number */
		xfer[0].addr = i2c->addr;
		xfer[0].flags = 0;
		xfer[0].len = 1;
		xfer[0].buf = &reg;

		/* read data */
		xfer[1].addr = i2c->addr;
		xfer[1].flags = I2C_M_RD;
		xfer[1].len = bytes;
		xfer[1].buf = dest;

		ret = i2c_transfer(i2c->adapter, xfer, 2);
		if (ret < 0)
			return ret;
		if (ret != 2)
			return -EIO;
	}
	return 0;
}

static int i2c_client_write_device(struct i2c_client *i2c, char reg, int bytes, void *src, u8 flags)
{
	int ret;
	/* three more bytes for register number, length, and CRC */
	u8 msg[EUB_MOBO_REG_MAX + 3];

	if (bytes <= 0 || EUB_MOBO_REG_MAX < bytes)
		return -EINVAL;
	if (flags & EUB_MOBO_I2C_CRC) {
		msg[0] = CMD_WRITE | reg;
		msg[1] = bytes;
		memcpy(&msg[2], src, bytes);
		bytes += 2;
		msg[bytes] = i2c_crc8((flags & EUB_MOBO_I2C_CRC_JAM) ? 1 : 0, msg, bytes);
		++bytes;
	} else {
		dev_info(&i2c->dev, "i2c_client_write_device: standard\n");

		msg[0] = reg;
		memcpy(&msg[1], src, bytes);
		++bytes;
	}
	ret = i2c_master_send(i2c, msg, bytes);
	if (ret == bytes)
		return 0;
	if (0 <= ret) {
		dev_info(&i2c->dev, "i2c_client_write_device EIO: %d;\n", ret);
		return -EIO;
	}
	dev_info(&i2c->dev, "i2c_client_write_device error: %d;\n", ret);
	return ret;
}

static int eub5_mobo_i2c_read_device(struct eub5_mobo_dev *eub5_mobo, char reg,
				     int bytes, void *dest, u8 flags)
{
	int ret;
	int i;

	if (!(flags & EUB_MOBO_I2C_RETRY))
		return i2c_client_read_device(eub5_mobo->i2c_client, reg, bytes, dest, flags);

	for (i = 0;; ++i) {
		ret = i2c_client_read_device(eub5_mobo->i2c_client, reg, bytes, dest, flags);
		if (ret == 0)
			break;
		if (MAX_RETRY - 1 <= i)
			return ret;
		msleep(1 << i);
	}
	return 0;
}

static int eub5_mobo_i2c_write_device(struct eub5_mobo_dev *eub5_mobo, char reg,
				      int bytes, void *src, u8 flags)
{
	int ret;
	int i;

	if (!(flags & EUB_MOBO_I2C_RETRY))
		return i2c_client_write_device(eub5_mobo->i2c_client, reg, bytes, src, flags);

	for (i = 0;; ++i) {
		ret = i2c_client_write_device(eub5_mobo->i2c_client, reg, bytes, src, flags);
		if (ret == 0)
			break;
		if (MAX_RETRY - 1 <= i)
			return ret;
		msleep(1 << i);
	}
	return 0;
}

static int eub5_mobo_i2c_probe(struct i2c_client *i2c,
			      const struct i2c_device_id *id)
{
	struct eub5_mobo_dev *eub5_mobo;
	u8 buffer[3];
	int ret;

	eub5_mobo = devm_kzalloc(&i2c->dev, sizeof(struct eub5_mobo_dev), GFP_KERNEL);
	if (eub5_mobo == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, eub5_mobo);
	eub5_mobo->dev = &i2c->dev;
	eub5_mobo->i2c_client = i2c;
	eub5_mobo->read_dev = eub5_mobo_i2c_read_device;
	eub5_mobo->write_dev = eub5_mobo_i2c_write_device;

	ret = eub5_mobo_i2c_read_device(eub5_mobo, EUB_MOBO_REG_VENDOR,
					sizeof(buffer), buffer,
					EUB_MOBO_I2C_RETRY | EUB_MOBO_I2C_CRC);
	if (ret != 0) {
		dev_err(eub5_mobo->dev, "not found: %d\n", ret);
		return -ENODEV;
	}

	dev_info(&i2c->dev, "vendor: %u, id: %u, version: %u\n",
		 buffer[EUB_MOBO_REG_VENDOR],
		 buffer[EUB_MOBO_REG_ID],
		 buffer[EUB_MOBO_REG_VERSION]);
	eub5_mobo->version = buffer[EUB_MOBO_REG_VERSION];

	ret = devm_mfd_add_devices(eub5_mobo->dev, -1, eub5_mobo_devs,
				   ARRAY_SIZE(eub5_mobo_devs), NULL, 0, NULL);
	if (ret < 0) {
		dev_err(eub5_mobo->dev, "mfd_add_devices failed: %d\n", ret);
		return ret;
	}
	return 0;
}

static const struct i2c_device_id eub5_mobo_i2c_id[] = {
	{ "eub5_mobo" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, eub5_mobo_i2c_id);

static const struct of_device_id eub5_mobo_of_match[] = {
	{ .compatible = "esrille,eub5_mobo", },
	{},
};
MODULE_DEVICE_TABLE(of, eub5_mobo_of_match);

static struct i2c_driver eub5_mobo_i2c_driver = {
	.driver = {
		.name = "eub5_mobo",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = eub5_mobo_of_match,
	},
	.probe = eub5_mobo_i2c_probe,
	.id_table = eub5_mobo_i2c_id,
};
module_i2c_driver(eub5_mobo_i2c_driver);

MODULE_DESCRIPTION("Esrille New Unbrick Motherboard Multi-Function Driver");
MODULE_AUTHOR("Esrille Inc. <info@esrille.com>");
MODULE_LICENSE("GPL");
MODULE_SOFTDEP("pre: i2c6");
