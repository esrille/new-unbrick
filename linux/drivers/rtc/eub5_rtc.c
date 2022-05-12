/*
 * Esrille New Unbrick RTC Driver
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
 *
 * Credits
 *
 * rtc-ds1307.c - RTC driver for some mostly-compatible I2C chips.
 *
 *  Copyright (C) 2005 James Chapman (ds1337 core)
 *  Copyright (C) 2006 David Brownell
 *  Copyright (C) 2009 Matthias Fuchs (rx8025 support)
 *  Copyright (C) 2012 Bertrand Achard (nvram access fixes)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/acpi.h>
#include <linux/bcd.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/clk-provider.h>
#include <linux/regmap.h>

#include <linux/version.h>

#include <linux/mfd/eub5_mobo.h>

#define REG_OFFSET	EUB_MOBO_REG_SECONDS

#define REG_SECONDS	(EUB_MOBO_REG_SECONDS - REG_OFFSET)
#define REG_MINUTES	(EUB_MOBO_REG_MINUTES - REG_OFFSET)
#define REG_HOURS	(EUB_MOBO_REG_HOURS - REG_OFFSET)
#define REG_WEEKDAY	(EUB_MOBO_REG_WEEKDAY - REG_OFFSET)
#define REG_DAY		(EUB_MOBO_REG_DAY - REG_OFFSET)
#define REG_MONTH	(EUB_MOBO_REG_MONTH - REG_OFFSET)
#define REG_YEAR	(EUB_MOBO_REG_YEAR - REG_OFFSET)
#define REG_RTC_CONTROL	(EUB_MOBO_REG_RTC_CONTROL - REG_OFFSET)
#define BIT_RTCEN	0x80

struct eub5_rtcc {
	struct eub5_mobo_dev	*mfd;
	struct device		*dev;
	struct rtc_device	*rtc;
};

static int eub5_rtcc_get_time(struct device *dev, struct rtc_time *t);
static int eub5_rtcc_set_time(struct device *dev, struct rtc_time *t);

static int eub5_rtcc_get_time(struct device *dev, struct rtc_time *t)
{
	struct eub5_rtcc	*rtcc = dev_get_drvdata(dev);
	int			ret;
	u8			regs[7];

	/* read the RTC date and time registers all at once */
	ret = rtcc->mfd->read_dev(rtcc->mfd, REG_OFFSET, sizeof(regs), regs,
				  EUB_MOBO_I2C_RETRY | EUB_MOBO_I2C_CRC);
	if (ret < 0) {
		dev_err(dev, "%s error %d\n", "read", ret);
		return ret;
	}

	dev_dbg(dev, "%s: %7ph\n", "read", regs);

	t->tm_sec = bcd2bin(regs[REG_SECONDS] & 0x7f);
	t->tm_min = bcd2bin(regs[REG_MINUTES] & 0x7f);
	t->tm_hour = bcd2bin(regs[REG_HOURS] & 0x3f);
	t->tm_wday = bcd2bin(regs[REG_WEEKDAY] & 0x07);
	t->tm_mday = bcd2bin(regs[REG_DAY] & 0x3f);
	t->tm_mon = bcd2bin(regs[REG_MONTH] & 0x1f) - 1;
	t->tm_year = bcd2bin(regs[REG_YEAR]) + 100;

	dev_dbg(dev, "%s %4u/%02u/%02u (%u) %02u:%02u:%02u\n",
		"read",
		1900 + t->tm_year, 1 + t->tm_mon, t->tm_mday, t->tm_wday,
		t->tm_hour, t->tm_min, t->tm_sec);

	return 0;
}

static int eub5_rtcc_set_time(struct device *dev, struct rtc_time *t)
{
	struct eub5_rtcc	*rtcc = dev_get_drvdata(dev);
	int			ret;
	int			tmp;
	u8			regs[7];

	dev_dbg(dev, "%s %4u/%02u/%02u (%u) %02u:%02u:%02u\n",
		"write",
		1900 + t->tm_year, 1 + t->tm_mon, t->tm_mday, t->tm_wday,
		t->tm_hour, t->tm_min, t->tm_sec);

	if (t->tm_year < 100 || 199 < t->tm_year)
		return -EINVAL;

	regs[REG_SECONDS] = bin2bcd(t->tm_sec);
	regs[REG_MINUTES] = bin2bcd(t->tm_min);
	regs[REG_HOURS] = bin2bcd(t->tm_hour);
	regs[REG_WEEKDAY] = bin2bcd(t->tm_wday);
	regs[REG_DAY] = bin2bcd(t->tm_mday);
	regs[REG_MONTH] = bin2bcd(t->tm_mon + 1);

	/* assume 20YY not 19YY */
	tmp = t->tm_year - 100;
	regs[REG_YEAR] = bin2bcd(tmp);

	ret = rtcc->mfd->write_dev(rtcc->mfd, REG_OFFSET, sizeof(regs), regs,
				   EUB_MOBO_I2C_RETRY | EUB_MOBO_I2C_CRC);
	if (ret < 0) {
		dev_err(dev, "%s error %d\n", "write", ret);
	}
	return ret;
}

static const struct rtc_class_ops rtc_ops = {
	.read_time	= eub5_rtcc_get_time,
	.set_time	= eub5_rtcc_set_time,
};

static int eub5_rtcc_probe(struct platform_device *pdev)
{
	struct eub5_mobo_dev	*eub5_mobo_dev = dev_get_drvdata(pdev->dev.parent);
	struct eub5_rtcc	*rtcc;
	int			err = -ENODEV;

	rtcc = devm_kzalloc(&pdev->dev, sizeof(struct eub5_rtcc), GFP_KERNEL);
	if (!rtcc)
		return -ENOMEM;
	platform_set_drvdata(pdev, rtcc);
	rtcc->mfd = eub5_mobo_dev;
	rtcc->dev = &pdev->dev;
	rtcc->rtc = devm_rtc_allocate_device(rtcc->dev);
	if (IS_ERR(rtcc->rtc))
		return PTR_ERR(rtcc->rtc);
	rtcc->rtc->ops = &rtc_ops;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0)
	err = devm_rtc_register_device(rtcc->rtc);
#else
	err = rtc_register_device(rtcc->rtc);
#endif
	if (0 <= err) {
		struct rtc_time	tm;

		err = eub5_rtcc_get_time(&pdev->dev, &tm);
		if (0 <= err) {
			dev_info(&pdev->dev, "%s %4u/%02u/%02u (%u) %02u:%02u:%02u\n",
				"read",
				1900 + tm.tm_year, 1 + tm.tm_mon, tm.tm_mday, tm.tm_wday,
				tm.tm_hour, tm.tm_min, tm.tm_sec);
		}
	}
	return err;
}

static const struct of_device_id eub5_rtcc_of_match[] = {
	{ .compatible = "esrille,eub5_rtc", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, eub5_rtcc_of_match);

static struct platform_driver eub5_rtcc_driver = {
	.driver = {
		.name	= "eub5_rtc",
		.of_match_table = of_match_ptr(eub5_rtcc_of_match),
	},
	.probe		= eub5_rtcc_probe,
};

module_platform_driver(eub5_rtcc_driver);

MODULE_DESCRIPTION("Esrille New Unbrick RTC driver");
MODULE_AUTHOR("Esrille Inc. <info@esrille.com>");
MODULE_LICENSE("GPL");
