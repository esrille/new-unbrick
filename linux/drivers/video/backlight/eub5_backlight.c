/*
 * Esrille New Unbrick Backlight Driver
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

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/mfd/eub5_mobo.h>

#define DEFAULT_BRIGHTNESS	46	// in % about 150 cd/m²

struct eub5_backlight {
	struct eub5_mobo_dev	*mfd;
	struct device		*dev;
};

static inline int eub5_backlight_read(struct eub5_backlight *gbl, u8 reg, u8 *val, u8 flags)
{
	return gbl->mfd->read_dev(gbl->mfd, reg, 1, val, flags);
}

static inline int eub5_backlight_write(struct eub5_backlight *gbl, u8 reg, u8 val, u8 flags)
{
	return gbl->mfd->write_dev(gbl->mfd, reg, 1, &val, flags);
}

static int eub5_backlight_update_status(struct backlight_device *bl)
{
	struct eub5_backlight *gbl = bl_get_data(bl);
	int brightness = bl->props.brightness;
	int ret;

	if (bl->props.power != FB_BLANK_UNBLANK ||
	    bl->props.fb_blank != FB_BLANK_UNBLANK ||
	    (bl->props.state & BL_CORE_FBBLANK))
		brightness = 0;

	ret = eub5_backlight_write(gbl, EUB_MOBO_REG_BRIGHTNESS, brightness,
				   EUB_MOBO_I2C_RETRY | EUB_MOBO_I2C_CRC);
	if (ret < 0)
		dev_err(gbl->dev, "failed to set brightness\n");
	return ret;
}

static const struct backlight_ops eub5_backlight_ops = {
	.update_status	= eub5_backlight_update_status,
};

static int eub5_backlight_probe(struct platform_device *pdev)
{
	struct eub5_mobo_dev *eub5_mobo_dev = dev_get_drvdata(pdev->dev.parent);
	struct eub5_backlight *gbl;
	struct backlight_properties props;
	struct backlight_device *bl;
	int ret;
	u8 val;

	gbl = devm_kzalloc(&pdev->dev, sizeof(*gbl), GFP_KERNEL);
	if (!gbl)
		return -ENOMEM;

	gbl->mfd = eub5_mobo_dev;
	gbl->dev = eub5_mobo_dev->dev;

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = 100;
	bl = devm_backlight_device_register(&pdev->dev, dev_name(&pdev->dev),
					    &pdev->dev, gbl, &eub5_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		return PTR_ERR(bl);
	}

	ret = eub5_backlight_read(gbl, EUB_MOBO_REG_DISPLAY, &val,
				  EUB_MOBO_I2C_RETRY | EUB_MOBO_I2C_CRC);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to read LCD power\n");
		return ret;
	}
	if (val == 0) {
		ret = eub5_backlight_write(gbl, EUB_MOBO_REG_DISPLAY, 1,
					   EUB_MOBO_I2C_RETRY | EUB_MOBO_I2C_CRC);
		if (ret < 0) {
			dev_err(gbl->dev, "failed to set LCD power\n");
			return ret;
		}
                /* wait for eight frame */
                msleep(160);
	}
	ret = eub5_backlight_read(gbl, EUB_MOBO_REG_BRIGHTNESS, &val,
				  EUB_MOBO_I2C_RETRY | EUB_MOBO_I2C_CRC);
        if (ret < 0) {
            val = DEFAULT_BRIGHTNESS;
	}
        eub5_backlight_write(gbl, EUB_MOBO_REG_BRIGHTNESS, val,
			     EUB_MOBO_I2C_RETRY | EUB_MOBO_I2C_CRC);
        bl->props.brightness = (u8) val;

	platform_set_drvdata(pdev, gbl);
	return 0;
}

static void eub5_backlight_shutdown(struct platform_device *pdev)
{
	struct eub5_backlight *gbl = platform_get_drvdata(pdev);

        /* Raspberry Pi 4B turns off 3.3V during reboot:
         *   https://github.com/raspberrypi/linux/issues/3065
         * Therefore, the LCD panel must also be turned off during the reboot
         * process.
         */
	eub5_backlight_write(gbl, EUB_MOBO_REG_DISPLAY, 0,
			     EUB_MOBO_I2C_RETRY | EUB_MOBO_I2C_CRC);
        /* wait for one frame */
        msleep(20);
        /* then, turn off the backlight */
        eub5_backlight_write(gbl, EUB_MOBO_REG_BRIGHTNESS, 0,
			     EUB_MOBO_I2C_RETRY | EUB_MOBO_I2C_CRC);
}

static const struct of_device_id eub5_backlight_of_match[] = {
	{ .compatible = "esrille,eub5_backlight", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, eub5_backlight_of_match);

static struct platform_driver eub5_backlight_driver = {
	.driver = {
		.name = "eub5_backlight",
		.of_match_table = of_match_ptr(eub5_backlight_of_match),
	},
	.probe = eub5_backlight_probe,
	.shutdown = eub5_backlight_shutdown,
};

module_platform_driver(eub5_backlight_driver);

MODULE_DESCRIPTION("Esrille New Unbrick Backlight Driver");
MODULE_AUTHOR("Esrille Inc. <info@esrille.com>");
MODULE_LICENSE("GPL");
