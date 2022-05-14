/*
 * Esrille New Unbrick Battery Driver
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#include <linux/mfd/eub5_mobo.h>

#define DRIVER_NAME		"eub5_battery"

#define BATTERY_LEVELS_SIZE	100	/* from 2.00 (200) to 2.99 (299) */
#define SCAN_DELAY		1000	/* in msec */
#define FACTOR			80
#define FACTOR_DIV		18

#define BATTERY_EMPTY		 8000000
#define BATTERY_FULL		12000000
#define BATTERY_MAX		14400000
#define BATTERY_NO_LOAD		  460000

#define ADC_REF		 	 5100000	/* units of µV */
#define CC_THRESH                 850000	/* units of µV */

#define IS_ONLINE(power)	(CC_THRESH <= (power)->cc1 || CC_THRESH <= (power)->cc2)

/* The main device structure */
struct eub5_power {
	struct eub5_mobo_dev	*mfd;
	struct device		*dev;

	int			voltage;		/* units of µV */
	int			rated_capacity;		/* units of µAh */
	int			rem_capacity;		/* of % */
	int			cc1;
	int			cc2;

	struct power_supply	*battery;
	struct power_supply	*ac;

	struct delayed_work	dwork;
};

/* Percentage from 2.0V to 2.99V */
static const uint8_t battery_levels[BATTERY_LEVELS_SIZE] = {
/*  .00  .01  .02  .03  .04  .05  .06  .07  .08  .09 */
      0,   0,   0,   0,   1,   1,   1,   1,   1,   2,	/* .0 */
      2,   2,   2,   2,   3,   3,   3,   3,   4,   5,	/* .1 */
      5,   5,   6,   7,   7,   8,   9,  10,  11,  12,	/* .2 */
     14,  15,  16,  18,  19,  21,  23,  26,  30,  35,	/* .3 */
     43,  49,  57,  69,  72,  74,  78,  79,  80,  82,	/* .4 */
     84,  86,  89,  92,  93,  95,  96,  97,  98,  98,	/* .5 */
     99,  99,  99, 100, 100, 100, 100, 100, 100, 100,	/* .6 */
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100,	/* .7 */
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100,	/* .8 */
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100,	/* .9 */
};

static u16 rated_capacity = 2500 * 8;	/* units of mAh */

static int __maybe_unused eub5_battery_reg_get(struct eub5_power *power, u8 reg, u8 *val, u8 flags)
{
	return power->mfd->read_dev(power->mfd, reg, 1, val, flags);
}

static void eub5_battery_init_status(struct eub5_power *power)
{
	/* Note initially pretend to be operated by AC */
	power->voltage = 0;
	power->rated_capacity = rated_capacity * 1000;
	power->rem_capacity = 0;
	power->cc1 = CC_THRESH;
	power->cc2 = 0;
}

static void eub5_battery_update_status(struct eub5_power *power)
{
	int ret;
	u8 regs[3];
	int voltage;
	int cc1;
	int cc2;
	int capacity;	/* of % */
	int level;

	ret = power->mfd->read_dev(power->mfd, EUB_MOBO_REG_VREF, 3, regs, EUB_MOBO_I2C_RETRY | EUB_MOBO_I2C_CRC);
	if (ret < 0)
		return;

	voltage = ADC_REF * regs[0] / 255; /* µV */
	voltage = (FACTOR * voltage) / FACTOR_DIV;
	cc1 = ADC_REF * regs[1] / 255;
	cc2 = ADC_REF * regs[2] / 255;
	if (CC_THRESH <= cc1 || CC_THRESH <= cc2) {
		voltage -= BATTERY_NO_LOAD;
		if (voltage < 0)
			voltage = 0;
	}
	level = voltage / 40000;
	if (level < 200)
		capacity = 0;
	else if (300 <= level)
		capacity = 100;
	else
		capacity = battery_levels[level - 200];
	if (0 < capacity || CC_THRESH <= cc1 || CC_THRESH <= cc2) {
		power->voltage = voltage;
		power->rem_capacity = capacity;
		power->cc1 = cc1;
		power->cc2 = cc2;
	} else {
		dev_info(power->dev, "UNEXPECTED VALUES\n");
	}
	dev_dbg(power->dev, "battery %u, capacity %d, cc1 %u, cc2 %u\n",
		voltage, capacity, cc1, cc2);
}

static void eub5_battery_work(struct work_struct *work)
{
	struct eub5_power *power = container_of(work, struct eub5_power, dwork.work);

	eub5_battery_update_status(power);
	mod_delayed_work(system_wq, &power->dwork, msecs_to_jiffies(SCAN_DELAY));
}

static int eub5_ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct eub5_power *power = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = (IS_ONLINE(power)) ? 1 : 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int eub5_battery_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct eub5_power *power = power_supply_get_drvdata(psy);
	int present = (BATTERY_EMPTY / 2 <= power->voltage) ? 1 : 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (IS_ONLINE(power))
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = present;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = BATTERY_MAX;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = BATTERY_EMPTY;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (present)
			val->intval = power->voltage;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = power->rated_capacity;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = (power->rated_capacity / 100) * power->rem_capacity;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = power->rem_capacity;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property eub5_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property eub5_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc battery_desc = {
	.name		= "battery",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.properties	= eub5_battery_props,
	.num_properties	= ARRAY_SIZE(eub5_battery_props),
	.get_property	= eub5_battery_get_property,
};

static const struct power_supply_desc ac_desc = {
	.name		= "ac",
	.type		= POWER_SUPPLY_TYPE_MAINS,
	.properties	= eub5_ac_props,
	.num_properties	= ARRAY_SIZE(eub5_ac_props),
	.get_property	= eub5_ac_get_property,
};

static int eub5_battery_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct eub5_mobo_dev *eub5_mobo_dev = dev_get_drvdata(dev->parent);
	struct eub5_power *power;
	struct power_supply_config ac_cfg = {};
	struct power_supply_config battery_cfg = {};

	if (dev->of_node)
		of_property_read_u16(pdev->dev.of_node, "esrille,rated_capacity", &rated_capacity);

	power = devm_kzalloc(dev, sizeof(struct eub5_power), GFP_KERNEL);
	if (!power)
		return -ENOMEM;

	power->mfd = eub5_mobo_dev;
	power->dev = dev;

	eub5_battery_init_status(power);
	eub5_battery_update_status(power);

	ac_cfg.drv_data = power;
	power->ac = power_supply_register(dev, &ac_desc, &ac_cfg);
	if (IS_ERR(power->ac)) {
		dev_info(dev, "failed to register ac\n");
		return PTR_ERR(power->ac);
	}

	battery_cfg.drv_data = power;
	power->battery = power_supply_register(dev, &battery_desc, &battery_cfg);
	if (IS_ERR(power->battery)) {
		dev_info(dev, "failed to register battery\n");
		power_supply_unregister(power->ac);
		return PTR_ERR(power->battery);
	}

	dev_info(dev, "registered eub5_battery driver [%u mAh]\n", rated_capacity);

	INIT_DELAYED_WORK(&power->dwork, eub5_battery_work);
	mod_delayed_work(system_wq, &power->dwork, msecs_to_jiffies(SCAN_DELAY));

	platform_set_drvdata(pdev, power);
	return 0;
}

static int eub5_battery_remove(struct platform_device *pdev)
{
	struct eub5_power *power = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&power->dwork);
	power_supply_unregister(power->battery);
	power_supply_unregister(power->ac);
	return 0;
}

static const struct of_device_id eub5_battery_of_match[] = {
	{ .compatible = "esrille,eub5_battery", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, eub5_battery_of_match);

static struct platform_driver eub5_battery_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(eub5_battery_of_match),
	},
	.probe = eub5_battery_probe,
	.remove = eub5_battery_remove,
};

module_platform_driver(eub5_battery_driver);

MODULE_DESCRIPTION("Esrille New Unbrick Battery Driver");
MODULE_AUTHOR("Esrille Inc. <info@esrille.com>");
MODULE_LICENSE("GPL");
