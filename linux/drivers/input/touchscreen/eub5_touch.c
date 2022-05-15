/*
 * Esrille New Unbrick Touch Screen Driver
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
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/mfd/eub5_mobo.h>

#define DRIVER_NAME	"eub5_touch"

#define TFT_070

#ifdef TFT_070

#define MIN_X		150
#define MAX_X		3870
#define PLAY_X		50
#define MIN_Y		300
#define MAX_Y		3420
#define PLAY_Y		30

#define RES_X		24	/* [units/mm] (MAX_X - MIN_X) / 154.21 */
#define RES_Y		36	/* [units/mm] (MAX_Y - MIN_Y) / 85.92 */

#define THRESH_Z	0xf90
#define THRESH_NOISE	10000

#endif

#ifdef TFT_101

#define MIN_X		220
#define MAX_X		3760
#define PLAY_X		50
#define MIN_Y		340
#define MAX_Y		3120
#define PLAY_Y		30

#define RES_X		15	/* [units/mm] */
#define RES_Y		22	/* [units/mm] */

#define THRESH_Z	0xf50
#define THRESH_NOISE	10000

#endif

#define GPIO_PIC_INT	24

static u16 min_x = MIN_X;
static u16 max_x = MAX_X;
static u16 min_y = MIN_Y;
static u16 max_y = MAX_Y;
static s32 res_x = RES_X;
static s32 res_y = RES_Y;
static u16 thresh = THRESH_Z;
static u16 pad_buttons[3] = {
	KEY_LEFTSHIFT,
	KEY_LEFTMETA,
	KEY_POWER
};

/* The main device structure */
struct eub5_touch {
	struct eub5_mobo_dev	*mfd;
	struct device		*dev;
	struct input_dev	*pen_dev;
	struct input_dev	*pad_dev;
	int			irq;
	int			x;		// previous reported x position
	int			y;		// previous reported y position
	u8			buttons;	// previous button state
	int			xr[3];		// previous x positions
	int			yr[3];		// previous y positions
	int			n;
};

static int __maybe_unused eub5_touch_reg_get(struct eub5_touch *touch, u8 reg, u8 *val, u8 flags)
{
	return touch->mfd->read_dev(touch->mfd, reg, 1, val, flags);
}

static int __maybe_unused eub5_touch_reg_set(struct eub5_touch *touch, u8 reg, u8 val, u8 flags)
{
	return touch->mfd->write_dev(touch->mfd, reg, 1, &val, flags);
}

static int outlying(const int* x, const int* y)
{
	int d[3];
	int dx, dy;

	dx = x[0] - x[1];
	dy = y[0] - y[1];
	d[0] = dx * dx + dy * dy;
	dx = x[1] - x[2];
	dy = y[1] - y[2];
	d[1] = dx * dx + dy * dy;
	dx = x[2] - x[0];
	dy = y[2] - y[0];
	d[2] = dx * dx + dy * dy;
	/* if d[0] is the smallest, point (x[2], y[2]) is outlying */
	return (d[0] < d[1] && d[0] < d[2]) ? d[1] : 0;
}

static void eub5_touch_get_input(struct eub5_touch *touch)
{
	struct input_dev *pen_dev = touch->pen_dev;
	struct input_dev *pad_dev = touch->pad_dev;
	u8 val[7];
	s32 x, y, z;
	u8 buttons;
	int ret;
	int drop;

	ret = touch->mfd->read_dev(touch->mfd, EUB_MOBO_REG_BUTTONS, sizeof(val), val,
				   EUB_MOBO_I2C_RETRY | EUB_MOBO_I2C_CRC);
	if (ret < 0)
		return;

	buttons = val[0];
	x = val[1] | (val[2] << 8);
	y = val[3] | (val[4] << 8);
	z = val[5] | (val[6] << 8);
	dev_dbg(touch->dev, "touch: (%d, %d, %03x) %02x\n", x, y, z, buttons);

	input_report_key(pad_dev, pad_buttons[0], buttons & 1);
	input_report_key(pad_dev, pad_buttons[1], buttons & 2);
	input_report_key(pad_dev, pad_buttons[2], buttons & 4);
	input_sync(pad_dev);
	buttons &= 7;

	if (x < (min_x - PLAY_X) || (max_x + PLAY_X) < x || y < (min_y - PLAY_Y) || (max_y + PLAY_Y) < y)
		z = 0xfff;

	/* Report the pen event */
	if (z <= thresh) {
		/* drop an outlying point */
		touch->xr[touch->n] = x;
		touch->yr[touch->n] = y;
		if (++touch->n < 3)
			drop = 1;
		else {
			drop = outlying(touch->xr, touch->yr);
			touch->n = 2;
			if (drop < THRESH_NOISE) {
				touch->xr[0] = touch->xr[1];
				touch->yr[0] = touch->yr[1];
				touch->xr[1] = touch->xr[2];
				touch->yr[1] = touch->yr[2];
			}
		}
		dev_dbg(touch->dev, "touch: (%d, %d, %03x) %02x, n: %d, drop: %d\n", x, y, z, buttons, touch->n, drop);
		if (!drop) {
			/* Report the positions */
			touch->x = x;
			touch->y = y;
			// low pass filter
			if (touch->x == -1 || touch->y == -1) {
				touch->x = x;
				touch->y = y;
			} else {
				touch->x += (x >> 2) - (touch->x >> 2);
				touch->y += (y >> 2) - (touch->y >> 2);
			}
			input_report_key(pen_dev, BTN_TOUCH, 1);
			input_report_key(pen_dev, BTN_TOOL_PEN, 1);
			input_report_abs(pen_dev, ABS_X, touch->x);
			input_report_abs(pen_dev, ABS_Y, touch->y);
		}
	} else {
		/* If z is at Vdd, the X-plate and Y-plate are not touching. */
		touch->n = 0;
		input_report_key(pen_dev, BTN_TOUCH, 0);
		input_report_key(pen_dev, BTN_TOOL_PEN, buttons);
	}
	if (buttons != touch->buttons) {
		touch->buttons = buttons;
	}
	input_sync(pen_dev);

#ifdef TEST_JAM
	if (buttons & 1) {
		ret = eub5_touch_reg_set(touch, EUB_MOBO_REG_BUTTONS, 1, EUB_MOBO_I2C_CRC | EUB_MOBO_I2C_CRC_JAM);
		dev_info(touch->dev, "jam: %d\n", ret);
	}
#endif
}

static void eub5_touch_set_pen_params(struct eub5_touch *touch)
{
	struct input_dev *pen_dev = touch->pen_dev;

	pen_dev->name = "Esrille New Unbrick Touch Screen";
	pen_dev->id.bustype = BUS_I2C;
	pen_dev->id.version = touch->mfd->version;
	pen_dev->dev.parent = touch->dev;

	input_set_drvdata(pen_dev, touch);

	__set_bit(INPUT_PROP_DIRECT, pen_dev->propbit);
	__set_bit(INPUT_PROP_POINTER, pen_dev->propbit);
	input_set_abs_params(pen_dev, ABS_X, min_x, max_x, 8, 0);
	input_set_abs_params(pen_dev, ABS_Y, min_y, max_y, 10, 0);
	input_abs_set_res(pen_dev, ABS_X, res_x);
	input_abs_set_res(pen_dev, ABS_Y, res_y);
	input_set_capability(pen_dev, EV_KEY, BTN_TOUCH);
	input_set_capability(pen_dev, EV_KEY, BTN_TOOL_PEN);
}

static void eub5_touch_set_pad_params(struct eub5_touch *touch)
{
	struct input_dev *pad_dev = touch->pad_dev;

	pad_dev->name = "Esrille New Unbrick Buttons";
	pad_dev->id.bustype = BUS_I2C;
	pad_dev->id.version = touch->mfd->version;
	pad_dev->dev.parent = touch->dev;
	input_set_drvdata(pad_dev, touch);

	/* Register device's keys */
	input_set_capability(pad_dev, EV_KEY, pad_buttons[0]);
	input_set_capability(pad_dev, EV_KEY, pad_buttons[1]);
	input_set_capability(pad_dev, EV_KEY, pad_buttons[2]);
}

static void eub5_touch_configure(struct eub5_touch *touch)
{
	touch->x = -1;
	touch->y = -1;
	touch->buttons = 0;
	touch->n = 0;
}

static irqreturn_t eub5_touch_isr(int irq, void *id)
{
	struct eub5_touch *touch = id;

	eub5_touch_get_input(touch);
	return IRQ_HANDLED;
}

static int eub5_touch_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct eub5_mobo_dev *eub5_mobo_dev = dev_get_drvdata(dev->parent);
	struct eub5_touch *touch;
	int ret;

	if (dev->of_node) {
		of_property_read_u16(pdev->dev.of_node, "esrille,touch_min_x", &min_x);
		of_property_read_u16(pdev->dev.of_node, "esrille,touch_max_x", &max_x);
		of_property_read_u16(pdev->dev.of_node, "esrille,touch_min_y", &min_y);
		of_property_read_u16(pdev->dev.of_node, "esrille,touch_max_y", &max_y);
		of_property_read_s32(pdev->dev.of_node, "esrille,touch_res_x", &res_x);
		of_property_read_s32(pdev->dev.of_node, "esrille,touch_res_y", &res_y);
		of_property_read_u16(pdev->dev.of_node, "esrille,touch_thresh", &thresh);
		of_property_read_u16_array(pdev->dev.of_node, "esrille,buttons", pad_buttons, 3);
	}

	dev_info(&pdev->dev,
		"min_x: %u, max_x: %u, min_y: %u, max_y: %u\n"
		"res_x: %d, res_y: %d, thresh: %u\n"
		"buttons: %u, %u, %u\n",
		min_x, max_x, min_y, max_y, res_x, res_y, thresh,
		pad_buttons[0], pad_buttons[1], pad_buttons[2]);

	touch = devm_kzalloc(dev, sizeof(struct eub5_touch), GFP_KERNEL);
	if (!touch)
		return -ENOMEM;

	touch->pen_dev = devm_input_allocate_device(dev);
	touch->pad_dev = devm_input_allocate_device(dev);
	touch->mfd = eub5_mobo_dev;
	touch->dev = &pdev->dev;
	if (!touch->pen_dev || !touch->pad_dev) {
		return -ENOMEM;
	}

	eub5_touch_set_pen_params(touch);
	eub5_touch_set_pad_params(touch);
	eub5_touch_configure(touch);

	/* Register the pen device in input subsystem */
	ret = input_register_device(touch->pen_dev);
	if (ret) {
		dev_err(&pdev->dev,
			"Input device register failed: %d\n", ret);
		return ret;
	}
	/* Register the pad device in input subsystem */
	ret = input_register_device(touch->pad_dev);
	if (ret) {
		dev_err(&pdev->dev,
			"Input device register failed: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, touch);

	gpio_direction_input(GPIO_PIC_INT);
	touch->irq = gpio_to_irq(GPIO_PIC_INT);
	ret = request_threaded_irq(touch->irq, NULL, eub5_touch_isr,
				   IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
				   "eub5_touch_isr", (void *) touch);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"request_irq failed: %d\n", ret);
		return ret;
	}

	eub5_touch_get_input(touch);

	return 0;
}

static int eub5_touch_remove(struct platform_device *pdev)
{
	struct eub5_touch *touch = platform_get_drvdata(pdev);

	free_irq(touch->irq, touch);
	return 0;
}

static const struct of_device_id eub5_touch_of_match[] = {
	{ .compatible = "esrille,eub5_touch", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, eub5_touch_of_match);

static struct platform_driver eub5_touch_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(eub5_touch_of_match),
	},
	.probe = eub5_touch_probe,
	.remove = eub5_touch_remove
};

module_platform_driver(eub5_touch_driver);

MODULE_DESCRIPTION("Esrille New Unbrick Touch Screen Driver");
MODULE_AUTHOR("Esrille Inc. <info@esrille.com>");
MODULE_LICENSE("GPL");
