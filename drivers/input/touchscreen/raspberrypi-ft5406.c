/*
 * Driver for ft5406 touchscreen controller accessible via
 * Raspberry Pi firmware
 *
 * Copyright (C) 2015 Raspberry Pi
 * Copyright (C) 2016 Lubomir Rintel <lkundrak@v3.sk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/input/mt.h>
#include <linux/input-polldev.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <soc/bcm2835/raspberrypi-firmware.h>

#define DEFAULT_POLL_PERIOD 17 /* 60 fps */

#define MAXIMUM_SUPPORTED_POINTS 10
struct ft5406_regs {
	uint8_t device_mode;
	uint8_t gesture_id;
	uint8_t num_points;
	struct {
		uint8_t xh;
		uint8_t xl;
		uint8_t yh;
		uint8_t yl;
		uint8_t res1;
		uint8_t res2;
	} point[MAXIMUM_SUPPORTED_POINTS];
} __packed;

#define SCREEN_WIDTH  800
#define SCREEN_HEIGHT 480

struct rpi_ft5406 {
	struct device           *dev;
	struct input_polled_dev *poll_dev;
	void __iomem            *base;
	int known_ids;
};

/* This function polls the memory based register copy of the ft5406
 * registers using the number of points register to know whether
 * the copy has been updated (we write 99 to the memory copy, the
 * GPU will write between 0 - 10 points)
 */
static void rpi_ft5406_poll(struct input_polled_dev *poll_dev)
{
	struct rpi_ft5406 *ts = poll_dev->private;
	struct input_dev *input = poll_dev->input;
	struct ft5406_regs regs;
	int modified_ids = 0, released_ids;
	int i;

	memcpy_fromio(&regs, ts->base, sizeof(regs));
	writel(99, ts->base + offsetof(struct ft5406_regs, num_points));

	/* Check whether firmware updated the buffer since last check. */
	if (regs.num_points == 99)
		return;

	/* Do nothing if there's no touch nor release. */
	if (regs.num_points == 0 && ts->known_ids == 0)
		return;

	for (i = 0; i < regs.num_points; i++) {
		int x, y, touchid;

		x = (((int) regs.point[i].xh & 0xf) << 8) + regs.point[i].xl;
		y = (((int) regs.point[i].yh & 0xf) << 8) + regs.point[i].yl;
		touchid = (regs.point[i].yh >> 4) & 0xf;

		modified_ids |= 1 << touchid;

		if (!((1 << touchid) & ts->known_ids)) {
			dev_dbg(ts->dev, "x = %d, y = %d, touchid = %d\n",
							x, y, touchid);
		}

		input_mt_slot(input, touchid);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, 1);

		input_report_abs(input, ABS_MT_POSITION_X, x);
		input_report_abs(input, ABS_MT_POSITION_Y, y);

	}

	released_ids = ts->known_ids & ~modified_ids;
	for (i = 0; released_ids && i < MAXIMUM_SUPPORTED_POINTS; i++) {
		if (released_ids & (1<<i)) {
			dev_dbg(ts->dev, "Released %d, known = %x modified = %x\n",
						i, ts->known_ids, modified_ids);
			input_mt_slot(input, i);
			input_mt_report_slot_state(input, MT_TOOL_FINGER, 0);
			modified_ids &= ~(1 << i);
		}
	}
	ts->known_ids = modified_ids;

	input_mt_report_pointer_emulation(input, true);
	input_sync(input);
}

static int rpi_ft5406_probe(struct platform_device *pdev)
{
	int ret;
	struct input_dev *input;
	struct rpi_ft5406 *ts;
	struct device *dev = &pdev->dev;
	struct resource *res;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ts->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ts->base))
		return PTR_ERR(ts->base);

	ts->poll_dev = devm_input_allocate_polled_device(dev);
	if (!ts->poll_dev)
		return -ENOMEM;

	ts->dev = dev;
	ts->poll_dev->private = ts;
	ts->poll_dev->poll = rpi_ft5406_poll;
	ts->poll_dev->poll_interval = DEFAULT_POLL_PERIOD;

	platform_set_drvdata(pdev, ts);

	input = ts->poll_dev->input;
	input->name = "Raspberry Pi Touchscreen";

	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_SYN, input->evbit);
	__set_bit(EV_ABS, input->evbit);

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, SCREEN_WIDTH, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, SCREEN_HEIGHT, 0, 0);

	input_mt_init_slots(input, MAXIMUM_SUPPORTED_POINTS, INPUT_MT_DIRECT);

	ret = input_register_polled_device(ts->poll_dev);
	if (ret)
		return ret;

	return 0;
}

static int rpi_ft5406_remove(struct platform_device *pdev)
{
	struct rpi_ft5406 *ts = platform_get_drvdata(pdev);

	input_unregister_polled_device(ts->poll_dev);

	return 0;
}

static const struct of_device_id rpi_ft5406_match[] = {
	{ .compatible = "raspberrypi,raspberrypi-ft5406", },
	{},
};
MODULE_DEVICE_TABLE(of, rpi_ft5406_match);

static struct platform_driver rpi_ft5406_driver = {
	.driver = {
		.name   = "raspberrypi-ft5406",
		.owner  = THIS_MODULE,
		.of_match_table = rpi_ft5406_match,
	},
	.probe          = rpi_ft5406_probe,
	.remove         = rpi_ft5406_remove,
};

module_platform_driver(rpi_ft5406_driver);

MODULE_AUTHOR("Gordon Hollingworth");
MODULE_AUTHOR("Lubomir Rintel <lkundrak@v3.sk>");
MODULE_DESCRIPTION("Raspberry Pi FT5406 Touchscreen driver");
MODULE_LICENSE("GPL v2");
