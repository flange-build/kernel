// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Polling-only driver for Goodix Touchscreens
 *
 *  Based on goodix.c, simplified for polling-only operation
 *  without IRQ support to avoid spurious I2C errors.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <asm/unaligned.h>

#define GOODIX_POLL_MAX_HEIGHT		4096
#define GOODIX_POLL_MAX_WIDTH		4096
#define GOODIX_POLL_CONTACT_SIZE	8
#define GOODIX_POLL_MAX_CONTACTS	10

#define GOODIX_POLL_CONFIG_MIN_LENGTH	186
#define GOODIX_POLL_CONFIG_MAX_LENGTH	240
#define GOODIX_POLL_CONFIG_911_LENGTH	186
#define GOODIX_POLL_CONFIG_GT9X_LENGTH	240

#define GOODIX_POLL_BUFFER_STATUS_READY	BIT(7)
#define GOODIX_POLL_HAVE_KEY		BIT(4)

#define GOODIX_POLL_REG_CONFIG_DATA	0x8047
#define GOODIX_POLL_REG_ID		0x8140
#define GOODIX_POLL_READ_COOR_ADDR	0x814E

#define GOODIX_POLL_ID_MAX_LEN		4
#define GOODIX_POLL_MAX_KEYS		7

#define RESOLUTION_LOC			1
#define MAX_CONTACTS_LOC		5
#define TRIGGER_LOC			6

/* Poll interval in milliseconds (17ms = ~60fps) */
#define POLL_INTERVAL_MS		17

struct goodix_poll_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct touchscreen_properties prop;
	struct regulator *avdd28;
	struct regulator *vddio;
	struct gpio_desc *gpiod_rst;
	char id[GOODIX_POLL_ID_MAX_LEN + 1];
	u16 version;
	unsigned int max_touch_num;
	unsigned int contact_size;
	unsigned short keymap[GOODIX_POLL_MAX_KEYS];
	u8 config[GOODIX_POLL_CONFIG_MAX_LENGTH];

	/* Polling mechanism */
	struct timer_list timer;
	struct work_struct work_poll;
	bool polling_enabled;
};

static int goodix_poll_i2c_read(struct i2c_client *client, u16 reg, u8 *buf, int len)
{
	struct i2c_msg msgs[2];
	__be16 wbuf = cpu_to_be16(reg);
	int ret;

	msgs[0].flags = 0;
	msgs[0].addr  = client->addr;
	msgs[0].len   = 2;
	msgs[0].buf   = (u8 *)&wbuf;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len;
	msgs[1].buf   = buf;

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret >= 0)
		ret = (ret == ARRAY_SIZE(msgs) ? 0 : -EIO);

	return ret;
}

static int goodix_poll_i2c_write_u8(struct i2c_client *client, u16 reg, u8 value)
{
	u8 buf[3];
	struct i2c_msg msg;
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;
	buf[2] = value;

	msg.flags = 0;
	msg.addr = client->addr;
	msg.buf = buf;
	msg.len = 3;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		ret = (ret == 1 ? 0 : -EIO);

	return ret;
}

static int goodix_poll_ts_read_input_report(struct goodix_poll_ts_data *ts, u8 *data)
{
	int touch_num;
	int error;
	u16 addr = GOODIX_POLL_READ_COOR_ADDR;
	const int header_contact_keycode_size = 1 + ts->contact_size + 1;

	error = goodix_poll_i2c_read(ts->client, addr, data, header_contact_keycode_size);
	if (error)
		return error;

	if (!(data[0] & GOODIX_POLL_BUFFER_STATUS_READY))
		return -EAGAIN;

	touch_num = data[0] & 0x0f;
	if (touch_num > ts->max_touch_num)
		return -EPROTO;

	if (touch_num > 1) {
		addr += header_contact_keycode_size;
		data += header_contact_keycode_size;
		error = goodix_poll_i2c_read(ts->client, addr, data,
					     ts->contact_size * (touch_num - 1));
		if (error)
			return error;
	}

	return touch_num;
}

static void goodix_poll_ts_report_touch_8b(struct goodix_poll_ts_data *ts, u8 *coor_data)
{
	int id = coor_data[0] & 0x0F;
	int input_x = get_unaligned_le16(&coor_data[1]);
	int input_y = get_unaligned_le16(&coor_data[3]);
	int input_w = get_unaligned_le16(&coor_data[5]);

	input_mt_slot(ts->input_dev, id);
	input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
	touchscreen_report_pos(ts->input_dev, &ts->prop, input_x, input_y, true);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, input_w);
}

static void goodix_poll_ts_release_keys(struct goodix_poll_ts_data *ts)
{
	int i;

	for (i = 0; i < GOODIX_POLL_MAX_KEYS; i++)
		input_report_key(ts->input_dev, ts->keymap[i], 0);
}

static void goodix_poll_ts_report_key(struct goodix_poll_ts_data *ts, u8 *data)
{
	int touch_num;
	u8 key_value;
	int i;

	if (data[0] & GOODIX_POLL_HAVE_KEY) {
		touch_num = data[0] & 0x0f;
		key_value = data[1 + ts->contact_size * touch_num];
		for (i = 0; i < GOODIX_POLL_MAX_KEYS; i++)
			if (key_value & BIT(i))
				input_report_key(ts->input_dev, ts->keymap[i], 1);
	} else {
		goodix_poll_ts_release_keys(ts);
	}
}

static void goodix_poll_process_events(struct goodix_poll_ts_data *ts)
{
	u8 point_data[2 + 9 * GOODIX_POLL_MAX_CONTACTS];
	int touch_num;
	int i;

	touch_num = goodix_poll_ts_read_input_report(ts, point_data);
	if (touch_num < 0)
		return;

	goodix_poll_ts_report_key(ts, point_data);

	for (i = 0; i < touch_num; i++)
		goodix_poll_ts_report_touch_8b(ts, &point_data[1 + ts->contact_size * i]);

	input_mt_sync_frame(ts->input_dev);
	input_sync(ts->input_dev);
}

static void goodix_poll_timer_handler(struct timer_list *t)
{
	struct goodix_poll_ts_data *ts = from_timer(ts, t, timer);

	if (ts->polling_enabled) {
		schedule_work(&ts->work_poll);
		mod_timer(&ts->timer, jiffies + msecs_to_jiffies(POLL_INTERVAL_MS));
	}
}

static void goodix_poll_work_handler(struct work_struct *work)
{
	struct goodix_poll_ts_data *ts = container_of(work, struct goodix_poll_ts_data, work_poll);

	goodix_poll_process_events(ts);
	/* Clear buffer status */
	goodix_poll_i2c_write_u8(ts->client, GOODIX_POLL_READ_COOR_ADDR, 0);
}

static void goodix_poll_enable(struct goodix_poll_ts_data *ts)
{
	if (!ts->polling_enabled) {
		ts->polling_enabled = true;
		ts->timer.expires = jiffies + msecs_to_jiffies(POLL_INTERVAL_MS);
		add_timer(&ts->timer);
	}
}

static void goodix_poll_disable(struct goodix_poll_ts_data *ts)
{
	if (ts->polling_enabled) {
		ts->polling_enabled = false;
		del_timer_sync(&ts->timer);
		cancel_work_sync(&ts->work_poll);
	}
}

static int goodix_poll_i2c_test(struct i2c_client *client)
{
	int retry = 0;
	int error;
	u8 test;

	while (retry++ < 5) {
		error = goodix_poll_i2c_read(client, GOODIX_POLL_REG_ID, &test, 1);
		if (!error)
			return 0;
		msleep(50);
	}

	return error;
}

static int goodix_poll_read_version(struct goodix_poll_ts_data *ts)
{
	int error;
	u8 buf[6];

	error = goodix_poll_i2c_read(ts->client, GOODIX_POLL_REG_ID, buf, sizeof(buf));
	if (error)
		return error;

	memcpy(ts->id, buf, GOODIX_POLL_ID_MAX_LEN);
	ts->id[GOODIX_POLL_ID_MAX_LEN] = 0;
	ts->version = get_unaligned_le16(&buf[4]);

	dev_info(&ts->client->dev, "ID %s, version: %04x\n", ts->id, ts->version);

	return 0;
}

static void goodix_poll_read_config(struct goodix_poll_ts_data *ts)
{
	int x_max, y_max;
	int error;

	error = goodix_poll_i2c_read(ts->client, GOODIX_POLL_REG_CONFIG_DATA,
				     ts->config, GOODIX_POLL_CONFIG_GT9X_LENGTH);
	if (error) {
		ts->max_touch_num = GOODIX_POLL_MAX_CONTACTS;
		return;
	}

	ts->max_touch_num = ts->config[MAX_CONTACTS_LOC] & 0x0f;
	if (ts->max_touch_num == 0 || ts->max_touch_num > GOODIX_POLL_MAX_CONTACTS)
		ts->max_touch_num = GOODIX_POLL_MAX_CONTACTS;

	x_max = get_unaligned_le16(&ts->config[RESOLUTION_LOC]);
	y_max = get_unaligned_le16(&ts->config[RESOLUTION_LOC + 2]);

	if (x_max && y_max) {
		input_abs_set_max(ts->input_dev, ABS_MT_POSITION_X, x_max - 1);
		input_abs_set_max(ts->input_dev, ABS_MT_POSITION_Y, y_max - 1);
	}
}

static int goodix_poll_reset(struct goodix_poll_ts_data *ts)
{
	if (!ts->gpiod_rst)
		return 0;

	/* Reset sequence */
	gpiod_direction_output(ts->gpiod_rst, 0);
	msleep(20);
	gpiod_direction_output(ts->gpiod_rst, 1);
	msleep(50);

	/* Put reset pin back to input mode */
	gpiod_direction_input(ts->gpiod_rst);

	return 0;
}

static void goodix_poll_disable_regulators(void *arg)
{
	struct goodix_poll_ts_data *ts = arg;

	if (ts->vddio)
		regulator_disable(ts->vddio);
	if (ts->avdd28)
		regulator_disable(ts->avdd28);
}

static int goodix_poll_ts_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct goodix_poll_ts_data *ts;
	struct device *dev = &client->dev;
	int error;
	int i;

	dev_info(dev, "Goodix Poll TS probe, I2C address: 0x%02x\n", client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "I2C check functionality failed\n");
		return -ENXIO;
	}

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	ts->contact_size = GOODIX_POLL_CONTACT_SIZE;
	i2c_set_clientdata(client, ts);

	/* Get regulators (optional) */
	ts->avdd28 = devm_regulator_get_optional(dev, "AVDD28");
	if (IS_ERR(ts->avdd28))
		ts->avdd28 = NULL;

	ts->vddio = devm_regulator_get_optional(dev, "VDDIO");
	if (IS_ERR(ts->vddio))
		ts->vddio = NULL;

	/* Get reset GPIO (optional) */
	ts->gpiod_rst = devm_gpiod_get_optional(dev, "reset", GPIOD_IN);
	if (IS_ERR(ts->gpiod_rst)) {
		error = PTR_ERR(ts->gpiod_rst);
		dev_warn(dev, "Failed to get reset GPIO: %d\n", error);
		ts->gpiod_rst = NULL;
	}

	/* Power up */
	if (ts->avdd28) {
		error = regulator_enable(ts->avdd28);
		if (error) {
			dev_err(dev, "Failed to enable AVDD28: %d\n", error);
			return error;
		}
	}

	if (ts->vddio) {
		error = regulator_enable(ts->vddio);
		if (error) {
			dev_err(dev, "Failed to enable VDDIO: %d\n", error);
			if (ts->avdd28)
				regulator_disable(ts->avdd28);
			return error;
		}
	}

	error = devm_add_action_or_reset(dev, goodix_poll_disable_regulators, ts);
	if (error)
		return error;

	/* Reset the controller */
	goodix_poll_reset(ts);

	/* Wait for controller to be ready */
	msleep(100);

	/* Test I2C communication */
	error = goodix_poll_i2c_test(client);
	if (error) {
		dev_err(dev, "I2C communication failure: %d\n", error);
		return error;
	}

	/* Read version */
	error = goodix_poll_read_version(ts);
	if (error) {
		dev_err(dev, "Failed to read version: %d\n", error);
		return error;
	}

	/* Allocate input device */
	ts->input_dev = devm_input_allocate_device(dev);
	if (!ts->input_dev) {
		dev_err(dev, "Failed to allocate input device\n");
		return -ENOMEM;
	}

	ts->input_dev->name = "Goodix Poll Capacitive TouchScreen";
	ts->input_dev->phys = "input/ts";
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0x0416;
	if (kstrtou16(ts->id, 10, &ts->input_dev->id.product))
		ts->input_dev->id.product = 0x1001;
	ts->input_dev->id.version = ts->version;

	/* Setup key mappings */
	ts->input_dev->keycode = ts->keymap;
	ts->input_dev->keycodesize = sizeof(ts->keymap[0]);
	ts->input_dev->keycodemax = GOODIX_POLL_MAX_KEYS;

	for (i = 0; i < GOODIX_POLL_MAX_KEYS; i++) {
		if (i == 0)
			ts->keymap[i] = KEY_LEFTMETA;
		else
			ts->keymap[i] = KEY_F1 + (i - 1);
		input_set_capability(ts->input_dev, EV_KEY, ts->keymap[i]);
	}

	/* Setup touch capabilities */
	input_set_capability(ts->input_dev, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(ts->input_dev, EV_ABS, ABS_MT_POSITION_Y);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	/* Set default resolution, will be updated by goodix_poll_read_config */
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, GOODIX_POLL_MAX_WIDTH - 1, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, GOODIX_POLL_MAX_HEIGHT - 1, 0, 0);

	/* Read configuration from device */
	goodix_poll_read_config(ts);

	/* Try to get touchscreen properties from device tree */
	touchscreen_parse_properties(ts->input_dev, true, &ts->prop);

	/* Initialize MT slots */
	error = input_mt_init_slots(ts->input_dev, ts->max_touch_num,
				    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (error) {
		dev_err(dev, "Failed to initialize MT slots: %d\n", error);
		return error;
	}

	/* Register input device */
	error = input_register_device(ts->input_dev);
	if (error) {
		dev_err(dev, "Failed to register input device: %d\n", error);
		return error;
	}

	/* Setup polling */
	INIT_WORK(&ts->work_poll, goodix_poll_work_handler);
	timer_setup(&ts->timer, goodix_poll_timer_handler, 0);

	/* Enable polling */
	goodix_poll_enable(ts);

	dev_info(dev, "Goodix Poll TS initialized successfully\n");

	return 0;
}

static void goodix_poll_ts_remove(struct i2c_client *client)
{
	struct goodix_poll_ts_data *ts = i2c_get_clientdata(client);

	goodix_poll_disable(ts);
}

static int goodix_poll_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct goodix_poll_ts_data *ts = i2c_get_clientdata(client);

	goodix_poll_disable(ts);

	return 0;
}

static int goodix_poll_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct goodix_poll_ts_data *ts = i2c_get_clientdata(client);

	goodix_poll_reset(ts);
	msleep(50);
	goodix_poll_enable(ts);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(goodix_poll_pm_ops, goodix_poll_suspend, goodix_poll_resume);

static const struct i2c_device_id goodix_poll_ts_id[] = {
	{ "goodix-poll", 0 },
	{ "gt9271-poll", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, goodix_poll_ts_id);

#ifdef CONFIG_OF
static const struct of_device_id goodix_poll_of_match[] = {
	{ .compatible = "goodix,gt9271-poll" },
	{ .compatible = "goodix,gt911-poll" },
	{ .compatible = "goodix,gt927-poll" },
	{ .compatible = "goodix,gt928-poll" },
	{ .compatible = "goodix,gt9xx-poll" },
	{ }
};
MODULE_DEVICE_TABLE(of, goodix_poll_of_match);
#endif

static struct i2c_driver goodix_poll_ts_driver = {
	.probe = goodix_poll_ts_probe,
	.remove = goodix_poll_ts_remove,
	.id_table = goodix_poll_ts_id,
	.driver = {
		.name = "Goodix-Poll-TS",
		.of_match_table = of_match_ptr(goodix_poll_of_match),
		.pm = pm_sleep_ptr(&goodix_poll_pm_ops),
	},
};
module_i2c_driver(goodix_poll_ts_driver);

MODULE_AUTHOR("Eric");
MODULE_DESCRIPTION("Goodix touchscreen polling-only driver");
MODULE_LICENSE("GPL v2");
