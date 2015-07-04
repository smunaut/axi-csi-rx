/*
 * ov5647.c
 *
 * Copyright (C) 2015 Sylvain Munaut <tnt@246tNt.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>


#define DRIVER_NAME	"ov5647"

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-2)");


struct ov5647 {
	struct v4l2_subdev sd;		/* Must be first ! */
	struct media_pad pad;
	struct i2c_client *client;
	struct gpio_desc *gpio_ena;
	u32 xclk_freq;
};

/* Data ------------------------------------------------------------------- */

const static struct {
	enum v4l2_mbus_pixelcode code;
	int h_lsb;
	int v_msb;
} ov5647_mbus_fmts[] = {
	{ V4L2_MBUS_FMT_SBGGR8_1X8,	0, 0 },
	{ V4L2_MBUS_FMT_SGBRG8_1X8,	1, 0 },
	{ V4L2_MBUS_FMT_SGRBG8_1X8,	0, 1 },
	{ V4L2_MBUS_FMT_SRGGB8_1X8,	1, 1 },
	{ V4L2_MBUS_FMT_SBGGR10_1X10,	0, 0 },
	{ V4L2_MBUS_FMT_SGBRG10_1X10,	1, 0 },
	{ V4L2_MBUS_FMT_SGRBG10_1X10,	0, 1 },
	{ V4L2_MBUS_FMT_SRGGB10_1X10,	1, 1 },
};


/* Helpers ---------------------------------------------------------------- */

static uint8_t
ov5647_reg_read(struct ov5647 *s, uint16_t reg)
{
	int rv;
	u8 buf[3];
	struct i2c_msg msgs[2] = {
		{
			.addr  = s->client->addr,
			.flags = 0,
			.len   = 2,
			.buf   = &buf[0],
		},
		{
			.addr  = s->client->addr,
			.flags = I2C_M_RD,
			.len   = 1,
			.buf   = &buf[2],
		},
	};

	buf[0] = (reg >> 8) & 0xff;
	buf[1] =  reg       & 0xff;

	rv = i2c_transfer(s->client->adapter, msgs, 2);

	v4l2_dbg(2, debug, s->client, "%s: 0x%02hhx @ 0x%04hx. (%d)\n",
		__func__, buf[2], reg, rv);

	return (rv == 2) ? buf[2] : 0x00;
}

static int
ov5647_reg_write(struct ov5647 *s, uint16_t reg, uint8_t val)
{
	int rv;
	u8 buf[3];
	struct i2c_msg msg = {
		.addr  = s->client->addr,
		.flags = 0,
		.len   = 3,
		.buf   = &buf[0],
	};

	buf[0] = (reg >> 8) & 0xff;
	buf[1] =  reg       & 0xff;
	buf[2] =  val;

	rv = i2c_transfer(s->client->adapter, &msg, 1);

	v4l2_dbg(2, debug, s->client, "%s: 0x%02hhx @ 0x%04hx. (%d)\n",
		__func__, val, reg, rv);

	return rv != 1;
}

static void
ov5647_power_on(struct ov5647 *s)
{
	if (s->gpio_ena) {
		gpiod_set_value_cansleep(s->gpio_ena, 1);
		msleep(30); /* 5ms ramp-up, 5ms pwdn, 20ms boot */
	}
}

static void
ov5647_power_off(struct ov5647 *s)
{
	if (s->gpio_ena)
		gpiod_set_value_cansleep(s->gpio_ena, 0);
}


/* V4L sub device --------------------------------------------------------- */

 /* Core ops */

static inline struct ov5647 *to_sensor(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov5647, sd);
}


static int
ov5647_co_get_register(struct v4l2_subdev *sd,
                       struct v4l2_dbg_register *reg)
{
	struct ov5647 *s = to_sensor(sd);
	reg->val = ov5647_reg_read(s, reg->reg & 0xffff);
	reg->size = 1;
	return 0;
}

static int
ov5647_co_set_register(struct v4l2_subdev *sd,
                       const struct v4l2_dbg_register *reg)
{
	struct ov5647 *s = to_sensor(sd);
	ov5647_reg_write(s, reg->reg & 0xffff, reg->val & 0xff);
	return 0;
}

static int
ov5647_co_set_power(struct v4l2_subdev *sd, int on)
{
	struct ov5647 *s = to_sensor(sd);

	v4l2_dbg(1, debug, s->client, "%s: on: %d\n", __func__, on);

	if (on) {
		ov5647_power_on(s);
			/* FIXME: Issue init */
	} else
		ov5647_power_off(s);

	return 0;
}

static const struct v4l2_subdev_core_ops ov5647_subdev_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register     = ov5647_co_get_register,
	.s_register     = ov5647_co_set_register,
#endif
	.s_power        = ov5647_co_set_power,
};


 /* Pad ops */

static int
ov5647_po_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
                         struct v4l2_subdev_mbus_code_enum *code)
{
	if ((code->pad != 0) || (code->index >= ARRAY_SIZE(ov5647_mbus_fmts)))
		return -EINVAL;

	code->code = ov5647_mbus_fmts[code->index].code;

	return 0;
}

static int
ov5647_po_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
                          struct v4l2_subdev_frame_size_enum *fse)
{
	return 0; /* FIXME */
}

static int
ov5647_po_enum_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
                              struct v4l2_subdev_frame_interval_enum *fie)
{
	return 0; /* FIXME */
}

static int
ov5647_po_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
                  struct v4l2_subdev_format *format)
{
	return 0; /* FIXME */
}

static int
ov5647_po_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
                  struct v4l2_subdev_format *format)
{
	return 0; /* FIXME */
}

static int
ov5647_po_set_crop(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
                   struct v4l2_subdev_crop *crop)
{
	return 0; /* FIXME */
}

static int
ov5647_po_get_crop(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
                   struct v4l2_subdev_crop *crop)
{
	return 0; /* FIXME */
}

static const struct v4l2_subdev_pad_ops ov5647_subdev_pad_ops = {
	.enum_mbus_code		= ov5647_po_enum_mbus_code,
	.enum_frame_size	= ov5647_po_enum_frame_size,
	.enum_frame_interval	= ov5647_po_enum_frame_interval,
	.get_fmt		= ov5647_po_get_fmt,
	.set_fmt		= ov5647_po_set_fmt,
	.get_crop		= ov5647_po_get_crop,
	.set_crop		= ov5647_po_set_crop,
};


 /* Video ops */

static int
ov5647_vo_set_crystal_freq(struct v4l2_subdev *sd, u32 freq, u32 flags)
{
	struct ov5647 *s = to_sensor(sd);

	if ((freq < 6000000) || (freq > 27000000))
		return -EINVAL;

	s->xclk_freq = freq;

	return 0;
}

static int
ov5647_vo_set_stream(struct v4l2_subdev *sd, int on)
{
	struct ov5647 *s = to_sensor(sd);

	v4l2_dbg(1, debug, s->client, "%s: on: %d\n", __func__, on);

	return 0;
}

static const struct v4l2_subdev_video_ops ov5647_subdev_video_ops = {
	.s_crystal_freq		= ov5647_vo_set_crystal_freq,
	.s_stream		= ov5647_vo_set_stream,
};


 /* SubDev ops */

static const struct v4l2_subdev_ops ov5647_ops = {
	.core  = &ov5647_subdev_core_ops,
	.pad   = &ov5647_subdev_pad_ops,
	.video = &ov5647_subdev_video_ops,
};


/* I2C Driver  ------------------------------------------------------------ */

static int
ov5647_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ov5647 *s;
	int rv;

	/* Get private sensor struct */
	s = devm_kzalloc(&client->dev, sizeof(struct ov5647), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	s->client = client;
	s->client->flags = I2C_CLIENT_SCCB;

	/* Get GPIO */
	s->gpio_ena = devm_gpiod_get(&client->dev, "enable");
	if (!IS_ERR(s->gpio_ena)) {
		gpiod_direction_output(s->gpio_ena, 0);
	} else {
		s->gpio_ena = NULL;
	}

	/* Get XCLK frequency */
	rv = of_property_read_u32(client->dev.of_node,
			"clock-frequency", &s->xclk_freq);
	if (rv < 0)
		s->xclk_freq = 25000000; /* 25 MHz default */

	/* Power-on briefly to check it's the right sensor */
	ov5647_power_on(s);

	if ((ov5647_reg_read(s, 0x300A) != 0x56) ||
	    (ov5647_reg_read(s, 0x300B) != 0x47))
	{
		v4l_err(client, "chip not found\n");
		return -ENODEV;
	}

	ov5647_power_off(s);

	v4l_info(client, "chip found @ 0x%02x (%s)\n",
		client->addr << 1, client->adapter->name);

	/* V4L init */
	v4l2_i2c_subdev_init(&s->sd, client, &ov5647_ops);
	strlcpy(s->sd.name, DRIVER_NAME, sizeof(s->sd.name));

	s->pad.flags = MEDIA_PAD_FL_SOURCE;
	rv = media_entity_init(&s->sd.entity, 1, &s->pad, 0);
	if (rv) {
		v4l_err(client, "Failed to initialized pad\n");
		goto done;
	}

	rv = v4l2_async_register_subdev(&s->sd);
	if (rv) {
		v4l_err(client, "Failed to register subdev for async detection\n");
		goto done;
	}

	rv = 0;

done:
	if (rv > 0) {
		media_entity_cleanup(&s->sd.entity);
	}

	return rv;
}

static int
ov5647_remove(struct i2c_client *client)
{
	struct ov5647 *s = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(&s->sd);

	return 0;
}


static const struct i2c_device_id ov5647_id[] = {
	{ DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov5647_id);

static const struct of_device_id ov5647_of_match[] = {
	{ .compatible = "omnivision,ov5647" },
	{ }
};
MODULE_DEVICE_TABLE(of, ov5647_of_match);

static struct i2c_driver ov5647_driver = {
	.probe		= ov5647_probe,
	.remove		= ov5647_remove,
	.id_table	= ov5647_id,
	.driver		= {
		.owner		= THIS_MODULE,
		.name		= DRIVER_NAME,
		.of_match_table	= ov5647_of_match,
	},
};
module_i2c_driver(ov5647_driver);

MODULE_AUTHOR("Sylvain Munaut <tnt@246tNt.com>");
MODULE_DESCRIPTION("Omnivision OV5647 5MP sensor driver (MIPI CSI-2 mode)");
MODULE_LICENSE("GPL");
