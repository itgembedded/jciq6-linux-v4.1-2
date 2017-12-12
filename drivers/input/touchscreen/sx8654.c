/*
 * Driver for Semtech SX8654 I2C touchscreen controller.
 *
 * Copyright (c) 2015 Armadeus Systems
 *	Sébastien Szymanski <sebastien.szymanski@armadeus.com>
 *
 * Using code from:
 *  - sx865x.c
 *	Copyright (c) 2013 U-MoBo Srl
 *	Pierluigi Passaro <p.passaro@u-mobo.com>
 *  - sx8650.c
 *      Copyright (c) 2009 Wayne Roberts
 *  - tsc2007.c
 *      Copyright (c) 2008 Kwangwoo Lee
 *  - ads7846.c
 *      Copyright (c) 2005 David Brownell
 *      Copyright (c) 2006 Nokia Corporation
 *  - corgi_ts.c
 *      Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], ads7846.h, ts_osk.c
 *      Copyright (C) 2002 MontaVista Software
 *      Copyright (C) 2004 Texas Instruments
 *      Copyright (C) 2005 Dirk Behme
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

/* Q6 - enable debug msgs */
#define DEBUG

#include <linux/input.h>
#include <linux/module.h>
/* #include <linux/of.h> */
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <linux/ktime.h>

/* register addresses */
#define I2C_REG_TOUCH0			0x00
#define I2C_REG_TOUCH1			0x01
#define I2C_REG_TOUCH2			0x02
#define I2C_REG_CHANMASK		0x04
#define I2C_REG_IRQMASK			0x22
#define I2C_REG_IRQSRC			0x23
#define I2C_REG_SOFTRESET		0x3f

/* commands */
#define CMD_READ_REGISTER		0x40
#define CMD_MANUAL			0xc0
#define CMD_PENTRG			0xe0

/* value for I2C_REG_SOFTRESET */
#define SOFTRESET_VALUE			0xde

/* bits for I2C_REG_IRQSRC */
#define IRQ_PENTOUCH_TOUCHCONVDONE	0x08
#define IRQ_PENRELEASE			0x04

/* bits for RegTouch1 */
#define CONDIRQ				0x20
#define FILT_7SA			0x03
#define DEFAULT_REG1_VALUE		0x20

/* bits for I2C_REG_CHANMASK */
#define CONV_X				0x80
#define CONV_Y				0x40

/* coordinates rate: higher nibble of CTRL0 register */
#define RATE_MANUAL			0x00
#define RATE_5000CPS			0xf0

/* power delay: lower nibble of CTRL0 register */
#define POWDLY_1_1MS			0x0b

#define MAX_12BIT			((1 << 12) - 1)

#define SKIP_NEXT_VALUE_DEFAULT		1	/* skip # new press reports after a release */
#define SKIP_TIME_MS_DEFAULT		20	/* in msec */
#define MAX_SKIPS			3	/* #presses skipped after skipnext before try again, prevent blocked press */

struct sx8654 {
	struct input_dev *input;
	struct i2c_client *client;
	int reset_gpio;		/* added for Q6 */
	int activity_gpio;
	int skip_next;		/* added to "debounce" results */
	int skip_next_value;
	int skip_til_release;
	int touch_active;
	int skip_time_ms;
	int debug_msg;
	int first_pair;
	unsigned int prev_x;
	unsigned int prev_y;
	unsigned int valid_since_skipnext;
	unsigned int skipped_since_skipnext;
	ktime_t last_release_ts;
};

static irqreturn_t sx8654_irq(int irq, void *handle)
{
	struct sx8654 *sx8654 = handle;
	int irqsrc;
	u8 data[4];
	unsigned int x, y;
	int retval;
	ktime_t now;

	irqsrc = i2c_smbus_read_byte_data(sx8654->client, CMD_READ_REGISTER | I2C_REG_IRQSRC);
/*	dev_dbg(&sx8654->client->dev, "irqsrc = 0x%x", irqsrc);	*/

	if (irqsrc < 0)
		goto out;

	if (irqsrc & IRQ_PENRELEASE) {
		sx8654->last_release_ts = ktime_get();

		/* only report Release if Touch was currently active */
		if(sx8654->touch_active) {
			input_report_key(sx8654->input, BTN_TOUCH, 0);
			input_sync(sx8654->input);
			sx8654->touch_active = 0;
			if(sx8654->debug_msg)
				dev_dbg(&sx8654->client->dev, "rel");
		}
		else {
			if(sx8654->debug_msg > 1)
				dev_dbg(&sx8654->client->dev, "rel skip");
		}

		sx8654->skip_next = sx8654->skip_next_value;	/* always reset these */
		sx8654->skip_til_release = 0;
		sx8654->first_pair = 1;

		/* Q6 - toggle off Activity LED, regardless of debouncing status */
		gpio_set_value(sx8654->activity_gpio, 0);
	}

	if (irqsrc & IRQ_PENTOUCH_TOUCHCONVDONE) {
/*		dev_dbg(&sx8654->client->dev, "pen touch interrupt");	*/

		if(sx8654->first_pair) {	/* only need to check this if processing first pair since release */
			now = ktime_get();
			if(ktime_to_ms(ktime_sub(now, sx8654->last_release_ts)) < sx8654->skip_time_ms) 
				sx8654->skip_til_release = 1;	
		}

		retval = i2c_master_recv(sx8654->client, data, sizeof(data));
		if (retval != sizeof(data)) {
				dev_dbg(&sx8654->client->dev, "invalid data size %d\n", retval);		
			goto out;
		}

		/* invalid data */
		if (unlikely((data[0] & 0x80) || (data[2] & 0x80))) {
			dev_dbg(&sx8654->client->dev, "invalid data %u, %u\n", data[0], data[2]);		
			goto out;
		}

		x = ((data[0] & 0xf) << 8) | (data[1]);
		y = ((data[2] & 0xf) << 8) | (data[3]);

		if(!sx8654->skip_til_release) {
			if(sx8654->skip_next) { /* skip reporting press, queue'd up */
				sx8654->skip_next--;
				sx8654->valid_since_skipnext = 0;	/* reset values */
				sx8654->skipped_since_skipnext = 0;
				if(sx8654->debug_msg > 1)
					dev_dbg(&sx8654->client->dev, "(%4d,%4d) skipN\n", x, y);
			}
			else {	/* report "previous" x|y press */
				if(sx8654->first_pair) {	/* only used if skip_next_value = 0 */
					sx8654->prev_x = x;
					sx8654->prev_y = y;			
				}		
				input_report_abs(sx8654->input, ABS_X, sx8654->prev_x);
				input_report_abs(sx8654->input, ABS_Y, sx8654->prev_y);
				input_report_key(sx8654->input, BTN_TOUCH, 1);
				input_sync(sx8654->input);
				sx8654->touch_active = 1;
				sx8654->valid_since_skipnext++;
				if(sx8654->debug_msg)
					dev_dbg(&sx8654->client->dev, "(%4d,%4d)\n", sx8654->prev_x, sx8654->prev_y);
			}
		}
		else {
			sx8654->skipped_since_skipnext++;
			if((sx8654->valid_since_skipnext == 0) && (sx8654->skipped_since_skipnext > MAX_SKIPS)) 
				sx8654->skip_til_release = 0;	/* reset it to try again */
			if(sx8654->debug_msg > 1) 
				dev_dbg(&sx8654->client->dev, "(%4d,%4d) skipR\n", x, y);
		}

		/* queue'ing x/y pairs to report 1 conversion later, always skipping last conv upon release (i.e. bounces) */
		sx8654->prev_x = x;
		sx8654->prev_y = y; 
		sx8654->first_pair = 0;

		/* Q6 - toggle on Activity LED, regardless of debouncing status */
		gpio_set_value(sx8654->activity_gpio, 1);
	}

out:
	return IRQ_HANDLED;
}

static int sx8654_open(struct input_dev *dev)
{
	struct sx8654 *sx8654 = input_get_drvdata(dev);
	struct i2c_client *client = sx8654->client;
	int error;

	/* enable pen trigger mode */
	error = i2c_smbus_write_byte_data(client, I2C_REG_TOUCH0,
					  RATE_5000CPS | POWDLY_1_1MS);
	if (error) {
		dev_err(&client->dev, "sx8654: writing to I2C_REG_TOUCH0 failed");
		return error;
	}

	error = i2c_smbus_write_byte(client, CMD_PENTRG);
	if (error) {
		dev_err(&client->dev, "sx8654: writing command CMD_PENTRG failed");
		return error;
	}

	enable_irq(client->irq);

	return 0;
}

static void sx8654_close(struct input_dev *dev)
{
	struct sx8654 *sx8654 = input_get_drvdata(dev);
	struct i2c_client *client = sx8654->client;
	int error;

	disable_irq(client->irq);

	/* enable manual mode mode */
	error = i2c_smbus_write_byte(client, CMD_MANUAL);
	if (error) {
		dev_err(&client->dev, "sx8654: writing command CMD_MANUAL failed");
		return;
	}

	error = i2c_smbus_write_byte_data(client, I2C_REG_TOUCH0, 0);
	if (error) {
		dev_err(&client->dev, "sx8654: writing to I2C_REG_TOUCH0 failed");
		return;
	}
}

/* adding sysfs debug functions */
static ssize_t sx8654_reg0_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sx8654 *sx8654 = dev_get_drvdata(dev);
	int regval;

	regval = i2c_smbus_read_byte_data(sx8654->client, CMD_READ_REGISTER | I2C_REG_TOUCH0);
	return sprintf(buf, "0x%x\n", regval);
}

static ssize_t sx8654_reg1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sx8654 *sx8654 = dev_get_drvdata(dev);
	int regval;

	regval = i2c_smbus_read_byte_data(sx8654->client, CMD_READ_REGISTER | I2C_REG_TOUCH1);
	return sprintf(buf, "0x%x\n", regval);
}

static ssize_t sx8654_reg2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sx8654 *sx8654 = dev_get_drvdata(dev);
	int regval;

	regval = i2c_smbus_read_byte_data(sx8654->client, CMD_READ_REGISTER | I2C_REG_TOUCH2);
	return sprintf(buf, "0x%x\n", regval);
}

static ssize_t sx8654_skip_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sx8654 *sx8654 = dev_get_drvdata(dev);

	return sprintf(buf, "%i\n", sx8654->skip_next_value);
}

static ssize_t sx8654_msec_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sx8654 *sx8654 = dev_get_drvdata(dev);

	return sprintf(buf, "%i\n", sx8654->skip_time_ms);
}

static ssize_t sx8654_debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sx8654 *sx8654 = dev_get_drvdata(dev);

	return sprintf(buf, "%i\n", sx8654->debug_msg);
}

static ssize_t sx8654_reg0_store(struct device *dev, struct device_attribute *attr, 
							const char *buf, size_t count)
{
	struct sx8654 *sx8654 = dev_get_drvdata(dev);
	int regval, error;
	
	if (sscanf(buf, "%x", &regval) != 1) {
		dev_err(&sx8654->client->dev, "sx8654: no value provided in buf for reg0 (%s)\n", buf);	
		return -EINVAL;
	}
	
	regval &= 0xff;	

	error = i2c_smbus_write_byte_data(sx8654->client, I2C_REG_TOUCH0, regval);
	if (error) {
		dev_err(&sx8654->client->dev, "sx8654: writing %d to I2C_REG_TOUCH0 failed\n", regval);
		return -EINVAL;
	}

	return strnlen(buf, count);
}


static ssize_t sx8654_reg1_store(struct device *dev, struct device_attribute *attr, 
							const char *buf, size_t count)
{
	struct sx8654 *sx8654 = dev_get_drvdata(dev);
	int regval, error;
	
	if (sscanf(buf, "%x", &regval) != 1) {
		dev_err(&sx8654->client->dev, "sx8654: no value provided in buf for reg1 (%s)\n", buf);	
		return -EINVAL;
	}
	
	regval &= 0xff;	

	error = i2c_smbus_write_byte_data(sx8654->client, I2C_REG_TOUCH1, regval);
	if (error) {
		dev_err(&sx8654->client->dev, "sx8654: writing %d to I2C_REG_TOUCH1 failed\n", regval);
		return -EINVAL;
	}

	return strnlen(buf, count);
}

static ssize_t sx8654_reg2_store(struct device *dev, struct device_attribute *attr, 
							const char *buf, size_t count)
{
	struct sx8654 *sx8654 = dev_get_drvdata(dev);
	int regval, error;
	
	if (sscanf(buf, "%x", &regval) != 1) {
		dev_err(&sx8654->client->dev, "sx8654: no value provided in buf for reg2 (%s)\n", buf);	
		return -EINVAL;
	}
	
	regval &= 0xff;	

	error = i2c_smbus_write_byte_data(sx8654->client, I2C_REG_TOUCH2, regval);
	if (error) {
		dev_err(&sx8654->client->dev, "sx8654: writing %d to I2C_REG_TOUCH2 failed\n", regval);
		return -EINVAL;
	}

	return strnlen(buf, count);
}

static ssize_t sx8654_skip_store(struct device *dev, struct device_attribute *attr, 
							const char *buf, size_t count)
{
	struct sx8654 *sx8654 = dev_get_drvdata(dev);
	int val;
	
	if (sscanf(buf, "%i", &val) != 1) {
		dev_err(&sx8654->client->dev, "sx8654: no value provided in buf for skip_next_value (%s)\n", buf);	
		return -EINVAL;
	}
	
	sx8654->skip_next_value = val;	
	return strnlen(buf, count);
}

static ssize_t sx8654_msec_store(struct device *dev, struct device_attribute *attr, 
							const char *buf, size_t count)
{
	struct sx8654 *sx8654 = dev_get_drvdata(dev);
	int val;
	
	if (sscanf(buf, "%i", &val) != 1) {
		dev_err(&sx8654->client->dev, "sx8654: no value provided in buf for skip_time_msec (%s)\n", buf);	
		return -EINVAL;
	}
	
	sx8654->skip_time_ms = val;	
	return strnlen(buf, count);
}

static ssize_t sx8654_debug_store(struct device *dev, struct device_attribute *attr, 
							const char *buf, size_t count)
{
	struct sx8654 *sx8654 = dev_get_drvdata(dev);
	int val;
	
	if (sscanf(buf, "%i", &val) != 1) {
		dev_err(&sx8654->client->dev, "sx8654: no value provided in buf for debug_msg (%s)\n", buf);	
		return -EINVAL;
	}
	
	sx8654->debug_msg = val;	
	return strnlen(buf, count);
}


static DEVICE_ATTR(reg0, 0644, sx8654_reg0_show, sx8654_reg0_store);
static DEVICE_ATTR(reg1, 0644, sx8654_reg1_show, sx8654_reg1_store);
static DEVICE_ATTR(reg2, 0644, sx8654_reg2_show, sx8654_reg2_store);
static DEVICE_ATTR(skip, 0644, sx8654_skip_show, sx8654_skip_store);
static DEVICE_ATTR(msec, 0644, sx8654_msec_show, sx8654_msec_store);
static DEVICE_ATTR(debug, 0644, sx8654_debug_show, sx8654_debug_store);


static int sx8654_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sx8654 *sx8654;
	struct input_dev *input;
	int error;
	struct device_node *np;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -ENXIO;

	sx8654 = devm_kzalloc(&client->dev, sizeof(*sx8654), GFP_KERNEL);
	if (!sx8654)
		return -ENOMEM;

	input = devm_input_allocate_device(&client->dev);
	if (!input)
		return -ENOMEM;

	/* adding sysfs debug functions */
	error = device_create_file(&client->dev, &dev_attr_reg0);
	if(error < 0 )
		dev_err(&client->dev, "sx8654: failed to create reg0 file (%d)\n", error);
	error = device_create_file(&client->dev, &dev_attr_reg1);
	if(error < 0 )
		dev_err(&client->dev, "sx8654: failed to create reg1 file (%d)\n", error);
	error = device_create_file(&client->dev, &dev_attr_reg2);
	if(error < 0 )
		dev_err(&client->dev, "sx8654: failed to create reg2 file (%d)\n", error);
	error = device_create_file(&client->dev, &dev_attr_skip);
	if(error < 0 )
		dev_err(&client->dev, "sx8654: failed to create skip file (%d)\n", error);
	error = device_create_file(&client->dev, &dev_attr_msec);
	if(error < 0 )
		dev_err(&client->dev, "sx8654: failed to create msec file (%d)\n", error);
	error = device_create_file(&client->dev, &dev_attr_debug);
	if(error < 0 )
		dev_err(&client->dev, "sx8654: failed to create debug file (%d)\n", error);
	

	input->name = "SX8654 I2C Touchscreen";
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;
	input->open = sx8654_open;
	input->close = sx8654_close;

	__set_bit(INPUT_PROP_DIRECT, input->propbit);
	input_set_capability(input, EV_KEY, BTN_TOUCH);
	input_set_abs_params(input, ABS_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, MAX_12BIT, 0, 0);

	sx8654->client = client;
	sx8654->input = input;

	/* Q6 - init reset and activity GPIOs per Device Tree */
	sx8654->reset_gpio = -1;
	sx8654->activity_gpio = -1;
	np = client->dev.of_node;
	if(np) {
		sx8654->reset_gpio = of_get_named_gpio(np, "rs-gpios", 0);
		if(gpio_is_valid(sx8654->reset_gpio)) {
			if(devm_gpio_request_one(&client->dev, sx8654->reset_gpio, GPIOF_OUT_INIT_HIGH, "rs-gpio"))
				dev_err(&client->dev, "sx8654: failed to init Reset GPIO (%d)\n", sx8654->reset_gpio);
		}
		sx8654->activity_gpio = of_get_named_gpio(np, "act-gpios", 0);
		if(gpio_is_valid(sx8654->activity_gpio)) {
			if(devm_gpio_request_one(&client->dev, sx8654->activity_gpio, GPIOF_OUT_INIT_LOW, "act-gpio"))
				dev_err(&client->dev, "sx8654: failed to init Activity GPIO (%i)\n", sx8654->activity_gpio);
		}
	}	

	/* init other Q6 vars */
	sx8654->skip_next = SKIP_NEXT_VALUE_DEFAULT;		/* skip first touch(es) */
	sx8654->skip_next_value = SKIP_NEXT_VALUE_DEFAULT;	/* skip first touch(es) */
	sx8654->skip_time_ms = SKIP_TIME_MS_DEFAULT;
	sx8654->skip_til_release = 0;
	sx8654->touch_active = 0;				/* touch not active */
	sx8654->last_release_ts = ktime_set(0,0);		/* zero timestamp */
	sx8654->debug_msg = 0;					/* debug msgs OFF by default */
	sx8654->first_pair = 1;					/* initially, waiting for first x|y pair */
	sx8654->valid_since_skipnext = 0;
	sx8654->skipped_since_skipnext = 0;

	input_set_drvdata(sx8654->input, sx8654);

	error = i2c_smbus_write_byte_data(client, I2C_REG_SOFTRESET, SOFTRESET_VALUE);
	if (error) {
		dev_err(&client->dev, "sx8654: writing softreset value failed");
		return error;
	}

	error = i2c_smbus_write_byte_data(client, I2C_REG_CHANMASK, CONV_X | CONV_Y);
	if (error) {
		dev_err(&client->dev, "sx8654: writing to I2C_REG_CHANMASK failed");
		return error;
	}

	error = i2c_smbus_write_byte_data(client, I2C_REG_IRQMASK, IRQ_PENTOUCH_TOUCHCONVDONE | IRQ_PENRELEASE);
	if (error) {
		dev_err(&client->dev, "sx8654: writing to I2C_REG_IRQMASK failed");
		return error;
	}

	error = i2c_smbus_write_byte_data(client, I2C_REG_TOUCH1, DEFAULT_REG1_VALUE);
	if (error) {
		dev_err(&client->dev, "sx8654: writing to I2C_REG_TOUCH1 failed");
		return error;
	}

	error = devm_request_threaded_irq(&client->dev, client->irq,
					  NULL, sx8654_irq,
					  IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					  client->name, sx8654);
	if (error) {
		dev_err(&client->dev,
			"sx8654: Failed to enable IRQ %d, error: %d\n",
			client->irq, error);
		return error;
	}

	/* Disable the IRQ, we'll enable it in sx8654_open() */
	disable_irq(client->irq);

	error = input_register_device(sx8654->input);
	if (error)
		return error;

	i2c_set_clientdata(client, sx8654);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sx8654_of_match[] = {
	{ .compatible = "semtech,sx8654", },
	{ },
};
MODULE_DEVICE_TABLE(of, sx8654_of_match);
#endif

static const struct i2c_device_id sx8654_id_table[] = {
	{ "semtech_sx8654", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, sx8654_id_table);

static struct i2c_driver sx8654_driver = {
	.driver = {
		.name = "sx8654",
		.of_match_table = of_match_ptr(sx8654_of_match),
	},
	.id_table = sx8654_id_table,
	.probe = sx8654_probe,
};
module_i2c_driver(sx8654_driver);

MODULE_AUTHOR("Sébastien Szymanski <sebastien.szymanski@armadeus.com>");
MODULE_DESCRIPTION("Semtech SX8654 I2C Touchscreen Driver");
MODULE_LICENSE("GPL");
