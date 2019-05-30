/*
* FPC1020 Fingerprint sensor device driver
*
* This driver will control the platform resources that the FPC fingerprint
* sensor needs to operate. The major things are probing the sensor to check
* that it is actually connected and let the Kernel know this and with that also
* enabling and disabling of regulators, enabling and disabling of platform
* clocks, controlling GPIOs such as SPI chip select, sensor reset line, sensor
* IRQ line, MISO and MOSI lines.
*
* The driver will expose most of its available functionality in sysfs which
* enables dynamic control of these features from eg. a user space process.
*
* The sensor's IRQ events will be pushed to Kernel's event handling system and
* are exposed in the drivers event node. This makes it possible for a user
* space process to poll the input node and receive IRQ events easily. Usually
* this node is available under /dev/input/eventX where 'X' is a number given by
* the event system. A user space process will need to traverse all the event
* nodes and ask for its parent's name (through EVIOCGNAME) which should match
* the value in device tree named input-device-name.
*
* This driver will NOT send any SPI commands to the sensor it only controls the
* electrical parts.
*
*
* Copyright (c) 2015 Fingerprint Cards AB <tech@fingerprints.com>
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License Version 2
* as published by the Free Software Foundation.
*/
#include "fpc1020_tee.h"

static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);

static int fpc1020_request_named_gpio(struct fpc1020_data *fpc1020,
		const char *label, int *gpio)
{
	struct device *dev = fpc1020->dev;
	struct device_node *np = dev->of_node;
	int rc = of_get_named_gpio(np, label, 0);
	if (rc < 0) {
		FPC_LOG_ERROR("failed to get '%s'\n", label);
		return rc;
	}
	*gpio = rc;
	rc = devm_gpio_request(dev, *gpio, label);
	if (rc) {
		FPC_LOG_ERROR("failed to request gpio %d\n", *gpio);
		return rc;
	}
	return rc;
}


static int fpc1020_pinctrl_init(struct fpc1020_data *fpc1020)
{
	int ret = 0;
	struct device *dev = fpc1020->dev;

	fpc1020->ts_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(fpc1020->ts_pinctrl)) {
		FPC_LOG_ERROR("Target does not use pinctrl\n");
		ret = PTR_ERR(fpc1020->ts_pinctrl);
		goto err;
	}

	fpc1020->gpio_state_active = pinctrl_lookup_state(fpc1020->ts_pinctrl, "pmx_fp_active");
	if (IS_ERR_OR_NULL(fpc1020->gpio_state_active)) {
		FPC_LOG_ERROR("Cannot get active pinstate\n");
		ret = PTR_ERR(fpc1020->gpio_state_active);
		goto err;
	}

	fpc1020->gpio_state_suspend = pinctrl_lookup_state(fpc1020->ts_pinctrl, "pmx_fp_suspend");
	if (IS_ERR_OR_NULL(fpc1020->gpio_state_suspend)) {
		FPC_LOG_ERROR("Cannot get sleep pinstate\n");
		ret = PTR_ERR(fpc1020->gpio_state_suspend);
		goto err;
	}

	return 0;
err:
	fpc1020->ts_pinctrl = NULL;
	fpc1020->gpio_state_active = NULL;
	fpc1020->gpio_state_suspend = NULL;
	return ret;
}

static int fpc1020_pinctrl_select(struct fpc1020_data *fpc1020, bool on)
{
	int ret = 0;
	struct pinctrl_state *pins_state;

	pins_state = on ? fpc1020->gpio_state_active : fpc1020->gpio_state_suspend;
	if (IS_ERR_OR_NULL(pins_state)) {
		FPC_LOG_ERROR("not a valid '%s' pinstate\n",
			on ? "pmx_ts_active" : "pmx_ts_suspend");
		return -1;
	}

	ret = pinctrl_select_state(fpc1020->ts_pinctrl, pins_state);
	if (ret) {
		FPC_LOG_ERROR("can not set %s pins\n",
			on ? "pmx_ts_active" : "pmx_ts_suspend");
	}

	return ret;
}

/**
* sysfs node for controlling whether the driver is allowed
* to wake up the platform on interrupt.
*/
static ssize_t wakeup_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "enable", strlen("enable"))) {
		fpc1020->wakeup_enabled = true;
	} else if (!strncmp(buf, "disable", strlen("disable"))) {
		fpc1020->wakeup_enabled = false;
	} else
		return -EINVAL;

	return count;
}
static DEVICE_ATTR(wakeup_enable, S_IWUSR, NULL, wakeup_enable_set);

/**
* sysf node to check the interrupt status of the sensor, the interrupt
* handler should perform sysf_notify to allow userland to poll the node.
*/
static ssize_t irq_get(struct device *device,
				struct device_attribute *attribute,
				char* buffer)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(device);
	int irq = gpio_get_value(fpc1020->irq_gpio);
	return scnprintf(buffer, PAGE_SIZE, "%i\n", irq);
}


/**
* writing to the irq node will just drop a printk message
* and return success, used for latency measurement.
*/
static ssize_t irq_ack(struct device *device,
				struct device_attribute *attribute,
				const char *buffer, size_t count)
{
	FPC_LOG_INFO("enter\n");
	return count;
}
static DEVICE_ATTR(irq, S_IRUSR | S_IWUSR, irq_get, irq_ack);

static struct attribute *attributes[] = {
	&dev_attr_wakeup_enable.attr,
	&dev_attr_irq.attr,
	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

static irqreturn_t fpc1020_irq_handler(int irq, void *handle)
{
	struct fpc1020_data *fpc1020 = handle;
	FPC_LOG_DEBUG("enter\n");
	if (fpc1020->wakeup_enabled) {
		wake_lock_timeout(&fpc1020->ttw_wl,
					msecs_to_jiffies(FPC_TTW_HOLD_TIME));
	}

	sysfs_notify(&fpc1020->dev->kobj, NULL, dev_attr_irq.attr.name);

	return IRQ_HANDLED;
}

static int fpc1020_parse_dt(struct fpc1020_data *fpc1020)
{
	int rc = -1;

	if (IS_ERR_OR_NULL(fpc1020)) {
		FPC_LOG_ERROR("null pointer\n");
		return -ENODEV;
	}

	rc = fpc1020_request_named_gpio(fpc1020, "fpc,enable-vdd-ldo",
			&fpc1020->enable_gpio);
	if (rc) {
		return -ENODEV;
	}
	FPC_LOG_INFO("fpc1020->enable_gpio is %d\n", fpc1020->enable_gpio);

	rc = fpc1020_request_named_gpio(fpc1020, "fpc,enable-vddio-ldo",
			&fpc1020->enable_vddio_gpio);
	if (rc) {
		return -ENODEV;
	}
	FPC_LOG_INFO("fpc1020->enable_vddio_gpio is %d\n", fpc1020->enable_vddio_gpio);

	rc = fpc1020_request_named_gpio(fpc1020, "fpc,reset-gpio",
			&fpc1020->rst_gpio);
	if (rc) {
		return -ENODEV;
	}
	FPC_LOG_INFO("fpc1020->rst_gpio is %d\n", fpc1020->rst_gpio);

	rc = fpc1020_request_named_gpio(fpc1020, "fpc,irq-gpio",
			&fpc1020->irq_gpio);
	if (rc) {
		return -ENODEV;
	}
	FPC_LOG_INFO("fpc1020->irq_gpio is %d\n", fpc1020->irq_gpio);

	return 0;
}

static int fpc1020_hw_configure(struct fpc1020_data *fpc1020)
{
	int rc;

	if (IS_ERR_OR_NULL(fpc1020)) {
		FPC_LOG_ERROR("null pointer\n");
		goto exit;
	}

	rc = gpio_direction_output(fpc1020->enable_gpio, 1);
	if (rc) {
		FPC_LOG_ERROR("set enable gpio direction error\n");
		goto exit;
	}
	rc = gpio_direction_output(fpc1020->enable_vddio_gpio, 1);
	if (rc) {
		FPC_LOG_ERROR("set vddio ldo gpio direction error\n");
		goto exit;
	}
	rc = gpio_direction_output(fpc1020->rst_gpio, 1);
	if (rc) {
		FPC_LOG_ERROR("set reset gpio direction error\n");
		goto exit;
	}
	rc = gpio_direction_input(fpc1020->irq_gpio);
	if (rc) {
		FPC_LOG_ERROR("set irq gpio direction error\n");
		goto exit;
	}

	rc = fpc1020_pinctrl_init(fpc1020);
	if (rc)
		goto exit;

	rc = fpc1020_pinctrl_select(fpc1020, true);
	if (rc)
		goto exit;
	FPC_LOG_INFO("fpc hw configure finish\n");
	return 0;
exit:
	if (fpc1020->irq_gpio)
		gpio_free(fpc1020->irq_gpio);
	if (fpc1020->rst_gpio)
		gpio_free(fpc1020->rst_gpio);
	if (fpc1020->enable_vddio_gpio)
		gpio_free(fpc1020->enable_vddio_gpio);
	if (fpc1020->enable_gpio)
		gpio_free(fpc1020->enable_gpio);
	return -ENODEV;
}
static void fpc1020_regulator_power_on(struct fpc1020_data *fpc1020, bool on)
{
	if (IS_ERR_OR_NULL(fpc1020)) {
		FPC_LOG_ERROR("null pointer\n");
		return;
	}
	if(on) {
		gpio_set_value(fpc1020->enable_gpio, 1);
		gpio_set_value(fpc1020->enable_vddio_gpio, 1);

	} else {
		gpio_set_value(fpc1020->enable_gpio, 0);
		gpio_set_value(fpc1020->enable_vddio_gpio, 0);
	}
	FPC_LOG_INFO("fpc power %s\n", on ? "on" : "off");
}
static void fpc1020_device_reset(struct fpc1020_data *fpc1020)
{
	if (IS_ERR_OR_NULL(fpc1020)) {
		FPC_LOG_ERROR("null pointer\n");
		return;
	}
	gpio_set_value(fpc1020->rst_gpio, 1);
	udelay(FPC1020_RESET_HIGH1_US);

	gpio_set_value(fpc1020->rst_gpio, 0);
	udelay(FPC1020_RESET_LOW_US);

	gpio_set_value(fpc1020->rst_gpio, 1);
	udelay(FPC1020_RESET_HIGH2_US);
}
static int fpc1020_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fpc1020_data *fpc1020;
	int rc = 0;

	FPC_LOG_INFO("probe start\n");
	fpc1020 = devm_kzalloc(dev, sizeof(*fpc1020), GFP_KERNEL);
	if (!fpc1020) {
		FPC_LOG_ERROR("failed to allocate memory for struct fpc1020_data\n");
		rc = -ENOMEM;
		goto exit;
	}

	fpc1020->dev = dev;
	fpc1020->wakeup_enabled = false;
	dev_set_drvdata(dev, fpc1020);
	INIT_LIST_HEAD(&fpc1020->device_entry);
	mutex_init(&fpc1020->lock);
	wake_lock_init(&fpc1020->ttw_wl, WAKE_LOCK_SUSPEND, "fpc_ttw_wl");

	/* parse device tree*/
	rc = fpc1020_parse_dt(fpc1020);
	if (rc < 0) {
		FPC_LOG_ERROR("parse device tree fail\n");
		goto exit;
	}
	/* fpc1020 hradware configure */
	rc = fpc1020_hw_configure(fpc1020);
	if (rc < 0) {
		FPC_LOG_ERROR("fpc1020 hardware init fail\n");
		goto exit;
	}
	fpc1020_regulator_power_on(fpc1020, true);

	/* create the sysfs node */
	fpc1020->class = class_create(THIS_MODULE, FPC1020_CLASS_NAME);
	fpc1020->minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (fpc1020->minor < N_SPI_MINORS) {
		fpc1020->dev_num = MKDEV(SPIDEV_MAJOR, fpc1020->minor);
		fpc1020->node_dev = device_create(fpc1020->class, &pdev->dev, fpc1020->dev_num, fpc1020, FPC1020_DEV_NAME);
		rc = IS_ERR(fpc1020->node_dev) ? PTR_ERR(fpc1020->node_dev) : 0;
	} else {
		FPC_LOG_ERROR("no minor number available\n");
		rc = -ENODEV;
	}

	if (rc == 0) {
		set_bit(fpc1020->minor, minors);
		list_add(&fpc1020->device_entry, &device_list);
	} else {
		FPC_LOG_ERROR("failed to mkdev\n");
		goto exit_hw_put;
	}
	rc = sysfs_create_group(&dev->kobj, &attribute_group);
	if (rc) {
		FPC_LOG_ERROR("could not create sysfs\n");
		goto exit_create_sysfs;
	}

	/* fpc1020 IRQ configure */
	rc = devm_request_threaded_irq(dev, gpio_to_irq(fpc1020->irq_gpio),
			NULL, fpc1020_irq_handler, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			dev_name(dev), fpc1020);
	if (rc) {
		FPC_LOG_ERROR("could not request irq %d\n",
				gpio_to_irq(fpc1020->irq_gpio));
		goto exit_create_sysfs;
	}
	FPC_LOG_INFO("requested irq %d\n", gpio_to_irq(fpc1020->irq_gpio));

	/* Request that the interrupt should be wakeable */
	enable_irq_wake(gpio_to_irq(fpc1020->irq_gpio));

	/* reset device */
	fpc1020_device_reset(fpc1020);

	FPC_LOG_INFO("probe ok\n");
	return 0;

exit_create_sysfs:
	device_destroy(fpc1020->class, fpc1020->dev_num);
	class_destroy(fpc1020->class);
exit_hw_put:
	fpc1020_regulator_power_on(fpc1020, false);
exit:
	FPC_LOG_ERROR("exit\n");
	return rc;
}

static int fpc_open(struct inode *inode, struct file *filp)
{
	int	rc = 0;
	FPC_LOG_INFO("\n");
	return rc;
}

static const struct file_operations fpc_fops = {
	.owner =	THIS_MODULE,
	//.write = fpc_write,
	//.read  = fpc_read,
	//.unlocked_ioctl = fpc_ioctl,
#ifdef CONFIG_COMPAT
	//.compat_ioctl = fpc_compat_ioctl,
#endif
	.open    = fpc_open,
	//.release = fpc_release,
	//.poll    = fpc_poll,
	//.fasync  = fpc_fasync,
};

static struct of_device_id fpc1020_of_match[] = {
	{ .compatible = "fpc,fpc1020", },
	{}
};
MODULE_DEVICE_TABLE(of, fpc1020_of_match);

static struct platform_driver fpc1020_driver = {
	.driver = {
		.name		= "fpc1020",
		.owner		= THIS_MODULE,
		.of_match_table = fpc1020_of_match,
	},
	.probe = fpc1020_probe,
};

static int __init fpc1020_init(void)
{
	int rc = 0;

#ifdef CONFIG_NUBIA_FP_AUTODETECT
	if (fingerprint_device_autodetect(AUTODETECT_NAME)==false) {
		return -ENODEV;
	}
#endif

	rc = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &fpc_fops);
	if (rc < 0) {
		FPC_LOG_ERROR("failed to register char device\n");
		return rc;
	}

	return platform_driver_register(&fpc1020_driver);
}
module_init(fpc1020_init);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aleksej Makarov");
MODULE_AUTHOR("Henrik Tillman <henrik.tillman@fingerprints.com>");
MODULE_AUTHOR("Martin Trulsson <martin.trulsson@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 Fingerprint sensor device driver.");
