/*
 * platform indepent driver interface
 *
 * Coypritht (c) 2017 Goodix
 */
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#include "gf_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

int gf_parse_dts(struct gf_dev* gf_dev)
{
	int rc = 0;

	/*get reset resource*/
	gf_dev->reset_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node,"fp-gpio-reset",0);
	if(!gpio_is_valid(gf_dev->reset_gpio)) {
		SENSOR_LOG_ERROR("RESET GPIO is invalid\n");
		return -1;
	}

	rc = gpio_request(gf_dev->reset_gpio, "goodix_reset");
	if(rc) {
		SENSOR_LOG_ERROR("Failed to request RESET GPIO. rc = %d\n", rc);
		return -1;
	}

	gpio_direction_output(gf_dev->reset_gpio, 1);

	/*get irq resourece*/
	gf_dev->irq_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node,"fp-gpio-irq",0);
	pr_info("gf::irq_gpio:%d\n", gf_dev->irq_gpio);
	if(!gpio_is_valid(gf_dev->irq_gpio)) {
		SENSOR_LOG_ERROR("IRQ GPIO is invalid\n");
		return -1;
	}

	rc = gpio_request(gf_dev->irq_gpio, "goodix_irq");
	if(rc) {
		SENSOR_LOG_ERROR("Failed to request IRQ GPIO. rc = %d\n", rc);
		return -1;
	}
	gpio_direction_input(gf_dev->irq_gpio);

	return 0;
}

void gf_cleanup(struct gf_dev	* gf_dev)
{
	pr_info("[info] %s\n",__func__);
	if (gpio_is_valid(gf_dev->irq_gpio))
	{
		gpio_free(gf_dev->irq_gpio);
		pr_info("remove irq_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->reset_gpio))
	{
		gpio_free(gf_dev->reset_gpio);
		pr_info("remove reset_gpio success\n");
	}
}

int gf_power_control(struct gf_dev* gf_dev, bool enable)
{
	int rc = 0;

    if (gf_dev->power_on == enable) {
        SENSOR_LOG_ERROR("power double %s\n",enable ? "enable" : "disable");
        goto err_double_power_control;
    } else {
        gf_dev->power_on = enable;

        if (gf_dev->power_on) {
            rc = regulator_enable(gf_dev->vdd);
        } else {
            rc = regulator_disable(gf_dev->vdd);
        }

        if (rc) {
            SENSOR_LOG_ERROR("power %s failed. rc=%d\n",enable ? "enable" : "disable", rc);
            gf_dev->power_on = !enable;
        } else {
            SENSOR_LOG_ERROR("power %s success",enable ? "enable" : "disable");
        }

    }

err_double_power_control:
    return rc;
}

int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	if(gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -1;
	}
	gpio_direction_output(gf_dev->reset_gpio, 1);
	gpio_set_value(gf_dev->reset_gpio, 0);
	mdelay(3);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay_ms);
	return 0;
}

int gf_irq_num(struct gf_dev *gf_dev)
{
	if(gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -1;
	} else {
		return gpio_to_irq(gf_dev->irq_gpio);
	}
}

