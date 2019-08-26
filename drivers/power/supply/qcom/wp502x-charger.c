/* Copyright (c) 2014-2015 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt) "wp502x: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/debugfs.h>
#include <linux/pmic-voter.h>
#if defined(CONFIG_FB)
#include <linux/fb.h>
#endif
/***********************************************************************/
#define FAILURE_RETRY(expression) \
	({\
	int __result;\
	int __timeout = 5;\
	do\
	{\
	__result = (expression);\
	usleep_range(100000, 110000);\
	}\
	while(__result != 0 && __timeout-- > 0);\
	if(__timeout <= 0){\
	pr_err("wp502x i2c timeout.\n");\
	return -1;\
	}\
	})\

#define FAILURE_RETRY_VOID(expression) \
	({\
	int __result;\
	int __timeout = 5;\
	do\
	{\
	__result = (expression);\
	usleep_range(100000, 110000);\
	}\
	while(__result != 0 && __timeout-- > 0);\
	if(__timeout <= 0){\
	pr_err("wp502x i2c timeout.\n");\
	return;\
	}\
	})\

typedef struct i2c_bus_data {
	int scl;
	int	sda;
	unsigned char 	addr;
	unsigned char 	udelay;
	struct mutex	io_lock;
}i2c_bus_data_t;

/************************************************************************/
enum {
	USBIN_NORMAL_CONFIGS	= 0x55,
	USBIN_IDETIFY_CONFIGS	= 0xA5,
	USBIN_DIRECT_CONFIGS	= 0xAA,
};

struct dt_params {
	int 	cfg_irq_gpio;
	int 	cfg_en_gpio;
	int		cfg_vref_1p8_en_gpio;
	int		cfg_idetify_threshold;
	int		cfg_pcba_resistance;
	int		cfg_support_current_threshold;
	int 	bat_cfg_cool_temp;
	int		bat_cfg_clod_temp;
	int 	bat_cfg_hot_temp;
	int 	bat_cfg_fcc_limited_temp;
	int		bat_cfg_fastchg_current;
};

struct wp502x_chip
{
	struct device			*dev;
	struct dt_params		dt;

	int						batt_ma;
	int						batt_mv;
	int						batt_soc;
	int						batt_temp;
	int						batt_temp_limit_mask;
	int						usb_input_mv;
	int						wp502x_irq;
	int						wp502x_enable;

	struct i2c_bus_data		i2c_bus;

	struct mutex			control_lock;
	struct delayed_work	    monitor_work;
#if defined(CONFIG_FB)
	struct notifier_block 	fb_notifier;
#endif
	struct votable			*dc_awake_votable;
	struct votable			*direct_chg_enable_votable;
	struct votable			*apsd_rerun_votable;
	struct dentry			*debug_root;

	struct power_supply		*batt_psy;
	struct power_supply		*usb_psy;
	struct power_supply		*main_psy;
#if defined(CONFIG_NEO_EXTERNAL_FG_SUPPORT)
	struct power_supply		*ex_batt_psy;
#endif
};

static enum power_supply_property wp502x_main_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
};

static inline int get_sda(struct wp502x_chip *chip)
{
	return gpio_get_value(chip->i2c_bus.sda);
}

static inline void sda_low(struct wp502x_chip *chip)
{
	gpio_direction_output(chip->i2c_bus.sda, 0);
}

static inline void sda_high(struct wp502x_chip *chip)
{
	gpio_direction_input(chip->i2c_bus.sda);
}

static inline void scl_low(struct wp502x_chip *chip)
{
	gpio_direction_output(chip->i2c_bus.scl, 0);
}

static inline void scl_high(struct wp502x_chip *chip)
{
	gpio_direction_input(chip->i2c_bus.scl);
}

static void i2c_start(struct wp502x_chip *chip)
{
	sda_high(chip);
	scl_high(chip);
	udelay(chip->i2c_bus.udelay * 10);
	sda_low(chip);
	udelay(chip->i2c_bus.udelay);
	scl_low(chip);
	udelay(chip->i2c_bus.udelay);
}

static void i2c_stop(struct wp502x_chip *chip)
{
	sda_low(chip);
	scl_high(chip);
	udelay(chip->i2c_bus.udelay);
	sda_high(chip);
	udelay(chip->i2c_bus.udelay);
}

static inline void i2c_send_ack(struct wp502x_chip *chip, unsigned char ack)
{
	ack ? sda_high(chip) : sda_low(chip);
	scl_high(chip);
	udelay(chip->i2c_bus.udelay);
	scl_low(chip);
	udelay(chip->i2c_bus.udelay);
}

/* send a byte without start cond., look for arbitration,
   check ackn. from slave */
/* returns:
 * 1 if the device acknowledged
 * 0 if the device did not ack
 * -ETIMEDOUT if an error occurred (while raising the scl line)
 */
static int i2c_outb(struct wp502x_chip *chip, unsigned char c)
{
	int i;
	int sb;
	int ack;

	/* assert: scl is low */
	for (i = 7; i >= 0; i--) {
		sb = (c >> i) & 1;
		scl_low(chip);
		udelay(chip->i2c_bus.udelay);
		sb ? sda_high(chip) : sda_low(chip);
		scl_high(chip);
		udelay(chip->i2c_bus.udelay);
	}
	scl_low(chip);
	sda_high(chip);
	udelay(chip->i2c_bus.udelay);
	scl_high(chip);
	udelay(chip->i2c_bus.udelay);
	ack = !get_sda(chip);    /* ack: sda is pulled low -> success */
	if(!ack) {
		pr_err("i2c_outb: 0x%02x %s\n", (int)c, ack ? "A" : "NA");
		usleep_range(500000, 510000);
	}
	udelay(chip->i2c_bus.udelay);
	scl_low(chip);

	return ack;
}

static int i2c_inb(struct wp502x_chip *chip)
{
	/* read byte via i2c port, without start/stop sequence	*/
	/* acknowledge is sent in i2c_read.			*/
	int i;
	unsigned char indata = 0;

	/* assert: scl is low */
	sda_high(chip);
	for (i = 0; i < 8; i++) {
		indata <<= 1;
		scl_high(chip);
		if (get_sda(chip))
			indata |= 0x01;
		else
			indata &= 0xFE;
		udelay(chip->i2c_bus.udelay);
		scl_low(chip);
		udelay(chip->i2c_bus.udelay);
	}
	/* assert: scl is low */
	return indata;
}

static int i2c_read(struct wp502x_chip *chip, unsigned char reg, unsigned char *data)
{
	int rc;
	mutex_lock(&chip->i2c_bus.io_lock);
	i2c_start(chip);
	rc = i2c_outb(chip, chip->i2c_bus.addr);
	if(rc != 1){
		pr_err("Faild to send i2c addr.\n");
		goto ERROR;
	}
	rc = i2c_outb(chip, reg);
	if(rc != 1){
		pr_err("Faild to send i2c reg addr.\n");
		goto ERROR;
	}
	i2c_start(chip);
	rc = i2c_outb(chip, chip->i2c_bus.addr | 0x01);
	if(rc != 1){
		pr_err("Faild to send i2c addr.\n");
		goto ERROR;
	}
	*data = i2c_inb(chip);
	i2c_send_ack(chip, true);
	i2c_stop(chip);
	mutex_unlock(&chip->i2c_bus.io_lock);
	return 0;
ERROR:
	i2c_stop(chip);
	pr_err("Failed to read 0x%x.\n", reg);
	mutex_unlock(&chip->i2c_bus.io_lock);
	return -1;
}

static int i2c_write(struct wp502x_chip *chip, unsigned char reg, unsigned char data)
{
	int rc;
	mutex_lock(&chip->i2c_bus.io_lock);
	i2c_start(chip);
	rc = i2c_outb(chip, chip->i2c_bus.addr);
	if(rc != 1){
		pr_err("Faild to send i2c addr.\n");
		goto ERROR;
	}
	rc = i2c_outb(chip, reg);
	if(rc != 1){
		pr_err("Faild to send reg addr.\n");
		goto ERROR;
	}
	rc = i2c_outb(chip, data);
	if(rc != 1){
		pr_err("Faild to send data.\n");
		goto ERROR;
	}
	i2c_stop(chip);
	mutex_unlock(&chip->i2c_bus.io_lock);
	return 0;
ERROR:
	i2c_stop(chip);
	pr_err("Failed to write 0x%x.\n", reg);
	mutex_unlock(&chip->i2c_bus.io_lock);
	return -1;
}

static int wp502x_charger_set_pcba_resistance(struct wp502x_chip *chip, int mohm)
{
	unsigned char val = 0;
	/** Try to check pcba impedance, it should be set once. */
	FAILURE_RETRY(i2c_read(chip, 0x06, &val));

	if (val == mohm) {
		pr_info("wp502x get pcba resistance %d mohm.\n", val);
		return 0;
	}

	pr_info("wp502x get pcba resistance %d mohm, try to set default %d mohm.\n", val, mohm);

	FAILURE_RETRY(i2c_write(chip, 0x08, 0xa5)); //Clear IIC REG Lock 1
	usleep_range(100000, 110000);
	FAILURE_RETRY(i2c_write(chip, 0x09, 0x5a)); //Clear IIC REG Lock 2
	usleep_range(100000, 110000);
	FAILURE_RETRY(i2c_write(chip, 0x06, mohm));
	FAILURE_RETRY(i2c_read(chip, 0x06, &val));

	FAILURE_RETRY(i2c_write(chip, 0x08, 0x00)); //Clear IIC REG Lock 1
	usleep_range(100000, 110000);
	FAILURE_RETRY(i2c_write(chip, 0x09, 0x00)); //Clear IIC REG Lock 2
	usleep_range(100000, 110000);

	if(val != mohm){
		pr_err("set pcba resistance failed.\n");
		return -1;
	}

	return 0;
}

static int wp502x_charger_set_max_charge_current(struct wp502x_chip *chip, int batt_ma)
{
	unsigned char stat = 0;
	unsigned char val = 0;

	val = batt_ma/100;
	if((val > 80) || (val < 20))
		val = 50;

	/** Try to check max current, it should be set once. */
	FAILURE_RETRY(i2c_read(chip, 0x03, &stat));

	if (val == stat) {
		pr_info("wp502x get max charge current %d mA.\n", stat*100);
		return 0;
	}

	pr_info("wp502x get max charge current %d mA, try to set default %d mA.\n", stat*100, chip->dt.bat_cfg_fastchg_current);

	FAILURE_RETRY(i2c_write(chip, 0x08, 0xa5)); //Clear IIC REG Lock 1
	usleep_range(100000, 110000);
	FAILURE_RETRY(i2c_write(chip, 0x09, 0x5a)); //Clear IIC REG Lock 2
	usleep_range(100000, 110000);

	FAILURE_RETRY(i2c_write(chip, 0x03, val));
	FAILURE_RETRY(i2c_read(chip, 0x03, &stat));

	FAILURE_RETRY(i2c_write(chip, 0x08, 0x00)); //Clear IIC REG Lock 1
	usleep_range(100000, 110000);
	FAILURE_RETRY(i2c_write(chip, 0x09, 0x00)); //Clear IIC REG Lock 2
	usleep_range(100000, 110000);

	if(stat != val){
		pr_err("set max current failed\n");
		return -1;
	}
	return 0;
}

static int wp502x_charger_set_system_charge_current(struct wp502x_chip *chip, int batt_ma)
{
	unsigned char stat = 0;
	unsigned char val = 0;

	val = batt_ma/100;

	if((val > 80) || (val < 20))
		val = 50;

	FAILURE_RETRY(i2c_write(chip, 0x01, val));
	FAILURE_RETRY(i2c_read(chip, 0x01, &stat));

	if(stat != val){
		pr_err("set max current failed\n");
		return -1;
	}
	return 0;
}

static void wp502x_charger_get_battery_information(struct wp502x_chip *chip)
{
	union power_supply_propval val = {0, };

	if (chip->batt_psy == NULL)
		chip->batt_psy = power_supply_get_by_name("battery");

	if (chip->batt_psy) {
		power_supply_get_property(chip->batt_psy,
                    POWER_SUPPLY_PROP_TEMP,
                    &val);
		chip->batt_temp = val.intval;

		power_supply_get_property(chip->batt_psy,
                    POWER_SUPPLY_PROP_VOLTAGE_NOW,
                    &val);
		chip->batt_mv = val.intval/1000;
	}

#if defined(CONFIG_NEO_EXTERNAL_FG_SUPPORT)
	if (chip->ex_batt_psy == NULL)
	#if defined(CONFIG_NEO_BQ27520_BATTERY)
		chip->ex_batt_psy = power_supply_get_by_name("bq-battery");
	#else
		chip->ex_batt_psy = power_supply_get_by_name("battery");
	#endif

	if (chip->ex_batt_psy) {
		power_supply_get_property(chip->ex_batt_psy,
                    POWER_SUPPLY_PROP_CAPACITY,
                    &val);
		chip->batt_soc = val.intval;

		power_supply_get_property(chip->ex_batt_psy,
                    POWER_SUPPLY_PROP_CURRENT_NOW,
                    &val);
		chip->batt_ma = val.intval/1000;
	}
#else
	power_supply_get_property(chip->batt_psy,
                POWER_SUPPLY_PROP_CAPACITY,
                &val);
	chip->batt_soc = val.intval;

	power_supply_get_property(chip->batt_psy,
                POWER_SUPPLY_PROP_CURRENT_NOW,
                &val);
	chip->batt_ma = val.intval/1000;
#endif
}

static void wp502x_charger_get_usb_input_voltage(struct wp502x_chip *chip)
{
	union power_supply_propval val = {0,};
	int rc = -1;

	if (chip->usb_psy == NULL)
		chip->usb_psy = power_supply_get_by_name("usb");

	if (chip->usb_psy) {
		rc = power_supply_get_property(chip->usb_psy,
			 POWER_SUPPLY_PROP_VOLTAGE_NOW,
			 &val);
	}

	chip->usb_input_mv = val.intval/1000;
}

static int wp502x_charger_select_usbin_cofiguration(struct wp502x_chip *chip, u8 cfgs)
{
	u8 stat = 0;

	vote(chip->direct_chg_enable_votable, "DC_USBIN_VOTER", (bool)(cfgs == USBIN_DIRECT_CONFIGS), 0);

	usleep_range(10000, 11000);

	mutex_lock(&chip->control_lock);
	gpio_direction_output(chip->dt.cfg_en_gpio, (int)(cfgs == USBIN_NORMAL_CONFIGS));
	mutex_unlock(&chip->control_lock);

	usleep_range(10000, 11000);

	FAILURE_RETRY(i2c_write(chip, 0x00, cfgs));
	FAILURE_RETRY(i2c_read(chip, 0x00, &stat));

	if(stat != cfgs){
		pr_info("Retry to set deb register, stat:0x%x,cfgs:0x%x\n", stat, cfgs);
		FAILURE_RETRY(i2c_write(chip, 0x00, cfgs));
		return -1;
	}

	pr_info("Success to set usb cfgs:0x%x\n", cfgs);

	return 0;
}

static int wp502x_charger_registers_verify_workaround(struct wp502x_chip *chip)
{
	unsigned char stat6, stat7;
	unsigned char mohm = chip->dt.cfg_pcba_resistance;

	FAILURE_RETRY(i2c_read(chip, 0x06, &stat6));
	FAILURE_RETRY(i2c_read(chip, 0x07, &stat7));

	pr_info("Check 0x06 and 0x07 registers values:0x%x, 0x%x\n", stat6, stat7);

	/*****************************************************************************
	*   stat6 must be default pcba resistance , it shows PCBA impedance          *
	*   stat7 must be 0x8c, it shows that battery max float voltage is 4400mv    *
	******************************************************************************/
	if ((stat6 != mohm) || (stat7 != 0x8c)) {
		FAILURE_RETRY(i2c_write(chip, 0x08, 0xa5)); //Clear IIC REG Lock 1
		usleep_range(100000, 110000);
		FAILURE_RETRY(i2c_write(chip, 0x09, 0x5a)); //Clear IIC REG Lock 2
		usleep_range(100000, 110000);

		if (stat6 != mohm)
			FAILURE_RETRY(i2c_write(chip, 0x06, mohm));
		if (stat7 != 0x8c)
			FAILURE_RETRY(i2c_write(chip, 0x07, 0x8c));

		FAILURE_RETRY(i2c_write(chip, 0x08, 0x00)); //Clear IIC REG Lock 1
		usleep_range(100000, 110000);
		FAILURE_RETRY(i2c_write(chip, 0x09, 0x00)); //Clear IIC REG Lock 2
		usleep_range(100000, 110000);

		FAILURE_RETRY(i2c_read(chip, 0x06, &stat6));
		FAILURE_RETRY(i2c_read(chip, 0x07, &stat7));

		pr_info("Re-check 0x06 and 0x07 registers values:0x%02x, 0x%02x\n", stat6, stat7);
	}

	return 0;
}

#define MONITOR_VOTER		"MONITOR_VOTER"
static int wp502x_charger_try_to_enable_direct_charge(struct wp502x_chip *chip, int enable)
{
	if(enable){
		wp502x_charger_registers_verify_workaround(chip);
		wp502x_charger_select_usbin_cofiguration(chip, USBIN_IDETIFY_CONFIGS);
		/** Schedule control work now, in case something error happened.
		 ** wait 4500ms for NEO direct charge enable timeout.
		 **/
		cancel_delayed_work(&chip->monitor_work);
		schedule_delayed_work(&chip->monitor_work, msecs_to_jiffies(4500));
	}

	return 0;
}

#define USB_VALID_MIN_MV	    	3000
#define DIRECT_CHG_SUPPORT_MIN_MV	3800
#define MONITOR_WAIT_PER_MS     	10000
#define THERMAL_LEVEL_1_IBATT		3000
#define THERMAL_LEVEL_2_IBATT		2500
#define THERMAL_LEVEL_3_IBATT		2000
#define THERMAL_MONITOR_IBATT		2500
#define THERMAL_LEVEL_1_ENABLE		BIT(0)
#define THERMAL_LEVEL_2_ENABLE		BIT(1)
#define THERMAL_LEVEL_3_ENABLE		BIT(2)
#define THERMAL_MONITOR_ENABLE		BIT(7)
static void wp502x_charger_monitor_work(struct work_struct *work)
{
	u8 stat;
	struct wp502x_chip *chip = container_of(work, struct wp502x_chip, monitor_work.work);

	/** Check usb input voltage */
	wp502x_charger_get_usb_input_voltage(chip);
	if (chip->usb_input_mv < USB_VALID_MIN_MV) {
		pr_info("usb absent，cancel workqueue.\n");
		return;
	}

	FAILURE_RETRY_VOID(i2c_read(chip, 0x0A, &stat));
	wp502x_charger_get_battery_information(chip);
	chip->wp502x_irq = gpio_get_value(chip->dt.cfg_irq_gpio);
	chip->wp502x_enable = gpio_get_value(chip->dt.cfg_en_gpio);

	switch(stat){
		case USBIN_DIRECT_CONFIGS:
			/** wp502x interrupt, but the battery capacity more than threshold or battery charging current
				less than threshold or battery in un-normal degree. start to switching charge.
			*/
			if (!chip->wp502x_irq || chip->batt_soc > chip->dt.cfg_idetify_threshold
					|| chip->batt_ma > chip->dt.cfg_support_current_threshold
					|| chip->batt_temp < chip->dt.bat_cfg_clod_temp
					|| chip->batt_temp > chip->dt.bat_cfg_hot_temp) {
				pr_info("st:%d,soc:%d,t:%d,current:%d, switch to idetify mode.\n",
						chip->wp502x_irq, chip->batt_soc, chip->batt_temp, chip->batt_ma);
				wp502x_charger_select_usbin_cofiguration(chip, USBIN_IDETIFY_CONFIGS);
			}
			if (chip->batt_temp_limit_mask & THERMAL_MONITOR_ENABLE) {
				/** Decrease direct charging current to THERMAL_MONITOR_IBATT.*/
				if (!(chip->batt_temp_limit_mask & THERMAL_LEVEL_2_ENABLE)
							&& (chip->batt_temp >= chip->dt.bat_cfg_fcc_limited_temp)){
					chip->batt_temp_limit_mask |= THERMAL_LEVEL_2_ENABLE;
					pr_info("bat temp more than %d degree. decrease current.\n", chip->dt.bat_cfg_fcc_limited_temp);
					wp502x_charger_set_system_charge_current(chip, THERMAL_MONITOR_IBATT);
				}
				/** Battery temperature becomes normal. increase direct charging current to max support current.*/
				else if ((chip->batt_temp_limit_mask & THERMAL_LEVEL_2_ENABLE)
							&& (chip->batt_temp <= chip->dt.bat_cfg_fcc_limited_temp - 20)) {
					chip->batt_temp_limit_mask &= ~THERMAL_LEVEL_2_ENABLE;
					pr_info("bat temp less than %d degree. increase current.\n", chip->dt.bat_cfg_fcc_limited_temp - 20);
					wp502x_charger_set_system_charge_current(chip, chip->dt.bat_cfg_fastchg_current);
				}
			}
			else {
				/** Decrease direct charging current to THERMAL_LEVEL_1_IBATT.*/
				if (!(chip->batt_temp_limit_mask & THERMAL_LEVEL_1_ENABLE)
							&& (chip->batt_temp <= chip->dt.bat_cfg_cool_temp)){
					chip->batt_temp_limit_mask |= THERMAL_LEVEL_1_ENABLE;
					pr_info("bat temp less than %d degree. decrease current.\n", chip->dt.bat_cfg_cool_temp);
					wp502x_charger_set_system_charge_current(chip, THERMAL_LEVEL_1_IBATT);
				}
				/** Battery temperature becomes normal. increase direct charging current to max support current.*/
				else if((chip->batt_temp_limit_mask & THERMAL_LEVEL_1_ENABLE)
							&& (chip->batt_temp >= chip->dt.bat_cfg_cool_temp + 20)) {
					chip->batt_temp_limit_mask &= ~(THERMAL_LEVEL_1_ENABLE);
					pr_info("bat temp more than %d degree. increase current.\n", chip->dt.bat_cfg_cool_temp + 20);
					wp502x_charger_set_system_charge_current(chip, chip->dt.bat_cfg_fastchg_current);
				}
				/** Decrease direct charging current to THERMAL_LEVEL_2_IBATT.*/
				else if (!(chip->batt_temp_limit_mask & THERMAL_LEVEL_2_ENABLE)
							&& (chip->batt_temp >= chip->dt.bat_cfg_fcc_limited_temp)){
					chip->batt_temp_limit_mask |= THERMAL_LEVEL_2_ENABLE;
					pr_info("bat temp more than %d degree. decrease current.\n", chip->dt.bat_cfg_fcc_limited_temp);
					wp502x_charger_set_system_charge_current(chip, THERMAL_LEVEL_2_IBATT);
				}
				/** Decrease direct charging current to THERMAL_LEVEL_3_IBATT.*/
				else if (!(chip->batt_temp_limit_mask & THERMAL_LEVEL_3_ENABLE)
							&& (chip->batt_temp >= chip->dt.bat_cfg_fcc_limited_temp + 20)) {
					chip->batt_temp_limit_mask |= THERMAL_LEVEL_3_ENABLE;
					pr_info("bat temp more than %d degree. decrease current.\n", chip->dt.bat_cfg_fcc_limited_temp + 20);
					wp502x_charger_set_system_charge_current(chip, THERMAL_LEVEL_3_IBATT);
				}
				/** Battery temperature becomes normal. increase direct charging current to max support current.*/
				else if((chip->batt_temp_limit_mask & (THERMAL_LEVEL_2_ENABLE | THERMAL_LEVEL_3_ENABLE))
							&& (chip->batt_temp <= chip->dt.bat_cfg_fcc_limited_temp - 20)) {
					chip->batt_temp_limit_mask &= ~(THERMAL_LEVEL_2_ENABLE | THERMAL_LEVEL_3_ENABLE);
					pr_info("bat temp less than %d degree. increase current.\n", chip->dt.bat_cfg_fcc_limited_temp - 20);
					wp502x_charger_set_system_charge_current(chip, chip->dt.bat_cfg_fastchg_current);
				}
			}
			break;
		case USBIN_NORMAL_CONFIGS:
			if(!get_effective_result(chip->apsd_rerun_votable)) {
				wp502x_charger_select_usbin_cofiguration(chip, USBIN_NORMAL_CONFIGS);
				pr_info("irq:%d, en:%d, Switch to PMI charging and re-run APSD.\n", chip->wp502x_irq, chip->wp502x_enable);
				vote(chip->apsd_rerun_votable, "DC_USBIN_VOTER", true, 0);
			}
			break;
		case USBIN_IDETIFY_CONFIGS:
			if (chip->wp502x_irq && chip->batt_soc < chip->dt.cfg_idetify_threshold
					&& chip->batt_ma <= chip->dt.cfg_support_current_threshold
					&& chip->batt_mv >= DIRECT_CHG_SUPPORT_MIN_MV
					&& chip->batt_temp >= chip->dt.bat_cfg_clod_temp
					&& chip->batt_temp <= chip->dt.bat_cfg_hot_temp ) {
				pr_info("Try to recover to direct mode.\n");
				wp502x_charger_set_system_charge_current(chip, chip->dt.bat_cfg_fastchg_current);
				wp502x_charger_select_usbin_cofiguration(chip, USBIN_DIRECT_CONFIGS);
			}
			break;
		default:
			break;
	}

	pr_info("st:%d, en:%d, uv:%d, soc:%d, mV:%d, mA:%d, t:%d, p:0x%x\n",
		chip->wp502x_irq, chip->wp502x_enable, chip->usb_input_mv, chip->batt_soc, chip->batt_mv, chip->batt_ma, chip->batt_temp, stat);

	if (stat != USBIN_NORMAL_CONFIGS)
		schedule_delayed_work(&chip->monitor_work, msecs_to_jiffies(MONITOR_WAIT_PER_MS));

	return;
}

static irqreturn_t wp502x_charger_irq_handler(int irq, void *_chip)
{
    struct wp502x_chip *chip = _chip;
	chip->wp502x_irq = gpio_get_value(chip->dt.cfg_irq_gpio);
	chip->wp502x_enable = gpio_get_value(chip->dt.cfg_en_gpio);
	pr_info("wp502x charger irq handler:%d\n", chip->wp502x_irq);

	vote(chip->dc_awake_votable, MONITOR_VOTER, chip->wp502x_irq, 0);
	cancel_delayed_work(&chip->monitor_work);

	if(chip->wp502x_irq){
		/** Sehedule monitor_work in 1000ms */
		schedule_delayed_work(&chip->monitor_work, msecs_to_jiffies(1000));
	}
	else{
		/** Avoid revert boost back issue.*/		
		vote(chip->direct_chg_enable_votable, "DC_USBIN_VOTER", false, 0);
	}

	return IRQ_HANDLED;
}

static int wp502x_charger_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{	
    struct wp502x_chip *chip = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = chip->wp502x_irq;
		break;
	default:
		return -EINVAL;
	}
	
	return 0;
}

static int wp502x_charger_set_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     const union power_supply_propval *val)
{
	struct wp502x_chip *chip = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		wp502x_charger_try_to_enable_direct_charge(chip, val->intval);
		break;
	default:
		pr_debug("parallel power supply set prop %d not supported\n",
			prop);
		return -EINVAL;
	}

	return rc;
}

static int wp502x_charger_property_is_writeable(struct power_supply *psy,
					      enum power_supply_property prop)
{
	switch (prop) {
		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
			return 1;
		default:
			break;
	}

	return 0;
}

static int wp502x_charger_awake_vote_callback(struct votable *votable,
			void *data, int awake, const char *client)
{
	struct wp502x_chip *chip = data;

	if (awake)
		pm_stay_awake(chip->dev);
	else
		pm_relax(chip->dev);

	return 0;
}

#if defined(CONFIG_FB)
static int wp502x_charger_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	int *transition;
	struct fb_event *evdata = data;
	struct wp502x_chip *chip = container_of(self, struct wp502x_chip, fb_notifier);
	if ( evdata && evdata->data && chip ) {
		if ( event == FB_EVENT_BLANK ) {
			transition = evdata->data;
			if( *transition == FB_BLANK_POWERDOWN ){
				chip->batt_temp_limit_mask &= ~THERMAL_MONITOR_ENABLE;
			}
			else if ( *transition == FB_BLANK_UNBLANK ){
				chip->batt_temp_limit_mask |= THERMAL_MONITOR_ENABLE;
			}
			else {
				pr_info("Do nothing.\n");
			}
			pr_info("LCD:%d\n", chip->batt_temp_limit_mask);
		}
	}
	return 0;
}
#endif //CONFIG_FB

static int wp502x_charger_parse_dt(struct wp502x_chip *chip)
{
	int rc = 0;
	struct device_node *node = chip->dev->of_node;
	
	chip->dt.cfg_irq_gpio = of_get_named_gpio(node, "wp502x,fastchg-irq-gpio", 0);
	chip->dt.cfg_en_gpio = of_get_named_gpio(node, "wp502x,fastchg-en-gpio", 0);
	chip->dt.cfg_vref_1p8_en_gpio = of_get_named_gpio(node, "wp502x,vref1p8-en-gpio", 0);

	mutex_init(&chip->i2c_bus.io_lock);
	chip->i2c_bus.scl = of_get_named_gpio(node, "wp502x,i2c-scl-gpio", 0);
	rc = devm_gpio_request(chip->dev, chip->i2c_bus.scl, "wp502x_scl");
	if (rc) {
		pr_info("request wp502x_scl gpio failed, rc=%d\n", rc);
	}
	chip->i2c_bus.sda = of_get_named_gpio(node, "wp502x,i2c-sda-gpio", 0);
	rc = devm_gpio_request(chip->dev, chip->i2c_bus.sda, "wp502x_sda");
	if (rc) {
		pr_info("request wp502x_sda failed, rc=%d\n", rc);
	}

	gpio_direction_output(chip->i2c_bus.scl, 1);
	gpio_direction_output(chip->i2c_bus.sda, 1);

	chip->i2c_bus.udelay = 5;
	chip->i2c_bus.addr = 0x18;

	of_property_read_u32(node,
						 "wp502x,idetify-threshold",
						 &chip->dt.cfg_idetify_threshold);
	if (rc < 0)
		chip->dt.cfg_idetify_threshold = 80;

	rc = of_property_read_u32(node,
						 "wp502x,pcba-resistance",
						 &chip->dt.cfg_pcba_resistance);
	if (rc < 0)
		chip->dt.cfg_pcba_resistance = 10;

	of_property_read_u32(node,
						 "wp502x,support-current-threshold",
						 &chip->dt.cfg_support_current_threshold);
	if (rc < 0)
		chip->dt.cfg_support_current_threshold = -1200;

	of_property_read_u32(node,
						 "wp502x,bat-clod-limit-threshold",
						 &chip->dt.bat_cfg_clod_temp);
	if (rc < 0)
		chip->dt.bat_cfg_clod_temp = 100;

	of_property_read_u32(node,
						 "wp502x,bat-cool-limit-threshold",
						 &chip->dt.bat_cfg_cool_temp);
	if (rc < 0)
		chip->dt.bat_cfg_cool_temp = 150;
	
	of_property_read_u32(node,
						 "wp502x,bat-hot-limit-threshold",
						 &chip->dt.bat_cfg_hot_temp);
	if (rc < 0)
		chip->dt.bat_cfg_hot_temp = 440;

	of_property_read_u32(node,
						 "wp502x,bat-temp-limit-threshold",
						 &chip->dt.bat_cfg_fcc_limited_temp);
	if (rc < 0)
		chip->dt.bat_cfg_fcc_limited_temp = 400;

	of_property_read_u32(node,
						 "wp502x,fastchg-max-ma",
						 &chip->dt.bat_cfg_fastchg_current);
	if (rc < 0)
		chip->dt.bat_cfg_fastchg_current = 3200;

	pr_info("bat_cfg_cool_temp=%d bat_cfg_hot_temp=%d fast_max_chg_ma=%d, pcba_resistance=%d.\n",
		chip->dt.bat_cfg_cool_temp, chip->dt.bat_cfg_hot_temp, chip->dt.bat_cfg_fastchg_current, chip->dt.cfg_pcba_resistance);

	return rc;
}

/***********************************************************
*  for debug register read or write, path: sys/kernel/debug/wp502x
*  addr:   the register to read or write
*  data: 'echo x > data' to write the register  and  'cat data' to read the register
************************************************************/
static u8 base_addr = 0;
static int show_address(void *data, u64 *val)
{
	*val = base_addr;
	return 0;
}

static int store_address(void *data, u64 val)
{
	if(val < 0x08)
		base_addr = val;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(wp502x_set_fops, show_address, store_address, "0x%02llx\n");

static int show_data(void *data, u64 *val)
{
	struct wp502x_chip *chip = (struct wp502x_chip *)data;
	u8 value;

	FAILURE_RETRY(i2c_read(chip, base_addr, &value));

	*val = value;
	return 0;
}

static int store_data(void *data, u64 val)
{
	struct wp502x_chip *chip = (struct wp502x_chip *)data;
	u8 value = val;

	if(base_addr > 0x01){
		FAILURE_RETRY(i2c_write(chip, 0x08, 0xa5)); //Clear IIC REG Lock 1
		usleep_range(100000, 110000);
		FAILURE_RETRY(i2c_write(chip, 0x09, 0x5a)); //Clear IIC REG Lock 2
		usleep_range(100000, 110000);
	}

	FAILURE_RETRY(i2c_write(chip, base_addr, value));

	if(base_addr > 0x01){
		FAILURE_RETRY(i2c_write(chip, 0x08, 0x00)); //Clear IIC REG Lock 1
		usleep_range(100000, 110000);
		FAILURE_RETRY(i2c_write(chip, 0x09, 0x00)); //Clear IIC REG Lock 2
		usleep_range(100000, 110000);
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(wp502x_rw_fops, show_data, store_data, "0x%02llx\n");

static int store_mode(void *data, u64 val)
{
	struct wp502x_chip *chip = (struct wp502x_chip *)data;
	u8 mode = val;

	wp502x_charger_select_usbin_cofiguration(chip, mode);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(wp502x_mode_fops, NULL, store_mode, "0x%02llx\n");

static void wp502x_charger_create_debugfs_entries(struct wp502x_chip *chip)
{
	chip->debug_root = debugfs_create_dir("wp502x", NULL);

	if (IS_ERR(chip->debug_root)) {
		pr_err("wp502x couldnt create debugfs dir\n");
		return;
	}

	debugfs_create_file("mode", 0644, chip->debug_root, chip, &wp502x_mode_fops);
	debugfs_create_file("address", 0644, chip->debug_root, chip, &wp502x_set_fops);
	debugfs_create_file("data", 0644, chip->debug_root, chip, &wp502x_rw_fops);

	return;
}

static const struct power_supply_desc wp502x_psy_desc = {
	.name = "neo-charger",
	.type = POWER_SUUPLY_TYPE_NEOCHARGER,
	.properties = wp502x_main_props,
	.num_properties = ARRAY_SIZE(wp502x_main_props),
	.get_property = wp502x_charger_get_property,
	.set_property = wp502x_charger_set_property,
	.property_is_writeable = wp502x_charger_property_is_writeable,
};

static int wp502x_charger_init_power_supply(struct wp502x_chip *chip)
{
	struct power_supply_config main_cfg = {};
	int rc = 0;

	chip->batt_psy = power_supply_get_by_name("battery");
	if (!chip->batt_psy){
		pr_err("Couldn't get battery power supply.\n");
		rc = -EPROBE_DEFER;
		return rc;
	}

	chip->usb_psy = power_supply_get_by_name("usb");
	if (!chip->usb_psy){
		pr_err("Couldn't get usb power supply.\n");
		rc = -EPROBE_DEFER;
		return rc;
	}

	main_cfg.drv_data = chip;
	main_cfg.of_node = chip->dev->of_node;
	chip->main_psy = devm_power_supply_register(chip->dev,
						   &wp502x_psy_desc,
						   &main_cfg);
	if (IS_ERR(chip->main_psy)) {
		pr_err("Couldn't register battery power supply\n");
		return PTR_ERR(chip->main_psy);
	}
	return rc;
}

static int wp502x_charger_probe(struct platform_device *pdev)
{	
    int rc;
	struct wp502x_chip *chip;
	
	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&pdev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);

	chip->dc_awake_votable = create_votable("MONITOR_AWAKE", VOTE_SET_ANY,
					wp502x_charger_awake_vote_callback, chip);
	if (IS_ERR(chip->dc_awake_votable)) {
		rc = PTR_ERR(chip->dc_awake_votable);
		goto cleanup;
	}

	chip->apsd_rerun_votable = find_votable("APSD_RERUN");
	if (IS_ERR(chip->apsd_rerun_votable)) {
		dev_err(chip->dev, "Couldn't find votable.\n");
		rc = PTR_ERR(chip->apsd_rerun_votable);
		goto cleanup;
	}

	chip->direct_chg_enable_votable = find_votable("DIRECT_CHG_ENABLE");
	if (IS_ERR(chip->direct_chg_enable_votable)) {
		dev_err(chip->dev, "Couldn't find votable.\n");
		rc = PTR_ERR(chip->direct_chg_enable_votable);
		goto cleanup;
	}

	rc = wp502x_charger_init_power_supply(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't initialize power supply rc=%d\n", rc);
		goto cleanup;
	}

	mutex_init(&chip->control_lock);
	INIT_DELAYED_WORK(&chip->monitor_work, wp502x_charger_monitor_work);

	rc = wp502x_charger_parse_dt(chip);
	if(rc < 0)
		goto cleanup;

	wp502x_charger_create_debugfs_entries(chip);
	
	/** wp502x ldo enable pin, high level.*/
	if (gpio_is_valid(chip->dt.cfg_vref_1p8_en_gpio)) {
        rc = devm_gpio_request(chip->dev, chip->dt.cfg_vref_1p8_en_gpio, "vref_1p8_en");
		if (rc) {
			dev_err(chip->dev, "request vref_1p8_en gpio failed, rc=%d\n", rc);
			goto cleanup;
		}
		/** Configuration GPIO to default value */
		rc = gpio_direction_output(chip->dt.cfg_vref_1p8_en_gpio, 1);
		if (rc) {
			dev_err(chip->dev, "Set vref_1p8_en gpio output fail.\n");
		}
    }

	rc = devm_request_threaded_irq(chip->dev, gpio_to_irq(chip->dt.cfg_irq_gpio), 
			NULL, 
			wp502x_charger_irq_handler, 
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT, 
			"wp502x-irq",	
			chip);
	if (rc < 0){						
		dev_err(chip->dev, "Unable to request stage_irq: %d\n",rc);
		goto cleanup;
	}

#if defined(CONFIG_FB)
	chip->fb_notifier.notifier_call = wp502x_charger_fb_notifier_callback;
	rc = fb_register_client(&chip->fb_notifier);
			if (rc < 0)
				dev_err(chip->dev, "Failed to register fb notifier client\n");
#endif

	wp502x_charger_set_pcba_resistance(chip, chip->dt.cfg_pcba_resistance);
	wp502x_charger_set_max_charge_current(chip, chip->dt.bat_cfg_fastchg_current);
	wp502x_charger_registers_verify_workaround(chip);

	device_init_wakeup(chip->dev, true);

	pr_info("wp502x Probe Success.\n");
	return 0;
	
cleanup:
	if (chip->main_psy)
		power_supply_unregister(chip->main_psy);

	if (chip->dc_awake_votable)
		destroy_votable(chip->dc_awake_votable);

	platform_set_drvdata(pdev, NULL);
	return rc;
}

static int wp502x_charger_remove(struct platform_device *pdev)
{	
	struct wp502x_chip *chip = platform_get_drvdata(pdev);

	
	if(gpio_is_valid(chip->dt.cfg_vref_1p8_en_gpio))
		gpio_free(chip->dt.cfg_vref_1p8_en_gpio);

	kfree(chip);
	return 0;
}

static struct of_device_id wp502x_dt_match[] = {
	{
		.compatible     = "wp502x,charger",
	},
	{ },
};

static struct platform_driver wp502x_charger_driver = {	
	.probe	= wp502x_charger_probe,	
	.remove	= wp502x_charger_remove,	
	.driver	= {		
		.name	= "wp502x_charger_driver",		
		.owner	= THIS_MODULE,		
		.of_match_table = wp502x_dt_match,	
	},
};  

static int __init wp502x_charger_init(void) 
{    
	return platform_driver_register(&wp502x_charger_driver);	 
}

static void __exit wp502x_charger_exit(void)
{
	return platform_driver_unregister(&wp502x_charger_driver);
}

late_initcall(wp502x_charger_init);
module_exit(wp502x_charger_exit);

MODULE_AUTHOR("Leo.guo");
MODULE_DESCRIPTION("NUBIA wp502x charger driver");
MODULE_LICENSE("GPL");