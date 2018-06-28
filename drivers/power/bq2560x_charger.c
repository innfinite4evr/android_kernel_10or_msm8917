/*
 * BQ2415x battery charging driver
 *
 * Copyright (C) 2013 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#define pr_fmt(fmt)	"bq2415x: %s: " fmt, __func__

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/alarmtimer.h>

#include "bq2560x_charger.h"


#if 0
#undef pr_debug
#define pr_debug pr_err
#undef pr_info
#define pr_info pr_err
#undef dev_dbg
#define dev_dbg dev_err
#else
#undef pr_info
#define pr_info pr_debug
#endif

#define JEITA_HOT_COLD_TIME	15
#define JEITA_WARM_COOL_TIME	30
#define JEITA_NORMAL_TIME	60

enum bq2415x_part_no {
	BQ24150 = 0x00,
	BQ24157 = 0x02,
};


enum {
	USER		= BIT(0),
	JEITA		= BIT(1),
	BATT_FC		= BIT(2),
	BATT_PRES	= BIT(3),
};


enum wakeup_src {
	WAKEUP_SRC_MONITOR = 0,
	WAKEUP_SRC_JEITA,
	WAKEUP_SRC_MAX,
};


#define WAKEUP_SRC_MASK (~(~0 << WAKEUP_SRC_MAX))
struct bq2415x_wakeup_source {
	struct wakeup_source source;
	unsigned long enabled_bitmap;
	spinlock_t ws_lock;
};


struct bq2415x_otg_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};


struct bq2415x {
	struct device *dev;
	struct i2c_client *client;

	enum bq2415x_part_no part_no;
	int	revision;

	int gpio_ce;

	int	vbus_type;

	int status;

	struct mutex data_lock;
	struct mutex i2c_rw_lock;
	struct mutex profile_change_lock;
	struct mutex charging_disable_lock;
	struct mutex irq_complete;

	struct bq2415x_wakeup_source bq2415x_ws;

	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;

	bool batt_present;
	bool usb_present;

	bool batt_full;

	bool charge_enabled;/* Register bit status */
	bool otg_enabled;
	bool batfet_enabled;
	bool in_hiz;

	bool vindpm_triggered;
	bool iindpm_triggered;

	bool in_therm_regulation;
	bool in_vsys_regulation;

	bool power_good;
	bool vbus_good;

	bool topoff_active;
	bool acov_triggered;

	/* if use software jeita in case of NTC is connected to gauge */
	bool software_jeita_supported;
	bool jeita_active;

	bool batt_hot;
	bool batt_cold;
	bool batt_warm;
	bool batt_cool;

	int batt_hot_degc;
	int batt_warm_degc;
	int batt_cool_degc;
	int batt_cold_degc;
	int hot_temp_hysteresis;
	int cold_temp_hysteresis;

	int batt_cool_ma;
	int batt_warm_ma;
	int batt_cool_mv;
	int batt_warm_mv;


	int batt_temp;

	int jeita_ma;
	int jeita_mv;

	int usb_psy_ma;
	int chg_resistor;

	unsigned int thermal_levels;
	unsigned int therm_lvl_sel;
	unsigned int *thermal_mitigation;

	int charge_state;
	int boost_state;
	int charging_disabled_status;

	int fault_status;

	int skip_writes;
	int skip_reads;
	int vbus_state;
	int vbus_irq;
	int irq;
	int irq_wake;
	u32 vbus_irq_flags;
	u32 led_ctrl_gpio;
	u32 led_ctrl_gpio_state;
	u32 led_ctrl_gpio_flag;

	struct bq2415x_platform_data* platform_data;

	struct delayed_work discharge_jeita_work; //normal no charge mode
	struct delayed_work charge_jeita_work; //charge mode jeita work
	//struct delayed_work monitor_work;

	struct alarm jeita_alarm;

	struct dentry *debug_root;

	struct bq2415x_otg_regulator otg_vreg;

	struct power_supply *usb_psy;
	struct power_supply *bms_psy;
	struct power_supply batt_psy;
};

static int BatteryTestStatus_enable = 0;

static void bq2415x_dump_status(struct bq2415x* bq);

static int __bq2415x_read_reg(struct bq2415x* bq, u8 reg, u8 *data)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u8)ret;

	return 0;
}

static int __bq2415x_write_reg(struct bq2415x* bq, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(bq->client, reg, val);
	if (ret < 0) {
		pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
				val, reg, ret);
		return ret;
	}
	return 0;
}

static int bq2415x_read_byte(struct bq2415x *bq, u8 *data, u8 reg)
{
	int ret;

	if (bq->skip_reads) {
		*data = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2415x_read_reg(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq2415x_write_byte(struct bq2415x *bq, u8 reg, u8 data)
{
	int ret;

	if (bq->skip_writes) {
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2415x_write_reg(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	if (ret) {
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
	}

	return ret;
}
static int bq2415x_update_bits(struct bq2415x *bq, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;


	if (bq->skip_reads || bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2415x_read_reg(bq, reg, &tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __bq2415x_write_reg(bq, reg, tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
	}

out:
	mutex_unlock(&bq->i2c_rw_lock);
	return ret;
}

static void bq2415x_stay_awake(struct bq2415x_wakeup_source *source,
	enum wakeup_src wk_src)
{
	unsigned long flags;

	spin_lock_irqsave(&source->ws_lock, flags);

	if (!__test_and_set_bit(wk_src, &source->enabled_bitmap)) {
		__pm_stay_awake(&source->source);
		pr_debug("enabled source %s, wakeup_src %d\n",
			source->source.name, wk_src);
	}
	spin_unlock_irqrestore(&source->ws_lock, flags);
}

static void bq2415x_relax(struct bq2415x_wakeup_source *source,
	enum wakeup_src wk_src)
{
	unsigned long flags;

	spin_lock_irqsave(&source->ws_lock, flags);
	if (__test_and_clear_bit(wk_src, &source->enabled_bitmap) &&
		!(source->enabled_bitmap & WAKEUP_SRC_MASK)) {
		__pm_relax(&source->source);
		pr_debug("disabled source %s\n", source->source.name);
	}
	spin_unlock_irqrestore(&source->ws_lock, flags);

	pr_debug("relax source %s, wakeup_src %d\n",
		source->source.name, wk_src);
}

static void bq2415x_wakeup_src_init(struct bq2415x *bq)
{
	spin_lock_init(&bq->bq2415x_ws.ws_lock);
	wakeup_source_init(&bq->bq2415x_ws.source, "bq2415x");
}



static int bq2415x_adaptive_lowcharge(struct bq2415x *bq)
{
	u8 val = BQ2415X_DIS_LOWCHG << BQ2415X_SHIFT_LOWCHG;

	return bq2415x_update_bits(bq, BQ2415X_REG05_ADAPTIVE, BQ2415X_MASK_LOWCHG, val);
}


static int bq2415x_enable_otg(struct bq2415x *bq)
{
	int ret = 0;

#if BQ2415X_OTG_FUNC_EN
	u8 val;

	val = BQ2415X_BOOST_MODE << BQ2415X_SHIFT_OPA_MODE;

	ret = bq2415x_update_bits(bq, BQ2415X_REG01_CONTROL, BQ2415X_MASK_OPA_MODE, val);
#endif

	return ret;
}


static int bq2415x_disable_otg(struct bq2415x *bq)
{
	int ret = 0;

#if BQ2415X_OTG_FUNC_EN
	u8 val;

	val = BQ2415X_CHARGER_MODE << BQ2415X_SHIFT_OPA_MODE;

	ret = bq2415x_update_bits(bq, BQ2415X_REG01_CONTROL, BQ2415X_MASK_OPA_MODE, val);
#endif

	return ret;
}


static int bq2415x_enable_charger(struct bq2415x *bq)
{
	int ret = 0;
/*	u8 val;

	val = BQ2415X_ENABLE_CHARG << BQ2415X_SHIFT_CHARG;
	ret = bq2415x_update_bits(bq, BQ2415X_REG01_CONTROL, BQ2415X_MASK_CHARG, val);
*/
	gpio_set_value(bq->gpio_ce, 0);
	bq->charge_enabled = 1;

	return ret;
}


static int bq2415x_disable_charger(struct bq2415x *bq)
{
	int ret = 0;
/*	u8 val;

	val = BQ2415X_ENABLE_CHARG << BQ2415X_SHIFT_CHARG;
	ret = bq2415x_update_bits(bq, BQ2415X_REG01_CONTROL, BQ2415X_MASK_CHARG, val);
*/
	gpio_set_value(bq->gpio_ce, 1);
	bq->charge_enabled = 0;

	return ret;
}


static int bq2415x_set_charge_current(struct bq2415x *bq, int mA)
{
	int val;

	val = (mA * bq->chg_resistor - 37400)/6800;
	if (val < 0)
		val = 0;
	else if (val > 7)
		val = 7;
	pr_err("the charge current is %dmA, (%d) \n", mA, val);
	val = val << BQ2415X_SHIFT_ICHRG;

	return bq2415x_update_bits(bq, BQ2415X_REG04_CURRENT, BQ2415X_MASK_ICHRG, val);
}

static int bq2415x_iin_limit(struct bq2415x *bq, int mA)
{
	int ret;
	u8 val;

/**    mA                LIMIT
 *  mA < 0         :      NO
 *   0 < mA <= 100 :     100
 * 100 < mA <= 500 :     500
 * 500 < mA <= 800 :     800
 *  mA > 800       :      NO
*/
	if (mA <= 0 || mA > 800) {
		val = BQ2415X_NO_IIN_LIMIT << BQ2415X_SHIFT_IIN_LIMIT;
		ret = bq2415x_update_bits(bq, BQ2415X_REG01_CONTROL, BQ2415X_MASK_IIN_LIMIT, val);
	}else {

		if (mA > 0 && mA <= 100) {
		val = BQ2415X_100_IIN_LIMIT << BQ2415X_SHIFT_IIN_LIMIT;
		ret = bq2415x_update_bits(bq, BQ2415X_REG01_CONTROL, BQ2415X_MASK_IIN_LIMIT, val);
		}

		if (mA > 100 && mA <= 500) {
		val = BQ2415X_500_IIN_LIMIT << BQ2415X_SHIFT_IIN_LIMIT;
		ret = bq2415x_update_bits(bq, BQ2415X_REG01_CONTROL, BQ2415X_MASK_IIN_LIMIT, val);
		}

		if (mA > 500) {
		val = BQ2415X_800_IIN_LIMIT << BQ2415X_SHIFT_IIN_LIMIT;
		ret = bq2415x_update_bits(bq, BQ2415X_REG01_CONTROL, BQ2415X_MASK_IIN_LIMIT, val);
		}
	}

	return ret;
}


int bq2415x_set_term_current(struct bq2415x *bq, int curr)
{
	u8 val = BQ2415X_100MA_ITERM << BQ2415X_SHIFT_ITERM;

	return bq2415x_update_bits(bq, BQ2415X_REG04_CURRENT, BQ2415X_MASK_ITERM, val);
}


static int bq2415x_set_charge_volt(struct bq2415x *bq, int mV)
{
	int val,ret;

	val=(mV -3500)/20;
	pr_info("the max charge volt is %dmV ,(%d)\n", mV, val);
	val = val << BQ2415X_SHIFT_VO;

	ret = bq2415x_update_bits(bq, BQ2415X_REG02_VOLTAGE, BQ2415X_MASK_VO, val);

	return ret;

}



/* the max charge current must first set */
static int bq2415x_set_max_charge_current(struct bq2415x *bq, int mA)
{
	int val,ret;
	u8 data;

	val=(mA * bq->chg_resistor -37400)/6800;
	pr_info("the max charge current is %dmA ,(%d) \n", mA, val);
	val = val << BQ2415X_SHIFT_IMAX_LIMIT;

	ret = bq2415x_update_bits(bq, BQ2415X_REG06_LIMIT, BQ2415X_MASK_IMAX_LIMIT, val);

	__bq2415x_read_reg(bq, 0x06,&data);
	pr_info("the 06 reg is %x\n",data);


	return ret;
}

/* the max charge volt must first set */
static int bq2415x_set_max_charge_volt(struct bq2415x *bq, int mV)
{
	int val,ret;
	u8 data;

	val=(mV -4200)/20;
	pr_info("the max charge volt is %dmV ,(%d)\n", mV, val);
	val = val << BQ2415X_SHIFT_VMAX_LIMIT;

	ret = bq2415x_update_bits(bq, BQ2415X_REG06_LIMIT, BQ2415X_MASK_VMAX_LIMIT, val);

	__bq2415x_read_reg(bq, 0x06,&data);
	pr_info(" the 06 reg is %x\n",data);

	return ret;
}


static void bq2415x_reset_chip(struct bq2415x *bq)
{
	int ret;
	u8 val;

	val = BQ2415X_CHG_RESET << BQ2415X_SHIFT_RESET;
	ret = bq2415x_update_bits(bq, BQ2415X_REG04_CURRENT, BQ2415X_MASK_RESET, val);
}
EXPORT_SYMBOL_GPL(bq2415x_reset_chip);


int bq2415x_enter_hiz_mode(struct bq2415x *bq)
{
	u8 val = BQ2415X_IN_HZ << BQ2415X_SHIFT_HZ;

	return bq2415x_update_bits(bq, BQ2415X_REG01_CONTROL, BQ2415X_MASK_HZ, val);

}
EXPORT_SYMBOL_GPL(bq2415x_enter_hiz_mode);


int bq2415x_exit_hiz_mode(struct bq2415x *bq)
{
	u8 val = BQ2415X_NOTIN_HZ << BQ2415X_SHIFT_HZ;

	return bq2415x_update_bits(bq, BQ2415X_REG01_CONTROL, BQ2415X_MASK_HZ, val);
}
EXPORT_SYMBOL_GPL(bq2415x_exit_hiz_mode);


int bq2415x_get_hiz_mode(struct bq2415x *bq, u8 *state)
{
	u8 val;
	int ret;

	ret = bq2415x_read_byte(bq, &val, BQ2415X_REG01_CONTROL);
	if (ret)
		return ret;
	*state = (val & BQ2415X_IN_HZ) >> BQ2415X_IN_HZ;

	return 0;
}
EXPORT_SYMBOL_GPL(bq2415x_get_hiz_mode);


static int bq2415x_enable_term(struct bq2415x* bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2415X_ENABLE_TERM << BQ2415X_SHIFT_TERM;
	else
		val = BQ2415X_DISABLE_TERM << BQ2415X_SHIFT_TERM;

	ret = bq2415x_update_bits(bq, BQ2415X_REG01_CONTROL, BQ2415X_MASK_TERM, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2415x_enable_term);

#if 0
static int bq2415x_set_stat_ctrl(struct bq2415x *bq, int ctrl)
{
	u8 val;

	val = ctrl;

	return bq2415x_update_bits(bq, bq2415x_REG_00, REG00_STAT_CTRL_MASK,
							val << REG00_STAT_CTRL_SHIFT);
}
#endif

static int bq2415x_charging_disable(struct bq2415x *bq, int reason,
								int disable)
{
	int ret = 0;
	int disabled;

	mutex_lock(&bq->charging_disable_lock);

	disabled = bq->charging_disabled_status;

	pr_info("reason=%d requested_disable=%d disabled_status=%d\n",
					reason, disable, disabled);

	if (disable == true)
		disabled |= reason;
	else
		disabled &= ~reason;

	if (disabled && bq->charge_enabled)
		ret = bq2415x_disable_charger(bq);
	else if (!disabled && !bq->charge_enabled)
		ret = bq2415x_enable_charger(bq);

	if (ret) {
		pr_err("Couldn't disable/enable charging for reason=%d ret=%d\n",
							ret, reason);
	} else {
		bq->charging_disabled_status = disabled;
		mutex_lock(&bq->data_lock);
		bq->charge_enabled = !disabled;
		mutex_unlock(&bq->data_lock);
	}
	mutex_unlock(&bq->charging_disable_lock);

	return ret;
}


static struct power_supply *get_bms_psy(struct bq2415x *bq)
{
	if (bq->bms_psy)
		return bq->bms_psy;
	bq->bms_psy = power_supply_get_by_name("bms");
	if (!bq->bms_psy)
		pr_debug("bms power supply not found\n");

	return bq->bms_psy;
}

static int bq2415x_get_batt_property(struct bq2415x *bq,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct power_supply *bms_psy = get_bms_psy(bq);

	int ret;

	if (!bms_psy)
		return -EINVAL;

	ret = bms_psy->get_property(bms_psy, psp, val);

	return ret;
}

static inline bool is_device_suspended(struct bq2415x *bq);
static int bq2415x_get_prop_charge_type(struct bq2415x *bq)
{
	u8 val = 0;

	bq2415x_read_byte(bq, &val, BQ2415X_REG00_STATUS);
	val &= BQ2415X_MASK_CHGSTAT;
	val >>= BQ2415X_SHIFT_CHGSTAT;
	switch (val) {
	case BQ2415X_CHARGING_CHGSTAT:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case BQ2415X_READY_CHGSTAT:
	case BQ2415X_DONE_CHGSTAT:
	case BQ2415X_FAULT_CHGSTAT:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	default:
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}
}

static int bq2415x_get_prop_batt_present(struct bq2415x *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret;

	ret = bq2415x_get_batt_property(bq,
				POWER_SUPPLY_PROP_PRESENT, &batt_prop);
	if (!ret)
		bq->batt_present = batt_prop.intval;

	return ret;

}

static int bq2415x_get_prop_batt_full(struct bq2415x *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret;

	ret = bq2415x_get_batt_property(bq,
					POWER_SUPPLY_PROP_STATUS, &batt_prop);
	if (!ret)
		bq->batt_full = (batt_prop.intval == POWER_SUPPLY_STATUS_FULL);

	return ret;
}

static int bq2415x_get_prop_charge_status(struct bq2415x *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret;
	u8 status;

	ret = bq2415x_read_byte(bq, &status, BQ2415X_REG00_STATUS);
	if (ret) {
		return	POWER_SUPPLY_STATUS_UNKNOWN;
	}

	mutex_lock(&bq->data_lock);
	bq->charge_state = (status & BQ2415X_MASK_CHGSTAT) >> BQ2415X_SHIFT_CHGSTAT;
	mutex_unlock(&bq->data_lock);

	ret = bq2415x_get_batt_property(bq,
					POWER_SUPPLY_PROP_STATUS, &batt_prop);
	if (!ret && batt_prop.intval == POWER_SUPPLY_STATUS_FULL
		&& bq->usb_present == true) {
		return POWER_SUPPLY_STATUS_FULL;
	}

	switch(bq->charge_state) {
		case BQ2415X_CHARGING_CHGSTAT:
			return POWER_SUPPLY_STATUS_CHARGING;
		case BQ2415X_DONE_CHGSTAT:
			return POWER_SUPPLY_STATUS_NOT_CHARGING;
		case BQ2415X_READY_CHGSTAT:
		case BQ2415X_FAULT_CHGSTAT:
			return POWER_SUPPLY_STATUS_DISCHARGING;
		default:
			return POWER_SUPPLY_STATUS_UNKNOWN;
	}

}

static int bq2415x_get_prop_health(struct bq2415x *bq)
{
	int ret;
	union power_supply_propval batt_prop = {0,};

	if (bq->software_jeita_supported) {
		if (bq->jeita_active) {
			if (bq->batt_hot)
				ret = POWER_SUPPLY_HEALTH_OVERHEAT;
			else if (bq->batt_warm)
				ret = POWER_SUPPLY_HEALTH_WARM;
			else if (bq->batt_cool)
				ret = POWER_SUPPLY_HEALTH_COOL;
			else if (bq->batt_cold)
				ret = POWER_SUPPLY_HEALTH_COLD;
		} else {
			ret = POWER_SUPPLY_HEALTH_GOOD;
		}
	} else {/* get health status from gauge */
		ret = bq2415x_get_batt_property(bq,
					POWER_SUPPLY_PROP_HEALTH, &batt_prop);
		if (!ret)
			ret = batt_prop.intval;
		else
			ret = POWER_SUPPLY_HEALTH_UNKNOWN;
	}
	return ret;
}


static enum power_supply_property bq2415x_charger_props[] = {

	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,

	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	//POWER_SUPPLY_PROP_TIME_TO_EMPTY,
	POWER_SUPPLY_PROP_CHARGE_FULL,

//	POWER_SUPPLY_PROP_CYCLE_COUNT,

	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_RESISTANCE_ID,
};

void static runin_work(struct bq2415x *bq, int batt_capacity)
{
#if 0
	int rc;

	printk("%s:BatteryTestStatus_enable = %d bq->usb_present = %d \n",
			__func__,BatteryTestStatus_enable,bq->usb_present);

	if (/*!bq->usb_present || */!BatteryTestStatus_enable) {
		if (bq->in_hiz) {
			rc = bq2415x_exit_hiz_mode(bq);
			if (rc) {
				dev_err(bq->dev, "Couldn't enable charge rc=%d\n", rc);
			} else {
				pr_err("Exit Hiz Successfully\n");
				bq->in_hiz = false;
			}
		}
		return;
	}

	if (batt_capacity >= 80) {
		pr_debug("bq2415x_get_prop_batt_capacity > 80\n");
		//rc = bq2415x_charging_disable(bq, USER, true);
		if (!bq->in_hiz) {
			rc = bq2415x_enter_hiz_mode(bq);
			if (rc) {
				dev_err(bq->dev, "Couldn't disenable charge rc=%d\n", rc);
			} else {
				pr_err("Enter Hiz Successfully\n");
				bq->in_hiz = true;
			}
		}
	} else if (batt_capacity < 60) {
		pr_debug("bq2415x_get_prop_batt_capacity < 60\n");
		//rc = bq2415x_charging_disable(bq, USER, false);
		if (bq->in_hiz) {
			rc = bq2415x_exit_hiz_mode(bq);
			if (rc) {
				dev_err(bq->dev, "Couldn't enable charge rc=%d\n", rc);
			} else {
				pr_err("Exit Hiz Successfully\n");
				bq->in_hiz = false;
			}
		}
	}
#endif
}

static int bq2415x_charger_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{

	struct bq2415x *bq = container_of(psy, struct bq2415x, batt_psy);
	//int ret;
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq2415x_get_prop_charge_type(bq);
		pr_debug("POWER_SUPPLY_PROP_CHARGE_TYPE:%d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = bq->charge_enabled;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = 3550;
		break;

	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq2415x_get_prop_charge_status(bq);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bq2415x_get_prop_health(bq);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		bq2415x_get_batt_property(bq, psp, val);
		runin_work(bq, val->intval);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		val->intval = bq->therm_lvl_sel;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_TECHNOLOGY:
	case POWER_SUPPLY_PROP_RESISTANCE_ID:
		return bq2415x_get_batt_property(bq, psp, val);
	default:
		return -EINVAL;

	}

	return 0;
}

static int bq2415x_system_temp_level_set(struct bq2415x *bq, int);

static int bq2415x_charger_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct bq2415x *bq = container_of(psy,
				struct bq2415x, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		bq2415x_charging_disable(bq, USER, !val->intval);

		power_supply_changed(&bq->batt_psy);
		power_supply_changed(bq->usb_psy);
		pr_info("POWER_SUPPLY_PROP_CHARGING_ENABLED: %s\n",
						val->intval ? "enable" : "disable");
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		bq2415x_system_temp_level_set(bq, val->intval);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bq2415x_charger_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static int bq2415x_update_charging_profile(struct bq2415x *bq)
{
	int ret;
	int chg_ma;
	int chg_mv;
	int icl;
	int therm_ma;

	union power_supply_propval prop = {0,};


	if (!bq->batt_present || !bq->usb_present)
		return 0;

	ret = bq->usb_psy->get_property(bq->usb_psy,
							POWER_SUPPLY_PROP_TYPE, &prop);

	if (ret < 0) {
		pr_err("couldn't read USB TYPE property, ret=%d\n", ret);
		return ret;
	}

	mutex_lock(&bq->profile_change_lock);
	if (bq->jeita_active) {
		chg_ma = bq->jeita_ma;
		chg_mv = bq->jeita_mv;
	} else {
		if (prop.intval == POWER_SUPPLY_TYPE_USB_DCP || prop.intval == POWER_SUPPLY_TYPE_USB_CDP) {
			chg_ma = bq->platform_data->atl.ichg;
			chg_mv = bq->platform_data->atl.vreg;
		} else {
			chg_ma = bq->platform_data->usb.ichg;
			chg_mv = bq->platform_data->usb.vreg;
		}
	}

	icl = bq->usb_psy_ma;
	if (bq->usb_psy_ma < chg_ma) {
		chg_ma = bq->usb_psy_ma;
	}

	if (bq->therm_lvl_sel > 0
			&& bq->therm_lvl_sel < (bq->thermal_levels - 1))
		/*
		 * consider thermal limit only when it is active and not at
		 * the highest level
		 */
		therm_ma = bq->thermal_mitigation[bq->therm_lvl_sel];
	else
		therm_ma = chg_ma;

	chg_ma = min(therm_ma, chg_ma);

	pr_err("volt=%d, curr=%d, usb curr limit=%d ,therm_ma =%d\n",
				chg_mv, chg_ma, icl, therm_ma);

	ret = bq2415x_iin_limit(bq, chg_ma);
	if (ret < 0)
		pr_err("Failed to set charge in_limit ret=%d\n", ret);

	ret = bq2415x_set_charge_volt(bq, chg_mv);
	if (ret < 0)
		pr_err("couldn't set charge voltage ret=%d\n", ret);

	ret = bq2415x_set_charge_current(bq, chg_ma);
	if (ret < 0)
		pr_err("couldn't set charge current, ret=%d\n", ret);

	ret = bq2415x_adaptive_lowcharge(bq);
	if (ret<0)
		pr_err("Failed to set adaptive, ret = %d\n", ret);

	if (bq->jeita_active && (bq->batt_hot || bq->batt_cold))
		bq2415x_charging_disable(bq, JEITA, true);
	else
		bq2415x_charging_disable(bq, JEITA, false);
	mutex_unlock(&bq->profile_change_lock);

	return 0;
}


static int bq2415x_system_temp_level_set(struct bq2415x *bq,
							int lvl_sel)
{
	int ret = 0;
	int prev_therm_lvl = 0;

	pr_err("%s lvl_sel=%d, bq->therm_lvl_sel = %d\n", __func__, lvl_sel, bq->therm_lvl_sel);
	if (BatteryTestStatus_enable)
		return 0;

	if (!bq->thermal_mitigation) {
		pr_err("Thermal mitigation not supported\n");
		return -EINVAL;
	}

	if (lvl_sel < 0) {
		pr_err("Unsupported level selected %d\n", lvl_sel);
		return -EINVAL;
	}

	if (lvl_sel >= bq->thermal_levels) {
		pr_err("Unsupported level selected %d forcing %d\n", lvl_sel,
				bq->thermal_levels - 1);
		lvl_sel = bq->thermal_levels - 1;
	}

	if (lvl_sel == bq->therm_lvl_sel)
		return 0;

//	mutex_lock(&bq->current_change_lock);
	prev_therm_lvl = bq->therm_lvl_sel;
	bq->therm_lvl_sel = lvl_sel;

	ret = bq2415x_update_charging_profile(bq);
	if (ret)
		pr_err("Couldn't update charging profile ret = %d\n", ret);

//	mutex_unlock(&bq->current_change_lock);
	return ret;
}


static void bq2415x_external_power_changed(struct power_supply *psy)
{
	struct bq2415x *bq = container_of(psy, struct bq2415x, batt_psy);

	union power_supply_propval prop = {0,};
	int ret, current_limit = 0;


	ret = bq->usb_psy->get_property(bq->usb_psy,
			POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (ret < 0)
		pr_err("could not read USB current_max property, ret=%d\n", ret);
	else
		current_limit = prop.intval / 1000;

	pr_info("current_limit = %d\n", current_limit);

	if (bq->usb_psy_ma != current_limit) {
		bq->usb_psy_ma = current_limit;
		bq2415x_update_charging_profile(bq);
	}

	ret = bq->usb_psy->get_property(bq->usb_psy,
					POWER_SUPPLY_PROP_ONLINE, &prop);
	if (ret < 0)
		pr_err("could not read USB ONLINE property, ret=%d\n", ret);
	else
		pr_info("usb online status =%d\n", prop.intval);

	ret = 0;
	if (bq->usb_present && bq->usb_psy_ma != 2) {//for pc sleep
		if (prop.intval == 0){
			pr_err("set usb online\n");
			ret = power_supply_set_online(bq->usb_psy, true);
		} else
			pr_info("the usb is already online\n");

	} else {
		if (prop.intval == 1) {
			pr_err("set usb offline\n");
			ret = power_supply_set_online(bq->usb_psy, false);
		} else
			pr_info("the usb is already offline\n");
	}

	if (ret < 0)
		pr_info("could not set usb online state, ret=%d\n", ret);

}


static int bq2415x_psy_register(struct bq2415x *bq)
{
	int ret;

	bq->batt_psy.name = "battery";
	bq->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY;
	bq->batt_psy.properties = bq2415x_charger_props;
	bq->batt_psy.num_properties = ARRAY_SIZE(bq2415x_charger_props);
	bq->batt_psy.get_property = bq2415x_charger_get_property;
	bq->batt_psy.set_property = bq2415x_charger_set_property;
	bq->batt_psy.external_power_changed = bq2415x_external_power_changed;
	bq->batt_psy.property_is_writeable = bq2415x_charger_is_writeable;

	ret = power_supply_register(bq->dev, &bq->batt_psy);
	if (ret < 0) {
		pr_err("failed to register batt_psy:%d\n", ret);
		return ret;
	}

	return 0;
}

static void bq2415x_psy_unregister(struct bq2415x *bq)
{
	power_supply_unregister(&bq->batt_psy);
}


static int bq2415x_otg_regulator_enable(struct regulator_dev *rdev)
{
	int ret;
	struct bq2415x *bq = rdev_get_drvdata(rdev);

	bq->otg_enabled = true;
	ret = bq2415x_enable_otg(bq);
	if (ret) {
		bq->otg_enabled = false;
		pr_err("Couldn't enable OTG mode ret=%d\n", ret);
	} else {
		pr_info("bq2415x OTG mode Enabled!\n");
	}

	return ret;
}


static int bq2415x_otg_regulator_disable(struct regulator_dev *rdev)
{
	int ret;
	struct bq2415x *bq = rdev_get_drvdata(rdev);

	bq->otg_enabled = false;
	ret = bq2415x_disable_otg(bq);
	if (ret) {
		bq->otg_enabled = true;
		pr_err("Couldn't disable OTG mode, ret=%d\n", ret);
	} else {
		pr_info("bq2415x OTG mode Disabled\n");
	}

	return ret;
}


static int bq2415x_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int ret;
	u8 status;
	u8 enabled;
	struct bq2415x *bq = rdev_get_drvdata(rdev);

	ret = bq2415x_read_byte(bq, &status, BQ2415X_REG01_CONTROL);
	if (ret)
		return ret;
	enabled = ((status & BQ2415X_MASK_OPA_MODE) >> BQ2415X_SHIFT_OPA_MODE);

	return (enabled == BQ2415X_BOOST_MODE) ? 1 : 0;

}


struct regulator_ops bq2415x_otg_reg_ops = {
	.enable		= bq2415x_otg_regulator_enable,
	.disable		= bq2415x_otg_regulator_disable,
	.is_enabled 	= bq2415x_otg_regulator_is_enable,
};

static int bq2415x_regulator_init(struct bq2415x *bq)
{
	int ret = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(bq->dev, bq->dev->of_node);
	if (!init_data) {
		dev_err(bq->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		bq->otg_vreg.rdesc.owner = THIS_MODULE;
		bq->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		bq->otg_vreg.rdesc.ops = &bq2415x_otg_reg_ops;
		bq->otg_vreg.rdesc.name = init_data->constraints.name;
		pr_info("regualtor name = %s\n", bq->otg_vreg.rdesc.name);

		cfg.dev = bq->dev;
		cfg.init_data = init_data;
		cfg.driver_data = bq;
		cfg.of_node = bq->dev->of_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		bq->otg_vreg.rdev = regulator_register(
					&bq->otg_vreg.rdesc, &cfg);
		if (IS_ERR(bq->otg_vreg.rdev)) {
			ret = PTR_ERR(bq->otg_vreg.rdev);
			bq->otg_vreg.rdev = NULL;
			if (ret != -EPROBE_DEFER)
				dev_err(bq->dev,
					"OTG reg failed, rc=%d\n", ret);
		}
	}

	return ret;
}



static int bq2415x_parse_jeita_dt(struct device *dev, struct bq2415x* bq)
{
	struct device_node *np = dev->of_node;
	int ret;

	ret = of_property_read_u32(np,"ti,bq2415x,jeita-hot-degc",
						&bq->batt_hot_degc);
	if(ret) {
		pr_err("Failed to read ti,bq2415x,jeita-hot-degc\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2415x,jeita-warm-degc",
						&bq->batt_warm_degc);
	if(ret) {
		pr_err("Failed to read ti,bq2415x,jeita-warm-degc\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2415x,jeita-cool-degc",
						&bq->batt_cool_degc);
	if(ret) {
		pr_err("Failed to read ti,bq2415x,jeita-cool-degc\n");
		return ret;
	}
	ret = of_property_read_u32(np,"ti,bq2415x,jeita-cold-degc",
						&bq->batt_cold_degc);
	if(ret) {
		pr_err("Failed to read ti,bq2415x,jeita-cold-degc\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2415x,jeita-hot-hysteresis",
						&bq->hot_temp_hysteresis);
	if(ret) {
		pr_err("Failed to read ti,bq2415x,jeita-hot-hysteresis\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2415x,jeita-cold-hysteresis",
						&bq->cold_temp_hysteresis);
	if(ret) {
		pr_err("Failed to read ti,bq2415x,jeita-cold-hysteresis\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2415x,jeita-cool-ma",
						&bq->batt_cool_ma);
	if(ret) {
		pr_err("Failed to read ti,bq2415x,jeita-cool-ma\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2415x,jeita-cool-mv",
						&bq->batt_cool_mv);
	if(ret) {
		pr_err("Failed to read ti,bq2415x,jeita-cool-mv\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2415x,jeita-warm-ma",
						&bq->batt_warm_ma);
	if(ret) {
		pr_err("Failed to read ti,bq2415x,jeita-warm-ma\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2415x,jeita-warm-mv",
						&bq->batt_warm_mv);
	if(ret) {
		pr_err("Failed to read ti,bq2415x,jeita-warm-mv\n");
		return ret;
	}

	bq->software_jeita_supported =
		of_property_read_bool(np,"ti,bq2415x,software-jeita-supported");

	return 0;
}


static struct bq2415x_platform_data* bq2415x_parse_dt(struct device *dev,
													struct bq2415x * bq)
{
	int ret;
	struct device_node *np = dev->of_node;
	struct bq2415x_platform_data* pdata;

	pdata = devm_kzalloc(dev, sizeof(struct bq2415x_platform_data),
						GFP_KERNEL);
	if (!pdata) {
		pr_err("Out of memory\n");
		return NULL;
	}

	ret = of_property_read_u32(np, "ti,bq2415x,chip-enable-gpio", &bq->gpio_ce);
	if(ret) {
		pr_err("Failed to read node of ti,bq2415x,chip-enable-gpio\n");
	}

	ret = of_property_read_u32(np,"ti,bq2415x,usb-vlim",&pdata->usb.vlim);
	if(ret) {
		pr_err("Failed to read node of ti,bq2415x,usb-vlim\n");
	}

	ret = of_property_read_u32(np,"ti,bq2415x,usb-ilim",&pdata->usb.ilim);
	if(ret) {
		pr_err("Failed to read node of ti,bq2415x,usb-ilim\n");
	}

	ret = of_property_read_u32(np,"ti,bq2415x,usb-vreg",&pdata->usb.vreg);
	if(ret) {
		pr_err("Failed to read node of ti,bq2415x,usb-vreg\n");
	}

	ret = of_property_read_u32(np,"ti,bq2415x,usb-ichg",&pdata->usb.ichg);
	if(ret) {
		pr_err("Failed to read node of ti,bq2415x,usb-ichg\n");
	}

	ret = of_property_read_u32(np,"ti,bq2415x,atl-vlim",&pdata->atl.vlim);
	if(ret) {
		pr_err("Failed to read node of ti,bq2415x,atl-vlim\n");
	}

	ret = of_property_read_u32(np,"ti,bq2415x,atl-ilim",&pdata->atl.ilim);
	if(ret) {
		pr_err("Failed to read node of ti,bq2415x,atl-ilim\n");
	}

	ret = of_property_read_u32(np,"ti,bq2415x,atl-vreg",&pdata->atl.vreg);
	if(ret) {
		pr_err("Failed to read node of ti,bq2415x,atl-vreg\n");
	}

	ret = of_property_read_u32(np,"ti,bq2415x,atl-ichg",&pdata->atl.ichg);
	if(ret) {
		pr_err("Failed to read node of ti,bq2415x,atl-ichg\n");
	}

	ret = of_property_read_u32(np,"ti,bq2415x,chg-resistor",&bq->chg_resistor);
	if(ret) {
		pr_err("Failed to read node of ti,bq2415x,chg-resistor\n");
	}

	ret = of_property_read_u32(np,"ti,bq2415x,stat-pin-ctrl",&pdata->statctrl);
	if(ret) {
		pr_err("Failed to read node of ti,bq2415x,stat-pin-ctrl\n");
	}

	ret = of_property_read_u32(np,"ti,bq2415x,precharge-current",&pdata->iprechg);
	if(ret) {
		pr_err("Failed to read node of ti,bq2415x,precharge-current\n");
	}

	ret = of_property_read_u32(np,"ti,bq2415x,termination-current",&pdata->iterm);
	if(ret) {
		pr_err("Failed to read node of ti,bq2415x,termination-current\n");
	}

	if (of_find_property(np, "qcom,thermal-mitigation",
					&bq->thermal_levels)) {
		bq->thermal_mitigation = devm_kzalloc(bq->dev,
					bq->thermal_levels,
						GFP_KERNEL);

		if (bq->thermal_mitigation == NULL) {
			pr_err("thermal mitigation kzalloc() failed.\n");
			//return -ENOMEM;
		}

		bq->thermal_levels /= sizeof(int);
		ret = of_property_read_u32_array(np,
				"qcom,thermal-mitigation",
				bq->thermal_mitigation, bq->thermal_levels);
		if (ret) {
			pr_err("Couldn't read thermal limits ret = %d\n", ret);
			//return ret;
		}
	}

	bq->vbus_irq = of_get_named_gpio_flags(np, "vbus,irq_gpio", 0, &bq->vbus_irq_flags);
	if(!bq->vbus_irq) {
		pr_err("Failed to read node of vbus,irq_gpio\n");
	}
	pr_err("the VBUS irq gpio  is %d\n",bq->vbus_irq);

	bq->led_ctrl_gpio = of_get_named_gpio_flags(np, "qcom,led_control", 0, &bq->led_ctrl_gpio_flag);
	if(!bq->led_ctrl_gpio) {
		pr_err("Failed to read node of led_control_gpio\n");
	}
	pr_err("gpio_leds_create_of led_control_gpio = %d\n", bq->led_ctrl_gpio);

	return pdata;
}


static void bq2415x_init_jeita(struct bq2415x *bq)
{

	bq->batt_temp = -EINVAL;

	/* set default value in case of dts read fail */
	bq->batt_hot_degc = 600;
	bq->batt_warm_degc = 450;
	bq->batt_cool_degc = 100;
	bq->batt_cold_degc = 0;

	bq->hot_temp_hysteresis = 50;
	bq->cold_temp_hysteresis = 50;

	bq->batt_cool_ma = 400;
	bq->batt_cool_mv = 4100;
	bq->batt_warm_ma = 400;
	bq->batt_warm_mv = 4100;

	bq->software_jeita_supported = true;

	/* DTS setting will overwrite above default value */
	pr_err("bq2415x_init_jeita\n");
	bq2415x_parse_jeita_dt(&bq->client->dev, bq);
}

static int bq2415x_init_device(struct bq2415x *bq)
{
	int ret;

	ret = bq2415x_set_max_charge_current(bq, bq->platform_data->atl.ilim);
	if (ret < 0)
		pr_err("couldn't set max charge current limit, ret=%d\n", ret);

	ret = bq2415x_set_max_charge_volt(bq, bq->platform_data->atl.vlim);
	if (ret < 0)
		pr_err("couldn't set max charge voltage limit, ret=%d\n", ret);

	ret = bq2415x_iin_limit(bq, 0);
	if (ret)
		pr_err("Failed to set stat in_limit, ret = %d\n",ret);

	ret = bq2415x_set_charge_volt(bq, bq->platform_data->atl.vreg);
	if (ret)
		pr_err("Failed to set charger volt, ret = %d\n",ret);

	ret = bq2415x_set_charge_current(bq, bq->platform_data->atl.ichg);
	if (ret)
		pr_err("Failed to set charger current, ret = %d\n",ret);

	ret = bq2415x_adaptive_lowcharge(bq);
	if (ret)
		pr_err("Failed to set adaptive, ret = %d\n",ret);

	ret = bq2415x_enable_charger(bq);
	if (ret)
		pr_err("Failed to set enable charger, ret = %d\n",ret);

	ret = bq2415x_enable_term(bq, true);
	if (ret)
		pr_err("Failed to set enable charger, ret = %d\n",ret);

	return 0;
}


static int bq2415x_detect_device(struct bq2415x* bq)
{
	int ret;
	u8 data;

	ret = bq2415x_read_byte(bq, &data, BQ2415X_REG03_VENDER);

	if(ret == 0){
		bq->part_no = (data & BQ2415X_MASK_PN) >> BQ2415X_SHIFT_PN;
		bq->revision = (data & BQ2415X_MASK_REVISION) >> BQ2415X_SHIFT_REVISION;
	}

	return ret;
}


static void bq2415x_check_jeita(struct bq2415x *bq)
{

	int ret;
	bool last_hot, last_warm, last_cool, last_cold;
	union power_supply_propval batt_prop = {0,};

	ret = bq2415x_get_batt_property(bq,
					POWER_SUPPLY_PROP_TEMP, &batt_prop);
	if (!ret)
		bq->batt_temp = batt_prop.intval;

	if (bq->batt_temp == -EINVAL)
		return;
	pr_debug("bq2415x_check_jeita\n");

	last_hot = bq->batt_hot;
	last_warm = bq->batt_warm;
	last_cool = bq->batt_cool;
	last_cold = bq->batt_cold;

	if (bq->batt_temp >= bq->batt_hot_degc) {/* HOT */
		if (!bq->batt_hot) {
			bq->batt_hot  = true;
			bq->batt_warm = false;
			bq->batt_cool = false;
			bq->batt_cold = false;
			bq->jeita_ma = 0;
			bq->jeita_mv = 0;
		}
	} else if (bq->batt_temp >= bq->batt_warm_degc) {/* WARM */
		if (!bq->batt_hot ||
				(bq->batt_temp < bq->batt_hot_degc - bq->hot_temp_hysteresis)){
			bq->batt_hot  = false;
			bq->batt_warm = true;
			bq->batt_cool = false;
			bq->batt_cold = false;
			bq->jeita_mv = bq->batt_warm_mv;
			bq->jeita_ma = bq->batt_warm_ma;
		}
	} else if (bq->batt_temp < bq->batt_cold_degc) {/* COLD */
		if (!bq->batt_cold) {
			bq->batt_hot  = false;
			bq->batt_warm = false;
			bq->batt_cool = false;
			bq->batt_cold = true;
			bq->jeita_ma = 0;
			bq->jeita_mv = 0;
		}
	} else if (bq->batt_temp < bq->batt_cool_degc) {/* COOL */
		if (!bq->batt_cold ||
				(bq->batt_temp > bq->batt_cold_degc + bq->cold_temp_hysteresis)) {
			bq->batt_hot  = false;
			bq->batt_warm = false;
			bq->batt_cool = true;
			bq->batt_cold = false;
			bq->jeita_mv = bq->batt_cool_mv;
			bq->jeita_ma = bq->batt_cool_ma;
		}
	} else {/* NORMAL */
		bq->batt_hot  = false;
		bq->batt_warm = false;
		bq->batt_cool = false;
		bq->batt_cold = false;
	}

	bq->jeita_active = bq->batt_cool || bq->batt_hot ||
					   bq->batt_cold || bq->batt_warm;

	if ((last_cold != bq->batt_cold) || (last_warm != bq->batt_warm) ||
		(last_cool != bq->batt_cool) || (last_hot != bq->batt_hot)) {
		bq2415x_update_charging_profile(bq);
		power_supply_changed(&bq->batt_psy);
		power_supply_changed(bq->usb_psy);
	} else if (bq->batt_hot || bq->batt_cold) { //continuely update event
		power_supply_changed(&bq->batt_psy);
		power_supply_changed(bq->usb_psy);
	}

}


static void bq2415x_check_batt_pres(struct bq2415x *bq)
{
	int ret = 0;
	bool last_batt_pres = bq->batt_present;

	ret = bq2415x_get_prop_batt_present(bq);
	if (!ret) {
		if (last_batt_pres != bq->batt_present) {
			ret = bq2415x_charging_disable(bq, BATT_PRES, !bq->batt_present);
			if (ret) {
				pr_err("failed to %s charging, ret = %d\n",
						bq->batt_full ? "disable" : "enable",
						ret);
			}
			power_supply_changed(&bq->batt_psy);
			power_supply_changed(bq->usb_psy);
		}
	}

}


static void bq2415x_check_batt_full(struct bq2415x *bq)
{
	int ret = 0;
	bool last_batt_full = bq->batt_full;

	ret = bq2415x_get_prop_batt_full(bq);
	if (!ret) {
		if (last_batt_full != bq->batt_full) {
			//ret = bq2415x_charging_disable(bq, BATT_FC, bq->batt_full);
			if (ret) {
				pr_err("failed to %s charging, ret = %d\n",
						bq->batt_full ? "disable" : "enable",
						ret);
			}
			power_supply_changed(&bq->batt_psy);
			power_supply_changed(bq->usb_psy);
		}
	}
}


static int calculate_jeita_poll_interval(struct bq2415x* bq)
{
	int interval;
	pr_debug("calculate_jeita_poll_interval\n");

	if (bq->batt_hot || bq->batt_cold)
		interval = JEITA_HOT_COLD_TIME;
	else if (bq->batt_warm || bq->batt_cool)
		interval = JEITA_WARM_COOL_TIME;
	else
		interval = JEITA_NORMAL_TIME;
	return interval;
}

static enum alarmtimer_restart bq2415x_jeita_alarm_cb(struct alarm *alarm,
							ktime_t now)
{
	struct bq2415x *bq = container_of(alarm,
							struct bq2415x, jeita_alarm);
	unsigned long ns;

	bq2415x_stay_awake(&bq->bq2415x_ws, WAKEUP_SRC_JEITA);
	schedule_delayed_work(&bq->charge_jeita_work, HZ/2);
	pr_debug("bq2415x_jeita_alarm_cb\n");

	ns = calculate_jeita_poll_interval(bq) * 1000000000LL;
	alarm_forward_now(alarm, ns_to_ktime(ns));
	return ALARMTIMER_RESTART;
}

static void bq2415x_charge_jeita_workfunc(struct work_struct *work)
{
	struct bq2415x *bq = container_of(work,
							struct bq2415x, charge_jeita_work.work);

	bq2415x_check_batt_pres(bq);
	bq2415x_check_batt_full(bq);

	bq2415x_check_jeita(bq);
	bq2415x_dump_status(bq);
	bq2415x_relax(&bq->bq2415x_ws, WAKEUP_SRC_JEITA);
	pr_debug("jeita mv=%d  ma=%d temp=%d\n",
				bq->jeita_mv, bq->jeita_ma, bq->batt_temp);
}

static void bq2415x_discharge_jeita_workfunc(struct work_struct *work)
{
	struct bq2415x *bq = container_of(work,
							struct bq2415x, discharge_jeita_work.work);

	bq2415x_check_batt_pres(bq);
	bq2415x_check_batt_full(bq);

	bq2415x_check_jeita(bq);
	bq2415x_dump_status(bq);

	pr_debug("jeita mv=%d  ma=%d temp=%d\n",
				bq->jeita_mv, bq->jeita_ma, bq->batt_temp);

	schedule_delayed_work(&bq->discharge_jeita_work,
							calculate_jeita_poll_interval(bq) * HZ);
}

/*
static const unsigned char* charge_stat_str[] = {
	"Not Charging",
	"Precharging",
	"Fast Charging",
	"Charge Done",
};
*/

static void bq2415x_dump_status(struct bq2415x* bq)
{
	u8 data[7];

	bq2415x_read_byte(bq, &data[0], 0x00);
	bq2415x_read_byte(bq, &data[1], 0x01);
	bq2415x_read_byte(bq, &data[2], 0x02);
	bq2415x_read_byte(bq, &data[3], 0x03);
	bq2415x_read_byte(bq, &data[4], 0x04);
	bq2415x_read_byte(bq, &data[5], 0x05);
	bq2415x_read_byte(bq, &data[6], 0x06);

	pr_err("Reg [0]=%x,[1]=%x,[2]=%x,[3]=%x,[4]=%x,[5]=%x,[6]=%x\n",
		data[0],data[1],data[2],data[3],data[4],data[5],data[6]);

}


static void bq2415x_update_status(struct bq2415x *bq)
{
	u8 status;
	int ret;
	bool last_batt_pres = bq->batt_present;

	/* can only called here to detect state change */
	bq2415x_get_prop_batt_present(bq);
	if (last_batt_pres != bq->batt_present)
		bq2415x_charging_disable(bq, BATT_PRES, !bq->batt_present);

	ret = bq2415x_read_byte(bq, &status, BQ2415X_REG00_STATUS);
	if (ret) {
		pr_err("failed to read reg BQ2415X_REG00_STATUS");
	}
	mutex_lock(&bq->data_lock);
	bq->charge_state = (status & BQ2415X_MASK_CHGSTAT) >> BQ2415X_SHIFT_CHGSTAT;
	bq->fault_status =  (status & BQ2415X_MASK_FAULT);
	mutex_unlock(&bq->data_lock);

}

static irqreturn_t bq2415x_charger_interrupt(int irq, void *dev_id)
{

	struct bq2415x *bq = dev_id;
	int ret;
	u8 status;

	pr_debug("bq2415x interrupt trigged!\n");

	mutex_lock(&bq->irq_complete);
	ret = bq2415x_read_byte(bq, &status, BQ2415X_REG00_STATUS);
	if (ret) {
		pr_err("failed to read BQ2415X_REG00_STATUS!\n");
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}
	bq->fault_status =  (status & BQ2415X_MASK_FAULT);
	if (bq->fault_status > 0) {
		bq2415x_dump_status(bq);
		pr_err("bq2415x interrupt ,fault status %d!\n", bq->fault_status);
	}
	mutex_unlock(&bq->irq_complete);

	return IRQ_HANDLED;

}

static bool is_charger_mode = false;

extern void set_led_green(bool on);

static void set_charger_mode_led(bool on)
{
	set_led_green(on);
}

static int __init early_parse_charger_mode(char *cmdline)
{
	if ((cmdline) && !strncmp(cmdline, "charger", strlen("charger"))) {
		is_charger_mode = true;
	}

	return 0;
}
early_param("androidboot.mode", early_parse_charger_mode);

static irqreturn_t vbus_irq_handler(int irq, void *dev_id)
{
	struct bq2415x *bq = dev_id;
	int ret;

	pr_debug("bq2415x VBUS irq trigged!\n");

	if (is_charger_mode) {
		if (!gpio_get_value(bq->vbus_irq))
			set_charger_mode_led(false);
	}

	mutex_lock(&bq->irq_complete);
	bq->irq_waiting = true;
	if (!bq->resume_completed) {
		dev_err(bq->dev, "IRQ triggered before device-resume\n");
		if (!bq->irq_disabled) {
			disable_irq_nosync(irq);
			bq->irq_disabled = true;
		}
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}
	bq->irq_waiting = false;

#if BQ2415X_OTG_FUNC_EN
	if (bq->otg_enabled == 1) {
		pr_err("The state is in boost mode\n");
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}
#endif

	bq->vbus_state = gpio_get_value(bq->vbus_irq);

	if(bq->vbus_state){
		bq->usb_present = true;
		power_supply_set_present(bq->usb_psy, bq->usb_present);
		gpio_set_value(bq->led_ctrl_gpio, 1);

		cancel_delayed_work(&bq->discharge_jeita_work);

		if (bq->software_jeita_supported) {
			ret = alarm_start_relative(&bq->jeita_alarm,
		ns_to_ktime(calculate_jeita_poll_interval(bq) * 1000000000LL));

			if (ret)
				pr_err("start alarm for JEITA detection failed, ret=%d\n",
							ret);
		pr_err("VBUS irq trigged,usb present\n");
		}
	}else{
		bq->usb_present = false;
		power_supply_set_present(bq->usb_psy, bq->usb_present);
		gpio_set_value(bq->led_ctrl_gpio, 0);

		if (bq->software_jeita_supported)
			alarm_try_to_cancel(&bq->jeita_alarm);

		schedule_delayed_work(&bq->discharge_jeita_work,
					calculate_jeita_poll_interval(bq) * HZ);
		pr_err("VBUS irq trigged,usb not present\n");
	}

	bq->led_ctrl_gpio_state = gpio_get_value(bq->led_ctrl_gpio);
	pr_err("VBUS GPIO VALUE: vbus= %d , led_ctrl = %d\n",
					bq->vbus_state, bq->led_ctrl_gpio_state);
	bq2415x_update_status(bq);
	mutex_unlock(&bq->irq_complete);
	power_supply_changed(&bq->batt_psy);

	return IRQ_HANDLED;
}


static ssize_t bq2415x_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct bq2415x *bq = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[200];
	int len;
	int idx = 0;
	int ret ;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "bq2415x Reg");
	for (addr = 0x0; addr <= 0x0B; addr++) {
		ret = bq2415x_read_byte(bq, &val, addr);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,"Reg[0x%.2x] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t bq2415x_store_registers(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct bq2415x *bq = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg < 0x0B) {
		bq2415x_write_byte(bq, (unsigned char)reg, (unsigned char)val);
	}

	return count;
}

static ssize_t bq2415x_battery_test_status_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", BatteryTestStatus_enable);
}
static ssize_t bq2415x_battery_test_status_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;

	if (sscanf(buf, "%u", &input) != 1)
		retval = -EINVAL;
	else
			BatteryTestStatus_enable = input;

	pr_err("BatteryTestStatus_enable = %d\n", BatteryTestStatus_enable);

	return retval;
}
static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR, bq2415x_show_registers, bq2415x_store_registers);
static DEVICE_ATTR(BatteryTestStatus, S_IRUGO | S_IWUSR, bq2415x_battery_test_status_show, bq2415x_battery_test_status_store);

static struct attribute *bq2415x_attributes[] = {
	&dev_attr_registers.attr,
	&dev_attr_BatteryTestStatus.attr,
	NULL,
};

static const struct attribute_group bq2415x_attr_group = {
	.attrs = bq2415x_attributes,
};

static int show_registers(struct seq_file *m, void *data)
{
	struct bq2415x *bq = m->private;
	u8 addr;
	int ret;
	u8 val;

	for (addr = 0x0; addr <= 0x06; addr++) {
		ret = bq2415x_read_byte(bq, &val, addr);
		if (!ret)
			seq_printf(m, "Reg[0x%02X] = 0x%02X\n", addr, val);
	}
	return 0;
}


static int reg_debugfs_open(struct inode *inode, struct file *file)
{
	struct bq2415x *bq = inode->i_private;

	return single_open(file, show_registers, bq);
}


static const struct file_operations reg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= reg_debugfs_open,
	.read			= seq_read,
	.llseek		= seq_lseek,
	.release		= single_release,
};

static void create_debugfs_entry(struct bq2415x *bq)
{
	bq->debug_root = debugfs_create_dir("bq2415x", NULL);
	if (!bq->debug_root)
		pr_err("Failed to create debug dir\n");

	if (bq->debug_root) {

		debugfs_create_file("registers", S_IFREG | S_IRUGO,
						bq->debug_root, bq, &reg_debugfs_ops);

		debugfs_create_x32("charging_disable_status", S_IFREG | S_IRUGO,
						bq->debug_root, &(bq->charging_disabled_status));

		debugfs_create_x32("fault_status", S_IFREG | S_IRUGO,
						bq->debug_root, &(bq->fault_status));

		debugfs_create_x32("vbus_type", S_IFREG | S_IRUGO,
						bq->debug_root, &(bq->vbus_type));

		debugfs_create_x32("charge_state", S_IFREG | S_IRUGO,
						bq->debug_root, &(bq->charge_state));

		debugfs_create_x32("skip_reads",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  bq->debug_root,
					  &(bq->skip_reads));
		debugfs_create_x32("skip_writes",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  bq->debug_root,
					  &(bq->skip_writes));
	}
}

static int bq2415x_charger_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct bq2415x *bq;
	struct power_supply *usb_psy;
	struct power_supply *bms_psy;
	int ret;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_dbg(&client->dev, "USB supply not found, defer probe\n");
		return -EPROBE_DEFER;
	}

	bms_psy = power_supply_get_by_name("bms");
	if (!bms_psy) {
		dev_dbg(&client->dev, "bms supply not found, defer probe\n");
		return -EPROBE_DEFER;
	}

	bq = devm_kzalloc(&client->dev, sizeof(struct bq2415x), GFP_KERNEL);
	if (!bq) {
		pr_err("Out of memory\n");
		return -ENOMEM;
	}

	bq->dev = &client->dev;
	bq->usb_psy = usb_psy;
	bq->bms_psy = bms_psy;

	bq->client = client;
	i2c_set_clientdata(client, bq);

	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->data_lock);
	mutex_init(&bq->profile_change_lock);
	mutex_init(&bq->charging_disable_lock);
	mutex_init(&bq->irq_complete);

	bq->resume_completed = true;
	bq->irq_waiting = false;

	ret = bq2415x_detect_device(bq);
	if(ret) {
		pr_err("No bq2415x device found!\n");
		return -ENODEV;
	}

	bq2415x_init_jeita(bq);

	if (client->dev.of_node)
		bq->platform_data = bq2415x_parse_dt(&client->dev, bq);
	else
		bq->platform_data = client->dev.platform_data;

	if (!bq->platform_data) {
		pr_err("No platform data provided.\n");
		return -EINVAL;
	}
	/* the charger enable gpio config */
	if (gpio_is_valid(bq->gpio_ce)) {
		ret = devm_gpio_request(&client->dev, bq->gpio_ce, "bq2415x_ce");
		if (ret) {
			pr_err("Failed to request chip enable gpio %d:, err: %d\n", bq->gpio_ce, ret);
			return ret;
		}
		gpio_direction_output(bq->gpio_ce, 0);
	}

	/* the LED gpio config */
	if (gpio_is_valid(bq->led_ctrl_gpio))
		{
		ret = devm_gpio_request(&client->dev, bq->led_ctrl_gpio, "led_control_gpio");
		if (ret) {
			pr_err("Unable to request led control gpio ret=%d\n", ret);
		}
		else
		{
			pr_err("set led control gpio: %d to 1\n", bq->led_ctrl_gpio);
			ret = gpio_direction_output(bq->led_ctrl_gpio, 1);
		}
		pr_err("led_control_gpio: %d, value: %d\n", bq->led_ctrl_gpio, gpio_get_value(bq->led_ctrl_gpio));
	}

	ret = bq2415x_init_device(bq);
	if (ret) {
		pr_err("Failed to init device\n");
		return ret;
	}

	msleep(20);
	ret = bq2415x_psy_register(bq);
	if (ret)
		return ret;
	ret = bq2415x_regulator_init(bq);
	if (ret) {
		pr_err("Couldn't initialize bq2415x regulator ret=%d\n", ret);
		return ret;
	}

	INIT_DELAYED_WORK(&bq->charge_jeita_work, bq2415x_charge_jeita_workfunc);
	INIT_DELAYED_WORK(&bq->discharge_jeita_work, bq2415x_discharge_jeita_workfunc);

	alarm_init(&bq->jeita_alarm, ALARM_BOOTTIME, bq2415x_jeita_alarm_cb);

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
				bq2415x_charger_interrupt,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"bq2415x charger irq", bq);
		if (ret < 0) {
			pr_err("request irq for irq=%d failed, ret =%d\n", client->irq, ret);
			goto err_psy;
		}
		pr_err("Charger interrrupt request for irq=%d success, ret =%d\n", client->irq, ret);
		enable_irq_wake(client->irq);
	}

	/*interrupt config*/
		if (gpio_is_valid(bq->vbus_irq)) {
			ret = gpio_request(bq->vbus_irq, "vbus_irq");
			if (ret < 0) {
					pr_err("gpio_request fail ret =%d\n", ret);
					goto err_irq;
			}
			ret = gpio_direction_input(bq->vbus_irq);
			if (ret < 0) {
					pr_err("gpio_direction_input fail ret =%d\n", ret);
					goto err_irq;
			}

			bq->vbus_state = gpio_get_value(bq->vbus_irq);
			pr_err("VBUS gpios = %d, gpion = %d\n", bq->vbus_state, bq->vbus_irq );

			bq->irq = gpio_to_irq(bq->vbus_irq);
			pr_err("VBUS  irq = %d\n", bq->irq);

			ret= request_threaded_irq(bq->irq, NULL,
					vbus_irq_handler,
					IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
					"vbus_irq", bq);
			if (ret < 0) {
				pr_err("VBUS request_irq fail ret =%d\n", ret);
				goto err_irq;
			}
			irq_set_irq_wake(bq->irq,1);

		}
		else{
			pr_err("VBUS irq gpio not provided\n");
			goto err_psy;
		}

	bq2415x_wakeup_src_init(bq);

	device_init_wakeup(bq->dev, 1);
	create_debugfs_entry(bq);

	ret = sysfs_create_group(&bq->dev->kobj, &bq2415x_attr_group);
	if (ret) {
		dev_err(bq->dev, "failed to register sysfs. err: %d\n", ret);
	}

	vbus_irq_handler(bq->vbus_irq, bq);
	bq2415x_charger_interrupt(bq->irq, bq);
	bq2415x_dump_status(bq);

	pr_err("bq2415x probe successfully, Part Num:%d, Revision:%d\n!",
				bq->part_no, bq->revision);

	return 0;

err_psy:
	bq2415x_psy_unregister(bq);
err_irq:
	gpio_free(bq->vbus_irq);

	return ret;
}

static inline bool is_device_suspended(struct bq2415x *bq)
{
	return !bq->resume_completed;
}

static int bq2415x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2415x *bq = i2c_get_clientdata(client);

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = false;
	mutex_unlock(&bq->irq_complete);
	pr_err("Suspend successfully!\n");

	return 0;
}

static int bq2415x_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2415x *bq = i2c_get_clientdata(client);

	if (bq->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}


static int bq2415x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2415x *bq = i2c_get_clientdata(client);

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = true;
	if (bq->irq_waiting) {
		bq->irq_disabled = false;
		enable_irq(bq->irq);
		mutex_unlock(&bq->irq_complete);
		vbus_irq_handler(bq->vbus_irq, bq);
		//bq2415x_charger_interrupt(client->irq, bq);
	} else {
		mutex_unlock(&bq->irq_complete);
	}

	power_supply_changed(&bq->batt_psy);
	//pr_err("Resume successfully!");

	return 0;
}

static int bq2415x_charger_remove(struct i2c_client *client)
{
	struct bq2415x *bq = i2c_get_clientdata(client);

	alarm_try_to_cancel(&bq->jeita_alarm);

	cancel_delayed_work_sync(&bq->charge_jeita_work);
	cancel_delayed_work_sync(&bq->discharge_jeita_work);

	regulator_unregister(bq->otg_vreg.rdev);

	bq2415x_psy_unregister(bq);

	mutex_destroy(&bq->charging_disable_lock);
	mutex_destroy(&bq->profile_change_lock);
	mutex_destroy(&bq->data_lock);
	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->irq_complete);
	gpio_free(bq->vbus_irq);

	debugfs_remove_recursive(bq->debug_root);
	sysfs_remove_group(&bq->dev->kobj, &bq2415x_attr_group);

	return 0;

}

static void bq2415x_charger_shutdown(struct i2c_client *client)
{
	struct bq2415x *bq = i2c_get_clientdata(client);

	bq2415x_disable_otg(bq);
	bq2415x_disable_charger(bq);
	bq2415x_reset_chip(bq);

	pr_info("shutdown\n");
}

static struct of_device_id bq2415x_charger_match_table[] = {
	{.compatible = "ti,bq24150-charger",},
	{.compatible = "ti,bq24157-charger",},
	{},
};
MODULE_DEVICE_TABLE(of,bq2415x_charger_match_table);

static const struct i2c_device_id bq2415x_charger_id[] = {
	{ "bq24150-charger", BQ24150 },
	{ "bq24157-charger", BQ24157 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq2415x_charger_id);

static const struct dev_pm_ops bq2415x_pm_ops = {
	.resume 		= bq2415x_resume,
	.suspend_noirq = bq2415x_suspend_noirq,
	.suspend		= bq2415x_suspend,
};

static struct i2c_driver bq2415x_charger_driver = {
	.driver 	= {
		.name 		= "bq2415x-charger",
		.owner 		= THIS_MODULE,
		.of_match_table = bq2415x_charger_match_table,
		.pm		= &bq2415x_pm_ops,
	},
	.id_table	= bq2415x_charger_id,
	.probe		= bq2415x_charger_probe,
	.remove		= bq2415x_charger_remove,
	.shutdown	= bq2415x_charger_shutdown,
};

module_i2c_driver(bq2415x_charger_driver);

MODULE_DESCRIPTION("TI BQ2415x Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");
