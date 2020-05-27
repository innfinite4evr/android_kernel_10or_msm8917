/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) "Cw2015- %s: " fmt, __func__
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/qpnp/qpnp-adc.h>



#define CWFG_ENABLE_LOG 1 //CHANGE   Customer need to change this for enable/disable log
#define CWFG_I2C_BUSNUM 2 //CHANGE   Customer need to change this number according to the principle of hardware
#define DOUBLE_SERIES_BATTERY 0
/*
#define USB_CHARGING_FILE "/sys/class/power_supply/usb/online" // Chaman
#define DC_CHARGING_FILE "/sys/class/power_supply/ac/online"
*/
#define queue_delayed_work_time  8000

#define REG_VERSION             0x0
#define REG_VCELL               0x2
#define REG_SOC                 0x4
#define REG_RRT_ALERT           0x6
#define REG_CONFIG              0x8
#define REG_MODE                0xA
#define REG_VTEMPL              0xC
#define REG_VTEMPH              0xD
#define REG_BATINFO             0x10

#define MODE_SLEEP_MASK         (0x3<<6)
#define MODE_SLEEP              (0x3<<6)
#define MODE_NORMAL             (0x0<<6)
#define MODE_QUICK_START        (0x3<<4)
#define MODE_RESTART            (0xf<<0)

#define CONFIG_UPDATE_FLG       (0x1<<1)
#define ATHD                    (0x0<<3)        // ATHD = 0%

#define BATTERY_UP_MAX_CHANGE   420*1000            // The time for add capacity when charging
#define BATTERY_DOWN_MAX_CHANGE 120*1000
#define BATTERY_JUMP_TO_ZERO    30*1000
#define BATTERY_CAPACITY_ERROR  40*1000
#define BATTERY_CHARGING_ZERO   1800*1000

#define CHARGING_ON 1
#define NO_CHARGING 0

#define SIZE_BATINFO    64

/* The battery modeling version v2 */
static unsigned char config_info[SIZE_BATINFO] = {
	0x16, 0xF0, 0x69, 0x68, 0x65, 0x62, 0x60, 0x58,
	0x69, 0x51, 0x57, 0x5A, 0x51, 0x44, 0x3D, 0x33,
	0x2E, 0x25, 0x20, 0x1D, 0x26, 0x33, 0x43, 0x48,
	0x17, 0x5C, 0x0B, 0x85, 0x29, 0x49, 0x59, 0x65,
	0x7A, 0x78, 0x72, 0x77, 0x43, 0x1C, 0x5F, 0x39,
	0x18, 0x3A, 0x52, 0x87, 0x8F, 0x91, 0x94, 0x52,
	0x82, 0x8C, 0x92, 0x96, 0xFF, 0x93, 0xB3, 0xCB,
	0x2F, 0x7D, 0x72, 0xA5, 0xB5, 0xC1, 0x2C, 0xCB,
};

static struct power_supply *chrg_usb_psy;
static struct power_supply *chrg_ac_psy;
static struct timespec suspend_time_before;
static struct timespec after;
static int suspend_resume_mark = 0;


struct cw_battery {
	struct device		*dev;
	struct i2c_client *client;
	struct workqueue_struct *cwfg_workqueue;
	struct delayed_work battery_delay_work;
	struct regulator	*vdd;
	struct qpnp_vadc_chip	*vadc_dev;

	struct power_supply cw_bat;

	u8	chip;

	int charger_mode;
	int capacity;
	int voltage;
	int batt_curr;
	int batt_temp;
	int status;
	int time_to_empty;
	int change;
	int batt_fcc;
    //int alt;
};

int g_cw2015_capacity = 0;
int g_cw2015_vol = 0;

/*Define CW2015 iic read function*/
int _fg_read_byte(struct i2c_client *client, unsigned char reg, unsigned char *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		pr_err("i2c read byte fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*val = (u8)ret;

	return 0;

}
/*Define CW2015 iic write function*/
int _fg_write_byte(struct i2c_client *client, unsigned char reg, unsigned char const val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		pr_err("i2c write byte fail: can't write 0x%02X to reg 0x%02X\n",
				val, reg);
		return ret;
	}

	return 0;

}
/*Define CW2015 iic read word function*/
int _fg_read_word(struct i2c_client *client, unsigned char reg, unsigned char *val)
{
	s32 ret;

	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		pr_err("i2c read word fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*val = (u16)ret;

	return 0;

}

/*CW2015 update profile function, Often called during initialization*/
int cw_update_config_info(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val;
	int i;
	unsigned char reset_val;

	pr_err("\n");
	pr_err("[FGADC] test config_info = 0x%x\n",config_info[0]);


	// make sure no in sleep mode
	ret = _fg_read_byte(cw_bat->client, REG_MODE, &reg_val);
	if(ret < 0) {
		return ret;
	}

	reset_val = reg_val;
	if((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) {
		return -1;
	}

	// update new battery info
	for (i = 0; i < SIZE_BATINFO; i++) {
		ret = _fg_write_byte(cw_bat->client, REG_BATINFO + i, config_info[i]);
		if(ret < 0)
			return ret;
	}

	reg_val |= CONFIG_UPDATE_FLG;   // set UPDATE_FLAG
	reg_val &= 0x07;                // clear ATHD
	reg_val |= ATHD;                // set ATHD
	ret = _fg_write_byte(cw_bat->client, REG_CONFIG, reg_val);
	if(ret < 0)
		return ret;
	// read back and check
	ret = _fg_read_byte(cw_bat->client, REG_CONFIG, &reg_val);
	if(ret < 0) {
		return ret;
	}

	if (!(reg_val & CONFIG_UPDATE_FLG)) {
		pr_err("Error: The new config set fail\n");
		//return -1;
	}

	if ((reg_val & 0xf8) != ATHD) {
		pr_err("Error: The new ATHD set fail\n");
		//return -1;
	}

	// reset
	reset_val &= ~(MODE_RESTART);
	reg_val = reset_val | MODE_RESTART;
	ret = _fg_write_byte(cw_bat->client, REG_MODE, reg_val);
	if(ret < 0) return ret;

	msleep(10);

	ret = _fg_write_byte(cw_bat->client, REG_MODE, reset_val);
	if(ret < 0) return ret;

	pr_err("cw2015 update config success!\n");

	return 0;
}
/*CW2015 init function, Often called during initialization*/
static int cw_init(struct cw_battery *cw_bat)
{
	int ret;
	int i;
	unsigned char reg_val = MODE_SLEEP;

	if ((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) {
		reg_val = MODE_NORMAL;
		ret = _fg_write_byte(cw_bat->client, REG_MODE, reg_val);
		if (ret < 0)
			return ret;
	}

	ret = _fg_read_byte(cw_bat->client, REG_CONFIG, &reg_val);
	if (ret < 0)
    		return ret;

	if ((reg_val & 0xf8) != ATHD) {
		reg_val &= 0x07;    /* clear ATHD */
		reg_val |= ATHD;    /* set ATHD */
		ret = _fg_write_byte(cw_bat->client, REG_CONFIG, reg_val);
		if (ret < 0)
			return ret;
	}

	ret = _fg_read_byte(cw_bat->client, REG_CONFIG, &reg_val);
	if (ret < 0)
	return ret;

	if (!(reg_val & CONFIG_UPDATE_FLG)) {
		pr_err("update config flg is true, need update config\n");
		ret = cw_update_config_info(cw_bat);
		if (ret < 0) {
			pr_err("%s : update config fail\n", __func__);
			return ret;
		}
	} else {
			for(i = 0; i < SIZE_BATINFO; i++) {
				ret = _fg_read_byte(cw_bat->client, (REG_BATINFO + i), &reg_val);
				if (ret < 0)
	        				return ret;

				if (config_info[i] != reg_val)
					break;
			}
		if (i != SIZE_BATINFO) {
			pr_err("config didn't match, need update config\n");
			ret = cw_update_config_info(cw_bat);
			if (ret < 0){
				return ret;
			}
		}
	}

	msleep(10);
	for (i = 0; i < 30; i++) {
		ret = _fg_read_byte(cw_bat->client, REG_SOC, &reg_val);
		if (ret < 0)
			return ret;
		else if (reg_val <= 0x64)
			break;
		msleep(120);
	}

	if (i >= 30 ){
		reg_val = MODE_SLEEP;
		ret = _fg_write_byte(cw_bat->client, REG_MODE, reg_val);
		pr_err("cw2015 input unvalid power error, cw2015 join sleep mode\n");
		return -1;
	}

	pr_err("cw2015 init success!\n");
	return 0;
}

/*
*Functions:< check_chrg_usb_psy check_chrg_ac_psy get_chrg_psy get_charge_state >
*for Get Charger Status from outside
*/
static int check_chrg_usb_psy(struct device *dev, void *data)
{
        struct power_supply *psy = dev_get_drvdata(dev);

        if (psy->type == POWER_SUPPLY_TYPE_USB) {
                chrg_usb_psy = psy;
                return 1;
        }
        return 0;
}

static int check_chrg_ac_psy(struct device *dev, void *data)
{
        struct power_supply *psy = dev_get_drvdata(dev);

        if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
                chrg_ac_psy = psy;
                return 1;
        }
        return 0;
}

static void get_chrg_psy(void)
{
	if(!chrg_usb_psy)
		class_for_each_device(power_supply_class, NULL, NULL, check_chrg_usb_psy);
	if(!chrg_ac_psy)
		class_for_each_device(power_supply_class, NULL, NULL, check_chrg_ac_psy);
}

static int get_charge_state(void)
{
        union power_supply_propval val;
        int ret = -ENODEV;
		int usb_online = 0;
		int ac_online = 0;

        if (!chrg_usb_psy || !chrg_ac_psy)
                get_chrg_psy();

        if(chrg_usb_psy) {
            ret = chrg_usb_psy->get_property(chrg_usb_psy, POWER_SUPPLY_PROP_ONLINE, &val);
            if (!ret)
                usb_online = val.intval;
        }
		if(chrg_ac_psy) {
            ret = chrg_ac_psy->get_property(chrg_ac_psy, POWER_SUPPLY_PROP_ONLINE, &val);
            if (!ret)
                ac_online = val.intval;
		}
		if(!chrg_usb_psy){
			pr_debug("Usb online didn't find\n");
		}
		if(!chrg_ac_psy){
			pr_debug("Ac online didn't find\n");
		}
		pr_debug("ac_online = %d    usb_online = %d\n", ac_online, usb_online);
		if(ac_online || usb_online){
			return 1;
		}
        return 0;
}


static int cw_por(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reset_val;

	reset_val = MODE_SLEEP;
	ret = _fg_write_byte(cw_bat->client, REG_MODE, reset_val);
	if (ret < 0)
		return ret;
	reset_val = MODE_NORMAL;
	msleep(10);
	ret = _fg_write_byte(cw_bat->client, REG_MODE, reset_val);
	if (ret < 0)
		return ret;
	ret = cw_init(cw_bat);
	if (ret)
		return ret;
	return 0;
}

static int cw_get_capacity(struct cw_battery *cw_bat)
{
	int cw_capacity;
	int ret;
	unsigned char reg_val;
	int cap_decimal;

	static int reset_loop = 0;
	//static int charging_loop = 0;
	//static int discharging_loop = 0;
	//static int jump_flag = 0;
	static int charging_5_loop = 0;
	//int sleep_cap = 0;

	ret = _fg_read_byte(cw_bat->client, REG_SOC, &reg_val);
	if (ret < 0)
		return ret;

	cw_capacity = reg_val;

        /* Read the SOC decimal places reg */
	ret = _fg_read_byte(cw_bat->client, REG_SOC+1, &reg_val);
	if (ret < 0)
		return ret;
	cap_decimal = reg_val;

	if ((cw_capacity < 0) || (cw_capacity > 100)) {
		pr_err("Error:  cw_capacity = %d\n", cw_capacity);
		reset_loop++;
		if (reset_loop > (BATTERY_CAPACITY_ERROR / queue_delayed_work_time)){
			cw_por(cw_bat);
			reset_loop =0;
		}

		/*   cw_capacity Chaman change because
			I think customer didn't want to get error capacity.*/
		return cw_bat->capacity;
	}else {
		reset_loop =0;
	}

	if((cw_capacity == 0 &&  cap_decimal >= 190) ||
		(cw_capacity == 1 && cap_decimal <= 80)) {
		cw_capacity = cw_bat->capacity;
	}

	/* case 1 : aviod swing */
	if (((cw_bat->charger_mode > 0)
					&& (cw_capacity == (cw_bat->capacity - 1)))
					|| ((cw_bat->charger_mode == 0)
					&& (cw_capacity == (cw_bat->capacity + 1)))) {

		if (!(cw_capacity == 0 && cw_bat->capacity <= 2)) {
			cw_capacity = cw_bat->capacity;
		}
	}

	/*case 4 : avoid battery level is 0% when long time charging*/
	if((cw_bat->charger_mode > 0) &&(cw_capacity == 0))
	{
		charging_5_loop++;
		if (charging_5_loop > BATTERY_CHARGING_ZERO / queue_delayed_work_time) {
			cw_por(cw_bat);
			charging_5_loop = 0;
		}
	}else if(charging_5_loop != 0){
		charging_5_loop = 0;
	}
	if(suspend_resume_mark == 1)
		suspend_resume_mark = 0;
	return cw_capacity;
}

/*This function called when get voltage from cw2015*/
static int cw_get_voltage(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val[2];
	u16 value16, value16_1, value16_2, value16_3;
	int voltage;

	//	ret = _fg_read_word(cw_bat->client, REG_VCELL, reg_val);
	ret = _fg_read_byte(cw_bat->client, REG_VCELL, &reg_val[0]);
	if(ret < 0) {
		return ret;
	}
	ret = _fg_read_byte(cw_bat->client, REG_VCELL+1, &reg_val[1]);
	if(ret < 0) {
		return ret;
	}
	value16 = (reg_val[0] << 8) + reg_val[1];

	ret = _fg_read_byte(cw_bat->client, REG_VCELL, &reg_val[0]);
	if(ret < 0) {
		return ret;
	}
	ret = _fg_read_byte(cw_bat->client, REG_VCELL+1, &reg_val[1]);
	if(ret < 0) {
		return ret;
	}
	value16_1 = (reg_val[0] << 8) + reg_val[1];

	ret = _fg_read_byte(cw_bat->client, REG_VCELL, &reg_val[0]);
	if(ret < 0) {
		return ret;
	}
	ret = _fg_read_byte(cw_bat->client, REG_VCELL+1, &reg_val[1]);
	if(ret < 0) {
		return ret;
	}
	value16_2 = (reg_val[0] << 8) + reg_val[1];

	if(value16 > value16_1) {
		value16_3 = value16;
		value16 = value16_1;
		value16_1 = value16_3;
	}

	if(value16_1 > value16_2) {
		value16_3 =value16_1;
		value16_1 =value16_2;
		value16_2 =value16_3;
	}

	if(value16 >value16_1) {
		value16_3 =value16;
		value16 =value16_1;
		value16_1 =value16_3;
	}

	voltage = value16_1 * 305;

	if(DOUBLE_SERIES_BATTERY)
		voltage = voltage * 2;
	return voltage;
}

/*This function called when get RRT from cw2015*/
static int cw_get_time_to_empty(struct cw_battery *cw_bat)
{
	int ret;
   	unsigned char reg_val;
	u16 value16;

	ret = _fg_read_byte(cw_bat->client, REG_RRT_ALERT, &reg_val);
	if (ret < 0)
		return ret;

	value16 = reg_val;

	ret = _fg_read_byte(cw_bat->client, REG_RRT_ALERT + 1, &reg_val);
	if (ret < 0)
		return ret;

	value16 = ((value16 << 8) + reg_val) & 0x1fff;
	return value16;
}

#define SMB_VTG_MIN_UV		1800000
#define SMB_VTG_MAX_UV		1800000
static int fg_update_temperature(struct cw_battery *cw_bat)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	/* read battery ID */
	rc = qpnp_vadc_read(cw_bat->vadc_dev, P_MUX4_1_1, &results);
	if (rc) {
		pr_err("error reading batt id channel ,rc = %d\n", rc);
		return rc;
	}

	pr_debug("get_usb_temp_value %d, %lld\n", results.adc_code,
							results.physical);

	cw_bat->batt_temp = results.physical;
	pr_debug("The Battery temp =%d\n", cw_bat->batt_temp);

	return 0;

}

/*
int check_charging_state(const char *filename)
{
	struct file *fp;
	mm_segment_t fs;
	loff_t pos;
	int read_size = 8;
	int state = 0;
	char buf[read_size];
	int ret;

	cw_printk("\n");
	fp = filp_open(filename, O_RDONLY, 0644);
	if (IS_ERR(fp))
		return -1;
	fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	ret = vfs_read(fp, buf, read_size, &pos);
	if(ret < 0)
		return -1;

	filp_close(fp,NULL);
	set_fs(fs);

	state = buf[0] - '0';
	cw_printk(" filename = %s  state = %d \n", filename, state);
	return state;
}
*/ //Old function of get charger status

static void cw_update_charge_status(struct cw_battery *cw_bat)
{
/*
	int if_charging = 0;
	if(check_charging_state(USB_CHARGING_FILE) == 1
		|| check_charging_state(DC_CHARGING_FILE) == 1)
	{
		if_charging = CHARGING_ON;
	}else{
		if_charging = NO_CHARGING;
	}
	if(if_charging != cw_bat->charger_mode){
		cw_bat->charger_mode = if_charging;
	}
*/ //Old function of get charger status
	int cw_charger_mode;
	cw_charger_mode = get_charge_state();
	if(cw_bat->charger_mode != cw_charger_mode){
        cw_bat->charger_mode = cw_charger_mode;
		cw_bat->change = 1;
	}
}


static void cw_update_capacity(struct cw_battery *cw_bat)
{
	int cw_capacity;
	cw_capacity = cw_get_capacity(cw_bat);

	if ((cw_capacity >= 0) && (cw_capacity <= 100) && (cw_bat->capacity != cw_capacity)) {
		cw_bat->capacity = cw_capacity;
		cw_bat->change = 1;
    }
}


static void cw_update_vol(struct cw_battery *cw_bat)
{
	int ret;
	ret = cw_get_voltage(cw_bat);
	if ((ret >= 0) && (cw_bat->voltage != ret)) {
		cw_bat->voltage = ret;
		cw_bat->change = 1;
	}
}

static void cw_update_status(struct cw_battery *cw_bat)
{
	int status;

	cw_update_charge_status(cw_bat);
	if(cw_bat->charger_mode > 0) {
		if (cw_bat->capacity >= 100)
			status = POWER_SUPPLY_STATUS_FULL;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;
	} else {
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	if (cw_bat->status != status) {
		cw_bat->status = status;
		cw_bat->change = 1;
	}
}

static void cw_update_time_to_empty(struct cw_battery *cw_bat)
{
	int ret;
	ret = cw_get_time_to_empty(cw_bat);
	if ((ret >= 0) && (cw_bat->time_to_empty != ret)) {
		cw_bat->time_to_empty = ret;
		cw_bat->change = 1;
	}
}


static void cw_bat_work(struct work_struct *work)
{
	struct delayed_work *delay_work;
	struct cw_battery *cw_bat;
	/*Add for battery swap start*/
	int ret;
	unsigned char reg_val;
	int i = 0;
	/*Add for battery swap end*/

	delay_work = container_of(work, struct delayed_work, work);
	cw_bat = container_of(delay_work, struct cw_battery, battery_delay_work);

	/*Add for battery swap start*/
	ret = _fg_read_byte(cw_bat->client, REG_MODE, &reg_val);
	if(ret < 0){
		//battery is out
		//you can send new battery capacity vol here what you want set
		//for example
		cw_bat->capacity = 100;
		cw_bat->voltage = 4400;
		cw_bat->change = 1;
	}else{
		if((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP){
			for(i = 0; i < 5; i++){
				if(cw_por(cw_bat) == 0)
					break;
			}
		}
		cw_update_capacity(cw_bat);
		cw_update_vol(cw_bat);
		cw_update_charge_status(cw_bat);
		cw_update_status(cw_bat);
		fg_update_temperature(cw_bat);
		cw_update_time_to_empty(cw_bat);

	}
	/*
	cw_update_capacity(cw_bat);
	cw_update_vol(cw_bat);
	cw_update_charge_status(cw_bat);
	cw_update_status(cw_bat);
	cw_update_time_to_empty(cw_bat);
	*/
	/*Add for battery swap end*/
	pr_err("M=%d  S=%d  L=%d  V=%d  T=%d\n",
							cw_bat->charger_mode,
							cw_bat->status,
							cw_bat->capacity,
							cw_bat->voltage,
							cw_bat->batt_temp);

	if(suspend_resume_mark == 1)
		suspend_resume_mark = 0;

	if (cw_bat->change == 1){
		power_supply_changed(&cw_bat->cw_bat);
		cw_bat->change = 0;
	}

	g_cw2015_capacity = cw_bat->capacity;
	g_cw2015_vol = cw_bat->voltage;

	queue_delayed_work(cw_bat->cwfg_workqueue,
		&cw_bat->battery_delay_work, msecs_to_jiffies(queue_delayed_work_time));
}


static int cw_battery_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
	int ret = 0;

	struct cw_battery *cw_bat;
	cw_bat = container_of(psy, struct cw_battery, cw_bat);

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = cw_bat->capacity;
		break;

	case POWER_SUPPLY_PROP_STATUS:   //Chaman charger ic will give a real value
		cw_update_status(cw_bat);
		val->intval = cw_bat->status;
		break;

	case POWER_SUPPLY_PROP_HEALTH:   //Chaman charger ic will give a real value
		val->intval= POWER_SUPPLY_HEALTH_GOOD;
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = cw_bat->voltage <= 0 ? 0 : 1;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = cw_bat->voltage;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = cw_bat->batt_curr;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = cw_bat->batt_fcc * 1000;
		break;

	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		val->intval = cw_bat->time_to_empty;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:  //Chaman this value no need
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		val->intval = cw_bat->batt_temp*10;
		break;

	default:
		return -EINVAL;
	}
	return ret;
}

static enum power_supply_property cw_battery_properties[] = {
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_TEMP,
};

static void fg_external_power_changed(struct power_supply *psy)
{

}

static int fg_prop_is_writeable(struct power_supply *psy,
					   enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static int cw2015_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	 int loop = 0;
	struct cw_battery *cw_bat;
	pr_err("Now start probe\n");

	cw_bat = devm_kzalloc(&client->dev, sizeof(*cw_bat), GFP_KERNEL);
	if (!cw_bat) {
		pr_err("cw_bat create fail!\n");
		return -ENOMEM;
	}

	cw_bat->dev = &client->dev;
	cw_bat->chip = id->driver_data;

	cw_bat->client = client;
	cw_bat->capacity = 1;
	cw_bat->voltage = 0;
	cw_bat->status = 0;
	cw_bat->batt_temp = 2980;
	cw_bat->charger_mode = NO_CHARGING;
	cw_bat->change = 0;
	cw_bat->batt_fcc = -ENODATA;
	cw_bat->batt_curr = -ENODATA;

	i2c_set_clientdata(client, cw_bat);

	ret = cw_init(cw_bat);
	while ((loop++ < 2) && (ret != 0)) {
		msleep(200);
		ret = cw_init(cw_bat);
	}
	if (ret) {
		pr_err("cw2015 init fail!\n");
		return ret;
	}

	/* the batt therm */
	cw_bat->vadc_dev = qpnp_get_vadc(cw_bat->dev, "batt_therm");
	if (IS_ERR(cw_bat->vadc_dev)) {
		ret = PTR_ERR(cw_bat->vadc_dev);
		if (ret == -EPROBE_DEFER)
			pr_err("vadc not found - defer rc=%d\n", ret);
		else
			pr_err("vadc property missing, rc=%d\n", ret);

		return ret;
	}

	cw_bat->vdd = regulator_get(cw_bat->dev, "vdd");
	if (IS_ERR(cw_bat->vdd))
		pr_err("Regulator get failed vdd rc=%d\n", ret);

	if (regulator_count_voltages(cw_bat->vdd) > 0) {
		ret = regulator_set_voltage(cw_bat->vdd, SMB_VTG_MIN_UV,
					   SMB_VTG_MAX_UV);
		if (ret)
			pr_err("Regulator set_vtg failed vdd rc=%d\n", ret);
	}

	ret = regulator_enable(cw_bat->vdd);
	if (ret)
		pr_err("Regulator vdd enable failed rc=%d\n", ret);

	/* the batt therm end */

	cw_update_capacity(cw_bat);
	cw_update_vol(cw_bat);
	cw_update_charge_status(cw_bat);
	cw_update_status(cw_bat);
	fg_update_temperature(cw_bat);
	cw_update_time_to_empty(cw_bat);

	cw_bat->cw_bat.name = "bms";
	cw_bat->cw_bat.type = POWER_SUPPLY_TYPE_BMS;
	cw_bat->cw_bat.properties = cw_battery_properties;
	cw_bat->cw_bat.num_properties = ARRAY_SIZE(cw_battery_properties);
	cw_bat->cw_bat.get_property = cw_battery_get_property;
	cw_bat->cw_bat.external_power_changed = fg_external_power_changed;
	cw_bat->cw_bat.property_is_writeable = fg_prop_is_writeable;
	ret = power_supply_register(&client->dev, &cw_bat->cw_bat);
	if(ret < 0) {
		power_supply_unregister(&cw_bat->cw_bat);
		pr_err("Failed to register fg_psy:%d\n", ret);
		return ret;
	}

	cw_bat->cwfg_workqueue = create_singlethread_workqueue("cwfg_gauge");
	INIT_DELAYED_WORK(&cw_bat->battery_delay_work, cw_bat_work);
	queue_delayed_work(cw_bat->cwfg_workqueue,
		&cw_bat->battery_delay_work , msecs_to_jiffies(50));

	pr_err("cw2015 driver probe success!\n");

	return 0;
}

/*
static int cw2015_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	cw_printk("\n");
	strcpy(info->type, CWFG_NAME);
	return 0;
}
*/

static int cw_bat_suspend(struct device *dev)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct cw_battery *cw_bat = i2c_get_clientdata(client);
		read_persistent_clock(&suspend_time_before);
		cancel_delayed_work(&cw_bat->battery_delay_work);
        return 0;
}

static int cw_bat_resume(struct device *dev)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct cw_battery *cw_bat = i2c_get_clientdata(client);
		suspend_resume_mark = 1;
		read_persistent_clock(&after);
		after = timespec_sub(after, suspend_time_before);
        queue_delayed_work(cw_bat->cwfg_workqueue,
			&cw_bat->battery_delay_work, msecs_to_jiffies(2));
        return 0;
}

static const struct dev_pm_ops cw_bat_pm_ops = {
        .suspend  = cw_bat_suspend,
        .resume   = cw_bat_resume,
};

static int cw2015_remove(struct i2c_client *client)
{
	pr_err("\n");
	return 0;
}

static struct of_device_id cw_fg_match_table[] = {
	{.compatible = "cw,cw2015",},
	{},
};
MODULE_DEVICE_TABLE(of,cw_fg_match_table);

static const struct i2c_device_id cw_fg_id_table[] = {
	{"cw2015", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, cw_fg_id_table);

static struct i2c_driver cw2015_driver = {
	.driver 	  = {
		.name		= "cw2015",
		.owner		= THIS_MODULE,
		.of_match_table = cw_fg_match_table,
		.pm			= &cw_bat_pm_ops,
	},
	.probe		= cw2015_probe,
	.remove		= cw2015_remove,
	//.detect 	  = cw2015_detect,
	.id_table		= cw_fg_id_table,
};


module_i2c_driver(cw2015_driver);

MODULE_AUTHOR("Shannon Liu");
MODULE_DESCRIPTION("CW2015 FGADC Device Driver V4.0");
MODULE_LICENSE("GPL");
