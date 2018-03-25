/**
 * ChipSailing Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the ChipSailing fingerprint
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
 * Copyright (C) 2016 chipsailing Corporation. <http://www.chipsailing.com>
 * Copyright (C) 2016 XXX <mailto:xxx@chipsailing.com>
 *
 * This program is free software; you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by the Free 
 * Software Foundation; either version 2 of the License, or (at your option) 
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General 
 * Public License for more details.
 **/

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio.h>
#include <linux/of_irq.h>
#include <linux/input.h>
#include <linux/platform_device.h>
//#include <soc/qcom/scm.h>
#ifdef CONFIG_FB //system-defined Macro!!
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND  //system-defined Macro!!
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define FT_SUSPEND_LEVEL 1
#endif

#include "cf_ctl.h"

#define MODULE_NAME "cf_ctl"
#ifndef CONFIG_OF
# error "error: this driver 'MODULE_NAME' only support dts."
#endif

#define ANDROID_WAKELOCK 1
#if ANDROID_WAKELOCK
#include <linux/wakelock.h>
#endif

#define CF_RESET_LOW_US      10000
#define CF_RESET_HIGH1_US    1000
#define CF_RESET_HIGH2_US    1250
#define PWR_ON_STEP_SLEEP    100
#define PWR_ON_STEP_RANGE1   100
#define PWR_ON_STEP_RANGE2   900
#define CF_TTW_HOLD_TIME     1000
#define NUM_PARAMS_REG_ENABLE_SET 2

typedef enum {
    CF_PIN_STATE_RST_HIGH,
    CF_PIN_STATE_RST_LOW,
    CF_PIN_STATE_INT,
	
    /* Array size */
    CF_PIN_STATE_MAX
} cf_pin_state_t;

static const char * const pctl_names[] = {
	"fingerprint_reset_active",
	"fingerprint_reset_suspend",
	"fingerprint_irq",
};

struct vreg_config {
	char *name;
	unsigned long vmin;
	unsigned long vmax;
	int ua_load;
};

static const struct vreg_config const vreg_conf[] = {
	{ "vdd", 2800000UL, 2800000UL, 10, },
};


/**
 * Define the driver version string.
 * There is NO need to modify 'rXXXX_yyyymmdd', it should be updated automatically
 * by the building script (see the 'Driver-revision' section in 'build.sh').
 */
#define CF_DRV_VERSION "v1.9.1-rXXXX_20160502"
#define bufsiz (10240*2)

struct cf_device {
	struct device *dev;
	struct platform_device *spi;
	struct cdev     cdev;
	struct class*    class;
	struct device*   device;
	dev_t             devno;
	struct pinctrl *fingerprint_pinctrl;
	struct pinctrl_state *pinctrl_state[CF_PIN_STATE_MAX];
	struct clk *iface_clk;
	struct clk *core_clk;
	struct regulator *vreg[ARRAY_SIZE(vreg_conf)];
	struct input_dev *input;
	struct fasync_struct *async;

#ifdef CONFIG_FB
    struct notifier_block fb_notify;
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif

	struct wake_lock ttw_wl;
	int irq;
	int irq_gpio;
	int cs0_gpio;
	int cs1_gpio;
	int rst_gpio;
	int qup_id;
	struct mutex lock;
	spinlock_t spin_lock;
	bool prepared;
	atomic_t wakeup_enabled;
	bool clocks_enabled;
	bool clocks_suspended;
	u8 *buf;
    int irq_enabled;
};

/**************************debug******************************/
#define ERR_LOG  (0)
#define INFO_LOG (1)
#define DEBUG_LOG (2)

/* debug log setting */
u8 cf_debug_level = DEBUG_LOG;

#define cf_debug(level, fmt, args...) do { \
    if (cf_debug_level >= level) {\
        printk("[chipsailing]%5d:<%s>  "fmt,__LINE__,__func__,##args);\
    } \
} while (0)

#define FUNC_ENTRY()  cf_debug(DEBUG_LOG, "entry. \n")
#define FUNC_EXIT()  cf_debug(DEBUG_LOG, "exit. \n")

/*************************************************************/
//extern int cf_sfr_read(struct spi_device *spi, unsigned short addr, unsigned char *recv_buf, unsigned short buflen);
//extern int cf_sfr_write(struct spi_device *spi, unsigned short addr, unsigned char *send_buf, unsigned short buflen);
//extern int cf_sram_read(struct spi_device *spi, unsigned short addr, unsigned char *recv_buf, unsigned short buflen);
//extern int cf_sram_write(struct spi_device *spi, unsigned short addr, unsigned char *send_buf, unsigned short buflen);
//extern int cf_spi_cmd(struct spi_device *spi, unsigned char *cmd, unsigned short cmdlen);
//extern int cf_write_configs(struct spi_device *spi, struct param *p_param, int num);


static int vreg_setup(struct cf_device *cf_dev, const char *name,
	bool enable)
{
	size_t i;
	int rc;
	struct regulator *vreg;
	struct device *dev = cf_dev->dev;

	FUNC_ENTRY();

	for (i = 0; i < ARRAY_SIZE(cf_dev->vreg); i++) {
		const char *n = vreg_conf[i].name;
		if (!strncmp(n, name, strlen(n)))
			goto found;
	}
	dev_err(dev, "Regulator %s not found\n", name);
	return -EINVAL;
found:
	vreg = cf_dev->vreg[i];
	if (enable) {
		if (!vreg) {
			vreg = regulator_get(dev, name);
			if (IS_ERR(vreg)) {
				dev_err(dev, "Unable to get %s\n", name);
				return PTR_ERR(vreg);
			}
		}
		if (regulator_count_voltages(vreg) > 0) {
			rc = regulator_set_voltage(vreg, vreg_conf[i].vmin,
					vreg_conf[i].vmax);
			if (rc)
				dev_err(dev,
					"Unable to set voltage on %s, %d\n",
					name, rc);
		}
		rc = regulator_set_optimum_mode(vreg, vreg_conf[i].ua_load);
		if (rc < 0)
			dev_err(dev, "Unable to set current on %s, %d\n",
					name, rc);
		rc = regulator_enable(vreg);
		if (rc) {
			dev_err(dev, "error enabling %s: %d\n", name, rc);
			regulator_put(vreg);
			vreg = NULL;
		}
		cf_dev->vreg[i] = vreg;
	} else {
		if (vreg) {
			if (regulator_is_enabled(vreg)) {
				regulator_disable(vreg);
				dev_dbg(dev, "disabled %s\n", name);
			}
			regulator_put(vreg);
			cf_dev->vreg[i] = NULL;
		}
		rc = 0;
	}
	return rc;
}

/**
 * Prepare or unprepare the SPI master that we are soon to transfer something
 * over SPI.
 *
 * Please see Linux Kernel manual for SPI master methods for more information.
 *
 * @see Linux SPI master methods
 */
static int spi_set_fabric(struct cf_device *cf_dev, bool active)
{
    return 0;
}

/**
 * Changes ownership of SPI transfers from TEE to REE side or vice versa.
 *
 * SPI transfers can be owned only by one of TEE or REE side at any given time.
 * This can be changed dynamically if needed but of course that needs support
 * from underlaying layers. This function will transfer the ownership from REE
 * to TEE or vice versa.
 *
 * If REE side uses the SPI master when TEE owns the pipe or vice versa the
 * system will most likely crash dump.
 *
 * If available this should be set at boot time to eg. TEE side and not
 * dynamically as that will increase the security of the system. This however
 * implies that there are no other SPI slaves connected that should be handled
 * from REE side.
 *
 * @see SET_PIPE_OWNERSHIP
 */
static int set_pipe_ownership(struct cf_device *cf_dev, bool to_tz)
{
#ifdef SET_PIPE_OWNERSHIP
	int rc;
	const u32 TZ_BLSP_MODIFY_OWNERSHIP_ID = 3;
	const u32 TZBSP_APSS_ID = 1;
	const u32 TZBSP_TZ_ID = 3;
	struct scm_desc desc = {
		.arginfo = SCM_ARGS(2),
		.args[0] = cf_dev->qup_id,
		.args[1] = to_tz ? TZBSP_TZ_ID : TZBSP_APSS_ID,
	};

	FUNC_ENTRY();

	rc = scm_call2(SCM_SIP_FNID(SCM_SVC_TZ, TZ_BLSP_MODIFY_OWNERSHIP_ID),
		&desc);

	if (rc || desc.ret[0]) {
		dev_err(cf_dev->dev, "%s: scm_call2: responce %llu, rc %d\n",
				__func__, desc.ret[0], rc);
		return -EINVAL;
	}

#endif
	return 0;
}

static int set_clks(struct cf_device *cf_dev, bool enable)
{
   return 0;
}


static ssize_t clk_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct cf_device *cf_dev = dev_get_drvdata(dev);

	FUNC_ENTRY();

	rc = set_clks(cf_dev, (*buf == '1'));
	return rc ? rc : count;
}

static DEVICE_ATTR(clk_enable, S_IWUSR, NULL, clk_enable_set);

/**
 * Will try to select the set of pins (GPIOS) defined in a pin control node of
 * the device tree named @p name.
 *
 * The node can contain several eg. GPIOs that is controlled when selecting it.
 * The node may activate or deactivate the pins it contains, the action is
 * defined in the device tree node itself and not here. The states used
 * internally is fetched at probe time.
 *
 * @see pctl_names
 * @see cf_probe
 */
static int select_pin_ctl(struct cf_device *cf_dev, cf_pin_state_t state)
{
	int rc;
	struct device *dev = cf_dev->dev;
	
	FUNC_ENTRY();

	rc = pinctrl_select_state(cf_dev->fingerprint_pinctrl,
			cf_dev->pinctrl_state[state]);
	if (rc)
		dev_info(dev, "pinctrl_select_state(..) '%s' fail. \n", pctl_names[state]);
	else
		dev_info(dev, "pinctrl_select_state(..) '%s' pass. \n", pctl_names[state]);
	
	return rc;
}

/**
 * sysfs node handler to support dynamic change of SPI transfers' ownership
 * between TEE and REE side.
 *
 * An owner in this context is REE or TEE.
 *
 * @see set_pipe_ownership
 * @see SET_PIPE_OWNERSHIP
 */
static ssize_t spi_owner_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct cf_device *cf_dev = dev_get_drvdata(dev);
	int rc;
	bool to_tz;

	FUNC_ENTRY();

	if (!strncmp(buf, "tz", strlen("tz")))
		to_tz = true;
	else if (!strncmp(buf, "app", strlen("app")))
		to_tz = false;
	else
		return -EINVAL;

	rc = set_pipe_ownership(cf_dev, to_tz);
	return rc ? rc : count;
}
static DEVICE_ATTR(spi_owner, S_IWUSR, NULL, spi_owner_set);

static ssize_t pinctl_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct cf_device *cf_dev = dev_get_drvdata(dev);
	int rc = select_pin_ctl(cf_dev, (cf_pin_state_t)(*(int *)buf));

	FUNC_ENTRY();

	return rc ? rc : count;
	return 0;
}
static DEVICE_ATTR(pinctl_set, S_IWUSR, NULL, pinctl_set);

/**
 * Will indicate to the SPI driver that a message is soon to be delivered over
 * it.
 *
 * Exactly what fabric resources are requested is up to the SPI device driver.
 *
 * @see spi_set_fabric
 */
static ssize_t fabric_vote_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct cf_device *cf_dev = dev_get_drvdata(dev);
	int rc = spi_set_fabric(cf_dev, *buf == '1');

	FUNC_ENTRY();

	return rc ? rc : count;
}
static DEVICE_ATTR(fabric_vote, S_IWUSR, NULL, fabric_vote_set);

static ssize_t regulator_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct cf_device *cf_dev = dev_get_drvdata(dev);
	char op;
	char name[16];
	int rc;
	bool enable;

	FUNC_ENTRY();

	if (NUM_PARAMS_REG_ENABLE_SET != sscanf(buf, "%15[^,],%c", name, &op))
		return -EINVAL;
	if (op == 'e')
		enable = true;
	else if (op == 'd')
		enable = false;
	else
		return -EINVAL;
	rc = vreg_setup(cf_dev, name, enable);
	return rc ? rc : count;
}
static DEVICE_ATTR(regulator_enable, S_IWUSR, NULL, regulator_enable_set);

static ssize_t spi_bus_lock_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    return 0;
}
static DEVICE_ATTR(bus_lock, S_IWUSR, NULL, spi_bus_lock_set);

static int hw_reset(struct cf_device *cf_dev)
{
	int rst_gpio;
	int rc = select_pin_ctl(cf_dev, CF_PIN_STATE_RST_HIGH);
	if (rc)
		goto exit;
	usleep_range(CF_RESET_HIGH1_US, CF_RESET_HIGH1_US + 100);

	FUNC_ENTRY();

	rc = select_pin_ctl(cf_dev, CF_PIN_STATE_RST_LOW);
	if (rc)
		goto exit;
	usleep_range(CF_RESET_LOW_US, CF_RESET_LOW_US + 100);

	rc = select_pin_ctl(cf_dev, CF_PIN_STATE_RST_HIGH);
	if (rc)
		goto exit;
	usleep_range(CF_RESET_HIGH1_US, CF_RESET_HIGH1_US + 100);

	rst_gpio = gpio_get_value(cf_dev->rst_gpio);

exit:
	return rc;
}

static ssize_t hw_reset_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct cf_device *cf_dev = dev_get_drvdata(dev);

	if (!strncmp(buf, "reset", strlen("reset")))
		rc = hw_reset(cf_dev);
	else
		return -EINVAL;
	return rc ? rc : count;
}
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, hw_reset_set);

/**
 * Will setup clocks, GPIOs, and regulators to correctly initialize the touch
 * sensor to be ready for work.
 *
 * In the correct order according to the sensor spec this function will
 * enable/disable regulators, SPI platform clocks, and reset line, all to set
 * the sensor in a correct power on or off state "electrical" wise.
 *
 * @see  spi_prepare_set
 * @note This function will not send any commands to the sensor it will only
 *       control it "electrically".
 */
static int device_prepare(struct cf_device *cf_dev, bool enable)
{
    return 0;
/*
	int rc;

	mutex_lock(&cf_dev->lock);
	if (enable && !cf_dev->prepared) {
		spi_bus_lock(cf_dev->spi->master);
		cf_dev->prepared = true;
		select_pin_ctl(cf_dev, CF_PIN_STATE_RST_LOW);

		rc = vreg_setup(cf_dev, "vdd", true);
		if (rc)
			goto exit;


		usleep_range(PWR_ON_STEP_SLEEP, PWR_ON_STEP_RANGE2);

		rc = spi_set_fabric(cf_dev, true);
		if (rc)
			goto exit_3;


		rc = set_pipe_ownership(cf_dev, true);
		if (rc)
			goto exit_4;
	} else if (!enable && cf_dev->prepared) {
		rc = 0;
		(void)set_pipe_ownership(cf_dev, false);
exit_4:
		(void)spi_set_fabric(cf_dev, false);
exit_3:
		(void)select_pin_ctl(cf_dev, CF_PIN_STATE_RST_LOW);
		usleep_range(PWR_ON_STEP_SLEEP, PWR_ON_STEP_RANGE2);


//exit_1:
		(void)vreg_setup(cf_dev, "vdd", false);
exit:
		//(void)select_pin_ctl(cf_dev, CF_PIN_STATE_MI_GPIO);

		cf_dev->prepared = false;
		spi_bus_unlock(cf_dev->spi->master);
	} else {
		rc = 0;
	}
	mutex_unlock(&cf_dev->lock);
	return rc;*/
}

static ssize_t spi_prepare_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct cf_device *cf_dev = dev_get_drvdata(dev);

	FUNC_ENTRY();

	if (!strncmp(buf, "enable", strlen("enable")))
		rc = device_prepare(cf_dev, true);
	else if (!strncmp(buf, "disable", strlen("disable")))
		rc = device_prepare(cf_dev, false);
	else
		return -EINVAL;
	return rc ? rc : count;
}
static DEVICE_ATTR(device_prepare, S_IWUSR, NULL, spi_prepare_set);

/**
 * sysfs node for controlling whether the driver is allowed
 * to wake up the platform on interrupt.
 */
static ssize_t wakeup_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct cf_device *cf_dev = dev_get_drvdata(dev);

	FUNC_ENTRY();

	if (!strncmp(buf, "enable", strlen("enable"))) {
		atomic_set(&cf_dev->wakeup_enabled, 1);
	} else if (!strncmp(buf, "disable", strlen("disable"))) {
		atomic_set(&cf_dev->wakeup_enabled, 0);
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
	char *buffer)
{
	struct cf_device *cf_dev = dev_get_drvdata(device);
	int irq = gpio_get_value(cf_dev->irq_gpio);

	FUNC_ENTRY();

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
	struct cf_device *cf_dev = dev_get_drvdata(device);

	dev_dbg(cf_dev->dev, "%s\n", __func__);
	FUNC_ENTRY();

	return count;
}

static DEVICE_ATTR(irq, S_IRUSR | S_IWUSR, irq_get, irq_ack);

static struct attribute *attributes[] = {
	&dev_attr_pinctl_set.attr,
	&dev_attr_spi_owner.attr,
	&dev_attr_device_prepare.attr,
	&dev_attr_fabric_vote.attr,
	&dev_attr_regulator_enable.attr,
	&dev_attr_bus_lock.attr,
	&dev_attr_hw_reset.attr,
	&dev_attr_wakeup_enable.attr,
	&dev_attr_clk_enable.attr,
	&dev_attr_irq.attr,
	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

static void cf_disable_irq(struct cf_device *cf_dev)
{
	FUNC_ENTRY();

	if(cf_dev->irq_enabled == 1)
	{
		disable_irq_nosync(cf_dev->irq);
		cf_dev->irq_enabled = 0;
	}
}

static void cf_enable_irq(struct cf_device *cf_dev)
{
	FUNC_ENTRY();

	if(cf_dev->irq_enabled == 0)
	{
		enable_irq(cf_dev->irq);
		cf_dev->irq_enabled =1;
	}
}

static irqreturn_t cf_irq_handler(int irq, void * dev_id)
{
	struct cf_device *cf_dev =(struct cf_device *)dev_id;

	if (atomic_read(&cf_dev->wakeup_enabled)) {
		wake_lock_timeout(&cf_dev->ttw_wl,
					msecs_to_jiffies(CF_TTW_HOLD_TIME));
	}

	if (cf_dev->async) {
		kill_fasync(&cf_dev->async, SIGIO, POLL_IN);
	}

	return IRQ_HANDLED;
}

static int cf_request_named_gpio(struct cf_device *cf_dev,
	const char *label, int *gpio)
{
	int rc;
	struct device_node *np;
	struct device *dev = cf_dev->dev;

	FUNC_ENTRY();

    np = of_find_compatible_node(NULL, NULL, "csfinger,cs358");
    if (IS_ERR(np)) {
		dev_err(dev, "device node is not ready! \n");
		return PTR_ERR(np);
    }

	rc = of_get_named_gpio(np, label, 0);
	if (rc < 0) {
		dev_err(dev, "failed to get '%s'\n", label);
		return rc;
	}
	*gpio = rc;
	if (gpio_is_valid(*gpio)) {
		rc = devm_gpio_request(dev, *gpio, label);  //gpio_request(gpio, label)?
		if (rc) {
			dev_err(dev, "failed to request gpio %d\n", *gpio);
			return rc;
		}		
	} else {
		dev_err(dev, "not valid gpio: %d\n", *gpio);
		rc = -EIO;
		return rc;
	}

	return 0;
}

static int cf_gpio_init(struct cf_device *cf_dev)
{
	int rc;

	FUNC_ENTRY();

	rc = cf_request_named_gpio(cf_dev, "cdfinger,irq_gpio",
			&cf_dev->irq_gpio);

	rc = cf_request_named_gpio(cf_dev, "cdfinger,reset_gpio",
			&cf_dev->rst_gpio);
	if (rc)
		goto exit;

exit:
    return rc;		
}

static int cf_gpio_uninit(struct cf_device *cf_dev)
{
	int rc = 0;

	FUNC_ENTRY();

	if (gpio_is_valid(cf_dev->irq_gpio)) {
		gpio_free(cf_dev->irq_gpio);
	}

	if (gpio_is_valid(cf_dev->rst_gpio)) {
		gpio_free(cf_dev->rst_gpio);
	}

    return rc;		
}

static int cf_pinctrl_look_up_states(struct cf_device *cf_dev)
{
	int rc = 0;
	size_t i;
	struct device_node *np;
	struct platform_device *pdev;
	struct device *dev = cf_dev->dev;
	
	FUNC_ENTRY();

    np = of_find_compatible_node(NULL, NULL, "csfinger,cs358");
    if (IS_ERR(np)) {
		dev_err(dev, "device node is null. \n");
		rc = PTR_ERR(np);
		goto exit;
    }
	pdev = of_find_device_by_node(np);
	if (IS_ERR(pdev)) {
		dev_err(dev, "platform device is null. \n");
		rc = PTR_ERR(pdev);
		goto exit;
	}
	cf_dev->fingerprint_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(cf_dev->fingerprint_pinctrl)) {
		dev_err(dev, "Target does not use pinctrl. \n");
		cf_dev->fingerprint_pinctrl = NULL;
		rc = -EINVAL;
		goto exit;
	}

	for (i = 0; i < CF_PIN_STATE_MAX; i++) {
		const char *n = pctl_names[i];
		struct pinctrl_state *state =
			pinctrl_lookup_state(cf_dev->fingerprint_pinctrl, n);
		if (IS_ERR(state)) {
			dev_err(dev, "cannot find '%s'. \n", n);
			rc = -EINVAL;
			goto exit;
		}

		cf_dev->pinctrl_state[i] = state;
	}	
	
exit:
    return rc;
}

extern char *fingerprint_info;

static int cf_irq_init(struct cf_device *cf_dev)
{
	int rc;
	int irqf;
	struct device *dev = cf_dev->dev;
	struct device_node *np;
	u32 ints[2];

	FUNC_ENTRY();

    np = of_find_compatible_node(NULL, NULL, "csfinger,cs358");
    if (IS_ERR(np)) {
		dev_err(dev, "device node is null! \n");
		rc = PTR_ERR(np);
		goto exit;
    }

	irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
	if (of_property_read_bool(np, "fpc,enable-wakeup")) {  //dev->of_node
		irqf |= IRQF_NO_SUSPEND;
		device_init_wakeup(dev, 1);
	}

    select_pin_ctl(cf_dev, CF_PIN_STATE_INT);
	if (np) {
		of_property_read_u32_array(np, "debounce",ints,ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);
		cf_dev->irq_gpio = ints[0];
		cf_dev->irq = irq_of_parse_and_map(np, 0);   //dev->of_node
		if (!cf_dev->irq) {
			dev_err(dev, "irq_of_parse_and_map(..) fail. \n");
			rc = -EINVAL;
			goto exit;
		}
	} else {
		dev_err(dev, "device node is null. \n");
		rc = -ENODEV;
		goto exit;
	}
	
	rc = devm_request_threaded_irq(dev, cf_dev->irq, NULL, cf_irq_handler, irqf,
			"cf_irq", cf_dev);
	if (rc) {
		dev_err(dev, "could not request irq %d\n",
				cf_dev->irq);
		goto exit;
	}

	enable_irq_wake(gpio_to_irq(cf_dev->irq_gpio));	
	cf_dev->irq_enabled = 1;

	fingerprint_info = "csfinger";

exit:
	return rc;
}


static int cf_input_init(struct cf_device *cf_dev)
{
	struct device *dev = cf_dev->dev;
	int rc;

	FUNC_ENTRY();

	cf_dev->input = input_allocate_device();
	if (!cf_dev->input) {
		dev_err(dev ,"input_allocate_device(..) fail. \n");
		return (-ENOMEM);
	}
	
	cf_dev->input->name = "cf-keys";
    __set_bit(EV_KEY  , cf_dev->input->evbit );
    __set_bit(KEY_HOME, cf_dev->input->keybit);
    __set_bit(KEY_MENU, cf_dev->input->keybit);
    __set_bit(KEY_BACK, cf_dev->input->keybit);
	__set_bit(KEY_F18, cf_dev->input->keybit);
	__set_bit(KEY_F19, cf_dev->input->keybit);
	__set_bit(KEY_F20, cf_dev->input->keybit);
	__set_bit(KEY_F21, cf_dev->input->keybit);
	__set_bit(KEY_ENTER, cf_dev->input->evbit );
    __set_bit(KEY_UP, cf_dev->input->keybit);
    __set_bit(KEY_LEFT, cf_dev->input->keybit);
    __set_bit(KEY_RIGHT, cf_dev->input->keybit);
	__set_bit(KEY_DOWN, cf_dev->input->keybit);
	__set_bit(KEY_WAKEUP, cf_dev->input->keybit);	
	
	rc = input_register_device(cf_dev->input);
	if (rc) {
		dev_err(dev ,"input_register_device fail. \n");
		input_free_device(cf_dev->input);
		cf_dev->input = NULL;
		return (-ENODEV);
	}

	return rc;
}


static int cf_reset_gpio_set_value(struct cf_device *cf_dev, unsigned char th)
{
	int rc;

	FUNC_ENTRY();

	if (cf_dev->rst_gpio == 0) {
		dev_info(cf_dev->dev, "rst_gpio is not get. \n");
		rc = (-EINVAL);
		goto exit;
	}

	gpio_direction_output(cf_dev->rst_gpio,1);
	msleep(1);
	
	if (!!th) 
		gpio_set_value(cf_dev->rst_gpio, 1);
	else 
		gpio_set_value(cf_dev->rst_gpio, 0);

exit:
	return rc;
}

static int cf_report_key_event(struct input_dev* input, cf_key_event_t* kevent)
{
    int rc = 0;
    unsigned int key_code = KEY_UNKNOWN;

	FUNC_ENTRY();

    switch (kevent->key) {
		case CF_KEY_HOME:	key_code = KEY_HOME;   break;
		case CF_KEY_MENU:	key_code = KEY_MENU;   break;
		case CF_KEY_BACK:	key_code = KEY_BACK;   break;
		case CF_KEY_F18:	key_code = KEY_F18;    break;
		case CF_KEY_F19:	key_code = KEY_F19;    break;
		case CF_KEY_F20:	key_code = KEY_F20;    break;
		case CF_KEY_F21:	key_code = KEY_F21;    break;
		case CF_KEY_ENTER:	key_code = KEY_ENTER;  break;
		case CF_KEY_UP: 	key_code = KEY_UP;	   break;
		case CF_KEY_LEFT:	key_code = KEY_LEFT;   break;
		case CF_KEY_RIGHT:	key_code = KEY_RIGHT;  break;
		case CF_KEY_DOWN:	key_code = KEY_DOWN;   break;
		case CF_KEY_WAKEUP: key_code = KEY_WAKEUP; break;
		
		default: break;
    }

	if (kevent->value == 2) {
        input_report_key(input, key_code, 1);
        input_sync(input);	
        input_report_key(input, key_code, 0);
        input_sync(input);		
	} else {
        input_report_key(input, key_code, kevent->value);
        input_sync(input);		
	}

    return rc;	
}


static const char* cf_get_version(void)
{
    static char version[CF_DRV_VERSION_LEN] = {'\0', };

	FUNC_ENTRY();

    strncpy(version, CF_DRV_VERSION, CF_DRV_VERSION_LEN);
    version[CF_DRV_VERSION_LEN - 1] = '\0';
    return (const char*)version;
}

/*
static int cf_free_gpio(struct cf_device *cf_dev)
{
	return 0;
}
*/

static int cf_open(struct inode* inode, struct file* file)
{
	struct cf_device *cf_dev;

	FUNC_ENTRY();

	cf_dev = container_of(inode->i_cdev, struct cf_device, cdev);
	file->private_data = cf_dev;
	return 0;	
}

static int cf_release(struct inode* inode, struct file* file)
{
	return 0;
}

static ssize_t cf_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	struct cf_device *cf_dev = file->private_data;
	u8 rxbuf[2] = {0};

	FUNC_ENTRY();

	hw_reset(cf_dev);

	hw_reset(cf_dev);
	memset(rxbuf,0,sizeof(rxbuf));

	return count;
}

static long cf_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
	
	struct cf_device *cf_dev = NULL;
	int err = 0;
	unsigned char th = 0;
	cf_key_event_t kevent;
	
	cf_dev = file->private_data;
	
	if(_IOC_TYPE(cmd)!=CF_IOC_MAGIC)
		return -ENOTTY;
		
	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg,_IOC_SIZE(cmd));
	if(err==0 && _IOC_DIR(cmd)&_IOC_WRITE)
		err = !access_ok(VERIFY_READ,(void __user*)arg,_IOC_SIZE(cmd));
	if(err)
		return -EFAULT;	

	spin_lock_irq(&cf_dev->spin_lock);
	spin_unlock_irq(&cf_dev->spin_lock);

	mutex_lock(&cf_dev->lock);
	switch (cmd) {
		case CF_IOC_INIT_GPIO: {
 			err = cf_gpio_init(cf_dev);
			//TODO
			break;
		}
	
		case CF_IOC_DEINIT_GPIO: {
			err = cf_gpio_uninit(cf_dev);
			break;
		}
		
		case CF_IOC_RESET_DEVICE: {
			if (__get_user(th, (u8 __user*)arg)) {
				err = (-EFAULT);
				break;
			}

			cf_reset_gpio_set_value(cf_dev, th);
			break;
		}
		
        case CF_IOC_ENABLE_IRQ: {
            cf_enable_irq(cf_dev);
            break;
        }

        case CF_IOC_DISABLE_IRQ: {
            cf_disable_irq(cf_dev);
            break;
        }

        case CF_IOC_REQUEST_IRQ: {
            err = cf_irq_init(cf_dev);
            break;
        }

        case CF_IOC_ENABLE_SPI_CLK: {
            break;
        }

        case CF_IOC_DISABLE_SPI_CLK: {
            break;
        }

        case CF_IOC_ENABLE_POWER: {
			err = vreg_setup(cf_dev, "vdd", true);
            break;
        }

        case CF_IOC_DISABLE_POWER: {
			err = vreg_setup(cf_dev, "vdd", false);
            break;
        }		

        case CF_IOC_REPORT_KEY_EVENT: {
            if (copy_from_user(&kevent, (cf_key_event_t*)arg, sizeof(cf_key_event_t))) {
                dev_err(cf_dev->dev, "copy_from_user(..) failed.\n");
                err = (-EFAULT);
                break;
            }
            err = cf_report_key_event(cf_dev->input, &kevent);
            break;
        }

        case CF_IOC_SYNC_CONFIG: {
            break;
        }

        case CF_IOC_GET_VERSION: {
            if (copy_to_user((void*)arg, cf_get_version(), CF_DRV_VERSION_LEN)) {
                dev_err(cf_dev->dev, "copy_to_user(..) failed.\n");
                err = (-EFAULT);
                break;
            }

            break;
        }

        default:
            err = (-EINVAL);
            break;
	}

	if(NULL != cf_dev->buf){
		kfree(cf_dev->buf);
		cf_dev->buf = NULL;
	}

	mutex_unlock(&cf_dev->lock);

	return err;	
}

static int cf_fasync(int fd, struct file *fp, int mode)
{
	struct cf_device *cf_dev;

//	FUNC_ENTRY();
//	printk("chipsailing\n");

	cf_dev = fp->private_data;
	return fasync_helper(fd, fp, mode, &cf_dev->async);
}

static const struct file_operations cf_fops =
{
    .owner			= THIS_MODULE,
    .open			= cf_open,
    .release		= cf_release,
    .unlocked_ioctl	= cf_ioctl,
	.fasync         = cf_fasync,
	.write          = cf_write,
};

#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block* self, unsigned long event, void* data)
{
    struct fb_event *evdata = data;
    int* blank;
    struct cf_device *cf_dev = container_of(self, struct cf_device, fb_notify);

    if (evdata && evdata->data && cf_dev) {
        if (event == FB_EVENT_BLANK) {
            blank = evdata->data;

            if (*blank == FB_BLANK_UNBLANK) {
			}
            else if (*blank == FB_BLANK_POWERDOWN) {
                kill_fasync(&cf_dev->async, SIGIO, POLL_IN);
			}
        }
    }

    return 0;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND

static void cf_early_suspend(struct early_suspend *handler)
{
	FUNC_ENTRY();

	struct cf_device *cf_dev = container_of(handler, struct cf_device, early_suspend);
}

static void cf_late_resume(struct early_suspend *handler)
{
	FUNC_ENTRY();

	struct cf_device *cf_dev = container_of(handler, struct cf_device, early_suspend);
}
#endif


static int cf_probe(struct platform_device *spi)
{
	int rc = 0;
	struct device *dev;
	struct device_node *np;
	struct cf_device *cf_dev;
	dev = &spi->dev;

	FUNC_ENTRY();

    np = of_find_compatible_node(NULL, NULL, "csfinger,cs358");
    if (!np) {
		dev_err(dev, "device node is null! \n");
		rc = (-EINVAL);
		goto exit;
    }

	cf_dev = devm_kzalloc(dev, sizeof(*cf_dev),
			GFP_KERNEL);
	if (!cf_dev) {
		dev_err(dev,
			"failed to allocate memory for struct cf_device. \n");
		rc = -ENOMEM;
		goto exit;
	}
	
	cf_dev->dev = dev;
	dev_set_drvdata(dev, cf_dev);
	cf_dev->spi = spi;

	cf_dev->class = class_create(THIS_MODULE, FP_CLASS_NAME);

	rc = alloc_chrdev_region(&cf_dev->devno, 0, 1, FP_DEV_NAME);
	if (rc) {
	  dev_err(dev, "alloc_chrdev_region failed, error = %d. \n", rc);
	  goto exit;
	}
	cf_dev->device = device_create(cf_dev->class, NULL, cf_dev->devno,
									  NULL, "%s", FP_DEV_NAME);
	cdev_init(&cf_dev->cdev, &cf_fops);
	cf_dev->cdev.owner = THIS_MODULE;
	
	rc = cdev_add(&cf_dev->cdev, cf_dev->devno, 1);
	if (rc) {
	  dev_err(dev, "cdev_add failed, error = %d. \n", rc);
	  goto exit;
	}

	rc = cf_pinctrl_look_up_states(cf_dev);
    if (rc)
		goto exit;

	atomic_set(&cf_dev->wakeup_enabled, 1);
	cf_dev->clocks_enabled = false;
	cf_dev->clocks_suspended = false;
	
	mutex_init(&cf_dev->lock);
	spin_lock_init(&cf_dev->spin_lock);
	wake_lock_init(&cf_dev->ttw_wl, WAKE_LOCK_SUSPEND, "fpc_ttw_wl");

    rc = cf_input_init(cf_dev);
	if (rc) {
		dev_err(dev, "could not register input. \n");
		goto exit;
	}
	
	rc = sysfs_create_group(&dev->kobj, &attribute_group);
	if (rc) {
		dev_err(dev, "could not create sysfs. \n");
		goto exit;
	}
	
	if (of_property_read_bool(np, "cdfinger,enable-on-boot")) {   //dev->of_node
		(void)device_prepare(cf_dev, true);
		(void)set_clks(cf_dev, false);
	}

#ifdef CONFIG_FB
	cf_dev->fb_notify.notifier_call = fb_notifier_callback;
	fb_register_client(&cf_dev->fb_notify);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	cf_dev->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
						    FT_SUSPEND_LEVEL;
	cf_dev->early_suspend.suspend = cf_early_suspend;
	cf_dev->early_suspend.resume = cf_late_resume;
	register_early_suspend(&cf_dev->early_suspend);
#endif	

exit:
	return rc;
}

static int cf_remove(struct platform_device *spi)
{
	struct cf_device *cf_dev = dev_get_drvdata(&spi->dev);
	
#ifdef CONFIG_HAS_EARLYSUSPENDCONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&cf_dev->early_suspend);
#endif

#ifdef CONFIG_FB
    if (cf_dev->fb_notify.notifier_call) {
		cf_dev->fb_notify.notifier_call = NULL;
		fb_unregister_client(&cf_dev->fb_notify);
	}
#endif

	sysfs_remove_group(&spi->dev.kobj, &attribute_group);
	mutex_destroy(&cf_dev->lock);
	wake_lock_destroy(&cf_dev->ttw_wl);
	
	(void)vreg_setup(cf_dev, "vdd", false);

	return 0;
}

static int cf_suspend(struct device *dev)
{
	return 0;
}

static int cf_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops cf_pm_ops = {
	.suspend = cf_suspend,
	.resume = cf_resume,
};

static const struct of_device_id cf_of_match[] = {
	{ .compatible = "csfinger,cs358", },
	{},
};

static const struct platform_device_id cf_id[] = {
	{"csfinger_fp", 0},
	{}
};

static struct platform_driver cf_driver = {
	.driver = {
		.name	= "csfinger_fp",
		.owner	= THIS_MODULE,
		.of_match_table = cf_of_match,
		.pm = &cf_pm_ops,
	},
	.id_table = cf_id,
	.probe		= cf_probe,
	.remove		= cf_remove,
};

static int __init cf_driver_init(void)
{
	int rc;
	
	rc = platform_driver_register(&cf_driver);
	if (!rc)
		pr_info("spi_register_driver(..) pass. \n");
	else
		pr_err("spi_register_driver(..) fail, error = %d. \n", rc);
			
	return rc;
}

static void __exit cf_driver_exit(void)
{
	platform_driver_unregister(&cf_driver);
}

module_init(cf_driver_init);
module_exit(cf_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("zwp@chipsailing.com");
MODULE_DESCRIPTION("ChipSailing Fingerprint sensor device driver.");
