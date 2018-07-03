/*! \file sar_input.c
 * \brief  sar_input Driver
 *
 * Driver for the sar_input
 * Copyright (c) 2017
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#define DEBUG

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/init.h>

#define INVALID_GPIO (-1)

struct sar_input {
	struct device *dev;
	int irq;
	struct input_dev *input_dev;
	int irq_gpio;
	struct delayed_work input_report;
	spinlock_t r_lock;
};

static int sar_input_irq_state(struct sar_input *sdata)
{
	int ret = 0;
	if (!gpio_is_valid(sdata->irq_gpio)) {
		dev_err(sdata->dev, "kongkou irq_gpio was not assigned properly");
		ret = -EINVAL;
		goto err_state;
	}

	ret = gpio_get_value(sdata->irq_gpio);
err_state:
	return ret;
}

static void input_report_work(struct  work_struct *work)
{
	int value;
	struct input_dev *input = NULL;
	unsigned long flags;
	struct sar_input *sdata = container_of(work, struct sar_input, input_report.work);

	spin_lock_irqsave(&sdata->r_lock, flags);
	value = sar_input_irq_state(sdata);
	if (value < 0) {
		dev_err(sdata->dev, "error to get gpio status\n");
		goto err_value;
	}

	input = sdata->input_dev;
	if (value) {
		input_report_key(input, KEY_KONGKOU_CONNECT, 1);
		input_report_key(input, KEY_KONGKOU_CONNECT, 0);
	} else {
		input_report_key(input, KEY_KONGKOU_UNCONNECT, 1);
		input_report_key(input, KEY_KONGKOU_UNCONNECT, 0);
	}

	input_sync(input);

err_value:
	spin_unlock_irqrestore(&sdata->r_lock, flags);
	dev_dbg(sdata->dev, "%s value = %d\n", __func__, value);

	return;
}

#define TIME_TO_GET_GPIO_STATE_DELAY 20
static irqreturn_t sar_input_handle_irq(int irq, void *data)
{

	struct sar_input *pdata = (struct sar_input *)data;

	schedule_delayed_work(&pdata->input_report, msecs_to_jiffies(TIME_TO_GET_GPIO_STATE_DELAY));

	return IRQ_HANDLED;
}

static int sar_input_parse_dt(struct device *dev, struct sar_input *pdata)
{
	int ret = 0;
	pdata->irq_gpio = of_get_named_gpio_flags(dev->of_node, "qcom,sar_input_gpio_irq", 0, NULL);
	dev_dbg(dev, "irq_gpio %d\n", pdata->irq_gpio);
	if (pdata->irq_gpio < 0) {
		pdata->irq_gpio = INVALID_GPIO;
		dev_err(dev, "sar input irq gpio is not available\n");
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t si_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned long flag;
	int state;
	struct sar_input *sdata = dev_get_drvdata(dev);

	spin_lock_irqsave(&sdata->r_lock, flag);
	state = sar_input_irq_state(sdata);
	spin_unlock_irqrestore(&sdata->r_lock, flag);
	dev_dbg(dev, "sar input state_show = %d\n", state);

	return sprintf(buf, "%d\n", (state? 1: 0));
}

static DEVICE_ATTR(si_state, S_IRUGO, si_state_show, NULL);

static int sar_input_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct sar_input *sar_input_data;

	dev_dbg(&pdev->dev, "sar input probe\n");

	sar_input_data = devm_kzalloc(&pdev->dev, sizeof(struct sar_input), GFP_KERNEL);
	sar_input_data->dev = &pdev->dev;

	if (!sar_input_data) {
		dev_err(&pdev->dev, "Can't allocate sar_input_data\n");
		return -ENOMEM;
	}

	spin_lock_init(&sar_input_data->r_lock);
	INIT_DELAYED_WORK(&sar_input_data->input_report, input_report_work);

	ret = sar_input_parse_dt(sar_input_data->dev, sar_input_data);
	if (ret) {
		dev_err(&pdev->dev, "Failed to parse device tree\n");
		goto err;
	}

	if (gpio_is_valid(sar_input_data->irq_gpio)) {
		/* sar input interrupt gpio request */
		ret = gpio_request(sar_input_data->irq_gpio, "KONGKOU_NIRQ");
		if (ret < 0) {
			dev_err(&pdev->dev, "gpio req failed for kongkou interrupt\n");
			goto err;
		}
		ret = gpio_direction_input(sar_input_data->irq_gpio);
		if (ret) {
			dev_err(&pdev->dev,"unable to set direction for gpio [%d]\n", sar_input_data->irq_gpio);
			goto err_1;
		}
	} else {
		dev_err(&pdev->dev,"the irq gpio is invalid\n");
		ret = -EINVAL;
		goto err;
	}
	/* do we need to create an irq timer after interrupt ? */
    sar_input_data->irq = gpio_to_irq(sar_input_data->irq_gpio);
    dev_dbg(&pdev->dev, "gpio_to_irq %d\n", sar_input_data->irq);

	sar_input_data->input_dev = devm_input_allocate_device(&pdev->dev);
	if (!sar_input_data->input_dev) {
		dev_err(&pdev->dev, "Can't allocate input dev\n");
		ret = -ENOMEM;
		goto err_1;
	}

	__set_bit(EV_KEY, sar_input_data->input_dev->evbit);
	__set_bit(KEY_KONGKOU_CONNECT, sar_input_data->input_dev->keybit);
	__set_bit(KEY_KONGKOU_UNCONNECT, sar_input_data->input_dev->keybit);
	sar_input_data->input_dev->name = "sar_input";
	sar_input_data->input_dev->phys = "sar_input_data/input0";
	sar_input_data->input_dev->dev.parent = &pdev->dev;
	sar_input_data->input_dev->id.bustype = BUS_HOST;

	ret = request_threaded_irq(sar_input_data->irq, NULL, sar_input_handle_irq,
			   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING| IRQF_ONESHOT, "sar_input", sar_input_data);

	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to request IRQ: %d\n", ret);
		goto err_1;
	}

	ret = input_register_device(sar_input_data->input_dev);
	if (ret) {
		dev_dbg(&pdev->dev, "Can't register input device: %d\n", ret);
		goto err_2;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_si_state);
	if (ret) {
		dev_dbg(&pdev->dev, "Can't creat sar input device file: %d\n", ret);
		goto err_3;
	}

	dev_set_drvdata(&pdev->dev, sar_input_data);
	platform_set_drvdata(pdev, sar_input_data);

	return 0;

err_3:
	input_unregister_device(sar_input_data->input_dev);
err_2:
	free_irq(sar_input_data->irq, sar_input_data);
err_1:
	gpio_free(sar_input_data->irq_gpio);
err:
	sar_input_data->irq_gpio = INVALID_GPIO;
	cancel_delayed_work_sync(&sar_input_data->input_report);
	devm_kfree(sar_input_data->dev, sar_input_data);

	return ret;

}

static int sar_input_remove(struct platform_device *pdev)
{
	struct sar_input *pdata = platform_get_drvdata(pdev);

	input_unregister_device(pdata->input_dev);
	gpio_free(pdata->irq_gpio);
	free_irq(pdata->irq, pdata);
	cancel_delayed_work_sync(&pdata->input_report);
	devm_kfree(pdata->dev, pdata);

	return 0;
}

 static  struct of_device_id sar_input_gpio_of_match[]  = {
	{ .compatible = "qcom,sar_input", },
	{},
};

static struct platform_driver sar_input_driver = {
	.probe		= sar_input_probe,
	.remove		= sar_input_remove,
	.driver		= {
		.name	= "sar_input",
		.owner	= THIS_MODULE,
		.of_match_table = sar_input_gpio_of_match,
	},
};


static int __init sar_input_gpio_init(void)
{
     return platform_driver_register(&sar_input_driver);
}

static void __init sar_input_gpio_exit(void)
{
     platform_driver_unregister(&sar_input_driver);
}

module_init(sar_input_gpio_init);
module_exit(sar_input_gpio_exit);

MODULE_ALIAS("platform:sar_input");
MODULE_DESCRIPTION("Sar Input Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Wangzhe");

