#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include <linux/kthread.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/spi/spidev.h>
#include <linux/semaphore.h>
#include <linux/poll.h>
#include <linux/fcntl.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/input.h>
#include <linux/signal.h>
#include <linux/gpio.h>
#include <linux/mm.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/of_irq.h>
#include <linux/regulator/consumer.h>
#include <linux/fb.h>
#include <linux/notifier.h>

#define DEVICE_NAME "fpsdev0"
#define CDFINGER_IOCTL_MAGIC_NO          0xFB
#define CDFINGER_INIT                    _IOW(CDFINGER_IOCTL_MAGIC_NO, 0, uint8_t)
#define CDFINGER_GETIMAGE                _IOW(CDFINGER_IOCTL_MAGIC_NO, 1, uint8_t)
#define CDFINGER_INITERRUPT_MODE	 _IOW(CDFINGER_IOCTL_MAGIC_NO, 2, uint8_t)
#define CDFINGER_INITERRUPT_KEYMODE      _IOW(CDFINGER_IOCTL_MAGIC_NO, 3, uint8_t)
#define CDFINGER_INITERRUPT_FINGERUPMODE _IOW(CDFINGER_IOCTL_MAGIC_NO, 4, uint8_t)
#define CDFINGER_RELEASE_WAKELOCK        _IO(CDFINGER_IOCTL_MAGIC_NO, 5)
#define CDFINGER_CHECK_INTERRUPT         _IO(CDFINGER_IOCTL_MAGIC_NO, 6)
#define CDFINGER_SET_SPI_SPEED           _IOW(CDFINGER_IOCTL_MAGIC_NO, 7, uint32_t)
#define CDFINGER_REPORT_KEY              _IOW(CDFINGER_IOCTL_MAGIC_NO, 10, uint8_t)
#define CDFINGER_POWERDOWN               _IO(CDFINGER_IOCTL_MAGIC_NO, 11)
#define	CDFINGER_GETID			 _IO(CDFINGER_IOCTL_MAGIC_NO,12)

#define CDFINGER_INIT_GPIO		 _IO(CDFINGER_IOCTL_MAGIC_NO,20)
#define CDFINGER_INIT_IRQ		 _IO(CDFINGER_IOCTL_MAGIC_NO,21)
#define CDFINGER_POWER_ON		 _IO(CDFINGER_IOCTL_MAGIC_NO,22)
#define CDFINGER_RESET		 	 _IO(CDFINGER_IOCTL_MAGIC_NO,23)
#define CDFINGER_RELEASE_DEVICE		_IO(CDFINGER_IOCTL_MAGIC_NO,25)

#define CDFINGER_DISABLE_IRQ               _IO(CDFINGER_IOCTL_MAGIC_NO, 13)
#define CDFINGER_HW_RESET               _IOW(CDFINGER_IOCTL_MAGIC_NO, 14, uint8_t)
#define CDFINGER_GET_STATUS               _IO(CDFINGER_IOCTL_MAGIC_NO, 15)
#define CDFINGER_NEW_KEYMODE		_IOW(CDFINGER_IOCTL_MAGIC_NO, 37, uint8_t)
#define CDFINGER_WAKE_LOCK	 _IOW(CDFINGER_IOCTL_MAGIC_NO,26,uint8_t)

static int isInKeyMode = 0; // key mode
static int screen_status = 1; // screen on
static u8 cdfinger_debug = 0x01;
static int isInit = 0;

#define CDFINGER_DBG(fmt, args...) \
	do{ \
		if(cdfinger_debug & 0x01) \
		printk( "[DBG][cdfinger]:%5d: <%s>" fmt, __LINE__,__func__,##args ); \
	}while(0)

#define CDFINGER_ERR(fmt, args...) \
	do{ \
		printk( "[DBG][cdfinger]:%5d: <%s>" fmt, __LINE__,__func__,##args ); \
	}while(0)

struct cdfingerfp_data {
	struct platform_device *cdfinger_dev;
	struct miscdevice *miscdev;
	u32 irq_num;
	u32 reset_num;
	struct regulator *iovdd;
	struct regulator *vdd;
	struct fasync_struct *async_queue;
	struct wake_lock cdfinger_lock;
	struct input_dev* input;
	struct notifier_block notifier;
	struct mutex buf_lock;
}*g_cdfingerfp_data;

static int cdfinger_init_gpio(struct cdfingerfp_data *cdfinger)
{
	int err = 0;

	if (gpio_is_valid(cdfinger->reset_num)) {
		err = gpio_request(cdfinger->reset_num, "cdfinger-reset");
		if (err) {
			gpio_free(cdfinger->reset_num);
			err = gpio_request(cdfinger->reset_num, "cdfinger-reset");
			if (err) {
				CDFINGER_ERR("Could not request reset gpio.\n");
				return err;
			}
		}
		gpio_direction_output(cdfinger->reset_num, 1);
	}
	else {
		CDFINGER_ERR("not valid reset gpio\n");
		return -EIO;
	}

	if (gpio_is_valid(cdfinger->irq_num)) {
		err = gpio_request(cdfinger->irq_num, "cdfinger-irq");
		if (err) {
            gpio_free(cdfinger->irq_num);
            err = gpio_request(cdfinger->irq_num, "cdfinger-irq");
			if (err) {
				CDFINGER_ERR("Could not request irq gpio.\n");
				gpio_free(cdfinger->reset_num);
				return err;
			}
		}
		gpio_direction_input(cdfinger->irq_num);
	}
	else {
		CDFINGER_ERR(KERN_ERR "not valid irq gpio\n");
		gpio_free(cdfinger->reset_num);
		return -EIO;
	}

	return err;
}

static int cdfinger_free_gpio(struct cdfingerfp_data *cdfinger)
{
	int err = 0;

	CDFINGER_DBG("enter\n");

	if (gpio_is_valid(cdfinger->irq_num)) {
		gpio_free(cdfinger->irq_num);
		free_irq(gpio_to_irq(cdfinger->irq_num), (void*)cdfinger);
	}

	if (gpio_is_valid(cdfinger->reset_num)) {
		gpio_free(cdfinger->reset_num);
	}

	return err;
}

static void cdfinger_reset(struct cdfingerfp_data *pdata, int ms)
{
	CDFINGER_DBG("enter\n");

	gpio_set_value(pdata->reset_num, 1);
	mdelay(ms);
	gpio_set_value(pdata->reset_num, 0);
	mdelay(ms);
	gpio_set_value(pdata->reset_num, 1);
	mdelay(ms);
}

static int cdfinger_parse_dts(struct device *dev, struct cdfingerfp_data *cdfinger)
{
	int err = 0;

	CDFINGER_DBG("enter\n");

	cdfinger->reset_num = of_get_named_gpio(dev->of_node, "cdfinger,reset_gpio", 0);
	cdfinger->irq_num = of_get_named_gpio(dev->of_node, "cdfinger,irq_gpio", 0);
    cdfinger->iovdd = regulator_get(dev, "vio");
    cdfinger->vdd = regulator_get(dev, "vdd");

	return err;
}

static int cdfinger_power_on(struct cdfingerfp_data *pdata)
{
    int ret = 0;

	CDFINGER_DBG("enter\n");

    regulator_set_voltage(pdata->vdd, 0, 2800000);
	ret = regulator_enable(pdata->vdd);
	if(ret)  {
		CDFINGER_ERR("enable regulato fail\n");
		return ret;
	}

	mdelay(1);
	gpio_set_value(pdata->reset_num, 1);
	msleep(10);

	return 0;
}

static int cdfinger_open(struct inode *inode, struct file *file)
{
	CDFINGER_DBG("enter\n");

	file->private_data = g_cdfingerfp_data;
	return 0;
}

static int cdfinger_async_fasync(int fd, struct file *file,int mode)
{
	struct cdfingerfp_data *cdfingerfp = g_cdfingerfp_data;
	return fasync_helper(fd,file,mode,&cdfingerfp->async_queue);
}

static int cdfinger_release(struct inode *inode,struct file *file)
{
	struct cdfingerfp_data *cdfingerfp = file->private_data;

	CDFINGER_DBG("enter\n");

	if(NULL == cdfingerfp)
	{
		return -EIO;
	}

	file->private_data = NULL;

	return 0;
}

static void cdfinger_async_report(void)
{
	struct cdfingerfp_data *cdfingerfp = g_cdfingerfp_data;
	wake_lock_timeout(&cdfingerfp->cdfinger_lock, msecs_to_jiffies(1000));
	kill_fasync(&cdfingerfp->async_queue,SIGIO,POLL_IN);
}

static irqreturn_t cdfinger_eint_handler(int irq, void *dev_id)
{
	cdfinger_async_report();
	return IRQ_HANDLED;
}

extern char *fingerprint_info;

static int cdfinger_init_irq(struct cdfingerfp_data *pdata)
{
	int error = 0;

	CDFINGER_DBG("enter\n");

	if (isInit == 1)
		return 0;

	error =request_irq(gpio_to_irq(pdata->irq_num),cdfinger_eint_handler,IRQF_TRIGGER_RISING,"cdfinger_eint", NULL);
	if (error < 0)
	{
        CDFINGER_ERR("irq init err\n");
		return error;
	}
	enable_irq_wake(gpio_to_irq(pdata->irq_num));
	isInit = 1;

	fingerprint_info = "cdfinger";

	return error;
}

static void cdfinger_wake_lock(struct cdfingerfp_data *pdata,int arg)
{
	if(arg)
	{
		wake_lock(&pdata->cdfinger_lock);	
	}else{
		wake_unlock(&pdata->cdfinger_lock);
		wake_lock_timeout(&pdata->cdfinger_lock, msecs_to_jiffies(3000));
	}
}

static long cdfinger_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;

	struct cdfingerfp_data *cdfinger = filp->private_data;

	mutex_lock(&cdfinger->buf_lock);
	switch (cmd) {
		case CDFINGER_INIT_GPIO:
			err = cdfinger_init_gpio(cdfinger);
			break;

		case CDFINGER_INIT_IRQ:
			err = cdfinger_init_irq(cdfinger);
			break;

		case CDFINGER_RELEASE_DEVICE:
			isInit = 0;
			cdfinger_free_gpio(cdfinger);
			if (cdfinger->input) {
				input_unregister_device(cdfinger->input);
			}
			misc_deregister(cdfinger->miscdev);
			break;

		case CDFINGER_WAKE_LOCK:
			cdfinger_wake_lock(cdfinger,arg);
			break;	
		
		case CDFINGER_POWER_ON:
			err = cdfinger_power_on(cdfinger);
			break;

		case CDFINGER_RESET:
			cdfinger_reset(cdfinger,1);
			break;

		case CDFINGER_INITERRUPT_MODE:
			isInKeyMode = 1;  // not key mode
			cdfinger_reset(cdfinger,1);
			break;

		case CDFINGER_NEW_KEYMODE:
			isInKeyMode = 0;
			cdfinger_reset(cdfinger,1);
			break;

		case CDFINGER_HW_RESET:
			cdfinger_reset(cdfinger,arg);
			break;

		case CDFINGER_GET_STATUS:
			err = screen_status;
			break;

		default:
			break;	
	}
	mutex_unlock(&cdfinger->buf_lock);

	return err;
}

static const struct file_operations cdfinger_fops = {
	.owner	 = THIS_MODULE,
	.open	 = cdfinger_open,
	.unlocked_ioctl = cdfinger_ioctl,
	.release = cdfinger_release,
	.fasync  = cdfinger_async_fasync,
#ifdef CONFIG_COMPAT
	.compat_ioctl = cdfinger_ioctl,
#endif
};

static struct miscdevice st_cdfinger_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &cdfinger_fops,
};

static int cdfinger_fb_notifier_callback(struct notifier_block* self,
                                        unsigned long event, void* data)
{
    struct fb_event* evdata = data;
    unsigned int blank;
    int retval = 0;

    if (event != FB_EVENT_BLANK /* FB_EARLY_EVENT_BLANK */) {
        return 0;
    }

    blank = *(int*)evdata->data;

    switch (blank) {
        case FB_BLANK_UNBLANK:
			mutex_lock(&g_cdfingerfp_data->buf_lock);
			screen_status = 1;
			if (isInKeyMode == 0)
				cdfinger_async_report();
			mutex_unlock(&g_cdfingerfp_data->buf_lock);
            break;

        case FB_BLANK_POWERDOWN:
			mutex_lock(&g_cdfingerfp_data->buf_lock);
			screen_status = 0;
			if (isInKeyMode == 0)
				cdfinger_async_report();
			mutex_unlock(&g_cdfingerfp_data->buf_lock);
            break;

        default:
            break;
    }

    return retval;
}

static int cdfinger_probe(struct platform_device *pdev)
{
	struct cdfingerfp_data *cdfingerdev= NULL;
	int status = -ENODEV;

	CDFINGER_DBG("enter\n");

	status = misc_register(&st_cdfinger_dev);

	if (status) {
		CDFINGER_ERR("cdfinger misc register err%d\n",status);
		return -1;	
	}

	cdfingerdev = kzalloc(sizeof(struct cdfingerfp_data),GFP_KERNEL);
	cdfingerdev->miscdev = &st_cdfinger_dev;
	cdfingerdev->cdfinger_dev = pdev;

	mutex_init(&cdfingerdev->buf_lock);
	wake_lock_init(&cdfingerdev->cdfinger_lock, WAKE_LOCK_SUSPEND, "cdfinger wakelock");

	status=cdfinger_parse_dts(&cdfingerdev->cdfinger_dev->dev, cdfingerdev);
	if(status){
		CDFINGER_ERR("cdfinger parse err %d\n",status);
		goto unregister_dev;	
	}

	cdfingerdev->notifier.notifier_call = cdfinger_fb_notifier_callback;
    fb_register_client(&cdfingerdev->notifier);

	g_cdfingerfp_data = cdfingerdev;

	CDFINGER_DBG("exit\n");

	return 0;

unregister_dev:
	misc_deregister(&st_cdfinger_dev);

	return  status;
}


static const struct of_device_id cdfinger_of_match[] = {
	{ .compatible = "cdfinger,fps998", },
	{},
};

static const struct platform_device_id cdfinger_id[] = {
	{"cdfinger_fp", 0},
	{}
};

static struct platform_driver cdfinger_driver = {
	.driver = {
		.name = "cdfinger_fp",
		.of_match_table = cdfinger_of_match,
	},
	.id_table = cdfinger_id,
	.probe = cdfinger_probe,
};

static int __init cdfinger_fp_init(void)
{
	return platform_driver_register(&cdfinger_driver);
}

static void __exit cdfinger_fp_exit(void)
{
	platform_driver_unregister(&cdfinger_driver);
}

module_init(cdfinger_fp_init);

module_exit(cdfinger_fp_exit);

MODULE_DESCRIPTION("cdfinger Driver");
MODULE_AUTHOR("cdfinger@cdfinger.com");
MODULE_LICENSE("GPL");
MODULE_ALIAS("cdfinger");
