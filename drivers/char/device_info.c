
#include <linux/sysfs.h>
#include <linux/kobject.h>

char *fingerprint_info = "unknow";

static ssize_t fingerprint_info_show(struct kobject *kobj, struct kobj_attribute *attr,
	char *buf)
{
	int size = 0;

	size = sprintf(buf, "%s\n", fingerprint_info);

	return size;
}

static struct kobj_attribute fingerprint_info_attribute = {
	.attr = {
		.mode = S_IRUGO,
		.name = "fingerprint_info",
	},
	.show = fingerprint_info_show,
};

static struct attribute *attrs[] = {
	&fingerprint_info_attribute.attr,
	NULL,
};


static struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct kobject *device_info_kobj;

static int __init device_info_init(void)
{
	int retval;

	device_info_kobj = kobject_create_and_add("device_info", NULL);
	if (!device_info_kobj){
		printk("device_info_init: unable to create kobject\n");
		return -ENOMEM;
	}
	retval = sysfs_create_group(device_info_kobj, &attr_group);
	if (retval){
		printk("device_info_init: error to create kobject\n");
		kobject_put(device_info_kobj);
	}

	return retval;
}

static void __exit device_info_exit(void)
{
	kobject_put(device_info_kobj);
}

module_init(device_info_init);
module_exit(device_info_exit);
