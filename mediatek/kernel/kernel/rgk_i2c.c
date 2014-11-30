#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/rgk_board.h>
#include <linux/string.h>
#include <linux/i2c.h>

static ssize_t i2c_status(struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
	ssize_t rc;

	rc = snprintf(buf, 200, "%s\n", "hello i2c");
	return rc;
}

/**buf_out
 * [0]: i2c num;
 * [1]: slave addr
 * [2]: write data length
 * [3]: read data length
 * [4]: write data start
 * [4+[2]]: read data start
 * [4+[2]+[3]]: end
 */
static ssize_t i2c_store(struct kobject *kobj,
		struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	int i;
	int ret;
	unsigned char buf_out[1024];
	struct i2c_adapter *adapter;
	struct i2c_msg msgs[2];
	unsigned char slave_addr;
	unsigned char write_data_length;
	unsigned char read_data_length;

	memset(buf_out, 0, sizeof(buf_out));
	ret = rgk_attr_data_analyse(buf_out, sizeof(buf_out), buf, n);

	adapter = i2c_get_adapter(buf_out[0]);
	if(!adapter) {
		printk("rgk: i2c_get_adapter error!\n");
		return n;
	}

	slave_addr = buf_out[1];
	write_data_length = buf_out[2];
	read_data_length = buf_out[3];

	memset(msgs, 0, sizeof(msgs));
	if(read_data_length)
	{
		msgs[0].addr = slave_addr;
		msgs[0].flags = 0;
		msgs[0].len = write_data_length;
		msgs[0].buf = &buf_out[4];
		msgs[0].timing = 300;
		
		msgs[1].addr = slave_addr;
		msgs[1].flags = I2C_M_RD;
		msgs[1].len = read_data_length;
		msgs[1].buf = &buf_out[4+write_data_length];
		msgs[1].timing = 300;
		i2c_transfer(adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	} else {
		msgs[0].addr = slave_addr;
		msgs[0].flags = 0;
		msgs[0].len = write_data_length;
		msgs[0].buf = &buf_out[4];
		msgs[0].timing = 300;
		i2c_transfer(adapter, msgs, 1);
	}

	i2c_put_adapter(adapter);
	
	printk("rgk: i2c[%d]--[%02x]: w-%d, r-%d\n", buf_out[0], slave_addr, write_data_length, read_data_length);

	if(write_data_length)
	{
		printk("w: ");
		for(i=0; i<write_data_length; i++)
		{
			printk("%02x ", buf_out[4+i]);
		}
		printk("\n");
	}
	if(read_data_length)
	{
		printk("r: ");
		for(i=0; i<read_data_length; i++)
		{
			printk("%02x ", buf_out[4+write_data_length+i]);
		}
		printk("\n");
	}
	printk("\n");

	return n;
}

static struct kobj_attribute i2c_attr = {
	.attr = {
		.name = "i2c",
		.mode = 0644,
	},
	.show = i2c_status,
	.store = i2c_store,
};

static struct attribute *i2c_attrs[] = {
	&i2c_attr.attr,
	NULL,
};

static struct attribute_group i2c_attr_group = {
	.name = "i2c",
	.attrs = i2c_attrs,
};

static int __init rgk_board_i2c_init(void)
{
	rgk_board_properties(&i2c_attr_group);
	return 0;
}

static void __exit rgk_board_i2c_exit(void)
{

}

module_init(rgk_board_i2c_init);
module_exit(rgk_board_i2c_exit);
