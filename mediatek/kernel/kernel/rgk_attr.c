#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/rgk_board.h>
#include <linux/string.h>

static ssize_t example_status(struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
	ssize_t rc;

	rc = snprintf(buf, 200, "%s\n", "hello test");
	return rc;
}

size_t rgk_attr_data_analyse(unsigned char *buf_out, size_t n_out, const char *buf, size_t n)
{
	int i, j, k;
	char *buf0;
	unsigned char *rgk_data;
	char *token;
	char *end;
	char *str;

	str = buf0 = kmalloc(n+1, GFP_KERNEL);
	strcpy(buf0, buf);

	end = strstr(buf0, "//");
	if(end)
		end[0] = '\0';
	end = strstr(buf0, "/*");
	if(end)
		end[0] = '\0';
	
	rgk_data = kmalloc(n+1, GFP_KERNEL);
	i = 0;
	while(token = strsep(&str, RGK_ATTR_CT))
	{
		if(strlen(token) != 0)
		{
			rgk_data[i] = simple_strtoul(token, NULL, 0);
			i++;
		}
	}
	
	k = sprintf(buf0, "%d:", i);
	for(j=0; j<i; j++)
	{
		k += sprintf(buf0+k, " %02x", rgk_data[j]);
	}
	printk("rgk_data: %s\n", buf0);

	if(i > n_out)
		i = n_out;
	for(j=0; j<i; j++)
		buf_out[j] = rgk_data[j];

	kfree(rgk_data);	
	kfree(buf0);
	return i;
}

static ssize_t example_store(struct kobject *kobj,
		struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	int i;
	int ret;
	unsigned char buf_out[5];

	ret = rgk_attr_data_analyse(buf_out, sizeof(buf_out), buf, n);

	return n;
}

static struct kobj_attribute example_attr = {
	.attr = {
		.name = "example",
		.mode = 0644,
	},
	.show = example_status,
	.store = example_store,
};

static struct attribute *example_attrs[] = {
	&example_attr.attr,
	NULL,
};

static struct attribute_group example_attr_group = {
	.name = "example",
	.attrs = example_attrs,
};

int rgk_board_properties(struct attribute_group *rgk_projects_attr_group)
{
	static struct kobject *rgk_board_properties_kobj = NULL;
	int rc;

	if(rgk_board_properties_kobj == NULL)
		rgk_board_properties_kobj = kobject_create_and_add("rgk_board_properties", NULL);
	if (rgk_board_properties_kobj)
		rc = sysfs_create_group(rgk_board_properties_kobj,
				rgk_projects_attr_group);

	if (!rgk_board_properties_kobj || rc)
	{
		printk("%s: failed to create rgk_board_properties\n", __func__);
		return -1;
	} else {
		return 0;
	}
	return 0;
}
EXPORT_SYMBOL(rgk_board_properties);

static int __init rgk_board_init(void)
{
	rgk_board_properties(&example_attr_group);
	return 0;
}

static void __exit rgk_board_exit(void)
{

}

module_init(rgk_board_init);
module_exit(rgk_board_exit);
