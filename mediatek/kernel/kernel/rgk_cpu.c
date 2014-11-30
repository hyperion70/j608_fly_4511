#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/rgk_board.h>
#include <linux/string.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>

#define CPU_DEBUG(fmt,arg...)	printk("%s-%d: " fmt "\n", __FUNCTION__, __LINE__, ##arg)

char *cpu_type[] = {
	"MT6591",
	"MT6591M",
	"MT6592",
	"MT6592M",
	"unknow"
};

static ssize_t cpu_type_status(struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
	ssize_t rc;
	unsigned int cpu;
	unsigned int n_cpu=0;
	struct cpufreq_policy *policy;
	unsigned cpu_type_id = 0;

	for_each_possible_cpu(cpu)
		n_cpu++;

	CPU_DEBUG("n_cpu=%d\n", n_cpu);
	policy = cpufreq_cpu_get(0);
	CPU_DEBUG("policy=%p\n", policy);

	if(n_cpu == 6)
	{
#if 0
		if(policy->cpuinfo.max_freq == 1495000)
			cpu_type_id = 0;
		else
			cpu_type_id = ARRAY_SIZE(cpu_type) - 1;
#endif
		cpu_type_id = 0;
	} else if (n_cpu == 8) {
		cpu_type_id = 2;
	} else
		cpu_type_id =  ARRAY_SIZE(cpu_type) - 1;
	cpufreq_cpu_put(policy);

	CPU_DEBUG("cpu_type_id=%d, %s\n", cpu_type_id, cpu_type[cpu_type_id]);
	rc = snprintf(buf, 200, "%s\n", cpu_type[cpu_type_id]);
	return rc;
}

static struct kobj_attribute cpu_attr = {
	.attr = {
		.name = "cpu_type",
		.mode = 0444,
	},
	.show = cpu_type_status,
	.store = NULL,
};

static struct attribute *cpu_attrs[] = {
	&cpu_attr.attr,
	NULL,
};

static struct attribute_group cpu_attr_group = {
	.name = "cpu",
	.attrs = cpu_attrs,
};

static int __init rgk_board_cpu_init(void)
{
	rgk_board_properties(&cpu_attr_group);
	return 0;
}

static void __exit rgk_board_cpu_exit(void)
{

}

module_init(rgk_board_cpu_init);
module_exit(rgk_board_cpu_exit);
