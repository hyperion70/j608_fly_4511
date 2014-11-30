/* mhall motion sensor driver
 *auth:fangxing 
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
//#include <mach/mt_gpio.h>


#ifdef MT6575
#include <mach/mt6575_devs.h>
#include <mach/mt6575_typedefs.h>
#include <mach/mt6575_gpio.h>
#include <mach/mt6575_pm_ldo.h>
#endif
#include <cust_eint.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <mach/mt_gpio.h>
#include "cust_gpio_usage.h"
#include <linux/kthread.h>
#include <mtk_kpd.h>

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <linux/hwmsen_helper.h>


//#define GPIO_ALS_EINT_PIN         GPIO17
//#define GPIO_ALS_EINT_PIN_M_GPIO  GPIO_MODE_00
//#define GPIO_ALS_EINT_PIN_M_EINT  GPIO_MODE_01
//#define GPIO_ALS_EINT_PIN_M_PWM  GPIO_MODE_04
//#define CUST_EINT_ALS_NUM              3
//#define CUST_EINT_ALS_DEBOUNCE_CN      0mt_eint_set_sens
//#define CUST_EINT_ALS_POLARITY         CUST_EINT_POLARITY_LOW
#define CUST_EINT_MHALL_SENSITIVE        1
#define CUST_EINT_MHALL_DEBOUNCE_CN      64

/*----------------------------------------------------------------------------*/
#define OCH165TWAD_MHALL_DEBUG		1
#if OCH165TWAD_MHALL_DEBUG
#define MHALL_TAG                  "[och165twad_mhall] "
#define MHALL_FUN(f)               printk(KERN_ERR MHALL_TAG"%s\n", __FUNCTION__)
#define MHALL_ERR(fmt, args...)    printk(KERN_ERR MHALL_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define MHALL_LOG(fmt, args...)    printk(KERN_ERR MHALL_TAG fmt, ##args)
#else
#define MHALL_FUN(f)               	 do{}while(0)
#define MHALL_ERR(fmt, args...)    do{}while(0)
#define MHALL_LOG(fmt, args...)    do{}while(0)
#endif


#define MHALL_NAME	"mhall_key"
static struct input_dev *mhall_input_dev;


struct och165twad_mhall_data {
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};

static DECLARE_WAIT_QUEUE_HEAD(mhall_thread_wq);


static atomic_t mhall_wakeup_flag = ATOMIC_INIT(0);
static void och165twad_mhall_eint_interrupt_handler(void)
{
	MHALL_ERR("%s: fangxing  \n",__func__);
    atomic_set(&mhall_wakeup_flag, 1);
    wake_up_interruptible(&mhall_thread_wq);
} 
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol); 
extern void mt_eint_unmask(unsigned int line);
extern void mt_eint_mask(unsigned int line);
extern void mt_eint_set_hw_debounce(unsigned int eintno, unsigned int ms);
extern unsigned int mt_eint_set_sens(unsigned int eintno, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flag, void (EINT_FUNC_PTR) (void), unsigned int is_auto_umask);

static void och165twad_mhall_eint_config_polarity(u8 polarity)
{
	MHALL_ERR("%s: penny   \n", __func__);
	mt_set_gpio_mode(GPIO_MHALL_EINT_PIN, GPIO_MHALL_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_MHALL_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, GPIO_PULL_DISABLE);

	mt_eint_set_sens(CUST_EINT_MHALL_NUM, CUST_EINT_MHALL_SENSITIVE);
	mt_eint_set_hw_debounce(CUST_EINT_MHALL_NUM, CUST_EINT_MHALL_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_MHALL_NUM, CUST_EINT_MHALL_TYPE, och165twad_mhall_eint_interrupt_handler, 0); 
	mt_eint_unmask(CUST_EINT_MHALL_NUM);
}

//add by lihua BUG_ID:NULL Description: add node 2013-10-29 (start)
static int mhall_flag = 1;
//add by lihua BUG_ID:NULL Description: add node 2013-10-29 (end)
static int mhall_thread_kthread(void *x)
{
    while(1)
    {
	  s32	mhall_value;
        wait_event_interruptible(mhall_thread_wq, atomic_read(&mhall_wakeup_flag));
        atomic_set(&mhall_wakeup_flag, 0);
#if 0
        kpd_pwrkey_pmic_handler(1);
    	msleep(100);
    	kpd_pwrkey_pmic_handler(0);
#endif
//		mt_set_gpio_mode(GPIO_MHALL_EINT_PIN, GPIO_MHALL_EINT_PIN_M_GPIO);
//		mt_set_gpio_dir(GPIO_MHALL_EINT_PIN, GPIO_DIR_IN);
//		mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, GPIO_PULL_DISABLE);
		mhall_value = mt_get_gpio_in(GPIO_MHALL_EINT_PIN);
		MHALL_ERR("%s: fangxing  mhall_value = %d \n",__func__,mhall_value);
		if(mhall_value  == 1 ) {		
		   		input_report_key(mhall_input_dev, KEY_MHALL_UP, 1);
				input_sync(mhall_input_dev);
				msleep(10);
		   		input_report_key(mhall_input_dev, KEY_MHALL_UP, 0);
				input_sync(mhall_input_dev);
//				mt_eint_mask(CUST_EINT_MHALL_NUM);
//				och165twad_mhall_eint_config_polarity(CUST_EINT_POLARITY_LOW); 

                		mt_eint_set_polarity(CUST_EINT_MHALL_NUM, MT_POLARITY_LOW);
                		mt_eint_unmask(CUST_EINT_MHALL_NUM);
                		mhall_flag = 1;//add by lihua BUG_ID:NULL Description: add node 2013-10-29

		} else {
		   		input_report_key(mhall_input_dev, KEY_MHALL_DOWN, 1);
				input_sync(mhall_input_dev);
				msleep(10);
		   		input_report_key(mhall_input_dev, KEY_MHALL_DOWN, 0);
				input_sync(mhall_input_dev);
//				mt_eint_mask(CUST_EINT_MHALL_NUM);
//				och165twad_mhall_eint_config_polarity(CUST_EINT_POLARITY_HIGH); 
                		mt_eint_set_polarity(CUST_EINT_MHALL_NUM, MT_POLARITY_HIGH);
                		mt_eint_unmask(CUST_EINT_MHALL_NUM);
                		mhall_flag = 0;//add by lihua BUG_ID:NULL Description: add node 2013-10-29
		}
    }
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void och165twad_mhall_early_suspend(struct early_suspend *h) 
{
	MHALL_FUN();  
//	mt_eint_mask(CUST_EINT_MHALL_NUM);
//	och165twad_mhall_eint_config_polarity(CUST_EINT_POLARITY_HIGH); 
}
/*----------------------------------------------------------------------------*/
static void och165twad_mhall_late_resume(struct early_suspend *h)
{
	MHALL_FUN();
//	mt_eint_mask(CUST_EINT_MHALL_NUM);
//	och165twad_mhall_eint_config_polarity(CUST_EINT_POLARITY_LOW); 
}
#endif

static bool mhall = 0;
core_param(mhall, mhall, bool, 0444);
static bool pull_u = 0;
core_param(pull_u, pull_u, bool, 0444);
static bool pull_d = 0;
core_param(pull_d, pull_d, bool, 0444);

static int och165twad_mhall_probe(struct platform_device *pdev) 
{
	struct och165twad_mhall_data *obj;
	int err = 0;
	
	MHALL_FUN();
	
	mt_set_gpio_mode(GPIO_MHALL_EINT_PIN, GPIO_MHALL_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_MHALL_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, GPIO_PULL_DISABLE);
	mt_set_gpio_pull_select(GPIO_MHALL_EINT_PIN, GPIO_PULL_DOWN);
	mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, GPIO_PULL_ENABLE);
	if((pull_d = mt_get_gpio_in(GPIO_MHALL_EINT_PIN)) != 0)
		mhall = 1;
	
	mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, GPIO_PULL_DISABLE);
	mt_set_gpio_pull_select(GPIO_MHALL_EINT_PIN, GPIO_PULL_UP);
	mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, GPIO_PULL_ENABLE);
	if((pull_u = mt_get_gpio_in(GPIO_MHALL_EINT_PIN)) == 0)
		mhall = 1;

	mt_set_gpio_pull_select(GPIO_MHALL_EINT_PIN, GPIO_PULL_DOWN);
	mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, GPIO_PULL_DISABLE);
//	if(mhall == 0)
//		return -ENODEV;

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = och165twad_mhall_early_suspend,
	obj->early_drv.resume   = och165twad_mhall_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif 

	platform_set_drvdata(pdev, obj);

	/* initialize and register input device (/dev/input/eventX) */
	mhall_input_dev = input_allocate_device();
	if (!mhall_input_dev) {
		err = -ENOMEM;
		goto exit;
	}
	mhall_input_dev->name = MHALL_NAME;

	__set_bit(EV_KEY, mhall_input_dev->evbit);
	__set_bit(KEY_MHALL_DOWN, mhall_input_dev->keybit);
	__set_bit(KEY_MHALL_UP, mhall_input_dev->keybit);
		
	err = input_register_device(mhall_input_dev);
	if (err) {
		MHALL_ERR( "register mhall_input_dev input device failed (%d)\n", err);
		input_free_device(mhall_input_dev);
		goto exit;
	}

    msleep(5);

    och165twad_mhall_eint_config_polarity(0); 


	MHALL_LOG("%s: OK\n", __func__);    

	return 0;

exit:
	MHALL_ERR("%s: err = %d\n", __func__, err);        
	return err;
}
/*----------------------------------------------------------------------------*/
static int och165twad_mhall_remove(struct platform_device *pdev)
{
	struct och165twad_mhall_data *obj = platform_get_drvdata(pdev);
	MHALL_FUN();    
    input_unregister_device(mhall_input_dev);   
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&obj->early_drv);
#endif   
	if(obj)
		kfree(obj);

	return 0;
}


/*----------------------------------------------------------------------------*/
static struct platform_driver och165twad_mhall_driver = {
	.probe      = och165twad_mhall_probe,
	.remove     = och165twad_mhall_remove,    
	.driver     = {
		.name  = "mhall",
        .owner = THIS_MODULE,
	}
};

#if 0//add by lisong for test
/*----------------------------------------------------------------------------*/
static ssize_t store_power_key(struct device_driver *ddri, char *buf, size_t count)
{
    GSE_FUN();
    kpd_pwrkey_pmic_handler(1);
	msleep(100);
	kpd_pwrkey_pmic_handler(0);
    
    return count;
}


static DRIVER_ATTR(power_key, 0777, NULL, store_power_key );
#endif
//add by lihua BUG_ID:NULL Description: add node 2013-10-29 (start)

static ssize_t show_mhall_key(struct device_driver *ddri, char *buf, size_t count)
{
    return sprintf(buf, "%d\n", mhall_flag);
}


static DRIVER_ATTR(mhall_key, 0444, show_mhall_key, NULL);

static ssize_t show_mhall_gpio(struct device_driver *ddri, char *buf, size_t count)
{
	int gpio_up, gpio_down;
	mt_set_gpio_pull_select(GPIO_MHALL_EINT_PIN, GPIO_PULL_DOWN);
	mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, GPIO_PULL_ENABLE);
	gpio_down = mt_get_gpio_in(GPIO_MHALL_EINT_PIN);
	
	mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, GPIO_PULL_DISABLE);
	mt_set_gpio_pull_select(GPIO_MHALL_EINT_PIN, GPIO_PULL_UP);
	mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, GPIO_PULL_ENABLE);
	gpio_up = mt_get_gpio_in(GPIO_MHALL_EINT_PIN);

	mt_set_gpio_pull_select(GPIO_MHALL_EINT_PIN, GPIO_PULL_DOWN);
	mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, GPIO_PULL_DISABLE);
	
 	return sprintf(buf, "up:%d, down:%d\n", gpio_up, gpio_down);
}


static DRIVER_ATTR(mhall_gpio, 0444, show_mhall_gpio, NULL);

//add by lihua BUG_ID:NULL Description: add node 2013-10-29 (end)
static int __init och165twad_mhall_init(void)
{
	MHALL_FUN();

	if(platform_driver_register(&och165twad_mhall_driver))
	{
		MHALL_ERR("failed to register driver");
		return -ENODEV;
	}

    kthread_run(mhall_thread_kthread, NULL, "mhall_thread_kthread"); 
    //add by lihua BUG_ID:NULL Description: add node 2013-10-29 (start)
    if(driver_create_file(&och165twad_mhall_driver.driver, &driver_attr_mhall_key))
    {
        MHALL_ERR("driver_create_file driver_attr_mhall_key failed!!\n");
        return -ENODEV;
    }
	driver_create_file(&och165twad_mhall_driver.driver, &driver_attr_mhall_gpio);
    //add by lihua BUG_ID:NULL Description: add node 2013-10-29 (end)
#if 0//add by lisong for test
    if(driver_create_file(&och165twad_driver.driver, &driver_attr_power_key))
	{            
		GSE_ERR("driver_create_file driver_attr_power_key failed!!\n");
		return -ENODEV;
	}
#endif
	return 0;    
}
/*----------------------------------------------------------------------------*/
static void __exit och165twad_mhall_exit(void)
{
	MHALL_FUN();
	platform_driver_unregister(&och165twad_mhall_driver);
}
/*----------------------------------------------------------------------------*/
module_init(och165twad_mhall_init);
module_exit(och165twad_mhall_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("och165twad_mhall platform driver");
MODULE_AUTHOR("xing.fang@ragentek.com");
