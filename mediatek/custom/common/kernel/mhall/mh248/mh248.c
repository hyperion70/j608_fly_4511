/******************************************************************************

  Copyright (C), 2010-2012, mh248.

 ******************************************************************************
Filename      : mh248.c
Version       : R1.0
Aurthor       : lisong
Creattime     : 2013.11.1
Description   : Driver for mh248 hall.

 ******************************************************************************/



#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/input.h>
#include <linux/platform_device.h>

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <mach/mt_gpio.h>
#include <mach/eint.h>


#include "cust_gpio_usage.h"






#define MH248_MHALL_DEBUG		0
#if MH248_MHALL_DEBUG
#define MHALL_TAG                  "[mh248_mhall] "
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

#if 1//lisong 2013-11-15 [NO BUGID][add mhall fd for cit]start
static int mhall_flag = 1;
#endif//lisong 2013-11-15 [NO BUGID][add mhall fd for cit]end

static DECLARE_WAIT_QUEUE_HEAD(mhall_thread_wq);


static atomic_t mhall_wakeup_flag = ATOMIC_INIT(0);
static void mh248_mhall_eint_interrupt_handler(void)
{
    MHALL_FUN();
    atomic_set(&mhall_wakeup_flag, 1);
    wake_up(&mhall_thread_wq);
} 

static void mh248_mhall_eint_config_polarity(u8 polarity)
{	
	mt_set_gpio_mode(GPIO_MHALL_EINT_PIN, GPIO_MHALL_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_MHALL_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, GPIO_PULL_DISABLE);

    //mt_eint_set_sens(CUST_EINT_MHALL_NUM, MT_LEVEL_SENSITIVE);
	mt_eint_set_hw_debounce(CUST_EINT_MHALL_NUM, CUST_EINT_MHALL_DEBOUNCE_CN);
    mt_eint_registration(CUST_EINT_MHALL_NUM, polarity, mh248_mhall_eint_interrupt_handler, 0);
	mt_eint_unmask(CUST_EINT_MHALL_NUM);
}

static int mhall_thread_kthread(void *x)
{
    while(1)
    {
	    s32	mhall_value;
        wait_event(mhall_thread_wq, atomic_read(&mhall_wakeup_flag));
        atomic_set(&mhall_wakeup_flag, 0);

		mhall_value = mt_get_gpio_in(GPIO_MHALL_EINT_PIN);
		if(mhall_value  == 1 ) {		
		   		input_report_key(mhall_input_dev, KEY_MHALL_UP, 1);
				input_sync(mhall_input_dev);
				msleep(10);
		   		input_report_key(mhall_input_dev, KEY_MHALL_UP, 0);
				input_sync(mhall_input_dev);
                
                mt_eint_set_polarity(CUST_EINT_MHALL_NUM, MT_EINT_POL_NEG);
                mt_eint_unmask(CUST_EINT_MHALL_NUM);
#if 1//lisong 2013-11-15 [NO BUGID][add mhall fd for cit]start
                mhall_flag = 1;
#endif//lisong 2013-11-15 [NO BUGID][add mhall fd for cit]end
		} else {
		   		input_report_key(mhall_input_dev, KEY_MHALL_DOWN, 1);
				input_sync(mhall_input_dev);
				msleep(10);
		   		input_report_key(mhall_input_dev, KEY_MHALL_DOWN, 0);
				input_sync(mhall_input_dev);
                
                mt_eint_set_polarity(CUST_EINT_MHALL_NUM, MT_EINT_POL_POS);
                mt_eint_unmask(CUST_EINT_MHALL_NUM);
#if 1//lisong 2013-11-15 [NO BUGID][add mhall fd for cit]start
                mhall_flag = 0;
#endif//lisong 2013-11-15 [NO BUGID][add mhall fd for cit]end                
		}     
    }
}


static int mh248_mhall_probe(struct platform_device *pdev) 
{
	int err = 0;
	
	MHALL_FUN();

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

    mh248_mhall_eint_config_polarity(CUST_EINT_MHALL_TYPE); 

	MHALL_LOG("%s: OK\n", __func__);    

	return 0;

exit:
	MHALL_ERR("%s: err = %d\n", __func__, err);        
	return err;
}

static int mh248_mhall_remove(struct platform_device *pdev)
{ 
	MHALL_FUN();    
    
    input_unregister_device(mhall_input_dev);   
 
	return 0;
}


static struct platform_driver mh248_mhall_driver = {
	.probe      = mh248_mhall_probe,
	.remove     = mh248_mhall_remove,    
	.driver     = {
		.name  = "mhall",
        .owner = THIS_MODULE,
	}
};

#if 1//lisong 2013-11-15 [NO BUGID][add mhall fd for cit]start
static ssize_t show_mhall_key(struct device_driver *ddri, char *buf, size_t count)
{
    return sprintf(buf, "%d\n", mhall_flag);
}

static DRIVER_ATTR(mhall_key, 0777, show_mhall_key, NULL);

#endif//lisong 2013-11-15 [NO BUGID][add mhall fd for cit]end

static int __init mh248_mhall_init(void)
{
	MHALL_FUN();

	if(platform_driver_register(&mh248_mhall_driver))
	{
		MHALL_ERR("failed to register driver");
		return -ENODEV;
	}
#if 1//lisong 2013-11-15 [NO BUGID][add mhall fd for cit]start
    if(driver_create_file(&mh248_mhall_driver.driver, &driver_attr_mhall_key))
    {
        MHALL_ERR("driver_create_file driver_attr_mhall_key failed!!\n");
        return -ENODEV;
    }
#endif//lisong 2013-11-15 [NO BUGID][add mhall fd for cit]end
    kthread_run(mhall_thread_kthread, NULL, "mhall_thread_kthread"); 

	return 0;    
}

static void __exit mh248_mhall_exit(void)
{
	MHALL_FUN();
    
	platform_driver_unregister(&mh248_mhall_driver);
}

module_init(mh248_mhall_init);
module_exit(mh248_mhall_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("mh248_mhall platform driver");
MODULE_AUTHOR("song.li@ragentek.com");
