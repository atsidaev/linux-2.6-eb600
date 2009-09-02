/*
 * Jz OnChip Real Time Clock interface for Linux
 *
 * NOTE: we need to wait rtc write ready before read or write RTC registers.
 *
 */

#define DEBUG

#include <linux/autoconf.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/ioport.h>
#include <linux/fcntl.h>
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/platform_device.h>

#include <linux/rtc.h>			/* get the user-level API */
#include <asm/system.h>
#include <asm/jzsoc.h>

#include "rtc-jz.h"

extern spinlock_t rtc_lock;

static int jz_rtc_get_time(struct device *dev, struct rtc_time *rtc_tm)
{
	unsigned long lval, lval_old;
	struct rtc_time ltm;
	unsigned long flags;

	spin_lock_irqsave(&rtc_lock, flags);
	while (!__rtc_write_ready());

	lval = REG_RTC_RSR;
	do {
		lval_old = lval;
		lval = REG_RTC_RSR;
	} while (lval_old != lval);

	spin_unlock_irqrestore(&rtc_lock, flags);

	pr_debug("read time: %lu\n", lval);

	rtc_time_to_tm(lval, &ltm);
	if(rtc_valid_tm(&ltm) == 0) {
		/* is valid */
		rtc_tm->tm_sec = ltm.tm_sec;
		rtc_tm->tm_min = ltm.tm_min;
		rtc_tm->tm_hour = ltm.tm_hour;
		rtc_tm->tm_mday = ltm.tm_mday;
		rtc_tm->tm_wday = ltm.tm_wday;
		rtc_tm->tm_mon = ltm.tm_mon;
		rtc_tm->tm_year = ltm.tm_year;
	} else {
		printk("invlaid data / time!\n");
	}

	return 0;
}

static int jz_rtc_set_time(struct device *dev, struct rtc_time *rtc_tm)
{
	unsigned long lval;
	unsigned long flags;

	spin_lock_irqsave(&rtc_lock, flags);
	rtc_tm_to_time(rtc_tm, &lval);

	while ( !__rtc_write_ready() ) ;

	REG_RTC_RSR = lval;
	spin_unlock_irqrestore(&rtc_lock, flags);

	return 0;
}

static unsigned int get_rcr(void)
{
	unsigned int rcr;
	unsigned long flags;

	spin_lock_irqsave(&rtc_lock, flags);

	while ( !__rtc_write_ready() ) ;
	rcr = REG_RTC_RCR;

	spin_unlock_irqrestore(&rtc_lock, flags);
	return rcr;
}

static void mask_rtc_irq_bit( int bit )
{
	unsigned long flags;

	spin_lock_irqsave(&rtc_lock, flags);

	while ( !__rtc_write_ready() ) ;
	REG_RTC_RCR &= ~(1<<bit);

	spin_unlock_irqrestore(&rtc_lock, flags);
}

static void set_rtc_irq_bit( int bit )
{
	unsigned long flags;

	spin_lock_irqsave(&rtc_lock, flags);

	while ( !__rtc_write_ready() ) ;
	REG_RTC_RCR |= (1<<bit);

	spin_unlock_irqrestore(&rtc_lock, flags);
}

static int jz_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	if (enabled)
		set_rtc_irq_bit(RTC_ALM_EN);
	else
		mask_rtc_irq_bit(RTC_ALM_EN);

	return 0;
}

static int jz_rtc_get_alarm(struct device *dev, struct rtc_wkalrm *alm_tm)
{
	unsigned long lval;
	struct rtc_time altm;
	unsigned long rcr;
	unsigned long flags;

	spin_lock_irqsave(&rtc_lock, flags);
	while ( !__rtc_write_ready() ) ;
	lval = REG_RTC_RSAR;
	rcr = REG_RTC_RCR;
	spin_unlock_irqrestore(&rtc_lock, flags);

	rtc_time_to_tm(lval, &altm);

	if(rtc_valid_tm(&altm) == 0) {
		/* is valid */
		alm_tm->time.tm_sec = altm.tm_sec;
		alm_tm->time.tm_min = altm.tm_min;
		alm_tm->time.tm_hour = altm.tm_hour;
		alm_tm->time.tm_mday = altm.tm_mday;
		alm_tm->time.tm_wday = altm.tm_wday;
		alm_tm->time.tm_mon = altm.tm_mon;
		alm_tm->time.tm_year = altm.tm_year;
	} else {
		dev_err(dev, "invalid data / time\n");
	}

	alm_tm->enabled = !!(rcr & RTC_ALM_EN);


	return 0;
}

static int jz_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm_tm)
{
	unsigned long lval;
	unsigned long flags;

	rtc_tm_to_time(&alm_tm->time, &lval);

	spin_lock_irqsave(&rtc_lock, flags);
	while ( !__rtc_write_ready() ) ;
	REG_RTC_RSAR = lval;

	if (alm_tm->enabled) {
		while ( !__rtc_write_ready() ) ; /* set alarm function */
		if ( !((REG_RTC_RCR>>2) & 0x1) ) {
			while ( !__rtc_write_ready() ) ;
			__rtc_enable_alarm();
		}
//		while ( !__rtc_write_ready() ) ;
//		if ( !(REG_RTC_RCR & RTC_RCR_AIE) ) { /* Enable alarm irq */
//			__rtc_enable_alarm_irq();
//		}
	} else {
		while ( !__rtc_write_ready() ) ;
		__rtc_disable_alarm();
//		while ( !__rtc_write_ready() ) ;
//		__rtc_enable_alarm_irq();
	}
	jz_rtc_alarm_irq_enable(dev, alm_tm->enabled);

	spin_unlock_irqrestore(&rtc_lock, flags);
	return 0;
}

#if 0

static void get_rtc_wakeup_alarm(struct rtc_wkalrm *wkalm)
{
	int enabled, pending;

	get_rtc_alm_time(&wkalm->time);

	spin_lock_irq(&rtc_lock);
	while ( !__rtc_write_ready() ) ;
	enabled = (REG_RTC_HWCR & 0x1);
	pending = 0;
	if ( enabled ) {
		if ( (u32)REG_RTC_RSAR > (u32)REG_RTC_RSR ) /* 32bit val */
			pending = 1;
	}

	wkalm->enabled = enabled;
	wkalm->pending = pending;
	spin_unlock_irq(&rtc_lock);
}

static int set_rtc_wakeup_alarm(struct rtc_wkalrm *wkalm)
{
	int enabled;
	//int pending;

	enabled = wkalm->enabled;
	//pending = wkalm->pending; /* Fix me, what's pending mean??? */

	while ( !__rtc_write_ready() ) ; /* set wakeup alarm enable */
	if ( enabled != (REG_RTC_HWCR & 0x1) ) {
		while ( !__rtc_write_ready() ) ;
		REG_RTC_HWCR = (REG_RTC_HWCR & ~0x1) | enabled;
	}
	while ( !__rtc_write_ready() ) ; /* set alarm function */
	if ( enabled != ((REG_RTC_RCR>>2) & 0x1) ) {
		while ( !__rtc_write_ready() ) ;
		REG_RTC_RCR = (REG_RTC_RCR & ~(1<<2)) | (enabled<<2);
	}

	if ( !enabled )		/* if disabled wkalrm, rturn.  */
	{
		return 0;
	}

	while ( !__rtc_write_ready() ) ;
	if ( !(REG_RTC_RCR & RTC_RCR_AIE) ) { /* Enable alarm irq */
		__rtc_enable_alarm_irq();
	}

	set_rtc_alm_time(&wkalm->time);

	return 0;
}
#endif

static irqreturn_t jz_rtc_interrupt(int irq, void *dev_id)
{
	struct rtc_device *rdev = dev_id;

	REG_RTC_HCR = 0x0;
	printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
	spin_lock(&rtc_lock);

	if ( __rtc_get_1Hz_flag() ) {
		while ( !__rtc_write_ready() ) ;
		__rtc_clear_1Hz_flag();
		pr_debug("RTC 1Hz interrupt occur.\n");
		rtc_update_irq(rdev, 1, RTC_PF | RTC_AF);
	}

	if ( __rtc_get_alarm_flag() ) {	/* rtc alarm interrupt */
		while ( !__rtc_write_ready() ) ;
		__rtc_clear_alarm_flag();
		pr_debug("RTC alarm interrupt occur.\n");
		rtc_update_irq(rdev, 1, RTC_PF | RTC_AF);
	}
	spin_unlock(&rtc_lock);

	return IRQ_HANDLED;
}


static int jz_rtc_set_irq_state(struct device *dev, int enabled)
{
	if (enabled) {
		__rtc_clear_1Hz_flag();
		set_rtc_irq_bit(RTC_1HZIE);
	} else
		mask_rtc_irq_bit(RTC_1HZIE);

	return 0;
}

static int jz_rtc_open(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtc_device *rtc_dev = platform_get_drvdata(pdev);
	int ret;

	ret = request_irq(IRQ_RTC, jz_rtc_interrupt, IRQF_DISABLED, "jz-rtc", rtc_dev);
	if (ret) {
		dev_err(dev, "IRQ registering error\n");
		return ret;
	}

	return ret;
}

static void jz_rtc_release(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtc_device *rtc_dev = platform_get_drvdata(pdev);

	free_irq(IRQ_RTC, rtc_dev);
}

static const struct rtc_class_ops jz_rtcops = {
	.open		= jz_rtc_open,
	.release	= jz_rtc_release,
	.read_time	= jz_rtc_get_time,
	.set_time	= jz_rtc_set_time,
	.read_alarm	= jz_rtc_get_alarm,
	.set_alarm	= jz_rtc_set_alarm,
	.irq_set_state	= jz_rtc_set_irq_state,
	.alarm_irq_enable = jz_rtc_alarm_irq_enable,
};


static int __init jz_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;
	int ret;

	/* Enabled rtc function, enable rtc alarm function */
	while ( !__rtc_write_ready() ) ; /* need we wait for WRDY??? */
	if ( !(REG_RTC_RCR & RTC_RCR_RTCE) || !(REG_RTC_RCR &RTC_RCR_AE) ) {
		REG_RTC_RCR |= RTC_RCR_AE | RTC_RCR_RTCE;
	}
	/* clear irq flags */
	mask_rtc_irq_bit(RTC_1HZIE);
	__rtc_clear_1Hz_flag();
	/* In a alarm reset, we expect a alarm interrupt. 
	 * We can do something in the interrupt handler.
	 * So, do not clear alarm flag.
	 */
	__rtc_clear_alarm_flag();

	device_init_wakeup(&pdev->dev, 1);

	rtc = rtc_device_register("jz", &pdev->dev, &jz_rtcops, THIS_MODULE);

	if (IS_ERR(rtc)) {
		dev_err(&pdev->dev, "cannot attach rtc\n");
		ret = PTR_ERR(rtc);
		return ret;
	}

	rtc->max_user_freq = 1;

	platform_set_drvdata(pdev, rtc);

	printk("JzSOC onchip RTC has been installed\n");

	return 0;
}

static int __exit jz_rtc_remove(struct platform_device *dev)
{
	struct rtc_device *rtc = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);
	rtc_device_unregister(rtc);

	return 0;
}

#if CONFIG_PM

static int int_save;

static int jz_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	int_save = get_rcr() & (1 << RTC_1HZIE);

	mask_rtc_irq_bit(RTC_1HZIE);
	__rtc_clear_alarm_flag();

	return 0;
}

static int jz_rtc_resume(struct platform_device *pdev)
{
	if (int_save)
		set_rtc_irq_bit(RTC_1HZIE);
	else
		mask_rtc_irq_bit(RTC_1HZIE);

	return 0;
}

#else
#define jz_rtc_suspend NULL
#define jz_rtc_resume NULL
#endif

static struct platform_driver jz_rtc_driver = {
	.probe		= jz_rtc_probe,
	.remove		= __devexit_p(jz_rtc_remove),
	.suspend	= jz_rtc_suspend,
	.resume		= jz_rtc_resume,
	.driver		= {
		.name	= "jz-rtc",
		.owner	= THIS_MODULE,
	},
};

static int __init jz_rtc_init(void)
{
	return platform_driver_register(&jz_rtc_driver);
}

static void __exit jz_rtc_exit(void)
{
	platform_driver_unregister(&jz_rtc_driver);
}

module_init(jz_rtc_init);
module_exit(jz_rtc_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yauhen Kharuzhy <jekhor@gmail.com>");
MODULE_DESCRIPTION("RTC driver for JZ47x0 SOCs");
