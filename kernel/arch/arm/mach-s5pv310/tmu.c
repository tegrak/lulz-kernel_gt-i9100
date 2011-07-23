/* linux/arch/arm/mach-s5pv310/tmu.c

* Copyright (c) 2010 Samsung Electronics Co., Ltd.
*      http://www.samsung.com/
*
* S5PV310 - TMU driver
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/slab.h>

#include <asm/irq.h>

#include <mach/regs-tmu.h>
#include <mach/cpufreq.h>
#include <mach/map.h>
#include <plat/s5p-tmu.h>
#include <plat/gpio-cfg.h>

/* Support Bootloader parameter setting by SBL interface */
#undef CONFIG_TMU_DEBUG_ENABLE
//#define CONFIG_TMU_DEBUG_ENABLE

/* Selectable one room temperature among 3 kinds */
#define  OPERATION_TEMP_BASE_78
//#define OPERATION_TEMP_BASE_35
//#define OPERATION_TEMP_BASE_40

#define TIMMING_AREF 0x30
#define AUTO_REFRESH_PERIOD_TQ0		0x2E /* auto refresh preiod 1.95us */
#define AUTO_REFRESH_PERIOD_NORMAL	0x5D /* auto refresh period 3.9us */

/*
 *  For example
 *  test value for room temperature
 *  base operation temp : OPERATION_TEMP_BASE_78
*/

#ifdef OPERATION_TEMP_BASE_78
/* TMU register setting value */
#define THD_TEMP 0x80	  /* 78 degree  : threshold temp */
#define TRIGGER_LEV0 0x9  /* 87 degree  : throttling temperature */
#define	TRIGGER_LEV1 0x19 /* 103 degree  : Waring temperature */
#define TRIGGER_LEV2 0x20 /* 110 degree : Tripping temperature  */
#define	TRIGGER_LEV3 0xFF /* Reserved */

/* interrupt level by celcius degree */
#define TEMP_MIN_CELCIUS	 25
#define TEMP_TROTTLED_CELCIUS	 87
#define TEMP_TQ0_CELCIUS	 90
#define TEMP_WARNING_CELCIUS	103
#define TEMP_TRIPPED_CELCIUS	110
#define TEMP_MAX_CELCIUS	125
#endif

#ifdef OPERATION_TEMP_BASE_35
/* test on 35 celsius base */
#define THD_TEMP     0x55 /* 35 degree: thershold temp */
#define TRIGGER_LEV0 0x5  /* 40	degree: Throttling temperature */
#define TRIGGER_LEV1 0xA  /* 45	degree: Waring temperature */
#define TRIGGER_LEV2 0xF  /* 50	degree: Tripping temperature */
#define TRIGGER_LEV3 0xFF /* Reserved */
#define TEMP_TROTTLED_CELCIUS	 40
#define TEMP_WARNING_CELCIUS	 45
#define TEMP_TQ0_CELCIUS	 48
#define TEMP_TRIPPED_CELCIUS	 50
#define TEMP_MIN_CELCIUS	 25
#define TEMP_MAX_CELCIUS	125
#endif

#ifdef OPERATION_TEMP_BASE_40
/* test on 40 celsius base */
#define THD_TEMP     0x5A /* 40 degree: thershold temp */
#define TRIGGER_LEV0 0x5  /* 45	degree: Throttling temperature */
#define TRIGGER_LEV1 0xA  /* 50	degree: Waring temperature */
#define TRIGGER_LEV2 0xF  /* 55	degree: Tripping temperature */
#define TRIGGER_LEV3 0xFF /* Reserved */
#define TEMP_TROTTLED_CELCIUS	 45
#define TEMP_WARNING_CELCIUS	 50
#define TEMP_TQ0_CELCIUS	 52
#define TEMP_TRIPPED_CELCIUS	 55
#define TEMP_MIN_CELCIUS	 25
#define TEMP_MAX_CELCIUS	125
#endif

#define TMU_SAVE_NUM   10
#define VREF_SLOPE     0x07000F02
#define TMU_EN         0x1
#define TMU_DC_VALUE   25
#define TMU_CODE_25_DEGREE 0x4B
#define EFUSE_MIN_VALUE 60
#define EFUSE_AVG_VALUE 80
#define EFUSE_MAX_VALUE 100
#define FIN	24*1000*1000

static struct workqueue_struct  *tmu_monitor_wq;
static struct resource *s5p_tmu_mem;
static int irq_tmu = NO_IRQ;
unsigned int tmu_save[TMU_SAVE_NUM];

enum tmu_status_t {
	TMU_STATUS_NORMAL = 0,
	TMU_STATUS_THROTTLED,
	TMU_STATUS_WARNING,
	TMU_STATUS_TRIPPED,
	TMU_STATUS_INIT,
};

struct tmu_data_band {
	int thr_low;
	int thr_high;
	int warn_low;
	int warn_high;
	int trip_retry;
	int tq0_temp;
};

struct tmu_data_band tmu_temp_band = {
	.thr_low	= TEMP_TROTTLED_CELCIUS	- 4,	/*  83 : low temp of throttling */
	.thr_high	= TEMP_WARNING_CELCIUS	- 5,	/*  90 : hith temp of warning */
	.warn_low	= TEMP_WARNING_CELCIUS	- 6,	/*  97 : low temp of warning */
	.warn_high	= TEMP_WARNING_CELCIUS	+ 3,	/* 105 : high temp of warning */
	.trip_retry	= TEMP_TRIPPED_CELCIUS  + 3,    /* 113 : trip re-try */
	.tq0_temp	= TEMP_TQ0_CELCIUS,    /* 90: tq0 temp */   
};

static DEFINE_MUTEX(tmu_lock);

struct s5p_tmu_info {
	struct device *dev;

	char *tmu_name;
	struct tmu_data_band tmu_a;
	struct s5p_tmu *ctz;

	struct tmu_data_band *tmu_data;
	int thr_low;
	int thr_high;
	int warn_low;
	int warn_high;
	int trip_retry;
	int tq0_temp;

	struct delayed_work monitor_work;
	struct delayed_work polling_work;

	unsigned int monitor_period;
	unsigned int sampling_rate;

	int tmu_status;
};
struct s5p_tmu_info *tmu_info;

#ifdef CONFIG_TMU_DEBUG_ENABLE
static int irq_tq0;
static struct delayed_work tq0_polling_work;

static int set_tmu_test		= 0;
static int set_thr_stop		= 0;
static int set_thr_temp		= 0;
static int set_warn_stop	= 0;
static int set_warn_temp	= 0;
static int set_trip_temp	= 0;
static int set_tq0_temp	= 0;

static int set_sampling_rate	= 0;
static int set_cpu_level	= 3;
static int set_tq0_enable	= 0;
static int set_tq0_mode	= 1;
static int set_tq0_period	= 1000;

static int __init tmu_test_param(char *str)
{
	int tmu_temp[7];

	get_options(str, 7, tmu_temp);

	set_tmu_test	= tmu_temp[0];
	set_thr_stop	= tmu_temp[1];
	set_thr_temp	= tmu_temp[2];
	set_warn_stop   = tmu_temp[3];
	set_warn_temp	= tmu_temp[4];
	set_trip_temp	= tmu_temp[5];
	set_tq0_temp	= tmu_temp[6];

	return 0;
}
early_param("tmu_test", tmu_test_param);

static int __init tq0_param(char *str)
{
	int tq0_temp[3];

	get_options(str, 3, tq0_temp);

	set_tq0_enable	= tq0_temp[0];
	set_tq0_mode	= tq0_temp[1];
	set_tq0_period	= tq0_temp[2];
	
	return 0;
}
early_param("tq0_mode", tq0_param);

static int __init limit_param(char *str)
{
	get_option(&str, &set_cpu_level);
	if (set_cpu_level < 0)
		set_cpu_level = 0;

	return 0;
}
early_param("max_cpu_level", limit_param);

static int __init sampling_rate_param(char *str)
{
	get_option(&str, &set_sampling_rate);
	if (set_sampling_rate < 0)
		set_sampling_rate = 0;

	return 0;
}
early_param("tmu_sampling_rate", sampling_rate_param);

static void tmu_start_testmode(struct platform_device *pdev)
{
	struct s5p_tmu *tz = platform_get_drvdata(pdev);
	unsigned int con;
	unsigned int thresh_temp_adc, thr_temp_adc, trip_temp_adc;
	unsigned int warn_temp_adc = 0xFF;

	/* Compensation temperature THD_TEMP */
	thresh_temp_adc	= set_thr_stop + tz->data.te1 - TMU_DC_VALUE;
	thr_temp_adc	= set_thr_temp + tz->data.te1 - TMU_DC_VALUE - thresh_temp_adc;
	warn_temp_adc   = set_warn_temp + tz->data.te1 - TMU_DC_VALUE - thresh_temp_adc;

	trip_temp_adc	= set_trip_temp + tz->data.te1 - TMU_DC_VALUE - thresh_temp_adc;
	printk(KERN_INFO "Compensated Threshold: 0x%2x\n", thresh_temp_adc);
	
	/* Set interrupt trigger level */
	__raw_writel(thresh_temp_adc, tz->tmu_base + THRESHOLD_TEMP);
	__raw_writel(thr_temp_adc, tz->tmu_base + TRG_LEV0);
	__raw_writel(warn_temp_adc, tz->tmu_base + TRG_LEV1);
	__raw_writel(trip_temp_adc, tz->tmu_base + TRG_LEV2);
	__raw_writel(TRIGGER_LEV3, tz->tmu_base + TRG_LEV3);

	pr_info("Cooling: %dc  THD_TEMP:0x%02x:  TRIG_LEV0: 0x%02x\
		TRIG_LEV1: 0x%02x TRIG_LEV2: 0x%02x, TRIG_LEV3: 0x%02x\n",
		set_thr_stop,
		__raw_readl(tz->tmu_base + THRESHOLD_TEMP),
		__raw_readl(tz->tmu_base + TRG_LEV0),
		__raw_readl(tz->tmu_base + TRG_LEV1),
		__raw_readl(tz->tmu_base + TRG_LEV2),
		__raw_readl(tz->tmu_base + TRG_LEV3));

	mdelay(50);    
	/* TMU core enable */
	con = __raw_readl(tz->tmu_base + TMU_CON0);
	con |= TMU_EN;

	__raw_writel(con, tz->tmu_base + TMU_CON0);

	/*LEV0 LEV1 LEV2 interrupt enable */
	__raw_writel(INTEN0 | INTEN1 | INTEN2, tz->tmu_base + INTEN);
	
	return;
}

static void tq0_poll_timer(struct work_struct *work)
{
	unsigned char cur_temp;

	/* Compensation temperature */
	cur_temp = __raw_readb(tmu_info->ctz->tmu_base + CURRENT_TEMP)
			- tmu_info->ctz->data.te1 + TMU_DC_VALUE;

	pr_info("current temp at tq0 interrupt = %d\n", cur_temp);

	/* clear pendig bit */
	//__raw_writel(0x8, S5PV310_VA_GPIO2 + 0xA20);
	enable_irq(irq_tq0);
}
#endif

static void set_refresh_rate(unsigned int auto_refresh)
{
	/*
	 * uRlk = FIN / 100000;
	 * refresh_usec =  (unsigned int)(fMicrosec * 10);
	 * uRegVal = ((unsigned int)(uRlk * uMicroSec / 100)) - 1;
	*/
	/* change auto refresh period of dmc0 */
	__raw_writel(auto_refresh, S5P_VA_DMC0 + TIMMING_AREF);

	/* change auto refresh period of dmc1 */
	__raw_writel(auto_refresh, S5P_VA_DMC1 + TIMMING_AREF);
}

static unsigned int tq0_signal_handle_init(void);

#ifdef CONFIG_TMU_DEBUG_ENABLE
static irqreturn_t tq0_signal_isr(int irq, void *id)
{
	unsigned int status, data;

	/* disable interrupt */
	disable_irq_nosync(irq_tq0);

	/* Read EXT_INT29 pending register */
	status = __raw_readl(S5PV310_VA_GPIO2 + 0xA20);
	printk(KERN_INFO "tq0 status: 0x%02x\n", status);

	/* check tq0 input */
	data = __raw_readl(S5PV310_VA_GPIO2 + 0x104);
	printk(KERN_INFO "tq0 input : 0x%02x\n", data);

	if (data & 0x8) {
		set_refresh_rate(AUTO_REFRESH_PERIOD_TQ0);
		set_irq_type(gpio_to_irq(S5PV310_GPL2(3)), IRQ_TYPE_LEVEL_LOW);
	} else {
		set_refresh_rate(AUTO_REFRESH_PERIOD_NORMAL);
		set_irq_type(gpio_to_irq(S5PV310_GPL2(3)), IRQ_TYPE_LEVEL_HIGH);
	}

	if (set_tq0_period != 2000) {
		/* rescheduling next work */
		queue_delayed_work_on(0, tmu_monitor_wq, &tq0_polling_work,
			usecs_to_jiffies(set_tq0_period * 1000));

	} else {
		/* rescheduling next work */
		queue_delayed_work_on(0, tmu_monitor_wq, &tq0_polling_work,
			tmu_info->sampling_rate * 2);
	}

	/* clear interrupt source */
	__raw_writel(status, S5PV310_VA_GPIO2 + 0xA20);

	return IRQ_HANDLED;
}

static struct irqaction tq0_signal_irq = {
	.name       = "tq0 singal handler",
	.flags      = IRQF_SHARED,
	.handler    = tq0_signal_isr,
};
#endif

static int tmu_tripped_cb(int state)
{
	struct power_supply *psy = power_supply_get_by_name("battery");
	union power_supply_propval value;

	if(!psy) {
		pr_err("%s:fail to get batter ps\n", __func__);
		return -ENODEV;
	}
	printk(KERN_INFO "%s:is pass, state %d.\n", __func__, state);

	switch(state) {
	case TMU_STATUS_NORMAL:
		value.intval = TMU_STATUS_NORMAL;
		break;
	case TMU_STATUS_THROTTLED:
		value.intval = TMU_STATUS_THROTTLED;
		break;
	case TMU_STATUS_WARNING:
		value.intval = TMU_STATUS_WARNING;
		break;
	case TMU_STATUS_TRIPPED:
		value.intval = TMU_STATUS_TRIPPED;
		break;
	default:
		printk(KERN_WARNING "value is not correct.\n");
		return -EINVAL;
	}

	return psy->set_property(psy, POWER_SUPPLY_PROP_HEALTH, &value);
}

#ifdef CONFIG_TMU_DEBUG_ENABLE
static void tmu_mon_timer(struct work_struct *work)
{
	unsigned char cur_temp_adc, cur_temp;

	/* Compensation temperature */
	cur_temp_adc = __raw_readb(tmu_info->ctz->tmu_base + CURRENT_TEMP);
	cur_temp = cur_temp_adc - tmu_info->ctz->data.te1 + TMU_DC_VALUE;

	if (set_tmu_test) {
		pr_info("Current: %d c, Cooling: %d c, Throttling: %d c, Warning: %d c, Tripping: %d c\n",
			cur_temp, tmu_temp_band.thr_low,
			set_thr_temp, set_warn_temp, set_trip_temp);
	} else {
		pr_info("Current: %d c, Cooling: %d c  Throttling: %d c Warning: %d c  Tripping: %d c\n",
			cur_temp, tmu_temp_band.thr_low,
	 		TEMP_TROTTLED_CELCIUS,
			TEMP_WARNING_CELCIUS,
			TEMP_TRIPPED_CELCIUS);
	}
	/* check tq0 input */
	printk(KERN_INFO "tq0 input polling: 0x%02x\n", __raw_readl(S5PV310_VA_GPIO2 + 0x104));

	queue_delayed_work_on(0, tmu_monitor_wq, &tmu_info->monitor_work,
			tmu_info->monitor_period);
}

static void tmu_poll_testmode(void)
{
	unsigned char cur_temp_adc, cur_temp;
	int thr_temp, trip_temp, warn_temp;
	static int cpufreq_limited_thr	= 0;
	static int cpufreq_limited_warn	= 0;
	static int send_msg_battery = 0;
	static int auto_refresh_changed = 0;

	thr_temp  = set_thr_temp;
	warn_temp = set_warn_temp;
	trip_temp = set_trip_temp;

	/* Compensation temperature */
	cur_temp_adc = __raw_readb(tmu_info->ctz->tmu_base + CURRENT_TEMP);
	cur_temp = cur_temp_adc - tmu_info->ctz->data.te1 + TMU_DC_VALUE;

	pr_info("current temp = %d, %d\n", cur_temp, tmu_info->ctz->data.tmu_flag);

	switch (tmu_info->ctz->data.tmu_flag) {
	case TMU_STATUS_NORMAL:
		if (cur_temp <= tmu_temp_band.thr_low) {
			//cancel_delayed_work(&tmu_info->polling_work);
			if (tmu_tripped_cb(TMU_STATUS_NORMAL) < 0) 
				printk(KERN_ERR "Error inform to battery driver !\n");
			else
				printk(KERN_INFO "normal: interrupt enable.\n");
	
			/* To prevent from interrupt by current pending bit */
			__raw_writel(INTCLEARALL, tmu_info->ctz->tmu_base + INTCLEAR);
			enable_irq(irq_tmu);
			return;
		}

		if (cur_temp >= set_thr_temp) { /* 85 */
			tmu_info->ctz->data.tmu_flag = TMU_STATUS_THROTTLED;
			s5pv310_cpufreq_upper_limit(DVFS_LOCK_ID_TMU, CPU_L2); /* CPU_L2 */
			cpufreq_limited_thr = 1;
			if (tmu_tripped_cb(TMU_STATUS_THROTTLED) < 0) 
				printk(KERN_ERR "Error inform to battery driver !\n");
			else
				printk(KERN_INFO "normal->throttle: set cpufreq upper limit.\n");
		}
		break;

	case TMU_STATUS_THROTTLED:
		if (cur_temp >= set_thr_temp && !(cpufreq_limited_thr)) { /* 85 */
			s5pv310_cpufreq_upper_limit(DVFS_LOCK_ID_TMU, CPU_L2); /* CPU_L2 */
			cpufreq_limited_thr = 1;
			if (tmu_tripped_cb(TMU_STATUS_THROTTLED) < 0) 
				printk(KERN_ERR "Error inform to battery driver !\n");
			else
				printk(KERN_INFO "throttling: set cpufreq upper limit.\n");
		}

		if (cur_temp <= tmu_temp_band.thr_low) {
			s5pv310_cpufreq_upper_limit_free(DVFS_LOCK_ID_TMU);
			tmu_info->ctz->data.tmu_flag = TMU_STATUS_NORMAL;
			cpufreq_limited_thr = 0;
			printk(KERN_INFO "throttling->normal: free cpufreq upper limit.\n");
		}

		if (cur_temp >= set_warn_temp) { /* 100 */
			tmu_info->ctz->data.tmu_flag = TMU_STATUS_WARNING;
			s5pv310_cpufreq_upper_limit_free(DVFS_LOCK_ID_TMU);
			cpufreq_limited_thr = 0;
			if (set_cpu_level == 3)
				s5pv310_cpufreq_upper_limit(DVFS_LOCK_ID_TMU, CPU_L3); /* CPU_L4 */
			else
				s5pv310_cpufreq_upper_limit(DVFS_LOCK_ID_TMU, CPU_L4); /* CPU_L4 */

			cpufreq_limited_warn = 1;
			if (tmu_tripped_cb(TMU_STATUS_WARNING) < 0) 
				printk(KERN_ERR "Error inform to battery driver !\n");
			else
				printk(KERN_INFO "throttling->warning: up cpufreq upper limit.\n");
		}	
		break;

	case TMU_STATUS_WARNING:
		if (cur_temp >= set_warn_temp && !(cpufreq_limited_warn)) { /* 100 */
			s5pv310_cpufreq_upper_limit_free(DVFS_LOCK_ID_TMU);
			cpufreq_limited_thr = 0;
			if (set_cpu_level == 3)
				s5pv310_cpufreq_upper_limit(DVFS_LOCK_ID_TMU, CPU_L3); /* CPU_L4 */
			else
				s5pv310_cpufreq_upper_limit(DVFS_LOCK_ID_TMU, CPU_L4); /* CPU_L4 */
			
			cpufreq_limited_warn = 1;
			if (tmu_tripped_cb(TMU_STATUS_WARNING) < 0) 
				printk(KERN_ERR "Error inform to battery driver !\n");
			else
				printk(KERN_INFO "warning: set cpufreq upper limit.\n");
		}

		/* if (cur_temp < tmu_temp_band.warn_low) { */
		if (cur_temp < set_warn_stop) {
			tmu_info->ctz->data.tmu_flag = TMU_STATUS_THROTTLED;
			s5pv310_cpufreq_upper_limit_free(DVFS_LOCK_ID_TMU);
			cpufreq_limited_warn = 0;
			s5pv310_cpufreq_upper_limit(DVFS_LOCK_ID_TMU, CPU_L2); /* CPU_L2 */
			cpufreq_limited_thr = 1;
			if (tmu_tripped_cb(TMU_STATUS_THROTTLED) < 0)
				printk(KERN_ERR "Error inform to battery driver !\n");
			else
				printk(KERN_INFO "warning->throttling: down cpufreq upper limit.\n");
		}

		if (cur_temp >= set_trip_temp) { /* 110 */
			tmu_info->ctz->data.tmu_flag = TMU_STATUS_TRIPPED;
			if (tmu_tripped_cb(TMU_STATUS_TRIPPED) < 0)
				printk(KERN_ERR  "Error inform to battery driver !\n");
			else
				printk(KERN_INFO "warning->tripping: waiting shutdown !!!\n");
		}
		break;

	case TMU_STATUS_TRIPPED:  
		if (cur_temp >= set_trip_temp && !(send_msg_battery)) { /* 110 */
			if (tmu_tripped_cb(TMU_STATUS_TRIPPED) < 0) 
				printk(KERN_ERR "Error inform to battery driver !\n");
			else {
				printk(KERN_INFO "tripping: waiting shutdown.\n");
				send_msg_battery = 1;
			}
		}

		//if (cur_temp >= (TEMP_MAX_CELCIUS - 5)) {
		if (cur_temp >= (set_trip_temp + 5)) {
			panic("Emergency!!!! tmu tripping event is not treated! \n");
		}

		if (cur_temp >= tmu_temp_band.trip_retry) {
			printk(KERN_WARNING "WARNING!!: try to send msg to battery driver again\n");
			send_msg_battery = 0;
		}

		/* safety code */
		if (cur_temp <= tmu_temp_band.thr_low) {
			s5pv310_cpufreq_upper_limit_free(DVFS_LOCK_ID_TMU);
			tmu_info->ctz->data.tmu_flag = TMU_STATUS_NORMAL;
			cpufreq_limited_thr = 0;
			printk(KERN_INFO "tripping->normal: check! occured only test mode.\n");
		}
		break;

	default:
		printk(KERN_WARNING "bug: checked tmu_state.\n");
		if (cur_temp < tmu_temp_band.warn_high) {
			tmu_info->ctz->data.tmu_flag = TMU_STATUS_WARNING;
		} else {
			tmu_info->ctz->data.tmu_flag = TMU_STATUS_TRIPPED;
			if (tmu_tripped_cb(TMU_STATUS_TRIPPED) < 0)
				printk(KERN_ERR  "Error inform to battery driver !\n");
		}
		break;

	}

	if (set_tq0_mode == 1) {
		if (cur_temp >= tmu_temp_band.tq0_temp) {
			if (!(auto_refresh_changed)) {
				printk(KERN_INFO "set auto_refresh 1.95us\n");
				set_refresh_rate(AUTO_REFRESH_PERIOD_TQ0);
				auto_refresh_changed = 1;
			}
		}
		//if (cur_temp <= (TEMP_TQ0_CELCIUS - 5)) {
		if (cur_temp <= (tmu_temp_band.tq0_temp - 5)) {
			if (auto_refresh_changed) {
				printk(KERN_INFO "set auto_refresh 3.9us\n");
				set_refresh_rate(AUTO_REFRESH_PERIOD_NORMAL);
				auto_refresh_changed = 0;
			}
		}
		/* check tq0 input */
		printk(KERN_INFO "tq0 input : 0x%02x\n", __raw_readl(S5PV310_VA_GPIO2 + 0x104));

	} else if (set_tq0_mode == 3) { /* autor refresh change by tq0 input value */
		unsigned int data;

		/* check tq0 input */
		data = __raw_readl(S5PV310_VA_GPIO2 + 0x104);
		data = data & (1<<3);

		if (data) {
			if (!(auto_refresh_changed)) {
				printk(KERN_INFO "set auto_refresh 1.95us : 0x%08x", data);
				set_refresh_rate(AUTO_REFRESH_PERIOD_TQ0);
				auto_refresh_changed = 1;
			}
		} else {
			if (auto_refresh_changed) {
				printk(KERN_INFO "set auto_refresh 3.9us : 0x%08x", data);
				set_refresh_rate(AUTO_REFRESH_PERIOD_NORMAL);
				auto_refresh_changed = 0;
			}
		}
	}

	/* rescheduling next work */
	queue_delayed_work_on(0, tmu_monitor_wq, &tmu_info->polling_work,
			tmu_info->sampling_rate);
}
#endif

static void tmu_poll_timer(struct work_struct *work)
{
	unsigned char cur_temp;
	static int cpufreq_limited_thr	= 0;
	static int cpufreq_limited_warn	= 0;
	static int send_msg_battery = 0;
	static int auto_refresh_changed = 0;

#ifdef CONFIG_TMU_DEBUG_ENABLE
	if (set_tmu_test) {
		tmu_poll_testmode();
		return;
	}
#endif

	mutex_lock(&tmu_lock);

	/* Compensation temperature */
	cur_temp = __raw_readb(tmu_info->ctz->tmu_base + CURRENT_TEMP)
			- tmu_info->ctz->data.te1 + TMU_DC_VALUE;
	pr_info("current temp = %d, %d\n", cur_temp, tmu_info->ctz->data.tmu_flag);

	switch (tmu_info->ctz->data.tmu_flag) {
	case TMU_STATUS_NORMAL:
		if (cur_temp <= tmu_temp_band.thr_low) {
			//cancel_delayed_work(&tmu_info->polling_work);
			if (tmu_tripped_cb(TMU_STATUS_NORMAL) < 0) 
				printk(KERN_ERR "Error inform to battery driver !\n");
			else
				printk(KERN_INFO "normal: interrupt enable.\n");

			/* clear to prevent from interfupt by peindig bit */
			__raw_writel(INTCLEARALL, tmu_info->ctz->tmu_base + INTCLEAR);
			enable_irq(irq_tmu);
			mutex_unlock(&tmu_lock);
			return;
		}
		if (cur_temp >= TEMP_TROTTLED_CELCIUS) { /* 87 */
			tmu_info->ctz->data.tmu_flag = TMU_STATUS_THROTTLED;
			s5pv310_cpufreq_upper_limit(DVFS_LOCK_ID_TMU, CPU_L2); /*CPU_L2 */
			cpufreq_limited_thr = 1;
			if (tmu_tripped_cb(TMU_STATUS_THROTTLED) < 0) 
				printk(KERN_ERR "Error inform to battery driver !\n");
			else
				printk(KERN_INFO "normal->throttle: set cpufreq upper limit.\n");
		}
		break;

	case TMU_STATUS_THROTTLED:
		if (cur_temp >= TEMP_TROTTLED_CELCIUS && !(cpufreq_limited_thr)) { /* 87 */
			s5pv310_cpufreq_upper_limit(DVFS_LOCK_ID_TMU, CPU_L2); /* CPU_L2 */
			cpufreq_limited_thr = 1;
			if (tmu_tripped_cb(TMU_STATUS_THROTTLED) < 0) 
				printk(KERN_ERR "Error inform to battery driver !\n");
			else
				printk(KERN_INFO "throttling: set cpufreq upper limit.\n");
		}
		if (cur_temp <= tmu_temp_band.thr_low) {
			s5pv310_cpufreq_upper_limit_free(DVFS_LOCK_ID_TMU);
			tmu_info->ctz->data.tmu_flag = TMU_STATUS_NORMAL;
			cpufreq_limited_thr = 0;
			printk(KERN_INFO "throttling->normal: free cpufreq upper limit.\n");
		}
		if (cur_temp >= TEMP_WARNING_CELCIUS) { /* 103 */
			tmu_info->ctz->data.tmu_flag = TMU_STATUS_WARNING;
			s5pv310_cpufreq_upper_limit_free(DVFS_LOCK_ID_TMU);
			cpufreq_limited_thr = 0;
			s5pv310_cpufreq_upper_limit(DVFS_LOCK_ID_TMU, CPU_L4); /* CPU_L4 */
			cpufreq_limited_warn = 1;
			if (tmu_tripped_cb(TMU_STATUS_WARNING) < 0) 
				printk(KERN_ERR "Error inform to battery driver !\n");
			else
				printk(KERN_INFO "throttling->warning: set cpufreq upper limit.\n");
		}		
		break;

	case TMU_STATUS_WARNING:
		if (cur_temp >= TEMP_WARNING_CELCIUS && !(cpufreq_limited_warn)) { /* 103 */
			s5pv310_cpufreq_upper_limit_free(DVFS_LOCK_ID_TMU);
			cpufreq_limited_thr = 0;
			s5pv310_cpufreq_upper_limit(DVFS_LOCK_ID_TMU, CPU_L4); /* CPU_L4 */

			cpufreq_limited_warn = 1;
			if (tmu_tripped_cb(TMU_STATUS_WARNING) < 0) 
				printk(KERN_ERR "Error inform to battery driver !\n");
			else
				printk(KERN_INFO "warning: set cpufreq upper limit.\n");
		}
		if (cur_temp <= tmu_temp_band.warn_low) {
			tmu_info->ctz->data.tmu_flag = TMU_STATUS_THROTTLED;
			s5pv310_cpufreq_upper_limit_free(DVFS_LOCK_ID_TMU);
			cpufreq_limited_warn = 0;
			s5pv310_cpufreq_upper_limit(DVFS_LOCK_ID_TMU, CPU_L2); /* CPU_L2 */
			cpufreq_limited_thr = 1;
			if (tmu_tripped_cb(TMU_STATUS_THROTTLED) < 0)
				printk(KERN_ERR "Error inform to battery driver !\n");
			else
				printk(KERN_INFO "warning->throttling: up cpufreq upper limit.\n");
		}
		if (cur_temp >= TEMP_TRIPPED_CELCIUS) { /* 110 */
			tmu_info->ctz->data.tmu_flag = TMU_STATUS_TRIPPED;
			if (tmu_tripped_cb(TMU_STATUS_TRIPPED) < 0)
				printk(KERN_ERR  "Error inform to battery driver !\n");
			else
				printk(KERN_INFO "warning->tripping: waiting shutdown !!!\n");
		}
		break;


	case TMU_STATUS_TRIPPED:  
		if (cur_temp >= TEMP_TRIPPED_CELCIUS && !(send_msg_battery)) { /* 110 */
			if (tmu_tripped_cb(TMU_STATUS_TRIPPED) < 0) 
				printk(KERN_ERR "Error inform to battery driver !\n");
			else {
				printk(KERN_INFO "tripping: waiting shutdown.\n");
				send_msg_battery = 1;
			}
		}
		if (cur_temp >= (TEMP_MAX_CELCIUS - 5)) { /* 120 */
			panic("Emergency!!!! tmu tripping event is not treated! \n");
		}

		if (cur_temp >= tmu_temp_band.trip_retry) {
			printk(KERN_WARNING "WARNING!!: try to send msg to battery driver again\n");
			send_msg_battery = 0;
		}

		/* safety code */
		if (cur_temp <= tmu_temp_band.thr_low) {
			s5pv310_cpufreq_upper_limit_free(DVFS_LOCK_ID_TMU);
			tmu_info->ctz->data.tmu_flag = TMU_STATUS_NORMAL;
			cpufreq_limited_thr = 0;
			printk(KERN_INFO "tripping->normal: Check! occured only test mode.\n");
		}
		break;

	case TMU_STATUS_INIT: /* sned tmu initial status to battery drvier */
		disable_irq(irq_tmu);

		if (cur_temp <= tmu_temp_band.thr_low) {
			tmu_info->ctz->data.tmu_flag = TMU_STATUS_NORMAL;
		} else {
			tmu_info->ctz->data.tmu_flag = TMU_STATUS_THROTTLED;
		}
		break;

	default:
		printk(KERN_WARNING "Bug: checked tmu_state.\n");
		if (cur_temp < tmu_temp_band.warn_high) {
			tmu_info->ctz->data.tmu_flag = TMU_STATUS_WARNING;
		} else {
			tmu_info->ctz->data.tmu_flag = TMU_STATUS_TRIPPED;
			if (tmu_tripped_cb(TMU_STATUS_TRIPPED) < 0) 
				printk(KERN_ERR  "Error inform to battery driver !\n");
		}
		break;
	} /* end */

	if (cur_temp >= TEMP_TQ0_CELCIUS) { /* 90 */
		if (!(auto_refresh_changed)) {
			printk(KERN_INFO "set auto_refresh 1.95us\n");
			set_refresh_rate(AUTO_REFRESH_PERIOD_TQ0);
			auto_refresh_changed = 1;
		}
	}
	if (cur_temp <= (TEMP_TQ0_CELCIUS - 5)) { /* 85 */
		if (auto_refresh_changed) {
			printk(KERN_INFO "set auto_refresh 3.9us\n");
			set_refresh_rate(AUTO_REFRESH_PERIOD_NORMAL);
			auto_refresh_changed = 0;
		}
	}
	/* read tq0 input */
	/* printk(KERN_INFO "tq0 input : 0x%02x\n", __raw_readl(S5PV310_VA_GPIO2 + 0x104));
	*/

	/* rescheduling next work */
	queue_delayed_work_on(0, tmu_monitor_wq, &tmu_info->polling_work,
			tmu_info->sampling_rate);

	mutex_unlock(&tmu_lock);

	return;
}

static int tmu_initialize(struct platform_device *pdev)
{
	struct s5p_tmu *tz = platform_get_drvdata(pdev);
	unsigned int en;
	unsigned int te_temp;

	__raw_writel(INTCLEAR2, tz->tmu_base + INTCLEAR);

	en = (__raw_readl(tz->tmu_base + TMU_STATUS) & 0x1);

	if (!en) {
		dev_err(&pdev->dev, "failed to start tmu drvier\n");
		return -ENOENT;
	}

	/* get the compensation parameter */
	te_temp = __raw_readl(tz->tmu_base + TRIMINFO);
	tz->data.te1 = te_temp & TRIM_TEMP_MASK;
	tz->data.te2 = ((te_temp >> 8) & TRIM_TEMP_MASK);

	printk(KERN_INFO "%s: te_temp = 0x%08x, low 8bit = %d, high 24 bit = %d\n",
			__func__, te_temp, tz->data.te1, tz->data.te2);

	if((EFUSE_MIN_VALUE > tz->data.te1) || (tz->data.te1 > EFUSE_MAX_VALUE)
		||  (tz->data.te2 != 0))

	tz->data.te1 = EFUSE_AVG_VALUE;

	/* Need to initail regsiter setting after getting parameter info */
	/* [28:23] vref [11:8] slope - Tunning parameter */
	__raw_writel(VREF_SLOPE, tz->tmu_base + TMU_CON0);

	return 0;
}

static void tmu_start(struct platform_device *pdev)
{
	struct s5p_tmu *tz = platform_get_drvdata(pdev);
	unsigned int con;
	unsigned int thresh_temp_adc;

	__raw_writel(INTCLEARALL, tz->tmu_base + INTCLEAR);

#ifdef CONFIG_TMU_DEBUG_ENABLE
	if (set_tmu_test) {
		tmu_start_testmode(pdev);
		return;
	}
#endif

	/* Compensation temperature THD_TEMP */
	thresh_temp_adc = THD_TEMP + tz->data.te1 - TMU_CODE_25_DEGREE;
	printk(KERN_INFO "Compensated Threshold: 0x%2x\n", thresh_temp_adc);

	/* Set interrupt trigger level */
	__raw_writel(thresh_temp_adc, tz->tmu_base + THRESHOLD_TEMP);
	__raw_writel(TRIGGER_LEV0, tz->tmu_base + TRG_LEV0);
	__raw_writel(TRIGGER_LEV1, tz->tmu_base + TRG_LEV1);
	__raw_writel(TRIGGER_LEV2, tz->tmu_base + TRG_LEV2);
	__raw_writel(TRIGGER_LEV3, tz->tmu_base + TRG_LEV3);

	mdelay(50);    
	/* TMU core enable */
	con = __raw_readl(tz->tmu_base + TMU_CON0);
	con |= TMU_EN;

	__raw_writel(con, tz->tmu_base + TMU_CON0);

	/*LEV0 LEV1 LEV2 interrupt enable */
	__raw_writel(INTEN0 | INTEN1 | INTEN2, tz->tmu_base + INTEN);

	pr_info("Cooling: %dc  THD_TEMP:0x%02x:  TRIG_LEV0: 0x%02x\
		TRIG_LEV1: 0x%02x TRIG_LEV2: 0x%02x\n",
		tz->data.cooling,
		THD_TEMP,
		THD_TEMP + TRIGGER_LEV0,
		THD_TEMP + TRIGGER_LEV1,
		THD_TEMP + TRIGGER_LEV2);
}

static irqreturn_t s5p_tmu_irq(int irq, void *id)
{
	struct s5p_tmu *tz = id;
	unsigned int status;

	disable_irq_nosync(irq);
	
	status = __raw_readl(tz->tmu_base + INTSTAT);

	pr_info("TMU interrupt occured : status = 0x%08x\n", status);

	if (status & INTSTAT2) {
		tz->data.tmu_flag = TMU_STATUS_TRIPPED;
		__raw_writel(INTCLEAR2, tz->tmu_base + INTCLEAR);
	}
	else if (status & INTSTAT1) {
		tz->data.tmu_flag = TMU_STATUS_WARNING;
		__raw_writel(INTCLEAR1, tz->tmu_base + INTCLEAR);
	}
	else if (status & INTSTAT0) {
		tz->data.tmu_flag = TMU_STATUS_THROTTLED;
		__raw_writel(INTCLEAR0, tz->tmu_base + INTCLEAR);
	}
	else {
		pr_err("%s: TMU interrupt error\n", __func__);
		__raw_writel(INTCLEARALL, tz->tmu_base + INTCLEAR);
	        queue_delayed_work_on(0, tmu_monitor_wq, &tmu_info->polling_work,
			tmu_info->sampling_rate / 2);
		return -ENODEV;
	}

	queue_delayed_work_on(0, tmu_monitor_wq, &tmu_info->polling_work,
		tmu_info->sampling_rate);

	return IRQ_HANDLED;
}

static int __devinit s5p_tmu_probe(struct platform_device *pdev)
{
	struct s5p_tmu *tz = platform_get_drvdata(pdev);
	struct resource *res;
	int ret = 0;

	pr_debug("%s: probe=%p\n", __func__, pdev);

	tmu_info = kzalloc(sizeof(struct s5p_tmu_info), GFP_KERNEL);
	if (!tmu_info) {
		printk(KERN_ERR "%s: failed to alloc memory!\n", __func__);
		ret = -ENOMEM;
		goto err_nomem;
	}
	tmu_info->dev = &pdev->dev;
	tmu_info->ctz = tz;
	tmu_info->ctz->data.tmu_flag = TMU_STATUS_INIT;

	irq_tmu = platform_get_irq(pdev, 0);
	if (irq_tmu < 0) {
		dev_err(&pdev->dev, "no irq for thermal\n");
		return -ENOENT;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get memory region resource\n");
		return -ENOENT;
	}

	s5p_tmu_mem = request_mem_region(res->start,
					res->end-res->start+1,
					pdev->name);
	if (s5p_tmu_mem == NULL) {
		dev_err(&pdev->dev, "failed to reserve memory region\n");
		ret = -ENOENT;
		goto err_nores;
	}

	tz->tmu_base = ioremap(res->start, (res->end - res->start) + 1);
	if (tz->tmu_base == NULL) {
		dev_err(&pdev->dev, "failed ioremap()\n");
		ret = -EINVAL;
		goto err_nomap;
	}

#ifdef CONFIG_TMU_DEBUG_ENABLE
	if (set_tmu_test) {
		tmu_temp_band.thr_low	 = set_thr_stop;
		tmu_temp_band.trip_retry = set_trip_temp + 3;
		tmu_temp_band.thr_high	 = set_warn_temp - 5;
		tmu_temp_band.warn_low	 = set_warn_stop; /* set_warn_temp - 5 */
		tmu_temp_band.warn_high	 = set_warn_temp + 5;
		tmu_temp_band.tq0_temp	 = set_tq0_temp;

		printk(KERN_INFO "thr_stop = %d, thr_temp = %d,\
			warn_temp = %d, trip_temp = %d, tq0_temp = %d\n",
			set_thr_stop, set_thr_temp,
			set_warn_temp, set_trip_temp,
			set_tq0_temp);

	}
#endif

	pr_info("thr_low: %d, thr_high: %d  warn_low: %d c warn_high %d\n",
		tmu_temp_band.thr_low, tmu_temp_band.thr_high,
		tmu_temp_band.warn_low, tmu_temp_band.warn_high);

	tmu_monitor_wq = create_freezeable_workqueue(dev_name(&pdev->dev));
        if (!tmu_monitor_wq) {
                printk(KERN_INFO "Creation of tmu_monitor_wq failed\n");
                return -EFAULT;
        }

	/* set sampling rate to poll current temp */
	tmu_info->sampling_rate  = usecs_to_jiffies(1000 * 1000);  /* 1 sec sampling */

#ifdef CONFIG_TMU_DEBUG_ENABLE
	if (set_sampling_rate) {
		tmu_info->sampling_rate  = usecs_to_jiffies(set_sampling_rate * 1000);
		tmu_info->monitor_period = usecs_to_jiffies(set_sampling_rate * 10 * 1000);
	} else {
		tmu_info->monitor_period = usecs_to_jiffies(10000 * 1000); /* 10sec monitroing */
	}

	if (set_tmu_test) {
		INIT_DELAYED_WORK_DEFERRABLE(&tmu_info->monitor_work, tmu_mon_timer);

		queue_delayed_work_on(0, tmu_monitor_wq, &tmu_info->monitor_work, tmu_info->monitor_period);
	}

	if (set_tq0_mode == 2)
		INIT_DELAYED_WORK_DEFERRABLE(&tq0_polling_work, tq0_poll_timer);
#endif

	if (tq0_signal_handle_init())
		pr_err("%s failed\n", __func__);

	INIT_DELAYED_WORK_DEFERRABLE(&tmu_info->polling_work, tmu_poll_timer);

	/* rescheduling next work to inform tmu status to battery driver */
	queue_delayed_work_on(0, tmu_monitor_wq, &tmu_info->polling_work,
		tmu_info->sampling_rate * 10);

	ret = request_irq(irq_tmu, s5p_tmu_irq,
			IRQF_DISABLED,  "s5p-tmu interrupt", tz);
	if (ret) {
		dev_err(&pdev->dev, "IRQ%d error %d\n", irq_tmu, ret);
		goto err_irq;
	}

	ret = tmu_initialize(pdev);
	if (ret)
		goto err_nores;

	tmu_start(pdev);

	return ret;

err_irq:
	free_irq(irq_tmu, tz);

err_nomap:
	release_resource(s5p_tmu_mem);
	kfree(tmu_info);

err_nomem:
err_nores:
	return ret;
}

static int __devinit s5p_tmu_remove(struct platform_device *pdev)
{
	struct s5p_tmu *tz = platform_get_drvdata(pdev);

	free_irq(irq_tmu, (void *)pdev);

	kfree(tmu_info);

	iounmap(tz->tmu_base);

	//kfree(pdev->dev.platform_data);

	printk("%s is removed\n", dev_name(&pdev->dev));
	return 0;
}

#ifdef CONFIG_PM
static int s5p_tmu_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct s5p_tmu *tz = platform_get_drvdata(pdev);

	/* save tmu register value */ 
	tmu_save[0] = __raw_readl(tz->tmu_base + TMU_CON0);
	tmu_save[1] = __raw_readl(tz->tmu_base + SAMPLING_INTERNAL);
	tmu_save[2] = __raw_readl(tz->tmu_base + CNT_VALUE0);
	tmu_save[3] = __raw_readl(tz->tmu_base + CNT_VALUE1);
	tmu_save[4] = __raw_readl(tz->tmu_base + THRESHOLD_TEMP);
	tmu_save[5] = __raw_readl(tz->tmu_base + INTEN);
	tmu_save[6] = __raw_readl(tz->tmu_base + TRG_LEV0);
	tmu_save[7] = __raw_readl(tz->tmu_base + TRG_LEV1);
	tmu_save[8] = __raw_readl(tz->tmu_base + TRG_LEV2);
	tmu_save[9] = __raw_readl(tz->tmu_base + TRG_LEV3);

	disable_irq(irq_tmu);

#ifdef CONFIG_TMU_DEBUG_ENABLE
	disable_irq(irq_tq0);
#endif
	return 0;
}

static int s5p_tmu_resume(struct platform_device *pdev)
{
	struct s5p_tmu *tz = platform_get_drvdata(pdev);

	/* save tmu register value */ 
	__raw_writel(tmu_save[0], tz->tmu_base + TMU_CON0);
	__raw_writel(tmu_save[1], tz->tmu_base + SAMPLING_INTERNAL);
	__raw_writel(tmu_save[2], tz->tmu_base + CNT_VALUE0);
	__raw_writel(tmu_save[3], tz->tmu_base + CNT_VALUE1);
	__raw_writel(tmu_save[4], tz->tmu_base + THRESHOLD_TEMP);
	__raw_writel(tmu_save[5], tz->tmu_base + INTEN);
	__raw_writel(tmu_save[6], tz->tmu_base + TRG_LEV0);
	__raw_writel(tmu_save[7], tz->tmu_base + TRG_LEV1);
	__raw_writel(tmu_save[8], tz->tmu_base + TRG_LEV2);
	__raw_writel(tmu_save[9], tz->tmu_base + TRG_LEV3);
	
	enable_irq(irq_tmu);

#ifdef CONFIG_TMU_DEBUG_ENABLE
	enable_irq(irq_tq0);
#endif
	return 0;
}
#else
#define s5p_tmu_suspend	NULL
#define s5p_tmu_resume	NULL
#endif

static struct platform_driver s5p_tmu_driver = {
	.probe		= s5p_tmu_probe,
	.remove		= s5p_tmu_remove,
	.suspend	= s5p_tmu_suspend,
	.resume		= s5p_tmu_resume,
	.driver		= {
		.name   = "s5p-tmu",
		.owner  = THIS_MODULE,
	},
};

static unsigned int tq0_signal_handle_init(void)
{
	u32 err = 0;

	printk(KERN_INFO "tq0_signal_handle_init");

	err = gpio_request(S5PV310_GPL2(3), "GPL2");
	if (err) {
		pr_err("gpio request error : %d\n", err);
	}

	s3c_gpio_cfgpin(S5PV310_GPL2(3), (0xf << 12));


#ifdef CONFIG_TMU_DEBUG_ENABLE
	/* tq0 interrupt */
	if (set_tq0_mode == 2) {
		irq_tq0 = gpio_to_irq(S5PV310_GPL2(3));
		if (irq_tq0 < 0)
			pr_err("irq request error : %d\n", err);

		/* clear interrupt source */
		__raw_writel(0x8, S5PV310_VA_GPIO2 + 0xA20);

		set_irq_type(irq_tq0, IRQ_TYPE_LEVEL_HIGH);
		setup_irq(irq_tq0, &tq0_signal_irq);
	}

	/* eint29[3] is flter enable and filtering width */
	/*__raw_writel(0x1 << 31 | 0x7F << 24, S5PV310_VA_GPIO2 + 0x840);
	*/
#endif
	return err;
}

static int __init s5p_tmu_driver_init(void)
{
/*	if (tq0_signal_handle_init())
		pr_err("%s failed\n", __func__);
*/
	return platform_driver_register(&s5p_tmu_driver);
}

arch_initcall(s5p_tmu_driver_init);
