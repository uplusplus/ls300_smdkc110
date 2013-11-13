/* linux/arch/arm/mach-s5pv210/button-smdkv210.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * S5PV210 - Button Driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
* modify:
*	author: uplusplus
*	email: yjcpui (at) gmail.com
*	1 add timer to avoid key jitter
*	2 add reset key handler to restart system
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>

#include <linux/io.h>
#include <mach/hardware.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <plat/map-base.h>
#include <plat/gpio-cfg.h>

#include <mach/regs-gpio.h>
#include <mach/regs-irq.h>
#include <linux/gpio.h>
#include <linux/irq.h>


 //===
#include <linux/reboot.h>
#include <linux/sched.h>
#include <linux/syscalls.h>
 //===

static struct input_dev *button_dev;

//==============================================================
//

struct button {
	int irq;
	char *name;
	unsigned pin;
	unsigned int cfg;
	unsigned int key;
	int pressed;
};

/* 六个按键的相关定义整合到结构体 */
static struct button buttons[3] = {
	{IRQ_EINT1,  "Power", S5PV210_GPH0(1), 0xf<<4, 	10,			0},
	{IRQ_EINT10, "Menu",  S5PV210_GPH1(2), 0xf<<8, 	KEY_MENU,	0},
	{IRQ_EINT12, "Back",  S5PV210_GPH1(4), 0xf<<16, KEY_BACK,	0},
};

static struct timer_list seventh_timer;
static struct button *buttonp = NULL;

//=======================================================
//reset

int restart_requested;
static void deferred_restart(struct work_struct *dummy)
{
	restart_requested = 2;
	sys_sync();
	restart_requested = 3;
	kernel_restart(NULL);
}
static DECLARE_WORK(restart_work, deferred_restart);

static void restart(){
	pr_info("keyboard reset\n");
	schedule_work(&restart_work);
}

//=======================================================


/* 中断处理函数 */
static irqreturn_t seventhdrv_intr(int irq, void *data)
{
	buttonp = (struct button*)data;
	mod_timer(&seventh_timer, jiffies+HZ/100);

	return IRQ_RETVAL(IRQ_HANDLED);
}

static void button_timer(unsigned long data)
{
	if (buttonp == NULL ) {
		printk(KERN_INFO "No button pressed!\n");
		return ;
	}

	if(!buttonp->pressed) {
		input_report_key(button_dev,buttonp->key,1);
		printk(KERN_INFO "button %s down\n",buttonp->name);
	}else{
		input_report_key(button_dev,buttonp->key,0);
		printk(KERN_INFO "button %s up\n",buttonp->name);

		if(buttonp == &buttons[0]){
			//power key
			restart();
		}
	}

	buttonp->pressed = !buttonp->pressed;

	buttonp = NULL;
}

//==============================================================


static unsigned int s3c_button_gpio_init(void)
{
	u32 err,i;

	for(i=0;i<3;i++){
		err = gpio_request(buttons[i].pin, buttons[i].name);
		if (err) {
			printk(KERN_INFO "gpio request %s,%d error : %d\n", 
				buttons[i].name, buttons[i].pin, err);
		} else {
			s3c_gpio_cfgpin(buttons[i].pin,buttons[i].cfg);
			s3c_gpio_setpull(buttons[i].pin, S3C_GPIO_PULL_NONE);
		}
	}

	return err;
}


static int __init button_init(void)
{
	int error,i;
	printk("button_init\n");
	if(s3c_button_gpio_init())
	{
		printk("button_init error\n");
		return -1;
	}


	for(i=0;i<3;i++){	
		if(request_irq(buttons[i].irq, &seventhdrv_intr,IRQ_TYPE_EDGE_BOTH,
				buttons[i].name, &buttons[i]))
		{
			printk(KERN_ERR "button.c: Can't allocate irq %d\n", buttons[i].irq);
			return -EBUSY;
		}
	}

	button_dev = input_allocate_device();
	if (!button_dev) {
		printk(KERN_ERR "button.c: Not enough memory\n");
		error = -ENOMEM;
		goto err_free_irq;
	}

	set_bit(EV_KEY, button_dev->evbit);
	set_bit(KEY_BACK, button_dev->keybit);
	set_bit(KEY_MENU, button_dev->keybit);	


	error = input_register_device(button_dev);
	if (error) {
		printk(KERN_ERR "button.c: Failed to register device\n");
		goto err_free_dev;
	}


	setup_timer(&seventh_timer, button_timer, (unsigned long)&buttons);

	return 0;
err_free_dev:
	input_free_device(button_dev);
err_free_irq:
	for(i=0;i<3;i++){
		free_irq(buttons[i].irq, &buttons[i]);
	}
	return error;
}
static void __exit button_exit(void)
{
	int i;
	input_unregister_device(button_dev);
	for(i=0;i<3;i++){
		free_irq(buttons[i].irq, &buttons[i]);
	}

}
module_init(button_init);
module_exit(button_exit);
