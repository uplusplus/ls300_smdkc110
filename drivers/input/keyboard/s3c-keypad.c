/* drivers/input/keyboard/s3c-keypad.c
 *
 * Driver core for Samsung SoC onboard UARTs.
 *
 * Kim Kyoungil, Copyright (c) 2006-2009 Samsung Electronics
 *      http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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

#include <mach/regs-gpio.h>
#include <plat/regs-keypad.h>

#include "s3c-keypad.h"

#include<linux/proc_fs.h>
#include<linux/string.h>
#include<asm/uaccess.h> 

#ifdef CONFIG_S3C_KEYPAD_DEBUG
#define S3C_KEYPAD_DEBUG    1
#else
#undef S3C_KEYPAD_DEBUG
#endif

#ifdef S3C_KEYPAD_DEBUG
#define DPRINTK(x...) printk(KERN_INFO"S3C-Keypad " x)
#else
#define DPRINTK(x...)		/* !!!! */
#endif

#define DEVICE_NAME "s3c-keypad"

#define TRUE 1
#define FALSE 0

static struct timer_list keypad_timer;
static int is_timer_on = FALSE;
static struct clk *keypad_clock;

static u32 keymask[KEYPAD_COLUMNS];
static u32 prevmask[KEYPAD_COLUMNS];


//follow by fanwb,merge button drvier to keypad,only GPH0 poweron

struct input_dev * gdev = NULL; // by fanwb

///////////////////////////////////////////////////////
struct proc_dir_entry *keypad_proc_ptr=NULL; 
char msg[512]; 

int keypad_proc_read( char *page, char **start, off_t off,
		int count, int *eof, void *data )
{
	//printk("keypad_proc_read\n");

	return sprintf(page, "%s\n", msg);
}


ssize_t keypad_proc_write( struct file *filp, const char __user *buff,
		unsigned long len, void *data )
{
	//printk("keypad_proc_write\n");

	copy_from_user(msg,buff,len);

	input_report_key(gdev,msg[0],1);
	udelay(5);
	input_report_key(gdev,msg[0],0);

	printk("write data:%d\n",msg[0]);

	return len;
}
///////////////////////////////////////////////////////////


static int keypad_scan(void)
{
	u32 col,cval,rval;

	//	printk("H3C %x H2C %x \n",readl(S5PV210_GPH3_BASE),readl(S5PV210_GPH2_BASE));
	//	printk("keypad_scan() is called\n");

	DPRINTK("row val = %x",readl(key_base + S3C_KEYIFROW));

	for (col=0; col < KEYPAD_COLUMNS; col++) {
		/* clear that column number and make that normal output */
		cval = KEYCOL_DMASK & ~((1 << col) | (1 << col+ 8));
		writel(cval, key_base+S3C_KEYIFCOL);

		udelay(KEYPAD_DELAY);

		rval = ~(readl(key_base+S3C_KEYIFROW)) & ((1<<KEYPAD_ROWS)-1) ;
		keymask[col] = rval; 
	}

	writel(KEYIFCOL_CLEAR, key_base+S3C_KEYIFCOL);

	return 0;
}

static void keypad_timer_handler(unsigned long data)
{
	u32 press_mask;
	u32 release_mask;
	u32 restart_timer = 0;
	int i,col;
	struct s3c_keypad *pdata = (struct s3c_keypad *)data;
	struct input_dev *dev = pdata->dev;

	keypad_scan();

	for (col=0; col < KEYPAD_COLUMNS; col++) {
		press_mask = ((keymask[col] ^ prevmask[col]) & keymask[col]); 
		release_mask = ((keymask[col] ^ prevmask[col]) & prevmask[col]); 
		i = col * KEYPAD_ROWS;

		while (press_mask) {
			if (press_mask & 1) {
				input_report_key(dev,pdata->keycodes[i],1);
				DPRINTK("\nkey Pressed  : key %d map %d\n",i, pdata->keycodes[i]);
			}
			press_mask >>= 1;
			i++;
		}

		i = col * KEYPAD_ROWS;

		while (release_mask) {
			if (release_mask & 1) {
				input_report_key(dev,pdata->keycodes[i],0);
				DPRINTK("\nkey Released : %d  map %d\n",i,pdata->keycodes[i]);
			}
			release_mask >>= 1;
			i++;
		}
		prevmask[col] = keymask[col];

		restart_timer |= keymask[col];
	}

	if (restart_timer) {
		mod_timer(&keypad_timer,jiffies + HZ/10);
	} else {
		writel(KEYIFCON_INIT, key_base+S3C_KEYIFCON);
		is_timer_on = FALSE;
	}
}

static irqreturn_t s3c_keypad_isr(int irq, void *dev_id)
{
	/* disable keypad interrupt and schedule for keypad timer handler */
	writel(readl(key_base+S3C_KEYIFCON) & ~(INT_F_EN|INT_R_EN), key_base+S3C_KEYIFCON);

	keypad_timer.expires = jiffies + (HZ/100);
	if (is_timer_on == FALSE) {
		add_timer(&keypad_timer);
		is_timer_on = TRUE;
	} else {
		mod_timer(&keypad_timer, keypad_timer.expires);
	}
	/*Clear the keypad interrupt status*/
	writel(KEYIFSTSCLR_CLEAR, key_base+S3C_KEYIFSTSCLR);

	return IRQ_HANDLED;
}


static int __init s3c_keypad_probe(struct platform_device *pdev)
{
	struct resource *res, *keypad_mem, *keypad_irq;
	struct input_dev *input_dev;
	struct s3c_keypad *s3c_keypad;
	int ret, size;
	int key, code;

	printk("s3c_keypad_probe\n");
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no memory resource specified\n");
		return -ENOENT;
	}

	size = (res->end - res->start) + 1;

	keypad_mem = request_mem_region(res->start, size, pdev->name);
	if (keypad_mem == NULL) {
		dev_err(&pdev->dev, "failed to get memory region\n");
		ret = -ENOENT;
		goto err_req;
	}

	key_base = ioremap(res->start, size);
	if (key_base == NULL) {
		printk(KERN_ERR "Failed to remap register block\n");
		ret = -ENOMEM;
		goto err_map;
	}

	keypad_clock = clk_get(&pdev->dev, "keypad");
	if (IS_ERR(keypad_clock)) {
		dev_err(&pdev->dev, "failed to find keypad clock source\n");
		ret = PTR_ERR(keypad_clock);
		goto err_clk;
	}

	clk_enable(keypad_clock);

	s3c_keypad = kzalloc(sizeof(struct s3c_keypad), GFP_KERNEL);
	input_dev = input_allocate_device();

	if (!s3c_keypad || !input_dev) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	platform_set_drvdata(pdev, s3c_keypad);
	s3c_keypad->dev = input_dev;
	gdev = input_dev;// by fanwb

	/////////////////////////////////////////////////////////////////
	//by zorro
	printk(KERN_INFO "keypad_proc init.  Module is now loaded.\n");
	keypad_proc_ptr = create_proc_entry("keypad", S_IFREG|S_IRUGO|S_IWUSR, NULL);
	keypad_proc_ptr->data = (void *)0;
	keypad_proc_ptr->read_proc  = keypad_proc_read;
	keypad_proc_ptr->write_proc = keypad_proc_write;



	writel(KEYIFCON_INIT, key_base+S3C_KEYIFCON);
	writel(KEYIFFC_DIV, key_base+S3C_KEYIFFC);

	/* Set GPIO Port for keypad mode and pull-up disable*/

#if defined(CONFIG_KEYPAD_S3C_MSM)
	s3c_setup_keypad_cfg_gpio();
#else
	s3c_setup_keypad_cfg_gpio(KEYPAD_ROWS, KEYPAD_COLUMNS);
#endif
	writel(KEYIFCOL_CLEAR, key_base+S3C_KEYIFCOL);

	/* create and register the input driver */
	set_bit(EV_KEY, input_dev->evbit);
	/* Commenting the generation of repeat events */
	//set_bit(EV_REP, input_dev->evbit);
	s3c_keypad->nr_rows = KEYPAD_ROWS;
	s3c_keypad->no_cols = KEYPAD_COLUMNS;
	s3c_keypad->total_keys = MAX_KEYPAD_NR;

	for (key = 0; key < s3c_keypad->total_keys; key++) {
		code = s3c_keypad->keycodes[key] = keypad_keycode[key];
		if (code <= 0)
			continue;
		set_bit(code, input_dev->keybit);
	}

	input_dev->name = DEVICE_NAME;
	input_dev->phys = "s3c-keypad/input0";

	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0001;

	input_dev->keycode = keypad_keycode;

	/* Scan timer init */
	init_timer(&keypad_timer);
	keypad_timer.function = keypad_timer_handler;
	keypad_timer.data = (unsigned long)s3c_keypad;

	/* For IRQ_KEYPAD */
	keypad_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (keypad_irq == NULL) {
		dev_err(&pdev->dev, "no irq resource specified\n");
		ret = -ENOENT;
		goto err_irq;
	}
	ret = request_irq(keypad_irq->start, s3c_keypad_isr, IRQF_SAMPLE_RANDOM,
			DEVICE_NAME, (void *) pdev);
	if (ret) {
		printk(KERN_ERR"request_irq failed (IRQ_KEYPAD) !!!\n");
		ret = -EIO;
		goto err_irq;
	}

	ret = input_register_device(input_dev);
	if (ret) {
		printk(KERN_ERR"Unable to register s3c-keypad input device!!!\n");
		goto out;
	}

	keypad_timer.expires = jiffies + (HZ/10);

	if (is_timer_on == FALSE) {
		add_timer(&keypad_timer);
		is_timer_on = TRUE;
	} else {
		mod_timer(&keypad_timer, keypad_timer.expires);
	}

	printk(DEVICE_NAME"Initialized\n");
	return 0;

out:
	free_irq(keypad_irq->start, input_dev);
	free_irq(keypad_irq->end, input_dev);

err_irq:
	input_free_device(input_dev);
	kfree(s3c_keypad);

err_alloc:
	clk_disable(keypad_clock);
	clk_put(keypad_clock);

err_clk:
	iounmap(key_base);

err_map:
	release_resource(keypad_mem);
	kfree(keypad_mem);

err_req:
	return ret;
}

static int s3c_keypad_remove(struct platform_device *pdev)
{
	struct input_dev *input_dev = platform_get_drvdata(pdev);
	writel(KEYIFCON_CLEAR, key_base+S3C_KEYIFCON);

	if (keypad_clock) {
		clk_disable(keypad_clock);
		clk_put(keypad_clock);
		keypad_clock = NULL;
	}

	input_unregister_device(input_dev);
	iounmap(key_base);
	kfree(pdev->dev.platform_data);
	free_irq(IRQ_KEYPAD, (void *) pdev);

	del_timer(&keypad_timer);
	printk(DEVICE_NAME " Removed.\n");
	return 0;
}

#ifdef CONFIG_PM

static unsigned int keyiffc;

static int s3c_keypad_suspend(struct platform_device *dev, pm_message_t state)
{
	keyiffc = readl(key_base+S3C_KEYIFFC);

	/* Clear the keypad interrupt status */
	writel(KEYIFSTSCLR_CLEAR, key_base+S3C_KEYIFSTSCLR);

	del_timer(&keypad_timer);
	is_timer_on = FALSE;

	disable_irq(IRQ_KEYPAD);

	clk_disable(keypad_clock);

	return 0;
}

static int s3c_keypad_resume(struct platform_device *dev)
{
#ifdef CONFIG_ANDROID
	struct s3c_keypad *s3c_keypad = (struct s3c_keypad *) platform_get_drvdata(dev);
	struct input_dev *idev = s3c_keypad->dev;
#endif

	clk_enable(keypad_clock);

	writel(KEYIFCON_INIT, key_base+S3C_KEYIFCON);
	writel(keyiffc, key_base+S3C_KEYIFFC);
	writel(KEYIFCOL_CLEAR, key_base+S3C_KEYIFCOL);

#ifdef CONFIG_ANDROID
#define KEYCODE_UNKNOWN 10
	input_report_key(idev, KEYCODE_UNKNOWN, 1);
	udelay(5);
	input_report_key(idev, KEYCODE_UNKNOWN, 0);
#endif

	/* Clear the keypad interrupt status */
	writel(KEYIFSTSCLR_CLEAR, key_base+S3C_KEYIFSTSCLR);

	enable_irq(IRQ_KEYPAD);

	return 0;
}
#else
#define s3c_keypad_suspend NULL
#define s3c_keypad_resume  NULL
#endif /* CONFIG_PM */

static struct platform_driver s3c_keypad_driver = {
	.probe		= s3c_keypad_probe,
	.remove		= s3c_keypad_remove,
	.suspend	= s3c_keypad_suspend,
	.resume		= s3c_keypad_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "s3c-keypad",
	},
};

	static irqreturn_t
s3c_button_interrupt(int irq, void *dev_id)
{
	printk("s3c_button_interrupt\n");
	switch(irq)
	{

		case  IRQ_EINT1:
			{
				printk(KERN_INFO "XEINT 1 as powerkey Button Interrupt occure\n");
				if(gdev)
				{
					input_report_key(gdev,10,1);
					udelay(5);
					input_report_key(gdev,10,0);
				}

			}
			break;
		case IRQ_EINT10:
			printk(KERN_INFO "XEINT 10 Button Interrupt occure\n");
			if(gdev)
			{
				input_report_key(gdev,50,1);
				udelay(5);
				input_report_key(gdev,50,0);
			}

			break;
		case IRQ_EINT12:
			printk(KERN_INFO "XEINT 12 Button Interrupt occure\n");
			if(gdev)
			{
				input_report_key(gdev,34,1);
				udelay(5);
				input_report_key(gdev,34,0);
			}		
			break;
		default:
			printk(KERN_INFO "%d Button Interrupt occure\n", irq);
	}

	return IRQ_HANDLED;
}


static struct irqaction s3c_button_irq = {
	.name		= "s3c button Tick",
	.flags		= IRQF_SHARED ,
	.handler	= s3c_button_interrupt,
};

static unsigned int s3c_button_gpio_init(void)
{
	u32 err;
#ifndef CONFIG_REGULATOR
	err = gpio_request(S5PV210_GPH0(4), "GPH0");
	if (err) {
		printk(KERN_INFO "gpio request error : %d\n", err);
	} else {
		s3c_gpio_cfgpin(S5PV210_GPH0(4), (0xf << 16));
		s3c_gpio_setpull(S5PV210_GPH0(4), S3C_GPIO_PULL_NONE);
	}
#endif
	err = gpio_request(S5PV210_GPH0(1), "GPH0");
	if (err) {
		printk(KERN_INFO "gpio request error : %d\n", err);
	} else {
		s3c_gpio_cfgpin(S5PV210_GPH0(1), (0xf << 4));
		s3c_gpio_setpull(S5PV210_GPH0(1), S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(S5PV210_GPH1(2), "GPH1");
	if (err) {
		printk(KERN_INFO "gpio request error : %d\n", err);
	} else {
		s3c_gpio_cfgpin(S5PV210_GPH1(2), (0xf << 8));
		s3c_gpio_setpull(S5PV210_GPH1(2), S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(S5PV210_GPH1(4), "GPH1");
	if (err) {
		printk(KERN_INFO "gpio request error : %d\n", err);
	} else {
		s3c_gpio_cfgpin(S5PV210_GPH1(4), (0xf << 16));
		s3c_gpio_setpull(S5PV210_GPH1(4), S3C_GPIO_PULL_NONE);
	}



	return err;
}



static int __init s3c_keypad_init(void)
{
	int ret;

	printk("s3c_keypad_init\n");

	if (s3c_button_gpio_init()) {
		printk(KERN_ERR "%s failed\n", __func__);
		return 0;
	}
	set_irq_type(IRQ_EINT1, IRQ_TYPE_EDGE_FALLING);
	set_irq_wake(IRQ_EINT1, 1);
	setup_irq(IRQ_EINT1, &s3c_button_irq);

	set_irq_type(IRQ_EINT(10), IRQ_TYPE_EDGE_FALLING);
	set_irq_wake(IRQ_EINT(10), 1);
	setup_irq(IRQ_EINT(10), &s3c_button_irq);

	set_irq_type(IRQ_EINT(12), IRQ_TYPE_EDGE_FALLING);
	set_irq_wake(IRQ_EINT(12), 1);
	setup_irq(IRQ_EINT(12), &s3c_button_irq);



	ret = platform_driver_register(&s3c_keypad_driver);

	if (!ret)
		printk(KERN_INFO "S3C Keypad Driver\n");

	return ret;
}

static void __exit s3c_keypad_exit(void)
{
	platform_driver_unregister(&s3c_keypad_driver);
}

module_init(s3c_keypad_init);
module_exit(s3c_keypad_exit);

MODULE_AUTHOR("Samsung");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("KeyPad interface for Samsung S3C");
