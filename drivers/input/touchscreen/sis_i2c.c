/* drivers/input/touchscreen/sis_i2c.c
 *
 * Copyright (C) 2009 SiS, Inc.
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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include "sis_i2c.h"
#include <linux/linkage.h>
#include <linux/slab.h>
#include <mach/regs-gpio.h>
#include <linux/gpio.h>

#define FOR_X86       1

struct Point {
	int id;
	unsigned short x, y;   // uint16_t ?
	bool bPressure;
	bool bWidth;
	bool touch, pre;
};

struct sisTP_driver_data {
	int id, fingers;
	bool valid;	//BTN_TOUCH is on/off
	struct Point pt[MAX_FINGERS];
	uint16_t CRC;
};


struct sisTP_driver_data *TPInfo = NULL;

/* Addresses to scan */
static const unsigned short normal_i2c[] = { 0x5, I2C_CLIENT_END };

static struct workqueue_struct *sis_wq;

static int driver_paused = 0;
struct sis_ts_data *ts_bak = 0;

struct sis_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;
	uint16_t max[2];
	int snap_state[2][2];
	int snap_down_on[2];
	int snap_down_off[2];
	int snap_up_on[2];
	int snap_up_off[2];
	int snap_down[2];
	int snap_up[2];
	uint32_t flags;
	int (*power)(int on);
	struct early_suspend early_suspend;
};

int sis_ReadPacket(struct i2c_client *client, uint8_t cmd, uint8_t* buf);
static void sis_tpinfo_clear(struct sisTP_driver_data *TPInfo, int max);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void sis_ts_early_suspend(struct early_suspend *h);
static void sis_ts_late_resume(struct early_suspend *h);
#endif

#ifdef FOR_X86
//static const struct i2c_client_address_data addr_data;
/* Insmod parameters */
//enya I2C_CLIENT_INSMOD_1(synaptics);
static int sis_ts_detect(struct i2c_client *client, /*int kind,//enya*/
		       struct i2c_board_info *info);
#endif

int sis_ReadPacket(struct i2c_client *client, uint8_t cmd, uint8_t* buf)
{
//    uint8_t tmpbuf[32] = {0};
    uint8_t tmpbuf[MAX_READ_BYTE_COUNT] = {0};
    int ret = -1;
	uint8_t offset = 0;
	bool bReRead = false;
	uint8_t ByteCount = 0;
	uint8_t fingers = 0;

	uint16_t xx,yy;
	
	//cmd=0x87;

#ifndef _SMBUS_INTERFACE
    struct i2c_msg msg[2];

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = (char *)(&cmd);
    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = MAX_READ_BYTE_COUNT;
    msg[1].buf = tmpbuf;
//    msg[1].buf = (char *)buf
#endif

	do
    {
#ifdef _SMBUS_INTERFACE
        ret = i2c_smbus_read_block_data(client, cmd, tmpbuf);
#else
        ret = i2c_transfer(client->adapter, msg, 2);
#endif
		printk( "test: ret = %d\n", ret);
		printk( "test: tmpbuf_Data = %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\r\n",
			tmpbuf[0], tmpbuf[1], tmpbuf[2], tmpbuf[3], tmpbuf[4],
			tmpbuf[5], tmpbuf[6], tmpbuf[7], tmpbuf[8], tmpbuf[9],
			tmpbuf[10], tmpbuf[11], tmpbuf[12], tmpbuf[13], tmpbuf[14],
			tmpbuf[15]
			);
			
			if((cmd==SIS_CMD_RECALIBRATE)&(ret==2))
				return SIS_CMD_RECALIBRATE;
			
		//xx=(tmpbuf[4]<<8) | (tmpbuf[5]);
		//yy=(tmpbuf[6]<<8) | (tmpbuf[7]);
		//printk( "x:%d, y:%d\n", xx, yy);	

// FOR SiS9200
        switch (ret)
        {
        	case NO_T:
        	case SINGLE_T:  // Has get single touch point
        	case LAST_ONE:  // Has get the last one point
        	    bReRead = false;
        	    break;
        	case MULTI_T:   // Receive 2 to 10 touch data
        		fingers = (tmpbuf[PKTINFO] & MSK_TOUCHNUM);
//				printk(KERN_INFO "chaoban test: TouchNum = %d\n", fingers);

        		if ((fingers <= 0) || (fingers > MAX_FINGERS))
        		{
        		    return -1;
        		}

        		ByteCount = 2 + (fingers * 5 ) + CRCCNT(fingers);   // Total byte count
        		ByteCount = ByteCount - ret;    // Byte counts that remain to be received
        		bReRead = ByteCount > 0 ? true : false;
        		break;
        	case LAST_TWO:  // Has get last two points
        	    ByteCount = ByteCount - ret;
        	    bReRead = ByteCount > 0 ? true : false;
        	    break;
        	default:    // SMBus Read fail or Data incorrect
        	    printk( "chaoban test: Unknow bytecount = %d\n", ret);
        		return -1;
        		break;
        }

#ifndef _SMBUS_INTERFACE
        ret = tmpbuf[0] & 0xff;
#endif
        if (ret > MAX_READ_BYTE_COUNT)
        {
            return -1;
        }
        memcpy(&buf[offset], &tmpbuf[CMD_BASE], ret);
        offset += ret;
    }
    while (bReRead);

    return ret;
}

int check_gpio_interrupt()
{
    //TODO
    //CHECK GPIO INTERRUPT STATUS BY YOUR PLATFORM SETTING.
	int ret;
	ret=gpio_get_value(S5PV210_GPH1(3));
	printk("GPIO:%d",ret);
    return !ret;  
}

static void sis_ts_work_func(struct work_struct *work)
{
	struct sis_ts_data *ts = container_of(work, struct sis_ts_data, work);
    int ret = -1;
	uint8_t buf[64] = {0};
	uint8_t i = 0, fingers = 0;
//	uint8_t crc = 0,
	uint8_t px = 0, py = 0, pstatus = 0;
	static int calibra=1;

	//printk(KERN_INFO "chaoban test: sis_ts_work_func\n");
	printk("sis_ts_work_func\n");
    if ( driver_paused )
    {
        return;
    }
	
		
    /* I2C or SMBUS block data read */
    /*if(calibra)
    {
	    calibra=0;
	    for(i=0;i<10;i++)
	    {
		    printk( "calibra++++\n");
		    mdelay(100);
		    ret = sis_ReadPacket(ts->client, SIS_CMD_RECALIBRATE, buf);
		    
		    if(ret==SIS_CMD_RECALIBRATE)
		    {
		    	enable_irq(ts->client->irq);
		    	return;
		    }
	  	}
	    return;
	  }
	  else*/
    	ret = sis_ReadPacket(ts->client, SIS_CMD_NORMAL, buf);

#if 0
	if (ret > 2)
	{   
		printk( "chaoban test: Buf_Data = %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x \r\n",
			buf[0], buf[1], buf[2], buf[3], buf[4],
			buf[5], buf[6], buf[7], buf[8], buf[9],
			buf[10], buf[11], buf[12], buf[13], buf[14],
			buf[15],
			buf[16], buf[17], buf[18], buf[19], buf[20],
			buf[21], buf[22], buf[23], buf[24], buf[25],
			buf[26], buf[27], buf[28], buf[29], buf[30],
			buf[31],
			buf[32], buf[33], buf[34], buf[35], buf[36],
			buf[37], buf[38], buf[39], buf[40], buf[41],
			buf[42], buf[43], buf[44], buf[45], buf[46],
			buf[47],
			buf[48], buf[49], buf[50], buf[51], buf[52],
			buf[53], buf[54], buf[55], buf[56], buf[57],
			buf[58], buf[59], buf[60], buf[61], buf[62],
			buf[63]
			);
	}
#endif
    
	if (ret < 0)
	{
	    printk(KERN_INFO "chaoban test: ret = -1\n");
	    enable_irq(ts->client->irq);  
		goto err_free_allocate;
	}
	else if ((ret > 2) && (!(buf[1] & MSK_HAS_CRC)))
	{
	    printk(KERN_INFO "chaoban test: command type error\n");
#if 0
		printk(KERN_INFO "chaoban test: Buf_Data = %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\r\n",
			buf[0], buf[1], buf[2], buf[3], buf[4],
			buf[5], buf[6], buf[7], buf[8], buf[9],
			buf[10], buf[11], buf[12], buf[13], buf[14],
			buf[15]);
#endif
	    enable_irq(ts->client->irq);  
		goto err_free_allocate;
	}

	sis_tpinfo_clear(TPInfo, MAX_FINGERS);

// FOR SiS9200
	/* Parser and Get the sis9200 data */
	fingers = (buf[1] & MSK_TOUCHNUM);
	TPInfo->fingers = fingers = (fingers > MAX_FINGERS ? 0 : fingers);

	for (i = 0; i < fingers; i++)
	{
        pstatus = 2 + (i * 5) + 2 * (i >> 1);    // Calc point status
	    px = pstatus + 1;                   // Calc point x_coord
	    py = px + 2;                        // Calc point y_coord

//	    printk(KERN_INFO "chaoban test: pnt = %d, ps = %d, px = %d, py = %d\n", i, pstatus, px, py);
	    
		TPInfo->pt[i].bPressure = (buf[pstatus] & MSK_PSTATE) == TOUCHDOWN ? 1 : 0;
		TPInfo->pt[i].bWidth = (buf[pstatus] & MSK_PSTATE) == TOUCHDOWN ? 1 : 0;
		TPInfo->pt[i].id = (buf[pstatus] & MSK_PID) >> 4;
		TPInfo->pt[i].x = SIS_MAX_X-(((buf[px] & 0xff) << 8) | (buf[px + 1] & 0xff));
        TPInfo->pt[i].y = (((buf[py] & 0xff) << 8) | (buf[py + 1] & 0xff));
	}

    /* Report co-ordinates to the multi-touch stack */
	i = 0;
	do 
	{
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, TPInfo->pt[i].bPressure);
//		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, TPInfo->pt[i].bWidth);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, TPInfo->pt[i].id);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, SIS_MAX_X-TPInfo->pt[i].x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, SIS_MAX_Y-TPInfo->pt[i].y);
//		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, TPInfo->pt[i].id);
		input_mt_sync(ts->input_dev);
		printk("%d,%d,%d,%d\n",TPInfo->pt[i].bPressure,TPInfo->pt[i].id,TPInfo->pt[i].x,TPInfo->pt[i].y);
		i++;
	} 
	while ((i < fingers) && (i < MAX_FINGERS));
	
	input_sync(ts->input_dev);
	
	//TODO: After interrupt status low, read i2c bus data by polling, until interrupt status is high
	ret = check_gpio_interrupt();
	if (ret)
	{
	    hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	}
	else
	{
	    //Enable irq again.
	    enable_irq(ts->client->irq);   
	}

err_free_allocate:
		 
    return;
}

static void sis_tpinfo_clear(struct sisTP_driver_data *TPInfo, int max)
{
	int i = 0;
	for(i = 0; i < max; i++)
	{
		TPInfo->pt[i].id = -1;
		TPInfo->pt[i].touch = -1;
		TPInfo->pt[i].x = 0;
		TPInfo->pt[i].y = 0;
		TPInfo->pt[i].bPressure = 0;
		TPInfo->pt[i].bWidth = 0;
	}
	TPInfo->CRC = 0x0;
}

static enum hrtimer_restart sis_ts_timer_func(struct hrtimer *timer)
{
	int ret=0;
	struct sis_ts_data *ts = container_of(timer, struct sis_ts_data, timer);
	
	printk("sis_ts_timer_func\n");

	ret=queue_work(sis_wq, &ts->work);
	//hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	
	printk("queue_work ret%d\n",ret);
	return HRTIMER_NORESTART;
}

static irqreturn_t sis_ts_irq_handler(int irq, void *dev_id)
{
	struct sis_ts_data *ts = dev_id;
	int ret=0;
	printk("sis_ts_irq_handler 1\n");
	
	//disable_irq(ts->client->irq);
	disable_irq_nosync(ts->client->irq);
	printk("sis_ts_irq_handler 2\n");

	
	queue_work(sis_wq, &ts->work);
	printk("queue_work ret%d\n",ret);
	return IRQ_HANDLED;
}

#ifdef _FOR_LEGACY_SIS81X
int i2c_read_86_data( struct i2c_client *client, unsigned int addr, u8* data )
{
    int ret;
    int i;
    unsigned char tmp;

    g_crc = 0;

    for ( i = 0; i < 4; i++ )
    {
        tmp = ( addr >> (8*(3-i)) ) & 0xff;
        data[i] = tmp;
        AppendCRC( tmp );
    }

    for ( i = 0; i < 2; i++ )
    {
        data[4+i] = ( g_crc >> (8*(1-i)) ) & 0xff;
    }

    ret = i2c_smbus_write_block_data( client, 0x86, 6, data );
    ret = i2c_smbus_read_block_data( client, 0x02, data );

    return ret;
}

void i2c_read_phase3_data( struct i2c_client *client )
{
    int ret;
    int i, j, shift;

    int* fw_mem;
    u8 *rep_data;

    rep_data = kmalloc(16,GFP_KERNEL);  //return value will be 0xabcd
    fw_mem = kmalloc( sizeof(int) * _4K_ALIGN_12, GFP_KERNEL );

    if ( !fw_mem || !rep_data )
    {
        printk( KERN_INFO "no memory for read phase3 data" );
        return;
    }

    driver_paused = 1;

    // suzin, read 0x7000 phase3 parameter

    printk( KERN_INFO "start read parameter" );

    for ( i = 0; i < _4K_PACKNUM; i++ )
    {
        fw_mem[3 * i] = fw_mem[3 * i + 1] = fw_mem[3 * i + 2] = 0;

        ret = i2c_read_86_data( client, 0xa0007020 + i * 12, rep_data );
        if ( ret < 0 )
        {
            printk( KERN_INFO "read 86 data fail, addr = %08x, ret = %d", 0xa0007020 + i * 12, ret );
            break;
        }


        for ( j = 0; j < 12; j++ )
        {

            shift = 8 * ( 3 - (j % 4) );
            fw_mem[ 3 * i + j / 4 ] |= rep_data[j] << shift;

        }
    }

    printk( KERN_INFO "finish read parameter" );
    printk( KERN_INFO "Registry Info %08x %08x", fw_mem[0], fw_mem[1] );

    if ( fw_mem[0] == 0x50383132 )
    {
        update_drivereg_from_firmware( fw_mem + 2 );
        update_global_from_firmware( fw_mem + 2 );
    }

    kfree( fw_mem );
    kfree( rep_data );

    driver_paused = 0;
}
#endif

static int sis_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sis_ts_data *ts = NULL;
	int ret = 0;
	int err=0;
	struct sis_i2c_rmi_platform_data *pdata = NULL;
    printk( "sis_ts_probe\n" );

    TPInfo = kzalloc(sizeof(struct sisTP_driver_data), GFP_KERNEL);
    if (TPInfo == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	
	
			//--------------------
	//GPH0_2 -->POWER enable
	err = gpio_request(S5PV210_GPD0(2), "GPD0");
	if (err) {
		printk(KERN_ERR "failed to request GPH0 for "
			"LVDS chip enable\n");
		return err;
	}
	
	gpio_direction_output(S5PV210_GPD0(2), 1);
	//s3c_gpio_cfgpin(S5PV210_GPD0(2), 0x1<<16);
	gpio_set_value(S5PV210_GPD0(2), 1);
	gpio_free(S5PV210_GPD0(2));
	

	ts_bak = ts;
	driver_paused = 0;

	//i2c_read_phase3_data( client );//chaoban test

	//1. Init Work queue and necessary buffers
	INIT_WORK(&ts->work, sis_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
    pdata = client->dev.platform_data;

	if (pdata)
		ts->power = pdata->power;
	if (ts->power) {
		ret = ts->power(1);
		if (ret < 0) {
			printk(KERN_ERR "sis_ts_probe power on failed\n");
			goto err_power_failed;
		}
	}

	//2. Allocate input device
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "sis_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	ts->input_dev->name = "SiS 81x family I2C touchscreen";

	set_bit(EV_ABS, ts->input_dev->evbit);
    set_bit(ABS_MT_POSITION_X, ts->input_dev->absbit);
    set_bit(ABS_MT_POSITION_Y, ts->input_dev->absbit);
    set_bit(ABS_MT_TOUCH_MAJOR, ts->input_dev->absbit);
    set_bit(ABS_MT_WIDTH_MAJOR, ts->input_dev->absbit);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, SIS_MAX_X, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, SIS_MAX_Y, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 1, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 1, 0, 0);

	//3. Register input device to core
	printk("sis 81x input_register_device -\n");
	ret = input_register_device(ts->input_dev);
	printk("sis 81x input_register_device +\n");

	if (ret) {
		printk("sis_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}
	
	client->irq=IRQ_EINT11;
	
	
	//4. irp or timer setup
	if (client->irq) {
		printk("sis 81x setup gpio \n");
		
		ret = gpio_request(S5PV210_GPH1(3), "GPH1");
		if (ret) {
			dev_err(&client->dev, "failed to request pendown GPIO%d\n",
					S5PV210_GPH1(3));
			return ret;
		}
		
		gpio_to_irq(S5PV210_GPH1(3));
		
		printk("sis 81x setup irq -\n");

	    // Set Active Low. Please reference the file include/linux/interrupt.h
		ret = request_irq(client->irq, sis_ts_irq_handler, IRQF_TRIGGER_FALLING,client->name, ts);//IRQF_TRIGGER_LOW, client->name, ts);
		
		
		
		printk("sis 81x setup irq +\n");
		//ret = 0; //Forced enable int
		if (ret == 0) {
		    // TO DO: Enable IRQ
		    ts->client->irq=client->irq;
		    //enable_irq(ts->client->irq);
		}
		if (ret == 0)
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}

	//if (!ts->use_irq) {
		printk("sis hrtimer_init\n");
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = sis_ts_timer_func;
		//hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	//}

	/*if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = sis_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}*/
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = sis_ts_early_suspend;
	ts->early_suspend.resume = sis_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	printk( "sis_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_power_failed:
	kfree(ts);
err_alloc_data_failed:
	return ret;
}

static int sis_ts_remove(struct i2c_client *client)
{
	struct sis_ts_data *ts = i2c_get_clientdata(client);
	//printk( KERN_INFO "sis_ts_remove\n" );
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int sis_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct sis_ts_data *ts = i2c_get_clientdata(client);

	if (ts->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ts->timer);
	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
		enable_irq(client->irq);
	ret = i2c_smbus_write_byte_data(ts->client, 0xf1, 0); /* disable interrupt */
	if (ret < 0)
		printk("sis_ts_suspend: i2c_smbus_write_byte_data failed\n");

	ret = i2c_smbus_write_byte_data(client, 0xf0, 0x86); /* deep sleep */
	if (ret < 0)
		printk("sis_ts_suspend: i2c_smbus_write_byte_data failed\n");
	if (ts->power) {
		ret = ts->power(0);
		if (ret < 0)
			printk("sis_ts_resume power off failed\n");
	}
	return 0;
}

static int sis_ts_resume(struct i2c_client *client)
{
	int ret;
	struct sis_ts_data *ts = i2c_get_clientdata(client);

	if (ts->power) {
		ret = ts->power(1);
		if (ret < 0)
			printk(KERN_ERR "sis_ts_resume power on failed\n");
	}

	if (ts->use_irq)
		enable_irq(client->irq);

	if (!ts->use_irq)
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	else
		i2c_smbus_write_byte_data(ts->client, 0xf1, 0x01); /* enable abs int */

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void sis_ts_early_suspend(struct early_suspend *h)
{
	struct sis_ts_data *ts;
	ts = container_of(h, struct sis_ts_data, early_suspend);
	sis_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void sis_ts_late_resume(struct early_suspend *h)
{
	struct sis_ts_data *ts;
	ts = container_of(h, struct sis_ts_data, early_suspend);
	sis_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id sis_ts_id[] = {
	{ SIS_I2C_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, sis_ts_id);


const static u16 ignore[] = { I2C_CLIENT_END };
//static const unsigned short normal_i2c[] = { 0x5, I2C_CLIENT_END };
const static u16 normal_addr[] = {0x5, I2C_CLIENT_END };
const static u16 *forces[] = { NULL };

static struct i2c_client_address_data addr_data = {
	.normal_i2c	= normal_addr,
	.probe		= ignore,
	.ignore		= ignore,
	.forces		= forces,
};


static struct i2c_driver sis_ts_driver = {
	.probe		= sis_ts_probe,
	.remove		= sis_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= sis_ts_suspend,
	.resume		= sis_ts_resume,
#endif
#ifdef FOR_X86
    .class      = I2C_CLASS_HWMON,
    .detect		= sis_ts_detect,
//	.address_list	= &addr_data,
	//.address_list	= normal_i2c,//enya .address_data	= &addr_data,
	.address_data	= &addr_data,
#endif
	.id_table	= sis_ts_id,
	.driver = {
		.name	= SIS_I2C_NAME,
	},
};

//.owner  = THIS_MODULE,
// I2C_CLASS_SPD
static int __devinit sis_ts_init(void)
{
printk( KERN_INFO "sis_ts_init\n" );

	sis_wq = create_singlethread_workqueue("sis_wq");

	if (!sis_wq)
		return -ENOMEM;
	return i2c_add_driver(&sis_ts_driver);
}

#ifdef FOR_X86
/* Return 0 if detection is successful, -ENODEV otherwise */
static int sis_ts_detect(struct i2c_client *client, /*int kind,//enya*/
		       struct i2c_board_info *info)
{
	const char *type_name;
printk( KERN_INFO "sis_ts_detect\n" );

	type_name = "sis_i2c_ts";
	info->irq = _I2C_INT_ENABLE;  //chaoban test
	strlcpy(info->type, type_name, I2C_NAME_SIZE);
	return 0;
}
#endif

static void __exit sis_ts_exit(void)
{
printk( KERN_INFO "sis_ts_exit\n" );

	i2c_del_driver(&sis_ts_driver);
	if (sis_wq)
		destroy_workqueue(sis_wq);
}

asmlinkage long sys_sis_I2C_stop( void )
{
    printk(KERN_INFO "sys_sis_I2C_stop\n" );

    driver_paused = 1;

    flush_workqueue( sis_wq );

    return 0;
}

asmlinkage long sys_sis_I2C_start( void )
{
    printk(KERN_INFO "sys_sis_I2C_start\n" );

    driver_paused = 0;

    return 0;
}

asmlinkage long sys_sis_I2C_IO( unsigned char* data, int size )
{

    unsigned char cmd = data[0];
    int ret = 0;
    int i;

    printk(KERN_INFO "sys_sis_I2C_IO, cmd=%02x\n", data[0] );

    if ( ts_bak == 0 ) return -13;


    if ( cmd == 0x83 || cmd == 0x84 || cmd == 0x85 || cmd == 0x86 )
    {
        printk(KERN_INFO "write\n" );
        ret = i2c_smbus_write_block_data( ts_bak->client, cmd, size-1, data+1 );
        ret = i2c_smbus_read_block_data( ts_bak->client, 0x02, data );
    }
    else
    {
        printk(KERN_INFO "read\n" );
        ret = i2c_smbus_read_block_data( ts_bak->client, cmd, data );
    }

    printk( KERN_INFO "%d\n", ret );

    for ( i = 0; i < ret; i++ )
        printk( "%x ", data[i] );

    printk( "\n" );

    return ret;
}

module_init(sis_ts_init);
module_exit(sis_ts_exit);

MODULE_DESCRIPTION("SiS 81x Family Touchscreen Driver");
MODULE_LICENSE("GPL");

