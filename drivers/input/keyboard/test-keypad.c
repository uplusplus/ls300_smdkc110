#include<linux/kernel.h>
#include<linux/module.h>
#include<linux/proc_fs.h>
#include<linux/string.h>
#include<linux/init.h>
#include<asm/uaccess.h> 


#include <linux/init.h>
#include <linux/input.h>
#include <linux/miscdevice.h>

#include <linux/io.h>
#include <linux/delay.h>



struct input_dev *input_dev= NULL;
struct proc_dir_entry *my_proc_ptr=NULL; 
char msg[512]; 

#define DEVICE_NAME "s3c-keypad"


int test_proc_read( char *page, char **start, off_t off,
                   int count, int *eof, void *data )
{
  printk("test_proc_read\n");

  return sprintf(page, "%s\n", msg);
}


ssize_t test_proc_write( struct file *filp, const char __user *buff,
                        unsigned long len, void *data )
{
	printk("test_proc_write\n");
	
	copy_from_user(msg,buff,len);
	
	
	input_report_key(input_dev,2,1);
	udelay(5);
	input_report_key(input_dev,2,0);
	
	printk("write data:%s\n",msg);

	return len;
}


int my_module_init( void )
{
  
  int ret;
  
  printk(KERN_INFO "my_module_init called.  Module is now loaded.\n");
  my_proc_ptr = create_proc_entry("test", S_IFREG|S_IRUGO|S_IWUSR, NULL);
	my_proc_ptr->data = (void *)0;
	my_proc_ptr->read_proc  = test_proc_read;
	my_proc_ptr->write_proc = test_proc_write;

  
  input_dev = input_allocate_device();
  
  set_bit(EV_KEY, input_dev->evbit);
  
  input_dev->name = DEVICE_NAME;
	input_dev->phys = "s3c-keypad/input0";

	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0001;
	
	ret = input_register_device(input_dev);
	if (ret) {
		printk(KERN_ERR"Unable to register s3c-keypad input device!!!\n");
		//goto out;
	}
  
  
  return 0;
}
/* Cleanup function called on module exit */
void my_module_cleanup( void )
{
  printk(KERN_INFO "my_module_cleanup called.  Module is now unloaded.\n");
  return;
}



/* Declare entry and exit functions */
module_init( my_module_init );
module_exit( my_module_cleanup );

/* Defines the license for this LKM */
MODULE_LICENSE("GPL");
/* Init function called on module entry */