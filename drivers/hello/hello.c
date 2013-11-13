#include <linux/init.h> 
#include <linux/module.h> 
#include <linux/types.h> 
#include <linux/fs.h> 
#include <linux/proc_fs.h> 
#include <linux/device.h> 
#include <asm/uaccess.h>
#include <asm/io.h>
#include "hello.h"

volatile unsigned long *gpjcon = NULL;
volatile unsigned long *gpjdat = NULL;


/*主设备和从设备号变量*/   
static  int hello_major = 0; 
static  int hello_minor = 0; 
/*设备类别和设备变量*/ 
static  struct  class* hello_class = NULL; 
static  struct  hello_android_dev* hello_dev = NULL; 
/*传统的设备文件操作方法*/ 
static  int hello_open(struct inode* inode, struct file* filp); 
static  int hello_release(struct inode* inode, struct file* filp); 
static  ssize_t   hello_read(struct file* filp, char __user *buf, size_t  count, loff_t* f_pos); 
static  ssize_t   hello_write(struct file* filp, const char __user *buf, size_t  count, 
		loff_t* f_pos); 
/*设备文件操作方法表*/ 
static  struct  file_operations hello_fops = { 
	.owner = THIS_MODULE, 
	.open = hello_open, 
	.release = hello_release, 
	.read = hello_read, 
	.write = hello_write, 
}; 
/*访问设置属性方法*/ 
static  ssize_t   hello_val_show(struct device* dev, struct device_attribute* attr, 
		char* buf); 
static  ssize_t   hello_val_store(struct device* dev, struct device_attribute* attr, 
		const char* buf, size_t  count); 
/*定义设备属性*/   
static  DEVICE_ATTR(val, S_IRUGO | S_IWUSR, hello_val_show, hello_val_store);

//make sure the gpio is initialized
static int init_gpio(){
    static int is_inited = 0;
    if(is_inited) return 0;
    if(gpjcon){
        *gpjcon &= ~((0xf<<(6*4)) | (0xf<<(7*4)));//将第24-27bit和28-31bit清零
        *gpjcon |= ((0x1<<(6*4))  | (0x1<<(7*4)));//将第24-27bit和28-31bit置一
        is_inited = 1;
        return 0;
    }
    return -1;
}

/*打开设备方法*/ 
static  int hello_open(struct inode* inode, struct file* filp) { 
	struct hello_android_dev* dev; 
	dev = container_of(inode->i_cdev, struct hello_android_dev, dev); 
	filp->private_data = dev; 

	printk("led driver_open\n");
    return  init_gpio();
} 
/*设备文件释放时调用，空实现*/ 
static  int hello_release(struct inode* inode, struct file* filp) { 
	return 0; 
} 
/*读取设备的寄存器 val 的值*/ 
static ssize_t   hello_read(struct file* filp, char __user *buf, size_t  count, loff_t* 
		f_pos) { 
	ssize_t   err = 0; 
	struct hello_android_dev* dev = filp->private_data; 
	/*同步访问*/
	if(down_interruptible(&(dev->sem))) { 
		return -ERESTARTSYS; 
	}   
	if(count < sizeof(dev->val)) { 
		goto out; 
	} 
	/*将寄存器 val 的值拷贝到用户提供的缓冲区*/
	if(copy_to_user(buf, &(dev->val), sizeof(dev->val))) { 
		err = -EFAULT; 
		goto out; 
	} 
	err = sizeof(dev->val); 
out: 
	up(&(dev->sem)); 
	return err; 
} 
/*写设备的寄存器值 val*/ 
static  ssize_t   hello_write(struct file* filp, const char __user *buf, size_t  count, 
		loff_t* f_pos) { 
	struct  hello_android_dev* dev = filp->private_data; 
	ssize_t   err = 0; 
	/*同步访问*/
	if(down_interruptible(&(dev->sem))) { 
		return  -ERESTARTSYS; 
	} 
	if(count != sizeof(dev->val)) {   
		goto  out; 
	} 
	/*将用户提供的缓冲区的值写到设备寄存器去*/
	if(copy_from_user(&(dev->val), buf, count)) { 
		err = -EFAULT; 
		goto  out; 
	} 

	printk("led driver_write,dev->val=%d\n",dev->val);

	if (dev->val == 1)
	{
		// 点灯
		*gpjdat |= (1<<6) | (1<<7);
	}
	else
	{
		// 灭灯
		*gpjdat &= ~((1<<6) | (1<<7));
	}

	err = sizeof(dev->val); 
out: 
	up(&(dev->sem)); 
	return  err; 
} 


/*读取寄存器 val 的值到缓冲区 buf 中，内部使用*/ 
static ssize_t  __hello_get_val(struct hello_android_dev* dev, char* buf) { 
	int val = 0; 
	/*同步访问*/
	if(down_interruptible(&(dev->sem))) { 
		return -ERESTARTSYS; 
	} 
	val = dev->val;   
	up(&(dev->sem)); 
	return snprintf(buf, PAGE_SIZE, "%d\n", val); 
} 
/*把缓冲区 buf 的值写到设备寄存器 val 中去，内部使用*/ 
static ssize_t  __hello_set_val(struct hello_android_dev* dev, const char* buf, 
		size_t count) { 
	int val = 0; 
	/*将字符串转换成数字*/ 
	val = simple_strtol(buf, NULL, 10); 
	/*同步访问*/
	if(down_interruptible(&(dev->sem))) { 
		return -ERESTARTSYS; 
	} 
	dev->val = val; 
	
	printk("led driver_write,dev->val=%d,gpjdat=%d,addr gpjdat=%X\n",dev->val,*gpjdat,gpjdat);
	if(!init_gpio()) {
	    if (dev->val == 1)
	    {
		    // 点灯
		    *gpjdat |= (1<<6) | (1<<7);
	    }
	    else
	    {
		    // 灭灯
		    *gpjdat &= ~((1<<6) | (1<<7));
	    }
	    printk("led driver_write done,gpjdat=%d\n",*gpjdat);
	}
	
	up(&(dev->sem)); 
	return count; 
} 
/*读取设备属性 val*/ 
static ssize_t  hello_val_show(struct device* dev, struct device_attribute* attr, 
		char* buf) { 
	struct hello_android_dev* hdev = (struct hello_android_dev*)dev_get_drvdata(dev); 
	return __hello_get_val(hdev, buf);   
} 
/*写设备属性 val*/ 
static ssize_t  hello_val_store(struct device* dev, struct device_attribute* attr, 
		const char* buf, size_t count) { 
	struct hello_android_dev* hdev = (struct hello_android_dev*)dev_get_drvdata(dev); 
	return __hello_set_val(hdev, buf, count); 
}

/*读取设备寄存器 val 的值，保存在 page 缓冲区中*/ 
static ssize_t  hello_proc_read(char* page, char** start, off_t off, int count, int* 
		eof, void* data) { 
	if(off > 0) { 
		*eof = 1; 
		return 0; 
	} 
	return __hello_get_val(hello_dev, page); 
} 
/*把缓冲区的值 buff 保存到设备寄存器 val 中去*/ 
static ssize_t  hello_proc_write(struct file* filp, const char __user *buff, unsigned 
		long len, void* data) { 
	int err = 0; 
	char* page = NULL; 
	if(len > PAGE_SIZE) { 
		printk(KERN_ALERT"The buff is too large: %lu.\n", len); 
		return -EFAULT; 
	} 
	page = (char*)__get_free_page(GFP_KERNEL); 
	if(!page) { 
		printk(KERN_ALERT"Failed to alloc page.\n"); 
		return -ENOMEM; 
	} 
	/*先把用户提供的缓冲区值拷贝到内核缓冲区中去*/
	if(copy_from_user(page, buff, len)) { 
		printk(KERN_ALERT"Failed to copy buff from user.\n"); 
		err = -EFAULT; 
		goto out; 
	} 
	err = __hello_set_val(hello_dev, page, len); 
out: 
	free_page((unsigned long)page); 
	return err; 
} 
/*创建/proc/hello 文件*/ 
static  void hello_create_proc(void) { 
	struct proc_dir_entry* entry; 
	entry = create_proc_entry(HELLO_DEVICE_PROC_NAME, 0, NULL);
	if(entry) { 
		//It appears that the kernel developers have removed the owner field from procfs because it is
		//"racy" and it is now implicitly set by other operations. The fix is to simply remove attempts
		//to set the owner.
		//entry->owner = THIS_MODULE; 
		entry->read_proc = hello_proc_read; 
		entry->write_proc = hello_proc_write; 
	} 
} 
/*删除/proc/hello 文件*/ 
static  void hello_remove_proc(void) { 
	remove_proc_entry(HELLO_DEVICE_PROC_NAME, NULL); 
} 
//最后，定义模块加载和卸载方法，这里只要是执行设备注册和初始化操作： 
/*初始化设备*/ 
static  int __hello_setup_dev(struct hello_android_dev* dev) { 
	int err; 
	dev_t devno = MKDEV(hello_major, hello_minor); 
	memset(dev, 0, sizeof(struct hello_android_dev)); 
	cdev_init(&(dev->dev), &hello_fops); 
	dev->dev.owner = THIS_MODULE; 
	dev->dev.ops = &hello_fops; 
	/*注册字符设备*/ 
	err = cdev_add(&(dev->dev),devno, 1); 
	if(err) { 
		return err;   
	} 
	/*初始化信号量和寄存器 val 的值*/ 
	init_MUTEX(&(dev->sem)); 
	dev->val = 0; 
	return 0; 
} 
/*模块加载方法*/ 
static  int __init hello_init(void){ 
	int err = -1; 
	dev_t dev = 0; 
	struct device* temp = NULL; 
	printk(KERN_ALERT"Initializing hello device.\n"); 
	/*动态分配主设备和从设备号*/ 
	err = alloc_chrdev_region(&dev,0, 1, HELLO_DEVICE_NODE_NAME); 
	if(err < 0) { 
		printk(KERN_ALERT"Failed to alloc char dev region.\n"); 
		goto fail; 
	} 
	hello_major = MAJOR(dev); 
	hello_minor = MINOR(dev); 
	/*分配 helo 设备结构体变量*/ 
	hello_dev = kmalloc(sizeof(struct hello_android_dev), GFP_KERNEL);   
	if(!hello_dev) { 
		err = -ENOMEM; 
		printk(KERN_ALERT"Failed to alloc hello_dev.\n"); 
		goto unregister; 
	} 
	/*初始化设备*/ 
	err = __hello_setup_dev(hello_dev); 
	if(err) { 
		printk(KERN_ALERT"Failed to setup dev: %d.\n", err); 
		goto cleanup; 
	} 
	/*在/sys/class/目录下创建设备类别目录 hello*/ 
	hello_class = class_create(THIS_MODULE, HELLO_DEVICE_CLASS_NAME); 
	if(IS_ERR(hello_class)) { 
		err = PTR_ERR(hello_class); 
		printk(KERN_ALERT"Failed to create hello class.\n"); 
		goto destroy_cdev; 
	} 
	/*在/dev/目录和/sys/class/hello 目录下分别创建设备文件 hello*/ 
	temp = device_create(hello_class, NULL, dev, "%s", 
			HELLO_DEVICE_FILE_NAME); 
	if(IS_ERR(temp)) { 
		err = PTR_ERR(temp); 
		printk(KERN_ALERT"Failed to create hello device."); 
		goto destroy_class; 
	} 
	/*在/sys/class/hello/hello 目录下创建属性文件 val*/ 
	err = device_create_file(temp, &dev_attr_val); 
	if(err < 0) { 
		printk(KERN_ALERT"Failed to create attribute val."); 
		goto destroy_device; 
	} 
	dev_set_drvdata(temp, hello_dev); 
	/*创建/proc/hello 文件*/ 
	hello_create_proc(); 

	printk("Led init:ioremap"); 
	gpjcon = (volatile unsigned long *)ioremap(0xe02002a0, 4);
	gpjdat = (volatile unsigned long *)ioremap(0xe02002a4, 1);

	printk(KERN_ALERT"Succedded to initialize hello device.\n"); 
	return 0; 
destroy_device: 
	device_destroy(hello_class, dev); 
destroy_class: 
	class_destroy(hello_class); 
destroy_cdev: 
	cdev_del(&(hello_dev->dev)); 
cleanup: 
	kfree(hello_dev); 
unregister: 
	unregister_chrdev_region(MKDEV(hello_major, hello_minor), 1);   
fail: 
	return err; 
} 
/*模块卸载方法*/ 
static  void __exit hello_exit(void) { 
	dev_t devno = MKDEV(hello_major, hello_minor); 
	printk(KERN_ALERT"Destroy hello device.\n"); 
	/*删除/proc/hello 文件*/ 
	hello_remove_proc(); 
	/*销毁设备类别和设备*/
	if(hello_class) { 

		device_destroy(hello_class, MKDEV(hello_major, hello_minor)); 
		class_destroy(hello_class); 
	} 
	/*删除字符设备和释放设备内存*/
	if(hello_dev) { 
		cdev_del(&(hello_dev->dev)); 
		kfree(hello_dev); 
	} 
	/*释放设备号*/ 
	unregister_chrdev_region(devno, 1); 

	iounmap(gpjcon);
	iounmap(gpjdat);

}   
MODULE_LICENSE("GPL"); 
MODULE_DESCRIPTION("First Android Driver"); 
module_init(hello_init); 
module_exit(hello_exit); 
