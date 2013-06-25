/*
 *  dali_drv.c
 *
 *  Created on: Jun 21, 2013
 *  Author: Florian Feurstein
 *
 *  Description:
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <asm-generic/uaccess.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/slab.h>
 
#define GPIO_138            138
#define GPIO_137            137

#define EXP_HEADER_PIN_5    GPIO_138
#define EXP_HEADER_PIN_7    GPIO_137

#define DALI_IN_PORT        EXP_HEADER_PIN_5  //Expansion Header port 5. GPIO number 138. Page 108 of BB-xM Sys Ref Manual.
#define DALI_OUT_PORT       EXP_HEADER_PIN_7  //Expansion Header port 7. GPIO number 137. Page 108 of BB-xM Sys Ref Manual.

#define DALI_BIT_WAIT_TIME  416 //TODO: should be in microseconds

#define DALI_DATA_SIZE      2
 
static dev_t first;         // Global variable for the first device number
static struct cdev c_dev;     // Global variable for the character device structure
static struct class *cl;     // Global variable for the device class
 
static int init_result;
 
static ssize_t dali_read( struct file* F, char *buf, size_t count, loff_t *f_pos )
{
	char buffer[10];
	 
	//implement as blocking read
	int temp = gpio_get_value(DALI_IN_PORT);
	 
	sprintf( buffer, "%1d" , temp );
	 
	count = sizeof( buffer );
	 
	if( copy_to_user( buf, buffer, count ) )
	{
		return -EFAULT;
	}
	 
	/*if( *f_pos == 0 )
	{
		*f_pos += 1;
		return 1;
	}
	else
	{
		return 0;
	}*/
	return 0;
 
}

/*
 * dali_manchesterSendByte
 *
 * Manchester encode each bit of the byte and send
 * it to the gpio port. Keep transmission rate
 * at 1200 Hz +- 10% - 0.83 ms per logical bit,
 * about 0.41 (half of the logical bit) per
 * manchester bit
 *
 * @param byte data to encode and send
 *
 */
void dali_manchesterSendByte(char byte)
{
  int i = 0;

  for(i = 7; i >= 0; i--){
      switch( byte & (0x1 << i) )
        {
          case 0:
            gpio_set_value(DALI_OUT_PORT, 1);
            //delay(about 0.41 ms);
            mdelay(DALI_BIT_WAIT_TIME);
            gpio_set_value(DALI_OUT_PORT, 0);
            //delay(about 0.41 ms);
            mdelay(DALI_BIT_WAIT_TIME);
            printk("10");
            break;

          default: //if it's bigger than 0, it's a one
            gpio_set_value(DALI_OUT_PORT, 0);
            //delay(about 0.41 ms);
            mdelay(DALI_BIT_WAIT_TIME);
            gpio_set_value(DALI_OUT_PORT, 1);
            //delay(about 0.41 ms);
            mdelay(DALI_BIT_WAIT_TIME);
            printk("01");
          break;

//          default:
//          printk("Wrong option1. %d \n", byte & (0x1 << i));
//          break;
        }
    }
}

static ssize_t dali_write( struct file* F, const char *buf, size_t count, loff_t *f_pos )
{
 	char writeBuff[DALI_DATA_SIZE];// = kmalloc(count, GFP_KERNEL);

	if(writeBuff == NULL){
		return -1;
	}

	copy_from_user(writeBuff, buf, DALI_DATA_SIZE);

	printk(KERN_INFO "Executing WRITE.\n");

	/*
	 * Send start bit: logical 1 manchester code
	 */
	gpio_set_value(DALI_OUT_PORT, 0);
	//delay(about 0.41 ms);
	//TODO: use a timer!
	mdelay(DALI_BIT_WAIT_TIME);
	gpio_set_value(DALI_OUT_PORT, 1);
	//delay(about 0.41 ms);
	mdelay(DALI_BIT_WAIT_TIME);

	/*
	 * Send address byte
	 */
	dali_manchesterSendByte(writeBuff[0]);

	/*
   * Send data byte
   */
  dali_manchesterSendByte(writeBuff[1]);
	 
  /*
   * Send 2 stop bits (idle), no phase change for stop bits
   */
  gpio_set_value(DALI_OUT_PORT, 1);
  //delay(about 0.41 ms);
  mdelay(DALI_BIT_WAIT_TIME);
  gpio_set_value(DALI_OUT_PORT, 1);
  //delay(about 0.41 ms);
  mdelay(DALI_BIT_WAIT_TIME);
  gpio_set_value(DALI_OUT_PORT, 1);
  //delay(about 0.41 ms);
  mdelay(DALI_BIT_WAIT_TIME);
  gpio_set_value(DALI_OUT_PORT, 1);
  //delay(about 0.41 ms);
  mdelay(DALI_BIT_WAIT_TIME);

  printk("\n");

	return DALI_DATA_SIZE;
}
 
static int dali_open( struct inode *inode, struct file *file )
{
  gpio_direction_output(DALI_OUT_PORT, 1);
	return 0;
}
 
static int dali_close( struct inode *inode, struct file *file )
{
	return 0;
}
 
static struct file_operations FileOps =
{
.owner        = THIS_MODULE,
.open         = dali_open,
.read         = dali_read,
.write        = dali_write,
.release      = dali_close,
};
 
static int init_dali(void)
{ 
	/*
	 * Request and register device number
	 */
	init_result = alloc_chrdev_region( &first, 0, 1, "dali_drv" );
	 
	if( 0 > init_result )
	{
		printk( KERN_ALERT "Device Registration failed\n" );
		return -1;
	}
	 
	/*
	 * Create /sys/class
	 */
	if ( (cl = class_create( THIS_MODULE, "dali" ) ) == NULL )
	{
		printk( KERN_ALERT "Class creation failed\n" );
		unregister_chrdev_region( first, 1 );
		return -1;
	}
	 
	/*
	 * Create device /dev/dali_drv
	 */
	if( device_create( cl, NULL, first, NULL, "dali_drv" ) == NULL )
	{
		printk( KERN_ALERT "Device creation failed\n" );
		class_destroy(cl);
		unregister_chrdev_region( first, 1 );
		return -1;
	}
	 
	/*
	 * Initialize char device
	 */
	cdev_init( &c_dev, &FileOps );
	 
	if( cdev_add( &c_dev, first, 1 ) == -1)
	{
		printk( KERN_ALERT "Device addition failed\n" );
		device_destroy( cl, first );
		class_destroy( cl );
		unregister_chrdev_region( first, 1 );
		return -1;
	}

	return 0;
 
}
 
void cleanup_dali(void)
{
	 
	cdev_del( &c_dev );
	device_destroy( cl, first );
	class_destroy( cl );
	unregister_chrdev_region( first, 1 );
	 
	printk(KERN_ALERT "Device unregistered\n");
 
}
 
module_init(init_dali);
module_exit(cleanup_dali);
 
MODULE_AUTHOR("Florian Feurstein");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Beagleboard-xM DALI Driver");
