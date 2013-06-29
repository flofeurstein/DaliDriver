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
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/hrtimer.h>

#define GPIO_138            138
#define GPIO_137            137

#define EXP_HEADER_PIN_5    GPIO_138
#define EXP_HEADER_PIN_7    GPIO_137

#define DALI_IN_PORT        EXP_HEADER_PIN_5  //Expansion Header port 5. GPIO number 138. Page 108 of BB-xM Sys Ref Manual.
#define DALI_OUT_PORT       EXP_HEADER_PIN_7  //Expansion Header port 7. GPIO number 137. Page 108 of BB-xM Sys Ref Manual.

/*
 * Microseconds from sending one bit to the next.
 * Frequency for DALI is 1200 Hz
 * -> 1s/1200 = 0.000833 s = 833 us = 833333 ns
 *
 */
#define DALI_FREQ_SEC       0
#define DALI_FREQ_NS        833333

#define DALI_DATA_SIZE      2

#define DALI_STOPBIT_VAL    3
 
static dev_t first;         // Global variable for the first device number
static struct cdev c_dev;     // Global variable for the character device structure
static struct class *cl;     // Global variable for the device class
 
static int init_result;

typedef struct manchesterBitValList_t {
  struct manchesterBitValList_t *pNext;
  uint8_t bitVal;
}manchesterBitValList_t;

static manchesterBitValList_t* pBitValRoot = NULL;

//hrtimer
static struct hrtimer high_res_timer;
#define DALI_TIMER_MODE     HRTIMER_MODE_REL
ktime_t dali_freq_time;

static void dali_start_timer(void)
{
  hrtimer_start(&high_res_timer, dali_freq_time, DALI_TIMER_MODE);
}

static enum hrtimer_restart dali_timerCB(struct hrtimer * hrtimer)
{
  ktime_t now;
  manchesterBitValList_t* pTemp = NULL;

  if(pBitValRoot != NULL)
  {
    gpio_set_value(DALI_OUT_PORT, pBitValRoot->bitVal);
    printk("Bit val %x \n", pBitValRoot->bitVal);

    if(pBitValRoot->pNext != NULL)
    {
      pTemp = pBitValRoot;
      pBitValRoot = pTemp->pNext;
      kfree(pTemp);
      pTemp = NULL;
      now = hrtimer_cb_get_time(&high_res_timer);
      hrtimer_forward(&high_res_timer,now , dali_freq_time);
      return HRTIMER_RESTART;
    }
    else
    {
      kfree(pBitValRoot);
      pBitValRoot = NULL;
    }
  }
  return HRTIMER_NORESTART;
}
 

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

	return 0;
 
}


static void dali_manchesterListAddVal(uint8_t val)
{
  manchesterBitValList_t* pTemp = kmalloc(sizeof(manchesterBitValList_t), GFP_KERNEL);
  pTemp->pNext = kmalloc(sizeof(manchesterBitValList_t), GFP_KERNEL);
  pTemp->pNext->pNext = NULL;
  switch(val)
    {
      case 0:
        /*
         * logical 0 is the transition from 1 to 0
         * because we
         */
        pTemp->bitVal = 0;
        pTemp->pNext->bitVal = 1;
        break;

      case DALI_STOPBIT_VAL:
        pTemp->bitVal = 1;
        pTemp->pNext->bitVal = 1;
        break;

      default: //if it's bigger than 0, it's a one
        pTemp->bitVal = 1;
        pTemp->pNext->bitVal = 0;
      break;
    }

  if(pBitValRoot != NULL){
    pTemp->pNext->pNext = pBitValRoot;
  }
  pBitValRoot = pTemp;

}

/*
 * dali_manchesterListAddByte
 *
 * Manchester encode each bit of the byte and add it to
 * the manchesterBitValList
 *
 * @param byte data to encode and send
 *
 */
static void dali_manchesterListAddByte(char byte)
{
  int i = 0;

  for(i = 0; i < 8; i++)
  {
    dali_manchesterListAddVal(byte & (0x1 << i));
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
	 * Fill the Manchester list from behind (stop bits first, then data byte and address byte, start
	 * bit last) to get a properly ordered list (adding a node to the
	 * list adds it to the front!)
	 *
	 */

	/*
   * Send 2 stop bits (idle), no phase change for stop bits
   */
	dali_manchesterListAddVal(DALI_STOPBIT_VAL);
	dali_manchesterListAddVal(DALI_STOPBIT_VAL);

	/*
   * Send data byte
   */
	dali_manchesterListAddByte(writeBuff[1]);

	/*
   * Send address byte
   */
	dali_manchesterListAddByte(writeBuff[0]);

	/*
	 * Send start bit: logical 1 manchester code
	 */
	dali_manchesterListAddVal(1);

  printk("start timer \n");

  dali_start_timer();

  printk("timer started \n");

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

	/*
	 * Initialize high res timer
	 */
	hrtimer_init(&high_res_timer, CLOCK_MONOTONIC, DALI_TIMER_MODE);
	dali_freq_time = ktime_set(DALI_FREQ_SEC, DALI_FREQ_NS);
	high_res_timer.function = dali_timerCB;

	return 0;
 
}
 
void cleanup_dali(void)
{

  if(hrtimer_try_to_cancel(&high_res_timer) < 0)
  {
    printk(KERN_ALERT "Timer not stopped!\n");
  }
	 
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
