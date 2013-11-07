/*
    i2c-flash.c - i2c-bus driver, char device interface for EEPROM 24FC256

	   The 2nd task of the assignment is to build an i2c client driver for EEPROM such that user programs
	can invoke read, write, and seek (as a lseek function) driver operations. You can assume the EEPROM
	chip has a fixed I2C address and is connected to a specific i2c bus. Thus, when the module is initialized,
	the EEPROM device should be created and named as “i2c_flash”. The module should enable device file
	operations:

	1. int open(const char *pathname, int flags);
	2. ssize_t read(int fd, void *buf, size_t count);
	3. ssize_t write(int fd, const void *buf, size_t count);
	4. off_t lseek(int fd, off_t offset, int whence);	 whence= SEEK_SET
	5. int close(int fd);

	where count and offset are page number of the EEPROM memory and is ranged from 0 to 2k-1, and
	the size of the “buf” should be 64*count bytes. Also, the read and write calls are blocking calls, i.e., they
	will be returned only when the operations are done or have an error. Hence, the calling user thread is
	blocked while the requested operation is in progress.
 */

/* The I2C_RDWR ioctl code is written by Kolja Waschk <waschk@telos.de> */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

/*
 * An i2c_dev represents an i2c_adapter ... an I2C or SMBus master, not a
 * slave (i2c_client) with which messages will be exchanged.  It's coupled
 * with a character special file which is accessed by user mode drivers.
 *
 * The list of i2c_dev structures is parallel to the i2c_adapter lists
 * maintained by the driver model, and is updated using bus notifications.
 */

typedef struct {				// work structure for workqueues
	struct work_struct  my_work;
	int count;
	char *buffer;
	struct i2c_client *client;
} my_work_t;


struct i2c_dev {
	struct list_head list;
	struct i2c_adapter *adap;
	struct device *dev;
	int I2C_EEPROM_ADDR;
	int EEPROM_PAGE_SIZE;
	uint8_t HIGHER_BYTE;
	uint8_t LOWER_BYTE;
	int SLEEP_INTERVAL;			// sleep after every read and write for SLEEP_INTERVAL seconds
	struct workqueue_struct *MY_WQ;		// work queue
	my_work_t  *READ_WORK;
	my_work_t  *WRITE_WORK;
	int is_read_work_ready;
	int is_write_work_ready;
};

#define DEVICE_NAME  "i2c-flash"					 /* device would be named as /dev/gmem	*/

#define I2C_MINORS	256
static LIST_HEAD(i2c_dev_list);
static DEFINE_SPINLOCK(i2c_dev_list_lock);

/*
 * basically returns the i2c_dev in the i2c_dev_list which is bound to adapter number index
 */
static struct i2c_dev *i2c_dev_get_by_minor(unsigned index)
{
	struct i2c_dev *i2c_dev;
	//printk("i2c_dev_get_by_minor()\n");
	spin_lock(&i2c_dev_list_lock);
	list_for_each_entry(i2c_dev, &i2c_dev_list, list) {
		if (i2c_dev->adap->nr == index)
			goto found;
	}
	i2c_dev = NULL;
	found:
	spin_unlock(&i2c_dev_list_lock);
	return i2c_dev;
}

/*
 * allocates memory for i2c_dev and also adds it to the i2c_dev_list
 */
static struct i2c_dev *get_free_i2c_dev(struct i2c_adapter *adap)
{
	struct i2c_dev *i2c_dev;

	if (adap->nr >= I2C_MINORS) {
		printk(KERN_ERR "i2c-dev: Out of device minors (%d)\n",
				adap->nr);
		return ERR_PTR(-ENODEV);
	}

	i2c_dev = kzalloc(sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return ERR_PTR(-ENOMEM);
	i2c_dev->adap = adap;

	spin_lock(&i2c_dev_list_lock);
	list_add_tail(&i2c_dev->list, &i2c_dev_list);
	spin_unlock(&i2c_dev_list_lock);
	return i2c_dev;
}

/*
 * deletes i2c_dev from the i2c_dev_list
 */
static void return_i2c_dev(struct i2c_dev *i2c_dev)
{
	spin_lock(&i2c_dev_list_lock);
	list_del(&i2c_dev->list);
	spin_unlock(&i2c_dev_list_lock);
	kfree(i2c_dev);
}


static ssize_t show_adapter_name(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	//struct i2c_dev *i2c_dev = i2c_dev_get_by_minor(MINOR(dev->devt));
	struct i2c_dev *i2c_dev = i2c_dev_get_by_minor(2);		//	i2c_dev bound to adap-nr=2

	if (!i2c_dev)
		return -ENODEV;
	return sprintf(buf, "%s\n", i2c_dev->adap->name);
}
static DEVICE_ATTR(name, S_IRUGO, show_adapter_name, NULL);

/* ------------------------------------------------------------------------- */


/*
 * updates the global address maintained by LOWER_BYTE and HIGHER_BYTE by incrementing by 64 ie PAGE SIZE
 * and wraps around if its the last page ie 32704
 */
void updateAddress(struct i2c_dev *i2c_dev){

	uint address;
	address = i2c_dev->LOWER_BYTE + ((i2c_dev->HIGHER_BYTE)<<8);
	if (address >= 32704)
		//last page starts from addr 32704 to 32767
		address = 0;
	else
		address += 64;

	i2c_dev->LOWER_BYTE = address & 255;
	i2c_dev->HIGHER_BYTE = address >> 8;
	printk("EEPROM Address updated to %d\n", i2c_dev->LOWER_BYTE + ((i2c_dev->HIGHER_BYTE)<<8));
}


/*
 * After opening an instance of this character special file, a file
 * descriptor starts out associated only with an i2c_adapter (and bus).
 *
 * Using the I2C_RDWR ioctl(), you can then *immediately* issue i2c_msg
 * traffic to any devices on the bus used by that adapter.  That's because
 * the i2c_msg vectors embed all the addressing information they need, and
 * are submitted directly to an i2c_adapter.  However, SMBus-only adapters
 * don't support that interface.
 *
 * To use read()/write() system calls on that file descriptor, or to use
 * SMBus interfaces (and work with SMBus-only hosts!), you must first issue
 * an I2C_SLAVE (or I2C_SLAVE_FORCE) ioctl.  That configures an anonymous
 * (never registered) i2c_client so it holds the addressing information
 * needed by those system calls and by this SMBus interface.
 */

static int my_wq_read_function( struct work_struct *work)	// function to be call
{
	int ret,i, no_of_bytes, page_size, k;
	struct i2c_dev *i2c_dev;
	my_work_t *my_work = (my_work_t *)work;
	i2c_dev = i2c_dev_get_by_minor(2);
	if(!i2c_dev){
		printk("Bad i2c_dev\n");
		return -1;
	}

	page_size = i2c_dev->EEPROM_PAGE_SIZE;
	no_of_bytes = (my_work->count)*page_size;
	printk("no_of_bytes to read=%d\n", no_of_bytes);

	for(i=0; i<my_work->count; i++){
		if ( (ret=i2c_master_recv(my_work->client, (my_work->buffer)+(i*page_size), page_size)) != page_size) {
			printk("Error: Failed to read from the i2c bus: res=%d\n",ret);
			return -1;
		}
		printk("read successful at address=%d\n", i2c_dev->LOWER_BYTE + ((i2c_dev->HIGHER_BYTE)<<8));
		for(k=0; k<page_size; k++){									/*	print what you read	*/
			printk("%c", my_work->buffer[i*page_size+k]);
		}
		printk("\n");
		msleep(i2c_dev->SLEEP_INTERVAL);
		updateAddress(i2c_dev);
	}
	printk("READ WORK IS READY !!!\n");
	i2c_dev->is_read_work_ready = 1;
	return no_of_bytes;
}


/*
 *	When a read system call is
 *	requested to the EEPROM device, the driver takes one of the following operations:
 *
 *	-- If no EEPROM data is ready for read and no pending work request, the driver submits a read
 *	   transfer request to the work queue and returns immediately with -2.
 *	-- If no EEPROM data is ready for read and the work request submitted last time is still in
 *	   processing, the call returns immediately with -1. This condition just indicates that the EEPROM is
 *     busy.
 *	-- If EEPROM data is ready for read, return with 0 after the data of 64*count bytes is copied to the
 * 	   buffer.
 *
 * 	   NOTE -->
 *	   	 	Work could be in two states - 1. PENDING(work is in the queue but hasn't started yet) and 2. BUSY WORKING(in the queue and working)
 *	    	PENDING can be checked by work_pending() whereas work_busy() == 0 shows neither pending nor busy working.
 *
 *	    	WORK_BUSY_PENDING=1, WORK_BUSY_RUNNING=2
 */
static ssize_t i2cdev_read(struct file *file, char __user *buf, size_t count, loff_t *offset){

	int ret, no_of_bytes, page_size, h;
	struct i2c_dev *i2c_dev;
	printk("-------------------------------i2cdev_read()---------------------------------\n");

	if(!buf || (count<1)){
		printk("buffer is NULL or count<1\n");
		return -1;
	}
	ret = 10;								/* default return value	*/
	i2c_dev = i2c_dev_get_by_minor(2);
	//printk("\n=========> is_read_work_ready=%d, work_pending()=%d, work_busy()=%d \n", i2c_dev->is_read_work_ready, work_pending( &(i2c_dev->READ_WORK->my_work)), work_busy( &(i2c_dev->READ_WORK->my_work)) );

	/* If EEPROM data is ready for read, return with 0 after the data of 64*count bytes is copied to the buffer.*/
	if(i2c_dev->is_read_work_ready){
		printk("********READ WORK IS READY\n");
		page_size = i2c_dev->EEPROM_PAGE_SIZE;
		no_of_bytes = count*(page_size);
		for(h=0; h<no_of_bytes; h++){						/*	verify what have you read	*/
			printk("%c", i2c_dev->READ_WORK->buffer[h]);
		}
		printk("\n");
		copy_to_user(buf, i2c_dev->READ_WORK->buffer, no_of_bytes) ? -EFAULT : no_of_bytes;
		kfree(i2c_dev->READ_WORK->buffer);

		i2c_dev->is_read_work_ready = 0;								/* reset flag	*/
		printk("^^^^^^RETURNING after COPYING TO USER\n");
		return 0;
	}
	else /* If no EEPROM data is ready for read and no pending work request,submit a read  request to the work queue and returns immediately with -2.*/
		if(!i2c_dev->is_read_work_ready && work_busy( &(i2c_dev->READ_WORK->my_work)) == 0 ){
			printk("*******ASSIGN READ work\n");
			i2c_dev->READ_WORK->count = count;							/*	set structs for READ_WORK */
			i2c_dev->READ_WORK->client =  file->private_data;
			i2c_dev->READ_WORK->buffer = kmalloc(count*(i2c_dev_get_by_minor(2)->EEPROM_PAGE_SIZE), GFP_KERNEL);
			ret = queue_work( i2c_dev->MY_WQ, (struct work_struct *)(i2c_dev->READ_WORK));
			printk("^^^^^^RETURNING -2 after ASSIGNING READ WORK\n");
			return -2;
		}
		else
			/* If no EEPROM data is ready for read and the work request submitted last time is still in processing, the call returns immediately with -1.*/
			if(!i2c_dev->is_read_work_ready && (work_busy( &(i2c_dev->READ_WORK->my_work)) > 1) ){
				printk("*****BUSY WORKING so RETURNING -1\n");
				return -1;
			}
			else	/*	shouldn't happen	*/
				printk("****WHERE AM I?\n");

	/*	shouldn't happen	*/
	printk("^^^^RETURNING ret=%d\n", ret);
	return ret;
}


/*
 * helper function to set page_number of EEPROM
 *
 * returns   -1 , 			if fails
 * 			 page_number , 	if successful
 *
 * 			called by i2cdev_llseek() and i2c_write()
 */
int seek_EEPROM(int page_number, struct i2c_client *client ){
	uint address;
	int page_size, res, ret;
	struct i2c_dev *i2c_dev;
	char buffer[2];

	printk("seek_EEPROM at page_number=%d\n", page_number);
	i2c_dev = i2c_dev_get_by_minor(2);
	page_size = i2c_dev->EEPROM_PAGE_SIZE;

	if(page_number<0 || page_number>511){										// total PAGE COUNT = 512 (0 to 511)
		printk("Invalid offset or page number\n");
		return -1;
	}

	address = page_number*page_size;
	i2c_dev->LOWER_BYTE = address & 255;
	i2c_dev->HIGHER_BYTE = address >> 8;

	buffer[0] = i2c_dev->HIGHER_BYTE;
	buffer[1] = i2c_dev->LOWER_BYTE;

	res = i2c_master_send(client, buffer, 2);
	msleep(i2c_dev->SLEEP_INTERVAL);
	if(res==2){
		printk("seek successful at address=%d\n", address);
		ret = page_number;
	}
	else{
		printk("seek unsuccessful, res=%d\n", res);
		ret = -1;
	}
	return ret;
}


/*
 * returns   -1 , 			if fails
 * 			 page_number , 	if successful
 *
 */
loff_t i2cdev_llseek(struct file *file, loff_t off, int whence){

	int page_size;
	struct i2c_dev *i2c_dev;
	int res;
	int page_number;
	loff_t newpos;
	struct i2c_client *client = file->private_data;

	printk("i2cdev_llseek()\n");
	i2c_dev = i2c_dev_get_by_minor(2);
	if(!i2c_dev){
		printk("Bad i2c_dev\n");
		return -1;
	}

	page_size = i2c_dev->EEPROM_PAGE_SIZE;
	printk("seek at page number=%d\n", (int)off);

	switch(whence) {
	case 0: /* SEEK_SET */
		printk("SEEK_SET\n");
		page_number = off;				//off is page number
		if((res=seek_EEPROM(page_number, client)) == -1)									// returns page number or -1
			newpos=-1;
		else
			newpos = res;

		break;

	case 1: /* SEEK_CUR */
	case 2: /* SEEK_END */

	default:
		printk("whence != SEEK_SET\n");
		return -EINVAL;
	}

	if (newpos<0) return -EINVAL;

	return newpos;
}


static int my_wq_write_function( struct work_struct *work)	// function to be call
{
	int ret,i, page_size, k;
	struct i2c_dev *i2c_dev;
	char *tmp;
	uint address;
	my_work_t *my_work = (my_work_t *)work;			//	work is the first member of my-work_t hence same address.

	i2c_dev = i2c_dev_get_by_minor(2);
	if(!i2c_dev){
		printk("Bad i2c_dev\n");
		return -1;
	}

	page_size = i2c_dev->EEPROM_PAGE_SIZE;
	printk("Writing count=%d pages\n", my_work->count);

	ret=0;
	tmp = kmalloc(page_size+2, GFP_KERNEL);
	for(i=0; i<my_work->count; i++){
		tmp[0] = i2c_dev->HIGHER_BYTE;						// append two byte address while writing
		tmp[1] = i2c_dev->LOWER_BYTE;

		memcpy(tmp+2, (my_work->buffer + (i*page_size)), page_size);
		printk("Writing -->");								/*	verifying data before writing	*/
		for(k=0; k<page_size; k++){
			printk("%c", tmp[2+i]);
		}
		printk("\n");

		ret += i2c_master_send(my_work->client, tmp, page_size+2);
		printk("Wrote %zu bytes successfully at address %d\n", page_size+2, i2c_dev->LOWER_BYTE + ((i2c_dev->HIGHER_BYTE)<<8));
		msleep(i2c_dev->SLEEP_INTERVAL);
		updateAddress(i2c_dev);				// address updated in i2c-flash driver
	}

	kfree(tmp);										/* can free tmp now	*/
	if(ret == my_work->count*(page_size+2))			/*	readjust ret as it would contain address bytes as well	*/
		ret = my_work->count*page_size;
	else ret = -1;

	/*	after every write, address pointer of the user-app points to the next page but the internal address pointer of EEPROM
		is still pointing to the address set by the last write eg. Lets say initially user-app addr ptr is at 0, then after 2 writes,
		user-app addr ptr is at 2, but EEPROM addr ptr will be at 1
	 */
	address = i2c_dev->LOWER_BYTE + ((i2c_dev->HIGHER_BYTE)<<8);	// address is updated in i2c-flash driver above
	printk("SEEK after WRITE, UPDATE EEPROM internal ptr to address=%d\n", address);
	seek_EEPROM(address/page_size, my_work->client);

	i2c_dev->is_write_work_ready = (ret==-1? -1 : 1);						// set the flag accordingly
	printk("************WRITE WORK IS READY = %d\n", i2c_dev->is_write_work_ready);
	return 0;
}

/*
 * 		when a write system call is requested, the driver:
 *		--> Issues a write request to the work queue to perform the write operation, and returns
 *			immediately with -2, if EEPROM is not busy and no pending work request.
 *		-->	Returns immediately with -1 to indicate the EEPROM is busy of the previous operation.
 *		--> Returns immediately with 0 if the previous operation has been done successfully, or -3 if failed.
 *
 *		 NOTE -->
 *	   		 Work could be in two states - 1. PENDING(work is in the queue but hasn't started yet) and 2. BUSY WORKING(in the queue and working)
 *	    	 PENDING can be checked by work_pending() whereas work_busy(&(WRITE_WORK->my_work)) == 0 shows neither pending nor busy working.
 */
static ssize_t i2cdev_write(struct file *file, const char __user *buf, size_t count, loff_t *offset){

	int ret, page_size, res;
	struct i2c_dev *i2c_dev;
	ret=10;
	printk("----------------------------------i2cdev_write()----------------------------------------\n");

	if(!buf || (count<1)){
		printk("buffer is NULL or count<1\n");
		return -1;
	}
	i2c_dev = i2c_dev_get_by_minor(2);
	//	printk("\n=========> is_write_work_ready=%d, work_pending()=%d, work_busy()=%d \n", i2c_dev->is_write_work_ready, work_pending( &(i2c_dev->WRITE_WORK->my_work)), work_busy( &(i2c_dev->WRITE_WORK->my_work)) );

	/*	--> Returns immediately with 0 if the previous operation has been done successfully, or -3 if failed.	*/
	if(i2c_dev->is_write_work_ready==1 || i2c_dev->is_write_work_ready<0){		// should be -1 if it fails
		ret = i2c_dev->is_write_work_ready==1 ? 0 : -3;
		if(i2c_dev->is_write_work_ready)
			printk("*****WRITE WORK IS READY, RETURNING %d \n", ret);
		else
			printk("******ERROR, RETURNING %d\n", ret);
		return ret;
	}
	else  /* --> Issues a write request to the work queue, and returns with -2, if EEPROM is not busy and no pending work request.*/
		if(!i2c_dev->is_write_work_ready && work_busy( &(i2c_dev->WRITE_WORK->my_work)) == 0 ){
			printk("********ASSIGN WRITE work\n");
			page_size = i2c_dev->EEPROM_PAGE_SIZE;	/*	prepare write work data structs	*/
			i2c_dev->WRITE_WORK->count = count;
			i2c_dev->WRITE_WORK->client =  file->private_data;
			i2c_dev->WRITE_WORK->buffer = kmalloc(count*page_size, GFP_KERNEL);
			res = copy_from_user((void *)(i2c_dev->WRITE_WORK->buffer), (void __user *)buf, count*page_size);
			if(res){
				return -EFAULT;
			}
			ret = queue_work(i2c_dev->MY_WQ, (struct work_struct *)(i2c_dev->WRITE_WORK));		/*	Issue write work request	*/
			printk("^^^^^^RETURNING -2 AFTER ASSIGNING WRITE WORK\n");
			return -2;
		}
		else	/*	-->	Returns immediately with -1 to indicate the EEPROM is busy of the previous operation.	*/
			if(!i2c_dev->is_write_work_ready && (work_busy( &(i2c_dev->WRITE_WORK->my_work)) > 1) ){
				printk("***********BUSY WRITING, so RETURNING -1\n");
				return -1;
			}
			else	/*	should not happen	*/
				printk("WHERE AM I??\n");

	/*	should not happen	*/
	printk("^^^^RETURNING ret=%d\n", ret);
	return ret;
}


static int i2cdev_check(struct device *dev, void *addrp)
{
	struct i2c_client *client = i2c_verify_client(dev);

	if (!client || client->addr != *(unsigned int *)addrp)
		return 0;

	return dev->driver ? -EBUSY : 0;
}

/* walk up mux tree */
static int i2cdev_check_mux_parents(struct i2c_adapter *adapter, int addr)
{
	struct i2c_adapter *parent = i2c_parent_is_i2c_adapter(adapter);
	int result;

	result = device_for_each_child(&adapter->dev, &addr, i2cdev_check);
	if (!result && parent)
		result = i2cdev_check_mux_parents(parent, addr);

	return result;
}

/* recurse down mux tree */
static int i2cdev_check_mux_children(struct device *dev, void *addrp)
{
	int result;

	if (dev->type == &i2c_adapter_type)
		result = device_for_each_child(dev, addrp,
				i2cdev_check_mux_children);
	else
		result = i2cdev_check(dev, addrp);

	return result;
}

/* This address checking function differs from the one in i2c-core
   in that it considers an address with a registered device, but no
   driver bound to it, as NOT busy. */
static int i2cdev_check_addr(struct i2c_adapter *adapter, unsigned int addr)
{
	struct i2c_adapter *parent = i2c_parent_is_i2c_adapter(adapter);
	int result = 0;

	if (parent)
		result = i2cdev_check_mux_parents(parent, addr);

	if (!result)
		result = device_for_each_child(&adapter->dev, &addr,
				i2cdev_check_mux_children);

	return result;
}

static noinline int i2cdev_ioctl_rdrw(struct i2c_client *client,
		unsigned long arg)
{
	struct i2c_rdwr_ioctl_data rdwr_arg;
	struct i2c_msg *rdwr_pa;
	u8 __user **data_ptrs;
	int i, res;

	if (copy_from_user(&rdwr_arg,
			(struct i2c_rdwr_ioctl_data __user *)arg,
			sizeof(rdwr_arg)))
		return -EFAULT;

	/* Put an arbitrary limit on the number of messages that can
	 * be sent at once */
	if (rdwr_arg.nmsgs > I2C_RDRW_IOCTL_MAX_MSGS)
		return -EINVAL;

	rdwr_pa = memdup_user(rdwr_arg.msgs,
			rdwr_arg.nmsgs * sizeof(struct i2c_msg));
	if (IS_ERR(rdwr_pa))
		return PTR_ERR(rdwr_pa);

	data_ptrs = kmalloc(rdwr_arg.nmsgs * sizeof(u8 __user *), GFP_KERNEL);
	if (data_ptrs == NULL) {
		kfree(rdwr_pa);
		return -ENOMEM;
	}

	res = 0;
	for (i = 0; i < rdwr_arg.nmsgs; i++) {
		/* Limit the size of the message to a sane amount */
		if (rdwr_pa[i].len > 8192) {
			res = -EINVAL;
			break;
		}

		data_ptrs[i] = (u8 __user *)rdwr_pa[i].buf;
		rdwr_pa[i].buf = memdup_user(data_ptrs[i], rdwr_pa[i].len);
		if (IS_ERR(rdwr_pa[i].buf)) {
			res = PTR_ERR(rdwr_pa[i].buf);
			break;
		}

		/*
		 * If the message length is received from the slave (similar
		 * to SMBus block read), we must ensure that the buffer will
		 * be large enough to cope with a message length of
		 * I2C_SMBUS_BLOCK_MAX as this is the maximum underlying bus
		 * drivers allow. The first byte in the buffer must be
		 * pre-filled with the number of extra bytes, which must be
		 * at least one to hold the message length, but can be
		 * greater (for example to account for a checksum byte at
		 * the end of the message.)
		 */
		if (rdwr_pa[i].flags & I2C_M_RECV_LEN) {
			if (!(rdwr_pa[i].flags & I2C_M_RD) ||
					rdwr_pa[i].buf[0] < 1 ||
					rdwr_pa[i].len < rdwr_pa[i].buf[0] +
					I2C_SMBUS_BLOCK_MAX) {
				res = -EINVAL;
				break;
			}

			rdwr_pa[i].len = rdwr_pa[i].buf[0];
		}
	}
	if (res < 0) {
		int j;
		for (j = 0; j < i; ++j)
			kfree(rdwr_pa[j].buf);
		kfree(data_ptrs);
		kfree(rdwr_pa);
		return res;
	}

	res = i2c_transfer(client->adapter, rdwr_pa, rdwr_arg.nmsgs);
	while (i-- > 0) {
		if (res >= 0 && (rdwr_pa[i].flags & I2C_M_RD)) {
			if (copy_to_user(data_ptrs[i], rdwr_pa[i].buf,
					rdwr_pa[i].len))
				res = -EFAULT;
		}
		kfree(rdwr_pa[i].buf);
	}
	kfree(data_ptrs);
	kfree(rdwr_pa);
	return res;
}

static noinline int i2cdev_ioctl_smbus(struct i2c_client *client,
		unsigned long arg)
{
	struct i2c_smbus_ioctl_data data_arg;
	union i2c_smbus_data temp;
	int datasize, res;

	if (copy_from_user(&data_arg,
			(struct i2c_smbus_ioctl_data __user *) arg,
			sizeof(struct i2c_smbus_ioctl_data)))
		return -EFAULT;
	if ((data_arg.size != I2C_SMBUS_BYTE) &&
			(data_arg.size != I2C_SMBUS_QUICK) &&
			(data_arg.size != I2C_SMBUS_BYTE_DATA) &&
			(data_arg.size != I2C_SMBUS_WORD_DATA) &&
			(data_arg.size != I2C_SMBUS_PROC_CALL) &&
			(data_arg.size != I2C_SMBUS_BLOCK_DATA) &&
			(data_arg.size != I2C_SMBUS_I2C_BLOCK_BROKEN) &&
			(data_arg.size != I2C_SMBUS_I2C_BLOCK_DATA) &&
			(data_arg.size != I2C_SMBUS_BLOCK_PROC_CALL)) {
		dev_dbg(&client->adapter->dev,
				"size out of range (%x) in ioctl I2C_SMBUS.\n",
				data_arg.size);
		return -EINVAL;
	}
	/* Note that I2C_SMBUS_READ and I2C_SMBUS_WRITE are 0 and 1,
	   so the check is valid if size==I2C_SMBUS_QUICK too. */
	if ((data_arg.read_write != I2C_SMBUS_READ) &&
			(data_arg.read_write != I2C_SMBUS_WRITE)) {
		dev_dbg(&client->adapter->dev,
				"read_write out of range (%x) in ioctl I2C_SMBUS.\n",
				data_arg.read_write);
		return -EINVAL;
	}

	/* Note that command values are always valid! */

	if ((data_arg.size == I2C_SMBUS_QUICK) ||
			((data_arg.size == I2C_SMBUS_BYTE) &&
					(data_arg.read_write == I2C_SMBUS_WRITE)))
		/* These are special: we do not use data */
		return i2c_smbus_xfer(client->adapter, client->addr,
				client->flags, data_arg.read_write,
				data_arg.command, data_arg.size, NULL);

	if (data_arg.data == NULL) {
		dev_dbg(&client->adapter->dev,
				"data is NULL pointer in ioctl I2C_SMBUS.\n");
		return -EINVAL;
	}

	if ((data_arg.size == I2C_SMBUS_BYTE_DATA) ||
			(data_arg.size == I2C_SMBUS_BYTE))
		datasize = sizeof(data_arg.data->byte);
	else if ((data_arg.size == I2C_SMBUS_WORD_DATA) ||
			(data_arg.size == I2C_SMBUS_PROC_CALL))
		datasize = sizeof(data_arg.data->word);
	else /* size == smbus block, i2c block, or block proc. call */
		datasize = sizeof(data_arg.data->block);

	if ((data_arg.size == I2C_SMBUS_PROC_CALL) ||
			(data_arg.size == I2C_SMBUS_BLOCK_PROC_CALL) ||
			(data_arg.size == I2C_SMBUS_I2C_BLOCK_DATA) ||
			(data_arg.read_write == I2C_SMBUS_WRITE)) {
		if (copy_from_user(&temp, data_arg.data, datasize))
			return -EFAULT;
	}
	if (data_arg.size == I2C_SMBUS_I2C_BLOCK_BROKEN) {
		/* Convert old I2C block commands to the new
		   convention. This preserves binary compatibility. */
		data_arg.size = I2C_SMBUS_I2C_BLOCK_DATA;
		if (data_arg.read_write == I2C_SMBUS_READ)
			temp.block[0] = I2C_SMBUS_BLOCK_MAX;
	}
	res = i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			data_arg.read_write, data_arg.command, data_arg.size, &temp);
	if (!res && ((data_arg.size == I2C_SMBUS_PROC_CALL) ||
			(data_arg.size == I2C_SMBUS_BLOCK_PROC_CALL) ||
			(data_arg.read_write == I2C_SMBUS_READ))) {
		if (copy_to_user(data_arg.data, &temp, datasize))
			return -EFAULT;
	}
	return res;
}

static long i2cdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = file->private_data;
	unsigned long funcs;

	dev_dbg(&client->adapter->dev, "ioctl, cmd=0x%02x, arg=0x%02lx\n",
			cmd, arg);

	switch (cmd) {
	case I2C_SLAVE:
	case I2C_SLAVE_FORCE:
		/* NOTE:  devices set up to work with "new style" drivers
		 * can't use I2C_SLAVE, even when the device node is not
		 * bound to a driver.  Only I2C_SLAVE_FORCE will work.
		 *
		 * Setting the PEC flag here won't affect kernel drivers,
		 * which will be using the i2c_client node registered with
		 * the driver model core.  Likewise, when that client has
		 * the PEC flag already set, the i2c-dev driver won't see
		 * (or use) this setting.
		 */
		if ((arg > 0x3ff) ||
				(((client->flags & I2C_M_TEN) == 0) && arg > 0x7f))
			return -EINVAL;
		if (cmd == I2C_SLAVE && i2cdev_check_addr(client->adapter, arg))
			return -EBUSY;
		/* REVISIT: address could become busy later */
		client->addr = arg;
		return 0;
	case I2C_TENBIT:
		if (arg)
			client->flags |= I2C_M_TEN;
		else
			client->flags &= ~I2C_M_TEN;
		return 0;
	case I2C_PEC:
		if (arg)
			client->flags |= I2C_CLIENT_PEC;
		else
			client->flags &= ~I2C_CLIENT_PEC;
		return 0;
	case I2C_FUNCS:
		funcs = i2c_get_functionality(client->adapter);
		return put_user(funcs, (unsigned long __user *)arg);

	case I2C_RDWR:
		return i2cdev_ioctl_rdrw(client, arg);

	case I2C_SMBUS:
		return i2cdev_ioctl_smbus(client, arg);

	case I2C_RETRIES:
		client->adapter->retries = arg;
		break;
	case I2C_TIMEOUT:
		/* For historical reasons, user-space sets the timeout
		 * value in units of 10 ms.
		 */
		client->adapter->timeout = msecs_to_jiffies(arg * 10);
		break;
	default:
		/* NOTE:  returning a fault code here could cause trouble
		 * in buggy userspace code.  Some old kernel bugs returned
		 * zero in this case, and userspace code might accidentally
		 * have depended on that bug.
		 */
		return -ENOTTY;
	}
	return 0;
}

static int i2cdev_open(struct inode *inode, struct file *file)
{
	//unsigned int minor = iminor(inode);
	struct i2c_client *client;
	struct i2c_adapter *adap;
	struct i2c_dev *i2c_dev;

	printk("i2cdev_open\n");
	i2c_dev = i2c_dev_get_by_minor(2);			// returns i2c_dev bound to adapter number 2
	if (!i2c_dev)
		return -ENODEV;
	printk("i2c_dev adap name=%s, adap nr=%d\n",  i2c_dev->adap->name , i2c_dev->adap->nr);
	adap = i2c_get_adapter(i2c_dev->adap->nr);
	if (!adap)
		return -ENODEV;

	/*
	 * This creates an anonymous i2c_client, which may later be
	 * pointed to some address using I2C_SLAVE or I2C_SLAVE_FORCE.
	 *
	 * This client is ** NEVER REGISTERED ** with the driver model
	 * or I2C core code!!  It just holds private copies of addressing
	 * information and maybe a PEC flag.
	 */
	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client) {
		i2c_put_adapter(adap);
		return -ENOMEM;
	}
	snprintf(client->name, I2C_NAME_SIZE, "i2c-flash %d", adap->nr);

	client->adapter = adap;
	printk("setting up client->addr = i2c_eeprom_addr=%d\n", i2c_dev->I2C_EEPROM_ADDR );
	client->addr = i2c_dev->I2C_EEPROM_ADDR;									// set i2c address here only which was done by ioctl earlier
	file->private_data = client;

	return 0;
}

static int i2cdev_release(struct inode *inode, struct file *file)
{
	struct i2c_client *client = file->private_data;

	i2c_put_adapter(client->adapter);
	kfree(client);
	file->private_data = NULL;

	return 0;
}

static const struct file_operations i2cdev_fops = {
		.owner		= THIS_MODULE,
		.llseek		= i2cdev_llseek,
		.read		= i2cdev_read,
		.write		= i2cdev_write,
		.unlocked_ioctl	= i2cdev_ioctl,
		.open		= i2cdev_open,
		.release	= i2cdev_release,
};

/* ------------------------------------------------------------------------- */

static struct class *i2c_dev_class;


static int i2cdev_attach_adapter_my(struct device *dev, void *dummy)
{
	struct i2c_adapter *adap;
	struct i2c_dev *i2c_dev;
	int res;

	if (dev->type != &i2c_adapter_type)
		return 0;
	adap = to_i2c_adapter(dev);
	if(adap->nr ==2){
		i2c_dev = get_free_i2c_dev(adap);
		if (IS_ERR(i2c_dev))
			return PTR_ERR(i2c_dev);

		/* register this i2c device with the driver core */
		i2c_dev->dev = device_create(i2c_dev_class, &adap->dev, MKDEV(I2C_MAJOR, 0),
				NULL, DEVICE_NAME);
		if (IS_ERR(i2c_dev->dev)) {
			res = PTR_ERR(i2c_dev->dev);
			return -1;
		}
		res = device_create_file(i2c_dev->dev, &dev_attr_name);
		if (res)
			goto error_destroy;

		printk("i2c-dev: adapter [%s] with adapter number=[%d] registered with MAJOR=[%d]\n", adap->name,
				adap->nr, I2C_MAJOR);

		i2c_dev->HIGHER_BYTE=0;
		i2c_dev->LOWER_BYTE=0;
		i2c_dev->EEPROM_PAGE_SIZE=64;
		i2c_dev->SLEEP_INTERVAL=10;
		i2c_dev->I2C_EEPROM_ADDR=0x52;
		printk("i2c_dev->eeprom_addr=%d\n", i2c_dev->I2C_EEPROM_ADDR);

		i2c_dev->is_read_work_ready=0;
		i2c_dev->is_write_work_ready=0;


	}
	return 0;



	error_destroy:
	device_destroy(i2c_dev_class, MKDEV(I2C_MAJOR, adap->nr));

	return res;
}

/*	i2cdev_attach_adapter()	*/
static int i2cdev_attach_adapter(struct device *dev, void *dummy)
{
	struct i2c_adapter *adap;
	struct i2c_dev *i2c_dev;
	int res;

	if (dev->type != &i2c_adapter_type)
		return 0;
	adap = to_i2c_adapter(dev);

	i2c_dev = get_free_i2c_dev(adap);
	if (IS_ERR(i2c_dev))
		return PTR_ERR(i2c_dev);

	/* register this i2c device with the driver core */
	i2c_dev->dev = device_create(i2c_dev_class, &adap->dev,
			MKDEV(I2C_MAJOR, adap->nr), NULL,
			"i2c-%d", adap->nr);
	if (IS_ERR(i2c_dev->dev)) {
		res = PTR_ERR(i2c_dev->dev);
		return -1;
	}
	res = device_create_file(i2c_dev->dev, &dev_attr_name);
	if (res)
		goto error_destroy;

	pr_debug("i2c-dev: adapter [%s] registered as minor %d\n",
			adap->name, adap->nr);
	return 0;
	error_destroy:
	device_destroy(i2c_dev_class, MKDEV(I2C_MAJOR, adap->nr));

	return res;
}


/*	  i2cdev_detach_adapter()	*/
static int i2cdev_detach_adapter(struct device *dev, void *dummy)
{
	struct i2c_adapter *adap;
	struct i2c_dev *i2c_dev;

	if (dev->type != &i2c_adapter_type)
		return 0;
	adap = to_i2c_adapter(dev);

	if(adap->nr ==2){
		i2c_dev = i2c_dev_get_by_minor(adap->nr);
		if (!i2c_dev) /* attach_adapter must have failed */
			return 0;

		device_remove_file(i2c_dev->dev, &dev_attr_name);
		return_i2c_dev(i2c_dev);
		device_destroy(i2c_dev_class, MKDEV(I2C_MAJOR, 0));

		printk("i2c-dev: adapter [%s] unregistered\n", adap->name);
	}
	return 0;
}

static int i2cdev_notifier_call(struct notifier_block *nb, unsigned long action,
		void *data)
{
	struct device *dev = data;

	switch (action) {
	case BUS_NOTIFY_ADD_DEVICE:
		return i2cdev_attach_adapter(dev, NULL);
	case BUS_NOTIFY_DEL_DEVICE:
		return i2cdev_detach_adapter(dev, NULL);
	}

	return 0;
}

static struct notifier_block i2cdev_notifier = {
		.notifier_call = i2cdev_notifier_call,
};

/* ------------------------------------------------------------------------- */

/*
 * module load/unload record keeping
 */

static int __init i2c_dev_init(void)
{
	int res;
	struct i2c_dev *i2c_dev;
	printk("i2c_dev_init()\n");
	res = register_chrdev(I2C_MAJOR, DEVICE_NAME, &i2cdev_fops);
	if (res)
		goto out;

	printk("My major number =  res =%d\n", res);

	i2c_dev_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(i2c_dev_class)) {
		res = PTR_ERR(i2c_dev_class);
		goto out_unreg_chrdev;
	}

	/* Keep track of adapters which will be added or removed later */
	res = bus_register_notifier(&i2c_bus_type, &i2cdev_notifier);
	if (res)
		goto out_unreg_class;

	/* Bind to already existing adapters right away */
	i2c_for_each_dev(NULL, i2cdev_attach_adapter_my);
	printk("Adapter with nr=2 attached with i2c_dev\n");

	i2c_dev = i2c_dev_get_by_minor(2);
	printk("i2c_dev->i2c_eeprom_addr = %d page size=%d, adap nr=%d\n", i2c_dev->I2C_EEPROM_ADDR,
			i2c_dev->EEPROM_PAGE_SIZE,i2c_dev->adap->nr);

	/*	set up work queue	*/
	if ( !(i2c_dev->MY_WQ=create_singlethread_workqueue("I2C-FLASH-WQ")) ){
		printk("unable to setup workqueue\n");
		return -1;
	}
	printk("MY_WQ is set\n");

	i2c_dev->READ_WORK = (my_work_t *)kmalloc(sizeof(my_work_t), GFP_KERNEL);
	if (i2c_dev->READ_WORK) {				// Queue work (item 1)
		INIT_WORK( (struct work_struct *)(i2c_dev->READ_WORK), my_wq_read_function );
	}

	i2c_dev->WRITE_WORK = (my_work_t *)kmalloc(sizeof(my_work_t), GFP_KERNEL);
	if (i2c_dev->WRITE_WORK) {				// Queue work (item 1)
		INIT_WORK( (struct work_struct *)(i2c_dev->WRITE_WORK), my_wq_write_function );
	}

	return 0;

	out_unreg_class:
	class_destroy(i2c_dev_class);
	out_unreg_chrdev:
	unregister_chrdev(I2C_MAJOR, "i2c-flash");
	out:
	printk(KERN_ERR "%s: Driver Initialization failed\n", __FILE__);
	return res;
}

static void __exit i2c_dev_exit(void)
{
	struct i2c_dev *i2c_dev;
	i2c_dev = i2c_dev_get_by_minor(2);
	printk("freeing WORK\n");
	kfree(i2c_dev->READ_WORK);
	kfree(i2c_dev->WRITE_WORK);
	bus_unregister_notifier(&i2c_bus_type, &i2cdev_notifier);
	i2c_for_each_dev(NULL, i2cdev_detach_adapter);

	class_destroy(i2c_dev_class);
	unregister_chrdev(I2C_MAJOR, DEVICE_NAME);
}

MODULE_AUTHOR("Rohit Khanna");
MODULE_DESCRIPTION("I2C Client driver for EEPROM");
MODULE_LICENSE("GPL");

module_init(i2c_dev_init);
module_exit(i2c_dev_exit);
