/*
 * @file drivers/misc/dmt050607.c
 * @brief DMT g-sensor Linux device driver
 * @author Domintech Technology Co., Ltd (http://www.domintech.com.tw)
 * @version 1.34
 * @date 2012/9/24
 *
 * @section LICENSE
 *
 *  Copyright 2012 Domintech Technology Co., Ltd
 *
 * 	This software is licensed under the terms of the GNU General Public
 * 	License version 2, as published by the Free Software Foundation, and
 * 	may be copied, distributed, and modified under those terms.
 *
 * 	This program is distributed in the hope that it will be useful,
 * 	but WITHOUT ANY WARRANTY; without even the implied warranty of
 * 	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * 	GNU General Public License for more details.
 *
 *  V1.32	ADD AUTO_CALIBRATION FUNCTION
 *  V1.33	1. ADD device_i2c_txdata FUNCTION
 *			2. SET BANDWIDTH_REG to No filter
 *  V1.33a	1. Default Open AUTO_CALIBRATION FUNCTION
 *			2. Check the file "offset.txt". If not exist,immediate implementation of the automatic calibration.
 *  V1.34	1. remove IOCTL(_IOC_NR(cmd) > SENSOR_MAXNR)
 *			2. Unified 1g = 1024 LSB
 *			3. Fix sensorlayout's define
 */
#include <linux/dmt050607.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/earlysuspend.h>
#include <linux/wakelock.h>
#include <asm/uaccess.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#define DMT_DEBUG_DATA
static unsigned int interval;
void gsensor_write_offset_to_file(void);
void gsensor_read_offset_from_file(void);
char OffsetFileName[] = "/data/misc/dmt/offset.txt";	/* FILE offset.txt */
static char const *const ACCELEMETER_CLASS_NAME = "accelemeter";
static char const *const GSENSOR_DEVICE_NAME = "dmt";
static raw_data offset;
static struct dev_data devdata;

static int device_init(void);
static void device_exit(void);

static int device_open(struct inode*, struct file*);
//static ssize_t device_write(struct file*, const char*, size_t, loff_t*);
//static ssize_t device_read(struct file*, char*, size_t, loff_t*);
static long device_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int device_close(struct inode*, struct file*);

static int device_i2c_suspend(struct i2c_client *client, pm_message_t mesg);
static int device_i2c_resume(struct i2c_client *client);
static int __devinit device_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __devexit device_i2c_remove(struct i2c_client *client);
void device_i2c_read_xyz(struct i2c_client *client, s16 *xyz);
static int device_i2c_rxdata(struct i2c_client *client, unsigned char *rxData, int length);
static int device_i2c_txdata(struct i2c_client *client, unsigned char *txData, int length);

static int DMT_GetOpenStatus(void){	
	dmtprintk(KERN_INFO "start active=%d\n",devdata.active.counter);
	wait_event_interruptible(devdata.open_wq, (atomic_read(&devdata.active) != 0));
	return 0;
}

static int DMT_GetCloseStatus(void){
	dmtprintk(KERN_INFO "start active=%d\n",devdata.active.counter);
	wait_event_interruptible(devdata.open_wq, (atomic_read(&devdata.active) <= 0));
	return 0;
}

static void DMT_sysfs_update_active_status(int en){
	unsigned long dmt_delay;
	if(en){
		dmt_delay=msecs_to_jiffies(atomic_read(&devdata.delay));
		if(dmt_delay<1)
			dmt_delay=1;

		dmtprintk(KERN_INFO "schedule_delayed_work start with delay time=%lu\n",dmt_delay);
		schedule_delayed_work(&devdata.delaywork,dmt_delay);
	}
	else 
		cancel_delayed_work_sync(&devdata.delaywork);
}

static bool get_value_as_int(char const *buf, size_t size, int *value){
	long tmp;
	if (size == 0)
		return false;
	/* maybe text format value */
	if ((buf[0] == '0') && (size > 1)) {
		if ((buf[1] == 'x') || (buf[1] == 'X')) {
			/* hexadecimal format */
			if (0 != strict_strtol(buf, 16, &tmp))
				return false;
		} else {
			/* octal format */
			if (0 != strict_strtol(buf, 8, &tmp))
				return false;
		}
	} else {
		/* decimal format */
		if (0 != strict_strtol(buf, 10, &tmp))
			return false;
	}
	if (tmp > INT_MAX)
		return false;

	*value = tmp;
	return true;
}

static bool get_value_as_int64(char const *buf, size_t size, long long *value){
	long long tmp;
	if (size == 0)
		return false;

	/* maybe text format value */
	if ((buf[0] == '0') && (size > 1)) {
		if ((buf[1] == 'x') || (buf[1] == 'X')) {
			/* hexadecimal format */
			if (0 != strict_strtoll(buf, 16, &tmp))
				return false;
		} else {
			/* octal format */
			if (0 != strict_strtoll(buf, 8, &tmp))
				return false;
		}
	} else {
		/* decimal format */
		if (0 != strict_strtoll(buf, 10, &tmp))
			return false;
	}

	if (tmp > LLONG_MAX)
		return false;

	*value = tmp;
	return true;
}

static ssize_t DMT_enable_acc_show(struct device *dev, struct device_attribute *attr, char *buf){
	char str[2][16]={"ACC enable OFF","ACC enable ON"};
	int flag;
	flag=atomic_read(&devdata.enable); 
	
	return sprintf(buf, "%s\n", str[flag]);
}

static ssize_t DMT_enable_acc_store( struct device *dev, struct device_attribute *attr, char const *buf, size_t count){
	int en;
	dmtprintk(KERN_INFO "%s:buf=%x %x\n",__func__,buf[0],buf[1]);
	if (false == get_value_as_int(buf, count, &en))
		return -EINVAL;

	en = en ? 1 : 0;
	atomic_set(&devdata.enable,en);
	DMT_sysfs_update_active_status(en);
	return 1;
}

static ssize_t DMT_delay_acc_show( struct device *dev, struct device_attribute *attr, char *buf){
	return sprintf(buf, "%d\n", atomic_read(&devdata.delay));
}

static ssize_t DMT_delay_acc_store( struct device *dev, struct device_attribute *attr,char const *buf, size_t count){
	long long data;
	if (false == get_value_as_int64(buf, count, &data))
		return -EINVAL;
	atomic_set(&devdata.delay, (unsigned int) data);
	dmtprintk(KERN_INFO "Driver attribute set delay =%ld\n",data);
	return count;
}

static struct device_attribute DMT_attributes[] = {
	__ATTR(enable_acc, 0660, DMT_enable_acc_show, DMT_enable_acc_store),
	__ATTR(delay_acc,  0660, DMT_delay_acc_show,  DMT_delay_acc_store),
	__ATTR_NULL,
};

static int create_device_attributes(struct device *dev,	struct device_attribute *attrs){
	int i;
	int err = 0;
	for (i = 0 ; NULL != attrs[i].attr.name ; ++i) {
		err = device_create_file(dev, &attrs[i]);
		if (0 != err)
			break;
	}

	if (0 != err) {
		for (; i >= 0 ; --i)
			device_remove_file(dev, &attrs[i]);
	}
	return err;
}

int input_init(void)
{
	int err=0;
	devdata.input=input_allocate_device();
	if (!devdata.input)
		return -ENOMEM;
	else
	   dmtprintk(KERN_INFO "input device allocate Success !!\n");
	/* Setup input device */
	set_bit(EV_ABS, devdata.input->evbit);
	/* Accelerometer [-78.5, 78.5]m/s2 in Q16*/
	input_set_abs_params(devdata.input, ABS_X, -5144576, 5144576, 0, 0);
	input_set_abs_params(devdata.input, ABS_Y, -5144576, 5144576, 0, 0);
	input_set_abs_params(devdata.input, ABS_Z, -5144576, 5144576, 0, 0);

	/* Set InputDevice Name */
	devdata.input->name = INPUT_NAME_ACC;

	/* Register */
	err = input_register_device(devdata.input);
	if (err) {
		input_free_device(devdata.input);
		return err;
	}
	atomic_set(&devdata.active, 0);
	dmtprintk(KERN_INFO "in driver ,active=%d\n",devdata.active.counter);

	init_waitqueue_head(&devdata.open_wq);
	return err;
}

int gsensor_calibrate(void)
{	
	raw_data avg;
	int i, j ,ret=0;
	long xyz_acc[SENSOR_DATA_SIZE];   
  	s16 xyz[SENSOR_DATA_SIZE];
	/* initialize the accumulation buffer */
  	for(i = 0; i < SENSOR_DATA_SIZE; ++i) 
		xyz_acc[i] = 0;

	for(i = 0; i < AVG_NUM; i++) {      
		device_i2c_read_xyz(devdata.client, (s16 *)&xyz);
		for(j = 0; j < SENSOR_DATA_SIZE; ++j) 
			xyz_acc[j] += xyz[j];
  	}
	/* calculate averages */
  	for(i = 0; i < SENSOR_DATA_SIZE; ++i) 
		avg.v[i] = (s16) (xyz_acc[i] / AVG_NUM);
		
	if(avg.v[2] < 0){
		offset.u.x =  avg.v[0] ;    
		offset.u.y =  avg.v[1] ;
		offset.u.z =  avg.v[2] + DEFAULT_SENSITIVITY;
		return CONFIG_GSEN_CALIBRATION_GRAVITY_ON_Z_POSITIVE;
	}
	else{	
		offset.u.x =  avg.v[0] ;    
		offset.u.y =  avg.v[1] ;
		offset.u.z =  avg.v[2] - DEFAULT_SENSITIVITY;
		return CONFIG_GSEN_CALIBRATION_GRAVITY_ON_Z_NEGATIVE; 
	}
	return 0;
}

int gsensor_reset(struct i2c_client *client){
	unsigned char buffer[2];
	buffer[0] = REG_SW_RESET;
	device_i2c_rxdata(client, buffer, 1);
	dmtprintk(KERN_INFO "i2c Read SW_RESET = %x \n", buffer[0]);
	buffer[0] = REG_WHO_AM_I;
	device_i2c_rxdata(client, buffer, 1);
	dmtprintk(KERN_INFO "i2c Read WHO_AM_I = %d \n", buffer[0]);
	if( ( buffer[0]&0x00FF) == WHO_AM_I_VALUE){
		dmtprintk(KERN_INFO "@@@ %s DMT_DEVICE_NAME registered I2C driver!\n",__FUNCTION__);
		devdata.client = client;
	}
	else{
		dmtprintk(KERN_INFO "@@@ %s gsensor I2C err = %d!\n",__FUNCTION__,buffer[1]);
		devdata.client = NULL;
		return -1;
	}
	
	//buffer[0] = REG_CONTROL_2;
	//device_i2c_rxdata(client, buffer, 1);
	dmtprintk(KERN_INFO " Default Control Register 2 = %d \n", buffer[0]);
	/* set Low-pass Filter to No filter */
	buffer[0] = REG_CONTROL_2;
	buffer[1] = CONTROL_REGISTER_2_VALUE;
	device_i2c_txdata(client, buffer, 2);	

	return 0;
}

void gsensor_set_offset(int val[3]){
	int i;
	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		offset.v[i] = (s16) val[i];
}

struct file_operations dmt_g_sensor_fops = 
{
	.owner = THIS_MODULE,
	//.read = device_read,
	//.write = device_write,
	.unlocked_ioctl = device_ioctl,
	.open = device_open,
	.release = device_close,
};

static int sensor_close_dev(struct i2c_client *client){    	
	char buffer[2];
	buffer[0] = REG_CONTROL_1;
	buffer[1] = POWERDOWN_MODE;
	dmtprintk("%s\n",__FUNCTION__);	
	return device_i2c_txdata(client,buffer, 2);
}

static int device_i2c_suspend(struct i2c_client *client, pm_message_t mesg){
	dmtprintk("Gsensor enter 2 level  suspend\n");
	return sensor_close_dev(client);
}

static int device_i2c_resume(struct i2c_client *client){
	dmtprintk("Gsensor 2 level resume!!\n");
	return gsensor_reset(client);
}

static int __devexit device_i2c_remove(struct i2c_client *client){
	return 0;
}

static const struct i2c_device_id device_i2c_ids[] = {
	{DEVICE_I2C_NAME, 0},
	{}   
};

MODULE_DEVICE_TABLE(i2c, device_i2c_ids);

static struct i2c_driver device_i2c_driver = {
	.driver	= {
		.owner = THIS_MODULE,
		.name = DEVICE_I2C_NAME,
		},
	.class = I2C_CLASS_HWMON,
	.id_table = device_i2c_ids,
	.probe = device_i2c_probe,
	.remove	= __devexit_p(device_i2c_remove),
#ifndef CONFIG_ANDROID_POWER
	.suspend = device_i2c_suspend,
	.resume	= device_i2c_resume,
#endif	
};

static int device_open(struct inode *inode, struct file *filp){
	return 0; 
}
/*
static ssize_t device_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	return 0;
}

static ssize_t device_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	s16 xyz[SENSOR_DATA_SIZE];
	int i;

	device_i2c_read_xyz(devdata.client, (s16 *)&xyz);
	//offset compensation 
	for(i = 0; i < SENSOR_DATA_SIZE; i++)
		xyz[i] -= offset.v[i];
	
	if(copy_to_user(buf, &xyz, count)) 
		return -EFAULT;
		
	return count;
}
*/
static long device_ioctl(struct file *filp, unsigned int cmd, unsigned long arg){
	int err = 0, ret = 0, i;
	int intBuf[SENSOR_DATA_SIZE];
	s16 xyz[SENSOR_DATA_SIZE];
	//check type and number
	if (_IOC_TYPE(cmd) != IOCTL_MAGIC) return -ENOTTY;

	//check user space pointer is valid
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;
	
	switch(cmd) 
	{
		case SENSOR_RESET:
			gsensor_reset(devdata.client);
			return ret;

		case SENSOR_CALIBRATION:
			// get orientation info
			//if(copy_from_user(&intBuf, (int*)arg, sizeof(intBuf))) return -EFAULT;
			gsensor_calibrate();
			dmtprintk("Sensor_calibration:%d %d %d\n",offset.u.x,offset.u.y,offset.u.z);
			// save file
			gsensor_write_offset_to_file();
			
			// return the offset
			for(i = 0; i < SENSOR_DATA_SIZE; ++i)
				intBuf[i] = offset.v[i];

			ret = copy_to_user((int *)arg, &intBuf, sizeof(intBuf));
			return ret;
		
		case SENSOR_GET_OFFSET:
			// get data from file 
			gsensor_read_offset_from_file();
			for(i = 0; i < SENSOR_DATA_SIZE; ++i)
				intBuf[i] = offset.v[i];

			ret = copy_to_user((int *)arg, &intBuf, sizeof(intBuf));
			return ret;

		case SENSOR_SET_OFFSET:
			ret = copy_from_user(&intBuf, (int *)arg, sizeof(intBuf));
			gsensor_set_offset(intBuf);
			// write in to file
			gsensor_write_offset_to_file();
			return ret;
		
		case SENSOR_READ_ACCEL_XYZ:
			device_i2c_read_xyz(devdata.client, (s16 *)&xyz);
			for(i = 0; i < SENSOR_DATA_SIZE; ++i)
				intBuf[i] = xyz[i] - offset.v[i];
			
		  	ret = copy_to_user((int*)arg, &intBuf, sizeof(intBuf));
			//PRINT_X_Y_Z(intBuf[0], intBuf[1], intBuf[2]);
			return ret;
		case SENSOR_SETYPR:
			if(copy_from_user(&intBuf, (int*)arg, sizeof(intBuf))) {
				dmtprintk("%s:copy_from_user(&intBuf, (int*)arg, sizeof(intBuf)) ERROR, -EFAULT\n",__func__);			
				return -EFAULT;
			}
			input_report_abs(devdata.input, ABS_X, intBuf[0]);
			input_report_abs(devdata.input, ABS_Y, -intBuf[1]);
			input_report_abs(devdata.input, ABS_Z, -intBuf[2]);
			input_sync(devdata.input);
			dmtprintk(KERN_INFO "%s:SENSOR_SETYPR OK! x=%d,y=%d,z=%d\n",__func__,intBuf[0],intBuf[1],intBuf[2]);
			return 1;
		case SENSOR_GET_OPEN_STATUS:
			dmtprintk(KERN_INFO "%s:Going into DMT_GetOpenStatus()\n",__func__);
			DMT_GetOpenStatus();
			dmtprintk(KERN_INFO "%s:DMT_GetOpenStatus() finished\n",__func__);
			return 1;
			break;
		case SENSOR_GET_CLOSE_STATUS:
			dmtprintk(KERN_INFO "%s:Going into DMT_GetCloseStatus()\n",__func__);
			DMT_GetCloseStatus();	
			dmtprintk(KERN_INFO "%s:DMT_GetCloseStatus() finished\n",__func__);
			return 1;
			break;		
		case SENSOR_GET_DELAY:
		  	ret = copy_to_user((int*)arg, &interval, sizeof(interval));
			return 1;
			break;
		
		default:  /* redundant, as cmd was checked against MAXNR */
			return -ENOTTY;
	}	
	return ret;
}
	
static int device_close(struct inode *inode, struct file *filp){
	return 0;
}

/***** I2C I/O function ***********************************************/
static int device_i2c_rxdata( struct i2c_client *client, unsigned char *rxData, int length){
	struct i2c_msg msgs[] = 
	{
		{.addr = client->addr, .flags = 0, .len = 1, .buf = rxData,}, 
		{.addr = client->addr, .flags = I2C_M_RD, .len = length, .buf = rxData,},
	};
	unsigned char addr = rxData[0];
	if (i2c_transfer(client->adapter, msgs, 2) < 0) {
		dev_err(&client->dev, "%s: transfer failed.", __func__);
		return -EIO;
	}
	//DMT_DATA(&client->dev, "RxData: len=%02x, addr=%02x, data=%02x\n",
		//length, addr, rxData[0]);

	return 0;
}

static int device_i2c_txdata( struct i2c_client *client, unsigned char *txData, int length){
	struct i2c_msg msg[] = {
		{.addr = client->addr, .flags = 0, .len = length, .buf = txData,}, 
	};

	if (i2c_transfer(client->adapter, msg, 1) < 0) {
		dev_err(&client->dev, "%s: transfer failed.", __func__);
		return -EIO;
	}
	//DMT_DATA(&client->dev, "TxData: len=%02x, addr=%02x data=%02x\n",
		//length, txData[0], txData[1]);
	return 0;
}

static inline void device_i2c_correct_accel_sign(s16 *val){  
#if (defined(CONFIG_SENSORS_DMARD06) || defined(CONFIG_SENSORS_DMARD06_MODULE))
	*val>>= 9; // shift to 7bit value
	*val<<= 5; // shift to 12bit value
#else
	*val>>= 4; // shift to 12bit value
#endif  
}

void device_i2c_merge_register_values(struct i2c_client *client, s16 *val, u8 msb){
	*val = (((u16)msb) << 8);	
	device_i2c_correct_accel_sign(val);
}

void device_i2c_read_xyz(struct i2c_client *client, s16 *xyz_p){
	s16 xyzTmp[SENSOR_DATA_SIZE];
	int i, j;
	u8 buffer[3];
	buffer[0] = REG_X_OUT;
	device_i2c_rxdata(client, buffer, 3); 
	
	//merge value
	for(i = 0; i < SENSOR_DATA_SIZE; ++i){
		xyz_p[i] = 0;
		device_i2c_merge_register_values(client, (xyzTmp + i), buffer[i]);
	//transfer to the default layout
		for(j = 0; j < 3; j++)
			xyz_p[i] += sensorlayout[i][j] * xyzTmp[j];
	}
	dmtprintk(KERN_INFO "@DMT@ xyzTmp: %04d , %04d , %04d\n", xyzTmp[0], xyzTmp[1], xyzTmp[2]);
	dmtprintk(KERN_INFO "@DMT@ xyz_p: %04d , %04d , %04d\n", xyz_p[0], xyz_p[1], xyz_p[2]);

}

static void DMT_work_func(struct work_struct *fakework){
	int i;
	static int firsttime=0;
	s16 xyz[SENSOR_DATA_SIZE];
	unsigned long t=atomic_read(&devdata.delay);
  	unsigned long dmt_delay = msecs_to_jiffies(t);
	if(!firsttime){
		gsensor_read_offset_from_file();	
	 	firsttime=1;
	}
	dmtprintk(KERN_INFO "t=%lu , dmt_delay=%lu\n", t, dmt_delay);
  	device_i2c_read_xyz(devdata.client, (s16 *)&xyz);
  	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
     		xyz[i] -= offset.v[i];

	PRINT_X_Y_Z(xyz[0], xyz[1], xyz[2]);
	PRINT_X_Y_Z(offset.u.x, offset.u.y, offset.u.z);
	input_report_abs(devdata.input, ABS_X, xyz[0]);
	input_report_abs(devdata.input, ABS_Y, -xyz[1]);
	input_report_abs(devdata.input, ABS_Z, -xyz[2]);
	input_sync(devdata.input);
		
	if(dmt_delay<1)
		dmt_delay=1;
	schedule_delayed_work(&devdata.delaywork, dmt_delay);
}

static int __devinit device_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id){
	int i;
	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		offset.v[i] = 0;
	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)){
  		dmtprintk(KERN_INFO "%s, functionality check failed\n", __func__);
		return -1;
  	}
	gsensor_reset(client);
	return 0;
}

static int __init device_init(void){
	int err=-1;	
	struct device *device;
	int ret = 0;
	ret = alloc_chrdev_region(&devdata.devno, 0, 1, GSENSOR_DEVICE_NAME);
  	if(ret){
		dmtprintk(KERN_INFO "%s, can't allocate chrdev\n", __func__);
		return ret;
	}
	dmtprintk(KERN_INFO "%s, register chrdev(%d, %d)\n", __func__, MAJOR(devdata.devno), MINOR(devdata.devno));
	
	cdev_init(&devdata.cdev, &dmt_g_sensor_fops);  
	devdata.cdev.owner = THIS_MODULE;
  	ret = cdev_add(&devdata.cdev, devdata.devno, 1);
  	if(ret < 0){
		dmtprintk(KERN_INFO "%s, add character device error, ret %d\n", __func__, ret);
		return ret;
	}
	devdata.class = class_create(THIS_MODULE, ACCELEMETER_CLASS_NAME);
	if(IS_ERR(devdata.class)){
   		dmtprintk(KERN_INFO "%s, create class, error\n", __func__);
		return ret;
  	}
	device=device_create(devdata.class, NULL, devdata.devno, NULL, GSENSOR_DEVICE_NAME);
	mutex_init(&devdata.DMT_mutex);
	INIT_DELAYED_WORK(&devdata.delaywork, DMT_work_func);
	dmtprintk(KERN_INFO "DMT: INIT_DELAYED_WORK\n");

	err=input_init();
	if(err)
		dmtprintk(KERN_INFO "%s:input_init fail, error code= %d\n",__func__,err);

	err = create_device_attributes(device,DMT_attributes);
	return i2c_add_driver(&device_i2c_driver);
}

static void __exit device_exit(void){
	input_unregister_device(devdata.input);
	input_free_device(devdata.input);
	cdev_del(&devdata.cdev);
	unregister_chrdev_region(devdata.devno, 1);
	device_destroy(devdata.class, devdata.devno);
	class_destroy(devdata.class);
	i2c_del_driver(&device_i2c_driver);
}

void gsensor_write_offset_to_file(void){
	char data[18];
	unsigned int orgfs;
	struct file *fp;

	sprintf(data,"%5d %5d %5d",offset.u.x,offset.u.y,offset.u.z);

	orgfs = get_fs();
	/* Set segment descriptor associated to kernel space */
	set_fs(KERNEL_DS);

	fp = filp_open(OffsetFileName, O_RDWR | O_CREAT, 0777);
	if(IS_ERR(fp)){
		dmtprintk(KERN_INFO "filp_open %s error!!.\n",OffsetFileName);
	}
	else{
		dmtprintk(KERN_INFO "filp_open %s SUCCESS!!.\n",OffsetFileName);
		fp->f_op->write(fp,data,18, &fp->f_pos);
 		filp_close(fp,NULL);
	}
	set_fs(orgfs);
}

void gsensor_read_offset_from_file(void){
	unsigned int orgfs;
	char data[18];
	struct file *fp;
	int ux,uy,uz;
	orgfs = get_fs();
	/* Set segment descriptor associated to kernel space */
	set_fs(KERNEL_DS);

	fp = filp_open(OffsetFileName, O_RDWR , 0);
	dmtprintk(KERN_INFO "%s,try file open !\n",__func__);
	if(IS_ERR(fp)){
		dmtprintk(KERN_INFO "%s:Sorry,file open ERROR !\n",__func__);
		offset.u.x=0;offset.u.y=0;offset.u.z=0;
#if AUTO_CALIBRATION
		/* get acceleration average reading */
		gsensor_calibrate();
		gsensor_write_offset_to_file();
#endif
	}
	else{
		dmtprintk("filp_open %s SUCCESS!!.\n",OffsetFileName);
		fp->f_op->read(fp,data,18, &fp->f_pos);
		dmtprintk("filp_read result %s\n",data);
		sscanf(data,"%d %d %d",&ux,&uy,&uz);
		offset.u.x=ux;
		offset.u.y=uy;
		offset.u.z=uz;
		filp_close(fp,NULL);
	}
	set_fs(orgfs);
}
//*********************************************************************************************************
MODULE_AUTHOR("DMT_RD");
MODULE_DESCRIPTION("DMT Gsensor Driver");
MODULE_LICENSE("GPL");

module_init(device_init);
module_exit(device_exit);
