/*
 * @file sensors.cpp
 * @brief DMT g-sensor android driver
 * @author Domintech Technology Co., Ltd (http://www.domintech.com.tw)
 * @version 1.31
 * @date 2012/3/25
 *
 * @section LICENSE
 *
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <cutils/sockets.h>
#include <cutils/properties.h>
#include <hardware/sensors.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <sys/select.h>
#include <linux/input.h>
#include <cutils/atomic.h>
#include <cutils/log.h>
#include <cutils/native_handle.h>
#define LOG_TAG "Sensors"
/*****************************************************************************/

#define SENSORS_ACCELERATION_HANDLE     0
#define SENSORS_MAGNETIC_FIELD_HANDLE   1
#define SENSORS_ORIENTATION_HANDLE      2
#define INPUT_NAME_ACC		"DMT_accel"
#define SensorDevice		"/dev/dmt"
#define CONVERT_A		(GRAVITY_EARTH / 1024.0f)  //1g = 1024codes
/*****************************************************************************/

#if ENABLE_DMT_DMARD03
static const struct sensor_t sSensorList = {
  	"DMARD03 Accelerometer",
	"DMT",
	1,
	SENSORS_ACCELERATION_HANDLE,
	SENSOR_TYPE_ACCELEROMETER,
	(GRAVITY_EARTH * 3.0f),
	CONVERT_A,
	0.25f,
	0, {}
};
#endif
#if ENABLE_DMT_DMARD05
static const struct sensor_t sSensorList = {
  	"DMARD05 Accelerometer",
	"DMT",
	1,
	SENSORS_ACCELERATION_HANDLE,
	SENSOR_TYPE_ACCELEROMETER,
	(GRAVITY_EARTH * 2.0f),
	CONVERT_A,
	0.145f,
	0, {}
};
#endif
#if ENABLE_DMT_DMARD06
static const struct sensor_t sSensorList = {
  	"DMARD06 Accelerometer",
	"DMT",
	1,
	SENSORS_ACCELERATION_HANDLE,
	SENSOR_TYPE_ACCELEROMETER,
	(GRAVITY_EARTH * 2.0f),
	CONVERT_A,
	0.145f,
	0, {}
};
#endif
#if ENABLE_DMT_DMARD07
static const struct sensor_t sSensorList = {
  	"DMARD07 Accelerometer",
	"DMT",
	1,
	SENSORS_ACCELERATION_HANDLE,
	SENSOR_TYPE_ACCELEROMETER,
	(GRAVITY_EARTH * 2.0f),
	CONVERT_A,
	0.145f,
	0, {}
};
#endif
#if ENABLE_DMT_DMARD08
static const struct sensor_t sSensorList = {
  	"DMARD08 Accelerometer",
	"DMT",
	1,
	SENSORS_ACCELERATION_HANDLE,
	SENSOR_TYPE_ACCELEROMETER,
	(GRAVITY_EARTH * 3.0f),
	CONVERT_A,
	0.25f,
	0, {}
};
#endif
#if ENABLE_DMT_DMARD10
static const struct sensor_t sSensorList = {
  	"DMARD10 Accelerometer",
	"DMT",
	1,
	SENSORS_ACCELERATION_HANDLE,
	SENSOR_TYPE_ACCELEROMETER,
	(GRAVITY_EARTH * 2.0f),
	CONVERT_A,
	0.145f,
	0, {}
};
#endif

static unsigned int  gsensor_delay_time = 100000; // 100 ms
int fd=-1;
sensors_event_t mPendingEvents[3];//3 for acc,mag,ori sensors

int write_sys_attribute(const char *path, const char *value, int bytes)
{
    int ifd, amt;

	ifd = open(path, O_WRONLY);
    if (ifd < 0) {
        LOGE("Write_attr failed to open %s (%s)",path, strerror(errno));
        return -1;
	}

    amt = write(ifd, value, bytes);
	amt = ((amt == -1) ? -errno : 0);
	LOGE_IF(amt < 0, "Write_int failed to write %s (%s)",
		path, strerror(errno));
    close(ifd);
	return amt;
}


int open_input_dev(void)
{
 const char *dirname = "/dev/input";  
 char devname[30];  
 char *filename;  
 //int i,version;
 char name[80] ;  
 DIR *dir;  
 struct dirent *de;  
 start:
 dir = opendir(dirname);  
 strcpy(devname,dirname);  
 LOGE("%s:devname=%s\n",__func__,devname);  //"/dev/input"
 filename = devname + strlen(devname);  
 *filename++ = '/';  
 while((de = readdir(dir)))
 {  
  if(   ( (de->d_name[0]=='.') && (de->d_name[1]=='\0') ) ||  \
	( (de->d_name[1]=='.') && (de->d_name[2]=='\0') ) )  
   continue;  
  LOGE("%s:d_name=%s\n",__func__,de->d_name);  
  strcpy(filename,de->d_name);  
    
  fd = open(devname,O_RDONLY);  
  LOGE("%s:devname=%s and filename=%s,fd=%d\n",__func__,devname,filename,fd);
  if(fd > 0)
  {  
  	 if(ioctl(fd, EVIOCGNAME(sizeof(name)),name) > 0)
   	{  
    		LOGE("devname=%s\n",devname);  
    		LOGE("name=%s\n",name);  
    		if(!strcmp(name,INPUT_NAME_ACC))//"DMT_Gsensor"))
    		{		 
     			LOGE("%s:name=%s,fd=%d\n",__func__,name,fd);
     			break;
    		}
		else
    		{
  			LOGE("%s: fd= %d\n",__func__,fd);
     			close(fd);
       			fd=-1;
		}
   	}  
  }//end of if(fd > 0)
 } // end of while((de = readdir(dir))) 
 closedir(dir);
 if(strcmp(name,INPUT_NAME_ACC))
 {
	LOGE("%s:Cannot find DMT_Compass !\n",__func__);
 }

 return 0;
}

static int ProcessEvent(int code,int value)
{
	switch (code)
	{
		case ABS_X:
			mPendingEvents[0].acceleration.x=value*(CONVERT_A);
			break;
		case ABS_Y:
			mPendingEvents[0].acceleration.y=value*(CONVERT_A);
			break;
		case ABS_Z:
			mPendingEvents[0].acceleration.z=value*(CONVERT_A);
			break;
	}
	return 0;
}

//*****************************************************************************************************
static int open_sensors(const struct hw_module_t* module, const char* name, struct hw_device_t** device);

struct sensors_poll_context_t {
    struct sensors_poll_device_t device;
	int fd;
    sensors_event_t sensors[3];
    uint32_t pending_sensors;
};

static struct hw_module_methods_t sensors_module_methods = {
    open: open_sensors
};

static int sensors__get_sensors_list(struct sensors_module_t* module, struct sensor_t const** list)
{
    *list = &sSensorList;
    return 1;
}

struct sensors_module_t HAL_MODULE_INFO_SYM = {
    common: {
       tag: HARDWARE_MODULE_TAG,
			 version_major: 1,
			 version_minor: 0,
			 id: SENSORS_HARDWARE_MODULE_ID,
			 name: "DMT Sensor Module",
			 author: "DMT_RD",
			 methods: &sensors_module_methods,
    },
    get_sensors_list: sensors__get_sensors_list,
};

static int gsensor_open(struct sensors_poll_context_t *dev)
{
	if(dev->fd < 0){
		dev->fd = open(SensorDevice, O_RDONLY);
		if(dev->fd >= 0) {
			printf("Open GSensor Success\n");
		}
		else{
			printf("Open GSensor Error\n");
		}
	}
	return dev->fd;
}

static int poll__close(struct hw_device_t *device)
{
	struct sensors_poll_context_t *dev = (struct sensors_poll_context_t*)device;
	if(dev->fd >= 0){
		close(dev->fd);
	}
	free(dev);
	return 0;
}

static int poll__activate(struct sensors_poll_device_t *device, int handle, int enabled)
{
	char path[]="/sys/class/accelemeter/dmt/enable_acc";
	char value[2]={0,0};
	value[0]= enabled ? '1':'0';
	LOGE("%s:enable=%s\n",__func__,value);
	write_sys_attribute(path,value, 1);
	return 0;	return 0;
}

static int poll__setDelay(struct sensors_poll_device_t *dev, int handle, int64_t ns)
{
	char value[11]={0};
	int bytes;
	char path[]="/sys/class/accelemeter/dmt/delay_acc";
	gsensor_delay_time = ns / 1000000;
	bytes=sprintf(value,"%u",gsensor_delay_time);
	LOGE("value=%s ,gsensor=%u, bytes=%d\n",value,gsensor_delay_time,bytes);
		
	write_sys_attribute(path,value, bytes);
	return 0;
}



//*******************************************************************************************************


static int poll__poll(struct sensors_poll_device_t *device, sensors_event_t* data, int count)
{
	struct sensors_poll_context_t *dev = (struct sensors_poll_context_t *) device;
	
	struct input_event event[5];
	uint32_t new_sensors = 0;
	int64_t time=0 ;
	int nread=0;
	int i,ReturntoPoll;
	if(fd<0)
	{
	 LOGE("%s: input device is not ready !\n",__func__);
	 return 0;
	}
	ReturntoPoll=0;
   	nread=read(fd, &event[0], 5*sizeof(struct input_event));
	nread/=sizeof(struct input_event);
  	if(nread>0)
	{
	 for(i=0;i<nread;i++)
	 {
		switch(event[i].type)
		{
		case EV_ABS:	
			if(event[i].value==65536)
			{
				close(fd);
				event[i].value=0;
			}	
			ProcessEvent(event[i].code,event[i].value);
			break;
		case EV_SYN:
      			time= event[0].time.tv_sec * 1000000000LL + event[0].time.tv_usec * 1000;
               		mPendingEvents[0].timestamp = time;	
			*data=mPendingEvents[0];
			ReturntoPoll=1;
			break;		
		}
	 }
	}
	return ReturntoPoll;// then Android will not analysis the data , even data is legal.
}

static int open_sensors(const struct hw_module_t* module, const char* name, struct hw_device_t** device)
{
	int status = -EINVAL;
	if(!strcmp(name, SENSORS_HARDWARE_POLL)){
		struct sensors_poll_context_t *dev;

		dev = (struct sensors_poll_context_t *) malloc(sizeof(*dev));
		memset(dev, 0, sizeof(*dev));
		dev->fd = -1;
		dev->device.common.tag = HARDWARE_DEVICE_TAG;
		dev->device.common.version = 0;
		dev->device.common.module = const_cast<hw_module_t*> (module);
		dev->device.common.close  = poll__close;
		dev->device.activate      = poll__activate;
		dev->device.setDelay      = poll__setDelay;
		dev->device.poll          = poll__poll;
		*device = &dev->device.common;
 		open_input_dev();
		memset(mPendingEvents, 0, sizeof(mPendingEvents));
    		mPendingEvents[0].version = sizeof(sensors_event_t);
    		mPendingEvents[0].sensor = 0;
    		mPendingEvents[0].type = SENSOR_TYPE_ACCELEROMETER;
    		mPendingEvents[0].acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;
		status = 0;
	}

	return status;
}
