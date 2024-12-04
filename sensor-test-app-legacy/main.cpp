/* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include <dlfcn.h>
#include <errno.h>
#include <inttypes.h>
#include <linux/input.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/signalfd.h>
#include <sys/eventfd.h>
#include <sys/wait.h>
#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include <utils/Log.h>
#include <ctype.h>
#include <stdarg.h>
#include "SensorTestApp.h"

#define CMD_OPTIONS         "s:l:b:d:a:n:t:g:"
/*
 * Global variables
 */
struct sensors_poll_device_t *poll_dev_v0 = NULL;
struct sensors_poll_device_1 *poll_dev    = NULL;
static struct sensors_module_t *hmi       = NULL;
struct hw_device_t *dev                   = NULL;

int64_t batch_time = DEFAULT_BATCH;
int64_t odr	= DEFAULT_ODR;
static int sensor_type = SENSOR_ASM330;
bool mBufferSupported = false;
bool mTempSupported = false;


static void printHelp() {
    SENSOR_LOGI(LOG_TAG "\n************* options *************\n");
    SENSOR_LOGI(LOG_TAG "h: help\n");
    SENSOR_LOGI(LOG_TAG "l: lib path\n");
    SENSOR_LOGI(LOG_TAG "g: Get Sensor list\n");
    SENSOR_LOGI(LOG_TAG "d: Sampling rate in ns\n");
    SENSOR_LOGI(LOG_TAG "b: Batching rate in ns\n");
    SENSOR_LOGI(LOG_TAG "a: Read Live data\n");
    SENSOR_LOGI(LOG_TAG "n: Read Sensor Bufffer Data\n");
    SENSOR_LOGI(LOG_TAG "t: Read Sensor Temperature\n");
    SENSOR_LOGI(LOG_TAG "s: Run on boot\n");
    SENSOR_LOGI(LOG_TAG "q: Quit\n");
    SENSOR_LOGI(LOG_TAG "Ex: sensor_test -l /usr/lib/libsensors.so -d 10000000 -d 100000000 -n 1 -a 1 -t 1 -g 1\n");
}

void term(int signum) {
   SENSOR_LOGI(LOG_TAG "  Caught!signum %d\n", signum);
   int ret = 0;
   if (SIGTERM == signum) {
	   poll_dev_v0->common.close(&poll_dev_v0->common);
	   pthread_exit((void *) &ret);
   }
   return;
}

/*
 * Functions : get the current timestamp value
 */
static uint64_t getTimestamp() {
   struct timespec ts;
   clock_gettime(CLOCK_BOOTTIME, &ts);
   uint64_t system_ts =
	   ((uint64_t)(ts.tv_sec)) * 1000000000ULL + ((uint64_t)(ts.tv_nsec));
   return system_ts;
}

static void dump_event(struct sensors_event_t *e)
{
	static int64_t acc_ts = 0;
	static int64_t gyro_ts = 0;
	static int account = 0;
	uint64_t ts_cur = getTimestamp();
	switch (e->type)
	{
		case SENSOR_TYPE_ACCELEROMETER:
			SENSOR_LOGI(LOG_TAG "Live: ACC: count=%d, x=%+f y=%+f z=%+f, event_ts_ns=%lld, latency in ms=%lld, sampling rate in ms=%lld\n",
				account, e->acceleration.x, e->acceleration.y, e->acceleration.z,
				e->timestamp, (ts_cur - e->timestamp)/1000000, (e->timestamp - acc_ts)/1000000);
			acc_ts = e->timestamp;
			break;

		case SENSOR_TYPE_MAGNETIC_FIELD:
			SENSOR_LOGI(LOG_TAG "Live: MAG: x=%f y=%f z=%f, ts=%lld ns, event ts=%lld ns\n",
				e->magnetic.x, e->magnetic.y, e->magnetic.z, ts_cur, e->timestamp);
			break;

		case SENSOR_TYPE_GYROSCOPE:
			SENSOR_LOGI(LOG_TAG "Live: GYRO: count=%d, x=%f y=%f z=%f, event_ts_ns=%lld, latency in ms=%lld, sampling rate in ms=%lld\n",
				account, e->gyro.x, e->gyro.y, e->gyro.z,
				e->timestamp, (ts_cur - e->timestamp)/1000000, (e->timestamp - gyro_ts)/1000000);
			gyro_ts = e->timestamp;
			break;

		case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
			SENSOR_LOGI(LOG_TAG "Live: GYRO_UN: count=%d, x=%f y=%f z=%f, event_ts_ns=%lld, latency in ms=%lld, sampling rate in ms=%lld\n",
				account, e->uncalibrated_gyro.x_uncalib, e->uncalibrated_gyro.y_uncalib, e->uncalibrated_gyro.z_uncalib,
				e->timestamp, (ts_cur - e->timestamp)/1000000, (e->timestamp - gyro_ts)/1000000);
			gyro_ts = e->timestamp;
			break;

		default:
			SENSOR_LOGE(LOG_TAG "Live: Unhandled events %d", e->type);
			break;
		}
		account++;
	return;
}

static int sensor_set_delay(int handle, int64_t delay)
{
	return poll_dev->setDelay(poll_dev_v0, handle, delay);
}

static int sensor_set_batch(int handle, int64_t delay, int64_t latency)
{
	return poll_dev->batch(poll_dev, handle, 0, delay, latency);
}

static int sensor_activate(int handle, int enable)
{
	return poll_dev->activate(poll_dev_v0, handle, enable);
}

static int sensor_flush(int handle)
{
	return poll_dev->flush(poll_dev, handle);
}

int get_sensor_list_all(struct sensor_t const** s) {
   int sensor_num = 0;
   struct sensor_t const* list;
   sensor_num = hmi->get_sensors_list(hmi, &list);
   *s = (struct sensor_t const *)list;
   for (int i = 0; i < sensor_num ; i++)
   {
           SENSOR_LOGI(LOG_TAG "%s\n"
                           "\tvendor: %s\n"
                           "\tversion: %d\n"
                           "\thandle: %d\n"
                           "\ttype: %d\n"
                           "\tmaxRange: %f\n"
                           "\tresolution: %f\n"
                           "\tpower: %f mA\n",
                           list[i].name,
                           list[i].vendor,
                           list[i].version,
                           list[i].handle,
                           list[i].type,
                           list[i].maxRange,
                           list[i].resolution,
                           list[i].power);
   }
   return sensor_num;
}

static void configure_activate_sensors() {
   int i = 0;
   int sensor_num = 0;
   const struct sensor_t *s = NULL;
   sensor_num = get_sensor_list_all(&s);
   for(i = 0; i < sensor_num; i++)
   {
	   sensor_activate(s[i].handle, SENSOR_DISABLE);
   }
   for(i = 0; i < sensor_num; i++)
   {
	   SENSOR_LOGI(LOG_TAG "Handle %d\n", s[i].handle);
		sensor_activate(s[i].handle, SENSOR_DISABLE);
		sensor_set_batch(s[i].handle, odr, batch_time);
		//sensor_set_delay(s[i].handle, odr);
		sensor_flush(s[i].handle);
		sensor_activate(s[i].handle, SENSOR_ENABLE);
   }
   return;
}

static void activate_sensors() {
   int sensor_num = 0;
   const struct sensor_t *s;
   sensor_num = get_sensor_list_all(&s);
   for(int i = 0; i < sensor_num; i++)
   {
	   SENSOR_LOGI(LOG_TAG "  Activate Handle %d\n", s[i].handle);
	   sensor_activate(s[i].handle, SENSOR_ENABLE);
   }
   return;
}

static void deactivate_sensors() {
   int sensor_num = 0;
   const struct sensor_t *s;
   sensor_num = get_sensor_list_all(&s);
   for(int i = 0; i < sensor_num; i++)
   {
	   SENSOR_LOGI(LOG_TAG "  Deactivate Handle %d\n", s[i].handle);
	   sensor_activate(s[i].handle, SENSOR_DISABLE);
   }
   return;
}

static int sensors_initialize(const char *lib_path) {
   int err = 0;
   void *handle = NULL;

   if(!lib_path)
   {
	   SENSOR_LOGE(LOG_TAG "Library Path not provided\n");
	   return -1;
   }
   handle = dlopen(lib_path, RTLD_NOW);
   if (!handle)
   {
	   char *error = dlerror();
	   if(error) SENSOR_LOGE(LOG_TAG "Unable to load senor HAL lib %s: %s\n", lib_path, error);
	   return -1;
   }

   hmi = (struct sensors_module_t *) dlsym(handle, HAL_MODULE_INFO_SYM_AS_STR);
   if (!hmi)
   {
	   SENSOR_LOGE(LOG_TAG "unable to find %s entry point in HAL\n", HAL_MODULE_INFO_SYM_AS_STR);
	   return -1;
   }

   err = hmi->common.methods->open((struct hw_module_t *)hmi, SENSORS_HARDWARE_POLL, &dev);
   if (err)
   {
	   SENSOR_LOGE(LOG_TAG "failed to open senor HAL: %d\n", err);
	   return -1;
   }

   poll_dev = (struct sensors_poll_device_1 *)dev;
   poll_dev_v0 = (struct sensors_poll_device_t *)dev;
   if (NULL == poll_dev)
   {
	   SENSOR_LOGE(LOG_TAG "poll_dev: sensors_poll_device_t() failed \n");
	   return -1;
   }
   if (NULL == poll_dev_v0)
   {
	   SENSOR_LOGE(LOG_TAG "poll_dev_v0: sensors_poll_device_t() failed \n");
	   return -1;
   }
   return 0;
}

void read_live_data()
{
	sensors_event_t events[BUFFER_EVENT];
	int count = 0;
	static uint64_t ts_cur = 0;
	static uint64_t ts_prv = 0;
	static uint64_t sensor_ts = 0;
	int event = 0;
	SENSOR_LOGI(LOG_TAG "Live: Polling sensor data\n");
	ts_cur = getTimestamp();
	ts_prv = ts_cur;
	sensor_ts = ts_cur;
	while(1)
	{
	  count = poll_dev->poll(poll_dev_v0, events, sizeof(events) / sizeof(sensors_event_t));
	  sensor_ts = events[count - 1].timestamp;
	  ts_cur = getTimestamp();
	  SENSOR_LOGI(LOG_TAG "Live: read events=%d, batch delta in ms=%lld latency in ms=%lld\n", count, (ts_cur - ts_prv) / 1000000, (ts_cur - sensor_ts) / 1000000);
	  ts_prv = ts_cur;
	  for(int i = 0; i < count; i++)
		  dump_event(&events[i]);
	}
}

void *sensor_live_data_poll(void *arg)
{
	read_live_data();
}

void read_buffer_data()
{
	SensorBuffread(sensor_type);
}

void *sensor_buffer_poll(void *arg)
{
	read_buffer_data();
	pthread_exit(NULL);
}

void read_sensor_temperature()
{
	float temperature = 0;
	while(1)
	{
		sleep(1);
		tempSensorDataPollTask(&temperature, sensor_type);
		SENSOR_LOGI(LOG_TAG "temperature: %f\n", temperature);
	}
}

void *sensor_temperature_poll(void *arg)
{
	read_sensor_temperature();
}

int main(int argc, char **argv)
{
	char *lib = NULL;
	char *stopstring = NULL;
	char enable[10];
	int state = 0;
	int ip_char = 0;
	int enable_temperature = 0;
	int enable_buffer = 0;
	int enable_live = 0;
	int get_sensor_list = 0;
	int service_run = 0;
	pthread_t livetid;
	pthread_t buffertid;
	pthread_t temptid;

	if(argc < 2)
		printHelp();

	//checking command line argument
	while ((ip_char = getopt (argc, argv, CMD_OPTIONS)) != -1)
	{
		//SENSOR_LOGI(LOG_TAG " %s : char %d\n", __func__, ip_char);
		switch (ip_char)
		{
			case 'l':
				lib = argv[2];
				if(strstr(lib, "asm3330") != NULL) {
					sensor_type = SENSOR_ASM330;
				}
				if(strstr(lib, "smi130") != NULL) {
					sensor_type = SENSOR_SMI130;
				}
				if(strstr(lib, "smi230") != NULL) {
					sensor_type = SENSOR_SMI230;
				}
				if(strstr(lib, "iam20680") != NULL) {
					sensor_type = SENSOR_IAM20680;
				}
				SENSOR_LOGV(LOG_TAG "sensor_type %d\n", sensor_type);
				break;
			case 'g':
				get_sensor_list = strtol(optarg, &stopstring, 10);
				break;
			case 'b':
				batch_time = strtol(optarg, &stopstring, 10);
				break;
			case 'd':
				odr = strtol(optarg, &stopstring, 10);
				break;
			case 's':
				service_run = strtol(optarg, &stopstring, 10);
				break;
			case 'a':
				enable_live = strtol(optarg, &stopstring, 10);
				break;
			case 't':
				enable_temperature = strtol(optarg, &stopstring, 10);
				break;
			case 'n':
				enable_buffer = strtol(optarg, &stopstring, 10);
				break;
			default:
				printHelp();
				return 0;
		}
	}

	if (sensors_initialize(lib) == -1)
	{
		SENSOR_LOGE(LOG_TAG "sensor initialization failed\n");
	}

	SENSOR_LOGI(LOG_TAG "batch_time = %lld odr %lld enable_temperature %d enable_buffer %d enable_live %d\n", batch_time, odr, enable_temperature, enable_buffer, enable_live);

	//register signal handler
	//struct sigaction action;
	//memset(&action, 0, sizeof(action));
	//action.sa_handler = term;
	//sigaction(SIGTERM, &action, NULL);

	if(get_sensor_list) {
		const struct sensor_t *s = NULL;
		int sensor_num = 0;
		sensor_num = get_sensor_list_all(&s);
		SENSOR_LOGI("sensor_num %d\n",sensor_num);
	}

	//Initialize Temp Sensor
	if (tempSensorDataInit(sensor_type))
	{
		SENSOR_LOGV("tempture supported \n");
		mTempSupported = true;
	}	else 	{
		SENSOR_LOGE("tempture feature not supported \n");
	}

	//Check Buffer support
	if (CheckBufferReadFile(sensor_type))
	{
		SENSOR_LOGV("buffer feature is supported \n");
		mBufferSupported = true;
	}	else 	{
		SENSOR_LOGE("buffer feature is not supported \n");
	}

	// checking flashback data
	if (enable_buffer && mBufferSupported)
	{
		if (pthread_create(&buffertid, NULL, sensor_buffer_poll, NULL) != 0)
		{
			SENSOR_LOGE(LOG_TAG "Buffer: pthread_create() error\n");
			exit(1);
		}
	}

	// checking temp data
	if (enable_temperature && mTempSupported)
	{
		if (pthread_create(&temptid, NULL, sensor_temperature_poll, NULL) != 0)
		{
			SENSOR_LOGE(LOG_TAG "temp: pthread_create() error\n");
			exit(1);
		}
	}

	// checking live data
	if (service_run)
	{
		SENSOR_LOGI(LOG_TAG "as a service running\n");
		if (enable_live)
		{
			configure_activate_sensors();
		}
		read_live_data();
	}
	else {
		// create thread to read live data
		if (enable_live)
		{
			configure_activate_sensors();
			if (pthread_create(&livetid, NULL, sensor_live_data_poll, poll_dev) != 0)
			{
				SENSOR_LOGE(LOG_TAG "pthread_create() error\n");
				exit(1);
			}
		}
		state = SENSOR_ENABLE;
		while (true)
		{
			memset(enable, 0, sizeof(enable) / sizeof(enable[0]));
			fgets(enable, sizeof(enable) / sizeof(enable[0]), stdin);
			state = strtol(enable, &stopstring, 10);
			if (state == 1)
				activate_sensors();
			else if (state == 2)
				deactivate_sensors();
			else if (state == 3)
				break;
			else
				SENSOR_LOGE(LOG_TAG "Wrong Input\n");
		}
	}
	SENSOR_LOGI(LOG_TAG "Exiting main app\n");
	return 0;
}
