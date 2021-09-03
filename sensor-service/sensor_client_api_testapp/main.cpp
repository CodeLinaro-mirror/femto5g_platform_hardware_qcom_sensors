/* Copyright (c) 2021 The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation, nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h>
#include <string.h>
#include <sys/types.h>
#include <SensorClientApi.h>
#include <pwd.h>

#define CMD_OPTIONS     "b:d:"

using namespace sensor_client;

SensorClient* pClient;

void Usage(void)
{
  printf("\nUsage:   ./sensor_test -d odr_rate -b batch_count\n");
  printf("\t\t\tex: ./sensor_test -d 104 -b 10\n");
  return;
}

static uint64_t getTimestamp() {
    struct timespec ts;
    clock_gettime(CLOCK_BOOTTIME, &ts);
    uint64_t system_ts =
            ((uint64_t)(ts.tv_sec)) * 1000000000ULL + ((uint64_t)(ts.tv_nsec));
    return system_ts;
}

static void PrintSensorList(struct sensor_list *sensor, int sensor_count)
{
   printf("sensor_count %d\n", sensor_count);
   for (int i=0 ; i< sensor_count ; i++) {
           printf("%s\n",sensor[i].name);
           printf("\tvendor: %s\n",sensor[i].vendor);
           printf("\tsensor_id: %d\n",sensor[i].sensor_id);
           printf("\ttype: %d\n",sensor[i].type);
           printf("\trange: %d\n",sensor[i].range);
           printf("\tmaxSamplingRate: %d\n",sensor[i].maxSamplingRate);
           printf("\tminBatchCount: %d\n",sensor[i].minBatchCount);
           printf("\tmaxBatchCount: %d\n",sensor[i].maxBatchCount);
           printf("\todr rate: %fHZ %fHZ %fHZ %fHZ %fHZ %fHZ\n",
                           sensor[i].odr[0],sensor[i].odr[1],sensor[i].odr[2],
                           sensor[i].odr[3],sensor[i].odr[4],sensor[i].odr[5]);
   }
}

static void PrintSensorMlcCaseList(struct sensor_mlc_case_list *mlc_case, int mlc_case_count)
{
   printf("mlc_case_count %d\n", mlc_case_count);
   for (int i=0 ; i< mlc_case_count ; i++) {
           printf("mlc_case_count %d: %s\n", i, mlc_case[i].name);
   }
}

static void dump_event(const struct sensors_event_t *e)
{
    static int64_t acc_ts = 0;
    static int64_t gyro_ts = 0;

    switch (e->type) {
    case SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED:
        printf("ACC event:x=%f y=%f z=%f x_bias=%.2f y_bias=%.2f z_bias=%.2f timestamp=%lld s&s delta %lld HZ=%f\n",
            e->uncalibrated_accelerometer.x_uncalib, e->uncalibrated_accelerometer.y_uncalib,
            e->uncalibrated_accelerometer.z_uncalib, e->uncalibrated_accelerometer.x_bias,
            e->uncalibrated_accelerometer.y_bias, e->uncalibrated_accelerometer.z_bias,
            e->timestamp, (getTimestamp() - e->timestamp)/1000000, (1.0F/(e->timestamp-acc_ts))*1000000000);
        acc_ts = e->timestamp;
        break;
    case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
        printf("GYRO event:x=%f y=%f z=%f x_bias=%.2f y_bias=%.2f z_bias=%.2f timestamp=%lld s&s delta %lld HZ=%f\n",
            e->uncalibrated_gyro.x_uncalib, e->uncalibrated_gyro.y_uncalib,
            e->uncalibrated_gyro.z_uncalib, e->uncalibrated_gyro.x_bias,
            e->uncalibrated_gyro.y_bias, e->uncalibrated_gyro.z_bias,
            e->timestamp,(getTimestamp() - e->timestamp)/1000000, (1.0F/(e->timestamp-gyro_ts))*1000000000);
        gyro_ts = e->timestamp;
        break;
    default:
        printf("Unknown sensor_id events %d\n", e->type);
    }
}


static void onCapabilitiesCb(SensorCapabilitiesMask mask) {
    printf("<<< onCapabilitiesCb mask=%d\n", mask);
    switch (mask) {
	case SHD_READY:
		printf("Sensor Hal daemon is Ready to commnunicate\n");
		break;
	case SHD_RESTARTED:
		printf("Sensor Hal daemon is Restarted\n");
		break;
	case SHD_NOT_RUNNING:
		printf("Sensor Hal daemon is not available\n");
		break;
	case DEVICE_SUSPEND:
		printf("Device is about go to suspend state\n");
		break;
	case DEVICE_RESUME:
		printf("Device is resumed\n");
		break;
	case DEVICE_SHUTDOWN:
		printf("Device is about go to shutdown\n");
		break;
	defult:
		printf("Unknown mask\n");
		break;
    }
}

static void onBatchingCb(int sensor_id , float sampling_rate, int batch_count)
{
  printf("<<< sensor_client_test_app: hanndle %d sampling_rate %f batch_count %d \n", sensor_id, sampling_rate, batch_count);
}

static void onSensorDataReadCb(int sensor_id, const sensors_event_t *events, uint32_t count)
{
    int i =0;
    static uint64_t ts_prv_acc = 0, ts_prv_gyro = 0 , ts_cur = 0;
    static uint64_t acc_sensor_ts = 0, gyro_sensor_ts = 0;
    ts_cur = getTimestamp();

    if (sensor_id == 1) {
	    acc_sensor_ts = events[count-1].timestamp;
	    printf("<<<sensor_id %d: read events count %d batch delta %lld sensor and system delta %lld\n",
			    sensor_id, count, (ts_cur - ts_prv_acc)/1000000, (ts_cur - acc_sensor_ts)/1000000);
	    ts_prv_acc = ts_cur;
    }
    else {
	    gyro_sensor_ts = events[count-1].timestamp;
	    printf("<<<sensor_id %d: read events count %d batch delta %lld sensor and system delta %lld\n",
			    sensor_id, count, (ts_cur - ts_prv_gyro)/1000000, (ts_cur - gyro_sensor_ts)/1000000);
	    ts_prv_gyro = ts_cur;
    }
    for ( i = 0; i < count ; i++)
	    dump_event(&events[i]);
}

static void onSensorMLCEventCb(const char *case_name, struct mlc_event_data *event)
{
    unsigned char *pevent;

    pevent = (unsigned char *)event;

    printf("%s Event: %x %x %x %x %x %x %x %x time: %lld\n", case_name,
                    pevent[0], pevent[1], pevent[2], pevent[3],
                    pevent[4], pevent[5], pevent[6], pevent[7],
		    event->timestamp);
}

static void onMfifoDataReadCb(int sensor_id, const sensors_event_t *events, uint32_t count)
{
    printf("<<<sensor_id %d: read events count %d\n", sensor_id, count);
    for (int i = 0; i < count ; i++)
	    dump_event(&events[i]);
}

static void onSensorBufferDataReadCb(const sensors_event_t *events, uint32_t count)
{
    int i =0;
    printf("<<< sensor_client_api_test_app: buffer read events count %d\n",count);
    for ( i = 0; i < count ; i++)
            dump_event(&events[i]);
}

static void onSensorTempReadCb(float tempreature)
{
   printf("<<< sensor_client_test_app: tempreature %f \n",tempreature);
}

static void printHelp() {
    printf("\n************* options *************\n");
    printf("h: help\n");
    printf("g: Get Sensor list\n");
    printf("c: configure Sensor sampling and batch count\n");
    printf("a: Activate/Deactivate the Sensor\n");
    printf("p: Start reading Sensor Data\n");
    printf("t: Read Sensor Temperature\n");
    printf("b: Read Sensor Bufffer Data\n");
    printf("f: flush/Delete Sensor Bufffer Data\n");
    printf("m: get mlc case list\n");
    printf("e: enable/disable mlc case event\n");
    printf("q: Quit\n");
}

/******************************************************************************
Main function
******************************************************************************/
int main(int argc, char *argv[]) {
   char *stopstring;
   char odr[10];
   char enable[10];
   char batchcount[10];
   int batch_count = 0;
   sensor_state state = SENSOR_DISABLE;
   int mlc_enable = 0;
   int i = 0, j= 0;
   int ret = 0;
   struct sensor_list *sensor;
   int sensor_count = 0;
   struct sensor_mlc_case_list *mlc_case;
   int mlc_case_count = 0;
   float odr_rate = 0;
   int c;

   printHelp();
   sleep(1);

   pClient = new SensorClient(onCapabilitiesCb);

   ret = pClient->get_sensor_list(&sensor, &sensor_count);
   if(ret < 0) {
           printf("get sensor list failed ret %d \n", ret);
   }

   PrintSensorList(sensor,sensor_count);


   if(argc > 2) {
     Usage();
     while ((c = getopt (argc, argv, CMD_OPTIONS)) != -1) {
	   switch (c) {
		   case 'b':
			   batch_count = strtol(optarg, &stopstring, 10);
			   break;
		   case 'd':
			   odr_rate = strtol(optarg, &stopstring, 10);
			   break;
		   default:
			   Usage();
			   break;
	   }
     }
     printf(" Sensor odr rate %f batch count %d\n", odr_rate, batch_count);

     for(int i=0; i < sensor_count; i++) {
	   state = SENSOR_DISABLE;
	   ret = pClient->sensor_control(sensor[i].sensor_id, state);
	   if(ret < 0) {
		   printf("sensor control failed sensor[i].sensor_id %d ret %d \n", sensor[i].sensor_id, ret);
	   }
	   ret = pClient->sensor_config(sensor[i].sensor_id, odr_rate, batch_count, onBatchingCb);
	   if(ret < 0) {
		   printf("sensor config  failed sensor[i].sensor_id %d ret %d \n", sensor[i].sensor_id, ret);
	   }
	   state = SENSOR_ENABLE;
	   ret = pClient->sensor_control(sensor[i].sensor_id, state);
	   if(ret < 0) {
		   printf("sensor control failed sensor[i].sensor_id %d ret %d \n", sensor[i].sensor_id, ret);
	   }
	   ret = pClient->sensor_read_events(sensor[i].sensor_id, onSensorDataReadCb);
	   if(ret < 0) {
		   printf("sensor read events failed ret %d \n", ret);
	   }
     }
   }

   // main loop
   while (1) {
	   char buf[10];
	   memset (buf, 0, sizeof(buf)/sizeof(buf[0]));
	   fgets(buf, sizeof(buf)/sizeof(buf[0]), stdin);
	   int command = buf[0];
    switch(command) {

     case 'g':
	if (pClient) {
         ret = pClient->get_sensor_list(&sensor, &sensor_count);
	 if(ret < 0) {
		printf("get sensor list failed ret %d \n", ret);
		break;
	}
	PrintSensorList(sensor,sensor_count);
	break;

     case 'c':
	if (pClient) {
	   for(int i=0; i < sensor_count; i++) {
		printf("Enter 0:%fHZ 1:%fHZ 2:%fHZ 3:%fHZ 4:%fHZ 5:%fHZ\n",
                       sensor[i].odr[0],sensor[i].odr[1],sensor[i].odr[2],
		       sensor[i].odr[3],sensor[i].odr[4],sensor[i].odr[5]);

		memset (odr, 0, sizeof(odr)/sizeof(odr[0]));
		memset (batchcount, 0, sizeof(batchcount)/sizeof(batchcount[0]));
		fgets(odr, sizeof(odr)/sizeof(odr[0]), stdin);
		j=strtol(odr,&stopstring,10);
		printf("Enter batch set: ");
		fgets(batchcount, sizeof(batchcount)/sizeof(batchcount[0]), stdin);
		batch_count=strtol(batchcount,&stopstring,10);

		printf("\nConfiguring sensor sensor_id %d  sampling rate %fHZ and batch count %d\n",
				sensor[i].sensor_id, sensor[i].odr[j], batch_count);

		ret = pClient->sensor_config(sensor[i].sensor_id, sensor[i].odr[j], batch_count, onBatchingCb);
		if(ret < 0) {
			printf("sensor config  failed ret %d \n", ret);
		}
	   }
	}
	break;

     case 'a':
	if (pClient) {
           for(int i=0; i < sensor_count; i++) {
	        printf("Enable/Disable the sensor sensor[i].sensor_id %d\n",sensor[i].sensor_id);
		printf(" 0:SENSOR_DISABLE\n 1:SENSOR_ENABLE\n 2:SENSOR_LPM\n 3:SENSOR_HPM\nEnter value:");
		memset(enable, 0, sizeof(enable)/sizeof(enable[0]));
		fgets(enable, sizeof(enable)/sizeof(enable[0]), stdin);
                state=(sensor_state)strtol(enable,&stopstring,10);
		printf("sensor id %d with contol value %d\n",sensor[i].sensor_id, state);
		ret = pClient->sensor_control(sensor[i].sensor_id, state);
		if(ret < 0) {
			printf("sensor control failed ret %d \n", ret);
			break;
		}
	   }
	}
	break;

     case 'p':
	if (pClient) {
		printf("start reading the sensor data\n");
		for(int i=0; i < sensor_count; i++) {
			ret = pClient->sensor_read_events(sensor[i].sensor_id, onSensorDataReadCb);
			if(ret < 0) {
				printf("sensor read events failed ret %d \n", ret);
				break;
			}
		}
	}
	break;

     case 't':
	if (pClient) {
		printf("read temperature\n");
		ret = pClient->sensor_read_temperature(onSensorTempReadCb);
		if(ret < 0) {
			printf("sensor read temperature failed ret %d \n", ret);
			break;
		}
	}
	break;

     case 'b':
	if (pClient) {
		printf("read buffer data\n");
		ret = pClient->sensor_read_buffer_data(1, onSensorBufferDataReadCb);
		if(ret < 0) {
			printf("sensor buffer read failed ret %d \n", ret);
			break;
		}
	}
	break;
     case 'f':
	if (pClient) {
		printf("flush/Delete the buffer data\n");
		ret = pClient->sensor_read_buffer_data(0, onSensorBufferDataReadCb);
		if(ret < 0) {
			printf("sensor buffer read failed ret %d \n", ret);
			break;
		}
		else
			printf("buffer deleted\n");
	}
	break;
     case 'm':
        if (pClient) {
                printf("get mlc case list supported\n");
                ret = pClient->sensor_request_mlc_case(&mlc_case, &mlc_case_count);
                if(ret < 0) {
                        printf("sensor get mlc case list failed ret %d \n", ret);
                        break;
                }
		PrintSensorMlcCaseList(mlc_case, mlc_case_count);
        }
	break;
     case 'e':
        if (pClient) {
                printf("enable/disable mlc case event\n");
		printf(" 0:DISABLE\n 1:ENABLE\nEnter value:");
		memset(enable, 0, sizeof(enable)/sizeof(enable[0]));
		fgets(enable, sizeof(enable)/sizeof(enable[0]), stdin);
		mlc_enable=strtol(enable,&stopstring,10);
		printf("mlc_enable %d\n", mlc_enable);
		for(int i=0; i < mlc_case_count; i++) {
			ret = pClient->sensor_mlc_event_enable(mlc_case[i].name, mlc_enable, onSensorMLCEventCb, onMfifoDataReadCb);
			if(ret < 0) {
				printf("sensor enable/disable mlc case event failed ret %d \n", ret);
				break;
			}
		}
	}
	break;
     case 'h':
	printHelp();
	break;

     case 'q':
	goto EXIT;
	break;

     default:
	printf("unknown command %s\n", buf);
	break;
	}
    }
   }//while(1)

EXIT:
   if (pClient) {
	   delete pClient;
   }
   printf("Done\n");
   exit(0);
}
