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
 * Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
 *
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the
 * disclaimer below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 * GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 * HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h>
#include <string.h>
#include <sys/types.h>
#include <SensorClientApi.h>
#include <pwd.h>
#include <utils/Log.h>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>

#define CMD_OPTIONS     "l:s:b:n:e:t:"

#undef LOG_TAG
#define LOG_TAG "Sensor-Test-App:"

//#define LOGCAT_ENABLED // for getting logs in the logcat

#ifdef LOGCAT_ENABLED
#define SENSOR_LOGE(...) { ALOGE(__VA_ARGS__); }
#define SENSOR_LOGW(...) { ALOGW(__VA_ARGS__); }
#define SENSOR_LOGI(...) { ALOGI(__VA_ARGS__); }
#define SENSOR_LOGD(...) { ALOGD(__VA_ARGS__); }
#define SENSOR_LOGV(...) { ALOGV(__VA_ARGS__); }
#else
#define SENSOR_LOGE(...) { (void)printf(__VA_ARGS__); }
#define SENSOR_LOGW(...) { (void)printf(__VA_ARGS__); }
#define SENSOR_LOGI(...) { (void)printf(__VA_ARGS__); }
#define SENSOR_LOGD(...) { (void)printf(__VA_ARGS__); }
#define SENSOR_LOGV(...) { (void)printf(__VA_ARGS__); }
#endif

static uint64_t start_time = 0, end_time = 0, selftest_time = 0;
using namespace sensor_client;
SensorClient* pClient;
static int live_count = 0;

std::queue<std::vector<sensors_event_t>> dataQueue;
std::mutex queueMutex;
std::condition_variable dataCondition;
std::atomic<bool> running(true);

void Usage(void)
{
  SENSOR_LOGI(LOG_TAG "\nUsage:\nsensor_client_api_testapp --live <enable> --sampling <> --batch <> --count <> --temperature <enbale> --buffer <enable>\n");
  SENSOR_LOGI(LOG_TAG "Ex: sensor_client_api_testapp -l 1 -s 104 -b 10 -n 10 -e 1 -t 1\n");
  return;
}

static uint64_t getTimestamp() {
    struct timespec ts;
    (void)clock_gettime(CLOCK_BOOTTIME, &ts);
    uint64_t system_ts =
            ((uint64_t)(ts.tv_sec)) * 1000000000ULL + ((uint64_t)(ts.tv_nsec));
    return system_ts;
}

static void PrintSensorList(struct sensor_list *sensor, int sensor_count)
{
   SENSOR_LOGI(LOG_TAG "sensor_count %d\n", sensor_count);
   for (int i=0 ; i< sensor_count ; i++) {
           SENSOR_LOGI(LOG_TAG "%s\n",sensor[i].name);
           SENSOR_LOGI(LOG_TAG "\tvendor: %s\n",sensor[i].vendor);
           SENSOR_LOGI(LOG_TAG "\tversion: %d\n",sensor[i].version);
           SENSOR_LOGI(LOG_TAG "\tresolution: %f\n",sensor[i].resolution);
           SENSOR_LOGI(LOG_TAG "\tmaxRange %f\n",sensor[i].maxRange);
           SENSOR_LOGI(LOG_TAG "\tsensor_id: %d\n",sensor[i].sensor_id);
           SENSOR_LOGI(LOG_TAG "\ttype: %d\n",sensor[i].type);
           SENSOR_LOGI(LOG_TAG "\trange: %d\n",sensor[i].range);
           SENSOR_LOGI(LOG_TAG "\tmaxSamplingRate: %f\n",sensor[i].maxSamplingRate);
           SENSOR_LOGI(LOG_TAG "\tminBatchCount: %d\n",sensor[i].minBatchCount);
           SENSOR_LOGI(LOG_TAG "\tmaxBatchCount: %d\n",sensor[i].maxBatchCount);
           SENSOR_LOGI(LOG_TAG "\todr rate: %fHZ %fHZ %fHZ %fHZ %fHZ %fHZ\n",
                           sensor[i].odr[0],sensor[i].odr[1],sensor[i].odr[2],
                           sensor[i].odr[3],sensor[i].odr[4],sensor[i].odr[5]);
   }
}

static void PrintSensorMlcCaseList(struct sensor_mlc_case_list *mlc_case, int mlc_case_count)
{
   SENSOR_LOGI(LOG_TAG "mlc_case_count %d\n", mlc_case_count);
   for (int i=0 ; i< mlc_case_count ; i++) {
           SENSOR_LOGI(LOG_TAG "mlc_case_count %d: %s\n", i, mlc_case[i].name);
   }
}

static void dump_live_event(const struct sensors_event_t *e)
{
    static int64_t acc_ts = 0;
    static int64_t gyro_ts = 0;
    static int AccCount = 0, GyroCount = 0;
    switch (e->type) {
    case SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED:
    case SENSOR_TYPE_ACCELEROMETER:
	SENSOR_LOGI(LOG_TAG "Sensor ACC Live event:%d x=%f y=%f z=%f x_bias=%.2f y_bias=%.2f z_bias=%.2f timestamp=%lld latency in ms=%lld sampling rate in ms=%lld\n",
            AccCount++, e->uncalibrated_accelerometer.x_uncalib, e->uncalibrated_accelerometer.y_uncalib,
            e->uncalibrated_accelerometer.z_uncalib, e->uncalibrated_accelerometer.x_bias,
            e->uncalibrated_accelerometer.y_bias, e->uncalibrated_accelerometer.z_bias,
            e->timestamp, (getTimestamp() - e->timestamp)/1000000, (e->timestamp-acc_ts)/1000000);
        acc_ts = e->timestamp;
        break;
    case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
    case SENSOR_TYPE_GYROSCOPE:
	SENSOR_LOGI(LOG_TAG "Sensor GYRO Live event:%d x=%f y=%f z=%f x_bias=%.2f y_bias=%.2f z_bias=%.2f timestamp=%lld latency in ms=%lld sampling rate in ms=%lld\n",
            GyroCount++, e->uncalibrated_gyro.x_uncalib, e->uncalibrated_gyro.y_uncalib,
            e->uncalibrated_gyro.z_uncalib, e->uncalibrated_gyro.x_bias,
            e->uncalibrated_gyro.y_bias, e->uncalibrated_gyro.z_bias,
            e->timestamp,(getTimestamp() - e->timestamp)/1000000, (e->timestamp-gyro_ts)/1000000);
        gyro_ts = e->timestamp;
        break;
    default:
        SENSOR_LOGI(LOG_TAG "Sensor Live unknown sensor_id events %d\n", e->type);
    }
}

static void dump_buffer_event(const struct sensors_event_t *e)
{
    static int64_t acc_ts = 0;
    static int64_t gyro_ts = 0;
    static int AccCount = 0, GyroCount = 0;

    switch (e->type) {
    case SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED:
    case SENSOR_TYPE_ACCELEROMETER:
	SENSOR_LOGI(LOG_TAG "Sensor ACC Buff event:%d x=%f y=%f z=%f x_bias=%.2f y_bias=%.2f z_bias=%.2f timestamp=%lld sampling rate in ms=%lld\n",
            AccCount++, e->uncalibrated_accelerometer.x_uncalib, e->uncalibrated_accelerometer.y_uncalib,
            e->uncalibrated_accelerometer.z_uncalib, e->uncalibrated_accelerometer.x_bias,
            e->uncalibrated_accelerometer.y_bias, e->uncalibrated_accelerometer.z_bias,
            e->timestamp, (e->timestamp-acc_ts)/1000000);
        acc_ts = e->timestamp;
        break;
    case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
    case SENSOR_TYPE_GYROSCOPE:
	SENSOR_LOGI(LOG_TAG "Sensor GYRO Buff event:%d x=%f y=%f z=%f x_bias=%.2f y_bias=%.2f z_bias=%.2f timestamp=%lld sampling rate in ms=%lld\n",
            GyroCount++, e->uncalibrated_gyro.x_uncalib, e->uncalibrated_gyro.y_uncalib,
            e->uncalibrated_gyro.z_uncalib, e->uncalibrated_gyro.x_bias,
            e->uncalibrated_gyro.y_bias, e->uncalibrated_gyro.z_bias,
            e->timestamp,(e->timestamp-gyro_ts)/1000000);
        gyro_ts = e->timestamp;
        break;
    default:
        SENSOR_LOGI(LOG_TAG "Sensor Buff unknown sensor_id events %d\n", e->type);
    }
}


static void onCapabilitiesCb(SensorCapabilitiesMask mask) {
    SENSOR_LOGI(LOG_TAG "<<< Recieved onCapabilitiesCb mask=%d\n", mask);
    switch (mask) {
	case SHD_READY:
		SENSOR_LOGI(LOG_TAG "Sensor Hal daemon is Ready to commnunicate\n");
		break;
	case SHD_RESTARTED:
		SENSOR_LOGI(LOG_TAG "Sensor Hal daemon is Restarted\n");
		break;
	case SHD_NOT_RUNNING:
		SENSOR_LOGI(LOG_TAG "Sensor Hal daemon is not available\n");
		break;
	case DEVICE_SUSPEND:
		SENSOR_LOGI(LOG_TAG "Device is about go to suspend state\n");
		break;
	case DEVICE_RESUME:
		SENSOR_LOGI(LOG_TAG "Device is resumed\n");
		break;
	case DEVICE_SHUTDOWN:
		SENSOR_LOGI(LOG_TAG "Device is about go to shutdown\n");
		break;
	case ACCEL_SELFTEST_FAIL:
		SENSOR_LOGI(LOG_TAG "Accel self-test fail\n");
                break;
	case GYRO_SELFTEST_FAIL:
		SENSOR_LOGI(LOG_TAG "Gyro self-test fail\n");
                break;
	case ACCEL_GYRO_BOTH_SELFTEST_FAIL:
		SENSOR_LOGI(LOG_TAG "Accel and Gyro self-test both failed\n");
                break;
	defult:
		SENSOR_LOGI(LOG_TAG "Unknown mask\n");
		break;
    }
}

static void onBatchingCb(int sensor_id , float sampling_rate, int batch_count, bool Rotate)
{
  SENSOR_LOGI(LOG_TAG "Recieved Sensor id %d sampling_rate %f batch_count %d Rotate %d\n", sensor_id, sampling_rate, batch_count, Rotate);
}

void workerThread() {
   int i =0;
   static uint64_t ts_prv_acc = 0, ts_prv_gyro = 0 , ts_cur = 0;
   static uint64_t acc_sensor_ts = 0, gyro_sensor_ts = 0;
   while (running) {
      std::unique_lock<std::mutex> lock(queueMutex);
      dataCondition.wait(lock, [] { return !dataQueue.empty() || !running; });

      while (!dataQueue.empty()) {
	      std::vector<sensors_event_t> events = std::move(dataQueue.front());
	      dataQueue.pop();
	      lock.unlock();
	      int count =  events.size();
	      int sensor_id = events[0].sensor;
	      ts_cur = getTimestamp();

	      if (sensor_id == 1) {
		      acc_sensor_ts = events[count-1].timestamp;
		      SENSOR_LOGI(LOG_TAG "Sensor ACC Live: sensor_id %d: read events count %d batch delta in ms=%lld latency in ms=%lld\n",
				      sensor_id, count, (ts_cur - ts_prv_acc)/1000000, (ts_cur - acc_sensor_ts)/1000000);
		      ts_prv_acc = ts_cur;
	      }
	      else {
		      gyro_sensor_ts = events[count-1].timestamp;
		      SENSOR_LOGI(LOG_TAG "Sensor GYRO Live: sensor_id %d: read events count %d batch delta in ms=%lld latency in ms=%lld\n",
				      sensor_id, count, (ts_cur - ts_prv_gyro)/1000000, (ts_cur - gyro_sensor_ts)/1000000);
		      ts_prv_gyro = ts_cur;
	      }
	      // Process the events
	      for (const auto& event : events) {
		    dump_live_event(&event);
	      }
	      lock.lock();
      }
   }
}

static void onSensorTempReadCb(float tempreature)
{
   SENSOR_LOGI(LOG_TAG "tempreature %f \n",tempreature);
}

void temperatureThread() {
    int ret = 0;
    while(1)
    {
	ret = pClient->sensor_read_temperature(onSensorTempReadCb);
	if(ret < 0) {
		SENSOR_LOGE(LOG_TAG "sensor read temperature failed ret %d \n", ret);
		break;
	}
	(void)sleep(1);
    }
}
static void onSensorDataReadCb(int sensor_id, const sensors_event_t *events, uint32_t count)
{
    std::vector<sensors_event_t> eventBatch(events, events + count);
    {
	    std::lock_guard<std::mutex> lock(queueMutex);
	    dataQueue.push(std::move(eventBatch));
    }
    dataCondition.notify_one();
}

static void onSensorMLCEventCb(const char *case_name, struct mlc_event_data *event)
{
    unsigned char *pevent;
    pevent = (unsigned char *)event;
    SENSOR_LOGI(LOG_TAG "%s Event: %x %x %x %x %x %x %x %x time: %lld\n", case_name,
                    pevent[0], pevent[1], pevent[2], pevent[3],
                    pevent[4], pevent[5], pevent[6], pevent[7],
		    event->timestamp);
}

static void onMfifoDataReadCb(int sensor_id, const sensors_event_t *events, uint32_t count)
{
    SENSOR_LOGI(LOG_TAG "sensor_id %d: read events count %d\n", sensor_id, count);
    for (int i = 0; i < count ; i++)
	    dump_live_event(&events[i]);
}

static void onSensorBufferDataReadCb(const sensors_event_t *events, uint32_t count)
{
    int i =0;
    SENSOR_LOGI(LOG_TAG "Sensor ACC/GYRO Buff: buffer read events count %d\n",count);
    for ( i = 0; i < count ; i++)
            dump_buffer_event(&events[i]);
}

static void onSelfTestResultCallback(int sensor_id, int request_id, SelfTestResult result, SelfTestResultType resultType, uint64_t timestamp)
{
   end_time = getTimestamp();
   selftest_time = (end_time - start_time);
   SENSOR_LOGI(LOG_TAG "\nselftest time taken = %lldms\n", selftest_time/1000000);
   SENSOR_LOGI(LOG_TAG "self_test- sensor_id %d request_id %d result %d resultType %d timestamp %lld\n",sensor_id, request_id, result, resultType, timestamp);
}

static void printHelp() {
    SENSOR_LOGI(LOG_TAG "\n************* options *************\n");
    SENSOR_LOGI(LOG_TAG "h: help\n");
    SENSOR_LOGI(LOG_TAG "g: Get Sensor list\n");
    SENSOR_LOGI(LOG_TAG "c: configure Sensor sampling and batch count\n");
    SENSOR_LOGI(LOG_TAG "a: Activate/Deactivate the Sensor\n");
    SENSOR_LOGI(LOG_TAG "p: Start reading Sensor Data\n");
    SENSOR_LOGI(LOG_TAG "t: Read Sensor Temperature\n");
    SENSOR_LOGI(LOG_TAG "b: Read Sensor Bufffer Data\n");
    SENSOR_LOGI(LOG_TAG "f: flush/Delete Sensor Bufffer Data\n");
    SENSOR_LOGI(LOG_TAG "m: get mlc case list\n");
    SENSOR_LOGI(LOG_TAG "e: enable/disable mlc case event\n");
    SENSOR_LOGI(LOG_TAG "r: set sensor rotation matrix\n");
    SENSOR_LOGI(LOG_TAG "q: Quit\n");
}

/******************************************************************************
Main function
******************************************************************************/
int main(int argc, char *argv[]) {
   char *stopstring;
   char odr[10];
   char enable[10];
   char batchcount[10];
   char rm[10];
   char rotate[10];
   bool Rotate = 0;
   int batch_count = 0;
   uint32_t roll = 0, pitch = 0, yaw = 0;
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
   int request_id = 1;
   static int enable_buffer = 0;
   static int enable_live = 0;
   static int enable_temperature = 0;
   std::thread temperature;

   SelfTestType selfTestType;


   pClient = new SensorClient(onCapabilitiesCb);

   (void)sleep(2);
   ret = pClient->get_sensor_list(&sensor, &sensor_count);
   if(ret < 0) {
           SENSOR_LOGE(LOG_TAG "get sensor list failed ret %d \n", ret);
   }

   PrintSensorList(sensor,sensor_count);
   Usage();

   std::thread worker(workerThread);

   if(argc > 2) {
     while ((c = getopt (argc, argv, CMD_OPTIONS)) != -1) {
	   switch (c) {
		   case 'l':
			   enable_live = strtol(optarg, &stopstring, 10);
			   break;
		   case 'b':
			   batch_count = strtol(optarg, &stopstring, 10);
			   break;
		   case 's':
			   odr_rate = strtol(optarg, &stopstring, 10);
			   break;
		   case 'n':
			   live_count = strtol(optarg, &stopstring, 10);
			   break;
		   case 'e':
			   enable_buffer = strtol(optarg, &stopstring, 10);
			   break;
		   case 't':
			   enable_temperature = strtol(optarg, &stopstring, 10);
		   default:
			   Usage();
			   break;
	   }
     }
     SENSOR_LOGI(LOG_TAG "Sensor enable_live %d sampling rate %f batch count %d live_count %d enable_buffer %d enable_temperature %d\n",
		     enable_live, odr_rate, batch_count, live_count, enable_buffer, enable_temperature);

     if(enable_buffer == 1){
	ret = pClient->sensor_read_buffer_data(1, onSensorBufferDataReadCb);
	if(ret < 0) {
		SENSOR_LOGE(LOG_TAG "sensor buffer read failed ret %d \n", ret);
	}
     }

    if(enable_live == 1) {
     for(int i=0; i < sensor_count; i++) {
	   state = SENSOR_DISABLE;
	   ret = pClient->sensor_control(sensor[i].sensor_id, state);
	   if(ret < 0) {
		   SENSOR_LOGE(LOG_TAG "sensor control failed sensor[i].sensor_id %d ret %d \n", sensor[i].sensor_id, ret);
	   }
	   ret = pClient->sensor_config(sensor[i].sensor_id, odr_rate, batch_count, Rotate, onBatchingCb);
	   if(ret < 0) {
		   SENSOR_LOGE(LOG_TAG "sensor config  failed sensor[i].sensor_id %d ret %d \n", sensor[i].sensor_id, ret);
	   }
	   state = SENSOR_ENABLE;
	   ret = pClient->sensor_control(sensor[i].sensor_id, state);
	   if(ret < 0) {
		   SENSOR_LOGE(LOG_TAG "sensor control failed sensor[i].sensor_id %d ret %d \n", sensor[i].sensor_id, ret);
	   }
	   ret = pClient->sensor_read_events(sensor[i].sensor_id, onSensorDataReadCb);
	   if(ret < 0) {
		   SENSOR_LOGE(LOG_TAG "sensor read events failed ret %d \n", ret);
	   }
     }
    }
    if (enable_temperature == 1) {
	SENSOR_LOGI(LOG_TAG "read temperature\n");
	temperature = std::thread(temperatureThread);
    }
     while(1) {
	/*Enable blocking call to avoid high cpu usage for test app*/
	char buf[10];
	(void)memset (buf, 0, sizeof(buf)/sizeof(buf[0]));
	(void)fgets(buf, sizeof(buf)/sizeof(buf[0]), stdin);
	int command = buf[0];
     }
   }

   printHelp();
   // main loop
   while (1) {
    char buf[10];
    (void)memset (buf, 0, sizeof(buf)/sizeof(buf[0]));
    (void)fgets(buf, sizeof(buf)/sizeof(buf[0]), stdin);
    int command = buf[0];

    switch(command) {
     case 'g':
	if (pClient) {
         ret = pClient->get_sensor_list(&sensor, &sensor_count);
	 if(ret < 0) {
		SENSOR_LOGE(LOG_TAG "get sensor list failed ret %d \n", ret);
		break;
	 }
	 PrintSensorList(sensor,sensor_count);
	}
	break;
     case 'c':
	if (pClient) {
	   for(int i=0; i < sensor_count; i++) {
		SENSOR_LOGI(LOG_TAG "Enter 0:%fHZ 1:%fHZ 2:%fHZ 3:%fHZ 4:%fHZ 5:%fHZ\n",
                       sensor[i].odr[0],sensor[i].odr[1],sensor[i].odr[2],
		       sensor[i].odr[3],sensor[i].odr[4],sensor[i].odr[5]);
		(void)memset (odr, 0, sizeof(odr)/sizeof(odr[0]));
		(void)memset (batchcount, 0, sizeof(batchcount)/sizeof(batchcount[0]));
		(void)fgets(odr, sizeof(odr)/sizeof(odr[0]), stdin);
		j=strtol(odr,&stopstring,10);

		SENSOR_LOGI(LOG_TAG "Enter batch set: ");
		(void)fgets(batchcount, sizeof(batchcount)/sizeof(batchcount[0]), stdin);
		batch_count=strtol(batchcount,&stopstring,10);

		SENSOR_LOGI(LOG_TAG "Enter Rotate:1  or Rotate:0 ");
		(void)fgets(rotate, sizeof(rotate)/sizeof(rotate[0]), stdin);
		Rotate=strtol(rotate,&stopstring,10);

		SENSOR_LOGI(LOG_TAG "\nConfiguring sensor sensor_id %d  sampling rate %fHZ and batch count %d Rotate %d\n",
				sensor[i].sensor_id, sensor[i].odr[j], batch_count, Rotate);

		ret = pClient->sensor_config(sensor[i].sensor_id, sensor[i].odr[j], batch_count, Rotate, onBatchingCb);
		if(ret < 0) {
			SENSOR_LOGE(LOG_TAG "sensor config  failed ret %d \n", ret);
		}
	   }
	}
	break;
     case 'a':
	if (pClient) {
           for(int i=0; i < sensor_count; i++) {
	        SENSOR_LOGI(LOG_TAG "Enable/Disable the sensor sensor[i].sensor_id %d\n",sensor[i].sensor_id);
		SENSOR_LOGI(LOG_TAG " 0:SENSOR_DISABLE\n 1:SENSOR_ENABLE\n 2:SENSOR_LPM\n 3:SENSOR_HPM\nEnter value:");
		(void)memset(enable, 0, sizeof(enable)/sizeof(enable[0]));
		(void)fgets(enable, sizeof(enable)/sizeof(enable[0]), stdin);
                state=(sensor_state)strtol(enable,&stopstring,10);
		SENSOR_LOGI(LOG_TAG "sensor id %d with contol value %d\n",sensor[i].sensor_id, state);
		ret = pClient->sensor_control(sensor[i].sensor_id, state);
		if(ret < 0) {
			SENSOR_LOGI(LOG_TAG "sensor control failed ret %d \n", ret);
			break;
		}
	   }
	}
	break;
     case 'p':
	if (pClient) {
		SENSOR_LOGI(LOG_TAG "start reading the sensor data\n");
		for(int i=0; i < sensor_count; i++) {
			ret = pClient->sensor_read_events(sensor[i].sensor_id, onSensorDataReadCb);
			if(ret < 0) {
				SENSOR_LOGE(LOG_TAG "sensor read events failed ret %d \n", ret);
				break;
			}
		}
	}
	break;
     case 't':
	if (pClient) {
		SENSOR_LOGI(LOG_TAG "read temperature\n");
		ret = pClient->sensor_read_temperature(onSensorTempReadCb);
		if(ret < 0) {
			SENSOR_LOGE(LOG_TAG "sensor read temperature failed ret %d \n", ret);
			break;
		}
	}
	break;
     case 'b':
	if (pClient) {
		SENSOR_LOGI(LOG_TAG "read buffer data\n");
		ret = pClient->sensor_read_buffer_data(1, onSensorBufferDataReadCb);
		if(ret < 0) {
			SENSOR_LOGE(LOG_TAG "sensor buffer read failed ret %d \n", ret);
			break;
		}
	}
	break;
     case 'f':
	if (pClient) {
		SENSOR_LOGI(LOG_TAG "flush/Delete the buffer data\n");
		ret = pClient->sensor_read_buffer_data(0, onSensorBufferDataReadCb);
		if(ret < 0) {
			SENSOR_LOGE(LOG_TAG "sensor buffer read failed ret %d \n", ret);
			break;
		}
		else
			SENSOR_LOGE(LOG_TAG "buffer deleted\n");
	}
	break;
     case 'm':
        if (pClient) {
                SENSOR_LOGI(LOG_TAG "get mlc case list supported\n");
                ret = pClient->sensor_request_mlc_case(&mlc_case, &mlc_case_count);
                if(ret < 0) {
                        SENSOR_LOGE(LOG_TAG "sensor get mlc case list failed ret %d \n", ret);
                        break;
                }
		PrintSensorMlcCaseList(mlc_case, mlc_case_count);
        }
	break;
     case 'e':
        if (pClient) {
                SENSOR_LOGI(LOG_TAG "enable/disable mlc case event\n");
		SENSOR_LOGI(LOG_TAG " 0:DISABLE\n 1:ENABLE\nEnter value:");
		(void)memset(enable, 0, sizeof(enable)/sizeof(enable[0]));
		(void)fgets(enable, sizeof(enable)/sizeof(enable[0]), stdin);
		mlc_enable=strtol(enable,&stopstring,10);
		SENSOR_LOGI(LOG_TAG "mlc_enable %d\n", mlc_enable);
		for(int i=0; i < mlc_case_count; i++) {
			ret = pClient->sensor_mlc_event_enable(mlc_case[i].name, mlc_enable, onSensorMLCEventCb, onMfifoDataReadCb);
			if(ret < 0) {
				SENSOR_LOGE(LOG_TAG "sensor enable/disable mlc case event failed ret %d \n", ret);
				break;
			}
		}
	}
	break;
     case 's':
        if (pClient) {
		SENSOR_LOGI(LOG_TAG "sensor self test\n");
		for(int i=0; i < sensor_count; i++) {
			SENSOR_LOGI(LOG_TAG "\n 0:Positive\n 1:Neagative\n 2:All\n Enter value:");
			(void)memset(enable, 0, sizeof(enable)/sizeof(enable[0]));
			(void)fgets(enable, sizeof(enable)/sizeof(enable[0]), stdin);
			selfTestType=(SelfTestType)strtol(enable,&stopstring,10);
			start_time = getTimestamp();
			SENSOR_LOGI(LOG_TAG "sensor[i].sensor_id %d\n", sensor[i].sensor_id);
			ret = pClient->sensor_self_test(sensor[i].sensor_id, selfTestType, request_id++, onSelfTestResultCallback);
			if(ret < 0) {
				SENSOR_LOGE(LOG_TAG "sensor self test request failed ret %d \n", ret);
                                break;
                        }
		}
        }
        break;
     case 'r':
        if (pClient) {
		SENSOR_LOGI(LOG_TAG "sensor set Euler angles\n");

		SENSOR_LOGI(LOG_TAG "\nEnter roll min 0 max 3600 value:");
		(void)memset(rm, 0, sizeof(rm)/sizeof(rm[0]));
		(void)fgets(rm, sizeof(rm)/sizeof(rm[0]), stdin);
		roll=strtol(rm,&stopstring,10);

		SENSOR_LOGI(LOG_TAG "\nEnter pitch min 0 max 3600 value:");
		(void)memset(rm, 0, sizeof(rm)/sizeof(rm[0]));
		(void)fgets(rm, sizeof(rm)/sizeof(rm[0]), stdin);
		pitch=strtol(rm,&stopstring,10);

		SENSOR_LOGI(LOG_TAG "\nEnter yaw min 0 max 3600 value:");
		(void)memset(rm, 0, sizeof(rm)/sizeof(rm[0]));
		(void)fgets(rm, sizeof(rm)/sizeof(rm[0]), stdin);
		yaw=strtol(rm,&stopstring,10);

		ret = pClient->sensor_update_rotation_matrix(roll, pitch, yaw);
		if(ret < 0) {
			SENSOR_LOGE(LOG_TAG "sensor set Euler angles failed ret %d \n", ret);
			break;
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
	SENSOR_LOGI(LOG_TAG "unknown command %s\n", buf);
	break;
    }
   }//while(1)

EXIT:
   if (pClient) {
	   delete pClient;
   }
   running = false;
   dataCondition.notify_all();
   worker.join();
   if (enable_temperature == 1) temperature.join();
   SENSOR_LOGI(LOG_TAG "Done\n");
   exit(0);
}
