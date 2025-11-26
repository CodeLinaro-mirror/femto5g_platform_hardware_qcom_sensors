/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 */
#include <iostream>
#include <string>
#ifndef _WIN32
#include <unistd.h>
#endif
#include <chrono>
#include <variant>
#include <future>
#include <CommonAPI/CommonAPI.hpp>
#include <v1/com/qualcomm/qti/sensor/SensorInterfaceProxy.hpp>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h>
#include <string.h>
#include <dlfcn.h>
#include <sys/types.h>
#include <gptp_helper.h>
#include <SensorCore.h>
#include <utils/SystemClock.h>

#define HAL_CONFIGURATION_FILE "hal_config"
#define HAL_CONFIGURATION_PATH "/vendor/etc"

#define RM_MIN 0
#define RM_MAX 3600

#define NSEC_IN_ONE_SEC       (1000000000ULL)   /* nanosec in a sec */
#define GPTP_IF_LIB_NAME      "libgptp.so"

using namespace v1::com::qualcomm::qti::sensor;
using namespace std;

int DEBUG_LEVEL = 0;
int ENABLE_10Hz = 0;

float SensorCore::rot[3][3] = {};
float SensorCore::location[3] = {};

int init_sensor_location_vector(float (&location)[3]);

static uint16_t roll;
static uint16_t pitch;
static uint16_t yaw;

struct SensorTrackingOption {
   int sensor_id;
   SensorInterfaceTypes::SensorStateT state;
};

void gptpUpdateNotification(struct gptp_update update);
const char * libName = GPTP_IF_LIB_NAME;
void *gPTPLibHandle = nullptr;
const static gPTPLibInterfaceReq  *gPTPReqIf = nullptr;

shared_ptr<SensorInterfaceProxy<>> myProxy;
CommonAPI::CallInfo info(1000);
SensorInterfaceTypes::SensorStateT state = SensorInterfaceTypes::SensorStateT::SENSOR_STATE_DISABLE;
SensorInterfaceTypes::SensorReturnT resp;
CommonAPI::CallStatus callStatus;
vector<SensorInterfaceTypes::SensorInfoT> mSensorList;
int32_t mSensorCount = 0;
uint32_t capSubscription;
uint32_t batchSubscription;
uint32_t imuDataSubscription;
uint32_t headingDataSubscription;
struct SensorTrackingOption mSensorTrackingOption [] = {{ACCEL_UNCALIBRATED_SENSOR_ID,state},
	                                                {GYRO_UNCALIBRATED_SENSOR_ID,state},
							{HEADING_SENSOR_ID,state}
						       };

int64_t SensorCore::nowBoottimeNanos() {
    timespec ts{};
    clock_gettime(CLOCK_BOOTTIME, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL +
           static_cast<int64_t>(ts.tv_nsec);
}

void parseSensorReturnT(SensorInterfaceTypes::SensorReturnT resp) {
   switch(resp) {
	   case SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_SUCCESS:
		   SENSOR_LOGI(SENSOR_TAG "SENSOR_RETURN_SUCCESS\n"); 
		   break;
	   case SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_CLIENT_REGISTER_FAILED:
		   SENSOR_LOGI(SENSOR_TAG "SENSOR_RETURN_ERROR_CLIENT_REGISTER_FAILED\n"); 
		   break;
	   case SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_INVALID_CLIENT:
		   SENSOR_LOGI(SENSOR_TAG "SENSOR_RETURN_ERROR_INVALID_CLIENT\n"); 
		   break;
	   case SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_INVALID_INPUT_PARAMETER:
		   SENSOR_LOGI(SENSOR_TAG "SENSOR_RETURN_ERROR_INVALID_INPUT_PARAMETER\n"); 
		   break;
	   case SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_CONTROL_FAILED:
		   SENSOR_LOGI(SENSOR_TAG "SENSOR_RETURN_ERROR_CONTROL_FAILED\n");
		   break;
	   case SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_CONFIG_FAILED:
		   SENSOR_LOGI(SENSOR_TAG "SENSOR_RETURN_ERROR_CONFIG_FAILED\n");
		   break;
	   case SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_NO_SENSORS_FOUND:
		   SENSOR_LOGI(SENSOR_TAG "SENSOR_RETURN_ERROR_NO_SENSORS_FOUND\n");
		   break;
	   case SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_TRACKING_FAILED:
		   SENSOR_LOGI(SENSOR_TAG "SENSOR_RETURN_ERROR_TRACKING_FAILED\n");
		   break;
	   case SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_UNKNOWN:
		   SENSOR_LOGI(SENSOR_TAG "SENSOR_RETURN_ERROR_UNKNOWN\n");
		   break;
	   default: 
		   SENSOR_LOGI(SENSOR_TAG "SENSOR_RETURN_ERROR_UNKNOWN\n");
		   break;
   }
   return;
}

int getSensorDebugLevel() {
   char *file_path_name = NULL;
   FILE *fd_config = NULL;
   int fsize = 0;
   int size;
   char buffer[BUFSIZ];
   char *line = NULL;
   int err = 0;

   file_path_name = (char *)calloc(strlen(HAL_CONFIGURATION_PATH) + strlen(HAL_CONFIGURATION_FILE) + 2, 1);
   if (!file_path_name) {
             err = -errno;
             return -ENOMEM;
   }

   fsize = strlen(HAL_CONFIGURATION_PATH) + strlen(HAL_CONFIGURATION_FILE);
   snprintf(file_path_name, fsize + 2, "%s/%s", HAL_CONFIGURATION_PATH, HAL_CONFIGURATION_FILE);
   fd_config = fopen(file_path_name, "r");
   if (fd_config == NULL) {
            err = -errno;
            goto fail;
   }
   while(fgets(buffer, sizeof(buffer), fd_config) != NULL) {
       for(int i = 0; i < strlen(buffer); i++) {
           if(strstr(buffer, "DEBUG_LEVEL=")) {
               line = strstr(buffer, "=");
               if(line != NULL){
                   sscanf(&line[1], "%d", &DEBUG_LEVEL);
                   SENSOR_LOGI(SENSOR_TAG "Info Sensor Debug Level is %d\n", DEBUG_LEVEL);
               }
               break;
           }
	   if(strstr(buffer, "ENABLE_10Hz=")) {
               line = strstr(buffer, "=");
	       if(line != NULL){
                   sscanf(&line[1], "%d", &ENABLE_10Hz);
		   SENSOR_LOGI(SENSOR_TAG "Info Sensor ENABLE_10Hz is %d\n", ENABLE_10Hz);
               }
	       break;
	   }
       }
   }
   fclose(fd_config);
fail:
     free(file_path_name);
     file_path_name = NULL;

     return err;
}

void DeInitHandles()
{
   CommonAPI::CallStatus callStatus = CommonAPI::CallStatus::UNKNOWN;
   SensorInterfaceTypes::SensorReturnT resp;

   myProxy->getSensorCapabilitiesEvent().unsubscribe(capSubscription);
   myProxy->getSensorConfigUpdateEvent().unsubscribe(batchSubscription);
   myProxy->getSensorImuDataReadEvent().unsubscribe(imuDataSubscription);
   myProxy->getSensorHeadingDataReadEvent().unsubscribe(headingDataSubscription);

   SENSOR_LOGI(SENSOR_TAG "==== DeRegister client ====>>\n");
   myProxy->DeRegisterSensorClientReq(callStatus, resp, &info);
   if (callStatus != CommonAPI::CallStatus::SUCCESS) {
	   SENSOR_LOGE(SENSOR_TAG "DeRegisterSensorClientReqReq() Remote call failed! callStatus %d\n", (int)callStatus);
   }
   else
	   parseSensorReturnT(resp);

   usleep(5000);
   if ((nullptr != gPTPReqIf) && (nullptr != gPTPReqIf->gptpDeinitIf)) {
	   gPTPReqIf->gptpDeinitIf();
	   gPTPReqIf = nullptr;
   }
   usleep(5000);
   if (nullptr != gPTPLibHandle)
	   dlclose(gPTPLibHandle);
   return;
}

void signalHandler(int signal)
{
   SENSOR_LOGI(SENSOR_TAG "signalHandler\n");
   DeInitHandles();
   exit(0);
   return;
}

void regSigHandler()
{
   struct sigaction mySigAction = {};

   mySigAction.sa_handler = signalHandler;
   sigemptyset(&mySigAction.sa_mask);
   sigaction(SIGHUP, &mySigAction, NULL);
   sigaction(SIGTERM, &mySigAction, NULL);
   sigaction(SIGINT, &mySigAction, NULL);
   sigaction(SIGPIPE, &mySigAction, NULL);
   return;
}

const static gPTPLibInterfaceEvent gPTPEvent = {
   gptpUpdateNotification,
};

void gptpUpdateNotification(struct gptp_update update)
{
   SENSOR_LOGI(SENSOR_TAG "GPTP Update Notification\n");
   return;
}

void loadGptpLibFile(void)
{
   char *errorDll;

   if ((gPTPLibHandle = dlopen(libName, RTLD_NOW)) != nullptr) {
       SENSOR_LOGI(SENSOR_TAG "%s is present\n", libName);
       get_gPTPLib_if_t getter = (get_gPTPLib_if_t)dlsym(gPTPLibHandle, "get_gPTPLib_if");

       if ((errorDll = dlerror()) != nullptr) {
	       SENSOR_LOGE(SENSOR_TAG "dlsym for %s get_gPTPLib_if failed, error = %s\n", libName, errorDll);
	       getter = nullptr;
       }

       if (getter != nullptr) {
	       gPTPReqIf = (getter)(&gPTPEvent);
	       if (gPTPReqIf != nullptr) {
			   return;
	       } else {
		       SENSOR_LOGE(SENSOR_TAG "%s lib provided Command Interface as NULL\n", libName);
	       }
       }
   } else {
	   errorDll = dlerror();
	   SENSOR_LOGE(SENSOR_TAG "dlopen for %s failed, handle %p error: %s", libName, gPTPLibHandle,
			   ((nullptr != errorDll) ? errorDll : "No Error"));
   }
   return;
}

static void onCapabilitiesCb(SensorInterfaceTypes::SensorServiceStateMaskT mask) {
   switch (mask) {
    case SensorInterfaceTypes::SensorServiceStateMaskT::SENSOR_SERVICE_STATE_MASK_READY:
	  SENSOR_LOGI(SENSOR_TAG "Sensor Hal daemon is Ready to commnunicate\n");
	  break;
   }
   return;
}

static void dump_live_event(SensorInterfaceTypes::SensorImuEventT *e)
{
  static int64_t acc_ts = 0;
  static int64_t gyro_ts = 0;
  static int64_t head_ts = 0;
  static int AccCount = 0, GyroCount = 0;
  uint64_t currPTPtime = 0;

  bool retPtp = false;
  if ((nullptr != gPTPReqIf) && (nullptr != gPTPReqIf->gptpGetCurPtpTimeIf)) {
	  retPtp = gPTPReqIf->gptpGetCurPtpTimeIf(&currPTPtime);
  }
  if((e->getType() == SensorInterfaceTypes::SensorTypeT::SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED ) ||
		  (e->getType() == SensorInterfaceTypes::SensorTypeT::SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED ) ) {
     const SensorInterfaceTypes::SensorUncalibratedEventT & data = e->getData();
     SENSOR_LOGD(SENSOR_TAG "Accel Live event:%d xyz_raw<%f %f %f> xyz_bias<%f %f %f> timestamp<boot,gptp> %lld %lld latency(ms)=%lld odr(ms)=%lld\n",
			  AccCount++,
			  data.getXUncalib(), data.getYUncalib(), data.getZUncalib(),
			  data.getXBias(), data.getYBias(), data.getZBias(),
			  e->getTimestamp(), e->getGptpTimestamp(),
			  (currPTPtime - e->getGptpTimestamp())/1000000,
			  (e->getGptpTimestamp()- acc_ts)/1000000);
     acc_ts = e->getGptpTimestamp();
  }
  else if((e->getType() == SensorInterfaceTypes::SensorTypeT::SENSOR_TYPE_GYROSCOPE_UNCALIBRATED ) ||
		  (e->getType() == SensorInterfaceTypes::SensorTypeT::SENSOR_TYPE_GYROSCOPE ) ) {
      const SensorInterfaceTypes::SensorUncalibratedEventT & data = e->getData();
      SENSOR_LOGD(SENSOR_TAG "Gyro Live event:%d xyz_raw<%f %f %f> xyz_bias<%f %f %f> timestamp<boot,gptp> %lld %lld latency(ms)=%lld odr(ms)=%lld\n",
			  GyroCount++,
			  data.getXUncalib(), data.getYUncalib(), data.getZUncalib(),
			  data.getXBias(), data.getYBias(), data.getZBias(),
			  e->getTimestamp(),
			  e->getGptpTimestamp(),
			  (currPTPtime - e->getGptpTimestamp())/1000000,
			  (e->getGptpTimestamp()-gyro_ts)/1000000);
      gyro_ts = e->getGptpTimestamp();
  }
  else {
	  printf( "Sensor Live unknown sensor_id events %d\n", e->getType());
  }
  return;
}

static void  onSensorImuDataReadCb(vector<SensorInterfaceTypes::SensorImuEventT> events, uint32_t count)
{
  int i =0;
  static uint64_t ts_prv_acc = 0, ts_prv_gyro = 0 , ts_cur = 0;
  static uint64_t acc_sensor_ts = 0, gyro_sensor_ts = 0;
  bool retPtp = false;

  int sensor_id = events[0].getSensorId();

  if ((nullptr != gPTPReqIf) && (nullptr != gPTPReqIf->gptpGetCurPtpTimeIf)) {
	  retPtp = gPTPReqIf->gptpGetCurPtpTimeIf(&ts_cur);
  }
  if (sensor_id == ACCEL_UNCALIBRATED_SENSOR_ID) {
	  acc_sensor_ts = events[count-1].getGptpTimestamp();
	  SENSOR_LOGD(SENSOR_TAG "Sensor ACC Live: sensor_id %d: count %d batch_delta(ms)=%lld cur_gptp_ts %lld latency(ms)=%lld\n",
			  sensor_id, count, (ts_cur - ts_prv_acc)/1000000, ts_cur, (ts_cur - acc_sensor_ts)/1000000);
	  ts_prv_acc = ts_cur;
  }
  if (sensor_id == GYRO_UNCALIBRATED_SENSOR_ID ) {
	  gyro_sensor_ts = events[count-1].getGptpTimestamp();
	  SENSOR_LOGD(SENSOR_TAG "Sensor GYRO Live: sensor_id %d: count %d batch_delta(ms)=%lld cur_gptp_ts %lld latency(ms)=%lld\n",
			  sensor_id, count, (ts_cur - ts_prv_gyro)/1000000, ts_cur, (ts_cur - gyro_sensor_ts)/1000000);
	  ts_prv_gyro = ts_cur;
  }
  for ( i = 0; i < count ; i++){
	  dump_live_event(&events[i]);
  }
  return;
}

static void  onSensorHeadingDataReadCb(vector<SensorInterfaceTypes::SensorHeadEventT> events, uint32_t count)
{
  int i =0;
  static uint64_t ts_prv_head = 0, ts_cur = 0;
  static uint64_t head_sensor_ts = 0;
  static int64_t head_ts = 0;
  static int HeadCount = 0;
  bool retPtp = false;
  if ((nullptr != gPTPReqIf) && (nullptr != gPTPReqIf->gptpGetCurPtpTimeIf)) {
	  retPtp = gPTPReqIf->gptpGetCurPtpTimeIf(&ts_cur);
  }

  int sensor_id = events[0].getSensorId();

  if (sensor_id == HEADING_SENSOR_ID) {
	  head_sensor_ts = events[count-1].getGptpTimestamp();
	  SENSOR_LOGD(SENSOR_TAG "Sensor HEAD Live: sensor_id %d: count %d batch_delta(ms)=%lld cur_gptp_ts %lld latency(ms)=%lld\n",
			  sensor_id, count, (ts_cur - ts_prv_head)/1000000, ts_cur, (ts_cur - head_sensor_ts)/1000000);
	  ts_prv_head = ts_cur;
  }
  for ( i = 0; i < count ; i++) {
	  if(events[i].getType() == SensorInterfaceTypes::SensorTypeT::SENSOR_TYPE_HEADING ) {
	     const SensorInterfaceTypes::SensorHeadingEventT & head = events[i].getData();
	     SENSOR_LOGD(SENSOR_TAG "Head Live event:%d heading and accuracy<%f %f> timestamp<boot,gptp> %lld %lld latency(ms)=%lld odr(ms)=%lld\n",
				  HeadCount++,
				  head.getHeading(), head.getAccuracy(),
				  events[i].getTimestamp(), events[i].getGptpTimestamp(),
				  (ts_cur - events[i].getGptpTimestamp())/1000000,
				  (events[i].getGptpTimestamp()- head_ts)/1000000);
	     head_ts = events[i].getGptpTimestamp();
	  }
  }
}

static void PrintSensorList(vector<SensorInterfaceTypes::SensorInfoT> sensor, int32_t sensor_count)
{
  for (int i=0 ; i< sensor_count ; i++) {
     vector<float>odr = sensor[i].getOdr();

     SENSOR_LOGI(SENSOR_TAG "Name: %s\n", sensor[i].getName().c_str());
     SENSOR_LOGI(SENSOR_TAG "\tvendor: %s\n", sensor[i].getVendor().c_str());
     SENSOR_LOGI(SENSOR_TAG "\tversion: %d\n",sensor[i].getSensorVersion());
     SENSOR_LOGI(SENSOR_TAG "\tresolution: %f\n",sensor[i].getResolution());
     SENSOR_LOGI(SENSOR_TAG "\tmaxRange %f\n",sensor[i].getMaxRange());
     SENSOR_LOGI(SENSOR_TAG "\tsensor_id: %d\n",sensor[i].getSensorId());
     SENSOR_LOGI(SENSOR_TAG "\ttype: %d\n",sensor[i].getType());
     SENSOR_LOGI(SENSOR_TAG "\tmaxSamplingRate: %f\n",sensor[i].getMaxSamplingRate());
     SENSOR_LOGI(SENSOR_TAG "\tminBatchCount: %d\n",sensor[i].getMinBatchCount());
     SENSOR_LOGI(SENSOR_TAG "\tmaxBatchCount: %d\n",sensor[i].getMaxBatchCount());
     SENSOR_LOGI(SENSOR_TAG "\todr rate: %fHZ %fHZ %fHZ %fHZ %fHZ %fHZ\n",
		     odr[0], odr[1],odr[2],odr[3],odr[4],odr[5]);
    }
    return;
}

int init_sensor_location_vector(float (&location) [3])
{
  location[0] = 0.0f;
  location[1] = 0.0f;
  location[2] = 0.0f;

  return 0;
}

// This API is called to initialise rotational matrix
int init_sensor_rotation_matrix(float (*rot) [3])
{
  rot[0][0] = 1;
  rot[0][1] = 0;
  rot[0][2] = 0;

  rot[1][0] = 0;
  rot[1][1] = 1;
  rot[1][2] = 0;

  rot[2][0] = 0;
  rot[2][1] = 0;
  rot[2][2] = 1;

  return 0;
}

// This API is called to calculate rotational matrix
int calculate_sensor_rotation_matrix(uint16_t rolld, uint16_t pitchd, uint16_t yawd, float (*rot) [3])
{
  float roll = (rolld / 10.0f) * M_PI / 180.0f;
  float pitch = (pitchd / 10.0f) * M_PI / 180.0f;
  float yaw = (yawd / 10.0f) * M_PI / 180.0f;

  rot[0][0] = cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) * sin(roll);
  rot[0][1] = -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll);
  rot[0][2] = cos(pitch) * sin(roll);

  rot[1][0] = sin(yaw) * cos(pitch);
  rot[1][1] = cos(yaw) * cos(pitch);
  rot[1][2] = -sin(pitch);

  rot[2][0] = -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll);
  rot[2][1] = sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll);
  rot[2][2] = cos(pitch) * cos(roll);

  return 0;
}

int read_sensor_placement_matrix(float (&location) [3])
{
  char *file_path_name = NULL;
  FILE *fd_config = NULL;
  int fsize = 0;
  int size;
  char buffer[BUFSIZ];
  char *line = NULL;
  int err = 0;

  file_path_name = (char *)calloc(strlen(HAL_CONFIGURATION_PATH) + strlen(HAL_CONFIGURATION_FILE) + 2, 1);
  if (!file_path_name) {
            err = -errno;
            SENSOR_LOGE(SENSOR_TAG "Unable to allocate memory (errno %d)\n", err);
            return -ENOMEM;
  }

  fsize = strlen(HAL_CONFIGURATION_PATH) + strlen(HAL_CONFIGURATION_FILE);
  snprintf(file_path_name, fsize + 2, "%s/%s", HAL_CONFIGURATION_PATH, HAL_CONFIGURATION_FILE);
  SENSOR_LOGI(SENSOR_TAG "Hal Config file_path_name %s\n", file_path_name);
  fd_config = fopen(file_path_name, "r");
  if (fd_config == NULL) {
      err = -errno;
      SENSOR_LOGE(SENSOR_TAG "Sensor Filed to open %s (errno %d)\n",
                      file_path_name, err);
      goto fail;
  }

  while(fgets(buffer, sizeof(buffer), fd_config) != NULL) {
      if(strstr(buffer, "imu_sensor_placement = ")) {
          line = strstr(buffer, "[");
          if(line != NULL){
              size = sscanf(&line[1], "%f,%f,%f", &location[0], &location[1], &location[2]);
              SENSOR_LOGI(SENSOR_TAG "Read hal config file successful, location[0] %f, location[1] %f, location[2] %f\n", location[0], location[1], location[2]);
          }
          break;
      }
  }
  fclose(fd_config);
fail:
  free(file_path_name);
  file_path_name = NULL;

  return err;
}

// This API is called to read rotational matrix
int read_sensor_rotation_matrix(uint16_t *roll, uint16_t *pitch, uint16_t *yaw)
{
  char *file_path_name = NULL;
  FILE *fd_config = NULL;
  int fsize = 0;
  int size;
  char buffer[BUFSIZ];
  char *line = NULL;
  int err = 0;

  file_path_name = (char *)calloc(strlen(HAL_CONFIGURATION_PATH) + strlen(HAL_CONFIGURATION_FILE) + 2, 1);
  if (!file_path_name) {
            err = -errno;
            SENSOR_LOGE(SENSOR_TAG "Unable to allocate memory (errno %d)\n", err);
            return -ENOMEM;
  }

  fsize = strlen(HAL_CONFIGURATION_PATH) + strlen(HAL_CONFIGURATION_FILE);
  snprintf(file_path_name, fsize + 2, "%s/%s", HAL_CONFIGURATION_PATH, HAL_CONFIGURATION_FILE);
  SENSOR_LOGI(SENSOR_TAG "Hal Config file_path_name %s\n", file_path_name);
  fd_config = fopen(file_path_name, "r");
  if (fd_config == NULL) {
      err = -errno;
      SENSOR_LOGE(SENSOR_TAG "Sensor Filed to open %s (errno %d)\n",
                      file_path_name, err);
      goto fail;
  }

  while(fgets(buffer, sizeof(buffer), fd_config) != NULL) {
      if(strstr(buffer, "imu_sensor_euler_angles = ")) {
          line = strstr(buffer, "[");
          if(line != NULL){
              size = sscanf(&line[1], "%d,%d,%d", roll, pitch, yaw);
              SENSOR_LOGI(SENSOR_TAG "Read hal config file successful, roll %d, pitch %d, yaw %d\n", roll, pitch, yaw);
          }
          break;
      }
  }
  fclose(fd_config);
fail:
  free(file_path_name);
  file_path_name = NULL;

  return err;
}

void SensorCore::SensorCore_Init() {

    regSigHandler();

    init_sensor_rotation_matrix(rot);
    init_sensor_location_vector(location);
    if ( !read_sensor_rotation_matrix(&roll, &pitch, &yaw) ) {
        if(yaw < RM_MIN || pitch < RM_MIN || roll < RM_MIN || yaw > RM_MAX || pitch > RM_MAX || roll > RM_MAX){
            SENSOR_LOGE(SENSOR_TAG "Error: Euler Angles Invalid Range\n");
        }
        else {
            SENSOR_LOGI(SENSOR_TAG "Sensor Euler angles <roll %d, pitch %d, yaw %d>\n", roll, pitch, yaw);
            calculate_sensor_rotation_matrix(roll, pitch, yaw, rot);
            SENSOR_LOGI(SENSOR_TAG "Sensor rotation matrix: \t%5.2f %5.2f %5.2f\t%5.2f %5.2f %5.2f\t%5.2f %5.2f %5.2f\n",
                            rot[0][0], rot[0][1], rot[0][2],
                            rot[1][0], rot[1][1], rot[1][2],
                            rot[2][0], rot[2][1], rot[2][2]);
        }
    }

    read_sensor_placement_matrix(location);

    /* GPTP */
    loadGptpLibFile();
    GptpInitialized = false;
    if ((nullptr != gPTPReqIf) && (nullptr != gPTPReqIf->gptpInitIf)) {
	    SENSOR_LOGI(SENSOR_TAG "GPTP init\n");
	    bool ret = false;
	    ret = gPTPReqIf->gptpInitIf();
	    SENSOR_LOGI(SENSOR_TAG "GPTP init ret %d\n", ret);
	    if(ret)
		    GptpInitialized = true;
    }

    CommonAPI::Runtime::setProperty("LogContext", "SensorInterface");
    CommonAPI::Runtime::setProperty("LogApplication", "SensorInterface");
    CommonAPI::Runtime::setProperty("LibraryBase", "SensorInterface");

    shared_ptr < CommonAPI::Runtime > runtime = CommonAPI::Runtime::get();

    string domain = "local";
    string instance = "com.qualcomm.qti.sensor.SensorInterface";
    string connection = "sensor-fidl-client";

    myProxy=runtime->buildProxy<SensorInterfaceProxy>(domain,instance,connection);

    SENSOR_LOGI(SENSOR_TAG "Checking Sensor Service availability !!\n");
    myProxy->getProxyStatusEvent().subscribe([&] (const CommonAPI::AvailabilityStatus status) {
       switch (status) {
       case CommonAPI::AvailabilityStatus::UNKNOWN:
       		SENSOR_LOGI(SENSOR_TAG "Sensor Service Unkown\n");
		SensorServiceAvailable = false;
       break;
       case CommonAPI::AvailabilityStatus::NOT_AVAILABLE:
       		SENSOR_LOGI(SENSOR_TAG "Sensor Service NOT_AVAILABLE\n");
		SensorServiceAvailable = false;
       break;
       case CommonAPI::AvailabilityStatus::AVAILABLE:
       		SENSOR_LOGI(SENSOR_TAG "Sensor Service AVAILABLE\n");
		SensorServiceAvailable = true;

		SENSOR_LOGI(SENSOR_TAG "==== Register new client ====>>\n");
		usleep(10*1000);
		myProxy->RegisterSensorClientReq(callStatus, resp, &info);
		if (callStatus != CommonAPI::CallStatus::SUCCESS) {
		  SENSOR_LOGE(SENSOR_TAG "RegisterSensorClientReq() Remote call failed! callStatus %d\n", (int)callStatus);
		  return;
		}
		parseSensorReturnT(resp);

		SENSOR_LOGI(SENSOR_TAG "==== Get Sensor List supported ====>>\n");
		usleep(10*1000);
		myProxy->GetSensorListReq(callStatus, mSensorList, mSensorCount, &info);
		if (callStatus != CommonAPI::CallStatus::SUCCESS) {
			SENSOR_LOGE(SENSOR_TAG "sensor get list failed ret %d \n", (int)callStatus);
			return;
		}
		PrintSensorList(mSensorList, mSensorCount);

		usleep(10*1000);
		for (int i=0; i < mSensorCount; i++) {
			if (mSensorTrackingOption[i].state == SensorInterfaceTypes::SensorStateT::SENSOR_STATE_ENABLE &&
					 mSensorTrackingOption[i].sensor_id == mSensorList[i].getSensorId()) {
				SENSOR_LOGI(SENSOR_TAG "Reconfiguring Enabled Sensor %d on sensor service restart\n", mSensorList[i].getSensorId());
				myProxy->SensorConfigReq(mSensorList[i].getSensorId(), mSensorList[i].getMaxSamplingRate(),
						mSensorList[i].getMinBatchCount(), callStatus, resp, &info);
				myProxy->SensorControlReq(mSensorList[i].getSensorId(), state, callStatus, resp, &info);
			}
		}
		break;
       }
    });

    capSubscription = myProxy->getSensorCapabilitiesEvent().subscribe(
       [&](const ::v1::com::qualcomm::qti::sensor::SensorInterfaceTypes::SensorServiceStateMaskT &mask) {
       SENSOR_LOGI(SENSOR_TAG "<<--Received SensorCapabilitiesMask mask %d\n", static_cast<int>(mask));
       onCapabilitiesCb(mask);
    });

    batchSubscription = myProxy->getSensorConfigUpdateEvent().subscribe(
       [&](int32_t sensor_id, float SamplingRate, int32_t BatchCount) {
       SENSOR_LOGI(SENSOR_TAG "<<--Received SensorConfigUpdateCb id: %d SamplingRate : %f BatchCount: %d\n", sensor_id, SamplingRate, BatchCount);
    });

    imuDataSubscription = myProxy->getSensorImuDataReadEvent().subscribe(
       [&](vector< ::v1::com::qualcomm::qti::sensor::SensorInterfaceTypes::SensorImuEventT > events, uint32_t count) {
       onSensorImuDataReadCb(events, count);
       vector<SensorCoreData> idlSensorEventsData;
       SensorCoreData idlSensorEvents = {};
       for (int i=0 ;i <count; i++){
	    idlSensorEvents = {}; // Safe reset
	    const auto & data = events[i].getData();
            float a = data.getXUncalib();
            float b = data.getYUncalib();
            float c = data.getZUncalib();
            float x_uncalib = rot[0][0] * a + rot[1][0] * b + rot[2][0] * c;
            float y_uncalib = rot[0][1] * a + rot[1][1] * b + rot[2][1] * c;
            float z_uncalib = rot[0][2] * a + rot[1][2] * b + rot[2][2] * c;
	    idlSensorEvents.sensorId = events[i].getSensorId();
	    idlSensorEvents.Type = events[i].getType();
	    idlSensorEvents.timestamp = events[i].getTimestamp();
	    idlSensorEvents.gptptimestamp = events[i].getGptpTimestamp();
            idlSensorEvents.xyz = {x_uncalib, y_uncalib, z_uncalib,
		    data.getXBias(), data.getYBias(), data.getZBias()};
	    idlSensorEventsData.push_back(idlSensorEvents);
	 }
       onNewSensorsData(idlSensorEventsData);
    });

    headingDataSubscription = myProxy->getSensorHeadingDataReadEvent().subscribe(
       [&](vector< ::v1::com::qualcomm::qti::sensor::SensorInterfaceTypes::SensorHeadEventT > events, uint32_t count) {
       onSensorHeadingDataReadCb(events, count);
       vector<SensorCoreData> idlSensorEventsData;
       SensorCoreData idlSensorEvents = {};
       for (int i=0 ;i <count; i++){
         if(events[i].getType() == SensorInterfaceTypes::SensorTypeT::SENSOR_TYPE_HEADING ) {
            memset(&idlSensorEvents, 0, sizeof(idlSensorEvents));
	    const SensorInterfaceTypes::SensorHeadingEventT & data = events[i].getData();
	    idlSensorEvents.sensorId = events[i].getSensorId();
	    idlSensorEvents.Type = events[i].getType();
	    idlSensorEvents.timestamp = events[i].getTimestamp();
	    idlSensorEvents.gptptimestamp = events[i].getGptpTimestamp();
	    idlSensorEvents.xyz = {data.getHeading(), data.getAccuracy()};
	    idlSensorEventsData.push_back(idlSensorEvents);
	 }
       }
       onNewSensorsData(idlSensorEventsData);
    });
    return;
}

void SensorCore::SensorCore_getSensorList(std::vector<SensorCoreList> &sensorVector, int32_t *sensorcount) {
    static bool sensorlistready = false;
    SENSOR_LOGI(SENSOR_TAG "==== Calling GetSensorListReq ====>> sensorlistready %d \n", sensorlistready);
    if(sensorlistready == false) {
       myProxy->GetSensorListReq(callStatus, mSensorList, mSensorCount, &info);
       if (callStatus != CommonAPI::CallStatus::SUCCESS) {
	       SENSOR_LOGE(SENSOR_TAG "sensor get list failed ret %d \n", (int)callStatus);
	       return;
       }
       PrintSensorList(mSensorList, mSensorCount);
       *sensorcount = mSensorCount;
       for (int i=0 ; i < mSensorCount ; i++) {
	       vector<float>odr = mSensorList[i].getOdr();
	       setSensorCoreList(mSensorList[i].getName(),
			         mSensorList[i].getVendor(),
				 mSensorList[i].getSensorVersion(),
			         mSensorList[i].getResolution(),
				 mSensorList[i].getMaxRange(),
				 mSensorList[i].getSensorId(),
				 mSensorList[i].getType(),
				 mSensorList[i].getMaxSamplingRate(),
				 mSensorList[i].getMinBatchCount(),
			         mSensorList[i].getMaxBatchCount(),
				 odr);
       }
       sensorlistready = true;
    }
    sensorVector = getSensorCoreList();
    *sensorcount = mSensorCount;
    return;
}

void SensorCore::SensorCore_acitvateSensor(int32_t in_sensorHandle, bool in_enabled) {
    SENSOR_LOGI(SENSOR_TAG "==== Calling SensorControlReq ====>> sensor_id: %d in_enabled: %d \n", in_sensorHandle, in_enabled);

    //Enable uncalibrated sensor when calibrated request received
    if (in_sensorHandle == ACCEL_CALIBRATED_SENSOR_ID)
	    in_sensorHandle = ACCEL_UNCALIBRATED_SENSOR_ID;
    if (in_sensorHandle == GYRO_CALIBRATED_SENSOR_ID)
	    in_sensorHandle = GYRO_UNCALIBRATED_SENSOR_ID;

    if (in_enabled == true)
        state = SensorInterfaceTypes::SensorStateT::SENSOR_STATE_ENABLE;
    else 
	state = SensorInterfaceTypes::SensorStateT::SENSOR_STATE_DISABLE;
 
    if (mSensorCount != 0) {
       for (int i=0; i < mSensorCount; i++) {
	  if (mSensorList[i].getSensorId() == in_sensorHandle) {
		  SENSOR_LOGI(SENSOR_TAG "Sensor %d activate set to %d\n", in_sensorHandle, in_enabled);
		  myProxy->SensorControlReq(mSensorList[i].getSensorId(), state, callStatus, resp, &info);
		  if (callStatus != CommonAPI::CallStatus::SUCCESS) {
			  SENSOR_LOGE(SENSOR_TAG "sensor control failed sensor[i].sensor_id %d ret %d \n", in_sensorHandle, (int)callStatus);
			  return;
		  }
		  parseSensorReturnT(resp);
		  //store state of enabled sensors to reconfigure on SHD restart.
		  if(resp == SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_SUCCESS) {
			  mSensorTrackingOption[i].sensor_id = mSensorList[i].getSensorId();
			  mSensorTrackingOption[i].state = state;
		  }
	  }
       }
    }
    
    else {
	    SENSOR_LOGE(SENSOR_TAG "Sensor service not yet available to activate sensor\n");
	    for (auto& option : mSensorTrackingOption) {
		    if(option.sensor_id == in_sensorHandle) {
			    option.state =  state;
			    break;
		    }
	    }
    }
    return;
}

void SensorCore::SensorCore_configSensor(int32_t in_sensorHandle, int64_t in_samplingPeriodNs,int64_t in_maxReportLatencyNs) {
    SENSOR_LOGI(SENSOR_TAG  "==== Calling SensorConfigReq ====>> sensor_id: %d in_samplingPeriodNs %lld in_maxReportLatencyNs %lld\n", in_sensorHandle, in_samplingPeriodNs, in_maxReportLatencyNs);

    //Configure uncalibrated sensor when calibrated request received
    if (in_sensorHandle == ACCEL_CALIBRATED_SENSOR_ID)
	    in_sensorHandle = ACCEL_UNCALIBRATED_SENSOR_ID;
    if (in_sensorHandle == GYRO_CALIBRATED_SENSOR_ID)
	    in_sensorHandle = GYRO_UNCALIBRATED_SENSOR_ID;

    if (mSensorCount != 0) {
       for (int i=0; i < mSensorCount; i++) {
	  if (mSensorList[i].getSensorId() == in_sensorHandle) {
		  SENSOR_LOGI(SENSOR_TAG "Sensor %d configure for sampling rate %f and batch count %d\n", mSensorList[i].getSensorId(), mSensorList[i].getMaxSamplingRate(), mSensorList[i].getMinBatchCount());
		  int batch_count = mSensorList[i].getMaxSamplingRate() * 0.1;
		  SENSOR_LOGI(SENSOR_TAG "mSensorList[i].getMaxSamplingRate() %f, batch_count %d ENABLE_10Hz %d \n", mSensorList[i].getMaxSamplingRate(), batch_count, ENABLE_10Hz);
		  if(ENABLE_10Hz == 1) {
			  SENSOR_LOGI(SENSOR_TAG "Enable 10Hz Batching %d\n", ENABLE_10Hz);
			  myProxy->SensorConfigReq(mSensorList[i].getSensorId(), mSensorList[i].getMaxSamplingRate(),
					                                    batch_count, callStatus, resp, &info);
		  }
		  else {
			SENSOR_LOGI(SENSOR_TAG "Disabled 10Hz Batching %d\n", ENABLE_10Hz);
		  	myProxy->SensorConfigReq(mSensorList[i].getSensorId(), mSensorList[i].getMaxSamplingRate(),
					  mSensorList[i].getMinBatchCount(), callStatus, resp, &info);
		  }
		  if (callStatus != CommonAPI::CallStatus::SUCCESS) {
			  SENSOR_LOGE(SENSOR_TAG "sensor config  failed sensor_id %d ret %d \n", in_sensorHandle, (int)callStatus);
			  return;
		  }
		  parseSensorReturnT(resp);
	  }
       }
    }
    else {
	    SENSOR_LOGE(SENSOR_TAG "Sensor service not yet available to configure sensor\n");
    }
    return;
}

uint64_t SensorCore::SensorCore_getBootTimeFromPtpTime(uint64_t ptp_time_ns)
{
   uint64_t boot_time_ns;
   if (!GptpInitialized && gPTPReqIf->gptpInitIf()) {
	   SENSOR_LOGI(SENSOR_TAG "GPTP init success \n");
	   GptpInitialized = true;
   }
   gPTPReqIf->gptpGetBootTimeFromPtpTimeIf(&boot_time_ns, ptp_time_ns);
   SENSOR_LOGD(SENSOR_TAG "gptpGetBootTimeFromPtpTimeIf Sensor gptp ts %lld boot time %lld now_ns %lld\n", ptp_time_ns, boot_time_ns, android::elapsedRealtimeNano());
   return boot_time_ns;
}

void SensorCore_Deinit() {
    SENSOR_LOGI(SENSOR_TAG  "==== SensorCore_Deinit ====>>\n");
    DeInitHandles();
    return;
}
