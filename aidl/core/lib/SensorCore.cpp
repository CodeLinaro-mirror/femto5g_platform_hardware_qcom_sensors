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
#include <v0/com/qualcomm/qti/sensor/SensorInterfaceProxy.hpp>
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

#define NSEC_IN_ONE_SEC       (1000000000ULL)   /* nanosec in a sec */
#define GPTP_IF_LIB_NAME      "libgptp.so"

using namespace v0::com::qualcomm::qti::sensor;
using namespace std;

void gptpUpdateNotification(struct gptp_update update);
const char * libName = GPTP_IF_LIB_NAME;
void *gPTPLibHandle = nullptr;
const static gPTPLibInterfaceReq  *gPTPReqIf = nullptr;

shared_ptr<SensorInterfaceProxy<>> myProxy;
CommonAPI::CallInfo info(1000);
SensorInterface::SensorState state = SensorInterface::SensorState::SENSOR_DISABLE;
SensorInterface::SensorResponse resp;
CommonAPI::CallStatus callStatus;
vector<SensorInterface::SensorList> sensor;
int32_t sensor_count = 0;
uint32_t capSubscription;
uint32_t batchSubscription;
uint32_t dataSubscription;

void parseSensorResponse(SensorInterface::SensorResponse resp) {
   switch(resp) {
	   case SensorInterface::SensorResponse::SENSOR_RESPONSE_SUCCESS:
		   SENSOR_LOGI(SENSOR_TAG "SENSOR_RESPONSE_SUCCESS\n"); 
		   break;
	   case SensorInterface::SensorResponse::SENSOR_ERROR_CLIENT_REGISTER_FAILED:
		   SENSOR_LOGI(SENSOR_TAG "SENSOR_ERROR_CLIENT_REGISTER_FAILED\n"); 
		   break;
	   case SensorInterface::SensorResponse::SENSOR_ERROR_INVALID_CLIENT:
		   SENSOR_LOGI(SENSOR_TAG "SENSOR_ERROR_INVALID_CLIENT\n"); 
		   break;
	   case SensorInterface::SensorResponse::SENSOR_ERROR_INVALID_INPUT_PARAMETER:
		   SENSOR_LOGI(SENSOR_TAG "SENSOR_ERROR_INVALID_INPUT_PARAMETER\n"); 
		   break;
	   case SensorInterface::SensorResponse::SENSOR_ERROR_CONTROL_FAILED:
		   SENSOR_LOGI(SENSOR_TAG "SENSOR_ERROR_CONTROL_FAILED\n");
		   break;
	   case SensorInterface::SensorResponse::SENSOR_ERROR_CONFIG_FAILED:
		   SENSOR_LOGI(SENSOR_TAG "SENSOR_ERROR_CONFIG_FAILED\n");
		   break;
	   case SensorInterface::SensorResponse::SENSOR_ERROR_NO_SENSORS_FOUND:
		   SENSOR_LOGI(SENSOR_TAG "SENSOR_ERROR_NO_SENSORS_FOUND\n");
		   break;
	   case SensorInterface::SensorResponse::SENSOR_ERROR_TRACKING_FAILED:
		   SENSOR_LOGI(SENSOR_TAG "SENSOR_ERROR_TRACKING_FAILED\n");
		   break;
	   case SensorInterface::SensorResponse::SENSOR_ERROR_UNKNOWN:
		   SENSOR_LOGI(SENSOR_TAG "SENSOR_ERROR_UNKNOWN\n");
		   break;
	   default: 
		   SENSOR_LOGI(SENSOR_TAG "SENSOR_ERROR_UNKNOWN\n");
		   break;
   }
   return;
}

void DeInitHandles()
{
   CommonAPI::CallStatus callStatus;
   SensorInterface::SensorResponse resp;

   myProxy->getSensorCapabilitiesEvent().unsubscribe(capSubscription);
   myProxy->getSensorConfigUpdateEvent().unsubscribe(batchSubscription);
   myProxy->getSensorDataReadEvent().unsubscribe(dataSubscription);

   SENSOR_LOGI(SENSOR_TAG "==== DeRegister client ====>>\n");
   myProxy->DeRegisterSensorClient(callStatus, resp, &info);
   if (callStatus != CommonAPI::CallStatus::SUCCESS) {
	   SENSOR_LOGE(SENSOR_TAG "DeRegisterSensorClient() Remote call failed! callStatus %d\n", (int)callStatus);
   }
   else
	   parseSensorResponse(resp);

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

static void onCapabilitiesCb(SensorInterface::SensorCapabilitiesMask mask) {
   switch (mask) {
    case SensorInterface::SensorCapabilitiesMask::SHD_READY:
	  SENSOR_LOGI(SENSOR_TAG "Sensor Hal daemon is Ready to commnunicate\n");
	  break;
   }
   return;
}

static void dump_live_event(SensorInterface::SensorEvent *e)
{
  static int64_t acc_ts = 0;
  static int64_t gyro_ts = 0;
  static int64_t head_ts = 0;
  static int AccCount = 0, GyroCount = 0, HeadCount = 0;
  uint64_t currPTPtime = 0;

  bool retPtp = false;
  if ((nullptr != gPTPReqIf) && (nullptr != gPTPReqIf->gptpGetCurPtpTimeIf)) {
	  retPtp = gPTPReqIf->gptpGetCurPtpTimeIf(&currPTPtime);
  }

  if((e->getType() == SensorInterface::Sensortype::SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED ) || 
		  (e->getType() == SensorInterface::Sensortype::SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED ) ) {
	  const SensorInterface::SensorUncalibratedEvent & data = e->getData().get<SensorInterface::SensorUncalibratedEvent>();
	  SENSOR_LOGV(SENSOR_TAG "Accel Live event:%d xyz_raw<%f %f %f> xyz_bias<%f %f %f> timestamp<boot,gptp> %lld %lld latency(ms)=%lld odr(ms)=%lld\n",
			      AccCount++,
			      data.getX_uncalib(), data.getY_uncalib(), data.getZ_uncalib(),
			      data.getX_bias(), data.getY_bias(), data.getZ_bias(),
			      e->getTimestamp(), e->getGptptimestamp(),
			      (currPTPtime - e->getGptptimestamp())/1000000,
			      (e->getGptptimestamp()- acc_ts)/1000000);
	  acc_ts = e->getGptptimestamp();
  }
  else if((e->getType() == SensorInterface::Sensortype::SENSOR_TYPE_GYROSCOPE_UNCALIBRATED ) || 
		  (e->getType() == SensorInterface::Sensortype::SENSOR_TYPE_GYROSCOPE ) ) {
	  const SensorInterface::SensorUncalibratedEvent & data = e->getData().get<SensorInterface::SensorUncalibratedEvent>();
	  SENSOR_LOGV(SENSOR_TAG "Gyro Live event:%d xyz_raw<%f %f %f> xyz_bias<%f %f %f> timestamp<boot,gptp> %lld %lld latency(ms)=%lld odr(ms)=%lld\n",
			      GyroCount++,
			      data.getX_uncalib(), data.getY_uncalib(), data.getZ_uncalib(),
			      data.getX_bias(), data.getY_bias(), data.getZ_bias(),
			      e->getTimestamp(),
			      e->getGptptimestamp(),
			      (currPTPtime - e->getGptptimestamp())/1000000,
			      (e->getGptptimestamp()-gyro_ts)/1000000);
	  gyro_ts = e->getGptptimestamp();
  }
  else if((e->getType() == SensorInterface::Sensortype::SENSOR_TYPE_HEADING ) ) {
	  const SensorInterface::SensorHeadingEvent & head = e->getData().get<SensorInterface::SensorHeadingEvent>();
	  SENSOR_LOGV(SENSOR_TAG "Head Live event:%d heading and accuracy<%f %f> timestamp<boot,gptp> %lld %lld latency(ms)=%lld odr(ms)=%lld\n",
		 HeadCount++,
		 head.getHeading(), head.getAccuracy(),
		 e->getTimestamp(), e->getGptptimestamp(),
		 (currPTPtime - e->getGptptimestamp())/1000000,
		 (e->getGptptimestamp()- head_ts)/1000000);
	  head_ts = e->getGptptimestamp();
  }
  else {
	  SENSOR_LOGV(SENSOR_TAG "Sensor Live unknown sensor_id events %d\n", e->getType());
  }
  return;
}

static void  onSensorDataReadCb(vector<SensorInterface::SensorEvent> events, uint32_t count)
{
  int i =0;
  static uint64_t ts_prv_acc = 0, ts_prv_gyro = 0 , ts_prv_head = 0, ts_cur = 0;
  static uint64_t acc_sensor_ts = 0, gyro_sensor_ts = 0, head_sensor_ts = 0;
  bool retPtp = false;

  int sensor_id = events[0].getSensorId();

  if ((nullptr != gPTPReqIf) && (nullptr != gPTPReqIf->gptpGetCurPtpTimeIf)) {
	  retPtp = gPTPReqIf->gptpGetCurPtpTimeIf(&ts_cur);
  }
  if (sensor_id == 1) {
	  acc_sensor_ts = events[count-1].getGptptimestamp();
	  SENSOR_LOGV(SENSOR_TAG "Sensor ACC Live: sensor_id %d: count %d batch_delta(ms)=%lld cur_gptp_ts %lld latency(ms)=%lld\n",
			  sensor_id, count, (ts_cur - ts_prv_acc)/1000000, ts_cur, (ts_cur - acc_sensor_ts)/1000000);
	  ts_prv_acc = ts_cur;
  }
  if (sensor_id == 16 ) {
	  gyro_sensor_ts = events[count-1].getGptptimestamp();
	  SENSOR_LOGV(SENSOR_TAG "Sensor GYRO Live: sensor_id %d: count %d batch_delta(ms)=%lld cur_gptp_ts %lld latency(ms)=%lld\n",
			  sensor_id, count, (ts_cur - ts_prv_gyro)/1000000, ts_cur, (ts_cur - gyro_sensor_ts)/1000000);
	  ts_prv_gyro = ts_cur;
  }
  if (sensor_id == 3 ) {
	  head_sensor_ts = events[count-1].getGptptimestamp();
	  SENSOR_LOGV(SENSOR_TAG "Sensor HEAD Live: sensor_id %d: count %d batch_delta(ms)=%lld cur_gptp_ts %lld latency(ms)=%lld\n",
			  sensor_id, count, (ts_cur - ts_prv_head)/1000000, ts_cur, (ts_cur - head_sensor_ts)/1000000);
	  ts_prv_head = ts_cur;
  }
  for ( i = 0; i < count ; i++){
	  dump_live_event(&events[i]);
  }
  return;
}

static void PrintSensorList(vector<SensorInterface::SensorList> sensor, int32_t sensor_count)
{
  for (int i=0 ; i< sensor_count ; i++) {
     vector<float>odr = sensor[i].getOdr();

     SENSOR_LOGV(SENSOR_TAG "Name: %s\n", sensor[i].getName().c_str());
     SENSOR_LOGV(SENSOR_TAG "\tvendor: %s\n", sensor[i].getVendor().c_str());
     SENSOR_LOGV(SENSOR_TAG "\tversion: %d\n",sensor[i].getSensorVersion());
     SENSOR_LOGV(SENSOR_TAG "\tresolution: %f\n",sensor[i].getResolution());
     SENSOR_LOGV(SENSOR_TAG "\tmaxRange %f\n",sensor[i].getMaxRange());
     SENSOR_LOGV(SENSOR_TAG "\tsensor_id: %d\n",sensor[i].getSensorId());
     SENSOR_LOGV(SENSOR_TAG "\ttype: %d\n",sensor[i].getType());
     SENSOR_LOGV(SENSOR_TAG "\trange: %d\n",sensor[i].getRange());
     SENSOR_LOGV(SENSOR_TAG "\tmaxSamplingRate: %f\n",sensor[i].getMaxSamplingRate());
     SENSOR_LOGV(SENSOR_TAG "\tminBatchCount: %d\n",sensor[i].getMinBatchCount());
     SENSOR_LOGV(SENSOR_TAG "\tmaxBatchCount: %d\n",sensor[i].getMaxBatchCount());
     SENSOR_LOGV(SENSOR_TAG "\todr rate: %fHZ %fHZ %fHZ %fHZ %fHZ %fHZ\n",
		     odr[0], odr[1],odr[2],odr[3],odr[4],odr[5]);
    }
    return;
}

void SensorCore::SensorCore_Init() {

    regSigHandler();

    /* GPTP */
    loadGptpLibFile();
    if ((nullptr != gPTPReqIf) && (nullptr != gPTPReqIf->gptpInitIf)) {
	    SENSOR_LOGI(SENSOR_TAG "GPTP init\n");
	    gPTPReqIf->gptpInitIf();
    }

    CommonAPI::Runtime::setProperty("LogContext", "E01C");
    CommonAPI::Runtime::setProperty("LogApplication", "E01C");
    CommonAPI::Runtime::setProperty("LibraryBase", "SensorInterface");

    shared_ptr < CommonAPI::Runtime > runtime = CommonAPI::Runtime::get();

    string domain = "local";
    string instance = "com.qualcomm.qti.sensor.SensorInterface";
    string connection = "client-sample";

    myProxy=runtime->buildProxy<SensorInterfaceProxy>(domain,instance,connection);

    SENSOR_LOGI(SENSOR_TAG "Checking IDL Service availability !!\n");
    while (!myProxy->isAvailable())
	    usleep(10);
    SENSOR_LOGI(SENSOR_TAG "IDL Service is now available !!\n");


    capSubscription = myProxy->getSensorCapabilitiesEvent().subscribe(
       [&](const ::v0::com::qualcomm::qti::sensor::SensorInterface::SensorCapabilitiesMask &mask) {
       SENSOR_LOGI(SENSOR_TAG "<<--Received SensorCapabilitiesMask mask %d\n", static_cast<int>(mask));
       onCapabilitiesCb(mask);
    });

    batchSubscription = myProxy->getSensorConfigUpdateEvent().subscribe(
       [&](int32_t sensor_id, float SamplingRate, int32_t BatchCount) {
       SENSOR_LOGI(SENSOR_TAG "<<--Received SensorConfigUpdateCb id: %d SamplingRate : %f BatchCount: %f\n", sensor_id, SamplingRate, BatchCount);
    });

    dataSubscription = myProxy->getSensorDataReadEvent().subscribe(
       [&](vector< ::v0::com::qualcomm::qti::sensor::SensorInterface::SensorEvent > events, uint32_t count) {
       //onSensorDataReadCb(events, count);
       vector<SensorCoreData> idlSensorEventsData;
       SensorCoreData idlSensorEvents = {};
       for (int i=0 ;i <count; i++){
         memset(&idlSensorEvents, 0, sizeof(idlSensorEvents));
	 const SensorInterface::SensorUncalibratedEvent & data = events[i].getData().get<SensorInterface::SensorUncalibratedEvent>();
	 idlSensorEvents.sensorId = events[i].getSensorId();
	 idlSensorEvents.Type = events[i].getType();
	 idlSensorEvents.timestamp = events[i].getTimestamp();
	 idlSensorEvents.gptptimestamp = events[i].getGptptimestamp();
	 idlSensorEvents.xyz = {data.getX_uncalib(), data.getY_uncalib(), data.getZ_uncalib(),
	 			data.getX_bias(), data.getY_bias(), data.getZ_bias()};  
	 idlSensorEventsData.push_back(idlSensorEvents);
       }
       onNewSensorsData(idlSensorEventsData); 
    });

    SENSOR_LOGI(SENSOR_TAG "==== Register new client ====>>\n");
    myProxy->RegisterSensorClient(callStatus, resp, &info);
    if (callStatus != CommonAPI::CallStatus::SUCCESS) {
       SENSOR_LOGE(SENSOR_TAG "RegisterSensorClient() Remote call failed! callStatus %d\n", (int)callStatus);
       return;
    }
    parseSensorResponse(resp);
    return;
}

void SensorCore::SensorCore_getSensorList(std::vector<SensorCoreList> &sensorVector, int32_t *sensorcount) {
    static bool sensorlistready = false;
    SENSOR_LOGI(SENSOR_TAG "==== Calling GetSensorList ====>> sensorlistready %d \n", sensorlistready);
    if(sensorlistready == false) {
       myProxy->GetSensorList(callStatus, sensor, sensor_count, &info);
       if (callStatus != CommonAPI::CallStatus::SUCCESS) {
	       SENSOR_LOGE(SENSOR_TAG "sensor get list failed ret %d \n", (int)callStatus);
	       return;
       }
       PrintSensorList(sensor, sensor_count);
       *sensorcount = sensor_count;
       for (int i=0 ; i < sensor_count ; i++) {
	       vector<float>odr = sensor[i].getOdr();
	       setSensorCoreList(sensor[i].getName(), sensor[i].getVendor(),sensor[i].getSensorVersion(),
			       sensor[i].getResolution(), sensor[i].getMaxRange(), sensor[i].getSensorId(), sensor[i].getType(),
			       sensor[i].getRange(), sensor[i].getMaxSamplingRate(), sensor[i].getMinBatchCount(),
			     sensor[i].getMaxBatchCount(),odr);
       }
       sensorlistready = true;
    }
    sensorVector = getSensorCoreList();
    *sensorcount = sensor_count;
    return;
}

void SensorCore::SensorCore_acitvateSensor(int32_t in_sensorHandle, bool in_enabled) {
    if (in_sensorHandle == accel_cal_id)
	    in_sensorHandle = accel_uncal_id;
    if (in_sensorHandle == gyro_cal_id)
	    in_sensorHandle = gyro_uncal_id;

    if (in_enabled == true)
        state = SensorInterface::SensorState::SENSOR_ENABLE;
    else 
	state = SensorInterface::SensorState::SENSOR_DISABLE;
 
    SENSOR_LOGI(SENSOR_TAG "==== Calling SensorControl ====>> sensor_id: %d state: %d \n", in_sensorHandle, static_cast<int>(state));
    myProxy->SensorControl(in_sensorHandle, state, callStatus, resp, &info);
    if (callStatus != CommonAPI::CallStatus::SUCCESS) {
	    SENSOR_LOGE(SENSOR_TAG "sensor control failed sensor[i].sensor_id %d ret %d \n", in_sensorHandle, (int)callStatus);
	    return;
    }
 
    parseSensorResponse(resp);
    return;
}

void SensorCore::SensorCore_configSensor(int32_t in_sensorHandle, int64_t in_samplingPeriodNs,int64_t in_maxReportLatencyNs) {
    SENSOR_LOGI(SENSOR_TAG  "==== Calling SensorConfig ====>> sensor_id: %d in_samplingPeriodNs %lld in_maxReportLatencyNs %lld\n", in_sensorHandle, in_samplingPeriodNs, in_maxReportLatencyNs);

    if (in_sensorHandle == accel_cal_id)
	    in_sensorHandle = accel_uncal_id;
    if (in_sensorHandle == gyro_cal_id)
	    in_sensorHandle = gyro_uncal_id;

    myProxy->SensorConfig(in_sensorHandle, 100, 10, callStatus, resp, &info);
    if (callStatus != CommonAPI::CallStatus::SUCCESS) {
	    SENSOR_LOGE(SENSOR_TAG "sensor config  failed sensor_id %d ret %d \n", in_sensorHandle, (int)callStatus);
	    return;
    }

    parseSensorResponse(resp);
    return;
}

uint64_t SensorCore::SensorCore_getBootTimeFromPtpTime(uint64_t ptp_time_ns)
{
   uint64_t boot_time_ns;
   gPTPReqIf->gptpGetBootTimeFromPtpTimeIf(&boot_time_ns, ptp_time_ns);
   SENSOR_LOGV(SENSOR_TAG "gptp converted ptp_time_ns %lld boot_time_ns %lld\n", ptp_time_ns, boot_time_ns);
   return boot_time_ns;
}

void SensorCore_Deinit() {
    SENSOR_LOGI(SENSOR_TAG  "==== SensorCore_Deinit ====>>\n");
    DeInitHandles();
    return;
}
