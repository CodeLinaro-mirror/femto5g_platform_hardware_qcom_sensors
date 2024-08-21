/*
 *  Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 *
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
#include <sys/types.h>
#include <gptp_helper.h>
#include <signal.h>

using namespace v1::com::qualcomm::qti::sensor;
using namespace std;
#define NSEC_IN_ONE_SEC       (1000000000ULL)   /* nanosec in a sec */
#define ACCEL_UNCALIBRATED_SENSOR_ID 1
#define GYRO_UNCALIBRATED_SENSOR_ID  16
#define HEADING_SENSOR_ID   5
#define GID_SENSORCLIENT (3011)

struct SensorTrackingOption {
   int sensor_id;
   float sampling_rate;
   int batch_count;
   SensorInterfaceTypes::SensorStateT state;
};

shared_ptr<SensorInterfaceProxy<>> myProxy;
int32_t sensor_count = 0;
SensorInterfaceTypes::SensorStateT state = SensorInterfaceTypes::SensorStateT::SENSOR_STATE_DISABLE;
SensorInterfaceTypes::SensorReturnT resp;
struct SensorTrackingOption *mSensorTrackingOption;
vector<SensorInterfaceTypes::SensorInfoT> sensor;
CommonAPI::CallStatus callStatus;
CommonAPI::CallInfo info(1000);
bool SHD_RESTARTED = false;
uint32_t capSubscription;
uint32_t batchSubscription;
uint32_t imuDataSubscription;
uint32_t headingDataSubscription;

static uint64_t getTimestamp() {
    struct timespec ts;
    clock_gettime(CLOCK_BOOTTIME, &ts);
    uint64_t system_ts =
	    ((uint64_t)(ts.tv_sec)) * 1000000000ULL + ((uint64_t)(ts.tv_nsec));
    return system_ts;
}

void parseSensorReturnT(SensorInterfaceTypes::SensorReturnT resp) {
    switch(resp) {
       case SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_SUCCESS:
	       cout << "SENSOR_RETURN_SUCCESS " << resp << endl;
	       break;
       case SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_CLIENT_REGISTER_FAILED:
	       cout << "SENSOR_RETURN_ERROR_CLIENT_REGISTER_FAILED " << resp << endl;
	       break;
       case SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_INVALID_CLIENT:
	       cout << "SENSOR_RETURN_ERROR_INVALID_CLIENT " << resp << endl;
	       break;
       case SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_INVALID_INPUT_PARAMETER:
	       cout << "SENSOR_RETURN_ERROR_INVALID_INPUT_PARAMETER " << resp << endl;
	       break;
       case SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_CONTROL_FAILED:
	       cout << "SENSOR_RETURN_ERROR_CONTROL_FAILED " << resp << endl;
	       break;
       case SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_CONFIG_FAILED:
	       cout << "SENSOR_RETURN_ERROR_CONFIG_FAILED " << resp << endl;
	       break;
       case SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_NO_SENSORS_FOUND:
	       cout << "SENSOR_RETURN_ERROR_NO_SENSORS_FOUND " << resp << endl;
	       break;
       case SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_TRACKING_FAILED:
	       cout << "SENSOR_RETURN_ERROR_TRACKING_FAILED " << resp << endl;
	       break;
       case SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_UNKNOWN:
	       cout << "SENSOR_RETURN_ERROR_UNKNOWN " << resp << endl;
	       break;
       default:
	       cout << "SENSOR_RETURN_ERROR_UNKNOWN " << resp << endl;
	       break;
    }
}

void DeInitHandles()
{
   CommonAPI::CallStatus callStatus;
   SensorInterfaceTypes::SensorReturnT resp;

   myProxy->getSensorCapabilitiesEvent().unsubscribe(capSubscription);
   myProxy->getSensorConfigUpdateEvent().unsubscribe(batchSubscription);
   myProxy->getSensorImuDataReadEvent().unsubscribe(imuDataSubscription);
   myProxy->getSensorHeadingDataReadEvent().unsubscribe(headingDataSubscription);

   myProxy->DeRegisterSensorClientReq(callStatus, resp, &info);
   if (callStatus != CommonAPI::CallStatus::SUCCESS) {
	   cout << "DeRegisterSensorClientReq() Remote call failed! callStatus " << (int)callStatus << endl;
   }
   parseSensorReturnT(resp);
}

void signalHandler(int signal)
{
   cout << "signalHandler " <<endl;
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
}

static void onCapabilitiesCb(SensorInterfaceTypes::SensorServiceStateMaskT mask) {
    switch (mask) {
      case SensorInterfaceTypes::SensorServiceStateMaskT::SENSOR_SERVICE_STATE_MASK_READY:
	      printf( "Sensor Hal daemon is Ready to commnunicate\n");
	      break;
    }
}

static void dump_live_event(SensorInterfaceTypes::SensorImuEventT *e)  {
    static int64_t acc_ts = 0;
    static int64_t gyro_ts = 0;
    static int AccCount = 0, GyroCount = 0;
    uint64_t currPTPtime = 0;

    gptpGetCurPtpTime(&currPTPtime);

    if((e->getType() == SensorInterfaceTypes::SensorTypeT::SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED ) ||
		    (e->getType() == SensorInterfaceTypes::SensorTypeT::SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED ) ) {
	    const SensorInterfaceTypes::SensorUncalibratedEventT & data = e->getData();
	    printf("Accel Live event:%d xyz_raw<%f %f %f> xyz_bias<%f %f %f> timestamp<boot,gptp> %lld %lld latency(ms)=%lld odr(ms)=%lld\n", 
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
	    printf("Gyro Live event:%d xyz_raw<%f %f %f> xyz_bias<%f %f %f> timestamp<boot,gptp> %lld %lld latency(ms)=%lld odr(ms)=%lld\n", 
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
}

static void  onSensorImuDataReadCb(vector<SensorInterfaceTypes::SensorImuEventT> events, uint32_t count)
{
    int i =0;
    static uint64_t ts_prv_acc = 0, ts_prv_gyro = 0 , ts_cur = 0;
    static uint64_t acc_sensor_ts = 0, gyro_sensor_ts = 0;
    gptpGetCurPtpTime(&ts_cur);

    int sensor_id = events[0].getSensorId();

    if (sensor_id == ACCEL_UNCALIBRATED_SENSOR_ID) {
	    acc_sensor_ts = events[count-1].getGptpTimestamp();
	    printf( "Sensor ACC Live: sensor_id %d: count %d batch_delta(ms)=%lld cur_gptp_ts %lld latency(ms)=%lld\n",
			    sensor_id, count, (ts_cur - ts_prv_acc)/1000000, ts_cur, (ts_cur - acc_sensor_ts)/1000000);
	    ts_prv_acc = ts_cur;
    }
    if (sensor_id == GYRO_UNCALIBRATED_SENSOR_ID) {
	    gyro_sensor_ts = events[count-1].getGptpTimestamp();
	    printf( "Sensor GYRO Live: sensor_id %d: count %d batch_delta(ms)=%lld cur_gptp_ts %lld latency(ms)=%lld\n",
			    sensor_id, count, (ts_cur - ts_prv_gyro)/1000000, ts_cur, (ts_cur - gyro_sensor_ts)/1000000);
	    ts_prv_gyro = ts_cur;
    }

    for ( i = 0; i < count ; i++)
	    dump_live_event(&events[i]);
}

static void  onSensorHeadingDataReadCb(vector<SensorInterfaceTypes::SensorHeadEventT> events, uint32_t count)
{
    int i =0;
    static uint64_t ts_prv_head = 0, ts_cur = 0;
    static uint64_t head_sensor_ts = 0;
    static int64_t head_ts = 0;
    static int HeadCount = 0;
    gptpGetCurPtpTime(&ts_cur);

    int sensor_id = events[0].getSensorId();

    if (sensor_id == HEADING_SENSOR_ID) {
	 head_sensor_ts = events[count-1].getGptpTimestamp();
	 printf( "Sensor HEAD Live: sensor_id %d: count %d batch_delta(ms)=%lld cur_gptp_ts %lld latency(ms)=%lld\n",
			 sensor_id, count, (ts_cur - ts_prv_head)/1000000, ts_cur, (ts_cur - head_sensor_ts)/1000000);
	 ts_prv_head = ts_cur;
    }
    for ( i = 0; i < count ; i++) {
	 if(events[i].getType() == SensorInterfaceTypes::SensorTypeT::SENSOR_TYPE_HEADING ) { 
		 const SensorInterfaceTypes::SensorHeadingEventT & head = events[i].getData();
		 printf("Head Live event:%d heading and accuracy<%f %f> timestamp<boot,gptp> %lld %lld latency(ms)=%lld odr(ms)=%lld\n",
				 HeadCount++,
				 head.getHeading(), head.getAccuracy(),
				  events[i].getTimestamp(), events[i].getGptpTimestamp(),
				  (ts_cur - events[i].getGptpTimestamp())/1000000,
				  (events[i].getGptpTimestamp()- head_ts)/1000000);
		 head_ts = events[i].getGptpTimestamp();
	 }
    }
}

static void PrintSensorInfoT(vector<SensorInterfaceTypes::SensorInfoT> sensor, int32_t sensor_count)
{
    cout << "sensor_count: " << sensor_count << endl;
    for (int i=0 ; i< sensor_count ; i++) {
	   cout << "Name:" << sensor[i].getName() << endl;
	   cout << "\tvendor: " << sensor[i].getVendor() << endl;
	   printf( "\tversion: %d\n",sensor[i].getSensorVersion());
	   printf( "\tresolution: %f\n",sensor[i].getResolution());
	   printf( "\tmaxRange %f\n",sensor[i].getMaxRange());
	   printf( "\tsensor_id: %d\n",sensor[i].getSensorId());
	   printf( "\ttype: %d\n",sensor[i].getType());
	   printf( "\tmaxSamplingRate: %f\n",sensor[i].getMaxSamplingRate());
	   printf( "\tminBatchCount: %d\n",sensor[i].getMinBatchCount());
	   printf( "\tmaxBatchCount: %d\n",sensor[i].getMaxBatchCount());
	   vector<float>odr = sensor[i].getOdr();
	   printf( "\todr rate: %fHZ %fHZ %fHZ %fHZ %fHZ %fHZ\n",
			       odr[0], odr[1],odr[2],odr[3],odr[4],odr[5]);
    }
}

static void printHelp() {
    printf( "\n************* options *************\n");
    printf( "h: help\n");
    printf( "g: Get Sensor list\n");
    printf( "c: configure Sensor sampling and batch count\n");
    printf( "a: Activate/Deactivate the Sensor\n");
    printf( "q: Quit\n");
}

int main() {
    char *stopstring;
    char odr[10];
    char enable[10];
    char batchcount[10];
    int batch_count = 0, j = 0, i = 0, Enable = 0;
    info.sender_ = 1234;

    // check if this process started by root
    if (0 == getuid()) { 
        // started as root.
        printf("!!! sensor idl client started as root change to sensors group\n");
        // Set the group id first and then set the effective userid, to sensors.
        if(-1 == setgid(GID_SENSORCLIENT)) {
            printf("Error: setgid failed. %s\n", strerror(errno));
        }
    }

    regSigHandler();

    CommonAPI::Runtime::setProperty("LogContext", "SensorInterface");
    CommonAPI::Runtime::setProperty("LogApplication", "SensorInterface");
    CommonAPI::Runtime::setProperty("LibraryBase", "SensorInterface");

    shared_ptr < CommonAPI::Runtime > runtime = CommonAPI::Runtime::get();

    string domain = "local";
    string instance = "com.qualcomm.qti.sensor.SensorInterface";
    string connection = "sensor-fidl-client";

    myProxy=runtime->buildProxy<SensorInterfaceProxy>(domain,instance,connection);

    cout << "Checking Sensor Service availability !!" << endl;
    while (!myProxy->isAvailable())
	    usleep(10);
    cout << "Sensor Service is now available !!" << endl;

    if (gptpInit())
	    cout << " gptpinit success" << endl;
    else 
	    cout << " gptpinit failed" << endl;

    myProxy->getProxyStatusEvent().subscribe([&] (const CommonAPI::AvailabilityStatus status) {
      switch (status) {
	case CommonAPI::AvailabilityStatus::UNKNOWN:
	cout << "Sensor Service Unkown" << endl;
	SHD_RESTARTED = true;
	break;
	case CommonAPI::AvailabilityStatus::NOT_AVAILABLE:
	cout << "Sensor Service NOT_AVAILABLE" << endl;
	SHD_RESTARTED = true;
	break;
	case CommonAPI::AvailabilityStatus::AVAILABLE:
	cout << "Sensor Service AVAILABLE" << endl;
	SHD_RESTARTED = false;
	myProxy->RegisterSensorClientReq(callStatus, resp, &info);
	for (int i=0; i < sensor_count; i++) {
	 cout << "Reconfiguring Enabled Sensors" << endl;
	 if (mSensorTrackingOption[i].state == SensorInterfaceTypes::SensorStateT::SENSOR_STATE_ENABLE) {
	   myProxy->SensorConfigReq(mSensorTrackingOption[i].sensor_id, mSensorTrackingOption[i].sampling_rate,
			mSensorTrackingOption[i].batch_count, callStatus, resp, &info);
	   myProxy->SensorControlReq(mSensorTrackingOption[i].sensor_id, mSensorTrackingOption[i].state, callStatus, resp, &info);
	 }
	}
	break;
      }
    });

    capSubscription = myProxy->getSensorCapabilitiesEvent().subscribe(
        [&](const ::v1::com::qualcomm::qti::sensor::SensorInterfaceTypes::SensorServiceStateMaskT &mask) {
	cout << "<<--Received SensorServiceStateMaskT :" << mask << endl;
	onCapabilitiesCb(mask);
	cout << "<<-----" << endl;
    });

    batchSubscription = myProxy->getSensorConfigUpdateEvent().subscribe(
       [&](int32_t sensor_id, float SamplingRate, int32_t BatchCount) {
       cout << "<<--Received SensorConfigReqReqReqUpdate id: " << sensor_id << " SamplingRate : " << SamplingRate << " BatchCount : " << BatchCount << endl;
       cout << "<<-------" << endl;
    });

    imuDataSubscription = myProxy->getSensorImuDataReadEvent().subscribe(
       [&](vector< ::v1::com::qualcomm::qti::sensor::SensorInterfaceTypes::SensorImuEventT > events, uint32_t count) {
       cout << "<<--Received SensorImuDataRead sensor_id: " << events[0].getSensorId() << endl;
       onSensorImuDataReadCb(events, count);
       cout << "<<-------" << endl;
    });

    headingDataSubscription = myProxy->getSensorHeadingDataReadEvent().subscribe(
       [&](vector< ::v1::com::qualcomm::qti::sensor::SensorInterfaceTypes::SensorHeadEventT > events, uint32_t count) {
       cout << "<<--Received SensorHeadingDataRead sensor_id: " << events[0].getSensorId() << endl;
       onSensorHeadingDataReadCb(events, count);
       cout << "<<-------" << endl;
    });
 
    cout << "==== Register new client ====>> " << endl;
    myProxy->RegisterSensorClientReq(callStatus, resp, &info);
    if (callStatus != CommonAPI::CallStatus::SUCCESS) {
	    cout << "SensorClient() Remote call failed! callStatus " << (int)callStatus << endl;
    }
    parseSensorReturnT(resp);
    sleep(1);

    cout << "==== Calling GetSensorListReq ====>> " << endl;
    myProxy->GetSensorListReq(callStatus, sensor, sensor_count, &info);
    if (callStatus != CommonAPI::CallStatus::SUCCESS) {
	    printf( "sensor get list ret %d \n", (int)callStatus);
	    return;
    }
    PrintSensorInfoT(sensor, sensor_count);
    mSensorTrackingOption = new (std::nothrow) struct SensorTrackingOption[sensor_count];

    sleep(1);

    printHelp();

    while (true) {
      char buf[10];
      memset (buf, 0, sizeof(buf)/sizeof(buf[0]));
      fgets(buf, sizeof(buf)/sizeof(buf[0]), stdin);
      int command = buf[0];

      switch(command) {
	  case 'g':
	    cout << "==== Calling GetSensorListReq ====>> " << endl;
	    myProxy->GetSensorListReq(callStatus, sensor, sensor_count, &info);
	    if (callStatus != CommonAPI::CallStatus::SUCCESS) {
		    printf( "sensor get list ret %d \n", (int)callStatus);
	    }
	    PrintSensorInfoT(sensor,sensor_count);
	    break;
	  case 'c':
	    cout << "==== Calling SensorConfigReqReqReq ====>> " << endl;
	    for(int i=0; i < sensor_count; i++) {
		vector<float>mODR = sensor[i].getOdr();
	        printf("Enter 0:%fHZ 1:%fHZ 2:%fHZ 3:%fHZ 4:%fHZ 5:%fHZ\n", mODR[0],mODR[1],mODR[2],mODR[3],mODR[4],mODR[5]);
	        memset (odr, 0, sizeof(odr)/sizeof(odr[0]));
		memset (batchcount, 0, sizeof(batchcount)/sizeof(batchcount[0]));
		printf( "Enter sampling rate: ");
		fgets(odr, sizeof(odr)/sizeof(odr[0]), stdin);
		j=strtol(odr,&stopstring,10);
		printf( "Enter batch set: ");
		fgets(batchcount, sizeof(batchcount)/sizeof(batchcount[0]), stdin);
		batch_count=strtol(batchcount,&stopstring,10);
		printf("\nConfig sensor_id %d  sampling %fHZ and batch %d\n", sensor[i].getSensorId(), mODR[j], batch_count);
		myProxy->SensorConfigReq(sensor[i].getSensorId(), mODR[j], batch_count, callStatus, resp, &info);
		if (callStatus != CommonAPI::CallStatus::SUCCESS) {
			printf( "sensor config  failed sensor[i].sensor_id %d ret %d \n", sensor[i].getSensorId(), (int)callStatus);
		}
		else {
			parseSensorReturnT(resp);
			mSensorTrackingOption[i].sensor_id = sensor[i].getSensorId();
			mSensorTrackingOption[i].sampling_rate = mODR[j];
			mSensorTrackingOption[i].batch_count = batch_count;
		}
	    }
	    break;
	  case 'a':
	    cout << "==== Calling SensorControlReq ====>>" <<  endl;
	    for(int i=0; i < sensor_count; i++) {
		printf( "Enable/Disable the sensor id %d\n",sensor[i].getSensorId());
		fgets(enable, sizeof(enable)/sizeof(enable[0]), stdin);
		Enable=strtol(enable,&stopstring,10);
		if ( Enable == 1)
			state = SensorInterfaceTypes::SensorStateT::SENSOR_STATE_ENABLE;
		else
			state = SensorInterfaceTypes::SensorStateT::SENSOR_STATE_DISABLE;
		cout << "sensor id:" << sensor[i].getSensorId() << " with contol value: " << (SensorInterfaceTypes::SensorStateT)state << endl;
		myProxy->SensorControlReq(sensor[i].getSensorId(), state, callStatus, resp, &info);
		if (callStatus != CommonAPI::CallStatus::SUCCESS) {
			printf( "sensor control failed sensor[i].sensor_id %d ret %d \n", sensor[i].getSensorId(), (int)callStatus);
		}
		else {
			parseSensorReturnT(resp);
			mSensorTrackingOption[i].state = state;
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
      }//end of switch

    }//while(1)

EXIT:
    printf("Done\n");
    usleep(5000);
    DeInitHandles();
    exit(0);
}
