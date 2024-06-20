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
#include <v0/com/qualcomm/qti/sensor/SensorInterfaceProxy.hpp>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h>
#include <string.h>
#include <sys/types.h>
#include <gptp_helper.h>
#include <signal.h>

using namespace v0::com::qualcomm::qti::sensor;
using namespace std;
#define NSEC_IN_ONE_SEC       (1000000000ULL)   /* nanosec in a sec */
#define ACCEL_UNCALIBRATED_SENSOR_ID 1
#define GYRO_UNCALIBRATED_SENSOR_ID  16
#define HEADING_SENSOR_ID   5

struct SensorTrackingOption {
   int sensor_id;
   float sampling_rate;
   int batch_count;
   SensorInterface::SensorState state;
};

shared_ptr<SensorInterfaceProxy<>> myProxy;
int32_t sensor_count = 0;
SensorInterface::SensorState state = SensorInterface::SensorState::SENSOR_DISABLE;
SensorInterface::SensorResponse resp;
struct SensorTrackingOption *mSensorTrackingOption;
vector<SensorInterface::SensorList> sensor;
CommonAPI::CallStatus callStatus;
CommonAPI::CallInfo info(1000);
bool SHD_RESTARTED = false;
uint32_t capSubscription;
uint32_t batchSubscription;
uint32_t dataSubscription;

static uint64_t getTimestamp() {
    struct timespec ts;
    clock_gettime(CLOCK_BOOTTIME, &ts);
    uint64_t system_ts =
	    ((uint64_t)(ts.tv_sec)) * 1000000000ULL + ((uint64_t)(ts.tv_nsec));
    return system_ts;
}

void parseSensorResponse(SensorInterface::SensorResponse resp) {
    switch(resp) {
       case SensorInterface::SensorResponse::SENSOR_RESPONSE_SUCCESS:
	       cout << "SENSOR_RESPONSE_SUCCESS " << resp << endl;
	       break;
       case SensorInterface::SensorResponse::SENSOR_ERROR_CLIENT_REGISTER_FAILED:
	       cout << "SENSOR_ERROR_CLIENT_REGISTER_FAILED " << resp << endl;
	       break;
       case SensorInterface::SensorResponse::SENSOR_ERROR_INVALID_CLIENT:
	       cout << "SENSOR_ERROR_INVALID_CLIENT " << resp << endl;
	       break;
       case SensorInterface::SensorResponse::SENSOR_ERROR_INVALID_INPUT_PARAMETER:
	       cout << "SENSOR_ERROR_INVALID_INPUT_PARAMETER " << resp << endl;
	       break;
       case SensorInterface::SensorResponse::SENSOR_ERROR_CONTROL_FAILED:
	       cout << "SENSOR_ERROR_CONTROL_FAILED " << resp << endl;
	       break;
       case SensorInterface::SensorResponse::SENSOR_ERROR_CONFIG_FAILED:
	       cout << "SENSOR_ERROR_CONFIG_FAILED " << resp << endl;
	       break;
       case SensorInterface::SensorResponse::SENSOR_ERROR_NO_SENSORS_FOUND:
	       cout << "SENSOR_ERROR_NO_SENSORS_FOUND " << resp << endl;
	       break;
       case SensorInterface::SensorResponse::SENSOR_ERROR_TRACKING_FAILED:
	       cout << "SENSOR_ERROR_TRACKING_FAILED " << resp << endl;
	       break;
       case SensorInterface::SensorResponse::SENSOR_ERROR_UNKNOWN:
	       cout << "SENSOR_ERROR_UNKNOWN " << resp << endl;
	       break;
       default:
	       cout << "SENSOR_ERROR_UNKNOWN " << resp << endl;
	       break;
    }
}

void DeInitHandles()
{
   CommonAPI::CallStatus callStatus;
    SensorInterface::SensorResponse resp;

   myProxy->getSensorCapabilitiesEvent().unsubscribe(capSubscription);
   myProxy->getSensorConfigUpdateEvent().unsubscribe(batchSubscription);
   myProxy->getSensorDataReadEvent().unsubscribe(dataSubscription);

   myProxy->DeRegisterSensorClient(callStatus, resp, &info);
   if (callStatus != CommonAPI::CallStatus::SUCCESS) {
	   cout << "DeRegisterSensorClient() Remote call failed! callStatus " << (int)callStatus << endl;
   }
   parseSensorResponse(resp);
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

static void onCapabilitiesCb(SensorInterface::SensorCapabilitiesMask mask) {
    switch (mask) {
      case SensorInterface::SensorCapabilitiesMask::SHD_READY:
	      printf( "Sensor Hal daemon is Ready to commnunicate\n");
	      break;
    }
}

static void dump_live_event(SensorInterface::SensorEvent *e)  {
    static int64_t acc_ts = 0;
    static int64_t gyro_ts = 0;
    static int64_t head_ts = 0;
    static int AccCount = 0, GyroCount = 0, HeadCount = 0;
    uint64_t currPTPtime = 0;

    gptpGetCurPtpTime(&currPTPtime);

    if((e->getType() == SensorInterface::Sensortype::SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED ) ||
		    (e->getType() == SensorInterface::Sensortype::SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED ) ) {
	    const SensorInterface::SensorUncalibratedEvent & data = e->getData().get<SensorInterface::SensorUncalibratedEvent>();
	    printf("Accel Live event:%d xyz_raw<%f %f %f> xyz_bias<%f %f %f> timestamp<boot,gptp> %lld %lld latency(ms)=%lld odr(ms)=%lld\n", 
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
	    printf("Gyro Live event:%d xyz_raw<%f %f %f> xyz_bias<%f %f %f> timestamp<boot,gptp> %lld %lld latency(ms)=%lld odr(ms)=%lld\n", 
                                    GyroCount++,
				    data.getX_uncalib(), data.getY_uncalib(), data.getZ_uncalib(),
				    data.getX_bias(), data.getY_bias(), data.getZ_bias(),
                                    e->getTimestamp(),
                                    e->getGptptimestamp(),
				    (currPTPtime - e->getGptptimestamp())/1000000,
				    (e->getGptptimestamp()-gyro_ts)/1000000);
	    gyro_ts = e->getGptptimestamp();
    }
    else if(e->getType() == SensorInterface::Sensortype::SENSOR_TYPE_HEADING ) { 
	    const SensorInterface::SensorHeadingEvent & head = e->getData().get<SensorInterface::SensorHeadingEvent>();
	    printf("Head Live event:%d heading and accuracy<%f %f> timestamp<boot,gptp> %lld %lld latency(ms)=%lld odr(ms)=%lld\n",
				  HeadCount++,
				  head.getHeading(), head.getAccuracy(),
				  e->getTimestamp(), e->getGptptimestamp(),
				  (currPTPtime - e->getGptptimestamp())/1000000,
				  (e->getGptptimestamp()- head_ts)/1000000);
	    head_ts = e->getGptptimestamp();
    }
    else {
	    printf( "Sensor Live unknown sensor_id events %d\n", e->getType());
    }
}

static void  onSensorDataReadCb(vector<SensorInterface::SensorEvent> events, uint32_t count)
{
    int i =0;
    static uint64_t ts_prv_acc = 0, ts_prv_gyro = 0 , ts_prv_head = 0, ts_cur = 0;
    static uint64_t acc_sensor_ts = 0, gyro_sensor_ts = 0, head_sensor_ts = 0;
    gptpGetCurPtpTime(&ts_cur);

    int sensor_id = events[0].getSensorId();

    if (sensor_id == ACCEL_UNCALIBRATED_SENSOR_ID) {
	    acc_sensor_ts = events[count-1].getGptptimestamp();
	    printf( "Sensor ACC Live: sensor_id %d: count %d batch_delta(ms)=%lld cur_gptp_ts %lld latency(ms)=%lld\n",
			    sensor_id, count, (ts_cur - ts_prv_acc)/1000000, ts_cur, (ts_cur - acc_sensor_ts)/1000000);
	    ts_prv_acc = ts_cur;
    }
    if (sensor_id == GYRO_UNCALIBRATED_SENSOR_ID) {
	    gyro_sensor_ts = events[count-1].getGptptimestamp();
	    printf( "Sensor GYRO Live: sensor_id %d: count %d batch_delta(ms)=%lld cur_gptp_ts %lld latency(ms)=%lld\n",
			    sensor_id, count, (ts_cur - ts_prv_gyro)/1000000, ts_cur, (ts_cur - gyro_sensor_ts)/1000000);
	    ts_prv_gyro = ts_cur;
    }
    if (sensor_id == HEADING_SENSOR_ID) {
	    head_sensor_ts = events[count-1].getGptptimestamp();
	    printf( "Sensor HEAD Live: sensor_id %d: count %d batch_delta(ms)=%lld cur_gptp_ts %lld latency(ms)=%lld\n",
			    sensor_id, count, (ts_cur - ts_prv_head)/1000000, ts_cur, (ts_cur - head_sensor_ts)/1000000);
	    ts_prv_head = ts_cur;
    }

    for ( i = 0; i < count ; i++)
	    dump_live_event(&events[i]);
}

static void PrintSensorList(vector<SensorInterface::SensorList> sensor, int32_t sensor_count)
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
	   printf( "\trange: %d\n",sensor[i].getRange());
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
    int batch_count = 0, j = 0, i = 0;
    info.sender_ = 1234;

    const char* envVarName = "COMMONAPI_CONFIG"; // Specify the environment variable name
    const char* envVarValue = std::getenv(envVarName); // Get the value
    const char* envVarNameSomeIP = "VSOMEIP_CONFIGURATION"; // Specify the environment variable name
    const char* envVarValueSomeIP = std::getenv(envVarNameSomeIP); // Get the value
    const char* envCommonApi = "/usr/etc/vsomeip/SensorInterface/commonapi4someip.ini";
    const char* envSomeIP = "/usr/etc/vsomeip/SensorInterface/vsomeip-sensor_local.json";

    if(envVarValue != nullptr && envVarValueSomeIP != nullptr){
	    if(strcmp(envVarValue, envCommonApi) == 0 && strcmp(envVarValueSomeIP, envSomeIP) == 0){
	            cout << "Environment variables are set properly" << endl;
	    }
	    else {
		    cout << "Environment variables are not set properly" << endl;
	            exit(EXIT_FAILURE);
	    }
    }
    else {
            cout << "Environment variables are not set properly" << endl;
            exit(EXIT_FAILURE);
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
	myProxy->RegisterSensorClient(callStatus, resp, &info);
	for (int i=0; i < sensor_count; i++) {
	 cout << "Reconfiguring Enabled Sensors" << endl;
	 if (mSensorTrackingOption[i].state == SensorInterface::SensorState::SENSOR_ENABLE) {
	   myProxy->SensorConfig(mSensorTrackingOption[i].sensor_id, mSensorTrackingOption[i].sampling_rate,
			mSensorTrackingOption[i].batch_count, callStatus, resp, &info);
	   myProxy->SensorControl(mSensorTrackingOption[i].sensor_id, mSensorTrackingOption[i].state, callStatus, resp, &info);
	 }
	}
	break;
      }
    });

    capSubscription = myProxy->getSensorCapabilitiesEvent().subscribe(
        [&](const ::v0::com::qualcomm::qti::sensor::SensorInterface::SensorCapabilitiesMask &mask) {
	cout << "<<--Received SensorCapabilitiesMask :" << mask << endl;
	onCapabilitiesCb(mask);
	cout << "<<-----" << endl;
    });

    batchSubscription = myProxy->getSensorConfigUpdateEvent().subscribe(
       [&](int32_t sensor_id, float SamplingRate, int32_t BatchCount) {
       cout << "<<--Received SensorConfigUpdate id: " << sensor_id << " SamplingRate : " << SamplingRate << " BatchCount : " << BatchCount << endl;
       cout << "<<-------" << endl;
    });

    dataSubscription = myProxy->getSensorDataReadEvent().subscribe(
       [&](vector< ::v0::com::qualcomm::qti::sensor::SensorInterface::SensorEvent > events, uint32_t count) {
       cout << "<<--Received SensorDataRead sensor_id: " << events[0].getSensorId() << endl;
       onSensorDataReadCb(events, count);
       cout << "<<-------" << endl;
    });

    cout << "==== Register new client ====>> " << endl;
    myProxy->RegisterSensorClient(callStatus, resp, &info);
    if (callStatus != CommonAPI::CallStatus::SUCCESS) {
	    cout << "SensorClient() Remote call failed! callStatus " << (int)callStatus << endl;
    }
    parseSensorResponse(resp);
    sleep(1);

    cout << "==== Calling GetSensorList ====>> " << endl;
    myProxy->GetSensorList(callStatus, sensor, sensor_count, &info);
    if (callStatus != CommonAPI::CallStatus::SUCCESS) {
	    printf( "sensor get list ret %d \n", (int)callStatus);
	    return;
    }
    PrintSensorList(sensor, sensor_count);
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
	    cout << "==== Calling GetSensorList ====>> " << endl;
	    myProxy->GetSensorList(callStatus, sensor, sensor_count, &info);
	    if (callStatus != CommonAPI::CallStatus::SUCCESS) {
		    printf( "sensor get list ret %d \n", (int)callStatus);
	    }
	    PrintSensorList(sensor,sensor_count);
	    break;
	  case 'c':
	    cout << "==== Calling SensorConfig ====>> " << endl;
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
		myProxy->SensorConfig(sensor[i].getSensorId(), mODR[j], batch_count, callStatus, resp, &info);
		if (callStatus != CommonAPI::CallStatus::SUCCESS) {
			printf( "sensor config  failed sensor[i].sensor_id %d ret %d \n", sensor[i].getSensorId(), (int)callStatus);
		}
		else {
			parseSensorResponse(resp);
			mSensorTrackingOption[i].sensor_id = sensor[i].getSensorId();
			mSensorTrackingOption[i].sampling_rate = mODR[j];
			mSensorTrackingOption[i].batch_count = batch_count;
		}
	    }
	    break;
	  case 'a':
	    cout << "==== Calling SensorControl ====>>" <<  endl;
	    for(int i=0; i < sensor_count; i++) {
		printf( "Enable/Disable the sensor id %d\n",sensor[i].getSensorId());
		printf( " 0:SENSOR_DISABLE\n 1:SENSOR_ENABLE\n");
		memset(enable, 0, sizeof(enable)/sizeof(enable[0]));
		fgets(enable, sizeof(enable)/sizeof(enable[0]), stdin);
		state=(SensorInterface::SensorState)strtol(enable,&stopstring,10);
		cout << "sensor id:" << sensor[i].getSensorId() << " with contol value: " << (SensorInterface::SensorState)state << endl;
		myProxy->SensorControl(sensor[i].getSensorId(), state, callStatus, resp, &info);
		if (callStatus != CommonAPI::CallStatus::SUCCESS) {
			printf( "sensor control failed sensor[i].sensor_id %d ret %d \n", sensor[i].getSensorId(), (int)callStatus);
		}
		else {
			parseSensorResponse(resp);
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
