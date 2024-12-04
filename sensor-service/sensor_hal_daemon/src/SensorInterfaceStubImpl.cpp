/*
Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the
disclaimer below) provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

    * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifdef SENSOR_IVSS_ENABLED
#include "SensorInterfaceStubImpl.hpp"
#include <time.h>

#define SENSOR_TYPE_HEADING   42
#define SENSOR_ID_HEADING   5

#define NSEC_IN_ONE_SEC       (1000000000ULL)   /* nanosec in a sec */

mutex SensorInterfaceStubImpl::mIdlMutex;
map<int32_t, float> SensorInterfaceStubImpl::sensorSamplingRates;
map<int32_t, bool> SensorInterfaceStubImpl::sensorStates;
map<int32_t, int32_t> SensorInterfaceStubImpl::sensorControlRequest;

SensorInterfaceStubImpl::SensorInterfaceStubImpl(SensorApiService* service):
	mService(service),
	mClientId(0),
	mHeadTracking(false),
	mClientname("tosomeipclient"){
}

SensorInterfaceStubImpl::~SensorInterfaceStubImpl() {
}

uint64_t SensorInterfaceStubImpl::getGptpTimeFromBootTime(uint64_t boot_time_ns) {
   uint64_t gptpTimestamp;
   if (!mService->GptpInitialized && gptpInit()) {
	   SENSOR_LOGI(LOG_IVSS_TAG "GPTP init success \n");
	   mService->GptpInitialized = true;
   }
   (void)gptpGetPtpTimeFromMonoTime(&gptpTimestamp, boot_time_ns);
   return gptpTimestamp;
}

SensorInterfaceTypes::SensorServiceStateMaskT SensorInterfaceStubImpl::parseSensorServiceStateMaskT(SensorCapabilitiesMask mask) {
    switch (mask) {
        case SHD_READY:
	      return SensorInterfaceTypes::SensorServiceStateMaskT::SENSOR_SERVICE_STATE_MASK_READY;
              break;
	default:
	      return SensorInterfaceTypes::SensorServiceStateMaskT::SENSOR_SERVICE_STATE_MASK_UNKNOWN;
              break;
    }
}

SensorInterfaceTypes::SensorReturnT SensorInterfaceStubImpl::parseSensorReturnT(int res) {
   SensorInterfaceTypes::SensorReturnT resp = SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_UNKNOWN;
   switch(res) {
      case SENSOR_RESPONSE_SUCCESS:
	      resp = SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_SUCCESS;
	      break;
      case SENSOR_ERROR_CLIENT_REGISTER_FAILED:
	      resp = SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_CLIENT_REGISTER_FAILED;
              break;
      case SENSOR_ERROR_INVALID_CLIENT:
	      resp = SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_INVALID_CLIENT;
              break;
      case SENSOR_ERROR_INVALID_INPUT_PARAMETER:
	      resp = SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_INVALID_INPUT_PARAMETER;
              break;
      case SENSOR_ERROR_CONTROL_FAILED:
	      resp = SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_CONTROL_FAILED;
              break;
      case SENSOR_ERROR_CONFIG_FAILED:
	      resp = SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_CONFIG_FAILED;
              break;
      case SENSOR_ERROR_NO_SENSORS_FOUND:
	      resp = SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_NO_SENSORS_FOUND;
              break;
      case SENSOR_ERROR_TRACKING_FAILED:
	      resp = SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_TRACKING_FAILED;
              break;
      default: 
	      resp = SensorInterfaceTypes::SensorReturnT::SENSOR_RETURN_ERROR_UNKNOWN;
              break;
   }
   return resp;
}

vector<SensorInterfaceTypes::SensorImuEventT> SensorInterfaceStubImpl::parseSensorImuEventTs(sensors_event_t *e, int count) {
  int i=0;
  uint64_t gptpTimestamp = 0;
  vector<SensorInterfaceTypes::SensorImuEventT > idlSensorImuEventTsData;
  SensorInterfaceTypes::SensorImuEventT idlSensorImuEventTs = {};
  for (i=0 ;i <count; i++){
	  //dump_sensor_event(&e[i]);
	  (void)memset(&idlSensorImuEventTs, 0, sizeof(idlSensorImuEventTs));
	  gptpTimestamp = getGptpTimeFromBootTime(e[i].timestamp);
	  idlSensorImuEventTs.setSensorId(e[i].sensor);
	  idlSensorImuEventTs.setType(e[i].type);
	  idlSensorImuEventTs.setTimestamp(e[i].timestamp);
	  idlSensorImuEventTs.setGptpTimestamp(gptpTimestamp);
	  idlSensorImuEventTs.setData(SensorInterfaceTypes::SensorUncalibratedEventT{
				  e[i].uncalibrated_accelerometer.x_uncalib,
				  e[i].uncalibrated_accelerometer.y_uncalib,
				  e[i].uncalibrated_accelerometer.z_uncalib,
				  e[i].uncalibrated_accelerometer.x_bias,
				  e[i].uncalibrated_accelerometer.y_bias,
				  e[i].uncalibrated_accelerometer.z_bias
				  });
	  idlSensorImuEventTsData.push_back(idlSensorImuEventTs);
	  
  }
  return idlSensorImuEventTsData;
}

vector<SensorInterfaceTypes::SensorInfoT> SensorInterfaceStubImpl::parseSensorInfoT(struct sensor_list *s, int sensor_count) {
  int i=0;
  vector<SensorInterfaceTypes::SensorInfoT> idlSensorInfoTData;
  SensorInterfaceTypes::SensorInfoT idlSensorInfoT = {};
  for (i=0 ;i < sensor_count; i++){
	  (void)memset(&idlSensorInfoT, 0, sizeof(idlSensorInfoT));
	  idlSensorInfoT.setName(s[i].name);
	  idlSensorInfoT.setVendor(s[i].vendor);
	  idlSensorInfoT.setSensorVersion(s[i].version);
	  idlSensorInfoT.setSensorId(s[i].sensor_id);
	  idlSensorInfoT.setType(s[i].type);
	  idlSensorInfoT.setMaxSamplingRate(s[i].maxSamplingRate);
	  idlSensorInfoT.setMinBatchCount(s[i].minBatchCount);
	  idlSensorInfoT.setMaxBatchCount(s[i].maxBatchCount);
	  idlSensorInfoT.setResolution(s[i].resolution);
	  idlSensorInfoT.setMaxRange(s[i].maxRange);
	  idlSensorInfoT.setOdr({s[i].odr[0],s[i].odr[1],s[i].odr[2],s[i].odr[3],s[i].odr[4],s[i].odr[5]});
	  idlSensorInfoTData.push_back(idlSensorInfoT);
  }
#ifdef SENSOR_HEAD_TYPE_SUPPORT
	  (void)memset(&idlSensorInfoT, 0, sizeof(idlSensorInfoT));
	  idlSensorInfoT.setName("Heading");
	  idlSensorInfoT.setVendor("Qcom");
	  idlSensorInfoT.setSensorVersion(1);
	  idlSensorInfoT.setSensorId(SENSOR_ID_HEADING);
	  idlSensorInfoT.setType(SENSOR_TYPE_HEADING);
	  idlSensorInfoT.setMaxSamplingRate(100);
	  idlSensorInfoT.setMinBatchCount(1);
	  idlSensorInfoT.setMaxBatchCount(1);
	  idlSensorInfoT.setResolution(0);
	  idlSensorInfoT.setMaxRange(0);
	  idlSensorInfoT.setOdr({100});
	  idlSensorInfoTData.push_back(idlSensorInfoT);
#endif
  return idlSensorInfoTData;
}


void SensorInterfaceStubImpl::onCapabilitiesCallback(SensorCapabilitiesMask mask) {
    SensorInterfaceTypes::SensorServiceStateMaskT capsMask = parseSensorServiceStateMaskT(mask);
    fireSensorCapabilitiesEvent(capsMask);
}

// This is the broadcast sensor heading events.
#ifdef SENSOR_HEAD_TYPE_SUPPORT
void SensorInterfaceStubImpl::onSensorHeadingDataReadCb(float heading, float accuracy, uint64_t ts) {
   uint64_t gptpTimestamp = 0;
   vector<SensorInterfaceTypes::SensorHeadEventT > idlSensorHeadEventTsData;
   SensorInterfaceTypes::SensorHeadEventT idlSensorHeadEventTs = {};
   if(mHeadTracking) {
    (void)memset(&idlSensorHeadEventTs, 0, sizeof(idlSensorHeadEventTs));
    gptpTimestamp = getGptpTimeFromBootTime(ts);
    idlSensorHeadEventTs.setSensorId(SENSOR_ID_HEADING);
    idlSensorHeadEventTs.setType(SENSOR_TYPE_HEADING);
    idlSensorHeadEventTs.setTimestamp(ts);
    idlSensorHeadEventTs.setGptpTimestamp(gptpTimestamp);
    idlSensorHeadEventTs.setData(SensorInterfaceTypes::SensorHeadingEventT{heading, accuracy});
    idlSensorHeadEventTsData.push_back(idlSensorHeadEventTs);
    fireSensorHeadingDataReadEvent(idlSensorHeadEventTsData, 1);
   }
}
#endif

// This is the broadcast sensor events.
void SensorInterfaceStubImpl::onSensorDataReadCb(sensors_event_t *events, int count) {
    vector<SensorInterfaceTypes::SensorImuEventT > idlSensorImuEventTsData = parseSensorImuEventTs (events, count);
    fireSensorImuDataReadEvent(idlSensorImuEventTsData, count);
}

// This is the method that will be called on remote calls on the method RegisterSensorClientReq.
void SensorInterfaceStubImpl::RegisterSensorClientReq(const shared_ptr<CommonAPI::ClientId> client, RegisterSensorClientReqReply_t reply)
{
    lock_guard<mutex> lock(mIdlMutex);
    int resp = 0;

    mClientId++;
    SENSOR_LOGI(LOG_IVSS_TAG  "<<==== New SensorClient mClientId %d ==== \n", mClientId);

    SensorAPIClientRegisterReqMsg msg(mClientname.c_str(), SENSOR_CLIENT_API);
    resp = mService->newClient(&msg);
 
    SensorAPIStartTrackingReqMsg Trackmsg(mClientname.c_str());
    resp += mService->startTracking(&Trackmsg);

    SensorInterfaceTypes::SensorReturnT response = parseSensorReturnT(resp);
    reply(response);
}

// This is the method that will be called on remote calls on the method DeRegisterSensorClientReq.
void SensorInterfaceStubImpl::DeRegisterSensorClientReq(const shared_ptr<CommonAPI::ClientId> client, DeRegisterSensorClientReqReply_t reply)
{
    lock_guard<mutex> lock(mIdlMutex);
    int resp = 0;

    if(mClientId != 0)
	    mClientId--;
    SENSOR_LOGI(LOG_IVSS_TAG  "<<==== Delete SensorClient mClientId %d ====\n", mClientId);
    if(mClientId == 0) { 
            SensorAPIClientDeregisterReqMsg msg(mClientname.c_str());
	    mService->deleteClient(&msg);
	    /*reset all flags when ivss client is removed from shd**/
	    mHeadTracking = false;
	    for(int i=0; i< mService->mSensorCount; i++) {
		    sensorStates[mService->mSensorList[i].sensor_id] = false;
		    sensorSamplingRates[mService->mSensorList[i].sensor_id] = 0;
		    sensorControlRequest[mService->mSensorList[i].sensor_id] = 0;
	    }
    }

    SensorInterfaceTypes::SensorReturnT response = parseSensorReturnT(resp);
    reply(response);
}

// This is the method that will be called on remote calls on the method GetSensorListReq.
void SensorInterfaceStubImpl::GetSensorListReq(const shared_ptr<CommonAPI::ClientId> client, GetSensorListReqReply_t reply)
{
    lock_guard<mutex> lock(mIdlMutex);
    SENSOR_LOGI(LOG_IVSS_TAG  "<<==== GetSensorInfoTReq ==== \n");
    int count = mService->mSensorCount;

    if(mService->mSensorCount > 0) {
       vector<SensorInterfaceTypes::SensorInfoT > idlSensorInfoT = parseSensorInfoT(mService->mSensorList, count); 
#ifdef SENSOR_HEAD_TYPE_SUPPORT
       count++;
#endif
       reply(idlSensorInfoT, count);
    }
}

// This is the method that will be called on remote calls on the method SensorConfig.
void SensorInterfaceStubImpl::SensorConfigReq(const shared_ptr<CommonAPI::ClientId> client, int32_t sensorId, float samplingRate, int32_t batchCount, SensorConfigReqReply_t reply)
{
    lock_guard<mutex> lock(mIdlMutex);
    SENSOR_LOGI(LOG_IVSS_TAG  "<<==== SensorConfig ==== Client %s sensor_id: %d SamplingRate: %f  BatchCount: %d \n", 
		    mClientname.c_str(), sensorId, samplingRate, batchCount);
    int resp = 0;
    SensorInterfaceTypes::SensorReturnT response = 1;

    /*Handle Accel and gyro config request*/
    if ( sensorId != SENSOR_ID_HEADING) {
	    /*Configure sensor if required more than already configured rate*/
	    if (samplingRate > sensorSamplingRates[sensorId]) {
		    SENSOR_LOGI(LOG_IVSS_TAG "<= Switch sensor config id:%d from %fhz to %fhz\n",sensorId, sensorSamplingRates[sensorId], samplingRate);
		    SensorAPIStartBatchingReqMsg msg (mClientname.c_str(), sensorId, samplingRate, batchCount, 1);
		    resp = mService->startBatching(&msg);
		    /*store the configured samplingrate for respective sensor id if config success*/
		    if(resp == 0)
			    sensorSamplingRates[sensorId] = samplingRate;
		    /*re-enable sensor if already enabled as config is updated*/
		    if(sensorStates[sensorId]) {
			    SensorAPIEnableReqMsg msg (mClientname.c_str(), sensorId, 1);
			    (void)mService->activateSensor(&msg);
	            }
	    }
	    response  = parseSensorReturnT(resp);
	    reply(response);
    }
    /*Handle heading config request*/
    else {
           fireSensorConfigUpdateEvent(SENSOR_ID_HEADING, 100, 1);
	    response  = parseSensorReturnT(resp);
	   reply(response);
    }
}

// This is the method that will be called on remote calls on the method SensorControl.
void SensorInterfaceStubImpl::SensorControlReq(const shared_ptr<CommonAPI::ClientId> client, int32_t sensorId, SensorInterfaceTypes::SensorStateT sensorState, SensorControlReqReply_t reply)
{
    lock_guard<mutex> lock(mIdlMutex);
    SENSOR_LOGI(LOG_IVSS_TAG  "<<==== SensorControl === Client %s sensor_id:%d state:%d\n",mClientname.c_str(), sensorId, static_cast<int>(sensorState));
    int resp = 0, enable = 0;
    SensorInterfaceTypes::SensorReturnT response = 1;

    if(sensorState == SensorInterfaceTypes::SensorStateT::SENSOR_STATE_ENABLE) 
	    enable = 1;
    else
	    enable = 0;

    /*Handle Accel and gyro enable/disable request*/
    if ( sensorId != SENSOR_ID_HEADING) {
       if (enable == 1) {
	    SensorAPIEnableReqMsg msg (mClientname.c_str(), sensorId, enable);
	    resp = mService->activateSensor(&msg);
	    if(resp == 0) {
		    sensorStates[sensorId] = true;
		    sensorControlRequest[sensorId]++;
		    SENSOR_LOGI(LOG_IVSS_TAG  "<= Sensor enable request id %d count %d \n", sensorId, sensorControlRequest[sensorId]);
	    }
       }
       if (enable == 0) {
	    sensorControlRequest[sensorId]--;
	    SENSOR_LOGI(LOG_IVSS_TAG  "<= Sensor disable request id %d count %d \n", sensorId, sensorControlRequest[sensorId]);
	    if (sensorControlRequest[sensorId] <= 0)  {
	       SensorAPIEnableReqMsg msg (mClientname.c_str(), sensorId, enable);
	       resp = mService->activateSensor(&msg);
	       if(resp == 0) {
		       sensorStates[sensorId] = false;
		       sensorSamplingRates[sensorId] = 0;
	       }
	    }
       }
       response = parseSensorReturnT(resp);
       reply(response);
    } 
    /*Handle heading enable/disable request*/
    else {
	    if (enable == 1){ 
		    mService->EnableHeadingSensor();
		    mHeadTracking = true;
		    sensorStates[sensorId] = true;
		    sensorControlRequest[sensorId]++;
	    }
	    else { 
		    sensorControlRequest[sensorId]--;
		    if (sensorControlRequest[sensorId] <= 0) {
		       mService->DisableHeadingSensor();
		       mHeadTracking = false;
		       sensorStates[sensorId] = false;
		       sensorSamplingRates[sensorId] = 0;
		    }
	    }
	    response = parseSensorReturnT(resp);
	    reply(response);
    }
}
#endif
