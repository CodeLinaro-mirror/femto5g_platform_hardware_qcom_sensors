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

#include "SensorInterfaceStubImpl.hpp"
#include <time.h>

#define SENSOR_TYPE_HEADING   5
#define SENSOR_ID_HEADING   3

#define NSEC_IN_ONE_SEC       (1000000000ULL)   /* nanosec in a sec */

SensorInterfaceStubImpl::SensorInterfaceStubImpl(SensorApiService* service):
	mService(service),
	mClientId(0),
	mHeadTracking(false),
	mClientname("tosomeipclient"){
}

SensorInterfaceStubImpl::~SensorInterfaceStubImpl() {
}

SensorInterface::SensorCapabilitiesMask SensorInterfaceStubImpl::parseSensorCapabilitiesMask(SensorCapabilitiesMask mask) {
    switch (mask) {
        case SHD_READY:
	      return SensorInterface::SensorCapabilitiesMask::SHD_READY;
              break;
    }
}

SensorInterface::SensorResponse SensorInterfaceStubImpl::parseSensorResponse(int res) {
   SensorInterface::SensorResponse resp = SensorInterface::SensorResponse::SENSOR_ERROR_UNKNOWN;
   switch(res) {
      case SENSOR_RESPONSE_SUCCESS:
	      resp = SensorInterface::SensorResponse::SENSOR_RESPONSE_SUCCESS;
	      break;
      case SENSOR_ERROR_CLIENT_REGISTER_FAILED:
	      resp = SensorInterface::SensorResponse::SENSOR_ERROR_CLIENT_REGISTER_FAILED;
              break;
      case SENSOR_ERROR_INVALID_CLIENT:
	      resp = SensorInterface::SensorResponse::SENSOR_ERROR_INVALID_CLIENT;
              break;
      case SENSOR_ERROR_INVALID_INPUT_PARAMETER:
	      resp = SensorInterface::SensorResponse::SENSOR_ERROR_INVALID_INPUT_PARAMETER;
              break;
      case SENSOR_ERROR_CONTROL_FAILED:
	      resp = SensorInterface::SensorResponse::SENSOR_ERROR_CONTROL_FAILED;
              break;
      case SENSOR_ERROR_CONFIG_FAILED:
	      resp = SensorInterface::SensorResponse::SENSOR_ERROR_CONFIG_FAILED;
              break;
      case SENSOR_ERROR_NO_SENSORS_FOUND:
	      resp = SensorInterface::SensorResponse::SENSOR_ERROR_NO_SENSORS_FOUND;
              break;
      case SENSOR_ERROR_TRACKING_FAILED:
	      resp = SensorInterface::SensorResponse::SENSOR_ERROR_TRACKING_FAILED;
              break;
      default: 
	      resp = SensorInterface::SensorResponse::SENSOR_ERROR_UNKNOWN;
              break;
   }
   return resp;
}

vector<SensorInterface::SensorEvent> SensorInterfaceStubImpl::parseSensorEvents(sensors_event_t *e, int count) {
  int i=0;
  uint64_t gptpTimestamp = 0;
  vector<SensorInterface::SensorEvent > idlSensorEventsData;
  SensorInterface::SensorEvent idlSensorEvents = {};
  for (i=0 ;i <count; i++){
	  //dump_sensor_event(&e[i]);
	  memset(&idlSensorEvents, 0, sizeof(idlSensorEvents));
	  gptpGetPtpTimeFromMonoTime(&gptpTimestamp, e[i].timestamp);
	  idlSensorEvents.setSensorId(e[i].sensor);
	  idlSensorEvents.setType(e[i].type);
	  idlSensorEvents.setTimestamp(e[i].timestamp);
	  idlSensorEvents.setGptptimestamp(gptpTimestamp);
	  idlSensorEvents.setData(SensorInterface::SensorUncalibratedEvent{
				  e[i].uncalibrated_accelerometer.x_uncalib,
				  e[i].uncalibrated_accelerometer.y_uncalib,
				  e[i].uncalibrated_accelerometer.z_uncalib,
				  e[i].uncalibrated_accelerometer.x_bias,
				  e[i].uncalibrated_accelerometer.y_bias,
				  e[i].uncalibrated_accelerometer.z_bias
				  });
	  idlSensorEventsData.push_back(idlSensorEvents);
	  
  }
  return idlSensorEventsData;
}

vector<SensorInterface::SensorList> SensorInterfaceStubImpl::parseSensorList(struct sensor_list *s, int sensor_count) {
  int i=0;
  vector<SensorInterface::SensorList> idlSensorListData;
  SensorInterface::SensorList idlSensorList = {};
  for (i=0 ;i < sensor_count; i++){
	  memset(&idlSensorList, 0, sizeof(idlSensorList));
	  idlSensorList.setName(s[i].name);
	  idlSensorList.setVendor(s[i].vendor);
	  idlSensorList.setSensorVersion(s[i].version);
	  idlSensorList.setSensorId(s[i].sensor_id);
	  idlSensorList.setType(s[i].type);
	  idlSensorList.setMaxSamplingRate(s[i].maxSamplingRate);
	  idlSensorList.setMinBatchCount(s[i].minBatchCount);
	  idlSensorList.setMaxBatchCount(s[i].maxBatchCount);
	  idlSensorList.setRange(s[i].range);
	  idlSensorList.setResolution(s[i].resolution);
	  idlSensorList.setMaxRange(s[i].maxRange);
	  idlSensorList.setOdr({s[i].odr[0],s[i].odr[1],s[i].odr[2],s[i].odr[3],s[i].odr[4],s[i].odr[5]});
	  idlSensorListData.push_back(idlSensorList);
  }
#ifdef SENSOR_HEAD_TYPE_SUPPORT
	  memset(&idlSensorList, 0, sizeof(idlSensorList));
	  idlSensorList.setName("Heading");
	  idlSensorList.setVendor("Heading-Sensor");
	  idlSensorList.setSensorVersion(1);
	  idlSensorList.setSensorId(SENSOR_ID_HEADING);
	  idlSensorList.setType(SENSOR_TYPE_HEADING);
	  idlSensorList.setMaxSamplingRate(100);
	  idlSensorList.setMinBatchCount(1);
	  idlSensorList.setMaxBatchCount(1);
	  idlSensorList.setRange(0);
	  idlSensorList.setResolution(0);
	  idlSensorList.setMaxRange(0);
	  idlSensorList.setOdr({100});
	  idlSensorListData.push_back(idlSensorList);
#endif
  return idlSensorListData;
}


void SensorInterfaceStubImpl::onCapabilitiesCallback(SensorCapabilitiesMask mask) {
    SensorInterface::SensorCapabilitiesMask capsMask = parseSensorCapabilitiesMask(mask);
    fireSensorCapabilitiesEvent(capsMask);
}

// This is the broadcast sensor heading events.
#ifdef SENSOR_HEAD_TYPE_SUPPORT
void SensorInterfaceStubImpl::onSensorHeadingDataReadCb(float heading, float accuracy, uint64_t ts) {
   uint64_t gptpTimestamp = 0;
   vector<SensorInterface::SensorEvent > idlSensorEventsData;
   SensorInterface::SensorEvent idlSensorEvents = {};
   if(mHeadTracking) {
    memset(&idlSensorEvents, 0, sizeof(idlSensorEvents));
    gptpGetPtpTimeFromMonoTime(&gptpTimestamp, ts);
    idlSensorEvents.setSensorId(SENSOR_ID_HEADING);
    idlSensorEvents.setType(SENSOR_TYPE_HEADING);
    idlSensorEvents.setTimestamp(ts);
    idlSensorEvents.setGptptimestamp(gptpTimestamp);
    idlSensorEvents.setData(SensorInterface::SensorHeadingEvent{heading, accuracy});
    idlSensorEventsData.push_back(idlSensorEvents);
    fireSensorDataReadEvent(idlSensorEventsData, 1);
   }
}
#endif

// This is the broadcast sensor events.
void SensorInterfaceStubImpl::onSensorDataReadCb(sensors_event_t *events, int count) {
    vector<SensorInterface::SensorEvent > idlSensorEventsData = parseSensorEvents (events, count);
    fireSensorDataReadEvent(idlSensorEventsData, count);
}

// This is the method that will be called on remote calls on the method RegisterSensorClient.
void SensorInterfaceStubImpl::RegisterSensorClient(const shared_ptr<CommonAPI::ClientId> _client, RegisterSensorClientReply_t _reply)
{
    int resp = 0;

    mClientId++;
    SENSOR_LOGI(LOG_TAG  "<<==== New SensorClient mClientId %d ==== \n", mClientId);

    SensorAPIClientRegisterReqMsg msg(mClientname.c_str(), SENSOR_CLIENT_API);
    resp = mService->newClient(&msg);
 
    SensorAPIStartTrackingReqMsg Trackmsg(mClientname.c_str());
    resp += mService->startTracking(&Trackmsg);

    SensorInterface::SensorResponse response = parseSensorResponse(resp);
    _reply(response);
}

// This is the method that will be called on remote calls on the method DeRegisterSensorClient.
void SensorInterfaceStubImpl::DeRegisterSensorClient(const shared_ptr<CommonAPI::ClientId> _client, DeRegisterSensorClientReply_t _reply)
{
    int resp = 0;

    if(mClientId != 0)
	    mClientId--;
    SENSOR_LOGI(LOG_TAG  "<<==== Delete SensorClient mClientId %d ====\n", mClientId);
    if(mClientId == 0) { 
	    SensorAPIClientDeregisterReqMsg msg(mClientname.c_str());
	    mService->deleteClient(&msg);
    }

    SensorInterface::SensorResponse response = parseSensorResponse(resp);
    _reply(response);
}

// This is the method that will be called on remote calls on the method GetSensorList.
void SensorInterfaceStubImpl::GetSensorList(const shared_ptr<CommonAPI::ClientId> _client, GetSensorListReply_t _reply)
{
    SENSOR_LOGI(LOG_TAG  "<<==== GetSensorList ==== \n");
    int count = mService->mSensorCount;

    if(mService->mSensorCount > 0) {
       vector<SensorInterface::SensorList > idlSensorList = parseSensorList(mService->mSensorList, count); 
#ifdef SENSOR_HEAD_TYPE_SUPPORT
       count++;
#endif
       _reply(idlSensorList, count);
    }
}

// This is the method that will be called on remote calls on the method SensorConfig.
void SensorInterfaceStubImpl::SensorConfig(const shared_ptr<CommonAPI::ClientId> _client, int32_t _sensorId, float _samplingRate, int32_t _batchCount, SensorConfigReply_t _reply)
{
    SENSOR_LOGI(LOG_TAG  "<<==== SensorConfig ==== Client %s sensor_id: %d SamplingRate: %f  BatchCount: %d \n", 
		    mClientname.c_str(), _sensorId, _samplingRate, _batchCount);
    int resp = 0;
    SensorInterface::SensorResponse response = 0;

    if ( _sensorId != SENSOR_ID_HEADING) {
	    SensorAPIStartBatchingReqMsg msg (mClientname.c_str(), _sensorId, _samplingRate, _batchCount, 0);
	    resp = mService->startBatching(&msg);
	    response  = parseSensorResponse(resp);
	    _reply(response);
    }
    else {
           fireSensorConfigUpdateEvent(SENSOR_ID_HEADING, 100, 1);
	   _reply(response);
    }
}

// This is the method that will be called on remote calls on the method SensorControl.
void SensorInterfaceStubImpl::SensorControl(const shared_ptr<CommonAPI::ClientId> _client, int32_t _sensorId, SensorInterface::SensorState _sensorState, SensorControlReply_t _reply)
{
    SENSOR_LOGI(LOG_TAG  "<<==== SensorControl ==== Client %s sensor_id: %d  state: %d \n", mClientname.c_str(), _sensorId, _sensorState);
    int resp = 0;
    SensorInterface::SensorResponse response = 0;

    if ( _sensorId != SENSOR_ID_HEADING) {
	    SensorAPIEnableReqMsg msg (mClientname.c_str(), _sensorId, _sensorState);
	    resp = mService->activateSensor(&msg);

	    response = parseSensorResponse(resp);
	    _reply(response);
    } else if (_sensorState == 1){ 
	    mHeadTracking = true;
    }
    _reply(response);
}
