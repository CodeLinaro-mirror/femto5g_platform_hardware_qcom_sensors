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

#ifndef SENSORSTUBIMPL_HPP_
#define SENSORSTUBIMPL_HPP_

#include <CommonAPI/CommonAPI.hpp>
#include <v0/com/qualcomm/qti/sensor/SensorInterface.hpp>
#include <v0/com/qualcomm/qti/sensor/SensorInterfaceStub.hpp>
#include <v0/com/qualcomm/qti/sensor/SensorInterfaceStubDefault.hpp>
#include <SensorApiMsg.h>
#include <SensorApiService.h>
#include <SensorList.h>

using namespace v0::com::qualcomm::qti::sensor;
using namespace std;

// forward declaration
class SensorApiService;

class SensorInterfaceStubImpl: public v0::com::qualcomm::qti::sensor::SensorInterfaceStubDefault {

public:
  SensorInterfaceStubImpl(SensorApiService* service);
  virtual ~SensorInterfaceStubImpl();

  /// This is the method that will be called on remote calls on the method RegisterSensorClient.
  virtual void RegisterSensorClient(const shared_ptr<CommonAPI::ClientId> _client, RegisterSensorClientReply_t _reply);
  /// This is the method that will be called on remote calls on the method DeRegisterSensorClient.
  virtual void DeRegisterSensorClient(const shared_ptr<CommonAPI::ClientId> _client, DeRegisterSensorClientReply_t _reply);
  /// This is the method that will be called on remote calls on the method GetSensorList.
  virtual void GetSensorList(const shared_ptr<CommonAPI::ClientId> _client, GetSensorListReply_t _reply);
  /// This is the method that will be called on remote calls on the method SensorConfig.
  virtual void SensorConfig(const shared_ptr<CommonAPI::ClientId> _client, int32_t _sensorId, float _samplingRate, int32_t _batchCount, SensorConfigReply_t _reply);
  /// This is the method that will be called on remote calls on the method SensorControl.
  virtual void SensorControl(const shared_ptr<CommonAPI::ClientId> _client, int32_t _sensorId, SensorInterface::SensorState _sensorState, SensorControlReply_t _reply);

  SensorInterface::SensorCapabilitiesMask    parseSensorCapabilitiesMask(SensorCapabilitiesMask mask);
  SensorInterface::SensorResponse            parseSensorResponse(int res);
  vector<SensorInterface::SensorEvent >     parseSensorEvents(sensors_event_t *e, int count);
  vector<SensorInterface::SensorList >       parseSensorList(struct sensor_list *s, int sensor_count);

  void onCapabilitiesCallback(SensorCapabilitiesMask mask);
  void onSensorDataReadCb(sensors_event_t *events, int count);
#ifdef SENSOR_HEAD_TYPE_SUPPORT
  void onSensorHeadingDataReadCb(float heading, float accuracy, uint64_t ts);
#endif

  // name of this client
  string mClientname;
  uint32_t    mClientId;
  bool mHeadTracking;
  // pointer to parent service
  SensorApiService* mService;
};

#endif // SENSORSTUBIMPL_HPP_
