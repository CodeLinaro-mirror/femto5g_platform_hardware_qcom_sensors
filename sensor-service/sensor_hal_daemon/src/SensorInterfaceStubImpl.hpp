/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#ifdef SENSOR_IVSS_ENABLED
#ifndef SENSORSTUBIMPL_HPP_
#define SENSORSTUBIMPL_HPP_

#include <CommonAPI/CommonAPI.hpp>
#include <v1/com/qualcomm/qti/sensor/SensorInterface.hpp>
#include <v1/com/qualcomm/qti/sensor/SensorInterfaceStub.hpp>
#include <v1/com/qualcomm/qti/sensor/SensorInterfaceStubDefault.hpp>
#include <SensorApiMsg.h>
#include <SensorApiService.h>
#include <SensorList.h>

#undef LOG_IVSS_TAG
#define LOG_IVSS_TAG "SensorSvc_IVSS:"

using namespace v1::com::qualcomm::qti::sensor;
using namespace std;

// forward declaration
class SensorApiService;

class SensorInterfaceStubImpl: public v1::com::qualcomm::qti::sensor::SensorInterfaceStubDefault {

public:
  SensorInterfaceStubImpl(SensorApiService* service);
  virtual ~SensorInterfaceStubImpl();

  /// This is the method that will be called on remote calls on the method RegisterSensorClient.
  virtual void RegisterSensorClientReq(const shared_ptr<CommonAPI::ClientId> client, RegisterSensorClientReqReply_t reply);
  /// This is the method that will be called on remote calls on the method DeRegisterSensorClient.
  virtual void DeRegisterSensorClientReq(const shared_ptr<CommonAPI::ClientId> client, DeRegisterSensorClientReqReply_t reply);
  /// This is the method that will be called on remote calls on the method GetSensorInfoT.
  virtual void GetSensorListReq(const shared_ptr<CommonAPI::ClientId> client, GetSensorListReqReply_t reply);
  /// This is the method that will be called on remote calls on the method SensorConfig.
  virtual void SensorConfigReq(const shared_ptr<CommonAPI::ClientId> client, int32_t sensorId, float samplingRate, int32_t batchCount, SensorConfigReqReply_t reply);
  /// This is the method that will be called on remote calls on the method SensorControl.
  virtual void SensorControlReq(const shared_ptr<CommonAPI::ClientId> client, int32_t sensorId, SensorInterfaceTypes::SensorStateT sensorState, SensorControlReqReply_t reply);

  SensorInterfaceTypes::SensorServiceStateMaskT  parseSensorServiceStateMaskT(SensorCapabilitiesMask mask);
  SensorInterfaceTypes::SensorReturnT            parseSensorReturnT(int res);
  vector<SensorInterfaceTypes::SensorImuEventT > parseSensorImuEventTs(sensors_event_t *e, int count);
  vector<SensorInterfaceTypes::SensorInfoT >     parseSensorInfoT(struct sensor_list *s, int sensor_count);

  void onCapabilitiesCallback(SensorCapabilitiesMask mask);
  void onSensorDataReadCb(sensors_event_t *events, int count);
  uint64_t getGptpTimeFromBootTime(uint64_t boot_time_ns);
#ifdef SENSOR_HEAD_TYPE_SUPPORT
  void onSensorHeadingDataReadCb(float heading, float accuracy, uint64_t ts);
#endif

  // name of this client
  string mClientname;
  uint32_t    mClientId;
  bool mHeadTracking;
  static mutex mIdlMutex;
  static map<int32_t, float> sensorSamplingRates;
  static map<int32_t, bool> sensorStates;
  static map<int32_t, int32_t> sensorControlRequest;
  // pointer to parent service
  SensorApiService* mService;
};

#endif // SENSORSTUBIMPL_HPP_
#endif
