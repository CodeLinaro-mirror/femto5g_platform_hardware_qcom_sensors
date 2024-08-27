/*
 * Copyright (C) 2021 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 */

#include "sensors-impl/Sensors.h"

#include <iostream>
#include <string>

#ifndef _WIN32
#include <unistd.h>
#endif
#include <chrono>
#include <variant>
#include <future>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h>
#include <string.h>
#include <dlfcn.h>
#include <sys/types.h>

#include <aidl/android/hardware/common/fmq/SynchronizedReadWrite.h>

using namespace std;
using ::aidl::android::hardware::common::fmq::MQDescriptor;
using ::aidl::android::hardware::common::fmq::SynchronizedReadWrite;
using ::aidl::android::hardware::sensors::Event;
using ::aidl::android::hardware::sensors::ISensors;
using ::aidl::android::hardware::sensors::ISensorsCallback;
using ::aidl::android::hardware::sensors::SensorInfo;
using ::ndk::ScopedAStatus;

namespace aidl {
namespace android {
namespace hardware {
namespace sensors {

void Sensors::onNewSensorsData(std::vector<SensorCoreData> &sensorData){
   bool containsWakeUpEvents = false;
   if (!mInitLock.try_lock()) {
	   return;
   }
   vector<Event> eventsList;
   Event SensorEvents = {};

   for (auto& data : sensorData) {
       SENSOR_LOGD(SENSOR_TAG "Sensor ID: %d Type: %d Timestamp: %lld, GPTP Timestamp: %lld, xyz:%f %f %f bias: %f %f %f\n", data.sensorId, data.Type ,data.timestamp, data.gptptimestamp, data.xyz[0], data.xyz[1], data.xyz[2], data.xyz[3],data.xyz[4],data.xyz[5]);

       memset(&SensorEvents, 0, sizeof(SensorEvents));
       SensorEvents.sensorType = (SensorType)data.Type;

       switch(SensorEvents.sensorType) {
	   case SensorType::ACCELEROMETER_UNCALIBRATED:
	   case SensorType::GYROSCOPE_UNCALIBRATED:
	   case SensorType::ACCELEROMETER:
	   case SensorType::GYROSCOPE: {
		   //send uncalibrated accel or gyro data
		   SensorEvents.timestamp = SensorCore_getBootTimeFromPtpTime(static_cast<uint64_t>(data.gptptimestamp));
		   SensorEvents.sensorHandle = data.sensorId;
		   SensorEvents.sensorType = (SensorType)data.Type;
		   Event::EventPayload::Uncal uncal;
		   uncal.x = data.xyz[0];
		   uncal.y = data.xyz[1];
		   uncal.z = data.xyz[2];
		   uncal.xBias = data.xyz[3];
		   uncal.yBias = data.xyz[4];
		   uncal.zBias = data.xyz[5];
		   SensorEvents.payload.set<Event::EventPayload::Tag::uncal>(uncal);
		   eventsList.push_back(SensorEvents);
		   //send calibrated accel 
		   if (SensorEvents.sensorType == SensorType::ACCELEROMETER_UNCALIBRATED) {
			   SensorEvents.sensorHandle = ACCEL_CALIBRATED_SENSOR_ID;
			   SensorEvents.sensorType = SensorType::ACCELEROMETER;
			   Event::EventPayload::Vec3 vec3;
			   vec3.x = data.xyz[0];
			   vec3.y = data.xyz[1];
			   vec3.z = data.xyz[2];
			   vec3.status = SensorStatus::ACCURACY_HIGH;
			   SensorEvents.payload.set<Event::EventPayload::Tag::vec3>(vec3);
			   eventsList.push_back(SensorEvents);
		   }
		   if (SensorEvents.sensorType == SensorType::GYROSCOPE_UNCALIBRATED) {
			   SensorEvents.sensorHandle = GYRO_CALIBRATED_SENSOR_ID;
			   SensorEvents.sensorType = SensorType::GYROSCOPE;
			   Event::EventPayload::Vec3 vec3;
			   vec3.x = data.xyz[0];
			   vec3.y = data.xyz[1];
			   vec3.z = data.xyz[2];
			   vec3.status = SensorStatus::ACCURACY_HIGH;
			   SensorEvents.payload.set<Event::EventPayload::Tag::vec3>(vec3);
			   eventsList.push_back(SensorEvents);
		   }
		   break;
	   }
	   case SensorType::HEADING: {
		   //send heading sensor data
		   SensorEvents.timestamp = SensorCore_getBootTimeFromPtpTime(static_cast<uint64_t>(data.gptptimestamp));
		   SensorEvents.sensorHandle = data.sensorId;
		   SensorEvents.sensorType = (SensorType)data.Type;
		   Event::EventPayload::Heading heading;
		   heading.heading  = data.xyz[0];
		   heading.accuracy = data.xyz[1];
		   SensorEvents.payload.set<Event::EventPayload::Tag::heading>(heading);
		   eventsList.push_back(SensorEvents);
		   break;
	   }
	   default:
		   break;
       } //end of switch

   } //end of for loop

   //send data to android framework
   postEvents(eventsList, containsWakeUpEvents);

   mInitLock.unlock();
   return;
}

ScopedAStatus Sensors::activate(int32_t in_sensorHandle, bool in_enabled) {
   SENSOR_LOGI(SENSOR_TAG "Sensor activate Call in_sensorHandle: %d, in_enabled: %d \n", in_sensorHandle, in_enabled);
   if(SensorServiceAvailable) {
	   SensorCore_acitvateSensor(in_sensorHandle, in_enabled);
           return ScopedAStatus::ok();
   }
   else {
	   SENSOR_LOGE(SENSOR_TAG "Sensor service not available to activate %d\n", in_sensorHandle);
	   return ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
   }
}

ScopedAStatus Sensors::batch(int32_t in_sensorHandle, int64_t in_samplingPeriodNs, int64_t  in_maxReportLatencyNs ) {
   SENSOR_LOGI(SENSOR_TAG "Sensor batch Call in_sensorHandle:%d in_samplingPeriodNs %lld, in_maxReportLatencyNs %lld\n", in_sensorHandle, in_samplingPeriodNs,in_maxReportLatencyNs);
   SensorCore_configSensor(in_sensorHandle, in_samplingPeriodNs, in_maxReportLatencyNs);
   return ScopedAStatus::ok();
}

ScopedAStatus Sensors::configDirectReport(int32_t /* in_sensorHandle */,
                                          int32_t /* in_channelHandle */,
                                          ISensors::RateLevel /* in_rate */,
                                          int32_t* _aidl_return) {
    SENSOR_LOGI(SENSOR_TAG "%s\n", __func__);
    *_aidl_return = EX_UNSUPPORTED_OPERATION;

    return ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ScopedAStatus Sensors::flush(int32_t in_sensorHandle) {
   SENSOR_LOGI(SENSOR_TAG "Sensor flush Call in_sensorHandle %d\n", in_sensorHandle);
   return ScopedAStatus::ok();
}

ScopedAStatus Sensors::getSensorsList(std::vector<SensorInfo>* _aidl_return) {
   SENSOR_LOGI(SENSOR_TAG "Sensor getSensorsList Call\n");
   int32_t sensorcount = 0;
   vector<SensorCoreList> sensorListVector;

   SensorCore_getSensorList(sensorListVector, &sensorcount);
   SENSOR_LOGI(SENSOR_TAG "sensor count %d\n", sensorcount);

   if(sensorcount != 0) {
     for (const auto& mSensorList : sensorListVector) {
	     auto sensor = mSensors.find(mSensorList.sensorId);
	     if (sensor != mSensors.end()) {
		  sensor->second->getSensorInfo().version = mSensorList.sensorVersion;
		  sensor->second->getSensorInfo().maxRange = mSensorList.maxRange;
		  sensor->second->getSensorInfo().minDelayUs = (1.0f/mSensorList.maxSamplingRate) * 1000000L;
		  sensor->second->getSensorInfo().maxDelayUs = (1.0f/mSensorList.odr[0])          * 1000000L;
	     }
	     if ((SensorType)mSensorList.type == SensorType::ACCELEROMETER_UNCALIBRATED) {
	       auto sensor = mSensors.find(ACCEL_CALIBRATED_SENSOR_ID);
	       if (sensor != mSensors.end()) {
		  sensor->second->getSensorInfo().version = mSensorList.sensorVersion;
		  sensor->second->getSensorInfo().maxRange = mSensorList.maxRange;
		  sensor->second->getSensorInfo().minDelayUs = (1.0f/mSensorList.maxSamplingRate) * 1000000L;
		  sensor->second->getSensorInfo().maxDelayUs = (1.0f/mSensorList.odr[0])          * 1000000L;
	       }
	     }
	     if ((SensorType)mSensorList.type == SensorType::GYROSCOPE_UNCALIBRATED) {
	       auto sensor = mSensors.find(GYRO_CALIBRATED_SENSOR_ID);
	       if (sensor != mSensors.end()) {
		  sensor->second->getSensorInfo().version = mSensorList.sensorVersion;
		  sensor->second->getSensorInfo().maxRange = mSensorList.maxRange;
		  sensor->second->getSensorInfo().minDelayUs = (1.0f/mSensorList.maxSamplingRate) * 1000000L;
		  sensor->second->getSensorInfo().maxDelayUs = (1.0f/mSensorList.odr[0])          * 1000000L;
	       }
	     }
     }
   }

   for (const auto& sensor : mSensors) {
	   SENSOR_LOGI(SENSOR_TAG "SensorInfo version:%d | name: %s | vendor: %s | maxRange: %f | resoluton: %f | sensorHandle: %d | minDelayUs: %d | maxDelayUs: %d | type:%d\n\n", sensor.second->getSensorInfo().version, sensor.second->getSensorInfo().name.c_str(), sensor.second->getSensorInfo().vendor.c_str(), sensor.second->getSensorInfo().maxRange, sensor.second->getSensorInfo().resolution, sensor.second->getSensorInfo().sensorHandle, sensor.second->getSensorInfo().minDelayUs, sensor.second->getSensorInfo().maxDelayUs, sensor.second->getSensorInfo().type);
	   _aidl_return->push_back(sensor.second->getSensorInfo());
   }

   return ScopedAStatus::ok();
}

ScopedAStatus Sensors::initialize(
        const MQDescriptor<Event, SynchronizedReadWrite>& in_eventQueueDescriptor,
        const MQDescriptor<int32_t, SynchronizedReadWrite>& in_wakeLockDescriptor,
        const std::shared_ptr<::aidl::android::hardware::sensors::ISensorsCallback>&
                in_sensorsCallback) {
    SENSOR_LOGI(SENSOR_TAG "%s\n", __func__);
    
    std::lock_guard<std::mutex> lock(mInitLock);

    ScopedAStatus result = ScopedAStatus::ok();

    mEventQueue = std::make_unique<AidlMessageQueue<Event, SynchronizedReadWrite>>(
            in_eventQueueDescriptor, true /* resetPointers */);

    // Stop the Wake Lock thread if it is currently running
    if (mReadWakeLockQueueRun.load()) {
        mReadWakeLockQueueRun = false;
        mWakeLockThread.join();
    }

    // Save a reference to the callback
    mCallback = in_sensorsCallback;

    // Ensure that any existing EventFlag is properly deleted
    deleteEventFlag();

    // Create the EventFlag that is used to signal to the framework that sensor events have been
    // written to the Event FMQ
    if (EventFlag::createEventFlag(mEventQueue->getEventFlagWord(), &mEventQueueFlag) != OK) {
        result = ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }

    // Create the Wake Lock FMQ that is used by the framework to communicate whenever WAKE_UP
    // events have been successfully read and handled by the framework.
    mWakeLockQueue = std::make_unique<AidlMessageQueue<int32_t, SynchronizedReadWrite>>(
            in_wakeLockDescriptor, true /* resetPointers */);

    if (!mCallback || !mEventQueue || !mWakeLockQueue || mEventQueueFlag == nullptr) {
        result = ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }

    // Start the thread to read events from the Wake Lock FMQ
    mReadWakeLockQueueRun = true;
    mWakeLockThread = std::thread(startReadWakeLockThread, this);
    return result;
}

ScopedAStatus Sensors::injectSensorData(const Event& in_event) {
    SENSOR_LOGI(SENSOR_TAG "%s\n", __func__);
    return ScopedAStatus::ok();
}

ScopedAStatus Sensors::registerDirectChannel(const ISensors::SharedMemInfo& /* in_mem */,
                                             int32_t* _aidl_return) {
    *_aidl_return = EX_UNSUPPORTED_OPERATION;

    return ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ScopedAStatus Sensors::setOperationMode(OperationMode in_mode) {
    SENSOR_LOGI(SENSOR_TAG "%s\n", __func__);
    return ScopedAStatus::ok();
}

ScopedAStatus Sensors::unregisterDirectChannel(int32_t /* in_channelHandle */) {
    SENSOR_LOGI(SENSOR_TAG "%s\n", __func__);
    return ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

}  // namespace sensors
}  // namespace hardware
}  // namespace android
}  // namespace aidl
