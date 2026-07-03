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

bool Info = true;

std::atomic<bool> mFlushPending = false;
int32_t flushHandle;

std::atomic<bool> AdditionalFlag = false;

void Sensors::onNewSensorsData(std::vector<SensorCoreData> &sensorData){
   bool containsWakeUpEvents = false;
   if (!mInitLock.try_lock()) {
	   return;
   }
   vector<Event> eventsList;
   Event SensorEvents = {};

   for (auto& data : sensorData) {
       SENSOR_LOGD(SENSOR_TAG "Sensor ID: %d Type: %d Timestamp: %" PRId64 ", GPTP Timestamp: %" PRId64 ", xyz:%f %f %f bias: %f %f %f\n", data.sensorId, data.Type, data.timestamp, data.gptptimestamp, data.xyz[0], data.xyz[1], data.xyz[2], data.xyz[3], data.xyz[4], data.xyz[5]);

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

   // Append flush-complete events (one per flush() call)
   {
       std::lock_guard<std::mutex> lk(mFlushLock);

       using MetaDataEventType =
           ::aidl::android::hardware::sensors::Event::EventPayload::MetaData::MetaDataEventType;

       for (auto& [handle, count] : mFlushPendingCount) {
           for (uint32_t i = 0; i < count; i++) {
               Event flushEvent = {};
               flushEvent.sensorHandle = handle;
               flushEvent.sensorType = SensorType::META_DATA;

               Event::EventPayload::MetaData meta = {
                   .what = MetaDataEventType::META_DATA_FLUSH_COMPLETE,
               };

               flushEvent.payload.set<Event::EventPayload::Tag::meta>(meta);
               eventsList.push_back(flushEvent);
           }
       }
       mFlushPendingCount.clear();
   }
   //send data to android framework
   postEvents(eventsList, containsWakeUpEvents);

   std::vector<int32_t> toPlace;
   {
       std::lock_guard<std::mutex> lk(mPlacementLock);
       toPlace.assign(mPlacementPending.begin(), mPlacementPending.end());
       mPlacementPending.clear();
   }

   // Call outside lock
   for (int32_t h : toPlace) {
       SensorPlacement(h);
   }

   mInitLock.unlock();
   return;
}

int Sensors::SensorCore_flush(int32_t in_sensorHandle) {
   SENSOR_LOGI(SENSOR_TAG "Set sensor flush to true\n");
   {
        std::lock_guard<std::mutex> lk(mFlushLock);
        mFlushPendingCount[in_sensorHandle]++; // one completion per flush() call
   }

   {
        std::lock_guard<std::mutex> lk(mPlacementLock);
        mPlacementPending.insert(in_sensorHandle);
   }

   return 0;
}

int Sensors::SensorPlacement(int32_t in_sensorHandle) {
   // Capture timestamps for ordering
   const uint64_t t_begin = nowBoottimeNanos();
   const uint64_t t_place = nowBoottimeNanos();
   const uint64_t t_end   = nowBoottimeNanos();

   Event event;
   AdditionalInfo info;

   // -------------------------
   // BEGIN frame
   // -------------------------
   event = {};
   info = {};
   event.sensorHandle = in_sensorHandle;
   event.sensorType   = SensorType::ADDITIONAL_INFO;
   event.timestamp    = t_begin;

   info.serial = 0;
   info.type   = AdditionalInfo::AdditionalInfoType::AINFO_BEGIN;

   AdditionalInfo::AdditionalInfoPayload::Int32Values beginPayload;
   beginPayload.values = {0};
   info.payload.set<AdditionalInfo::AdditionalInfoPayload::dataInt32>(beginPayload);

   event.payload.set<Event::EventPayload::additional>(info);
   postEvents({event}, false);
   SENSOR_LOGI(SENSOR_TAG "AINFO_BEGIN posted for handle=%d ts=%" PRIu64,
       in_sensorHandle, event.timestamp);

   // -------------------------
   // SENSOR_PLACEMENT frame
   // -------------------------
   event = {};
   info = {};
   event.sensorHandle = in_sensorHandle;
   event.sensorType   = SensorType::ADDITIONAL_INFO;
   event.timestamp    = t_place;

   info.serial = 1;
   info.type   = AdditionalInfo::AdditionalInfoType::AINFO_SENSOR_PLACEMENT;

   AdditionalInfo::AdditionalInfoPayload::FloatValues placementPayload;

   // Apply transpose of rot (sensor→Android) to get R (Android→sensor) for
   // TYPE_SENSOR_PLACEMENT
   placementPayload.values = {
       rot[0][0], rot[1][0], rot[2][0], location[0],
       rot[0][1], rot[1][1], rot[2][1], location[1],
       rot[0][2], rot[1][2], rot[2][2], location[2]
   };

   info.payload.set<AdditionalInfo::AdditionalInfoPayload::dataFloat>(placementPayload);
   event.payload.set<Event::EventPayload::additional>(info);
   postEvents({event}, false);
   SENSOR_LOGI(SENSOR_TAG "AINFO_SENSOR_PLACEMENT posted for handle=%d ts=%" PRIu64,
          in_sensorHandle, event.timestamp);

   // -------------------------
   // END frame
   // -------------------------
   event = {};
   info = {};
   event.sensorHandle = in_sensorHandle;
   event.sensorType   = SensorType::ADDITIONAL_INFO;
   event.timestamp    = t_end;

   info.serial = 2;
   info.type   = AdditionalInfo::AdditionalInfoType::AINFO_END;

   AdditionalInfo::AdditionalInfoPayload::Int32Values endPayload;
   endPayload.values = {0};
   info.payload.set<AdditionalInfo::AdditionalInfoPayload::dataInt32>(endPayload);

   event.payload.set<Event::EventPayload::additional>(info);
   postEvents({event}, false);
   SENSOR_LOGI(SENSOR_TAG "AINFO_END posted for handle=%d ts=%" PRIu64,
          in_sensorHandle, event.timestamp);

   return 0;
}

bool Sensors::isValidHandle(int32_t handle) const {
    return mSensorInfoMap.find(handle) != mSensorInfoMap.end();
}

ScopedAStatus Sensors::activate(int32_t in_sensorHandle, bool in_enabled) {
   SENSOR_LOGI(SENSOR_TAG "Sensor activate Call in_sensorHandle: %d, in_enabled: %d \n", in_sensorHandle, in_enabled);
   if (!isValidHandle(in_sensorHandle)) {
           SENSOR_LOGE(SENSOR_TAG "activate: invalid handle=%d\n", in_sensorHandle);
           return ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
   }
   if(SensorServiceAvailable) {
	   {
                std::lock_guard<std::mutex> lk(mLock);
                mEnabled[in_sensorHandle] = in_enabled;
           }
           SensorCore_acitvateSensor(in_sensorHandle, in_enabled);
           if (in_enabled) {
	    SensorPlacement(in_sensorHandle);
           }
           return ScopedAStatus::ok();
   }
   else {
           SENSOR_LOGE(SENSOR_TAG "Sensor service not available to activate %d\n", in_sensorHandle);
           return ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
   }
}

ScopedAStatus Sensors::batch(int32_t in_sensorHandle, int64_t in_samplingPeriodNs, int64_t  in_maxReportLatencyNs ) {
   SENSOR_LOGI(SENSOR_TAG "Sensor batch Call in_sensorHandle:%d in_samplingPeriodNs %" PRId64 ", in_maxReportLatencyNs %" PRId64 "\n", in_sensorHandle, in_samplingPeriodNs, in_maxReportLatencyNs);
   auto it = mSensorInfoMap.find(in_sensorHandle);
   if (it == mSensorInfoMap.end()) {
        return ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
   }
   const auto &info = it->second;

   // Convert us → ns
   const int64_t minDelayNs =
        (info.minDelayUs > 0) ? (static_cast<int64_t>(info.minDelayUs) * 1000LL) : 0;

   // Condition checks for sampling period
   if (in_samplingPeriodNs < 0 ||  in_maxReportLatencyNs < 0) {
        return ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
   }

   SensorCore_configSensor(in_sensorHandle, in_samplingPeriodNs, in_maxReportLatencyNs);
   SensorPlacement(in_sensorHandle);
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
   {
       std::lock_guard<std::mutex> lk(mLock);
       auto it = mEnabled.find(in_sensorHandle);
       bool enabled = (it != mEnabled.end()) ? it->second : false;

       if (!enabled) {
           SENSOR_LOGE(SENSOR_TAG "flush: handle=%d is inactive, returning error\n",
                       in_sensorHandle);
           // VTS wants isOk()==false here
           return ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
       }
   }

   SensorCore_flush(in_sensorHandle);
   return ScopedAStatus::ok();
}

ScopedAStatus Sensors::getSensorsList(std::vector<SensorInfo>* _aidl_return) {
   SENSOR_LOGI(SENSOR_TAG "Sensor getSensorsList Call\n");
   int32_t sensorcount = 0;
   vector<SensorCoreList> sensorListVector;

   constexpr auto isAdditionalInfoEligibleType =  [](SensorType t){
	return t == SensorType::ACCELEROMETER ||
	       t == SensorType::GYROSCOPE;
   };

   SensorCore_getSensorList(sensorListVector, &sensorcount);
   SENSOR_LOGI(SENSOR_TAG "sensor count %d\n", sensorcount);

   if(sensorcount != 0) {
     for (const auto& mSensorList : sensorListVector) {
	     auto sensor = mSensors.find(mSensorList.sensorId);
	     if (sensor != mSensors.end()) {
		  sensor->second->getSensorInfo().version = mSensorList.sensorVersion;
		  sensor->second->getSensorInfo().maxRange = mSensorList.maxRange;
		  sensor->second->getSensorInfo().minDelayUs = (1.0f/mSensorList.maxSamplingRate) * 1000000L;
		  sensor->second->getSensorInfo().maxDelayUs = (1.0f/mSensorList.maxSamplingRate) * 1000000L;

		  // ---- Advertise AdditionalInfo capability if eligible ----
                 SensorInfo& si = sensor->second->getSensorInfo();
                 const auto stype = static_cast<SensorType>(si.type);
                 if (isAdditionalInfoEligibleType(stype)) {
                     si.flags |= SensorInfo::SENSOR_FLAG_BITS_ADDITIONAL_INFO;
                 }
	     }
	     if ((SensorType)mSensorList.type == SensorType::ACCELEROMETER_UNCALIBRATED) {
	       auto sensor = mSensors.find(ACCEL_CALIBRATED_SENSOR_ID);
	       if (sensor != mSensors.end()) {
		  sensor->second->getSensorInfo().version = mSensorList.sensorVersion;
		  sensor->second->getSensorInfo().maxRange = mSensorList.maxRange;
		  sensor->second->getSensorInfo().minDelayUs = (1.0f/mSensorList.maxSamplingRate) * 1000000L;
		  sensor->second->getSensorInfo().maxDelayUs = (1.0f/mSensorList.maxSamplingRate) * 1000000L;

		  // Calibrated accel is of type ACCELEROMETER -> eligible; set flag.
                 sensor->second->getSensorInfo().flags |= SensorInfo::SENSOR_FLAG_BITS_ADDITIONAL_INFO;
	       }
	     }
	     if ((SensorType)mSensorList.type == SensorType::GYROSCOPE_UNCALIBRATED) {
	       auto sensor = mSensors.find(GYRO_CALIBRATED_SENSOR_ID);
	       if (sensor != mSensors.end()) {
		  sensor->second->getSensorInfo().version = mSensorList.sensorVersion;
		  sensor->second->getSensorInfo().maxRange = mSensorList.maxRange;
		  sensor->second->getSensorInfo().minDelayUs = (1.0f/mSensorList.maxSamplingRate) * 1000000L;
		  sensor->second->getSensorInfo().maxDelayUs = (1.0f/mSensorList.maxSamplingRate) * 1000000L;

		  // Calibrated gyro is of type GYROSCOPE -> eligible; set flag.
                 sensor->second->getSensorInfo().flags |= SensorInfo::SENSOR_FLAG_BITS_ADDITIONAL_INFO;
	       }
	     }
	     if ((SensorType)mSensorList.type == SensorType::HEADING){
                    sensor->second->getSensorInfo().maxRange = 2.0f * M_PI;
             }
     }
   }
   else {
       for (auto& kv : mSensors) {
          SensorInfo& si = kv.second->getSensorInfo();
          const auto stype = static_cast<SensorType>(si.type);
          if (stype == SensorType::ACCELEROMETER ||
             stype == SensorType::GYROSCOPE    ||
             stype == SensorType::MAGNETIC_FIELD) {
             si.flags |= SensorInfo::SENSOR_FLAG_BITS_ADDITIONAL_INFO;
          }
       }
   }


   for (const auto& sensor : mSensors) {
	   const SensorInfo& info = sensor.second->getSensorInfo();
	   mSensorInfoMap[info.sensorHandle] = info;
	   SENSOR_LOGI(SENSOR_TAG "SensorInfo version:%d | name: %s | vendor: %s | maxRange: %f | resoluton: %f | sensorHandle: %d | minDelayUs: %d | maxDelayUs: %d | type:%d | flags:0x%x\n\n", sensor.second->getSensorInfo().version, sensor.second->getSensorInfo().name.c_str(), sensor.second->getSensorInfo().vendor.c_str(), sensor.second->getSensorInfo().maxRange, sensor.second->getSensorInfo().resolution, sensor.second->getSensorInfo().sensorHandle, sensor.second->getSensorInfo().minDelayUs, sensor.second->getSensorInfo().maxDelayUs, sensor.second->getSensorInfo().type, sensor.second->getSensorInfo().flags);
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
    switch (in_mode) {
            case ISensors::OperationMode::NORMAL:
                           return ScopedAStatus::ok();

            case ISensors::OperationMode::DATA_INJECTION:
                           return ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);

            default:
                           return ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
}

ScopedAStatus Sensors::unregisterDirectChannel(int32_t /* in_channelHandle */) {
    SENSOR_LOGI(SENSOR_TAG "%s\n", __func__);
    return ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

}  // namespace sensors
}  // namespace hardware
}  // namespace android
}  // namespace aidl
