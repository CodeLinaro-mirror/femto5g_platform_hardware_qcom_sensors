/*
 * Copyright (C) 2025 Robert Bosch GmbH. All rights reserved.
 * Copyright (C) 2023 The Android Open Source Project
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
 */

#include "Sensor.h"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <iostream>
#include <memory>

Sensor::Sensor(SensorsEventCallback* callback, const sensor_t& sensorInfo,
               std::shared_ptr<bosch::sensors::ISensorHal> sensor)
  : mSensorInfo(sensorInfo), mCallback(callback), mSensor(sensor) {}

Sensor::~Sensor() { mIsEnabled = false; }

const sensor_t& Sensor::getSensorInfo() const { return mSensorInfo; }

void Sensor::batch(int64_t samplingPeriodNs, int64_t maxReportLatencyNs) {
  std::cout << "Sensor batch " << mSensorInfo.name << " " << samplingPeriodNs << " " << maxReportLatencyNs << std::endl;

  const int64_t samplingPeriod = std::clamp(samplingPeriodNs, static_cast<int64_t>(mSensorInfo.minDelay) * 1000,
                                            static_cast<int64_t>(mSensorInfo.maxDelay) * 1000);

  mSensor->batch(samplingPeriod, maxReportLatencyNs);
}

void Sensor::activate(bool enable) {
  std::cout << "Sensor activate " << mSensorInfo.name << " " << enable << std::endl;

  std::lock_guard<std::mutex> lock(mMutex);
  if (mIsEnabled != enable) {
    mIsEnabled = enable;
    mSensor->activate(enable);
    if (enable) {
      mSensor->registerObserver(shared_from_this());
    } else {
      mSensor->removeObserver(shared_from_this());
    }
  }
}

bool Sensor::isEnabled() const { return mIsEnabled; }

int Sensor::flush() {
  if (!mIsEnabled) {
    return -1;
  }

  // Note: If a sensor supports batching, write all of the currently batched
  // events for the sensor to the Event FMQ prior to writing the flush complete
  // event.
  sensors_event_t ev;
  ev.sensor = mSensorInfo.handle;
  ev.type = SENSOR_TYPE_META_DATA;
  ev.meta_data.what = META_DATA_FLUSH_COMPLETE;
  std::vector<sensors_event_t> evs{ev};
  mCallback->postEvents(evs);

  return 0;
}

void Sensor::update(const std::vector<bosch::sensors::SensorValues>& values) {
  std::vector<sensors_event_t> events;
  const size_t xyzLength = 3;

  std::unique_lock<std::mutex> lock(mMutex);

  for (const auto& value : values) {
    if (static_cast<int>(value.type) == static_cast<int>(mSensorInfo.type)) {
      sensors_event_t event{};
      event.sensor = mSensorInfo.handle;
      event.type = mSensorInfo.type;
      event.timestamp = value.timestamp;

      if (value.data.size() == xyzLength) {
        if (mSensorInfo.type == SENSOR_TYPE_ACCELEROMETER) {
          event.acceleration.x = value.data[0];
          event.acceleration.y = value.data[1];
          event.acceleration.z = value.data[2];
          event.acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;
        } else if (mSensorInfo.type == SENSOR_TYPE_GYROSCOPE) {
          event.gyro.x = value.data[0];
          event.gyro.y = value.data[1];
          event.gyro.z = value.data[2];
          event.gyro.status = SENSOR_STATUS_ACCURACY_HIGH;
        }
      } else {
        std::cerr << "Read failed: " << static_cast<size_t>(value.data.size()) << std::endl;
        continue;
      }

      if (mIsEnabled) {
        events.push_back(event);
      }
    }
  }

  lock.unlock();

  if (!events.empty()) {
    mCallback->postEvents(events);
  }
}
