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

#ifndef ANDROID_HARDWARE_SENSORS_V1_0_SENSOR_H
#define ANDROID_HARDWARE_SENSORS_V1_0_SENSOR_H

#include <hardware/sensors.h>

#include <map>
#include <memory>
#include <mutex>

#include "ISensorHal.h"
#include "SensorsEventCallback.h"

class Sensor : public bosch::sensors::ISensorValues, public std::enable_shared_from_this<Sensor> {
public:
  Sensor(SensorsEventCallback* callback, const sensor_t& sensorInfo,
         std::shared_ptr<bosch::sensors::ISensorHal> sensor);
  ~Sensor();
  Sensor(const Sensor&) = delete;
  Sensor(Sensor&&) = delete;
  Sensor& operator=(const Sensor&) = delete;
  Sensor& operator=(Sensor&&) = delete;

  void update(const std::vector<bosch::sensors::SensorValues>& values) override;

  const sensor_t& getSensorInfo() const;
  void batch(int64_t samplingPeriodNs, int64_t maxReportLatencyNs);
  void activate(bool enable);
  bool isEnabled() const;
  int flush();

private:
  bool mIsEnabled{false};

  sensor_t mSensorInfo;

  SensorsEventCallback* mCallback;
  std::shared_ptr<bosch::sensors::ISensorHal> mSensor;

  std::mutex mMutex;
};

#endif  // ANDROID_HARDWARE_SENSORS_V1_0_SENSOR_H
