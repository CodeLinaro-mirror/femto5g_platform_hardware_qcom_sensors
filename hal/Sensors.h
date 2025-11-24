/*
 * Copyright (C) 2025 Robert Bosch GmbH. All rights reserved.
 * Copyright (C) 2015 The Android Open Source Project
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

#ifndef SENSORS_H_
#define SENSORS_H_

#include <hardware/hardware.h>
#include <hardware/sensors.h>

#include <cstring>
#include <map>

#include "Sensor.h"
#include "SensorList.h"
#include "SensorsEventCallback.h"

class SensorContext {
public:
  // must be first, DON'T CHANGE
  sensors_poll_device_1 device;

  explicit SensorContext(const hw_module_t* module);
  SensorContext(const SensorContext&) = delete;
  SensorContext(SensorContext&&) = delete;
  SensorContext& operator=(const SensorContext&) = delete;
  SensorContext& operator=(SensorContext&&) = delete;

  size_t getSensorList(sensor_t const** list);

private:
  int close();
  int activate(int handle, int enabled);
  int setDelay(int handle, int64_t delayNs);
  int poll(sensors_event_t* data, int count);

  int batch(int handle, int64_t sampling_period_ns, int64_t max_report_latency_ns);

  int flush(int handle);

  // static wrappers
  static int CloseWrapper(hw_device_t* dev);

  static int ActivateWrapper(sensors_poll_device_t* dev, int handle, int enabled);

  static int SetDelayWrapper(sensors_poll_device_t* dev, int handle, int64_t delayNs);

  static int PollWrapper(sensors_poll_device_t* dev, sensors_event_t* data, int count);

  static int BatchWrapper(sensors_poll_device_1* dev, int handle, int flags, int64_t sampling_period_ns,
                          int64_t max_report_latency_ns);

  static int FlushWrapper(sensors_poll_device_1* dev, int handle);

  std::queue<sensors_event_t> mFifo;
  mutable std::mutex mFifoLock;
  std::condition_variable mWaitCV;

  SensorsEventCallback mCallback{mFifo, mFifoLock, mWaitCV};

  int32_t mNextHandle = 0;
  bosch::sensors::SensorList mSensorList;
  std::vector<sensor_t> mSensorListInfo;
  std::map<int32_t, std::shared_ptr<Sensor>> mSensors;
  std::map<int32_t, int64_t> mReportLatencyNs;
};

#endif  // SENSORS_H_
