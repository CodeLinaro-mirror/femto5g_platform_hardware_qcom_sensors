/*
 * Copyright (C) 2025 Robert Bosch GmbH. All rights reserved.
 * Copyright (C) 2017 The Android Open Source Project
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

#include "Sensors.h"

#include <iostream>

SensorContext::SensorContext(const hw_module_t* module) {
  memset(&device, 0, sizeof(device));

  device.common.tag = HARDWARE_DEVICE_TAG;
  device.common.version = SENSORS_DEVICE_API_VERSION_1_3;
  device.common.module = const_cast<hw_module_t*>(module);
  device.common.close = CloseWrapper;
  device.activate = ActivateWrapper;
  device.setDelay = SetDelayWrapper;
  device.poll = PollWrapper;
  device.batch = BatchWrapper;
  device.flush = FlushWrapper;

  for (const auto& sensor : mSensorList.getAvailableSensors()) {
    const auto& data = sensor->getSensorData();
    sensor_t sensorInfo{};
    sensorInfo.handle = mNextHandle++;
    sensorInfo.name = data.sensorName.c_str();
    sensorInfo.vendor = data.vendor.c_str();
    sensorInfo.type = data.type;
    sensorInfo.stringType = "";
    sensorInfo.version = 1;
    sensorInfo.fifoReservedEventCount = 0;
    sensorInfo.fifoMaxEventCount = 0;
    sensorInfo.requiredPermission = "";
    sensorInfo.flags = SENSOR_FLAG_CONTINUOUS_MODE;
    sensorInfo.minDelay = data.minDelayUs;
    sensorInfo.maxDelay = data.maxDelayUs;
    sensorInfo.power = data.power;
    sensorInfo.maxRange = data.range;
    sensorInfo.resolution = data.resolution;

    std::shared_ptr<Sensor> halSensor = std::make_shared<Sensor>(&mCallback, sensorInfo, sensor);
    mSensors[halSensor->getSensorInfo().handle] = halSensor;
    mSensorListInfo.push_back(sensorInfo);
    std::cout << "AddSensor[" << halSensor->getSensorInfo().handle << "] " << halSensor->getSensorInfo().name
              << std::endl;
  }
}

int SensorContext::close() {
  delete this;
  return 0;
}

int SensorContext::activate(int handle, int enabled) {
  auto sensor = mSensors.find(handle);
  if (sensor != mSensors.end()) {
    sensor->second->activate(enabled != 0);
    return 0;
  }
  return -1;
}

int SensorContext::setDelay(int handle, int64_t delayNs) {
  auto sensor = mSensors.find(handle);
  if (sensor != mSensors.end()) {
    sensor->second->batch(delayNs, 0);
    return 0;
  }
  return -1;
}

int SensorContext::poll(sensors_event_t* data, int count) {
  std::unique_lock<std::mutex> lock(mFifoLock);
  mWaitCV.wait(lock, [&] { return !mFifo.empty(); });
  const size_t eventsToRead = std::min(static_cast<size_t>(count), mFifo.size());
  for (size_t i = 0; i < eventsToRead; i++) {
    memcpy(&data[i], &mFifo.front(), sizeof(sensors_event_t));
    mFifo.pop();
  }
  return eventsToRead;
}

int SensorContext::batch(int handle, int64_t sampling_period_ns, int64_t max_report_latency_ns) {
  int64_t usedLatency = max_report_latency_ns;
  if (max_report_latency_ns > 0) {
    mReportLatencyNs[handle] = max_report_latency_ns;
    for (auto sensor : mSensors) {
      if (sensor.second->isEnabled()) {
        auto latency = mReportLatencyNs.find(handle);
        if (latency != mReportLatencyNs.end()) {
          usedLatency = std::min(usedLatency, latency->second);
        }
      }
    }
  }

  auto sensor = mSensors.find(handle);
  if (sensor != mSensors.end()) {
    sensor->second->batch(sampling_period_ns, usedLatency);
    return 0;
  }
  return -1;
}

int SensorContext::flush(int handle) {
  auto sensor = mSensors.find(handle);
  if (sensor != mSensors.end()) {
    return sensor->second->flush();
  }
  return 0;
}

// static
int SensorContext::CloseWrapper(hw_device_t* dev) { return reinterpret_cast<SensorContext*>(dev)->close(); }

// static
int SensorContext::ActivateWrapper(sensors_poll_device_t* dev, int handle, int enabled) {
  return reinterpret_cast<SensorContext*>(dev)->activate(handle, enabled);
}

// static
int SensorContext::SetDelayWrapper(sensors_poll_device_t* dev, int handle, int64_t delayNs) {
  return reinterpret_cast<SensorContext*>(dev)->setDelay(handle, delayNs);
}

// static
int SensorContext::PollWrapper(sensors_poll_device_t* dev, sensors_event_t* data, int count) {
  return reinterpret_cast<SensorContext*>(dev)->poll(data, count);
}

// static
int SensorContext::BatchWrapper(sensors_poll_device_1* dev, int handle, int flags, int64_t sampling_period_ns,
                                int64_t max_report_latency_ns) {
  (void)flags;
  return reinterpret_cast<SensorContext*>(dev)->batch(handle, sampling_period_ns, max_report_latency_ns);
}

// static
int SensorContext::FlushWrapper(sensors_poll_device_1* dev, int handle) {
  return reinterpret_cast<SensorContext*>(dev)->flush(handle);
}

size_t SensorContext::getSensorList(sensor_t const** list) {
  *list = mSensorListInfo.data();
  return mSensorListInfo.size();
}

////////////////////////////////////////////////////////////////////////////////

static sensor_t const* sensor_list;
static size_t sensor_list_lentgh;

static int open_sensors(const hw_module_t* module, const char* /*unused*/, hw_device_t** dev) {
  SensorContext* ctx = new SensorContext(module);
  sensor_list_lentgh = ctx->getSensorList(&sensor_list);
  *dev = &ctx->device.common;
  return 0;
}

static hw_module_methods_t sensors_module_methods = {.open = open_sensors};

static int get_sensors_list(sensors_module_t*, sensor_t const** list) {
  *list = sensor_list;
  return sensor_list_lentgh;
}

static int set_operation_mode(unsigned int mode) { return mode ? -EINVAL : 0; }

sensors_module_t HAL_MODULE_INFO_SYM = {
  .common =
    {
      .tag = HARDWARE_MODULE_TAG,
      .version_major = 1,
      .version_minor = 0,
      .id = SENSORS_HARDWARE_MODULE_ID,
      .name = "Bosch Sensor module",
      .author = "Robert Bosch,GmbH",
      .methods = &sensors_module_methods,
      .dso = NULL,
      .reserved = {0},
    },
  .get_sensors_list = get_sensors_list,
  .set_operation_mode = set_operation_mode,
};
