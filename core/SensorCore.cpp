/*
 * Copyright (C) 2025 Robert Bosch GmbH
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

#include "SensorCore.h"

#include <fcntl.h>

#include <algorithm>
#include <cinttypes>
#include <iostream>
#include <sstream>

#include "FileHandler.h"

using bosch::sensors::SensorCore;

SensorCore::SensorCore(const std::shared_ptr<SensorThread> sensorThread) : mSensorThread(sensorThread) {
  mRunThread = std::thread(startThread, this);
}

SensorCore::~SensorCore() {
  mStopThread.store(true);
  mWaitCV.notify_all();
  mRunThread.join();
}

void SensorCore::initBase(int deviceNum) {
  mAvailable = true;
  mDevice = "/sys/bus/iio/devices/iio:device" + std::to_string(deviceNum) + "/";
  batch(mSensorData.minDelayUs * 1000, 0);  // Set default batch parameters
}

void SensorCore::registerObserver(const std::shared_ptr<ISensorValues>& observer) {
  std::lock_guard<std::mutex> lock(mRunMutex);
  mObservers.push_back(observer);
}

void SensorCore::removeObserver(const std::shared_ptr<ISensorValues>& observer) {
  std::lock_guard<std::mutex> lock(mRunMutex);
  mObservers.erase(
    std::remove_if(mObservers.begin(), mObservers.end(),
                   [observer](const std::weak_ptr<ISensorValues>& obs) { return obs.lock() == observer; }),
    mObservers.end());
}

void SensorCore::cleanAndCopyObservers(std::vector<std::weak_ptr<ISensorValues>>& observersCopy) {
  mObservers.erase(std::remove_if(mObservers.begin(), mObservers.end(),
                                  [&observersCopy](const std::weak_ptr<ISensorValues>& obs) {
                                    std::shared_ptr<ISensorValues> sharedObs = obs.lock();
                                    if (sharedObs) {
                                      observersCopy.push_back(obs);
                                      return false;
                                    }
                                    return true;
                                  }),
                   mObservers.end());
}

void SensorCore::activate(bool enable) {
  const SensorTypeIndex idx = mSensorData.type == BoschSensorType::ACCEL ? ACCEL_IDX : GYRO_IDX;

  if (enable) {
    mSensorThread->registerObserver(idx, shared_from_this());
  } else {
    mSensorThread->removeObserver(idx, shared_from_this());
  }

  enableSensor(enable);
}

void SensorCore::batch(int64_t samplingPeriodNs, int64_t maxReportLatencyNs) {
  {
    std::lock_guard<std::mutex> lock(mRunMutex);
    mSamplingPeriodNs = samplingPeriodNs;
    mMinValidDiffNs = samplingPeriodNs * VALID_TIMESTAMP_FACTOR;  // default value
  }
  const SensorTypeIndex idx = mSensorData.type == BoschSensorType::ACCEL ? ACCEL_IDX : GYRO_IDX;
  mSensorThread->setSamplingRate(idx, samplingPeriodNs, maxReportLatencyNs);
}

void SensorCore::updateValidTimestampFactor(int64_t odrNs) {
  std::lock_guard<std::mutex> lock(mRunMutex);
  const float factor = (mSamplingPeriodNs / odrNs) / (mSamplingPeriodNs / static_cast<float>(odrNs));
  mMinValidDiffNs = mSamplingPeriodNs * std::max(factor * VALID_TIMESTAMP_FACTOR, MIN_VALID_TIMESTAMP_FACTOR);
  std::cout << "SensorCore[" << mSensorData.sensorName << "]: Set check diff to " << mMinValidDiffNs << " ns"
            << std::endl;
}

bool SensorCore::readSensorTemperature(float* temperature) {
  if (mSensorData.temperatureSysfsRaw.empty()) return false;

  std::string data;
  bosch::hwctl::ReadHandler fileHandler(mDevice, mSensorData.temperatureSysfsRaw);

  if (0 == fileHandler.read(data)) {
    *temperature = (::atof(data.c_str()) + mSensorData.temperatureOffset) * mSensorData.temperatureScale;
    return true;
  } else {
    std::cerr << "Failed to read sensor temperature" << std::endl;
    return false;
  }
}

void SensorCore::startThread(SensorCore* sensor) { sensor->run(); }

void SensorCore::run() {
  std::unique_lock<std::mutex> lock(mRunMutex);
  while (!mStopThread) {
    mWaitCV.wait(lock, [this] { return mStopThread || !mSensorValues.empty(); });

    if (mStopThread) break;

    std::vector<SensorValues> values;
    while (!mSensorValues.empty()) {
      values.push_back(mSensorValues.front());
      mSensorValues.pop();
    }

    std::vector<std::weak_ptr<ISensorValues>> observersCopy;
    cleanAndCopyObservers(observersCopy);
    lock.unlock();

    for (const auto& obs : observersCopy) {
      if (auto observerPtr = obs.lock()) {
        observerPtr->update(values);
      }
    }

    lock.lock();
  }
}

void SensorCore::update(const std::vector<SensorValues>& values) {
  std::lock_guard<std::mutex> lock(mRunMutex);

  for (const auto& value : values) {
    const int64_t timeDiff = value.timestamp - mLastTimestampNs;
    if (timeDiff >= mMinValidDiffNs) {
      mSensorValues.push(value);
      mLastTimestampNs = value.timestamp;
    }
  }

  if (!mSensorValues.empty()) {
    mWaitCV.notify_all();
  }
}
