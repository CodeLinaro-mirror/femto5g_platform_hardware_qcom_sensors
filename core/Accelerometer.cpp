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

#include "Accelerometer.h"

#include <unistd.h>

#include <iostream>

#include "FileHandler.h"

namespace bosch::sensors {

Accelerometer::Accelerometer(const std::shared_ptr<SensorThread> sensorThread, const AccelData& accelData,
                             AccelRange range)
  : SensorCore(sensorThread), mSensorThread(sensorThread), mAccelData(accelData), mRange(range) {
  mSensorData.type = BoschSensorType::ACCEL;
}

void Accelerometer::init(int deviceNum) {
  SensorCore::initBase(deviceNum);
  mSensorThread->init(ACCEL_IDX, deviceNum, std::bind(&Accelerometer::setOdr, this, std::placeholders::_1),
                      mSensorData.scanType);
  setScale();
  checkAndUpdateRange();
}

void Accelerometer::enableSensor(bool enable) {
  checkAndUpdateRange();
  mSensorThread->enable(ACCEL_IDX, enable, mSysfsPower, enable ? mAccelData.sysfsEnable : mAccelData.sysfsDisable);
}

void Accelerometer::setSamplingRate(int64_t samplingPeriodNs, int64_t reportLatencyNs) {
  mSensorThread->setSamplingRate(ACCEL_IDX, samplingPeriodNs, reportLatencyNs);
}

void Accelerometer::setOdr(int64_t samplingPeriodNs) {
  int64_t selectedPeriodNs = us_ns(mAccelData.odrMap.begin()->first);
  std::string sysfsOdr = mAccelData.odrMap.begin()->second;

  for (auto it = mAccelData.odrMap.rbegin(); it != mAccelData.odrMap.rend(); ++it) {
    if (us_ns(it->first) <= samplingPeriodNs) {
      selectedPeriodNs = us_ns(it->first);
      sysfsOdr = it->second;
      break;
    }
  }

  if (mSensorData.scanType == ScanType::SYSFS) {
    SensorCore::updateValidTimestampFactor(samplingPeriodNs);
  } else {
    SensorCore::updateValidTimestampFactor(selectedPeriodNs);
  }

  bosch::hwctl::WriteReadbackHandler handler(mDevice, mSysfsOdr, sysfsOdr);
}

void Accelerometer::setScale() {
  if (mRangeMap.find(mRange) == mRangeMap.end()) {
    std::cerr << "Invalid accel range for scale setting" << std::endl;
    return;
  }
  const std::string& sysfs = mRangeMap.at(mRange);

  bosch::hwctl::WriteReadbackHandler handler(mDevice, mSysfsScale, sysfs);
}

void Accelerometer::checkAndUpdateRange() {
  AccelRange range = mAccelData.defaultRange;
  bosch::hwctl::ReadHandler handler(mDevice, mSysfsScale);
  std::string scale;

  if (handler.read(scale) == 0) {
    for (const auto& [rangeKey, scaleValue] : mRangeMap) {
      if (scale == scaleValue) {
        range = rangeKey;
        break;
      }
    }

    if (range < mAccelData.minRange || range > mAccelData.maxRange) {
      range = mAccelData.defaultRange;
      std::cerr << "Invalid accel scale read from sysfs '" << scale
                << "', use default: " << mRangeMap.at(mAccelData.defaultRange) << std::endl;
    }
  } else {
    std::cerr << "Failed to read accel scale from sysfs" << std::endl;
  }

  mSensorData.range = gravityToAcceleration(static_cast<int>(range));
  mSensorData.resolution = gravityToAcceleration(static_cast<int>(range) / 32768.0f);
  mSensorThread->setResolution(ACCEL_IDX, mSensorData.resolution);
}

}  // namespace bosch::sensors