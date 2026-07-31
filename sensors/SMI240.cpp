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

#include "SMI240.h"

namespace bosch::sensors {

Smi240Acc::Smi240Acc(const std::shared_ptr<SensorThread> sensorThread)
  : SensorCore(sensorThread), mSensorThread(sensorThread) {
  mSensorData.driverName = "smi240";
  mSensorData.sensorName = "SMI240 BOSCH Accelerometer Sensor";
  mSensorData.propertyIdentifier = "smi240";
  mSensorData.temperatureSysfsRaw = "in_temp_raw";
  mSensorData.type = BoschSensorType::ACCEL;
  mSensorData.minDelayUs = 5000;
  mSensorData.maxDelayUs = 2000000;
  mSensorData.power = 5.0f;
  mSensorData.range = gravityToAcceleration(16);
  mSensorData.resolution = gravityToAcceleration(1.0f / 2000);
  mSensorData.temperatureScale = 1.0f / 256;
  mSensorData.temperatureOffset = 25.0f * 256;
  mSensorData.scanType = ScanType::SYSFS;
}

void Smi240Acc::init(int deviceNum) {
  SensorCore::initBase(deviceNum);
  mSensorThread->init(ACCEL_IDX, deviceNum,
                      std::bind(&SensorCore::updateValidTimestampFactor, this, std::placeholders::_1),
                      mSensorData.scanType);
  mSensorThread->setResolution(ACCEL_IDX, mSensorData.resolution);
}

void Smi240Acc::enableSensor(bool enable) { mSensorThread->enable(ACCEL_IDX, enable, "", ""); }

void Smi240Acc::setSamplingRate(int64_t samplingPeriodNs, int64_t reportLatencyNs) {
  mSensorThread->setSamplingRate(ACCEL_IDX, samplingPeriodNs, reportLatencyNs);
}

Smi240Gyro::Smi240Gyro(const std::shared_ptr<SensorThread> sensorThread)
  : SensorCore(sensorThread), mSensorThread(sensorThread) {
  mSensorData.driverName = "smi240";
  mSensorData.sensorName = "SMI240 BOSCH Gyroscope Sensor";
  mSensorData.propertyIdentifier = "smi240";
  mSensorData.temperatureSysfsRaw = "in_temp_raw";
  mSensorData.type = BoschSensorType::GYRO;
  mSensorData.minDelayUs = 5000;
  mSensorData.maxDelayUs = 2000000;
  mSensorData.power = 5.0f;
  mSensorData.range = degreeToRad(300);
  mSensorData.resolution = degreeToRad(1.0f / 100);
  mSensorData.temperatureScale = 1.0f / 256;
  mSensorData.temperatureOffset = 25.0f * 256;
  mSensorData.scanType = ScanType::SYSFS;
}

void Smi240Gyro::init(int deviceNum) {
  SensorCore::initBase(deviceNum);
  mSensorThread->init(GYRO_IDX, deviceNum,
                      std::bind(&SensorCore::updateValidTimestampFactor, this, std::placeholders::_1),
                      mSensorData.scanType);
  mSensorThread->setResolution(GYRO_IDX, mSensorData.resolution);
}

void Smi240Gyro::enableSensor(bool enable) { mSensorThread->enable(GYRO_IDX, enable, "", ""); }

void Smi240Gyro::setSamplingRate(int64_t samplingPeriodNs, int64_t reportLatencyNs) {
  mSensorThread->setSamplingRate(GYRO_IDX, samplingPeriodNs, reportLatencyNs);
}

}  // namespace bosch::sensors