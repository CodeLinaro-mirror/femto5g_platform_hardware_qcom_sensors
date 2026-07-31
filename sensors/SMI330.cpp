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

#include "SMI330.h"

namespace bosch::sensors {

#ifdef SMI330_SCAN_TYPE
static_assert(SMI330_SCAN_TYPE >= 0 && SMI330_SCAN_TYPE <= 2,
              "SMI330_SCAN_TYPE must be 0 (IIO_BUFFER_DRDY), 1 (IIO_BUFFER_FIFO), or 2 (SYSFS)");
static constexpr ScanType kSmi330ScanType = static_cast<ScanType>(SMI330_SCAN_TYPE);
#else
static constexpr ScanType kSmi330ScanType = ScanType::IIO_BUFFER_FIFO;
#endif

#ifdef SMI330_ACCEL_RANGE
static_assert(SMI330_ACCEL_RANGE >= 2 && SMI330_ACCEL_RANGE <= 16,
              "SMI330_ACCEL_RANGE must be 2 (2G), 4 (4G), 8 (8G), or 16 (16G)");
static constexpr AccelRange kSmi330AccelRange = static_cast<AccelRange>(SMI330_ACCEL_RANGE);
#else
static constexpr AccelRange kSmi330AccelRange = AccelRange::RANGE_8G;
#endif

#ifdef SMI330_GYRO_RANGE
static_assert(SMI330_GYRO_RANGE >= 125 && SMI330_GYRO_RANGE <= 500,
              "SMI330_GYRO_RANGE must be 125 (125°/s), 250 (250°/s), 500 (500°/s)");
static constexpr GyroRange kSmi330GyroRange = static_cast<GyroRange>(SMI330_GYRO_RANGE);
#else
static constexpr GyroRange kSmi330GyroRange = GyroRange::RANGE_250_DEG_S;
#endif

Smi330Acc::Smi330Acc(const std::shared_ptr<SensorThread> sensorThread)
  : Accelerometer(sensorThread, mAccelData, kSmi330AccelRange) {
  mSensorData.driverName = "smi330";
  mSensorData.sensorName = "SMI330 BOSCH Accelerometer Sensor";
  mSensorData.propertyIdentifier = "smi330";
  mSensorData.temperatureSysfsRaw = "in_temp_raw";
  mSensorData.minDelayUs = 5000;
  mSensorData.scanType = kSmi330ScanType;
  mSensorData.maxDelayUs = 2000000;
  mSensorData.power = 0.4f;
  mSensorData.temperatureScale = 1.0f / 512;
  mSensorData.temperatureOffset = 23.0f * 512;
}

Smi330Gyro::Smi330Gyro(const std::shared_ptr<SensorThread> sensorThread)
  : Gyroscope(sensorThread, mGyroData, kSmi330GyroRange) {
  mSensorData.driverName = "smi330";
  mSensorData.sensorName = "SMI330 BOSCH Gyroscope Sensor";
  mSensorData.propertyIdentifier = "smi330";
  mSensorData.temperatureSysfsRaw = "in_temp_raw";
  mSensorData.minDelayUs = 5000;
  mSensorData.scanType = kSmi330ScanType;
  mSensorData.maxDelayUs = 2000000;
  mSensorData.power = 0.4f;
  mSensorData.temperatureScale = 1.0f / 512;
  mSensorData.temperatureOffset = 23.0f * 512;
}

}  // namespace bosch::sensors