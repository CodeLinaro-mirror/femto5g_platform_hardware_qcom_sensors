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

#include "SMI230.h"

namespace bosch::sensors {

#ifdef SMI230_SCAN_TYPE
static_assert(SMI230_SCAN_TYPE >= 0 && SMI230_SCAN_TYPE <= 2,
              "SMI230_SCAN_TYPE must be 0 (IIO_BUFFER_DRDY), 1 (IIO_BUFFER_FIFO), or 2 (SYSFS)");
static constexpr ScanType kSmi230ScanType = static_cast<ScanType>(SMI230_SCAN_TYPE);
#else
static constexpr ScanType kSmi230ScanType = ScanType::IIO_BUFFER_DRDY;
#endif

#ifdef SMI230_ACCEL_RANGE
static_assert(SMI230_ACCEL_RANGE >= 2 && SMI230_ACCEL_RANGE <= 16,
              "SMI230_ACCEL_RANGE must be 2 (2G), 4 (4G), 8 (8G), or 16 (16G)");
static constexpr AccelRange kSmi230AccelRange = static_cast<AccelRange>(SMI230_ACCEL_RANGE);
#else
static constexpr AccelRange kSmi230AccelRange = AccelRange::RANGE_4G;
#endif

#ifdef SMI230_GYRO_RANGE
static_assert(SMI230_GYRO_RANGE >= 125 && SMI230_GYRO_RANGE <= 2000,
              "SMI230_GYRO_RANGE must be 125 (125°/s), 250 (250°/s), 500 (500°/s), 1000 (1000°/s), or 2000 (2000°/s)");
static constexpr GyroRange kSmi230GyroRange = static_cast<GyroRange>(SMI230_GYRO_RANGE);
#else
static constexpr GyroRange kSmi230GyroRange = GyroRange::RANGE_2000_DEG_S;
#endif

Smi230Acc::Smi230Acc(const std::shared_ptr<SensorThread> sensorThread)
  : Accelerometer(sensorThread, mAccelData, kSmi230AccelRange) {
  mSensorData.driverName = "smi230acc";
  mSensorData.sensorName = "SMI230 BOSCH Accelerometer Sensor";
  mSensorData.propertyIdentifier = "smi230";
  mSensorData.temperatureSysfsRaw = "in_temp_raw";
  mSensorData.minDelayUs = 5000;
  mSensorData.scanType = kSmi230ScanType;
  mSensorData.maxDelayUs = 2000000;
  mSensorData.power = 0.2f;
  mSensorData.temperatureScale = 0.001f;
  mSensorData.temperatureOffset = 0;
};

Smi230Gyro::Smi230Gyro(const std::shared_ptr<SensorThread> sensorThread)
  : Gyroscope(sensorThread, mGyroData, kSmi230GyroRange) {
  mSensorData.driverName = "smi230gyro";
  mSensorData.sensorName = "SMI230 BOSCH Gyroscope Sensor";
  mSensorData.propertyIdentifier = "smi230";
  mSensorData.minDelayUs = 5000;
  mSensorData.scanType = kSmi230ScanType;
  mSensorData.maxDelayUs = 2000000;
  mSensorData.power = 5.0f;
}

}  // namespace bosch::sensors