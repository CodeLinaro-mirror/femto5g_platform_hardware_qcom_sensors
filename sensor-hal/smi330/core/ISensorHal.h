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

#ifndef ANDROID_HARDWARE_BOSCH_ISENSOR_HAL_H
#define ANDROID_HARDWARE_BOSCH_ISENSOR_HAL_H

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace bosch {
namespace sensors {

/*
 * Reduce polling time of the sensor thread by this factor.
 * 1 means no reduction, 0.5 means half the polling time.
 */
constexpr float POLL_TIME_REDUCTION_FACTOR = 1.0f;

/*
 * Accept sensor data with timestamp diffs > factor * expected diff.
 */
constexpr float VALID_TIMESTAMP_FACTOR = 0.9f;

constexpr float MIN_VALID_TIMESTAMP_FACTOR = 0.5f;

enum BoschSensorType {
  INVALID = 0,
  ACCEL = 1,  // SensorType::ACCELEROMETER
  GYRO = 4,   // SensorType::GYROSCOPE
};

enum class ScanType {
  IIO_BUFFER_DRDY,
  IIO_BUFFER_FIFO,
  SYSFS,
};

enum class AccelRange {
  RANGE_2G = 2,
  RANGE_4G = 4,
  RANGE_8G = 8,
  RANGE_16G = 16,
};

enum class GyroRange {
  RANGE_125_DEG_S = 125,
  RANGE_250_DEG_S = 250,
  RANGE_500_DEG_S = 500,
  RANGE_1000_DEG_S = 1000,
  RANGE_2000_DEG_S = 2000,
};

struct SensorData {
  std::string vendor{"Robert Bosch GmbH"};
  std::string driverName;
  std::string sensorName;
  std::string propertyIdentifier;
  std::string temperatureSysfsRaw;
  BoschSensorType type;
  int32_t minDelayUs;
  int32_t maxDelayUs;
  float power;
  float range;
  float resolution;
  float temperatureScale;
  float temperatureOffset;
  enum ScanType scanType;
};

struct SensorValues {
  BoschSensorType type;
  int64_t timestamp;
  std::vector<float> data;
};

class ISensorValues {
public:
  ISensorValues() = default;
  virtual ~ISensorValues() = default;

  virtual void update(const std::vector<SensorValues>& values) = 0;
};

class ISensorHal {
public:
  ISensorHal() = default;
  virtual ~ISensorHal() = default;

  virtual void registerObserver(const std::shared_ptr<ISensorValues>& observer) = 0;
  virtual void removeObserver(const std::shared_ptr<ISensorValues>& observer) = 0;
  virtual bool readSensorTemperature(float* temperature) = 0;
  virtual void activate(bool enable) = 0;
  virtual void batch(int64_t samplingPeriodNs, int64_t maxReportLatencyNs) = 0;
  virtual const SensorData& getSensorData() const = 0;
};

}  // namespace sensors
}  // namespace bosch

#endif  // ANDROID_HARDWARE_BOSCH_ISENSOR_HAL_H
