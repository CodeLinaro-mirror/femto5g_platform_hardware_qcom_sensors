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

#ifndef ANDROID_HARDWARE_BOSCH_SENSOR_THREAD_H
#define ANDROID_HARDWARE_BOSCH_SENSOR_THREAD_H

#include <poll.h>

#include <array>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "FileHandler.h"
#include "ISensorHal.h"

namespace bosch {
namespace sensors {

enum SensorTypeIndex { ACCEL_IDX, GYRO_IDX, IDX_LENGTH };
enum class IioDevice { ACC_AND_GYRO_COMBINED, ACC_AND_GYRO_SEPARATE };

class SensorThread {
public:
  SensorThread(IioDevice mode);
  ~SensorThread();

  void init(SensorTypeIndex idx, int deviceNum, std::function<void(int64_t)> odrCallback, ScanType scanType);
  void setResolution(SensorTypeIndex idx, float resolution);

  void registerObserver(SensorTypeIndex idx, const std::shared_ptr<ISensorValues>& observer);
  void removeObserver(SensorTypeIndex idx, const std::shared_ptr<ISensorValues>& observer);

  void enable(SensorTypeIndex idx, bool enable, const std::string& sysfsPwr, const std::string& mode);
  void setSamplingRate(SensorTypeIndex idx, int64_t samplingPeriodNs, int64_t reportLatencyNs);

private:
  void run();
  void start();
  void stop();

  void updateSamplingRate(bool enableBuffer);
  void readIioBufferAndNotify();
  void readSysfsAndNotify(int64_t timestamp);
  void notify(int readSize);

  static void startThread(SensorThread* sensor);

  static constexpr size_t kIioBufferLength = 32;
  static constexpr size_t kMaxFrameLength = 24;
  static constexpr size_t kBufferLength = kIioBufferLength * kMaxFrameLength;
  uint8_t mData[kBufferLength];

  enum ScanType mScanType;
  int mAccXIdx{0};
  int mAccYIdx{0};
  int mAccZIdx{0};
  int mGyroXIdx{0};
  int mGyroYIdx{0};
  int mGyroZIdx{0};
  int mTimestampIdx{0};
  int mFrameLength{0};

  int mDeviceNum{-1};
  float mAccResolution{1};
  float mGyroResolution{1};
  std::string mIioDevice{};
  std::string mCharDevice{};
  std::array<std::function<void(int64_t)>, IDX_LENGTH> mOdrCallback{};

  std::array<std::vector<std::weak_ptr<ISensorValues>>, IDX_LENGTH> mObservers{};

  int64_t mLastSampleTime{0};
  int64_t mCurrentTime{0};

  pollfd mPollfd;

  std::atomic_bool mStopThread{false};
  std::mutex mRunMutex;
  std::thread mRunThread;
  std::condition_variable mWaitCV;

  std::array<bool, IDX_LENGTH> mIsEnabled{false, false};
  std::array<int64_t, IDX_LENGTH> mSamplingPeriodNs{0, 0};
  std::array<int64_t, IDX_LENGTH> mReportLatencyNs{0, 0};
  int64_t mHwSamplingPeriodNs{0};

  int mReportLatencyFactor{1};
  int mPollCount{0};

  std::array<bosch::hwctl::RawSysfsHandler, IDX_LENGTH> mSysfsHandler;
};

}  // namespace sensors
}  // namespace bosch

#endif  // ANDROID_HARDWARE_BOSCH_SENSOR_THREAD_H
