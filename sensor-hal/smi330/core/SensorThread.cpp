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

#include "SensorThread.h"

#include <fcntl.h>
#include <unistd.h>

#include <cinttypes>
#include <cstring>
#include <iostream>
#include <sstream>

namespace {
enum class IioSingleBufferIndex {
  IIO_ACCEL_X = 0,
  IIO_ACCEL_Y = 2,
  IIO_ACCEL_Z = 4,
  IIO_GYRO_X = 0,
  IIO_GYRO_Y = 2,
  IIO_GYRO_Z = 4,
  IIO_TIMESTAMP = 8,
  IIO_LENGTH = 16
};

enum class IioCombinedBufferIndex {
  IIO_ACCEL_X = 0,
  IIO_ACCEL_Y = 2,
  IIO_ACCEL_Z = 4,
  IIO_GYRO_X = 6,
  IIO_GYRO_Y = 8,
  IIO_GYRO_Z = 10,
  IIO_TIMESTAMP = 16,
  IIO_LENGTH = 24
};

static inline int64_t getCurrentTimeNs() {
  struct timespec timestamp;
  clock_gettime(CLOCK_MONOTONIC, &timestamp);
  return timestamp.tv_sec * 1000000000LL + timestamp.tv_nsec;
}

}  // namespace

using bosch::hwctl::ReadHandler;
using bosch::hwctl::WriteHandler;
using bosch::hwctl::WriteReadbackHandler;
using bosch::sensors::SensorThread;

SensorThread::SensorThread(IioDevice mode) {
  if (mode == IioDevice::ACC_AND_GYRO_COMBINED) {
    mAccXIdx = static_cast<int>(IioCombinedBufferIndex::IIO_ACCEL_X);
    mAccYIdx = static_cast<int>(IioCombinedBufferIndex::IIO_ACCEL_Y);
    mAccZIdx = static_cast<int>(IioCombinedBufferIndex::IIO_ACCEL_Z);
    mGyroXIdx = static_cast<int>(IioCombinedBufferIndex::IIO_GYRO_X);
    mGyroYIdx = static_cast<int>(IioCombinedBufferIndex::IIO_GYRO_Y);
    mGyroZIdx = static_cast<int>(IioCombinedBufferIndex::IIO_GYRO_Z);
    mTimestampIdx = static_cast<int>(IioCombinedBufferIndex::IIO_TIMESTAMP);
    mFrameLength = static_cast<int>(IioCombinedBufferIndex::IIO_LENGTH);
  } else {
    mAccXIdx = static_cast<int>(IioSingleBufferIndex::IIO_ACCEL_X);
    mAccYIdx = static_cast<int>(IioSingleBufferIndex::IIO_ACCEL_Y);
    mAccZIdx = static_cast<int>(IioSingleBufferIndex::IIO_ACCEL_Z);
    mGyroXIdx = static_cast<int>(IioSingleBufferIndex::IIO_GYRO_X);
    mGyroYIdx = static_cast<int>(IioSingleBufferIndex::IIO_GYRO_Y);
    mGyroZIdx = static_cast<int>(IioSingleBufferIndex::IIO_GYRO_Z);
    mTimestampIdx = static_cast<int>(IioSingleBufferIndex::IIO_TIMESTAMP);
    mFrameLength = static_cast<int>(IioSingleBufferIndex::IIO_LENGTH);
  }
}

SensorThread::~SensorThread() { stop(); }

void SensorThread::init(SensorTypeIndex idx, int deviceNum, std::function<void(int64_t)> odrCallback,
                        ScanType scanType) {
  if (deviceNum != mDeviceNum) {
    mDeviceNum = deviceNum;
    mScanType = scanType;
    mIioDevice = "/sys/bus/iio/devices/iio:device" + std::to_string(deviceNum) + "/";

    std::cout << "SensorThread[" << deviceNum << "]: Initializing with scan type " << static_cast<int>(mScanType)
              << std::endl;

    if (mScanType != ScanType::SYSFS) {
      mCharDevice = "/dev/iio:device" + std::to_string(deviceNum);

      { WriteReadbackHandler handler(mIioDevice, "buffer0/enable", "0"); }
      { WriteReadbackHandler handler(mIioDevice, "buffer0/in_timestamp_en", "1"); }
      { WriteReadbackHandler handler(mIioDevice, "buffer0/length", std::to_string(kIioBufferLength)); }
      { WriteReadbackHandler handler(mIioDevice, "buffer0/watermark", "1"); }

      if (mScanType == ScanType::IIO_BUFFER_FIFO) {
        // Remove trigger to enable hardware fifo mode
        WriteHandler handler(mIioDevice, "trigger/current_trigger", "none");
      }

      std::string readback;
      ReadHandler handler(mIioDevice, "trigger/current_trigger");
      const int ret = handler.read(readback);
      if (ret) {
        std::cerr << "Failed to read current trigger" << std::endl;
      } else {
        if (mScanType == ScanType::IIO_BUFFER_DRDY) {
          if (readback.empty()) std::cerr << "Failed to switch to data ready mode" << std::endl;
        } else if (mScanType == ScanType::IIO_BUFFER_FIFO) {
          if (!readback.empty()) std::cerr << "Failed to switch to hardware fifo mode" << std::endl;
        }
      }
    }
  }

  mOdrCallback[idx] = odrCallback;

  if (mScanType != ScanType::SYSFS) {
    if (idx == ACCEL_IDX) {
      // SMI230 has a separate thread for accel and gyro
      { WriteReadbackHandler handler(mIioDevice, "buffer0/in_accel_x_en", "1"); }
      { WriteReadbackHandler handler(mIioDevice, "buffer0/in_accel_y_en", "1"); }
      { WriteReadbackHandler handler(mIioDevice, "buffer0/in_accel_z_en", "1"); }
    } else if (idx == GYRO_IDX) {
      { WriteReadbackHandler handler(mIioDevice, "buffer0/in_anglvel_x_en", "1"); }
      { WriteReadbackHandler handler(mIioDevice, "buffer0/in_anglvel_y_en", "1"); }
      { WriteReadbackHandler handler(mIioDevice, "buffer0/in_anglvel_z_en", "1"); }
    }
  }
}

void SensorThread::setResolution(SensorTypeIndex idx, float resolution) {
  if (idx == ACCEL_IDX) {
    mAccResolution = resolution;
  } else if (idx == GYRO_IDX) {
    mGyroResolution = resolution;
  }
}

void SensorThread::start() {
  std::lock_guard<std::mutex> lock(mRunMutex);
  if (mRunThread.joinable()) {
    std::cerr << "SensorThread[" << mDeviceNum << "]: already running" << std::endl;
    return;
  }
  mCurrentTime = getCurrentTimeNs();
  mLastSampleTime = 0;

  if (mScanType != ScanType::SYSFS) {
    mPollfd.fd = open(mCharDevice.c_str(), O_RDONLY | O_NONBLOCK);
    if (mPollfd.fd < 0) {
      std::cerr << "Failed to open iio char device: " << mCharDevice << std::endl;
    }
    mPollfd.events = POLLIN;

    WriteReadbackHandler handler(mIioDevice, "buffer0/enable", "1");
  }

  mStopThread.store(false);
  mRunThread = std::thread(startThread, this);
}

void SensorThread::stop() {
  std::thread threadToJoin;
  {
    std::lock_guard<std::mutex> lock(mRunMutex);
    if (!mRunThread.joinable()) {
      std::cerr << "SensorThread[" << mDeviceNum << "]: already stopped" << std::endl;
      return;
    }

    mStopThread.store(true);
    mWaitCV.notify_all();

    threadToJoin = std::move(mRunThread);
  }

  threadToJoin.join();

  if (mScanType != ScanType::SYSFS) {
    std::lock_guard<std::mutex> lock(mRunMutex);
    if (mPollfd.fd >= 0) {
      close(mPollfd.fd);
      mPollfd.fd = -1;
    }
    WriteReadbackHandler handler(mIioDevice, "buffer0/enable", "0");
  }
}

void SensorThread::startThread(SensorThread* sensor) { sensor->run(); }

void SensorThread::run() {
  std::unique_lock<std::mutex> runLock(mRunMutex);

  while (!mStopThread.load()) {
    if (mScanType != ScanType::SYSFS) {
      runLock.unlock();
      const int ret = poll(&mPollfd, 1, 200);
      runLock.lock();
      if ((ret > 0) && (mPollfd.revents & POLLIN)) {
        readIioBufferAndNotify();
      }
    } else {
      const int64_t start = getCurrentTimeNs();
      readSysfsAndNotify(start);
      const int64_t end = getCurrentTimeNs();
      int64_t waitTime = POLL_TIME_REDUCTION_FACTOR * mHwSamplingPeriodNs - (end - start);
      if (waitTime < 1000000) waitTime = 1000000;
      mWaitCV.wait_for(runLock, std::chrono::nanoseconds(waitTime), [this] { return mStopThread.load(); });
    }
  }
}

void SensorThread::readIioBufferAndNotify() {
  const auto readSize = read(mPollfd.fd, mData, mFrameLength * kIioBufferLength);
  if (readSize < mFrameLength) {
    std::cerr << "Failed to read iio buffer: " << static_cast<size_t>(readSize) << std::endl;
    return;
  }
  notify(readSize);
}

void SensorThread::readSysfsAndNotify(int64_t timestamp) {
  std::string sensorData;

  uint8_t* bufferPtr = &mData[mPollCount * mFrameLength];

  memset(bufferPtr, 0, mFrameLength);

  if (mIsEnabled[ACCEL_IDX]) {
    std::vector<int> data;
    if (mSysfsHandler[ACCEL_IDX].read(data) == 0) {
      memcpy(bufferPtr + mAccXIdx, &data[0], sizeof(data[0]));
      memcpy(bufferPtr + mAccYIdx, &data[1], sizeof(data[1]));
      memcpy(bufferPtr + mAccZIdx, &data[2], sizeof(data[2]));
    } else {
      std::cerr << "Failed to read accel data" << std::endl;
      return;
    }
  }

  if (mIsEnabled[GYRO_IDX]) {
    std::vector<int> data;
    if (mSysfsHandler[GYRO_IDX].read(data) == 0) {
      memcpy(bufferPtr + mGyroXIdx, &data[0], sizeof(data[0]));
      memcpy(bufferPtr + mGyroYIdx, &data[1], sizeof(data[1]));
      memcpy(bufferPtr + mGyroZIdx, &data[2], sizeof(data[2]));
    } else {
      std::cerr << "Failed to read gyro data" << std::endl;
      return;
    }
  }

  memcpy(bufferPtr + mTimestampIdx, &timestamp, sizeof(timestamp));
  mPollCount++;

  if (mPollCount >= mReportLatencyFactor) {
    notify(mPollCount * mFrameLength);
    mPollCount = 0;
  }
}

void SensorThread::notify(int readSize) {
  std::vector<SensorValues> accValues{};
  std::vector<SensorValues> gyroValues{};

  for (int i = 0; i < readSize / mFrameLength; i++) {
    const int idx = i * mFrameLength;
    const int64_t timestamp = *reinterpret_cast<int64_t*>(mData + mTimestampIdx + idx);

    int64_t delta = 0;
    if ((mLastSampleTime != 0) && (timestamp > mLastSampleTime)) {
      delta = timestamp - mLastSampleTime;
      if (delta > 10 * mHwSamplingPeriodNs) {  // Clamp delta
        delta = mHwSamplingPeriodNs;
      }
    }
    mLastSampleTime = timestamp;
    mCurrentTime += delta;

    if (mIsEnabled[ACCEL_IDX]) {
      SensorValues value{};
      value.type = BoschSensorType::ACCEL;
      value.timestamp = mCurrentTime;
      const int16_t x = static_cast<int16_t>((mData[mAccXIdx + idx + 1] << 8) | mData[mAccXIdx + idx]);
      const int16_t y = static_cast<int16_t>((mData[mAccYIdx + idx + 1] << 8) | mData[mAccYIdx + idx]);
      const int16_t z = static_cast<int16_t>((mData[mAccZIdx + idx + 1] << 8) | mData[mAccZIdx + idx]);
      value.data.push_back(x * mAccResolution);
      value.data.push_back(y * mAccResolution);
      value.data.push_back(z * mAccResolution);
      accValues.push_back(value);
    }

    if (mIsEnabled[GYRO_IDX]) {
      SensorValues value{};
      value.type = BoschSensorType::GYRO;
      value.timestamp = mCurrentTime;
      const int16_t x = static_cast<int16_t>((mData[mGyroXIdx + idx + 1] << 8) | mData[mGyroXIdx + idx]);
      const int16_t y = static_cast<int16_t>((mData[mGyroYIdx + idx + 1] << 8) | mData[mGyroYIdx + idx]);
      const int16_t z = static_cast<int16_t>((mData[mGyroZIdx + idx + 1] << 8) | mData[mGyroZIdx + idx]);
      value.data.push_back(x * mGyroResolution);
      value.data.push_back(y * mGyroResolution);
      value.data.push_back(z * mGyroResolution);
      gyroValues.push_back(value);
    }
  }

  if (!accValues.empty()) {
    for (const auto& obs : mObservers[ACCEL_IDX]) {
      if (auto observerPtr = obs.lock()) {
        observerPtr->update(accValues);
      }
    }
  }
  if (!gyroValues.empty()) {
    for (const auto& obs : mObservers[GYRO_IDX]) {
      if (auto observerPtr = obs.lock()) {
        observerPtr->update(gyroValues);
      }
    }
  }
}

void SensorThread::registerObserver(SensorTypeIndex idx, const std::shared_ptr<ISensorValues>& observer) {
  std::lock_guard<std::mutex> lock(mRunMutex);
  mObservers[idx].push_back(observer);
}

void SensorThread::removeObserver(SensorTypeIndex idx, const std::shared_ptr<ISensorValues>& observer) {
  std::lock_guard<std::mutex> lock(mRunMutex);
  mObservers[idx].erase(
    std::remove_if(mObservers[idx].begin(), mObservers[idx].end(),
                   [observer](const std::weak_ptr<ISensorValues>& obs) { return obs.lock() == observer; }),
    mObservers[idx].end());
}

void SensorThread::enable(SensorTypeIndex idx, bool enable, const std::string& sysfsPwr, const std::string& mode) {
  if (!sysfsPwr.empty() && !mode.empty()) {
    { WriteHandler handler(mIioDevice, sysfsPwr, mode); }
    usleep(200000);

    std::string readback;
    ReadHandler handler(mIioDevice, sysfsPwr);
    const int ret = handler.read(readback);
    if (ret || (readback != mode)) std::cerr << "Failed to set power mode " << mode << " to " << sysfsPwr << std::endl;
  }

  std::unique_lock<std::mutex> lock(mRunMutex);

  const bool wasAnyEnabled = mIsEnabled[ACCEL_IDX] || mIsEnabled[GYRO_IDX];
  mIsEnabled[idx] = enable;
  if (mScanType == ScanType::SYSFS) {
    if (idx == ACCEL_IDX) {
      if (enable) {
        mSysfsHandler[ACCEL_IDX].open(mIioDevice, {"in_accel_x_raw", "in_accel_y_raw", "in_accel_z_raw"});
      } else {
        mSysfsHandler[ACCEL_IDX].close();
      }
    } else if (idx == GYRO_IDX) {
      if (enable) {
        mSysfsHandler[GYRO_IDX].open(mIioDevice, {"in_anglvel_x_raw", "in_anglvel_y_raw", "in_anglvel_z_raw"});
      } else {
        mSysfsHandler[GYRO_IDX].close();
      }
    }
  }

  if (enable) {
    updateSamplingRate(wasAnyEnabled);
  }

  if (enable && !wasAnyEnabled) {
    lock.unlock();
    start();
  } else if (!mIsEnabled[ACCEL_IDX] && !mIsEnabled[GYRO_IDX]) {
    lock.unlock();
    stop();
  } else {
    return;  // nothing to do
  }
}

void SensorThread::setSamplingRate(SensorTypeIndex idx, int64_t samplingPeriodNs, int64_t reportLatencyNs) {
  std::lock_guard<std::mutex> lock(mRunMutex);

  mSamplingPeriodNs[idx] = samplingPeriodNs;
  mReportLatencyNs[idx] = reportLatencyNs;
  updateSamplingRate(true);
}

void SensorThread::updateSamplingRate(bool enableBuffer) {
  int64_t usedReportLatencyNs = 0;

  if (mIsEnabled[ACCEL_IDX] && mIsEnabled[GYRO_IDX]) {
    mHwSamplingPeriodNs = std::min(mSamplingPeriodNs[ACCEL_IDX], mSamplingPeriodNs[GYRO_IDX]);
    usedReportLatencyNs = std::min(mReportLatencyNs[ACCEL_IDX], mReportLatencyNs[GYRO_IDX]);
  } else if (mIsEnabled[ACCEL_IDX]) {
    mHwSamplingPeriodNs = mSamplingPeriodNs[ACCEL_IDX];
    usedReportLatencyNs = mReportLatencyNs[ACCEL_IDX];
  } else if (mIsEnabled[GYRO_IDX]) {
    mHwSamplingPeriodNs = mSamplingPeriodNs[GYRO_IDX];
    usedReportLatencyNs = mReportLatencyNs[GYRO_IDX];
  } else {
    return;
  }

  mPollCount = 0;
  mReportLatencyFactor = std::max(
    1, std::min(static_cast<int>(kIioBufferLength), static_cast<int>(usedReportLatencyNs / mHwSamplingPeriodNs)));

  std::cout << "SensorThread[" << mDeviceNum << "]: Setting ODR to " << mHwSamplingPeriodNs
            << " ns, report latency factor to " << mReportLatencyFactor << std::endl;

  if (mScanType == ScanType::SYSFS) {
    for (auto& cb : mOdrCallback) {
      if (cb) cb(mHwSamplingPeriodNs);
    }
  } else {
    WriteReadbackHandler enableHandler(mIioDevice, "buffer0/enable", "0");

    for (auto& cb : mOdrCallback) {
      if (cb) cb(mHwSamplingPeriodNs);
    }

    {
      std::string watermark = std::to_string(mReportLatencyFactor);
      WriteReadbackHandler wmHandler(mIioDevice, "buffer0/watermark", watermark);
    }

    mCurrentTime = getCurrentTimeNs();
    mLastSampleTime = 0;

    if (enableBuffer && (mIsEnabled[ACCEL_IDX] || mIsEnabled[GYRO_IDX])) {
      const int ret = enableHandler.writeAndRead("1");
      if (ret != 0) std::cerr << "Failed to enable iio buffer" << std::endl;
    }
  }

  mWaitCV.notify_all();
}
