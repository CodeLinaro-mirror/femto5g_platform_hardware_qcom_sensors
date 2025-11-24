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

#pragma once

#include <array>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

/**
 * FileHandler: Utilities for sysfs interaction
 *
 * Provides robust read/write operations for sysfs files with proper error handling,
 * path management, and retry mechanisms for unreliable sysfs attributes.
 *
 * Changes:
 * - Added robust error reporting with errno logging
 * - Improved path concatenation handling
 * - Enhanced input validation and edge case handling
 * - Optimized string operations and reduced copies
 * - RAII for resource management
 * - Better integer parsing with overflow detection
 */

namespace bosch::hwctl {

/**
 * ReadHandler: Sysfs file reader
 *
 * Safely reads text content from sysfs files with automatic trimming of whitespace.
 */
class ReadHandler {
public:
  /**
   * Construct a reader for a sysfs file
   * @param path Base path (e.g., "/sys/bus/iio/devices/iio:device0/")
   * @param file File name or relative path
   */
  ReadHandler(const std::string& path, const std::string& file) noexcept;
  ~ReadHandler() noexcept = default;

  /**
   * Read and trim content from the file
   * @param result Output string to store the trimmed content
   * @return 0 on success, -1 on failure (logs error to stderr)
   */
  int read(std::string& result) noexcept;

private:
  std::string mFilePath;
};

/**
 * WriteHandler: Sysfs file writer
 *
 * Safely writes text content to sysfs files with automatic flushing.
 */
class WriteHandler {
public:
  /**
   * Construct a writer for a sysfs file
   * @param path Base path (e.g., "/sys/bus/iio/devices/iio:device0/")
   * @param file File name or relative path
   */
  WriteHandler(const std::string& path, const std::string& file) noexcept;

  /**
   * Construct and immediately write to a sysfs file
   * @param path Base path
   * @param file File name or relative path
   * @param content Content to write
   */
  WriteHandler(const std::string& path, const std::string& file, const std::string& content) noexcept;
  ~WriteHandler() noexcept = default;

  /**
   * Write content to the file
   * @param content String to write
   * @return 0 on success, -1 on failure (logs error to stderr)
   */
  int write(const std::string& content) noexcept;

private:
  std::string mFilePath;
};

/**
 * WriteReadbackHandler: Write with verification
 *
 * Writes to a sysfs file and verifies the content was accepted by reading it back.
 * Some sysfs attributes don't immediately reflect changes, so this retries with delays.
 */
class WriteReadbackHandler {
public:
  /**
   * Construct a write-readback handler
   * @param path Base path
   * @param file File name or relative path
   */
  WriteReadbackHandler(const std::string& path, const std::string& file) noexcept;

  /**
   * Construct and immediately write with verification
   * @param path Base path
   * @param file File name or relative path
   * @param content Content to write and verify
   */
  WriteReadbackHandler(const std::string& path, const std::string& file, const std::string& content) noexcept;
  ~WriteReadbackHandler() noexcept = default;

  /**
   * Write content and verify by reading back
   * @param content String to write
   * @return 0 on success (content verified), -1 on failure or mismatch
   */
  int writeAndRead(const std::string& content) noexcept;

private:
  std::string mPath;
  std::string mFile;
  static constexpr int kMaxRetries = 5;              // retry attempts if immediate readback mismatch
  static constexpr useconds_t kRetryDelayUs = 1000;  // 1ms between retries
};

/**
 * RawSysfsHandler: High-performance batch integer reader
 *
 * Uses raw file descriptors and pread() for efficient reading of multiple
 * integer-valued sysfs files without repeated open/close overhead.
 * Ideal for high-frequency sensor data polling.
 */
class RawSysfsHandler {
public:
  ~RawSysfsHandler() noexcept { close(); }

  /**
   * Open multiple sysfs files for reading
   * @param path Base path
   * @param files Array of up to 3 file names (empty strings are ignored)
   */
  void open(const std::string& path, const std::array<std::string, 3>& files) noexcept;

  /**
   * Close all open file descriptors
   */
  void close() noexcept;

  /**
   * Read integer values from all open files
   * @param results Output vector of parsed integers (one per file)
   * @return 0 on success, -1 on any read or parse failure
   */
  int read(std::vector<int>& results) noexcept;

private:
  std::vector<int> mFileDescriptors{};
  std::vector<std::string> mFilePaths{};
  static constexpr size_t kReadBufferSize = 32;
};

/**
 * Check if a sensor driver is available in the IIO subsystem
 *
 * Scans /sys/bus/iio/devices/ for a device with matching driver name.
 *
 * @param driverName The driver name to search for (e.g., "smi230_accel")
 * @param deviceNum Output parameter for the device number if found
 * @return true if sensor found, false otherwise
 */
bool isSensorAvailable(const std::string& driverName, int* deviceNum) noexcept;

}  // namespace bosch::hwctl