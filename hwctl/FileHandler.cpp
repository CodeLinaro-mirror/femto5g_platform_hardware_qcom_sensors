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

#include "FileHandler.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>

namespace bosch::hwctl {

static std::string trim(const std::string& s) noexcept {
  auto start = std::find_if(s.begin(), s.end(), [](unsigned char c) { return !std::isspace(c); });
  auto end = std::find_if(s.rbegin(), s.rend(), [](unsigned char c) { return !std::isspace(c); }).base();
  return (start < end) ? std::string(start, end) : std::string();
}

ReadHandler::ReadHandler(const std::string& path, const std::string& file) noexcept : mFilePath(path + file) {}

int ReadHandler::read(std::string& result) noexcept {
  std::ifstream f(mFilePath);
  if (!f.is_open()) {
    std::cerr << "ReadHandler: Failed to open " << mFilePath << " (errno: " << errno << ")\n";
    return -1;
  }

  result.assign((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  if (f.fail() && !f.eof()) {
    std::cerr << "ReadHandler: Error reading from " << mFilePath << "\n";
    return -1;
  }
  result = trim(result);

  return 0;
}

WriteHandler::WriteHandler(const std::string& path, const std::string& file) noexcept : mFilePath(path + file) {}

WriteHandler::WriteHandler(const std::string& path, const std::string& file, const std::string& content) noexcept
  : mFilePath(path + file) {
  const int status = write(content);
  if (status != 0) {
    std::cerr << "WriteHandler: Failed to write during construction to " << mFilePath << "\n";
  }
}

int WriteHandler::write(const std::string& content) noexcept {
  std::ofstream f(mFilePath);
  if (!f.is_open()) {
    std::cerr << "WriteHandler: Failed to open " << mFilePath << " for writing (errno: " << errno << ")\n";
    return -1;
  }

  f << content;
  f.flush();
  if (f.fail()) {
    std::cerr << "WriteHandler: Failed to write to " << mFilePath << "\n";
    return -1;
  }

  return 0;
}

WriteReadbackHandler::WriteReadbackHandler(const std::string& path, const std::string& file) noexcept
  : mPath(path), mFile(file) {}

WriteReadbackHandler::WriteReadbackHandler(const std::string& path, const std::string& file,
                                           const std::string& content) noexcept
  : mPath(path), mFile(file) {
  const int status = writeAndRead(content);
  if (status != 0) {
    std::cerr << "WriteReadbackHandler: Failed to write and verify '" << content << "' to " << path + file << "\n";
  }
}

int WriteReadbackHandler::writeAndRead(const std::string& content) noexcept {
  {
    WriteHandler handler(mPath, mFile);
    const int status = handler.write(content);
    if (status != 0) return status;
  }

  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    std::string readback;
    ReadHandler handler(mPath, mFile);
    const int status = handler.read(readback);
    if ((status == 0) && (readback == content)) return 0;
    usleep(kRetryDelayUs);
  }
  return -1;
}

void RawSysfsHandler::open(const std::string& path, const std::array<std::string, 3>& files) noexcept {
  close();
  mFileDescriptors.reserve(files.size());
  mFilePaths.reserve(files.size());

  for (const auto& file : files) {
    if (file.empty()) break;

    std::string fullPath = path + file;
    int fd = ::open(fullPath.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd >= 0) {
      mFileDescriptors.push_back(fd);
      mFilePaths.push_back(std::move(fullPath));
    } else {
      std::cerr << "RawSysfsHandler: Failed to open " << fullPath << " (errno: " << errno << ")\n";
      close();
      break;
    }
  }
}

void RawSysfsHandler::close() noexcept {
  for (int fd : mFileDescriptors) {
    if (fd >= 0) ::close(fd);
  }
  mFileDescriptors.clear();
  mFilePaths.clear();
}

int RawSysfsHandler::read(std::vector<int>& results) noexcept {
  results.clear();
  if (mFileDescriptors.empty()) {
    std::cerr << "RawSysfsHandler: No file descriptors to read from\n";
    return -1;
  }
  results.resize(mFileDescriptors.size());

  char buffer[kReadBufferSize];

  for (size_t i = 0; i < mFileDescriptors.size(); ++i) {
    const int fd = mFileDescriptors[i];

    ssize_t bytesRead;
    do {
      bytesRead = ::pread(fd, buffer, kReadBufferSize - 1, 0);
    } while (bytesRead < 0 && errno == EINTR);

    if (bytesRead < 0) {
      std::cerr << "RawSysfsHandler: Failed to read " << mFilePaths[i] << " (errno: " << errno << ")\n";
      return -1;
    }
    if (bytesRead == 0) {
      std::cerr << "RawSysfsHandler: Empty read from " << mFilePaths[i] << "\n";
      return -1;
    }

    buffer[bytesRead] = '\0';

    char* end = nullptr;
    errno = 0;
    const int64_t value = std::strtol(buffer, &end, 10);

    if (end == buffer) {
      std::cerr << "RawSysfsHandler: No digits found in " << mFilePaths[i] << ": " << buffer << "\n";
      return -1;
    }
    if (errno == ERANGE) {
      std::cerr << "RawSysfsHandler: Value out of range in " << mFilePaths[i] << ": " << buffer << "\n";
      return -1;
    }
    if (*end != '\0' && *end != '\n' && !std::isspace(static_cast<unsigned char>(*end))) {
      std::cerr << "RawSysfsHandler: Invalid integer format in " << mFilePaths[i] << ": " << buffer << "\n";
      return -1;
    }

    results[i] = static_cast<int>(value);
  }

  return 0;
}

bool isSensorAvailable(const std::string& driverName, int* deviceNum) noexcept {
  if (deviceNum == nullptr) {
    std::cerr << "isSensorAvailable: deviceNum pointer is null\n";
    return false;
  }
  if (driverName.empty()) {
    std::cerr << "isSensorAvailable: driverName is empty\n";
    return false;
  }

  *deviceNum = -1;
  const std::string iioPath = "/sys/bus/iio/devices/";
  std::string name;

  struct DirCloser {
    DIR* dir;
    explicit DirCloser(DIR* d) : dir(d) {}
    ~DirCloser() {
      if (dir) closedir(dir);
    }
    DIR* get() const { return dir; }
  };

  DirCloser dirCloser(opendir(iioPath.c_str()));
  DIR* dir = dirCloser.get();
  if (!dir) {
    std::cerr << "isSensorAvailable: Failed to open " << iioPath << " (errno: " << errno << ")\n";
    return false;
  }

  for (dirent* entry = readdir(dir); entry != nullptr; entry = readdir(dir)) {
    const char* dname = entry->d_name;

    if (std::strncmp(dname, "iio:device", 10) != 0) continue;

    const char* numPart = dname + 10;
    if (*numPart == '\0') continue;

    char* endPtr = nullptr;
    errno = 0;
    const int64_t devNum = std::strtol(numPart, &endPtr, 10);

    if (endPtr == numPart || *endPtr != '\0' || errno == ERANGE || devNum < 0) continue;

    const std::string namePath = std::string(dname) + "/name";
    ReadHandler fileHandler(iioPath, namePath);

    if (fileHandler.read(name) == 0 && !name.empty()) {
      if (name.compare(0, driverName.size(), driverName) == 0) {
        *deviceNum = static_cast<int>(devNum);
        return true;
      }
    }
  }

  return false;
}

}  // namespace bosch::hwctl