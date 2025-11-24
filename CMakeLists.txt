cmake_minimum_required(VERSION 3.16)

project(AndroidSensorLegacyHAL
  DESCRIPTION "Android Sensor Legacy HAL"
  LANGUAGES C CXX)

find_program(CLANG_TIDY_EXE NAMES clang-tidy REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_CLANG_TIDY ${CLANG_TIDY_EXE};-checks=google-*;-extra-arg=-std=c++17)

set(MODULE smi330 CACHE INTERNAL "")         # Valid options: smi230, smi240, smi330
set(ARCH arm64 CACHE INTERNAL "")            # Valid options: arm, arm64
set(OPERATION_MODE sysfs CACHE INTERNAL "")  # Valid options: sysfs, drdy, fifo
set(ACCEL_RANGE 8 CACHE INTERNAL "")         # Valid options: 2=2G, 4=4G, 8=8G, 16=16G
set(GYRO_RANGE 250 CACHE INTERNAL "")        # Valid options: 125=125°/s, 250=250°/s, 500=500°/s, 1000=1000°/s, 2000=2000°/s

string(TOLOWER "${MODULE}" MODULE)
string(TOUPPER "${MODULE}" MODULE_UPPER)

if(ARCH STREQUAL arm)
  set(CMAKE_C_COMPILER arm-linux-gnueabihf-gcc)
  set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)
elseif (ARCH STREQUAL arm64)
  set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
else()
  message(FATAL_ERROR "ARCH not specified or ${ARCH} not supported")
endif()

# Mapping operation mode strings to enum class ScanType values
if (OPERATION_MODE STREQUAL sysfs)
  set(OPERATION_MODE 2)
elseif (OPERATION_MODE STREQUAL drdy)
  set(OPERATION_MODE 0)
elseif (OPERATION_MODE STREQUAL fifo)
  set(OPERATION_MODE 1)
endif()

########################################### LegacyHal ###########################################
add_library(LegacyHal SHARED
  core/SensorCore.cpp
  core/SensorThread.cpp
  core/Accelerometer.cpp
  core/Gyroscope.cpp
  hwctl/FileHandler.cpp
  sensors/SensorList.cpp
  sensors/SMI230.cpp
  sensors/SMI240.cpp
  sensors/SMI330.cpp
  hal/Sensor.cpp
  hal/Sensors.cpp
  hal/SensorsEventCallback.cpp
)

target_include_directories(LegacyHal PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/core
  ${PROJECT_SOURCE_DIR}/hwctl
  ${PROJECT_SOURCE_DIR}/sensors
  ${PROJECT_SOURCE_DIR}/hal
)

target_link_libraries(LegacyHal PRIVATE
  pthread
  stdc++
)

target_compile_options(LegacyHal PRIVATE -Wall -Wno-psabi 
                                         -D${MODULE_UPPER}_SCAN_TYPE=${OPERATION_MODE} 
                                         -D${MODULE_UPPER}_ACCEL_RANGE=${ACCEL_RANGE} 
                                         -D${MODULE_UPPER}_GYRO_RANGE=${GYRO_RANGE}
)
