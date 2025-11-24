# Bosch MEMS Legacy HAL

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Overview

The Bosch MEMS Legacy HAL (Hardware Abstraction Layer) is a platform-independent implementation of Google's Android Sensor HAL 1.0 specification for Bosch IMU (Inertial Measurement Unit) sensors. This library serves as a bridge between Linux kernel drivers and user-space applications, providing a standardized interface for sensor data acquisition and control.

### Key Features

- **Platform Independent**: Built without dependencies on Android SDK environment
- **Standard Interface**: Implements Android Sensor HAL 1.0 specification
- **Multi-Sensor Support**: Compatible with SMI230, SMI240, and SMI330 sensor modules
- **Cross-Architecture**: Supports ARM and ARM64 architectures
- **Flexible Communication**: Supports both SPI and I2C interfaces
- **Multiple Operation Modes**: Supports data-ready triggered, FIFO, and sysfs polling modes
- **Configurable Sensor Ranges**: Buildtime configuration of accelerometer and gyroscope measurement ranges

### Supported Sensors

- SMI230
- SMI240
- SMI330

## Architecture

The Legacy HAL acts as a middleware layer with two primary responsibilities:

### 1. Data Bridge
- Provides data buffering between kernel drivers and applications
- Implements asynchronous data processing with dedicated sensor threads
- Supports multiple operation modes:
  - **IIO Buffer (Data-Ready)**: Interrupt-driven data acquisition
  - **IIO Buffer (FIFO)**: Hardware FIFO-based batching
  - **Sysfs Polling**: Software polling of sysfs nodes
- Manages sensor data flow and synchronization using observer pattern

### 2. Hardware Control
- Exposes unified APIs for sensor configuration
- Controls sampling frequency, power states, and operational modes
- Forwards control parameters to hardware through driver interface
- Supports configurable measurement ranges for accelerometer and gyroscope

### Interface Design

**Driver Interface**: Flexible communication with kernel drivers
- **IIO Buffer Mode**: Character device interface (`/dev/iio:deviceX`) for streaming data
- **Sysfs Mode**: File-based interface for configuration and polling
- Supports both synchronous and asynchronous data acquisition

**Application Interface**: Standard Android HAL 1.0 APIs:
- `get_sensors_list()` - Enumerate available sensors
- `activate()` - Enable/disable sensors
- `batch()` - Configure batching parameters
- `setDelay()` - Set sampling intervals
- `flush()` - Flush sensor data
- `poll()` - Retrieve sensor events

### Source Code Structure

```
bosch-mems-legacy-hal/
├── core/           # Core sensor functionality
│   ├── ISensorHal.h        # Sensor HAL interface definitions
│   ├── SensorCore.cpp/h    # Base sensor implementation
│   └── SensorThread.cpp/h  # Asynchronous data acquisition thread
├── hal/            # HAL implementation layer
│   ├── Sensor.cpp
│   ├── Sensors.cpp
│   └── SensorsEventCallback.cpp
├── hardware/       # Hardware abstraction headers
├── hwctl/          # Hardware control utilities
│   └── FileHandler.cpp/h   # Sysfs and IIO file operations
└── sensors/        # Sensor-specific implementations
    ├── SMI230.cpp/h    # SMI230 sensor driver
    ├── SMI240.cpp/h    # SMI240 sensor driver
    ├── SMI330.cpp/h    # SMI330 sensor driver
    └── SensorList.cpp/h

```

#### Component Details

- **core**: Sensor core functionality
  - Removed Android-specific logging
  - Adapted clock source for platform independence
- **hal**: Reference implementation based on:
  - [libhardware dynamic_sensor](https://android.googlesource.com/platform/hardware/libhardware/+/master/modules/sensors/dynamic_sensor)
- **hardware**: Headers from [libhardware](https://android.googlesource.com/platform/hardware/libhardware/+/refs/heads/main/include_all/hardware)
  - Adapted includes for standalone usage
- **hwctl**: Hardware control utilities
  - Removed Android logging dependencies
- **sensors**: Sensor implementations 
  - Removed Android logging dependencies
- **test**: Comprehensive test suite and specifications

## Prerequisites

### Development Environment

- **Build System**: CMake 3.16 or higher
- **Build Tool**: Ninja (recommended) or Make
- **Compiler**: Cross-compilation toolchain for target architecture
  - ARM: `arm-linux-gnueabihf-gcc/g++`
  - ARM64: `aarch64-linux-gnu-gcc/g++`
- **Code Quality**: clang-tidy (for static analysis)
- **Python**: Python 3 with development libraries (for testing)

### Target Hardware

- **Platform**: Raspberry Pi (recommended for testing)
- **Sensor Connection**: MEMS sensor connected via SPI or I2C
- **Driver**: Appropriate Linux kernel driver for your sensor module
- **Network**: SSH access for remote testing and deployment

### System Configuration

1. **Enable SPI/I2C**: Ensure SPI or I2C interface is active on target platform
2. **Driver Installation**: Load appropriate sensor kernel driver
3. **Permissions**: Configure proper permissions for sysfs access

## Quick Start

### 1. Configuration

Configure the build for your target sensor and architecture:

```bash
# Navigate to build directory
cd /path/to/build

# Configure for SMI330 on ARM64 with SPI interface and FIFO mode
cmake ../bosch-mems-legacy-hal -GNinja \
    -DMODULE=smi330 \
    -DARCH=arm64 \
    -DOPERATION_MODE=1 \
    -DACCEL_RANGE=8 \
    -DGYRO_RANGE=250
```

**Configuration Options:**

**Required:**
- `MODULE`: Sensor module (`smi230`, `smi240`, or `smi330`)
- `ARCH`: Target architecture (`arm` or `arm64`)

**Optional:**
- `OPERATION_MODE`: Data acquisition mode (default varies by sensor)
  - `0` = IIO Buffer (Data-Ready triggered)
  - `1` = IIO Buffer (FIFO mode)
  - `2` = Sysfs polling
- `ACCEL_RANGE`: Accelerometer measurement range in G (default varies by sensor)
  - SMI230: `2`, `4`, `8`, or `16`
  - SMI330: `2`, `4`, `8`, or `16`
  - SMI240: Fixed at 16G
- `GYRO_RANGE`: Gyroscope measurement range in °/s (default varies by sensor)
  - SMI230: `125`, `250`, `500`, `1000`, or `2000`
  - SMI330: `125`, `250`, or `500`
  - SMI240: Fixed at 300°/s

**Note:** SMI240 does not support IIO buffer modes and always operates in sysfs polling mode.

### 2. Build

```bash
# Build the library and test applications
ninja

# This creates:
# - libLegacyHal.so (shared library)
# - SensorTestApp (test application)
```

### 3. Testing

```bash
# Run comprehensive test suite
ninja run_test
```

## License

This project is licensed under the Apache License 2.0 

## Related Projects
SMI230 Linux IIO driver
SMI240 Linux IIO driver  
SMI330 Linux IIO driver

---

**Copyright (C) 2025 Robert Bosch GmbH**
