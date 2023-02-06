/* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include <stdint.h>
#include <sys/stat.h>
#include <memory>
#include <algorithm>
#include <dlfcn.h>
#include <errno.h>
#include <inttypes.h>
#include <linux/input.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <sensors.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/signalfd.h>
#include <sys/eventfd.h>
#include <sys/wait.h>
#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include <utils/Log.h>
#include <ctype.h>
#include <stdarg.h>
#include <sys/types.h>
#include <dirent.h>
#include <string>
#include <memory>
#include <algorithm>
#include <glib.h>
#include <stdint.h>
#include <functional>
#include <vector>

#undef LOG_TAG
#define LOG_TAG "sensor-test-app:"

#define strlcpy g_strlcpy
#define strlcat g_strlcat

/*
 * Constants
 */
#define DEFAULT_BATCH              100000000
#define DEFAULT_ODR                10000000
/* Max event buffer for poll sensor */
#define BUFFER_EVENT               2048

/* Defines to enable/disable sensors */
#define SENSOR_DISABLE             0
#define SENSOR_ENABLE              1

#define false                      0
#define true                       1

//#define LOGCAT_ENABLED // for getting logs in the logcat

#ifdef LOGCAT_ENABLED
#define SENSOR_LOGE(...) { ALOGE(__VA_ARGS__); }
#define SENSOR_LOGW(...) { ALOGW(__VA_ARGS__); }
#define SENSOR_LOGI(...) { ALOGI(__VA_ARGS__); }
#define SENSOR_LOGD(...) { ALOGD(__VA_ARGS__); }
#define SENSOR_LOGV(...) { ALOGV(__VA_ARGS__); }
#else
#define SENSOR_LOGE(...) { printf(__VA_ARGS__); }
#define SENSOR_LOGW(...) { printf(__VA_ARGS__); }
#define SENSOR_LOGI(...) { printf(__VA_ARGS__); }
#define SENSOR_LOGD(...) { printf(__VA_ARGS__); }
#define SENSOR_LOGV(...) { printf(__VA_ARGS__); }
#endif

#define SEARCH_PATH_SIZE            100
#define NAME_FILE                   "name"

#define DEVICE_IIO_MAX_FILENAME_LEN             256
#define DEVICE_IIO_MAX_NAME_LENGTH              32

#define IIO_PATH                "/sys/bus/iio/devices/"
#define INPUT_PATH              "/sys/devices/virtual/input/"

#define CLOSE_FILE_HANDLE(fd) do {  \
  if (fd) {                         \
    fflush(fd);                     \
    fclose(fd);                     \
    fd = NULL;                      \
  }                                 \
} while(0)

#define NS_TO_MS(x)                             (x / 1E6)
#define NS_TO_FREQUENCY(x)                      (1E9 / x)
#define FREQUENCY_TO_NS(x)                      (1E9 / x)
#define FREQUENCY_TO_US(x)                      (1E6 / x)

//To check sensor type defined in config file
typedef enum
{
  SENSOR_UNKN = 0,
  SENSOR_ASM330,
  SENSOR_IAM20680,
  SENSOR_SMI130,
  SENSOR_SMI230,
  SENSOR_BMI160
} sensorType;

//Type of devices based on sensor driver sysfs path mount.
typedef enum
{
  DYN_IIO_TYPE,
  DYN_INPUT_TYPE
} dynDeviceType;


bool tempSensorDataInit(int mSensorType);
bool CheckBufferReadFile(int mSensorType);
int tempSensorDataPollTask(float *temperature, int mSensorType);
void SensorBuffread(int mSensorType);

