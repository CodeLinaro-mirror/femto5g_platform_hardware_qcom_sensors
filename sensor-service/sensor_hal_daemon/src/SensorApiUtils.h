/* Copyright (c) 2021, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Changes from Qualcomm Technologies, Inc. are provided under the following license:
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef SENSORUTILS_H
#define SENSORUTILS_H

#include <string>
#include <memory>
#include <algorithm>
#include <glib.h>
#include <stdint.h>
#include <functional>
#include <sensors.h>

#define DEVICE_MAX_FILENAME_LEN             256
#define DEVICE_MAX_NAME_LENGTH              32

#define CLOSE_FILE_HANDLE(fd) do {  \
  if (fd) {                         \
    fflush(fd);                     \
    fclose(fd);                     \
    fd = NULL;                      \
  }                                 \
} while(0)

#define NS_TO_MS(x)                             (x / 1000000)
#define NS_TO_FREQUENCY(x)                      (1E9 / x)
#define FREQUENCY_TO_NS(x)                      (1E9 / x)
#define FREQUENCY_TO_US(x)                      (1E6 / x)
#define MAX_COMMAND_STR_LEN (255)
#define BOOT_KPI_FILE "/sys/kernel/boot_kpi/kpi_values"

static const char *device_input_dir = "/sys/devices/virtual/input/";
static const char *device_input_device_name = "input";

static const char *device_iio_dir = "/sys/bus/iio/devices/";
static const char *device_iio_device_name = "iio:device";
static const char *device_iio_buffer_enable = "buffer/enable";
static const char *device_iio_buffer_length = "buffer/length";

struct device_iio_info_channel {
        char *name;
        char *type_name;
        unsigned int index;
        unsigned int enabled;
        float scale;
        float offset;
        unsigned int bytes;
        unsigned int bits_used;
        unsigned int shift;
        unsigned long long int mask;
        unsigned int be;
        unsigned int sign;
        unsigned int location;
};

uint64_t get_timestamp();
int sysfs_write_scale(char *file, float val);
int sysfs_read_scale(char *file, float *val);
int sysfs_write_int(char *file, int val);
int sysfs_read_int(char *file, int *val);
int get_iio_sensor_device_by_name(const char *name);
int get_input_sensor_device_by_name(const char *name);
int get_sensor_type(struct device_iio_info_channel *channel,
		const char *device_dir, const char *name,
		const char *post);
int get_sensor_scale(const char *device_dir, float *value);
void dump_sensor_event(const struct sensors_event_t *e);
int size_from_channelarray(struct device_iio_info_channel *channels,
		int num_channels);
float process_2byte_received(int input,
		struct device_iio_info_channel *info);
float process_3byte_received(int input,
		struct device_iio_info_channel *info);
int sensor_boot_kpi_marker(const char * pFmt, ...);
int enable_sensor_channels(const char *device_dir, bool enable);

#endif //SENSORUTILS_H
