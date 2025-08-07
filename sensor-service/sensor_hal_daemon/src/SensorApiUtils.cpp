/* Copyright (c) 2021 The Linux Foundation. All rights reserved.
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
 *     * Neither the name of The Linux Foundation, nor the names of its
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
 * Copyright 2015-2018 STMicroelectronics Inc.
 * Author: Denis Ciocca - <denis.ciocca@st.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 *
 * Changes from Qualcomm Technologies, Inc. are provided under the following license:
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <stdint.h>
#include <sys/stat.h>
#include <dlfcn.h>
#include <memory>
#include <algorithm>
#include <stack>
#include <SensorApiService.h>

using namespace std;

// Monotonic boot time
uint64_t get_timestamp() {
        struct timespec ts;
        clock_gettime(CLOCK_BOOTTIME, &ts);
        uint64_t system_ts =
                ((uint64_t)(ts.tv_sec)) * 1000000000ULL + ((uint64_t)(ts.tv_nsec));
        return system_ts;
}

int sysfs_write_scale(char *file, float val)
{
	FILE *fp;

	fp = fopen(file, "w");
	if (NULL == fp)
		return -errno;

	int ret = fprintf(fp, "%.9g", val);
	(void)fclose(fp);

	return ret > 0 ? 0 : ret;
}


int sysfs_read_scale(char *file, float *val)
{
        FILE *fp;

        fp = fopen(file, "r");
        if (NULL == fp)
                return -errno;

        (void)fscanf(fp, "%f", val);
        (void)fclose(fp);

        return 0;
}

int sysfs_write_int(char *file, int val)
{
        FILE *fp;

        fp = fopen(file, "w");
        if (NULL == fp)
                return -errno;

        (void)fprintf(fp, "%d", val);
        (void)fclose(fp);

        return 0;
}

int sysfs_read_int(char *file, int *val)
{
	FILE *fp;
	int ret;

	fp = fopen(file, "r");
	if (NULL == fp)
		return -errno;

	ret = fscanf(fp, "%d\n", val);
	(void)fclose(fp);

	return ret;
}
/**
 * get_input_sensor_device_by_name() - function to match top level types by name
 * @type: the type of top level instance being searched
 *
 * Returns the device number of a matched input device on success, otherwise a
 * negative error code.
 * Typical types this is used for are device and trigger.
 **/
int get_input_sensor_device_by_name(const char *name)
{
        struct dirent *ent;
        int number, numstrlen;
        FILE *devilceFile;
        DIR *dp;
        char dname[DEVICE_MAX_NAME_LENGTH];
        char dfilename[DEVICE_MAX_FILENAME_LEN + 1];
        int ret;
        int fnamelen;

        dp = opendir(device_input_dir);
        if (NULL == dp)
                return -ENODEV;

        for (ent = readdir(dp); ent != 0; ent = readdir(dp)) {
                if (strlen(ent->d_name) <= strlen(device_input_device_name) ||
                    !strcmp(ent->d_name, ".") ||
                    !strcmp(ent->d_name, ".."))
                        continue;

                if (strncmp(ent->d_name, device_input_device_name,
                            strlen(device_input_device_name)) == 0) {
                        numstrlen = sscanf(ent->d_name +
                                           strlen(device_input_device_name),
                                           "%d", &number);
                        fnamelen = numstrlen + strlen(device_input_dir) +
                                   strlen(device_input_device_name);
                        if (fnamelen > DEVICE_MAX_FILENAME_LEN)
                                continue;
                        (void)snprintf(dfilename, DEVICE_MAX_FILENAME_LEN,
                                "%s%s%d/name",
                                device_input_dir,
                                device_input_device_name,
                                number);
                        devilceFile = fopen(dfilename, "r");
                        if (!devilceFile)
                                continue;

                        ret = fscanf(devilceFile, "%s", dname);
                        if (ret <= 0) {
                                (void)fclose(devilceFile);
                                break;
                        }

                        if (strncmp(name, dname, strlen(dname)) == 0 &&
                            /* check if asm330lhh and asm330lhhx */
                            strlen(name) == strlen(dname)) {
                                (void)fclose(devilceFile);
                                (void)closedir(dp);
                                return number;
                        }

                (void)fclose(devilceFile);
                }
        }

        (void)closedir(dp);

        return -ENODEV;
}

/**
 * get_iio_sensor_device_by_name() - function to match top level types by name
 * @type: the type of top level instance being searched
 *
 * Returns the device number of a matched IIO device on success, otherwise a
 * negative error code.
 * Typical types this is used for are device and trigger.
 **/
int get_iio_sensor_device_by_name(const char *name)
{
        struct dirent *ent;
        int number, numstrlen;
        FILE *devilceFile;
        DIR *dp;
        char dname[DEVICE_MAX_NAME_LENGTH];
        char dfilename[DEVICE_MAX_FILENAME_LEN + 1];
        int ret;
        int fnamelen;

        dp = opendir(device_iio_dir);
        if (NULL == dp)
                return -ENODEV;

        for (ent = readdir(dp); ent != 0; ent = readdir(dp)) {
                if (strlen(ent->d_name) <= strlen(device_iio_device_name) ||
                    !strcmp(ent->d_name, ".") ||
                    !strcmp(ent->d_name, ".."))
                        continue;

                if (strncmp(ent->d_name, device_iio_device_name,
                            strlen(device_iio_device_name)) == 0) {
                        numstrlen = sscanf(ent->d_name +
                                           strlen(device_iio_device_name),
                                           "%d", &number);
                        fnamelen = numstrlen + strlen(device_iio_dir) +
                                   strlen(device_iio_device_name);
                        if (fnamelen > DEVICE_MAX_FILENAME_LEN)
                                continue;
                        (void)snprintf(dfilename, DEVICE_MAX_FILENAME_LEN,
                                "%s%s%d/name",
                                device_iio_dir,
                                device_iio_device_name,
                                number);
                        devilceFile = fopen(dfilename, "r");
                        if (!devilceFile)
                                continue;

                        ret = fscanf(devilceFile, "%s", dname);
                        if (ret <= 0) {
                                (void)fclose(devilceFile);
                                break;
                        }

                        if (strncmp(name, dname, strlen(dname)) == 0 &&
                            /* check if asm330lhh and asm330lhhx */
                            strlen(name) == strlen(dname)) {
                                (void)fclose(devilceFile);
                                (void)closedir(dp);
                                return number;
                        }

                (void)fclose(devilceFile);
                }
        }

        (void)closedir(dp);

        return -ENODEV;
}

int get_sensor_type(struct device_iio_info_channel *channel,
                               const char *device_dir, const char *name,
                               const char *post)
{
        DIR *dp;
        int ret;
        FILE *sysfsfp;
        unsigned padint;
        const struct dirent *ent;
        char signchar, endianchar;
        char dir[DEVICE_MAX_FILENAME_LEN + 1];
        char type_name[DEVICE_MAX_FILENAME_LEN + 1];
        char name_post[DEVICE_MAX_FILENAME_LEN + 1];
        char filename[DEVICE_MAX_FILENAME_LEN + 1];

        /* Check string len */
        if (strlen(device_dir) +
            strlen("scan_elements") + 1 > DEVICE_MAX_FILENAME_LEN)
                return -1;

        if (strlen(name) +
            strlen("_type") + 1 > DEVICE_MAX_FILENAME_LEN)
                return -1;

        if (strlen(post) +
            strlen("_type") + 1 > DEVICE_MAX_FILENAME_LEN)
                return -1;

        (void)snprintf(dir, DEVICE_MAX_FILENAME_LEN, "%s/scan_elements", device_dir);
        (void)snprintf(type_name, DEVICE_MAX_FILENAME_LEN, "%s_type", name);
        (void)snprintf(name_post, DEVICE_MAX_FILENAME_LEN, "%s_type", post);

        dp = opendir(dir);
        if (dp == NULL)
                return -errno;

        while (ent = readdir(dp), ent != NULL) {
                if ((strcmp(type_name, ent->d_name) == 0) ||
                    (strcmp(name_post, ent->d_name) == 0)) {
                        (void)snprintf(filename, DEVICE_MAX_FILENAME_LEN, "%s/%s", dir, ent->d_name);
                        sysfsfp = fopen(filename, "r");
                        if (sysfsfp == NULL)
                                continue;

                        /* scan format like "le:s16/16>>0" */
                        ret = fscanf(sysfsfp, "%ce:%c%u/%u>>%u",
                                     &endianchar,
                                     &signchar,
                                     &channel->bits_used,
                                     &padint,
                                     &channel->shift);
                        if (ret < 0)
                                continue;

                        channel->be = (endianchar == 'b');
                        channel->sign = (signchar == 's');
                        channel->bytes = (padint >> 3);

                        if (channel->bits_used == 64)
                                channel->mask = ~0;
                        else
                                channel->mask = (1 << channel->bits_used) - 1;

                        (void)fclose(sysfsfp);
                }
        }

        (void)closedir(dp);

        return 0;
}

int get_sensor_scale(const char *device_dir, float *value)
{
        int ret;
        char tmp_filaname[DEVICE_MAX_FILENAME_LEN];
        char *scale_filename;

        scale_filename = (char *)"in_accel_x_scale";

        /* read <iio:devicex>/in_<device_type>_x_scale */
        ret = snprintf(tmp_filaname, DEVICE_MAX_FILENAME_LEN,
                       "%s/%s", device_dir, scale_filename);

        return ret < 0 ? -ENOMEM : sysfs_read_scale(tmp_filaname, value);
}

int enable_sensor_channels(const char *device_dir, bool enable)
{
        char dir[DEVICE_MAX_FILENAME_LEN + 1];
        char filename[DEVICE_MAX_FILENAME_LEN + 1];
        const struct dirent *ent;
        FILE *sysfsfp;
        DIR *dp;

        if (strlen(device_dir) +
                strlen("scan_elements") + 1 > DEVICE_MAX_FILENAME_LEN)
                return -1;

        (void)snprintf(dir, DEVICE_MAX_FILENAME_LEN, "%s/scan_elements", device_dir);
        dp = opendir(dir);
        if (!dp)
                return -errno;

        while (ent = readdir(dp), ent != NULL) {
        if (strlen(dir) +
                strlen(ent->d_name) > DEVICE_MAX_FILENAME_LEN)
                continue;

                if (!strcmp(ent->d_name + strlen(ent->d_name) - strlen("_en"),
                            "_en")) {
                        (void)snprintf(filename, DEVICE_MAX_FILENAME_LEN, "%s/%s", dir, ent->d_name);
                        sysfsfp = fopen(filename, "r+");
                        if (!sysfsfp) {
                                (void)closedir(dp);
                                return -errno;
                        }

                        (void)fprintf(sysfsfp, "%d", enable);
                        (void)fclose(sysfsfp);
                }
        }

        (void)closedir(dp);

        return 0;
}

/**
 * size_from_channelarray() - Calculate the storage size of a scan
 * @channels: the channel info array.
 * @num_channels: number of channels.
 **/
int size_from_channelarray(struct device_iio_info_channel *channels,
                                  int num_channels)
{
        int bytes = 0, i;

        for (i = 0; i < num_channels; i++) {
                channels[i].location = 0;

                if (channels[i].bytes == 0)
                        continue;

                if (bytes % channels[i].bytes == 0)
                        channels[i].location = bytes;
                else
                        channels[i].location = bytes -
                          (bytes % channels[i].bytes) + channels[i].bytes;

                bytes = channels[i].location + channels[i].bytes;
        }

        return bytes;
}

/**
 * process_2byte_received() - Return channel data from 2 byte
 * @input: 2 byte of data received from buffer channel.
 * @info: information about channel structure.
 **/
float process_2byte_received(int input,
                                    struct device_iio_info_channel *info)
{
        float res;
        int16_t val;

        if (info->be)
                input = be16toh((uint16_t)input);
        else
                input = le16toh((uint16_t)input);

        val = input >> info->shift;

        if (info->sign) {
                val &= (1 << info->bits_used) - 1;
                val = (int16_t)(val << (16 - info->bits_used)) >> (16 - info->bits_used);
                res = (float)val;
        } else {
                val &= (1 << info->bits_used) - 1;
                res = (float)((uint16_t)val);
        }

        return ((res + info->offset) * info->scale); // Calculate scaled value 
}

/**
 * process_3byte_received() - Return channel data from 3 byte
 * @input: 3 byte of data received from buffer channel.
 * @info: information about channel structure.
 **/
float process_3byte_received(int input,
                                    struct device_iio_info_channel *info)
{
        float res;
        int32_t val;

        if (info->be)
                input = be32toh((uint32_t)input);
        else
                input = le32toh((uint32_t)input);

        val = input >> info->shift;
        if (info->sign) {
                val &= (1 << info->bits_used) - 1;
                val = (int32_t)(val << (24 - info->bits_used)) >> (24 - info->bits_used);
                res = (float)val;
        } else {
                val &= (1 << info->bits_used) - 1;
                res = (float)((uint32_t)val);
        }

        return ((res + info->offset) * info->scale); // Calculate scaled value 
}

//To Dump the Senor Events.
void dump_sensor_event(const struct sensors_event_t *e)
{
    switch (e->type) {
    case SENSOR_TYPE_ACCELEROMETER:
        SENSOR_LOGD(LOG_TAG "ACC event: x=%f y=%f z=%f timestamp=%lld\n",
			e->acceleration.x, e->acceleration.y, e->acceleration.z,
			e->timestamp);
	break;
    case SENSOR_TYPE_MAGNETIC_FIELD:
        SENSOR_LOGD(LOG_TAG "MAG event: x=%f y=%f z=%f timestamp=%lld\n",
			e->magnetic.x, e->magnetic.y, e->magnetic.z,
			e->timestamp);
        break;
    case SENSOR_TYPE_GYROSCOPE:
        SENSOR_LOGD(LOG_TAG "GYRO event: x=%f y=%f z=%f timestamp=%lld\n",
			e->gyro.x, e->gyro.y, e->gyro.z, e->timestamp);
        break;
    case SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED:
        SENSOR_LOGD(LOG_TAG "ACC event: x=%f y=%f z=%f\
			x_bias=%.2f y_bias=%.2f z_bias=%.2f timestamp=%lld\n",
			e->uncalibrated_accelerometer.x_uncalib, e->uncalibrated_accelerometer.y_uncalib,
			e->uncalibrated_accelerometer.z_uncalib, e->uncalibrated_accelerometer.x_bias,
			e->uncalibrated_accelerometer.y_bias, e->uncalibrated_accelerometer.z_bias, e->timestamp);
        break;
    case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
        SENSOR_LOGD(LOG_TAG "GYRO event x=%f y=%f z=%f\
			x_bias=%.2f y_bias=%.2f z_bias=%.2f timestamp=%lld\n",
			e->uncalibrated_gyro.x_uncalib, e->uncalibrated_gyro.y_uncalib,
			e->uncalibrated_gyro.z_uncalib, e->uncalibrated_gyro.x_bias,
			e->uncalibrated_gyro.y_bias, e->uncalibrated_gyro.z_bias, e->timestamp);
        break;
    default:
        SENSOR_LOGD(LOG_TAG "Unknown sensor_id events %d\n", e->type);
        break;
    }
}

/*API for boot kpi marker prints
 * on Success : return 0
 * on Failure : return -1 */
int sensor_boot_kpi_marker(const char * pFmt, ...)
{
    int result = -1;
    int32_t errRet = -1;
    struct stat nodeStat;

    // Check if the KPI node exists exists
    errRet = stat(BOOT_KPI_FILE, &nodeStat);
    if (errRet == 0) {
        char buf[MAX_COMMAND_STR_LEN] = {};
        va_list ap;
        va_start(ap, pFmt);
        vsnprintf(&buf[0], sizeof(buf), pFmt, ap);
        int fd = 0;
        fd = open(BOOT_KPI_FILE, O_WRONLY);
        if (fd > 0) {
            write(fd, buf, strlen(buf));
            close(fd);
	    result = 0;
        }
        va_end(ap);
    }
    return result;
}
