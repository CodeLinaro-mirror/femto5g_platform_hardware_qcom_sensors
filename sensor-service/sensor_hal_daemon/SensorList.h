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
 */

#ifndef SENSORLIST_H
#define SENSORLIST_H

#include <string>
#include <memory>
#include <algorithm>
#include <glib.h>
#include <stdint.h>
#include <functional>
#include <sensors.h>

#define MAX_PATH_SIZE 100
#define MAX_ODR 6

struct sensor_list;

struct sensor_list {
    /* Name of this sensor.
     * All sensors of the same "type" must have a different "name".
     */
    char name[MAX_PATH_SIZE];
    /* vendor of the hardware part */
    char vendor[MAX_PATH_SIZE];
    /* sensor_id that identifies this sensors. This is used to reference
     * this sensor throughout the Sensor API.
     */
    int sensor_id;
    /* this sensor's type. */
    int type;
    /*maxSamplingRate: Max sampling rate supported by the sensor as defined in
     sensors.conf file.client can’t request the samples more than this*/
    int maxSamplingRate;
    /*minBatchCount: Min batch count supported by the sensor as defined in,
     sensors.conf file. clitn  can’t request the samples less than this*/
    int minBatchCount;
    /*maxBatchCount: Max batch count supported by the sensor,
     *client can’t request the samples more than this*/
    int maxBatchCount;
    /*SamplingRate: List of sampling rate supported by the sensor*/
    float odr[MAX_ODR];
    /**Range of sensor**/
    int range;
};

struct sensor_mlc_case_list;

struct sensor_mlc_case_list {
    /* Name of mlc case.
     */
    char name[MAX_PATH_SIZE];
};

/**
 * struct mlc_event_data - The actual event being pushed to userspace
 * @id:         event identifier
 * @timestamp:  best estimate of time of event occurrence (often from
 *              the interrupt handler)
 */
struct mlc_event_data {
        uint64_t id;
        int64_t  timestamp;
};

#endif //SENSORLIST_H
