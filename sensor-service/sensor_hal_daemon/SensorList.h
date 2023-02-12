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

typedef enum {
        /*Senshor HAL Deamon is ready to communicate*/
        SHD_READY        = (1<<0),
	/*Sensor Hal Daemon is not running*/
        SHD_NOT_RUNNING  = (1<<1),
	/*Sensor Hal Daemon is restartd*/
        SHD_RESTARTED    = (1<<2),
	/*Device is about to go suspend state, This notifcation
	will get when SHD is enabled with power manager daemon*/
        DEVICE_SUSPEND   = (1<<3),
	/*Device is about to go resume state, This notifcation
	will get when SHD is enabled with power manager daemon*/
        DEVICE_RESUME    = (1<<4),
	/*Device is about to go shutdown state, This notifcation
	will get when SHD is enabled with power manager daemon*/
        DEVICE_SHUTDOWN  = (1<<5),
}SensorCapabilitiesMask;

typedef enum {
        /** On Success **/
        SENSOR_RESPONSE_SUCCESS=0,
        /** Client is not registered to SHD **/
        SENSOR_ERROR_CLIENT_REGISTER_FAILED=-1,
        /** Client is not generated while registering the client **/
        SENSOR_ERROR_INVALID_CLIENT=-2,
        /** Invalid input parameteres from respective AP **/
        SENSOR_ERROR_INVALID_INPUT_PARAMETER=-3,
        /** Callback is null in respective API **/
        SENSOR_ERROR_CALLBACK_MISSING=-4,
        /** Not supported feature of sensor **/
        SENSOR_ERROR_NOT_SUPPORTED=-5,
        /** Physical Sensor Enable/Disable failed **/
        SENSOR_ERROR_CONTROL_FAILED=-6,
        /** Physical Sensor Config failed **/
        SENSOR_ERROR_CONFIG_FAILED=-7,
        /** Socket communication failed b/w SHD and client lib **/
        SENSOR_ERROR_IPC_FAILED=-8,
        /** No sensors supported in h/w **/
        SENSOR_ERROR_NO_SENSORS_FOUND=-9,
        /**No snesor is activated and configured**/
        SENSOR_ERROR_TRACKING_FAILED=-10,
        /** Unknown error **/
        SENSOR_ERROR_UNKNOWN=-11,
        /** Buffer is not supported by sensor**/
        SENSOR_ERROR_BUFFER_NOT_SUPPORTED=-12,
        /** Buffer is deleted**/
        SENSOR_ERROR_BUFFER_DELETED=-13,
        /** MLC Event Enable failed**/
        SENSOR_ERROR_MLC_EVENT_ENABLE_FAILED=-14,
        /** NO MLC case found**/
        SENSOR_ERROR_NO_MLC_CASE_FOUND=-15,
        /**Sensor No response from SHD timeout happens*/
        SENSOR_ERROR_NO_RESPONSE_FROM_SHD_TIMEOUT = -16,
        /**Sensor selftest is not supported**/
        SENSOR_SELFTEST_NOT_SUPPORTED = -17,
}SensorRet;

typedef enum {
    /*Disable the sensor*/
     SENSOR_DISABLE = 0,
    /*Enable the sensor*/
     SENSOR_ENABLE,
    /*Low power mode*/
     SENSOR_LPM,
    /*High power mode*/
     SENSOR_HPM,
}sensor_state;

enum SelfTestType {
        /*Positive-Sign of selftest*/
        Positive,
        /*Negative-Sign of selftest*/
        Negative,
        /*Any future mode of selftest*/
        // Any other
};

enum SelfTestResult {
        /*selftest is passed*/
        Passed,
        /*selftest is failed*/
        Failed,
        /*future error code*/
        // Any other error-code
};

struct sensor_list;

struct sensor_list {
    /* version of the hardware part + driver. The value of this field
     * must increase when the driver is updated in a way that changes the
     * output of this sensor. This is important for fused sensors when the
     * fusion algorithm is updated.
     */
    int             version;
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
    /* smallest difference between two values reported by this sensor */
    float           resolution;
    /* maximum range of this sensor's value in SI units */
    float           maxRange;
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
