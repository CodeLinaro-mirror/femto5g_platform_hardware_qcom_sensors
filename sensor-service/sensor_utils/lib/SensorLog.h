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
 * Changes from Qualcomm Technologies, Inc. are provided under the following license:
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef __SENSOR_LOG__
#define __SENSOR_LOG__

#include <ctype.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <pthread.h>
#include <vector>
#include <utils/Log.h>

extern int DEBUG_LEVEL;

#define LOGCAT_ENABLED

/* Choose fallback logging */
#ifdef LOGCAT_ENABLED
#define SENSOR_FALLBACK_LOGE(...) ALOGE(__VA_ARGS__)
#define SENSOR_FALLBACK_LOGW(...) ALOGW(__VA_ARGS__)
#define SENSOR_FALLBACK_LOGI(...) ALOGI(__VA_ARGS__)
#define SENSOR_FALLBACK_LOGD(...) ALOGD(__VA_ARGS__)
#define SENSOR_FALLBACK_LOGV(...) ALOGV(__VA_ARGS__)
#else
#define SENSOR_FALLBACK_LOGE(...) printf(__VA_ARGS__)
#define SENSOR_FALLBACK_LOGW(...) printf(__VA_ARGS__)
#define SENSOR_FALLBACK_LOGI(...) printf(__VA_ARGS__)
#define SENSOR_FALLBACK_LOGD(...) printf(__VA_ARGS__)
#define SENSOR_FALLBACK_LOGV(...) printf(__VA_ARGS__)
#endif

/* Final logging macros */
#ifdef USE_DLT

#include "SensorDltLog.h"

#define SENSOR_LOGE(...) \
    do { \
        if (DLT_ENABLE) \
            logtodlt(DLT_LOG_ERROR, __VA_ARGS__); \
        else \
            SENSOR_FALLBACK_LOGE(__VA_ARGS__); \
    } while (0)

#define SENSOR_LOGW(...) \
    do { \
        if (DLT_ENABLE) \
            logtodlt(DLT_LOG_WARN, __VA_ARGS__); \
        else \
            SENSOR_FALLBACK_LOGW(__VA_ARGS__); \
    } while (0)

#define SENSOR_LOGI(...) \
    do { \
        if (DLT_ENABLE) \
            logtodlt(DLT_LOG_INFO, __VA_ARGS__); \
        else \
            SENSOR_FALLBACK_LOGI(__VA_ARGS__); \
    } while (0)

#define SENSOR_LOGD(...) \
    do { \
        if (DLT_ENABLE) \
            logtodlt(DLT_LOG_DEBUG, __VA_ARGS__); \
        else \
            SENSOR_FALLBACK_LOGD(__VA_ARGS__); \
    } while (0)

#define SENSOR_LOGV(...) \
    do { \
        if (DLT_ENABLE) \
            logtodlt(DLT_LOG_VERBOSE, __VA_ARGS__); \
        else \
            SENSOR_FALLBACK_LOGV(__VA_ARGS__); \
    } while (0)

#else  /* USE_DLT not defined */

#define SENSOR_LOGE(...) SENSOR_FALLBACK_LOGE(__VA_ARGS__)
#define SENSOR_LOGW(...) SENSOR_FALLBACK_LOGW(__VA_ARGS__)
#define SENSOR_LOGI(...) SENSOR_FALLBACK_LOGI(__VA_ARGS__)
#define SENSOR_LOGD(...) SENSOR_FALLBACK_LOGD(__VA_ARGS__)
#define SENSOR_LOGV(...) SENSOR_FALLBACK_LOGV(__VA_ARGS__)

#endif

#define MAX_FIR_COEF_ORDER          20

void SetSensorDebugLevel(int debug_level);
int SensorReadDebugLevel();

/**
 * CheckDiagEnabled : Checks if diag for a client is enabled
 * @param client_name: Client name to check if diag is enabled
 * @return Returns 1 if the diag is enabled for the client, else returns 0.
 */
int CheckDiagEnabled(const char *client_name);


/**
 * GetFIRCoefficient: Get the FIR coefficients
 * @param coef: array to store coefficients
 * @param suffix: suffix for sensor conf key (eg:suffix=ACC_400 means look for FIR_COEFFICIENT_ACC_400 and so on)
 * @return Returns size of coefficient array on success else returns -1
 */
int GetFIRCoefficient(std::vector<float> &coef, char *suffix);

#endif //__SENSOR_LOG__
