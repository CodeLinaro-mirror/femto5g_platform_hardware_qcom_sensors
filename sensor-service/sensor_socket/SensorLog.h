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
 * Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
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

#define IF_SENSOR_LOGE if(( DEBUG_LEVEL >= 1) && ( DEBUG_LEVEL <= 5))
#define IF_SENSOR_LOGW if(( DEBUG_LEVEL >= 2) && ( DEBUG_LEVEL <= 5))
#define IF_SENSOR_LOGI if(( DEBUG_LEVEL >= 3) && ( DEBUG_LEVEL <= 5))
#define IF_SENSOR_LOGD if(( DEBUG_LEVEL >= 4) && ( DEBUG_LEVEL <= 5))
#define IF_SENSOR_LOGV if(( DEBUG_LEVEL >= 5) && ( DEBUG_LEVEL <= 5))

#ifdef LOGCAT_ENABLED
#define SENSOR_LOGE(...) IF_SENSOR_LOGE { ALOGE(__VA_ARGS__); }
#define SENSOR_LOGW(...) IF_SENSOR_LOGW { ALOGW(__VA_ARGS__); }
#define SENSOR_LOGI(...) IF_SENSOR_LOGI { ALOGI(__VA_ARGS__); }
#define SENSOR_LOGD(...) IF_SENSOR_LOGD { ALOGD(__VA_ARGS__); }
#define SENSOR_LOGV(...) IF_SENSOR_LOGV { ALOGV(__VA_ARGS__); }
#else
#define SENSOR_LOGE(...) IF_SENSOR_LOGE { printf(__VA_ARGS__); }
#define SENSOR_LOGW(...) IF_SENSOR_LOGW { printf(__VA_ARGS__); }
#define SENSOR_LOGI(...) IF_SENSOR_LOGI { printf(__VA_ARGS__); }
#define SENSOR_LOGD(...) IF_SENSOR_LOGD { printf(__VA_ARGS__); }
#define SENSOR_LOGV(...) IF_SENSOR_LOGV { printf(__VA_ARGS__); }
#endif

#define MAX_FIR_COEF_ORDER          20

void SetSensorDebugLevel(int debug_level);

int SensorReadDebugLevel();

/**
 * GetFIRCoefficient: Get the FIR coefficients
 * @param coef: array to store coefficients
 * @param suffix: suffix for sensor conf key (eg:suffix=ACC_400 means look for FIR_COEFFICIENT_ACC_400 and so on)
 * @return Returns size of coefficient array on success else returns -1
 */
int GetFIRCoefficient(std::vector<float> &coef, char *suffix);

/**
 * CheckDiagEnabled : Checks if diag for a client is enabled
 * @param client_name: Client name to check if diag is enabled
 * @return Returns 1 if the diag is enabled for the client, else returns 0.
 */
int CheckDiagEnabled(const char *client_name);

#endif //__SENSOR_LOG__
