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
#include <utils/Log.h>
#ifdef __cplusplus
#include <vector>
#endif

/* Backend-neutral logging shim: CLOG (commonlog) > DLT > caller's default logging.
 * The backend is picked at build time by USE_CLOG / USE_DLT. With neither
 * defined this still compiles: registration is a no-op and SensorLogWrite()
 * always returns 0, so callers use the default logging tier. */

/** @brief Severity, mapped onto CLOG or DLT levels internally. */
typedef enum {
    SENSOR_LOG_FATAL = 0,
    SENSOR_LOG_ERROR,
    SENSOR_LOG_WARN,
    SENSOR_LOG_INFO,
    SENSOR_LOG_DEBUG,
    SENSOR_LOG_VERBOSE
} SensorLogLevel;

/**
 * @brief Components that own a logging identity. Components not listed here
 *        log into the context the hosting process registered.
 */
typedef enum {
    /** sensor_hal_daemon: app "SENS", context "SHD". */
    SENSOR_LOG_ID_HAL_DAEMON = 0,
    /** sensor_client_api: hosted in a client's process, context "SCL". */
    SENSOR_LOG_ID_CLIENT_LIB
} SensorLogComponent;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Register this process's sensors logging identity.
 *
 * Refcounted and thread safe, so several component instances in one process
 * cannot deregister a context still in use by another. A library component
 * registers without claiming the application id.
 *
 * @param[in] component Which sensors component is registering.
 * @return    1 if a backend is available, 0 if the caller should use its
 *            default logging.
 */
int SensorLogRegister(SensorLogComponent component);

/** @brief Drop one reference taken by SensorLogRegister(). */
void SensorLogUnregister(void);

/**
 * @brief Emit one formatted message to the active backend.
 * @return 1 if the backend consumed the message, 0 if the caller should log it
 *         via its own default logging tier.
 */
int SensorLogWrite(SensorLogLevel level, const char *fmt, ...)
#ifdef __GNUC__
    __attribute__((format(printf, 2, 3)))
#endif
    ;

/** @brief Whether a backend is registered and accepting messages. */
int SensorLogBackendActive(void);

/**
 * @brief Whether the default logging tier should emit for a level, gated by
 *        DEBUG_LEVEL (E:1 W:2 I:3 D:4 V:5; 0, the default, prints nothing).
 */
int SensorLogDefaultEnabled(SensorLogLevel level);

#ifdef __cplusplus
}
#endif

#ifndef SENSOR_LOG_DEFAULT_E
#define SENSOR_LOG_DEFAULT_E(...) \
    do { if (SensorLogDefaultEnabled(SENSOR_LOG_ERROR))   ALOGE(__VA_ARGS__); } while (0)
#endif
#ifndef SENSOR_LOG_DEFAULT_W
#define SENSOR_LOG_DEFAULT_W(...) \
    do { if (SensorLogDefaultEnabled(SENSOR_LOG_WARN))    ALOGW(__VA_ARGS__); } while (0)
#endif
#ifndef SENSOR_LOG_DEFAULT_I
#define SENSOR_LOG_DEFAULT_I(...) \
    do { if (SensorLogDefaultEnabled(SENSOR_LOG_INFO))    ALOGI(__VA_ARGS__); } while (0)
#endif
#ifndef SENSOR_LOG_DEFAULT_D
#define SENSOR_LOG_DEFAULT_D(...) \
    do { if (SensorLogDefaultEnabled(SENSOR_LOG_DEBUG))   ALOGD(__VA_ARGS__); } while (0)
#endif
#ifndef SENSOR_LOG_DEFAULT_V
#define SENSOR_LOG_DEFAULT_V(...) \
    do { if (SensorLogDefaultEnabled(SENSOR_LOG_VERBOSE)) ALOGV(__VA_ARGS__); } while (0)
#endif

#ifndef SENSOR_LOGE
#define SENSOR_LOGE(...) \
    do { \
        if (!SensorLogWrite(SENSOR_LOG_ERROR, __VA_ARGS__)) \
            SENSOR_LOG_DEFAULT_E(__VA_ARGS__); \
    } while (0)

#define SENSOR_LOGW(...) \
    do { \
        if (!SensorLogWrite(SENSOR_LOG_WARN, __VA_ARGS__)) \
            SENSOR_LOG_DEFAULT_W(__VA_ARGS__); \
    } while (0)

#define SENSOR_LOGI(...) \
    do { \
        if (!SensorLogWrite(SENSOR_LOG_INFO, __VA_ARGS__)) \
            SENSOR_LOG_DEFAULT_I(__VA_ARGS__); \
    } while (0)

#define SENSOR_LOGD(...) \
    do { \
        if (!SensorLogWrite(SENSOR_LOG_DEBUG, __VA_ARGS__)) \
            SENSOR_LOG_DEFAULT_D(__VA_ARGS__); \
    } while (0)

#define SENSOR_LOGV(...) \
    do { \
        if (!SensorLogWrite(SENSOR_LOG_VERBOSE, __VA_ARGS__)) \
            SENSOR_LOG_DEFAULT_V(__VA_ARGS__); \
    } while (0)
#endif //SENSOR_LOGE

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
#ifdef __cplusplus
int GetFIRCoefficient(std::vector<float> &coef, char *suffix);
#endif

#endif //__SENSOR_LOG__
