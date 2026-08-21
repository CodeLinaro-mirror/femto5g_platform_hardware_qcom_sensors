/*
 * Copyright (C) 2018-2019 InvenSense, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Changes from Qualcomm Technologies, Inc. are provided under the following license:
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef LOG_H
#define LOG_H

#ifndef LOG_TAG
#define LOG_TAG "SensorSvc_HAL:"
#endif

#include <cutils/log.h>
#include <SensorLog.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __ANDROID__

/* Android logs */
#include <cutils/log.h>

#define ASSERT		ALOG_ASSERT

#define LOGV		SENSOR_LOGV
#define LOGD		SENSOR_LOGD
#define LOGI		SENSOR_LOGI
#define LOGW		SENSOR_LOGW
#define LOGE		SENSOR_LOGE

#define LOGV_IF		ALOGV_IF
#define LOGD_IF		ALOGD_IF
#define LOGI_IF		ALOGI_IF
#define LOGW_IF		ALOGW_IF
#define LOGE_IF		ALOGE_IF

#else

/* Linux logs */
#ifndef LOG_NDEBUG
#define LOG_NDEBUG 1
#endif

#if LOG_NDEBUG
#define NDEBUG
#endif
#include <assert.h>

#ifndef LOG_TAG
#define LOG_HEAD	"Sensor: "
#else
#define LOG_HEAD	LOG_TAG ": "
#endif

#if LOG_NDEBUG
#define ASSERT(cond, ...)	((void)0)
#else
#define ASSERT(cond, ...)				\
	do {						\
		SENSOR_LOGE(__VA_ARGS__);	\
		assert(cond);				\
	} while (0)
#endif

//#if LOG_NDEBUG
#define LOGV(...)	((void)0)
#define LOGD(...)	((void)0)
//#else
//#define LOGV(...)	SENSOR_LOGV("[IAM20680] Sensor:" __VA_ARGS__)
//#define LOGD(...)	SENSOR_LOGD("[IAM20680] Sensor:" __VA_ARGS__)
//#endif
#define LOGI(...)	SENSOR_LOGI(LOG_TAG "[IAM20680] Sensor:" __VA_ARGS__)
#define LOGW(...)	SENSOR_LOGW(LOG_TAG "[IAM20680] Sensor:" __VA_ARGS__)
#define LOGE(...)	SENSOR_LOGE(LOG_TAG "[IAM20680] Sensor:" __VA_ARGS__)


#define LOGV_IF(cond, ...)	((void)LOGV(__VA_ARGS__))
#define LOGD_IF(cond, ...)      ((void)LOGV(__VA_ARGS__))	
#define LOGI_IF(cond, ...)	((void)LOGV(__VA_ARGS__))
#define LOGW_IF(cond, ...)	((void)LOGV(__VA_ARGS__))
#define LOGE_IF(cond, ...)	((void)LOGV(__VA_ARGS__))
#endif		/* __ANDROID__ */

#ifdef __cplusplus
}
#endif

#endif		/* LOG_H */
