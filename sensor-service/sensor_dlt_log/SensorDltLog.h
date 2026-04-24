/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef __SENSOR_DLT_LOG__
#define __SENSOR_DLT_LOG__

#include <stdarg.h>
#include <dlt/dlt.h>

extern int DLT_ENABLE;

/* Reads the DLT enable config from sensors.conf and updates DLT_ENABLE. */
void SetDltEnable();

#ifdef __cplusplus
extern "C" {
#endif

void dltLogInit(const char *appid, const char *ctid, const char *description);
void logtodlt(DltLogLevelType dlt_level, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif //__SENSOR_DLT_LOG__
