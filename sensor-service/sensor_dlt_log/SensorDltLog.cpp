/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "SensorDltLog.h"

#define LOG_MAX_LENGTH 512
#define DLT_ID_SIZE 5

#define SENSOR_CONF_PATH "/etc/sensors.conf"

static DltContext ctx;
int DLT_ENABLE = 0;

/**@brief:  Reads DLT_ENABLED from the sensor config file and updates DLT_ENABLE.
 *          Kept self contained in sensor_dlt_log to avoid a circular dependency
 *          on sensor_util_lib (which links against this library).
 * @return  void
 */
void SetDltEnable()
{
    FILE *file = fopen(SENSOR_CONF_PATH, "r");
    if (file == NULL)
    {
        return;
    }

    char buffer[BUFSIZ];
    while (fgets(buffer, sizeof(buffer), file) != NULL)
    {
        for (size_t i = 0; i < strlen(buffer); i++)
        {
            if (buffer[i] == '#')
            {
                break;
            }
            else if (buffer[i] == ' ')
            {
                continue;
            }
            else if (strstr(buffer, "DLT_ENABLED="))
            {
                char *line = strstr(buffer, "=");
                if (line != NULL)
                {
                    (void)sscanf(&line[1], "%d", &DLT_ENABLE);
                }
                break;
            }
        }
    }
    (void)fclose(file);
}

#ifdef __cplusplus
extern "C" {
#endif

/**@brief:  Function to init dlt logging by registering appid and context
 * @return     void
 */
void dltLogInit(const char *appid, const char *ctid, const char *description)
{
    char AppId[DLT_ID_SIZE] = {0};

    SetDltEnable();

    DLT_GET_APPID(AppId);
    if (DLT_ENABLE && AppId[0] == '\0')
    {
        DLT_REGISTER_APP(appid, description);
    }
    DLT_REGISTER_CONTEXT(ctx, ctid, description);
}

/**@brief:      Function to route all log messages to DLT
 * @param[in]	dlt log level
 * @param[in]	variable arguments
 * @return	    void
 */
void logtodlt(DltLogLevelType dlt_level, const char *fmt, ...)
{
    /* If the length of log message is more than 512 then it may get truncated */
    char buf[LOG_MAX_LENGTH];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    DLT_LOG(ctx, dlt_level, DLT_CSTRING(buf));
}

#ifdef __cplusplus
}
#endif
