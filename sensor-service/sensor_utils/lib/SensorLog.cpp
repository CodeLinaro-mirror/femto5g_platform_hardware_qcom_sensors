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
#include <ctype.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <SensorLog.h>
#include <errno.h>
#include <stdlib.h>
#include <vector>
#include <pthread.h>
#include <SensorDiagLog.h>

#ifdef USE_CLOG
#include "clog.h"
#include "clog_ids.h"
#elif defined(USE_DLT)
#include <dlt/dlt.h>
#endif

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "SensorSvc_SensorLog:"

//Config file
#define SENSOR_CONF_PATH "/etc/sensors.conf"

/* Messages longer than this are truncated. */
#define LOG_MAX_LENGTH 512

/**@brief:  Reads a "key=value" config entry out of a config file. A '#'
 *          stops scanning of that line; later lines take precedence if the
 *          key repeats. Never logs, so it is safe to call from anywhere.
 * @param file_name   : Path to configuration file
 * @param config_name : Config name including '=' (eg: DIAG_CLIENTS=)
 * @param out_fmt      : sscanf format specifier for the value
 * @param out          : Address to store the parsed value
 * @return             : 0 on success, else -EINVAL
 */
static int ReadSensorConfig(const char *file_name, const char *config_name,
const char *out_fmt, void *out)
{
    FILE *file;
    char buffer[BUFSIZ];
    char *line;
    int i;
    int ret = -EINVAL;

    file = fopen(file_name, "r");
    if (file == NULL) {
        return -EINVAL;
    }

    while(fgets(buffer, sizeof(buffer), file) != NULL) {
        for(i = 0; i < strlen(buffer); i++) { // iterate through the chars in a line
            if(buffer[i] == '#') { // if char is a #, stop processing chars on this line
                break;
            } else if(buffer[i] == ' ') { // if char is whitespace, continue until something is found
                continue;
            }
            else if(strstr(buffer, config_name)) {
                line = strstr(buffer, "=");
                if (line == nullptr) {
                    continue;
                }
                (void)sscanf(&line[1], out_fmt, out);
                ret = 0;
                break;
            }
        }
    }
    (void)fclose(file);
    return ret;
}

/* 0 (silent) by default, not lazily read from sensors.conf, so there is no
 * conditional or file read on the SENSOR_LOGx hot path. */
static int DEBUG_LEVEL = 0;

int SensorLogDefaultEnabled(SensorLogLevel level)
{
    return (DEBUG_LEVEL >= (int)level) && (DEBUG_LEVEL <= 5);
}


typedef enum {
    ROLE_APP = 0,   /* owns the process, registers the application id */
    ROLE_LIBRARY    /* hosted elsewhere, must not claim the application id */
} SensorLogRole;

typedef struct {
    SensorLogRole role;
    const char   *appId;        /* NULL for libraries */
    const char   *ctxId;
    const char   *description;
} SensorLogIdentity;

/* Ids match the DLT ids already used, so log names do not change. Libraries
 * carry an appId too: CLOG ignores it, but DLT has no library registration. */
static const SensorLogIdentity kIdentities[] = {
    /* SENSOR_LOG_ID_HAL_DAEMON */
    { ROLE_APP,     "SENS", "SHD", "Sensor Hal Daemon" },
    /* SENSOR_LOG_ID_CLIENT_LIB */
    { ROLE_LIBRARY, "SENC", "SCL", "Sensor Client Library" }
};

#define IDENTITY_COUNT ((int)(sizeof(kIdentities) / sizeof(kIdentities[0])))

/* gRefCount: a client application may hold several SensorClient objects, and
 * tearing the backend down on the first destructor would break the others. */
static pthread_mutex_t gLock = PTHREAD_MUTEX_INITIALIZER;
static int gRefCount = 0;
static int gBackendActive = 0;

#ifdef USE_CLOG

static clog_context_t gClogCtx = NULL;
static int gClogAppRegistered = 0;
static int gClogLibRegistered = 0;

static clog_level_t ToClogLevel(SensorLogLevel level)
{
    switch (level)
    {
        case SENSOR_LOG_FATAL:   return CLOG_LEVEL_FATAL;
        case SENSOR_LOG_ERROR:   return CLOG_LEVEL_ERROR;
        case SENSOR_LOG_WARN:    return CLOG_LEVEL_WARN;
        case SENSOR_LOG_INFO:    return CLOG_LEVEL_INFO;
        case SENSOR_LOG_DEBUG:   return CLOG_LEVEL_DEBUG;
        case SENSOR_LOG_VERBOSE: return CLOG_LEVEL_VERBOSE;
        default:                 return CLOG_LEVEL_ERROR;
    }
}

/**@brief:  Maps a component onto its numeric ids in the CLOG registry. */
static void ClogIdsFor(SensorLogComponent component,
                       clog_app_id_t *appNumId, clog_context_id_t *ctxNumId)
{
    switch (component)
    {
        case SENSOR_LOG_ID_HAL_DAEMON:
            *appNumId = (clog_app_id_t)CLOG_APP_SENS;
            *ctxNumId = (clog_context_id_t)CLOG_ACTX_SENS_SHD;
            break;
        case SENSOR_LOG_ID_CLIENT_LIB:
            *appNumId = (clog_app_id_t)0;
            *ctxNumId = (clog_context_id_t)CLOG_LCTX_SCL;
            break;
        default:
            *appNumId = (clog_app_id_t)0;
            *ctxNumId = (clog_context_id_t)0;
            break;
    }
}

/* Caller holds gLock. Returns 1 when a context is registered and usable. */
static int BackendRegister(SensorLogComponent component,
                           const SensorLogIdentity *id)
{
    clog_app_id_t appNumId;
    clog_context_id_t ctxNumId;

    ClogIdsFor(component, &appNumId, &ctxNumId);

    if (id->role == ROLE_APP)
    {
        if (CLOG_REGISTER_APP(appNumId, id->appId, id->description) != CLOG_RETURN_OK)
        {
            return 0;
        }
        gClogAppRegistered = 1;
    }
    else
    {
        /* Libraries must not claim the app id: the host may already own one. */
        if (CLOG_REGISTER_LIBRARY() != CLOG_RETURN_OK)
        {
            return 0;
        }
        gClogLibRegistered = 1;
    }

    gClogCtx = CLOG_REGISTER_CONTEXT(ctxNumId, id->ctxId, id->description,
                                     CLOG_DIAG_SSID_NOT_AVAILABLE,
                                     id->ctxId, id->ctxId);
    if (gClogCtx == NULL)
    {
        if (gClogAppRegistered)
        {
            (void)CLOG_UNREGISTER_APP();
            gClogAppRegistered = 0;
        }
        if (gClogLibRegistered)
        {
            (void)CLOG_UNREGISTER_LIBRARY();
            gClogLibRegistered = 0;
        }
        return 0;
    }

    return 1;
}

/* Caller holds gLock. */
static void BackendUnregister(void)
{
    if (gClogCtx != NULL)
    {
        (void)CLOG_UNREGISTER_CONTEXT(gClogCtx);
        gClogCtx = NULL;
    }
    if (gClogAppRegistered)
    {
        (void)CLOG_UNREGISTER_APP();
        gClogAppRegistered = 0;
    }
    if (gClogLibRegistered)
    {
        (void)CLOG_UNREGISTER_LIBRARY();
        gClogLibRegistered = 0;
    }
}

static int BackendWrite(SensorLogLevel level, const char *buf)
{
    return (CLOG_LOG(gClogCtx, ToClogLevel(level), "%s", buf) == CLOG_RETURN_OK);
}

#elif defined(USE_DLT)

#define DLT_ID_SIZE 5
#define DLT_ENABLED_KEY "DLT_ENABLED="

static DltContext gDltCtx;
static int gDltAppRegistered = 0;

static DltLogLevelType ToDltLevel(SensorLogLevel level)
{
    switch (level)
    {
        case SENSOR_LOG_FATAL:   return DLT_LOG_FATAL;
        case SENSOR_LOG_ERROR:   return DLT_LOG_ERROR;
        case SENSOR_LOG_WARN:    return DLT_LOG_WARN;
        case SENSOR_LOG_INFO:    return DLT_LOG_INFO;
        case SENSOR_LOG_DEBUG:   return DLT_LOG_DEBUG;
        case SENSOR_LOG_VERBOSE: return DLT_LOG_VERBOSE;
        default:                 return DLT_LOG_ERROR;
    }
}

/**@brief:  Reads DLT_ENABLED from the sensor config file.
 * @return  Configured value, or 0 if the key or the file is absent.
 */
static int ReadDltEnabledConfig(void)
{
    int value = 0;

    if (ReadSensorConfig(SENSOR_CONF_PATH, DLT_ENABLED_KEY, "%d", &value) != 0)
    {
        value = 0;
    }
    return value;
}

/* Caller holds gLock. Returns 1 when a context is registered and usable. */
static int BackendRegister(SensorLogComponent component,
                           const SensorLogIdentity *id)
{
    char registeredAppId[DLT_ID_SIZE] = {0};

    (void)component;

    if (!ReadDltEnabledConfig())
    {
        return 0;
    }

    /* DLT has no library registration, so both roles claim the app id - but
     * only when the process has none, so a client's own id is kept. */
    DLT_GET_APPID(registeredAppId);
    if (id->appId != NULL && registeredAppId[0] == '\0')
    {
        DLT_REGISTER_APP(id->appId, id->description);
        gDltAppRegistered = 1;
    }

    if (dlt_register_context(&gDltCtx, id->ctxId, id->description) != DLT_RETURN_OK)
    {
        if (gDltAppRegistered)
        {
            DLT_UNREGISTER_APP();
            gDltAppRegistered = 0;
        }
        return 0;
    }

    return 1;
}

/* Caller holds gLock. */
static void BackendUnregister(void)
{
    DLT_UNREGISTER_CONTEXT(gDltCtx);

    if (gDltAppRegistered)
    {
        DLT_UNREGISTER_APP();
        gDltAppRegistered = 0;
    }
}

static int BackendWrite(SensorLogLevel level, const char *buf)
{
    DLT_LOG(gDltCtx, ToDltLevel(level), DLT_CSTRING(buf));
    return 1;
}

#else /* neither USE_CLOG nor USE_DLT */

static int BackendRegister(SensorLogComponent component,
                           const SensorLogIdentity *id)
{
    (void)component;
    (void)id;
    return 0;
}

static void BackendUnregister(void)
{
}

static int BackendWrite(SensorLogLevel level, const char *buf)
{
    (void)level;
    (void)buf;
    return 0;
}

#endif

#ifdef __cplusplus
extern "C" {
#endif

int SensorLogRegister(SensorLogComponent component)
{
    int active;

    if ((int)component < 0 || (int)component >= IDENTITY_COUNT)
    {
        return 0;
    }

    (void)pthread_mutex_lock(&gLock);

    if (gRefCount == 0)
    {
        gBackendActive = BackendRegister(component, &kIdentities[component]);
    }

    /* Counted even when the backend is unavailable, to keep the matching
     * SensorLogUnregister() balanced. */
    gRefCount++;
    active = gBackendActive;

    (void)pthread_mutex_unlock(&gLock);

    return active;
}

void SensorLogUnregister(void)
{
    (void)pthread_mutex_lock(&gLock);

    if (gRefCount > 0 && --gRefCount == 0)
    {
        if (gBackendActive)
        {
            BackendUnregister();
            gBackendActive = 0;
        }
    }

    (void)pthread_mutex_unlock(&gLock);
}

int SensorLogWrite(SensorLogLevel level, const char *fmt, ...)
{
    char buf[LOG_MAX_LENGTH];
    va_list ap;
    int ret;

    if (fmt == NULL || !gBackendActive)
    {
        return 0;
    }

    va_start(ap, fmt);
    ret = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (ret < 0)
    {
        return 0;
    }

    return BackendWrite(level, buf);
}

int SensorLogBackendActive(void)
{
    return gBackendActive;
}

#ifdef __cplusplus
}
#endif

/* Safety net for hosts that exit without unregistering. No-op once the
 * references are balanced, so an explicit unregister is not undone twice. */
static void __attribute__((destructor)) SensorLogAtUnload(void)
{
    (void)pthread_mutex_lock(&gLock);

    if (gRefCount > 0 && gBackendActive)
    {
        BackendUnregister();
        gBackendActive = 0;
        gRefCount = 0;
    }

    (void)pthread_mutex_unlock(&gLock);
}

/**
 * strtok_safe:  Safer version of strtok
 * @param inp_str:  Input string to search in
 * @param inp_size: Size of input string
 * @param skip: No of characters to skip from input string. '2' means skip first 2 character
 * @param del: Delimeter string. It must end with '\0'
 * @param out_tok: Pointer to string to store the output token. The size of out_tok must be atleast inp_size
 * @return Returns -1 if no more matches. Else returns the starting index of the match
 */

static int strtok_safe(const char *inp_str, const size_t inp_size, const size_t skip, const char *del, char *out_tok)
{
    int idx = skip;
    int out_idx = 0;
    int check_size = 0;
    if (inp_str == NULL)
    {
        return -1;
    }
    check_size = strlen(inp_str);
    if(check_size > inp_size)
    {
        check_size = inp_size;
    }
    if(skip >= check_size)
    {
        return -1;
    }
    while(idx < check_size)
    {
        bool delFound = false;
        for(size_t i=0; i<(check_size - idx); i++)
        {
            if(del[i] == '\0')
            {
                delFound = true;
                break;
            }
            else if(del[i] != inp_str[idx + i])
            {
                break;
            }
        }
        if(delFound)
        {
            out_tok[out_idx] = '\0';
            break;
        }
        if(idx >= inp_size) break;
        out_tok[out_idx] = inp_str[idx];
        idx++;
        out_idx++;
    }
    if(idx < inp_size && inp_str[idx] == '\0')
    {
        out_tok[out_idx] = '\0';
    }
    return idx - out_idx;
}

void SetSensorDebugLevel(int debug_level) {
    DEBUG_LEVEL = debug_level;
}

int SensorReadDebugLevel() {
    int value = 0;

    if (ReadSensorConfig(SENSOR_CONF_PATH, "DEBUG_LEVEL=", "%d", &value) == 0) {
        DEBUG_LEVEL = value;
    }
    return DEBUG_LEVEL;
}


/**
 * frac_to_float_array: Parses list of fractions in the form a1/b1,a2/b2,... and returns a float array
 */
static int frac_to_float_array(char *str_arr, std::vector<float> &out, size_t max_size)
{
    char val_str[1024] = {0};
    char val_str1[21] = {0};
    int numerator, denominator;
    int skip1=0;
    int skip = 0;
    int order = 0;
    std::vector<float> parsed_arr;
    parsed_arr.reserve(max_size);
    while((skip = strtok_safe(str_arr, 1024, skip, ",", val_str)) != -1)
    {
        if(order >= max_size)
        {
            SENSOR_LOGE(LOG_TAG "size of x coefficient array is more than max defined (%lu), skipping rest\n", max_size);
            break;
        }
        skip1 = 0;
        skip1 = strtok_safe(val_str, 21, skip1, "/", val_str1);
        if(skip1 == -1)
        {
            SENSOR_LOGE(LOG_TAG "Invalid coefficient value, entries must be in format a/b\n");
            return 0;
        }
        //Check if all the characters are numeric for numerator
        skip1 = strlen(val_str1);
        for(int i=0; i<skip1; i++)
        {
            if(!isdigit(val_str1[i]))
            {
                SENSOR_LOGE(LOG_TAG "Invalid coefficient value, value (%s) is not numeric\n", val_str1);
                return 0;
            }
        }
        numerator = atoi(val_str1);

        //check if all the characters are numeric for denonimator
        for(int i=skip1+1; i<strlen(val_str); i++)
        {
            if(!isdigit(val_str[i]))
            {
                SENSOR_LOGE(LOG_TAG "Invalid coefficient value, value (%s) is not numeric\n", val_str + skip1+1);
                return 0;
            }
        }
        denominator = atoi(val_str + skip1 + 1);

        if(denominator == 0)
        {
            SENSOR_LOGE(LOG_TAG "Invalid coefficient value, denominator is zero\n");
            return 0;
        }
        parsed_arr.push_back((numerator + 0.0) / denominator);
        skip = skip + strlen(val_str) + 1; //+1 for delimeter ','
        order++;
    }
    out.insert(out.end(), parsed_arr.begin(), parsed_arr.end());
    return order;
}

int CheckDiagEnabled(const char *client_name)
{
    char diag_clients[MAX_DIAG_CLIENTS * MAX_CLIENT_NAME_LEN + 1];//+1 for /0
    int ret = ReadSensorConfig(SENSOR_CONF_PATH, "DIAG_CLIENTS=", "%s", (void*)&diag_clients[0]);
    if(ret != 0)
    {
        SENSOR_LOGE(LOG_TAG "invalid config\n");
        return 0;
    }
    char client[MAX_DIAG_CLIENTS * MAX_CLIENT_NAME_LEN + 1];
    int skip = 0;
    while((skip = strtok_safe(diag_clients, MAX_DIAG_CLIENTS * MAX_CLIENT_NAME_LEN + 1, skip, ",", client)) != -1)
    {
        if(strncmp(client, client_name, MAX_CLIENT_NAME_LEN) == 0)
        {
            return 1;
        }
        skip = skip + strlen(client) + 1; //+1 for delimeter ","
    }
    return 0;
}

int GetFIRCoefficient(std::vector<float> &coef, char *suffix)
{
    char conf_coef[1024];
    char key_val[1024];
    int order = 0;
    int ret = 0;

    snprintf(key_val, 1024, "FIR_COEFFICIENT_%s=", suffix);
    ret = ReadSensorConfig(SENSOR_CONF_PATH, key_val, "%s", (void*)&conf_coef);
    if(ret != 0)
    {
        SENSOR_LOGE(LOG_TAG "invalid config for FIR_COEFFICIENT_%s\n", suffix);
        return -1;
    }
    coef.reserve(MAX_FIR_COEF_ORDER);
    order = frac_to_float_array(conf_coef, coef, MAX_FIR_COEF_ORDER);
    return order == 0 ? -1 : order;
}
