/**
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear 
*/

#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <SensorLog.h>
#include <SensorDiagLog.h>

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "SensorHalDaemonDiag:"

static uint64_t get_monotonic_boottime()
{
    struct timespec ts;
    if(clock_gettime(CLOCK_BOOTTIME, &ts) == 0)
    {
        return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
    }
    else
    {
        SENSOR_LOGE(LOG_TAG "failed to get timestamp : %d (%s)\n", errno, strerror(errno));
    }
    return 0;
}

bool SensorDiagLog::SendSensorEvent(sensors_event_t *event, uint64_t batch_count, uint64_t total_count, sensor_diag_log_type evt_type)
{
    sensors_diag_info_t *log = NULL;
    bool ret = false;

    //allocates memory from shared pool
    log = (sensors_diag_info_t *)log_alloc(SENSORHAL_DIAG_LOG_CODE, sizeof(sensors_diag_info_t));
    if(log != NULL)
    {
        //log_alloc sets the header
        log->log_type = (uint8_t) evt_type;
        log->pid = getpid();
        log->timestamp = event->timestamp;
        log->batch_count = batch_count;
        log->total_count = total_count;
        log->ts_received = get_monotonic_boottime();
        if(evt_type == SENSORDIAGLOG_BUFF_ACCEL || evt_type == SENSORDIAGLOG_LIVE_ACCEL)
        {
            log->payload.accel_data.x_uncalib = event->uncalibrated_accelerometer.x_uncalib;
            log->payload.accel_data.y_uncalib = event->uncalibrated_accelerometer.y_uncalib;
            log->payload.accel_data.z_uncalib = event->uncalibrated_accelerometer.z_uncalib;
            log->payload.accel_data.x_bias = event->uncalibrated_accelerometer.x_bias;
            log->payload.accel_data.y_bias = event->uncalibrated_accelerometer.y_bias;
            log->payload.accel_data.z_bias = event->uncalibrated_accelerometer.z_bias;
            //log_commit release the memory to shared pool. explicit free not required
            log_commit(log);
            ret = true;
        }
        else if(evt_type == SENSORDIAGLOG_BUFF_GYRO || evt_type == SENSORDIAGLOG_LIVE_GYRO)
        {
            log->payload.gyro_data.x_uncalib = event->uncalibrated_gyro.x_uncalib;
            log->payload.gyro_data.y_uncalib = event->uncalibrated_gyro.y_uncalib;
            log->payload.gyro_data.z_uncalib = event->uncalibrated_gyro.z_uncalib;
            log->payload.gyro_data.x_bias = event->uncalibrated_gyro.x_bias;
            log->payload.gyro_data.y_bias = event->uncalibrated_gyro.y_bias;
            log->payload.gyro_data.z_bias = event->uncalibrated_gyro.z_bias;
            //log_commit release the memory to shared pool. explicit free not required
            log_commit(log);
            ret = true;
        }
        else
        {
            SENSOR_LOGE(LOG_TAG "Invalid evt_type\n");
            log_free(log);
        }
    }
    return ret;
}

bool SensorDiagLog::SendSensorBuffAccelEvent(sensors_event_t *event, uint64_t total_count)
{
    if(!diagEnabled)
    {
        return false;
    }
    return SendSensorEvent(event, total_count, total_count, SENSORDIAGLOG_BUFF_ACCEL);
}

bool SensorDiagLog::SendSensorBuffGyroEvent(sensors_event_t *event, uint64_t total_count)
{
    if(!diagEnabled)
    {
        return false;
    }
    return SendSensorEvent(event, total_count, total_count, SENSORDIAGLOG_BUFF_GYRO);
}

bool SensorDiagLog::SendSensorLiveAccelEvent(sensors_event_t *event, uint64_t batch_count)
{
    if(!diagEnabled)
    {
        return false;
    }
    accel_count++;
    return SendSensorEvent(event, batch_count, accel_count, SENSORDIAGLOG_LIVE_ACCEL);
}

bool SensorDiagLog::SendSensorLiveGyroEvent(sensors_event_t *event, uint64_t batch_count)
{
    if(!diagEnabled)
    {
        return false;
    }
    gyro_count++;
    return SendSensorEvent(event, batch_count, gyro_count, SENSORDIAGLOG_LIVE_GYRO);
}

bool SensorDiagLog::EnableDiag()
{
    if(!diagEnabled)
    {
        if(Diag_LSM_Init(NULL))
        {
            diagEnabled = true;
        }
        else
        {
            SENSOR_LOGE(LOG_TAG "Diag_LSM_Init failed\n");
        }
    }
    else
    {
        SENSOR_LOGI(LOG_TAG "Diag is already enabled\n");
    }
    return diagEnabled;
}

bool SensorDiagLog::DisableDiag()
{
    if(diagEnabled)
    {
        if(Diag_LSM_DeInit())
        {
            diagEnabled = false;
        }
        else
        {
            SENSOR_LOGE(LOG_TAG "Diag_LSM_DeInit failed\n");
        }
    }
    else
    {
        SENSOR_LOGI(LOG_TAG "Diag is already disabled\n");
    }
    return !diagEnabled;
}

SensorDiagLog::SensorDiagLog():diagEnabled(false), accel_count(0), gyro_count(0)
{}

SensorDiagLog::~SensorDiagLog()
{
    DisableDiag();
}

bool SensorDiagLog::IsEnabled()
{
    return diagEnabled;
}

bool SensorDiagLog::SendTestLog()
{
    sensors_event_t evt;
    evt.uncalibrated_accelerometer.x_uncalib = 1.1;
    evt.uncalibrated_accelerometer.y_uncalib = -1.1;
    evt.uncalibrated_accelerometer.z_uncalib = 4.5;
    evt.uncalibrated_accelerometer.x_bias = 0.0;
    evt.uncalibrated_accelerometer.y_bias = -324234.3;
    evt.uncalibrated_accelerometer.z_bias = 112.323213;
    evt.timestamp = 1224324;
    return SendSensorEvent(&evt, 1123, 1334, SENSORDIAGLOG_BUFF_ACCEL);
}