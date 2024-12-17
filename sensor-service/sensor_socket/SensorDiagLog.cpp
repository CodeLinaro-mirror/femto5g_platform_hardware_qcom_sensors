/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the
 * disclaimer below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 * GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 * HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

static sensors_batch_diag_info_t log_acc_buff = {
    .log_type = SENSORDIAGLOG_BUFF_ACCEL,
    .nsamples = 0
};
static sensors_batch_diag_info_t log_gyro_buff = {
    .log_type = SENSORDIAGLOG_BUFF_GYRO,
    .nsamples = 0
};
static sensors_batch_diag_info_t log_acc_live = {
    .log_type = SENSORDIAGLOG_LIVE_ACCEL,
    .nsamples = 0
};
static sensors_batch_diag_info_t log_gyro_live = {
    .log_type = SENSORDIAGLOG_LIVE_GYRO,
    .nsamples = 0
};


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
    bool ret = true;

    sensors_batch_diag_info_t *batch_data = NULL;
    if(evt_type == SENSORDIAGLOG_BUFF_ACCEL)
    {
        batch_data = &log_acc_buff;
    }
    else if(evt_type == SENSORDIAGLOG_BUFF_GYRO)
    {
        batch_data = &log_gyro_buff;
    }
    else if(evt_type == SENSORDIAGLOG_LIVE_ACCEL)
    {
        batch_data = &log_acc_live;
    }
    else if(evt_type == SENSORDIAGLOG_LIVE_GYRO)
    {
        batch_data = &log_gyro_live;
    }
    else
    {
        return false;
    }

    //Set the timestamp of the batch to the first received
    if(batch_data->nsamples == 0)
    {
        batch_data->ts_received = get_monotonic_boottime();
    }

    batch_data->samples[batch_data->nsamples].timestamp = event->timestamp;
    batch_data->samples[batch_data->nsamples].batch_count = batch_count;
    batch_data->samples[batch_data->nsamples].total_count = total_count;
    if(evt_type == SENSORDIAGLOG_BUFF_ACCEL || evt_type == SENSORDIAGLOG_LIVE_ACCEL)
    {
        batch_data->samples[batch_data->nsamples].payload.accel_data.x_uncalib = event->uncalibrated_accelerometer.x_uncalib;
        batch_data->samples[batch_data->nsamples].payload.accel_data.y_uncalib = event->uncalibrated_accelerometer.y_uncalib;
        batch_data->samples[batch_data->nsamples].payload.accel_data.z_uncalib = event->uncalibrated_accelerometer.z_uncalib;
        batch_data->samples[batch_data->nsamples].payload.accel_data.x_bias = event->uncalibrated_accelerometer.x_bias;
        batch_data->samples[batch_data->nsamples].payload.accel_data.y_bias = event->uncalibrated_accelerometer.y_bias;
        batch_data->samples[batch_data->nsamples].payload.accel_data.z_bias = event->uncalibrated_accelerometer.z_bias;
    }
    else if(evt_type == SENSORDIAGLOG_BUFF_GYRO || evt_type == SENSORDIAGLOG_LIVE_GYRO)
    {
        batch_data->samples[batch_data->nsamples].payload.gyro_data.x_uncalib = event->uncalibrated_gyro.x_uncalib;
        batch_data->samples[batch_data->nsamples].payload.gyro_data.y_uncalib = event->uncalibrated_gyro.y_uncalib;
        batch_data->samples[batch_data->nsamples].payload.gyro_data.z_uncalib = event->uncalibrated_gyro.z_uncalib;
        batch_data->samples[batch_data->nsamples].payload.gyro_data.x_bias = event->uncalibrated_gyro.x_bias;
        batch_data->samples[batch_data->nsamples].payload.gyro_data.y_bias = event->uncalibrated_gyro.y_bias;
        batch_data->samples[batch_data->nsamples].payload.gyro_data.z_bias = event->uncalibrated_gyro.z_bias;
    }
    batch_data->nsamples++;

    if(batch_data->nsamples == MAX_DIAG_BATCH_SIZE)
    {
        ret = CommitToDiag(batch_data);
    }
    return ret;
}

bool SensorDiagLog::CommitToDiag(sensors_batch_diag_info_t *batch_data)
{
    if(batch_data->nsamples == 0 || !diagEnabled)
        return false;
    
    bool ret = false;
    void *log = NULL;
    uint64_t log_size = sizeof(sensors_batch_diag_info_t) - sizeof(sensors_diag_info_t) * MAX_DIAG_BATCH_SIZE
            + sizeof(sensors_diag_info_t) * batch_data->nsamples;

    batch_data->pid = getpid();
    //allocates memory from shared pool
    log = log_alloc(SENSORHAL_DIAG_LOG_CODE, log_size);
    if(log != NULL)
    {
        //skip the header while memcpy
        memcpy((uint8_t*)log + sizeof(log_hdr_type), (uint8_t*)batch_data + sizeof(log_hdr_type), log_size - sizeof(log_hdr_type));
        //log_commit release the memory to shared pool. explicit free not required
        log_commit(log);
        ret = true;
    }
    else
    {
        SENSOR_LOGE(LOG_TAG "log_alloc failed \n");
    }
    batch_data->nsamples = 0;

    return ret;
}

bool SensorDiagLog::SendSensorBuffAccelEvent(sensors_event_t *event, uint64_t batch_count)
{
    if(!diagEnabled)
    {
        return false;
    }
    accel_count_buff++;
    return SendSensorEvent(event, batch_count, accel_count_buff, SENSORDIAGLOG_BUFF_ACCEL);
}

bool SensorDiagLog::SendSensorBuffGyroEvent(sensors_event_t *event, uint64_t batch_count)
{
    if(!diagEnabled)
    {
        return false;
    }
    gyro_count_buff++;
    return SendSensorEvent(event, batch_count, gyro_count_buff, SENSORDIAGLOG_BUFF_GYRO);
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
        //Push the pending cache to diag interface
        CommitToDiagAccelBuff();
        CommitToDiagGyroBuff();
        CommitToDiagAccelLive();
        CommitToDiagGyroLive();
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

SensorDiagLog::SensorDiagLog():diagEnabled(false), accel_count(0), gyro_count(0), 
                            accel_count_buff(0), gyro_count_buff(0)
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

bool SensorDiagLog::CommitToDiagAccelBuff()
{
    return CommitToDiag(&log_acc_buff);
}

bool SensorDiagLog::CommitToDiagGyroBuff()
{
    return CommitToDiag(&log_gyro_buff);
}

bool SensorDiagLog::CommitToDiagAccelLive()
{
    return CommitToDiag(&log_acc_live);
}

bool SensorDiagLog::CommitToDiagGyroLive()
{
    return CommitToDiag(&log_gyro_live);
}