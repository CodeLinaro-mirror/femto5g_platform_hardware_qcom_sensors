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

#ifndef SENSORDIAGLOG_H
#define SENSORDIAGLOG_H

#include <stdint.h>
#include <stdio.h>

#include <diag/log.h>
#include <diag/diag_lsm.h>
#include <sensors.h>

#define SENSORHAL_DIAG_LOG_CODE LOG_DIAG_WRAPPED_KEY_INFO_C
#define MAX_DIAG_CLIENTS                                    20
#define MAX_CLIENT_NAME_LEN                                 20

#define DIAG_CLIENT_SHD                                     "SHD"
#define DIAG_CLIENT_SENSOR_CLIENT                           "CLIENT"

typedef enum{
    SENSORDIAGLOG_BUFF_ACCEL = 0,
    SENSORDIAGLOG_BUFF_GYRO = 1,
    SENSORDIAGLOG_LIVE_ACCEL = 2,
    SENSORDIAGLOG_LIVE_GYRO = 3
} sensor_diag_log_type;

typedef uncalibrated_event_t sensors_diag_accel_data_t;
typedef uncalibrated_event_t sensors_diag_gyro_data_t;

typedef union{
    sensors_diag_accel_data_t accel_data;
    sensors_diag_gyro_data_t gyro_data;
}__attribute__((packed)) diag_payload_t;

typedef struct {
    log_hdr_type hdr;
    uint8_t log_type; //sensors_diag_log_type
    int32_t pid;                //Pid of the process reporting this event
    uint64_t timestamp;         //Timestamp copied from sensor event
    uint64_t batch_count;       //Indicates the sequence number for each batch
    uint64_t total_count;       //Indicates the total sequence number for each sensor type
    uint64_t ts_received;       //Timestamp the data was received by the diag client
    diag_payload_t payload;     //payload data for diag
} __attribute__((packed)) sensors_diag_info_t;


/**
 * class SensorDiagLog
 * Exposes methods to send sensor data to diag interface via Logging service
 */
class SensorDiagLog
{
private:
    /**
     * SendSensorEvent  : Sends diag data from the sensor event 
     * @param event     : Sensor event to extract info from
     * @param count     : Event count to send
     * @param evt_type  : Diag log event type
     * @return boolean  : Returns true if sending to diag interface was successful, else returns false.
     */
    bool SendSensorEvent(sensors_event_t *event, uint64_t batch_count, uint64_t total_count, sensor_diag_log_type evt_type);
protected:
public:
    SensorDiagLog();
    ~SensorDiagLog();
    
    /**
     * SendSensorBuffAccelEvent : Sends diag data from the buffer sensor accel event 
     * @param event             : Sensor Accel event to extract info from
     * @param total_count       : Accel event count to send
     * @return boolean          : Returns true if sending to diag interface was successful, else returns false.
     */
    bool SendSensorBuffAccelEvent(sensors_event_t *event, uint64_t total_count);

    /**
     * SendSensorBuffGyroEvent  : Sends diag data from the buffer sensor gyro event 
     * @param event             : Sensor Gyro event to extract info from
     * @param total_count       : Gyro event count to send
     * @return boolean          : Returns true if sending to diag interface was successful, else returns false.
     */
    bool SendSensorBuffGyroEvent(sensors_event_t *event, uint64_t total_count);

    /**
     * SendSensorLiveAccelEvent : Sends diag data from the live sensor accel event 
     * @param event             : Sensor Accel event to extract info from
     * @param count             : Accel event count to send
     * @return boolean          : Returns true if sending to diag interface was successful, else returns false.
     */
    bool SendSensorLiveAccelEvent(sensors_event_t *event, uint64_t batch_count);

    /**
     * SendSensorLiveGyroEvent  : Sends diag data from the live sensor gyro event 
     * @param event             : Sensor Gyro event to extract info from
     * @param count             : Gyro event count to send
     * @return boolean          : Returns true if sending to diag interface was successful, else returns false.
     */
    bool SendSensorLiveGyroEvent(sensors_event_t *event, uint64_t batch_count);

    /**
     * DisableDiag      : Disable diag logging service
     * @return boolean  : Returns true if disabling was successful, else returns false.
     */
    bool DisableDiag();

    /**
     * EnableDiag       : Enable diag logging service. This method must be called before sending the first diag packet.
     * @return boolean  : Returns true if enabling was successful, else returns false.
     */
    bool EnableDiag();

    /**
     * IsEnabled        : Returns true is diag is enabled, else returns flase
     */
    bool IsEnabled();

    bool SendTestLog();

private:
    alignas(sizeof(int32_t)) bool diagEnabled;
    uint64_t accel_count;
    uint64_t gyro_count;
protected:
public:
};

#endif //SENSORDIAGLOG_H