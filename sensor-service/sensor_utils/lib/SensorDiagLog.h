/*
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear 
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