/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef ANDROID_SENSOR_CORE_LIB
#define ANDROID_SENSOR_CORE_LIB

#include <cstdint>
#include <string>
#include <vector>
#include <cutils/log.h>

#define ACCEL_CALIBRATED_SENSOR_ID   3
#define ACCEL_UNCALIBRATED_SENSOR_ID 1 
#define GYRO_CALIBRATED_SENSOR_ID    4
#define GYRO_UNCALIBRATED_SENSOR_ID  16
#define HEADING_SENSOR_ID   5

#define DEBUG_LEVEL 3
#define SENSOR_TAG "SensorInterface_AIDL:"

#define LOGCAT_ENABLED

#define IF_SENSOR_LOGE if(( DEBUG_LEVEL >= 1) && ( DEBUG_LEVEL <= 5))
#define IF_SENSOR_LOGW if(( DEBUG_LEVEL >= 2) && ( DEBUG_LEVEL <= 5))
#define IF_SENSOR_LOGI if(( DEBUG_LEVEL >= 3) && ( DEBUG_LEVEL <= 5))
#define IF_SENSOR_LOGD if(( DEBUG_LEVEL >= 4) && ( DEBUG_LEVEL <= 5))

#ifdef LOGCAT_ENABLED
#define SENSOR_LOGE(...) IF_SENSOR_LOGE { ALOGE(__VA_ARGS__); }
#define SENSOR_LOGW(...) IF_SENSOR_LOGW { ALOGW(__VA_ARGS__); }
#define SENSOR_LOGI(...) IF_SENSOR_LOGI { ALOGI(__VA_ARGS__); }
#define SENSOR_LOGD(...) IF_SENSOR_LOGD { ALOGD(__VA_ARGS__); }
#else
#define SENSOR_LOGE(...) IF_SENSOR_LOGE { printf(__VA_ARGS__); }
#define SENSOR_LOGW(...) IF_SENSOR_LOGW { printf(__VA_ARGS__); }
#define SENSOR_LOGI(...) IF_SENSOR_LOGI { printf(__VA_ARGS__); }
#define SENSOR_LOGD(...) IF_SENSOR_LOGD { printf(__VA_ARGS__); }
#endif

using std::vector;
using std::string;

// Define the SensorCoreData struct
struct SensorCoreData {
    int32_t sensorId;
    int32_t Type;
    int64_t timestamp;
    int64_t gptptimestamp;
    vector<float> xyz;
};

// Define the SensorCoreList struct
struct SensorCoreList {
    string name;
    string vendor;
    int32_t sensorVersion;
    float resolution;
    float maxRange;
    int32_t sensorId;
    int32_t type;
    float maxSamplingRate;
    int32_t minBatchCount;
    int32_t maxBatchCount;
    vector<float> odr;
};

// Define the SensorCore class
class SensorCore {
private:
    std::vector<SensorCoreData> sensorDataVector;
    std::vector<SensorCoreList> sensorListVector;
public:
    void SensorCore_Init();
    void SensorCore_getSensorList(std::vector<SensorCoreList> &sensorVector, int32_t *sensorcount);
    void SensorCore_acitvateSensor(int32_t in_sensorHandle, bool in_enabled);
    void SensorCore_configSensor(int32_t in_sensorHandle, int64_t in_samplingPeriodNs,int64_t in_maxReportLatencyNs);
    uint64_t SensorCore_getBootTimeFromPtpTime(uint64_t ptp_time_ns);
    void SensorCore_Deinit();
    virtual ~SensorCore() {}
    virtual void onNewSensorsData(std::vector<SensorCoreData> &data) = 0;

    // Setter for SensorCoreData
    void setSensorCoreData(int32_t sensorId, int32_t Type, int64_t timestamp, int64_t gptptimestamp, vector<float> xyz) {
        SensorCoreData data = {sensorId, Type, timestamp, gptptimestamp, xyz};
        sensorDataVector.push_back(data);
    }

    // Getter for SensorCoreData
    std::vector<SensorCoreData> getSensorCoreData() {
        return sensorDataVector;
    }

    // Setter for SensorCoreList
    void setSensorCoreList(string name, string vendor, int32_t sensorVersion,
		    float resolution, float maxRange, int32_t sensorId, int32_t type,
		    float maxSamplingRate, int32_t minBatchCount, int32_t maxBatchCount,
		    vector<float> odr) {
       SensorCoreList list = { name, vendor, sensorVersion, resolution, maxRange, sensorId, type,
	                       maxSamplingRate, minBatchCount, maxBatchCount, odr};
       sensorListVector.push_back(list);
    }

    // Getter for SensorCoreList
    std::vector<SensorCoreList> getSensorCoreList() {
        return sensorListVector;
    }
};

#endif /* ANDROID_SENSOR_CORE_LIB */
