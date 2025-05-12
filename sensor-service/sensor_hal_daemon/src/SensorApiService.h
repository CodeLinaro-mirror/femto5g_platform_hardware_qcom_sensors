/* Copyright (c) 2018-2021 The Linux Foundation. All rights reserved.
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
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef SENSORAPISERVICE_H
#define SENSORAPISERVICE_H

#include <string>
#include <mutex>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/un.h>
#include <dlfcn.h>
#include <pthread.h>
#include <errno.h>
#include <signal.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <sensors.h>
#include <SensorIpc.h>
#include <fstream>
#include <linux/input.h>
#include <vector>
#include <string>
#include <functional>
#include <memory>

#include <SensorLog.h>
#include <SensorApiMsg.h>
#include <SensorApiUtils.h>
#include <SensorUtils.h>
#include <SensorHalDaemonClientHandler.h>
#ifdef SENSOR_IVSS_ENABLED
#include <CommonAPI/CommonAPI.hpp>
#include <SensorInterfaceStubImpl.hpp>
#endif
#include <SensorDiagLog.h>
#include <SensorDevice.h>

#ifdef POWERMANAGER_ENABLED
#include <PowerEvtHandler.h>
#endif

#ifdef PTP_SUPPORTED
#include <gptp_helper.h>
#endif

#ifdef SENSOR_HEAD_TYPE_SUPPORT
#include <LocationClientApi.h>
#endif

#ifdef NO_UNORDERED_SET_OR_MAP
    #include <map>
#else
    #include <unordered_map>
#endif

#undef LOG_TAG
#define LOG_TAG "SensorSvc_HalDaemon:"

#define SERVICE_NAME "sensorapiservice"

//Config file
#define SENSOR_CONF_PATH "/etc/sensors.conf"

#ifdef SENSOR_IVSS_ENABLED
using namespace std;
using namespace v1::com::qualcomm::qti::sensor;
#endif
#ifdef SENSOR_HEAD_TYPE_SUPPORT
using namespace location_client;
#endif

enum PowerStateType {
    POWER_STATE_UNKNOWN  = 0,
    POWER_STATE_SUSPEND  = 1,
    POWER_STATE_RESUME   = 2,
    POWER_STATE_SHUTDOWN = 3
};

typedef struct {
    char  AccelName[100];
    char  GyroName[100];
    float MaxAccSampleRate;
    float MaxGyroSampleRate;
    int   MinAccBatchCount;
    int   MinGyroBatchCount;
    int   AccRange;
    int   GyroRange;
    int   DebugLevel;
    int   EnableFIR;
    int   SensorSelfTest;
} configParamToRead;


//SensorConfig Parameters to track the status of sensor configuration
typedef struct {
   int   sensor_id;
   int   type;
   int   Activate;
   int   BatchCount;
   float SamplingRate;
   SelfTestResult selfTest;
   uint64_t selfTestTS;
} SensorConfig;

typedef struct {
    // this stores the client name and the command type that client requests
    // the info will be used to send back command response
    string clientName;
    ESensorMsgID   configMsgId;
} ConfigReqClientData;

// forward declaration
class SensorHalDaemonIPCReceiver;
class SensorHalDaemonQsockReceiver;
#ifdef SENSOR_IVSS_ENABLED
class SensorInterfaceStubImpl;
#endif
class SensorHalDaemonClientHandler;
class SensorDevice;

/******************************************************************************
SensorApiService
******************************************************************************/
class SensorApiService
{
public:
    // singleton instance
    SensorApiService(const SensorApiService&) = delete;
    SensorApiService& operator = (const SensorApiService&) = delete;
    static SensorApiService* getInstance(
            const configParamToRead & configParamRead) {
        if (nullptr == mInstance) {
            mInstance = new SensorApiService(configParamRead);
        }
        return mInstance;
    }
    static void destroy() {
        if (nullptr != mInstance) {
            delete mInstance;
            mInstance = nullptr;
        }
    }
    SensorApiService(const configParamToRead & configParamRead);
    virtual ~SensorApiService();
    // APIs can be invoked by IPC
    void processClientMsg(const string& data);
    // from IPC receiver
    void onListenerReady();
    void onServiceStatusChange(int serviceId, int instanceId, int status, const SensorQsocketSender& refSender);
    //API to Sensor Lib
    bool openSensor(const configParamToRead & configParamRead);
    int  getSensorListDevice(const struct sensor_t **s);
    int  sensorActivate(int sensor_id , int enable);
    int  sensorSetBatch(int sensor_id, int64_t delay, int64_t latency);
    // other APIs
    int  newClient(SensorAPIClientRegisterReqMsg*);
    void deleteClient(SensorAPIClientDeregisterReqMsg*);
    int  startTracking(SensorAPIStartTrackingReqMsg*);
    int  startBatching(SensorAPIStartBatchingReqMsg*);
    int  activateSensor(SensorAPIEnableReqMsg*);
    void deleteEapClientByIds(int id1, int id2);
    unordered_map<string, SensorHalDaemonClientHandler*>::iterator deleteClientbyName(const string clientname);
#ifdef POWERMANAGER_ENABLED
    void onPowerEvent(PowerStateType powerState, SensorCapabilitiesMask mask);
#endif
    static mutex mMutex;
    pthread_t mSensorThreadtid;
    pthread_t mBufferThreadtid;
    // Client propery database
    unordered_map<string, SensorHalDaemonClientHandler*> mClients;
    unordered_map<uint32_t, ConfigReqClientData> mConfigReqs;
    struct sensor_list *mSensorList;
    int mSensorCount;
    SensorConfig *mSensor;
    uint32_t mSensorClient;
    struct sensors_module_t *mhmi;
    struct hw_device_t *mdev;
    struct sensors_poll_device_t *mpoll_dev_v0;
    struct sensors_poll_device_1 *mpoll_dev;
    // Configration
    float mMaxAccSampleRate;
    float mMaxGyroSampleRate;
    int   mMinAccBatchCount;
    int   mMinGyroBatchCount;
    int   mAccRange;
    int   mGyroRange;
    bool  mBufferSupported;
    bool  mBufferDeleted;
    bool  mTempSupported;
    bool  mMlcSupported;
    int   mBatchConst;
    int   mEnableFIR;
    float rot[3][3];
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
    //Mlc sesnros list
    struct sensor_mlc_case_list *mSensorMlcCaseList;
    int mSensorMlcCaseCount;
    //SensorDevice
    SensorDevice *mSensorDevice;
#ifdef PTP_SUPPORTED
    bool mGptpInitialized;
#endif
#ifdef SENSOR_IVSS_ENABLED
    shared_ptr<SensorInterfaceStubImpl> myService;
#endif
#ifdef SENSOR_HEAD_TYPE_SUPPORT
    void enableHeadingSensor();
    void disableHeadingSensor();
#endif
    float nearBySamplingRate(float input_rates[], float target_rate);
    int   nearByBatchCount(int minBatchCount, int ReqBatchCount, float input_rate, float output_rate, int factor);
private:
    //API to SHD
    void  getSensorList(SensorAPIListReqMsg*);
    void  getSensorTemp(SensorAPITempReqMsg*);
    void  getSensorBufferData(SensorAPIBufferDataReqMsg*);
    int   sensorCofig(SensorAPIStartBatchingReqMsg*);
    void  sensorSelfTest(SensorAPISelfTestReqMsg*);
    void  setEulerAngles(SensorAPIEulerAnglesReqMsg*);
    void  onSelfTestRequest(SensorHalDaemonClientHandler*,
		    int sensor_id, SelfTestType selfTestType, int request_id);
    int   sensorSelfTest(int sensor_id, SelfTestType selfTestType, SelfTestResult &SelfTestResult,
		    SelfTestResultType &resultType, int &AccelTest, int &GyroTest, bool voluntary);
    //threads
    static void* sendSensorDataToClients(void *arg);
    static void* bufferDataprocessTask(void *arg);
    //MLC API's
    void sensorEnableMLCCase(SensorAPIMLCCaseEnableMsg*);
    //Buffer API's
    bool checkSensorBufferSupport();
    void sensorBufferReadThread();
    bool readSensorBufferData(const string clientname);
    void bufferDataScaling(int type, sensors_event_t *event);
    bool getBufferedSample(int type,  FILE* fd, sensors_event_t *event);
    bool writeToBufferFile(bool enable);
    // private utilities
    inline SensorHalDaemonClientHandler* getClient(const string& clientname) {
	    // find client from property db
	    auto client = mClients.find(clientname);
	    if (client == end(mClients)) {
		    SENSOR_LOGE(LOG_TAG "Failed to find client %s\n", clientname.c_str());
		    return nullptr;
	    }
	    return client->second;
    }
    inline SensorHalDaemonClientHandler* getClient(const char* socketName) {
	    string clientname(socketName);
	    return getClient(clientname);
    }
    const char* getClientNameByIds(int id1, int id2);
    // singleton instance
    static SensorApiService *mInstance;
    // IPC interface
    SensorHalDaemonIPCReceiver* mIpcReceiver;
    // QSocket interface
    SensorHalDaemonQsockReceiver* mQsockReceiver;
#ifdef POWERMANAGER_ENABLED
    // power event observer
    PowerEvtHandler* mPowerEventObserver;
#endif
    PowerStateType  mPowerState;
    //To Store Buffer file paths
    string mAccBootSample;
    string mGyroBootSample;
    //To send diag logs
    SensorDiagLog mDiagLogger;
    //To wake up Buffer Thread
    pthread_mutex_t mHalBuffMutex;
    pthread_cond_t mHalBuffCond;
#ifdef SENSOR_HEAD_TYPE_SUPPORT
    static void onLocationCapabilitiesCb(location_client::LocationCapabilitiesMask mask);
    static void onLocationResponseCb(location_client::LocationResponse response);
    static void onGnssLocationCb(const location_client::GnssLocation& location);
    void onSensorHeadingDataReadCb(float heading, float accuracy, uint64_t ts);
#endif
};

#endif //SENSORAPISERVICE_H
