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
 * Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
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

#define ASM330LHHX_ACC_SEARCH   "asm330lhhx_accel"
#define ASM330LHHX_GYRO_SEARCH  "asm330lhhx_gyro"

#define SMI230_TEMP_SEARCH         "SMI230ACC"
#define SMI230_GYR_SEARCH          "SMI230GYRO"

#define SELFTEST_WAIT_TIME 3600000000000LL

#define HEADING_ODR_IN_MS 100

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
//MLC bin path
#define PATH_MLC_BINARY  "/lib/firmware/st_asm330lhhx_mlc.bin"

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
    int   SensorType;
    char  SensorHalLibPath[100];
    char  AccelName[100];
    char  GyroName[100];
    int   DynamicConfigEnabled;
    float MaxAccSampleRate;
    float MaxGyroSampleRate;
    int   MinAccBatchCount;
    int   MinGyroBatchCount;
    int   AccRange;
    int   GyroRange;
    int   AccBuffRange;
    int   GyroBuffRange;
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
} SensorConfig;

//To check sensor type defined in config file
typedef enum
{
  SENSOR_UNKN = 0,
  SENSOR_ASM330,
  SENSOR_IAM20680,
  SENSOR_SMI130,
  SENSOR_SMI230,
  SENSOR_BMI160
} sensorType;

typedef struct {
    // this stores the client name and the command type that client requests
    // the info will be used to send back command response
    std::string clientName;
    ESensorMsgID   configMsgId;
} ConfigReqClientData;

// forward declaration
class SensorHalDaemonIPCReceiver;
class SensorHalDaemonQsockReceiver;
#ifdef SENSOR_IVSS_ENABLED
class SensorInterfaceStubImpl;
#endif
class SensorHalDaemonClientHandler;

/**
 * set_buff_scaling_factor: Sets the scaling factor based on dynamic range for buffer data
 * @param sensorType: Sensor id (1:asm, 2:iam, 3:smi130, 4:smi230)
 * @param accRange: Range of accel to set
 * @param gyroRange: Range of gyro o set
 */
void set_buff_scaling_factor(int sensorType, int accRange, int gyroRange);

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
    void processClientMsg(const std::string& data);
    static void requestStop();
    void stopInternal();

    // from IPC receiver
    void onListenerReady();
    void onServiceStatusChange(int serviceId, int instanceId, int status, const SensorQsocketSender& refSender);

    // other APIs
    std::unordered_map<std::string, SensorHalDaemonClientHandler*>::iterator deleteClientbyName(const std::string clientname);
    void deleteEapClientByIds(int id1, int id2);

#ifdef POWERMANAGER_ENABLED
    void onPowerEvent(PowerStateType powerState, SensorCapabilitiesMask mask);
#endif

    bool open_sensor(const configParamToRead & configParamRead);
    int get_sensor_list(const struct sensor_t **s);
    int sensor_activate(int sensor_id , int enable);
    int sensor_set_batch(int sensor_id, int64_t delay, int64_t latency);
    static void* send_sensor_data_to_clients(void *arg);
    static void* bufferDataprocessTask(void *arg);
    static std::mutex mMutex;
    pthread_t mSensorThreadtid;
    pthread_t mBufferThreadtid;
    pthread_t mMlcThreadtid;
    pthread_t sig_thread;
    static bool mRequestStop;
    // Client propery database
    std::unordered_map<std::string, SensorHalDaemonClientHandler*> mClients;
    std::unordered_map<uint32_t, ConfigReqClientData> mConfigReqs;
    int  newClient(SensorAPIClientRegisterReqMsg*);
    void  deleteClient(SensorAPIClientDeregisterReqMsg*);
    int  startTracking(SensorAPIStartTrackingReqMsg*);
    int  startBatching(SensorAPIStartBatchingReqMsg*);
    int  activateSensor(SensorAPIEnableReqMsg*);
    void  getSensorList(SensorAPIListReqMsg*);
    struct sensor_list *mSensorList;
    int mSensorCount;
    SensorConfig *mSensor;
    bool GptpInitialized;
#ifdef SENSOR_IVSS_ENABLED
    std::shared_ptr<SensorInterfaceStubImpl> myService;
#endif
    float rot[3][3];
#ifdef SENSOR_HEAD_TYPE_SUPPORT
    void EnableHeadingSensor();
    void DisableHeadingSensor();
#endif
private:
    void  getSensorTemp(SensorAPITempReqMsg*);
    void  getSensorBufferData(SensorAPIBufferDataReqMsg*);
    int   SensorCofig(SensorAPIStartBatchingReqMsg*);
    void  sensorSelfTest(SensorAPISelfTestReqMsg*);
    void  setEulerAngles(SensorAPIEulerAnglesReqMsg*);
    void  onSelfTestRequest(SensorHalDaemonClientHandler*,
		    int sensor_id, SelfTestType selfTestType, int request_id);
    int   SensorSelfTest(int sensor_id, SelfTestType selfTestType, SelfTestResult &SelfTestResult,
		    SelfTestResultType &resultType, int &AccelTest, int &GyroTest);
    void  onPowerEventSelfTest();
    void  GetSupportedSamplingRateAndRange(struct sensor_list *s);
    int   NearByBatchCount(int minBatchCount, int ReqBatchCount, float input_rate, float output_rate, int factor);
    float NearBySamplingRate(float input_rates[], float target_rate);

    //MLC API's
    bool LoadMLC(const char *mcl_fw_name);
    void SensorEnableMLCCase(SensorAPIMLCCaseEnableMsg*);
    int  SetPowerMode(int handle, int mode);
    bool SensorMlcEnableEvents(char *mlc_case_name, int enable);
    static void* mlcPollEvents(void *arg);
    void pollEvents(void);

    /* Self-Test Variables */
    int SelfTestResultAccel;
    int SelfTestResultGyro;
    uint64_t Acceltimestamp;
    uint64_t Gyrotimestamp;
    uint64_t timestamp;

    //Temperature API's
    int  tempSensorDataPollTask(float* temperature);
    bool tempSensorDataInit();
    int  readTempASM(float* temperature);
    int  readTempBMI(float* temperature);
    int  readTempIAM(float* temperature);
    int  readTempSMI(float* temperature);
    int  readTempSMI230(float* temperature);

    //Buffer API's
    bool CheckBufferReadFile();
    void SensorBuffread();
    bool ReadSensorBufferData(const std::string clientname);
    void bufferDataScaling(int SensorType, sensors_event_t *event);
    bool getBufferedSample(int SensorType,  FILE* fd, sensors_event_t *event);
    bool WritetoBufferFile(bool enable);
    // private utilities
    inline SensorHalDaemonClientHandler* getClient(const std::string& clientname) {
	    // find client from property db
	    auto client = mClients.find(clientname);
	    if (client == std::end(mClients)) {
		    SENSOR_LOGE(LOG_TAG "Failed to find client %s\n", clientname.c_str());
		    return nullptr;
	    }
	    return client->second;
    }

    inline SensorHalDaemonClientHandler* getClient(const char* socketName) {
	    std::string clientname(socketName);
	    return getClient(clientname);
    }
    const char* getClientNameByIds(int id1, int id2);


    // singleton instance
    static SensorApiService *mInstance;
    // IPC interface
    SensorHalDaemonIPCReceiver* mIpcReceiver;
    // QSocket interface
    SensorHalDaemonQsockReceiver* mQsockReceiver;

    uint32_t mSensorClient;
    int mSensorType;
    struct sensors_module_t *mhmi;
    struct hw_device_t *mdev;
    struct sensors_poll_device_t *mpoll_dev_v0;
    struct sensors_poll_device_1 *mpoll_dev;

#ifdef POWERMANAGER_ENABLED
    // power event observer
    PowerEvtHandler* mPowerEventObserver;
#endif
    PowerStateType  mPowerState;

    //Mlc sesnros list
    struct sensor_mlc_case_list *mSesnorMlcCaseList;
    int mSensorMlcCaseCount;

    // Configration
    float mMaxAccSampleRate;
    float mMaxGyroSampleRate;
    int   mMinAccBatchCount;
    int   mMinGyroBatchCount;
    int   mAccRange;
    int   mGyroRange;
    int   mAccBuffRange;
    int   mGyroBuffRange;
    bool  mBufferSupported;
    bool  mBufferDeleted;
    bool  mTempSupported;
    bool  mMlcSupported;
    int   mBatchConst;
    int   mDynamicConfigEnabled;
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
    int   mEnableFIR;
    SensorDiagLog mDiagLogger;



    //Temperature file pointers
    struct asmFilePtr
    {
      std::ifstream *tScaleFile;
      std::ifstream *tOffsetFile;
      std::ifstream *tRawDataFile;
    };
    struct smiFilePtr
    {
      std::ifstream *tempFile;
    };
    struct bmiFilePtr
    {
      std::ifstream *tTempFile;
    };
    struct iamFilePtr
    {
     std::ifstream *dataFile;
    };
    union tempFilePtr
    {
      asmFilePtr asmTempFile;
      smiFilePtr smiTempFile;
      bmiFilePtr bmiTempFile;
      iamFilePtr iamTempFile;
    };
    tempFilePtr mTempFilePtr;

    //To Store Buffer file paths
    std::string mAccBootSample;
    std::string mGyroBootSample;

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
