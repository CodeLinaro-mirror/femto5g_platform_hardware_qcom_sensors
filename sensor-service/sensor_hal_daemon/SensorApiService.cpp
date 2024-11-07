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
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
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

#include <stdint.h>
#include <sys/stat.h>
#include <dlfcn.h>
#include <memory>
#include <algorithm>
#include <SensorApiMsg.h>
#include <SensorList.h>
#include <math.h>
#include <SensorHalDaemonIPCReceiver.h>
#include <SensorHalDaemonIPCSender.h>
#include <SensorHalDaemonClientHandler.h>
#include <SensorApiService.h>
#include <thread>
#ifdef POWERMANAGER_ENABLED
#include <PowerEvtHandler.h>
#endif

#define ASMLIB "/usr/lib/libasm330sensors.so.1.0.0"
#define IAMLIB "/usr/lib/libiam20680sensors.so.1"
#define SMI130LIB "/usr/lib/libsmi130sensors.so.1"
#define SMI230LIB "/usr/lib/libsmi230sensors.so.1"

using namespace std;

/******************************************************************************
SensorApiService - static members
******************************************************************************/
SensorApiService* SensorApiService::mInstance = nullptr;
std::mutex SensorApiService::mMutex;
/******************************************************************************
SensorApiService - constructors
******************************************************************************/
SensorApiService::SensorApiService(const configParamToRead & configParamRead) :
    mSensorCount(0),
    mSensorList(nullptr),
    mSensorMlcCaseCount(0),
    mSesnorMlcCaseList(nullptr),
    mSensorClient(0),
    mSensor(nullptr),
    mSensorType(configParamRead.SensorType),
    mDynamicConfigEnabled(configParamRead.DynamicConfigEnabled),
    mMaxAccSampleRate(configParamRead.MaxAccSampleRate),
    mMaxGyroSampleRate(configParamRead.MaxGyroSampleRate),
    mMinAccBatchCount(configParamRead.MinAccBatchCount),
    mMinGyroBatchCount(configParamRead.MinGyroBatchCount),
    mAccRange(configParamRead.AccRange),
    mGyroRange(configParamRead.GyroRange),
    mAccBuffRange(configParamRead.AccBuffRange),
    mGyroBuffRange(configParamRead.GyroBuffRange),
    mVariableCountBatching(configParamRead.VariableCountBatching),
    mhmi(nullptr),
    mdev(nullptr),
    mpoll_dev_v0(nullptr),
    mpoll_dev(nullptr),
    mBufferSupported(false),
    mTempSupported(false),
    mBufferDeleted(false),
    mMlcSupported(false)
#ifdef POWERMANAGER_ENABLED
    ,mPowerEventObserver(nullptr)
#endif
{
    SENSOR_LOGI(LOG_TAG "SensorApiService constructor is called\n");

    //Check Sensor Availability
    if(!open_sensor(configParamRead)) {
	SENSOR_LOGE(LOG_TAG "no sensor supported \n");
	return;
    }

#ifdef POWERMANAGER_ENABLED
    // register power event handler
    mPowerEventObserver = PowerEvtHandler::getPwrEvtHandler(this);
    if (nullptr == mPowerEventObserver) {
        SENSOR_LOGE(LOG_TAG "Failed to regiseter Powerevent handler");
        return;
    }
#endif

    // create IPC receiver
    mIpcReceiver = new SensorHalDaemonIPCReceiver(this);
    if (nullptr == mIpcReceiver) {
        SENSOR_LOGE(LOG_TAG "Failed to create SensorHalDaemonIPCReceiver\n");
        return;
    }

    // create Qsock receiver
    mQsockReceiver = new SensorHalDaemonQsockReceiver(this);
    if (nullptr == mQsockReceiver) {
	    SENSOR_LOGE(LOG_TAG "Failed to create SensorHalDaemonQsockReceiver\n");
	    return;
    }

    mInstance = this;

    // start receiver - never return
    SENSOR_LOGI(LOG_TAG "Ready, start Ipc Receiver\n");
    // blocking: set to false
    mIpcReceiver->start(false);

    SENSOR_LOGI(LOG_TAG "Ready, start qsock Receiver\n");
    // blocking: set to true
    mQsockReceiver->start(true);
}

/******************************************************************************
SensorApiService - Destructors
******************************************************************************/
SensorApiService::~SensorApiService() {
    SENSOR_LOGI(LOG_TAG "SensorApiService Destructor is called\n");
    if(mSensorType == 3 || mSensorType == 4){
        mpoll_dev_v0->common.close(&mpoll_dev_v0->common);
    }

    // stop ipc receiver thread
    if (nullptr != mIpcReceiver) {
        mIpcReceiver->stop();
        delete mIpcReceiver;
    }

    if (nullptr != mQsockReceiver) {
        mQsockReceiver->stop();
        delete mQsockReceiver;
    }

    //Delete mSensor memory
    if (nullptr != mSensor) {
	delete mSensor;
	mSensor = nullptr;
    }

    //Delete mSensorList memory
    if (nullptr != mSensorList) {
	delete mSensorList;
	mSensorList = nullptr;
    }

    //Delete mSesnorMlcCaseList memory
    if (nullptr != mSesnorMlcCaseList) {
        delete mSesnorMlcCaseList;
        mSesnorMlcCaseList = nullptr;
    }

    SENSOR_LOGI(LOG_TAG "SensorApiService destructor has executed\n");
}

/******************************************************************************
  SensorApiService - onListenerReady send HAL READY message to all clients.
******************************************************************************/
void SensorApiService::onListenerReady() {

    // traverse client sockets directory - then broadcast READY message
    SENSOR_LOGD(LOG_TAG ">-- onListenerReady Finding client sockets...\n");

    DIR *dirp = opendir(SOCKET_SENSOR_CLIENT_DIR);
    if (!dirp) {
        return;
    }

    struct dirent *dp = nullptr;
    struct stat sbuf = {0};
    const std::string fnamebase = SOCKET_TO_SENSOR_CLIENT_BASE;
    while (nullptr != (dp = readdir(dirp))) {
        std::string fname = SOCKET_SENSOR_CLIENT_DIR;
        fname += dp->d_name;
        if (-1 == lstat(fname.c_str(), &sbuf)) {
            continue;
        }
        if ('.' == (dp->d_name[0])) {
            continue;
        }
        const char* clientName = NULL;
        if (0 == fname.compare(0, fnamebase.size(), fnamebase)) {
            clientName = fname.c_str();
            SENSOR_LOGV(LOG_TAG "<-- Sending ready to socket: %s\n", clientName);
        }
        if (NULL != clientName) {
            SensorHalDaemonIPCSender* pIpcSender = new SensorHalDaemonIPCSender(clientName);
            SensorAPIHalReadyIndMsg msg(SERVICE_NAME);
            SENSOR_LOGD(LOG_TAG "<-- Sending ready to socket: %s, msg size %d\n", clientName, sizeof(msg));
            pIpcSender->send(reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
            delete pIpcSender;
        }
    }
    closedir(dirp);
}

/******************************************************************************
SensorApiService - onServiceStatusChange override
******************************************************************************/
void SensorApiService::onServiceStatusChange(int serviceId, int instanceId, int status,
		const SensorQsocketSender& sender) {
	SENSOR_LOGI(LOG_TAG ">-- onServiceStatusChange: (%d, %d) status %d\n", serviceId, instanceId, status);
	if (status == 0) {
		SENSOR_LOGI(LOG_TAG ">-- client deleted by qrtr: (%d, %d)\n", serviceId, instanceId);
		deleteEapClientByIds(serviceId, instanceId);
	}
}

/******************************************************************************
SensorApiService - Print sensor List
******************************************************************************/
static void PrintSensorList(struct sensor_list *sensor, int sensor_count)
{
   SENSOR_LOGD(LOG_TAG "sensor_count %d\n", sensor_count);
   for (int i=0 ; i< sensor_count ; i++) {
           SENSOR_LOGD(LOG_TAG"%s\n",sensor[i].name);
           SENSOR_LOGD(LOG_TAG"\tvendor: %s\n",sensor[i].vendor);
           SENSOR_LOGD(LOG_TAG"\tversion: %d\n",sensor[i].version);
           SENSOR_LOGD(LOG_TAG"\tresolution: %f\n",sensor[i].resolution);
           SENSOR_LOGD(LOG_TAG"\tmaxRange %f\n",sensor[i].maxRange);
           SENSOR_LOGD(LOG_TAG"\tsensor_id: %d\n",sensor[i].sensor_id);
           SENSOR_LOGD(LOG_TAG"\ttype: %d\n",sensor[i].type);
           SENSOR_LOGD(LOG_TAG"\trange: %d\n",sensor[i].range);
           SENSOR_LOGD(LOG_TAG"\tmaxSamplingRate: %f\n",sensor[i].maxSamplingRate);
           SENSOR_LOGD(LOG_TAG"\tminBatchCount: %d\n",sensor[i].minBatchCount);
           SENSOR_LOGD(LOG_TAG"\tmaxBatchCount: %d\n",sensor[i].maxBatchCount);
           SENSOR_LOGD(LOG_TAG"\todr rate: %fHZ %fHZ %fHZ %fHZ %fHZ %fHZ\n",
                           sensor[i].odr[0],sensor[i].odr[1],sensor[i].odr[2],
                           sensor[i].odr[3],sensor[i].odr[4],sensor[i].odr[5]);
   }
}

/******************************************************************************
SensorApiService - open_sensor to check sensor supported by device
******************************************************************************/
bool SensorApiService::open_sensor(const configParamToRead & configParamRead)
{

   void *hal;
   int err;
   const struct sensor_t *s;

   //Open sensor Lib
   hal = dlopen(configParamRead.SensorHalLibPath, RTLD_NOW);
   if (!hal) {
	   SENSOR_LOGE(LOG_TAG "ERROR: unable to load HAL %s: %s\n", configParamRead.SensorHalLibPath,
			   dlerror());
	   return false;
   }

   mhmi = (struct sensors_module_t *)dlsym(hal, HAL_MODULE_INFO_SYM_AS_STR);
   if (!mhmi) {
	   SENSOR_LOGE(LOG_TAG "ERROR: unable to find %s entry point in HAL\n",
			   HAL_MODULE_INFO_SYM_AS_STR);
	   return false;
   }

   SENSOR_LOGI(LOG_TAG "HAL loaded: name %s vendor %s version %d.%d id %s\n",
		   mhmi->common.name, mhmi->common.author,
		   mhmi->common.version_major, mhmi->common.version_minor,
		   mhmi->common.id);

   SENSOR_LOGI(LOG_TAG "mSensorType = %d configParamRead.SensorHalLibPath = %s\n", mSensorType, configParamRead.SensorHalLibPath);
   if (strcmp(ASMLIB, configParamRead.SensorHalLibPath) == 0) {
        if (mSensorType != 1) {
                SENSOR_LOGE(LOG_TAG "ERROR: ASM Sensor Type doesn't match with the lib\n");
                return false;
        }
   }

   if (strcmp(IAMLIB, configParamRead.SensorHalLibPath) == 0) {
        if (mSensorType != 2) {
                SENSOR_LOGE(LOG_TAG "ERROR: IAM Sensor Type doesn't match with the lib\n");
                return false;
        }

   }

   if (strcmp(SMI130LIB, configParamRead.SensorHalLibPath) == 0) {
        if (mSensorType != 3) {
                SENSOR_LOGE(LOG_TAG "ERROR: SMI130 Sensor Type doesn't match with the lib\n");
                return false;
        }
   }

   if (strcmp(SMI230LIB, configParamRead.SensorHalLibPath) == 0) {
        if (mSensorType != 4) {
                SENSOR_LOGE(LOG_TAG "ERROR: SMI230 Sensor Type doesn't match with the lib\n");
                return false;
        }
   }

    //set buffer data scaling factor
    set_buff_scaling_factor(mSensorType, mAccBuffRange, mGyroBuffRange);

   err = mhmi->common.methods->open((struct hw_module_t *)mhmi,
		   SENSORS_HARDWARE_POLL, &mdev);
   if (err) {
	   SENSOR_LOGE(LOG_TAG "ERROR: failed to initialize HAL: %d\n", err);
	   return false;
   }

   mpoll_dev = (struct sensors_poll_device_1 *)mdev;
   mpoll_dev_v0 = (struct sensors_poll_device_t *)mdev;


   //Get Sensor List Supported by HAL
   mSensorCount = get_sensor_list(&s);
   if (mSensorCount <= 0 ){
	   mSensorCount = 0;
	   return false;
   }

   if (mSensorType != SENSOR_ASM330  && mSensorType != SENSOR_IAM20680 &&
		   mSensorType != SENSOR_SMI130 &&  mSensorType != SENSOR_SMI230) {
	   SENSOR_LOGE(LOG_TAG "ERROR: Invalid sensor type: %d\n", mSensorType);
	   return false;
   }

   //Copy data to mSensor to track the sensor configuration parameters till last client deregistered.
   mSensor = new (std::nothrow) SensorConfig[mSensorCount];

   if (mSensor == nullptr){
	return false;
   }

   for(int i=0; i < mSensorCount; i++) {
     mSensor[i].sensor_id    = s[i].handle;
     mSensor[i].type         = s[i].type;
     mSensor[i].Activate     = 0;
     mSensor[i].SamplingRate = 0;
     mSensor[i].BatchCount   = 0;
   }

   //Create Sensor List to send to all Clients.
   mSensorList = new (std::nothrow) struct sensor_list[mSensorCount];

   if (mSensorList == nullptr) {
	return false;
   }

   for(int i= 0; i< mSensorCount; i++) {
     if (s[i].type == SENSOR_TYPE_ACCELEROMETER) {
	     strlcpy(&mSensorList[i].name[0], configParamRead.AccelName, MAX_PATH_SIZE);
	     mSensorList[i].type = SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED;
     }
     if (s[i].type == SENSOR_TYPE_GYROSCOPE || s[i].type == SENSOR_TYPE_GYROSCOPE_UNCALIBRATED) {
	     strlcpy(&mSensorList[i].name[0], configParamRead.GyroName, MAX_PATH_SIZE);
	     mSensorList[i].type = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
     }
     strlcpy(&mSensorList[i].vendor[0], s[i].vendor, MAX_PATH_SIZE);
     if(mSensorType == SENSOR_SMI130){
           strlcat(&mSensorList[i].vendor[0], "-SMI130", MAX_PATH_SIZE);
     }
     else if(mSensorType == SENSOR_SMI230){
           strlcat(&mSensorList[i].vendor[0], "-SMI230", MAX_PATH_SIZE);
     }
     else if(mSensorType == SENSOR_ASM330){
        strlcat(&mSensorList[i].vendor[0], "-ASM330", MAX_PATH_SIZE);
     }
     else if(mSensorType == SENSOR_IAM20680){
        strlcat(&mSensorList[i].vendor[0], "-IAM20680", MAX_PATH_SIZE);
     }
     mSensorList[i].version = s[i].version;
     mSensorList[i].resolution = s[i].resolution;
     mSensorList[i].maxRange = s[i].maxRange;
     mSensorList[i].sensor_id = s[i].handle;
     mSensorList[i].maxBatchCount = MAX_BATCH_COUNT;
     GetSupportedSamplingRateAndRange(&mSensorList[i]);
   }
   //Print sensor list for debug
   PrintSensorList(mSensorList, mSensorCount);

   //Initialize Temp Sensor
   if (tempSensorDataInit())
     mTempSupported = true;

   //Check Buffer support
   if (CheckBufferReadFile())
     mBufferSupported = true;

   //Load MLC firmware if available in /lib/firmware folder
   if (LoadMLC(PATH_MLC_BINARY))
     mMlcSupported = true;

   //Create the thread to send data to all clients.
   if (!Sensor_ThreadCreate(&mSensorThreadtid, send_sensor_data_to_clients, this, "SensorPoll-")) {
      SENSOR_LOGE(LOG_TAG "Sensor Poll Data thread failed \n");
      return false;
   }

   //Create the thread to send buffer data to all clients if buffering supported by sensor.
   if(mBufferSupported == true) {
      if (!Sensor_ThreadCreate(&mBufferThreadtid, bufferDataprocessTask, this, "SensorBufferRead-")) {
	     SENSOR_LOGE(LOG_TAG "Sensor Buffer Data read thread failed \n");
	     return false;
     }
   }

   return true;
}

/******************************************************************************
SensorApiService - sensor_activate to actiate/deactivate the sensor
******************************************************************************/
int SensorApiService::sensor_activate(int sensor_id , int enable)
{
   return mpoll_dev->activate(mpoll_dev_v0, sensor_id, enable);
}

/******************************************************************************
SensorApiService - sensor_set_batch to configure the sensor
******************************************************************************/
int SensorApiService::sensor_set_batch(int sensor_id, int64_t delay, int64_t latency)
{
   return mpoll_dev->batch(mpoll_dev, sensor_id, 0, delay, latency);
}

/******************************************************************************
SensorApiService - get_sensor_list to get the sensor list supported by device
******************************************************************************/
int SensorApiService::get_sensor_list(struct sensor_t const **s)
{
  int sensor_num = 0;
  struct sensor_t const* list;
  sensor_num = mhmi->get_sensors_list(mhmi, &list);
  *s= (struct sensor_t const *)list;

  //Print Sensor Info
  SENSOR_LOGD(LOG_TAG "%d sensors found:\n", sensor_num);
  for (int i=0 ; i< sensor_num ; i++) {
        SENSOR_LOGD(LOG_TAG "%s\n"
                "\tvendor: %s\n"
                "\tversion: %d\n"
                "\tsensor_id: %d\n"
                "\ttype: %d\n"
                "\tmaxRange: %f\n"
                "\tresolution: %f\n"
                "\tpower: %f mA\n",
                list[i].name,
                list[i].vendor,
                list[i].version,
                list[i].handle,
                list[i].type,
                list[i].maxRange,
                list[i].resolution,
                list[i].power);
  }

  return sensor_num;
}

/******************************************************************************
SensorApiService - send_sensor_data_to_clients thread to process sensor data
******************************************************************************/
void* SensorApiService::send_sensor_data_to_clients(void *arg) {
  SensorApiService* mSensorService = (SensorApiService*)(arg);
  int count = 0; bool rc = false;
  sensors_event_t events[BUFFER_EVENT];
  while(1)
  {
     count = mSensorService->mpoll_dev->poll(mSensorService->mpoll_dev_v0,
		     events, sizeof(events)/sizeof(sensors_event_t));
     SENSOR_LOGV(LOG_TAG "read events = %d\n",count);
     std::lock_guard<std::mutex> lock(SensorApiService::mMutex);
#ifdef POWERMANAGER_ENABLED
     if ((POWER_STATE_SUSPEND != mSensorService->mPowerState) &&
        (POWER_STATE_SHUTDOWN != mSensorService->mPowerState))
#endif
    {
        for (auto it = mSensorService->mClients.begin(); it != mSensorService->mClients.end();) {
            if (it->second && it->second->mTracking && (it->second->mAccTracking || it->second->mGyroTracking)) {
                rc = it->second->onSensorDataReadCb(events, count);
                // purge this client if failed
                if (!rc) {
                    SENSOR_LOGE(LOG_TAG "failed rc=%d purging client=%s\n", rc, it->first.c_str());
                    it = mSensorService->deleteClientbyName(it->first.c_str());
                }
                else
                    ++it;
            }
            else {
                ++it;
            }
        }
    }
  }
}

/******************************************************************************
SensorApiService - bufferDataprocessTask thread to process buffer data
******************************************************************************/
void* SensorApiService::bufferDataprocessTask(void * arg)
{
  SensorApiService* mSensorService = (SensorApiService*)(arg);
  mSensorService->SensorBuffread();
  return 0;
}


/******************************************************************************
  SensorApiService - processClientMsg recieved from clients
******************************************************************************/
void SensorApiService::processClientMsg(const std::string& data) {

    SensorAPIMsgHeader* pMsg = (SensorAPIMsgHeader*)(data.data());
    uint32_t length = data.length();

    switch (pMsg->msgId) {
        case E_SENSORAPI_CLIENT_REGISTER_MSG_ID: {
            // new client
            if (sizeof(SensorAPIClientRegisterReqMsg) != length) {
                SENSOR_LOGE(LOG_TAG "invalid message\n");
                break;
            }
            newClient(reinterpret_cast<SensorAPIClientRegisterReqMsg*>(pMsg));
            break;
        }
        case E_SENSORAPI_CLIENT_DEREGISTER_MSG_ID: {
            // delete client
            if (sizeof(SensorAPIClientDeregisterReqMsg) != length) {
                SENSOR_LOGE(LOG_TAG "invalid message\n");
                break;
            }
            deleteClient(reinterpret_cast<SensorAPIClientDeregisterReqMsg*>(pMsg));
            break;
        }
        case E_SENSORAPI_GET_SENSOR_LIST_MSG_ID: {
            // List
            if (sizeof(SensorAPIListReqMsg) != length) {
                SENSOR_LOGE(LOG_TAG "invalid message\n");
                break;
            }
            getSensorList(reinterpret_cast<SensorAPIListReqMsg*>(pMsg));
            break;
        }
        case E_SENSORAPI_SENSOR_ENABLE_MSG_ID: {
            // Enable
            if (sizeof(SensorAPIEnableReqMsg) != length) {
                SENSOR_LOGE(LOG_TAG "invalid message\n");
                break;
            }
	    activateSensor(reinterpret_cast<SensorAPIEnableReqMsg*>(pMsg));
            break;
        }
        case E_SENSORAPI_START_BATCHING_MSG_ID: {
            // start batching
            if (sizeof(SensorAPIStartBatchingReqMsg) != length) {
                SENSOR_LOGE(LOG_TAG "invalid message\n");
                break;
            }
            startBatching(reinterpret_cast<SensorAPIStartBatchingReqMsg*>(pMsg));
            break;
        }
        case E_SENSORAPI_START_TRACKING_MSG_ID: {
            // start tracking
            if (sizeof(SensorAPIStartTrackingReqMsg) != length) {
                SENSOR_LOGE(LOG_TAG "invalid message\n");
                break;
            }
            startTracking(reinterpret_cast<SensorAPIStartTrackingReqMsg*>(pMsg));
            break;
        }
        case E_SENSORAPI_SENSOR_MLC_CASE_ENABLE_MSG_ID: {
            // Sensor MLC case enabel/disable request
            if (sizeof(SensorAPIMLCCaseEnableMsg) != length) {
                SENSOR_LOGE(LOG_TAG "invalid message\n");
                break;
            }
	    SensorEnableMLCCase(reinterpret_cast<SensorAPIMLCCaseEnableMsg*>(pMsg));
            break;
        }
        case E_SENSORAPI_SENSOR_TEMP_REQ_MSG_ID: {
            // Sensor Temperature
            if (sizeof(SensorAPITempReqMsg) != length) {
                SENSOR_LOGE(LOG_TAG "invalid message\n");
                break;
            }
            getSensorTemp(reinterpret_cast<SensorAPITempReqMsg*>(pMsg));
            break;
        }
        case E_SENSORAPI_SENSOR_BUFFER_REQ_MSG_ID: {
            // Sensor buffer Data
            if (sizeof(SensorAPIBufferDataReqMsg) != length) {
                SENSOR_LOGE(LOG_TAG "invalid message\n");
                break;
            }
            getSensorBufferData(reinterpret_cast<SensorAPIBufferDataReqMsg*>(pMsg));
            break;
        }
        case E_SENSORAPI_SENSOR_SELFTEST_REQ_MSG_ID: {
            // Sensor buffer Data
            if (sizeof(SensorAPISelfTestReqMsg) != length) {
                SENSOR_LOGE(LOG_TAG "invalid message\n");
                break;
            }
            sensorSelfTest(reinterpret_cast<SensorAPISelfTestReqMsg*>(pMsg));
            break;
        }
        default: {
            SENSOR_LOGV(LOG_TAG "Unknown message with id: %d\n", pMsg->msgId);
            break;
        }
    }
}

/******************************************************************************
SensorApiService - implementation - registration
******************************************************************************/
void SensorApiService::newClient(SensorAPIClientRegisterReqMsg *pMsg) {

    std::lock_guard<std::mutex> lock(mMutex);
    std::string clientname(pMsg->mSocketName);
    SENSOR_LOGI(LOG_TAG ">-- newClient %s\n", clientname.c_str());

    // if this name is already used return error
    if (mClients.find(clientname) != mClients.end()) {
        SENSOR_LOGE(LOG_TAG "invalid client=%s already existing\n", clientname.c_str());
        return;
    }

    // store it in client property database
    SensorHalDaemonClientHandler *pClient =
            new SensorHalDaemonClientHandler(this, clientname, pMsg->mClientType, mSensorCount);
    if (!pClient) {
        SENSOR_LOGE(LOG_TAG "failed to register client=%s\n", clientname.c_str());
        return;
    }
    //Send Sensor List to client
    if(mSensorCount > 0)
	    pClient->onSensorListCb(mSensorList, mSensorCount);
    if (mSensorMlcCaseCount > 0)
	    pClient->onSensorMlcCaseListCb(mSesnorMlcCaseList, mSensorMlcCaseCount);

    pClient->onCapabilitiesCallback(SHD_READY);

    mSensorClient++;
    mClients.emplace(clientname, pClient);
    SENSOR_LOGI(LOG_TAG ">-- registered new client=%s\n", clientname.c_str());
}

/******************************************************************************
SensorApiService - implementation - deregistration
******************************************************************************/
void SensorApiService::deleteClient(SensorAPIClientDeregisterReqMsg *pMsg) {
    std::lock_guard<std::mutex> lock(mMutex);
    SENSOR_LOGI(LOG_TAG ">-- deleteClient\n");
    std::string clientname(pMsg->mSocketName);

    deleteClientbyName(clientname);
}

std::unordered_map<std::string, SensorHalDaemonClientHandler*>::iterator SensorApiService::deleteClientbyName(const std::string clientname) {
    // We shall not hold the lock, as lock already held by the caller
    //
    mSensorClient--;
    bool activate = false;
    SENSOR_LOGI(LOG_TAG ">-- deleteClientbyName %s mSensorClient %d\n",clientname.c_str(), mSensorClient);

    // remove the client from the config request map
    for (auto it = mConfigReqs.begin(); it != mConfigReqs.end();) {
     if (strncmp(it->second.clientName.c_str(), clientname.c_str(),
			     strlen (clientname.c_str())) == 0) {
	     it = mConfigReqs.erase(it);
     } else {
	     ++it;
     }
    }
    // delete this client from property db
    SensorHalDaemonClientHandler* pClient = getClient(clientname);

    if (!pClient) {
        SENSOR_LOGE(LOG_TAG ">-- deleteClient invlalid client=%s\n", clientname.c_str());
        return;
    }

    std::unordered_map<std::string, SensorHalDaemonClientHandler*>::iterator itr = mClients.find(clientname);
    itr = mClients.erase(itr);
    pClient->cleanup();

    activate = false;
    //deactivate sensor if no client activated sensors
    for (auto it = mClients.begin(); it != mClients.end(); ++it){
         if (it->second && (it->second->mAccTracking || it->second->mGyroTracking)) {
                             activate = true;
         }
    }

    if (activate != true){
       for(int i=0; i < mSensorCount; i++) {
         sensor_activate(mSensor[i].sensor_id, SENSOR_DISABLE);
         mSensor[i].Activate = 0;
         mSensor[i].SamplingRate = 0;
         mSensor[i].BatchCount = 0;
       }
    }

    SENSOR_LOGI(LOG_TAG ">-- deleteClient client=%s\n", clientname.c_str());
    return itr;
}


void SensorApiService::deleteEapClientByIds(int serviceId, int instanceId) {

    std::lock_guard<std::mutex> lock(mMutex);

    const char* clientName = getClientNameByIds(serviceId, instanceId);
    if (clientName) {
        SENSOR_LOGI(LOG_TAG ">-- service id: %d, instance id: %d, client name: %s",
                 serviceId, instanceId, clientName);
        deleteClientbyName(std::string(clientName));
    }
}

/******************************************************************************
SensorApiService - implementation - StartTracking
******************************************************************************/
void SensorApiService::startTracking(SensorAPIStartTrackingReqMsg *pMsg) {
    std::lock_guard<std::mutex> lock(mMutex);
    SENSOR_LOGI(LOG_TAG ">-- startTracking\n");

    int ret = 0;
    SensorHalDaemonClientHandler* pClient = getClient(pMsg->mSocketName);
    if (!pClient) {
        SENSOR_LOGE(LOG_TAG ">-- start invlalid client=%s\n", pMsg->mSocketName);
        return;
    }

    pClient->mTracking = true;
    pClient->mVariableCountBatching = mVariableCountBatching;
    pClient->mPendingMessages.push(E_SENSORAPI_START_TRACKING_MSG_ID);
    pClient->onResponseCb(ret, E_SENSORAPI_START_TRACKING_MSG_ID);

    SENSOR_LOGI(LOG_TAG ">-- star session AccTracking %d GyroTracking %d\n",
		    pClient->mAccTracking, pClient->mGyroTracking);

    return;
}

/******************************************************************************
SensorApiService - implementation - StartBatching
******************************************************************************/
int SensorApiService::SensorCofig(SensorAPIStartBatchingReqMsg *pMsg) {
    int ret = 0;
    int64_t SamplingRate = 0;
    int64_t BatchingRate = 0;

    SensorHalDaemonClientHandler* pClient = getClient(pMsg->mSocketName);

    if (pClient == nullptr) {
	return SENSOR_ERROR_CONFIG_FAILED;
    }

    for (int i=0 ; i < mSensorCount; i++)  {
         //Check for proper sensor_id
         if (pMsg->sensor_id == mSensor[i].sensor_id) {
	   //Check for Accel Parameteters
           if (mSensor[i].type == SENSOR_TYPE_ACCELEROMETER) {
	     //Return error if requested sampling rate is more than config file parameters.
             if (pMsg->samplingRate > mMaxAccSampleRate) {
                     ret = SENSOR_ERROR_INVALID_INPUT_PARAMETER;
                     break;
             }
	     //Configure sensor to parameters defined in /etc/sensors.conf file , config only once
             if (mSensor[i].SamplingRate != mMaxAccSampleRate) {
                     mSensor[i].SamplingRate = mMaxAccSampleRate;
                     mSensor[i].BatchCount = mMinAccBatchCount;
                     SamplingRate = FREQUENCY_TO_NS(mMaxAccSampleRate);
                     BatchingRate = mMinAccBatchCount * SamplingRate  * mBatchConst;

                     SENSOR_LOGI(LOG_TAG ">-- Configure sensor Acc sensor_id %d sampling Rate %lld BatchingRate %lld\n",
                                     pMsg->sensor_id, SamplingRate, BatchingRate);
                     ret = sensor_set_batch(pMsg->sensor_id, SamplingRate, BatchingRate);
                     if (ret != 0) {
                          ret = SENSOR_ERROR_CONFIG_FAILED;
                          break;
		     }
             }
	     //Calcualate the client config parameters based on sensor physical configuration
	     pClient->mAccFactor = lroundf(mSensor[i].SamplingRate / pMsg->samplingRate);
	     pMsg->batchCount = NearByBatchCount(mMinAccBatchCount, pMsg->batchCount, mSensor[i].SamplingRate, pMsg->samplingRate, pClient->mAccFactor);
	     pClient->mAccBatchCount = pMsg->batchCount;
	     pClient->mAccCount = 0;
	     pClient->mAccMovingCount = 0;
	     pClient->mAccTracking = false;
         pClient->setFIRFilter(mSensor[i].SamplingRate, mSensor[i].SamplingRate / pClient->mAccFactor, true);
	     //Allocate memory to store acc events based on requested batch count by client
	     if (pClient->mAccEvents) {
		     delete pClient->mAccEvents;
		     pClient->mAccEvents = nullptr;
             }
	     pClient->mAccEvents = new (std::nothrow) sensors_event_t [pClient->mAccBatchCount];

	     if (pClient->mAccEvents == nullptr){
		return SENSOR_ERROR_CONFIG_FAILED;
	     }

	     memset(pClient->mAccEvents, 0, sizeof(sensors_event_t) * pClient->mAccBatchCount);
           }

	   ////Check for Gyro Parameteters////
           if (mSensor[i].type == SENSOR_TYPE_GYROSCOPE || mSensor[i].type == SENSOR_TYPE_GYROSCOPE_UNCALIBRATED) {
	     //Return error if requested sampling rate is more than config file parameters.
             if (pMsg->samplingRate > mMaxGyroSampleRate) {
                     ret = SENSOR_ERROR_INVALID_INPUT_PARAMETER;
		     break;
             }
	     //Configure sensor to parameters defined in /etc/sensors.conf file , config only once
             if (mSensor[i].SamplingRate != mMaxGyroSampleRate) {
                     mSensor[i].SamplingRate = mMaxGyroSampleRate;
                     mSensor[i].BatchCount = mMinGyroBatchCount;
                     SamplingRate = FREQUENCY_TO_NS(mMaxGyroSampleRate);
                     BatchingRate = mMinGyroBatchCount * SamplingRate  * mBatchConst;
                     SENSOR_LOGI(LOG_TAG ">--Configure sensor Gyro sensor_id %d sampling Rate %lld BatchingRate %lld\n",
                                     pMsg->sensor_id, SamplingRate, BatchingRate);
                     ret = sensor_set_batch(pMsg->sensor_id, SamplingRate, BatchingRate);
                     if (ret != 0) {
                          ret = SENSOR_ERROR_CONFIG_FAILED;
                          break;
		     }
	     }
	     //Calcualate the client config parameters based on sensor physical configuration
	     pClient->mGyroFactor = lroundf(mSensor[i].SamplingRate / pMsg->samplingRate);
	     pMsg->batchCount = NearByBatchCount(mMinGyroBatchCount, pMsg->batchCount, mSensor[i].SamplingRate, pMsg->samplingRate, pClient->mGyroFactor);
	     pClient->mGyroBatchCount = pMsg->batchCount;
	     pClient->mGyroCount = 0;
	     pClient->mGyroMovingCount = 0;
	     pClient->mGyroTracking = false;
         pClient->setFIRFilter(mSensor[i].SamplingRate, mSensor[i].SamplingRate / pClient->mGyroFactor, false);

	     //Allocate memory to store Gyro events based on requested batch count by client
	     if (pClient->mGyroEvents) {
		     delete pClient->mGyroEvents;
		     pClient->mGyroEvents = nullptr;
	     }
	     pClient->mGyroEvents = new (std::nothrow) sensors_event_t [pClient->mGyroBatchCount];

	     if (pClient->mGyroEvents == nullptr) {
		return SENSOR_ERROR_CONFIG_FAILED;
	     }

	     memset(pClient->mGyroEvents, 0, sizeof(sensors_event_t) * pClient->mGyroBatchCount);
	   }
	 }
    }

    SENSOR_LOGI(LOG_TAG ">-- start batching session AccFactor %d AccBatchcount %d GyroFactor %d GyroBatchCount %d \
		    AccTracking %d GyroTracking %d\n", pClient->mAccFactor, pClient->mAccBatchCount, pClient->mGyroFactor,
		    pClient->mGyroBatchCount, pClient->mAccTracking, pClient->mGyroTracking);

    return ret;
}

void SensorApiService::startBatching(SensorAPIStartBatchingReqMsg *pMsg) {
    std::lock_guard<std::mutex> lock(mMutex);
    int ret = 0;
    bool SensorId = false;

    SensorHalDaemonClientHandler* pClient = getClient(pMsg->mSocketName);
    if (!pClient) {
	    SENSOR_LOGE(LOG_TAG ">-- start invalid client=%s\n", pMsg->mSocketName);
	    return;
    }

    SENSOR_LOGI(LOG_TAG ">-- SensorId %d SampleRate %f BatchCount %d\n",
		    pMsg->sensor_id, pMsg->samplingRate, pMsg->batchCount);

    //Input parameter check
    if (mSensorCount != 0) {
      for (int i=0; i < mSensorCount; i++) {
	      if (mSensorList[i].sensor_id == pMsg->sensor_id) {
		      SensorId = true;
                     if (pMsg->batchCount <=0 || pMsg->samplingRate <=0 ) {
			      ret = SENSOR_ERROR_INVALID_INPUT_PARAMETER;
			      goto fail;
		      }
		      else {
			      pMsg->samplingRate = NearBySamplingRate(mSensorList[i].odr, pMsg->samplingRate);
                      }
		      break;
	      }
      }
      if (SensorId != true ) {
	      ret = SENSOR_ERROR_INVALID_INPUT_PARAMETER;
	      goto fail;
      }
    }
    else {
	    ret = SENSOR_ERROR_NO_SENSORS_FOUND;
	    goto fail;
    }

    //Configure the sensor to parameters defined in /etc/sensors.conf file.
    ret = SensorCofig(pMsg);

fail:
    pClient->mPendingMessages.push(E_SENSORAPI_START_BATCHING_RES_ID);
    pClient->onResponseCb(ret, E_SENSORAPI_START_BATCHING_RES_ID);
    if(ret == 0)
	    pClient->onSensorBatchingCb(pMsg->sensor_id, pMsg->samplingRate,  pMsg->batchCount);

    return;
}

/******************************************************************************
SensorApiService - implementation - Activate/Deactivate Sensor
******************************************************************************/
void SensorApiService::activateSensor(SensorAPIEnableReqMsg* pMsg) {
   std::lock_guard<std::mutex> lock(mMutex);
   int ret = 0;
   int enable = 0;
   bool SensorId = false;

   SensorHalDaemonClientHandler* pClient = getClient(pMsg->mSocketName);
   if (!pClient) {
	   SENSOR_LOGI(LOG_TAG ">-- start invlalid client=%s\n", pMsg->mSocketName);
	   return;
   }
   SENSOR_LOGI(LOG_TAG "<-- sensor sensor_id %d sensor enable %d \n", pMsg->sensor_id, pMsg->enable);

   //Input parameter check
   if (pMsg->enable < SENSOR_DISABLE  || pMsg->enable > SENSOR_HPM) {
	   ret=SENSOR_ERROR_INVALID_INPUT_PARAMETER;
	   goto fail;
   }

   if (mSensorCount != 0) {
	   for (int i=0; i < mSensorCount; i++) {
		   if (mSensorList[i].sensor_id == pMsg->sensor_id) {
			   SensorId = true;
			   break;
		   }
	    }
	    if (SensorId != true ) {
		    ret = SENSOR_ERROR_INVALID_INPUT_PARAMETER;
		    goto fail;
	    }
   }
   else {
	   ret = SENSOR_ERROR_NO_SENSORS_FOUND;
	   goto fail;
   }


   //Set Sensor LPM or HPM mode based on mode
   if ( pMsg->enable == SENSOR_LPM || pMsg->enable == SENSOR_HPM) {
	   ret = SetPowerMode(pMsg->sensor_id, pMsg->enable);
	   goto fail;
   }

   //Activating/Deactivating the sensor
   for (int i=0 ; i < mSensorCount; i++) {
     if (pMsg->sensor_id == mSensor[i].sensor_id) {
	pClient->mActivate[i] = pMsg->enable;
	if (mSensor[i].type == SENSOR_TYPE_ACCELEROMETER) {
		pClient->mAccTracking = pMsg->enable;
		pClient->mAccCount = 0;
		pClient->mAccMovingCount = 0;
	}
	if (mSensor[i].type == SENSOR_TYPE_GYROSCOPE || mSensor[i].type == SENSOR_TYPE_GYROSCOPE_UNCALIBRATED) {
		pClient->mGyroTracking = pMsg->enable;
		pClient->mGyroCount = 0;
		pClient->mGyroMovingCount = 0;
	}
        //Check the enable request of all clients
	for (auto each : mClients) {
	   enable = max(enable, each.second->mActivate[i]);
	   SENSOR_LOGD(LOG_TAG "<-- enable %d each.second->mActivate[%d] %d \n",enable,i,each.second->mActivate[i]);
	}
        // activate or deactivate the sensor
	if(enable != mSensor[i].Activate) {
           SENSOR_LOGI(LOG_TAG "<-- calling sensor_activate sensor_id %d  enable %d \n",pMsg->sensor_id, enable);
	   ret = sensor_activate(pMsg->sensor_id , enable);
	   mSensor[i].Activate = enable;
	   //reset Sensor config parameters to zero once it deactivated.
	   if(enable == 0 && ret == 0) {
		   mSensor[i].SamplingRate = 0;
		   mSensor[i].BatchCount = 0;
	   }
	}
     }
   }

   SENSOR_LOGI(LOG_TAG ">-- sensor activation AccTracking %d GyroTracking %d\n",
		   pClient->mAccTracking, pClient->mGyroTracking);
fail:
   pClient->mPendingMessages.push(E_SENSORAPI_SENSOR_ENABLE_MSG_ID);
   pClient->onResponseCb(ret, E_SENSORAPI_SENSOR_ENABLE_MSG_ID);
   return;
}

/******************************************************************************
SensorApiService - implementation - getSensorList - to send the sensor list
******************************************************************************/
void SensorApiService::getSensorList(SensorAPIListReqMsg* pMsg) {
    std::lock_guard<std::mutex> lock(mMutex);

    SENSOR_LOGI(LOG_TAG "--<getSensorList\n");

    SensorHalDaemonClientHandler* pClient = getClient(pMsg->mSocketName);
    if (!pClient) {
	    SENSOR_LOGE(LOG_TAG ">-- get sensor list invlalid client=%s\n", pMsg->mSocketName);
	    return;
    }

    //Send Sensor List to client
    pClient->onSensorListCb(mSensorList, mSensorCount);
}

/******************************************************************************
SensorApiService - implementation - SensorMLCCaseEnable to enable/disable mlc cases
******************************************************************************/
void SensorApiService::SensorEnableMLCCase(SensorAPIMLCCaseEnableMsg* pMsg) {
    std::lock_guard<std::mutex> lock(mMutex);
    int ret = SENSOR_ERROR_MLC_EVENT_ENABLE_FAILED;

    SENSOR_LOGI(LOG_TAG "--<SensorEnableMLCCase name %s enable %d \n",
		    pMsg->mlc_case_name, pMsg->enable);

    SensorHalDaemonClientHandler* pClient = getClient(pMsg->mSocketName);
    if (!pClient) {
            SENSOR_LOGE(LOG_TAG ">-- SensorEnableMLCCase invlalid client=%s\n", pMsg->mSocketName);
            return;
    }

    //Input parameter check
    if (pMsg->enable != 1 && pMsg->enable != 0) {
            ret = SENSOR_ERROR_INVALID_INPUT_PARAMETER;
            goto fail;
    }

    pClient->mMlcEnable = false;
    if (mMlcSupported == true) {
	      for (int i = 0; i < mSensorMlcCaseCount ; i++) {
		 if (strcmp(pClient->mMlcCaseList[i].name, pMsg->mlc_case_name) == 0) {
			 pClient->mMlcCaseList[i].enable = pMsg->enable;
			 SENSOR_LOGI(LOG_TAG "pClient->mMlcCaseList[i].name %s enable %d\n",
					 pClient->mMlcCaseList[i].name,pClient->mMlcCaseList[i].enable);
		 }
	      }
	      if(SensorMlcEnableEvents(pMsg->mlc_case_name, pMsg->enable))
		      ret = SENSOR_RESPONSE_SUCCESS;
    }

    //Check MLC enable status for client
    for (int i = 0; i < mSensorMlcCaseCount ; i++)
	    if ( pClient->mMlcCaseList[i].enable == true)
		    pClient->mMlcEnable = true;
fail:
    pClient->mPendingMessages.push(E_SENSORAPI_SENSOR_MLC_CASE_ENABLE_MSG_ID);
    pClient->onResponseCb(ret, E_SENSORAPI_SENSOR_MLC_CASE_ENABLE_MSG_ID);

    return;
}

/******************************************************************************
SensorApiService - implementation - getSensorTemp to send temperature
******************************************************************************/
void SensorApiService::getSensorTemp(SensorAPITempReqMsg* pMsg) {
    float temperature = 0;

    SENSOR_LOGD(LOG_TAG "--<getSensorTemp\n");

    SensorHalDaemonClientHandler* pClient = getClient(pMsg->mSocketName);
    if (!pClient) {
            SENSOR_LOGE(LOG_TAG ">-- get sensor temperatue invlalid client=%s\n", pMsg->mSocketName);
            return;
    }
    if (mTempSupported)
	    tempSensorDataPollTask(&temperature);
    pClient->onSensorTempCb(temperature);
}

/******************************************************************************
SensorApiService - implementation - getSensorBufferData to send buffer data
******************************************************************************/
void SensorApiService::getSensorBufferData(SensorAPIBufferDataReqMsg* pMsg) {
    std::lock_guard<std::mutex> lock(mMutex);
    int ret = SENSOR_ERROR_BUFFER_NOT_SUPPORTED;
    bool rc;
    SENSOR_LOGI(LOG_TAG "--<getSensorBufferData pMsg->enable %d\n",pMsg->enable);

    SensorHalDaemonClientHandler* pClient = getClient(pMsg->mSocketName);
    if (!pClient) {
            SENSOR_LOGE(LOG_TAG ">-- get sensor buffer data invlalid client=%s\n", pMsg->mSocketName);
            return;
    }

    //Input parameter check
    if (pMsg->enable != 1 && pMsg->enable != 0) {
	    ret = SENSOR_ERROR_INVALID_INPUT_PARAMETER;
	    goto fail;
    }

    if (mBufferSupported == true) {
	    pClient->mBufferRead = pMsg->enable;
	    if (mBufferDeleted == true && pClient->mBufferRead == true) {
		    ret = SENSOR_ERROR_BUFFER_DELETED;
	    }
	    else {
                    rc = WritetoBufferFile(pMsg->enable);
                    if(rc){
			ret = SENSOR_RESPONSE_SUCCESS;
                    }
		    pClient->mPendingMessages.push(E_SENSORAPI_SENSOR_BUFFER_REQ_MSG_ID);
		    pClient->onResponseCb(ret, E_SENSORAPI_SENSOR_BUFFER_REQ_MSG_ID);
		    pthread_mutex_lock (&mHalBuffMutex);
		    pthread_cond_signal (&mHalBuffCond);
		    pthread_mutex_unlock (&mHalBuffMutex);
		    return;
	    }
    }
fail:
    pClient->mPendingMessages.push(E_SENSORAPI_SENSOR_BUFFER_REQ_MSG_ID);
    pClient->onResponseCb(ret, E_SENSORAPI_SENSOR_BUFFER_REQ_MSG_ID);

    return;
}

/******************************************************************************
SensorApiService - implementation - sensorSelfTest to do the self test of sensor
*****************************************************************************/
void SensorApiService::onSelfTestRequest(SensorHalDaemonClientHandler* pClient,
		int sensor_id, SelfTestType selfTestType, int request_id) {

    std::lock_guard<std::mutex> lock(mMutex);
    SENSOR_LOGI(LOG_TAG "--< onSelfTestRequest sensor_id %d SelfTestType %d request_id %d\n",
			sensor_id, selfTestType, request_id);
    int ret = 0;
    FILE *self_test_fd = NULL;
    char *file_path_name = NULL;
    int fsize = 256;
    char self_test_file_name[DEVICE_IIO_MAX_FILENAME_LEN] = {'\0'};
    int len = 0;
    char buffer_string[DEVICE_IIO_MAX_FILENAME_LEN];
    SelfTestResult result;

    for (int i=0 ; i < mSensorCount; i++) {
	    if (sensor_id == mSensor[i].sensor_id) {
		    if (mSensor[i].type == SENSOR_TYPE_ACCELEROMETER) {
			    find_path(DYN_IIO_TYPE, self_test_file_name, ASM330LHHX_ACC_SEARCH,
					    sizeof(self_test_file_name));
		    }
		    if (mSensor[i].type == SENSOR_TYPE_GYROSCOPE) {
			    find_path(DYN_IIO_TYPE, self_test_file_name, ASM330LHHX_GYRO_SEARCH,
					    sizeof(self_test_file_name));
		    }
	    }
    }

    strlcat(self_test_file_name, "selftest", sizeof(self_test_file_name));
    SENSOR_LOGI(LOG_TAG "self test file name %s\n", self_test_file_name);

    for(int i = 0 ; i < mSensorCount; i++)  {
	    sensor_activate(mSensor[i].sensor_id, SENSOR_DISABLE); //Enable the sensor
    }
    self_test_fd = fopen(self_test_file_name, "w+");

    /*If the file discriptor to open self-test is null,
      the execution will be returned back. */
    if (self_test_fd == nullptr) {
	    SENSOR_LOGE(LOG_TAG "NULL");
	    result = Failed;
            goto fail;
    }

    if (selfTestType == Positive) {
	    SENSOR_LOGI(LOG_TAG "wrting Positive sign to self test file\n");
	    ret = fprintf(self_test_fd, "positive-sign");
    }
    else if (selfTestType == Negative) {
	    SENSOR_LOGI(LOG_TAG "wrting Negative sign to self test file\n");
	    ret = fprintf(self_test_fd, "negative-sign");
    }

    rewind(self_test_fd);
    fgets(buffer_string, 50, self_test_fd);
    SENSOR_LOGI(LOG_TAG "buffer_string = %s ", buffer_string);

    if(strstr(buffer_string, "pass")){
	    SENSOR_LOGI(LOG_TAG "self test is passed\n");
	    result = Passed;
    }
    else{
	    SENSOR_LOGI(LOG_TAG "self test failed but made it pass\n");
	    result = Failed;
    }
    fclose(self_test_fd);
    for(int i = 0 ; i < mSensorCount; i++)  {
	    if (mSensor[i].Activate == SENSOR_ENABLE){
		    int64_t SamplingRate = FREQUENCY_TO_NS(mSensor[i].SamplingRate);
		    int64_t BatchingRate =  mSensor[i].BatchCount *  mSensor[i].SamplingRate  * mBatchConst;
		    SENSOR_LOGI(LOG_TAG ">-- onSelfTest Re-Configure sensor sensor_id %d sampling Rate %lld BatchingRate %lld\n",
				    mSensor[i].sensor_id, SamplingRate, BatchingRate);
		    sensor_set_batch(mSensor[i].sensor_id, SamplingRate, BatchingRate); //configure the sensor
		    sensor_activate(mSensor[i].sensor_id, SENSOR_ENABLE); //Enable the sensor
	    }
    }
    SENSOR_LOGI(LOG_TAG "sensor_id = %d request_id = %d result = %d\n", sensor_id, request_id, result);

fail:
    pClient->onSensorSelfTestResultCb(sensor_id, request_id, result);

    return;
}

void SensorApiService::sensorSelfTest(SensorAPISelfTestReqMsg* pMsg) {
    std::lock_guard<std::mutex> lock(mMutex);
    int ret = 0;
    bool SensorId = false;
    int value = 0;
    int sensor_id =  pMsg->sensor_id;
    int request_id = pMsg->request_id;
    char self_test_file_name[DEVICE_IIO_MAX_FILENAME_LEN] = {'\0'};
    SelfTestType  selfTestType = pMsg->selfTestType;

    SENSOR_LOGI(LOG_TAG "--<sensorSelfTest sensor_id %d SelfTestType %d request_id %d\n",
		    pMsg->sensor_id, pMsg->selfTestType, pMsg->request_id);

    SensorHalDaemonClientHandler* pClient = getClient(pMsg->mSocketName);
    if (!pClient) {
            SENSOR_LOGE(LOG_TAG ">-- sensorSelfTest invlalid client=%s\n", pMsg->mSocketName);
            return;
    }

   //Input parameter check
    if (mSensorCount != 0) {
      for (int i=0; i < mSensorCount; i++) {
              if (mSensorList[i].sensor_id == pMsg->sensor_id) {
		      SensorId = true;
		      if (pMsg->selfTestType != Positive && pMsg->selfTestType != Negative) {
			      ret = SENSOR_ERROR_INVALID_INPUT_PARAMETER;
			      break;
		      }
	      }
      }
      if (SensorId != true ) {
	      ret = SENSOR_ERROR_INVALID_INPUT_PARAMETER;
      }
    }
    else {
	    ret = SENSOR_ERROR_NO_SENSORS_FOUND;
    }

    if(mSensorType == 1) {
        for (int i=0 ; i < mSensorCount; i++) {
            if (sensor_id == mSensor[i].sensor_id) {
                    if (mSensor[i].type == SENSOR_TYPE_ACCELEROMETER) {
                            find_path(DYN_IIO_TYPE, self_test_file_name, ASM330LHHX_ACC_SEARCH,
                                            sizeof(self_test_file_name));
                    }
                    if (mSensor[i].type == SENSOR_TYPE_GYROSCOPE) {
                            find_path(DYN_IIO_TYPE, self_test_file_name, ASM330LHHX_GYRO_SEARCH,
                                            sizeof(self_test_file_name));
                    }
            }
        }

        strlcat(self_test_file_name, "selftest", sizeof(self_test_file_name));
        value = strncmp(self_test_file_name, "selftest", sizeof(self_test_file_name));
        if(value == 0){
                ret = SENSOR_SELFTEST_NOT_SUPPORTED;
                pClient->mPendingMessages.push(E_SENSORAPI_SENSOR_SELFTEST_REQ_MSG_ID);
                pClient->onResponseCb(ret, E_SENSORAPI_SENSOR_SELFTEST_REQ_MSG_ID);
        }
        else {
                if(ret == 0){
                        pClient->mPendingMessages.push(E_SENSORAPI_SENSOR_SELFTEST_REQ_MSG_ID);
                        pClient->onResponseCb(ret, E_SENSORAPI_SENSOR_SELFTEST_REQ_MSG_ID);
                        std::thread appCallbackThread([this, pClient, sensor_id, selfTestType, request_id] {
                                    onSelfTestRequest(pClient, sensor_id, selfTestType, request_id);
                        });
                        appCallbackThread.detach();
                }
        }
    }
    else {
        ret = SENSOR_SELFTEST_NOT_SUPPORTED;
        pClient->mPendingMessages.push(E_SENSORAPI_SENSOR_SELFTEST_REQ_MSG_ID);
        pClient->onResponseCb(ret, E_SENSORAPI_SENSOR_SELFTEST_REQ_MSG_ID);
    }

    return;
}

/******************************************************************************
SensorApiService - implementation - NearByBatchCount to check nearby batch
count, return batch count which is multiplication of min batch count defined for
each sensor in /etc/sensors.conf file and should be less than or equal to
ReqBatchCount
******************************************************************************/
int SensorApiService::NearByBatchCount(int minBatchCount, int ReqBatchCount, float input_rate, float output_rate, int factor) {
   int count = 0;
   int supported_batches[MAX_BATCH_COUNT];

   if ( minBatchCount <= 0 || ReqBatchCount <= 0)
	   return minBatchCount;
   //No FIFO support
   if (mBatchConst == 0)
	   return ReqBatchCount;

   for (int i = 1; i <= MAX_BATCH_COUNT; i++) {
	   if ((i * factor) % minBatchCount == 0) {
		   supported_batches[count] = i;
		   count++;
	   }
   }

   SENSOR_LOGI(LOG_TAG "Supported batch rates for input rate %.2fHz, output rate %.2fHz, and batch count %d:\n", input_rate, output_rate, minBatchCount);
   for (int i = 0; i < count; i++) {
	   SENSOR_LOGI(LOG_TAG "%d ", supported_batches[i]);
   }
   SENSOR_LOGI(LOG_TAG "\n");

   int nearest = supported_batches[0];
   int min_diff = abs(supported_batches[0] - ReqBatchCount);

   for (int i = 0; i < count; i++) {
	   int diff = abs(supported_batches[i] - ReqBatchCount);
	   if (diff < 0)
		   diff = -diff;
	   if (diff < min_diff) {
		   nearest = supported_batches[i];
		   min_diff = diff;
	   }
	   // If an exact match is found, return it immediately
	   if (diff == 0) {
		   SENSOR_LOGI(LOG_TAG "Nearest supported batch rate to %d: %d\n", ReqBatchCount, supported_batches[i]);
		   return supported_batches[i];
	   }
   }
   SENSOR_LOGI(LOG_TAG "Nearest supported batch rate to %d: %d\n", ReqBatchCount, nearest);
   return nearest;
}

/******************************************************************************
SensorApiService - implementation - NearBySamplingRate
Find out near by Nearby sampling rate which client requested
******************************************************************************/
float SensorApiService::NearBySamplingRate(float input_rates[], float target_rate) {
   float nearest_rate = input_rates[0];
   float min_diff = fabs(input_rates[0] - target_rate);

   for (int i = 0; i < MAX_ODR; i++) {
	   if (input_rates[i] == 0) {
		   continue; // Skip zero values
	   }
	   float diff = fabs(input_rates[i] - target_rate);
	   if (diff < 0)
		   diff = -diff;
	   if (diff < min_diff) {
		   min_diff = diff;
		   nearest_rate = input_rates[i];
	   }
	   //If an exact match is found, return it immediately
	   if (diff == 0) {
		   return input_rates[i];
	   }
   }

   return nearest_rate;
}

/******************************************************************************
SensorApiService - implementation - GetSupportedSamplingRate
Sampling rate supported by each sensor
******************************************************************************/
void SensorApiService::GetSupportedSamplingRateAndRange(struct sensor_list *s) {
  switch(mSensorType) {
      //Check for ASM330 sensor
      case SENSOR_ASM330: {
	if (s->type == SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED) {
		float samplingRate[6] = {13, 26, 52, 104, 208, 416};
		float acc_range[4][2] = { {0.000598,2}, { 0.001196,4}, {0.002392,8}, {0.004785,16}};
		float scale_value = 0;
		int acc_num = -1;
		char tmp_filaname[DEVICE_IIO_MAX_FILENAME_LEN] = {'\0'};
		s->range = -1;
		memcpy(&s->odr[0], samplingRate, sizeof(samplingRate));
		acc_num = get_sensor_device_by_name("asm330lhhx_accel");
		if (acc_num < 0) {
			acc_num = get_sensor_device_by_name("asm330lhh_accel");
			if (acc_num < 0)
				SENSOR_LOGE(LOG_TAG "No asm330 accel sensor found into /sys/bus/iio/devices/ folder.\n");
		}
		/* save path to acc. iio device in sysfs */
		snprintf(tmp_filaname, DEVICE_IIO_MAX_FILENAME_LEN,
				"/sys/bus/iio/devices/iio:device%d/in_accel_x_scale",
				acc_num);
		SENSOR_LOGI(LOG_TAG "Acc tmp_filaname %s acc_num %d\n", tmp_filaname, acc_num);
		if(!sysfs_read_scale(tmp_filaname, &scale_value)) {
			SENSOR_LOGI(LOG_TAG "Acc scale_value %f\n", scale_value);
			for(int i = 0; i < 4; i++)
				if(acc_range[i][0] == scale_value)
					s->range = acc_range[i][1];
		}
		mMaxAccSampleRate  = NearBySamplingRate(samplingRate, mMaxAccSampleRate);
		s->maxSamplingRate = mMaxAccSampleRate;
		if (mMinAccBatchCount >= MAX_BATCH_COUNT)
			mMinAccBatchCount = MAX_BATCH_COUNT;
		else if (mMinAccBatchCount <= 0)
			mMinAccBatchCount = 1;
		s->minBatchCount   = mMinAccBatchCount;
        }
	if (s->type == SENSOR_TYPE_GYROSCOPE_UNCALIBRATED){
		float samplingRate[6] = {13, 26, 52, 104, 208, 416};
		float gyro_range[6][2] = {{0.000076,125}, {0.000153,250}, {0.000305,500}, {0.000611,1000}, {0.001222,2000}, {0.002443,4000}};
		float scale_value = 0;
		int gyro_num = -1;
		char tmp_filaname[DEVICE_IIO_MAX_FILENAME_LEN] = {'\0'};
		s->range = -1;
		memcpy(&s->odr[0], samplingRate, sizeof(samplingRate));
                gyro_num = get_sensor_device_by_name("asm330lhhx_gyro");
                if (gyro_num < 0) {
			gyro_num = get_sensor_device_by_name("asm330lhh_gyro");
			if (gyro_num < 0)
				SENSOR_LOGE(LOG_TAG "No asm330 gyro sensor found into /sys/bus/iio/devices/ folder.\n");
		}
		/* save path to acc. iio device in sysfs */
		snprintf(tmp_filaname, DEVICE_IIO_MAX_FILENAME_LEN,
				"/sys/bus/iio/devices/iio:device%d/in_anglvel_x_scale",
				gyro_num);
		SENSOR_LOGI(LOG_TAG "Gyro tmp_filaname %s gyro_num %d\n", tmp_filaname, gyro_num);
		if(!sysfs_read_scale(tmp_filaname, &scale_value)) {
			SENSOR_LOGI(LOG_TAG "gyro scale_value %f\n", scale_value);
			for(int i = 0; i < 6; i++)
				if(gyro_range[i][0] == scale_value)
					s->range = gyro_range[i][1];
		}
		mMaxGyroSampleRate = NearBySamplingRate(samplingRate, mMaxGyroSampleRate);
		s->maxSamplingRate = mMaxGyroSampleRate;
		if (mMinGyroBatchCount >= MAX_BATCH_COUNT)
			mMinGyroBatchCount = MAX_BATCH_COUNT;
		else if (mMinGyroBatchCount <= 0)
			mMinGyroBatchCount = 1;
		s->minBatchCount   = mMinGyroBatchCount;
        }
	mBatchConst =  3;
        }
        break;

      //Check for IAM20680 sensor
      case SENSOR_IAM20680: {
        if (s->type == SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED) {
                float samplingRate[6] = {6.25, 12.5, 25, 50, 100, 200};
                int acc_range[4] = {2, 4, 8, 16};
                memcpy(&s->odr[0], samplingRate, sizeof(samplingRate));
                s->range = (mAccRange >= 0 && mAccRange <= 3 ) ? acc_range[mAccRange] : acc_range[3];
                mMaxAccSampleRate  = NearBySamplingRate(samplingRate, mMaxAccSampleRate);
                s->maxSamplingRate = mMaxAccSampleRate;
                if (mMinAccBatchCount >= MAX_BATCH_COUNT)
                        mMinAccBatchCount = MAX_BATCH_COUNT;
                else if (mMinAccBatchCount <= 0)
                        mMinAccBatchCount = 1;
                s->minBatchCount   = mMinAccBatchCount;
        }
        if (s->type == SENSOR_TYPE_GYROSCOPE_UNCALIBRATED){
                float samplingRate[6] = {6.25, 12.5, 25, 50, 100, 200};
                int gyro_range[4] = {250, 500, 1000, 2000};
                memcpy(&s->odr[0], samplingRate, sizeof(samplingRate));
                s->range = (mGyroRange >= 0 && mGyroRange <= 3 ) ? gyro_range[mGyroRange] : gyro_range[3];
                mMaxGyroSampleRate = NearBySamplingRate(samplingRate, mMaxGyroSampleRate);
                s->maxSamplingRate = mMaxGyroSampleRate;
                if (mMinGyroBatchCount >= MAX_BATCH_COUNT)
                        mMinGyroBatchCount = MAX_BATCH_COUNT;
                else if (mMinGyroBatchCount <= 0)
                        mMinGyroBatchCount = 1;
                s->minBatchCount   = mMinGyroBatchCount;
        }
        mBatchConst =  1;
        }
        break;

      //Check for SMI130 sensor
      case SENSOR_SMI130: {
        if (s->type == SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED) {
                float samplingRate[6] = {15, 31, 62, 125, 250};
                int acc_range[1] = {2};
                memcpy(&s->odr[0], samplingRate, sizeof(samplingRate));
                s->range = acc_range[0];
                mMaxAccSampleRate  = NearBySamplingRate(samplingRate, mMaxAccSampleRate);
                s->maxSamplingRate = mMaxAccSampleRate;
                if (mMinGyroBatchCount >= MAX_BATCH_COUNT)
                        mMinGyroBatchCount = MAX_BATCH_COUNT;
		else if (mMinAccBatchCount <= 0)
                        mMinAccBatchCount = 1;
                s->minBatchCount   = 1;
        }
        if (s->type == SENSOR_TYPE_GYROSCOPE_UNCALIBRATED){
                float samplingRate[6] = {100, 200};
                int gyro_range[1] = {250};
                memcpy(&s->odr[0], samplingRate, sizeof(samplingRate));
                s->range = gyro_range[0];
                mMaxGyroSampleRate = NearBySamplingRate(samplingRate, mMaxGyroSampleRate);
                s->maxSamplingRate = mMaxGyroSampleRate;
                if (mMinGyroBatchCount >= MAX_BATCH_COUNT)
                        mMinGyroBatchCount = MAX_BATCH_COUNT;
		else if (mMinGyroBatchCount <= 0)
                        mMinGyroBatchCount = 1;
                s->minBatchCount   = 1;
        }
        mBatchConst =  0;
        }
        break;

      //Check for SMI230 sensor
      case SENSOR_SMI230: {
        if (s->type == SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED) {
                float samplingRate[6] = {12.5, 25, 50, 100, 200, 400};
                int acc_range[4] = {2, 4, 8, 16};
		char rangeFilePath[SEARCH_PATH_SIZE]={'\0'};
		find_path(DYN_INPUT_TYPE, rangeFilePath, "SMI230ACC", sizeof(rangeFilePath));
		strlcat(rangeFilePath, "range", sizeof(rangeFilePath));
		sysfs_read_int(rangeFilePath, &s->range);
                memcpy(&s->odr[0], samplingRate, sizeof(samplingRate));
                mMaxAccSampleRate  = NearBySamplingRate(samplingRate, mMaxAccSampleRate);
                s->maxSamplingRate = mMaxAccSampleRate;
                if (mMinAccBatchCount >= MAX_BATCH_COUNT)
                        mMinAccBatchCount = MAX_BATCH_COUNT;
                else if (mMinAccBatchCount <= 0)
                        mMinAccBatchCount = 1;
                s->minBatchCount   = mMinAccBatchCount;
	}
        if (s->type == SENSOR_TYPE_GYROSCOPE_UNCALIBRATED){
                float samplingRate[6] = {100, 200, 400};
                int gyro_range[5] = {125, 250, 500, 1000, 2000};
		char rangeFilePath[SEARCH_PATH_SIZE]={'\0'};
		find_path(DYN_INPUT_TYPE, rangeFilePath, "SMI230GYRO", sizeof(rangeFilePath));
		strlcat(rangeFilePath, "range", sizeof(rangeFilePath));
		sysfs_read_int(rangeFilePath, &s->range);
                memcpy(&s->odr[0], samplingRate, sizeof(samplingRate));
                mMaxGyroSampleRate = NearBySamplingRate(samplingRate, mMaxGyroSampleRate);
                s->maxSamplingRate = mMaxGyroSampleRate;
		if (mMinGyroBatchCount >= MAX_BATCH_COUNT)
                        mMinGyroBatchCount = MAX_BATCH_COUNT;
                else if (mMinGyroBatchCount <= 0)
                        mMinGyroBatchCount = 1;
                s->minBatchCount   = mMinGyroBatchCount;
        }
        mBatchConst =  1;
        }
        break;

      //Check for BMI160 sensor
      case SENSOR_BMI160: {
	if (s->type == SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED) {
		float samplingRate[6] = {25, 50, 100, 200, 400};
		int acc_range[1] = {2};
		memcpy(&s->odr[0], samplingRate, sizeof(samplingRate));
		s->range = acc_range[0];
		mMaxAccSampleRate  = NearBySamplingRate(samplingRate, mMaxAccSampleRate);
		s->maxSamplingRate = mMaxAccSampleRate;
		s->minBatchCount   = 1;
        }
	if (s->type == SENSOR_TYPE_GYROSCOPE_UNCALIBRATED){
		float samplingRate[6] = {25, 50, 100, 200, 400};
		int gyro_range[1] = {250};
		memcpy(&s->odr[0], samplingRate, sizeof(samplingRate));
		s->range = gyro_range[0];
		mMaxGyroSampleRate = NearBySamplingRate(samplingRate, mMaxGyroSampleRate);
		s->maxSamplingRate = mMaxGyroSampleRate;
		s->minBatchCount   = 1;
        }
        mBatchConst =  0;
        }
	break;

      //default
      default: {
        float samplingRate[6] = {0};
	memcpy(&s->odr[0], samplingRate, sizeof(samplingRate));
	mBatchConst =  0;
	s->range = 0;
        }
        break;
  }
}

/******************************************************************************
SensorApiService - power event handlers
******************************************************************************/
#ifdef POWERMANAGER_ENABLED
void SensorApiService::onPowerEvent(PowerStateType powerState) {
    std::lock_guard<std::mutex> lock(mMutex);
    SENSOR_LOGI(LOG_TAG "--< onPowerEvent %d", powerState);
    mPowerState = powerState;
}
#endif
