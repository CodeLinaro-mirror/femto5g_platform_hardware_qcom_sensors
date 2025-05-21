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

/******************************************************************************
SensorApiService - static members
******************************************************************************/
SensorApiService* SensorApiService::mInstance = nullptr;
mutex SensorApiService::mMutex;

#ifdef SENSOR_HEAD_TYPE_SUPPORT
static LocationClientApi* pLcaClient = nullptr;
#endif

/******************************************************************************
SensorApiService - constructors
******************************************************************************/
SensorApiService::SensorApiService(const configParamToRead & configParamRead) :
    mSensorCount(0),
    mSensorList(nullptr),
    mSensorMlcCaseCount(0),
    mSensorMlcCaseList(nullptr),
    mSensorClient(0),
    mSensor(nullptr),
    mIpcReceiver(nullptr),
    mQsockReceiver(nullptr),
    mMaxAccSampleRate(configParamRead.MaxAccSampleRate),
    mMaxGyroSampleRate(configParamRead.MaxGyroSampleRate),
    mMinAccBatchCount(configParamRead.MinAccBatchCount),
    mMinGyroBatchCount(configParamRead.MinGyroBatchCount),
    mAccRange(configParamRead.AccRange),
    mGyroRange(configParamRead.GyroRange),
    mEnableFIR(configParamRead.EnableFIR),
    mhmi(nullptr),
    mdev(nullptr),
    mpoll_dev_v0(nullptr),
    mpoll_dev(nullptr),
    mBufferSupported(false),
    mTempSupported(false),
    mBufferDeleted(false),
    mMlcSupported(false),
#ifdef PTP_SUPPORTED
    mGptpInitialized(false)
#endif
#ifdef POWERMANAGER_ENABLED
    ,mPowerEventObserver(nullptr)
#endif
{
    SENSOR_LOGI(LOG_TAG "SensorApiService constructor is called\n");

    //Enable Diag if enabled
    if(CheckDiagEnabled(DIAG_CLIENT_SHD))
    {
        SENSOR_LOGI(LOG_TAG "diag is enabled\n");
        (void)mDiagLogger.EnableDiag();
    }
    else
    {
        SENSOR_LOGI(LOG_TAG "diag is disabled\n");
    }

    //Check Sensor Availability
    if(!openSensor(configParamRead)) {
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

#ifdef PTP_SUPPORTED
    if (gptpInit()) {
	    SENSOR_LOGI(LOG_TAG "gptpinit success\n");
	    mGptpInitialized = true;
    }
    else {
	    SENSOR_LOGE(LOG_TAG "gptpinit failed\n");
	    mGptpInitialized = false;
    }
#endif

    //read Euler angles from file
    (void)init_sensor_rotation_matrix(rot);
    if ( !read_sensor_rotation_matrix(&roll, &pitch, &yaw) ) {
       SENSOR_LOGI(LOG_TAG "Sensor Euler anglers <roll %d, pitch %d, yaw %d>\n", roll, pitch, yaw);
       (void)calculate_sensor_rotation_matrix(roll, pitch, yaw, rot);
       SENSOR_LOGI(LOG_TAG "Sensor rotation matrix: \t%5.2f %5.2f %5.2f\t%5.2f %5.2f %5.2f\t%5.2f %5.2f %5.2f\n",
		       rot[0][0], rot[0][1], rot[0][2],
		       rot[1][0], rot[1][1], rot[1][2],
		       rot[2][0], rot[2][1], rot[2][2]);
    }

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

#ifdef SENSOR_IVSS_ENABLED
    CommonAPI::Runtime::setProperty("LogContext", "SensorInterface");
    CommonAPI::Runtime::setProperty("LogApplication", "SensorInterface");
    CommonAPI::Runtime::setProperty("LibraryBase", "SensorInterface");

    shared_ptr<CommonAPI::Runtime> runtime = CommonAPI::Runtime::get();

    string domain = "local";
    string instance = "com.qualcomm.qti.sensor.SensorInterface";
    string connection = "sensor-fidl-service";
    myService = make_shared<SensorInterfaceStubImpl>(this);
    bool successfullyRegistered = runtime->registerService(domain, instance, myService, connection);

    while (!successfullyRegistered) {
        SENSOR_LOGE(LOG_TAG "Register SOMEIP Service failed, trying again in 100 milliseconds...\n");
        this_thread::sleep_for(chrono::milliseconds(100));
        successfullyRegistered = runtime->registerService(domain, instance, myService, connection);
    }

    SENSOR_LOGI(LOG_TAG "Successfully Registered SOMEIP Service!\n");
#endif
    // start receiver - never return
    SENSOR_LOGI(LOG_TAG "Ready, start Ipc Receiver\n");
    // blocking: set to false
    (void)mIpcReceiver->start(false);

    SENSOR_LOGI(LOG_TAG "Ready, start qsock Receiver\n");
    // blocking: set to true
    (void)mQsockReceiver->start(true);
}

/******************************************************************************
SensorApiService - Destructors
******************************************************************************/
SensorApiService::~SensorApiService() {
    SENSOR_LOGI(LOG_TAG "SensorApiService Destructor is called\n");

    for(int i = 0 ; i < mSensorCount; i++)  {
        SENSOR_LOGI(LOG_TAG ">--Disable the sensor mSensor[i].sensor_id %d\n", mSensor[i].sensor_id);
        (void)sensorActivate(mSensor[i].sensor_id, SENSOR_DISABLE); //Disable the sensor
    }

    //Delete mSensorDevice memory
    if (nullptr != mSensorDevice) {
	delete mSensorDevice;
	mSensorDevice = nullptr;
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

    //Delete mSensorMlcCaseList memory
    if (nullptr != mSensorMlcCaseList) {
        delete mSensorMlcCaseList;
        mSensorMlcCaseList = nullptr;
    }

    (void)mDiagLogger.DisableDiag();

    SENSOR_LOGI(LOG_TAG "SensorApiService destructor has executed\n");
}

/******************************************************************************
  SensorApiService - onListenerReady send HAL READY message to all clients.
******************************************************************************/
void SensorApiService::onListenerReady() {

    // traverse client sockets directory - then broadcast READY message
    SENSOR_LOGI(LOG_TAG ">-- onListenerReady Finding client sockets...\n");

    DIR *dirp = opendir(SOCKET_SENSOR_CLIENT_DIR);
    if (!dirp) {
        return;
    }

    struct dirent *dp = nullptr;
    struct stat sbuf = {0};
    const string fnamebase = SOCKET_TO_SENSOR_CLIENT_BASE;
    while (nullptr != (dp = readdir(dirp))) {
        string fname = SOCKET_SENSOR_CLIENT_DIR;
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
            (void)pIpcSender->send(reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
            delete pIpcSender;
        }
    }
    (void)closedir(dirp);
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
           SENSOR_LOGD(LOG_TAG"\tmaxSamplingRate: %d\n",sensor[i].maxSamplingRate);
           SENSOR_LOGD(LOG_TAG"\tminBatchCount: %d\n",sensor[i].minBatchCount);
           SENSOR_LOGD(LOG_TAG"\tmaxBatchCount: %d\n",sensor[i].maxBatchCount);
           SENSOR_LOGD(LOG_TAG"\todr rate: %fHZ %fHZ %fHZ %fHZ %fHZ %fHZ\n",
                           sensor[i].odr[0],sensor[i].odr[1],sensor[i].odr[2],
                           sensor[i].odr[3],sensor[i].odr[4],sensor[i].odr[5]);
   }
}

/******************************************************************************
SensorApiService - openSensor to check sensor supported by device
******************************************************************************/
bool SensorApiService::openSensor(const configParamToRead & configParamRead)
{

   void *hal;
   int err;
   const struct sensor_t *s;
   string sensor_lib, name;

   mSensorDevice = new SensorDevice (this);

   if (!mSensorDevice->openSensorDevice(&sensor_lib)) {
	   SENSOR_LOGE(LOG_TAG "ERROR: not able to find any sensor\n");
	   return false;
   } else  {
	   SENSOR_LOGI(LOG_TAG "Sensor found and lib %s\n", sensor_lib.c_str());
   }
   mSensorDevice->setDefaultFIRCoeff();

   //Open sensor Lib
   hal = dlopen(sensor_lib.c_str(), RTLD_NOW);
   if (!hal) {
	   SENSOR_LOGE(LOG_TAG "ERROR: unable to load HAL %s: %s\n", sensor_lib.c_str(),
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


   err = mhmi->common.methods->open((struct hw_module_t *)mhmi,
		   SENSORS_HARDWARE_POLL, &mdev);
   if (err) {
	   SENSOR_LOGE(LOG_TAG "ERROR: failed to initialize HAL: %d\n", err);
	   return false;
   }

   mpoll_dev = (struct sensors_poll_device_1 *)mdev;
   mpoll_dev_v0 = (struct sensors_poll_device_t *)mdev;

   //Get Sensor List Supported by HAL
   mSensorCount = getSensorListDevice(&s);
   if (mSensorCount <= 0 ){
      mSensorCount = 0;
      return false;
   }


   //Copy data to mSensor to track the sensor configuration parameters till last client deregistered.
   mSensor = new (nothrow) SensorConfig[mSensorCount];
   if (mSensor == nullptr){
	return false;
   }
   for(int i=0; i < mSensorCount; i++) {
     mSensor[i].sensor_id    = s[i].handle;
     if (s[i].type == SENSOR_TYPE_ACCELEROMETER || s[i].type == SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED)
	     mSensor[i].type = SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED;
     if (s[i].type == SENSOR_TYPE_GYROSCOPE || s[i].type == SENSOR_TYPE_GYROSCOPE_UNCALIBRATED)
	     mSensor[i].type = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
     mSensor[i].Activate     = 0;
     mSensor[i].SamplingRate = 0;
     mSensor[i].BatchCount   = 0;
     mSensor[i].selfTest     = NotAvailable;
     mSensor[i].selfTestTS   = 0;
   }

   //Create Sensor List to send to all Clients.
   mSensorList = new (nothrow) struct sensor_list[mSensorCount];
   if (mSensorList == nullptr)
	   return false;
   mSensorDevice->getSensorDeviceName(&name);
   for(int i= 0; i< mSensorCount; i++) {
     if (s[i].type == SENSOR_TYPE_ACCELEROMETER || s[i].type == SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED) {
	     strlcpy(&mSensorList[i].name[0], configParamRead.AccelName, MAX_PATH_SIZE);
	     mSensorList[i].type = SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED;
     }
     if (s[i].type == SENSOR_TYPE_GYROSCOPE || s[i].type == SENSOR_TYPE_GYROSCOPE_UNCALIBRATED) {
	     (void)strlcpy(&mSensorList[i].name[0], configParamRead.GyroName, MAX_PATH_SIZE);
	     mSensorList[i].type = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
     }
     strlcpy(&mSensorList[i].vendor[0], s[i].vendor, MAX_PATH_SIZE);
     name = "-" + name;
     strlcat(mSensorList[i].vendor, name.c_str(), MAX_PATH_SIZE);
     mSensorList[i].version = s[i].version;
     mSensorList[i].resolution = s[i].resolution;
     mSensorList[i].maxRange = s[i].maxRange;
     mSensorList[i].sensor_id = s[i].handle;
     mSensorList[i].maxBatchCount = MAX_BATCH_COUNT;
     mSensorDevice->getSupportedSamplingRateAndRange(&mSensorList[i]);
   }

   //Print sensor list for debug
   PrintSensorList(mSensorList, mSensorCount);

   //Initialize Temp Sensor
   if (mSensorDevice->checkTempSupport()) {
     SENSOR_LOGI(LOG_TAG "Sensor Temperatue supported\n");
     mTempSupported = true;
   }

   //Check Buffer support
   if (checkSensorBufferSupport()) {
     SENSOR_LOGI(LOG_TAG "Sensor Buffer supported\n");
     mBufferSupported = true;
   }

   //Load MLC firmware if available in /lib/firmware folder
   if (mSensorDevice->loadMLC(PATH_MLC_BINARY)) {
     SENSOR_LOGI(LOG_TAG "MLC features are supported\n");
     mMlcSupported = true;
   }

   //Create the thread to send data to all clients.
   if (!Sensor_ThreadCreate(&mSensorThreadtid, sendSensorDataToClients, this, "SensorPoll-")) {
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
SensorApiService - sensorActivate to actiate/deactivate the sensor
******************************************************************************/
int SensorApiService::sensorActivate(int sensor_id , int enable)
{
   return mpoll_dev->activate(mpoll_dev_v0, sensor_id, enable);
}

/******************************************************************************
SensorApiService - sensorSetBatch to configure the sensor
******************************************************************************/
int SensorApiService::sensorSetBatch(int sensor_id, int64_t delay, int64_t latency)
{
   return mpoll_dev->batch(mpoll_dev, sensor_id, 0, delay, latency);
}

/******************************************************************************
SensorApiService - getSensorListDevice to get the sensor list supported by device
******************************************************************************/
int SensorApiService::getSensorListDevice(struct sensor_t const **s)
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
SensorApiService - sendSensorDataToClients thread to process sensor data
******************************************************************************/
void* SensorApiService::sendSensorDataToClients(void *arg) {
  SensorApiService* mSensorService = (SensorApiService*)(arg);
  int count = 0; bool rc = false;
  sensors_event_t events[BUFFER_EVENT];
  while(1)
  {
     count = mSensorService->mpoll_dev->poll(mSensorService->mpoll_dev_v0,
            events, sizeof(events)/sizeof(sensors_event_t));
     SENSOR_LOGV(LOG_TAG "read events = %d\n",count);
     lock_guard<mutex> lock(SensorApiService::mMutex);

#ifdef POWERMANAGER_ENABLED
     if ((POWER_STATE_SUSPEND != mSensorService->mPowerState) &&
        (POWER_STATE_SHUTDOWN != mSensorService->mPowerState)) 
#endif
    {
	auto it = mSensorService->mClients.begin();
	while (it != mSensorService->mClients.end() && it != (unordered_map<string, SensorHalDaemonClientHandler*>::iterator)NULL) {
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

        if(mSensorService->mDiagLogger.IsEnabled())
        {
            uint64_t accCountLocal = 0;
            uint64_t gyroCountLocal = 0;
            for(uint64_t i=0; i<count; i++)
            {
                switch(events[i].type)
                {
                    case SENSOR_TYPE_ACCELEROMETER:
                    case SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED:
                        (void)mSensorService->mDiagLogger.SendSensorLiveAccelEvent(&events[i], ++accCountLocal);
                        break;
                    case SENSOR_TYPE_GYROSCOPE:
                    case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
                        (void)mSensorService->mDiagLogger.SendSensorLiveGyroEvent(&events[i], ++gyroCountLocal);
                        break;
                }
            }
            (void)mSensorService->mDiagLogger.CommitToDiagAccelLive();
            (void)mSensorService->mDiagLogger.CommitToDiagGyroLive();
        }
    }
  }
  return NULL;
}

/******************************************************************************
SensorApiService - bufferDataprocessTask thread to process buffer data
******************************************************************************/
void* SensorApiService::bufferDataprocessTask(void * arg)
{
  SensorApiService* mSensorService = (SensorApiService*)(arg);
  mSensorService->sensorBufferReadThread();
  return 0;
}


/******************************************************************************
  SensorApiService - processClientMsg recieved from clients
******************************************************************************/
void SensorApiService::processClientMsg(const string& data) {

    SensorAPIMsgHeader* pMsg = (SensorAPIMsgHeader*)(data.data());
    uint32_t length = data.length();

    switch (pMsg->msgId) {
        case E_SENSORAPI_CLIENT_REGISTER_MSG_ID: {
            // new client
            if (sizeof(SensorAPIClientRegisterReqMsg) != length) {
                SENSOR_LOGE(LOG_TAG "invalid message\n");
                break;
            }
            (void)newClient(reinterpret_cast<SensorAPIClientRegisterReqMsg*>(pMsg));
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
	    (void)activateSensor(reinterpret_cast<SensorAPIEnableReqMsg*>(pMsg));
            break;
        }
        case E_SENSORAPI_START_BATCHING_MSG_ID: {
            // start batching
            if (sizeof(SensorAPIStartBatchingReqMsg) != length) {
                SENSOR_LOGE(LOG_TAG "invalid message\n");
                break;
            }
            (void)startBatching(reinterpret_cast<SensorAPIStartBatchingReqMsg*>(pMsg));
            break;
        }
        case E_SENSORAPI_START_TRACKING_MSG_ID: {
            // start tracking
            if (sizeof(SensorAPIStartTrackingReqMsg) != length) {
                SENSOR_LOGE(LOG_TAG "invalid message\n");
                break;
            }
            (void)startTracking(reinterpret_cast<SensorAPIStartTrackingReqMsg*>(pMsg));
            break;
        }
        case E_SENSORAPI_SENSOR_MLC_CASE_ENABLE_MSG_ID: {
            // Sensor MLC case enabel/disable request
            if (sizeof(SensorAPIMLCCaseEnableMsg) != length) {
                SENSOR_LOGE(LOG_TAG "invalid message\n");
                break;
            }
	    sensorEnableMLCCase(reinterpret_cast<SensorAPIMLCCaseEnableMsg*>(pMsg));
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
        case E_SENSORAPI_SENSOR_EULER_ANGLES_REQ_MSG_ID: {
            // Sensor Euler Angles
            if (sizeof(SensorAPIEulerAnglesReqMsg) != length) {
                SENSOR_LOGE(LOG_TAG "invalid message\n");
                break;
            }
            setEulerAngles(reinterpret_cast<SensorAPIEulerAnglesReqMsg*>(pMsg));
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
int SensorApiService::newClient(SensorAPIClientRegisterReqMsg *pMsg) {

    lock_guard<mutex> lock(mMutex);
    string clientname(pMsg->mSocketName);
    SENSOR_LOGI(LOG_TAG ">-- newClient %s\n", clientname.c_str());
    int ret = SENSOR_RESPONSE_SUCCESS;

    // if this name is already used return error
    if (mClients.find(clientname) != mClients.end()) {
        SENSOR_LOGE(LOG_TAG "invalid client=%s already existing\n", clientname.c_str());
        return ret;
    }

    // store it in client property database
    SensorHalDaemonClientHandler *pClient =
            new SensorHalDaemonClientHandler(this, clientname, pMsg->mClientType, mSensorCount);
    if (!pClient) {
        SENSOR_LOGE(LOG_TAG "failed to register client=%s\n", clientname.c_str());
	ret = SENSOR_ERROR_CLIENT_REGISTER_FAILED;
        return ret;
    }
    //Send Sensor List to client
    if(mSensorCount > 0)
	    pClient->onSensorListCb(mSensorList, mSensorCount);
    if (mSensorMlcCaseCount > 0)
	    pClient->onSensorMlcCaseListCb(mSensorMlcCaseList, mSensorMlcCaseCount);

    (void)pClient->onCapabilitiesCallback(SHD_READY);

    mSensorClient++;
    (void)mClients.emplace(clientname, pClient);
    SENSOR_LOGI(LOG_TAG ">-- registered new client=%s\n", clientname.c_str());

    return ret;
}

/******************************************************************************
SensorApiService - implementation - deregistration
******************************************************************************/
void SensorApiService::deleteClient(SensorAPIClientDeregisterReqMsg *pMsg) {
    lock_guard<mutex> lock(mMutex);
    SENSOR_LOGI(LOG_TAG ">-- deleteClient\n");
    string clientname(pMsg->mSocketName);

    (void)deleteClientbyName(clientname);
}

unordered_map<string, SensorHalDaemonClientHandler*>::iterator SensorApiService::deleteClientbyName(const string clientname) {
    // We shall not hold the lock, as lock already held by the caller
    //
    mSensorClient--;
    bool activate = false;
    SENSOR_LOGI(LOG_TAG ">-- deleteClientbyName %s mSensorClient %d\n",clientname.c_str(), mSensorClient);

    // remove the client from the config request map
    auto it = mConfigReqs.begin();
    while (it != mConfigReqs.end()) {
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
        return (unordered_map<string, SensorHalDaemonClientHandler*>::iterator)NULL;
    }

    unordered_map<string, SensorHalDaemonClientHandler*>::iterator itr = mClients.find(clientname);
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
         (void)sensorActivate(mSensor[i].sensor_id, SENSOR_DISABLE);
         mSensor[i].Activate = 0;
         mSensor[i].SamplingRate = 0;
         mSensor[i].BatchCount = 0;
       }
    }

    SENSOR_LOGI(LOG_TAG ">-- deleteClient client=%s\n", clientname.c_str());
    return itr;
}

const char* SensorApiService::getClientNameByIds(int id1, int id2) {
    for (auto it = mClients.begin(); it != mClients.end(); ++it) {
	    if (it->second->getServiceId() == id1 && it->second->getInstanceId() == id2) {
		    return it->first.c_str();
	    }
    }
    return nullptr;
}

void SensorApiService::deleteEapClientByIds(int serviceId, int instanceId) {

    lock_guard<mutex> lock(mMutex);
    const char* clientName = getClientNameByIds(serviceId, instanceId);
    if (clientName) {
        SENSOR_LOGI(LOG_TAG ">-- service id: %d, instance id: %d, client name: %s",
                 serviceId, instanceId, clientName);
        (void)deleteClientbyName(string(clientName));
    }
}

/******************************************************************************
SensorApiService - implementation - StartTracking
******************************************************************************/
int SensorApiService::startTracking(SensorAPIStartTrackingReqMsg *pMsg) {
    lock_guard<mutex> lock(mMutex);
    SENSOR_LOGI(LOG_TAG ">-- startTracking\n");

    int ret = 0;
    SensorHalDaemonClientHandler* pClient = getClient(pMsg->mSocketName);
    if (!pClient) {
        SENSOR_LOGE(LOG_TAG ">-- start invlalid client=%s\n", pMsg->mSocketName);
        return ret;
    }

    pClient->mTracking = true;
    pClient->mPendingMessages.push(E_SENSORAPI_START_TRACKING_MSG_ID);
    pClient->onResponseCb(ret, E_SENSORAPI_START_TRACKING_MSG_ID);

    SENSOR_LOGI(LOG_TAG ">-- star session AccTracking %d GyroTracking %d\n",
		    pClient->mAccTracking, pClient->mGyroTracking);

    return ret;
}

/******************************************************************************
SensorApiService - implementation - StartBatching
******************************************************************************/
int SensorApiService::sensorCofig(SensorAPIStartBatchingReqMsg *pMsg) {
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
           if (mSensor[i].type == SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED) {
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
                     ret = sensorSetBatch(pMsg->sensor_id, SamplingRate, BatchingRate);
                     if (ret != 0) {
                          ret = SENSOR_ERROR_CONFIG_FAILED;
                          break;
		     }
             }
	     //Calcualate the client config parameters based on sensor physical configuration
	     pClient->mAccFactor = lroundf(mSensor[i].SamplingRate / pMsg->samplingRate);
	     pMsg->batchCount = nearByBatchCount(mMinAccBatchCount, pMsg->batchCount,mSensor[i].SamplingRate, 
			                          pMsg->samplingRate, pClient->mAccFactor);
	     pClient->mAccBatchCount = pMsg->batchCount;
	     pClient->mAccCount = 0;
	     pClient->mAccMovingCount = 0;
	     pClient->mAccTracking = false;
	     pClient->mAccRotate = pMsg->rotate;
	     if(mEnableFIR)
	     {
		    pClient->setFIRFilter(mSensor[i].SamplingRate, mSensor[i].SamplingRate / pClient->mAccFactor, true);
	     }
	     //Allocate memory to store acc events based on requested batch count by client
	     if (pClient->mAccEvents) {
		     delete pClient->mAccEvents;
		     pClient->mAccEvents = nullptr;
             }
	     pClient->mAccEvents = new (nothrow) sensors_event_t [pClient->mAccBatchCount];

	     if (pClient->mAccEvents == nullptr){
		return SENSOR_ERROR_CONFIG_FAILED;
	     }

	     (void)memset(pClient->mAccEvents, 0, sizeof(sensors_event_t) * pClient->mAccBatchCount);
           }

	   ////Check for Gyro Parameteters////
           if (mSensor[i].type == SENSOR_TYPE_GYROSCOPE_UNCALIBRATED) {
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
                     ret = sensorSetBatch(pMsg->sensor_id, SamplingRate, BatchingRate);
                     if (ret != 0) {
                          ret = SENSOR_ERROR_CONFIG_FAILED;
                          break;
		     }
	     }
	     //Calcualate the client config parameters based on sensor physical configuration
	     pClient->mGyroFactor = lroundf(mSensor[i].SamplingRate / pMsg->samplingRate);
	     pMsg->batchCount = nearByBatchCount(mMinGyroBatchCount, pMsg->batchCount, mSensor[i].SamplingRate, pMsg->samplingRate, pClient->mGyroFactor);
	     pClient->mGyroBatchCount = pMsg->batchCount;
	     pClient->mGyroCount = 0;
	     pClient->mGyroMovingCount = 0;
	     pClient->mGyroTracking = false;
	     pClient->mGyroRotate = pMsg->rotate;
	     if(mEnableFIR)
	     {
		    pClient->setFIRFilter(mSensor[i].SamplingRate, mSensor[i].SamplingRate / pClient->mGyroFactor, false);
	     }
	     //Allocate memory to store Gyro events based on requested batch count by client
	     if (pClient->mGyroEvents) {
		     delete pClient->mGyroEvents;
		     pClient->mGyroEvents = nullptr;
	     }
	     pClient->mGyroEvents = new (nothrow) sensors_event_t [pClient->mGyroBatchCount];

	     if (pClient->mGyroEvents == nullptr) {
		return SENSOR_ERROR_CONFIG_FAILED;
	     }

	     (void)memset(pClient->mGyroEvents, 0, sizeof(sensors_event_t) * pClient->mGyroBatchCount);
	   }
	 }
    }

    SENSOR_LOGI(LOG_TAG ">-- start batching session AccFactor %d AccBatchcount %d GyroFactor %d GyroBatchCount %d \
      mAccRotate %d, mGyroRotate %d, AccTracking %d GyroTracking %d\n", pClient->mAccFactor, pClient->mAccBatchCount, pClient->mGyroFactor,
      pClient->mGyroBatchCount, pClient->mAccRotate, pClient->mGyroRotate, pClient->mAccTracking, pClient->mGyroTracking);

    return ret;
}

int SensorApiService::startBatching(SensorAPIStartBatchingReqMsg *pMsg) {
    lock_guard<mutex> lock(mMutex);
    int ret = 0;
    bool SensorId = false;

    SensorHalDaemonClientHandler* pClient = getClient(pMsg->mSocketName);
    if (!pClient) {
	    SENSOR_LOGE(LOG_TAG ">-- start invalid client=%s\n", pMsg->mSocketName);
	    return ret;
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
			      pMsg->samplingRate = nearBySamplingRate(mSensorList[i].odr, pMsg->samplingRate);
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
    ret = sensorCofig(pMsg);

fail:
    pClient->mPendingMessages.push(E_SENSORAPI_START_BATCHING_RES_ID);
    pClient->onResponseCb(ret, E_SENSORAPI_START_BATCHING_RES_ID);
    if(ret == 0)
	    pClient->onSensorBatchingCb(pMsg->sensor_id, pMsg->samplingRate,  pMsg->batchCount, pMsg->rotate);

    return ret;
}

/******************************************************************************
SensorApiService - implementation - Activate/Deactivate Sensor
******************************************************************************/
int SensorApiService::activateSensor(SensorAPIEnableReqMsg* pMsg) {
   lock_guard<mutex> lock(mMutex);
   int ret = 0;
   int enable = 0;
   bool SensorId = false;

   SensorHalDaemonClientHandler* pClient = getClient(pMsg->mSocketName);
   if (!pClient) {
	   SENSOR_LOGI(LOG_TAG ">-- start invlalid client=%s\n", pMsg->mSocketName);
	   return ret;
   }
   SENSOR_LOGI(LOG_TAG "<-- sensor sensor_id %d sensor enable %d \n", pMsg->sensor_id, pMsg->enable);

   //Input parameter check
   if (pMsg->enable < SENSOR_DISABLE  || pMsg->enable > SENSOR_HPM) {
	   ret=SENSOR_ERROR_INVALID_INPUT_PARAMETER;
	   goto fail;
   }

#ifdef POWERMANAGER_ENABLED
    //Don't allow activation when system is in suspend state
    if(mPowerState == POWER_STATE_SUSPEND && pMsg->enable == SENSOR_ENABLE)
    {
        ret = SENSOR_ERROR_NOT_SUPPORTED;
        goto fail;
    }
#endif //POWERMANAGER_ENABLED

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
	   ret = mSensorDevice->setPowerMode(pMsg->sensor_id, pMsg->enable);
	   goto fail;
   }

   //Activating/Deactivating the sensor
   for (int i=0 ; i < mSensorCount; i++) {
     if (pMsg->sensor_id == mSensor[i].sensor_id) {
	pClient->mActivate[i] = pMsg->enable;
	if (mSensor[i].type == SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED) {
		pClient->mAccTracking = pMsg->enable;
		pClient->mAccCount = 0;
		pClient->mAccMovingCount = 0;
	}
	if (mSensor[i].type == SENSOR_TYPE_GYROSCOPE_UNCALIBRATED) {
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
           SENSOR_LOGI(LOG_TAG "<-- calling sensorActivate sensor_id %d  enable %d \n",pMsg->sensor_id, enable);
	   ret = sensorActivate(pMsg->sensor_id , enable);
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
   return ret;
}

/******************************************************************************
SensorApiService - implementation - getSensorList - to send the sensor list
******************************************************************************/
void SensorApiService::getSensorList(SensorAPIListReqMsg* pMsg) {
    lock_guard<mutex> lock(mMutex);

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
SensorApiService - implementation - sensorMLCCaseEnable to enable/disable mlc cases
******************************************************************************/
void SensorApiService::sensorEnableMLCCase(SensorAPIMLCCaseEnableMsg* pMsg) {
    lock_guard<mutex> lock(mMutex);
    int ret = SENSOR_ERROR_MLC_EVENT_ENABLE_FAILED;

    SENSOR_LOGI(LOG_TAG "--<SensorEnableMLCCase name %s enable %d \n",
		    pMsg->mlc_case_name, pMsg->enable);

    SensorHalDaemonClientHandler* pClient = getClient(pMsg->mSocketName);
    if (!pClient) {
            SENSOR_LOGE(LOG_TAG ">-- SensorEnableMLCCase invlalid client=%s\n", pMsg->mSocketName);
            return;
    }

    //Input parameter check
    if (pMsg->enable != true && pMsg->enable != false) {
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
	      if(mSensorDevice->sensorMlcEnableEvents(pMsg->mlc_case_name, pMsg->enable))
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
	    mSensorDevice->readSensorTemperature(&temperature);

    pClient->onSensorTempCb(temperature);
}

/******************************************************************************
SensorApiService - implementation - getSensorBufferData to send buffer data
******************************************************************************/
void SensorApiService::getSensorBufferData(SensorAPIBufferDataReqMsg* pMsg) {
    lock_guard<mutex> lock(mMutex);
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
                    rc = writeToBufferFile(pMsg->enable);
                    if(rc){
			ret = SENSOR_RESPONSE_SUCCESS;
                    }
		    pClient->mPendingMessages.push(E_SENSORAPI_SENSOR_BUFFER_REQ_MSG_ID);
		    pClient->onResponseCb(ret, E_SENSORAPI_SENSOR_BUFFER_REQ_MSG_ID);
		    (void)pthread_mutex_lock (&mHalBuffMutex);
		    (void)pthread_cond_signal (&mHalBuffCond);
		    (void)pthread_mutex_unlock (&mHalBuffMutex);
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

    bool onDemand = true;
    bool result = true;
    lock_guard<mutex> lock(mMutex);
    SENSOR_LOGI(LOG_TAG "--< onSelfTestRequest sensor_id %d SelfTestType %d request_id %d\n",
			sensor_id, selfTestType, request_id);
    SelfTestResultType sensorState = SelfTestResultType::SENSOR_BUSY;

    //send real live result if  sensor is idle else last known result
    for(int i = 0 ; i < mSensorCount; i++)  {
        if (sensor_id == mSensor[i].sensor_id) {
	   if (mSensor[i].Activate != SENSOR_ENABLE || POWER_STATE_SUSPEND == mPowerState) {
	      sensorState = SelfTestResultType::SENSOR_IDLE;
	      result = mSensorDevice->sensorDevSelfTest(mSensor[i].sensor_id, mSensor[i].type, selfTestType,
			     &mSensor[i].selfTest, &mSensor[i].selfTestTS);
	      if(!result)
		      sensorState = SelfTestResultType::SENSOR_BUSY;
	   }
	   SENSOR_LOGI(LOG_TAG "sensor_id = %d request_id = %d sensorState %d result %d timestamp %lld\n",
			   sensor_id, request_id, sensorState, mSensor[i].selfTest, mSensor[i].selfTestTS);
	   pClient->onSensorSelfTestResultCb(sensor_id, request_id, mSensor[i].selfTest,
			   sensorState, mSensor[i].selfTestTS);
	}
    }
    return;
}

void SensorApiService::sensorSelfTest(SensorAPISelfTestReqMsg* pMsg) {
    lock_guard<mutex> lock(mMutex);
    int ret = 0;
    bool SensorId = false;
    int value = 0;
    int sensor_id =  pMsg->sensor_id;
    int request_id = pMsg->request_id;
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
	      if (pMsg->selfTestType != Positive && pMsg->selfTestType != Negative && pMsg->selfTestType != All) {
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

    pClient->mPendingMessages.push(E_SENSORAPI_SENSOR_SELFTEST_REQ_MSG_ID);
    pClient->onResponseCb(ret, E_SENSORAPI_SENSOR_SELFTEST_REQ_MSG_ID);
    thread appCallbackThread([this, pClient, sensor_id, selfTestType, request_id] {
		    onSelfTestRequest(pClient, sensor_id, selfTestType, request_id);
		    });
    appCallbackThread.detach();

    return;
}

/******************************************************************************
SensorApiService - implementation - update rotation matrix of sensor
*****************************************************************************/
void SensorApiService::setEulerAngles(SensorAPIEulerAnglesReqMsg*  pMsg) {
    lock_guard<mutex> lock(mMutex);
    int ret = 0;
    SENSOR_LOGI(LOG_TAG ">>> setEulerAngles roll %d  pitch %d yaw %d\n", pMsg->roll, pMsg->pitch, pMsg->yaw);
    SensorHalDaemonClientHandler* pClient = getClient(pMsg->mSocketName);
    if (!pClient) {
	    SENSOR_LOGE(LOG_TAG ">-- setEulerAngles invlalid client=%s\n", pMsg->mSocketName);
	    return;
    }
    //write Euler angles to file
    (void)update_sensor_rotation_matrix(pMsg->roll, pMsg->pitch, pMsg->yaw);
    //update rotation matrix
    (void)calculate_sensor_rotation_matrix(pMsg->roll, pMsg->pitch, pMsg->yaw, rot);
    //send response back to client
    pClient->mPendingMessages.push(E_SENSORAPI_SENSOR_EULER_ANGLES_REQ_MSG_ID);
    pClient->onResponseCb(ret, E_SENSORAPI_SENSOR_EULER_ANGLES_REQ_MSG_ID);

    SENSOR_LOGI(LOG_TAG "Sensor rotation matrix: \t%5.2f %5.2f %5.2f\t%5.2f %5.2f %5.2f\t%5.2f %5.2f %5.2f\n",
		    rot[0][0], rot[0][1], rot[0][2],
		    rot[1][0], rot[1][1], rot[1][2],
		    rot[2][0], rot[2][1], rot[2][2]);
}

/******************************************************************************
SensorApiService - implementation - :nearByBatchCount to check nearby batch
count, return batch count which is multiplication of min batch count defined for
each sensor in /etc/sensors.conf file and should be less than or equal to
ReqBatchCount
******************************************************************************/
int SensorApiService::nearByBatchCount(int minBatchCount, int ReqBatchCount, float input_rate, float output_rate, int factor) {
   int count = 0;
   int supported_batches[MAX_BATCH_COUNT] = {0};

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
SensorApiService - implementation - nearBySamplingRate
Find out near by Nearby sampling rate which client requested
******************************************************************************/
float SensorApiService::nearBySamplingRate(float input_rates[], float target_rate) {
   float nearest_rate = input_rates[0];
   float min_diff = fabs(input_rates[0] - target_rate); // Caluclate minimum differece

   for (int i = 0; i < MAX_ODR; i++) {
	   if (input_rates[i] == 0) {
		   continue; // Skip zero values
	   }
	   float diff = fabs(input_rates[i] - target_rate); // Calculate actual difference
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

#ifdef POWERMANAGER_ENABLED
/******************************************************************************
SensorApiService - power event handlers
******************************************************************************/
void SensorApiService::onPowerEvent(PowerStateType powerState, SensorCapabilitiesMask mask) {
    lock_guard<mutex> lock(mMutex);
    bool rc = false;
    bool onDemand = false;
    int64_t difference = 0;
    static int64_t selttest_time_delta = 0;

    SENSOR_LOGI(LOG_TAG "--< onPowerEvent %d", powerState);
    mPowerState = powerState;

    switch(mPowerState) {
	case POWER_STATE_SUSPEND:
             POWER_STATE_SHUTDOWN: {
		 for(int i = 0 ; i < mSensorCount; i++)  {
		     SENSOR_LOGI(LOG_TAG ">-- on Suspend/Shutdown Disable the sensor mSensor[i].sensor_id %d\n",
				     mSensor[i].sensor_id);
		     sensorActivate(mSensor[i].sensor_id, SENSOR_DISABLE);
		 }
		 break;
	}
	case POWER_STATE_RESUME: {
		 for(int i = 0 ; i < mSensorCount; i++)  {
	            if (mSensor[i].Activate == SENSOR_ENABLE){
			int64_t SamplingRate = FREQUENCY_TO_NS(mSensor[i].SamplingRate);
			int64_t BatchingRate =  mSensor[i].BatchCount *  SamplingRate  * mBatchConst;
			SENSOR_LOGI(LOG_TAG ">-- on Resume Re-Configure sensor sensor_id %d \
					sampling Rate %lld BatchingRate %lld\n", 
					mSensor[i].sensor_id, SamplingRate, BatchingRate);
			(void)sensorSetBatch(mSensor[i].sensor_id, SamplingRate, BatchingRate); 
			(void)sensorActivate(mSensor[i].sensor_id, SENSOR_ENABLE); 
		    }
		    //send self-test result if failed
		    if (mSensor[i].selfTest == SelfTestResult::Failed
				    && (mSensor[i].type == SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED))
			    mask |= ACCEL_SELFTEST_FAIL;
		    if (mSensor[i].selfTest  == SelfTestResult::Failed
				    && (mSensor[i].type == SENSOR_TYPE_GYROSCOPE_UNCALIBRATED))
			    mask |= GYRO_SELFTEST_FAIL;
		 }
		 break;
	}
    }

    //send events to all clients
    for (auto it = mClients.begin(); it != mClients.end();) {
        if (it->second != nullptr) {
            rc = it->second->onCapabilitiesCallback(mask);
            if(!rc) {
                SENSOR_LOGE(LOG_TAG "failed rc=%d purging client=%s\n", rc, it->first.c_str());
                it = deleteClientbyName(it->first.c_str());
	    }
	    else
		    ++it;
	}
	else
		++it;
    }
    return;
}
#endif

#ifdef SENSOR_HEAD_TYPE_SUPPORT
void SensorApiService::enableHeadingSensor() {
   SENSOR_LOGI(LOG_TAG "<<< Enable HEADING SENSOR \n");
    // create Location client API
    if (pLcaClient == nullptr)
	    pLcaClient = new LocationClientApi(onLocationCapabilitiesCb);
    if (pLcaClient == nullptr) {
	    SENSOR_LOGE(LOG_TAG "failed to create location client, return\n");
	    return;
    }
    SENSOR_LOGI(LOG_TAG "<<< start heading sensor session\n");
    GnssReportCbs reportcbs = {};
    reportcbs.gnssLocationCallback = GnssLocationCb(onGnssLocationCb);
    (void)pLcaClient->startPositionSession(100, reportcbs, onLocationResponseCb);
}

void SensorApiService::disableHeadingSensor() {
   SENSOR_LOGI(LOG_TAG "<<< Disable HEADING SENSOR \n");
   if (pLcaClient != nullptr) {
	   SENSOR_LOGI(LOG_TAG "<<< stop heading sensor session\n");
	   pLcaClient->stopPositionSession();
   }
}

/********************************************************************************
 * Callback functions
 * ******************************************************************************/
static void SensorApiService::onLocationCapabilitiesCb(location_client::LocationCapabilitiesMask mask) {
   SENSOR_LOGI(LOG_TAG "<<< Location onCapabilitiesCb mask=%d\n", mask);
}

static void SensorApiService::onLocationResponseCb(location_client::LocationResponse response) {
   SENSOR_LOGI(LOG_TAG "<<< Location onResponseCb err=%u\n", response);
}

void SensorApiService::onSensorHeadingDataReadCb(float heading, float accuracy, uint64_t ts) {
   lock_guard<mutex> lock(mMutex);
   float heading_degree = heading * (180.0f/M_PI); // Calculate Heading Degree
   float accuracy_degree = accuracy * (180.0f/M_PI); // Calculate Accuracy Degree
#ifdef SENSOR_IVSS_ENABLED
   myService->onSensorHeadingDataReadCb(heading_degree, accuracy_degree, ts);
#endif
}

static void SensorApiService::onGnssLocationCb(const location_client::GnssLocation& location) {
   SENSOR_LOGI(LOG_TAG "<<< Location yaw <%f %f> ts %lld\n", location.bodyFrameData.yaw, location.bodyFrameData.yawUnc, location.elapsedRealTimeNs);
   mInstance->onSensorHeadingDataReadCb(location.bodyFrameData.yaw, location.bodyFrameData.yawUnc, location.elapsedRealTimeNs);
}
#endif
