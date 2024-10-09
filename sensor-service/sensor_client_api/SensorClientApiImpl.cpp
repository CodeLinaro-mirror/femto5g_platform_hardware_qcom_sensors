/* Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
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
 *     * Neither the name of The Linux Foundation nor the names of its
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

#include <sys/types.h>
#include <unistd.h>
#include <SensorClientApiImpl.h>
#include <unistd.h>
#include <sstream>
#include <dlfcn.h>
#include <string.h>

namespace sensor_client {

/******************************************************************************
SensorClientImpl
******************************************************************************/
uint32_t  SensorClientImpl::mClientIdGenerator = SENSOR_CLIENT_SESSION_ID_INVALID;
mutex SensorClientImpl::mMutex;

/******************************************************************************
SensorClientImpl - constructors
******************************************************************************/
SensorClientImpl::SensorClientImpl(CapabilitiesCb capabitiescb) :
        mHalRegistered(false),
        mEapClient(false),
	mCapabilitiesCb(capabitiescb),
        mBatchingCb(nullptr),
        mSensorTempReadCb(nullptr),
        mSensorBufferDataReadCb(nullptr),
	mSensorList(nullptr),
	mSensorMlcCaseList(nullptr),
	mSensorTrackingOption(nullptr),
        mSensorMLCEventCbs(nullptr),
	mSensormFifoReadCb(nullptr),
	mSelfTestResultCb(nullptr),
	mSensorCount(0),
	mShdRestarted(false),
	mSensorMlcCaseCount(0)
{
    // get clientId
    uint32_t pid = (uint32_t)getpid();

#ifdef FEATURE_EXTERNAL_AP
    lock_guard<mutex> lock(mMutex);
    int service = SENSOR_CLIENT_API_QSOCKET_HALDAEMON_SERVICE_ID;
    // generate instance from pid and client id
    mClientId = ++mClientIdGenerator;
    int instance = pid * 100 + mClientId;
    int numChars = snprintf(mSocketName, sizeof(mSocketName), "%u.%u",
                            SENSOR_CLIENT_API_QSOCKET_CLIENT_SERVICE_ID,
                            instance);
    if (numChars >= (sizeof(mSocketName)-1)) {
        SENSOR_LOGE(LOG_TAG "mSocketName to small, need %d, buffer size %d",
                 numChars, sizeof(mSocketName));
	return;
    }

    // establish an ipc sender to the hal daemon
    mIpcSender = new SensorQsocketSender(SENSOR_CLIENT_API_QSOCKET_HALDAEMON_SERVICE_ID,
                                     SENSOR_CLIENT_API_QSOCKET_HALDAEMON_INSTANCE_ID);
    if (nullptr == mIpcSender) {
        SENSOR_LOGE(LOG_TAG "create Qsocket failed addr=%u:%u\n", service, instance);
        return;
    }
    SENSOR_LOGI(LOG_TAG "mClientId %d listen on socket: %s\n", mClientId, mSocketName);
    startListeningNonBlocking(mSocketName, SENSOR_CLIENT_API_QSOCKET_HALDAEMON_SERVICE_ID);
#else
    // create ipc socket to send
    mIpcSender = new SensorIpcSender(SOCKET_TO_SENSOR_HAL_DAEMON);
    if (nullptr == mIpcSender) {
        SENSOR_LOGE(LOG_TAG "create mIpcSender failed %s\n", SOCKET_TO_SENSOR_HAL_DAEMON);
        return;
    }

    // get clientId
    lock_guard<mutex> lock(mMutex);
    mClientId = ++mClientIdGenerator;
    int strCopied = strlcpy(mSocketName,SOCKET_TO_SENSOR_CLIENT_BASE,
                           MAX_SOCKET_PATHNAME_LENGTH);
    if (strCopied>0 && strCopied< MAX_SOCKET_PATHNAME_LENGTH) {
        snprintf(mSocketName+strCopied,
                 MAX_SOCKET_PATHNAME_LENGTH-strCopied,
                 ".%u.%u", pid, mClientId);
    } else {
        SENSOR_LOGE(LOG_TAG "strlcpy failed %d\n", strCopied);
        return;
    }
    SENSOR_LOGI(LOG_TAG "mClientId %d listen on socket: %s\n", mClientId, mSocketName);
    startListeningNonBlocking(mSocketName);
#endif

    pthread_mutex_init(&mSensorLibMutex, NULL);
    pthread_condattr_init(&mSensorLibattr);
    pthread_condattr_setclock(&mSensorLibattr, CLOCK_MONOTONIC);
    pthread_cond_init(&mSensorLibCond, &mSensorLibattr);

}

/******************************************************************************
SensorClientImpl - Distructor
******************************************************************************/
SensorClientImpl::~SensorClientImpl() {
}

void SensorClientImpl::destroy() {
    stopListening();
    if (mHalRegistered && (nullptr != mIpcSender)) {
	//Send Client Deregister Message Id to hal daemon
	SENSOR_LOGI(LOG_TAG "Send client De-Register message %s\n", mSocketName);
        SensorAPIClientDeregisterReqMsg msg(mSocketName);
	bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
	delete mIpcSender;
	mIpcSender = nullptr;
    }
    if (mSensorList) {
        delete mSensorList;
        mSensorList = nullptr;
    }
    if (mSensorMlcCaseList) {
        delete mSensorMlcCaseList;
	mSensorMlcCaseList = nullptr;
    }
    pthread_mutex_destroy(&mSensorLibMutex);
    pthread_condattr_destroy(&mSensorLibattr);
    pthread_cond_destroy(&mSensorLibCond);
}

/******************************************************************************
SensorClientImpl - GetSensorList
******************************************************************************/
int SensorClientImpl::getSensorList(struct sensor_list ***s, int *sensor_count) {

    SENSOR_LOGI(LOG_TAG ">>> GetSensorList \n");

    lock_guard<mutex> lock(mMutex);

    //Check about Client registered to daemon
    if (!mHalRegistered) {
	    SENSOR_LOGE(LOG_TAG ">>> getSensorList - Not registered yet\n");
	    return SENSOR_ERROR_CLIENT_REGISTER_FAILED;
    }

    **s = mSensorList;
    *sensor_count = mSensorCount;

    if(mSensorCount == 0)
	    return SENSOR_ERROR_NO_SENSORS_FOUND;

    else
	    return SENSOR_RESPONSE_SUCCESS;
}

/******************************************************************************
SensorClientImpl - SensorControl
******************************************************************************/
int SensorClientImpl::sensorControl(int sensor_id, sensor_state state) {

    SENSOR_LOGI(LOG_TAG ">>> sensorControl sensor_id %d sensor state %d\n", sensor_id, state);
    int ret = 0;
    bool SensorId = false;

    lock_guard<mutex> lock(mMutex);

   //Check about Client registered to daemon
    if (!mHalRegistered) {
	    SENSOR_LOGE(LOG_TAG ">>> sensorControl - Not registered yet\n");
	    return SENSOR_ERROR_CLIENT_REGISTER_FAILED;
    }

    //Input parameter check
    if (state < SENSOR_DISABLE ||  state > SENSOR_HPM)
	    return SENSOR_ERROR_INVALID_INPUT_PARAMETER;

    if (mSensorCount != 0) {
      for (int i=0; i < mSensorCount; i++) {
	 if (mSensorList[i].sensor_id == sensor_id) {
	    SensorId = true;
	    if (state == SENSOR_ENABLE || state == SENSOR_DISABLE)
		    mSensorTrackingOption[i].state = state;
	    break;
	 }
      }
      if (SensorId != true ) {
         return SENSOR_ERROR_INVALID_INPUT_PARAMETER;
      }
    }
    else
      return SENSOR_ERROR_NO_SENSORS_FOUND;

    if (SENSOR_CLIENT_SESSION_ID_INVALID != mClientId) {
      pthread_mutex_lock (&mSensorLibMutex);
      //Enable/Disable the sensor
      SensorAPIEnableReqMsg msg (mSocketName, sensor_id, state);
      bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
	      sizeof(msg));
      if (true != rc) {
	 pthread_mutex_unlock (&mSensorLibMutex);
	 return SENSOR_ERROR_IPC_FAILED;
      }
      mTimeout = timeout(3);
      ret = pthread_cond_timedwait(&mSensorLibCond, &mSensorLibMutex, &mTimeout);
      pthread_mutex_unlock (&mSensorLibMutex);
      if (ret == ETIMEDOUT)
	      return SENSOR_ERROR_NO_RESPONSE_FROM_SHD_TIMEOUT;
      else
	      return mRespReturn;
    }
    else
	return SENSOR_ERROR_INVALID_CLIENT;
}

/******************************************************************************
SensorClientImpl - StartBatching
******************************************************************************/
int SensorClientImpl::startBatching(int sensor_id, float sampling_rate, int batch_count, bool rotate, BatchingCb batchingCallback) {
    SENSOR_LOGI(LOG_TAG ">>> sensorBatching sensor_id %d sampling_rate %f batch_count %d rotate %d\n",
		    sensor_id, sampling_rate, batch_count, rotate);

    int ret = 0;
    bool SensorId = false;

    lock_guard<mutex> lock(mMutex);

    //Check about Client registered to daemon
    if (!mHalRegistered) {
            SENSOR_LOGE(LOG_TAG ">>> startBatching - Not registered yet\n");
	    return SENSOR_ERROR_CLIENT_REGISTER_FAILED;
    }

    mBatchingCb = batchingCallback;

    //Input parameter check
    if (mSensorCount != 0) {
      for (int i=0; i < mSensorCount; i++) {
         if (mSensorList[i].sensor_id == sensor_id) {
            SensorId = true;
	    if (batch_count <= 0 || sampling_rate <=0)
		    return SENSOR_ERROR_INVALID_INPUT_PARAMETER;
	    mSensorTrackingOption[i].sampling_rate = sampling_rate;
	    mSensorTrackingOption[i].batch_count = batch_count;
	    mSensorTrackingOption[i].rotate = rotate;
            break;
         }
      }
      if (SensorId != true )
         return SENSOR_ERROR_INVALID_INPUT_PARAMETER;
    }
    else
      return SENSOR_ERROR_NO_SENSORS_FOUND;

    if (SENSOR_CLIENT_SESSION_ID_INVALID != mClientId) {
      pthread_mutex_lock (&mSensorLibMutex);
      SensorAPIStartBatchingReqMsg msg (mSocketName,
	      sensor_id, sampling_rate, batch_count, rotate);
      bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
	      sizeof(msg));
      if (true != rc) {
	 pthread_mutex_unlock (&mSensorLibMutex);
	 return SENSOR_ERROR_IPC_FAILED;
      }
      mTimeout = timeout(3);
      ret = pthread_cond_timedwait(&mSensorLibCond, &mSensorLibMutex, &mTimeout);
      pthread_mutex_unlock (&mSensorLibMutex);
      if (ret == ETIMEDOUT)
	      return SENSOR_ERROR_NO_RESPONSE_FROM_SHD_TIMEOUT;
      else
	      return mRespReturn;
    }
    else
	return SENSOR_ERROR_INVALID_CLIENT;
}

/******************************************************************************
SensorClientImpl - StartTracking
******************************************************************************/
int SensorClientImpl::startTracking(int sensor_id, SensorDataReadCb sensorreadCallback) {
    SENSOR_LOGI(LOG_TAG ">>> sensorTracking sensor_id %d\n", sensor_id);

    int ret = 0;
    bool SensorId = false;

    lock_guard<mutex> lock(mMutex);

    //Input parameter check
    if (mSensorCount != 0) {
      for (int i=0; i < mSensorCount; i++) {
         if (mSensorList[i].sensor_id == sensor_id) {
	    SensorId = true;
	    mSensorTrackingOption[i].mSensorDataReadCb = sensorreadCallback;
	    break;
         }
      }
      if (SensorId != true )
         return SENSOR_ERROR_INVALID_INPUT_PARAMETER;
    }
    else
      return SENSOR_ERROR_NO_SENSORS_FOUND;

    //Check about Client registered to daemon
    if (!mHalRegistered) {
	    return SENSOR_ERROR_CLIENT_REGISTER_FAILED;
    }

    if (SENSOR_CLIENT_SESSION_ID_INVALID != mClientId) {
      pthread_mutex_lock (&mSensorLibMutex);
      SensorAPIStartTrackingReqMsg msg(mSocketName);
      bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
	     sizeof(msg));
      if (true != rc) {
	 pthread_mutex_unlock (&mSensorLibMutex);
         return SENSOR_ERROR_IPC_FAILED;
      }
      mTimeout = timeout(3);
      ret = pthread_cond_timedwait(&mSensorLibCond, &mSensorLibMutex, &mTimeout);
      pthread_mutex_unlock (&mSensorLibMutex);
      if (ret == ETIMEDOUT)
	      return SENSOR_ERROR_NO_RESPONSE_FROM_SHD_TIMEOUT;
      else
	      return mRespReturn;
    }
    else
	return SENSOR_ERROR_INVALID_CLIENT;
}
/******************************************************************************
SensorClientImpl - sensorRequestMLC
******************************************************************************/
int SensorClientImpl::sensorRequestMLC(struct sensor_mlc_case_list ***m, int *mlc_case_count) {

    SENSOR_LOGI(LOG_TAG ">>> sensorRequestMLC \n");

    lock_guard<mutex> lock(mMutex);

    //Check about Client registered to daemon
    if (!mHalRegistered) {
	    SENSOR_LOGE(LOG_TAG ">>> sensorRequestMLC - Not registered yet\n");
	    return SENSOR_ERROR_CLIENT_REGISTER_FAILED;
    }

    **m = mSensorMlcCaseList;
    *mlc_case_count = mSensorMlcCaseCount;

    if(mSensorMlcCaseCount == 0)
            return SENSOR_ERROR_NO_MLC_CASE_FOUND;

    else
            return SENSOR_RESPONSE_SUCCESS;
}

/******************************************************************************
SensorClientImpl - SensorMLCEventEnable
******************************************************************************/
int SensorClientImpl::sensorMLCEventEnable(char *mlc_case_name, bool enable,
				SensorMLCEventCb sensorMlcEventCallback,
				SensormFifoReadCb sensorMfifoReadCallback) {
    bool mlc_case = false;
    char case_name[100];
    int ret = 0;

    SENSOR_LOGI(LOG_TAG ">>> SensorMLCEventEnable name %s enable %d \n", mlc_case_name, enable);

    lock_guard<mutex> lock(mMutex);

    //Check about Client registered to daemon
    if (!mHalRegistered) {
            SENSOR_LOGE(LOG_TAG ">>> sensorMLCEventEnable - Not registered yet\n");
            return SENSOR_ERROR_CLIENT_REGISTER_FAILED;
    }

    //Input parameter check
    if (enable != 0 &&  enable != 1)
            return SENSOR_ERROR_INVALID_INPUT_PARAMETER;

    mSensormFifoReadCb = sensorMfifoReadCallback;

    if (mSensorMlcCaseCount != 0) {
      for (int i=0; i < mSensorMlcCaseCount; i++) {
	    if (strcmp(mSensorMlcCaseList[i].name, mlc_case_name) == 0) {
		    mlc_case = true;
		    mSensorMLCEventCbs[i].mSensorMLCEventCb = sensorMlcEventCallback;
		    mSensorMLCEventCbs[i].enable = enable;
		    break;
	    }
      }
      if (mlc_case != true ) {
         return SENSOR_ERROR_INVALID_INPUT_PARAMETER;
      }
    }
    else
      return SENSOR_ERROR_NO_MLC_CASE_FOUND;

    if (SENSOR_CLIENT_SESSION_ID_INVALID != mClientId) {
      pthread_mutex_lock (&mSensorLibMutex);
      //Enable/Disable the Mlc case event
      strlcpy(case_name, mlc_case_name, 100);
      SensorAPIMLCCaseEnableMsg msg (mSocketName, case_name, enable);
      bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
              sizeof(msg));
      if (true != rc) {
         pthread_mutex_unlock (&mSensorLibMutex);
         return SENSOR_ERROR_IPC_FAILED;
      }
      mTimeout = timeout(3);
      ret = pthread_cond_timedwait(&mSensorLibCond, &mSensorLibMutex, &mTimeout);
      pthread_mutex_unlock (&mSensorLibMutex);
      if (ret == ETIMEDOUT)
	      return SENSOR_ERROR_NO_RESPONSE_FROM_SHD_TIMEOUT;
      else
	      return mRespReturn;
    }
    else
	    return SENSOR_ERROR_INVALID_CLIENT;
}

/******************************************************************************
SensorClientImpl - ReadTemperature
******************************************************************************/
int SensorClientImpl::readTemperature(SensorTempReadCb sensortempreadCallback) {

    SENSOR_LOGD(LOG_TAG ">>> sensorReadTemp\n");

    lock_guard<mutex> lock(mMutex);

    mSensorTempReadCb = sensortempreadCallback;

    //Check about Client registered to daemon
    if (!mHalRegistered) {
	    return SENSOR_ERROR_CLIENT_REGISTER_FAILED;
    }

    if (SENSOR_CLIENT_SESSION_ID_INVALID != mClientId) {
	SensorAPITempReqMsg msg(mSocketName);
	bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
			sizeof(msg));
	if (true == rc) {
		return SENSOR_RESPONSE_SUCCESS;
	}
	else {
		return SENSOR_ERROR_IPC_FAILED;
	}
    }
    else
	return SENSOR_ERROR_INVALID_CLIENT;
}

/******************************************************************************
SensorClientImpl - StartBufferDataRead
******************************************************************************/
int SensorClientImpl::startBufferDataRead(bool enable, SensorBufferDataReadCb sensorbufferreadCallback) {

    SENSOR_LOGI(LOG_TAG ">>> sensorBufferRead enbale %d\n", enable);
    int ret = 0;

    lock_guard<mutex> lock(mMutex);

    mSensorBufferDataReadCb = sensorbufferreadCallback;

    //Check about Client registered to daemon
    if (!mHalRegistered) {
	    return SENSOR_ERROR_CLIENT_REGISTER_FAILED;
    }

    //Input parameter check
    if (enable != 1 && enable != 0)
	    return SENSOR_ERROR_INVALID_INPUT_PARAMETER;

    if (SENSOR_CLIENT_SESSION_ID_INVALID != mClientId) {
      pthread_mutex_lock (&mSensorLibMutex);
      SensorAPIBufferDataReqMsg msg(mSocketName, enable);
      bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
		      sizeof(msg));
      if (true != rc) {
              pthread_mutex_unlock (&mSensorLibMutex);
	      return SENSOR_ERROR_IPC_FAILED;
      }
      mTimeout = timeout(3);
      ret = pthread_cond_timedwait(&mSensorLibCond, &mSensorLibMutex, &mTimeout);
      pthread_mutex_unlock (&mSensorLibMutex);
      if (ret == ETIMEDOUT)
	      return SENSOR_ERROR_NO_RESPONSE_FROM_SHD_TIMEOUT;
      else
	      return mRespReturn;
    }
    else
	return SENSOR_ERROR_INVALID_CLIENT;
}

/******************************************************************************
SensorClientImpl - SelfTest
******************************************************************************/
int SensorClientImpl::selfTest(int sensor_id, SelfTestType selfTestType, int request_id, SelfTestResultCallback selftestResultCallback) {
    SENSOR_LOGI(LOG_TAG ">>> sensorSelfTest sensor_id %d SelfTestType %d request_id = %d\n", sensor_id, selfTestType, request_id);

    int ret = 0;
    bool SensorId = false;

    lock_guard<mutex> lock(mMutex);

    //Check about Client registered to daemon
    if (!mHalRegistered) {
            SENSOR_LOGE(LOG_TAG ">>> startBatching - Not registered yet\n");
            return SENSOR_ERROR_CLIENT_REGISTER_FAILED;
    }

    mSelfTestResultCb = selftestResultCallback;

    //Input parameter check
    if (mSensorCount != 0) {
      for (int i=0; i < mSensorCount; i++) {
         if (mSensorList[i].sensor_id == sensor_id) {
            SensorId = true;
	    if (selfTestType != Positive && selfTestType != Negative){
		    return SENSOR_ERROR_INVALID_INPUT_PARAMETER;
		}
            break;
         }
      }
      if (SensorId != true ){
         return SENSOR_ERROR_INVALID_INPUT_PARAMETER;
	}
    }
    else{
      return SENSOR_ERROR_NO_SENSORS_FOUND;
	}

    if (SENSOR_CLIENT_SESSION_ID_INVALID != mClientId) {
      pthread_mutex_lock (&mSensorLibMutex);
      SensorAPISelfTestReqMsg msg (mSocketName,
              sensor_id, selfTestType, request_id);
      bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
              sizeof(msg));
      if (true != rc) {
         pthread_mutex_unlock (&mSensorLibMutex);
         return SENSOR_ERROR_IPC_FAILED;
      }
      mTimeout = timeout(3);
      ret = pthread_cond_timedwait(&mSensorLibCond, &mSensorLibMutex, &mTimeout);
      pthread_mutex_unlock (&mSensorLibMutex);
      if (ret == ETIMEDOUT){
              return SENSOR_ERROR_NO_RESPONSE_FROM_SHD_TIMEOUT;
	}
      else{
              return mRespReturn;
	}
    }
    else{
        return SENSOR_ERROR_INVALID_CLIENT;
	}
}

/******************************************************************************************************
SensorClientImpl - setEulerAngles
******************************************************************************************************/
int SensorClientImpl::setEulerAngles(uint16_t rolld, uint16_t pitchd, uint16_t yawd) {
    SENSOR_LOGI(LOG_TAG ">>> setEulerAngles roll %d  pitch %d yaw %d\n", rolld, pitchd, yawd);

    int ret = 0;
    lock_guard<mutex> lock(mMutex);

    //Check about Client registered to daemon
    if (!mHalRegistered) {
            SENSOR_LOGE(LOG_TAG ">>> startBatching - Not registered yet\n");
            return SENSOR_ERROR_CLIENT_REGISTER_FAILED;
    }

    //Input parameter check
    if(yawd < RM_MIN || pitchd < RM_MIN || rolld < RM_MIN || yawd > RM_MAX || pitchd > RM_MAX || rolld > RM_MAX)
	    return SENSOR_ERROR_INVALID_INPUT_PARAMETER;

    if (SENSOR_CLIENT_SESSION_ID_INVALID != mClientId) {
	    pthread_mutex_lock (&mSensorLibMutex);
	    SensorAPIEulerAnglesReqMsg msg(mSocketName, rolld, pitchd, yawd);
	    bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
			    sizeof(msg));
	    if (true != rc) {
		    pthread_mutex_unlock (&mSensorLibMutex);
		    return SENSOR_ERROR_IPC_FAILED;
	    }
	    mTimeout = timeout(3);
	    ret = pthread_cond_timedwait(&mSensorLibCond, &mSensorLibMutex, &mTimeout);
	    pthread_mutex_unlock (&mSensorLibMutex);
	    if (ret == ETIMEDOUT)
		    return SENSOR_ERROR_NO_RESPONSE_FROM_SHD_TIMEOUT;
	    else
		    return mRespReturn;
    }
    else
	    return SENSOR_ERROR_INVALID_CLIENT;
}

/******************************************************************************
SensorClientImpl - SensorReconfigure Enable
******************************************************************************/
bool SensorClientImpl::SensorReconfigure(bool enable) {
  bool rc = 0;

  if (mClientId != SENSOR_CLIENT_SESSION_ID_INVALID) {
     if (enable) {
	for (int i = 0 ; i < mSensorCount ; i++) {
		if (mSensorTrackingOption[i].state == SENSOR_ENABLE) {
			SensorAPIStartTrackingReqMsg Trackingmsg(mSocketName);
			rc = sendMessage(reinterpret_cast<uint8_t*>(&Trackingmsg),
					sizeof(Trackingmsg));
			SensorAPIStartBatchingReqMsg Configmsg (mSocketName,
					mSensorTrackingOption[i].sensor_id,
					mSensorTrackingOption[i].sampling_rate,
					mSensorTrackingOption[i].batch_count,
					mSensorTrackingOption[i].rotate);
			rc = sendMessage(reinterpret_cast<uint8_t*>(&Configmsg),
					sizeof(Configmsg));
			SensorAPIEnableReqMsg Enablemsg (mSocketName, mSensorTrackingOption[i].sensor_id,
					mSensorTrackingOption[i].state);
			rc = sendMessage(reinterpret_cast<uint8_t*>(&Enablemsg),
					sizeof(Enablemsg));
		}
	}
	for (int i = 0 ; i < mSensorMlcCaseCount ; i++) {
		if (mSensorMLCEventCbs[i].enable) {
			SensorAPIMLCCaseEnableMsg Mlcmsg (mSocketName, mSensorMLCEventCbs[i].name,
					mSensorMLCEventCbs[i].enable);
			rc = sendMessage(reinterpret_cast<uint8_t*>(&Mlcmsg),
					sizeof(Mlcmsg));
		}
	}
     }
   }
   return rc;
}

/******************************************************************************
  SensorClientImpl - onListenerReady
*******************************************************************************/
void SensorClientImpl::onListenerReady() {
    SENSOR_LOGI(LOG_TAG "<<< onListenerReady\n");
#ifndef FEATURE_EXTERNAL_AP
    if (0 != chown(mSocketName, getuid(), GID_SENSORCLIENT)) {
	    SENSOR_LOGE(LOG_TAG "chown to group sensor client failed %s", strerror(errno));
    }
#endif
    //Send Client Register Message Id to Daemon if success,
    //set mHalRegistered to true
    if (!mHalRegistered) {
      SENSOR_LOGI(LOG_TAG "<<< Sending Client Register message %s\n", mSocketName);
      SensorAPIClientRegisterReqMsg msg(mSocketName, SENSOR_CLIENT_API);
      bool rc = sendMessage(reinterpret_cast<uint8_t *>(&msg), sizeof(msg));
      if(true != rc && mCapabilitiesCb) {
	      mCapabilitiesCb(SHD_NOT_RUNNING);
	      mEapClient  = true;
      }
    }
}

#ifdef FEATURE_EXTERNAL_AP
/******************************************************************************
  SensorClientImpl - onServiceStatusChange
*******************************************************************************/
void SensorClientImpl::onServiceStatusChange(int serviceId, int instanceId, int status,
		const SensorQsocketSender& sender) {
    if (status == 1) {
	SENSOR_LOGI(LOG_TAG "Sensor HAL Daemon ServiceStatus::UP serviceId %d instanceId %d\n",
			serviceId, instanceId);
	if (SENSOR_CLIENT_API_QSOCKET_HALDAEMON_SERVICE_ID == serviceId &&
			SENSOR_CLIENT_API_QSOCKET_HALDAEMON_INSTANCE_ID == instanceId) {
		if (mIpcSender->copyDestAddrFrom(sender)) {
			if (mEapClient) {
			    sleep(2);
			    SensorAPIHalReadyIndMsg pMsg(SERVICE_NAME);
			    string msg;
			    msg.resize(sizeof(pMsg));
			    memcpy(msg.data(), reinterpret_cast<uint8_t *>(&pMsg), sizeof(pMsg));
			    onReceive(msg);
			    mEapClient = true;
			}
		}
	}
    }
}
#endif

/******************************************************************************
 SensorClientImpl - Process Message Receive from Daemon
******************************************************************************/
void SensorClientImpl::onReceive(const string& data) {

   SensorAPIMsgHeader *pMsg = (SensorAPIMsgHeader *)(data.data());
   SENSOR_LOGD(LOG_TAG  "pMsg->msgId: %d", pMsg->msgId);

   uint32_t length = data.length();
    // throw away message that does not come from sensor hal daemon
   if (false == pMsg->isValidServerMsg(length)) {
	   return;
   }
   //Check for MSG ID
   switch (pMsg->msgId) {
       //Received hal capability message from SHD(SENSOR HAL DAEMON)
       case E_SENSORAPI_CAPABILILTIES_MSG_ID:
       {
                if (sizeof(SensorAPICapabilitiesIndMsg) != length) {
                    SENSOR_LOGE(LOG_TAG  "payload size does not match for message with id: %d",
                             pMsg->msgId);
                }
		SensorAPICapabilitiesIndMsg* pCapIndMsg = (SensorAPICapabilitiesIndMsg*)(pMsg);
                SENSOR_LOGI(LOG_TAG "<<< capabilities indication mask %d", pCapIndMsg->mask);
		mHalRegistered = true;
		SensorReconfigure(mShdRestarted);
		mShdRestarted = false;
		mEapClient  = true;
		if (mCapabilitiesCb)
			mCapabilitiesCb(pCapIndMsg->mask);
                break;
       }
       //Received hal ready message from SHD(SENSOR HAL DAEMON)
       case E_SENSORAPI_HAL_READY_MSG_ID:
       {
          SENSOR_LOGI(LOG_TAG "<<< HAL ready\n");
	  if (sizeof(SensorAPIHalReadyIndMsg) != length) {
		  SENSOR_LOGE(LOG_TAG "payload size does not match for message with id: %d\n",
				  pMsg->msgId);
	  }

	  // sensor hal deamon has restarted, need to set this
	  // flag to false to prevent messages to be sent to hal
	  // before registeration completes
	  mHalRegistered = false;
	  if (mCapabilitiesCb)
		  mCapabilitiesCb(SHD_RESTARTED);
	  mShdRestarted = true;
	  onListenerReady();
	  break;
       }
       //Received Sensor List from SHD(SENSOR HAL DAEMON)
       case E_SENSORAPI_SENSOR_LIST_MSG_ID:
       {
           if (mClientId != SENSOR_CLIENT_SESSION_ID_INVALID) {
		   SensorAPIListIndMsg* pListIndMsg = (SensorAPIListIndMsg*)(pMsg);
		   mSensorCount = pListIndMsg->sensorList.count;
		   if (mSensorList) {
		      delete mSensorList;
		      mSensorList = nullptr;
		   }
		   mSensorList = new (std::nothrow) struct sensor_list[mSensorCount];
		   if (mSensorList == nullptr) {
			return;
		   }

		   memcpy(mSensorList, &pListIndMsg->sensorList.s[0], sizeof(struct sensor_list) * mSensorCount);

		   if (mSensorTrackingOption == nullptr) {
		     mSensorTrackingOption = new (std::nothrow) struct SensorTrackingOption[mSensorCount];
		     if (mSensorTrackingOption == nullptr) {
			return;
		     }

		     for (int i = 0 ; i < mSensorCount ; i++) {
			     mSensorTrackingOption[i].sensor_id = mSensorList[i].sensor_id;
			     mSensorTrackingOption[i].mSensorDataReadCb = nullptr;
		     }
		   }
           }
           break;
       }
       //Received Sensor MLC Case List from SHD(SENSOR HAL DAEMON)
       case E_SENSORAPI_SENSOR_MLC_CASE_LIST_MSG_ID:
       {
           if (mClientId != SENSOR_CLIENT_SESSION_ID_INVALID) {
                   SensorMlcCaseListIndMsg* pListIndMsg = (SensorMlcCaseListIndMsg*)(pMsg);
                   mSensorMlcCaseCount = pListIndMsg->sensorMlcCaseList.count;
                   if (mSensorMlcCaseList) {
                      delete mSensorMlcCaseList;
                      mSensorMlcCaseList = nullptr;
                   }
                   mSensorMlcCaseList = new (std::nothrow) struct sensor_mlc_case_list[mSensorMlcCaseCount];
		   if (mSensorMlcCaseList == nullptr) {
			return;
		   }

                   memcpy(mSensorMlcCaseList, &pListIndMsg->sensorMlcCaseList.s[0],
				   sizeof(struct sensor_mlc_case_list) * mSensorMlcCaseCount);

		   if (mSensorMLCEventCbs == nullptr) {
                     mSensorMLCEventCbs = new (std::nothrow) struct MlcCaseListCb[mSensorMlcCaseCount];
		     for (int i = 0 ; i < mSensorMlcCaseCount ; i++) {
			     strlcpy(mSensorMLCEventCbs[i].name, mSensorMlcCaseList[i].name, 100);
			     mSensorMLCEventCbs[i].mSensorMLCEventCb = nullptr;
		     }
		   }
	   }
	   break;
       }
       //Received Sensor Enable/Disable Resp from SHD(SENSOR HAL DAEMON)
       case E_SENSORAPI_SENSOR_ENABLE_MSG_ID:
       {
	   if (sizeof(SensorAPIGenericRespMsg) != length) {
                   SENSOR_LOGE(LOG_TAG "payload size does not match for message with id: %d\n",
                                   pMsg->msgId);
           }

	   const SensorAPIGenericRespMsg* pRespMsg = (SensorAPIGenericRespMsg*)(pMsg);
	   pthread_mutex_lock (&mSensorLibMutex);
           mRespReturn = pRespMsg->ret;
	   pthread_cond_signal (&mSensorLibCond);
	   pthread_mutex_unlock (&mSensorLibMutex);
           break;
       }
       //Received Sensor MLC case enable/disable Resp from SHD(SENSOR HAL DAEMON)
       case E_SENSORAPI_SENSOR_MLC_CASE_ENABLE_MSG_ID:
       {
           if (sizeof(SensorAPIGenericRespMsg) != length) {
                   SENSOR_LOGE(LOG_TAG "payload size does not match for message with id: %d\n",
                                   pMsg->msgId);
           }

           const SensorAPIGenericRespMsg* pRespMsg = (SensorAPIGenericRespMsg*)(pMsg);
	   pthread_mutex_lock (&mSensorLibMutex);
           mRespReturn = pRespMsg->ret;
	   pthread_cond_signal (&mSensorLibCond);
	   pthread_mutex_unlock (&mSensorLibMutex);
           break;
       }
       //Received Batching Response message from SHD(SENSOR HAL DAEMON)
       case E_SENSORAPI_START_BATCHING_RES_ID:
       {
           if (sizeof(SensorAPIGenericRespMsg) != length) {
                   SENSOR_LOGE(LOG_TAG "payload size does not match for message with id: %d\n",
                                   pMsg->msgId);
           }

           const SensorAPIGenericRespMsg* pRespMsg = (SensorAPIGenericRespMsg*)(pMsg);
	   pthread_mutex_lock (&mSensorLibMutex);
           mRespReturn = pRespMsg->ret;
	   pthread_cond_signal (&mSensorLibCond);
	   pthread_mutex_unlock (&mSensorLibMutex);
           break;
       }
       //Received Sensor Read Events response message from SHD(SENSOR HAL DAEMON)
       case E_SENSORAPI_START_TRACKING_MSG_ID:
       {
           if (sizeof(SensorAPIGenericRespMsg) != length) {
                   SENSOR_LOGE(LOG_TAG "payload size does not match for message with id: %d\n",
                                   pMsg->msgId);
           }
           const SensorAPIGenericRespMsg* pRespMsg = (SensorAPIGenericRespMsg*)(pMsg);
	   pthread_mutex_lock (&mSensorLibMutex);
           mRespReturn = pRespMsg->ret;
	   pthread_cond_signal (&mSensorLibCond);
	   pthread_mutex_unlock (&mSensorLibMutex);
           break;
       }
       //Received buffer data read response message from SHD(SENSOR HAL DAEMON)
       case E_SENSORAPI_SENSOR_BUFFER_REQ_MSG_ID:
       {
           if (sizeof(SensorAPIGenericRespMsg) != length) {
                   SENSOR_LOGE(LOG_TAG "payload size does not match for message with id: %d\n",
                                   pMsg->msgId);
           }
           const SensorAPIGenericRespMsg* pRespMsg = (SensorAPIGenericRespMsg*)(pMsg);
	   pthread_mutex_lock (&mSensorLibMutex);
           mRespReturn = pRespMsg->ret;
	   pthread_cond_signal (&mSensorLibCond);
	   pthread_mutex_unlock (&mSensorLibMutex);
           break;
       }
       //Received Batching config notification message from SHD(SENSOR HAL DAEMON)
       case E_SENSORAPI_START_BATCHING_MSG_ID:
       {
	   if (sizeof(SensorAPIStartBatchingReqMsg) != length) {
		   SENSOR_LOGE(LOG_TAG "payload size does not match for message with id: %d\n",
				   pMsg->msgId);
	   }
	   const SensorAPIStartBatchingReqMsg* pBatchMsg = (SensorAPIStartBatchingReqMsg*)(pMsg);
           if (mBatchingCb) {
                   mBatchingCb(pBatchMsg->sensor_id, pBatchMsg->samplingRate, pBatchMsg->batchCount, pBatchMsg->rotate);
           }
           break;
       }
       //Received Sensor events from SHD(SENSOR HAL DAEMON)
       case E_SENSORAPI_DATA_READ_MSG_ID:
       {
	   if (mClientId != SENSOR_CLIENT_SESSION_ID_INVALID) {
		   const SensorAPIDataIndMsg* pDataIndMsg = (SensorAPIDataIndMsg*)(pMsg);
		   for (int i = 0; i < mSensorCount; i++) {
			   if((mSensorTrackingOption[i].sensor_id == pDataIndMsg->sensorData.events[0].sensor)
				   && mSensorTrackingOption[i].mSensorDataReadCb)
				mSensorTrackingOption[i].mSensorDataReadCb(pDataIndMsg->sensorData.events[0].sensor,
						&pDataIndMsg->sensorData.events[0], pDataIndMsg->sensorData.count);
		   }
	   }
	   break;
       }
       //Received Sensor MLC Case events from SHD(SENSOR HAL DAEMON)
       case E_SENSORAPI_SENSOR_MLC_EVENT_IND_MSG_ID:
       {
           if (mClientId != SENSOR_CLIENT_SESSION_ID_INVALID) {
		   SensorAPIMLCEventIndMsg* pMlcEventIndMsg = (SensorAPIMLCEventIndMsg*)(pMsg);
                   for (int i = 0; i < mSensorMlcCaseCount; i++) {
			   if ((strcmp(mSensorMLCEventCbs[i].name, pMlcEventIndMsg->mlcEventData.name) == 0)
					   && mSensorMLCEventCbs[i].mSensorMLCEventCb)
				   mSensorMLCEventCbs[i].mSensorMLCEventCb(&pMlcEventIndMsg->mlcEventData.name[0],
						   &pMlcEventIndMsg->mlcEventData.event[0]);
		   }
	   }
	   break;
       }
       //Received sensor temperature from SHD(SENSOR HAL DAEMON)
       case E_SENSORAPI_SENSOR_TEMP_IND_MSG_ID:
       {
	  if((mClientId != SENSOR_CLIENT_SESSION_ID_INVALID) && mSensorTempReadCb) {
		  const SensorAPITempIndMsg* pTempIndMsg = (SensorAPITempIndMsg*)(pMsg);
		  mSensorTempReadCb(pTempIndMsg->temperature);
	  }
	  break;
       }
       //Received sensor buffer data from  SHD(SENSOR HAL DAEMON)
       case E_SENSORAPI_SENSOR_BUFFER_IND_MSG_ID:
       {
	  if((mClientId != SENSOR_CLIENT_SESSION_ID_INVALID) && mSensorBufferDataReadCb) {
		  const SensorAPIBufferDataIndMsg* pBufferDataMsg = (SensorAPIBufferDataIndMsg*) (pMsg);
		  mSensorBufferDataReadCb(&pBufferDataMsg->sensorData.events[0], pBufferDataMsg->sensorData.count);
	  }
	  break;
       }
       //Received sensor mfifo data from  SHD(SENSOR HAL DAEMON)
       case E_SENSORAPI_SENSOR_MFIFO_IND_MSG_ID:
       {
          if((mClientId != SENSOR_CLIENT_SESSION_ID_INVALID) && mSensormFifoReadCb) {
                  const SensorAPImFifoIndMsg* pmFifoMsg = (SensorAPImFifoIndMsg*) (pMsg);
                  mSensormFifoReadCb(pmFifoMsg->sensorData.events[0].sensor,
				  &pmFifoMsg->sensorData.events[0], pmFifoMsg->sensorData.count);
          }
          break;
       }
       //Received Sensor MLC case enable/disable Resp from SHD(SENSOR HAL DAEMON)
       case E_SENSORAPI_SENSOR_SELFTEST_REQ_MSG_ID:
       {
           if (sizeof(SensorAPIGenericRespMsg) != length) {
                   SENSOR_LOGE(LOG_TAG "payload size does not match for message with id: %d\n",
                                   pMsg->msgId);
           }

           const SensorAPIGenericRespMsg* pRespMsg = (SensorAPIGenericRespMsg*)(pMsg);
           pthread_mutex_lock (&mSensorLibMutex);
           mRespReturn = pRespMsg->ret;
           pthread_cond_signal (&mSensorLibCond);
           pthread_mutex_unlock (&mSensorLibMutex);
           break;
       }
       //Received sensor self test result from  SHD(SENSOR HAL DAEMON)
       case E_SENSORAPI_SENSOR_SELFTEST_IND_MSG_ID:
       {
          if((mClientId != SENSOR_CLIENT_SESSION_ID_INVALID) && mSelfTestResultCb) {
                  const SensorAPISelfTestIndMsg* pSelfMsg = (SensorAPISelfTestIndMsg*) (pMsg);
		  SENSOR_LOGI(LOG_TAG "mselftestresultcb: sensor_id = %d request_id = %d result = %d\n",
				  pSelfMsg->sensor_id, pSelfMsg->request_id, pSelfMsg->result);
                  mSelfTestResultCb(pSelfMsg->sensor_id, pSelfMsg->request_id, pSelfMsg->result);
          }
          break;
       }
       //Received Sensor Euler angle set Resp from SHD(SENSOR HAL DAEMON)
       case E_SENSORAPI_SENSOR_EULER_ANGLES_REQ_MSG_ID:
       {
           if (sizeof(SensorAPIGenericRespMsg) != length) {
                   SENSOR_LOGE(LOG_TAG "payload size does not match for message with id: %d\n",
                                   pMsg->msgId);
           }

           const SensorAPIGenericRespMsg* pRespMsg = (SensorAPIGenericRespMsg*)(pMsg);
           pthread_mutex_lock (&mSensorLibMutex);
           mRespReturn = pRespMsg->ret;
           pthread_cond_signal (&mSensorLibCond);
           pthread_mutex_unlock (&mSensorLibMutex);
           break;
       }
       //Received unknown message from SHD(SENSOR HAL DAEMON)
       default:
       {
          SENSOR_LOGE(LOG_TAG "<<< unknown message %d\n", pMsg->msgId);
	  break;
       }
   }
}


} // namespace sensor_client
