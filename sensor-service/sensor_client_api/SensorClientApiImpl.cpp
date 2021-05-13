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
SensorClientImpl::SensorClientImpl() :
        mHalRegistered(false),
	mSensorDataReadCb(nullptr),
        mBatchingCb(nullptr),
        mSensorTempReadCb(nullptr),
        mSensorBufferDataReadCb(nullptr),
	mSensorList(nullptr),
	mSensorMlcCaseList(nullptr),
	mSensorCount(0),
	mSensorMlcCaseCount(0)
{
    onResponse = false;
    // get clientId
    uint32_t pid = (uint32_t)getpid();

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

    SENSOR_LOGI(LOG_TAG "listen on socket: %s\n", mSocketName);
    startListeningNonBlocking(mSocketName);
    //Wait till client lib is registered to deamon and received sensorlist
    while(!onResponse);
}

/******************************************************************************
SensorClientImpl - Distructor
******************************************************************************/
SensorClientImpl::~SensorClientImpl() {
}

void SensorClientImpl::destroy() {
    if (mHalRegistered && (nullptr != mIpcSender)) {
	//Send Client Deregister Message Id to hal daemon
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
}

/******************************************************************************
SensorClientImpl - GetSensorList
******************************************************************************/
int SensorClientImpl::getSensorList(struct sensor_list ***s, int *sensor_count) {

    SENSOR_LOGI(LOG_TAG ">>> GetSensorList \n");
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

    onResponse = false;
    bool SensorId = false;

    SENSOR_LOGI(LOG_TAG ">>> sensorControl sensor_id %d sensor state %d\n", sensor_id, state);

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
      //Enable/Disable the sensor
      SensorAPIEnableReqMsg msg (mSocketName, sensor_id, state);
      bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
	      sizeof(msg));
      if (true != rc) {
	 return SENSOR_ERROR_IPC_FAILED;
      }
      while(!onResponse);
      onResponse = false;
      return mRespReturn;
    }
    else
	return SENSOR_ERROR_INVALID_CLIENT;
}

/******************************************************************************
SensorClientImpl - StartBatching
******************************************************************************/
int SensorClientImpl::startBatching(int sensor_id, float sampling_rate, int batch_count, BatchingCb batchingCallback) {

    onResponse = false;
    bool SensorId = false;

    SENSOR_LOGI(LOG_TAG ">>> sensorBatching sensor_id %d sampling_rate %f batch_count %d\n", sensor_id, sampling_rate, batch_count);

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
	    if (batch_count > mSensorList[i].maxBatchCount || batch_count < mSensorList[i].minBatchCount)
		    return SENSOR_ERROR_INVALID_INPUT_PARAMETER;
	    if (sampling_rate <=0)
		    return SENSOR_ERROR_INVALID_INPUT_PARAMETER;
            break;
         }
      }
      if (SensorId != true )
         return SENSOR_ERROR_INVALID_INPUT_PARAMETER;
    }
    else
      return SENSOR_ERROR_NO_SENSORS_FOUND;

    if (SENSOR_CLIENT_SESSION_ID_INVALID != mClientId) {
      SensorAPIStartBatchingReqMsg msg (mSocketName,
	      sensor_id, sampling_rate, batch_count);
      bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
	      sizeof(msg));
      if (true != rc) {
	 return SENSOR_ERROR_IPC_FAILED;
      }
      while(!onResponse);
      onResponse = false;
      return mRespReturn;
    }
    else
	return SENSOR_ERROR_INVALID_CLIENT;
}

/******************************************************************************
SensorClientImpl - StartTracking
******************************************************************************/
int SensorClientImpl::startTracking(int sensor_id, SensorDataReadCb sensorreadCallback) {

    onResponse = false;
    bool SensorId = false;

    SENSOR_LOGI(LOG_TAG ">>> sensorTracking\n");

    //Input parameter check
    if (mSensorCount != 0) {
      for (int i=0; i < mSensorCount; i++) {
         if (mSensorList[i].sensor_id == sensor_id) {
	    SensorId = true;
         }
      }
      if (SensorId != true )
         return SENSOR_ERROR_INVALID_INPUT_PARAMETER;
    }
    else
      return SENSOR_ERROR_NO_SENSORS_FOUND;

    mSensorDataReadCb = sensorreadCallback;

    //Check about Client registered to daemon
    if (!mHalRegistered) {
	    return SENSOR_ERROR_CLIENT_REGISTER_FAILED;
    }

    if (SENSOR_CLIENT_SESSION_ID_INVALID != mClientId) {
      SensorAPIStartTrackingReqMsg msg(mSocketName);
      bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
	     sizeof(msg));
      if (true != rc) {
         return SENSOR_ERROR_IPC_FAILED;
      }
      while(!onResponse);
      onResponse = false;
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
				SensorMLCEventCb sensorMlcEventCallback) {
    onResponse = false;
    bool mlc_case = false;
    char case_name[100];

    SENSOR_LOGI(LOG_TAG ">>> SensorMLCEventEnable name %s enable %d \n", mlc_case_name, enable);

    //Check about Client registered to daemon
    if (!mHalRegistered) {
            SENSOR_LOGE(LOG_TAG ">>> sensorMLCEventEnable - Not registered yet\n");
            return SENSOR_ERROR_CLIENT_REGISTER_FAILED;
    }

    //Input parameter check
    if (enable != 0 &&  enable != 1)
            return SENSOR_ERROR_INVALID_INPUT_PARAMETER;

    if (mSensorMlcCaseCount != 0) {
      for (int i=0; i < mSensorMlcCaseCount; i++) {
	    if (strcmp(mSensorMlcCaseList[i].name, mlc_case_name) == 0) {
		    mlc_case = true;
		    break;
	    }
      }
      if (mlc_case != true ) {
         return SENSOR_ERROR_INVALID_INPUT_PARAMETER;
      }
    }
    else
      return SENSOR_ERROR_NO_MLC_CASE_FOUND;

    for (int i = 0 ; i < mSensorMlcCaseCount; i++) {
	    if (strcmp(mSensorMLCEventCbs[i].name, mlc_case_name) == 0)
		    mSensorMLCEventCbs[i].mSensorMLCEventCb = sensorMlcEventCallback;
    }

    if (SENSOR_CLIENT_SESSION_ID_INVALID != mClientId) {
      //Enable/Disable the Mlc case event
      strlcpy(case_name, mlc_case_name,100);
      SensorAPIMLCCaseEnableMsg msg (mSocketName, case_name, enable);
      bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
              sizeof(msg));
      if (true != rc) {
         return SENSOR_ERROR_IPC_FAILED;
      }
      while(!onResponse);
      onResponse = false;
      return mRespReturn;
    }
    else
	    return SENSOR_ERROR_INVALID_CLIENT;
}

/******************************************************************************
SensorClientImpl - ReadTemperature
******************************************************************************/
int SensorClientImpl::readTemperature(SensorTempReadCb sensortempreadCallback) {

    SENSOR_LOGI(LOG_TAG ">>> sensorReadTemp\n");

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

    onResponse = false;

    SENSOR_LOGI(LOG_TAG ">>> sensorBufferRead\n");

    mSensorBufferDataReadCb = sensorbufferreadCallback;

    //Check about Client registered to daemon
    if (!mHalRegistered) {
	    return SENSOR_ERROR_CLIENT_REGISTER_FAILED;
    }

    //Input parameter check
    if (enable != 1 && enable != 0)
	    return SENSOR_ERROR_INVALID_INPUT_PARAMETER;

    if (SENSOR_CLIENT_SESSION_ID_INVALID != mClientId) {
      SensorAPIBufferDataReqMsg msg(mSocketName, enable);
      bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
		      sizeof(msg));
      if (true != rc) {
	      return SENSOR_ERROR_IPC_FAILED;
      }
      while(!onResponse);
      onResponse = false;
      return mRespReturn;
    }
    else
	return SENSOR_ERROR_INVALID_CLIENT;
}

/******************************************************************************
  SensorClientImpl - onListenerReady
******************************************************************************/
void SensorClientImpl::onListenerReady() {

    SENSOR_LOGI(LOG_TAG "<<< onListenerReady\n");
    if (0 != chown(mSocketName, UID_SENSOR, GID_SENSORCLIENT)) {
	    SENSOR_LOGE(LOG_TAG "chown to group sensor client failed %s", strerror(errno));
    }
    //Send Client Register Message Id to Daemon if success,
    //set mHalRegistered to true
    if (!mHalRegistered) {
      SensorAPIClientRegisterReqMsg msg(mSocketName, SENSOR_CLIENT_API);
      bool rc = sendMessage(reinterpret_cast<uint8_t *>(&msg), sizeof(msg));
      if(true == rc)
	  mHalRegistered = true;
      else
	  onResponse = true;
    }
}

/******************************************************************************
SensorClientImpl - Process Message Receive from Daemon
******************************************************************************/
void SensorClientImpl::onReceive(const string& data) {

   SensorAPIMsgHeader *pMsg = (SensorAPIMsgHeader *)(data.data());
   uint32_t length = data.length();
    // throw away message that does not come from sensor hal daemon
   if (false == pMsg->isValidServerMsg(length)) {
	   return;
   }
   //Check for MSG ID
   switch (pMsg->msgId) {
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
		   memcpy(mSensorList, &pListIndMsg->sensorList.s[0], sizeof(struct sensor_list) * mSensorCount);
           }
	   onResponse = true;
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
                   memcpy(mSensorMlcCaseList, &pListIndMsg->sensorMlcCaseList.s[0],
				   sizeof(struct sensor_mlc_case_list) * mSensorMlcCaseCount);

                   mSensorMLCEventCbs = new (std::nothrow) struct MlcCaseListCb[mSensorMlcCaseCount];
		   for (int i = 0 ; i < mSensorMlcCaseCount ; i++) {
			   strlcpy(mSensorMLCEventCbs[i].name, mSensorMlcCaseList[i].name, 100);
			   mSensorMLCEventCbs[i].mSensorMLCEventCb = nullptr;
		   }
	   }
	   onResponse = true;
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
           mRespReturn = pRespMsg->ret;
	   onResponse = true;
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
           mRespReturn = pRespMsg->ret;
           onResponse = true;
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
           mRespReturn = pRespMsg->ret;
           onResponse = true;
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
           mRespReturn = pRespMsg->ret;
           onResponse = true;
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
           mRespReturn = pRespMsg->ret;
           onResponse = true;
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
                   mBatchingCb(pBatchMsg->sensor_id, pBatchMsg->samplingRate, pBatchMsg->batchCount);
           }
           break;
       }
       //Received Sensor events from SHD(SENSOR HAL DAEMON)
       case E_SENSORAPI_DATA_READ_MSG_ID:
       {
	   if ((mClientId != SENSOR_CLIENT_SESSION_ID_INVALID) && mSensorDataReadCb) {
		   const SensorAPIDataIndMsg* pDataIndMsg = (SensorAPIDataIndMsg*)(pMsg);
		   mSensorDataReadCb(pDataIndMsg->sensorData.events[0].sensor,
				   &pDataIndMsg->sensorData.events[0], pDataIndMsg->sensorData.count);
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
       //Received unknown message from SHD(SENSOR HAL DAEMON)
       default:
       {
          SENSOR_LOGE(LOG_TAG "<<< unknown message %d\n", pMsg->msgId);
	  break;
       }
   }
}
} // namespace sensor_client
