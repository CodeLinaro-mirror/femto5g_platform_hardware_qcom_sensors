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
 */

#include <cinttypes>
#include <SensorApiMsg.h>
#include <SensorHalDaemonClientHandler.h>
#include <SensorApiService.h>

/************************************************************************************
SensorHalDaemonClientHandler - cleanup called by SensorAPIService on delete of client
************************************************************************************/
void SensorHalDaemonClientHandler::cleanup() {
   // please do not attempt to hold the lock, as the caller of this function
   // already holds the lock
   if (0 != remove(mName.c_str())) {
	   SENSOR_LOGE(LOG_TAG "<-- failed to remove file %s error %s", mName.c_str(), strerror(errno));
   }

   //Delete memory allocated of all pointers.
   if (mIpcSender) {
        delete mIpcSender;
        mIpcSender = nullptr;
   }
   if (mAccEvents) {
	delete mAccEvents;
        mAccEvents = nullptr;
   }
   if (mGyroEvents) {
	delete mGyroEvents;
        mGyroEvents = nullptr;
   }
   if (mActivate) {
	delete mActivate;
        mActivate = nullptr;
   }
   if (mSampleRate) {
	delete mSampleRate;
        mSampleRate = nullptr;
   }
   if (mBatchCount) {
	delete mBatchCount;
        mBatchCount = nullptr;
   }
   if (mMlcCaseList) {
	delete mMlcCaseList;
        mMlcCaseList = nullptr;
   }
   //Disable Tracking Status
   mTracking = false;
   mAccTracking = false;
   mGyroTracking = false;
}

/******************************************************************************
SensorHalDaemonClientHandler - Sensor API callback functions
******************************************************************************/
void SensorHalDaemonClientHandler::onCapabilitiesCallback(SensorCapabilitiesMask mask) {
   // please do not attempt to hold the lock, as the caller of this function
   // already holds the lock
   SENSOR_LOGI(LOG_TAG "--< onCapabilitiesCallback=0x%x", mask);

   if (nullptr != mIpcSender) {
        // broadcast
        SensorAPICapabilitiesIndMsg msg(SERVICE_NAME, mask);
        bool rc = sendMessage(msg);
	// purge this client if failed
	if (!rc) {
		SENSOR_LOGE(LOG_TAG "failed rc=%d purging client=%s", rc, mName.c_str());
		mService->deleteClientbyName(mName);
	}
    }
}

/************************************************************************************
SensorHalDaemonClientHandler - OnResponseCb to send response message to client
************************************************************************************/
void SensorHalDaemonClientHandler::onResponseCb(int ret, ESensorMsgID id) {
   // please do not attempt to hold the lock, as the caller of this function
   // already holds the lock
   SENSOR_LOGI(LOG_TAG "--< onResponseCb\n");
   if (nullptr != mIpcSender) {
    SENSOR_LOGI(LOG_TAG "--< onResponseCb ret=%d id=%d\n", ret, id);
    ESensorMsgID pendingMsgId = E_SENSORAPI_UNDEFINED_MSG_ID;
    if (!mPendingMessages.empty()) {
	    pendingMsgId = mPendingMessages.front();
	    mPendingMessages.pop();
    }
    bool rc = false;
    // send corresponding indication message if pending
    switch (pendingMsgId) {
     case E_SENSORAPI_START_TRACKING_MSG_ID: {
	SENSOR_LOGI(LOG_TAG "<-- start tracking resp ret=%d id=%u pending=%u\n", ret, id, pendingMsgId);
        SensorAPIGenericRespMsg msg(SERVICE_NAME, E_SENSORAPI_START_TRACKING_MSG_ID, ret);
	rc = sendMessage(msg);
	break;
     }
     case E_SENSORAPI_SENSOR_ENABLE_MSG_ID : {
        SENSOR_LOGI(LOG_TAG "<-- start sensor enable resp ret=%d id=%u pending=%u\n", ret, id, pendingMsgId);
	SensorAPIGenericRespMsg msg(SERVICE_NAME, E_SENSORAPI_SENSOR_ENABLE_MSG_ID , ret);
	rc = sendMessage(msg);
	break;
     }
     case E_SENSORAPI_START_BATCHING_RES_ID: {
        SENSOR_LOGI(LOG_TAG "<-- start batching resp ret=%d id=%u pending=%u\n", ret, id, pendingMsgId);
        SensorAPIGenericRespMsg msg(SERVICE_NAME, E_SENSORAPI_START_BATCHING_RES_ID, ret);
        rc = sendMessage(msg);
        break;
     }
     case E_SENSORAPI_SENSOR_BUFFER_REQ_MSG_ID: {
        SENSOR_LOGI(LOG_TAG "<-- start buffer read resp ret=%d id=%u pending=%u\n", ret, id, pendingMsgId);
        SensorAPIGenericRespMsg msg(SERVICE_NAME, E_SENSORAPI_SENSOR_BUFFER_REQ_MSG_ID, ret);
        rc = sendMessage(msg);
        break;
     }
     case E_SENSORAPI_SENSOR_MLC_CASE_ENABLE_MSG_ID: {
        SENSOR_LOGI(LOG_TAG "<-- start mlc case enable/disable resp ret=%d id=%u pending=%u\n", ret, id, pendingMsgId);
        SensorAPIGenericRespMsg msg(SERVICE_NAME, E_SENSORAPI_SENSOR_MLC_CASE_ENABLE_MSG_ID, ret);
        rc = sendMessage(msg);
        break;
     }
     default: {
        SENSOR_LOGI(LOG_TAG "no pending message for %s\n", mName.c_str());
        return;
     }
    }
    // purge this client if failed
    if (!rc) {
        SENSOR_LOGE(LOG_TAG "failed rc=%d purging client=%s", rc, mName.c_str());
        mService->deleteClientbyName(mName);
    }
  }
}

/************************************************************************************
SensorHalDaemonClientHandler - SendDataToClient to send the events to clients
************************************************************************************/
void SensorHalDaemonClientHandler::SendDataToClient(sensors_event_t *e, int count) {
  // please do not attempt to hold the lock, as the caller of this function
  // already holds the lock

  if (nullptr != mIpcSender) {
     SENSOR_LOGV(LOG_TAG "--< Count %d\n", count);
     size_t msglen = sizeof(SensorAPIDataIndMsg) + sizeof(sensors_event_t) * (count - 1);
     uint8_t *msg = new(std::nothrow) uint8_t[msglen];
     if (nullptr == msg) {
	     return;
     }
     memset(msg, 0, msglen);
     SensorAPIDataIndMsg *pmsg = reinterpret_cast<SensorAPIDataIndMsg*>(msg);
     strlcpy(pmsg->mSocketName, SERVICE_NAME, MAX_SOCKET_PATHNAME_LENGTH);
     pmsg->msgId = E_SENSORAPI_DATA_READ_MSG_ID;
     pmsg->msgVersion = SENSOR_REMOTE_API_MSG_VERSION;
     pmsg->sensorData.count = count;
     memcpy(&(pmsg->sensorData.events[0]), e, sizeof(sensors_event_t) * count);
     memset(e, 0, sizeof(sensors_event_t) * count);
     bool rc = sendMessage(msg, msglen);
     // purge this client if failed
     if (!rc) {
	     SENSOR_LOGE(LOG_TAG "failed rc=%d purging client=%s\n", rc, mName.c_str());
	     mService->deleteClientbyName(mName);
     }
     delete[] msg;
  }
}

/************************************************************************************
SensorHalDaemonClientHandler - onSensorDataReadCb to store samples by taking moving avg
of samples till it reaches to requested count, once reached requested count send to client.
************************************************************************************/
void SensorHalDaemonClientHandler::onSensorDataReadCb(sensors_event_t *e, int count) {
  // please do not attempt to hold the lock, as the caller of this function
  // already holds the lock

   SENSOR_LOGV(LOG_TAG "--< onSensorDataReadCb\n");
   if (nullptr != mIpcSender) {
    for (int i = 0; i < count ; i++) {
     switch (e[i].type) {
	 case SENSOR_TYPE_ACCELEROMETER:
	 case SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED:
	    if (mAccTracking == true && mAccEvents) {
		 mAccEvents[mAccCount].uncalibrated_accelerometer.x_uncalib += e[i].acceleration.x;
		 mAccEvents[mAccCount].uncalibrated_accelerometer.y_uncalib += e[i].acceleration.y;
		 mAccEvents[mAccCount].uncalibrated_accelerometer.z_uncalib += e[i].acceleration.z;
		 mAccMovingCount++;
		 if(mAccMovingCount >= mAccFactor) {
		     mAccEvents[mAccCount].uncalibrated_accelerometer.x_uncalib /= mAccMovingCount;
		     mAccEvents[mAccCount].uncalibrated_accelerometer.y_uncalib /= mAccMovingCount;
		     mAccEvents[mAccCount].uncalibrated_accelerometer.z_uncalib /= mAccMovingCount;
		     mAccEvents[mAccCount].timestamp = e[i].timestamp;
		     mAccEvents[mAccCount].sensor    = e[i].sensor;
		     mAccEvents[mAccCount++].type    = SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED;
		     mAccMovingCount = 0;
		 }
		 if (mAccCount >=  mAccBatchCount) {
			 SendDataToClient(&mAccEvents[0], mAccCount);
			 mAccCount = 0;
		 }
	    }
	    break;
	 case SENSOR_TYPE_GYROSCOPE:
	 case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
	    if (mGyroTracking == true && mGyroEvents) {
		 mGyroEvents[mGyroCount].uncalibrated_gyro.x_uncalib += e[i].gyro.x;
		 mGyroEvents[mGyroCount].uncalibrated_gyro.y_uncalib += e[i].gyro.y;
		 mGyroEvents[mGyroCount].uncalibrated_gyro.z_uncalib += e[i].gyro.z;
		 mGyroMovingCount++;
		 if (mGyroMovingCount >= mGyroFactor) {
		      mGyroEvents[mGyroCount].uncalibrated_gyro.x_uncalib /= mGyroMovingCount;
		      mGyroEvents[mGyroCount].uncalibrated_gyro.y_uncalib /= mGyroMovingCount;
		      mGyroEvents[mGyroCount].uncalibrated_gyro.z_uncalib /= mGyroMovingCount;
		      mGyroEvents[mGyroCount].timestamp = e[i].timestamp;
		      mGyroEvents[mGyroCount].sensor    = e[i].sensor;
		      mGyroEvents[mGyroCount++].type    = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
		      mGyroMovingCount = 0;
		 }
		 if (mGyroCount >=  mGyroBatchCount) {
			 SendDataToClient(&mGyroEvents[0], mGyroCount);
			 mGyroCount = 0;
		 }
	    }
	    break;
     }
    }
   }
}

/************************************************************************************
SensorHalDaemonClientHandler - onSensorListCb to send sensorlist supported to client
************************************************************************************/
void SensorHalDaemonClientHandler::onSensorListCb(struct sensor_list *s, int count) {
   // please do not attempt to hold the lock, as the caller of this function
   // already holds the lock
   SENSOR_LOGI(LOG_TAG "--< onSensorListCb\n");

   if (nullptr != mIpcSender) {
	   size_t msglen = sizeof(SensorAPIListIndMsg) + sizeof(sensor_list) * (count - 1);
	   uint8_t *msg = new(std::nothrow) uint8_t[msglen];
	   if (nullptr == msg) {
		   return;
	   }
	   memset(msg, 0, msglen);
	   SensorAPIListIndMsg *pmsg = reinterpret_cast<SensorAPIListIndMsg*>(msg);
	   strlcpy(pmsg->mSocketName, SERVICE_NAME, MAX_SOCKET_PATHNAME_LENGTH);
	   pmsg->msgId = E_SENSORAPI_SENSOR_LIST_MSG_ID;
	   pmsg->msgVersion = SENSOR_REMOTE_API_MSG_VERSION;
	   pmsg->sensorList.count = count;
	   memcpy(&(pmsg->sensorList.s[0]), s, sizeof(struct sensor_list) * count);

	   bool rc = sendMessage(msg, msglen);
	   // purge this client if failed
	   if (!rc) {
		   SENSOR_LOGE(LOG_TAG "failed rc=%d purging client=%s\n", rc, mName.c_str());
		   mService->deleteClientbyName(mName);
	   }

	   delete[] msg;
   }
}

/**************************************************************************************
SensorHalDaemonClientHandler - onSensorBatchingCb to nofiy client with updated sampling
rate and batch count.
**************************************************************************************/
void SensorHalDaemonClientHandler::onSensorBatchingCb(int sensor_id, float SamplingRate, int BatchCount) {
   // please do not attempt to hold the lock, as the caller of this function
   // already holds the lock
   SENSOR_LOGI(LOG_TAG "--< onSensorBatchingCb\n");

   if (nullptr != mIpcSender) {
	   SensorAPIStartBatchingReqMsg msg (SERVICE_NAME, sensor_id, SamplingRate, BatchCount);
	   bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
			   sizeof(msg));
	   // purge this client if failed
	   if (!rc) {
		   SENSOR_LOGE(LOG_TAG "failed rc=%d purging client=%s\n", rc, mName.c_str());
		   mService->deleteClientbyName(mName);
	   }
   }
}

void SensorHalDaemonClientHandler::StoreMlcCaseListStatus(struct sensor_mlc_case_list *s, int count) {
	mMlcCaseList = new(std::nothrow) struct mlc_case_list[count];
	for ( int i = 0; i < count; i++) {
		strlcpy(mMlcCaseList[i].name, s[i].name, 100);
		mMlcCaseList[i].enable = 0;
	}
}
/************************************************************************************
SensorHalDaemonClientHandler - onSensorMlcCaseListCb to send mlc cse list to client
************************************************************************************/
void SensorHalDaemonClientHandler::onSensorMlcCaseListCb(struct sensor_mlc_case_list *s, int count) {
   // please do not attempt to hold the lock, as the caller of this function
   // already holds the lock
   SENSOR_LOGI(LOG_TAG "--< onSensorMlcCaseListCb\n");
   StoreMlcCaseListStatus(s, count);
   if (nullptr != mIpcSender) {
           size_t msglen = sizeof(SensorMlcCaseListIndMsg) + sizeof(sensor_mlc_case_list) * (count - 1);
           uint8_t *msg = new(std::nothrow) uint8_t[msglen];
           if (nullptr == msg) {
                   return;
           }
           memset(msg, 0, msglen);
           SensorMlcCaseListIndMsg *pmsg = reinterpret_cast<SensorMlcCaseListIndMsg*>(msg);
           strlcpy(pmsg->mSocketName, SERVICE_NAME, MAX_SOCKET_PATHNAME_LENGTH);
           pmsg->msgId = E_SENSORAPI_SENSOR_MLC_CASE_LIST_MSG_ID;
           pmsg->msgVersion = SENSOR_REMOTE_API_MSG_VERSION;
           pmsg->sensorMlcCaseList.count = count;
           memcpy(&(pmsg->sensorMlcCaseList.s[0]), s, sizeof(struct sensor_mlc_case_list) * count);

           bool rc = sendMessage(msg, msglen);
           // purge this client if failed
           if (!rc) {
                   SENSOR_LOGE(LOG_TAG "failed rc=%d purging client=%s\n", rc, mName.c_str());
                   mService->deleteClientbyName(mName);
           }

           delete[] msg;
   }
}

/************************************************************************************
SensorHalDaemonClientHandler - onSensorMlcCaseEventCb to send event to client
************************************************************************************/
void SensorHalDaemonClientHandler::onSensorMlcCaseEventCb(char *name , struct mlc_event_data *event) {
   std::lock_guard<std::mutex> lock(SensorApiService::mMutex);
   SENSOR_LOGI(LOG_TAG "--< onSensorMlcCaseEventCb name %s\n", name);

   if (nullptr != mIpcSender) {
	   SensorAPIMLCEventIndMsg msg (SERVICE_NAME, name, event);
           bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
                           sizeof(msg));
           // purge this client if failed
           if (!rc) {
                   SENSOR_LOGE(LOG_TAG "failed rc=%d purging client=%s\n", rc, mName.c_str());
                   mService->deleteClientbyName(mName);
           }
   }
}

/************************************************************************************
SensorHalDaemonClientHandler - onSensorMFifoDataReadCb to send buffer data to client
************************************************************************************/
void SensorHalDaemonClientHandler::onSensorMFifoDataReadCb(sensors_event_t *events, int count) {
   std::lock_guard<std::mutex> lock(SensorApiService::mMutex);
   SENSOR_LOGV(LOG_TAG "--< onSensorMFifoDataReadCb count %d\n", count);

   if (nullptr != mIpcSender) {
           size_t msglen = sizeof(SensorAPImFifoIndMsg) + sizeof(sensors_event_t) * (count-1);
           uint8_t *msg = new(std::nothrow) uint8_t[msglen];
           if (nullptr == msg) {
                   return;
           }
           memset(msg, 0, msglen);
           SensorAPImFifoIndMsg *pmsg = reinterpret_cast<SensorAPImFifoIndMsg*>(msg);
           strlcpy(pmsg->mSocketName, SERVICE_NAME, MAX_SOCKET_PATHNAME_LENGTH);
           pmsg->msgId = E_SENSORAPI_SENSOR_MFIFO_IND_MSG_ID;
           pmsg->msgVersion = SENSOR_REMOTE_API_MSG_VERSION;
           pmsg->sensorData.count = count;
           memcpy(&pmsg->sensorData.events[0], events, sizeof(sensors_event_t) * count);
           bool rc = sendMessage(msg, msglen);
           // purge this client if failed
           if (!rc) {
                   SENSOR_LOGE(LOG_TAG "failed rc=%d purging client=%s\n", rc, mName.c_str());
                   mService->deleteClientbyName(mName);
           }
           delete[] msg;
   }
}

/************************************************************************************
SensorHalDaemonClientHandler - onSensorTempCb to send temperature to client
************************************************************************************/
void SensorHalDaemonClientHandler::onSensorTempCb(float temperature) {
   std::lock_guard<std::mutex> lock(SensorApiService::mMutex);
   SENSOR_LOGV(LOG_TAG "--< onSensorTempCb temperature %f\n", temperature);

   if (nullptr != mIpcSender) {
	   SensorAPITempIndMsg msg (SERVICE_NAME, temperature);
	   bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
			   sizeof(msg));
	   // purge this client if failed
	   if (!rc) {
		   SENSOR_LOGE(LOG_TAG "failed rc=%d purging client=%s\n", rc, mName.c_str());
		   mService->deleteClientbyName(mName);
	   }
   }
}

/************************************************************************************
SensorHalDaemonClientHandler - onSensorBufferDataReadCb to send buffer data to client
************************************************************************************/
void SensorHalDaemonClientHandler::onSensorBufferDataReadCb(sensors_event_t *events, int count) {
   std::lock_guard<std::mutex> lock(SensorApiService::mMutex);
   SENSOR_LOGV(LOG_TAG "--< onSensorBufferReadCb count %d\n", count);

   if (nullptr != mIpcSender) {
	   size_t msglen = sizeof(SensorAPIBufferDataIndMsg) + sizeof(sensors_event_t) * (count-1);
	   uint8_t *msg = new(std::nothrow) uint8_t[msglen];
	   if (nullptr == msg) {
		   return;
	   }
	   memset(msg, 0, msglen);
	   SensorAPIBufferDataIndMsg *pmsg = reinterpret_cast<SensorAPIBufferDataIndMsg*>(msg);
	   strlcpy(pmsg->mSocketName, SERVICE_NAME, MAX_SOCKET_PATHNAME_LENGTH);
	   pmsg->msgId = E_SENSORAPI_SENSOR_BUFFER_IND_MSG_ID;
	   pmsg->msgVersion = SENSOR_REMOTE_API_MSG_VERSION;
	   pmsg->sensorData.count = count;
	   memcpy(&pmsg->sensorData.events[0], events, sizeof(sensors_event_t) * count);
	   bool rc = sendMessage(msg, msglen);
	   // purge this client if failed
	   if (!rc) {
		   SENSOR_LOGE(LOG_TAG "failed rc=%d purging client=%s\n", rc, mName.c_str());
		   mService->deleteClientbyName(mName);
	   }
	   delete[] msg;
   }
}
