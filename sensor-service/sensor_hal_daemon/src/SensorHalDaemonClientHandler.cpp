/* Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 /       disclaimer in the documentation and/or other materials provided
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

#include <cinttypes>
#include <numeric>
#include <SensorApiMsg.h>
#include <SensorHalDaemonClientHandler.h>
#include <SensorApiService.h>
#include <SensorInterfaceStubImpl.hpp>

int FIRFilter::initFilter(uint32_t factor, bool is_accel)
{
   vector<float> &fir_coef = is_accel ? fir_coef_acc : fir_coef_gyro;
   auto &state = is_accel ? accel_state : gyro_state;
   uint32_t &order = is_accel ? order_acc : order_gyro;
   int &ptr = is_accel ? accel_ptr : gyro_ptr;
   if(factor == 0)
   {
      return -1;
   }
   if(order != 0)
   {
      fir_coef.clear();
      get<0>(state).clear();
      get<1>(state).clear();
      get<2>(state).clear();
   }
   order = factor;
   fir_coef.reserve(order);
   get<0>(state).reserve(2*order);
   get<1>(state).reserve(2*order);
   get<2>(state).reserve(2*order);
   ptr = 0;
   fir_coef.assign(order, 0.0f);
   get<0>(state).assign(2*order, 0.0f);
   get<1>(state).assign(2*order, 0.0f);
   get<2>(state).assign(2*order, 0.0f);
   return 0;
}

int FIRFilter::initFilter_acc(uint32_t factor)
{
   return initFilter(factor, true);
}

int FIRFilter::initFilter_gyro(uint32_t factor)
{
   return initFilter(factor, false);
}

int FIRFilter::setFilter(const vector<float> &coef, bool is_accel)
{
   vector<float> &fir_coef = is_accel ? fir_coef_acc : fir_coef_gyro;
   auto &state = is_accel ? accel_state : gyro_state;
   uint32_t &order = is_accel ? order_acc : order_gyro;
   int &ptr = is_accel ? accel_ptr : gyro_ptr;

   if(coef.size() != order)
   {
      return -1;
   }
   fir_coef = coef;
   return 0;
}


int FIRFilter::setFilter_acc(const vector<float> &coef)
{
   return setFilter(coef, true);
}

int FIRFilter::setFilter_gyro(const vector<float> &coef)
{
   return setFilter(coef, false);
}


//Ref: https://ccrma.stanford.edu/~jatin/Notebooks/FIRBenchmarks.html
tuple<float,float,float> FIRFilter::convl(tuple<float,float,float> sample, bool is_accel)
{
   tuple<float,float,float> new_sample = sample;
   vector<float> &fir_coef = is_accel ? fir_coef_acc : fir_coef_gyro;
   auto &state = is_accel ? accel_state : gyro_state;
   uint32_t &order = is_accel ? order_acc : order_gyro;
   int &ptr = is_accel ? accel_ptr : gyro_ptr;

   if(order > 0)
   {
      get<0>(state)[ptr] = get<0>(sample);
      get<1>(state)[ptr] = get<1>(sample);
      get<2>(state)[ptr] = get<2>(sample);
      get<0>(state)[ptr + order] = get<0>(sample);
      get<1>(state)[ptr + order] = get<1>(sample);
      get<2>(state)[ptr + order] = get<2>(sample);

      get<0>(new_sample) = inner_product(get<0>(state).begin() + ptr,
                                 get<0>(state).begin() + ptr + order, fir_coef.begin(), 0.0f);
      get<1>(new_sample) = inner_product(get<1>(state).begin() + ptr,
                                 get<1>(state).begin() + ptr + order, fir_coef.begin(), 0.0f);
      get<2>(new_sample) = inner_product(get<2>(state).begin() + ptr,
                                 get<2>(state).begin() + ptr + order, fir_coef.begin(), 0.0f);
      ptr = (ptr == 0 ? order - 1 : ptr - 1);
   }
   return new_sample;
}

static void FIRFilter::print_coefficients(const FIRFilter &filter, const char *prefix, bool is_accel)
{
   vector<float> fir_coef = is_accel ? filter.fir_coef_acc : filter.fir_coef_gyro;
   if(fir_coef.size() == 0)
   {
      return;
   }
   char val_str[1024];
   val_str[0] = '[';
   val_str[1] = '\0';
   for(int i=0; i<fir_coef.size(); i++)
   {
      snprintf(val_str + strlen(val_str), 1024 - strlen(val_str), "%.2f,", fir_coef[i]);
   }
   if(strlen(val_str) < 1023)
   {
      val_str[strlen(val_str)-1] = ']';
      val_str[strlen(val_str)] = '\0';
   }
   SENSOR_LOGI(LOG_TAG "FIR Coef %s = %s\n",prefix,val_str);
}

int SensorHalDaemonClientHandler::setFIRFilter(float sensor_rate, float client_rate, bool is_accel)
{
   int ret = 0;
   vector<float> coef;
   char conf_suffix[64];

   is_accel ? fir_enabled_acc = false : fir_enabled_gyro = false;


   //Check if custom coefficient values is defined in sensors.conf
   if(is_accel)
   {
      snprintf(conf_suffix, 64, "ACC_%0.0f_%0.0f", sensor_rate, client_rate);
   }
   else
   {
      snprintf(conf_suffix, 64, "GYRO_%0.0f_%0.0f", sensor_rate, client_rate);
   }
   ret = GetFIRCoefficient(coef, conf_suffix);
   if(ret <= 0)
   {
      coef.clear();
      //check if default config is defined
      ret = mService->mSensorDevice->getDefaultFIRCoeff(is_accel, (int)sensor_rate, (int)client_rate, coef);
      if(ret != 0)
      {
         SENSOR_LOGI(LOG_TAG "No default FIR coefficient found for %0.0f to %0.0f (%s)", sensor_rate, client_rate, is_accel? "ACC" : "GYRO");
      }
   }
   if(coef.size() > 0)
   {
      ret = is_accel ? ((filter.initFilter_acc(coef.size())==0) && filter.setFilter_acc(coef)) 
                     : ((filter.initFilter_gyro(coef.size())==0) && filter.setFilter_gyro(coef));
      if(ret == 0)
      {
         SENSOR_LOGI(LOG_TAG "FIR coeffcient set for %0.0f to %0.0f (%s)", sensor_rate, client_rate, is_accel? "ACC" : "GYRO");
         is_accel ? fir_enabled_acc = true : fir_enabled_gyro = true;
      }
      else
      {
         SENSOR_LOGE(LOG_TAG "FIR setFilter failed for %0.0f to %0.0f (%s)", sensor_rate, client_rate, is_accel? "ACC" : "GYRO");
      }
   }
   else
   {
      SENSOR_LOGI(LOG_TAG "FIR coeffcient set to moving average for %0.0f to %0.0f (%s)", sensor_rate, client_rate, is_accel? "ACC" : "GYRO");
   }

   FIRFilter::print_coefficients(filter, conf_suffix, is_accel);
   return 0;
}

/************************************************************************************
SensorHalDaemonClientHandler - cleanup called by SensorAPIService on delete of client
************************************************************************************/
void SensorHalDaemonClientHandler::cleanup() {
   // please do not attempt to hold the lock, as the caller of this function
   // already holds the lock
   SENSOR_LOGI(LOG_TAG "--< SensorHalDaemonClientHandler cleanup\n");
   // check whether this is client from external AP,
   // mName for client on external ap is of format "serviceid.instanceid"
   if(strncmp(mName.c_str(),"tosomeip",sizeof(mName.c_str())) != 0) {
   if (strncmp(mName.c_str(), SOCKET_SENSOR_CLIENT_DIR,
                sizeof(SOCKET_SENSOR_CLIENT_DIR)-1) != 0 ) {
        char fileName[MAX_SOCKET_PATHNAME_LENGTH];
        (void)snprintf (fileName, sizeof(fileName), "%s%s",
                  SOCKET_TO_EXTERANL_AP_SENSOR_CLIENT_BASE, mName.c_str());
        SENSOR_LOGI(LOG_TAG "removed file name %s\n", fileName);
        if (0 != remove(fileName)) {
		SENSOR_LOGE(LOG_TAG "<-- failed to remove file %s error %s\n", fileName, strerror(errno));
        }
   } else {
	   if (0 != remove(mName.c_str()))
		   SENSOR_LOGE(LOG_TAG "<-- failed to remove file %s error %s", mName.c_str(), strerror(errno));
   }

   //Delete memory allocated of all pointers.
   if (mIpcSender) {
	mIpcSender->cleanup(mName);
        delete mIpcSender;
        mIpcSender = nullptr;
   }
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
   if (mMlcCaseList) {
	delete mMlcCaseList;
        mMlcCaseList = nullptr;
   }

   //Disable Tracking Status
   mTracking = false;
   mAccTracking = false;
   mGyroTracking = false;
   mMlcEnable = false;
}

/******************************************************************************
SensorHalDaemonClientHandler - Sensor API callback functions
******************************************************************************/
bool SensorHalDaemonClientHandler::onCapabilitiesCallback(SensorCapabilitiesMask mask) {
   // please do not attempt to hold the lock, as the caller of this function
   // already holds the lock
   SENSOR_LOGI(LOG_TAG "--< onCapabilitiesCallback=0x%x", mask);
   bool rc = false;
#ifdef SENSOR_IVSS_ENABLED
   if(strncmp(mName.c_str(),"tosomeip",sizeof(mName.c_str())) == 0) {
     mService->myService->onCapabilitiesCallback(mask);
     return true;
   }
#endif
   if (nullptr != mIpcSender) {
        // broadcast
        SensorAPICapabilitiesIndMsg msg(SERVICE_NAME, mask);
        rc = sendMessage(msg);
   }
   return rc;
}

/************************************************************************************
SensorHalDaemonClientHandler - OnResponseCb to send response message to client
************************************************************************************/
void SensorHalDaemonClientHandler::onResponseCb(int ret, ESensorMsgID id) {
   // please do not attempt to hold the lock, as the caller of this function
   // already holds the lock
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
     case E_SENSORAPI_SENSOR_SELFTEST_REQ_MSG_ID: {
	SENSOR_LOGI(LOG_TAG "<-- start selftest resp ret=%d id=%u pending=%u\n", ret, id, pendingMsgId);
	SensorAPIGenericRespMsg msg(SERVICE_NAME, E_SENSORAPI_SENSOR_SELFTEST_REQ_MSG_ID, ret);
        rc = sendMessage(msg);
        break;
     }
     case E_SENSORAPI_SENSOR_EULER_ANGLES_REQ_MSG_ID: {
	SENSOR_LOGI(LOG_TAG "<-- start upadte rotation matrix resp ret=%d id=%u pending=%u\n", ret, id, pendingMsgId);
	SensorAPIGenericRespMsg msg(SERVICE_NAME, E_SENSORAPI_SENSOR_EULER_ANGLES_REQ_MSG_ID, ret);
        rc = sendMessage(msg);
        break;
     }
     case E_SENSORAPI_SENSOR_WAKEUP_CONFIG_REQ_MSG_ID: {
	SENSOR_LOGI(LOG_TAG "<-- start sensor wakeup config get ret=%d id=%u pending=%u\n", ret, id, pendingMsgId);
	SensorAPIGenericRespMsg msg(SERVICE_NAME, E_SENSORAPI_SENSOR_WAKEUP_CONFIG_REQ_MSG_ID, ret);
        rc = sendMessage(msg);
        break;
     }
     case E_SENSORAPI_SENSOR_WAKEUP_UPDATE_REQ_MSG_ID: {
	SENSOR_LOGI(LOG_TAG "<-- start sensor wakeup update get ret=%d id=%u pending=%u\n", ret, id, pendingMsgId);
	SensorAPIGenericRespMsg msg(SERVICE_NAME, E_SENSORAPI_SENSOR_WAKEUP_CONFIG_REQ_MSG_ID, ret);
        rc = sendMessage(msg);
        break;
     }
     case E_SENSORAPI_SENSOR_WAKEUP_ENABLE_REQ_MSG_ID: {
	SENSOR_LOGI(LOG_TAG "<-- start sensor wakeup enable/disable resp ret=%d id=%u pending=%u\n", ret, id, pendingMsgId);
	SensorAPIGenericRespMsg msg(SERVICE_NAME, E_SENSORAPI_SENSOR_WAKEUP_ENABLE_REQ_MSG_ID, ret);
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
        (void)mService->deleteClientbyName(mName);
    }
  }
}

/************************************************************************************
SensorHalDaemonClientHandler - SendDataToClient to send the events to clients
************************************************************************************/
bool SensorHalDaemonClientHandler::SendDataToClient(sensors_event_t *e, int count) {
  // please do not attempt to hold the lock, as the caller of this function
  // already holds the lock
#ifdef SENSOR_IVSS_ENABLED
  if(strncmp(mName.c_str(),"tosomeip",sizeof(mName.c_str())) == 0) {
     mService->myService->onSensorDataReadCb(e, count);
     (void)memset(e, 0, sizeof(sensors_event_t) * count);
     return true;
  }
#endif
  if (nullptr != mIpcSender) {
     SENSOR_LOGV(LOG_TAG "--< Count %d\n", count);
     size_t msglen = sizeof(SensorAPIDataIndMsg) + sizeof(sensors_event_t) * (count - 1);
     uint8_t *msg = new(nothrow) uint8_t[msglen];
     if (nullptr == msg) {
	     return false;
     }
     (void)memset(msg, 0, msglen);
     SensorAPIDataIndMsg *pmsg = reinterpret_cast<SensorAPIDataIndMsg*>(msg);
     (void)strlcpy(pmsg->mSocketName, SERVICE_NAME, MAX_SOCKET_PATHNAME_LENGTH);
     pmsg->msgId = E_SENSORAPI_DATA_READ_MSG_ID;
     pmsg->msgVersion = SENSOR_REMOTE_API_MSG_VERSION;
     pmsg->sensorData.count = count;
     (void)memcpy(&(pmsg->sensorData.events[0]), e, sizeof(sensors_event_t) * count);
     (void)memset(e, 0, sizeof(sensors_event_t) * count);
     bool rc = sendMessage(msg, msglen);
     delete[] msg;
     return rc;
  }
  return false;
}

/************************************************************************************
SensorHalDaemonClientHandler - onSensorDataReadCb to store samples by taking moving avg
of samples till it reaches to requested count, once reached requested count send to client.
************************************************************************************/
bool SensorHalDaemonClientHandler::onSensorDataReadCb(sensors_event_t *e, int count) {
  // please do not attempt to hold the lock, as the caller of this function
  // already holds the lock
  bool rc = true;
  float temp_data[3];
  tuple<float,float,float> new_fir_sample;

   SENSOR_LOGV(LOG_TAG "--< onSensorDataReadCb\n");
   if (nullptr != mIpcSender || (strncmp(mName.c_str(),"tosomeip",sizeof(mName.c_str())) == 0)) {
    for (int i = 0; i < count ; i++) {
     switch (e[i].type) {
	 case SENSOR_TYPE_ACCELEROMETER:
	 case SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED:
	    if (mAccTracking == true && mAccEvents) {
	       if(fir_enabled_acc)
		       new_fir_sample = filter.convl(tuple<float,float,float>(e[i].acceleration.x, e[i].acceleration.y, e[i].acceleration.z),
				       true);
	       else {
		       //fallback to moving average
		       mAccEvents[mAccCount].uncalibrated_accelerometer.x_uncalib += e[i].acceleration.x;
		       mAccEvents[mAccCount].uncalibrated_accelerometer.y_uncalib += e[i].acceleration.y;
		       mAccEvents[mAccCount].uncalibrated_accelerometer.z_uncalib += e[i].acceleration.z;
	       }
	       mAccMovingCount++;
	       if(mAccMovingCount >= mAccFactor) {
		 if(fir_enabled_acc)
		 {
			 mAccEvents[mAccCount].uncalibrated_accelerometer.x_uncalib = get<0>(new_fir_sample);
			 mAccEvents[mAccCount].uncalibrated_accelerometer.y_uncalib = get<1>(new_fir_sample);
			 mAccEvents[mAccCount].uncalibrated_accelerometer.z_uncalib = get<2>(new_fir_sample);
		 }
		 else
		 {
			 mAccEvents[mAccCount].uncalibrated_accelerometer.x_uncalib /= mAccMovingCount;
			 mAccEvents[mAccCount].uncalibrated_accelerometer.y_uncalib /= mAccMovingCount;
			 mAccEvents[mAccCount].uncalibrated_accelerometer.z_uncalib /= mAccMovingCount;
		 }
		 mAccEvents[mAccCount].timestamp = e[i].timestamp;
		 mAccEvents[mAccCount].sensor    = e[i].sensor;
		 mAccEvents[mAccCount].type      = SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED;
		 if(mAccRotate) {
			 (void)memcpy(&temp_data, &mAccEvents[mAccCount].uncalibrated_accelerometer, 3 * sizeof(float));
			 mAccEvents[mAccCount].uncalibrated_accelerometer.x_uncalib =
				 mService->rot[0][0] * temp_data[0] + // Matrix Multtiplication
				 mService->rot[1][0] * temp_data[1] + // Matrix Multtiplication
				 mService->rot[2][0] * temp_data[2]; // Get Rotated X coordinate
			 mAccEvents[mAccCount].uncalibrated_accelerometer.y_uncalib =
				 mService->rot[0][1] * temp_data[0] + // Matrix Multtiplication
				 mService->rot[1][1] * temp_data[1] + // Matrix Multtiplication
				 mService->rot[2][1] * temp_data[2]; // Get Rotated Y coordinate
			 mAccEvents[mAccCount].uncalibrated_accelerometer.z_uncalib =
				 mService->rot[0][2] * temp_data[0] + // Matrix Multtiplication
				 mService->rot[1][2] * temp_data[1] + // Matrix Multtiplication
				 mService->rot[2][2] * temp_data[2]; // Get Rotated Z coordinate
		 }
		 mAccMovingCount = 0;
		 mAccCount++;
	       }
	       if (mAccCount >=  mAccBatchCount) {
		       rc = SendDataToClient(&mAccEvents[0], mAccCount);
		       mAccCount = 0;
	       }
	    }
	    break;
	 case SENSOR_TYPE_GYROSCOPE:
	 case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
	    if (mGyroTracking == true && mGyroEvents) {
               if(fir_enabled_gyro)
		       new_fir_sample = filter.convl(tuple<float,float,float>(e[i].gyro.x, e[i].gyro.y, e[i].gyro.z), false);
	       else
	       {
		       //fallback to moving average
		       mGyroEvents[mGyroCount].uncalibrated_gyro.x_uncalib += e[i].gyro.x;
		       mGyroEvents[mGyroCount].uncalibrated_gyro.y_uncalib += e[i].gyro.y;
		       mGyroEvents[mGyroCount].uncalibrated_gyro.z_uncalib += e[i].gyro.z;
	       }
	       mGyroMovingCount++;
	       if (mGyroMovingCount >= mGyroFactor) {
		       if(fir_enabled_gyro)
		       {
			       mGyroEvents[mGyroCount].uncalibrated_gyro.x_uncalib = get<0>(new_fir_sample);
			       mGyroEvents[mGyroCount].uncalibrated_gyro.y_uncalib = get<1>(new_fir_sample);
			       mGyroEvents[mGyroCount].uncalibrated_gyro.z_uncalib = get<2>(new_fir_sample);
		       }
		       else
		       {
			       mGyroEvents[mGyroCount].uncalibrated_gyro.x_uncalib /= mGyroMovingCount;
			       mGyroEvents[mGyroCount].uncalibrated_gyro.y_uncalib /= mGyroMovingCount;
			       mGyroEvents[mGyroCount].uncalibrated_gyro.z_uncalib /= mGyroMovingCount;
		       }

		       mGyroEvents[mGyroCount].timestamp = e[i].timestamp;
		       mGyroEvents[mGyroCount].sensor    = e[i].sensor;
		       mGyroEvents[mGyroCount].type    = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
		       if(mGyroRotate) {
			       (void)memcpy(&temp_data, &mGyroEvents[mGyroCount].uncalibrated_gyro, 3 * sizeof(float));
			       mGyroEvents[mGyroCount].uncalibrated_gyro.x_uncalib =
				       mService->rot[0][0] * temp_data[0] + // Matrix Multtiplication
				       mService->rot[1][0] * temp_data[1] + // Matrix Multtiplication
				       mService->rot[2][0] * temp_data[2]; // Get Rotated X coordinate
			       mGyroEvents[mGyroCount].uncalibrated_gyro.y_uncalib =
				       mService->rot[0][1] * temp_data[0] + // Matrix Multtiplication
				       mService->rot[1][1] * temp_data[1] + // Matrix Multtiplication
				       mService->rot[2][1] * temp_data[2]; // Get Rotated Y coordinate
			       mGyroEvents[mGyroCount].uncalibrated_gyro.z_uncalib =
				       mService->rot[0][2] * temp_data[0] + // Matrix Multtiplication
				       mService->rot[1][2] * temp_data[1] + // Matrix Multtiplication
				       mService->rot[2][2] * temp_data[2]; // Get Rotated Z coordinate
		       }
		       mGyroMovingCount = 0;
		       mGyroCount++;
	       }
	       if (mGyroCount >=  mGyroBatchCount) {
		       rc = SendDataToClient(&mGyroEvents[0], mGyroCount);
		       mGyroCount = 0;
	       }
	    }
	    break;
     }//End of switch
    }//End of for loop for samples
   }//End of client

   return rc;
}

/************************************************************************************
SensorHalDaemonClientHandler - onSensorListCb to send sensorlist supported to client
************************************************************************************/
void SensorHalDaemonClientHandler::onSensorListCb(struct sensor_list *s, int count) {
   // please do not attempt to hold the lock, as the caller of this function
   // already holds the lock
   SENSOR_LOGI(LOG_TAG "--< onSensorListCb\n");
#ifdef SENSOR_IVSS_ENABLED
   if(strncmp(mName.c_str(),"tosomeip",sizeof(mName.c_str())) == 0)
	   return;
#endif
   if (nullptr != mIpcSender) {
	   size_t msglen = sizeof(SensorAPIListIndMsg) + sizeof(sensor_list) * (count - 1);
	   uint8_t *msg = new(nothrow) uint8_t[msglen];
	   if (nullptr == msg) {
		   return;
	   }
	   (void)memset(msg, 0, msglen);
	   SensorAPIListIndMsg *pmsg = reinterpret_cast<SensorAPIListIndMsg*>(msg);
	   (void)strlcpy(pmsg->mSocketName, SERVICE_NAME, MAX_SOCKET_PATHNAME_LENGTH);
	   pmsg->msgId = E_SENSORAPI_SENSOR_LIST_MSG_ID;
	   pmsg->msgVersion = SENSOR_REMOTE_API_MSG_VERSION;
	   pmsg->sensorList.count = count;
	   (void)memcpy(&(pmsg->sensorList.s[0]), s, sizeof(struct sensor_list) * count);

	   bool rc = sendMessage(msg, msglen);
	   // purge this client if failed
	   if (!rc) {
		   SENSOR_LOGE(LOG_TAG "failed rc=%d purging client=%s\n", rc, mName.c_str());
		   (void)mService->deleteClientbyName(mName);
	   }

	   delete[] msg;
   }
}

/**************************************************************************************
SensorHalDaemonClientHandler - onSensorBatchingCb to nofiy client with updated sampling
rate and batch count.
**************************************************************************************/
void SensorHalDaemonClientHandler::onSensorBatchingCb(int sensor_id, float SamplingRate, int BatchCount, bool Rotate) {
   // please do not attempt to hold the lock, as the caller of this function
   // already holds the lock
   SENSOR_LOGI(LOG_TAG "--< onSensorBatchingCb\n");
#ifdef SENSOR_IVSS_ENABLED
   if(strncmp(mName.c_str(),"tosomeip",sizeof(mName.c_str())) == 0) {
           mService->myService->fireSensorConfigUpdateEvent(sensor_id, SamplingRate, BatchCount);
	   return;
   }
#endif 
   if (nullptr != mIpcSender) {
	   SensorAPIStartBatchingReqMsg msg (SERVICE_NAME, sensor_id, SamplingRate, BatchCount, Rotate);
	   bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
			   sizeof(msg));
	   // purge this client if failed
	   if (!rc) {
		   SENSOR_LOGE(LOG_TAG "failed rc=%d purging client=%s\n", rc, mName.c_str());
		   (void)mService->deleteClientbyName(mName);
	   }
   }
}

void SensorHalDaemonClientHandler::StoreMlcCaseListStatus(struct sensor_mlc_case_list *s, int count) {
   mMlcCaseList = new(nothrow) struct mlc_case_list[count];
   if (mMlcCaseList == nullptr) {
	   return;
   }

   for ( int i = 0; i < count; i++) {
	   (void)strlcpy(mMlcCaseList[i].name, s[i].name, 100);
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
#ifdef SENSOR_IVSS_ENABLED
   if(strncmp(mName.c_str(),"tosomeip",sizeof(mName.c_str())) == 0)
	   return;
#endif
   StoreMlcCaseListStatus(s, count);
   if (nullptr != mIpcSender) {
           size_t msglen = sizeof(SensorMlcCaseListIndMsg) + sizeof(sensor_mlc_case_list) * (count - 1);
           uint8_t *msg = new(nothrow) uint8_t[msglen];
           if (nullptr == msg) {
                   return;
           }
           (void)memset(msg, 0, msglen);
           SensorMlcCaseListIndMsg *pmsg = reinterpret_cast<SensorMlcCaseListIndMsg*>(msg);
           (void)strlcpy(pmsg->mSocketName, SERVICE_NAME, MAX_SOCKET_PATHNAME_LENGTH);
           pmsg->msgId = E_SENSORAPI_SENSOR_MLC_CASE_LIST_MSG_ID;
           pmsg->msgVersion = SENSOR_REMOTE_API_MSG_VERSION;
           pmsg->sensorMlcCaseList.count = count;
           (void)memcpy(&(pmsg->sensorMlcCaseList.s[0]), s, sizeof(struct sensor_mlc_case_list) * count);

           bool rc = sendMessage(msg, msglen);
           // purge this client if failed
           if (!rc) {
                   SENSOR_LOGE(LOG_TAG "failed rc=%d purging client=%s\n", rc, mName.c_str());
                   (void)mService->deleteClientbyName(mName);
           }

           delete[] msg;
   }
}

/************************************************************************************
SensorHalDaemonClientHandler - onSensorMlcCaseEventCb to send event to client
************************************************************************************/
bool SensorHalDaemonClientHandler::onSensorMlcCaseEventCb(char *name , struct mlc_event_data *event) {
   lock_guard<mutex> lock(SensorApiService::mMutex);
   SENSOR_LOGI(LOG_TAG "--< onSensorMlcCaseEventCb name %s\n", name);
   if (nullptr != mIpcSender) {
	   SensorAPIMLCEventIndMsg msg (SERVICE_NAME, name, event);
           bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
                           sizeof(msg));
	   return rc;
   }
   return true;
}

/************************************************************************************
SensorHalDaemonClientHandler - onSensorMFifoDataReadCb to send buffer data to client
************************************************************************************/
bool SensorHalDaemonClientHandler::onSensorMFifoDataReadCb(sensors_event_t *events, int count) {
   lock_guard<mutex> lock(SensorApiService::mMutex);
   SENSOR_LOGV(LOG_TAG "--< onSensorMFifoDataReadCb count %d\n", count);
   if (nullptr != mIpcSender) {
           size_t msglen = sizeof(SensorAPImFifoIndMsg) + sizeof(sensors_event_t) * (count-1);
           uint8_t *msg = new(nothrow) uint8_t[msglen];
           if (nullptr == msg) {
                   return false;
           }
           (void)memset(msg, 0, msglen);
           SensorAPImFifoIndMsg *pmsg = reinterpret_cast<SensorAPImFifoIndMsg*>(msg);
           (void)strlcpy(pmsg->mSocketName, SERVICE_NAME, MAX_SOCKET_PATHNAME_LENGTH);
           pmsg->msgId = E_SENSORAPI_SENSOR_MFIFO_IND_MSG_ID;
           pmsg->msgVersion = SENSOR_REMOTE_API_MSG_VERSION;
           pmsg->sensorData.count = count;
           (void)memcpy(&pmsg->sensorData.events[0], events, sizeof(sensors_event_t) * count);
           bool rc = sendMessage(msg, msglen);
           delete[] msg;
	   return rc;
   }
   return true;
}

/************************************************************************************
SensorHalDaemonClientHandler - onSensorTempCb to send temperature to client
************************************************************************************/
void SensorHalDaemonClientHandler::onSensorTempCb(float temperature) {
   lock_guard<mutex> lock(SensorApiService::mMutex);
   SENSOR_LOGV(LOG_TAG "--< onSensorTempCb temperature %f\n", temperature);

   if (nullptr != mIpcSender) {
	   SensorAPITempIndMsg msg (SERVICE_NAME, temperature);
	   bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
			   sizeof(msg));
	   // purge this client if failed
	   if (!rc) {
		   SENSOR_LOGE(LOG_TAG "failed rc=%d purging client=%s\n", rc, mName.c_str());
		   (void)mService->deleteClientbyName(mName);
	   }
   }
}

/************************************************************************************
SensorHalDaemonClientHandler - onSensorBufferDataReadCb to send buffer data to client
************************************************************************************/
bool SensorHalDaemonClientHandler::onSensorBufferDataReadCb(sensors_event_t *events, int count) {
   lock_guard<mutex> lock(SensorApiService::mMutex);
   SENSOR_LOGV(LOG_TAG "--< onSensorBufferReadCb count %d\n", count);

   if (nullptr != mIpcSender) {
	   size_t msglen = sizeof(SensorAPIBufferDataIndMsg) + sizeof(sensors_event_t) * (count-1);
	   uint8_t *msg = new(nothrow) uint8_t[msglen];
	   if (nullptr == msg) {
		   return false;
	   }
	   (void)memset(msg, 0, msglen);
	   SensorAPIBufferDataIndMsg *pmsg = reinterpret_cast<SensorAPIBufferDataIndMsg*>(msg);
	   (void)strlcpy(pmsg->mSocketName, SERVICE_NAME, MAX_SOCKET_PATHNAME_LENGTH);
	   pmsg->msgId = E_SENSORAPI_SENSOR_BUFFER_IND_MSG_ID;
	   pmsg->msgVersion = SENSOR_REMOTE_API_MSG_VERSION;
	   pmsg->sensorData.count = count;
	   (void)memcpy(&pmsg->sensorData.events[0], events, sizeof(sensors_event_t) * count);
	   bool rc = sendMessage(msg, msglen);
	   delete[] msg;
	   return rc;
   }
   return true;
}

/**************************************************************************************
SensorHalDaemonClientHandler - onSensorSelfTestResultCb to nofiy client with self test result
along with sensor id.
**************************************************************************************/
void SensorHalDaemonClientHandler::onSensorSelfTestResultCb(int sensor_id, int request_id, SelfTestResult result, SelfTestResultType resultType, uint64_t timestamp) {
   // please do not attempt to hold the lock, as the caller of this function
   // already holds the lock
   SENSOR_LOGI(LOG_TAG "--< onSensorSelfTestResultCb\n");

   if (nullptr != mIpcSender) {
           SensorAPISelfTestIndMsg msg (SERVICE_NAME, sensor_id, request_id, result, resultType, timestamp);
           bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
                           sizeof(msg));
           // purge this client if failed
           if (!rc) {
                   SENSOR_LOGE(LOG_TAG "failed rc=%d purging client=%s\n", rc, mName.c_str());
                   (void)mService->deleteClientbyName(mName);
           }
   }
}
/**************************************************************************************
SensorHalDaemonClientHandler - onSensorEventCb to nofiy client with wake up status
**************************************************************************************/
bool SensorHalDaemonClientHandler::onSensorWakeupConfigRequestCb(struct wakeup_config_info wakeup_info) {
   // please do not attempt to hold the lock, as the caller of this function
   // already holds the lock
   SENSOR_LOGI(LOG_TAG "--< onSensorWakeupConfigRequestCb\n");
   if (nullptr != mIpcSender) {
           SensorAPIWakeupConfigIndMsg msg (SERVICE_NAME, wakeup_info);
           bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
                           sizeof(msg));
	   return rc;
   }
   return true;
}

/**************************************************************************************
SensorHalDaemonClientHandler - onSensorWakeupConfigUpdateCb to nofiy client with wakeup config
**************************************************************************************/
bool SensorHalDaemonClientHandler::onSensorWakeupConfigUpdateCb(int sensor_id, struct wakeup_config wakeup) {
   // please do not attempt to hold the lock, as the caller of this function
   // already holds the lock
   SENSOR_LOGI(LOG_TAG "--< onSensorWakeupConfigUpdateCb\n");

   if (nullptr != mIpcSender) {
           SensorAPIWakeupConfigUpdateIndMsg msg (SERVICE_NAME, sensor_id, wakeup);
           bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
                           sizeof(msg));
	   return rc;
   }
   return true;
}

/**************************************************************************************
SensorHalDaemonClientHandler - onSensorEventCb to nofiy client with wake up status
**************************************************************************************/
bool SensorHalDaemonClientHandler::onSensorEventCb(int sensor_id, struct iio_event_data event) {
   lock_guard<mutex> lock(SensorApiService::mMutex);
   SENSOR_LOGI(LOG_TAG "--< onSensorEventCb\n");

   if (nullptr != mIpcSender) {
           SensorAPIWakeupEnableIndMsg msg (SERVICE_NAME, sensor_id, event);
           bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
                           sizeof(msg));
	   return rc;
   }
   return true;
}

/**************************************************************************************
SensorHalDaemonClientHandler - onSensorWakeupEnableConfigUpdateCb to nofiy client with with wakeup config on enable
**************************************************************************************/
bool SensorHalDaemonClientHandler::onSensorWakeupEnableConfigUpdateCb(int sensor_id, struct wakeup_config wakeup) {
   // please do not attempt to hold the lock, as the caller of this function
   // already holds the lock
   SENSOR_LOGI(LOG_TAG "--< onSensorWakeupEnableConfigUpdateCb\n");

   if (nullptr != mIpcSender) {
         SensorAPIWakeupEnableConfigUpdateIndMsg msg (SERVICE_NAME, sensor_id, wakeup);
         bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
                        sizeof(msg));
	   return rc;
   }
   return true;
}
