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
#include <numeric>
#include <SensorApiMsg.h>
#include <SensorHalDaemonClientHandler.h>
#include <SensorApiService.h>

default_fir_coef_t DEFAULT_ACC_FIR_COEF_SMI230 = {
   {400, {
      {200, {1.0/2, 1.0/2}},
      {100, {1.0/12, 3.0/12, 4.0/12, 3.0/12, 1.0/12}}
   }},
   {200, {
      {100, {1.0/2, 1.0/2}}
   }}
};

default_fir_coef_t DEFAULT_GYRO_FIR_COEF_SMI230 = {
   {400, {
      {200, {1.0/12, 3.0/12, 4.0/12, 3.0/12, 1.0/12}},
      {100, {1.0/45, 3.0/45, 6.0/45, 8.0/45, 9.0/45, 8.0/45, 6.0/45, 3.0/45, 1.0/45}}
   }},
   {200, {
      {100, {1.0/12, 3.0/12, 4.0/12, 3.0/12, 1.0/12}}
   }}
};

default_fir_coef_t *DEFAULT_ACC_FIR_COEF = NULL;
default_fir_coef_t *DEFAULT_GYRO_FIR_COEF = NULL;

static int get_default_fir_coef(int sensor_rate, int client_rate, bool is_accel, std::vector<float> &out_coef)
{
   const default_fir_coef_t &default_coef = is_accel ? *DEFAULT_ACC_FIR_COEF : *DEFAULT_GYRO_FIR_COEF;
   default_fir_coef_t::const_iterator it = default_coef.find(sensor_rate);
   if (it != default_coef.end())
   {
      for (const std::pair<int, std::vector<float>> &coef : it->second)
      {
         if (coef.first == client_rate)
         {
            out_coef = coef.second;
            return 0;
         }
      }
   }
   return -1;
}

void set_default_fir_coef(int sensorType)
{
   if(sensorType == 4)
   {
      //smi230
      DEFAULT_ACC_FIR_COEF = &DEFAULT_ACC_FIR_COEF_SMI230;
      DEFAULT_GYRO_FIR_COEF = &DEFAULT_GYRO_FIR_COEF_SMI230;
   }
}

int FIRFilter::init_filter(uint32_t factor, bool is_accel)
{
   std::vector<float> &fir_coef = is_accel ? fir_coef_acc : fir_coef_gyro;
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
      std::get<0>(state).clear();
      std::get<1>(state).clear();
      std::get<2>(state).clear();
   }
   order = factor;
   fir_coef.reserve(order);
   std::get<0>(state).reserve(2*order);
   std::get<1>(state).reserve(2*order);
   std::get<2>(state).reserve(2*order);
   ptr = 0;
   fir_coef.assign(order, 0.0f);
   std::get<0>(state).assign(2*order, 0.0f);
   std::get<1>(state).assign(2*order, 0.0f);
   std::get<2>(state).assign(2*order, 0.0f);
   return 0;
}

int FIRFilter::init_filter_acc(uint32_t factor)
{
   return init_filter(factor, true);
}

int FIRFilter::init_filter_gyro(uint32_t factor)
{
   return init_filter(factor, false);
}

int FIRFilter::set_filter(const std::vector<float> &coef, bool is_accel)
{
   std::vector<float> &fir_coef = is_accel ? fir_coef_acc : fir_coef_gyro;
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


int FIRFilter::set_filter_acc(const std::vector<float> &coef)
{
   return set_filter(coef, true);
}

int FIRFilter::set_filter_gyro(const std::vector<float> &coef)
{
   return set_filter(coef, false);
}


//Ref: https://ccrma.stanford.edu/~jatin/Notebooks/FIRBenchmarks.html
std::tuple<float,float,float> FIRFilter::convl(std::tuple<float,float,float> sample, bool is_accel)
{
   std::tuple<float,float,float> new_sample = sample;
   std::vector<float> &fir_coef = is_accel ? fir_coef_acc : fir_coef_gyro;
   auto &state = is_accel ? accel_state : gyro_state;
   uint32_t &order = is_accel ? order_acc : order_gyro;
   int &ptr = is_accel ? accel_ptr : gyro_ptr;

   if(order > 0)
   {
      std::get<0>(state)[ptr] = std::get<0>(sample);
      std::get<1>(state)[ptr] = std::get<1>(sample);
      std::get<2>(state)[ptr] = std::get<2>(sample);
      std::get<0>(state)[ptr + order] = std::get<0>(sample);
      std::get<1>(state)[ptr + order] = std::get<1>(sample);
      std::get<2>(state)[ptr + order] = std::get<2>(sample);

      std::get<0>(new_sample) = std::inner_product(std::get<0>(state).begin() + ptr, 
                                 std::get<0>(state).begin() + ptr + order, fir_coef.begin(), 0.0f);
      std::get<1>(new_sample) = std::inner_product(std::get<1>(state).begin() + ptr, 
                                 std::get<1>(state).begin() + ptr + order, fir_coef.begin(), 0.0f);
      std::get<2>(new_sample) = std::inner_product(std::get<2>(state).begin() + ptr, 
                                 std::get<2>(state).begin() + ptr + order, fir_coef.begin(), 0.0f);
      ptr = (ptr == 0 ? order - 1 : ptr - 1);
   }
   return new_sample;
}

static void FIRFilter::print_coefficients(const FIRFilter &filter, const char *prefix, bool is_accel)
{
   std::vector<float> fir_coef = is_accel ? filter.fir_coef_acc : filter.fir_coef_gyro;
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
   std::vector<float> coef;
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
      ret = get_default_fir_coef((int)sensor_rate, (int)client_rate, is_accel, coef);
   }
   if(coef.size() > 0)
   {
      ret = is_accel ? ((filter.init_filter_acc(coef.size())==0) && filter.set_filter_acc(coef)) 
                     : ((filter.init_filter_gyro(coef.size())==0) && filter.set_filter_gyro(coef));
      if(ret == 0)
      {
         SENSOR_LOGI(LOG_TAG "FIR coeffcient set for %0.0f to %0.0f (%s)", sensor_rate, client_rate, is_accel? "ACC" : "GYRO");
         is_accel ? fir_enabled_acc = true : fir_enabled_gyro = true;
      }
      else
      {
         SENSOR_LOGE(LOG_TAG "FIR set_filter failed for %0.0f to %0.0f (%s)", sensor_rate, client_rate, is_accel? "ACC" : "GYRO");
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
   if (strncmp(mName.c_str(), SOCKET_SENSOR_CLIENT_DIR,
                sizeof(SOCKET_SENSOR_CLIENT_DIR)-1) != 0 ) {
        char fileName[MAX_SOCKET_PATHNAME_LENGTH];
        snprintf (fileName, sizeof(fileName), "%s%s",
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
     case E_SENSORAPI_SENSOR_SELFTEST_REQ_MSG_ID: {
	SENSOR_LOGI(LOG_TAG "<-- start selftest resp ret=%d id=%u pending=%u\n", ret, id, pendingMsgId);
	SensorAPIGenericRespMsg msg(SERVICE_NAME, E_SENSORAPI_SENSOR_SELFTEST_REQ_MSG_ID, ret);
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
bool SensorHalDaemonClientHandler::SendDataToClient(sensors_event_t *e, int count) {
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
  std::tuple<float,float,float> new_fir_sample;

   SENSOR_LOGV(LOG_TAG "--< onSensorDataReadCb\n");
   if (nullptr != mIpcSender) {
    for (int i = 0; i < count ; i++) {
     switch (e[i].type) {
	 case SENSOR_TYPE_ACCELEROMETER:
	 case SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED:
	    if (mAccTracking == true && mAccEvents) {
         if(fir_enabled_acc)
         {
            new_fir_sample = filter.convl(std::tuple<float,float,float>(e[i].acceleration.x, e[i].acceleration.y, e[i].acceleration.z), true);
         }
         else
         {
            //fallback to moving average
            mAccEvents[mAccCount].uncalibrated_accelerometer.x_uncalib += e[i].acceleration.x;
            mAccEvents[mAccCount].uncalibrated_accelerometer.y_uncalib += e[i].acceleration.y;
            mAccEvents[mAccCount].uncalibrated_accelerometer.z_uncalib += e[i].acceleration.z;
         }
		 mAccMovingCount++;
		 if(mAccMovingCount >= mAccFactor) {
            if(fir_enabled_acc)
            {
               mAccEvents[mAccCount].uncalibrated_accelerometer.x_uncalib = std::get<0>(new_fir_sample);
               mAccEvents[mAccCount].uncalibrated_accelerometer.y_uncalib = std::get<1>(new_fir_sample);
               mAccEvents[mAccCount].uncalibrated_accelerometer.z_uncalib = std::get<2>(new_fir_sample);
            }
            else
            {
               mAccEvents[mAccCount].uncalibrated_accelerometer.x_uncalib /= mAccMovingCount;
               mAccEvents[mAccCount].uncalibrated_accelerometer.y_uncalib /= mAccMovingCount;
               mAccEvents[mAccCount].uncalibrated_accelerometer.z_uncalib /= mAccMovingCount;
            }
		     mAccEvents[mAccCount].timestamp = e[i].timestamp;
		     mAccEvents[mAccCount].sensor    = e[i].sensor;
		     mAccEvents[mAccCount++].type    = SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED;
		     mAccMovingCount = 0;
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
         {
            new_fir_sample = filter.convl(std::tuple<float,float,float>(e[i].gyro.x, e[i].gyro.y, e[i].gyro.z), false);
         }
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
               mGyroEvents[mGyroCount].uncalibrated_gyro.x_uncalib = std::get<0>(new_fir_sample);
               mGyroEvents[mGyroCount].uncalibrated_gyro.y_uncalib = std::get<1>(new_fir_sample);
               mGyroEvents[mGyroCount].uncalibrated_gyro.z_uncalib = std::get<2>(new_fir_sample);
            }
            else
            {
               mGyroEvents[mGyroCount].uncalibrated_gyro.x_uncalib /= mGyroMovingCount;
               mGyroEvents[mGyroCount].uncalibrated_gyro.y_uncalib /= mGyroMovingCount;
               mGyroEvents[mGyroCount].uncalibrated_gyro.z_uncalib /= mGyroMovingCount;
            }
		      mGyroEvents[mGyroCount].timestamp = e[i].timestamp;
		      mGyroEvents[mGyroCount].sensor    = e[i].sensor;
		      mGyroEvents[mGyroCount++].type    = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
		      mGyroMovingCount = 0;
		 }
		 if (mGyroCount >=  mGyroBatchCount) {
			 rc = SendDataToClient(&mGyroEvents[0], mGyroCount);
			 mGyroCount = 0;
		 }
	    }
	    break;
     }
    }
    /*Send Remaning of accel and gyro if sampling factor and batch count same to
     * avoid latency issue*/
    if (mVariableCountBatching == 1) {
	    if(mAccCount != 0 && mAccFactor == 1){
		    rc = SendDataToClient(&mAccEvents[0], mAccCount);
		    mAccCount = 0;
	    }
	    if(mGyroCount != 0 && mGyroFactor == 1){
		    rc = SendDataToClient(&mGyroEvents[0], mGyroCount);
		    mGyroCount = 0;
	    }
    }
   }
   return rc;
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
	if (mMlcCaseList == nullptr) {
		return;
	}

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
bool SensorHalDaemonClientHandler::onSensorMlcCaseEventCb(char *name , struct mlc_event_data *event) {
   std::lock_guard<std::mutex> lock(SensorApiService::mMutex);
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
           delete[] msg;
	   return rc;
   }
   return true;
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
bool SensorHalDaemonClientHandler::onSensorBufferDataReadCb(sensors_event_t *events, int count) {
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
	   delete[] msg;
	   return rc;
   }
   return true;
}

/**************************************************************************************
SensorHalDaemonClientHandler - onSensorSelfTestResultCb to nofiy client with self test result
along with sensor id.
**************************************************************************************/
void SensorHalDaemonClientHandler::onSensorSelfTestResultCb(int sensor_id, int request_id, SelfTestResult result) {
   // please do not attempt to hold the lock, as the caller of this function
   // already holds the lock
   SENSOR_LOGI(LOG_TAG "--< onSensorSelfTestResultCb\n");

   if (nullptr != mIpcSender) {
           SensorAPISelfTestIndMsg msg (SERVICE_NAME, sensor_id, request_id, result);
           bool rc = sendMessage(reinterpret_cast<uint8_t*>(&msg),
                           sizeof(msg));
           // purge this client if failed
           if (!rc) {
                   SENSOR_LOGE(LOG_TAG "failed rc=%d purging client=%s\n", rc, mName.c_str());
                   mService->deleteClientbyName(mName);
           }
   }
}
