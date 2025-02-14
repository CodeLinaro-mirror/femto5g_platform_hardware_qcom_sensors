/* Copyright (c) 2021 The Linux Foundation. All rights reserved.
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
#include <sstream>
#include <fstream>
#include <algorithm>
#include <SensorHalDaemonClientHandler.h>
#include <SensorApiService.h>
#include <cstring>
#include <math.h>
#include <errno.h>

using namespace std;

/*Buffer read files - common*/
#define ACCNAME_BUFF_PATH       "/dev/input/accbuff"
#define GYRNAME_BUFF_PATH       "/dev/input/gyrobuff"
static FILE *mfdBuffAccel  = NULL;
static FILE *mfdBuffGyro   = NULL;
static float mAccelBuffScale   = 0;
static float mGyroBuffScale    = 0;

/* @brief Check for bufffer support
 *
 * @return true if buffer supported else false.
 */
bool SensorApiService::checkSensorBufferSupport()
{

 if (!mSensorDevice->getSensorBufferFile(&mAccBootSample, &mGyroBootSample))
	 return false;

 SENSOR_LOGI(LOG_TAG "mAccBootSample-%s,mGyroBootSample-%s\n",mAccBootSample.c_str(),mGyroBootSample.c_str());

 mSensorDevice->getSensorBufferScalarValue(&mAccelBuffScale, &mGyroBuffScale);

 (void)pthread_mutex_init(&mHalBuffMutex, NULL);
 (void)pthread_cond_init(&mHalBuffCond, NULL);

 return true;
}

/* @brief enable and delete buffer data
 *
 * write 1 to read buffer data and 0 to delete buffer data.
 */
bool SensorApiService::writeToBufferFile(bool enable) {
  FILE  *mfdBuffAccelE = NULL;
  FILE  *mfdBuffGyroE  = NULL;

  if ((mfdBuffAccelE = fopen(mAccBootSample.c_str(), "w")) == NULL) {
	  SENSOR_LOGE(LOG_TAG "failed to open %s errno %d, (%s)\n",
			  mAccBootSample.c_str(), errno, strerror(errno));
	  goto fail;
  }

  if ((mfdBuffGyroE = fopen(mGyroBootSample.c_str(), "w")) == NULL) {
	  SENSOR_LOGE(LOG_TAG "failed to open %s errno %d, (%s)\n",
			  mGyroBootSample.c_str(), errno, strerror(errno));
	  goto fail;
  }

  if (enable == true) {
      if ((mfdBuffAccel = fopen(ACCNAME_BUFF_PATH, "r")) == NULL) {
         SENSOR_LOGE(LOG_TAG "failed to open %s errno %d, (%s)\n", ACCNAME_BUFF_PATH, errno, strerror(errno));
         goto fail;
      }
      /* Open Gyro Bufferd Sensor input device */
      if ((mfdBuffGyro = fopen(GYRNAME_BUFF_PATH, "r")) == NULL) {
         SENSOR_LOGE(LOG_TAG "failed to open %s errno %d, (%s)\n", GYRNAME_BUFF_PATH, errno, strerror(errno));
         goto fail;
      }

      if (fwrite("1", 1, 1, mfdBuffAccelE) != 1){
	      SENSOR_LOGE(LOG_TAG "failed to write data into %s, errno = %d (%s)\n",
			      mAccBootSample.c_str(), errno, strerror(errno));
	      goto fail;
      } else {
	      (void)fflush(mfdBuffAccelE);
      }
      if (fwrite("1", 1, 1, mfdBuffGyroE) != 1){
	      SENSOR_LOGE(LOG_TAG "failed to write data into %s, errno = %d (%s)\n",
			      mGyroBootSample.c_str(), errno, strerror(errno));
	      goto fail;
      } else {
	      (void)fflush(mfdBuffGyroE);
      }
  }

  if (enable == false) {
      if ((fwrite("0", 1, 1, mfdBuffAccelE) != 1)) {
	      SENSOR_LOGE(LOG_TAG "failed to write data into %s, errno = %d (%s)",
			      mAccBootSample.c_str(), errno, strerror(errno));
	      goto fail;
      } else {
	      (void)fflush(mfdBuffAccelE);
      }
      if (fwrite("0", 1, 1, mfdBuffGyroE) != 1){
	      SENSOR_LOGE(LOG_TAG "failed to write data into %s, errno = %d (%s)",
			      mGyroBootSample.c_str(), errno, strerror(errno));
	      goto fail;
      } else {
	      (void)fflush(mfdBuffGyroE);
      }
      mBufferDeleted = true;
  }
  CLOSE_FILE_HANDLE(mfdBuffAccelE);
  CLOSE_FILE_HANDLE(mfdBuffGyroE);
  return true;
fail:
  CLOSE_FILE_HANDLE(mfdBuffAccel);
  CLOSE_FILE_HANDLE(mfdBuffGyro);
  CLOSE_FILE_HANDLE(mfdBuffAccelE);
  CLOSE_FILE_HANDLE(mfdBuffGyroE);
  return false;
}
/* @brief Check for sensor and scale buffer data
 * event - structure containing raw data
*/
void SensorApiService::bufferDataScaling(int type, sensors_event_t *event ) {
   /* Get the scale factor based on sensor type */
   if (type == SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED) {
	   event->type = SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED;
	   event->acceleration.x = ((float) ((int16_t)event->acceleration.x) * mAccelBuffScale);
	   event->acceleration.y = ((float) ((int16_t)event->acceleration.y) * mAccelBuffScale);
	   event->acceleration.z = ((float) ((int16_t)event->acceleration.z) * mAccelBuffScale);
   }
   else {
	   event->type = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
	   event->gyro.x = ((float) ((int16_t)event->gyro.x) * mGyroBuffScale);
	   event->gyro.y = ((float) ((int16_t)event->gyro.y) * mGyroBuffScale);
	   event->gyro.z = ((float) ((int16_t)event->gyro.z) * mGyroBuffScale);
   }
}

/**
 * @brief baching and for formatting of buffered data.
 *
 * Function for converting buffered data to SLIM service sepecific format
 * Read queue until next EV_SYN , this will fill only newly received data
 * if some fields of sample are not received will maintain older data
 *
 * @param[in/out] event - input previous data, output new data,
 * if some fields are not received will hold previous data.
 * @param[in] fd - pointer to file descritor
 * @param[in] type - sensorType
 *
 * @return void.
 */
bool SensorApiService::getBufferedSample(int type, FILE* fd, sensors_event_t *event) {

  struct input_event   ev[1];
  static int acc_second = 0;
  static int gyro_second = 0;
  bool retVal = false;
  /* Read queue until next EV_SYN */
  while(1) {
    // Read queue until next EV_SYN
    if(0 == fread(ev,1,sizeof(struct input_event),fd)) {
       return false;
       break;
    }
    else {
       //return a sample if read item is EV_SYN
       if (ev[0].type == EV_SYN) {
	       if(ev[0].value != 0xFFFFFFFF) {
		       return true;
		       break;
	       }
	       else {
		       return false;
		       break;
	       }
       }
       else {
	   if (SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED == type) {
		   if (ev[0].code == ABS_X)
			   event->acceleration.x = ev[0].value;
		   else if (ev[0].code == ABS_Y)
			   event->acceleration.y = ev[0].value;
		   else if (ev[0].code == ABS_Z)
			   event->acceleration.z = ev[0].value;
		   else if (ev[0].code == ABS_RX)
			   acc_second = ev[0].value; // how to extract time
		   else if (ev[0].code == ABS_RY) ////nano seconds
			   event->timestamp = (int64_t)((acc_second*1000000000LL) + ev[0].value);
	   }
	   if (SENSOR_TYPE_GYROSCOPE_UNCALIBRATED == type) {
		   if (ev[0].code == ABS_X)
			   event->gyro.x = ev[0].value;
		   else if (ev[0].code == ABS_Y)
			   event->gyro.y = ev[0].value;
		   else if (ev[0].code == ABS_Z)
			   event->gyro.z = ev[0].value;
		   else if (ev[0].code == ABS_RX)
			   gyro_second = ev[0].value; // how to extract time
		   else if (ev[0].code  == ABS_RY) ////nano seconds
		 	   event->timestamp = (int64_t)((gyro_second*1000000000LL) + ev[0].value);
	   }
       }
    }
  }
  return retVal;
}


bool SensorApiService::readSensorBufferData(const string clientname) {
  // Init sysFs files for both ACCEL & GYRO buffered data
  bool accelBuffDataTxProgress = false;
  bool gyroBuffDataTxProgress = false;
  bool rc = false;
  int acccount = 0;
  int gyrocount = 0;
  int count = 0;
  bool enable = true;
  sensors_event_t events[60];
  sensors_event_t zevents[2];

  unordered_map<string, SensorHalDaemonClientHandler*>::iterator it = mClients.find(clientname);

  accelBuffDataTxProgress = true;
  gyroBuffDataTxProgress = true;

  if(mfdBuffAccel == nullptr || mfdBuffGyro == nullptr) {
      writeToBufferFile(enable);
  }

  //start reading kernel buffered data;
  // Read and process all buffered data from sysfs
  // Before reading and processing buffered data make sure client is available to receive data
  // interleave the data while transferring to client aplication as per batching size
  while(accelBuffDataTxProgress || gyroBuffDataTxProgress)
  {
     /* Fill the accel buffered data from kernel bufer */
     if (accelBuffDataTxProgress) {
	     if(getBufferedSample(SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED, mfdBuffAccel, &zevents[0])) {
		     memcpy(&events[count], &zevents[0], sizeof(sensors_event_t));
		     bufferDataScaling(SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED, &events[count]);
		     SENSOR_LOGV(LOG_TAG "ACC Buffer event: x=%f y=%f z=%f timestamp=%lld acccount %d\n",
				     events[count].acceleration.x, events[count].acceleration.y,
				     events[count].acceleration.z, events[count].timestamp, acccount);
         (void)mDiagLogger.SendSensorBuffAccelEvent(&events[count], acccount);
         ++acccount;
		     count++;
	     } else {
		     accelBuffDataTxProgress = false;
		     SENSOR_LOGV(LOG_TAG "End ACCEL data acccount %d\n",acccount);
	     }
     }
     /* Fill the gyro buffered data into from kernel buffer */
     if (gyroBuffDataTxProgress) {
	     if (getBufferedSample(SENSOR_TYPE_GYROSCOPE_UNCALIBRATED, mfdBuffGyro, &zevents[1])) {
		     memcpy(&events[count], &zevents[1], sizeof(sensors_event_t));
		     bufferDataScaling(SENSOR_TYPE_GYROSCOPE_UNCALIBRATED, &events[count]);
		     SENSOR_LOGV(LOG_TAG "GYRO Buffer event: x=%f y=%f z=%f timestamp=%lld gyrocount %d\n",
				     events[count].gyro.x, events[count].gyro.y, events[count].gyro.z,
				     events[count].timestamp, gyrocount);
         (void)mDiagLogger.SendSensorBuffGyroEvent(&events[count], gyrocount);
         ++gyrocount;
		     count++;
	     } else {
		     gyroBuffDataTxProgress = false;
		     SENSOR_LOGV(LOG_TAG "END GYRO data gyrocount %d\n", gyrocount);
	     }
     }
     if (count >= 50) {
	     (void)usleep(1*1000);
	     rc = it->second->onSensorBufferDataReadCb(events, count);
	     (void)mDiagLogger.CommitToDiagAccelBuff();
	     (void)mDiagLogger.CommitToDiagGyroBuff();
	     // purge this client if failed
	     if (!rc) {
		     return rc;
	     }
	     (void)usleep(1*1000);
	     count = 0;
     }
  }
  SENSOR_LOGV(LOG_TAG "End of buffer data acccount %d gyrocount %d remaining packets %d\n",acccount, gyrocount, count);
  /***Send remainging packets**/
  if (count != 0) {
	  rc = it->second->onSensorBufferDataReadCb(events, count);
	  (void)mDiagLogger.CommitToDiagAccelBuff();
	  (void)mDiagLogger.CommitToDiagGyroBuff();
	  // purge this client if failed
	  if (!rc) {
		  return rc;
	  }
  }
  /***Send BUFFER END PACKET*/
  (void)memset(&events[0], 0, sizeof(sensors_event_t));
  events[0].type = SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED;
  events[0].timestamp = 0xFFFFFFFF;
  it->second->mBufferRead = false;
  if(acccount == 0 && gyrocount == 0)
      mBufferDeleted = true;

  rc = it->second->onSensorBufferDataReadCb(&events[0], 1);
  // purge this client if failed
  if (!rc) {
	  return rc;
  }
fail:
  CLOSE_FILE_HANDLE(mfdBuffAccel);
  CLOSE_FILE_HANDLE(mfdBuffGyro);
  return rc;
}

void SensorApiService::sensorBufferReadThread() {
  bool rc = false;
  while(mBufferSupported) {
    (void)pthread_mutex_lock (&mHalBuffMutex);
    (void)pthread_cond_wait (&mHalBuffCond, &mHalBuffMutex);
    (void)pthread_mutex_unlock (&mHalBuffMutex);

    auto it = mClients.begin();
    while (it != mClients.end() && it != (unordered_map<string, SensorHalDaemonClientHandler*>::iterator)NULL) {
	    if (it->second->mBufferRead == true && mBufferDeleted != true) {
		    rc = readSensorBufferData(it->first.c_str());
		    // purge this client if failed
		    if(!rc) {
			    SENSOR_LOGE(LOG_TAG "failed rc=%d purging client=%s\n", rc, it->first.c_str());
			    lock_guard<mutex> lock(SensorApiService::mMutex);
			    it =deleteClientbyName(it->first.c_str());
		    } else{
			    ++it;
		    }
	    } else{
		    ++it;
	    }
    }
  }
  (void)pthread_mutex_destroy (&mHalBuffMutex);
  (void)pthread_cond_destroy (&mHalBuffCond);
  SENSOR_LOGI(LOG_TAG "Exiting bufferDataprocessTask.. \n");
}
