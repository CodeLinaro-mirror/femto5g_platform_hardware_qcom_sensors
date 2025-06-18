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

#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <filesystem>
#include <SensorLog.h>
#include <SensorDevice.h>

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "SensorDevice:"

SensorDevice::SensorInfo initASM330() {
   SensorDevice::SensorInfo sensor = {
	   "ASM330",
	   "asm330lhh_accel",
	   "asm330lhh_gyro",
	   "asm330lhh_temp",
	   "in_accel_x_scale",
	   "in_anglvel_x_scale",
	   "selftest",
	   {},
	   "/usr/lib/libasm330sensors.so.1.0.0",
	   {13, 26, 52, 104, 208, 416},
	   {13, 26, 52, 104, 208, 416},
	   {{2, {0.000598205, 0.000598205}}, {4,{0.001196411,0.001196411}}, {8,{0.002392822,0.002392822}}, {16, {0.004785645,0.004785645}}},
	   {{125,{0.000076271, 0.000076271}}, {250,{0.000152716, 0.000152716}}, {500, {0.000305432, 0.000305432}}, {1000, {0.000610865,0.000610865}}, {2000, {0.001221729,0.001221729}}, {4000, {0.002443459,0.002443459}}},
	   4,
	   125,
	   3,
	   false
   };
   sensor.temp_files.push_back(SensorDevice::tempPtr("in_temp_scale"));
   sensor.temp_files.push_back(SensorDevice::tempPtr("in_temp_offset"));
   sensor.temp_files.push_back(SensorDevice::tempPtr("in_temp_raw"));
   return sensor;
}

SensorDevice::SensorInfo initIAM20680() {
   SensorDevice::SensorInfo sensor = {
	   "IAM20680",
	   "iam20680",
	   "iam20680",
	   "iam20680",
	   "in_accel_x_scale",
	   "in_anglvel_x_scale",
	   "misc_self_test",
	   {},
	   "/usr/lib/libiam20680sensors.so.1.0.0",
	   {6.25, 12.5, 25, 50, 100, 200},
	   {6.25, 12.5, 25, 50, 100, 200},
	   {{2,{0.000598755, 2}}, {4, {0.00119751,4}}, {8, {0.00239502, 8}}, {16, {0.00479004, 16}}},
	   {{250,{0.000133158,250}}, {500, {0.000266316, 500}}, {1000, {0.000532632,1000}}, {2000, {0.00106526,2000}}},
	   4,
	   250,
	   1,
	   false
   };
   sensor.temp_files.push_back(SensorDevice::tempPtr("out_temperature"));
   return sensor;
}

SensorDevice::SensorInfo initSMI230() {
   SensorDevice::SensorInfo sensor = {
            "SMI230",
	    "SMI230ACC",
	    "SMI230GYRO",
	    "SMI230ACC",
	    "range",
	    "range",
	    "self_test",
	    {},
	    "/usr/lib/libsmi230sensors.so.1.0.0",
	    {12.5, 25, 50, 100, 200, 400},
	    {100, 200, 400},
	    {{2,{0.000598755,2}}, {4,{0.001197510,4}}, {8,{0.002395020,8}}, {16,{0.004790039,16}}},
	    {{125,{0.00006657903,125}}, {250,{0.00013315805,250}}, {500,{0.00026631611,500}}, {1000,{0.00053263222,1000}}, {2000,{0.00106526444,2000}}},
	    4,
	    125,
	    1,
	    true
   };
   sensor.temp_files.push_back(SensorDevice::tempPtr("temp"));
   return sensor;
}


SensorDevice::SensorDevice(SensorApiService* service) :
   mService(service),
   mSensorType(SENSOR_UNKNOWN),
   mAccel(""),
   mGyro(""),
   mTemp("")
{
   SENSOR_LOGI(LOG_TAG "Sensor Device Constructor\n");
   mSensorInfo[SENSOR_ASM330].push_back(initASM330());
   mSensorInfo[SENSOR_IAM20680].push_back(initIAM20680());
   mSensorInfo[SENSOR_SMI230].push_back(initSMI230());
}

SensorDevice::~SensorDevice() {
   for (auto& sensorPair : mSensorInfo) {
	for (auto& sensorInfo : sensorPair.second) {
	   for (auto& tempfilePtr : sensorInfo.temp_files) {
	       if (tempfilePtr.tempfile) {
		       delete tempfilePtr.tempfile;
		       tempfilePtr.tempfile = nullptr;
	       }
	   }
	}
   }
   if(mSensorType == SENSOR_SMI230){
	if (mService && mService->mpoll_dev_v0) {
		mService->mpoll_dev_v0->common.close(&mService->mpoll_dev_v0->common);
	}
   }
}

bool SensorDevice::openSensorDevice(string *libname) {
   int dev_num = -1;
   for (auto& [type, sensors] : mSensorInfo) {
    for (auto& sensor : sensors) {
     SENSOR_LOGD("Checking Sensor %s availability\n", sensor.accel_name);
     dev_num = sensor.is_input ? get_input_sensor_device_by_name(sensor.accel_name) : get_iio_sensor_device_by_name(sensor.accel_name);

     // If not found and it's asm330lhh_accel, try fallback
     if (dev_num < 0 && strncmp(sensor.accel_name, "asm330lhh_accel", strlen("asm330lhh_accel")) == 0) {
         SENSOR_LOGI("Sensor %s not found, retrying with asm330lhhx variant...\n", sensor.accel_name);
         sensor.accel_name = "asm330lhhx_accel";
         sensor.gyro_name = "asm330lhhx_gyro";
         sensor.temp_name = "asm330lhhx_temp";

	 dev_num = sensor.is_input ? get_input_sensor_device_by_name(sensor.accel_name) : get_iio_sensor_device_by_name(sensor.accel_name);
     }

     if (dev_num >= 0) {
         SENSOR_LOGI("Sensor %s found into %s\n", sensor.accel_name, sensor.is_input ?"/sys/devices/virtual/input/" : "/sys/bus/iio/devices/");
	 mAccel = (sensor.is_input ? "/sys/devices/virtual/input/input" : "/sys/bus/iio/devices/iio:device") + to_string(dev_num) + "/";
	 dev_num = sensor.is_input ? get_input_sensor_device_by_name(sensor.gyro_name) : get_iio_sensor_device_by_name(sensor.gyro_name);
	 if (dev_num >= 0) {
	     SENSOR_LOGI("Sensor %s found into %s\n", sensor.gyro_name, sensor.is_input ? "/sys/devices/virtual/input/" : "/sys/bus/iio/devices/");
	     mGyro = (sensor.is_input ? "/sys/devices/virtual/input/input" : "/sys/bus/iio/devices/iio:device") + to_string(dev_num) + "/";
	 }
	 dev_num = sensor.is_input ? get_input_sensor_device_by_name(sensor.temp_name) : get_iio_sensor_device_by_name(sensor.temp_name);
	 if (dev_num >= 0) {
	     SENSOR_LOGI("Sensor %s found into %s\n", sensor.temp_name, sensor.is_input ? "/sys/devices/virtual/input/" : "/sys/bus/iio/devices/");
	     mTemp = (sensor.is_input ? "/sys/devices/virtual/input/input" : "/sys/bus/iio/devices/iio:device") + to_string(dev_num) + "/";
	 }
	 mSensorType = type;
	 *libname = sensor.lib_name;  // Assign libname here*/
	 SENSOR_LOGI("mSensorType %d \n \
		     libname %s \n \
		     mAccel %s \n \
		     mGyro %s \n \
		     mTemp %s\n",
		     mSensorType, sensor.lib_name, mAccel.c_str(), mGyro.c_str(), mTemp.c_str());
	 return true;
     }
    }
   }
   return false;
}

void SensorDevice::getSensorDeviceName(string *sensorName) {
   auto sensor = mSensorInfo.find(mSensorType);
   if (sensor != mSensorInfo.end()) {
        *sensorName = sensor->second[0].chip_name;
   } else {
	   *sensorName = "";
   }
}

int SensorDevice::initMaxRange(int type, int sensor_id) {
  string rangeFile;
  float scale_value = 0, range = 0;
  float epsilon = std::numeric_limits<float>::epsilon();

  auto sensor = mSensorInfo.find(mSensorType);
  if (sensor != mSensorInfo.end()) {
     // Limit AccRangeIndex to valid bounds
     if (mService->mAccRange < 0) {
	     mService->mAccRange = 0;
     } else if (mService->mAccRange >= sensor->second[0].accel_range_scale_map.size()) {
	     mService->mAccRange = sensor->second[0].accel_range_scale_map.size() - 1;
     }
     // Limit GyroRangeIndex to valid bounds
     if (mService->mGyroRange < 0) {
	     mService->mGyroRange = 0;
     } else if (mService->mGyroRange >= sensor->second[0].gyro_range_scale_map.size()) {
	     mService->mGyroRange = sensor->second[0].gyro_range_scale_map.size() - 1;
     }
     if (type == SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED) {
	    rangeFile = mAccel + "/" + sensor->second[0].accel_range_name;
	    SENSOR_LOGI(LOG_TAG "rangeFile %s \n", rangeFile.c_str());
	    // Use AccRange as the index to get the third value in the map
	    auto it = next(sensor->second[0].accel_range_scale_map.begin(), mService->mAccRange);
	    if (it != sensor->second[0].accel_range_scale_map.end()) {
		    scale_value = std::get<1>(it->second);
		    sysfs_write_scale(rangeFile.c_str(), scale_value);
		    SENSOR_LOGI(LOG_TAG "Accel scale_value written %d %f %f\n", mService->mAccRange,
				    std::get<0>(it->second), scale_value);
		    sysfs_read_scale(rangeFile.c_str(), &scale_value);
		    SENSOR_LOGI(LOG_TAG "Accel scale_value read %f\n", scale_value);
		    // Find the corresponding key for the read scale value
		    for (const auto& entry : sensor->second[0].accel_range_scale_map) {
			    if (std::fabs(scale_value - std::get<1>(entry.second)) < epsilon) {
				    range = entry.first;
				    SENSOR_LOGI(LOG_TAG "Accel range: %f\n", range);
			    }
		    }
	    }
     }
     if (type == SENSOR_TYPE_GYROSCOPE_UNCALIBRATED){
	    rangeFile = mGyro + "/" + sensor->second[0].gyro_range_name;
	    SENSOR_LOGI(LOG_TAG "rangeFile %s \n", rangeFile.c_str());
	    // Use GyroRange as the index to get the third value in the map
	    auto it = next(sensor->second[0].gyro_range_scale_map.begin(), mService->mGyroRange);
	    if (it != sensor->second[0].gyro_range_scale_map.end()) {
		    scale_value = std::get<1>(it->second);
	            if (mSensorType == SENSOR_SMI230) mService->sensorActivate(sensor_id, SENSOR_ENABLE);
		    sysfs_write_scale(rangeFile.c_str(), scale_value);
		    SENSOR_LOGI(LOG_TAG "Gyro scale_value written %d %f %f\n", mService->mGyroRange,
				    std::get<0>(it->second), scale_value);
		    sysfs_read_scale(rangeFile.c_str(), &scale_value);
		    SENSOR_LOGI(LOG_TAG "Gyro scale_value read %f\n", scale_value);
	            if (mSensorType == SENSOR_SMI230) mService->sensorActivate(sensor_id, SENSOR_DISABLE);
		    // Find the corresponding key for the read scale value
		    for (const auto& entry : sensor->second[0].gyro_range_scale_map) {
			    if (std::fabs(scale_value - std::get<1>(entry.second)) < epsilon) {
				    range = entry.first;
				    SENSOR_LOGI(LOG_TAG "Gyro range: %f\n", range);
			    }
		    }
	    }
     }
  }
  return range;
}

/*
 * SensorDevice - implementation - GetSupportedSamplingRate
 * Sampling rate supported by each sensor
 *
*/
void SensorDevice::getSupportedSamplingRateAndRange(struct sensor_list *s) {
  auto sensor = mSensorInfo.find(mSensorType);
  if (sensor != mSensorInfo.end()) {
     if (s->type == SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED) {
	 fill(s->odr, s->odr + MAX_ODR, 0);
	 copy(sensor->second[0].accel_odr.begin(), sensor->second[0].accel_odr.begin() + min(sensor->second[0].accel_odr.size(), static_cast<size_t>(MAX_ODR)), s->odr);
         mService->mMaxAccSampleRate  = mService->nearBySamplingRate(s->odr, mService->mMaxAccSampleRate);
	 if (mService->mMinAccBatchCount >= MAX_BATCH_COUNT)
		 mService->mMinAccBatchCount = MAX_BATCH_COUNT;
	 else if (mService->mMinAccBatchCount <= 0)
		 mService->mMinAccBatchCount = 1;

	 if (mSensorType == SENSOR_SMI230) {
	    //Adjusting batch rate to reduce the irq freq at higher rate
	    if (mService->mMaxGyroSampleRate == 100 && mService->mMinGyroBatchCount < 2)
		    mService->mMinGyroBatchCount = 2;
	    else if (mService->mMaxAccSampleRate == 200 && mService->mMinAccBatchCount < 4)
		    mService->mMinAccBatchCount = 4;
	    else if (mService->mMaxAccSampleRate == 400 && mService->mMinAccBatchCount < 8)
		    mService->mMinAccBatchCount = 8;
	 }

	 s->maxSamplingRate = mService->mMaxAccSampleRate;
	 s->range = initMaxRange(s->type, s->sensor_id);
	 SENSOR_LOGI("Accel Range %d\n", s->range);
	 s->minBatchCount   = mService->mMinAccBatchCount;
     }
     if (s->type == SENSOR_TYPE_GYROSCOPE_UNCALIBRATED){
	 fill(s->odr, s->odr + MAX_ODR, 0);
	 copy(sensor->second[0].gyro_odr.begin(), sensor->second[0].gyro_odr.begin() + min(sensor->second[0].gyro_odr.size(), static_cast<size_t>(MAX_ODR)), s->odr);
	 mService->mMaxGyroSampleRate = mService->nearBySamplingRate(s->odr, mService->mMaxGyroSampleRate);
	 if (mService->mMinGyroBatchCount >= MAX_BATCH_COUNT)
		 mService->mMinGyroBatchCount = MAX_BATCH_COUNT;
	 else if (mService->mMinGyroBatchCount <= 0)
		 mService->mMinGyroBatchCount = 1;

	 if (mSensorType == SENSOR_SMI230) {
	    //Adjusting batch rate to reduce the irq freq at higher rate
	    if (mService->mMaxGyroSampleRate == 100 && mService->mMinGyroBatchCount < 2)
		    mService->mMinGyroBatchCount = 2;
	    else if (mService->mMaxGyroSampleRate == 200 && mService->mMinGyroBatchCount < 4)
		    mService->mMinGyroBatchCount = 4;
	    else if (mService->mMaxGyroSampleRate == 400 && mService->mMinGyroBatchCount < 8)
		    mService->mMinGyroBatchCount = 8;
	 }

	 //update sensor sampling rate, range and batch count supported to list
	 s->maxSamplingRate = mService->mMaxGyroSampleRate;
	 s->range = initMaxRange(s->type, s->sensor_id);
	 SENSOR_LOGI("Gyro Range %d\n", s->range);
	 s->minBatchCount   = mService->mMinGyroBatchCount;
     }
     mService->mBatchConst =  sensor->second[0].batch_const;
  }
}

bool SensorDevice::getSensorBufferFile(string *acc_name, string *gyro_name) {
  *acc_name = mAccel + "read_acc_boot_sample";
  *gyro_name = mGyro + "read_gyro_boot_sample";
  // Check if the files exist
  if (access(acc_name->c_str(), F_OK) != -1 && access(gyro_name->c_str(), F_OK) != -1)
	  return true;
  else
	  return false;
}

void SensorDevice::getSensorBufferScalarValue(float *accel_buff_scale, float *gyro_buff_scale) {
  auto sensor = mSensorInfo.find(mSensorType);
  if (sensor != mSensorInfo.end()) {
	  auto accel_it = sensor->second[0].accel_range_scale_map.find(static_cast<int>(sensor->second[0].accel_buff_range));
	  if (accel_it != sensor->second[0].accel_range_scale_map.end()) {
		  *accel_buff_scale = std::get<0>(accel_it->second);
	  } else {
		  *accel_buff_scale = 0.0; // Default value if not found
	  }

	  auto gyro_it = sensor->second[0].gyro_range_scale_map.find(static_cast<int>(sensor->second[0].gyro_buff_range));
	  if (gyro_it != sensor->second[0].gyro_range_scale_map.end()) {
		  *gyro_buff_scale = std::get<0>(gyro_it->second);
	  } else {
		  *gyro_buff_scale = 0.0; // Default value if not found
	  }
  } else {
	  *accel_buff_scale = 0.0;
	  *gyro_buff_scale = 0.0;
  }
  SENSOR_LOGI(LOG_TAG "accel_buff_scale %f gyro_buff_scale %f \n", *accel_buff_scale, *gyro_buff_scale);
}

bool SensorDevice::checkTempSupport() {
  auto sensor = mSensorInfo.find(mSensorType);
  if (sensor == mSensorInfo.end()) {
	  SENSOR_LOGE(LOG_TAG "Unsupported sensor type\n");
	  return false;
  }
  auto& sensorInfoVector = sensor->second;
  for (auto& sensorinfo : sensorInfoVector) {
	  for (auto& filePtr : sensorinfo.temp_files) {
		  string temperaturefile = mTemp + "/" + filePtr.tempfilePath;
		  SENSOR_LOGI(LOG_TAG "temperaturefile %s\n", temperaturefile.c_str());
		  filePtr.tempfile = new ifstream(temperaturefile);
		  if (!filePtr.tempfile->is_open()) {
			  SENSOR_LOGE(LOG_TAG "Failed to open temperature file: %s\n", temperaturefile.c_str());
			  delete filePtr.tempfile;
			  filePtr.tempfile = nullptr;
			  return false;
		  }
	  }
  }
  return true;
}

int SensorDevice::readSensorTemperature(float *temperature) {
  auto sensor = mSensorInfo.find(mSensorType);
  if (sensor == mSensorInfo.end()) {
	  SENSOR_LOGE(LOG_TAG "Unsupported sensor type\n");
	  return -1;
  }
  const auto& sensorInfoVector = sensor->second;
  for (const auto& sensorInfo : sensorInfoVector) {
	  if (mSensorType == SENSOR_ASM330) {
		  float scale = 0.0f, offset = 0.0f, raw = 0.0f;
		  for (const auto& tempPtr : sensorInfo.temp_files) {
			  if (tempPtr.tempfilePath == "in_temp_scale") {
				  tempPtr.tempfile->clear(); // Clear any error flags
				  tempPtr.tempfile->seekg(0); // Reset file pointer to the beginning
				  *tempPtr.tempfile >> scale;
			  } else if (tempPtr.tempfilePath == "in_temp_offset") {
				  tempPtr.tempfile->clear(); // Clear any error flags
				  tempPtr.tempfile->seekg(0); // Reset file pointer to the beginning
				  *tempPtr.tempfile >> offset;
			  } else if (tempPtr.tempfilePath == "in_temp_raw") {
				  tempPtr.tempfile->clear(); // Clear any error flags
				  tempPtr.tempfile->seekg(0); // Reset file pointer to the beginning
				  *tempPtr.tempfile >> raw;
			  }
		  }
		  *temperature = (float) (raw + offset) * (scale/1000);
		  SENSOR_LOGI(LOG_TAG "ASM330 Temperature: %f\n", *temperature);
		  return 0;
	  } else if (mSensorType == SENSOR_IAM20680) {
		  float temp = 0.0f;
		  for (const auto& tempPtr : sensorInfo.temp_files) {
			  tempPtr.tempfile->clear(); // Clear any error flags
			  tempPtr.tempfile->seekg(0); // Reset file pointer to the beginning
			  *tempPtr.tempfile >> temp;
		  }
		  *temperature = temp / 100.0f;
		  SENSOR_LOGI(LOG_TAG "IAM20680 Temperature: %f\n", *temperature);
		  return 0;
	  } else if (mSensorType == SENSOR_SMI230) {
		  float temp = 0.0f;
		  for (const auto& tempPtr : sensorInfo.temp_files) {
			  tempPtr.tempfile->clear(); // Clear any error flags
			  tempPtr.tempfile->seekg(0); // Reset file pointer to the beginning
			  *tempPtr.tempfile >> temp;
		  }
		  *temperature = temp / 1000.0f; // Convert to Degree Celsius
		  SENSOR_LOGI(LOG_TAG "SMI230 Temperature: %f\n", *temperature);
		  return 0;
	  }
  }
  SENSOR_LOGE(LOG_TAG "Temperature support not found for sensor type\n");
  return -1;
}

bool SensorDevice::sensorDevSelfTest(int sensor_id, int type, SelfTestType selfTestType,
		SelfTestResult *selfTest, uint64_t *selfTestTS) {
   FILE *self_test_fd = NULL;
   uint64_t selftest_time = 0 , elapsed_time = 0;
   char buffer_string[DEVICE_MAX_FILENAME_LEN] = {'\0'};
   string self_test_file_name;
   SelfTestResult result = NotAvailable;

   auto sensor = mSensorInfo.find(mSensorType);
   if (sensor != mSensorInfo.end()) {
       if (type == SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED)
	       self_test_file_name = mAccel + "/" + sensor->second[0].selftest_name;
       else if (type == SENSOR_TYPE_GYROSCOPE_UNCALIBRATED)
	       self_test_file_name = mGyro + "/" + sensor->second[0].selftest_name;
       SENSOR_LOGI(LOG_TAG "self test file name %s\n", self_test_file_name.c_str());
   }

   //Run self test for each as proces varies from sensor to sensor
   switch(mSensorType) {
      case SENSOR_ASM330:  {
		mService->sensorActivate(sensor_id, SENSOR_DISABLE); //Disable the sensor
		selftest_time = get_timestamp();
		FILE *self_test_fd = fopen(self_test_file_name.c_str(), "w+");
		if (self_test_fd == NULL) {
			SENSOR_LOGE(LOG_TAG "NULL");
			result = Failed;
		}
		if (selfTestType == Positive || selfTestType == All)  {
			int ret = fprintf(self_test_fd, "%s", "positive-sign");
			if (ret < 0) {
				SENSOR_LOGE(LOG_TAG "Failed to write to self test file");
				result = Failed;
			}
			rewind(self_test_fd);
			fgets(buffer_string, sizeof(buffer_string), self_test_fd);
			SENSOR_LOGI(LOG_TAG "buffer_string = %s ", buffer_string);
			if(strstr(buffer_string, "pass"))
				result = Passed;
			else
				result = Failed;
		}
		if (selfTestType == Negative || selfTestType == All)  {
			int ret = fprintf(self_test_fd, "%s", "negative-sign");
			if (ret < 0) {
				SENSOR_LOGE(LOG_TAG "Failed to write to self test file");
				result = Failed;
			}
			rewind(self_test_fd);
			fgets(buffer_string, sizeof(buffer_string), self_test_fd);
			SENSOR_LOGI(LOG_TAG "buffer_string = %s ", buffer_string);
			if(strstr(buffer_string, "pass")) {
			    if (selfTestType == All && result == Passed)
				// Both tests passed
				result = Passed;
			    else if (selfTestType != All)
				result = Passed;
			}
			else {
				result = Failed;
			}
		}
		fclose(self_test_fd);
		elapsed_time = get_timestamp() - selftest_time;
		SENSOR_LOGI(LOG_TAG "SelfTest time for sensor id %d: %lldms\n", sensor_id, NS_TO_MS(elapsed_time));
	        *selfTest = result;
		*selfTestTS = get_timestamp();
		break;
       }
       case SENSOR_SMI230:  {
		mService->sensorActivate(sensor_id, SENSOR_ENABLE); //Enable the sensor
		selftest_time = get_timestamp();
		FILE *self_test_fd = fopen(self_test_file_name.c_str(), "r");
		if (self_test_fd == NULL) {
			SENSOR_LOGE(LOG_TAG "NULL");
			result = Failed;
		}
		rewind(self_test_fd);
		fgets(buffer_string, sizeof(buffer_string), self_test_fd);
		if(strstr(buffer_string, "self test success")){
			SENSOR_LOGI(LOG_TAG "SMI230 self test is passed buffer_string:%s\n", buffer_string);
			result = Passed;
		} else {
			SENSOR_LOGE(LOG_TAG "SMI230 self test is failed buffer_string:%s\n", buffer_string);
			result = Failed;
		}
		fclose(self_test_fd);
		elapsed_time = get_timestamp() - selftest_time;
		SENSOR_LOGI(LOG_TAG "SelfTest time for sensor id %d: %lldms\n", sensor_id, NS_TO_MS(elapsed_time));
		*selfTest = result;
		*selfTestTS = get_timestamp();
		mService->sensorActivate(sensor_id, SENSOR_DISABLE); //Disable the sensor
		break;
	}
        case SENSOR_IAM20680: {
                selftest_time = get_timestamp();
		for(int i = 0 ; i < mService->mSensorCount; i++)  {
			if (mService->mSensor[i].Activate == SENSOR_ENABLE)
				return false;
		}
                SENSOR_LOGI(LOG_TAG "self_test file name %s\n", self_test_file_name.c_str());
                FILE *self_test_fd = fopen(self_test_file_name.c_str(), "r");
                if (self_test_fd == NULL) {
                        SENSOR_LOGE(LOG_TAG "NULL");
                        result = Failed;
                }
                rewind(self_test_fd);
                if(fgets(buffer_string, sizeof(buffer_string), self_test_fd) != NULL) {
                   if(type == SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED && strstr(buffer_string, "3") || strstr(buffer_string, "2")) {
			   SENSOR_LOGI(LOG_TAG "IAM20680 Accel self test is passed buffer_string:%s\n", buffer_string);
			   result = Passed;
                   } else if (type == SENSOR_TYPE_GYROSCOPE_UNCALIBRATED && strstr(buffer_string, "3") || strstr(buffer_string, "1")) {
			   SENSOR_LOGI(LOG_TAG "IAM20680 Gyro self test is passed buffer_string:%s\n", buffer_string);
			   result = Passed;
		   } else {
			   SENSOR_LOGE(LOG_TAG "IAM20680 self test is failed buffer_string:%s\n", buffer_string);
			   result = Failed;
		   }
                }
                fclose(self_test_fd);
                elapsed_time = get_timestamp() - selftest_time;
                SENSOR_LOGI(LOG_TAG "SelfTest time for sensor id %d: %lldms\n", sensor_id, NS_TO_MS(elapsed_time));
                *selfTest = result;
                *selfTestTS = get_timestamp();
	        break;
	}
	default: {
	        SENSOR_LOGE(LOG_TAG "Self Test not supported for this sensor type- %d\n", mSensorType);
		break;
	}
   }
   return true;
}

void SensorDevice::setDefaultFIRCoeff()
{
   	switch (mSensorType) {
	   case SENSOR_SMI230: {
			mDefaultAccelCoef = {
				{400, {
						{200, {1.0/2, 1.0/2}},
						{100, {1.0/12, 3.0/12, 4.0/12, 3.0/12, 1.0/12}}
					}},
				{200, {
						{100, {1.0/2, 1.0/2}}
					}}
			};
			mDefaultGyroCoef = {
				{400, {
						{200, {1.0/12, 3.0/12, 4.0/12, 3.0/12, 1.0/12}},
						{100, {1.0/45, 3.0/45, 6.0/45, 8.0/45, 9.0/45, 8.0/45, 6.0/45, 3.0/45, 1.0/45}}
					}},
				{200, {
						{100, {1.0/12, 3.0/12, 4.0/12, 3.0/12, 1.0/12}}
					}}
			};
			break;
		}
		default: {
			mDefaultAccelCoef = {};
			mDefaultGyroCoef = {};
			SENSOR_LOGI(LOG_TAG "No default FIR coefficient defined for sensor %d\n", mSensorType);
			break;
		}
   }
   return;
}

int SensorDevice::getDefaultFIRCoeff(bool is_accel, int sensor_rate, int client_rate, vector<float> &out_coef)
{
   const default_fir_coef_t &default_coef = is_accel ? mDefaultAccelCoef : mDefaultGyroCoef;
   default_fir_coef_t::const_iterator it = default_coef.find(sensor_rate);
   if (it != default_coef.end())
   {
      for (const pair<int, vector<float>> &coef : it->second)
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
