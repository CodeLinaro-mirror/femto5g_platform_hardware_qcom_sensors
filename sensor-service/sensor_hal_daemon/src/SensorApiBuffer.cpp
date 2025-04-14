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
 * Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
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
#define ACC_BUFFER_READ         "read_acc_boot_sample"
#define GYRO_BUFFER_READ        "read_gyro_boot_sample"
#define ACCNAME_BUFF_PATH       "/dev/input/accbuff"
#define GYRNAME_BUFF_PATH       "/dev/input/gyrobuff"

/*ASM330 temperature*/
#define ASM_TEMP_SEARCH         "asm330lhh_temp"
#define ASMX_TEMP_SEARCH        "asm330lhhx_temp"
#define ASM_ACCEL_FSR           0.001196   // 2G:0.000598, 4G:0.001196, 8G:0.002392, 16G:0.004785
#define ASM_GYRO_FSR            0.000076   // 125:0.000076, 250:0.000153, 500:0.000305, 1000:0.000611, 2000:0
#define ASM_ACC_SEARCH          "asm330lhh_accel"
#define ASM_GYR_SEARCH          "asm330lhh_gyro"
#define ASMX_ACC_SEARCH         "asm330lhhx_accel"
#define ASMX_GYR_SEARCH         "asm330lhhx_gyro"
#define ASM_TEMP_SCALE          "in_temp_scale"
#define ASM_TEMP_OFFSET         "in_temp_offset"
#define ASM_TEMP_RAW            "in_temp_raw"
#define ASM_BATCH_TIME          (3.0 * NS_IN_ONE_SECOND)

/*SMI130  temperature*/
#define SMI_TEMP_SEARCH         "smi130_acc"
#define SMI_TEMP_NAME           "temp"
#define SMI_GYR_SEARCH          "smi130_gyro"
#define SMI_ACC_RESL            (0.97656f)
#define SMI_CONVERT_ACC         (0.0098) //library output is in mg = 0.0098 m/s^2
#define SMI_CONVERT_GYRO        (0.000066322)
#define SMI_BATCH_TIME          0

/*SMI230 temperature*/
#define SMI230_TEMP_SEARCH         "SMI230ACC"  // Search key is common for both Accel & Temp
#define SMI230_GYR_SEARCH          "SMI230GYRO" // Search key for Gyro
#define SMI230_CONVERT_ACC         (0.001197510) // 2G:0.000598755, 4G:0.001197510, 8G:0.002395020, 16G:0.004790039
#define SMI230_CONVERT_GYRO        (0.00006657903) //125:0.00006657903, 250:0.00013315805, 500:0.00026631611, 2000:0.00053263222

/*IAM20680 temperature and buffer read*/
#define IAM_TEMP_SEARCH         "iam20680"
#define IAM_TEMP_NAME           "out_temperature"
#define IAM_ACCEL_FSR           4.0f // 2:2g, 4:4g, 8:8g, 16:16g
#define IAM_GYRO_FSR            131.0f // 131:250dbps 65.5:500dbps 32.8:1000dbps 16.4:2000dbps
#define IAM_BATCH_TIME          (1.0 * NS_IN_ONE_SECOND)

/*BMI160 temperature*/
#define BMI_BATCH_TIME          0
#define BMI_TEMP_SEARCH         "bmi160_accl"
#define BMI_TEMP_NAME           "temperature"
#define BMI_ACC_RESL            (0.061)
#define BMI_CONVERT_ACC         (0.0098) //library output is in mg = 0.0098 m/s^2
#define BMI_CONVERT_GYRO        (0.000066322)

static FILE *mfdBuffAccel  = NULL;
static FILE *mfdBuffGyro   = NULL;



static float convert_acc_buff_asm = ASM_ACCEL_FSR;
static float convert_gyro_buff_asm = ASM_GYRO_FSR;

static float convert_acc_buff_iam = IAM_ACCEL_FSR;
static float convert_gyro_buff_iam = IAM_GYRO_FSR;

static float convert_acc_buff_smi130 = SMI_CONVERT_ACC;
static float convert_gyro_buff_smi130 = SMI_CONVERT_GYRO;

static float convert_acc_buff_smi230 = SMI230_CONVERT_ACC;
static float convert_gyro_buff_smi230 = SMI230_CONVERT_GYRO;


void set_buff_scaling_factor(int sensorType, int accRange, int gyroRange)
{
  if(sensorType == 1)
  {
    //asm
    float acc_range_conv[4] = {0.000598, 0.001196, 0.002392, 0.004785};
    float gyro_range_conv[6] = {0.000076, 0.000153, 0.000305, 0.000611, 0.001222, 0.002443};
    if(accRange >= 0 && accRange <= 3)
    {
      convert_acc_buff_asm = acc_range_conv[accRange];
    }
    if(gyroRange >= 0 && gyroRange <= 5)
    {
      convert_gyro_buff_asm = gyro_range_conv[gyroRange];
    }
    SENSOR_LOGI(LOG_TAG "convert_acc_buff_asm : %f, convert_gyro_buff_asm : %f\n", convert_acc_buff_asm, convert_gyro_buff_asm);
  }
  else if(sensorType == 2)
  {
    //iam
    float acc_range_conv[4] = {2.0, 4.0, 8.0, 16.0};
    float gyro_range_conv[4] = {131.0, 65.5, 32.8, 16.4};
    if(accRange >= 0 && accRange <= 3)
    {
      convert_acc_buff_iam = acc_range_conv[accRange];
    }
    if(gyroRange >= 0 && gyroRange <= 3)
    {
      convert_gyro_buff_iam = gyro_range_conv[gyroRange];
    }
    SENSOR_LOGI(LOG_TAG "convert_acc_buff_iam : %f, convert_gyro_buff_iam : %f\n", convert_acc_buff_iam, convert_gyro_buff_iam);
  }
  else if(sensorType == 3)
  {
    //smi130, only one range supported
    convert_acc_buff_smi130 = 0.0098;
    convert_gyro_buff_smi130 = 0.000066322;
    SENSOR_LOGI(LOG_TAG "convert_acc_buff_smi130 : %f, convert_gyro_buff_smi130 : %f\n", convert_acc_buff_smi130, convert_gyro_buff_smi130);
  }
  else if(sensorType == 4)
  {
    //smi230
    float acc_range_conv[4] = {0.000598755, 0.001197510, 0.002395020, 0.004790039};
    float gyro_range_conv[5] = {0.00006657903, 0.00013315805, 0.00026631611, 0.00053263222, 0.00106526444};
    if(accRange >= 0 && accRange <= 3)
    {
      convert_acc_buff_smi230 = acc_range_conv[accRange];
    }
    if(gyroRange >= 0 && gyroRange <= 4)
    {
      convert_gyro_buff_smi230 = gyro_range_conv[gyroRange];
    }
    SENSOR_LOGI(LOG_TAG "convert_acc_buff_smi230 : %f, convert_gyro_buff_smi230 : %f\n", convert_acc_buff_smi230, convert_gyro_buff_smi230);
  }
}



/**
 * @brief Read Temp Sensor data from SYS File System for ASM330 Sensor.
 *
 * Function for Reading Temp Sensor data from SYS File System
 * and send it to client.
 *
 * @return int.
 */
int SensorApiService::readTempASM(float *temperature)
{
  float tscale = 0;
  int toffset  = 0;
  int trawdata = 0;
  if ( NULL == mTempFilePtr.asmTempFile.tScaleFile ||
       NULL == mTempFilePtr.asmTempFile.tOffsetFile ||
       NULL == mTempFilePtr.asmTempFile.tRawDataFile)
  {
    return -1;
  }
  (void)mTempFilePtr.asmTempFile.tScaleFile->seekg(0);
  (void)mTempFilePtr.asmTempFile.tOffsetFile->seekg(0);
  (void)mTempFilePtr.asmTempFile.tRawDataFile->seekg(0);
  /* Read offset, scale and raw data */
  (*mTempFilePtr.asmTempFile.tScaleFile) >> tscale;
  (*mTempFilePtr.asmTempFile.tOffsetFile) >> toffset;
  if (mTempFilePtr.asmTempFile.tRawDataFile) {
    (*mTempFilePtr.asmTempFile.tRawDataFile) >> trawdata;
    SENSOR_LOGD(LOG_TAG "Read Raw:%d, Offset: %d, Scale: %f, Temperature: %f\n", trawdata, toffset, (tscale/1000), (trawdata + toffset) * (tscale/1000));
    *temperature = (float) (trawdata + toffset) * (tscale/1000);
  }
  return 0;
}
/**
 * @brief Read Temp Sensor data from SYS File System for IAM Sensor.
 *
 * Function for Reading Temp Sensor data from SYS File System
 * and send it to client.
 *
 * @return int.
*/
int SensorApiService::readTempIAM(float *temperature)
{
  int data;
  uint64_t timeStamp;
  if ( NULL == mTempFilePtr.iamTempFile.dataFile )
  {
    return 0;
  }
  (void)mTempFilePtr.iamTempFile.dataFile->seekg(0);

  if(mTempFilePtr.iamTempFile.dataFile->peek() == EOF)
  {
    SENSOR_LOGE(LOG_TAG "Temperature data not available in file");
    return 0;
  }
  (*mTempFilePtr.iamTempFile.dataFile)>>data;

  if(mTempFilePtr.iamTempFile.dataFile->peek() == EOF)
  {
    SENSOR_LOGE(LOG_TAG "Temperature data time not available in file");
    return 0;
  }
  (*mTempFilePtr.iamTempFile.dataFile)>>timeStamp;
  SENSOR_LOGD(LOG_TAG "Temp Raw values - %d,time base %lld",data,timeStamp);
  *temperature = (float)data / 100.0F; // Calculate Temperature Value
  return 0;
}


/**
 * @brief Read Temp Sensor data from SYS File System for BMI Sensor.
 *
 * Function for Reading Temp Sensor data from SYS File System
 * and send it to client.
 *
 * @return int.
 */
int SensorApiService::readTempBMI(float *temperature)
{
  int16_t tempSign = 0;
  unsigned int tempRead = 0;

  if ( NULL == mTempFilePtr.bmiTempFile.tTempFile)
  {
    return 0;
  }
  (void)mTempFilePtr.bmiTempFile.tTempFile->seekg(0);
  /* Read Temp data */
  (*mTempFilePtr.bmiTempFile.tTempFile)>>std::hex>>tempRead;

  tempSign = (int16_t)tempRead;
  /* Extract Sign bit */
  if(0x8000 == tempSign)
  {
    SENSOR_LOGE(LOG_TAG "Read Raw:%d, Temperature: Invalid \n", tempSign);
    return 0;
  }
  *temperature = 23 + ((float)tempSign / 512.0F); // Calculate Temperature Value
  SENSOR_LOGD(LOG_TAG "Read Raw:%x, Temperature: %f\n", tempSign, *temperature);
  return 0;
}

/**
 * @brief Read Temp Sensor data from SYS File System for SMI Sensor.
 *
 * Function for Reading Temp Sensor data from SYS File System
 * and send it to client.
 *
 * @return int.
 */
int SensorApiService::readTempSMI(float *temperature)
{

  int tempRead = 0;
  if ( NULL == mTempFilePtr.smiTempFile.tempFile )
  {
    return 0;
  }
  /* Read Temp data */
  (void)mTempFilePtr.smiTempFile.tempFile->seekg(0);

  if (mTempFilePtr.smiTempFile.tempFile->eof() )
  {
    SENSOR_LOGE(LOG_TAG "Temperature data not available in file");
    return 0;
  }
  (*mTempFilePtr.smiTempFile.tempFile)>>tempRead;
  SENSOR_LOGD(LOG_TAG "Read Raw:%d, \n", tempRead);

  if ( tempRead & 0x80 )
  {
    tempRead |= 0xFFFFFF80;
  }
  *temperature = 87.5 - ((float)(127 - tempRead)/2); // Calculate Temperature Value

  SENSOR_LOGD(LOG_TAG "Read Raw:%d, Temperature: %f\n", tempRead, *temperature);
  return 0;
}

/**
 * @brief Read Temp Sensor data from SYS File System for SMI230 Sensor.
 *
 * Function for Reading Temp Sensor data from SYS File System
 * and send it to slim core.
 *
 * @return void.
 */
int SensorApiService::readTempSMI230(float *temperature)
{

  string tempString;

  int tempRead = 0;
  if ( NULL == mTempFilePtr.smiTempFile.tempFile )
  {
    return 0;
  }
  /* Read Temp data */
  (void)mTempFilePtr.smiTempFile.tempFile->seekg(0);

  if ( mTempFilePtr.smiTempFile.tempFile->peek() == NULL )
  {
    SENSOR_LOGE(LOG_TAG "Temperature string not available in file\n");
    return 0;
  }

  (void)std::getline(*mTempFilePtr.smiTempFile.tempFile,  tempString);
  std::vector<std::string> split;
  {
    std::stringstream ss (tempString);
    std::string val;
    while (std::getline (ss, val, ' ')) {
      split.push_back (val);
    }
  }
  if (split.size() < 2 ){
    SENSOR_LOGE(LOG_TAG "temperature value unavailable\n");
    return 0;
  }
  tempRead = std::stoi(split[1]);
  *temperature =  (float) (tempRead / 1000.0); //Convert to Degree celsius
  SENSOR_LOGD(LOG_TAG "Read Raw:%d, Temperature: %f\n", tempRead, temperature);
  return 0;
}

/**
 * @brief Temp Sensor data processing task.
 *
 * Function for processing buffered data from sysfs interface
 * and send it for formatting and routing through client  .
 *
 * @return int.
 */
int SensorApiService::tempSensorDataPollTask(float *temperature)
{
    int ret = -1;
    SENSOR_LOGD(LOG_TAG "Polling Temp Sensor ..\n");
    switch(mSensorType)
    {
      case SENSOR_ASM330:
        ret = readTempASM(temperature);
        break;
      case SENSOR_BMI160:
        ret = readTempBMI(temperature);
        break;
      case SENSOR_IAM20680:
        ret = readTempIAM(temperature);
        break;
      case SENSOR_SMI130:
        ret = readTempSMI(temperature);
        break;
      case SENSOR_SMI230:
        ret = readTempSMI230(temperature);
        break;
      default:
        ret = -1;
        SENSOR_LOGE(LOG_TAG "Temp error - invalid config, type- %d",mSensorType);
        break;
    }
    return 0;
}


/**
 * @brief Intialize temp sensor.
 *
 * Function for initializing Temperature sensor thread
 *
 * @param[in] pData .
 *
 * @return success/failure as true /false
 */
bool SensorApiService::tempSensorDataInit()
{

  SENSOR_LOGI(LOG_TAG "Initializing SensorTempDataInit ..\n");
  switch ( mSensorType )
  {
    case SENSOR_ASM330:

      {
        char tScaleFilePath[SEARCH_PATH_SIZE]={'\0'};
        char tOffsetFilePath[SEARCH_PATH_SIZE]={'\0'};
        char tRawDataFilePath[SEARCH_PATH_SIZE]={'\0'};

        find_path(DYN_IIO_TYPE, tScaleFilePath, ASM_TEMP_SEARCH, sizeof(tScaleFilePath));

        if(strlen(tScaleFilePath) == 0)
        {
	  find_path(DYN_IIO_TYPE, tScaleFilePath, ASMX_TEMP_SEARCH, sizeof(tScaleFilePath));
	  if(strlen(tScaleFilePath) == 0)
		  return false;
	}
        SENSOR_LOGI(LOG_TAG "found path in temp:%s",tScaleFilePath);
        (void)strlcpy(tOffsetFilePath, tScaleFilePath, sizeof(tOffsetFilePath));
        (void)strlcpy(tRawDataFilePath, tScaleFilePath, sizeof(tRawDataFilePath));

        (void)strlcat(tScaleFilePath, ASM_TEMP_SCALE, sizeof(tScaleFilePath));
        (void)strlcat(tOffsetFilePath, ASM_TEMP_OFFSET, sizeof(tOffsetFilePath));
        (void)strlcat(tRawDataFilePath, ASM_TEMP_RAW , sizeof(tRawDataFilePath));

        mTempFilePtr.asmTempFile.tScaleFile = new(std::nothrow) ifstream;
        if (!mTempFilePtr.asmTempFile.tScaleFile) {
          SENSOR_LOGE(LOG_TAG "failed to allocate %s errno %d, (%s)", tScaleFilePath, errno, strerror(errno));
          return false;
        }
        mTempFilePtr.asmTempFile.tScaleFile->open(tScaleFilePath);
        if (!mTempFilePtr.asmTempFile.tScaleFile->is_open()) {
          SENSOR_LOGE(LOG_TAG "failed to open %s errno %d, (%s)", tScaleFilePath, errno, strerror(errno));
          return false;
        }

        mTempFilePtr.asmTempFile.tOffsetFile = new(std::nothrow) ifstream;
        if (!mTempFilePtr.asmTempFile.tOffsetFile) {
          SENSOR_LOGE(LOG_TAG "failed to allocate %s errno %d, (%s)", tOffsetFilePath, errno, strerror(errno));
          return false;
        }
        mTempFilePtr.asmTempFile.tOffsetFile->open(tOffsetFilePath);
        if (!mTempFilePtr.asmTempFile.tOffsetFile->is_open()) {
          SENSOR_LOGE(LOG_TAG "failed to open %s errno %d, (%s)", tOffsetFilePath, errno, strerror(errno));
          return false;
        }

        mTempFilePtr.asmTempFile.tRawDataFile = new(std::nothrow) ifstream;
        if (!mTempFilePtr.asmTempFile.tRawDataFile) {
          SENSOR_LOGE(LOG_TAG "failed to allocate %s errno %d, (%s)", tRawDataFilePath, errno, strerror(errno));
          return false;
        }
        mTempFilePtr.asmTempFile.tRawDataFile->open(tRawDataFilePath);
        if (!mTempFilePtr.asmTempFile.tRawDataFile->is_open()) {
          SENSOR_LOGE(LOG_TAG "failed to open %s errno %d, (%s)", tRawDataFilePath, errno, strerror(errno));
          return false;
        }
        break;
      }
    case SENSOR_BMI160:
      {
        char tTempFilePath[SEARCH_PATH_SIZE]={'\0'};
        find_path(DYN_IIO_TYPE, tTempFilePath, BMI_TEMP_SEARCH, sizeof(tTempFilePath));
        if (strlen(tTempFilePath) ==0 )
        {
          return false;
        }
        (void)strlcat(tTempFilePath, BMI_TEMP_NAME, sizeof(tTempFilePath));
        SENSOR_LOGI(LOG_TAG "Bmi temp path - %s",tTempFilePath);
        mTempFilePtr.bmiTempFile.tTempFile = new(std::nothrow)ifstream;
        if (!mTempFilePtr.bmiTempFile.tTempFile) {
          SENSOR_LOGE(LOG_TAG "failed to allocate %s errno %d, (%s)", tTempFilePath, errno, strerror(errno));
          return false;
        }
        mTempFilePtr.bmiTempFile.tTempFile->open(tTempFilePath);
        if (!mTempFilePtr.bmiTempFile.tTempFile->is_open()) {
          SENSOR_LOGE(LOG_TAG "failed to open %s errno %d, (%s)", tTempFilePath, errno, strerror(errno));
          return false;
        }
      }
      break;
    case SENSOR_IAM20680:
      {
        char iamPathTemp[SEARCH_PATH_SIZE]={'\0'};
        find_path(DYN_IIO_TYPE, iamPathTemp, IAM_TEMP_SEARCH, sizeof(iamPathTemp));
        if(strlen(iamPathTemp) == 0)
        {
          return false;
        }
        (void)strlcat(iamPathTemp, IAM_TEMP_NAME, sizeof(iamPathTemp));
        mTempFilePtr.iamTempFile.dataFile = new(std::nothrow) ifstream;
        if (!mTempFilePtr.iamTempFile.dataFile) {
          SENSOR_LOGE(LOG_TAG "failed to allocate %s errno %d, (%s)", iamPathTemp, errno, strerror(errno));
          return false;
        }
        mTempFilePtr.iamTempFile.dataFile->open(iamPathTemp);
        if (!mTempFilePtr.iamTempFile.dataFile->is_open()) {
          SENSOR_LOGE(LOG_TAG "failed to open %s errno %d, (%s)", iamPathTemp, errno, strerror(errno));
          return false;
        }
        break;
      }
    case SENSOR_SMI130:
    case SENSOR_SMI230:
      {
        char smiPathTemp[SEARCH_PATH_SIZE]={'\0'};
	char smiSearchKey[SEARCH_PATH_SIZE]={'\0'};
	if (SENSOR_SMI130 == mSensorType )
          (void)memcpy(smiSearchKey, SMI_TEMP_SEARCH, sizeof(SMI_TEMP_SEARCH));
        else
          (void)memcpy(smiSearchKey, SMI230_TEMP_SEARCH, sizeof(SMI230_TEMP_SEARCH));

        find_path(DYN_INPUT_TYPE,smiPathTemp, smiSearchKey, sizeof(smiPathTemp));
        if (strlen(smiPathTemp) != 0)
        {
          (void)strlcat(smiPathTemp, SMI_TEMP_NAME, sizeof(smiPathTemp));
          SENSOR_LOGI(LOG_TAG "Smi temperature Path detected: %s \n",smiPathTemp);
        }
        else
        {
          SENSOR_LOGE(LOG_TAG "No Smi Temp path detected");
          return false;
        }
        SENSOR_LOGI(LOG_TAG "Entered in SMI temp, file path: %s, length: %d \n",smiPathTemp,strlen(smiPathTemp));

        mTempFilePtr.smiTempFile.tempFile = new(std::nothrow)ifstream;
        if (!mTempFilePtr.smiTempFile.tempFile) {
          SENSOR_LOGE(LOG_TAG "failed to allocate %s errno %d, (%s)", smiPathTemp, errno, strerror(errno));
          return false;
        }
        mTempFilePtr.smiTempFile.tempFile->open(smiPathTemp);

        if (!mTempFilePtr.smiTempFile.tempFile->is_open()) {
          SENSOR_LOGE(LOG_TAG "failed to open %s errno %d, (%s)", smiPathTemp, errno, strerror(errno));
          return false;
        }
        break;
      }
    default:
      SENSOR_LOGE(LOG_TAG "Temp open error - invalid config, type- %d",mSensorType);
      return false;
      break;
  }
  return true;
}
/**
 * @brief Conversion of raw data.
 *
 * Function calculates BMI buffer data.
 * @param[in] SensorType- sensor type.
 *            event - structure containing raw data
 * @return void.
 */
void scalingBMIBufferData(int SensorType,sensors_event_t *event)
{
  float scaleFactor = 1;
  /* Get the scale factor based on sensor type */
  if (SENSOR_TYPE_ACCELEROMETER == SensorType) {
    scaleFactor = BMI_ACC_RESL * BMI_CONVERT_ACC; // Calculate Scale Value
    event->type = SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED;
    event->acceleration.x *= scaleFactor; // Calculate scaled X coordinate
    event->acceleration.y *= scaleFactor; // Calculate scaled Y coordinate
    event->acceleration.z *= scaleFactor; // Calculate scaled Z coordinate
  }
  else {
    scaleFactor = BMI_CONVERT_GYRO;
    event->type = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
    event->gyro.x *= scaleFactor; // Calculate scaled X coordinate
    event->gyro.y *= scaleFactor; // Calculate scaled Y coordinate
    event->gyro.z *= scaleFactor; // Calculate scaled Z coordinate
  }
}

/**
 * @brief Conversion of raw data.
 *
 * Function calculates IAM buffer data.
 *
 * @param[in] SensorType- sensor type.
 *            event - structure containing raw data
 *
 * @return void.
 */
void scalingIAMBufferData(int SensorType,sensors_event_t *event)
{
    signed char orientationMatrix[9]={0, -1, 0, 1, 0, 0, 0, 0, 1};
    float scale = 0;
    float data[3];
    if (SENSOR_TYPE_ACCELEROMETER == SensorType) {
      scale = 1.f / (32768.0f / convert_acc_buff_iam) * 9.80665f; // Calculate scale value
      /* convert to body frame */
      for (int i = 0; i < 3 ; i++) {
	      data[i] = event->acceleration.x * orientationMatrix[i * 3] + // Caluclate oriented data using X coordinate
		      event->acceleration.y * orientationMatrix[i * 3 + 1] + // Caluclate oriented data using Y coordinate
		      event->acceleration.z * orientationMatrix[i * 3 + 2]; // Caluclate oriented data using Z coordinate
      }
      event->type = SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED;
      event->acceleration.x = (float)data[0] * scale; // Calculate scaled X coordinate
      event->acceleration.y = (float)data[1] * scale; // Calculate scaled Y coordinate
      event->acceleration.z = (float)data[2] * scale; // Calculate scaled Z coordinate
    }
    else if (SENSOR_TYPE_GYROSCOPE == SensorType) {
      scale = 1.f / convert_gyro_buff_iam * 0.0174532925f; // Calculate scale value
      /* convert to body frame */
      for (int i = 0; i < 3 ; i++) {
              data[i] = event->gyro.x * orientationMatrix[i * 3] + // Caluclate oriented data using X coordinate
                      event->gyro.y * orientationMatrix[i * 3 + 1] + // Caluclate oriented data using Y coordinate
                      event->gyro.z * orientationMatrix[i * 3 + 2]; // Caluclate oriented data using Z coordinate
      }
      event->type = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
      event->gyro.x = (float)data[0] * scale; // Calculate scaled X coordinate
      event->gyro.y = (float)data[1] * scale; // Calculate scaled Y coordinate
      event->gyro.z = (float)data[2] * scale; // Calculate scaled Z coordinate
    }
}

/**
 * @brief Conversion of raw data.
 *
 * Function calculates SMI130 buffer data.
 * @param[in] SensorType- sensor type.
 *            event - structure containing raw data
 * @return void.
 */
void scalingSMIBufferData(int SensorType,sensors_event_t *event)
{
  int cnt;
  float scaleFactor = 1;
  /* Get the scale factor based on sensor type */
  if (SENSOR_TYPE_ACCELEROMETER == SensorType) {
    scaleFactor = SMI_ACC_RESL * convert_acc_buff_smi130; // Calculate scale value
    event->type = SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED;
    event->acceleration.x *= scaleFactor; // Calculate scaled X coordinate
    event->acceleration.y *= scaleFactor; // Calculate scaled Y coordinate
    event->acceleration.z *= scaleFactor; // Calculate scaled Z coordinate
  }
  else {
    scaleFactor = convert_gyro_buff_smi130;
    event->type = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
    event->gyro.x *= scaleFactor; // Calculate scaled X coordinate
    event->gyro.y *= scaleFactor; // Calculate scaled Y coordinate
    event->gyro.z *= scaleFactor; // Calculate scaled Z coordinate
  }
}

/**
 * @brief Conversion of raw data.
 *
 * Function calculates SMI230 buffer data.
 * @param[in] SensorType- sensor type.
 *            event - structure containing raw data
 * @return void.
 */
void scalingSMI230BufferData(int SensorType,sensors_event_t *event)
{
  int cnt;
  /* Get the scale factor based on sensor type */
  if (SENSOR_TYPE_ACCELEROMETER == SensorType) {
    event->type = SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED;
    event->acceleration.x *= convert_acc_buff_smi230; // Calculate scaled X coordinate
    event->acceleration.y *= convert_acc_buff_smi230; // Calculate scaled Y coordinate
    event->acceleration.z *= convert_acc_buff_smi230; // Calculate scaled Z coordinate
  }
  else {
    event->type = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
    event->gyro.x *= convert_gyro_buff_smi230; // Calculate scaled X coordinate
    event->gyro.y *= convert_gyro_buff_smi230; // Calculate scaled Y coordinate
    event->gyro.z *= convert_gyro_buff_smi230; // Calculate scaled Z coordinate
  }
}

/**
 * @brief Conversion of raw data.
 *
 * Function calculates ASM buffer data.
 * @param[in] SensorType- sensor type.
 *            event - structure containing raw data
 * @return void.
 */
static float process_2byte_received(int input, float scale)
{
  input = le16toh((uint16_t)input);
  return ((float) ((int16_t)input) * scale); // Calculate scaled data
}

void scalingASMBufferData(int SensorType,sensors_event_t *event)
{
  int cnt;
  /* Get the scale factor based on sensor type */
  if (SENSOR_TYPE_ACCELEROMETER == SensorType)
  {
    event->type = SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED;
    event->acceleration.x = process_2byte_received(event->acceleration.x, convert_acc_buff_asm);
    event->acceleration.y = process_2byte_received(event->acceleration.y, convert_acc_buff_asm);
    event->acceleration.z = process_2byte_received(event->acceleration.z, convert_acc_buff_asm);
  }
  else {
    event->type = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
    event->gyro.x = process_2byte_received(event->gyro.x, convert_gyro_buff_asm);
    event->gyro.y = process_2byte_received(event->gyro.y, convert_gyro_buff_asm);
    event->gyro.z = process_2byte_received(event->gyro.z, convert_gyro_buff_asm);
  }
}

/* @brief Check for sensor and scale buffer data
 *
 * @param[in] SensorType- sensor type.
 *            event - structure containing raw data
 * @return void.
 */
void SensorApiService::bufferDataScaling( int SensorType, sensors_event_t *event )
{
  switch( mSensorType )
  {
    case SENSOR_BMI160:
      scalingBMIBufferData(SensorType, event);
      break;
    case SENSOR_IAM20680:
      scalingIAMBufferData(SensorType, event);
      break;
    case SENSOR_SMI130:
      scalingSMIBufferData(SensorType, event);
      break;
    case SENSOR_ASM330:
      scalingASMBufferData(SensorType, event);
      break;
    case SENSOR_SMI230:
      scalingSMI230BufferData(SensorType, event);
      break;
  }
}

/* @brief Check for bufffer support
 *
 * @return true if buffer supported else false.
 */
bool SensorApiService::CheckBufferReadFile()
{
 char acc_boot_sample[SEARCH_PATH_SIZE]={'\0'};
 char gyr_boot_sample[SEARCH_PATH_SIZE]={'\0'};

 switch (mSensorType)
 {
    case SENSOR_BMI160:
      find_path(DYN_IIO_TYPE, acc_boot_sample, BMI_TEMP_SEARCH, sizeof(acc_boot_sample));
      (void)strlcpy(gyr_boot_sample, acc_boot_sample, sizeof(gyr_boot_sample));
      break;
    case SENSOR_IAM20680:
      find_path(DYN_IIO_TYPE, acc_boot_sample, IAM_TEMP_SEARCH, sizeof(acc_boot_sample));
      (void)strlcpy(gyr_boot_sample, acc_boot_sample, sizeof(gyr_boot_sample));
      break;
    case SENSOR_SMI130:
      find_path(DYN_INPUT_TYPE, acc_boot_sample, SMI_TEMP_SEARCH, sizeof(acc_boot_sample));
      find_path(DYN_INPUT_TYPE, gyr_boot_sample, SMI_GYR_SEARCH, sizeof(gyr_boot_sample));
      break;
   case SENSOR_SMI230:
      find_path(DYN_INPUT_TYPE, acc_boot_sample, SMI230_TEMP_SEARCH, sizeof(acc_boot_sample));
      find_path(DYN_INPUT_TYPE, gyr_boot_sample, SMI230_GYR_SEARCH, sizeof(gyr_boot_sample));
      break;
    case SENSOR_ASM330:
      find_path(DYN_IIO_TYPE, acc_boot_sample, ASM_ACC_SEARCH, sizeof(acc_boot_sample));
      if(strlen(acc_boot_sample) == 0)
	      find_path(DYN_IIO_TYPE, acc_boot_sample, ASMX_ACC_SEARCH, sizeof(acc_boot_sample));
      find_path(DYN_IIO_TYPE, gyr_boot_sample, ASM_GYR_SEARCH, sizeof(gyr_boot_sample));
      if(strlen(gyr_boot_sample) == 0)
	      find_path(DYN_IIO_TYPE, gyr_boot_sample, ASMX_GYR_SEARCH, sizeof(gyr_boot_sample));
      break;
    default:
      SENSOR_LOGE(LOG_TAG "Buffer find error - invalid config, type- %d\n",mSensorType);
      break;
 }
 SENSOR_LOGI(LOG_TAG "acc_boot_sample-%s,gyr_boot_sample-%s\n",acc_boot_sample,gyr_boot_sample);
 if(strlen(acc_boot_sample) == 0 || strlen(gyr_boot_sample) == 0)
 {
         return false;
 }
 (void)strlcat(acc_boot_sample, ACC_BUFFER_READ, sizeof(acc_boot_sample));
 (void)strlcat(gyr_boot_sample, GYRO_BUFFER_READ, sizeof(gyr_boot_sample));

 mAccBootSample = acc_boot_sample;
 mGyroBootSample = gyr_boot_sample;

 SENSOR_LOGI(LOG_TAG "mAccBootSample-%s,mGyroBootSample-%s\n",mAccBootSample.c_str(),mGyroBootSample.c_str());

 (void)pthread_mutex_init(&mHalBuffMutex, NULL);
 (void)pthread_cond_init(&mHalBuffCond, NULL);

 return true;
}

/* @brief enable and delete buffer data
 *
 * write 1 to read buffer data and 0 to delete buffer data.
 */
bool SensorApiService::WritetoBufferFile(bool enable) {
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
 * @param[in] SensorType - sensorType
 *
 * @return void.
 */
bool SensorApiService::getBufferedSample(int SensorType, FILE* fd, sensors_event_t *event) {

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
	   if (SENSOR_TYPE_ACCELEROMETER == SensorType) {
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
	   if (SENSOR_TYPE_GYROSCOPE == SensorType) {
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


bool SensorApiService::ReadSensorBufferData(const std::string clientname) {
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

  std::unordered_map<std::string, SensorHalDaemonClientHandler*>::iterator it = mClients.find(clientname);

  accelBuffDataTxProgress = true;
  gyroBuffDataTxProgress = true;

  if(mfdBuffAccel == nullptr || mfdBuffGyro == nullptr) {
      WritetoBufferFile(enable);
  }

  //start reading kernel buffered data;
  // Read and process all buffered data from sysfs
  // Before reading and processing buffered data make sure client is available to receive data
  // interleave the data while transferring to client aplication as per batching size
  while(accelBuffDataTxProgress || gyroBuffDataTxProgress)
  {
     /* Fill the accel buffered data from kernel bufer */
     if (accelBuffDataTxProgress) {
	     if(getBufferedSample(SENSOR_TYPE_ACCELEROMETER, mfdBuffAccel, &zevents[0])) {
		     (void)memcpy(&events[count], &zevents[0], sizeof(sensors_event_t));
		     bufferDataScaling(SENSOR_TYPE_ACCELEROMETER, &events[count]);
		     SENSOR_LOGW(LOG_TAG "ACC Buffer event: x=%f y=%f z=%f timestamp=%lld acccount %d\n",
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
	     if (getBufferedSample(SENSOR_TYPE_GYROSCOPE, mfdBuffGyro, &zevents[1])) {
		     (void)memcpy(&events[count], &zevents[1], sizeof(sensors_event_t));
		     bufferDataScaling(SENSOR_TYPE_GYROSCOPE, &events[count]);
		     SENSOR_LOGW(LOG_TAG "GYRO Buffer event: x=%f y=%f z=%f timestamp=%lld gyrocount %d\n",
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

void SensorApiService::SensorBuffread() {
  bool rc = false;
  while(mBufferSupported) {
    (void)pthread_mutex_lock (&mHalBuffMutex);
    (void)pthread_cond_wait (&mHalBuffCond, &mHalBuffMutex);
    (void)pthread_mutex_unlock (&mHalBuffMutex);

    auto it = mClients.begin();
    while (it != mClients.end() && it != (std::unordered_map<std::string, SensorHalDaemonClientHandler*>::iterator)NULL) {
	    if (it->second->mBufferRead == true && mBufferDeleted != true) {
		    rc = ReadSensorBufferData(it->first.c_str());
		    // purge this client if failed
		    if(!rc) {
			    SENSOR_LOGE(LOG_TAG "failed rc=%d purging client=%s\n", rc, it->first.c_str());
			    std::lock_guard<std::mutex> lock(SensorApiService::mMutex);
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
