/*
* Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/
#ifndef SENSOR_DEVICE_H
#define SENSOR_DEVICE_H

#include <stdint.h>
#include <stdio.h>
#include <fstream>
#include <linux/input.h>
#include <vector>
#include <string>
#include <functional>
#include <memory>
#include <map>
#include <variant>
#include <sensors.h>
#include <SensorApiUtils.h>
#include <SensorHalDaemonClientHandler.h>
#include <SensorApiService.h>

#define PATH_MLC_BINARY  "/lib/firmware/st_asm330lhhx_mlc.bin"

using namespace std;

// Forward declaration of default_fir_coef_t
#ifdef NO_UNORDERED_SET_OR_MAP
typedef map<int, vector<pair<int, vector<float>>>> default_fir_coef_t;
#else
typedef unordered_map<int, vector<pair<int, vector<float>>>> default_fir_coef_t;
#endif

//To check sensor type defined in config file
typedef enum
{
  SENSOR_UNKNOWN = 0,
  SENSOR_ASM330,
  SENSOR_IAM20680,
  SENSOR_SMI230,
} sensorType;

// forward declaration
class SensorApiService;

/**
 * class SensorDevice
 * Exposes methods to communicate Sensor Specific implementation.
 */
class SensorDevice {
public:
	SensorDevice(SensorApiService* service);
	~SensorDevice();
	bool openSensorDevice(string *libname);
	void getSensorDeviceName(string *sensorName);
	bool getSensorBufferFile(string *acc_name, string *gyro_name);
	void getSensorBufferScalarValue(float *accel_scale, float *gyro_scale);
	bool checkTempSupport();
	int  readSensorTemperature(float *temperature);
	void setDefaultFIRCoeff();
	int getDefaultFIRCoeff(bool is_accel, int sensor_rate, int client_rate, vector<float> &out_coef);
	bool  sensorDevSelfTest(int sensor_id, int type, SelfTestType selfTestType, SelfTestResult *selfTest, uint64_t *selfTestTs);
	int  initMaxRange(int, int);
        void getSupportedSamplingRateAndRange(struct sensor_list *s);
	//MLC API
	bool loadMLC(const char *mcl_fw_name);
	bool sensorMlcEnableEvents(char *mlc_case_name, int enable);
	int  setPowerMode(int handle, int mode);
	void pollEvents(void);
	static void* mlcPollEvents(void *arg);
	//temp files
	struct tempPtr {
		ifstream *tempfile;
		string tempfilePath;
		tempPtr() : tempfile(nullptr) {}
		tempPtr(const string& path) : tempfile(nullptr), tempfilePath(path) {}
	};
	struct SensorInfo {
		const char*     chip_name;
		char*     accel_name;
		char*     gyro_name;
		char*     temp_name;
		const char*     accel_range_name;
		const char*     gyro_range_name;
		const char*     selftest_name;
		vector<tempPtr> temp_files;
		const char*     lib_name;
		vector<float>   accel_odr;
		vector<float>   gyro_odr;
		map<float, tuple<float, float>> accel_range_scale_map;
	        map<float, tuple<float, float>> gyro_range_scale_map;
		float           accel_buff_range;
		float           gyro_buff_range;
		int             batch_const;
		bool            is_input;
	};
protected:
private:
	sensorType mSensorType;
	string mAccel;
	string mGyro;
	string mTemp;
	map<sensorType, vector<SensorInfo>> mSensorInfo;
	SensorApiService* mService;
	default_fir_coef_t mDefaultAccelCoef;
	default_fir_coef_t mDefaultGyroCoef;
};

#endif //SENSOR_DEVICE_H
