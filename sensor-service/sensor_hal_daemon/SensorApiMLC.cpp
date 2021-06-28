/* Copyright (c) 2021 The Linux Foundation. All rights reserved.
 *
 * Not a contribution.
 *
 * STMicroelectronics SensorHAL simple test
 *
 * Copyright 2021 STMicroelectronics Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 */
#include <stdint.h>
#include <sys/stat.h>
#include <dlfcn.h>
#include <memory>
#include <algorithm>
#include <SensorHalDaemonClientHandler.h>
#include <SensorApiService.h>
#include <poll.h>

#define IIO_DEVICE_NAME         "/sys/bus/iio/devices/iio:device"

#define IIO_GYRO_IIO_DEVICE_NUMBER "0"
#define IIO_ACC_IIO_DEVICE_NUMBER  "1"

#define IIO_GYRO_DEVICE_NAME    IIO_DEVICE_NAME IIO_GYRO_IIO_DEVICE_NUMBER
#define IIO_ACC_DEVICE_NAME     IIO_DEVICE_NAME IIO_ACC_IIO_DEVICE_NUMBER

#define IIO_MLC_EVENT_NAME      "events/in_activity0_thresh_rising_en"
#define IIO_MLC_LOAD_FILE       "/load_mlc"
#define IIO_MLC_FLUSH_FILE      "/mlc_flush"
#define IIO_MLC_VERSION_FILE    "/mlc_version"
#define IIO_MLC_INFO_FILE       "/mlc_info"

#define IIO_GET_EVENT_FD_IOCTL _IOR('i', 0x90, int)

#define UCF_STR_LEN             256

#define ASM330LHHX_ACC_SEARCH         "asm330lhhx_accel"
#define ASM330LHHX_GYRO_SEARCH        "asm330lhhx_gyro"

static int mlc_case_device_number = 0;
static int mlc_case_device_start_index = 0;

/*
 *SensorApiService - Print sensor mlc case List
 */
static void PrintSensorMLCCaseList(struct sensor_mlc_case_list *m, int mlc_case_count)
{
   SENSOR_LOGI(LOG_TAG "mlc_case_count %d\n", mlc_case_count);
   for (int i=0 ; i< mlc_case_count ; i++) {
           SENSOR_LOGD(LOG_TAG"%s\n",m[i].name);
   }
}

/*
 * MLC case devices number
 */
static bool find_mlc_case_iio_device_number(int i)
{
        char iio_device_name[UCF_STR_LEN];
        struct stat sb;

	snprintf(iio_device_name, sizeof(iio_device_name),
			"/sys/bus/iio/devices/iio:device%d/events/in_activity0_thresh_rising_en",
			i);
	//SENSOR_LOGI(LOG_TAG "iio_device_name %s\n", iio_device_name);
	if (stat(iio_device_name, &sb) == 0) {
		SENSOR_LOGI(LOG_TAG "Found MLC IIO case device number %d", i);
		return true;
	}

	return false;
}

/*
 * MLC
 */
static int find_mlc_iio_device_number(void)
{
        char iio_device_name[UCF_STR_LEN];
        struct stat sb;
        int i;

        for (i = 3; i <=6; i++) {
                snprintf(iio_device_name, sizeof(iio_device_name),
                        "/sys/bus/iio/devices/iio:device%d/mlc_version",
                        i);

                if (stat(iio_device_name, &sb) == 0) {
                        SENSOR_LOGI(LOG_TAG "Found MLC IIO device number %d", i);

                        return i;
                }
        }

        return -1;
}

static int get_mlc_case_name(int i, char *name) {
        char iio_device_file[UCF_STR_LEN];
	FILE *nameFile;
	int ret =0;

	snprintf(iio_device_file, sizeof(iio_device_file),
			"/sys/bus/iio/devices/iio:device%d/name",
			i);

        nameFile = fopen(iio_device_file, "r");
        if (!nameFile)
        {
            return false;
        }

        ret = fscanf(nameFile, "%s", name);
        if(ret <= 0)
        {
            fclose(nameFile);
            return false;
        }
        fclose(nameFile);
	return true;
}

/*
 * MLC info
 */
static int mlc_info(int iio_device_number)
{
	FILE *mlc_info_fd = NULL;
        FILE *mlc_version_fd = NULL;
        char mlc_info_file_name[UCF_STR_LEN];
        char mlc_version_file_name[UCF_STR_LEN];
        char str[UCF_STR_LEN];
        int ret;

        ret = snprintf(mlc_info_file_name, sizeof(mlc_info_file_name), "%s%d/%s",
                      IIO_DEVICE_NAME,
                      iio_device_number,
                      IIO_MLC_INFO_FILE);
        if (ret < 0) {
                SENSOR_LOGE(LOG_TAG "MLC: Failed to retrive mlc_info file name %s\n", mlc_info_file_name);
                return -1;
        }
        ret = snprintf(mlc_version_file_name, sizeof(mlc_version_file_name), "%s%d/%s",
                      IIO_DEVICE_NAME,
                      iio_device_number,
                      IIO_MLC_VERSION_FILE);
        if (ret < 0) {
                SENSOR_LOGE(LOG_TAG "MLC: Failed to retrive mlc_version file name %s\n", mlc_version_file_name);
                return -1;
        }

        SENSOR_LOGI(LOG_TAG "MLC: opening file %s\n", mlc_version_file_name);
        mlc_version_fd = fopen(mlc_version_file_name, "r");
        if (!mlc_version_fd) {
                SENSOR_LOGE(LOG_TAG "MLC: open mlc_version_file_name %s failed\n", mlc_version_file_name);
                return -1;
        }

        SENSOR_LOGI(LOG_TAG "MLC: opening file %s\n", mlc_info_file_name);
        mlc_info_fd = fopen(mlc_info_file_name, "r");
        if (!mlc_info_fd) {
                SENSOR_LOGE(LOG_TAG "MLC: open mlc_info_file_name %s failed\n", mlc_info_file_name);
                return -1;
        }

        if (fgets(str, UCF_STR_LEN, mlc_info_fd) != NULL) {
                SENSOR_LOGI(LOG_TAG "MLC INFO: %s\n", str);
        }

        memset(str, 0, UCF_STR_LEN);
        if (fgets(str, UCF_STR_LEN, mlc_version_fd) != NULL) {
                SENSOR_LOGI(LOG_TAG "MLC VERSION: %s\n", str);
        }

        fclose(mlc_info_fd);
        fclose(mlc_version_fd);

        return 0;
}

/*
 * MLC flush
 */
static int mlc_flush(int iio_device_number)
{
        char mlc_file_name[UCF_STR_LEN];
        FILE *sysfs_flush = NULL;
        char str[UCF_STR_LEN];
        int ret;

        ret = snprintf(mlc_file_name, sizeof(mlc_file_name), "%s%d/%s",
                      IIO_DEVICE_NAME,
                      iio_device_number,
                      IIO_MLC_FLUSH_FILE);
        if (ret < 0) {
                SENSOR_LOGE(LOG_TAG "MLC: Failed to retrive mlc device name %s\n", mlc_file_name);
                return -1;
        }

        SENSOR_LOGI(LOG_TAG "MLC: opening file %s\n", mlc_file_name);
        sysfs_flush = fopen(mlc_file_name, "w");
        if (sysfs_flush == NULL) {
                SENSOR_LOGE(LOG_TAG "MLC: Failed to open %s\n", mlc_file_name);
                return -1;
        }

        ret = fprintf(sysfs_flush, "1");
        if (ret < 0) {
                SENSOR_LOGI(LOG_TAG "MLC: Flush [FAIL]\n");
        } else {
                SENSOR_LOGI(LOG_TAG "MLC: Flush [DONE]\n");
        }

        if (sysfs_flush)
                fclose(sysfs_flush);

        return 0;
}
/*
 * echo 1 > load_mlc
 */
bool SensorApiService::LoadMLC(const char *mcl_fw_name)
{
        char mlc_file_name[UCF_STR_LEN];
        FILE *sysfs_load = NULL;
        FILE *mcl_fw = NULL;
        int ret = 0, i = 0;
	int iio_device_number = -1;
	bool mlc_start_index = true;
	//Find the MLC node
	iio_device_number = find_mlc_iio_device_number();
	mlc_case_device_number = iio_device_number + 1;

	//Flush the MLC
	mlc_flush(iio_device_number);

        mcl_fw = fopen(mcl_fw_name, "r");
        if (mcl_fw == 0) {
                SENSOR_LOGE(LOG_TAG "MLC: Unable to open file mcl_fw_name %s\n",mcl_fw_name);
		return false;
        }
        if (mcl_fw)
                fclose(mcl_fw);

	//Flash Firmaware to MLC
        ret = snprintf(mlc_file_name, sizeof(mlc_file_name), "%s%d/%s",
                      IIO_DEVICE_NAME,
                      iio_device_number,
                      IIO_MLC_LOAD_FILE);
        if (ret < 0) {
                SENSOR_LOGE(LOG_TAG "MLC: Failed to retrive device name\n");
		return false;
        }

        sysfs_load = fopen(mlc_file_name, "w");
        if (sysfs_load == NULL) {
                SENSOR_LOGE(LOG_TAG "MLC: Failed to open mcl_file_name %s\n", mlc_file_name);
		return false;
        }

        SENSOR_LOGI(LOG_TAG "Loading MLC.......\n");
        ret = fprintf(sysfs_load, "1");
        if (ret < 0) {
                SENSOR_LOGE(LOG_TAG "Loading MLC [FAIL]\n");
		return false;
        } else {
                SENSOR_LOGE(LOG_TAG "Loading MLC [DONE]\n");
        }
        if (sysfs_load)
                fclose(sysfs_load);
	sleep(3);
	//Print MLC INFO
	mlc_info(iio_device_number);
	memset(mlc_file_name, 0 ,sizeof(mlc_file_name));
	mSesnorMlcCaseList = (struct sensor_mlc_case_list*) malloc(sizeof(struct sensor_mlc_case_list));
	mSensorMlcCaseCount = 0;
	for (int i = mlc_case_device_number; i < iio_device_number+12; i++) {
		if(find_mlc_case_iio_device_number(i)) {
			if (mlc_start_index)
				mlc_case_device_start_index = i;
			mlc_start_index = false;
			//Create MLC case List to send to all Clients.
			get_mlc_case_name(i, mlc_file_name);
			strlcpy(&mSesnorMlcCaseList[mSensorMlcCaseCount].name[0], mlc_file_name, MAX_PATH_SIZE);
			mSensorMlcCaseCount++;
			mSesnorMlcCaseList = (struct sensor_mlc_case_list*) realloc(mSesnorMlcCaseList,
					(mSensorMlcCaseCount+1) * sizeof(struct sensor_mlc_case_list));
			memset(mlc_file_name, 0 ,sizeof(mlc_file_name));
		}
	}
	//Print sensor list for debug
	PrintSensorMLCCaseList(mSesnorMlcCaseList, mSensorMlcCaseCount);

	if(mSensorMlcCaseCount > 0) {
		//Create the thread to read mlc case events
		if (!Sensor_ThreadCreate(&mMlcThreadtid, mlcPollEvents, this, "SensorMlcEventsRead-")) {
			SENSOR_LOGE(LOG_TAG "Sensor Mlc Events Read thread failed \n");
			return false;
		}
		return true;
        }
	return false;
}

bool SensorApiService::SensorMlcEnableEvents(char *mlc_case_name, int enable)
{
	FILE *event_file_enable = NULL;
	char event_file_enable_name[UCF_STR_LEN] = {'\0'};
	int ret;

	SENSOR_LOGI(LOG_TAG "mlc_case_name %s\n",mlc_case_name);

	findPath(DYN_IIO_TYPE, event_file_enable_name, mlc_case_name, sizeof(event_file_enable_name));

	strlcat(event_file_enable_name, IIO_MLC_EVENT_NAME, sizeof(event_file_enable_name));

	SENSOR_LOGI(LOG_TAG "event_file_enable_name %s", event_file_enable_name);

	event_file_enable = fopen(event_file_enable_name, "r+");
	if (!event_file_enable) {
		SENSOR_LOGE(LOG_TAG "open event_file_enable_name %s failed",event_file_enable_name);
		return false;
	}

	if (enable)
		ret = fprintf(event_file_enable, "1");
	else
		ret = fprintf(event_file_enable, "0");

	fclose(event_file_enable);

	return true;
}


int SensorApiService::SetPowerMode(int sensor_id, int mode)
{
	FILE *power_mode_fd = NULL;
	char power_mode_file_name[UCF_STR_LEN] = {'\0'};
	int ret = SENSOR_ERROR_CONTROL_FAILED;

	//Check the sensor for sensor_id
	for (int i=0 ; i < mSensorCount; i++) {
	  if (sensor_id == mSensor[i].sensor_id) {
	    if (mSensor[i].type == SENSOR_TYPE_ACCELEROMETER) {
		    if (mode == SENSOR_LPM && mMaxAccSampleRate > 208)
			    return SENSOR_ERROR_CONTROL_FAILED;
		    if (mSensor[i].Activate == SENSOR_ENABLE)
			    return SENSOR_ERROR_CONTROL_FAILED;
		    findPath(DYN_IIO_TYPE, power_mode_file_name, ASM330LHHX_ACC_SEARCH,
				    sizeof(power_mode_file_name));
	    }
	    if (mSensor[i].type == SENSOR_TYPE_GYROSCOPE) {
		    if (mode == SENSOR_LPM && mMaxGyroSampleRate > 208)
			    return SENSOR_ERROR_CONTROL_FAILED;
		    if (mSensor[i].Activate == SENSOR_ENABLE)
			    return SENSOR_ERROR_CONTROL_FAILED;
		    findPath(DYN_IIO_TYPE, power_mode_file_name, ASM330LHHX_GYRO_SEARCH,
				    sizeof(power_mode_file_name));
	    }
	  }
	}

	strlcat(power_mode_file_name, "power_mode", sizeof(power_mode_file_name));
	SENSOR_LOGI(LOG_TAG "power mode file name %s\n", power_mode_file_name);

	power_mode_fd = fopen(power_mode_file_name, "r+");
	if (!power_mode_fd) {
		SENSOR_LOGE(LOG_TAG "open");
		return ret;
	}

	if (mode == SENSOR_LPM) {
		SENSOR_LOGI(LOG_TAG "wrting 1 to power mode file\n");
		ret = fprintf(power_mode_fd, "1");
	}
	else {
		SENSOR_LOGI(LOG_TAG "wrting 0 to power mode file\n");
		ret = fprintf(power_mode_fd, "0");
	}

	fclose(power_mode_fd);

	SENSOR_LOGI(LOG_TAG "Power mode ret %d\n", ret);

	if (ret = 1)
		return SENSOR_RESPONSE_SUCCESS;
	else
		return SENSOR_ERROR_CONTROL_FAILED;
}


static void print_event(int iio_device_number, struct mlc_event_data *event)
{

	unsigned char *pevent;

	pevent = (unsigned char *)event;

	SENSOR_LOGI(LOG_TAG "Event: %x %x %x %x %x %x %x %x time: %lld\n",
			iio_device_number,
			pevent[0], pevent[1], pevent[2], pevent[3],
			pevent[4], pevent[5], pevent[6], pevent[7],
			event->timestamp);
}

/*
 *SensorApiService - MLCaseEvents Poll
 */
void* SensorApiService::mlcPollEvents(void *arg)
{
	SensorApiService* mSensorService = (SensorApiService*)(arg);
	mSensorService->pollEvents();
}

void SensorApiService::pollEvents(void) {

	char mlc_case_name[UCF_STR_LEN];
        int i = 0 , j = 0;
        int fd[10] = { -1 };
        int ret = 0;
        char device_path[UCF_STR_LEN];
        int event_fd[10] = { -1 };
        struct mlc_event_data event;
        struct pollfd pollfd_iio[10];
	int mlc_case_available = 0;

	for (j = mlc_case_device_number; j < 12; j++) {
	  if(find_mlc_case_iio_device_number(j)) {
                ret = snprintf(device_path, sizeof(device_path),
                        "/dev/iio:device%d", j);

		SENSOR_LOGI(LOG_TAG "opening device %s\n", device_path);

                fd[i] = open(device_path, 0);
                if (fd[i] == -1) {
			SENSOR_LOGE(LOG_TAG "opening device failed %s\n", device_path);
			continue;
                }

                ret = ioctl(fd[i], IIO_GET_EVENT_FD_IOCTL, &event_fd[i]);
                close(fd[i]);
                fd[i] = -1;

                if (ret == -1 || event_fd[i] == -1) {
			SENSOR_LOGE(LOG_TAG  "Failed to retrieve event fd\n")
				continue;
                }
		pollfd_iio[i].fd = event_fd[i];
		pollfd_iio[i].events = POLLIN;
		i++;
	  }
	}

	mlc_case_available = i;

	SENSOR_LOGI(LOG_TAG "MLC COUNT %d mlc_case_device_start_index %d\n", i,
			mlc_case_device_start_index);
	while (1) {
	    ret = poll(pollfd_iio, mlc_case_available, -1);
	    if(ret < 0)  {
		    SENSOR_LOGE(LOG_TAG "MLC POLL failed %d\n", ret);
		    break;
	    }
	    for(i = 0 ; i < mlc_case_available; i++) {
	        if (pollfd_iio[i].revents & POLLIN) {
		    ret = read(event_fd[i], &event, sizeof(event));
		    if (ret == -1) {
			    if (errno == EAGAIN) {
				    SENSOR_LOGE(LOG_TAG "MLC: nothing available\n");
				    continue;
			    } else {
				    SENSOR_LOGE(LOG_TAG "MLC: Failed to read event from device\n");
				    break;
			    }
		    }
		    print_event(i, &event);
		    get_mlc_case_name(i+mlc_case_device_start_index, mlc_case_name);
		    for (auto each : mClients) {
		       for (int i = 0; i < mSensorMlcCaseCount ; i++) {
			 if (each.second->mMlcCaseList != nullptr) {
			   if ((strcmp(each.second->mMlcCaseList[i].name, mlc_case_name) == 0 )
					   && each.second->mMlcCaseList[i].enable == 1) {
				   each.second->onSensorMlcCaseEventCb(mlc_case_name, &event);
			   }
			 }
		       }
		    }
		    memset(mlc_case_name, 0 ,sizeof(mlc_case_name));
		}
	    }
	}
}
