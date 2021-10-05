/*
 * Copyright (C) 2015-2018 STMicroelectronics
 * Author: Denis Ciocca - <denis.ciocca@st.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
Changes from Qualcomm Innovation Center are provided under the following license:

Copyright (c) 2021 Qualcomm Innovation Center, Inc. All rights reserved.
 
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the
disclaimer below) provided that the following conditions are met:
 
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
 
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
 
    * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.
 
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ANDROID_SENSOR_HAL_COMMON_DATA
#define ANDROID_SENSOR_HAL_COMMON_DATA

#include <hardware/sensors.h>

#include "../configuration.h"
#include <utils/Log.h>

#define ST_HAL_GRAVITY_MAX_ON_EART		(10.7f)

/* Android version */
#define ST_HAL_KITKAT_VERSION			(0)
#define ST_HAL_LOLLIPOP_VERSION			(1)
#define ST_HAL_MARSHMALLOW_VERSION		(2)
#define ST_HAL_NOUGAT_VERSION			(3)
#define ST_HAL_OREO_VERSION			(4)
#define ST_HAL_PIE_VERSION			(5)
#define ST_HAL_10_VERSION			(6)

#define CONCATENATE_STRING(x, y)		(x y)

#define ST_HAL_DATA_PATH			"/data/STSensorHAL"
#define ST_HAL_PRIVATE_DATA_PATH		"/data/STSensorHAL/private_data.dat"
#define ST_HAL_FACTORY_DATA_PATH		"/data/STSensorHAL/factory_calibration"
#define ST_HAL_FACTORY_ACCEL_DATA_FILENAME	CONCATENATE_STRING(ST_HAL_FACTORY_DATA_PATH, "/accel.txt")
#define ST_HAL_FACTORY_GYRO_DATA_FILENAME	CONCATENATE_STRING(ST_HAL_FACTORY_DATA_PATH, "/gyro.txt")

#define SENSOR_TYPE_ST_CUSTOM_NO_SENSOR		(SENSOR_TYPE_DEVICE_PRIVATE_BASE + 20)

#define ST_HAL_IIO_MAX_DEVICES			(50)

#define SENSOR_DATA_X(datax, datay, dataz, x1, y1, z1, x2, y2, z2, x3, y3, z3) \
		     ((x1 == 1 ? datax : (x1 == -1 ? -datax : 0)) + \
		      (x2 == 1 ? datay : (x2 == -1 ? -datay : 0)) + \
		      (x3 == 1 ? dataz : (x3 == -1 ? -dataz : 0)))

#define SENSOR_DATA_Y(datax, datay, dataz, x1, y1, z1, x2, y2, z2, x3, y3, z3) \
		     ((y1 == 1 ? datax : (y1 == -1 ? -datax : 0)) + \
		      (y2 == 1 ? datay : (y2 == -1 ? -datay : 0)) + \
		      (y3 == 1 ? dataz : (y3 == -1 ? -dataz : 0)))

#define SENSOR_DATA_Z(datax, datay, dataz, x1, y1, z1, x2, y2, z2, x3, y3, z3) \
		     ((z1 == 1 ? datax : (z1 == -1 ? -datax : 0)) + \
		      (z2 == 1 ? datay : (z2 == -1 ? -datay : 0)) + \
		      (z3 == 1 ? dataz : (z3 == -1 ? -dataz : 0)))

#define SENSOR_X_DATA(...)			SENSOR_DATA_X(__VA_ARGS__)
#define SENSOR_Y_DATA(...)			SENSOR_DATA_Y(__VA_ARGS__)
#define SENSOR_Z_DATA(...)			SENSOR_DATA_Z(__VA_ARGS__)

#define ST_HAL_DEBUG_INFO			(1)
#define ST_HAL_DEBUG_VERBOSE			(2)
#define ST_HAL_DEBUG_EXTRA_VERBOSE		(3)

/* Overrite default log utility */
#ifdef PLTF_LINUX_ENABLED
//#define ALOGV(fmt, ...) fprintf(stdout, "[VERBOSE] %s(%u): " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
//#define ALOGD(fmt, ...) fprintf(stdout, "[DEBUG] %s(%u): " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
//#define ALOGW(fmt, ...) fprintf(stdout, "[WARNING] %s(%u): " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
//#define ALOGE(fmt,...)  fprintf(stderr, "[ERROR] %s(%u): " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
//#define ALOGV(fmt, ...) ALOGV("[VERBOSE] %s(%u): " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
//#define ALOGD(fmt, ...) ALOGD("[DEBUG] %s(%u): " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
//#define ALOGW(fmt, ...) ALOGW("[WARNING] %s(%u): " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
//#define ALOGE(fmt,...)  ALOGE("[ERROR] %s(%u): " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#endif

#endif /* ANDROID_SENSOR_HAL_COMMON_DATA */
