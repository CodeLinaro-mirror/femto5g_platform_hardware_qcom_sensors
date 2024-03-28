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

#ifndef ST_HWSENSOR_BASE_H
#define ST_HWSENSOR_BASE_H

#include <poll.h>
#include <math.h>

#include "SensorBase.h"

extern "C" {
	#include "utils.h"
};

#define HW_SENSOR_BASE_DEFAULT_IIO_BUFFER_LEN	(2)
#define HW_SENSOR_BASE_IIO_SYSFS_PATH_MAX	(50)
#define HW_SENSOR_BASE_IIO_DEVICE_NAME_MAX	(30)
#define HW_SENSOR_BASE_MAX_CHANNELS		(8)

struct HWSensorBaseCommonData {
	char device_iio_sysfs_path[HW_SENSOR_BASE_IIO_SYSFS_PATH_MAX];
	char device_name[HW_SENSOR_BASE_IIO_DEVICE_NAME_MAX];
	unsigned int device_iio_dev_num;

	int num_channels;
	struct device_iio_info_channel channels[HW_SENSOR_BASE_MAX_CHANNELS];

	struct device_iio_scales sa;
} typedef HWSensorBaseCommonData;


class HWSensorBase;
class HWSensorBaseWithPollrate;

/*
 * class HWSensorBase
 */
class HWSensorBase : public SensorBase {
private:
protected:
	ssize_t scan_size;
	struct pollfd pollfd_iio[2];
	FlushRequested flush_requested;
	HWSensorBaseCommonData common_data;
	ChangeODRTimestampStack odr_switch;
#ifdef CONFIG_ST_HAL_FACTORY_CALIBRATION
	bool factory_calibration_updated;
	float factory_offset[3];
	float factory_scale[3];
#endif /* CONFIG_ST_HAL_FACTORY_CALIBRATION */
#if (CONFIG_ST_HAL_ANDROID_VERSION >= ST_HAL_MARSHMALLOW_VERSION)
	uint8_t *injection_data;
#endif /* CONFIG_ST_HAL_ANDROID_VERSION */
	bool has_event_channels;

	int WriteBufferLenght(unsigned int buf_len);

public:
	HWSensorBase(HWSensorBaseCommonData *data, const char *name,
		     int handle, int sensor_type, unsigned int hw_fifo_len,
		     float power_consumption);
	virtual ~HWSensorBase();

	virtual int Enable(int handle, bool enable, bool lock_en_mute);
	virtual int SetDelay(int handle, int64_t period_ns,
			     int64_t timeout, bool lock_en_mute);

	virtual int AddSensorDependency(SensorBase *p);
	virtual void RemoveSensorDependency(SensorBase *p);

	int ApplyFactoryCalibrationData(char *filename,
					time_t *last_modification);

	virtual void ProcessEvent(struct device_iio_events *event_data);
	virtual int FlushData(int handle, bool lock_en_mute);
	virtual void ProcessFlushData(int handle, int64_t timestamp);
	virtual void ThreadDataTask();
	virtual void ThreadEventsTask();

#if (CONFIG_ST_HAL_ANDROID_VERSION >= ST_HAL_MARSHMALLOW_VERSION)
	virtual int InjectionMode(bool enable);
	virtual int InjectSensorData(const sensors_event_t *data);
#endif /* CONFIG_ST_HAL_ANDROID_VERSION */
	bool hasEventChannels() { return has_event_channels; }
	bool hasDataChannels() { return common_data.num_channels > 0; }

#ifdef PLTF_LINUX_ENABLED
	/* set engine ignition status (on/off) */
	virtual int Ignition(int val);
#endif /* PLTF_LINUX_ENABLED */
};


/*
 * class HWSensorBaseWithPollrate
 */
class HWSensorBaseWithPollrate : public HWSensorBase {
private:
	struct device_iio_sampling_freqs sampling_frequency_available;

public:
	HWSensorBaseWithPollrate(HWSensorBaseCommonData *data,
				 const char *name,
				 struct device_iio_sampling_freqs *sfa,
				 int handle,
				 int sensor_type,
				 unsigned int hw_fifo_len,
				 float power_consumption);
	virtual ~HWSensorBaseWithPollrate();

	virtual int SetDelay(int handle,
			     int64_t period_ns,
			     int64_t timeout,
			     bool lock_en_mute);
	virtual int FlushData(int handle, bool lock_en_mute);
	virtual void WriteDataToPipe(int64_t hw_pollrate);
};

#endif /* ST_HWSENSOR_BASE_H */
