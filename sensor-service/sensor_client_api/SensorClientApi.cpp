/* Copyright (c) 2021, The Linux Foundation. All rights reserved.
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
 *     * Neither the name of The Linux Foundation nor the names of its
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
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 */
#include <SensorClientApi.h>
#include <SensorClientApiImpl.h>

using std::string;

namespace sensor_client {

static int DebugLevel = 0;

/******************************************************************************
SensorClient - constructor
******************************************************************************/
SensorClient::SensorClient(CapabilitiesCb capabitiescb) {

#ifdef USE_DLT
    dltLogInit("SENC", "SCL", "Sensor Client Library");
#endif
    DebugLevel = SensorReadDebugLevel();
    SENSOR_LOGI(LOG_TAG "debug_level %d\n", DebugLevel);
    mApiImpl = new SensorClientImpl(capabitiescb);
}

/******************************************************************************
SensorClient - Destructor
******************************************************************************/
SensorClient::~SensorClient() {
    if (mApiImpl) {
        // two steps processes due to asynchronous message processing
        mApiImpl->destroy();
        // deletion of mApiImpl will be done after messages in the queue are processed
#ifdef USE_DLT
        dltLogDeInit();
#endif
    }
}

/******************************************************************************
SensorClient - SensorList
******************************************************************************/
int SensorClient::get_sensor_list(struct sensor_list **s, int *sensor_count) {
    //Chek for Client Register
    if (!mApiImpl) {
        SENSOR_LOGE(LOG_TAG "NULL mApiImpl\n");
        return SENSOR_ERROR_CLIENT_REGISTER_FAILED;
    }
    return mApiImpl->getSensorList(&s, sensor_count);
}

/******************************************************************************
SensorClientApi - SensorControl
******************************************************************************/
int SensorClient::sensor_control(int sensor_id, sensor_state state) {
    //Chek for Client Register
    if (!mApiImpl) {
        SENSOR_LOGE(LOG_TAG "NULL mApiImpl\n");
        return SENSOR_ERROR_CLIENT_REGISTER_FAILED;
    }
    return mApiImpl->sensorControl(sensor_id, state);
}

/******************************************************************************
SensorClient - SensorConfig
******************************************************************************/
int SensorClient::sensor_config(int sensor_id, float sampling_rate, int batch_count, bool rotate, BatchingCb batchingCallback) {
    //Chek for Client Register
    if (!mApiImpl) {
        SENSOR_LOGE(LOG_TAG "NULL mApiImpl\n");
        return SENSOR_ERROR_CLIENT_REGISTER_FAILED;
    }
    //Input parameter check
    if (!batchingCallback) {
        SENSOR_LOGE(LOG_TAG "NULL BatchingCb\n");
        return SENSOR_ERROR_CALLBACK_MISSING;
    }
    return mApiImpl->startBatching(sensor_id, sampling_rate, batch_count, rotate, batchingCallback);
}

/******************************************************************************
SensorClient - SensorReadEvents
******************************************************************************/
int SensorClient::sensor_read_events(int sensor_id, SensorDataReadCb sensorreadCallback) {
    //Chek for Client Register
    if (!mApiImpl) {
        SENSOR_LOGE(LOG_TAG "NULL mApiImpl\n");
        return SENSOR_ERROR_CLIENT_REGISTER_FAILED;
    }
    //Input parameter check
    if (!sensorreadCallback) {
        SENSOR_LOGE(LOG_TAG "NULL SensorDataReadCb\n");
        return SENSOR_ERROR_CALLBACK_MISSING;
    }
    return mApiImpl->startTracking(sensor_id, sensorreadCallback);
}
/******************************************************************************
SensorClientApi - SensorRequestMLC
******************************************************************************/
int SensorClient::sensor_request_mlc_case(struct sensor_mlc_case_list **m, int *mlc_case_count) {
    //Chek for Client Register
    if (!mApiImpl) {
        SENSOR_LOGE(LOG_TAG "NULL mApiImpl\n");
        return SENSOR_ERROR_CLIENT_REGISTER_FAILED;
    }

    return mApiImpl->sensorRequestMLC(&m, mlc_case_count);
}

/******************************************************************************
SensorClientApi - SensorMLCEventEnable
******************************************************************************/
int SensorClient::sensor_mlc_event_enable(char *mlc_case_name, bool enable,
		SensorMLCEventCb sensorMlcEventCallback, SensormFifoReadCb sensorMfifoReadCallback) {
    //Chek for Client Register
    if (!mApiImpl) {
        SENSOR_LOGE(LOG_TAG "NULL mApiImpl\n");
        return SENSOR_ERROR_CLIENT_REGISTER_FAILED;
    }

    //Input parameter check
    if (!sensorMlcEventCallback) {
        SENSOR_LOGE(LOG_TAG "NULL sensorMlcEventCallback\n");
        return SENSOR_ERROR_CALLBACK_MISSING;
    }

    return mApiImpl->sensorMLCEventEnable(mlc_case_name, enable,
		    sensorMlcEventCallback, sensorMfifoReadCallback);
}

/******************************************************************************
SensorClient - SensorTemperature
******************************************************************************/
int SensorClient::sensor_read_temperature(SensorTempReadCb sensortempreadCallback) {
    //Chek for Client Register
    if (!mApiImpl) {
        SENSOR_LOGE(LOG_TAG "NULL mApiImpl\n");
        return SENSOR_ERROR_CLIENT_REGISTER_FAILED;
    }
    //Input parameter check
    if (!sensortempreadCallback) {
        SENSOR_LOGE(LOG_TAG "NULL SensorTempReadCb\n");
        return SENSOR_ERROR_CALLBACK_MISSING;
    }
    return mApiImpl->readTemperature(sensortempreadCallback);
}

/******************************************************************************
SensorClient - SensorBufferRead
******************************************************************************/
int SensorClient::sensor_read_buffer_data(bool enable, SensorBufferDataReadCb sensorbufferreadCallback) {
    //Chek for Client Register
    if (!mApiImpl) {
        SENSOR_LOGE(LOG_TAG "NULL mApiImpl\n");
        return SENSOR_ERROR_CLIENT_REGISTER_FAILED;
    }
    //Input parameter check
    if (!sensorbufferreadCallback) {
        SENSOR_LOGE(LOG_TAG "NULL SensorBufferDataReadCb\n");
        return SENSOR_ERROR_CALLBACK_MISSING;
    }
    return mApiImpl->startBufferDataRead(enable, sensorbufferreadCallback);
}

/******************************************************************************
SensorClient - SensorSelfTest
******************************************************************************/
int SensorClient::sensor_self_test(int sensor_id, SelfTestType selfTestType, int request_id, SelfTestResultCallback selftestResultCallback) {
    //Chek for Client Register
    if (!mApiImpl) {
        SENSOR_LOGE(LOG_TAG "NULL mApiImpl\n");
        return SENSOR_ERROR_CLIENT_REGISTER_FAILED;
    }

    //Input parameter check
    if (!selftestResultCallback) {
        SENSOR_LOGE(LOG_TAG "NULL SelfTestResultCallback\n");
        return SENSOR_ERROR_CALLBACK_MISSING;
    }
    return mApiImpl->selfTest(sensor_id, selfTestType, request_id, selftestResultCallback);
}

/******************************************************************************
SensorClient - SensorUpdate Rotation Matrix
******************************************************************************/
int SensorClient::sensor_update_rotation_matrix(uint16_t rolld, uint16_t pitchd, uint16_t yawd) {
    //Chek for Client Register
    if (!mApiImpl) {
        SENSOR_LOGE(LOG_TAG "NULL mApiImpl\n");
        return SENSOR_ERROR_CLIENT_REGISTER_FAILED;
    }
    return mApiImpl->setEulerAngles(rolld, pitchd, yawd);
}

} // namespace sensor_client
