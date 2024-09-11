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
 * Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef SENSORCLIENTAPIIMPL_H
#define SENSORCLIENTAPIIMPL_H

#include <mutex>
#include <time.h>
#ifdef FEATURE_EXTERNAL_AP
#include <SensorQsocket.h>
#else  // FEATURE_EXTERNAL_AP
#include <SensorIpc.h>
#endif // FEATURE_EXTERNAL_AP
#include <SensorLog.h>
#include <SensorClientApi.h>
#include <SensorApiMsg.h>
#include <SensorDiagLog.h>

#ifdef NO_UNORDERED_SET_OR_MAP
    #include <set>
    #include <map>
#else
    #include <unordered_set>
    #include <unordered_map>
#endif

#undef LOG_TAG
#define LOG_TAG "SensorSvc_ClientApp:"

using namespace std;
using namespace sensor_util;

#ifdef FEATURE_EXTERNAL_AP
using sensor_util::SensorQsocket;
using sensor_util::SensorQsocketSender;
#else  // FEATURE_EXTERNAL_AP
using sensor_util::SensorIpc;
using sensor_util::SensorIpcSender;
#endif // FEATURE_EXTERNAL_AP

namespace sensor_client
{

struct MlcCaseListCb {
      char name[100];
      bool enable;
      SensorMLCEventCb mSensorMLCEventCb;
};

struct SensorTrackingOption {
	int sensor_id;
	float sampling_rate;
	int batch_count;
	sensor_state state;
	SensorDataReadCb mSensorDataReadCb;
};

class SensorClientImpl :
#ifdef FEATURE_EXTERNAL_AP
    public SensorQsocket
#else // FEATURE_EXTERNAL_AP
    public SensorIpc
#endif // FEATURE_EXTERNAL_AP
{
public:
    SensorClientImpl(CapabilitiesCb capabitiescb);
    void destroy();
    //Get Sensor List
    virtual int getSensorList(struct sensor_list***, int *sensor_count);
    //Sensor Control
    virtual int sensorControl(int sensor_id, sensor_state state);
    //Sensor Config
    virtual int startBatching(int sensor_id, float sampling_rate, int batch_count, BatchingCb);
    //Sensor Read Events
    virtual int startTracking(int sensor_id, SensorDataReadCb);
    //Sensor request MLC case supported
    virtual int sensorRequestMLC(struct sensor_mlc_case_list ***m, int *mlc_case_count);
    //Sensor Enable/Disable MLC case
    virtual int sensorMLCEventEnable(char *mlc_case_name, bool enable, SensorMLCEventCb, SensormFifoReadCb);
    //Sensor Temperature Read
    virtual int readTemperature(SensorTempReadCb);
    //Sensor Buffer Read
    virtual int startBufferDataRead(bool enable, SensorBufferDataReadCb);
    //Sensor Self Test
    virtual int selfTest(int sensor_id, SelfTestType selfTestType, int request_id, SelfTestResultCallback);
    // convenient methods
    inline bool sendMessage(const uint8_t* data, uint32_t length) const {
        return (mIpcSender != nullptr) && mIpcSender->send(data, length);
    }
private:
    ~SensorClientImpl();

    //timeout function
    inline struct timespec timeout(int sec) {
	    struct timespec ts;
	    clock_gettime(CLOCK_MONOTONIC, &ts);
	    ts.tv_sec +=  sec;
	    return ts;
    }

    //Sensor Reconnection API
    bool SensorReconfigure(bool enable);

    // override from SensorIpc
    virtual void onListenerReady() override;
    virtual void onReceive(const string& data) override;
#ifdef FEATURE_EXTERNAL_AP
    virtual void onServiceStatusChange(int serviceId, int instanceId, int status, const SensorQsocketSender& refSender) override;
#endif

    // internal session parameter
    struct sensor_list*     mSensorList;
    int 		    mSensorCount;
    static uint32_t         mClientIdGenerator;
    static mutex            mMutex;
    uint32_t                mClientId;
    bool                    mHalRegistered;
    char                    mSocketName[MAX_SOCKET_PATHNAME_LENGTH];
    bool 		    mShdRestarted;
    bool                    mEapClient;
    // for client on a different processor, 0 is invalid
    uint32_t                mInstanceId;

    //MLC case list
    int 		             mSensorMlcCaseCount;
    struct sensor_mlc_case_list*     mSensorMlcCaseList;

    // callbacks
    CapabilitiesCb          mCapabilitiesCb;
    BatchingCb              mBatchingCb;
    SensorTempReadCb        mSensorTempReadCb;
    SensorBufferDataReadCb  mSensorBufferDataReadCb;
    SensorTrackingOption*   mSensorTrackingOption;
    MlcCaseListCb*          mSensorMLCEventCbs;
    SensormFifoReadCb       mSensormFifoReadCb;
    SelfTestResultCallback  mSelfTestResultCb;

    //Ipc sender
#ifdef FEATURE_EXTERNAL_AP
    SensorQsocketSender*       mIpcSender;
#else  // FEATURE_EXTERNAL_AP
    SensorIpcSender*          mIpcSender;
#endif // FEATURE_EXTERNAL_AP

    //Response and to wake up Sensor API
    volatile int       mRespReturn;
    struct timespec    mTimeout;
    pthread_mutex_t    mSensorLibMutex;
    pthread_cond_t     mSensorLibCond;
    pthread_condattr_t mSensorLibattr;

    //Diag
    SensorDiagLog mDiagLogger;
};

} // namespace sensor_client

#endif /* SENSORCLIENTAPIIMPL_H */
