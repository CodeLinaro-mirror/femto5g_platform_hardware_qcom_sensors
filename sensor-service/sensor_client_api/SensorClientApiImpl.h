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
 */
#ifndef SENSORCLIENTAPIIMPL_H
#define SENSORCLIENTAPIIMPL_H

#include <mutex>

#include <SensorIpc.h>
#include <SensorLog.h>
#include <SensorClientApi.h>
#include <SensorApiMsg.h>

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

using sensor_util::SensorIpc;
using sensor_util::SensorIpcSender;

namespace sensor_client
{

struct MlcCaseListCb {
      char name[100];
      SensorMLCEventCb mSensorMLCEventCb;
};

class SensorClientImpl :
    public SensorIpc
{
public:
    SensorClientImpl();
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
    virtual int sensorMLCEventEnable(char *mlc_case_name, bool enable, SensorMLCEventCb);
    //Sensor Temperature Read
    virtual int readTemperature(SensorTempReadCb);
    //Sensor Buffer Read
    virtual int startBufferDataRead(bool enable, SensorBufferDataReadCb);
    // convenient methods
    inline bool sendMessage(const uint8_t* data, uint32_t length) const {
        return (mIpcSender != nullptr) && mIpcSender->send(data, length);
    }
private:
    ~SensorClientImpl();

    // override from SensorIpc
    virtual void onListenerReady() override;
    virtual void onReceive(const string& data) override;

    // internal session parameter
    struct sensor_list*     mSensorList;
    int 		    mSensorCount;
    static uint32_t         mClientIdGenerator;
    static mutex            mMutex;
    uint32_t                mClientId;
    bool                    mHalRegistered;
    char                    mSocketName[MAX_SOCKET_PATHNAME_LENGTH];
    //MLC case list
    int 		             mSensorMlcCaseCount;
    struct sensor_mlc_case_list*     mSensorMlcCaseList;

    // callbacks
    BatchingCb              mBatchingCb;
    SensorDataReadCb        mSensorDataReadCb;
    SensorTempReadCb        mSensorTempReadCb;
    SensorBufferDataReadCb  mSensorBufferDataReadCb;
    struct MlcCaseListCb    *mSensorMLCEventCbs;

    //Response
    volatile bool onResponse;
    volatile int  mRespReturn;

    SensorIpcSender*          mIpcSender;

    //To wake up Sensor Api
    pthread_mutex_t mSensorLibMutex;
    pthread_cond_t mSensorLibCond;
};

} // namespace sensor_client

#endif /* SENSORCLIENTAPIIMPL_H */
