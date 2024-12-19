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

#ifndef SENSORHAL_CLIENT_HANDLER_H
#define SENSORHAL_CLIENT_HANDLER_H

#include <queue>
#include <mutex>

#ifdef NO_UNORDERED_SET_OR_MAP
    #include <map>
#else
    #include <unordered_map>
#endif

#include <SensorIpc.h>
#include <SensorApiMsg.h>
#include <SensorHalDaemonIPCSender.h>
#ifdef SENSOR_IVSS_ENABLED
#include <SensorInterfaceStubImpl.hpp>
#endif
#undef LOG_TAG
#define LOG_TAG "SensorSvc_HalDaemon:"

using namespace sensor_socket;

// forward declaration
class SensorApiService;
class SensorInterfaceStubImpl;


struct mlc_case_list {
	char name[100];
	int  enable;
};

/******************************************************************************
SensorHalDaemonClientHandler
******************************************************************************/
class SensorHalDaemonClientHandler
{
public:
    //Constructor of SensorHalDaemonClientHandler class
    inline SensorHalDaemonClientHandler(SensorApiService* service, const std::string& clientname, ClientType clientType, int SensorCount) :
            mService(service),
	    mName(clientname),
	    mSensorCount(SensorCount),
	    mClientType(clientType),
	    mServiceId(-1),
	    mInstanceId(-1),
	    mTracking(false),
	    mAccTracking(false),
	    mGyroTracking(false),
	    mActivate(nullptr),
	    mIpcSender(nullptr),
	    mAccEvents(nullptr),
	    mGyroEvents(nullptr),
	    mMlcCaseList(nullptr),
	    mMlcEnable(false),
	    mAccFactor(0),
	    mGyroFactor(0),
	    mAccCount(0),
	    mGyroCount(0),
	    mAccMovingCount(0),
	    mGyroMovingCount(0),
	    mAccBatchCount(0),
	    mGyroBatchCount(0),
	    mBufferRead(-1),
	    mAccRotate(1),
	    mGyroRotate(1)
    {
	    SENSOR_LOGI(LOG_TAG "new SensorHalDaemonClientHandler \n");
	    if(strncmp(mName.c_str(),"tosomeip",sizeof(mName.c_str())) != 0) {
	    mIpcSender = new SensorHalDaemonIPCSender(mName.c_str());
	    // Create a file name with instanceId. The file handle
            // will be used by hal daemon when it crashes to figure out
            // the running clients.
            if (strncmp(mName.c_str(), SOCKET_SENSOR_CLIENT_DIR,
                sizeof(SOCKET_SENSOR_CLIENT_DIR)-1) != 0 ) {

                char fileName[MAX_SOCKET_PATHNAME_LENGTH];
                snprintf (fileName, sizeof(fileName), "%s%s",
                          SOCKET_TO_EXTERANL_AP_SENSOR_CLIENT_BASE, mName.c_str());
                SENSOR_LOGI(LOG_TAG "<-- attempt to open file %s\n", fileName);
		FILE *fd;
		fd = fopen (fileName, "w");
                if (nullptr == fd) {
                    SENSOR_LOGE(LOG_TAG "<-- failed to open file %s\n", fileName);
		    exit(1);
                }
		fclose(fd);
		getId1Id2(mName.c_str(), mName.length(),
				mServiceId, mInstanceId);
		SENSOR_LOGI("EAP client: clientname %s, service id: %d, instance id: %d",
				mName.c_str(), mServiceId, mInstanceId);
            }
            }

	    //Intialise the client parameters and set to zero
	    mActivate = new (std::nothrow) int[mSensorCount];
	    if (mActivate == nullptr) {
		return;
	    }

	    for(int i=0; i < mSensorCount; i++) {
		    mActivate[i]=0;
	    }
    }

    // public APIs
    void cleanup();
    void onSensorListCb(struct sensor_list *s, int count);
    void onSensorActivateCb(int sensor_id, int enable);
    void onSensorBatchingCb(int sensor_id, float SamplingRate, int BatchingRate, bool Rotate);
    void onResponseCb(int ret, ESensorMsgID id);
    bool onSensorDataReadCb(sensors_event_t *events, int count);
    bool onSensorBufferDataReadCb(sensors_event_t *events, int count);
    void onSensorTempCb(float temperature);
    bool onCapabilitiesCallback(SensorCapabilitiesMask mask);
    void onSensorSelfTestResultCb(int sensor_id, int request_id, SelfTestResult result, SelfTestResultType resultType, uint64_t timestamp);

    //MLC public APIs
    void onSensorMlcCaseListCb(struct sensor_mlc_case_list *s, int count);
    bool onSensorMlcCaseEventCb(char *case_name, struct mlc_event_data *event);
    bool onSensorMFifoDataReadCb(sensors_event_t *events, int count);

    //To check Tracking status of client
    bool    mTracking;
    bool    mAccTracking;
    bool    mGyroTracking;
    bool    mAccRotate;
    bool    mGyroRotate;

    //Client config parameters of sensor
    int*    mActivate;
    int     mAccBatchCount;
    int     mGyroBatchCount;
    int     mAccFactor;
    int     mGyroFactor;

    //Parameters used to calculate moving avg of sensor
    int mAccCount;
    int mGyroCount;
    int mAccMovingCount;
    int mGyroMovingCount;

    //Sensors_event pointer to store the samples till samples reached requested batch count.
    sensors_event_t *mAccEvents;
    sensors_event_t *mGyroEvents;

    //MLC LIST for clients
    struct mlc_case_list *mMlcCaseList;
    bool mMlcEnable;

    //To Check Buffer read or delete status
    int mBufferRead;

    // name of this client
    const std::string mName;

    //Queue used to send response message
    std::queue<ESensorMsgID> mPendingMessages;

    inline int getServiceId() {return mServiceId;}  // for EAP client
    inline int getInstanceId() {return mInstanceId;} // for EAP client
private:
    //Destructor of SensorHalDaemonClientHandler class
    inline ~SensorHalDaemonClientHandler() {}
    template <typename MESSAGE>
    bool sendMessage(const MESSAGE& msg) {
        bool retVal= sendMessage(reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
        if (retVal == false) {
            SENSOR_LOGE(LOG_TAG "failed: client %s, msg id: %d, err %s\n",
                     mName.c_str(), ((SensorAPIMsgHeader) msg).msgId, strerror(errno));
        }
        return retVal;
    }

    // send ipc message to this client for serialized payload
    bool sendMessage(const uint8_t* pmsg, size_t msglen) {
	if (mIpcSender) {
	 bool retVal= mIpcSender->send(pmsg, msglen);
	 if (retVal == false) {
		 SENSOR_LOGE(LOG_TAG "failed: client %s, msg id: %d, err %s\n",
				 mName.c_str(), ((SensorAPIMsgHeader*) pmsg)->msgId, strerror(errno));
	 }
	 return retVal;
	}
	return false;
    }

    //To Send Sensor events to client once sample count reached to requested count.
    bool SendDataToClient(sensors_event_t *e, int count);
    void StoreMlcCaseListStatus(struct sensor_mlc_case_list *s, int count);
    // pointer to parent service
    SensorApiService* mService;
    //The total number of sensor supported
    int mSensorCount;

    ClientType mClientType;
    int mServiceId;  // For EAP client
    int mInstanceId; // For EAP client

    SensorHalDaemonIPCSender* mIpcSender;
};

#endif //SENSORHAL_CLIENT_HANDLER_H
