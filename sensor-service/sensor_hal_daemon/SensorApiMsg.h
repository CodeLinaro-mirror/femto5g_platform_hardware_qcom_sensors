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
 */

#ifndef SENSORAPIMSG_H
#define SENSORAPIMSG_H

#include <string>
#include <memory>
#include <algorithm>
#include <glib.h>
#include <stdint.h>
#include <functional>
#include <SensorIpc.h>
#include <SensorLog.h>
#include <sensors.h>
#include <SensorList.h>

#undef LOG_TAG
#define LOG_TAG "SensorSvc_ApiMsg:"

/******************************************************************************
Constants
******************************************************************************/
#define BUFFER_EVENT    2048
#define SENSOR_REMOTE_API_MSG_VERSION (1)

//Max Batch Count supported by SHD
#define MAX_BATCH_COUNT 50

#define strlcpy g_strlcpy
#define strlcat g_strlcat

#define UID_SENSOR (3011)
#define GID_SENSORCLIENT (3011)

#define SOCKET_SENSOR_CLIENT_DIR     "/dev/socket/sensor_client/"
//#define EAP_SENSOR_CLIENT_DIR        "/dev/socket/sensor_client/"
#define SOCKET_TO_SENSOR_HAL_DAEMON  "/dev/socket/sensor_client/hal_daemon"
#define SOCKET_TO_SENSOR_CLIENT_BASE "/dev/socket/sensor_client/toclient"
#define SOCKET_TO_EXTERANL_AP_LOCATION_CLIENT_BASE "/dev/socket/sensor_client/extap.toclient"

// Maximum fully qualified path(including the file name)
// for the sensor remote API service and client socket name
#define MAX_SOCKET_PATHNAME_LENGTH (128)

#define SENSOR_CLIENT_SESSION_ID_INVALID (0)

#define SENSOR_CLIENT_API_QSOCKET_HALDAEMON_SERVICE_ID    (8001)
#define SENSOR_CLIENT_API_QSOCKET_HALDAEMON_INSTANCE_ID   (1)
#define SENSOR_CLIENT_API_QSOCKET_CLIENT_SERVICE_ID       (8002)

using namespace std;
using namespace sensor_util;

static const char SERVICE_NAME[] = "sensorapiservice";

enum ClientType {
    SENSOR_CLIENT_API = 1,
};

/******************************************************************************
List of message IDs supported by Sensor Remote API
******************************************************************************/
enum ESensorMsgID {
    E_SENSORAPI_UNDEFINED_MSG_ID = 0,

    // registration
    E_SENSORAPI_CLIENT_REGISTER_MSG_ID = 1,
    E_SENSORAPI_CLIENT_DEREGISTER_MSG_ID = 2,
    E_SENSORAPI_CAPABILILTIES_MSG_ID = 3,
    E_SENSORAPI_HAL_READY_MSG_ID = 4,

    // tracking session
    E_SENSORAPI_START_TRACKING_MSG_ID = 5,
    E_SENSORAPI_STOP_TRACKING_MSG_ID = 6,

    //Sensor Data indication message id
    E_SENSORAPI_DATA_READ_MSG_ID = 7,

    // batching session
    E_SENSORAPI_START_BATCHING_MSG_ID = 8,
    E_SENSORAPI_START_BATCHING_RES_ID = 9,
    E_SENSORAPI_STOP_BATCHING_MSG_ID = 10,

    // Get Sensor LIST
    E_SENSORAPI_GET_SENSOR_LIST_MSG_ID = 11,
    E_SENSORAPI_SENSOR_LIST_MSG_ID = 12,

    //Enable Sensor
    E_SENSORAPI_SENSOR_ENABLE_MSG_ID = 13,

    //MLC Message ID
    E_SENSORAPI_SENSOR_MLC_CASE_LIST_MSG_ID = 14,
    E_SENSORAPI_SENSOR_MLC_CASE_ENABLE_MSG_ID = 15,
    E_SENSORAPI_SENSOR_MLC_EVENT_IND_MSG_ID = 16,

    //Read Sensor temperature
    E_SENSORAPI_SENSOR_TEMP_REQ_MSG_ID = 17,
    E_SENSORAPI_SENSOR_TEMP_IND_MSG_ID = 18,

    //Buffer data
    E_SENSORAPI_SENSOR_BUFFER_REQ_MSG_ID = 19,
    E_SENSORAPI_SENSOR_BUFFER_IND_MSG_ID = 20,

    //mFifo data
    E_SENSORAPI_SENSOR_MFIFO_IND_MSG_ID = 21,

    //selfTest
    E_SENSORAPI_SENSOR_SELFTEST_REQ_MSG_ID = 22,
    E_SENSORAPI_SENSOR_SELFTEST_IND_MSG_ID = 23
};


/******************************************************************************
Common data structure
******************************************************************************/
struct SensorDataPayload {
    uint32_t count;
    sensors_event_t events[1];
};

struct SensorList {
    uint32_t count;
    struct sensor_list s[1];
};

struct SensorMlcCaseList {
    uint32_t count;
    struct sensor_mlc_case_list s[1];
};

struct SensorMlcEventData {
    char name[100];
    struct mlc_event_data event[1];
};

static int getId1Id2(const char* fullPathName, int32_t length, int32_t& id1, int32_t& id2) {
        int32_t indx = 0;
	sscanf(fullPathName, "%d.%d", &id1, &id2);
        return indx;
}

/******************************************************************************
  IPC message header structure
 ******************************************************************************/
struct SensorAPIMsgHeader
{
    char          mSocketName[MAX_SOCKET_PATHNAME_LENGTH]; /**< Processor string */
    ESensorMsgID  msgId;               /**< SensorMsgID */
    uint32_t      msgVersion;          /**< Sensor remote API message version */

    inline SensorAPIMsgHeader(const char* name, ESensorMsgID msgId):
	    msgId(msgId),
	    msgVersion(SENSOR_REMOTE_API_MSG_VERSION) {
		    memset(mSocketName, 0, MAX_SOCKET_PATHNAME_LENGTH);
		    strlcpy(mSocketName, name, MAX_SOCKET_PATHNAME_LENGTH);
	    }

    inline bool isValidMsg(uint32_t msgSize) {
	    bool msgValid = true;
	    if (msgSize < sizeof(SensorAPIMsgHeader)) {
		    SENSOR_LOGE(LOG_TAG "payload size %d smaller than minimum payload size %d\n",
				    msgSize, sizeof(SensorAPIMsgHeader));
		    msgValid = false;
	    } else if (msgVersion != SENSOR_REMOTE_API_MSG_VERSION) {
		    SENSOR_LOGE(LOG_TAG "msg id %d, msg version %d not matching with expected version %d\n",
				    msgId, msgVersion, SENSOR_REMOTE_API_MSG_VERSION);
		    msgValid = false;
	    }
                return msgValid;
        }

    bool isValidClientMsg(uint32_t msgSize) {
	    bool msgValid = isValidMsg(msgSize);
	    if ((true== msgValid) &&
			    ((strncmp(mSocketName, SOCKET_SENSOR_CLIENT_DIR,
				      sizeof(SOCKET_SENSOR_CLIENT_DIR)-1) != 0))) {
		    SENSOR_LOGE(LOG_TAG "msg not from expected client\n");
		    msgValid = false;
	    }

	    return msgValid;
    }

    bool isValidServerMsg(uint32_t msgSize) {
	    bool msgValid = isValidMsg(msgSize);
	    if ((true== msgValid) &&
			    (strncmp(mSocketName, SERVICE_NAME, sizeof(SERVICE_NAME)) != 0)) {
		    SENSOR_LOGE(LOG_TAG "msg not from expected server %s\n", SERVICE_NAME);
		    msgValid = false;
	    }

	    return msgValid;
    }

};

/******************************************************************************
  IPC message structure
 ******************************************************************************/
// defintion for message with msg id of E_SENSORAPI_CLIENT_REGISTER_MSG_ID
struct SensorAPIClientRegisterReqMsg: SensorAPIMsgHeader
{
    ClientType mClientType;

    inline SensorAPIClientRegisterReqMsg(const char* name, ClientType clientType) :
        SensorAPIMsgHeader(name, E_SENSORAPI_CLIENT_REGISTER_MSG_ID),
        mClientType(clientType) { }
};

// defintion for message with msg id of E_SENSORAPI_CLIENT_DEREGISTER_MSG_ID
struct SensorAPIClientDeregisterReqMsg: SensorAPIMsgHeader
{
    inline SensorAPIClientDeregisterReqMsg(const char* name) :
        SensorAPIMsgHeader(name, E_SENSORAPI_CLIENT_DEREGISTER_MSG_ID) { }
};

// defintion for message with msg id of E_SENSORAPI_CAPABILILTIES_MSG_ID
struct SensorAPICapabilitiesIndMsg: SensorAPIMsgHeader
{
    SensorCapabilitiesMask mask;

    inline SensorAPICapabilitiesIndMsg(const char* name, SensorCapabilitiesMask mask) :
        SensorAPIMsgHeader(name, E_SENSORAPI_CAPABILILTIES_MSG_ID),
        mask(mask) { }
};

// defintion for message with msg id of E_SENSORAPI_HAL_READY_MSG_ID
struct SensorAPIHalReadyIndMsg: SensorAPIMsgHeader
{
    inline SensorAPIHalReadyIndMsg(const char* name) :
        SensorAPIMsgHeader(name, E_SENSORAPI_HAL_READY_MSG_ID) { }
};

// defintion for respone message for any msg id requested
struct SensorAPIGenericRespMsg: SensorAPIMsgHeader
{
    int ret;

    inline SensorAPIGenericRespMsg(const char* name, ESensorMsgID msgId, int ret) :
        SensorAPIMsgHeader(name, msgId),
        ret(ret) { }
};

// defintion for message with msg id of E_SENSORAPI_GET_SENSOR_LIST_MSG_ID
struct SensorAPIListReqMsg: SensorAPIMsgHeader
{
    inline SensorAPIListReqMsg(const char* name) :
        SensorAPIMsgHeader(name, E_SENSORAPI_GET_SENSOR_LIST_MSG_ID) { }
};

// defintion for message with msg id of E_SENSORAPI_SENSOR_LIST_MSG_ID
struct SensorAPIListIndMsg : SensorAPIMsgHeader
{
    SensorList sensorList;

    inline SensorAPIListIndMsg(const char* name) :
        SensorAPIMsgHeader(name, E_SENSORAPI_SENSOR_LIST_MSG_ID) { }
};

// defintion for message with msg id of E_SENSORAPI_SENSOR_ENABLE_MSG_ID
struct SensorAPIEnableReqMsg: SensorAPIMsgHeader
{
    int sensor_id;
    int enable;

    inline SensorAPIEnableReqMsg(const char* name, int sensor_id, int enable):
        SensorAPIMsgHeader(name, E_SENSORAPI_SENSOR_ENABLE_MSG_ID),
        sensor_id(sensor_id),
        enable(enable) { }
};

// defintion for message with msg id of E_SENSORAPI_START_BATCHING_MSG_ID
struct SensorAPIStartBatchingReqMsg: SensorAPIMsgHeader
{
    int sensor_id;
    float samplingRate;
    int batchCount;

    inline SensorAPIStartBatchingReqMsg(const char* name,
                                     int sensor_id,
                                     float Sampling_rate,
                                     int Batch_Count
                                     ):
	SensorAPIMsgHeader(name, E_SENSORAPI_START_BATCHING_MSG_ID),
        sensor_id(sensor_id),
        samplingRate(Sampling_rate),
        batchCount(Batch_Count) { }
};

// defintion for message with msg id of E_SENSORAPI_START_TRACKING_MSG_ID
struct SensorAPIStartTrackingReqMsg: SensorAPIMsgHeader
{
    inline SensorAPIStartTrackingReqMsg(const char* name):
        SensorAPIMsgHeader(name, E_SENSORAPI_START_TRACKING_MSG_ID) { }
};

// defintion for message with msg id of E_SENSORAPI_DATA_READ_MSG_ID
struct SensorAPIDataIndMsg: SensorAPIMsgHeader
{
    SensorDataPayload sensorData;

    inline SensorAPIDataIndMsg(const char* name) :
        SensorAPIMsgHeader(name, E_SENSORAPI_DATA_READ_MSG_ID) { }
};

// defintion for message with msg id of E_SENSORAPI_SENSOR_MLC_CASE_LIST_MSG_ID
struct SensorMlcCaseListIndMsg : SensorAPIMsgHeader
{
    SensorMlcCaseList sensorMlcCaseList;

    inline SensorMlcCaseListIndMsg(const char* name) :
        SensorAPIMsgHeader(name, E_SENSORAPI_SENSOR_MLC_CASE_LIST_MSG_ID) { }
};

// defintion for message with msg id of E_SENSORAPI_SENSOR_MLC_CASE_ENABLE_MSG_ID
struct SensorAPIMLCCaseEnableMsg: SensorAPIMsgHeader
{
    char mlc_case_name[100];
    bool enable;

    inline SensorAPIMLCCaseEnableMsg(const char* name, char *case_name, bool enable) :
        SensorAPIMsgHeader(name, E_SENSORAPI_SENSOR_MLC_CASE_ENABLE_MSG_ID),
	enable(enable) {
		memset(mlc_case_name, 0, 100);
		strlcpy(mlc_case_name, case_name, 100);
	}
};

// defintion for message with msg id of E_SENSORAPI_SENSOR_MLC_EVENT_IND_MSG_ID
struct SensorAPIMLCEventIndMsg: SensorAPIMsgHeader
{
    SensorMlcEventData mlcEventData;

    inline SensorAPIMLCEventIndMsg(const char* name, char *case_name, struct mlc_event_data *event) :
        SensorAPIMsgHeader(name, E_SENSORAPI_SENSOR_MLC_EVENT_IND_MSG_ID) {
                memset(&mlcEventData.name[0], 0, 100);
                strlcpy(&mlcEventData.name[0], case_name, 100);
                memcpy(&mlcEventData.event[0], event, sizeof(struct mlc_event_data));
        }
};

// defintion for message with msg id of E_SENSORAPI_SENSOR_TEMP_REQ_MSG_ID
struct SensorAPITempReqMsg: SensorAPIMsgHeader
{
    inline SensorAPITempReqMsg(const char* name) :
        SensorAPIMsgHeader(name, E_SENSORAPI_SENSOR_TEMP_REQ_MSG_ID) { }
};

// defintion for message with msg id of E_SENSORAPI_SENSOR_TEMP_IND_MSG_ID
struct SensorAPITempIndMsg: SensorAPIMsgHeader
{
    float temperature;

    inline SensorAPITempIndMsg(const char* name, float Temperature) :
        SensorAPIMsgHeader(name, E_SENSORAPI_SENSOR_TEMP_IND_MSG_ID),
	temperature(Temperature) { }
};

// defintion for message with msg id of E_SENSORAPI_SENSOR_BUFFER_REQ_MSG_ID
struct SensorAPIBufferDataReqMsg: SensorAPIMsgHeader
{
    bool enable;

    inline SensorAPIBufferDataReqMsg(const char* name, bool Enable) :
        SensorAPIMsgHeader(name, E_SENSORAPI_SENSOR_BUFFER_REQ_MSG_ID),
        enable(Enable) { }
};

// defintion for message with msg id of E_SENSORAPI_SENSOR_BUFFER_IND_MSG_ID
struct SensorAPIBufferDataIndMsg: SensorAPIMsgHeader
{
    SensorDataPayload sensorData;

    inline SensorAPIBufferDataIndMsg(const char* name) :
        SensorAPIMsgHeader(name, E_SENSORAPI_SENSOR_BUFFER_IND_MSG_ID) { }
};

// defintion for message with msg id of E_SENSORAPI_SENSOR_MFIFO_IND_MSG_ID
struct SensorAPImFifoIndMsg: SensorAPIMsgHeader
{
    SensorDataPayload sensorData;

    inline SensorAPImFifoIndMsg(const char* name) :
        SensorAPIMsgHeader(name, E_SENSORAPI_SENSOR_MFIFO_IND_MSG_ID) { }
};

// defintion for message with msg id of E_SENSORAPI_SENSOR_SELFTEST_REQ_MSG_ID
struct SensorAPISelfTestReqMsg: SensorAPIMsgHeader
{
    int sensor_id;
    int request_id;
    SelfTestType selfTestType;

    inline SensorAPISelfTestReqMsg(const char* name,
                                     int sensor_id,
                                     SelfTestType selfTestType, int request_id
                                     ):
        SensorAPIMsgHeader(name, E_SENSORAPI_SENSOR_SELFTEST_REQ_MSG_ID),
        sensor_id(sensor_id),
        selfTestType(selfTestType),
	request_id (request_id){ }
};

// defintion for message with msg id of E_SENSORAPI_SENSOR_SELFTEST_IND_MSG_ID
struct SensorAPISelfTestIndMsg: SensorAPIMsgHeader
{
    int sensor_id;
    int request_id;
    SelfTestResult result;

    inline SensorAPISelfTestIndMsg(const char* name,
                                     int sensor_id, int request_id,
                                     SelfTestResult result
                                     ):
        SensorAPIMsgHeader(name, E_SENSORAPI_SENSOR_SELFTEST_IND_MSG_ID),
        sensor_id(sensor_id),
	request_id (request_id),
        result(result){ }
};

#endif /* SENSORAPIMSG_H */
