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
#ifndef SENSORHALDAEMON_IPCSENDER_H
#define SENSORHALDAEMON_IPCSENDER_H

#include <SensorIpc.h>
#include <SensorQsocket.h>
#include <SensorApiMsg.h>

using sensor_socket::SensorIpcSender;
using sensor_socket::SensorQsocketSender;

class SensorHalDaemonIPCSender
{
public:
    //Constructor Function
    inline SensorHalDaemonIPCSender(const char* destSocket, SocketType socketType) :
            mIpcSender(nullptr),
	    mQsockSender(nullptr) {
	if (socketType != Q_SOCKET && strncmp(destSocket, SOCKET_SENSOR_CLIENT_DIR,
		sizeof(SOCKET_SENSOR_CLIENT_DIR)-1) == 0 ) {
	    mIpcSender = new SensorIpcSender(destSocket);
	}
	else if(socketType != IPC_SOCKET) {
            uint32_t serviceId = atoi(destSocket);
            const char* instance_ptr = strchr(destSocket, '.');
            if (nullptr != instance_ptr) {
		    uint32_t instanceId = atoi(++instance_ptr);
		    mQsockSender = new SensorQsocketSender(serviceId, instanceId);
	    }

        }
    }

    //Cleanup Function to delete mQsockSender or mIpcSender
    void cleanup(string mName){
        if (strncmp(mName.c_str(), SOCKET_SENSOR_CLIENT_DIR,
                sizeof(SOCKET_SENSOR_CLIENT_DIR)-1) != 0 ) {
                SENSOR_LOGI(LOG_TAG "EAP cleanup \n");
                if(mQsockSender){
                        delete mQsockSender;
                        mQsockSender = nullptr;
                }

        } else {
                SENSOR_LOGI(LOG_TAG "MDM cleanup \n");
                if(mIpcSender){
                        delete mIpcSender;
                        mIpcSender = nullptr;
                }
          }
    }

    //Destructor Function
    inline ~SensorHalDaemonIPCSender() {
	SENSOR_LOGI("Cleanup Destructor\n");
    }

    bool send(const uint8_t data[], uint32_t length);

private:
    SensorIpcSender* mIpcSender;
    SensorQsocketSender* mQsockSender;
};

#endif //SENSORHALDAEMON_IPCSENDER_H
