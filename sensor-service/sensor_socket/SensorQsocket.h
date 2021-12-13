/* Copyright (c) 2021 The Linux Foundation. All rights reserved.
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
 * Changes from Qualcomm Innovation Center are provided under the following license:
 *
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the
 * disclaimer below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 * GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 * HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __SENSOR_QSOCKET__
#define __SENSOR_QSOCKET__

#include <string>
#include <memory>
#include <unistd.h>
#include <SensorIpc.h>
#include <SensorThread.h>
#include <SensorLog.h>

#ifdef USE_QSOCKET
#include <sys/ioctl.h>
#include <poll.h>
#include <qsocket.h>
#include <qsocket_ipcr.h>
#else
#include <endian.h>
#include <linux/qrtr.h>
#endif

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "SensorSvc_SensorQSocket:"

namespace sensor_util {


#ifdef USE_QSOCKET

class SensorQsocketSender;

class SensorQsocket : SensorIpc {
friend SensorQsocketSender;
public:
    inline SensorQsocket() :
            mService(0), mInstance(0),
            mFdMe(-1) {}
    inline virtual ~SensorQsocket() { stopListening(); }

    // Listen for new messages in current thread. The socket to listen on
    // is identified by the argument name, which contains the service and
    // instance id of the socket.
    // Calling this funciton will bsensork current thread. The listening can
    // be stopped by calling stopListening().
    bool startListeningBlocking(const std::string& name);

    // Create a new SensorThread and listen for new messages in it.
    // The socket to listen on is identified by the argument name,
    // which contains the service and instance id of the socket.
    // Calling this function will return immediately and won't bsensork current thread.
    // The listening can be stopped by calling stopListening().
    bool startListeningNonBlocking(const std::string& name);
    static void *startListeningNonBlockingThread(void *arg);

    // Stop listening to new messages.
    void stopListening();

    // Send out a message.
    // Call this function to send a message in argument data to socket in argument name.
    //
    // Argument name contains the name of the target unix socket. data contains the
    // message to be sent out. Convert your message to a string before calling this function.
    // The function will return true on success, and false on failure.
    static bool send(int service, int instance, const std::string& data);
    static bool send(int service, int instance, const uint8_t data[], uint32_t length);

protected:
    // Callback function for receiving incoming messages.
    // Override this function in your derived class to process incoming messages.
    // For each received message, this callback function will be called once.
    // This callback function will be called in the calling thread of startListeningBlocking
    // or in the new SensorThread created by startListeningNonBlocking.
    //
    // Argument data contains the received message. You need to parse it.
    inline virtual void onReceive(const std::string& /*data*/) {}

    // SensorQsocket client can overwrite this function to get notification
    // when the socket for SensorQsocket is ready to receive messages.
    inline virtual void onListenerReady() {}

private:
    static bool sendData(int fd, const qsockaddr_ipcr& addr,
            const uint8_t data[], uint32_t length);
    static bool findService(int fd, qsockaddr_ipcr& addr,
                            int service, int instance);
    // this call will find service with retry attempt of
    // default retry count and interval
    static bool findServiceWithRetry(int fd, qsockaddr_ipcr& addr,
		    int service, int instance);
    struct qsockaddr_ipcr mDestAddr;
    int mService;
    int mInstance;
    int mFdMe;
    std::string mQsocketName;
    pthread_t mQsocketThread;
};

static void printaddr(const struct qsockaddr_ipcr* p)
{
    SENSOR_LOGI(LOG_TAG "family=%u\n", p->family);
    SENSOR_LOGI(LOG_TAG "addr type=%u\n", p->address.addrtype);
    SENSOR_LOGI(LOG_TAG "port addr node=0x%x\n", p->address.addr.port_addr.node_id);
    SENSOR_LOGI(LOG_TAG "port addr port=0x%x\n", p->address.addr.port_addr.port_id);
    SENSOR_LOGI(LOG_TAG "port name serv=%d\n", p->address.addr.port_name.service);
    SENSOR_LOGI(LOG_TAG "port name inst=%d\n", p->address.addr.port_name.instance);
}

class SensorQsocketSender {
public:
    // Constructor of SensorQsocketSender class
    //
    // Argument service/instance contain destination port.
    // This class hides generated fd and destination address object from user.
    inline SensorQsocketSender(int service, int instance) {

        mService = service;
        mInstance = instance;
	memset(&mDestAddr, 0, sizeof(mDestAddr));
        mSocket = qsocket(AF_IPC_ROUTER, QSOCK_DGRAM, 0);
        if (mSocket < 0) {
            return;
        }
        printaddr(&mDestAddr);
    }

    inline ~SensorQsocketSender() {
        if (mSocket >= 0) {
            qclose(mSocket);
	    mSocket = -1;
	}
    }

    // Send out a message.
    // Call this function to send a message
    //
    // Argument data and length contains the message to be sent out.
    // Return true when succeeded
    inline bool send(const uint8_t data[], uint32_t length) {
        bool rtv = true;
        if (nullptr != data) {
                rtv = SensorQsocket::sendData(mSocket, mDestAddr, data, length);
        }
        return rtv;
    }

private:
    int mSocket;
    int mService;
    int mInstance;
    qsockaddr_ipcr mDestAddr;
};

#else //QRTRT famlily

#define RETRY_FINDNEWSERVICE_MAX_COUNT 200
#define RETRY_FINDNEWSERVICE_SLEEP_MS  5
#define SOCKET_TIMEOUT_SEC 2

class SensorQsocketSender;

class SensorQsocket : SensorIpc {
friend SensorQsocketSender;
public:
    inline SensorQsocket() :
            mService(0), mInstance(0),
            mFdMe(-1) {}
    inline virtual ~SensorQsocket() { stopListening(); }

    // Listen for new messages in current thread. The socket to listen on
    // is identified by the argument name, which contains the service and
    // instance id of the socket.
    // Calling this funciton will bsensork current thread. The listening can
    // be stopped by calling stopListening().
    bool startListeningBlocking(const std::string& name);

    // Create a new SensorThread and listen for new messages in it.
    // The socket to listen on is identified by the argument name,
    // which contains the service and instance id of the socket.
    // Calling this function will return immediately and won't bsensork current thread.
    // The listening can be stopped by calling stopListening().
    bool startListeningNonBlocking(const std::string& name);
    static void *startListeningNonBlockingThread(void *arg);

    // Stop listening to new messages.
    void stopListening();

    // Send out a message.
    // Call this function to send a message in argument data to socket in argument name.
    //
    // Argument name contains the name of the target unix socket. data contains the
    // message to be sent out. Convert your message to a string before calling this function.
    // The function will return true on success, and false on failure.
    static bool send(int service, int instance, const std::string& data);
    static bool send(int service, int instance, const uint8_t data[], uint32_t length);

protected:
    // Callback function for receiving incoming messages.
    // Override this function in your derived class to process incoming messages.
    // For each received message, this callback function will be called once.
    // This callback function will be called in the calling thread of startListeningBlocking
    // or in the new SensorThread created by startListeningNonBlocking.
    //
    // Argument data contains the received message. You need to parse it.
    inline virtual void onReceive(const std::string& /*data*/) {}

    // SensorQsocket client can overwrite this function to get notification
    // when the socket for SensorQsocket is ready to receive messages.
    inline virtual void onListenerReady() {}

private:
    static bool sendData(int fd, const sockaddr_qrtr& addr,
		    const uint8_t data[], uint32_t length); 
    static bool findService(int fd, sockaddr_qrtr& addr, int service, int instance,
                            bool & serviceDeleted);
    // this call will find service with retry attempt of
    // default retry count and interval
    static bool findServiceWithRetry(int fd, sockaddr_qrtr& addr, int service, int instance,
                                     bool & serviceDeleted);
    struct sockaddr_qrtr mDestAddr;
    int mService;
    int mInstance;
    int mFdMe;
    std::string mQsocketName;
    pthread_t mQsocketThread;
};


class SensorQsocketSender {
public:
    // Constructor of SensorQsocketSender class
    //
    // Argument service/instance contain destination port.
    // This class hides generated fd and destination address object from user.
    inline SensorQsocketSender(int service, int instance) {
        mService = service;
        mInstance = instance;
	mServiceDeleted = false;
	memset(&mDestAddr, 0, sizeof(mDestAddr));
	mSocket = socket(AF_QIPCRTR, SOCK_DGRAM, 0);
        if (mSocket < 0) {
            return;
        }
        // 2 second timeout value for socket operation
        timeval timeout;
        timeout.tv_sec = SOCKET_TIMEOUT_SEC;
        timeout.tv_usec = 0;
        setsockopt(mSocket, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

        // find the server address
	SensorQsocket::findServiceWithRetry(mSocket, mDestAddr, mService, mInstance,
			mServiceDeleted);
    }

    inline ~SensorQsocketSender() {
        if (mSocket >= 0) {
	    close(mSocket);
	}
    }

    // Send out a message.
    // Call this function to send a message
    //
    // Argument data and length contains the message to be sent out.
    // Return true when succeeded
    inline bool send(const uint8_t data[], uint32_t length) {
        bool rtv = true;
	if ((nullptr != data) && (false == mServiceDeleted)){
            if ((mDestAddr.sq_node == 0) && (mDestAddr.sq_port == 0)) {
                SENSOR_LOGE(LOG_TAG "service not ready");
            } else{
                rtv = SensorQsocket::sendData(mSocket, mDestAddr, data, length);
            }
        }
        return rtv;
    }

    // This routine will attempt to find new service when service
    // has crashed and restarted.
    // We need to make sure that we get the new dest info (node/port) as the
    // crashed service may lingering around for some time and findService may
    // return the node/port belong to the crashed service.
    inline bool findNewService() {
        bool rtv = false;
	bool newServiceFound = false;
	int retryCount = 0;
	sockaddr_qrtr newDestAddr;
	do {
		memset(&newDestAddr, 0, sizeof(newDestAddr));
		mServiceDeleted = false;
		rtv = SensorQsocket::findServiceWithRetry(mSocket, newDestAddr, mService, mInstance, mServiceDeleted);
		if (true == rtv) {
			if ((mDestAddr.sq_node != newDestAddr.sq_node) ||
					(mDestAddr.sq_port != newDestAddr.sq_port)) {
				mDestAddr = newDestAddr;
				newServiceFound = true;
				break;
			}
			usleep(RETRY_FINDNEWSERVICE_SLEEP_MS*1000);
			retryCount++;
		}
	} while (retryCount < RETRY_FINDNEWSERVICE_MAX_COUNT);
	SENSOR_LOGD(LOG_TAG "find new service: service found %d, new service found %d,"
			"retry count %d", rtv, newServiceFound, retryCount);
	return rtv;
    }
private:
    int mSocket;
    int mService;
    int mInstance;
    bool mServiceDeleted;
    sockaddr_qrtr mDestAddr;
};
#endif //end of QRTR_
} // namespace sensor_util 
#endif //__SENSOR_QSOCKET__
