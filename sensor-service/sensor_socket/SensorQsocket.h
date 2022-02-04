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
    bool startListeningBlocking(const std::string& name, int ServiceIdToWatch);

    // Create a new SensorThread and listen for new messages in it.
    // The socket to listen on is identified by the argument name,
    // which contains the service and instance id of the socket.
    // Calling this function will return immediately and won't bsensork current thread.
    // The listening can be stopped by calling stopListening().
    bool startListeningNonBlocking(const std::string& name, int ServiceIdToWatch);
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
    inline virtual void onServiceStatusChange(int serviceId, int instanceId, int status, const SensorQsocketSender& refSender) {}

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
    int mServiceIdToWatch;
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
#define SOCKET_SENDER_SEND_TIMEOUT_MSEC 100

static inline __le32 cpu_to_le32(uint32_t x) { return htole32(x); }
static inline uint32_t le32_to_cpu(__le32 x) { return le32toh(x); }

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
    bool startListeningBlocking(const std::string& name, int ServiceIdToWatch);

    // Create a new SensorThread and listen for new messages in it.
    // The socket to listen on is identified by the argument name,
    // which contains the service and instance id of the socket.
    // Calling this function will return immediately and won't bsensork current thread.
    // The listening can be stopped by calling stopListening().
    bool startListeningNonBlocking(const std::string& name, int ServiceIdToWatch);
    static void *startListeningNonBlockingThread(void *arg);

    // Stop listening to new messages.
    void stopListening();

    // Send out a message.
    // Call this function to send a message in argument data to socket in argument name.
    //
    // Argument name contains the name of the target unix socket. data contains the
    // message to be sent out. Convert your message to a string before calling this function.
    // The function will return true on success, and false on failure.
    bool send(int service, int instance, const std::string& data);
    bool send(int service, int instance, const uint8_t data[], uint32_t length);
    bool handleQrtrCtrlMsg(const char* data, uint32_t len);

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
    inline virtual void onServiceStatusChange(int serviceId, int instanceId, int status, const SensorQsocketSender& refSender) {}

private:
    static bool sendData(int fd, const sockaddr_qrtr& addr,
		    const uint8_t data[], uint32_t length); 
    struct sockaddr_qrtr mAddr;
    mutable sockaddr_qrtr mCtrlPntAddr;
    mutable struct qrtr_ctrl_pkt mCtrlPkt;
    int mService;
    int mInstance;
    int mFdMe;
    int mServiceIdToWatch;
    std::string mQsocketName;
    pthread_t mQsocketThread;
};


class SensorQsocketSender {
protected:
    int mSocket;
    int mService;
    int mInstance;
    mutable sockaddr_qrtr mAddr;
    mutable sockaddr_qrtr mCtrlPntAddr;
    mutable struct qrtr_ctrl_pkt mCtrlPkt;
    mutable bool mLookupPending;
    bool ctrlCmdAndResponse(enum qrtr_pkt_type cmd) const {
	    SENSOR_LOGI(LOG_TAG "cmd: %d, isValid %d sock id %d, service id %d, instance id %d",
		cmd, isValid(), mSocket,
		le32_to_cpu(mCtrlPkt.server.service),
		le32_to_cpu(mCtrlPkt.server.instance));
        if (isValid()) {
            int rc = 0;
            mCtrlPkt.cmd = cpu_to_le32(cmd);
            if ((rc = ::sendto(mSocket, &mCtrlPkt, sizeof(mCtrlPkt), 0,
                               (const struct sockaddr *)&mCtrlPntAddr, sizeof(mCtrlPntAddr))) < 0) {
		    SENSOR_LOGE(LOG_TAG "failed: sendto rc=%d reason=(%s)", rc, strerror(errno));
            } else if (QRTR_TYPE_NEW_LOOKUP == cmd) {
                int len = 0;
                struct qrtr_ctrl_pkt pkt;
                while ((len = ::recv(mSocket, &pkt, sizeof(pkt), 0)) > 0) {
                    if (len >= (decltype(len))sizeof(pkt)) {
                        qrtr_pkt_type pktType = le32_to_cpu(pkt.cmd);
                        SENSOR_LOGI(LOG_TAG "qrtr new lookup received pkt type: %d, "
                                 "pkt service id: %d, instance id: %d,"
                                 "node %d, port %d",
                                 pktType, le32_to_cpu(pkt.server.service),
                                 le32_to_cpu(pkt.server.instance),
                                 le32_to_cpu(pkt.server.node),
                                 le32_to_cpu(pkt.server.port));

                        switch (pktType){
                        case QRTR_TYPE_NEW_SERVER:
                            if (le32_to_cpu(pkt.server.service) == 0 &&
                                       le32_to_cpu(pkt.server.instance) == 0) {
                                return false;
                            } else if ((mCtrlPkt.server.service == pkt.server.service) &&
                                (mCtrlPkt.server.instance == pkt.server.instance)) {
                                mAddr.sq_node = le32_to_cpu(pkt.server.node);
                                mAddr.sq_port = le32_to_cpu(pkt.server.port);
                                return true;
                            }
                            break;
                        case QRTR_TYPE_DEL_SERVER:
                            if ((mCtrlPkt.server.service == pkt.server.service) &&
                                (mCtrlPkt.server.instance == pkt.server.instance)) {
                                // service of particular service id, instance id gets deleted
                                return false;
                            }
                            break;
                        default:
                            break;
                        }
                    }
                }
            }
        }
        SENSOR_LOGI(LOG_TAG "cmd: %d, return %d", cmd, isValid());
        return isValid();
    }
    inline virtual bool isOperable() {
        return isValid() &&
            (mAddr.sq_node != 0 || mAddr.sq_port != 0);
    }

public:
    inline bool send(const uint8_t data[], uint32_t length) {
        if (mLookupPending) {
            if (ctrlCmdAndResponse(QRTR_TYPE_NEW_LOOKUP) == false) {
                return 0;
            }
            mLookupPending = false;
        }
	bool result = SensorQsocket::sendData(mSocket, mAddr, data, length);
        return result;
    }
 
    inline bool isValid() const { return -1 != mSocket; }

    inline SensorQsocketSender(const sockaddr_qrtr& destAddr) :
            mService(0), mInstance(0),
            mAddr(destAddr), mCtrlPkt({}), mLookupPending(false) {
		    mSocket =  socket(AF_QIPCRTR, SOCK_DGRAM, 0);
		    if (mSocket < 0) {
			    return;
		    }
    }
    inline SensorQsocketSender(int service, int instance) : 
	    mService(service),
            mInstance(instance),
            mAddr({AF_QIPCRTR, 0, 0}),
            mCtrlPkt({}),
            mLookupPending(true) {
        mSocket =  socket(AF_QIPCRTR, SOCK_DGRAM, 0);
	if (mSocket < 0) {
		return;
	}
	// set timeout so if failed to send, call will return after SOCKET_TIMEOUT_MSEC
        // otherwise, call may never return
        timeval timeout;

        timeout.tv_sec = SOCKET_SENDER_SEND_TIMEOUT_MSEC / 1000;
        timeout.tv_usec = SOCKET_SENDER_SEND_TIMEOUT_MSEC % 1000 * 1000;
        setsockopt(mSocket, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

        socklen_t sl = sizeof(mAddr);
        int rc = 0;
        if ((rc = getsockname(mSocket, (struct sockaddr*)&mAddr, &sl)) ||
            mAddr.sq_family != AF_QIPCRTR || sl != sizeof(mAddr)) {
            SENSOR_LOGE(LOG_TAG "failed: getsockname rc=%d reason=(%s), mAddr.sq_family=%d",
                     rc, strerror(errno), mAddr.sq_family);
            close(mSocket);
        } else {
            mCtrlPkt.server.service = cpu_to_le32(service);
            mCtrlPkt.server.instance = cpu_to_le32(instance);
            mCtrlPkt.server.node = cpu_to_le32(mAddr.sq_node);
            mCtrlPkt.server.port = cpu_to_le32(mAddr.sq_port);
            mCtrlPntAddr = mAddr;
            mCtrlPntAddr.sq_port = QRTR_PORT_CTRL;
        }
    }

    inline virtual bool copyDestAddrFrom(const SensorQsocketSender& otherSender) {
        bool retval = true;
        sockaddr_qrtr otherAddr = (reinterpret_cast<const SensorQsocketSender&>(otherSender)).mAddr;
        if (mAddr.sq_family != otherAddr.sq_family ||
                mAddr.sq_node != otherAddr.sq_node ||
                mAddr.sq_port != otherAddr.sq_port) {
            mAddr = otherAddr;
            retval = true;
        }
        mLookupPending = false;

        return retval;
    }

    inline ~SensorQsocketSender() {
        if (mSocket >= 0) {
	    close(mSocket);
	}
    }

};
#endif //end of QRTR_
} // namespace sensor_util 
#endif //__SENSOR_QSOCKET__
