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
 * Changes from Qualcomm Technologies, Inc. are provided under the following license:
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <sys/types.h>
#include <sys/stat.h>
#include "SensorQsocket.h"

namespace sensor_socket {

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "SensorSvc_SensorQSocket:"

#define SENSOR_MSG_BUF_LEN 8192
#define SENSOR_MSG_HEAD "_MSGLEN_"
#define SENSOR_MSG_ABORT "SensorQsocketMsg::ABORT"

#ifdef USE_QSOCKET

#define RETRY_FINDSERVICE_MAX_COUNT 10
#define RETRY_FINDSERVICE_SLEEP_MS  5

bool SensorQsocket::startListeningNonBlocking(const std::string& name, int ServiceIdToWatch) {
    mQsocketName = name;
    mServiceIdToWatch = ServiceIdToWatch;
    return Sensor_ThreadCreate(&mQsocketThread, startListeningNonBlockingThread, this, "SensorQsocket-");
}

void* SensorQsocket::startListeningNonBlockingThread(void *arg) {
   SensorQsocket* mSensorQsocket = (SensorQsocket*)arg;
   mSensorQsocket->startListeningBlocking(mSensorQsocket->mQsocketName, mSensorQsocket->mServiceIdToWatch);
   return NULL;
}

bool SensorQsocket::startListeningBlocking(const std::string& name, int ServiceIdToWatch) {
    bool stopRequested = false;

    // socket
    int fd  = qsocket(AF_IPC_ROUTER, QSOCK_DGRAM, 0);
    if (fd < 0) {
        SENSOR_LOGE(LOG_TAG "create socket error. reason:%s\n", strerror(errno));
        return false;
    }

    // name is of format "serviceid.instanceid", and thus
    // should not exceed 65 bytes
    mService = atoi(name.c_str());
    const char* instance_ptr = strchr(name.c_str(), '.');
    if (nullptr != instance_ptr) {
	    return false;
    }

    mInstance = atoi(++instance_ptr);

    // bind
    memset(&mDestAddr, 0, sizeof(mDestAddr));
    mDestAddr.family = AF_IPC_ROUTER;
    mDestAddr.address.addrtype = IPCR_ADDR_NAME;
    mDestAddr.address.addr.port_name.service = mService;
    mDestAddr.address.addr.port_name.instance = mInstance;
    if (qbind(fd, (struct qsockaddr*)&mDestAddr, sizeof(mDestAddr)) < 0) {
        SENSOR_LOGE(LOG_TAG "bind socket error. reason:%s\n", strerror(errno));
        qclose(fd);
        return false;
    }

    mFdMe = fd;
    SENSOR_LOGI(LOG_TAG "startListeningBlocking mFd=%d\n", mFdMe);
    printaddr(&mDestAddr);

    // inform that the socket is ready to receive message
    onListenerReady();

    ssize_t nBytes = 0;
    std::string msg = "";
    std::string abort = SENSOR_MSG_ABORT;
    while (1) {
        struct qsockaddr_ipcr tmpAddr = {0};
        qsocklen_t tmpAddrlen = sizeof(tmpAddr);

        msg.resize(SENSOR_MSG_BUF_LEN);

        nBytes = qrecvfrom(fd, (void*)(msg.data()), msg.size(), 0, (struct qsockaddr *)&tmpAddr, &tmpAddrlen);
        SENSOR_LOGE(LOG_TAG "recvfrom done=%d\n", nBytes);
        if (nBytes < 0) {
            break;
        } else if (nBytes == 0) {
            continue;
        }

        if (strncmp(msg.data(), abort.c_str(), abort.length()) == 0) {
            stopRequested = true;
            SENSOR_LOGE(LOG_TAG "recvd abort msg.data %s\n", msg.data());
            break;
        }

        if (strncmp(msg.data(), SENSOR_MSG_HEAD, sizeof(SENSOR_MSG_HEAD) - 1)) {
            // short message
            msg.resize(nBytes);
            onReceive(msg);
        } else {
            // long message
            size_t msgLen = 0;
            sscanf(msg.data(), SENSOR_MSG_HEAD"%zu", &msgLen);
            msg.resize(msgLen);
            size_t msgLenReceived = 0;
            while ((msgLenReceived < msgLen) && (nBytes > 0)) {
                nBytes = qrecvfrom(fd, (void*)&(msg[msgLenReceived]),
                        msg.size() - msgLenReceived, 0, NULL, NULL);
                msgLenReceived += nBytes;
            }
            if (nBytes > 0) {
                onReceive(msg);
            } else {
                break;
            }
        }
    }

    if (::close(fd)) {
        SENSOR_LOGE(LOG_TAG "cannot close socket:%s\n", strerror(errno));
    }

    pthread_exit((void *)0);
    return stopRequested;
}

bool SensorQsocket::findService(int fd, qsockaddr_ipcr& addr,
                             int service, int instance) {
    bool serviceFound = false;
    if (0 <= fd) {
        memset(&addr, 0, sizeof(addr));
        ipcr_name_t ipcr_name = { service, instance };

        unsigned int num_entries = 1;
        int rc = ipcr_find_name(fd, &ipcr_name, &addr, NULL, &num_entries, 0);
        if ((rc > 0) && (1 == num_entries)) {
            serviceFound = true;
            SENSOR_LOGI(LOG_TAG "for service %d, instance %d, number of services successfully found %d\n",
                      service, instance, num_entries);
            printaddr(&addr);
	}else {
		SENSOR_LOGE(LOG_TAG "service found error code %d, service num %d\n", rc, num_entries);
	}
    }
    return serviceFound;
}

bool SensorQsocket::findServiceWithRetry(int fd, qsockaddr_ipcr& addr,
		int service, int instance) {
    int retryCount = 0;
    bool result = false;
    do {
	    result = SensorQsocket::findService(fd, addr, service, instance);
	    if (true == result) {
		    break;
	    }
	    usleep(RETRY_FINDSERVICE_SLEEP_MS * 1000);
    } while (retryCount++ <= RETRY_FINDSERVICE_MAX_COUNT);
    SENSOR_LOGD(LOG_TAG "find service returned %d, retryCount %d", result, retryCount);
    return result;
}

void SensorQsocket::stopListening() {
    if (mFdMe >= 0) {
        std::string abort = SENSOR_MSG_ABORT;
        send(mService, mInstance, abort);
        mFdMe = -1;
    }
}

// static
bool SensorQsocket::send(int service, int instance, const std::string& data) {
    return send(service, instance, (const uint8_t*)data.c_str(), data.length());
}

// static
bool SensorQsocket::send(int service, int instance, const uint8_t data[], uint32_t length) {

    bool result = true;
    int fd = qsocket(AF_IPC_ROUTER, QSOCK_DGRAM, 0);
    if (fd < 0) {
        SENSOR_LOGE(LOG_TAG "create socket error. reason:%s\n", strerror(errno));
        return false;
    }

    qsockaddr_ipcr addr;
    result = findServiceWithRetry(fd, addr, service, instance);
    if (result == true) {
	    result = sendData(fd, addr, data, length);
    }
    (void)qclose(fd);
    return result;
}

// static
bool SensorQsocket::sendData(int fd, const qsockaddr_ipcr& addr, const uint8_t data[], uint32_t length) {

    bool result = true;

    if (length <= SENSOR_MSG_BUF_LEN) {
        if (qsendto(fd, data, length, 0,
                (struct qsockaddr*)&addr, sizeof(addr)) < 0) {
            SENSOR_LOGE(LOG_TAG "cannot send to socket. reason:%s\n", strerror(errno));
            result = false;
        }
    } else {
        std::string head = SENSOR_MSG_HEAD;
        head.append(std::to_string(length));
        if (qsendto(fd, head.c_str(), head.length(), 0,
                (struct qsockaddr*)&addr, sizeof(addr)) < 0) {
            SENSOR_LOGE(LOG_TAG "cannot send to socket. reason:%s\n", strerror(errno));
            result = false;
        } else {
            size_t sentBytes = 0;
            while(sentBytes < length) {
                size_t partLen = length - sentBytes;
                if (partLen > SENSOR_MSG_BUF_LEN) {
                    partLen = SENSOR_MSG_BUF_LEN;
                }
                ssize_t rv = qsendto(fd, (data + sentBytes), partLen, 0,
                        (struct qsockaddr*)&addr, sizeof(addr));
                if (rv < 0) {
                    SENSOR_LOGE(LOG_TAG "cannot send to socket. reason:%s\n", strerror(errno));
                    result = false;
                    break;
                }
                sentBytes += rv;
            }
        }
    }
    SENSOR_LOGE(LOG_TAG "send len=%u result=%d\n", length, result);
    return result;
}
#else  //QRTR family
bool SensorQsocket::startListeningNonBlocking(const std::string& name, int ServiceIdToWatch) {
    mQsocketName = name;
    mServiceIdToWatch = ServiceIdToWatch;
    return Sensor_ThreadCreate(&mQsocketThread, startListeningNonBlockingThread, this, "SensorQsocket-");
}

void* SensorQsocket::startListeningNonBlockingThread(void *arg) {
   SensorQsocket* mSensorQsocket = (SensorQsocket*)arg;
   (void)mSensorQsocket->startListeningBlocking(mSensorQsocket->mQsocketName, mSensorQsocket->mServiceIdToWatch);
   return NULL;
}

bool SensorQsocket::handleQrtrCtrlMsg(const char* data, uint32_t len) { 
   const struct qrtr_ctrl_pkt* pkt = reinterpret_cast<const struct qrtr_ctrl_pkt*>(data);
   bool handledAsQrtrCtrlMsg = false;
   if (sizeof(*pkt) == len) {
      const uint32_t cmd = le32_to_cpu(pkt->cmd);
      if (cmd >= QRTR_TYPE_DATA && cmd <= QRTR_TYPE_DEL_LOOKUP) {
	      handledAsQrtrCtrlMsg = true;
	      int serviceId = le32_to_cpu(pkt->server.service);
	      int instanceId = le32_to_cpu(pkt->server.instance);
	      int serverNodeId = le32_to_cpu(pkt->server.node);
	      int serverPort = le32_to_cpu(pkt->server.port);
	      int clientNodeId = le32_to_cpu(pkt->client.node);
	      int clientPort = le32_to_cpu(pkt->client.port);
	      SENSOR_LOGV(LOG_TAG "qrtr control msg: cmd %d, server.service:instance-node:port %d:%d-%d:%d,"
			      "client node:port %d:%d\n", cmd, serviceId, instanceId, serverNodeId,
			      serverPort, clientNodeId, clientPort);
	      if ((QRTR_TYPE_NEW_SERVER == cmd || QRTR_TYPE_DEL_SERVER == cmd)) {
		      int status = (QRTR_TYPE_NEW_SERVER == cmd) ? 1 : 0;
		      SENSOR_LOGD(LOG_TAG "qrtr server up/down status %d\n",status);
		      sockaddr_qrtr addr = {AF_QIPCRTR, serverNodeId, serverPort};
		      const SensorQsocketSender sender(addr);
		      onServiceStatusChange(serviceId, instanceId, status, sender);
	      } else if ((QRTR_TYPE_DEL_CLIENT == cmd || QRTR_TYPE_BYE == cmd)) {
		      SENSOR_LOGV(LOG_TAG "qrtr client deleted\n");
	      }
      }
   }
   return handledAsQrtrCtrlMsg;
}

bool SensorQsocket::startListeningBlocking(const std::string& name, int ServiceIdToWatch) {
    bool stopRequested = false;

    // name is of format "serviceid.instanceid", and thus
    // should not exceed 65 bytes
    mService = atoi(name.c_str());
    const char* instance_ptr = strchr(name.c_str(), '.');
    if (nullptr == instance_ptr) {
	    return false;
    }
    else {
	    instance_ptr++;
    }

    mInstance = atoi(instance_ptr);

    // socket
    int fd = socket(AF_QIPCRTR, SOCK_DGRAM, 0);
    if (fd < 0) {
        SENSOR_LOGE(LOG_TAG "create socket error. reason:%s\n", strerror(errno));
        return false;
    }

    // set timeout so if failed to send, call will return after SOCKET_TIMEOUT_MSEC
    // otherwise, call may never return
    timeval timeout;
    timeout.tv_sec = SOCKET_SENDER_SEND_TIMEOUT_MSEC / 1000;
    timeout.tv_usec = SOCKET_SENDER_SEND_TIMEOUT_MSEC % 1000 * 1000;
    (void)setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    socklen_t sl = sizeof(mAddr);
    int rc = 0;
    if ((rc = getsockname(fd, (struct sockaddr*)&mAddr, &sl)) ||
		    mAddr.sq_family != AF_QIPCRTR || sl != sizeof(mAddr)) {
	    SENSOR_LOGE(LOG_TAG "failed: getsockname rc=%d reason=(%s), mAddr.sq_family=%d",
			    rc, strerror(errno), mAddr.sq_family);
	    (void)close(fd);
    } else {
	    mCtrlPkt.server.service = cpu_to_le32(mService);
	    mCtrlPkt.server.instance = cpu_to_le32(mInstance);
	    mCtrlPkt.server.node = cpu_to_le32(mAddr.sq_node);
	    mCtrlPkt.server.port = cpu_to_le32(mAddr.sq_port);
	    mCtrlPntAddr = mAddr;
	    mCtrlPntAddr.sq_port = QRTR_PORT_CTRL;
    }

    SENSOR_LOGI(LOG_TAG "QRTR_TYPE_NEW_SERVER: sock id %d, service id %d, instance id %d",
		    fd,
		    le32_to_cpu(mCtrlPkt.server.service),
		    le32_to_cpu(mCtrlPkt.server.instance));
    if (fd > 0) {
	    int rc = 0;
	    mCtrlPkt.cmd = cpu_to_le32(QRTR_TYPE_NEW_SERVER);
	    if ((rc = ::sendto(fd, &mCtrlPkt, sizeof(mCtrlPkt), 0,
					    (const struct sockaddr *)&mCtrlPntAddr, sizeof(mCtrlPntAddr))) < 0) {
		   SENSOR_LOGE(LOG_TAG "failed: sendto rc=%d reason=(%s)", rc, strerror(errno));
	    } 
    }

    /**Sending Look up command for service id to watch********************/
    struct qrtr_ctrl_pkt pkt = {};
    pkt.cmd = cpu_to_le32(QRTR_TYPE_NEW_LOOKUP);
    pkt.server.service = cpu_to_le32(ServiceIdToWatch);
    SENSOR_LOGI(LOG_TAG "watch for service: %d fd %d\n", pkt.server.service, fd);
    if ((rc = ::sendto(fd, &pkt, sizeof(pkt), 0,
				    (const struct sockaddr *)&mCtrlPntAddr,
				    sizeof(mCtrlPntAddr))) < 0) {
	    SENSOR_LOGE(LOG_TAG "failed: sendto rc=%d reason=(%s)\n", rc, strerror(errno));
    }

    mFdMe = fd;
    SENSOR_LOGI(LOG_TAG "family=%u, node=%d, port=%d\n", mAddr.sq_family, mAddr.sq_node, mAddr.sq_port);

    // inform that the socket is ready to receive message
    onListenerReady();

    ssize_t nBytes = 0;
    std::string msg = "";
    std::string abort = SENSOR_MSG_ABORT;
    while (1) {
        struct sockaddr_qrtr tmpAddr = {0};
        socklen_t tmpAddrlen = sizeof(tmpAddr);

        msg.resize(SENSOR_MSG_BUF_LEN);

        nBytes = recvfrom(fd, (void*)(msg.data()), msg.size(), 0, (void*)&tmpAddr, &tmpAddrlen);
        if (nBytes < 0) {
            break;
        } else if (nBytes == 0) {
            continue;
        }

        if (strncmp(msg.data(), abort.c_str(), abort.length()) == 0) {
            stopRequested = true;
            SENSOR_LOGE(LOG_TAG "recvd abort msg.data %s\n", msg.data());
            break;
        }

        struct sockaddr_qrtr clientAddr = {0};
        clientAddr = tmpAddr;

        if (strncmp(msg.data(), SENSOR_MSG_HEAD, sizeof(SENSOR_MSG_HEAD) - 1)) {
            // short message
            msg.resize(nBytes);
	    if ((!handleQrtrCtrlMsg(msg.data(), msg.length())))
		    onReceive(msg);
        } else {
            // long message
            size_t msgLen = 0;
            (void)sscanf(msg.data(), SENSOR_MSG_HEAD"%zu", &msgLen);
            msg.resize(msgLen);
            size_t msgLenReceived = 0;
            while ((msgLenReceived < msgLen) && (nBytes > 0)) {
                nBytes = recvfrom(fd, (void*)&(msg[msgLenReceived]),
                        msg.size() - msgLenReceived, 0, NULL, NULL);
                msgLenReceived += nBytes;
            }
            if (nBytes > 0) {
		    if ((!handleQrtrCtrlMsg(msg.data(), msg.length())))
			    onReceive(msg);
            } else {
                break;
            }
        }
    }
    if (::close(fd)) {
        SENSOR_LOGE(LOG_TAG "cannot close socket:%s\n", strerror(errno));
    }
    pthread_exit((void *)0);
    return stopRequested;
}

void SensorQsocket::stopListening() {
    if (mFdMe >= 0) {
        std::string abort = SENSOR_MSG_ABORT;
        (void)send(mService, mInstance, abort);
        mFdMe = -1;
    }
}


// static
bool SensorQsocket::send(int service, int instance, const std::string& data) {
    return send(service, instance, (const uint8_t*)data.c_str(), data.length());
}

bool findService(int fd, sockaddr_qrtr& addr, int service, int instance, bool &serviceDeleted) {
    bool serviceFound = false;
    int len = 0;
    do {
        // service has been deleted and it has not restarted yet, simply return
        if (true == serviceDeleted) {
            break;
        }
        (void)memset(&addr, 0, sizeof(addr));
        socklen_t sl = sizeof(addr);
        // get socket name
        int rc = getsockname(fd, (void*)&addr, &sl);
        if (rc || addr.sq_family != AF_QIPCRTR || sl != sizeof(addr)) {
            SENSOR_LOGE(LOG_TAG "error: getsockname rc=%d errno=%d (%s)", rc, errno, strerror(errno));
            break;
        }
        addr.sq_port = QRTR_PORT_CTRL;
        // send control packet
        struct qrtr_ctrl_pkt pkt = {0};
	struct qrtr_ctrl_pkt mCtrlPkt = {0};

        mCtrlPkt.cmd = cpu_to_le32(QRTR_TYPE_NEW_LOOKUP);
        mCtrlPkt.server.service = cpu_to_le32(service);
        mCtrlPkt.server.instance = cpu_to_le32(instance);

        rc = sendto(fd, &mCtrlPkt, sizeof(mCtrlPkt), 0, (void*)&addr, sizeof(addr));
        if (rc < 0) {
            SENSOR_LOGE(LOG_TAG "sendto failed!");
            break;
        }
        // get server addr from ipc router
        serviceFound = false;
        while ((len = recv(fd, &pkt, sizeof(pkt), 0)) > 0) {
            unsigned int type = le32_to_cpu(pkt.cmd);
	    SENSOR_LOGD(LOG_TAG "qrtr new lookup pkt type:%d,"
                                 "service id:%d,instance id:%d,"
                                 "node:%d, port:%d\n",
                                 type, le32_to_cpu(pkt.server.service),
                                 le32_to_cpu(pkt.server.instance),
                                 le32_to_cpu(pkt.server.node),
                                 le32_to_cpu(pkt.server.port));

            if (len < sizeof(pkt)) {
                SENSOR_LOGE(LOG_TAG "invalid/short packet size %d, expected size %d\n", len, sizeof(pkt));
                continue;
            }
	    if ( type == QRTR_TYPE_NEW_SERVER) {
		    if (le32_to_cpu(pkt.server.service) == 0 &&
				    le32_to_cpu(pkt.server.instance) == 0) {
			    serviceFound = false;
			    break;
		    } else if ((mCtrlPkt.server.service == pkt.server.service) &&
				    (mCtrlPkt.server.instance == pkt.server.instance)) {
			    addr.sq_node = le32_to_cpu(pkt.server.node);
			    addr.sq_port = le32_to_cpu(pkt.server.port);
			    serviceFound = true;
			    break;
		    }
	    }
	    if ( type == QRTR_TYPE_DEL_SERVER) {
		    SENSOR_LOGE(LOG_TAG "server deleted\n");
		    serviceDeleted = true;
		    if ((mCtrlPkt.server.service == pkt.server.service) &&
				    (mCtrlPkt.server.instance == pkt.server.instance)) {
			    // service of particular service id, instance id gets deleted
			    serviceFound = false;
			    break;
		    }
	    }
	}
	if (true == serviceFound) {
		addr.sq_node = le32_to_cpu(pkt.server.node);
		addr.sq_port = le32_to_cpu(pkt.server.port);
	}
	SENSOR_LOGD(LOG_TAG "after while loop len %d, serviceFound = %d!\n", len, serviceFound);
    } while (0);

    return serviceFound;
}


// static
bool SensorQsocket::send(int service, int instance, const uint8_t data[], uint32_t length) {
    bool result = false;
    int fd = socket(AF_QIPCRTR, SOCK_DGRAM, 0);
    if (fd < 0) {
        SENSOR_LOGE(LOG_TAG "create socket error. reason:%s\n", strerror(errno));
        return false;
    }

    timeval timeout;
    timeout.tv_sec = SOCKET_SENDER_SEND_TIMEOUT_MSEC / 1000;
    timeout.tv_usec = SOCKET_SENDER_SEND_TIMEOUT_MSEC % 1000 * 1000;
    (void)setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    sockaddr_qrtr addr;
    (void)memset(&addr, 0, sizeof(addr));
    // this routine is used to send an abort message to
    // the socket itself, so it is safe to set serviceDeleted to false
    bool serviceDeleted = false;
    result = findService(fd, addr, service, instance, serviceDeleted);
    if (true == result) {
	    result = sendData(fd, addr, data, length);
    }
    (void)close(fd);
    return result;
}

// static
bool SensorQsocket::sendData(int fd, const sockaddr_qrtr& addr, const uint8_t data[], uint32_t length) {

    bool result = true;

    if (length <= SENSOR_MSG_BUF_LEN) {
        if (sendto(fd, data, length, 0, (void*)&addr, sizeof(addr)) < 0) {
            SENSOR_LOGE(LOG_TAG "cannot send to socket. reason:%s\n", strerror(errno));
            result = false;
        }
    } else {
        std::string head = SENSOR_MSG_HEAD;
        (void)head.append(std::to_string(length));
        if (sendto(fd, head.c_str(), head.length(), 0, (void*)&addr, sizeof(addr)) < 0) {
            SENSOR_LOGE(LOG_TAG "cannot send to socket. reason:%s\n", strerror(errno));
            result = false;
        } else {
            size_t sentBytes = 0;
            while(sentBytes < length) {
                size_t partLen = length - sentBytes;
                if (partLen > SENSOR_MSG_BUF_LEN) {
                    partLen = SENSOR_MSG_BUF_LEN;
                }
                ssize_t rv = sendto(fd, (data + sentBytes), partLen, 0, (void*)&addr, sizeof(addr));
                if (rv < 0) {
                    SENSOR_LOGE(LOG_TAG "cannot send to socket. reason:%s\n", strerror(errno));
                    result = false;
                    break;
                }
                sentBytes += rv;
            }
        }
    }
    return result;
}
#endif
}
