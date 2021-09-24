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
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <SensorQsocket.h>

namespace sensor_util {

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "SensorSvc_SensorQSocket:"

#define SENSOR_MSG_BUF_LEN 8192
#define SENSOR_MSG_HEAD "$MSGLEN$"
#define SENSOR_MSG_ABORT "SensorQsocketMsg::ABORT"
#define RETRY_FINDSERVICE_MAX_COUNT 10
#define RETRY_FINDSERVICE_SLEEP_MS  5

bool SensorQsocket::startListeningNonBlocking(const std::string& name) {
    mQsocketName = name;
    return Sensor_ThreadCreate(&mQsocketThread, startListeningNonBlockingThread, this, "SensorQsocket-");
}

void* SensorQsocket::startListeningNonBlockingThread(void *arg) {
   SensorQsocket* mSensorQsocket = (SensorQsocket*)arg;
   mSensorQsocket->startListeningBlocking(mSensorQsocket->mQsocketName);
}

#ifdef USE_QSOCKET
bool SensorQsocket::startListeningBlocking(const std::string& name) {
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
    SENSOR_LOGV(LOG_TAG "find service returned %d, retryCount %d", result, retryCount);
    return result;
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

#else

static inline __le32 cpu_to_le32(uint32_t x) { return htole32(x); }
static inline uint32_t le32_to_cpu(__le32 x) { return le32toh(x); }

bool SensorQsocket::startListeningBlocking(const std::string& name) {
    bool stopRequested = false;

    // name is of format "serviceid.instanceid", and thus
    // should not exceed 65 bytes
    mService = atoi(name.c_str());
    const char* instance_ptr = strchr(name.c_str(), '.');
    if (nullptr != instance_ptr) {
	    instance_ptr++;
    }

    mInstance = atoi(instance_ptr);

    // socket
    int fd = socket(AF_QIPCRTR, SOCK_DGRAM, 0);
    if (fd < 0) {
        SENSOR_LOGE(LOG_TAG "create socket error. reason:%s\n", strerror(errno));
        return false;
    }

    // get socket name
    sockaddr_qrtr addr = {0};
    socklen_t sl = sizeof(addr);
    int rc = getsockname(fd, (void*)&addr, &sl);
    if (rc || addr.sq_family != AF_QIPCRTR || sl != sizeof(addr)) {
        SENSOR_LOGE(LOG_TAG "error: getsockname rc=%d errno=%d (%s)\n", rc, errno, strerror(errno));
        return false;
    }

    SENSOR_LOGI(LOG_TAG "family=%u, node=%d, port=%d\n", addr.sq_family, addr.sq_node, addr.sq_port);
    // register this server by sending control packet
    struct qrtr_ctrl_pkt pkt = {0};
    pkt.cmd = cpu_to_le32(QRTR_TYPE_NEW_SERVER);
    pkt.server.service = cpu_to_le32(mService);
    pkt.server.instance = cpu_to_le32(mInstance);
    pkt.server.node = cpu_to_le32(addr.sq_node);
    pkt.server.port = cpu_to_le32(addr.sq_port);
    addr.sq_port = QRTR_PORT_CTRL;
    rc = sendto(fd, &pkt, sizeof(pkt), 0, (void*)&addr, sizeof(addr));
    if (rc < 0) {
        SENSOR_LOGE(LOG_TAG "send control packet failed\n");
        return false;
    }

    mFdMe = fd;
    SENSOR_LOGI(LOG_TAG "family=%u, node=%d, port=%d\n", mDestAddr.sq_family, mDestAddr.sq_node, mDestAddr.sq_port);

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
            onReceive(msg);
        } else {
            // long message
            size_t msgLen = 0;
            sscanf(msg.data(), SENSOR_MSG_HEAD"%zu", &msgLen);
            msg.resize(msgLen);
            size_t msgLenReceived = 0;
            while ((msgLenReceived < msgLen) && (nBytes > 0)) {
                nBytes = recvfrom(fd, (void*)&(msg[msgLenReceived]),
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

    return stopRequested;
}

bool SensorQsocket::findService(int fd, sockaddr_qrtr& addr, int service, int instance) {

    memset(&addr, 0, sizeof(addr));
    socklen_t sl = sizeof(addr);

    // get socket name
    int rc = getsockname(fd, (void*)&addr, &sl);
    if (rc || addr.sq_family != AF_QIPCRTR || sl != sizeof(addr)) {
        SENSOR_LOGE(LOG_TAG "error: getsockname rc=%d errno=%d (%s)\n", rc, errno, strerror(errno));
        return false;
    }
    addr.sq_port = QRTR_PORT_CTRL;
    // send control packet
    struct qrtr_ctrl_pkt pkt = {0};
    pkt.cmd = cpu_to_le32(QRTR_TYPE_NEW_LOOKUP);
    pkt.server.service = cpu_to_le32(service);
    pkt.server.instance = cpu_to_le32(instance);
    rc = sendto(fd, &pkt, sizeof(pkt), 0, (void*)&addr, sizeof(addr));
    if (rc < 0) {
        SENSOR_LOGE(LOG_TAG "sendto failed!\n");
        return false;
    }
    // get server addr from ipc router
    int len;
    while ((len = recv(fd, &pkt, sizeof(pkt), 0)) > 0) {
        unsigned int type = le32_to_cpu(pkt.cmd);

	if (QRTR_TYPE_DEL_SERVER == type) {
		SENSOR_LOGE(LOG_TAG "server deleted\n");
		return false;
	}

        if (len < sizeof(pkt) || type != QRTR_TYPE_NEW_SERVER) {
            SENSOR_LOGE(LOG_TAG "invalid/short packet\n");
            continue;
        }

        SENSOR_LOGD(LOG_TAG "service=%d\n", le32_to_cpu(pkt.server.service));
        SENSOR_LOGD(LOG_TAG "version=%d\n", le32_to_cpu(pkt.server.instance) & 0xff);
        SENSOR_LOGD(LOG_TAG "instance=%d\n", le32_to_cpu(pkt.server.instance) >> 8);
        SENSOR_LOGD(LOG_TAG "node=%d\n", le32_to_cpu(pkt.server.node));
        SENSOR_LOGD(LOG_TAG "port=%d\n", le32_to_cpu(pkt.server.port));

        if (0 != pkt.server.service) {
            break;
        }
    }
    if (len <= 0) {
        SENSOR_LOGE(LOG_TAG "recv() returned %d bytes\n", len);
        return false;
    }

    addr.sq_node = le32_to_cpu(pkt.server.node);
    addr.sq_port = le32_to_cpu(pkt.server.port);
    return true;
}

bool SensorQsocket::findServiceWithRetry(int fd, sockaddr_qrtr& addr, int service, int instance) {
    int retryCount = 0;
    bool result = false;
    do {
        result = SensorQsocket::findService(fd, addr, service, instance);
        if (true == result) {
            break;
        }
        usleep(RETRY_FINDSERVICE_SLEEP_MS * 1000);
    } while (retryCount++ <= RETRY_FINDSERVICE_MAX_COUNT);
    SENSOR_LOGV(LOG_TAG "find service with retry returned %d, retry count %d\n", result, retryCount);
    return result;
}

// static
bool SensorQsocket::send(int service, int instance, const uint8_t data[], uint32_t length) {

    bool result = false;
    int fd = socket(AF_QIPCRTR, SOCK_DGRAM, 0);
    if (fd < 0) {
        SENSOR_LOGE(LOG_TAG "create socket error. reason:%s\n", strerror(errno));
        return false;
    }

    sockaddr_qrtr addr;
    memset(&addr, 0, sizeof(addr));
    result = findServiceWithRetry(fd, addr, service, instance);
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
        head.append(std::to_string(length));
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

}
