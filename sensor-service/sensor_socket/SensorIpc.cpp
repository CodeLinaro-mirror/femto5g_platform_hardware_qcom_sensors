/* Copyright (c) 2019-2021 The Linux Foundation. All rights reserved.
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
#include <errno.h>
#include "SensorIpc.h"

namespace sensor_util {

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "SensorSvc_SensorIpc:"

#define SENSOR_MSG_BUF_LEN 8192
#define SENSOR_MSG_HEAD "$MSGLEN$"
#define SENSOR_MSG_ABORT "SensorIpcMsg::ABORT"

bool SensorIpc::startListeningNonBlocking(const std::string& name) {
    mIpcName = name;
    return Sensor_ThreadCreate(&mIpcThread, startListeningNonBlockingThread, this, "SensorIpc-");
}

void* SensorIpc::startListeningNonBlockingThread(void *arg) {
    SensorIpc* mSensorIpc = (SensorIpc*)arg;
    mSensorIpc->startListeningBlocking(mSensorIpc->mIpcName);
}

bool SensorIpc::startListeningBlocking(const std::string& name) {
    bool stopRequested = false;
    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);

    if (fd < 0) {
        SENSOR_LOGE(LOG_TAG "create socket error. reason:%s\n", strerror(errno));
        return false;
    }

    if ((unlink(name.c_str()) < 0) && (errno != ENOENT)) {
       SENSOR_LOGE(LOG_TAG "unlink socket error. reason:%s\n", strerror(errno));
    }

    struct sockaddr_un addr = { .sun_family = AF_UNIX };
    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", name.c_str());

    umask(0157);

    if (::bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        SENSOR_LOGE(LOG_TAG "bind socket error. reason:%s\n", strerror(errno));
    } else {
        mIpcFd = fd;

	// inform that the socket is ready to receive message
	onListenerReady();

        ssize_t nBytes = 0;
        std::string msg = "";
        std::string abort = SENSOR_MSG_ABORT;
        while (1) {
            msg.resize(SENSOR_MSG_BUF_LEN);
            nBytes = ::recvfrom(fd, (void*)(msg.data()), msg.size(), 0, NULL, NULL);
            if (nBytes < 0) {
                SENSOR_LOGE(LOG_TAG "cannot read socket. reason:%s\n", strerror(errno));
                break;
            } else if (0 == nBytes) {
                continue;
            }

            if (strncmp(msg.data(), abort.c_str(), abort.length()) == 0) {
                SENSOR_LOGE(LOG_TAG "recvd abort msg.data %s\n", msg.data());
                stopRequested = true;
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
                    nBytes = recvfrom(fd, (void*)&(msg[msgLenReceived]),
                                      msg.size() - msgLenReceived, 0, NULL, NULL);
                    msgLenReceived += nBytes;
                }
                if (nBytes > 0) {
		    nBytes = msgLenReceived;
                    onReceive(msg);
                } else {
                    SENSOR_LOGE(LOG_TAG "cannot read socket. reason:%s\n", strerror(errno));
                    break;
                }
            }
        }
    }

    if (::close(fd)) {
        SENSOR_LOGE(LOG_TAG "cannot close socket:%s\n", strerror(errno));
    }
    unlink(name.c_str());

    return stopRequested;
}

void SensorIpc::stopListening() {
    const char *socketName = nullptr;

    if (mIpcFd >= 0) {
        std::string abort = SENSOR_MSG_ABORT;
        socketName = mIpcName.c_str();
        send(socketName, abort);
        mIpcFd = -1;
    }
}

bool SensorIpc::send(const char name[], const std::string& data) {
    return send(name, (const uint8_t*)data.c_str(), data.length());
}

bool SensorIpc::send(const char name[], const uint8_t data[], uint32_t length) {

    bool result = true;
    int fd = ::socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd < 0) {
        SENSOR_LOGE(LOG_TAG "create socket error. reason:%s\n", strerror(errno));
        return false;
    }

    struct sockaddr_un addr = { .sun_family = AF_UNIX };
    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", name);

    result = sendData(fd, addr, data, length);

    (void)::close(fd);
    return result;
}


bool SensorIpc::sendData(int fd, const sockaddr_un &addr, const uint8_t data[], uint32_t length) {

    bool result = true;

    if (length <= SENSOR_MSG_BUF_LEN) {
        if (::sendto(fd, data, length, 0,
                (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            SENSOR_LOGE(LOG_TAG "cannot send to socket:%s. reason:%s\n",
                    addr.sun_path, strerror(errno));
            result = false;
        }
    } else {
        std::string head = SENSOR_MSG_HEAD;
        head.append(std::to_string(length));
        if (::sendto(fd, head.c_str(), head.length(), 0,
                (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            SENSOR_LOGE(LOG_TAG "cannot send to socket:%s. reason:%s\n",
                    addr.sun_path, strerror(errno));
            result = false;
        } else {
            size_t sentBytes = 0;
            while(sentBytes < length) {
                size_t partLen = length - sentBytes;
                if (partLen > SENSOR_MSG_BUF_LEN) {
                    partLen = SENSOR_MSG_BUF_LEN;
                }
                ssize_t rv = ::sendto(fd, data + sentBytes, partLen, 0,
                        (struct sockaddr*)&addr, sizeof(addr));
                if (rv < 0) {
                    SENSOR_LOGE(LOG_TAG "cannot send to socket:%s. reason:%s\n",
                            addr.sun_path, strerror(errno));
                    result = false;
                    break;
                }
                sentBytes += rv;
            }
        }
    }
    return result;
}

}
