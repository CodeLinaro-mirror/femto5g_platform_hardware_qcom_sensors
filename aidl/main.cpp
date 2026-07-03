/*
 * Copyright (C) 2021 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * ​​​​​Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
 *
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include "sensors-impl/Sensors.h"

#include <android-base/logging.h>
#include <android/binder_manager.h>
#include <android/binder_process.h>
#include <signal.h>
#include <pthread.h>

using aidl::android::hardware::sensors::Sensors;

static void block_sigterm()
{
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGTERM);

    // Block SIGTERM in current thread (and all future threads)
    int rc = pthread_sigmask(SIG_BLOCK, &set, nullptr);
    if(rc != 0)
        SENSOR_LOGE(SENSOR_TAG "pthread_sigmask failed\n");

}

static void* sigterm_wait_thread(void*)
{
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGTERM);

    int sig = 0;
    int rc = sigwait(&set, &sig);
    if(rc == 0 && sig == SIGTERM){
        SENSOR_LOGI(SENSOR_TAG "SIGTERM received, shutting down...\n");
        cleanup();
    }
    else
        SENSOR_LOGE(SENSOR_TAG "sigwait failed rc=%d sig=%d\n", rc, sig);

    exit(0);
}

int main() {
    block_sigterm();
    pthread_t sig_thread;
    int rc = pthread_create(&sig_thread, nullptr, sigterm_wait_thread, nullptr);
    if(rc == 0){
        pthread_setname_np(sig_thread, "sigterm_wait_main_func");
        pthread_detach(sig_thread);
    }
    else
        SENSOR_LOGE(SENSOR_TAG "pthread_create failed rc=%d\n", rc);

    getSensorDebugLevel();
    SENSOR_LOGI(SENSOR_TAG "main start %s --> ", __func__);
    setenv("VSOMEIP_CONFIGURATION", "/vendor/etc/vsomeip_vlan1500.json", 1);
    setenv("COMMONAPI_CONFIG", "/vendor/etc/commonapi4someip.ini" ,1);

    ABinderProcess_setThreadPoolMaxThreadCount(0);

    // Make a default sensors service
    auto sensor = ndk::SharedRefBase::make<Sensors>();
    const std::string sensorName = std::string() + Sensors::descriptor + "/default";
    binder_status_t status =
            AServiceManager_addService(sensor->asBinder().get(), sensorName.c_str());
    CHECK_EQ(status, STATUS_OK);

    ABinderProcess_joinThreadPool();
    return EXIT_FAILURE;  // should not reach
}
