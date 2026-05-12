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
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <grp.h>
#include <unistd.h>
#include <sys/prctl.h>
#include <sys/capability.h>
#include <atomic>
#include <SensorApiService.h>

#define HAL_DAEMON_VERSION "1.0.0"

std::atomic<bool> shutdown_due_to_sigterm{false};

// this function will block until the directory specified in
// dirName has been created
static inline void waitForDir(const char* dirName) {
    // wait for parent direcoty to be created...
    struct stat buf_stat;
    while (1) {
        SENSOR_LOGI(LOG_TAG "waiting for %s...\n", dirName);
        int rc = stat(dirName, &buf_stat);
        if (!rc) {
            break;
        }
        usleep(100000); //100ms
    }
    SENSOR_LOGI(LOG_TAG "WaitForDir done\n");
}

//Read all parameters defined etc/sensors.conf
void SENSOR_READ_CONF(char *file_name, configParamToRead *configParamRead)
{
    FILE *file = NULL;
    char buffer[BUFSIZ];
    char *line;
    int i;

    file = fopen(file_name, "r");
    if (file == NULL) {
	SENSOR_LOGE(LOG_TAG "open failed: %s: %s\n", file_name, strerror(errno));
	return;
    }

    while(fgets(buffer, sizeof(buffer), file) != NULL) {
       for(i = 0; i < strlen(buffer); i++) { // iterate through the chars in a line
         if(buffer[i] == '#') { // if char is a #, stop processing chars on this line
		 break;
	 } else if(buffer[i] == ' ') { // if char is whitespace, continue until something is found
		 continue;
	 } else if(strstr(buffer, "SENSOR_VENDOR=")) {
		 line = strstr(buffer, "=");
		 sscanf(&line[1], "%d", &configParamRead->SensorType);
		 break;
	 }
	 else if(strstr(buffer, "SENSOR_HAL_LIB_PATH=")) {
		 line = strstr(buffer, "=");
		 strlcpy(configParamRead->SensorHalLibPath, &line[1], sizeof(configParamRead->SensorHalLibPath));
		 configParamRead->SensorHalLibPath[strlen(configParamRead->SensorHalLibPath)-1] = '\0';
		 break;
	 }
	 else if(strstr(buffer, "ACCEL_NAME=")) {
                 line = strstr(buffer, "=");
                 strlcpy(configParamRead->AccelName, &line[1], sizeof(configParamRead->AccelName));
                 configParamRead->AccelName[strlen(configParamRead->AccelName)-1] = '\0';
                 break;
         }
	 else if(strstr(buffer, "GYRO_NAME=")) {
                 line = strstr(buffer, "=");
                 strlcpy(configParamRead->GyroName, &line[1],sizeof(configParamRead->GyroName));
                 configParamRead->GyroName[strlen(configParamRead->GyroName)-1] = '\0';
                 break;
         }
         else if(strstr(buffer, "DYAMIC_CONFIG_ENABLED=")) {
                 line = strstr(buffer, "=");
		 sscanf(&line[1], "%d", &configParamRead->DynamicConfigEnabled);
                 break;
         }
         else if(strstr(buffer, "MAX_ACC_SAMPLING_RATE=")) {
                 line = strstr(buffer, "=");
                 sscanf(&line[1], "%f", &configParamRead->MaxAccSampleRate);
                 break;
         }
         else if(strstr(buffer, "MAX_GYRO_SAMPLING_RATE=")) {
                 line = strstr(buffer, "=");
                 sscanf(&line[1], "%f", &configParamRead->MaxGyroSampleRate);
                 break;
         }
         else if(strstr(buffer, "ACC_RANGE=")) {
                 line = strstr(buffer, "=");
                 sscanf(&line[1], "%d", &configParamRead->AccRange);
                 break;
         }
         else if(strstr(buffer, "GYRO_RANGE=")) {
		 line = strstr(buffer, "=");
		 sscanf(&line[1], "%d", &configParamRead->GyroRange);
		 break;
	 }
	 else if(strstr(buffer, "MIN_ACC_BATCH_COUNT=")) {
                 line = strstr(buffer, "=");
                 sscanf(&line[1], "%d", &configParamRead->MinAccBatchCount);
                 break;
         }
         else if(strstr(buffer, "MIN_GYRO_BATCH_COUNT=")) {
                 line = strstr(buffer, "=");
                 sscanf(&line[1], "%d", &configParamRead->MinGyroBatchCount);
                 break;
         }
         else if(strstr(buffer, "DEBUG_LEVEL=")) {
                 line = strstr(buffer, "=");
                 sscanf(&line[1], "%d", &configParamRead->DebugLevel);
                 break;
         }
       }
    }
    fclose(file);
}

//Print the all values of parametets defined in etc/sensors.conf file
void PrintSensorConfigParameters(configParamToRead configParamRead)
{
   SENSOR_LOGI(LOG_TAG "sensor type:%d\n \
	\tsensor_lib:%s\n \
	\tdynamicconfig:%d\n \
	\tAccelName %s: AccSamplingRate :%f AccBatchcount :%d AccRange :%d\n \
	\tGyroName  %s:GyroSamplingRate:%f GyroBatchcount:%d GyroRange:%d\n \
	\tDebugLevel:%d\n",
	configParamRead.SensorType,
	configParamRead.SensorHalLibPath,
	configParamRead.DynamicConfigEnabled,
	configParamRead.AccelName, configParamRead.MaxAccSampleRate, configParamRead.MinAccBatchCount, configParamRead.AccRange,
	configParamRead.GyroName,  configParamRead.MaxGyroSampleRate, configParamRead.MinGyroBatchCount, configParamRead.GyroRange,
	configParamRead.DebugLevel);
}

static void block_sigterm()
{
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGTERM);

    // Block SIGTERM in current thread (and all future threads)
    int rc = pthread_sigmask(SIG_BLOCK, &set, nullptr);
         if(rc != 0)
       	    SENSOR_LOGE(LOG_TAG "pthread_sigmask failed rc=%d\n", rc);

}

static void* sigterm_wait_thread(void*)
{
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, nullptr);
    pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, nullptr);

    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGTERM);

    int sig = 0;
    int rc = sigwait(&set, &sig);
    if(rc == 0 && sig == SIGTERM){
       	SENSOR_LOGI(LOG_TAG "SIGTERM received, shutting down...\n");
	shutdown_due_to_sigterm.store(true, std::memory_order_release);
        SensorApiService::requestStop();
    }
    else
        SENSOR_LOGE(LOG_TAG "sigwait failed rc=%d sig=%d\n", rc, sig);

    return nullptr;
}

//MAIN
int main(int argc, char *argv[])
{
    block_sigterm();
    pthread_t sig_thread;
    bool rc = Sensor_ThreadCreate(&sig_thread, sigterm_wait_thread, NULL, "sigterm_wait_main_func", false);
    if(rc == false)
       SENSOR_LOGE(LOG_TAG "pthread_create failed rc=%d\n", rc);

    configParamToRead configParamRead = {};

    // read configuration file
    SENSOR_READ_CONF(SENSOR_CONF_PATH, &configParamRead);
    SetSensorDebugLevel(configParamRead.DebugLevel);
    PrintSensorConfigParameters(configParamRead);

    SENSOR_LOGI(LOG_TAG "sensor hal daemon - ver %s\n", HAL_DAEMON_VERSION);

    waitForDir(SOCKET_SENSOR_CLIENT_DIR);

    SENSOR_LOGI(LOG_TAG "starting sensor_hal_daemon\n");

    // start listening for client events - will not return
    (void)SensorApiService::getInstance(configParamRead);

    if(SensorApiService::mStopNeeded == true)
        SensorApiService::requestStop();

    // If shutdown was NOT caused by SIGTERM, cancel sigwait thread
    if (!shutdown_due_to_sigterm.load(std::memory_order_acquire) && rc == true)
        pthread_cancel(sig_thread);

    if (rc == true)
        pthread_join(sig_thread, nullptr);

    SensorApiService::destroy();
    SENSOR_LOGI(LOG_TAG "done\n");
    exit(0);
}
