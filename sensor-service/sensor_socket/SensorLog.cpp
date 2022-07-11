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
 */
#include <ctype.h>
#include <stdio.h>
#include <stdarg.h>
#include <SensorLog.h>
#include <errno.h>

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "SensorSvc_SensorLog:"

//Config file
#define SENSOR_CONF_PATH "/etc/sensors.conf"

int DEBUG_LEVEL = 0;

//Read DEBUG_LEVEL defined /etc/sensors.conf
static void Sensor_Read_Debug_Level(char *file_name) {

    FILE *file;
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
         }
         else if(strstr(buffer, "DEBUG_LEVEL=")) {
                 line = strstr(buffer, "=");
		 if (line == nullptr) {
			return;
		 }

                 sscanf(&line[1], "%d", &DEBUG_LEVEL);
                 break;
         }
       }
    }
    fclose(file);
}

int SensorReadDebugLevel() {

   Sensor_Read_Debug_Level(SENSOR_CONF_PATH);

   return DEBUG_LEVEL;
}

void SetSensorDebugLevel(int debug_level) {

    DEBUG_LEVEL = debug_level;
}
