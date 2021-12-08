/*
Copyright (c) 2021 Qualcomm Innovation Center, Inc. All rights reserved.
 
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the
disclaimer below) provided that the following conditions are met:
 
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
 
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
 
    * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.
 
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <dlfcn.h>
#include <pthread.h>
#include <getopt.h>
#include <stdbool.h>
#include <linux/limits.h>
#include <sys/un.h>
#include <sys/time.h>
#include <sensor_util.h>
#include <math.h>

#define HAL_CONFIGURATION_FILE "hal_config"
#define HAL_CONFIGURATION_PATH "/data/sensorhal"

// This API is called to calculate rotational matrix
// It takes yaw, pitch and roll as a parameters
// The values should be inbetween o to 3600 range

int update_sensor_rotation_matrix(uint16_t yaw, uint16_t pitch, uint16_t roll)
{
        char *file_path_name = NULL;
        char *buffer_string = NULL;
        FILE *fd_config = NULL;
        int len = 0;
        int err = 0;
        char *ptr;
        int size = 100;
	int fsize = 0;
	char buffer[256];
	int point = 0;

	if(yaw < 0 || pitch < 0 || roll < 0 || yaw > 3600 || pitch > 3600 || roll > 3600){
		printf("Error: Invalid Range\n");
		return -1;
	}

        file_path_name = (char *)calloc(strlen(HAL_CONFIGURATION_PATH) + strlen(HAL_CONFIGURATION_FILE) + 2, 1);
        if (!file_path_name) {
		err = -errno;
                printf("Unable to allocate memory (errno %d)\n", err);

                return -ENOMEM;
        }

	fsize = strlen(HAL_CONFIGURATION_PATH) + strlen(HAL_CONFIGURATION_FILE);
        snprintf(file_path_name, fsize + 2, "%s/%s", HAL_CONFIGURATION_PATH, HAL_CONFIGURATION_FILE);
        fd_config = fopen(file_path_name, "r+");
        if (!fd_config) {
                err = -errno;
                printf("Filed to open %s (errno %d)\n",
                         file_path_name, err);
		perror("Error print by sensor util");

                goto err_out;
        }

        buffer_string = (char *)calloc(size, 1);
        if (!buffer_string) {
                err = -errno;
		perror("Error print by sensor util");
                printf("Unable to allocate memory (errno %d)\n", err);

                goto err_out;
        }

	fsize = strlen("imu_sensor_euler_angles = [00000,00000,00000]");
        size = snprintf(buffer_string, fsize + 1, "imu_sensor_euler_angles = [%5u,%5u,%5u]",
                       yaw, pitch, roll);

	printf("yaw = %u %d, pitch = %u %d, roll = %u %d\n", yaw, pitch, roll);
	printf("buffer_string = %s\n", buffer_string);
        printf("Update file in %s with %s\n", file_path_name, buffer_string);
	while(fgets(buffer, sizeof(buffer), fd_config) != NULL) {
			if(strstr(buffer, "imu_sensor_euler_angles = ")) {
				point = strlen(buffer);
				fseek(fd_config, -point, SEEK_CUR);
				len = fwrite(buffer_string, 1, size, fd_config);
                                 if (!len) {
                                        err = -errno;
					perror("Error print by sensor util");
                                        printf("Filed to write data to %s (errno %d)\n",
                                        file_path_name, err);
                                }
			break;
			}

	}

err_out:
        if (fd_config) {
                fclose(fd_config);
        }

        if (file_path_name) {
                free(file_path_name);
        }

        if (buffer_string) {
                free(buffer_string);
        }

        return err;
}

// This API is used to set the placement of the sensor
// It takes x, y and z as paramenters which represents the axis

int update_sensor_placement(int16_t x, int16_t y, int16_t z)
{
        char *file_path_name = NULL;
        char *buffer_string = NULL;
        FILE *fd_config = NULL;
        int len = 0;
        int err = 0;
        char *ptr;
        int size = 256;
	int fsize = 0;
	int point = 0;
	char bufferp[256];

        file_path_name = (char *)calloc(strlen(HAL_CONFIGURATION_PATH) + strlen(HAL_CONFIGURATION_FILE) + 2, 1);
        if (!file_path_name){
		err = -errno;
                printf("Unable to allocate memory (errno %d)\n", err);
		perror("Error print by sensor util");

                return -ENOMEM;
        }

	fsize = strlen(HAL_CONFIGURATION_PATH) + strlen(HAL_CONFIGURATION_FILE);
        snprintf(file_path_name, fsize + 2, "%s/%s", HAL_CONFIGURATION_PATH, HAL_CONFIGURATION_FILE);
        fd_config = fopen(file_path_name, "r+");
        if (!fd_config) {
                err = -errno;
                printf("Filed to open %s (errno %d)\n",
                         file_path_name, err);
		perror("Error print by sensor util");

                goto err_out;
        }

        buffer_string = (char *)calloc(size, 1);
        if (!buffer_string) {
                err = -errno;
                printf("Unable to allocate memory (errno %d)\n", err);

                goto err_out;
        }

	fsize = strlen("imu_sensor_placement = [000000,000000,000000]");
        size = snprintf(buffer_string, fsize + 1, "imu_sensor_placement = [%6d,%6d,%6d]",
                       x, y, z);

        printf("Update file in %s with %s\n", file_path_name, buffer_string);
	while(fgets(bufferp, sizeof(bufferp), fd_config) != NULL) {
                       if(strstr(bufferp, "imu_sensor_placement = ")) {
                                point = strlen(bufferp);
                                fseek(fd_config, -point, SEEK_CUR);
                                len = fwrite(buffer_string, 1, size, fd_config);
                                 if (!len) {
                                        err = -errno;
					perror("Error print by sensor util");
                                        printf("Filed to write data to %s (errno %d)\n",
                                        file_path_name, err);
                                }
                        break;
                        }
        }

err_out:
        if (fd_config) {
                fclose(fd_config);
        }

        if (file_path_name) {
                free(file_path_name);
        }

        if (buffer_string) {
                free(buffer_string);
        }

        return err;
}

int update_sensor_towing_jack_parameters(uint16_t threshold, uint32_t timer) {
//update below parameters with new values
//algo_towing_jack_delta_th = threshold
//algo_towing_jack_min_duration = timer

}

int update_sensor_crash_detection_parameters(uint16_t threshold, uint32_t timer) {

//update below parameters with new values
//algo_crash_impact_th = threshold
//algo_crash_min_duration = timer
}
