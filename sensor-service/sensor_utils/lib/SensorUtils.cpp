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
#include <math.h>
#include <SensorUtils.h>
#include <SensorLog.h>
#include <SensorThread.h>

#define HAL_CONFIGURATION_FILE "hal_config"
#define HAL_CONFIGURATION_PATH "/systemrw/sensorhal"

#define RM_MIN 0
#define RM_MAX 3600

#define POS_MIN -65
#define POS_MAX 65

#define CRASH_TH_MIN 100
#define CRASH_TH_MAX 2000
#define CRASH_TIMER_MIN 2
#define CRASH_TIMER_MAX 89000

#define TOWING_TH_MIN 10
#define TOWING_TH_MAX 1000
#define TOWING_TIMER_MIN 2
#define TOWING_TIMER_MAX 89000

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "SensorSvc_SensorUtils:"

// This API is called to initialise rotational matrix
int init_sensor_rotation_matrix(float (*rot) [3])
{
  rot[0][0] = 1;
  rot[0][1] = 0;
  rot[0][2] = 0;

  rot[1][0] = 0;
  rot[1][1] = 1;
  rot[1][2] = 0;

  rot[2][0] = 0;
  rot[2][1] = 0;
  rot[2][2] = 1;

  return 0;
}

// This API is called to calculate rotational matrix
int calculate_sensor_rotation_matrix(uint16_t rolld, uint16_t pitchd, uint16_t yawd, float (*rot) [3])
{
  // Calculate roll value
  float roll = (rolld / 10.0f) * M_PI / 180.0f;
  // Calculate pitch value
  float pitch = (pitchd / 10.0f) * M_PI / 180.0f;
  // Calculate yaw value
  float yaw = (yawd / 10.0f) * M_PI / 180.0f;

  // Calculate index[0][0] of Rotation Matrix
  rot[0][0] = cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) * sin(roll);
  // Calculate index[0][1] of Rotation Matrix
  rot[0][1] = -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll);
  // Calculate index[0][2] of Rotation Matrix
  rot[0][2] = cos(pitch) * sin(roll);

  // Calculate index[1][0] of Rotation Matrix
  rot[1][0] = sin(yaw) * cos(pitch);
  // Calculate index[1][1] of Rotation Matrix
  rot[1][1] = cos(yaw) * cos(pitch);
  // Calculate index[1][2] of Rotation Matrix
  rot[1][2] = -sin(pitch);

  // Calculate index[2][0] of Rotation Matrix
  rot[2][0] = -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll);
  // Calculate index[2][1] of Rotation Matrix
  rot[2][1] = sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll);
  // Calculate index[2][2] of Rotation Matrix
  rot[2][2] = cos(pitch) * cos(roll);

  SENSOR_LOGD(LOG_TAG "Sensor rotation matrix: \t%5.2f %5.2f %5.2f\t%5.2f %5.2f %5.2f\t%5.2f %5.2f %5.2f\n",
		  rot[0][0], rot[0][1], rot[0][2],
		  rot[1][0], rot[1][1], rot[1][2],
		  rot[2][0], rot[2][1], rot[2][2]);

  return 0;
}

// This API is called to read rotational matrix
int read_sensor_rotation_matrix(uint16_t *roll, uint16_t *pitch, uint16_t *yaw)
{
  char *file_path_name = NULL;
  FILE *fd_config = NULL;
  int fsize = 0;
  int size;
  char buffer[BUFSIZ];
  char *line = NULL;
  int err = 0;

  file_path_name = (char *)calloc(strlen(HAL_CONFIGURATION_PATH) + strlen(HAL_CONFIGURATION_FILE) + 2, 1);
  if (!file_path_name) {
	  err = -errno;
	  SENSOR_LOGE(LOG_TAG "Unable to allocate memory (errno %d)\n", err);
	  return -ENOMEM;
  }

  fsize = strlen(HAL_CONFIGURATION_PATH) + strlen(HAL_CONFIGURATION_FILE);
  (void)snprintf(file_path_name, fsize + 2, "%s/%s", HAL_CONFIGURATION_PATH, HAL_CONFIGURATION_FILE);
  fd_config = fopen(file_path_name, "r+");
  if (fd_config == NULL) {
	  err = -errno;
	  SENSOR_LOGE(LOG_TAG "Sensor Filed to open %s (errno %d)\n",
			  file_path_name, err);
  }
  else
  {
	  while(fgets(buffer, sizeof(buffer), fd_config) != NULL) {
		  if(strstr(buffer, "imu_sensor_euler_angles = ")) {
			  line = strstr(buffer, "[");
			  if(line != NULL){
				  size = sscanf(&line[1], "%d,%d,%d", roll, pitch, yaw);
			  }
			  break;
		  }
	  }
  }

  if(fd_config) (void)fclose(fd_config);
  free(file_path_name);
  file_path_name = NULL;

  return err;
}

// This API is called to calculate rotational matrix
// It takes yaw, pitch and roll as a parameters
// The values should be in between 0 to 3600 range
int update_sensor_rotation_matrix(uint16_t roll, uint16_t pitch, uint16_t yaw)
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

  if(yaw < RM_MIN || pitch < RM_MIN || roll < RM_MIN || yaw > RM_MAX || pitch > RM_MAX || roll > RM_MAX){
	  SENSOR_LOGE(LOG_TAG "Error: Invalid Range\n");
	  return -1;
  }

  file_path_name = (char *)calloc(strlen(HAL_CONFIGURATION_PATH) + strlen(HAL_CONFIGURATION_FILE) + 2, 1);
  if (!file_path_name) {
	  err = -errno;
	  SENSOR_LOGE(LOG_TAG "Unable to allocate memory (errno %d)\n", err);

	  return -ENOMEM;
  }

  fsize = strlen(HAL_CONFIGURATION_PATH) + strlen(HAL_CONFIGURATION_FILE);
  (void)snprintf(file_path_name, fsize + 2, "%s/%s", HAL_CONFIGURATION_PATH, HAL_CONFIGURATION_FILE);
  fd_config = fopen(file_path_name, "r+");
  if (!fd_config) {
	  err = -errno;
	  SENSOR_LOGE(LOG_TAG "Filed to open %s (errno %d)\n",
			  file_path_name, err);
	  goto err_out;
  }

  buffer_string = (char *)calloc(size, 1);
  if (!buffer_string) {
	  err = -errno;
	  SENSOR_LOGE(LOG_TAG "Unable to allocate memory (errno %d)\n", err);
	  goto err_out;
  }

  fsize = strlen("imu_sensor_euler_angles = [00000,00000,00000]");
  size = snprintf(buffer_string, fsize + 1, "imu_sensor_euler_angles = [%5u,%5u,%5u]",
		  roll, pitch, yaw);

  SENSOR_LOGI(LOG_TAG "roll = %u %d, pitch = %u %d, yaw = %u %d\n", roll, pitch, yaw);
  SENSOR_LOGI(LOG_TAG "buffer_string = %s\n", buffer_string);
  SENSOR_LOGI(LOG_TAG "Update file in %s with %s\n", file_path_name, buffer_string);
  while(fgets(buffer, sizeof(buffer), fd_config) != NULL) {
	  if(strstr(buffer, "imu_sensor_euler_angles = ")) {
		  point = strlen(buffer);
		  (void)fseek(fd_config, -point, SEEK_CUR);
		  len = fwrite(buffer_string, 1, size, fd_config);
		  if (!len) {
			  err = -errno;
			  SENSOR_LOGE(LOG_TAG "Filed to write data to %s (errno %d)\n",
					  file_path_name, err);
		  }
		  break;
	  }

  }

err_out:
  if (fd_config) {
	  (void)fclose(fd_config);
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

  if(x < POS_MIN || y < POS_MIN || z < POS_MIN || x > POS_MAX || y > POS_MAX || z > POS_MAX){
	  SENSOR_LOGE(LOG_TAG "Error: Invalid Range\n");
	  return -1;
  }

  file_path_name = (char *)calloc(strlen(HAL_CONFIGURATION_PATH) + strlen(HAL_CONFIGURATION_FILE) + 2, 1);
  if (!file_path_name){
	  err = -errno;
	  SENSOR_LOGE(LOG_TAG "Unable to allocate memory (errno %d)\n", err);
	  return -ENOMEM;
  }

  fsize = strlen(HAL_CONFIGURATION_PATH) + strlen(HAL_CONFIGURATION_FILE);
  (void)snprintf(file_path_name, fsize + 2, "%s/%s", HAL_CONFIGURATION_PATH, HAL_CONFIGURATION_FILE);
  fd_config = fopen(file_path_name, "r+");
  if (!fd_config) {
	  err = -errno;
	  SENSOR_LOGE(LOG_TAG "Filed to open %s (errno %d)\n",
			  file_path_name, err);
	  goto err_out;
  }

  buffer_string = (char *)calloc(size, 1);
  if (!buffer_string) {
	  err = -errno;
	  SENSOR_LOGE(LOG_TAG "Unable to allocate memory (errno %d)\n", err);
	  goto err_out;
  }

  fsize = strlen("imu_sensor_placement = [000000,000000,000000]");
  size = snprintf(buffer_string, fsize + 1, "imu_sensor_placement = [%6d,%6d,%6d]",
		  x, y, z);

  SENSOR_LOGI(LOG_TAG "Update file in %s with %s\n", file_path_name, buffer_string);
  while(fgets(bufferp, sizeof(bufferp), fd_config) != NULL) {
	  if(strstr(bufferp, "imu_sensor_placement = ")) {
		  point = strlen(bufferp);
		  (void)fseek(fd_config, -point, SEEK_CUR);
		  len = fwrite(buffer_string, 1, size, fd_config);
		  if (!len) {
			  err = -errno;
			  SENSOR_LOGE(LOG_TAG "Filed to write data to %s (errno %d)\n",
					  file_path_name, err);
		  }
		  break;
	  }
  }

err_out:
  if (fd_config) {
	  (void)fclose(fd_config);
  }

  if (file_path_name) {
	  free(file_path_name);
  }

  if (buffer_string) {
	  free(buffer_string);
  }

  return err;
}

// This API is used to update towing threshold and timer values
int update_sensor_towing_jack_parameters(uint16_t threshold, uint32_t timer) 
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

  if(threshold < TOWING_TH_MIN || threshold > TOWING_TH_MAX || timer < TOWING_TIMER_MIN || timer > TOWING_TIMER_MAX){
	  SENSOR_LOGE(LOG_TAG "Error: Invalid Range\n");
	  return -1;
  }

  file_path_name = (char *)calloc(strlen(HAL_CONFIGURATION_PATH) + strlen(HAL_CONFIGURATION_FILE) + 2, 1);
  if (!file_path_name){
	  err = -errno;
	  SENSOR_LOGE(LOG_TAG "Unable to allocate memory (errno %d)\n", err);
	  return -ENOMEM;
  }

  fsize = strlen(HAL_CONFIGURATION_PATH) + strlen(HAL_CONFIGURATION_FILE);
  (void)snprintf(file_path_name, fsize + 2, "%s/%s", HAL_CONFIGURATION_PATH, HAL_CONFIGURATION_FILE);
  fd_config = fopen(file_path_name, "r+");
  if (!fd_config) {
	  err = -errno;
	  SENSOR_LOGE(LOG_TAG "Filed to open %s (errno %d)\n",
			  file_path_name, err);
	  goto err_out;
  }

  buffer_string = (char *)calloc(size, 1);
  if (!buffer_string) {
	  err = -errno;
	  SENSOR_LOGE(LOG_TAG "Unable to allocate memory (errno %d)\n", err);
	  goto err_out;
  }

  fsize = strlen("algo_towing_jack_delta_th = 00000");
  size = snprintf(buffer_string, fsize + 1, "algo_towing_jack_delta_th = %5u",
		  threshold);

  SENSOR_LOGE(LOG_TAG "Update file in %s with %s\n", file_path_name, buffer_string);
  while(fgets(bufferp, sizeof(bufferp), fd_config) != NULL) {
	  if(strstr(bufferp, "algo_towing_jack_delta_th = ")) {
		  point = strlen(bufferp);
		  (void)fseek(fd_config, -point, SEEK_CUR);
		  len = fwrite(buffer_string, 1, size, fd_config);
		  if (!len) {
			  err = -errno;
			  SENSOR_LOGE(LOG_TAG "Filed to write data to %s (errno %d)\n",
					  file_path_name, err);
		  }
		  break;
	  }
  }

  fsize = strlen("algo_towing_jack_min_duration = 0000000000");
  size = snprintf(buffer_string, fsize + 1, "algo_towing_jack_min_duration = %10u",
		  timer);

  SENSOR_LOGI(LOG_TAG "Update file in %s with %s\n", file_path_name, buffer_string);
  while(fgets(bufferp, sizeof(bufferp), fd_config) != NULL) {
	  if(strstr(bufferp, "algo_towing_jack_min_duration = ")) {
		  point = strlen(bufferp);
		  (void)fseek(fd_config, -point, SEEK_CUR);
		  len = fwrite(buffer_string, 1, size, fd_config);
		  if (!len) {
			  err = -errno;
			  SENSOR_LOGE(LOG_TAG "Filed to write data to %s (errno %d)\n",
					  file_path_name, err);
		  }
		  break;
	  }
  }

err_out:
  if (fd_config) {
	  (void)fclose(fd_config);
  }

  if (file_path_name) {
	  free(file_path_name);
  }

  if (buffer_string) {
	  free(buffer_string);
  }

  return err;

}

// This API is used to update crash threshold and timer values
int update_sensor_crash_detection_parameters(uint16_t threshold, uint32_t timer) 
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

  if(threshold < CRASH_TH_MIN || threshold > CRASH_TH_MAX || timer < CRASH_TIMER_MIN || timer > CRASH_TIMER_MAX){
	  SENSOR_LOGE(LOG_TAG "Error: Invalid Range\n");
	  return -1;
  }

  file_path_name = (char *)calloc(strlen(HAL_CONFIGURATION_PATH) + strlen(HAL_CONFIGURATION_FILE) + 2, 1);
  if (!file_path_name){
	  err = -errno;
	  SENSOR_LOGE(LOG_TAG "Unable to allocate memory (errno %d)\n", err);
	  return -ENOMEM;
  }

  fsize = strlen(HAL_CONFIGURATION_PATH) + strlen(HAL_CONFIGURATION_FILE);
  (void)snprintf(file_path_name, fsize + 2, "%s/%s", HAL_CONFIGURATION_PATH, HAL_CONFIGURATION_FILE);
  fd_config = fopen(file_path_name, "r+");
  if (!fd_config) {
	  err = -errno;
	  SENSOR_LOGE(LOG_TAG "Filed to open %s (errno %d)\n",
			  file_path_name, err);
	  goto err_out;
  }

  buffer_string = (char *)calloc(size, 1);
  if (!buffer_string) {
	  err = -errno;
	  SENSOR_LOGE(LOG_TAG "Unable to allocate memory (errno %d)\n", err);
	  goto err_out;
  }

  fsize = strlen("algo_crash_impact_th = 00000");
  size = snprintf(buffer_string, fsize + 1, "algo_crash_impact_th = %5u",
		  threshold);

  SENSOR_LOGI(LOG_TAG "Update file in %s with %s\n", file_path_name, buffer_string);
  while(fgets(bufferp, sizeof(bufferp), fd_config) != NULL) {
	  if(strstr(bufferp, "algo_crash_impact_th = ")) {
		  point = strlen(bufferp);
		  (void)fseek(fd_config, -point, SEEK_CUR);
		  len = fwrite(buffer_string, 1, size, fd_config);
		  if (!len) {
			  err = -errno;
			  SENSOR_LOGE(LOG_TAG "Filed to write data to %s (errno %d)\n",
					  file_path_name, err);
		  }
		  break;
	  }
  }

  fsize = strlen("algo_crash_min_duration = 0000000000");
  size = snprintf(buffer_string, fsize + 1, "algo_crash_min_duration = %10u",
		  timer);

  SENSOR_LOGI(LOG_TAG "Update file in %s with %s\n", file_path_name, buffer_string);
  while(fgets(bufferp, sizeof(bufferp), fd_config) != NULL) {
	  if(strstr(bufferp, "algo_crash_min_duration = ")) {
		  point = strlen(bufferp);
		  (void)fseek(fd_config, -point, SEEK_CUR);
		  len = fwrite(buffer_string, 1, size, fd_config);
		  if (!len) {
			  err = -errno;
			  SENSOR_LOGE(LOG_TAG "Filed to write data to %s (errno %d)\n",
					  file_path_name, err);
		  }
		  break;
	  }
  }

err_out:
  if (fd_config) {
	  (void)fclose(fd_config);
  }

  if (file_path_name) {
	  free(file_path_name);
  }

  if (buffer_string) {
	  free(buffer_string);
  }

  return err;
}

// This API is used to update crash threshold and timer values
int update_ignition_state(uint32_t ign_state) 
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

  SENSOR_LOGI(LOG_TAG "ign_state %d\n", ign_state);
  if (ign_state != 1 && ign_state != 0) {
	  SENSOR_LOGE(LOG_TAG "Error: Invalid Range\n");
	  return -1;
  }

  file_path_name = (char *)calloc(strlen(HAL_CONFIGURATION_PATH) + strlen(HAL_CONFIGURATION_FILE) + 2, 1);
  if (!file_path_name){
	  err = -errno;
	  SENSOR_LOGE(LOG_TAG "Unable to allocate memory (errno %d)\n", err);
	  return -ENOMEM;
  }

  fsize = strlen(HAL_CONFIGURATION_PATH) + strlen(HAL_CONFIGURATION_FILE);
  (void)snprintf(file_path_name, fsize + 2, "%s/%s", HAL_CONFIGURATION_PATH, HAL_CONFIGURATION_FILE);
  fd_config = fopen(file_path_name, "r+");
  if (!fd_config) {
	  err = -errno;
	  SENSOR_LOGE(LOG_TAG "Filed to open %s (errno %d)\n",
			  file_path_name, err);
	  goto err_out;
  }

  buffer_string = (char *)calloc(size, 1);
  if (!buffer_string) {
	  err = -errno;
	  SENSOR_LOGE(LOG_TAG "Unable to allocate memory (errno %d)\n", err);
	  goto err_out;
  }

  fsize = strlen("ignition_off = 0");
  size = snprintf(buffer_string, fsize + 1, "ignition_off = %1u",
		  ign_state);

  SENSOR_LOGI(LOG_TAG "Update file in %s with %s\n", file_path_name, buffer_string);
  while(fgets(bufferp, sizeof(bufferp), fd_config) != NULL) {
	  if(strstr(bufferp, "ignition_off = ")) {
		  point = strlen(bufferp);
		  (void)fseek(fd_config, -point, SEEK_CUR);
		  len = fwrite(buffer_string, 1, size, fd_config);
		  if (!len) {
			  err = -errno;
			  SENSOR_LOGE(LOG_TAG "Filed to write data to %s (errno %d)\n",
					  file_path_name, err);
		  }
		  break;
	  }
  }

err_out:
  if (fd_config) {
	  (void)fclose(fd_config);
  }

  if (file_path_name) {
	  free(file_path_name);
  }

  if (buffer_string) {
	  free(buffer_string);
  }

  return err;
}
