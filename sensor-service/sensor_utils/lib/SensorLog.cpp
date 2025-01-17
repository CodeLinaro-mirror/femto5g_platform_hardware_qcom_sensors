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
 * Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include <ctype.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <SensorLog.h>
#include <errno.h>
#include <stdlib.h>
#include <vector>
#include <SensorDiagLog.h>

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "SensorSvc_SensorLog:"

//Config file
#define SENSOR_CONF_PATH "/etc/sensors.conf"

int DEBUG_LEVEL = 0;

/**
 * Sensor_Read_Sensor_Config: Read config from the given file and store the value
 * @param file_name         : Path to configuration file
 * @param config_name       : Config name including '=' (eg: DEBUG=)
 * @param out_fmt           : Format specifier for the output
 * @param out               : Address to store the output to
 * @return                  : Returns 0 on success, else returns -ve errno
 */
static int Sensor_Read_Sensor_Config(const char *file_name, const char *config_name, 
const char *out_fmt, void *out)
{
    FILE *file;
    char buffer[BUFSIZ];
    char *line;
    int i;
    int ret = -EINVAL;

    file = fopen(file_name, "r");
    if (file == NULL) {
        SENSOR_LOGE(LOG_TAG "file open failed : %d(%s)\n", errno, strerror(errno));
        return -EINVAL;
    }

    while(fgets(buffer, sizeof(buffer), file) != NULL) {
        for(i = 0; i < strlen(buffer); i++) { // iterate through the chars in a line
            if(buffer[i] == '#') { // if char is a #, stop processing chars on this line
                break;
            } else if(buffer[i] == ' ') { // if char is whitespace, continue until something is found
                continue;
            }
            else if(strstr(buffer, config_name)) {
                line = strstr(buffer, "=");
                if (line == nullptr) {
                    continue;
                }
                (void)sscanf(&line[1], out_fmt, out);
                ret = 0;
                break;
            }
        }
    }
    (void)fclose(file);
    return ret;
}

/**
 * strtok_safe:  Safer version of strtok
 * @param inp_str:  Input string to search in
 * @param inp_size: Size of input string
 * @param skip: No of characters to skip from input string. '2' means skip first 2 character
 * @param del: Delimeter string. It must end with '\0'
 * @param out_tok: Pointer to string to store the output token. The size of out_tok must be atleast inp_size
 * @return Returns -1 if no more matches. Else returns the starting index of the match
 */

static int strtok_safe(const char *inp_str, const size_t inp_size, const size_t skip, const char *del, char *out_tok)
{
    int idx = skip;
    int out_idx = 0;
    int check_size = strlen(inp_str);
    if(check_size > inp_size)
    {
        check_size = inp_size;
    }
    if(skip >= check_size)
    {
        return -1;
    }
    while(idx < check_size)
    {
        bool delFound = false;
        for(size_t i=0; i<(check_size - idx); i++)
        {
            if(del[i] == '\0')
            {
                delFound = true;
                break;
            }
            else if(del[i] != inp_str[idx + i])
            {
                break;
            }
        }
        if(delFound)
        {
            out_tok[out_idx] = '\0';
            break;
        }
        out_tok[out_idx] = inp_str[idx];
        idx++;
        out_idx++;
    }
    if(idx < inp_size && inp_str[idx] == '\0')
    {
        out_tok[out_idx] = '\0';
    }
    return idx - out_idx;
}

int SensorReadDebugLevel() {
    int ret = Sensor_Read_Sensor_Config(SENSOR_CONF_PATH, "DEBUG_LEVEL=", "%d", &DEBUG_LEVEL);
    if(ret != 0)
    {
        SENSOR_LOGE(LOG_TAG "invalid config\n");
    }
    return DEBUG_LEVEL;
}

void SetSensorDebugLevel(int debug_level) {

    DEBUG_LEVEL = debug_level;
}


/**
 * frac_to_float_array: Parses list of fractions in the form a1/b1,a2/b2,... and returns a float array
 */
static int frac_to_float_array(char *str_arr, std::vector<float> &out, size_t max_size)
{
    char val_str[1024];
    char val_str1[21];
    int numerator, denominator;
    int skip1=0;
    int skip = 0;
    int order = 0;
    std::vector<float> parsed_arr;
    parsed_arr.reserve(max_size);
    while((skip = strtok_safe(str_arr, 1024, skip, ",", val_str)) != -1)
    {
        if(order >= max_size)
        {
            SENSOR_LOGE(LOG_TAG "size of x coefficient array is more than max defined (%lu), skipping rest\n", max_size);
            break;
        }
        skip1 = 0;
        skip1 = strtok_safe(val_str, 21, skip1, "/", val_str1);
        if(skip1 == -1)
        {
            SENSOR_LOGE(LOG_TAG "Invalid coefficient value, entries must be in format a/b\n");
            return 0;
        }
        //Check if all the characters are numeric for numerator
        skip1 = strlen(val_str1);
        for(int i=0; i<skip1; i++)
        {
            if(!isdigit(val_str1[i]))
            {
                SENSOR_LOGE(LOG_TAG "Invalid coefficient value, value (%s) is not numeric\n", val_str1);
                return 0;
            }
        }
        numerator = atoi(val_str1);

        //check if all the characters are numeric for denonimator
        for(int i=skip1+1; i<strlen(val_str); i++)
        {
            if(!isdigit(val_str[i]))
            {
                SENSOR_LOGE(LOG_TAG "Invalid coefficient value, value (%s) is not numeric\n", val_str + skip1+1);
                return 0;
            }
        }
        denominator = atoi(val_str + skip1 + 1);

        if(denominator == 0)
        {
            SENSOR_LOGE(LOG_TAG "Invalid coefficient value, denominator is zero\n");
            return 0;
        }
        parsed_arr.push_back((numerator + 0.0) / denominator);
        skip = skip + strlen(val_str) + 1; //+1 for delimeter ','
        order++;
    }
    out.insert(out.end(), parsed_arr.begin(), parsed_arr.end());
    return order;
}

int CheckDiagEnabled(const char *client_name)
{
    char diag_clients[MAX_DIAG_CLIENTS * MAX_CLIENT_NAME_LEN + 1];//+1 for /0
    int ret = Sensor_Read_Sensor_Config(SENSOR_CONF_PATH, "DIAG_CLIENTS=", "%s", (void*)&diag_clients[0]);
    if(ret != 0)
    {
        SENSOR_LOGE(LOG_TAG "invalid config\n");
        return 0;
    }
    char client[MAX_DIAG_CLIENTS * MAX_CLIENT_NAME_LEN + 1];
    int skip = 0;
    while((skip = strtok_safe(diag_clients, MAX_DIAG_CLIENTS * MAX_CLIENT_NAME_LEN + 1, skip, ",", client)) != -1)
    {
        if(strncmp(client, client_name, MAX_CLIENT_NAME_LEN) == 0)
        {
            return 1;
        }
        skip = skip + strlen(client) + 1; //+1 for delimeter ","
    }
    return 0;
}

int GetFIRCoefficient(std::vector<float> &coef, char *suffix)
{
    char conf_coef[1024];
    char key_val[1024];
    int order = 0;
    int ret = 0;

    snprintf(key_val, 1024, "FIR_COEFFICIENT_%s=", suffix);
    ret = Sensor_Read_Sensor_Config(SENSOR_CONF_PATH, key_val, "%s", (void*)&conf_coef);
    if(ret != 0)
    {
        SENSOR_LOGE(LOG_TAG "invalid config for FIR_COEFFICIENT_%s\n", suffix);
        return -1;
    }
    coef.reserve(MAX_FIR_COEF_ORDER);
    order = frac_to_float_array(conf_coef, coef, MAX_FIR_COEF_ORDER);
    return order == 0 ? -1 : order;
}
