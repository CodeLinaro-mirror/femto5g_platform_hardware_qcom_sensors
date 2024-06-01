/*!
 * @section LICENSE
 *
 * (C) Copyright 2011~2015 Bosch Sensortec GmbH All Rights Reserved
 *
 * (C) Modification Copyright 2018 Robert Bosch Kft  All Rights Reserved
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
 * Special: Description of the Software:
 *
 * This software module (hereinafter called "Software") and any
 * information on application-sheets (hereinafter called "Information") is
 * provided free of charge for the sole purpose to support your application
 * work. 
 *
 * As such, the Software is merely an experimental software, not tested for
 * safety in the field and only intended for inspiration for further development 
 * and testing. Any usage in a safety-relevant field of use (like automotive,
 * seafaring, spacefaring, industrial plants etc.) was not intended, so there are
 * no precautions for such usage incorporated in the Software.
 * 
 * The Software is specifically designed for the exclusive use for Bosch
 * Sensortec products by personnel who have special experience and training. Do
 * not use this Software if you do not have the proper experience or training.
 * 
 * This Software package is provided as is and without any expressed or
 * implied warranties, including without limitation, the implied warranties of
 * merchantability and fitness for a particular purpose.
 * 
 * Bosch Sensortec and their representatives and agents deny any liability for
 * the functional impairment of this Software in terms of fitness, performance
 * and safety. Bosch Sensortec and their representatives and agents shall not be
 * liable for any direct or indirect damages or injury, except as otherwise
 * stipulated in mandatory applicable law.
 * The Information provided is believed to be accurate and reliable. Bosch
 * Sensortec assumes no responsibility for the consequences of use of such
 * Information nor for any infringement of patents or other rights of third
 * parties which may result from its use.
 * 
 *------------------------------------------------------------------------------
 * The following Product Disclaimer does not apply to the BSX4-HAL-4.1NoFusion Software 
 * which is licensed under the Apache License, Version 2.0 as stated above.  
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Product Disclaimer
 *
 * Common:
 *
 * Assessment of Products Returned from Field
 *
 * Returned products are considered good if they fulfill the specifications / 
 * test data for 0-mileage and field listed in this document.
 *
 * Engineering Samples
 * 
 * Engineering samples are marked with (e) or (E). Samples may vary from the
 * valid technical specifications of the series product contained in this
 * data sheet. Therefore, they are not intended or fit for resale to
 * third parties or for use in end products. Their sole purpose is internal
 * client testing. The testing of an engineering sample may in no way replace
 * the testing of a series product. Bosch assumes no liability for the use
 * of engineering samples. The purchaser shall indemnify Bosch from all claims
 * arising from the use of engineering samples.
 *
 * Intended use
 *
 * Provided that SMI130 is used within the conditions (environment, application,
 * installation, loads) as described in this TCD and the corresponding
 * agreed upon documents, Bosch ensures that the product complies with
 * the agreed properties. Agreements beyond this require
 * the written approval by Bosch. The product is considered fit for the intended
 * use when the product successfully has passed the tests
 * in accordance with the TCD and agreed upon documents.
 *
 * It is the responsibility of the customer to ensure the proper application
 * of the product in the overall system/vehicle.
 *
 * Bosch does not assume any responsibility for changes to the environment
 * of the product that deviate from the TCD and the agreed upon documents 
 * as well as all applications not released by Bosch
  *
 * The resale and/or use of products are at the purchaser’s own risk and 
 * responsibility. The examination and testing of the SMI130 
 * is the sole responsibility of the purchaser.
 *
 * The purchaser shall indemnify Bosch from all third party claims 
 * arising from any product use not covered by the parameters of 
 * this product data sheet or not approved by Bosch and reimburse Bosch 
 * for all costs and damages in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products,
 * particularly with regard to product safety, and inform Bosch without delay
 * of all security relevant incidents.
 *
 * Application Examples and Hints
 *
 * With respect to any application examples, advice, normal values
 * and/or any information regarding the application of the device,
 * Bosch hereby disclaims any and all warranties and liabilities of any kind,
 * including without limitation warranties of
 * non-infringement of intellectual property rights or copyrights
 * of any third party.
 * The information given in this document shall in no event be regarded 
 * as a guarantee of conditions or characteristics. They are provided
 * for illustrative purposes only and no evaluation regarding infringement
 * of intellectual property rights or copyrights or regarding functionality,
 * performance or error has been made.
 *
 * @file         sensord_pltf.c
 * @date         "Thu Feb 4 13:36:50 2016 +0800"
 * @commit       "7020e4d"
 *
 * @modification date         "Thu May 3 12:23:56 2018 +0100"
 *
 * @brief
 *
 * @detail
 *
 */
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include <errno.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include "sensord_def.h"
#include "sensord_cfg.h"
#include "sensord_pltf.h"

#define SENSORD_TRACE_FILE (PATH_DIR_SENSOR_STORAGE "/sensord.log")
#define DATA_IN_FILE (PATH_DIR_SENSOR_STORAGE "/data_in.log")
#define BSX_DATA_LOG (PATH_DIR_SENSOR_STORAGE "/bsx_datalog.log")

static FILE *g_fp_trace = NULL;
static FILE *g_dlog_input = NULL;
static FILE *g_bsx_dlog = NULL;

static inline void storage_init()
{
    char *path = NULL;
    int ret = 0;
    struct stat st;

    path = (char *) (PATH_DIR_SENSOR_STORAGE);

    ret = stat(path, &st);
    if (0 == ret)
    {
        if (S_IFDIR == (st.st_mode & S_IFMT))
        {
            /*already exist*/
            ret = chmod(path, 0766);
            if (ret)
            {
                printf("error chmod on %s", path);
            }
        }

        return;
    }

    ret = mkdir(path, 0766);
    if (ret)
    {
        printf("error creating storage dir\n");
    }
    chmod(path, 0766); //notice the "umask" could mask some privilege when mkdir

    return;
}

void sensord_trace_init()
{
    g_fp_trace = fopen(SENSORD_TRACE_FILE, "w");
    if(NULL == g_fp_trace)
    {
        printf("sensord_trace_init: fail to open log file %s! \n", SENSORD_TRACE_FILE);
        g_fp_trace = stdout;
        return;
    }

    chmod(SENSORD_TRACE_FILE, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH);

    return;
}

int64_t sensord_get_tmstmp_ns(void)
{

    int64_t ap_time;
    struct timespec ts;

    clock_gettime(CLOCK_BOOTTIME, &ts);
    ap_time = (int64_t) ts.tv_sec * 1000000000 + ts.tv_nsec;

    return ap_time;

}

void trace_log(uint32_t level, const char *fmt, ...)
{
    int ret = 0;
    va_list ap;
#if !defined(PLTF_LINUX_ENABLED)
    char buffer[256] = { 0 };
#endif

    if (0 == trace_to_logcat)
    {
        if (0 == (trace_level & level))
        {
            return;
        }

        va_start(ap, fmt);
        ret = vfprintf(g_fp_trace, fmt, ap);
        va_end(ap);

        // otherwise, data is buffered rather than be wrote to file
        // therefore when stopped by signal, NO data left in file!
        fflush(g_fp_trace);

        if (ret < 0)
        {
            printf("trace_log: fprintf(g_fp_trace, fmt, ap)  fail!!\n");
        }
    }
    else
    {

#if !defined(PLTF_LINUX_ENABLED)
        /**
         * here use android api
         * Let it use Android trace level.
         */
#include<android/log.h>
#define BST_LOG_TAG    "sensord"

        va_start(ap, fmt);
        vsnprintf(buffer, sizeof(buffer) - 1, fmt, ap);
        va_end(ap);

        switch (level)
        {
            case LOG_LEVEL_N:
                __android_log_print(ANDROID_LOG_FATAL, BST_LOG_TAG, "%s", buffer);
                break;
            case LOG_LEVEL_E:
                __android_log_print(ANDROID_LOG_ERROR, BST_LOG_TAG, "%s", buffer);
                break;
            case LOG_LEVEL_W:
                __android_log_print(ANDROID_LOG_WARN, BST_LOG_TAG, "%s", buffer);
                break;
            case LOG_LEVEL_I:
                __android_log_print(ANDROID_LOG_INFO, BST_LOG_TAG, "%s", buffer);
                break;
            case LOG_LEVEL_D:
                __android_log_print(ANDROID_LOG_DEBUG, BST_LOG_TAG, "%s", buffer);
                break;
            case LOG_LEVEL_LADON:
                __android_log_print(ANDROID_LOG_WARN, BST_LOG_TAG, "%s", buffer);
                break;
            default:
                break;
        }

#else

        if(0 == (trace_level & level))
        {
            return;
        }

        va_start(ap, fmt);
        vprintf(fmt, ap);
        va_end(ap);

#endif
    }

    return;
}

static void generic_data_log(const char*dest_path, FILE **p_dest_fp, char *info_str)
{
    if (NULL == (*p_dest_fp))
    {
        (*p_dest_fp) = fopen(dest_path, "w");
        if (NULL == (*p_dest_fp))
        {
            printf("fail to open file %s! \n", dest_path);
            return;
        }

        chmod(dest_path, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
    }

    fprintf((*p_dest_fp), "%s", info_str);

    // otherwise, data is buffered rather than be wrote to file
    // therefore when stopped by signal, NO data left in file!
    fflush((*p_dest_fp));

}

void data_log_algo_input(char *info_str)
{
    generic_data_log(DATA_IN_FILE, &g_dlog_input, info_str);
}

void bsx_datalog_algo(char *info_str)
{
    generic_data_log(BSX_DATA_LOG, &g_bsx_dlog, info_str);
}


void sensord_pltf_init(void)
{
    storage_init();

    sensord_trace_init();

    return;
}

void sensord_pltf_clearup(void)
{
    fclose(g_fp_trace);

    if (g_dlog_input)
    {
        fclose(g_dlog_input);
    }
    if(g_bsx_dlog)
    {
        fclose(g_bsx_dlog);
    }
    return;
}

