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
 *
 * Changes from Qualcomm Innovation Center are provided under the following license:
 *
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the
 * disclaimer below) provided that the following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 *
 *   Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 * GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 * HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
 * @file         sensord_cfg.cpp
 * @date         "Tue Jan 26 11:14:07 2016 +0800"
 * @commit       "c6abe62"
 *
 * @modification date         "Thu May 3 12:23:56 2018 +0100"
 *
 * @brief
 *
 * @detail
 *
 */

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include "BoschSensor.h"
#include "sensord_cfg.h"
#include "sensord_pltf.h"
#include "sensord_hwcntl.h"
#include "util_misc.h"

int g_place_a = 0;
int g_place_m = 0;
int g_place_g = 0;
int solution_type = SOLUTION_IMU;
int accl_chip = ACC_CHIP_BMA2x2;
int gyro_chip = GYR_CHIP_BMG160;
int magn_chip = MAG_CHIP_BMM150;
int accl_range = ACC_CHIP_RANGCONF_2G;
int algo_pass = 1;
int amsh_intr_pin = 0;
int amsh_calibration = 0;
int data_log = 0;
int bsx_datalog = 0;
int trace_level = 0x1C; //NOTE + ERR + WARN
int trace_to_logcat = 0;
long long unsigned int sensors_mask = 0;


void BoschSensor::sensord_cfg_init()
{

    return;
}

