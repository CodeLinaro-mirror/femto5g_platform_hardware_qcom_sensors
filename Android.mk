# Copyright (C) 2008 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
## build Invensense (IAM) HAL module
#
LOCAL_PATH := $(call my-dir)

ifneq ($(TARGET_SIMULATOR),true)

include $(CLEAR_VARS)

ifeq ($(origin TARGET_BOARD_PLATFORM), undefined)
    LOCAL_MODULE := sensors.default
else
    LOCAL_MODULE := sensors.$(TARGET_BOARD_PLATFORM).iam_$(TARGET_BOARD_TYPE)
ifeq ($(PLATFORM_VERSION), 8.0.0)
    LOCAL_VENDOR_MODULE := true
    LOCAL_MODULE_PATH_32 := $(TARGET_OUT_VENDOR)/lib
else
    LOCAL_MODULE_RELATIVE_PATH := hw
endif
endif

LOCAL_MODULE_TAGS := eng

LOCAL_SRC_FILES :=\
                 ./iam20680/sensors/MPLSupport.cpp \
                 ./iam20680/sensors/SensorsMain.cpp\
                 ./iam20680/sensors/SensorBase.cpp\
                 ./iam20680/sensors/tools/ml_sensor_parsing.c\
                 ./iam20680/sensors/tools/ml_sysfs_helper.c\
                 ./iam20680/sensors/tools/inv_sysfs_utils.c\
                 ./iam20680/sensors/tools/inv_iio_buffer.c\
                 ./iam20680/sensors/CompassSensor.IIO.primary.cpp\
                 ./iam20680/sensors/MPLSensor.cpp


LOCAL_C_INCLUDES += $(LOCAL_PATH)/iam20680/sensors/tools

LOCAL_CFLAGS := -DBATCH_MODE_SUPPORT -DLOG_TAG=\"Invensense\" -pthread -Wno-error=date-time\

LOCAL_CPPFLAGS := -pthread\

LOCAL_LDLIBS += -lm -llog -lutils -lcutils
#LOCAL_SHARED_LIBRARIES := libm
#LOCAL_LDFLAGS :=
#LOCAL_LDFLAGS_arm :=
#LOCAL_LDFLAGS_arm64 :=
LOCAL_PRELINK_MODULE := false

include $(BUILD_SHARED_LIBRARY)

endif  # TARGET_SIMULATOR != true

#
## build BMI (BMI160) HAL module
#
ifneq ($(TARGET_SIMULATOR),true)

include $(CLEAR_VARS)

ifeq ($(origin TARGET_BOARD_PLATFORM), undefined)
    LOCAL_MODULE := sensors.default
else
    LOCAL_MODULE := sensors.$(TARGET_BOARD_PLATFORM).bmi_$(TARGET_BOARD_TYPE)
ifeq ($(PLATFORM_VERSION), 8.0.0)
    LOCAL_VENDOR_MODULE := true
    LOCAL_MODULE_PATH_32 := $(TARGET_OUT_VENDOR)/lib
else
    LOCAL_MODULE_RELATIVE_PATH := hw
endif
endif

LOCAL_MODULE_TAGS := eng

LOCAL_SRC_FILES :=\
                 ./bmi160/HAL_DataReady/hal/sensors.cpp \
                 ./bmi160/HAL_DataReady/hal/BstSensor.cpp \
                 ./bmi160/HAL_DataReady/sensord/sensord.cpp \
                 ./bmi160/HAL_DataReady/sensord/sensord_hwcntl_implement.cpp \
                 ./bmi160/HAL_DataReady/sensord/bstsimple_list.cpp \
                 ./bmi160/HAL_DataReady/sensord/sensord_hwcntl.cpp \
                 ./bmi160/HAL_DataReady/sensord/sensord_algo.cpp \
                 ./bmi160/HAL_DataReady/sensord/sensord_cfg.cpp \
                 ./bmi160/HAL_DataReady/sensord/sensord_pltf.c \
                 ./bmi160/HAL_DataReady/sensord/axis_remap.c \
                 ./bmi160/HAL_DataReady/sensord/util_misc.c

LOCAL_C_INCLUDES := $(LOCAL_PATH)/bmi160/HAL_DataReady/hal\
            $(LOCAL_PATH)/bmi160/HAL_DataReady/sensord/bsx/inc\
            $(LOCAL_PATH)/bmi160/HAL_DataReady/sensord/inc

LOCAL_CFLAGS := -pthread -Wno-error=date-time\

LOCAL_CPPFLAGS := -pthread\

LOCAL_LDLIBS += -lm -llog -lutils -lcutils
#LOCAL_SHARED_LIBRARIES := libm
#LOCAL_LDFLAGS :=
#LOCAL_LDFLAGS_arm :=
#LOCAL_LDFLAGS_arm64 :=
LOCAL_PRELINK_MODULE := false

include $(BUILD_SHARED_LIBRARY)

#Copy bst_hal_cfg.txt to system img
#include $(CLEAR_VARS)
#LOCAL_MODULE := bst_hal_cfg.txt
#LOCAL_MODULE_TAGS := optional
#LOCAL_MODULE_CLASS := ETC
#LOCAL_SRC_FILES := bst_hal_cfg.txt
#LOCAL_MODULE_PATH := $(TARGET_OUT_ETC)
#include $(BUILD_PREBUILT)

endif  # TARGET_SIMULATOR != true
