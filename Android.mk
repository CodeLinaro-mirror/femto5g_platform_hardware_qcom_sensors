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
endif
LOCAL_VENDOR_MODULE := true
LOCAL_MODULE_RELATIVE_PATH := hw

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

LOCAL_SHARED_LIBRARIES := \
      libhardware \

LOCAL_C_INCLUDES += $(LOCAL_PATH)/iam20680/sensors/tools

LOCAL_CFLAGS := -DBATCH_MODE_SUPPORT -DLOG_TAG=\"Invensense\" -pthread -Wno-error=date-time\

LOCAL_CPPFLAGS := -pthread\

LOCAL_LDLIBS += -lm -llog -lutils -lcutils
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
endif
LOCAL_VENDOR_MODULE := true
LOCAL_MODULE_RELATIVE_PATH := hw

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

LOCAL_SHARED_LIBRARIES := \
      libhardware \

LOCAL_CFLAGS := -pthread -Wno-error=date-time\

LOCAL_CPPFLAGS := -pthread\

LOCAL_LDLIBS += -lm -llog -lutils -lcutils
LOCAL_PRELINK_MODULE := false

include $(BUILD_SHARED_LIBRARY)

endif  # TARGET_SIMULATOR != true


#
## build ASM (ASM330) HAL module
#
ifneq ($(TARGET_SIMULATOR),true)

include $(CLEAR_VARS)

ifeq ($(origin TARGET_BOARD_PLATFORM), undefined)
    LOCAL_MODULE := sensors.default
else
    LOCAL_MODULE := sensors.$(TARGET_BOARD_PLATFORM).asm_$(TARGET_BOARD_TYPE)
endif
LOCAL_VENDOR_MODULE := true
LOCAL_MODULE_RELATIVE_PATH := hw

LOCAL_SRC_FILES :=\
                  ./st/asm330lhh/src/SensorHAL.cpp \
                  ./st/asm330lhh/src/Accelerometer.cpp \
                  ./st/asm330lhh/src/FlushBufferStack.cpp \
                  ./st/asm330lhh/src/HWSensorBase.cpp \
                  ./st/asm330lhh/src/SensorBase.cpp \
                  ./st/asm330lhh/src/utils.cpp \
                  ./st/asm330lhh/src/CircularBuffer.cpp \
                  ./st/asm330lhh/src/FlushRequested.cpp \
                  ./st/asm330lhh/src/ChangeODRTimestampStack.cpp \
                  ./st/asm330lhh/src/Gyroscope.cpp\
		  ./st/asm330lhh/src/SensorAdditionalInfo.cpp

ifdef CONFIG_ST_HAL_HAS_SELFTEST_FUNCTIONS
LOCAL_SRC_FILES += ./st/asm330lhh/src/SelfTest.cpp
endif # CONFIG_ST_HAL_HAS_SELFTEST_FUNCTIONS
LOCAL_SHARED_LIBRARIES := \
      libhardware \

LOCAL_C_INCLUDES := $(LOCAL_PATH)/st/asm330lhh/src

LOCAL_CFLAGS := -pthread -Wno-error=date-time\

LOCAL_CPPFLAGS := -pthread\

LOCAL_LDLIBS += -lm -llog -lutils -lcutils
LOCAL_PRELINK_MODULE := false

include $(BUILD_SHARED_LIBRARY)
endif  # TARGET_SIMULATOR != true
