# Makefile.am - Automake script for sensor lib
#
AM_CFLAGS = \
-I./src \
-I./linux \
-D__func__=__PRETTY_FUNCTION__ \
-DON_TARGET_TEST \
-DPLTF_LINUX_ENABLED

ACLOCAL_AMFLAGS = -I m4

libasm330sensors_la_SOURCES = \
                ./src/SensorHAL.cpp \
                ./src/CircularBuffer.cpp \
                ./src/FlushBufferStack.cpp \
                ./src/FlushRequested.cpp \
                ./src/ChangeODRTimestampStack.cpp \
                ./src/SensorBase.cpp \
                ./src/HWSensorBase.cpp \
                ./src/Accelerometer.cpp \
                ./src/Gyroscope.cpp \
                ./src/utils.cpp

if CONFIG_ST_HAL_HAS_SELFTEST_FUNCTIONS
libasm330sensors_la_SOURCES += ./src/SelfTest.cpp
endif # CONFIG_ST_HAL_HAS_SELFTEST_FUNCTIONS

library_includedir = $(pkgincludedir)

if USE_GLIB
libasm330sensors_la_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@
libasm330sensors_la_LDFLAGS = -lstdc++ -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
libasm330sensors_la_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@
else
libasm330sensors_la_CFLAGS = $(AM_CFLAGS)
libasm330sensors_la_LDFLAGS = -lpthread -shared -version-info 1:0:0
libasm330sensors_la_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS)
endif
libasm330sensors_la_LIBADD = -lstdc++

#Create and Install libraries
lib_LTLIBRARIES = libasm330sensors.la
