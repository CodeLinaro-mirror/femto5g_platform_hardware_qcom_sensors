# Makefile.am - Automake script for sensor lib
#
AM_CFLAGS = \
-I./sensors \
-I./sensors/tools \
-I./sensors/linux/test-sensors-hal/android_linux \
-D__func__=__PRETTY_FUNCTION__ \
-DON_TARGET_TEST \
-DBATCH_MODE_SUPPORT \
-D_HW_DONT_INCLUDE_CORE_

ACLOCAL_AMFLAGS = -I m4

libiam20680sensors_la_SOURCES = \
                             ./sensors/MPLSupport.cpp\
                             ./sensors/SensorsMain.cpp\
                             ./sensors/SensorBase.cpp \
                             ./sensors/CompassSensor.IIO.primary.cpp \
                             ./sensors/MPLSensor.cpp\
                             ./sensors/tools/ml_sysfs_helper.c\
                             ./sensors/tools/inv_sysfs_utils.c\
                             ./sensors/tools/inv_iio_buffer.c

library_includedir = $(pkgincludedir)

if USE_GLIB
libiam20680sensors_la_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@
libiam20680sensors_la_LDFLAGS = -lstdc++ -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
libiam20680sensors_la_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@
else
libiam20680sensors_la_CFLAGS = $(AM_CFLAGS)
libiam20680sensors_la_LDFLAGS = -lpthread -shared -version-info 1:0:0
libiam20680sensors_la_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS)
endif
libiam20680sensors_la_LIBADD = -lstdc++

#Create and Install libraries
lib_LTLIBRARIES = libiam20680sensors.la
