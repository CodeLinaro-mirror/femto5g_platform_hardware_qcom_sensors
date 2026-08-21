AM_CFLAGS = \
    -DDEBUG \
    -I inc/ \
    -I src/ \
    -std=c++11\
    -fno-permissive

ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -std=c++11

SENSORUTILS_CFLAGS =
SENSORUTILS_LIBS =

if USE_CLOG
SENSORUTILS_CFLAGS += -DUSE_CLOG $(CLOGLOG_CFLAGS)
SENSORUTILS_LIBS += $(CLOGLOG_LIBS)
endif

if USE_DLT
SENSORUTILS_CFLAGS += -DUSE_DLT $(DLTLOG_CFLAGS)
SENSORUTILS_LIBS += $(DLTLOG_LIBS)
endif

requiredlibs = $(SENSORUTILS_LIBS)

h_sources = \
	    SensorThread.h \
	    SensorLog.h \
	    SensorUtils.h \
	    SensorDiagLog.h

c_sources = \
	    SensorThread.cpp \
	    SensorLog.cpp \
	    SensorUtils.cpp \
        SensorDiagLog.cpp

libsensor_util_lib_la_SOURCES = \
    $(c_sources) $(h_sources)

libsensor_util_lib_la_h_sources = $(h_sources)

######################
# Build sensor_util_lib
######################

if USE_GLIB
libsensor_util_lib_la_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@ $(SENSORUTILS_CFLAGS)
libsensor_util_lib_la_LDFLAGS = -lstdc++ -g -Wl,-z,defs -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
libsensor_util_lib_la_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@ $(SENSORUTILS_CFLAGS)
else
libsensor_util_lib_la_CFLAGS = $(AM_CFLAGS) $(SENSORUTILS_CFLAGS)
libsensor_util_lib_la_LDFLAGS = -lstdc++ -Wl,-z,defs -lpthread -shared -version-info 1:0:0
libsensor_util_lib_la_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS) $(SENSORUTILS_CFLAGS)
endif

libsensor_util_lib_la_LIBADD = $(requiredlibs) -lstdc++ -ldl -llog -lm -ldiag

#Create and Install libraries
library_include_HEADERS = $(h_sources)
lib_LTLIBRARIES = libsensor_util_lib.la

library_includedir = $(pkgincludedir)
pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = sensor-util-lib.pc
EXTRA_DIST = $(pkgconfig_DATA)
