AM_CFLAGS = \
    -DDEBUG \
    -I inc/ \
    -I src/ \
    $(SENSORS_CFLAGS)\
    $(SENSORUTILLIB_CFLAGS)\
    $(SENSORSOCKET_CFLAGS)\
    -std=c++11\
    -fpermissive


ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -std=c++11

requiredlibs = \
     $(SENSORUTILLIB_LIBS) \
     $(SENSORSOCKET_LIBS)

if USE_DLT
AM_CFLAGS += -DUSE_DLT $(DLTLOG_CFLAGS) $(SENSORDLTLOGLIB_CFLAGS)
requiredlibs += $(DLTLOG_LIBS) $(SENSORDLTLOGLIB_LIBS)
endif

h_sources = \
    inc/SensorClientApi.h \
    SensorClientApiImpl.h

c_sources = \
    SensorClientApiImpl.cpp \
    SensorClientApi.cpp

libsensor_client_api_la_SOURCES = \
    $(c_sources) $(h_sources)

libsensor_client_api_la_h_sources = $(h_sources)

######################
# Build sensor_client_api
######################

if USE_GLIB
libsensor_client_api_la_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@
libsensor_client_api_la_LDFLAGS = -lstdc++ -g -Wl,-z,defs -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
libsensor_client_api_la_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@
else
libsensor_client_api_la_CFLAGS = $(AM_CFLAGS)
libsensor_client_api_la_LDFLAGS = -lstdc++ -Wl,-z,defs -lpthread -shared -version-info 1:0:0
libsensor_client_api_la_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS)
endif

if USE_EXTERNAL_AP
AM_CFLAGS += -DUSE_EXTERNAL_AP $(SENSORSOCKET_CFLAGS)
requiredlibs += $(SENSORSOCKET_LIBS)
endif

libsensor_client_api_la_LIBADD = $(requiredlibs) -lstdc++ -ldl -llog

#Create and Install libraries
library_include_HEADERS = $(h_sources)
lib_LTLIBRARIES = libsensor_client_api.la

library_includedir = $(pkgincludedir)
pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = sensor-client-api.pc
EXTRA_DIST = $(pkgconfig_DATA)
