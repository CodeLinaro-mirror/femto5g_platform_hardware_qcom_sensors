AM_CFLAGS = \
    -DDEBUG \
    -fpermissive

ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -std=c++14

h_sources = \
	    ./SensorDltLog.h

c_sources = \
	    ./SensorDltLog.cpp

libsensor_dlt_log_la_SOURCES = \
	$(c_sources) $(h_sources)

######################
# Build sensor_dlt_log
######################

if USE_GLIB
libsensor_dlt_log_la_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@ $(DLTLOG_CFLAGS)
libsensor_dlt_log_la_LDFLAGS = -lstdc++ -g -Wl,-z,defs -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
libsensor_dlt_log_la_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@ $(DLTLOG_CFLAGS)
else
libsensor_dlt_log_la_CFLAGS = $(AM_CFLAGS) $(DLTLOG_CFLAGS)
libsensor_dlt_log_la_LDFLAGS = -lstdc++ -Wl,-z,defs -lpthread -shared -version-info 1:0:0
libsensor_dlt_log_la_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS) $(DLTLOG_CFLAGS)
endif

libsensor_dlt_log_la_LIBADD = $(requiredlibs) -lstdc++ -ldl -llog $(DLTLOG_LIBS)

#Create and Install libraries
library_include_HEADERS = $(h_sources)

lib_LTLIBRARIES = libsensor_dlt_log.la

library_includedir = $(pkgincludedir)
pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = sensor-dlt-log.pc
EXTRA_DIST = $(pkgconfig_DATA)
