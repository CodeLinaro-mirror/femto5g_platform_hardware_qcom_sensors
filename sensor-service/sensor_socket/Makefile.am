AM_CFLAGS = \
    -DDEBUG \
    -I \
    -fno-permissive

ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -std=c++14

h_sources = \
	    ./SensorIpc.h \
	    ./SensorThread.h \
	    ./SensorLog.h

c_sources = \
	    ./SensorIpc.cpp \
	    ./SensorThread.cpp \
	    ./SensorLog.cpp

libsensor_socket_la_SOURCES = \
	$(c_sources) $(h_sources)

######################
# Build sensor_socket
######################

if USE_GLIB
libsensor_socket_la_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@
libsensor_socket_la_LDFLAGS = -lstdc++ -g -Wl,-z,defs -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
libsensor_socket_la_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@
else
libsensor_socket_la_CFLAGS = $(AM_CFLAGS)
libsensor_socket_la_LDFLAGS = -lstdc++ -Wl,-z,defs -lpthread -shared -version-info 1:0:0
libsensor_socket_la_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS)
endif

libsensor_socket_la_LIBADD = $(requiredlibs) -lstdc++ -ldl -llog

#Create and Install libraries
library_include_HEADERS = $(h_sources)

lib_LTLIBRARIES = libsensor_socket.la

library_includedir = $(pkgincludedir)
pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = sensor-socket.pc
EXTRA_DIST = $(pkgconfig_DATA)
