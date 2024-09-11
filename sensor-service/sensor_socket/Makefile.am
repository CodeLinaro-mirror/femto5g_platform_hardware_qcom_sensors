AM_CFLAGS = \
    -DDEBUG \
    -I \
    $(QMIFW_CFLAGS) \
	 $(SENSORS_CFLAGS)\
    -fpermissive

if USE_QSOCKET
  AM_CFLAGS += -DUSE_QSOCKET
endif

ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -std=c++14

requiredlibs = \
	       $(QMIFW_LIBS)

h_sources = \
	    ./SensorIpc.h \
	    ./SensorQsocket.h \
	    ./SensorThread.h \
	    ./SensorLog.h \
	    ./SensorDiagLog.h

c_sources = \
	    ./SensorIpc.cpp \
	    ./SensorQsocket.cpp\
	    ./SensorThread.cpp \
	    ./SensorLog.cpp \
	    ./SensorDiagLog.cpp

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

libsensor_socket_la_LIBADD = $(requiredlibs) -lstdc++ -ldl -llog -ldiag

#Create and Install libraries
library_include_HEADERS = $(h_sources)

lib_LTLIBRARIES = libsensor_socket.la

library_includedir = $(pkgincludedir)
pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = sensor-socket.pc
EXTRA_DIST = $(pkgconfig_DATA)
