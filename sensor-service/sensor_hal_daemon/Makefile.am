AM_CFLAGS = \
    -DDEBUG \
    -I./ \
    $(SENSORS_CFLAGS)\
    $(SENSORSOCKET_CFLAGS)\
    -std=c++11

requiredlibs = \
     $(SENSORSOCKET_LIBS)

h_sources = \
    SensorApiService.h \
    SensorApiMsg.h\
    SensorHalDaemonClientHandler.h \
    SensorHalDaemonIPCReceiver.h \
    SensorHalDaemonIPCSender.h \
    SensorApiUtils.h \
    SensorList.h

c_sources = \
    main.cpp \
    SensorApiService.cpp \
    SensorApiBuffer.cpp \
    SensorApiMLC.cpp \
    SensorHalDaemonClientHandler.cpp \
    SensorHalDaemonIPCReceiver.cpp \
    SensorHalDaemonIPCSender.cpp \
    SensorApiUtils.cpp

if POWERMANAGER_ENABLED
AM_CFLAGS += $(PM_CFLAGS) -DPOWERMANAGER_ENABLED
c_sources += PowerEvtHandler.cpp
requiredlibs += $(PM_LIBS)
endif

sensor_hal_daemon_SOURCES = \
    $(c_sources) $(h_sources)

######################
# Build sensor_hal_daemon
######################

if USE_GLIB
sensor_hal_daemon_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@ -fpermissive
sensor_hal_daemon_LDFLAGS = -lstdc++ -g -Wl,-z,defs -lpthread @GLIB_LIBS@ -shared
sensor_hal_daemon_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@
else
sensor_hal_daemon_CFLAGS = $(AM_CFLAGS) -fpermissive
sensor_hal_daemon_LDFLAGS = -Wl,-z,defs -lpthread -shared
sensor_hal_daemon_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS)
endif

sensor_hal_daemon_LDADD = $(requiredlibs) -lcutils -ldl -llog

bin_PROGRAMS = sensor_hal_daemon

library_include_HEADERS = $(h_sources)
library_includedir = $(pkgincludedir)

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = sensor-hal-daemon.pc
EXTRA_DIST = $(pkgconfig_DATA)
