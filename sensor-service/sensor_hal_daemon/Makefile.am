AM_CFLAGS = \
    -DDEBUG \
    -I $(srcdir)/src/ \
    $(SENSORS_CFLAGS)\
    $(SENSORSOCKET_CFLAGS)\
    $(SENSORUTILLIB_CFLAGS) \
    -std=c++11 \
    -fpermissive

ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -std=c++11

requiredlibs = \
     $(SENSORSOCKET_LIBS) \
     $(SENSORUTILLIB_LIBS)

if SENSOR_IVSS_ENABLED
AM_CFLAGS += $(COMMONAPI_SOMEIP_CFLAGS) $(COMMONAPI_CFLAGS) \
	     -I $(srcdir)/src-gen/core/ \
             -I $(srcdir)/src-gen/someip/ \
	     -DSENSOR_IVSS_ENABLED
requiredlibs += $(COMMONAPI_LIBS) $(COMMONAPI_SOMEIP_LIBS) -lCommonAPI -lc
endif

if SENSOR_IVSS_ENABLED
####################################################################################################
# Build SOMEIP library
#####################################################################################################
if USE_GLIB
libSensorInterface_someip_la_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@
libSensorInterface_someip_la_LDFLAGS = -lstdc++ -g -Wl,-z,defs -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
libSensorInterface_someip_la_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@
else
libSensorInterface_someip_la_CFLAGS = $(AM_CFLAGS)
libSensorInterface_someip_la_LDFLAGS = -lstdc++ -Wl,-z,defs -lpthread -shared -version-info 1:0:0
libSensorInterface_someip_la_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS)
endif
libSensorInterface_someip_la_LIBADD = $(requiredlibs) -lstdc++ -ldl -lCommonAPI -lCommonAPI-SomeIP
libSensorInterface_someip_la_SOURCES = \
    src-gen/someip/v1/com/qualcomm/qti/sensor/SensorInterfaceSomeIPDeployment.cpp \
    src-gen/someip/v1/com/qualcomm/qti/sensor/SensorInterfaceTypesSomeIPDeployment.cpp \
    src-gen/someip/v1/com/qualcomm/qti/sensor/SensorInterfaceSomeIPProxy.cpp \
    src-gen/someip/v1/com/qualcomm/qti/sensor/SensorInterfaceSomeIPStubAdapter.cpp

lib_LTLIBRARIES = libSensorInterface-someip.la
endif

##########################################################################
# Build sensor_hal_daemon
##########################################################################

h_sources = \
    src/SensorApiService.h \
    src/SensorApiMsg.h\
    src/SensorHalDaemonClientHandler.h \
    src/SensorHalDaemonIPCReceiver.h \
    src/SensorHalDaemonIPCSender.h \
    src/SensorApiUtils.h \
    src/SensorList.h \
    src/SensorInterfaceStubImpl.hpp

c_sources = \
    src/main.cpp \
    src/SensorApiService.cpp \
    src/SensorApiBuffer.cpp \
    src/SensorApiMLC.cpp \
    src/SensorHalDaemonClientHandler.cpp \
    src/SensorHalDaemonIPCReceiver.cpp \
    src/SensorHalDaemonIPCSender.cpp \
    src/SensorApiUtils.cpp \
    src/SensorInterfaceStubImpl.cpp

if POWERMANAGER_ENABLED
AM_CFLAGS += $(PM_CFLAGS) -DPOWERMANAGER_ENABLED
c_sources += src/PowerEvtHandler.cpp
requiredlibs += $(PM_LIBS)
endif

if SENSOR_HEAD_TYPE_SUPPORT
AM_CFLAGS += $(LOCCLIENTAPI_CFLAGS) -DSENSOR_HEAD_TYPE_SUPPORT
requiredlibs += $(LOCCLIENTAPI_LIBS)
endif

if PTP_SUPPORTED
AM_CFLAGS += -I$(WORKSPACE)/external/open-avb/lib/libgptp \
	     -DPTP_SUPPORTED
requiredlibs += -lgptp
endif

sensor_hal_daemon_SOURCES = \
    $(c_sources) $(h_sources)

sensor_hal_daemon_h_sources = $(h_sources)

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
if SENSOR_IVSS_ENABLED
sensor_hal_daemon_DEPENDENCIES = libSensorInterface-someip.la
endif

bin_PROGRAMS = sensor_hal_daemon

library_include_HEADERS = $(h_sources)
library_includedir = $(pkgincludedir)

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = sensor-hal-daemon.pc
EXTRA_DIST = $(pkgconfig_DATA)

#if SENSOR_IVSS_ENABLED
#############################################################################################################
# define the files to be installed in /usr/etc/vsomeip
#############################################################################################################
sysconfdir = $(prefix)/etc/vsomeip/SensorInterface
sysconf_DATA = vsomeip-sensor_service.json commonapi4someip.ini
#endif
