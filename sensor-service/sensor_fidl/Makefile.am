AM_CFLAGS = \
    -DDEBUG \
    -I ./src-gen/core/ \
    -I ./src-gen/someip/ \
    $(COMMONAPI_CFLAGS)\
    $(COMMONAPI_SOMEIP_CFLAGS)\
    -std=c++11 \
    -fpermissive

ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -std=c++11

requiredlibs = \
    $(COMMONAPI_LIBS) \
    $(COMMONAPI_SOMEIP_LIBS)

####################################################################################################
# Build SOMEIP library
#####################################################################################################
libSensorInterface_someip_la_SOURCES = \
    src-gen/someip/v1/com/qualcomm/qti/sensor/SensorInterfaceSomeIPDeployment.cpp \
    src-gen/someip/v1/com/qualcomm/qti/sensor/SensorInterfaceTypesSomeIPDeployment.cpp \
    src-gen/someip/v1/com/qualcomm/qti/sensor/SensorInterfaceSomeIPProxy.cpp \
    src-gen/someip/v1/com/qualcomm/qti/sensor/SensorInterfaceSomeIPStubAdapter.cpp

library_include_HEADERS = \
    src-gen/core/v1/com/qualcomm/qti/sensor/SensorInterface.hpp \
    src-gen/core/v1/com/qualcomm/qti/sensor/SensorInterfaceProxy.hpp \
    src-gen/core/v1/com/qualcomm/qti/sensor/SensorInterfaceProxyBase.hpp \
    src-gen/core/v1/com/qualcomm/qti/sensor/SensorInterfaceStub.hpp \
    src-gen/core/v1/com/qualcomm/qti/sensor/SensorInterfaceStubDefault.hpp \
    src-gen/core/v1/com/qualcomm/qti/sensor/SensorInterfaceTypes.hpp \
    src-gen/someip/v1/com/qualcomm/qti/sensor/SensorInterfaceSomeIPDeployment.hpp \
    src-gen/someip/v1/com/qualcomm/qti/sensor/SensorInterfaceSomeIPProxy.hpp \
    src-gen/someip/v1/com/qualcomm/qti/sensor/SensorInterfaceSomeIPStubAdapter.hpp \
    src-gen/someip/v1/com/qualcomm/qti/sensor/SensorInterfaceTypesSomeIPDeployment.hpp

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

#Create and Install libraries
lib_LTLIBRARIES = libSensorInterface-someip.la

library_includedir = $(pkgincludedir)/v1/com/qualcomm/qti/sensor/
pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = sensor-fidl.pc
EXTRA_DIST = $(pkgconfig_DATA)

#####################################################################################
# define the files to be installed in /usr/etc/vsomeip
#####################################################################################
sysconfdir = $(prefix)/etc/vsomeip/SensorInterface
sysconf_DATA = vsomeip-sensor_service.json vsomeip-sensor_client.json commonapi4someip.ini
