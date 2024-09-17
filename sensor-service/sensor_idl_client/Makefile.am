AM_CFLAGS = \
    -DDEBUG \
    -I $(srcdir)/src-gen/core/ \
    -I $(srcdir)/src-gen/someip/ \
    $(COMMONAPI_CFLAGS)\
    $(COMMONAPI_SOMEIP_CFLAGS)\
    -std=c++11 \
    -fpermissive

ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -std=c++11

requiredlibs = \
    $(COMMONAPI_LIBS) \
    $(COMMONAPI_SOMEIP_LIBS)

if PTP_SUPPORTED
AM_CFLAGS += -I$(WORKSPACE)/external/open-avb/lib/libgptp \
	     -DPTP_SUPPORTED
requiredlibs += -lgptp
endif

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

#############################################################################################################
# Build sensor_idl_client
#############################################################################################################
h_sources = 

c_sources = \
    src/SensorInterfaceClient.cpp

sensor_idl_client_SOURCES = \
    $(c_sources) $(h_sources)

if USE_GLIB
sensor_idl_client_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@
sensor_idl_client_LDFLAGS = -lstdc++ -g -Wl,-z,defs -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
sensor_idl_client_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@
else
sensor_idl_client_CFLAGS = $(AM_CFLAGS)
sensor_idl_client_LDFLAGS = -lstdc++ -Wl,-z,defs -lpthread -shared -version-info 1:0:0
sensor_idl_client_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS)
endif

sensor_idl_client_LDADD = $(requiredlibs) -lstdc++ -lc -ldl -lCommonAPI -lSensorInterface-someip
sensor_idl_client_DEPENDENCIES = libSensorInterface-someip.la

bin_PROGRAMS = sensor_idl_client

#############################################################################################################
# define the files to be installed in /usr/etc/vsomeip
#############################################################################################################
sysconfdir = $(prefix)/etc/vsomeip/SensorInterface
sysconf_DATA = vsomeip-sensor_client.json commonapi4someip.ini
