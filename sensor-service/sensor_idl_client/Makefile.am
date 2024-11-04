AM_CFLAGS = \
    -DDEBUG \
    $(COMMONAPI_CFLAGS)\
    $(COMMONAPI_SOMEIP_CFLAGS)\
    $(SENSORFIDL_CFLAGS)\
    -std=c++11 \
    -fpermissive

ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -std=c++11

requiredlibs = \
    $(COMMONAPI_LIBS) \
    $(COMMONAPI_SOMEIP_LIBS) \
    $(SENSORFIDL_LIBS)

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

sensor_idl_client_LDADD = $(requiredlibs) -lstdc++ -lc -ldl -lCommonAPI -lSensorInterface-someip -lgptp

bin_PROGRAMS = sensor_idl_client
