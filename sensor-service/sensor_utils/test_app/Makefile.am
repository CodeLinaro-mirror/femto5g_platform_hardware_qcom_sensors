AM_CFLAGS = \
    -DDEBUG \
    -I./ \
    $(SENSORUTILLIB_CFLAGS) \
    -std=c++11

ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -std=c++11

requiredlibs = \
	       $(SENSORUTILLIB_LIBS)

h_sources =

c_sources = \
	    main.cpp

sensor_util_lib_testapp_SOURCES = \
    $(c_sources) $(h_sources)

######################
# Build sensor_util_lib_testapp
######################

if USE_GLIB
sensor_util_lib_testapp_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@
sensor_util_lib_testapp_LDFLAGS = -lstdc++ -g -Wl,-z,defs -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
sensor_util_lib_testapp_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@
else
sensor_util_lib_testapp_CFLAGS = $(AM_CFLAGS)
sensor_util_lib_testapp_LDFLAGS = -lstdc++ -Wl,-z,defs -lpthread -shared -version-info 1:0:0
sensor_util_lib_testapp_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS)
endif

sensor_util_lib_testapp_LDADD = $(requiredlibs) -lstdc++ -lc -ldl -llog

bin_PROGRAMS = sensor_util_lib_testapp
