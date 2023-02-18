AM_CPPFLAGS = \
	      -DDEBUG \
	      -DPLTF_LINUX_ENABLED\
	      -DBATCH_MODE_SUPPORT\
	      $(SENSORS_CFLAGS)

sensor_test_c_sources = \
			sensor_test_app.cpp \
			sensor_buffer.cpp
sensor_test_SOURCES = $(sensor_test_c_sources)

if USE_GLIB
sensor_test_CFLAGS = -DUSE_GLIB $(AM_CFLAGS)  @GLIB_CFLAGS@
sensor_test_LDFLAGS = -lstdc++ -g -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
sensor_test_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@
else
sensor_test_CFLAGS = $(AM_CFLAGS)
sensor_test_LDFLAGS = -lpthread -shared -version-info 1:0:0
sensor_test_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS)
endif

sensor_test_LDADD = -ldl -llog

bin_PROGRAMS = sensor_test

