# Makefile.am - Automake script for sensor lib
#
AM_CFLAGS = \
-I./make \
-I./sensord/inc \
-I./sensord/bsx/inc \
-I./hal \
-I./sensord \
-D__func__=__PRETTY_FUNCTION__ \
-DON_TARGET_TEST \
-DPLTF_LINUX_ENABLED

ACLOCAL_AMFLAGS = -I m4

libbmi160sensors_la_SOURCES = \
                        hal/sensors.cpp \
                        hal/BstSensor.cpp \
                        sensord/sensord.cpp \
                        sensord/sensord_hwcntl_implement.cpp \
                        sensord/bstsimple_list.cpp \
                        sensord/sensord_hwcntl.cpp \
                        sensord/sensord_algo.cpp \
                        sensord/sensord_cfg.cpp \
                        sensord/sensord_pltf.c \
                        sensord/axis_remap.c \
                        sensord/util_misc.c

library_include_HEADERS = \
                        ./make/hardware.h \
                        ./make/sensors.h

library_includedir = $(pkgincludedir)

if USE_GLIB
libbmi160sensors_la_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@
libbmi160sensors_la_LDFLAGS = -lstdc++ -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
libbmi160sensors_la_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@
else
libbmi160sensors_la_CFLAGS = $(AM_CFLAGS)
libbmi160sensors_la_LDFLAGS = -lpthread -shared -version-info 1:0:0
libbmi160sensors_la_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS)
endif
libbmi160sensors_la_LIBADD = -lstdc++

#Create and Install libraries
lib_LTLIBRARIES = libbmi160sensors.la
