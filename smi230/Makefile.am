# Makefile.am - Automake script for gps garden shippable test app
#
AM_CFLAGS = \
-I./hardware \
-I./sensord/inc \
-I./sensord/bsx/inc \
-I./hal \
-I./sensord \
-D__func__=__PRETTY_FUNCTION__ \
-DON_TARGET_TEST \
-DPLTF_LINUX_ENABLED

ACLOCAL_AMFLAGS = -I m4

libsmi230sensors_la_SOURCES = \
                              ./hal/sensors.cpp \
                              ./hal/BoschSensor.cpp \
                              ./sensord/sensord.cpp\
                              ./sensord/sensord_hwcntl_implement.cpp \
                              ./sensord/boschsimple_list.cpp \
                              ./sensord/sensord_hwcntl.cpp \
                              ./sensord/sensord_algo.cpp \
                              ./sensord/sensord_cfg.cpp \
                              ./sensord/sensord_pltf.c \
                              ./sensord/axis_remap.c \
                              ./sensord/util_misc.c

library_includedir = $(pkgincludedir)

if USE_GLIB
libsmi230sensors_la_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@
libsmi230sensors_la_LDFLAGS = -lstdc++ -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
libsmi230sensors_la_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@
else
libsmi230sensors_la_CFLAGS = $(AM_CFLAGS)
libsmi230sensors_la_LDFLAGS = -lpthread -shared -version-info 1:0:0
libsmi230sensors_la_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS)
endif
libsmi230sensors_la_LIBADD = -lstdc++ -llog

#Create and Install libraries
lib_LTLIBRARIES = libsmi230sensors.la

