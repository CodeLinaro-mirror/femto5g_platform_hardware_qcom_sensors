# configure.ac -- Autoconf script for sensor-hal-daemon
#
# Process this file with autoconf to produce a configure script

# Requires autoconf tool later than 2.61
AC_PREREQ(2.61)
# Initialize the sensor-hal-daemon package version 1.0.0
AC_INIT([sensor-hal-daemon],1.0.0)
# Does not strictly follow GNU Coding standards
AM_INIT_AUTOMAKE([foreign subdir-objects])
# Disables auto rebuilding of configure, Makefile.ins
AM_MAINTAINER_MODE
# Verifies the --srcdir is correct by checking for the path
AC_CONFIG_SRCDIR([Makefile.am])
# defines some macros variable to be included by source
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

# Checks for programs.
AC_PROG_LIBTOOL
AC_PROG_CXX
AC_PROG_CC
AM_PROG_CC_C_O
AC_PROG_AWK
AC_PROG_CPP
AC_PROG_INSTALL
AC_PROG_LN_S
AC_PROG_MAKE_SET
PKG_PROG_PKG_CONFIG

# Checks for libraries.
PKG_CHECK_MODULES([SENSORS], [libsensors])
AC_SUBST([SENSORS_CFLAGS])
AC_SUBST([SENSORS_LIBS])

PKG_CHECK_MODULES([SENSORSOCKET], [sensor-socket])
AC_SUBST([SENSORSOCKET_CFLAGS])
AC_SUBST([SENSORSOCKET_LIBS])

PKG_CHECK_MODULES([SENSORUTILLIB], [sensor-util-lib])
AC_SUBST([SENSORUTILLIB_CFLAGS])
AC_SUBST([SENSORUTILLIB_LIBS])

AC_ARG_WITH([pwrmgrlib],
    AC_HELP_STRING([--with-pwrmgrlib], [with power manager library]))

AM_CONDITIONAL(POWERMANAGER_ENABLED, test "x$with_pwrmgrlib" = "xyes")

AM_COND_IF(POWERMANAGER_ENABLED, [
    PKG_CHECK_MODULES([PM], [powermanager])
    AC_SUBST([PM_CFLAGS])
    AC_SUBST([PM_LIBS])
])

AC_ARG_WITH([sensor_head_type],
	        AC_HELP_STRING([--with-sensor_head_type], [with location client library]))

AM_CONDITIONAL(SENSOR_HEAD_TYPE_SUPPORT, test "x$with_sensor_head_type" = "xyes")

AM_COND_IF(SENSOR_HEAD_TYPE_SUPPORT, [
    PKG_CHECK_MODULES([LOCCLIENTAPI], [location-client-api])
    AC_SUBST([LOCCLIENTAPI_CFLAGS])
    AC_SUBST([LOCCLIENTAPI_LIBS])
])

AC_ARG_WITH([gptp],
    AC_HELP_STRING([--with-gptp], [with gptp support library]))

AM_CONDITIONAL(PTP_SUPPORTED, test "x$with_gptp" = "xyes")

AC_ARG_WITH([sensor_ivss],
    AC_HELP_STRING([--with-sensor_ivss], [with common api support library]))

AM_CONDITIONAL(SENSOR_IVSS_ENABLED, test "x$with_sensor_ivss" = "xyes")

AM_COND_IF(SENSOR_IVSS_ENABLED, [
   PKG_CHECK_MODULES([COMMONAPI], [CommonAPI])
   AC_SUBST([COMMONAPI_CFLAGS])
   AC_SUBST([COMMONAPI_LIBS])

   PKG_CHECK_MODULES([COMMONAPI_SOMEIP], [CommonAPI-SomeIP])
   AC_SUBST([COMMONAPI_SOMEIP_CFLAGS])
   AC_SUBST([COMMONAPI_SOMEIP_LIBS])

   PKG_CHECK_MODULES([SENSORFIDL], [sensor-fidl])
   AC_SUBST([SENSORFIDL_CFLAGS])
   AC_SUBST([SENSORFIDL_LIBS])
])

AC_ARG_WITH([glib],
      AC_HELP_STRING([--with-glib],
         [enable glib, building HLOS systems which use glib]))

if (test "x${with_glib}" = "xyes"); then
        AC_DEFINE(ENABLE_USEGLIB, 1, [Define if HLOS systems uses glib])
        PKG_CHECK_MODULES(GTHREAD, gthread-2.0 >= 2.16, dummy=yes,
                                AC_MSG_ERROR(GThread >= 2.16 is required))
        PKG_CHECK_MODULES(GLIB, glib-2.0 >= 2.16, dummy=yes,
                                AC_MSG_ERROR(GLib >= 2.16 is required))
        GLIB_CFLAGS="$GLIB_CFLAGS $GTHREAD_CFLAGS"
        GLIB_LIBS="$GLIB_LIBS $GTHREAD_LIBS"

        AC_SUBST(GLIB_CFLAGS)
        AC_SUBST(GLIB_LIBS)
fi

AM_CONDITIONAL(USE_GLIB, test "x${with_glib}" = "xyes")

AC_ARG_WITH([core_includes],
      AC_HELP_STRING([--with-core-includes=@<:@dir@:>@],
         [Specify the sensor of the core headers]),
      [core_incdir=$withval],
      with_core_includes=no)

if (test "x$with_core_includes" != "xno"); then
   CPPFLAGS="${CPPFLAGS} -I${core_incdir}"
fi

AC_ARG_WITH([dltlogging],
        AC_HELP_STRING([--with-dltlogging], [with DLT logging enabled]))

AM_CONDITIONAL(USE_DLT, test "x${with_dltlogging}" = "xyes")

if (test "x${with_dltlogging}" = "xyes"); then
        PKG_CHECK_MODULES([DLTLOG], [automotive-dlt])
        AC_SUBST([DLTLOG_CFLAGS])
        AC_SUBST([DLTLOG_LIBS])

        PKG_CHECK_MODULES([SENSORDLTLOGLIB], [sensor-dlt-log])
        AC_SUBST([SENSORDLTLOGLIB_CFLAGS])
        AC_SUBST([SENSORDLTLOGLIB_LIBS])
fi

AC_CONFIG_FILES([ \
        Makefile \
        sensor-hal-daemon.pc
        ])

AC_OUTPUT
