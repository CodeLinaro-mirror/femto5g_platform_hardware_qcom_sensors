# Makefile.am - Automake script for gps garden shippable test app
#

ACLOCAL_AMFLAGS = -I m4

SUBDIRS = bmi160
SUBDIRS += iam20680
SUBDIRS += st/asm330lhh/

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = libsensors.pc
EXTRA_DIST = $(pkgconfig_DATA)
