# Makefile.am - Automake script for gps garden shippable test app
#

ACLOCAL_AMFLAGS = -I m4

SUBDIRS = iam20680
SUBDIRS += st/
SUBDIRS += smi230/

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = libsensors.pc
EXTRA_DIST = $(pkgconfig_DATA)
