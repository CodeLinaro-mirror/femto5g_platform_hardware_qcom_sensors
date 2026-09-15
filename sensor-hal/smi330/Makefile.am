
# Automake for SMI330 sensors library, matching project style (libsmi330sensors.la)

# Common C/C++ include paths and defines (similar to your other sensors)
AM_CPPFLAGS = \
  -I./core \
  -I./hwctl \
  -I./sensors \
  -I./hal \
  -D__func__=__PRETTY_FUNCTION__ \
  -DON_TARGET_TEST \
  -DPLTF_LINUX_ENABLED \
  -D$(MODULE_UPPER)_SCAN_TYPE=$(OPERATION_MODE) \
  -D$(MODULE_UPPER)_ACCEL_RANGE=$(ACCEL_RANGE) \
  -D$(MODULE_UPPER)_GYRO_RANGE=$(GYRO_RANGE)

# C++ compile options to match CMake
AM_CXXFLAGS = -std=c++17 -Wall -Wno-psabi

# Library sources (from your CMake add_library list)
libsmi330sensors_la_SOURCES = \
  core/SensorCore.cpp \
  core/SensorThread.cpp \
  core/Accelerometer.cpp \
  core/Gyroscope.cpp \
  hwctl/FileHandler.cpp \
  sensors/SensorList.cpp \
  sensors/SMI230.cpp \
  sensors/SMI240.cpp \
  sensors/SMI330.cpp \
  hal/Sensor.cpp \
  hal/Sensors.cpp \
  hal/SensorsEventCallback.cpp

# Provide a pkg-includedir symbol like your other subdirs (even if unused)
library_includedir = $(pkgincludedir)

# GLib conditional, following your exact pattern (libs in LDFLAGS as in examples)
if USE_GLIB
libsmi330sensors_la_CFLAGS   = -DUSE_GLIB @GLIB_CFLAGS@
libsmi330sensors_la_CPPFLAGS = -DUSE_GLIB $(AM_CPPFLAGS) @GLIB_CFLAGS@
libsmi330sensors_la_LDFLAGS  = -lstdc++ -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
else
libsmi330sensors_la_CFLAGS   =
libsmi330sensors_la_CPPFLAGS = $(AM_CPPFLAGS)
libsmi330sensors_la_LDFLAGS  = -lpthread -shared -version-info 1:0:0
endif

# Extra libraries (matching your pattern)
libsmi330sensors_la_LIBADD = -lstdc++ -llog

# Create and install library (consistent with other subdirs)
lib_LTLIBRARIES = libsmi330sensors.la

# Optional: keep your original CMake file packaged
EXTRA_DIST = CMakeLists.txt

