ENABLE_SENSOR_CONFIGS := false

ifeq ($(USE_SENSOR_HAL_VER),2.0)
    PRODUCT_PACKAGES += android.hardware.sensors@2.0-service
    PRODUCT_PACKAGES += android.hardware.sensors@2.0-impl
    PRODUCT_PACKAGES += android.hardware.sensors@2.0-service.rc
else
ifeq ($(ENABLE_AIDL_SENSOR),true)
ifeq ($(filter $(TARGET_BOARD_PLATFORM),msmnile),$(TARGET_BOARD_PLATFORM))
ifneq ($(ENABLE_HYP),true)
      PRODUCT_PACKAGES += android.hardware.sensors@aidl-service.stmicroelectronics
      ENABLE_SENSOR_CONFIGS := true
endif
endif
else
PRODUCT_PACKAGES += android.hardware.sensors-service.example
endif
PRODUCT_PACKAGES += android.hardware.sensors@1.0-service
PRODUCT_PACKAGES += android.hardware.sensors@1.0-impl
endif

# Copy sensor configuration files if ENABLE_SENSOR_CONFIGS is true
ifeq ($(ENABLE_SENSOR_CONFIGS),true)
    PRODUCT_COPY_FILES += \
        frameworks/native/data/etc/android.hardware.sensor.accelerometer.xml:vendor/etc/permissions/android.hardware.sensor.accelerometer.xml \
        frameworks/native/data/etc/android.hardware.sensor.ambient_temperature.xml:vendor/etc/permissions/android.hardware.sensor.ambient_temperature.xml \
        frameworks/native/data/etc/android.hardware.sensor.compass.xml:vendor/etc/permissions/android.hardware.sensor.compass.xml \
        frameworks/native/data/etc/android.hardware.sensor.gyroscope.xml:vendor/etc/permissions/android.hardware.sensor.gyroscope.xml \
        frameworks/native/data/etc/android.hardware.sensor.hifi_sensors.xml:vendor/etc/permissions/android.hardware.sensor.hifi_sensors.xml \
        frameworks/native/data/etc/android.hardware.sensor.relative_humidity.xml:vendor/etc/permissions/android.hardware.sensor.relative_humidity.xml \
        frameworks/native/data/etc/android.hardware.sensor.stepcounter.xml:vendor/etc/permissions/android.hardware.sensor.stepcounter.xml \
        frameworks/native/data/etc/android.hardware.sensor.stepdetector.xml:vendor/etc/permissions/android.hardware.sensor.stepdetector.xml
endif
