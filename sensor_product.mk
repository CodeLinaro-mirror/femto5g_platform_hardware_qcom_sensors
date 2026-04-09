ENABLE_SENSOR_CONFIGS := false
ifeq ($(USE_SENSOR_HAL_VER),2.0)
PRODUCT_PACKAGES += android.hardware.sensors@2.0-service
PRODUCT_PACKAGES += android.hardware.sensors@2.0-impl
PRODUCT_PACKAGES += android.hardware.sensors@2.0-service.rc
else
ifeq ($(ENABLE_AIDL_SENSOR),true)

ifeq ($(filter $(TARGET_BOARD_PLATFORM), gen4),$(TARGET_BOARD_PLATFORM)) #gen4_gvm -> lemans, monaco HQX. gen4_au -> lemans metal
ifneq ($(ENABLE_HYP), false)
ifneq ( ,$(filter _sdvcomm _cdccomm, $(TARGET_BOARD_DERIVATIVE_SUFFIX)))
ifneq ($(TARGET_SOMEIP_ENABLE), false)
PRODUCT_PACKAGES += android.hardware.sensors@aidl-service-qc
PRODUCT_PACKAGES += vsomeip-sensor_client.json
PRODUCT_PACKAGES += vsomeip-sensor_test_client.json
PRODUCT_PACKAGES += SensorInterfaceClient
PRODUCT_PACKAGES += hal_config
ENABLE_SENSOR_CONFIGS := true
endif#TARGET_SOMEIP_ENABLE
endif
endif

else
PRODUCT_PACKAGES += android.hardware.sensors-service.example
endif

else
PRODUCT_PACKAGES += android.hardware.sensors@1.0-service
PRODUCT_PACKAGES += android.hardware.sensors@1.0-impl
endif # ENABLE_AIDL_SENSOR
endif #USE_SENSOR_HAL_VER

# Copy sensors config file(s)
ifeq ($(ENABLE_SENSOR_CONFIGS), true)
PRODUCT_COPY_FILES += \
        frameworks/native/data/etc/android.hardware.sensor.accelerometer.xml:vendor/etc/permissions/android.hardware.sensor.accelerometer.xml \
        frameworks/native/data/etc/android.hardware.sensor.gyroscope.xml:vendor/etc/permissions/android.hardware.sensor.gyroscope.xml
endif
