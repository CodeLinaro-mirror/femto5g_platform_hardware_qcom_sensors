ENABLE_SENSOR_CONFIGS := false

ifeq ($(USE_SENSOR_HAL_VER),2.0)
PRODUCT_PACKAGES += android.hardware.sensors@2.0-service
PRODUCT_PACKAGES += android.hardware.sensors@2.0-impl
PRODUCT_PACKAGES += android.hardware.sensors@2.0-service.rc
else
ifeq ($(ENABLE_AIDL_SENSOR),true)
ifeq ($(filter $(TARGET_BOARD_PLATFORM), msmnile),$(TARGET_BOARD_PLATFORM))   #Hana metal -> msmnile_au,  msmnile_gvmq -> makena
ifneq ($(ENABLE_HYP), true)   #msmnile_gvmq -> makena
PRODUCT_PACKAGES += android.hardware.sensors@aidl-service.stmicroelectronics
ENABLE_SENSOR_CONFIGS := true
endif
else
PRODUCT_PACKAGES += android.hardware.sensors-service.example
endif
else
PRODUCT_PACKAGES += android.hardware.sensors@1.0-service
PRODUCT_PACKAGES += android.hardware.sensors@1.0-impl
endif # ENABLE_AIDL_SENSOR
endif #USE_SENSOR_HAL_VER
