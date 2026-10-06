idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_cam esp_driver_i2c esp_cam_sensor esp_sccb_intf
)
