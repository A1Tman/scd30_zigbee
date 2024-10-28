idf_component_register(
    SRCS 
        "main.c"
        "i2c_handler.c"
        "zigbee_handler.c"
        "scd30_driver.c"
    INCLUDE_DIRS "."
    REQUIRES 
        driver 
        nvs_flash 
)