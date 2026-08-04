idf_component_register(
    SRCS
        "twai_network_example_master_main.c"
        "robstride_control.c"
        "ble_console.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES bt nvs_flash driver
)
