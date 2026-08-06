idf_component_register(
    SRCS
        "twai_network_example_master_main.c"
        "exo_config.c"
        "rs01_driver.c"
        "ble_console.c"
        "impedance_control.c"
    INCLUDE_DIRS "." "include"
    REQUIRES driver
    PRIV_REQUIRES bt nvs_flash
)
