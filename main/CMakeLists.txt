idf_component_register(
    SRCS
        "app_main.c"
        "exo_config.c"
        "rs01_driver.c"
        "hip_joint.c"
        "exo_control_loop.c"
        "ble_console.c"
        "impedance_control.c"
    INCLUDE_DIRS "." "include"
    REQUIRES driver
    PRIV_REQUIRES bt nvs_flash
)
