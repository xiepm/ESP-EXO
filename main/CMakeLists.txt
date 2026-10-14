# ESP-IDF 把 main/ 目录作为一个“组件”编译。这里列出的每个 .c 都会进入固件；
# 新增源文件后若忘记加入 SRCS，头文件即使能被编辑器找到也会在链接时缺少实现。
idf_component_register(
    SRCS
        # 启动入口与全局静态配置。
        "app_main.c"
        "exo_config.c"
        "exo_factory_direction.c"
        "exo_wear_zero.c"
        # CAN 电机驱动、人体髋关节抽象和唯一 100 Hz 调度器。
        "rs01_driver.c"
        "hip_joint.c"
        # 阶段 9：同周期传感快照、轻量连续相位估计和高层助力SHADOW框架。
        "exo_sensor_snapshot.c"
        "gait_estimator.c"
        "exo_phase_profile.c"
        "exo_assist.c"
        "exo_swing_pulse.c"
        "exo_command_composer.c"
        "exo_telemetry.c"
        "exo_control_loop.c"
        # 产品生命周期状态机以及手机 BLE 传输/协议解析。
        "exo_system.c"
        "exo_feedback_recovery.c"
        "exo_ble_protocol.c"
        "exo_ble.c"
        # 工程串口、纯参考生成器和阶段 8 通用控制链。
        "debug_console.c"
        "sine_reference.c"
        # 阶段 8 通用控制器通过唯一五元关节命令出口驱动两侧 RS01。
        "exo_joint_command.c"
        "exo_controller.c"
    # “.”供 main/ 内部头文件使用；include/ 存放跨模块公开接口。
    INCLUDE_DIRS "." "include"
    # driver是公开依赖；bt供BLE使用，nvs_flash供BLE与出厂方向配置使用。
    REQUIRES driver
    PRIV_REQUIRES bt nvs_flash
)
