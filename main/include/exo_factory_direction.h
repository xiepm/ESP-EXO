#ifndef EXO_FACTORY_DIRECTION_H
#define EXO_FACTORY_DIRECTION_H

#include "esp_err.h"
#include "exo_types.h"

/*
 * 仅保存出厂方向，不保存穿戴零位。NVS访问只发生在上电初始化或IDLE示教完成时，
 * 不进入100 Hz控制任务。记录绑定人体侧别、执行器槽位、CAN ID和格式版本，防止
 * 更改左右映射后误用旧方向；更换同ID同槽位电机仍需操作者重新标定。
 */
esp_err_t exo_factory_direction_storage_init(void);
esp_err_t exo_factory_direction_load(exo_hip_joint_id_t joint,
    exo_actuator_id_t actuator, exo_can_node_id_t can_id,
    exo_direction_t *direction);
esp_err_t exo_factory_direction_save(exo_hip_joint_id_t joint,
    exo_actuator_id_t actuator, exo_can_node_id_t can_id,
    exo_direction_t direction);

#endif
