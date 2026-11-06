#ifndef CHASSIS_H
#define CHASSIS_H

#include "robot_def.h"

typedef struct {
    float command_vx_m_s;
    float command_vy_m_s;
    float command_wz_rad_s;
    uint8_t command_enabled;
    float front_left_steer_target_deg;
    float rear_right_steer_target_deg;
    float front_left_drive_target_aps;
    float rear_right_drive_target_aps;
    uint8_t motors_online;
} SemiSteer_Chassis_Debug_s;

extern volatile SemiSteer_Chassis_Debug_s semi_steer_chassis_debug;

void ChassisInit(void);
void ChassisTask(void);

#endif
