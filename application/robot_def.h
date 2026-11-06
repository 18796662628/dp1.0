#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include <stdint.h>

/* This firmware targets one STM32F407 chassis controller. */
#define ONE_BOARD

/* Remote-control command scaling. These are software limits and can be
 * changed after the final drive motors, reduction ratios, and wheel diameter
 * are known. */
#define RC_STICK_FULL_SCALE             660.0f
#define CHASSIS_RC_DEADBAND              15.0f
#define CHASSIS_RC_MAX_SPEED_M_S          1.0f
#define CHASSIS_RC_MAX_YAW_RATE_RAD_S     2.0f

/* Temporary mechanical and CAN configuration copied from the sentry chassis.
 * Replace these values after the real semi-steer chassis is assembled. */
#define SEMI_STEER_WHEEL_BASE_M            0.344f
#define SEMI_STEER_TRACK_WIDTH_M           0.344f
#define SEMI_STEER_WHEEL_RADIUS_M          0.055f
#define SEMI_STEER_DRIVE_REDUCTION_RATIO  15.76f

/* Active steering modules: front-left and rear-right. */
#define SEMI_STEER_FRONT_LEFT_ZERO_ECD     4683.0f
#define SEMI_STEER_REAR_RIGHT_ZERO_ECD     2700.0f

#define SEMI_STEER_CAN                     hcan1
#define SEMI_STEER_FRONT_LEFT_STEER_ID     1U
#define SEMI_STEER_REAR_RIGHT_STEER_ID     2U
#define SEMI_STEER_FRONT_LEFT_DRIVE_ID     3U
#define SEMI_STEER_REAR_RIGHT_DRIVE_ID     4U

typedef enum {
    ROBOT_SAFETY_ESTOP = 0,
    ROBOT_SAFETY_DISARMED,
    ROBOT_SAFETY_ARMED,
} Robot_Safety_State_e;

typedef enum {
    CHASSIS_DISABLED = 0,
    CHASSIS_MANUAL_DRIVE,
} Chassis_Mode_e;

/* Commands use the chassis-body frame: +x forward, +y left, +z yaw CCW.
 * Velocity units are m/s and rad/s. */
typedef struct {
    float vx_m_s;
    float vy_m_s;
    float wz_rad_s;
    Chassis_Mode_e chassis_mode;
    uint8_t robot_enabled;
} Chassis_Ctrl_Cmd_s;

#endif
