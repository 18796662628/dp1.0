#include "chassis.h"

#include "bsp_log.h"
#include "dji_motor.h"
#include "message_center.h"
#include "user_lib.h"

#include <math.h>

#define STEER_ECD_TO_DEG                 (360.0f / 8192.0f)
#define SEMI_STEER_SPEED_DEADBAND_M_S      0.01f
#define SEMI_STEER_PI                      3.14159265359f
#define DRIVE_SPEED_TO_MOTOR_APS \
    (SEMI_STEER_DRIVE_REDUCTION_RATIO * 360.0f \
        / (2.0f * SEMI_STEER_PI * SEMI_STEER_WHEEL_RADIUS_M))

typedef struct {
    float x_m;
    float y_m;
    float zero_ecd;
    DJIMotorInstance *steer_motor;
    DJIMotorInstance *drive_motor;
    float target_angle_deg;
    float drive_target_aps;
} SemiSteerModule_s;

static Subscriber_t *chassis_cmd_sub;
static Chassis_Ctrl_Cmd_s chassis_cmd;
static SemiSteerModule_s front_left_module;
static SemiSteerModule_s rear_right_module;

volatile SemiSteer_Chassis_Debug_s semi_steer_chassis_debug;

static void StopAllMotors(void)
{
    DJIMotorStop(front_left_module.steer_motor);
    DJIMotorStop(front_left_module.drive_motor);
    DJIMotorStop(rear_right_module.steer_motor);
    DJIMotorStop(rear_right_module.drive_motor);
}

static uint8_t MotorsOnline(void)
{
    return DaemonIsOnline(front_left_module.steer_motor->daemon)
        && DaemonIsOnline(front_left_module.drive_motor->daemon)
        && DaemonIsOnline(rear_right_module.steer_motor->daemon)
        && DaemonIsOnline(rear_right_module.drive_motor->daemon);
}

static void ConfigureModule(SemiSteerModule_s *module, float x_m, float y_m,
    float zero_ecd, uint8_t steer_id, uint8_t drive_id)
{
    Motor_Init_Config_s drive_config = {
        .can_init_config = {
            .can_handle = &SEMI_STEER_CAN,
            .tx_id = drive_id,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 1.6f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .IntegralLimit = 3000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit
                    | PID_Derivative_On_Measurement,
                .MaxOut = 15000.0f,
            },
            .pid_struct_type = Cascade_PID,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508,
    };
    Motor_Init_Config_s steer_config = {
        .can_init_config = {
            .can_handle = &SEMI_STEER_CAN,
            .tx_id = steer_id,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 60.0f,
                .Ki = 7.0f,
                .Kd = 0.0f,
                .DeadBand = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                .IntegralLimit = 20.0f,
                .MaxOut = 10000.0f,
            },
            .speed_PID = {
                .Kp = 18.0f,
                .Ki = 8.0f,
                .Kd = 0.01f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                .IntegralLimit = 3000.0f,
                .MaxOut = 20000.0f,
            },
            .pid_struct_type = Cascade_PID,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };

    module->x_m = x_m;
    module->y_m = y_m;
    module->zero_ecd = zero_ecd;
    module->steer_motor = DJIMotorInit(&steer_config);
    module->drive_motor = DJIMotorInit(&drive_config);
}

/* Calculate one powered steering module. x points forward, y points left. */
static void CalculateModule(SemiSteerModule_s *module, float vx_m_s,
    float vy_m_s, float wz_rad_s)
{
    float module_vx = vx_m_s - wz_rad_s * module->y_m;
    float module_vy = vy_m_s + wz_rad_s * module->x_m;
    float target_angle_deg;
    float current_angle_deg;
    float steering_error_deg;
    float speed_m_s;

    speed_m_s = sqrtf(module_vx * module_vx + module_vy * module_vy);
    if (speed_m_s < SEMI_STEER_SPEED_DEADBAND_M_S) {
        module->target_angle_deg = 0.0f;
        module->drive_target_aps = 0.0f;
        return;
    }

    target_angle_deg = atan2f(module_vy, module_vx) * 180.0f / SEMI_STEER_PI;
    current_angle_deg = ((float)module->steer_motor->measure.ecd
        - module->zero_ecd) * STEER_ECD_TO_DEG;
    steering_error_deg = theta_format(target_angle_deg - current_angle_deg);

    /* Turn the shorter way. Reversing drive speed preserves the same wheel
     * velocity vector when the steering target is shifted by 180 degrees. */
    if (fabsf(steering_error_deg) > 90.0f) {
        steering_error_deg += steering_error_deg > 0.0f ? -180.0f : 180.0f;
    }

    module->target_angle_deg = current_angle_deg + steering_error_deg;
    module->drive_target_aps = cosf((target_angle_deg - current_angle_deg)
        * SEMI_STEER_PI / 180.0f) * speed_m_s * DRIVE_SPEED_TO_MOTOR_APS;

    /* total_angle avoids a discontinuity when the GM6020 encoder wraps. */
    DJIMotorSetRef(module->steer_motor,
        module->steer_motor->measure.total_angle + steering_error_deg);
    DJIMotorSetRef(module->drive_motor, module->drive_target_aps);
}

void ChassisInit(void)
{
    const float half_base = SEMI_STEER_WHEEL_BASE_M * 0.5f;
    const float half_track = SEMI_STEER_TRACK_WIDTH_M * 0.5f;

    chassis_cmd_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));

    ConfigureModule(&front_left_module, +half_base, +half_track,
        SEMI_STEER_FRONT_LEFT_ZERO_ECD, SEMI_STEER_FRONT_LEFT_STEER_ID,
        SEMI_STEER_FRONT_LEFT_DRIVE_ID);
    ConfigureModule(&rear_right_module, -half_base, -half_track,
        SEMI_STEER_REAR_RIGHT_ZERO_ECD, SEMI_STEER_REAR_RIGHT_STEER_ID,
        SEMI_STEER_REAR_RIGHT_DRIVE_ID);

    StopAllMotors();
    LOGINFO("[chassis] two steer modules: CAN1 GM6020[1,2], M3508[3,4]");
}

void ChassisTask(void)
{
    uint8_t motors_online;

    SubGetMessage(chassis_cmd_sub, &chassis_cmd);
    motors_online = MotorsOnline();
    semi_steer_chassis_debug.motors_online = motors_online;

    if (!chassis_cmd.robot_enabled
        || chassis_cmd.chassis_mode != CHASSIS_MANUAL_DRIVE
        || !motors_online) {
        StopAllMotors();
        semi_steer_chassis_debug.command_vx_m_s = 0.0f;
        semi_steer_chassis_debug.command_vy_m_s = 0.0f;
        semi_steer_chassis_debug.command_wz_rad_s = 0.0f;
        semi_steer_chassis_debug.command_enabled = 0U;
        semi_steer_chassis_debug.front_left_drive_target_aps = 0.0f;
        semi_steer_chassis_debug.rear_right_drive_target_aps = 0.0f;
        return;
    }

    DJIMotorEnable(front_left_module.steer_motor);
    DJIMotorEnable(front_left_module.drive_motor);
    DJIMotorEnable(rear_right_module.steer_motor);
    DJIMotorEnable(rear_right_module.drive_motor);

    CalculateModule(&front_left_module, chassis_cmd.vx_m_s,
        chassis_cmd.vy_m_s, chassis_cmd.wz_rad_s);
    CalculateModule(&rear_right_module, chassis_cmd.vx_m_s,
        chassis_cmd.vy_m_s, chassis_cmd.wz_rad_s);

    semi_steer_chassis_debug.command_vx_m_s = chassis_cmd.vx_m_s;
    semi_steer_chassis_debug.command_vy_m_s = chassis_cmd.vy_m_s;
    semi_steer_chassis_debug.command_wz_rad_s = chassis_cmd.wz_rad_s;
    semi_steer_chassis_debug.command_enabled = 1U;
    semi_steer_chassis_debug.front_left_steer_target_deg =
        front_left_module.target_angle_deg;
    semi_steer_chassis_debug.rear_right_steer_target_deg =
        rear_right_module.target_angle_deg;
    semi_steer_chassis_debug.front_left_drive_target_aps =
        front_left_module.drive_target_aps;
    semi_steer_chassis_debug.rear_right_drive_target_aps =
        rear_right_module.drive_target_aps;
}
