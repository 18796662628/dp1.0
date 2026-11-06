#include "robot_cmd.h"

#include "bsp_log.h"
#include "message_center.h"
#include "remote_control.h"

#include <math.h>

#define ARM_HOLD_TICKS 500U

static Publisher_t *chassis_cmd_pub;
static RC_ctrl_t *remote_control;
static Chassis_Ctrl_Cmd_s chassis_cmd;
static Robot_Safety_State_e safety_state = ROBOT_SAFETY_ESTOP;
static uint16_t arm_hold_ticks;
static uint8_t remote_frame_seen;

static uint8_t IsValidSwitchState(uint8_t switch_state)
{
    return switch_is_up(switch_state)
        || switch_is_mid(switch_state)
        || switch_is_down(switch_state);
}

static uint8_t IsEmergencyStopRequested(void)
{
    if (!remote_frame_seen || !RemoteControlIsOnline()) {
        return 1U;
    }

    if (!IsValidSwitchState(remote_control[TEMP].rc.switch_left)
        || !IsValidSwitchState(remote_control[TEMP].rc.switch_right)) {
        return 1U;
    }

    return switch_is_down(remote_control[TEMP].rc.switch_left)
        && switch_is_down(remote_control[TEMP].rc.switch_right);
}

static uint8_t AreSwitchesCentered(void)
{
    return switch_is_mid(remote_control[TEMP].rc.switch_left)
        && switch_is_mid(remote_control[TEMP].rc.switch_right);
}

static void SetSafetyState(Robot_Safety_State_e next_state)
{
    if (safety_state == next_state) {
        return;
    }

    safety_state = next_state;
    if (next_state == ROBOT_SAFETY_ESTOP) {
        LOGWARNING("[safety] emergency stop");
    } else if (next_state == ROBOT_SAFETY_DISARMED) {
        LOGINFO("[safety] disarmed; center both switches to arm");
    } else {
        LOGINFO("[safety] armed");
    }
}

static void UpdateSafetyState(void)
{
    if (IsEmergencyStopRequested()) {
        arm_hold_ticks = 0U;
        SetSafetyState(ROBOT_SAFETY_ESTOP);
        return;
    }

    if (safety_state == ROBOT_SAFETY_ESTOP) {
        arm_hold_ticks = 0U;
        SetSafetyState(ROBOT_SAFETY_DISARMED);
    }

    if (safety_state != ROBOT_SAFETY_DISARMED || !AreSwitchesCentered()) {
        if (!AreSwitchesCentered()) {
            arm_hold_ticks = 0U;
        }
        return;
    }

    if (arm_hold_ticks < ARM_HOLD_TICKS) {
        arm_hold_ticks++;
    }
    if (arm_hold_ticks == ARM_HOLD_TICKS) {
        SetSafetyState(ROBOT_SAFETY_ARMED);
    }
}

static float MapStick(int16_t stick, float max_value)
{
    float value = (float)stick;

    if (fabsf(value) < CHASSIS_RC_DEADBAND) {
        return 0.0f;
    }
    return value * max_value / RC_STICK_FULL_SCALE;
}

static void SetSafeChassisCommand(void)
{
    chassis_cmd.vx_m_s = 0.0f;
    chassis_cmd.vy_m_s = 0.0f;
    chassis_cmd.wz_rad_s = 0.0f;
    chassis_cmd.chassis_mode = CHASSIS_DISABLED;
    chassis_cmd.robot_enabled = 0U;
}

static void BuildManualChassisCommand(void)
{
    /* Right stick: translation; side dial: yaw. */
    chassis_cmd.vx_m_s = MapStick(remote_control[TEMP].rc.rocker_r1,
        CHASSIS_RC_MAX_SPEED_M_S);
    chassis_cmd.vy_m_s = MapStick(remote_control[TEMP].rc.rocker_r_,
        CHASSIS_RC_MAX_SPEED_M_S);
    chassis_cmd.wz_rad_s = MapStick(remote_control[TEMP].rc.dial,
        CHASSIS_RC_MAX_YAW_RATE_RAD_S);
    chassis_cmd.chassis_mode = CHASSIS_MANUAL_DRIVE;
    chassis_cmd.robot_enabled = 1U;
}

void RobotCMDInit(void)
{
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    remote_control = RemoteControlInit(&huart3);
    SetSafeChassisCommand();
    LOGWARNING("[safety] booted locked");
}

void RobotCMDTask(void)
{
    if (remote_control[TEMP].rc_update_flag) {
        remote_frame_seen = 1U;
        remote_control[TEMP].rc_update_flag = 0U;
    }

    UpdateSafetyState();
    SetSafeChassisCommand();
    if (safety_state == ROBOT_SAFETY_ARMED) {
        BuildManualChassisCommand();
    }
    PubPushMessage(chassis_cmd_pub, &chassis_cmd);
}

Robot_Safety_State_e RobotCMDGetSafetyState(void)
{
    return safety_state;
}
