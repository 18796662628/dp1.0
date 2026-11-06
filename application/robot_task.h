/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "daemon.h"
#include "robot_cmd.h"
#include "chassis.h"
#include "motor_task.h"

//! 任务直接在cubeMX中配置,不再使用这种方式
__attribute__((noreturn)) void _RobotCMDTask(void *argument)
{
    UNUSED(argument);
    for (;;) {
        RobotCMDTask();
        osDelay(1);
    }
}
__attribute__((noreturn)) void _ChassisTask(void *argument)
{
    UNUSED(argument);
    for (;;) {
        ChassisTask();
        osDelay(1);
    }
}
__attribute__((noreturn)) void motorControlTask(void *argument)
{
    UNUSED(argument);
    for (;;) {
        MotorControlTask();
        osDelay(1);
    }
}

__attribute__((noreturn)) void _DaemonTask(void *argument)
{
    UNUSED(argument);
    for (;;) {
        DaemonTask();
        osDelay(1);
    }
}
