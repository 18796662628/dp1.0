#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

#include "robot_def.h"

void RobotCMDInit(void);
void RobotCMDTask(void);
Robot_Safety_State_e RobotCMDGetSafetyState(void);

#endif
