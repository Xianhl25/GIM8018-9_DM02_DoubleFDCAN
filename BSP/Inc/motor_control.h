#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "stm32h7xx_hal.h"
#include "stdbool.h"

void Can1MotorControl_Start(void);
void Can2MotorControl_Start(void);
void Can1MotorControl_Stop(void);
void Can2MotorControl_Stop(void);

#endif