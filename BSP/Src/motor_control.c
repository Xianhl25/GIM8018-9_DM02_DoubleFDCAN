#include "motor_control.h"
#include "can_comm.h"
#include "math.h"

/* 在发送电机位置零前，需要把电机的所有控制参数设置为0 */
static void Can1MotorZeroPosition(void)
{
    Can1Comm_ControlCmd(CMD_MOTOR_MODE);
    HAL_Delay(100);
    Can1Comm_SendControlPara(0,0,0,0,0);
    HAL_Delay(100);
}

static void Can2MotorZeroPosition(void)
{
    Can2Comm_ControlCmd(CMD_MOTOR_MODE);
    HAL_Delay(100);
    Can2Comm_SendControlPara(0,0,0,0,0);
    HAL_Delay(100);
}

/* 启动电机控制 */
void Can1MotorControl_Start(void)
{
    Can1MotorZeroPosition();
    Can1Comm_ControlCmd(CMD_ZERO_POSITION);
}

void Can2MotorControl_Start(void)
{
    Can2MotorZeroPosition();
    Can2Comm_ControlCmd(CMD_ZERO_POSITION);
}

/* 停止电机控制 */
void Can1MotorControl_Stop(void)
{
    Can1Comm_ControlCmd(CMD_RESET_MODE);
}

void Can2MotorControl_Stop(void)
{
    Can2Comm_ControlCmd(CMD_RESET_MODE);
}



