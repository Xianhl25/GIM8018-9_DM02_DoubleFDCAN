#ifndef DMMOTOR_H
#define DMMOTOR_H

#include "fdcan.h"

#define MIT_MODE 		0x000
#define POS_MODE		0x100
#define SPEED_MODE		0x200

//以下是DM4310的参数，用其他电机需要更改下面参数
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

typedef struct 
{
	uint16_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;

	float pos;
	float vel;
	float tor;

	float Tmos;
	float Tcoil;
}motor_fbpara_t;

typedef struct
{
	uint16_t mode;
	motor_fbpara_t para;
}Joint_Motor_t ;

void joint_motor_init(Joint_Motor_t *motor,uint16_t id,uint16_t mode);
void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);

#endif
