#ifndef __POSITION_PID_H
#define __POSITION_PID_H
#include "pid_types.h"
#include "pid.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * λ��PID���ƴ���	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

extern PidObject pidVX;
extern PidObject pidVY;
extern PidObject pidVZ;

extern PidObject pidX;
extern PidObject pidY;
extern PidObject pidZ;


float constrainf(float amt, float low, float high);
void positionControlInit(pidInit_t vX, pidInit_t vY, pidInit_t vZ,
						 pidInit_t X, pidInit_t Y, pidInit_t Z,
						 float velocityPidDt, float posPidDt);
void positionResetAllPID(void);
void positionController(float* thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state);
float getAltholdThrust(void);
	
#endif /* __POSITION_PID_H */
