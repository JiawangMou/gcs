#ifndef __PID_TYPES_H
#define __PID_TYPES_H
#include <stdint.h>

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * �ṹ�����Ͷ���	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

typedef struct  
{
	uint32_t timestamp;	/*ʱ���*/

	float roll;
	float pitch;
	float yaw;
} attitude_t;


typedef struct
{
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
	float thrust;
} control_t;

typedef struct 
{
	float kp;
	float ki;
	float kd;
} pidInit_t;

typedef struct
{
	pidInit_t roll;
	pidInit_t pitch;	
	pidInit_t yaw;	
} pidParam_t;

typedef struct
{
	pidInit_t vx;
	pidInit_t vy;
	pidInit_t vz;
	
	pidInit_t x;
	pidInit_t y;
	pidInit_t z;
} pidParamPos_t;


struct vec3_s
{
	float x;
	float y;
	float z;
};

typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;

typedef struct
{
	attitude_t attitude;		// deg	
	attitude_t attitudeRate;	// deg/s
	point_t position;         	// m
	velocity_t velocity;      	// m/s
	float thrust;
} setpoint_t;

typedef struct
{
	attitude_t attitude;
	point_t position;
	velocity_t velocity;
} state_t;

#define RATE_5_HZ		5
#define RATE_10_HZ		10
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000

#define MAIN_LOOP_RATE 	RATE_1000_HZ
#define MAIN_LOOP_DT	(u32)(1000/MAIN_LOOP_RATE)	/*��λms*/

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (MAIN_LOOP_RATE / RATE_HZ)) == 0)

#endif


