#include <stdint.h>
#include "pid.h"
#include "attitude_pid.h"

/*角度环积分限幅*/
#define PID_ANGLE_ROLL_INTEGRATION_LIMIT    30.0
#define PID_ANGLE_PITCH_INTEGRATION_LIMIT   30.0
#define PID_ANGLE_YAW_INTEGRATION_LIMIT     180.0

/*角速度环积分限幅*/
#define PID_RATE_ROLL_INTEGRATION_LIMIT		500.0
#define PID_RATE_PITCH_INTEGRATION_LIMIT	500.0
#define PID_RATE_YAW_INTEGRATION_LIMIT		50.0

PidObject pidAngleRoll;
PidObject pidAnglePitch;
PidObject pidAngleYaw;
PidObject pidRateRoll;
PidObject pidRatePitch;
PidObject pidRateYaw;

static inline int16_t pidOutLimit(float in)
{
	if (in > INT16_MAX)
		return INT16_MAX;
	else if (in < -INT16_MAX)
		return -INT16_MAX;
	else
		return (int16_t)in;
}

void attitudeControlInit(pidInit_t angleRoll, pidInit_t anglePitch, pidInit_t angleYaw,
						 pidInit_t rateRoll, pidInit_t ratePitch, pidInit_t rateYaw,
						 float ratePidDt, float anglePidDt)
{
	pidInit(&pidAngleRoll, 0, angleRoll, anglePidDt);			/*roll  角度PID初始化*/
	pidInit(&pidAnglePitch, 0, anglePitch, anglePidDt);			/*pitch 角度PID初始化*/
	pidInit(&pidAngleYaw, 0, angleYaw, anglePidDt);				/*yaw   角度PID初始化*/
	pidSetIntegralLimit(&pidAngleRoll, PID_ANGLE_ROLL_INTEGRATION_LIMIT);		/*roll  角度积分限幅设置*/
	pidSetIntegralLimit(&pidAnglePitch, PID_ANGLE_PITCH_INTEGRATION_LIMIT);		/*pitch 角度积分限幅设置*/
	pidSetIntegralLimit(&pidAngleYaw, PID_ANGLE_YAW_INTEGRATION_LIMIT);			/*yaw   角度积分限幅设置*/
	
	pidInit(&pidRateRoll, 0, rateRoll, ratePidDt);				/*roll  角速度PID初始化*/
	pidInit(&pidRatePitch, 0, ratePitch, ratePidDt);			/*pitch 角速度PID初始化*/
	pidInit(&pidRateYaw, 0, rateYaw, ratePidDt);				/*yaw   角速度PID初始化*/
	pidSetIntegralLimit(&pidRateRoll, PID_RATE_ROLL_INTEGRATION_LIMIT);			/*roll  角速度积分限幅设置*/
	pidSetIntegralLimit(&pidRatePitch, PID_RATE_PITCH_INTEGRATION_LIMIT);		/*pitch 角速度积分限幅设置*/
	pidSetIntegralLimit(&pidRateYaw, PID_RATE_YAW_INTEGRATION_LIMIT);			/*yaw   角速度积分限幅设置*/
}

void attitudeRatePID(attitude_t *actualRate,attitude_t *desiredRate,control_t *output)	/* 角速度环PID */
{
	output->roll = pidOutLimit(pidUpdate(&pidRateRoll, desiredRate->roll - actualRate->roll));
	output->pitch = pidOutLimit(pidUpdate(&pidRatePitch, desiredRate->pitch - actualRate->pitch));
	output->yaw = pidOutLimit(pidUpdate(&pidRateYaw, desiredRate->yaw - actualRate->yaw));
}

void attitudeAnglePID(attitude_t *actualAngle,attitude_t *desiredAngle,attitude_t *outDesiredRate)	/* 角度环PID */
{
	outDesiredRate->roll = pidUpdate(&pidAngleRoll, desiredAngle->roll - actualAngle->roll);
	outDesiredRate->pitch = pidUpdate(&pidAnglePitch, desiredAngle->pitch - actualAngle->pitch);

	float yawError = desiredAngle->yaw - actualAngle->yaw ;
	if (yawError > 180.0f) 
		yawError -= 360.0f;
	else if (yawError < -180.0) 
		yawError += 360.0f;
	outDesiredRate->yaw = pidUpdate(&pidAngleYaw, yawError);
}

void attitudeControllerResetRollAttitudePID(void)
{
    pidReset(&pidAngleRoll);
}

void attitudeControllerResetPitchAttitudePID(void)
{
    pidReset(&pidAnglePitch);
}

void attitudeResetAllPID(void)	/*复位PID*/
{
	pidReset(&pidAngleRoll);
	pidReset(&pidAnglePitch);
	pidReset(&pidAngleYaw);
	pidReset(&pidRateRoll);
	pidReset(&pidRatePitch);
	pidReset(&pidRateYaw);
}
