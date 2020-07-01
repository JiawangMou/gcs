#include <stdint.h>
#include "pid.h"
#include "attitude_pid.h"

/*�ǶȻ������޷�*/
#define PID_ANGLE_ROLL_INTEGRATION_LIMIT    30.0
#define PID_ANGLE_PITCH_INTEGRATION_LIMIT   30.0
#define PID_ANGLE_YAW_INTEGRATION_LIMIT     180.0

/*���ٶȻ������޷�*/
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
	pidInit(&pidAngleRoll, 0, angleRoll, anglePidDt);			/*roll  �Ƕ�PID��ʼ��*/
	pidInit(&pidAnglePitch, 0, anglePitch, anglePidDt);			/*pitch �Ƕ�PID��ʼ��*/
	pidInit(&pidAngleYaw, 0, angleYaw, anglePidDt);				/*yaw   �Ƕ�PID��ʼ��*/
	pidSetIntegralLimit(&pidAngleRoll, PID_ANGLE_ROLL_INTEGRATION_LIMIT);		/*roll  �ǶȻ����޷�����*/
	pidSetIntegralLimit(&pidAnglePitch, PID_ANGLE_PITCH_INTEGRATION_LIMIT);		/*pitch �ǶȻ����޷�����*/
	pidSetIntegralLimit(&pidAngleYaw, PID_ANGLE_YAW_INTEGRATION_LIMIT);			/*yaw   �ǶȻ����޷�����*/
	
	pidInit(&pidRateRoll, 0, rateRoll, ratePidDt);				/*roll  ���ٶ�PID��ʼ��*/
	pidInit(&pidRatePitch, 0, ratePitch, ratePidDt);			/*pitch ���ٶ�PID��ʼ��*/
	pidInit(&pidRateYaw, 0, rateYaw, ratePidDt);				/*yaw   ���ٶ�PID��ʼ��*/
	pidSetIntegralLimit(&pidRateRoll, PID_RATE_ROLL_INTEGRATION_LIMIT);			/*roll  ���ٶȻ����޷�����*/
	pidSetIntegralLimit(&pidRatePitch, PID_RATE_PITCH_INTEGRATION_LIMIT);		/*pitch ���ٶȻ����޷�����*/
	pidSetIntegralLimit(&pidRateYaw, PID_RATE_YAW_INTEGRATION_LIMIT);			/*yaw   ���ٶȻ����޷�����*/
}

void attitudeRatePID(attitude_t *actualRate,attitude_t *desiredRate,control_t *output)	/* ���ٶȻ�PID */
{
	output->roll = pidOutLimit(pidUpdate(&pidRateRoll, desiredRate->roll - actualRate->roll));
	output->pitch = pidOutLimit(pidUpdate(&pidRatePitch, desiredRate->pitch - actualRate->pitch));
	output->yaw = pidOutLimit(pidUpdate(&pidRateYaw, desiredRate->yaw - actualRate->yaw));
}

void attitudeAnglePID(attitude_t *actualAngle,attitude_t *desiredAngle,attitude_t *outDesiredRate)	/* �ǶȻ�PID */
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

void attitudeResetAllPID(void)	/*��λPID*/
{
	pidReset(&pidAngleRoll);
	pidReset(&pidAnglePitch);
	pidReset(&pidAngleYaw);
	pidReset(&pidRateRoll);
	pidReset(&pidRatePitch);
	pidReset(&pidRateYaw);
}
