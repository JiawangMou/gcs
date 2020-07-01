#ifndef __ATTITUDE_PID_H
#define __ATTITUDE_PID_H
#include <stdbool.h>
#include "pid.h"

extern PidObject pidAngleRoll;
extern PidObject pidAnglePitch;
extern PidObject pidAngleYaw;

extern PidObject pidRateRoll;
extern PidObject pidRatePitch;
extern PidObject pidRateYaw;

void attitudeControlInit(pidInit_t angleRoll, pidInit_t anglePitch, pidInit_t angleYaw,
						 pidInit_t rateRoll, pidInit_t ratePitch, pidInit_t rateYaw,
						 float ratePidDt, float anglePidDt);

void attitudeRatePID(attitude_t *actualRate,attitude_t *desiredRate,control_t *output);
void attitudeAnglePID(attitude_t *actualAngle,attitude_t *desiredAngle,attitude_t *outDesiredRate);
void attitudeControllerResetRollAttitudePID(void);
void attitudeControllerResetPitchAttitudePID(void);
void attitudeResetAllPID(void);
void attitudePIDwriteToConfigParam(void);

#endif /* __ATTITUDE_PID_H */
