#ifndef _MADGWICK_H_
#define _MADGWICK_H_

#include <stdint.h>
#include <math.h>

namespace madgwick{

    #define Kp 2.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
    #define Ki 0.01f   // integral gain governs rate of convergence of gyroscope biases

    #define BIQUAD_Q 1.0f / sqrtf(2.0f)     /* quality factor - 2nd order butterworth*/

    extern float gx_bias;
    extern float gy_bias;
    extern float gz_bias;

    typedef struct
    {
        float q0;	//q0-q3四元数
        float q1;
        float q2;
        float q3;
        float ex_inte;
        float ey_inte;
        float ez_inte;
        unsigned int last_update;

    } OrientationEstimator;

    typedef struct biquadFilter_s {
        float b0, b1, b2, a1, a2;
        float x1, x2, y1, y2;
    } biquadFilter_t;

    extern biquadFilter_t accFilter[3];
    extern biquadFilter_t gyroFilter[3];

    extern OrientationEstimator estimator;

    void IMU_init(OrientationEstimator* estimator);
    void IMU_update(OrientationEstimator* estimator, float gx, float gy, float gz,
                                                       float ax, float ay, float az,
                                                       float mx, float my, float mz,
                                                       unsigned int now);

    float biquadFilterApply(biquadFilter_t *filter, float input);


}

#endif