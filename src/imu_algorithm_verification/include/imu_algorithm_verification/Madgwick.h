#ifndef _MADGWICK_H_
#define _MADGWICK_H_

namespace madgwick{

    #define Kp 2.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
    #define Ki 0.01f   // integral gain governs rate of convergence of gyroscope biases

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

    extern OrientationEstimator estimator;

    void IMU_init(OrientationEstimator* estimator);
    void IMU_update(OrientationEstimator* estimator, float gx, float gy, float gz,
                                                       float ax, float ay, float az,
                                                       float mx, float my, float mz,
                                                       unsigned int now);


}

#endif