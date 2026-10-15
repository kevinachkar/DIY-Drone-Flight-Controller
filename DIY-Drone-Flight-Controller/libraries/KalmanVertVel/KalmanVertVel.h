#ifndef KALMAN_VERT_VEL_H
#define KALMAN_VERT_VEL_H

#include <Arduino.h>
#include "MPU6050_Mahony_DynamicNotch.h"
#include "NonBlockingBaro.h"

class KalmanVertVel {
public:
    // Constructor takes references to your existing sensor objects
    KalmanVertVel(MPU6050_Mahony& imu, NonBlockingBaro& baro);

    // Call once in setup()
    void begin();

    // Call this every loop iteration. It automatically updates the sensors and runs the math.
    void update();

    // Returns ONLY the filtered vertical velocity (m/s)
    float getVelocity();

    // Change the LPF smoothing value on the fly (0.0 to 1.0)
    void setLPF(float alpha);

private:
    MPU6050_Mahony& _imu;
    NonBlockingBaro& _baro;

    // Kalman State
    float k_alt;
    float k_vel;

    // Covariance Matrix
    float P00, P01, P10, P11;

    // Tuning Parameters
    float Q_alt;
    float Q_vel;
    float R_baro;

    // LPF Variables
    float filtered_k_vel;
    float vel_lpf_alpha;

    // Timing
    unsigned long lastLoopTime;
    float last_baro_alt;
};

#endif
