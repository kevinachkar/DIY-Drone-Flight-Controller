#include "KalmanVertVel.h"

KalmanVertVel::KalmanVertVel(MPU6050_Mahony& imu, NonBlockingBaro& baro) 
    : _imu(imu), _baro(baro) {
    
    // Initialize default state[cite: 5]
    k_alt = 0.0f;
    k_vel = 0.0f;

    P00 = 1.0f; P01 = 0.0f;
    P10 = 0.0f; P11 = 1.0f;

    // Default tuning values from your working script[cite: 5]
    Q_alt = 0.001f;
    Q_vel = 0.005f;
    R_baro = 0.4f;

    filtered_k_vel = 0.0f;
    vel_lpf_alpha = 0.30f; 

    lastLoopTime = 0;
    last_baro_alt = 0.0f;
}

void KalmanVertVel::begin() {
    lastLoopTime = micros();
}

void KalmanVertVel::setLPF(float alpha) {
    // Constrain safety limits to prevent math errors
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    vel_lpf_alpha = alpha;
}

void KalmanVertVel::update() {
    unsigned long now = micros();
    
    // Safety check for first loop
    if (lastLoopTime == 0) {
        lastLoopTime = now;
        return;
    }
    
    float dt = (now - lastLoopTime) / 1000000.0f;
    lastLoopTime = now;

    // 1. The library automatically updates your sensors for you
    _imu.update();
    _baro.update();

    // 2. Extract Earth-Frame Acceleration
    float ax_earth, ay_earth, az_earth;
    _imu.getEarthAccel(ax_earth, ay_earth, az_earth);

    // ==============================================================
    // KALMAN PREDICTION
    // ==============================================================
    k_alt += (k_vel * dt) + (0.5f * az_earth * dt * dt);
    k_vel += (az_earth * dt);

    P00 = P00 + (P10 * dt) + (P01 * dt) + (P11 * dt * dt) + Q_alt;
    P01 = P01 + (P11 * dt);
    P10 = P10 + (P11 * dt);
    P11 = P11 + Q_vel;

    // ==============================================================
    // KALMAN UPDATE
    // ==============================================================
    float current_baro_alt = _baro.getRelativeAltitude();

    if (current_baro_alt != last_baro_alt) {
        last_baro_alt = current_baro_alt;

        float y = current_baro_alt - k_alt;
        float S = P00 + R_baro;

        float K0 = P00 / S;
        float K1 = P10 / S;

        k_alt += K0 * y;
        k_vel += K1 * y;

        float P00_temp = P00;
        float P01_temp = P01;

        P00 = (1.0f - K0) * P00_temp;
        P01 = (1.0f - K0) * P01_temp;
        P10 = -(K1 * P00_temp) + P10;
        P11 = -(K1 * P01_temp) + P11;
    }

    // ==============================================================
    // LPF OUTPUT
    // ==============================================================
    filtered_k_vel = (vel_lpf_alpha * k_vel) + ((1.0f - vel_lpf_alpha) * filtered_k_vel);
}

float KalmanVertVel::getVelocity() {
    return filtered_k_vel;
}
