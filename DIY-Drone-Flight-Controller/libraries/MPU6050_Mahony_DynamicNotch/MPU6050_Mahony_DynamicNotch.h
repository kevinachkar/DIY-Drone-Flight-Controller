#ifndef MPU6050_MAHONY_DYNAMICNOTCH_H
#define MPU6050_MAHONY_DYNAMICNOTCH_H

#include <Arduino.h>
#include <Wire.h>

class MPU6050_Mahony {
public:
    MPU6050_Mahony(int sda, int scl);

    void begin();
    void calibrate();
    void update();

    float getRoll();
    float getPitch();

    float getAccelX();
    float getAccelY();
    float getAccelZ();

    void getEarthAccel(float &ax_earth, float &ay_earth, float &az_earth);

    float getQ0();
    float getQ1();
    float getQ2();
    float getQ3();

    float getGyroX(); // RAW for PID
    float getGyroY();
    float getGyroZ();

    // ===================== CONTROL =====================
    bool enableDynamicNotch = false; // <<< TOGGLE THIS IN MAIN CODE

private:
    int _sda, _scl;

    // Raw data
    int16_t accX, accY, accZ;
    int16_t gyroX, gyroY, gyroZ;

    // Converted accel
    float ax, ay, az;

    // Raw gyro (deg/s)
    float gyroX_raw, gyroY_raw, gyroZ_raw;

    // Mahony gyro (rad/s)
    float gx, gy, gz;

    // Offsets
    float gx_offset = 0, gy_offset = 0, gz_offset = 0;

    // Quaternion
    float q0 = 1, q1 = 0, q2 = 0, q3 = 0;

    // Integral feedback
    float integralFBx = 0, integralFBy = 0, integralFBz = 0;

    // Gains
    float Kp = 2.0f;
    float Ki = 0.005f;

    // Timing
    unsigned long prevTime;
    float dt;

    // ===================== DYNAMIC NOTCH =====================
    float notchFreq = 20.0f;
    float notchAlpha = 0.02f;
    float notchQ = 8.0f;

    float nx_x = 0, nx_x2 = 0;
    float nx_y = 0, nx_y2 = 0;
    float nx_z = 0, nx_z2 = 0;

    float applyNotch(float input, float &x1, float &x2);
    float updateNotch(float gyroMag);

    void writeMPU(uint8_t reg, uint8_t data);
    void readMPU();
    void updateMahony();
};

#endif
