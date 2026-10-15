#include "MPU6050_Mahony_DynamicNotch.h"

#define MPU6050_ADDR 0x68

MPU6050_Mahony::MPU6050_Mahony(int sda, int scl) {
    _sda = sda;
    _scl = scl;
}

void MPU6050_Mahony::begin() {
    Wire.begin(_sda, _scl);
    Wire.setClock(400000);

    writeMPU(0x6B, 0x00);
    writeMPU(0x1B, 0x08);
    writeMPU(0x1C, 0x08);
    writeMPU(0x1A, 0x03);

    delay(500);
    prevTime = micros();
}

void MPU6050_Mahony::calibrate() {
    int samples = 2000;

    for (int i = 0; i < samples; i++) {
        readMPU();
        gx_offset += gyroX;
        gy_offset += gyroY;
        gz_offset += gyroZ;
        delay(2);
    }

    gx_offset /= samples;
    gy_offset /= samples;
    gz_offset /= samples;
}

void MPU6050_Mahony::writeMPU(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

void MPU6050_Mahony::readMPU() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14);

    accX = Wire.read() << 8 | Wire.read();
    accY = Wire.read() << 8 | Wire.read();
    accZ = Wire.read() << 8 | Wire.read();

    Wire.read(); Wire.read();

    gyroX = Wire.read() << 8 | Wire.read();
    gyroY = Wire.read() << 8 | Wire.read();
    gyroZ = Wire.read() << 8 | Wire.read();
}

void MPU6050_Mahony::update() {
    unsigned long now = micros();
    dt = (now - prevTime) / 1000000.0f;
    prevTime = now;

    readMPU();

    ax = accX / 8192.0f;
    ay = accY / 8192.0f;
    az = accZ / 8192.0f;

    gyroX_raw = (gyroX - gx_offset) / 65.5f;
    gyroY_raw = (gyroY - gy_offset) / 65.5f;
    gyroZ_raw = (gyroZ - gz_offset) / 65.5f;

    // ===================== DYNAMIC NOTCH (OPTIONAL) =====================
    if (enableDynamicNotch) {

        float gyroMag = sqrt(
            gyroX_raw * gyroX_raw +
            gyroY_raw * gyroY_raw +
            gyroZ_raw * gyroZ_raw
        );

        updateNotch(gyroMag);

        gyroX_raw = applyNotch(gyroX_raw, nx_x, nx_x2);
        gyroY_raw = applyNotch(gyroY_raw, nx_y, nx_y2);
        gyroZ_raw = applyNotch(gyroZ_raw, nx_z, nx_z2);
    }

    gx = gyroX_raw * DEG_TO_RAD;
    gy = gyroY_raw * DEG_TO_RAD;
    gz = gyroZ_raw * DEG_TO_RAD;

    updateMahony();
}

// ===================== NOTCH =====================

float MPU6050_Mahony::updateNotch(float gyroMag) {
    float estimatedFreq = gyroMag * 10.0f;
    estimatedFreq = constrain(estimatedFreq, 10.0f, 200.0f);

    notchFreq = notchFreq + notchAlpha * (estimatedFreq - notchFreq);
    return notchFreq;
}

float MPU6050_Mahony::applyNotch(float input, float &x1, float &x2) {

    float w0 = 2.0f * PI * notchFreq * dt;
    float cos_w0 = cos(w0);
    float sin_w0 = sin(w0);

    float alpha = sin_w0 / (2.0f * notchQ);

    float b0 = 1;
    float b1 = -2 * cos_w0;
    float b2 = 1;

    float a0 = 1 + alpha;
    float a1 = -2 * cos_w0;
    float a2 = 1 - alpha;

    float output =
        (b0/a0)*input +
        (b1/a0)*x1 +
        (b2/a0)*x2;

    output -= (a1/a0)*x1;
    output -= (a2/a0)*x2;

    x2 = x1;
    x1 = input;

    return output;
}

// ===================== MAHONY (UNCHANGED) =====================

void MPU6050_Mahony::updateMahony() {
    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez;

    recipNorm = sqrt(ax * ax + ay * ay + az * az);
    if (recipNorm == 0) return;
    recipNorm = 1.0f / recipNorm;

    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    integralFBx += Ki * ex * dt;
    integralFBy += Ki * ey * dt;
    integralFBz += Ki * ez * dt;

    gx += Kp * ex + integralFBx;
    gy += Kp * ey + integralFBy;
    gz += Kp * ez + integralFBz;

    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    float qa = q0;
    float qb = q1;
    float qc = q2;

    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    recipNorm = 1.0f / sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

// ===================== OUTPUT =====================

float MPU6050_Mahony::getRoll() {
    return atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * 180.0f / PI;
}

float MPU6050_Mahony::getPitch() {
    return asin(2*(q0*q2 - q3*q1)) * 180.0f / PI;
}

float MPU6050_Mahony::getGyroX() { return gyroX_raw; }
float MPU6050_Mahony::getGyroY() { return gyroY_raw; }
float MPU6050_Mahony::getGyroZ() { return gyroZ_raw; }

//accel for gps fusion

void MPU6050_Mahony::getEarthAccel(float &ax_earth, float &ay_earth, float &az_earth) {
    // Convert g's to m/s^2
    float ax_ms2 = ax * 9.81f;
    float ay_ms2 = ay * 9.81f;
    float az_ms2 = az * 9.81f;

    // Rotate body acceleration to Earth frame using quaternion conjugate
    ax_earth = ax_ms2 * (1 - 2*(q2*q2 + q3*q3)) + ay_ms2 * (2*(q1*q2 - q0*q3))     + az_ms2 * (2*(q1*q3 + q0*q2));
    ay_earth = ax_ms2 * (2*(q1*q2 + q0*q3))     + ay_ms2 * (1 - 2*(q1*q1 + q3*q3)) + az_ms2 * (2*(q2*q3 - q0*q1));
    az_earth = ax_ms2 * (2*(q1*q3 - q0*q2))     + ay_ms2 * (2*(q2*q3 + q0*q1))     + az_ms2 * (1 - 2*(q1*q1 + q2*q2));

    // Remove 1G gravity from the Z axis
    az_earth -= 9.81f; 
}

float MPU6050_Mahony::getAccelX() { return ax; }
float MPU6050_Mahony::getAccelY() { return ay; }
float MPU6050_Mahony::getAccelZ() { return az; }

float MPU6050_Mahony::getQ0(){return q0;}
float MPU6050_Mahony::getQ1(){return q1;}
float MPU6050_Mahony::getQ2(){return q2;}
float MPU6050_Mahony::getQ3(){return q3;}
