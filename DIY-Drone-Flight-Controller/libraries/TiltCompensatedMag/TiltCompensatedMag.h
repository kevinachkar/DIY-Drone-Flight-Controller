#ifndef TiltCompensatedMag_h
#define TiltCompensatedMag_h

#include <Arduino.h>
#include <Adafruit_QMC5883P.h>
#include "MPU6050_Mahony_DynamicNotch.h"

class TiltCompensatedMag {
public:
  TiltCompensatedMag(uint8_t sdaPin = 10, uint8_t sclPin = 9);

  bool begin();

  // call at 600Hz
  void update();

  // calibration
  void setMagOffsets(float x, float y, float z);
  void setHeadingOffset(float offsetDeg);

  float getHeading();

  void getRawMagData(int16_t *x, int16_t *y, int16_t *z);

private:
  uint8_t _sda, _scl;

  // sensors (NO heap)
  Adafruit_QMC5883P mag;
  MPU6050_Mahony imu;

  // calibration
  float xOffset, yOffset, zOffset;
  float headingOffset;

  // outputs
  float heading;

  // cached IMU values
  float rollRad;
  float pitchRad;

  // cached mag values
  float X_body, Y_body, Z_body;

  // timing control
  uint32_t lastMagUpdate;
  const uint16_t magIntervalUs = 20000; // 50 Hz

  void updateIMU();
  void updateMag();
  void computeHeading();
};

#endif
