#include "TiltCompensatedMag.h"
#include <Wire.h>

TiltCompensatedMag::TiltCompensatedMag(uint8_t sdaPin, uint8_t sclPin)
: imu(sdaPin, sclPin)
{
  _sda = sdaPin;
  _scl = sclPin;

  xOffset = 0;
  yOffset = 0;
  zOffset = 0;
  headingOffset = 0;

  heading = 0;

  rollRad = 0;
  pitchRad = 0;

  X_body = 0;
  Y_body = 0;
  Z_body = 0;

  lastMagUpdate = 0;
}

bool TiltCompensatedMag::begin() {
  Wire.begin(_sda, _scl);
  
  // OPTIMIZATION 1: Force I2C to 400kHz (Fast Mode). 
  // This reduces the time it takes to read the magnetometer by ~75%, 
  // drastically reducing I2C blocking time on your 600Hz loop.
  Wire.setClock(400000); 

  imu.begin();
  imu.calibrate();

  if (!mag.begin()) {
    return false;
  }

  mag.setMode(QMC5883P_MODE_CONTINUOUS);

  return true;
}

void TiltCompensatedMag::setMagOffsets(float x, float y, float z) {
  xOffset = x;
  yOffset = y;
  zOffset = z;
}

void TiltCompensatedMag::setHeadingOffset(float offsetDeg) {
  headingOffset = offsetDeg;
}

void TiltCompensatedMag::update() {
  // ALWAYS update IMU at full rate (600Hz safe)
  updateIMU();

  // throttle magnetometer internally
  uint32_t now = micros();
  if (now - lastMagUpdate >= magIntervalUs) {
    updateMag();
    
    // OPTIMIZATION 2: Moved computeHeading() inside the 50Hz block.
    // Drones rely on the gyro for fast yaw tracking, and use the compass 
    // to correct long-term drift. Running heavy Trig math at 50Hz instead of 
    // 600Hz will free up massive amounts of CPU time and stop the D-term vibrations.
    computeHeading(); 
    
    lastMagUpdate = now;
  }
}

void TiltCompensatedMag::updateIMU() {
  imu.update();

  // cache angles ONCE per loop (important for speed stability)
  rollRad  = (-imu.getPitch()) * DEG_TO_RAD;
  pitchRad = (-imu.getRoll())  * DEG_TO_RAD;
}

void TiltCompensatedMag::updateMag() {
  int16_t x, y, z;
  mag.getRawMagnetic(&x, &y, &z); // This is the blocking call being sped up by 400kHz

  float X_raw = x - xOffset;
  float Y_raw = y - yOffset;
  float Z_raw = z - zOffset;

  // body alignment (your original mapping)
  X_body = -Y_raw;
  Y_body =  X_raw;
  Z_body =  Z_raw;
}

void TiltCompensatedMag::computeHeading() {
  float cosR = cos(rollRad);
  float sinR = sin(rollRad);
  float cosP = cos(pitchRad);
  float sinP = sin(pitchRad);

  float XH = X_body * cosP + Y_body * sinR * sinP + Z_body * cosR * sinP;
  float YH = Y_body * cosR - Z_body * sinR;

  float h = atan2(-YH, XH) * 180.0 / PI;

  if (h < 0) h += 360;
  if (h >= 360) h -= 360;

  h += headingOffset;

  if (h < 0) h += 360;
  if (h >= 360) h -= 360;

  heading = h;
}

float TiltCompensatedMag::getHeading() {
  return heading;
}

void TiltCompensatedMag::getRawMagData(int16_t *x, int16_t *y, int16_t *z) {
  mag.getRawMagnetic(x, y, z);
}
