#include "NonBlockingBaro.h"

NonBlockingBaro::NonBlockingBaro(uint8_t address) {
  _address = address;
  currentAltitude = 0.0f;
  groundAltitude = 0.0f;
  lastBaroTime = 0;
  baroState = 0;
}

void NonBlockingBaro::begin(int sda_pin, int scl_pin) {
  Wire.begin(sda_pin, scl_pin);
  Wire.setClock(400000); // 400kHz Fast Mode

  // Reset MS5611
  Wire.beginTransmission(_address);
  Wire.write(0x1E);
  Wire.endTransmission();
  delay(10); // Safe to use delay here as it's only in setup()

  // Read Factory Calibration PROM
  for (uint8_t i = 1; i <= 6; i++) {
    Wire.beginTransmission(_address);
    Wire.write(0xA0 + (i * 2));
    Wire.endTransmission();
    Wire.requestFrom((int)_address, 2);
    C[i] = (Wire.read() << 8) | Wire.read();
  }

  // Establish Zero Altitude
  calibrateGroundLevel();

  // Trigger the very first background conversion
  Wire.beginTransmission(_address);
  Wire.write(0x48); // Pressure OSR 4096
  Wire.endTransmission();
  lastBaroTime = micros();
}

void NonBlockingBaro::update() {
  uint32_t currentMicros = micros();

  // 9.5ms asynchronous interval for OSR 4096 conversions
  if (currentMicros - lastBaroTime >= 9500) {
    if (baroState == 0) {
      D1 = readADC(); // Store Pressure
      
      Wire.beginTransmission(_address);
      Wire.write(0x58); // Request Temperature OSR 4096
      Wire.endTransmission();
      
      baroState = 1;
    } else {
      D2 = readADC(); // Store Temperature
      
      calculateAltitude(); // Crunch the math
      
      Wire.beginTransmission(_address);
      Wire.write(0x48); // Request Pressure OSR 4096 for next cycle
      Wire.endTransmission();
      
      baroState = 0;
    }
    lastBaroTime = currentMicros;
  }
}

float NonBlockingBaro::getRelativeAltitude() {
  return currentAltitude - groundAltitude;
}

// --- PRIVATE HELPER FUNCTIONS --- //

uint32_t NonBlockingBaro::readADC() {
  Wire.beginTransmission(_address);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom((int)_address, 3);
  return ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
}

void NonBlockingBaro::calculateAltitude() {
  int32_t dT = D2 - ((uint32_t)C[5] << 8);
  
  // 64-bit int for math to prevent overflow
  int64_t OFF = ((int64_t)C[2] << 16) + (((int64_t)C[4] * dT) >> 7);
  int64_t SENS = ((int64_t)C[1] << 15) + (((int64_t)C[3] * dT) >> 8);

  float pressure_hPa = ((((D1 * SENS) >> 21) - OFF) >> 15) / 100.0f;

  // 32-bit float math for speed
  currentAltitude = 44330.0f * (1.0f - pow(pressure_hPa / 1013.25f, 0.1903f));
}

void NonBlockingBaro::calibrateGroundLevel() {
  float altSum = 0;
  int samples = 20;

  // Take 20 blocking reads during setup to establish a solid baseline
  for (int i = 0; i < samples; i++) {
    // Force a pressure read
    Wire.beginTransmission(_address);
    Wire.write(0x48);
    Wire.endTransmission();
    delay(10);
    D1 = readADC();

    // Force a temperature read
    Wire.beginTransmission(_address);
    Wire.write(0x58);
    Wire.endTransmission();
    delay(10);
    D2 = readADC();

    calculateAltitude();
    altSum += currentAltitude;
  }
  
  groundAltitude = altSum / (float)samples;
}
