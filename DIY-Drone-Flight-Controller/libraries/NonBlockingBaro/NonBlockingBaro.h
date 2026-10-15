#ifndef NON_BLOCKING_BARO_H
#define NON_BLOCKING_BARO_H

#include <Arduino.h>
#include <Wire.h>

class NonBlockingBaro {
  public:
    NonBlockingBaro(uint8_t address = 0x77);
    
    // Initializes I2C, reads calibration data, and sets the ground zero altitude
    void begin(int sda_pin, int scl_pin);
    
    // Must be called every loop. Handles the non-blocking state machine.
    void update();
    
    // Returns the altitude in meters relative to where it was powered on
    float getRelativeAltitude();

  private:
    uint8_t _address;
    uint16_t C[7];
    uint32_t D1, D2;
    
    float currentAltitude;
    float groundAltitude;
    
    uint32_t lastBaroTime;
    uint8_t baroState;

    uint32_t readADC();
    void calculateAltitude();
    void calibrateGroundLevel();
};

#endif
