# Custom ESP32 Quadcopter Flight Controller

A custom-built quadcopter featuring an ESP32-based flight controller, cascaded PID control, sensor fusion, GPS-assisted navigation, custom PCB design, and a 3D-printed airframe.

## Overview

This project is a personally designed and developed quadcopter, built around a custom flight controller rather than a commercial flight-control stack.

The project combines embedded C++, control systems, sensor processing, wireless communication, Python, PCB design, and 3D CAD. The aircraft has been flown successfully in manual mode, while altitude and GPS-based navigation features are being developed and tuned.

## Key Features

- **Manual flight control:** Full pilot control of roll, pitch, yaw, and throttle.
- **Cascaded PID stabilization:** Angle and angular-rate control loops for attitude stabilization.
- **Altitude hold:** Barometric altitude measurement, vertical velocity estimation, and throttle compensation.
- **GPS position hold:** Cascaded position and velocity controllers for horizontal navigation.
- **Return to home:** An automatic recovery sequence triggered by loss of the radio link.
- **Sensor fusion:** IMU, magnetometer, GPS, and barometer integration.
- **Custom sensor processing:** Mahony-based attitude estimation, filtering, and tilt-compensated compass heading.
- **Custom hardware:** Flight-controller PCB, 3D-modeled and printed frame, and integrated electronics.
- **Custom radio-control pipeline:** Python-based transmitter input processing, USB serial communication, an Arduino Nano receiver, and an nRF24L01 radio link.

## Hardware

### Onboard electronics
- ESP32-C3 SuperMini flight controller
- MPU6050 IMU
- NEO-M8N GPS module
- Magnetometer
- Barometric pressure sensor
- nRF24L01 wireless transceiver
- Four brushless motors and ESCs
- Custom PCB and 3D-printed frame

### Ground-control system
- RadioMaster Pocket transmitter
- Laptop running a Python controller script in VS Code
- USB serial connection to an Arduino Nano
- nRF24L01 transmitter connected to the Nano

## System Architecture

The radio-control system uses the following communication chain:

RadioMaster Pocket → Python controller → USB serial → Arduino Nano → nRF24L01 → ESP32 flight controller

The Python program reads the transmitter inputs and forwards the control data over serial to the Arduino Nano. The Nano transmits these inputs wirelessly to the onboard ESP32, which processes the commands and generates the motor outputs.

The onboard flight controller independently handles sensor readings, attitude stabilization, navigation calculations, and motor mixing.

## Flight-Control Architecture

### Attitude stabilization

Roll and pitch use cascaded PID controllers, where the outer angle loop generates a desired angular rate and the inner rate loop calculates corrective motor commands. Yaw control uses compass heading and angular-rate feedback.

### Altitude control

The controller combines barometric relative altitude with GPS altitude and estimates vertical velocity. An outer altitude loop and an inner vertical-velocity loop calculate throttle corrections.

Tilt compensation is applied to the requested throttle during altitude-hold operation.

### GPS navigation

Horizontal position control uses local North/East coordinates derived from GPS latitude and longitude. Cascaded position and velocity controllers calculate corrections that are converted into body-relative commands using the compass heading.

### Motor mixing

The controller combines throttle and roll, pitch, and yaw corrections using an X-configuration motor mixer to generate commands for the four ESCs.

The main control loop targets approximately 420 Hz, with GPS navigation updates configured for 10 Hz.

## Flight Modes and Controls

The transmitter switches are configured to control the following functions.

| Control | Function |
|---|---|
| SC / mode-selection switch | Selects manual, altitude-hold, or altitude-and-position-hold operation |
| SA | Arm/disarm control |
| SD | Hover-throttle calibration |
| SB | Magnetometer calibration |
| SE | Save the current GPS position as a position setpoint |

The exact switch behavior depends on the current transmitter and receiver configuration.

### Hover-throttle calibration

1. Enter the appropriate operating mode.
2. Press and hold SD for approximately two seconds.
3. Adjust the throttle until the aircraft is hovering.
4. Keep the throttle at the hovering value and release SD.

The captured throttle value is used as the hover-throttle reference for altitude-hold control.

### Magnetometer calibration

1. Toggle SB into the calibration position.
2. Hold it there for approximately two to three seconds to enter calibration.
3. Rotate the aircraft through different orientations so the magnetometer can collect readings across its axes.
4. Return the switch to its normal position when finished.

### Position setpoint

Press SE to save the current position as the navigation setpoint. Activate position hold to attempt to maintain that saved position.

### GPS setup

GPS-assisted modes require an adequate GPS fix and correct initialization of the home position. GPS acquisition and sensor initialization must be verified before relying on navigation features.

## Software and Dependencies

The project uses embedded C++ for the flight controller and receiver, alongside Python for ground-side transmitter input processing.

The repository includes four custom libraries developed for the project. Other dependencies are available through the Arduino IDE Library Manager or their respective upstream repositories.

The repository will document the required board configuration, library dependencies, wiring, and build instructions.

## Current Development Status

| Feature | Status |
|---|---|
| Manual flight | Working; full pilot control |
| Attitude stabilization | Implemented and flight-tested |
| Altitude hold | Implemented; requires appropriate hover-throttle calibration and tuning |
| GPS position hold | Functional in development; further tuning required |
| Return to home | Implemented in development; behavior and reliability require further validation |
| Custom PCB and frame | Built and integrated |

In relatively calm conditions with a stable GPS fix, the aircraft's current position-hold performance is approximately within a 3–4 m area. This is an observed result rather than a guaranteed accuracy specification.

Position hold and return-to-home remain ongoing development areas. Wind, GPS quality, sensor calibration, and controller tuning affect their performance.

## Safety and Limitations

This is an experimental flight-control system and is not a certified flight controller.

GPS-based recovery and position-hold functions should not be relied upon as guaranteed safety systems. Flight tests should be conducted in an appropriate open area, with propeller-off bench testing, motor-output checks, and independent verification of arming and failsafe behavior before flight.

## Future Improvements

- Further tune GPS position and velocity controllers.
- Improve position-hold accuracy and robustness in wind.
- Validate and refine return-to-home behavior.
- Improve source-code organization and configuration management.
- Expand flight-test documentation and performance measurements.

## Project Motivation

This project was developed to gain practical experience in embedded systems, control theory, sensor fusion, real-time programming, wireless communication, and hardware development by designing and building a complete working quadcopter.
