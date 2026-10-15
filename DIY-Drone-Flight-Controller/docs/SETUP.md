# Setup and controls

## 1. Install the Arduino dependencies

Install the ESP32 board package in Arduino IDE and select the board matching the ESP32-C3 hardware.

From Arduino Library Manager, install the libraries providing:

- RF24 and nRF24L01.h
- ESP32Servo
- SparkFun_u-blox_GNSS_Arduino_Library.h (use the library exposing this exact header)
- Adafruit_QMC5883P.h and its required dependencies

Arduino.h, Wire.h, and SPI.h come with the relevant board core. Exact working dependency versions were not supplied, so no tested version pins are claimed.

In Arduino IDE, open Preferences and find **Sketchbook location**. Copy the four subfolders from this repository's `libraries/` into that sketchbook's `libraries/` folder. Restart Arduino IDE.

Keep all four libraries installed together: some depend on each other.

## 2. Open the sketches

- Drone: `firmware/DRONE_V2.0_CABLE_GPS_RTH/DRONE_V2.0_CABLE_GPS_RTH.ino`
- Nano: `transmitter/transmitter_pocket/transmitter_pocket.ino`

Each sketch is inside a matching folder so Arduino IDE can open it normally.

## 3. Run the PC bridge

Connect the RadioMaster Pocket as a USB joystick and connect the Nano to the laptop.

Install the Python dependencies:

```powershell
python -m pip install -r ground-control/requirements.txt
```

Edit `ground-control/pocket_controller.py`: replace `COM4` with the Nano's actual serial port. The script uses the first joystick (index 0) and sends nine channels at approximately 50 Hz over 115200 baud serial.

```powershell
python ground-control/pocket_controller.py
```

The Python script's original filename was `9 channel pocket cable.py`. Its contents have been retained under a simpler filename. Verify joystick axes and switches for your Pocket configuration.

## 4. Switches

| Switch | Function |
| --- | --- |
| SA | Arm/disarm; source checks btn2 = 1 to send motor commands and -1 to disarm |
| SC | Mode selection: -1 manual, 0 altitude hold, 1 altitude + position hold |
| SD | Capture hover throttle while held |
| SB | Magnetometer calibration when backward / btn3 = -1 for about 2 seconds |
| SE | Save current North/East position and altitude setpoints while pressed |

The project description called the mode switch “CS”; the source comments label it SC. Verify the actual physical mapping before use.

### Hover-throttle calibration

In manual mode, hold SD, establish a steady hover, keep the throttle at the hovering position, then release SD. The builder described waiting about 2 seconds; the supplied source starts updating `hover_throttle` after **1000 ms** and continues updating while SD remains active. Releasing SD retains the last captured value.

### Magnetometer calibration

With motors disarmed, hold SB backward for about 2 seconds, rotate the drone through multiple orientations, then return SB to exit calibration. The source updates offsets in RAM. It does not write the new offsets to persistent storage; startup offsets are hard-coded.

### Position and altitude setpoints

Press SE at the desired position and altitude to capture setpoints. Select altitude hold or position hold with SC when ready. The startup GPS home reference used by RTH is separate from the setpoint saved with SE.

### GPS home and RTH

The firmware tries GPS serial at 115200 baud, then 9600 before switching to 115200. It requests UBX Auto-PVT at 10 Hz and sets its initial home reference after about 6 seconds following the first accepted fix.

In this source version, loss of radio packets for more than 700 ms triggers the experimental RTH sequence: target at least 10 m altitude (or preserve the current altitude if higher), return toward home, descend, and hold near 3 m. It is not an automatic landing implementation. Reconnection resets the failsafe sequence.

## Source-specific notes

- Startup IMU calibration requires keeping the drone still.
- Position hold and RTH need further tuning and depend on usable GPS and heading data.
- The Nano source transmits only when SB / btn3 is -1 or 0. Setting it to 1 stops transmissions and can trigger the drone's failsafe.
- These instructions document the supplied build; the firmware was not compiled or hardware-tested during packaging.
- Remove propellers for bench checks of wiring, motor outputs, arming, and switch mappings.

